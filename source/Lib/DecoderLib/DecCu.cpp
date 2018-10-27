/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2018, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     DecCu.cpp
    \brief    CU decoder class
*/

#include "DecCu.h"

#include "CommonLib/CrossCompPrediction.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_buffer.h"

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#if K0149_BLOCK_STATISTICS
#include "CommonLib/ChromaFormat.h"
#include "CommonLib/dtrace_blockstatistics.h"
#endif

//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

DecCu::DecCu()
{
}

DecCu::~DecCu()
{
}

void DecCu::init( TrQuant* pcTrQuant, IntraPrediction* pcIntra, InterPrediction* pcInter)
{
  m_pcTrQuant       = pcTrQuant;
  m_pcIntraPred     = pcIntra;
  m_pcInterPred     = pcInter;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void DecCu::decompressCtu( CodingStructure& cs, const UnitArea& ctuArea )
{
  const int maxNumChannelType = cs.pcv->chrFormat != CHROMA_400 && CS::isDualITree( cs ) ? 2 : 1;

  for( int ch = 0; ch < maxNumChannelType; ch++ )
  {
    const ChannelType chType = ChannelType( ch );

    for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, chType ), chType ) )
    {
      switch( currCU.predMode )
      {
      case MODE_INTER:
        xDeriveCUMV( currCU );
        xReconInter( currCU );
        break;
      case MODE_INTRA:
        xReconIntraQT( currCU );
        break;
      default:
        THROW( "Invalid prediction mode" );
        break;
      }

      if( CU::isLosslessCoded( currCU ) && !currCU.ipcm )
      {
        xFillPCMBuffer( currCU );
      }

      DTRACE_BLOCK_REC( cs.picture->getRecoBuf( currCU ), currCU, currCU.predMode );
    }
  }
#if K0149_BLOCK_STATISTICS
  getAndStoreBlockStatistics(cs, ctuArea);
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void DecCu::xIntraRecBlk( TransformUnit& tu, const ComponentID compID )
{
  if( !tu.blocks[ compID ].valid() )
  {
    return;
  }

        CodingStructure &cs = *tu.cs;
  const CompArea &area      = tu.blocks[compID];

  const ChannelType chType  = toChannelType( compID );

        PelBuf piPred       = cs.getPredBuf( area );

  const PredictionUnit &pu  = *tu.cs->getPU( area.pos(), chType );
  const uint32_t uiChFinalMode  = PU::getFinalIntraMode( pu, chType );

  //===== init availability pattern =====

  const bool bUseFilteredPredictions = IntraPrediction::useFilteredIntraRefSamples( compID, pu, true, tu );
  m_pcIntraPred->initIntraPatternChType( *tu.cu, area, bUseFilteredPredictions );

  //===== get prediction signal =====
  if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
  {
    const PredictionUnit& pu = cs.pcv->noRQT && cs.pcv->only2Nx2N ? *tu.cu->firstPU : *tu.cs->getPU( tu.block( compID ), CHANNEL_TYPE_CHROMA );
    m_pcIntraPred->xGetLumaRecPixels( pu, area );
    m_pcIntraPred->predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
  }
  else
  {
    m_pcIntraPred->predIntraAng( compID, piPred, pu, bUseFilteredPredictions );
  }

  //===== inverse transform =====
  PelBuf piResi = cs.getResiBuf( area );

  const QpParam cQP( tu, compID );

  if( TU::getCbf( tu, compID ) )
  {
    m_pcTrQuant->invTransformNxN( tu, compID, piResi, cQP );
  }
  else
  {
    piResi.fill( 0 );
  }

  //===== reconstruction =====
  if( isChroma(compID) && tu.compAlpha[compID] != 0 )
  {
    CrossComponentPrediction::crossComponentPrediction( tu, compID, cs.getResiBuf( tu.Y() ), piResi, piResi, true );
  }

  PelBuf pReco = cs.getRecoBuf( area );

  cs.setDecomp( area );

#if KEEP_PRED_AND_RESI_SIGNALS
  pReco.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
#else
  piPred.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
#endif
#if !KEEP_PRED_AND_RESI_SIGNALS
  pReco.copyFrom( piPred );
#endif
#if REUSE_CU_RESULTS
  if( cs.pcv->isEncoder )
  {
    cs.picture->getRecoBuf( area ).copyFrom( pReco );
  }
#endif
}

void DecCu::xReconIntraQT( CodingUnit &cu )
{
  if( cu.ipcm )
  {
    xReconPCM( *cu.firstTU );
    return;
  }

  const uint32_t numChType = ::getNumberValidChannels( cu.chromaFormat );

  for( uint32_t chType = CHANNEL_TYPE_LUMA; chType < numChType; chType++ )
  {
    if( cu.blocks[chType].valid() )
    {
      xIntraRecQT( cu, ChannelType( chType ) );
    }
  }
}

/** Function for deriving reconstructed luma/chroma samples of a PCM mode CU.
* \param pcCU pointer to current CU
* \param uiPartIdx part index
* \param piPCM pointer to PCM code arrays
* \param piReco pointer to reconstructed sample arrays
* \param uiStride stride of reconstructed sample arrays
* \param uiWidth CU width
* \param uiHeight CU height
* \param compID colour component ID
* \returns void
*/
void DecCu::xDecodePCMTexture(TransformUnit &tu, const ComponentID compID)
{
  const CompArea &area         = tu.blocks[compID];
        PelBuf piPicReco       = tu.cs->getRecoBuf( area );
  const CPelBuf piPicPcm       = tu.getPcmbuf(compID);
  const SPS &sps               = *tu.cs->sps;
  const uint32_t uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));

  for (uint32_t uiY = 0; uiY < area.height; uiY++)
  {
    for (uint32_t uiX = 0; uiX < area.width; uiX++)
    {
      piPicReco.at(uiX, uiY) = (piPicPcm.at(uiX, uiY) << uiPcmLeftShiftBit);
    }
  }

  tu.cs->picture->getRecoBuf( area ).copyFrom( piPicReco );
  tu.cs->setDecomp( area );
}

/** Function for reconstructing a PCM mode CU.
* \param pcCU pointer to current CU
* \param uiDepth CU Depth
* \returns void
*/
void DecCu::xReconPCM(TransformUnit &tu)
{
  for (uint32_t ch = 0; ch < tu.blocks.size(); ch++)
  {
    ComponentID compID = ComponentID(ch);

    xDecodePCMTexture(tu, compID);
  }
}

/** Function for deriving reconstructed PU/CU chroma samples with QTree structure
* \param pcRecoYuv pointer to reconstructed sample arrays
* \param pcPredYuv pointer to prediction sample arrays
* \param pcResiYuv pointer to residue sample arrays
* \param chType    texture channel type (luma/chroma)
* \param rTu       reference to transform data
*
\ This function derives reconstructed PU/CU chroma samples with QTree recursive structure
*/

void
DecCu::xIntraRecQT(CodingUnit &cu, const ChannelType chType)
{
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    if( isLuma( chType ) )
    {
      xIntraRecBlk( currTU, COMPONENT_Y );
    }
    else
    {
      const uint32_t numValidComp = getNumberValidComponents( cu.chromaFormat );

      for( uint32_t compID = COMPONENT_Cb; compID < numValidComp; compID++ )
      {
        xIntraRecBlk( currTU, ComponentID( compID ) );
      }
    }
  }
}

/** Function for filling the PCM buffer of a CU using its reconstructed sample array
* \param pCU   pointer to current CU
* \param depth CU Depth
*/
void DecCu::xFillPCMBuffer(CodingUnit &cu)
{
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    for (const CompArea &area : currTU.blocks)
    {
      if( !area.valid() ) continue;;

      CPelBuf source      = cu.cs->getRecoBuf(area);
       PelBuf destination = currTU.getPcmbuf(area.compID);

      destination.copyFrom(source);
    }
  }
}

#include "CommonLib/dtrace_buffer.h"

void DecCu::xReconInter(CodingUnit &cu)
{
  // inter prediction
  m_pcInterPred->motionCompensation( cu );
#if JVET_L0266_HMVP
  cu.slice->updateMotionLUTs(cu.slice->getMotionLUTs(), cu);
#endif

  DTRACE    ( g_trace_ctx, D_TMP, "pred " );
  DTRACE_CRC( g_trace_ctx, D_TMP, *cu.cs, cu.cs->getPredBuf( cu ), &cu.Y() );
    
  // inter recon
  xDecodeInterTexture(cu);

  // clip for only non-zero cbf case
  CodingStructure &cs = *cu.cs;

  if (cu.rootCbf)
  {
#if KEEP_PRED_AND_RESI_SIGNALS
    cs.getRecoBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
#else
    cs.getResiBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
    cs.getRecoBuf( cu ).copyFrom   (                      cs.getResiBuf( cu ) );
#endif
  }
  else
  {
    cs.getRecoBuf(cu).copyClip(cs.getPredBuf(cu), cs.slice->clpRngs());
  }
  
  DTRACE    ( g_trace_ctx, D_TMP, "reco " );
  DTRACE_CRC( g_trace_ctx, D_TMP, *cu.cs, cu.cs->getRecoBuf( cu ), &cu.Y() );

  cs.setDecomp(cu);
}

void DecCu::xDecodeInterTU( TransformUnit & currTU, const ComponentID compID )
{
  if( !currTU.blocks[compID].valid() ) return;

  const CompArea &area = currTU.blocks[compID];

  CodingStructure& cs = *currTU.cs;

  //===== inverse transform =====
  PelBuf resiBuf  = cs.getResiBuf(area);

  const QpParam cQP(currTU, compID);

  if( TU::getCbf( currTU, compID ) )
  {
    m_pcTrQuant->invTransformNxN( currTU, compID, resiBuf, cQP );
  }
  else
  {
    resiBuf.fill( 0 );
  }

  //===== reconstruction =====
  if( isChroma( compID ) && currTU.compAlpha[compID] != 0 )
  {
    CrossComponentPrediction::crossComponentPrediction( currTU, compID, cs.getResiBuf( currTU.Y() ), resiBuf, resiBuf, true );
  }
}

void DecCu::xDecodeInterTexture(CodingUnit &cu)
{
  if( !cu.rootCbf )
  {
    return;
  }

  const uint32_t uiNumVaildComp = getNumberValidComponents(cu.chromaFormat);

  for (uint32_t ch = 0; ch < uiNumVaildComp; ch++)
  {
    const ComponentID compID = ComponentID(ch);

    for( auto& currTU : CU::traverseTUs( cu ) )
    {
      xDecodeInterTU( currTU, compID );
    }
  }
}

void DecCu::xDeriveCUMV( CodingUnit &cu )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    MergeCtx mrgCtx;

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
    if( pu.cu->affine )
    {
      CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType{ STATS__TOOL_AFF, pu.Y().width, pu.Y().height } );
    }
#endif

#if JVET_L0646_GBI
    uint8_t gbiIdx = GBI_DEFAULT;
#endif

    if( pu.mergeFlag )
    {
#if JVET_L0054_MMVD
      if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
      {
        if (pu.cs->sps->getSpsNext().getUseSubPuMvp())
        {
          Size bufSize = g_miScaling.scale(pu.lumaSize());
          mrgCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
        }

        if (cu.cs->pps->getLog2ParallelMergeLevelMinus2() && cu.partSize != SIZE_2Nx2N && cu.lumaSize().width <= 8)
        {
          if (!mrgCtx.hasMergedCandList)
          {
            // temporarily set size to 2Nx2N
            PartSize                 tmpPS = SIZE_2Nx2N;
            PredictionUnit           tmpPU = pu;
            static_cast<UnitArea&> (tmpPU) = cu;
            std::swap(tmpPS, cu.partSize);
#if JVET_L0054_MMVD
            int   fPosBaseIdx = pu.mmvdMergeIdx / MMVD_MAX_REFINE_NUM;
            PU::getInterMergeCandidates(tmpPU, mrgCtx, 1, fPosBaseIdx + 1);
#else
            PU::getInterMergeCandidates(tmpPU, mrgCtx, 255);
#endif
            PU::restrictBiPredMergeCands(pu, mrgCtx);
            PU::getInterMMVDMergeCandidates(tmpPU, mrgCtx,
              pu.mmvdMergeIdx
            );
            std::swap(tmpPS, cu.partSize);
            mrgCtx.hasMergedCandList = true;
          }
        }
        else
        {
#if JVET_L0054_MMVD
          int   fPosBaseIdx = pu.mmvdMergeIdx / MMVD_MAX_REFINE_NUM;
          PU::getInterMergeCandidates(pu, mrgCtx, 1, fPosBaseIdx + 1);
#else
          PU::getInterMergeCandidates(pu, mrgCtx, 255);
#endif
          PU::restrictBiPredMergeCands(pu, mrgCtx);
          PU::getInterMMVDMergeCandidates(pu, mrgCtx,
            pu.mmvdMergeIdx
          );
        }
        mrgCtx.setMmvdMergeCandiInfo(pu, pu.mmvdMergeIdx);

        PU::spanMotionInfo(pu, mrgCtx);
      }
      else
      {
#endif
      {
        if( pu.cu->affine )
        {
          pu.mergeIdx = 0;
          MvField       affineMvField[2][3];
          unsigned char interDirNeighbours;
          int           numValidMergeCand;
#if JVET_L0646_GBI
          PU::getAffineMergeCand( pu, affineMvField, interDirNeighbours, gbiIdx, numValidMergeCand);
#else
          PU::getAffineMergeCand( pu, affineMvField, interDirNeighbours, numValidMergeCand );
#endif
          pu.interDir = interDirNeighbours;
          for( int i = 0; i < 2; ++i )
          {
            if( pu.cs->slice->getNumRefIdx( RefPicList( i ) ) > 0 )
            {
              MvField* mvField = affineMvField[i];

              pu.mvpIdx[i] = 0;
              pu.mvpNum[i] = 0;
              pu.mvd[i]    = Mv();
              PU::setAllAffineMvField( pu, mvField, RefPicList( i ) );
#if JVET_L0646_GBI
              pu.cu->GBiIdx = gbiIdx;
#endif
            }
          }
          PU::spanMotionInfo( pu, mrgCtx );
        }
        else
        {
          if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
          {
            Size bufSize = g_miScaling.scale( pu.lumaSize() );
            mrgCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
          }

          if( cu.cs->pps->getLog2ParallelMergeLevelMinus2() && cu.partSize != SIZE_2Nx2N && cu.lumaSize().width <= 8 )
          {
            if( !mrgCtx.hasMergedCandList )
            {
              // temporarily set size to 2Nx2N
              PartSize                 tmpPS    = SIZE_2Nx2N;
              PredictionUnit           tmpPU    = pu;
              static_cast<UnitArea&> ( tmpPU )  = cu;
              std::swap( tmpPS, cu.partSize );
#if JVET_L0054_MMVD
              PU::getInterMergeCandidates(tmpPU, mrgCtx, 0, pu.mergeIdx);
#else
              PU::getInterMergeCandidates( tmpPU, mrgCtx, pu.mergeIdx );
#endif
              PU::restrictBiPredMergeCands(pu, mrgCtx);
              std::swap( tmpPS, cu.partSize );
              mrgCtx.hasMergedCandList          = true;
            }
          }
          else
          {
#if JVET_L0054_MMVD
            PU::getInterMergeCandidates(pu, mrgCtx, 0, pu.mergeIdx);
#else
            PU::getInterMergeCandidates( pu, mrgCtx, pu.mergeIdx );
#endif
            PU::restrictBiPredMergeCands(pu, mrgCtx);
          }

          mrgCtx.setMergeInfo( pu, pu.mergeIdx );

          PU::spanMotionInfo( pu, mrgCtx );
        }
      }
#if JVET_L0054_MMVD
      }
#endif
    }
    else
    {
#if REUSE_CU_RESULTS
        if (cu.imv && !cu.cs->pcv->isEncoder)
#else
        if (cu.imv)
#endif
        {
          PU::applyImv(pu, mrgCtx, m_pcInterPred);
        }
        else
      {
        if( pu.cu->affine )
        {
          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            RefPicList eRefList = RefPicList( uiRefListIdx );
            if ( pu.cs->slice->getNumRefIdx( eRefList ) > 0 && ( pu.interDir & ( 1 << uiRefListIdx ) ) )
            {
              AffineAMVPInfo affineAMVPInfo;
              PU::fillAffineMvpCand( pu, eRefList, pu.refIdx[eRefList], affineAMVPInfo );

              const unsigned mvp_idx = pu.mvpIdx[eRefList];

              pu.mvpNum[eRefList] = affineAMVPInfo.numCand;

              //    Mv mv[3];
              CHECK( pu.refIdx[eRefList] < 0, "Unexpected negative refIdx." );

              Mv mvLT = affineAMVPInfo.mvCandLT[mvp_idx] + pu.mvdAffi[eRefList][0];
              Mv mvRT = affineAMVPInfo.mvCandRT[mvp_idx] + pu.mvdAffi[eRefList][1];
              mvRT += pu.mvdAffi[eRefList][0];
#if REMOVE_MV_ADAPT_PREC
              mvLT.hor = mvLT.hor << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
              mvLT.ver = mvLT.ver << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
              mvRT.hor = mvRT.hor << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
              mvRT.ver = mvRT.ver << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#else
              CHECK(!mvLT.highPrec, "unexpected lp mv");
              CHECK(!mvRT.highPrec, "unexpected lp mv");
#endif

              Mv mvLB;
              if ( cu.affineType == AFFINEMODEL_6PARAM )
              {
                mvLB = affineAMVPInfo.mvCandLB[mvp_idx] + pu.mvdAffi[eRefList][2];
                mvLB += pu.mvdAffi[eRefList][0];
#if REMOVE_MV_ADAPT_PREC
                mvLB.hor = mvLB.hor << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
                mvLB.ver = mvLB.ver << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#else
                CHECK(!mvLB.highPrec, "unexpected lp mv");
#endif
              }
              PU::setAllAffineMv( pu, mvLT, mvRT, mvLB, eRefList );
            }
          }
        }
        else
        {
          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            RefPicList eRefList = RefPicList( uiRefListIdx );
            if ( pu.cs->slice->getNumRefIdx( eRefList ) > 0 && ( pu.interDir & ( 1 << uiRefListIdx ) ) )
            {
              AMVPInfo amvpInfo;
              PU::fillMvpCand(pu, eRefList, pu.refIdx[eRefList], amvpInfo);
              pu.mvpNum [eRefList] = amvpInfo.numCand;
              pu.mv     [eRefList] = amvpInfo.mvCand[pu.mvpIdx [eRefList]] + pu.mvd[eRefList];

#if REMOVE_MV_ADAPT_PREC
              pu.mv[eRefList].hor = pu.mv[eRefList].hor << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
              pu.mv[eRefList].ver = pu.mv[eRefList].ver << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#else
              if( pu.cs->sps->getSpsNext().getUseAffine() )
              {
                pu.mv[eRefList].setHighPrec();
              }
#endif
            }
          }
        }
        PU::spanMotionInfo( pu, mrgCtx );
      }
    }
  }
}
//! \}
