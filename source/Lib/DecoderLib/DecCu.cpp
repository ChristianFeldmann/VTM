/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
  m_tmpStorageLCU = NULL;
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
void DecCu::initDecCuReshaper  (Reshape* pcReshape, ChromaFormat chromaFormatIDC)
{
  m_pcReshape = pcReshape;
  if (m_tmpStorageLCU == NULL)
  {
    m_tmpStorageLCU = new PelStorage;
    m_tmpStorageLCU->create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  }

}
void DecCu::destoryDecCuReshaprBuf()
{
  if (m_tmpStorageLCU)
  {
    m_tmpStorageLCU->destroy();
    delete m_tmpStorageLCU;
    m_tmpStorageLCU = NULL;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void DecCu::decompressCtu( CodingStructure& cs, const UnitArea& ctuArea )
{

  const int maxNumChannelType = cs.pcv->chrFormat != CHROMA_400 && CS::isDualITree( cs ) ? 2 : 1;
  if (!cs.pcv->isEncoder)
  {
    m_shareStateDec = NO_SHARE;
  }
  bool sharePrepareCondition = ((!cs.pcv->isEncoder) && (!(cs.slice->isIntra()) || cs.slice->getSPS()->getIBCFlag()));

  for( int ch = 0; ch < maxNumChannelType; ch++ )
  {
    const ChannelType chType = ChannelType( ch );
    Position prevTmpPos;
    prevTmpPos.x = -1; prevTmpPos.y = -1;

    for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, chType ), chType ) )
    {
      if(sharePrepareCondition)
      {
        if ((currCU.shareParentPos.x >= 0) && (!(currCU.shareParentPos.x == prevTmpPos.x && currCU.shareParentPos.y == prevTmpPos.y)))
        {
          m_shareStateDec = GEN_ON_SHARED_BOUND;
#if !JVET_N0266_SMALL_BLOCKS
          cs.motionLut.lutShare = cs.motionLut.lut;
#endif
          cs.motionLut.lutShareIbc = cs.motionLut.lutIbc;
        }

        if (currCU.shareParentPos.x < 0)
        {
          m_shareStateDec = 0;
        }
        prevTmpPos = currCU.shareParentPos;
      }
      if (currCU.predMode != MODE_INTRA && currCU.Y().valid())
      {
        xDeriveCUMV(currCU);
      }
      switch( currCU.predMode )
      {
      case MODE_INTER:
      case MODE_IBC:
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
  m_pcIntraPred->initIntraPatternChType( *tu.cu, area);

  //===== get prediction signal =====
  if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
  {
    const PredictionUnit& pu = *tu.cu->firstPU;
    m_pcIntraPred->xGetLumaRecPixels( pu, area );
    m_pcIntraPred->predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
  }
  else
  {
#if JVET_N0217_MATRIX_INTRAPRED
    if( PU::isMIP( pu, chType ) )
    {
      m_pcIntraPred->initIntraMip( pu );
      m_pcIntraPred->predIntraMip( compID, piPred, pu );
    }
    else
    {
#endif
    m_pcIntraPred->predIntraAng( compID, piPred, pu );
#if JVET_N0217_MATRIX_INTRAPRED
    }
#endif
  }
  const Slice           &slice = *cs.slice;
#if JVET_N0805_APS_LMCS
  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
#else
  bool flag = slice.getReshapeInfo().getUseSliceReshaper() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
#endif
#if JVET_N0477_LMCS_CLEANUP
#if JVET_N0805_APS_LMCS
  if (flag && slice.getLmcsChromaResidualScaleFlag() && (compID != COMPONENT_Y) && (tu.cbf[COMPONENT_Cb] || tu.cbf[COMPONENT_Cr]))
#else
  if (flag && slice.getReshapeInfo().getSliceReshapeChromaAdj() && (compID != COMPONENT_Y) && (tu.cbf[COMPONENT_Cb] || tu.cbf[COMPONENT_Cr]))
#endif
#else
  if (flag && slice.getReshapeInfo().getSliceReshapeChromaAdj() && (compID != COMPONENT_Y))
#endif
  {
    const Area area = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].size()));
    const CompArea &areaY = CompArea(COMPONENT_Y, tu.chromaFormat, area);
    PelBuf piPredY;
    piPredY = cs.picture->getPredBuf(areaY);
    const Pel avgLuma = piPredY.computeAvg();
    int adj = m_pcReshape->calculateChromaAdj(avgLuma);
    tu.setChromaAdj(adj);
  }
  //===== inverse transform =====
  PelBuf piResi = cs.getResiBuf( area );

  const QpParam cQP( tu, compID );

#if JVET_N0054_JOINT_CHROMA
  // Joint chroma residual mode: Cr uses negative of the signalled Cb residual
  if ( tu.jointCbCr && compID == COMPONENT_Cr )
    piResi.copyAndNegate( cs.getResiBuf( tu.blocks[COMPONENT_Cb] ) );
  else
#endif
  if( TU::getCbf( tu, compID ) )
  {
    m_pcTrQuant->invTransformNxN( tu, compID, piResi, cQP );
  }
  else
  {
    piResi.fill( 0 );
  }

  //===== reconstruction =====
  flag = flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
#if JVET_N0805_APS_LMCS
  if (flag && TU::getCbf(tu, compID) && isChroma(compID) && slice.getLmcsChromaResidualScaleFlag())
#else
  if (flag && TU::getCbf(tu, compID) && isChroma(compID) && slice.getReshapeInfo().getSliceReshapeChromaAdj())
#endif
  {
#if JVET_N0054_JOINT_CHROMA
    if ( !(tu.jointCbCr && compID == COMPONENT_Cr) ) // // Joint chroma residual mode: chroma scaling took place already when doing Cb
#endif
    piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
  }
  if( isChroma(compID) && tu.compAlpha[compID] != 0 )
  {
    CrossComponentPrediction::crossComponentPrediction( tu, compID, cs.getResiBuf( tu.Y() ), piResi, piResi, true );
  }

  PelBuf pReco = cs.getRecoBuf( area );

  if( !tu.cu->ispMode || !isLuma( compID ) )
  {
    cs.setDecomp( area );
  }
  else if( tu.cu->ispMode && isLuma( compID ) && CU::isISPFirst( *tu.cu, tu.blocks[compID], compID ) )
  {
    cs.setDecomp( tu.cu->blocks[compID] );
  }

#if REUSE_CU_RESULTS
  CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
  PelBuf tmpPred;
#endif
#if JVET_N0805_APS_LMCS
  if (slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
#else
  if (slice.getReshapeInfo().getUseSliceReshaper() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
#endif
  {
#if REUSE_CU_RESULTS
    {
      tmpPred = m_tmpStorageLCU->getBuf(tmpArea);
      tmpPred.copyFrom(piPred);
    }
#endif
  }
#if KEEP_PRED_AND_RESI_SIGNALS
  pReco.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
#else
  piPred.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
#endif
#if !KEEP_PRED_AND_RESI_SIGNALS
  pReco.copyFrom( piPred );
#endif
#if JVET_N0805_APS_LMCS
  if (slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
#else
  if (slice.getReshapeInfo().getUseSliceReshaper() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
#endif
  {
#if REUSE_CU_RESULTS
    {
      piPred.copyFrom(tmpPred);
    }
#endif
  }
#if REUSE_CU_RESULTS
  if( cs.pcv->isEncoder )
  {
    cs.picture->getRecoBuf( area ).copyFrom( pReco );
    cs.picture->getPredBuf(area).copyFrom(piPred);
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
  const CodingStructure *cs = tu.cs;
  const ChannelType chType = tu.chType;

  ComponentID compStr = (CS::isDualITree(*cs) && !isLuma(chType)) ? COMPONENT_Cb: COMPONENT_Y;
  ComponentID compEnd = (CS::isDualITree(*cs) && isLuma(chType)) ? COMPONENT_Y : COMPONENT_Cr;
  for( ComponentID compID = compStr; compID <= compEnd; compID = ComponentID(compID+1) )
  {

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
  if( cu.triangle )
  {
    const bool    splitDir = cu.firstPU->triangleSplitDir;
    const uint8_t candIdx0 = cu.firstPU->triangleMergeIdx0;
    const uint8_t candIdx1 = cu.firstPU->triangleMergeIdx1;
    m_pcInterPred->motionCompensation4Triangle( cu, m_triangleMrgCtx, splitDir, candIdx0, candIdx1 );
    PU::spanTriangleMotionInfo( *cu.firstPU, m_triangleMrgCtx, splitDir, candIdx0, candIdx1 );
  }
  else
  {
  m_pcIntraPred->geneIntrainterPred(cu);

  // inter prediction
  CHECK(CU::isIBC(cu) && cu.firstPU->mhIntraFlag, "IBC and MHIntra cannot be used together");
  CHECK(CU::isIBC(cu) && cu.affine, "IBC and Affine cannot be used together");
  CHECK(CU::isIBC(cu) && cu.triangle, "IBC and triangle cannot be used together");
  CHECK(CU::isIBC(cu) && cu.firstPU->mmvdMergeFlag, "IBC and MMVD cannot be used together");
  const bool luma = cu.Y().valid();
  const bool chroma = cu.Cb().valid();
  if (luma && chroma)
  {
    m_pcInterPred->motionCompensation(cu);
  }
  else
  {
    m_pcInterPred->motionCompensation(cu, REF_PIC_LIST_0, luma, chroma);
  }
  }
  if (cu.Y().valid())
  {
    const PredictionUnit &pu = *cu.firstPU;
    if (!cu.affine && !cu.triangle)
    {
      MotionInfo mi = pu.getMotionInfo();
      mi.GBiIdx = (mi.interDir == 3) ? cu.GBiIdx : GBI_DEFAULT;
      cu.cs->addMiToLut(CU::isIBC(cu) ? cu.cs->motionLut.lutIbc : cu.cs->motionLut.lut, mi );
    }
  }

  if (cu.firstPU->mhIntraFlag)
  {
#if JVET_N0805_APS_LMCS
    if (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
#else
    if (cu.cs->slice->getReshapeInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag())
#endif
    {
      cu.cs->getPredBuf(*cu.firstPU).Y().rspSignal(m_pcReshape->getFwdLUT());
    }
    m_pcIntraPred->geneWeightedPred(COMPONENT_Y, cu.cs->getPredBuf(*cu.firstPU).Y(), *cu.firstPU, m_pcIntraPred->getPredictorPtr2(COMPONENT_Y, 0));
    m_pcIntraPred->geneWeightedPred(COMPONENT_Cb, cu.cs->getPredBuf(*cu.firstPU).Cb(), *cu.firstPU, m_pcIntraPred->getPredictorPtr2(COMPONENT_Cb, 0));
    m_pcIntraPred->geneWeightedPred(COMPONENT_Cr, cu.cs->getPredBuf(*cu.firstPU).Cr(), *cu.firstPU, m_pcIntraPred->getPredictorPtr2(COMPONENT_Cr, 0));
  }

  DTRACE    ( g_trace_ctx, D_TMP, "pred " );
  DTRACE_CRC( g_trace_ctx, D_TMP, *cu.cs, cu.cs->getPredBuf( cu ), &cu.Y() );

  // inter recon
  xDecodeInterTexture(cu);

  // clip for only non-zero cbf case
  CodingStructure &cs = *cu.cs;

  if (cu.rootCbf)
  {
#if REUSE_CU_RESULTS
    const CompArea &area = cu.blocks[COMPONENT_Y];
    CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
    PelBuf tmpPred;
#endif
#if JVET_N0805_APS_LMCS
    if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
#else
    if (cs.slice->getReshapeInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag())
#endif
    {
#if REUSE_CU_RESULTS
      if (cs.pcv->isEncoder)
      {
        tmpPred = m_tmpStorageLCU->getBuf(tmpArea);
        tmpPred.copyFrom(cs.getPredBuf(cu).get(COMPONENT_Y));
      }
#endif
      if (!cu.firstPU->mhIntraFlag && !CU::isIBC(cu))
        cs.getPredBuf(cu).get(COMPONENT_Y).rspSignal(m_pcReshape->getFwdLUT());
    }
#if KEEP_PRED_AND_RESI_SIGNALS
    cs.getRecoBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
#else
    cs.getResiBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
    cs.getRecoBuf( cu ).copyFrom   (                      cs.getResiBuf( cu ) );
#endif
#if JVET_N0805_APS_LMCS
    if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
#else
    if (cs.slice->getReshapeInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag())
#endif
    {
#if REUSE_CU_RESULTS
      if (cs.pcv->isEncoder)
      {
        cs.getPredBuf(cu).get(COMPONENT_Y).copyFrom(tmpPred);
      }
#endif
    }
  }
  else
  {
    cs.getRecoBuf(cu).copyClip(cs.getPredBuf(cu), cs.slice->clpRngs());
#if JVET_N0805_APS_LMCS
    if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->mhIntraFlag && !CU::isIBC(cu))
#else
    if (cs.slice->getReshapeInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag() && !cu.firstPU->mhIntraFlag && !CU::isIBC(cu))
#endif
    {
      cs.getRecoBuf(cu).get(COMPONENT_Y).rspSignal(m_pcReshape->getFwdLUT());
    }
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

#if JVET_N0054_JOINT_CHROMA
  // Joint chroma residual mode: Cr uses negative of the signalled Cb residual
  if ( currTU.jointCbCr && compID == COMPONENT_Cr )
    resiBuf.copyAndNegate( cs.getResiBuf( currTU.blocks[COMPONENT_Cb] ) );
  else
#endif
  if( TU::getCbf( currTU, compID ) )
  {
    m_pcTrQuant->invTransformNxN( currTU, compID, resiBuf, cQP );
  }
  else
  {
    resiBuf.fill( 0 );
  }

  //===== reconstruction =====
  const Slice           &slice = *cs.slice;
#if JVET_N0805_APS_LMCS
  if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isChroma(compID) && TU::getCbf(currTU, compID) && slice.getLmcsChromaResidualScaleFlag() && currTU.blocks[compID].width*currTU.blocks[compID].height > 4)
#else
  if ( slice.getReshapeInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag() && isChroma(compID) && TU::getCbf(currTU, compID) && slice.getReshapeInfo().getSliceReshapeChromaAdj() && currTU.blocks[compID].width*currTU.blocks[compID].height > 4 )
#endif
  {
#if JVET_N0054_JOINT_CHROMA
    if ( !(currTU.jointCbCr && compID == COMPONENT_Cr) ) // Joint chroma residual mode: chroma scaling took place already when doing Cb
#endif
    resiBuf.scaleSignal(currTU.getChromaAdj(), 0, currTU.cu->cs->slice->clpRng(compID));
  }
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
      CodingStructure  &cs = *cu.cs;
      const Slice &slice = *cs.slice;
#if JVET_N0477_LMCS_CLEANUP
#if JVET_N0805_APS_LMCS
      if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && slice.getLmcsChromaResidualScaleFlag() && (compID == COMPONENT_Y) && (currTU.cbf[COMPONENT_Cb] || currTU.cbf[COMPONENT_Cr]))
#else
      if (slice.getReshapeInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag() && slice.getReshapeInfo().getSliceReshapeChromaAdj() && (compID == COMPONENT_Y) && (currTU.cbf[COMPONENT_Cb] || currTU.cbf[COMPONENT_Cr]))
#endif
#else
      if (slice.getReshapeInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag() && slice.getReshapeInfo().getSliceReshapeChromaAdj() && (compID == COMPONENT_Y))
#endif
      {
        const CompArea &areaY = currTU.blocks[COMPONENT_Y];
        PelBuf predY = cs.getPredBuf(areaY);
        CompArea tmpArea(COMPONENT_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
        PelBuf tmpPred = m_tmpStorageLCU->getBuf(tmpArea);
        tmpPred.copyFrom(predY);
      if (!cu.firstPU->mhIntraFlag && !CU::isIBC(cu))
          tmpPred.rspSignal(m_pcReshape->getFwdLUT());
        const Pel avgLuma = tmpPred.computeAvg();
        int adj = m_pcReshape->calculateChromaAdj(avgLuma);
        currTU.setChromaAdj(adj);
    }
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


    if( pu.mergeFlag )
    {
      if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
      {
        CHECK(pu.mhIntraFlag == true, "invalid MHIntra");
        if (pu.cs->sps->getSBTMVPEnabledFlag())
        {
          Size bufSize = g_miScaling.scale(pu.lumaSize());
          mrgCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
        }

        int   fPosBaseIdx = pu.mmvdMergeIdx / MMVD_MAX_REFINE_NUM;
          pu.shareParentPos = cu.shareParentPos;
          pu.shareParentSize = cu.shareParentSize;
        PU::getInterMergeCandidates(pu, mrgCtx, 1, fPosBaseIdx + 1);
        PU::getInterMMVDMergeCandidates(pu, mrgCtx,
          pu.mmvdMergeIdx
        );
        mrgCtx.setMmvdMergeCandiInfo(pu, pu.mmvdMergeIdx);

        PU::spanMotionInfo(pu, mrgCtx);
      }
      else
      {
      {
        if( pu.cu->triangle )
        {
#if JVET_N0340_TRI_MERGE_CAND && !JVET_N0266_SMALL_BLOCKS
          pu.shareParentPos = cu.shareParentPos;
          pu.shareParentSize = cu.shareParentSize;
#endif
          PU::getTriangleMergeCandidates( pu, m_triangleMrgCtx );
        }
        else
        {
        if( pu.cu->affine )
        {
          AffineMergeCtx affineMergeCtx;
          if ( pu.cs->sps->getSBTMVPEnabledFlag() )
          {
            Size bufSize = g_miScaling.scale( pu.lumaSize() );
            mrgCtx.subPuMvpMiBuf = MotionBuf( m_SubPuMiBuf, bufSize );
            affineMergeCtx.mrgCtx = &mrgCtx;
          }
          PU::getAffineMergeCand( pu, affineMergeCtx, pu.mergeIdx );
          pu.interDir = affineMergeCtx.interDirNeighbours[pu.mergeIdx];
          pu.cu->affineType = affineMergeCtx.affineType[pu.mergeIdx];
          pu.cu->GBiIdx = affineMergeCtx.GBiIdx[pu.mergeIdx];
          pu.mergeType = affineMergeCtx.mergeType[pu.mergeIdx];
          if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP )
          {
            pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + 0][0].refIdx;
            pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + 1][0].refIdx;
          }
          else
          {
          for( int i = 0; i < 2; ++i )
          {
            if( pu.cs->slice->getNumRefIdx( RefPicList( i ) ) > 0 )
            {
              MvField* mvField = affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + i];
              pu.mvpIdx[i] = 0;
              pu.mvpNum[i] = 0;
              pu.mvd[i]    = Mv();
              PU::setAllAffineMvField( pu, mvField, RefPicList( i ) );
            }
          }
        }
          PU::spanMotionInfo( pu, mrgCtx );
        }
        else
        {
          pu.shareParentPos = cu.shareParentPos;
          pu.shareParentSize = cu.shareParentSize;
          if (CU::isIBC(*pu.cu))
            PU::getIBCMergeCandidates(pu, mrgCtx, pu.mergeIdx);
          else
            PU::getInterMergeCandidates(pu, mrgCtx, 0, pu.mergeIdx);
          mrgCtx.setMergeInfo( pu, pu.mergeIdx );

          PU::spanMotionInfo( pu, mrgCtx );
        }
        }
      }
      }
    }
    else
    {
#if JVET_N0843_BVP_SIMPLIFICATION
      pu.shareParentPos = cu.shareParentPos;
      pu.shareParentSize = cu.shareParentSize;
#endif
#if REUSE_CU_RESULTS
      if ( cu.imv && !pu.cu->affine && !cu.cs->pcv->isEncoder )
#else
        if (cu.imv && !pu.cu->affine)
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
              if (!cu.cs->pcv->isEncoder)
              {
                pu.mvdAffi[eRefList][0].changeAffinePrecAmvr2Internal(pu.cu->imv);
                pu.mvdAffi[eRefList][1].changeAffinePrecAmvr2Internal(pu.cu->imv);
                if (cu.affineType == AFFINEMODEL_6PARAM)
                {
                  pu.mvdAffi[eRefList][2].changeAffinePrecAmvr2Internal(pu.cu->imv);
                }
              }

              Mv mvLT = affineAMVPInfo.mvCandLT[mvp_idx] + pu.mvdAffi[eRefList][0];
              Mv mvRT = affineAMVPInfo.mvCandRT[mvp_idx] + pu.mvdAffi[eRefList][1];
              mvRT += pu.mvdAffi[eRefList][0];

              Mv mvLB;
              if ( cu.affineType == AFFINEMODEL_6PARAM )
              {
                mvLB = affineAMVPInfo.mvCandLB[mvp_idx] + pu.mvdAffi[eRefList][2];
                mvLB += pu.mvdAffi[eRefList][0];
              }
#if JVET_N0334_MVCLIPPING
              PU::setAllAffineMv(pu, mvLT, mvRT, mvLB, eRefList, true);
#else
              PU::setAllAffineMv( pu, mvLT, mvRT, mvLB, eRefList );
#endif
            }
          }
        }
        else if (CU::isIBC(*pu.cu) && pu.interDir == 1)
        {
          AMVPInfo amvpInfo;
          PU::fillIBCMvpCand(pu, amvpInfo);
          pu.mvpNum[REF_PIC_LIST_0] = amvpInfo.numCand;
          Mv mvd = pu.mvd[REF_PIC_LIST_0];
#if REUSE_CU_RESULTS
          if (!cu.cs->pcv->isEncoder)
#endif
          {
            mvd.changeIbcPrecAmvr2Internal(pu.cu->imv);
          }
          pu.mv[REF_PIC_LIST_0] = amvpInfo.mvCand[pu.mvpIdx[REF_PIC_LIST_0]] + mvd;
#if JVET_N0334_MVCLIPPING
          pu.mv[REF_PIC_LIST_0].mvCliptoStorageBitDepth();
#endif
        }
        else
        {
          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            RefPicList eRefList = RefPicList( uiRefListIdx );
            if ((pu.cs->slice->getNumRefIdx(eRefList) > 0 || (eRefList == REF_PIC_LIST_0 && CU::isIBC(*pu.cu))) && (pu.interDir & (1 << uiRefListIdx)))
            {
              AMVPInfo amvpInfo;
              PU::fillMvpCand(pu, eRefList, pu.refIdx[eRefList], amvpInfo);
              pu.mvpNum [eRefList] = amvpInfo.numCand;
              if (!cu.cs->pcv->isEncoder)
              {
                pu.mvd[eRefList].changeTransPrecAmvr2Internal(pu.cu->imv);
              }
              pu.mv[eRefList] = amvpInfo.mvCand[pu.mvpIdx[eRefList]] + pu.mvd[eRefList];
#if JVET_N0334_MVCLIPPING
              pu.mv[eRefList].mvCliptoStorageBitDepth();
#endif
            }
          }
        }
        PU::spanMotionInfo( pu, mrgCtx );
      }
    }
    if( !cu.triangle )
    {
      if( g_mctsDecCheckEnabled && !MCTSHelper::checkMvBufferForMCTSConstraint( pu, true ) )
      {
        printf( "DECODER: pu motion vector across tile boundaries (%d,%d,%d,%d)\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight() );
      }
    }
    if (CU::isIBC(cu))
    {
      const int cuPelX = pu.Y().x;
      const int cuPelY = pu.Y().y;
      int roiWidth = pu.lwidth();
      int roiHeight = pu.lheight();
      const int picWidth = pu.cs->slice->getSPS()->getPicWidthInLumaSamples();
      const int picHeight = pu.cs->slice->getSPS()->getPicHeightInLumaSamples();
      const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
      int xPred = pu.mv[0].getHor() >> MV_FRACTIONAL_BITS_INTERNAL;
      int yPred = pu.mv[0].getVer() >> MV_FRACTIONAL_BITS_INTERNAL;
      CHECK(!PU::isBlockVectorValid(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, 0, xPred, yPred, lcuWidth), "invalid block vector for IBC detected.");
    }
  }
}
//! \}
