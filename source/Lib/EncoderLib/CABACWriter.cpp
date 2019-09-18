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

/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CommonLib/Contexts.h"
#include "CABACWriter.h"

#include "EncLib.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"

#include <map>
#include <algorithm>
#include <limits>


//! \ingroup EncoderLib
//! \{

void CABACWriter::initCtxModels( const Slice& slice )
{
  int       qp                = slice.getSliceQp();
  SliceType sliceType         = slice.getSliceType();
  SliceType encCABACTableIdx  = slice.getEncCABACTableIdx();
  if( !slice.isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && slice.getPPS()->getCabacInitPresentFlag() )
  {
    sliceType = encCABACTableIdx;
  }
  m_BinEncoder.reset( qp, (int)sliceType );
}



template <class BinProbModel>
SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore<BinProbModel>& ctxStoreTest = static_cast<const CtxStore<BinProbModel>&>( ctxTest );
  const CtxStore<BinProbModel>& ctxStoreRef  = static_cast<const CtxStore<BinProbModel>&>( binEncoder.getCtx() );
  int qp = slice.getSliceQp();
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (uint32_t idx=0; idx<2; idx++)
    {
      uint64_t  curCost           = 0;
      SliceType curSliceType      = aSliceTypeChoices[idx];
      ctxTest.init( qp, (int)curSliceType );
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          curCost += uint64_t( binEncoder.getNumBins(k) ) * ctxStoreRef[k].estFracExcessBits( ctxStoreTest[k] );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}


SliceType CABACWriter::getCtxInitId( const Slice& slice )
{
  switch( m_TestCtx.getBPMType() )
  {
  case BPM_Std:   return  xGetCtxInitId<BinProbModel_Std>   ( slice, m_BinEncoder, m_TestCtx );
  default:        return  NUMBER_OF_SLICE_TYPES;
  }
}



unsigned estBits( BinEncIf& binEnc, const std::vector<bool>& bins, const Ctx& ctx, const int ctxId, const uint8_t winSize )
{
  binEnc.initCtxAndWinSize( ctxId, ctx, winSize );
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for( std::size_t binId = 0; binId < numBins; binId++ )
  {
    unsigned  bin = ( bins[binId] ? 1 : 0 );
    binEnc.encodeBin( bin, ctxId );
  }
  unsigned endBits    = binEnc.getNumWrittenBits();
  unsigned codedBits  = endBits - startBits;
  return   codedBits;
}





//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_BinEncoder.encodeBinTrm ( 1 );
  m_BinEncoder.finish       ();
}




//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao, skipAlf )
//================================================================================

void CABACWriter::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr, bool skipSao /* = false */, bool skipAlf /* = false */ )
{
  CUCtx cuCtx( qps[CH_L] );
  QTBTPartitioner partitioner;

  partitioner.initCtu(area, CH_L, *cs.slice);

  if( !skipSao )
  {
    sao( *cs.slice, ctuRsAddr );
  }

  if (!skipAlf)
  {
    for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      codeAlfCtuEnableFlag(cs, ctuRsAddr, compIdx, NULL);
      if (isLuma(ComponentID(compIdx)))
      {
        codeAlfCtuFilterIndex(cs, ctuRsAddr, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y));
      }
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if (isChroma(ComponentID(compIdx)))
      {
        uint8_t* ctbAlfFlag = cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx) ? cs.slice->getPic()->getAlfCtuEnableFlag( compIdx ) : nullptr;
        if( ctbAlfFlag && ctbAlfFlag[ctuRsAddr] )
        {
          codeAlfCtuAlternative( cs, ctuRsAddr, compIdx );
        }
      }
#endif
    }
  }

  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
  {
    CUCtx chromaCuCtx(qps[CH_C]);
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, CH_C, *cs.slice);
    coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &chromaCuCtx);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = chromaCuCtx.qp;
  }
  else
  {
    coding_tree(cs, partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner.initCtu(area, CH_C, *cs.slice);
      coding_tree(cs, partitioner, cuCtxChroma);
      qps[CH_C] = cuCtxChroma.qp;
    }
  }
}





//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_pars  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_pars ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao( const Slice& slice, unsigned ctuRsAddr )
{
  const SPS& sps = *slice.getSPS();
  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  CodingStructure&     cs                     = *slice.getPic()->cs;
  const PreCalcValues& pcv                    = *cs.pcv;
  const SAOBlkParam&  sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool                slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool                slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  bool                sliceEnabled[3]         = { slice_sao_luma_flag, slice_sao_chroma_flag, slice_sao_chroma_flag };
  int                 frame_width_in_ctus     = pcv.widthInCtus;
  int                 ry                      = ctuRsAddr      / frame_width_in_ctus;
  int                 rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                     ( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned      curSliceIdx             = slice.getIndependentSliceIdx();
  const unsigned      curTileIdx              = cs.picture->brickMap->getBrickIdxRsMap( pos );
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0  ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  sao_block_pars( sao_ctu_pars, sps.getBitDepths(), sliceEnabled, leftMergeAvail, aboveMergeAvail, false );
}


void CABACWriter::sao_block_pars( const SAOBlkParam& saoPars, const BitDepths& bitDepths, bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo )
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if( leftMergeAvail )
  {
    // sao_merge_left_flag
    isLeftMerge   = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_LEFT );
    m_BinEncoder.encodeBin( (isLeftMerge), Ctx::SaoMergeFlag() );
  }
  if( aboveMergeAvail && !isLeftMerge )
  {
    // sao_merge_above_flag
    isAboveMerge  = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_ABOVE );
    m_BinEncoder.encodeBin( (isAboveMerge), Ctx::SaoMergeFlag() );
  }
  if( onlyEstMergeInfo )
  {
    return; //only for RDO
  }
  if( !isLeftMerge && !isAboveMerge )
  {
    // explicit parameters
    for( int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      sao_offset_pars( saoPars[compIdx], ComponentID(compIdx), sliceEnabled[compIdx], bitDepths.recon[ toChannelType(ComponentID(compIdx)) ] );
    }
  }
}


void CABACWriter::sao_offset_pars( const SAOOffset& ctbPars, ComponentID compID, bool sliceEnabled, int bitDepth )
{
  if( !sliceEnabled )
  {
    CHECK( ctbPars.modeIdc != SAO_MODE_OFF, "Sao must be off, if it is disabled on slice level" );
    return;
  }
  const bool isFirstCompOfChType = ( getFirstComponentOfChannel( toChannelType(compID) ) == compID );

  if( isFirstCompOfChType )
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if( ctbPars.modeIdc == SAO_MODE_OFF )
    {
      m_BinEncoder.encodeBin  ( 0, Ctx::SaoTypeIdx() );
    }
    else if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 0 );
    }
    else
    {
      CHECK(!( ctbPars.typeIdc < SAO_TYPE_START_BO ), "Unspecified error");
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 1 );
    }
  }

  if( ctbPars.modeIdc == SAO_MODE_NEW )
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
    int       numClasses    = ( ctbPars.typeIdc == SAO_TYPE_BO ? 4 : NUM_SAO_EO_CLASSES );
    int       k             = 0;
    int       offset[4];
    for( int i = 0; i < numClasses; i++ )
    {
      if( ctbPars.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN )
      {
        continue;
      }
      int classIdx = ( ctbPars.typeIdc == SAO_TYPE_BO ? ( ctbPars.typeAuxInfo + i ) % NUM_SAO_BO_CLASSES : i );
      offset[k++]  = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for( int i = 0; i < 4; i++ )
    {
      unsigned absOffset = ( offset[i] < 0 ? -offset[i] : offset[i] );
      unary_max_eqprob( absOffset, maxOffsetQVal );
    }

    // band offset mode
    if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      // sao_offset_sign
      for( int i = 0; i < 4; i++ )
      {
        if( offset[i] )
        {
          m_BinEncoder.encodeBinEP( (offset[i] < 0) );
        }
      }
      // sao_band_position
      m_BinEncoder.encodeBinsEP( ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2 );
    }
    // edge offset mode
    else
    {
      if( isFirstCompOfChType )
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        CHECK( ctbPars.typeIdc - SAO_TYPE_START_EO < 0, "sao edge offset class is outside valid range" );
        m_BinEncoder.encodeBinsEP( ctbPars.typeIdc - SAO_TYPE_START_EO, NUM_SAO_EO_TYPES_LOG2 );
      }
    }
  }
}



//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

void CABACWriter::coding_tree(const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  const CodingUnit &cu        = *cs.getCU( currArea.blocks[partitioner.chType], partitioner.chType );

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
#if JVET_O0050_LOCAL_DUAL_TREE
  //Note: do not reset qg at chroma CU
  if( pps.getUseDQP() && partitioner.currQgEnable() && !isChroma( partitioner.chType ) )
#else
  if( pps.getUseDQP() && partitioner.currQgEnable() )
#endif
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded          = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
  }
  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.getUseDQP() && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->getUseChromaQpAdj() && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
    }
  }

  const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

  split_cu_mode( splitMode, cs, partitioner );

  CHECK( !partitioner.canSplit( splitMode, cs ), "The chosen split mode is invalid!" );

  if( splitMode != CU_DONT_SPLIT )
  {
      if (CS::isDualITree(cs) && pPartitionerChroma != nullptr && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
      {
        partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
        pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
        bool beContinue = true;
        bool lumaContinue = true;
        bool chromaContinue = true;

        while (beContinue)
        {
          if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
          {
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
            }
            lumaContinue = partitioner.nextPart(cs);
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
          else
          {
            //dual tree coding under 64x64 block
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx);
            }
            lumaContinue = partitioner.nextPart(cs);
            if (cs.picture->blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
            {
              coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
            }
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
        }
        partitioner.exitCurrSplit();
        pPartitionerChroma->exitCurrSplit();

      }
      else
      {
#if JVET_O0050_LOCAL_DUAL_TREE
        const ModeType modeTypeParent = partitioner.modeType;
        const ModeType modeTypeChild = CU::getModeTypeAtDepth( cu, partitioner.currDepth );
        mode_constraint( splitMode, cs, partitioner, modeTypeChild );
        partitioner.modeType = modeTypeChild;

        bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA ? true : false;
        CHECK( chromaNotSplit && partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma" );
        if( partitioner.treeType == TREE_D )
        {
          partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
        }
#endif
      partitioner.splitCurrArea( splitMode, cs );

      do
      {
        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
#if JVET_O0050_LOCAL_DUAL_TREE
      if( chromaNotSplit )
      {
        CHECK( partitioner.chType != CHANNEL_TYPE_LUMA, "must be luma status" );
        partitioner.chType = CHANNEL_TYPE_CHROMA;
        partitioner.treeType = TREE_C;

        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }

        //recover
        partitioner.chType = CHANNEL_TYPE_LUMA;
        partitioner.treeType = TREE_D;
      }
      partitioner.modeType = modeTypeParent;
#endif
      }
      return;
  }

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }
#if JVET_O0050_LOCAL_DUAL_TREE
  CHECK( cu.treeType != partitioner.treeType, "treeType mismatch" );
#endif


  // coding unit
  coding_unit( cu, partitioner, cuCtx );

#if JVET_O0050_LOCAL_DUAL_TREE
  if( cu.chType == CHANNEL_TYPE_CHROMA )
  {
    DTRACE_COND( (isEncoding()), g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
#endif
  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
#if JVET_O0050_LOCAL_DUAL_TREE
  }
#endif
  DTRACE_BLOCK_REC_COND( ( !isEncoding() ), cs.picture->getRecoBuf( cu ), cu, cu.predMode );
}

#if JVET_O0050_LOCAL_DUAL_TREE
void CABACWriter::mode_constraint( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner, const ModeType modeType )
{
  CHECK( split == CU_DONT_SPLIT, "splitMode shall not be no split" );
  int val = cs.signalModeCons( split, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    CHECK( modeType == MODE_TYPE_ALL, "shall not be no constraint case" );
    bool flag = modeType == MODE_TYPE_INTRA;
    int ctxIdx = DeriveCtx::CtxModeConsFlag( cs, partitioner );
    m_BinEncoder.encodeBin( flag, Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    assert( modeType == MODE_TYPE_INTRA );
  }
  else
  {
    assert( modeType == partitioner.modeType );
  }
}
#endif

void CABACWriter::split_cu_mode( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner )
{
  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  const bool canSplit = canBh || canBv || canTh || canTv || canQt;
  const bool isNo     = split == CU_DONT_SPLIT;

  if( canNo && canSplit )
  {
    m_BinEncoder.encodeBin( !isNo, Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, !isNo );

  if( isNo )
  {
    return;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  const bool isQt   = split == CU_QUAD_SPLIT;

  if( canQt && canBtt )
  {
    m_BinEncoder.encodeBin( isQt, Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return;
  }

  const bool canHor = canBh || canTh;
  const bool canVer = canBv || canTv;
  const bool  isVer = split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT;

  if( canVer && canHor )
  {
    m_BinEncoder.encodeBin( isVer, Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  const bool can12 = isVer ? canBv : canBh;
  const bool  is12 = isVer ? ( split == CU_VERT_SPLIT ) : ( split == CU_HORZ_SPLIT );

  if( can12 && can14 )
  {
    m_BinEncoder.encodeBin( is12, Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, split );
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_transquant_bypass_flag ( cu )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
#if !JVET_O0525_REMOVE_PCM
//    void  pcm_flag                  ( cu )
//    void  pcm_samples               ( tu )
#endif
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACWriter::coding_unit( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
#if JVET_O0050_LOCAL_DUAL_TREE
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
#endif
  CodingStructure& cs = *cu.cs;
  // transquant bypass flag
  if( cs.pps->getTransquantBypassEnabledFlag() )
  {
    cu_transquant_bypass_flag( cu );
  }

  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
  {
    cu_skip_flag( cu );
  }


  // skip data
  if( cu.skip )
  {
    CHECK( !cu.firstPU->mergeFlag, "Merge flag has to be on!" );
    PredictionUnit&   pu = *cu.firstPU;
    prediction_unit ( pu );
    end_of_ctu      ( cu, cuCtx );
    return;
  }

#if !JVET_O0525_REMOVE_PCM
#if !FIX_PCM
  // pcm samples
  if( CU::isIntra(cu) )
  {
    pcm_data( cu, partitioner );
    if( cu.ipcm )
    {
      end_of_ctu( cu, cuCtx );
      return;
    }
  }
#endif
#endif

  // prediction mode and partitioning data
  pred_mode ( cu );
#if JVET_O0119_BASE_PALETTE_444
  if (CU::isPLT(cu))
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    if (cu.isSepTree())
#else
    if (CS::isDualITree(*cu.cs))
#endif
    {
      if (isLuma(partitioner.chType))
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
      if (cu.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
      {
        cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
      }
    }
    else
    {
      cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
    }
    end_of_ctu(cu, cuCtx);
    return;
  }
#endif
  bdpcm_mode( cu, ComponentID( partitioner.chType ) );

#if !JVET_O0525_REMOVE_PCM
#if FIX_PCM
  // pcm samples
  if( CU::isIntra(cu) )
  {
    pcm_data( cu, partitioner );
    if( cu.ipcm )
    {
      end_of_ctu( cu, cuCtx );
      return;
    }
  }
#endif
#endif

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // end of cu
  end_of_ctu( cu, cuCtx );
}


void CABACWriter::cu_transquant_bypass_flag( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( (cu.transQuantBypass), Ctx::TransquantBypassFlag() );
}


void CABACWriter::cu_skip_flag( const CodingUnit& cu )
{
  unsigned ctxId = DeriveCtx::CtxSkipFlag( cu );

#if JVET_O0050_LOCAL_DUAL_TREE
  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getSPS()->getIBCFlag())
#else
  if (cu.slice->isIntra() && cu.cs->slice->getSPS()->getIBCFlag())
#endif
  {
#if JVET_O1161_IBC_MAX_SIZE
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
    if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
    {
    m_BinEncoder.encodeBin((cu.skip), Ctx::SkipFlag(ctxId));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0);
    }
    return;
  }
  if ( !cu.cs->slice->getSPS()->getIBCFlag() && cu.lwidth() == 4 && cu.lheight() == 4 )
  {
    return;
  }
#if JVET_O0050_LOCAL_DUAL_TREE
  if( !cu.cs->slice->getSPS()->getIBCFlag() && cu.isConsIntra() )
  {
    return;
  }
#endif
  m_BinEncoder.encodeBin( ( cu.skip ), Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0 );
  if (cu.skip && cu.cs->slice->getSPS()->getIBCFlag())
  {
#if JVET_O1161_IBC_MAX_SIZE
#if JVET_O0050_LOCAL_DUAL_TREE
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !cu.isConsInter()) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#else
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#endif
#else
#if JVET_O0050_LOCAL_DUAL_TREE
    if ((cu.lwidth() < 128 || cu.lheight() < 128) && !cu.isConsInter()) // disable 128x128 IBC mode and disable IBC when only allowing inter mode
#else
    if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
#endif
    {
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        return;
      }
    unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
    m_BinEncoder.encodeBin(CU::isIBC(cu) ? 1 : 0, Ctx::IBCFlag(ctxidx));
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
#if !JVET_O0249_MERGE_SYNTAX
    if (CU::isInter(cu))
    {
      if (!cu.cs->slice->getSPS()->getUseMMVD() && (cu.firstPU->lwidth() * cu.firstPU->lheight() == 32))
      {
        CHECK(!cu.firstPU->regularMergeFlag, "regular_merge_flag must be true!");
      }
      else
      {
        m_BinEncoder.encodeBin(cu.firstPU->regularMergeFlag, Ctx::RegularMergeFlag(0));
        DTRACE(g_trace_ctx, D_SYNTAX, "regularMergeFlag() ctx=%d regularMergeFlag=%d\n", 0, cu.firstPU->regularMergeFlag?1:0);
      }
      if (cu.cs->slice->getSPS()->getUseMMVD())
      {
        bool isCUWithOnlyRegularAndMMVD=((cu.firstPU->lwidth() == 8 && cu.firstPU->lheight() == 4) || (cu.firstPU->lwidth() == 4 && cu.firstPU->lheight() == 8));
        if (isCUWithOnlyRegularAndMMVD)
        {
          CHECK(cu.mmvdSkip==cu.firstPU->regularMergeFlag, "mmvdSkip_flag must be !regularMergeFlag");
        }
        else if (!cu.firstPU->regularMergeFlag)
        {
          m_BinEncoder.encodeBin(cu.mmvdSkip, Ctx::MmvdFlag(0));
          DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, cu.mmvdSkip ? 1 : 0);
        }
      }
    }
#endif
  }
#if !JVET_O0249_MERGE_SYNTAX
  if (cu.skip && !cu.cs->slice->getSPS()->getIBCFlag())
  {
    if (!cu.cs->slice->getSPS()->getUseMMVD() && (cu.firstPU->lwidth() * cu.firstPU->lheight() == 32))
    {
      CHECK(!cu.firstPU->regularMergeFlag, "regular_merge_flag must be true!");
    }
    else
    {
      m_BinEncoder.encodeBin(cu.firstPU->regularMergeFlag, Ctx::RegularMergeFlag(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "regularMergeFlag() ctx=%d regularMergeFlag=%d\n", 0, cu.firstPU->regularMergeFlag?1:0);
    }
    if (cu.cs->slice->getSPS()->getUseMMVD())
    {
      bool isCUWithOnlyRegularAndMMVD=((cu.firstPU->lwidth() == 8 && cu.firstPU->lheight() == 4) || (cu.firstPU->lwidth() == 4 && cu.firstPU->lheight() == 8));
      if (isCUWithOnlyRegularAndMMVD)
      {
        CHECK(cu.mmvdSkip==cu.firstPU->regularMergeFlag, "mmvdSkip_flag must be !regularMergeFlag");
      }
      else if (!cu.firstPU->regularMergeFlag)
      {
        m_BinEncoder.encodeBin(cu.mmvdSkip, Ctx::MmvdFlag(0));
        DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, cu.mmvdSkip ? 1 : 0);
      }
    }
  }
#endif
}


void CABACWriter::pred_mode( const CodingUnit& cu )
{
#if JVET_O0258_REMOVE_CHROMA_IBC_FOR_DUALTREE
  if (cu.cs->slice->getSPS()->getIBCFlag() && cu.chType != CHANNEL_TYPE_CHROMA)
#else
  if (cu.cs->slice->getSPS()->getIBCFlag())
#endif
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    if( cu.isConsInter() )
    {
      assert( CU::isInter( cu ) );
      return;
    }
#endif

#if JVET_O0050_LOCAL_DUAL_TREE
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
#else
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) )
#endif
    {
#if JVET_O1161_IBC_MAX_SIZE
      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
      if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
      {
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
      }
#if JVET_O0119_BASE_PALETTE_444
      if (!CU::isIBC(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
      {
        m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
#endif
    }
    else
    {
#if JVET_O0050_LOCAL_DUAL_TREE
      if( cu.isConsInter() )
      {
        return;
      }
#endif
#if JVET_O0119_BASE_PALETTE_444
      m_BinEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
      if (CU::isIntra(cu) || CU::isPLT(cu))
      {
        if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
          m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
      else
      {
#else
        m_BinEncoder.encodeBin((CU::isIntra(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
        if (!CU::isIntra(cu))
        {
#endif
#if JVET_O1161_IBC_MAX_SIZE
        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
        if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
        {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
        }
      }
    }
  }
  else
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    if( cu.isConsInter() )
    {
      assert( CU::isInter( cu ) );
      return;
    }
#endif

#if JVET_O0050_LOCAL_DUAL_TREE
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
#else
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) )
#endif
    {
#if JVET_O0119_BASE_PALETTE_444
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
        m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
#endif
      return;
    }
    m_BinEncoder.encodeBin((CU::isIntra(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
#if JVET_O0119_BASE_PALETTE_444
    if (!CU::isIntra(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
    {
      m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
    }
#endif
  }
}
void CABACWriter::bdpcm_mode( const CodingUnit& cu, const ComponentID compID )
{
#if JVET_O1136_TS_BDPCM_SIGNALLING
  if( !cu.cs->sps->getBDPCMEnabledFlag() ) return;
#endif
  if( !CU::bdpcmAllowed( cu, compID ) ) return;

  m_BinEncoder.encodeBin( cu.bdpcmMode > 0 ? 1 : 0, Ctx::BDPCMMode( 0 ) );

  if( cu.bdpcmMode )
  {
    m_BinEncoder.encodeBin( cu.bdpcmMode > 1 ? 1 : 0, Ctx::BDPCMMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "bdpcm_mode() x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode );
}
#if !JVET_O0525_REMOVE_PCM
void CABACWriter::pcm_data( const CodingUnit& cu, Partitioner& partitioner  )
{
  pcm_flag( cu, partitioner );
  if( cu.ipcm )
  {
    m_BinEncoder.pcmAlignBits();
    pcm_samples( *cu.firstTU );
  }
}

void CABACWriter::pcm_flag( const CodingUnit& cu, Partitioner& partitioner )
{
  const SPS& sps = *cu.cs->sps;
  if( !sps.getPCMEnabledFlag() || partitioner.currArea().lwidth() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lwidth() < (1 << sps.getPCMLog2MinSize())
      || partitioner.currArea().lheight() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lheight() < (1 << sps.getPCMLog2MinSize()) )
  {
    return;
  }
  m_BinEncoder.encodeBinTrm( cu.ipcm );
}
#endif


void CABACWriter::cu_pred_data( const CodingUnit& cu )
{
  if( CU::isIntra( cu ) )
  {
    intra_luma_pred_modes  ( cu );
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu );
  }

  imv_mode   ( cu );
  affine_amvr_mode( cu );

  cu_gbi_flag( cu );

}

void CABACWriter::cu_gbi_flag(const CodingUnit& cu)
{
  if(!CU::isGBiIdxCoded(cu))
  {
    return;
  }

  CHECK(!(GBI_NUM > 1 && (GBI_NUM == 2 || (GBI_NUM & 0x01) == 1)), " !( GBI_NUM > 1 && ( GBI_NUM == 2 || ( GBI_NUM & 0x01 ) == 1 ) ) ");
  const uint8_t gbiCodingIdx = (uint8_t)g_GbiCodingOrder[CU::getValidGbiIdx(cu)];

  const int32_t numGBi = (cu.slice->getCheckLDC()) ? 5 : 3;
#if JVET_O0126_BPWA_INDEX_CODING_FIX
  m_BinEncoder.encodeBin((gbiCodingIdx == 0 ? 0 : 1), Ctx::GBiIdx(0));
#else
  m_BinEncoder.encodeBin((gbiCodingIdx == 0 ? 1 : 0), Ctx::GBiIdx(0));
#endif
  if(numGBi > 2 && gbiCodingIdx != 0)
  {
    const uint32_t prefixNumBits = numGBi - 2;
    const uint32_t step = 1;

    uint8_t idx = 1;
    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (gbiCodingIdx == idx)
      {
#if JVET_O0126_BPWA_INDEX_CODING_FIX
        m_BinEncoder.encodeBinEP(0);
#else
        m_BinEncoder.encodeBinEP(1);
#endif
        break;
      }
      else
      {
#if JVET_O0126_BPWA_INDEX_CODING_FIX
        m_BinEncoder.encodeBinEP(1);
#else
        m_BinEncoder.encodeBinEP(0);
#endif
        idx += step;
      }
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_gbi_flag() gbi_idx=%d\n", cu.GBiIdx ? 1 : 0);
}

void CABACWriter::xWriteTruncBinCode(uint32_t symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  assert(val <= maxSymbol);
  assert((val << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  int b = maxSymbol - val;
  assert(b < val);
  if (symbol < val - b)
  {
    m_BinEncoder.encodeBinsEP(symbol, thresh);
  }
  else
  {
    symbol += val - b;
    assert(symbol < (val << 1));
    assert((symbol >> 1) >= val - b);
    m_BinEncoder.encodeBinsEP(symbol, thresh + 1);
  }
}

void CABACWriter::extend_ref_line(const PredictionUnit& pu)
{
#if !ENABLE_JVET_L0283_MRL
  return;
#endif

  const CodingUnit& cu = *pu.cu;
  if( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma( cu.chType ) || cu.bdpcmMode )
  {
    return;
  }
  bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
  if (isFirstLineOfCtu)
  {
    return;
  }
  int multiRefIdx = pu.multiRefIdx;
  if (MRL_NUM_REF_LINES > 1)
  {
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
    }
  }
}

void CABACWriter::extend_ref_line(const CodingUnit& cu)
{
#if !ENABLE_JVET_L0283_MRL
  return;
#endif

#if !JVET_O0525_REMOVE_PCM
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.ipcm || cu.bdpcmMode )
#else
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode )
#endif
  {
    return;
  }

  const int numBlocks = CU::getNumPUs(cu);
  const PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
    if (isFirstLineOfCtu)
    {
      return;
    }
    int multiRefIdx = pu->multiRefIdx;
    if (MRL_NUM_REF_LINES > 1)
    {
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
      }

    }
    pu = pu->next;
  }
}

void CABACWriter::intra_luma_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  if( cu.bdpcmMode )
  {
#if JVET_O0315_RDPCM_INTRAMODE_ALIGN
    cu.firstPU->intraDir[0] = cu.bdpcmMode == 2? VER_IDX : HOR_IDX;
#else
    PredictionUnit *pu = cu.firstPU;
    unsigned mpm_pred[NUM_MOST_PROBABLE_MODES];
    PU::getIntraMPMs( *pu, mpm_pred );
    cu.firstPU->intraDir[0] = mpm_pred[0];
#endif
    return;
  }

  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
  extend_ref_line( cu );

  isp_mode( cu );

  const int numMPMs   = NUM_MOST_PROBABLE_MODES;
  const int numBlocks = CU::getNumPUs( cu );
  unsigned  mpm_preds   [4][numMPMs];
  unsigned  mpm_idxs    [4];
  unsigned  ipred_modes [4];

  const PredictionUnit* pu = cu.firstPU;

  // prev_intra_luma_pred_flag
  for( int k = 0; k < numBlocks; k++ )
  {
    unsigned*  mpm_pred   = mpm_preds[k];
    unsigned&  mpm_idx    = mpm_idxs[k];
    unsigned&  ipred_mode = ipred_modes[k];

    PU::getIntraMPMs( *pu, mpm_pred );

    ipred_mode = pu->intraDir[0];
    mpm_idx    = numMPMs;
    for( unsigned idx = 0; idx < numMPMs; idx++ )
    {
      if( ipred_mode == mpm_pred[idx] )
      {
        mpm_idx = idx;
        break;
      }
    }
#if JVET_O0502_ISP_CLEANUP
    if ( pu->multiRefIdx )
#else
    if( pu->multiRefIdx || ( cu.ispMode && isLuma( cu.chType ) ) )
#endif
    {
      CHECK(mpm_idx >= numMPMs, "use of non-MPM");
    }
    else
    {
      m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
    }

    pu = pu->next;
  }

  pu = cu.firstPU;

  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    const unsigned& mpm_idx = mpm_idxs[k];
    if( mpm_idx < numMPMs )
    {
      {
        unsigned ctx = (pu->cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
        if (pu->multiRefIdx == 0)
          m_BinEncoder.encodeBin(mpm_idx > 0, Ctx::IntraLumaPlanarFlag(ctx));
        if( mpm_idx )
        {
          m_BinEncoder.encodeBinEP( mpm_idx > 1 );
        }
        if (mpm_idx > 1)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 2);
        }
        if (mpm_idx > 2)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 3);
        }
        if (mpm_idx > 3)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 4);
        }
      }
    }
    else
    {
      unsigned* mpm_pred   = mpm_preds[k];
      unsigned  ipred_mode = ipred_modes[k];

      // sorting of MPMs
      std::sort( mpm_pred, mpm_pred + numMPMs );

      {
        for (int idx = numMPMs - 1; idx >= 0; idx--)
        {
          if (ipred_mode > mpm_pred[idx])
          {
            ipred_mode--;
          }
        }
        CHECK(ipred_mode >= 64, "Incorrect mode");
        xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
      }
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}


void CABACWriter::intra_luma_pred_mode( const PredictionUnit& pu )
{

  if( pu.cu->bdpcmMode ) return;
  mip_flag(*pu.cu);
  if (pu.cu->mipFlag)
  {
    mip_pred_mode(pu);
    return;
  }
  extend_ref_line( pu );

  isp_mode( *pu.cu );

  // prev_intra_luma_pred_flag
  const int numMPMs  = NUM_MOST_PROBABLE_MODES;
  unsigned  mpm_pred[numMPMs];

  PU::getIntraMPMs( pu, mpm_pred );

  unsigned ipred_mode = pu.intraDir[0];
  unsigned mpm_idx = numMPMs;

  for( int idx = 0; idx < numMPMs; idx++ )
  {
    if( ipred_mode == mpm_pred[idx] )
    {
      mpm_idx = idx;
      break;
    }
  }
#if JVET_O0502_ISP_CLEANUP
  if ( pu.multiRefIdx )
#else
  if( pu.multiRefIdx || ( pu.cu->ispMode && isLuma( pu.cu->chType ) ) )
#endif
  {
    CHECK(mpm_idx >= numMPMs, "use of non-MPM");
  }
  else
  {
    m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
  }

  // mpm_idx / rem_intra_luma_pred_mode
  if( mpm_idx < numMPMs )
  {
    {
      unsigned ctx = (pu.cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
      if (pu.multiRefIdx == 0)
        m_BinEncoder.encodeBin( mpm_idx > 0, Ctx::IntraLumaPlanarFlag(ctx) );
      if( mpm_idx )
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 1 );
      }
      if (mpm_idx > 1)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 2);
      }
      if (mpm_idx > 2)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 3);
      }
      if (mpm_idx > 3)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 4);
      }
    }
  }
  else
  {
    std::sort( mpm_pred, mpm_pred + numMPMs );
    {
      for (int idx = numMPMs - 1; idx >= 0; idx--)
      {
        if (ipred_mode > mpm_pred[idx])
        {
          ipred_mode--;
        }
      }
      xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
    }
  }
}


void CABACWriter::intra_chroma_pred_modes( const CodingUnit& cu )
{
#if JVET_O0050_LOCAL_DUAL_TREE
  if( cu.chromaFormat == CHROMA_400 || ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_LUMA ) )
#else
  if( cu.chromaFormat == CHROMA_400 || ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_LUMA ) )
#endif
  {
    return;
  }

  const PredictionUnit* pu = cu.firstPU;

  intra_chroma_pred_mode( *pu );
}
#if JVET_O1153_INTRA_CHROMAMODE_CODING
void CABACWriter::intra_chroma_lmc_mode(const PredictionUnit& pu)
{
  const unsigned intraDir = pu.intraDir[1];
  int lmModeList[10];
  PU::getLMSymbolList(pu, lmModeList);
  int symbol = -1;
  for (int k = 0; k < LM_SYMBOL_NUM; k++)
  {
    if (lmModeList[k] == intraDir)
    {
      symbol = k;
      break;
    }
  }
  CHECK(symbol < 0, "invalid symbol found");

  m_BinEncoder.encodeBin(symbol == 0 ? 0 : 1, Ctx::IntraChromaPredMode(0));

  if (symbol > 0)
  {
    CHECK(symbol > 2, "invalid symbol for MMLM");
    unsigned int symbol_minus_1 = symbol - 1;
    m_BinEncoder.encodeBinEP(symbol_minus_1);
  }
}


void CABACWriter::intra_chroma_pred_mode(const PredictionUnit& pu)
{
  const unsigned intraDir = pu.intraDir[1];
#if JVET_O1124_ALLOW_CCLM_COND
  if (pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed())
#else
  if (pu.cs->sps->getUseLMChroma())
#endif
  {
    m_BinEncoder.encodeBin(PU::isLMCMode(intraDir) ? 1 : 0, Ctx::CclmModeFlag(0));
    if (PU::isLMCMode(intraDir))
    {
      intra_chroma_lmc_mode(pu);
      return;
    }
  }

  const bool     isDerivedMode = intraDir == DM_CHROMA_IDX;
  m_BinEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));
  if (isDerivedMode)
  {
    return;
  }

  // chroma candidate index
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(pu, chromaCandModes);

  int candId = 0;
  for (; candId < NUM_CHROMA_MODE; candId++)
  {
    if (intraDir == chromaCandModes[candId])
    {
      break;
    }
  }

  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
  {
    m_BinEncoder.encodeBinsEP(candId, 2);
  }
}
#else
void CABACWriter::intra_chroma_lmc_mode( const PredictionUnit& pu )
{
  const unsigned intraDir = pu.intraDir[1];
    int lmModeList[10];
    int maxSymbol = PU::getLMSymbolList( pu, lmModeList );
    int symbol    = -1;
    for ( int k = 0; k < LM_SYMBOL_NUM; k++ )
    {
      if ( lmModeList[k] == intraDir || ( lmModeList[k] == -1 && intraDir < LM_CHROMA_IDX ) )
      {
        symbol = k;
        break;
      }
    }
    CHECK( symbol < 0, "invalid symbol found" );

    unary_max_symbol(symbol, Ctx::IntraChromaPredMode(1), Ctx::IntraChromaPredMode(2), maxSymbol - 1);
}


void CABACWriter::intra_chroma_pred_mode( const PredictionUnit& pu )
{
  const unsigned intraDir = pu.intraDir[1];
  const bool     isDerivedMode = intraDir == DM_CHROMA_IDX;

  m_BinEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));

  if (isDerivedMode)
  {
    return;
  }

  // LM chroma mode
#if JVET_O1124_ALLOW_CCLM_COND
  if( pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed() )
#else
  if( pu.cs->sps->getUseLMChroma() )
#endif
  {
    intra_chroma_lmc_mode( pu );
    if ( PU::isLMCMode( intraDir ) )
    {
      return;
    }
  }

  // chroma candidate index
  unsigned chromaCandModes[ NUM_CHROMA_MODE ];
  PU::getIntraChromaCandModes( pu, chromaCandModes );

  int candId = 0;
  for ( ; candId < NUM_CHROMA_MODE; candId++ )
  {
    if( intraDir == chromaCandModes[ candId ] )
    {
      break;
    }
  }

  CHECK( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  CHECK( chromaCandModes[ candId ] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );
  {
    m_BinEncoder.encodeBinsEP( candId, 2 );
  }
}
#endif

void CABACWriter::cu_residual( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    PredictionUnit& pu = *cu.firstPU;
    if( !pu.mergeFlag )
    {
      rqt_root_cbf( cu );
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }

    if( !cu.rootCbf )
    {
      return;
    }
  }

#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA]   = false;
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
#endif
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  cuCtx.lfnstLastScanPos = false;
#endif

#if !JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  ChromaCbfs chromaCbfs;
#endif
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    transform_tree( *cu.cs, subTuPartitioner, cuCtx,             CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType)  ), 0 );
#else
    transform_tree( *cu.cs, subTuPartitioner, cuCtx, chromaCbfs, CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType ) ), 0 );
#endif
  }
  else
  {
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    transform_tree( *cu.cs, partitioner, cuCtx );
#else
    transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
#endif
  }

  residual_lfnst_mode( cu, cuCtx );
}

void CABACWriter::rqt_root_cbf( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( cu.rootCbf, Ctx::QtRootCbf() );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACWriter::sbt_mode( const CodingUnit& cu )
{
  uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();
  uint8_t sbtIdx = cu.getSbtIdx();
  uint8_t sbtPos = cu.getSbtPos();

  //bin - flag
  bool sbtFlag = cu.sbtInfo != 0;
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  m_BinEncoder.encodeBin( sbtFlag, Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  bool sbtQuadFlag = sbtIdx == SBT_HOR_QUAD || sbtIdx == SBT_VER_QUAD;
  bool sbtHorFlag = sbtIdx == SBT_HOR_HALF || sbtIdx == SBT_HOR_QUAD;
  bool sbtPosFlag = sbtPos == SBT_POS1;

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  //bin - type
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    m_BinEncoder.encodeBin( sbtQuadFlag, Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    assert( sbtQuadFlag == 0 );
  }

  //bin - dir
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    m_BinEncoder.encodeBin( sbtHorFlag, Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    assert( sbtHorFlag == ( ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow ) ) );
  }

  //bin - pos
  m_BinEncoder.encodeBin( sbtPosFlag, Ctx::SbtPosFlag( 0 ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}

void CABACWriter::end_of_ctu( const CodingUnit& cu, CUCtx& cuCtx )
{
  const Slice*  slice             = cu.cs->slice;
  const int     currentCTUTsAddr  = cu.cs->picture->brickMap->getCtuRsToBsAddrMap( CU::getCtuAddr( cu ) );
  const bool    isLastSubCUOfCtu  = CU::isLastSubCUOfCtu( cu );

  if ( isLastSubCUOfCtu
#if JVET_O0050_LOCAL_DUAL_TREE
    && ( !cu.isSepTree() || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
#else
    && ( !CS::isDualITree( *cu.cs ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
#endif
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    // i.e. when the slice segment CurEnd CTU address is the current CTU address+1.
    if(slice->getSliceCurEndCtuTsAddr() != currentCTUTsAddr + 1)
    {
      m_BinEncoder.encodeBinTrm( 0 );
    }
  }
}

#if JVET_O0119_BASE_PALETTE_444
void CABACWriter::cu_palette_info(const CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  const SPS&       sps = *(cu.cs->sps);
  TransformUnit&   tu = *cu.firstTU;
  uint32_t indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];

  if (cu.lastPLTSize[compBegin])
  {
    xEncodePLTPredIndicator(cu, MAXPLTSIZE, compBegin);
  }

  uint32_t reusedPLTnum = 0;
  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
      reusedPLTnum++;
  }

  if (reusedPLTnum < MAXPLTSIZE)
  {
    exp_golomb_eqprob(cu.curPLTSize[compBegin] - reusedPLTnum, 0);
  }

  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = cu.reusePLTSize[compBegin]; idx < cu.curPLTSize[compBegin]; idx++)
    {
      ComponentID compID = (ComponentID)comp;
      const int  channelBitDepth = sps.getBitDepth(toChannelType(compID));
      m_BinEncoder.encodeBinsEP(cu.curPLT[comp][idx], channelBitDepth);
    }
  }
  uint32_t signalEscape = (cu.useEscape[compBegin]) ? 1 : 0;
  if (cu.curPLTSize[compBegin] > 0)
  {
    m_BinEncoder.encodeBinEP(signalEscape);
  }
  //encode index map
  PLTtypeBuf runType = tu.getrunType(compBegin);
  PelBuf     runLength = tu.getrunLength(compBegin);
  PelBuf     curPLTIdx = tu.getcurPLTIdx(compBegin);
  uint32_t   height = cu.block(compBegin).height;
  uint32_t   width = cu.block(compBegin).width;

  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t total = height * width;
  int lastRunPos = -1;
  uint32_t lastRunType = 0;
  uint32_t numIndices = 0;
  std::vector<int> idxPos, parsedIdx;
  idxPos.reserve(total);
  parsedIdx.reserve(total);
  if (indexMaxSize > 1)
  {
    int idx = 0, run = 0;
    while (idx < total)
    {
      uint32_t posy = m_scanOrder[idx].y;
      uint32_t posx = m_scanOrder[idx].x;
      if (runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        idxPos.push_back(idx);
        numIndices++;
      }
      lastRunType = runType.at(posx, posy);
      lastRunPos = idx;
      run = runLength.at(posx, posy);
      idx += run;
    }
    uint32_t currParam = 3 + ((indexMaxSize) >> 3);
    uint32_t mappedValue;
    assert(numIndices);
    assert(numIndices > 0);
    mappedValue = numIndices - 1;
    m_BinEncoder.encodeRemAbsEP(mappedValue, currParam, false, MAX_NUM_CHANNEL_TYPE); // JC: code number of indices (PLT_RUN_INDEX)
    auto idxPosEnd = idxPos.end();
    for (auto iter = idxPos.begin(); iter != idxPosEnd; ++iter)
    {
      parsedIdx.push_back( writePLTIndex(cu, *iter, curPLTIdx, runType, indexMaxSize, compBegin));
    }
    m_BinEncoder.encodeBin(lastRunType, Ctx::RunTypeFlag());
    codeScanRotationModeFlag(cu, compBegin);
  }
  else
  {
    assert(!cu.useRotation[compBegin]);
  }

  if (cu.useEscape[compBegin] && cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    if (!cu.isSepTree() || isLuma(tu.chType))
#else
    if (!CS::isDualITree(*tu.cs) || isLuma(tu.chType))
#endif
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
#if JVET_O1168_CU_CHROMA_QP_OFFSET
  if ( cu.useEscape[compBegin] && cu.cs->slice->getUseChromaQpAdj() && !cuCtx.isChromaQpAdjCoded)
#else
  if (cu.cs->slice->getUseChromaQpAdj() && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded)
#endif
  {
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  uint32_t strPos = 0;
  uint32_t endPos = height * width;
  auto parsedIdxEnd = parsedIdx.end();
  auto parsedIdxIter = parsedIdx.begin();
  while (strPos < endPos)
  {
    uint32_t posy = m_scanOrder[strPos].y;
    uint32_t posx = m_scanOrder[strPos].x;
    uint32_t posyprev = strPos == 0 ? 0 : m_scanOrder[strPos - 1].y;
    uint32_t posxprev = strPos == 0 ? 0 : m_scanOrder[strPos - 1].x;

    if (indexMaxSize > 1)
    {
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))

      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else if (strPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else
      {
        if (numIndices && strPos < endPos - 1) // if numIndices (decoder will know this value) == 0 - > only CopyAbove, if strPos == endPos - 1, the last RunType was already coded
        {
          m_BinEncoder.encodeBin((runType.at(posx, posy)), Ctx::RunTypeFlag());
        }
      }
    }

    Pel curLevel = 0;
    if (runType.at(posx, posy) == PLT_RUN_INDEX)
    {
      if (parsedIdxIter != parsedIdxEnd)
      {
        curLevel = *parsedIdxIter++;
      }
      else
      {
        curLevel = 0;
      }
    }

    if (indexMaxSize > 1)
    {
      if (lastRunPos != strPos)
      {
        numIndices -= (runType.at(posx, posy) == PLT_RUN_INDEX);
        cu_run_val(runLength.at(posx, posy) - 1, (PLTRunMode)runType.at(posx, posy), curLevel, endPos - strPos - numIndices - 1 - lastRunType);
      }

    }

    strPos += (runLength.at(posx, posy));
  }
  assert(strPos == endPos);

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, sps.getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, sps.getChromaFormatIdc());
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    ComponentID compID = (ComponentID)comp;
    for (strPos = 0; strPos < endPos; strPos++)
    {
      uint32_t posy = m_scanOrder[strPos].y;
      uint32_t posx = m_scanOrder[strPos].x;
      if (curPLTIdx.at(posx, posy) == cu.curPLTSize[compBegin])
      {
        {
          PLTescapeBuf    escapeValue = tu.getescapeValue((ComponentID)comp);
          if (compID == COMPONENT_Y || compBegin != COMPONENT_Y)
          {
            exp_golomb_eqprob((unsigned)escapeValue.at(posx, posy), 3);
          }
          if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
          {
            uint32_t posxC = posx >> scaleX;
            uint32_t posyC = posy >> scaleY;
            exp_golomb_eqprob((unsigned)escapeValue.at(posxC, posyC), 3);
          }
        }
      }
    }
  }

}
void CABACWriter::codeScanRotationModeFlag(const CodingUnit& cu, ComponentID compBegin)
{
  m_BinEncoder.encodeBin((cu.useRotation[compBegin]), Ctx::RotationFlag());
}
void CABACWriter::xEncodePLTPredIndicator(const CodingUnit& cu, uint32_t maxPLTSize, ComponentID compBegin)
{
  int lastPredIdx = -1;
  uint32_t run = 0;
  uint32_t numPLTPredicted = 0;
  for (uint32_t idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      numPLTPredicted++;
      lastPredIdx = idx;
    }
  }

  int idx = 0;
  while (idx <= lastPredIdx)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      exp_golomb_eqprob(run ? run + 1 : run, 0);
      run = 0;
    }
    else
    {
      run++;
    }
    idx++;
  }
  if ((numPLTPredicted < maxPLTSize && lastPredIdx + 1 < cu.lastPLTSize[compBegin]) || !numPLTPredicted)
  {
    exp_golomb_eqprob(1, 0);
  }
}
Pel CABACWriter::writePLTIndex(const CodingUnit& cu, uint32_t idx, PelBuf& paletteIdx, PLTtypeBuf& paletteRunType, int maxSymbol, ComponentID compBegin)
{
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  Pel curLevel = (paletteIdx.at(posx, posy) == cu.curPLTSize[compBegin]) ? (maxSymbol - 1) : paletteIdx.at(posx, posy);
  if (idx) // R0348: remove index redundancy
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      Pel leftLevel = paletteIdx.at(prevposx, prevposy); // left index
      if (leftLevel == cu.curPLTSize[compBegin]) // escape mode
      {
        leftLevel = maxSymbol - 1;
      }
      assert(leftLevel != curLevel);
      if (curLevel > leftLevel)
      {
        curLevel--;
      }
    }
    else
    {
      Pel aboveLevel;
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        aboveLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin]) // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        aboveLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin]) // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      assert(curLevel != aboveLevel);
      if (curLevel > aboveLevel)
      {
        curLevel--;
      }
    }
    maxSymbol--;
  }
  assert(maxSymbol > 0);
  assert(curLevel >= 0);
  assert(maxSymbol > curLevel);
  if (maxSymbol > 1)
  {
    xWriteTruncBinCode(curLevel, maxSymbol);
  }
  return curLevel;
}

void CABACWriter::encodeRunType(const CodingUnit&  cu, PLTtypeBuf& runType, uint32_t idx, ScanElement *refScanOrder, ComponentID compBegin)
{
  if (refScanOrder)
  {
    m_scanOrder = refScanOrder;
  }
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  uint32_t posyprev = (idx == 0) ? 0 : m_scanOrder[idx - 1].y;
  uint32_t posxprev = (idx == 0) ? 0 : m_scanOrder[idx - 1].x;

  if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
  {
    assert(runType.at(posx, posy) == PLT_RUN_INDEX);
  }
  else if (idx != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
  {
    assert(runType.at(posx, posy) == PLT_RUN_INDEX);
  }
  else
  {
    m_BinEncoder.encodeBin((runType.at(posx, posy)), Ctx::RunTypeFlag());
  }
}

void CABACWriter::cu_run_val(uint32_t run, PLTRunMode runtype, const uint32_t paletteIdx, const uint32_t maxRun)
{
  if (runtype == PLT_RUN_COPY)
  {
  }
  else
  {
    g_paletteRunLeftLut[0] = (paletteIdx < PLT_RUN_MSB_IDX_CTX_T1 ? 0 : (paletteIdx < PLT_RUN_MSB_IDX_CTX_T2 ? 1 : 2));
  }
  xWriteTruncMsbP1RefinementBits(run, runtype, maxRun, PLT_RUN_MSB_IDX_CABAC_BYPASS_THRE);
}
uint32_t CABACWriter::xWriteTruncMsbP1(uint32_t symbol, PLTRunMode runtype, uint32_t uiMax, uint32_t uiCtxT)
{
  if (uiMax == 0)
    return 0;

  uint8_t *ctxLut;
  ctxLut = (runtype == PLT_RUN_INDEX) ? g_paletteRunLeftLut : g_paletteRunTopLut;

  uint32_t msbP1;
  for (msbP1 = 0; symbol > 0; msbP1++)
  {
    symbol >>= 1;
    if (msbP1 > uiCtxT)
    {
      m_BinEncoder.encodeBinEP(1);
    }
    else
      m_BinEncoder.encodeBin(1, (msbP1 <= uiCtxT)
        ? ((runtype == PLT_RUN_INDEX) ? Ctx::IdxRunModel(ctxLut[msbP1]) : Ctx::CopyRunModel(ctxLut[msbP1]))
        : ((runtype == PLT_RUN_INDEX) ? Ctx::IdxRunModel(ctxLut[uiCtxT]) : Ctx::CopyRunModel(ctxLut[uiCtxT])));
  }
  assert(msbP1 <= uiMax);
  if (msbP1 < uiMax)
  {
    if (msbP1 > uiCtxT)
    {
      m_BinEncoder.encodeBinEP(0);
    }
    else
      m_BinEncoder.encodeBin(0, msbP1 <= uiCtxT
        ? ((runtype == PLT_RUN_INDEX) ? Ctx::IdxRunModel(ctxLut[msbP1]) : Ctx::CopyRunModel(ctxLut[msbP1]))
        : ((runtype == PLT_RUN_INDEX) ? Ctx::IdxRunModel(ctxLut[uiCtxT]) : Ctx::CopyRunModel(ctxLut[uiCtxT])));

    //m_pcBinIf->encodeBin(0, msbP1 <= uiCtxT? pcSCModel[ctxLut[msbP1]] : pcSCModel[ctxLut[uiCtxT]]);
  }
  return msbP1;
}

void CABACWriter::xWriteTruncMsbP1RefinementBits(uint32_t symbol, PLTRunMode runtype, uint32_t maxVal, uint32_t uiCtxT)
{
  if (maxVal == 0)
    return;

  uint32_t msbP1 = xWriteTruncMsbP1(symbol, runtype, floorLog2(maxVal) + 1, uiCtxT);
  if (msbP1 > 1)
  {
    uint32_t numBins = floorLog2(maxVal) + 1;
    if (msbP1 < numBins)
    {

      uint32_t bits = msbP1 - 1;
      m_BinEncoder.encodeBinsEP(symbol & ((1 << bits) - 1), bits);
    }
    else
    {
      uint32_t curValue = 1 << (numBins - 1);
      xWriteTruncBinCode(symbol - curValue, maxVal + 1 - curValue);
    }
  }
}

#endif

//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu );
//    void  merge_flag      ( pu );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACWriter::prediction_unit( const PredictionUnit& pu )
{
#if JVET_O0050_LOCAL_DUAL_TREE
  CHECK( pu.cu->treeType == TREE_C, "cannot be chroma CU" );
#endif
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  CHECK( pu.cacheUsed, "Processing a PU that should be in cache!" );
  CHECK( pu.cu->cacheUsed, "Processing a CU that should be in cache!" );

#endif
  if( pu.cu->skip )
  {
    CHECK( !pu.mergeFlag, "merge_flag must be true for skipped CUs" );
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
#if JVET_O0249_MERGE_SYNTAX
    merge_data(pu);
#else
    if (CU::isIBC(*pu.cu))
    {
      merge_idx(pu);
      return;
    }
    if (pu.regularMergeFlag)
    {
      merge_idx(pu);
    }
    else
    {
      subblock_merge_flag( *pu.cu );
      MHIntra_flag( pu );
      if (!pu.mhIntraFlag)
      {
        if (!pu.cu->affine && !pu.mmvdMergeFlag && !pu.cu->mmvdSkip)
        {
          CHECK(!pu.cu->triangle, "triangle_flag must be true");
        }
      }
      if (pu.mmvdMergeFlag)
      {
        mmvd_merge_idx(pu);
      }
      else
        merge_idx    ( pu );
    }
#endif
  }
  else if (CU::isIBC(*pu.cu))
  {
    ref_idx(pu, REF_PIC_LIST_0);
    Mv mvd = pu.mvd[REF_PIC_LIST_0];
    mvd.changeIbcPrecInternal2Amvr(pu.cu->imv);
    mvd_coding(mvd, 0); // already changed to signaling precision
#if JVET_O0162_IBC_MVP_FLAG
#if JVET_O0455_IBC_MAX_MERGE_NUM
    if ( pu.cu->slice->getMaxNumIBCMergeCand() == 1 )
#else
    if ( pu.cu->slice->getMaxNumMergeCand() == 1 )
#endif
    {
      CHECK( pu.mvpIdx[REF_PIC_LIST_0], "mvpIdx for IBC mode should be 0" );
    }
    else
#endif
    mvp_flag(pu, REF_PIC_LIST_0);
  }
  else
  {
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
    smvd_mode( pu );
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_0 );
      if ( pu.cu->affine )
      {
        Mv mvd = pu.mvdAffi[REF_PIC_LIST_0][0];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
        mvd = pu.mvdAffi[REF_PIC_LIST_0][1];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd = pu.mvdAffi[REF_PIC_LIST_0][2];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
        }
      }
      else
      {
        Mv mvd = pu.mvd[REF_PIC_LIST_0];
        mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }
    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      if ( pu.cu->smvdMode != 1 )
      {
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( !pu.cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
      {
        if ( pu.cu->affine )
        {
          Mv mvd = pu.mvdAffi[REF_PIC_LIST_1][0];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
          mvd = pu.mvdAffi[REF_PIC_LIST_1][1];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
          if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
          {
            mvd = pu.mvdAffi[REF_PIC_LIST_1][2];
            mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
            mvd_coding(mvd, 0); // already changed to signaling precision
          }
        }
        else
        {
          Mv mvd = pu.mvd[REF_PIC_LIST_1];
          mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
        }
      }
      }
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
  }
}

void CABACWriter::smvd_mode( const PredictionUnit& pu )
{
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }

  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }

  m_BinEncoder.encodeBin( pu.cu->smvdMode ? 1 : 0, Ctx::SmvdFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

void CABACWriter::subblock_merge_flag( const CodingUnit& cu )
{
#if !JVET_O0249_MERGE_SYNTAX
  if ( cu.firstPU->mergeFlag && (cu.firstPU->mmvdMergeFlag || cu.mmvdSkip) )
  {
    return;
  }
#endif

#if JVET_O0220_METHOD1_SUBBLK_FLAG_PARSING
  if ( !cu.cs->slice->isIntra() && (cu.slice->getMaxNumAffineMergeCand() > 0) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
#else
  if ( !cu.cs->slice->isIntra() && (cu.cs->sps->getUseAffine() || cu.cs->sps->getSBTMVPEnabledFlag()) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
#endif
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
#if JVET_O0500_SEP_CTX_AFFINE_SUBBLOCK_MRG
    m_BinEncoder.encodeBin( cu.affine, Ctx::SubblockMergeFlag( ctxId ) );
#else
    m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
#endif
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACWriter::affine_flag( const CodingUnit& cu )
{
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      unsigned ctxId = 0;
      m_BinEncoder.encodeBin( cu.affineType, Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
  }
}

void CABACWriter::merge_flag( const PredictionUnit& pu )
{
  m_BinEncoder.encodeBin( pu.mergeFlag, Ctx::MergeFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

#if !JVET_O0249_MERGE_SYNTAX
  if (pu.mergeFlag && CU::isIBC(*pu.cu))
  {
    return;
  }
  if (pu.mergeFlag)
  {
    if (!pu.cs->sps->getUseMMVD() && (pu.lwidth() * pu.lheight() == 32))
    {
      CHECK(!pu.regularMergeFlag, "regular_merge_flag must be true!");
    }
    else
    {
      m_BinEncoder.encodeBin(pu.regularMergeFlag, Ctx::RegularMergeFlag(1));
      DTRACE(g_trace_ctx, D_SYNTAX, "regularMergeFlag() ctx=%d regularMergeFlag=%d\n", 1, pu.regularMergeFlag?1:0);
    }
    if (pu.cs->sps->getUseMMVD())
    {
      bool isCUWithOnlyRegularAndMMVD=((pu.lwidth() == 8 && pu.lheight() == 4) || (pu.lwidth() == 4 && pu.lheight() == 8));
      if (isCUWithOnlyRegularAndMMVD)
      {
        CHECK(pu.mmvdMergeFlag==pu.regularMergeFlag, "mmvdMergeFlag must be !regularMergeFlag");
      }
      else if (!pu.regularMergeFlag)
      {
        m_BinEncoder.encodeBin(pu.mmvdMergeFlag, Ctx::MmvdFlag(0));
        DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
      }
    }
  }
#endif
}

#if JVET_O0249_MERGE_SYNTAX
void CABACWriter::merge_data(const PredictionUnit& pu)
{
  if (CU::isIBC(*pu.cu))
  {
    merge_idx(pu);
    return;
  }
  subblock_merge_flag(*pu.cu);
  if (pu.cu->affine)
  {
    merge_idx(pu);
    return;
  }
  const bool triangleAvailable = pu.cu->cs->slice->getSPS()->getUseTriangle() && pu.cu->cs->slice->isInterB() && pu.cu->cs->slice->getMaxNumTriangleCand() > 1;
  const bool ciipAvailable = pu.cs->sps->getUseMHIntra() && !pu.cu->skip && pu.cu->lwidth() < MAX_CU_SIZE && pu.cu->lheight() < MAX_CU_SIZE;
  if (pu.cu->lwidth() * pu.cu->lheight() >= 64
    && (triangleAvailable || ciipAvailable))
  {
    m_BinEncoder.encodeBin(pu.regularMergeFlag, Ctx::RegularMergeFlag(pu.cu->skip ? 0 : 1));
  }
  if (pu.regularMergeFlag)
  {
    if (pu.cs->sps->getUseMMVD())
    {
      m_BinEncoder.encodeBin(pu.mmvdMergeFlag, Ctx::MmvdFlag(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
    }
    if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
    {
      mmvd_merge_idx(pu);
    }
    else
    {
      merge_idx(pu);
    }
  }
  else
  {
    if (triangleAvailable && ciipAvailable)
    {
      MHIntra_flag(pu);
    }
    merge_idx(pu);
  }
}
#endif

void CABACWriter::imv_mode( const CodingUnit& cu )
{
  const SPS *sps = cu.cs->sps;

  if( !sps->getAMVREnabledFlag() )
  {
    return;
  }
  if ( cu.affine )
  {
    return;
  }

  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  if (CU::isIBC(cu) == false)
    m_BinEncoder.encodeBin( (cu.imv > 0), Ctx::ImvFlag( 0 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 0), 0 );

  if( sps->getAMVREnabledFlag() && cu.imv > 0 )
  {
#if JVET_O0057_ALTHPELIF
    if (!CU::isIBC(cu))
    {
      m_BinEncoder.encodeBin(cu.imv < IMV_HPEL, Ctx::ImvFlag(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", cu.imv < 3, 4);
    }
    if (cu.imv < IMV_HPEL)
    {
#endif
    m_BinEncoder.encodeBin( (cu.imv > 1), Ctx::ImvFlag( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 1), 1 );
#if JVET_O0057_ALTHPELIF
    }
#endif
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACWriter::affine_amvr_mode( const CodingUnit& cu )
{
  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  m_BinEncoder.encodeBin( (cu.imv > 0), Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 0), 2 );

  if( cu.imv > 0 )
  {
    m_BinEncoder.encodeBin( (cu.imv > 1), Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 1), 3 );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

void CABACWriter::merge_idx( const PredictionUnit& pu )
{

  if ( pu.cu->affine )
  {
    int numCandminus1 = int( pu.cs->slice->getMaxNumAffineMergeCand() ) - 1;
    if ( numCandminus1 > 0 )
    {
      if ( pu.mergeIdx == 0 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::AffMergeIdx() );
        DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
        return;
      }
      else
      {
        m_BinEncoder.encodeBin( 1, Ctx::AffMergeIdx() );
        for ( unsigned idx = 1; idx < numCandminus1; idx++ )
        {
            m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
          if ( pu.mergeIdx == idx )
          {
            break;
          }
        }
      }
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
  }
  else
  {
    if( pu.cu->triangle )
    {
      bool    splitDir = pu.triangleSplitDir;
      uint8_t candIdx0 = pu.triangleMergeIdx0;
      uint8_t candIdx1 = pu.triangleMergeIdx1;
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_split_dir=%d\n", splitDir );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx0=%d\n", candIdx0 );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx1=%d\n", candIdx1 );
      candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;
      auto encodeOneIdx = [this](uint8_t mrgIdx, int numCandminus1)
      {
        if (numCandminus1 == 0)
        {
          CHECK(mrgIdx, "Incorrect index!");
          return;
        }
        if(mrgIdx == 0)
        {
          this->m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
          return;
        }
        else
        {
          this->m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
          for( unsigned idx = 1; idx < numCandminus1; idx++ )
          {
            this->m_BinEncoder.encodeBinEP( mrgIdx == idx ? 0 : 1 );
            if( mrgIdx == idx )
            {
              break;
            }
          }
        }
      };
      m_BinEncoder.encodeBinEP(splitDir);
      const int maxNumTriangleCand = pu.cs->slice->getMaxNumTriangleCand();
      CHECK(maxNumTriangleCand < 2, "Incorrect max number of triangle candidates");
      CHECK(candIdx0 >= maxNumTriangleCand, "Incorrect candIdx0");
      CHECK(candIdx1 >= maxNumTriangleCand, "Incorrect candIdx1");
      encodeOneIdx(candIdx0, maxNumTriangleCand - 1);
      encodeOneIdx(candIdx1, maxNumTriangleCand - 2);
      return;
    }
#if JVET_O0455_IBC_MAX_MERGE_NUM
    int numCandminus1;
    if (pu.cu->predMode == MODE_IBC)
      numCandminus1 = int(pu.cs->slice->getMaxNumIBCMergeCand()) - 1;
    else
      numCandminus1 = int(pu.cs->slice->getMaxNumMergeCand()) - 1;
#else
    int numCandminus1 = int(pu.cs->slice->getMaxNumMergeCand()) - 1;
#endif
  if( numCandminus1 > 0 )
  {
    if( pu.mergeIdx == 0 )
    {
      m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
      for( unsigned idx = 1; idx < numCandminus1; idx++ )
      {
          m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
        if( pu.mergeIdx == idx )
        {
          break;
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
  }
}
void CABACWriter::mmvd_merge_idx(const PredictionUnit& pu)
{
  int var0, var1, var2;
  int mvpIdx = pu.mmvdMergeIdx;
  var0 = mvpIdx / MMVD_MAX_REFINE_NUM;
  var1 = (mvpIdx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
  var2 = mvpIdx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;

  if (pu.cs->slice->getMaxNumMergeCand() > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    assert(var0 < 2);
    m_BinEncoder.encodeBin(var0, Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);

  int numCandminus1_step = MMVD_REFINE_STEP - 1;
  if (numCandminus1_step > 0)
  {
    if (var1 == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::MmvdStepMvpIdx());
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::MmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numCandminus1_step; idx++)
      {
        m_BinEncoder.encodeBinEP(var1 == idx ? 0 : 1);
        if (var1 == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);

  m_BinEncoder.encodeBinsEP(var2, 2);

  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
}
void CABACWriter::inter_pred_idc( const PredictionUnit& pu )
{
  if( !pu.cs->slice->isInterB() )
  {
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( pu.interDir == 3 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::InterDir(ctxId) );
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 0, Ctx::InterDir(ctxId) );
    }
  }
  m_BinEncoder.encodeBin( ( pu.interDir == 2 ), Ctx::InterDir( 4 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
}


void CABACWriter::ref_idx( const PredictionUnit& pu, RefPicList eRefList )
{
  if ( pu.cu->smvdMode )
  {
    CHECK( pu.refIdx[eRefList] != pu.cs->slice->getSymRefIdx( eRefList ), "Invalid reference index!\n" );
    return;
  }

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

  if (eRefList == REF_PIC_LIST_0 && pu.cs->sps->getIBCFlag())
  {
    if (CU::isIBC(*pu.cu))
      return;
  }

  if( numRef <= 1 )
  {
    return;
  }
  int refIdx  = pu.refIdx[eRefList];
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::RefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::RefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      break;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
}

void CABACWriter::mvp_flag( const PredictionUnit& pu, RefPicList eRefList )
{
  m_BinEncoder.encodeBin( pu.mvpIdx[eRefList], Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", pu.mvpIdx[eRefList], pu.lumaPos().x, pu.lumaPos().y );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, pu.mvpIdx[eRefList] );
}

void CABACWriter::MHIntra_flag(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseMHIntra())
  {
    CHECK(pu.mhIntraFlag == true, "invalid MHIntra SPS");
    return;
  }
  if (pu.cu->skip)
  {
    CHECK(pu.mhIntraFlag == true, "invalid MHIntra and skip");
    return;
  }
#if !JVET_O0249_MERGE_SYNTAX
  if (pu.mmvdMergeFlag)
  {
    CHECK(pu.mhIntraFlag == true, "invalid MHIntra and mmvd");
    return;
  }
  if (pu.cu->affine)
  {
    CHECK(pu.mhIntraFlag == true, "invalid MHIntra and affine");
    return;
  }
  if (pu.cu->lwidth() * pu.cu->lheight() < 64 || pu.cu->lwidth() >= MAX_CU_SIZE || pu.cu->lheight() >= MAX_CU_SIZE)
  {
    CHECK(pu.mhIntraFlag == true, "invalid MHIntra and blk");
    return;
  }
#endif
  m_BinEncoder.encodeBin(pu.mhIntraFlag, Ctx::MHIntraFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "MHIntra_flag() MHIntra=%d pos=(%d,%d) size=%dx%d\n", pu.mhIntraFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
}



#if !JVET_O0525_REMOVE_PCM
//================================================================================
//  clause 7.3.8.7
//--------------------------------------------------------------------------------
//    void  pcm_samples( tu )
//================================================================================

void CABACWriter::pcm_samples( const TransformUnit& tu )
{
  CHECK( !tu.cu->ipcm, "pcm mode expected" );

  const SPS&        sps       = *tu.cu->cs->sps;
#if !JVET_O0050_LOCAL_DUAL_TREE
  const CodingStructure *cs = tu.cs;
#endif
  const ChannelType chType = tu.chType;

#if JVET_O0050_LOCAL_DUAL_TREE
  ComponentID compStr = (tu.cu->isSepTree() && !isLuma( chType )) ? COMPONENT_Cb : COMPONENT_Y;
  ComponentID compEnd = (tu.cu->isSepTree() && isLuma( chType )) ? COMPONENT_Y : COMPONENT_Cr;
#else
  ComponentID compStr = (CS::isDualITree(*cs) && !isLuma(chType)) ? COMPONENT_Cb: COMPONENT_Y;
  ComponentID compEnd = (CS::isDualITree(*cs) && isLuma(chType)) ? COMPONENT_Y : COMPONENT_Cr;
#endif
  for( ComponentID compID = compStr; compID <= compEnd; compID = ComponentID(compID+1) )
  {
    const CPelBuf   samples     = tu.getPcmbuf( compID );
    const unsigned  sampleBits  = sps.getPCMBitDepth( toChannelType(compID) );
    for( unsigned y = 0; y < samples.height; y++ )
    {
      for( unsigned x = 0; x < samples.width; x++ )
      {
        m_BinEncoder.encodeBinsPCM( samples.at(x, y), sampleBits );
      }
    }
  }
  m_BinEncoder.restart();
}

#endif


//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx,                         const PartSplit ispType, const int subTuIdx )
#else
void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs, const PartSplit ispType, const int subTuIdx )
#endif
{
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  const UnitArea&       area = partitioner.currArea();
  int             subTuCounter = subTuIdx;
  const TransformUnit&  tu = *cs.getTU(area.blocks[partitioner.chType].pos(), partitioner.chType, subTuIdx);
  const CodingUnit&     cu = *tu.cu;
  const unsigned        trDepth = partitioner.currTrDepth;
  const bool            split = (tu.depth > trDepth);
#else
  ChromaCbfs chromaCbfsLastDepth;
  chromaCbfsLastDepth.Cb              = chromaCbfs.Cb;
  chromaCbfsLastDepth.Cr              = chromaCbfs.Cr;
  const UnitArea&       area          = partitioner.currArea();
        int             subTuCounter  = subTuIdx;
  const TransformUnit&  tu            = *cs.getTU( area.blocks[partitioner.chType].pos(), partitioner.chType, subTuIdx );
  const CodingUnit&     cu            = *tu.cu;
  const unsigned        trDepth       = partitioner.currTrDepth;
  const bool            split         = ( tu.depth > trDepth );
  const bool            chromaCbfISP  = area.blocks[COMPONENT_Cb].valid() && cu.ispMode && !split;
  bool max_tu_split = false;
#endif

  // split_transform_flag
  if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
  {
#if !JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    max_tu_split = true;
#endif
    CHECK( !split, "transform split implied" );
  }
  else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
  {
    CHECK( !split, "transform split implied - sbt" );
  }
  else
  CHECK( split && !cu.ispMode, "transform split not allowed with QTBT" );

#if !JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  // cbf_cb & cbf_cr
#if JVET_O0050_LOCAL_DUAL_TREE
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA ) && ( !cu.ispMode || chromaCbfISP ) )
#else
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) && ( !cu.ispMode || chromaCbfISP ) )
#endif
  {
    {
      unsigned cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
      if (!max_tu_split || chromaCbfISP)
      {
        chromaCbfs.Cb = TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth );
        if( !( cu.sbtInfo && trDepth == 1 ) )
        cbf_comp( cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], cbfDepth );
      }

      if (!max_tu_split || chromaCbfISP)
      {
        chromaCbfs.Cr = TU::getCbfAtDepth( tu, COMPONENT_Cr, trDepth );
        if( !( cu.sbtInfo && trDepth == 1 ) )
        cbf_comp( cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb );
      }
    }
  }
#if JVET_O0050_LOCAL_DUAL_TREE
  else if( cu.isSepTree() )
#else
  else if( CS::isDualITree( cs ) )
#endif
  {
    chromaCbfs = ChromaCbfs( false );
  }
#endif

  if( split )
  {
#if !JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    if( area.chromaFormat != CHROMA_400 )
    {
      chromaCbfs.Cb        = TU::getCbfAtDepth( tu, COMPONENT_Cb,  trDepth );
      chromaCbfs.Cr        = TU::getCbfAtDepth( tu, COMPONENT_Cr,  trDepth );
    }
#endif

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n", partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( cu.ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
    {
      partitioner.splitCurrArea( PartSplit( cu.getSbtTuSplit() ), cs );
    }
    else
      THROW( "Implicit TU split not available" );

    do
    {
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
      transform_tree( cs, partitioner, cuCtx,                ispType, subTuCounter );
#else
      ChromaCbfs subChromaCbfs = chromaCbfs;
      transform_tree( cs, partitioner, cuCtx, subChromaCbfs, ispType, subTuCounter );
#endif
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    transform_unit( tu, cuCtx, partitioner, subTuCounter);
#else
    if( !isChroma( partitioner.chType ) )
    {
      if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        CHECK( !TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be true for inter units with no chroma coeffs" );
      }
      else if( cu.sbtInfo && tu.noResidual )
      {
        CHECK( TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be false for inter sbt no-residual tu" );
      }
      else if( cu.sbtInfo && !chromaCbfsLastDepth.sigChroma( area.chromaFormat ) )
      {
        assert( !tu.noResidual );
        CHECK( !TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be true for inter sbt residual tu" );
      }
      else
      {
        bool previousCbf       = false;
        bool rootCbfSoFar      = false;
        bool lastCbfIsInferred = false;
        if( cu.ispMode )
        {
          uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
          if( subTuCounter == nTus - 1 )
          {
            TransformUnit* tuPointer = cu.firstTU;
            for( int tuIdx = 0; tuIdx < subTuCounter; tuIdx++ )
            {
              rootCbfSoFar |= TU::getCbfAtDepth( *tuPointer, COMPONENT_Y, trDepth );
              tuPointer = tuPointer->next;
            }
            if( !rootCbfSoFar )
            {
              lastCbfIsInferred = true;
            }
          }
          if( !lastCbfIsInferred )
          {
            previousCbf = TU::getPrevTuCbfAtDepth( tu, COMPONENT_Y, partitioner.currTrDepth );
          }
        }
        if( !lastCbfIsInferred )
        {
          cbf_comp( cs, TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), tu.Y(), trDepth, previousCbf, cu.ispMode );
        }
      }
    }


    transform_unit( tu, cuCtx, chromaCbfs );
#endif
  }
}

void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
#if JVET_O0193_REMOVE_TR_DEPTH_IN_CBF_CTX
  const unsigned  ctxId   = DeriveCtx::CtxQtCbf( area.compID, prevCbf, useISP && isLuma(area.compID) );
#else
  const unsigned  ctxId   = DeriveCtx::CtxQtCbf( area.compID, depth, prevCbf, useISP && isLuma(area.compID) );
#endif
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID ];
  if( area.compID == COMPONENT_Y && cs.getCU( area.pos(), ChannelType( area.compID ) )->bdpcmMode )
  {
#if JVET_O0193_REMOVE_TR_DEPTH_IN_CBF_CTX
    m_BinEncoder.encodeBin( cbf, ctxSet( 1 ) );
#else
    m_BinEncoder.encodeBin( cbf, ctxSet( 4 ) );
#endif
  }
  else
  {
  m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================
void CABACWriter::mvd_coding( const Mv &rMvd, int8_t imv )
{
  int       horMvd = rMvd.getHor();
  int       verMvd = rMvd.getVer();
  if ( imv > 0 )
  {
#if JVET_O0057_ALTHPELIF
    CHECK((horMvd % 2) != 0 && (verMvd % 2) != 0, "IMV: MVD is not a multiple of 2");
    horMvd >>= 1;
    verMvd >>= 1;
    if (imv < IMV_HPEL)
    {
      CHECK((horMvd % 2) != 0 && (verMvd % 2) != 0, "IMV: MVD is not a multiple of 4");
      horMvd >>= 1;
      verMvd >>= 1;
      if (imv == IMV_4PEL)//IMV_4PEL
      {
        CHECK((horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 16");
        horMvd >>= 2;
        verMvd >>= 2;
      }
    }
#else
    CHECK( (horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 4" );
    horMvd >>= 2;
    verMvd >>= 2;
    if( imv == 2 )//IMV_4PEL
    {
      CHECK( (horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 16" );
      horMvd >>= 2;
      verMvd >>= 2;
    }
#endif
  }
  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );


  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_BinEncoder.encodeBin( (horAbs > 0), Ctx::Mvd() );
  m_BinEncoder.encodeBin( (verAbs > 0), Ctx::Mvd() );

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
  }
  if( verAbs > 0 )
  {
    m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    if( horAbs > 1 )
    {
      exp_golomb_eqprob( horAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (horMvd < 0) );
  }
  if( verAbs > 0 )
  {
    if( verAbs > 1 )
    {
      exp_golomb_eqprob( verAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (verMvd < 0) );
  }
}




//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
void CABACWriter::transform_unit( const TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter)
#else
void CABACWriter::transform_unit( const TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
#endif
{
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  const CodingStructure&  cs = *tu.cs;
  const CodingUnit&       cu = *tu.cu;
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;
  const bool              chromaCbfISP = area.blocks[COMPONENT_Cb].valid() && cu.ispMode;
  ChromaCbfs              chromaCbfs;
  CHECK(tu.depth != trDepth, " transform unit should be not be futher partitioned");

  // cbf_cb & cbf_cr
#if JVET_O0050_LOCAL_DUAL_TREE
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#else
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!CS::isDualITree(cs) || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#endif
  {
    {
      unsigned cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
      {
        chromaCbfs.Cb = TU::getCbfAtDepth(tu, COMPONENT_Cb, trDepth);
        //if (!(cu.sbtInfo && trDepth == 1))
        if (!(cu.sbtInfo && tu.noResidual))
          cbf_comp(cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], cbfDepth);
      }

      {
        chromaCbfs.Cr = TU::getCbfAtDepth(tu, COMPONENT_Cr, trDepth);
        //if (!(cu.sbtInfo && trDepth == 1))
        if (!(cu.sbtInfo && tu.noResidual))
          cbf_comp(cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb);
      }
    }
  }
#if JVET_O0050_LOCAL_DUAL_TREE
  else if (cu.isSepTree())
#else
  else if (CS::isDualITree(cs))
#endif
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      CHECK(!TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be true for inter units with no chroma coeffs");
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      CHECK(TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be false for inter sbt no-residual tu");
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      CHECK(!TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be true for inter sbt residual tu");
    }
    else
    {
      bool previousCbf = false;
      bool rootCbfSoFar = false;
      bool lastCbfIsInferred = false;
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < subTuCounter; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, trDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMPONENT_Y, partitioner.currTrDepth);
        }
      }
      if (!lastCbfIsInferred)
      {
        cbf_comp(cs, TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), tu.Y(), trDepth, previousCbf, cu.ispMode);
      }
    }
  }
#else
  CodingUnit& cu        = *tu.cu;
#endif
  bool        lumaOnly  = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbf[3]    = { TU::getCbf( tu, COMPONENT_Y ), chromaCbfs.Cb, chromaCbfs.Cr };
  bool        cbfLuma   = ( cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma = false;

  if( !lumaOnly )
  {
    if( tu.blocks[COMPONENT_Cb].valid() )
    {
      cbf   [ COMPONENT_Cb  ] = TU::getCbf( tu, COMPONENT_Cb );
      cbf   [ COMPONENT_Cr  ] = TU::getCbf( tu, COMPONENT_Cr );
    }
    cbfChroma = ( cbf[ COMPONENT_Cb ] || cbf[ COMPONENT_Cr ] );
  }

#if JVET_O0046_DQ_SIGNALLING
  if( ( cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma ) &&
#else
  if( ( cbfLuma || cbfChroma ) &&
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
    (!tu.cu->isSepTree() || isLuma(tu.chType)) )
#else
    (!CS::isDualITree(*tu.cs) || isLuma(tu.chType)) )
#endif
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
#if JVET_O1168_CU_CHROMA_QP_OFFSET
    if (cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cuCtx.isChromaQpAdjCoded)
#else
    if( cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded )
#endif
    {
      cu_chroma_qp_offset( cu );
      cuCtx.isChromaQpAdjCoded = true;
    }

#if JVET_O0105_ICT
  if( !lumaOnly )
  {
    joint_cb_cr( tu, ( cbf[COMPONENT_Cb] ? 2 : 0 ) + ( cbf[COMPONENT_Cr] ? 1 : 0 ) );
  }
#endif

    if( cbfLuma )
    {
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
      residual_coding( tu, COMPONENT_Y, &cuCtx );
#else
      residual_coding( tu, COMPONENT_Y );
#endif
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( TU::hasCrossCompPredInfo( tu, compID ) )
        {
          cross_comp_pred( tu, compID );
        }
        if( cbf[ compID ] )
        {
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
          residual_coding( tu, compID, &cuCtx );
#else
          residual_coding( tu, compID );
#endif
      }
    }
  }
}

void CABACWriter::cu_qp_delta( const CodingUnit& cu, int predQP, const int8_t qp )
{
  CHECK(!( predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int       DQp         = qp - predQP;
  int       qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
  DQp                   = ( DQp + (MAX_QP + 1) + (MAX_QP + 1) / 2 + qpBdOffsetY + (qpBdOffsetY / 2)) % ((MAX_QP + 1) + qpBdOffsetY) - (MAX_QP + 1) / 2 - (qpBdOffsetY / 2);
  unsigned  absDQP      = unsigned( DQp < 0 ? -DQp : DQp );
  unsigned  unaryDQP    = std::min<unsigned>( absDQP, CU_DQP_TU_CMAX );

  unary_max_symbol( unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( absDQP >= CU_DQP_TU_CMAX )
  {
    exp_golomb_eqprob( absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }
  if( absDQP > 0 )
  {
    m_BinEncoder.encodeBinEP( DQp < 0 );
  }

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACWriter::cu_chroma_qp_offset( const CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if( qpAdj == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ChromaQpAdjFlag() );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ChromaQpAdjFlag() );
    int length = cu.cs->pps->getChromaQpOffsetListLen();
    if( length > 1 )
    {
      unary_max_symbol( qpAdj-1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
    }
  }
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        explicit_rdpcm_mode     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

#if JVET_O0105_ICT
void CABACWriter::joint_cb_cr( const TransformUnit& tu, const int cbfMask )
{
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  if ( !tu.cu->slice->getSPS()->getJointCbCrEnabledFlag() )
  {
    return;
  }
#endif

  CHECK( tu.jointCbCr && tu.jointCbCr != cbfMask, "wrong value of jointCbCr (" << (int)tu.jointCbCr << " vs " << (int)cbfMask << ")" );
#if JVET_O0543_ICT_ICU_ONLY
  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
#else
  if( cbfMask )
#endif
  {
    m_BinEncoder.encodeBin( tu.jointCbCr ? 1 : 0, Ctx::JointCbCrFlag( cbfMask - 1 ) );
  }
}
#else
void CABACWriter::joint_cb_cr( const TransformUnit& tu )
{
  m_BinEncoder.encodeBin( tu.jointCbCr ? 1 : 0, Ctx::JointCbCrFlag( 0 ) );
}
#endif

#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID, CUCtx* cuCtx )
#else
void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID)
#endif
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

#if JVET_O0105_ICT
  if( compID == COMPONENT_Cr && tu.jointCbCr == 3 )
    return;
#else
  // Joint Cb-Cr residual mode is signalled if both Cb and Cr cbfs are true
  if ( compID == COMPONENT_Cr && TU::getCbf( tu, COMPONENT_Cb ) )
  {
    joint_cb_cr( tu );

    // No Cr residual in bitstream in joint Cb-Cr residual mode
    if ( tu.jointCbCr )
      return;
  }
#endif

  // code transform skip and explicit rdpcm mode
  mts_coding         ( tu, compID );
  explicit_rdpcm_mode( tu, compID );

  if( isLuma( compID ) && ( tu.mtsIdx == MTS_SKIP || tu.cu->bdpcmMode ) )
  {
    residual_codingTS( tu, compID );
    return;
  }

  // determine sign hiding
  bool signHiding  = ( cu.cs->slice->getSignDataHidingEnabledFlag() && !cu.transQuantBypass && tu.rdpcm[compID] == RDPCM_OFF );
  if(  signHiding && CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && tu.mtsIdx==MTS_SKIP )
  {
    const ChannelType chType    = toChannelType( compID );
    const unsigned    intraMode = PU::getFinalIntraMode( *cu.cs->getPU( tu.blocks[compID].pos(), chType ), chType );
    if( intraMode == HOR_IDX || intraMode == VER_IDX )
    {
      signHiding = false;
    }
  }

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;

  // determine and set last coeff position and sig group flags
  int                      scanPosLast = -1;
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      scanPosLast = scanPos;
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }
  CHECK( scanPosLast < 0, "Coefficient coding called for empty TU" );
  cctx.setScanPosLast(scanPosLast);

#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS
  if( cuCtx && tu.mtsIdx != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
    cuCtx->violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
#endif
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  if( cuCtx && tu.mtsIdx != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx->lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
#endif
  // code last coeff position
  last_sig_coeff( cctx, tu, compID );

  // code subblocks
  const int stateTab  = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
  int       state     = 0;

#if JVET_O0052_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT
  int ctxBinSampleRatio = (compID == COMPONENT_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;
#endif

  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock       ( subSetId, sigGroupFlags[subSetId] );
#if JVET_O0538_SPS_CONTROL_ISP_SBT
    if( ( tu.mtsIdx > MTS_SKIP || ( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
    if( ( tu.mtsIdx > MTS_SKIP || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
    {
      if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) )
       || ( tu.blocks[ compID ].width  == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth()  ) ) )
      {
        continue;
      }
    }
    residual_coding_subblock( cctx, coeff, stateTab, state );

  }


}

void CABACWriter::mts_coding( const TransformUnit& tu, ComponentID compID )
{
#if !JVET_O0294_TRANSFORM_CLEANUP
  const CodingUnit  &cu = *tu.cu;
#endif
  const bool  tsAllowed = TU::isTSAllowed ( tu, compID );
  const bool mtsAllowed = TU::isMTSAllowed( tu, compID );

  if( !mtsAllowed && !tsAllowed ) return;

  int symbol  = 0;
  int ctxIdx  = 0;

  if( tsAllowed )
  {
#if JVET_O0294_TRANSFORM_CLEANUP
    symbol = (tu.mtsIdx == MTS_SKIP) ? 1 : 0;
#else
    symbol = (tu.mtsIdx == MTS_SKIP) ? 0 : 1;
#endif
    ctxIdx = 6;
    m_BinEncoder.encodeBin( symbol, Ctx::MTSIndex( ctxIdx ) );
  }

  if( tu.mtsIdx != MTS_SKIP )
  {
    if( mtsAllowed )
    {
      symbol = tu.mtsIdx != MTS_DCT2_DCT2 ? 1 : 0;
#if JVET_O0294_TRANSFORM_CLEANUP
      ctxIdx = 0;
#else
      ctxIdx = std::min( (int)cu.qtDepth, 5 );
#endif
      m_BinEncoder.encodeBin( symbol, Ctx::MTSIndex( ctxIdx ) );

      if( symbol )
      {
        ctxIdx = 7;
        for( int i = 0; i < 3; i++, ctxIdx++ )
        {
          symbol = tu.mtsIdx > i + MTS_DST7_DST7 ? 1 : 0;
          m_BinEncoder.encodeBin( symbol, Ctx::MTSIndex( ctxIdx ) );

          if( !symbol )
          {
            break;
          }
        }
      }
    }
  }
#if JVET_O0294_TRANSFORM_CLEANUP
  DTRACE( g_trace_ctx, D_SYNTAX, "mts_coding() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), tu.mtsIdx);
#else
  DTRACE( g_trace_ctx, D_SYNTAX, "mts_coding() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), tu.mtsIdx );
#endif
}

void CABACWriter::isp_mode( const CodingUnit& cu )
{
#if !JVET_O0525_REMOVE_PCM
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || cu.ipcm || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) )
#else
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) )
#endif
  {
    CHECK( cu.ispMode != NOT_INTRA_SUBPARTITIONS, "cu.ispMode != 0" );
    return;
  }
  if ( cu.ispMode == NOT_INTRA_SUBPARTITIONS )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ISPMode( 0 ) );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ISPMode( 0 ) );
    m_BinEncoder.encodeBin( cu.ispMode - 1, Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACWriter::explicit_rdpcm_mode( const TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;
  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.mtsIdx==MTS_SKIP || cu.transQuantBypass ) )
  {
    ChannelType chType = toChannelType( compID );
    switch( tu.rdpcm[compID] )
    {
    case RDPCM_VER:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmDir (chType) );
      break;
    case RDPCM_HOR:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmDir (chType) );
      break;
    default: // RDPCM_OFF
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmFlag(chType) );
    }
  }
}

void CABACWriter::residual_lfnst_mode( const CodingUnit& cu, CUCtx& cuCtx )
{
#if JVET_O0213_RESTRICT_LFNST_TO_MAX_TB_SIZE
  int chIdx = CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#endif
  if( cu.ispMode != NOT_INTRA_SUBPARTITIONS ||
#if JVET_O0925_MIP_SIMPLIFICATIONS
      (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize())) ||
#else
      cu.mipFlag == true ||
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
    ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
#else
    ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
#endif
#if JVET_O0213_RESTRICT_LFNST_TO_MAX_TB_SIZE
#if JVET_O0545_MAX_TB_SIGNALLING
    || ( cu.blocks[ chIdx ].lumaSize().width > cu.cs->sps->getMaxTbSize() || cu.blocks[ chIdx ].lumaSize().height > cu.cs->sps->getMaxTbSize() )
#else
    || ( cu.blocks[ chIdx ].lumaSize().width > MAX_TB_SIZEY || cu.blocks[ chIdx ].lumaSize().height > MAX_TB_SIZEY )
#endif
#endif
    )
  {
    return;
  }

  if( cu.cs->sps->getUseLFNST() && CU::isIntra( cu ) && !CU::isLosslessCoded( cu ) )
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    const bool lumaFlag                   = cu.isSepTree() ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag                 = cu.isSepTree() ? ( isChroma( cu.chType ) ? true : false ) : true;
#else
    const bool lumaFlag                   = CS::isDualITree( *cu.cs ) ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag                 = CS::isDualITree( *cu.cs ) ? ( isChroma( cu.chType ) ? true : false ) : true;
#endif
#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
          bool nonZeroCoeffNonTs;
#endif
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS
          bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] );
#else
          bool nonZeroCoeffNonTsCorner8x8 = CU::getNumNonZeroCoeffNonTsCorner8x8( cu, lumaFlag, chromaFlag ) > 0;
#endif
#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
#if JVET_O0050_LOCAL_DUAL_TREE
    const int  nonZeroCoeffThr            = cu.isSepTree() ? ( isLuma( cu.chType ) ? LFNST_SIG_NZ_LUMA : LFNST_SIG_NZ_CHROMA ) : LFNST_SIG_NZ_LUMA + LFNST_SIG_NZ_CHROMA;
#else
    const int  nonZeroCoeffThr            = CS::isDualITree( *cu.cs ) ? ( isLuma( cu.chType ) ? LFNST_SIG_NZ_LUMA : LFNST_SIG_NZ_CHROMA ) : LFNST_SIG_NZ_LUMA + LFNST_SIG_NZ_CHROMA;
#endif
    cuCtx.numNonZeroCoeffNonTs            = CU::getNumNonZeroCoeffNonTs( cu, lumaFlag, chromaFlag );
    nonZeroCoeffNonTs                     = cuCtx.numNonZeroCoeffNonTs > nonZeroCoeffThr;
#endif
#if JVET_O0368_LFNST_WITH_DCT2_ONLY
    const bool isNonDCT2 = (TU::getCbf(*cu.firstTU, ComponentID(COMPONENT_Y)) && cu.firstTU->mtsIdx != MTS_DCT2_DCT2);
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
    if( !cuCtx.lfnstLastScanPos || nonZeroCoeffNonTsCorner8x8 || isNonDCT2 )
#else
    if (!nonZeroCoeffNonTs || nonZeroCoeffNonTsCorner8x8 || isNonDCT2 )
#endif
#else
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
    if( !cuCtx.lfnstLastScanPos || nonZeroCoeffNonTsCorner8x8 )
#else
    if( !nonZeroCoeffNonTs || nonZeroCoeffNonTsCorner8x8 )
#endif
#endif
    {
      return;
    }
  }
  else
  {
    return;
  }


  unsigned cctx = 0;
#if JVET_O0368_LFNST_WITH_DCT2_ONLY
#if JVET_O0050_LOCAL_DUAL_TREE
  if ( cu.isSepTree() ) cctx++;
#else
  if ( CS::isDualITree(*cu.cs) ) cctx++;
#endif
#else
#if JVET_O0545_MAX_TB_SIGNALLING
#if JVET_O0050_LOCAL_DUAL_TREE
  if( ( cu.firstTU->mtsIdx < MTS_DST7_DST7 || !TU::getCbf(*cu.firstTU, COMPONENT_Y) ) && cu.isSepTree() ) cctx++;
#else
  if( ( cu.firstTU->mtsIdx < MTS_DST7_DST7 || !TU::getCbf(*cu.firstTU, COMPONENT_Y) ) && CS::isDualITree( *cu.cs ) ) cctx++;
#endif
#else
#if JVET_O0050_LOCAL_DUAL_TREE
  if( cu.firstTU->mtsIdx < MTS_DST7_DST7 && cu.isSepTree() ) cctx++;
#else
  if( cu.firstTU->mtsIdx < MTS_DST7_DST7 && CS::isDualITree( *cu.cs ) ) cctx++;
#endif
#endif
#endif

  const uint32_t idxLFNST = cu.lfnstIdx;
  assert( idxLFNST < 3 );
  m_BinEncoder.encodeBin( idxLFNST ? 1 : 0, Ctx::LFNSTIdx( cctx ) );

  if( idxLFNST )
  {
    m_BinEncoder.encodeBinEP( ( idxLFNST - 1 ) ? 1 : 0 );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}

void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx, const TransformUnit& tu, ComponentID compID )
{
  unsigned blkPos = cctx.blockPos( cctx.scanPosLast() );
  unsigned posX, posY;
  {
    posY  = blkPos / cctx.width();
    posX  = blkPos - ( posY * cctx.width() );
  }

  unsigned CtxLast;
  unsigned GroupIdxX = g_uiGroupIdx[ posX ];
  unsigned GroupIdxY = g_uiGroupIdx[ posY ];

  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

#if JVET_O0538_SPS_CONTROL_ISP_SBT
  if( ( tu.mtsIdx > MTS_SKIP || ( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
  if( ( tu.mtsIdx > MTS_SKIP || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
  {
    maxLastPosX = ( tu.blocks[compID].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[compID].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }

  for( CtxLast = 0; CtxLast < GroupIdxX; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastXCtxId( CtxLast ) );
  }
  if( GroupIdxX < maxLastPosX )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastXCtxId( CtxLast ) );
  }
  for( CtxLast = 0; CtxLast < GroupIdxY; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxY < maxLastPosY )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxX > 3 )
  {
    posX -= g_uiMinInGroup[ GroupIdxX ];
    for (int i = ( ( GroupIdxX - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posX >> i ) & 1 );
    }
  }
  if( GroupIdxY > 3 )
  {
    posY -= g_uiMinInGroup[ GroupIdxY ];
    for ( int i = ( ( GroupIdxY - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posY >> i ) & 1 );
    }
  }
}



void CABACWriter::residual_coding_subblock( CoeffCodingContext& cctx, const TCoeff* coeff, const int stateTransTable, int& state )
{
  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  uint8_t   ctxOffset[16];

  //===== encode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;
  unsigned  signPattern   =  0;
#if JVET_O0052_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT
  int       remRegBins    = cctx.regBinLimit;
#else
  bool      is2x2subblock = ( cctx.log2CGSize() == 2 );
  int       remRegBins    = ( is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK );
#endif
  int       firstPosMode2 = minSubPos - 1;

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      m_BinEncoder.encodeBin( sigFlag, sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
      cctx.sigCtxIdAbs( nextSigPos, coeff, state ); // required for setting variables that are needed for gtx/par context selection
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff  = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff            = cctx.ctxOffsetAbs();
      numNonZero++;
      firstNZPos  = nextSigPos;
      lastNZPos   = std::max<int>( lastNZPos, nextSigPos );
      remAbsLevel = abs( Coeff ) - 1;

      if( nextSigPos != cctx.scanPosLast() ) signPattern <<= 1;
      if( Coeff < 0 )                        signPattern++;

      unsigned gt1 = !!remAbsLevel;
      m_BinEncoder.encodeBin( gt1, cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      if( gt1 )
      {
        remAbsLevel  -= 1;
        m_BinEncoder.encodeBin( remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        remAbsLevel >>= 1;

        remRegBins--;
        unsigned gt2 = !!remAbsLevel;
        m_BinEncoder.encodeBin(gt2, cctx.greater2CtxIdAbs(ctxOff));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2, cctx.greater2CtxIdAbs(ctxOff));
        remRegBins--;
      }
    }

    state = ( stateTransTable >> ((state<<2)+((Coeff&1)<<1)) ) & 3;
  }
  firstPosMode2 = nextSigPos;
#if JVET_O0052_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT
  cctx.regBinLimit = remRegBins;
#endif


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
    ricePar = g_auiGoRiceParsCoeff[sumAll];
    unsigned absLevel = abs( coeff[ cctx.blockPos( scanPos ) ] );
    if( absLevel >= 4 )
    {
      unsigned rem      = ( absLevel - 4 ) >> 1;
      m_BinEncoder.encodeRemAbsEP( rem, ricePar, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    TCoeff    Coeff     = coeff[ cctx.blockPos( scanPos ) ];
    unsigned  absLevel  = abs( Coeff );
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0[std::max(0, state - 1)][sumAll];
    unsigned  rem       = ( absLevel == 0 ? pos0 : absLevel <= pos0 ? absLevel-1 : absLevel );
    m_BinEncoder.encodeRemAbsEP( rem, rice, cctx.extPrec(), cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    state = ( stateTransTable >> ((state<<2)+((absLevel&1)<<1)) ) & 3;
    if( absLevel )
    {
      numNonZero++;
      firstNZPos = scanPos;
      lastNZPos   = std::max<int>( lastNZPos, scanPos );
      signPattern <<= 1;
      if( Coeff < 0 ) signPattern++;
    }
  }

  //===== encode sign's =====
  unsigned numSigns = numNonZero;
  if( cctx.hideSign( firstNZPos, lastNZPos ) )
  {
    numSigns    --;
    signPattern >>= 1;
  }
  m_BinEncoder.encodeBinsEP( signPattern, numSigns );
}

void CABACWriter::residual_codingTS( const TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, tu.cu->bdpcmMode );
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;

  cctx.setNumCtxBins( 2 * tu.lwidth()*tu.lheight() );

  // determine and set last coeff position and sig group flags
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }

  // code subblocks
  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId, sigGroupFlags[subSetId] );
    residual_coding_subblockTS( cctx, coeff );
  }
}

void CABACWriter::residual_coding_subblockTS( CoeffCodingContext& cctx, const TCoeff* coeff )
{
  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !cctx.isLastSubSet() || !cctx.only1stSigGroup() )
  {
    if( cctx.isSigGroup() )
    {
#if !JVET_O0409_EXCLUDE_CODED_SUB_BLK_FLAG_FROM_COUNT
      if( cctx.isContextCoded() )
      {
#endif
        m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 1, cctx.sigGroupCtxId() );
#if !JVET_O0409_EXCLUDE_CODED_SUB_BLK_FLAG_FROM_COUNT
      }
      else
      {
        m_BinEncoder.encodeBinEP( 1 );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() EPbin=%d\n", 1 );
      }
#endif
    }
    else
    {
#if !JVET_O0409_EXCLUDE_CODED_SUB_BLK_FLAG_FROM_COUNT
      if( cctx.isContextCoded() )
      {
#endif
        m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 0, cctx.sigGroupCtxId() );
#if !JVET_O0409_EXCLUDE_CODED_SUB_BLK_FLAG_FROM_COUNT
      }
      else
      {
        m_BinEncoder.encodeBinEP( 0 );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() EPbin=%d\n", 0 );
      }
#endif
      return;
    }
  }

  //===== encode absolute values =====
  const int inferSigPos   = minSubPos;
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;

#if JVET_O0122_TS_SIGN_LEVEL
  int rightPixel, belowPixel, modAbsCoeff;
#endif

  for( ; nextSigPos <= minSubPos; nextSigPos++ )
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
      if( cctx.isContextCoded() )
      {
        const unsigned sigCtxId = cctx.sigCtxIdAbsTS( nextSigPos, coeff );
        m_BinEncoder.encodeBin( sigFlag, sigCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      }
      else
      {
        m_BinEncoder.encodeBinEP( sigFlag );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() EPbin=%d\n", sigFlag );
      }
    }

    if( sigFlag )
    {
      //===== encode sign's =====
      int sign = Coeff < 0;
      if( cctx.isContextCoded() )
      {
#if JVET_O0122_TS_SIGN_LEVEL
        const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
        m_BinEncoder.encodeBin(sign, signCtxId);
#else
        m_BinEncoder.encodeBin( sign, Ctx::TsResidualSign( cctx.bdpcm() ? 1 : 0 ) );
#endif
      }
      else
      {
        m_BinEncoder.encodeBinEP( sign );
      }
      numNonZero++;
#if JVET_O0122_TS_SIGN_LEVEL
      cctx.neighTS(rightPixel, belowPixel, nextSigPos, coeff);
      modAbsCoeff = cctx.deriveModCoeff(rightPixel, belowPixel, abs(Coeff), cctx.bdpcm());
      remAbsLevel = modAbsCoeff - 1;
#else
      remAbsLevel = abs( Coeff ) - 1;
#endif

      unsigned gt1 = !!remAbsLevel;
#if JVET_O0122_TS_SIGN_LEVEL
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      if (cctx.isContextCoded())
      {
        m_BinEncoder.encodeBin(gt1, gt1CtxId);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1, gt1CtxId);
      }
      else
      {
        m_BinEncoder.encodeBinEP(gt1);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() EPbin=%d\n", gt1);
      }
#else
      if( cctx.isContextCoded() )
      {
        m_BinEncoder.encodeBin( gt1, cctx.greaterXCtxIdAbsTS(0) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1, cctx.greaterXCtxIdAbsTS(0) );
      }
      else
      {
        m_BinEncoder.encodeBinEP( gt1 );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() EPbin=%d\n", gt1 );
      }
#endif

      if( gt1 )
      {
        remAbsLevel  -= 1;
        if( cctx.isContextCoded() )
        {
          m_BinEncoder.encodeBin( remAbsLevel&1, cctx.parityCtxIdAbsTS() );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbsTS() );
        }
        else
        {
          m_BinEncoder.encodeBinEP( remAbsLevel&1 );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() EPbin=%d\n", remAbsLevel&1 );
        }
      }
    }
  }

  int cutoffVal = 2;
  int numGtBins = 4;

#if JVET_O0619_GTX_SINGLE_PASS_TS_RESIDUAL_CODING
  for (int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++)
  {
#if JVET_O0122_TS_SIGN_LEVEL
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm());
#else
    unsigned absLevel = abs(coeff[cctx.blockPos(scanPos)]);
#endif
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if (absLevel >= cutoffVal)
      {
        unsigned gt2 = (absLevel >= (cutoffVal + 2));
        if (cctx.isContextCoded())
        {
          m_BinEncoder.encodeBin(gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
          DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, min<int>(absLevel, cutoffVal + 2));
        }
        else
        {
          m_BinEncoder.encodeBinEP(gt2);
          DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() EPbin=%d sp=%d coeff=%d\n", i, gt2, scanPos, min<int>(absLevel, cutoffVal + 2));
        }
      }
      cutoffVal += 2;
    }
  }
#else
  for( int i = 0; i < numGtBins; i++ )
  {
    for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
    {
#if JVET_O0122_TS_SIGN_LEVEL
      unsigned absLevel;
      cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
      absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm());
#else
      unsigned absLevel = abs( coeff[cctx.blockPos( scanPos )] );
#endif
      if( absLevel >= cutoffVal )
      {
        unsigned gt2 = ( absLevel >= ( cutoffVal + 2 ) );
        if( cctx.isContextCoded() )
        {
          m_BinEncoder.encodeBin( gt2, cctx.greaterXCtxIdAbsTS( cutoffVal>>1 ) );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2, cctx.greaterXCtxIdAbsTS( cutoffVal>>1 ), scanPos, min<int>( absLevel, cutoffVal+2 ) );
        }
        else
        {
          m_BinEncoder.encodeBinEP( gt2 );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() EPbin=%d sp=%d coeff=%d\n", i, gt2, scanPos, min<int>( absLevel, cutoffVal+2 ) );
        }
      }
    }
    cutoffVal += 2;
  }
#endif

  //===== coeff bypass ====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
#if JVET_O0122_TS_SIGN_LEVEL
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm());
#else
    TCoeff    Coeff     = coeff[ cctx.blockPos( scanPos ) ];
    unsigned  absLevel  = abs( Coeff );
#endif
    if( absLevel >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      unsigned  rem  = ( absLevel - cutoffVal ) >> 1;
      m_BinEncoder.encodeRemAbsEP( rem, rice, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );
    }
  }
}





//================================================================================
//  clause 7.3.8.12
//--------------------------------------------------------------------------------
//    void  cross_comp_pred( tu, compID )
//================================================================================

void CABACWriter::cross_comp_pred( const TransformUnit& tu, ComponentID compID )
{
  CHECK(!( !isLuma( compID ) ), "Unspecified error");
  signed char alpha   = tu.compAlpha[compID];
  unsigned    ctxBase = ( compID == COMPONENT_Cr ? 5 : 0 );
  if( alpha == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred( ctxBase ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
    return;
  }

  static const unsigned log2AbsAlphaMinus1Table[8] = { 0, 1, 1, 2, 2, 2, 3, 3 };
  unsigned sign = ( alpha < 0 );
  if( sign )
  {
    alpha = -alpha;
  }
  CHECK(!( alpha <= 8 ), "Unspecified error");
  m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase) );
  if( alpha > 1)
  {
     m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase+1) );
     unary_max_symbol( log2AbsAlphaMinus1Table[alpha-1]-1, Ctx::CrossCompPred(ctxBase+2), Ctx::CrossCompPred(ctxBase+3), 2 );
  }
  else
  {
     m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred(ctxBase+1) );
  }
  m_BinEncoder.encodeBin( sign, Ctx::CrossCompPred(ctxBase+4) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
}




//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol )
{
  CHECK( symbol > maxSymbol, "symbol > maxSymbol" );
  const unsigned totalBinsToWrite = std::min( symbol + 1, maxSymbol );
  for( unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten )
  {
    const unsigned nextBin = symbol > binsWritten;
    m_BinEncoder.encodeBin( nextBin, binsWritten == 0 ? ctxId0 : ctxIdN );
  }
}


void CABACWriter::unary_max_eqprob( unsigned symbol, unsigned maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }
  bool     codeLast = ( maxSymbol > symbol );
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while( symbol-- )
  {
    bins   <<= 1;
    bins   ++;
    numBins++;
  }
  if( codeLast )
  {
    bins  <<= 1;
    numBins++;
  }
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::exp_golomb_eqprob( unsigned symbol, unsigned count )
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while( symbol >= (unsigned)(1<<count) )
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  //CHECK(!( numBins + count <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP(bins, numBins);
  m_BinEncoder.encodeBinsEP(symbol, count);
}

void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isLuma( channel ) )
  {
    if (alfParam->enabledFlag[COMPONENT_Y])
      codeAlfCtuEnableFlags( cs, COMPONENT_Y, alfParam );
  }
  else
  {
    if (alfParam->enabledFlag[COMPONENT_Cb])
      codeAlfCtuEnableFlags( cs, COMPONENT_Cb, alfParam );
    if (alfParam->enabledFlag[COMPONENT_Cr])
      codeAlfCtuEnableFlags( cs, COMPONENT_Cr, alfParam );
  }
}
void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  uint32_t numCTUs = cs.pcv->sizeInCtus;

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    codeAlfCtuEnableFlag( cs, ctuIdx, compID, alfParam );
  }
}

void CABACWriter::codeAlfCtuEnableFlag( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfParam* alfParam)
{
  const bool alfComponentEnabled = (alfParam != NULL) ? alfParam->enabledFlag[compIdx] : cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx);

  if( cs.sps->getALFEnabledFlag() && alfComponentEnabled )
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
    const uint32_t      curTileIdx = cs.picture->brickMap->getBrickIdxRsMap( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
    int ctx = 0;
    ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
    ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;
    m_BinEncoder.encodeBin( ctbAlfFlag[ctuRsAddr], Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );
  }
}

void CABACWriter::code_unary_fixed( unsigned symbol, unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  bool unary = (symbol <= unary_max);
  m_BinEncoder.encodeBin( unary, ctxId );
  if( unary )
  {
    unary_max_eqprob( symbol, unary_max );
  }
  else
  {
    m_BinEncoder.encodeBinsEP( symbol - unary_max - 1, fixed );
  }
}

void CABACWriter::mip_flag( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->getUseMIP() )
  {
    return;
  }
#if JVET_O0545_MAX_TB_SIGNALLING
  if( cu.lwidth() > cu.cs->sps->getMaxTbSize() || cu.lheight() > cu.cs->sps->getMaxTbSize())
#else
  if( cu.lwidth() > MIP_MAX_WIDTH || cu.lheight() > MIP_MAX_HEIGHT )
#endif
  {
    return;
  }
  if( !mipModesAvailable( cu.Y() ) )
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  m_BinEncoder.encodeBin( cu.mipFlag, Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACWriter::mip_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  for( const auto &pu : CU::traversePUs( cu ) )
  {
    mip_pred_mode( pu );
  }
}

void CABACWriter::mip_pred_mode( const PredictionUnit& pu )
{
  const int numModes = getNumModesMip( pu.Y() ); CHECKD( numModes > MAX_NUM_MIP_MODE, "Error: too many MIP modes" );

#if JVET_O0925_MIP_SIMPLIFICATIONS
  xWriteTruncBinCode( pu.intraDir[CHANNEL_TYPE_LUMA], numModes );
#else
  // derive modeIdx from true MIP mode
  unsigned mpm[NUM_MPM_MIP];
  PU::getMipMPMs(pu, mpm);

  unsigned mipMode = pu.intraDir[CHANNEL_TYPE_LUMA];
  unsigned mpmIdx   = NUM_MPM_MIP;
  for( auto k = 0; k < NUM_MPM_MIP; k++ )
  {
    if( mipMode == mpm[k] )
    {
      mpmIdx = k;
      break;
    }
  }

  unsigned modeIdx;
  if (mpmIdx < NUM_MPM_MIP)
  {
    modeIdx = mpmIdx;
  }
  else
  {
    std::sort( mpm, mpm + NUM_MPM_MIP);

    modeIdx = mipMode;
    for( auto k = (NUM_MPM_MIP - 1); k >= 0; k-- )
    {
      if( modeIdx > mpm[k] )
      {
        modeIdx--;
      }
    }
    CHECK( modeIdx >= (1<<getNumEpBinsMip( pu.Y() )), "Incorrect mode" );
    modeIdx += NUM_MPM_MIP;
  }

  CHECK( modeIdx >= numModes, "modeIdx out of range" );
  int unaryMax = NUM_MPM_MIP - 1;
  int fixedLength = getNumEpBinsMip( pu.Y() );
  code_unary_fixed( modeIdx, Ctx::MipMode( 0 ), unaryMax, fixedLength );
#endif

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA] );
}

void CABACWriter::codeAlfCtuFilterIndex(CodingStructure& cs, uint32_t ctuRsAddr, bool alfEnableLuma)
{
  if ( (!cs.sps->getALFEnabledFlag()) || (!alfEnableLuma))
  {
    return;
  }

  uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag(COMPONENT_Y);
  if (!ctbAlfFlag[ctuRsAddr])
  {
    return;
  }

  short* alfCtbFilterIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  const unsigned filterSetIdx = alfCtbFilterIndex[ctuRsAddr];
  unsigned numAps = cs.slice->getTileGroupNumAps();
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    int useLatestFilt = (filterSetIdx == NUM_FIXED_FILTER_SETS) ? 1 : 0;
    m_BinEncoder.encodeBin(useLatestFilt, Ctx::AlfUseLatestFilt());
    if (!useLatestFilt)
    {

      if (numAps == 1)
      {
        CHECK(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set numavail < num_fixed");
        xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
      }
      else
      {
        int useTemporalFilt = (filterSetIdx > NUM_FIXED_FILTER_SETS) ? 1 : 0;
        m_BinEncoder.encodeBin(useTemporalFilt, Ctx::AlfUseTemporalFilt());

        if (useTemporalFilt)
        {
          CHECK((filterSetIdx - (NUM_FIXED_FILTER_SETS + 1)) >= (numAvailableFiltSets - (NUM_FIXED_FILTER_SETS + 1)), "temporal non-latest set");
#if JVET_O0247_ALF_CTB_CODING_REDUNDANCY_REMOVAL
          if (numAps > 2)
          {
#endif
            xWriteTruncBinCode(filterSetIdx - (NUM_FIXED_FILTER_SETS + 1), numAvailableFiltSets - (NUM_FIXED_FILTER_SETS + 1));
#if JVET_O0247_ALF_CTB_CODING_REDUNDANCY_REMOVAL
          }
#endif
        }
        else
        {
          CHECK(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set larger than temporal");
          xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
        }
      }
    }
  }
  else
  {
    CHECK(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set numavail < num_fixed");
    xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
  }
}
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isChroma( channel ) )
  {
    if (alfParam->enabledFlag[COMPONENT_Cb])
      codeAlfCtuAlternatives( cs, COMPONENT_Cb, alfParam );
    if (alfParam->enabledFlag[COMPONENT_Cr])
      codeAlfCtuAlternatives( cs, COMPONENT_Cr, alfParam );
  }
}
void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  if( compID == COMPONENT_Y )
    return;
  uint32_t numCTUs = cs.pcv->sizeInCtus;
  uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compID );

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    if( ctbAlfFlag[ctuIdx] )
    {
      codeAlfCtuAlternative( cs, ctuIdx, compID, alfParam );
    }
  }
}

void CABACWriter::codeAlfCtuAlternative( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, const AlfParam* alfParam)
{
  if( compIdx == COMPONENT_Y )
    return;
  int apsIdx = alfParam ? 0 : cs.slice->getTileGroupApsIdChroma();
  const AlfParam& alfParamRef = alfParam ? (*alfParam) : cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();

  if( alfParam || (cs.sps->getALFEnabledFlag() && cs.slice->getTileGroupAlfEnabledFlag( (ComponentID)compIdx )) )
  {
    uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );

    if( ctbAlfFlag[ctuRsAddr] )
    {
      const int numAlts = alfParamRef.numAlternativesChroma;
      uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData( compIdx );
      unsigned numOnes = ctbAlfAlternative[ctuRsAddr];
      assert( ctbAlfAlternative[ctuRsAddr] < numAlts );
      for( int i = 0; i < numOnes; ++i )
        m_BinEncoder.encodeBin( 1, Ctx::ctbAlfAlternative( compIdx-1 ) );
      if( numOnes < numAlts-1 )
        m_BinEncoder.encodeBin( 0, Ctx::ctbAlfAlternative( compIdx-1 ) );
    }
  }
}

#endif

//! \}
