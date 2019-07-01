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

/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)           const CodingStatisticsClassType CSCT(x);                       m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)        const CodingStatisticsClassType CSCT(x,y);                     m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)    const CodingStatisticsClassType CSCT(x, s.width, s.height);    m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z) const CodingStatisticsClassType CSCT(x, s.width, s.height, z); m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)                  m_BinDecoder.set( x );
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)
#endif


void CABACReader::initCtxModels( Slice& slice )
{
  SliceType sliceType  = slice.getSliceType();
  int       qp         = slice.getSliceQp();
  if( slice.getPPS()->getCabacInitPresentFlag() && slice.getCabacInitFlag() )
  {
    switch( sliceType )
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      THROW( "Invalid slice type" );
      break;
    }
  }
  m_BinDecoder.reset( qp, (int)sliceType );
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if( m_BinDecoder.decodeBinTrm() )
  {
    m_BinDecoder.finish();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::IncrementStatisticEP( STATS__TRAILING_BITS, m_Bitstream->readOutTrailingBits(), 0 );
#else
    m_Bitstream->readOutTrailingBits();
#endif
    return true;
  }
  return false;
}

void CABACReader::remaining_bytes( bool noTrailingBytesExpected )
{
  if( noTrailingBytesExpected )
  {
    CHECK( 0 != m_Bitstream->getNumBitsLeft(), "Bits left when not supposed" );
  }
  else
  {
    while( m_Bitstream->getNumBitsLeft() )
    {
      unsigned trailingNullByte = m_Bitstream->readByte();
      if( trailingNullByte != 0 )
      {
        THROW( "Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n" );
      }
    }
  }
}

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qpL, qpC, ctuRsAddr )
//================================================================================

bool CABACReader::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr )
{
  CUCtx cuCtx( qps[CH_L] );
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );

  partitioner->initCtu( area, CH_L, *cs.slice );


  sao( cs, ctuRsAddr );
#if JVET_N0415_CTB_ALF
  if (cs.sps->getALFEnabledFlag() && (cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y)))
  {
#else
  if (cs.sps->getALFEnabledFlag() && (cs.slice->getTileGroupAlfEnabledFlag()))
  {
    CHECK(cs.aps == nullptr, "APS not initialized");
    const AlfSliceParam& alfSliceParam = cs.aps->getAlfAPSParam();
#endif
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
#if JVET_N0857_TILES_BRICKS
    const uint32_t          curTileIdx = cs.picture->brickMap->getBrickIdxRsMap( pos );
#else
    const uint32_t          curTileIdx = cs.picture->tileMap->getTileIdxMap( pos );
#endif
#if JVET_N0150_ONE_CTU_DELAY_WPP
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
#else
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
#endif

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
#if JVET_N0415_CTB_ALF
      if (cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx))
#else
      if( alfSliceParam.enabledFlag[compIdx] )
#endif
      {
        uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
        int ctx = 0;
        ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
        ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;

        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__ALF);
        ctbAlfFlag[ctuRsAddr] = m_BinDecoder.decodeBin( Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );

#if JVET_N0415_CTB_ALF
        if (isLuma((ComponentID)compIdx) && ctbAlfFlag[ctuRsAddr])
        {
          readAlfCtuFilterIndex(cs, ctuRsAddr);
        }
#endif
      }
    }
  }

  bool isLast = false;

  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
  {
    Partitioner *chromaPartitioner = PartitionerFactory::get(*cs.slice);
    chromaPartitioner->initCtu(area, CH_C, *cs.slice);
    CUCtx cuCtxChroma(qps[CH_C]);
    isLast = coding_tree(cs, *partitioner, cuCtx, chromaPartitioner, &cuCtxChroma);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = cuCtxChroma.qp;
    delete chromaPartitioner;
  }
  else
  {
    isLast = coding_tree(cs, *partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( !isLast && CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner->initCtu( area, CH_C, *cs.slice );
      isLast = coding_tree( cs, *partitioner, cuCtxChroma );
      qps[CH_C] = cuCtxChroma.qp;
    }
  }

  DTRACE_COND( ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->poc, cs.slice->getSliceQpBase() );
  DTRACE     (                 g_trace_ctx, D_QP_PER_CTU, " %3d",           qps[CH_L] - cs.slice->getSliceQpBase() );

  delete partitioner;
  return isLast;
}

#if JVET_N0415_CTB_ALF
void CABACReader::readAlfCtuFilterIndex(CodingStructure& cs, unsigned ctuRsAddr)
{
  short* alfCtbFilterSetIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  unsigned numAps = cs.slice->getTileGroupNumAps();
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  uint32_t filtIndex = 0;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    int useLatestFilt = m_BinDecoder.decodeBin(Ctx::AlfUseLatestFilt());
    if (useLatestFilt)
    {
      filtIndex = NUM_FIXED_FILTER_SETS;
    }
    else
    {
      if (numAps == 1)
      {
        xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
      }
      else
      {
        unsigned usePrevFilt = m_BinDecoder.decodeBin(Ctx::AlfUseTemporalFilt());
        if (usePrevFilt)
        {
          xReadTruncBinCode(filtIndex, numAvailableFiltSets - (NUM_FIXED_FILTER_SETS + 1));
          filtIndex += (unsigned)(NUM_FIXED_FILTER_SETS + 1);
        }
        else
        {
          xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
        }
      }
    }
  }
  else
  {
    xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
  }
  alfCtbFilterSetIndex[ctuRsAddr] = filtIndex;
}
#endif
//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao( slice, ctuRsAddr )
//================================================================================

void CABACReader::sao( CodingStructure& cs, unsigned ctuRsAddr )
{
  const SPS&   sps   = *cs.sps;

  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  const Slice& slice                        = *cs.slice;
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool              slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool              slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  sao_ctu_pars[ COMPONENT_Y  ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cb ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cr ].modeIdc      = SAO_MODE_OFF;
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  // merge
  int             frame_width_in_ctus     = cs.pcv->widthInCtus;
  int             ry                      = ctuRsAddr      / frame_width_in_ctus;
  int             rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  int             sao_merge_type          = -1;
  const Position  pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned  curSliceIdx = cs.slice->getIndependentSliceIdx();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SAO );

#if JVET_N0857_TILES_BRICKS
  const unsigned  curTileIdx  = cs.picture->brickMap->getBrickIdxRsMap( pos );
#else
  const unsigned  curTileIdx  = cs.picture->tileMap->getTileIdxMap( pos );
#endif
#if JVET_N0150_ONE_CTU_DELAY_WPP
  if( cs.getCURestricted( pos.offset(-(int)cs.pcv->maxCUWidth, 0), pos, curSliceIdx, curTileIdx, CH_L ) )
#else
  if( cs.getCURestricted( pos.offset(-(int)cs.pcv->maxCUWidth, 0), curSliceIdx, curTileIdx, CH_L ) )
#endif
  {
    // sao_merge_left_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) );
  }

#if JVET_N0150_ONE_CTU_DELAY_WPP
  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(int)cs.pcv->maxCUHeight), pos, curSliceIdx, curTileIdx, CH_L ) )
#else
  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(int)cs.pcv->maxCUHeight), curSliceIdx, curTileIdx, CH_L ) )
#endif
  {
    // sao_merge_above_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) ) << 1;
  }
  if( sao_merge_type >= 0 )
  {
    if( slice_sao_luma_flag || slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Y  ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Y  ].typeIdc  = sao_merge_type;
    }
    if( slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Cb ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cr ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cb ].typeIdc  = sao_merge_type;
      sao_ctu_pars[ COMPONENT_Cr ].typeIdc  = sao_merge_type;
    }
    return;
  }

  // explicit parameters
  ComponentID firstComp = ( slice_sao_luma_flag   ? COMPONENT_Y  : COMPONENT_Cb );
  ComponentID lastComp  = ( slice_sao_chroma_flag ? COMPONENT_Cr : COMPONENT_Y  );
  for( ComponentID compID = firstComp; compID <= lastComp; compID = ComponentID( compID + 1 ) )
  {
    SAOOffset& sao_pars = sao_ctu_pars[ compID ];

    // sao_type_idx_luma / sao_type_idx_chroma
    if( compID != COMPONENT_Cr )
    {
      if( m_BinDecoder.decodeBin( Ctx::SaoTypeIdx() ) )
      {
        if( m_BinDecoder.decodeBinEP( ) )
        {
          // edge offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_EO;
        }
        else
        {
          // band offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_BO;
        }
      }
    }
    else //Cr, follow Cb SAO type
    {
      sao_pars.modeIdc = sao_ctu_pars[ COMPONENT_Cb ].modeIdc;
      sao_pars.typeIdc = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    if( sao_pars.modeIdc == SAO_MODE_OFF )
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( sps.getBitDepth( toChannelType(compID) ) );
    offset    [0]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [1]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [2]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [3]           = (int)unary_max_eqprob( maxOffsetQVal );

    // band offset mode
    if( sao_pars.typeIdc == SAO_TYPE_START_BO )
    {
      // sao_offset_sign
      for( int k = 0; k < 4; k++ )
      {
        if( offset[k] && m_BinDecoder.decodeBinEP( ) )
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      sao_pars.typeAuxInfo = m_BinDecoder.decodeBinsEP( NUM_SAO_BO_CLASSES_LOG2 );
      for( int k = 0; k < 4; k++ )
      {
        sao_pars.offset[ ( sao_pars.typeAuxInfo + k ) % MAX_NUM_SAO_CLASSES ] = offset[k];
      }
      continue;
    }

    // edge offset mode
    sao_pars.typeAuxInfo = 0;
    if( compID != COMPONENT_Cr )
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      sao_pars.typeIdc += m_BinDecoder.decodeBinsEP( NUM_SAO_EO_TYPES_LOG2 );
    }
    else
    {
      sao_pars.typeIdc  = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    sao_pars.offset[ SAO_CLASS_EO_FULL_VALLEY ] =  offset[0];
    sao_pars.offset[ SAO_CLASS_EO_HALF_VALLEY ] =  offset[1];
    sao_pars.offset[ SAO_CLASS_EO_PLAIN       ] =  0;
    sao_pars.offset[ SAO_CLASS_EO_HALF_PEAK   ] = -offset[2];
    sao_pars.offset[ SAO_CLASS_EO_FULL_PEAK   ] = -offset[3];
  }
}

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    bool  coding_tree       ( cs, partitioner, cuCtx )
//    bool  split_cu_flag     ( cs, partitioner )
//    split split_cu_mode_mt  ( cs, partitioner )
//================================================================================

bool CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  bool           lastSegment  = false;

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if( pps.getUseDQP() && partitioner.currQgEnable() )
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded = false;
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
  int startShareThisLevel = 0;

  const PartSplit splitMode = split_cu_mode( cs, partitioner );

  CHECK( !partitioner.canSplit( splitMode, cs ), "Got an invalid split!" );

  if( splitMode != CU_DONT_SPLIT )
  {
      const PartSplit split = splitMode;
      int splitRatio = 1;
      CHECK(!(split == CU_QUAD_SPLIT || split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT
        || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT), "invalid split type");
      splitRatio = (split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT) ? 1 : 2;

      bool isOneChildSmall = (((partitioner.currArea().lwidth())*(partitioner.currArea().lheight())) >> splitRatio) < MRG_SHARELIST_SHARSIZE;

      if ((((partitioner.currArea().lwidth())*(partitioner.currArea().lheight())) > (MRG_SHARELIST_SHARSIZE * 1)))
      {
        shareStateDec = NO_SHARE;
      }

      if (shareStateDec == NO_SHARE)//init state
      {
        if (isOneChildSmall)
        {
          shareStateDec = SHARING;//share start state
          startShareThisLevel = 1;

          shareParentPos = partitioner.currArea().lumaPos();
          shareParentSize.width = partitioner.currArea().lwidth();
          shareParentSize.height = partitioner.currArea().lheight();
        }
      }
      if (CS::isDualITree(cs) && pPartitionerChroma != nullptr && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
      {
        partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
        pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
        bool beContinue = true;
        bool lumaContinue = true;
        bool chromaContinue = true;
        bool lastSegmentC = false;

        while (beContinue)
        {
          if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
          {
            if (!lastSegmentC && cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              lastSegmentC = coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
            }
            lumaContinue = partitioner.nextPart(cs);
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
          else
          {
            //dual tree coding under 64x64 block
            if (!lastSegment && cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              lastSegment = coding_tree(cs, partitioner, cuCtx);
            }
            lumaContinue = partitioner.nextPart(cs);
            if (!lastSegmentC && cs.area.blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
            {
              lastSegmentC = coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
            }
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            CHECK(lastSegment == true, "luma should not be the last segment");
            beContinue = lumaContinue;
          }
        }
        partitioner.exitCurrSplit();
        pPartitionerChroma->exitCurrSplit();

        //cat the chroma CUs together
        CodingUnit* currentCu = cs.getCU(partitioner.currArea().lumaPos(), CHANNEL_TYPE_LUMA);
        CodingUnit* nextCu = nullptr;
        CodingUnit* tempLastLumaCu = nullptr;
        CodingUnit* tempLastChromaCu = nullptr;
        ChannelType currentChType = currentCu->chType;
        while (currentCu->next != nullptr)
        {
          nextCu = currentCu->next;
          if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_LUMA)
          {
            tempLastLumaCu = currentCu;
            if (tempLastChromaCu != nullptr) //swap
            {
              tempLastChromaCu->next = nextCu;
            }
          }
          else if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_CHROMA)
          {
            tempLastChromaCu = currentCu;
            if (tempLastLumaCu != nullptr) //swap
            {
              tempLastLumaCu->next = nextCu;
            }
          }
          currentCu = nextCu;
          currentChType = currentCu->chType;
        }

        CodingUnit* chromaFirstCu = cs.getCU(pPartitionerChroma->currArea().chromaPos(), CHANNEL_TYPE_CHROMA);
        tempLastLumaCu->next = chromaFirstCu;

        lastSegment = lastSegmentC;
      }
      else
      {
      partitioner.splitCurrArea( splitMode, cs );
      do
      {
        if( !lastSegment && cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          lastSegment = coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      }
      if (startShareThisLevel == 1)
        shareStateDec = NO_SHARE;
      return lastSegment;
  }

  CodingUnit& cu = cs.addCU( CS::getArea( cs, currArea, partitioner.chType ), partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice   = cs.slice;
#if JVET_N0857_TILES_BRICKS
  cu.tileIdx = cs.picture->brickMap->getBrickIdxRsMap( currArea.lumaPos() );
#else
  cu.tileIdx = cs.picture->tileMap->getTileIdxMap( currArea.lumaPos() );
#endif

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }

  if (pps.getUseDQP() && CS::isDualITree(cs) && isChroma(cu.chType))
  {
    const Position chromaCentral(cu.chromaPos().offset(cu.chromaSize().width >> 1, cu.chromaSize().height >> 1));
    const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, cu.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, cu.chromaFormat));
    const CodingUnit* colLumaCu = cs.getCU(lumaRefPos, CHANNEL_TYPE_LUMA);

    if (colLumaCu) cuCtx.qp = colLumaCu->qp;
  }

  cu.qp = cuCtx.qp;                 //NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;  //NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level

  // coding unit
    cu.shareParentPos = (shareStateDec == SHARING) ? shareParentPos : partitioner.currArea().lumaPos();
    cu.shareParentSize = (shareStateDec == SHARING) ? shareParentSize : partitioner.currArea().lumaSize();

  bool isLastCtu = coding_unit( cu, partitioner, cuCtx );

  DTRACE( g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  if (startShareThisLevel == 1)
    shareStateDec = NO_SHARE;
  return isLastCtu;
}

PartSplit CABACReader::split_cu_mode( CodingStructure& cs, Partitioner &partitioner )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SPLIT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );

  PartSplit mode = CU_DONT_SPLIT;

  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  bool isSplit = canBh || canBv || canTh || canTv || canQt;

  if( canNo && isSplit )
  {
    isSplit = m_BinDecoder.decodeBin( Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, isSplit );

  if( !isSplit )
  {
    return CU_DONT_SPLIT;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  bool       isQt   = canQt;

  if( isQt && canBtt )
  {
    isQt = m_BinDecoder.decodeBin( Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return CU_QUAD_SPLIT;
  }

  const bool canHor = canBh || canTh;
  bool        isVer = canBv || canTv;

  if( isVer && canHor )
  {
    isVer = m_BinDecoder.decodeBin( Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  bool        is12 = isVer ? canBv : canBh;

  if( is12 && can14 )
  {
    is12 = m_BinDecoder.decodeBin( Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  if     ( isVer && is12 )  mode = CU_VERT_SPLIT;
  else if( isVer && !is12 ) mode = CU_TRIV_SPLIT;
  else if( !isVer && is12 ) mode = CU_HORZ_SPLIT;
  else                      mode = CU_TRIH_SPLIT;

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, mode );

  return mode;
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    bool  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_transquant_bypass_flag ( cu )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  pcm_flag                  ( cu )
//    void  pcm_samples               ( tu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    bool  end_of_ctu                ( cu, cuCtx )
//================================================================================

bool CABACReader::coding_unit( CodingUnit &cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;
  // transquant bypass flag
  if( cs.pps->getTransquantBypassEnabledFlag() )
  {
    cu_transquant_bypass_flag( cu );
  }
#if JVET_N0324_REGULAR_MRG_FLAG
  PredictionUnit&    pu = cs.addPU(cu, partitioner.chType);
#endif
  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
  {
    cu_skip_flag( cu );
  }

  // skip data
  if( cu.skip )
  {
    cs.addTU         ( cu, partitioner.chType );
#if !JVET_N0324_REGULAR_MRG_FLAG
    PredictionUnit&    pu = cs.addPU( cu, partitioner.chType );
#endif
    pu.shareParentPos = cu.shareParentPos;
    pu.shareParentSize = cu.shareParentSize;
    MergeCtx           mrgCtx;
    prediction_unit  ( pu, mrgCtx );
    return end_of_ctu( cu, cuCtx );
  }

  // prediction mode and partitioning data
  pred_mode ( cu );
#if JVET_N0413_RDPCM
  bdpcm_mode( cu, ComponentID( partitioner.chType ) );
#endif

  // --> create PUs
#if !JVET_N0324_REGULAR_MRG_FLAG
  CU::addPUs( cu );
#endif
  // pcm samples
  if( CU::isIntra(cu) )
  {
    pcm_flag( cu, partitioner );
    if( cu.ipcm )
    {
      TransformUnit& tu = cs.addTU( cu, partitioner.chType );
      pcm_samples( tu );
      return end_of_ctu( cu, cuCtx );
    }
  }

#if !JVET_N0217_MATRIX_INTRAPRED
  extend_ref_line( cu );

  isp_mode( cu );
#endif

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // check end of cu
  return end_of_ctu( cu, cuCtx );
}


void CABACReader::cu_transquant_bypass_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__TQ_BYPASS_FLAG );

  cu.transQuantBypass = ( m_BinDecoder.decodeBin( Ctx::TransquantBypassFlag() ) );
}


void CABACReader::cu_skip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SKIP_FLAG );

  if (cu.slice->isIntra() && cu.cs->slice->getSPS()->getIBCFlag())
  {
    cu.skip = false;
    cu.rootCbf = false;
    cu.predMode = MODE_INTRA;
    cu.mmvdSkip = false;
#if JVET_N0318_N0467_IBC_SIZE
    if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
    {
#endif
    unsigned ctxId = DeriveCtx::CtxSkipFlag(cu);
    unsigned skip = m_BinDecoder.decodeBin(Ctx::SkipFlag(ctxId));
    DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );
    if (skip)
    {
      cu.skip = true;
      cu.rootCbf = false;
      cu.predMode = MODE_IBC;
      cu.mmvdSkip = false;
    }
#if JVET_N0318_N0467_IBC_SIZE
    }
#endif
    return;
  }
#if JVET_N0266_SMALL_BLOCKS
  if ( !cu.cs->slice->getSPS()->getIBCFlag() && cu.lwidth() == 4 && cu.lheight() == 4 )
  {
    return;
  }
#endif
  unsigned ctxId  = DeriveCtx::CtxSkipFlag(cu);
  unsigned skip   = m_BinDecoder.decodeBin( Ctx::SkipFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

  if (skip && cu.cs->slice->getSPS()->getIBCFlag())
  {
#if JVET_N0318_N0467_IBC_SIZE
    if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
    {
#endif
#if JVET_N0266_SMALL_BLOCKS
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
        return;
      }
#endif
    unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
    if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
    {
      cu.skip = true;
      cu.rootCbf = false;
      cu.predMode = MODE_IBC;
      cu.mmvdSkip = false;
#if JVET_N0324_REGULAR_MRG_FLAG
      cu.firstPU->regularMergeFlag = false;
#endif
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
#if JVET_N0318_N0467_IBC_SIZE
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
#endif
  }
  if ((skip && CU::isInter(cu) && cu.cs->slice->getSPS()->getIBCFlag()) ||
    (skip && !cu.cs->slice->getSPS()->getIBCFlag()))
  {
#if JVET_N0324_REGULAR_MRG_FLAG
    if (!cu.cs->slice->getSPS()->getUseMMVD() && (cu.firstPU->lwidth() * cu.firstPU->lheight() == 32))
    {
      cu.firstPU->regularMergeFlag = true;
    }
    else
    {
      unsigned regularMergeFlag = (m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(0)));
      DTRACE(g_trace_ctx, D_SYNTAX, "regular_merge_flag() ctx=%d regularMergeFlag=%d\n", 0, regularMergeFlag?1:0);
      cu.firstPU->regularMergeFlag = regularMergeFlag;
    }
    if (cu.firstPU->regularMergeFlag)
    {
      cu.mmvdSkip = false;
      cu.firstPU->mmvdMergeFlag = false;
      cu.firstPU->mhIntraFlag = false;
      cu.affine = false;
      cu.triangle = false;
    }
    else
    {
#if JVET_N0127_MMVD_SPS_FLAG
      if (cu.cs->slice->getSPS()->getUseMMVD())
      {
#endif
#if JVET_N0266_SMALL_BLOCKS
        bool isCUWithOnlyRegularAndMMVD=((cu.firstPU->lwidth() == 8 && cu.firstPU->lheight() == 4) || (cu.firstPU->lwidth() == 4 && cu.firstPU->lheight() == 8));
#else
        bool isCUWithOnlyRegularAndMMVD=((cu.firstPU->lwidth() == 4 && cu.firstPU->lheight() == 4) || (cu.firstPU->lwidth() == 8 && cu.firstPU->lheight() == 4) || (cu.firstPU->lwidth() == 4 && cu.firstPU->lheight() == 8));
#endif
        if (isCUWithOnlyRegularAndMMVD)
        {
          cu.mmvdSkip = !(cu.firstPU->regularMergeFlag);
        }
        else
        {
          unsigned mmvdSkip = m_BinDecoder.decodeBin(Ctx::MmvdFlag(0));
          cu.mmvdSkip = mmvdSkip;
          DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, mmvdSkip ? 1 : 0);
        }
#if JVET_N0127_MMVD_SPS_FLAG
      }
      else
      {
        cu.mmvdSkip = false;
      }
#endif
    }
#else
#if JVET_N0127_MMVD_SPS_FLAG
    if (cu.cs->slice->getSPS()->getUseMMVD())
    {
#endif
      unsigned mmvdSkip = m_BinDecoder.decodeBin(Ctx::MmvdFlag(0));
      cu.mmvdSkip = mmvdSkip;
      DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, mmvdSkip ? 1 : 0);
#if JVET_N0127_MMVD_SPS_FLAG
    }
    else
    {
      cu.mmvdSkip = false;
    }
#endif
#endif
    cu.skip     = true;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTER;
  }
}

void CABACReader::imv_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.cs->sps->getAMVREnabledFlag() )
  {
    return;
  }

  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  if ( cu.affine )
  {
    return;
  }

  const SPS *sps = cu.cs->sps;

  unsigned value = 0;
#if !JVET_N600_AMVR_TPM_CTX_REDUCTION
  unsigned ctxId = DeriveCtx::CtxIMVFlag( cu );
#endif
  if (CU::isIBC(cu))
    value = 1;
  else
#if JVET_N600_AMVR_TPM_CTX_REDUCTION
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 0 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 0 );
#else
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, ctxId );
#endif

  if( sps->getAMVREnabledFlag() && value )
  {
#if JVET_N600_AMVR_TPM_CTX_REDUCTION
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 1 );
#else
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 3 );
#endif
    value++;
  }

  cu.imv = value;
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::affine_amvr_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  unsigned value = 0;
#if JVET_N600_AMVR_TPM_CTX_REDUCTION
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 2 );
#else
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 4 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 4 );
#endif

  if( value )
  {
#if JVET_N600_AMVR_TPM_CTX_REDUCTION
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 3 );
#else
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 5 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 5 );
#endif
    value++;
  }

  cu.imv = value;
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::pred_mode( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__PRED_MODE );

  if (cu.cs->slice->getSPS()->getIBCFlag())
  {
#if JVET_N0266_SMALL_BLOCKS
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) )
#else
    if (cu.cs->slice->isIntra())
#endif
    {
      cu.predMode = MODE_INTRA;
#if JVET_N0318_N0467_IBC_SIZE
      if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
      {
#endif
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
      {
        cu.predMode = MODE_IBC;
      }
#if JVET_N0318_N0467_IBC_SIZE
      }
#endif
    }
    else
    {
      if (m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))))
      {
        cu.predMode = MODE_INTRA;
      }
      else
      {
        cu.predMode = MODE_INTER;
#if JVET_N0318_N0467_IBC_SIZE
        if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
        {
#endif
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
        {
          cu.predMode = MODE_IBC;
        }
#if JVET_N0318_N0467_IBC_SIZE
        }
#endif
      }
    }
  }
  else
  {
#if JVET_N0266_SMALL_BLOCKS
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || m_BinDecoder.decodeBin( Ctx::PredMode( DeriveCtx::CtxPredModeFlag( cu ) ) ) )
#else
    if (cu.cs->slice->isIntra() || m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))))
#endif
    {
      cu.predMode = MODE_INTRA;
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
  }
}
#if JVET_N0413_RDPCM
void CABACReader::bdpcm_mode( CodingUnit& cu, const ComponentID compID )
{
  cu.bdpcmMode = 0;

  if( !CU::bdpcmAllowed( cu, compID ) ) return;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__BDPCM_MODE, cu.block(compID).lumaSize(), compID );

  cu.bdpcmMode = m_BinDecoder.decodeBin( Ctx::BDPCMMode( 0 ) );

  if( cu.bdpcmMode )
  {
    cu.bdpcmMode += m_BinDecoder.decodeBin( Ctx::BDPCMMode( 1 ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "bdpcm_mode() x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode );
}
#endif
void CABACReader::pcm_flag( CodingUnit& cu, Partitioner &partitioner )
{
  const SPS& sps = *cu.cs->sps;
  if( !sps.getPCMEnabledFlag() || partitioner.currArea().lwidth() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lwidth() < (1 << sps.getPCMLog2MinSize())
      || partitioner.currArea().lheight() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lheight() < (1 << sps.getPCMLog2MinSize()) )
  {
    cu.ipcm = false;
    return;
  }
  cu.ipcm = ( m_BinDecoder.decodeBinTrm() );
}


void CABACReader::cu_pred_data( CodingUnit &cu )
{
  if( CU::isIntra( cu ) )
  {
    intra_luma_pred_modes( cu );
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    cu.predMode = MODE_IBC;
    return;
  }
  MergeCtx mrgCtx;

  for( auto &pu : CU::traversePUs( cu ) )
  {
    pu.shareParentPos = cu.shareParentPos;
    pu.shareParentSize = cu.shareParentSize;
    prediction_unit( pu, mrgCtx );
  }

  imv_mode   ( cu, mrgCtx );
  affine_amvr_mode( cu, mrgCtx );
  cu_gbi_flag( cu );

}

void CABACReader::cu_gbi_flag(CodingUnit& cu)
{
  if(!CU::isGBiIdxCoded(cu))
  {
    return;
  }

  CHECK(!(GBI_NUM > 1 && (GBI_NUM == 2 || (GBI_NUM & 0x01) == 1)), " !( GBI_NUM > 1 && ( GBI_NUM == 2 || ( GBI_NUM & 0x01 ) == 1 ) ) ");

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__GBI_IDX);

  uint32_t idx = 0;

  uint32_t symbol = m_BinDecoder.decodeBin(Ctx::GBiIdx(0));

  int32_t numGBi = (cu.slice->getCheckLDC()) ? 5 : 3;

  if(symbol == 0)
  {
    uint32_t prefixNumBits = numGBi - 2;
    uint32_t step = 1;

#if !JVET_N0286_SIMPLIFIED_GBI_IDX
    unsigned ctxIdGBi = 4;
#endif
    idx = 1;

    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
#if JVET_N0286_SIMPLIFIED_GBI_IDX
      symbol = m_BinDecoder.decodeBinEP();
#else
      symbol = m_BinDecoder.decodeBin(Ctx::GBiIdx(ctxIdGBi));
#endif

      if (symbol == 1)
      {
        break;
      }
#if !JVET_N0286_SIMPLIFIED_GBI_IDX
      ctxIdGBi += step;
#endif
      idx += step;
    }
  }

  uint8_t gbiIdx = (uint8_t)g_GbiParsingOrder[idx];
  CU::setGbiIdx(cu, gbiIdx);

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_gbi_flag() gbi_idx=%d\n", cu.GBiIdx ? 1 : 0);
}

void CABACReader::xReadTruncBinCode(uint32_t& symbol, uint32_t maxSymbol)
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
  int b = maxSymbol - val;
  symbol = m_BinDecoder.decodeBinsEP(thresh);
  if (symbol >= val - b)
  {
    uint32_t altSymbol;
    altSymbol = m_BinDecoder.decodeBinEP();
    symbol <<= 1;
    symbol += altSymbol;
    symbol -= (val - b);
  }
}

void CABACReader::extend_ref_line(CodingUnit& cu)
{
#if !ENABLE_JVET_L0283_MRL
  return;
#endif
#if JVET_N0413_RDPCM
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.ipcm || cu.bdpcmMode )
#else
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.ipcm)
#endif
  {
    cu.firstPU->multiRefIdx = 0;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MULTI_REF_LINE);

  const int numBlocks = CU::getNumPUs(cu);
  PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
    if (isFirstLineOfCtu)
    {
      pu->multiRefIdx = 0;
      continue;
    }
    int multiRefIdx = 0;

    if (MRL_NUM_REF_LINES > 1)
    {
      multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
        multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
        if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1])
        {
          multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(2)) == 1 ? MULTI_REF_LINE_IDX[3] : MULTI_REF_LINE_IDX[2];
        }
      }

    }
    pu->multiRefIdx = multiRefIdx;
    pu = pu->next;
  }
}

void CABACReader::intra_luma_pred_modes( CodingUnit &cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

#if JVET_N0413_RDPCM
  if( cu.bdpcmMode )
  {
    PredictionUnit *pu = cu.firstPU;
    unsigned mpm_pred[NUM_MOST_PROBABLE_MODES];
    PU::getIntraMPMs(*pu, mpm_pred);
    cu.firstPU->intraDir[0] = mpm_pred[0];
    return;
  }
#endif

#if JVET_N0217_MATRIX_INTRAPRED
  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
  extend_ref_line( cu );
  isp_mode( cu );
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, cu.lumaSize(), CHANNEL_TYPE_LUMA );

  // prev_intra_luma_pred_flag
  int numBlocks = CU::getNumPUs( cu );
  int mpmFlag[4];
  for( int k = 0; k < numBlocks; k++ )
  {
    CHECK(numBlocks != 1, "not supported yet");
    if( cu.firstPU->multiRefIdx || ( cu.ispMode && isLuma( cu.chType ) ) )
    {
      mpmFlag[0] = true;
    }
    else
    {
      mpmFlag[k] = m_BinDecoder.decodeBin(Ctx::IntraLumaMpmFlag());
    }
  }

  PredictionUnit *pu = cu.firstPU;

  unsigned mpm_pred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    PU::getIntraMPMs( *pu, mpm_pred );

    if( mpmFlag[k] )
    {
      uint32_t ipred_idx = 0;
      {
#if JVET_N0185_UNIFIED_MPM
        unsigned ctx = (pu->cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
        if (pu->multiRefIdx == 0)
          ipred_idx = m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(ctx));
        else
          ipred_idx = 1;
#else
        ipred_idx = m_BinDecoder.decodeBinEP();
#endif
        if( ipred_idx )
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 1)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 2)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 3)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
      }
      pu->intraDir[0] = mpm_pred[ipred_idx];
    }
    else
    {
      unsigned ipred_mode = 0;

      {
        xReadTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);
      }
      //postponed sorting of MPMs (only in remaining branch)
      std::sort( mpm_pred, mpm_pred + NUM_MOST_PROBABLE_MODES );

      for( uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
      {
        ipred_mode += (ipred_mode >= mpm_pred[i]);
      }

      pu->intraDir[0] = ipred_mode;
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}

void CABACReader::intra_chroma_pred_modes( CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_LUMA ) )
  {
    return;
  }

  PredictionUnit *pu = cu.firstPU;

  {
    CHECK( pu->cu != &cu, "Inkonsistent PU-CU mapping" );
    intra_chroma_pred_mode( *pu );
  }
}

bool CABACReader::intra_chroma_lmc_mode( PredictionUnit& pu )
{
  int lmModeList[10];
  int maxSymbol = PU::getLMSymbolList(pu, lmModeList);
  int symbol    = unary_max_symbol(Ctx::IntraChromaPredMode(1), Ctx::IntraChromaPredMode(2), maxSymbol - 1);
  if (lmModeList[symbol] != -1)
  {
    pu.intraDir[1] = lmModeList[symbol];
    return true;
  }
  return false;
}

void CABACReader::intra_chroma_pred_mode( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, pu.cu->blocks[pu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  if (m_BinDecoder.decodeBin(Ctx::IntraChromaPredMode(0)) == 0)
  {
    pu.intraDir[1] = DM_CHROMA_IDX;
    return;
  }

  // LM chroma mode
  if( pu.cs->sps->getUseLMChroma() )
  {
    if( intra_chroma_lmc_mode( pu ) )
    {
      return;
    }
  }
  unsigned candId = m_BinDecoder.decodeBinsEP( 2 );

  unsigned chromaCandModes[ NUM_CHROMA_MODE ];
  PU::getIntraChromaCandModes( pu, chromaCandModes );

  CHECK( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  CHECK( PU::isLMCMode( chromaCandModes[ candId ] ), "The intra dir cannot be LM_CHROMA for this path" );
  CHECK( chromaCandModes[ candId ] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );

  pu.intraDir[1] = chromaCandModes[ candId ];
}

void CABACReader::cu_residual( CodingUnit& cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    PredictionUnit& pu = *cu.firstPU;
    if( !pu.mergeFlag )
    {
      rqt_root_cbf( cu );
    }
    else
    {
      cu.rootCbf = true;
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }
    if( !cu.rootCbf )
    {
      TransformUnit& tu = cu.cs->addTU(cu, partitioner.chType);
      tu.depth = 0;
      for( unsigned c = 0; c < tu.blocks.size(); c++ )
      {
        tu.cbf[c]             = 0;
        ComponentID   compID  = ComponentID(c);
        tu.getCoeffs( compID ).fill( 0 );
        tu.getPcmbuf( compID ).fill( 0 );
      }
      return;
    }
  }

  ChromaCbfs chromaCbfs;
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    transform_tree( *cu.cs, subTuPartitioner, cuCtx, chromaCbfs, CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType ) ), 0 );
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
  }

#if JVET_N0193_LFNST
  residual_lfnst_mode( cu );
#endif
}

void CABACReader::rqt_root_cbf( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__QT_ROOT_CBF );

  cu.rootCbf = ( m_BinDecoder.decodeBin( Ctx::QtRootCbf() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACReader::sbt_mode( CodingUnit& cu )
{
  const uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SBT_MODE );
  //bin - flag
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  bool sbtFlag = m_BinDecoder.decodeBin( Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );

  //bin - type
  bool sbtQuadFlag = false;
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    sbtQuadFlag = m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    sbtQuadFlag = 0;
  }

  //bin - dir
  bool sbtHorFlag = false;
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    sbtHorFlag = m_BinDecoder.decodeBin( Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    sbtHorFlag = ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow );
  }
  cu.setSbtIdx( sbtHorFlag ? ( sbtQuadFlag ? SBT_HOR_QUAD : SBT_HOR_HALF ) : ( sbtQuadFlag ? SBT_VER_QUAD : SBT_VER_HALF ) );

  //bin - pos
  bool sbtPosFlag = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 0 ) );
  cu.setSbtPos( sbtPosFlag ? SBT_POS1 : SBT_POS0 );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}


bool CABACReader::end_of_ctu( CodingUnit& cu, CUCtx& cuCtx )
{
  const SPS     &sps   = *cu.cs->sps;
  const Position rbPos = recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].bottomRight().offset( 1, 1 ) );

  if ( ( ( rbPos.x & cu.cs->pcv->maxCUWidthMask  ) == 0 || rbPos.x == sps.getPicWidthInLumaSamples () )
    && ( ( rbPos.y & cu.cs->pcv->maxCUHeightMask ) == 0 || rbPos.y == sps.getPicHeightInLumaSamples() )
    && ( !CS::isDualITree( *cu.cs ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

    return terminating_bit();
  }

  return false;
}

//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu, mrgCtx );
//    void  merge_flag      ( pu );
//    void  merge_data      ( pu, mrgCtx );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACReader::prediction_unit( PredictionUnit& pu, MergeCtx& mrgCtx )
{
  if( pu.cu->skip )
  {
    pu.mergeFlag = true;
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
    if (CU::isIBC(*pu.cu))
    {
      merge_idx(pu);
    }
    else
    {
#if JVET_N0324_REGULAR_MRG_FLAG
      if (pu.regularMergeFlag)
      {
        merge_idx(pu);
      }
      else
      {
#endif
        subblock_merge_flag( *pu.cu );
        MHIntra_flag(pu);
        if (pu.mhIntraFlag)
        {
#if JVET_N0302_SIMPLFIED_CIIP
          pu.intraDir[0] = PLANAR_IDX;
#else
          MHIntra_luma_pred_modes(*pu.cu);
#endif
          pu.intraDir[1] = DM_CHROMA_IDX;
        }
#if JVET_N0324_REGULAR_MRG_FLAG
        else
        {
          pu.cu->triangle = pu.cu->cs->slice->getSPS()->getUseTriangle() && pu.cu->cs->slice->isInterB() && !pu.cu->affine && !pu.mmvdMergeFlag && !pu.cu->mmvdSkip;
        }
#else
        triangle_mode( *pu.cu );
#endif
        if (pu.mmvdMergeFlag)
        {
          mmvd_merge_idx(pu);
        }
        else
          merge_data   ( pu );
#if JVET_N0324_REGULAR_MRG_FLAG
      }
#endif
    }
  }
  else if (CU::isIBC(*pu.cu))
  {
    pu.interDir = 1;
    pu.cu->affine = false;
    pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
    mvd_coding(pu.mvd[REF_PIC_LIST_0]);
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
      if( pu.cu->affine )
      {
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][0] );
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][1] );
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][2] );
        }
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_0] );
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }

    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      if ( pu.cu->smvdMode != 1 )
      {
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 /* PRED_BI */ )
      {
        pu.mvd[ REF_PIC_LIST_1 ] = Mv();
        pu.mvdAffi[REF_PIC_LIST_1][0] = Mv();
        pu.mvdAffi[REF_PIC_LIST_1][1] = Mv();
        pu.mvdAffi[REF_PIC_LIST_1][2] = Mv();
      }
      else if( pu.cu->affine )
      {
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][0] );
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][1] );
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][2] );
        }
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_1] );
      }
      }
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
  }
  if( pu.interDir == 3 /* PRED_BI */ && PU::isBipredRestriction(pu) )
  {
    pu.mv    [REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
    pu.interDir               =  1;
    pu.cu->GBiIdx = GBI_DEFAULT;
  }

  if ( pu.cu->smvdMode )
  {
    RefPicList eCurRefList = (RefPicList)(pu.cu->smvdMode - 1);
    pu.mvd[1 - eCurRefList].set( -pu.mvd[eCurRefList].hor, -pu.mvd[eCurRefList].ver );
    pu.refIdx[1 - eCurRefList] = pu.cs->slice->getSymRefIdx( 1 - eCurRefList );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}

void CABACReader::smvd_mode( PredictionUnit& pu )
{
  pu.cu->smvdMode = 0;
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }

  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SYMMVD_FLAG );

  pu.cu->smvdMode = m_BinDecoder.decodeBin( Ctx::SmvdFlag() ) ? 1 : 0;

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

void CABACReader::subblock_merge_flag( CodingUnit& cu )
{
  if ( cu.firstPU->mergeFlag && (cu.firstPU->mmvdMergeFlag || cu.mmvdSkip) )
  {
    return;
  }

  if ( !cu.cs->slice->isIntra() && (cu.cs->sps->getUseAffine() || cu.cs->sps->getSBTMVPEnabledFlag()) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__AFFINE_FLAG );

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACReader::affine_flag( CodingUnit& cu )
{
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__AFFINE_FLAG );

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      ctxId = 0;
      cu.affineType = m_BinDecoder.decodeBin( Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
    else
    {
      cu.affineType = AFFINEMODEL_4PARAM;
    }
  }
}

void CABACReader::merge_flag( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MERGE_FLAG );

  pu.mergeFlag = ( m_BinDecoder.decodeBin( Ctx::MergeFlag() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

  if (pu.mergeFlag && CU::isIBC(*pu.cu))
  {
    pu.mmvdMergeFlag = false;
#if JVET_N0324_REGULAR_MRG_FLAG
    pu.regularMergeFlag = false;
#endif
    return;
  }
#if JVET_N0324_REGULAR_MRG_FLAG
  if (pu.mergeFlag)
  {
    if (!pu.cs->sps->getUseMMVD() && (pu.lwidth() * pu.lheight() == 32))
    {
      pu.regularMergeFlag = true;
    }
    else
    {
      pu.regularMergeFlag = (m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(1)));
      DTRACE(g_trace_ctx, D_SYNTAX, "regular_merge_flag() ctx=%d pu.regularMergeFlag=%d\n", 1, pu.regularMergeFlag?1:0);
    }
    if (pu.regularMergeFlag)
    {
      pu.mmvdMergeFlag = false;
      pu.mhIntraFlag = false;
      pu.cu->affine = false;
      pu.cu->triangle = false;
    }
    else
    {
#if JVET_N0127_MMVD_SPS_FLAG
      if (pu.cs->sps->getUseMMVD())
      {
#endif
#if JVET_N0266_SMALL_BLOCKS
        bool isCUWithOnlyRegularAndMMVD=((pu.lwidth() == 8 && pu.lheight() == 4) || (pu.lwidth() == 4 && pu.lheight() == 8));
#else
        bool isCUWithOnlyRegularAndMMVD=((pu.lwidth() == 4 && pu.lheight() == 4) || (pu.lwidth() == 8 && pu.lheight() == 4) || (pu.lwidth() == 4 && pu.lheight() == 8));
#endif
        if (isCUWithOnlyRegularAndMMVD)
        {
          pu.mmvdMergeFlag = !(pu.regularMergeFlag);
        }
        else
        {
          pu.mmvdMergeFlag = (m_BinDecoder.decodeBin(Ctx::MmvdFlag(0)));
          DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
        }
#if JVET_N0127_MMVD_SPS_FLAG
      }
      else
      {
        pu.mmvdMergeFlag = false;
      }
#endif
    }
  }
#else
#if JVET_N0127_MMVD_SPS_FLAG
  if (pu.mergeFlag && pu.cs->sps->getUseMMVD())
#else
  if (pu.mergeFlag)
#endif
  {
    pu.mmvdMergeFlag = (m_BinDecoder.decodeBin(Ctx::MmvdFlag(0)));
    DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  }
#if JVET_N0127_MMVD_SPS_FLAG
  else
  {
    pu.mmvdMergeFlag = false;
  }
#endif
#endif
}


void CABACReader::merge_data( PredictionUnit& pu )
{
  if (pu.cu->mmvdSkip)
  {
    mmvd_merge_idx(pu);
  }
  else
  {
    merge_idx(pu);
  }
}


void CABACReader::merge_idx( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MERGE_INDEX );

  if ( pu.cu->affine )
  {
    int numCandminus1 = int( pu.cs->slice->getMaxNumAffineMergeCand() ) - 1;
    pu.mergeIdx = 0;
    if ( numCandminus1 > 0 )
    {
      if ( m_BinDecoder.decodeBin( Ctx::AffMergeIdx() ) )
      {
        pu.mergeIdx++;
        for ( ; pu.mergeIdx < numCandminus1; pu.mergeIdx++ )
        {
            if ( !m_BinDecoder.decodeBinEP() )
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
  int numCandminus1 = int( pu.cs->slice->getMaxNumMergeCand() ) - 1;
  pu.mergeIdx       = 0;

  if( pu.cu->triangle )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__TRIANGLE_INDEX );
    bool    splitDir;
    uint8_t candIdx0;
    uint8_t candIdx1;
    splitDir = m_BinDecoder.decodeBinEP();
    auto decodeOneIdx = [this](int numCandminus1) -> uint8_t
    {
      uint8_t decIdx = 0;
      if( numCandminus1 > 0 )
      {
        if( this->m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
        {
          decIdx++;
          for( ; decIdx < numCandminus1; decIdx++ )
          {
            if( !this->m_BinDecoder.decodeBinEP() )
              break;
          }
        }
      }
      return decIdx;
    };
#if JVET_N0400_SIGNAL_TRIANGLE_CAND_NUM
    const int maxNumTriangleCand = pu.cs->slice->getMaxNumTriangleCand();
    CHECK(maxNumTriangleCand < 2, "Incorrect max number of triangle candidates");
    candIdx0 = decodeOneIdx(maxNumTriangleCand - 1);
    candIdx1 = decodeOneIdx(maxNumTriangleCand - 2);
#else
    candIdx0 = decodeOneIdx(TRIANGLE_MAX_NUM_UNI_CANDS - 1);
    candIdx1 = decodeOneIdx(TRIANGLE_MAX_NUM_UNI_CANDS - 2);
#endif
    candIdx1 += candIdx1 >= candIdx0 ? 1 : 0;
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_split_dir=%d\n", splitDir );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx0=%d\n", candIdx0 );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx1=%d\n", candIdx1 );
    pu.triangleSplitDir = splitDir;
    pu.triangleMergeIdx0 = candIdx0;
    pu.triangleMergeIdx1 = candIdx1;
    return;
  }

  if( numCandminus1 > 0 )
  {
    if( m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
    {
      pu.mergeIdx++;
      for( ; pu.mergeIdx < numCandminus1; pu.mergeIdx++ )
      {
          if( !m_BinDecoder.decodeBinEP() )
          {
            break;
          }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
  }
}

void CABACReader::mmvd_merge_idx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MERGE_INDEX);
  int var0, var1, var2;
  int dir0 = 0;
  int var = 0;
  int mvpIdx = 0;

  pu.mmvdMergeIdx = 0;

  mvpIdx = (var + dir0)*(MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);

#if JVET_N0448_N0380
  int numCand = int(pu.cs->slice->getMaxNumMergeCand());
  int numCandminus1_base = (numCand > 1) ? MMVD_BASE_MV_NUM - 1 : 0;
#else
  int numCandminus1_base = MMVD_BASE_MV_NUM - 1;
#endif
  var0 = 0;
  if (numCandminus1_base > 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx()))
    {
      var0++;
      for (; var0 < numCandminus1_base; var0++)
      {
        if (!m_BinDecoder.decodeBinEP())
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);
  int numCandminus1_step = MMVD_REFINE_STEP - 1;
  var1 = 0;
  if (numCandminus1_step > 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdx()))
    {
      var1++;
      for (; var1 < numCandminus1_step; var1++)
      {
        if (!m_BinDecoder.decodeBinEP())
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);
  var2 = 0;
  if (m_BinDecoder.decodeBinEP())
  {
    var2 += 2;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  else
  {
    var2 += 0;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  mvpIdx += (var0 * MMVD_MAX_REFINE_NUM + var1 * 4 + var2);
  pu.mmvdMergeIdx = mvpIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
}

void CABACReader::inter_pred_idc( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__INTER_DIR );

  if( pu.cs->slice->isInterP() )
  {
    pu.interDir = 1;
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( m_BinDecoder.decodeBin( Ctx::InterDir(ctxId) ) )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, pu.lumaPos().x, pu.lumaPos().y );
      pu.interDir = 3;
      return;
    }
  }
  if( m_BinDecoder.decodeBin( Ctx::InterDir(4) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", 2, pu.lumaPos().x, pu.lumaPos().y );
    pu.interDir = 2;
    return;
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
  pu.interDir = 1;
  return;
}


void CABACReader::ref_idx( PredictionUnit &pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__REF_FRM_IDX );

  if ( pu.cu->smvdMode )
  {
    pu.refIdx[eRefList] = pu.cs->slice->getSymRefIdx( eRefList );
    return;
  }

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::RefPic() ) )
  {
    if( numRef > 1 )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, pu.lumaPos().x, pu.lumaPos().y );
    }
    pu.refIdx[eRefList] = 0;
    return;
  }
  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::RefPic(1) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
    pu.refIdx[eRefList] = 1;
    return;
  }
  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      pu.refIdx[eRefList] = (signed char)( idx - 1 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx-1, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
  }
}



void CABACReader::mvp_flag( PredictionUnit& pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MVP_IDX );

  unsigned mvp_idx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvp_idx, pu.lumaPos().x, pu.lumaPos().y );
  pu.mvpIdx [eRefList] = mvp_idx;
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvp_idx );
}


void CABACReader::MHIntra_flag(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseMHIntra())
  {
    pu.mhIntraFlag = false;
    return;
  }
  if (pu.cu->skip)
  {
    pu.mhIntraFlag = false;
    return;
  }

  if (pu.mmvdMergeFlag)
  {
    pu.mhIntraFlag = false;
    return;
  }
  if (pu.cu->affine)
  {
    pu.mhIntraFlag = false;
    return;
  }
  if (pu.cu->lwidth() * pu.cu->lheight() < 64 || pu.cu->lwidth() >= MAX_CU_SIZE || pu.cu->lheight() >= MAX_CU_SIZE)
  {
    pu.mhIntraFlag = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MH_INTRA_FLAG);

  pu.mhIntraFlag = (m_BinDecoder.decodeBin(Ctx::MHIntraFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "MHIntra_flag() MHIntra=%d pos=(%d,%d) size=%dx%d\n", pu.mhIntraFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
}

#if !JVET_N0302_SIMPLFIED_CIIP
void CABACReader::MHIntra_luma_pred_modes(CodingUnit &cu)
{
  if (!cu.Y().valid())
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIR_ANG, cu.lumaSize(), CHANNEL_TYPE_LUMA);

  const int numMPMs = 3;  // Multi-hypothesis intra uses only 3 MPM

  // prev_intra_luma_pred_flag
  int numBlocks = CU::getNumPUs(cu);
  int mpmFlag[4];
  PredictionUnit *pu = cu.firstPU;
  for (int k = 0; k < numBlocks; k++)
  {
    if (PU::getNarrowShape(pu->lwidth(), pu->lheight()) == 0)
    {
      mpmFlag[k] = m_BinDecoder.decodeBin(Ctx::MHIntraPredMode());
    }
    else
    {
      mpmFlag[k] = 1;
    }
  }

  unsigned mpm_pred[numMPMs];
  for (int k = 0; k < numBlocks; k++)
  {
    PU::getMHIntraMPMs(*pu, mpm_pred);

    if (mpmFlag[k])
    {
      unsigned pred_idx = 0;

      pred_idx = m_BinDecoder.decodeBinEP();
      if (pred_idx)
      {
        pred_idx += m_BinDecoder.decodeBinEP();
      }
      pu->intraDir[0] = mpm_pred[pred_idx];
    }
    else
    {
      unsigned pred_mode = 0;

      bool isMPMCand[4];
      for (unsigned i = 0; i < 4; i++)
      {
        isMPMCand[i] = false;
      }
      for (unsigned i = 0; i < 3; i++)
      {
        if (mpm_pred[i] == PLANAR_IDX)
        {
          isMPMCand[0] = true;
        }
        else if (mpm_pred[i] == DC_IDX)
        {
          isMPMCand[1] = true;
        }
        else if (mpm_pred[i] == HOR_IDX)
        {
          isMPMCand[2] = true;
        }
        else if (mpm_pred[i] == VER_IDX)
        {
          isMPMCand[3] = true;
        }
      }
      if (!isMPMCand[0])
      {
        pred_mode = PLANAR_IDX;
      }
      if (!isMPMCand[1])
      {
        pred_mode = DC_IDX;
      }
      if (!isMPMCand[2])
      {
        pred_mode = HOR_IDX;
      }
      if (!isMPMCand[3])
      {
        pred_mode = VER_IDX;
      }
      pu->intraDir[0] = pred_mode;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0]);
    pu = pu->next;
  }
}
#endif

void CABACReader::triangle_mode( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__TRIANGLE_FLAG );

  if( !cu.cs->slice->getSPS()->getUseTriangle() || !cu.cs->slice->isInterB() || cu.lwidth() * cu.lheight() < TRIANGLE_MIN_SIZE || cu.affine )
  {
    return;
  }

  if ( cu.firstPU->mmvdMergeFlag || cu.mmvdSkip )
  {
    return;
  }

  if ( cu.firstPU->mhIntraFlag )
  {
    return;
  }

#if JVET_N0400_SIGNAL_TRIANGLE_CAND_NUM
  if (cu.cs->slice->getMaxNumTriangleCand() < 2)
  {
    return;
  }
#endif

#if JVET_N600_AMVR_TPM_CTX_REDUCTION
  cu.triangle = m_BinDecoder.decodeBin( Ctx::TriangleFlag(0) );
#else
  unsigned flag_idx = DeriveCtx::CtxTriangleFlag( cu );
  cu.triangle = m_BinDecoder.decodeBin( Ctx::TriangleFlag(flag_idx) );
#endif


  DTRACE( g_trace_ctx, D_SYNTAX, "triangle_mode() triangle_mode=%d pos=(%d,%d) size: %dx%d\n", cu.triangle, cu.Y().x, cu.Y().y, cu.lumaSize().width, cu.lumaSize().height );
}

//================================================================================
//  clause 7.3.8.7
//--------------------------------------------------------------------------------
//    void  pcm_samples( tu )
//================================================================================

void CABACReader::pcm_samples( TransformUnit& tu )
{
  CHECK( !tu.cu->ipcm, "pcm mode expected" );

  const CodingStructure *cs = tu.cs;
  const ChannelType chType = tu.chType;

  const SPS&        sps       = *tu.cu->cs->sps;
  tu.depth                    = 0;

  ComponentID compStr = (CS::isDualITree(*cs) && !isLuma(chType)) ? COMPONENT_Cb: COMPONENT_Y;
  ComponentID compEnd = (CS::isDualITree(*cs) && isLuma(chType)) ? COMPONENT_Y : COMPONENT_Cr;
  for( ComponentID compID = compStr; compID <= compEnd; compID = ComponentID(compID+1) )
  {
    PelBuf          samples     = tu.getPcmbuf( compID );
    const unsigned  sampleBits  = sps.getPCMBitDepth( toChannelType(compID) );
    for( unsigned y = 0; y < samples.height; y++ )
    {
      for( unsigned x = 0; x < samples.width; x++ )
      {
        samples.at(x, y) = m_BinDecoder.decodeBinsPCM( sampleBits );
      }
    }
  }
  m_BinDecoder.start();
}

//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================

void CABACReader::transform_tree( CodingStructure &cs, Partitioner &partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs, const PartSplit ispType, const int subTuIdx )
{
  ChromaCbfs chromaCbfsLastDepth;
  chromaCbfsLastDepth.Cb        = chromaCbfs.Cb;
  chromaCbfsLastDepth.Cr        = chromaCbfs.Cr;
  const UnitArea& area          = partitioner.currArea();

  CodingUnit&     cu            = *cs.getCU( area.blocks[partitioner.chType], partitioner.chType );
  const unsigned  trDepth       = partitioner.currTrDepth;
        int       subTuCounter  = subTuIdx;

  // split_transform_flag
  bool split = false;

  split = partitioner.canSplit( TU_MAX_TR_SPLIT, cs );

#if JVET_N0492_NO_HIERARCH_CBF
  bool max_tu_split = split;
#endif

  if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
  {
    split = true;
  }

  if( !split && cu.ispMode )
  {
    split = partitioner.canSplit( ispType, cs );
  }
  const bool chromaCbfISP = area.blocks[COMPONENT_Cb].valid() && cu.ispMode && !split;

  // cbf_cb & cbf_cr
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) && ( !cu.ispMode || chromaCbfISP ) )
  {
    const int cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
#if JVET_N0492_NO_HIERARCH_CBF
    if (!max_tu_split)
#endif
    {
#if !JVET_N0492_NO_HIERARCH_CBF
      if (chromaCbfs.Cb)
#endif
      {
        if (!(cu.sbtInfo && trDepth == 1))
          chromaCbfs.Cb &= cbf_comp(cs, area.blocks[COMPONENT_Cb], cbfDepth);
      }
#if !JVET_N0492_NO_HIERARCH_CBF
      if (chromaCbfs.Cr)
#endif
      {
        if (!(cu.sbtInfo && trDepth == 1))
          chromaCbfs.Cr &= cbf_comp(cs, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb);
      }
    }
  }
  else if( CS::isDualITree( cs ) )
  {
    chromaCbfs = ChromaCbfs( false );
  }

  if( split )
  {
    {

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
        THROW( "Implicit TU split not available!" );
    }

    do
    {
      ChromaCbfs subCbfs = chromaCbfs;
      transform_tree( cs, partitioner, cuCtx, subCbfs, ispType, subTuCounter );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();

    const UnitArea &currArea  = partitioner.currArea();
    const unsigned  currDepth = partitioner.currTrDepth;
    const unsigned numTBlocks = getNumberValidTBlocks( *cs.pcv );

    unsigned        compCbf[3] = { 0, 0, 0 };
    unsigned        cbfDepth   = 0;
    for( auto &currTU : cs.traverseTUs( currArea, partitioner.chType ) )
    {
      for( unsigned ch = 0; ch < numTBlocks; ch++ )
      {
        cbfDepth     = !isLuma( ComponentID( ch ) ) && cu.ispMode ? currDepth : currDepth + 1;
        compCbf[ch] |= ( TU::getCbfAtDepth( currTU, ComponentID( ch ), cbfDepth ) ? 1 : 0 );
      }
    }

    for (auto &currTU: cs.traverseTUs(currArea, partitioner.chType))
    {
      TU::setCbfAtDepth(currTU, COMPONENT_Y, currDepth, compCbf[COMPONENT_Y]);
      if (currArea.chromaFormat != CHROMA_400)
      {
        TU::setCbfAtDepth(currTU, COMPONENT_Cb, currDepth, compCbf[COMPONENT_Cb]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cr, currDepth, compCbf[COMPONENT_Cr]);
      }
    }
  }
  else
  {
    TransformUnit &tu = cs.addTU( CS::getArea( cs, area, partitioner.chType ), partitioner.chType );
    unsigned numBlocks = ::getNumberValidTBlocks( *cs.pcv );
    tu.checkTuNoResidual( partitioner.currPartIdx() );
    chromaCbfs.Cb &= !tu.noResidual;
    chromaCbfs.Cr &= !tu.noResidual;

    for( unsigned compID = COMPONENT_Y; compID < numBlocks; compID++ )
    {
      if( tu.blocks[compID].valid() )
      {
        tu.getCoeffs( ComponentID( compID ) ).fill( 0 );
        tu.getPcmbuf( ComponentID( compID ) ).fill( 0 );
      }
    }
    tu.depth = trDepth;
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    if( !isChroma( partitioner.chType ) )
    {
      if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, 1 );
      }
      else if( cu.sbtInfo && tu.noResidual )
      {
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, 0 );
      }
      else if( cu.sbtInfo && !chromaCbfsLastDepth.sigChroma( area.chromaFormat ) )
      {
        assert( !tu.noResidual );
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, 1 );
      }
      else
      {
        bool previousCbf       = false;
        bool rootCbfSoFar      = false;
        bool lastCbfIsInferred = false;
        if( cu.ispMode )
        {
          uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> g_aucLog2[tu.lheight()] : cu.lwidth() >> g_aucLog2[tu.lwidth()];
          if( subTuCounter == nTus - 1 )
          {
            TransformUnit* tuPointer = cu.firstTU;
            for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
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
            previousCbf = TU::getPrevTuCbfAtDepth( tu, COMPONENT_Y, trDepth );
          }
        }
        bool cbfY = lastCbfIsInferred ? true : cbf_comp( cs, tu.Y(), trDepth, previousCbf, cu.ispMode );
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, ( cbfY ? 1 : 0 ) );
      }
    }
    if( area.chromaFormat != CHROMA_400 && ( !cu.ispMode || chromaCbfISP ) )
    {
      TU::setCbfAtDepth( tu, COMPONENT_Cb, trDepth, ( chromaCbfs.Cb ? 1 : 0 ) );
      TU::setCbfAtDepth( tu, COMPONENT_Cr, trDepth, ( chromaCbfs.Cr ? 1 : 0 ) );
    }


    transform_unit( tu, cuCtx, chromaCbfs );
  }
}

bool CABACReader::cbf_comp( CodingStructure& cs, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
  const unsigned  ctxId = DeriveCtx::CtxQtCbf( area.compID, depth, prevCbf, useISP && isLuma( area.compID ) );
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID ];

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__QT_CBF, area.size(), area.compID);

#if JVET_N0413_RDPCM
  unsigned  cbf = 0;
  if( area.compID == COMPONENT_Y && cs.getCU( area.pos(), ChannelType( area.compID ) )->bdpcmMode )
  {
    cbf = m_BinDecoder.decodeBin( ctxSet( 4 ) );
  }
  else
  {
    cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
  }
#else
  const unsigned  cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
#endif

  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
  return cbf;
}

//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================

void CABACReader::mvd_coding( Mv &rMvd )
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_mvd    ( STATS__CABAC_BITS__MVD );
  CodingStatisticsClassType ctype_mvd_ep ( STATS__CABAC_BITS__MVD_EP );
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_mvd );

  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_mvd_ep );

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += exp_golomb_eqprob(1 );
    }
    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += exp_golomb_eqprob(1 );
    }
    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
}


//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================

void CABACReader::transform_unit( TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  CodingUnit& cu         = *tu.cu;
  bool        lumaOnly   = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbfLuma    = ( tu.cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma  = ( lumaOnly ? false : ( chromaCbfs.Cb || chromaCbfs.Cr ) );

  if( cbfLuma || cbfChroma )
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      if (!CS::isDualITree(*tu.cs) || isLuma(tu.chType))
      {
        cu_qp_delta(cu, cuCtx.qp, cu.qp);
        cuCtx.qp = cu.qp;
        cuCtx.isDQPCoded = true;
      }
    }
    if( cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded )
    {
      cu_chroma_qp_offset( cu );
      cuCtx.isChromaQpAdjCoded = true;
    }
    if( cbfLuma )
    {
      residual_coding( tu, COMPONENT_Y );
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( TU::hasCrossCompPredInfo( tu, compID ) )
        {
          cross_comp_pred( tu, compID );
        }
        if( tu.cbf[ compID ] )
        {
          residual_coding( tu, compID );
        }
      }
    }
  }
}

void CABACReader::cu_qp_delta( CodingUnit& cu, int predQP, int8_t& qp )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__DELTA_QP_EP );

  CHECK( predQP == std::numeric_limits<int>::max(), "Invalid predicted QP" );
  int qpY = predQP;
  int DQp = unary_max_symbol( Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( DQp >= CU_DQP_TU_CMAX )
  {
    DQp += exp_golomb_eqprob( CU_DQP_EG_k  );
  }
  if( DQp > 0 )
  {
    if( m_BinDecoder.decodeBinEP( ) )
    {
      DQp = -DQp;
    }
    int     qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
    qpY = ( (predQP + DQp + (MAX_QP + 1) + 2 * qpBdOffsetY) % ((MAX_QP + 1) + qpBdOffsetY)) - qpBdOffsetY;
  }
  qp = (int8_t)qpY;

  DTRACE( g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACReader::cu_chroma_qp_offset( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT, cu.blocks[cu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  // cu_chroma_qp_offset_flag
  int       length  = cu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListLen();
  unsigned  qpAdj   = m_BinDecoder.decodeBin( Ctx::ChromaQpAdjFlag() );
  if( qpAdj && length > 1 )
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol( Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
}

//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    RDPCMMode   explicit_rdpcm_mode     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

#if JVET_N0054_JOINT_CHROMA
void CABACReader::joint_cb_cr( TransformUnit& tu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__JOINT_CB_CR, tu.blocks[COMPONENT_Cr].lumaSize(), CHANNEL_TYPE_CHROMA );
  tu.jointCbCr = m_BinDecoder.decodeBin( Ctx::JointCbCrFlag( 0 ) );
}
#endif

void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

#if JVET_N0054_JOINT_CHROMA
  // Joint Cb-Cr residual mode is signalled if both Cb and Cr cbfs are true
  if ( compID == COMPONENT_Cr && TU::getCbf( tu, COMPONENT_Cb ) )
  {
    joint_cb_cr( tu );

    // No Cr residual in bitstream in joint Cb-Cr residual mode
    if ( tu.jointCbCr )
      return;
  }
#endif

  // parse transform skip and explicit rdpcm mode
  mts_coding         ( tu, compID );
  explicit_rdpcm_mode( tu, compID );

#if JVET_N0280_RESIDUAL_CODING_TS
#if JVET_N0413_RDPCM
  if( isLuma( compID ) && ( tu.mtsIdx == MTS_SKIP || tu.cu->bdpcmMode ) )
#else
  if( isLuma( compID ) && tu.mtsIdx == MTS_SKIP )
#endif
  {
    residual_codingTS( tu, compID );
    return;
  }
#endif

#if HEVC_USE_SIGN_HIDING
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
#endif

  // init coeff coding context
#if HEVC_USE_SIGN_HIDING
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
#else
  CoeffCodingContext  cctx    ( tu, compID );
#endif
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;

  // parse last coeff position
  cctx.setScanPosLast( last_sig_coeff( cctx, tu, compID ) );

  // parse subblocks
  const int stateTransTab = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
  int       state         = 0;


    for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
    {
      cctx.initSubblock       ( subSetId );
      if( ( tu.mtsIdx > MTS_SKIP || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
      {
        if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) ) || ( tu.blocks[ compID ].width == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth() ) ) )
        {
          continue;
        }
      }
      residual_coding_subblock( cctx, coeff, stateTransTab, state );
    }

}

void CABACReader::mts_coding( TransformUnit& tu, ComponentID compID )
{
  const CodingUnit  &cu = *tu.cu;
  const bool  tsAllowed = TU::isTSAllowed ( tu, compID );
  const bool mtsAllowed = TU::isMTSAllowed( tu, compID );

#if JVET_N0413_RDPCM
  if( tu.cu->bdpcmMode ) tu.mtsIdx = MTS_SKIP;
#endif
  if( !mtsAllowed && !tsAllowed ) return;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[compID], compID );

  int symbol = 0;
  int ctxIdx = 0;

  if( tsAllowed )
  {
    ctxIdx = 6;
    symbol = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );
    tu.mtsIdx = symbol ? MTS_DCT2_DCT2 : MTS_SKIP;
  }

  if( tu.mtsIdx != MTS_SKIP )
  {
    if( mtsAllowed )
    {
      ctxIdx = std::min( (int)cu.qtDepth, 5 );
      symbol = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );

      if( symbol )
      {
        ctxIdx    = 7;
        tu.mtsIdx = MTS_DST7_DST7; // mtsIdx = 2 -- 4
        for( int i = 0; i < 3; i++, ctxIdx++ )
        {
          symbol = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );
          tu.mtsIdx += symbol;

          if( !symbol )
          {
            break;
          }
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "mts_coding() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), tu.mtsIdx );
}

void CABACReader::isp_mode( CodingUnit& cu )
{
#if INCLUDE_ISP_CFG_FLAG
#if JVET_N0413_RDPCM
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || cu.ipcm || !cu.cs->sps->getUseISP() || cu.bdpcmMode )
#else
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || cu.ipcm || !cu.cs->sps->getUseISP() )
#endif
#else
#if JVET_N0413_RDPCM
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || cu.ipcm || cu.bdpcmMode )
#else
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || cu.ipcm )
#endif
#endif
  {
    cu.ispMode = NOT_INTRA_SUBPARTITIONS;
    return;
  }

  const ISPType allowedSplits = CU::canUseISPSplit( cu, getFirstComponentOfChannel( cu.chType ) );
  if( allowedSplits == NOT_INTRA_SUBPARTITIONS )
  {
    cu.ispMode = NOT_INTRA_SUBPARTITIONS;
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__ISP_MODE_FLAG );
  cu.ispMode = NOT_INTRA_SUBPARTITIONS;
  int symbol = m_BinDecoder.decodeBin( Ctx::ISPMode( 0 ) );

  if( symbol )
  {
    if( allowedSplits == HOR_INTRA_SUBPARTITIONS )
    {
      cu.ispMode = HOR_INTRA_SUBPARTITIONS;
    }
    else if( allowedSplits == VER_INTRA_SUBPARTITIONS )
    {
      cu.ispMode = VER_INTRA_SUBPARTITIONS;
    }
    else
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__ISP_SPLIT_FLAG );
      cu.ispMode = 1 + m_BinDecoder.decodeBin( Ctx::ISPMode( 1 ) );
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACReader::explicit_rdpcm_mode( TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;

  tu.rdpcm[compID] = RDPCM_OFF;

  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.mtsIdx==MTS_SKIP || cu.transQuantBypass ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__EXPLICIT_RDPCM_BITS, tu.blocks[tu.chType].lumaSize() );

    ChannelType chType = toChannelType( compID );
    if( m_BinDecoder.decodeBin( Ctx::RdpcmFlag( chType ) ) )
    {
      if( m_BinDecoder.decodeBin( Ctx::RdpcmDir( chType ) ) )
      {
        tu.rdpcm[compID] = RDPCM_VER;
      }
      else
      {
        tu.rdpcm[compID] = RDPCM_HOR;
      }
    }
  }
}

#if JVET_N0193_LFNST
void CABACReader::residual_lfnst_mode( CodingUnit& cu )
{
#if JVET_N0217_MATRIX_INTRAPRED
  if( cu.ispMode != NOT_INTRA_SUBPARTITIONS || cu.mipFlag == true ||
#else
  if( cu.ispMode != NOT_INTRA_SUBPARTITIONS ||
#endif
    ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 ) )
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__LFNST );

  if( cu.cs->sps->getUseLFNST() && CU::isIntra( cu ) && !CU::isLosslessCoded( cu ) )
  {
    const bool lumaFlag              = CS::isDualITree( *cu.cs ) ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag            = CS::isDualITree( *cu.cs ) ? ( isChroma( cu.chType ) ? true : false ) : true;
    bool nonZeroCoeffNonTs;
    bool nonZeroCoeffNonTsCorner8x8  = CU::getNumNonZeroCoeffNonTsCorner8x8( cu, lumaFlag, chromaFlag ) > 0;
    const int  nonZeroCoeffThr       = CS::isDualITree( *cu.cs ) ? ( isLuma( cu.chType ) ? LFNST_SIG_NZ_LUMA : LFNST_SIG_NZ_CHROMA ) : LFNST_SIG_NZ_LUMA + LFNST_SIG_NZ_CHROMA;
    nonZeroCoeffNonTs = CU::getNumNonZeroCoeffNonTs( cu, lumaFlag, chromaFlag ) > nonZeroCoeffThr;

    if( !nonZeroCoeffNonTs || nonZeroCoeffNonTsCorner8x8 )
    {
      cu.lfnstIdx = 0;
      return;
    }
  }
  else
  {
    cu.lfnstIdx = 0;
    return;
  }

#if !JVET_N0105_LFNST_CTX_MODELLING
  uint32_t ctxOff = 0;

  int intraMode = cu.firstPU->intraDir[ cu.chType ];
  if( intraMode == DM_CHROMA_IDX && !isLuma( cu.chType ) )
  {
    intraMode = PLANAR_IDX;
  }
  if( cu.chromaFormat == CHROMA_422 && !isLuma( cu.chType ) )
  {
    intraMode = g_chroma422IntraAngleMappingTable[ intraMode ];
  }
  ctxOff = PU::isLMCMode( intraMode ) || intraMode <= DC_IDX;
#endif

  unsigned cctx = 0;
  if( cu.firstTU->mtsIdx < MTS_DST7_DST7 && CS::isDualITree( *cu.cs ) ) cctx++;

#if JVET_N0105_LFNST_CTX_MODELLING
  uint32_t idxLFNST = m_BinDecoder.decodeBin( Ctx::LFNSTIdx( cctx ) );
#else
  uint32_t idxLFNST = m_BinDecoder.decodeBin( Ctx::LFNSTIdx( ctxOff + 4 * cctx ) );
#endif
  if( idxLFNST )
  {
#if JVET_N0105_LFNST_CTX_MODELLING
    idxLFNST += m_BinDecoder.decodeBinEP();
#else
    idxLFNST += m_BinDecoder.decodeBin( Ctx::LFNSTIdx( 2 + ctxOff + 4 * cctx ) );
#endif
  }
  cu.lfnstIdx = idxLFNST;

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}
#endif

int CABACReader::last_sig_coeff( CoeffCodingContext& cctx, TransformUnit& tu, ComponentID compID )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__LAST_SIG_X_Y, Size( cctx.width(), cctx.height() ), cctx.compID() );

  unsigned PosLastX = 0, PosLastY = 0;
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

  if( ( tu.mtsIdx > MTS_SKIP || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
  {
    maxLastPosX = ( tu.blocks[ compID ].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[ compID ].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }

  for( ; PosLastX < maxLastPosX; PosLastX++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastXCtxId( PosLastX ) ) )
    {
      break;
    }
  }
  for( ; PosLastY < maxLastPosY; PosLastY++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastYCtxId( PosLastY ) ) )
    {
      break;
    }
  }
  if( PosLastX > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastX - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastX = g_uiMinInGroup[ PosLastX ] + uiTemp;
  }
  if( PosLastY > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastY - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastY = g_uiMinInGroup[ PosLastY ] + uiTemp;
  }

  int blkPos;
#if HEVC_USE_MDCS
  if( cctx.scanType() == SCAN_VER )
  {
    blkPos = PosLastY + ( PosLastX * cctx.width() );
  }
  else
#endif
  {
    blkPos = PosLastX + ( PosLastY * cctx.width() );
  }

  int scanPos = 0;
  for( ; scanPos < cctx.maxNumCoeff() - 1; scanPos++ )
  {
    if( blkPos == cctx.blockPos( scanPos ) )
    {
      break;
    }
  }
  return scanPos;
}



void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, const int stateTransTable, int& state )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = ( isLast || !minSubPos );
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  uint8_t   ctxOffset[16];

  //===== decode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
#if HEVC_USE_SIGN_HIDING
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
#endif
  int       numNonZero    =  0;
  bool      is2x2subblock = ( cctx.log2CGSize() == 2 );
  int       remRegBins    = ( is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK );
  int       firstPosMode2 = minSubPos - 1;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      sigFlag = m_BinDecoder.decodeBin( sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
      cctx.sigCtxIdAbs( nextSigPos, coeff, state ); // required for setting variables that are needed for gtx/par context selection
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff           = cctx.ctxOffsetAbs();
      sigBlkPos[ numNonZero++ ] = blkPos;
#if HEVC_USE_SIGN_HIDING
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>( lastNZPos, nextSigPos );
#endif

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      unsigned gt1Flag = m_BinDecoder.decodeBin( cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      unsigned parFlag = 0;
      unsigned gt2Flag = 0;
      if( gt1Flag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbs( ctxOff ) );

        remRegBins--;
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);
        gt2Flag = m_BinDecoder.decodeBin( cctx.greater2CtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2Flag, cctx.greater2CtxIdAbs( ctxOff ) );
        remRegBins--;
      }
      coeff[ blkPos ] += 1 + parFlag + gt1Flag + (gt2Flag << 1);
    }

    state = ( stateTransTable >> ((state<<2)+((coeff[blkPos]&1)<<1)) ) & 3;
  }
  firstPosMode2 = nextSigPos;


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
#if JVET_N0188_UNIFY_RICEPARA
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
    ricePar = g_auiGoRiceParsCoeff[sumAll];
#endif
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    if( tcoeff >= 4 )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
      int       rem     = m_BinDecoder.decodeRemAbsEP( ricePar, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      tcoeff += (rem<<1);
#if !JVET_N0188_UNIFY_RICEPARA
      if( ricePar < 3 && rem > (3<<ricePar)-1 )
      {
        ricePar++;
      }
#endif
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
#if JVET_N0188_UNIFY_RICEPARA
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
#else
    int       sumAll    = cctx.templateAbsSum(scanPos, coeff);
#endif
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0[std::max(0, state - 1)][sumAll];
    int       rem       = m_BinDecoder.decodeRemAbsEP( rice, cctx.extPrec(), cctx.maxLog2TrDRange() );
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_escs);
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    TCoeff    tcoeff  = ( rem == pos0 ? 0 : rem < pos0 ? rem+1 : rem );
    state = ( stateTransTable >> ((state<<2)+((tcoeff&1)<<1)) ) & 3;
    if( tcoeff )
    {
      int        blkPos         = cctx.blockPos( scanPos );
      sigBlkPos[ numNonZero++ ] = blkPos;
#if HEVC_USE_SIGN_HIDING
      lastNZPos  = std::max<int>( lastNZPos, scanPos );
#endif
      coeff[blkPos] = tcoeff;
    }
  }

  //===== decode sign's =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
#if HEVC_USE_SIGN_HIDING
  const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
  unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );
#else
  unsigned        signPattern = m_BinDecoder.decodeBinsEP( numNonZero ) << ( 32 - numNonZero );
#endif

  //===== set final coefficents =====
  int sumAbs = 0;
#if HEVC_USE_SIGN_HIDING
  for( unsigned k = 0; k < numSigns; k++ )
#else
  for( unsigned k = 0; k < numNonZero; k++ )
#endif
  {
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
    signPattern         <<= 1;
  }
#if HEVC_USE_SIGN_HIDING
  if( numNonZero > numSigns )
  {
    int k                 = numSigns;
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
  }
#endif
}

#if JVET_N0280_RESIDUAL_CODING_TS
void CABACReader::residual_codingTS( TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
#if JVET_N0413_RDPCM
  CoeffCodingContext  cctx    ( tu, compID, false, tu.cu->bdpcmMode );
#else
  CoeffCodingContext  cctx    ( tu, compID, false );
#endif
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;

  cctx.setNumCtxBins( 2 * tu.lwidth()*tu.lheight() );

  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId );
    residual_coding_subblockTS( cctx, coeff );
  }
}

void CABACReader::residual_coding_subblockTS( CoeffCodingContext& cctx, TCoeff* coeff )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;
  unsigned    signPattern = 0;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = cctx.isLastSubSet() && cctx.noneSigGroup();
  if( !sigGroup )
  {
    if( cctx.isContextCoded() )
    {
      sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId( true ) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", sigGroup, cctx.sigGroupCtxId() );
    }
    else
    {
      sigGroup = m_BinDecoder.decodeBinEP( );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() EPbin=%d\n", sigGroup );
    }
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode absolute values =====
  const int inferSigPos   = minSubPos;
  int       numNonZero    =  0;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  for( ; nextSigPos <= minSubPos; nextSigPos++ )
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      if( cctx.isContextCoded() )
      {
        const unsigned sigCtxId = cctx.sigCtxIdAbsTS( nextSigPos, coeff );
        sigFlag = m_BinDecoder.decodeBin( sigCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      }
      else
      {
        sigFlag = m_BinDecoder.decodeBinEP( );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() EPbin=%d\n", sigFlag );
      }
    }

    if( sigFlag )
    {
      //===== decode sign's =====
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
      int sign;
      if( cctx.isContextCoded() )
      {
#if JVET_N0413_RDPCM
        sign = m_BinDecoder.decodeBin( Ctx::TsResidualSign(  cctx.bdpcm() ? 1 : 0 ) );
#else
        sign = m_BinDecoder.decodeBin( Ctx::TsResidualSign( toChannelType( cctx.compID() ) ) );
#endif
      }
      else
      {
        sign = m_BinDecoder.decodeBinEP( );
      }

      signPattern += ( sign << numNonZero );

      sigBlkPos[numNonZero++] = blkPos;

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      unsigned gt1Flag;
      if( cctx.isContextCoded() )
      {
        gt1Flag = m_BinDecoder.decodeBin( cctx.greaterXCtxIdAbsTS(0) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greaterXCtxIdAbsTS(0) );
      }
      else
      {
        gt1Flag = m_BinDecoder.decodeBinEP( );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() EPbin=%d\n", gt1Flag );
      }

      unsigned parFlag = 0;
      if( gt1Flag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        if( cctx.isContextCoded() )
        {
          parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbsTS() );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbsTS() );
        }
        else
        {
          parFlag = m_BinDecoder.decodeBinEP( );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() EPbin=%d\n", parFlag );
        }
      }
      coeff[ blkPos ] += 1 + parFlag + gt1Flag;
    }
  }

  int cutoffVal = 2;
  int numGtBins = 4;

  //===== 2nd PASS: gt2 =====
  for( int i = 0; i < numGtBins; i++ )
  {
    for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
    {
      TCoeff& tcoeff = coeff[cctx.blockPos( scanPos )];
      if( tcoeff >= cutoffVal )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt2 );
        unsigned gt2Flag;
        if( cctx.isContextCoded() )
        {
          gt2Flag = m_BinDecoder.decodeBin( cctx.greaterXCtxIdAbsTS(cutoffVal>>1) );
          tcoeff += ( gt2Flag << 1 );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2Flag, cctx.greaterXCtxIdAbsTS(cutoffVal>>1), scanPos, tcoeff );
        }
        else
        {
          gt2Flag = m_BinDecoder.decodeBinEP( );
          tcoeff += ( gt2Flag << 1 );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() EPbin=%d sp=%d coeff=%d\n", i, gt2Flag, scanPos, tcoeff );
        }
      }
    }
    cutoffVal += 2;
  }

  //===== 3rd PASS: Go-rice codes =====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
    if( tcoeff >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      int       rem  = m_BinDecoder.decodeRemAbsEP( rice, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );
      tcoeff += ( rem << 1 );
    }
  }

  //===== set final coefficents =====
  for( unsigned k = 0; k < numNonZero; k++ )
  {
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    coeff[ sigBlkPos[k] ] = ( signPattern & 1 ? -AbsCoeff : AbsCoeff );
    signPattern         >>= 1;
  }
}
#endif


//================================================================================
//  clause 7.3.8.12
//--------------------------------------------------------------------------------
//    void  cross_comp_pred( tu, compID )
//================================================================================

void CABACReader::cross_comp_pred( TransformUnit& tu, ComponentID compID )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__CROSS_COMPONENT_PREDICTION, tu.blocks[compID], compID);

  signed char alpha   = 0;
  unsigned    ctxBase = ( compID == COMPONENT_Cr ? 5 : 0 );
  unsigned    symbol  = m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase) );
  if( symbol )
  {
    // Cross-component prediction alpha is non-zero.
    symbol = m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase+1) );
    if( symbol )
    {
      // alpha is 2 (symbol=1), 4(symbol=2) or 8(symbol=3).
      // Read up to two more bits
      symbol += unary_max_symbol( Ctx::CrossCompPred(ctxBase+2), Ctx::CrossCompPred(ctxBase+3), 2 );
    }
    alpha = ( 1 << symbol );
    if( m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase+4) ) )
    {
      alpha = -alpha;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
  tu.compAlpha[compID] = alpha;
}



//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol  )
{
  unsigned onesRead = 0;
  while( onesRead < maxSymbol && m_BinDecoder.decodeBin( onesRead == 0 ? ctxId0 : ctxIdN ) == 1 )
  {
    ++onesRead;
  }
  return onesRead;
}


unsigned CABACReader::unary_max_eqprob( unsigned maxSymbol )
{
  for( unsigned k = 0; k < maxSymbol; k++ )
  {
    if( !m_BinDecoder.decodeBinEP() )
    {
      return k;
    }
  }
  return maxSymbol;
}


unsigned CABACReader::exp_golomb_eqprob( unsigned count )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while( bit )
  {
    bit     = m_BinDecoder.decodeBinEP( );
    symbol += bit << count++;
  }
  if( --count )
  {
    symbol += m_BinDecoder.decodeBinsEP( count );
  }
  return symbol;
}

#if JVET_N0217_MATRIX_INTRAPRED
unsigned CABACReader::code_unary_fixed( unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  unsigned idx;
  bool unary = m_BinDecoder.decodeBin( ctxId );
  if( unary )
  {
    idx = unary_max_eqprob( unary_max );
  }
  else
  {
    idx = unary_max + 1 + m_BinDecoder.decodeBinsEP( fixed );
  }
  return idx;
}

void CABACReader::mip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->getUseMIP() )
  {
    cu.mipFlag = false;
    return;
  }
  if( cu.lwidth() > MIP_MAX_WIDTH || cu.lheight() > MIP_MAX_HEIGHT )
  {
    cu.mipFlag = false;
    return;
  }
  if( !mipModesAvailable( cu.Y() ) )
  {
    cu.mipFlag = false;
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  cu.mipFlag = m_BinDecoder.decodeBin( Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACReader::mip_pred_modes( CodingUnit &cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.Y().valid() )
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    mip_pred_mode( pu );
  }
}

void CABACReader::mip_pred_mode( PredictionUnit &pu )
{
  CHECK( pu.lwidth() > MIP_MAX_WIDTH || pu.lheight() > MIP_MAX_HEIGHT, "Error: block size not supported" );

  const int numModes   = getNumModesMip( pu.Y() ); CHECKD( numModes > MAX_NUM_MIP_MODE, "Error: too many MIP modes" );

  int      unaryMax    = NUM_MPM_MIP - 1;
  int      fixedLength = getNumEpBinsMip( pu.Y() );
  unsigned modeIdx     = code_unary_fixed( Ctx::MipMode( 0 ), unaryMax, fixedLength );
  CHECK( modeIdx >= numModes, "modeIdx out of range" );

  // derive true MIP mode from modeIdx
  unsigned mpm[NUM_MPM_MIP];
  PU::getMipMPMs(pu, mpm);

  if (modeIdx < NUM_MPM_MIP)
  {
    pu.intraDir[CHANNEL_TYPE_LUMA] = mpm[modeIdx];
  }
  else
  {
    modeIdx -= NUM_MPM_MIP;
    std::sort(mpm, mpm + NUM_MPM_MIP);

    for( unsigned i = 0; i < NUM_MPM_MIP; i++ )
    {
      modeIdx += (modeIdx >= mpm[i]);
    }
    pu.intraDir[CHANNEL_TYPE_LUMA] = modeIdx;
  }
  CHECKD(pu.intraDir[CHANNEL_TYPE_LUMA] < 0 || pu.intraDir[CHANNEL_TYPE_LUMA] >= numModes, "Invalid MIP mode");

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA] );
}
#endif
