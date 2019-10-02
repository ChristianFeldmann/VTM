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

/** \file     VLCWriter.cpp
 *  \brief    Writer for high level syntax
 */

#include "VLCWriter.h"
#include "SEIwrite.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Picture.h" // th remove this
#include "CommonLib/dtrace_next.h"
#include "EncAdaptiveLoopFilter.h"
#include "CommonLib/AdaptiveLoopFilter.h"

//! \ingroup EncoderLib
//! \{

#if ENABLE_TRACING

void  VLCWriter::xWriteCodeTr (uint32_t value, uint32_t  length, const char *pSymbolName)
{
  xWriteCode (value,length);

  if( g_HLSTraceEnable )
  {
    if( length < 10 )
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %d\n", pSymbolName, length, value );
    }
    else
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %d\n", pSymbolName, length, value );
    }
  }
}

void  VLCWriter::xWriteUvlcTr (uint32_t value, const char *pSymbolName)
{
  xWriteUvlc (value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %d\n", pSymbolName, value );
  }
}

void  VLCWriter::xWriteSvlcTr (int value, const char *pSymbolName)
{
  xWriteSvlc(value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, value );
  }
}

void  VLCWriter::xWriteFlagTr(uint32_t value, const char *pSymbolName)
{
  xWriteFlag(value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, value );
  }
}

bool g_HLSTraceEnable = true;

#endif


void VLCWriter::xWriteCode     ( uint32_t uiCode, uint32_t uiLength )
{
  CHECK( uiLength == 0, "Code of length '0' not supported" );
  m_pcBitIf->write( uiCode, uiLength );
}

void VLCWriter::xWriteUvlc     ( uint32_t uiCode )
{
  uint32_t uiLength = 1;
  uint32_t uiTemp = ++uiCode;

  CHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  m_pcBitIf->write( 0, uiLength >> 1);
  m_pcBitIf->write( uiCode, (uiLength+1) >> 1);
}

void VLCWriter::xWriteSvlc     ( int iCode )
{
  uint32_t uiCode = uint32_t( iCode <= 0 ? (-iCode)<<1 : (iCode<<1)-1);
  xWriteUvlc( uiCode );
}

void VLCWriter::xWriteFlag( uint32_t uiCode )
{
  m_pcBitIf->write( uiCode, 1 );
}

void VLCWriter::xWriteRbspTrailingBits()
{
  WRITE_FLAG( 1, "rbsp_stop_one_bit");
  int cnt = 0;
  while (m_pcBitIf->getNumBitsUntilByteAligned())
  {
    WRITE_FLAG( 0, "rbsp_alignment_zero_bit");
    cnt++;
  }
  CHECK(cnt>=8, "More than '8' alignment bytes read");
}

void AUDWriter::codeAUD(OutputBitstream& bs, const int pictureType)
{
#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  CHECK(pictureType >= 3, "Invalid picture type");
  setBitstream(&bs);
  WRITE_CODE(pictureType, 3, "pic_type");
  xWriteRbspTrailingBits();
}

#if JVET_O0244_DELTA_POC
void HLSWriter::xCodeRefPicList( const ReferencePictureList* rpl, bool isLongTermPresent, uint32_t ltLsbBitsCount, const bool isForbiddenZeroDeltaPoc )
#else
void HLSWriter::xCodeRefPicList(const ReferencePictureList* rpl, bool isLongTermPresent, uint32_t ltLsbBitsCount)
#endif
{
  WRITE_UVLC(rpl->getNumberOfShorttermPictures() + rpl->getNumberOfLongtermPictures(), "num_ref_entries[ listIdx ][ rplsIdx ]");
  uint32_t numRefPic = rpl->getNumberOfShorttermPictures() + rpl->getNumberOfLongtermPictures();
#if JVET_N0100_PROPOSAL1
  if (isLongTermPresent)
  {
    WRITE_FLAG(rpl->getLtrpInSliceHeaderFlag(), "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
  }
#endif
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;
  for (int ii = 0; ii < numRefPic; ii++)
  {
#if JVET_N0494_DRAP
    if (isLongTermPresent)
#else
    if (rpl->getNumberOfLongtermPictures() > 0)
#endif
      WRITE_FLAG(!rpl->isRefPicLongterm(ii), "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");
    if (!rpl->isRefPicLongterm(ii))
    {
      if (firstSTRP)
      {
        firstSTRP = false;
        deltaValue = prevDelta = rpl->getRefPicIdentifier(ii);
      }
      else
      {
        deltaValue = rpl->getRefPicIdentifier(ii) - prevDelta;
        prevDelta = rpl->getRefPicIdentifier(ii);
      }
      unsigned int absDeltaValue = (deltaValue < 0) ? 0 - deltaValue : deltaValue;
#if JVET_O0244_DELTA_POC
      if( isForbiddenZeroDeltaPoc )
      {
        CHECK( !absDeltaValue, "Zero delta POC is not used without WP" );
        WRITE_UVLC( absDeltaValue - 1, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]" );
      }
      else
#endif
      WRITE_UVLC(absDeltaValue, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
      if (absDeltaValue > 0)
        WRITE_FLAG((deltaValue < 0) ? 0 : 1, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");  //0  means negative delta POC : 1 means positive
    }
#if JVET_N0100_PROPOSAL1
    else if (!rpl->getLtrpInSliceHeaderFlag())
#else
    else
#endif
    {
      WRITE_CODE(rpl->getRefPicIdentifier(ii), ltLsbBitsCount, "poc_lsb_lt[listIdx][rplsIdx][i]");
    }
  }
}

#if JVET_O1136_TS_BDPCM_SIGNALLING
void HLSWriter::codePPS( const PPS* pcPPS, const SPS* pcSPS )
#else
void HLSWriter::codePPS( const PPS* pcPPS )
#endif
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif

  WRITE_UVLC( pcPPS->getPPSId(),                             "pps_pic_parameter_set_id" );
  WRITE_UVLC( pcPPS->getSPSId(),                             "pps_seq_parameter_set_id" );

#if JVET_O1164_PS
  WRITE_UVLC( pcPPS->getPicWidthInLumaSamples(), "pic_width_in_luma_samples" );
  WRITE_UVLC( pcPPS->getPicHeightInLumaSamples(), "pic_height_in_luma_samples" );
  Window conf = pcPPS->getConformanceWindow();

  WRITE_FLAG( conf.getWindowEnabledFlag(), "conformance_window_flag" );
  if( conf.getWindowEnabledFlag() )
  {
    WRITE_UVLC( conf.getWindowLeftOffset(),   "conf_win_left_offset" );
    WRITE_UVLC( conf.getWindowRightOffset(),  "conf_win_right_offset" );
    WRITE_UVLC( conf.getWindowTopOffset(),    "conf_win_top_offset" );
    WRITE_UVLC( conf.getWindowBottomOffset(), "conf_win_bottom_offset" );
  }
#endif

  WRITE_FLAG( pcPPS->getOutputFlagPresentFlag() ? 1 : 0,     "output_flag_present_flag" );
  WRITE_CODE( pcPPS->getNumExtraSliceHeaderBits(), 3,        "num_extra_slice_header_bits");
  WRITE_FLAG( pcPPS->getCabacInitPresentFlag() ? 1 : 0,   "cabac_init_present_flag" );
  WRITE_UVLC( pcPPS->getNumRefIdxL0DefaultActive()-1,     "num_ref_idx_l0_default_active_minus1");
  WRITE_UVLC( pcPPS->getNumRefIdxL1DefaultActive()-1,     "num_ref_idx_l1_default_active_minus1");
  WRITE_FLAG(pcPPS->getRpl1IdxPresentFlag() ? 1 : 0, "rpl1IdxPresentFlag");

  #if JVET_O0238_PPS_OR_SLICE
  WRITE_FLAG( pcPPS->getConstantSliceHeaderParamsEnabledFlag(),              "constant_slice_header_params_enabled_flag");
  if ( pcPPS->getConstantSliceHeaderParamsEnabledFlag() ) {
    WRITE_CODE( pcPPS->getPPSDepQuantEnabledIdc(), 2,                        "pps_dep_quant_enabled_idc");
    WRITE_CODE( pcPPS->getPPSRefPicListSPSIdc0(), 2,                         "pps_ref_pic_list_sps_idc[0]");
    WRITE_CODE( pcPPS->getPPSRefPicListSPSIdc1(), 2,                         "pps_ref_pic_list_sps_idc[1]");
    WRITE_CODE( pcPPS->getPPSTemporalMVPEnabledIdc(), 2,                     "pps_temporal_mvp_enabled_idc");
    WRITE_CODE( pcPPS->getPPSMvdL1ZeroIdc(), 2,                              "pps_mvd_l1_zero_idc");
    WRITE_CODE( pcPPS->getPPSCollocatedFromL0Idc(), 2,                       "pps_collocated_from_l0_idc");
    WRITE_UVLC( pcPPS->getPPSSixMinusMaxNumMergeCandPlus1(),                 "pps_six_minus_max_num_merge_cand_plus1");
    WRITE_UVLC( pcPPS->getPPSFiveMinusMaxNumSubblockMergeCandPlus1(),        "pps_five_minus_max_num_subblock_merge_cand_plus1");
    WRITE_UVLC( pcPPS->getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1(),  "pps_max_num_merge_cand_minus_max_num_triangle_cand_plus1");
  }
#endif

  WRITE_SVLC( pcPPS->getPicInitQPMinus26(),                  "init_qp_minus26");
  WRITE_FLAG( pcPPS->getConstrainedIntraPred() ? 1 : 0,      "constrained_intra_pred_flag" );
#if JVET_O1136_TS_BDPCM_SIGNALLING
  if (pcSPS->getTransformSkipEnabledFlag())
  {
    WRITE_UVLC(pcPPS->getLog2MaxTransformSkipBlockSize() - 2, "log2_max_transform_skip_block_size_minus2");
  }
#else
  WRITE_FLAG( pcPPS->getUseTransformSkip() ? 1 : 0,  "transform_skip_enabled_flag" );
#endif
  WRITE_FLAG( pcPPS->getUseDQP() ? 1 : 0, "cu_qp_delta_enabled_flag" );
  if ( pcPPS->getUseDQP() )
  {
    WRITE_UVLC( pcPPS->getCuQpDeltaSubdiv(), "cu_qp_delta_subdiv" );
  }

  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cb), "pps_cb_qp_offset" );
  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cr), "pps_cr_qp_offset" );
  WRITE_SVLC( pcPPS->getQpOffset(JOINT_CbCr),   "pps_joint_cbcr_qp_offset" );

  WRITE_FLAG( pcPPS->getSliceChromaQpFlag() ? 1 : 0,          "pps_slice_chroma_qp_offsets_present_flag" );

  WRITE_FLAG(uint32_t(pcPPS->getCuChromaQpOffsetEnabledFlag()),         "cu_chroma_qp_offset_enabled_flag" );
  if (pcPPS->getCuChromaQpOffsetEnabledFlag())
  {
    WRITE_UVLC(pcPPS->getCuChromaQpOffsetSubdiv(),                      "cu_chroma_qp_offset_subdiv");
    WRITE_UVLC(pcPPS->getChromaQpOffsetListLen() - 1,                   "chroma_qp_offset_list_len_minus1");
    /* skip zero index */
    for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx < pcPPS->getChromaQpOffsetListLen(); cuChromaQpOffsetIdx++)
    {
      WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CbOffset,     "cb_qp_offset_list[i]");
      WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CrOffset,     "cr_qp_offset_list[i]");
#if JVET_O1168_CU_CHROMA_QP_OFFSET
      WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1).u.comp.JointCbCrOffset, "joint_cbcr_qp_offset_list[i]");
#endif
    }
  }

  WRITE_FLAG( pcPPS->getUseWP() ? 1 : 0,  "weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcPPS->getWPBiPred() ? 1 : 0, "weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)
  WRITE_FLAG( pcPPS->getTransquantBypassEnabledFlag()  ? 1 : 0, "transquant_bypass_enabled_flag" );

  WRITE_FLAG( pcPPS->getSingleTileInPicFlag() ? 1 : 0, "single_tile_in_pic_flag" );
  if (!pcPPS->getSingleTileInPicFlag())
  {
    WRITE_FLAG( pcPPS->getUniformTileSpacingFlag() ? 1 : 0, "uniform_tile_spacing_flag" );
    if (pcPPS->getUniformTileSpacingFlag())
    {
      WRITE_UVLC( pcPPS->getTileColsWidthMinus1(),   "tile_cols_width_minus1" );
      WRITE_UVLC( pcPPS->getTileRowsHeightMinus1(),  "tile_rows_height_minus1" );
    }
    else
    {
      WRITE_UVLC( pcPPS->getNumTileColumnsMinus1(), "num_tile_columns_minus1" );
      WRITE_UVLC( pcPPS->getNumTileRowsMinus1(),    "num_tile_rows_minus1" );

      CHECK( ((pcPPS->getNumTileColumnsMinus1() + 1) * (pcPPS->getNumTileRowsMinus1() + 1)) < 2, "tile colums * rows must be > 1 when explicitly signalled.");

      for (int i = 0; i < pcPPS->getNumTileColumnsMinus1(); i++)
      {
        WRITE_UVLC( pcPPS->getTileColumnWidth(i) - 1, "tile_column_width_minus1" );
      }
      for (int i = 0; i < pcPPS->getNumTileRowsMinus1(); i++)
      {
        WRITE_UVLC( pcPPS->getTileRowHeight(i) - 1, "tile_row_height_minus1" );
      }
    }
    WRITE_FLAG( pcPPS->getBrickSplittingPresentFlag() ? 1 : 0, "brick_splitting_present_flag" );
#if JVET_O0176_PROPOSAL3
    if (pcPPS->getBrickSplittingPresentFlag())
    {
      CHECK(pcPPS->getRectSliceFlag() != true, "rect_slice_flag must be equal to 1 for brick_splitting_present_flag equal to 1");
    }
#endif

#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
    int numTilesInPic = (pcPPS->getNumTileColumnsMinus1() + 1) * (pcPPS->getNumTileRowsMinus1() + 1);
#else
    int numTilesInPic = pcPPS->getUniformTileSpacingFlag() ? 0 : (pcPPS->getNumTileColumnsMinus1() + 1) * (pcPPS->getNumTileRowsMinus1() + 1);
#endif
#if JVET_O0236_PPS_PARSING_DEPENDENCY
    if (pcPPS->getUniformTileSpacingFlag() && pcPPS->getBrickSplittingPresentFlag())
    {
      WRITE_UVLC(numTilesInPic - 1, "num_tiles_in_pic_minus1");
    }
#endif

    for( int i = 0; pcPPS->getBrickSplittingPresentFlag()  &&  i < numTilesInPic; i++ )
    {
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
      if (pcPPS->getTileHeight(i) > 1)
      {
#endif
        WRITE_FLAG(pcPPS->getBrickSplitFlag(i) ? 1 : 0, "brick_split_flag [i]");
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
      }
#endif
      if( pcPPS->getBrickSplitFlag(i) )
      {
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
        if (pcPPS->getTileHeight(i) > 2)
        {
#endif
          WRITE_FLAG(pcPPS->getUniformBrickSpacingFlag(i) ? 1 : 0, "uniform_brick_spacing_flag [i]");
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
        }
#endif
        if( pcPPS->getUniformBrickSpacingFlag(i) )
          WRITE_UVLC( pcPPS->getBrickHeightMinus1(i), "brick_height_minus1" );
        else
        {
#if JVET_O0173_O0176_O0338_NUMBRICK_M2
          WRITE_UVLC(pcPPS->getNumBrickRowsMinus2(i), "num_brick_rows_minus2 [i]");
          for (int j = 0; j <= pcPPS->getNumBrickRowsMinus2(i); j++)
            WRITE_UVLC(pcPPS->getBrickRowHeightMinus1(i, j), "brick_row_height_minus1 [i][j]");
#else                    
          WRITE_UVLC( pcPPS->getNumBrickRowsMinus1(i), "num_brick_rows_minus1 [i]" );
          for(int j = 0; j < pcPPS->getNumBrickRowsMinus1(i); j++ )
            WRITE_UVLC( pcPPS->getBrickRowHeightMinus1(i,j), "brick_row_height_minus1 [i][j]" );
#endif 
        }
      }
    }

    WRITE_FLAG( pcPPS->getSingleBrickPerSliceFlag() ? 1 : 0, "single_brick_per_slice_flag" );
    if (!pcPPS->getSingleBrickPerSliceFlag())
    {
      WRITE_FLAG( pcPPS->getRectSliceFlag() ? 1 : 0, "rect_slice_flag" );
    }
    else
    {
      // make sure rect_slice_flag is set
      CHECK (pcPPS->getRectSliceFlag()!=true, "RectSliceFlag must be equal to 1 for single_brick_per_slice_flag equal to 1");
    }

    if (pcPPS->getRectSliceFlag() && !pcPPS->getSingleBrickPerSliceFlag())
    {
      WRITE_UVLC( pcPPS->getNumSlicesInPicMinus1(), "num_slices_in_pic_minus1" );
      int numSlicesInPic = pcPPS->getNumSlicesInPicMinus1() + 1;
#if !JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
      int numTilesInPic = (pcPPS->getNumTileColumnsMinus1() + 1) * (pcPPS->getNumTileRowsMinus1() + 1);
#endif
      int codeLength = ceilLog2(numTilesInPic);
#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
      WRITE_UVLC(codeLength, "bottom_right_brick_idx_length_minus1 ");
#else
      int codeLength2 = codeLength;
#endif
      for (int i = 0; i < numSlicesInPic; ++i)
      {
#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
        int delta = (i == 0) ? pcPPS->getBottomRightBrickIdx(i) : pcPPS->getBottomRightBrickIdx(i) - pcPPS->getBottomRightBrickIdx(i - 1);
        int sign = (delta > 0) ? 1 : 0;
        WRITE_CODE(delta, codeLength, "bottom_right_brick_idx_delta");
        WRITE_FLAG(sign, "brick_idx_delta_sign_flag");
#else
        if (i > 0)
        {
          WRITE_CODE(pcPPS->getTopLeftBrickIdx(i), codeLength, "top_left_brick_idx ");
          codeLength2 = ceilLog2((numTilesInPic - pcPPS->getTopLeftBrickIdx(i) < 2) ? 2 : numTilesInPic - pcPPS->getTopLeftBrickIdx(i));
        }
        WRITE_CODE(pcPPS->getBottomRightBrickIdx(i) - pcPPS->getTopLeftBrickIdx(i), codeLength2, "bottom_right_brick_idx_delta");
#endif
      }
    }

    WRITE_FLAG( pcPPS->getLoopFilterAcrossBricksEnabledFlag() ? 1 : 0, "loop_filter_across_bricks_enabled_flag" );
    if (pcPPS->getLoopFilterAcrossBricksEnabledFlag())
    {
      WRITE_FLAG( pcPPS->getLoopFilterAcrossSlicesEnabledFlag() ? 1 : 0, "loop_filter_across_slices_enabled_flag" );
    }
  }
  else
  {
    // make sure single brick per slice is set by encoder such that the behaviour is same as for setting it to true
    CHECK(pcPPS->getSingleBrickPerSliceFlag() != true, "SingleBrickPerSliceFlag must be set to 1 when not present");
    // make sure rect_slice_flag is set
    CHECK (pcPPS->getRectSliceFlag()!=true, "RectSliceFlag must be equalt to 1 for single_tile_in_pic_flag equal to 1");
  }

  if (pcPPS->getRectSliceFlag())
  {
    WRITE_FLAG( pcPPS->getSignalledSliceIdFlag() ? 1 : 0, "signalled_slice_id_flag" );
    if (pcPPS->getSignalledSliceIdFlag())
    {
      WRITE_UVLC( pcPPS->getSignalledSliceIdLengthMinus1(), "signalled_slice_id_length_minus1" );
      int signalledTileGroupIdLength = pcPPS->getSignalledSliceIdLengthMinus1() + 1;
      int numTileGroupsInPic = pcPPS->getNumSlicesInPicMinus1() + 1;
      for (int i = 0; i < numTileGroupsInPic; ++i)
      {
        WRITE_CODE (pcPPS->getSliceId(i), signalledTileGroupIdLength, "slice_id" );
      }
    }
  }


  WRITE_FLAG( pcPPS->getEntropyCodingSyncEnabledFlag() ? 1 : 0, "entropy_coding_sync_enabled_flag" );

  WRITE_FLAG( pcPPS->getDeblockingFilterControlPresentFlag()?1 : 0,       "deblocking_filter_control_present_flag");
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    WRITE_FLAG( pcPPS->getDeblockingFilterOverrideEnabledFlag() ? 1 : 0,  "deblocking_filter_override_enabled_flag" );
    WRITE_FLAG( pcPPS->getPPSDeblockingFilterDisabledFlag() ? 1 : 0,      "pps_deblocking_filter_disabled_flag" );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      WRITE_SVLC( pcPPS->getDeblockingFilterBetaOffsetDiv2(),             "pps_beta_offset_div2" );
      WRITE_SVLC( pcPPS->getDeblockingFilterTcOffsetDiv2(),               "pps_tc_offset_div2" );
    }
  }

  WRITE_FLAG( pcPPS->getLoopFilterAcrossVirtualBoundariesDisabledFlag() ? 1 : 0,     "pps_loop_filter_across_virtual_boundaries_disabled_flag" );
  if( pcPPS->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
  {
    WRITE_CODE( pcPPS->getNumVerVirtualBoundaries(), 2,                              "pps_num_ver_virtual_boundaries");
    for( unsigned i = 0; i < pcPPS->getNumVerVirtualBoundaries(); i++ )
    {
      WRITE_CODE(pcPPS->getVirtualBoundariesPosX(i) >> 3, 13,                        "pps_virtual_boundaries_pos_x");
    }
    WRITE_CODE( pcPPS->getNumHorVirtualBoundaries(), 2,                              "pps_num_hor_virtual_boundaries");
    for( unsigned i = 0; i < pcPPS->getNumHorVirtualBoundaries(); i++ )
    {
      WRITE_CODE(pcPPS->getVirtualBoundariesPosY(i) >> 3, 13,                        "pps_virtual_boundaries_pos_y");
    }
  }

#if !JVET_O0299_APS_SCALINGLIST
  WRITE_FLAG( pcPPS->getScalingListPresentFlag() ? 1 : 0,                          "pps_scaling_list_data_present_flag" );
  if( pcPPS->getScalingListPresentFlag() )
  {
    codeScalingList( pcPPS->getScalingList() );
  }
#endif
  WRITE_UVLC( pcPPS->getLog2ParallelMergeLevelMinus2(), "log2_parallel_merge_level_minus2");
  WRITE_FLAG( pcPPS->getSliceHeaderExtensionPresentFlag() ? 1 : 0, "slice_segment_header_extension_present_flag");

  bool pps_extension_present_flag=false;
  bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS]={false};

#if !JVET_O1136_TS_BDPCM_SIGNALLING
  pps_extension_flags[PPS_EXT__REXT] = pcPPS->getPpsRangeExtension().settingsDifferFromDefaults(pcPPS->getUseTransformSkip());

  // Other PPS extension flags checked here.

  for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
  {
    pps_extension_present_flag|=pps_extension_flags[i];
  }
#endif
  WRITE_FLAG( (pps_extension_present_flag?1:0), "pps_extension_present_flag" );

  if (pps_extension_present_flag)
  {
#if ENABLE_TRACING /*|| RExt__DECODER_DEBUG_BIT_STATISTICS*/
    static const char *syntaxStrings[]={ "pps_range_extension_flag",
      "pps_multilayer_extension_flag",
      "pps_extension_6bits[0]",
      "pps_extension_6bits[1]",
      "pps_extension_6bits[2]",
      "pps_extension_6bits[3]",
      "pps_extension_6bits[4]",
      "pps_extension_6bits[5]" };
#endif

    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      WRITE_FLAG( pps_extension_flags[i]?1:0, syntaxStrings[i] );
    }

    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
        case PPS_EXT__REXT:
        {
          const PPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();

          WRITE_FLAG((ppsRangeExtension.getCrossComponentPredictionEnabledFlag() ? 1 : 0), "cross_component_prediction_enabled_flag" );

          WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA),           "log2_sao_offset_scale_luma"   );
          WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA),         "log2_sao_offset_scale_chroma" );
        }
        break;
        default:
          CHECK(pps_extension_flags[i]==false, "Unknown PPS extension signalled"); // Should never get here with an active PPS extension flag.
          break;
        } // switch
      } // if flag present
    } // loop over PPS flags
  } // pps_extension_present_flag is non-zero
  xWriteRbspTrailingBits();
}

void HLSWriter::codeAPS( APS* pcAPS )
{
#if ENABLE_TRACING
  xTraceAPSHeader();
#endif

  WRITE_CODE(pcAPS->getAPSId(), 5, "adaptation_parameter_set_id");
#if JVET_O0245_VPS_DPS_APS
  WRITE_CODE( (int)pcAPS->getAPSType(), 3, "aps_params_type" );
#else
  WRITE_CODE(pcAPS->getAPSType(), 3, "aps_params_type");
#endif

  if (pcAPS->getAPSType() == ALF_APS)
  {
    codeAlfAps(pcAPS);
  }
  else if (pcAPS->getAPSType() == LMCS_APS)
  {
    codeLmcsAps (pcAPS);
  }
#if JVET_O0299_APS_SCALINGLIST
  else if( pcAPS->getAPSType() == SCALING_LIST_APS )
  {
    codeScalingListAps( pcAPS );
  }
#endif
  WRITE_FLAG(0, "aps_extension_flag");   //Implementation when this flag is equal to 1 should be added when it is needed. Currently in the spec we don't have case when this flag is equal to 1
  xWriteRbspTrailingBits();
}

void HLSWriter::codeAlfAps( APS* pcAPS )
{
  AlfParam param = pcAPS->getAlfAPSParam();

  WRITE_FLAG(param.newFilterFlag[CHANNEL_TYPE_LUMA], "alf_luma_new_filter");
  WRITE_FLAG(param.newFilterFlag[CHANNEL_TYPE_CHROMA], "alf_chroma_new_filter");

  if (param.newFilterFlag[CHANNEL_TYPE_LUMA])
  {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    WRITE_FLAG( param.nonLinearFlag[CHANNEL_TYPE_LUMA][0], "alf_luma_clip" );
#else
    WRITE_FLAG( param.nonLinearFlag[CHANNEL_TYPE_LUMA], "alf_luma_clip" );
#endif

#if JVET_O0491_HLS_CLEANUP
    WRITE_UVLC(param.numLumaFilters - 1, "alf_luma_num_filters_signalled_minus1");
#else
    xWriteTruncBinCode(param.numLumaFilters - 1, MAX_NUM_ALF_CLASSES);  //number_of_filters_minus1
#endif
    if (param.numLumaFilters > 1)
    {
#if JVET_O0491_HLS_CLEANUP
      const int length =  ceilLog2( param.numLumaFilters);
#endif
      for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
      {
#if JVET_O0491_HLS_CLEANUP
        WRITE_CODE(param.filterCoeffDeltaIdx[i], length, "alf_luma_coeff_delta_idx" );
#else
        xWriteTruncBinCode((uint32_t)param.filterCoeffDeltaIdx[i], param.numLumaFilters);  //filter_coeff_delta[i]
#endif
      }
    }
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    WRITE_FLAG(param.fixedFilterSetIndex > 0 ? 1 : 0, "fixed_filter_set_flag");
    if (param.fixedFilterSetIndex > 0)
    {
      xWriteTruncBinCode(param.fixedFilterSetIndex - 1, NUM_FIXED_FILTER_SETS);
      WRITE_FLAG(param.fixedFilterPattern, "fixed_filter_flag_pattern");
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
        if (param.fixedFilterPattern > 0)
        {
          WRITE_FLAG(param.fixedFilterIdx[classIdx], "fixed_filter_flag");
        }
        else
        {
          CHECK(param.fixedFilterIdx[classIdx] != 1, "Disabled fixed filter");
        }
      }
    }
#endif
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    alfFilter(param, false, 0);
#else
    alfFilter(param, false);
#endif

  }
  if (param.newFilterFlag[CHANNEL_TYPE_CHROMA])
  {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    if( MAX_NUM_ALF_ALTERNATIVES_CHROMA > 1 )
      WRITE_UVLC( param.numAlternativesChroma - 1, "alf_chroma_num_alts_minus1" );
    for( int altIdx=0; altIdx < param.numAlternativesChroma; ++altIdx )
    {
      WRITE_FLAG( param.nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx], "alf_nonlinear_enable_flag_chroma" );
      alfFilter(param, true, altIdx);
    }
#else
    WRITE_FLAG(param.nonLinearFlag[CHANNEL_TYPE_CHROMA], "alf_chroma_clip");
      alfFilter(param, true);
#endif
  }
}

void HLSWriter::codeLmcsAps( APS* pcAPS )
{
  SliceReshapeInfo param = pcAPS->getReshaperAPSInfo();
  WRITE_UVLC(param.reshaperModelMinBinIdx, "lmcs_min_bin_idx");
  WRITE_UVLC(PIC_CODE_CW_BINS - 1 - param.reshaperModelMaxBinIdx, "lmcs_delta_max_bin_idx");
  assert(param.maxNbitsNeededDeltaCW > 0);
  WRITE_UVLC(param.maxNbitsNeededDeltaCW - 1, "lmcs_delta_cw_prec_minus1");

  for (int i = param.reshaperModelMinBinIdx; i <= param.reshaperModelMaxBinIdx; i++)
  {
    int deltaCW = param.reshaperModelBinCWDelta[i];
    int signCW = (deltaCW < 0) ? 1 : 0;
    int absCW = (deltaCW < 0) ? (-deltaCW) : deltaCW;
    WRITE_CODE(absCW, param.maxNbitsNeededDeltaCW, "lmcs_delta_abs_cw[ i ]");
    if (absCW > 0)
    {
      WRITE_FLAG(signCW, "lmcs_delta_sign_cw_flag[ i ]");
    }
  }
}

#if JVET_O0299_APS_SCALINGLIST
void HLSWriter::codeScalingListAps( APS* pcAPS )
{
  ScalingList param = pcAPS->getScalingList();
  codeScalingList( param );
}
#endif

void HLSWriter::codeVUI( const VUI *pcVUI, const SPS* pcSPS )
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif


  WRITE_FLAG(pcVUI->getAspectRatioInfoPresentFlag(),            "aspect_ratio_info_present_flag");
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    WRITE_CODE(pcVUI->getAspectRatioIdc(), 8,                   "aspect_ratio_idc" );
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      WRITE_CODE(pcVUI->getSarWidth(), 16,                      "sar_width");
      WRITE_CODE(pcVUI->getSarHeight(), 16,                     "sar_height");
    }
  }
  WRITE_FLAG(pcVUI->getColourDescriptionPresentFlag(),        "colour_description_present_flag");
  if (pcVUI->getColourDescriptionPresentFlag())
  {
    WRITE_CODE(pcVUI->getColourPrimaries(), 8,                "colour_primaries");
    WRITE_CODE(pcVUI->getTransferCharacteristics(), 8,        "transfer_characteristics");
    WRITE_CODE(pcVUI->getMatrixCoefficients(), 8,             "matrix_coeffs");
  }
  WRITE_FLAG(pcVUI->getFieldSeqFlag(),                          "field_seq_flag");
  WRITE_FLAG(pcVUI->getChromaLocInfoPresentFlag(),              "chroma_loc_info_present_flag");
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    if(pcVUI->getFieldSeqFlag())
    {
      WRITE_UVLC(pcVUI->getChromaSampleLocTypeTopField(),         "chroma_sample_loc_type_top_field");
      WRITE_UVLC(pcVUI->getChromaSampleLocTypeBottomField(),      "chroma_sample_loc_type_bottom_field");
    }
    else
    {
      WRITE_UVLC(pcVUI->getChromaSampleLocType(),         "chroma_sample_loc_type");
    }
  }
  WRITE_FLAG(pcVUI->getOverscanInfoPresentFlag(),               "overscan_info_present_flag");
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    WRITE_FLAG(pcVUI->getOverscanAppropriateFlag(),             "overscan_appropriate_flag");
  }
  WRITE_FLAG(pcVUI->getVideoSignalTypePresentFlag(),            "video_signal_type_present_flag");
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    WRITE_FLAG(pcVUI->getVideoFullRangeFlag(),                  "video_full_range_flag");
  }

}

#if !JVET_N0353_INDEP_BUFF_TIME_SEI
void HLSWriter::codeHrdParameters( const HRDParameters *hrd, bool commonInfPresentFlag, uint32_t maxNumSubLayersMinus1 )
#else
void HLSWriter::codeHrdParameters( const HRDParameters *hrd, const uint32_t firstSubLayer, const uint32_t maxNumSubLayersMinus1)
#endif
{
#if !JVET_N0353_INDEP_BUFF_TIME_SEI
  if( commonInfPresentFlag )
  {
    WRITE_FLAG( hrd->getNalHrdParametersPresentFlag() ? 1 : 0 ,  "nal_hrd_parameters_present_flag" );
    WRITE_FLAG( hrd->getVclHrdParametersPresentFlag() ? 1 : 0 ,  "vcl_hrd_parameters_present_flag" );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      WRITE_FLAG( hrd->getSubPicCpbParamsPresentFlag() ? 1 : 0,  "sub_pic_hrd_params_present_flag" );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        WRITE_CODE( hrd->getTickDivisorMinus2(), 8,              "tick_divisor_minus2" );
        WRITE_CODE( hrd->getDuCpbRemovalDelayLengthMinus1(), 5,  "du_cpb_removal_delay_increment_length_minus1" );
        WRITE_FLAG( hrd->getSubPicCpbParamsInPicTimingSEIFlag() ? 1 : 0, "sub_pic_cpb_params_in_pic_timing_sei_flag" );
        WRITE_CODE( hrd->getDpbOutputDelayDuLengthMinus1(), 5,   "dpb_output_delay_du_length_minus1"  );
      }
      WRITE_CODE( hrd->getBitRateScale(), 4,                     "bit_rate_scale" );
      WRITE_CODE( hrd->getCpbSizeScale(), 4,                     "cpb_size_scale" );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        WRITE_CODE( hrd->getDuCpbSizeScale(), 4,                "du_cpb_size_scale" );
      }
      WRITE_CODE( hrd->getInitialCpbRemovalDelayLengthMinus1(), 5, "initial_cpb_removal_delay_length_minus1" );
      WRITE_CODE( hrd->getCpbRemovalDelayLengthMinus1(),        5, "au_cpb_removal_delay_length_minus1" );
      WRITE_CODE( hrd->getDpbOutputDelayLengthMinus1(),         5, "dpb_output_delay_length_minus1" );
    }
  }
  int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
  {
    WRITE_FLAG( hrd->getFixedPicRateFlag( i ) ? 1 : 0,          "fixed_pic_rate_general_flag");
    bool fixedPixRateWithinCvsFlag = true;
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      fixedPixRateWithinCvsFlag = hrd->getFixedPicRateWithinCvsFlag( i );
      WRITE_FLAG( hrd->getFixedPicRateWithinCvsFlag( i ) ? 1 : 0, "fixed_pic_rate_within_cvs_flag");
    }
    if( fixedPixRateWithinCvsFlag )
    {
      WRITE_UVLC( hrd->getPicDurationInTcMinus1( i ),           "elemental_duration_in_tc_minus1");
    }
    else
    {
      WRITE_FLAG( hrd->getLowDelayHrdFlag( i ) ? 1 : 0,           "low_delay_hrd_flag");
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      WRITE_UVLC( hrd->getCpbCntMinus1( i ),                      "cpb_cnt_minus1");
    }

    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          WRITE_UVLC( hrd->getBitRateValueMinus1( i, j, nalOrVcl ), "bit_rate_value_minus1");
          WRITE_UVLC( hrd->getCpbSizeValueMinus1( i, j, nalOrVcl ), "cpb_size_value_minus1");
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            WRITE_UVLC( hrd->getDuCpbSizeValueMinus1( i, j, nalOrVcl ), "cpb_size_du_value_minus1");
            WRITE_UVLC( hrd->getDuBitRateValueMinus1( i, j, nalOrVcl ), "bit_rate_du_value_minus1");
          }
          WRITE_FLAG( hrd->getCbrFlag( i, j, nalOrVcl ) ? 1 : 0, "cbr_flag");
        }
      }
    }
  }
#else
  WRITE_FLAG( hrd->getNalHrdParametersPresentFlag() ? 1 : 0 ,  "general_nal_hrd_parameters_present_flag" );
  WRITE_FLAG( hrd->getVclHrdParametersPresentFlag() ? 1 : 0 ,  "general_vcl_hrd_parameters_present_flag" );
  if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
  {
#if JVET_O0189_DU
    WRITE_FLAG( hrd->getDecodingUnitHrdParamsPresentFlag() ? 1 : 0,  "decoding_unit_hrd_params_present_flag" );
#else
    WRITE_FLAG( hrd->getSubPicCpbParamsPresentFlag() ? 1 : 0,  "decoding_unit_hrd_params_present_flag" );
#endif
#if JVET_O0189_DU
    if( hrd->getDecodingUnitHrdParamsPresentFlag() )
    {
      WRITE_CODE( hrd->getTickDivisorMinus2(), 8,            "tick_divisor_minus2" );
      WRITE_FLAG( hrd->getDecodingUnitCpbParamsInPicTimingSeiFlag() ? 1 : 0, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );
    }
#endif
#if FIX_HRD_O0189
    WRITE_CODE( hrd->getBitRateScale(), 4,                     "bit_rate_scale" );
    WRITE_CODE( hrd->getCpbSizeScale(), 4,                     "cpb_size_scale" );
#endif
#if JVET_O0189_DU
    if( hrd->getDecodingUnitHrdParamsPresentFlag() )
    {
      WRITE_CODE( hrd->getCpbSizeDuScale(), 4,               "cpb_size_du_scale" );
  }
#endif
  }
#if !FIX_HRD_O0189
  WRITE_CODE( hrd->getBitRateScale(), 4,                     "bit_rate_scale" );
  WRITE_CODE( hrd->getCpbSizeScale(), 4,                     "cpb_size_scale" );
#endif

  for( int i = firstSubLayer; i <= maxNumSubLayersMinus1; i ++ )
  {
    WRITE_FLAG( hrd->getFixedPicRateFlag( i ) ? 1 : 0,          "fixed_pic_rate_general_flag");
    bool fixedPixRateWithinCvsFlag = true;
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      fixedPixRateWithinCvsFlag = hrd->getFixedPicRateWithinCvsFlag( i );
      WRITE_FLAG( hrd->getFixedPicRateWithinCvsFlag( i ) ? 1 : 0, "fixed_pic_rate_within_cvs_flag");
    }
    if( fixedPixRateWithinCvsFlag )
    {
      WRITE_UVLC( hrd->getPicDurationInTcMinus1( i ),           "elemental_duration_in_tc_minus1");
    }
    else
    {
      WRITE_FLAG( hrd->getLowDelayHrdFlag( i ) ? 1 : 0,           "low_delay_hrd_flag");
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      WRITE_UVLC( hrd->getCpbCntMinus1( i ),                      "cpb_cnt_minus1");
    }

    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( int j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          WRITE_UVLC( hrd->getBitRateValueMinus1( i, j, nalOrVcl ), "bit_rate_value_minus1");
          WRITE_UVLC( hrd->getCpbSizeValueMinus1( i, j, nalOrVcl ), "cpb_size_value_minus1");
          WRITE_FLAG( hrd->getCbrFlag( i, j, nalOrVcl ) ? 1 : 0, "cbr_flag");
        }
      }
    }
  }
#endif
}


void HLSWriter::codeSPS( const SPS* pcSPS )
{
#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif
  WRITE_CODE( pcSPS->getDecodingParameterSetId (), 4,       "sps_decoding_parameter_set_id" );
  CHECK(pcSPS->getMaxTLayers() == 0, "Maximum number of temporal sub-layers is '0'");

  WRITE_CODE(pcSPS->getMaxTLayers() - 1, 3, "sps_max_sub_layers_minus1");
  WRITE_CODE(0,                          5, "sps_reserved_zero_5bits");

  codeProfileTierLevel( pcSPS->getProfileTierLevel(), pcSPS->getMaxTLayers() - 1 );
#if JVET_N0865_SYNTAX
  WRITE_FLAG(pcSPS->getGDREnabledFlag(), "gdr_enabled_flag");
#endif

  WRITE_UVLC(pcSPS->getSPSId (), "sps_seq_parameter_set_id");

  WRITE_UVLC( int(pcSPS->getChromaFormatIdc ()),    "chroma_format_idc" );

  const ChromaFormat format                = pcSPS->getChromaFormatIdc();
  if( format == CHROMA_444 )
  {
    WRITE_FLAG( 0,                                  "separate_colour_plane_flag");
  }

#if JVET_O1164_PS
  WRITE_UVLC( pcSPS->getMaxPicWidthInLumaSamples(), "pic_width_max_in_luma_samples" );
  WRITE_UVLC( pcSPS->getMaxPicHeightInLumaSamples(), "pic_height_max_in_luma_samples" );  
#else
  WRITE_UVLC( pcSPS->getPicWidthInLumaSamples (),   "pic_width_in_luma_samples" );
  WRITE_UVLC( pcSPS->getPicHeightInLumaSamples(),   "pic_height_in_luma_samples" );
  Window conf = pcSPS->getConformanceWindow();

  // KJS: not removing yet
  WRITE_FLAG( conf.getWindowEnabledFlag(),          "conformance_window_flag" );
  if (conf.getWindowEnabledFlag())
  {
    WRITE_UVLC( conf.getWindowLeftOffset()   / SPS::getWinUnitX(pcSPS->getChromaFormatIdc() ), "conf_win_left_offset" );
    WRITE_UVLC( conf.getWindowRightOffset()  / SPS::getWinUnitX(pcSPS->getChromaFormatIdc() ), "conf_win_right_offset" );
    WRITE_UVLC( conf.getWindowTopOffset()    / SPS::getWinUnitY(pcSPS->getChromaFormatIdc() ), "conf_win_top_offset" );
    WRITE_UVLC( conf.getWindowBottomOffset() / SPS::getWinUnitY(pcSPS->getChromaFormatIdc() ), "conf_win_bottom_offset" );
  }
#endif

  WRITE_UVLC( pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) - 8,                      "bit_depth_luma_minus8" );

  const bool         chromaEnabled         = isChromaEnabled(format);
  WRITE_UVLC( chromaEnabled ? (pcSPS->getBitDepth(CHANNEL_TYPE_CHROMA) - 8):0,  "bit_depth_chroma_minus8" );

#if JVET_O0919_TS_MIN_QP
  WRITE_UVLC( pcSPS->getMinQpPrimeTsMinus4(CHANNEL_TYPE_LUMA),                      "min_qp_prime_ts_minus4" );
#endif

  WRITE_UVLC( pcSPS->getBitsForPOC()-4,                 "log2_max_pic_order_cnt_lsb_minus4" );
  WRITE_FLAG( pcSPS->getIDRRefParamListPresent(),                 "sps_idr_rpl_present_flag" );
  // KJS: Marakech decision: sub-layers added back
  const bool subLayerOrderingInfoPresentFlag = 1;
#if JVET_O0178
  if (pcSPS->getMaxTLayers() > 1)
  {
    WRITE_FLAG(subLayerOrderingInfoPresentFlag,       "sps_sub_layer_ordering_info_present_flag");
  }
#else
  WRITE_FLAG(subLayerOrderingInfoPresentFlag,       "sps_sub_layer_ordering_info_present_flag");
#endif
  for(uint32_t i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    WRITE_UVLC( pcSPS->getMaxDecPicBuffering(i) - 1,       "sps_max_dec_pic_buffering_minus1[i]" );
    WRITE_UVLC( pcSPS->getNumReorderPics(i),               "sps_max_num_reorder_pics[i]" );
    WRITE_UVLC( pcSPS->getMaxLatencyIncreasePlus1(i),      "sps_max_latency_increase_plus1[i]" );
    if (!subLayerOrderingInfoPresentFlag)
    {
      break;
    }
  }
  CHECK( pcSPS->getMaxCUWidth() != pcSPS->getMaxCUHeight(),                          "Rectangular CTUs not supported" );
  WRITE_FLAG(pcSPS->getLongTermRefsPresent() ? 1 : 0, "long_term_ref_pics_flag");
  WRITE_FLAG(pcSPS->getRPL1CopyFromRPL0Flag() ? 1 : 0, "rpl1_copy_from_rpl0_flag");

  const RPLList* rplList0 = pcSPS->getRPLList0();
  const RPLList* rplList1 = pcSPS->getRPLList1();

  //Write candidate for List0
  uint32_t numberOfRPL = pcSPS->getNumRPL0();
  WRITE_UVLC(numberOfRPL, "num_ref_pic_lists_in_sps[0]");
  for (int ii = 0; ii < numberOfRPL; ii++)
  {
    const ReferencePictureList* rpl = rplList0->getReferencePictureList(ii);
#if JVET_O0244_DELTA_POC
    xCodeRefPicList( rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC(), !pcSPS->getUseWP() && !pcSPS->getUseWPBiPred() );
#else
    xCodeRefPicList(rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC());
#endif
  }

  //Write candidate for List1
  if (!pcSPS->getRPL1CopyFromRPL0Flag())
  {
    numberOfRPL = pcSPS->getNumRPL1();
    WRITE_UVLC(numberOfRPL, "num_ref_pic_lists_in_sps[1]");
    for (int ii = 0; ii < numberOfRPL; ii++)
    {
      const ReferencePictureList* rpl = rplList1->getReferencePictureList(ii);
#if JVET_O0244_DELTA_POC
      xCodeRefPicList( rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC(), !pcSPS->getUseWP() && !pcSPS->getUseWPBiPred() );
#else
      xCodeRefPicList(rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC());
#endif
    }
  }
  WRITE_FLAG(pcSPS->getUseDualITree(), "qtbtt_dual_tree_intra_flag");
#if JVET_O0526_MIN_CTU_SIZE
  WRITE_CODE(floorLog2(pcSPS->getCTUSize()) - 5, 2, "log2_ctu_size_minus5");
#else
  WRITE_UVLC(floorLog2(pcSPS->getCTUSize()) - MIN_CU_LOG2, "log2_ctu_size_minus2");
#endif
  WRITE_UVLC(pcSPS->getLog2MinCodingBlockSize() - 2, "log2_min_luma_coding_block_size_minus2");
  WRITE_FLAG(pcSPS->getSplitConsOverrideEnabledFlag(), "partition_constraints_override_enabled_flag");
  WRITE_UVLC(floorLog2(pcSPS->getMinQTSize(I_SLICE)) - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
  WRITE_UVLC(floorLog2(pcSPS->getMinQTSize(B_SLICE)) - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_inter_slice");
  WRITE_UVLC(pcSPS->getMaxMTTHierarchyDepth(), "sps_max_mtt_hierarchy_depth_inter_slice");
  WRITE_UVLC(pcSPS->getMaxMTTHierarchyDepthI(), "sps_max_mtt_hierarchy_depth_intra_slice_luma");
  if (pcSPS->getMaxMTTHierarchyDepthI() != 0)
  {
    WRITE_UVLC(floorLog2(pcSPS->getMaxBTSizeI()) - floorLog2(pcSPS->getMinQTSize(I_SLICE)), "sps_log2_diff_max_bt_min_qt_intra_slice_luma");
    WRITE_UVLC(floorLog2(pcSPS->getMaxTTSizeI()) - floorLog2(pcSPS->getMinQTSize(I_SLICE)), "sps_log2_diff_max_tt_min_qt_intra_slice_luma");
  }
  if (pcSPS->getMaxMTTHierarchyDepth() != 0)
  {
    WRITE_UVLC(floorLog2(pcSPS->getMaxBTSize()) - floorLog2(pcSPS->getMinQTSize(B_SLICE)), "sps_log2_diff_max_bt_min_qt_inter_slice");
    WRITE_UVLC(floorLog2(pcSPS->getMaxTTSize()) - floorLog2(pcSPS->getMinQTSize(B_SLICE)), "sps_log2_diff_max_tt_min_qt_inter_slice");
  }
  if (pcSPS->getUseDualITree())
  {
    WRITE_UVLC(floorLog2(pcSPS->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)) - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_intra_slice_chroma");
    WRITE_UVLC(pcSPS->getMaxMTTHierarchyDepthIChroma(), "sps_max_mtt_hierarchy_depth_intra_slice_chroma");
    if (pcSPS->getMaxMTTHierarchyDepthIChroma() != 0)
    {
      WRITE_UVLC(floorLog2(pcSPS->getMaxBTSizeIChroma()) - floorLog2(pcSPS->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)), "sps_log2_diff_max_bt_min_qt_intra_slice_chroma");
      WRITE_UVLC(floorLog2(pcSPS->getMaxTTSizeIChroma()) - floorLog2(pcSPS->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)), "sps_log2_diff_max_tt_min_qt_intra_slice_chroma");
    }
  }

#if MAX_TB_SIZE_SIGNALLING
#if JVET_O0545_MAX_TB_SIGNALLING
  WRITE_FLAG( (pcSPS->getLog2MaxTbSize() - 5) ? 1 : 0,                       "sps_max_luma_transform_size_64_flag" );
#else
  // KJS: Not in syntax
  WRITE_UVLC( pcSPS->getLog2MaxTbSize() - 2,                                 "log2_max_luma_transform_block_size_minus2" );
#endif
#endif

#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
  if (pcSPS->getChromaFormatIdc() != CHROMA_400)
  {
    const ChromaQpMappingTable& chromaQpMappingTable = pcSPS->getChromaQpMappingTable();
    WRITE_FLAG(chromaQpMappingTable.getSameCQPTableForAllChromaFlag(), "same_qp_table_for_chroma");
    for (int i = 0; i < (chromaQpMappingTable.getSameCQPTableForAllChromaFlag() ? 1 : 3); i++)
    {
      WRITE_UVLC(chromaQpMappingTable.getNumPtsInCQPTableMinus1(i), "num_points_in_qp_table_minus1");

      for (int j = 0; j <= chromaQpMappingTable.getNumPtsInCQPTableMinus1(i); j++)
      {
        WRITE_UVLC(chromaQpMappingTable.getDeltaQpInValMinus1(i,j),  "delta_qp_in_val_minus1");
        WRITE_UVLC(chromaQpMappingTable.getDeltaQpOutVal(i, j),       "delta_qp_out_val");
      }
    }
  }
#endif

#if JVET_O0244_DELTA_POC
  WRITE_FLAG( pcSPS->getUseWP() ? 1 : 0, "sps_weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcSPS->getUseWPBiPred() ? 1 : 0, "sps_weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)
#endif

  WRITE_FLAG( pcSPS->getSAOEnabledFlag(),                                            "sps_sao_enabled_flag");
  WRITE_FLAG( pcSPS->getALFEnabledFlag(),                                            "sps_alf_enabled_flag" );

#if !JVET_O0525_REMOVE_PCM
  WRITE_FLAG( pcSPS->getPCMEnabledFlag() ? 1 : 0,                                    "sps_pcm_enabled_flag");
  if( pcSPS->getPCMEnabledFlag() )
  {
    WRITE_CODE( pcSPS->getPCMBitDepth(CHANNEL_TYPE_LUMA) - 1, 4,                            "pcm_sample_bit_depth_luma_minus1" );
    WRITE_CODE( chromaEnabled ? (pcSPS->getPCMBitDepth(CHANNEL_TYPE_CHROMA) - 1) : 0, 4,    "pcm_sample_bit_depth_chroma_minus1" );
    WRITE_UVLC( pcSPS->getPCMLog2MinSize() - 3,                                      "log2_min_pcm_luma_coding_block_size_minus3" );
    WRITE_UVLC( pcSPS->getPCMLog2MaxSize() - pcSPS->getPCMLog2MinSize(),             "log2_diff_max_min_pcm_luma_coding_block_size" );
    WRITE_FLAG( pcSPS->getPCMFilterDisableFlag()?1 : 0,                              "pcm_loop_filter_disable_flag");
  }
#endif

#if JVET_O1136_TS_BDPCM_SIGNALLING
  WRITE_FLAG(pcSPS->getTransformSkipEnabledFlag() ? 1 : 0, "sps_transform_skip_enabled_flag");
  if (pcSPS->getTransformSkipEnabledFlag())
  {
    WRITE_FLAG(pcSPS->getBDPCMEnabledFlag() ? 1 : 0, "sps_bdpcm_enabled_flag");
  }
  else
  {
    CHECK(pcSPS->getBDPCMEnabledFlag(), "BDPCM cannot be used when transform skip is disabled");
  }
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  WRITE_FLAG( pcSPS->getJointCbCrEnabledFlag(),                                           "sps_joint_cbcr_enabled_flag");
#endif

#if !JVET_O1164_PS
  if( pcSPS->getCTUSize() + 2*(1 << pcSPS->getLog2MinCodingBlockSize()) <= pcSPS->getPicWidthInLumaSamples() )
  {
#endif
  WRITE_FLAG( pcSPS->getWrapAroundEnabledFlag() ? 1 : 0,                              "sps_ref_wraparound_enabled_flag" );
  if( pcSPS->getWrapAroundEnabledFlag() )
  {
    WRITE_UVLC( (pcSPS->getWrapAroundOffset()/(1 <<  pcSPS->getLog2MinCodingBlockSize()))-1,  "sps_ref_wraparound_offset_minus1" );
  }
#if !JVET_O1164_PS
  }
#endif

  WRITE_FLAG( pcSPS->getSPSTemporalMVPEnabledFlag()  ? 1 : 0,                        "sps_temporal_mvp_enabled_flag" );

  if ( pcSPS->getSPSTemporalMVPEnabledFlag() )
  {
    WRITE_FLAG( pcSPS->getSBTMVPEnabledFlag() ? 1 : 0,                               "sps_sbtmvp_enabled_flag");
  }

  WRITE_FLAG( pcSPS->getAMVREnabledFlag() ? 1 : 0,                                   "sps_amvr_enabled_flag" );

  WRITE_FLAG( pcSPS->getBDOFEnabledFlag() ? 1 : 0,                                   "sps_bdof_enabled_flag" );
#if !JVET_O0438_SPS_AFFINE_AMVR_FLAG
  WRITE_FLAG( pcSPS->getAffineAmvrEnabledFlag() ? 1 : 0,                             "sps_affine_amvr_enabled_flag" );
#endif
  WRITE_FLAG( pcSPS->getUseDMVR() ? 1 : 0,                                            "sps_dmvr_enable_flag" );
  WRITE_FLAG(pcSPS->getUseMMVD() ? 1 : 0,                                             "sps_mmvd_enable_flag");
  // KJS: sps_cclm_enabled_flag
  WRITE_FLAG( pcSPS->getUseLMChroma() ? 1 : 0,                                                 "lm_chroma_enabled_flag" );
  if ( pcSPS->getUseLMChroma() && pcSPS->getChromaFormatIdc() == CHROMA_420 )
  {
    WRITE_FLAG( pcSPS->getCclmCollocatedChromaFlag() ? 1 : 0,                                  "sps_cclm_collocated_chroma_flag" );
  }

  WRITE_FLAG( pcSPS->getUseMTS() ? 1 : 0,                                                      "mts_enabled_flag" );
  if ( pcSPS->getUseMTS() )
  {
    WRITE_FLAG( pcSPS->getUseIntraMTS() ? 1 : 0,                                               "mts_intra_enabled_flag" );
    WRITE_FLAG( pcSPS->getUseInterMTS() ? 1 : 0,                                               "mts_inter_enabled_flag" );
  }
  WRITE_FLAG( pcSPS->getUseLFNST() ? 1 : 0,                                                    "lfnst_enabled_flag" );
  WRITE_FLAG( pcSPS->getUseSMVD() ? 1 : 0,                                                     "smvd_flag" );
  // KJS: sps_affine_enabled_flag
  WRITE_FLAG( pcSPS->getUseAffine() ? 1 : 0,                                                   "affine_flag" );
  if ( pcSPS->getUseAffine() )
  {
    WRITE_FLAG( pcSPS->getUseAffineType() ? 1 : 0,                                             "affine_type_flag" );
#if JVET_O0070_PROF
    WRITE_FLAG( pcSPS->getUsePROF() ? 1 : 0,                                                   "sps_prof_enabled_flag" );
#endif
#if JVET_O0438_SPS_AFFINE_AMVR_FLAG
    WRITE_FLAG( pcSPS->getAffineAmvrEnabledFlag() ? 1 : 0,                                     "sps_affine_amvr_enabled_flag" );
#endif
  }
  WRITE_FLAG( pcSPS->getUseGBi() ? 1 : 0,                                                      "gbi_flag" );
#if JVET_O0119_BASE_PALETTE_444
  if (pcSPS->getChromaFormatIdc() == CHROMA_444)
  {
    WRITE_FLAG(pcSPS->getPLTMode() ? 1 : 0,                                                    "plt_flag" );
  }
#endif
  WRITE_FLAG(pcSPS->getIBCFlag() ? 1 : 0,                                                      "ibc_flag");

  // KJS: sps_ciip_enabled_flag
  WRITE_FLAG( pcSPS->getUseMHIntra() ? 1 : 0,                                                  "mhintra_flag" );

  if ( pcSPS->getUseMMVD() )
  {
    WRITE_FLAG( pcSPS->getFpelMmvdEnabledFlag() ? 1 : 0,                            "sps_fpel_mmvd_enabled_flag" );
  }
#if JVET_O1140_SLICE_DISABLE_BDOF_DMVR_FLAG
  if(pcSPS->getBDOFEnabledFlag() || pcSPS->getUseDMVR())
  {
    WRITE_FLAG(pcSPS->getBdofDmvrSlicePresentFlag() ? 1 : 0,                            "sps_bdof_dmvr_slice_level_present_flag");
  }
#endif
  WRITE_FLAG( pcSPS->getUseTriangle() ? 1: 0,                                                  "triangle_flag" );

  WRITE_FLAG( pcSPS->getUseMIP() ? 1: 0,                                                       "sps_mip_flag" );
  // KJS: not in draft yet
  WRITE_FLAG( pcSPS->getUseSBT() ? 1 : 0,                                             "sbt_enable_flag");
  if( pcSPS->getUseSBT() )
  {
    WRITE_FLAG(pcSPS->getMaxSbtSize() == 64 ? 1 : 0,                                  "max_sbt_size_64_flag");
  }
  // KJS: not in draft yet
  WRITE_FLAG(pcSPS->getUseReshaper() ? 1 : 0, "sps_reshaper_enable_flag");
  WRITE_FLAG( pcSPS->getUseISP() ? 1 : 0,                                             "isp_enable_flag");

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  WRITE_FLAG( pcSPS->getLadfEnabled() ? 1 : 0,                                                 "sps_ladf_enabled_flag" );
  if ( pcSPS->getLadfEnabled() )
  {
    WRITE_CODE( pcSPS->getLadfNumIntervals() - 2, 2,                                           "sps_num_ladf_intervals_minus2" );
    WRITE_SVLC( pcSPS->getLadfQpOffset( 0 ),                                                   "sps_ladf_lowest_interval_qp_offset");
    for ( int k = 1; k< pcSPS->getLadfNumIntervals(); k++ )
    {
      WRITE_SVLC( pcSPS->getLadfQpOffset( k ),                                                 "sps_ladf_qp_offset" );
      WRITE_UVLC( pcSPS->getLadfIntervalLowerBound( k ) - pcSPS->getLadfIntervalLowerBound( k - 1 ) - 1, "sps_ladf_delta_threshold_minus1" );
    }
  }
#endif

  // KJS: reference picture sets to be replaced


  // KJS: remove scaling lists?
  WRITE_FLAG( pcSPS->getScalingListFlag() ? 1 : 0,                                   "scaling_list_enabled_flag" );
#if !JVET_O0299_APS_SCALINGLIST
  if(pcSPS->getScalingListFlag())
  {
    WRITE_FLAG( pcSPS->getScalingListPresentFlag() ? 1 : 0,                          "sps_scaling_list_data_present_flag" );
    if(pcSPS->getScalingListPresentFlag())
    {
      codeScalingList( pcSPS->getScalingList() );
    }
  }
#endif

  const TimingInfo *timingInfo = pcSPS->getTimingInfo();
#if JVET_O0189_DU
  WRITE_FLAG(pcSPS->getHrdParametersPresentFlag(),          "general_hrd_parameters_present_flag");
    if( pcSPS->getHrdParametersPresentFlag() )
#else
  WRITE_FLAG(timingInfo->getTimingInfoPresentFlag(),          "timing_info_present_flag");
  if(timingInfo->getTimingInfoPresentFlag())
#endif
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "time_scale");
#if !JVET_O0189_DU
    WRITE_FLAG(pcSPS->getHrdParametersPresentFlag(),              "hrd_parameters_present_flag");
#endif
#if JVET_O0177_PROPOSAL1
    WRITE_FLAG(pcSPS->getSubLayerParametersPresentFlag(), "sub_layer_cpb_parameters_present_flag");
    if (pcSPS->getSubLayerParametersPresentFlag())
    {
      codeHrdParameters(pcSPS->getHrdParameters(), 0, pcSPS->getMaxTLayers() - 1);
    }
    else
    {
      codeHrdParameters(pcSPS->getHrdParameters(), pcSPS->getMaxTLayers() - 1, pcSPS->getMaxTLayers() - 1);
    }
#else
    if( pcSPS->getHrdParametersPresentFlag() )
    {
      codeHrdParameters(pcSPS->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
#endif
  }

  WRITE_FLAG( pcSPS->getVuiParametersPresentFlag(),            "vui_parameters_present_flag" );
  if (pcSPS->getVuiParametersPresentFlag())
  {
    codeVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  bool sps_extension_present_flag=false;
  bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS]={false};

  sps_extension_flags[SPS_EXT__REXT] = pcSPS->getSpsRangeExtension().settingsDifferFromDefaults();

  // Other SPS extension flags checked here.

  for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
  {
    sps_extension_present_flag|=sps_extension_flags[i];
  }

  WRITE_FLAG( (sps_extension_present_flag?1:0), "sps_extension_present_flag" );

  if (sps_extension_present_flag)
  {
#if ENABLE_TRACING /*|| RExt__DECODER_DEBUG_BIT_STATISTICS*/
    static const char *syntaxStrings[]={ "sps_range_extension_flag",
      "sps_multilayer_extension_flag",
      "sps_extension_6bits[0]",
      "sps_extension_6bits[1]",
      "sps_extension_6bits[2]",
      "sps_extension_6bits[3]",
      "sps_extension_6bits[4]",
      "sps_extension_6bits[5]" };
#endif

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      WRITE_FLAG( sps_extension_flags[i]?1:0, syntaxStrings[i] );
    }

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
        {
          const SPSRExt &spsRangeExtension=pcSPS->getSpsRangeExtension();

          WRITE_FLAG( (spsRangeExtension.getTransformSkipRotationEnabledFlag() ? 1 : 0),      "transform_skip_rotation_enabled_flag");
          WRITE_FLAG( (spsRangeExtension.getTransformSkipContextEnabledFlag() ? 1 : 0),       "transform_skip_context_enabled_flag");
          WRITE_FLAG( (spsRangeExtension.getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT) ? 1 : 0), "implicit_rdpcm_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT) ? 1 : 0), "explicit_rdpcm_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getExtendedPrecisionProcessingFlag() ? 1 : 0),       "extended_precision_processing_flag" );
          WRITE_FLAG( (spsRangeExtension.getIntraSmoothingDisabledFlag() ? 1 : 0),            "intra_smoothing_disabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getHighPrecisionOffsetsEnabledFlag() ? 1 : 0),       "high_precision_offsets_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getPersistentRiceAdaptationEnabledFlag() ? 1 : 0),   "persistent_rice_adaptation_enabled_flag" );
          WRITE_FLAG( (spsRangeExtension.getCabacBypassAlignmentEnabledFlag() ? 1 : 0),       "cabac_bypass_alignment_enabled_flag" );
          break;
        }
        default:
          CHECK(sps_extension_flags[i]!=false, "Unknown PPS extension signalled"); // Should never get here with an active SPS extension flag.
          break;
        }
      }
    }
  }
  xWriteRbspTrailingBits();
}

void HLSWriter::codeDPS( const DPS* dps )
{
#if ENABLE_TRACING
  xTraceDPSHeader();
#endif
  WRITE_CODE( dps->getDecodingParameterSetId(),     4,        "dps_decoding_parameter_set_id" );
  WRITE_CODE( dps->getMaxSubLayersMinus1(),         3,        "dps_max_sub_layers_minus1" );
  WRITE_FLAG( 0,                                              "dps_reserved_zero_bit" );

  ProfileTierLevel ptl = dps->getProfileTierLevel();
  codeProfileTierLevel( &ptl, dps->getMaxSubLayersMinus1() );

  WRITE_FLAG( 0,                                              "dps_extension_flag" );
  xWriteRbspTrailingBits();
}

void HLSWriter::codeVPS(const VPS* pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  WRITE_CODE(pcVPS->getVPSId(), 4, "vps_video_parameter_set_id");
  WRITE_CODE(pcVPS->getMaxLayers() - 1, 8, "vps_max_layers_minus1");
  for (uint32_t i = 0; i <= pcVPS->getMaxLayers() - 1; i++)
  {
    WRITE_CODE(pcVPS->getVPSIncludedLayerId(i), 7, "vps_included_layer_id");
    WRITE_FLAG(0, "vps_reserved_zero_1bit");
  }

  WRITE_FLAG(0, "vps_extension_flag");

  //future extensions here..
  xWriteRbspTrailingBits();
}

void HLSWriter::codeSliceHeader         ( Slice* pcSlice )
{
#if ENABLE_TRACING
  xTraceSliceHeader ();
#endif

  CodingStructure& cs = *pcSlice->getPic()->cs;
  const ChromaFormat format                = pcSlice->getSPS()->getChromaFormatIdc();
  const uint32_t         numberValidComponents = getNumberValidComponents(format);
  const bool         chromaEnabled         = isChromaEnabled(format);

#if !JVET_N0865_SYNTAX
  if (pcSlice->getRapPicFlag())
  {
    WRITE_FLAG(pcSlice->getNoOutputPriorPicsFlag() ? 1 : 0, "no_output_of_prior_pics_flag");
  }
#endif
  WRITE_UVLC( pcSlice->getPPS()->getPPSId(), "slice_pic_parameter_set_id" );
  int bitsSliceAddress = 1;
  if (!pcSlice->getPPS()->getRectSliceFlag())
  {
    while (pcSlice->getPPS()->getNumBricksInPic() > (1 << bitsSliceAddress))
    {
      bitsSliceAddress++;
    }
  }
  else
  {
    if (pcSlice->getPPS()->getSignalledSliceIdFlag())
    {
      bitsSliceAddress = pcSlice->getPPS()->getSignalledSliceIdLengthMinus1() + 1;
    }
    else
    {
      while ((pcSlice->getPPS()->getNumSlicesInPicMinus1() + 1) > (1 << bitsSliceAddress))
      {
        bitsSliceAddress++;
      }
    }
  }
  if (pcSlice->getPPS()->getRectSliceFlag() || pcSlice->getPPS()->getNumBricksInPic() > 1)
  {
    if (pcSlice->getPPS()->getRectSliceFlag())
    {
      WRITE_CODE(pcSlice->getPPS()->getSliceId(pcSlice->getSliceIndex()), bitsSliceAddress, "slice_address");
    }
    else
    {
      WRITE_CODE(pcSlice->getSliceCurStartBrickIdx(), bitsSliceAddress, "slice_address");
    }
  }
  if (!pcSlice->getPPS()->getRectSliceFlag() && !pcSlice->getPPS()->getSingleBrickPerSliceFlag())
  {
    WRITE_UVLC(pcSlice->getSliceNumBricks() - 1, "num_bricks_in_slice_minus1");
  }

#if JVET_O0181
    WRITE_FLAG(pcSlice->getNonRefPictFlag() ? 1 : 0, "non_reference_picture_flag");
#endif

    for( int i = 0; i < pcSlice->getPPS()->getNumExtraSliceHeaderBits(); i++ )
    {
      WRITE_FLAG( 0, "slice_reserved_flag[]" );
    }

    WRITE_UVLC( pcSlice->getSliceType(), "slice_type" );

#if !JVET_N0865_SYNTAX
    if (pcSlice->getPPS()->getOutputFlagPresentFlag())
    {
      WRITE_FLAG(pcSlice->getPicOutputFlag() ? 1 : 0, "pic_output_flag");
    }
#endif

    int pocBits = pcSlice->getSPS()->getBitsForPOC();
    int pocMask = (1 << pocBits) - 1;
    WRITE_CODE(pcSlice->getPOC() & pocMask, pocBits, "slice_pic_order_cnt_lsb");
#if JVET_N0865_SYNTAX
#if JVET_N0865_GRA2GDR
    if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR)
#else
    if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GRA)
#endif
    {
      int maxPicOrderCntLsb = (int) pow(2, pcSlice->getSPS()->getBitsForPOC());
      CHECK((pcSlice->getRecoveryPocCnt() < maxPicOrderCntLsb), "The value of recovery_poc_cnt exceeds (POC LSB cycle - 1)");
      WRITE_UVLC(pcSlice->getRecoveryPocCnt(), "recovery_poc_cnt");
    }
#if JVET_N0865_GRA2GDR
    if (pcSlice->getRapPicFlag() || (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR))
#else
    if (pcSlice->getRapPicFlag() || (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GRA))
#endif
    {
      WRITE_FLAG(pcSlice->getNoOutputPriorPicsFlag() ? 1 : 0, "no_output_of_prior_pics_flag");
    }
    if (pcSlice->getPPS()->getOutputFlagPresentFlag())
    {
      WRITE_FLAG(pcSlice->getPicOutputFlag() ? 1 : 0, "pic_output_flag");
    }
#endif
    if( !pcSlice->getIdrPicFlag() || pcSlice->getSPS()->getIDRRefParamListPresent())
    {
      //Write L0 related syntax elements
      if (pcSlice->getSPS()->getNumRPL0() > 0)
      {
#if JVET_O0238_PPS_OR_SLICE
        if (!pcSlice->getPPS()->getPPSRefPicListSPSIdc0())
        {
          WRITE_FLAG(pcSlice->getRPL0idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[0]");
        } 
#else
        WRITE_FLAG(pcSlice->getRPL0idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[0]");
#endif
      }
      if (pcSlice->getRPL0idx() != -1)
      {
        if (pcSlice->getSPS()->getNumRPL0() > 1)
        {
          int numBits = 0;
          while ((1 << numBits) < pcSlice->getSPS()->getNumRPL0())
          {
            numBits++;
          }
          WRITE_CODE(pcSlice->getRPL0idx(), numBits, "ref_pic_list_idx[0]");
        }
      }
      else
      {  //write local RPL0
#if JVET_O0244_DELTA_POC
        xCodeRefPicList( pcSlice->getRPL0(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC(), !pcSlice->getSPS()->getUseWP() && !pcSlice->getSPS()->getUseWPBiPred() );
#else
        xCodeRefPicList(pcSlice->getRPL0(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC());
#endif
      }
      //Deal POC Msb cycle signalling for LTRP
      if (pcSlice->getRPL0()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL0()->getNumberOfLongtermPictures() + pcSlice->getRPL0()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL0()->isRefPicLongterm(i))
          {
#if JVET_N0100_PROPOSAL1
            if (pcSlice->getRPL0()->getLtrpInSliceHeaderFlag())
            {
              WRITE_CODE(pcSlice->getRPL0()->getRefPicIdentifier(i), pcSlice->getSPS()->getBitsForPOC(),
                         "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
            }
#endif
            WRITE_FLAG(pcSlice->getLocalRPL0()->getDeltaPocMSBPresentFlag(i) ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
            if (pcSlice->getLocalRPL0()->getDeltaPocMSBPresentFlag(i))
            {
              WRITE_UVLC(pcSlice->getLocalRPL0()->getDeltaPocMSBCycleLT(i), "delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }

      //Write L1 related syntax elements
      if (!pcSlice->getPPS()->getRpl1IdxPresentFlag())
      {
        CHECK(pcSlice->getRPL1idx() != pcSlice->getRPL0idx(), "RPL1Idx is not signalled but it is not the same as RPL0Idx");
        if (pcSlice->getRPL1idx() == -1)
        {  //write local RPL1
#if JVET_O0244_DELTA_POC
          xCodeRefPicList( pcSlice->getRPL1(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC(), !pcSlice->getSPS()->getUseWP() && !pcSlice->getSPS()->getUseWPBiPred() );
#else
          xCodeRefPicList(pcSlice->getRPL1(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC());
#endif
        }
      }
      else
      {
        if (pcSlice->getSPS()->getNumRPL1() > 0)
        {
#if JVET_O0238_PPS_OR_SLICE
        if (!pcSlice->getPPS()->getPPSRefPicListSPSIdc1())
        {
          WRITE_FLAG(pcSlice->getRPL1idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[1]");
        } 
#else
          WRITE_FLAG(pcSlice->getRPL1idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[1]");
#endif
        }
        if (pcSlice->getRPL1idx() != -1)
        {
          if (pcSlice->getSPS()->getNumRPL1() > 1)
          {
            int numBits = 0;
            while ((1 << numBits) < pcSlice->getSPS()->getNumRPL1())
            {
              numBits++;
            }
            WRITE_CODE(pcSlice->getRPL1idx(), numBits, "ref_pic_list_idx[1]");
          }
        }
        else
        {  //write local RPL1
#if JVET_O0244_DELTA_POC
          xCodeRefPicList( pcSlice->getRPL1(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC(), !pcSlice->getSPS()->getUseWP() && !pcSlice->getSPS()->getUseWPBiPred() );
#else
          xCodeRefPicList(pcSlice->getRPL1(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC());
#endif
        }
      }
      //Deal POC Msb cycle signalling for LTRP
      if (pcSlice->getRPL1()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL1()->getNumberOfLongtermPictures() + pcSlice->getRPL1()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL1()->isRefPicLongterm(i))
          {
#if JVET_N0100_PROPOSAL1
            if (pcSlice->getRPL1()->getLtrpInSliceHeaderFlag())
            {
              WRITE_CODE(pcSlice->getRPL1()->getRefPicIdentifier(i), pcSlice->getSPS()->getBitsForPOC(),
                         "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
            }
#endif
            WRITE_FLAG(pcSlice->getLocalRPL1()->getDeltaPocMSBPresentFlag(i) ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
            if (pcSlice->getLocalRPL1()->getDeltaPocMSBPresentFlag(i))
            {
              WRITE_UVLC(pcSlice->getLocalRPL1()->getDeltaPocMSBCycleLT(i), "delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }

      //check if numrefidxes match the defaults. If not, override

      if ((!pcSlice->isIntra() && pcSlice->getRPL0()->getNumRefEntries() > 1) ||
          (pcSlice->isInterB() && pcSlice->getRPL1()->getNumRefEntries() > 1) )
      {
        int defaultL0 = std::min<int>(pcSlice->getRPL0()->getNumRefEntries(), pcSlice->getPPS()->getNumRefIdxL0DefaultActive());
        int defaultL1 = pcSlice->isInterB() ? std::min<int>(pcSlice->getRPL1()->getNumRefEntries(), pcSlice->getPPS()->getNumRefIdxL1DefaultActive()) : 0;
        bool overrideFlag = ( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) != defaultL0 || ( pcSlice->isInterB() && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) != defaultL1 ) );
        WRITE_FLAG( overrideFlag ? 1 : 0, "num_ref_idx_active_override_flag" );
        if( overrideFlag )
        {
          if(pcSlice->getRPL0()->getNumRefEntries() > 1)
          {
            WRITE_UVLC( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) - 1, "num_ref_idx_l0_active_minus1" );
          }
          else
          {
            pcSlice->setNumRefIdx( REF_PIC_LIST_0, 1);
          }

          if( pcSlice->isInterB() && pcSlice->getRPL1()->getNumRefEntries() > 1)
          {
            WRITE_UVLC( pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) - 1, "num_ref_idx_l1_active_minus1" );
          }
          else
          {
            pcSlice->setNumRefIdx( REF_PIC_LIST_1, pcSlice->isInterB() ? 1 : 0);
          }
        }
        else
        {
          pcSlice->setNumRefIdx( REF_PIC_LIST_0, defaultL0 );
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, defaultL1 );
        }
      }
      else
      {
        pcSlice->setNumRefIdx( REF_PIC_LIST_0, pcSlice->isIntra() ? 0 : 1 );
        pcSlice->setNumRefIdx( REF_PIC_LIST_1, pcSlice->isInterB() ? 1 : 0 );
      }
    }

    if (
      pcSlice->getSPS()->getSplitConsOverrideEnabledFlag()
      )
    {
      WRITE_FLAG(pcSlice->getSplitConsOverrideFlag() ? 1 : 0, "partition_constraints_override_flag");
      if (pcSlice->getSplitConsOverrideFlag())
      {
        WRITE_UVLC(floorLog2(pcSlice->getMinQTSize()) - pcSlice->getSPS()->getLog2MinCodingBlockSize(), "slice_log2_diff_min_qt_min_cb");
        WRITE_UVLC(pcSlice->getMaxMTTHierarchyDepth(), "slice_max_mtt_hierarchy_depth_luma");
        if (pcSlice->getMaxMTTHierarchyDepth() != 0)
        {
          CHECK(pcSlice->getMaxBTSize() < pcSlice->getMinQTSize(), "maxBtSize is smaller than minQtSize");
          WRITE_UVLC(floorLog2(pcSlice->getMaxBTSize()) - floorLog2(pcSlice->getMinQTSize()), "slice_log2_diff_max_bt_min_qt");
          CHECK(pcSlice->getMaxTTSize() < pcSlice->getMinQTSize(), "maxTtSize is smaller than minQtSize");
          WRITE_UVLC(floorLog2(pcSlice->getMaxTTSize()) - floorLog2(pcSlice->getMinQTSize()), "slice_log2_diff_max_tt_min_qt");
        }
        if (
          pcSlice->isIntra() && pcSlice->getSPS()->getUseDualITree()
          )
        {
          WRITE_UVLC(floorLog2(pcSlice->getMinQTSizeIChroma()) - pcSlice->getSPS()->getLog2MinCodingBlockSize(), "slice_log2_diff_min_qt_min_cb_chroma");
          WRITE_UVLC(pcSlice->getMaxMTTHierarchyDepthIChroma(), "slice_max_mtt_hierarchy_depth_chroma");
          if (pcSlice->getMaxMTTHierarchyDepthIChroma() != 0)
          {
            CHECK(pcSlice->getMaxBTSizeIChroma() < pcSlice->getMinQTSizeIChroma(), "maxBtSizeC is smaller than minQtSizeC");
            WRITE_UVLC(floorLog2(pcSlice->getMaxBTSizeIChroma()) - floorLog2(pcSlice->getMinQTSizeIChroma()), "slice_log2_diff_max_bt_min_qt_chroma");
            CHECK(pcSlice->getMaxTTSizeIChroma() < pcSlice->getMinQTSizeIChroma(), "maxTtSizeC is smaller than minQtSizeC");
            WRITE_UVLC(floorLog2(pcSlice->getMaxTTSizeIChroma()) - floorLog2(pcSlice->getMinQTSizeIChroma()), "slice_log2_diff_max_tt_min_qt_chroma");
          }
        }
      }
    }

    if(!pcSlice->isIntra())
    {
#if JVET_O0238_PPS_OR_SLICE
      if( pcSlice->getSPS()->getSPSTemporalMVPEnabledFlag() && !pcSlice->getPPS()->getPPSTemporalMVPEnabledIdc() )
      {
        WRITE_FLAG( pcSlice->getEnableTMVPFlag() ? 1 : 0, "slice_temporal_mvp_enabled_flag" );
      }
#else
      if( pcSlice->getSPS()->getSPSTemporalMVPEnabledFlag() )
      {
        WRITE_FLAG( pcSlice->getEnableTMVPFlag() ? 1 : 0, "slice_temporal_mvp_enabled_flag" );
      }
#endif
    }

    if( pcSlice->isInterB() )
    {
#if JVET_O0238_PPS_OR_SLICE
      if (!pcSlice->getPPS()->getPPSMvdL1ZeroIdc())
      {
        WRITE_FLAG( pcSlice->getMvdL1ZeroFlag() ? 1 : 0, "mvd_l1_zero_flag" );
      }
#else
      WRITE_FLAG( pcSlice->getMvdL1ZeroFlag() ? 1 : 0, "mvd_l1_zero_flag" );
#endif
    }

    if( !pcSlice->isIntra() )
    {
      if( !pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() )
      {
        SliceType sliceType = pcSlice->getSliceType();
        SliceType  encCABACTableIdx = pcSlice->getEncCABACTableIdx();
        bool encCabacInitFlag = ( sliceType != encCABACTableIdx && encCABACTableIdx != I_SLICE ) ? true : false;
        pcSlice->setCabacInitFlag( encCabacInitFlag );
        WRITE_FLAG( encCabacInitFlag ? 1 : 0, "cabac_init_flag" );
      }
    }

    if( pcSlice->getEnableTMVPFlag() )
    {
      if( pcSlice->getSliceType() == B_SLICE )
      {
#if JVET_O0238_PPS_OR_SLICE
        if (!pcSlice->getPPS()->getPPSCollocatedFromL0Idc())
        {
          WRITE_FLAG( pcSlice->getColFromL0Flag(), "collocated_from_l0_flag" );
        }
#else
        WRITE_FLAG( pcSlice->getColFromL0Flag(), "collocated_from_l0_flag" );
#endif
      }

      if( pcSlice->getSliceType() != I_SLICE &&
        ( ( pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > 1 ) ||
          ( pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > 1 ) ) )
      {
        WRITE_UVLC( pcSlice->getColRefIdx(), "collocated_ref_idx" );
      }
    }
#if JVET_O0455_IBC_MAX_MERGE_NUM
    if (!cs.slice->isIntra())
#else
    if (!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag())
#endif
    {
      CHECK(pcSlice->getMaxNumMergeCand() > MRG_MAX_NUM_CANDS, "More merge candidates signalled than supported");
#if JVET_O0238_PPS_OR_SLICE      
      if (!pcSlice->getPPS()->getPPSSixMinusMaxNumMergeCandPlus1())
      {
        WRITE_UVLC(MRG_MAX_NUM_CANDS - pcSlice->getMaxNumMergeCand(), "six_minus_max_num_merge_cand");
      }
#else
      WRITE_UVLC(MRG_MAX_NUM_CANDS - pcSlice->getMaxNumMergeCand(), "six_minus_max_num_merge_cand");
#endif
    }
    if( !pcSlice->isIntra() )
    {
#if JVET_O0263_O0220_SUBBLOCK_SYNTAX_CLEANUP
      if (pcSlice->getSPS()->getSBTMVPEnabledFlag() && pcSlice->getEnableTMVPFlag() && !pcSlice->getSPS()->getUseAffine())// ATMVP only
#else
      if ( pcSlice->getSPS()->getSBTMVPEnabledFlag() && !pcSlice->getSPS()->getUseAffine() ) // ATMVP only
#endif
      {
        CHECK( pcSlice->getMaxNumAffineMergeCand() != 1, "Sub-block merge can number should be 1" );
      }
      else
#if JVET_O0263_O0220_SUBBLOCK_SYNTAX_CLEANUP
      if (!(pcSlice->getSPS()->getSBTMVPEnabledFlag() && pcSlice->getEnableTMVPFlag()) && !pcSlice->getSPS()->getUseAffine()) // both off
#else
      if ( !pcSlice->getSPS()->getSBTMVPEnabledFlag() && !pcSlice->getSPS()->getUseAffine() ) // both off
#endif
      {
        CHECK( pcSlice->getMaxNumAffineMergeCand() != 0, "Sub-block merge can number should be 0" );
      }
      else
      if ( pcSlice->getSPS()->getUseAffine() )
      {
        CHECK( pcSlice->getMaxNumAffineMergeCand() > AFFINE_MRG_MAX_NUM_CANDS, "More affine merge candidates signalled than supported" );
#if JVET_O0238_PPS_OR_SLICE
        if (!pcSlice->getPPS()->getPPSFiveMinusMaxNumSubblockMergeCandPlus1())
        {
          WRITE_UVLC( AFFINE_MRG_MAX_NUM_CANDS - pcSlice->getMaxNumAffineMergeCand(), "five_minus_max_num_subblock_merge_cand" );
        }
#else
        WRITE_UVLC( AFFINE_MRG_MAX_NUM_CANDS - pcSlice->getMaxNumAffineMergeCand(), "five_minus_max_num_subblock_merge_cand" );
#endif
      }
      if ( pcSlice->getSPS()->getFpelMmvdEnabledFlag() )
      {
        WRITE_FLAG( pcSlice->getDisFracMMVD(), "slice_fpel_mmvd_enabled_flag" );
      }
#if JVET_O1140_SLICE_DISABLE_BDOF_DMVR_FLAG
      if (pcSlice->getSPS()->getBdofDmvrSlicePresentFlag())
      {
        WRITE_FLAG(pcSlice->getDisBdofDmvrFlag(), "slice_disable_bdof_dmvr_flag");
      }
#endif
      if (pcSlice->getSPS()->getUseTriangle() && pcSlice->getMaxNumMergeCand() >= 2)
      {
        CHECK(pcSlice->getMaxNumMergeCand() < pcSlice->getMaxNumTriangleCand(), "Incorrrect max number of triangle candidates!");
#if JVET_O0238_PPS_OR_SLICE
        if (!pcSlice->getPPS()->getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1()) 
        {
          WRITE_UVLC(pcSlice->getMaxNumMergeCand() - pcSlice->getMaxNumTriangleCand(), "max_num_merge_cand_minus_max_num_triangle_cand");
        }
#else
        WRITE_UVLC(pcSlice->getMaxNumMergeCand() - pcSlice->getMaxNumTriangleCand(), "max_num_merge_cand_minus_max_num_triangle_cand");
#endif
      }
      else
      {
        pcSlice->setMaxNumTriangleCand(0);
      }
    }
#if JVET_O0455_IBC_MAX_MERGE_NUM
    if ( pcSlice->getSPS()->getIBCFlag() )
    {
      CHECK( pcSlice->getMaxNumIBCMergeCand() > IBC_MRG_MAX_NUM_CANDS, "More IBC merge candidates signalled than supported" );
      WRITE_UVLC( IBC_MRG_MAX_NUM_CANDS - pcSlice->getMaxNumIBCMergeCand(), "slice_six_minus_max_num_ibc_merge_cand" );
    }
#endif
#if JVET_O0105_ICT
#if JVET_O0376_SPS_JOINTCBCR_FLAG
    if (pcSlice->getSPS()->getJointCbCrEnabledFlag())
    {
      WRITE_FLAG( pcSlice->getJointCbCrSignFlag() ? 1 : 0, "slice_joint_cbcr_sign_flag");
    }
#else
    if (chromaEnabled)
    {
      WRITE_FLAG( pcSlice->getJointCbCrSignFlag() ? 1 : 0, "slice_joint_cbcr_sign_flag");
    }
#endif
#endif

    int iCode = pcSlice->getSliceQp() - ( pcSlice->getPPS()->getPicInitQPMinus26() + 26 );
    WRITE_SVLC( iCode, "slice_qp_delta" );
    if (pcSlice->getPPS()->getSliceChromaQpFlag())
    {
      if (numberValidComponents > COMPONENT_Cb)
      {
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb), "slice_cb_qp_offset" );
      }
      if (numberValidComponents > COMPONENT_Cr)
      {
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr), "slice_cr_qp_offset" );
#if JVET_O0376_SPS_JOINTCBCR_FLAG
        if (pcSlice->getSPS()->getJointCbCrEnabledFlag())
        {
          WRITE_SVLC( pcSlice->getSliceChromaQpDelta(JOINT_CbCr), "slice_joint_cbcr_qp_offset");
        }
#else
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(JOINT_CbCr), "slice_joint_cbcr_qp_offset");
#endif
      }
      CHECK(numberValidComponents < COMPONENT_Cr+1, "Too many valid components");
    }

    if (pcSlice->getPPS()->getCuChromaQpOffsetEnabledFlag())
    {
      WRITE_FLAG(pcSlice->getUseChromaQpAdj(), "cu_chroma_qp_offset_enabled_flag");
    }

    if( ( pcSlice->getPPS()->getUseWP() && pcSlice->getSliceType() == P_SLICE ) || ( pcSlice->getPPS()->getWPBiPred() && pcSlice->getSliceType() == B_SLICE ) )
    {
      xCodePredWeightTable( pcSlice );
    }
    if( pcSlice->getSPS()->getSAOEnabledFlag() )
    {
      WRITE_FLAG( pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_LUMA ), "slice_sao_luma_flag" );
      if( chromaEnabled )
      {
        WRITE_FLAG( pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ), "slice_sao_chroma_flag" );
      }
    }

    if( pcSlice->getSPS()->getALFEnabledFlag() )
    {
      const int alfEnabled = pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y);
      WRITE_FLAG(alfEnabled, "slice_alf_enabled_flag");

      if (alfEnabled)
      {
#if JVET_O0288_UNIFY_ALF_SLICE_TYPE_REMOVAL
#if JVET_O_MAX_NUM_ALF_APS_8
        WRITE_CODE(pcSlice->getTileGroupNumAps(), 3, "slice_num_alf_aps_ids_luma");
#else
        xWriteTruncBinCode(pcSlice->getTileGroupNumAps(), ALF_CTB_MAX_NUM_APS + 1);
#endif
#else
        if (pcSlice->isIntra())
        {
          WRITE_FLAG(pcSlice->getTileGroupNumAps(), "slice_num_alf_aps_ids_luma");
        }
        else
        {
#if JVET_O_MAX_NUM_ALF_APS_8
          WRITE_CODE(pcSlice->getTileGroupNumAps(), 3, "slice_num_alf_aps_ids_luma");
#else
          xWriteTruncBinCode(pcSlice->getTileGroupNumAps(), ALF_CTB_MAX_NUM_APS + 1);
#endif
        }
#endif
        const std::vector<int>&   apsId = pcSlice->getTileGroupApsIdLuma();
        for (int i = 0; i < pcSlice->getTileGroupNumAps(); i++)
        {
#if JVET_O_MAX_NUM_ALF_APS_8
          WRITE_CODE(apsId[i], 3, "slice_alf_aps_id_luma");
#else
          WRITE_CODE(apsId[i], 5, "slice_alf_aps_id_luma");
#endif
        }

        const int alfChromaIdc = pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) + pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) * 2 ;
#if JVET_O0616_400_CHROMA_SUPPORT
        if (chromaEnabled)
        {
#endif
#if JVET_O0491_HLS_CLEANUP
          WRITE_CODE(alfChromaIdc, 2, "slice_alf_chroma_idc");
#else
          truncatedUnaryEqProb(alfChromaIdc, 3);   // alf_chroma_idc
#endif
#if JVET_O0616_400_CHROMA_SUPPORT
        }
#endif
        if (alfChromaIdc)
        {
#if JVET_O0288_UNIFY_ALF_SLICE_TYPE_REMOVAL
#if JVET_O_MAX_NUM_ALF_APS_8
          WRITE_CODE(pcSlice->getTileGroupApsIdChroma(), 3, "slice_alf_aps_id_chroma");
#else
          WRITE_CODE(pcSlice->getTileGroupApsIdChroma(), 5, "slice_alf_aps_id_chroma");
#endif
#else
          if (pcSlice->isIntra()&& pcSlice->getTileGroupNumAps() == 1)
          {
            CHECK(pcSlice->getTileGroupApsIdChroma() != apsId[0], "wrong tile group chroma aps id");
          }
          else
          {
#if JVET_O_MAX_NUM_ALF_APS_8
            WRITE_CODE(pcSlice->getTileGroupApsIdChroma(), 3, "slice_alf_aps_id_chroma");
#else
            WRITE_CODE(pcSlice->getTileGroupApsIdChroma(), 5, "slice_alf_aps_id_chroma");
#endif
          }
#endif
        }
      }
    }

#if JVET_O0238_PPS_OR_SLICE
    if (!pcSlice->getPPS()->getPPSDepQuantEnabledIdc())
    {
      WRITE_FLAG( pcSlice->getDepQuantEnabledFlag() ? 1 : 0, "dep_quant_enabled_flag" );
    }
#else
    WRITE_FLAG( pcSlice->getDepQuantEnabledFlag() ? 1 : 0, "dep_quant_enabled_flag" );
#endif
    if( !pcSlice->getDepQuantEnabledFlag() )
    {
      WRITE_FLAG( pcSlice->getSignDataHidingEnabledFlag() ? 1 : 0, "sign_data_hiding_enabled_flag" );
    }
    else
    {
      CHECK( pcSlice->getSignDataHidingEnabledFlag(), "sign data hiding not supported when dependent quantization is enabled" );
    }
    if (pcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
    {
      if (pcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() )
      {
        WRITE_FLAG(pcSlice->getDeblockingFilterOverrideFlag(), "deblocking_filter_override_flag");
      }
      if (pcSlice->getDeblockingFilterOverrideFlag())
      {
        WRITE_FLAG(pcSlice->getDeblockingFilterDisable(), "slice_deblocking_filter_disabled_flag");
        if(!pcSlice->getDeblockingFilterDisable())
        {
          WRITE_SVLC (pcSlice->getDeblockingFilterBetaOffsetDiv2(), "slice_beta_offset_div2");
          WRITE_SVLC (pcSlice->getDeblockingFilterTcOffsetDiv2(),   "slice_tc_offset_div2");
        }
      }
    }

    bool isSAOEnabled = pcSlice->getSPS()->getSAOEnabledFlag() && (pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_LUMA) || (chromaEnabled && pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA)));
    bool isDBFEnabled = (!pcSlice->getDeblockingFilterDisable());

    if(pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ))
    {
      WRITE_FLAG(pcSlice->getLFCrossSliceBoundaryFlag()?1:0, "slice_loop_filter_across_slices_enabled_flag");
    }

    if (pcSlice->getSPS()->getUseReshaper())
    {
      WRITE_FLAG( pcSlice->getLmcsEnabledFlag()? 1 : 0, "slice_lmcs_enabled_flag");
      if (pcSlice->getLmcsEnabledFlag())
      {
#if JVET_O0428_LMCS_CLEANUP
        WRITE_CODE(pcSlice->getLmcsAPSId(), 2, "slice_lmcs_aps_id");
#else
        WRITE_CODE(pcSlice->getLmcsAPSId(), 5, "slice_lmcs_aps_id");
#endif
#if !JVET_O1109_UNFIY_CRS
        if (!(pcSlice->getSPS()->getUseDualITree() && pcSlice->isIntra()))
#endif
#if JVET_O0616_400_CHROMA_SUPPORT
          if (chromaEnabled)
          {
#endif
            WRITE_FLAG(pcSlice->getLmcsChromaResidualScaleFlag(), "slice_chroma_residual_scale_flag");
#if JVET_O0616_400_CHROMA_SUPPORT
          }
#endif
      }
    }

#if JVET_O0299_APS_SCALINGLIST
    if( pcSlice->getSPS()->getScalingListFlag() )
    {
      WRITE_FLAG( pcSlice->getscalingListPresentFlag() ? 1 : 0, "slice_scaling_list_present_flag" );
      if( pcSlice->getscalingListPresentFlag() )
      {
        WRITE_CODE( pcSlice->getscalingListAPSId(), 3, "slice_scaling_list_aps_id" );
      }
    }
#endif
  if(pcSlice->getPPS()->getSliceHeaderExtensionPresentFlag())
  {
    WRITE_UVLC(0,"slice_segment_header_extension_length");
  }

}

void  HLSWriter::codeConstraintInfo  ( const ConstraintInfo* cinfo )
{
  WRITE_FLAG(cinfo->getProgressiveSourceFlag(),   "general_progressive_source_flag"         );
  WRITE_FLAG(cinfo->getInterlacedSourceFlag(),    "general_interlaced_source_flag"          );
  WRITE_FLAG(cinfo->getNonPackedConstraintFlag(), "general_non_packed_constraint_flag"      );
  WRITE_FLAG(cinfo->getFrameOnlyConstraintFlag(), "general_frame_only_constraint_flag"      );
  WRITE_FLAG(cinfo->getIntraOnlyConstraintFlag(),     "intra_only_constraint_flag"      );

  WRITE_CODE(cinfo->getMaxBitDepthConstraintIdc(), 4, "max_bitdepth_constraint_idc" );
  WRITE_CODE(cinfo->getMaxChromaFormatConstraintIdc(), 2, "max_chroma_format_constraint_idc" );

  WRITE_FLAG(cinfo->getNoQtbttDualTreeIntraConstraintFlag() ? 1 : 0, "no_qtbtt_dual_tree_intra_constraint_flag");
  WRITE_FLAG(cinfo->getNoPartitionConstraintsOverrideConstraintFlag() ? 1 : 0, "no_partition_constraints_override_constraint_flag");
  WRITE_FLAG(cinfo->getNoSaoConstraintFlag() ? 1 : 0, "no_sao_constraint_flag");
  WRITE_FLAG(cinfo->getNoAlfConstraintFlag() ? 1 : 0, "no_alf_constraint_flag");
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  WRITE_FLAG(cinfo->getNoJointCbCrConstraintFlag() ? 1 : 0, "no_joint_cbcr_constraint_flag");
#endif
#if !JVET_O0525_REMOVE_PCM
  WRITE_FLAG(cinfo->getNoPcmConstraintFlag() ? 1 : 0, "no_pcm_constraint_flag");
#endif
  WRITE_FLAG(cinfo->getNoRefWraparoundConstraintFlag() ? 1 : 0, "no_ref_wraparound_constraint_flag");
  WRITE_FLAG(cinfo->getNoTemporalMvpConstraintFlag() ? 1 : 0, "no_temporal_mvp_constraint_flag");
  WRITE_FLAG(cinfo->getNoSbtmvpConstraintFlag() ? 1 : 0, "no_sbtmvp_constraint_flag");
  WRITE_FLAG(cinfo->getNoAmvrConstraintFlag() ? 1 : 0, "no_amvr_constraint_flag");
  WRITE_FLAG(cinfo->getNoBdofConstraintFlag() ? 1 : 0, "no_bdof_constraint_flag");
  WRITE_FLAG(cinfo->getNoDmvrConstraintFlag() ? 1 : 0, "no_dmvr_constraint_flag");
  WRITE_FLAG(cinfo->getNoCclmConstraintFlag() ? 1 : 0, "no_cclm_constraint_flag");
  WRITE_FLAG(cinfo->getNoMtsConstraintFlag() ? 1 : 0, "no_mts_constraint_flag");
  WRITE_FLAG(cinfo->getNoSbtConstraintFlag() ? 1 : 0, "no_sbt_constraint_flag");
  WRITE_FLAG(cinfo->getNoAffineMotionConstraintFlag() ? 1 : 0, "no_affine_motion_constraint_flag");
  WRITE_FLAG(cinfo->getNoGbiConstraintFlag() ? 1 : 0, "no_gbi_constraint_flag");
  WRITE_FLAG(cinfo->getNoIbcConstraintFlag() ? 1 : 0, "no_ibc_constraint_flag");
  WRITE_FLAG(cinfo->getNoMhIntraConstraintFlag() ? 1 : 0, "no_mh_intra_constraint_flag");
  WRITE_FLAG(cinfo->getNoFPelMmvdConstraintFlag() ? 1 : 0, "no_fpel_mmvd_constraint_flag");
  WRITE_FLAG(cinfo->getNoTriangleConstraintFlag() ? 1 : 0, "no_triangle_constraint_flag");
  WRITE_FLAG(cinfo->getNoLadfConstraintFlag() ? 1 : 0, "no_ladf_constraint_flag");
  WRITE_FLAG(cinfo->getNoTransformSkipConstraintFlag() ? 1 : 0, "no_transform_skip_constraint_flag");
#if JVET_O1136_TS_BDPCM_SIGNALLING
  WRITE_FLAG(cinfo->getNoBDPCMConstraintFlag() ? 1 : 0, "no_bdpcm_constraint_flag");
#endif
  WRITE_FLAG(cinfo->getNoQpDeltaConstraintFlag() ? 1 : 0, "no_qp_delta_constraint_flag");
  WRITE_FLAG(cinfo->getNoDepQuantConstraintFlag() ? 1 : 0, "no_dep_quant_constraint_flag");
  WRITE_FLAG(cinfo->getNoSignDataHidingConstraintFlag() ? 1 : 0, "no_sign_data_hiding_constraint_flag");
}


void  HLSWriter::codeProfileTierLevel    ( const ProfileTierLevel* ptl, int maxNumSubLayersMinus1 )
{
  WRITE_CODE( int(ptl->getProfileIdc()), 7 ,   "general_profile_idc"                     );
  WRITE_FLAG( ptl->getTierFlag()==Level::HIGH, "general_tier_flag"                       );

#if JVET_O0044_MULTI_SUB_PROFILE
  WRITE_CODE(ptl->getNumSubProfile(), 8, "num_sub_profiles");
  for (int i = 0; i < ptl->getNumSubProfile(); i++)
  {
    WRITE_CODE(ptl->getSubProfileIdc(i) , 32, "general_sub_profile_idc[i]");
  }
#else
  WRITE_CODE(ptl->getSubProfileIdc(), 24, "general_sub_profile_idc");
#endif

  codeConstraintInfo(ptl->getConstraintInfo());

  WRITE_CODE( int(ptl->getLevelIdc()), 8 ,     "general_level_idc"                     );

  for (int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    WRITE_FLAG( ptl->getSubLayerLevelPresentFlag(i),   "sub_layer_level_present_flag[i]" );
  }

  while (!isByteAligned())
  {
    WRITE_FLAG(0, "ptl_alignment_zero_bit");
  }

  for(int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if( ptl->getSubLayerLevelPresentFlag(i) )
    {
      WRITE_CODE( int(ptl->getSubLayerLevelIdc(i)), 8, "sub_layer_level_idc[i]" );
    }
  }

}


/**
* Write tiles and wavefront substreams sizes for the slice header (entry points).
*
* \param pSlice Slice structure that contains the substream size information.
*/
void  HLSWriter::codeTilesWPPEntryPoint( Slice* pSlice )
{
  if (pSlice->getPPS()->getSingleTileInPicFlag() && !pSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
  {
    return;
  }
  uint32_t maxOffset = 0;
  for(int idx=0; idx<pSlice->getNumberOfSubstreamSizes(); idx++)
  {
    uint32_t offset=pSlice->getSubstreamSize(idx);
    if ( offset > maxOffset )
    {
      maxOffset = offset;
    }
  }

  // Determine number of bits "offsetLenMinus1+1" required for entry point information
  uint32_t offsetLenMinus1 = 0;
  while (maxOffset >= (1u << (offsetLenMinus1 + 1)))
  {
    offsetLenMinus1++;
    CHECK(offsetLenMinus1 + 1 >= 32, "Invalid offset length minus 1");
  }

#if !JVET_O0145_ENTRYPOINT_SIGNALLING
  WRITE_UVLC(pSlice->getNumberOfSubstreamSizes(), "num_entry_point_offsets");
#endif
  if (pSlice->getNumberOfSubstreamSizes()>0)
  {
    WRITE_UVLC(offsetLenMinus1, "offset_len_minus1");

    for (uint32_t idx=0; idx<pSlice->getNumberOfSubstreamSizes(); idx++)
    {
      WRITE_CODE(pSlice->getSubstreamSize(idx)-1, offsetLenMinus1+1, "entry_point_offset_minus1");
    }
  }
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! Code weighted prediction tables
void HLSWriter::xCodePredWeightTable( Slice* pcSlice )
{
  WPScalingParam  *wp;
  const ChromaFormat    format                = pcSlice->getSPS()->getChromaFormatIdc();
  const uint32_t            numberValidComponents = getNumberValidComponents(format);
  const bool            bChroma               = isChromaEnabled(format);
  const int             iNbRef                = (pcSlice->getSliceType() == B_SLICE ) ? (2) : (1);
  bool            bDenomCoded           = false;
  uint32_t            uiTotalSignalledWeightFlags = 0;

  if ( (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred()) )
  {
    for ( int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
    {
      RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

      // NOTE: wp[].uiLog2WeightDenom and wp[].bPresentFlag are actually per-channel-type settings.

      for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        if ( !bDenomCoded )
        {
          int iDeltaDenom;
          WRITE_UVLC( wp[COMPONENT_Y].uiLog2WeightDenom, "luma_log2_weight_denom" );

          if( bChroma )
          {
            CHECK( wp[COMPONENT_Cb].uiLog2WeightDenom != wp[COMPONENT_Cr].uiLog2WeightDenom, "Chroma blocks of different size not supported" );
            iDeltaDenom = (wp[COMPONENT_Cb].uiLog2WeightDenom - wp[COMPONENT_Y].uiLog2WeightDenom);
            WRITE_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
          }
          bDenomCoded = true;
        }
        WRITE_FLAG( wp[COMPONENT_Y].bPresentFlag, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
        uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
      }
      if (bChroma)
      {
        for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
        {
          pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );
          CHECK( wp[COMPONENT_Cb].bPresentFlag != wp[COMPONENT_Cr].bPresentFlag, "Inconsistent settings for chroma channels" );
          WRITE_FLAG( wp[COMPONENT_Cb].bPresentFlag, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
          uiTotalSignalledWeightFlags += 2*wp[COMPONENT_Cb].bPresentFlag;
        }
      }

      for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        if ( wp[COMPONENT_Y].bPresentFlag )
        {
          int iDeltaWeight = (wp[COMPONENT_Y].iWeight - (1<<wp[COMPONENT_Y].uiLog2WeightDenom));
          WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
          WRITE_SVLC( wp[COMPONENT_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        }

        if ( bChroma )
        {
          if ( wp[COMPONENT_Cb].bPresentFlag )
          {
            for ( int j = COMPONENT_Cb ; j < numberValidComponents ; j++ )
            {
              CHECK(wp[COMPONENT_Cb].uiLog2WeightDenom != wp[COMPONENT_Cr].uiLog2WeightDenom, "Chroma blocks of different size not supported");
              int iDeltaWeight = (wp[j].iWeight - (1<<wp[COMPONENT_Cb].uiLog2WeightDenom));
              WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );

              int range=pcSlice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA))/2 : 128;
              int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].uiLog2WeightDenom) ) );
              int iDeltaChroma = (wp[j].iOffset - pred);
              WRITE_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            }
          }
        }
      }
    }
    CHECK(uiTotalSignalledWeightFlags>24, "Too many signalled weight flags");
  }
}

/** code quantization matrix
*  \param scalingList quantization matrix information
*/
void HLSWriter::codeScalingList( const ScalingList &scalingList )
{
  //for each size
  for(uint32_t sizeId = SCALING_LIST_FIRST_CODED; sizeId <= SCALING_LIST_LAST_CODED; sizeId++)
  {
    const int predListStep = (sizeId > SCALING_LIST_32x32 ? (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) : 1); // if 64x64, skip over chroma entries.

    for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId+=predListStep)
    {
      if ((sizeId == SCALING_LIST_2x2) && ((listId % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0)))
      {
        continue;
      }
      bool scalingListPredModeFlag = scalingList.getScalingListPredModeFlag(sizeId, listId);
      WRITE_FLAG( scalingListPredModeFlag, "scaling_list_pred_mode_flag" );
      if(!scalingListPredModeFlag)// Copy Mode
      {
        if (sizeId > SCALING_LIST_32x32) //64x64 luma
        {
          // adjust the code, to cope with the missing chroma entries
          WRITE_UVLC( ((int)listId - (int)scalingList.getRefMatrixId(sizeId, listId)) / (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES), "scaling_list_pred_matrix_id_delta");
        }
        else
        {
          WRITE_UVLC( (int)listId - (int)scalingList.getRefMatrixId (sizeId,listId), "scaling_list_pred_matrix_id_delta");
        }
      }
      else// DPCM Mode
      {
        xCodeScalingList(&scalingList, sizeId, listId);
      }
    }
  }
  return;
}
/** code DPCM
* \param scalingList quantization matrix information
* \param sizeId      size index
* \param listId      list index
*/
void HLSWriter::xCodeScalingList(const ScalingList* scalingList, uint32_t sizeId, uint32_t listId)
{
  int coefNum = std::min( MAX_MATRIX_COEF_NUM, ( int ) g_scalingListSize[sizeId] );
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(1 << (sizeId == SCALING_LIST_2x2 ? 1 : (sizeId == SCALING_LIST_4x4 ? 2 : 3)))][gp_sizeIdxInfo->idxFrom(1 << (sizeId == SCALING_LIST_2x2 ? 1 : (sizeId == SCALING_LIST_4x4 ? 2 : 3)))];
  int nextCoef = SCALING_LIST_START_VALUE;
  int data;
  const int *src = scalingList->getScalingListAddress(sizeId, listId);
  if( sizeId > SCALING_LIST_8x8 )
  {
    WRITE_SVLC( scalingList->getScalingListDC(sizeId,listId) - 8, "scaling_list_dc_coef_minus8");
    nextCoef = scalingList->getScalingListDC(sizeId,listId);
  }
  for(int i=0;i<coefNum;i++)
  {
    if (sizeId == SCALING_LIST_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
      continue;
    data = src[scan[i].idx] - nextCoef;
    nextCoef = src[scan[i].idx];
    if(data > 127)
    {
      data = data - 256;
    }
    if(data < -128)
    {
      data = data + 256;
    }

    WRITE_SVLC( data,  "scaling_list_delta_coef");
  }
}

bool HLSWriter::xFindMatchingLTRP(Slice* pcSlice, uint32_t *ltrpsIndex, int ltrpPOC, bool usedFlag)
{
  // bool state = true, state2 = false;
  int lsb = ltrpPOC & ((1<<pcSlice->getSPS()->getBitsForPOC())-1);
  for (int k = 0; k < pcSlice->getSPS()->getNumLongTermRefPicSPS(); k++)
  {
    if ( (lsb == pcSlice->getSPS()->getLtRefPicPocLsbSps(k)) && (usedFlag == pcSlice->getSPS()->getUsedByCurrPicLtSPSFlag(k)) )
    {
      *ltrpsIndex = k;
      return true;
    }
  }
  return false;
}

void HLSWriter::alfGolombEncode( int coeff, int k, const bool signed_coeff )
{
  unsigned int symbol = abs( coeff );
  while ( symbol >= (unsigned int)( 1 << k ) )
  {
    symbol -= 1 << k;
    k++;
    WRITE_FLAG( 0, "alf_coeff_abs_prefix" );
  }
  WRITE_FLAG( 1, "alf_coeff_abs_prefix" );

  if ( k > 0 )
  {
    WRITE_CODE( symbol, k, "alf_coeff_abs_suffix" );
  }
  if ( signed_coeff && coeff != 0 )
  {
    WRITE_FLAG( (coeff < 0) ? 1 : 0, "alf_coeff_sign" );
  }
}

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
void HLSWriter::alfFilter( const AlfParam& alfParam, const bool isChroma, const int altIdx )
#else
void HLSWriter::alfFilter( const AlfParam& alfParam, const bool isChroma )
#endif
{
  if( !isChroma )
  {
    WRITE_FLAG( alfParam.alfLumaCoeffDeltaFlag, "alf_luma_coeff_delta_flag" );
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    if( !alfParam.alfLumaCoeffDeltaFlag )
    {
      if( alfParam.numLumaFilters > 1 )
      {
        WRITE_FLAG( alfParam.alfLumaCoeffDeltaPredictionFlag, "alf_luma_coeff_delta_prediction_flag" );
      }
    }
#endif
  }
  AlfFilterShape alfShape(isChroma ? 5 : 7);
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
  static int bitsCoeffScan[EncAdaptiveLoopFilter::m_MAX_SCAN_VAL][EncAdaptiveLoopFilter::m_MAX_EXP_GOLOMB];
  memset( bitsCoeffScan, 0, sizeof( bitsCoeffScan ) );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
#endif
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  const short* coeff = isChroma ? alfParam.chromaCoeff[altIdx] : alfParam.lumaCoeff;
  const short* clipp = isChroma ? alfParam.chromaClipp[altIdx] : alfParam.lumaClipp;
#else
  const short* coeff = isChroma ? alfParam.chromaCoeff : alfParam.lumaCoeff;
  const short* clipp = isChroma ? alfParam.chromaClipp : alfParam.lumaClipp;
#endif
  const int numFilters = isChroma ? 1 : alfParam.numLumaFilters;

  // vlc for all
#if !JVET_O0216_ALF_COEFF_EG3
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( isChroma || !alfParam.alfLumaCoeffDeltaFlag || alfParam.alfLumaCoeffFlag[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        int coeffVal = abs( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] );

        for( int k = 1; k < 15; k++ )
        {
          bitsCoeffScan[alfShape.golombIdx[i]][k] += EncAdaptiveLoopFilter::lengthGolomb( coeffVal, k );
        }
      }
    }
  }
#endif
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
  static int kMinTab[MAX_NUM_ALF_COEFF];
#endif
#if !JVET_O0216_ALF_COEFF_EG3
  int kMin = EncAdaptiveLoopFilter::getGolombKMin( alfShape, numFilters, kMinTab, bitsCoeffScan );
  // Golomb parameters
  WRITE_UVLC( kMin - 1,  isChroma ? "alf_chroma_min_eg_order_minus1" : "alf_luma_min_eg_order_minus1" );

  for( int idx = 0; idx < maxGolombIdx; idx++ )
  {
    bool golombOrderIncreaseFlag = ( kMinTab[idx] != kMin ) ? true : false;
    CHECK( !( kMinTab[idx] <= kMin + 1 ), "ALF Golomb parameter not consistent" );
    WRITE_FLAG( golombOrderIncreaseFlag, isChroma ? "alf_chroma_eg_order_increase_flag"  : "alf_luma_eg_order_increase_flag" );
    kMin = kMinTab[idx];
  }
#endif
  if( !isChroma )
  {
    if( alfParam.alfLumaCoeffDeltaFlag )
    {
      for( int ind = 0; ind < numFilters; ++ind )
      {
        WRITE_FLAG( alfParam.alfLumaCoeffFlag[ind], "alf_luma_coeff_flag[i]" );
      }
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfParam.alfLumaCoeffFlag[ind] && alfParam.alfLumaCoeffDeltaFlag )
    {
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
#if JVET_O0216_ALF_COEFF_EG3
      alfGolombEncode( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], 3 );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#else
      alfGolombEncode( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#endif
    }
  }

  // Clipping values coding
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  if( alfParam.nonLinearFlag[isChroma][altIdx] )
#else
  if( alfParam.nonLinearFlag[isChroma] )
#endif
  {
#if JVET_O0064_SIMP_ALF_CLIP_CODING
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        WRITE_CODE(clipp[ind* MAX_NUM_ALF_LUMA_COEFF + i], 2, "alf_clipping_index");
      }
    }
#else
    memset( bitsCoeffScan, 0, sizeof( bitsCoeffScan ) );

    short recCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
    if( isChroma )
    {
      memcpy( recCoeff, coeff, sizeof(short) * MAX_NUM_ALF_CHROMA_COEFF );
    }
    else
    {
      memcpy( recCoeff, coeff, sizeof(short) * numFilters * MAX_NUM_ALF_LUMA_COEFF );
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
      if( alfParam.alfLumaCoeffDeltaPredictionFlag )
      {
        for( int i = 1; i < numFilters; i++ )
        {
          for( int j = 0; j < alfShape.numCoeff - 1; j++ )
          {
            recCoeff[i * MAX_NUM_ALF_LUMA_COEFF + j] += recCoeff[( i - 1 ) * MAX_NUM_ALF_LUMA_COEFF + j];
          }
        }
      }
#endif
    }
    // vlc for all
    for( int ind = 0; ind < numFilters; ++ind )
    {
      if( isChroma || !alfParam.alfLumaCoeffDeltaFlag || alfParam.alfLumaCoeffFlag[ind] )
      {
        for( int i = 0; i < alfShape.numCoeff - 1; i++ )
        {
          if( !abs( recCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] ) )
            continue;
          int coeffVal = abs( clipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] );

          for( int k = 1; k < 15; k++ )
          {
            bitsCoeffScan[alfShape.golombIdx[i]][k] += EncAdaptiveLoopFilter::lengthGolomb( coeffVal, k, false );
          }
        }
      }
    }
#if JVET_O0216_ALF_COEFF_EG3
    int kMin = EncAdaptiveLoopFilter::getGolombKMin(alfShape, numFilters, kMinTab, bitsCoeffScan);
#else
    kMin = EncAdaptiveLoopFilter::getGolombKMin( alfShape, numFilters, kMinTab, bitsCoeffScan );
#endif

    // Golomb parameters
    WRITE_UVLC( kMin - 1, "clip_min_golomb_order" );

    for( int idx = 0; idx < maxGolombIdx; idx++ )
    {
      bool golombOrderIncreaseFlag = ( kMinTab[idx] != kMin ) ? true : false;
      CHECK( !( kMinTab[idx] <= kMin + 1 ), "ALF Golomb parameter not consistent" );
      WRITE_FLAG( golombOrderIncreaseFlag, "clip_golomb_order_increase_flag" );
      kMin = kMinTab[idx];
    }

    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {
      if( !isChroma && !alfParam.alfLumaCoeffFlag[ind] && alfParam.alfLumaCoeffDeltaFlag )
      {
        continue;
      }

      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        if( !abs( recCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] ) )
          continue;
        alfGolombEncode( clipp[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]], false );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
      }
    }
#endif
  }
}

#if !JVET_O0491_HLS_CLEANUP
void HLSWriter::xWriteTruncBinCode( uint32_t uiSymbol, const int uiMaxSymbol )
{
  int uiThresh;
  if( uiMaxSymbol > 256 )
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while( uiThreshVal <= uiMaxSymbol )
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_tbMax[uiMaxSymbol];
  }

  int uiVal = 1 << uiThresh;
  assert( uiVal <= uiMaxSymbol );
  assert( ( uiVal << 1 ) > uiMaxSymbol );
  assert( uiSymbol < uiMaxSymbol );
  int b = uiMaxSymbol - uiVal;
  assert( b < uiVal );
  if( uiSymbol < uiVal - b )
  {
    xWriteCode( uiSymbol, uiThresh );
  }
  else
  {
    uiSymbol += uiVal - b;
    assert( uiSymbol < ( uiVal << 1 ) );
    assert( ( uiSymbol >> 1 ) >= uiVal - b );
    xWriteCode( uiSymbol, uiThresh + 1 );
  }
}

void HLSWriter::truncatedUnaryEqProb( int symbol, const int maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }

  bool codeLast = ( maxSymbol > symbol );
  int bins = 0;
  int numBins = 0;

  while( symbol-- )
  {
    bins <<= 1;
    bins++;
    numBins++;
  }
  if( codeLast )
  {
    bins <<= 1;
    numBins++;
  }
  CHECK( !( numBins <= 32 ), "Unspecified error" );
  xWriteCode( bins, numBins );
}
#endif

//! \}
