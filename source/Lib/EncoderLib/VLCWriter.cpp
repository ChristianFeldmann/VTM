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
  CHECK( uiLength == 0, "Code of lenght '0' not supported" );
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

void HLSWriter::xCodeRefPicList(const ReferencePictureList* rpl, bool isLongTermPresent, uint32_t ltLsbBitsCount)
{
  WRITE_UVLC(rpl->getNumberOfShorttermPictures() + rpl->getNumberOfLongtermPictures(), "num_ref_entries[ listIdx ][ rplsIdx ]");
  uint32_t numRefPic = rpl->getNumberOfShorttermPictures() + rpl->getNumberOfLongtermPictures();
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;
  for (int ii = 0; ii < numRefPic; ii++)
  {
    if (rpl->getNumberOfLongtermPictures() > 0)
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
      WRITE_UVLC(absDeltaValue, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
      if (absDeltaValue > 0)
        WRITE_FLAG((deltaValue < 0) ? 0 : 1, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");  //0  means negative delta POC : 1 means positive
    }
    else
    {
      WRITE_CODE(rpl->getRefPicIdentifier(ii), ltLsbBitsCount, "poc_lsb_lt[listIdx][rplsIdx][i]");
    }
  }
}

void HLSWriter::codePPS( const PPS* pcPPS )
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif

  WRITE_UVLC( pcPPS->getPPSId(),                             "pps_pic_parameter_set_id" );
  WRITE_UVLC( pcPPS->getSPSId(),                             "pps_seq_parameter_set_id" );
  WRITE_FLAG( pcPPS->getOutputFlagPresentFlag() ? 1 : 0,     "output_flag_present_flag" );
  WRITE_CODE( pcPPS->getNumExtraSliceHeaderBits(), 3,        "num_extra_slice_header_bits");
  WRITE_FLAG( pcPPS->getCabacInitPresentFlag() ? 1 : 0,   "cabac_init_present_flag" );
  WRITE_UVLC( pcPPS->getNumRefIdxL0DefaultActive()-1,     "num_ref_idx_l0_default_active_minus1");
  WRITE_UVLC( pcPPS->getNumRefIdxL1DefaultActive()-1,     "num_ref_idx_l1_default_active_minus1");
  WRITE_FLAG(pcPPS->getRpl1IdxPresentFlag() ? 1 : 0, "rpl1IdxPresentFlag");

  WRITE_SVLC( pcPPS->getPicInitQPMinus26(),                  "init_qp_minus26");
  WRITE_FLAG( pcPPS->getConstrainedIntraPred() ? 1 : 0,      "constrained_intra_pred_flag" );
  WRITE_FLAG( pcPPS->getUseTransformSkip() ? 1 : 0,  "transform_skip_enabled_flag" );
  WRITE_FLAG( pcPPS->getUseDQP() ? 1 : 0, "cu_qp_delta_enabled_flag" );
  if ( pcPPS->getUseDQP() )
  {
    WRITE_UVLC( pcPPS->getCuQpDeltaSubdiv(), "cu_qp_delta_subdiv" );
  }

  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cb), "pps_cb_qp_offset" );
  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cr), "pps_cr_qp_offset" );
  WRITE_SVLC( pcPPS->getQpOffset(JOINT_CbCr),   "pps_cb_cr_qp_offset" );

  WRITE_FLAG( pcPPS->getSliceChromaQpFlag() ? 1 : 0,          "pps_slice_chroma_qp_offsets_present_flag" );

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

    int numTilesInPic = pcPPS->getUniformTileSpacingFlag() ? 0 : (pcPPS->getNumTileColumnsMinus1() + 1) * (pcPPS->getNumTileRowsMinus1() + 1);

    for( int i = 0; pcPPS->getBrickSplittingPresentFlag()  &&  i < numTilesInPic; i++ )
    {
      WRITE_FLAG( pcPPS->getBrickSplitFlag(i) ? 1 : 0, "brick_split_flag [i]" );
      if( pcPPS->getBrickSplitFlag(i) )
      {
        WRITE_FLAG( pcPPS->getUniformBrickSpacingFlag(i) ? 1 : 0, "uniform_brick_spacing_flag [i]" );
        if( pcPPS->getUniformBrickSpacingFlag(i) )
          WRITE_UVLC( pcPPS->getBrickHeightMinus1(i), "brick_height_minus1" );
        else
        {
          WRITE_UVLC( pcPPS->getNumBrickRowsMinus1(i), "num_brick_rows_minus1 [i]" );
          for(int j = 0; j < pcPPS->getNumBrickRowsMinus1(i); j++ )
            WRITE_UVLC( pcPPS->getBrickRowHeightMinus1(i,j), "brick_row_height_minus1 [i][j]" );
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
      int numTilesInPic = (pcPPS->getNumTileColumnsMinus1() + 1) * (pcPPS->getNumTileRowsMinus1() + 1);
      int codeLength = (int)ceil(log2(numTilesInPic));
      int codeLength2 = codeLength;
      for (int i = 0; i < numSlicesInPic; ++i)
      {
        if (i > 0)
        {
          WRITE_CODE(pcPPS->getTopLeftBrickIdx(i), codeLength, "top_left_brick_idx ");
          codeLength2 = (int)ceil(log2((numTilesInPic - pcPPS->getTopLeftBrickIdx(i) < 2) ? 2 : numTilesInPic - pcPPS->getTopLeftBrickIdx(i)));
        }
        WRITE_CODE(pcPPS->getBottomRightBrickIdx(i) - pcPPS->getTopLeftBrickIdx(i), codeLength2, "bottom_right_brick_idx_delta");
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
    int numBits = (int)ceil(log2(pcPPS->pcv->lumaWidth) - 3);
    for( unsigned i = 0; i < pcPPS->getNumVerVirtualBoundaries(); i++ )
    {
      WRITE_CODE( pcPPS->getVirtualBoundariesPosX( i ) >> 3, numBits,                "pps_virtual_boundaries_pos_x" );
    }
    WRITE_CODE( pcPPS->getNumHorVirtualBoundaries(), 2,                              "pps_num_hor_virtual_boundaries");
    numBits = (int)ceil(log2(pcPPS->pcv->lumaHeight) - 3);
    for( unsigned i = 0; i < pcPPS->getNumHorVirtualBoundaries(); i++ )
    {
      WRITE_CODE( pcPPS->getVirtualBoundariesPosY( i ) >> 3, numBits,                "pps_virtual_boundaries_pos_y" );
    }
  }

  WRITE_FLAG( pcPPS->getScalingListPresentFlag() ? 1 : 0,                          "pps_scaling_list_data_present_flag" );
  if( pcPPS->getScalingListPresentFlag() )
  {
    codeScalingList( pcPPS->getScalingList() );
  }
  WRITE_UVLC( pcPPS->getLog2ParallelMergeLevelMinus2(), "log2_parallel_merge_level_minus2");
  WRITE_FLAG( pcPPS->getSliceHeaderExtensionPresentFlag() ? 1 : 0, "slice_segment_header_extension_present_flag");

  bool pps_extension_present_flag=false;
  bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS]={false};

  pps_extension_flags[PPS_EXT__REXT] = pcPPS->getPpsRangeExtension().settingsDifferFromDefaults(pcPPS->getUseTransformSkip());

  // Other PPS extension flags checked here.

  for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
  {
    pps_extension_present_flag|=pps_extension_flags[i];
  }

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
          if (pcPPS->getUseTransformSkip())
          {
            WRITE_UVLC( ppsRangeExtension.getLog2MaxTransformSkipBlockSize()-2,            "log2_max_transform_skip_block_size_minus2");
          }

          WRITE_FLAG((ppsRangeExtension.getCrossComponentPredictionEnabledFlag() ? 1 : 0), "cross_component_prediction_enabled_flag" );

          WRITE_FLAG(uint32_t(ppsRangeExtension.getChromaQpOffsetListEnabledFlag()),           "chroma_qp_offset_list_enabled_flag" );
          if (ppsRangeExtension.getChromaQpOffsetListEnabledFlag())
          {
            WRITE_UVLC(ppsRangeExtension.getCuChromaQpOffsetSubdiv(),                      "cu_chroma_qp_offset_subdiv");
            WRITE_UVLC(ppsRangeExtension.getChromaQpOffsetListLen() - 1,                   "chroma_qp_offset_list_len_minus1");
            /* skip zero index */
            for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx < ppsRangeExtension.getChromaQpOffsetListLen(); cuChromaQpOffsetIdx++)
            {
              WRITE_SVLC(ppsRangeExtension.getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CbOffset,     "cb_qp_offset_list[i]");
              WRITE_SVLC(ppsRangeExtension.getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CrOffset,     "cr_qp_offset_list[i]");
            }
          }

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
  WRITE_CODE(pcAPS->getAPSType(), 3, "aps_params_type");
  

  if (pcAPS->getAPSType() == ALF_APS)
  {
    codeAlfAps(pcAPS);
  }
  else if (pcAPS->getAPSType() == LMCS_APS)
  {
    codeLmcsAps (pcAPS);
  }
  WRITE_FLAG(0, "aps_extension_flag");   //Implementation when this flag is equal to 1 should be added when it is needed. Currently in the spec we don't have case when this flag is equal to 1
  xWriteRbspTrailingBits();
}

void HLSWriter::codeAlfAps( APS* pcAPS )
{
  AlfSliceParam param = pcAPS->getAlfAPSParam();

  WRITE_FLAG(param.newFilterFlag[CHANNEL_TYPE_LUMA], "alf_luma_new_filter");
  WRITE_FLAG(param.newFilterFlag[CHANNEL_TYPE_CHROMA], "alf_chroma_new_filter");

  if (param.newFilterFlag[CHANNEL_TYPE_LUMA])
  {
    WRITE_FLAG(param.nonLinearFlag[CHANNEL_TYPE_LUMA], "alf_luma_clip");

    xWriteTruncBinCode(param.numLumaFilters - 1, MAX_NUM_ALF_CLASSES);  //number_of_filters_minus1
    if (param.numLumaFilters > 1)
    {
      for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
      {
        xWriteTruncBinCode((uint32_t)param.filterCoeffDeltaIdx[i], param.numLumaFilters);  //filter_coeff_delta[i]
      }
    }

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

    alfFilter(param, false);

  }
  if (param.newFilterFlag[CHANNEL_TYPE_CHROMA])
  {
    WRITE_FLAG(param.nonLinearFlag[CHANNEL_TYPE_CHROMA], "alf_chroma_clip");
      alfFilter(param, true);
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

void HLSWriter::codeHrdParameters( const HRDParameters *hrd, bool commonInfPresentFlag, uint32_t maxNumSubLayersMinus1 )
{
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
}


void HLSWriter::codeSPS( const SPS* pcSPS )
{
#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif
  WRITE_CODE( pcSPS->getDecodingParameterSetId (), 4,       "sps_decoding_parameter_set_id" );
#if HEVC_VPS
  WRITE_CODE( pcSPS->getVPSId (),          4,       "sps_video_parameter_set_id" );
#endif
  CHECK(pcSPS->getMaxTLayers() == 0, "Maximum number of temporal sub-layers is '0'");

  WRITE_CODE(pcSPS->getMaxTLayers() - 1, 3, "sps_max_sub_layers_minus1");
  WRITE_CODE(0,                          5, "sps_reserved_zero_5bits");

  codeProfileTierLevel( pcSPS->getProfileTierLevel(), pcSPS->getMaxTLayers() - 1 );

  WRITE_UVLC(pcSPS->getSPSId (), "sps_seq_parameter_set_id");

  WRITE_UVLC( int(pcSPS->getChromaFormatIdc ()),    "chroma_format_idc" );

  const ChromaFormat format                = pcSPS->getChromaFormatIdc();
  if( format == CHROMA_444 )
  {
    WRITE_FLAG( 0,                                  "separate_colour_plane_flag");
  }

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

  WRITE_UVLC( pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) - 8,                      "bit_depth_luma_minus8" );

  const bool         chromaEnabled         = isChromaEnabled(format);
  WRITE_UVLC( chromaEnabled ? (pcSPS->getBitDepth(CHANNEL_TYPE_CHROMA) - 8):0,  "bit_depth_chroma_minus8" );

  WRITE_UVLC( pcSPS->getBitsForPOC()-4,                 "log2_max_pic_order_cnt_lsb_minus4" );
  WRITE_FLAG( pcSPS->getIDRRefParamListPresent(),                 "sps_idr_rpl_present_flag" );
  // KJS: Marakech decision: sub-layers added back
  const bool subLayerOrderingInfoPresentFlag = 1;
  WRITE_FLAG(subLayerOrderingInfoPresentFlag,       "sps_sub_layer_ordering_info_present_flag");
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
  uint32_t numberOfRPL = pcSPS->getNumRPL0() - 1;

  //Write candidate for List0
  numberOfRPL = pcSPS->getNumRPL0() - 1;
  WRITE_UVLC(numberOfRPL, "num_ref_pic_lists_in_sps[0]");
  for (int ii = 0; ii < numberOfRPL; ii++)
  {
    const ReferencePictureList* rpl = rplList0->getReferencePictureList(ii);
    xCodeRefPicList(rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC());
  }

  //Write candidate for List1
  if (!pcSPS->getRPL1CopyFromRPL0Flag())
  {
    numberOfRPL = pcSPS->getNumRPL1() - 1;
    WRITE_UVLC(numberOfRPL, "num_ref_pic_lists_in_sps[1]");
    for (int ii = 0; ii < numberOfRPL; ii++)
    {
      const ReferencePictureList* rpl = rplList1->getReferencePictureList(ii);
      xCodeRefPicList(rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC());
    }
  }
  WRITE_FLAG(pcSPS->getUseDualITree(), "qtbtt_dual_tree_intra_flag");
  WRITE_UVLC(g_aucLog2[pcSPS->getCTUSize()] - MIN_CU_LOG2, "log2_ctu_size_minus2");
  WRITE_UVLC(pcSPS->getLog2MinCodingBlockSize() - 2, "log2_min_luma_coding_block_size_minus2");
  WRITE_FLAG(pcSPS->getSplitConsOverrideEnabledFlag(), "partition_constraints_override_enabled_flag");
  WRITE_UVLC(g_aucLog2[pcSPS->getMinQTSize(I_SLICE)] - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_intra_tile_group_luma");
  WRITE_UVLC(g_aucLog2[pcSPS->getMinQTSize(B_SLICE)] - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_inter_tile_group");
  WRITE_UVLC(pcSPS->getMaxBTDepth(), "sps_max_mtt_hierarchy_depth_inter_tile_group");
  WRITE_UVLC(pcSPS->getMaxBTDepthI(), "sps_max_mtt_hierarchy_depth_intra_tile_group_luma");
  if (pcSPS->getMaxBTDepthI() != 0)
  {
    WRITE_UVLC(g_aucLog2[pcSPS->getMaxBTSizeI()] - g_aucLog2[pcSPS->getMinQTSize(I_SLICE)], "sps_log2_diff_max_bt_min_qt_intra_tile_group_luma");
    WRITE_UVLC(g_aucLog2[pcSPS->getMaxTTSizeI()] - g_aucLog2[pcSPS->getMinQTSize(I_SLICE)], "sps_log2_diff_max_tt_min_qt_intra_tile_group_luma");
  }
  if (pcSPS->getMaxBTDepth() != 0)
  {
    WRITE_UVLC(g_aucLog2[pcSPS->getMaxBTSize()] - g_aucLog2[pcSPS->getMinQTSize(B_SLICE)], "sps_log2_diff_max_bt_min_qt_inter_tile_group");
    WRITE_UVLC(g_aucLog2[pcSPS->getMaxTTSize()] - g_aucLog2[pcSPS->getMinQTSize(B_SLICE)], "sps_log2_diff_max_tt_min_qt_inter_tile_group");
  }
  if (pcSPS->getUseDualITree())
  {
    WRITE_UVLC(g_aucLog2[pcSPS->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)] - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_intra_tile_group_chroma");
    WRITE_UVLC(pcSPS->getMaxBTDepthIChroma(), "sps_max_mtt_hierarchy_depth_intra_tile_group_chroma");
    if (pcSPS->getMaxBTDepthIChroma() != 0)
    {
      WRITE_UVLC(g_aucLog2[pcSPS->getMaxBTSizeIChroma()] - g_aucLog2[pcSPS->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)], "sps_log2_diff_max_bt_min_qt_intra_tile_group_chroma");
      WRITE_UVLC(g_aucLog2[pcSPS->getMaxTTSizeIChroma()] - g_aucLog2[pcSPS->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)], "sps_log2_diff_max_tt_min_qt_intra_tile_group_chroma");
    }
  }

#if MAX_TB_SIZE_SIGNALLING
  // KJS: Not in syntax
  WRITE_UVLC( pcSPS->getLog2MaxTbSize() - 2,                                 "log2_max_luma_transform_block_size_minus2" );
#endif
  WRITE_FLAG( pcSPS->getSAOEnabledFlag(),                                            "sps_sao_enabled_flag");
  WRITE_FLAG( pcSPS->getALFEnabledFlag(),                                            "sps_alf_enabled_flag" );

  WRITE_FLAG( pcSPS->getPCMEnabledFlag() ? 1 : 0,                                    "sps_pcm_enabled_flag");
  if( pcSPS->getPCMEnabledFlag() )
  {
    WRITE_CODE( pcSPS->getPCMBitDepth(CHANNEL_TYPE_LUMA) - 1, 4,                            "pcm_sample_bit_depth_luma_minus1" );
    WRITE_CODE( chromaEnabled ? (pcSPS->getPCMBitDepth(CHANNEL_TYPE_CHROMA) - 1) : 0, 4,    "pcm_sample_bit_depth_chroma_minus1" );
    WRITE_UVLC( pcSPS->getPCMLog2MinSize() - 3,                                      "log2_min_pcm_luma_coding_block_size_minus3" );
    WRITE_UVLC( pcSPS->getPCMLog2MaxSize() - pcSPS->getPCMLog2MinSize(),             "log2_diff_max_min_pcm_luma_coding_block_size" );
    WRITE_FLAG( pcSPS->getPCMFilterDisableFlag()?1 : 0,                              "pcm_loop_filter_disable_flag");
  }

  if( pcSPS->getCTUSize() + 2*(1 << pcSPS->getLog2MinCodingBlockSize()) <= pcSPS->getPicWidthInLumaSamples() )
  {    
  WRITE_FLAG( pcSPS->getWrapAroundEnabledFlag() ? 1 : 0,                              "sps_ref_wraparound_enabled_flag" );
  if( pcSPS->getWrapAroundEnabledFlag() )
  {
    WRITE_UVLC( (pcSPS->getWrapAroundOffset()/(1 <<  pcSPS->getLog2MinCodingBlockSize()))-1,  "sps_ref_wraparound_offset_minus1" );
  }
  }

  WRITE_FLAG( pcSPS->getSPSTemporalMVPEnabledFlag()  ? 1 : 0,                        "sps_temporal_mvp_enabled_flag" );

  if ( pcSPS->getSPSTemporalMVPEnabledFlag() )
  {
    WRITE_FLAG( pcSPS->getSBTMVPEnabledFlag() ? 1 : 0,                               "sps_sbtmvp_enabled_flag");
  }

  WRITE_FLAG( pcSPS->getAMVREnabledFlag() ? 1 : 0,                                   "sps_amvr_enabled_flag" );

  WRITE_FLAG( pcSPS->getBDOFEnabledFlag() ? 1 : 0,                                   "sps_bdof_enabled_flag" );

  WRITE_FLAG( pcSPS->getAffineAmvrEnabledFlag() ? 1 : 0,                             "sps_affine_amvr_enabled_flag" );

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
  }
  WRITE_FLAG( pcSPS->getUseGBi() ? 1 : 0,                                                      "gbi_flag" );
  WRITE_FLAG(pcSPS->getIBCFlag() ? 1 : 0,                                                      "ibc_flag");

  // KJS: sps_ciip_enabled_flag
  WRITE_FLAG( pcSPS->getUseMHIntra() ? 1 : 0,                                                  "mhintra_flag" );

  if ( pcSPS->getUseMMVD() )
  {
    WRITE_FLAG( pcSPS->getFpelMmvdEnabledFlag() ? 1 : 0,                            "sps_fpel_mmvd_enabled_flag" );
  }

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
#if INCLUDE_ISP_CFG_FLAG
  WRITE_FLAG( pcSPS->getUseISP() ? 1 : 0,                                             "isp_enable_flag");
#endif

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
  if(pcSPS->getScalingListFlag())
  {
    WRITE_FLAG( pcSPS->getScalingListPresentFlag() ? 1 : 0,                          "sps_scaling_list_data_present_flag" );
    if(pcSPS->getScalingListPresentFlag())
    {
      codeScalingList( pcSPS->getScalingList() );
    }
  }

  const TimingInfo *timingInfo = pcSPS->getTimingInfo();
  WRITE_FLAG(timingInfo->getTimingInfoPresentFlag(),          "timing_info_present_flag");
  if(timingInfo->getTimingInfoPresentFlag())
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "time_scale");
    WRITE_FLAG(pcSPS->getHrdParametersPresentFlag(),              "hrd_parameters_present_flag");
    if( pcSPS->getHrdParametersPresentFlag() )
    {
      codeHrdParameters(pcSPS->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
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

#if HEVC_VPS
void HLSWriter::codeVPS( const VPS* pcVPS )
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  WRITE_CODE( pcVPS->getVPSId(),                    4,        "vps_video_parameter_set_id" );
  WRITE_FLAG(                                       1,        "vps_base_layer_internal_flag" );
  WRITE_FLAG(                                       1,        "vps_base_layer_available_flag" );
  WRITE_CODE( 0,                                    6,        "vps_max_layers_minus1" );
  WRITE_CODE( pcVPS->getMaxTLayers() - 1,           3,        "vps_max_sub_layers_minus1" );
  WRITE_FLAG( pcVPS->getTemporalNestingFlag(),                "vps_temporal_id_nesting_flag" );
  CHECK(pcVPS->getMaxTLayers()<=1&&!pcVPS->getTemporalNestingFlag(), "Invalud parameters");
  WRITE_CODE( 0xffff,                              16,        "vps_reserved_0xffff_16bits" );
  codePTL( pcVPS->getPTL(), true, pcVPS->getMaxTLayers() - 1 );
  const bool subLayerOrderingInfoPresentFlag = 1;
  WRITE_FLAG(subLayerOrderingInfoPresentFlag,              "vps_sub_layer_ordering_info_present_flag");
  for(uint32_t i=0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    WRITE_UVLC( pcVPS->getMaxDecPicBuffering(i) - 1,       "vps_max_dec_pic_buffering_minus1[i]" );
    WRITE_UVLC( pcVPS->getNumReorderPics(i),               "vps_max_num_reorder_pics[i]" );
    WRITE_UVLC( pcVPS->getMaxLatencyIncrease(i),           "vps_max_latency_increase_plus1[i]" );
    if (!subLayerOrderingInfoPresentFlag)
    {
      break;
    }
  }

  CHECK( pcVPS->getNumHrdParameters() > MAX_VPS_NUM_HRD_PARAMETERS, "Too many HRD parameters" );
  CHECK( pcVPS->getMaxNuhReservedZeroLayerId() >= MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1, "Invalid parameters read" );
  WRITE_CODE( pcVPS->getMaxNuhReservedZeroLayerId(), 6,     "vps_max_layer_id" );
  WRITE_UVLC( pcVPS->getMaxOpSets() - 1,                    "vps_num_layer_sets_minus1" );
  for( uint32_t opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( uint32_t i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
    {
      // Only applicable for version 1
      // pcVPS->setLayerIdIncludedFlag( true, opsIdx, i );
      WRITE_FLAG( pcVPS->getLayerIdIncludedFlag( opsIdx, i ) ? 1 : 0, "layer_id_included_flag[opsIdx][i]" );
    }
  }
  const TimingInfo *timingInfo = pcVPS->getTimingInfo();
  WRITE_FLAG(timingInfo->getTimingInfoPresentFlag(),          "vps_timing_info_present_flag");
  if(timingInfo->getTimingInfoPresentFlag())
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "vps_num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "vps_time_scale");
    WRITE_FLAG(timingInfo->getPocProportionalToTimingFlag(),  "vps_poc_proportional_to_timing_flag");
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      WRITE_UVLC(timingInfo->getNumTicksPocDiffOneMinus1(),   "vps_num_ticks_poc_diff_one_minus1");
    }
    WRITE_UVLC( pcVPS->getNumHrdParameters(),                 "vps_num_hrd_parameters" );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      for( uint32_t i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
      {
        // Only applicable for version 1
        WRITE_UVLC( pcVPS->getHrdOpSetIdx( i ),                "hrd_layer_set_idx" );
        if( i > 0 )
        {
          WRITE_FLAG( pcVPS->getCprmsPresentFlag( i ) ? 1 : 0, "cprms_present_flag[i]" );
        }
        codeHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
      }
    }
  }
  WRITE_FLAG( 0,                     "vps_extension_flag" );

  //future extensions here..
  xWriteRbspTrailingBits();
}
#else
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
#endif

void HLSWriter::codeSliceHeader         ( Slice* pcSlice )
{
#if ENABLE_TRACING
  xTraceSliceHeader ();
#endif

  CodingStructure& cs = *pcSlice->getPic()->cs;
  const ChromaFormat format                = pcSlice->getSPS()->getChromaFormatIdc();
  const uint32_t         numberValidComponents = getNumberValidComponents(format);
  const bool         chromaEnabled         = isChromaEnabled(format);

  if ( pcSlice->getRapPicFlag() )
  {
    WRITE_FLAG( pcSlice->getNoOutputPriorPicsFlag() ? 1 : 0, "no_output_of_prior_pics_flag" );
  }
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
      WRITE_CODE(pcSlice->getPPS()->getSliceId(pcSlice->setSliceIndex()), bitsSliceAddress, "slice_address");
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
  
    for( int i = 0; i < pcSlice->getPPS()->getNumExtraSliceHeaderBits(); i++ )
    {
      WRITE_FLAG( 0, "slice_reserved_flag[]" );
    }

    WRITE_UVLC( pcSlice->getSliceType(), "slice_type" );

    if( pcSlice->getPPS()->getOutputFlagPresentFlag() )
    {
      WRITE_FLAG( pcSlice->getPicOutputFlag() ? 1 : 0, "pic_output_flag" );
    }

    int pocBits = pcSlice->getSPS()->getBitsForPOC();
    int pocMask = (1 << pocBits) - 1;
    WRITE_CODE(pcSlice->getPOC() & pocMask, pocBits, "slice_pic_order_cnt_lsb");
    if( !pcSlice->getIdrPicFlag() || pcSlice->getSPS()->getIDRRefParamListPresent())
    {
      //Write L0 related syntax elements
      if (pcSlice->getSPS()->getNumRPL0() > 0)
      {
        WRITE_FLAG(pcSlice->getRPL0idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[0]");
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
        xCodeRefPicList(pcSlice->getRPL0(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC());
      }
      //Deal POC Msb cycle signalling for LTRP
      if (pcSlice->getRPL0()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL0()->getNumberOfLongtermPictures() + pcSlice->getRPL0()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL0()->isRefPicLongterm(i))
          {
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
          xCodeRefPicList(pcSlice->getRPL1(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC());
        }
      }
      else
      {
        if (pcSlice->getSPS()->getNumRPL1() > 0)
        {
          WRITE_FLAG(pcSlice->getRPL1idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[1]");
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
          xCodeRefPicList(pcSlice->getRPL1(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC());
        }
      }
      //Deal POC Msb cycle signalling for LTRP
      if (pcSlice->getRPL1()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL1()->getNumberOfLongtermPictures() + pcSlice->getRPL1()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL1()->isRefPicLongterm(i))
          {
            WRITE_FLAG(pcSlice->getLocalRPL1()->getDeltaPocMSBPresentFlag(i) ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
            if (pcSlice->getLocalRPL1()->getDeltaPocMSBPresentFlag(i))
            {
              WRITE_UVLC(pcSlice->getLocalRPL1()->getDeltaPocMSBCycleLT(i), "delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }
      if( pcSlice->getSPS()->getSPSTemporalMVPEnabledFlag() )
      {
        WRITE_FLAG( pcSlice->getEnableTMVPFlag() ? 1 : 0, "slice_temporal_mvp_enabled_flag" );
      }
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
      WRITE_FLAG(alfEnabled, "tile_group_alf_enabled_flag");

      if (alfEnabled)
      {

        if (pcSlice->isIntra())
        {
          WRITE_FLAG(pcSlice->getTileGroupNumAps(), "tile_group_num_APS");
        }
        else
        {
          xWriteTruncBinCode(pcSlice->getTileGroupNumAps(), ALF_CTB_MAX_NUM_APS + 1);
        }

        const std::vector<int>&   apsId = pcSlice->getTileGroupApsIdLuma();
        for (int i = 0; i < pcSlice->getTileGroupNumAps(); i++)
        {
          WRITE_CODE(apsId[i], 5, "tile_group_aps_id");
        }

        const int alfChromaIdc = pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) + pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) * 2 ;
        truncatedUnaryEqProb(alfChromaIdc, 3);   // alf_chroma_idc
        if (alfChromaIdc)
        {
          if (pcSlice->isIntra()&& pcSlice->getTileGroupNumAps() == 1)
          {
            CHECK(pcSlice->getTileGroupApsIdChroma() != apsId[0], "wrong tile group chroma aps id");
          }
          else
          {
            WRITE_CODE(pcSlice->getTileGroupApsIdChroma(), 5, "tile_group_aps_id_chroma");
          }
        }
      }
    }

    //check if numrefidxes match the defaults. If not, override

    if( !pcSlice->isIntra() )
    {
      bool overrideFlag = ( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) != pcSlice->getPPS()->getNumRefIdxL0DefaultActive() || ( pcSlice->isInterB() && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) != pcSlice->getPPS()->getNumRefIdxL1DefaultActive() ) );
      WRITE_FLAG( overrideFlag ? 1 : 0, "num_ref_idx_active_override_flag" );
      if( overrideFlag )
      {
        WRITE_UVLC( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) - 1, "num_ref_idx_l0_active_minus1" );
        if( pcSlice->isInterB() )
        {
          WRITE_UVLC( pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) - 1, "num_ref_idx_l1_active_minus1" );
        }
        else
        {
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
        }
      }
    }
    else
    {
      pcSlice->setNumRefIdx( REF_PIC_LIST_0, 0 );
      pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
    }


    if( pcSlice->isInterB() )
    {
      WRITE_FLAG( pcSlice->getMvdL1ZeroFlag() ? 1 : 0, "mvd_l1_zero_flag" );
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
        WRITE_FLAG( pcSlice->getColFromL0Flag(), "collocated_from_l0_flag" );
      }

      if( pcSlice->getSliceType() != I_SLICE &&
        ( ( pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > 1 ) ||
          ( pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > 1 ) ) )
      {
        WRITE_UVLC( pcSlice->getColRefIdx(), "collocated_ref_idx" );
      }
    }
    if( ( pcSlice->getPPS()->getUseWP() && pcSlice->getSliceType() == P_SLICE ) || ( pcSlice->getPPS()->getWPBiPred() && pcSlice->getSliceType() == B_SLICE ) )
    {
      xCodePredWeightTable( pcSlice );
    }
    WRITE_FLAG( pcSlice->getDepQuantEnabledFlag() ? 1 : 0, "dep_quant_enabled_flag" );
    if( !pcSlice->getDepQuantEnabledFlag() )
    {
      WRITE_FLAG( pcSlice->getSignDataHidingEnabledFlag() ? 1 : 0, "sign_data_hiding_enabled_flag" );
    }
    else
    {
      CHECK( pcSlice->getSignDataHidingEnabledFlag(), "sign data hiding not supported when dependent quantization is enabled" );
    }
    if (
      pcSlice->getSPS()->getSplitConsOverrideEnabledFlag()
      )
    {
      WRITE_FLAG(pcSlice->getSplitConsOverrideFlag() ? 1 : 0, "partition_constrainst_override_flag");
      if (pcSlice->getSplitConsOverrideFlag())
      {
        WRITE_UVLC(g_aucLog2[pcSlice->getMinQTSize()] - pcSlice->getSPS()->getLog2MinCodingBlockSize(), "log2_diff_min_qt_min_cb");
        WRITE_UVLC(pcSlice->getMaxBTDepth(), "max_bt_depth");
        if (pcSlice->getMaxBTDepth() != 0)
        {
          CHECK(pcSlice->getMaxBTSize() < pcSlice->getMinQTSize(), "maxBtSize is smaller than minQtSize");
          WRITE_UVLC(g_aucLog2[pcSlice->getMaxBTSize()] - g_aucLog2[pcSlice->getMinQTSize()], "log2_diff_max_bt_min_qt");
          CHECK(pcSlice->getMaxTTSize() < pcSlice->getMinQTSize(), "maxTtSize is smaller than minQtSize");
          WRITE_UVLC(g_aucLog2[pcSlice->getMaxTTSize()] - g_aucLog2[pcSlice->getMinQTSize()], "log2_diff_max_tt_min_qt");
        }
        if (
          pcSlice->isIntra() && pcSlice->getSPS()->getUseDualITree()
          )
        {
          WRITE_UVLC(g_aucLog2[pcSlice->getMinQTSizeIChroma()] - pcSlice->getSPS()->getLog2MinCodingBlockSize(), "log2_diff_min_qt_min_cb_chroma");
          WRITE_UVLC(pcSlice->getMaxBTDepthIChroma(), "max_mtt_hierarchy_depth_chroma");
          if (pcSlice->getMaxBTDepthIChroma() != 0)
          {
            CHECK(pcSlice->getMaxBTSizeIChroma() < pcSlice->getMinQTSizeIChroma(), "maxBtSizeC is smaller than minQtSizeC");
            WRITE_UVLC(g_aucLog2[pcSlice->getMaxBTSizeIChroma()] - g_aucLog2[pcSlice->getMinQTSizeIChroma()], "log2_diff_max_bt_min_qt_chroma");
            CHECK(pcSlice->getMaxTTSizeIChroma() < pcSlice->getMinQTSizeIChroma(), "maxTtSizeC is smaller than minQtSizeC");
            WRITE_UVLC(g_aucLog2[pcSlice->getMaxTTSizeIChroma()] - g_aucLog2[pcSlice->getMinQTSizeIChroma()], "log2_diff_max_tt_min_qt_chroma");
          }
        }
      }
    }
    if (!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag())
    {
      CHECK(pcSlice->getMaxNumMergeCand() > MRG_MAX_NUM_CANDS, "More merge candidates signalled than supported");
      WRITE_UVLC(MRG_MAX_NUM_CANDS - pcSlice->getMaxNumMergeCand(), "six_minus_max_num_merge_cand");
    }
    if( !pcSlice->isIntra() )
    {

      if ( pcSlice->getSPS()->getSBTMVPEnabledFlag() && !pcSlice->getSPS()->getUseAffine() ) // ATMVP only
      {
        CHECK( pcSlice->getMaxNumAffineMergeCand() != 1, "Sub-block merge can number should be 1" );
      }
      else
      if ( !pcSlice->getSPS()->getSBTMVPEnabledFlag() && !pcSlice->getSPS()->getUseAffine() ) // both off
      {
        CHECK( pcSlice->getMaxNumAffineMergeCand() != 0, "Sub-block merge can number should be 0" );
      }
      else
      if ( pcSlice->getSPS()->getUseAffine() )
      {
        CHECK( pcSlice->getMaxNumAffineMergeCand() > AFFINE_MRG_MAX_NUM_CANDS, "More affine merge candidates signalled than supported" );
        WRITE_UVLC( AFFINE_MRG_MAX_NUM_CANDS - pcSlice->getMaxNumAffineMergeCand(), "five_minus_max_num_affine_merge_cand" );
      }
      if ( pcSlice->getSPS()->getFpelMmvdEnabledFlag() )
      {
        WRITE_FLAG( pcSlice->getDisFracMMVD(), "tile_group_fracmmvd_disabled_flag" );
      }
      if (pcSlice->getSPS()->getUseTriangle() && pcSlice->getMaxNumMergeCand() >= 2)
      {
        CHECK(pcSlice->getMaxNumMergeCand() < pcSlice->getMaxNumTriangleCand(), "Incorrrect max number of triangle candidates!");
        WRITE_UVLC(pcSlice->getMaxNumMergeCand() - pcSlice->getMaxNumTriangleCand(), "max_num_merge_cand_minus_max_num_triangle_cand");
      }
      else
      {
        pcSlice->setMaxNumTriangleCand(0);
      }
    }
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
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(JOINT_CbCr),   "slice_cb_cr_qp_offset" );
      }
      CHECK(numberValidComponents < COMPONENT_Cr+1, "Too many valid components");
    }

    if (pcSlice->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag())
    {
      WRITE_FLAG(pcSlice->getUseChromaQpAdj(), "cu_chroma_qp_offset_enabled_flag");
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
        WRITE_CODE(pcSlice->getLmcsAPSId(), 5, "slice_lmcs_aps_id");
        if (!(pcSlice->getSPS()->getUseDualITree() && pcSlice->isIntra()))
          WRITE_FLAG(pcSlice->getLmcsChromaResidualScaleFlag(), "slice_chroma_residual_scale_flag");
      }
    }

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
  WRITE_FLAG(cinfo->getNoPcmConstraintFlag() ? 1 : 0, "no_pcm_constraint_flag");
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
  WRITE_FLAG(cinfo->getNoQpDeltaConstraintFlag() ? 1 : 0, "no_qp_delta_constraint_flag");
  WRITE_FLAG(cinfo->getNoDepQuantConstraintFlag() ? 1 : 0, "no_dep_quant_constraint_flag");
  WRITE_FLAG(cinfo->getNoSignDataHidingConstraintFlag() ? 1 : 0, "no_sign_data_hiding_constraint_flag");
}


void  HLSWriter::codeProfileTierLevel    ( const ProfileTierLevel* ptl, int maxNumSubLayersMinus1 )
{
  WRITE_CODE( int(ptl->getProfileIdc()), 7 ,   "general_profile_idc"                     );
  WRITE_FLAG( ptl->getTierFlag()==Level::HIGH, "general_tier_flag"                       );
  WRITE_CODE( ptl->getSubProfileIdc(), 24,      "general_sub_profile_idc"                );

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
    CHECK(offsetLenMinus1 + 1 >= 32, "Invalid offset lenght minus 1");
  }

  WRITE_UVLC(pSlice->getNumberOfSubstreamSizes(), "num_entry_point_offsets");
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
    const int predListStep = (sizeId > SCALING_LIST_32x32 ? (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) : 1); // if 64x64, skip over chroma entries.

    for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId+=predListStep)
    {
      if ((sizeId == SCALING_LIST_2x2) && ((listId % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) == 0)))
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
          WRITE_UVLC( ((int)listId - (int)scalingList.getRefMatrixId(sizeId, listId)) / (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)), "scaling_list_pred_matrix_id_delta");
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
  int symbol = abs( coeff );

  int m = (int)pow( 2.0, k );
  int q = symbol / m;

  for( int i = 0; i < q; i++ )
  {
    xWriteFlag( 1 );
  }
  xWriteFlag( 0 );
  // write one zero

  for( int i = 0; i < k; i++ )
  {
    xWriteFlag( symbol & 0x01 );
    symbol >>= 1;
  }

  if( signed_coeff && coeff != 0 )
  {
    int sign = ( coeff < 0 ) ? 1 : 0;
    xWriteFlag( sign );
  }
}

void HLSWriter::alfFilter( const AlfSliceParam& alfSliceParam, const bool isChroma )
{
  if( !isChroma )
  {
    WRITE_FLAG( alfSliceParam.alfLumaCoeffDeltaFlag, "alf_luma_coeff_delta_flag" );
    if( !alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      if( alfSliceParam.numLumaFilters > 1 )
      {
        WRITE_FLAG( alfSliceParam.alfLumaCoeffDeltaPredictionFlag, "alf_luma_coeff_delta_prediction_flag" );
      }
    }
  }

  static int bitsCoeffScan[EncAdaptiveLoopFilter::m_MAX_SCAN_VAL][EncAdaptiveLoopFilter::m_MAX_EXP_GOLOMB];
  memset( bitsCoeffScan, 0, sizeof( bitsCoeffScan ) );
  AlfFilterShape alfShape( isChroma ? 5 : 7 );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  const short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;
  const short* clipp = isChroma ? alfSliceParam.chromaClipp : alfSliceParam.lumaClipp;
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;

  // vlc for all
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( isChroma || !alfSliceParam.alfLumaCoeffDeltaFlag || alfSliceParam.alfLumaCoeffFlag[ind] )
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

  static int kMinTab[MAX_NUM_ALF_COEFF];
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

  if( !isChroma )
  {
    if( alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      for( int ind = 0; ind < numFilters; ++ind )
      {
        WRITE_FLAG( alfSliceParam.alfLumaCoeffFlag[ind], "alf_luma_coeff_flag[i]" );
      }
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfSliceParam.alfLumaCoeffFlag[ind] && alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      alfGolombEncode( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
    }
  }

  // Clipping values coding
  if( alfSliceParam.nonLinearFlag[isChroma] )
  {
    memset( bitsCoeffScan, 0, sizeof( bitsCoeffScan ) );

    short recCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
    if( isChroma )
    {
      memcpy( recCoeff, coeff, sizeof(short) * MAX_NUM_ALF_CHROMA_COEFF );
    }
    else
    {
      memcpy( recCoeff, coeff, sizeof(short) * numFilters * MAX_NUM_ALF_LUMA_COEFF );

      if( alfSliceParam.alfLumaCoeffDeltaPredictionFlag )
      {
        for( int i = 1; i < numFilters; i++ )
        {
          for( int j = 0; j < alfShape.numCoeff - 1; j++ )
          {
            recCoeff[i * MAX_NUM_ALF_LUMA_COEFF + j] += recCoeff[( i - 1 ) * MAX_NUM_ALF_LUMA_COEFF + j];
          }
        }
      }
    }
    // vlc for all
    for( int ind = 0; ind < numFilters; ++ind )
    {
      if( isChroma || !alfSliceParam.alfLumaCoeffDeltaFlag || alfSliceParam.alfLumaCoeffFlag[ind] )
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

    kMin = EncAdaptiveLoopFilter::getGolombKMin( alfShape, numFilters, kMinTab, bitsCoeffScan );

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
      if( !isChroma && !alfSliceParam.alfLumaCoeffFlag[ind] && alfSliceParam.alfLumaCoeffDeltaFlag )
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
  }
}

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

//! \}
