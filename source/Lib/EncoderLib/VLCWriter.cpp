/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2020, ITU/ISO/IEC
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

void  VLCWriter::xWriteSCodeTr (int value, uint32_t  length, const char *pSymbolName)
{
  xWriteSCode (value,length);
  if( g_HLSTraceEnable )
  {
    if( length<10 )
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %d\n", pSymbolName, length, value );
    }
    else
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %d\n", pSymbolName, length, value );
    }
  }
}

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

void VLCWriter::xWriteSCode    ( int code, uint32_t length )
{
  assert ( length > 0 && length<=32 );
  assert( length==32 || (code>=-(1<<(length-1)) && code<(1<<(length-1))) );
  m_pcBitIf->write( length==32 ? uint32_t(code) : ( uint32_t(code)&((1<<length)-1) ), length );
}

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

void HLSWriter::xCodeRefPicList( const ReferencePictureList* rpl, bool isLongTermPresent, uint32_t ltLsbBitsCount, const bool isForbiddenZeroDeltaPoc )
{
  uint32_t numRefPic = rpl->getNumberOfShorttermPictures() + rpl->getNumberOfLongtermPictures() + rpl->getNumberOfInterLayerPictures();
  WRITE_UVLC( numRefPic, "num_ref_entries[ listIdx ][ rplsIdx ]" );

  if (isLongTermPresent)
  {
    WRITE_FLAG(rpl->getLtrpInSliceHeaderFlag(), "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
  }
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;
  for (int ii = 0; ii < numRefPic; ii++)
  {
    if( rpl->getInterLayerPresentFlag() )
    {
      WRITE_FLAG( rpl->isInterLayerRefPic( ii ), "inter_layer_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]" );

      if( rpl->isInterLayerRefPic( ii ) )
      {
        CHECK( rpl->getInterLayerRefPicIdx( ii ) < 0, "Wrong inter-layer reference index" );
        WRITE_UVLC( rpl->getInterLayerRefPicIdx( ii ), "ilrp_idx[ listIdx ][ rplsIdx ][ i ]" );
      }
    }

    if( !rpl->isInterLayerRefPic( ii ) )
    {
    if( isLongTermPresent )
    {
      WRITE_FLAG( !rpl->isRefPicLongterm( ii ), "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]" );
    }

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
      if( isForbiddenZeroDeltaPoc )
      {
        CHECK( !absDeltaValue, "Zero delta POC is not used without WP" );
        WRITE_UVLC( absDeltaValue - 1, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]" );
      }
      else
      WRITE_UVLC(absDeltaValue, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
      if (absDeltaValue > 0)
        WRITE_FLAG((deltaValue < 0) ? 0 : 1, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");  //0  means negative delta POC : 1 means positive
    }
    else if (!rpl->getLtrpInSliceHeaderFlag())
    {
      WRITE_CODE(rpl->getRefPicIdentifier(ii), ltLsbBitsCount, "poc_lsb_lt[listIdx][rplsIdx][i]");
    }
    }
  }
}

void HLSWriter::codePPS( const PPS* pcPPS )
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif

  WRITE_UVLC( pcPPS->getPPSId(),                             "pps_pic_parameter_set_id" );
  WRITE_CODE( pcPPS->getSPSId(), 4,                          "pps_seq_parameter_set_id" );

#if SPS_ID_CHECK
  WRITE_FLAG( pcPPS->getMixedNaluTypesInPicFlag() ? 1 : 0,   "mixed_nalu_types_in_pic_flag" );
#endif

  WRITE_UVLC( pcPPS->getPicWidthInLumaSamples(), "pic_width_in_luma_samples" );
  WRITE_UVLC( pcPPS->getPicHeightInLumaSamples(), "pic_height_in_luma_samples" );
  Window conf = pcPPS->getConformanceWindow();
#if JVET_Q0260_CONFORMANCE_WINDOW_IN_SPS
  WRITE_FLAG(conf.getWindowEnabledFlag(), "pps_conformance_window_flag");
  if (conf.getWindowEnabledFlag())
  {
    WRITE_UVLC(conf.getWindowLeftOffset(), "pps_conf_win_left_offset");
    WRITE_UVLC(conf.getWindowRightOffset(), "pps_conf_win_right_offset");
    WRITE_UVLC(conf.getWindowTopOffset(), "pps_conf_win_top_offset");
    WRITE_UVLC(conf.getWindowBottomOffset(), "pps_conf_win_bottom_offset");
  }
#else
  WRITE_FLAG( conf.getWindowEnabledFlag(), "conformance_window_flag" );
  if( conf.getWindowEnabledFlag() )
  {
    WRITE_UVLC( conf.getWindowLeftOffset(),   "conf_win_left_offset" );
    WRITE_UVLC( conf.getWindowRightOffset(),  "conf_win_right_offset" );
    WRITE_UVLC( conf.getWindowTopOffset(),    "conf_win_top_offset" );
    WRITE_UVLC( conf.getWindowBottomOffset(), "conf_win_bottom_offset" );
  }
#endif
  Window scalingWindow = pcPPS->getScalingWindow();

  WRITE_FLAG( scalingWindow.getWindowEnabledFlag(), "scaling_window_flag" );
  if( scalingWindow.getWindowEnabledFlag() )
  {
    WRITE_UVLC( scalingWindow.getWindowLeftOffset(), "scaling_win_left_offset" );
    WRITE_UVLC( scalingWindow.getWindowRightOffset(), "scaling_win_right_offset" );
    WRITE_UVLC( scalingWindow.getWindowTopOffset(), "scaling_win_top_offset" );
    WRITE_UVLC( scalingWindow.getWindowBottomOffset(), "scaling_win_bottom_offset" );
  }

  WRITE_FLAG( pcPPS->getOutputFlagPresentFlag() ? 1 : 0,     "output_flag_present_flag" );
#if JVET_Q0119_CLEANUPS
  WRITE_FLAG( pcPPS->getSubPicIdMappingInPpsFlag() ? 1 : 0, "subpic_id_mapping_in_pps_flag" );
  if( pcPPS->getSubPicIdMappingInPpsFlag() )
#else
  WRITE_FLAG(pcPPS->getSubPicIdSignallingPresentFlag(), "pps_subpic_id_signalling_present_flag");
  if( pcPPS->getSubPicIdSignallingPresentFlag() )
#endif
  {
    WRITE_UVLC( pcPPS->getNumSubPics() - 1, "pps_num_subpics_minus1" );

    WRITE_UVLC( pcPPS->getSubPicIdLen() - 1, "pps_subpic_id_len_minus1" );

#if JVET_Q0169_SUBPIC_LEN_CONFORM    
    CHECK((1 << pcPPS->getSubPicIdLen()) < pcPPS->getNumSubPics(), "pps_subpic_id_len exceeds valid range");
#endif
    for( int picIdx = 0; picIdx < pcPPS->getNumSubPics( ); picIdx++ )
    {
      WRITE_CODE( pcPPS->getSubPicId(picIdx), pcPPS->getSubPicIdLen( ), "pps_subpic_id[i]" );
    }
  }

  WRITE_FLAG( pcPPS->getNoPicPartitionFlag( ) ? 1 : 0, "no_pic_partition_flag" );
  if( !pcPPS->getNoPicPartitionFlag() )
  {
    int colIdx, rowIdx;

    // CTU size - required to match size in SPS
    WRITE_CODE( pcPPS->getLog2CtuSize() - 5, 2, "pps_log2_ctu_size_minus5" );
    
    // number of explicit tile columns/rows
    WRITE_UVLC( pcPPS->getNumExpTileColumns() - 1, "num_exp_tile_columns_minus1" );
    WRITE_UVLC( pcPPS->getNumExpTileRows() - 1,    "num_exp_tile_rows_minus1" );
        
    // tile sizes
    for( colIdx = 0; colIdx < pcPPS->getNumExpTileColumns(); colIdx++ )
    {
      WRITE_UVLC( pcPPS->getTileColumnWidth( colIdx ) - 1, "tile_column_width_minus1[i]" );
    }
    for( rowIdx = 0; rowIdx < pcPPS->getNumExpTileRows(); rowIdx++ )
    {
      WRITE_UVLC( pcPPS->getTileRowHeight( rowIdx ) - 1, "tile_row_height_minus1[i]" );
    }
     
    // rectangular slice signalling
    WRITE_FLAG( pcPPS->getRectSliceFlag( ) ? 1 : 0, "rect_slice_flag");
    if (pcPPS->getRectSliceFlag())
    {
      WRITE_FLAG(pcPPS->getSingleSlicePerSubPicFlag( ) ? 1 : 0, "single_slice_per_subpic_flag");
    }
    if (pcPPS->getRectSliceFlag() & !(pcPPS->getSingleSlicePerSubPicFlag()))
    {      
      WRITE_UVLC( pcPPS->getNumSlicesInPic( ) - 1, "num_slices_in_pic_minus1" );
      WRITE_FLAG( pcPPS->getTileIdxDeltaPresentFlag( ) ? 1 : 0, "tile_idx_delta_present_flag");
      
      // write rectangular slice parameters
      for( int i = 0; i < pcPPS->getNumSlicesInPic()-1; i++ )
      {
        // complete tiles within a single slice
        WRITE_UVLC( pcPPS->getSliceWidthInTiles( i ) - 1,  "slice_width_in_tiles_minus1[i]" );
#if JVET_Q0480_RASTER_RECT_SLICES
        if( pcPPS->getTileIdxDeltaPresentFlag() || ( (pcPPS->getSliceTileIdx( i ) % pcPPS->getNumTileColumns()) == 0 ) )
        {
          WRITE_UVLC( pcPPS->getSliceHeightInTiles( i ) - 1, "slice_height_in_tiles_minus1[i]" );
        }
#else
        WRITE_UVLC( pcPPS->getSliceHeightInTiles( i ) - 1, "slice_height_in_tiles_minus1[i]" );
#endif

        // multiple slices within a single tile special case
#if JVET_Q0203_MULTI_SLICE_IN_TILE
        if( pcPPS->getSliceWidthInTiles(i) == 1 && pcPPS->getSliceHeightInTiles(i) == 1 && pcPPS->getTileRowHeight(pcPPS->getSliceTileIdx(i) / pcPPS->getNumTileColumns()) > 1 )
        {
          uint32_t numExpSliceInTile = pcPPS->getNumSlicesInTile(i) - 1;
          if( pcPPS->getSliceHeightInCtu(i + numExpSliceInTile - 2) >= pcPPS->getSliceHeightInCtu(i + numExpSliceInTile - 1) )
          {
            for( int j = numExpSliceInTile - 2; j >= 0; j-- )
            {
              if( pcPPS->getSliceHeightInCtu(i + j) == pcPPS->getSliceHeightInCtu(i + j + 1) )
              {
                numExpSliceInTile--;
              }
            }
          }
          WRITE_UVLC(numExpSliceInTile, "num_exp_slices_in_tile[i]");
          for( int j = 0; j < numExpSliceInTile; j++ )
          {
            WRITE_UVLC(pcPPS->getSliceHeightInCtu(i + j) - 1, "exp_slice_height_in_ctus_minus1[i]");
          }
          i += (pcPPS->getNumSlicesInTile(i) - 1);
        }
#else
        if( pcPPS->getSliceWidthInTiles( i ) == 1 && pcPPS->getSliceHeightInTiles( i ) == 1 ) 
        {
          WRITE_UVLC( pcPPS->getNumSlicesInTile( i ) - 1,  "num_slices_in_tile_minus1[i]" );
          uint32_t numSlicesInTile = pcPPS->getNumSlicesInTile( i );
          for( int j = 0; j < numSlicesInTile-1; j++ )
          {
            WRITE_UVLC( pcPPS->getSliceHeightInCtu( i ) - 1,  "slice_height_in_ctu_minus1[i]" );
            i++;
          }
        }
#endif

        // tile index offset to start of next slice
        if( i < pcPPS->getNumSlicesInPic()-1 ) 
        {
          if( pcPPS->getTileIdxDeltaPresentFlag() ) 
          {
            int32_t  tileIdxDelta = pcPPS->getSliceTileIdx( i + 1 ) - pcPPS->getSliceTileIdx( i );
            WRITE_SVLC( tileIdxDelta,  "tile_idx_delta[i]" );
          }
        }
      }
    }

    // loop filtering across slice/tile controls
    WRITE_FLAG( pcPPS->getLoopFilterAcrossTilesEnabledFlag(), "loop_filter_across_tiles_enabled_flag");
    WRITE_FLAG( pcPPS->getLoopFilterAcrossSlicesEnabledFlag(), "loop_filter_across_slices_enabled_flag");
  }

#if !JVET_Q0151_Q0205_ENTRYPOINTS
  WRITE_FLAG( pcPPS->getEntropyCodingSyncEnabledFlag() ? 1 : 0, "entropy_coding_sync_enabled_flag" );
#endif
  WRITE_FLAG( pcPPS->getCabacInitPresentFlag() ? 1 : 0,   "cabac_init_present_flag" );
  WRITE_UVLC( pcPPS->getNumRefIdxL0DefaultActive()-1,     "num_ref_idx_l0_default_active_minus1");
  WRITE_UVLC( pcPPS->getNumRefIdxL1DefaultActive()-1,     "num_ref_idx_l1_default_active_minus1");
  WRITE_FLAG( pcPPS->getRpl1IdxPresentFlag() ? 1 : 0,     "rpl1_idx_present_flag");


  WRITE_SVLC( pcPPS->getPicInitQPMinus26(),                  "init_qp_minus26");
#if !JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
  WRITE_UVLC( pcPPS->getLog2MaxTransformSkipBlockSize() - 2, "log2_transform_skip_max_size_minus2");
#endif
  WRITE_FLAG( pcPPS->getUseDQP() ? 1 : 0, "cu_qp_delta_enabled_flag" );
#if JVET_Q0420_PPS_CHROMA_TOOL_FLAG
  WRITE_FLAG(pcPPS->getPPSChromaToolFlag() ? 1 : 0, "pps_chroma_tool_offsets_present_flag");
  if (pcPPS->getPPSChromaToolFlag())
  {
#endif
  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cb), "pps_cb_qp_offset" );
  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cr), "pps_cr_qp_offset" );
  WRITE_FLAG(pcPPS->getJointCbCrQpOffsetPresentFlag() ? 1 : 0, "pps_joint_cbcr_qp_offset_present_flag");
  if (pcPPS->getJointCbCrQpOffsetPresentFlag())
  {
    WRITE_SVLC(pcPPS->getQpOffset(JOINT_CbCr), "pps_joint_cbcr_qp_offset_value");
  }

  WRITE_FLAG( pcPPS->getSliceChromaQpFlag() ? 1 : 0,          "pps_slice_chroma_qp_offsets_present_flag" );

  WRITE_FLAG(uint32_t(pcPPS->getCuChromaQpOffsetListEnabledFlag()),         "pps_cu_chroma_qp_offset_list_enabled_flag" );
  if (pcPPS->getCuChromaQpOffsetListEnabledFlag())
  {
    WRITE_UVLC(pcPPS->getChromaQpOffsetListLen() - 1,                   "chroma_qp_offset_list_len_minus1");
    /* skip zero index */
    for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx < pcPPS->getChromaQpOffsetListLen(); cuChromaQpOffsetIdx++)
    {
      WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CbOffset,     "cb_qp_offset_list[i]");
      WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CrOffset,     "cr_qp_offset_list[i]");
      if (pcPPS->getJointCbCrQpOffsetPresentFlag())
      {
        WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1).u.comp.JointCbCrOffset, "joint_cbcr_qp_offset_list[i]");
      }
    }
  }
#if JVET_Q0420_PPS_CHROMA_TOOL_FLAG
  }
#endif

  WRITE_FLAG( pcPPS->getUseWP() ? 1 : 0,  "weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcPPS->getWPBiPred() ? 1 : 0, "weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)

  WRITE_FLAG( pcPPS->getDeblockingFilterControlPresentFlag()?1 : 0,       "deblocking_filter_control_present_flag");
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    WRITE_FLAG( pcPPS->getDeblockingFilterOverrideEnabledFlag() ? 1 : 0,  "deblocking_filter_override_enabled_flag" );
    WRITE_FLAG( pcPPS->getPPSDeblockingFilterDisabledFlag() ? 1 : 0,      "pps_deblocking_filter_disabled_flag" );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      WRITE_SVLC( pcPPS->getDeblockingFilterBetaOffsetDiv2(),             "pps_beta_offset_div2" );
      WRITE_SVLC( pcPPS->getDeblockingFilterTcOffsetDiv2(),               "pps_tc_offset_div2" );
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
      WRITE_SVLC( pcPPS->getDeblockingFilterCbBetaOffsetDiv2(),           "pps_cb_beta_offset_div2" );
      WRITE_SVLC( pcPPS->getDeblockingFilterCbTcOffsetDiv2(),             "pps_cb_tc_offset_div2" );
      WRITE_SVLC( pcPPS->getDeblockingFilterCrBetaOffsetDiv2(),           "pps_cr_beta_offset_div2" );
      WRITE_SVLC( pcPPS->getDeblockingFilterCrTcOffsetDiv2(),             "pps_cr_tc_offset_div2" );
#endif
    }
  }

#if JVET_Q0819_PH_CHANGES
  WRITE_FLAG(pcPPS->getRplInfoInPhFlag() ? 1 : 0, "rpl_info_in_ph_flag");
  if (pcPPS->getDeblockingFilterOverrideEnabledFlag())
  {
    WRITE_FLAG(pcPPS->getDbfInfoInPhFlag() ? 1 : 0, "dbf_info_in_ph_flag");
  }
  WRITE_FLAG(pcPPS->getSaoInfoInPhFlag() ? 1 : 0, "sao_info_in_ph_flag");
  WRITE_FLAG(pcPPS->getAlfInfoInPhFlag() ? 1 : 0, "alf_info_in_ph_flag");
  if ((pcPPS->getUseWP() || pcPPS->getWPBiPred()) && pcPPS->getRplInfoInPhFlag())
  {
    WRITE_FLAG(pcPPS->getWpInfoInPhFlag() ? 1 : 0, "wp_info_in_ph_flag");
  }
  WRITE_FLAG(pcPPS->getQpDeltaInfoInPhFlag() ? 1 : 0, "qp_delta_info_in_ph_flag");
#endif

#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
  WRITE_FLAG( pcPPS->getConstantSliceHeaderParamsEnabledFlag(),              "constant_slice_header_params_enabled_flag");
  if ( pcPPS->getConstantSliceHeaderParamsEnabledFlag() ) {
    WRITE_CODE( pcPPS->getPPSDepQuantEnabledIdc(), 2,                        "pps_dep_quant_enabled_idc");
    WRITE_CODE( pcPPS->getPPSRefPicListSPSIdc0(), 2,                         "pps_ref_pic_list_sps_idc[0]");
    WRITE_CODE( pcPPS->getPPSRefPicListSPSIdc1(), 2,                         "pps_ref_pic_list_sps_idc[1]");
    WRITE_CODE( pcPPS->getPPSMvdL1ZeroIdc(), 2,                              "pps_mvd_l1_zero_idc");
    WRITE_CODE( pcPPS->getPPSCollocatedFromL0Idc(), 2,                       "pps_collocated_from_l0_idc");
    WRITE_UVLC( pcPPS->getPPSSixMinusMaxNumMergeCandPlus1(),                 "pps_six_minus_max_num_merge_cand_plus1");
#if !JVET_Q0806
    WRITE_UVLC( pcPPS->getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1(),  "pps_max_num_merge_cand_minus_max_num_triangle_cand_plus1");
#else
    WRITE_UVLC(pcPPS->getPPSMaxNumMergeCandMinusMaxNumGeoCandPlus1(), "pps_max_num_merge_cand_minus_max_num_gpm_cand_plus1");
#endif
  }
#endif


  WRITE_FLAG( pcPPS->getPictureHeaderExtensionPresentFlag() ? 1 : 0, "picture_header_extension_present_flag");
  WRITE_FLAG( pcPPS->getSliceHeaderExtensionPresentFlag() ? 1 : 0, "slice_header_extension_present_flag");

  bool pps_extension_present_flag=false;
  bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS]={false};

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
#if !JVET_Q0441_SAO_MOD_12_BIT
          WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA),           "log2_sao_offset_scale_luma"   );
          WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA),         "log2_sao_offset_scale_chroma" );
#endif
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
  WRITE_CODE( (int)pcAPS->getAPSType(), 3, "aps_params_type" );

  if (pcAPS->getAPSType() == ALF_APS)
  {
    codeAlfAps(pcAPS);
  }
  else if (pcAPS->getAPSType() == LMCS_APS)
  {
    codeLmcsAps (pcAPS);
  }
  else if( pcAPS->getAPSType() == SCALING_LIST_APS )
  {
    codeScalingListAps( pcAPS );
  }
  WRITE_FLAG(0, "aps_extension_flag");   //Implementation when this flag is equal to 1 should be added when it is needed. Currently in the spec we don't have case when this flag is equal to 1
  xWriteRbspTrailingBits();
}

void HLSWriter::codeAlfAps( APS* pcAPS )
{
  AlfParam param = pcAPS->getAlfAPSParam();

  WRITE_FLAG(param.newFilterFlag[CHANNEL_TYPE_LUMA], "alf_luma_new_filter");
  WRITE_FLAG(param.newFilterFlag[CHANNEL_TYPE_CHROMA], "alf_chroma_new_filter");

#if JVET_Q0795_CCALF
  CcAlfFilterParam paramCcAlf = pcAPS->getCcAlfAPSParam();
  WRITE_FLAG(paramCcAlf.newCcAlfFilter[COMPONENT_Cb - 1], "alf_cc_cb_filter_signal_flag");
  WRITE_FLAG(paramCcAlf.newCcAlfFilter[COMPONENT_Cr - 1], "alf_cc_cr_filter_signal_flag");
#endif

  if (param.newFilterFlag[CHANNEL_TYPE_LUMA])
  {
#if JVET_Q0249_ALF_CHROMA_CLIPFLAG
    WRITE_FLAG( param.nonLinearFlag[CHANNEL_TYPE_LUMA], "alf_luma_clip" );
#else
    WRITE_FLAG( param.nonLinearFlag[CHANNEL_TYPE_LUMA][0], "alf_luma_clip" );
#endif

    WRITE_UVLC(param.numLumaFilters - 1, "alf_luma_num_filters_signalled_minus1");
    if (param.numLumaFilters > 1)
    {
      const int length =  ceilLog2( param.numLumaFilters);
      for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
      {
        WRITE_CODE(param.filterCoeffDeltaIdx[i], length, "alf_luma_coeff_delta_idx" );
      }
    }
    alfFilter(param, false, 0);

  }
  if (param.newFilterFlag[CHANNEL_TYPE_CHROMA])
  {
#if JVET_Q0249_ALF_CHROMA_CLIPFLAG
    WRITE_FLAG(param.nonLinearFlag[CHANNEL_TYPE_CHROMA], "alf_nonlinear_enable_flag_chroma");
#endif
    if( MAX_NUM_ALF_ALTERNATIVES_CHROMA > 1 )
      WRITE_UVLC( param.numAlternativesChroma - 1, "alf_chroma_num_alts_minus1" );
    for( int altIdx=0; altIdx < param.numAlternativesChroma; ++altIdx )
    {
#if !JVET_Q0249_ALF_CHROMA_CLIPFLAG
      WRITE_FLAG( param.nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx], "alf_nonlinear_enable_flag_chroma" );
#endif
      alfFilter(param, true, altIdx);
    }
  }
#if JVET_Q0795_CCALF
  for (int ccIdx = 0; ccIdx < 2; ccIdx++)
  {
    if (paramCcAlf.newCcAlfFilter[ccIdx])
    {
      const int filterCount = paramCcAlf.ccAlfFilterCount[ccIdx];
      CHECK(filterCount > MAX_NUM_CC_ALF_FILTERS, "CC ALF Filter count is too large");
      CHECK(filterCount == 0, "CC ALF Filter count is too small");

      if (MAX_NUM_CC_ALF_FILTERS > 1)
      {
        WRITE_UVLC(filterCount - 1,
                   ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1");
      }

      for (int filterIdx = 0; filterIdx < filterCount; filterIdx++)
      {
        AlfFilterShape alfShape(size_CC_ALF);

        const short *coeff = paramCcAlf.ccAlfCoeff[ccIdx][filterIdx];
        // Filter coefficients
        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
          if (coeff[i] == 0)
          {
            WRITE_CODE(0, CCALF_BITS_PER_COEFF_LEVEL,
                       ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
          }
          else
          {
            WRITE_CODE(1 + floorLog2(abs(coeff[i])), CCALF_BITS_PER_COEFF_LEVEL,
                       ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
            WRITE_FLAG(coeff[i] < 0 ? 1 : 0, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign");
          }
        }

        DTRACE(g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx);
        for (int i = 0; i < alfShape.numCoeff; i++)
        {
          DTRACE(g_trace_ctx, D_SYNTAX, "%d ", coeff[i]);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "\n");
      }
    }
  }
#endif
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
  int deltaCRS = param.chrResScalingOffset;
  int signCRS = (deltaCRS < 0) ? 1 : 0;
  int absCRS = (deltaCRS < 0) ? (-deltaCRS) : deltaCRS;
  WRITE_CODE(absCRS, 3, "lmcs_delta_abs_crs");
  if (absCRS > 0)
  {
    WRITE_FLAG(signCRS, "lmcs_delta_sign_crs_flag");
  }
}

void HLSWriter::codeScalingListAps( APS* pcAPS )
{
  ScalingList param = pcAPS->getScalingList();
  codeScalingList( param );
}

#if JVET_Q0042_VUI
void HLSWriter::codeVUI( const VUI *pcVUI, const SPS* pcSPS )
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif


  WRITE_FLAG(pcVUI->getAspectRatioInfoPresentFlag(),            "vui_aspect_ratio_info_present_flag");
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    WRITE_FLAG(pcVUI->getAspectRatioConstantFlag(),             "vui_aspect_ratio_constant_flag");
    WRITE_CODE(pcVUI->getAspectRatioIdc(), 8,                   "vui_aspect_ratio_idc" );
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      WRITE_CODE(pcVUI->getSarWidth(), 16,                      "vui_sar_width");
      WRITE_CODE(pcVUI->getSarHeight(), 16,                     "vui_sar_height");
    }
  }
  WRITE_FLAG(pcVUI->getOverscanInfoPresentFlag(),               "vui_overscan_info_present_flag");
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    WRITE_FLAG(pcVUI->getOverscanAppropriateFlag(),             "vui_overscan_appropriate_flag");
  }
  WRITE_FLAG(pcVUI->getColourDescriptionPresentFlag(),        "vui_colour_description_present_flag");
  if (pcVUI->getColourDescriptionPresentFlag())
  {
    WRITE_CODE(pcVUI->getColourPrimaries(), 8,                "vui_colour_primaries");
    WRITE_CODE(pcVUI->getTransferCharacteristics(), 8,        "vui_transfer_characteristics");
    WRITE_CODE(pcVUI->getMatrixCoefficients(), 8,             "vui_matrix_coeffs");
    WRITE_FLAG(pcVUI->getVideoFullRangeFlag(),                "vui_video_full_range_flag");
  }
  WRITE_FLAG(pcVUI->getChromaLocInfoPresentFlag(),              "vui_chroma_loc_info_present_flag");
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    if(pcSPS->getProfileTierLevel()->getConstraintInfo()->getProgressiveSourceFlag() &&
       !pcSPS->getProfileTierLevel()->getConstraintInfo()->getInterlacedSourceFlag())
    {
      WRITE_UVLC(pcVUI->getChromaSampleLocType(),         "vui_chroma_sample_loc_type");
    }
    else 
    {
      WRITE_UVLC(pcVUI->getChromaSampleLocTypeTopField(),         "vui_chroma_sample_loc_type_top_field");
      WRITE_UVLC(pcVUI->getChromaSampleLocTypeBottomField(),      "vui_chroma_sample_loc_type_bottom_field");
    }
  }
}
#else
void HLSWriter::codeVUI( const VUI *pcVUI, const SPS* pcSPS )
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif


  WRITE_FLAG(pcVUI->getAspectRatioInfoPresentFlag(),            "aspect_ratio_info_present_flag");
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    WRITE_FLAG(pcVUI->getAspectRatioConstantFlag(),             "aspect_ratio_constant_flag");
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
    WRITE_FLAG(pcVUI->getVideoFullRangeFlag(),                "video_full_range_flag");
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
}
#endif

void HLSWriter::codeHrdParameters( const HRDParameters *hrd, const uint32_t firstSubLayer, const uint32_t maxNumSubLayersMinus1)
{
  WRITE_FLAG( hrd->getNalHrdParametersPresentFlag() ? 1 : 0 ,  "general_nal_hrd_parameters_present_flag" );
  WRITE_FLAG( hrd->getVclHrdParametersPresentFlag() ? 1 : 0 ,  "general_vcl_hrd_parameters_present_flag" );
  WRITE_FLAG( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() ? 1 : 0,  "general_decoding_unit_hrd_params_present_flag" );
  if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
  {
    WRITE_CODE( hrd->getTickDivisorMinus2(), 8,            "tick_divisor_minus2" );
  }
  WRITE_CODE( hrd->getBitRateScale(), 4,                     "bit_rate_scale" );
  WRITE_CODE( hrd->getCpbSizeScale(), 4,                     "cpb_size_scale" );
  if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
  {
    WRITE_CODE( hrd->getCpbSizeDuScale(), 4,               "cpb_size_du_scale" );
  }

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
}

#if JVET_P0117_PTL_SCALABILITY
void HLSWriter::dpb_parameters(int maxSubLayersMinus1, bool subLayerInfoFlag, const SPS *pcSPS)
{
  for (uint32_t i = (subLayerInfoFlag ? 0 : maxSubLayersMinus1); i <= maxSubLayersMinus1; i++)
  {
    WRITE_UVLC(pcSPS->getMaxDecPicBuffering(i) - 1, "max_dec_pic_buffering_minus1[i]");
    WRITE_UVLC(pcSPS->getNumReorderPics(i),                 "max_num_reorder_pics[i]");
    WRITE_UVLC(pcSPS->getMaxLatencyIncreasePlus1(i),  "max_latency_increase_plus1[i]");
  }
}
#endif

void HLSWriter::codeSPS( const SPS* pcSPS )
{
#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif
#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  WRITE_CODE(pcSPS->getSPSId(), 4, "sps_seq_parameter_set_id");
#else
  WRITE_CODE( pcSPS->getDecodingParameterSetId (), 4,       "sps_decoding_parameter_set_id" );
#endif
  WRITE_CODE( pcSPS->getVPSId(), 4, "sps_video_parameter_set_id" );
  CHECK(pcSPS->getMaxTLayers() == 0, "Maximum number of temporal sub-layers is '0'");

  WRITE_CODE(pcSPS->getMaxTLayers() - 1, 3, "sps_max_sub_layers_minus1");
#if JVET_P0117_PTL_SCALABILITY
  WRITE_CODE(0,                          4, "sps_reserved_zero_4bits");
  WRITE_FLAG(pcSPS->getPtlDpbHrdParamsPresentFlag(), "sps_ptl_dpb_hrd_params_present_flag");
#else
  WRITE_CODE(0,                          5, "sps_reserved_zero_5bits");
#endif

#if JVET_P0117_PTL_SCALABILITY
  if (pcSPS->getPtlDpbHrdParamsPresentFlag())
  {
#endif
#if JVET_Q0786_PTL_only
    codeProfileTierLevel(pcSPS->getProfileTierLevel(), true, pcSPS->getMaxTLayers() - 1);
#else
    codeProfileTierLevel(pcSPS->getProfileTierLevel(), pcSPS->getMaxTLayers() - 1);
#endif
#if JVET_P0117_PTL_SCALABILITY
  }
#endif
  
  WRITE_FLAG(pcSPS->getGDREnabledFlag(), "gdr_enabled_flag");
#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  WRITE_CODE( pcSPS->getSPSId (), 4, "sps_seq_parameter_set_id" );
#endif
  WRITE_CODE(int(pcSPS->getChromaFormatIdc ()), 2, "chroma_format_idc");

  const ChromaFormat format                = pcSPS->getChromaFormatIdc();
#if JVET_Q0265
  const uint32_t  separate_colour_plane_flag = pcSPS->getSeparateColourPlaneFlag();
  if( format == CHROMA_444 )
  {
     CHECK(separate_colour_plane_flag != 0,         "separate_colour_plane_flag is not '0'");
     WRITE_FLAG( separate_colour_plane_flag,        "separate_colour_plane_flag");
  }

  const uint32_t chromaArrayType = separate_colour_plane_flag ? 0 : format;
#else
  if( format == CHROMA_444 )
  {
    WRITE_FLAG( 0,                                  "separate_colour_plane_flag");
  }
#endif

#if JVET_Q0043_RPR_and_Subpics
  WRITE_FLAG( pcSPS->getRprEnabledFlag(), "res_change_in_clvs_allowed_flag" );
#else
  WRITE_FLAG( pcSPS->getRprEnabledFlag(), "ref_pic_resampling_enabled_flag" );
#endif

  WRITE_UVLC( pcSPS->getMaxPicWidthInLumaSamples(), "pic_width_max_in_luma_samples" );
  WRITE_UVLC( pcSPS->getMaxPicHeightInLumaSamples(), "pic_height_max_in_luma_samples" );
#if JVET_Q0260_CONFORMANCE_WINDOW_IN_SPS
  Window conf = pcSPS->getConformanceWindow();
  WRITE_FLAG(conf.getWindowEnabledFlag(), "sps_conformance_window_flag");
  if (conf.getWindowEnabledFlag())
  {
    WRITE_UVLC(conf.getWindowLeftOffset(), "sps_conf_win_left_offset");
    WRITE_UVLC(conf.getWindowRightOffset(), "sps_conf_win_right_offset");
    WRITE_UVLC(conf.getWindowTopOffset(), "sps_conf_win_top_offset");
    WRITE_UVLC(conf.getWindowBottomOffset(), "sps_conf_win_bottom_offset");
  }

#endif
  WRITE_CODE(floorLog2(pcSPS->getCTUSize()) - 5, 2, "sps_log2_ctu_size_minus5");

#if JVET_Q0043_RPR_and_Subpics | JVET_Q0119_CLEANUPS
#if JVET_Q0119_CLEANUPS
  WRITE_FLAG(pcSPS->getSubPicInfoPresentFlag(), "subpic_info_present_flag");
#else
  WRITE_FLAG(pcSPS->getSubPicPresentFlag(), "subpic_info_present_flag");
#endif
#else
  WRITE_FLAG(pcSPS->getSubPicPresentFlag(), "subpics_present_flag");
#endif
#if JVET_Q0119_CLEANUPS
  if (pcSPS->getSubPicInfoPresentFlag())
#else
  if(pcSPS->getSubPicPresentFlag())
#endif
  {
#if JVET_Q0119_CLEANUPS
    WRITE_UVLC(pcSPS->getNumSubPics() - 1, "sps_num_subpics_minus1");
#else
    WRITE_CODE(pcSPS->getNumSubPics() - 1, 8, "sps_num_subpics_minus1");
#endif
#if JVET_Q0816
    if( pcSPS->getNumSubPics() > 1 )
    {
#endif
    for (int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++)
    {
#if JVET_Q0787_SUBPIC
      if (pcSPS->getMaxPicWidthInLumaSamples() > pcSPS->getCTUSize())
      {
        WRITE_CODE( pcSPS->getSubPicCtuTopLeftX(picIdx), ceilLog2(( pcSPS->getMaxPicWidthInLumaSamples()  +  pcSPS->getCTUSize() - 1)  / pcSPS->getCTUSize()), "subpic_ctu_top_left_x[ i ]"  );
      }
      if (pcSPS->getMaxPicHeightInLumaSamples() > pcSPS->getCTUSize())
      {
        WRITE_CODE( pcSPS->getSubPicCtuTopLeftY(picIdx), ceilLog2(( pcSPS->getMaxPicHeightInLumaSamples() +  pcSPS->getCTUSize() - 1)  / pcSPS->getCTUSize()), "subpic_ctu_top_left_y[ i ]"  );
      }
#if  JVET_Q0413_SKIP_LAST_SUBPIC_SIG
      if (picIdx<pcSPS->getNumSubPics()-1 && pcSPS->getMaxPicWidthInLumaSamples() > pcSPS->getCTUSize())
#else
      if (pcSPS->getMaxPicWidthInLumaSamples() > pcSPS->getCTUSize())
#endif
      {
        WRITE_CODE( pcSPS->getSubPicWidth(picIdx) - 1,   ceilLog2(( pcSPS->getMaxPicWidthInLumaSamples()  +  pcSPS->getCTUSize() - 1)  / pcSPS->getCTUSize()), "subpic_width_minus1[ i ]"    );
      }
#if  JVET_Q0413_SKIP_LAST_SUBPIC_SIG
      if (picIdx<pcSPS->getNumSubPics() - 1 && pcSPS->getMaxPicHeightInLumaSamples() > pcSPS->getCTUSize())
#else
      if (pcSPS->getMaxPicHeightInLumaSamples() > pcSPS->getCTUSize())
#endif
      {
        WRITE_CODE( pcSPS->getSubPicHeight(picIdx) - 1,  ceilLog2(( pcSPS->getMaxPicHeightInLumaSamples() +  pcSPS->getCTUSize() - 1)  / pcSPS->getCTUSize()), "subpic_height_minus1[ i ]"   );
      }
#else
      WRITE_CODE( pcSPS->getSubPicCtuTopLeftX(picIdx), std::max(1, ceilLog2((( pcSPS->getMaxPicWidthInLumaSamples()  +  pcSPS->getCTUSize() - 1)  >> floorLog2( pcSPS->getCTUSize())))), "subpic_ctu_top_left_x[ i ]"  );
      WRITE_CODE( pcSPS->getSubPicCtuTopLeftY(picIdx), std::max(1, ceilLog2((( pcSPS->getMaxPicHeightInLumaSamples() +  pcSPS->getCTUSize() - 1)  >> floorLog2( pcSPS->getCTUSize())))), "subpic_ctu_top_left_y[ i ]"  );
      WRITE_CODE( pcSPS->getSubPicWidth(picIdx) - 1,   std::max(1, ceilLog2((( pcSPS->getMaxPicWidthInLumaSamples()  +  pcSPS->getCTUSize() - 1)  >> floorLog2( pcSPS->getCTUSize())))), "subpic_width_minus1[ i ]"    );
      WRITE_CODE( pcSPS->getSubPicHeight(picIdx) - 1,  std::max(1, ceilLog2((( pcSPS->getMaxPicHeightInLumaSamples() +  pcSPS->getCTUSize() - 1)  >> floorLog2( pcSPS->getCTUSize())))), "subpic_height_minus1[ i ]"   );
#endif
      WRITE_FLAG( pcSPS->getSubPicTreatedAsPicFlag(picIdx),  "subpic_treated_as_pic_flag[ i ]" );
      WRITE_FLAG( pcSPS->getLoopFilterAcrossSubpicEnabledFlag(picIdx),  "loop_filter_across_subpic_enabled_flag[ i ]" );
    }
#if JVET_Q0816
    }
#endif

#if JVET_Q0119_CLEANUPS
    WRITE_UVLC(pcSPS->getSubPicIdLen() - 1, "sps_subpic_id_len_minus1");
    WRITE_FLAG(pcSPS->getSubPicIdMappingExplicitlySignalledFlag(), "subpic_id_mapping_explicitly_signalled_flag");
    if (pcSPS->getSubPicIdMappingExplicitlySignalledFlag())
    {
      WRITE_FLAG(pcSPS->getSubPicIdMappingInSpsFlag(), "subpic_id_mapping_in_sps_flag");
      if (pcSPS->getSubPicIdMappingInSpsFlag())
      {
        for (int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++)
        {
          WRITE_CODE(pcSPS->getSubPicId(picIdx), pcSPS->getSubPicIdLen(), "sps_subpic_id[i]");
        }
      }
    }
  }
#else
  }

  WRITE_FLAG( pcSPS->getSubPicIdPresentFlag(), "sps_subpic_id_present_flag");
  if( pcSPS->getSubPicIdPresentFlag() )
  {
    WRITE_FLAG(pcSPS->getSubPicIdSignallingPresentFlag(), "sps_subpic_id_signalling_present_flag");
    if( pcSPS->getSubPicIdSignallingPresentFlag() )
    {
      WRITE_UVLC( pcSPS->getSubPicIdLen( ) - 1, "sps_subpic_id_len_minus1" );
#if JVET_Q0169_SUBPIC_LEN_CONFORM
      CHECK((1 << pcSPS->getSubPicIdLen()) < pcSPS->getNumSubPics(), "sps_subpic_id_len exceeds valid range");
#endif
      for( int picIdx = 0; picIdx < pcSPS->getNumSubPics( ); picIdx++ )
      {
        WRITE_CODE( pcSPS->getSubPicId(picIdx), pcSPS->getSubPicIdLen( ), "sps_subpic_id[i]" );
      }
    }
  }
#endif

  WRITE_UVLC( pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) - 8,                      "bit_depth_minus8" );
#if !JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
  WRITE_UVLC( pcSPS->getMinQpPrimeTsMinus4(CHANNEL_TYPE_LUMA),                      "min_qp_prime_ts_minus4" );
#endif
#if JVET_Q0151_Q0205_ENTRYPOINTS
  WRITE_FLAG( pcSPS->getEntropyCodingSyncEnabledFlag() ? 1 : 0, "sps_entropy_coding_sync_enabled_flag" );
  if (pcSPS->getEntropyCodingSyncEnabledFlag()) 
  {
    WRITE_FLAG( pcSPS->getEntropyCodingSyncEntryPointsPresentFlag() ? 1 : 0, "sps_wpp_entry_point_offsets_present_flag" );
  }
#endif

  WRITE_FLAG( pcSPS->getUseWP() ? 1 : 0, "sps_weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcSPS->getUseWPBiPred() ? 1 : 0, "sps_weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)

  WRITE_CODE(pcSPS->getBitsForPOC()-4, 4, "log2_max_pic_order_cnt_lsb_minus4");

#if JVET_P0116_POC_MSB
  WRITE_FLAG(pcSPS->getPocMsbFlag() ? 1 : 0, "sps_poc_msb_flag");
  if (pcSPS->getPocMsbFlag())
  {
    WRITE_UVLC(pcSPS->getPocMsbLen() - 1, "poc_msb_len_minus1");
  }
#endif
#if JVET_Q0400_EXTRA_BITS
  // extra bits are for future extensions, so these are currently hard coded to not being sent
  WRITE_CODE(0, 2, "num_extra_ph_bits_bytes");
  // extra_ph_bits_struct( num_extra_ph_bits_bytes )
  WRITE_CODE(0, 2, "num_extra_sh_bits_bytes");
  // extra_sh_bits_struct( num_extra_sh_bits_bytes )
#endif

#if JVET_P0117_PTL_SCALABILITY
  if (pcSPS->getMaxTLayers() - 1 > 0)
    WRITE_FLAG(pcSPS->getSubLayerDpbParamsFlag(), "sps_sublayer_dpb_params_flag");
  if (pcSPS->getPtlDpbHrdParamsPresentFlag())
  {
    dpb_parameters(pcSPS->getMaxTLayers() - 1, pcSPS->getSubLayerDpbParamsFlag(), pcSPS);
  }
#else
  // KJS: Marakech decision: sub-layers added back
  const bool subLayerOrderingInfoPresentFlag = 1;
  if (pcSPS->getMaxTLayers() > 1)
  {
    WRITE_FLAG(subLayerOrderingInfoPresentFlag,       "sps_sub_layer_ordering_info_present_flag");
  }
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
#endif
  CHECK( pcSPS->getMaxCUWidth() != pcSPS->getMaxCUHeight(),                          "Rectangular CTUs not supported" );
  WRITE_FLAG(pcSPS->getLongTermRefsPresent() ? 1 : 0, "long_term_ref_pics_flag");
  WRITE_FLAG( pcSPS->getInterLayerPresentFlag() ? 1 : 0, "inter_layer_ref_pics_present_flag" );
  WRITE_FLAG(pcSPS->getIDRRefParamListPresent() ? 1 : 0, "sps_idr_rpl_present_flag" );
  WRITE_FLAG(pcSPS->getRPL1CopyFromRPL0Flag() ? 1 : 0, "rpl1_copy_from_rpl0_flag");

  const RPLList* rplList0 = pcSPS->getRPLList0();
  const RPLList* rplList1 = pcSPS->getRPLList1();

  //Write candidate for List0
  uint32_t numberOfRPL = pcSPS->getNumRPL0();
  WRITE_UVLC(numberOfRPL, "num_ref_pic_lists_in_sps[0]");
  for (int ii = 0; ii < numberOfRPL; ii++)
  {
    const ReferencePictureList* rpl = rplList0->getReferencePictureList(ii);
    xCodeRefPicList( rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC(), !pcSPS->getUseWP() && !pcSPS->getUseWPBiPred() );
  }

  //Write candidate for List1
  if (!pcSPS->getRPL1CopyFromRPL0Flag())
  {
    numberOfRPL = pcSPS->getNumRPL1();
    WRITE_UVLC(numberOfRPL, "num_ref_pic_lists_in_sps[1]");
    for (int ii = 0; ii < numberOfRPL; ii++)
    {
      const ReferencePictureList* rpl = rplList1->getReferencePictureList(ii);
      xCodeRefPicList( rpl, pcSPS->getLongTermRefsPresent(), pcSPS->getBitsForPOC(), !pcSPS->getUseWP() && !pcSPS->getUseWPBiPred() );
    }
  }
  if( pcSPS->getChromaFormatIdc() != CHROMA_400 ) 
  {
    WRITE_FLAG(pcSPS->getUseDualITree(), "qtbtt_dual_tree_intra_flag");
  }
  WRITE_UVLC(pcSPS->getLog2MinCodingBlockSize() - 2, "log2_min_luma_coding_block_size_minus2");
  WRITE_FLAG(pcSPS->getSplitConsOverrideEnabledFlag(), "partition_constraints_override_enabled_flag");
  WRITE_UVLC(floorLog2(pcSPS->getMinQTSize(I_SLICE)) - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
#if !JVET_Q0481_PARTITION_CONSTRAINTS_ORDER
  WRITE_UVLC(floorLog2(pcSPS->getMinQTSize(B_SLICE)) - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_inter_slice");
  WRITE_UVLC(pcSPS->getMaxMTTHierarchyDepth(), "sps_max_mtt_hierarchy_depth_inter_slice");
#endif
  WRITE_UVLC(pcSPS->getMaxMTTHierarchyDepthI(), "sps_max_mtt_hierarchy_depth_intra_slice_luma");
  if (pcSPS->getMaxMTTHierarchyDepthI() != 0)
  {
    WRITE_UVLC(floorLog2(pcSPS->getMaxBTSizeI()) - floorLog2(pcSPS->getMinQTSize(I_SLICE)), "sps_log2_diff_max_bt_min_qt_intra_slice_luma");
    WRITE_UVLC(floorLog2(pcSPS->getMaxTTSizeI()) - floorLog2(pcSPS->getMinQTSize(I_SLICE)), "sps_log2_diff_max_tt_min_qt_intra_slice_luma");
  }
#if JVET_Q0481_PARTITION_CONSTRAINTS_ORDER
  WRITE_UVLC(floorLog2(pcSPS->getMinQTSize(B_SLICE)) - pcSPS->getLog2MinCodingBlockSize(), "sps_log2_diff_min_qt_min_cb_inter_slice");
  WRITE_UVLC(pcSPS->getMaxMTTHierarchyDepth(), "sps_max_mtt_hierarchy_depth_inter_slice");
#endif
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

  WRITE_FLAG( (pcSPS->getLog2MaxTbSize() - 5) ? 1 : 0,                       "sps_max_luma_transform_size_64_flag" );

#if JVET_Q0147_JCCR_SIGNALLING
#if JVET_Q0265
  if (chromaArrayType != CHROMA_400)
#else
  if (pcSPS->getChromaFormatIdc() != CHROMA_400)
#endif
  {
    WRITE_FLAG(pcSPS->getJointCbCrEnabledFlag(), "sps_joint_cbcr_enabled_flag");
#else
  WRITE_FLAG(pcSPS->getJointCbCrEnabledFlag(), "sps_joint_cbcr_enabled_flag");
  if (pcSPS->getChromaFormatIdc() != CHROMA_400)
  {
#endif
    const ChromaQpMappingTable& chromaQpMappingTable = pcSPS->getChromaQpMappingTable();
    WRITE_FLAG(chromaQpMappingTable.getSameCQPTableForAllChromaFlag(), "same_qp_table_for_chroma");
    int numQpTables = chromaQpMappingTable.getSameCQPTableForAllChromaFlag() ? 1 : (pcSPS->getJointCbCrEnabledFlag() ? 3 : 2);
    CHECK(numQpTables != chromaQpMappingTable.getNumQpTables(), " numQpTables does not match at encoder side ");
    for (int i = 0; i < numQpTables; i++)
    {
      WRITE_SVLC(chromaQpMappingTable.getQpTableStartMinus26(i), "qp_table_starts_minus26");
      WRITE_UVLC(chromaQpMappingTable.getNumPtsInCQPTableMinus1(i), "num_points_in_qp_table_minus1");

      for (int j = 0; j <= chromaQpMappingTable.getNumPtsInCQPTableMinus1(i); j++)
      {
        WRITE_UVLC(chromaQpMappingTable.getDeltaQpInValMinus1(i,j),  "delta_qp_in_val_minus1");
        WRITE_UVLC(chromaQpMappingTable.getDeltaQpOutVal(i, j) ^ chromaQpMappingTable.getDeltaQpInValMinus1(i, j),
                   "delta_qp_diff_val");
      }
    }
  }

  WRITE_FLAG( pcSPS->getSAOEnabledFlag(),                                            "sps_sao_enabled_flag");
  WRITE_FLAG( pcSPS->getALFEnabledFlag(),                                            "sps_alf_enabled_flag" );
#if JVET_Q0795_CCALF
  if (pcSPS->getALFEnabledFlag() && pcSPS->getChromaFormatIdc() != CHROMA_400)
  {
    WRITE_FLAG( pcSPS->getCCALFEnabledFlag(),                                            "sps_ccalf_enabled_flag" );
  }
#endif

  WRITE_FLAG(pcSPS->getTransformSkipEnabledFlag() ? 1 : 0, "sps_transform_skip_enabled_flag");
  if (pcSPS->getTransformSkipEnabledFlag())
  {
#if JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
    WRITE_UVLC(pcSPS->getLog2MaxTransformSkipBlockSize() - 2, "log2_transform_skip_max_size_minus2");
#endif
#if JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
    WRITE_FLAG(pcSPS->getBDPCMEnabledFlag() ? 1 : 0, "sps_bdpcm_enabled_flag");
#else
      WRITE_FLAG(pcSPS->getBDPCMEnabled() ? 1 : 0, "sps_bdpcm_enabled_flag");
#if JVET_Q0110_Q0785_CHROMA_BDPCM_420
      if( pcSPS->getBDPCMEnabled() )
#else
      if (pcSPS->getBDPCMEnabled() && pcSPS->getChromaFormatIdc() == CHROMA_444)
#endif
      {
          WRITE_FLAG(pcSPS->getBDPCMEnabled() == BDPCM_LUMACHROMA ? 1 : 0, "sps_bdpcm_enabled_chroma_flag");
      }
#if !JVET_Q0110_Q0785_CHROMA_BDPCM_420
      else 
      {
        CHECK(pcSPS->getBDPCMEnabled() == BDPCM_LUMACHROMA, "BDPCM for chroma can be used for 444 only.")
      }
#endif
#endif
  }
  else
  {
#if JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
    CHECK(pcSPS->getBDPCMEnabledFlag(), "BDPCM cannot be used when transform skip is disabled");
#else
    CHECK(pcSPS->getBDPCMEnabled()!=0, "BDPCM cannot be used when transform skip is disabled");
#endif
  }

  WRITE_FLAG( pcSPS->getWrapAroundEnabledFlag() ? 1 : 0,                              "sps_ref_wraparound_enabled_flag" );
  if( pcSPS->getWrapAroundEnabledFlag() )
  {
#if JVET_Q0416_WRAPAROUND_OFFSET
    WRITE_UVLC((pcSPS->getWrapAroundOffset() / (1 << pcSPS->getLog2MinCodingBlockSize())) - 2 - pcSPS->getCTUSize()/(1<<pcSPS->getLog2MinCodingBlockSize()), "sps_ref_wraparound_offset");
#else
    WRITE_UVLC( (pcSPS->getWrapAroundOffset()/(1 <<  pcSPS->getLog2MinCodingBlockSize()))-1,  "sps_ref_wraparound_offset_minus1" );
#endif
  }

  WRITE_FLAG( pcSPS->getSPSTemporalMVPEnabledFlag()  ? 1 : 0,                        "sps_temporal_mvp_enabled_flag" );

  if ( pcSPS->getSPSTemporalMVPEnabledFlag() )
  {
    WRITE_FLAG( pcSPS->getSBTMVPEnabledFlag() ? 1 : 0,                               "sps_sbtmvp_enabled_flag");
  }

  WRITE_FLAG( pcSPS->getAMVREnabledFlag() ? 1 : 0,                                   "sps_amvr_enabled_flag" );

  WRITE_FLAG( pcSPS->getBDOFEnabledFlag() ? 1 : 0,                                   "sps_bdof_enabled_flag" );
  if (pcSPS->getBDOFEnabledFlag())
  {
    WRITE_FLAG(pcSPS->getBdofControlPresentFlag() ? 1 : 0,                           "sps_bdof_pic_present_flag");
  }
  WRITE_FLAG( pcSPS->getUseSMVD() ? 1 : 0,                                            "sps_smvd_enabled_flag" );
  WRITE_FLAG( pcSPS->getUseDMVR() ? 1 : 0,                                            "sps_dmvr_enabled_flag" );
  if (pcSPS->getUseDMVR())
  {
    WRITE_FLAG(pcSPS->getDmvrControlPresentFlag() ? 1 : 0,                            "sps_dmvr_pic_present_flag");
  }
  WRITE_FLAG(pcSPS->getUseMMVD() ? 1 : 0,                                             "sps_mmvd_enabled_flag");
  WRITE_FLAG( pcSPS->getUseISP() ? 1 : 0,                                             "sps_isp_enabled_flag");
  WRITE_FLAG( pcSPS->getUseMRL() ? 1 : 0,                                             "sps_mrl_enabled_flag");
  WRITE_FLAG( pcSPS->getUseMIP() ? 1 : 0,                                             "sps_mip_enabled_flag");
  if( pcSPS->getChromaFormatIdc() != CHROMA_400) 
  {
    WRITE_FLAG( pcSPS->getUseLMChroma() ? 1 : 0,                                      "sps_cclm_enabled_flag");
  }
  if( pcSPS->getChromaFormatIdc() == CHROMA_420 )
  {
    WRITE_FLAG( pcSPS->getHorCollocatedChromaFlag() ? 1 : 0, "sps_chroma_horizontal_collocated_flag" );
    WRITE_FLAG( pcSPS->getVerCollocatedChromaFlag() ? 1 : 0, "sps_chroma_vertical_collocated_flag" );
  }

  WRITE_FLAG( pcSPS->getUseMTS() ? 1 : 0,                                                      "sps_mts_enabled_flag" );
  if ( pcSPS->getUseMTS() )
  {
    WRITE_FLAG( pcSPS->getUseIntraMTS() ? 1 : 0,                                               "sps_explicit_mts_intra_enabled_flag" );
    WRITE_FLAG( pcSPS->getUseInterMTS() ? 1 : 0,                                               "sps_explicit_mts_inter_enabled_flag" );
  }
  WRITE_FLAG( pcSPS->getUseSBT() ? 1 : 0,                                                      "sps_sbt_enabled_flag");
  WRITE_FLAG( pcSPS->getUseAffine() ? 1 : 0,                                                   "sps_affine_enabled_flag" );
  if ( pcSPS->getUseAffine() )
  {
    WRITE_FLAG( pcSPS->getUseAffineType() ? 1 : 0,                                             "sps_affine_type_flag" );
#if JVET_Q0444_AMVR_SIGNALLING
    if (pcSPS->getAMVREnabledFlag())
    {
#endif
      WRITE_FLAG( pcSPS->getAffineAmvrEnabledFlag() ? 1 : 0,                                     "sps_affine_amvr_enabled_flag" );
#if JVET_Q0444_AMVR_SIGNALLING
    }
#endif
    WRITE_FLAG( pcSPS->getUsePROF() ? 1 : 0,                                                   "sps_affine_prof_enabled_flag" );
    if (pcSPS->getUsePROF())
    {
      WRITE_FLAG(pcSPS->getProfControlPresentFlag() ? 1 : 0,                                   "sps_prof_pic_present_flag" );
    }
  }
#if !JVET_Q0820_ACT
  if (pcSPS->getChromaFormatIdc() == CHROMA_444)
  {
    WRITE_FLAG(pcSPS->getUseColorTrans() ? 1 : 0, "sps_act_enabled_flag");
  }
#endif
#if JVET_Q0504_PLT_NON444
  WRITE_FLAG(pcSPS->getPLTMode() ? 1 : 0,                                                    "sps_palette_enabled_flag" );
#else
  if (pcSPS->getChromaFormatIdc() == CHROMA_444)
  {
    WRITE_FLAG(pcSPS->getPLTMode() ? 1 : 0,                                                    "sps_palette_enabled_flag" );
  }
#endif
#if JVET_Q0820_ACT
  if (pcSPS->getChromaFormatIdc() == CHROMA_444 && pcSPS->getLog2MaxTbSize() != 6)
  {
    WRITE_FLAG(pcSPS->getUseColorTrans() ? 1 : 0, "sps_act_enabled_flag");
  }
#endif
#if JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
  if (pcSPS->getTransformSkipEnabledFlag() || pcSPS->getPLTMode())
  {
    WRITE_UVLC(pcSPS->getMinQpPrimeTsMinus4(CHANNEL_TYPE_LUMA),                                "min_qp_prime_ts_minus4");
  }
#endif
  WRITE_FLAG( pcSPS->getUseBcw() ? 1 : 0,                                                      "sps_bcw_enabled_flag" );
  WRITE_FLAG(pcSPS->getIBCFlag() ? 1 : 0,                                                      "sps_ibc_enabled_flag");

  // KJS: sps_ciip_enabled_flag
  WRITE_FLAG( pcSPS->getUseCiip() ? 1 : 0,                                                  "sps_ciip_enabled_flag" );

  if ( pcSPS->getUseMMVD() )
  {
    WRITE_FLAG( pcSPS->getFpelMmvdEnabledFlag() ? 1 : 0,                            "sps_fpel_mmvd_enabled_flag" );
  }

#if !JVET_Q0806
  WRITE_FLAG( pcSPS->getUseTriangle() ? 1: 0,                                                  "sps_triangle_enabled_flag" );
#else
  WRITE_FLAG( pcSPS->getUseGeo() ? 1: 0,                                                       "sps_gpm_enabled_flag" );
#endif

  WRITE_FLAG(pcSPS->getUseLmcs() ? 1 : 0, "sps_lmcs_enable_flag");
  WRITE_FLAG( pcSPS->getUseLFNST() ? 1 : 0,                                                    "sps_lfnst_enabled_flag" );

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
#if JVET_Q0297_MER
  WRITE_UVLC(pcSPS->getLog2ParallelMergeLevelMinus2(), "log2_parallel_merge_level_minus2");
#endif

  // KJS: reference picture sets to be replaced


  // KJS: remove scaling lists?
  WRITE_FLAG( pcSPS->getScalingListFlag() ? 1 : 0,                                   "sps_scaling_list_enabled_flag" );

  WRITE_FLAG( pcSPS->getLoopFilterAcrossVirtualBoundariesDisabledFlag(), "sps_loop_filter_across_virtual_boundaries_disabled_present_flag" );
  if( pcSPS->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
  {
    WRITE_CODE( pcSPS->getNumVerVirtualBoundaries(), 2, "sps_num_ver_virtual_boundaries");
    for( unsigned i = 0; i < pcSPS->getNumVerVirtualBoundaries(); i++ )
    {
      WRITE_CODE((pcSPS->getVirtualBoundariesPosX(i)>>3), 13, "sps_virtual_boundaries_pos_x");
    }
    WRITE_CODE(pcSPS->getNumHorVirtualBoundaries(), 2, "sps_num_hor_virtual_boundaries");
    for( unsigned i = 0; i < pcSPS->getNumHorVirtualBoundaries(); i++ )
    {
      WRITE_CODE((pcSPS->getVirtualBoundariesPosY(i)>>3), 13, "sps_virtual_boundaries_pos_y");
    }
  }
#if JVET_P0117_PTL_SCALABILITY
  if (pcSPS->getPtlDpbHrdParamsPresentFlag())
  {
#endif
  const TimingInfo *timingInfo = pcSPS->getTimingInfo();
  WRITE_FLAG(pcSPS->getHrdParametersPresentFlag(),          "general_hrd_parameters_present_flag");
    if( pcSPS->getHrdParametersPresentFlag() )
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "time_scale");
    WRITE_FLAG(pcSPS->getSubLayerParametersPresentFlag(), "sub_layer_cpb_parameters_present_flag");
    if (pcSPS->getSubLayerParametersPresentFlag())
    {
      codeHrdParameters(pcSPS->getHrdParameters(), 0, pcSPS->getMaxTLayers() - 1);
    }
    else
    {
      codeHrdParameters(pcSPS->getHrdParameters(), pcSPS->getMaxTLayers() - 1, pcSPS->getMaxTLayers() - 1);
    }
  }
#if JVET_P0117_PTL_SCALABILITY
  }
#endif

#if JVET_Q0042_VUI
  WRITE_FLAG(pcSPS->getFieldSeqFlag(),                          "field_seq_flag");
#endif
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
#if JVET_Q0117_PARAMETER_SETS_CLEANUP
void HLSWriter::codeDCI(const DCI* dci)
{
#if ENABLE_TRACING
  xTraceDPSHeader();
#endif
  WRITE_CODE(dci->getMaxSubLayersMinus1(), 3, "dci_max_sub_layers_minus1");
  WRITE_CODE(0, 1, "dci_reserved_zero_bit");
  uint32_t numPTLs = (uint32_t)dci->getNumPTLs();
  CHECK(numPTLs < 1, "At least one PTL must be available in DCI");

  WRITE_CODE(numPTLs - 1, 4, "dci_num_ptls_minus1");

  for (int i = 0; i < numPTLs; i++)
  {
    ProfileTierLevel ptl = dci->getProfileTierLevel(i);
#if JVET_Q0786_PTL_only
    codeProfileTierLevel(&ptl, true, 0);
#else
    codeProfileTierLevel(&ptl, 0);
#endif

  }
  WRITE_FLAG(0, "dci_extension_flag");
  xWriteRbspTrailingBits();
}
#else
void HLSWriter::codeDPS( const DPS* dps )
{
#if ENABLE_TRACING
  xTraceDPSHeader();
#endif
  WRITE_CODE( dps->getDecodingParameterSetId(),     4,        "dps_decoding_parameter_set_id" );
  WRITE_CODE( dps->getMaxSubLayersMinus1(),         3,        "dps_max_sub_layers_minus1" );
  WRITE_CODE( 0,                                    5,         "dps_reserved_zero_5bits" );
  uint32_t numPTLs = (uint32_t) dps->getNumPTLs();
  CHECK (numPTLs<1, "At least one PTL must be available in DPS");

  WRITE_CODE( numPTLs - 1,                          4,         "dps_num_ptls_minus1" );

  for (int i=0; i< numPTLs; i++)
  {
    ProfileTierLevel ptl = dps->getProfileTierLevel(i);
#if JVET_Q0786_PTL_only
    codeProfileTierLevel( &ptl, true, 0 );
#else
    codeProfileTierLevel( &ptl, dps->getMaxSubLayersMinus1() );
#endif
  }
  WRITE_FLAG( 0,                                              "dps_extension_flag" );
  xWriteRbspTrailingBits();
}
#endif
void HLSWriter::codeVPS(const VPS* pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  WRITE_CODE(pcVPS->getVPSId(), 4, "vps_video_parameter_set_id");
  WRITE_CODE(pcVPS->getMaxLayers() - 1, 6, "vps_max_layers_minus1");
  WRITE_CODE(pcVPS->getMaxSubLayers() - 1, 3, "vps_max_sublayers_minus1");
  if (pcVPS->getMaxLayers() > 1 && pcVPS->getMaxSubLayers() > 1) 
  {
    WRITE_FLAG(pcVPS->getAllLayersSameNumSublayersFlag(), "vps_all_layers_same_num_sublayers_flag");
  }
  if (pcVPS->getMaxLayers() > 1)
  {
    WRITE_FLAG(pcVPS->getAllIndependentLayersFlag(), "vps_all_independent_layers_flag");
  }
  for (uint32_t i = 0; i < pcVPS->getMaxLayers(); i++)
  {
    WRITE_CODE(pcVPS->getLayerId(i), 6, "vps_layer_id");
    if (i > 0 && !pcVPS->getAllIndependentLayersFlag())
    {
      WRITE_FLAG(pcVPS->getIndependentLayerFlag(i), "vps_independent_layer_flag");
      if (!pcVPS->getIndependentLayerFlag(i))
      {
        for (int j = 0; j < i; j++)
        {
          WRITE_FLAG(pcVPS->getDirectRefLayerFlag(i, j), "vps_direct_dependency_flag");
        }
      }
    }
  }
  if( pcVPS->getMaxLayers() > 1 )
  {
    if (pcVPS->getAllIndependentLayersFlag()) 
    {
      WRITE_FLAG(pcVPS->getEachLayerIsAnOlsFlag(), "vps_each_layer_is_an_ols_flag");
    }
    if (!pcVPS->getEachLayerIsAnOlsFlag()) 
    {
      if (!pcVPS->getAllIndependentLayersFlag()) {
        WRITE_CODE(pcVPS->getOlsModeIdc(), 2, "vps_ols_mode_idc");
      }
      if (pcVPS->getOlsModeIdc() == 2)
      {
        WRITE_CODE(pcVPS->getNumOutputLayerSets() - 1, 8, "vps_num_output_layer_sets_minus1");
        for (uint32_t i = 1; i < pcVPS->getNumOutputLayerSets(); i++)
        {
          for (uint32_t j = 0; j < pcVPS->getMaxLayers(); j++)
          {
            WRITE_FLAG(pcVPS->getOlsOutputLayerFlag(i, j), "vps_ols_output_layer_flag");
          }
        }
      }
    }
  }

#if JVET_Q0786_PTL_only
  int totalNumOlss = pcVPS->getTotalNumOLSs();
  WRITE_CODE(pcVPS->getNumPtls() - 1, 8, "vps_num_ptls_minus1");
  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    if(i > 0)
      WRITE_FLAG(pcVPS->getPtPresentFlag(i), "pt_present_flag");           
    if(pcVPS->getMaxSubLayers() > 1 && !pcVPS->getAllLayersSameNumSublayersFlag())
      WRITE_CODE(pcVPS->getPtlMaxTemporalId(i) ,3, "ptl_max_temporal_id");    
  }
  int cnt = 0;
  while (m_pcBitIf->getNumBitsUntilByteAligned())
  {
    WRITE_FLAG( 0, "rbsp_alignment_zero_bit");
    cnt++;
  }
  CHECK(cnt>=8, "More than '8' alignment bytes written");
  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    codeProfileTierLevel(&pcVPS->getProfileTierLevel(i), pcVPS->getPtPresentFlag(i), pcVPS->getPtlMaxTemporalId(i) - 1);
  }
  for (int i = 0; i < totalNumOlss; i++)
  {
    if(pcVPS->getNumPtls() > 1)
      WRITE_CODE(pcVPS->getOlsPtlIdx(i), 8, "ols_ptl_idx");
  }
#endif

#if JVET_Q0814_DPB
  if( !pcVPS->getAllIndependentLayersFlag() )
  {
    WRITE_UVLC( pcVPS->m_numDpbParams, "vps_num_dpb_params" );
  }

  if( pcVPS->m_numDpbParams > 0 && pcVPS->getMaxSubLayers() > 1 )
  {
    WRITE_FLAG( pcVPS->m_sublayerDpbParamsPresentFlag, "vps_sublayer_dpb_params_present_flag" );
  }

  for( int i = 0; i < pcVPS->m_numDpbParams; i++ )
  {
    if( pcVPS->getMaxSubLayers() == 1 )
    {
      CHECK( pcVPS->m_dpbMaxTemporalId[i] != 0, "When vps_max_sublayers_minus1 is equal to 0, the value of dpb_max_temporal_id[ i ] is inferred to be equal to 0" );
    }
    else
    {
      if( pcVPS->getAllLayersSameNumSublayersFlag() )
      {
        CHECK( pcVPS->m_dpbMaxTemporalId[i] != pcVPS->getMaxSubLayers() - 1, "When vps_max_sublayers_minus1 is greater than 0 and vps_all_layers_same_num_sublayers_flag is equal to 1, the value of dpb_max_temporal_id[ i ] is inferred to be equal to vps_max_sublayers_minus1" );
      }
      else
      {
        WRITE_CODE( pcVPS->m_dpbMaxTemporalId[i], 3, "dpb_max_temporal_id[i]" );
      }
    }

    for( int j = ( pcVPS->m_sublayerDpbParamsPresentFlag ? 0 : pcVPS->m_dpbMaxTemporalId[i] ); j <= pcVPS->m_dpbMaxTemporalId[i]; j++ )
    {
      WRITE_UVLC( pcVPS->m_dpbParameters[i].m_maxDecPicBuffering[j], "max_dec_pic_buffering_minus1[i]" );
      WRITE_UVLC( pcVPS->m_dpbParameters[i].m_numReorderPics[j], "max_num_reorder_pics[i]" );
      WRITE_UVLC( pcVPS->m_dpbParameters[i].m_maxLatencyIncreasePlus1[j], "max_latency_increase_plus1[i]" );
    }
  }

  for( int i = 0; i < pcVPS->getTotalNumOLSs(); i++ )
  {
    if( pcVPS->m_numLayersInOls[i] > 1 )
    {
      WRITE_UVLC( pcVPS->getOlsDpbPicSize( i ).width, "ols_dpb_pic_width[i]" );
      WRITE_UVLC( pcVPS->getOlsDpbPicSize( i ).height, "ols_dpb_pic_height[i]" );
      if( pcVPS->m_numDpbParams > 1 )
      {
        WRITE_UVLC( pcVPS->getOlsDpbParamsIdx( i ), "ols_dpb_params_idx[i]" );
      }
    }
  }
#endif

  WRITE_FLAG(0, "vps_extension_flag");

  //future extensions here..
  xWriteRbspTrailingBits();
}

#if JVET_Q0775_PH_IN_SH
void HLSWriter::codePictureHeader( PicHeader* picHeader, bool writeRbspTrailingBits )
#else
void HLSWriter::codePictureHeader( PicHeader* picHeader )
#endif
{
  const PPS*  pps = NULL;
  const SPS*  sps = NULL;

#if ENABLE_TRACING
  xTracePictureHeader ();
#endif

  CodingStructure& cs = *picHeader->getPic()->cs;
#if JVET_Q0819_PH_CHANGES
  WRITE_FLAG(picHeader->getGdrOrIrapPicFlag(), "gdr_or_irap_pic_flag");
  if (picHeader->getGdrOrIrapPicFlag())
  {
    WRITE_FLAG(picHeader->getGdrPicFlag(), "gdr_pic_flag");
  }
  // Q0781, two-flags
  WRITE_FLAG(picHeader->getPicInterSliceAllowedFlag(), "pic_inter_slice_allowed_flag");
  if (picHeader->getPicInterSliceAllowedFlag())
  {
    WRITE_FLAG(picHeader->getPicIntraSliceAllowedFlag(), "pic_intra_slice_allowed_flag");
  }
#endif
  WRITE_FLAG(picHeader->getNonReferencePictureFlag(), "non_reference_picture_flag");
#if !JVET_Q0819_PH_CHANGES
  WRITE_FLAG(picHeader->getGdrPicFlag(), "gdr_pic_flag");
#endif
#if JVET_Q0819_PH_CHANGES
  // parameter sets
  WRITE_UVLC(picHeader->getPPSId(), "ph_pic_parameter_set_id");
  pps = cs.slice->getPPS();
  CHECK(pps == 0, "Invalid PPS");
  sps = cs.slice->getSPS();
  CHECK(sps == 0, "Invalid SPS");
  int pocBits = cs.slice->getSPS()->getBitsForPOC();
  int pocMask = (1 << pocBits) - 1;
  WRITE_CODE(cs.slice->getPOC() & pocMask, pocBits, "ph_pic_order_cnt_lsb");
  if (picHeader->getGdrOrIrapPicFlag())
  {
    WRITE_FLAG(picHeader->getNoOutputOfPriorPicsFlag(), "no_output_of_prior_pics_flag");
  }
#else
  WRITE_FLAG(picHeader->getNoOutputOfPriorPicsFlag(), "no_output_of_prior_pics_flag");
#endif
  if( picHeader->getGdrPicFlag() ) 
  {
    WRITE_UVLC(picHeader->getRecoveryPocCnt(), "recovery_poc_cnt");
  }
  else 
  {
    picHeader->setRecoveryPocCnt( 0 );
  }
#if JVET_Q0400_EXTRA_BITS
  // PH extra bits are not written in the reference encoder
  // as these bits are reserved for future extensions
  // for( i = 0; i < NumExtraPhBits; i++ )
  //    ph_extra_bit[ i ]
#endif
            
#if JVET_P0116_POC_MSB
  if (sps->getPocMsbFlag())
  {
    WRITE_FLAG(picHeader->getPocMsbPresentFlag(), "ph_poc_msb_present_flag");
    if (picHeader->getPocMsbPresentFlag())
    {
      WRITE_CODE(picHeader->getPocMsbVal(), sps->getPocMsbLen(), "poc_msb_val");
    }
  }
#endif

#if !JVET_Q0819_PH_CHANGES
  // parameter sets
  WRITE_UVLC(picHeader->getPPSId(), "ph_pic_parameter_set_id");
  pps = cs.slice->getPPS();
  CHECK(pps==0, "Invalid PPS");  
  sps = cs.slice->getSPS();
  CHECK(sps==0, "Invalid SPS");
#endif

#if JVET_Q0819_PH_CHANGES
   // alf enable flags and aps IDs
  if( sps->getALFEnabledFlag() )
  {
#if JVET_Q0819_PH_CHANGES
    if (pps->getAlfInfoInPhFlag())
#else
    WRITE_FLAG(picHeader->getAlfEnabledPresentFlag(), "pic_alf_enabled_present_flag");
    if (picHeader->getAlfEnabledPresentFlag())
#endif
    {
      WRITE_FLAG(picHeader->getAlfEnabledFlag(COMPONENT_Y), "pic_alf_enabled_flag");
      if (picHeader->getAlfEnabledFlag(COMPONENT_Y))
      {
        WRITE_CODE(picHeader->getNumAlfAps(), 3, "pic_num_alf_aps_ids_luma");
        const std::vector<int>&   apsId = picHeader->getAlfAPSs();
        for (int i = 0; i < picHeader->getNumAlfAps(); i++)
        {
          WRITE_CODE(apsId[i], 3, "pic_alf_aps_id_luma");
        }

        const int alfChromaIdc = picHeader->getAlfEnabledFlag(COMPONENT_Cb) + picHeader->getAlfEnabledFlag(COMPONENT_Cr) * 2 ;
        if (sps->getChromaFormatIdc() != CHROMA_400)
        {
          WRITE_CODE(alfChromaIdc, 2, "pic_alf_chroma_idc");
        }
        if (alfChromaIdc)
        {
          WRITE_CODE(picHeader->getAlfApsIdChroma(), 3, "pic_alf_aps_id_chroma");
        }
#if JVET_Q0795_CCALF
        if (sps->getCCALFEnabledFlag())
        {
          WRITE_FLAG(picHeader->getCcAlfEnabledFlag(COMPONENT_Cb), "ph_cc_alf_cb_enabled_flag");
          if (picHeader->getCcAlfEnabledFlag(COMPONENT_Cb))
          {
            WRITE_CODE(picHeader->getCcAlfCbApsId(), 3, "ph_cc_alf_cb_aps_id");
          }
          WRITE_FLAG(picHeader->getCcAlfEnabledFlag(COMPONENT_Cr), "ph_cc_alf_cr_enabled_flag");
          if (picHeader->getCcAlfEnabledFlag(COMPONENT_Cr))
          {
            WRITE_CODE(picHeader->getCcAlfCrApsId(), 3, "ph_cc_alf_cr_aps_id");
          }
        }
#endif
      }
    }
    else
    {
      picHeader->setAlfEnabledFlag(COMPONENT_Y,  true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cb, true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cr, true);
#if JVET_Q0795_CCALF
      picHeader->setCcAlfEnabledFlag(COMPONENT_Cb, sps->getCCALFEnabledFlag());
      picHeader->setCcAlfEnabledFlag(COMPONENT_Cr, sps->getCCALFEnabledFlag());
#endif
    }
  }
  else
  {
    picHeader->setAlfEnabledFlag(COMPONENT_Y,  false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cb, false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cr, false);
#if JVET_Q0795_CCALF
    picHeader->setCcAlfEnabledFlag(COMPONENT_Cb, false);
    picHeader->setCcAlfEnabledFlag(COMPONENT_Cr, false);
#endif
  }

  // luma mapping / chroma scaling controls
  if (sps->getUseLmcs())
  {
    WRITE_FLAG(picHeader->getLmcsEnabledFlag(), "pic_lmcs_enabled_flag");
    if (picHeader->getLmcsEnabledFlag())
    {
      WRITE_CODE(picHeader->getLmcsAPSId(), 2, "pic_lmcs_aps_id");
      if (sps->getChromaFormatIdc() != CHROMA_400)
      {
        WRITE_FLAG(picHeader->getLmcsChromaResidualScaleFlag(), "pic_chroma_residual_scale_flag");
      }
      else
      {
        picHeader->setLmcsChromaResidualScaleFlag(false);
      }
    }
  }
  else
  {
    picHeader->setLmcsEnabledFlag(false);
    picHeader->setLmcsChromaResidualScaleFlag(false);
  }

  // quantization scaling lists
  if( sps->getScalingListFlag() )
  {
    WRITE_FLAG( picHeader->getScalingListPresentFlag(), "pic_scaling_list_present_flag" );
    if( picHeader->getScalingListPresentFlag() )
    {
      WRITE_CODE( picHeader->getScalingListAPSId(), 3, "pic_scaling_list_aps_id" );
    }
  }
  else
  {
    picHeader->setScalingListPresentFlag( false );
  }

#endif
  
#if !JVET_Q0119_CLEANUPS
  // sub-picture IDs
  if( sps->getSubPicIdPresentFlag() ) 
  {
    if( sps->getSubPicIdSignallingPresentFlag() ) 
    {
      for( int picIdx = 0; picIdx < sps->getNumSubPics( ); picIdx++ )
      {
        picHeader->setSubPicId( picIdx, sps->getSubPicId( picIdx ) );
      }
    }
    else 
    {
      WRITE_FLAG(picHeader->getSubPicIdSignallingPresentFlag(), "ph_subpic_id_signalling_present_flag");
      if( picHeader->getSubPicIdSignallingPresentFlag() )
      {
        WRITE_UVLC( picHeader->getSubPicIdLen() - 1, "ph_subpic_id_len_minus1" );
#if JVET_Q0169_SUBPIC_LEN_CONFORM
        CHECK((1 << picHeader->getSubPicIdLen()) < sps->getNumSubPics(), "ph_subpic_id_len exceeds valid range");
#endif
        for( int picIdx = 0; picIdx < sps->getNumSubPics( ); picIdx++ )
        {
          WRITE_CODE(picHeader->getSubPicId(picIdx), picHeader->getSubPicIdLen( ), "ph_subpic_id[i]" );
        }
      }
      else 
      {
        for( int picIdx = 0; picIdx < pps->getNumSubPics( ); picIdx++ )
        {
          picHeader->setSubPicId( picIdx, pps->getSubPicId( picIdx ) );
        }
      }
    }
  }
  else 
  {
    for( int picIdx = 0; picIdx < sps->getNumSubPics( ); picIdx++ )
    {
      picHeader->setSubPicId( picIdx, picIdx );
    }
  }
#endif

  // virtual boundaries
  if( !sps->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
  {
    WRITE_FLAG( picHeader->getLoopFilterAcrossVirtualBoundariesDisabledFlag(), "ph_loop_filter_across_virtual_boundaries_disabled_present_flag" );
    if( picHeader->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
    {
      WRITE_CODE(picHeader->getNumVerVirtualBoundaries(), 2, "ph_num_ver_virtual_boundaries");
      for( unsigned i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++ )
      {
        WRITE_CODE(picHeader->getVirtualBoundariesPosX(i) >> 3, 13, "ph_virtual_boundaries_pos_x");
      }
      WRITE_CODE(picHeader->getNumHorVirtualBoundaries(), 2, "ph_num_hor_virtual_boundaries");
      for( unsigned i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++ )
      {
        WRITE_CODE(picHeader->getVirtualBoundariesPosY(i)>>3, 13, "ph_virtual_boundaries_pos_y");
      }
    }
    else
    {
      picHeader->setLoopFilterAcrossVirtualBoundariesDisabledFlag( 0 );
      picHeader->setNumVerVirtualBoundaries( 0 );
      picHeader->setNumHorVirtualBoundaries( 0 );
    }
  }
  else
  {
    picHeader->setLoopFilterAcrossVirtualBoundariesDisabledFlag( sps->getLoopFilterAcrossVirtualBoundariesDisabledFlag() );
    picHeader->setNumVerVirtualBoundaries( sps->getNumVerVirtualBoundaries() );
    picHeader->setNumHorVirtualBoundaries( sps->getNumHorVirtualBoundaries() );
    for( unsigned i = 0; i < 3; i++ ) 
    {
      picHeader->setVirtualBoundariesPosX( sps->getVirtualBoundariesPosX(i), i );
      picHeader->setVirtualBoundariesPosY( sps->getVirtualBoundariesPosY(i), i );
    }
  }
  
#if !JVET_Q0155_COLOUR_ID
  // 4:4:4 colour plane ID
  if( sps->getSeparateColourPlaneFlag() )
  {
    WRITE_CODE( picHeader->getColourPlaneId(), 2, "colour_plane_id" );
  }
  else 
  {
    picHeader->setColourPlaneId( 0 );
  }
#endif
  
  // picture output flag
  if( pps->getOutputFlagPresentFlag() )
  {
    WRITE_FLAG( picHeader->getPicOutputFlag(), "pic_output_flag" );
  }
  else 
  {
    picHeader->setPicOutputFlag(true);
  }

  // reference picture lists
#if JVET_Q0819_PH_CHANGES
  if (pps->getRplInfoInPhFlag())
#else
  WRITE_FLAG(picHeader->getPicRplPresentFlag(), "pic_rpl_present_flag"); 
  if( picHeader->getPicRplPresentFlag() )
#endif
  {
    // List0 and List1
    for(int listIdx = 0; listIdx < 2; listIdx++) 
    {                 
#if JVET_Q0482_REMOVE_CONSTANT_PARAMS
      if(sps->getNumRPL(listIdx) > 0 &&
          (listIdx == 0 || (listIdx == 1 && pps->getRpl1IdxPresentFlag())))
#else
      if(sps->getNumRPL(listIdx) > 0 && !pps->getPPSRefPicListSPSIdc(listIdx) &&
          (listIdx == 0 || (listIdx == 1 && pps->getRpl1IdxPresentFlag())))
#endif
      {
        WRITE_FLAG(picHeader->getRPLIdx(listIdx) != -1 ? 1 : 0, "pic_rpl_sps_flag[i]");
      }
      else if(sps->getNumRPL(listIdx) == 0)
      {
        CHECK(picHeader->getRPLIdx(listIdx) != -1, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }
      else if(listIdx == 1)
      {
        auto rplsSpsFlag0 = picHeader->getRPLIdx(0) != -1 ? 1 : 0;
        auto rplsSpsFlag1 = picHeader->getRPLIdx(1) != -1 ? 1 : 0;
        CHECK(rplsSpsFlag1 != rplsSpsFlag0, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }

      if(picHeader->getRPLIdx(listIdx) != -1)
      {
        if(sps->getNumRPL(listIdx) > 1 &&
            (listIdx == 0 || (listIdx == 1 && pps->getRpl1IdxPresentFlag())))
        {
          int numBits = ceilLog2(sps->getNumRPL( listIdx ));
          WRITE_CODE(picHeader->getRPLIdx(listIdx), numBits, "pic_rpl_idx[i]");
        }
        else if(sps->getNumRPL(listIdx) == 1)
        {
          CHECK(picHeader->getRPLIdx(listIdx) != 0, "RPL1Idx is not signalled but it is not equal to 0");
        }
        else
        {
          CHECK(picHeader->getRPL1idx() != picHeader->getRPL0idx(), "RPL1Idx is not signalled but it is not the same as RPL0Idx");
        }
      }
      // explicit RPL in picture header
      else
      {
        xCodeRefPicList( picHeader->getRPL(listIdx), sps->getLongTermRefsPresent(), sps->getBitsForPOC(), !sps->getUseWP() && !sps->getUseWPBiPred() );
      }

      // POC MSB cycle signalling for LTRP
      if (picHeader->getRPL(listIdx) && picHeader->getRPL(listIdx)->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < picHeader->getRPL(listIdx)->getNumberOfLongtermPictures() + picHeader->getRPL(listIdx)->getNumberOfShorttermPictures(); i++)
        {
          if (picHeader->getRPL(listIdx)->isRefPicLongterm(i))
          {
            if (picHeader->getRPL(listIdx)->getLtrpInSliceHeaderFlag())
            { 
              WRITE_CODE(picHeader->getRPL(listIdx)->getRefPicIdentifier(i), sps->getBitsForPOC(),
                         "pic_poc_lsb_lt[listIdx][rplsIdx][j]");
            }
            WRITE_FLAG(picHeader->getLocalRPL(listIdx)->getDeltaPocMSBPresentFlag(i) ? 1 : 0, "pic_delta_poc_msb_present_flag[i][j]");
            if (picHeader->getLocalRPL(listIdx)->getDeltaPocMSBPresentFlag(i))
            {
              WRITE_UVLC(picHeader->getLocalRPL(listIdx)->getDeltaPocMSBCycleLT(i), "pic_delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }
    }
  }

  // partitioning constraint overrides
  if (sps->getSplitConsOverrideEnabledFlag())
  {
    WRITE_FLAG(picHeader->getSplitConsOverrideFlag(), "partition_constraints_override_flag");
#if JVET_Q0819_PH_CHANGES
  }
  else
  {
    picHeader->setSplitConsOverrideFlag(0);
  }
  // Q0781, two-flags
  if (picHeader->getPicIntraSliceAllowedFlag())
  {
#endif
    if (picHeader->getSplitConsOverrideFlag())
    {
      WRITE_UVLC(floorLog2(picHeader->getMinQTSize(I_SLICE)) - sps->getLog2MinCodingBlockSize(), "pic_log2_diff_min_qt_min_cb_intra_slice_luma");
#if !JVET_Q0819_PH_CHANGES
      WRITE_UVLC(floorLog2(picHeader->getMinQTSize(P_SLICE)) - sps->getLog2MinCodingBlockSize(), "pic_log2_diff_min_qt_min_cb_inter_slice");
      WRITE_UVLC(picHeader->getMaxMTTHierarchyDepth(P_SLICE),  "pic_max_mtt_hierarchy_depth_inter_slice");
#endif
      WRITE_UVLC(picHeader->getMaxMTTHierarchyDepth(I_SLICE), "pic_max_mtt_hierarchy_depth_intra_slice_luma");
      if (picHeader->getMaxMTTHierarchyDepth(I_SLICE) != 0)
      {
        WRITE_UVLC(floorLog2(picHeader->getMaxBTSize(I_SLICE)) - floorLog2(picHeader->getMinQTSize(I_SLICE)), "pic_log2_diff_max_bt_min_qt_intra_slice_luma");
        WRITE_UVLC(floorLog2(picHeader->getMaxTTSize(I_SLICE)) - floorLog2(picHeader->getMinQTSize(I_SLICE)), "pic_log2_diff_max_tt_min_qt_intra_slice_luma");
      }

#if !JVET_Q0819_PH_CHANGES
      if (picHeader->getMaxMTTHierarchyDepth(P_SLICE) != 0)
      {
        WRITE_UVLC(floorLog2(picHeader->getMaxBTSize(P_SLICE)) - floorLog2(picHeader->getMinQTSize(P_SLICE)), "pic_log2_diff_max_bt_min_qt_inter_slice");
        WRITE_UVLC(floorLog2(picHeader->getMaxTTSize(P_SLICE)) - floorLog2(picHeader->getMinQTSize(P_SLICE)), "pic_log2_diff_max_tt_min_qt_inter_slice");
      }
#endif
      if (sps->getUseDualITree())
      {
        WRITE_UVLC(floorLog2(picHeader->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)) - sps->getLog2MinCodingBlockSize(), "pic_log2_diff_min_qt_min_cb_intra_slice_chroma");
        WRITE_UVLC(picHeader->getMaxMTTHierarchyDepth(I_SLICE, CHANNEL_TYPE_CHROMA), "pic_max_mtt_hierarchy_depth_intra_slice_chroma");
        if (picHeader->getMaxMTTHierarchyDepth(I_SLICE, CHANNEL_TYPE_CHROMA) != 0)
        {
          WRITE_UVLC(floorLog2(picHeader->getMaxBTSize(I_SLICE, CHANNEL_TYPE_CHROMA)) - floorLog2(picHeader->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)), "pic_log2_diff_max_bt_min_qt_intra_slice_chroma");
          WRITE_UVLC(floorLog2(picHeader->getMaxTTSize(I_SLICE, CHANNEL_TYPE_CHROMA)) - floorLog2(picHeader->getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA)), "pic_log2_diff_max_tt_min_qt_intra_slice_chroma");
        }
      }
    }
  }
#if !JVET_Q0819_PH_CHANGES
  else 
  {
    picHeader->setSplitConsOverrideFlag(0);
  }

  // inherit constraint values from SPS
  if (!sps->getSplitConsOverrideEnabledFlag() || !picHeader->getSplitConsOverrideFlag()) 
  {
      picHeader->setMinQTSizes(sps->getMinQTSizes());
      picHeader->setMaxMTTHierarchyDepths(sps->getMaxMTTHierarchyDepths());
      picHeader->setMaxBTSizes(sps->getMaxBTSizes());
      picHeader->setMaxTTSizes(sps->getMaxTTSizes());
  }
#endif
#if JVET_Q0819_PH_CHANGES
  if (picHeader->getPicIntraSliceAllowedFlag())
  {
#endif
  // delta quantization and chrom and chroma offset
    if (pps->getUseDQP())
    {
      WRITE_UVLC( picHeader->getCuQpDeltaSubdivIntra(), "pic_cu_qp_delta_subdiv_intra_slice" );
#if !JVET_Q0819_PH_CHANGES
      WRITE_UVLC( picHeader->getCuQpDeltaSubdivInter(), "pic_cu_qp_delta_subdiv_inter_slice" );
#endif
    }
    else 
    {
      picHeader->setCuQpDeltaSubdivIntra( 0 );
#if !JVET_Q0819_PH_CHANGES
      picHeader->setCuQpDeltaSubdivInter( 0 );
#endif
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      WRITE_UVLC( picHeader->getCuChromaQpOffsetSubdivIntra(), "pic_cu_chroma_qp_offset_subdiv_intra_slice" );
#if !JVET_Q0819_PH_CHANGES
      WRITE_UVLC( picHeader->getCuChromaQpOffsetSubdivInter(), "pic_cu_chroma_qp_offset_subdiv_inter_slice" );
#endif
    }
    else 
    {
      picHeader->setCuChromaQpOffsetSubdivIntra( 0 );
#if !JVET_Q0819_PH_CHANGES
      picHeader->setCuChromaQpOffsetSubdivInter( 0 );
#endif
    }
#if JVET_Q0819_PH_CHANGES
  }


  if (picHeader->getPicInterSliceAllowedFlag())
  {
    if (picHeader->getSplitConsOverrideFlag())
    {
      WRITE_UVLC(floorLog2(picHeader->getMinQTSize(P_SLICE)) - sps->getLog2MinCodingBlockSize(), "pic_log2_diff_min_qt_min_cb_inter_slice");
      WRITE_UVLC(picHeader->getMaxMTTHierarchyDepth(P_SLICE), "pic_max_mtt_hierarchy_depth_inter_slice");
      if (picHeader->getMaxMTTHierarchyDepth(P_SLICE) != 0)
      {
        WRITE_UVLC(floorLog2(picHeader->getMaxBTSize(P_SLICE)) - floorLog2(picHeader->getMinQTSize(P_SLICE)), "pic_log2_diff_max_bt_min_qt_inter_slice");
        WRITE_UVLC(floorLog2(picHeader->getMaxTTSize(P_SLICE)) - floorLog2(picHeader->getMinQTSize(P_SLICE)), "pic_log2_diff_max_tt_min_qt_inter_slice");
      }
    }

    // delta quantization and chrom and chroma offset
    if (pps->getUseDQP())
    {
      WRITE_UVLC(picHeader->getCuQpDeltaSubdivInter(), "pic_cu_qp_delta_subdiv_inter_slice");
    }
    else
    {
      picHeader->setCuQpDeltaSubdivInter(0);
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      WRITE_UVLC(picHeader->getCuChromaQpOffsetSubdivInter(), "pic_cu_chroma_qp_offset_subdiv_inter_slice");
    }
    else
    {
      picHeader->setCuChromaQpOffsetSubdivInter(0);
    }
#endif
  // temporal motion vector prediction
    if (sps->getSPSTemporalMVPEnabledFlag())
    {
      WRITE_FLAG( picHeader->getEnableTMVPFlag(), "pic_temporal_mvp_enabled_flag" );
#if JVET_Q0259_COLLOCATED_PIC_IN_PH
      if (picHeader->getEnableTMVPFlag() && pps->getRplInfoInPhFlag())
      {
        WRITE_CODE(picHeader->getPicColFromL0Flag(), 1, "pic_collocated_from_l0_flag");
        if ((picHeader->getPicColFromL0Flag() && picHeader->getRPL(0)->getNumRefEntries() > 1) ||
          (!picHeader->getPicColFromL0Flag() && picHeader->getRPL(1)->getNumRefEntries() > 1))
        {
          WRITE_UVLC(picHeader->getColRefIdx(), "collocated_ref_idx");
        }
      }
#endif
    }
    else
    {
      picHeader->setEnableTMVPFlag(false);
    }

  // mvd L1 zero flag
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
    if (!pps->getPPSMvdL1ZeroIdc())
    {
      WRITE_FLAG(picHeader->getMvdL1ZeroFlag(), "pic_mvd_l1_zero_flag");
    }
    else
    {
      picHeader->setMvdL1ZeroFlag( pps->getPPSMvdL1ZeroIdc() - 1 );
    }
#else
#if !JVET_Q0259_COLLOCATED_PIC_IN_PH
    if (picHeader->getEnableTMVPFlag() && pps->getRplInfoInPhFlag())
    {
      WRITE_CODE(picHeader->getPicColFromL0Flag(), 1, "pic_collocated_from_l0_flag");
    }
#endif
    WRITE_FLAG(picHeader->getMvdL1ZeroFlag(), "pic_mvd_l1_zero_flag");
#endif
   
  // merge candidate list size
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
    if (!pps->getPPSSixMinusMaxNumMergeCandPlus1())
    {
#endif
      CHECK(picHeader->getMaxNumMergeCand() > MRG_MAX_NUM_CANDS, "More merge candidates signalled than supported");
      WRITE_UVLC(MRG_MAX_NUM_CANDS - picHeader->getMaxNumMergeCand(), "pic_six_minus_max_num_merge_cand");
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
    }
    else
    {
      picHeader->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - (pps->getPPSSixMinusMaxNumMergeCandPlus1() - 1));
    }
#endif
  // subblock merge candidate list size
    if ( sps->getUseAffine() )
    {
      CHECK( picHeader->getMaxNumAffineMergeCand() > AFFINE_MRG_MAX_NUM_CANDS, "More affine merge candidates signalled than supported" );
      WRITE_UVLC(AFFINE_MRG_MAX_NUM_CANDS - picHeader->getMaxNumAffineMergeCand(), "pic_five_minus_max_num_subblock_merge_cand");
    }
    else
    {
      picHeader->setMaxNumAffineMergeCand( sps->getSBTMVPEnabledFlag() && picHeader->getEnableTMVPFlag() );
    }

  // full-pel MMVD flag
    if (sps->getFpelMmvdEnabledFlag())
    {
      WRITE_FLAG( picHeader->getDisFracMMVD(), "pic_fpel_mmvd_enabled_flag" );
    }
    else
    {
      picHeader->setDisFracMMVD(false);
    }
  
  // picture level BDOF disable flags
    if (sps->getBdofControlPresentFlag())
    {
      WRITE_FLAG(picHeader->getDisBdofFlag(), "pic_disable_bdof_flag");
    }
    else
    {
      picHeader->setDisBdofFlag(0);
    }

  // picture level DMVR disable flags
    if (sps->getDmvrControlPresentFlag())
    {
      WRITE_FLAG(picHeader->getDisDmvrFlag(), "pic_disable_dmvr_flag");
    }
    else
    {
      picHeader->setDisDmvrFlag(0);
    }

  // picture level PROF disable flags
    if (sps->getProfControlPresentFlag())
    {
      WRITE_FLAG(picHeader->getDisProfFlag(), "pic_disable_prof_flag");
    }
    else
    {
      picHeader->setDisProfFlag(0);
    }

#if JVET_Q0819_PH_CHANGES
    if ((pps->getUseWP() || pps->getWPBiPred()) && pps->getWpInfoInPhFlag())
    {
      xCodePredWeightTable(picHeader, sps);
    }
#endif

#if !JVET_Q0806
  // triangle merge candidate list size
    if (sps->getUseTriangle() && picHeader->getMaxNumMergeCand() >= 2)
    {
      if (!pps->getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1())
      {
        CHECK(picHeader->getMaxNumMergeCand() < picHeader->getMaxNumTriangleCand(), "Incorrrect max number of triangle candidates!");
        WRITE_UVLC(picHeader->getMaxNumMergeCand() - picHeader->getMaxNumTriangleCand(), "pic_max_num_merge_cand_minus_max_num_triangle_cand");
      }
      else
      {
        picHeader->setMaxNumTriangleCand((uint32_t)(picHeader->getMaxNumMergeCand() - (pps->getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1() - 1)));
      }    
    }

#else
  // geometric merge candidate list size
    if (sps->getUseGeo() && picHeader->getMaxNumMergeCand() >= 2)
    {
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
      if (!pps->getPPSMaxNumMergeCandMinusMaxNumGeoCandPlus1())
      {
#endif
        CHECK(picHeader->getMaxNumMergeCand() < picHeader->getMaxNumGeoCand(), "Incorrrect max number of gpm candidates!");
        WRITE_UVLC(picHeader->getMaxNumMergeCand() - picHeader->getMaxNumGeoCand(), "pic_max_num_merge_cand_minus_max_num_gpm_cand");
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
      }
      else
      {
      picHeader->setMaxNumGeoCand((uint32_t)(picHeader->getMaxNumMergeCand() - (pps->getPPSMaxNumMergeCandMinusMaxNumGeoCandPlus1() - 1)));
      }
#endif
    }
#endif
#if JVET_Q0819_PH_CHANGES
  }
  // inherit constraint values from SPS
  if (!sps->getSplitConsOverrideEnabledFlag() || !picHeader->getSplitConsOverrideFlag())
  {
    picHeader->setMinQTSizes(sps->getMinQTSizes());
    picHeader->setMaxMTTHierarchyDepths(sps->getMaxMTTHierarchyDepths());
    picHeader->setMaxBTSizes(sps->getMaxBTSizes());
    picHeader->setMaxTTSizes(sps->getMaxTTSizes());
  }
#endif
  // ibc merge candidate list size
  if (sps->getIBCFlag())
  {
    CHECK( picHeader->getMaxNumIBCMergeCand() > IBC_MRG_MAX_NUM_CANDS, "More IBC merge candidates signalled than supported" );
    WRITE_UVLC(IBC_MRG_MAX_NUM_CANDS - picHeader->getMaxNumIBCMergeCand(), "pic_six_minus_max_num_ibc_merge_cand");
  }

#if JVET_Q0819_PH_CHANGES
  if (pps->getQpDeltaInfoInPhFlag())
  {
    WRITE_SVLC(picHeader->getQpDelta(), "ph_qp_delta");
  }
#endif

  // joint Cb/Cr sign flag
  if (sps->getJointCbCrEnabledFlag())
  {
    WRITE_FLAG( picHeader->getJointCbCrSignFlag(), "pic_joint_cbcr_sign_flag" );
  }
  else
  {
    picHeader->setJointCbCrSignFlag(false);
  }

  // sao enable flags
  if(sps->getSAOEnabledFlag())
  {
#if JVET_Q0819_PH_CHANGES
    if (pps->getSaoInfoInPhFlag())
#else
    WRITE_FLAG(picHeader->getSaoEnabledPresentFlag(), "pic_sao_enabled_present_flag");
    if (picHeader->getSaoEnabledPresentFlag())
#endif
    {    
#if JVET_Q0819_PH_CHANGES
      WRITE_FLAG(picHeader->getSaoEnabledFlag(CHANNEL_TYPE_LUMA), "ph_sao_luma_enabled_flag");
#else
      WRITE_FLAG(picHeader->getSaoEnabledFlag(CHANNEL_TYPE_LUMA), "slice_sao_luma_flag");  
#endif
      if (sps->getChromaFormatIdc() != CHROMA_400)
      {
#if JVET_Q0819_PH_CHANGES
        WRITE_FLAG(picHeader->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA), "ph_sao_chroma_enabled_flag");
#else
        WRITE_FLAG(picHeader->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA), "slice_sao_chroma_flag");
#endif
      }
    }
    else 
    {
      picHeader->setSaoEnabledFlag(CHANNEL_TYPE_LUMA,   true);
      picHeader->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, true);
    }
  }
  else 
  {
    picHeader->setSaoEnabledFlag(CHANNEL_TYPE_LUMA,   false);
    picHeader->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, false);
  }
  
#if !JVET_Q0819_PH_CHANGES
  // alf enable flags and aps IDs
  if( sps->getALFEnabledFlag() )
  {
#if JVET_Q0819_PH_CHANGES
    if (pps->getAlfInfoInPhFlag())
#else
    WRITE_FLAG(picHeader->getAlfEnabledPresentFlag(), "pic_alf_enabled_present_flag");
    if (picHeader->getAlfEnabledPresentFlag()) 
#endif
    {
      WRITE_FLAG(picHeader->getAlfEnabledFlag(COMPONENT_Y), "pic_alf_enabled_flag");
      if (picHeader->getAlfEnabledFlag(COMPONENT_Y))
      {
        WRITE_CODE(picHeader->getNumAlfAps(), 3, "pic_num_alf_aps_ids_luma");
        const std::vector<int>&   apsId = picHeader->getAlfAPSs();
        for (int i = 0; i < picHeader->getNumAlfAps(); i++)
        {
          WRITE_CODE(apsId[i], 3, "pic_alf_aps_id_luma");
        }
        
        const int alfChromaIdc = picHeader->getAlfEnabledFlag(COMPONENT_Cb) + picHeader->getAlfEnabledFlag(COMPONENT_Cr) * 2 ;
        if (sps->getChromaFormatIdc() != CHROMA_400)
        {
          WRITE_CODE(alfChromaIdc, 2, "pic_alf_chroma_idc");
        }
        if (alfChromaIdc)
        {
          WRITE_CODE(picHeader->getAlfApsIdChroma(), 3, "pic_alf_aps_id_chroma");
        }
#if JVET_Q0795_CCALF
        if (sps->getCCALFEnabledFlag())
        {
          WRITE_FLAG(picHeader->getCcAlfEnabledFlag(COMPONENT_Cb), "ph_cc_alf_cb_enabled_flag");
          if (picHeader->getCcAlfEnabledFlag(COMPONENT_Cb))
          {
            WRITE_CODE(picHeader->getCcAlfCbApsId(), 3, "ph_cc_alf_cb_aps_id");
          }
          WRITE_FLAG(picHeader->getCcAlfEnabledFlag(COMPONENT_Cr), "ph_cc_alf_cr_enabled_flag");
          if (picHeader->getCcAlfEnabledFlag(COMPONENT_Cr))
          {
            WRITE_CODE(picHeader->getCcAlfCrApsId(), 3, "ph_cc_alf_cr_aps_id");
          }
        }
#endif
      }
    }
    else 
    {
      picHeader->setAlfEnabledFlag(COMPONENT_Y,  true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cb, true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cr, true);
#if JVET_Q0795_CCALF
      picHeader->setCcAlfEnabledFlag(COMPONENT_Cb, sps->getCCALFEnabledFlag());
      picHeader->setCcAlfEnabledFlag(COMPONENT_Cr, sps->getCCALFEnabledFlag());
#endif
    }
  }
  else 
  {
    picHeader->setAlfEnabledFlag(COMPONENT_Y,  false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cb, false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cr, false);
#if JVET_Q0795_CCALF
    picHeader->setCcAlfEnabledFlag(COMPONENT_Cb, false);
    picHeader->setCcAlfEnabledFlag(COMPONENT_Cr, false);
#endif
  }
#endif

  // dependent quantization
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
  if (!pps->getPPSDepQuantEnabledIdc())
  {
    WRITE_FLAG(picHeader->getDepQuantEnabledFlag(), "pic_dep_quant_enabled_flag");
  }
  else
  {
    picHeader->setDepQuantEnabledFlag( pps->getPPSDepQuantEnabledIdc() - 1 );
  }
#else
  WRITE_FLAG(picHeader->getDepQuantEnabledFlag(), "pic_dep_quant_enabled_flag");
#endif
  // sign data hiding
  if( !picHeader->getDepQuantEnabledFlag() )
  {
    WRITE_FLAG( picHeader->getSignDataHidingEnabledFlag(), "pic_sign_data_hiding_enabled_flag" );
  }
  else
  {
    picHeader->setSignDataHidingEnabledFlag(false);
  }

  // deblocking filter controls
  if (pps->getDeblockingFilterControlPresentFlag())
  {
    if(pps->getDeblockingFilterOverrideEnabledFlag())
    {    
#if JVET_Q0819_PH_CHANGES
      if (pps->getDbfInfoInPhFlag())
#else
      WRITE_FLAG(picHeader->getDeblockingFilterOverridePresentFlag(), "pic_deblocking_filter_override_present_flag");    
      if( picHeader->getDeblockingFilterOverridePresentFlag() ) 
#endif
      {
        WRITE_FLAG ( picHeader->getDeblockingFilterOverrideFlag(), "pic_deblocking_filter_override_flag" );
      }
      else
      {    
        picHeader->setDeblockingFilterOverrideFlag(false);
      }
    }
    else
    {
#if !JVET_Q0819_PH_CHANGES
      picHeader->setDeblockingFilterOverridePresentFlag(false);
#endif
      picHeader->setDeblockingFilterOverrideFlag(false);
    }

    if(picHeader->getDeblockingFilterOverrideFlag())
    {
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
      WRITE_FLAG( picHeader->getDeblockingFilterDisable(), "ph_deblocking_filter_disabled_flag" );
      if( !picHeader->getDeblockingFilterDisable() )
      {
        WRITE_SVLC( picHeader->getDeblockingFilterBetaOffsetDiv2(), "ph_beta_offset_div2" );
        WRITE_SVLC( picHeader->getDeblockingFilterTcOffsetDiv2(), "ph_tc_offset_div2" );
        WRITE_SVLC( picHeader->getDeblockingFilterBetaOffsetDiv2(), "ph_cb_beta_offset_div2" );
        WRITE_SVLC( picHeader->getDeblockingFilterTcOffsetDiv2(), "ph_cb_tc_offset_div2" );
        WRITE_SVLC( picHeader->getDeblockingFilterBetaOffsetDiv2(), "ph_cr_beta_offset_div2" );
        WRITE_SVLC( picHeader->getDeblockingFilterTcOffsetDiv2(), "ph_cr_tc_offset_div2" );
      }
#else
      WRITE_FLAG ( picHeader->getDeblockingFilterDisable(), "pic_deblocking_filter_disabled_flag" );
      if(!picHeader->getDeblockingFilterDisable())
      {
        WRITE_SVLC( picHeader->getDeblockingFilterBetaOffsetDiv2(), "pic_beta_offset_div2" );
        WRITE_SVLC( picHeader->getDeblockingFilterTcOffsetDiv2(), "pic_tc_offset_div2" );
      }
#endif
    }
    else
    {
      picHeader->setDeblockingFilterDisable       ( pps->getPPSDeblockingFilterDisabledFlag() );
      picHeader->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
      picHeader->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
      picHeader->setDeblockingFilterCbBetaOffsetDiv2( pps->getDeblockingFilterCbBetaOffsetDiv2() );
      picHeader->setDeblockingFilterCbTcOffsetDiv2  ( pps->getDeblockingFilterCbTcOffsetDiv2() );
      picHeader->setDeblockingFilterCrBetaOffsetDiv2( pps->getDeblockingFilterCrBetaOffsetDiv2() );
      picHeader->setDeblockingFilterCrTcOffsetDiv2  ( pps->getDeblockingFilterCrTcOffsetDiv2() );
#endif
    }
  }
  else
  {
    picHeader->setDeblockingFilterDisable       ( false );
    picHeader->setDeblockingFilterBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterTcOffsetDiv2  ( 0 );
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
    picHeader->setDeblockingFilterCbBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterCbTcOffsetDiv2  ( 0 );
    picHeader->setDeblockingFilterCrBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterCrTcOffsetDiv2  ( 0 );
#endif
  }

#if !JVET_Q0819_PH_CHANGES
  // luma mapping / chroma scaling controls
  if (sps->getUseLmcs())
  {
    WRITE_FLAG(picHeader->getLmcsEnabledFlag(), "pic_lmcs_enabled_flag");
    if (picHeader->getLmcsEnabledFlag())
    {
      WRITE_CODE(picHeader->getLmcsAPSId(), 2, "pic_lmcs_aps_id");
      if (sps->getChromaFormatIdc() != CHROMA_400)
      {
        WRITE_FLAG(picHeader->getLmcsChromaResidualScaleFlag(), "pic_chroma_residual_scale_flag");
      }
      else
      {
        picHeader->setLmcsChromaResidualScaleFlag(false);
      }
    }
  }
  else
  {
    picHeader->setLmcsEnabledFlag(false);
    picHeader->setLmcsChromaResidualScaleFlag(false);
  }

  // quantization scaling lists
  if( sps->getScalingListFlag() )
  {
    WRITE_FLAG( picHeader->getScalingListPresentFlag(), "pic_scaling_list_present_flag" );
    if( picHeader->getScalingListPresentFlag() )
    {
      WRITE_CODE( picHeader->getScalingListAPSId(), 3, "pic_scaling_list_aps_id" );
    }
  }
  else 
  {
    picHeader->setScalingListPresentFlag( false );
  }
#endif

  // picture header extension
  if(pps->getPictureHeaderExtensionPresentFlag())
  {
    WRITE_UVLC(0,"pic_segment_header_extension_length");
  }
  
#if JVET_Q0775_PH_IN_SH
  if ( writeRbspTrailingBits )
  {
    xWriteRbspTrailingBits();
  }
#else
  xWriteRbspTrailingBits();
#endif
}

void HLSWriter::codeSliceHeader         ( Slice* pcSlice )
{
#if ENABLE_TRACING
  xTraceSliceHeader ();
#endif

  CodingStructure& cs = *pcSlice->getPic()->cs;
#if JVET_Q0775_PH_IN_SH
  PicHeader *picHeader = cs.picHeader;
#else
  const PicHeader *picHeader = cs.picHeader;
#endif
  const ChromaFormat format                = pcSlice->getSPS()->getChromaFormatIdc();
  const uint32_t         numberValidComponents = getNumberValidComponents(format);
  const bool         chromaEnabled         = isChromaEnabled(format);
#if JVET_Q0775_PH_IN_SH
  WRITE_FLAG(pcSlice->getPictureHeaderInSliceHeader() ? 1 : 0, "picture_header_in_slice_header_flag");
  if (pcSlice->getPictureHeaderInSliceHeader())
  {
    codePictureHeader(picHeader, false);
  }
#endif
#if !JVET_Q0819_PH_CHANGES
  int pocBits = pcSlice->getSPS()->getBitsForPOC();
  int pocMask = (1 << pocBits) - 1;
  WRITE_CODE(pcSlice->getPOC() & pocMask, pocBits, "slice_pic_order_cnt_lsb");
#endif

#if JVET_Q0119_CLEANUPS
  if (pcSlice->getSPS()->getSubPicInfoPresentFlag())
#else
  if (pcSlice->getSPS()->getSubPicPresentFlag())
#endif
  {
    uint32_t bitsSubPicId;
#if JVET_Q0119_CLEANUPS
    if (pcSlice->getSPS()->getSubPicIdMappingExplicitlySignalledFlag())
#else
    if (pcSlice->getSPS()->getSubPicIdSignallingPresentFlag())
#endif
    {
      bitsSubPicId = pcSlice->getSPS()->getSubPicIdLen();
    }
#if !JVET_Q0119_CLEANUPS
    else if (picHeader->getSubPicIdSignallingPresentFlag())
    {
      bitsSubPicId = picHeader->getSubPicIdLen();
    }
#endif
#if JVET_Q0119_CLEANUPS
    else if (pcSlice->getPPS()->getSubPicIdMappingInPpsFlag())
#else
    else if (pcSlice->getPPS()->getSubPicIdSignallingPresentFlag())
#endif
    {
      bitsSubPicId = pcSlice->getPPS()->getSubPicIdLen();
    }
    else
    {
      bitsSubPicId = ceilLog2(pcSlice->getSPS()->getNumSubPics());
    }
    WRITE_CODE(pcSlice->getSliceSubPicId(), bitsSubPicId, "slice_subpic_id");
  }

  // raster scan slices
  if( pcSlice->getPPS()->getRectSliceFlag() == 0 ) 
  {
    // slice address is the raster scan tile index of first tile in slice
    if( pcSlice->getPPS()->getNumTiles() > 1 ) 
    {      
      int bitsSliceAddress = ceilLog2(pcSlice->getPPS()->getNumTiles());
      WRITE_CODE( pcSlice->getSliceID(), bitsSliceAddress, "slice_address");
      WRITE_UVLC( pcSlice->getNumTilesInSlice() - 1, "num_tiles_in_slice_minus1");
    }
  }
  // rectangular slices
  else 
  {
    // slice address is the index of the slice within the current sub-picture
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
    uint32_t currSubPicIdx = pcSlice->getPPS()->getSubPicIdxFromSubPicId( pcSlice->getSliceSubPicId() );
    SubPic currSubPic = pcSlice->getPPS()->getSubPic(currSubPicIdx);
    if( currSubPic.getNumSlicesInSubPic() > 1 ) 
    {
      int numSlicesInPreviousSubPics = 0;
      for(int sp = 0; sp < currSubPicIdx; sp++)
      {
        numSlicesInPreviousSubPics += pcSlice->getPPS()->getSubPic(sp).getNumSlicesInSubPic();
      }
      int bitsSliceAddress = ceilLog2(currSubPic.getNumSlicesInSubPic());
      WRITE_CODE( pcSlice->getSliceID() - numSlicesInPreviousSubPics, bitsSliceAddress, "slice_address");
    }
#else
    if( pcSlice->getPPS()->getNumSlicesInPic() > 1 ) 
    {
      int bitsSliceAddress = ceilLog2(pcSlice->getPPS()->getNumSlicesInPic());  // change to NumSlicesInSubPic when available
      WRITE_CODE( pcSlice->getSliceID(), bitsSliceAddress, "slice_address");
    }
#endif
  }

#if JVET_Q0819_PH_CHANGES
  if (picHeader->getPicInterSliceAllowedFlag())
  {
#endif
    WRITE_UVLC(pcSlice->getSliceType(), "slice_type");
#if JVET_Q0819_PH_CHANGES
  }
  if (!picHeader->getPicIntraSliceAllowedFlag())
  {
    CHECK(pcSlice->getSliceType() == I_SLICE, "when pic_intra_slice_allowed_flag = 0, no I_Slice is allowed");
  }
#endif

#if JVET_Q0819_PH_CHANGES
  if (pcSlice->getSPS()->getALFEnabledFlag() && !pcSlice->getPPS()->getAlfInfoInPhFlag())
  {
    const int alfEnabled = pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y);
    WRITE_FLAG(alfEnabled, "slice_alf_enabled_flag");

    if (alfEnabled)
    {
      WRITE_CODE(pcSlice->getTileGroupNumAps(), 3, "slice_num_alf_aps_ids_luma");
      const std::vector<int>&   apsId = pcSlice->getTileGroupApsIdLuma();
      for (int i = 0; i < pcSlice->getTileGroupNumAps(); i++)
      {
        WRITE_CODE(apsId[i], 3, "slice_alf_aps_id_luma");
      }

      const int alfChromaIdc = pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) + pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) * 2;
      if (chromaEnabled)
      {
        WRITE_CODE(alfChromaIdc, 2, "slice_alf_chroma_idc");
      }
      if (alfChromaIdc)
      {
        WRITE_CODE(pcSlice->getTileGroupApsIdChroma(), 3, "slice_alf_aps_id_chroma");
      }

#if JVET_Q0795_CCALF
      if (pcSlice->getSPS()->getCCALFEnabledFlag())
      {
        CcAlfFilterParam &filterParam = pcSlice->m_ccAlfFilterParam;
        WRITE_FLAG(filterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1] ? 1 : 0, "slice_cc_alf_cb_enabled_flag");
        if (filterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1])
        {
          // write CC ALF Cb APS ID
          WRITE_CODE(pcSlice->getTileGroupCcAlfCbApsId(), 3, "slice_cc_alf_cb_aps_id");
        }
        // Cr
        WRITE_FLAG(filterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1] ? 1 : 0, "slice_cc_alf_cr_enabled_flag");
        if (filterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1])
        {
          // write CC ALF Cr APS ID
          WRITE_CODE(pcSlice->getTileGroupCcAlfCrApsId(), 3, "slice_cc_alf_cr_aps_id");
        }
      }
#endif
    }
  }
#endif

#if JVET_Q0155_COLOUR_ID
    // 4:4:4 colour plane ID
    if( pcSlice->getSPS()->getSeparateColourPlaneFlag() )
    {
      WRITE_CODE( pcSlice->getColourPlaneId(), 2, "colour_plane_id" );
    }
#endif

#if JVET_Q0819_PH_CHANGES
  if( !pcSlice->getPPS()->getRplInfoInPhFlag() && (!pcSlice->getIdrPicFlag() || pcSlice->getSPS()->getIDRRefParamListPresent()))
#else
    if( !picHeader->getPicRplPresentFlag() && (!pcSlice->getIdrPicFlag() || pcSlice->getSPS()->getIDRRefParamListPresent()) )
#endif
    {
      //Write L0 related syntax elements
      if (pcSlice->getSPS()->getNumRPL0() > 0)
      {
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
        if (!pcSlice->getPPS()->getPPSRefPicListSPSIdc0())
        {
#endif
          WRITE_FLAG(pcSlice->getRPL0idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[0]");
#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
        }
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
        xCodeRefPicList( pcSlice->getRPL0(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC(), !pcSlice->getSPS()->getUseWP() && !pcSlice->getSPS()->getUseWPBiPred() );
      }
      //Deal POC Msb cycle signalling for LTRP
      if (pcSlice->getRPL0()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL0()->getNumberOfLongtermPictures() + pcSlice->getRPL0()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL0()->isRefPicLongterm(i))
          {
            if (pcSlice->getRPL0()->getLtrpInSliceHeaderFlag())
            {
              WRITE_CODE(pcSlice->getRPL0()->getRefPicIdentifier(i), pcSlice->getSPS()->getBitsForPOC(),
                         "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
            }
            WRITE_FLAG(pcSlice->getLocalRPL0()->getDeltaPocMSBPresentFlag(i) ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
            if (pcSlice->getLocalRPL0()->getDeltaPocMSBPresentFlag(i))
            {
              WRITE_UVLC(pcSlice->getLocalRPL0()->getDeltaPocMSBCycleLT(i), "delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }

      //Write L1 related syntax elements
#if JVET_Q0482_REMOVE_CONSTANT_PARAMS
      if (pcSlice->getSPS()->getNumRPL1() > 0 && pcSlice->getPPS()->getRpl1IdxPresentFlag())
#else
      if (pcSlice->getSPS()->getNumRPL1() > 0 && pcSlice->getPPS()->getRpl1IdxPresentFlag() && !pcSlice->getPPS()->getPPSRefPicListSPSIdc1())
#endif
      {
        WRITE_FLAG(pcSlice->getRPL1idx() != -1 ? 1 : 0, "ref_pic_list_sps_flag[1]");
      }
      else if (pcSlice->getSPS()->getNumRPL1() == 0)
      {
        CHECK(pcSlice->getRPL1idx() != -1, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }
      else
      {
        auto rplsSpsFlag0 = pcSlice->getRPL0idx() != -1 ? 1 : 0;
        auto rplsSpsFlag1 = pcSlice->getRPL1idx() != -1 ? 1 : 0;
        CHECK(rplsSpsFlag1 != rplsSpsFlag0, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }

      if (pcSlice->getRPL1idx() != -1)
      {
        if (pcSlice->getSPS()->getNumRPL1() > 1 && pcSlice->getPPS()->getRpl1IdxPresentFlag())
        {
          int numBits = 0;
          while ((1 << numBits) < pcSlice->getSPS()->getNumRPL1())
          {
            numBits++;
          }
          WRITE_CODE(pcSlice->getRPL1idx(), numBits, "ref_pic_list_idx[1]");
        }
        else if (pcSlice->getSPS()->getNumRPL1() == 1)
        {
          CHECK(pcSlice->getRPL1idx() != 0, "RPL1Idx is not signalled but it is not equal to 0");
        }
        else
        {
          CHECK(pcSlice->getRPL1idx() != pcSlice->getRPL0idx(), "RPL1Idx is not signalled but it is not the same as RPL0Idx");
        }
      }
      else
      {  //write local RPL1
        xCodeRefPicList( pcSlice->getRPL1(), pcSlice->getSPS()->getLongTermRefsPresent(), pcSlice->getSPS()->getBitsForPOC(), !pcSlice->getSPS()->getUseWP() && !pcSlice->getSPS()->getUseWPBiPred() );
      }
      //Deal POC Msb cycle signalling for LTRP
      if (pcSlice->getRPL1()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL1()->getNumberOfLongtermPictures() + pcSlice->getRPL1()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL1()->isRefPicLongterm(i))
          {
            if (pcSlice->getRPL1()->getLtrpInSliceHeaderFlag())
            {
              WRITE_CODE(pcSlice->getRPL1()->getRefPicIdentifier(i), pcSlice->getSPS()->getBitsForPOC(),
                         "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
            }
            WRITE_FLAG(pcSlice->getLocalRPL1()->getDeltaPocMSBPresentFlag(i) ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
            if (pcSlice->getLocalRPL1()->getDeltaPocMSBPresentFlag(i))
            {
              WRITE_UVLC(pcSlice->getLocalRPL1()->getDeltaPocMSBCycleLT(i), "delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }
    }

#if JVET_Q0819_PH_CHANGES
    if( pcSlice->getPPS()->getRplInfoInPhFlag() || !pcSlice->getIdrPicFlag()|| pcSlice->getSPS()->getIDRRefParamListPresent() )
#else
    if( picHeader->getPicRplPresentFlag() || !pcSlice->getIdrPicFlag() || pcSlice->getSPS()->getIDRRefParamListPresent() )
#endif
    {
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
#if JVET_Q0259_COLLOCATED_PIC_IN_PH
    if (pcSlice->getPicHeader()->getEnableTMVPFlag() && !pcSlice->getPPS()->getRplInfoInPhFlag())
#else
    if( pcSlice->getPicHeader()->getEnableTMVPFlag() )
#endif
    {
#if JVET_Q0482_REMOVE_CONSTANT_PARAMS
      if(!pcSlice->getPPS()->getRplInfoInPhFlag())
      {
        if (pcSlice->getSliceType() == B_SLICE)
        {
          WRITE_FLAG(pcSlice->getColFromL0Flag(), "collocated_from_l0_flag");
        }
      }
#else
      if( pcSlice->getSliceType() == B_SLICE )
      {
        if (!pcSlice->getPPS()->getPPSCollocatedFromL0Idc())
        {
          WRITE_FLAG( pcSlice->getColFromL0Flag(), "collocated_from_l0_flag" );
        }
      }
#endif

      if( pcSlice->getSliceType() != I_SLICE &&
        ( ( pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > 1 ) ||
          ( pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > 1 ) ) )
      {
        WRITE_UVLC( pcSlice->getColRefIdx(), "collocated_ref_idx" );
      }
    }

    if( ( pcSlice->getPPS()->getUseWP() && pcSlice->getSliceType() == P_SLICE ) || ( pcSlice->getPPS()->getWPBiPred() && pcSlice->getSliceType() == B_SLICE ) )
    {
#if JVET_Q0819_PH_CHANGES
      if( !pcSlice->getPPS()->getWpInfoInPhFlag() )
      {
        xCodePredWeightTable(pcSlice);
      }
#else
      xCodePredWeightTable(pcSlice);
#endif
    }


#if JVET_Q0819_PH_CHANGES
    if (!pcSlice->getPPS()->getQpDeltaInfoInPhFlag())
    {
      WRITE_SVLC(pcSlice->getSliceQp() - (pcSlice->getPPS()->getPicInitQPMinus26() + 26), "slice_qp_delta");
    }
#else
    int iCode = pcSlice->getSliceQp() - (pcSlice->getPPS()->getPicInitQPMinus26() + 26);
    WRITE_SVLC( iCode, "slice_qp_delta" );
#endif
    if (pcSlice->getPPS()->getSliceChromaQpFlag())
    {
      if (numberValidComponents > COMPONENT_Cb)
      {
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb), "slice_cb_qp_offset" );
      }
      if (numberValidComponents > COMPONENT_Cr)
      {
        WRITE_SVLC( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr), "slice_cr_qp_offset" );
        if (pcSlice->getSPS()->getJointCbCrEnabledFlag())
        {
          WRITE_SVLC( pcSlice->getSliceChromaQpDelta(JOINT_CbCr), "slice_joint_cbcr_qp_offset");
        }
      }
      CHECK(numberValidComponents < COMPONENT_Cr+1, "Too many valid components");
    }

    if (pcSlice->getPPS()->getCuChromaQpOffsetListEnabledFlag())
    {
      WRITE_FLAG(pcSlice->getUseChromaQpAdj(), "cu_chroma_qp_offset_enabled_flag");
    }

#if JVET_Q0819_PH_CHANGES
    if (pcSlice->getSPS()->getSAOEnabledFlag() && !pcSlice->getPPS()->getSaoInfoInPhFlag())
#else
    if (pcSlice->getSPS()->getSAOEnabledFlag() && !picHeader->getSaoEnabledPresentFlag())
#endif
    {
      WRITE_FLAG( pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_LUMA ), "slice_sao_luma_flag" );
      if( chromaEnabled )
      {
        WRITE_FLAG( pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ), "slice_sao_chroma_flag" );
      }
    }

#if !JVET_Q0819_PH_CHANGES
#if JVET_Q0819_PH_CHANGES
    if (pcSlice->getSPS()->getALFEnabledFlag() && !pcSlice->getPPS()->getAlfInfoInPhFlag())
#else
    if (pcSlice->getSPS()->getALFEnabledFlag() && !picHeader->getAlfEnabledPresentFlag())
#endif
    {
      const int alfEnabled = pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y);
      WRITE_FLAG(alfEnabled, "slice_alf_enabled_flag");

      if (alfEnabled)
      {
        WRITE_CODE(pcSlice->getTileGroupNumAps(), 3, "slice_num_alf_aps_ids_luma");
        const std::vector<int>&   apsId = pcSlice->getTileGroupApsIdLuma();
        for (int i = 0; i < pcSlice->getTileGroupNumAps(); i++)
        {
          WRITE_CODE(apsId[i], 3, "slice_alf_aps_id_luma");
        }

        const int alfChromaIdc = pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) + pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) * 2 ;
        if (chromaEnabled)
        {
          WRITE_CODE(alfChromaIdc, 2, "slice_alf_chroma_idc");
        }
        if (alfChromaIdc)
        {
          WRITE_CODE(pcSlice->getTileGroupApsIdChroma(), 3, "slice_alf_aps_id_chroma");
        }

#if JVET_Q0795_CCALF
        if (pcSlice->getSPS()->getCCALFEnabledFlag())
        {
          CcAlfFilterParam &filterParam = pcSlice->m_ccAlfFilterParam;
          WRITE_FLAG(filterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1] ? 1 : 0, "slice_cc_alf_cb_enabled_flag");
          if (filterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1])
          {
            // write CC ALF Cb APS ID
            WRITE_CODE(pcSlice->getTileGroupCcAlfCbApsId(), 3, "slice_cc_alf_cb_aps_id");
          }
          // Cr
          WRITE_FLAG(filterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1] ? 1 : 0, "slice_cc_alf_cr_enabled_flag");
          if (filterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1])
          {
            // write CC ALF Cr APS ID
            WRITE_CODE(pcSlice->getTileGroupCcAlfCrApsId(), 3, "slice_cc_alf_cr_aps_id");
          }
        }
#endif
      }
    }
#endif	

    if (pcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
    {
#if JVET_Q0819_PH_CHANGES
    if( pcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() && !pcSlice->getPPS()->getDbfInfoInPhFlag() )
    {
      WRITE_FLAG(pcSlice->getDeblockingFilterOverrideFlag(), "slice_deblocking_filter_override_flag");
#else
      if (pcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() && !picHeader->getDeblockingFilterOverridePresentFlag())
      {
        WRITE_FLAG(pcSlice->getDeblockingFilterOverrideFlag(), "deblocking_filter_override_flag");
#endif
      }
      else
      {
        pcSlice->setDeblockingFilterOverrideFlag(0);
      }
      if (pcSlice->getDeblockingFilterOverrideFlag())
      {
        WRITE_FLAG(pcSlice->getDeblockingFilterDisable(), "slice_deblocking_filter_disabled_flag");
        if(!pcSlice->getDeblockingFilterDisable())
        {
          WRITE_SVLC (pcSlice->getDeblockingFilterBetaOffsetDiv2(), "slice_beta_offset_div2");
          WRITE_SVLC (pcSlice->getDeblockingFilterTcOffsetDiv2(),   "slice_tc_offset_div2");
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
          WRITE_SVLC (pcSlice->getDeblockingFilterCbBetaOffsetDiv2(), "slice_cb_beta_offset_div2");
          WRITE_SVLC (pcSlice->getDeblockingFilterCbTcOffsetDiv2(),   "slice_cb_tc_offset_div2");
          WRITE_SVLC (pcSlice->getDeblockingFilterCrBetaOffsetDiv2(), "slice_cr_beta_offset_div2");
          WRITE_SVLC (pcSlice->getDeblockingFilterCrTcOffsetDiv2(),   "slice_cr_tc_offset_div2");
#endif
        }
      }
      else
      {
        pcSlice->setDeblockingFilterDisable       ( picHeader->getDeblockingFilterDisable() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( picHeader->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( picHeader->getDeblockingFilterTcOffsetDiv2() );
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
        pcSlice->setDeblockingFilterCbBetaOffsetDiv2( picHeader->getDeblockingFilterCbBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( picHeader->getDeblockingFilterCbTcOffsetDiv2() );
        pcSlice->setDeblockingFilterCrBetaOffsetDiv2( picHeader->getDeblockingFilterCrBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( picHeader->getDeblockingFilterCrTcOffsetDiv2() );
#endif
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable       ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
      pcSlice->setDeblockingFilterCbBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( 0 );
      pcSlice->setDeblockingFilterCrBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( 0 );
#endif
    }

#if JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
	WRITE_FLAG(pcSlice->getTSResidualCodingDisabledFlag() ? 1 : 0, "slice_ts_residual_coding_disabled_flag");
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
#if JVET_Q0114_CONSTRAINT_FLAGS
  WRITE_FLAG(cinfo->getNonProjectedConstraintFlag(), "general_non_projected_constraint_flag");
#endif
  WRITE_FLAG(cinfo->getIntraOnlyConstraintFlag(),     "intra_only_constraint_flag"      );

  WRITE_CODE(cinfo->getMaxBitDepthConstraintIdc(), 4, "max_bitdepth_constraint_idc" );
  WRITE_CODE(cinfo->getMaxChromaFormatConstraintIdc(), 2, "max_chroma_format_constraint_idc" );
#if JVET_Q0114_CONSTRAINT_FLAGS
  WRITE_FLAG(cinfo->getNoResChangeInClvsConstraintFlag(), "no_res_change_in_clvs_constraint_flag");
  WRITE_FLAG(cinfo->getOneTilePerPicConstraintFlag(), "one_tile_per_pic_constraint_flag");
  WRITE_FLAG(cinfo->getOneSlicePerPicConstraintFlag(), "one_slice_per_pic_constraint_flag");
  WRITE_FLAG(cinfo->getOneSubpicPerPicConstraintFlag(), "one_subpic_per_pic_constraint_flag");
#endif

  WRITE_FLAG(cinfo->getNoQtbttDualTreeIntraConstraintFlag() ? 1 : 0, "no_qtbtt_dual_tree_intra_constraint_flag");
  WRITE_FLAG(cinfo->getNoPartitionConstraintsOverrideConstraintFlag() ? 1 : 0, "no_partition_constraints_override_constraint_flag");
  WRITE_FLAG(cinfo->getNoSaoConstraintFlag() ? 1 : 0, "no_sao_constraint_flag");
  WRITE_FLAG(cinfo->getNoAlfConstraintFlag() ? 1 : 0, "no_alf_constraint_flag");
#if JVET_Q0795_CCALF
  WRITE_FLAG(cinfo->getNoCCAlfConstraintFlag() ? 1 : 0, "no_ccalf_constraint_flag");
#endif
  WRITE_FLAG(cinfo->getNoJointCbCrConstraintFlag() ? 1 : 0, "no_joint_cbcr_constraint_flag");
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
  WRITE_FLAG(cinfo->getNoBcwConstraintFlag() ? 1 : 0, "no_bcw_constraint_flag");
  WRITE_FLAG(cinfo->getNoIbcConstraintFlag() ? 1 : 0, "no_ibc_constraint_flag");
  WRITE_FLAG(cinfo->getNoCiipConstraintFlag() ? 1 : 0, "no_ciip_constraint_flag");
  WRITE_FLAG(cinfo->getNoFPelMmvdConstraintFlag() ? 1 : 0, "no_fpel_mmvd_constraint_flag");
#if !JVET_Q0806
  WRITE_FLAG(cinfo->getNoTriangleConstraintFlag() ? 1 : 0, "no_triangle_constraint_flag");
#else
  WRITE_FLAG(cinfo->getNoGeoConstraintFlag() ? 1 : 0, "no_gpm_constraint_flag");
#endif
  WRITE_FLAG(cinfo->getNoLadfConstraintFlag() ? 1 : 0, "no_ladf_constraint_flag");
  WRITE_FLAG(cinfo->getNoTransformSkipConstraintFlag() ? 1 : 0, "no_transform_skip_constraint_flag");
  WRITE_FLAG(cinfo->getNoBDPCMConstraintFlag() ? 1 : 0, "no_bdpcm_constraint_flag");
  WRITE_FLAG(cinfo->getNoQpDeltaConstraintFlag() ? 1 : 0, "no_qp_delta_constraint_flag");
  WRITE_FLAG(cinfo->getNoDepQuantConstraintFlag() ? 1 : 0, "no_dep_quant_constraint_flag");
  WRITE_FLAG(cinfo->getNoSignDataHidingConstraintFlag() ? 1 : 0, "no_sign_data_hiding_constraint_flag");
  WRITE_FLAG(cinfo->getNoTrailConstraintFlag() ? 1 : 0, "no_trail_constraint_flag");
  WRITE_FLAG(cinfo->getNoStsaConstraintFlag() ? 1 : 0, "no_stsa_constraint_flag");
  WRITE_FLAG(cinfo->getNoRaslConstraintFlag() ? 1 : 0, "no_rasl_constraint_flag");
  WRITE_FLAG(cinfo->getNoRadlConstraintFlag() ? 1 : 0, "no_radl_constraint_flag");
  WRITE_FLAG(cinfo->getNoIdrConstraintFlag() ? 1 : 0, "no_idr_constraint_flag");
  WRITE_FLAG(cinfo->getNoCraConstraintFlag() ? 1 : 0, "no_cra_constraint_flag");
  WRITE_FLAG(cinfo->getNoGdrConstraintFlag() ? 1 : 0, "no_gdr_constraint_flag");
  WRITE_FLAG(cinfo->getNoApsConstraintFlag() ? 1 : 0, "no_aps_constraint_flag");
}

#if JVET_Q0786_PTL_only
void  HLSWriter::codeProfileTierLevel    ( const ProfileTierLevel* ptl, bool profileTierPresentFlag, int maxNumSubLayersMinus1 )
#else
void  HLSWriter::codeProfileTierLevel    ( const ProfileTierLevel* ptl, int maxNumSubLayersMinus1 )
#endif
{
#if JVET_Q0786_PTL_only
  if(profileTierPresentFlag)
  {
    WRITE_CODE( int(ptl->getProfileIdc()), 7 ,   "general_profile_idc"                     );
    WRITE_FLAG( ptl->getTierFlag()==Level::HIGH, "general_tier_flag"                       );
    codeConstraintInfo( ptl->getConstraintInfo() );
  }
#else
  WRITE_CODE( int(ptl->getProfileIdc()), 7 ,   "general_profile_idc"                     );
  WRITE_FLAG( ptl->getTierFlag()==Level::HIGH, "general_tier_flag"                       );

  codeConstraintInfo( ptl->getConstraintInfo() );
#endif

  WRITE_CODE( int( ptl->getLevelIdc() ), 8, "general_level_idc" );

#if JVET_Q0786_PTL_only
  if(profileTierPresentFlag)
  {
    WRITE_CODE(ptl->getNumSubProfile(), 8, "num_sub_profiles");
    for (int i = 0; i < ptl->getNumSubProfile(); i++)
    {
      WRITE_CODE(ptl->getSubProfileIdc(i) , 32, "general_sub_profile_idc[i]");
    }
  }
#else
  WRITE_CODE(ptl->getNumSubProfile(), 8, "num_sub_profiles");
  for (int i = 0; i < ptl->getNumSubProfile(); i++)
  {
    WRITE_CODE(ptl->getSubProfileIdc(i) , 32, "general_sub_profile_idc[i]");
  }
#endif

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
#if JVET_Q0151_Q0205_ENTRYPOINTS
  pSlice->setNumEntryPoints( pSlice->getSPS(), pSlice->getPPS() );
#else
  pSlice->setNumEntryPoints( pSlice->getPPS() );
#endif
  if( pSlice->getNumEntryPoints() == 0 )
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

#if JVET_Q0819_PH_CHANGES
void HLSWriter::xCodePredWeightTable(PicHeader *picHeader, const SPS *sps)
{
  WPScalingParam *   wp;
  const ChromaFormat format                      = sps->getChromaFormatIdc();
  const uint32_t     numberValidComponents       = getNumberValidComponents(format);
  const bool         chroma                      = isChromaEnabled(format);
  bool               denomCoded                  = false;
  uint32_t           totalSignalledWeightFlags   = 0;

  uint32_t numLxWeights                          = picHeader->getNumL0Weights();
  bool     moreSyntaxToBeParsed                  = true;
  for (int numRef = 0; numRef < NUM_REF_PIC_LIST_01 && moreSyntaxToBeParsed; numRef++)   // loop over l0 and l1 syntax elements
  {
    RefPicList refPicList = (numRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    // NOTE: wp[].uiLog2WeightDenom and wp[].bPresentFlag are actually per-channel-type settings.

    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      picHeader->getWpScaling(refPicList, refIdx, wp);
      if (!denomCoded)
      {
        int deltaDenom;
        WRITE_UVLC(wp[COMPONENT_Y].uiLog2WeightDenom, "luma_log2_weight_denom");

        if (chroma)
        {
          CHECK(wp[COMPONENT_Cb].uiLog2WeightDenom != wp[COMPONENT_Cr].uiLog2WeightDenom, "Chroma blocks of different size not supported");
          deltaDenom = (wp[COMPONENT_Cb].uiLog2WeightDenom - wp[COMPONENT_Y].uiLog2WeightDenom);
          WRITE_SVLC(deltaDenom, "delta_chroma_log2_weight_denom");
        }
        denomCoded = true;
      }
      WRITE_FLAG(wp[COMPONENT_Y].bPresentFlag, numRef == 0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]");
      totalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if (chroma)
    {
      for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
      {
        picHeader->getWpScaling(refPicList, refIdx, wp);
        CHECK(wp[COMPONENT_Cb].bPresentFlag != wp[COMPONENT_Cr].bPresentFlag, "Inconsistent settings for chroma channels");
        WRITE_FLAG(wp[COMPONENT_Cb].bPresentFlag, numRef == 0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]");
        totalSignalledWeightFlags += 2 * wp[COMPONENT_Cb].bPresentFlag;
      }
    }

    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      picHeader->getWpScaling(refPicList, refIdx, wp);
      if (wp[COMPONENT_Y].bPresentFlag)
      {
        int deltaWeight = (wp[COMPONENT_Y].iWeight - (1 << wp[COMPONENT_Y].uiLog2WeightDenom));
        WRITE_SVLC(deltaWeight, numRef == 0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]");
        WRITE_SVLC(wp[COMPONENT_Y].iOffset, numRef == 0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]");
      }

      if (chroma)
      {
        if (wp[COMPONENT_Cb].bPresentFlag)
        {
          for (int j = COMPONENT_Cb; j < numberValidComponents; j++)
          {
            CHECK(wp[COMPONENT_Cb].uiLog2WeightDenom != wp[COMPONENT_Cr].uiLog2WeightDenom, "Chroma blocks of different size not supported");
            int deltaWeight = (wp[j].iWeight - (1 << wp[COMPONENT_Cb].uiLog2WeightDenom));
            WRITE_SVLC(deltaWeight, numRef == 0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]");

            int range = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1 << sps->getBitDepth(CHANNEL_TYPE_CHROMA)) / 2 : 128;
            int pred         = (range - ((range * wp[j].iWeight) >> (wp[j].uiLog2WeightDenom)));
            int deltaChroma = (wp[j].iOffset - pred);
            WRITE_SVLC(deltaChroma, numRef == 0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]");
          }
        }
      }
    }
    if (numRef == 0)
    {
      numLxWeights         = picHeader->getNumL1Weights();
      moreSyntaxToBeParsed = (numLxWeights == 0) ? false : true;
    }
  }
  CHECK(totalSignalledWeightFlags > 24, "Too many signalled weight flags");
}
#endif

/** code quantization matrix
*  \param scalingList quantization matrix information
*/
void HLSWriter::codeScalingList( const ScalingList &scalingList )
{
  //for each size
  WRITE_FLAG(scalingList.getDisableScalingMatrixForLfnstBlks(), "scaling_matrix_for_lfnst_disabled_flag"); 
#if JVET_Q0505_CHROAM_QM_SIGNALING_400
  WRITE_FLAG(scalingList.getChromaScalingListPresentFlag(), "scaling_list_chroma_present_flag");
#endif
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
#if JVET_Q0505_CHROAM_QM_SIGNALING_400
    if(scalingList.getChromaScalingListPresentFlag()|| scalingList.isLumaScalingList(scalingListId))
   {
#endif
    bool scalingListCopyModeFlag = scalingList.getScalingListCopyModeFlag(scalingListId);
    WRITE_FLAG(scalingListCopyModeFlag, "scaling_list_copy_mode_flag"); //copy mode
    if (!scalingListCopyModeFlag)// Copy Mode
    {
      WRITE_FLAG(scalingList.getScalingListPreditorModeFlag(scalingListId), "scaling_list_predictor_mode_flag");
    }
    if ((scalingListCopyModeFlag || scalingList.getScalingListPreditorModeFlag(scalingListId)) && scalingListId!= SCALING_LIST_1D_START_2x2 && scalingListId != SCALING_LIST_1D_START_4x4 && scalingListId != SCALING_LIST_1D_START_8x8)
    {
      WRITE_UVLC((int)scalingListId - (int)scalingList.getRefMatrixId(scalingListId), "scaling_list_pred_matrix_id_delta");
    }
    if (!scalingListCopyModeFlag)
    {
      //DPCM
      xCodeScalingList(&scalingList, scalingListId, scalingList.getScalingListPreditorModeFlag(scalingListId));
    }
#if JVET_Q0505_CHROAM_QM_SIGNALING_400
   }
#endif
  }
  return;
}
/** code DPCM
* \param scalingList quantization matrix information
* \param sizeId      size index
* \param listId      list index
*/
void HLSWriter::xCodeScalingList(const ScalingList* scalingList, uint32_t scalingListId, bool isPredictor)
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : ((scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8);
  int coefNum = matrixSize * matrixSize;
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)][gp_sizeIdxInfo->idxFrom(matrixSize)];
  int nextCoef = (isPredictor) ? 0 : SCALING_LIST_START_VALUE;

  int data;
  const int *src = scalingList->getScalingListAddress(scalingListId);
  int PredListId = scalingList->getRefMatrixId(scalingListId);
  const int *srcPred = (isPredictor) ? ((scalingListId==PredListId) ? scalingList->getScalingListDefaultAddress(scalingListId) : scalingList->getScalingListAddress(PredListId)) : NULL;
  int deltasrc[65] = { 0 };

  if (isPredictor)
  {
    if (scalingListId >= SCALING_LIST_1D_START_16x16)
    {
      deltasrc[64] = scalingList->getScalingListDC(scalingListId) - ((PredListId >= SCALING_LIST_1D_START_16x16) ? ((scalingListId == PredListId) ? 16 : scalingList->getScalingListDC(PredListId)) : srcPred[scan[0].idx]);
    }
    for (int i = 0; i < coefNum; i++)
    {
      deltasrc[i] = (src[scan[i].idx] - srcPred[scan[i].idx]);
    }
  }
  if (scalingListId >= SCALING_LIST_1D_START_16x16)
  {
    if (isPredictor)
    {
      data = deltasrc[64];
      nextCoef = deltasrc[64];
    }
    else
    {
      data = scalingList->getScalingListDC(scalingListId) - nextCoef;
      nextCoef = scalingList->getScalingListDC(scalingListId);
    }
    data = ((data + 128) & 255) - 128;
    WRITE_SVLC((int8_t)data, "scaling_list_dc_coef");
  }
  for(int i=0;i<coefNum;i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
      continue;
    data = (isPredictor) ? (deltasrc[i] - nextCoef) : (src[scan[i].idx] - nextCoef);
    nextCoef = (isPredictor) ? deltasrc[i] : src[scan[i].idx];
    data = ((data + 128) & 255) - 128;
    WRITE_SVLC((int8_t)data, "scaling_list_delta_coef");
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

#if !JVET_Q0210_UEK_REMOVAL
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
#endif

void HLSWriter::alfFilter( const AlfParam& alfParam, const bool isChroma, const int altIdx )
{
  AlfFilterShape alfShape(isChroma ? 5 : 7);
  const short* coeff = isChroma ? alfParam.chromaCoeff[altIdx] : alfParam.lumaCoeff;
  const short* clipp = isChroma ? alfParam.chromaClipp[altIdx] : alfParam.lumaClipp;
  const int numFilters = isChroma ? 1 : alfParam.numLumaFilters;

  // vlc for all

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
#if JVET_Q0210_UEK_REMOVAL 
      WRITE_UVLC( abs(coeff[ ind* MAX_NUM_ALF_LUMA_COEFF + i ]), isChroma ? "alf_chroma_coeff_abs" : "alf_luma_coeff_abs" ); //alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
      if( abs( coeff[ ind* MAX_NUM_ALF_LUMA_COEFF + i ] ) != 0 )
      {
        WRITE_FLAG( ( coeff[ ind* MAX_NUM_ALF_LUMA_COEFF + i ] < 0 ) ? 1 : 0, isChroma ? "alf_chroma_coeff_sign" : "alf_luma_coeff_sign" );
      }
#else
      alfGolombEncode( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], 3 );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#endif
    }
  }

  // Clipping values coding
#if JVET_Q0249_ALF_CHROMA_CLIPFLAG
  if( alfParam.nonLinearFlag[isChroma] )
#else
  if( alfParam.nonLinearFlag[isChroma][altIdx] )
#endif
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        WRITE_CODE(clipp[ind* MAX_NUM_ALF_LUMA_COEFF + i], 2, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
      }
    }
  }
}


//! \}
