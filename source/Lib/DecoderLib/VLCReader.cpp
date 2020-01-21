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

/** \file     VLCWReader.cpp
 *  \brief    Reader for high level syntax
 */

//! \ingroup DecoderLib
//! \{

#include "VLCReader.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/dtrace_next.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#include "CommonLib/AdaptiveLoopFilter.h"

#if ENABLE_TRACING

void  VLCReader::xReadCodeTr(uint32_t length, uint32_t& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadCode (length, rValue, pSymbolName);
#else
  xReadCode (length, rValue);
#endif
  if (length < 10)
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %u\n", pSymbolName, length, rValue );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %u\n", pSymbolName, length, rValue );
  }
}

void  VLCReader::xReadUvlcTr(uint32_t& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadUvlc (rValue, pSymbolName);
#else
  xReadUvlc (rValue);
#endif
  DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %u\n", pSymbolName, rValue );
}

void  VLCReader::xReadSvlcTr(int& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadSvlc (rValue, pSymbolName);
#else
  xReadSvlc (rValue);
#endif
  DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, rValue );
}

void  VLCReader::xReadFlagTr(uint32_t& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadFlag (rValue, pSymbolName);
#else
  xReadFlag (rValue);
#endif
  DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, rValue );
}

void xTraceFillerData ()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Filler Data ===========\n");
}

#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS || ENABLE_TRACING
void VLCReader::xReadSCode (uint32_t length, int& value, const char *pSymbolName)
#else
void VLCReader::xReadSCode (uint32_t length, int& value)
#endif
{
  uint32_t val;
  assert ( length > 0 && length<=32);
  m_pcBitstream->read (length, val);
  value= length>=32 ? int(val) : ( (-int( val & (uint32_t(1)<<(length-1)))) | int(val) );

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, length, value);
#endif
#if ENABLE_TRACING
  if (length < 10)
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d)  : %d\n", pSymbolName, length, value );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d) : %d\n", pSymbolName, length, value );
  }
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadCode (uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName)
#else
void VLCReader::xReadCode (uint32_t uiLength, uint32_t& ruiCode)
#endif
{
  CHECK( uiLength == 0, "Reading a code of length '0'" );
  m_pcBitstream->read (uiLength, ruiCode);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, uiLength, ruiCode);
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadUvlc( uint32_t& ruiVal, const char *pSymbolName)
#else
void VLCReader::xReadUvlc( uint32_t& ruiVal)
#endif
{
  uint32_t uiVal = 0;
  uint32_t uiCode = 0;
  uint32_t uiLength;
  m_pcBitstream->read( 1, uiCode );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  uint32_t totalLen=1;
#endif

  if( 0 == uiCode )
  {
    uiLength = 0;

    while( ! ( uiCode & 1 ))
    {
      m_pcBitstream->read( 1, uiCode );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiVal );

    uiVal += (1 << uiLength)-1;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    totalLen+=uiLength+uiLength;
#endif
  }

  ruiVal = uiVal;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, int(totalLen), ruiVal);
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadSvlc( int& riVal, const char *pSymbolName)
#else
void VLCReader::xReadSvlc( int& riVal)
#endif
{
  uint32_t uiBits = 0;
  m_pcBitstream->read( 1, uiBits );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  uint32_t totalLen=1;
#endif
  if( 0 == uiBits )
  {
    uint32_t uiLength = 0;

    while( ! ( uiBits & 1 ))
    {
      m_pcBitstream->read( 1, uiBits );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiBits );

    uiBits += (1 << uiLength);
    riVal = ( uiBits & 1) ? -(int)(uiBits>>1) : (int)(uiBits>>1);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    totalLen+=uiLength+uiLength;
#endif
  }
  else
  {
    riVal = 0;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, int(totalLen), uiBits);
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadFlag (uint32_t& ruiCode, const char *pSymbolName)
#else
void VLCReader::xReadFlag (uint32_t& ruiCode)
#endif
{
  m_pcBitstream->read( 1, ruiCode );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, 1, int(/*ruiCode*/0));
#endif
}

void VLCReader::xReadRbspTrailingBits()
{
  uint32_t bit;
  READ_FLAG( bit, "rbsp_stop_one_bit");
  CHECK(bit!=1, "Trailing bit not '1'");
  int cnt = 0;
  while (m_pcBitstream->getNumBitsUntilByteAligned())
  {
    READ_FLAG( bit, "rbsp_alignment_zero_bit");
    CHECK(bit!=0, "Alignment bit is not '0'");
    cnt++;
  }
  CHECK(cnt >= 8, "Read more than '8' trailing bits");
}

void AUDReader::parseAccessUnitDelimiter(InputBitstream* bs, uint32_t &picType)
{
  setBitstream(bs);

#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  READ_CODE (3, picType, "pic_type");
  xReadRbspTrailingBits();
}

void FDReader::parseFillerData(InputBitstream* bs, uint32_t &fdSize)
{
  setBitstream(bs);
#if ENABLE_TRACING
  xTraceFillerData();
#endif
  uint32_t ffByte;
  fdSize = 0;
  while( m_pcBitstream->getNumBitsLeft() >8 )
  {
    READ_CODE (8, ffByte, "ff_byte");
    CHECK(ffByte!=0xff, "Invalid filler data : not '0xff'");
    fdSize++;
  }
  xReadRbspTrailingBits();
}

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

HLSyntaxReader::HLSyntaxReader()
{
}

HLSyntaxReader::~HLSyntaxReader()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void HLSyntaxReader::copyRefPicList(SPS* sps, ReferencePictureList* source_rpl, ReferencePictureList* dest_rp)
{
  dest_rp->setNumberOfShorttermPictures(source_rpl->getNumberOfShorttermPictures());

  dest_rp->setNumberOfInterLayerPictures( sps->getInterLayerPresentFlag() ? dest_rp->getNumberOfInterLayerPictures() : 0 );

  if( sps->getLongTermRefsPresent() )
  {
    dest_rp->setNumberOfLongtermPictures( dest_rp->getNumberOfLongtermPictures() );
  }
  else
    dest_rp->setNumberOfLongtermPictures(0);

  uint32_t numRefPic = dest_rp->getNumberOfShorttermPictures() + dest_rp->getNumberOfLongtermPictures();

  for( int ii = 0; ii < numRefPic; ii++ )
  {
    dest_rp->setRefPicIdentifier( ii, source_rpl->getRefPicIdentifier( ii ), source_rpl->isRefPicLongterm( ii ), source_rpl->isInterLayerRefPic( ii ), source_rpl->getInterLayerRefPicIdx( ii ) );
  }
}

void HLSyntaxReader::parseRefPicList(SPS* sps, ReferencePictureList* rpl)
{
  uint32_t code;
  READ_UVLC(code, "num_ref_entries[ listIdx ][ rplsIdx ]");
  uint32_t numRefPic = code;
  uint32_t numStrp = 0;
  uint32_t numLtrp = 0;
  uint32_t numIlrp = 0;

  if (sps->getLongTermRefsPresent())
  {
    READ_FLAG(code, "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
    rpl->setLtrpInSliceHeaderFlag(code);
  }

  bool isLongTerm;
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;

  rpl->setInterLayerPresentFlag( sps->getInterLayerPresentFlag() );

  for (int ii = 0; ii < numRefPic; ii++)
  {
    uint32_t isInterLayerRefPic = 0;

    if( rpl->getInterLayerPresentFlag() )
    {
      READ_FLAG( isInterLayerRefPic, "inter_layer_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]" );

      if( isInterLayerRefPic )
      {
        READ_UVLC( code, "ilrp_idx[ listIdx ][ rplsIdx ][ i ]" );
        rpl->setRefPicIdentifier( ii, 0, true, true, code );
        numIlrp++;
      }
    }

    if( !isInterLayerRefPic )
    {
    isLongTerm = false;
    if (sps->getLongTermRefsPresent())
    {
      READ_FLAG(code, "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");
      isLongTerm = (code == 1) ? false : true;
    }
    else
      isLongTerm = false;

    if (!isLongTerm)
    {
      READ_UVLC(code, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
      if( !sps->getUseWP() && !sps->getUseWPBiPred() )
      {
        code++;
      }
      int readValue = code;
      if (readValue > 0)
        READ_FLAG(code, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");
      else
        code = 1;
      readValue = (code) ? readValue : 0 - readValue; //true means positive delta POC -- false otherwise
      if (firstSTRP)
      {
        firstSTRP = false;
        prevDelta = deltaValue = readValue;
      }
      else
      {
        deltaValue = prevDelta + readValue;
        prevDelta = deltaValue;
      }

      rpl->setRefPicIdentifier( ii, deltaValue, isLongTerm, false, 0 );
      numStrp++;
    }
    else
    {
      if (!rpl->getLtrpInSliceHeaderFlag())
        READ_CODE(sps->getBitsForPOC(), code, "poc_lsb_lt[listIdx][rplsIdx][j]");
      rpl->setRefPicIdentifier( ii, code, isLongTerm, false, 0 );
      numLtrp++;
    }
    }
  }
  rpl->setNumberOfShorttermPictures(numStrp);
  rpl->setNumberOfLongtermPictures(numLtrp);
  rpl->setNumberOfInterLayerPictures( numIlrp );
}

void HLSyntaxReader::parsePPS( PPS* pcPPS, ParameterSetManager *parameterSetManager )
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif
  uint32_t  uiCode;

  int   iCode;

  READ_UVLC( uiCode, "pps_pic_parameter_set_id");
  CHECK(uiCode > 63, "PPS id exceeds boundary (63)");
  pcPPS->setPPSId (uiCode);

  READ_CODE(4, uiCode, "pps_seq_parameter_set_id");
  pcPPS->setSPSId (uiCode);

  READ_UVLC( uiCode, "pic_width_in_luma_samples" );          pcPPS->setPicWidthInLumaSamples( uiCode );
  READ_UVLC( uiCode, "pic_height_in_luma_samples" );         pcPPS->setPicHeightInLumaSamples( uiCode );

  READ_FLAG( uiCode, "conformance_window_flag" );
  if( uiCode != 0 )
  {
    Window &conf = pcPPS->getConformanceWindow();
    READ_UVLC( uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset( uiCode );
    READ_UVLC( uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset( uiCode );
    READ_UVLC( uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset( uiCode );
    READ_UVLC( uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode );
  }

  READ_FLAG( uiCode, "scaling_window_flag" );
  if( uiCode != 0 )
  {
    Window &scalingWindow = pcPPS->getScalingWindow();
    READ_UVLC( uiCode, "scaling_win_left_offset" );               scalingWindow.setWindowLeftOffset( uiCode );
    READ_UVLC( uiCode, "scaling_win_right_offset" );              scalingWindow.setWindowRightOffset( uiCode );
    READ_UVLC( uiCode, "scaling_win_top_offset" );                scalingWindow.setWindowTopOffset( uiCode );
    READ_UVLC( uiCode, "scaling_win_bottom_offset" );             scalingWindow.setWindowBottomOffset( uiCode );
  }

  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_FLAG(uiCode, "pps_subpic_id_signalling_present_flag");              pcPPS->setSubPicIdSignallingPresentFlag( uiCode != 0 );
  if( pcPPS->getSubPicIdSignallingPresentFlag() )
  {
    READ_UVLC( uiCode, "pps_num_subpics_minus1" );                         pcPPS->setNumSubPics( uiCode + 1 );
    CHECK( uiCode > MAX_NUM_SUB_PICS-1,  "Number of sub-pictures exceeds limit");

    READ_UVLC( uiCode, "pps_subpic_id_len_minus1" );                       pcPPS->setSubPicIdLen( uiCode + 1 );
    CHECK( uiCode > 15, "Invalid pps_subpic_id_len_minus1 signalled");

    for( int picIdx = 0; picIdx < pcPPS->getNumSubPics( ); picIdx++ )
    {
      READ_CODE( pcPPS->getSubPicIdLen( ), uiCode, "pps_subpic_id[i]" );   pcPPS->setSubPicId( picIdx, uiCode );
    }
  }
  else 
  {
    for( int picIdx = 0; picIdx < MAX_NUM_SUB_PICS; picIdx++ )
    {
      pcPPS->setSubPicId( picIdx, picIdx );
    }
  }


  READ_FLAG( uiCode, "no_pic_partition_flag" );                       pcPPS->setNoPicPartitionFlag( uiCode == 1 );
  if(!pcPPS->getNoPicPartitionFlag())
  {
    int colIdx, rowIdx;
    pcPPS->resetTileSliceInfo();

    // CTU size - required to match size in SPS
    READ_CODE(2, uiCode, "pps_log2_ctu_size_minus5");                 pcPPS->setLog2CtuSize(uiCode + 5);
    CHECK(uiCode > 2, "pps_log2_ctu_size_minus5 must be less than or equal to 2");
    
    // number of explicit tile columns/rows
    READ_UVLC( uiCode, "num_exp_tile_columns_minus1" );               pcPPS->setNumExpTileColumns( uiCode + 1 );
    READ_UVLC( uiCode, "num_exp_tile_rows_minus1" );                  pcPPS->setNumExpTileRows( uiCode + 1 );
    CHECK(pcPPS->getNumExpTileColumns() > MAX_TILE_COLS,              "Number of explicit tile columns exceeds valid range");
    CHECK(pcPPS->getNumExpTileRows() > MAX_TILE_ROWS,                 "Number of explicit tile rows exceeds valid range");
    
    // tile sizes
    for( colIdx = 0; colIdx < pcPPS->getNumExpTileColumns(); colIdx++ )
    {
      READ_UVLC( uiCode, "tile_column_width_minus1[i]" );             pcPPS->addTileColumnWidth( uiCode + 1 );
    }
    for( rowIdx = 0; rowIdx < pcPPS->getNumExpTileRows(); rowIdx++ )
    {
      READ_UVLC( uiCode, "tile_row_height_minus1[i]" );               pcPPS->addTileRowHeight( uiCode + 1 );
    }
    pcPPS->initTiles();
     
    // rectangular slice signalling
    READ_CODE(1, uiCode, "rect_slice_flag");                          pcPPS->setRectSliceFlag( uiCode == 1 );
    if (pcPPS->getRectSliceFlag()) 
    {
      READ_FLAG(uiCode, "single_slice_per_subpic_flag");            pcPPS->setSingleSlicePerSubPicFlag(uiCode == 1);
    }
    if (pcPPS->getRectSliceFlag() & !(pcPPS->getSingleSlicePerSubPicFlag()))
    {
      int32_t tileIdx = 0;

      READ_UVLC( uiCode, "num_slices_in_pic_minus1" );                pcPPS->setNumSlicesInPic( uiCode + 1 );
      CHECK(pcPPS->getNumSlicesInPic() > MAX_SLICES,                  "Number of slices in picture exceeds valid range");
      READ_CODE(1, uiCode, "tile_idx_delta_present_flag");            pcPPS->setTileIdxDeltaPresentFlag( uiCode == 1 );
      pcPPS->initRectSlices();
      
      // read rectangular slice parameters
      for( int i = 0; i < pcPPS->getNumSlicesInPic()-1; i++ )
      {
        pcPPS->setSliceTileIdx( i, tileIdx );

        // complete tiles within a single slice
        READ_UVLC( uiCode, "slice_width_in_tiles_minus1[i]" );        pcPPS->setSliceWidthInTiles ( i, uiCode + 1 );
#if JVET_Q0480_RASTER_RECT_SLICES
        if( pcPPS->getTileIdxDeltaPresentFlag() || ( (tileIdx % pcPPS->getNumTileColumns()) == 0 ) )
        {
          READ_UVLC( uiCode, "slice_height_in_tiles_minus1[i]" );     pcPPS->setSliceHeightInTiles( i, uiCode + 1 );
        }
        else 
        {
          pcPPS->setSliceHeightInTiles( i, pcPPS->getSliceHeightInTiles(i-1) );
        }
#else
        READ_UVLC( uiCode, "slice_height_in_tiles_minus1[i]" );       pcPPS->setSliceHeightInTiles( i, uiCode + 1 );
#endif

        // multiple slices within a single tile special case
        if( pcPPS->getSliceWidthInTiles( i ) == 1 && pcPPS->getSliceHeightInTiles( i ) == 1 ) 
        {
          READ_UVLC( uiCode, "num_slices_in_tile_minus1[i]" );        pcPPS->setNumSlicesInTile( i, uiCode + 1 );
          uint32_t numSlicesInTile = pcPPS->getNumSlicesInTile( i );
          for( int j = 0; j < numSlicesInTile-1; j++ )
          {
            READ_UVLC( uiCode, "slice_height_in_ctu_minus1[i]" );     pcPPS->setSliceHeightInCtu( i, uiCode + 1 );
            i++;
            pcPPS->setSliceWidthInTiles ( i, 1 );
            pcPPS->setSliceHeightInTiles( i, 1 );
            pcPPS->setNumSlicesInTile   ( i, numSlicesInTile );
            pcPPS->setSliceTileIdx      ( i, tileIdx );
          }
        }

        // tile index offset to start of next slice
        if( i < pcPPS->getNumSlicesInPic()-1 )
        {
          if( pcPPS->getTileIdxDeltaPresentFlag() ) 
          {
            int32_t  tileIdxDelta;
            READ_SVLC( tileIdxDelta, "tile_idx_delta[i]" );
            tileIdx += tileIdxDelta;
            CHECK( tileIdx < 0 || tileIdx >= pcPPS->getNumTiles(), "Invalid tile_idx_delta.");
          }
          else
          {
            tileIdx += pcPPS->getSliceWidthInTiles( i );
            if( tileIdx % pcPPS->getNumTileColumns() == 0)
            {
              tileIdx += (pcPPS->getSliceHeightInTiles( i ) - 1) * pcPPS->getNumTileColumns();
            }
          }
        }
      }
      pcPPS->setSliceTileIdx(pcPPS->getNumSlicesInPic()-1, tileIdx );
      
      // initialize mapping between rectangular slices and CTUs
      pcPPS->initRectSliceMap();
    }

    // loop filtering across slice/tile controls
    READ_CODE(1, uiCode, "loop_filter_across_tiles_enabled_flag");    pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode == 1 );
    READ_CODE(1, uiCode, "loop_filter_across_slices_enabled_flag");   pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode == 1 );
  }

  READ_FLAG(uiCode, "entropy_coding_sync_enabled_flag");       pcPPS->setEntropyCodingSyncEnabledFlag(uiCode == 1);
  READ_FLAG( uiCode,   "cabac_init_present_flag" );            pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC(uiCode, "num_ref_idx_l0_default_active_minus1");
  CHECK(uiCode > 14, "Invalid code read");
  pcPPS->setNumRefIdxL0DefaultActive(uiCode+1);

  READ_UVLC(uiCode, "num_ref_idx_l1_default_active_minus1");
  CHECK(uiCode > 14, "Invalid code read");
  pcPPS->setNumRefIdxL1DefaultActive(uiCode+1);

  READ_FLAG(uiCode, "rpl1_idx_present_flag");
  pcPPS->setRpl1IdxPresentFlag(uiCode);


  READ_SVLC(iCode, "init_qp_minus26" );                            pcPPS->setPicInitQPMinus26(iCode);
  READ_UVLC(uiCode, "log2_transform_skip_max_size_minus2");
  pcPPS->setLog2MaxTransformSkipBlockSize(uiCode + 2);

  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );            pcPPS->setUseDQP( uiCode ? true : false );
  READ_SVLC( iCode, "pps_cb_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cb, iCode);
  CHECK( pcPPS->getQpOffset(COMPONENT_Cb) < -12, "Invalid Cb QP offset" );
  CHECK( pcPPS->getQpOffset(COMPONENT_Cb) >  12, "Invalid Cb QP offset" );

  READ_SVLC( iCode, "pps_cr_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cr, iCode);
  CHECK( pcPPS->getQpOffset(COMPONENT_Cr) < -12, "Invalid Cr QP offset" );
  CHECK( pcPPS->getQpOffset(COMPONENT_Cr) >  12, "Invalid Cr QP offset" );

  READ_FLAG(uiCode, "pps_joint_cbcr_qp_offset_present_flag");
  pcPPS->setJointCbCrQpOffsetPresentFlag(uiCode ? true : false);

  if (pcPPS->getJointCbCrQpOffsetPresentFlag())
  {
    READ_SVLC(iCode, "pps_joint_cbcr_qp_offset_value");
  }
  else
  {
    iCode = 0;
  }
  pcPPS->setQpOffset(JOINT_CbCr, iCode);

  CHECK( pcPPS->getQpOffset(JOINT_CbCr) < -12, "Invalid CbCr QP offset" );
  CHECK( pcPPS->getQpOffset(JOINT_CbCr) >  12, "Invalid CbCr QP offset" );

  CHECK(MAX_NUM_COMPONENT>3, "Invalid maximal number of components");

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "pps_cu_chroma_qp_offset_enabled_flag");
  if (uiCode == 0)
  {
    pcPPS->clearChromaQpOffsetList();
  }
  else
  {
    uint32_t tableSizeMinus1 = 0;
    READ_UVLC(tableSizeMinus1, "chroma_qp_offset_list_len_minus1");
    CHECK(tableSizeMinus1 >= MAX_QP_OFFSET_LIST_SIZE, "Table size exceeds maximum");

    for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
    {
      int cbOffset;
      int crOffset;
      int jointCbCrOffset;
      READ_SVLC(cbOffset, "cb_qp_offset_list[i]");
      CHECK(cbOffset < -12 || cbOffset > 12, "Invalid chroma QP offset");
      READ_SVLC(crOffset, "cr_qp_offset_list[i]");
      CHECK(crOffset < -12 || crOffset > 12, "Invalid chroma QP offset");
      if (pcPPS->getJointCbCrQpOffsetPresentFlag())
      {
        READ_SVLC(jointCbCrOffset, "joint_cbcr_qp_offset_list[i]");
      }
      else
      {
        jointCbCrOffset = 0;
      }
      CHECK(jointCbCrOffset < -12 || jointCbCrOffset > 12, "Invalid chroma QP offset");
      // table uses +1 for index (see comment inside the function)
      pcPPS->setChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1, cbOffset, crOffset, jointCbCrOffset);
    }
    CHECK(pcPPS->getChromaQpOffsetListLen() != tableSizeMinus1 + 1, "Invalid chroma QP offset list length");
  }

  READ_FLAG( uiCode, "weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode==1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode==1 );

  READ_FLAG( uiCode, "deblocking_filter_control_present_flag" );       pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    READ_FLAG( uiCode, "deblocking_filter_override_enabled_flag" );    pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_deblocking_filter_disabled_flag" );        pcPPS->setPPSDeblockingFilterDisabledFlag(uiCode ? true : false );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      READ_SVLC ( iCode, "pps_beta_offset_div2" );                     pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      READ_SVLC ( iCode, "pps_tc_offset_div2" );                       pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
    }
  }
  READ_FLAG( uiCode, "constant_slice_header_params_enabled_flag"); pcPPS->setConstantSliceHeaderParamsEnabledFlag(uiCode);
  if ( pcPPS->getConstantSliceHeaderParamsEnabledFlag() ) {
    READ_CODE( 2, uiCode, "pps_dep_quant_enabled_idc");        pcPPS->setPPSDepQuantEnabledIdc(uiCode);
    READ_CODE( 2, uiCode, "pps_ref_pic_list_sps_idc[0]");      pcPPS->setPPSRefPicListSPSIdc0(uiCode);
    READ_CODE( 2, uiCode, "pps_ref_pic_list_sps_idc[1]");      pcPPS->setPPSRefPicListSPSIdc1(uiCode);
    READ_CODE( 2, uiCode, "pps_mvd_l1_zero_idc");              pcPPS->setPPSMvdL1ZeroIdc(uiCode);
    READ_CODE( 2, uiCode, "pps_collocated_from_l0_idc");       pcPPS->setPPSCollocatedFromL0Idc(uiCode);
    READ_UVLC( uiCode, "pps_six_minus_max_num_merge_cand_plus1"); pcPPS->setPPSSixMinusMaxNumMergeCandPlus1(uiCode);
    READ_UVLC( uiCode, "pps_max_num_merge_cand_minus_max_num_triangle_cand_plus1");pcPPS->setPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1(uiCode);
  }
  else
  {
    pcPPS->setPPSDepQuantEnabledIdc(0);
    pcPPS->setPPSRefPicListSPSIdc0(0);
    pcPPS->setPPSRefPicListSPSIdc1(0);
    pcPPS->setPPSMvdL1ZeroIdc(0);
    pcPPS->setPPSCollocatedFromL0Idc(0);
    pcPPS->setPPSSixMinusMaxNumMergeCandPlus1(0);
    pcPPS->setPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1(0);
  }


  READ_FLAG( uiCode, "picture_header_extension_present_flag");
  pcPPS->setPictureHeaderExtensionPresentFlag(uiCode);
  READ_FLAG( uiCode, "slice_header_extension_present_flag");
  pcPPS->setSliceHeaderExtensionPresentFlag(uiCode);


  READ_FLAG( uiCode, "pps_extension_present_flag");
  if (uiCode)
  {
#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const char *syntaxStrings[]={ "pps_range_extension_flag",
      "pps_multilayer_extension_flag",
      "pps_extension_6bits[0]",
      "pps_extension_6bits[1]",
      "pps_extension_6bits[2]",
      "pps_extension_6bits[3]",
      "pps_extension_6bits[4]",
      "pps_extension_6bits[5]" };
#endif

    bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS];
    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      pps_extension_flags[i] = uiCode!=0;
    }

    bool bSkipTrailingExtensionBits=false;
    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
        case PPS_EXT__REXT:
        {
          PPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();
          CHECK(bSkipTrailingExtensionBits, "Invalid state");

          READ_FLAG( uiCode, "cross_component_prediction_enabled_flag");
          ppsRangeExtension.setCrossComponentPredictionEnabledFlag(uiCode != 0);

          READ_UVLC( uiCode, "log2_sao_offset_scale_luma");
          ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_LUMA, uiCode);
          READ_UVLC( uiCode, "log2_sao_offset_scale_chroma");
          ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA, uiCode);
        }
        break;
        default:
          bSkipTrailingExtensionBits=true;
          break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "pps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseAPS( APS* aps )
{
#if ENABLE_TRACING
  xTraceAPSHeader();
#endif

  uint32_t  code;

  READ_CODE(5, code, "adaptation_parameter_set_id");
  aps->setAPSId(code);

  READ_CODE(3, code, "aps_params_type");
  aps->setAPSType( ApsType(code) );
  if( code == ALF_APS )
  {
    parseAlfAps( aps );
  }
  else if( code == LMCS_APS )
  {
    parseLmcsAps( aps );
  }
  else if( code == SCALING_LIST_APS )
  {
    parseScalingListAps( aps );
  }
  READ_FLAG(code, "aps_extension_flag");
  if (code)
  {
    while (xMoreRbspData())
    {
      READ_FLAG(code, "aps_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseAlfAps( APS* aps )
{
  uint32_t  code;

  AlfParam param = aps->getAlfAPSParam();
  param.reset();
  param.enabledFlag[COMPONENT_Y] = param.enabledFlag[COMPONENT_Cb] = param.enabledFlag[COMPONENT_Cr] = true;
  READ_FLAG(code, "alf_luma_new_filter");
  param.newFilterFlag[CHANNEL_TYPE_LUMA] = code;
  READ_FLAG(code, "alf_chroma_new_filter");
  param.newFilterFlag[CHANNEL_TYPE_CHROMA] = code;

  CHECK(param.newFilterFlag[CHANNEL_TYPE_LUMA] == 0 && param.newFilterFlag[CHANNEL_TYPE_CHROMA] == 0,
    "bitstream conformance error, alf_luma_filter_signal_flag and alf_chroma_filter_signal_flag shall not equal to zero at the same time");

  if (param.newFilterFlag[CHANNEL_TYPE_LUMA])
  {
    READ_FLAG(code, "alf_luma_clip");
    param.nonLinearFlag[CHANNEL_TYPE_LUMA][0] = code ? true : false;
    READ_UVLC(code, "alf_luma_num_filters_signalled_minus1");
    param.numLumaFilters = code + 1;
    if (param.numLumaFilters > 1)
    {
      const int length =  ceilLog2(param.numLumaFilters);
      for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
      {
        READ_CODE(length, code, "alf_luma_coeff_delta_idx");
        param.filterCoeffDeltaIdx[i] = code;
      }
    }
    else
    {
      memset(param.filterCoeffDeltaIdx, 0, sizeof(param.filterCoeffDeltaIdx));
    }
    alfFilter( param, false, 0 );
  }
  if (param.newFilterFlag[CHANNEL_TYPE_CHROMA])
  {
    if( MAX_NUM_ALF_ALTERNATIVES_CHROMA > 1 )
      READ_UVLC( code, "alf_chroma_num_alts_minus1" );
    else
      code = 0;

    param.numAlternativesChroma = code + 1;

    for( int altIdx=0; altIdx < param.numAlternativesChroma; ++altIdx )
    {
      READ_FLAG( code, "alf_nonlinear_enable_flag_chroma" );
      param.nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx] = code ? true : false;
      alfFilter( param, true, altIdx );
    }
  }
  aps->setAlfAPSParam(param);
}

void HLSyntaxReader::parseLmcsAps( APS* aps )
{
  uint32_t  code;

  SliceReshapeInfo& info = aps->getReshaperAPSInfo();
  memset(info.reshaperModelBinCWDelta, 0, PIC_CODE_CW_BINS * sizeof(int));
  READ_UVLC(code, "lmcs_min_bin_idx");                             info.reshaperModelMinBinIdx = code;
  READ_UVLC(code, "lmcs_delta_max_bin_idx");                       info.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1 - code;
  READ_UVLC(code, "lmcs_delta_cw_prec_minus1");                    info.maxNbitsNeededDeltaCW = code + 1;
  assert(info.maxNbitsNeededDeltaCW > 0);
  for (uint32_t i = info.reshaperModelMinBinIdx; i <= info.reshaperModelMaxBinIdx; i++)
  {
    READ_CODE(info.maxNbitsNeededDeltaCW, code, "lmcs_delta_abs_cw[ i ]");
    int absCW = code;
    if (absCW > 0)
    {
      READ_CODE(1, code, "lmcs_delta_sign_cw_flag[ i ]");
    }
    int signCW = code;
    info.reshaperModelBinCWDelta[i] = (1 - 2 * signCW) * absCW;
  }
  READ_CODE(3, code, "lmcs_delta_abs_crs");
  int absCW = code;
  if (absCW > 0)
  {
    READ_CODE(1, code, "lmcs_delta_sign_crs_flag");
  }
  int signCW = code;
  info.chrResScalingOffset = (1 - 2 * signCW) * absCW;

  aps->setReshaperAPSInfo(info);
}

void HLSyntaxReader::parseScalingListAps( APS* aps )
{
  ScalingList& info = aps->getScalingList();
  parseScalingList( &info );
}

void  HLSyntaxReader::parseVUI(VUI* pcVUI, SPS *pcSPS)
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif


  uint32_t  symbol;

  READ_FLAG( symbol, "aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(symbol);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    READ_FLAG( symbol, "aspect_ratio_constant_flag");           pcVUI->setAspectRatioConstantFlag(symbol);
    READ_CODE(8, symbol, "aspect_ratio_idc");                         pcVUI->setAspectRatioIdc(symbol);
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      READ_CODE(16, symbol, "sar_width");                             pcVUI->setSarWidth(symbol);
      READ_CODE(16, symbol, "sar_height");                            pcVUI->setSarHeight(symbol);
    }
  }

  READ_FLAG(   symbol, "colour_description_present_flag");          pcVUI->setColourDescriptionPresentFlag(symbol);
  if (pcVUI->getColourDescriptionPresentFlag())
  {
    READ_CODE(8, symbol, "colour_primaries");                       pcVUI->setColourPrimaries(symbol);
    READ_CODE(8, symbol, "transfer_characteristics");               pcVUI->setTransferCharacteristics(symbol);
    READ_CODE(8, symbol, "matrix_coeffs");                          pcVUI->setMatrixCoefficients(symbol);
    READ_FLAG(   symbol, "video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(symbol);
  }

  READ_FLAG(     symbol, "field_seq_flag");                           pcVUI->setFieldSeqFlag(symbol);

  READ_FLAG(     symbol, "chroma_loc_info_present_flag");             pcVUI->setChromaLocInfoPresentFlag(symbol);
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    if(pcVUI->getFieldSeqFlag())
    {
      READ_UVLC(   symbol, "chroma_sample_loc_type_top_field" );        pcVUI->setChromaSampleLocTypeTopField(symbol);
      READ_UVLC(   symbol, "chroma_sample_loc_type_bottom_field" );     pcVUI->setChromaSampleLocTypeBottomField(symbol);
    }
    else
    {
      READ_UVLC(   symbol, "chroma_sample_loc_type" );        pcVUI->setChromaSampleLocType(symbol);
    }
  }

  READ_FLAG(     symbol, "overscan_info_present_flag");               pcVUI->setOverscanInfoPresentFlag(symbol);
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    READ_FLAG(   symbol, "overscan_appropriate_flag");                pcVUI->setOverscanAppropriateFlag(symbol);
  }
}

void HLSyntaxReader::parseHrdParameters(HRDParameters *hrd, uint32_t firstSubLayer, uint32_t maxNumSubLayersMinus1)
{
  uint32_t  symbol;
  READ_FLAG( symbol, "general_nal_hrd_parameters_present_flag" );           hrd->setNalHrdParametersPresentFlag( symbol == 1 ? true : false );
  READ_FLAG( symbol, "general_vcl_hrd_parameters_present_flag" );           hrd->setVclHrdParametersPresentFlag( symbol == 1 ? true : false );
  READ_FLAG( symbol, "general_decoding_unit_hrd_params_present_flag" );           hrd->setGeneralDecodingUnitHrdParamsPresentFlag( symbol == 1 ? true : false );

  if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
  {
    READ_CODE( 8, symbol, "tick_divisor_minus2" );                        hrd->setTickDivisorMinus2( symbol );
  }
  READ_CODE( 4, symbol, "bit_rate_scale" );                       hrd->setBitRateScale( symbol );
  READ_CODE( 4, symbol, "cpb_size_scale" );                       hrd->setCpbSizeScale( symbol );
  if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
  {
    READ_CODE( 4, symbol, "cpb_size_du_scale" );                  hrd->setCpbSizeDuScale( symbol );
  }

  for( int i = firstSubLayer; i <= maxNumSubLayersMinus1; i ++ )
  {
    READ_FLAG( symbol, "fixed_pic_rate_general_flag" );                     hrd->setFixedPicRateFlag( i, symbol == 1 ? true : false  );
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      READ_FLAG( symbol, "fixed_pic_rate_within_cvs_flag" );                hrd->setFixedPicRateWithinCvsFlag( i, symbol == 1 ? true : false  );
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag( i, true );
    }

    hrd->setLowDelayHrdFlag( i, false ); // Inferred to be 0 when not present
    hrd->setCpbCntMinus1   ( i, 0 );     // Inferred to be 0 when not present

    if( hrd->getFixedPicRateWithinCvsFlag( i ) )
    {
      READ_UVLC( symbol, "elemental_duration_in_tc_minus1" );             hrd->setPicDurationInTcMinus1( i, symbol );
    }
    else
    {
      READ_FLAG( symbol, "low_delay_hrd_flag" );                      hrd->setLowDelayHrdFlag( i, symbol == 1 ? true : false  );
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      READ_UVLC( symbol, "cpb_cnt_minus1" );                          hrd->setCpbCntMinus1( i, symbol );
    }

    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( int j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          READ_UVLC( symbol, "bit_rate_value_minus1" );             hrd->setBitRateValueMinus1( i, j, nalOrVcl, symbol );
          READ_UVLC( symbol, "cpb_size_value_minus1" );             hrd->setCpbSizeValueMinus1( i, j, nalOrVcl, symbol );
          READ_FLAG( symbol, "cbr_flag" );                          hrd->setCbrFlag( i, j, nalOrVcl, symbol == 1 ? true : false  );
        }
      }
    }
  }
  for (int i = 0; i < firstSubLayer; i++)
  {
    for (int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++)
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for (int j = 0; j <= (hrd->getCpbCntMinus1(i)); j++)
        {
          uint32_t bitRate = hrd->getBitRateValueMinus1(maxNumSubLayersMinus1, j, nalOrVcl);
          hrd->setBitRateValueMinus1(i, j, nalOrVcl, bitRate);
          uint32_t cpbSize = hrd->getCpbSizeValueMinus1(maxNumSubLayersMinus1, j, nalOrVcl);
          hrd->setCpbSizeValueMinus1(i, j, nalOrVcl, cpbSize);
          bool flag = hrd->getCbrFlag(maxNumSubLayersMinus1, j, nalOrVcl);
          hrd->setCbrFlag(i, j, nalOrVcl, flag);
        }
      }
    }
  }
}


void HLSyntaxReader::parseSPS(SPS* pcSPS)
{
  uint32_t  uiCode;

#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif
  READ_CODE( 4,  uiCode, "sps_decoding_parameter_set_id");       pcSPS->setDecodingParameterSetId( uiCode );
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id" );      pcSPS->setVPSId( uiCode );
  READ_CODE(3, uiCode, "sps_max_sub_layers_minus1");          pcSPS->setMaxTLayers   (uiCode + 1);
  CHECK(uiCode > 6, "Invalid maximum number of T-layer signalled");
  READ_CODE(5, uiCode, "sps_reserved_zero_5bits");
  CHECK(uiCode != 0, "sps_reserved_zero_5bits not equal to zero");

  parseProfileTierLevel(pcSPS->getProfileTierLevel(), pcSPS->getMaxTLayers() - 1);

  READ_FLAG(uiCode, "gdr_enabled_flag");
  pcSPS->setGDREnabledFlag(uiCode);

  READ_CODE(4, uiCode, "sps_seq_parameter_set_id");              pcSPS->setSPSId(uiCode);

  READ_CODE(2, uiCode, "chroma_format_idc");                     pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG(     uiCode, "separate_colour_plane_flag");        CHECK(uiCode != 0, "Invalid code");
    pcSPS->setSeparateColourPlaneFlag( uiCode != 0 );
  }

  READ_FLAG( uiCode, "ref_pic_resampling_enabled_flag" );        pcSPS->setRprEnabledFlag( uiCode );

  READ_UVLC( uiCode, "pic_width_max_in_luma_samples" );          pcSPS->setMaxPicWidthInLumaSamples( uiCode );
  READ_UVLC( uiCode, "pic_height_max_in_luma_samples" );         pcSPS->setMaxPicHeightInLumaSamples( uiCode );

  READ_CODE(2, uiCode, "sps_log2_ctu_size_minus5");              pcSPS->setCTUSize(1 << (uiCode + 5));
  CHECK(uiCode > 2, "sps_log2_ctu_size_minus5 must be less than or equal to 2");
  unsigned ctbLog2SizeY = uiCode + 5;
  pcSPS->setMaxCodingDepth(uiCode+3);
  pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode+3);
  pcSPS->setMaxCUWidth(pcSPS->getCTUSize());
  pcSPS->setMaxCUHeight(pcSPS->getCTUSize());
  READ_FLAG( uiCode, "subpics_present_flag" );                   pcSPS->setSubPicPresentFlag(uiCode);

  if (pcSPS->getSubPicPresentFlag()) 
  {
    READ_CODE(8, uiCode, "sps_num_subpics_minus1"); pcSPS->setNumSubPics(uiCode + 1);
    for (int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++)
    {
      READ_CODE(std::max(1, ceilLog2(((pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1) >> floorLog2(pcSPS->getCTUSize())))), uiCode, "subpic_ctu_top_left_x[ i ]");
      pcSPS->setSubPicCtuTopLeftX(picIdx, uiCode);
      READ_CODE(std::max(1, ceilLog2(((pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1) >> floorLog2(pcSPS->getCTUSize())))), uiCode, "subpic_ctu_top_left_y[ i ]");
      pcSPS->setSubPicCtuTopLeftY(picIdx, uiCode);
      READ_CODE(std::max(1, ceilLog2(((pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1) >> floorLog2(pcSPS->getCTUSize())))), uiCode, "subpic_width_minus1[ i ]");
      pcSPS->setSubPicWidth(picIdx, uiCode + 1);
      READ_CODE(std::max(1, ceilLog2(((pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1) >> floorLog2(pcSPS->getCTUSize())))), uiCode, "subpic_height_minus1[ i ]");
      pcSPS->setSubPicHeight(picIdx, uiCode + 1);
      READ_FLAG(uiCode, "subpic_treated_as_pic_flag[ i ]");
      pcSPS->setSubPicTreatedAsPicFlag(picIdx, uiCode);
      READ_FLAG(uiCode, "loop_filter_across_subpic_enabled_flag[ i ]");
      pcSPS->setLoopFilterAcrossSubpicEnabledFlag(picIdx, uiCode);
    }
  }
  READ_FLAG(uiCode, "sps_subpic_id_present_flag");                           pcSPS->setSubPicIdPresentFlag( uiCode != 0 );
  if( pcSPS->getSubPicIdPresentFlag() )
  {
    READ_FLAG(uiCode, "sps_subpic_id_signalling_present_flag");              pcSPS->setSubPicIdSignallingPresentFlag( uiCode != 0 );
    if( pcSPS->getSubPicIdSignallingPresentFlag() )
    {
      READ_UVLC( uiCode, "sps_subpic_id_len_minus1" );                       pcSPS->setSubPicIdLen( uiCode + 1 );
      CHECK( uiCode > 15, "Invalid sps_subpic_id_len_minus1 signalled");
      for( int picIdx = 0; picIdx < pcSPS->getNumSubPics( ); picIdx++ )
      {
        READ_CODE( pcSPS->getSubPicIdLen( ), uiCode, "sps_subpic_id[i]" );   pcSPS->setSubPicId( picIdx, uiCode );
      }
    }
  }
  if( pcSPS->getSubPicIdPresentFlag() == false || pcSPS->getSubPicIdSignallingPresentFlag() == false )
  {
    for( int picIdx = 0; picIdx < pcSPS->getNumSubPics( ); picIdx++ )
    {
      pcSPS->setSubPicId( picIdx, picIdx );
    }
  }

  READ_UVLC(     uiCode, "bit_depth_minus8" );
  CHECK(uiCode > 8, "Invalid bit depth signalled");
  pcSPS->setBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);
  pcSPS->setBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (int) (6*uiCode) );
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA, (int) (6*uiCode) );

  READ_UVLC(     uiCode, "min_qp_prime_ts_minus4" );
  pcSPS->setMinQpPrimeTsMinus4(CHANNEL_TYPE_LUMA, uiCode);
  CHECK(uiCode > 48, "Invalid min_qp_prime_ts_minus4 signalled");
  pcSPS->setMinQpPrimeTsMinus4(CHANNEL_TYPE_CHROMA, uiCode);
  READ_FLAG( uiCode, "sps_weighted_pred_flag" );                    pcSPS->setUseWP( uiCode ? true : false );
  READ_FLAG( uiCode, "sps_weighted_bipred_flag" );                  pcSPS->setUseWPBiPred( uiCode ? true : false );

  READ_CODE(4, uiCode, "log2_max_pic_order_cnt_lsb_minus4");     pcSPS->setBitsForPOC( 4 + uiCode );
  CHECK(uiCode > 12, "Invalid code");
  // KJS: Marakech decision: sub-layers added back
  uint32_t subLayerOrderingInfoPresentFlag;
  if (pcSPS->getMaxTLayers() > 1)
  {
    READ_FLAG(subLayerOrderingInfoPresentFlag, "sps_sub_layer_ordering_info_present_flag");
  }
  else
  {
    subLayerOrderingInfoPresentFlag = 0;
  }

  for(uint32_t i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC ( uiCode, "sps_max_dec_pic_buffering_minus1[i]");
    pcSPS->setMaxDecPicBuffering( uiCode + 1, i);
    READ_UVLC ( uiCode, "sps_max_num_reorder_pics[i]" );
    pcSPS->setNumReorderPics(uiCode, i);
    READ_UVLC ( uiCode, "sps_max_latency_increase_plus1[i]");
    pcSPS->setMaxLatencyIncreasePlus1( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcSPS->getMaxTLayers()-1; i++)
      {
        pcSPS->setMaxDecPicBuffering(pcSPS->getMaxDecPicBuffering(0), i);
        pcSPS->setNumReorderPics(pcSPS->getNumReorderPics(0), i);
        pcSPS->setMaxLatencyIncreasePlus1(pcSPS->getMaxLatencyIncreasePlus1(0), i);
      }
      break;
    }
  }

  READ_FLAG(uiCode, "long_term_ref_pics_flag");          pcSPS->setLongTermRefsPresent(uiCode);
  READ_FLAG( uiCode, "inter_layer_ref_pics_present_flag" );  pcSPS->setInterLayerPresentFlag( uiCode );
  READ_FLAG( uiCode, "sps_idr_rpl_present_flag" );       pcSPS->setIDRRefParamListPresent( (bool) uiCode );
  READ_FLAG(uiCode, "rpl1_copy_from_rpl0_flag");
  pcSPS->setRPL1CopyFromRPL0Flag(uiCode);

  //Read candidate for List0
  READ_UVLC(uiCode, "num_ref_pic_lists_in_sps[0]");
  uint32_t numberOfRPL = uiCode;
  pcSPS->createRPLList0(numberOfRPL);
  RPLList* rplList = pcSPS->getRPLList0();
  ReferencePictureList* rpl;
  for (uint32_t ii = 0; ii < numberOfRPL; ii++)
  {
    rpl = rplList->getReferencePictureList(ii);
    parseRefPicList(pcSPS, rpl);
  }

  //Read candidate for List1
  if (!pcSPS->getRPL1CopyFromRPL0Flag())
  {
    READ_UVLC(uiCode, "num_ref_pic_lists_in_sps[1]");
    numberOfRPL = uiCode;
    pcSPS->createRPLList1(numberOfRPL);
    rplList = pcSPS->getRPLList1();
    for (uint32_t ii = 0; ii < numberOfRPL; ii++)
    {
      rpl = rplList->getReferencePictureList(ii);
      parseRefPicList(pcSPS, rpl);
    }
  }
  else
  {
    numberOfRPL = pcSPS->getNumRPL0();
    pcSPS->createRPLList1(numberOfRPL);
    RPLList* rplListSource = pcSPS->getRPLList0();
    RPLList* rplListDest = pcSPS->getRPLList1();
    for (uint32_t ii = 0; ii < numberOfRPL; ii++)
      copyRefPicList(pcSPS, rplListSource->getReferencePictureList(ii), rplListDest->getReferencePictureList(ii));
  }

  unsigned  minQT[3] = { 0, 0, 0 };
  unsigned  maxBTD[3] = { 0, 0, 0 };

  unsigned  maxBTSize[3] = { 0, 0, 0 };
  unsigned  maxTTSize[3] = { 0, 0, 0 };
  if( pcSPS->getChromaFormatIdc() != CHROMA_400 ) 
  {
    READ_FLAG(uiCode, "qtbtt_dual_tree_intra_flag");           pcSPS->setUseDualITree(uiCode);
  }
  else
  {
    pcSPS->setUseDualITree(0);
  }

  READ_UVLC(uiCode, "log2_min_luma_coding_block_size_minus2");
  int log2MinCUSize = uiCode + 2;
  pcSPS->setLog2MinCodingBlockSize(log2MinCUSize);
  CHECK(uiCode > ctbLog2SizeY - 2, "Invalid log2_min_luma_coding_block_size_minus2 signalled");

  CHECK(log2MinCUSize > std::min(6, (int)(ctbLog2SizeY)), "log2_min_luma_coding_block_size_minus2 shall be in the range of 0 to min (4, log2_ctu_size - 2)");
  CHECK( ( pcSPS->getMaxPicWidthInLumaSamples() % ( std::max( 8, int( pcSPS->getMaxCUWidth() >> ( pcSPS->getMaxCodingDepth() - 1 ) ) ) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( pcSPS->getMaxPicHeightInLumaSamples() % ( std::max( 8, int( pcSPS->getMaxCUHeight() >> ( pcSPS->getMaxCodingDepth() - 1 ) ) ) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );

  READ_FLAG(uiCode, "partition_constraints_override_enabled_flag"); pcSPS->setSplitConsOverrideEnabledFlag(uiCode);
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
  unsigned minQtLog2SizeIntraY = uiCode + pcSPS->getLog2MinCodingBlockSize();
  minQT[0] = 1 << minQtLog2SizeIntraY;
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_inter_slice");
  unsigned minQtLog2SizeInterY = uiCode + pcSPS->getLog2MinCodingBlockSize();
  minQT[1] = 1 << minQtLog2SizeInterY;
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_inter_slice");     maxBTD[1] = uiCode;
  CHECK(uiCode > 2*(ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_luma");     maxBTD[0] = uiCode;
  CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  maxTTSize[0] = maxBTSize[0] = minQT[0];
  if (maxBTD[0] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_luma");     maxBTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_luma");     maxTTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
  }
  maxTTSize[1] = maxBTSize[1] = minQT[1];
  if (maxBTD[1] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_inter_slice");     maxBTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_inter_slice");     maxTTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
  }
  if (pcSPS->getUseDualITree())
  {
    READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_chroma"); minQT[2] = 1 << (uiCode + pcSPS->getLog2MinCodingBlockSize());
    READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_chroma"); maxBTD[2] = uiCode;
    CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
    maxTTSize[2] = maxBTSize[2] = minQT[2];
    if (maxBTD[2] != 0)
    {
      READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_chroma");       maxBTSize[2] <<= uiCode;
      READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_chroma");       maxTTSize[2] <<= uiCode;
    }
}

  pcSPS->setMinQTSizes(minQT);
  pcSPS->setMaxMTTHierarchyDepth(maxBTD[1], maxBTD[0], maxBTD[2]);
  pcSPS->setMaxBTSize(maxBTSize[1], maxBTSize[0], maxBTSize[2]);
  pcSPS->setMaxTTSize(maxTTSize[1], maxTTSize[0], maxTTSize[2]);


  READ_FLAG( uiCode, "sps_max_luma_transform_size_64_flag");        pcSPS->setLog2MaxTbSize( (uiCode ? 1 : 0) + 5 );

  READ_FLAG(uiCode, "sps_joint_cbcr_enabled_flag");                pcSPS->setJointCbCrEnabledFlag(uiCode ? true : false);
  if (pcSPS->getChromaFormatIdc() != CHROMA_400)
  {
    ChromaQpMappingTableParams chromaQpMappingTableParams;
    READ_FLAG(uiCode, "same_qp_table_for_chroma");        chromaQpMappingTableParams.setSameCQPTableForAllChromaFlag(uiCode);
    int numQpTables = chromaQpMappingTableParams.getSameCQPTableForAllChromaFlag() ? 1 : (pcSPS->getJointCbCrEnabledFlag() ? 3 : 2);
    chromaQpMappingTableParams.setNumQpTables(numQpTables);
    for (int i = 0; i < numQpTables; i++)
    {
      int32_t qpTableStart = 0;
      READ_SVLC(qpTableStart, "qp_table_starts_minus26"); chromaQpMappingTableParams.setQpTableStartMinus26(i, qpTableStart);
      READ_UVLC(uiCode, "num_points_in_qp_table_minus1"); chromaQpMappingTableParams.setNumPtsInCQPTableMinus1(i,uiCode);
      std::vector<int> deltaQpInValMinus1(chromaQpMappingTableParams.getNumPtsInCQPTableMinus1(i) + 1);
      std::vector<int> deltaQpOutVal(chromaQpMappingTableParams.getNumPtsInCQPTableMinus1(i) + 1);
      for (int j = 0; j <= chromaQpMappingTableParams.getNumPtsInCQPTableMinus1(i); j++)
      {
        READ_UVLC(uiCode, "delta_qp_in_val_minus1");  deltaQpInValMinus1[j] = uiCode;
        READ_UVLC(uiCode, "delta_qp_diff_val");
        deltaQpOutVal[j] = uiCode ^ deltaQpInValMinus1[j];
      }
      chromaQpMappingTableParams.setDeltaQpInValMinus1(i, deltaQpInValMinus1);
      chromaQpMappingTableParams.setDeltaQpOutVal(i, deltaQpOutVal);
    }
    pcSPS->setChromaQpMappingTableFromParams(chromaQpMappingTableParams, pcSPS->getQpBDOffset(CHANNEL_TYPE_CHROMA));
    pcSPS->derivedChromaQPMappingTables();
  }


  READ_FLAG( uiCode, "sps_sao_enabled_flag" );                      pcSPS->setSAOEnabledFlag ( uiCode ? true : false );
  READ_FLAG( uiCode, "sps_alf_enabled_flag" );                      pcSPS->setALFEnabledFlag ( uiCode ? true : false );

  READ_FLAG(uiCode, "sps_transform_skip_enabled_flag"); pcSPS->setTransformSkipEnabledFlag(uiCode ? true : false);
  if (pcSPS->getTransformSkipEnabledFlag())
  {
      READ_FLAG(uiCode, "sps_bdpcm_enabled_flag");
      if (uiCode && pcSPS->getChromaFormatIdc() == CHROMA_444 )
      {
          READ_FLAG(uiCode, "sps_bdpcm_enabled_chroma_flag");
          uiCode++;
      }
      pcSPS->setBDPCMEnabled(uiCode);
  }

  READ_FLAG(uiCode, "sps_ref_wraparound_enabled_flag");                  pcSPS->setWrapAroundEnabledFlag( uiCode ? true : false );

  if (pcSPS->getWrapAroundEnabledFlag())
  {
    READ_UVLC(uiCode, "sps_ref_wraparound_offset_minus1");               pcSPS->setWrapAroundOffset( (uiCode+1)*(1 <<  pcSPS->getLog2MinCodingBlockSize()));
  }

  READ_FLAG( uiCode, "sps_temporal_mvp_enabled_flag" );                  pcSPS->setSPSTemporalMVPEnabledFlag(uiCode);

  if ( pcSPS->getSPSTemporalMVPEnabledFlag() )
  {
    READ_FLAG( uiCode,    "sps_sbtmvp_enabled_flag" );                   pcSPS->setSBTMVPEnabledFlag      ( uiCode != 0 );
  }
  else
  {
    pcSPS->setSBTMVPEnabledFlag(false);
  }

  READ_FLAG( uiCode,  "sps_amvr_enabled_flag" );                     pcSPS->setAMVREnabledFlag ( uiCode != 0 );

  READ_FLAG( uiCode, "sps_bdof_enabled_flag" );                      pcSPS->setBDOFEnabledFlag ( uiCode != 0 );
  if (pcSPS->getBDOFEnabledFlag())
  {
    READ_FLAG(uiCode, "sps_bdof_pic_present_flag");                 pcSPS->setBdofControlPresentFlag( uiCode != 0 );
  }
  else {
    pcSPS->setBdofControlPresentFlag( false );
  }
  READ_FLAG(uiCode, "sps_smvd_enabled_flag");                       pcSPS->setUseSMVD( uiCode != 0 );
  READ_FLAG(uiCode, "sps_dmvr_enabled_flag");                        pcSPS->setUseDMVR(uiCode != 0);
  if (pcSPS->getUseDMVR())
  {
    READ_FLAG(uiCode, "sps_dmvr_pic_present_flag");                 pcSPS->setDmvrControlPresentFlag( uiCode != 0 );
  }
  else {
    pcSPS->setDmvrControlPresentFlag( false );
  }
  READ_FLAG(uiCode, "sps_mmvd_enabled_flag");                        pcSPS->setUseMMVD(uiCode != 0);
  READ_FLAG(uiCode, "sps_isp_enabled_flag");                        pcSPS->setUseISP( uiCode != 0 );
  READ_FLAG(uiCode, "sps_mrl_enabled_flag");                        pcSPS->setUseMRL( uiCode != 0 );
  READ_FLAG(uiCode, "sps_mip_enabled_flag");                        pcSPS->setUseMIP( uiCode != 0 );
  if( pcSPS->getChromaFormatIdc() != CHROMA_400) 
  {
    READ_FLAG( uiCode, "sps_cclm_enabled_flag" );                   pcSPS->setUseLMChroma( uiCode != 0 );
  }
  else
  {
    pcSPS->setUseLMChroma(0);
  }
  if( pcSPS->getChromaFormatIdc() == CHROMA_420 )
  {
    READ_FLAG( uiCode, "sps_chroma_horizontal_collocated_flag" );   pcSPS->setHorCollocatedChromaFlag( uiCode != 0 );
    READ_FLAG( uiCode, "sps_chroma_vertical_collocated_flag" );     pcSPS->setVerCollocatedChromaFlag( uiCode != 0 );
  }

  READ_FLAG( uiCode,    "sps_mts_enabled_flag" );                       pcSPS->setUseMTS                 ( uiCode != 0 );
  if ( pcSPS->getUseMTS() )
  {
    READ_FLAG( uiCode,    "sps_explicit_mts_intra_enabled_flag" );               pcSPS->setUseIntraMTS            ( uiCode != 0 );
    READ_FLAG( uiCode,    "sps_explicit_mts_inter_enabled_flag" );               pcSPS->setUseInterMTS            ( uiCode != 0 );
  }
  READ_FLAG(uiCode, "sps_sbt_enabled_flag");                        pcSPS->setUseSBT                 ( uiCode != 0 );
  READ_FLAG( uiCode,    "sps_affine_enabled_flag" );                            pcSPS->setUseAffine              ( uiCode != 0 );
  if ( pcSPS->getUseAffine() )
  {
    READ_FLAG( uiCode,  "sps_affine_type_flag" );                       pcSPS->setUseAffineType          ( uiCode != 0 );
    READ_FLAG( uiCode, "sps_affine_amvr_enabled_flag" );            pcSPS->setAffineAmvrEnabledFlag  ( uiCode != 0 );
    READ_FLAG( uiCode, "sps_affine_prof_enabled_flag" );            pcSPS->setUsePROF                ( uiCode != 0 );
    if (pcSPS->getUsePROF())
    {
      READ_FLAG(uiCode, "sps_prof_pic_present_flag");               pcSPS->setProfControlPresentFlag ( uiCode != 0 );
    }
    else {
      pcSPS->setProfControlPresentFlag( false );
    }
  }
  if (pcSPS->getChromaFormatIdc() == CHROMA_444)
  {
    READ_FLAG(uiCode, "sps_act_enabled_flag");                                  pcSPS->setUseColorTrans(uiCode != 0);
  }
  else
  {
    pcSPS->setUseColorTrans(false);
  }
  if (pcSPS->getChromaFormatIdc() == CHROMA_444)
  {
    READ_FLAG( uiCode,  "sps_palette_enabled_flag");                                pcSPS->setPLTMode                ( uiCode != 0 );
  }
  else
  {
    pcSPS->setPLTMode(false);
  }
  READ_FLAG( uiCode,    "sps_bcw_enabled_flag" );                   pcSPS->setUseBcw( uiCode != 0 );
  READ_FLAG(uiCode, "sps_ibc_enabled_flag");                                    pcSPS->setIBCFlag(uiCode);
  // KJS: sps_ciip_enabled_flag
  READ_FLAG( uiCode,     "sps_ciip_enabled_flag" );                           pcSPS->setUseCiip             ( uiCode != 0 );

  if ( pcSPS->getUseMMVD() )
  {
    READ_FLAG( uiCode,  "sps_fpel_mmvd_enabled_flag" );             pcSPS->setFpelMmvdEnabledFlag ( uiCode != 0 );
  }

  READ_FLAG( uiCode,    "triangle_flag" );                          pcSPS->setUseTriangle            ( uiCode != 0 );

  READ_FLAG(uiCode, "sps_lmcs_enable_flag");                   pcSPS->setUseLmcs(uiCode == 1);
  READ_FLAG( uiCode, "sps_lfnst_enabled_flag" );                    pcSPS->setUseLFNST( uiCode != 0 );

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  READ_FLAG( uiCode, "sps_ladf_enabled_flag" );                     pcSPS->setLadfEnabled( uiCode != 0 );
  if ( pcSPS->getLadfEnabled() )
  {
    int signedSymbol = 0;
    READ_CODE( 2, uiCode, "sps_num_ladf_intervals_minus2");         pcSPS->setLadfNumIntervals( uiCode + 2 );
    READ_SVLC(signedSymbol, "sps_ladf_lowest_interval_qp_offset" );      pcSPS->setLadfQpOffset( signedSymbol, 0 );
    for ( int k = 1; k < pcSPS->getLadfNumIntervals(); k++ )
    {
      READ_SVLC(signedSymbol, "sps_ladf_qp_offset" );                    pcSPS->setLadfQpOffset( signedSymbol, k );
      READ_UVLC( uiCode, "sps_ladf_delta_threshold_minus1");
      pcSPS->setLadfIntervalLowerBound(uiCode + pcSPS->getLadfIntervalLowerBound(k - 1) + 1, k);
    }
  }
#endif

  // KJS: reference picture sets to be replaced

  // KJS: not found in draft -> does not exist

  // KJS: remove scaling lists?
  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );

  READ_FLAG( uiCode, "sps_loop_filter_across_virtual_boundaries_disabled_present_flag" ); pcSPS->setLoopFilterAcrossVirtualBoundariesDisabledFlag( uiCode != 0 );
  if( pcSPS->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
  {
    READ_CODE( 2, uiCode, "sps_num_ver_virtual_boundaries");        pcSPS->setNumVerVirtualBoundaries( uiCode );
    for( unsigned i = 0; i < pcSPS->getNumVerVirtualBoundaries(); i++ )
    {
      READ_CODE(13, uiCode, "sps_virtual_boundaries_pos_x");        pcSPS->setVirtualBoundariesPosX(uiCode << 3, i);
    }
    READ_CODE( 2, uiCode, "sps_num_hor_virtual_boundaries");        pcSPS->setNumHorVirtualBoundaries( uiCode );
    for( unsigned i = 0; i < pcSPS->getNumHorVirtualBoundaries(); i++ )
    {
      READ_CODE(13, uiCode, "sps_virtual_boundaries_pos_y");        pcSPS->setVirtualBoundariesPosY(uiCode << 3, i);
    }
  }
  else
  {
    pcSPS->setNumVerVirtualBoundaries( 0 );
    pcSPS->setNumHorVirtualBoundaries( 0 );
  }

  TimingInfo *timingInfo = pcSPS->getTimingInfo();
  READ_FLAG(     uiCode, "general_hrd_parameters_present_flag");        pcSPS->setHrdParametersPresentFlag(uiCode);
  if( pcSPS->getHrdParametersPresentFlag() )
  {
    READ_CODE( 32, uiCode, "num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "time_scale");                       timingInfo->setTimeScale                  (uiCode);

    READ_FLAG( uiCode, "sub_layer_cpb_parameters_present_flag");  pcSPS->setSubLayerParametersPresentFlag(uiCode);
    if (pcSPS->getSubLayerParametersPresentFlag())
    {
      parseHrdParameters(pcSPS->getHrdParameters(), 0, pcSPS->getMaxTLayers() - 1);
    }
    else
    {
      parseHrdParameters(pcSPS->getHrdParameters(), pcSPS->getMaxTLayers() - 1, pcSPS->getMaxTLayers() - 1);
    }
  }

  READ_FLAG( uiCode, "vui_parameters_present_flag" );             pcSPS->setVuiParametersPresentFlag(uiCode);

  if (pcSPS->getVuiParametersPresentFlag())
  {
    parseVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  // KJS: no SPS extensions defined yet

  READ_FLAG( uiCode, "sps_extension_present_flag");
  if (uiCode)
  {
#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const char *syntaxStrings[]={ "sps_range_extension_flag",
      "sps_multilayer_extension_flag",
      "sps_extension_6bits[0]",
      "sps_extension_6bits[1]",
      "sps_extension_6bits[2]",
      "sps_extension_6bits[3]",
      "sps_extension_6bits[4]",
      "sps_extension_6bits[5]" };
#endif
    bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS];

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      sps_extension_flags[i] = uiCode!=0;
    }

    bool bSkipTrailingExtensionBits=false;
    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
          CHECK(bSkipTrailingExtensionBits, "Skipping trailing extension bits not supported");
          {
            SPSRExt &spsRangeExtension = pcSPS->getSpsRangeExtension();
            READ_FLAG( uiCode, "transform_skip_rotation_enabled_flag");     spsRangeExtension.setTransformSkipRotationEnabledFlag(uiCode != 0);
            READ_FLAG( uiCode, "transform_skip_context_enabled_flag");      spsRangeExtension.setTransformSkipContextEnabledFlag (uiCode != 0);
            READ_FLAG( uiCode, "implicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT, (uiCode != 0));
            READ_FLAG( uiCode, "explicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT, (uiCode != 0));
            READ_FLAG( uiCode, "extended_precision_processing_flag");       spsRangeExtension.setExtendedPrecisionProcessingFlag (uiCode != 0);
            READ_FLAG( uiCode, "intra_smoothing_disabled_flag");            spsRangeExtension.setIntraSmoothingDisabledFlag      (uiCode != 0);
            READ_FLAG( uiCode, "high_precision_offsets_enabled_flag");      spsRangeExtension.setHighPrecisionOffsetsEnabledFlag (uiCode != 0);
            READ_FLAG( uiCode, "persistent_rice_adaptation_enabled_flag");  spsRangeExtension.setPersistentRiceAdaptationEnabledFlag (uiCode != 0);
            READ_FLAG( uiCode, "cabac_bypass_alignment_enabled_flag");      spsRangeExtension.setCabacBypassAlignmentEnabledFlag  (uiCode != 0);
          }
          break;
        default:
          bSkipTrailingExtensionBits=true;
          break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "sps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseDPS(DPS* dps)
{
#if ENABLE_TRACING
  xTraceDPSHeader ();
#endif
  uint32_t  symbol;

  READ_CODE( 4,  symbol,  "dps_decoding_parameter_set_id" );
  CHECK(symbol == 0, "dps_decoding_parameter_set_id equal to zero is reserved and should not be use in a bitstream");
  dps->setDecodingParameterSetId( symbol );

  READ_CODE( 3,  symbol,  "dps_max_sub_layers_minus1" );          dps->setMaxSubLayersMinus1( symbol );
  READ_CODE( 5, symbol,       "dps_reserved_zero_5bits" );              CHECK(symbol != 0, "dps_reserved_zero_5bits must be equal to zero");
  
  uint32_t numPTLs;
  READ_CODE( 4, numPTLs,       "dps_num_ptls_minus1" );
  numPTLs += 1;

  std::vector<ProfileTierLevel> ptls;
  ptls.resize(numPTLs);
  for (int i=0; i<numPTLs; i++)
  {
     parseProfileTierLevel(&ptls[i], dps->getMaxSubLayersMinus1());
  }
  dps->setProfileTierLevel(ptls);

  READ_FLAG( symbol,      "dps_extension_flag" );
  if (symbol)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( symbol, "dps_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseVPS(VPS* pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  uint32_t  uiCode;

  READ_CODE(4, uiCode, "vps_video_parameter_set_id");         
  CHECK( uiCode == 0, "vps_video_parameter_set_id equal to zero is reserved and shall not be used in a bitstream" );
  pcVPS->setVPSId(uiCode);

  READ_CODE(6, uiCode, "vps_max_layers_minus1");              pcVPS->setMaxLayers(uiCode + 1);    CHECK(uiCode + 1 > MAX_VPS_LAYERS, "Invalid code");
  if (pcVPS->getMaxLayers() - 1 == 0)
  {
    pcVPS->setEachLayerIsAnOlsFlag(1);
  }
  READ_CODE(3, uiCode, "vps_max_sublayers_minus1");           pcVPS->setMaxSubLayers(uiCode + 1); CHECK(uiCode + 1 > MAX_VPS_SUBLAYERS, "Invalid code");
  if( pcVPS->getMaxLayers() > 1 && pcVPS->getMaxSubLayers() > 1)
  {
    READ_FLAG(uiCode, "vps_all_layers_same_num_sublayers_flag"); pcVPS->setAllLayersSameNumSublayersFlag(uiCode);
  }
  else
  {
    pcVPS->setAllLayersSameNumSublayersFlag(1);
  }
  if( pcVPS->getMaxLayers() > 1 )
  {
    READ_FLAG(uiCode, "vps_all_independent_layers_flag");  pcVPS->setAllIndependentLayersFlag(uiCode);
    if (pcVPS->getAllIndependentLayersFlag() == 0)
    {
      pcVPS->setEachLayerIsAnOlsFlag(0);
    }
  }
  for (uint32_t i = 0; i < pcVPS->getMaxLayers(); i++)
  {
    READ_CODE(6, uiCode, "vps_layer_id");                     pcVPS->setLayerId(i, uiCode);
    pcVPS->setGeneralLayerIdx(uiCode, i);

    if (i > 0 && !pcVPS->getAllIndependentLayersFlag())
    {
      READ_FLAG(uiCode, "vps_independent_layer_flag");     pcVPS->setIndependentLayerFlag(i, uiCode);
      if (!pcVPS->getIndependentLayerFlag(i))
      {
        uint16_t sumUiCode = 0;
        for (int j = 0, k = 0; j < i; j++)
        {
          READ_FLAG(uiCode, "vps_direct_dependency_flag"); pcVPS->setDirectRefLayerFlag(i, j, uiCode);
          if( uiCode )
          {
            pcVPS->setInterLayerRefIdc( i, j, k );
            pcVPS->setDirectRefLayerIdx( i, k++, j );
            sumUiCode++;
          }
        }
        CHECK(sumUiCode == 0, "There has to be at least one value of j such that the value of vps_direct_dependency_flag[ i ][ j ] is equal to 1,when vps_independent_layer_flag[ i ] is equal to 0 ");
      }
    }
  }

  if (pcVPS->getMaxLayers() > 1)
  {
    if (pcVPS->getAllIndependentLayersFlag())
    {
      READ_FLAG(uiCode, "vps_each_layer_is_an_ols_flag");  pcVPS->setEachLayerIsAnOlsFlag(uiCode);
      if (pcVPS->getEachLayerIsAnOlsFlag() == 0)
      {
        pcVPS->setOlsModeIdc(2);
      }
    }
    if (!pcVPS->getEachLayerIsAnOlsFlag())
    {
      if (!pcVPS->getAllIndependentLayersFlag())
      {
        READ_CODE(2, uiCode, "vps_ols_mode_idc");             pcVPS->setOlsModeIdc(uiCode); CHECK(uiCode > MAX_VPS_OLS_MODE_IDC, "Invalid code");
      }
      if (pcVPS->getOlsModeIdc() == 2)
      {
        READ_CODE(8, uiCode, "num_output_layer_sets_minus1");   pcVPS->setNumOutputLayerSets(uiCode + 1);
        for (uint32_t i = 1; i <= pcVPS->getNumOutputLayerSets() - 1; i++)
        {
          for (uint32_t j = 0; j < pcVPS->getMaxLayers(); j++)
          {
            READ_FLAG(uiCode, "vps_ols_output_layer_flag");        pcVPS->setOlsOutputLayerFlag(i, j, uiCode);
          }
        }
      }
    }
  }
  READ_FLAG(uiCode, "vps_extension_flag");
  if (uiCode)
  {
    while (xMoreRbspData())
    {
      READ_FLAG(uiCode, "vps_extension_data_flag");
    }
  }

  xReadRbspTrailingBits();
}

void HLSyntaxReader::parsePictureHeader( PicHeader* picHeader, ParameterSetManager *parameterSetManager )
{
  uint32_t  uiCode; 
  int       iCode;
  PPS*      pps = NULL;
  SPS*      sps = NULL;
  
#if ENABLE_TRACING
  xTracePictureHeader();
#endif

  READ_FLAG(uiCode, "non_reference_picture_flag");       picHeader->setNonReferencePictureFlag( uiCode != 0 );
  READ_FLAG(uiCode, "gdr_pic_flag");                     picHeader->setGdrPicFlag( uiCode != 0 );
  READ_FLAG(uiCode, "no_output_of_prior_pics_flag");     picHeader->setNoOutputOfPriorPicsFlag( uiCode != 0 );
  if( picHeader->getGdrPicFlag() ) 
  {
    READ_UVLC(uiCode, "recovery_poc_cnt");               picHeader->setRecoveryPocCnt( uiCode );
  }
  else 
  {
    picHeader->setRecoveryPocCnt( 0 );
  }
  
  // parameter sets
  READ_UVLC(uiCode, "ph_pic_parameter_set_id");
  picHeader->setPPSId( uiCode );
  pps = parameterSetManager->getPPS(picHeader->getPPSId());
  CHECK(pps==0, "Invalid PPS");  
  picHeader->setSPSId( pps->getSPSId() );
  sps = parameterSetManager->getSPS(picHeader->getSPSId());
  CHECK(sps==0, "Invalid SPS");
  
  // initialize tile/slice info for no partitioning case
  if( pps->getNoPicPartitionFlag() )
  {
    pps->resetTileSliceInfo();
    pps->setLog2CtuSize( ceilLog2(sps->getCTUSize()) );
    pps->setNumExpTileColumns(1);
    pps->setNumExpTileRows(1);
    pps->addTileColumnWidth( pps->getPicWidthInCtu( ) );
    pps->addTileRowHeight( pps->getPicHeightInCtu( ) );
    pps->initTiles();
    pps->setRectSliceFlag( 1 );
    pps->setNumSlicesInPic( 1 );
    pps->initRectSlices( );
    pps->setTileIdxDeltaPresentFlag( 0 );
    pps->setSliceTileIdx( 0, 0 );
    pps->initRectSliceMap( );
  }
  else 
  {
    CHECK(pps->getCtuSize() != sps->getCTUSize(), "PPS CTU size does not match CTU size in SPS");
  }

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
      READ_FLAG(uiCode, "ph_subpic_id_signalling_present_flag");                 picHeader->setSubPicIdSignallingPresentFlag( uiCode != 0 );
      if( picHeader->getSubPicIdSignallingPresentFlag() )
      {
        READ_UVLC( uiCode, "ph_subpic_id_len_minus1" );                          picHeader->setSubPicIdLen( uiCode + 1 );
        CHECK( uiCode > 15, "Invalid ph_subpic_id_len_minus1 signalled");
        for( int picIdx = 0; picIdx < sps->getNumSubPics( ); picIdx++ )
        {
          READ_CODE( picHeader->getSubPicIdLen( ), uiCode, "ph_subpic_id[i]" );   picHeader->setSubPicId( picIdx, uiCode );
        }
      }
      else 
      {
        for( int picIdx = 0; picIdx < sps->getNumSubPics( ); picIdx++ )
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

  // virtual boundaries
  if( !sps->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
  {
    READ_FLAG( uiCode, "ph_loop_filter_across_virtual_boundaries_disabled_present_flag" ); picHeader->setLoopFilterAcrossVirtualBoundariesDisabledFlag( uiCode != 0 );
    if( picHeader->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
    {
      READ_CODE( 2, uiCode, "ph_num_ver_virtual_boundaries");        picHeader->setNumVerVirtualBoundaries( uiCode );
      for( unsigned i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++ )
      {
        READ_CODE(13, uiCode, "ph_virtual_boundaries_pos_x");        picHeader->setVirtualBoundariesPosX(uiCode << 3, i);
      }
      READ_CODE( 2, uiCode, "ph_num_hor_virtual_boundaries");        picHeader->setNumHorVirtualBoundaries( uiCode );
      for( unsigned i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++ )
      {
        READ_CODE(13, uiCode, "ph_virtual_boundaries_pos_y");        picHeader->setVirtualBoundariesPosY(uiCode << 3, i);
      }
    }
    else
    {
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
  
  // 4:4:4 colour plane ID
  if( sps->getSeparateColourPlaneFlag() )
  {
    READ_CODE( 2, uiCode, "colour_plane_id" ); picHeader->setColourPlaneId( uiCode );
    CHECK(uiCode > 2, "colour_plane_id exceeds valid range");
  }
  else 
  {
    picHeader->setColourPlaneId( 0 );
  }

  // picture output flag
  if( pps->getOutputFlagPresentFlag() )
  {
    READ_FLAG( uiCode, "pic_output_flag" ); picHeader->setPicOutputFlag( uiCode != 0 );
  }
  else 
  {
    picHeader->setPicOutputFlag( true );
  }

  // reference picture lists
  READ_FLAG( uiCode, "pic_rpl_present_flag" ); picHeader->setPicRplPresentFlag( uiCode != 0 );
  if( picHeader->getPicRplPresentFlag() )
  {
    // List0 and List1
    for(int listIdx = 0; listIdx < 2; listIdx++) 
    {                 
      // copy L1 index from L0 index
      if (listIdx == 1 && !pps->getRpl1IdxPresentFlag())
      {
        picHeader->setRPL1idx(picHeader->getRPL0idx());
        uiCode = (picHeader->getRPL0idx() != -1);
      }
      // RPL in picture header or SPS
      else if (sps->getNumRPL( listIdx ) == 0)
      {
        uiCode = 0;
      }
      else if (!pps->getPPSRefPicListSPSIdc( listIdx ))
      {
        READ_FLAG(uiCode, "pic_rpl_sps_flag[i]");
      }
      else
      {
        uiCode = pps->getPPSRefPicListSPSIdc( listIdx ) - 1;
      }

      // explicit RPL in picture header
      if (!uiCode)
      {
        ReferencePictureList* rpl = picHeader->getLocalRPL( listIdx );
        (*rpl) = ReferencePictureList();
        parseRefPicList(sps, rpl);
        picHeader->setRPLIdx(listIdx, -1);
        picHeader->setRPL(listIdx, rpl);
      }
      // use list from SPS
      else 
      { 
        if (listIdx == 1 && !pps->getRpl1IdxPresentFlag())
        {
          picHeader->setRPL( listIdx, sps->getRPLList( listIdx )->getReferencePictureList(picHeader->getRPLIdx( listIdx )));
        }
        else if (sps->getNumRPL( listIdx ) > 1)
        {
          int numBits = ceilLog2(sps->getNumRPL( listIdx ));
          READ_CODE(numBits, uiCode, "pic_rpl_idx[i]");
          picHeader->setRPLIdx( listIdx, uiCode );
          picHeader->setRPL( listIdx, sps->getRPLList( listIdx )->getReferencePictureList(uiCode));
        }
        else
        {
          picHeader->setRPLIdx( listIdx, 0 );
          picHeader->setRPL( listIdx, sps->getRPLList( listIdx )->getReferencePictureList(0));
        }
      }

      // POC MSB cycle signalling for LTRP
      for (int i = 0; i < picHeader->getRPL( listIdx )->getNumberOfLongtermPictures() + picHeader->getRPL( listIdx )->getNumberOfShorttermPictures(); i++)
      {
        picHeader->getLocalRPL( listIdx )->setDeltaPocMSBPresentFlag(i, false);
        picHeader->getLocalRPL( listIdx )->setDeltaPocMSBCycleLT(i, 0);
      }
      if (picHeader->getRPL( listIdx )->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < picHeader->getRPL( listIdx )->getNumberOfLongtermPictures() + picHeader->getRPL( listIdx )->getNumberOfShorttermPictures(); i++)
        {
          if (picHeader->getRPL( listIdx )->isRefPicLongterm(i))
          {
            if (picHeader->getRPL( listIdx )->getLtrpInSliceHeaderFlag())
            {
              READ_CODE(sps->getBitsForPOC(), uiCode, "pic_poc_lsb_lt[i][j]");
              picHeader->getLocalRPL( listIdx )->setRefPicIdentifier( i, uiCode, true, false, 0 );
            }
            READ_FLAG(uiCode, "pic_delta_poc_msb_present_flag[i][j]");
            picHeader->getLocalRPL( listIdx )->setDeltaPocMSBPresentFlag(i, uiCode ? true : false);
            if (uiCode)
            {
              READ_UVLC(uiCode, "pic_delta_poc_msb_cycle_lt[i][j]");
              picHeader->getLocalRPL( listIdx )->setDeltaPocMSBCycleLT(i, uiCode);
            }
          }
        }
      }
    }
  }

  // partitioning constraint overrides
  if (sps->getSplitConsOverrideEnabledFlag())
  {
    READ_FLAG(uiCode, "partition_constraints_override_flag");  picHeader->setSplitConsOverrideFlag( uiCode != 0 );
    if (picHeader->getSplitConsOverrideFlag())
    {
      unsigned  minQT[3]     = { 0, 0, 0 };
      unsigned  maxBTD[3]    = { 0, 0, 0 };
      unsigned  maxBTSize[3] = { 0, 0, 0 };
      unsigned  maxTTSize[3] = { 0, 0, 0 };
      unsigned  ctbLog2SizeY = floorLog2(sps->getCTUSize());

      READ_UVLC(uiCode, "pic_log2_diff_min_qt_min_cb_intra_slice_luma");
      unsigned minQtLog2SizeIntraY = uiCode + sps->getLog2MinCodingBlockSize();
      minQT[0] = 1 << minQtLog2SizeIntraY;
      READ_UVLC(uiCode, "pic_log2_diff_min_qt_min_cb_inter_slice");
      unsigned minQtLog2SizeInterY = uiCode + sps->getLog2MinCodingBlockSize();
      minQT[1] = 1 << minQtLog2SizeInterY;
      READ_UVLC(uiCode, "pic_max_mtt_hierarchy_depth_inter_slice");              maxBTD[1] = uiCode;
      READ_UVLC(uiCode, "pic_max_mtt_hierarchy_depth_intra_slice_luma");         maxBTD[0] = uiCode;

      maxTTSize[0] = maxBTSize[0] = minQT[0];
      if (maxBTD[0] != 0)
      {
        READ_UVLC(uiCode, "pic_log2_diff_max_bt_min_qt_intra_slice_luma");       maxBTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
        READ_UVLC(uiCode, "pic_log2_diff_max_tt_min_qt_intra_slice_luma");       maxTTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
      }
      maxTTSize[1] = maxBTSize[1] = minQT[1];
      if (maxBTD[1] != 0)
      {
        READ_UVLC(uiCode, "pic_log2_diff_max_bt_min_qt_inter_slice");            maxBTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
        READ_UVLC(uiCode, "pic_log2_diff_max_tt_min_qt_inter_slice");            maxTTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
      }
      if (sps->getUseDualITree())
      {
        READ_UVLC(uiCode, "pic_log2_diff_min_qt_min_cb_intra_slice_chroma");     minQT[2] = 1 << (uiCode + sps->getLog2MinCodingBlockSize());
        READ_UVLC(uiCode, "pic_max_mtt_hierarchy_depth_intra_slice_chroma");     maxBTD[2] = uiCode;
        maxTTSize[2] = maxBTSize[2] = minQT[2];
        if (maxBTD[2] != 0)
        {
          READ_UVLC(uiCode, "pic_log2_diff_max_bt_min_qt_intra_slice_chroma");   maxBTSize[2] <<= uiCode;
          READ_UVLC(uiCode, "pic_log2_diff_max_tt_min_qt_intra_slice_chroma");   maxTTSize[2] <<= uiCode;
        }
      }

      picHeader->setMinQTSizes(minQT);
      picHeader->setMaxMTTHierarchyDepths(maxBTD);
      picHeader->setMaxBTSizes(maxBTSize);
      picHeader->setMaxTTSizes(maxTTSize);
    }
  }
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

  // delta quantization and chrom and chroma offset
  if (pps->getUseDQP())
  {
    READ_UVLC( uiCode, "pic_cu_qp_delta_subdiv_intra_slice" );   picHeader->setCuQpDeltaSubdivIntra( uiCode );
    READ_UVLC( uiCode, "pic_cu_qp_delta_subdiv_inter_slice" );   picHeader->setCuQpDeltaSubdivInter( uiCode );
  }
  else 
  {
    picHeader->setCuQpDeltaSubdivIntra( 0 );
    picHeader->setCuQpDeltaSubdivInter( 0 );
  }
  if (pps->getCuChromaQpOffsetEnabledFlag())
  {
    READ_UVLC( uiCode, "pic_cu_chroma_qp_offset_subdiv_intra_slice" );   picHeader->setCuChromaQpOffsetSubdivIntra( uiCode );
    READ_UVLC( uiCode, "pic_cu_chroma_qp_offset_subdiv_inter_slice" );   picHeader->setCuChromaQpOffsetSubdivInter( uiCode );
  }
  else 
  {
    picHeader->setCuChromaQpOffsetSubdivIntra( 0 );
    picHeader->setCuChromaQpOffsetSubdivInter( 0 );
  }
  
  // temporal motion vector prediction
  if (sps->getSPSTemporalMVPEnabledFlag())
  {
    READ_FLAG( uiCode, "pic_temporal_mvp_enabled_flag" );
    picHeader->setEnableTMVPFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setEnableTMVPFlag(false);
  }

  // mvd L1 zero flag
  if (!pps->getPPSMvdL1ZeroIdc())
  {
    READ_FLAG(uiCode, "pic_mvd_l1_zero_flag");
  }
  else
  {
    uiCode = pps->getPPSMvdL1ZeroIdc() - 1;
  }
  picHeader->setMvdL1ZeroFlag( uiCode != 0 );
     
  // merge candidate list size
  if (!pps->getPPSSixMinusMaxNumMergeCandPlus1())
  {
    READ_UVLC(uiCode, "pic_six_minus_max_num_merge_cand");
  }
  else
  {
    uiCode = pps->getPPSSixMinusMaxNumMergeCandPlus1() - 1;
  }
  CHECK(MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of merge candidates!");
  picHeader->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);

  // subblock merge candidate list size
  if ( sps->getUseAffine() )
  {
    READ_UVLC(uiCode, "pic_five_minus_max_num_subblock_merge_cand");
    CHECK(AFFINE_MRG_MAX_NUM_CANDS < uiCode, "Incorrrect max number of affine merge candidates!");
    picHeader->setMaxNumAffineMergeCand( AFFINE_MRG_MAX_NUM_CANDS - uiCode );
  }
  else
  {
    picHeader->setMaxNumAffineMergeCand( sps->getSBTMVPEnabledFlag() && picHeader->getEnableTMVPFlag() );
  }

  // full-pel MMVD flag
  if (sps->getFpelMmvdEnabledFlag())
  {
    READ_FLAG( uiCode, "pic_fpel_mmvd_enabled_flag" );
    picHeader->setDisFracMMVD( uiCode != 0 );
  }
  else
  {
    picHeader->setDisFracMMVD(false);
  }
  
  // picture level BDOF disable flags
  if (sps->getBdofControlPresentFlag())
  {
    READ_FLAG(uiCode, "pic_disable_bdof_flag");  picHeader->setDisBdofFlag(uiCode != 0);
  }
  else
  {
    picHeader->setDisBdofFlag(0);
  }

  // picture level DMVR disable flags
  if (sps->getDmvrControlPresentFlag())
  {
    READ_FLAG(uiCode, "pic_disable_dmvr_flag");  picHeader->setDisDmvrFlag(uiCode != 0);
  }
  else
  {
    picHeader->setDisDmvrFlag(0);
  }

  // picture level PROF disable flags
  if (sps->getProfControlPresentFlag())
  {
    READ_FLAG(uiCode, "pic_disable_prof_flag");  picHeader->setDisProfFlag(uiCode != 0);
  }
  else
  {
    picHeader->setDisProfFlag(0);
  }

  // triangle merge candidate list size
  if (sps->getUseTriangle() && picHeader->getMaxNumMergeCand() >= 2)
  {
    if (!pps->getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1())
    {
      READ_UVLC(uiCode, "pic_max_num_merge_cand_minus_max_num_triangle_cand");
    }
    else
    {
      uiCode = pps->getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1() - 1;
    }
    CHECK(picHeader->getMaxNumMergeCand() < uiCode, "Incorrrect max number of triangle candidates!");
    picHeader->setMaxNumTriangleCand((uint32_t)(picHeader->getMaxNumMergeCand() - uiCode));
  }
  else
  {
    picHeader->setMaxNumTriangleCand(0);
  }

  // ibc merge candidate list size
  if (sps->getIBCFlag())
  {
    READ_UVLC(uiCode, "pic_six_minus_max_num_ibc_merge_cand");
    CHECK(IBC_MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of IBC merge candidates!");
    picHeader->setMaxNumIBCMergeCand(IBC_MRG_MAX_NUM_CANDS - uiCode);
  }
  else 
   {
    picHeader->setMaxNumIBCMergeCand(0);
  }

  // joint Cb/Cr sign flag
  if (sps->getJointCbCrEnabledFlag())
  {
    READ_FLAG( uiCode, "pic_joint_cbcr_sign_flag" ); 
    picHeader->setJointCbCrSignFlag(uiCode != 0);
  }
  else
  {
    picHeader->setJointCbCrSignFlag(false);
  }

  // sao enable flags
  if(sps->getSAOEnabledFlag())
  {
    READ_FLAG(uiCode, "pic_sao_enabled_present_flag");  
    picHeader->setSaoEnabledPresentFlag(uiCode != 0);

    if (picHeader->getSaoEnabledPresentFlag())
    {    
      READ_FLAG(uiCode, "slice_sao_luma_flag");  
      picHeader->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, uiCode != 0);

      if (sps->getChromaFormatIdc() != CHROMA_400)
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  
        picHeader->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, uiCode != 0);
      }
    }
    else 
    {
      picHeader->setSaoEnabledFlag(CHANNEL_TYPE_LUMA,   true);
      picHeader->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() != CHROMA_400);
    }
  }
  else 
  {
    picHeader->setSaoEnabledFlag(CHANNEL_TYPE_LUMA,   false);
    picHeader->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, false);
  }
  
  // alf enable flags and aps IDs
  if( sps->getALFEnabledFlag() )
  {
    READ_FLAG(uiCode, "pic_alf_enabled_present_flag");  
    picHeader->setAlfEnabledPresentFlag(uiCode != 0);

    if (picHeader->getAlfEnabledPresentFlag()) 
    {
      READ_FLAG(uiCode, "pic_alf_enabled_flag");
      picHeader->setAlfEnabledFlag(COMPONENT_Y, uiCode);

      int alfChromaIdc = 0;
      if (uiCode)
      {
        READ_CODE(3, uiCode, "pic_num_alf_aps_ids_luma");
        int numAps = uiCode;
        picHeader->setNumAlfAps(numAps);

        std::vector<int> apsId(numAps, -1);
        for (int i = 0; i < numAps; i++)
        {
          READ_CODE(3, uiCode, "pic_alf_aps_id_luma");
          apsId[i] = uiCode;
        }
        picHeader->setAlfAPSs(apsId);

        if (sps->getChromaFormatIdc() != CHROMA_400)
        {
          READ_CODE(2, uiCode, "pic_alf_chroma_idc");
          alfChromaIdc = uiCode;
        }
        else
        {
          alfChromaIdc = 0;
        }
        if (alfChromaIdc)
        {
          READ_CODE(3, uiCode, "pic_alf_aps_id_chroma");
          picHeader->setAlfApsIdChroma(uiCode);
        }
      }
      else
      {
        picHeader->setNumAlfAps(0);
      }
      picHeader->setAlfEnabledFlag(COMPONENT_Cb, alfChromaIdc & 1);
      picHeader->setAlfEnabledFlag(COMPONENT_Cr, alfChromaIdc >> 1);
    }
    else 
    {
      picHeader->setAlfEnabledFlag(COMPONENT_Y,  true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cb, true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cr, true);
    }
  }
  else 
  {
    picHeader->setAlfEnabledFlag(COMPONENT_Y,  false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cb, false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cr, false);
  }

  // dependent quantization
  if (!pps->getPPSDepQuantEnabledIdc())
  {
    READ_FLAG(uiCode, "pic_dep_quant_enabled_flag");
  }
  else
  {
    uiCode = pps->getPPSDepQuantEnabledIdc() - 1;
  }
  picHeader->setDepQuantEnabledFlag( uiCode != 0 );

  // sign data hiding
  if( !picHeader->getDepQuantEnabledFlag() )
  {
    READ_FLAG( uiCode, "pic_sign_data_hiding_enabled_flag" );
    picHeader->setSignDataHidingEnabledFlag( uiCode != 0 );
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
      READ_FLAG ( uiCode, "pic_deblocking_filter_override_present_flag" );
      picHeader->setDeblockingFilterOverridePresentFlag(uiCode != 0);
    
      if( picHeader->getDeblockingFilterOverridePresentFlag() ) 
      {
        READ_FLAG ( uiCode, "pic_deblocking_filter_override_flag" );
        picHeader->setDeblockingFilterOverrideFlag(uiCode != 0);
      }
      else
      {
        picHeader->setDeblockingFilterOverrideFlag(false);
      }
    }
    else
    {
      picHeader->setDeblockingFilterOverridePresentFlag(false);
      picHeader->setDeblockingFilterOverrideFlag(false);
    }

    if(picHeader->getDeblockingFilterOverrideFlag())
    {
      READ_FLAG ( uiCode, "pic_deblocking_filter_disabled_flag" );
      picHeader->setDeblockingFilterDisable(uiCode != 0);
      if(!picHeader->getDeblockingFilterDisable())
      {
        READ_SVLC( iCode, "pic_beta_offset_div2" );
        picHeader->setDeblockingFilterBetaOffsetDiv2(iCode);
        CHECK(  picHeader->getDeblockingFilterBetaOffsetDiv2() < -6 &&
                picHeader->getDeblockingFilterBetaOffsetDiv2() >  6, "Invalid deblocking filter configuration");

        READ_SVLC( iCode, "pic_tc_offset_div2" );
        picHeader->setDeblockingFilterTcOffsetDiv2(iCode);
        CHECK  (picHeader->getDeblockingFilterTcOffsetDiv2() < -6 &&
                picHeader->getDeblockingFilterTcOffsetDiv2() >  6, "Invalid deblocking filter configuration");
      }
    }
    else
    {
      picHeader->setDeblockingFilterDisable       ( pps->getPPSDeblockingFilterDisabledFlag() );
      picHeader->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
      picHeader->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
    }
  }
  else
  {
    picHeader->setDeblockingFilterDisable       ( false );
    picHeader->setDeblockingFilterBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterTcOffsetDiv2  ( 0 );
  }

  // luma mapping / chroma scaling controls
  if (sps->getUseLmcs())
  {
    READ_FLAG(uiCode, "pic_lmcs_enabled_flag");
    picHeader->setLmcsEnabledFlag(uiCode != 0);

    if (picHeader->getLmcsEnabledFlag())
    {
      READ_CODE(2, uiCode, "pic_lmcs_aps_id");
      picHeader->setLmcsAPSId(uiCode);

      if (sps->getChromaFormatIdc() != CHROMA_400)
      {
        READ_FLAG(uiCode, "pic_chroma_residual_scale_flag");
        picHeader->setLmcsChromaResidualScaleFlag(uiCode != 0);
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
    READ_FLAG( uiCode, "pic_scaling_list_present_flag" );
    picHeader->setScalingListPresentFlag( uiCode );
    if( picHeader->getScalingListPresentFlag() )
    {
      READ_CODE( 3, uiCode, "pic_scaling_list_aps_id" );
      picHeader->setScalingListAPSId( uiCode );
    }
  }
  else 
  {
    picHeader->setScalingListPresentFlag( false );
  }

  // picture header extension
  if(pps->getPictureHeaderExtensionPresentFlag())
  {
    READ_UVLC(uiCode,"pic_segment_header_extension_length");
    for(int i=0; i<uiCode; i++)
    {
      uint32_t ignore_;
      READ_CODE(8,ignore_,"pic_segment_header_extension_data_byte");
    }
  }

  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseSliceHeader (Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager, const int prevTid0POC)
{
  uint32_t  uiCode;
  int   iCode;

#if ENABLE_TRACING
  xTraceSliceHeader();
#endif
  PPS* pps = NULL;
  SPS* sps = NULL;

  CHECK(picHeader==0, "Invalid Picture Header");
  CHECK(picHeader->isValid()==false, "Invalid Picture Header");
  pps = parameterSetManager->getPPS( picHeader->getPPSId() );
  //!KS: need to add error handling code here, if PPS is not available
  CHECK(pps==0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  CHECK(sps==0, "Invalid SPS");

  const ChromaFormat chFmt = sps->getChromaFormatIdc();
  const uint32_t numValidComp=getNumberValidComponents(chFmt);
  const bool bChroma=(chFmt!=CHROMA_400);

  // picture order count
  READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
  if (pcSlice->getIdrPicFlag())
  {
    pcSlice->setPOC(uiCode);
  }
  else
  {
    int iPOClsb = uiCode;
    int iPrevPOC = prevTid0POC;
    int iMaxPOClsb = 1 << sps->getBitsForPOC();
    int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
    int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
    int iPOCmsb;
    if ((iPOClsb  <  iPrevPOClsb) && ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2)))
    {
      iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
    }
    else if ((iPOClsb  >  iPrevPOClsb) && ((iPOClsb - iPrevPOClsb)  >  (iMaxPOClsb / 2)))
    {
      iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
    }
    else
    {
      iPOCmsb = iPrevPOCmsb;
    }
    pcSlice->setPOC(iPOCmsb + iPOClsb);
  }

  if (sps->getSubPicPresentFlag())
  {
    uint32_t bitsSubPicId;
    if (sps->getSubPicIdSignallingPresentFlag())
    {
      bitsSubPicId = sps->getSubPicIdLen();
    }
    else if (picHeader->getSubPicIdSignallingPresentFlag())
    {
      bitsSubPicId = picHeader->getSubPicIdLen();
    }
    else if (pps->getSubPicIdSignallingPresentFlag())
    {
      bitsSubPicId = pps->getSubPicIdLen();
    }
    else
    {
      bitsSubPicId = ceilLog2(sps->getNumSubPics());
    }
    READ_CODE(bitsSubPicId, uiCode, "slice_subpic_id");    pcSlice->setSliceSubPicId(uiCode);
  }

  // raster scan slices
  if(pps->getRectSliceFlag() == 0) 
  {
    uint32_t sliceAddr, numTilesInSlice;

    // slice address is the raster scan tile index of first tile in slice
    if( pps->getNumTiles() > 1 ) 
    {      
      int bitsSliceAddress = ceilLog2(pps->getNumTiles());
      READ_CODE(bitsSliceAddress, uiCode, "slice_address");  sliceAddr = uiCode;
      READ_UVLC(uiCode, "num_tiles_in_slice_minus1");        numTilesInSlice = uiCode + 1;      
    }
    else {
      sliceAddr = 0;
      numTilesInSlice = 1;
    }
    CHECK(sliceAddr >= pps->getNumTiles(), "Invalid slice address");
    pcSlice->initSliceMap();
    pcSlice->setSliceID(sliceAddr);
    
    for( uint32_t tileIdx = sliceAddr; tileIdx < sliceAddr + numTilesInSlice; tileIdx++ )
    {
      uint32_t tileX = tileIdx % pps->getNumTileColumns();
      uint32_t tileY = tileIdx / pps->getNumTileColumns();
      CHECK(tileY >= pps->getNumTileRows(), "Number of tiles in slice exceeds the remaining number of tiles in picture");

      pcSlice->addCtusToSlice(pps->getTileColumnBd(tileX), pps->getTileColumnBd(tileX + 1),
                              pps->getTileRowBd(tileY), pps->getTileRowBd(tileY + 1), pps->getPicWidthInCtu());
   }
  }
  // rectangular slices
  else 
  {
    uint32_t sliceAddr;

    // slice address is the index of the slice within the current sub-picture
    if( pps->getNumSlicesInPic() > 1 ) 
    {
      int bitsSliceAddress = ceilLog2(pps->getNumSlicesInPic());  // change to NumSlicesInSubPic when available
      READ_CODE(bitsSliceAddress, uiCode, "slice_address");  sliceAddr = uiCode;
      CHECK(sliceAddr >= pps->getNumSlicesInPic(), "Invalid slice address");
    }
    else {
      sliceAddr = 0;
    }
    pcSlice->setSliceMap( pps->getSliceMap(sliceAddr) );
    pcSlice->setSliceID(sliceAddr);
  }


    READ_UVLC (    uiCode, "slice_type" );            pcSlice->setSliceType((SliceType)uiCode);

    // inherit values from picture header
    //   set default values in case slice overrides are disabled
    pcSlice->inheritFromPicHeader( picHeader, pps, sps );


    if( picHeader->getPicRplPresentFlag() )
    {
      pcSlice->setRPL0(picHeader->getRPL0());
      pcSlice->setRPL1(picHeader->getRPL1());
      *pcSlice->getLocalRPL0() = *picHeader->getLocalRPL0();
      *pcSlice->getLocalRPL1() = *picHeader->getLocalRPL1();
    }
    else if( pcSlice->getIdrPicFlag() && !(sps->getIDRRefParamListPresent()) )
    {
      ReferencePictureList* rpl0 = pcSlice->getLocalRPL0();
      (*rpl0) = ReferencePictureList();
      pcSlice->setRPL0(rpl0);
      ReferencePictureList* rpl1 = pcSlice->getLocalRPL1();
      (*rpl1) = ReferencePictureList();
      pcSlice->setRPL1(rpl1);
    }
    else
    {
      //Read L0 related syntax elements
      if (sps->getNumRPL0() > 0)
      {
        if (!pps->getPPSRefPicListSPSIdc0())
        {
          READ_FLAG(uiCode, "ref_pic_list_sps_flag[0]");
        }
        else
        {
          uiCode = pps->getPPSRefPicListSPSIdc0() - 1;
        }
      }
      else
      {
        uiCode = 0;
      }

      if (!uiCode) //explicitly carried in this SH
      {
        ReferencePictureList* rpl0 = pcSlice->getLocalRPL0();
        (*rpl0) = ReferencePictureList();
        parseRefPicList(sps, rpl0);
        pcSlice->setRPL0idx(-1);
        pcSlice->setRPL0(rpl0);
      }
      else    //Refer to list in SPS
      {
        if (sps->getNumRPL0() > 1)
        {
          int numBits = ceilLog2(sps->getNumRPL0());
          READ_CODE(numBits, uiCode, "ref_pic_list_idx[0]");
          pcSlice->setRPL0idx(uiCode);
          pcSlice->setRPL0(sps->getRPLList0()->getReferencePictureList(uiCode));
        }
        else
        {
          pcSlice->setRPL0idx(0);
          pcSlice->setRPL0(sps->getRPLList0()->getReferencePictureList(0));
        }
      }
      //Deal POC Msb cycle signalling for LTRP
      for (int i = 0; i < pcSlice->getRPL0()->getNumberOfLongtermPictures() + pcSlice->getRPL0()->getNumberOfShorttermPictures(); i++)
      {
        pcSlice->getLocalRPL0()->setDeltaPocMSBPresentFlag(i, false);
        pcSlice->getLocalRPL0()->setDeltaPocMSBCycleLT(i, 0);
      }
      if (pcSlice->getRPL0()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL0()->getNumberOfLongtermPictures() + pcSlice->getRPL0()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL0()->isRefPicLongterm(i))
          {
            if (pcSlice->getRPL0()->getLtrpInSliceHeaderFlag())
            {
              READ_CODE(sps->getBitsForPOC(), uiCode, "slice_poc_lsb_lt[i][j]");
              pcSlice->getLocalRPL0()->setRefPicIdentifier( i, uiCode, true, false, 0 );
            }
            READ_FLAG(uiCode, "delta_poc_msb_present_flag[i][j]");
            pcSlice->getLocalRPL0()->setDeltaPocMSBPresentFlag(i, uiCode ? true : false);
            if (uiCode)
            {
              READ_UVLC(uiCode, "slice_delta_poc_msb_cycle_lt[i][j]");
              pcSlice->getLocalRPL0()->setDeltaPocMSBCycleLT(i, uiCode);
            }
          }
        }
      }

      //Read L1 related syntax elements
      if (!pps->getRpl1IdxPresentFlag())
      {
        pcSlice->setRPL1idx(pcSlice->getRPL0idx());
        if (pcSlice->getRPL1idx() != -1)
          pcSlice->setRPL1(sps->getRPLList1()->getReferencePictureList(pcSlice->getRPL0idx()));
      }
      else
      {
        if (sps->getNumRPL1() > 0)
        {
          if (!pps->getPPSRefPicListSPSIdc1())
          {
            READ_FLAG(uiCode, "ref_pic_list_sps_flag[1]");
          }
          else
          {
            uiCode = pps->getPPSRefPicListSPSIdc1() - 1;
          }
        }
        else
        {
          uiCode = 0;
        }
        if (uiCode == 1)
        {
          if (sps->getNumRPL1() > 1)
          {
            int numBits = ceilLog2(sps->getNumRPL1());
            READ_CODE(numBits, uiCode, "ref_pic_list_idx[1]");
            pcSlice->setRPL1idx(uiCode);
            pcSlice->setRPL1(sps->getRPLList1()->getReferencePictureList(uiCode));
          }
          else
          {
            pcSlice->setRPL1idx(0);
            pcSlice->setRPL1(sps->getRPLList1()->getReferencePictureList(0));
          }
        }
        else
        {
          pcSlice->setRPL1idx(-1);
        }
      }
      if (pcSlice->getRPL1idx() == -1) //explicitly carried in this SH
      {
        ReferencePictureList* rpl1 = pcSlice->getLocalRPL1();
        (*rpl1) = ReferencePictureList();
        parseRefPicList(sps, rpl1);
        pcSlice->setRPL1idx(-1);
        pcSlice->setRPL1(rpl1);
      }

      //Deal POC Msb cycle signalling for LTRP
      for (int i = 0; i < pcSlice->getRPL1()->getNumberOfLongtermPictures() + pcSlice->getRPL1()->getNumberOfShorttermPictures(); i++)
      {
        pcSlice->getLocalRPL1()->setDeltaPocMSBPresentFlag(i, false);
        pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i, 0);
      }
      if (pcSlice->getRPL1()->getNumberOfLongtermPictures())
      {
        for (int i = 0; i < pcSlice->getRPL1()->getNumberOfLongtermPictures() + pcSlice->getRPL1()->getNumberOfShorttermPictures(); i++)
        {
          if (pcSlice->getRPL1()->isRefPicLongterm(i))
          {
            if (pcSlice->getRPL1()->getLtrpInSliceHeaderFlag())
            {
              READ_CODE(sps->getBitsForPOC(), uiCode, "slice_poc_lsb_lt[i][j]");
              pcSlice->getLocalRPL1()->setRefPicIdentifier( i, uiCode, true, false, 0 );
            }
            READ_FLAG(uiCode, "delta_poc_msb_present_flag[i][j]");
            pcSlice->getLocalRPL1()->setDeltaPocMSBPresentFlag(i, uiCode ? true : false);
            if (uiCode)
            {
              READ_UVLC(uiCode, "slice_delta_poc_msb_cycle_lt[i][j]");
              pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i, uiCode);
            }
          }
        }
      }

    }
    if( !picHeader->getPicRplPresentFlag() && pcSlice->getIdrPicFlag() && !(sps->getIDRRefParamListPresent()))
    {
      pcSlice->setNumRefIdx(REF_PIC_LIST_0, 0);
      pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
    }
    else
    {
      if ((!pcSlice->isIntra() && pcSlice->getRPL0()->getNumRefEntries() > 1) ||
          (pcSlice->isInterB() && pcSlice->getRPL1()->getNumRefEntries() > 1) )
      {
        READ_FLAG( uiCode, "num_ref_idx_active_override_flag");
        if (uiCode)
        {
          if(pcSlice->getRPL0()->getNumRefEntries() > 1)
          {
            READ_UVLC (uiCode, "num_ref_idx_l0_active_minus1" );
          }
          else
          {
            uiCode = 0;
          }
          pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
          if (pcSlice->isInterB())
          {
            if(pcSlice->getRPL1()->getNumRefEntries() > 1)
            {
              READ_UVLC (uiCode, "num_ref_idx_l1_active_minus1" );
            }
            else
            {
              uiCode = 0;
            }
            pcSlice->setNumRefIdx(REF_PIC_LIST_1, uiCode + 1);
          }
          else
          {
            pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
          }
        }
        else
        {
          if(pcSlice->getRPL0()->getNumRefEntries() >= pps->getNumRefIdxL0DefaultActive())
          {
            pcSlice->setNumRefIdx(REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive());
          }
          else
          {
            pcSlice->setNumRefIdx(REF_PIC_LIST_0, pcSlice->getRPL0()->getNumRefEntries());
          }

          if (pcSlice->isInterB())
          {
            if(pcSlice->getRPL1()->getNumRefEntries() >= pps->getNumRefIdxL1DefaultActive())
            {
              pcSlice->setNumRefIdx(REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive());
            }
            else
            {
              pcSlice->setNumRefIdx(REF_PIC_LIST_1, pcSlice->getRPL1()->getNumRefEntries());
            }
          }
          else
          {
            pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
          }
        }
      }
      else
      {
        pcSlice->setNumRefIdx( REF_PIC_LIST_0, pcSlice->isIntra() ? 0 : 1 );
        pcSlice->setNumRefIdx( REF_PIC_LIST_1, pcSlice->isInterB() ? 1 : 0 );
      }
    }

    if (pcSlice->isInterP() || pcSlice->isInterB())
    {
      CHECK(pcSlice->getNumRefIdx(REF_PIC_LIST_0) == 0, "Number of active entries in RPL0 of P or B picture shall be greater than 0");
      if (pcSlice->isInterB())
        CHECK(pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0, "Number of active entries in RPL1 of B picture shall be greater than 0");
    }


    pcSlice->setCabacInitFlag( false ); // default
    if(pps->getCabacInitPresentFlag() && !pcSlice->isIntra())
    {
      READ_FLAG(uiCode, "cabac_init_flag");
      pcSlice->setCabacInitFlag( uiCode ? true : false );
      pcSlice->setEncCABACTableIdx( pcSlice->getSliceType() == B_SLICE ? ( uiCode ? P_SLICE : B_SLICE ) : ( uiCode ? B_SLICE : P_SLICE ) );
    }

    if ( pcSlice->getPicHeader()->getEnableTMVPFlag() )
    {
      if ( pcSlice->getSliceType() == B_SLICE )
      {
        if (!pps->getPPSCollocatedFromL0Idc())
        {
          READ_FLAG(uiCode, "collocated_from_l0_flag");
        }
        else
        {
          uiCode = pps->getPPSCollocatedFromL0Idc() - 1;
        }
        pcSlice->setColFromL0Flag(uiCode);
      }
      else
      {
        pcSlice->setColFromL0Flag( 1 );
      }

      if ( pcSlice->getSliceType() != I_SLICE &&
           ((pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx(REF_PIC_LIST_0) > 1)||
           (pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx(REF_PIC_LIST_1) > 1)))
      {
        READ_UVLC( uiCode, "collocated_ref_idx" );
        pcSlice->setColRefIdx(uiCode);
      }
      else
      {
        pcSlice->setColRefIdx(0);
      }
    }
    if ( (pps->getUseWP() && pcSlice->getSliceType()==P_SLICE) || (pps->getWPBiPred() && pcSlice->getSliceType()==B_SLICE) )
    {
      parsePredWeightTable(pcSlice, sps);
      pcSlice->initWpScaling(sps);
    }
    else
    {
      WPScalingParam *wp;
      for ( int iNumRef=0 ; iNumRef<((pcSlice->getSliceType() == B_SLICE )?2:1); iNumRef++ )
      {
        RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
        for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
        {
          pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
          wp[0].bPresentFlag = false;
          wp[1].bPresentFlag = false;
          wp[2].bPresentFlag = false;
        }
      }
    }


    READ_SVLC( iCode, "slice_qp_delta" );
    pcSlice->setSliceQp (26 + pps->getPicInitQPMinus26() + iCode);
    pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

    CHECK( pcSlice->getSliceQp() < -sps->getQpBDOffset(CHANNEL_TYPE_LUMA), "Invalid slice QP delta" );
    CHECK( pcSlice->getSliceQp() > MAX_QP, "Invalid slice QP" );

    if (pps->getSliceChromaQpFlag())
    {
      if (numValidComp>COMPONENT_Cb)
      {
        READ_SVLC( iCode, "slice_cb_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cb, iCode );
        CHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) < -12, "Invalid chroma QP offset" );
        CHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) >  12, "Invalid chroma QP offset" );
        CHECK( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) < -12, "Invalid chroma QP offset" );
        CHECK( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) >  12, "Invalid chroma QP offset" );
      }

      if (numValidComp>COMPONENT_Cr)
      {
        READ_SVLC( iCode, "slice_cr_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cr, iCode );
        CHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) < -12, "Invalid chroma QP offset" );
        CHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) >  12, "Invalid chroma QP offset" );
        CHECK( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) < -12, "Invalid chroma QP offset" );
        CHECK( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) >  12, "Invalid chroma QP offset" );
        if (sps->getJointCbCrEnabledFlag())
        {
          READ_SVLC(iCode, "slice_joint_cbcr_qp_offset" );
          pcSlice->setSliceChromaQpDelta(JOINT_CbCr, iCode);
          CHECK( pcSlice->getSliceChromaQpDelta(JOINT_CbCr) < -12, "Invalid chroma QP offset");
          CHECK( pcSlice->getSliceChromaQpDelta(JOINT_CbCr) >  12, "Invalid chroma QP offset");
          CHECK( (pps->getQpOffset(JOINT_CbCr) + pcSlice->getSliceChromaQpDelta(JOINT_CbCr)) < -12, "Invalid chroma QP offset");
          CHECK( (pps->getQpOffset(JOINT_CbCr) + pcSlice->getSliceChromaQpDelta(JOINT_CbCr)) >  12, "Invalid chroma QP offset");
        }
      }
    }

    if (pps->getCuChromaQpOffsetEnabledFlag())
    {
      READ_FLAG(uiCode, "cu_chroma_qp_offset_enabled_flag"); pcSlice->setUseChromaQpAdj(uiCode != 0);
    }
    else
    {
      pcSlice->setUseChromaQpAdj(false);
    }

    if( sps->getSAOEnabledFlag() && !picHeader->getSaoEnabledPresentFlag() )
    {
      READ_FLAG(uiCode, "slice_sao_luma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, (bool)uiCode);

      if (bChroma)
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, (bool)uiCode);
      }
    }

    if( sps->getALFEnabledFlag() && !picHeader->getAlfEnabledPresentFlag() )
    {
      READ_FLAG(uiCode, "slice_alf_enabled_flag");
      pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Y, uiCode);
      int alfChromaIdc = 0;
      if (uiCode)
      {
        READ_CODE(3, uiCode, "slice_num_alf_aps_ids_luma");
        int numAps = uiCode;
        pcSlice->setTileGroupNumAps(numAps);
        std::vector<int> apsId(numAps, -1);
        for (int i = 0; i < numAps; i++)
        {
          READ_CODE(3, uiCode, "slice_alf_aps_id_luma");
          apsId[i] = uiCode;
          APS* APStoCheckLuma = parameterSetManager->getAPS(apsId[i], ALF_APS);
          CHECK(APStoCheckLuma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] != 1, "bitstream conformance error, alf_luma_filter_signal_flag shall be equal to 1");
        }


        pcSlice->setAlfAPSs(apsId);
        if (bChroma)
        {
          READ_CODE(2, uiCode, "slice_alf_chroma_idc");   alfChromaIdc = uiCode;
        }
        else
        {
          alfChromaIdc = 0;
        }
        if (alfChromaIdc)
        {
          READ_CODE(3, uiCode, "slice_alf_aps_id_chroma");
          pcSlice->setTileGroupApsIdChroma(uiCode);
          APS* APStoCheckChroma = parameterSetManager->getAPS(uiCode, ALF_APS);
          CHECK(APStoCheckChroma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] != 1, "bitstream conformance error, alf_chroma_filter_signal_flag shall be equal to 1");
        }
      }
      else
      {
        pcSlice->setTileGroupNumAps(0);
      }
      pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cb, alfChromaIdc & 1);
      pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cr, alfChromaIdc >> 1);
    }


    if (pps->getDeblockingFilterControlPresentFlag())
    {
      if( pps->getDeblockingFilterOverrideEnabledFlag() && !picHeader->getDeblockingFilterOverridePresentFlag() )
      {
        READ_FLAG ( uiCode, "slice_deblocking_filter_override_flag" );        pcSlice->setDeblockingFilterOverrideFlag(uiCode ? true : false);
      }
      else
      {
        pcSlice->setDeblockingFilterOverrideFlag(0);
      }
      if(pcSlice->getDeblockingFilterOverrideFlag())
      {
        READ_FLAG ( uiCode, "slice_deblocking_filter_disabled_flag" );   pcSlice->setDeblockingFilterDisable(uiCode ? 1 : 0);
        if(!pcSlice->getDeblockingFilterDisable())
        {
          READ_SVLC( iCode, "slice_beta_offset_div2" );                       pcSlice->setDeblockingFilterBetaOffsetDiv2(iCode);
          CHECK(  pcSlice->getDeblockingFilterBetaOffsetDiv2() < -6 &&
                  pcSlice->getDeblockingFilterBetaOffsetDiv2() >  6, "Invalid deblocking filter configuration");
          READ_SVLC( iCode, "slice_tc_offset_div2" );                         pcSlice->setDeblockingFilterTcOffsetDiv2(iCode);
          CHECK  (pcSlice->getDeblockingFilterTcOffsetDiv2() < -6 &&
                  pcSlice->getDeblockingFilterTcOffsetDiv2() >  6, "Invalid deblocking filter configuration");
        }
      }
      else
      {
        pcSlice->setDeblockingFilterDisable       ( picHeader->getDeblockingFilterDisable() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( picHeader->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( picHeader->getDeblockingFilterTcOffsetDiv2() );
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable       ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
    }



  if( pcSlice->getFirstCtuRsAddrInSlice() == 0 )
  {
    pcSlice->setDefaultClpRng( *sps );

  }

  if(pps->getSliceHeaderExtensionPresentFlag())
  {
    READ_UVLC(uiCode,"slice_segment_header_extension_length");
    for(int i=0; i<uiCode; i++)
    {
      uint32_t ignore_;
      READ_CODE(8,ignore_,"slice_segment_header_extension_data_byte");
    }
  }

  std::vector<uint32_t> entryPointOffset;

  pcSlice->setNumEntryPoints( pps );
  if( pcSlice->getNumEntryPoints() > 0 )
  {
    uint32_t offsetLenMinus1;
    READ_UVLC( offsetLenMinus1, "offset_len_minus1" );
    entryPointOffset.resize( pcSlice->getNumEntryPoints() );
    for( uint32_t idx = 0; idx < pcSlice->getNumEntryPoints(); idx++ )
    {
      READ_CODE( offsetLenMinus1 + 1, uiCode, "entry_point_offset_minus1" );
      entryPointOffset[idx] = uiCode + 1;
    }
  }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
#else
  m_pcBitstream->readByteAlignment();
#endif

  pcSlice->clearSubstreamSizes();

  if( pcSlice->getNumEntryPoints() > 0 )
  {
    int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for ( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
      {
        endOfSliceHeaderLocation++;
      }
    }

    int  curEntryPointOffset     = 0;
    int  prevEntryPointOffset    = 0;
    for (uint32_t idx=0; idx<entryPointOffset.size(); idx++)
    {
      curEntryPointOffset += entryPointOffset[ idx ];

      int emulationPreventionByteCount = 0;
      for ( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
      {
        if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= ( prevEntryPointOffset + endOfSliceHeaderLocation ) &&
             m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) <  ( curEntryPointOffset  + endOfSliceHeaderLocation ) )
        {
          emulationPreventionByteCount++;
        }
      }

      entryPointOffset[ idx ] -= emulationPreventionByteCount;
      prevEntryPointOffset = curEntryPointOffset;
      pcSlice->addSubstreamSize(entryPointOffset [ idx ] );
    }
  }
  return;
}

void HLSyntaxReader::parseSliceHeaderToPoc (Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager, const int prevTid0POC)
{
  uint32_t  uiCode;
  PPS* pps = NULL;
  SPS* sps = NULL;

  CHECK(picHeader==0, "Invalid Picture Header");
  CHECK(picHeader->isValid()==false, "Invalid Picture Header");
  pps = parameterSetManager->getPPS( picHeader->getPPSId() );
  //!KS: need to add error handling code here, if PPS is not available
  CHECK(pps==0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  CHECK(sps==0, "Invalid SPS");
  
  // picture order count
  READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
  if (pcSlice->getIdrPicFlag())
  {
    pcSlice->setPOC(uiCode);
  }
  else
  {
    int iPOClsb = uiCode;
    int iPrevPOC = prevTid0POC;
    int iMaxPOClsb = 1 << sps->getBitsForPOC();
    int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
    int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
    int iPOCmsb;
    if ((iPOClsb  <  iPrevPOClsb) && ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2)))
    {
      iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
    }
    else if ((iPOClsb  >  iPrevPOClsb) && ((iPOClsb - iPrevPOClsb)  >  (iMaxPOClsb / 2)))
    {
      iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
    }
    else
    {
      iPOCmsb = iPrevPOCmsb;
    }
    pcSlice->setPOC(iPOCmsb + iPOClsb);
  }
}

void HLSyntaxReader::parseConstraintInfo(ConstraintInfo *cinfo)
{
  uint32_t symbol;
  READ_FLAG(symbol,  "general_progressive_source_flag"          ); cinfo->setProgressiveSourceFlag(symbol ? true : false);
  READ_FLAG(symbol,  "general_interlaced_source_flag"           ); cinfo->setInterlacedSourceFlag(symbol ? true : false);
  READ_FLAG(symbol,  "general_non_packed_constraint_flag"       ); cinfo->setNonPackedConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "general_frame_only_constraint_flag"       ); cinfo->setFrameOnlyConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "intra_only_constraint_flag"               ); cinfo->setIntraOnlyConstraintFlag(symbol ? true : false);

  READ_CODE(4, symbol,  "max_bitdepth_constraint_idc"              ); cinfo->setMaxBitDepthConstraintIdc(symbol);
  READ_CODE(2, symbol,  "max_chroma_format_constraint_idc"         ); cinfo->setMaxChromaFormatConstraintIdc((ChromaFormat)symbol);

  READ_FLAG(symbol,  "no_qtbtt_dual_tree_intra_constraint_flag" ); cinfo->setNoQtbttDualTreeIntraConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_partition_constraints_override_constraint_flag"); cinfo->setNoPartitionConstraintsOverrideConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_sao_constraint_flag");                    cinfo->setNoSaoConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_alf_constraint_flag");                    cinfo->setNoAlfConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_joint_cbcr_constraint_flag");             cinfo->setNoJointCbCrConstraintFlag(symbol > 0 ? true : false);

  READ_FLAG(symbol,  "no_ref_wraparound_constraint_flag");         cinfo->setNoRefWraparoundConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_temporal_mvp_constraint_flag");           cinfo->setNoTemporalMvpConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_sbtmvp_constraint_flag");                 cinfo->setNoSbtmvpConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_amvr_constraint_flag");                   cinfo->setNoAmvrConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_bdof_constraint_flag");                   cinfo->setNoBdofConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_dmvr_constraint_flag");                    cinfo->setNoDmvrConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_cclm_constraint_flag");                    cinfo->setNoCclmConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_mts_constraint_flag");                     cinfo->setNoMtsConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_sbt_constraint_flag");                     cinfo->setNoSbtConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_affine_motion_constraint_flag");           cinfo->setNoAffineMotionConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_bcw_constraint_flag");                     cinfo->setNoBcwConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_ibc_constraint_flag");                     cinfo->setNoIbcConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_ciip_constraint_flag");                    cinfo->setNoCiipConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_fpel_mmvd_constraint_flag");               cinfo->setNoFPelMmvdConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_triangle_constraint_flag");                cinfo->setNoTriangleConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_ladf_constraint_flag");                    cinfo->setNoLadfConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_transform_skip_constraint_flag");          cinfo->setNoTransformSkipConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_bdpcm_constraint_flag");                   cinfo->setNoBDPCMConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_qp_delta_constraint_flag");                cinfo->setNoQpDeltaConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_dep_quant_constraint_flag");               cinfo->setNoDepQuantConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_sign_data_hiding_constraint_flag");        cinfo->setNoSignDataHidingConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_trail_constraint_flag");                   cinfo->setNoTrailConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_stsa_constraint_flag");                    cinfo->setNoStsaConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_rasl_constraint_flag");                    cinfo->setNoRaslConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_radl_constraint_flag");                    cinfo->setNoRadlConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_idr_constraint_flag");                     cinfo->setNoIdrConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_cra_constraint_flag");                     cinfo->setNoCraConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_gdr_constraint_flag");                     cinfo->setNoGdrConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_aps_constraint_flag");                     cinfo->setNoApsConstraintFlag(symbol > 0 ? true : false);
}


void HLSyntaxReader::parseProfileTierLevel(ProfileTierLevel *ptl, int maxNumSubLayersMinus1)
{
  uint32_t symbol;
  READ_CODE(7 , symbol,   "general_profile_idc"              ); ptl->setProfileIdc  (Profile::Name(symbol));
  READ_FLAG(    symbol,   "general_tier_flag"                ); ptl->setTierFlag    (symbol ? Level::HIGH : Level::MAIN);

  parseConstraintInfo( ptl->getConstraintInfo() );

  READ_CODE( 8, symbol, "general_level_idc" ); ptl->setLevelIdc( Level::Name( symbol ) );

  READ_CODE(8, symbol, "num_sub_profiles");
  uint8_t numSubProfiles = symbol;
  ptl->setNumSubProfile( numSubProfiles );
  for (int i = 0; i < numSubProfiles; i++)
  {
    READ_CODE(32, symbol, "general_sub_profile_idc[i]"); ptl->setSubProfileIdc(i, symbol);
  }


  for (int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    READ_FLAG( symbol, "sub_layer_level_present_flag[i]"   ); ptl->setSubLayerLevelPresentFlag  (i, symbol);
  }

  while (!isByteAligned())
  {
    READ_FLAG(    symbol,   "ptl_alignment_zero_bit"        ); CHECK (symbol != 0, "ptl_alignment_zero_bit not equal to zero");
  }

  for (int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if (ptl->getSubLayerLevelPresentFlag(i))
    {
      READ_CODE(8 , symbol,   "sub_layer_level_idc"                ); ptl->setSubLayerLevelIdc    (i, Level::Name(symbol));
    }
  }
  ptl->setSubLayerLevelIdc(maxNumSubLayersMinus1, ptl->getLevelIdc());
  for( int i = maxNumSubLayersMinus1 - 1; i >= 0; i-- )
  {
    if( !ptl->getSubLayerLevelPresentFlag( i ) )
    {
      ptl->setSubLayerLevelIdc( i, ptl->getSubLayerLevelIdc( i + 1 ) );
    }
  }
}



void HLSyntaxReader::parseTerminatingBit( uint32_t& ruiBit )
{
  ruiBit = false;
  int iBitsLeft = m_pcBitstream->getNumBitsLeft();
  if(iBitsLeft <= 8)
  {
    uint32_t uiPeekValue = m_pcBitstream->peekBits(iBitsLeft);
    if (uiPeekValue == (1<<(iBitsLeft-1)))
    {
      ruiBit = true;
    }
  }
}

void HLSyntaxReader::parseRemainingBytes( bool noTrailingBytesExpected )
{
  if (noTrailingBytesExpected)
  {
    CHECK( 0 != m_pcBitstream->getNumBitsLeft(), "Bits left although no bits expected" );
  }
  else
  {
    while (m_pcBitstream->getNumBitsLeft())
    {
      uint32_t trailingNullByte=m_pcBitstream->readByte();
      if (trailingNullByte!=0)
      {
        msg( ERROR, "Trailing byte should be 0, but has value %02x\n", trailingNullByte);
        THROW("Invalid trailing '0' byte");
      }
    }
  }
}




// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
void HLSyntaxReader::parsePredWeightTable( Slice* pcSlice, const SPS *sps )
{
  WPScalingParam *wp;
  const ChromaFormat    chFmt        = sps->getChromaFormatIdc();
  const int             numValidComp = int(getNumberValidComponents(chFmt));
  const bool            bChroma      = (chFmt!=CHROMA_400);
  const SliceType       eSliceType   = pcSlice->getSliceType();
  const int             iNbRef       = (eSliceType == B_SLICE ) ? (2) : (1);
  uint32_t            uiLog2WeightDenomLuma=0, uiLog2WeightDenomChroma=0;
  uint32_t            uiTotalSignalledWeightFlags = 0;

  int iDeltaDenom;
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( uiLog2WeightDenomLuma, "luma_log2_weight_denom" );
  CHECK( uiLog2WeightDenomLuma > 7, "Invalid code" );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    CHECK((iDeltaDenom + (int)uiLog2WeightDenomLuma)<0, "Invalid code");
    CHECK((iDeltaDenom + (int)uiLog2WeightDenomLuma)>7, "Invalid code");
    uiLog2WeightDenomChroma = (uint32_t)(iDeltaDenom + uiLog2WeightDenomLuma);
  }

  for ( int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
  {
    RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[COMPONENT_Y].uiLog2WeightDenom = uiLog2WeightDenomLuma;
      for(int j=1; j<numValidComp; j++)
      {
        wp[j].uiLog2WeightDenom = uiLog2WeightDenomChroma;
      }

      uint32_t  uiCode;
      READ_FLAG( uiCode, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
      wp[COMPONENT_Y].bPresentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if ( bChroma )
    {
      uint32_t  uiCode;
      for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        READ_FLAG( uiCode, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
        for(int j=1; j<numValidComp; j++)
        {
          wp[j].bPresentFlag = ( uiCode == 1 );
        }
        uiTotalSignalledWeightFlags += 2*wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
      if ( wp[COMPONENT_Y].bPresentFlag )
      {
        int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
        CHECK( iDeltaWeight < -128, "Invalid code" );
        CHECK( iDeltaWeight >  127, "Invalid code" );
        wp[COMPONENT_Y].iWeight = (iDeltaWeight + (1<<wp[COMPONENT_Y].uiLog2WeightDenom));
        READ_SVLC( wp[COMPONENT_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        const int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_LUMA))/2 : 128;
        if( wp[0].iOffset < -range ) { THROW("Offset out of range"); }
        if( wp[0].iOffset >= range ) { THROW("Offset out of range"); }
      }
      else
      {
        wp[COMPONENT_Y].iWeight = (1 << wp[COMPONENT_Y].uiLog2WeightDenom);
        wp[COMPONENT_Y].iOffset = 0;
      }
      if ( bChroma )
      {
        if ( wp[COMPONENT_Cb].bPresentFlag )
        {
          int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_CHROMA))/2 : 128;
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            int iDeltaWeight;
            READ_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );
            CHECK( iDeltaWeight < -128, "Invalid code" );
            CHECK( iDeltaWeight >  127, "Invalid code" );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[j].uiLog2WeightDenom));

            int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            CHECK( iDeltaChroma <  -4*range, "Invalid code" );
            CHECK( iDeltaChroma >=  4*range, "Invalid code" );
            int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].uiLog2WeightDenom) ) );
            wp[j].iOffset = Clip3(-range, range-1, (iDeltaChroma + pred) );
          }
        }
        else
        {
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            wp[j].iWeight = (1 << wp[j].uiLog2WeightDenom);
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for ( int iRefIdx=pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }
  }
  CHECK(uiTotalSignalledWeightFlags>24, "Too many weight flag signalled");
}

/** decode quantization matrix
* \param scalingList quantization matrix information
*/
void HLSyntaxReader::parseScalingList(ScalingList* scalingList)
{
  uint32_t  code;
  bool scalingListCopyModeFlag;
  READ_FLAG(code, "scaling_matrix_for_lfnst_disabled_flag"); scalingList->setDisableScalingMatrixForLfnstBlks(code ? true : false);
  for (int scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    READ_FLAG(code, "scaling_list_copy_mode_flag");
    scalingListCopyModeFlag = (code) ? true : false;
    scalingList->setScalingListCopyModeFlag(scalingListId, scalingListCopyModeFlag);

    scalingList->setScalingListPreditorModeFlag(scalingListId, false);
    if (!scalingListCopyModeFlag)
    {
      READ_FLAG(code, "scaling_list_predictor_mode_flag");
      scalingList->setScalingListPreditorModeFlag(scalingListId, code);
    }

    if ((scalingListCopyModeFlag || scalingList->getScalingListPreditorModeFlag(scalingListId)) && scalingListId!= SCALING_LIST_1D_START_2x2 && scalingListId!= SCALING_LIST_1D_START_4x4 && scalingListId!= SCALING_LIST_1D_START_8x8) //Copy Mode
    {
      READ_UVLC(code, "scaling_list_pred_matrix_id_delta");
      scalingList->setRefMatrixId(scalingListId, (uint32_t)((int)(scalingListId)-(code)));
    }    
    else if (scalingListCopyModeFlag || scalingList->getScalingListPreditorModeFlag(scalingListId))
    {
      scalingList->setRefMatrixId(scalingListId, (uint32_t)((int)(scalingListId)));
    }
    if (scalingListCopyModeFlag)//copy
    {
      if (scalingListId >= SCALING_LIST_1D_START_16x16)
      {
        scalingList->setScalingListDC(scalingListId,
          ((scalingListId == scalingList->getRefMatrixId(scalingListId)) ? 16
            : (scalingList->getRefMatrixId(scalingListId) < SCALING_LIST_1D_START_16x16) ? scalingList->getScalingListAddress(scalingList->getRefMatrixId(scalingListId))[0] : scalingList->getScalingListDC(scalingList->getRefMatrixId(scalingListId))));
      }
      scalingList->processRefMatrix(scalingListId, scalingList->getRefMatrixId(scalingListId));
    }
    else
    {
      decodeScalingList(scalingList, scalingListId, scalingList->getScalingListPreditorModeFlag(scalingListId));
    }
  }

  return;
}

/** decode DPCM
* \param scalingList  quantization matrix information
* \param sizeId size index
* \param listId list index
*/
void HLSyntaxReader::decodeScalingList(ScalingList *scalingList, uint32_t scalingListId, bool isPredictor)
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  int i, coefNum = matrixSize * matrixSize;
  int data;
  int scalingListDcCoefMinus8 = 0;
  int nextCoef = (isPredictor) ? 0 : SCALING_LIST_START_VALUE;
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)][gp_sizeIdxInfo->idxFrom(matrixSize)];
  int *dst = scalingList->getScalingListAddress(scalingListId);

  int PredListId = scalingList->getRefMatrixId(scalingListId);
  CHECK(isPredictor && PredListId > scalingListId, "Scaling List error predictor!");
  const int *srcPred = (isPredictor) ? ((scalingListId == PredListId) ? scalingList->getScalingListDefaultAddress(scalingListId) : scalingList->getScalingListAddress(PredListId)) : NULL;
  if(isPredictor && scalingListId == PredListId)
    scalingList->setScalingListDC(PredListId, SCALING_LIST_DC);
  int predCoef = 0;

  if (scalingListId >= SCALING_LIST_1D_START_16x16)
  {
    READ_SVLC(scalingListDcCoefMinus8, "scaling_list_dc_coef_minus8");
    nextCoef += scalingListDcCoefMinus8;
    if (isPredictor)
    {
      predCoef = (PredListId >= SCALING_LIST_1D_START_16x16) ? scalingList->getScalingListDC(PredListId) : srcPred[0];
    }
    scalingList->setScalingListDC(scalingListId, (nextCoef + predCoef + 256) & 255);
  }

  for(i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
    {
      dst[scan[i].idx] = 0;
      continue;
    }
    READ_SVLC( data, "scaling_list_delta_coef");
    nextCoef += data;  
    predCoef = (isPredictor) ? srcPred[scan[i].idx] : 0;
    dst[scan[i].idx] = (nextCoef + predCoef + 256) & 255;
  }
}

bool HLSyntaxReader::xMoreRbspData()
{
  int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // if there are more than 8 bits, it cannot be rbsp_trailing_bits
  if (bitsLeft > 8)
  {
    return true;
  }

  uint8_t lastByte = m_pcBitstream->peekBits(bitsLeft);
  int cnt = bitsLeft;

  // remove trailing bits equal to zero
  while ((cnt>0) && ((lastByte & 1) == 0))
  {
    lastByte >>= 1;
    cnt--;
  }
  // remove bit equal to one
  cnt--;

  // we should not have a negative number of bits
  CHECK (cnt<0, "Negative number of bits");

  // we have more data, if cnt is not zero
  return (cnt>0);
}

int HLSyntaxReader::alfGolombDecode( const int k, const bool signed_val )
{
  int numLeadingBits = -1;
  uint32_t b = 0;
  for (; !b; numLeadingBits++)
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( b, "" );
#else
    READ_FLAG( b, "alf_coeff_abs_prefix");
#endif
  }

  int symbol = ( ( 1 << numLeadingBits ) - 1 ) << k;
  if ( numLeadingBits + k > 0)
  {
    uint32_t bins;
    READ_CODE( numLeadingBits + k, bins, "alf_coeff_abs_suffix" );
    symbol += bins;
  }

  if ( signed_val && symbol != 0 )
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( b, "" );
#else
    READ_FLAG( b, "alf_coeff_sign" );
#endif
    symbol = ( b ) ? -symbol : symbol;
  }
  return symbol;
}

void HLSyntaxReader::alfFilter( AlfParam& alfParam, const bool isChroma, const int altIdx )
{
  uint32_t code;

  // derive maxGolombIdx
  AlfFilterShape alfShape( isChroma ? 5 : 7 );
  const int numFilters = isChroma ? 1 : alfParam.numLumaFilters;
  short* coeff = isChroma ? alfParam.chromaCoeff[altIdx] : alfParam.lumaCoeff;
  short* clipp = isChroma ? alfParam.chromaClipp[altIdx] : alfParam.lumaClipp;


  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode( 3 );
      CHECK( isChroma &&
             ( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] > 127 || coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] < -127 )
             , "AlfCoeffC shall be in the range of -127 to 127, inclusive" );
    }
  }

  // Clipping values coding
  if ( alfParam.nonLinearFlag[isChroma][altIdx] )
  {

    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {

      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        READ_CODE(2, code, "alf_clipping_index");
        clipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = code;
      }
    }
  }
  else
  {
    for( int ind = 0; ind < numFilters; ++ind )
    {
      std::fill_n( clipp + ind * MAX_NUM_ALF_LUMA_COEFF, alfShape.numCoeff, 0 );
    }
  }
}


//! \}

