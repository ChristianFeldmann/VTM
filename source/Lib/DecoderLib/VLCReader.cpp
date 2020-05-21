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

#if JVET_R0059_RPL_CLEANUP
void HLSyntaxReader::parseRefPicList(SPS* sps, ReferencePictureList* rpl, int rplIdx)
#else
void HLSyntaxReader::parseRefPicList(SPS* sps, ReferencePictureList* rpl)
#endif
{
  uint32_t code;
  READ_UVLC(code, "num_ref_entries[ listIdx ][ rplsIdx ]");
  uint32_t numRefPic = code;
  uint32_t numStrp = 0;
  uint32_t numLtrp = 0;
  uint32_t numIlrp = 0;

#if JVET_R0059_RPL_CLEANUP
  if (sps->getLongTermRefsPresent() && rplIdx != -1)
#else
  if (sps->getLongTermRefsPresent())
#endif
  {
    READ_FLAG(code, "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
    rpl->setLtrpInSliceHeaderFlag(code);
  }
#if JVET_R0059_RPL_CLEANUP
  else if(sps->getLongTermRefsPresent())
  {
    rpl->setLtrpInSliceHeaderFlag( 1 );
  }
#endif

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
      if ((!sps->getUseWP() && !sps->getUseWPBiPred()) || (ii == 0))
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

void HLSyntaxReader::parsePPS( PPS* pcPPS )
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif
  uint32_t  uiCode;

  int   iCode;
#if JVET_R0266_DESC
  READ_CODE(6, uiCode, "pps_pic_parameter_set_id");
#else
  READ_UVLC( uiCode, "pps_pic_parameter_set_id");
#endif
  CHECK(uiCode > 63, "PPS id exceeds boundary (63)");
  pcPPS->setPPSId (uiCode);

  READ_CODE(4, uiCode, "pps_seq_parameter_set_id");
  pcPPS->setSPSId (uiCode);

  READ_FLAG( uiCode, "mixed_nalu_types_in_pic_flag" );       pcPPS->setMixedNaluTypesInPicFlag( uiCode == 1 );

  READ_UVLC( uiCode, "pic_width_in_luma_samples" );          pcPPS->setPicWidthInLumaSamples( uiCode );
  READ_UVLC( uiCode, "pic_height_in_luma_samples" );         pcPPS->setPicHeightInLumaSamples( uiCode );
  READ_FLAG(uiCode, "pps_conformance_window_flag");
  if (uiCode != 0)
  {
    Window& conf = pcPPS->getConformanceWindow();
    READ_UVLC(uiCode, "pps_conf_win_left_offset");               conf.setWindowLeftOffset(uiCode);
    READ_UVLC(uiCode, "pps_conf_win_right_offset");              conf.setWindowRightOffset(uiCode);
    READ_UVLC(uiCode, "pps_conf_win_top_offset");                conf.setWindowTopOffset(uiCode);
    READ_UVLC(uiCode, "pps_conf_win_bottom_offset");             conf.setWindowBottomOffset(uiCode);
  }
  READ_FLAG( uiCode, "scaling_window_flag" );
  if( uiCode != 0 )
  {
    Window &scalingWindow = pcPPS->getScalingWindow();
#if JVET_R0114_NEGATIVE_SCALING_WINDOW_OFFSETS
    READ_SVLC( iCode, "pps_scaling_win_left_offset" );               scalingWindow.setWindowLeftOffset( iCode );
    READ_SVLC( iCode, "pps_scaling_win_right_offset" );              scalingWindow.setWindowRightOffset( iCode );
    READ_SVLC( iCode, "pps_scaling_win_top_offset" );                scalingWindow.setWindowTopOffset( iCode );
    READ_SVLC( iCode, "pps_scaling_win_bottom_offset" );             scalingWindow.setWindowBottomOffset( iCode );
#else
    READ_UVLC( uiCode, "scaling_win_left_offset" );               scalingWindow.setWindowLeftOffset( uiCode );
    READ_UVLC( uiCode, "scaling_win_right_offset" );              scalingWindow.setWindowRightOffset( uiCode );
    READ_UVLC( uiCode, "scaling_win_top_offset" );                scalingWindow.setWindowTopOffset( uiCode );
    READ_UVLC( uiCode, "scaling_win_bottom_offset" );             scalingWindow.setWindowBottomOffset( uiCode );
#endif
  }
  else
  {
    Window &scalingWindow = pcPPS->getScalingWindow();
    Window& conf = pcPPS->getConformanceWindow();
    scalingWindow.setWindowLeftOffset( conf.getWindowLeftOffset() );
    scalingWindow.setWindowRightOffset( conf.getWindowRightOffset() );
    scalingWindow.setWindowTopOffset( conf.getWindowTopOffset() );
    scalingWindow.setWindowBottomOffset( conf.getWindowBottomOffset() );
  }

  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

#if JVET_R0186_CLEANUP
  READ_FLAG( uiCode, "pps_no_pic_partition_flag");                     pcPPS->setNoPicPartitionFlag(uiCode == 1);
#endif  
  READ_FLAG( uiCode, "subpic_id_mapping_in_pps_flag" );           pcPPS->setSubPicIdMappingInPpsFlag( uiCode != 0 );
  if( pcPPS->getSubPicIdMappingInPpsFlag() )
  {
#if JVET_R0186_CLEANUP
    if( !pcPPS->getNoPicPartitionFlag() )
    {
      READ_UVLC(uiCode, "pps_num_subpics_minus1");                         pcPPS->setNumSubPics(uiCode + 1);
    }
    else
    {
      pcPPS->setNumSubPics(1);
    }
#else
    READ_UVLC( uiCode, "pps_num_subpics_minus1" );                         pcPPS->setNumSubPics( uiCode + 1 );
#endif
    CHECK( uiCode > MAX_NUM_SUB_PICS-1,  "Number of sub-pictures exceeds limit");

    READ_UVLC( uiCode, "pps_subpic_id_len_minus1" );                       pcPPS->setSubPicIdLen( uiCode + 1 );
    CHECK( uiCode > 15, "Invalid pps_subpic_id_len_minus1 signalled");

    CHECK((1 << pcPPS->getSubPicIdLen()) < pcPPS->getNumSubPics(), "pps_subpic_id_len exceeds valid range");
    for( int picIdx = 0; picIdx < pcPPS->getNumSubPics( ); picIdx++ )
    {
      READ_CODE( pcPPS->getSubPicIdLen( ), uiCode, "pps_subpic_id[i]" );   pcPPS->setSubPicId( picIdx, uiCode );
    }
  }
#if !JVET_R0186_CLEANUP
  READ_FLAG( uiCode, "no_pic_partition_flag" );                       pcPPS->setNoPicPartitionFlag( uiCode == 1 );
#endif
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
      CHECK(uiCode  > (pcPPS->getPicWidthInCtu()-1),                 "The value of tile_column_width_minus1[i] shall be in the range of 0 to PicWidthInCtbY-1, inclusive");
    }
    for( rowIdx = 0; rowIdx < pcPPS->getNumExpTileRows(); rowIdx++ )
    {
      READ_UVLC( uiCode, "tile_row_height_minus1[i]" );               pcPPS->addTileRowHeight( uiCode + 1 );
      CHECK(uiCode > (pcPPS->getPicHeightInCtu() - 1),                "The value of tile_row_height_minus shall be in the range of 0 to PicHeightInCtbY-1, inclusive");
    }
    pcPPS->initTiles();
    // rectangular slice signalling
    if (pcPPS->getNumTiles() > 1)
    {
#if JVET_R0113_AND_JVET_R0106_PPS_CLEANUP
      READ_CODE(1, uiCode, "pps_loop_filter_across_tiles_enabled_flag");    pcPPS->setLoopFilterAcrossTilesEnabledFlag(uiCode == 1);
#endif
      READ_CODE(1, uiCode, "rect_slice_flag");
    }
    else
    {
#if JVET_R0113_AND_JVET_R0106_PPS_CLEANUP
      pcPPS->setLoopFilterAcrossTilesEnabledFlag(true);
#endif
      uiCode = 1;
    }
    pcPPS->setRectSliceFlag(uiCode == 1);
    if (pcPPS->getRectSliceFlag())
    {
      READ_FLAG(uiCode, "single_slice_per_subpic_flag");            pcPPS->setSingleSlicePerSubPicFlag(uiCode == 1);
    }
    else
    {
      pcPPS->setSingleSlicePerSubPicFlag(0);
    }
    if (pcPPS->getRectSliceFlag() & !(pcPPS->getSingleSlicePerSubPicFlag()))
    {
      int32_t tileIdx = 0;

      READ_UVLC( uiCode, "num_slices_in_pic_minus1" );                pcPPS->setNumSlicesInPic( uiCode + 1 );
      CHECK(pcPPS->getNumSlicesInPic() > MAX_SLICES,                  "Number of slices in picture exceeds valid range");

      if ((pcPPS->getNumSlicesInPic() - 1) > 0)
      {
        READ_CODE(1, uiCode, "tile_idx_delta_present_flag");
        pcPPS->setTileIdxDeltaPresentFlag(uiCode == 1);
      }
      else
      {
        pcPPS->setTileIdxDeltaPresentFlag(0);
      }
      pcPPS->initRectSlices();

      // read rectangular slice parameters
      for( int i = 0; i < pcPPS->getNumSlicesInPic()-1; i++ )
      {
        pcPPS->setSliceTileIdx( i, tileIdx );

        // complete tiles within a single slice
#if JVET_R0188
        if( ( tileIdx % pcPPS->getNumTileColumns() ) != pcPPS->getNumTileColumns() - 1 )
        {
          READ_UVLC( uiCode, "slice_width_in_tiles_minus1[i]" );
          pcPPS->setSliceWidthInTiles ( i, uiCode + 1 );
        }
        else
        {
          pcPPS->setSliceWidthInTiles( i, 1 );
        }
#else
        uiCode = 0;
        if( pcPPS->getNumTileColumns() > 1 )
        {
          READ_UVLC( uiCode, "slice_width_in_tiles_minus1[i]" );
        }
        pcPPS->setSliceWidthInTiles ( i, uiCode + 1 );
#endif

#if JVET_R0188
        if( tileIdx / pcPPS->getNumTileColumns() != pcPPS->getNumTileRows() - 1  &&
         ( pcPPS->getTileIdxDeltaPresentFlag() || tileIdx % pcPPS->getNumTileColumns() == 0 ) ) 
        {
          READ_UVLC( uiCode, "slice_height_in_tiles_minus1[i]" );
          pcPPS->setSliceHeightInTiles( i, uiCode + 1 );
        }
        else
        {
          if( ( tileIdx / pcPPS->getNumTileColumns() ) == pcPPS->getNumTileRows() - 1 )
          {
            pcPPS->setSliceHeightInTiles( i, 1 );
          }
          else
          {
            pcPPS->setSliceHeightInTiles( i, pcPPS->getSliceHeightInTiles( i - 1 ) );
          }
      }
#else
        if( pcPPS->getTileIdxDeltaPresentFlag() || ( (tileIdx % pcPPS->getNumTileColumns()) == 0 ) )
        {
          uiCode = 0;
          if( pcPPS->getNumTileRows() > 1 )
          {
            READ_UVLC( uiCode, "slice_height_in_tiles_minus1[i]" );
          }
          pcPPS->setSliceHeightInTiles( i, uiCode + 1 );
        }
        else
        {
          pcPPS->setSliceHeightInTiles( i, pcPPS->getSliceHeightInTiles(i-1) );
        }
#endif

        // multiple slices within a single tile special case
        if( pcPPS->getSliceWidthInTiles(i) == 1 && pcPPS->getSliceHeightInTiles(i) == 1 )
        {
          if( pcPPS->getTileRowHeight(tileIdx / pcPPS->getNumTileColumns()) > 1 )
          {
            READ_UVLC(uiCode, "num_exp_slices_in_tile[i]");
            if (uiCode == 0)
            {
              pcPPS->setNumSlicesInTile(i, 1);
              pcPPS->setSliceHeightInCtu(i, pcPPS->getTileRowHeight(tileIdx / pcPPS->getNumTileColumns()));
            }
            else
            {
              uint32_t numExpSliceInTile = uiCode;
              uint32_t remTileRowHeight  = pcPPS->getTileRowHeight(tileIdx / pcPPS->getNumTileColumns());
              int j = 0;
              for( ; j < numExpSliceInTile - 1; j++ )
              {
                READ_UVLC(uiCode, "exp_slice_height_in_ctus_minus1[i]");
                pcPPS->setSliceHeightInCtu(i + j, uiCode + 1);
                remTileRowHeight -= (uiCode + 1);
              }
              READ_UVLC(uiCode, "exp_slice_height_in_ctus_minus1[i]");
              uint32_t uniformSliceHeight = uiCode + 1;

              while( remTileRowHeight >= uniformSliceHeight )
              {
                pcPPS->setSliceHeightInCtu(i + j, uniformSliceHeight);
                remTileRowHeight -= uniformSliceHeight;
                j++;
              }
              if( remTileRowHeight > 0 )
              {
                pcPPS->setSliceHeightInCtu(i + j, remTileRowHeight);
                j++;
              }
              for( int k = 0; k < j; k++ )
              {
                pcPPS->setNumSlicesInTile(i + k, j);
                pcPPS->setSliceWidthInTiles(i + k, 1);
                pcPPS->setSliceHeightInTiles(i + k, 1);
                pcPPS->setSliceTileIdx(i + k, tileIdx);
              }
              i += (j - 1);
            }
          }
          else
          {
            pcPPS->setNumSlicesInTile(i, 1);
            pcPPS->setSliceHeightInCtu(i, pcPPS->getTileRowHeight(tileIdx / pcPPS->getNumTileColumns()));
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
    }

#if !JVET_R0113_AND_JVET_R0106_PPS_CLEANUP
    // loop filtering across slice/tile controls
    READ_CODE(1, uiCode, "loop_filter_across_tiles_enabled_flag");    pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode == 1 );
#endif
#if JVET_R0247_PPS_LP_FTR_ACROSS_SLICES_FLAG_CLEANUP
    if (pcPPS->getRectSliceFlag() == 0 || pcPPS->getSingleSlicePerSubPicFlag() || pcPPS->getNumSlicesInPic() > 1)
    {
#endif
      READ_CODE(1, uiCode, "loop_filter_across_slices_enabled_flag");   pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode == 1 );
#if JVET_R0247_PPS_LP_FTR_ACROSS_SLICES_FLAG_CLEANUP
    }
    else
    {
      pcPPS->setLoopFilterAcrossSlicesEnabledFlag( false );
    }
#endif
  }
#if JVET_R0071_SPS_PPS_CELANUP
  else
  {
    pcPPS->setSingleSlicePerSubPicFlag(1);
  }
#endif

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
  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );            pcPPS->setUseDQP( uiCode ? true : false );
  READ_FLAG(uiCode, "pps_chroma_tool_offsets_present_flag");
  pcPPS->setPPSChromaToolFlag(uiCode ? true : false);
  if (pcPPS->getPPSChromaToolFlag())
  {
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

  READ_FLAG( uiCode, "pps_cu_chroma_qp_offset_list_enabled_flag");
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
  }
  else
  {
    pcPPS->setQpOffset(COMPONENT_Cb, 0);
    pcPPS->setQpOffset(COMPONENT_Cr, 0);
    pcPPS->setJointCbCrQpOffsetPresentFlag(0);
    pcPPS->setSliceChromaQpFlag(0);
    pcPPS->clearChromaQpOffsetList();
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
#if JVET_R0113_AND_JVET_R0106_PPS_CLEANUP
    if (!pcPPS->getNoPicPartitionFlag() && pcPPS->getDeblockingFilterOverrideEnabledFlag())
    {
      READ_FLAG(uiCode, "pps_dbf_info_in_ph_flag");
      pcPPS->setDbfInfoInPhFlag(uiCode ? true : false);
    }
    else
    {
      pcPPS->setDbfInfoInPhFlag(false);
    }
#endif
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      READ_SVLC( iCode, "pps_beta_offset_div2" );                    pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      CHECK(  pcPPS->getDeblockingFilterBetaOffsetDiv2() < -12 ||
              pcPPS->getDeblockingFilterBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration" );

      READ_SVLC( iCode, "pps_tc_offset_div2");                       pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
      CHECK(  pcPPS->getDeblockingFilterTcOffsetDiv2() < -12 ||
              pcPPS->getDeblockingFilterTcOffsetDiv2() > 12, "Invalid deblocking filter configuration" );

#if JVET_R0078_DISABLE_CHROMA_DBF_OFFSET_SINGALLING
      if( pcPPS->getPPSChromaToolFlag() )
      {
#endif
        READ_SVLC( iCode, "pps_cb_beta_offset_div2" );                   pcPPS->setDeblockingFilterCbBetaOffsetDiv2( iCode );
        CHECK( pcPPS->getDeblockingFilterCbBetaOffsetDiv2() < -12 ||
          pcPPS->getDeblockingFilterCbBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration" );

        READ_SVLC( iCode, "pps_cb_tc_offset_div2" );                     pcPPS->setDeblockingFilterCbTcOffsetDiv2( iCode );
        CHECK( pcPPS->getDeblockingFilterCbTcOffsetDiv2() < -12 ||
          pcPPS->getDeblockingFilterCbTcOffsetDiv2() > 12, "Invalid deblocking filter configuration" );

        READ_SVLC( iCode, "pps_cr_beta_offset_div2") ;                   pcPPS->setDeblockingFilterCrBetaOffsetDiv2( iCode );
        CHECK( pcPPS->getDeblockingFilterCrBetaOffsetDiv2() < -12 ||
          pcPPS->getDeblockingFilterCrBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration" );

        READ_SVLC( iCode, "pps_cr_tc_offset_div2" );                     pcPPS->setDeblockingFilterCrTcOffsetDiv2( iCode );
        CHECK(pcPPS->getDeblockingFilterCrTcOffsetDiv2() < -12 ||
          pcPPS->getDeblockingFilterCrTcOffsetDiv2() > 12, "Invalid deblocking filter configuration");
#if JVET_R0078_DISABLE_CHROMA_DBF_OFFSET_SINGALLING
      }
      else
      {
        pcPPS->setDeblockingFilterCbBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
        pcPPS->setDeblockingFilterCbTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
        pcPPS->setDeblockingFilterCrBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
        pcPPS->setDeblockingFilterCrTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
      }
#endif
    }
  }
  else
  {
    pcPPS->setDeblockingFilterOverrideEnabledFlag(false);
#if JVET_R0113_AND_JVET_R0106_PPS_CLEANUP
    pcPPS->setDbfInfoInPhFlag(false);
#endif
  }

#if JVET_R0113_AND_JVET_R0106_PPS_CLEANUP
  if (!pcPPS->getNoPicPartitionFlag())
  {
    READ_FLAG(uiCode, "pps_rpl_info_in_ph_flag");                    pcPPS->setRplInfoInPhFlag(uiCode ? true : false);
    READ_FLAG(uiCode, "pps_sao_info_in_ph_flag");                    pcPPS->setSaoInfoInPhFlag(uiCode ? true : false);
    READ_FLAG(uiCode, "pps_alf_info_in_ph_flag");                    pcPPS->setAlfInfoInPhFlag(uiCode ? true : false);
    if ((pcPPS->getUseWP() || pcPPS->getWPBiPred()) && pcPPS->getRplInfoInPhFlag())
    {
      READ_FLAG(uiCode, "pps_wp_info_in_ph_flag");                   pcPPS->setWpInfoInPhFlag(uiCode ? true : false);
    }
    else
    {
      pcPPS->setWpInfoInPhFlag(false);
    }
    READ_FLAG(uiCode, "pps_qp_delta_info_in_ph_flag");               pcPPS->setQpDeltaInfoInPhFlag(uiCode ? true : false);
  }
  else
  {
    pcPPS->setRplInfoInPhFlag(false);
    pcPPS->setSaoInfoInPhFlag(false);
    pcPPS->setAlfInfoInPhFlag(false);
    pcPPS->setWpInfoInPhFlag(false);
    pcPPS->setQpDeltaInfoInPhFlag(false);
  }
#else
  READ_FLAG(uiCode, "rpl_info_in_ph_flag");                    pcPPS->setRplInfoInPhFlag(uiCode ? true : false);
  if( pcPPS->getDeblockingFilterOverrideEnabledFlag() )
  {
    READ_FLAG(uiCode, "dbf_info_in_ph_flag");
    pcPPS->setDbfInfoInPhFlag(uiCode ? true : false);
  }
  READ_FLAG(uiCode, "sao_info_in_ph_flag");                    pcPPS->setSaoInfoInPhFlag(uiCode ? true : false);
  READ_FLAG(uiCode, "alf_info_in_ph_flag");                    pcPPS->setAlfInfoInPhFlag(uiCode ? true : false);
  if ((pcPPS->getUseWP() || pcPPS->getWPBiPred()) && pcPPS->getRplInfoInPhFlag())
  {
    READ_FLAG(uiCode, "wp_info_in_ph_flag");                   pcPPS->setWpInfoInPhFlag(uiCode ? true : false);
  }
  else
  {
    pcPPS->setWpInfoInPhFlag(false);
  }
  READ_FLAG(uiCode, "qp_delta_info_in_ph_flag");               pcPPS->setQpDeltaInfoInPhFlag(uiCode ? true : false);
#endif

#if JVET_Q0764_WRAP_AROUND_WITH_RPR
  READ_FLAG(uiCode, "pps_ref_wraparound_enabled_flag");           pcPPS->setWrapAroundEnabledFlag( uiCode ? true : false );
  if (pcPPS->getWrapAroundEnabledFlag())
  {
    READ_UVLC(uiCode, "pps_ref_wraparound_offset");               
#if JVET_R0162_WRAPAROUND_OFFSET_SIGNALING
    pcPPS->setPicWidthMinusWrapAroundOffset(uiCode);
#else
    pcPPS->setWrapAroundOffsetMinusCtbSize( uiCode );
#endif
  }
  else
  {
#if JVET_R0162_WRAPAROUND_OFFSET_SIGNALING
    pcPPS->setPicWidthMinusWrapAroundOffset(0);
#else
    pcPPS->setWrapAroundOffsetMinusCtbSize( 0 );
#endif
  }
#endif

  READ_FLAG( uiCode, "picture_header_extension_present_flag");
  pcPPS->setPictureHeaderExtensionPresentFlag(uiCode);
  READ_FLAG( uiCode, "slice_header_extension_present_flag");
  pcPPS->setSliceHeaderExtensionPresentFlag(uiCode);


  READ_FLAG( uiCode, "pps_extension_present_flag");
  if (uiCode)
  {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "pps_extension_data_flag");
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

  CcAlfFilterParam ccAlfParam = aps->getCcAlfAPSParam();
  READ_FLAG(code, "alf_cc_cb_filter_signal_flag");
  ccAlfParam.newCcAlfFilter[COMPONENT_Cb - 1] = code;
  READ_FLAG(code, "alf_cc_cr_filter_signal_flag");
  ccAlfParam.newCcAlfFilter[COMPONENT_Cr - 1] = code;
  CHECK(param.newFilterFlag[CHANNEL_TYPE_LUMA] == 0 && param.newFilterFlag[CHANNEL_TYPE_CHROMA] == 0
          && ccAlfParam.newCcAlfFilter[COMPONENT_Cb - 1] == 0 && ccAlfParam.newCcAlfFilter[COMPONENT_Cr - 1] == 0,
        "bitstream conformance error: one of alf_luma_filter_signal_flag, alf_chroma_filter_signal_flag, "
        "alf_cross_component_cb_filter_signal_flag, and alf_cross_component_cr_filter_signal_flag shall be nonzero");

  if (param.newFilterFlag[CHANNEL_TYPE_LUMA])
  {
    READ_FLAG(code, "alf_luma_clip");
    param.nonLinearFlag[CHANNEL_TYPE_LUMA] = code ? true : false;
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
    READ_FLAG(code, "alf_nonlinear_enable_flag_chroma");
    param.nonLinearFlag[CHANNEL_TYPE_CHROMA] = code ? true : false;

    if( MAX_NUM_ALF_ALTERNATIVES_CHROMA > 1 )
      READ_UVLC( code, "alf_chroma_num_alts_minus1" );
    else
      code = 0;

    param.numAlternativesChroma = code + 1;

    for( int altIdx=0; altIdx < param.numAlternativesChroma; ++altIdx )
    {
      alfFilter( param, true, altIdx );
    }
  }

  for (int ccIdx = 0; ccIdx < 2; ccIdx++)
  {
    if (ccAlfParam.newCcAlfFilter[ccIdx])
    {
      if (MAX_NUM_CC_ALF_FILTERS > 1)
      {
        READ_UVLC(code, ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1");
      }
      else
      {
        code = 0;
      }
      ccAlfParam.ccAlfFilterCount[ccIdx] = code + 1;

      for (int filterIdx = 0; filterIdx < ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx++)
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = true;
        AlfFilterShape alfShape(size_CC_ALF);

        short *coeff = ccAlfParam.ccAlfCoeff[ccIdx][filterIdx];
        // Filter coefficients
        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
          READ_CODE(CCALF_BITS_PER_COEFF_LEVEL, code,
                    ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
          if (code == 0)
          {
            coeff[i] = 0;
          }
          else
          {
            coeff[i] = 1 << (code - 1);
            READ_FLAG(code, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign");
            coeff[i] *= 1 - 2 * code;
          }
        }

        DTRACE(g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx);
        for (int i = 0; i < alfShape.numCoeff; i++)
        {
          DTRACE(g_trace_ctx, D_SYNTAX, "%d ", coeff[i]);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "\n");
      }

      for (int filterIdx = ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = false;
      }
    }
  }
  aps->setCcAlfAPSParam(ccAlfParam);

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

#if JVET_R0090_VUI
  READ_FLAG(symbol,  "vui_general_progressive_source_flag"          ); pcVUI->setProgressiveSourceFlag(symbol ? true : false);
  READ_FLAG(symbol,  "vui_general_interlaced_source_flag"           ); pcVUI->setInterlacedSourceFlag(symbol ? true : false);
#endif
  READ_FLAG( symbol, "vui_aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(symbol);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    READ_FLAG( symbol, "vui_aspect_ratio_constant_flag");           pcVUI->setAspectRatioConstantFlag(symbol);
    READ_CODE(8, symbol, "vui_aspect_ratio_idc");                         pcVUI->setAspectRatioIdc(symbol);
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      READ_CODE(16, symbol, "vui_sar_width");                             pcVUI->setSarWidth(symbol);
      READ_CODE(16, symbol, "vui_sar_height");                            pcVUI->setSarHeight(symbol);
    }
  }

  READ_FLAG(     symbol, "vui_overscan_info_present_flag");               pcVUI->setOverscanInfoPresentFlag(symbol);
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    READ_FLAG(   symbol, "vui_overscan_appropriate_flag");                pcVUI->setOverscanAppropriateFlag(symbol);
  }

  READ_FLAG(   symbol, "vui_colour_description_present_flag");          pcVUI->setColourDescriptionPresentFlag(symbol);
  if (pcVUI->getColourDescriptionPresentFlag())
  {
    READ_CODE(8, symbol, "vui_colour_primaries");                       pcVUI->setColourPrimaries(symbol);
    READ_CODE(8, symbol, "vui_transfer_characteristics");               pcVUI->setTransferCharacteristics(symbol);
    READ_CODE(8, symbol, "vui_matrix_coeffs");                          pcVUI->setMatrixCoefficients(symbol);
    READ_FLAG(   symbol, "vui_video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(symbol);
  }

  READ_FLAG(     symbol, "vui_chroma_loc_info_present_flag");             pcVUI->setChromaLocInfoPresentFlag(symbol);
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
#if JVET_R0090_VUI
    if(pcVUI->getProgressiveSourceFlag() && !pcVUI->getInterlacedSourceFlag())
#else
    if(pcSPS->getProfileTierLevel()->getConstraintInfo()->getProgressiveSourceFlag() &&
       !pcSPS->getProfileTierLevel()->getConstraintInfo()->getInterlacedSourceFlag())
#endif
    {
      READ_UVLC(   symbol, "vui_chroma_sample_loc_type" );        pcVUI->setChromaSampleLocType(symbol);
    }
    else
    {
      READ_UVLC(   symbol, "vui_chroma_sample_loc_type_top_field" );        pcVUI->setChromaSampleLocTypeTopField(symbol);
      READ_UVLC(   symbol, "vui_chroma_sample_loc_type_bottom_field" );     pcVUI->setChromaSampleLocTypeBottomField(symbol);
    }
  }

}

void HLSyntaxReader::parseGeneralHrdParameters(GeneralHrdParams *hrd)
{
  uint32_t  symbol;
  READ_CODE(32, symbol, "num_units_in_tick");                hrd->setNumUnitsInTick(symbol);
  READ_CODE(32, symbol, "time_scale");                       hrd->setTimeScale(symbol);
  READ_FLAG(symbol, "general_nal_hrd_parameters_present_flag");           hrd->setGeneralNalHrdParametersPresentFlag(symbol == 1 ? true : false);
  READ_FLAG(symbol, "general_vcl_hrd_parameters_present_flag");           hrd->setGeneralVclHrdParametersPresentFlag(symbol == 1 ? true : false);
  CHECK((hrd->getGeneralNalHrdParametersPresentFlag() == 0) && (hrd->getGeneralVclHrdParametersPresentFlag() == 0), "general_nal_hrd_params_present_flag and general_vcl_hrd_params_present_flag in each general_hrd_parameters( ) syntax structure shall not be both equal to 0.");
  READ_FLAG(symbol, "general_same_pic_timing_in_all_ols_flag");           hrd->setGeneralSamePicTimingInAllOlsFlag(symbol == 1 ? true : false);
  READ_FLAG(symbol, "general_decoding_unit_hrd_params_present_flag");     hrd->setGeneralDecodingUnitHrdParamsPresentFlag(symbol == 1 ? true : false);
  if (hrd->getGeneralDecodingUnitHrdParamsPresentFlag())
  {
    READ_CODE(8, symbol, "tick_divisor_minus2");                        hrd->setTickDivisorMinus2(symbol);
  }
  READ_CODE(4, symbol, "bit_rate_scale");                       hrd->setBitRateScale(symbol);
  READ_CODE(4, symbol, "cpb_size_scale");                       hrd->setCpbSizeScale(symbol);
  if (hrd->getGeneralDecodingUnitHrdParamsPresentFlag())
  {
    READ_CODE(4, symbol, "cpb_size_du_scale");                  hrd->setCpbSizeDuScale(symbol);
  }
  READ_UVLC(symbol, "hrd_cpb_cnt_minus1");                      hrd->setHrdCpbCntMinus1(symbol);
  CHECK(symbol > 31,"The value of hrd_cpb_cnt_minus1 shall be in the range of 0 to 31, inclusive");
}
void HLSyntaxReader::parseOlsHrdParameters(GeneralHrdParams * generalHrd, OlsHrdParams *olsHrd, uint32_t firstSubLayer, uint32_t maxNumSubLayersMinus1)
{
  uint32_t  symbol;

  for( int i = firstSubLayer; i <= maxNumSubLayersMinus1; i ++ )
  {
    OlsHrdParams *hrd = &(olsHrd[i]);
    READ_FLAG(symbol, "fixed_pic_rate_general_flag");                     hrd->setFixedPicRateGeneralFlag(symbol == 1 ? true : false);
    if (!hrd->getFixedPicRateGeneralFlag())
    {
      READ_FLAG(symbol, "fixed_pic_rate_within_cvs_flag");                hrd->setFixedPicRateWithinCvsFlag(symbol == 1 ? true : false);
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag(true);
    }

    hrd->setLowDelayHrdFlag(false); // Inferred to be 0 when not present

    if (hrd->getFixedPicRateWithinCvsFlag())
    {
      READ_UVLC(symbol, "elemental_duration_in_tc_minus1");             hrd->setElementDurationInTcMinus1(symbol);
    }
    else if(generalHrd->getHrdCpbCntMinus1() == 0)
    {
      READ_FLAG(symbol, "low_delay_hrd_flag");                      hrd->setLowDelayHrdFlag(symbol == 1 ? true : false);
    }

    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if (((nalOrVcl == 0) && (generalHrd->getGeneralNalHrdParametersPresentFlag())) || ((nalOrVcl == 1) && (generalHrd->getGeneralVclHrdParametersPresentFlag())))
      {
        for (int j = 0; j <= (generalHrd->getHrdCpbCntMinus1()); j++)
        {
          READ_UVLC(symbol, "bit_rate_value_minus1");             hrd->setBitRateValueMinus1(j, nalOrVcl, symbol);
          READ_UVLC(symbol, "cpb_size_value_minus1");             hrd->setCpbSizeValueMinus1(j, nalOrVcl, symbol);
          if (generalHrd->getGeneralDecodingUnitHrdParamsPresentFlag())
          {
            READ_UVLC(symbol, "bit_rate_du_value_minus1");             hrd->setDuBitRateValueMinus1(j, nalOrVcl, symbol);
            READ_UVLC(symbol, "cpb_size_du_value_minus1");             hrd->setDuCpbSizeValueMinus1(j, nalOrVcl, symbol);
          }
          READ_FLAG(symbol, "cbr_flag");                          hrd->setCbrFlag(j, nalOrVcl, symbol == 1 ? true : false);
        }
      }
    }
  }
  for (int i = 0; i < firstSubLayer; i++)
  {
    OlsHrdParams* hrdHighestTLayer = &(olsHrd[maxNumSubLayersMinus1]);
    OlsHrdParams* hrdTemp = &(olsHrd[i]);
    bool tempFlag = hrdHighestTLayer->getFixedPicRateGeneralFlag();
    hrdTemp->setFixedPicRateGeneralFlag(tempFlag);
    tempFlag = hrdHighestTLayer->getFixedPicRateWithinCvsFlag();
    hrdTemp->setFixedPicRateWithinCvsFlag(tempFlag);
    uint32_t tempElementDurationInTcMinus1 = hrdHighestTLayer->getElementDurationInTcMinus1();
    hrdTemp->setElementDurationInTcMinus1(tempElementDurationInTcMinus1);
    for (int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++)
    {
      if (((nalOrVcl == 0) && (generalHrd->getGeneralNalHrdParametersPresentFlag())) || ((nalOrVcl == 1) && (generalHrd->getGeneralVclHrdParametersPresentFlag())))
      {
        for (int j = 0; j <= (generalHrd->getHrdCpbCntMinus1()); j++)
        {
          uint32_t bitRate = hrdHighestTLayer->getBitRateValueMinus1(j, nalOrVcl);
          hrdTemp->setBitRateValueMinus1(j, nalOrVcl, bitRate);
          uint32_t cpbSize = hrdHighestTLayer->getCpbSizeValueMinus1(j, nalOrVcl);
          hrdTemp->setCpbSizeValueMinus1(j, nalOrVcl, cpbSize);
          if (generalHrd->getGeneralDecodingUnitHrdParamsPresentFlag())
          {
            uint32_t bitRateDu = hrdHighestTLayer->getDuBitRateValueMinus1(j, nalOrVcl);
            hrdTemp->setDuBitRateValueMinus1(j, nalOrVcl, bitRateDu);
            uint32_t cpbSizeDu = hrdHighestTLayer->getDuCpbSizeValueMinus1(j, nalOrVcl);
            hrdTemp->setDuCpbSizeValueMinus1(j, nalOrVcl, cpbSizeDu);
          }
          bool flag = hrdHighestTLayer->getCbrFlag(j, nalOrVcl);
          hrdTemp->setCbrFlag(j, nalOrVcl, flag);
        }
      }
    }
  }
}

void HLSyntaxReader::dpb_parameters(int maxSubLayersMinus1, bool subLayerInfoFlag, SPS *pcSPS)
{
  uint32_t code;
  for (int i = (subLayerInfoFlag ? 0 : maxSubLayersMinus1); i <= maxSubLayersMinus1; i++)
  {
    READ_UVLC(code, "max_dec_pic_buffering_minus1[i]");
    pcSPS->setMaxDecPicBuffering(code + 1, i);
    READ_UVLC(code, "max_num_reorder_pics[i]");
    pcSPS->setNumReorderPics(code, i);
    READ_UVLC(code, "max_latency_increase_plus1[i]");
    pcSPS->setMaxLatencyIncreasePlus1(code, i);
  }
}

void HLSyntaxReader::parseExtraPHBitsStruct( SPS *sps, int numBytes )
{
  uint32_t symbol;
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );

  for (int i=0; i < 8*numBytes; i++)
  {
    READ_FLAG(symbol, "extra_ph_bit_present_flag[ i ]");
    presentFlags[i] = symbol;
  }

  sps->setExtraPHBitPresentFlags(presentFlags);
}

void HLSyntaxReader::parseExtraSHBitsStruct( SPS *sps, int numBytes )
{
  uint32_t symbol;
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );

  for (int i=0; i < 8*numBytes; i++)
  {
    READ_FLAG(symbol, "extra_sh_bit_present_flag[ i ]");
    presentFlags[i] = symbol;
  }

  sps->setExtraSHBitPresentFlags(presentFlags);
}

void HLSyntaxReader::parseSPS(SPS* pcSPS)
{
  uint32_t  uiCode;

#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif

  READ_CODE(4, uiCode, "sps_seq_parameter_set_id");              pcSPS->setSPSId(uiCode);
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id" );      pcSPS->setVPSId( uiCode );
  READ_CODE(3, uiCode, "sps_max_sub_layers_minus1");          pcSPS->setMaxTLayers   (uiCode + 1);
  CHECK(uiCode > 6, "Invalid maximum number of T-layer signalled");
  READ_CODE(4, uiCode, "sps_reserved_zero_4bits");
  CHECK(uiCode != 0, "sps_reserved_zero_4bits not equal to zero");
  READ_FLAG(uiCode, "sps_ptl_dpb_hrd_params_present_flag"); pcSPS->setPtlDpbHrdParamsPresentFlag(uiCode);

#if JVET_R0275_SPS_PTL_DBP_HRD
  if( !pcSPS->getVPSId() )
  {
    CHECK( !pcSPS->getPtlDpbHrdParamsPresentFlag(), "When sps_video_parameter_set_id is equal to 0, the value of sps_ptl_dpb_hrd_params_present_flag shall be equal to 1" );
  }
#endif

  if (pcSPS->getPtlDpbHrdParamsPresentFlag())
  {

    parseProfileTierLevel(pcSPS->getProfileTierLevel(), true, pcSPS->getMaxTLayers() - 1);

  }

  READ_FLAG(uiCode, "gdr_enabled_flag");
  pcSPS->setGDREnabledFlag(uiCode);
  READ_CODE(2, uiCode, "chroma_format_idc");                     pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG(     uiCode, "separate_colour_plane_flag");        CHECK(uiCode != 0, "separate_colour_plane_flag shall be equal to 0");
    pcSPS->setSeparateColourPlaneFlag( uiCode != 0 );
  }

#if JVET_R0058
  READ_FLAG(uiCode, "ref_pic_resampling_enabled_flag");          pcSPS->setRprEnabledFlag(uiCode);
  if (uiCode)
  {
    READ_FLAG(uiCode, "res_change_in_clvs_allowed_flag");        pcSPS->setResChangeInClvsEnabledFlag(uiCode);
  }
  else
  {
    pcSPS->setResChangeInClvsEnabledFlag(0);
  }
#else
  READ_FLAG( uiCode, "res_change_in_clvs_allowed_flag" );        pcSPS->setRprEnabledFlag( uiCode );
#endif

  if (pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoResChangeInClvsConstraintFlag())
  {
    CHECK(uiCode != 0, "When no_res_change_in_clvs_constraint_flag is equal to 1, res_change_in_clvs_allowed_flag shall be equal to 0");
  }

  READ_UVLC( uiCode, "pic_width_max_in_luma_samples" );          pcSPS->setMaxPicWidthInLumaSamples( uiCode );
  READ_UVLC( uiCode, "pic_height_max_in_luma_samples" );         pcSPS->setMaxPicHeightInLumaSamples( uiCode );
  READ_FLAG(uiCode, "sps_conformance_window_flag");
  if (uiCode != 0)
  {
    Window& conf = pcSPS->getConformanceWindow();
    READ_UVLC(uiCode, "sps_conf_win_left_offset");               conf.setWindowLeftOffset(uiCode);
    READ_UVLC(uiCode, "sps_conf_win_right_offset");              conf.setWindowRightOffset(uiCode);
    READ_UVLC(uiCode, "sps_conf_win_top_offset");                conf.setWindowTopOffset(uiCode);
    READ_UVLC(uiCode, "sps_conf_win_bottom_offset");             conf.setWindowBottomOffset(uiCode);
  }
  const uint32_t chromaArrayType = (int) pcSPS->getSeparateColourPlaneFlag() ? 0 : pcSPS->getChromaFormatIdc();

  READ_CODE(2, uiCode, "sps_log2_ctu_size_minus5");              pcSPS->setCTUSize(1 << (uiCode + 5));
  CHECK(uiCode > 2, "sps_log2_ctu_size_minus5 must be less than or equal to 2");
  unsigned ctbLog2SizeY = uiCode + 5;
  pcSPS->setMaxCUWidth(pcSPS->getCTUSize());
  pcSPS->setMaxCUHeight(pcSPS->getCTUSize());

  READ_FLAG( uiCode, "subpic_info_present_flag" );               pcSPS->setSubPicInfoPresentFlag(uiCode);

  if (pcSPS->getSubPicInfoPresentFlag())
  {
    READ_UVLC(uiCode, "sps_num_subpics_minus1"); pcSPS->setNumSubPics(uiCode + 1);
    CHECK(uiCode > (pcSPS->getMaxPicWidthInLumaSamples() / (1 << pcSPS->getCTUSize())) * (pcSPS->getMaxPicHeightInLumaSamples() / (1 << pcSPS->getCTUSize())) - 1, "Invalid sps_num_subpics_minus1 value");
    if (pcSPS->getProfileTierLevel()->getConstraintInfo()->getOneSubpicPerPicConstraintFlag())
    {
      CHECK(uiCode != 0, "When one_subpic_per_pic_constraint_flag is equal to 1, each picture shall contain only one subpicture");
    }

    if( pcSPS->getNumSubPics() == 1 )
    {
      pcSPS->setSubPicCtuTopLeftX( 0, 0 );
      pcSPS->setSubPicCtuTopLeftY( 0, 0 );
      pcSPS->setSubPicWidth( 0, ( pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1 ) >> floorLog2( pcSPS->getCTUSize() ) );
      pcSPS->setSubPicHeight( 0, ( pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1 ) >> floorLog2( pcSPS->getCTUSize() ) );

#if JVET_R0156_ASPECT4_SPS_CLEANUP
      pcSPS->setIndependentSubPicsFlag(1);
#endif

#if JVET_R0071_SPS_PPS_CELANUP
      pcSPS->setSubPicTreatedAsPicFlag(0, 1);
      pcSPS->setLoopFilterAcrossSubpicEnabledFlag(0, 0);
#else
      pcSPS->setSubPicTreatedAsPicFlag( 0, 0 );
      pcSPS->setLoopFilterAcrossSubpicEnabledFlag( 0, 1 );
#endif
    }
    else
    {
#if JVET_R0156_ASPECT4_SPS_CLEANUP
    READ_FLAG(uiCode, "sps_independent_subpics_flag"); pcSPS->setIndependentSubPicsFlag(uiCode != 0);
#endif
    for (int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++)
    {
      if ((picIdx > 0) && (pcSPS->getMaxPicWidthInLumaSamples() > pcSPS->getCTUSize()))
      {
        READ_CODE(ceilLog2((pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1) / pcSPS->getCTUSize()), uiCode, "subpic_ctu_top_left_x[ i ]");
        pcSPS->setSubPicCtuTopLeftX(picIdx, uiCode);
      }
      else
      {
        pcSPS->setSubPicCtuTopLeftX(picIdx, 0);
      }
      if ((picIdx > 0) && (pcSPS->getMaxPicHeightInLumaSamples() > pcSPS->getCTUSize()))
      {
        READ_CODE(ceilLog2((pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1) / pcSPS->getCTUSize()), uiCode, "subpic_ctu_top_left_y[ i ]");
        pcSPS->setSubPicCtuTopLeftY(picIdx, uiCode);
      }
      else
      {
        pcSPS->setSubPicCtuTopLeftY(picIdx, 0);
      }
      if (picIdx <pcSPS->getNumSubPics()-1 && pcSPS->getMaxPicWidthInLumaSamples() > pcSPS->getCTUSize())
      {
        READ_CODE(ceilLog2((pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1) / pcSPS->getCTUSize()), uiCode, "subpic_width_minus1[ i ]");
        pcSPS->setSubPicWidth(picIdx, uiCode + 1);
      }
      else
      {
        pcSPS->setSubPicWidth(picIdx, (pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1) /pcSPS->getCTUSize() - pcSPS->getSubPicCtuTopLeftX(picIdx));
      }
      if (picIdx <pcSPS->getNumSubPics() - 1 && pcSPS->getMaxPicHeightInLumaSamples() > pcSPS->getCTUSize())
      {
        READ_CODE(ceilLog2((pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1) / pcSPS->getCTUSize()), uiCode, "subpic_height_minus1[ i ]");
        pcSPS->setSubPicHeight(picIdx, uiCode + 1);
      }
      else
      {
        pcSPS->setSubPicHeight(picIdx, (pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1) /pcSPS->getCTUSize() - pcSPS->getSubPicCtuTopLeftY(picIdx));
      }
#if JVET_R0156_ASPECT4_SPS_CLEANUP
      if (!pcSPS->getIndependentSubPicsFlag())
      {
#endif
        READ_FLAG(uiCode, "subpic_treated_as_pic_flag[ i ]");
        pcSPS->setSubPicTreatedAsPicFlag(picIdx, uiCode);
        READ_FLAG(uiCode, "loop_filter_across_subpic_enabled_flag[ i ]");
        pcSPS->setLoopFilterAcrossSubpicEnabledFlag(picIdx, uiCode);
#if JVET_R0156_ASPECT4_SPS_CLEANUP
      }
#endif
    }
    }

    READ_UVLC( uiCode, "sps_subpic_id_len_minus1" );                       pcSPS->setSubPicIdLen( uiCode + 1 );
    CHECK( uiCode > 15, "Invalid sps_subpic_id_len_minus1 value" );
    CHECK( (1 << (uiCode + 1)) < pcSPS->getNumSubPics(), "Invalid sps_subpic_id_len_minus1 value" );
    READ_FLAG( uiCode, "subpic_id_mapping_explicitly_signalled_flag" );    pcSPS->setSubPicIdMappingExplicitlySignalledFlag( uiCode != 0 );
    if (pcSPS->getSubPicIdMappingExplicitlySignalledFlag())
    {
      READ_FLAG( uiCode, "subpic_id_mapping_in_sps_flag" );                pcSPS->setSubPicIdMappingInSpsFlag( uiCode != 0 );
      if (pcSPS->getSubPicIdMappingInSpsFlag())
      {
        for (int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++)
        {
          READ_CODE(pcSPS->getSubPicIdLen(), uiCode, "sps_subpic_id[i]");   pcSPS->setSubPicId(picIdx, uiCode);
        }
      }
    }
  }
  else
  {
    pcSPS->setSubPicIdMappingExplicitlySignalledFlag(0);
    pcSPS->setNumSubPics(1);
    pcSPS->setSubPicCtuTopLeftX(0, 0);
    pcSPS->setSubPicCtuTopLeftY(0, 0);
    pcSPS->setSubPicWidth(0, (pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1) >> floorLog2(pcSPS->getCTUSize()));
    pcSPS->setSubPicHeight(0, (pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1) >> floorLog2(pcSPS->getCTUSize()));
  }

  if( !pcSPS->getSubPicIdMappingExplicitlySignalledFlag() || !pcSPS->getSubPicIdMappingInSpsFlag() )
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

  READ_FLAG( uiCode, "sps_entropy_coding_sync_enabled_flag" );       pcSPS->setEntropyCodingSyncEnabledFlag(uiCode == 1);
#if JVET_R0165_OPTIONAL_ENTRY_POINT
  READ_FLAG(uiCode, "sps_entry_point_offsets_present_flag");   pcSPS->setEntryPointsPresentFlag(uiCode == 1);
#else
  if (pcSPS->getEntropyCodingSyncEnabledFlag())
  {
    READ_FLAG(uiCode, "sps_wpp_entry_point_offsets_present_flag");   pcSPS->setEntropyCodingSyncEntryPointsPresentFlag(uiCode == 1);
  }
#endif
#if !JVET_R0332_HLS_ORDER
  READ_FLAG( uiCode, "sps_weighted_pred_flag" );                    pcSPS->setUseWP( uiCode ? true : false );
  READ_FLAG( uiCode, "sps_weighted_bipred_flag" );                  pcSPS->setUseWPBiPred( uiCode ? true : false );
#endif
  READ_CODE(4, uiCode, "log2_max_pic_order_cnt_lsb_minus4");     pcSPS->setBitsForPOC( 4 + uiCode );
  CHECK(uiCode > 12, "log2_max_pic_order_cnt_lsb_minus4 shall be in the range of 0 to 12");

  READ_FLAG(uiCode, "sps_poc_msb_flag");                    pcSPS->setPocMsbFlag(uiCode ? true : false);
  if (pcSPS->getPocMsbFlag())
  {
    READ_UVLC(uiCode, "poc_msb_len_minus1");                  pcSPS->setPocMsbLen(1 + uiCode);
    CHECK(uiCode > (32 - ( pcSPS->getBitsForPOC() - 4 )- 5), "The value of poc_msb_len_minus1 shall be in the range of 0 to 32 - log2_max_pic_order_cnt_lsb_minus4 - 5, inclusive");
  }

  // extra bits are for future extensions, we will read, but ignore them,
  // unless a meaning is specified in the spec
  READ_CODE(2, uiCode, "num_extra_ph_bits_bytes");  pcSPS->setNumExtraPHBitsBytes(uiCode);
  parseExtraPHBitsStruct( pcSPS, uiCode );
  READ_CODE(2, uiCode, "num_extra_sh_bits_bytes");  pcSPS->setNumExtraSHBitsBytes(uiCode);
  parseExtraSHBitsStruct( pcSPS, uiCode );

#if !JVET_R0156_ASPECT3_SPS_CLEANUP
  if (pcSPS->getMaxTLayers() - 1 > 0)
  {
    READ_FLAG(uiCode, "sps_sublayer_dpb_params_flag");     pcSPS->setSubLayerDpbParamsFlag(uiCode ? true : false);
  }
#endif
  if (pcSPS->getPtlDpbHrdParamsPresentFlag())
  {
#if JVET_R0156_ASPECT3_SPS_CLEANUP
    if (pcSPS->getMaxTLayers() - 1 > 0)
    {
      READ_FLAG(uiCode, "sps_sublayer_dpb_params_flag");     pcSPS->setSubLayerDpbParamsFlag(uiCode ? true : false);
    }
#endif
    dpb_parameters(pcSPS->getMaxTLayers() - 1, pcSPS->getSubLayerDpbParamsFlag(), pcSPS);
  }
#if !JVET_R0332_HLS_ORDER 
  READ_FLAG(uiCode, "long_term_ref_pics_flag");          pcSPS->setLongTermRefsPresent(uiCode);
#if JVET_R0205
  if (pcSPS->getVPSId() > 0)
  {
    READ_FLAG( uiCode, "sps_inter_layer_ref_pics_present_flag" );  pcSPS->setInterLayerPresentFlag( uiCode );
  }
  else
  {
    pcSPS->setInterLayerPresentFlag(0);
  }
#else
  READ_FLAG( uiCode, "inter_layer_ref_pics_present_flag" );  pcSPS->setInterLayerPresentFlag( uiCode );
#endif
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
#if JVET_R0059_RPL_CLEANUP
    parseRefPicList(pcSPS, rpl, ii);
#else
    parseRefPicList(pcSPS, rpl);
#endif
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
#if JVET_R0059_RPL_CLEANUP
      parseRefPicList(pcSPS, rpl, ii);
#else
      parseRefPicList(pcSPS, rpl);
#endif
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
#endif
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
  const int minCuSize = 1 << pcSPS->getLog2MinCodingBlockSize();
  CHECK( ( pcSPS->getMaxPicWidthInLumaSamples() % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( pcSPS->getMaxPicHeightInLumaSamples() % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );

  READ_FLAG(uiCode, "partition_constraints_override_enabled_flag"); pcSPS->setSplitConsOverrideEnabledFlag(uiCode);
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
  unsigned minQtLog2SizeIntraY = uiCode + pcSPS->getLog2MinCodingBlockSize();
  minQT[0] = 1 << minQtLog2SizeIntraY;
#if JVET_R0347_MTT_SIZE_CONSTRAIN
  CHECK(minQT[0] > 64, "The value of sps_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size");
  CHECK(minQT[0] > (1<<ctbLog2SizeY), "The value of sps_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size");
#endif
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_luma");     maxBTD[0] = uiCode;
  CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  maxTTSize[0] = maxBTSize[0] = minQT[0];
  if (maxBTD[0] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_luma");     maxBTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "The value of sps_log2_diff_max_bt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeIntraY");
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_luma");     maxTTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "The value of sps_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeIntraY");
#if JVET_R0347_MTT_SIZE_CONSTRAIN
    CHECK(maxTTSize[0] > 64, "The value of sps_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraY");
#endif
  }
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_inter_slice");
  unsigned minQtLog2SizeInterY = uiCode + pcSPS->getLog2MinCodingBlockSize();
  minQT[1] = 1 << minQtLog2SizeInterY;
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_inter_slice");     maxBTD[1] = uiCode;
  CHECK(uiCode > 2*(ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  maxTTSize[1] = maxBTSize[1] = minQT[1];
  if (maxBTD[1] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_inter_slice");     maxBTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "The value of sps_log2_diff_max_bt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeInterY");
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_inter_slice");     maxTTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "The value of sps_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeInterY");
#if JVET_R0347_MTT_SIZE_CONSTRAIN
    CHECK(maxTTSize[1] > 64, "The value of sps_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeInterY");
#endif
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
#if JVET_R0347_MTT_SIZE_CONSTRAIN
      CHECK(maxTTSize[2] > 64, "The value of sps_log2_diff_max_tt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
      CHECK(maxBTSize[2] > 64, "The value of sps_log2_diff_max_bt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
#endif
    }
}

  pcSPS->setMinQTSizes(minQT);
  pcSPS->setMaxMTTHierarchyDepth(maxBTD[1], maxBTD[0], maxBTD[2]);
  pcSPS->setMaxBTSize(maxBTSize[1], maxBTSize[0], maxBTSize[2]);
  pcSPS->setMaxTTSize(maxTTSize[1], maxTTSize[0], maxTTSize[2]);

#if JVET_R0097_MAX_TRSIZE_CONDITIONALY_SIGNALING
  if (pcSPS->getCTUSize() > 32) {
#endif
    READ_FLAG(uiCode, "sps_max_luma_transform_size_64_flag");        pcSPS->setLog2MaxTbSize((uiCode ? 1 : 0) + 5);
#if JVET_R0097_MAX_TRSIZE_CONDITIONALY_SIGNALING
  }
  else
  {
    pcSPS->setLog2MaxTbSize(5);
  }
#endif

  if (chromaArrayType != CHROMA_400)
  {
    READ_FLAG(uiCode, "sps_joint_cbcr_enabled_flag");                pcSPS->setJointCbCrEnabledFlag(uiCode ? true : false);
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
  if (pcSPS->getALFEnabledFlag() && pcSPS->getChromaFormatIdc() != CHROMA_400)
  {
    READ_FLAG( uiCode, "sps_ccalf_enabled_flag" );                      pcSPS->setCCALFEnabledFlag ( uiCode ? true : false );
  }
  else
  {
    pcSPS->setCCALFEnabledFlag(false);
  }

  READ_FLAG(uiCode, "sps_transform_skip_enabled_flag"); pcSPS->setTransformSkipEnabledFlag(uiCode ? true : false);
  if (pcSPS->getTransformSkipEnabledFlag())
  {
    READ_UVLC(uiCode, "log2_transform_skip_max_size_minus2");
    pcSPS->setLog2MaxTransformSkipBlockSize(uiCode + 2);
    READ_FLAG(uiCode, "sps_bdpcm_enabled_flag"); pcSPS->setBDPCMEnabledFlag(uiCode ? true : false);
  }

#if JVET_R0332_HLS_ORDER
  READ_FLAG( uiCode, "sps_weighted_pred_flag" );                    pcSPS->setUseWP( uiCode ? true : false );
  READ_FLAG( uiCode, "sps_weighted_bipred_flag" );                  pcSPS->setUseWPBiPred( uiCode ? true : false );

  READ_FLAG(uiCode, "long_term_ref_pics_flag");          pcSPS->setLongTermRefsPresent(uiCode);
#if JVET_R0205
  if (pcSPS->getVPSId() > 0)
  {
    READ_FLAG( uiCode, "sps_inter_layer_ref_pics_present_flag" );  pcSPS->setInterLayerPresentFlag( uiCode );
  }
  else
  {
    pcSPS->setInterLayerPresentFlag(0);
  }
#else
  READ_FLAG( uiCode, "inter_layer_ref_pics_present_flag" );  pcSPS->setInterLayerPresentFlag( uiCode );
#endif
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
#if JVET_R0059_RPL_CLEANUP
    parseRefPicList(pcSPS, rpl, ii);
#else
    parseRefPicList(pcSPS, rpl);
#endif
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
#if JVET_R0059_RPL_CLEANUP
      parseRefPicList(pcSPS, rpl, ii);
#else
      parseRefPicList(pcSPS, rpl);
#endif
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
  
#endif

  READ_FLAG(uiCode, "sps_ref_wraparound_enabled_flag");                  pcSPS->setWrapAroundEnabledFlag( uiCode ? true : false );
#if !JVET_Q0764_WRAP_AROUND_WITH_RPR

  if (pcSPS->getWrapAroundEnabledFlag())
  {
    READ_UVLC(uiCode, "sps_ref_wraparound_offset");               pcSPS->setWrapAroundOffset((uiCode + 2 + pcSPS->getCTUSize() / (1 << pcSPS->getLog2MinCodingBlockSize()))*(1 << pcSPS->getLog2MinCodingBlockSize()));
  }
#endif

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
#if JVET_R0214_MMVD_SYNTAX_MODIFICATION
  if (pcSPS->getUseMMVD())
  {
    READ_FLAG(uiCode, "sps_mmvd_fullpel_only_flag");                pcSPS->setFpelMmvdEnabledFlag(uiCode != 0);
  }
  else
  {
    pcSPS->setFpelMmvdEnabledFlag( false );
  }
#endif

#if JVET_R0332_HLS_ORDER
  READ_UVLC(uiCode, "six_minus_max_num_merge_cand");
  CHECK(MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of merge candidates!");
  pcSPS->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
  READ_FLAG(uiCode, "sps_sbt_enabled_flag");                        pcSPS->setUseSBT                 ( uiCode != 0 );
  READ_FLAG( uiCode,    "sps_affine_enabled_flag" );                            pcSPS->setUseAffine              ( uiCode != 0 );
  if ( pcSPS->getUseAffine() )
  {
    READ_UVLC(uiCode, "five_minus_max_num_subblock_merge_cand");
#if JVET_R0371_MAX_NUM_SUB_BLK_MRG_CAND
    CHECK(uiCode > 5 - (pcSPS->getSBTMVPEnabledFlag() ? 1 : 0), "The value of five_minus_max_num_subblock_merge_cand shall be in the range of 0 to 5 - sps_sbtmvp_enabled_flag");
    CHECK(AFFINE_MRG_MAX_NUM_CANDS < uiCode, "The value of five_minus_max_num_subblock_merge_cand shall be in the range of 0 to 5 - sps_sbtmvp_enabled_flag");
#else
    CHECK(AFFINE_MRG_MAX_NUM_CANDS < uiCode, "Incorrrect max number of affine merge candidates!");
#endif
    pcSPS->setMaxNumAffineMergeCand(AFFINE_MRG_MAX_NUM_CANDS - uiCode);
    READ_FLAG( uiCode,  "sps_affine_type_flag" );                       pcSPS->setUseAffineType          ( uiCode != 0 );
    if( pcSPS->getAMVREnabledFlag())
    {
      READ_FLAG( uiCode, "sps_affine_amvr_enabled_flag" );            pcSPS->setAffineAmvrEnabledFlag  ( uiCode != 0 );
    }
    READ_FLAG( uiCode, "sps_affine_prof_enabled_flag" );            pcSPS->setUsePROF                ( uiCode != 0 );
    if (pcSPS->getUsePROF())
    {
      READ_FLAG(uiCode, "sps_prof_pic_present_flag");               pcSPS->setProfControlPresentFlag ( uiCode != 0 );
    }
    else {
      pcSPS->setProfControlPresentFlag( false );
    }
  }

  READ_FLAG( uiCode,    "sps_bcw_enabled_flag" );                   pcSPS->setUseBcw( uiCode != 0 );

  READ_FLAG( uiCode,     "sps_ciip_enabled_flag" );                           pcSPS->setUseCiip             ( uiCode != 0 );
#if !JVET_R0214_MMVD_SYNTAX_MODIFICATION
  if ( pcSPS->getUseMMVD() )
  {
    READ_FLAG( uiCode,  "sps_fpel_mmvd_enabled_flag" );             pcSPS->setFpelMmvdEnabledFlag ( uiCode != 0 );
  }
#endif
  if (pcSPS->getMaxNumMergeCand() >= 2)
  {
    READ_FLAG(uiCode, "sps_gpm_enabled_flag");
    pcSPS->setUseGeo(uiCode != 0);
    if (pcSPS->getUseGeo() && pcSPS->getMaxNumMergeCand() >= 3)
    {
      READ_UVLC(uiCode, "max_num_merge_cand_minus_max_num_gpm_cand");
      CHECK(pcSPS->getMaxNumMergeCand() < uiCode, "Incorrrect max number of GEO candidates!");
      pcSPS->setMaxNumGeoCand((uint32_t)(pcSPS->getMaxNumMergeCand() - uiCode));
    }
    else if (pcSPS->getUseGeo())
      pcSPS->setMaxNumGeoCand(2);
  }
  else
  {
    pcSPS->setUseGeo(0);
    pcSPS->setMaxNumGeoCand(0);
  }

  READ_UVLC(uiCode, "log2_parallel_merge_level_minus2");
  CHECK(uiCode + 2 > ctbLog2SizeY, "The value of log2_parallel_merge_level_minus2 shall be in the range of 0 to ctbLog2SizeY - 2");
  pcSPS->setLog2ParallelMergeLevelMinus2(uiCode);

#endif

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
  else
  {
    pcSPS->setHorCollocatedChromaFlag(true);
    pcSPS->setVerCollocatedChromaFlag(true);
  }

  READ_FLAG( uiCode,    "sps_mts_enabled_flag" );                       pcSPS->setUseMTS                 ( uiCode != 0 );
  if ( pcSPS->getUseMTS() )
  {
    READ_FLAG( uiCode,    "sps_explicit_mts_intra_enabled_flag" );               pcSPS->setUseIntraMTS            ( uiCode != 0 );
    READ_FLAG( uiCode,    "sps_explicit_mts_inter_enabled_flag" );               pcSPS->setUseInterMTS            ( uiCode != 0 );
  }
#if !JVET_R0332_HLS_ORDER
  READ_UVLC(uiCode, "six_minus_max_num_merge_cand");
  CHECK(MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of merge candidates!");
  pcSPS->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
  READ_FLAG(uiCode, "sps_sbt_enabled_flag");                        pcSPS->setUseSBT                 ( uiCode != 0 );
  READ_FLAG( uiCode,    "sps_affine_enabled_flag" );                            pcSPS->setUseAffine              ( uiCode != 0 );
  if ( pcSPS->getUseAffine() )
  {
    READ_UVLC(uiCode, "five_minus_max_num_subblock_merge_cand");
#if JVET_R0371_MAX_NUM_SUB_BLK_MRG_CAND
    CHECK(uiCode > 5 - (pcSPS->getSBTMVPEnabledFlag() ? 1 : 0), "The value of five_minus_max_num_subblock_merge_cand shall be in the range of 0 to 5 - sps_sbtmvp_enabled_flag");
    CHECK(AFFINE_MRG_MAX_NUM_CANDS < uiCode, "The value of five_minus_max_num_subblock_merge_cand shall be in the range of 0 to 5 - sps_sbtmvp_enabled_flag");
#else
    CHECK(AFFINE_MRG_MAX_NUM_CANDS < uiCode, "Incorrrect max number of affine merge candidates!");
#endif
    pcSPS->setMaxNumAffineMergeCand(AFFINE_MRG_MAX_NUM_CANDS - uiCode);
    READ_FLAG( uiCode,  "sps_affine_type_flag" );                       pcSPS->setUseAffineType          ( uiCode != 0 );
    if( pcSPS->getAMVREnabledFlag())
    {
      READ_FLAG( uiCode, "sps_affine_amvr_enabled_flag" );            pcSPS->setAffineAmvrEnabledFlag  ( uiCode != 0 );
    }
    READ_FLAG( uiCode, "sps_affine_prof_enabled_flag" );            pcSPS->setUsePROF                ( uiCode != 0 );
    if (pcSPS->getUsePROF())
    {
      READ_FLAG(uiCode, "sps_prof_pic_present_flag");               pcSPS->setProfControlPresentFlag ( uiCode != 0 );
    }
    else {
      pcSPS->setProfControlPresentFlag( false );
    }
  }
#endif
  READ_FLAG( uiCode,  "sps_palette_enabled_flag");                                pcSPS->setPLTMode                ( uiCode != 0 );
  if (chromaArrayType == CHROMA_444 && pcSPS->getLog2MaxTbSize() != 6)
  {
    READ_FLAG(uiCode, "sps_act_enabled_flag");                                pcSPS->setUseColorTrans(uiCode != 0);
  }
  else
  {
    pcSPS->setUseColorTrans(false);
  }
  if (pcSPS->getTransformSkipEnabledFlag() || pcSPS->getPLTMode())
  {
#if JVET_R0045_TS_MIN_QP_CLEANUP
    READ_UVLC(uiCode, "sps_internal_bit_depth_minus_input_bit_depth");
    pcSPS->setInternalMinusInputBitDepth(CHANNEL_TYPE_LUMA, uiCode);
    CHECK(uiCode > 8, "Invalid sps_internal_bit_depth_minus_input_bit_depth signalled");
    pcSPS->setInternalMinusInputBitDepth(CHANNEL_TYPE_CHROMA, uiCode);
#else
    READ_UVLC(uiCode, "min_qp_prime_ts_minus4");
    pcSPS->setMinQpPrimeTsMinus4(CHANNEL_TYPE_LUMA, uiCode);
    CHECK(uiCode > 48, "Invalid min_qp_prime_ts_minus4 signalled");
    pcSPS->setMinQpPrimeTsMinus4(CHANNEL_TYPE_CHROMA, uiCode);
#endif
  }
#if !JVET_R0332_HLS_ORDER
  READ_FLAG( uiCode,    "sps_bcw_enabled_flag" );                   pcSPS->setUseBcw( uiCode != 0 );
#endif
  READ_FLAG(uiCode, "sps_ibc_enabled_flag");                                    pcSPS->setIBCFlag(uiCode);
  if (pcSPS->getIBCFlag())
  {
    READ_UVLC(uiCode, "six_minus_max_num_ibc_merge_cand");
    CHECK(IBC_MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of IBC merge candidates!");
    pcSPS->setMaxNumIBCMergeCand(IBC_MRG_MAX_NUM_CANDS - uiCode);
  }
  else
    pcSPS->setMaxNumIBCMergeCand(0);
#if !JVET_R0332_HLS_ORDER
  // KJS: sps_ciip_enabled_flag
  READ_FLAG( uiCode,     "sps_ciip_enabled_flag" );                           pcSPS->setUseCiip             ( uiCode != 0 );
#if !JVET_R0214_MMVD_SYNTAX_MODIFICATION
  if ( pcSPS->getUseMMVD() )
  {
    READ_FLAG( uiCode,  "sps_fpel_mmvd_enabled_flag" );             pcSPS->setFpelMmvdEnabledFlag ( uiCode != 0 );
  }
#endif
  if (pcSPS->getMaxNumMergeCand() >= 2)
  {
    READ_FLAG(uiCode, "sps_gpm_enabled_flag");
    pcSPS->setUseGeo(uiCode != 0);
    if (pcSPS->getUseGeo() && pcSPS->getMaxNumMergeCand() >= 3)
    {
      READ_UVLC(uiCode, "max_num_merge_cand_minus_max_num_gpm_cand");
      CHECK(pcSPS->getMaxNumMergeCand() < uiCode, "Incorrrect max number of GEO candidates!");
      pcSPS->setMaxNumGeoCand((uint32_t)(pcSPS->getMaxNumMergeCand() - uiCode));
    }
    else if (pcSPS->getUseGeo())
      pcSPS->setMaxNumGeoCand(2);
  }
  else
  {
    pcSPS->setUseGeo(0);
    pcSPS->setMaxNumGeoCand(0);
  }
#endif
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
#if !JVET_R0332_HLS_ORDER
  READ_UVLC(uiCode, "log2_parallel_merge_level_minus2");
  CHECK(uiCode + 2 > ctbLog2SizeY, "The value of log2_parallel_merge_level_minus2 shall be in the range of 0 to ctbLog2SizeY - 2");
  pcSPS->setLog2ParallelMergeLevelMinus2(uiCode);
#endif
  READ_FLAG(uiCode, "sps_explicit_scaling_list_enabled_flag");                 pcSPS->setScalingListFlag(uiCode);
#if JVET_R0380_SCALING_MATRIX_DISABLE_YCC_OR_RGB
  if (pcSPS->getUseColorTrans() && pcSPS->getScalingListFlag())
  {
    READ_FLAG(uiCode, "sps_scaling_matrix_for_alternative_colour_space_disabled_flag"); pcSPS->setScalingMatrixForAlternativeColourSpaceDisabledFlag(uiCode);
  }
  if (pcSPS->getScalingMatrixForAlternativeColourSpaceDisabledFlag())
  {
    READ_FLAG(uiCode, "sps_scaling_matrix_designated_colour_space_flag"); pcSPS->setScalingMatrixDesignatedColourSpaceFlag(uiCode);
  }
#endif
  READ_FLAG(uiCode, "sps_dep_quant_enabled_flag"); pcSPS->setDepQuantEnabledFlag(uiCode);
  if (!pcSPS->getDepQuantEnabledFlag())
  {
    READ_FLAG(uiCode, "sps_sign_data_hiding_enabled_flag"); pcSPS->setSignDataHidingEnabledFlag(uiCode);
  }
  else
  {
    pcSPS->setSignDataHidingEnabledFlag(false);
  }
  READ_FLAG( uiCode, "sps_virtual_boundaries_enabled_flag" ); pcSPS->setVirtualBoundariesEnabledFlag( uiCode != 0 );
  if( pcSPS->getVirtualBoundariesEnabledFlag() )
  {
    READ_FLAG( uiCode, "sps_loop_filter_across_virtual_boundaries_present_flag" ); pcSPS->setVirtualBoundariesPresentFlag( uiCode != 0 );
    if( pcSPS->getVirtualBoundariesPresentFlag() )
    {
    READ_CODE( 2, uiCode, "sps_num_ver_virtual_boundaries");        pcSPS->setNumVerVirtualBoundaries( uiCode );
    for( unsigned i = 0; i < pcSPS->getNumVerVirtualBoundaries(); i++ )
    {
#if JVET_R0266_DESC
      READ_UVLC(uiCode, "sps_virtual_boundaries_pos_x");        pcSPS->setVirtualBoundariesPosX(uiCode << 3, i);
#else
      READ_CODE(13, uiCode, "sps_virtual_boundaries_pos_x");        pcSPS->setVirtualBoundariesPosX(uiCode << 3, i);
#endif
    }
    READ_CODE( 2, uiCode, "sps_num_hor_virtual_boundaries");        pcSPS->setNumHorVirtualBoundaries( uiCode );
    for( unsigned i = 0; i < pcSPS->getNumHorVirtualBoundaries(); i++ )
    {
#if JVET_R0266_DESC
      READ_UVLC(uiCode, "sps_virtual_boundaries_pos_y");        pcSPS->setVirtualBoundariesPosY(uiCode << 3, i);
#else
      READ_CODE(13, uiCode, "sps_virtual_boundaries_pos_y");        pcSPS->setVirtualBoundariesPosY(uiCode << 3, i);
#endif
    }
  }
  else
  {
    pcSPS->setNumVerVirtualBoundaries( 0 );
    pcSPS->setNumHorVirtualBoundaries( 0 );
  }
  }

  if (pcSPS->getPtlDpbHrdParamsPresentFlag())
  {
  READ_FLAG(uiCode, "sps_general_hrd_params_present_flag");        pcSPS->setGeneralHrdParametersPresentFlag(uiCode);
  if (pcSPS->getGeneralHrdParametersPresentFlag())
  {
    parseGeneralHrdParameters(pcSPS->getGeneralHrdParameters());
    if ((pcSPS->getMaxTLayers()-1) > 0)
    {
      READ_FLAG(uiCode, "sps_sublayer_cpb_params_present_flag");  pcSPS->setSubLayerParametersPresentFlag(uiCode);
    }
    else if((pcSPS->getMaxTLayers()-1) == 0)
    {
      pcSPS->setSubLayerParametersPresentFlag(0);
    }

    uint32_t firstSubLayer = pcSPS->getSubLayerParametersPresentFlag() ? 0 : (pcSPS->getMaxTLayers() - 1);
    parseOlsHrdParameters(pcSPS->getGeneralHrdParameters(),pcSPS->getOlsHrdParameters(), firstSubLayer, pcSPS->getMaxTLayers() - 1);
  }
  }

  READ_FLAG(     uiCode, "field_seq_flag");                       pcSPS->setFieldSeqFlag(uiCode);

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

void HLSyntaxReader::parseDCI(DCI* dci)
{
#if ENABLE_TRACING
  xTraceDCIHeader();
#endif
  uint32_t  symbol;

#if JVET_R0108_DCI_SIGNALING
  READ_CODE(4, symbol, "dci_reserved_zero_4bits");
#else
  READ_CODE(3, symbol, "dci_max_sub_layers_minus1");          dci->setMaxSubLayersMinus1(symbol);
  READ_CODE(1, symbol, "dci_reserved_zero_bit");              CHECK(symbol != 0, "dci_reserved_zero_bit must be equal to zero");
#endif

  uint32_t numPTLs;
  READ_CODE(4, numPTLs, "dci_num_ptls_minus1");
  numPTLs += 1;

  std::vector<ProfileTierLevel> ptls;
  ptls.resize(numPTLs);
  for (int i = 0; i < numPTLs; i++)
  {
    parseProfileTierLevel(&ptls[i], true, 0);
  }
  dci->setProfileTierLevel(ptls);

  READ_FLAG(symbol, "dci_extension_flag");
  if (symbol)
  {
    while (xMoreRbspData())
    {
      READ_FLAG(symbol, "dci_extension_data_flag");
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

  READ_CODE(6, uiCode, "vps_max_layers_minus1");              pcVPS->setMaxLayers(uiCode + 1);    CHECK(uiCode + 1 > MAX_VPS_LAYERS, "Signalled number of layers larger than MAX_VPS_LAYERS.");
  if (pcVPS->getMaxLayers() - 1 == 0)
  {
    pcVPS->setEachLayerIsAnOlsFlag(1);
  }
  READ_CODE(3, uiCode, "vps_max_sublayers_minus1");           pcVPS->setMaxSubLayers(uiCode + 1); CHECK(uiCode + 1 > MAX_VPS_SUBLAYERS, "Signalled number of sublayers larger than MAX_VPS_SUBLAYERS.");
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
      READ_FLAG(uiCode, "each_layer_is_an_ols_flag");  pcVPS->setEachLayerIsAnOlsFlag(uiCode);
      if (pcVPS->getEachLayerIsAnOlsFlag() == 0)
      {
        pcVPS->setOlsModeIdc(2);
      }
    }
    if (!pcVPS->getEachLayerIsAnOlsFlag())
    {
      if (!pcVPS->getAllIndependentLayersFlag())
      {
        READ_CODE(2, uiCode, "ols_mode_idc");             pcVPS->setOlsModeIdc(uiCode); CHECK(uiCode > MAX_VPS_OLS_MODE_IDC, "ols_mode_idc shall be in the rage of 0 to 2");
      }
      if (pcVPS->getOlsModeIdc() == 2)
      {
        READ_CODE(8, uiCode, "num_output_layer_sets_minus1");   pcVPS->setNumOutputLayerSets(uiCode + 1);
        for (uint32_t i = 1; i <= pcVPS->getNumOutputLayerSets() - 1; i++)
        {
          for (uint32_t j = 0; j < pcVPS->getMaxLayers(); j++)
          {
            READ_FLAG(uiCode, "ols_output_layer_flag");        pcVPS->setOlsOutputLayerFlag(i, j, uiCode);
          }
        }
      }
    }
  }

  pcVPS->deriveOutputLayerSets();
  READ_CODE(8, uiCode, "vps_num_ptls_minus1");        pcVPS->setNumPtls(uiCode + 1);
#if JVET_R0191_ASPECT3
  CHECK( uiCode >= pcVPS->getTotalNumOLSs(),"The value of vps_num_ptls_minus1 shall be less than TotalNumOlss");
  std::vector<bool> isPTLReferred( pcVPS->getNumPtls(), false);
#endif

  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    if(i > 0)
    {
      READ_FLAG(uiCode, "pt_present_flag");
      pcVPS->setPtPresentFlag(i, uiCode);
    }
    else
      pcVPS->setPtPresentFlag(0, 1);
#if JVET_R0107_VPS_SIGNALING
    if (!pcVPS->getAllLayersSameNumSublayersFlag())
#else
    if(pcVPS->getMaxSubLayers() > 1 && !pcVPS->getAllLayersSameNumSublayersFlag())
#endif
    {
      READ_CODE(3, uiCode, "ptl_max_temporal_id");
      pcVPS->setPtlMaxTemporalId(i, uiCode);
    }
#if JVET_R0107_VPS_SIGNALING
    else
    {
      pcVPS->setPtlMaxTemporalId(i, pcVPS->getMaxSubLayers() - 1);
    }
#else
    else if(pcVPS->getMaxSubLayers() > 1)
      pcVPS->setPtlMaxTemporalId(i, pcVPS->getMaxSubLayers() - 1);
    else
      pcVPS->setPtlMaxTemporalId(i, 0);
#endif
  }
  int cnt = 0;
  while (m_pcBitstream->getNumBitsUntilByteAligned())
  {
    READ_FLAG( uiCode, "vps_ptl_alignment_zero_bit");
    CHECK(uiCode!=0, "Alignment bit is not '0'");
    cnt++;
  }
  CHECK(cnt >= 8, "Read more than '8' alignment bits");
  std::vector<ProfileTierLevel> ptls;
  ptls.resize(pcVPS->getNumPtls());
  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    parseProfileTierLevel(&ptls[i], pcVPS->getPtPresentFlag(i), pcVPS->getPtlMaxTemporalId(i) - 1);
  }
  pcVPS->setProfileTierLevel(ptls);
  for (int i = 0; i < pcVPS->getTotalNumOLSs(); i++)
  {
#if JVET_R0161_CONDITION_SIGNAL_PTL_IDX
    if (pcVPS->getNumPtls() > 1 && pcVPS->getNumPtls() != pcVPS->getTotalNumOLSs())
#else
    if(pcVPS->getNumPtls() > 1)
#endif
    {
      READ_CODE(8, uiCode, "ols_ptl_idx");
      pcVPS->setOlsPtlIdx(i, uiCode);
    }
#if JVET_R0161_CONDITION_SIGNAL_PTL_IDX
    else if (pcVPS->getNumPtls() == pcVPS->getTotalNumOLSs())
    {
      pcVPS->setOlsPtlIdx(i, i);
    }
#endif
    else
      pcVPS->setOlsPtlIdx(i, 0);
#if JVET_R0191_ASPECT3
    isPTLReferred[pcVPS->getOlsPtlIdx(i)] = true;
#endif
  }
#if JVET_R0191_ASPECT3
  for( int i = 0; i < pcVPS->getNumPtls(); i++ )
  {
    CHECK( !isPTLReferred[i],"Each profile_tier_level( ) syntax structure in the VPS shall be referred to by at least one value of vps_ols_ptl_idx[ i ] for i in the range of 0 to TotalNumOlss ? 1, inclusive");
  }
#endif

  if( !pcVPS->getAllIndependentLayersFlag() )
  {
    READ_UVLC( uiCode, "vps_num_dpb_params" ); pcVPS->m_numDpbParams = uiCode;
  }

#if JVET_R0191_ASPECT3
  CHECK( pcVPS->m_numDpbParams > pcVPS->getNumMultiLayeredOlss(),"The value of vps_num_dpb_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, inclusive");
  std::vector<bool> isDPBParamReferred(pcVPS->m_numDpbParams, false);
#endif

  if( pcVPS->m_numDpbParams > 0 && pcVPS->getMaxSubLayers() > 1 )
  {
    READ_FLAG( uiCode, "vps_sublayer_dpb_params_present_flag" ); pcVPS->m_sublayerDpbParamsPresentFlag = uiCode;
  }

  pcVPS->m_dpbParameters.resize( pcVPS->m_numDpbParams );

  for( int i = 0; i < pcVPS->m_numDpbParams; i++ )
  {
#if JVET_R0107_VPS_SIGNALING
    if (!pcVPS->getAllLayersSameNumSublayersFlag())
    {
      READ_CODE(3, uiCode, "dpb_max_temporal_id[i]");
      pcVPS->m_dpbMaxTemporalId.push_back(uiCode);
    }
    else
    {
      pcVPS->m_dpbMaxTemporalId.push_back(pcVPS->getMaxSubLayers() - 1);
    }
#else
    if( pcVPS->getMaxSubLayers() == 1 )
    {
      // When vps_max_sublayers_minus1 is equal to 0, the value of dpb_max_temporal_id[ i ] is inferred to be equal to 0.
      pcVPS->m_dpbMaxTemporalId.push_back( 0 );
    }
    else
    {
      if( pcVPS->getAllLayersSameNumSublayersFlag() )
      {
        // When vps_max_sublayers_minus1 is greater than 0 and vps_all_layers_same_num_sublayers_flag is equal to 1, the value of dpb_max_temporal_id[ i ] is inferred to be equal to vps_max_sublayers_minus1.
        pcVPS->m_dpbMaxTemporalId.push_back( pcVPS->getMaxSubLayers() - 1 );
      }
      else
      {
        READ_CODE( 3, uiCode, "dpb_max_temporal_id[i]" );  pcVPS->m_dpbMaxTemporalId.push_back( uiCode );
      }
    }
#endif

    for( int j = ( pcVPS->m_sublayerDpbParamsPresentFlag ? 0 : pcVPS->m_dpbMaxTemporalId[i] ); j <= pcVPS->m_dpbMaxTemporalId[i]; j++ )
    {
      READ_UVLC( uiCode, "max_dec_pic_buffering_minus1[i]" );  pcVPS->m_dpbParameters[i].m_maxDecPicBuffering[j] = uiCode;
      READ_UVLC( uiCode, "max_num_reorder_pics[i]" );          pcVPS->m_dpbParameters[i].m_numReorderPics[j] = uiCode;
      READ_UVLC( uiCode, "max_latency_increase_plus1[i]" );    pcVPS->m_dpbParameters[i].m_maxLatencyIncreasePlus1[j] = uiCode;
    }

    for( int j = ( pcVPS->m_sublayerDpbParamsPresentFlag ? pcVPS->m_dpbMaxTemporalId[i] : 0 ); j < pcVPS->m_dpbMaxTemporalId[i]; j++ )
    {
      // When max_dec_pic_buffering_minus1[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_dec_pic_buffering_minus1[ maxSubLayersMinus1 ].
      pcVPS->m_dpbParameters[i].m_maxDecPicBuffering[j] = pcVPS->m_dpbParameters[i].m_maxDecPicBuffering[pcVPS->m_dpbMaxTemporalId[i]];

      // When max_num_reorder_pics[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_num_reorder_pics[ maxSubLayersMinus1 ].
      pcVPS->m_dpbParameters[i].m_numReorderPics[j] = pcVPS->m_dpbParameters[i].m_numReorderPics[pcVPS->m_dpbMaxTemporalId[i]];

      // When max_latency_increase_plus1[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_latency_increase_plus1[ maxSubLayersMinus1 ].
      pcVPS->m_dpbParameters[i].m_maxLatencyIncreasePlus1[j] = pcVPS->m_dpbParameters[i].m_maxLatencyIncreasePlus1[pcVPS->m_dpbMaxTemporalId[i]];
    }
  }


#if JVET_R0099_DPB_HRD_PARAMETERS_SIGNALLING
  for (int i = 1; i < pcVPS->m_numMultiLayeredOlss; i++)
  {
#else
  for( int i = 0; i < pcVPS->getTotalNumOLSs(); i++ )
  {
    if( pcVPS->m_numLayersInOls[i] > 1 )
    {
#endif
      READ_UVLC( uiCode, "ols_dpb_pic_width[i]" ); pcVPS->setOlsDpbPicWidth( i, uiCode );
      READ_UVLC( uiCode, "ols_dpb_pic_height[i]" ); pcVPS->setOlsDpbPicHeight( i, uiCode );
      if( pcVPS->m_numDpbParams > 1 )
      {
        READ_UVLC( uiCode, "ols_dpb_params_idx[i]" ); pcVPS->setOlsDpbParamsIdx( i, uiCode );
      }
#if JVET_R0191_ASPECT3
#if JVET_R0099_DPB_HRD_PARAMETERS_SIGNALLING
      else if (pcVPS->m_numDpbParams == 0)
      {
        pcVPS->setOlsDpbParamsIdx(i, 0);
      }
      else
      {
        pcVPS->setOlsDpbParamsIdx(i, i);
      }
#else
      else
      {
        pcVPS->setOlsDpbParamsIdx( i, 0 );
      }
#endif
      isDPBParamReferred[pcVPS->getOlsDpbParamsIdx(i)] = true;
#endif
#if !JVET_R0099_DPB_HRD_PARAMETERS_SIGNALLING
    }
#endif
  }
#if JVET_R0191_ASPECT3
  for( int i = 0; i < pcVPS->m_numDpbParams; i++ )
  {
    CHECK( !isDPBParamReferred[i],"Each dpb_parameters( ) syntax structure in the VPS shall be referred to by at least one value of vps_ols_dpb_params_idx[i] for i in the range of 0 to NumMultiLayerOlss - 1, inclusive");
  }
#endif
  if (!pcVPS->getEachLayerIsAnOlsFlag())
  {
    READ_FLAG(uiCode, "vps_general_hrd_params_present_flag");  pcVPS->setVPSGeneralHrdParamsPresentFlag(uiCode);
  }
  if (pcVPS->getVPSGeneralHrdParamsPresentFlag())
  {
    parseGeneralHrdParameters(pcVPS->getGeneralHrdParameters());
    if ((pcVPS->getMaxSubLayers() - 1) > 0)
    {
      READ_FLAG(uiCode, "vps_sublayer_cpb_params_present_flag");  pcVPS->setVPSSublayerCpbParamsPresentFlag(uiCode);
    }
    else
    {
      pcVPS->setVPSSublayerCpbParamsPresentFlag(0);
    }

    READ_UVLC(uiCode, "num_ols_hrd_params_minus1"); pcVPS->setNumOlsHrdParamsMinus1(uiCode);
#if JVET_R0191_ASPECT3
    CHECK( uiCode >= pcVPS->getNumMultiLayeredOlss(),"The value of vps_num_ols_hrd_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, inclusive");
    std::vector<bool> isHRDParamReferred( uiCode + 1, false);
#else
    CHECK(uiCode >= pcVPS->getTotalNumOLSs(),"The value of num_ols_hrd_params_minus1 shall be in the range of 0 to TotalNumOlss - 1, inclusive");
#endif
    pcVPS->m_olsHrdParams.clear();
    pcVPS->m_olsHrdParams.resize(pcVPS->getNumOlsHrdParamsMinus1(), std::vector<OlsHrdParams>(pcVPS->getMaxSubLayers()));
    for (int i = 0; i <= pcVPS->getNumOlsHrdParamsMinus1(); i++)
    {
#if JVET_R0107_VPS_SIGNALING
      if (!pcVPS->getAllLayersSameNumSublayersFlag())
#else
      if (((pcVPS->getMaxSubLayers() - 1) > 0) && (!pcVPS->getAllLayersSameNumSublayersFlag()))
#endif
      {
        READ_CODE(3, uiCode, "hrd_max_tid[i]");  pcVPS->setHrdMaxTid(i, uiCode);
      }
      else
      {
#if JVET_R0107_VPS_SIGNALING
        pcVPS->setHrdMaxTid(i, pcVPS->getMaxSubLayers() - 1);
#else
        if (pcVPS->getMaxSubLayers() == 1)
        {
          pcVPS->setHrdMaxTid(i, 0);
        }
        else if ((pcVPS->getMaxSubLayers() >= 1)&&(pcVPS->getAllLayersSameNumSublayersFlag()))
        {
          pcVPS->setHrdMaxTid(i, pcVPS->getMaxSubLayers()- 1);
        }
#endif

      }
      uint32_t firstSublayer = pcVPS->getVPSSublayerCpbParamsPresentFlag() ? 0 : pcVPS->getHrdMaxTid(i);
      parseOlsHrdParameters(pcVPS->getGeneralHrdParameters(),pcVPS->getOlsHrdParameters(i), firstSublayer, pcVPS->getHrdMaxTid(i));
    }
#if JVET_R0099_DPB_HRD_PARAMETERS_SIGNALLING
    for (int i = 0; i < pcVPS->m_numMultiLayeredOlss; i++) 
    {
      if (((pcVPS->getNumOlsHrdParamsMinus1() + 1) != pcVPS->m_numMultiLayeredOlss) && (pcVPS->getNumOlsHrdParamsMinus1() > 0))
      {
        READ_UVLC(uiCode, "ols_hrd_idx[i]"); pcVPS->setOlsHrdIdx(i, uiCode);
        CHECK(uiCode > pcVPS->getNumOlsHrdParamsMinus1(), "The value of ols_hrd_idx[[ i ] shall be in the range of 0 to num_ols_hrd_params_minus1, inclusive.");
      }
      else if (pcVPS->getNumOlsHrdParamsMinus1() == 0)
      {
        pcVPS->setOlsHrdIdx(i, 0);
      }
      else
      {
        pcVPS->setOlsHrdIdx(i, i);
      }
#if JVET_R0191_ASPECT3
      isHRDParamReferred[pcVPS->getOlsHrdIdx(i)] = true;
#endif
    }
#else
    for (int i = 1; i < pcVPS->getTotalNumOLSs(); i++)
    {
      if (((pcVPS->getNumOlsHrdParamsMinus1() + 1) != pcVPS->getTotalNumOLSs()) && (pcVPS->getNumOlsHrdParamsMinus1() > 0))
      {
        if (pcVPS->m_numLayersInOls[i] > 1)
        {
          READ_UVLC(uiCode, "ols_hrd_idx[i]"); pcVPS->setOlsHrdIdx(i, uiCode);
          CHECK(uiCode > pcVPS->getNumOlsHrdParamsMinus1(), "The value of ols_hrd_idx[[ i ] shall be in the range of 0 to num_ols_hrd_params_minus1, inclusive.");
        }
      }
      else if ((pcVPS->getNumOlsHrdParamsMinus1() + 1) == pcVPS->getTotalNumOLSs())
      {
        pcVPS->setOlsHrdIdx(i, i);
      }
      else if ((pcVPS->m_numLayersInOls[i] > 1) && (pcVPS->getNumOlsHrdParamsMinus1() == 0))
      {
        pcVPS->setOlsHrdIdx(i, 0);
      }
#if JVET_R0191_ASPECT3
      isHRDParamReferred[pcVPS->getOlsHrdIdx(i)] = true;
#endif
    }
#endif
#if JVET_R0191_ASPECT3
    for( int i = 0; i <= pcVPS->getNumOlsHrdParamsMinus1(); i++ )
    {
      CHECK( !isHRDParamReferred[i], "Each ols_hrd_parameters( ) syntax structure in the VPS shall be referred to by at least one value of vps_ols_hrd_idx[ i ] for i in the range of 1 to NumMultiLayerOlss - 1, inclusive");
    }
#endif
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

void HLSyntaxReader::parsePictureHeader( PicHeader* picHeader, ParameterSetManager *parameterSetManager, bool readRbspTrailingBits )
{
  uint32_t  uiCode;
  int       iCode;
  PPS*      pps = NULL;
  SPS*      sps = NULL;

#if ENABLE_TRACING
  xTracePictureHeader();
#endif

  READ_FLAG(uiCode, "gdr_or_irap_pic_flag");               picHeader->setGdrOrIrapPicFlag(uiCode != 0);
  if (picHeader->getGdrOrIrapPicFlag())
  {
    READ_FLAG(uiCode, "gdr_pic_flag");                     picHeader->setGdrPicFlag(uiCode != 0);
  }
#if JVET_R0112_SEMANTICS_CLEANUP
  else
  {
    picHeader->setGdrPicFlag(false);
  }
#endif
  READ_FLAG(uiCode, "ph_inter_slice_allowed_flag");       picHeader->setPicInterSliceAllowedFlag(uiCode != 0);
  if (picHeader->getPicInterSliceAllowedFlag())
  {
    READ_FLAG(uiCode, "ph_intra_slice_allowed_flag");       picHeader->setPicIntraSliceAllowedFlag(uiCode != 0);
  }
  else
  {
    picHeader->setPicIntraSliceAllowedFlag(true);
  }
  CHECK(picHeader->getPicInterSliceAllowedFlag() == 0 && picHeader->getPicIntraSliceAllowedFlag() == 0, "Invalid picture without intra or inter slice");
  READ_FLAG(uiCode, "non_reference_picture_flag");       picHeader->setNonReferencePictureFlag( uiCode != 0 );
  // parameter sets
  READ_UVLC(uiCode, "ph_pic_parameter_set_id");
  picHeader->setPPSId(uiCode);
  pps = parameterSetManager->getPPS(picHeader->getPPSId());
  CHECK(pps == 0, "Invalid PPS");
  picHeader->setSPSId(pps->getSPSId());
  sps = parameterSetManager->getSPS(picHeader->getSPSId());
  CHECK(sps == 0, "Invalid SPS");
  READ_CODE(sps->getBitsForPOC(), uiCode, "ph_pic_order_cnt_lsb");
  picHeader->setPocLsb(uiCode);
  if (picHeader->getGdrOrIrapPicFlag())
  {
    READ_FLAG(uiCode, "no_output_of_prior_pics_flag");   picHeader->setNoOutputOfPriorPicsFlag(uiCode != 0);
  }
  if( picHeader->getGdrPicFlag() )
  {
    READ_UVLC(uiCode, "recovery_poc_cnt");               picHeader->setRecoveryPocCnt( uiCode );
  }
  else
  {
    picHeader->setRecoveryPocCnt( 0 );
  }

  std::vector<bool> phExtraBitsPresent = sps->getExtraPHBitPresentFlags();
  for (int i=0; i< sps->getNumExtraPHBitsBytes() * 8; i++)
  {
    // extra bits are ignored (when present)
    if (phExtraBitsPresent[i])
    {
      READ_FLAG(uiCode, "ph_extra_bit[ i ]");
    }
  }

  if (sps->getPocMsbFlag())
  {
    READ_FLAG(uiCode, "ph_poc_msb_present_flag"); picHeader->setPocMsbPresentFlag(uiCode != 0);
    if (picHeader->getPocMsbPresentFlag())
    {
      READ_CODE(sps->getPocMsbLen(), uiCode, "poc_msb_val");
      picHeader->setPocMsbVal(uiCode);
    }
  }


  // alf enable flags and aps IDs
  picHeader->setCcAlfEnabledFlag(COMPONENT_Cb, false);
  picHeader->setCcAlfEnabledFlag(COMPONENT_Cr, false);
  if (sps->getALFEnabledFlag())
  {
    if (pps->getAlfInfoInPhFlag())
    {
      READ_FLAG(uiCode, "ph_alf_enabled_flag");
      picHeader->setAlfEnabledFlag(COMPONENT_Y, uiCode);

#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
      int alfCbEnabledFlag = 0;
      int alfCrEnabledFlag = 0;
#else
      int alfChromaIdc = 0;
#endif
      if (uiCode)
      {
        READ_CODE(3, uiCode, "ph_num_alf_aps_ids_luma");
        int numAps = uiCode;
        picHeader->setNumAlfAps(numAps);

        std::vector<int> apsId(numAps, -1);
        for (int i = 0; i < numAps; i++)
        {
          READ_CODE(3, uiCode, "ph_alf_aps_id_luma");
          apsId[i] = uiCode;
        }
        picHeader->setAlfAPSs(apsId);

        if (sps->getChromaFormatIdc() != CHROMA_400)
        {
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
          READ_CODE(1, uiCode, "ph_alf_cb_enabled_flag");   alfCbEnabledFlag = uiCode;
          READ_CODE(1, uiCode, "ph_alf_cr_enabled_flag");   alfCrEnabledFlag = uiCode;
#else
          READ_CODE(2, uiCode, "ph_alf_chroma_idc");
          alfChromaIdc = uiCode;
#endif
        }
        else
        {
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
          alfCbEnabledFlag = 0;
          alfCrEnabledFlag = 0;
#else
          alfChromaIdc = 0;
#endif
        }
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
        if (alfCbEnabledFlag || alfCrEnabledFlag)
#else
        picHeader->setAlfChromaIdc(alfChromaIdc);
        if (alfChromaIdc)
#endif
        {
          READ_CODE(3, uiCode, "ph_alf_aps_id_chroma");
          picHeader->setAlfApsIdChroma(uiCode);
        }
        if (sps->getCCALFEnabledFlag())
        {
          READ_FLAG(uiCode, "ph_cc_alf_cb_enabled_flag");
          picHeader->setCcAlfEnabledFlag(COMPONENT_Cb, uiCode != 0);
          picHeader->setCcAlfCbApsId(-1);
          if (picHeader->getCcAlfEnabledFlag(COMPONENT_Cb))
          {
            // parse APS ID
            READ_CODE(3, uiCode, "ph_cc_alf_cb_aps_id");
            picHeader->setCcAlfCbApsId(uiCode);
          }
          // Cr
          READ_FLAG(uiCode, "ph_cc_alf_cr_enabled_flag");
          picHeader->setCcAlfEnabledFlag(COMPONENT_Cr, uiCode != 0);
          picHeader->setCcAlfCrApsId(-1);
          if (picHeader->getCcAlfEnabledFlag(COMPONENT_Cr))
          {
            // parse APS ID
            READ_CODE(3, uiCode, "ph_cc_alf_cr_aps_id");
            picHeader->setCcAlfCrApsId(uiCode);
          }
        }
      }
      else
      {
        picHeader->setNumAlfAps(0);
      }
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
      picHeader->setAlfEnabledFlag(COMPONENT_Cb, alfCbEnabledFlag);
      picHeader->setAlfEnabledFlag(COMPONENT_Cr, alfCrEnabledFlag);
#else
      picHeader->setAlfEnabledFlag(COMPONENT_Cb, alfChromaIdc & 1);
      picHeader->setAlfEnabledFlag(COMPONENT_Cr, alfChromaIdc >> 1);
#endif
    }
    else
    {
      picHeader->setAlfEnabledFlag(COMPONENT_Y, true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cb, true);
      picHeader->setAlfEnabledFlag(COMPONENT_Cr, true);
    }
  }
  else
  {
    picHeader->setAlfEnabledFlag(COMPONENT_Y, false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cb, false);
    picHeader->setAlfEnabledFlag(COMPONENT_Cr, false);
  }
  // luma mapping / chroma scaling controls
  if (sps->getUseLmcs())
  {
    READ_FLAG(uiCode, "ph_lmcs_enabled_flag");
    picHeader->setLmcsEnabledFlag(uiCode != 0);

    if (picHeader->getLmcsEnabledFlag())
    {
      READ_CODE(2, uiCode, "ph_lmcs_aps_id");
      picHeader->setLmcsAPSId(uiCode);

      if (sps->getChromaFormatIdc() != CHROMA_400)
      {
        READ_FLAG(uiCode, "ph_chroma_residual_scale_flag");
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
  if (sps->getScalingListFlag())
  {
    READ_FLAG(uiCode, "ph_explicit_scaling_list_enabled_flag");
    picHeader->setExplicitScalingListEnabledFlag(uiCode);
    if (picHeader->getExplicitScalingListEnabledFlag())
    {
      READ_CODE(3, uiCode, "ph_scaling_list_aps_id");
      picHeader->setScalingListAPSId(uiCode);
    }
  }
  else
  {
    picHeader->setExplicitScalingListEnabledFlag(false);
  }

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
    pps->initRectSliceMap(sps);
    // when no Pic partition, number of sub picture shall be less than 2
    CHECK(pps->getNumSubPics()>=2, "error, no picture partitions, but have equal to or more than 2 sub pictures");
  }
  else
  {
    CHECK(pps->getCtuSize() != sps->getCTUSize(), "PPS CTU size does not match CTU size in SPS");
    if (pps->getRectSliceFlag())
    {
      pps->initRectSliceMap(sps);
    }
  }

  pps->initSubPic(*sps);

#if JVET_Q0764_WRAP_AROUND_WITH_RPR
  // set wraparound offset from PPS and SPS info
  int minCbSizeY = (1 << sps->getLog2MinCodingBlockSize());
  CHECK( !sps->getWrapAroundEnabledFlag() && pps->getWrapAroundEnabledFlag(), "When sps_ref_wraparound_enabled_flag is equal to 0, the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");
  CHECK( (((sps->getCTUSize() / minCbSizeY) + 1) > ((pps->getPicWidthInLumaSamples() / minCbSizeY) - 1)) && pps->getWrapAroundEnabledFlag(), "When the value of CtbSizeY / MinCbSizeY + 1 is greater than pic_width_in_luma_samples / MinCbSizeY - 1, the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");
  if( pps->getWrapAroundEnabledFlag() )
  {
#if JVET_R0162_WRAPAROUND_OFFSET_SIGNALING
    CHECK((pps->getPicWidthMinusWrapAroundOffset() > (pps->getPicWidthInLumaSamples() / minCbSizeY - sps->getCTUSize() / minCbSizeY - 2)), "pps_pic_width_minus_wraparound_ofsfet shall be less than or equal to pps_pic_width_in_luma_samples/MinCbSizeY - CtbSizeY/MinCbSizeY-2");
    pps->setWrapAroundOffset(minCbSizeY * (pps->getPicWidthInLumaSamples()/minCbSizeY- pps->getPicWidthMinusWrapAroundOffset()));
#else
    pps->setWrapAroundOffset( minCbSizeY * (pps->getWrapAroundOffsetMinusCtbSize() + 2 + sps->getCTUSize() / minCbSizeY) );
#endif
  }
  else
  {
    pps->setWrapAroundOffset( 0 );
  }
#endif

  // virtual boundaries
  if( sps->getVirtualBoundariesEnabledFlag() && !sps->getVirtualBoundariesPresentFlag() )
  {
    READ_FLAG( uiCode, "ph_virtual_boundaries_present_flag" );
    picHeader->setVirtualBoundariesPresentFlag( uiCode != 0 );
    CHECK( sps->getSubPicInfoPresentFlag() && picHeader->getVirtualBoundariesPresentFlag(), "When the subpicture info is present, the signalling of the virtual boundary position, if present, shall be in SPS" );
    if( picHeader->getVirtualBoundariesPresentFlag() )
    {
      READ_CODE( 2, uiCode, "ph_num_ver_virtual_boundaries");        picHeader->setNumVerVirtualBoundaries( uiCode );
      for( unsigned i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++ )
      {
#if JVET_R0266_DESC
        READ_UVLC(uiCode, "ph_virtual_boundaries_pos_x");        picHeader->setVirtualBoundariesPosX(uiCode << 3, i);
#else
        READ_CODE(13, uiCode, "ph_virtual_boundaries_pos_x");        picHeader->setVirtualBoundariesPosX(uiCode << 3, i);
#endif
      }
      READ_CODE( 2, uiCode, "ph_num_hor_virtual_boundaries");        picHeader->setNumHorVirtualBoundaries( uiCode );
      for( unsigned i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++ )
      {
#if JVET_R0266_DESC
        READ_UVLC(uiCode, "ph_virtual_boundaries_pos_y");        picHeader->setVirtualBoundariesPosY(uiCode << 3, i);
#else
        READ_CODE(13, uiCode, "ph_virtual_boundaries_pos_y");        picHeader->setVirtualBoundariesPosY(uiCode << 3, i);
#endif
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
    picHeader->setVirtualBoundariesPresentFlag( sps->getVirtualBoundariesPresentFlag() );
    if( picHeader->getVirtualBoundariesPresentFlag() )
    {
    picHeader->setNumVerVirtualBoundaries( sps->getNumVerVirtualBoundaries() );
    picHeader->setNumHorVirtualBoundaries( sps->getNumHorVirtualBoundaries() );
    for( unsigned i = 0; i < 3; i++ )
    {
      picHeader->setVirtualBoundariesPosX( sps->getVirtualBoundariesPosX(i), i );
      picHeader->setVirtualBoundariesPosY( sps->getVirtualBoundariesPosY(i), i );
    }
    }
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
  if (pps->getRplInfoInPhFlag())
  {
    bool rplSpsFlag0 = 0;

    // List0 and List1
    for(int listIdx = 0; listIdx < 2; listIdx++)
    {
      if (sps->getNumRPL(listIdx) > 0 &&
          (listIdx == 0 || (listIdx == 1 && pps->getRpl1IdxPresentFlag())))
      {
        READ_FLAG(uiCode, "rpl_sps_flag[i]");
      }
      else if (sps->getNumRPL(listIdx) == 0)
      {
        uiCode = 0;
      }
      else
      {
        uiCode = rplSpsFlag0;
      }

      if (listIdx == 0)
      {
        rplSpsFlag0 = uiCode;
      }

      // explicit RPL in picture header
      if (!uiCode)
      {
        ReferencePictureList* rpl = picHeader->getLocalRPL( listIdx );
        (*rpl) = ReferencePictureList();
#if JVET_R0059_RPL_CLEANUP
        parseRefPicList(sps, rpl, -1);
#else
        parseRefPicList(sps, rpl);
#endif
        picHeader->setRPLIdx(listIdx, -1);
        picHeader->setRPL(listIdx, rpl);
      }
      // use list from SPS
      else
      {
        if (sps->getNumRPL(listIdx) > 1 &&
            (listIdx == 0 || (listIdx == 1 && pps->getRpl1IdxPresentFlag())))
        {
          int numBits = ceilLog2(sps->getNumRPL( listIdx ));
          READ_CODE(numBits, uiCode, "rpl_idx[i]");
          picHeader->setRPLIdx( listIdx, uiCode );
          picHeader->setRPL( listIdx, sps->getRPLList( listIdx )->getReferencePictureList(uiCode));
        }
        else if (sps->getNumRPL(listIdx) == 1)
        {
          picHeader->setRPLIdx( listIdx, 0 );
          picHeader->setRPL( listIdx, sps->getRPLList( listIdx )->getReferencePictureList(0));
        }
        else
        {
          assert(picHeader->getRPLIdx(0) != -1);
          picHeader->setRPLIdx( listIdx, picHeader->getRPLIdx(0));
          picHeader->setRPL( listIdx, sps->getRPLList( listIdx )->getReferencePictureList(picHeader->getRPLIdx( listIdx )));
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
              READ_CODE(sps->getBitsForPOC(), uiCode, "poc_lsb_lt[i][j]");
              picHeader->getLocalRPL( listIdx )->setRefPicIdentifier( i, uiCode, true, false, 0 );
            }
            READ_FLAG(uiCode, "delta_poc_msb_present_flag[i][j]");
            picHeader->getLocalRPL( listIdx )->setDeltaPocMSBPresentFlag(i, uiCode ? true : false);
            if (uiCode)
            {
              READ_UVLC(uiCode, "delta_poc_msb_cycle_lt[i][j]");
              if(i != 0)
              {
                uiCode += picHeader->getLocalRPL( listIdx )->getDeltaPocMSBCycleLT(i-1);
              }
              picHeader->getLocalRPL( listIdx )->setDeltaPocMSBCycleLT(i, uiCode);
            }
            else if(i != 0)
            {
              picHeader->getLocalRPL( listIdx )->setDeltaPocMSBCycleLT(i, picHeader->getLocalRPL( listIdx )->getDeltaPocMSBCycleLT(i-1));
            }
            else
            {
              picHeader->getLocalRPL( listIdx )->setDeltaPocMSBCycleLT(i,0);
            }
          }
          else if(i != 0)
          {
            picHeader->getLocalRPL( listIdx )->setDeltaPocMSBCycleLT(i, picHeader->getLocalRPL( listIdx )->getDeltaPocMSBCycleLT(i-1));
          }
          else
          {
            picHeader->getLocalRPL( listIdx )->setDeltaPocMSBCycleLT(i,0);
          }
        }
      }
    }
  }

  // partitioning constraint overrides
  if (sps->getSplitConsOverrideEnabledFlag())
  {
    READ_FLAG(uiCode, "partition_constraints_override_flag");  picHeader->setSplitConsOverrideFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setSplitConsOverrideFlag(0);
  }
  // Q0781, two-flags
  unsigned  minQT[3] = { 0, 0, 0 };
  unsigned  maxBTD[3] = { 0, 0, 0 };
  unsigned  maxBTSize[3] = { 0, 0, 0 };
  unsigned  maxTTSize[3] = { 0, 0, 0 };
  unsigned  ctbLog2SizeY = floorLog2(sps->getCTUSize());

  if (picHeader->getPicIntraSliceAllowedFlag())
  {
    if (picHeader->getSplitConsOverrideFlag())
    {
      READ_UVLC(uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_luma");
      unsigned minQtLog2SizeIntraY = uiCode + sps->getLog2MinCodingBlockSize();
      minQT[0] = 1 << minQtLog2SizeIntraY;
#if JVET_R0347_MTT_SIZE_CONSTRAIN
      CHECK(minQT[0] > 64, "The value of ph_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size");
#endif
      READ_UVLC(uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_luma");         maxBTD[0] = uiCode;

      maxTTSize[0] = maxBTSize[0] = minQT[0];
      if (maxBTD[0] != 0)
      {
        READ_UVLC(uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_luma");       maxBTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "The value of ph_log2_diff_max_bt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeIntraY");
        READ_UVLC(uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_luma");       maxTTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "The value of ph_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeIntraY");
#if JVET_R0347_MTT_SIZE_CONSTRAIN
        CHECK(maxTTSize[0] > 64, "The value of ph_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraY");
#endif
      }

      if (sps->getUseDualITree())
      {
        READ_UVLC(uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_chroma");     minQT[2] = 1 << (uiCode + sps->getLog2MinCodingBlockSize());
#if JVET_R0347_MTT_SIZE_CONSTRAIN
        CHECK(minQT[2] > 64, "The value of ph_log2_diff_min_qt_min_cb_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size");
#endif
        READ_UVLC(uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_chroma");     maxBTD[2] = uiCode;
        maxTTSize[2] = maxBTSize[2] = minQT[2];
        if (maxBTD[2] != 0)
        {
          READ_UVLC(uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_chroma");   maxBTSize[2] <<= uiCode;
          READ_UVLC(uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_chroma");   maxTTSize[2] <<= uiCode;
#if JVET_R0347_MTT_SIZE_CONSTRAIN
          CHECK(maxBTSize[2] > 64, "The value of ph_log2_diff_max_bt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
          CHECK(maxTTSize[2] > 64, "The value of ph_log2_diff_max_tt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
#endif
        }
      }
    }
  }


  if (picHeader->getPicIntraSliceAllowedFlag())
  {
  // delta quantization and chrom and chroma offset
    if (pps->getUseDQP())
    {
      READ_UVLC( uiCode, "ph_cu_qp_delta_subdiv_intra_slice" );   picHeader->setCuQpDeltaSubdivIntra( uiCode );
    }
    else
    {
      picHeader->setCuQpDeltaSubdivIntra( 0 );
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      READ_UVLC( uiCode, "ph_cu_chroma_qp_offset_subdiv_intra_slice" );   picHeader->setCuChromaQpOffsetSubdivIntra( uiCode );
    }
    else
    {
      picHeader->setCuChromaQpOffsetSubdivIntra( 0 );
    }
  }


  if (picHeader->getPicInterSliceAllowedFlag())
  {
    if (picHeader->getSplitConsOverrideFlag())
    {
      READ_UVLC(uiCode, "ph_log2_diff_min_qt_min_cb_inter_slice");
      unsigned minQtLog2SizeInterY = uiCode + sps->getLog2MinCodingBlockSize();
      minQT[1] = 1 << minQtLog2SizeInterY;
#if JVET_R0347_MTT_SIZE_CONSTRAIN
      CHECK(minQT[1] > 64, "The value of ph_log2_diff_min_qt_min_cb_inter_slice shall be in the range of 0 to min(6, CtbLog2SizeY) - MinCbLog2SizeY.");
      CHECK(minQT[1] > (1<<ctbLog2SizeY), "The value of ph_log2_diff_min_qt_min_cb_inter_slice shall be in the range of 0 to min(6, CtbLog2SizeY) - MinCbLog2SizeY");
#endif
      READ_UVLC(uiCode, "ph_max_mtt_hierarchy_depth_inter_slice");              maxBTD[1] = uiCode;

      maxTTSize[1] = maxBTSize[1] = minQT[1];
      if (maxBTD[1] != 0)
      {
        READ_UVLC(uiCode, "ph_log2_diff_max_bt_min_qt_inter_slice");            maxBTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "The value of ph_log2_diff_max_bt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeInterY");
        READ_UVLC(uiCode, "ph_log2_diff_max_tt_min_qt_inter_slice");            maxTTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "The value of ph_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - MinQtLog2SizeInterY");
#if JVET_R0347_MTT_SIZE_CONSTRAIN
        CHECK(maxTTSize[1] > 64, "The value of ph_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeInterY.");
#endif
      }
    }
    // delta quantization and chrom and chroma offset
    if (pps->getUseDQP())
    {
      READ_UVLC(uiCode, "ph_cu_qp_delta_subdiv_inter_slice");   picHeader->setCuQpDeltaSubdivInter(uiCode);
    }
    else
    {
      picHeader->setCuQpDeltaSubdivInter(0);
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      READ_UVLC(uiCode, "ph_cu_chroma_qp_offset_subdiv_inter_slice");   picHeader->setCuChromaQpOffsetSubdivInter(uiCode);
    }
    else
    {
      picHeader->setCuChromaQpOffsetSubdivInter(0);
    }

  // temporal motion vector prediction
    if (sps->getSPSTemporalMVPEnabledFlag())
    {
      READ_FLAG( uiCode, "ph_temporal_mvp_enabled_flag" );
      picHeader->setEnableTMVPFlag( uiCode != 0 );
    }
    else
    {
      picHeader->setEnableTMVPFlag(false);
    }

    if (picHeader->getEnableTMVPFlag() && pps->getRplInfoInPhFlag())
    {
#if R0324_PH_SYNTAX_CONDITION_MODIFY
      if (picHeader->getRPL(1)->getNumRefEntries() > 0)
      {
        READ_CODE(1, uiCode, "ph_collocated_from_l0_flag");
        picHeader->setPicColFromL0Flag(uiCode);
      }
      else
      {
        picHeader->setPicColFromL0Flag(1);
      }
#else
      READ_CODE( 1, uiCode, "ph_collocated_from_l0_flag");
      picHeader->setPicColFromL0Flag(uiCode);
#endif
      if ((picHeader->getPicColFromL0Flag() == 1 && picHeader->getRPL(0)->getNumRefEntries() > 1) ||
        (picHeader->getPicColFromL0Flag() == 0 && picHeader->getRPL(1)->getNumRefEntries() > 1))
      {
        READ_UVLC(uiCode, "ph_collocated_ref_idx");
        picHeader->setColRefIdx(uiCode);
      }
      else
      {
        picHeader->setColRefIdx(0);
      }
    }
    else
    {
      picHeader->setPicColFromL0Flag(0);
    }

  // mvd L1 zero flag
#if R0324_PH_SYNTAX_CONDITION_MODIFY
    if (!pps->getRplInfoInPhFlag() || picHeader->getRPL(1)->getNumRefEntries() > 0)
    {
      READ_FLAG(uiCode, "pic_mvd_l1_zero_flag");
    }
    else
    {
      uiCode = 1;
    }
#else
    READ_FLAG(uiCode, "mvd_l1_zero_flag");
#endif
    picHeader->setMvdL1ZeroFlag( uiCode != 0 );

  // merge candidate list size
  // subblock merge candidate list size
    if ( sps->getUseAffine() )
    {
      picHeader->setMaxNumAffineMergeCand(sps->getMaxNumAffineMergeCand());
    }
    else
    {
      picHeader->setMaxNumAffineMergeCand( sps->getSBTMVPEnabledFlag() && picHeader->getEnableTMVPFlag() );
    }

  // full-pel MMVD flag
    if (sps->getFpelMmvdEnabledFlag())
    {
      READ_FLAG( uiCode, "ph_fpel_mmvd_enabled_flag" );
      picHeader->setDisFracMMVD( uiCode != 0 );
    }
    else
    {
      picHeader->setDisFracMMVD(false);
    }

  // picture level BDOF disable flags
#if R0324_PH_SYNTAX_CONDITION_MODIFY
    if (sps->getBdofControlPresentFlag() && (!pps->getRplInfoInPhFlag() || picHeader->getRPL(1)->getNumRefEntries() > 0))
#else
    if (sps->getBdofControlPresentFlag())
#endif
    {
      READ_FLAG(uiCode, "ph_disable_bdof_flag");  picHeader->setDisBdofFlag(uiCode != 0);
    }
    else
    {
#if R0324_PH_SYNTAX_CONDITION_MODIFY
      if (sps->getBdofControlPresentFlag() == 0)
      {
        picHeader->setDisBdofFlag(1 - (int)(sps->getBDOFEnabledFlag()));
      }
      else
      {
        picHeader->setDisBdofFlag(1);
      }
#else
      picHeader->setDisBdofFlag(0);
#endif
    }

  // picture level DMVR disable flags
#if R0324_PH_SYNTAX_CONDITION_MODIFY
    if (sps->getDmvrControlPresentFlag() && (!pps->getRplInfoInPhFlag() || picHeader->getRPL(1)->getNumRefEntries() > 0))
#else
    if (sps->getDmvrControlPresentFlag())
#endif
    {
      READ_FLAG(uiCode, "ph_disable_dmvr_flag");  picHeader->setDisDmvrFlag(uiCode != 0);
    }
    else
    {
#if R0324_PH_SYNTAX_CONDITION_MODIFY
      if (sps->getDmvrControlPresentFlag() == 0)
      {
        picHeader->setDisDmvrFlag(1 - (int)(sps->getUseDMVR()));
      }
      else
      {
        picHeader->setDisDmvrFlag(1);
      }
#else
      picHeader->setDisDmvrFlag(0);
#endif
    }

  // picture level PROF disable flags
    if (sps->getProfControlPresentFlag())
    {
      READ_FLAG(uiCode, "ph_disable_prof_flag");  picHeader->setDisProfFlag(uiCode != 0);
    }
    else
    {
      picHeader->setDisProfFlag(0);
    }

    if( (pps->getUseWP() || pps->getWPBiPred()) && pps->getWpInfoInPhFlag() )
    {
      parsePredWeightTable(picHeader, sps);
    }
  }
  // inherit constraint values from SPS
  if (!sps->getSplitConsOverrideEnabledFlag() || !picHeader->getSplitConsOverrideFlag())
  {
    picHeader->setMinQTSizes(sps->getMinQTSizes());
    picHeader->setMaxMTTHierarchyDepths(sps->getMaxMTTHierarchyDepths());
    picHeader->setMaxBTSizes(sps->getMaxBTSizes());
    picHeader->setMaxTTSizes(sps->getMaxTTSizes());
  }
  else
  {
    picHeader->setMinQTSizes(minQT);
    picHeader->setMaxMTTHierarchyDepths(maxBTD);
    picHeader->setMaxBTSizes(maxBTSize);
    picHeader->setMaxTTSizes(maxTTSize);
  }
  // ibc merge candidate list size
  if (pps->getQpDeltaInfoInPhFlag())
  {
    int iCode = 0;
    READ_SVLC(iCode, "ph_qp_delta");
    picHeader->setQpDelta(iCode);
  }

  // joint Cb/Cr sign flag
  if (sps->getJointCbCrEnabledFlag())
  {
    READ_FLAG( uiCode, "ph_joint_cbcr_sign_flag" );
    picHeader->setJointCbCrSignFlag(uiCode != 0);
  }
  else
  {
    picHeader->setJointCbCrSignFlag(false);
  }

  // sao enable flags
  if(sps->getSAOEnabledFlag())
  {
    if (pps->getSaoInfoInPhFlag())
    {
      READ_FLAG(uiCode, "ph_sao_luma_enabled_flag");
      picHeader->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, uiCode != 0);

      if (sps->getChromaFormatIdc() != CHROMA_400)
      {
        READ_FLAG(uiCode, "ph_sao_chroma_enabled_flag");
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


#if !JVET_R0271_SLICE_LEVEL_DQ_SDH_RRC
  // dependent quantization
  if (sps->getDepQuantEnabledFlag())
  {
    READ_FLAG(uiCode, "ph_dep_quant_enabled_flag");
    picHeader->setDepQuantEnabledFlag(uiCode != 0);
  }
  else
  {
    picHeader->setDepQuantEnabledFlag(false);
  }

  // sign data hiding
  if (sps->getSignDataHidingEnabledFlag() && !picHeader->getDepQuantEnabledFlag())
  {
    READ_FLAG( uiCode, "pic_sign_data_hiding_enabled_flag" );
    picHeader->setSignDataHidingEnabledFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setSignDataHidingEnabledFlag(false);
  }
#endif

  // deblocking filter controls
  if (pps->getDeblockingFilterControlPresentFlag())
  {
    if(pps->getDeblockingFilterOverrideEnabledFlag())
    {
      if (pps->getDbfInfoInPhFlag())
      {
        READ_FLAG ( uiCode, "ph_deblocking_filter_override_flag" );
        picHeader->setDeblockingFilterOverrideFlag(uiCode != 0);
      }
      else
      {
        picHeader->setDeblockingFilterOverrideFlag(false);
      }
    }
    else
    {
      picHeader->setDeblockingFilterOverrideFlag(false);
    }

    if(picHeader->getDeblockingFilterOverrideFlag())
    {
#if JVET_R0388_DBF_CLEANUP
      if (!pps->getPPSDeblockingFilterDisabledFlag())
      {
#endif
        READ_FLAG(uiCode, "ph_deblocking_filter_disabled_flag");
        picHeader->setDeblockingFilterDisable(uiCode != 0);
#if JVET_R0388_DBF_CLEANUP
      }
      else
      {
        picHeader->setDeblockingFilterDisable(false);
      }
#endif
      if (!picHeader->getDeblockingFilterDisable())
      {
        READ_SVLC( iCode, "ph_beta_offset_div2" );
        picHeader->setDeblockingFilterBetaOffsetDiv2(iCode);
        CHECK(  picHeader->getDeblockingFilterBetaOffsetDiv2() < -12 ||
                picHeader->getDeblockingFilterBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration");

        READ_SVLC( iCode, "ph_tc_offset_div2" );
        picHeader->setDeblockingFilterTcOffsetDiv2(iCode);
        CHECK(  picHeader->getDeblockingFilterTcOffsetDiv2() < -12 ||
                picHeader->getDeblockingFilterTcOffsetDiv2() > 12, "Invalid deblocking filter configuration");

#if JVET_R0078_DISABLE_CHROMA_DBF_OFFSET_SINGALLING
        if( pps->getPPSChromaToolFlag() )
        {
#endif
          READ_SVLC( iCode, "ph_cb_beta_offset_div2" );
          picHeader->setDeblockingFilterCbBetaOffsetDiv2(iCode);
          CHECK(  picHeader->getDeblockingFilterCbBetaOffsetDiv2() < -12 ||
                  picHeader->getDeblockingFilterCbBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration");

          READ_SVLC( iCode, "ph_cb_tc_offset_div2" );
          picHeader->setDeblockingFilterCbTcOffsetDiv2(iCode);
          CHECK(  picHeader->getDeblockingFilterCbTcOffsetDiv2() < -12 ||
                  picHeader->getDeblockingFilterCbTcOffsetDiv2() > 12, "Invalid deblocking filter configuration");

          READ_SVLC( iCode, "ph_cr_beta_offset_div2" );
          picHeader->setDeblockingFilterCrBetaOffsetDiv2(iCode);
          CHECK(  picHeader->getDeblockingFilterCrBetaOffsetDiv2() < -12 ||
                  picHeader->getDeblockingFilterCrBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration");

          READ_SVLC( iCode, "ph_cr_tc_offset_div2" );
          picHeader->setDeblockingFilterCrTcOffsetDiv2(iCode);
          CHECK(  picHeader->getDeblockingFilterCrTcOffsetDiv2() < -12 ||
                  picHeader->getDeblockingFilterCrTcOffsetDiv2() > 12, "Invalid deblocking filter configuration");
#if JVET_R0078_DISABLE_CHROMA_DBF_OFFSET_SINGALLING
        }
        else
        {
          picHeader->setDeblockingFilterCbBetaOffsetDiv2 ( picHeader->getDeblockingFilterBetaOffsetDiv2() );
          picHeader->setDeblockingFilterCbTcOffsetDiv2   ( picHeader->getDeblockingFilterTcOffsetDiv2()   );
          picHeader->setDeblockingFilterCrBetaOffsetDiv2 ( picHeader->getDeblockingFilterBetaOffsetDiv2() );
          picHeader->setDeblockingFilterCrTcOffsetDiv2   ( picHeader->getDeblockingFilterTcOffsetDiv2()   );
        }
#endif
      }
    }
    else
    {
      picHeader->setDeblockingFilterDisable       ( pps->getPPSDeblockingFilterDisabledFlag() );
      picHeader->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
      picHeader->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
      picHeader->setDeblockingFilterCbBetaOffsetDiv2( pps->getDeblockingFilterCbBetaOffsetDiv2() );
      picHeader->setDeblockingFilterCbTcOffsetDiv2  ( pps->getDeblockingFilterCbTcOffsetDiv2() );
      picHeader->setDeblockingFilterCrBetaOffsetDiv2( pps->getDeblockingFilterCrBetaOffsetDiv2() );
      picHeader->setDeblockingFilterCrTcOffsetDiv2  ( pps->getDeblockingFilterCrTcOffsetDiv2() );
    }
  }
  else
  {
    picHeader->setDeblockingFilterDisable       ( false );
    picHeader->setDeblockingFilterBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterTcOffsetDiv2  ( 0 );
    picHeader->setDeblockingFilterCbBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterCbTcOffsetDiv2  ( 0 );
    picHeader->setDeblockingFilterCrBetaOffsetDiv2(0);
    picHeader->setDeblockingFilterCrTcOffsetDiv2(0);
  }


  // picture header extension
  if(pps->getPictureHeaderExtensionPresentFlag())
  {
    READ_UVLC(uiCode,"ph_extension_length");
    for(int i=0; i<uiCode; i++)
    {
      uint32_t ignore_;
      READ_CODE(8,ignore_,"ph_extension_data_byte");
    }
  }

  if( readRbspTrailingBits )
  {
    xReadRbspTrailingBits();
  }
}

void  HLSyntaxReader::checkAlfNaluTidAndPicTid(Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager)
{
  SPS* sps = parameterSetManager->getSPS(picHeader->getSPSId());
  PPS* pps = parameterSetManager->getPPS(picHeader->getPPSId());
  int curPicTid = pcSlice->getTLayer();
  APS* aps;
  const std::vector<int>&   apsId = picHeader->getAlfAPSs();

  if (sps->getALFEnabledFlag() && pps->getAlfInfoInPhFlag() && picHeader->getAlfEnabledFlag(COMPONENT_Y))
  {
    //luma
    for (int i = 0; i < picHeader->getNumAlfAps(); i++)
    {
      aps = parameterSetManager->getAPS(apsId[i], ALF_APS);
      CHECK(aps->getTemporalId() > curPicTid, "The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and adaptation_parameter_set_id equal to ph_alf_aps_id_luma[ i ] shall be less than or equal to the TemporalId of the picture associated with the PH.");
#if JVET_R0194_CONSTRAINT_PS_SHARING_REFERENCING
      if( pcSlice->getNalUnitLayerId() != aps->getLayerId() )
      {
        CHECK( aps->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
        CHECK( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
        for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls = false;
          for( int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
          {
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
            {
              isCurrLayerInOls = true;
            }
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == aps->getLayerId() )
            {
              isRefLayerInOls = true;
            }
          }
          CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
        }
      }
#endif
    }
    //chroma
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
    if (picHeader->getAlfEnabledFlag(COMPONENT_Cb) || picHeader->getAlfEnabledFlag(COMPONENT_Cr))
#else
    if (picHeader->getAlfChromaIdc())
#endif
    {
      int chromaAlfApsId = picHeader->getAlfApsIdChroma();
      aps = parameterSetManager->getAPS(chromaAlfApsId, ALF_APS);
      CHECK(aps->getTemporalId() > curPicTid, "The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and adaptation_parameter_set_id equal to ph_alf_aps_id_chroma shall be less than or equal to the TemporalId of the picture associated with the PH.");
#if JVET_R0194_CONSTRAINT_PS_SHARING_REFERENCING
      if( pcSlice->getNalUnitLayerId() != aps->getLayerId() )
      {
        CHECK( aps->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
        CHECK( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
        for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls = false;
          for( int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
          {
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
            {
              isCurrLayerInOls = true;
            }
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == aps->getLayerId() )
            {
              isRefLayerInOls = true;
            }
          }
          CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
        }
      }
#endif
    }
  }
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
  READ_FLAG(uiCode, "picture_header_in_slice_header_flag");
  if (uiCode)
  {
    pcSlice->setPictureHeaderInSliceHeader(true);
    parsePictureHeader(picHeader, parameterSetManager, false);
    picHeader->setValid();
  }
  CHECK(picHeader==0, "Invalid Picture Header");
  CHECK(picHeader->isValid()==false, "Invalid Picture Header");
  checkAlfNaluTidAndPicTid(pcSlice, picHeader, parameterSetManager);
  pps = parameterSetManager->getPPS( picHeader->getPPSId() );
  //!KS: need to add error handling code here, if PPS is not available
  CHECK(pps==0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  CHECK(sps==0, "Invalid SPS");
#if JVET_R0202_WHEN_PH_IN_SH_INFO_FLAGS_EQUAL_0
  CHECK(pcSlice->getPictureHeaderInSliceHeader() && pps->getRplInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, rpl_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->getPictureHeaderInSliceHeader() && pps->getDbfInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, dbf_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->getPictureHeaderInSliceHeader() && pps->getSaoInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, sao_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->getPictureHeaderInSliceHeader() && pps->getWpInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, wp_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->getPictureHeaderInSliceHeader() && pps->getQpDeltaInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, qp_delta_info_in_ph_flag shall be equal to 0");
#endif
#if JVET_R0202_WHEN_PH_IN_SH_NO_SUBPIC_SEPARATE_COLOR
  CHECK(pcSlice->getPictureHeaderInSliceHeader() && sps->getSubPicInfoPresentFlag() == 1, "When sps_subpic_info_present_flag is equal to 1, the value of sh_picture_header_in_slice_header_flag shall be equal to 0");
  CHECK(pcSlice->getPictureHeaderInSliceHeader() && sps->getSeparateColourPlaneFlag() == 1, "when separate_colour_plane_flag is equal to 1, the value of picture_header_in_slice_header_flag shall be equal to 0");
#endif

  const ChromaFormat chFmt = sps->getChromaFormatIdc();
  const uint32_t numValidComp=getNumberValidComponents(chFmt);
  const bool bChroma=(chFmt!=CHROMA_400);

  // picture order count
  uiCode = picHeader->getPocLsb();
  int iPOClsb = uiCode;
  int iMaxPOClsb = 1 << sps->getBitsForPOC();
  int iPOCmsb;
  if (pcSlice->getIdrPicFlag())
  {
    if (picHeader->getPocMsbPresentFlag())
    {
      iPOCmsb = picHeader->getPocMsbVal()*iMaxPOClsb;
    }
    else
    {
      iPOCmsb = 0;
    }
    pcSlice->setPOC(iPOCmsb + iPOClsb);
  }
  else
  {
    int iPrevPOC = prevTid0POC;
    int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
    int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
    if (picHeader->getPocMsbPresentFlag())
    {
      iPOCmsb = picHeader->getPocMsbVal()*iMaxPOClsb;
    }
    else
    {
      if ((iPOClsb < iPrevPOClsb) && ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2)))
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if ((iPOClsb > iPrevPOClsb) && ((iPOClsb - iPrevPOClsb) > (iMaxPOClsb / 2)))
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
    }
    pcSlice->setPOC(iPOCmsb + iPOClsb);
  }

  if (sps->getSubPicInfoPresentFlag())
  {
    uint32_t bitsSubPicId;
    bitsSubPicId = sps->getSubPicIdLen();
    READ_CODE(bitsSubPicId, uiCode, "slice_subpic_id");    pcSlice->setSliceSubPicId(uiCode);
  }
  else
  {
    pcSlice->setSliceSubPicId(0);
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
#if JVET_R0210_NUMTILESINSLICE_SIGNALLING
      if (((int)pps->getNumTiles() - (int)sliceAddr) > 1)
      {
#endif
        READ_UVLC(uiCode, "num_tiles_in_slice_minus1");        numTilesInSlice = uiCode + 1;
#if JVET_R0210_NUMTILESINSLICE_SIGNALLING
      }
      else
      {
        numTilesInSlice = 1;
      }
#endif
      if (!pps->getRectSliceFlag() && sps->getProfileTierLevel()->getConstraintInfo()->getOneSlicePerPicConstraintFlag())
      {
        CHECK(pps->getNumTiles() != uiCode + 1, "When rect_slice_flag is equal to 0 and one_slice_per_pic_constraint_flag equal to 1, the value of num_tiles_in_slice_minus1 present in each slice header shall be equal to NumTilesInPic - 1");
      }
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
    uint32_t currSubPicIdx = pps->getSubPicIdxFromSubPicId( pcSlice->getSliceSubPicId() );
    SubPic currSubPic = pps->getSubPic(currSubPicIdx);
    if( currSubPic.getNumSlicesInSubPic() > 1 )
    {
      int bitsSliceAddress = ceilLog2(currSubPic.getNumSlicesInSubPic());
      READ_CODE(bitsSliceAddress, uiCode, "slice_address");  sliceAddr = uiCode;
      CHECK(sliceAddr >= pps->getNumSlicesInPic(), "Invalid slice address");
    }
    else
    {
      sliceAddr = 0;
    }
    uint32_t picLevelSliceIdx = sliceAddr;
    for(int subpic = 0; subpic < currSubPicIdx; subpic++)
    {
      picLevelSliceIdx += pps->getSubPic(subpic).getNumSlicesInSubPic();
    }
    pcSlice->setSliceMap( pps->getSliceMap(picLevelSliceIdx) );
    pcSlice->setSliceID(picLevelSliceIdx);
  }

  std::vector<bool> shExtraBitsPresent = sps->getExtraSHBitPresentFlags();
  for (int i=0; i< sps->getNumExtraSHBitsBytes() * 8; i++)
  {
    // extra bits are ignored (when present)
    if (shExtraBitsPresent[i])
    {
      READ_FLAG(uiCode, "sh_extra_bit[ i ]");
    }
  }

  if (picHeader->getPicInterSliceAllowedFlag())
  {
    READ_UVLC (    uiCode, "slice_type" );            pcSlice->setSliceType((SliceType)uiCode);
    VPS *vps = parameterSetManager->getVPS(sps->getVPSId());
    if (pcSlice->isIRAP() && (sps->getVPSId() == 0 || vps->getIndependentLayerFlag(vps->getGeneralLayerIdx(pcSlice->getNalUnitLayerId())) == 1))
    {
      CHECK(uiCode != 2, "When nal_unit_type is in the range of IDR_W_RADL to CRA_NUT, inclusive, and vps_independent_layer_flag[ GeneralLayerIdx[ nuh_layer_id ] ] is equal to 1, slice_type shall be equal to 2");
    }
  }
  else
  {
    pcSlice->setSliceType(I_SLICE);
  }
  if (!picHeader->getPicIntraSliceAllowedFlag())
  {
    CHECK(pcSlice->getSliceType() == I_SLICE, "when ph_intra_slice_allowed_flag = 0, no I_Slice is allowed");
  }

  // inherit values from picture header
  //   set default values in case slice overrides are disabled
  pcSlice->inheritFromPicHeader(picHeader, pps, sps);

  if (sps->getALFEnabledFlag() && !pps->getAlfInfoInPhFlag())
  {
    READ_FLAG(uiCode, "slice_alf_enabled_flag");
    pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Y, uiCode);
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
    int alfCbEnabledFlag = 0;
    int alfCrEnabledFlag = 0;
#else
    int alfChromaIdc = 0;
#endif

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
        CHECK(APStoCheckLuma == nullptr, "referenced APS not found");
        CHECK(APStoCheckLuma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] != 1, "bitstream conformance error, alf_luma_filter_signal_flag shall be equal to 1");
      }


      pcSlice->setAlfAPSs(apsId);
      if (bChroma)
      {
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
        READ_CODE(1, uiCode, "slice_alf_cb_enabled_flag");   alfCbEnabledFlag = uiCode;
        READ_CODE(1, uiCode, "slice_alf_cr_enabled_flag");   alfCrEnabledFlag = uiCode;
#else
        READ_CODE(2, uiCode, "slice_alf_chroma_idc");   alfChromaIdc = uiCode;
#endif
      }
      else
      {
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
        alfCbEnabledFlag = 0;
        alfCrEnabledFlag = 0;
#else
        alfChromaIdc = 0;
#endif
      }
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
      if (alfCbEnabledFlag || alfCrEnabledFlag)
#else
      if (alfChromaIdc)
#endif
      {
        READ_CODE(3, uiCode, "slice_alf_aps_id_chroma");
        pcSlice->setTileGroupApsIdChroma(uiCode);
        APS* APStoCheckChroma = parameterSetManager->getAPS(uiCode, ALF_APS);
        CHECK(APStoCheckChroma == nullptr, "referenced APS not found");
        CHECK(APStoCheckChroma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] != 1, "bitstream conformance error, alf_chroma_filter_signal_flag shall be equal to 1");
      }
    }
    else
    {
      pcSlice->setTileGroupNumAps(0);
    }
#if JVET_R0225_SEPERATE_FLAGS_ALF_CHROMA
    pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cb, alfCbEnabledFlag);
    pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cr, alfCrEnabledFlag);
#else
    pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cb, alfChromaIdc & 1);
    pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cr, alfChromaIdc >> 1);
#endif

    CcAlfFilterParam &filterParam = pcSlice->m_ccAlfFilterParam;
    if (sps->getCCALFEnabledFlag() && pcSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y))
    {
      READ_FLAG(uiCode, "slice_cc_alf_cb_enabled_flag");
      pcSlice->setTileGroupCcAlfCbEnabledFlag(uiCode);
      filterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1] = (uiCode == 1) ? true : false;
      pcSlice->setTileGroupCcAlfCbApsId(-1);
      if (filterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1])
      {
        // parse APS ID
        READ_CODE(3, uiCode, "slice_cc_alf_cb_aps_id");
        pcSlice->setTileGroupCcAlfCbApsId(uiCode);
      }
      // Cr
      READ_FLAG(uiCode, "slice_cc_alf_cr_enabled_flag");
      pcSlice->setTileGroupCcAlfCrEnabledFlag(uiCode);
      filterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1] = (uiCode == 1) ? true : false;
      pcSlice->setTileGroupCcAlfCrApsId(-1);
      if (filterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1])
      {
        // parse APS ID
        READ_CODE(3, uiCode, "slice_cc_alf_cr_aps_id");
        pcSlice->setTileGroupCcAlfCrApsId(uiCode);
      }
    }
    else
    {
      filterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1] = false;
      filterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1] = false;
      pcSlice->setTileGroupCcAlfCbApsId(-1);
      pcSlice->setTileGroupCcAlfCrApsId(-1);
    }
  }
#if JVET_R0200_MOVE_LMCS_AND_SCALING_LIST_SE
#if JVET_R0098_LMCS_AND_SCALING_LISTS_FOR_PH_IN_SH
  if (picHeader->getLmcsEnabledFlag() && !pcSlice->getPictureHeaderInSliceHeader())
#else
  if (picHeader->getLmcsEnabledFlag())
#endif
  {
    READ_FLAG(uiCode, "slice_lmcs_enabled_flag");
    pcSlice->setLmcsEnabledFlag(uiCode);
  }
  else
  {
#if JVET_R0098_LMCS_AND_SCALING_LISTS_FOR_PH_IN_SH
    pcSlice->setLmcsEnabledFlag(pcSlice->getPictureHeaderInSliceHeader() ? picHeader->getLmcsEnabledFlag() : false);
#else
    pcSlice->setLmcsEnabledFlag(false);
#endif
  }
#if JVET_R0098_LMCS_AND_SCALING_LISTS_FOR_PH_IN_SH
  if (picHeader->getExplicitScalingListEnabledFlag() && !pcSlice->getPictureHeaderInSliceHeader())
#else
  if (picHeader->getExplicitScalingListEnabledFlag())
#endif
  {
    READ_FLAG(uiCode, "slice_explicit_scaling_list_used_flag");
    pcSlice->setExplicitScalingListUsed(uiCode);
  }
  else
  {
#if JVET_R0098_LMCS_AND_SCALING_LISTS_FOR_PH_IN_SH
    pcSlice->setExplicitScalingListUsed(pcSlice->getPictureHeaderInSliceHeader() ? picHeader->getExplicitScalingListEnabledFlag() : false);
#else
    pcSlice->setExplicitScalingListUsed(false);
#endif
  }
#endif

    // 4:4:4 colour plane ID
    if( sps->getSeparateColourPlaneFlag() )
    {
      READ_CODE( 2, uiCode, "colour_plane_id" ); pcSlice->setColourPlaneId( uiCode );
      CHECK( uiCode > 2, "colour_plane_id exceeds valid range" );
    }
    else
    {
      pcSlice->setColourPlaneId( 0 );
    }


    if( pps->getRplInfoInPhFlag() )
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
      bool rplSpsFlag0 = 0;

      if (sps->getNumRPL0() > 0)
      {
        READ_FLAG(uiCode, "ref_pic_list_sps_flag[0]");
      }
      else
      {
        uiCode = 0;
      }

      rplSpsFlag0 = uiCode;

      if (!uiCode) //explicitly carried in this SH
      {
        ReferencePictureList* rpl0 = pcSlice->getLocalRPL0();
        (*rpl0) = ReferencePictureList();
#if JVET_R0059_RPL_CLEANUP
        parseRefPicList(sps, rpl0, -1);
#else
        parseRefPicList(sps, rpl0);
#endif
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
              if(i != 0)
              {
                uiCode += pcSlice->getLocalRPL0()->getDeltaPocMSBCycleLT(i-1);
              }
              pcSlice->getLocalRPL0()->setDeltaPocMSBCycleLT(i, uiCode);
            }
            else if(i != 0)
            {
              pcSlice->getLocalRPL0()->setDeltaPocMSBCycleLT(i, pcSlice->getLocalRPL0()->getDeltaPocMSBCycleLT(i-1));
            }
            else
            {
              pcSlice->getLocalRPL0()->setDeltaPocMSBCycleLT(i,0);
            }
          }
          else if(i != 0)
          {
            pcSlice->getLocalRPL0()->setDeltaPocMSBCycleLT(i, pcSlice->getLocalRPL0()->getDeltaPocMSBCycleLT(i-1));
          }
          else
          {
            pcSlice->getLocalRPL0()->setDeltaPocMSBCycleLT(i,0);
          }
        }
      }

      //Read L1 related syntax elements
      if (sps->getNumRPL(1) > 0 && pps->getRpl1IdxPresentFlag())
      {
          READ_FLAG(uiCode, "ref_pic_list_sps_flag[1]");
      }
      else if (sps->getNumRPL(1) == 0)
      {
        uiCode = 0;
      }
      else
      {
        uiCode = rplSpsFlag0;
      }

      if (uiCode == 1)
      {
        if (sps->getNumRPL(1) > 1 && pps->getRpl1IdxPresentFlag())
        {
          int numBits = ceilLog2(sps->getNumRPL1());
          READ_CODE(numBits, uiCode, "ref_pic_list_idx[1]");
          pcSlice->setRPL1idx(uiCode);
          pcSlice->setRPL1(sps->getRPLList1()->getReferencePictureList(uiCode));
        }
        else if (sps->getNumRPL(1) == 1)
        {
          pcSlice->setRPL1idx(0);
          pcSlice->setRPL1(sps->getRPLList1()->getReferencePictureList(0));
        }
        else
        {
          assert(pcSlice->getRPL0idx() != -1);
          pcSlice->setRPL1idx(pcSlice->getRPL0idx());
          pcSlice->setRPL1(sps->getRPLList1()->getReferencePictureList(pcSlice->getRPL0idx()));
        }
      }
      else
      {
        ReferencePictureList* rpl1 = pcSlice->getLocalRPL1();
        (*rpl1) = ReferencePictureList();
#if JVET_R0059_RPL_CLEANUP
        parseRefPicList(sps, rpl1, -1);
#else
        parseRefPicList(sps, rpl1);
#endif
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
              if(i != 0)
              {
                uiCode += pcSlice->getLocalRPL1()->getDeltaPocMSBCycleLT(i-1);
              }
              pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i, uiCode);
            }
            else if(i != 0)
            {
              pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i, pcSlice->getLocalRPL1()->getDeltaPocMSBCycleLT(i-1));
            }
            else
            {
              pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i,0);
            }
          }
          else if(i != 0)
          {
            pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i, pcSlice->getLocalRPL1()->getDeltaPocMSBCycleLT(i-1));
          }
          else
          {
            pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i,0);
          }
        }
      }

    }
    if( !pps->getRplInfoInPhFlag() && pcSlice->getIdrPicFlag() && !(sps->getIDRRefParamListPresent()) )
    {
      pcSlice->setNumRefIdx(REF_PIC_LIST_0, 0);
      pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
    }
#if !JVET_R0277_RPL
    else
    {
#endif
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
#if !JVET_R0277_RPL
    }
#endif

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

    if ( picHeader->getEnableTMVPFlag() )
    {
#if JVET_R0277_RPL
      if( pcSlice->getSliceType() == P_SLICE )
      {
        pcSlice->setColFromL0Flag( true );
      }
      else if( !pps->getRplInfoInPhFlag() && pcSlice->getSliceType() == B_SLICE )
      {
        READ_FLAG( uiCode, "collocated_from_l0_flag" );
        pcSlice->setColFromL0Flag( uiCode );
      }
      else
      {
        pcSlice->setColFromL0Flag( picHeader->getPicColFromL0Flag() );
      }
#else
      if ( !pps->getRplInfoInPhFlag())
      {
      if ( pcSlice->getSliceType() == B_SLICE )
      {
        READ_FLAG(uiCode, "collocated_from_l0_flag");
        pcSlice->setColFromL0Flag(uiCode);
      }
      else
      {
        pcSlice->setColFromL0Flag( 1 );
      }
      }
      else
      {
        pcSlice->setColFromL0Flag(picHeader->getPicColFromL0Flag());
      }
#endif

      if (!pps->getRplInfoInPhFlag())
      {
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
      else
      {
        pcSlice->setColRefIdx(picHeader->getColRefIdx());
      }
    }
    if ( (pps->getUseWP() && pcSlice->getSliceType()==P_SLICE) || (pps->getWPBiPred() && pcSlice->getSliceType()==B_SLICE) )
    {
      if (pps->getWpInfoInPhFlag())
      {
        CHECK(pcSlice->getNumRefIdx(REF_PIC_LIST_0) > picHeader->getNumL0Weights(), "ERROR: Number of active reference picture L0 is greater than the number of weighted prediction signalled in Picture Header");
        CHECK(pcSlice->getNumRefIdx(REF_PIC_LIST_1) > picHeader->getNumL1Weights(), "ERROR: Number of active reference picture L1 is greater than the number of weighted prediction signalled in Picture Header");
        pcSlice->setWpScaling(picHeader->getWpScalingAll());
      }
      else
      {
        parsePredWeightTable(pcSlice, sps);
      }
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

    int qpDelta = 0;
    if (pps->getQpDeltaInfoInPhFlag())
    {
      qpDelta = picHeader->getQpDelta();
    }
    else
    {
      READ_SVLC(iCode, "slice_qp_delta");
      qpDelta = iCode;
    }
    pcSlice->setSliceQp(26 + pps->getPicInitQPMinus26() + qpDelta);
    pcSlice->setSliceQpBase(pcSlice->getSliceQp());

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

    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      READ_FLAG(uiCode, "cu_chroma_qp_offset_enabled_flag"); pcSlice->setUseChromaQpAdj(uiCode != 0);
    }
    else
    {
      pcSlice->setUseChromaQpAdj(false);
    }

    if (sps->getSAOEnabledFlag() && !pps->getSaoInfoInPhFlag())
    {
      READ_FLAG(uiCode, "slice_sao_luma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, (bool)uiCode);

      if (bChroma)
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, (bool)uiCode);
      }
    }


    if (pps->getDeblockingFilterControlPresentFlag())
    {
      if (pps->getDeblockingFilterOverrideEnabledFlag() && !pps->getDbfInfoInPhFlag())
      {
        READ_FLAG ( uiCode, "slice_deblocking_filter_override_flag" );        pcSlice->setDeblockingFilterOverrideFlag(uiCode ? true : false);
      }
      else
      {
        pcSlice->setDeblockingFilterOverrideFlag(0);
      }
      if(pcSlice->getDeblockingFilterOverrideFlag())
      {
#if JVET_R0388_DBF_CLEANUP
        if (!pps->getPPSDeblockingFilterDisabledFlag())
        {
#endif
          READ_FLAG(uiCode, "slice_deblocking_filter_disabled_flag");   pcSlice->setDeblockingFilterDisable(uiCode ? 1 : 0);
#if JVET_R0388_DBF_CLEANUP
        }
        else
        {
          pcSlice->setDeblockingFilterDisable(false);
        }
#endif
        if(!pcSlice->getDeblockingFilterDisable())
        {
          READ_SVLC( iCode, "slice_beta_offset_div2" );                     pcSlice->setDeblockingFilterBetaOffsetDiv2( iCode );
          CHECK(  pcSlice->getDeblockingFilterBetaOffsetDiv2() < -12 ||
                  pcSlice->getDeblockingFilterBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration");
          READ_SVLC( iCode, "slice_tc_offset_div2" );                       pcSlice->setDeblockingFilterTcOffsetDiv2( iCode );
          CHECK(  pcSlice->getDeblockingFilterTcOffsetDiv2() < -12 ||
                  pcSlice->getDeblockingFilterTcOffsetDiv2() > 12, "Invalid deblocking filter configuration");

#if JVET_R0078_DISABLE_CHROMA_DBF_OFFSET_SINGALLING
          if( pps->getPPSChromaToolFlag() )
          {
#endif
            READ_SVLC( iCode, "slice_cb_beta_offset_div2" );                  pcSlice->setDeblockingFilterCbBetaOffsetDiv2( iCode );
            CHECK( pcSlice->getDeblockingFilterCbBetaOffsetDiv2() < -12 ||
              pcSlice->getDeblockingFilterCbBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration" );
            READ_SVLC( iCode, "slice_cb_tc_offset_div2" );                    pcSlice->setDeblockingFilterCbTcOffsetDiv2( iCode );
            CHECK( pcSlice->getDeblockingFilterCbTcOffsetDiv2() < -12 ||
              pcSlice->getDeblockingFilterCbTcOffsetDiv2() > 12, "Invalid deblocking filter configuration" );

            READ_SVLC( iCode, "slice_cr_beta_offset_div2" );                  pcSlice->setDeblockingFilterCrBetaOffsetDiv2( iCode );
            CHECK( pcSlice->getDeblockingFilterCrBetaOffsetDiv2() < -12 ||
              pcSlice->getDeblockingFilterCrBetaOffsetDiv2() > 12, "Invalid deblocking filter configuration" );
            READ_SVLC( iCode, "slice_cr_tc_offset_div2" );                    pcSlice->setDeblockingFilterCrTcOffsetDiv2( iCode );
            CHECK( pcSlice->getDeblockingFilterCrTcOffsetDiv2() < -12 ||
              pcSlice->getDeblockingFilterCrTcOffsetDiv2() > 12, "Invalid deblocking filter configuration" );
#if JVET_R0078_DISABLE_CHROMA_DBF_OFFSET_SINGALLING
          }
          else
          {
            pcSlice->setDeblockingFilterCbBetaOffsetDiv2 ( pcSlice->getDeblockingFilterBetaOffsetDiv2() );
            pcSlice->setDeblockingFilterCbTcOffsetDiv2   ( pcSlice->getDeblockingFilterTcOffsetDiv2()   );
            pcSlice->setDeblockingFilterCrBetaOffsetDiv2 ( pcSlice->getDeblockingFilterBetaOffsetDiv2() );
            pcSlice->setDeblockingFilterCrTcOffsetDiv2   ( pcSlice->getDeblockingFilterTcOffsetDiv2()   );
          }
#endif
        }
      }
      else
      {
        pcSlice->setDeblockingFilterDisable       ( picHeader->getDeblockingFilterDisable() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( picHeader->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( picHeader->getDeblockingFilterTcOffsetDiv2() );
        pcSlice->setDeblockingFilterCbBetaOffsetDiv2( picHeader->getDeblockingFilterCbBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( picHeader->getDeblockingFilterCbTcOffsetDiv2() );
        pcSlice->setDeblockingFilterCrBetaOffsetDiv2(picHeader->getDeblockingFilterCrBetaOffsetDiv2());
        pcSlice->setDeblockingFilterCrTcOffsetDiv2(picHeader->getDeblockingFilterCrTcOffsetDiv2());
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable       ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
      pcSlice->setDeblockingFilterCbBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( 0 );
      pcSlice->setDeblockingFilterCrBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( 0 );
    }

#if JVET_R0271_SLICE_LEVEL_DQ_SDH_RRC
  // dependent quantization
  if( sps->getDepQuantEnabledFlag() )
  {
    READ_FLAG(uiCode, "slice_dep_quant_enabled_flag");
    pcSlice->setDepQuantEnabledFlag(uiCode != 0);
  }
  else
  {
    pcSlice->setDepQuantEnabledFlag(false);
  }

  // sign data hiding
  if( sps->getSignDataHidingEnabledFlag() && !pcSlice->getDepQuantEnabledFlag() )
  {
    READ_FLAG( uiCode, "slice_sign_data_hiding_enabled_flag" );
    pcSlice->setSignDataHidingEnabledFlag( uiCode != 0 );
  }
  else
  {
    pcSlice->setSignDataHidingEnabledFlag(false);
  }

  // signal TS residual coding disabled flag
#if JVET_R0483_SH_TSRC_DISABLED_FLAG_CLEANUP
  if (sps->getTransformSkipEnabledFlag() && !pcSlice->getDepQuantEnabledFlag() && !pcSlice->getSignDataHidingEnabledFlag())
#else
  if( !pcSlice->getDepQuantEnabledFlag() && !pcSlice->getSignDataHidingEnabledFlag() )
#endif
  {
    READ_FLAG(uiCode, "slice_ts_residual_coding_disabled_flag");
    pcSlice->setTSResidualCodingDisabledFlag( uiCode != 0 );
  }
  else
  {
    pcSlice->setTSResidualCodingDisabledFlag( false );
  }
#else
	READ_FLAG(uiCode, "slice_ts_residual_coding_disabled_flag");
	pcSlice->setTSResidualCodingDisabledFlag(uiCode != 0);
#endif

#if !JVET_R0200_MOVE_LMCS_AND_SCALING_LIST_SE
  if (picHeader->getLmcsEnabledFlag())
  {
    READ_FLAG(uiCode, "slice_lmcs_enabled_flag");
    pcSlice->setLmcsEnabledFlag(uiCode);
  }
  else
  {
    pcSlice->setLmcsEnabledFlag(false);
  }
  if (picHeader->getExplicitScalingListEnabledFlag())
  {
    READ_FLAG(uiCode, "slice_explicit_scaling_list_used_flag");
    pcSlice->setExplicitScalingListUsed(uiCode);
  }
  else
  {
    pcSlice->setExplicitScalingListUsed(false);
  }
#endif
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

  pcSlice->resetNumberOfSubstream();
  pcSlice->setNumSubstream(sps, pps);

  pcSlice->setNumEntryPoints( sps, pps );
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

void HLSyntaxReader::getSlicePoc(Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager, const int prevTid0POC)
{
  uint32_t  uiCode;
  uint32_t  pocLsb;
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

  READ_FLAG(uiCode, "picture_header_in_slice_header_flag");
  if (uiCode == 0)
  {
    pocLsb = picHeader->getPocLsb();
  }
  else
  {
    READ_FLAG(uiCode, "gdr_or_irap_pic_flag");
    if (uiCode)
    {
      READ_FLAG(uiCode, "gdr_pic_flag");
    }
    READ_FLAG(uiCode, "ph_inter_slice_allowed_flag");
    if (uiCode)
    {
      READ_FLAG(uiCode, "ph_intra_slice_allowed_flag");
    }
    READ_FLAG(uiCode, "non_reference_picture_flag");
    // parameter sets
    READ_UVLC(uiCode, "ph_pic_parameter_set_id");
    // picture order count
    READ_CODE(sps->getBitsForPOC(), pocLsb, "ph_pic_order_cnt_lsb");
  }
  int maxPocLsb = 1 << sps->getBitsForPOC();
  int pocMsb;
  if (pcSlice->getIdrPicFlag())
  {
    if (picHeader->getPocMsbPresentFlag())
    {
      pocMsb = picHeader->getPocMsbVal()*maxPocLsb;
    }
    else
    {
      pocMsb = 0;
    }
    pcSlice->setPOC(pocMsb + pocLsb);
  }
  else
  {
    int prevPoc = prevTid0POC;
    int prevPocLsb = prevPoc & (maxPocLsb - 1);
    int prevPocMsb = prevPoc - prevPocLsb;
    if (picHeader->getPocMsbPresentFlag())
    {
      pocMsb = picHeader->getPocMsbVal()*maxPocLsb;
    }
    else
    {
      if ((pocLsb < prevPocLsb) && ((prevPocLsb - pocLsb) >= (maxPocLsb / 2)))
      {
        pocMsb = prevPocMsb + maxPocLsb;
      }
      else if ((pocLsb > prevPocLsb) && ((pocLsb - prevPocLsb) > (maxPocLsb / 2)))
      {
        pocMsb = prevPocMsb - maxPocLsb;
      }
      else
      {
        pocMsb = prevPocMsb;
      }
    }
    pcSlice->setPOC(pocMsb + pocLsb);
  }
}

void HLSyntaxReader::parseConstraintInfo(ConstraintInfo *cinfo)
{
  uint32_t symbol;
#if !JVET_R0090_VUI
  READ_FLAG(symbol,  "general_progressive_source_flag"          ); cinfo->setProgressiveSourceFlag(symbol ? true : false);
  READ_FLAG(symbol,  "general_interlaced_source_flag"           ); cinfo->setInterlacedSourceFlag(symbol ? true : false);
#endif
  READ_FLAG(symbol,  "general_non_packed_constraint_flag"       ); cinfo->setNonPackedConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "general_frame_only_constraint_flag"       ); cinfo->setFrameOnlyConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "general_non_projected_constraint_flag"    ); cinfo->setNonProjectedConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "intra_only_constraint_flag"               ); cinfo->setIntraOnlyConstraintFlag(symbol ? true : false);

  READ_CODE(4, symbol,  "max_bitdepth_constraint_idc"              ); cinfo->setMaxBitDepthConstraintIdc(symbol);
  READ_CODE(2, symbol,  "max_chroma_format_constraint_idc"         ); cinfo->setMaxChromaFormatConstraintIdc((ChromaFormat)symbol);
#if JVET_R0286_GCI_CLEANUP
  READ_FLAG(symbol, "single_layer_constraint_flag");               cinfo->setSingleLayerConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol, "all_layers_independent_constraint_flag");     cinfo->setAllLayersIndependentConstraintFlag(symbol ? true : false);
  if (cinfo->getSingleLayerConstraintFlag())
  {
    CHECK(symbol == 0, "When single_layer_constraint_flag is equal to 1, the value of all_layers_independent_ constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol,  "no_res_change_in_clvs_constraint_flag"    ); cinfo->setNoResChangeInClvsConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "one_tile_per_pic_constraint_flag"         ); cinfo->setOneTilePerPicConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "one_slice_per_pic_constraint_flag"        ); cinfo->setOneSlicePerPicConstraintFlag(symbol ? true : false);
  READ_FLAG(symbol,  "one_subpic_per_pic_constraint_flag"       ); cinfo->setOneSubpicPerPicConstraintFlag(symbol ? true : false);
  if (cinfo->getOneSlicePerPicConstraintFlag())
  {
    CHECK(symbol == 0, "When one_slice_per_pic_constraint_flag is equal to 1, the value of one_subpic_per_pic_constraint_flag shall be equal to 1");
  }

  READ_FLAG(symbol,  "no_qtbtt_dual_tree_intra_constraint_flag" ); cinfo->setNoQtbttDualTreeIntraConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getMaxChromaFormatConstraintIdc() == 0)
  {
    CHECK(symbol == 0, "When max_chroma_format_constraint_idc is equal to 0, the value of no_qtbtt_dual_tree_intra_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol, "no_partition_constraints_override_constraint_flag"); cinfo->setNoPartitionConstraintsOverrideConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_sao_constraint_flag");                    cinfo->setNoSaoConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_alf_constraint_flag");                    cinfo->setNoAlfConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_ccalf_constraint_flag");                  cinfo->setNoCCAlfConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getMaxChromaFormatConstraintIdc() == 0)
  {
    CHECK(symbol == 0, "When max_chroma_format_constraint_idc is equal to 0, the value of no_ccalf_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol,  "no_joint_cbcr_constraint_flag");             cinfo->setNoJointCbCrConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getMaxChromaFormatConstraintIdc() == 0)
  {
    CHECK(symbol == 0, "When max_chroma_format_constraint_idc is equal to 0, the value of no_joint_cbcr_constraint_flag shall be equal to 1");
  }
  READ_FLAG(symbol, "no_mrl_constraint_flag");                     cinfo->setNoMrlConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_isp_constraint_flag");                     cinfo->setNoIspConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_mip_constraint_flag");                     cinfo->setNoMipConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol,  "no_ref_wraparound_constraint_flag");         cinfo->setNoRefWraparoundConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_ref_wraparound_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol,  "no_temporal_mvp_constraint_flag");           cinfo->setNoTemporalMvpConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_temporal_mvp_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol,  "no_sbtmvp_constraint_flag");                 cinfo->setNoSbtmvpConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_sbtmvp_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol,  "no_amvr_constraint_flag");                   cinfo->setNoAmvrConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_amvr_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol,  "no_bdof_constraint_flag");                   cinfo->setNoBdofConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_bdof_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol, "no_dmvr_constraint_flag");                    cinfo->setNoDmvrConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_dmvr_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol, "no_cclm_constraint_flag");                    cinfo->setNoCclmConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getMaxChromaFormatConstraintIdc() == 0)
  {
    CHECK(symbol == 0, "When max_chroma_format_constraint_idc is equal to 0, the value of no_cclm_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol, "no_mts_constraint_flag");                     cinfo->setNoMtsConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_sbt_constraint_flag");                     cinfo->setNoSbtConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  READ_FLAG(symbol, "no_lfnst_constraint_flag");                   cinfo->setNoLfnstConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_affine_motion_constraint_flag");           cinfo->setNoAffineMotionConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_affine_motion_constraint_flag shall be equal to 1");
  }
  READ_FLAG(symbol, "no_mmvd_constraint_flag");                    cinfo->setNoMmvdConstraintFlag(symbol > 0 ? true : false);
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_mmvd_constraint_flag shall be equal to 1");
  }
  READ_FLAG(symbol, "no_smvd_constraint_flag");                    cinfo->setNoSmvdConstraintFlag(symbol > 0 ? true : false);
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_smvd_constraint_flag shall be equal to 1");
  }
  READ_FLAG(symbol, "no_prof_constraint_flag");                    cinfo->setNoProfConstraintFlag(symbol > 0 ? true : false);
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_prof_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol, "no_bcw_constraint_flag");                     cinfo->setNoBcwConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_bcw_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol, "no_ibc_constraint_flag");                     cinfo->setNoIbcConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_ciip_constraint_flag");                    cinfo->setNoCiipConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_ciip_constraint_flag shall be equal to 1");
  }
#endif
#if !JVET_R0214_MMVD_SYNTAX_MODIFICATION
  READ_FLAG(symbol, "no_fpel_mmvd_constraint_flag");               cinfo->setNoFPelMmvdConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_gpm_constraint_flag");                     cinfo->setNoGeoConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  if (cinfo->getIntraOnlyConstraintFlag() == 1)
  {
    CHECK(symbol == 0, "When intra_only_constraint_flag is equal to 1, the value of no_gpm_constraint_flag shall be equal to 1");
  }
#endif
  READ_FLAG(symbol, "no_ladf_constraint_flag");                    cinfo->setNoLadfConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_transform_skip_constraint_flag");          cinfo->setNoTransformSkipConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_bdpcm_constraint_flag");                   cinfo->setNoBDPCMConstraintFlag(symbol > 0 ? true : false);
#if JVET_R0286_GCI_CLEANUP
  READ_FLAG(symbol, "no_palette_constraint_flag");                 cinfo->setNoPaletteConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_act_constraint_flag");                     cinfo->setNoActConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_lmcs_constraint_flag");                    cinfo->setNoLmcsConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_qp_delta_constraint_flag");                cinfo->setNoQpDeltaConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_dep_quant_constraint_flag");               cinfo->setNoDepQuantConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_sign_data_hiding_constraint_flag");        cinfo->setNoSignDataHidingConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_mixed_nalu_types_in_pic_constraint_flag"); cinfo->setNoMixedNaluTypesInPicConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_trail_constraint_flag");                   cinfo->setNoTrailConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_stsa_constraint_flag");                    cinfo->setNoStsaConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_rasl_constraint_flag");                    cinfo->setNoRaslConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_radl_constraint_flag");                    cinfo->setNoRadlConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_idr_constraint_flag");                     cinfo->setNoIdrConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_cra_constraint_flag");                     cinfo->setNoCraConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_gdr_constraint_flag");                     cinfo->setNoGdrConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_aps_constraint_flag");                     cinfo->setNoApsConstraintFlag(symbol > 0 ? true : false);
}


void HLSyntaxReader::parseProfileTierLevel(ProfileTierLevel *ptl, bool profileTierPresentFlag, int maxNumSubLayersMinus1)
{
  uint32_t symbol;
  if(profileTierPresentFlag)
  {
    READ_CODE(7 , symbol,   "general_profile_idc"              ); ptl->setProfileIdc  (Profile::Name(symbol));
    READ_FLAG(    symbol,   "general_tier_flag"                ); ptl->setTierFlag    (symbol ? Level::HIGH : Level::MAIN);
    parseConstraintInfo( ptl->getConstraintInfo() );
  }

  READ_CODE( 8, symbol, "general_level_idc" ); ptl->setLevelIdc( Level::Name( symbol ) );

  if(profileTierPresentFlag)
  {
    READ_CODE(8, symbol, "num_sub_profiles");
    uint8_t numSubProfiles = symbol;
    ptl->setNumSubProfile( numSubProfiles );
    for (int i = 0; i < numSubProfiles; i++)
    {
      READ_CODE(32, symbol, "general_sub_profile_idc[i]"); ptl->setSubProfileIdc(i, symbol);
    }
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
  CHECK( uiLog2WeightDenomLuma > 7, "The value of luma_log2_weight_denom shall be in the range of 0 to 7" );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    CHECK((iDeltaDenom + (int)uiLog2WeightDenomLuma)<0, "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
    CHECK((iDeltaDenom + (int)uiLog2WeightDenomLuma)>7, "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
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
    else
    {
      for ( int iRefIdx=0; iRefIdx<MAX_NUM_REF; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        wp[1].bPresentFlag = false;
        wp[2].bPresentFlag = false;
      }
    }
    for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
      if ( wp[COMPONENT_Y].bPresentFlag )
      {
        int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
        CHECK( iDeltaWeight < -128, "delta_luma_weight_lx shall be in the rage of -128 to 127" );
        CHECK( iDeltaWeight >  127, "delta_luma_weight_lx shall be in the rage of -128 to 127" );
        wp[COMPONENT_Y].iWeight = (iDeltaWeight + (1<<wp[COMPONENT_Y].uiLog2WeightDenom));
        READ_SVLC( wp[COMPONENT_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        const int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_LUMA))/2 : 128;
        CHECK ( wp[0].iOffset < -range , "luma_offset_lx shall be in the rage of -128 to 127");
        CHECK ( wp[0].iOffset >= range , "luma_offset_lx shall be in the rage of -128 to 127");
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
            CHECK( iDeltaWeight < -128, "delta_chroma_weight_lx shall be in the rage of -128 to 127" );
            CHECK( iDeltaWeight >  127, "delta_chroma_weight_lx shall be in the rage of -128 to 127" );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[j].uiLog2WeightDenom));

            int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            CHECK( iDeltaChroma <  -4*range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127" );
            CHECK( iDeltaChroma >=  4*range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127" );
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

void HLSyntaxReader::parsePredWeightTable(PicHeader *picHeader, const SPS *sps)
{
  WPScalingParam *   wp;
  const ChromaFormat chFmt                     = sps->getChromaFormatIdc();
  const int          numValidComp              = int(getNumberValidComponents(chFmt));
  const bool         chroma                    = (chFmt != CHROMA_400);
  uint32_t           log2WeightDenomLuma       = 0;
  uint32_t           log2WeightDenomChroma     = 0;
  uint32_t           totalSignalledWeightFlags = 0;

  int deltaDenom;
  READ_UVLC(log2WeightDenomLuma, "luma_log2_weight_denom");
  CHECK(log2WeightDenomLuma > 7, "The value of luma_log2_weight_denom shall be in the range of 0 to 7");
  if (chroma)
  {
    READ_SVLC(deltaDenom, "delta_chroma_log2_weight_denom");
    CHECK((deltaDenom + (int) log2WeightDenomLuma) < 0, "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
    CHECK((deltaDenom + (int) log2WeightDenomLuma) > 7, "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
    log2WeightDenomChroma = (uint32_t)(deltaDenom + log2WeightDenomLuma);
  }

  uint32_t numLxWeights;
  READ_UVLC(numLxWeights, "num_l0_weights");
  picHeader->setNumL0Weights(numLxWeights);
  picHeader->setNumL1Weights(0);

  bool moreSyntaxToBeParsed = true;
  for (int numRef = 0; numRef < NUM_REF_PIC_LIST_01 && moreSyntaxToBeParsed; numRef++)
  {
    RefPicList refPicList = (numRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      picHeader->getWpScaling(refPicList, refIdx, wp);

      wp[COMPONENT_Y].uiLog2WeightDenom = log2WeightDenomLuma;
      for (int j = 1; j < numValidComp; j++)
      {
        wp[j].uiLog2WeightDenom = log2WeightDenomChroma;
      }

      uint32_t uiCode;
      READ_FLAG(uiCode, numRef == 0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]");
      wp[COMPONENT_Y].bPresentFlag = (uiCode == 1);
      totalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if (chroma)
    {
      uint32_t uiCode;
      for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
      {
        picHeader->getWpScaling(refPicList, refIdx, wp);
        READ_FLAG(uiCode, numRef == 0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]");
        for (int j = 1; j < numValidComp; j++)
        {
          wp[j].bPresentFlag = (uiCode == 1);
        }
        totalSignalledWeightFlags += 2 * wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    else
    {
      for ( int refIdx=0; refIdx<MAX_NUM_REF; refIdx++ )
      {
        picHeader->getWpScaling(refPicList, refIdx, wp);
        wp[1].bPresentFlag = false;
        wp[2].bPresentFlag = false;
      }
    }
    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      picHeader->getWpScaling(refPicList, refIdx, wp);
      if (wp[COMPONENT_Y].bPresentFlag)
      {
        int deltaWeight;
        READ_SVLC(deltaWeight, numRef == 0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]");
        CHECK(deltaWeight < -128, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        CHECK(deltaWeight > 127, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        wp[COMPONENT_Y].iWeight = (deltaWeight + (1 << wp[COMPONENT_Y].uiLog2WeightDenom));
        READ_SVLC(wp[COMPONENT_Y].iOffset, numRef == 0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]");
        const int range = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1 << sps->getBitDepth(CHANNEL_TYPE_LUMA)) / 2 : 128;
        CHECK ( wp[0].iOffset < -range , "luma_offset_lx shall be in the rage of -128 to 127");
        CHECK ( wp[0].iOffset >= range , "luma_offset_lx shall be in the rage of -128 to 127");
      }
      else
      {
        wp[COMPONENT_Y].iWeight = (1 << wp[COMPONENT_Y].uiLog2WeightDenom);
        wp[COMPONENT_Y].iOffset = 0;
      }
      if (chroma)
      {
        if (wp[COMPONENT_Cb].bPresentFlag)
        {
          int range = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1 << sps->getBitDepth(CHANNEL_TYPE_CHROMA)) / 2 : 128;
          for (int j = 1; j < numValidComp; j++)
          {
            int deltaWeight;
            READ_SVLC(deltaWeight, numRef == 0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]");
            CHECK( deltaWeight < -128, "delta_chroma_weight_lx shall be in the rage of -128 to 127" );
            CHECK( deltaWeight >  127, "delta_chroma_weight_lx shall be in the rage of -128 to 127" );
            wp[j].iWeight = (deltaWeight + (1 << wp[j].uiLog2WeightDenom));

            int deltaChroma;
            READ_SVLC(deltaChroma, numRef == 0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]");
            CHECK( deltaChroma <  -4*range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127" );
            CHECK( deltaChroma >=  4*range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127" );
            int pred      = (range - ((range * wp[j].iWeight) >> (wp[j].uiLog2WeightDenom)));
            wp[j].iOffset = Clip3(-range, range - 1, (deltaChroma + pred));
          }
        }
        else
        {
          for (int j = 1; j < numValidComp; j++)
          {
            wp[j].iWeight = (1 << wp[j].uiLog2WeightDenom);
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for (int refIdx = numLxWeights; refIdx < MAX_NUM_REF; refIdx++)
    {
      picHeader->getWpScaling(refPicList, refIdx, wp);

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }

    if (numRef == 0)
    {
#if R0324_PH_SYNTAX_CONDITION_MODIFY
      if (picHeader->getRPL(1)->getNumRefEntries() > 0)
      {
        READ_UVLC(numLxWeights, "num_l1_weights");
      }
      else
      {
        numLxWeights = 0;
      }
#else
      READ_UVLC(numLxWeights, "num_l1_weights");
#endif
      moreSyntaxToBeParsed = (numLxWeights == 0) ? false : true;
      picHeader->setNumL1Weights(numLxWeights);
    }
  }
  CHECK(totalSignalledWeightFlags > 24, "Too many weight flag signalled");
}

/** decode quantization matrix
* \param scalingList quantization matrix information
*/
void HLSyntaxReader::parseScalingList(ScalingList* scalingList)
{
  uint32_t  code;
  bool scalingListCopyModeFlag;
  READ_FLAG(code, "scaling_matrix_for_lfnst_disabled_flag"); scalingList->setDisableScalingMatrixForLfnstBlks(code ? true : false);
  READ_FLAG(code, "scaling_list_chroma_present_flag");
  scalingList->setChromaScalingListPresentFlag(code ? true : false);
  for (int scalingListId = 0; scalingListId < 28; scalingListId++)
  {
  if(scalingList->getChromaScalingListPresentFlag()|| scalingList->isLumaScalingList(scalingListId))
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
  else
  {
#if JVET_R0055_HANDLING_NON_EXISTENT_QM
    scalingListCopyModeFlag = true;
    scalingList->setScalingListCopyModeFlag(scalingListId, scalingListCopyModeFlag);
    scalingList->setRefMatrixId(scalingListId, (uint32_t)((int)(scalingListId)));
    if (scalingListId >= SCALING_LIST_1D_START_16x16)
    {
      scalingList->setScalingListDC(scalingListId, 16);
    }
    scalingList->processRefMatrix(scalingListId, scalingList->getRefMatrixId(scalingListId));
#else
    scalingList->processDefaultMatrix(scalingListId);
#endif
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
      READ_UVLC( code, isChroma ? "alf_chroma_coeff_abs" : "alf_luma_coeff_abs" );
      coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] = code;
      if( coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] != 0 )
      {
        READ_FLAG( code, isChroma ? "alf_chroma_coeff_sign" : "alf_luma_coeff_sign" );
        coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] = ( code ) ? -coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] : coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ];
       }
      CHECK( isChroma &&
             ( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] > 127 || coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] < -128 )
             , "AlfCoeffC shall be in the range of -128 to 127, inclusive" );
    }
  }

  // Clipping values coding
  if ( alfParam.nonLinearFlag[isChroma] )
  {

    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {

      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        READ_CODE(2, code, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
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

