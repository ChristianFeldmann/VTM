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


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadCode (uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName)
#else
void VLCReader::xReadCode (uint32_t uiLength, uint32_t& ruiCode)
#endif
{
  CHECK( uiLength == 0, "Reading a code of lenght '0'" );
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

#if JVET_M0128
void HLSyntaxReader::copyRefPicList(SPS* sps, ReferencePictureList* source_rpl, ReferencePictureList* dest_rp)
{
  dest_rp->setNumberOfShorttermPictures(source_rpl->getNumberOfShorttermPictures());

  if (sps->getLongTermRefsPresent())
    dest_rp->setNumberOfLongtermPictures(dest_rp->getNumberOfLongtermPictures());
  else
    dest_rp->setNumberOfLongtermPictures(0);

  uint32_t numRefPic = dest_rp->getNumberOfShorttermPictures() + dest_rp->getNumberOfLongtermPictures();
  for (int ii = 0; ii < numRefPic; ii++)
    dest_rp->setRefPicIdentifier(ii, source_rpl->getRefPicIdentifier(ii), source_rpl->isRefPicLongterm(ii));
}

void HLSyntaxReader::parseRefPicList(SPS* sps, ReferencePictureList* rpl)
{
  uint32_t code;
  READ_UVLC(code, "num_ref_entries[ listIdx ][ rplsIdx ]");
  uint32_t numRefPic = code;
  uint32_t numStrp = 0;
  uint32_t numLtrp = 0;

  bool isLongTerm;
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;
  for (int ii = 0; ii < numRefPic; ii++)
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
      rpl->setRefPicIdentifier(ii, deltaValue, isLongTerm);
      numStrp++;
    }
    else
    {
      READ_CODE(sps->getBitsForPOC(), code, "poc_lsb_lt[listIdx][rplsIdx][i]");
      rpl->setRefPicIdentifier(ii, code, isLongTerm);
      numLtrp++;
    }
  }
  rpl->setNumberOfShorttermPictures(numStrp);
  rpl->setNumberOfLongtermPictures(numLtrp);
}
#else
void HLSyntaxReader::parseShortTermRefPicSet( SPS* sps, ReferencePictureSet* rps, int idx )
{
  uint32_t code;
  uint32_t interRPSPred;
  if (idx > 0)
  {
    READ_FLAG(interRPSPred, "inter_ref_pic_set_prediction_flag");  rps->setInterRPSPrediction(interRPSPred);
  }
  else
  {
    interRPSPred = false;
    rps->setInterRPSPrediction(false);
  }

  if (interRPSPred)
  {
    uint32_t bit;
    if(idx == sps->getRPSList()->getNumberOfReferencePictureSets())
    {
      READ_UVLC(code, "delta_idx_minus1" ); // delta index of the Reference Picture Set used for prediction minus 1
    }
    else
    {
      code = 0;
    }
    rps->setDeltaRIdxMinus1(code); // th we need that for proper transcoding
    CHECK(code > idx-1, "Code exceeds boundary"); // delta_idx_minus1 shall not be larger than idx-1, otherwise we will predict from a negative row position that does not exist. When idx equals 0 there is no legal value and interRPSPred must be zero. See J0185-r2
    int rIdx =  idx - 1 - code;
    CHECK(rIdx > idx-1 || rIdx < 0, "Invalid index"); // Made assert tighter; if rIdx = idx then prediction is done from itself. rIdx must belong to range 0, idx-1, inclusive, see J0185-r2
    ReferencePictureSet*   rpsRef = sps->getRPSList()->getReferencePictureSet(rIdx);
    int k = 0, k0 = 0, k1 = 0;
    READ_CODE(1, bit, "delta_rps_sign"); // delta_RPS_sign
    READ_UVLC(code, "abs_delta_rps_minus1");  // absolute delta RPS minus 1
    int deltaRPS = (1 - 2 * bit) * (code + 1); // delta_RPS

    rps->setDeltaRPS( deltaRPS ); // th we need that for proper transcoding

    for(int j=0 ; j <= rpsRef->getNumberOfPictures(); j++)
    {
      READ_CODE(1, bit, "used_by_curr_pic_flag" ); //first bit is "1" if Idc is 1
      int refIdc = bit;
      if (refIdc == 0)
      {
        READ_CODE(1, bit, "use_delta_flag" ); //second bit is "1" if Idc is 2, "0" otherwise.
        refIdc = bit<<1; //second bit is "1" if refIdc is 2, "0" if refIdc = 0.
      }
      if (refIdc == 1 || refIdc == 2)
      {
        int deltaPOC = deltaRPS + ((j < rpsRef->getNumberOfPictures())? rpsRef->getDeltaPOC(j) : 0);
        rps->setDeltaPOC(k, deltaPOC);
        rps->setUsed(k, (refIdc == 1));

        if (deltaPOC < 0)
        {
          k0++;
        }
        else
        {
          k1++;
        }
        k++;
      }
      rps->setRefIdc(j,refIdc);
    }
    rps->setNumRefIdc(rpsRef->getNumberOfPictures()+1);
    rps->setNumberOfPictures(k);
    rps->setNumberOfNegativePictures(k0);
    rps->setNumberOfPositivePictures(k1);
    rps->sortDeltaPOC();
  }
  else
  {
    READ_UVLC(code, "num_negative_pics");           rps->setNumberOfNegativePictures(code);
    READ_UVLC(code, "num_positive_pics");           rps->setNumberOfPositivePictures(code);
    int prev = 0;
    int poc;
    for(int j=0 ; j < rps->getNumberOfNegativePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s0_minus1");
      poc = prev-code-1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s0_flag");  rps->setUsed(j,code);
    }
    prev = 0;
    for(int j=rps->getNumberOfNegativePictures(); j < rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s1_minus1");
      poc = prev+code+1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s1_flag");  rps->setUsed(j,code);
    }
    rps->setNumberOfPictures(rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures());
  }

  rps->printDeltaPOC();
}
#endif

#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
void HLSyntaxReader::parsePPS( PPS* pcPPS, ParameterSetManager *parameterSetManager )
#else
void HLSyntaxReader::parsePPS( PPS* pcPPS )
#endif
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif
  uint32_t  uiCode;

  int   iCode;

  READ_UVLC( uiCode, "pps_pic_parameter_set_id");
  CHECK(uiCode > 63, "PPS id exceeds boundary (63)");
  pcPPS->setPPSId (uiCode);

  READ_UVLC( uiCode, "pps_seq_parameter_set_id");
  CHECK(uiCode > 15, "SPS id exceeds boundary (15)");
  pcPPS->setSPSId (uiCode);


  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_CODE(3, uiCode, "num_extra_slice_header_bits");                pcPPS->setNumExtraSliceHeaderBits(uiCode);


  READ_FLAG( uiCode,   "cabac_init_present_flag" );            pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC(uiCode, "num_ref_idx_l0_default_active_minus1");
  CHECK(uiCode > 14, "Invalid code read");
  pcPPS->setNumRefIdxL0DefaultActive(uiCode+1);

  READ_UVLC(uiCode, "num_ref_idx_l1_default_active_minus1");
  CHECK(uiCode > 14, "Invalid code read");
  pcPPS->setNumRefIdxL1DefaultActive(uiCode+1);

#if JVET_M0128
  READ_FLAG(uiCode, "rpl1_idx_present_flag");
  pcPPS->setRpl1IdxPresentFlag(uiCode);
#endif

  READ_SVLC(iCode, "init_qp_minus26" );                            pcPPS->setPicInitQPMinus26(iCode);
  READ_FLAG( uiCode, "constrained_intra_pred_flag" );              pcPPS->setConstrainedIntraPred( uiCode ? true : false );
  READ_FLAG( uiCode, "transform_skip_enabled_flag" );
  pcPPS->setUseTransformSkip ( uiCode ? true : false );

  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );            pcPPS->setUseDQP( uiCode ? true : false );
  if( pcPPS->getUseDQP() )
  {
    READ_UVLC( uiCode, "cu_qp_delta_subdiv" );
    pcPPS->setCuQpDeltaSubdiv( uiCode );
  }
  else
  {
    pcPPS->setCuQpDeltaSubdiv( 0 );
  }
  READ_SVLC( iCode, "pps_cb_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cb, iCode);
  CHECK( pcPPS->getQpOffset(COMPONENT_Cb) < -12, "Invalid Cb QP offset" );
  CHECK( pcPPS->getQpOffset(COMPONENT_Cb) >  12, "Invalid Cb QP offset" );

  READ_SVLC( iCode, "pps_cr_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cr, iCode);
  CHECK( pcPPS->getQpOffset(COMPONENT_Cr) < -12, "Invalid Cr QP offset" );
  CHECK( pcPPS->getQpOffset(COMPONENT_Cr) >  12, "Invalid Cr QP offset" );

#if JVET_N0054_JOINT_CHROMA
  READ_SVLC( iCode, "pps_cb_cr_qp_offset");
  pcPPS->setQpOffset(JOINT_CbCr, iCode);
  CHECK( pcPPS->getQpOffset(JOINT_CbCr) < -12, "Invalid CbCr QP offset" );
  CHECK( pcPPS->getQpOffset(JOINT_CbCr) >  12, "Invalid CbCr QP offset" );
#endif

  CHECK(MAX_NUM_COMPONENT>3, "Invalid maximal number of components");

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode==1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode==1 );

  READ_FLAG( uiCode, "transquant_bypass_enabled_flag");
  pcPPS->setTransquantBypassEnabledFlag(uiCode ? true : false);
  
#if JVET_N0857_TILES_BRICKS
  READ_FLAG( uiCode, "single_tile_in_pic_flag" );                 pcPPS->setSingleTileInPicFlag(uiCode == 1);

  if(!pcPPS->getSingleTileInPicFlag())
  {
    READ_FLAG ( uiCode, "uniform_tile_spacing_flag" );            pcPPS->setUniformTileSpacingFlag( uiCode == 1 );
    if (pcPPS->getUniformTileSpacingFlag())
    {
      READ_UVLC ( uiCode, "tile_cols_width_minus1" );               pcPPS->setTileColsWidthMinus1( uiCode );
      READ_UVLC ( uiCode, "tile_rows_height_minus1" );              pcPPS->setTileRowsHeightMinus1( uiCode );
    }
    else
    {
      READ_UVLC ( uiCode, "num_tile_columns_minus1" );                pcPPS->setNumTileColumnsMinus1( uiCode );
      READ_UVLC ( uiCode, "num_tile_rows_minus1" );                   pcPPS->setNumTileRowsMinus1( uiCode );

      const int tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
      const int tileRowsMinus1    = pcPPS->getNumTileRowsMinus1();
      CHECK( ((tileColumnsMinus1 + 1) * (tileColumnsMinus1 + 1)) < 2, "tile colums * rows must be > 1 when explicitly signalled.");

      if (tileColumnsMinus1 > 0)
      {
        std::vector<int> columnWidth(tileColumnsMinus1);
        for(int i = 0; i < tileColumnsMinus1; i++)
        {
          READ_UVLC( uiCode, "tile_column_width_minus1" );
          columnWidth[i] = uiCode+1;
        }
        pcPPS->setTileColumnWidth(columnWidth);
      }

      if (tileRowsMinus1 > 0)
      {
        std::vector<int> rowHeight (tileRowsMinus1);
        for(int i = 0; i < tileRowsMinus1; i++)
        {
          READ_UVLC( uiCode, "tile_row_height_minus1" );
          rowHeight[i] = uiCode + 1;
        }
        pcPPS->setTileRowHeight(rowHeight);
      }
      CHECK( ( tileColumnsMinus1 + tileRowsMinus1 ) == 0, "Invalid tile configuration" );
    }

    READ_FLAG( uiCode, "brick_splitting_present_flag" );                 pcPPS->setBrickSplittingPresentFlag(uiCode == 1);

    int numTilesInPic = pcPPS->getUniformTileSpacingFlag() ? 0 : (pcPPS->getNumTileColumnsMinus1() + 1) * (pcPPS->getNumTileRowsMinus1() + 1);
#if JVET_N0857_RECT_SLICES
    pcPPS->setNumTilesInPic(numTilesInPic);
#endif

    if (pcPPS->getBrickSplittingPresentFlag())
    {
      std::vector<bool> brickSplitFlag (numTilesInPic);
      std::vector<bool> uniformBrickSpacingFlag (numTilesInPic);
      std::vector<int>  brickHeightMinus1 (numTilesInPic);
      std::vector<int>  numBrickRowsMinus1 (numTilesInPic);
      std::vector<std::vector<int>>  brickRowHeightMinus1 (numTilesInPic);
      for( int i = 0; i < numTilesInPic; i++ )
      {
        READ_FLAG( uiCode, "brick_split_flag [i]" );
        brickSplitFlag[i] = (uiCode == 1);

        if( brickSplitFlag[i] )
        {
          READ_FLAG( uiCode, "uniform_brick_spacing_flag [i]" );
          uniformBrickSpacingFlag[i] = (uiCode == 1);
          if( uniformBrickSpacingFlag[i] )
          {
            READ_UVLC( uiCode, "brick_height_minus1" );
            brickHeightMinus1[i] = uiCode;
          }
          else
          {
            READ_UVLC( uiCode, "num_brick_rows_minus1 [i]" );
            numBrickRowsMinus1[i] = uiCode;
            for(int j = 0; j < numBrickRowsMinus1[i]; j++ )
            {
              brickRowHeightMinus1[i].resize(numBrickRowsMinus1[i]);
              READ_UVLC( uiCode, "brick_row_height_minus1 [i][j]" );
              brickRowHeightMinus1[i][j]=uiCode;
            }
          }
        }
      }
      pcPPS->setBrickSplitFlag(brickSplitFlag);
      pcPPS->setUniformBrickSpacingFlag(uniformBrickSpacingFlag);
      pcPPS->setBrickHeightMinus1(brickHeightMinus1);
      pcPPS->setNumBrickRowsMinus1(numBrickRowsMinus1);
      pcPPS->setBrickRowHeightMinus1(brickRowHeightMinus1);
    }
    READ_FLAG (uiCode, "single_brick_per_slice_flag" );         pcPPS->setSingleBrickPerSliceFlag(uiCode == 1);
    if (!pcPPS->getSingleBrickPerSliceFlag())
    {
      READ_FLAG( uiCode, "rect_slice_flag" );                  pcPPS->setRectSliceFlag(uiCode == 1);
    }
    else
    {
      pcPPS->setRectSliceFlag(true);
    }

    if(pcPPS->getRectSliceFlag() && !pcPPS->getSingleBrickPerSliceFlag())
    {
      READ_UVLC (uiCode, "num_slices_in_pic_minus1" );          pcPPS->setNumSlicesInPicMinus1(uiCode);
      const uint32_t tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
      const uint32_t tileRowsMinus1 = pcPPS->getNumTileRowsMinus1();
      const uint32_t numSlicesInPic = pcPPS->getNumSlicesInPicMinus1() + 1;
      const uint32_t numTilesInPic = (tileColumnsMinus1 + 1) * (tileRowsMinus1 + 1);
      int codeLength = (int)ceil(log2(numTilesInPic));
#if JVET_N0124_PROPOSAL2
      int codeLength2 = codeLength;
#endif
      if (numSlicesInPic > 0)
      {
        std::vector<int> topLeft(numSlicesInPic);
        std::vector<int> bottomRight(numSlicesInPic);
        topLeft[0] = 0;
        for (uint32_t i = 0; i < numSlicesInPic; i++)
        {
          if (i > 0)
          {
            READ_CODE( codeLength, uiCode, "top_left_brick_idx" );
            topLeft[i] = uiCode;
#if JVET_N0124_PROPOSAL2
#if JVET_N0857_RECT_SLICES
            codeLength2 = (int)ceil(log2((numTilesInPic - topLeft[i] < 2) ? 2 : numTilesInPic - topLeft[i]));  //Bugfix
#else
            codeLength2 = (int)ceil(log2(numTilesInPic - topLeft[i]));
#endif
#endif
          }
#if JVET_N0124_PROPOSAL2
          READ_CODE( codeLength2, uiCode, "bottom_right_brick_idx_delta");
#else
          READ_CODE(codeLength, uiCode, "bottom_right_brick_idx_delta");
#endif
          bottomRight[i] = topLeft[i] + uiCode;
        }
#if JVET_N0857_RECT_SLICES
        pcPPS->setTopLeftBrickIdx(topLeft);
        pcPPS->setBottomRightBrickIdx(bottomRight);
#else
        pcPPS->setTopLeftTileIdx(topLeft);
        pcPPS->setBottomRightTileIdx(bottomRight);
#endif
      }
    }
#if JVET_N0857_RECT_SLICES
    if (pcPPS->getRectSliceFlag() && pcPPS->getSingleBrickPerSliceFlag())
    {
      std::vector<int> topLeft(numTilesInPic);  //TODO: this should be numBricksInPic. Fix it when the bricks codes have been updated
      std::vector<int> bottomRight(numTilesInPic);
      for (uint32_t i = 0; i < numTilesInPic; i++)
      {
        topLeft[i] = i;
        bottomRight[i] = i;
      }
      pcPPS->setTopLeftBrickIdx(topLeft);
      pcPPS->setBottomRightBrickIdx(bottomRight);
    }
#endif

    READ_FLAG( uiCode, "loop_filter_across_bricks_enabled_flag ");        pcPPS->setLoopFilterAcrossBricksEnabledFlag(uiCode ? true : false);
    if (pcPPS->getLoopFilterAcrossBricksEnabledFlag())
    {
      READ_FLAG( uiCode, "loop_filter_across_slices_enabled_flag" );      pcPPS->setLoopFilterAcrossSlicesEnabledFlag(uiCode == 1);
    }
  }
  else
  {
#if JVET_N0124_PROPOSAL1
    pcPPS->setSingleBrickPerSliceFlag(true);
#endif
    pcPPS->setRectSliceFlag(true);
#if JVET_N0857_RECT_SLICES
    std::vector<int> topLeft(1);
    topLeft[0] = 0;
    std::vector<int> bottomRight(1);
    bottomRight[0] = 0;
    pcPPS->setTopLeftBrickIdx(topLeft);
    pcPPS->setBottomRightBrickIdx(bottomRight);
#endif
  }

  if (pcPPS->getRectSliceFlag())
  {
    READ_FLAG( uiCode, "signalled_slice_id_flag ");                        pcPPS->setSignalledSliceIdFlag(uiCode == 1);
    if (pcPPS->getSignalledSliceIdFlag())
    {
      READ_UVLC( uiCode, "signalled_slice_id_length_minus1" );             pcPPS->setSignalledSliceIdLengthMinus1(uiCode);
#if JVET_N0857_RECT_SLICES
      const uint32_t numSlices = pcPPS->getNumSlicesInPicMinus1() + 1;
      int codeLength = pcPPS->getSignalledSliceIdLengthMinus1() + 1;
      if (numSlices > 0)
      {
        std::vector<int> sliceID(numSlices);
        for (uint32_t i = 0; i < numSlices; i++)
        {
          READ_CODE(codeLength, uiCode, "slice_id");
          sliceID[i] = uiCode;
        }
        pcPPS->setSliceId(sliceID);
      }
#else
      const uint32_t numTileGroups = pcPPS->getNumSlicesInPicMinus1() + 1;
      int codeLength = pcPPS->getSignalledSliceIdLengthMinus1() + 1;
      if (numTileGroups > 0)
      {
        std::vector<int> tileGroupID(numTileGroups);
        for (uint32_t i = 0; i < numTileGroups; i++)
        {
          READ_CODE( codeLength, uiCode, "slice_id" );
          tileGroupID[i] = uiCode;
        }
        pcPPS->setSliceId(tileGroupID);
      }
#endif
    }
#if JVET_N0857_RECT_SLICES
    else
    {
      std::vector<int> sliceID(pcPPS->getNumSlicesInPicMinus1() + 1);
      for (uint32_t i = 0; i <= pcPPS->getNumSlicesInPicMinus1(); i++)
      {
        sliceID[i] = i;
      }
      pcPPS->setSliceId(sliceID);
    }
#endif
  }
#endif

#if !JVET_N0857_TILES_BRICKS
  READ_FLAG(uiCode, "tiles_enabled_flag");                       pcPPS->setTilesEnabledFlag(uiCode == 1);

  if (pcPPS->getTilesEnabledFlag())
  {
    READ_UVLC(uiCode, "num_tile_columns_minus1");                pcPPS->setNumTileColumnsMinus1(uiCode);
    READ_UVLC(uiCode, "num_tile_rows_minus1");                   pcPPS->setNumTileRowsMinus1(uiCode);
    READ_FLAG(uiCode, "uniform_spacing_flag");                   pcPPS->setUniformTileSpacingFlag(uiCode == 1);

    const uint32_t tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
    const uint32_t tileRowsMinus1 = pcPPS->getNumTileRowsMinus1();

    if (!pcPPS->getUniformTileSpacingFlag())
    {
      if (tileColumnsMinus1 > 0)
      {
        std::vector<int> columnWidth(tileColumnsMinus1);
        for (uint32_t i = 0; i < tileColumnsMinus1; i++)
        {
          READ_UVLC(uiCode, "column_width_minus1");
          columnWidth[i] = uiCode + 1;
        }
        pcPPS->setTileColumnWidth(columnWidth);
      }

      if (tileRowsMinus1 > 0)
      {
        std::vector<int> rowHeight(tileRowsMinus1);
        for (uint32_t i = 0; i < tileRowsMinus1; i++)
        {
          READ_UVLC(uiCode, "row_height_minus1");
          rowHeight[i] = uiCode + 1;
        }
        pcPPS->setTileRowHeight(rowHeight);
      }
    }
    CHECK((tileColumnsMinus1 + tileRowsMinus1) == 0, "Invalid tile configuration");
    READ_FLAG ( uiCode, "loop_filter_across_tiles_enabled_flag" );     pcPPS->setLoopFilterAcrossBricksEnabledFlag( uiCode ? true : false );
  }
 
  READ_FLAG( uiCode, "pps_loop_filter_across_slices_enabled_flag" );   pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode ? true : false );
#endif
  READ_FLAG(uiCode, "entropy_coding_sync_enabled_flag");         pcPPS->setEntropyCodingSyncEnabledFlag(uiCode == 1);

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

#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  READ_FLAG( uiCode, "pps_loop_filter_across_virtual_boundaries_disabled_flag" ); pcPPS->setLoopFilterAcrossVirtualBoundariesDisabledFlag( uiCode != 0 );
  if( pcPPS->getLoopFilterAcrossVirtualBoundariesDisabledFlag() )
  {
    READ_CODE( 2, uiCode, "pps_num_ver_virtual_boundaries");        pcPPS->setNumVerVirtualBoundaries( uiCode );
    uint32_t picWidth = parameterSetManager->getSPS( pcPPS->getSPSId() )->getPicWidthInLumaSamples(); // pcPPS->getPicWidthInLumaSamples();
    int numBits = (int)ceil(log2(picWidth) - 3);
    for( unsigned i = 0; i < pcPPS->getNumVerVirtualBoundaries(); i++ )
    {
      READ_CODE( numBits, uiCode, "pps_virtual_boundaries_pos_x" ); pcPPS->setVirtualBoundariesPosX( uiCode << 3, i );
    }
    READ_CODE( 2, uiCode, "pps_num_hor_virtual_boundaries");        pcPPS->setNumHorVirtualBoundaries( uiCode );
    uint32_t picHeight = parameterSetManager->getSPS( pcPPS->getSPSId() )->getPicHeightInLumaSamples(); // pcPPS->getPicHeightInLumaSamples();
    numBits = (int)ceil(log2(picHeight) - 3);
    for( unsigned i = 0; i < pcPPS->getNumHorVirtualBoundaries(); i++ )
    {
      READ_CODE( numBits, uiCode, "pps_virtual_boundaries_pos_y" ); pcPPS->setVirtualBoundariesPosY( uiCode << 3, i );
    }
  }
#endif

#if HEVC_USE_SCALING_LISTS
  READ_FLAG( uiCode, "pps_scaling_list_data_present_flag" );           pcPPS->setScalingListPresentFlag( uiCode ? true : false );
  if(pcPPS->getScalingListPresentFlag ())
  {
    parseScalingList( &(pcPPS->getScalingList()) );
  }
#endif

#if !JVET_M0128
  READ_FLAG( uiCode, "lists_modification_present_flag");
  pcPPS->setListsModificationPresentFlag(uiCode);
#endif

  READ_UVLC( uiCode, "log2_parallel_merge_level_minus2");
  pcPPS->setLog2ParallelMergeLevelMinus2 (uiCode);

  READ_FLAG( uiCode, "slice_segment_header_extension_present_flag");
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

          if (pcPPS->getUseTransformSkip())
          {
            READ_UVLC( uiCode, "log2_max_transform_skip_block_size_minus2");
            ppsRangeExtension.setLog2MaxTransformSkipBlockSize(uiCode+2);
          }

          READ_FLAG( uiCode, "cross_component_prediction_enabled_flag");
          ppsRangeExtension.setCrossComponentPredictionEnabledFlag(uiCode != 0);

          READ_FLAG( uiCode, "chroma_qp_offset_list_enabled_flag");
          if (uiCode == 0)
          {
            ppsRangeExtension.clearChromaQpOffsetList();
            ppsRangeExtension.setCuChromaQpOffsetSubdiv(0);
          }
          else
          {
            READ_UVLC(uiCode, "cu_chroma_qp_offset_subdiv"); ppsRangeExtension.setCuChromaQpOffsetSubdiv(uiCode);
            uint32_t tableSizeMinus1 = 0;
            READ_UVLC(tableSizeMinus1, "chroma_qp_offset_list_len_minus1");
            CHECK(tableSizeMinus1 >= MAX_QP_OFFSET_LIST_SIZE, "Table size exceeds maximum");

            for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
            {
              int cbOffset;
              int crOffset;
              READ_SVLC(cbOffset, "cb_qp_offset_list[i]");
              CHECK(cbOffset < -12 || cbOffset > 12, "Invalid chroma QP offset");
              READ_SVLC(crOffset, "cr_qp_offset_list[i]");
              CHECK(crOffset < -12 || crOffset > 12, "Invalid chroma QP offset");
              // table uses +1 for index (see comment inside the function)
              ppsRangeExtension.setChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1, cbOffset, crOffset);
            }
            CHECK(ppsRangeExtension.getChromaQpOffsetListLen() != tableSizeMinus1 + 1, "Invalid chroma QP offset list lenght");
          }

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

 #if JVET_N0805_APS_LMCS
  READ_CODE(3, code, "aps_params_type");
  aps->setAPSType(code);
  if (code == ALF_APS)
  {
    parseAlfAps(aps);
  }
  else if (code == LMCS_APS)
  {
    parseLmcsAps(aps);
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
#else
  AlfSliceParam param = aps->getAlfAPSParam();
#if JVET_N0415_CTB_ALF
  param.enabledFlag[COMPONENT_Y] = param.enabledFlag[COMPONENT_Cb] = param.enabledFlag[COMPONENT_Cr] = true;
  READ_FLAG(code, "alf_luma_new_filter");
  param.newFilterFlag[CHANNEL_TYPE_LUMA] = code;
  READ_FLAG(code, "alf_chroma_new_filter");
  param.newFilterFlag[CHANNEL_TYPE_CHROMA] = code;
#else
  param.enabledFlag[COMPONENT_Y] = true;

  int alfChromaIdc = truncatedUnaryEqProb(3);        //alf_chroma_idc
  param.enabledFlag[COMPONENT_Cb] = alfChromaIdc >> 1;
  param.enabledFlag[COMPONENT_Cr] = alfChromaIdc & 1;
#endif

#if JVET_N0242_NON_LINEAR_ALF && !JVET_N0415_CTB_ALF
  READ_FLAG( code, "alf_luma_clip" );
  param.nonLinearFlag[CHANNEL_TYPE_LUMA] = code ? true : false;

  if( alfChromaIdc )
  {
    READ_FLAG( code, "alf_chroma_clip" );
    param.nonLinearFlag[CHANNEL_TYPE_CHROMA] = code ? true : false;
  }
#endif

#if JVET_N0415_CTB_ALF
  if (param.newFilterFlag[CHANNEL_TYPE_LUMA])
  {
#if JVET_N0242_NON_LINEAR_ALF
    READ_FLAG(code, "alf_luma_clip");
    param.nonLinearFlag[CHANNEL_TYPE_LUMA] = code ? true : false;
#endif
#endif
  xReadTruncBinCode(code, MAX_NUM_ALF_CLASSES);  //number_of_filters_minus1
  param.numLumaFilters = code + 1;
  if (param.numLumaFilters > 1)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
      xReadTruncBinCode(code, param.numLumaFilters);
      param.filterCoeffDeltaIdx[i] = code;
    }
  }
  else
  {
    memset(param.filterCoeffDeltaIdx, 0, sizeof(param.filterCoeffDeltaIdx));
  }

#if JVET_N0415_CTB_ALF
  READ_FLAG(code, "fixed_filter_set_flag");
  param.fixedFilterSetIndex = code;
  if (param.fixedFilterSetIndex > 0)
  {
    xReadTruncBinCode(code, NUM_FIXED_FILTER_SETS);
    param.fixedFilterSetIndex = code + 1;
    READ_FLAG(code, "fixed_filter_flag_pattern");
    param.fixedFilterPattern = code;
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      code = 1;
      if (param.fixedFilterPattern > 0)
      {
        READ_FLAG(code, "fixed_filter_flag");
      }
      param.fixedFilterIdx[classIdx] = code;
    }
  }
#endif
  alfFilter(param, false);
#if JVET_N0415_CTB_ALF
  }
  if (param.newFilterFlag[CHANNEL_TYPE_CHROMA])
  {
#if JVET_N0242_NON_LINEAR_ALF
    READ_FLAG(code, "alf_luma_clip");
    param.nonLinearFlag[CHANNEL_TYPE_CHROMA] = code ? true : false;
#endif
#else
  if (alfChromaIdc)
  {
#endif
    alfFilter(param, true);
  }
  aps->setAlfAPSParam(param);

  xReadRbspTrailingBits();
#endif
}

#if JVET_N0805_APS_LMCS
void HLSyntaxReader::parseAlfAps( APS* aps )
{
  uint32_t  code;

  AlfSliceParam param = aps->getAlfAPSParam();
#if JVET_N0415_CTB_ALF
  param.enabledFlag[COMPONENT_Y] = param.enabledFlag[COMPONENT_Cb] = param.enabledFlag[COMPONENT_Cr] = true;
  READ_FLAG(code, "alf_luma_new_filter");
  param.newFilterFlag[CHANNEL_TYPE_LUMA] = code;
  READ_FLAG(code, "alf_chroma_new_filter");
  param.newFilterFlag[CHANNEL_TYPE_CHROMA] = code;
#else
  param.enabledFlag[COMPONENT_Y] = true;

  int alfChromaIdc = truncatedUnaryEqProb(3);        //alf_chroma_idc
  param.enabledFlag[COMPONENT_Cb] = alfChromaIdc >> 1;
  param.enabledFlag[COMPONENT_Cr] = alfChromaIdc & 1;
#endif

#if JVET_N0242_NON_LINEAR_ALF && !JVET_N0415_CTB_ALF
  READ_FLAG(code, "alf_luma_clip");
  param.nonLinearFlag[CHANNEL_TYPE_LUMA] = code ? true : false;

  if (alfChromaIdc)
  {
    READ_FLAG(code, "alf_chroma_clip");
    param.nonLinearFlag[CHANNEL_TYPE_CHROMA] = code ? true : false;
  }
#endif

#if JVET_N0415_CTB_ALF
  if (param.newFilterFlag[CHANNEL_TYPE_LUMA])
  {
#if JVET_N0242_NON_LINEAR_ALF
    READ_FLAG(code, "alf_luma_clip");
    param.nonLinearFlag[CHANNEL_TYPE_LUMA] = code ? true : false;
#endif
#endif
    xReadTruncBinCode(code, MAX_NUM_ALF_CLASSES);  //number_of_filters_minus1
    param.numLumaFilters = code + 1;
    if (param.numLumaFilters > 1)
    {
      for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
      {
        xReadTruncBinCode(code, param.numLumaFilters);
        param.filterCoeffDeltaIdx[i] = code;
      }
    }
    else
    {
      memset(param.filterCoeffDeltaIdx, 0, sizeof(param.filterCoeffDeltaIdx));
    }

#if JVET_N0415_CTB_ALF
    READ_FLAG(code, "fixed_filter_set_flag");
    param.fixedFilterSetIndex = code;
    if (param.fixedFilterSetIndex > 0)
    {
      xReadTruncBinCode(code, NUM_FIXED_FILTER_SETS);
      param.fixedFilterSetIndex = code + 1;
      READ_FLAG(code, "fixed_filter_flag_pattern");
      param.fixedFilterPattern = code;
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
        code = 1;
        if (param.fixedFilterPattern > 0)
        {
          READ_FLAG(code, "fixed_filter_flag");
        }
        param.fixedFilterIdx[classIdx] = code;
      }
    }
#endif
    alfFilter(param, false);
#if JVET_N0415_CTB_ALF
  }
  if (param.newFilterFlag[CHANNEL_TYPE_CHROMA])
  {
#if JVET_N0242_NON_LINEAR_ALF
    READ_FLAG(code, "alf_luma_clip");
    param.nonLinearFlag[CHANNEL_TYPE_CHROMA] = code ? true : false;
#endif
#else
    if (alfChromaIdc)
    {
#endif
      alfFilter(param, true);
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
  aps->setReshaperAPSInfo(info);
}
#endif


void  HLSyntaxReader::parseVUI(VUI* pcVUI, SPS *pcSPS)
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif

#if JVET_N0063_VUI

  uint32_t  symbol;

  READ_FLAG( symbol, "aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(symbol);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
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

  READ_FLAG(     symbol, "video_signal_type_present_flag");           pcVUI->setVideoSignalTypePresentFlag(symbol);
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    READ_FLAG(   symbol, "video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(symbol);
  }

#else

  uint32_t  uiCode;

  READ_FLAG(     uiCode, "aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(uiCode);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    READ_CODE(8, uiCode, "aspect_ratio_idc");                         pcVUI->setAspectRatioIdc(uiCode);
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      READ_CODE(16, uiCode, "sar_width");                             pcVUI->setSarWidth(uiCode);
      READ_CODE(16, uiCode, "sar_height");                            pcVUI->setSarHeight(uiCode);
    }
  }

  READ_FLAG(     uiCode, "overscan_info_present_flag");               pcVUI->setOverscanInfoPresentFlag(uiCode);
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    READ_FLAG(   uiCode, "overscan_appropriate_flag");                pcVUI->setOverscanAppropriateFlag(uiCode);
  }

  READ_FLAG(     uiCode, "video_signal_type_present_flag");           pcVUI->setVideoSignalTypePresentFlag(uiCode);
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    READ_CODE(3, uiCode, "video_format");                             pcVUI->setVideoFormat(uiCode);
    READ_FLAG(   uiCode, "video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(uiCode);
    READ_FLAG(   uiCode, "colour_description_present_flag");          pcVUI->setColourDescriptionPresentFlag(uiCode);
    if (pcVUI->getColourDescriptionPresentFlag())
    {
      READ_CODE(8, uiCode, "colour_primaries");                       pcVUI->setColourPrimaries(uiCode);
      READ_CODE(8, uiCode, "transfer_characteristics");               pcVUI->setTransferCharacteristics(uiCode);
      READ_CODE(8, uiCode, "matrix_coeffs");                          pcVUI->setMatrixCoefficients(uiCode);
    }
  }

  READ_FLAG(     uiCode, "chroma_loc_info_present_flag");             pcVUI->setChromaLocInfoPresentFlag(uiCode);
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    READ_UVLC(   uiCode, "chroma_sample_loc_type_top_field" );        pcVUI->setChromaSampleLocTypeTopField(uiCode);
    READ_UVLC(   uiCode, "chroma_sample_loc_type_bottom_field" );     pcVUI->setChromaSampleLocTypeBottomField(uiCode);
  }

  READ_FLAG(     uiCode, "neutral_chroma_indication_flag");           pcVUI->setNeutralChromaIndicationFlag(uiCode);

  READ_FLAG(     uiCode, "field_seq_flag");                           pcVUI->setFieldSeqFlag(uiCode);

  READ_FLAG(uiCode, "frame_field_info_present_flag");                 pcVUI->setFrameFieldInfoPresentFlag(uiCode);

  READ_FLAG(     uiCode, "default_display_window_flag");
  if (uiCode != 0)
  {
    Window &defDisp = pcVUI->getDefaultDisplayWindow();
    READ_UVLC(   uiCode, "def_disp_win_left_offset" );                defDisp.setWindowLeftOffset  ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_right_offset" );               defDisp.setWindowRightOffset ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_top_offset" );                 defDisp.setWindowTopOffset   ( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_bottom_offset" );              defDisp.setWindowBottomOffset( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
  }

  TimingInfo *timingInfo = pcVUI->getTimingInfo();
  READ_FLAG(       uiCode, "vui_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vui_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vui_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vui_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vui_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_FLAG(     uiCode, "vui_hrd_parameters_present_flag");        pcVUI->setHrdParametersPresentFlag(uiCode);
    if( pcVUI->getHrdParametersPresentFlag() )
    {
      parseHrdParameters( pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }

  READ_FLAG(     uiCode, "bitstream_restriction_flag");               pcVUI->setBitstreamRestrictionFlag(uiCode);
  if (pcVUI->getBitstreamRestrictionFlag())
  {
    READ_FLAG(   uiCode, "tiles_fixed_structure_flag");               pcVUI->setTilesFixedStructureFlag(uiCode);
    READ_FLAG(   uiCode, "motion_vectors_over_pic_boundaries_flag");  pcVUI->setMotionVectorsOverPicBoundariesFlag(uiCode);
    READ_FLAG(   uiCode, "restricted_ref_pic_lists_flag");            pcVUI->setRestrictedRefPicListsFlag(uiCode);
    READ_UVLC(   uiCode, "min_spatial_segmentation_idc");             pcVUI->setMinSpatialSegmentationIdc(uiCode);
    CHECK(uiCode >= 4096, "Invalid code signalled");
    READ_UVLC(   uiCode, "max_bytes_per_pic_denom" );                 pcVUI->setMaxBytesPerPicDenom(uiCode);
    READ_UVLC(   uiCode, "max_bits_per_min_cu_denom" );               pcVUI->setMaxBitsPerMinCuDenom(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_horizontal" );           pcVUI->setLog2MaxMvLengthHorizontal(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_vertical" );             pcVUI->setLog2MaxMvLengthVertical(uiCode);
  }

#endif
}

void HLSyntaxReader::parseHrdParameters(HRDParameters *hrd, bool commonInfPresentFlag, uint32_t maxNumSubLayersMinus1)
{
  uint32_t  uiCode;
  if( commonInfPresentFlag )
  {
    READ_FLAG( uiCode, "nal_hrd_parameters_present_flag" );           hrd->setNalHrdParametersPresentFlag( uiCode == 1 ? true : false );
    READ_FLAG( uiCode, "vcl_hrd_parameters_present_flag" );           hrd->setVclHrdParametersPresentFlag( uiCode == 1 ? true : false );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      READ_FLAG( uiCode, "sub_pic_hrd_params_present_flag" );         hrd->setSubPicCpbParamsPresentFlag( uiCode == 1 ? true : false );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 8, uiCode, "tick_divisor_minus2" );                hrd->setTickDivisorMinus2( uiCode );
        READ_CODE( 5, uiCode, "du_cpb_removal_delay_increment_length_minus1" ); hrd->setDuCpbRemovalDelayLengthMinus1( uiCode );
        READ_FLAG( uiCode, "sub_pic_cpb_params_in_pic_timing_sei_flag" ); hrd->setSubPicCpbParamsInPicTimingSEIFlag( uiCode == 1 ? true : false );
        READ_CODE( 5, uiCode, "dpb_output_delay_du_length_minus1"  ); hrd->setDpbOutputDelayDuLengthMinus1( uiCode );
      }
      READ_CODE( 4, uiCode, "bit_rate_scale" );                       hrd->setBitRateScale( uiCode );
      READ_CODE( 4, uiCode, "cpb_size_scale" );                       hrd->setCpbSizeScale( uiCode );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 4, uiCode, "cpb_size_du_scale" );                  hrd->setDuCpbSizeScale( uiCode );
      }
      READ_CODE( 5, uiCode, "initial_cpb_removal_delay_length_minus1" ); hrd->setInitialCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "au_cpb_removal_delay_length_minus1" );      hrd->setCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "dpb_output_delay_length_minus1" );       hrd->setDpbOutputDelayLengthMinus1( uiCode );
    }
  }
  int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
  {
    READ_FLAG( uiCode, "fixed_pic_rate_general_flag" );                     hrd->setFixedPicRateFlag( i, uiCode == 1 ? true : false  );
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      READ_FLAG( uiCode, "fixed_pic_rate_within_cvs_flag" );                hrd->setFixedPicRateWithinCvsFlag( i, uiCode == 1 ? true : false  );
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag( i, true );
    }

    hrd->setLowDelayHrdFlag( i, 0 ); // Infered to be 0 when not present
    hrd->setCpbCntMinus1   ( i, 0 ); // Infered to be 0 when not present

    if( hrd->getFixedPicRateWithinCvsFlag( i ) )
    {
      READ_UVLC( uiCode, "elemental_duration_in_tc_minus1" );             hrd->setPicDurationInTcMinus1( i, uiCode );
    }
    else
    {
      READ_FLAG( uiCode, "low_delay_hrd_flag" );                      hrd->setLowDelayHrdFlag( i, uiCode == 1 ? true : false  );
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      READ_UVLC( uiCode, "cpb_cnt_minus1" );                          hrd->setCpbCntMinus1( i, uiCode );
    }

    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          READ_UVLC( uiCode, "bit_rate_value_minus1" );             hrd->setBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          READ_UVLC( uiCode, "cpb_size_value_minus1" );             hrd->setCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            READ_UVLC( uiCode, "cpb_size_du_value_minus1" );        hrd->setDuCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
            READ_UVLC( uiCode, "bit_rate_du_value_minus1" );        hrd->setDuBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          }
          READ_FLAG( uiCode, "cbr_flag" );                          hrd->setCbrFlag( i, j, nalOrVcl, uiCode == 1 ? true : false  );
        }
      }
    }
  }
}

#if !JVET_N0805_APS_LMCS
void HLSyntaxReader::parseReshaper(SliceReshapeInfo& info, const SPS* pcSPS, const bool isIntra)
{
  unsigned  symbol = 0;
  READ_FLAG(symbol, "tile_group_reshaper_model_present_flag");                 info.setSliceReshapeModelPresentFlag(symbol == 1);
  if (info.getSliceReshapeModelPresentFlag())
  {
    memset(info.reshaperModelBinCWDelta, 0, PIC_CODE_CW_BINS * sizeof(int));
    READ_UVLC(symbol, "reshaper_model_min_bin_idx");                             info.reshaperModelMinBinIdx = symbol;
    READ_UVLC(symbol, "reshaper_model_delta_max_bin_idx");                       info.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1 - symbol;
    READ_UVLC(symbol, "reshaper_model_bin_delta_abs_cw_prec_minus1");            info.maxNbitsNeededDeltaCW = symbol + 1;
    assert(info.maxNbitsNeededDeltaCW > 0);
    for (uint32_t i = info.reshaperModelMinBinIdx; i <= info.reshaperModelMaxBinIdx; i++)
    {
      READ_CODE(info.maxNbitsNeededDeltaCW, symbol, "reshaper_model_bin_delta_abs_CW");
      int absCW = symbol;
      if (absCW > 0)
      {
        READ_CODE(1, symbol, "reshaper_model_bin_delta_sign_CW_flag");
      }
      int signCW = symbol;
      info.reshaperModelBinCWDelta[i] = (1 - 2 * signCW) * absCW;
    }
  }
  READ_FLAG(symbol, "tile_group_reshaper_enable_flag");           info.setUseSliceReshaper(symbol == 1);
  if (info.getUseSliceReshaper())
  {
    if (!(pcSPS->getUseDualITree() && isIntra))
    {
      READ_FLAG(symbol, "slice_reshaper_ChromaAdj");                info.setSliceReshapeChromaAdj(symbol);
    }
    else
    {
      info.setSliceReshapeChromaAdj(0);
    }
  }
}
#endif

void HLSyntaxReader::parseSPS(SPS* pcSPS)
{
  uint32_t  uiCode;

#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif
#if JVET_N0349_DPS
  READ_CODE( 4,  uiCode, "sps_decoding_parameter_set_id");       pcSPS->setDecodingParameterSetId( uiCode );
#endif
#if HEVC_VPS
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id");          pcSPS->setVPSId        ( uiCode );
#endif
#if !JVET_M0101_HLS
  READ_UVLC(     uiCode, "sps_seq_parameter_set_id" );           pcSPS->setSPSId( uiCode );
  CHECK(uiCode > 15, "Invalid SPS id signalled");

  READ_FLAG(uiCode, "intra_only_constraint_flag");               pcSPS->setIntraOnlyConstraintFlag(uiCode > 0 ? true : false);
  READ_CODE(4, uiCode, "max_bitdepth_constraint_idc");           pcSPS->setMaxBitDepthConstraintIdc(uiCode);
  READ_CODE(2, uiCode, "max_chroma_format_constraint_idc");      pcSPS->setMaxChromaFormatConstraintIdc(uiCode);
  READ_FLAG(uiCode, "frame_only_constraint_flag");               pcSPS->setFrameConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_qtbtt_dual_tree_intra_constraint_flag"); pcSPS->setNoQtbttDualTreeIntraConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_sao_constraint_flag");                   pcSPS->setNoSaoConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_alf_constraint_flag");                   pcSPS->setNoAlfConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_pcm_constraint_flag");                   pcSPS->setNoPcmConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_ref_wraparound_constraint_flag");        pcSPS->setNoRefWraparoundConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_temporal_mvp_constraint_flag");          pcSPS->setNoTemporalMvpConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_sbtmvp_constraint_flag");                pcSPS->setNoSbtmvpConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_amvr_constraint_flag");                  pcSPS->setNoAmvrConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_bdof_constraint_flag");                  pcSPS->setNoBdofConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_cclm_constraint_flag");                  pcSPS->setNoCclmConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_mts_constraint_flag");                   pcSPS->setNoMtsConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_affine_motion_constraint_flag");         pcSPS->setNoAffineMotionConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_gbi_constraint_flag");                   pcSPS->setNoGbiConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_mh_intra_constraint_flag");              pcSPS->setNoMhIntraConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_triangle_constraint_flag");              pcSPS->setNoTriangleConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_ladf_constraint_flag");                  pcSPS->setNoLadfConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_curr_pic_ref_constraint_flag");          pcSPS->setNoCurrPicRefConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_qp_delta_constraint_flag");              pcSPS->setNoQpDeltaConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_dep_quant_constraint_flag");             pcSPS->setNoDepQuantConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_sign_data_hiding_constraint_flag");      pcSPS->setNoSignDataHidingConstraintFlag(uiCode > 0 ? true : false);

  // KJS: Marakech decision: sub-layers added back
  READ_CODE( 3,  uiCode, "sps_max_sub_layers_minus1" );          pcSPS->setMaxTLayers   ( uiCode+1 );
  CHECK(uiCode > 6, "Invalid maximum number of T-layer signalled");
  READ_FLAG( uiCode, "sps_temporal_id_nesting_flag" );           pcSPS->setTemporalIdNestingFlag ( uiCode > 0 ? true : false );
  if ( pcSPS->getMaxTLayers() == 1 )
  {
    // sps_temporal_id_nesting_flag must be 1 when sps_max_sub_layers_minus1 is 0
    CHECK( uiCode != 1, "Invalid maximum number of T-layers" );
  }
  parsePTL(pcSPS->getPTL(), true, pcSPS->getMaxTLayers() - 1);
#else
  READ_CODE(3, uiCode, "sps_max_sub_layers_minus1");          pcSPS->setMaxTLayers   (uiCode + 1);
  CHECK(uiCode > 6, "Invalid maximum number of T-layer signalled");
  READ_CODE(5, uiCode, "sps_reserved_zero_5bits");
  CHECK(uiCode != 0, "sps_reserved_zero_5bits not equal to zero");

  parseProfileTierLevel(pcSPS->getProfileTierLevel(), pcSPS->getMaxTLayers() - 1);

  READ_UVLC(uiCode, "sps_seq_parameter_set_id");           pcSPS->setSPSId(uiCode);
#endif

  READ_UVLC(     uiCode, "chroma_format_idc" );                  pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );
  CHECK(uiCode > 3, "Invalid chroma format signalled");

#if !JVET_N0671_CHROMA_FORMAT_422
  if (pcSPS->getChromaFormatIdc() == CHROMA_422)
  {
    EXIT("Error:  4:2:2 chroma sampling format not supported with current compiler setting."
      "\n        Set compiler flag \"JVET_N0671_CHROMA_FORMAT_422\" equal to 1 for enabling 4:2:2.\n");
  }
#endif //!JVET_N0671_CHROMA_FORMAT_422


  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG(     uiCode, "separate_colour_plane_flag");        CHECK(uiCode != 0, "Invalid code");
  }

  READ_UVLC (    uiCode, "pic_width_in_luma_samples" );          pcSPS->setPicWidthInLumaSamples ( uiCode    );
  READ_UVLC (    uiCode, "pic_height_in_luma_samples" );         pcSPS->setPicHeightInLumaSamples( uiCode    );

  // KJS: not removing yet
  READ_FLAG(     uiCode, "conformance_window_flag");
  if (uiCode != 0)
  {
    Window &conf = pcSPS->getConformanceWindow();
    READ_UVLC(   uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset  ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset   ( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
  }

  READ_UVLC(     uiCode, "bit_depth_luma_minus8" );
  CHECK(uiCode > 8, "Invalid luma bit depth signalled");
  pcSPS->setBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);

  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (int) (6*uiCode) );

  READ_UVLC( uiCode,    "bit_depth_chroma_minus8" );
  CHECK(uiCode > 8, "Invalid chroma bit depth signalled");
  pcSPS->setBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (int) (6*uiCode) );

  READ_UVLC( uiCode,    "log2_max_pic_order_cnt_lsb_minus4" );   pcSPS->setBitsForPOC( 4 + uiCode );
  CHECK(uiCode > 12, "Invalid code");
#if JVET_N0047_Merge_IDR_Non_IDR
  READ_FLAG( uiCode, "sps_idr_rpl_present_flag" ); pcSPS->setIDRRefParamListPresent( (bool) uiCode);                 
#endif
  // KJS: Marakech decision: sub-layers added back
  uint32_t subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "sps_sub_layer_ordering_info_present_flag");

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

#if JVET_M0128
  READ_FLAG(uiCode, "long_term_ref_pics_flag");          pcSPS->setLongTermRefsPresent(uiCode);
  READ_FLAG(uiCode, "rpl1_copy_from_rpl0_flag");
  pcSPS->setRPL1CopyFromRPL0Flag(uiCode);

  //Read candidate for List0
  READ_UVLC(uiCode, "num_ref_pic_lists_in_sps[0]");
  uint32_t numberOfRPL = uiCode;
  pcSPS->createRPLList0(numberOfRPL + 1);
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
    pcSPS->createRPLList1(numberOfRPL + 1);
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
#endif

  unsigned  minQT[3] = { 0, 0, 0 };
  unsigned  maxBTD[3] = { 0, 0, 0 };

  unsigned  maxBTSize[3] = { 0, 0, 0 };
  unsigned  maxTTSize[3] = { 0, 0, 0 };
  READ_FLAG(uiCode, "qtbtt_dual_tree_intra_flag");             pcSPS->setUseDualITree(uiCode);
  READ_UVLC(uiCode, "log2_ctu_size_minus2");                   pcSPS->setCTUSize(1 << (uiCode + 2));
  pcSPS->setMaxCodingDepth(uiCode);
  pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode);
  pcSPS->setMaxCUWidth(pcSPS->getCTUSize());
  pcSPS->setMaxCUHeight(pcSPS->getCTUSize());

  READ_UVLC(uiCode, "log2_min_luma_coding_block_size_minus2");
  int log2MinCUSize = uiCode + 2;
  pcSPS->setLog2MinCodingBlockSize(log2MinCUSize);
  READ_FLAG(uiCode, "partition_constraints_override_enabled_flag"); pcSPS->setSplitConsOverrideEnabledFlag(uiCode);
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_tile_group_luma");      minQT[0] = 1 << (uiCode + pcSPS->getLog2MinCodingBlockSize());
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_inter_tile_group");      minQT[1] = 1 << (uiCode + pcSPS->getLog2MinCodingBlockSize());
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_inter_tile_group");     maxBTD[1] = uiCode;
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_tile_group_luma");     maxBTD[0] = uiCode;

  maxTTSize[0] = maxBTSize[0] = minQT[0];
  if (maxBTD[0] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_tile_group_luma");     maxBTSize[0] <<= uiCode;
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_tile_group_luma");     maxTTSize[0] <<= uiCode;
  }
  maxTTSize[1] = maxBTSize[1] = minQT[1];
  if (maxBTD[1] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_inter_tile_group");     maxBTSize[1] <<= uiCode;
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_inter_tile_group");     maxTTSize[1] <<= uiCode;
  }
  if (pcSPS->getUseDualITree())
  {
    READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_tile_group_chroma"); minQT[2] = 1 << (uiCode + pcSPS->getLog2MinCodingBlockSize());
    READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_tile_group_chroma"); maxBTD[2] = uiCode;
    maxTTSize[2] = maxBTSize[2] = minQT[2];
    if (maxBTD[2] != 0)
    {
      READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_tile_group_chroma");       maxBTSize[2] <<= uiCode;
      READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_tile_group_chroma");       maxTTSize[2] <<= uiCode;
    }
}

  pcSPS->setMinQTSizes(minQT);
  pcSPS->setMaxBTDepth(maxBTD[1], maxBTD[0], maxBTD[2]);
  pcSPS->setMaxBTSize(maxBTSize[1], maxBTSize[0], maxBTSize[2]);
  pcSPS->setMaxTTSize(maxTTSize[1], maxTTSize[0], maxTTSize[2]);

#if !JVET_M0101_HLS
  if (pcSPS->getPTL()->getGeneralPTL()->getLevelIdc() >= Level::LEVEL5)
  {
    CHECK(log2MinCUSize + pcSPS->getLog2DiffMaxMinCodingBlockSize() < 5, "Invalid code");
  }
#endif

#if MAX_TB_SIZE_SIGNALLING
  // KJS: Not in syntax
  READ_UVLC( uiCode, "log2_max_luma_transform_block_size_minus2" ); pcSPS->setLog2MaxTbSize( uiCode + 2 );
#endif
  READ_FLAG( uiCode, "sps_sao_enabled_flag" );                      pcSPS->setSAOEnabledFlag ( uiCode ? true : false );
  READ_FLAG( uiCode, "sps_alf_enabled_flag" );                      pcSPS->setALFEnabledFlag ( uiCode ? true : false );

  READ_FLAG( uiCode, "sps_pcm_enabled_flag" );                          pcSPS->setPCMEnabledFlag( uiCode ? true : false );
  if( pcSPS->getPCMEnabledFlag() )
  {
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_luma_minus1" );          pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode );
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1" );        pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_CHROMA, 1 + uiCode );
    READ_UVLC( uiCode, "log2_min_pcm_luma_coding_block_size_minus3" );   pcSPS->setPCMLog2MinSize ( uiCode+3 );
    READ_UVLC( uiCode, "log2_diff_max_min_pcm_luma_coding_block_size" ); pcSPS->setPCMLog2MaxSize ( uiCode+pcSPS->getPCMLog2MinSize() );
    READ_FLAG( uiCode, "pcm_loop_filter_disable_flag" );                 pcSPS->setPCMFilterDisableFlag ( uiCode ? true : false );
  }

#if JVET_N0070_WRAPAROUND
  if( pcSPS->getCTUSize() + 2*pcSPS->getLog2MinCodingBlockSize() <= pcSPS->getPicWidthInLumaSamples() ) 
  {    
#endif
  READ_FLAG(uiCode, "sps_ref_wraparound_enabled_flag");                  pcSPS->setWrapAroundEnabledFlag( uiCode ? true : false );

  if (pcSPS->getWrapAroundEnabledFlag())
  {
    READ_UVLC(uiCode, "sps_ref_wraparound_offset_minus1");               pcSPS->setWrapAroundOffset( (uiCode+1)*(1 <<  pcSPS->getLog2MinCodingBlockSize()));
  }
#if JVET_N0070_WRAPAROUND
  }
  else 
  {
    pcSPS->setWrapAroundEnabledFlag(0);
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

  READ_FLAG( uiCode,  "sps_affine_amvr_enabled_flag" );             pcSPS->setAffineAmvrEnabledFlag ( uiCode != 0 );

  READ_FLAG(uiCode, "sps_dmvr_enable_flag");                        pcSPS->setUseDMVR(uiCode != 0);
#if JVET_N0127_MMVD_SPS_FLAG
  READ_FLAG(uiCode, "sps_mmvd_enable_flag");                        pcSPS->setUseMMVD(uiCode != 0);
#endif
  // KJS: sps_cclm_enabled_flag
  READ_FLAG( uiCode,    "lm_chroma_enabled_flag" );                 pcSPS->setUseLMChroma            ( uiCode != 0 );
  if ( pcSPS->getUseLMChroma() && pcSPS->getChromaFormatIdc() == CHROMA_420 )
  {
    READ_FLAG( uiCode,  "sps_cclm_collocated_chroma_flag" );        pcSPS->setCclmCollocatedChromaFlag( uiCode != 0 );
  }

  READ_FLAG( uiCode,    "mts_enabled_flag" );                       pcSPS->setUseMTS                 ( uiCode != 0 );
  if ( pcSPS->getUseMTS() )
  {
    READ_FLAG( uiCode,    "mts_intra_enabled_flag" );               pcSPS->setUseIntraMTS            ( uiCode != 0 );
    READ_FLAG( uiCode,    "mts_inter_enabled_flag" );               pcSPS->setUseInterMTS            ( uiCode != 0 );
  }
#if JVET_N0193_LFNST
  READ_FLAG( uiCode, "lfnst_enabled_flag" );                        pcSPS->setUseLFNST               ( uiCode != 0 );
#endif
#if JVET_N0235_SMVD_SPS
  READ_FLAG(uiCode, "smvd_flag");                                   pcSPS->setUseSMVD                ( uiCode != 0 );
#endif
  // KJS: sps_affine_enabled_flag
  READ_FLAG( uiCode,    "affine_flag" );                            pcSPS->setUseAffine              ( uiCode != 0 );
  if ( pcSPS->getUseAffine() )
  {
    READ_FLAG( uiCode,  "affine_type_flag" );                       pcSPS->setUseAffineType          ( uiCode != 0 );
  }
  READ_FLAG( uiCode,    "gbi_flag" );                               pcSPS->setUseGBi                 ( uiCode != 0 );
  READ_FLAG(uiCode, "ibc_flag");                                    pcSPS->setIBCFlag(uiCode);
  // KJS: sps_ciip_enabled_flag
  READ_FLAG( uiCode,     "mhintra_flag" );                           pcSPS->setUseMHIntra             ( uiCode != 0 );

#if JVET_N0127_MMVD_SPS_FLAG
  if ( pcSPS->getUseMMVD() )
  {
    READ_FLAG( uiCode,  "sps_fpel_mmvd_enabled_flag" );             pcSPS->setFpelMmvdEnabledFlag ( uiCode != 0 );
  }
#else
  READ_FLAG( uiCode,  "sps_fpel_mmvd_enabled_flag" );               pcSPS->setFpelMmvdEnabledFlag ( uiCode != 0 );
#endif

  READ_FLAG( uiCode,    "triangle_flag" );                          pcSPS->setUseTriangle            ( uiCode != 0 );

#if JVET_N0217_MATRIX_INTRAPRED
  READ_FLAG( uiCode,    "sps_mip_flag");                            pcSPS->setUseMIP                 ( uiCode != 0 );
#endif
  // KJS: not in draft yet
  READ_FLAG(uiCode, "sbt_enable_flag");                             pcSPS->setUseSBT(uiCode != 0);
  if( pcSPS->getUseSBT() )
  {
    READ_FLAG(uiCode, "max_sbt_size_64_flag");                      pcSPS->setMaxSbtSize(uiCode != 0 ? 64 : 32);
  }
  // KJS: not in draft yet
  READ_FLAG(uiCode, "sps_reshaper_enable_flag");                   pcSPS->setUseReshaper(uiCode == 1);
#if INCLUDE_ISP_CFG_FLAG
  READ_FLAG(uiCode, "isp_enable_flag");                            pcSPS->setUseISP(uiCode != 0);
#endif

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
#if !JVET_M0128
  READ_UVLC( uiCode, "num_short_term_ref_pic_sets" );
  CHECK(uiCode > 64, "Invalid code");
  pcSPS->createRPSList(uiCode);

  RPSList* rpsList = pcSPS->getRPSList();
  ReferencePictureSet* rps;

  for(uint32_t i=0; i< rpsList->getNumberOfReferencePictureSets(); i++)
  {
    rps = rpsList->getReferencePictureSet(i);
    parseShortTermRefPicSet(pcSPS,rps,i);
  }
  READ_FLAG( uiCode, "long_term_ref_pics_present_flag" );          pcSPS->setLongTermRefsPresent(uiCode);
  if (pcSPS->getLongTermRefsPresent())
  {
    READ_UVLC( uiCode, "num_long_term_ref_pics_sps" );
    pcSPS->setNumLongTermRefPicSPS(uiCode);
    for (uint32_t k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      READ_CODE( pcSPS->getBitsForPOC(), uiCode, "lt_ref_pic_poc_lsb_sps" );
      pcSPS->setLtRefPicPocLsbSps(k, uiCode);
      READ_FLAG( uiCode,  "used_by_curr_pic_lt_sps_flag[i]");
      pcSPS->setUsedByCurrPicLtSPSFlag(k, uiCode?1:0);
    }
  }
#endif

  // KJS: not found in draft -> does not exist
#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  READ_FLAG( uiCode, "strong_intra_smoothing_enable_flag" );      pcSPS->setUseStrongIntraSmoothing(uiCode);
#endif

  // KJS: remove scaling lists?
#if HEVC_USE_SCALING_LISTS
  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );
  if(pcSPS->getScalingListFlag())
  {
    READ_FLAG( uiCode, "sps_scaling_list_data_present_flag" );                 pcSPS->setScalingListPresentFlag ( uiCode );
    if(pcSPS->getScalingListPresentFlag ())
    {
      parseScalingList( &(pcSPS->getScalingList()) );
    }
  }
#endif

#if JVET_N0063_VUI
  TimingInfo *timingInfo = pcSPS->getTimingInfo();
  READ_FLAG(       uiCode, "timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "time_scale");                       timingInfo->setTimeScale                  (uiCode);

    READ_FLAG(     uiCode, "hrd_parameters_present_flag");        pcSPS->setHrdParametersPresentFlag(uiCode);
    if( pcSPS->getHrdParametersPresentFlag() )
    {
      parseHrdParameters( pcSPS->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }
#endif

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

#if JVET_N0349_DPS
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
  READ_FLAG( symbol,      "dps_reserved_zero_bit" );              CHECK(symbol != 0, "dps_reserved_zero_bit must be equal to zero");

  ProfileTierLevel ptl;
  parseProfileTierLevel(&ptl, dps->getMaxSubLayersMinus1());
  dps->setProfileTierLevel(ptl);
  
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
#endif

#if HEVC_VPS
void HLSyntaxReader::parseVPS(VPS* pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader ();
#endif
  uint32_t  uiCode;

  READ_CODE( 4,  uiCode,  "vps_video_parameter_set_id" );         pcVPS->setVPSId( uiCode );
  READ_FLAG( uiCode,      "vps_base_layer_internal_flag" );       CHECK(uiCode != 1, "Invalid code");
  READ_FLAG( uiCode,      "vps_base_layer_available_flag" );      CHECK(uiCode != 1, "Invalid code");
  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );
  READ_CODE( 3,  uiCode,  "vps_max_sub_layers_minus1" );          pcVPS->setMaxTLayers( uiCode + 1 );    CHECK(uiCode+1 > MAX_TLAYER, "Invalid code");
  READ_FLAG(     uiCode,  "vps_temporal_id_nesting_flag" );       pcVPS->setTemporalNestingFlag( uiCode ? true:false );
  CHECK (pcVPS->getMaxTLayers()<=1&&!pcVPS->getTemporalNestingFlag(), "Invalid VPS state");
  READ_CODE( 16, uiCode,  "vps_reserved_0xffff_16bits" );         CHECK(uiCode != 0xffff, "Invalid value for reserved bits");
  parsePTL ( pcVPS->getPTL(), true, pcVPS->getMaxTLayers()-1);
  uint32_t subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "vps_sub_layer_ordering_info_present_flag");
  for(uint32_t i = 0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC( uiCode,  "vps_max_dec_pic_buffering_minus1[i]" );    pcVPS->setMaxDecPicBuffering( uiCode + 1, i );
    READ_UVLC( uiCode,  "vps_max_num_reorder_pics[i]" );            pcVPS->setNumReorderPics( uiCode, i );
    READ_UVLC( uiCode,  "vps_max_latency_increase_plus1[i]" );      pcVPS->setMaxLatencyIncrease( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcVPS->getMaxTLayers()-1; i++)
      {
        pcVPS->setMaxDecPicBuffering(pcVPS->getMaxDecPicBuffering(0), i);
        pcVPS->setNumReorderPics(pcVPS->getNumReorderPics(0), i);
        pcVPS->setMaxLatencyIncrease(pcVPS->getMaxLatencyIncrease(0), i);
      }
      break;
    }
  }

  CHECK( pcVPS->getNumHrdParameters() >= MAX_VPS_OP_SETS_PLUS1, "Too many HDR parameters" );
  CHECK( pcVPS->getMaxNuhReservedZeroLayerId() >= MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1, "Reserved zero layer id too big" );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );                        pcVPS->setMaxNuhReservedZeroLayerId( uiCode );
  READ_UVLC(    uiCode, "vps_num_layer_sets_minus1" );               pcVPS->setMaxOpSets( uiCode + 1 );
  for( uint32_t opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( uint32_t i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
    {
      READ_FLAG( uiCode, "layer_id_included_flag[opsIdx][i]" );   pcVPS->setLayerIdIncludedFlag( uiCode == 1 ? true : false, opsIdx, i );
    }
  }

  TimingInfo *timingInfo = pcVPS->getTimingInfo();
  READ_FLAG(       uiCode, "vps_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vps_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vps_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vps_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vps_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_UVLC( uiCode, "vps_num_hrd_parameters" );                  pcVPS->setNumHrdParameters( uiCode );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      pcVPS->createHrdParamBuffer();
    }
    for( uint32_t i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
    {
      READ_UVLC( uiCode, "hrd_layer_set_idx[i]" );                  pcVPS->setHrdOpSetIdx( uiCode, i );
      if( i > 0 )
      {
        READ_FLAG( uiCode, "cprms_present_flag[i]" );               pcVPS->setCprmsPresentFlag( uiCode == 1 ? true : false, i );
      }
      else
      {
        pcVPS->setCprmsPresentFlag( true, i );
      }

      parseHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
    }
  }

  READ_FLAG( uiCode,  "vps_extension_flag" );
  if (uiCode)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "vps_extension_data_flag");
    }
  }

  xReadRbspTrailingBits();
}
#elif JVET_N0278_HLS
void HLSyntaxReader::parseVPS(VPS* pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  uint32_t  uiCode;

  READ_CODE(4, uiCode, "vps_video_parameter_set_id");         pcVPS->setVPSId(uiCode);
  READ_CODE(8, uiCode, "vps_max_layers_minus1");              pcVPS->setMaxLayers(uiCode + 1);    CHECK(uiCode + 1 > MAX_VPS_LAYERS, "Invalid code");
  for (uint32_t i = 0; i <= pcVPS->getMaxLayers() - 1; i++)
  {
    READ_CODE(7, uiCode, "vps_included_layer_id");          pcVPS->setVPSIncludedLayerId(uiCode, i);
    READ_FLAG(uiCode, "vps_reserved_zero_1bit");
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
#endif

void HLSyntaxReader::parseSliceHeader (Slice* pcSlice, ParameterSetManager *parameterSetManager, const int prevTid0POC)
{
  uint32_t  uiCode;
  int   iCode;

#if ENABLE_TRACING
  xTraceSliceHeader();
#endif
  PPS* pps = NULL;
  SPS* sps = NULL;

#if !JVET_N0857_RECT_SLICES
  uint32_t firstSliceSegmentInPic;
  READ_FLAG( firstSliceSegmentInPic, "first_slice_segment_in_pic_flag" );
#endif
  if( pcSlice->getRapPicFlag())
  {
    READ_FLAG( uiCode, "no_output_of_prior_pics_flag" );  //ignored -- updated already
    pcSlice->setNoOutputPriorPicsFlag(uiCode ? true : false);
  }
  READ_UVLC (    uiCode, "slice_pic_parameter_set_id" );  pcSlice->setPPSId(uiCode);
  pps = parameterSetManager->getPPS(uiCode);
  //!KS: need to add error handling code here, if PPS is not available
  CHECK(pps==0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  CHECK(sps==0, "Invalid SPS");

  const ChromaFormat chFmt = sps->getChromaFormatIdc();
  const uint32_t numValidComp=getNumberValidComponents(chFmt);
  const bool bChroma=(chFmt!=CHROMA_400);

#if !JVET_N0857_RECT_SLICES
  int numCTUs = ((sps->getPicWidthInLumaSamples()+sps->getMaxCUWidth()-1)/sps->getMaxCUWidth())*((sps->getPicHeightInLumaSamples()+sps->getMaxCUHeight()-1)/sps->getMaxCUHeight());
  uint32_t sliceSegmentAddress = 0;
  int bitsSliceSegmentAddress = 0;
  while(numCTUs>(1<<bitsSliceSegmentAddress))
  {
    bitsSliceSegmentAddress++;
  }

  if(!firstSliceSegmentInPic)
  {
    READ_CODE( bitsSliceSegmentAddress, sliceSegmentAddress, "slice_segment_address" );
  }
  //set uiCode to equal slice start address (or dependent slice start address)
    pcSlice->setSliceCurStartCtuTsAddr(sliceSegmentAddress); // this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
    pcSlice->setSliceCurEndCtuTsAddr(numCTUs);
#endif
#if JVET_N0857_RECT_SLICES
    int bitsSliceAddress = 1;
    if (!pps->getRectSliceFlag())
    {
      while (pps->getNumTilesInPic() > (1 << bitsSliceAddress))  //TODO: use the correct one
      {
        bitsSliceAddress++;
      }
    }
    else
    {
      if (pps->getSignalledSliceIdFlag())
      {
        bitsSliceAddress = pps->getSignalledSliceIdLengthMinus1() + 1;
      }
      else
      {
        while ((pps->getNumSlicesInPicMinus1() + 1) > (1 << bitsSliceAddress))
        {
          bitsSliceAddress++;
        }
      }
    }
    if (pps->getRectSliceFlag() || pps->getNumTilesInPic() > 1)   //TODO: change it to getNumBricksInPic when Tile/Brick is updated.
    {
      if (pps->getRectSliceFlag())
      {
        READ_CODE(bitsSliceAddress, uiCode, "slice_address");
        int sliceIdx = 0;
        while (pps->getSliceId(sliceIdx) != uiCode && sliceIdx <= pps->getNumSlicesInPicMinus1())
        {
          sliceIdx++;
        }
        pcSlice->setSliceCurStartBrickIdx(pps->getTopLeftBrickIdx(sliceIdx));
        pcSlice->setSliceCurEndBrickIdx(pps->getBottomRightBrickIdx(sliceIdx));
      }
      else
      {
        READ_CODE(bitsSliceAddress, uiCode, "slice_address");
        pcSlice->setSliceCurStartBrickIdx(uiCode);
      }
    }
    if (!pps->getRectSliceFlag() && !pps->getSingleBrickPerSliceFlag())
    {
      READ_UVLC(uiCode, "num_bricks_in_slice_minus1");
      pcSlice->setSliceNumBricks(uiCode + 1);
      pcSlice->setSliceCurEndBrickIdx(pcSlice->getSliceCurStartBrickIdx() + uiCode);
    }
    pcSlice->setSliceCurStartCtuTsAddr(pcSlice->getSliceCurStartBrickIdx());
#endif

    for (int i = 0; i < pps->getNumExtraSliceHeaderBits(); i++)
    {
      READ_FLAG(uiCode, "slice_reserved_flag[]"); // ignored
    }

    READ_UVLC (    uiCode, "slice_type" );            pcSlice->setSliceType((SliceType)uiCode);
    if( pps->getOutputFlagPresentFlag() )
    {
      READ_FLAG( uiCode, "pic_output_flag" );    pcSlice->setPicOutputFlag( uiCode ? true : false );
    }
    else
    {
      pcSlice->setPicOutputFlag( true );
    }

    // if (separate_colour_plane_flag == 1)
    //   read colour_plane_id
    //   (separate_colour_plane_flag == 1) is not supported in this version of the standard.

#if JVET_N0047_Merge_IDR_Non_IDR
    if( pcSlice->getIdrPicFlag() && !(sps->getIDRRefParamListPresent()))
#else
    if( pcSlice->getIdrPicFlag() )
#endif
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
      pcSlice->setPOC(uiCode);
#if !JVET_M0128
      ReferencePictureSet* rps = pcSlice->getLocalRPS();
      (*rps)=ReferencePictureSet();
      pcSlice->setRPS(rps);
#endif
    }
    else
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
      int iPOClsb = uiCode;
      int iPrevPOC = prevTid0POC;
      int iMaxPOClsb = 1<< sps->getBitsForPOC();
      int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
      int iPrevPOCmsb = iPrevPOC-iPrevPOClsb;
      int iPOCmsb;
      if( ( iPOClsb  <  iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb )  >=  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if( (iPOClsb  >  iPrevPOClsb )  && ( (iPOClsb - iPrevPOClsb )  >  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
#if !JVET_M0101_HLS
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // For BLA picture types, POCmsb is set to 0.
        iPOCmsb = 0;
      }
#endif
      pcSlice->setPOC              (iPOCmsb+iPOClsb);

#if JVET_M0128
      //Read L0 related syntax elements
      if (sps->getNumRPL0() > 0)
      {
        READ_FLAG(uiCode, "ref_pic_list_sps_flag[0]");
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
          int numBits = (int)ceil(log2(sps->getNumRPL0()));
          READ_CODE(numBits, uiCode, "ref_pic_list_idx[0]");
          pcSlice->setRPL0idx(uiCode);
          pcSlice->setRPL0(sps->getRPLList0()->getReferencePictureList(uiCode));
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
            READ_FLAG(uiCode, "delta_poc_msb_present_flag[i][j]");
            pcSlice->getLocalRPL0()->setDeltaPocMSBPresentFlag(i, uiCode ? true : false);
            if (uiCode)
            {
              READ_FLAG(uiCode, "delta_poc_msb_cycle_lt[i][j]");
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
        if (sps->getNumRPL0() > 0)
        {
          READ_FLAG(uiCode, "ref_pic_list_sps_flag[1]");
        }
        else
        {
          uiCode = 0;
        }
        if (uiCode == 1)
        {
          if (sps->getNumRPL1() > 1)
          {
            int numBits = (int)ceil(log2(sps->getNumRPL1()));
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
            READ_FLAG(uiCode, "delta_poc_msb_present_flag[i][j]");
            pcSlice->getLocalRPL1()->setDeltaPocMSBPresentFlag(i, uiCode ? true : false);
            if (uiCode)
            {
              READ_FLAG(uiCode, "delta_poc_msb_cycle_lt[i][j]");
              pcSlice->getLocalRPL1()->setDeltaPocMSBCycleLT(i, uiCode);
            }
          }
        }
      }
#else
      ReferencePictureSet* rps;
      rps = pcSlice->getLocalRPS();
      (*rps)=ReferencePictureSet();

      pcSlice->setRPS(rps);
      READ_FLAG( uiCode, "short_term_ref_pic_set_sps_flag" );
      if(uiCode == 0) // use short-term reference picture set explicitly signalled in slice header
      {
        parseShortTermRefPicSet(sps,rps, sps->getRPSList()->getNumberOfReferencePictureSets());
      }
      else // use reference to short-term reference picture set in PPS
      {
        int numBits = 0;
        while ((1 << numBits) < sps->getRPSList()->getNumberOfReferencePictureSets())
        {
          numBits++;
        }
        if (numBits > 0)
        {
          READ_CODE( numBits, uiCode, "short_term_ref_pic_set_idx");
        }
        else
        {
          uiCode = 0;

        }
        *rps = *(sps->getRPSList()->getReferencePictureSet(uiCode));
      }
      if(sps->getLongTermRefsPresent())
      {
        int offset = rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures();
        uint32_t numOfLtrp = 0;
        uint32_t numLtrpInSPS = 0;
        if (sps->getNumLongTermRefPicSPS() > 0)
        {
          READ_UVLC( uiCode, "num_long_term_sps");
          numLtrpInSPS = uiCode;
          numOfLtrp += numLtrpInSPS;
          rps->setNumberOfLongtermPictures(numOfLtrp);
        }
        int bitsForLtrpInSPS = 0;
        while (sps->getNumLongTermRefPicSPS() > (1 << bitsForLtrpInSPS))
        {
          bitsForLtrpInSPS++;
        }
        READ_UVLC( uiCode, "num_long_term_pics");             rps->setNumberOfLongtermPictures(uiCode);
        numOfLtrp += uiCode;
        rps->setNumberOfLongtermPictures(numOfLtrp);
        int maxPicOrderCntLSB = 1 << sps->getBitsForPOC();
        int prevDeltaMSB = 0, deltaPocMSBCycleLT = 0;
        for(int j=offset+rps->getNumberOfLongtermPictures()-1, k = 0; k < numOfLtrp; j--, k++)
        {
          int pocLsbLt;
          if (k < numLtrpInSPS)
          {
            uiCode = 0;
            if (bitsForLtrpInSPS > 0)
            {
              READ_CODE(bitsForLtrpInSPS, uiCode, "lt_idx_sps[i]");
            }
            bool usedByCurrFromSPS=sps->getUsedByCurrPicLtSPSFlag(uiCode);

            pocLsbLt = sps->getLtRefPicPocLsbSps(uiCode);
            rps->setUsed(j,usedByCurrFromSPS);
          }
          else
          {
            READ_CODE(sps->getBitsForPOC(), uiCode, "poc_lsb_lt"); pocLsbLt= uiCode;
            READ_FLAG( uiCode, "used_by_curr_pic_lt_flag");     rps->setUsed(j,uiCode);
          }
          READ_FLAG(uiCode,"delta_poc_msb_present_flag");
          bool mSBPresentFlag = uiCode ? true : false;
          if(mSBPresentFlag)
          {
            READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i]" );
            bool deltaFlag = false;
            //            First LTRP                               || First LTRP from SH
            if( (j == offset+rps->getNumberOfLongtermPictures()-1) || (j == offset+(numOfLtrp-numLtrpInSPS)-1) )
            {
              deltaFlag = true;
            }
            if(deltaFlag)
            {
              deltaPocMSBCycleLT = uiCode;
            }
            else
            {
              deltaPocMSBCycleLT = uiCode + prevDeltaMSB;
            }

            int pocLTCurr = pcSlice->getPOC() - deltaPocMSBCycleLT * maxPicOrderCntLSB
              - iPOClsb + pocLsbLt;
            rps->setPOC     (j, pocLTCurr);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLTCurr);
            rps->setCheckLTMSBPresent(j,true);
          }
          else
          {
            rps->setPOC     (j, pocLsbLt);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLsbLt);
            rps->setCheckLTMSBPresent(j,false);

            // reset deltaPocMSBCycleLT for first LTRP from slice header if MSB not present
            if( j == offset+(numOfLtrp-numLtrpInSPS)-1 )
            {
              deltaPocMSBCycleLT = 0;
            }
          }
          prevDeltaMSB = deltaPocMSBCycleLT;
        }
        offset += rps->getNumberOfLongtermPictures();
        rps->setNumberOfPictures(offset);
      }
#if !JVET_M0101_HLS
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // In the case of BLA picture types, rps data is read from slice header but ignored
        rps = pcSlice->getLocalRPS();
        (*rps)=ReferencePictureSet();
        pcSlice->setRPS(rps);
      }
#endif
#endif

      if (sps->getSPSTemporalMVPEnabledFlag())
      {
        READ_FLAG( uiCode, "slice_temporal_mvp_enabled_flag" );
        pcSlice->setEnableTMVPFlag( uiCode == 1 ? true : false );
      }
      else
      {
        pcSlice->setEnableTMVPFlag(false);
      }
    }


    if(sps->getSAOEnabledFlag())
    {
      READ_FLAG(uiCode, "slice_sao_luma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, (bool)uiCode);

      if (bChroma)
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, (bool)uiCode);
      }
    }

    if( sps->getALFEnabledFlag() )
    {
      READ_FLAG(uiCode, "tile_group_alf_enabled_flag");
#if JVET_N0415_CTB_ALF
      pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Y, uiCode);
      int alfChromaIdc = 0;
      if (uiCode)
      {
        if (pcSlice->isIntra())
        {
          READ_FLAG(uiCode, "tile_group_num_APS");
        }
        else
        {
          xReadTruncBinCode(uiCode, ALF_CTB_MAX_NUM_APS + 1);
        }
        int numAps = uiCode;
        pcSlice->setTileGroupNumAps(numAps);
        std::vector<int> apsId(numAps, -1);
        for (int i = 0; i < numAps; i++)
        {
          READ_CODE(5, uiCode, "tile_group_aps_id");
          apsId[i] = uiCode;
        }
		
		
#if JVET_N0805_APS_LMCS
        pcSlice->setAlfAPSs(apsId);
#else
        pcSlice->setAPSs(apsId);
#endif
        alfChromaIdc = truncatedUnaryEqProb(3);        //alf_chroma_idc
        if (alfChromaIdc)
        {
          if (pcSlice->isIntra() && pcSlice->getTileGroupNumAps() == 1)
          {
            uiCode = apsId[0];
          }
          else
          {
            READ_CODE(5, uiCode, "tile_group_aps_id_chroma");
          }
          pcSlice->setTileGroupApsIdChroma(uiCode);
        }
      }
      else
      {
        pcSlice->setTileGroupNumAps(0);
      }
      pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cb, alfChromaIdc & 1);
      pcSlice->setTileGroupAlfEnabledFlag(COMPONENT_Cr, alfChromaIdc >> 1);
#else
      if (uiCode)
      {
        READ_CODE(5, uiCode, "tile_group_aps_id");
        pcSlice->setAPSId(uiCode);
        pcSlice->setAPS(parameterSetManager->getAPS(uiCode));
        pcSlice->setTileGroupAlfEnabledFlag(true);
      }
      else
      {
        pcSlice->setTileGroupAlfEnabledFlag(false);
        pcSlice->setAPSId(-1);
        pcSlice->setAPS(nullptr);
      }
#endif
    }

    if (pcSlice->getIdrPicFlag())
    {
      pcSlice->setEnableTMVPFlag(false);
    }
    if (!pcSlice->isIntra())
    {

      READ_FLAG( uiCode, "num_ref_idx_active_override_flag");
      if (uiCode)
      {
        READ_UVLC (uiCode, "num_ref_idx_l0_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
        if (pcSlice->isInterB())
        {
          READ_UVLC (uiCode, "num_ref_idx_l1_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
        }
      }
      else
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive());
        if (pcSlice->isInterB())
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive());
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1,0);
        }
      }
    }
    // }
#if !JVET_M0128
    RefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    if(!pcSlice->isIntra())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
      {
        refPicListModification->setRefPicListModificationFlagL0( 0 );
      }
      else
      {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l0" ); refPicListModification->setRefPicListModificationFlagL0( uiCode ? 1 : 0 );
      }

      if(refPicListModification->getRefPicListModificationFlagL0())
      {
        uiCode = 0;
        int i = 0;
        int numRpsCurrTempList0 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList0 > 1 )
        {
          int length = 1;
          numRpsCurrTempList0 --;
          while ( numRpsCurrTempList0 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l0" );
            refPicListModification->setRefPicSetIdxL0(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
          {
            refPicListModification->setRefPicSetIdxL0(i, 0 );
          }
        }
      }
    }
    else
    {
      refPicListModification->setRefPicListModificationFlagL0(0);
    }
    if(pcSlice->isInterB())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
      {
        refPicListModification->setRefPicListModificationFlagL1( 0 );
      }
      else
      {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l1" ); refPicListModification->setRefPicListModificationFlagL1( uiCode ? 1 : 0 );
      }
      if(refPicListModification->getRefPicListModificationFlagL1())
      {
        uiCode = 0;
        int i = 0;
        int numRpsCurrTempList1 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList1 > 1 )
        {
          int length = 1;
          numRpsCurrTempList1 --;
          while ( numRpsCurrTempList1 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l1" );
            refPicListModification->setRefPicSetIdxL1(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
          {
            refPicListModification->setRefPicSetIdxL1(i, 0 );
          }
        }
      }
    }
    else
    {
      refPicListModification->setRefPicListModificationFlagL1(0);
    }
#endif

    if (pcSlice->isInterB())
    {
      READ_FLAG( uiCode, "mvd_l1_zero_flag" );       pcSlice->setMvdL1ZeroFlag( (uiCode ? true : false) );
    }

    pcSlice->setCabacInitFlag( false ); // default
    if(pps->getCabacInitPresentFlag() && !pcSlice->isIntra())
    {
      READ_FLAG(uiCode, "cabac_init_flag");
      pcSlice->setCabacInitFlag( uiCode ? true : false );
      pcSlice->setEncCABACTableIdx( pcSlice->getSliceType() == B_SLICE ? ( uiCode ? P_SLICE : B_SLICE ) : ( uiCode ? B_SLICE : P_SLICE ) );
    }

    if ( pcSlice->getEnableTMVPFlag() )
    {
      if ( pcSlice->getSliceType() == B_SLICE )
      {
        READ_FLAG( uiCode, "collocated_from_l0_flag" );
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
    READ_FLAG( uiCode, "dep_quant_enabled_flag" );
    pcSlice->setDepQuantEnabledFlag( uiCode != 0 );
#if HEVC_USE_SIGN_HIDING
    if( !pcSlice->getDepQuantEnabledFlag() )
    {
      READ_FLAG( uiCode, "sign_data_hiding_enabled_flag" );
      pcSlice->setSignDataHidingEnabledFlag( uiCode != 0 );
    }
    else
    {
      pcSlice->setSignDataHidingEnabledFlag( 0 );
    }
#endif
    if (
      sps->getSplitConsOverrideEnabledFlag()
      )
    {
      READ_FLAG(uiCode, "partition_constrainst_override_flag");        pcSlice->setSplitConsOverrideFlag(uiCode ? true : false);
      if (pcSlice->getSplitConsOverrideFlag())
      {
        READ_UVLC(uiCode, "log2_diff_min_qt_min_cb");                 pcSlice->setMinQTSize(1 << (uiCode + sps->getLog2MinCodingBlockSize()));
        READ_UVLC(uiCode, "max_mtt_hierarchy_depth");                 pcSlice->setMaxBTDepth(uiCode);
        if (pcSlice->getMaxBTDepth() != 0)
        {
          READ_UVLC(uiCode, "log2_diff_max_bt_min_qt");             pcSlice->setMaxBTSize(pcSlice->getMinQTSize() << uiCode);
          READ_UVLC(uiCode, "log2_diff_max_tt_min_qt");             pcSlice->setMaxTTSize(pcSlice->getMinQTSize() << uiCode);
        }
        else
        {
          pcSlice->setMaxBTSize(pcSlice->getMinQTSize());
          pcSlice->setMaxTTSize(pcSlice->getMinQTSize());
        }
        if (
          pcSlice->isIntra() && sps->getUseDualITree()
          )
        {
          READ_UVLC(uiCode, "log2_diff_min_qt_min_cb_chroma");                 pcSlice->setMinQTSizeIChroma(1 << (uiCode + sps->getLog2MinCodingBlockSize()));
          READ_UVLC(uiCode, "max_mtt_hierarchy_depth_chroma");                            pcSlice->setMaxBTDepthIChroma(uiCode);
          if (pcSlice->getMaxBTDepthIChroma() != 0)
          {
            READ_UVLC(uiCode, "log2_diff_max_bt_min_qt_chroma");             pcSlice->setMaxBTSizeIChroma(pcSlice->getMinQTSizeIChroma() << uiCode);
            READ_UVLC(uiCode, "log2_diff_max_tt_min_qt_chroma");             pcSlice->setMaxTTSizeIChroma(pcSlice->getMinQTSizeIChroma() << uiCode);
          }
          else
          {
            pcSlice->setMaxBTSizeIChroma(pcSlice->getMinQTSizeIChroma());
            pcSlice->setMaxTTSizeIChroma(pcSlice->getMinQTSizeIChroma());
          }
        }
      }
    }

    if (!pcSlice->isIntra() || sps->getIBCFlag())
    {
      READ_UVLC(uiCode, "six_minus_max_num_merge_cand");
      pcSlice->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
    }

    if (!pcSlice->isIntra())
    {

      if ( sps->getSBTMVPEnabledFlag() && !sps->getUseAffine() ) // ATMVP only
      {
        pcSlice->setMaxNumAffineMergeCand( 1 );
      }
      else if ( !sps->getSBTMVPEnabledFlag() && !sps->getUseAffine() ) // both off
      {
        pcSlice->setMaxNumAffineMergeCand( 0 );
      }
      else
      if ( sps->getUseAffine() )
      {
        READ_UVLC( uiCode, "five_minus_max_num_affine_merge_cand" );
        pcSlice->setMaxNumAffineMergeCand( AFFINE_MRG_MAX_NUM_CANDS - uiCode );
      }
      if ( sps->getFpelMmvdEnabledFlag() )
      {
        READ_FLAG( uiCode, "tile_group_fracmmvd_disabled_flag" );
        pcSlice->setDisFracMMVD( uiCode ? true : false );
      }
#if JVET_N0400_SIGNAL_TRIANGLE_CAND_NUM
      if (sps->getUseTriangle() && pcSlice->getMaxNumMergeCand() >= 2)
      {
        READ_UVLC(uiCode, "max_num_merge_cand_minus_max_num_triangle_cand");
        CHECK(pcSlice->getMaxNumMergeCand() < uiCode, "Incorrrect max number of triangle candidates!");
        pcSlice->setMaxNumTriangleCand((uint32_t)(pcSlice->getMaxNumMergeCand() - uiCode));
      }
      else
      {
        pcSlice->setMaxNumTriangleCand(0);
      }
#endif
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

#if JVET_N0054_JOINT_CHROMA
        READ_SVLC( iCode, "slice_cb_cr_qp_offset" );
        pcSlice->setSliceChromaQpDelta(JOINT_CbCr, iCode );
        CHECK( pcSlice->getSliceChromaQpDelta(JOINT_CbCr) < -12, "Invalid chroma QP offset" );
        CHECK( pcSlice->getSliceChromaQpDelta(JOINT_CbCr) >  12, "Invalid chroma QP offset" );
        CHECK( (pps->getQpOffset(JOINT_CbCr) + pcSlice->getSliceChromaQpDelta(JOINT_CbCr)) < -12, "Invalid chroma QP offset" );
        CHECK( (pps->getQpOffset(JOINT_CbCr) + pcSlice->getSliceChromaQpDelta(JOINT_CbCr)) >  12, "Invalid chroma QP offset" );
#endif
      }
    }

    if (pps->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag())
    {
      READ_FLAG(uiCode, "cu_chroma_qp_offset_enabled_flag"); pcSlice->setUseChromaQpAdj(uiCode != 0);
    }
    else
    {
      pcSlice->setUseChromaQpAdj(false);
    }

    if (pps->getDeblockingFilterControlPresentFlag())
    {
      if(pps->getDeblockingFilterOverrideEnabledFlag())
      {
        READ_FLAG ( uiCode, "deblocking_filter_override_flag" );        pcSlice->setDeblockingFilterOverrideFlag(uiCode ? true : false);
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
        pcSlice->setDeblockingFilterDisable       ( pps->getPPSDeblockingFilterDisabledFlag() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable       ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
    }

    bool isSAOEnabled = sps->getSAOEnabledFlag() && (pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_LUMA) || (bChroma && pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA)));
    bool isDBFEnabled = (!pcSlice->getDeblockingFilterDisable());

    if(pps->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ))
    {
      READ_FLAG( uiCode, "slice_loop_filter_across_slices_enabled_flag");
    }
    else
    {
      uiCode = pps->getLoopFilterAcrossSlicesEnabledFlag()?1:0;
    }
    pcSlice->setLFCrossSliceBoundaryFlag( (uiCode==1)?true:false);

    if (sps->getUseReshaper())
    {
#if JVET_N0805_APS_LMCS
      READ_FLAG(uiCode, "slice_lmcs_enabled_flag");          
      pcSlice->setLmcsEnabledFlag(uiCode == 1);

      if (pcSlice->getLmcsEnabledFlag())
      {
        READ_CODE(5, uiCode, "slice_lmcs_aps_id");
        
        pcSlice->setLmcsAPSId(uiCode);
        if (!(sps->getUseDualITree() && pcSlice->isIntra()))
        {
          READ_FLAG(uiCode, "slice_chroma_residual_scale_flag");                
          pcSlice->setLmcsChromaResidualScaleFlag(uiCode == 1);
        }
        else
        {
          pcSlice->setLmcsChromaResidualScaleFlag(false);
        }
      }
#else
      parseReshaper(pcSlice->getReshapeInfo(), sps, pcSlice->isIntra());
#endif
    }

#if JVET_N0857_RECT_SLICES
    if( pcSlice->getSliceCurStartBrickIdx() == 0 )
#else
    if( firstSliceSegmentInPic )
#endif
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
#if JVET_N0857_TILES_BRICKS
  if( !pps->getSingleTileInPicFlag() || pps->getEntropyCodingSyncEnabledFlag() )
#else
  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
#endif
  {
    uint32_t numEntryPointOffsets;
    uint32_t offsetLenMinus1;
    READ_UVLC( numEntryPointOffsets, "num_entry_point_offsets" );
    if( numEntryPointOffsets > 0 )
    {
      READ_UVLC( offsetLenMinus1, "offset_len_minus1" );
      entryPointOffset.resize( numEntryPointOffsets );
      for( uint32_t idx = 0; idx < numEntryPointOffsets; idx++ )
      {
        READ_CODE( offsetLenMinus1 + 1, uiCode, "entry_point_offset_minus1" );
        entryPointOffset[idx] = uiCode + 1;
      }
    }
  }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
#else
  m_pcBitstream->readByteAlignment();
#endif

  pcSlice->clearSubstreamSizes();

#if !JVET_N0857_TILES_BRICKS
  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
#else
  if( !pps->getSingleTileInPicFlag() || pps->getEntropyCodingSyncEnabledFlag() )
#endif
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

#if JVET_M0101_HLS
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
#if JVET_N0276_CONSTRAINT_FLAGS
  READ_FLAG(symbol, "no_partition_constraints_override_constraint_flag"); cinfo->setNoPartitionConstraintsOverrideConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol,  "no_sao_constraint_flag");                    cinfo->setNoSaoConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_alf_constraint_flag");                    cinfo->setNoAlfConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_pcm_constraint_flag");                    cinfo->setNoPcmConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_ref_wraparound_constraint_flag");         cinfo->setNoRefWraparoundConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_temporal_mvp_constraint_flag");           cinfo->setNoTemporalMvpConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_sbtmvp_constraint_flag");                 cinfo->setNoSbtmvpConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_amvr_constraint_flag");                   cinfo->setNoAmvrConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol,  "no_bdof_constraint_flag");                   cinfo->setNoBdofConstraintFlag(symbol > 0 ? true : false);
#if JVET_N0276_CONSTRAINT_FLAGS
  READ_FLAG(symbol, "no_dmvr_constraint_flag");                    cinfo->setNoDmvrConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_cclm_constraint_flag");                    cinfo->setNoCclmConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_mts_constraint_flag");                     cinfo->setNoMtsConstraintFlag(symbol > 0 ? true : false);
#if JVET_N0276_CONSTRAINT_FLAGS
  READ_FLAG(symbol, "no_sbt_constraint_flag");                     cinfo->setNoSbtConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_affine_motion_constraint_flag");           cinfo->setNoAffineMotionConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_gbi_constraint_flag");                     cinfo->setNoGbiConstraintFlag(symbol > 0 ? true : false);
#if JVET_N0276_CONSTRAINT_FLAGS
  READ_FLAG(symbol, "no_ibc_constraint_flag");                     cinfo->setNoIbcConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_mh_intra_constraint_flag");                cinfo->setNoMhIntraConstraintFlag(symbol > 0 ? true : false);
#if JVET_N0276_CONSTRAINT_FLAGS
  READ_FLAG(symbol, "no_fpel_mmvd_constraint_flag");               cinfo->setNoFPelMmvdConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_triangle_constraint_flag");                cinfo->setNoTriangleConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_ladf_constraint_flag");                    cinfo->setNoLadfConstraintFlag(symbol > 0 ? true : false);
#if JVET_N0276_CONSTRAINT_FLAGS
  READ_FLAG(symbol, "no_transform_skip_constraint_flag");          cinfo->setNoTransformSkipConstraintFlag(symbol > 0 ? true : false);
#endif
#if !JVET_N0276_CONSTRAINT_FLAGS
  READ_FLAG(symbol, "no_curr_pic_ref_constraint_flag");            cinfo->setNoCurrPicRefConstraintFlag(symbol > 0 ? true : false);
#endif
  READ_FLAG(symbol, "no_qp_delta_constraint_flag");                cinfo->setNoQpDeltaConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_dep_quant_constraint_flag");               cinfo->setNoDepQuantConstraintFlag(symbol > 0 ? true : false);
  READ_FLAG(symbol, "no_sign_data_hiding_constraint_flag");        cinfo->setNoSignDataHidingConstraintFlag(symbol > 0 ? true : false);
}


void HLSyntaxReader::parseProfileTierLevel(ProfileTierLevel *ptl, int maxNumSubLayersMinus1)
{
  uint32_t symbol;
  READ_CODE(7 , symbol,   "general_profile_idc"              ); ptl->setProfileIdc  (Profile::Name(symbol));
  READ_FLAG(    symbol,   "general_tier_flag"                ); ptl->setTierFlag    (symbol ? Level::HIGH : Level::MAIN);
#if JVET_N0276_CONSTRAINT_FLAGS
  READ_CODE(24 , symbol,   "general_sub_profile_idc"         ); ptl->setSubProfileIdc  (symbol);
#endif

  parseConstraintInfo( ptl->getConstraintInfo() );

  READ_CODE(8 , symbol,   "general_level_idc"                ); ptl->setLevelIdc    (Level::Name(symbol));

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
}


#else
void HLSyntaxReader::parsePTL( PTL *rpcPTL, bool profilePresentFlag, int maxNumSubLayersMinus1 )
{
  uint32_t uiCode;
  if(profilePresentFlag)
  {
    parseProfileTier(rpcPTL->getGeneralPTL(), false);
  }
  READ_CODE( 8, uiCode, "general_level_idc" );    rpcPTL->getGeneralPTL()->setLevelIdc(Level::Name(uiCode));

  for (int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    READ_FLAG( uiCode, "sub_layer_profile_present_flag[i]" ); rpcPTL->setSubLayerProfilePresentFlag(i, uiCode);
    READ_FLAG( uiCode, "sub_layer_level_present_flag[i]"   ); rpcPTL->setSubLayerLevelPresentFlag  (i, uiCode);
  }

  if (maxNumSubLayersMinus1 > 0)
  {
    for (int i = maxNumSubLayersMinus1; i < 8; i++)
    {
      READ_CODE(2, uiCode, "reserved_zero_2bits");
      CHECK(uiCode != 0, "Invalid code");
    }
  }

  for(int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if( rpcPTL->getSubLayerProfilePresentFlag(i) )
    {
      parseProfileTier(rpcPTL->getSubLayerPTL(i), true);
    }
    if(rpcPTL->getSubLayerLevelPresentFlag(i))
    {
      READ_CODE( 8, uiCode, "sub_layer_level_idc[i]" );   rpcPTL->getSubLayerPTL(i)->setLevelIdc(Level::Name(uiCode));
    }
  }
}

#if ENABLE_TRACING|| RExt__DECODER_DEBUG_BIT_STATISTICS
void HLSyntaxReader::parseProfileTier(ProfileTierLevel *ptl, const bool bIsSubLayer)
#define PTL_TRACE_TEXT(txt) bIsSubLayer?("sub_layer_" txt) : ("general_" txt)
#else
void HLSyntaxReader::parseProfileTier(ProfileTierLevel *ptl, const bool /*bIsSubLayer*/)
#define PTL_TRACE_TEXT(txt) txt
#endif
{
  uint32_t uiCode;
  READ_CODE(2 , uiCode,   PTL_TRACE_TEXT("profile_space"                   )); ptl->setProfileSpace(uiCode);
  READ_FLAG(    uiCode,   PTL_TRACE_TEXT("tier_flag"                       )); ptl->setTierFlag    (uiCode ? Level::HIGH : Level::MAIN);
  READ_CODE(5 , uiCode,   PTL_TRACE_TEXT("profile_idc"                     )); ptl->setProfileIdc  (Profile::Name(uiCode));
  for(int j = 0; j < 32; j++)
  {
    READ_FLAG(  uiCode,   PTL_TRACE_TEXT("profile_compatibility_flag[][j]" )); ptl->setProfileCompatibilityFlag(j, uiCode ? 1 : 0);
  }
  READ_FLAG(uiCode,       PTL_TRACE_TEXT("progressive_source_flag"         )); ptl->setProgressiveSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("interlaced_source_flag"          )); ptl->setInterlacedSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("non_packed_constraint_flag"      )); ptl->setNonPackedConstraintFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("frame_only_constraint_flag"      )); ptl->setFrameOnlyConstraintFlag(uiCode ? true : false);

  if (ptl->getProfileIdc() == Profile::MAINREXT           || ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
      ptl->getProfileIdc() == Profile::HIGHTHROUGHPUTREXT || ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT))
  {
    uint32_t maxBitDepth=16;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_12bit_constraint_flag"       )); if (uiCode) maxBitDepth=12;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_10bit_constraint_flag"       )); if (uiCode) maxBitDepth=10;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_8bit_constraint_flag"        )); if (uiCode) maxBitDepth=8;
    ptl->setBitDepthConstraint(maxBitDepth);
    ChromaFormat chromaFmtConstraint=CHROMA_444;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_422chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_422;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_420chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_420;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_monochrome_constraint_flag"  )); if (uiCode) chromaFmtConstraint=CHROMA_400;
    ptl->setChromaFormatConstraint(chromaFmtConstraint);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("intra_constraint_flag"           )); ptl->setIntraConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("one_picture_only_constraint_flag")); ptl->setOnePictureOnlyConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("lower_bit_rate_constraint_flag"  )); ptl->setLowerBitRateConstraintFlag(uiCode != 0);
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[16..31]"    ));
    READ_CODE(2,  uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[32..33]"    ));
  }
  else
  {
    ptl->setBitDepthConstraint( ( ptl->getProfileIdc() == Profile::MAIN10 || ptl->getProfileIdc() == Profile::NEXT ) ? 10 : 8 );
    ptl->setChromaFormatConstraint(CHROMA_420);
    ptl->setIntraConstraintFlag(false);
    ptl->setLowerBitRateConstraintFlag(true);
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[16..31]"    ));
    READ_CODE(11, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[32..42]"    ));
  }

  if ((ptl->getProfileIdc() >= Profile::MAIN && ptl->getProfileIdc() <= Profile::HIGHTHROUGHPUTREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN10) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINSTILLPICTURE) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT) )
  {
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("inbld_flag"                      )); CHECK(uiCode != 0, "Invalid code");
  }
  else
  {
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("reserved_zero_bit"               ));
  }
#undef PTL_TRACE_TEXT
}
#endif

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

#if HEVC_USE_SCALING_LISTS
/** decode quantization matrix
* \param scalingList quantization matrix information
*/
void HLSyntaxReader::parseScalingList(ScalingList* scalingList)
{
  uint32_t  code, sizeId, listId;
  bool scalingListPredModeFlag;
  //for each size
  for(sizeId = SCALING_LIST_FIRST_CODED; sizeId <= SCALING_LIST_LAST_CODED; sizeId++)
  {
    for(listId = 0; listId <  SCALING_LIST_NUM; listId++)
    {
#if JVET_N0847_SCALING_LISTS
      if (!(((sizeId == SCALING_LIST_2x2) && (listId % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) == 0)) || ((sizeId > SCALING_LIST_32x32) && (listId % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) != 0))))//2x2 luma
#else
      if ((sizeId==SCALING_LIST_32x32) && (listId%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) != 0))
      {
        int *src = scalingList->getScalingListAddress(sizeId, listId);
        const int size = std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]);
        const int *srcNextSmallerSize = scalingList->getScalingListAddress(sizeId-1, listId);
        for(int i=0; i<size; i++)
        {
          src[i] = srcNextSmallerSize[i];
        }
        scalingList->setScalingListDC(sizeId,listId,(sizeId > SCALING_LIST_8x8) ? scalingList->getScalingListDC(sizeId-1, listId) : src[0]);
      }
      else
#endif
      {
        READ_FLAG( code, "scaling_list_pred_mode_flag");
        scalingListPredModeFlag = (code) ? true : false;
        scalingList->setScalingListPredModeFlag(sizeId, listId, scalingListPredModeFlag);
        if(!scalingListPredModeFlag) //Copy Mode
        {
          READ_UVLC( code, "scaling_list_pred_matrix_id_delta");

#if JVET_N0847_SCALING_LISTS 
          if (sizeId == SCALING_LIST_64x64)
#else
          if (sizeId==SCALING_LIST_32x32)
#endif
          {
#if JVET_N0847_SCALING_LISTS
            code *= (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)); // Adjust the decoded code for this size, to cope with the missing 32x32 chroma entries.
#else
            code*=(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES); // Adjust the decoded code for this size, to cope with the missing 32x32 chroma entries.
#endif
	  }

          scalingList->setRefMatrixId (sizeId,listId,(uint32_t)((int)(listId)-(code)));
          if( sizeId > SCALING_LIST_8x8 )
          {
            scalingList->setScalingListDC(sizeId,listId,((listId == scalingList->getRefMatrixId (sizeId,listId))? 16 :scalingList->getScalingListDC(sizeId, scalingList->getRefMatrixId (sizeId,listId))));
	  }
          scalingList->processRefMatrix( sizeId, listId, scalingList->getRefMatrixId (sizeId,listId));

        }
        else //DPCM Mode
        {
          decodeScalingList(scalingList, sizeId, listId);
        }
      }
    }
  }

  return;
}

/** decode DPCM
* \param scalingList  quantization matrix information
* \param sizeId size index
* \param listId list index
*/
void HLSyntaxReader::decodeScalingList(ScalingList *scalingList, uint32_t sizeId, uint32_t listId)
{
  int i,coefNum = std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]);
  int data;
  int scalingListDcCoefMinus8 = 0;
  int nextCoef = SCALING_LIST_START_VALUE;
#if JVET_N0847_SCALING_LISTS 
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(1 << (sizeId == SCALING_LIST_2x2 ? 1 : (sizeId == SCALING_LIST_4x4 ? 2 : 3)))][gp_sizeIdxInfo->idxFrom(1 << (sizeId == SCALING_LIST_2x2 ? 1 : (sizeId == SCALING_LIST_4x4 ? 2 : 3)))];
#else
  uint32_t* scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom( 1 << ( sizeId == SCALING_LIST_FIRST_CODED ? 2 : 3 ) )][gp_sizeIdxInfo->idxFrom( 1 << ( sizeId == SCALING_LIST_FIRST_CODED ? 2 : 3 ) )];
#endif
  int *dst = scalingList->getScalingListAddress(sizeId, listId);

  if( sizeId > SCALING_LIST_8x8 )
  {
    READ_SVLC( scalingListDcCoefMinus8, "scaling_list_dc_coef_minus8");
    scalingList->setScalingListDC(sizeId,listId,scalingListDcCoefMinus8 + 8);
    nextCoef = scalingList->getScalingListDC(sizeId,listId);
  }

  for(i = 0; i < coefNum; i++)
  {
#if JVET_N0847_SCALING_LISTS
    if (sizeId == SCALING_LIST_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
    {
      dst[scan[i].idx] = 0;
      continue;
    }
#endif
    READ_SVLC( data, "scaling_list_delta_coef");
    nextCoef = (nextCoef + data + 256 ) % 256;
#if JVET_N0847_SCALING_LISTS
    dst[scan[i].idx] = nextCoef;//[scan[scanIdx].idx]
#else
    dst[scan[i]] = nextCoef;
#endif
  }
}
#endif

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

#if JVET_N0242_NON_LINEAR_ALF
int HLSyntaxReader::alfGolombDecode( const int k, const bool signed_val )
#else
int HLSyntaxReader::alfGolombDecode( const int k )
#endif
{
  uint32_t uiSymbol;
  int q = -1;
  int nr = 0;
  int m = (int)pow( 2.0, k );
  int a;

  uiSymbol = 1;
  while( uiSymbol )
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    q++;
  }

  for( a = 0; a < k; ++a )          // read out the sequential log2(M) bits
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    if( uiSymbol )
    {
      nr += 1 << a;
    }
  }
  nr += q * m;                    // add the bits and the multiple of M
#if JVET_N0242_NON_LINEAR_ALF
  if( signed_val && nr != 0 )
#else
  if( nr != 0 )
#endif
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    nr = ( uiSymbol ) ? -nr : nr;
  }
  return nr;
}

void HLSyntaxReader::alfFilter( AlfSliceParam& alfSliceParam, const bool isChroma )
{
  uint32_t code;
  if( !isChroma )
  {
    READ_FLAG( code, "alf_luma_coeff_delta_flag" );
    alfSliceParam.alfLumaCoeffDeltaFlag = code;

    if( !alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      std::memset( alfSliceParam.alfLumaCoeffFlag, true, sizeof( alfSliceParam.alfLumaCoeffFlag ) );

      if( alfSliceParam.numLumaFilters > 1 )
      {
        READ_FLAG( code, "alf_luma_coeff_delta_prediction_flag" );
        alfSliceParam.alfLumaCoeffDeltaPredictionFlag = code;
      }
      else
      {
        alfSliceParam.alfLumaCoeffDeltaPredictionFlag = 0;
      }
    }
    else
    {
      alfSliceParam.alfLumaCoeffDeltaPredictionFlag = 0;
    }
  }

  // derive maxGolombIdx
  AlfFilterShape alfShape( isChroma ? 5 : 7 );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  READ_UVLC( code, isChroma ? "alf_chroma_min_eg_order_minus1" : "alf_luma_min_eg_order_minus1" );

  int kMin = code + 1;
  static int kMinTab[MAX_NUM_ALF_COEFF];
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;
  short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;
#if JVET_N0242_NON_LINEAR_ALF
  short* clipp = isChroma ? alfSliceParam.chromaClipp : alfSliceParam.lumaClipp;
#endif

  for( int idx = 0; idx < maxGolombIdx; idx++ )
  {
    READ_FLAG( code, isChroma ? "alf_chroma_eg_order_increase_flag"  : "alf_luma_eg_order_increase_flag" );
    CHECK( code > 1, "Wrong golomb_order_increase_flag" );
    kMinTab[idx] = kMin + code;
    kMin = kMinTab[idx];
  }

  if( !isChroma )
  {
    if( alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      for( int ind = 0; ind < alfSliceParam.numLumaFilters; ++ind )
      {
        READ_FLAG( code, "alf_luma_coeff_flag[i]" );
        alfSliceParam.alfLumaCoeffFlag[ind] = code;
      }
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfSliceParam.alfLumaCoeffFlag[ind] && alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      memset( coeff + ind * MAX_NUM_ALF_LUMA_COEFF, 0, sizeof( *coeff ) * alfShape.numCoeff );
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode( kMinTab[alfShape.golombIdx[i]] );
    }
  }
#if JVET_N0242_NON_LINEAR_ALF

  // Clipping values coding
  if ( alfSliceParam.nonLinearFlag[isChroma] )
  {
    READ_UVLC( code, "clip_min_golomb_order" );

    kMin = code + 1;

    for( int idx = 0; idx < maxGolombIdx; idx++ )
    {
      READ_FLAG( code, "clip_golomb_order_increase_flag" );
      CHECK( code > 1, "Wrong golomb_order_increase_flag" );
      kMinTab[idx] = kMin + code;
      kMin = kMinTab[idx];
    }

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

    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {
      if( !isChroma && !alfSliceParam.alfLumaCoeffFlag[ind] && alfSliceParam.alfLumaCoeffDeltaFlag )
      {
        std::fill_n( clipp + ind * MAX_NUM_ALF_LUMA_COEFF, alfShape.numCoeff, 0 );
        continue;
      }

      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        if( recCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] )
          clipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode( kMinTab[alfShape.golombIdx[i]], false );
        else
          clipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = 0;
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
#endif
}

int HLSyntaxReader::truncatedUnaryEqProb( const int maxSymbol )
{
  for( int k = 0; k < maxSymbol; k++ )
  {
    uint32_t symbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( symbol, "" );
#else
    xReadFlag( symbol );
#endif

    if( !symbol )
    {
      return k;
    }
  }
  return maxSymbol;
}

void HLSyntaxReader::xReadTruncBinCode( uint32_t& ruiSymbol, const int uiMaxSymbol )
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
  int b = uiMaxSymbol - uiVal;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadCode( uiThresh, ruiSymbol, "" );
#else
  xReadCode( uiThresh, ruiSymbol );
#endif
  if( ruiSymbol >= uiVal - b )
  {
    uint32_t uiSymbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    ruiSymbol <<= 1;
    ruiSymbol += uiSymbol;
    ruiSymbol -= ( uiVal - b );
  }
}

//! \}

