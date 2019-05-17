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

/**
 \file     NALread.cpp
 \brief    reading functionality for NAL units
 */


#include <vector>
#include <algorithm>
#include <ostream>

#include "NALread.h"

#include "CommonLib/NAL.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Rom.h"
#include "CommonLib/dtrace_next.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

using namespace std;

//! \ingroup DecoderLib
//! \{
static void convertPayloadToRBSP(vector<uint8_t>& nalUnitBuf, InputBitstream *bitstream, bool isVclNalUnit)
{
  uint32_t zeroCount = 0;
  vector<uint8_t>::iterator it_read, it_write;

  uint32_t pos = 0;
  bitstream->clearEmulationPreventionByteLocation();
  for (it_read = it_write = nalUnitBuf.begin(); it_read != nalUnitBuf.end(); it_read++, it_write++, pos++)
  {
    CHECK(zeroCount >= 2 && *it_read < 0x03, "Zero count is '2' and read value is small than '3'");
    if (zeroCount == 2 && *it_read == 0x03)
    {
      bitstream->pushEmulationPreventionByteLocation( pos );
      pos++;
      it_read++;
      zeroCount = 0;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
      CodingStatistics::IncrementStatisticEP(STATS__EMULATION_PREVENTION_3_BYTES, 8, 0);
#endif
      if (it_read == nalUnitBuf.end())
      {
        break;
      }
      CHECK(*it_read > 0x03, "Read a value bigger than '3'");
    }
    zeroCount = (*it_read == 0x00) ? zeroCount+1 : 0;
    *it_write = *it_read;
  }
  CHECK(zeroCount != 0, "Zero count not '0'");

  if (isVclNalUnit)
  {
    // Remove cabac_zero_word from payload if present
    int n = 0;

    while (it_write[-1] == 0x00)
    {
      it_write--;
      n++;
    }

    if (n > 0)
    {
      msg( NOTICE, "\nDetected %d instances of cabac_zero_word\n", n/2);
    }
  }

  nalUnitBuf.resize(it_write - nalUnitBuf.begin());
}

#if ENABLE_TRACING
static void xTraceNalUnitHeader(InputNALUnit& nalu)
{
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "*********** NAL UNIT (%s) ***********\n", nalUnitTypeToString(nalu.m_nalUnitType) );
  #if JVET_N0067_NAL_Unit_Header
  bool zeroTidRequiredFlag = 0;
  if((nalu.m_nalUnitType >= 16) && (nalu.m_nalUnitType <= 31)) {
    zeroTidRequiredFlag = 1;
  }
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "zero_tid_required_flag", 1, zeroTidRequiredFlag );
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_temporal_id_plus1", 3, nalu.m_temporalId + 1 );
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nal_unit_type_lsb", 4, (nalu.m_nalUnitType) - (zeroTidRequiredFlag << 4));
#if EMULATION_PREVENTION_FIX
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_layer_id_plus1", 7, nalu.m_nuhLayerId+1); 
#else
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_layer_id", 7, nalu.m_nuhLayerId );
#endif
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_reserved_zero_bit", 1, 0 );
  #else
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "forbidden_zero_bit", 1, 0 );
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nal_unit_type", 6, nalu.m_nalUnitType );
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_layer_id", 6, nalu.m_nuhLayerId );
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_temporal_id_plus1", 3, nalu.m_temporalId + 1 );
  #endif
}
#endif

void readNalUnitHeader(InputNALUnit& nalu)
{
  InputBitstream& bs = nalu.getBitstream();

#if JVET_N0067_NAL_Unit_Header
  bool zeroTidRequiredFlag = bs.read(1);              // zero_tid_required_flag
  nalu.m_temporalId = bs.read(3) - 1;                 // nuh_temporal_id_plus1
  //When zero_tid_required_flag is equal to 1, the value of nuh_temporal_id_plus1 shall be equal to 1.
  CHECK((zeroTidRequiredFlag == 1) && (nalu.m_temporalId != 0), "Temporal ID is not '0' when zero tid is required.");
  uint32_t nalUnitTypeLsb = bs.read(4);             // nal_unit_type_lsb
  nalu.m_nalUnitType = (NalUnitType) ((zeroTidRequiredFlag << 4) + nalUnitTypeLsb);
  nalu.m_nuhLayerId = bs.read(7);                     // nuh_layer_id 
#if EMULATION_PREVENTION_FIX
  CHECK (nalu.m_nuhLayerId == 0, "nuh_layer_id_plus1 must be greater than zero");
  nalu.m_nuhLayerId--; 
  CHECK(nalu.m_nuhLayerId > 125, "Layer ID out of range");
#else
  CHECK(nalu.m_nuhLayerId > 126, "Layer ID out of range");
#endif
  uint32_t nuh_reserved_zero_bit = bs.read(1);        // nuh_reserved_zero_bit
  CHECK(nuh_reserved_zero_bit != 0, "Reserved zero bit is not '0'");
#else
  bool forbidden_zero_bit = bs.read(1);           // forbidden_zero_bit
  if(forbidden_zero_bit != 0) { THROW( "Forbidden zero-bit not '0'" );}
  nalu.m_nalUnitType = (NalUnitType) bs.read(6);  // nal_unit_type
  nalu.m_nuhLayerId = bs.read(6);                 // nuh_layer_id
  nalu.m_temporalId = bs.read(3) - 1;             // nuh_temporal_id_plus1
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#if JVET_N0067_NAL_Unit_Header
  CodingStatistics::IncrementStatisticEP(STATS__NAL_UNIT_HEADER_BITS, 1+3+4+7+1, 0);
#else
  CodingStatistics::IncrementStatisticEP(STATS__NAL_UNIT_HEADER_BITS, 1+6+6+3, 0);
#endif
#endif

#if ENABLE_TRACING
  xTraceNalUnitHeader(nalu);
#endif

  // only check these rules for base layer
  if (nalu.m_nuhLayerId == 0)
  {
    if ( nalu.m_temporalId )
    {
#if JVET_N0067_NAL_Unit_Header
      CHECK(  
           (uint32_t)nalu.m_nalUnitType >= 16
        && (uint32_t)nalu.m_nalUnitType <= 31
            , "Invalid NAL type" );
#else
#if HEVC_VPS
#if !JVET_M0101_HLS
      CHECK(  nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA
           || nalu.m_nalUnitType == NAL_UNIT_VPS
           || nalu.m_nalUnitType == NAL_UNIT_SPS
           || nalu.m_nalUnitType == NAL_UNIT_EOS
           || nalu.m_nalUnitType == NAL_UNIT_EOB
            , "Invalid NAL type" );
#else
      CHECK(  nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA
           || nalu.m_nalUnitType == NAL_UNIT_VPS
           || nalu.m_nalUnitType == NAL_UNIT_SPS
           || nalu.m_nalUnitType == NAL_UNIT_EOS
           || nalu.m_nalUnitType == NAL_UNIT_EOB
           , "Invalid NAL type" );
#endif
#else
#if !JVET_M0101_HLS
      CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA
           || nalu.m_nalUnitType == NAL_UNIT_SPS
           || nalu.m_nalUnitType == NAL_UNIT_EOS
           || nalu.m_nalUnitType == NAL_UNIT_EOB
           , "Invalid NAL type");
#else
      CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
         || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
         || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA
         || nalu.m_nalUnitType == NAL_UNIT_SPS
         || nalu.m_nalUnitType == NAL_UNIT_EOS
         || nalu.m_nalUnitType == NAL_UNIT_EOB
         , "Invalid NAL type");
#endif
#endif
#endif
    }
    else
    {
#if JVET_N0067_NAL_Unit_Header
      CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA
         , "Invalid NAL type");
#else
#if !JVET_M0101_HLS
      CHECK(  nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TSA_R
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TSA_N
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA_R
           || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA_N
            , "Invalid NAL type" );
#else
      CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA
         , "Invalid NAL type");
#endif
#endif
    }
  }
}
/**
 * create a NALunit structure with given header values and storage for
 * a bitstream
 */
void read(InputNALUnit& nalu)
{
  InputBitstream &bitstream = nalu.getBitstream();
  vector<uint8_t>& nalUnitBuf=bitstream.getFifo();
  // perform anti-emulation prevention
  convertPayloadToRBSP(nalUnitBuf, &bitstream, (nalUnitBuf[0] & 64) == 0);
  bitstream.resetToStart();
  readNalUnitHeader(nalu);
}
//! \}
