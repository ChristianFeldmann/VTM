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

#pragma once

#ifndef __SEIWRITE__
#define __SEIWRITE__

#include "VLCWriter.h"
#include "CommonLib/SEI.h"

class OutputBitstream;

//! \ingroup EncoderLib
//! \{
class SEIWriter : public VLCWriter
{
public:
  SEIWriter() {};
  virtual ~SEIWriter() {};

#if JVET_N0353_INDEP_BUFF_TIME_SEI
#if !JVET_N0867_TEMP_SCAL_HRD
  void writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, const SPS *sps, HRD &hrd, bool isNested);
#else
  void writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, const SPS *sps, HRD &hrd, bool isNested, const uint32_t temporalId);
#endif
#else
  void writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, const SPS *sps, bool isNested);
#endif

protected:
#if HEVC_SEI
  void xWriteSEIuserDataUnregistered(const SEIuserDataUnregistered &sei);
  void xWriteSEIActiveParameterSets(const SEIActiveParameterSets& sei);
#endif
#if JVET_O0189_DU
  void xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SPS *sps, HRD &hrd);
#else
  void xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SPS *sps);
#endif
  void xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei);
#if !JVET_N0353_INDEP_BUFF_TIME_SEI
  void xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei, const SPS *sps);
  void xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SPS *sps);
#else
#if JVET_O0189_DU
  void xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei, const SPS *sps);
#else
  void xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei);
#endif
#if JVET_O0189_DU
#if !JVET_N0867_TEMP_SCAL_HRD
  void xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SPS *sps, const SEIBufferingPeriod& bp);
#else
  void xWriteSEIPictureTiming(const SEIPictureTiming& sei,const SPS *sps, const SEIBufferingPeriod& bp, const uint32_t temporalId);
#endif
#else
  void xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SEIBufferingPeriod& bp);
#endif
#endif
#if JVET_O0041_FRAME_FIELD_SEI
  void xWriteSEIFrameFieldInfo(const SEIFrameFieldInfo& sei);
#endif
#if JVET_N0494_DRAP
  void xWriteSEIDependentRAPIndication(const SEIDependentRAPIndication& sei);
#endif
#if HEVC_SEI
  void xWriteSEIRecoveryPoint(const SEIRecoveryPoint& sei);
  void xWriteSEIFramePacking(const SEIFramePacking& sei);
  void xWriteSEISegmentedRectFramePacking(const SEISegmentedRectFramePacking& sei);
  void xWriteSEIDisplayOrientation(const SEIDisplayOrientation &sei);
  void xWriteSEITemporalLevel0Index(const SEITemporalLevel0Index &sei);
  void xWriteSEIGradualDecodingRefreshInfo(const SEIGradualDecodingRefreshInfo &sei);
  void xWriteSEINoDisplay(const SEINoDisplay &sei);
  void xWriteSEIToneMappingInfo(const SEIToneMappingInfo& sei);
  void xWriteSEISOPDescription(const SEISOPDescription& sei);
  void xWriteSEIScalableNesting(OutputBitstream& bs, const SEIScalableNesting& sei, const SPS *sps);
  void xWriteSEITempMotionConstrainedTileSets(const SEITempMotionConstrainedTileSets& sei);
  void xWriteSEITimeCode(const SEITimeCode& sei);
  void xWriteSEIChromaResamplingFilterHint(const SEIChromaResamplingFilterHint& sei);
  void xWriteSEIKneeFunctionInfo(const SEIKneeFunctionInfo &sei);
  void xWriteSEIColourRemappingInfo(const SEIColourRemappingInfo& sei);
  void xWriteSEIMasteringDisplayColourVolume( const SEIMasteringDisplayColourVolume& sei);
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei);
#endif
  void xWriteSEIGreenMetadataInfo(const SEIGreenMetadataInfo &sei);
#endif
  
#if !JVET_N0353_INDEP_BUFF_TIME_SEI
  void xWriteSEIpayloadData(OutputBitstream& bs, const SEI& sei, const SPS *sps);
#else
#if !JVET_N0867_TEMP_SCAL_HRD
  void xWriteSEIpayloadData(OutputBitstream& bs, const SEI& sei, const SPS *sps, HRD &hrd);
#else
  void xWriteSEIpayloadData(OutputBitstream& bs, const SEI& sei, const SPS *sps, HRD &hrd, const uint32_t temporalId);
#endif
#endif
  void xWriteByteAlign();
};

//! \}

#endif
