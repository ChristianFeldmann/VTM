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
 \file     SEIread.h
 \brief    reading funtionality for SEI messages
 */

#ifndef __SEIREAD__
#define __SEIREAD__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//! \ingroup DecoderLib
//! \{

#include "CommonLib/SEI.h"
class InputBitstream;


class SEIReader: public VLCReader
{
public:
  SEIReader() {};
  virtual ~SEIReader() {};
  void parseSEImessage(InputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t temporalId, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);

protected:
  void xReadSEImessage                        (SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t temporalId, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);
#if HEVC_SEI
#if !JVET_P0337_PORTING_SEI
  void xParseSEIuserDataUnregistered          (SEIuserDataUnregistered &sei,          uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEIActiveParameterSets           (SEIActiveParameterSets  &sei,          uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
  void xParseSEIDecodingUnitInfo              (SEIDecodingUnitInfo& sei,              uint32_t payloadSize, const SEIBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream);
#else
  void xParseSEIDecodingUnitInfo              (SEIDecodingUnitInfo& sei,              uint32_t payloadSize, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEIDecodedPictureHash            (SEIDecodedPictureHash& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
  void xParseSEIBufferingPeriod               (SEIBufferingPeriod& sei,               uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIPictureTiming                 (SEIPictureTiming& sei,                 uint32_t payloadSize, const uint32_t temporalId, const SEIBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream);
#else
  void xParseSEIBufferingPeriod               (SEIBufferingPeriod& sei,               uint32_t payloadSize, const SPS *sps, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIPictureTiming                 (SEIPictureTiming& sei,                 uint32_t payloadSize, const SPS *sps, const uint32_t temporalId, const SEIBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEIFrameFieldinfo                (SEIFrameFieldInfo& sei,                 uint32_t payloadSize,                    std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDependentRAPIndication        (SEIDependentRAPIndication& sei,        uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
#if HEVC_SEI
  void xParseSEIRecoveryPoint                 (SEIRecoveryPoint& sei,                 uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#if !JVET_P0337_PORTING_SEI
  void xParseSEIFramePacking                  (SEIFramePacking& sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEISegmentedRectFramePacking     (SEISegmentedRectFramePacking& sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDisplayOrientation            (SEIDisplayOrientation &sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEITemporalLevel0Index           (SEITemporalLevel0Index &sei,           uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIRegionRefreshInfo             (SEIGradualDecodingRefreshInfo &sei,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEINoDisplay                     (SEINoDisplay &sei,                     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIToneMappingInfo               (SEIToneMappingInfo& sei,               uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISOPDescription                (SEISOPDescription &sei,                uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIScalableNesting               (SEIScalableNesting& sei, const NalUnitType nalUnitType, uint32_t payloadSize, const SPS *sps, std::ostream *pDecodedMessageOutputStream);
  void xParseSEITempMotionConstraintsTileSets (SEITempMotionConstrainedTileSets& sei, uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEITimeCode                      (SEITimeCode& sei,                      uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIChromaResamplingFilterHint    (SEIChromaResamplingFilterHint& sei,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIKneeFunctionInfo              (SEIKneeFunctionInfo& sei,              uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#if !JVET_P0337_PORTING_SEI
  void xParseSEIMasteringDisplayColourVolume  (SEIMasteringDisplayColourVolume& sei,  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEIColourRemappingInfo           (SEIColourRemappingInfo& sei,           uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#if !JVET_P0337_PORTING_SEI
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void xParseSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics& sei,              uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
#endif
  void xParseSEIGreenMetadataInfo             (SEIGreenMetadataInfo& sei,             uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
#if JVET_P0462_SEI360
  void xParseSEIEquirectangularProjection     (SEIEquirectangularProjection &sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISphereRotation                (SEISphereRotation &sei,                uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIOmniViewport                  (SEIOmniViewport& sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIRegionWisePacking             (SEIRegionWisePacking& sei,             uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
#if JVET_P0450_SEI_SARI
  void xParseSEISampleAspectRatioInfo         (SEISampleAspectRatioInfo& sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
#if JVET_P0337_PORTING_SEI
  void xParseSEIUserDataRegistered            (SEIUserDataRegistered& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIuserDataUnregistered          (SEIuserDataUnregistered &sei,          uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFilmGrainCharacteristics      (SEIFilmGrainCharacteristics& sei,      uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFramePacking                  (SEIFramePacking& sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIMasteringDisplayColourVolume  (SEIMasteringDisplayColourVolume& sei,  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentLightLevelInfo         (SEIContentLightLevelInfo& sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics& sei,              uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIAmbientViewingEnvironment     (SEIAmbientViewingEnvironment& sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentColourVolume           (SEIContentColourVolume& sei,           uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif

#if JVET_P0462_SEI360 || JVET_P0337_PORTING_SEI
  void sei_read_scode(std::ostream *pOS, uint32_t length, int& code, const char *pSymbolName);
#endif
  void sei_read_code(std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_uvlc(std::ostream *pOS,                uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_svlc(std::ostream *pOS,                int&  ruiCode, const char *pSymbolName);
  void sei_read_flag(std::ostream *pOS,                uint32_t& ruiCode, const char *pSymbolName);
};


//! \}

#endif
