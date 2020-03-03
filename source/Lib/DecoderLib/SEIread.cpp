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

/**
 \file     SEIread.cpp
 \brief    reading funtionality for SEI messages
 */

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Slice.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"
#include <iomanip>


//! \ingroup DecoderLib
//! \{

void SEIReader::sei_read_scode(std::ostream *pOS, uint32_t length, int& code, const char *pSymbolName)
{
  READ_SCODE(length, code, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << code << "\n";
  }
}

void SEIReader::sei_read_code(std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_CODE(uiLength, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_uvlc(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_UVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_svlc(std::ostream *pOS, int& ruiCode, const char *pSymbolName)
{
  READ_SVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_flag(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_FLAG(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << (ruiCode?1:0) << "\n";
  }
}

static inline void output_sei_message_header(SEI &sei, std::ostream *pDecodedMessageOutputStream, uint32_t payloadSize)
{
  if (pDecodedMessageOutputStream)
  {
    std::string seiMessageHdr(SEI::getSEIMessageString(sei.payloadType())); seiMessageHdr+=" SEI message";
    (*pDecodedMessageOutputStream) << std::setfill('-') << std::setw(seiMessageHdr.size()) << "-" << std::setfill(' ') << "\n" << seiMessageHdr << " (" << payloadSize << " bytes)"<< "\n";
  }
}

#undef READ_CODE
#undef READ_SCODE
#undef READ_SVLC
#undef READ_UVLC
#undef READ_FLAG


/**
 * unmarshal a single SEI message from bitstream bs
 */
 // note: for independent parsing no parameter set should not be required here
void SEIReader::parseSEImessage(InputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t temporalId, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream)
{
  setBitstream(bs);

  CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");
  do
  {
    xReadSEImessage(seis, nalUnitType, temporalId, sps, hrd, pDecodedMessageOutputStream);

    /* SEI messages are an integer number of bytes, something has failed
    * in the parsing if bitstream not byte-aligned */
    CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");
  }
  while (m_pcBitstream->getNumBitsLeft() > 8);

  xReadRbspTrailingBits();
}

void SEIReader::xReadSEImessage(SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t temporalId, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream)
{
#if ENABLE_TRACING
  xTraceSEIHeader();
#endif
  int payloadType = 0;
  uint32_t val = 0;

  do
  {
    sei_read_code(NULL, 8, val, "payload_type");
    payloadType += val;
  } while (val==0xFF);

  uint32_t payloadSize = 0;
  do
  {
    sei_read_code(NULL, 8, val, "payload_size");
    payloadSize += val;
  } while (val==0xFF);

#if ENABLE_TRACING
  xTraceSEIMessageType((SEI::PayloadType)payloadType);
#endif

  /* extract the payload for this single SEI message.
   * This allows greater safety in erroneous parsing of an SEI message
   * from affecting subsequent messages.
   * After parsing the payload, bs needs to be restored as the primary
   * bitstream.
   */
  InputBitstream *bs = getBitstream();
  setBitstream(bs->extractSubstream(payloadSize * 8));

  SEI *sei = NULL;
  const SEIBufferingPeriod *bp = NULL;
#if JVET_Q0818_PT_SEI
  const SEIPictureTiming *pt = NULL;
#endif

  if(nalUnitType == NAL_UNIT_PREFIX_SEI)
  {
    switch (payloadType)
    {
    case SEI::USER_DATA_UNREGISTERED:
      sei = new SEIuserDataUnregistered;
      xParseSEIuserDataUnregistered((SEIuserDataUnregistered&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DECODING_UNIT_INFO:
      bp = hrd.getBufferingPeriodSEI();
      if (!bp)
      {
        msg( WARNING, "Warning: Found Decoding unit information SEI message, but no active buffering period is available. Ignoring.");
      }
      else
      {
        sei = new SEIDecodingUnitInfo;
        xParseSEIDecodingUnitInfo((SEIDecodingUnitInfo&) *sei, payloadSize, *bp, temporalId, pDecodedMessageOutputStream);
      }
      break;
    case SEI::BUFFERING_PERIOD:
      sei = new SEIBufferingPeriod;
      xParseSEIBufferingPeriod((SEIBufferingPeriod&) *sei, payloadSize, pDecodedMessageOutputStream);
      hrd.setBufferingPeriodSEI((SEIBufferingPeriod*) sei);
      break;
    case SEI::PICTURE_TIMING:
      {
        bp = hrd.getBufferingPeriodSEI();
        if (!bp)
        {
          msg( WARNING, "Warning: Found Picture timing SEI message, but no active buffering period is available. Ignoring.");
        }
        else
        {
          sei = new SEIPictureTiming;
          xParseSEIPictureTiming((SEIPictureTiming&)*sei, payloadSize, temporalId, *bp, pDecodedMessageOutputStream);
#if JVET_Q0818_PT_SEI
          hrd.setPictureTimingSEI( (SEIPictureTiming*) sei );
#endif
        }
      }
      break;
#if JVET_P0190_SCALABLE_NESTING_SEI
    case SEI::SCALABLE_NESTING:
      sei = new SEIScalableNesting;
      xParseSEIScalableNesting((SEIScalableNesting&)*sei, nalUnitType, payloadSize, sps, pDecodedMessageOutputStream);
      break;
#endif
    case SEI::FRAME_FIELD_INFO:
      sei = new SEIFrameFieldInfo;
#if JVET_Q0818_PT_SEI
      pt = hrd.getPictureTimingSEI();
      xParseSEIFrameFieldinfo((SEIFrameFieldInfo&) *sei, *pt, payloadSize, pDecodedMessageOutputStream);
#else
      xParseSEIFrameFieldinfo((SEIFrameFieldInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
#endif
      break;
    case SEI::DEPENDENT_RAP_INDICATION:
      sei = new SEIDependentRAPIndication;
      xParseSEIDependentRAPIndication((SEIDependentRAPIndication&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FRAME_PACKING:
      sei = new SEIFramePacking;
      xParseSEIFramePacking((SEIFramePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
      sei = new SEIMasteringDisplayColourVolume;
      xParseSEIMasteringDisplayColourVolume((SEIMasteringDisplayColourVolume&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
    case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
      sei = new SEIAlternativeTransferCharacteristics;
      xParseSEIAlternativeTransferCharacteristics((SEIAlternativeTransferCharacteristics&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
    case SEI::EQUIRECTANGULAR_PROJECTION:
      sei = new SEIEquirectangularProjection;
      xParseSEIEquirectangularProjection((SEIEquirectangularProjection&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SPHERE_ROTATION:
      sei = new SEISphereRotation;
      xParseSEISphereRotation((SEISphereRotation&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::OMNI_VIEWPORT:
      sei = new SEIOmniViewport;
      xParseSEIOmniViewport((SEIOmniViewport&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::REGION_WISE_PACKING:
      sei = new SEIRegionWisePacking;
      xParseSEIRegionWisePacking((SEIRegionWisePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::GENERALIZED_CUBEMAP_PROJECTION:
      sei = new SEIGeneralizedCubemapProjection;
      xParseSEIGeneralizedCubemapProjection((SEIGeneralizedCubemapProjection&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SUBPICTURE_LEVEL_INFO:
      sei = new SEISubpicureLevelInfo;
      xParseSEISubpictureLevelInfo((SEISubpicureLevelInfo&) *sei, sps, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SAMPLE_ASPECT_RATIO_INFO:
      sei = new SEISampleAspectRatioInfo;
      xParseSEISampleAspectRatioInfo((SEISampleAspectRatioInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:
      sei = new SEIUserDataRegistered;
      xParseSEIUserDataRegistered((SEIUserDataRegistered&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FILM_GRAIN_CHARACTERISTICS:
      sei = new SEIFilmGrainCharacteristics;
      xParseSEIFilmGrainCharacteristics((SEIFilmGrainCharacteristics&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_LIGHT_LEVEL_INFO:
      sei = new SEIContentLightLevelInfo;
      xParseSEIContentLightLevelInfo((SEIContentLightLevelInfo&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::AMBIENT_VIEWING_ENVIRONMENT:
      sei = new SEIAmbientViewingEnvironment;
      xParseSEIAmbientViewingEnvironment((SEIAmbientViewingEnvironment&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_COLOUR_VOLUME:
      sei = new SEIContentColourVolume;
      xParseSEIContentColourVolume((SEIContentColourVolume&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    default:
      for (uint32_t i = 0; i < payloadSize; i++)
      {
        uint32_t seiByte;
        sei_read_code (NULL, 8, seiByte, "unknown prefix SEI payload byte");
      }
      msg( WARNING, "Unknown prefix SEI message (payloadType = %d) was found!\n", payloadType);
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "Unknown prefix SEI message (payloadType = " << payloadType << ") was found!\n";
      }
      break;
    }
  }
  else
  {
    switch (payloadType)
    {
      case SEI::USER_DATA_UNREGISTERED:
        sei = new SEIuserDataUnregistered;
        xParseSEIuserDataUnregistered((SEIuserDataUnregistered&) *sei, payloadSize, pDecodedMessageOutputStream);
        break;
      case SEI::DECODED_PICTURE_HASH:
        sei = new SEIDecodedPictureHash;
        xParseSEIDecodedPictureHash((SEIDecodedPictureHash&) *sei, payloadSize, pDecodedMessageOutputStream);
        break;
      default:
        for (uint32_t i = 0; i < payloadSize; i++)
        {
          uint32_t seiByte;
          sei_read_code( NULL, 8, seiByte, "unknown suffix SEI payload byte");
        }
        msg( WARNING, "Unknown suffix SEI message (payloadType = %d) was found!\n", payloadType);
        if (pDecodedMessageOutputStream)
        {
          (*pDecodedMessageOutputStream) << "Unknown suffix SEI message (payloadType = " << payloadType << ") was found!\n";
        }
        break;
    }
  }

  if (sei != NULL)
  {
    seis.push_back(sei);
  }

  /* By definition the underlying bitstream terminates in a byte-aligned manner.
   * 1. Extract all bar the last MIN(bitsremaining,nine) bits as reserved_payload_extension_data
   * 2. Examine the final 8 bits to determine the payload_bit_equal_to_one marker
   * 3. Extract the remainingreserved_payload_extension_data bits.
   *
   * If there are fewer than 9 bits available, extract them.
   */
  int payloadBitsRemaining = getBitstream()->getNumBitsLeft();
  if (payloadBitsRemaining) /* more_data_in_payload() */
  {
    for (; payloadBitsRemaining > 9; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_code ( pDecodedMessageOutputStream, 1, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    /* 2 */
    int finalBits = getBitstream()->peekBits(payloadBitsRemaining);
    int finalPayloadBits = 0;
    for (int mask = 0xff; finalBits & (mask >> finalPayloadBits); finalPayloadBits++)
    {
      continue;
    }

    /* 3 */
    for (; payloadBitsRemaining > 9 - finalPayloadBits; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_flag ( 0, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    uint32_t dummy;
    sei_read_flag( 0, dummy, "payload_bit_equal_to_one"); payloadBitsRemaining--;
    while (payloadBitsRemaining)
    {
      sei_read_flag( 0, dummy, "payload_bit_equal_to_zero"); payloadBitsRemaining--;
    }
  }

  /* restore primary bitstream for sei_message */
  delete getBitstream();
  setBitstream(bs);
}

/**
 * parse bitstream bs and unpack a user_data_unregistered SEI message
 * of payloasSize bytes into sei.
 */

void SEIReader::xParseSEIuserDataUnregistered(SEIuserDataUnregistered &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  CHECK(payloadSize < ISO_IEC_11578_LEN, "Payload too small");
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  for (uint32_t i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 8, val, "uuid_iso_iec_11578");
    sei.uuid_iso_iec_11578[i] = val;
  }

  sei.userDataLength = payloadSize - ISO_IEC_11578_LEN;
  if (!sei.userDataLength)
  {
    sei.userData = 0;
    return;
  }

  sei.userData = new uint8_t[sei.userDataLength];
  for (uint32_t i = 0; i < sei.userDataLength; i++)
  {
    sei_read_code( NULL, 8, val, "user_data_payload_byte" );
    sei.userData[i] = val;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  User data payload size: " << sei.userDataLength << "\n";
  }
}

/**
 * parse bitstream bs and unpack a decoded picture hash SEI message
 * of payloadSize bytes into sei.
 */
void SEIReader::xParseSEIDecodedPictureHash(SEIDecodedPictureHash& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t bytesRead = 0;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream, 8, val, "hash_type");
  sei.method = static_cast<HashType>(val); bytesRead++;

  const char *traceString="\0";
  switch (sei.method)
  {
    case HASHTYPE_MD5: traceString="picture_md5"; break;
    case HASHTYPE_CRC: traceString="picture_crc"; break;
    case HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: THROW("Unknown hash type"); break;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  " << std::setw(55) << traceString << ": " << std::hex << std::setfill('0');
  }

  sei.m_pictureHash.hash.clear();
  for(;bytesRead < payloadSize; bytesRead++)
  {
    sei_read_code( NULL, 8, val, traceString);
    sei.m_pictureHash.hash.push_back((uint8_t)val);
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << std::setw(2) << val;
    }
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << std::dec << std::setfill(' ') << "\n";
  }
}


#if JVET_P0190_SCALABLE_NESTING_SEI
void SEIReader::xParseSEIScalableNesting(SEIScalableNesting& sei, const NalUnitType nalUnitType, uint32_t payloadSize, const SPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t uiCode;
  SEIMessages seis;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, uiCode, "nesting_ols_flag"); sei.m_nestingOlsFlag = uiCode;
  if (sei.m_nestingOlsFlag)
  {
    sei_read_uvlc(pDecodedMessageOutputStream, uiCode, "nesting_num_olss_minus1"); sei.m_nestingNumOlssMinus1 = uiCode;
    for (uint32_t i = 0; i <= sei.m_nestingNumOlssMinus1; i++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, uiCode, "nesting_ols_idx_delta_minus1[i]"); sei.m_nestingOlsIdxDeltaMinus1[i] = uiCode;
    }
  }
  else
  {
    sei_read_flag(pDecodedMessageOutputStream, uiCode, "nesting_all_layers_flag"); sei.m_nestingAllLayersFlag = uiCode;
    if (!sei.m_nestingAllLayersFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, uiCode, "nesting_num_layers_minus1"); sei.m_nestingNumLayersMinus1 = uiCode;
      for (uint32_t i = 0; i <= sei.m_nestingNumLayersMinus1; i++)
      {
        sei_read_code(pDecodedMessageOutputStream, 6, uiCode, "nesting_layer_id[i]"); sei.m_nestingLayerId[i] = uiCode;
      }
    }
  }

  // byte alignment
  while (m_pcBitstream->getNumBitsRead() % 8 != 0)
  {
    uint32_t code;
    sei_read_flag(pDecodedMessageOutputStream, code, "nesting_zero_bit");
  }

  // read nested SEI messages
  do
  {
    HRD hrd;
    xReadSEImessage(sei.m_nestedSEIs, nalUnitType, 0, sps, hrd, pDecodedMessageOutputStream);
  } while (m_pcBitstream->getNumBitsLeft() > 8);

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "End of scalable nesting SEI message\n";
  }
}
#endif
void SEIReader::xParseSEIDecodingUnitInfo(SEIDecodingUnitInfo& sei, uint32_t payloadSize, const SEIBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_uvlc( pDecodedMessageOutputStream, val, "decoding_unit_idx");
  sei.m_decodingUnitIdx = val;

  if(!bp.m_decodingUnitCpbParamsInPicTimingSeiFlag)
  {
    for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
    {
      sei_read_flag( pDecodedMessageOutputStream, val, "dui_sub_layer_delays_present_flag[i]" );
      sei.m_duiSubLayerDelaysPresentFlag[i] = val;
      if( sei.m_duiSubLayerDelaysPresentFlag[i] )
      {
        sei_read_code( pDecodedMessageOutputStream, bp.getDuCpbRemovalDelayIncrementLength(), val, "du_spt_cpb_removal_delay_increment[i]");
        sei.m_duSptCpbRemovalDelayIncrement[i] = val;
      }
      else
      {
        sei.m_duSptCpbRemovalDelayIncrement[i] = 0;
      }
    }
  }
  else
  {
    for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
    {
      sei.m_duSptCpbRemovalDelayIncrement[i] = 0;
    }
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "dpb_output_du_delay_present_flag"); sei.m_dpbOutputDuDelayPresentFlag = (val != 0);
  if(sei.m_dpbOutputDuDelayPresentFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, bp.getDpbOutputDelayDuLength(), val, "pic_spt_dpb_output_du_delay");
    sei.m_picSptDpbOutputDuDelay = val;
  }
}

void SEIReader::xParseSEIBufferingPeriod(SEIBufferingPeriod& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int i, nalOrVcl;
  uint32_t code;


  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, code, "bp_nal_hrd_parameters_present_flag" );               sei.m_bpNalCpbParamsPresentFlag = code;
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_vcl_hrd_parameters_present_flag" );               sei.m_bpVclCpbParamsPresentFlag = code;

  sei_read_code( pDecodedMessageOutputStream, 5, code, "initial_cpb_removal_delay_length_minus1" );     sei.m_initialCpbRemovalDelayLength = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "cpb_removal_delay_length_minus1" );             sei.m_cpbRemovalDelayLength        = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_length_minus1" );              sei.m_dpbOutputDelayLength         = code + 1;
  sei_read_flag( pDecodedMessageOutputStream, code, "alt_cpb_params_present_flag");                     sei.m_altCpbParamsPresentFlag      = code;
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_decoding_unit_hrd_params_present_flag" );       sei.m_bpDecodingUnitHrdParamsPresentFlag = code;
  if( sei.m_bpDecodingUnitHrdParamsPresentFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 5, code, "du_cpb_removal_delay_increment_length_minus1" );  sei.m_duCpbRemovalDelayIncrementLength = code + 1;
    sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_du_length_minus1" );             sei.m_dpbOutputDelayDuLength = code + 1;
    sei_read_flag( pDecodedMessageOutputStream, code, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );  sei.m_decodingUnitCpbParamsInPicTimingSeiFlag = code;
  }
  else
  {
    sei.m_duCpbRemovalDelayIncrementLength = 24;
    sei.m_dpbOutputDelayDuLength = 24;
  }

  sei_read_flag( pDecodedMessageOutputStream, code, "concatenation_flag");
  sei.m_concatenationFlag = code;
  sei_read_flag ( pDecodedMessageOutputStream, code, "additional_concatenation_info_present_flag");
  sei.m_additionalConcatenationInfoPresentFlag = code;
  if (sei.m_additionalConcatenationInfoPresentFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, sei.m_initialCpbRemovalDelayLength, code, "max_initial_removal_delay_for_concatenation" );
    sei.m_maxInitialRemovalDelayForConcatenation = code;
  }

  sei_read_code( pDecodedMessageOutputStream, ( sei.m_cpbRemovalDelayLength ), code, "au_cpb_removal_delay_delta_minus1" );
  sei.m_auCpbRemovalDelayDelta = code + 1;
  sei_read_flag( pDecodedMessageOutputStream, code, "cpb_removal_delay_deltas_present_flag" );               sei.m_cpbRemovalDelayDeltasPresentFlag = code;
  if (sei.m_cpbRemovalDelayDeltasPresentFlag)
  {
    sei_read_uvlc( pDecodedMessageOutputStream, code, "num_cpb_removal_delay_deltas_minus1" );               sei.m_numCpbRemovalDelayDeltas = code + 1;
    for( i = 0; i < sei.m_numCpbRemovalDelayDeltas; i ++ )
    {
      sei_read_code( pDecodedMessageOutputStream, ( sei.m_cpbRemovalDelayLength ), code, "cpb_removal_delay_delta[i]" );
      sei.m_cpbRemovalDelayDelta[ i ] = code;
    }
  }
  sei_read_code( pDecodedMessageOutputStream, 3, code, "bp_max_sub_layers_minus1" );     sei.m_bpMaxSubLayers = code + 1;
  sei_read_uvlc( pDecodedMessageOutputStream, code, "bp_cpb_cnt_minus1" ); sei.m_bpCpbCnt = code + 1;
  sei_read_flag(pDecodedMessageOutputStream, code, "sublayer_initial_cpb_removal_delay_present_flag");
  sei.m_sublayerInitialCpbRemovalDelayPresentFlag = code;
  for (i = (sei.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : sei.m_bpMaxSubLayers - 1); i < sei.m_bpMaxSubLayers; i++)
  {
    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei.m_bpNalCpbParamsPresentFlag ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei.m_bpVclCpbParamsPresentFlag ) ) )
      {
        for( int j = 0; j < ( sei.m_bpCpbCnt ); j ++ )
        {
          sei_read_code( pDecodedMessageOutputStream, sei.m_initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_delay[i][j]" : "nal_initial_cpb_removal_delay[i][j]" );
          sei.m_initialCpbRemovalDelay[i][j][nalOrVcl] = code;
          sei_read_code( pDecodedMessageOutputStream, sei.m_initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_offset[i][j]" : "nal_initial_cpb_removal_offset[i][j]" );
          sei.m_initialCpbRemovalDelay[i][j][nalOrVcl] = code;
        }
      }
    }
  }
  if (sei.m_altCpbParamsPresentFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "use_alt_cpb_params_flag"); sei.m_useAltCpbParamsFlag = code;
  }

}

void SEIReader::xParseSEIPictureTiming(SEIPictureTiming& sei, uint32_t payloadSize, const uint32_t temporalId, const SEIBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream)
{

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t symbol;
  sei_read_code( pDecodedMessageOutputStream, bp.m_cpbRemovalDelayLength, symbol, "cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  sei.m_auCpbRemovalDelay[bp.m_bpMaxSubLayers - 1] = symbol + 1;

  if( bp.m_altCpbParamsPresentFlag ) 
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol, "cpb_alt_timing_info_present_flag" ); sei.m_cpbAltTimingInfoPresentFlag = symbol;
    if( sei.m_cpbAltTimingInfoPresentFlag ) 
    {
      sei.m_cpbAltInitialCpbRemovalDelayDelta.resize(bp.m_bpCpbCnt);
      sei.m_cpbAltInitialCpbRemovalOffsetDelta.resize(bp.m_bpCpbCnt);
      for( int i = 0; i < bp.m_bpCpbCnt; i++ ) 
      {
        sei_read_code( pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, symbol, "cpb_alt_initial_cpb_removal_delay_delta[ i ]" );
        sei.m_cpbAltInitialCpbRemovalDelayDelta[i]= symbol;
        sei_read_code( pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, symbol, "cpb_alt_initial_cpb_removal_offset_delta[ i ]" );
        sei.m_cpbAltInitialCpbRemovalOffsetDelta[i]= symbol;
      }
      sei_read_code( pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, sei.m_cpbDelayOffset, "cpb_delay_offset" );
      sei_read_code( pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, sei.m_dpbDelayOffset, "dpb_delay_offset" );
    }
  }
  else
  {
    sei.m_cpbAltTimingInfoPresentFlag = false;
    sei.m_cpbDelayOffset = sei.m_dpbDelayOffset = 0;
  }

  for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
  {
    sei_read_flag( pDecodedMessageOutputStream,    symbol, "pt_sub_layer_delays_present_flag[i]" );    sei.m_ptSubLayerDelaysPresentFlag[i] = (symbol == 1);
    if( sei.m_ptSubLayerDelaysPresentFlag[ i ] )
    {
      if (bp.m_cpbRemovalDelayDeltasPresentFlag)
      {
        sei_read_flag(pDecodedMessageOutputStream, symbol, "cpb_removal_delay_delta_enabled_flag[i]");
        sei.m_cpbRemovalDelayDeltaEnabledFlag[i] = (symbol == 1);
      }
      else
      {
        sei.m_cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      if( sei.m_cpbRemovalDelayDeltaEnabledFlag[ i ] )
      {
        sei_read_code( pDecodedMessageOutputStream, ceilLog2(bp.m_numCpbRemovalDelayDeltas), symbol, "cpb_removal_delay_delta_idx[i]" );
        sei.m_cpbRemovalDelayDeltaIdx[ i ] = symbol;
      }
      else
      {
        sei_read_code( pDecodedMessageOutputStream, bp.m_cpbRemovalDelayLength, symbol, "cpb_removal_delay_minus1[i]" );
        sei.m_auCpbRemovalDelay[ i ] = symbol + 1;
      }
    }
  }
  sei_read_code( pDecodedMessageOutputStream, bp.m_dpbOutputDelayLength,  symbol, "dpb_output_delay" );
  sei.m_picDpbOutputDelay = symbol;
  if( bp.m_bpDecodingUnitHrdParamsPresentFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, bp.getDpbOutputDelayDuLength(), symbol, "pic_dpb_output_du_delay" );
    sei.m_picDpbOutputDuDelay = symbol;
  }
  if( bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    sei_read_uvlc( pDecodedMessageOutputStream, symbol, "num_decoding_units_minus1" );
    sei.m_numDecodingUnitsMinus1 = symbol;
    sei.m_numNalusInDuMinus1.resize(sei.m_numDecodingUnitsMinus1 + 1 );
    sei.m_duCpbRemovalDelayMinus1.resize( (sei.m_numDecodingUnitsMinus1 + 1) * bp.m_bpMaxSubLayers );

    sei_read_flag( pDecodedMessageOutputStream, symbol, "du_common_cpb_removal_delay_flag" );
    sei.m_duCommonCpbRemovalDelayFlag = symbol;
    if( sei.m_duCommonCpbRemovalDelayFlag )
    {
      for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
      {
        if( sei.m_ptSubLayerDelaysPresentFlag[i] )
        {
          sei_read_code( pDecodedMessageOutputStream, bp.getDuCpbRemovalDelayIncrementLength(), symbol, "du_common_cpb_removal_delay_increment_minus1[i]" );
          sei.m_duCommonCpbRemovalDelayMinus1[i] = symbol;
        }
      }
    }
    for( int i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
    {
      sei_read_uvlc( pDecodedMessageOutputStream, symbol, "num_nalus_in_du_minus1[i]" );
      sei.m_numNalusInDuMinus1[i] = symbol;
      if( !sei.m_duCommonCpbRemovalDelayFlag && i < sei.m_numDecodingUnitsMinus1 )
      {
        for( int j = temporalId; j < bp.m_bpMaxSubLayers - 1; j ++ )
        {
          if( sei.m_ptSubLayerDelaysPresentFlag[j] )
          {
            sei_read_code( pDecodedMessageOutputStream, bp.getDuCpbRemovalDelayIncrementLength(), symbol, "du_cpb_removal_delay_increment_minus1[i][j]" );
            sei.m_duCpbRemovalDelayMinus1[i * bp.m_bpMaxSubLayers + j] = symbol;
          }
        }
      }
    }
  }
#if JVET_Q0818_PT_SEI
  sei_read_uvlc( pDecodedMessageOutputStream, symbol,    "pt_display_elemental_periods_minus1" );
  sei.m_ptDisplayElementalPeriodsMinus1 = symbol;
#endif
}

#if JVET_Q0818_PT_SEI
void SEIReader::xParseSEIFrameFieldinfo(SEIFrameFieldInfo& sei, const SEIPictureTiming& pt, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
#else
void SEIReader::xParseSEIFrameFieldinfo(SEIFrameFieldInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
#endif
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "field_pic_flag" );
  sei.m_fieldPicFlag= symbol;
  if (sei.m_fieldPicFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "bottom_field_flag" );
    sei.m_bottomFieldFlag = symbol;
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "pairing_indicated_flag" );
    sei.m_pairingIndicatedFlag = symbol;
    if (sei.m_pairingIndicatedFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "paired_with_next_field_flag" );
      sei.m_pairedWithNextFieldFlag = symbol;
    }
  }
  else
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "display_fields_from_frame_flag" );
    sei.m_displayFieldsFromFrameFlag = symbol;
    if (sei.m_displayFieldsFromFrameFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "display_fields_from_frame_flag" );
      sei.m_topFieldFirstFlag = symbol;
    }
    sei_read_uvlc( pDecodedMessageOutputStream, symbol,    "display_elemental_periods_minus1" );
    sei.m_displayElementalPeriodsMinus1 = symbol;
#if JVET_Q0818_PT_SEI
    if( pt.m_ptDisplayElementalPeriodsMinus1 != sei.m_displayElementalPeriodsMinus1 )
      msg( WARNING, "Warning: display_elemental_periods_minus1 is different in picture timing and frame field information SEI messages!");
#endif
  }
  sei_read_code( pDecodedMessageOutputStream, 2, symbol,   "source_scan_type" );
  sei.m_sourceScanType = symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "duplicate_flag" );
  sei.m_duplicateFlag = symbol;
}

void SEIReader::xParseSEIDependentRAPIndication( SEIDependentRAPIndication& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
}


void SEIReader::xParseSEIFramePacking(SEIFramePacking& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, val, "frame_packing_arrangement_id" );                 sei.m_arrangementId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "frame_packing_arrangement_cancel_flag" );        sei.m_arrangementCancelFlag = val;

  if( !sei.m_arrangementCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 7, val, "frame_packing_arrangement_type" );          sei.m_arrangementType = val;
    CHECK( ( sei.m_arrangementType <= 2 ) || ( sei.m_arrangementType >= 6 ), "Invalid arrangement type" );

    sei_read_flag( pDecodedMessageOutputStream, val, "quincunx_sampling_flag" );                     sei.m_quincunxSamplingFlag = val;

    sei_read_code( pDecodedMessageOutputStream, 6, val, "content_interpretation_type" );             sei.m_contentInterpretationType = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "spatial_flipping_flag" );                      sei.m_spatialFlippingFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_flipped_flag" );                        sei.m_frame0FlippedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "field_views_flag" );                           sei.m_fieldViewsFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "current_frame_is_frame0_flag" );               sei.m_currentFrameIsFrame0Flag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_self_contained_flag" );                 sei.m_frame0SelfContainedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame1_self_contained_flag" );                 sei.m_frame1SelfContainedFlag = val;

    if ( sei.m_quincunxSamplingFlag == 0 && sei.m_arrangementType != 5)
    {
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_x" );                sei.m_frame0GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_y" );                sei.m_frame0GridPositionY = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_x" );                sei.m_frame1GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_y" );                sei.m_frame1GridPositionY = val;
    }

    sei_read_code( pDecodedMessageOutputStream, 8, val, "frame_packing_arrangement_reserved_byte" );   sei.m_arrangementReservedByte = val;
    sei_read_flag( pDecodedMessageOutputStream, val,  "frame_packing_arrangement_persistence_flag" );  sei.m_arrangementPersistenceFlag = (val != 0);
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "upsampled_aspect_ratio_flag" );                  sei.m_upsampledAspectRatio = val;
}
void SEIReader::xParseSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[0]" ); sei.values.primaries[0][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[0]" ); sei.values.primaries[0][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[1]" ); sei.values.primaries[1][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[1]" ); sei.values.primaries[1][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[2]" ); sei.values.primaries[2][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[2]" ); sei.values.primaries[2][1] = code;


  sei_read_code( pDecodedMessageOutputStream, 16, code, "white_point_x" ); sei.values.whitePoint[0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "white_point_y" ); sei.values.whitePoint[1] = code;

  sei_read_code( pDecodedMessageOutputStream, 32, code, "max_display_mastering_luminance" ); sei.values.maxLuminance = code;
  sei_read_code( pDecodedMessageOutputStream, 32, code, "min_display_mastering_luminance" ); sei.values.minLuminance = code;
}

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
void SEIReader::xParseSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 8, code, "preferred_transfer_characteristics"); sei.m_preferredTransferCharacteristics = code;
}
#endif
void SEIReader::xParseSEIUserDataRegistered(SEIUserDataRegistered& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t code;
  assert(payloadSize>0);
  sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code"); payloadSize--;
  if (code == 255)
  {
    assert(payloadSize>0);
    sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code_extension_byte"); payloadSize--;
    code += 255;
  }
  sei.m_ituCountryCode = code;
  sei.m_userData.resize(payloadSize);
  for (uint32_t i = 0; i < sei.m_userData.size(); i++)
  {
    sei_read_code(NULL, 8, code, "itu_t_t35_payload_byte");
    sei.m_userData[i] = code;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  itu_t_t35 payload size: " << sei.m_userData.size() << "\n";
  }
}

void SEIReader::xParseSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, code, "film_grain_characteristics_cancel_flag");     sei.m_filmGrainCharacteristicsCancelFlag = code != 0;
  if (!sei.m_filmGrainCharacteristicsCancelFlag)
  {
    sei_read_code(pDecodedMessageOutputStream, 2, code, "film_grain_model_id");                   sei.m_filmGrainModelId = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "separate_colour_description_present_flag"); sei.m_separateColourDescriptionPresentFlag = code != 0;
    if (sei.m_separateColourDescriptionPresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 3, code, "film_grain_bit_depth_luma_minus8");    sei.m_filmGrainBitDepthLumaMinus8 = code;
      sei_read_code(pDecodedMessageOutputStream, 3, code, "film_grain_bit_depth_chroma_minus8");  sei.m_filmGrainBitDepthChromaMinus8 = code;
      sei_read_flag(pDecodedMessageOutputStream, code, "film_grain_full_range_flag");             sei.m_filmGrainFullRangeFlag = code != 0;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "film_grain_colour_primaries");         sei.m_filmGrainColourPrimaries = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "film_grain_transfer_characteristics"); sei.m_filmGrainTransferCharacteristics = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "film_grain_matrix_coeffs");            sei.m_filmGrainMatrixCoeffs = code;
    }
    sei_read_code(pDecodedMessageOutputStream, 2, code, "blending_mode_id");                      sei.m_blendingModeId = code;
    sei_read_code(pDecodedMessageOutputStream, 4, code, "log2_scale_factor");                     sei.m_log2ScaleFactor = code;
    for (int c = 0; c<3; c++)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "comp_model_present_flag[c]");             sei.m_compModel[c].presentFlag = code != 0;
    }
    for (int c = 0; c<3; c++)
    {
      SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      if (cm.presentFlag)
      {
        uint32_t numIntensityIntervals;
        sei_read_code(pDecodedMessageOutputStream, 8, code, "num_intensity_intervals_minus1[c]"); numIntensityIntervals = code + 1;
        sei_read_code(pDecodedMessageOutputStream, 3, code, "num_model_values_minus1[c]");        cm.numModelValues = code + 1;
        cm.intensityValues.resize(numIntensityIntervals);
        for (uint32_t interval = 0; interval<numIntensityIntervals; interval++)
        {
          SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv = cm.intensityValues[interval];
          sei_read_code(pDecodedMessageOutputStream, 8, code, "intensity_interval_lower_bound[c][i]"); cmiv.intensityIntervalLowerBound = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "intensity_interval_upper_bound[c][i]"); cmiv.intensityIntervalUpperBound = code;
          cmiv.compModelValue.resize(cm.numModelValues);
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            sei_read_svlc(pDecodedMessageOutputStream, cmiv.compModelValue[j], "comp_model_value[c][i]");
          }
        }
      }
    } // for c
    sei_read_flag(pDecodedMessageOutputStream, code, "film_grain_characteristics_persistence_flag"); sei.m_filmGrainCharacteristicsPersistenceFlag = code != 0;
  } // cancel flag
}

void SEIReader::xParseSEIContentLightLevelInfo(SEIContentLightLevelInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_content_light_level");     sei.m_maxContentLightLevel = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_pic_average_light_level"); sei.m_maxPicAverageLightLevel = code;
}

void SEIReader::xParseSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 32, code, "ambient_illuminance"); sei.m_ambientIlluminance = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_x");     sei.m_ambientLightX = (uint16_t)code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_y");     sei.m_ambientLightY = (uint16_t)code;
}

void SEIReader::xParseSEIContentColourVolume(SEIContentColourVolume& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int i;
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, val, "ccv_cancel_flag");          sei.m_ccvCancelFlag = val;
  if (!sei.m_ccvCancelFlag)
  {
    int iVal;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_persistence_flag");   sei.m_ccvPersistenceFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_primaries_present_flag");   sei.m_ccvPrimariesPresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_min_luminance_value_present_flag");   sei.m_ccvMinLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_max_luminance_value_present_flag");   sei.m_ccvMaxLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_avg_luminance_value_present_flag");   sei.m_ccvAvgLuminanceValuePresentFlag = val;

    if (sei.m_ccvPrimariesPresentFlag)
    {
      for (i = 0; i < MAX_NUM_COMPONENT; i++)
      {
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_x[i]");          sei.m_ccvPrimariesX[i] = iVal;
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_y[i]");          sei.m_ccvPrimariesY[i] = iVal;
      }
    }
    if (sei.m_ccvMinLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_min_luminance_value");   sei.m_ccvMinLuminanceValue = val;
    }
    if (sei.m_ccvMaxLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_max_luminance_value");   sei.m_ccvMaxLuminanceValue = val;
    }
    if (sei.m_ccvAvgLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_avg_luminance_value");   sei.m_ccvAvgLuminanceValue = val;
    }
  }
}
void SEIReader::xParseSEIEquirectangularProjection(SEIEquirectangularProjection& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, val,       "erp_cancel_flag" );              sei.m_erpCancelFlag = val;
  if( !sei.m_erpCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_persistence_flag"    );     sei.m_erpPersistenceFlag   = val;
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_guard_band_flag"     );     sei.m_erpGuardBandFlag     = val;
    sei_read_code( pDecodedMessageOutputStream, 2, val,   "erp_reserved_zero_2bits" );
    if ( sei.m_erpGuardBandFlag == 1)
    {
      sei_read_code( pDecodedMessageOutputStream, 3, val,     "erp_guard_band_type"       );   sei.m_erpGuardBandType  = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_left_guard_band_width" );   sei.m_erpLeftGuardBandWidth = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_right_guard_band_width");   sei.m_erpRightGuardBandWidth = val;
    }
  }
}

void SEIReader::xParseSEISphereRotation(SEISphereRotation& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  int  sval;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, val,       "sphere_rotation_cancel_flag" );              sei.m_sphereRotationCancelFlag = val;
  if( !sei.m_sphereRotationCancelFlag )
  {
    sei_read_flag ( pDecodedMessageOutputStream,      val,   "sphere_rotation_persistence_flag"    );     sei.m_sphereRotationPersistenceFlag = val;
    sei_read_code ( pDecodedMessageOutputStream, 6,   val,   "sphere_rotation_reserved_zero_6bits" );
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_yaw"                 );     sei.m_sphereRotationYaw = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_pitch"               );     sei.m_sphereRotationPitch = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_roll"                );     sei.m_sphereRotationRoll = sval;
  }
}

void SEIReader::xParseSEIOmniViewport(SEIOmniViewport& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  int  scode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code( pDecodedMessageOutputStream, 10, code, "omni_viewport_id"          ); sei.m_omniViewportId         = code;
  sei_read_flag( pDecodedMessageOutputStream,     code, "omni_viewport_cancel_flag" ); sei.m_omniViewportCancelFlag = code;
  if (!sei.m_omniViewportCancelFlag)
  {
    uint32_t numRegions;
    sei_read_flag( pDecodedMessageOutputStream,    code,       "omni_viewport_persistence_flag" ); sei.m_omniViewportPersistenceFlag = code;    
    sei_read_code( pDecodedMessageOutputStream, 4, numRegions, "omni_viewport_cnt_minus1"       ); numRegions++;
    sei.m_omniViewportRegions.resize(numRegions);
    for(uint32_t region=0; region<numRegions; region++)
    {
      SEIOmniViewport::OmniViewport &viewport = sei.m_omniViewportRegions[region];
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_azimuth_centre"   );   viewport.azimuthCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_elevation_centre" );   viewport.elevationCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_tilt_centre"      );   viewport.tiltCentre = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_hor_range"         );   viewport.horRange        = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_ver_range"         );   viewport.verRange        = code;
    }    
  }
  else
  {
    sei.m_omniViewportRegions.clear();
    sei.m_omniViewportPersistenceFlag=false;
  }
}

void SEIReader::xParseSEIRegionWisePacking(SEIRegionWisePacking& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag( pDecodedMessageOutputStream,           val,      "rwp_cancel_flag" );                      sei.m_rwpCancelFlag = val;
  if (!sei.m_rwpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_persistence_flag" );                 sei.m_rwpPersistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream,           val,    "constituent_picture_matching_flag" );    sei.m_constituentPictureMatchingFlag = val;
    sei_read_code( pDecodedMessageOutputStream,       5,  val,    "rwp_reserved_zero_5bits" );
    sei_read_code( pDecodedMessageOutputStream,       8,  val,    "num_packed_regions" );                   sei.m_numPackedRegions = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_width" );                   sei.m_projPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_height" );                  sei.m_projPictureHeight = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_width" );                 sei.m_packedPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_height" );                sei.m_packedPictureHeight = val;

    sei.m_rwpTransformType.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandFlag.resize(sei.m_numPackedRegions);
    sei.m_projRegionWidth.resize(sei.m_numPackedRegions);
    sei.m_projRegionHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpProjRegionTop.resize(sei.m_numPackedRegions);
    sei.m_projRegionLeft.resize(sei.m_numPackedRegions);
    sei.m_packedRegionWidth.resize(sei.m_numPackedRegions);
    sei.m_packedRegionHeight.resize(sei.m_numPackedRegions);
    sei.m_packedRegionTop.resize(sei.m_numPackedRegions);
    sei.m_packedRegionLeft.resize(sei.m_numPackedRegions);
    sei.m_rwpLeftGuardBandWidth.resize(sei.m_numPackedRegions);
    sei.m_rwpRightGuardBandWidth.resize(sei.m_numPackedRegions);
    sei.m_rwpTopGuardBandHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpBottomGuardBandHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandNotUsedForPredFlag.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandType.resize(4*sei.m_numPackedRegions);

    for( int i=0; i < sei.m_numPackedRegions; i++ )
    {
      sei_read_code( pDecodedMessageOutputStream,     4,  val,    "rwp_reserved_zero_4bits" );
      sei_read_code( pDecodedMessageOutputStream,     3,  val,    "rwp_tTransform_type" );                  sei.m_rwpTransformType[i] = val;
      sei_read_flag( pDecodedMessageOutputStream,         val,    "rwp_guard_band_flag" );                  sei.m_rwpGuardBandFlag[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_width" );                    sei.m_projRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_height" );                   sei.m_projRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_regionTop" );                   sei.m_rwpProjRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_left" );                     sei.m_projRegionLeft[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_width" );                  sei.m_packedRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_height" );                 sei.m_packedRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_top" );                    sei.m_packedRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_left" );                   sei.m_packedRegionLeft[i] = val;
      if( sei.m_rwpGuardBandFlag[i] )
      {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_left_guard_band_width" );            sei.m_rwpLeftGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_right_guard_band_width" );           sei.m_rwpRightGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_top_guard_band_height" );            sei.m_rwpTopGuardBandHeight[i]  = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_bottom_guard_band_height" );         sei. m_rwpBottomGuardBandHeight[i]  = val;
        sei_read_flag( pDecodedMessageOutputStream,       val,    "rwp_guard_band_not_used_forPred_flag" ); sei.m_rwpGuardBandNotUsedForPredFlag[i] = val;
        for( int j=0; j < 4; j++ )
        {
          sei_read_code( pDecodedMessageOutputStream, 3,  val,     "rwp_guard_band_type" ); sei.m_rwpGuardBandType[i*4 + j] = val;
        }
        sei_read_code( pDecodedMessageOutputStream,   3,  val,    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

void SEIReader::xParseSEIGeneralizedCubemapProjection(SEIGeneralizedCubemapProjection& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream,          val,    "gcmp_cancel_flag" );                      sei.m_gcmpCancelFlag = val;
  if (!sei.m_gcmpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_persistence_flag"    );              sei.m_gcmpPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     3, val,    "gcmp_packing_type" );                     sei.m_gcmpPackingType = val;
    sei_read_code( pDecodedMessageOutputStream,     2, val,    "gcmp_mapping_function_type"     );        sei.m_gcmpMappingFunctionType = val;

    int numFace = sei.m_gcmpPackingType == 4 || sei.m_gcmpPackingType == 5 ? 5 : 6;
    sei.m_gcmpFaceIndex.resize(numFace);
    sei.m_gcmpFaceRotation.resize(numFace);
    if (sei.m_gcmpMappingFunctionType == 2)
    {
      sei.m_gcmpFunctionCoeffU.resize(numFace);
      sei.m_gcmpFunctionUAffectedByVFlag.resize(numFace);
      sei.m_gcmpFunctionCoeffV.resize(numFace);
      sei.m_gcmpFunctionVAffectedByUFlag.resize(numFace);
    }

    for (int i = 0; i < numFace; i++)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_face_index" );                       sei.m_gcmpFaceIndex[i] = val;
      sei_read_code( pDecodedMessageOutputStream,   2, val,    "gcmp_face_rotation" );                    sei.m_gcmpFaceRotation[i] = val;
      if (sei.m_gcmpMappingFunctionType == 2)
      {
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_u" );                 sei.m_gcmpFunctionCoeffU[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_u_affected_by_v_flag"    ); sei.m_gcmpFunctionUAffectedByVFlag[i] = val;
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_v" );                 sei.m_gcmpFunctionCoeffV[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_v_affected_by_u_flag"    ); sei.m_gcmpFunctionVAffectedByUFlag[i] = val;
      }
    }
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_guard_band_flag" );                  sei.m_gcmpGuardBandFlag = val;
    if (sei.m_gcmpGuardBandFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream,      val,    "gcmp_guard_band_boundary_type" );         sei.m_gcmpGuardBandBoundaryType  = val;
      sei_read_code( pDecodedMessageOutputStream,   4, val,    "gcmp_guard_band_samples_minus1" );        sei.m_gcmpGuardBandSamplesMinus1 = val;
    }
  }
}

void SEIReader::xParseSEISubpictureLevelInfo(SEISubpicureLevelInfo& sei, const SPS *sps, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  if (sps == nullptr)
  {
    fprintf (stderr, "no SPS available, not parsing Subpicture level information SEI");
    return;
  }
  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream,   4,  val,    "sli_seq_parameter_set_id" );         sei.m_sliSeqParameterSetId  = val;
  sei_read_code( pDecodedMessageOutputStream,   3,  val,    "num_ref_levels_minus1" );            sei.m_numRefLevels  = val + 1;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "explicit_fraction_present_flag" );   sei.m_explicitFractionPresentFlag = val;
  
  sei.m_refLevelIdc.resize(sei.m_numRefLevels);
  if (sei.m_explicitFractionPresentFlag)
  {
    sei.m_refLevelFraction.resize(sei.m_numRefLevels);
  }

  for( int i = 0; i  <  sei.m_numRefLevels; i++ ) 
  {
    sei_read_code( pDecodedMessageOutputStream,   8,  val,    "ref_level_idc[i]" );         sei.m_refLevelIdc[i]  = (Level::Name) val;
    if( sei.m_explicitFractionPresentFlag )
    {
      int numSubPics = sps->getNumSubPics();
      sei.m_refLevelFraction[i].resize(numSubPics);

      for( int j = 0; j  <  numSubPics; j++ )
      {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "ref_level_fraction_minus1[i][j]" );  sei.m_refLevelFraction[i][j]= val;
      }
    }
  }
}

void SEIReader::xParseSEISampleAspectRatioInfo(SEISampleAspectRatioInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag( pDecodedMessageOutputStream,           val,    "sari_cancel_flag" );                      sei.m_sariCancelFlag = val;
  if (!sei.m_sariCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,         val,    "sari_persistence_flag" );                 sei.m_sariPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     8,  val,    "sari_aspect_ratio_idc" );                 sei.m_sariAspectRatioIdc = val;
    if (sei.m_sariAspectRatioIdc == 255)
    { 
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_width" );                        sei.m_sariSarWidth = val;
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_height" );                       sei.m_sariSarHeight = val;
    }
  }
}



//! \}
