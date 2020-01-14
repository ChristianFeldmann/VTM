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

#include "CommonLib/BitStream.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Slice.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"
#include "SEIwrite.h"

//! \ingroup EncoderLib
//! \{

void SEIWriter::xWriteSEIpayloadData(OutputBitstream& bs, const SEI& sei, const SPS *sps, HRD &hrd, const uint32_t temporalId)
{
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
  const SEIBufferingPeriod *bp = NULL;
#endif
  switch (sei.payloadType())
  {
#if HEVC_SEI || JVET_P0337_PORTING_SEI
  case SEI::USER_DATA_UNREGISTERED:
    xWriteSEIuserDataUnregistered(*static_cast<const SEIuserDataUnregistered*>(&sei));
    break;
#if !JVET_P0337_PORTING_SEI
  case SEI::ACTIVE_PARAMETER_SETS:
    xWriteSEIActiveParameterSets(*static_cast<const SEIActiveParameterSets*>(& sei));
    break;
#endif
#endif
  case SEI::DECODING_UNIT_INFO:
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
    bp = hrd.getBufferingPeriodSEI();
    CHECK (bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Decoding Unit Information SEI");
    xWriteSEIDecodingUnitInfo(*static_cast<const SEIDecodingUnitInfo*>(& sei), *bp, temporalId);
#else
    xWriteSEIDecodingUnitInfo(*static_cast<const SEIDecodingUnitInfo*>(& sei), sps, hrd);
#endif
    break;
  case SEI::DECODED_PICTURE_HASH:
    xWriteSEIDecodedPictureHash(*static_cast<const SEIDecodedPictureHash*>(&sei));
    break;
  case SEI::BUFFERING_PERIOD:
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
    xWriteSEIBufferingPeriod(*static_cast<const SEIBufferingPeriod*>(&sei));
#else
    xWriteSEIBufferingPeriod(*static_cast<const SEIBufferingPeriod*>(&sei), sps);
#endif
    hrd.setBufferingPeriodSEI(static_cast<const SEIBufferingPeriod*>(&sei));
    break;
  case SEI::PICTURE_TIMING:
    {
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
      bp = hrd.getBufferingPeriodSEI();
#else
      const SEIBufferingPeriod *bp = hrd.getBufferingPeriodSEI();
#endif
      CHECK (bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Picture Timing SEI");
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
      xWriteSEIPictureTiming(*static_cast<const SEIPictureTiming*>(&sei), *bp, temporalId);
#else
      xWriteSEIPictureTiming(*static_cast<const SEIPictureTiming*>(&sei), sps, *bp, temporalId);
#endif
    }
    break;
  case SEI::FRAME_FIELD_INFO:
    xWriteSEIFrameFieldInfo(*static_cast<const SEIFrameFieldInfo*>(&sei));
    break;
  case SEI::DEPENDENT_RAP_INDICATION:
    xWriteSEIDependentRAPIndication(*static_cast<const SEIDependentRAPIndication*>(&sei));
    break;
#if HEVC_SEI || JVET_P0337_PORTING_SEI
#if !JVET_P0337_PORTING_SEI
  case SEI::RECOVERY_POINT:
    xWriteSEIRecoveryPoint(*static_cast<const SEIRecoveryPoint*>(&sei));
    break;
#endif
  case SEI::FRAME_PACKING:
    xWriteSEIFramePacking(*static_cast<const SEIFramePacking*>(&sei));
    break;
#if !JVET_P0337_PORTING_SEI
  case SEI::SEGM_RECT_FRAME_PACKING:
    xWriteSEISegmentedRectFramePacking(*static_cast<const SEISegmentedRectFramePacking*>(&sei));
    break;
  case SEI::DISPLAY_ORIENTATION:
    xWriteSEIDisplayOrientation(*static_cast<const SEIDisplayOrientation*>(&sei));
    break;
  case SEI::TEMPORAL_LEVEL0_INDEX:
    xWriteSEITemporalLevel0Index(*static_cast<const SEITemporalLevel0Index*>(&sei));
    break;
  case SEI::REGION_REFRESH_INFO:
    xWriteSEIGradualDecodingRefreshInfo(*static_cast<const SEIGradualDecodingRefreshInfo*>(&sei));
    break;
  case SEI::NO_DISPLAY:
    xWriteSEINoDisplay(*static_cast<const SEINoDisplay*>(&sei));
    break;
  case SEI::TONE_MAPPING_INFO:
    xWriteSEIToneMappingInfo(*static_cast<const SEIToneMappingInfo*>(&sei));
    break;
  case SEI::SOP_DESCRIPTION:
    xWriteSEISOPDescription(*static_cast<const SEISOPDescription*>(&sei));
    break;
  case SEI::SCALABLE_NESTING:
    xWriteSEIScalableNesting(bs, *static_cast<const SEIScalableNesting*>(&sei), sps);
    break;
  case SEI::CHROMA_RESAMPLING_FILTER_HINT:
    xWriteSEIChromaResamplingFilterHint(*static_cast<const SEIChromaResamplingFilterHint*>(&sei));
    break;
  case SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS:
    xWriteSEITempMotionConstrainedTileSets(*static_cast<const SEITempMotionConstrainedTileSets*>(&sei));
    break;
  case SEI::TIME_CODE:
    xWriteSEITimeCode(*static_cast<const SEITimeCode*>(&sei));
    break;
  case SEI::KNEE_FUNCTION_INFO:
    xWriteSEIKneeFunctionInfo(*static_cast<const SEIKneeFunctionInfo*>(&sei));
    break;
  case SEI::COLOUR_REMAPPING_INFO:
    xWriteSEIColourRemappingInfo(*static_cast<const SEIColourRemappingInfo*>(&sei));
    break;
#endif
  case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
    xWriteSEIMasteringDisplayColourVolume(*static_cast<const SEIMasteringDisplayColourVolume*>(&sei));
    break;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
    xWriteSEIAlternativeTransferCharacteristics(*static_cast<const SEIAlternativeTransferCharacteristics*>(&sei));
    break;
#endif
#if !JVET_P0337_PORTING_SEI
  case SEI::GREEN_METADATA:
      xWriteSEIGreenMetadataInfo(*static_cast<const SEIGreenMetadataInfo*>(&sei));
    break;
#endif
#endif
#if JVET_P0462_SEI360
  case SEI::EQUIRECTANGULAR_PROJECTION:
    xWriteSEIEquirectangularProjection(*static_cast<const SEIEquirectangularProjection*>(&sei));
    break;
  case SEI::SPHERE_ROTATION:
    xWriteSEISphereRotation(*static_cast<const SEISphereRotation*>(&sei));
    break;
  case SEI::OMNI_VIEWPORT:
    xWriteSEIOmniViewport(*static_cast<const SEIOmniViewport*>(&sei));
    break;
  case SEI::REGION_WISE_PACKING:
    xWriteSEIRegionWisePacking(*static_cast<const SEIRegionWisePacking*>(&sei));
    break;
#endif
#if JVET_P0597_GCMP_SEI
  case SEI::GENERALIZED_CUBEMAP_PROJECTION:
    xWriteSEIGeneralizedCubemapProjection(*static_cast<const SEIGeneralizedCubemapProjection*>(&sei));
    break;
#endif
#if JVET_P0337_PORTING_SEI
  case SEI::USER_DATA_REGISTERED_ITU_T_T35:
    xWriteSEIUserDataRegistered(*static_cast<const SEIUserDataRegistered*>(&sei));
    break;
  case SEI::FILM_GRAIN_CHARACTERISTICS:
    xWriteSEIFilmGrainCharacteristics(*static_cast<const SEIFilmGrainCharacteristics*>(&sei));
    break;
  case SEI::CONTENT_LIGHT_LEVEL_INFO:
    xWriteSEIContentLightLevelInfo(*static_cast<const SEIContentLightLevelInfo*>(&sei));
    break;
  case SEI::AMBIENT_VIEWING_ENVIRONMENT:
    xWriteSEIAmbientViewingEnvironment(*static_cast<const SEIAmbientViewingEnvironment*>(&sei));
    break;
  case SEI::CONTENT_COLOUR_VOLUME:
    xWriteSEIContentColourVolume(*static_cast<const SEIContentColourVolume*>(&sei));
    break;
#endif
#if JVET_P0984_SEI_SUBPIC_LEVEL
  case SEI::SUBPICTURE_LEVEL_INFO:
    xWriteSEISubpictureLevelInfo(*static_cast<const SEISubpicureLevelInfo*>(&sei), sps);
    break;
#endif
#if JVET_P0450_SEI_SARI
  case SEI::SAMPLE_ASPECT_RATIO_INFO:
    xWriteSEISampleAspectRatioInfo(*static_cast<const SEISampleAspectRatioInfo*>(&sei));
    break;
#endif
  default:
    THROW("Trying to write unhandled SEI message");
    break;
  }
  xWriteByteAlign();
}

/**
 * marshal all SEI messages in provided list into one bitstream bs
 */
void SEIWriter::writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, const SPS *sps, HRD &hrd, bool isNested, const uint32_t temporalId)
{
#if ENABLE_TRACING
  if (g_HLSTraceEnable)
    xTraceSEIHeader();
#endif

  OutputBitstream bs_count;

  for (SEIMessages::const_iterator sei=seiList.begin(); sei!=seiList.end(); sei++)
  {
    // calculate how large the payload data is
    // TODO: this would be far nicer if it used vectored buffers
    bs_count.clear();
    setBitstream(&bs_count);

#if ENABLE_TRACING
    bool traceEnable = g_HLSTraceEnable;
    g_HLSTraceEnable = false;
#endif
    xWriteSEIpayloadData(bs_count, **sei, sps, hrd, temporalId);
#if ENABLE_TRACING
    g_HLSTraceEnable = traceEnable;
#endif
    uint32_t payload_data_num_bits = bs_count.getNumberOfWrittenBits();
    CHECK(0 != payload_data_num_bits % 8, "Invalid number of payload data bits");

    setBitstream(&bs);
    uint32_t payloadType = (*sei)->payloadType();
    for (; payloadType >= 0xff; payloadType -= 0xff)
    {
      WRITE_CODE(0xff, 8, "payload_type");
    }
    WRITE_CODE(payloadType, 8, "payload_type");

    uint32_t payloadSize = payload_data_num_bits/8;
    for (; payloadSize >= 0xff; payloadSize -= 0xff)
    {
      WRITE_CODE(0xff, 8, "payload_size");
    }
    WRITE_CODE(payloadSize, 8, "payload_size");

    /* payloadData */
#if ENABLE_TRACING
    if (g_HLSTraceEnable)
      xTraceSEIMessageType((*sei)->payloadType());
#endif

    xWriteSEIpayloadData(bs_count, **sei, sps, hrd, temporalId);
  }
  if (!isNested)
  {
    xWriteRbspTrailingBits();
  }
}

#if HEVC_SEI || JVET_P0337_PORTING_SEI
/**
 * marshal a user_data_unregistered SEI message sei, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIuserDataUnregistered(const SEIuserDataUnregistered &sei)
{
  for (uint32_t i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    WRITE_CODE(sei.uuid_iso_iec_11578[i], 8 , "sei.uuid_iso_iec_11578[i]");
  }

  for (uint32_t i = 0; i < sei.userDataLength; i++)
  {
    WRITE_CODE(sei.userData[i], 8 , "user_data");
  }
}
#endif

/**
 * marshal a decoded picture hash SEI message, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei)
{
  const char *traceString="\0";
  switch (sei.method)
  {
    case HASHTYPE_MD5: traceString="picture_md5"; break;
    case HASHTYPE_CRC: traceString="picture_crc"; break;
    case HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: THROW("Unknown hash type"); break;
  }

  if (traceString != 0) //use of this variable is needed to avoid a compiler error with G++ 4.6.1
  {
    WRITE_CODE(sei.method, 8, "hash_type");
    for(uint32_t i=0; i<uint32_t(sei.m_pictureHash.hash.size()); i++)
    {
      WRITE_CODE(sei.m_pictureHash.hash[i], 8, traceString);
    }
  }
}

#if HEVC_SEI
void SEIWriter::xWriteSEIActiveParameterSets(const SEIActiveParameterSets& sei)
{
  WRITE_FLAG(sei.m_selfContainedCvsFlag,     "self_contained_cvs_flag");
  WRITE_FLAG(sei.m_noParameterSetUpdateFlag, "no_parameter_set_update_flag");
  WRITE_UVLC(sei.numSpsIdsMinus1,            "num_sps_ids_minus1");

  CHECK(sei.activeSeqParameterSetId.size() != (sei.numSpsIdsMinus1 + 1), "Unknown active SPS");

  for (int i = 0; i < sei.activeSeqParameterSetId.size(); i++)
  {
#if JVET_P0244_SPS_CLEAN_UP
    WRITE_CODE( sei.activeSeqParameterSetId[i], 4, "active_seq_parameter_set_id" );
#else
    WRITE_UVLC(sei.activeSeqParameterSetId[i], "active_seq_parameter_set_id");
#endif
  }
}
#endif

#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
void SEIWriter::xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SEIBufferingPeriod& bp, const uint32_t temporalId)
{
  WRITE_UVLC(sei.m_decodingUnitIdx, "decoding_unit_idx");
  if( !bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
    {
      WRITE_FLAG( sei.m_duiSubLayerDelaysPresentFlag[i], "dui_sub_layer_delays_present_flag[i]" );
      if( sei.m_duiSubLayerDelaysPresentFlag[i] )
        WRITE_CODE( sei.m_duSptCpbRemovalDelayIncrement[i], bp.getDuCpbRemovalDelayIncrementLength(), "du_spt_cpb_removal_delay_increment[i]");
    }
  }
  WRITE_FLAG( sei.m_dpbOutputDuDelayPresentFlag, "dpb_output_du_delay_present_flag");
  if(sei.m_dpbOutputDuDelayPresentFlag)
  {
    WRITE_CODE(sei.m_picSptDpbOutputDuDelay, bp.getDpbOutputDelayDuLength(), "pic_spt_dpb_output_du_delay");
  }
}
#else
void SEIWriter::xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SPS *sps, HRD &hrd)
{
  WRITE_UVLC(sei.m_decodingUnitIdx, "decoding_unit_idx");
  if(sps->getHrdParameters()->getDecodingUnitHrdParamsPresentFlag())
  {
    WRITE_CODE( sei.m_duSptCpbRemovalDelay, hrd.getBufferingPeriodSEI()->getDuCpbRemovalDelayIncrementLength(), "du_spt_cpb_removal_delay_increment");
  }
  WRITE_FLAG( sei.m_dpbOutputDuDelayPresentFlag, "dpb_output_du_delay_present_flag");
  if(sei.m_dpbOutputDuDelayPresentFlag)
  {
    WRITE_CODE(sei.m_picSptDpbOutputDuDelay, hrd.getBufferingPeriodSEI()->getDpbOutputDelayDuLength(), "pic_spt_dpb_output_du_delay");
  }
}
#endif

#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
void SEIWriter::xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei)
#else
void SEIWriter::xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei, const SPS *sps)
#endif
{
  WRITE_FLAG( sei.m_bpNalCpbParamsPresentFlag, "bp_nal_hrd_parameters_present_flag");
  WRITE_FLAG( sei.m_bpVclCpbParamsPresentFlag, "bp_vcl_hrd_parameters_present_flag");
#if JVET_P0181
  CHECK(!sei.m_bpNalCpbParamsPresentFlag && !sei.m_bpVclCpbParamsPresentFlag, "bp_nal_hrd_parameters_present_flag and/or bp_vcl_hrd_parameters_present_flag must be true");
#endif
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
  CHECK (sei.m_initialCpbRemovalDelayLength < 1, "sei.m_initialCpbRemovalDelayLength must be > 0");
  WRITE_CODE( sei.m_initialCpbRemovalDelayLength - 1, 5, "initial_cpb_removal_delay_length_minus1" );
  CHECK (sei.m_cpbRemovalDelayLength < 1, "sei.m_cpbRemovalDelayLength must be > 0");
  WRITE_CODE( sei.m_cpbRemovalDelayLength - 1,        5, "cpb_removal_delay_length_minus1" );
  CHECK (sei.m_dpbOutputDelayLength < 1, "sei.m_dpbOutputDelayLength must be > 0");
  WRITE_CODE( sei.m_dpbOutputDelayLength - 1,         5, "dpb_output_delay_length_minus1" );
#if JVET_P0446_ALT_CPB
  WRITE_FLAG(sei.m_altCpbParamsPresentFlag, "alt_cpb_params_present_flag");
#endif
  WRITE_FLAG( sei.m_bpDecodingUnitHrdParamsPresentFlag, "bp_decoding_unit_hrd_params_present_flag"  );
  if( sei.m_bpDecodingUnitHrdParamsPresentFlag )
  {
    CHECK (sei.m_duCpbRemovalDelayIncrementLength < 1, "sei.m_duCpbRemovalDelayIncrementLength must be > 0");
    WRITE_CODE( sei.m_duCpbRemovalDelayIncrementLength - 1, 5, "du_cpb_removal_delay_increment_length_minus1" );
    CHECK (sei.m_dpbOutputDelayDuLength < 1, "sei.m_dpbOutputDelayDuLength must be > 0");
    WRITE_CODE( sei.m_dpbOutputDelayDuLength - 1, 5, "dpb_output_delay_du_length_minus1" );
    WRITE_FLAG( sei.m_decodingUnitCpbParamsInPicTimingSeiFlag, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );
  }
#else
  if (sei.m_bpNalCpbParamsPresentFlag || sei.m_bpVclCpbParamsPresentFlag)
  {
    CHECK (sei.m_initialCpbRemovalDelayLength < 1, "sei.m_initialCpbRemovalDelayLength must be > 0");
    WRITE_CODE( sei.m_initialCpbRemovalDelayLength - 1, 5, "initial_cpb_removal_delay_length_minus1" );
    CHECK (sei.m_cpbRemovalDelayLength < 1, "sei.m_cpbRemovalDelayLength must be > 0");
    WRITE_CODE( sei.m_cpbRemovalDelayLength - 1,        5, "cpb_removal_delay_length_minus1" );
    CHECK (sei.m_dpbOutputDelayLength < 1, "sei.m_dpbOutputDelayLength must be > 0");
    WRITE_CODE( sei.m_dpbOutputDelayLength - 1,         5, "dpb_output_delay_length_minus1" );
    if( sps->getHrdParameters()->getDecodingUnitHrdParamsPresentFlag() )
    {
      CHECK (sei.m_duCpbRemovalDelayIncrementLength < 1, "sei.m_duCpbRemovalDelayIncrementLength must be > 0");
      WRITE_CODE( sei.m_duCpbRemovalDelayIncrementLength - 1, 5, "du_cpb_removal_delay_increment_length_minus1" );
      CHECK (sei.m_dpbOutputDelayDuLength < 1, "sei.m_dpbOutputDelayDuLength must be > 0");
      WRITE_CODE( sei.m_dpbOutputDelayDuLength - 1, 5, "dpb_output_delay_du_length_minus1" );
    }
  }
#endif

  WRITE_FLAG( sei.m_concatenationFlag, "concatenation_flag");
#if JVET_P0446_CONCATENATION
  WRITE_FLAG( sei.m_additionalConcatenationInfoPresentFlag, "additional_concatenation_info_present_flag");
  if (sei.m_additionalConcatenationInfoPresentFlag)
  {
    WRITE_CODE( sei.m_maxInitialRemovalDelayForConcatenation, sei.m_initialCpbRemovalDelayLength, "max_initial_removal_delay_for_concatenation" );
  }
#endif

  CHECK (sei.m_auCpbRemovalDelayDelta < 1, "sei.m_auCpbRemovalDelayDelta must be > 0");
  WRITE_CODE( sei.m_auCpbRemovalDelayDelta - 1, sei.m_cpbRemovalDelayLength, "au_cpb_removal_delay_delta_minus1" );

  WRITE_FLAG( sei.m_cpbRemovalDelayDeltasPresentFlag, "cpb_removal_delay_deltas_present_flag");
  if (sei.m_cpbRemovalDelayDeltasPresentFlag)
  {
    CHECK (sei.m_numCpbRemovalDelayDeltas < 1, "m_numCpbRemovalDelayDeltas must be > 0");
    WRITE_UVLC( sei.m_numCpbRemovalDelayDeltas - 1, "num_cpb_removal_delay_deltas_minus1" );
    for( int i = 0; i < sei.m_numCpbRemovalDelayDeltas; i ++ )
    {
      WRITE_CODE( sei.m_cpbRemovalDelayDelta[i],        sei.m_cpbRemovalDelayLength, "cpb_removal_delay_delta[i]" );
    }
    CHECK (sei.m_bpMaxSubLayers < 1, "bp_max_sub_layers_minus1 must be > 0");
    WRITE_CODE( sei.m_bpMaxSubLayers - 1,        3, "bp_max_sub_layers_minus1" );
  }
#if JVET_P0446_BP_CPB_CNT_FIX
  CHECK (sei.m_bpCpbCnt < 1, "sei.m_bpCpbCnt must be > 0");
  WRITE_UVLC( sei.m_bpCpbCnt - 1, "bp_cpb_cnt_minus1");
#endif
#if JVET_P0181
  WRITE_FLAG(sei.m_sublayerInitialCpbRemovalDelayPresentFlag, "sublayer_initial_cpb_removal_delay_present_flag");
  for (int i = (sei.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : sei.m_bpMaxSubLayers - 1); i < sei.m_bpMaxSubLayers; i++)
#else
  for (int i = 0; i < sei.m_bpMaxSubLayers; i++)
#endif
  {
#if !JVET_P0446_BP_CPB_CNT_FIX
    CHECK (sei.m_bpCpbCnt[i] < 1, "sei.m_bpCpbCnt[i] must be > 0");
    WRITE_UVLC( sei.m_bpCpbCnt[i] - 1, "bp_cpb_cnt_minus1[i]");
#endif
    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei.m_bpNalCpbParamsPresentFlag ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei.m_bpVclCpbParamsPresentFlag ) ) )
      {
#if !JVET_P0446_BP_CPB_CNT_FIX
        for( int j = 0; j < sei.m_bpCpbCnt[i]; j ++ )
#else
        for( int j = 0; j < sei.m_bpCpbCnt; j ++ )
#endif
        {
          WRITE_CODE( sei.m_initialCpbRemovalDelay[j][i][nalOrVcl],  sei.m_initialCpbRemovalDelayLength,           "initial_cpb_removal_delay[j][i][nalOrVcl]" );
          WRITE_CODE( sei.m_initialCpbRemovalOffset[j][i][nalOrVcl], sei.m_initialCpbRemovalDelayLength,           "initial_cpb_removal_delay_offset[j][i][nalOrVcl]" );
        }
      }
    }
  }
#if JVET_P0446_ALT_CPB
  if (sei.m_altCpbParamsPresentFlag)
  {
    WRITE_FLAG(sei.m_useAltCpbParamsFlag, "use_alt_cpb_params_flag");
  }
#endif

}

#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
void SEIWriter::xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SEIBufferingPeriod &bp, const uint32_t temporalId)
#else
void SEIWriter::xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SPS *sps, const SEIBufferingPeriod &bp, const uint32_t temporalId)
#endif
{
  
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
  WRITE_CODE( sei.m_auCpbRemovalDelay[bp.m_bpMaxSubLayers - 1] - 1, bp.m_cpbRemovalDelayLength,               "cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
#if JVET_P0446_ALT_CPB
  if( bp.m_altCpbParamsPresentFlag ) 
  {
    WRITE_FLAG( sei.m_cpbAltTimingInfoPresentFlag, "cpb_alt_timing_info_present_flag" );
    if( sei.m_cpbAltTimingInfoPresentFlag ) 
    {
      for( int i = 0; i < bp.m_bpCpbCnt; i++ ) 
      {
        WRITE_CODE( sei.m_cpbAltInitialCpbRemovalDelayDelta[i], bp.m_initialCpbRemovalDelayLength, "cpb_alt_initial_cpb_removal_delay_delta[ i ]" );
        WRITE_CODE( sei.m_cpbAltInitialCpbRemovalOffsetDelta[i], bp.m_initialCpbRemovalDelayLength, "cpb_alt_initial_cpb_removal_offset_delta[ i ]" );
      }
      WRITE_CODE( sei.m_cpbDelayOffset, bp.m_initialCpbRemovalDelayLength, "cpb_delay_offset" );
      WRITE_CODE( sei.m_dpbDelayOffset, bp.m_initialCpbRemovalDelayLength, "dpb_delay_offset" );
    }
  }
#endif
  for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
  {
    WRITE_FLAG( sei.m_ptSubLayerDelaysPresentFlag[i], "pt_sub_layer_delays_present_flag[i]" );
    if( sei.m_ptSubLayerDelaysPresentFlag[i] )
    {
#if JVET_P0183
      if (bp.m_cpbRemovalDelayDeltasPresentFlag)
      {
        WRITE_FLAG(sei.m_cpbRemovalDelayDeltaEnabledFlag[i], "cpb_removal_delay_delta_enabled_flag[i]");
      }
#else
      WRITE_FLAG(sei.m_cpbRemovalDelayDeltaEnabledFlag[i], "cpb_removal_delay_delta_enabled_flag[i]");
#endif
      if( sei.m_cpbRemovalDelayDeltaEnabledFlag[i] )
      {
        WRITE_CODE( sei.m_cpbRemovalDelayDeltaIdx[i], ceilLog2(bp.m_numCpbRemovalDelayDeltas),               "cpb_removal_delay_delta_idx[i]" );
      }
      else
      {
        WRITE_CODE( sei.m_auCpbRemovalDelay[i] - 1, bp.m_cpbRemovalDelayLength,                                "cpb_removal_delay_minus1[i]" );
      }
    }
  }
#else
  CHECK (sei.m_ptMaxSubLayers < 1, "pt_max_sub_layers_minus1 must be > 0");
  WRITE_CODE( sei.m_ptMaxSubLayers - 1,        3, "pt_max_sub_layers_minus1" );
  WRITE_CODE( sei.m_auCpbRemovalDelay[sei.m_ptMaxSubLayers - 1] - 1, bp.m_cpbRemovalDelayLength,               "cpb_removal_delay_minus1[pt_max_sub_layers_minus1]" );
  for( int i = temporalId; i < sei.m_ptMaxSubLayers - 1; i ++ )
  {
    WRITE_FLAG( sei.m_subLayerDelaysPresentFlag[i], "sub_layer_delays_present_flag[i]" );
    if( sei.m_subLayerDelaysPresentFlag[i] )
    {
      {
#if JVET_P0183
        if (bp.m_cpbRemovalDelayDeltasPresentFlag)
        {
          WRITE_FLAG(sei.m_cpbRemovalDelayDeltaEnabledFlag[i], "cpb_removal_delay_delta_enabled_flag[i]");
        }
#else
        WRITE_FLAG(sei.m_cpbRemovalDelayDeltaEnabledFlag[i], "cpb_removal_delay_delta_enabled_flag[i]");
#endif
        if( sei.m_cpbRemovalDelayDeltaEnabledFlag[i] )
      {
        WRITE_CODE( sei.m_cpbRemovalDelayDeltaIdx[i], ceilLog2(bp.m_numCpbRemovalDelayDeltas),               "cpb_removal_delay_delta_idx[i]" );
      }
      else
      {
        WRITE_CODE( sei.m_auCpbRemovalDelay[i] - 1, bp.m_cpbRemovalDelayLength,                                "cpb_removal_delay_minus1[i]" );
      }
    }
  }
#endif
  WRITE_CODE( sei.m_picDpbOutputDelay,     bp.m_dpbOutputDelayLength,                                          "dpb_output_delay" );
#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI 
  if( bp.m_bpDecodingUnitHrdParamsPresentFlag )
  {
    WRITE_CODE( sei.m_picDpbOutputDuDelay, bp.m_dpbOutputDelayDuLength, "pic_dpb_output_du_delay" );
  }
  if( bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    WRITE_UVLC( sei.m_numDecodingUnitsMinus1, "num_decoding_units_minus1" );
    WRITE_FLAG( sei.m_duCommonCpbRemovalDelayFlag, "du_commmon_cpb_removal_delay_flag" );
    if( sei.m_duCommonCpbRemovalDelayFlag )
    {
      for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
      {
        if( sei.m_ptSubLayerDelaysPresentFlag[i] )
          WRITE_CODE( sei.m_duCommonCpbRemovalDelayMinus1[i], bp.m_duCpbRemovalDelayIncrementLength, "du_common_cpb_removal_delay_increment_minus1[i]" );
      }
    }
    for( int i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
    {
      WRITE_UVLC( sei.m_numNalusInDuMinus1[i], "num_nalus_in_du_minus1[i]" );
      if( !sei.m_duCommonCpbRemovalDelayFlag && i < sei.m_numDecodingUnitsMinus1 )
      {
        for( int j = temporalId; j < bp.m_bpMaxSubLayers - 1; j ++ )
        {
          if( sei.m_ptSubLayerDelaysPresentFlag[j] )
            WRITE_CODE( sei.m_duCpbRemovalDelayMinus1[i * bp.m_bpMaxSubLayers + j], bp.m_duCpbRemovalDelayIncrementLength, "du_cpb_removal_delay_increment_minus1[i][j]" );
        }
      }
    }
  }
#else
  if( sps->getHrdParameters()->getDecodingUnitHrdParamsPresentFlag() )
  {
    WRITE_CODE( sei.m_picDpbOutputDuDelay, bp.m_dpbOutputDelayDuLength, "pic_dpb_output_du_delay" );
  }
  if( sps->getHrdParameters()->getDecodingUnitHrdParamsPresentFlag() && sps->getHrdParameters()->getDecodingUnitCpbParamsInPicTimingSeiFlag() )
  {
    WRITE_UVLC( sei.m_numDecodingUnitsMinus1, "num_decoding_units_minus1" );
    WRITE_FLAG( sei.m_duCommonCpbRemovalDelayFlag, "du_commmon_cpb_removal_delay_flag" );
    if( sei.m_duCommonCpbRemovalDelayFlag )
    {
      WRITE_CODE( sei.m_duCommonCpbRemovalDelayMinus1, bp.m_duCpbRemovalDelayIncrementLength, "du_common_cpb_removal_delay_increment_minus1" );
    }
    for( int i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
    {
      WRITE_UVLC( sei.m_numNalusInDuMinus1[i], "num_nalus_in_du_minus1[i]" );
      if( !sei.m_duCommonCpbRemovalDelayFlag && i < sei.m_numDecodingUnitsMinus1 )
      {
        WRITE_CODE( sei.m_duCpbRemovalDelayMinus1[i], bp.m_duCpbRemovalDelayIncrementLength, "du_cpb_removal_delay_increment_minus1[i]" );
      }
    }
  }
#endif
}

void SEIWriter::xWriteSEIFrameFieldInfo(const SEIFrameFieldInfo& sei)
{
  WRITE_FLAG( sei.m_fieldPicFlag ? 1 : 0,                    "field_pic_flag" );
  if (sei.m_fieldPicFlag)
  {
    WRITE_FLAG( sei.m_bottomFieldFlag ? 1 : 0,               "bottom_field_flag" );
    WRITE_FLAG( sei.m_pairingIndicatedFlag ? 1 : 0,          "pairing_indicated_flag" );
    if (sei.m_pairingIndicatedFlag)
    {
      WRITE_FLAG( sei.m_pairedWithNextFieldFlag ? 1 : 0,     "paired_with_next_field_flag" );
    }
  }
  else
  {
    WRITE_FLAG( sei.m_displayFieldsFromFrameFlag ? 1 : 0,     "display_fields_from_frame_flag" );
    if (sei.m_displayFieldsFromFrameFlag)
    {
      WRITE_FLAG( sei.m_topFieldFirstFlag ? 1 : 0,            "display_fields_from_frame_flag" );
    }
    WRITE_UVLC( sei.m_displayElementalPeriodsMinus1,          "display_elemental_periods_minus1" );
  }
  WRITE_CODE( sei.m_sourceScanType, 2,                        "source_scan_type" );
  WRITE_FLAG( sei.m_duplicateFlag ? 1 : 0,                    "duplicate_flag" );
}

void SEIWriter::xWriteSEIDependentRAPIndication(const SEIDependentRAPIndication& /*sei*/)
{
  // intentionally empty
}

#if HEVC_SEI || JVET_P0337_PORTING_SEI
#if !JVET_P0337_PORTING_SEI
void SEIWriter::xWriteSEIRecoveryPoint(const SEIRecoveryPoint& sei)
{
  WRITE_SVLC( sei.m_recoveryPocCnt,    "recovery_poc_cnt"    );
  WRITE_FLAG( sei.m_exactMatchingFlag, "exact_matching_flag" );
  WRITE_FLAG( sei.m_brokenLinkFlag,    "broken_link_flag"    );
}
#endif
void SEIWriter::xWriteSEIFramePacking(const SEIFramePacking& sei)
{
  WRITE_UVLC( sei.m_arrangementId,                  "frame_packing_arrangement_id" );
  WRITE_FLAG( sei.m_arrangementCancelFlag,          "frame_packing_arrangement_cancel_flag" );

  if( sei.m_arrangementCancelFlag == 0 )
  {
    WRITE_CODE( sei.m_arrangementType, 7,           "frame_packing_arrangement_type" );

    WRITE_FLAG( sei.m_quincunxSamplingFlag,         "quincunx_sampling_flag" );
    WRITE_CODE( sei.m_contentInterpretationType, 6, "content_interpretation_type" );
    WRITE_FLAG( sei.m_spatialFlippingFlag,          "spatial_flipping_flag" );
    WRITE_FLAG( sei.m_frame0FlippedFlag,            "frame0_flipped_flag" );
    WRITE_FLAG( sei.m_fieldViewsFlag,               "field_views_flag" );
    WRITE_FLAG( sei.m_currentFrameIsFrame0Flag,     "current_frame_is_frame0_flag" );

    WRITE_FLAG( sei.m_frame0SelfContainedFlag,      "frame0_self_contained_flag" );
    WRITE_FLAG( sei.m_frame1SelfContainedFlag,      "frame1_self_contained_flag" );

    if(sei.m_quincunxSamplingFlag == 0 && sei.m_arrangementType != 5)
    {
      WRITE_CODE( sei.m_frame0GridPositionX, 4,     "frame0_grid_position_x" );
      WRITE_CODE( sei.m_frame0GridPositionY, 4,     "frame0_grid_position_y" );
      WRITE_CODE( sei.m_frame1GridPositionX, 4,     "frame1_grid_position_x" );
      WRITE_CODE( sei.m_frame1GridPositionY, 4,     "frame1_grid_position_y" );
    }

    WRITE_CODE( sei.m_arrangementReservedByte, 8,   "frame_packing_arrangement_reserved_byte" );
    WRITE_FLAG( sei.m_arrangementPersistenceFlag,   "frame_packing_arrangement_persistence_flag" );
  }

  WRITE_FLAG( sei.m_upsampledAspectRatio,           "upsampled_aspect_ratio" );
}

#if !JVET_P0337_PORTING_SEI
void SEIWriter::xWriteSEISegmentedRectFramePacking(const SEISegmentedRectFramePacking& sei)
{
  WRITE_FLAG( sei.m_arrangementCancelFlag,          "segmented_rect_frame_packing_arrangement_cancel_flag" );
  if( sei.m_arrangementCancelFlag == 0 )
  {
    WRITE_CODE( sei.m_contentInterpretationType, 2, "segmented_rect_content_interpretation_type" );
    WRITE_FLAG( sei.m_arrangementPersistenceFlag,   "segmented_rect_frame_packing_arrangement_persistence" );
  }
}

void SEIWriter::xWriteSEIToneMappingInfo(const SEIToneMappingInfo& sei)
{
  int i;
  WRITE_UVLC( sei.m_toneMapId,                    "tone_map_id" );
  WRITE_FLAG( sei.m_toneMapCancelFlag,            "tone_map_cancel_flag" );
  if( !sei.m_toneMapCancelFlag )
  {
    WRITE_FLAG( sei.m_toneMapPersistenceFlag,     "tone_map_persistence_flag" );
    WRITE_CODE( sei.m_codedDataBitDepth,    8,    "coded_data_bit_depth" );
    WRITE_CODE( sei.m_targetBitDepth,       8,    "target_bit_depth" );
    WRITE_UVLC( sei.m_modelId,                    "model_id" );
    switch(sei.m_modelId)
    {
    case 0:
      {
        WRITE_CODE( sei.m_minValue,  32,        "min_value" );
        WRITE_CODE( sei.m_maxValue, 32,         "max_value" );
        break;
      }
    case 1:
      {
        WRITE_CODE( sei.m_sigmoidMidpoint, 32,  "sigmoid_midpoint" );
        WRITE_CODE( sei.m_sigmoidWidth,    32,  "sigmoid_width"    );
        break;
      }
    case 2:
      {
        uint32_t num = 1u << sei.m_targetBitDepth;
        for(i = 0; i < num; i++)
        {
          WRITE_CODE( sei.m_startOfCodedInterval[i], (( sei.m_codedDataBitDepth + 7 ) >> 3 ) << 3,  "start_of_coded_interval" );
        }
        break;
      }
    case 3:
      {
        WRITE_CODE( sei.m_numPivots, 16,          "num_pivots" );
        for(i = 0; i < sei.m_numPivots; i++ )
        {
          WRITE_CODE( sei.m_codedPivotValue[i], (( sei.m_codedDataBitDepth + 7 ) >> 3 ) << 3,       "coded_pivot_value" );
          WRITE_CODE( sei.m_targetPivotValue[i], (( sei.m_targetBitDepth + 7 ) >> 3 ) << 3,         "target_pivot_value");
        }
        break;
      }
    case 4:
      {
        WRITE_CODE( sei.m_cameraIsoSpeedIdc,    8,    "camera_iso_speed_idc" );
        if( sei.m_cameraIsoSpeedIdc == 255) //Extended_ISO
        {
          WRITE_CODE( sei.m_cameraIsoSpeedValue,    32,    "camera_iso_speed_value" );
        }
        WRITE_CODE( sei.m_exposureIndexIdc,     8,    "exposure_index_idc" );
        if( sei.m_exposureIndexIdc == 255) //Extended_ISO
        {
          WRITE_CODE( sei.m_exposureIndexValue,     32,    "exposure_index_value" );
        }
        WRITE_FLAG( sei.m_exposureCompensationValueSignFlag,           "exposure_compensation_value_sign_flag" );
        WRITE_CODE( sei.m_exposureCompensationValueNumerator,     16,  "exposure_compensation_value_numerator" );
        WRITE_CODE( sei.m_exposureCompensationValueDenomIdc,      16,  "exposure_compensation_value_denom_idc" );
        WRITE_CODE( sei.m_refScreenLuminanceWhite,                32,  "ref_screen_luminance_white" );
        WRITE_CODE( sei.m_extendedRangeWhiteLevel,                32,  "extended_range_white_level" );
        WRITE_CODE( sei.m_nominalBlackLevelLumaCodeValue,         16,  "nominal_black_level_luma_code_value" );
        WRITE_CODE( sei.m_nominalWhiteLevelLumaCodeValue,         16,  "nominal_white_level_luma_code_value" );
        WRITE_CODE( sei.m_extendedWhiteLevelLumaCodeValue,        16,  "extended_white_level_luma_code_value" );
        break;
      }
    default:
      {
        THROW("Undefined SEIToneMapModelId");
        break;
      }
    }//switch m_modelId
  }//if(!sei.m_toneMapCancelFlag)
}

void SEIWriter::xWriteSEIDisplayOrientation(const SEIDisplayOrientation &sei)
{
  WRITE_FLAG( sei.cancelFlag,           "display_orientation_cancel_flag" );
  if( !sei.cancelFlag )
  {
    WRITE_FLAG( sei.horFlip,                   "hor_flip" );
    WRITE_FLAG( sei.verFlip,                   "ver_flip" );
    WRITE_CODE( sei.anticlockwiseRotation, 16, "anticlockwise_rotation" );
    WRITE_FLAG( sei.persistenceFlag,          "display_orientation_persistence_flag" );
  }
}

void SEIWriter::xWriteSEITemporalLevel0Index(const SEITemporalLevel0Index &sei)
{
  WRITE_CODE( sei.tl0Idx, 8 , "tl0_idx" );
  WRITE_CODE( sei.rapIdx, 8 , "rap_idx" );
}

void SEIWriter::xWriteSEIGradualDecodingRefreshInfo(const SEIGradualDecodingRefreshInfo &sei)
{
  WRITE_FLAG( sei.m_gdrForegroundFlag, "gdr_foreground_flag");
}

void SEIWriter::xWriteSEINoDisplay(const SEINoDisplay& /*sei*/)
{
}

void SEIWriter::xWriteSEISOPDescription(const SEISOPDescription& sei)
{
#if JVET_P0244_SPS_CLEAN_UP
  WRITE_CODE( sei.m_sopSeqParameterSetId, 4,        "sop_seq_parameter_set_id" );
#else
  WRITE_UVLC( sei.m_sopSeqParameterSetId,           "sop_seq_parameter_set_id"               );
#endif
  WRITE_UVLC( sei.m_numPicsInSopMinus1,             "num_pics_in_sop_minus1"               );
  for (uint32_t i = 0; i <= sei.m_numPicsInSopMinus1; i++)
  {
    WRITE_CODE( sei.m_sopDescVclNaluType[i], 6, "sop_desc_vcl_nalu_type" );
    WRITE_CODE( sei.m_sopDescTemporalId[i],  3, "sop_desc_temporal_id" );
    if (sei.m_sopDescVclNaluType[i] != NAL_UNIT_CODED_SLICE_IDR_W_RADL && sei.m_sopDescVclNaluType[i] != NAL_UNIT_CODED_SLICE_IDR_N_LP)
    {
      WRITE_UVLC( sei.m_sopDescStRpsIdx[i],           "sop_desc_st_rps_idx"               );
    }
    if (i > 0)
    {
      WRITE_SVLC( sei.m_sopDescPocDelta[i],           "sop_desc_poc_delta"               );
    }
  }
}

void SEIWriter::xWriteSEIScalableNesting(OutputBitstream& bs, const SEIScalableNesting& sei, const SPS *sps)
{
  WRITE_FLAG( sei.m_bitStreamSubsetFlag,             "bitstream_subset_flag"         );
  WRITE_FLAG( sei.m_nestingOpFlag,                   "nesting_op_flag      "         );
  if (sei.m_nestingOpFlag)
  {
    WRITE_FLAG( sei.m_defaultOpFlag,                 "default_op_flag"               );
    WRITE_UVLC( sei.m_nestingNumOpsMinus1,           "nesting_num_ops_minus1"        );
    for (uint32_t i = (sei.m_defaultOpFlag ? 1 : 0); i <= sei.m_nestingNumOpsMinus1; i++)
    {
      WRITE_CODE( sei.m_nestingMaxTemporalIdPlus1[i], 3,  "nesting_max_temporal_id_plus1" );
      WRITE_UVLC( sei.m_nestingOpIdx[i],                  "nesting_op_idx"                );
    }
  }
  else
  {
    WRITE_FLAG( sei.m_allLayersFlag,                      "all_layers_flag"               );
    if (!sei.m_allLayersFlag)
    {
      WRITE_CODE( sei.m_nestingNoOpMaxTemporalIdPlus1, 3, "nesting_no_op_max_temporal_id_plus1" );
      WRITE_UVLC( sei.m_nestingNumLayersMinus1,           "nesting_num_layers"                  );
      for (uint32_t i = 0; i <= sei.m_nestingNumLayersMinus1; i++)
      {
        WRITE_CODE( sei.m_nestingLayerId[i], 6,           "nesting_layer_id"              );
      }
    }
  }

  // byte alignment
  while ( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0 )
  {
    WRITE_FLAG( 0, "nesting_zero_bit" );
  }

  // write nested SEI messages
  HRD hrd;
  writeSEImessages(bs, sei.m_nestedSEIs, sps, hrd, true);
}

void SEIWriter::xWriteSEITempMotionConstrainedTileSets(const SEITempMotionConstrainedTileSets& sei)
{
  //uint32_t code;
  WRITE_FLAG((sei.m_mc_all_tiles_exact_sample_value_match_flag ? 1 : 0), "mc_all_tiles_exact_sample_value_match_flag");
  WRITE_FLAG((sei.m_each_tile_one_tile_set_flag                ? 1 : 0), "each_tile_one_tile_set_flag"               );

  if(!sei.m_each_tile_one_tile_set_flag)
  {
    WRITE_FLAG((sei.m_limited_tile_set_display_flag ? 1 : 0), "limited_tile_set_display_flag");
    WRITE_UVLC((sei.getNumberOfTileSets() - 1),               "num_sets_in_message_minus1"   );

    if(sei.getNumberOfTileSets() > 0)
    {
      for(int i = 0; i < sei.getNumberOfTileSets(); i++)
      {
        WRITE_UVLC(sei.tileSetData(i).m_mcts_id, "mcts_id");

        if(sei.m_limited_tile_set_display_flag)
        {
          WRITE_FLAG((sei.tileSetData(i).m_display_tile_set_flag ? 1 : 0), "display_tile_set_flag");
        }

        WRITE_UVLC((sei.tileSetData(i).getNumberOfTileRects() - 1), "num_tile_rects_in_set_minus1");

        for(int j = 0; j < sei.tileSetData(i).getNumberOfTileRects(); j++)
        {
          WRITE_UVLC(sei.tileSetData(i).topLeftTileIndex    (j), "top_left_tile_index");
          WRITE_UVLC(sei.tileSetData(i).bottomRightTileIndex(j), "bottom_right_tile_index");
        }

        if(!sei.m_mc_all_tiles_exact_sample_value_match_flag)
        {
          WRITE_FLAG((sei.tileSetData(i).m_exact_sample_value_match_flag ? 1 : 0), "exact_sample_value_match_flag");
        }

        WRITE_FLAG((sei.tileSetData(i).m_mcts_tier_level_idc_present_flag ? 1 : 0), "mcts_tier_level_idc_present_flag");

        if(sei.tileSetData(i).m_mcts_tier_level_idc_present_flag)
        {
          WRITE_FLAG((sei.tileSetData(i).m_mcts_tier_flag ? 1 : 0), "mcts_tier_flag");
          WRITE_CODE( sei.tileSetData(i).m_mcts_level_idc, 8,       "mcts_level_idc");
        }
      }
    }
  }
  else
  {
    WRITE_FLAG((sei.m_max_mcs_tier_level_idc_present_flag ? 1 : 0), "max_mcs_tier_level_idc_present_flag");

    if(sei.m_max_mcs_tier_level_idc_present_flag)
    {
      WRITE_FLAG((sei.m_max_mcts_tier_flag ? 1 : 0), "max_mcts_tier_flag");
      WRITE_CODE( sei.m_max_mcts_level_idc, 8,       "max_mcts_level_idc");
    }
  }
}

void SEIWriter::xWriteSEITimeCode(const SEITimeCode& sei)
{
  WRITE_CODE(sei.numClockTs, 2, "num_clock_ts");
  for(int i = 0; i < sei.numClockTs; i++)
  {
    const SEITimeSet &currentTimeSet = sei.timeSetArray[i];
    WRITE_FLAG(currentTimeSet.clockTimeStampFlag, "clock_time_stamp_flag");
    if(currentTimeSet.clockTimeStampFlag)
    {
      WRITE_FLAG(currentTimeSet.numUnitFieldBasedFlag, "units_field_based_flag");
      WRITE_CODE(currentTimeSet.countingType, 5, "counting_type");
      WRITE_FLAG(currentTimeSet.fullTimeStampFlag, "full_timestamp_flag");
      WRITE_FLAG(currentTimeSet.discontinuityFlag, "discontinuity_flag");
      WRITE_FLAG(currentTimeSet.cntDroppedFlag, "cnt_dropped_flag");
      WRITE_CODE(currentTimeSet.numberOfFrames, 9, "n_frames");
      if(currentTimeSet.fullTimeStampFlag)
      {
        WRITE_CODE(currentTimeSet.secondsValue, 6, "seconds_value");
        WRITE_CODE(currentTimeSet.minutesValue, 6, "minutes_value");
        WRITE_CODE(currentTimeSet.hoursValue, 5, "hours_value");
      }
      else
      {
        WRITE_FLAG(currentTimeSet.secondsFlag, "seconds_flag");
        if(currentTimeSet.secondsFlag)
        {
          WRITE_CODE(currentTimeSet.secondsValue, 6, "seconds_value");
          WRITE_FLAG(currentTimeSet.minutesFlag, "minutes_flag");
          if(currentTimeSet.minutesFlag)
          {
            WRITE_CODE(currentTimeSet.minutesValue, 6, "minutes_value");
            WRITE_FLAG(currentTimeSet.hoursFlag, "hours_flag");
            if(currentTimeSet.hoursFlag)
            {
              WRITE_CODE(currentTimeSet.hoursValue, 5, "hours_value");
            }
          }
        }
      }
      WRITE_CODE(currentTimeSet.timeOffsetLength, 5, "time_offset_length");
      if(currentTimeSet.timeOffsetLength > 0)
      {
        if(currentTimeSet.timeOffsetValue >= 0)
        {
          WRITE_CODE((uint32_t)currentTimeSet.timeOffsetValue, currentTimeSet.timeOffsetLength, "time_offset_value");
        }
        else
        {
          //  Two's complement conversion
          uint32_t offsetValue = ~(currentTimeSet.timeOffsetValue) + 1;
          offsetValue |= (1 << (currentTimeSet.timeOffsetLength-1));
          WRITE_CODE(offsetValue, currentTimeSet.timeOffsetLength, "time_offset_value");
        }
      }
    }
  }
}

void SEIWriter::xWriteSEIChromaResamplingFilterHint(const SEIChromaResamplingFilterHint &sei)
{
  WRITE_CODE(sei.m_verChromaFilterIdc, 8, "ver_chroma_filter_idc");
  WRITE_CODE(sei.m_horChromaFilterIdc, 8, "hor_chroma_filter_idc");
  WRITE_FLAG(sei.m_verFilteringFieldProcessingFlag, "ver_filtering_field_processing_flag");
  if(sei.m_verChromaFilterIdc == 1 || sei.m_horChromaFilterIdc == 1)
  {
    WRITE_UVLC(sei.m_targetFormatIdc, "target_format_idc");
    if(sei.m_verChromaFilterIdc == 1)
    {
      const int numVerticalFilter = (int)sei.m_verFilterCoeff.size();
      WRITE_UVLC(numVerticalFilter, "num_vertical_filters");
      if(numVerticalFilter > 0)
      {
        for(int i = 0; i < numVerticalFilter; i ++)
        {
          const int verTapLengthMinus1 = (int) sei.m_verFilterCoeff[i].size() - 1;
          WRITE_UVLC(verTapLengthMinus1, "ver_tap_length_minus_1");
          for(int j = 0; j < (verTapLengthMinus1 + 1); j ++)
          {
            WRITE_SVLC(sei.m_verFilterCoeff[i][j], "ver_filter_coeff");
          }
        }
      }
    }
    if(sei.m_horChromaFilterIdc == 1)
    {
      const int numHorizontalFilter = (int) sei.m_horFilterCoeff.size();
      WRITE_UVLC(numHorizontalFilter, "num_horizontal_filters");
      if(numHorizontalFilter > 0)
      {
        for(int i = 0; i < numHorizontalFilter; i ++)
        {
          const int horTapLengthMinus1 = (int) sei.m_horFilterCoeff[i].size() - 1;
          WRITE_UVLC(horTapLengthMinus1, "hor_tap_length_minus_1");
          for(int j = 0; j < (horTapLengthMinus1 + 1); j ++)
          {
            WRITE_SVLC(sei.m_horFilterCoeff[i][j], "hor_filter_coeff");
          }
        }
      }
    }
  }
}

void SEIWriter::xWriteSEIKneeFunctionInfo(const SEIKneeFunctionInfo &sei)
{
  WRITE_UVLC( sei.m_kneeId, "knee_function_id" );
  WRITE_FLAG( sei.m_kneeCancelFlag, "knee_function_cancel_flag" );
  if ( !sei.m_kneeCancelFlag )
  {
    WRITE_FLAG( sei.m_kneePersistenceFlag, "knee_function_persistence_flag" );
    WRITE_CODE( (uint32_t)sei.m_kneeInputDrange , 32,  "input_d_range" );
    WRITE_CODE( (uint32_t)sei.m_kneeInputDispLuminance, 32,  "input_disp_luminance" );
    WRITE_CODE( (uint32_t)sei.m_kneeOutputDrange, 32,  "output_d_range" );
    WRITE_CODE( (uint32_t)sei.m_kneeOutputDispLuminance, 32,  "output_disp_luminance" );
    WRITE_UVLC( sei.m_kneeNumKneePointsMinus1, "num_knee_points_minus1" );
    for(int i = 0; i <= sei.m_kneeNumKneePointsMinus1; i++ )
    {
      WRITE_CODE( (uint32_t)sei.m_kneeInputKneePoint[i], 10,"input_knee_point" );
      WRITE_CODE( (uint32_t)sei.m_kneeOutputKneePoint[i], 10, "output_knee_point" );
    }
  }
}

void SEIWriter::xWriteSEIColourRemappingInfo(const SEIColourRemappingInfo& sei)
{
  WRITE_UVLC( sei.m_colourRemapId,                             "colour_remap_id" );
  WRITE_FLAG( sei.m_colourRemapCancelFlag,                     "colour_remap_cancel_flag" );
  if( !sei.m_colourRemapCancelFlag )
  {
    WRITE_FLAG( sei.m_colourRemapPersistenceFlag,              "colour_remap_persistence_flag" );
    WRITE_FLAG( sei.m_colourRemapVideoSignalInfoPresentFlag,   "colour_remap_video_signal_info_present_flag" );
    if ( sei.m_colourRemapVideoSignalInfoPresentFlag )
    {
      WRITE_FLAG( sei.m_colourRemapFullRangeFlag,              "colour_remap_full_range_flag" );
      WRITE_CODE( sei.m_colourRemapPrimaries,               8, "colour_remap_primaries" );
      WRITE_CODE( sei.m_colourRemapTransferFunction,        8, "colour_remap_transfer_function" );
      WRITE_CODE( sei.m_colourRemapMatrixCoefficients,      8, "colour_remap_matrix_coefficients" );
    }
    WRITE_CODE( sei.m_colourRemapInputBitDepth,             8, "colour_remap_input_bit_depth" );
    WRITE_CODE( sei.m_colourRemapBitDepth,                  8, "colour_remap_bit_depth" );
    for( int c=0 ; c<3 ; c++ )
    {
      WRITE_CODE( sei.m_preLutNumValMinus1[c],              8, "pre_lut_num_val_minus1[c]" );
      if( sei.m_preLutNumValMinus1[c]>0 )
      {
        for( int i=0 ; i<=sei.m_preLutNumValMinus1[c] ; i++ )
        {
          WRITE_CODE( sei.m_preLut[c][i].codedValue,  (( sei.m_colourRemapInputBitDepth + 7 ) >> 3 ) << 3, "pre_lut_coded_value[c][i]" );
          WRITE_CODE( sei.m_preLut[c][i].targetValue, (( sei.m_colourRemapBitDepth      + 7 ) >> 3 ) << 3, "pre_lut_target_value[c][i]" );
        }
      }
    }
    WRITE_FLAG( sei.m_colourRemapMatrixPresentFlag,            "colour_remap_matrix_present_flag" );
    if( sei.m_colourRemapMatrixPresentFlag )
    {
      WRITE_CODE( sei.m_log2MatrixDenom,                    4, "log2_matrix_denom" );
      for( int c=0 ; c<3 ; c++ )
      {
        for( int i=0 ; i<3 ; i++ )
        {
          WRITE_SVLC( sei.m_colourRemapCoeffs[c][i],           "colour_remap_coeffs[c][i]" );
        }
      }
    }

    for( int c=0 ; c<3 ; c++ )
    {
      WRITE_CODE( sei.m_postLutNumValMinus1[c],             8, "m_postLutNumValMinus1[c]" );
      if( sei.m_postLutNumValMinus1[c]>0 )
      {
        for( int i=0 ; i<=sei.m_postLutNumValMinus1[c] ; i++ )
        {
          WRITE_CODE( sei.m_postLut[c][i].codedValue, (( sei.m_colourRemapBitDepth + 7 ) >> 3 ) << 3, "post_lut_coded_value[c][i]" );
          WRITE_CODE( sei.m_postLut[c][i].targetValue, (( sei.m_colourRemapBitDepth + 7 ) >> 3 ) << 3, "post_lut_target_value[c][i]" );
        }
      }
    }
  }
}
#endif

void SEIWriter::xWriteSEIMasteringDisplayColourVolume(const SEIMasteringDisplayColourVolume& sei)
{
  WRITE_CODE( sei.values.primaries[0][0],  16,  "display_primaries_x[0]" );
  WRITE_CODE( sei.values.primaries[0][1],  16,  "display_primaries_y[0]" );

  WRITE_CODE( sei.values.primaries[1][0],  16,  "display_primaries_x[1]" );
  WRITE_CODE( sei.values.primaries[1][1],  16,  "display_primaries_y[1]" );

  WRITE_CODE( sei.values.primaries[2][0],  16,  "display_primaries_x[2]" );
  WRITE_CODE( sei.values.primaries[2][1],  16,  "display_primaries_y[2]" );

  WRITE_CODE( sei.values.whitePoint[0],    16,  "white_point_x" );
  WRITE_CODE( sei.values.whitePoint[1],    16,  "white_point_y" );

  WRITE_CODE( sei.values.maxLuminance,     32,  "max_display_mastering_luminance" );
  WRITE_CODE( sei.values.minLuminance,     32,  "min_display_mastering_luminance" );
}
#endif

void SEIWriter::xWriteByteAlign()
{
  if( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    WRITE_FLAG( 1, "payload_bit_equal_to_one" );
    while( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0 )
    {
      WRITE_FLAG( 0, "payload_bit_equal_to_zero" );
    }
  }
}

#if HEVC_SEI || JVET_P0337_PORTING_SEI
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
void SEIWriter::xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei)
{
  WRITE_CODE(sei.m_preferredTransferCharacteristics, 8, "preferred_transfer_characteristics");
}
#endif
#if !JVET_P0337_PORTING_SEI
void SEIWriter::xWriteSEIGreenMetadataInfo(const SEIGreenMetadataInfo& sei)
{
  WRITE_CODE(sei.m_greenMetadataType, 8, "green_metadata_type");

  WRITE_CODE(sei.m_xsdMetricType, 8, "xsd_metric_type");
  WRITE_CODE(sei.m_xsdMetricValue, 16, "xsd_metric_value");
}
#endif
#endif

#if JVET_P0462_SEI360
void SEIWriter::xWriteSEIEquirectangularProjection(const SEIEquirectangularProjection &sei)
{
  WRITE_FLAG( sei.m_erpCancelFlag, "erp_cancel_flag" );
  if( !sei.m_erpCancelFlag )
  {
    WRITE_FLAG( sei.m_erpPersistenceFlag, "erp_persistence_flag" );
    WRITE_FLAG( sei.m_erpGuardBandFlag,   "erp_guard_band_flag" );
    WRITE_CODE( 0, 2, "erp_reserved_zero_2bits" );
    if ( sei.m_erpGuardBandFlag == 1)
    {
      WRITE_CODE( sei.m_erpGuardBandType,       3, "erp_guard_band_type" );  
      WRITE_CODE( sei.m_erpLeftGuardBandWidth,  8, "erp_left_guard_band_width" );  
      WRITE_CODE( sei.m_erpRightGuardBandWidth, 8, "erp_right_guard_band_width" );  
    }
  }
}

void SEIWriter::xWriteSEISphereRotation(const SEISphereRotation &sei)
{
  WRITE_FLAG( sei.m_sphereRotationCancelFlag, "sphere_rotation_cancel_flag" );
  if( !sei.m_sphereRotationCancelFlag )
  {
    WRITE_FLAG( sei.m_sphereRotationPersistenceFlag,    "sphere_rotation_persistence_flag" );
    WRITE_CODE( 0,                                   6, "sphere_rotation_reserved_zero_6bits" );
    WRITE_SCODE(sei.m_sphereRotationYaw,            32, "sphere_rotation_yaw" );  
    WRITE_SCODE(sei.m_sphereRotationPitch,          32, "sphere_rotation_pitch" );  
    WRITE_SCODE(sei.m_sphereRotationRoll,           32, "sphere_rotation_roll" );  
  }
}

void SEIWriter::xWriteSEIOmniViewport(const SEIOmniViewport &sei)
{
  WRITE_CODE( sei.m_omniViewportId,     10,    "omni_viewport_id" );
  WRITE_FLAG( sei.m_omniViewportCancelFlag, "omni_viewport_cancel_flag" );
  if ( !sei.m_omniViewportCancelFlag )
  {
    WRITE_FLAG( sei.m_omniViewportPersistenceFlag, "omni_viewport_persistence_flag" );
    const uint32_t numRegions = (uint32_t) sei.m_omniViewportRegions.size();
    WRITE_CODE( numRegions - 1, 4, "omni_viewport_cnt_minus1" );
    for(uint32_t region=0; region<numRegions; region++)
    {
      const SEIOmniViewport::OmniViewport &viewport=sei.m_omniViewportRegions[region];
      WRITE_SCODE( viewport.azimuthCentre,     32,  "omni_viewport_azimuth_centre"   );  
      WRITE_SCODE( viewport.elevationCentre,   32,  "omni_viewport_elevation_centre" );  
      WRITE_SCODE( viewport.tiltCentre,        32,  "omni_viewport_tilt_center" );  
      WRITE_CODE( viewport.horRange,           32, "omni_viewport_hor_range[i]" );
      WRITE_CODE( viewport.verRange,           32, "omni_viewport_ver_range[i]" );
    }
  }
}

void SEIWriter::xWriteSEIRegionWisePacking(const SEIRegionWisePacking &sei)
{
  WRITE_FLAG( sei.m_rwpCancelFlag,                                           "rwp_cancel_flag" );
  if(!sei.m_rwpCancelFlag)
  {
    WRITE_FLAG( sei.m_rwpPersistenceFlag,                                    "rwp_persistence_flag" );
    WRITE_FLAG( sei.m_constituentPictureMatchingFlag,                        "constituent_picture_matching_flag" );
    WRITE_CODE( 0, 5,                                                        "rwp_reserved_zero_5bits" );
    WRITE_CODE( (uint32_t)sei.m_numPackedRegions,                 8,             "num_packed_regions" );
    WRITE_CODE( (uint32_t)sei.m_projPictureWidth,                 32,            "proj_picture_width" );
    WRITE_CODE( (uint32_t)sei.m_projPictureHeight,                32,            "proj_picture_height" );
    WRITE_CODE( (uint32_t)sei.m_packedPictureWidth,               16,            "packed_picture_width" );
    WRITE_CODE( (uint32_t)sei.m_packedPictureHeight,              16,            "packed_picture_height" );
    for( int i=0; i < sei.m_numPackedRegions; i++ )
    { 
      WRITE_CODE( 0, 4,                                                      "rwp_reserved_zero_4bits" );
      WRITE_CODE( (uint32_t)sei.m_rwpTransformType[i],            3,             "rwp_tTransform_type" );
      WRITE_FLAG( sei.m_rwpGuardBandFlag[i],                                 "rwp_guard_band_flag" );
      WRITE_CODE( (uint32_t)sei.m_projRegionWidth[i],             32,            "proj_region_width" );
      WRITE_CODE( (uint32_t)sei.m_projRegionHeight[i],            32,            "proj_region_height" );
      WRITE_CODE( (uint32_t)sei.m_rwpProjRegionTop[i],            32,            "rwp_proj_regionTop" );
      WRITE_CODE( (uint32_t)sei.m_projRegionLeft[i],              32,            "proj_region_left" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionWidth[i],           16,            "packed_region_width" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionHeight[i],          16,            "packed_region_height" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionTop[i],             16,            "packed_region_top" );
      WRITE_CODE( (uint32_t)sei.m_packedRegionLeft[i],            16,            "packed_region_left" );
      if( sei.m_rwpGuardBandFlag[i] )
      {
        WRITE_CODE( (uint32_t)sei.m_rwpLeftGuardBandWidth[i],     8,             "rwp_left_guard_band_width");
        WRITE_CODE( (uint32_t)sei.m_rwpRightGuardBandWidth[i],    8,             "rwp_right_guard_band_width");
        WRITE_CODE( (uint32_t)sei.m_rwpTopGuardBandHeight[i],     8,             "rwp_top_guard_band_height");
        WRITE_CODE( (uint32_t)sei. m_rwpBottomGuardBandHeight[i], 8,             "rwp_bottom_guard_band_height");
        WRITE_FLAG( sei.m_rwpGuardBandNotUsedForPredFlag[i],                 "rwp_guard_band_not_used_forPred_flag" );
        for( int j=0; j < 4; j++ )
        {
          WRITE_CODE( (uint32_t)sei.m_rwpGuardBandType[i*4 + j],  3,             "rwp_guard_band_type");
        }
        WRITE_CODE( 0, 3,                                                    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}
#endif

#if JVET_P0597_GCMP_SEI
void SEIWriter::xWriteSEIGeneralizedCubemapProjection(const SEIGeneralizedCubemapProjection &sei)
{
  WRITE_FLAG( sei.m_gcmpCancelFlag,                           "gcmp_cancel_flag" );
  if (!sei.m_gcmpCancelFlag)
  {
    WRITE_FLAG( sei.m_gcmpPersistenceFlag,                    "gcmp_persistence_flag" );
    WRITE_CODE( sei.m_gcmpPackingType,                     3, "gcmp_packing_type" );  
    WRITE_CODE( sei.m_gcmpMappingFunctionType,             2, "gcmp_mapping_function_type" );
    int numFace = sei.m_gcmpPackingType == 4 || sei.m_gcmpPackingType == 5 ? 5 : 6;
    for (int i = 0; i < numFace; i++)
    {
      WRITE_CODE( sei.m_gcmpFaceIndex[i],                  3, "gcmp_face_index" );  
      WRITE_CODE( sei.m_gcmpFaceRotation[i],               2, "gcmp_face_rotation" );  
      if (sei.m_gcmpMappingFunctionType == 2)
      {
        WRITE_CODE( sei.m_gcmpFunctionCoeffU[i],           7, "gcmp_function_coeff_u" );  
        WRITE_FLAG( sei.m_gcmpFunctionUAffectedByVFlag[i],    "gcmp_function_u_affected_by_v_flag" );
        WRITE_CODE( sei.m_gcmpFunctionCoeffV[i],           7, "gcmp_function_coeff_v" );  
        WRITE_FLAG( sei.m_gcmpFunctionVAffectedByUFlag[i],    "gcmp_function_v_affected_by_u_flag" );
      }
    }
    WRITE_FLAG( sei.m_gcmpGuardBandFlag,                      "gcmp_guard_band_flag" );
    if (sei.m_gcmpGuardBandFlag)
    {
      WRITE_FLAG( sei.m_gcmpGuardBandBoundaryType,            "gcmp_guard_band_boundary_type" );  
      WRITE_CODE( sei.m_gcmpGuardBandSamplesMinus1,        4, "gcmp_guard_band_samples_minus1" );  
    }
  }
}
#endif

#if JVET_P0984_SEI_SUBPIC_LEVEL
void SEIWriter::xWriteSEISubpictureLevelInfo(const SEISubpicureLevelInfo &sei, const SPS* sps)
{
  WRITE_CODE( (uint32_t)sei.m_sliSeqParameterSetId, 4,                        "sli_seq_parameter_set_id");
  CHECK(sei.m_numRefLevels < 1, "SEISubpicureLevelInfo: numRefLevels must be greater than zero");
  CHECK(sei.m_numRefLevels != (int)sei.m_refLevelIdc.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of levels");
  if (sei.m_explicitFractionPresentFlag)
  {
    CHECK(sei.m_numRefLevels != (int)sei.m_refLevelFraction.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of fractions");
  }
  WRITE_CODE( (uint32_t)sei.m_numRefLevels - 1, 3,                            "num_ref_levels_minus1");
  WRITE_FLAG(           sei.m_explicitFractionPresentFlag,                    "explicit_fraction_present_flag");

  for (int i=0; i<sei.m_numRefLevels; i++)
  {
    WRITE_CODE( (uint32_t)sei.m_refLevelIdc[i], 8,                            "ref_level_idc[i]");
    if (sei.m_explicitFractionPresentFlag)
    {
      CHECK(sps->getNumSubPics() != (int)sei.m_refLevelFraction[i].size(),    "SEISubpicureLevelInfo: number of fractions differs from number of subpictures");
      for (int j = 0; j < sps->getNumSubPics(); j++)
      {
        WRITE_CODE( (uint32_t)sei.m_refLevelFraction[i][j], 8,                "ref_level_fraction_minus1[i][j]");
      }
    }
  }
}
#endif

#if JVET_P0450_SEI_SARI
void SEIWriter::xWriteSEISampleAspectRatioInfo(const SEISampleAspectRatioInfo &sei)
{
  WRITE_FLAG( sei.m_sariCancelFlag,                                           "sari_cancel_flag" );
  if(!sei.m_sariCancelFlag)
  {
    WRITE_FLAG( sei.m_sariPersistenceFlag,                                    "sari_persistence_flag" );
    WRITE_CODE( (uint32_t)sei.m_sariAspectRatioIdc, 8,                        "sari_aspect_ratio_idc");
    if (sei.m_sariAspectRatioIdc == 255)
    {
      WRITE_CODE( (uint32_t)sei.m_sariSarWidth, 16,                           "sari_sar_width");
      WRITE_CODE( (uint32_t)sei.m_sariSarHeight, 16,                           "sari_sar_height");
    }
  }
}
#endif

#if JVET_P0337_PORTING_SEI
void SEIWriter::xWriteSEIUserDataRegistered(const SEIUserDataRegistered &sei)
{
  WRITE_CODE((sei.m_ituCountryCode>255) ? 0xff : sei.m_ituCountryCode, 8, "itu_t_t35_country_code");
  if (sei.m_ituCountryCode >= 255)
  {
    assert(sei.m_ituCountryCode < 255 + 256);
    WRITE_CODE(sei.m_ituCountryCode - 255, 8, "itu_t_t35_country_code_extension_byte");
  }
  for (uint32_t i = 0; i<sei.m_userData.size(); i++)
  {
    WRITE_CODE(sei.m_userData[i], 8, "itu_t_t35_payload_byte");
  }
}

void SEIWriter::xWriteSEIFilmGrainCharacteristics(const SEIFilmGrainCharacteristics &sei)
{
  WRITE_FLAG(sei.m_filmGrainCharacteristicsCancelFlag, "film_grain_characteristics_cancel_flag");
  if (!sei.m_filmGrainCharacteristicsCancelFlag)
  {
    WRITE_CODE(sei.m_filmGrainModelId, 2, "film_grain_model_id");
    WRITE_FLAG(sei.m_separateColourDescriptionPresentFlag, "separate_colour_description_present_flag");
    if (sei.m_separateColourDescriptionPresentFlag)
    {
      WRITE_CODE(sei.m_filmGrainBitDepthLumaMinus8, 3, "film_grain_bit_depth_luma_minus8");
      WRITE_CODE(sei.m_filmGrainBitDepthChromaMinus8, 3, "film_grain_bit_depth_chroma_minus8");
      WRITE_FLAG(sei.m_filmGrainFullRangeFlag, "film_grain_full_range_flag");
      WRITE_CODE(sei.m_filmGrainColourPrimaries, 8, "film_grain_colour_primaries");
      WRITE_CODE(sei.m_filmGrainTransferCharacteristics, 8, "film_grain_transfer_characteristics");
      WRITE_CODE(sei.m_filmGrainMatrixCoeffs, 8, "film_grain_matrix_coeffs");
    }
    WRITE_CODE(sei.m_blendingModeId, 2, "blending_mode_id");
    WRITE_CODE(sei.m_log2ScaleFactor, 4, "log2_scale_factor");
    for (int c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t)cm.intensityValues.size();
      const uint32_t numModelValues = cm.numModelValues;
      WRITE_FLAG(sei.m_compModel[c].presentFlag && numIntensityIntervals>0 && numModelValues>0, "comp_model_present_flag[c]");
    }
    for (uint32_t c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.m_compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t)cm.intensityValues.size();
      const uint32_t numModelValues = cm.numModelValues;
      if (cm.presentFlag && numIntensityIntervals>0 && numModelValues>0)
      {
        assert(numIntensityIntervals <= 256);
        assert(numModelValues <= 256);
        WRITE_CODE(numIntensityIntervals - 1, 8, "num_intensity_intervals_minus1[c]");
        WRITE_CODE(numModelValues - 1, 8, "num_model_values_minus1[c]");
        for (uint32_t interval = 0; interval<numIntensityIntervals; interval++)
        {
          const SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv = cm.intensityValues[interval];
          WRITE_CODE(cmiv.intensityIntervalLowerBound, 8, "intensity_interval_lower_bound[c][i]");
          WRITE_CODE(cmiv.intensityIntervalUpperBound, 8, "intensity_interval_upper_bound[c][i]");
          assert(cmiv.compModelValue.size() == numModelValues);
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            WRITE_SVLC(cmiv.compModelValue[j], "comp_model_value[c][i]");
          }
        }
      }
    } // for c
    WRITE_FLAG(sei.m_filmGrainCharacteristicsPersistenceFlag, "film_grain_characteristics_persistence_flag");
  } // cancel flag
}

void SEIWriter::xWriteSEIContentLightLevelInfo(const SEIContentLightLevelInfo& sei)
{
  WRITE_CODE( sei.m_maxContentLightLevel,    16, "max_content_light_level"     );
  WRITE_CODE( sei.m_maxPicAverageLightLevel, 16, "max_pic_average_light_level" );
}

void SEIWriter::xWriteSEIAmbientViewingEnvironment(const SEIAmbientViewingEnvironment& sei)
{
  WRITE_CODE(sei.m_ambientIlluminance, 32, "ambient_illuminance" );
  WRITE_CODE(sei.m_ambientLightX,      16, "ambient_light_x" );
  WRITE_CODE(sei.m_ambientLightY,      16, "ambient_light_y" );
}

void SEIWriter::xWriteSEIContentColourVolume(const SEIContentColourVolume &sei)
{
  WRITE_FLAG(sei.m_ccvCancelFlag, "ccv_cancel_flag");
  if (!sei.m_ccvCancelFlag)
  {
    WRITE_FLAG(sei.m_ccvPersistenceFlag, "ccv_persistence_flag");
    WRITE_FLAG(sei.m_ccvPrimariesPresentFlag, "ccv_primaries_present_flag");
    WRITE_FLAG(sei.m_ccvMinLuminanceValuePresentFlag, "ccv_min_luminance_value_present_flag");
    WRITE_FLAG(sei.m_ccvMaxLuminanceValuePresentFlag, "ccv_max_luminance_value_present_flag");
    WRITE_FLAG(sei.m_ccvAvgLuminanceValuePresentFlag, "ccv_avg_luminance_value_present_flag");

    if (sei.m_ccvPrimariesPresentFlag == true)
    {
      for (int i = 0; i < MAX_NUM_COMPONENT; i++)
      {
        WRITE_SCODE((int32_t)sei.m_ccvPrimariesX[i], 32, "ccv_primaries_x[i]");
        WRITE_SCODE((int32_t)sei.m_ccvPrimariesY[i], 32, "ccv_primaries_y[i]");
      }
    }

    if (sei.m_ccvMinLuminanceValuePresentFlag == true)
    {
      WRITE_CODE((uint32_t)sei.m_ccvMinLuminanceValue, 32, "ccv_min_luminance_value");
    }
    if (sei.m_ccvMinLuminanceValuePresentFlag == true)
    {
      WRITE_CODE((uint32_t)sei.m_ccvMaxLuminanceValue, 32, "ccv_max_luminance_value");
    }
    if (sei.m_ccvMinLuminanceValuePresentFlag == true)
    {
      WRITE_CODE((uint32_t)sei.m_ccvAvgLuminanceValue, 32, "ccv_avg_luminance_value");
    }
  }
}
#endif

//! \}
