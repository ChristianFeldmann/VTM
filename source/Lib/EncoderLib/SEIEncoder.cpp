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

#include "CommonLib/CommonDef.h"
#include "CommonLib/SEI.h"
#include "EncGOP.h"
#include "EncLib.h"

uint32_t calcMD5(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcCRC(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcChecksum(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths);
std::string hashToString(const PictureHash &digest, int numChar);

//! \ingroup EncoderLib
//! \{

#if HEVC_SEI || JVET_P0337_PORTING_SEI
#if !JVET_P0337_PORTING_SEI
void SEIEncoder::initSEIActiveParameterSets (SEIActiveParameterSets *seiActiveParameterSets, const SPS *sps)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiActiveParameterSets!=NULL), "Unspecified error");
  CHECK(!(sps!=NULL), "Unspecified error");

  seiActiveParameterSets->m_selfContainedCvsFlag = false;
  seiActiveParameterSets->m_noParameterSetUpdateFlag = false;
  seiActiveParameterSets->numSpsIdsMinus1 = 0;
  seiActiveParameterSets->activeSeqParameterSetId.resize(seiActiveParameterSets->numSpsIdsMinus1 + 1);
  seiActiveParameterSets->activeSeqParameterSetId[0] = sps->getSPSId();
}
#endif
void SEIEncoder::initSEIFramePacking(SEIFramePacking *seiFramePacking, int currPicNum)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiFramePacking!=NULL), "Unspecified error");

  seiFramePacking->m_arrangementId = m_pcCfg->getFramePackingArrangementSEIId();
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType = m_pcCfg->getFramePackingArrangementSEIType();
  CHECK(!((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) ), "Unspecified error");
  seiFramePacking->m_quincunxSamplingFlag = m_pcCfg->getFramePackingArrangementSEIQuincunx();
  seiFramePacking->m_contentInterpretationType = m_pcCfg->getFramePackingArrangementSEIInterpretation();
  seiFramePacking->m_spatialFlippingFlag = 0;
  seiFramePacking->m_frame0FlippedFlag = 0;
  seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && (currPicNum&1) );
  seiFramePacking->m_frame0SelfContainedFlag = 0;
  seiFramePacking->m_frame1SelfContainedFlag = 0;
  seiFramePacking->m_frame0GridPositionX = 0;
  seiFramePacking->m_frame0GridPositionY = 0;
  seiFramePacking->m_frame1GridPositionX = 0;
  seiFramePacking->m_frame1GridPositionY = 0;
  seiFramePacking->m_arrangementReservedByte = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio = 0;
}

#if !JVET_P0337_PORTING_SEI
void SEIEncoder::initSEISegmentedRectFramePacking(SEISegmentedRectFramePacking *seiSegmentedRectFramePacking)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiSegmentedRectFramePacking!=NULL), "Unspecified error");

  seiSegmentedRectFramePacking->m_arrangementCancelFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEICancel();
  seiSegmentedRectFramePacking->m_contentInterpretationType = m_pcCfg->getSegmentedRectFramePackingArrangementSEIType();
  seiSegmentedRectFramePacking->m_arrangementPersistenceFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEIPersistence();
}

void SEIEncoder::initSEIDisplayOrientation(SEIDisplayOrientation* seiDisplayOrientation)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiDisplayOrientation!=NULL), "Unspecified error");

  seiDisplayOrientation->cancelFlag = false;
  seiDisplayOrientation->horFlip = false;
  seiDisplayOrientation->verFlip = false;
  seiDisplayOrientation->anticlockwiseRotation = m_pcCfg->getDisplayOrientationSEIAngle();
}

void SEIEncoder::initSEIToneMappingInfo(SEIToneMappingInfo *seiToneMappingInfo)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiToneMappingInfo!=NULL), "Unspecified error");

  seiToneMappingInfo->m_toneMapId = m_pcCfg->getTMISEIToneMapId();
  seiToneMappingInfo->m_toneMapCancelFlag = m_pcCfg->getTMISEIToneMapCancelFlag();
  seiToneMappingInfo->m_toneMapPersistenceFlag = m_pcCfg->getTMISEIToneMapPersistenceFlag();

  seiToneMappingInfo->m_codedDataBitDepth = m_pcCfg->getTMISEICodedDataBitDepth();
  CHECK(!(seiToneMappingInfo->m_codedDataBitDepth >= 8 && seiToneMappingInfo->m_codedDataBitDepth <= 14), "Unspecified error");
  seiToneMappingInfo->m_targetBitDepth = m_pcCfg->getTMISEITargetBitDepth();
  CHECK(!(seiToneMappingInfo->m_targetBitDepth >= 1 && seiToneMappingInfo->m_targetBitDepth <= 17), "Unspecified error");
  seiToneMappingInfo->m_modelId = m_pcCfg->getTMISEIModelID();
  CHECK(!(seiToneMappingInfo->m_modelId >=0 &&seiToneMappingInfo->m_modelId<=4), "Unspecified error");

  switch( seiToneMappingInfo->m_modelId)
  {
  case 0:
    {
      seiToneMappingInfo->m_minValue = m_pcCfg->getTMISEIMinValue();
      seiToneMappingInfo->m_maxValue = m_pcCfg->getTMISEIMaxValue();
      break;
    }
  case 1:
    {
      seiToneMappingInfo->m_sigmoidMidpoint = m_pcCfg->getTMISEISigmoidMidpoint();
      seiToneMappingInfo->m_sigmoidWidth = m_pcCfg->getTMISEISigmoidWidth();
      break;
    }
  case 2:
    {
      uint32_t num = 1u<<(seiToneMappingInfo->m_targetBitDepth);
      seiToneMappingInfo->m_startOfCodedInterval.resize(num);
      int* ptmp = m_pcCfg->getTMISEIStartOfCodedInterva();
      if(ptmp)
      {
        for(int i=0; i<num;i++)
        {
          seiToneMappingInfo->m_startOfCodedInterval[i] = ptmp[i];
        }
      }
      break;
    }
  case 3:
    {
      seiToneMappingInfo->m_numPivots = m_pcCfg->getTMISEINumPivots();
      seiToneMappingInfo->m_codedPivotValue.resize(seiToneMappingInfo->m_numPivots);
      seiToneMappingInfo->m_targetPivotValue.resize(seiToneMappingInfo->m_numPivots);
      int* ptmpcoded = m_pcCfg->getTMISEICodedPivotValue();
      int* ptmptarget = m_pcCfg->getTMISEITargetPivotValue();
      if(ptmpcoded&&ptmptarget)
      {
        for(int i=0; i<(seiToneMappingInfo->m_numPivots);i++)
        {
          seiToneMappingInfo->m_codedPivotValue[i]=ptmpcoded[i];
          seiToneMappingInfo->m_targetPivotValue[i]=ptmptarget[i];
        }
      }
      break;
    }
  case 4:
    {
      seiToneMappingInfo->m_cameraIsoSpeedIdc = m_pcCfg->getTMISEICameraIsoSpeedIdc();
      seiToneMappingInfo->m_cameraIsoSpeedValue = m_pcCfg->getTMISEICameraIsoSpeedValue();
      CHECK(!( seiToneMappingInfo->m_cameraIsoSpeedValue !=0 ), "Unspecified error");
      seiToneMappingInfo->m_exposureIndexIdc = m_pcCfg->getTMISEIExposurIndexIdc();
      seiToneMappingInfo->m_exposureIndexValue = m_pcCfg->getTMISEIExposurIndexValue();
      CHECK(!( seiToneMappingInfo->m_exposureIndexValue !=0 ), "Unspecified error");
      seiToneMappingInfo->m_exposureCompensationValueSignFlag = m_pcCfg->getTMISEIExposureCompensationValueSignFlag();
      seiToneMappingInfo->m_exposureCompensationValueNumerator = m_pcCfg->getTMISEIExposureCompensationValueNumerator();
      seiToneMappingInfo->m_exposureCompensationValueDenomIdc = m_pcCfg->getTMISEIExposureCompensationValueDenomIdc();
      seiToneMappingInfo->m_refScreenLuminanceWhite = m_pcCfg->getTMISEIRefScreenLuminanceWhite();
      seiToneMappingInfo->m_extendedRangeWhiteLevel = m_pcCfg->getTMISEIExtendedRangeWhiteLevel();
      CHECK(!( seiToneMappingInfo->m_extendedRangeWhiteLevel >= 100 ), "Unspecified error");
      seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue = m_pcCfg->getTMISEINominalBlackLevelLumaCodeValue();
      seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue = m_pcCfg->getTMISEINominalWhiteLevelLumaCodeValue();
      CHECK(!( seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue > seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue ), "Unspecified error");
      seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue = m_pcCfg->getTMISEIExtendedWhiteLevelLumaCodeValue();
      CHECK(!( seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue >= seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue ), "Unspecified error");
      break;
    }
  default:
    {
      CHECK(!(!"Undefined SEIToneMapModelId"), "Unspecified error");
      break;
    }
  }
}

void SEIEncoder::initSEISOPDescription(SEISOPDescription *sopDescriptionSEI, Slice *slice, int picInGOP, int lastIdr, int currGOPSize)
{
}
#endif
#endif
void SEIEncoder::initSEIBufferingPeriod(SEIBufferingPeriod *bufferingPeriodSEI, bool noLeadingPictures)
{
  CHECK(!(m_isInitialized), "bufferingPeriodSEI already initialized");
  CHECK(!(bufferingPeriodSEI != nullptr), "Need a bufferingPeriodSEI for initialization (got nullptr)");

  uint32_t uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
  bufferingPeriodSEI->m_bpNalCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpVclCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpMaxSubLayers = m_pcCfg->getMaxTempLayer() ;
#if JVET_P0446_BP_CPB_CNT_FIX
  bufferingPeriodSEI->m_bpCpbCnt = 1;
#endif
  for(int i=0; i < bufferingPeriodSEI->m_bpMaxSubLayers; i++)
  {
#if !JVET_P0446_BP_CPB_CNT_FIX
    bufferingPeriodSEI->m_bpCpbCnt[i] = 1;
    for(int j=0; j < bufferingPeriodSEI->m_bpCpbCnt[i]; j++)
#else
    for(int j=0; j < bufferingPeriodSEI->m_bpCpbCnt; j++)
#endif
    {
      bufferingPeriodSEI->m_initialCpbRemovalDelay[j][i][0] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalDelay[j][i][1] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[j][i][0] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[j][i][1] = uiInitialCpbRemovalDelay;
    }
  }
#if JVET_P0446_CONCATENATION
  // We don't set concatenation_flag here. max_initial_removal_delay_for_concatenation depends on the usage scenario.
  // The parameters could be added to config file, but as long as the initialisation of generic buffering parameters is
  // not controllable, it does not seem to make sense to provide settings for these.
  bufferingPeriodSEI->m_concatenationFlag = false;
  bufferingPeriodSEI->m_maxInitialRemovalDelayForConcatenation = uiInitialCpbRemovalDelay;
#endif

#if JVET_P0202_P0203_FIX_HRD_RELATED_SEI
#if JVET_P1004_REMOVE_BRICKS
  bufferingPeriodSEI->m_bpDecodingUnitHrdParamsPresentFlag = m_pcCfg->getNoPicPartitionFlag() == false;
#else
  bufferingPeriodSEI->m_bpDecodingUnitHrdParamsPresentFlag = (m_pcCfg->getSliceMode() > 0) || (m_pcCfg->getSliceSegmentMode() > 0);
#endif
  bufferingPeriodSEI->m_decodingUnitCpbParamsInPicTimingSeiFlag = !m_pcCfg->getDecodingUnitInfoSEIEnabled();
#endif

  bufferingPeriodSEI->m_initialCpbRemovalDelayLength = 16;                  // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  // Note: The following parameters require some knowledge about the GOP structure.
  //       Using getIntraPeriod() should be avoided though, because it assumes certain GOP
  //       properties, which are only valid in CTC.
  //       Still copying this setting from HM for consistency, improvements welcome
  bool isRandomAccess  = m_pcCfg->getIntraPeriod() > 0;
  if( isRandomAccess )
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 6;                        // 32 = 2^5 (plus 1)
    bufferingPeriodSEI->m_dpbOutputDelayLength =  6;                        // 32 + 3 = 2^6
  }
  else
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 9;                        // max. 2^10
    bufferingPeriodSEI->m_dpbOutputDelayLength =  9;                        // max. 2^10
  }
  bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength = 7;               // ceil( log2( tick_divisor_minus2 + 2 ) )
  bufferingPeriodSEI->m_dpbOutputDelayDuLength = bufferingPeriodSEI->m_dpbOutputDelayLength + bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength;
  //for the concatenation, it can be set to one during splicing.
  bufferingPeriodSEI->m_concatenationFlag = 0;
  //since the temporal layer HRDParameters is not ready, we assumed it is fixed
  bufferingPeriodSEI->m_auCpbRemovalDelayDelta = 1;
  bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag = m_pcCfg->getBpDeltasGOPStructure() ;
  if (bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag)
  {
    switch (m_pcCfg->getGOPSize())
    {
      case 8:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 7;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
        }
      }
        break;
      case 16:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 4;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[5]          = 7;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[6]          = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[7]          = 14;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[8]          = 15;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas         = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]          = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]          = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]          = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]          = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]          = 7;
        }
      }
        break;
      default:
      {
        THROW("m_cpbRemovalDelayDelta not applicable for the GOP size");
      }
        break;
    }
  }
#if JVET_P0446_ALT_CPB
  // A commercial encoder should track the buffer state for all layers and sub-layers
  // to ensure CPB conformance. Such tracking is required for calculating alternative
  // CPB parameters.
  // Unfortunately VTM does not have such tracking. Thus we cannot encode alternative 
  // CPB parameters here.
  bufferingPeriodSEI->m_altCpbParamsPresentFlag = false;
  bufferingPeriodSEI->m_useAltCpbParamsFlag = false;
#endif
}

#if JVET_P0462_SEI360
void SEIEncoder::initSEIErp(SEIEquirectangularProjection* seiEquirectangularProjection)
{
  CHECK(!(m_isInitialized), "seiEquirectangularProjection already initialized");
  CHECK(!(seiEquirectangularProjection != nullptr), "Need a seiEquirectangularProjection for initialization (got nullptr)");

  seiEquirectangularProjection->m_erpCancelFlag = m_pcCfg->getErpSEICancelFlag();
  if (!seiEquirectangularProjection->m_erpCancelFlag)
  {
    seiEquirectangularProjection->m_erpPersistenceFlag   = m_pcCfg->getErpSEIPersistenceFlag();
    seiEquirectangularProjection->m_erpGuardBandFlag     = m_pcCfg->getErpSEIGuardBandFlag();
    if (seiEquirectangularProjection->m_erpGuardBandFlag == 1)
    {
      seiEquirectangularProjection->m_erpGuardBandType       = m_pcCfg->getErpSEIGuardBandType();
      seiEquirectangularProjection->m_erpLeftGuardBandWidth  = m_pcCfg->getErpSEILeftGuardBandWidth();
      seiEquirectangularProjection->m_erpRightGuardBandWidth = m_pcCfg->getErpSEIRightGuardBandWidth();
    }
  }
}

void SEIEncoder::initSEISphereRotation(SEISphereRotation* seiSphereRotation)
{
  CHECK(!(m_isInitialized), "seiSphereRotation already initialized");
  CHECK(!(seiSphereRotation != nullptr), "Need a seiSphereRotation for initialization (got nullptr)");

  seiSphereRotation->m_sphereRotationCancelFlag = m_pcCfg->getSphereRotationSEICancelFlag();
  if ( !seiSphereRotation->m_sphereRotationCancelFlag )
  {
    seiSphereRotation->m_sphereRotationPersistenceFlag = m_pcCfg->getSphereRotationSEIPersistenceFlag();
    seiSphereRotation->m_sphereRotationYaw = m_pcCfg->getSphereRotationSEIYaw();
    seiSphereRotation->m_sphereRotationPitch = m_pcCfg->getSphereRotationSEIPitch();
    seiSphereRotation->m_sphereRotationRoll = m_pcCfg->getSphereRotationSEIRoll();
  }
}

void SEIEncoder::initSEIOmniViewport(SEIOmniViewport* seiOmniViewport)
{
  CHECK(!(m_isInitialized), "seiOmniViewport already initialized");
  CHECK(!(seiOmniViewport != nullptr), "Need a seiOmniViewport for initialization (got nullptr)");

  seiOmniViewport->m_omniViewportId = m_pcCfg->getOmniViewportSEIId();
  seiOmniViewport->m_omniViewportCancelFlag = m_pcCfg->getOmniViewportSEICancelFlag();
  if ( !seiOmniViewport->m_omniViewportCancelFlag )
  {
    seiOmniViewport->m_omniViewportPersistenceFlag = m_pcCfg->getOmniViewportSEIPersistenceFlag();
    seiOmniViewport->m_omniViewportCntMinus1 = m_pcCfg->getOmniViewportSEICntMinus1();

    seiOmniViewport->m_omniViewportRegions.resize(seiOmniViewport->m_omniViewportCntMinus1+1);
    for (uint32_t i = 0; i <= seiOmniViewport->m_omniViewportCntMinus1; i++)
    {
      SEIOmniViewport::OmniViewport &viewport = seiOmniViewport->m_omniViewportRegions[i];
      viewport.azimuthCentre   = m_pcCfg->getOmniViewportSEIAzimuthCentre(i);
      viewport.elevationCentre = m_pcCfg->getOmniViewportSEIElevationCentre(i);
      viewport.tiltCentre      = m_pcCfg->getOmniViewportSEITiltCentre(i);
      viewport.horRange        = m_pcCfg->getOmniViewportSEIHorRange(i);
      viewport.verRange        = m_pcCfg->getOmniViewportSEIVerRange(i);
    }
  }
}

void SEIEncoder::initSEIRegionWisePacking(SEIRegionWisePacking *seiRegionWisePacking)
{
  CHECK(!(m_isInitialized), "seiRegionWisePacking already initialized");
  CHECK(!(seiRegionWisePacking != nullptr), "Need a seiRegionWisePacking for initialization (got nullptr)");

  seiRegionWisePacking->m_rwpCancelFlag                          = m_pcCfg->getRwpSEIRwpCancelFlag();
  seiRegionWisePacking->m_rwpPersistenceFlag                     = m_pcCfg->getRwpSEIRwpPersistenceFlag();
  seiRegionWisePacking->m_constituentPictureMatchingFlag         = m_pcCfg->getRwpSEIConstituentPictureMatchingFlag();
  seiRegionWisePacking->m_numPackedRegions                       = m_pcCfg->getRwpSEINumPackedRegions();
  seiRegionWisePacking->m_projPictureWidth                       = m_pcCfg->getRwpSEIProjPictureWidth();
  seiRegionWisePacking->m_projPictureHeight                      = m_pcCfg->getRwpSEIProjPictureHeight();
  seiRegionWisePacking->m_packedPictureWidth                     = m_pcCfg->getRwpSEIPackedPictureWidth();
  seiRegionWisePacking->m_packedPictureHeight                    = m_pcCfg->getRwpSEIPackedPictureHeight();
  seiRegionWisePacking->m_rwpTransformType.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpProjRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpLeftGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpRightGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpTopGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpBottomGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandType.resize(4*seiRegionWisePacking->m_numPackedRegions); 
  for( int i=0; i < seiRegionWisePacking->m_numPackedRegions; i++ )
  {
    seiRegionWisePacking->m_rwpTransformType[i]                  = m_pcCfg->getRwpSEIRwpTransformType(i);
    seiRegionWisePacking->m_rwpGuardBandFlag[i]                  = m_pcCfg->getRwpSEIRwpGuardBandFlag(i);
    seiRegionWisePacking->m_projRegionWidth[i]                   = m_pcCfg->getRwpSEIProjRegionWidth(i);
    seiRegionWisePacking->m_projRegionHeight[i]                  = m_pcCfg->getRwpSEIProjRegionHeight(i);
    seiRegionWisePacking->m_rwpProjRegionTop[i]                  = m_pcCfg->getRwpSEIRwpSEIProjRegionTop(i);
    seiRegionWisePacking->m_projRegionLeft[i]                    = m_pcCfg->getRwpSEIProjRegionLeft(i);
    seiRegionWisePacking->m_packedRegionWidth[i]                 = m_pcCfg->getRwpSEIPackedRegionWidth(i);
    seiRegionWisePacking->m_packedRegionHeight[i]                = m_pcCfg->getRwpSEIPackedRegionHeight(i);
    seiRegionWisePacking->m_packedRegionTop[i]                   = m_pcCfg->getRwpSEIPackedRegionTop(i);
    seiRegionWisePacking->m_packedRegionLeft[i]                  = m_pcCfg->getRwpSEIPackedRegionLeft(i);
    if( seiRegionWisePacking->m_rwpGuardBandFlag[i] )
    {
      seiRegionWisePacking->m_rwpLeftGuardBandWidth[i]           =  m_pcCfg->getRwpSEIRwpLeftGuardBandWidth(i);
      seiRegionWisePacking->m_rwpRightGuardBandWidth[i]          =  m_pcCfg->getRwpSEIRwpRightGuardBandWidth(i);
      seiRegionWisePacking->m_rwpTopGuardBandHeight[i]           =  m_pcCfg->getRwpSEIRwpTopGuardBandHeight(i);
      seiRegionWisePacking->m_rwpBottomGuardBandHeight[i]        =  m_pcCfg->getRwpSEIRwpBottomGuardBandHeight(i);
      seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag[i]  =  m_pcCfg->getRwpSEIRwpGuardBandNotUsedForPredFlag(i);
      for( int j=0; j < 4; j++ )
      {
        seiRegionWisePacking->m_rwpGuardBandType[i*4 + j]         =  m_pcCfg->getRwpSEIRwpGuardBandType(i*4 + j);
      }
    }
  }
}
#endif

#if JVET_P0597_GCMP_SEI
void SEIEncoder::initSEIGcmp(SEIGeneralizedCubemapProjection* seiGeneralizedCubemapProjection)
{
  CHECK(!(m_isInitialized), "seiGeneralizedCubemapProjection already initialized");
  CHECK(!(seiGeneralizedCubemapProjection != nullptr), "Need a seiGeneralizedCubemapProjection for initialization (got nullptr)");

  seiGeneralizedCubemapProjection->m_gcmpCancelFlag                      = m_pcCfg->getGcmpSEICancelFlag();
  if (!seiGeneralizedCubemapProjection->m_gcmpCancelFlag)
  {
    seiGeneralizedCubemapProjection->m_gcmpPersistenceFlag               = m_pcCfg->getGcmpSEIPersistenceFlag();
    seiGeneralizedCubemapProjection->m_gcmpPackingType                   = m_pcCfg->getGcmpSEIPackingType();
    seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType           = m_pcCfg->getGcmpSEIMappingFunctionType();

    int numFace = seiGeneralizedCubemapProjection->m_gcmpPackingType == 4 || seiGeneralizedCubemapProjection->m_gcmpPackingType == 5 ? 5 : 6;
    seiGeneralizedCubemapProjection->m_gcmpFaceIndex.resize(numFace);
    seiGeneralizedCubemapProjection->m_gcmpFaceRotation.resize(numFace);
    if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
    {
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag.resize(numFace);
    }
    for (int i = 0; i < numFace; i++)
    {
      seiGeneralizedCubemapProjection->m_gcmpFaceIndex[i]                = m_pcCfg->getGcmpSEIFaceIndex(i);
      seiGeneralizedCubemapProjection->m_gcmpFaceRotation[i]             = m_pcCfg->getGcmpSEIFaceRotation(i);
      if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
      {
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU[i]           = std::max<uint8_t>(1, (uint8_t)(128.0 * m_pcCfg->getGcmpSEIFunctionCoeffU(i) + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag[i] = m_pcCfg->getGcmpSEIFunctionUAffectedByVFlag(i);
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV[i]           = std::max<uint8_t>(1, (uint8_t)(128.0 * m_pcCfg->getGcmpSEIFunctionCoeffV(i) + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag[i] = m_pcCfg->getGcmpSEIFunctionVAffectedByUFlag(i);
      }
    }

    seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag                 = m_pcCfg->getGcmpSEIGuardBandFlag();
    if (seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag)
    {
      seiGeneralizedCubemapProjection->m_gcmpGuardBandBoundaryType       = m_pcCfg->getGcmpSEIGuardBandBoundaryType();
      seiGeneralizedCubemapProjection->m_gcmpGuardBandSamplesMinus1      = m_pcCfg->getGcmpSEIGuardBandSamplesMinus1();
    }
  }
}
#endif

#if JVET_P0450_SEI_SARI
void SEIEncoder::initSEISampleAspectRatioInfo(SEISampleAspectRatioInfo* seiSampleAspectRatioInfo)
{
  CHECK(!(m_isInitialized), "seiSampleAspectRatioInfo already initialized");
  CHECK(!(seiSampleAspectRatioInfo != nullptr), "Need a seiSampleAspectRatioInfo for initialization (got nullptr)");

  seiSampleAspectRatioInfo->m_sariCancelFlag = m_pcCfg->getSariCancelFlag();
  if (!seiSampleAspectRatioInfo->m_sariCancelFlag)
  {
    seiSampleAspectRatioInfo->m_sariPersistenceFlag   = m_pcCfg->getSariPersistenceFlag();
    seiSampleAspectRatioInfo->m_sariAspectRatioIdc    = m_pcCfg->getSariAspectRatioIdc();
    if (seiSampleAspectRatioInfo->m_sariAspectRatioIdc == 255)
    {
      seiSampleAspectRatioInfo->m_sariSarWidth   = m_pcCfg->getSariSarWidth();
      seiSampleAspectRatioInfo->m_sariSarHeight  = m_pcCfg->getSariSarHeight();
    }
    else
    {
      seiSampleAspectRatioInfo->m_sariSarWidth   = 0;
      seiSampleAspectRatioInfo->m_sariSarHeight  = 0;
    }
  }
}
#endif


#if HEVC_SEI
//! initialize scalable nesting SEI message.
//! Note: The SEI message structures input into this function will become part of the scalable nesting SEI and will be
//!       automatically freed, when the nesting SEI is disposed.
void SEIEncoder::initSEIScalableNesting(SEIScalableNesting *scalableNestingSEI, SEIMessages &nestedSEIs)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(scalableNestingSEI != NULL), "Unspecified error");

  scalableNestingSEI->m_bitStreamSubsetFlag           = 1;      // If the nested SEI messages are picture buffering SEI messages, picture timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
  scalableNestingSEI->m_nestingOpFlag                 = 0;
  scalableNestingSEI->m_nestingNumOpsMinus1           = 0;      //nesting_num_ops_minus1
  scalableNestingSEI->m_allLayersFlag                 = 0;
  scalableNestingSEI->m_nestingNoOpMaxTemporalIdPlus1 = 6 + 1;  //nesting_no_op_max_temporal_id_plus1
  scalableNestingSEI->m_nestingNumLayersMinus1        = 1 - 1;  //nesting_num_layers_minus1
  scalableNestingSEI->m_nestingLayerId[0]             = 0;

  scalableNestingSEI->m_nestedSEIs.clear();
  for (SEIMessages::iterator it=nestedSEIs.begin(); it!=nestedSEIs.end(); it++)
  {
    scalableNestingSEI->m_nestedSEIs.push_back((*it));
  }
}

void SEIEncoder::initSEIRecoveryPoint(SEIRecoveryPoint *recoveryPointSEI, Slice *slice)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(recoveryPointSEI != NULL), "Unspecified error");
  CHECK(!(slice != NULL), "Unspecified error");

  recoveryPointSEI->m_recoveryPocCnt    = 0;
  recoveryPointSEI->m_exactMatchingFlag = ( slice->getPOC() == 0 ) ? (true) : (false);
  recoveryPointSEI->m_brokenLinkFlag    = false;
}
#endif

//! calculate hashes for entire reconstructed picture
void SEIEncoder::initDecodedPictureHashSEI(SEIDecodedPictureHash *decodedPictureHashSEI, PelUnitBuf& pic, std::string &rHashString, const BitDepths &bitDepths)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(decodedPictureHashSEI!=NULL), "Unspecified error");

  decodedPictureHashSEI->method = m_pcCfg->getDecodedPictureHashSEIType();
  switch (m_pcCfg->getDecodedPictureHashSEIType())
  {
    case HASHTYPE_MD5:
      {
        uint32_t numChar=calcMD5(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
    case HASHTYPE_CRC:
      {
        uint32_t numChar=calcCRC(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
    case HASHTYPE_CHECKSUM:
    default:
      {
        uint32_t numChar=calcChecksum(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
  }
}

void SEIEncoder::initSEIDependentRAPIndication(SEIDependentRAPIndication *seiDependentRAPIndication)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiDependentRAPIndication!=NULL), "Unspecified error");
}

#if HEVC_SEI
void SEIEncoder::initTemporalLevel0IndexSEI(SEITemporalLevel0Index *temporalLevel0IndexSEI, Slice *slice)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(temporalLevel0IndexSEI!=NULL), "Unspecified error");
  CHECK(!(slice!=NULL), "Unspecified error");

  if (slice->getRapPicFlag())
  {
    m_tl0Idx = 0;
    m_rapIdx = (m_rapIdx + 1) & 0xFF;
  }
  else
  {
    m_tl0Idx = (m_tl0Idx + (slice->getTLayer() ? 0 : 1)) & 0xFF;
  }
  temporalLevel0IndexSEI->tl0Idx = m_tl0Idx;
  temporalLevel0IndexSEI->rapIdx = m_rapIdx;
}

void SEIEncoder::initSEITempMotionConstrainedTileSets (SEITempMotionConstrainedTileSets *sei, const PPS *pps)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(sei!=NULL), "Unspecified error");
  CHECK(!(pps!=NULL), "Unspecified error");

  if(!pps->getSingleTileInPicFlag())
  {
    if (m_pcCfg->getMCTSEncConstraint())
    {
      sei->m_mc_all_tiles_exact_sample_value_match_flag = true;
      sei->m_each_tile_one_tile_set_flag = true;
      sei->m_limited_tile_set_display_flag = false;
      sei->m_max_mcs_tier_level_idc_present_flag = false;
      sei->setNumberOfTileSets(0);
    }
    else
    {
    sei->m_mc_all_tiles_exact_sample_value_match_flag = false;
    sei->m_each_tile_one_tile_set_flag                = false;
    sei->m_limited_tile_set_display_flag              = false;
    sei->setNumberOfTileSets((pps->getNumTileColumnsMinus1() + 1) * (pps->getNumTileRowsMinus1() + 1));

    for(int i=0; i < sei->getNumberOfTileSets(); i++)
    {
      sei->tileSetData(i).m_mcts_id = i;  //depends the application;
      sei->tileSetData(i).setNumberOfTileRects(1);

      for(int j=0; j<sei->tileSetData(i).getNumberOfTileRects(); j++)
      {
        sei->tileSetData(i).topLeftTileIndex(j)     = i+j;
        sei->tileSetData(i).bottomRightTileIndex(j) = i+j;
      }

      sei->tileSetData(i).m_exact_sample_value_match_flag    = false;
      sei->tileSetData(i).m_mcts_tier_level_idc_present_flag = false;
    }
  }
  }
  else
  {
    CHECK(!(!"Tile is not enabled"), "Unspecified error");
  }
}

void SEIEncoder::initSEIKneeFunctionInfo(SEIKneeFunctionInfo *seiKneeFunctionInfo)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiKneeFunctionInfo!=NULL), "Unspecified error");

  seiKneeFunctionInfo->m_kneeId = m_pcCfg->getKneeSEIId();
  seiKneeFunctionInfo->m_kneeCancelFlag = m_pcCfg->getKneeSEICancelFlag();
  if ( !seiKneeFunctionInfo->m_kneeCancelFlag )
  {
    seiKneeFunctionInfo->m_kneePersistenceFlag = m_pcCfg->getKneeSEIPersistenceFlag();
    seiKneeFunctionInfo->m_kneeInputDrange = m_pcCfg->getKneeSEIInputDrange();
    seiKneeFunctionInfo->m_kneeInputDispLuminance = m_pcCfg->getKneeSEIInputDispLuminance();
    seiKneeFunctionInfo->m_kneeOutputDrange = m_pcCfg->getKneeSEIOutputDrange();
    seiKneeFunctionInfo->m_kneeOutputDispLuminance = m_pcCfg->getKneeSEIOutputDispLuminance();

    seiKneeFunctionInfo->m_kneeNumKneePointsMinus1 = m_pcCfg->getKneeSEINumKneePointsMinus1();
    int* piInputKneePoint  = m_pcCfg->getKneeSEIInputKneePoint();
    int* piOutputKneePoint = m_pcCfg->getKneeSEIOutputKneePoint();
    if(piInputKneePoint&&piOutputKneePoint)
    {
      seiKneeFunctionInfo->m_kneeInputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
      seiKneeFunctionInfo->m_kneeOutputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
      for(int i=0; i<=seiKneeFunctionInfo->m_kneeNumKneePointsMinus1; i++)
      {
        seiKneeFunctionInfo->m_kneeInputKneePoint[i] = piInputKneePoint[i];
        seiKneeFunctionInfo->m_kneeOutputKneePoint[i] = piOutputKneePoint[i];
      }
    }
  }
}
#endif

template <typename T>
static void readTokenValue(T            &returnedValue, /// value returned
                           bool         &failed,        /// used and updated
                           std::istream &is,            /// stream to read token from
                           const char  *pToken)        /// token string
{
  returnedValue=T();
  if (failed)
  {
    return;
  }

  int c;
  // Ignore any whitespace
  while ((c=is.get())!=EOF && isspace(c));
  // test for comment mark
  while (c=='#')
  {
    // Ignore to the end of the line
    while ((c=is.get())!=EOF && (c!=10 && c!=13));
    // Ignore any white space at the start of the next line
    while ((c=is.get())!=EOF && isspace(c));
  }
  // test first character of token
  failed=(c!=pToken[0]);
  // test remaining characters of token
  int pos;
  for(pos=1;!failed && pToken[pos]!=0 && is.get()==pToken[pos]; pos++);
  failed|=(pToken[pos]!=0);
  // Ignore any whitespace before the ':'
  while (!failed && (c=is.get())!=EOF && isspace(c));
  failed|=(c!=':');
  // Now read the value associated with the token:
  if (!failed)
  {
    is >> returnedValue;
    failed=!is.good();
    if (!failed)
    {
      c=is.get();
      failed=(c!=EOF && !isspace(c));
    }
  }
  if (failed)
  {
    std::cerr << "Unable to read token '" << pToken << "'\n";
  }
}

template <typename T>
static void readTokenValueAndValidate(T            &returnedValue, /// value returned
                                      bool         &failed,        /// used and updated
                                      std::istream &is,            /// stream to read token from
                                      const char  *pToken,        /// token string
                                      const T      &minInclusive,  /// minimum value allowed, inclusive
                                      const T      &maxInclusive)  /// maximum value allowed, inclusive
{
  readTokenValue(returnedValue, failed, is, pToken);
  if (!failed)
  {
    if (returnedValue<minInclusive || returnedValue>maxInclusive)
    {
      failed=true;
      std::cerr << "Value for token " << pToken << " must be in the range " << minInclusive << " to " << maxInclusive << " (inclusive); value read: " << returnedValue << std::endl;
    }
  }
}

#if HEVC_SEI || JVET_P0337_PORTING_SEI
#if !JVET_P0337_PORTING_SEI
// bool version does not have maximum and minimum values.
static void readTokenValueAndValidate(bool         &returnedValue, /// value returned
                                      bool         &failed,        /// used and updated
                                      std::istream &is,            /// stream to read token from
                                      const char  *pToken)        /// token string
{
  readTokenValue(returnedValue, failed, is, pToken);
}

bool SEIEncoder::initSEIColourRemappingInfo(SEIColourRemappingInfo* seiColourRemappingInfo, int currPOC) // returns true on success, false on failure.
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiColourRemappingInfo!=NULL), "Unspecified error");

  // reading external Colour Remapping Information SEI message parameters from file
  if( !m_pcCfg->getColourRemapInfoSEIFileRoot().empty())
  {
    bool failed=false;

    // building the CRI file name with poc num in prefix "_poc.txt"
    std::string colourRemapSEIFileWithPoc(m_pcCfg->getColourRemapInfoSEIFileRoot());
    {
      std::stringstream suffix;
      suffix << "_" << currPOC << ".txt";
      colourRemapSEIFileWithPoc+=suffix.str();
    }

    std::ifstream fic(colourRemapSEIFileWithPoc.c_str());
    if (!fic.good() || !fic.is_open())
    {
      std::cerr <<  "No Colour Remapping Information SEI parameters file " << colourRemapSEIFileWithPoc << " for POC " << currPOC << std::endl;
      return false;
    }

    // TODO: identify and remove duplication with decoder parsing through abstraction.

    readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapId,         failed, fic, "colour_remap_id",        uint32_t(0), uint32_t(0x7fffffff) );
    readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapCancelFlag, failed, fic, "colour_remap_cancel_flag" );
    if( !seiColourRemappingInfo->m_colourRemapCancelFlag )
    {
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapPersistenceFlag,            failed, fic, "colour_remap_persistence_flag" );
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapVideoSignalInfoPresentFlag, failed, fic, "colour_remap_video_signal_info_present_flag");
      if( seiColourRemappingInfo->m_colourRemapVideoSignalInfoPresentFlag )
      {
        readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapFullRangeFlag,      failed, fic, "colour_remap_full_range_flag" );
        readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapPrimaries,          failed, fic, "colour_remap_primaries",           int(0), int(255) );
        readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapTransferFunction,   failed, fic, "colour_remap_transfer_function",   int(0), int(255) );
        readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapMatrixCoefficients, failed, fic, "colour_remap_matrix_coefficients", int(0), int(255) );
      }
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapInputBitDepth, failed, fic, "colour_remap_input_bit_depth",            int(8), int(16) );
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapBitDepth,      failed, fic, "colour_remap_bit_depth",                  int(8), int(16) );

      const int maximumInputValue    = (1 << (((seiColourRemappingInfo->m_colourRemapInputBitDepth + 7) >> 3) << 3)) - 1;
      const int maximumRemappedValue = (1 << (((seiColourRemappingInfo->m_colourRemapBitDepth      + 7) >> 3) << 3)) - 1;

      for( int c=0 ; c<3 ; c++ )
      {
        readTokenValueAndValidate(seiColourRemappingInfo->m_preLutNumValMinus1[c],         failed, fic, "pre_lut_num_val_minus1[c]",        int(0), int(32) );
        if( seiColourRemappingInfo->m_preLutNumValMinus1[c]>0 )
        {
          seiColourRemappingInfo->m_preLut[c].resize(seiColourRemappingInfo->m_preLutNumValMinus1[c]+1);
          for( int i=0 ; i<=seiColourRemappingInfo->m_preLutNumValMinus1[c] ; i++ )
          {
            readTokenValueAndValidate(seiColourRemappingInfo->m_preLut[c][i].codedValue,   failed, fic, "pre_lut_coded_value[c][i]",  int(0), maximumInputValue    );
            readTokenValueAndValidate(seiColourRemappingInfo->m_preLut[c][i].targetValue,  failed, fic, "pre_lut_target_value[c][i]", int(0), maximumRemappedValue );
          }
        }
      }
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapMatrixPresentFlag, failed, fic, "colour_remap_matrix_present_flag" );
      if( seiColourRemappingInfo->m_colourRemapMatrixPresentFlag )
      {
        readTokenValueAndValidate(seiColourRemappingInfo->m_log2MatrixDenom, failed, fic, "log2_matrix_denom", int(0), int(15) );
        for( int c=0 ; c<3 ; c++ )
        {
          for( int i=0 ; i<3 ; i++ )
          {
            readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapCoeffs[c][i], failed, fic, "colour_remap_coeffs[c][i]", -32768, 32767 );
          }
        }
      }
      for( int c=0 ; c<3 ; c++ )
      {
        readTokenValueAndValidate(seiColourRemappingInfo->m_postLutNumValMinus1[c], failed, fic, "post_lut_num_val_minus1[c]", int(0), int(32) );
        if( seiColourRemappingInfo->m_postLutNumValMinus1[c]>0 )
        {
          seiColourRemappingInfo->m_postLut[c].resize(seiColourRemappingInfo->m_postLutNumValMinus1[c]+1);
          for( int i=0 ; i<=seiColourRemappingInfo->m_postLutNumValMinus1[c] ; i++ )
          {
            readTokenValueAndValidate(seiColourRemappingInfo->m_postLut[c][i].codedValue,  failed, fic, "post_lut_coded_value[c][i]",  int(0), maximumRemappedValue );
            readTokenValueAndValidate(seiColourRemappingInfo->m_postLut[c][i].targetValue, failed, fic, "post_lut_target_value[c][i]", int(0), maximumRemappedValue );
          }
        }
      }
    }

    if( failed )
    {
      EXIT( "Error while reading Colour Remapping Information SEI parameters file '" << colourRemapSEIFileWithPoc << "'" );
    }
  }
  return true;
}

void SEIEncoder::initSEIChromaResamplingFilterHint(SEIChromaResamplingFilterHint *seiChromaResamplingFilterHint, int iHorFilterIndex, int iVerFilterIndex)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiChromaResamplingFilterHint!=NULL), "Unspecified error");

  seiChromaResamplingFilterHint->m_verChromaFilterIdc = iVerFilterIndex;
  seiChromaResamplingFilterHint->m_horChromaFilterIdc = iHorFilterIndex;
  seiChromaResamplingFilterHint->m_verFilteringFieldProcessingFlag = 1;
  seiChromaResamplingFilterHint->m_targetFormatIdc = 3;
  seiChromaResamplingFilterHint->m_perfectReconstructionFlag = false;

  // this creates some example filter values, if explicit filter definition is selected
  if (seiChromaResamplingFilterHint->m_verChromaFilterIdc == 1)
  {
    const int numVerticalFilters = 3;
    const int verTapLengthMinus1[] = {5,3,3};

    seiChromaResamplingFilterHint->m_verFilterCoeff.resize(numVerticalFilters);
    for(int i = 0; i < numVerticalFilters; i ++)
    {
      seiChromaResamplingFilterHint->m_verFilterCoeff[i].resize(verTapLengthMinus1[i]+1);
    }
    // Note: C++11 -> seiChromaResamplingFilterHint->m_verFilterCoeff[0] = {-3,13,31,23,3,-3};
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][0] = -3;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][1] = 13;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][2] = 31;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][3] = 23;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][4] = 3;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][5] = -3;

    seiChromaResamplingFilterHint->m_verFilterCoeff[1][0] = -1;
    seiChromaResamplingFilterHint->m_verFilterCoeff[1][1] = 25;
    seiChromaResamplingFilterHint->m_verFilterCoeff[1][2] = 247;
    seiChromaResamplingFilterHint->m_verFilterCoeff[1][3] = -15;

    seiChromaResamplingFilterHint->m_verFilterCoeff[2][0] = -20;
    seiChromaResamplingFilterHint->m_verFilterCoeff[2][1] = 186;
    seiChromaResamplingFilterHint->m_verFilterCoeff[2][2] = 100;
    seiChromaResamplingFilterHint->m_verFilterCoeff[2][3] = -10;
  }
  else
  {
    seiChromaResamplingFilterHint->m_verFilterCoeff.resize(0);
  }

  if (seiChromaResamplingFilterHint->m_horChromaFilterIdc == 1)
  {
    int const numHorizontalFilters = 1;
    const int horTapLengthMinus1[] = {3};

    seiChromaResamplingFilterHint->m_horFilterCoeff.resize(numHorizontalFilters);
    for(int i = 0; i < numHorizontalFilters; i ++)
    {
      seiChromaResamplingFilterHint->m_horFilterCoeff[i].resize(horTapLengthMinus1[i]+1);
    }
    seiChromaResamplingFilterHint->m_horFilterCoeff[0][0] = 1;
    seiChromaResamplingFilterHint->m_horFilterCoeff[0][1] = 6;
    seiChromaResamplingFilterHint->m_horFilterCoeff[0][2] = 1;
  }
  else
  {
    seiChromaResamplingFilterHint->m_horFilterCoeff.resize(0);
  }
}

void SEIEncoder::initSEITimeCode(SEITimeCode *seiTimeCode)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiTimeCode!=NULL), "Unspecified error");
  //  Set data as per command line options
  seiTimeCode->numClockTs = m_pcCfg->getNumberOfTimesets();
  for(int i = 0; i < seiTimeCode->numClockTs; i++)
  {
    seiTimeCode->timeSetArray[i] = m_pcCfg->getTimeSet(i);
  }
}
#endif
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
void SEIEncoder::initSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics *seiAltTransCharacteristics)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiAltTransCharacteristics!=NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAltTransCharacteristics->m_preferredTransferCharacteristics = m_pcCfg->getSEIPreferredTransferCharacteristics();
}
#endif
#if !JVET_P0337_PORTING_SEI
void SEIEncoder::initSEIGreenMetadataInfo(SEIGreenMetadataInfo *seiGreenMetadataInfo, uint32_t u)
{
    CHECK(!(m_isInitialized), "Unspecified error");
    CHECK(!(seiGreenMetadataInfo!=NULL), "Unspecified error");

    seiGreenMetadataInfo->m_greenMetadataType = m_pcCfg->getSEIGreenMetadataType();
    seiGreenMetadataInfo->m_xsdMetricType = m_pcCfg->getSEIXSDMetricType();
    seiGreenMetadataInfo->m_xsdMetricValue = u;
}
#endif
#endif
#if JVET_P0337_PORTING_SEI
void SEIEncoder::initSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics *seiFilmGrain)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiFilmGrain != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiFilmGrain->m_filmGrainCharacteristicsCancelFlag      = m_pcCfg->getFilmGrainCharactersticsSEICancelFlag();
  seiFilmGrain->m_filmGrainCharacteristicsPersistenceFlag = m_pcCfg->getFilmGrainCharactersticsSEIPersistenceFlag();
  seiFilmGrain->m_filmGrainModelId                        = m_pcCfg->getFilmGrainCharactersticsSEIModelID();
  seiFilmGrain->m_separateColourDescriptionPresentFlag    = m_pcCfg->getFilmGrainCharactersticsSEISepColourDescPresent();
  seiFilmGrain->m_blendingModeId                          = m_pcCfg->getFilmGrainCharactersticsSEIBlendingModeID();
  seiFilmGrain->m_log2ScaleFactor                         = m_pcCfg->getFilmGrainCharactersticsSEILog2ScaleFactor();
  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    seiFilmGrain->m_compModel[i].presentFlag = m_pcCfg->getFGCSEICompModelPresent(i);
  }
}

void SEIEncoder::initSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume *seiMDCV)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiMDCV != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  for (int j = 0; j <= 1; j++)
  {
    for (int i = 0; i <= 2; i++)
    {
       seiMDCV->values.primaries[i][j] = m_pcCfg->getMasteringDisplaySEI().primaries[i][j];
    }
    seiMDCV->values.whitePoint[j] = m_pcCfg->getMasteringDisplaySEI().whitePoint[j];
  }
  seiMDCV->values.maxLuminance = m_pcCfg->getMasteringDisplaySEI().maxLuminance;
  seiMDCV->values.minLuminance = m_pcCfg->getMasteringDisplaySEI().minLuminance;
}

void SEIEncoder::initSEIContentLightLevel(SEIContentLightLevelInfo *seiCLL)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiCLL != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiCLL->m_maxContentLightLevel    = m_pcCfg->getCLLSEIMaxContentLightLevel();
  seiCLL->m_maxPicAverageLightLevel = m_pcCfg->getCLLSEIMaxPicAvgLightLevel();
}

void SEIEncoder::initSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment *seiAmbViewEnvironment)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiAmbViewEnvironment != NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAmbViewEnvironment->m_ambientIlluminance = m_pcCfg->getAmbientViewingEnvironmentSEIIlluminance();
  seiAmbViewEnvironment->m_ambientLightX      = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightX();
  seiAmbViewEnvironment->m_ambientLightY      = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightY();
}

void SEIEncoder::initSEIContentColourVolume(SEIContentColourVolume *seiContentColourVolume)
{
  assert(m_isInitialized);
  assert(seiContentColourVolume != NULL);
  seiContentColourVolume->m_ccvCancelFlag = m_pcCfg->getCcvSEICancelFlag();
  seiContentColourVolume->m_ccvPersistenceFlag = m_pcCfg->getCcvSEIPersistenceFlag();

  seiContentColourVolume->m_ccvPrimariesPresentFlag = m_pcCfg->getCcvSEIPrimariesPresentFlag();
  seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMinLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMaxLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag = m_pcCfg->getCcvSEIAvgLuminanceValuePresentFlag();

  // Currently we are using a floor operation for setting up the "integer" values for this SEI.
  // This applies to both primaries and luminance limits.
  if (seiContentColourVolume->m_ccvPrimariesPresentFlag == true)
  {
    for (int i = 0; i < MAX_NUM_COMPONENT; i++)
    {
      seiContentColourVolume->m_ccvPrimariesX[i] = (int32_t)(50000.0 * m_pcCfg->getCcvSEIPrimariesX(i));
      seiContentColourVolume->m_ccvPrimariesY[i] = (int32_t)(50000.0 * m_pcCfg->getCcvSEIPrimariesY(i));
    }
  }

  if (seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMinLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIMinLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMaxLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIMaxLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvAvgLuminanceValue = (uint32_t)(10000000 * m_pcCfg->getCcvSEIAvgLuminanceValue());
  }
}
#endif
#if JVET_P0984_SEI_SUBPIC_LEVEL
void SEIEncoder::initSEISubpictureLevelInfo(SEISubpicureLevelInfo *sei, const SPS *sps)
{
  // subpicture level information should be specified via config file
  // unfortunately the implementation of subpictures is still not available
  // TODO: implement config file parameters and intialization
  fprintf(stderr, "SEISubpicureLevelInfo depends on subpictures! Initializing to dummy values!\n");

  sei->m_sliSeqParameterSetId = sps->getSPSId();
  sei->m_numRefLevels = 2;
  sei->m_refLevelIdc.resize(2);
  sei->m_refLevelIdc[0] = Level::LEVEL4;
  sei->m_refLevelIdc[1] = Level::LEVEL8_5;
  sei->m_explicitFractionPresentFlag = false;
}
#endif


//! \}
