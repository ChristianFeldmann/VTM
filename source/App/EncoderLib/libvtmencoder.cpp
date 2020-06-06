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

 /** \file     libvtmencoder.cpp
     \brief    Encoder library class
 */

#include "libvtmencoder.h"

#include "libVTMEncoderDefaultConfigs.h"

#include "EncApp.h"
#include "EncoderLib/AnnexBwrite.h"
#include "EncoderLib/EncLib.h"
#include "EncoderLib/EncLibCommon.h"
#include "CommonLib/Buffer.h"
#include "Utilities/program_options_lite.h"
#include "EncGOP.h"

#include <vector>
#include <sstream>

class vtmEncoderWrapper : public EncApp
{
public:
  vtmEncoderWrapper() : EncApp(bitstream, &encLibCommon)
  {
    initROM();
  }
  ~vtmEncoderWrapper()
  {
    destroyLib();
    destroyROM();
  }

  void outputAU( const AccessUnit& au ) override
  {
    std::stringstream stream;
    const vector<uint32_t>& stats = writeAnnexB(stream, au);
    rateStatsAccum(au, stats);
    this->auDataList.push_back(stream.str());
  }

  bool configureAndOpenEncoder(vtm_settings_t *settings)
  {
    if (settings == nullptr)
    {
      return false;
    }

    auto parametersAsString = getRandomAccessParameters();
    parametersAsString += convertSettingsToParameters(settings);
    {
      if (!parseCfg(parametersAsString))
      {
        return false;
      }
    }

    this->vtm_settings = *settings;

    // EncApp::createLib just without opening the bitstream
    if (!createLib())
    {
      return false;
    }

    return true;
  }

  bool createLib()
  {
    // Same as EncApp::createLib just without opening a bitstream file
    // And only one layer supported
    const auto layerIdx = 0;

    const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
    UnitArea unitArea( m_chromaFormatIDC, Area( 0, 0, m_iSourceWidth, sourceHeight ) );

    m_orgPic = new PelStorage;
    m_trueOrgPic = new PelStorage;
    m_orgPic->create( unitArea );
    m_trueOrgPic->create( unitArea );

    // initialize internal class & member variables and VPS
    xInitLibCfg();
    const int layerId = m_cEncLib.getVPS() == nullptr ? 0 : m_cEncLib.getVPS()->getLayerId( layerIdx );
    xCreateLib( m_recBufList, layerId );
    xInitLib( m_isField );

    {
      if (m_InputChromaFormatIDC < 0 || m_InputChromaFormatIDC >= NUM_CHROMA_FORMAT)
        return false;
    }

    if( m_gopBasedTemporalFilterEnabled )
    {
      m_temporalFilter.init( m_FrameSkip, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth, m_iSourceWidth, m_iSourceHeight,
        m_aiPad, m_bClipInputVideoToRec709Range, m_inputFileName, m_chromaFormatIDC,
        m_inputColourSpaceConvert, m_iQP, m_gopBasedTemporalFilterStrengths,
        m_gopBasedTemporalFilterFutureReference );
    }

    return true;
  }

  vtm_pic_t *getVtmPicForPushing()
  {
    origVtmPic.i_dts = -1;
    origVtmPic.i_pts = -1;
    origVtmPic.i_type = -1;
    origVtmPic.chroma_format = vtm_settings.chroma_format;
    origVtmPic.img.nr_planes = (vtm_settings.chroma_format == VTM_CHROMA_400) ? 1 : 3;
    origVtmPic.img.size[0] = vtm_settings.source_width;
    origVtmPic.img.size[1] = vtm_settings.source_height;

    const auto m_bitdepthShift = m_internalBitDepth - m_MSBExtendedBitDepth;
    origVtmPic.desiredBitDepth[0] = m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA] + m_bitdepthShift;
    origVtmPic.desiredBitDepth[1] = m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] + m_bitdepthShift;
    
    for (int i=0; i<origVtmPic.img.nr_planes; i++)
    {
      auto component = ComponentID(i);
      auto luma = m_trueOrgPic->get(component);
      origVtmPic.img.stride[i] = luma.stride;
      origVtmPic.img.plane[i] = luma.bufAt(0, 0);
    }
    return &origVtmPic;
  }

  void encodeUntilPacketsReadyOrMoreFramesNeeded()
  {
    bool keepLoop = true;
    while (keepLoop)
    {
      keepLoop = encode();
    }

    // Check for data out?
  }

  bool pushFrame(vtm_pic_t *pic)
  {
    if (encodingState != EncodingState::PushFrames)
      return false;

    bool eos = (pic == nullptr);

    bool keepDoing = encodePrep(eos);
    if (!keepDoing)
    {
      encodeUntilPacketsReadyOrMoreFramesNeeded();
    }

    if (eos)
      encodingState = EncodingState::EndOfStream;
    else if (!keepDoing)
      encodingState = EncodingState::RetrieveFrames;

    return true;
  }

  bool retrievePacket(vtm_nal_t *nal)
  {
    nal->payload = nullptr;
    nal->payloadSizeBytes = 0;

    if (!auDataList.empty())
    {
      currentAUData = auDataList.front();
      auDataList.pop_front();

      nal->payload = (uint8_t*)(this->currentAUData.c_str());
      nal->payloadSizeBytes = this->currentAUData.size();
    }
    
    return true;
  }

protected:
  std::fstream bitstream;
  EncLibCommon encLibCommon;

  vtm_settings_t vtm_settings;

  std::vector<AccessUnit> m_lOutputAUList;
  
  vtm_pic_t origVtmPic;
  std::list<PelUnitBuf*> recBufList;
  int iNumEncoded = { 0 };

  enum class EncodingState
  {
    PushFrames,
    RetrieveFrames,
    EndOfStream
  };
  EncodingState encodingState { EncodingState::PushFrames };

  std::deque<std::string> auDataList;
  std::string currentAUData;
};

extern "C" {

  VTM_ENC_API const char *libVTMEncoder_get_version(void)
  {
    return VTM_VERSION;
  }

  VTM_ENC_API libVTMEncoder_context* libVTMEncoder_new_encoder()
  {
    vtmEncoderWrapper *encCtx = new vtmEncoderWrapper();
    if (!encCtx)
      return NULL;

    return (libVTMEncoder_context*)encCtx;
  }

  VTM_ENC_API libVTMEnc_error libVTMEncoder_init_encoder(libVTMEncoder_context* encCtx, vtm_settings_t *settings)
  {
    vtmEncoderWrapper *enc = (vtmEncoderWrapper*)encCtx;
    if (!enc)
      return LIBVTMENC_ERROR;

    if (!settings)
      return LIBVTMENC_ERROR;

    if (!enc->configureAndOpenEncoder(settings))
      return LIBVTMENC_ERROR;

    return LIBVTMENC_OK;
  }

  VTM_ENC_API libVTMEnc_error libVTMEncoder_free_encoder(libVTMEncoder_context* encCtx)
  {
    vtmEncoderWrapper *enc = (vtmEncoderWrapper*)encCtx;
    if (!enc)
      return LIBVTMENC_ERROR;

    delete enc;
    return LIBVTMENC_OK;
  }

  VTM_ENC_API vtm_pic_t *libVTMEncoder_get_input_frame(libVTMEncoder_context* encCtx)
  {
    vtmEncoderWrapper *enc = (vtmEncoderWrapper*)encCtx;
    if (!enc)
      return nullptr;

    return enc->getVtmPicForPushing();
  }

  VTM_ENC_API libVTMEnc_error libVTMEncoder_send_frame(libVTMEncoder_context* encCtx, vtm_pic_t *pic_in)
  {
    vtmEncoderWrapper *enc = (vtmEncoderWrapper*)encCtx;
    if (!enc)
      return LIBVTMENC_ERROR;

    if (!enc->pushFrame(pic_in))
      return LIBVTMENC_ERROR;

    return LIBVTMENC_OK;
  }

  VTM_ENC_API libVTMEnc_error libVTMEncoder_receive_packet(libVTMEncoder_context* encCtx, vtm_nal_t *pp_nal)
  {
    vtmEncoderWrapper *enc = (vtmEncoderWrapper*)encCtx;
    if (!enc)
      return LIBVTMENC_ERROR;

    if (!enc->retrievePacket(pp_nal))
      return LIBVTMENC_ERROR;

    return LIBVTMENC_OK;
  }

}
