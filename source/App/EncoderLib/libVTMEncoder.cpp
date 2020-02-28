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

 /** \file     libVTMEncoder.cpp
     \brief    Encoder library class
 */

#include "libVTMEncoder.h"

#include "libVTMEncoderCfg.h"
#include "Utilities/program_options_lite.h"
#include "EncGOP.h"

#include <vector>

class vtmEncoderWrapper : public LibVTMEncoderCfg, public AUWriterIf 
{
public:
  vtmEncoderWrapper()
  {
  }
  ~vtmEncoderWrapper()
  { 
    m_cEncLib.destroy(); 
  }

  bool init()
  {
    try
    {
      setRandomAccessConfig();
    }
    catch (df::program_options_lite::ParseFailure &e)
    {
      std::cerr << "Error parsing option \""<< e.arg <<"\" with argument \""<< e.val <<"\"." << std::endl;
      return false;
    }
    return true;
  }

  void outputAU(const AccessUnit& au) override
  {
    m_lOutputAUList.push_back(au);
  }

  bool configureEncoder(vtm_settings_t *settings)
  {
    if (settings == nullptr || !applySettings(settings))
    {
      return false;
    }

    vtm_settings = *settings;

    xInitLibCfg();
    m_cEncLib.create(); // The part we need from xCreateLib
    m_cEncLib.init(false, this); // xInitLib
    
    UnitArea unitArea( m_chromaFormatIDC, Area( 0, 0, m_iSourceWidth, m_iSourceHeight ) );
    origPic.create( unitArea );

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
    for (int i=0; i<origVtmPic.img.nr_planes; i++)
    {
      auto component = ComponentID(i);
      auto luma = origPic.get(component);
      origVtmPic.img.stride[i] = luma.stride;
      origVtmPic.img.plane[i] = luma.bufAt(0, 0);
    }
    return &origVtmPic;
  }

  void pushFrame(vtm_pic_t *pic)
  {
    const bool bEos = false;
    const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
    m_cEncLib.encode( bEos, flushing ? 0 : &origPic, nullptr, snrCSC, recBufList, iNumEncoded );
  }

protected:
  vtm_settings_t vtm_settings;

  std::vector<AccessUnit> m_lOutputAUList;
  PelStorage origPic;
  vtm_pic_t origVtmPic;
  std::list<PelUnitBuf*> recBufList;
  int iNumEncoded = { 0 };

  bool flushing { false };
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

    if (!enc->init())
      return LIBVTMENC_ERROR;

    if (!settings)
      return LIBVTMENC_ERROR;

    if (!enc->configureEncoder(settings))
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

    enc->pushFrame(pic_in);

    return LIBVTMENC_OK;
  }

  VTM_ENC_API libVTMEnc_error libVTMEncoder_receive_packet(libVTMEncoder_context* encCtx, vtm_nal_t **pp_nal, int *pi_nal, vtm_pic_t *pic_out)
  {
    vtmEncoderWrapper *enc = (vtmEncoderWrapper*)encCtx;
    if (!enc)
      return LIBVTMENC_ERROR;

    return LIBVTMENC_OK;
  }

}