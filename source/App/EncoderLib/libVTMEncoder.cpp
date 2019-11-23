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
      if(!parseCfg())
        return LIBVTMENC_ERROR;
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

  bool applySettings(vtm_settings_t *settings)
  {
    xInitLibCfg();
    m_cEncLib.create(); // The part we need from xCreateLib
    m_cEncLib.init(false, this); // xInitLib
    
    UnitArea unitArea( m_chromaFormatIDC, Area( 0, 0, m_iSourceWidth, m_iSourceHeight ) );
    orgPic.create( unitArea );

    return true;
  }

protected:
  vtm_settings_t vtm_settings;

  std::vector<AccessUnit> m_lOutputAUList;
  PelStorage orgPic;
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

    if (!enc->applySettings(settings))
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

  VTM_ENC_API libVTMEnc_error libVTMEncoder_encode(libVTMEncoder_context* encCtx, vtm_nal_t **pp_nal, int *pi_nal, vtm_pic_t *pic_in, vtm_pic_t *pic_out)
  {
    vtmEncoderWrapper *enc = (vtmEncoderWrapper*)encCtx;
    if (!enc)
      return LIBVTMENC_ERROR;

    return LIBVTMENC_OK;
  }

}