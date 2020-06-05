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

/** \file     libvtmencoder.h
    \brief    Encoder lib class (header)
*/

#ifndef __LIBVTMENC__
#define __LIBVTMENC__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER)
#define VTM_ENC_API __declspec(dllexport)
#else
#define VTM_ENC_API
#endif

//! \ingroup libvtmencoder
//! \{

/** The error codes that are returned if something goes wrong
 */
typedef enum
{
  LIBVTMENC_OK = 0,            ///< No error occured
  LIBVTMENC_ERROR              ///< There was an unspecified error
} libVTMEnc_error;

typedef enum
{
  VTM_CHROMA_400        = 0,
  VTM_CHROMA_420        = 1,
  VTM_CHROMA_422        = 2,
  VTM_CHROMA_444        = 3,
  VTM_NUM_CHROMA_FORMAT = 4
} vtm_chroma_format_t;

typedef struct vtm_nal_t
{
  int payloadSizeBytes;
  uint8_t *payload;
} vtm_nal_t;

typedef struct vtm_image_t
{
  int                 nr_planes; /* Number of image planes */
  int                 stride[4]; /* Strides for each plane */
  int                 size[2];   /* width/height */
  int16_t             *plane[4]; /* Pointers to each plane */
} vtm_image_t;

typedef struct vtm_pic_t
{
  int i_type;
  int64_t i_pts;
  int64_t i_dts;
  vtm_chroma_format_t chroma_format;
  vtm_image_t img;
} vtm_pic_t;

typedef struct vtm_settings_t
{
  int source_width;
  int source_height;
  int source_bitdepth;
  int fps_num;
  int fps_den;
  vtm_chroma_format_t chroma_format;
} vtm_settings_t;

/** Get info about the VTM encoder version (e.g. "VTM-6.0")
 */
VTM_ENC_API const char *libVTMEncoder_get_version(void);

/** This private structure is the encoder.
 * You can save a pointer to it and use all the following functions to access it
 * but it is not further defined as part of the public API.
 */
typedef void libVTMEncoder_context;

/** Allocate and initialize a new encoder.
  * \return Returns a pointer to the new encoder or NULL if an error occurred.
  */
VTM_ENC_API libVTMEncoder_context* libVTMEncoder_new_encoder();

/** Initialize the new encoder.
  * \param settings The settings to apply
  * \return Return an error code or LIBVTMENC_OK if no error occured
  */
VTM_ENC_API libVTMEnc_error libVTMEncoder_init_encoder(libVTMEncoder_context* encCtx, vtm_settings_t *settings);

/** Destroy an existing encoder.
 * \param encCtx The encoder context to destroy that was created with libVTMEncoder_new_encoder
 * \return Return an error code or LIBVTMENC_OK if no error occured
 */
VTM_ENC_API libVTMEnc_error libVTMEncoder_free_encoder(libVTMEncoder_context* encCtx);

/** Get a frame that can be fille with data for inpout.
 * The frame is already allocated with the settings that were provided in libVTMEncoder_init_encoder.
 * The ownership of the frame is and stays in this library. Just copy the frame data into it
 * and then use libVTMEncoder_send_frame to push it.
 */
VTM_ENC_API vtm_pic_t *libVTMEncoder_get_input_frame(libVTMEncoder_context* encCtx);

VTM_ENC_API libVTMEnc_error libVTMEncoder_send_frame(libVTMEncoder_context* encCtx, vtm_pic_t *pic_in);
VTM_ENC_API libVTMEnc_error libVTMEncoder_receive_packet(libVTMEncoder_context* encCtx, vtm_nal_t **pp_nal, int *pi_nal, vtm_pic_t *pic_out);

//! \}

#ifdef __cplusplus
}
#endif

#endif // __LIBVTMENC__