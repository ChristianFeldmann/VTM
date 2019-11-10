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

#ifndef libVTMDecoderInternals_H
#define libVTMDecoderInternals_H

#include <stdint.h>
#include "libVTMDecoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER)
#define VTM_DEC_API __declspec(dllexport)
#else
#define VTM_DEC_API
#endif

enum InternalsPredMode
{
  INTERNALS_MODE_INTER = 0,   ///< inter-prediction mode
  INTERNALS_MODE_INTRA = 1,   ///< intra-prediction mode
  INTERNALS_MODE_IBC = 2,     ///< ibc-prediction mode
  INTERNALS_MODE_PLT = 3,     ///< plt-prediction mode
  INTERNALS_NUMBER_OF_PREDICTION_MODES = 4
};

struct InternalsCUData
{
  uint8_t width, height;
  int x, y;
  InternalsPredMode predMode;
  uint8_t depth;
  int8_t qp;
};

/* Get the number of internals that can be retrived from this picture
 */
VTM_DEC_API int libVTMDec_get_internals_number_CU(libVTMDec_picture *pic);

/* Get the internals CU data for the given cu_index from the picture.
 * \param pic The libVTMDec_picture that was obtained using libVTMDec_get_picture.
 * \param cu_index The index into the list of data. Check libVTMDec_get_internals_number_CU to obtain the valid range.
 * \return A pointer to the CU's InternalsCUData struct.
 */
VTM_DEC_API InternalsCUData *libVTMDec_get_internals_CU(libVTMDec_picture *pic, int cu_index);

#ifdef __cplusplus
}
#endif

#endif // libVTMDecoderInternals_H