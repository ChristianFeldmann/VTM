#include "libVTMDecoderInternals.h"

#include "CommonLib/Picture.h"

extern "C" {

  VTM_DEC_API int libVTMDec_get_internals_number_CU(libVTMDec_picture *pic)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    return -1;
  }

  VTM_DEC_API InternalsCUData *libVTMDec_get_internals_CU(libVTMDec_picture *pic, int cu_index)
  {
    if (pic == NULL)
      return NULL;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return NULL;

    return NULL;
  }

}