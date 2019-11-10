
#include "libVTMDecoder.h"

#include "DecoderLib/DecLib.h"
#include "DecoderLib/NALread.h"
#include "DecoderLib/AnnexBread.h"

#include <fstream>

class vtmDecoderWrapper
{
public:
  vtmDecoderWrapper()
  {
    // Initialize the decoder
    initROM();
    m_cDecLib.create();
    m_cDecLib.init();
    m_cDecLib.setDecodedPictureHashSEIEnabled(true);
  }
  ~vtmDecoderWrapper() { m_cDecLib.destroy(); };

  int m_iSkipFrame{ 0 };
  int m_iMaxTemporalLayer{ -1 };
  int m_iPOCLastDisplay{ -MAX_INT };
  PicList *pcListPic{ nullptr };

  bool loopFiltered{ false };
  bool flushOutput{ false };
  int numPicsNotYetDisplayed{ 0 };
  int dpbFullness{ 0 };
  uint32_t numReorderPicsHighestTid{ 0 };
  uint32_t maxDecPicBufferingHighestTid{ 0 };
  int pcListPic_readIdx{ 0 };

  // In the DecApp the xWriteOutput function can be called up to 3 times after a NAL was pushed
  // (or maybe this is an unclear programming error). However, this is how me mimic this.
  int nrOfTimesToCheckForOutputPictures{ 0 };
  
  // The vector that is filled when internals are returned.
  // The vector is defined, filled and cleared only in this library so that no chaos is created
  // between the heap of the shared library and the caller programm.
  std::vector<libVTMDec_BlockValue> internalsBlockData;

  DecLib m_cDecLib;
};

void initFrameRetrieval(vtmDecoderWrapper *d)
{
  // This is what xWriteOutput does before iterating over the pictures
  d->numPicsNotYetDisplayed = 0;
  d->dpbFullness = 0;
  const SPS* activeSPS = (d->pcListPic->front()->cs->sps);
  uint32_t maxNrSublayers = activeSPS->getMaxTLayers();

  if (d->m_iMaxTemporalLayer == -1 || d->m_iMaxTemporalLayer >= maxNrSublayers)
  {
    d->numReorderPicsHighestTid = activeSPS->getNumReorderPics(maxNrSublayers - 1);
    d->maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering(maxNrSublayers - 1);
  }
  else
  {
    d->numReorderPicsHighestTid = activeSPS->getNumReorderPics(d->m_iMaxTemporalLayer);
    d->maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering(d->m_iMaxTemporalLayer);
  }

  PicList::iterator iterPic = d->pcListPic->begin();
  while (iterPic != d->pcListPic->end())
  {
    Picture* pcPic = *(iterPic);
    if (pcPic->neededForOutput && pcPic->getPOC() > d->m_iPOCLastDisplay)
    {
      d->numPicsNotYetDisplayed++;
      d->dpbFullness++;
    }
    else if (pcPic->referenced)
    {
      d->dpbFullness++;
    }
    iterPic++;
  }

  // Reset the iterator over the output images
  d->pcListPic_readIdx = 0;
}

void initFrameFlushing(vtmDecoderWrapper *d)
{
  d->flushOutput = true;

  // Reset the iterator over the output images
  d->pcListPic_readIdx = 0;
}

class rawDataAccessStreamBuffer : public std::streambuf
{
public:
  rawDataAccessStreamBuffer(uint8_t *data, size_t dataSize)
  {
    char *d = (char*)(data);
    setg(d, d, d + dataSize);
  }
};

extern "C" {

  VTM_DEC_API const char *libVTMDec_get_version(void)
  {
    return VTM_VERSION;
  }

  VTM_DEC_API libVTMDec_context* libVTMDec_new_decoder(void)
  {
    vtmDecoderWrapper *decCtx = new vtmDecoderWrapper();
    if (!decCtx)
      return NULL;

    return (libVTMDec_context*)decCtx;
  }

  VTM_DEC_API libVTMDec_error libVTMDec_free_decoder(libVTMDec_context* decCtx)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return LIBVTMDEC_ERROR;

    delete d;
    return LIBVTMDEC_OK;
  }

  VTM_DEC_API void libVTMDec_set_SEI_Check(libVTMDec_context* decCtx, bool check_hash)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->m_cDecLib.setDecodedPictureHashSEIEnabled(check_hash);
  }

  VTM_DEC_API void libVTMDec_set_max_temporal_layer(libVTMDec_context* decCtx, int max_layer)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->m_iMaxTemporalLayer = max_layer;
  }

  VTM_DEC_API libVTMDec_error libVTMDec_push_nal_unit(libVTMDec_context *decCtx, const void* data8, int length, bool eof, bool &bNewPicture, bool &checkOutputPictures)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return LIBVTMDEC_ERROR;

    if (length <= 0 && !eof)
      return LIBVTMDEC_ERROR_READ_ERROR;

    // Check the NAL unit header
    uint8_t *data = (uint8_t*)data8;
    if (length < 4 && !eof)
      return LIBVTMDEC_ERROR_READ_ERROR;

    rawDataAccessStreamBuffer dataStreamBuffer(data, length);

    std::istream dataInputStream(&dataStreamBuffer);
    InputByteStream bytestream(dataInputStream);

    if (length > 0)
    {
      // Read the NAL unit
      InputNALUnit nalu;
      AnnexBStats stats = AnnexBStats();
      byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);
      if (nalu.getBitstream().getFifo().empty())
      {
        /* this can happen if the following occur:
         *  - empty input file
         *  - two back-to-back start_code_prefixes
         *  - start_code_prefix immediately followed by EOF
         */
        return LIBVTMDEC_ERROR_READ_ERROR;
      }
      else
      {
        read(nalu);

        const int iTargetLayer = 0;
        const bool isNaluTheTargetLayer = nalu.m_nuhLayerId == iTargetLayer || iTargetLayer < 0;
        if ((d->m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > d->m_iMaxTemporalLayer) || !isNaluTheTargetLayer)
        {
          bNewPicture = false;
        }
        else
        {
          bNewPicture = d->m_cDecLib.decode(nalu, d->m_iSkipFrame, d->m_iPOCLastDisplay);
          if (bNewPicture)
          {
            // We encountered a new picture in this NAL unit. This means: we will filter the now complete former
            // picture. There might also be pictures to be output/read. After reading these pictures, this function
            // must be called again with the same NAL unit.
          }
        }
      }

      // Filter the picture if decoding is complete
      if ((bNewPicture || eof || nalu.m_nalUnitType == NAL_UNIT_EOS) && !d->m_cDecLib.getFirstSliceInSequence())
      {

        int poc;
        if (!d->loopFiltered || !eof)
        {
          d->m_cDecLib.executeLoopFilters();
          d->m_cDecLib.finishPicture(poc, d->pcListPic);
        }
        d->loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
      }
      else if ((bNewPicture || eof || nalu.m_nalUnitType == NAL_UNIT_EOS) && 
               d->m_cDecLib.getFirstSliceInSequence())
      {
        d->m_cDecLib.setFirstSliceInPicture(true);
      }

      // Check if we might be able to read pictures
      if (d->pcListPic != nullptr)
      {
        d->nrOfTimesToCheckForOutputPictures = 0;
        if (bNewPicture)
        {
          d->nrOfTimesToCheckForOutputPictures++;
        }
        if (nalu.m_nalUnitType == NAL_UNIT_EOS)
        {
          d->nrOfTimesToCheckForOutputPictures++;
          d->m_cDecLib.setFirstSliceInPicture(false);
        }
#if JVET_N0865_GRA2GDR
        if (!bNewPicture && ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL_15)
          || (nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR)))
#else
        if (!bNewPicture && ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL_15)
          || (nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GRA)))
#endif
        {
          d->nrOfTimesToCheckForOutputPictures++;
        }
      }

      checkOutputPictures = (d->nrOfTimesToCheckForOutputPictures > 0);
    }

    if (checkOutputPictures)
      initFrameRetrieval(d);

    if (eof)
    {
      // At the end of the file we have to use the normal output function once and then the flushing
      checkOutputPictures = true;
      initFrameFlushing(d);
    }

    return LIBVTMDEC_OK;
  }

  VTM_DEC_API libVTMDec_picture *libVTMDec_get_picture(libVTMDec_context* decCtx)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return NULL;

    if (d->pcListPic == NULL)
      return NULL;
    if (d->pcListPic->size() == 0)
      return NULL;
    if (d->pcListPic_readIdx < 0 || d->pcListPic_readIdx > d->pcListPic->size())
      return NULL;

    // Forward the iterator to the picture pcListPic_readIdx
    PicList::iterator iterPic = d->pcListPic->begin();
    for (int i = 0; i < d->pcListPic_readIdx; i++)
      iterPic++;

    if ((*(iterPic))->fieldPic)
      // TODO: Field output not supported (YET?)
      return NULL;

    // Go on in the list until we run out of frames or find one that we can output
    while (iterPic != d->pcListPic->end())
    {
      Picture* pcPic = *(iterPic);

      if ((d->flushOutput && pcPic->neededForOutput) || (pcPic->neededForOutput && pcPic->getPOC() > d->m_iPOCLastDisplay &&
        (d->numPicsNotYetDisplayed > d->numReorderPicsHighestTid || d->dpbFullness > d->maxDecPicBufferingHighestTid)))
      {
        if (!d->flushOutput)
        {
          // Output picture found
          d->numPicsNotYetDisplayed--;
          if (pcPic->referenced == false)
            d->dpbFullness--;
        }

        // update POC of display order
        d->m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if (!pcPic->referenced && pcPic->reconstructed)
        {
          pcPic->reconstructed = false;
        }
        pcPic->neededForOutput = false;

        // Return the picture. The next fime this function is called we want to continue at the next picture.
        d->pcListPic_readIdx++;
        return (libVTMDec_picture*)pcPic;
      }

      iterPic++;
      d->pcListPic_readIdx++;
    }
    // We reached the end of the list wothout finding an output picture
    if (d->nrOfTimesToCheckForOutputPictures > 0)
    {
      // Start the whole process over again
      d->nrOfTimesToCheckForOutputPictures--;
      initFrameRetrieval(d);
      return libVTMDec_get_picture(decCtx);
    }
    if (d->flushOutput)
    {
      // Flushing is over. There nothing we can decode anymore. The end.
      d->pcListPic->clear();
      d->m_iPOCLastDisplay = -MAX_INT;
      d->flushOutput = false;
    }

    return NULL;
  }

  VTM_DEC_API int libVTMDec_get_POC(libVTMDec_picture *pic)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    return pcPic->getPOC();
  }

  VTM_DEC_API int libVTMDec_get_picture_width(libVTMDec_picture *pic, libVTMDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    if (c == LIBVTMDEC_LUMA || pcPic->chromaFormat == CHROMA_444)
      return pcPic->lwidth();
    if (pcPic->chromaFormat == CHROMA_422 || pcPic->chromaFormat == CHROMA_420)
      return pcPic->lwidth() / 2;
    return -1;
  }

  VTM_DEC_API int libVTMDec_get_picture_height(libVTMDec_picture *pic, libVTMDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    if (c == LIBVTMDEC_LUMA || pcPic->chromaFormat == CHROMA_444 || pcPic->chromaFormat == CHROMA_422)
      return pcPic->lheight();
    if (pcPic->chromaFormat == CHROMA_420)
      return pcPic->lheight() / 2;
    return -1;
  }

  VTM_DEC_API int libVTMDec_get_picture_stride(libVTMDec_picture *pic, libVTMDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    if (c == LIBVTMDEC_LUMA)
      return pcPic->getRecoBuf().get(COMPONENT_Y).stride;
    if (c == LIBVTMDEC_CHROMA_U)
      return pcPic->getRecoBuf().get(COMPONENT_Cb).stride;
    if (c == LIBVTMDEC_CHROMA_V)
      return pcPic->getRecoBuf().get(COMPONENT_Cr).stride;
    return -1;
  }

  VTM_DEC_API const int16_t* libVTMDec_get_image_plane(libVTMDec_picture *pic, libVTMDec_ColorComponent c)
  {
    if (pic == NULL)
      return NULL;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return NULL;

    const int confLeft = 0;
    const int confTop = 0;
    ComponentID compID = COMPONENT_Y;
    if (c == LIBVTMDEC_CHROMA_U)
      compID = COMPONENT_Cb;
    if (c == LIBVTMDEC_CHROMA_V)
      compID = COMPONENT_Cr;

    const CPelUnitBuf &picBuf = pcPic->getRecoBuf();
    ChromaFormat format = picBuf.chromaFormat;

    const uint32_t csx = ::getComponentScaleX(compID, format);
    const uint32_t csy = ::getComponentScaleY(compID, format);
    
    const CPelBuf area = picBuf.get(compID);
    const int planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;
    return area.bufAt(0, 0) + planeOffset;
  }

  VTM_DEC_API libVTMDec_ChromaFormat libVTMDec_get_chroma_format(libVTMDec_picture *pic)
  {
    if (pic == NULL)
      return LIBVTMDEC_CHROMA_UNKNOWN;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return LIBVTMDEC_CHROMA_UNKNOWN;

    if (pcPic->chromaFormat == CHROMA_400)
      return LIBVTMDEC_CHROMA_400;
    if (pcPic->chromaFormat == CHROMA_420)
      return LIBVTMDEC_CHROMA_420;
    if (pcPic->chromaFormat == CHROMA_422)
      return LIBVTMDEC_CHROMA_422;
    if (pcPic->chromaFormat == CHROMA_444)
      return LIBVTMDEC_CHROMA_444;
    return LIBVTMDEC_CHROMA_UNKNOWN;
  }

  VTM_DEC_API int libVTMDec_get_internal_bit_depth(libVTMDec_picture *pic, libVTMDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    Picture* pcPic = (Picture*)pic;
    if (pcPic == NULL)
      return -1;

    const BitDepths &bitDepths = pcPic->cs->sps->getBitDepths();
    if (c == LIBVTMDEC_LUMA)
      return bitDepths.recon[CHANNEL_TYPE_LUMA];
    if (c == LIBVTMDEC_CHROMA_U || c == LIBVTMDEC_CHROMA_V)
      return bitDepths.recon[CHANNEL_TYPE_CHROMA];
    return -1;
  }

  // --------- internals --------

  //void addValuesForPUs(vtmDecoderWrapper *d, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, LIBVTMDEC_info_type type)
  //{
  //  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
  //  UInt uiNumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
  //  UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() - uiDepth ) << 1 ) ) >> 4;

  //  const int cuWidth = g_uiMaxCUWidth >> uiDepth;
  //  const int cuHeight = g_uiMaxCUHeight >> uiDepth;
  //  const int cuX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  //  const int cuY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

  //  for (UInt uiPartIdx = 0, uiSubPartIdx = uiAbsPartIdx; uiPartIdx < uiNumPU; uiPartIdx++, uiSubPartIdx += uiPUOffset)
  //  {
  //    // Set the size and position of the PU
  //    LIBVTMDEC_BlockValue b;
  //    switch (ePartSize)
  //    {
  //    case SIZE_2NxN:
  //      b.w = cuWidth;
  //      b.h = cuHeight >> 1;
  //      b.x = cuX;
  //      b.y = (uiPartIdx == 0) ? cuY : cuY + b.h;
  //      break;
  //    case SIZE_Nx2N:
  //      b.w = cuWidth >> 1;
  //      b.h = cuHeight;
  //      b.x = (uiPartIdx == 0) ? cuX : cuX + b.w;
  //      b.y = cuY;
  //      break;
  //    case SIZE_NxN:
  //      b.w = cuWidth >> 1;
  //      b.h = cuHeight >> 1;
  //      b.x = (uiPartIdx == 0 || uiPartIdx == 2) ? cuX : cuX + b.w;
  //      b.y = (uiPartIdx == 0 || uiPartIdx == 1) ? cuY : cuY + b.h;
  //      break;
  //    case SIZE_2NxnU:
  //      b.w = cuWidth;
  //      b.h = (uiPartIdx == 0) ? (cuHeight >> 2) : ((cuHeight >> 2) + (cuHeight >> 1));
  //      b.x = cuX;
  //      b.y = (uiPartIdx == 0) ? cuY : cuY + (cuHeight >> 2);
  //      break;
  //    case SIZE_2NxnD:
  //      b.w = cuWidth;
  //      b.h = (uiPartIdx == 0) ? ((cuHeight >> 2) + (cuHeight >> 1)) : (cuHeight >> 2);
  //      b.x = cuX;
  //      b.y = (uiPartIdx == 0) ? cuY : cuY + (cuHeight >> 2) + (cuHeight >> 1);
  //      break;
  //    case SIZE_nLx2N:
  //      b.w = (uiPartIdx == 0) ? (cuWidth >> 2) : ((cuWidth >> 2) + (cuWidth >> 1));
  //      b.h = cuHeight;
  //      b.x = (uiPartIdx == 0) ? cuX : cuX + (cuWidth >> 2);
  //      b.y = cuY;
  //      break;
  //    case SIZE_nRx2N:
  //      b.w = (uiPartIdx == 0) ? ((cuWidth >> 2) + (cuWidth >> 1)) : (cuWidth >> 2);
  //      b.h = cuHeight;
  //      b.x = (uiPartIdx == 0) ? cuX : cuX + (cuWidth >> 2) + (cuWidth >> 1);
  //      b.y = cuY;
  //      break;
  //    case SIZE_2Nx2N:
  //      b.w = cuWidth;
  //      b.h = cuHeight;
  //      b.x = cuX;
  //      b.y = cuY;
  //      break;
  //    default:
  //      assert(false);
  //    }

  //    // Get the value that we want to save for this PU
  //    if (type == LIBVTMDEC_PU_MERGE_FLAG)
  //      b.value = pcCU->getMergeFlag(uiSubPartIdx) ? 1 : 0;
  //    if (type == LIBVTMDEC_PU_MERGE_INDEX && pcCU->getMergeFlag(uiSubPartIdx))
  //      b.value = (int)pcCU->getMergeIndex(uiSubPartIdx);
  //    if (type == LIBVTMDEC_PU_UNI_BI_PREDICTION)
  //      b.value = (int)pcCU->getInterDir(uiSubPartIdx);
  //    if (type == LIBVTMDEC_PU_REFERENCE_POC_0)
  //      b.value = pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiSubPartIdx);
  //    if (type == LIBVTMDEC_PU_MV_0)
  //    {
  //      b.value  = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiSubPartIdx).getHor();
  //      b.value2 = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiSubPartIdx).getVer();
  //    }
  //    if (type == LIBVTMDEC_PU_REFERENCE_POC_1 && pcCU->getInterDir(uiSubPartIdx) == 2)
  //      b.value = pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(uiSubPartIdx);
  //    if (type == LIBVTMDEC_PU_MV_1 && pcCU->getInterDir(uiSubPartIdx) == 2)
  //    {
  //      b.value  = pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiSubPartIdx).getHor();
  //      b.value2 = pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiSubPartIdx).getVer();
  //    }

  //    // Add the value
  //    d->internalsBlockData.push_back(b);
  //  }
  //}

  //void addValuesForTURecursive(vtmDecoderWrapper *d, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt trDepth, LIBVTMDEC_info_type type)
  //{
  //  UInt trIdx = pcCU->getTransformIdx(uiAbsPartIdx);
  //  if (trDepth < trIdx)
  //  {
  //    // Split
  //    UInt uiNextDepth = uiDepth + trDepth + 1;
  //    UInt uiQNumParts = pcCU->getTotalNumPart() >> (uiNextDepth<<1);

  //    for (int i = 0; i < 4; i++)
  //      addValuesForTURecursive(d, pcCU, uiAbsPartIdx + i * uiQNumParts, uiDepth, trDepth + 1, type);
  //  }

  //  // We are not at the TU level
  //  UInt uiLPelX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  //  UInt uiTPelY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

  //  LIBVTMDEC_BlockValue b;
  //  b.x = uiLPelX;
  //  b.y = uiTPelY;
  //  b.w = (g_uiMaxCUWidth >> (uiDepth + trDepth));
  //  b.h = (g_uiMaxCUHeight >> (uiDepth + trDepth));
  //  if (type == LIBVTMDEC_TU_CBF_Y)
  //    b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, trDepth) != 0) ? 1 : 0;
  //  else if (type == LIBVTMDEC_TU_CBF_CB)
  //    b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cb, trDepth) != 0) ? 1 : 0;
  //  else if (type == LIBVTMDEC_TU_CBF_CR)
  //    b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cr, trDepth) != 0) ? 1 : 0;
  //  else if (type == LIBVTMDEC_TU_COEFF_TR_SKIP_Y)
  //    b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Y) != 0) ? 1 : 0;
  //  else if (type == LIBVTMDEC_TU_COEFF_TR_SKIP_Cb)
  //    b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Cb) != 0) ? 1 : 0;
  //  else if (type == LIBVTMDEC_TU_COEFF_TR_SKIP_Cr)
  //    b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Cr) != 0) ? 1 : 0;
  //  else if (type == LIBVTMDEC_TU_COEFF_ENERGY_Y || type == LIBVTMDEC_TU_COEFF_ENERGY_CB || type == LIBVTMDEC_TU_COEFF_ENERGY_CB)
  //  {
  //    ComponentID c = (type == LIBVTMDEC_TU_COEFF_ENERGY_Y) ? COMPONENT_Y : (type == LIBVTMDEC_TU_COEFF_ENERGY_CB) ? COMPONENT_Cb : COMPONENT_Cr;
  //    
  //    const int nrCoeff = (type == LIBVTMDEC_TU_COEFF_ENERGY_Y) ? b.w * b.h : b.w/2 * b.h/2;
  //    
  //    TCoeff* pcCoef = pcCU->getCoeff(c);
  //    int64_t e = 0;
  //    for (int i = 0; i < nrCoeff; i++)
  //    {
  //      TCoeff co = pcCoef[i];
  //      e += co * co;
  //    }
  //    if (e > MAX_INT)
  //      b.value = MAX_INT;
  //    else
  //      b.value = (int)e;
  //  }
  //  d->internalsBlockData.push_back(b);
  //}

  //void addValuesForCURecursively(vtmDecoderWrapper *d, TComDataCU* pcLCU, UInt uiAbsPartIdx, UInt uiDepth, LIBVTMDEC_info_type type)
  //{
  //  Picture* pcPic = pcLCU->getPic();

  //  Bool bBoundary = false;
  //  UInt uiLPelX   = pcLCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  //  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth >> uiDepth)  - 1;
  //  UInt uiTPelY   = pcLCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  //  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight >> uiDepth) - 1;

  //  UInt uiCurNumParts = pcPic->getNumPartInCU() >> (uiDepth<<1);
  //  TComSlice *pcSlice = pcLCU->getPic()->getSlice(pcLCU->getPic()->getCurrSliceIdx());
  //  Bool bStartInCU = (pcLCU->getSCUAddr() + uiAbsPartIdx + uiCurNumParts > pcSlice->getSliceSegmentCurStartCUAddr()) && (pcLCU->getSCUAddr() + uiAbsPartIdx < pcSlice->getSliceSegmentCurStartCUAddr());
  //  if (bStartInCU || (uiRPelX >= pcSlice->getSPS()->getPicWidthInLumaSamples()) || (uiBPelY >= pcSlice->getSPS()->getPicHeightInLumaSamples()))
  //    bBoundary = true;

  //  if(((uiDepth < pcLCU->getDepth(uiAbsPartIdx)) && (uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth)) || bBoundary)
  //  {
  //    UInt uiNextDepth = uiDepth + 1;
  //    UInt uiQNumParts = pcLCU->getTotalNumPart() >> (uiNextDepth<<1);
  //    UInt uiIdx = uiAbsPartIdx;
  //    for (UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++)
  //    {
  //      uiLPelX = pcLCU->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[uiIdx]];
  //      uiTPelY = pcLCU->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[uiIdx]];

  //      if ((uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples()) && (uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples()))
  //        addValuesForCURecursively(d, pcLCU, uiIdx, uiNextDepth, type);

  //      uiIdx += uiQNumParts;
  //    }
  //    return;
  //  }

  //  // We reached the CU
  //  if (type == LIBVTMDEC_CU_PREDICTION_MODE || type == LIBVTMDEC_CU_TRQ_BYPASS || type == LIBVTMDEC_CU_SKIP_FLAG || type == LIBVTMDEC_CU_PART_MODE || type == LIBVTMDEC_CU_INTRA_MODE_LUMA || type == LIBVTMDEC_CU_INTRA_MODE_CHROMA || type == LIBVTMDEC_CU_ROOT_CBF)
  //  {
  //    if ((type == LIBVTMDEC_CU_TRQ_BYPASS && !pcLCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) ||
  //        (type == LIBVTMDEC_CU_INTRA_MODE_LUMA && !pcLCU->isIntra(uiAbsPartIdx)) ||
  //        (type == LIBVTMDEC_CU_INTRA_MODE_CHROMA && !pcLCU->isIntra(uiAbsPartIdx)) ||
  //        (type == LIBVTMDEC_CU_ROOT_CBF && pcLCU->isInter(uiAbsPartIdx)))
  //      return;

  //    LIBVTMDEC_BlockValue b;
  //    b.x = uiLPelX;
  //    b.y = uiTPelY;
  //    b.w = (g_uiMaxCUWidth>>uiDepth);
  //    b.h = (g_uiMaxCUHeight>>uiDepth);
  //    if (type == LIBVTMDEC_CU_PREDICTION_MODE)
  //      b.value = int(pcLCU->getPredictionMode(uiAbsPartIdx));
  //    else if (type == LIBVTMDEC_CU_TRQ_BYPASS)
  //      b.value = pcLCU->getCUTransquantBypass(uiAbsPartIdx) ? 1 : 0;
  //    else if (type == LIBVTMDEC_CU_SKIP_FLAG)
  //      b.value =  pcLCU->isSkipped(uiAbsPartIdx) ? 1 : 0;
  //    else if (type == LIBVTMDEC_CU_PART_MODE)
  //      b.value = (int)pcLCU->getPartitionSize(uiAbsPartIdx);
  //    else if (type == LIBVTMDEC_CU_INTRA_MODE_LUMA)
  //      b.value = (int)pcLCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx);
  //    else if (type == LIBVTMDEC_CU_INTRA_MODE_CHROMA)
  //      b.value = (int)pcLCU->getIntraDir(CHANNEL_TYPE_CHROMA, uiAbsPartIdx);
  //    else if (type == LIBVTMDEC_CU_ROOT_CBF)
  //      b.value = (int)pcLCU->getQtRootCbf(uiAbsPartIdx);
  //    d->internalsBlockData.push_back(b);
  //  }
  //  else if (pcLCU->isInter(uiAbsPartIdx) && (type == LIBVTMDEC_PU_MERGE_FLAG || type == LIBVTMDEC_PU_UNI_BI_PREDICTION || type == LIBVTMDEC_PU_REFERENCE_POC_0 || type == LIBVTMDEC_PU_MV_0 || type == LIBVTMDEC_PU_REFERENCE_POC_1 || type == LIBVTMDEC_PU_MV_1))
  //    // Set values for every PU
  //    addValuesForPUs(d, pcLCU, uiAbsPartIdx, uiDepth, type);
  //  else if (type == LIBVTMDEC_TU_CBF_Y || type == LIBVTMDEC_TU_CBF_CB || type == LIBVTMDEC_TU_CBF_CR || type == LIBVTMDEC_TU_COEFF_ENERGY_Y || type == LIBVTMDEC_TU_COEFF_ENERGY_CB || type == LIBVTMDEC_TU_COEFF_ENERGY_CR || type == LIBVTMDEC_TU_COEFF_TR_SKIP_Y || type == LIBVTMDEC_TU_COEFF_TR_SKIP_Cb || type == LIBVTMDEC_TU_COEFF_TR_SKIP_Cr)
  //    addValuesForTURecursive(d, pcLCU, uiAbsPartIdx, uiDepth, 0, type);
  //}

  VTM_DEC_API std::vector<libVTMDec_BlockValue> *libVTMDec_get_internal_info(libVTMDec_context *decCtx, libVTMDec_picture *pic, libVTMDec_info_type type)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return NULL;

    // Clear the internals before adding new ones
    d->internalsBlockData.clear();

    //if (pic == NULL)
    //  return NULL;
    //Picture* pcPic = (Picture*)pic;
    //if (pcPic == NULL)
    //  return NULL;
    //PictureSym *s = pcPic->getPicSym();
    //if (s == NULL)
    //  return NULL;

    //int nrCU = s->getNumberOfCUsInFrame();
    //for (int i = 0; i < nrCU; i++)
    //{
    //  TComDataCU *pcLCU = s->getCU(i);

    //  if ((type == LIBVTMDEC_TU_COEFF_TR_SKIP_Y || type == LIBVTMDEC_TU_COEFF_TR_SKIP_Cb || type == LIBVTMDEC_TU_COEFF_TR_SKIP_Cr) && pcLCU->getSlice()->getPPS()->getUseTransformSkip())
    //    // Transform skip not enabled for this slice
    //    continue;

    //  if (type == LIBVTMDEC_CTU_SLICE_INDEX)
    //  {
    //    LIBVTMDEC_BlockValue b;
    //    b.x = pcLCU->getCUPelX();
    //    b.y = pcLCU->getCUPelY();
    //    b.w = g_uiMaxCUWidth;
    //    b.h = g_uiMaxCUHeight;
    //    b.value = (int)pcLCU->getPic()->getCurrSliceIdx();
    //    d->internalsBlockData.push_back(b);
    //  }
    //  else
    //    addValuesForCURecursively(d, pcLCU, 0, 0, type);
    //}

    return &d->internalsBlockData;
  }

  VTM_DEC_API libVTMDec_error libVTMDec_clear_internal_info(libVTMDec_context *decCtx)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return LIBVTMDEC_ERROR;

    // Clear the internals
    d->internalsBlockData.clear();

    return LIBVTMDEC_OK;
  }

} // extern "C"
