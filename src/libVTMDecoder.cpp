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

/** \file     libVTMDecoder.cpp
    \brief    Decoder library class
*/

#include "libVTMDecoder.h"

#include "Logger.h"
#include "DecoderLib/DecLib.h"
#include "DecoderLib/NALread.h"
#include "DecoderLib/AnnexBread.h"

#include <fstream>

enum class PictureBufferStatus
{
  NORMAL,
  SCHEDULE_FLUSHING_EOF,
  FLUSHING_EOF,
  FLUSHING_NEEDED_IN_BITSTREAM
};

class vtmDecoderWrapper
{
public:
  vtmDecoderWrapper()
  {
    // Initialize the decoder
    initROM();
    m_cDecLib.create();
    m_cDecLib.init();
    const bool m_decodedPictureHashSEIEnabled = true;
    m_cDecLib.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);

    m_cDecLib.m_targetSubPicIdx = 0;
    m_cDecLib.initScalingList();
  }
  ~vtmDecoderWrapper() 
  { 
    m_cDecLib.destroy(); 
  };

  bool initAndCheckFrameRetrieval();
  bool iterateLoopToNextPicture();

  // Members of the DecApp
  int m_iPOCLastDisplay{ -MAX_INT };
  DecLib m_cDecLib;
  // Members of the DecAppCfg
  int m_targetOlsIdx {-1};
  std::vector<int> m_targetOutputLayerIdSet;          ///< set of LayerIds to be outputted
  int m_iMaxTemporalLayer{ -1 };
  std::vector<int> m_targetDecLayerIdSet;             ///< set of LayerIds to be included in the sub-bitstream extraction process.

  unsigned int nrNalUnitsPushed {0};
  PictureBufferStatus bufferStatus { PictureBufferStatus::NORMAL };

  // Variables which are locally defined in DecApp::decode locally in the loop
  PicList *pcListPic{ nullptr };
  bool loopFiltered[MAX_VPS_LAYERS] = { false };
  bool bPicSkipped {false};

  // Variables which are locally defined in DecApp::xWriteOutput
  int numPicsNotYetDisplayed{ 0 };
  int dpbFullness{ 0 };
  uint32_t numReorderPicsHighestTid{ 0 };
  uint32_t maxDecPicBufferingHighestTid{ 0 };

  // 
  int pcListPic_readIdx{ 0 };

  // In the DecApp the xWriteOutput function can be called up to 3 times after a NAL was pushed
  // (or maybe this is an unclear programming error). However, this is how me mimic this.
  int nrOfTimesToCheckForOutputPictures{ 0 };
  
  // The vector that is filled when internals are returned.
  // The vector is defined, filled and cleared only in this library so that no chaos is created
  // between the heap of the shared library and the caller programm.
  std::vector<libVTMDec_BlockValue> internalsBlockData;

  Logger log;
};

bool vtmDecoderWrapper::iterateLoopToNextPicture()
{
  assert(this->pcListPic_readIdx >= 0);
  auto iterPic = this->pcListPic->begin();
  std::advance(iterPic, this->pcListPic_readIdx);

  const bool flushOutput = this->bufferStatus == PictureBufferStatus::FLUSHING_EOF;

  Picture* pcPic = *(iterPic);
  if (this->numPicsNotYetDisplayed>2 && pcPic->fieldPic)
  {
    this->log(Logger::LogLevel::ERROR) << "Field decoding not supported (yet)";
    return false;
  }
  else if( !pcPic->fieldPic ) //Frame Decoding
  {
    while (iterPic != this->pcListPic->end())
    {
      pcPic = *(iterPic);
      bool outputPic;
      if (flushOutput)
        outputPic = pcPic->neededForOutput;
      else
        outputPic = pcPic->neededForOutput && pcPic->getPOC() > this->m_iPOCLastDisplay &&
        (this->numPicsNotYetDisplayed > this->numReorderPicsHighestTid || this->dpbFullness > this->maxDecPicBufferingHighestTid);

      if(outputPic)
      {
        this->log(Logger::LogLevel::INFO) << "Found picture to " << (flushOutput ? "flush" : "writeOut") << " POC " << pcPic->getPOC() << std::endl;
        return true;
      }

      iterPic++;
      this->pcListPic_readIdx++;
    }
  }

  if (this->nrOfTimesToCheckForOutputPictures > 1)
  {
    this->nrOfTimesToCheckForOutputPictures--;
    if (this->bufferStatus == PictureBufferStatus::SCHEDULE_FLUSHING_EOF)
    {
      this->log(Logger::LogLevel::INFO) << "No more frames to write out normally. Switching to flushing." << std::endl;
      this->bufferStatus = PictureBufferStatus::FLUSHING_EOF;
    }
    return this->initAndCheckFrameRetrieval();
  }

  if (this->bufferStatus == PictureBufferStatus::FLUSHING_EOF)
  {
    this->log(Logger::LogLevel::INFO) << "No more pictures found for flushing. Decoding ended." << std::endl;
    return false;
  }
  
  this->log(Logger::LogLevel::INFO) << "No picture found for output. Waiting for more data to decode." << std::endl;
  this->pcListPic_readIdx = -1;
  return false;
}

/* Init frame retriveal (as xWriteOutput does). Then go to the next picture in the picList which
 * is ready to be returned. Return true if a picture is found. Return false if not.
 */
bool vtmDecoderWrapper::initAndCheckFrameRetrieval()
{
  this->log(Logger::LogLevel::INFO) << "Initializing frame retrieval (" << this->nrOfTimesToCheckForOutputPictures << ")" << std::endl;

  this->pcListPic_readIdx = 0;

  if (this->bufferStatus == PictureBufferStatus::FLUSHING_EOF)
  {
    // This is what xFlushOutput does for initialization. Well .. nothing really.
  }
  else
  {
    // This is what xWriteOutput does before iterating over the pictures
    this->numPicsNotYetDisplayed = 0;
    this->dpbFullness = 0;
    const SPS* activeSPS = (this->pcListPic->front()->cs->sps);
    uint32_t maxNrSublayers = activeSPS->getMaxTLayers();

    const VPS* referredVPS = this->pcListPic->front()->cs->vps;
    const int temporalId = ( this->m_iMaxTemporalLayer == -1 || this->m_iMaxTemporalLayer >= maxNrSublayers ) ? maxNrSublayers - 1 : this->m_iMaxTemporalLayer;

    if( referredVPS == nullptr || referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] == 1 )
    {
      this->numReorderPicsHighestTid = activeSPS->getNumReorderPics( temporalId );
      this->maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering( temporalId );
    }
    else
    {
      this->numReorderPicsHighestTid = referredVPS->getNumReorderPics( temporalId );
      this->maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering( temporalId );
    }

    for (const auto &pcPic : *(this->pcListPic))
    {
      if (pcPic->neededForOutput && pcPic->getPOC() > this->m_iPOCLastDisplay)
      {
        this->numPicsNotYetDisplayed++;
        this->dpbFullness++;
      }
      else if (pcPic->referenced)
      {
        this->dpbFullness++;
      }
    }

    if (this->numPicsNotYetDisplayed > 2)
    {
      auto it = this->pcListPic->begin();
      it++;
      const bool fieldDecoding = (*it)->fieldPic;
      if (fieldDecoding)
        this->pcListPic_readIdx = 1;
    }
  }
  
  {
    std::stringstream pocList;
    std::string delimiter;
    for (const auto &pic : *(this->pcListPic))
    {
      pocList << delimiter << pic->getPOC();
      if (pic->neededForOutput || pic->reconstructed)
      {
        pocList << "(" << (pic->neededForOutput ? "O" : "") << (pic->reconstructed ? "R" : "") << ")";
      }
      delimiter = ", ";
    }
    this->log(Logger::LogLevel::INFO) << "Get Picture - List: [" << pocList.str() << "] readIndex " << this->pcListPic_readIdx << std::endl;
  }

  return this->iterateLoopToNextPicture();
}

bool libCheckPictureHeaderInSliceHeaderFlag(InputNALUnit& nalu)
{
  InputBitstream& bitstream = nalu.getBitstream();
  // Or do we have to seek/peek here?
  return (bool)bitstream.read(1);
}

bool isNewPicture(InputNALUnit &nalu)
{
  switch( nalu.m_nalUnitType ) 
  {
    // NUT that indicate the start of a new picture
    case NAL_UNIT_ACCESS_UNIT_DELIMITER:
    case NAL_UNIT_DCI:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
    case NAL_UNIT_PH:
      return true;

    // NUT that may be the start of a new picture - check first bit in slice header
    case NAL_UNIT_CODED_SLICE_TRAIL:
    case NAL_UNIT_CODED_SLICE_STSA:
    case NAL_UNIT_CODED_SLICE_RASL:
    case NAL_UNIT_CODED_SLICE_RADL:
    case NAL_UNIT_RESERVED_VCL_4:
    case NAL_UNIT_RESERVED_VCL_5:
    case NAL_UNIT_RESERVED_VCL_6:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_GDR:
    case NAL_UNIT_RESERVED_IRAP_VCL_11:
    case NAL_UNIT_RESERVED_IRAP_VCL_12:
      return libCheckPictureHeaderInSliceHeaderFlag(nalu);
      
    // NUT that are not the start of a new picture
    case NAL_UNIT_EOS:
    case NAL_UNIT_EOB:
    case NAL_UNIT_SUFFIX_APS:
    case NAL_UNIT_SUFFIX_SEI:
    case NAL_UNIT_FD:
      return false;

    // NUT that might indicate the start of a new picture - keep looking
    case NAL_UNIT_PREFIX_APS:
    case NAL_UNIT_PREFIX_SEI:
    case NAL_UNIT_RESERVED_NVCL_26:
    case NAL_UNIT_RESERVED_NVCL_27:
    case NAL_UNIT_UNSPECIFIED_28:
    case NAL_UNIT_UNSPECIFIED_29:
    case NAL_UNIT_UNSPECIFIED_30:
    case NAL_UNIT_UNSPECIFIED_31:
    default:
      return false;
  }
}

void filterPicture(vtmDecoderWrapper *d, uint32_t nuhLayerId, bool eof = false)
{
  // Filter the picture if decoding is complete
  if (!d->m_cDecLib.getFirstSliceInSequence(nuhLayerId) && !d->bPicSkipped)
  {
    if (eof || !d->loopFiltered[nuhLayerId])
    {
      int poc;
      d->m_cDecLib.executeLoopFilters();
      d->m_cDecLib.finishPicture( poc, d->pcListPic );
      d->log(Logger::LogLevel::INFO) << "Executed loop filter for POC " << poc << std::endl;
    }
    if (!eof)
      d->loopFiltered[nuhLayerId] = eof;
    if (eof)
    {
      d->m_cDecLib.setFirstSliceInSequence(true, nuhLayerId);
    }

    d->m_cDecLib.updateAssociatedIRAP();
  }
  else if (d->m_cDecLib.getFirstSliceInSequence(nuhLayerId))
  {
    d->m_cDecLib.setFirstSliceInPicture(true);
  }
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
    {
      decCtx->log(Logger::LogLevel::ERROR) << "Error allocating new VTM decoder." << std::endl;
      return NULL;
    }

    decCtx->log(Logger::LogLevel::INFO) << "Allocated new VTM decoder." << std::endl;
    return (libVTMDec_context*)decCtx;
  }

  VTM_DEC_API libVTMDec_error libVTMDec_free_decoder(libVTMDec_context* decCtx)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return LIBVTMDEC_ERROR;

    if (d->pcListPic != nullptr)
    {
      d->log(Logger::LogLevel::INFO) << "Deleting decoder reference buffer." << std::endl;
      for (const auto &pcPic : *d->pcListPic)
      {
        pcPic->destroy();
        delete pcPic;
      }
      d->pcListPic->clear();
    }

    d->log(Logger::LogLevel::INFO) << "Deleting vtm decoder." << std::endl;
    delete d;
    return LIBVTMDEC_OK;
  }

  VTM_DEC_API libVTMDec_error libVTMDec_set_log_callback(libVTMDec_context* decCtx, void *userData, void (*callback)(void*, int, const char*))
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return LIBVTMDEC_ERROR;

    d->log.set_log_callback(userData, callback);
    d->log(Logger::LogLevel::INFO) << "Successfully set logger callback. Hello world." << std::endl;
    return LIBVTMDEC_OK;
  }

  VTM_DEC_API void libVTMDec_set_SEI_Check(libVTMDec_context* decCtx, bool check_hash)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->m_cDecLib.setDecodedPictureHashSEIEnabled(check_hash);
    d->log(Logger::LogLevel::INFO) << "Enabled SEI picture hash checking." << std::endl;
  }

  VTM_DEC_API void libVTMDec_set_max_temporal_layer(libVTMDec_context* decCtx, int max_layer)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->log(Logger::LogLevel::INFO) << "Set max temporal layer for decoding to " << max_layer << "." << std::endl;
    d->m_iMaxTemporalLayer = max_layer;
  }

  VTM_DEC_API libVTMDec_error libVTMDec_push_nal_unit(libVTMDec_context *decCtx, const void* data8, int length, bool eof)
  {
    vtmDecoderWrapper *d = (vtmDecoderWrapper*)decCtx;
    if (!d)
      return LIBVTMDEC_ERROR;

    if (length <= 0 && !eof)
    {
      d->log(Logger::LogLevel::ERROR) << "Trying to push NAL unit with length " << length << " which is <= 0 and eof is not set." << std::endl;
      return LIBVTMDEC_ERROR_READ_ERROR;
    }

    // Check the NAL unit header
    uint8_t *data = (uint8_t*)data8;
    if (length < 4 && !eof)
    {
      d->log(Logger::LogLevel::ERROR) << "Length of NAL unit is " << length << " which is < 4 and eof is not set. In order to read a NLA unit header we need at least 4 bytes." << std::endl;
      return LIBVTMDEC_ERROR_READ_ERROR;
    }

    rawDataAccessStreamBuffer dataStreamBuffer(data, length);

    std::istream dataInputStream(&dataStreamBuffer);
    InputByteStream bytestream(dataInputStream);
    d->nrOfTimesToCheckForOutputPictures = 0;

    if (d->bufferStatus == PictureBufferStatus::FLUSHING_EOF)
    {
      d->log(Logger::LogLevel::ERROR) << "Trying to push data but the decoder is in status flushing because of EOF. No new data must arrive after EOF." << std::endl;
      return LIBVTMDEC_ERROR;
    }
    d->bufferStatus = PictureBufferStatus::NORMAL;

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
        d->log(Logger::LogLevel::ERROR) << "Parsing the NAL bytestream failed." << std::endl;
        return LIBVTMDEC_ERROR_READ_ERROR;
      }

      read(nalu);
      bool bNewPicture = !d->m_cDecLib.getFirstSliceInPicture() && isNewPicture(nalu);
      d->log(Logger::LogLevel::INFO) << "Read NAL unit type " << nalu.m_nalUnitType << " layer " << nalu.m_nuhLayerId << (bNewPicture ? " newPicture" : "") << std::endl;

      if (bNewPicture)
      {
        filterPicture(d, nalu.m_nuhLayerId);
        d->nrOfTimesToCheckForOutputPictures++;
      }
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        d->nrOfTimesToCheckForOutputPictures++;
      }
      if (!bNewPicture && ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_IRAP_VCL_12)
        || (nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR)))
      {
        d->nrOfTimesToCheckForOutputPictures++;
      }

      if (d->nrOfTimesToCheckForOutputPictures > 0)
      {
        if (d->pcListPic == nullptr)
          d->nrOfTimesToCheckForOutputPictures = 0;
        else if (d->initAndCheckFrameRetrieval())
          return LIBVTMDEC_OK_FLUSH_REPUSH;
      }

      // flush output for first slice of an IDR picture
      if(d->bufferStatus != PictureBufferStatus::NORMAL &&
          d->m_cDecLib.getFirstSliceInPicture() &&
          (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP))
      {
        d->bufferStatus = PictureBufferStatus::FLUSHING_NEEDED_IN_BITSTREAM;
        bNewPicture = true;
        return LIBVTMDEC_OK_FLUSH_REPUSH;
      }

      bool isNaluWithinTargetDecLayerIdSet = false;
      if (d->m_targetDecLayerIdSet.empty())
      {
        isNaluWithinTargetDecLayerIdSet = true;
      }
      else
      {
        isNaluWithinTargetDecLayerIdSet = (std::count(d->m_targetDecLayerIdSet.begin(), d->m_targetDecLayerIdSet.end(), nalu.m_nuhLayerId) != 0);
      }

      // parse NAL unit syntax if within target decoding layer
      if( ( d->m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= d->m_iMaxTemporalLayer ) && isNaluWithinTargetDecLayerIdSet )
      {
        if (nalu.m_temporalId > d->m_iMaxTemporalLayer)
        {
          // bitstream shall not include any NAL unit with TemporalId greater than HighestTid
          return LIBVTMDEC_ERROR;
        }
        if (d->bPicSkipped)
        {
          if ((nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR))
          {
            if (d->m_cDecLib.isSliceNaluFirstInAU(true, nalu))
            {
              d->m_cDecLib.resetAccessUnitNals();
              d->m_cDecLib.resetAccessUnitApsNals();
              d->m_cDecLib.resetAccessUnitPicInfo();
            }
            d->bPicSkipped = false;
          }
        }
        // The concept of "skip frames" does not make sense for a library interface.
        // If you want to ignore N frames at the beginning then just do so.
        int iSkipFrame = 0;
        d->m_cDecLib.decode(nalu, iSkipFrame, d->m_iPOCLastDisplay, d->m_targetOlsIdx);
        d->nrNalUnitsPushed++;
        if (nalu.m_nalUnitType == NAL_UNIT_VPS)
        {
          d->m_cDecLib.deriveTargetOutputLayerSet( d->m_targetOlsIdx );
          d->m_targetDecLayerIdSet = d->m_cDecLib.getVPS()->m_targetLayerIdSet;
          d->m_targetOutputLayerIdSet = d->m_cDecLib.getVPS()->m_targetOutputLayerIdSet;
        }
      }
      else
      {
        d->bPicSkipped = true;
      }

      // TODO: We should be ablte to detect this here, right?
      //       When is this even used in the original decoder? I need to test this.
      //       For now, we rely on the AU delimiters.
      const bool bNewAccessUnit = (nalu.m_nalUnitType == NAL_UNIT_ACCESS_UNIT_DELIMITER);
      if (bNewAccessUnit && d->nrNalUnitsPushed > 1)
      {
        d->m_cDecLib.checkTidLayerIdInAccessUnit();
        d->m_cDecLib.resetAccessUnitSeiTids();
        d->m_cDecLib.checkSEIInAccessUnit();
        d->m_cDecLib.resetAccessUnitSeiPayLoadTypes();
        d->m_cDecLib.resetAccessUnitNals();
        d->m_cDecLib.resetAccessUnitApsNals();
        d->m_cDecLib.resetAccessUnitPicInfo();
      }
    }
    else // eof
    {
      filterPicture(d, -1, true);
    }

    if (eof)
    {
      // At the end of the file we have to use the normal output function once and then the flushing
      d->nrOfTimesToCheckForOutputPictures = 2;
      d->bufferStatus = PictureBufferStatus::SCHEDULE_FLUSHING_EOF;
      d->initAndCheckFrameRetrieval();
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

    auto iterPic = d->pcListPic->begin();
    std::advance(iterPic, d->pcListPic_readIdx);

    if ((*(iterPic))->fieldPic)
    {
      d->log(Logger::LogLevel::ERROR) << "Field decoding not supported (yet)" << std::endl;
      return NULL;
    }

    Picture* pcPic = *(iterPic);

    // update POC of display order
    d->m_iPOCLastDisplay = pcPic->getPOC();

    // erase non-referenced picture in the reference picture list after display
    if (!pcPic->referenced && pcPic->reconstructed)
    {
      pcPic->reconstructed = false;
    }
    pcPic->neededForOutput = false;

    Picture* pcOutputPic = pcPic;

    d->iterateLoopToNextPicture();

    d->log(Logger::LogLevel::INFO) << "Returning picture POC " << pcOutputPic->getPOC() << std::endl;
    return (libVTMDec_picture*)pcOutputPic;
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