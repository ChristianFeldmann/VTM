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

/** \file     DecApp.cpp
    \brief    Decoder application class
*/

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "DecApp.h"
#include "DecoderLib/AnnexBread.h"
#include "DecoderLib/NALread.h"
#if RExt__DECODER_DEBUG_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#include "CommonLib/dtrace_codingstruct.h"


//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

DecApp::DecApp()
: m_iPOCLastDisplay(-MAX_INT)
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in DecApp class
 - delete allocated buffers
 - destroy internal class
 - returns the number of mismatching pictures
 */
uint32_t DecApp::decode()
{
  int                 poc;
  PicList* pcListPic = NULL;

  ifstream bitstreamFile(m_bitstreamFileName.c_str(), ifstream::in | ifstream::binary);
  if (!bitstreamFile)
  {
    EXIT( "Failed to open bitstream file " << m_bitstreamFileName.c_str() << " for reading" ) ;
  }

  InputByteStream bytestream(bitstreamFile);

  if (!m_outputDecodedSEIMessagesFilename.empty() && m_outputDecodedSEIMessagesFilename!="-")
  {
    m_seiMessageFileStream.open(m_outputDecodedSEIMessagesFilename.c_str(), std::ios::out);
    if (!m_seiMessageFileStream.is_open() || !m_seiMessageFileStream.good())
    {
      EXIT( "Unable to open file "<< m_outputDecodedSEIMessagesFilename.c_str() << " for writing decoded SEI messages");
    }
  }

  if (!m_oplFilename.empty() && m_oplFilename!="-")
  {
    m_oplFileStream.open(m_oplFilename.c_str(), std::ios::out);
    if (!m_oplFileStream.is_open() || !m_oplFileStream.good())
    {
      EXIT( "Unable to open file "<< m_oplFilename.c_str() << " to write an opl-file for conformance testing (see JVET-P2008 for details)");
    }
  }

  // create & initialize internal classes
  xCreateDecLib();

  m_iPOCLastDisplay += m_iSkipFrame;      // set the last displayed POC correctly for skip forward.

  // clear contents of colour-remap-information-SEI output file
  if (!m_colourRemapSEIFileName.empty())
  {
    std::ofstream ofile(m_colourRemapSEIFileName.c_str());
    if (!ofile.good() || !ofile.is_open())
    {
      EXIT( "Unable to open file " << m_colourRemapSEIFileName.c_str() << " for writing colour-remap-information-SEI video");
    }
  }

  // main decoder loop
  bool loopFiltered[MAX_VPS_LAYERS] = { false };

  bool bPicSkipped = false;

  while (!!bitstreamFile)
  {
    InputNALUnit nalu;
    nalu.m_nalUnitType = NAL_UNIT_INVALID;

    // determine if next NAL unit will be the first one from a new picture
    bool bNewPicture = isNewPicture(&bitstreamFile, &bytestream);
    bool bNewAccessUnit = bNewPicture && isNewAccessUnit( bNewPicture, &bitstreamFile, &bytestream );
    if(!bNewPicture)
    {
      AnnexBStats stats = AnnexBStats();

      // find next NAL unit in stream
      byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);
      if (nalu.getBitstream().getFifo().empty())
      {
        /* this can happen if the following occur:
         *  - empty input file
         *  - two back-to-back start_code_prefixes
         *  - start_code_prefix immediately followed by EOF
         */
        msg( ERROR, "Warning: Attempt to decode an empty NAL unit\n");
      }
      else
      {
        // read NAL unit header
        read(nalu);

        // flush output for first slice of an IDR picture
        if(m_cDecLib.getFirstSliceInPicture() &&
            (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
             nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP))
        {
          xFlushOutput(pcListPic, nalu.m_nuhLayerId);
        }

        // parse NAL unit syntax if within target decoding layer
        if( ( m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= m_iMaxTemporalLayer ) && xIsNaluWithinTargetDecLayerIdSet( &nalu ) )
        {
          CHECK(nalu.m_temporalId > m_iMaxTemporalLayer, "bitstream shall not include any NAL unit with TemporalId greater than HighestTid");
          if (m_targetDecLayerIdSet.size())
          {
            CHECK(std::find(m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu.m_nuhLayerId) == m_targetDecLayerIdSet.end(), "bitstream shall not contain any other layers than included in the OLS with OlsIdx");
          }
          if (bPicSkipped)
          {
            if ((nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR))
            {
              if (m_cDecLib.isSliceNaluFirstInAU(true, nalu))
              {
                m_cDecLib.resetAccessUnitNals();
                m_cDecLib.resetAccessUnitApsNals();
                m_cDecLib.resetAccessUnitPicInfo();
              }
              bPicSkipped = false;
            }
          }
          m_cDecLib.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay, m_targetOlsIdx);
          if (nalu.m_nalUnitType == NAL_UNIT_VPS)
          {
            m_cDecLib.deriveTargetOutputLayerSet( m_targetOlsIdx );
            m_targetDecLayerIdSet = m_cDecLib.getVPS()->m_targetLayerIdSet;
            m_targetOutputLayerIdSet = m_cDecLib.getVPS()->m_targetOutputLayerIdSet;
          }
        }
        else
        {
          bPicSkipped = true;
        }
      }
    }

    if ((bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && !m_cDecLib.getFirstSliceInSequence(nalu.m_nuhLayerId) && !bPicSkipped)
    {
      if (!loopFiltered[nalu.m_nuhLayerId] || bitstreamFile)
      {
        m_cDecLib.executeLoopFilters();
        m_cDecLib.finishPicture( poc, pcListPic );
      }
      loopFiltered[nalu.m_nuhLayerId] = (nalu.m_nalUnitType == NAL_UNIT_EOS);
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        m_cDecLib.setFirstSliceInSequence(true, nalu.m_nuhLayerId);
      }

      m_cDecLib.updateAssociatedIRAP();
    }
    else if ( (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
      m_cDecLib.getFirstSliceInSequence(nalu.m_nuhLayerId))
    {
      m_cDecLib.setFirstSliceInPicture (true);
    }

    if( pcListPic )
    {
      if( !m_reconFileName.empty() && !m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].isOpen() )
      {
        const BitDepths &bitDepths=pcListPic->front()->cs->sps->getBitDepths(); // use bit depths of first reconstructed picture.
        for( uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++ )
        {
            if( m_outputBitDepth[channelType] == 0 )
            {
                m_outputBitDepth[channelType] = bitDepths.recon[channelType];
            }
        }

        if (m_packedYUVMode && (m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12))
        {
          EXIT ("Invalid output bit-depth for packed YUV output, aborting\n");
        }

        std::string reconFileName = m_reconFileName;
        if( m_reconFileName.compare( "/dev/null" ) && m_cDecLib.getVPS() != nullptr && m_cDecLib.getVPS()->getMaxLayers() > 1 && xIsNaluWithinTargetOutputLayerIdSet( &nalu ) )
        {
          size_t pos = reconFileName.find_last_of('.');
          if (pos != string::npos)
          {
            reconFileName.insert( pos, std::to_string( nalu.m_nuhLayerId ) );
          }
          else
          {
            reconFileName.append( std::to_string( nalu.m_nuhLayerId ) );
          }
        }
        if( ( m_cDecLib.getVPS() != nullptr && ( m_cDecLib.getVPS()->getMaxLayers() == 1 || xIsNaluWithinTargetOutputLayerIdSet( &nalu ) ) ) || m_cDecLib.getVPS() == nullptr )
        {
          m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].open( reconFileName, true, m_outputBitDepth, m_outputBitDepth, bitDepths.recon ); // write mode
        }
      }
      // write reconstruction to file
      if( bNewPicture )
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
        m_cDecLib.setFirstSliceInPicture (false);
      }
      // write reconstruction to file -- for additional bumping as defined in C.5.2.3
      if (!bNewPicture && ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_IRAP_VCL_12)
        || (nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR)))
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
    }
#if JVET_Q0488_SEI_REPETITION_CONSTRAINT
    if( bNewPicture )
    {
      m_cDecLib.checkSeiInPictureUnit();
      m_cDecLib.resetPictureSeiNalus();
    }
#endif
#if JVET_R0201_PREFIX_SUFFIX_APS_CLEANUP
    if (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      m_cDecLib.checkAPSInPictureUnit();
      m_cDecLib.resetPictureUnitNals();
    }
#endif
    if(bNewAccessUnit)
    {
      m_cDecLib.checkTidLayerIdInAccessUnit();
      m_cDecLib.resetAccessUnitSeiTids();
      m_cDecLib.checkSEIInAccessUnit();
      m_cDecLib.resetAccessUnitSeiPayLoadTypes();
        m_cDecLib.resetAccessUnitNals();
        m_cDecLib.resetAccessUnitApsNals();
        m_cDecLib.resetAccessUnitPicInfo();
    }
  }

  xFlushOutput( pcListPic );

  // get the number of checksum errors
  uint32_t nRet = m_cDecLib.getNumberOfChecksumErrorsDetected();

  // delete buffers
  m_cDecLib.deletePicBuffer();
  // destroy internal classes
  xDestroyDecLib();

#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::DestroyInstance();
#endif

  destroyROM();

  return nRet;
}


/**
 - lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new picture
 */
bool DecApp::isNewPicture(ifstream *bitstreamFile, class InputByteStream *bytestream)
{
  bool ret = false;
  bool finished = false;

  // cannot be a new picture if there haven't been any slices yet
  if(m_cDecLib.getFirstSliceInPicture())
  {
    return false;
  }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData* backupStats = new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
  streampos location = bitstreamFile->tellg() - streampos(bytestream->GetNumBufferedBytes());
#else
  streampos location = bitstreamFile->tellg();
#endif

  // look ahead until picture start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg( ERROR, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu);
      switch( nalu.m_nalUnitType ) {

        // NUT that indicate the start of a new picture
        case NAL_UNIT_ACCESS_UNIT_DELIMITER:
        case NAL_UNIT_DCI:
        case NAL_UNIT_VPS:
        case NAL_UNIT_SPS:
        case NAL_UNIT_PPS:
        case NAL_UNIT_PH:
          ret = true;
          finished = true;
          break;

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
          ret = checkPictureHeaderInSliceHeaderFlag(nalu);
          finished = true;
          break;

        // NUT that are not the start of a new picture
        case NAL_UNIT_EOS:
        case NAL_UNIT_EOB:
        case NAL_UNIT_SUFFIX_APS:
        case NAL_UNIT_SUFFIX_SEI:
        case NAL_UNIT_FD:
          ret = false;
          finished = true;
          break;

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
          break;
      }
    }
  }

  // restore previous stream location - minus 3 due to the need for the annexB parser to read three extra bytes
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
  CodingStatistics::SetStatistics(*backupStats);
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg(location-streamoff(3));
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

/**
 - lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new access unit
 */
bool DecApp::isNewAccessUnit( bool newPicture, ifstream *bitstreamFile, class InputByteStream *bytestream )
{
  bool ret = false;
  bool finished = false;

  // can only be the start of an AU if this is the start of a new picture
  if( newPicture == false )
  {
    return false;
  }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData* backupStats = new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
  streampos location = bitstreamFile->tellg() - streampos(bytestream->GetNumBufferedBytes());
#else
  streampos location = bitstreamFile->tellg();
#endif

  // look ahead until access unit start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg( ERROR, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu);
      switch( nalu.m_nalUnitType ) {

        // AUD always indicates the start of a new access unit
        case NAL_UNIT_ACCESS_UNIT_DELIMITER:
          ret = true;
          finished = true;
          break;

        // slice types - check layer ID and POC
        case NAL_UNIT_CODED_SLICE_TRAIL:
        case NAL_UNIT_CODED_SLICE_STSA:
        case NAL_UNIT_CODED_SLICE_RASL:
        case NAL_UNIT_CODED_SLICE_RADL:
        case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
        case NAL_UNIT_CODED_SLICE_IDR_N_LP:
        case NAL_UNIT_CODED_SLICE_CRA:
        case NAL_UNIT_CODED_SLICE_GDR:
          ret = m_cDecLib.isSliceNaluFirstInAU( newPicture, nalu );
          finished = true;
          break;

        // NUT that are not the start of a new access unit
        case NAL_UNIT_EOS:
        case NAL_UNIT_EOB:
        case NAL_UNIT_SUFFIX_APS:
        case NAL_UNIT_SUFFIX_SEI:
        case NAL_UNIT_FD:
          ret = false;
          finished = true;
          break;

        // all other NUT - keep looking to find first VCL
        default:
          break;
      }
    }
  }

  // restore previous stream location
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
  CodingStatistics::SetStatistics(*backupStats);
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

void DecApp::writeLineToOutputLog(Picture * pcPic)
{
  if (m_oplFileStream.is_open() && m_oplFileStream.good())
  {
    const SPS* sps = pcPic->cs->sps;
    PictureHash recon_digest;
    auto numChar = calcMD5(((const Picture*)pcPic)->getRecoBuf(), recon_digest, sps->getBitDepths());


    m_oplFileStream << std::setw(8) << pcPic->getPOC() << "," << std::setw(5) << pcPic->Y().width << "," << std::setw(5) << pcPic->Y().height << "," << hashToString(recon_digest, numChar) << "\n";
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void DecApp::xCreateDecLib()
{
  initROM();

  // create decoder class
  m_cDecLib.create();

  // initialize decoder class
  m_cDecLib.init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    m_cacheCfgFile
#endif
  );
  m_cDecLib.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);


  if (!m_outputDecodedSEIMessagesFilename.empty())
  {
    std::ostream &os=m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
    m_cDecLib.setDecodedSEIMessageOutputStream(&os);
  }
  m_cDecLib.m_targetSubPicIdx = this->m_targetSubPicIdx;
  m_cDecLib.initScalingList();
}

void DecApp::xDestroyDecLib()
{
  if( !m_reconFileName.empty() )
  {
    for( auto & recFile : m_cVideoIOYuvReconFile )
    {
      recFile.second.close();
    }
  }

  // destroy decoder class
  m_cDecLib.destroy();
}


/** \param pcListPic list of pictures to be written to file
    \param tId       temporal sub-layer ID
 */
void DecApp::xWriteOutput( PicList* pcListPic, uint32_t tId )
{
  if (pcListPic->empty())
  {
    return;
  }

  PicList::iterator iterPic   = pcListPic->begin();
  int numPicsNotYetDisplayed = 0;
  int dpbFullness = 0;
  const SPS* activeSPS = (pcListPic->front()->cs->sps);
  uint32_t numReorderPicsHighestTid;
  uint32_t maxDecPicBufferingHighestTid;
  uint32_t maxNrSublayers = activeSPS->getMaxTLayers();

  const VPS* referredVPS = pcListPic->front()->cs->vps;
  const int temporalId = ( m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= maxNrSublayers ) ? maxNrSublayers - 1 : m_iMaxTemporalLayer;

  if( referredVPS == nullptr || referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] == 1 )
  {
    numReorderPicsHighestTid = activeSPS->getNumReorderPics( temporalId );
    maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering( temporalId );
  }
  else
  {
    numReorderPicsHighestTid = referredVPS->getNumReorderPics( temporalId );
    maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering( temporalId );
  }

  while (iterPic != pcListPic->end())
  {
    Picture* pcPic = *(iterPic);
    if(pcPic->neededForOutput && pcPic->getPOC() > m_iPOCLastDisplay)
    {
       numPicsNotYetDisplayed++;
      dpbFullness++;
    }
    else if(pcPic->referenced)
    {
      dpbFullness++;
    }
    iterPic++;
  }

  iterPic = pcListPic->begin();

  if (numPicsNotYetDisplayed>2)
  {
    iterPic++;
  }

  Picture* pcPic = *(iterPic);
  if( numPicsNotYetDisplayed>2 && pcPic->fieldPic ) //Field Decoding
  {
    PicList::iterator endPic   = pcListPic->end();
    endPic--;
    iterPic   = pcListPic->begin();
    while (iterPic != endPic)
    {
      Picture* pcPicTop = *(iterPic);
      iterPic++;
      Picture* pcPicBottom = *(iterPic);

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput &&
          (numPicsNotYetDisplayed >  numReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid) &&
          (!(pcPicTop->getPOC()%2) && pcPicBottom->getPOC() == pcPicTop->getPOC()+1) &&
          (pcPicTop->getPOC() == m_iPOCLastDisplay+1 || m_iPOCLastDisplay < 0))
      {
        // write to file
        numPicsNotYetDisplayed = numPicsNotYetDisplayed-2;
        if ( !m_reconFileName.empty() )
        {
          const Window &conf = pcPicTop->cs->pps->getConformanceWindow();
          const bool isTff = pcPicTop->topField;

          bool display = true;

          if (display)
          {
            m_cVideoIOYuvReconFile[pcPicTop->layerId].write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
                                          m_outputColourSpaceConvert,
                                          false, // TODO: m_packedYUVMode,
                                          conf.getWindowLeftOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowRightOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowTopOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowBottomOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          NUM_CHROMA_FORMAT, isTff );
          }
        }
        writeLineToOutputLog(pcPicTop);
        writeLineToOutputLog(pcPicBottom);

        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( ! pcPicTop->referenced && pcPicTop->reconstructed )
        {
          pcPicTop->reconstructed = false;
        }
        if ( ! pcPicBottom->referenced && pcPicBottom->reconstructed )
        {
          pcPicBottom->reconstructed = false;
        }
        pcPicTop->neededForOutput = false;
        pcPicBottom->neededForOutput = false;
      }
    }
  }
  else if( !pcPic->fieldPic ) //Frame Decoding
  {
    iterPic = pcListPic->begin();

    while (iterPic != pcListPic->end())
    {
      pcPic = *(iterPic);

      if(pcPic->neededForOutput && pcPic->getPOC() > m_iPOCLastDisplay &&
        (numPicsNotYetDisplayed >  numReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
      {
        // write to file
        numPicsNotYetDisplayed--;
        if (!pcPic->referenced)
        {
          dpbFullness--;
        }


        if (!m_reconFileName.empty())
        {
          const Window &conf = pcPic->getConformanceWindow();
          const SPS* sps = pcPic->cs->sps;
          ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
          if( m_upscaledOutput )
          {
            m_cVideoIOYuvReconFile[pcPic->layerId].writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
          }
          else
          {
            m_cVideoIOYuvReconFile[pcPic->layerId].write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
            }
        }
        writeLineToOutputLog(pcPic);

        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if (!pcPic->referenced && pcPic->reconstructed)
        {
          pcPic->reconstructed = false;
        }
        pcPic->neededForOutput = false;
      }

      iterPic++;
    }
  }
}

/** \param pcListPic list of pictures to be written to file
 */
void DecApp::xFlushOutput( PicList* pcListPic, const int layerId )
{
  if(!pcListPic || pcListPic->empty())
  {
    return;
  }
  PicList::iterator iterPic   = pcListPic->begin();

  iterPic   = pcListPic->begin();
  Picture* pcPic = *(iterPic);

  if (pcPic->fieldPic ) //Field Decoding
  {
    PicList::iterator endPic   = pcListPic->end();
    endPic--;
    Picture *pcPicTop, *pcPicBottom = NULL;
    while (iterPic != endPic)
    {
      pcPicTop = *(iterPic);
      iterPic++;
      pcPicBottom = *(iterPic);

      if( pcPicTop->layerId != layerId && layerId != NOT_VALID )
      {
        continue;
      }

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput && !(pcPicTop->getPOC()%2) && (pcPicBottom->getPOC() == pcPicTop->getPOC()+1) )
      {
        // write to file
        if ( !m_reconFileName.empty() )
        {
          const Window &conf = pcPicTop->cs->pps->getConformanceWindow();
          const bool    isTff   = pcPicTop->topField;

          m_cVideoIOYuvReconFile[pcPicTop->layerId].write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        false, // TODO: m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        NUM_CHROMA_FORMAT, isTff );
        }
        writeLineToOutputLog(pcPicTop);
        writeLineToOutputLog(pcPicBottom);
        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if( ! pcPicTop->referenced && pcPicTop->reconstructed )
        {
          pcPicTop->reconstructed = false;
        }
        if( ! pcPicBottom->referenced && pcPicBottom->reconstructed )
        {
          pcPicBottom->reconstructed = false;
        }
        pcPicTop->neededForOutput = false;
        pcPicBottom->neededForOutput = false;

        if(pcPicTop)
        {
          pcPicTop->destroy();
          delete pcPicTop;
          pcPicTop = NULL;
        }
      }
    }
    if(pcPicBottom)
    {
      pcPicBottom->destroy();
      delete pcPicBottom;
      pcPicBottom = NULL;
    }
  }
  else //Frame decoding
  {
    while (iterPic != pcListPic->end())
    {
      pcPic = *(iterPic);

      if( pcPic->layerId != layerId && layerId != NOT_VALID )
      {
        iterPic++;
        continue;
      }

      if (pcPic->neededForOutput)
      {
        // write to file

        if (!m_reconFileName.empty())
        {
          const Window &conf = pcPic->getConformanceWindow();
          const SPS* sps = pcPic->cs->sps;
          ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
          if( m_upscaledOutput )
          {
            m_cVideoIOYuvReconFile[pcPic->layerId].writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
          }
          else
          {
            m_cVideoIOYuvReconFile[pcPic->layerId].write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
                                        m_outputColourSpaceConvert,
                                        m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
            }
        }
        writeLineToOutputLog(pcPic);

        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if (!pcPic->referenced && pcPic->reconstructed)
        {
          pcPic->reconstructed = false;
        }
        pcPic->neededForOutput = false;
      }
      if(pcPic != NULL)
      {
        pcPic->destroy();
        delete pcPic;
        pcPic = NULL;
        *iterPic = nullptr;
      }
      iterPic++;
    }
  }

  if( layerId != NOT_VALID )
  {
    pcListPic->remove_if([](Picture* p) { return p == nullptr; });
  }
  else
  pcListPic->clear();
  m_iPOCLastDisplay = -MAX_INT;
}

/** \param nalu Input nalu to check whether its LayerId is within targetDecLayerIdSet
 */
bool DecApp::xIsNaluWithinTargetDecLayerIdSet( const InputNALUnit* nalu ) const
{
  if( !m_targetDecLayerIdSet.size() ) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find( m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu->m_nuhLayerId ) != m_targetDecLayerIdSet.end();
}

/** \param nalu Input nalu to check whether its LayerId is within targetOutputLayerIdSet
 */
bool DecApp::xIsNaluWithinTargetOutputLayerIdSet( const InputNALUnit* nalu ) const
{
  if( !m_targetOutputLayerIdSet.size() ) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find( m_targetOutputLayerIdSet.begin(), m_targetOutputLayerIdSet.end(), nalu->m_nuhLayerId ) != m_targetOutputLayerIdSet.end();
}

//! \}
