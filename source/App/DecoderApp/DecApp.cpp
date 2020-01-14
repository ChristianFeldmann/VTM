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
#if !JVET_N0278_FIXES
  bool openedReconFile = false; // reconstruction file not yet opened. (must be performed after SPS is seen)
#endif
  bool loopFiltered = false;

#if JVET_P1019_OUTPUT_LAYER_SET
  bool bPicSkipped = false;
#endif

  while (!!bitstreamFile)
  {
#if JVET_P1006_PICTURE_HEADER
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
#if JVET_N0278_FIXES
          xFlushOutput(pcListPic, nalu.m_nuhLayerId);
#else
          xFlushOutput(pcListPic);
#endif
        }

        // parse NAL unit syntax if within target decoding layer
#if JVET_P1019_OUTPUT_LAYER_SET
        if ((m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= m_iMaxTemporalLayer) && isNaluWithinTargetDecLayerIdSet(&nalu))
#else
        if ((m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= m_iMaxTemporalLayer) && isNaluWithinTargetDecLayerIdSet(&nalu) && isNaluTheTargetLayer(&nalu))
#endif
        {
#if JVET_P1019_OUTPUT_LAYER_SET
          if (bPicSkipped)
          {
            if ((nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR))
            {
              if (m_cDecLib.isSliceNaluFirstInAU(true, nalu))
              {
                m_cDecLib.resetAccessUnitNals();
                m_cDecLib.resetAccessUnitApsNals();
              }
              bPicSkipped = false;
            }
          }
#endif
          m_cDecLib.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay);
#if JVET_P1019_OUTPUT_LAYER_SET
          if (nalu.m_nalUnitType == NAL_UNIT_VPS)
          {
            deriveOutputLayerSet();
          }
#endif
        }
#if JVET_P1019_OUTPUT_LAYER_SET
        else
        {
          bPicSkipped = true;
        }
#endif
      }
    }
#else 
    /* location serves to work around a design fault in the decoder, whereby
     * the process of reading a new slice that is the first slice of a new frame
     * requires the DecApp::decode() method to be called again with the same
     * nal unit. */
#if RExt__DECODER_DEBUG_STATISTICS
    CodingStatistics& stat = CodingStatistics::GetSingletonInstance();
    CHECK(m_statMode < STATS__MODE_NONE || m_statMode > STATS__MODE_ALL, "Wrong coding statistics output mode");
    stat.m_mode = m_statMode;

    CodingStatistics::CodingStatisticsData* backupStats = new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
    streampos location = bitstreamFile.tellg() - streampos(bytestream.GetNumBufferedBytes());
#else
    streampos location = bitstreamFile.tellg();
#endif
    AnnexBStats stats = AnnexBStats();

    InputNALUnit nalu;
    byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);

    // call actual decoding function
    bool bNewPicture = false;
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
      read(nalu);
#if JVET_P0366_NUT_CONSTRAINT_FLAGS
      m_cDecLib.checkNalUnitConstraints(nalu.m_nalUnitType);
#endif

      if(m_cDecLib.getFirstSliceInPicture() &&
          (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
           nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP))
      {
#if JVET_N0278_FIXES
        xFlushOutput( pcListPic, nalu.m_nuhLayerId );
#else
        xFlushOutput(pcListPic);
#endif
      }

#if JVET_P1019_OUTPUT_LAYER_SET
      if ((m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu))
#else
      if ((m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu) || !isNaluTheTargetLayer(&nalu))
#endif
      {
        bNewPicture = false;
      }
      else
      {
        bNewPicture = m_cDecLib.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay);
        if (bNewPicture)
        {
          // check if new picture was detected at an access unit delimiter NALU
#if !JVET_N0278_FIXES
          if(nalu.m_nalUnitType != NAL_UNIT_ACCESS_UNIT_DELIMITER)
          {
            msg( ERROR, "Error: New picture detected without access unit delimiter. VVC requires the presence of access unit delimiters.\n");
          }
#endif
          bitstreamFile.clear();
          /* location points to the current nalunit payload[1] due to the
           * need for the annexB parser to read three extra bytes.
           * [1] except for the first NAL unit in the file
           *     (but bNewPicture doesn't happen then) */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
          bitstreamFile.seekg(location);
          bytestream.reset();
          CodingStatistics::SetStatistics(*backupStats);
#else
          bitstreamFile.seekg(location-streamoff(3));
          bytestream.reset();
#endif
        }
      }
    }

#endif


#if JVET_P1019_OUTPUT_LAYER_SET
    if ((bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && !m_cDecLib.getFirstSliceInSequence() && !bPicSkipped)
#else
    if ((bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && !m_cDecLib.getFirstSliceInSequence())
#endif
    {
      if (!loopFiltered || bitstreamFile)
      {
        m_cDecLib.executeLoopFilters();
        m_cDecLib.finishPicture( poc, pcListPic );
#if !JVET_P1006_PICTURE_HEADER
#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
        CodingStatistics::UpdateMaxStat(backupStats);
#endif
#endif
      }
      loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        m_cDecLib.setFirstSliceInSequence(true);
      }

    }
    else if ( (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
              m_cDecLib.getFirstSliceInSequence () )
    {
      m_cDecLib.setFirstSliceInPicture (true);
    }

    if( pcListPic )
    {
#if JVET_N0278_FIXES
      if( !m_reconFileName.empty() && !m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].isOpen() )
#else
      if ( (!m_reconFileName.empty()) && (!openedReconFile) )
#endif
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

#if JVET_N0278_FIXES
        std::string reconFileName = m_reconFileName;
#if JVET_P1019_OUTPUT_LAYER_SET
        if (m_reconFileName.compare("/dev/null") && (m_cDecLib.getVPS() != nullptr) && (m_cDecLib.getVPS()->getMaxLayers() > 1) && (isNaluWithinTargetOutputLayerIdSet(&nalu)))
#else
        if (m_reconFileName.compare("/dev/null") && (m_cDecLib.getVPS() != nullptr) && (m_cDecLib.getVPS()->getMaxLayers() > 1) && (m_iTargetLayer == -1))
#endif
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
#if JVET_P1019_OUTPUT_LAYER_SET
        if(((m_cDecLib.getVPS() != nullptr) &&
              ((m_cDecLib.getVPS()->getMaxLayers() == 1) || (isNaluWithinTargetOutputLayerIdSet(&nalu)))) ||
            (m_cDecLib.getVPS() == nullptr))
#endif
        m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].open(reconFileName, true, m_outputBitDepth, m_outputBitDepth, bitDepths.recon); // write mode
#else
        m_cVideoIOYuvReconFile.open( m_reconFileName, true, m_outputBitDepth, m_outputBitDepth, bitDepths.recon ); // write mode
        openedReconFile = true;
#endif
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
#if JVET_P0363_CLEANUP_NUT_TABLE
      if (!bNewPicture && ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_IRAP_VCL_12)
#else
      if (!bNewPicture && ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL_15)
#endif
        || (nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR)))
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
    }
#if JVET_P1006_PICTURE_HEADER
    if(bNewAccessUnit) 
    {
        m_cDecLib.resetAccessUnitNals();
        m_cDecLib.resetAccessUnitApsNals();
    }
#endif
#if !JVET_P1006_PICTURE_HEADER
#if RExt__DECODER_DEBUG_STATISTICS
    delete backupStats;
#endif
#endif
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

#if JVET_P1019_OUTPUT_LAYER_SET
bool DecApp::deriveOutputLayerSet()
{
  int vps_max_layers_minus1 = m_cDecLib.getVPS()->getMaxLayers() - 1;
  int TotalNumOlss = 0;
  int each_layer_is_an_ols_flag = m_cDecLib.getVPS()->getEachLayerIsAnOlsFlag();
  int ols_mode_idc = m_cDecLib.getVPS()->getOlsModeIdc();
  int num_output_layer_sets_minus1 = m_cDecLib.getVPS()->getNumOutputLayerSets() - 1;
  int i = 0, j = 0, k = 0, r = 0;
  int*  NumOutputLayersInOls;
  int*  NumLayersInOls;
  int** OutputLayerIdInOls;
  int** OutputLayerIdx;
  int** layerIncludedInOlsFlag;
  int** LayerIdInOls;
  int** dependencyFlag;
  int** RefLayerIdx;
  int*  NumRefLayers;

  if (vps_max_layers_minus1 == 0)
    TotalNumOlss = 1;
  else if (each_layer_is_an_ols_flag || ols_mode_idc == 0 || ols_mode_idc == 1)
    TotalNumOlss = vps_max_layers_minus1 + 1;
  else if (ols_mode_idc == 2)
    TotalNumOlss = num_output_layer_sets_minus1 + 1;

  NumOutputLayersInOls = new int[m_cDecLib.getVPS()->getNumOutputLayerSets()];
  NumLayersInOls = new int[m_cDecLib.getVPS()->getNumOutputLayerSets()];
  OutputLayerIdInOls = new int*[TotalNumOlss];
  OutputLayerIdx = new int*[TotalNumOlss];
  layerIncludedInOlsFlag = new int*[TotalNumOlss];
  LayerIdInOls = new int*[TotalNumOlss];

  for (i = 0; i < TotalNumOlss; i++)
  {
    OutputLayerIdInOls[i] = new int[vps_max_layers_minus1 + 1];
    OutputLayerIdx[i] = new int[vps_max_layers_minus1 + 1];
    layerIncludedInOlsFlag[i] = new int[vps_max_layers_minus1 + 1];
    LayerIdInOls[i] = new int[vps_max_layers_minus1 + 1];
  }

  dependencyFlag = new int*[vps_max_layers_minus1 + 1];
  RefLayerIdx = new int*[vps_max_layers_minus1 + 1];
  NumRefLayers = new int[vps_max_layers_minus1 + 1];

  for (i = 0; i <= vps_max_layers_minus1; i++)
  {
    dependencyFlag[i] = new int[vps_max_layers_minus1 + 1];
    RefLayerIdx[i] = new int[vps_max_layers_minus1 + 1];
  }

  for (i = 0; i <= vps_max_layers_minus1; i++) {
    for (j = 0; j <= vps_max_layers_minus1; j++) {
      dependencyFlag[i][j] = m_cDecLib.getVPS()->getDirectRefLayerFlag(i, j);
      for (k = 0; k < i; k++)
        if (m_cDecLib.getVPS()->getDirectRefLayerFlag(i, k) && dependencyFlag[k][j])
          dependencyFlag[i][j] = 1;
    }
  }
  for (i = 0; i <= vps_max_layers_minus1; i++)
  {
    for (j = 0, r = 0; j <= vps_max_layers_minus1; j++)
    {
      if (dependencyFlag[i][j])
        RefLayerIdx[i][r++] = j;
    }
    NumRefLayers[i] = r;
  }

  NumOutputLayersInOls[0] = 1;
  OutputLayerIdInOls[0][0] = m_cDecLib.getVPS()->getLayerId(0);
  for (i = 1; i < TotalNumOlss; i++)
  {
    if (each_layer_is_an_ols_flag || ols_mode_idc == 0)
    {
      NumOutputLayersInOls[i] = 1;
      OutputLayerIdInOls[i][0] = m_cDecLib.getVPS()->getLayerId(i);
    }
    else if (ols_mode_idc == 1) {
      NumOutputLayersInOls[i] = i + 1;
      for (j = 0; j < NumOutputLayersInOls[i]; j++)
        OutputLayerIdInOls[i][j] = m_cDecLib.getVPS()->getLayerId(j);
    }
    else if (ols_mode_idc == 2) {
      for (j = 0; j <= vps_max_layers_minus1; j++)
      {
        layerIncludedInOlsFlag[i][j] = 0;
      }
      for (k = 0, j = 0; k <= vps_max_layers_minus1; k++)
      {
        if (m_cDecLib.getVPS()->getOlsOutputLayerFlag(i, k))
        {
          layerIncludedInOlsFlag[i][k] = 1;
          OutputLayerIdx[i][j] = k;
          OutputLayerIdInOls[i][j++] = m_cDecLib.getVPS()->getLayerId(k);
        }
      }
      NumOutputLayersInOls[i] = j;
      for (j = 0; j < NumOutputLayersInOls[i]; j++)
      {
        int idx = OutputLayerIdx[i][j];
        for (k = 0; k < NumRefLayers[idx]; k++)
          layerIncludedInOlsFlag[i][RefLayerIdx[idx][k]] = 1;
      }
    }
  }

  m_targetOutputLayerIdSet.clear();
  for (i = 0; i < NumOutputLayersInOls[m_iTargetOLS]; i++)
    m_targetOutputLayerIdSet.push_back(OutputLayerIdInOls[m_iTargetOLS][i]);

  NumLayersInOls[0] = 1;
  LayerIdInOls[0][0] = m_cDecLib.getVPS()->getLayerId(0);
  for (i = 1; i < TotalNumOlss; i++)
  {
    if (each_layer_is_an_ols_flag)
    {
      NumLayersInOls[i] = 1;
      LayerIdInOls[i][0] = m_cDecLib.getVPS()->getLayerId(i);
    }
    else if (ols_mode_idc == 0 || ols_mode_idc == 1)
    {
      NumLayersInOls[i] = i + 1;
      for (j = 0; j < NumLayersInOls[i]; j++)
        LayerIdInOls[i][j] = m_cDecLib.getVPS()->getLayerId(j);
    }
    else if (ols_mode_idc == 2)
    {
      for (k = 0, j = 0; k <= vps_max_layers_minus1; k++)
        if (layerIncludedInOlsFlag[i][k])
          LayerIdInOls[i][j++] = m_cDecLib.getVPS()->getLayerId(k);
      NumLayersInOls[i] = j;
    }
  }

  m_targetDecLayerIdSet.clear();
  for (i = 0; i < NumLayersInOls[m_iTargetOLS]; i++)
    m_targetDecLayerIdSet.push_back(LayerIdInOls[m_iTargetOLS][i]);

  delete[] NumOutputLayersInOls;
  delete[] NumLayersInOls;
  delete[] NumRefLayers;

  for (i = 0; i < TotalNumOlss; i++)
  {
    delete[] OutputLayerIdInOls[i];
    delete[] OutputLayerIdx[i];
    delete[] layerIncludedInOlsFlag[i];
    delete[] LayerIdInOls[i];
  }
  delete[] OutputLayerIdInOls;
  delete[] OutputLayerIdx;
  delete[] layerIncludedInOlsFlag;
  delete[] LayerIdInOls;

  for (i = 0; i <= vps_max_layers_minus1; i++)
  {
    delete[] dependencyFlag[i];
    delete[] RefLayerIdx[i];
  }
  delete[] dependencyFlag;
  delete[] RefLayerIdx;

  return true;
}
#endif

#if JVET_P1006_PICTURE_HEADER
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
        case NAL_UNIT_DPS:
        case NAL_UNIT_VPS:
        case NAL_UNIT_SPS:
        case NAL_UNIT_PPS:
        case NAL_UNIT_PH:
          ret = true;
          finished = true;
          break;
        
        // NUT that are not the start of a new picture
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
        case NAL_UNIT_EOS:
        case NAL_UNIT_EOB:
#if JVET_P0588_SUFFIX_APS
        case NAL_UNIT_SUFFIX_APS:
#endif
        case NAL_UNIT_SUFFIX_SEI:
        case NAL_UNIT_FD:
          ret = false;
          finished = true;
          break;
        
        // NUT that might indicate the start of a new picture - keep looking
#if JVET_P0588_SUFFIX_APS
        case NAL_UNIT_PREFIX_APS:
#else
        case NAL_UNIT_APS:
#endif
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
#if JVET_P0588_SUFFIX_APS
        case NAL_UNIT_SUFFIX_APS:
#endif
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
#endif

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

#if !JVET_P1019_OUTPUT_LAYER_SET
  m_cDecLib.setTargetDecLayer(m_iTargetLayer);
#endif

  if (!m_outputDecodedSEIMessagesFilename.empty())
  {
    std::ostream &os=m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
    m_cDecLib.setDecodedSEIMessageOutputStream(&os);
  }
#if JVET_P0257_SCALING_LISTS_SPEEDUP_DEC
  m_cDecLib.initScalingList();
#endif
}

void DecApp::xDestroyDecLib()
{
#if JVET_N0278_FIXES
  if( !m_reconFileName.empty() )
  {
    for( auto & recFile : m_cVideoIOYuvReconFile )
    {
      recFile.second.close();
    }
  }
#else
  if ( !m_reconFileName.empty() )
  {
    m_cVideoIOYuvReconFile.close();
  }
#endif

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

  if(m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= maxNrSublayers)
  {
    numReorderPicsHighestTid = activeSPS->getNumReorderPics(maxNrSublayers-1);
    maxDecPicBufferingHighestTid =  activeSPS->getMaxDecPicBuffering(maxNrSublayers-1);
  }
  else
  {
    numReorderPicsHighestTid = activeSPS->getNumReorderPics(m_iMaxTemporalLayer);
    maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering(m_iMaxTemporalLayer);
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
#if HEVC_SEI
          if( m_decodedNoDisplaySEIEnabled )
          {
            SEIMessages noDisplay = getSeisByType( pcPic->SEIs, SEI::NO_DISPLAY );
            const SEINoDisplay *nd = ( noDisplay.size() > 0 ) ? (SEINoDisplay*) *(noDisplay.begin()) : NULL;
            if( (nd != NULL) && nd->m_noDisplay )
            {
              display = false;
            }
          }
#endif

          if (display)
          {
#if JVET_N0278_FIXES
            m_cVideoIOYuvReconFile[pcPicTop->layerId].write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
#else
            m_cVideoIOYuvReconFile.write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
#endif
                                          m_outputColourSpaceConvert,
                                          false, // TODO: m_packedYUVMode,
                                          conf.getWindowLeftOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowRightOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowTopOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          conf.getWindowBottomOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                          NUM_CHROMA_FORMAT, isTff );
          }
        }

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
#if JVET_N0278_FIXES
            m_cVideoIOYuvReconFile[pcPic->layerId].writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
#else
            m_cVideoIOYuvReconFile.writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
#endif
          }
          else
          {
#if JVET_N0278_FIXES
            m_cVideoIOYuvReconFile[pcPic->layerId].write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
#else
            m_cVideoIOYuvReconFile.write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
#endif
                                        m_outputColourSpaceConvert,
                                        m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
            }
        }

#if HEVC_SEI
        if (m_seiMessageFileStream.is_open())
        {
          m_cColourRemapping.outputColourRemapPic (pcPic, m_seiMessageFileStream);
        }
#endif
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
#if JVET_N0278_FIXES
void DecApp::xFlushOutput( PicList* pcListPic, const int layerId )
#else
void DecApp::xFlushOutput( PicList* pcListPic )
#endif
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

#if JVET_N0278_FIXES
      if( pcPicTop->layerId != layerId && layerId != NOT_VALID )
      {
        continue;
      }
#endif

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput && !(pcPicTop->getPOC()%2) && (pcPicBottom->getPOC() == pcPicTop->getPOC()+1) )
      {
        // write to file
        if ( !m_reconFileName.empty() )
        {
          const Window &conf = pcPicTop->cs->pps->getConformanceWindow();
          const bool    isTff   = pcPicTop->topField;

#if JVET_N0278_FIXES
          m_cVideoIOYuvReconFile[pcPicTop->layerId].write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
#else
          m_cVideoIOYuvReconFile.write( pcPicTop->getRecoBuf(), pcPicBottom->getRecoBuf(),
#endif
                                        m_outputColourSpaceConvert,
                                        false, // TODO: m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( pcPicTop->cs->sps->getChromaFormatIdc() ),
                                        NUM_CHROMA_FORMAT, isTff );
        }

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

#if JVET_N0278_FIXES
      if( pcPic->layerId != layerId && layerId != NOT_VALID )
      {
        iterPic++;
        continue;
      }
#endif

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
#if JVET_N0278_FIXES
            m_cVideoIOYuvReconFile[pcPic->layerId].writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
#else
            m_cVideoIOYuvReconFile.writeUpscaledPicture( *sps, *pcPic->cs->pps, pcPic->getRecoBuf(), m_outputColourSpaceConvert, m_packedYUVMode, m_upscaledOutput, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
#endif
          }
          else
          {
#if JVET_N0278_FIXES
            m_cVideoIOYuvReconFile[pcPic->layerId].write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
#else
            m_cVideoIOYuvReconFile.write( pcPic->getRecoBuf().get( COMPONENT_Y ).width, pcPic->getRecoBuf().get( COMPONENT_Y ).height, pcPic->getRecoBuf(),
#endif
                                        m_outputColourSpaceConvert,
                                        m_packedYUVMode,
                                        conf.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowRightOffset() * SPS::getWinUnitX( chromaFormatIDC ),
                                        conf.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        conf.getWindowBottomOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                                        NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
            }
        }

#if HEVC_SEI
        if (m_seiMessageFileStream.is_open())
        {
          m_cColourRemapping.outputColourRemapPic (pcPic, m_seiMessageFileStream);
        }
#endif

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
#if JVET_N0278_FIXES
        *iterPic = nullptr;
#endif
      }
      iterPic++;
    }
  }

#if JVET_N0278_FIXES
  if( layerId != NOT_VALID )
  {
    pcListPic->remove_if([](Picture* p) { return p == nullptr; });
  }
  else
#endif
  pcListPic->clear();
  m_iPOCLastDisplay = -MAX_INT;
}

/** \param nalu Input nalu to check whether its LayerId is within targetDecLayerIdSet
 */
bool DecApp::isNaluWithinTargetDecLayerIdSet( InputNALUnit* nalu )
{
  if ( m_targetDecLayerIdSet.size() == 0 ) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }
  for (std::vector<int>::iterator it = m_targetDecLayerIdSet.begin(); it != m_targetDecLayerIdSet.end(); it++)
  {
    if ( nalu->m_nuhLayerId == (*it) )
    {
      return true;
    }
  }
  return false;
}

#if JVET_P1019_OUTPUT_LAYER_SET
/** \param nalu Input nalu to check whether its LayerId is within targetOutputLayerIdSet
 */
bool DecApp::isNaluWithinTargetOutputLayerIdSet(InputNALUnit* nalu)
{
  if (m_targetOutputLayerIdSet.size() == 0) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }
  for (std::vector<int>::iterator it = m_targetOutputLayerIdSet.begin(); it != m_targetOutputLayerIdSet.end(); it++)
  {
    if (nalu->m_nuhLayerId == (*it))
    {
      return true;
    }
  }
  return false;
}

#else
/** \param nalu Input nalu to check whether its LayerId is the specified target layer
*/
bool DecApp::isNaluTheTargetLayer(InputNALUnit* nalu)
{
  if (nalu->m_nuhLayerId == m_iTargetLayer || m_iTargetLayer < 0)
    return true;

  return false;
}
#endif

//! \}
