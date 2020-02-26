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

/** \file     DecLib.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "DecLib.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/UnitTools.h"

#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include "AnnexBread.h"
#include "NALread.h"
#if K0149_BLOCK_STATISTICS
#include "CommonLib/dtrace_blockstatistics.h"
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

bool tryDecodePicture( Picture* pcEncPic, const int expectedPoc, const std::string& bitstreamFileName, bool bDecodeUntilPocFound /* = false */, int debugCTU /* = -1*/, int debugPOC /* = -1*/ )
{
  int      poc;
  PicList* pcListPic = NULL;

  static bool bFirstCall      = true;             /* TODO: MT */
#if JVET_P0125_EOS_LAYER_SPECIFIC
  static bool loopFiltered[MAX_VPS_LAYERS] = { false };            /* TODO: MT */
#else
  static bool loopFiltered    = false;            /* TODO: MT */
#endif
  static int  iPOCLastDisplay = -MAX_INT;         /* TODO: MT */

  static std::ifstream* bitstreamFile = nullptr;  /* TODO: MT */
  static InputByteStream* bytestream  = nullptr;  /* TODO: MT */
  bool bRet = false;

  // create & initialize internal classes
  static DecLib *pcDecLib = nullptr;              /* TODO: MT */

  if( pcEncPic )
  {
    if( bFirstCall )
    {
      bitstreamFile = new std::ifstream( bitstreamFileName.c_str(), std::ifstream::in | std::ifstream::binary );
      bytestream    = new InputByteStream( *bitstreamFile );

      CHECK( !*bitstreamFile, "failed to open bitstream file " << bitstreamFileName.c_str() << " for reading" ) ;
      // create decoder class
      pcDecLib = new DecLib;
      pcDecLib->create();

      // initialize decoder class
      pcDecLib->init(
#if  JVET_J0090_MEMORY_BANDWITH_MEASURE
        ""
#endif
      );

      pcDecLib->setDebugCTU( debugCTU );
      pcDecLib->setDebugPOC( debugPOC );
      pcDecLib->setDecodedPictureHashSEIEnabled( true );

      bFirstCall = false;
      msg( INFO, "start to decode %s \n", bitstreamFileName.c_str() );
    }

    bool goOn = true;

    // main decoder loop
    while( !!*bitstreamFile && goOn )
    {
      /* location serves to work around a design fault in the decoder, whereby
       * the process of reading a new slice that is the first slice of a new frame
       * requires the DecApp::decode() method to be called again with the same
       * nal unit. */
      std::streampos location = bitstreamFile->tellg();
      AnnexBStats stats       = AnnexBStats();

      InputNALUnit nalu;
      byteStreamNALUnit( *bytestream, nalu.getBitstream().getFifo(), stats );

      // call actual decoding function
      bool bNewPicture = false;
      if( nalu.getBitstream().getFifo().empty() )
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
        read( nalu );
        int iSkipFrame = 0;

#if JVET_P0288_PIC_OUTPUT
        bNewPicture = pcDecLib->decode(nalu, iSkipFrame, iPOCLastDisplay, 0);
#else
        bNewPicture = pcDecLib->decode(nalu, iSkipFrame, iPOCLastDisplay);
#endif
        if( bNewPicture )
        {
          bitstreamFile->clear();
          /* location points to the current nalunit payload[1] due to the
            * need for the annexB parser to read three extra bytes.
            * [1] except for the first NAL unit in the file
            *     (but bNewPicture doesn't happen then) */
          bitstreamFile->seekg( location - std::streamoff( 3 ) );
          bytestream->reset();
        }
      }

#if JVET_P0125_EOS_LAYER_SPECIFIC
      if ((bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && !pcDecLib->getFirstSliceInSequence(nalu.m_nuhLayerId))
#else
      if( ( bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) && !pcDecLib->getFirstSliceInSequence() )
#endif
      {
#if JVET_P0125_EOS_LAYER_SPECIFIC
        if (!loopFiltered[nalu.m_nuhLayerId] || *bitstreamFile)
#else
        if( !loopFiltered || *bitstreamFile )
#endif
        {
          pcDecLib->finishPictureLight( poc, pcListPic );

          if( pcListPic )
          {
            for( auto & pic : *pcListPic )
            {
              if( pic->poc == poc && (!bDecodeUntilPocFound || expectedPoc == poc ) )
              {
                CHECK( pcEncPic->slices.size() == 0, "at least one slice should be available" );

                CHECK( expectedPoc != poc, "mismatch in POC - check encoder configuration" );

                if( debugCTU < 0 || poc != debugPOC )
                {
                for( int i = 0; i < pic->slices.size(); i++ )
                {
                  if( pcEncPic->slices.size() <= i )
                  {
                    pcEncPic->slices.push_back( new Slice );
                    pcEncPic->slices.back()->initSlice();
                    pcEncPic->slices.back()->setPPS( pcEncPic->slices[0]->getPPS() );
                    pcEncPic->slices.back()->setSPS( pcEncPic->slices[0]->getSPS() );
                    pcEncPic->slices.back()->setVPS( pcEncPic->slices[0]->getVPS() );
                    pcEncPic->slices.back()->setPic( pcEncPic->slices[0]->getPic() );
                  }
                  pcEncPic->slices[i]->copySliceInfo( pic->slices[i], false );
                }
                }

                pcEncPic->cs->slice = pcEncPic->slices.back();

                if( debugCTU >= 0 && poc == debugPOC )
                {
                  pcEncPic->cs->initStructData();

                  pcEncPic->cs->copyStructure( *pic->cs, CH_L, true, true );

                  if( CS::isDualITree( *pcEncPic->cs ) )
                  {
                    pcEncPic->cs->copyStructure( *pic->cs, CH_C, true, true );
                  }

                  for( auto &cu : pcEncPic->cs->cus )
                  {
                    cu->slice = pcEncPic->cs->slice;
                  }
                }
                else
                {
                if ( pic->cs->sps->getSAOEnabledFlag() )
                {
                  pcEncPic->copySAO( *pic, 0 );
                }

                if( pic->cs->sps->getALFEnabledFlag() )
                {
                  std::copy(pic->getAlfCtbFilterIndexVec().begin(), pic->getAlfCtbFilterIndexVec().end(), pcEncPic->getAlfCtbFilterIndexVec().begin());
                  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
                  {
                    std::copy( pic->getAlfCtuEnableFlag()[compIdx].begin(), pic->getAlfCtuEnableFlag()[compIdx].end(), pcEncPic->getAlfCtuEnableFlag()[compIdx].begin() );
                  }
                  pcEncPic->resizeAlfCtbFilterIndex(pic->cs->pcv->sizeInCtus);
                  memcpy( pcEncPic->getAlfCtbFilterIndex(), pic->getAlfCtbFilterIndex(), sizeof(short)*pic->cs->pcv->sizeInCtus );

                  std::copy( pic->getAlfCtuAlternative(COMPONENT_Cb).begin(), pic->getAlfCtuAlternative(COMPONENT_Cb).end(), pcEncPic->getAlfCtuAlternative(COMPONENT_Cb).begin() );
                  std::copy( pic->getAlfCtuAlternative(COMPONENT_Cr).begin(), pic->getAlfCtuAlternative(COMPONENT_Cr).end(), pcEncPic->getAlfCtuAlternative(COMPONENT_Cr).begin() );

                  for( int i = 0; i < pic->slices.size(); i++ )
                  {
                    pcEncPic->slices[i]->setTileGroupNumAps(pic->slices[i]->getTileGroupNumAps());
                    pcEncPic->slices[i]->setAlfAPSs(pic->slices[i]->getTileGroupApsIdLuma());
                    pcEncPic->slices[i]->setAlfAPSs(pic->slices[i]->getAlfAPSs());
                    pcEncPic->slices[i]->setTileGroupApsIdChroma(pic->slices[i]->getTileGroupApsIdChroma());
                    pcEncPic->slices[i]->setTileGroupAlfEnabledFlag(COMPONENT_Y,  pic->slices[i]->getTileGroupAlfEnabledFlag(COMPONENT_Y));
                    pcEncPic->slices[i]->setTileGroupAlfEnabledFlag(COMPONENT_Cb, pic->slices[i]->getTileGroupAlfEnabledFlag(COMPONENT_Cb));
                    pcEncPic->slices[i]->setTileGroupAlfEnabledFlag(COMPONENT_Cr, pic->slices[i]->getTileGroupAlfEnabledFlag(COMPONENT_Cr));
#if JVET_Q0795_CCALF
                    pcEncPic->slices[i]->setTileGroupCcAlfCbApsId(pic->slices[i]->getTileGroupCcAlfCbApsId());
                    pcEncPic->slices[i]->setTileGroupCcAlfCbEnabledFlag(pic->slices[i]->getTileGroupCcAlfCbEnabledFlag());
                    pcEncPic->slices[i]->setTileGroupCcAlfCrApsId(pic->slices[i]->getTileGroupCcAlfCrApsId());
                    pcEncPic->slices[i]->setTileGroupCcAlfCrEnabledFlag(pic->slices[i]->getTileGroupCcAlfCrEnabledFlag());
#endif
                  }
                }

                pcDecLib->executeLoopFilters();
                if ( pic->cs->sps->getSAOEnabledFlag() )
                {
                  pcEncPic->copySAO( *pic, 1 );
                }

                pcEncPic->cs->copyStructure( *pic->cs, CH_L, true, true );

                if( CS::isDualITree( *pcEncPic->cs ) )
                {
                  pcEncPic->cs->copyStructure( *pic->cs, CH_C, true, true );
                }
                }
                goOn = false; // exit the loop return
                bRet = true;
                break;
              }
            }
          }
          // postpone loop filters
          if (!bRet)
          {
            pcDecLib->executeLoopFilters();
          }

          pcDecLib->finishPicture( poc, pcListPic, DETAILS );

          // write output
          if( ! pcListPic->empty())
          {
            PicList::iterator iterPic   = pcListPic->begin();
            int numPicsNotYetDisplayed = 0;
            int dpbFullness = 0;
            const SPS* activeSPS = (pcListPic->front()->cs->sps);
            uint32_t maxNrSublayers = activeSPS->getMaxTLayers();
            uint32_t numReorderPicsHighestTid = activeSPS->getNumReorderPics(maxNrSublayers-1);
            uint32_t maxDecPicBufferingHighestTid =  activeSPS->getMaxDecPicBuffering(maxNrSublayers-1);
#if JVET_Q0814_DPB
            const VPS* referredVPS = pcListPic->front()->cs->vps;

            if( referredVPS != nullptr && referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] > 1 )
            {
              numReorderPicsHighestTid = referredVPS->getNumReorderPics( maxNrSublayers - 1 );
              maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering( maxNrSublayers - 1 );
            }
#endif

            while (iterPic != pcListPic->end())
            {
              Picture* pcCurPic = *(iterPic);
              if(pcCurPic->neededForOutput && pcCurPic->getPOC() > iPOCLastDisplay)
              {
                numPicsNotYetDisplayed++;
                dpbFullness++;
              }
              else if(pcCurPic->referenced)
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

            Picture* pcCurPic = *(iterPic);
            if( numPicsNotYetDisplayed>2 && pcCurPic->fieldPic ) //Field Decoding
            {
              THROW( "no field coding support ");
            }
            else if( !pcCurPic->fieldPic ) //Frame Decoding
            {
              iterPic = pcListPic->begin();

              while (iterPic != pcListPic->end())
              {
                pcCurPic = *(iterPic);

                if(pcCurPic->neededForOutput && pcCurPic->getPOC() > iPOCLastDisplay &&
                  (numPicsNotYetDisplayed >  numReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
                {
                    numPicsNotYetDisplayed--;
                  if( ! pcCurPic->referenced )
                  {
                    dpbFullness--;
                  }
                  // update POC of display order
                  iPOCLastDisplay = pcCurPic->getPOC();

                  // erase non-referenced picture in the reference picture list after display
                  if( ! pcCurPic->referenced && pcCurPic->reconstructed )
                  {
                    pcCurPic->reconstructed = false;
                  }
                  pcCurPic->neededForOutput = false;
                }

                iterPic++;
              }
            }
          }


        }
#if JVET_P0125_EOS_LAYER_SPECIFIC
        loopFiltered[nalu.m_nuhLayerId] = (nalu.m_nalUnitType == NAL_UNIT_EOS);
#else
        loopFiltered = ( nalu.m_nalUnitType == NAL_UNIT_EOS );
#endif
        if( nalu.m_nalUnitType == NAL_UNIT_EOS )
        {
#if JVET_P0125_EOS_LAYER_SPECIFIC
          pcDecLib->setFirstSliceInSequence(true, nalu.m_nuhLayerId);
#else
          pcDecLib->setFirstSliceInSequence( true );
#endif
        }

      }
#if JVET_P0125_EOS_LAYER_SPECIFIC
      else if ((bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) && pcDecLib->getFirstSliceInSequence(nalu.m_nuhLayerId))
#else
      else if( ( bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) && pcDecLib->getFirstSliceInSequence() )
#endif
      {
        pcDecLib->setFirstSliceInPicture( true );
      }
    }
  }

  if( !bRet )
  {
    CHECK( bDecodeUntilPocFound, " decoding failed - check decodeBitstream2 parameter File: " << bitstreamFileName.c_str() );
    if( pcDecLib )
    {
      pcDecLib->destroy();
      pcDecLib->deletePicBuffer();
      delete pcDecLib;
      pcDecLib = nullptr;
    }
    bFirstCall   = true;
#if JVET_P0125_EOS_LAYER_SPECIFIC
    for (int i = 0; i < MAX_VPS_LAYERS; i++)
    {
      loopFiltered[i] = false;
    }
#else
    loopFiltered = false;
#endif
    iPOCLastDisplay = -MAX_INT;

    if( bytestream )
    {
      delete bytestream;
      bytestream = nullptr;
    }

    if( bitstreamFile )
    {
      delete bitstreamFile;
      bitstreamFile = nullptr;
    }
  }

  return bRet;
}


//! \ingroup DecoderLib
//! \{

DecLib::DecLib()
  : m_iMaxRefPicNum(0)
  , m_associatedIRAPType(NAL_UNIT_INVALID)
  , m_pocCRA(0)
  , m_pocRandomAccess(MAX_INT)
  , m_lastRasPoc(MAX_INT)
  , m_cListPic()
  , m_parameterSetManager()
  , m_apcSlicePilot(NULL)
  , m_SEIs()
  , m_cIntraPred()
  , m_cInterPred()
  , m_cTrQuant()
  , m_cSliceDecoder()
  , m_cTrQuantScalingList()
  , m_cCuDecoder()
  , m_HLSReader()
  , m_seiReader()
  , m_cLoopFilter()
  , m_cSAO()
  , m_cReshaper()
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
  , m_pcPic(NULL)
  , m_prevLayerID(MAX_INT)
  , m_prevPOC(MAX_INT)
  , m_prevTid0POC(0)
  , m_bFirstSliceInPicture(true)
#if JVET_P0125_EOS_LAYER_SPECIFIC
  , m_bFirstSliceInSequence{ true }
#else
  , m_bFirstSliceInSequence(true)
#endif
  , m_prevSliceSkipped(false)
  , m_skippedPOC(0)
  , m_bFirstSliceInBitstream(true)
  , m_lastPOCNoOutputPriorPics(-1)
  , m_isNoOutputPriorPics(false)
  , m_lastNoIncorrectPicOutputFlag(false)
  , m_sliceLmcsApsId(-1)
  , m_pDecodedSEIOutputStream(NULL)
  , m_decodedPictureHashSEIEnabled(false)
  , m_numberOfChecksumErrorsDetected(0)
  , m_warningMessageSkipPicture(false)
  , m_prefixSEINALUs()
  , m_debugPOC( -1 )
  , m_debugCTU( -1 )
  , m_vps( nullptr )
  , m_scalingListUpdateFlag(true)
  , m_PreScalingListAPSId(-1)

#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  , m_maxDecSubPicIdx(0)
  , m_maxDecSliceAddrInSubPic(-1)
#endif
#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  , m_dci(NULL)
#endif
{
#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif
}

DecLib::~DecLib()
{
  while (!m_prefixSEINALUs.empty())
  {
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }

}

void DecLib::create()
{
  m_apcSlicePilot = new Slice;
  m_uiSliceSegmentIdx = 0;
}

void DecLib::destroy()
{
  delete m_apcSlicePilot;
  m_apcSlicePilot = NULL;

#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  if( m_dci )
  {
    delete m_dci;
    m_dci = NULL;
  }
#endif

  m_cSliceDecoder.destroy();
}

void DecLib::init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  const std::string& cacheCfgFileName
#endif
)
{
  m_cSliceDecoder.init( &m_CABACDecoder, &m_cCuDecoder );
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cacheModel.create( cacheCfgFileName );
  m_cacheModel.clear( );
  m_cInterPred.cacheAssign( &m_cacheModel );
#endif
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
}

void DecLib::deletePicBuffer ( )
{
  PicList::iterator  iterPic   = m_cListPic.begin();
  int iSize = int( m_cListPic.size() );

  for (int i = 0; i < iSize; i++ )
  {
    Picture* pcPic = *(iterPic++);
    pcPic->destroy();

    delete pcPic;
    pcPic = NULL;
  }
  m_cALF.destroy();
  m_cSAO.destroy();
  m_cLoopFilter.destroy();
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cacheModel.reportSequence( );
  m_cacheModel.destroy( );
#endif
  m_cCuDecoder.destoryDecCuReshaprBuf();
  m_cReshaper.destroy();
}

Picture* DecLib::xGetNewPicBuffer( const SPS &sps, const PPS &pps, const uint32_t temporalLayer, const int layerId )
{
  Picture * pcPic = nullptr;
#if JVET_Q0814_DPB
  m_iMaxRefPicNum = ( m_vps == nullptr || m_vps->m_numLayersInOls[m_vps->m_targetOlsIdx] == 1 ) ? sps.getMaxDecPicBuffering( temporalLayer ) : m_vps->getMaxDecPicBuffering( temporalLayer );     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#else
  m_iMaxRefPicNum = sps.getMaxDecPicBuffering(temporalLayer);     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#endif
  if (m_cListPic.size() < (uint32_t)m_iMaxRefPicNum)
  {
    pcPic = new Picture();

    pcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true, layerId );

    m_cListPic.push_back( pcPic );

    return pcPic;
  }

  bool bBufferIsAvailable = false;
  for(auto * p: m_cListPic)
  {
    pcPic = p;  // workaround because range-based for-loops don't work with existing variables
    if ( pcPic->reconstructed == false && ! pcPic->neededForOutput )
    {
      pcPic->neededForOutput = false;
      bBufferIsAvailable = true;
      break;
    }

    if( ! pcPic->referenced  && ! pcPic->neededForOutput )
    {
      pcPic->neededForOutput = false;
      pcPic->reconstructed = false;
      bBufferIsAvailable = true;
      break;
    }
  }

  if( ! bBufferIsAvailable )
  {
    //There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    m_iMaxRefPicNum++;

    pcPic = new Picture();

    m_cListPic.push_back( pcPic );

    pcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true, layerId );
  }
  else
  {
    if( !pcPic->Y().Size::operator==( Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ) ) || pps.pcv->maxCUWidth != sps.getMaxCUWidth() || pps.pcv->maxCUHeight != sps.getMaxCUHeight() )
    {
      pcPic->destroy();
      pcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, true, layerId );
    }
  }

  pcPic->setBorderExtension( false );
  pcPic->neededForOutput = false;
  pcPic->reconstructed = false;

  return pcPic;
}


void DecLib::executeLoopFilters()
{
  if( !m_pcPic )
  {
    return; // nothing to deblock
  }

  m_pcPic->cs->slice->startProcessingTimer();

  CodingStructure& cs = *m_pcPic->cs;

  if (cs.sps->getUseLmcs() && m_cReshaper.getSliceReshaperInfo().getUseSliceReshaper())
  {
      CHECK((m_cReshaper.getRecReshaped() == false), "Rec picture is not reshaped!");
      m_pcPic->getRecoBuf(COMPONENT_Y).rspSignal(m_cReshaper.getInvLUT());
      m_cReshaper.setRecReshaped(false);
      m_cSAO.setReshaper(&m_cReshaper);
  }
  // deblocking filter
  m_cLoopFilter.loopFilterPic( cs );
  CS::setRefinedMotionField(cs);
  if( cs.sps->getSAOEnabledFlag() )
  {
    m_cSAO.SAOProcess( cs, cs.picture->getSAO() );
  }

  if( cs.sps->getALFEnabledFlag() )
  {
#if JVET_Q0795_CCALF
    m_cALF.getCcAlfFilterParam() = cs.slice->m_ccAlfFilterParam;
#endif
    // ALF decodes the differentially coded coefficients and stores them in the parameters structure.
    // Code could be restructured to do directly after parsing. So far we just pass a fresh non-const
    // copy in case the APS gets used more than once.
    m_cALF.ALFProcess(cs);
  }

#if JVET_O1143_SUBPIC_BOUNDARY
  for (int i = 0; i < cs.pps->getNumSubPics() && m_targetSubPicIdx; i++)
  {
    // keep target subpic samples untouched, for other subpics mask their output sample value to 0
    int targetSubPicIdx = m_targetSubPicIdx - 1;
    if (i != targetSubPicIdx)
    {
      SubPic SubPicNoUse = cs.pps->getSubPics()[i];
      uint32_t left  = SubPicNoUse.getSubPicLeft();
      uint32_t right = SubPicNoUse.getSubPicRight();
      uint32_t top   = SubPicNoUse.getSubPicTop();
      uint32_t bottom= SubPicNoUse.getSubPicBottom();
      for (uint32_t row = top; row <= bottom; row++)
      {
        for (uint32_t col = left; col <= right; col++)
        {
          cs.getRecoBuf().Y().at(col, row) = 0;
          // for test only, hard coding using 4:2:0 chroma format
          cs.getRecoBuf().Cb().at(col>>1, row>>1) = 0;
          cs.getRecoBuf().Cr().at(col>>1, row>>1) = 0;
        }
      }
    } 
  }
#endif

  m_pcPic->cs->slice->stopProcessingTimer();
}

void DecLib::finishPictureLight(int& poc, PicList*& rpcListPic )
{
  Slice*  pcSlice = m_pcPic->cs->slice;

  m_pcPic->neededForOutput = (pcSlice->getPicHeader()->getPicOutputFlag() ? true : false);
  m_pcPic->reconstructed = true;

  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcSlice->getPOC();
  rpcListPic          = &m_cListPic;
}

void DecLib::finishPicture(int& poc, PicList*& rpcListPic, MsgLevel msgl )
{
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  CodingStatistics::StatTool& s = CodingStatistics::GetStatisticTool( STATS__TOOL_TOTAL_FRAME );
  s.count++;
  s.pixels = s.count * m_pcPic->Y().width * m_pcPic->Y().height;
#endif

  Slice*  pcSlice = m_pcPic->cs->slice;

  char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!m_pcPic->referenced)
  {
    c += 32;  // tolower
  }

  if (pcSlice->isDRAP()) c = 'D';

  //-- For time output for each slice
  msg( msgl, "POC %4d LId: %2d TId: %1d ( %c-SLICE, QP%3d ) ", pcSlice->getPOC(), pcSlice->getPic()->layerId,
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp() );
  msg( msgl, "[DT %6.3f] ", pcSlice->getProcessingTime() );

  for (int iRefList = 0; iRefList < 2; iRefList++)
  {
    msg( msgl, "[L%d", iRefList);
    for (int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      const std::pair<int, int>& scaleRatio = pcSlice->getScalingRatio( RefPicList( iRefList ), iRefIndex );

      if( pcSlice->getPicHeader()->getEnableTMVPFlag() && pcSlice->getColFromL0Flag() == bool(1 - iRefList) && pcSlice->getColRefIdx() == iRefIndex )
      {
        if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
        {
          msg( msgl, " %dc(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
        }
        else
        {
          msg( msgl, " %dc", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) );
        }
      }
      else
      {
        if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
        {
          msg( msgl, " %d(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
        }
        else
        {
          msg( msgl, " %d", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) );
        }
      }

      if( pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) == pcSlice->getPOC() )
      {
        msg( msgl, ".%d", pcSlice->getRefPic( RefPicList( iRefList ), iRefIndex )->layerId );
      }   
    }
    msg( msgl, "] ");
  }
  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages pictureHashes = getSeisByType(m_pcPic->SEIs, SEI::DECODED_PICTURE_HASH );
    const SEIDecodedPictureHash *hash = ( pictureHashes.size() > 0 ) ? (SEIDecodedPictureHash*) *(pictureHashes.begin()) : NULL;
    if (pictureHashes.size() > 1)
    {
      msg( WARNING, "Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }
    m_numberOfChecksumErrorsDetected += calcAndPrintHashStatus(((const Picture*) m_pcPic)->getRecoBuf(), hash, pcSlice->getSPS()->getBitDepths(), msgl);
  }

  msg( msgl, "\n");

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    m_cacheModel.reportFrame();
    m_cacheModel.accumulateFrame();
    m_cacheModel.clear();
#endif

  m_pcPic->neededForOutput = (pcSlice->getPicHeader()->getPicOutputFlag() ? true : false);
  m_pcPic->reconstructed = true;


  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcSlice->getPOC();
  rpcListPic          = &m_cListPic;
  m_bFirstSliceInPicture  = true; // TODO: immer true? hier ist irgendwas faul
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  m_maxDecSubPicIdx = 0;
  m_maxDecSliceAddrInSubPic = -1;
#endif

  m_pcPic->destroyTempBuffers();
  m_pcPic->cs->destroyCoeffs();
  m_pcPic->cs->releaseIntermediateData();
  m_pcPic->cs->picHeader->initPicHeader();
}

void DecLib::checkNoOutputPriorPics (PicList* pcListPic)
{
  if (!pcListPic || !m_isNoOutputPriorPics)
  {
    return;
  }

  PicList::iterator  iterPic   = pcListPic->begin();

  while (iterPic != pcListPic->end())
  {
    Picture* pcPicTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != pcPicTmp->getPOC())
    {
      pcPicTmp->neededForOutput = false;
    }
  }
}

void DecLib::xUpdateRasInit(Slice* slice)
{
  slice->setPendingRasInit( false );
  if ( slice->getPOC() > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->setPendingRasInit( true );
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->getPOC();
  }
}

void DecLib::xCreateLostPicture( int iLostPoc, const int layerId )
{
  msg( INFO, "\ninserting lost poc : %d\n",iLostPoc);
  Picture *cFillPic = xGetNewPicBuffer( *( m_parameterSetManager.getFirstSPS() ), *( m_parameterSetManager.getFirstPPS() ), 0, layerId );

  CHECK( !cFillPic->slices.size(), "No slices in picture" );

  cFillPic->slices[0]->initSlice();

  PicList::iterator iterPic = m_cListPic.begin();
  int closestPoc = 1000000;
  while ( iterPic != m_cListPic.end())
  {
    Picture * rpcPic = *(iterPic++);
    if(abs(rpcPic->getPOC() -iLostPoc)<closestPoc&&abs(rpcPic->getPOC() -iLostPoc)!=0&&rpcPic->getPOC()!=m_apcSlicePilot->getPOC())
    {
      closestPoc=abs(rpcPic->getPOC() -iLostPoc);
    }
  }
  iterPic = m_cListPic.begin();
  while ( iterPic != m_cListPic.end())
  {
    Picture *rpcPic = *(iterPic++);
    if(abs(rpcPic->getPOC() -iLostPoc)==closestPoc&&rpcPic->getPOC()!=m_apcSlicePilot->getPOC())
    {
      msg( INFO, "copying picture %d to %d (%d)\n",rpcPic->getPOC() ,iLostPoc,m_apcSlicePilot->getPOC());
      cFillPic->getRecoBuf().copyFrom( rpcPic->getRecoBuf() );
      break;
    }
  }

//  for(int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)  { cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr); }
  cFillPic->referenced = true;
  cFillPic->slices[0]->setPOC(iLostPoc);
  xUpdatePreviousTid0POC(cFillPic->slices[0]);
  cFillPic->reconstructed = true;
  cFillPic->neededForOutput = true;
  if(m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iLostPoc;
  }

}

void DecLib::xCreateUnavailablePicture(int iUnavailablePoc, bool longTermFlag, const int layerId, const bool interLayerRefPicFlag)
{
  msg(INFO, "\ninserting unavailable poc : %d\n", iUnavailablePoc);
  Picture* cFillPic = xGetNewPicBuffer( *( m_parameterSetManager.getFirstSPS() ), *( m_parameterSetManager.getFirstPPS() ), 0, layerId );

  CHECK(!cFillPic->slices.size(), "No slices in picture");

  cFillPic->slices[0]->initSlice();

  uint32_t yFill = 1 << (m_parameterSetManager.getFirstSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  uint32_t cFill = 1 << (m_parameterSetManager.getFirstSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
  cFillPic->getRecoBuf().Y().fill(yFill);
  cFillPic->getRecoBuf().Cb().fill(cFill);
  cFillPic->getRecoBuf().Cr().fill(cFill);

  //  for(int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)  { cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr); }
  cFillPic->referenced = true;
  cFillPic->interLayerRefPicFlag = interLayerRefPicFlag;
  cFillPic->longTerm = longTermFlag;
  cFillPic->slices[0]->setPOC(iUnavailablePoc);
  xUpdatePreviousTid0POC(cFillPic->slices[0]);
  cFillPic->reconstructed = true;
  cFillPic->neededForOutput = false;
  if (m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iUnavailablePoc;
  }

}
#if JVET_P0125_ASPECT_TID_LAYER_ID_NUH
void DecLib::checkTidLayerIdInAccessUnit()
{
  int firstPicTid = m_accessUnitPicInfo.begin()->m_temporalId;
  int firstPicLayerId = m_accessUnitPicInfo.begin()->m_nuhLayerId;

  bool isPicTidInAuSame = true;
  bool isSeiTidInAuSameAsAuTid = true;
  bool isFdNaluLayerIdSameAsVclNaluLayerId = true;

  for (auto pic = m_accessUnitPicInfo.begin(); pic != m_accessUnitPicInfo.end(); pic++)
  {
    if (pic->m_temporalId != firstPicTid)
    {
      isPicTidInAuSame = false;
      break;
    }
  }
  CHECK(!isPicTidInAuSame, "All pictures in an AU shall have the same value of TemporalId");

  for (auto tid = m_accessUnitSeiTids.begin(); tid != m_accessUnitSeiTids.end(); tid++)
  {
    if ((*tid) != firstPicTid)
    {
      isSeiTidInAuSameAsAuTid = false;
      break;
    }
  }
  CHECK(!isSeiTidInAuSameAsAuTid, "The TemporalId of an SEI NAL unit shall be equal to the TemporalId of the AU containing the NAL unit");

  for (auto tempNalu = m_accessUnitNals.begin(); tempNalu != m_accessUnitNals.end(); tempNalu++)
  {
    if ((tempNalu->first == NAL_UNIT_FD) && (tempNalu->second != firstPicLayerId))
    {
      isFdNaluLayerIdSameAsVclNaluLayerId = false;
      break;
    }
  }
  CHECK(!isFdNaluLayerIdSameAsVclNaluLayerId, "The nuh_layer_id of a filler data NAL unit shall be equal to the nuh_layer_id of associated VCL NAL unit");

}
#endif
/**
 - Determine if the first VCL NAL unit of a picture is also the first VCL NAL of an Access Unit
 */
bool DecLib::isSliceNaluFirstInAU( bool newPicture, InputNALUnit &nalu )
{
  // can only be the start of an AU if this is the start of a new picture
  if( newPicture == false )
  {
    return false;
  }

  // should only be called for slice NALU types
  if( nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_TRAIL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_STSA &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RADL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_IDR_W_RADL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_IDR_N_LP &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_CRA &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_GDR )
  {
    return false;
  }

#if !JVET_Q0819_PH_CHANGES
  // check for valid picture header
  if(m_picHeader.isValid() == false)
  {
    return false;
  }
#endif
  
  // check for layer ID less than or equal to previous picture's layer ID
  if( nalu.m_nuhLayerId <= m_prevLayerID )
  {
    return true;
  }

  // get slice POC
  m_apcSlicePilot->setPicHeader( &m_picHeader );
  m_apcSlicePilot->initSlice(); 
  m_HLSReader.setBitstream( &nalu.getBitstream() );
#if JVET_Q0819_PH_CHANGES
  m_HLSReader.getSlicePoc( m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC );
#else
  m_HLSReader.parseSliceHeaderToPoc( m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC );
#endif

  // check for different POC
  return (m_apcSlicePilot->getPOC() != m_prevPOC);
}

void activateAPS(PicHeader* picHeader, Slice* pSlice, ParameterSetManager& parameterSetManager, APS** apss, APS* lmcsAPS, APS* scalingListAPS)
{
  //luma APSs
  if (pSlice->getTileGroupAlfEnabledFlag(COMPONENT_Y))
  {
    for (int i = 0; i < pSlice->getTileGroupApsIdLuma().size(); i++)
    {
      int apsId = pSlice->getTileGroupApsIdLuma()[i];
      APS* aps = parameterSetManager.getAPS(apsId, ALF_APS);

      if (aps)
      {
        apss[apsId] = aps;
        if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
        {
          THROW("APS activation failed!");
        }

        CHECK( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
        //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
      }
    }
  }
  if (pSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cb)||pSlice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) )
  {
    //chroma APS
    int apsId = pSlice->getTileGroupApsIdChroma();
    APS* aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if (aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }

#if JVET_Q0795_CCALF
  CcAlfFilterParam &filterParam = pSlice->m_ccAlfFilterParam;
  // cleanup before copying
  for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
  {
    memset( filterParam.ccAlfCoeff[COMPONENT_Cb - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMPONENT_Cb - 1][filterIdx]) );
    memset( filterParam.ccAlfCoeff[COMPONENT_Cr - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMPONENT_Cr - 1][filterIdx]) );
  }
  memset( filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cb - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cb - 1]) );
  memset( filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cr - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cr - 1]) );

  if(pSlice->getTileGroupCcAlfCbEnabledFlag())
  {
    int apsId = pSlice->getTileGroupCcAlfCbApsId();
    APS *aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if(aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

      filterParam.ccAlfFilterCount[COMPONENT_Cb - 1] = aps->getCcAlfAPSParam().ccAlfFilterCount[COMPONENT_Cb - 1];
      for (int filterIdx=0; filterIdx < filterParam.ccAlfFilterCount[COMPONENT_Cb - 1]; filterIdx++ )
      {
        filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cb - 1][filterIdx] = aps->getCcAlfAPSParam().ccAlfFilterIdxEnabled[COMPONENT_Cb - 1][filterIdx];
        memcpy(filterParam.ccAlfCoeff[COMPONENT_Cb - 1][filterIdx], aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cb - 1][filterIdx], sizeof(aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cb - 1][filterIdx]));
      }
    }
  }

  if(pSlice->getTileGroupCcAlfCrEnabledFlag())
  {
    int apsId = pSlice->getTileGroupCcAlfCrApsId();
    APS *aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if(aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK( aps->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

      filterParam.ccAlfFilterCount[COMPONENT_Cr - 1] = aps->getCcAlfAPSParam().ccAlfFilterCount[COMPONENT_Cr - 1];
      for (int filterIdx=0; filterIdx < filterParam.ccAlfFilterCount[COMPONENT_Cr - 1]; filterIdx++ )
      {
        filterParam.ccAlfFilterIdxEnabled[COMPONENT_Cr - 1][filterIdx] = aps->getCcAlfAPSParam().ccAlfFilterIdxEnabled[COMPONENT_Cr - 1][filterIdx];
        memcpy(filterParam.ccAlfCoeff[COMPONENT_Cr - 1][filterIdx], aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cr - 1][filterIdx], sizeof(aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cr - 1][filterIdx]));
      }
    }
  }
#endif

  if (picHeader->getLmcsEnabledFlag() && lmcsAPS == nullptr)
  {
    lmcsAPS = parameterSetManager.getAPS(picHeader->getLmcsAPSId(), LMCS_APS);
    CHECK(lmcsAPS == nullptr, "No LMCS APS present");
    if (lmcsAPS)
    {
      parameterSetManager.clearAPSChangedFlag(picHeader->getLmcsAPSId(), LMCS_APS);
      if (false == parameterSetManager.activateAPS(picHeader->getLmcsAPSId(), LMCS_APS))
      {
        THROW("LMCS APS activation failed!");
      }

      CHECK( lmcsAPS->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->setLmcsAPS(lmcsAPS);

  if( picHeader->getScalingListPresentFlag() && scalingListAPS == nullptr)
  {
    scalingListAPS = parameterSetManager.getAPS( picHeader->getScalingListAPSId(), SCALING_LIST_APS );
    CHECK( scalingListAPS == nullptr, "No SCALING LIST APS present" );
    if( scalingListAPS )
    {
      parameterSetManager.clearAPSChangedFlag( picHeader->getScalingListAPSId(), SCALING_LIST_APS );
      if( false == parameterSetManager.activateAPS( picHeader->getScalingListAPSId(), SCALING_LIST_APS ) )
      {
        THROW( "SCALING LIST APS activation failed!" );
      }

      CHECK( scalingListAPS->getTemporalId() > pSlice->getTLayer(), "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->setScalingListAPS(scalingListAPS);
}

void DecLib::xActivateParameterSets( const int layerId )
{
  if (m_bFirstSliceInPicture)
  {
    APS** apss = m_parameterSetManager.getAPSs();
    memset(apss, 0, sizeof(*apss) * ALF_CTB_MAX_NUM_APS);
    const PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId()); // this is a temporary PPS object. Do not store this value
    CHECK(pps == 0, "No PPS present");

    const SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());             // this is a temporary SPS object. Do not store this value
    CHECK(sps == 0, "No SPS present");

    const VPS *vps = sps->getVPSId() ? m_parameterSetManager.getVPS( sps->getVPSId() ) : nullptr;

    if (NULL == pps->pcv)
    {
      m_parameterSetManager.getPPS( m_picHeader.getPPSId() )->pcv = new PreCalcValues( *sps, *pps, false );
    }
    m_parameterSetManager.clearSPSChangedFlag(sps->getSPSId());
    m_parameterSetManager.clearPPSChangedFlag(pps->getPPSId());

    if (false == m_parameterSetManager.activatePPS(m_picHeader.getPPSId(),m_apcSlicePilot->isIRAP()))
    {
      THROW("Parameter set activation failed!");
    }
    
#if JVET_O1143_SUBPIC_BOUNDARY && !JVET_Q0044_SLICE_IDX_WITH_SUBPICS
    PPS* nonconstPPS = m_parameterSetManager.getPPS(m_picHeader.getPPSId());
    nonconstPPS->initSubPic(*sps);
#endif

    m_parameterSetManager.getApsMap()->clear();
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* aps = m_parameterSetManager.getAPS(i, ALF_APS);
      if (aps)
      {
        m_parameterSetManager.clearAPSChangedFlag(i, ALF_APS);
      }
    }
    APS* lmcsAPS = nullptr;
    APS* scalinglistAPS = nullptr;
    activateAPS(&m_picHeader, m_apcSlicePilot, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    xParsePrefixSEImessages();

#if RExt__HIGH_BIT_DEPTH_SUPPORT==0
    if (sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() || sps->getBitDepth(CHANNEL_TYPE_LUMA)>12 || sps->getBitDepth(CHANNEL_TYPE_CHROMA)>12 )
    {
      THROW("High bit depth support must be enabled at compile-time in order to decode this bitstream\n");
    }
#endif

    //  Get a new picture buffer. This will also set up m_pcPic, and therefore give us a SPS and PPS pointer that we can use.
    m_pcPic = xGetNewPicBuffer( *sps, *pps, m_apcSlicePilot->getTLayer(), layerId );

    m_apcSlicePilot->applyReferencePictureListBasedMarking( m_cListPic, m_apcSlicePilot->getRPL0(), m_apcSlicePilot->getRPL1(), layerId );
    m_pcPic->finalInit( vps, *sps, *pps, &m_picHeader, apss, lmcsAPS, scalinglistAPS );
    m_pcPic->createTempBuffers( m_pcPic->cs->pps->pcv->maxCUWidth );
    m_pcPic->cs->createCoeffs((bool)m_pcPic->cs->sps->getPLTMode());

    m_pcPic->allocateNewSlice();
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    CHECK(m_pcPic->slices.size() != (m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pcPic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    // we now have a real slice:
    Slice *pSlice = m_pcPic->slices[m_uiSliceSegmentIdx];

    // Update the PPS and SPS pointers with the ones of the picture.
    pps=pSlice->getPPS();
    sps=pSlice->getSPS();

    // fix Parameter Sets, now that we have the real slice
    m_pcPic->cs->slice = pSlice;
    m_pcPic->cs->sps   = sps;
    m_pcPic->cs->pps   = pps;
    m_pcPic->cs->vps = vps;

    memcpy(m_pcPic->cs->alfApss, apss, sizeof(m_pcPic->cs->alfApss));
    m_pcPic->cs->lmcsAps = lmcsAPS;
    m_pcPic->cs->scalinglistAps = scalinglistAPS;

    m_pcPic->cs->pcv   = pps->pcv;

    // Initialise the various objects for the new set of settings
#if JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
    const int maxDepth = floorLog2(sps->getMaxCUWidth()) - pps->pcv->minCUWidthLog2;
#if JVET_Q0441_SAO_MOD_12_BIT
    const uint32_t  log2SaoOffsetScaleLuma   = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_LUMA  ) - MAX_SAO_TRUNCATED_BITDEPTH);
    const uint32_t  log2SaoOffsetScaleChroma = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_CHROMA) - MAX_SAO_TRUNCATED_BITDEPTH);
    m_cSAO.create( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(),
                   sps->getChromaFormatIdc(),
                   sps->getMaxCUWidth(), sps->getMaxCUHeight(),
                   maxDepth,
                   log2SaoOffsetScaleLuma, log2SaoOffsetScaleChroma );
#else
    m_cSAO.create(pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), maxDepth, pps->getPpsRangeExtension().getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA), pps->getPpsRangeExtension().getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA));
#endif
    m_cLoopFilter.create(maxDepth);
#else
#if JVET_Q0441_SAO_MOD_12_BIT
    const uint32_t  log2SaoOffsetScaleLuma   = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_LUMA  ) - MAX_SAO_TRUNCATED_BITDEPTH);
    const uint32_t  log2SaoOffsetScaleChroma = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_CHROMA) - MAX_SAO_TRUNCATED_BITDEPTH);
    m_cSAO.create( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(),
                   sps->getChromaFormatIdc(),
                   sps->getMaxCUWidth(), sps->getMaxCUHeight(),
                   sps->getMaxCodingDepth(),
                   log2SaoOffsetScaleLuma, log2SaoOffsetScaleChroma );
#else
    m_cSAO.create( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCodingDepth(), pps->getPpsRangeExtension().getLog2SaoOffsetScale( CHANNEL_TYPE_LUMA ), pps->getPpsRangeExtension().getLog2SaoOffsetScale( CHANNEL_TYPE_CHROMA ) );
#endif
    m_cLoopFilter.create( sps->getMaxCodingDepth() );
#endif
    m_cIntraPred.init( sps->getChromaFormatIdc(), sps->getBitDepth( CHANNEL_TYPE_LUMA ) );
    m_cInterPred.init( &m_cRdCost, sps->getChromaFormatIdc(), sps->getMaxCUHeight() );
    if (sps->getUseLmcs())
    {
      m_cReshaper.createDec(sps->getBitDepth(CHANNEL_TYPE_LUMA));
    }

    bool isField = false;
    bool isTopField = false;

    if(!m_SEIs.empty())
    {
      // Check if any new Frame Field Info SEI has arrived
      SEIMessages frameFieldSEIs = getSeisByType(m_SEIs, SEI::FRAME_FIELD_INFO);
      if (frameFieldSEIs.size()>0)
      {
        SEIFrameFieldInfo* ff = (SEIFrameFieldInfo*) *(frameFieldSEIs.begin());
        isField    = ff->m_fieldPicFlag;
        isTopField = isField && (!ff->m_bottomFieldFlag);
      }
    }

    //Set Field/Frame coding mode
    m_pcPic->fieldPic = isField;
    m_pcPic->topField = isTopField;

    // transfer any SEI messages that have been received to the picture
    m_pcPic->SEIs = m_SEIs;
    m_SEIs.clear();

    // Recursive structure
    m_cCuDecoder.init( &m_cTrQuant, &m_cIntraPred, &m_cInterPred );
    if (sps->getUseLmcs())
    {
      m_cCuDecoder.initDecCuReshaper(&m_cReshaper, sps->getChromaFormatIdc());
    }
    m_cTrQuant.init(m_cTrQuantScalingList.getQuant(), sps->getMaxTbSize(), false, false, false, false);

    // RdCost
    m_cRdCost.setCostMode ( COST_STANDARD_LOSSY ); // not used in decoder side RdCost stuff -> set to default

    m_cSliceDecoder.create();

    if( sps->getALFEnabledFlag() )
    {
#if JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
      const int maxDepth = floorLog2(sps->getMaxCUWidth()) - sps->getLog2MinCodingBlockSize();
      m_cALF.create( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), maxDepth, sps->getBitDepths().recon);
#else
      m_cALF.create( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCodingDepth(), sps->getBitDepths().recon );
#endif
    }
#if JVET_Q0795_CCALF
    pSlice->m_ccAlfFilterControl[0] = m_cALF.getCcAlfControlIdc(COMPONENT_Cb);
    pSlice->m_ccAlfFilterControl[1] = m_cALF.getCcAlfControlIdc(COMPONENT_Cr);
#endif
  }
  else
  {
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    m_pcPic->allocateNewSlice();
    CHECK(m_pcPic->slices.size() != (size_t)(m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pcPic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    Slice *pSlice = m_pcPic->slices[m_uiSliceSegmentIdx]; // we now have a real slice.

    const SPS *sps = pSlice->getSPS();
    const PPS *pps = pSlice->getPPS();
    APS** apss = pSlice->getAlfAPSs();
    APS *lmcsAPS = m_picHeader.getLmcsAPS();
    APS *scalinglistAPS = m_picHeader.getScalingListAPS();

    // fix Parameter Sets, now that we have the real slice
    m_pcPic->cs->slice = pSlice;
    m_pcPic->cs->sps   = sps;
    m_pcPic->cs->pps   = pps;
    memcpy(m_pcPic->cs->alfApss, apss, sizeof(m_pcPic->cs->alfApss));
    m_pcPic->cs->lmcsAps = lmcsAPS;
    m_pcPic->cs->scalinglistAps = scalinglistAPS;

    m_pcPic->cs->pcv   = pps->pcv;

    // check that the current active PPS has not changed...
    if (m_parameterSetManager.getSPSChangedFlag(sps->getSPSId()) )
    {
      EXIT("Error - a new SPS has been decoded while processing a picture");
    }
    if (m_parameterSetManager.getPPSChangedFlag(pps->getPPSId()) )
    {
      EXIT("Error - a new PPS has been decoded while processing a picture");
    }
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* aps = m_parameterSetManager.getAPS(i, ALF_APS);
      if (aps && m_parameterSetManager.getAPSChangedFlag(i, ALF_APS))
      {
        EXIT("Error - a new APS has been decoded while processing a picture");
      }
    }

    if (lmcsAPS && m_parameterSetManager.getAPSChangedFlag(lmcsAPS->getAPSId(), LMCS_APS) )
    {
      EXIT("Error - a new LMCS APS has been decoded while processing a picture");
    }
    if( scalinglistAPS && m_parameterSetManager.getAPSChangedFlag( scalinglistAPS->getAPSId(), SCALING_LIST_APS ) )
    {
      EXIT( "Error - a new SCALING LIST APS has been decoded while processing a picture" );
    }

    activateAPS(&m_picHeader, pSlice, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    m_pcPic->cs->lmcsAps = lmcsAPS;
    m_pcPic->cs->scalinglistAps = scalinglistAPS;

    xParsePrefixSEImessages();

    // Check if any new SEI has arrived
     if(!m_SEIs.empty())
     {
       // Currently only decoding Unit SEI message occurring between VCL NALUs copied
       SEIMessages& picSEI = m_pcPic->SEIs;
       SEIMessages decodingUnitInfos = extractSeisByType( picSEI, SEI::DECODING_UNIT_INFO);
       picSEI.insert(picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end());
       deleteSEIs(m_SEIs);
     }
  }
  xCheckParameterSetConstraints(layerId);
}

void DecLib::xCheckParameterSetConstraints(const int layerId)
{
  // Conformance checks
  Slice *slice = m_pcPic->slices[m_uiSliceSegmentIdx];
  const SPS *sps = slice->getSPS();
  const PPS *pps = slice->getPPS();
#if JVET_Q0814_DPB
  const VPS *vps = slice->getVPS();
#endif

#if SPS_ID_CHECK
  static std::unordered_map<int, int> m_clvssSPSid;

  if( slice->isClvssPu() && m_bFirstSliceInPicture )
  {
    m_clvssSPSid[layerId] = pps->getSPSId();
  }

  CHECK( m_clvssSPSid[layerId] != pps->getSPSId(), "The value of pps_seq_parameter_set_id shall be the same in all PPSs that are referred to by coded pictures in a CLVS" );
#endif

#if JVET_Q414_CONSTRAINT_ON_GDR_PIC_FLAG
  CHECK(sps->getGDREnabledFlag() == false && m_picHeader.getGdrPicFlag(), "When gdr_enabled_flag is equal to 0, the value of gdr_pic_flag shall be equal to 0 ");
#endif
  if( !sps->getUseWP() )
  {
    CHECK( pps->getUseWP(), "When sps_weighted_pred_flag is equal to 0, the value of pps_weighted_pred_flag shall be equal to 0." );
  }

  if( !sps->getUseWPBiPred() )
  {
    CHECK( pps->getWPBiPred(), "When sps_weighted_bipred_flag is equal to 0, the value of pps_weighted_bipred_flag shall be equal to 0." );
  }

#if JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
  const int minCuSize = 1 << sps->getLog2MinCodingBlockSize();
  CHECK( ( pps->getPicWidthInLumaSamples() % ( std::max( 8, minCuSize) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( pps->getPicHeightInLumaSamples() % ( std::max( 8, minCuSize) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );
#else
  CHECK( ( pps->getPicWidthInLumaSamples() % ( std::max( 8, int( sps->getMaxCUWidth() >> ( sps->getMaxCodingDepth() - 1 ) ) ) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( pps->getPicHeightInLumaSamples() % ( std::max( 8, int( sps->getMaxCUHeight() >> ( sps->getMaxCodingDepth() - 1 ) ) ) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );
#endif
#if JVET_Q0043_RPR_and_Subpics
  if( !sps->getRprEnabledFlag() ) 
  {
    CHECK( pps->getPicWidthInLumaSamples() != sps->getMaxPicWidthInLumaSamples(), "When res_change_in_clvs_allowed_flag equal to 0, the value of pic_width_in_luma_samples shall be equal to pic_width_max_in_luma_samples." );
    CHECK( pps->getPicHeightInLumaSamples() != sps->getMaxPicHeightInLumaSamples(), "When res_change_in_clvs_allowed_flag equal to 0, the value of pic_height_in_luma_samples shall be equal to pic_height_max_in_luma_samples." );
  }
  if( sps->getRprEnabledFlag() )
  {
#if JVET_Q0119_CLEANUPS
    CHECK( sps->getSubPicInfoPresentFlag() != 0, "When res_change_in_clvs_allowed_flag is equal to 1, the value of subpic_info_present_flag shall be equal to 0." );
#else
    CHECK( sps->getSubPicPresentFlag() != 0, "When res_change_in_clvs_allowed_flag is equal to 1, the value of subpic_info_present_flag shall be equal to 0." );
#endif
  }
  CHECK( !sps->getRprEnabledFlag() && pps->getScalingWindow().getWindowEnabledFlag(), "When res_change_in_clvs_allowed_flag is equal to 0, the value of scaling_window_flag shall be equal to 0." );
#else  
  if( !sps->getRprEnabledFlag() && sps->getSubPicPresentFlag() )
  {
    CHECK( pps->getPicWidthInLumaSamples() != sps->getMaxPicWidthInLumaSamples(), "When subpics_present_flag is equal to 1 or ref_pic_resampling_enabled_flag equal to 0, the value of pic_width_in_luma_samples shall be equal to pic_width_max_in_luma_samples." );
    CHECK( pps->getPicHeightInLumaSamples() != sps->getMaxPicHeightInLumaSamples(), "When subpics_present_flag is equal to 1 or ref_pic_resampling_enabled_flag equal to 0, the value of pic_height_in_luma_samples shall be equal to pic_height_max_in_luma_samples." );
  }
  CHECK( !sps->getRprEnabledFlag() && pps->getScalingWindow().getWindowEnabledFlag(), "When ref_pic_resampling_enabled_flag is equal to 0, the value of scaling_window_flag shall be equal to 0." );
#endif

#if JVET_Q0417_CONSTRAINT_SPS_VB_PRESENT_FLAG
  CHECK(sps->getRprEnabledFlag() && sps->getLoopFilterAcrossVirtualBoundariesDisabledFlag(), "when the value of res_change_in_clvs_allowed_flag is equal to 1, the value of sps_virtual_boundaries_present_flag shall be equal to 0");
#endif

  if( sps->getCTUSize() + 2 * ( 1 << sps->getLog2MinCodingBlockSize() ) > pps->getPicWidthInLumaSamples() )
  {
    CHECK( sps->getWrapAroundEnabledFlag(), "Wraparound shall be disabled when the value of ( CtbSizeY / MinCbSizeY + 1) is less than or equal to ( pic_width_in_luma_samples / MinCbSizeY - 1 )" );
  }

#if JVET_Q0814_DPB
  if( vps != nullptr && vps->m_numOutputLayersInOls[vps->m_targetOlsIdx] > 1 )
  {
    CHECK( sps->getMaxPicWidthInLumaSamples() > vps->getOlsDpbPicSize( vps->m_targetOlsIdx ).width, "pic_width_max_in_luma_samples shall be less than or equal to the value of ols_dpb_pic_width[ i ]" );
    CHECK( sps->getMaxPicHeightInLumaSamples() > vps->getOlsDpbPicSize( vps->m_targetOlsIdx ).height, "pic_height_max_in_luma_samples shall be less than or equal to the value of ols_dpb_pic_height[ i ]" );
  }
#endif

#if JVET_Q0172_CHROMA_FORMAT_BITDEPTH_CONSTRAINT
  static std::unordered_map<int, int> m_layerChromaFormat;
  static std::unordered_map<int, int> m_layerBitDepth;

  if (vps != nullptr && vps->getMaxLayers() > 1)
  {
    int curLayerIdx = vps->getGeneralLayerIdx(layerId);
    int curLayerChromaFormat = sps->getChromaFormatIdc();
    int curLayerBitDepth = sps->getBitDepth(CHANNEL_TYPE_LUMA);

#if SPS_ID_CHECK
    if (slice->isClvssPu() && m_bFirstSliceInPicture)
#else
    if (m_layerBitDepth[curLayerIdx] == 0)
#endif
    {
      m_layerChromaFormat[curLayerIdx] = curLayerChromaFormat;
      m_layerBitDepth[curLayerIdx] = curLayerBitDepth;
    }
    else
    {
      CHECK(m_layerChromaFormat[curLayerIdx] != curLayerChromaFormat, "Different chroma format in the same layer.");
      CHECK(m_layerBitDepth[curLayerIdx] != curLayerBitDepth, "Different bit-depth in the same layer.");
    }

    for (int i = 0; i < curLayerIdx; i++)
    {
      if (vps->getDirectRefLayerFlag(curLayerIdx, i))
      {
        int refLayerChromaFormat = m_layerChromaFormat[i];
        CHECK(curLayerChromaFormat != refLayerChromaFormat, "The chroma formats of the current layer and the reference layer are different");
        int refLayerBitDepth = m_layerBitDepth[i];
        CHECK(curLayerBitDepth != refLayerBitDepth, "The bit-depth of the current layer and the reference layer are different");
      }
    }
  }
#endif

#if JVET_Q0114_CONSTRAINT_FLAGS
  if (sps->getProfileTierLevel()->getConstraintInfo()->getOneTilePerPicConstraintFlag())
  {
    CHECK(pps->getNumTiles() != 1, "When one_tile_per_pic_constraint_flag is equal to 1, each picture shall contain only one tile");
  }
  if (sps->getProfileTierLevel()->getConstraintInfo()->getOneSlicePerPicConstraintFlag())
  {
    CHECK( pps->getNumSlicesInPic() != 1, "When one_slice_per_pic_constraint_flag is equal to 1, each picture shall contain only one slice");
  }
#endif

#if JVET_Q0260_CONFORMANCE_WINDOW_IN_SPS
  if (sps->getMaxPicWidthInLumaSamples() == pps->getPicWidthInLumaSamples() &&
      sps->getMaxPicHeightInLumaSamples() == pps->getPicHeightInLumaSamples())
  {
    const Window& spsConfWin = sps->getConformanceWindow();
    const Window& ppsConfWin = pps->getConformanceWindow();
    CHECK(spsConfWin.getWindowLeftOffset() != ppsConfWin.getWindowLeftOffset(), "When picture size is equal to maximum picutre size, conformance window left offset in SPS and PPS shall be equal");
    CHECK(spsConfWin.getWindowRightOffset() != ppsConfWin.getWindowRightOffset(), "When picture size is equal to maximum picutre size, conformance window right offset in SPS and PPS shall be equal");
    CHECK(spsConfWin.getWindowTopOffset() != ppsConfWin.getWindowTopOffset(), "When picture size is equal to maximum picutre size, conformance window top offset in SPS and PPS shall be equal");
    CHECK(spsConfWin.getWindowBottomOffset() != ppsConfWin.getWindowBottomOffset(), "When picture size is equal to maximum picutre size, conformance window bottom offset in SPS and PPS shall be equal");
  }
#endif
}


void DecLib::xParsePrefixSEIsForUnknownVCLNal()
{
  while (!m_prefixSEINALUs.empty())
  {
    // do nothing?
    msg( NOTICE, "Discarding Prefix SEI associated with unknown VCL NAL unit.\n");
    delete m_prefixSEINALUs.front();
  }
  // TODO: discard following suffix SEIs as well?
}


void DecLib::xParsePrefixSEImessages()
{
  while (!m_prefixSEINALUs.empty())
  {
    InputNALUnit &nalu=*m_prefixSEINALUs.front();
#if JVET_P0125_ASPECT_TID_LAYER_ID_NUH
    m_accessUnitSeiTids.push_back(nalu.m_temporalId);
#endif
    m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_SEIs, nalu.m_nalUnitType, nalu.m_temporalId, m_parameterSetManager.getActiveSPS(), m_HRD, m_pDecodedSEIOutputStream );
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
}

void DecLib::xDecodePicHeader( InputNALUnit& nalu )
{
  m_HLSReader.setBitstream( &nalu.getBitstream() );
#if JVET_Q0775_PH_IN_SH
  m_HLSReader.parsePictureHeader( &m_picHeader, &m_parameterSetManager, true );
#else
  m_HLSReader.parsePictureHeader( &m_picHeader, &m_parameterSetManager);
#endif
  m_picHeader.setValid();
}

bool DecLib::xDecodeSlice(InputNALUnit &nalu, int &iSkipFrame, int iPOCLastDisplay )
{
#if !JVET_Q0775_PH_IN_SH
  if(m_picHeader.isValid() == false)
  {
    return false;
  }
#endif
  m_apcSlicePilot->setPicHeader( &m_picHeader );
  m_apcSlicePilot->initSlice(); // the slice pilot is an object to prepare for a new slice
                                // it is not associated with picture, sps or pps structures.

  Picture* scaledRefPic[MAX_NUM_REF] = {};

  if (m_bFirstSliceInPicture)
  {
    m_uiSliceSegmentIdx = 0;
  }
  else
  {
    CHECK(nalu.m_nalUnitType != m_pcPic->slices[m_uiSliceSegmentIdx - 1]->getNalUnitType(), "The value of NAL unit type shall be the same for all coded slice NAL units of a picture");
    m_apcSlicePilot->copySliceInfo( m_pcPic->slices[m_uiSliceSegmentIdx-1] );
  }

  m_apcSlicePilot->setNalUnitType(nalu.m_nalUnitType);
#if JVET_Q0118_CLEANUPS
  m_apcSlicePilot->setNalUnitLayerId(nalu.m_nuhLayerId);
#endif
  m_apcSlicePilot->setTLayer(nalu.m_temporalId);

  for( auto& naluTemporalId : m_accessUnitNals )
  {
#if JVET_Q0117_PARAMETER_SETS_CLEANUP
    if (
      naluTemporalId.first != NAL_UNIT_DCI
      && naluTemporalId.first != NAL_UNIT_VPS
      && naluTemporalId.first != NAL_UNIT_SPS
      && naluTemporalId.first != NAL_UNIT_EOS
      && naluTemporalId.first != NAL_UNIT_EOB)
#else
    if (
      naluTemporalId.first != NAL_UNIT_DPS
      && naluTemporalId.first != NAL_UNIT_VPS
      && naluTemporalId.first != NAL_UNIT_SPS
      && naluTemporalId.first != NAL_UNIT_EOS
      && naluTemporalId.first != NAL_UNIT_EOB )
#endif

    {
      CHECK( naluTemporalId.second < nalu.m_temporalId, "TemporalId shall be greater than or equal to the TemporalId of the layer access unit containing the NAL unit" );
    }
  }

  if (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR)
  {
    CHECK(nalu.m_temporalId != 0, "Current GDR picture has TemporalId not equal to 0");
  }

  m_HLSReader.setBitstream( &nalu.getBitstream() );
#if JVET_Q0795_CCALF
  m_apcSlicePilot->m_ccAlfFilterParam = m_cALF.getCcAlfFilterParam();
#endif
  m_HLSReader.parseSliceHeader( m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC );

#if JVET_P0101_POC_MULTILAYER || JVET_P0097_REMOVE_VPS_DEP_NONSCALABLE_LAYER
  PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId());
  CHECK(pps == 0, "No PPS present");
  SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());
  CHECK(sps == 0, "No SPS present");
  VPS *vps = m_parameterSetManager.getVPS(sps->getVPSId());
#endif
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  int currSubPicIdx = pps->getSubPicIdxFromSubPicId( m_apcSlicePilot->getSliceSubPicId() );
  int currSliceAddr = m_apcSlicePilot->getSliceID();
  for(int sp = 0; sp < currSubPicIdx; sp++)
  {
    currSliceAddr -= pps->getSubPic(sp).getNumSlicesInSubPic();
  }
  CHECK( currSubPicIdx < m_maxDecSubPicIdx, "Error in the order of coded slice NAL units of subpictures" );
  CHECK( currSubPicIdx == m_maxDecSubPicIdx && currSliceAddr <= m_maxDecSliceAddrInSubPic, "Error in the order of coded slice NAL units within a subpicture" );
  if( currSubPicIdx == m_maxDecSubPicIdx )
  {
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
  if( currSubPicIdx > m_maxDecSubPicIdx )
  {
    m_maxDecSubPicIdx = currSubPicIdx; 
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
#endif
#if JVET_P0097_REMOVE_VPS_DEP_NONSCALABLE_LAYER
  if ((sps->getVPSId() == 0) && (m_prevLayerID != MAX_INT))
  {
    CHECK(m_prevLayerID != nalu.m_nuhLayerId, "All VCL NAL unit in the CVS shall have the same value of nuh_layer_id "
                                              "when sps_video_parameter_set_id is equal to 0");
  }
#endif
#if JVET_P0101_POC_MULTILAYER
  CHECK((sps->getVPSId() > 0) && (vps == 0), "Invalid VPS");
  if (vps != nullptr && (vps->getIndependentLayerFlag(nalu.m_nuhLayerId) == 0)) 
  {
    bool pocIsSet = false;
    for(auto auNALit=m_accessUnitPicInfo.begin(); auNALit != m_accessUnitPicInfo.end();auNALit++)
    {      
      for (int iRefIdx = 0; iRefIdx < m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_0) && !pocIsSet; iRefIdx++)
      {
#if JVET_Q0819_PH_CHANGES
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx) && m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() == (*auNALit).m_POC)
#else
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() == (*auNALit).m_POC)
#endif
        {
          m_apcSlicePilot->setPOC(m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC());
          pocIsSet = true;
        }
      }
      for (int iRefIdx = 0; iRefIdx < m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_1) && !pocIsSet; iRefIdx++)
      {
#if JVET_Q0819_PH_CHANGES
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx) && m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() == (*auNALit).m_POC)
#else
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() == (*auNALit).m_POC)
#endif
        {
          m_apcSlicePilot->setPOC(m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC());
          pocIsSet = true;
        }
      }
    }
  }
#endif

  // update independent slice index
  uint32_t uiIndependentSliceIdx = 0;
  if (!m_bFirstSliceInPicture)
  {
    uiIndependentSliceIdx = m_pcPic->slices[m_uiSliceSegmentIdx-1]->getIndependentSliceIdx();
      uiIndependentSliceIdx++;
  }
  m_apcSlicePilot->setIndependentSliceIdx(uiIndependentSliceIdx);

#if K0149_BLOCK_STATISTICS
#if !JVET_P0101_POC_MULTILAYER
  PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId());
  CHECK(pps == 0, "No PPS present");
  SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());
  CHECK(sps == 0, "No SPS present");
#endif
  writeBlockStatisticsHeader(sps);
#endif

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "poc", m_apcSlicePilot->getPOC() ) );

  if ((m_bFirstSliceInPicture ||
        m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ||
        m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) &&
      getNoOutputPriorPicsFlag())
    {
      checkNoOutputPriorPics(&m_cListPic);
      setNoOutputPriorPicsFlag (false);
    }

  xUpdatePreviousTid0POC(m_apcSlicePilot);

  m_apcSlicePilot->setAssociatedIRAPPOC(m_pocCRA);
  m_apcSlicePilot->setAssociatedIRAPType(m_associatedIRAPType);

  //For inference of NoOutputOfPriorPicsFlag
  if (m_apcSlicePilot->getRapPicFlag() || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR)
  {
#if JVET_P0125_EOS_LAYER_SPECIFIC
    if ((m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_bFirstSliceInSequence[nalu.m_nuhLayerId]) ||
      (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_apcSlicePilot->getHandleCraAsCvsStartFlag()) ||
      (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR && m_bFirstSliceInSequence[nalu.m_nuhLayerId]))
#else
    if ((m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_bFirstSliceInSequence) ||
        (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_apcSlicePilot->getHandleCraAsCvsStartFlag()) ||
        (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR && m_bFirstSliceInSequence))
#endif
    {
      m_apcSlicePilot->setNoIncorrectPicOutputFlag(true);
    }
    //the inference for NoOutputPriorPicsFlag
    if (!m_bFirstSliceInBitstream &&
        (m_apcSlicePilot->getRapPicFlag() || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) &&
        m_apcSlicePilot->getNoIncorrectPicOutputFlag())
    {
      if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR)
      {
        m_picHeader.setNoOutputOfPriorPicsFlag(true);
      }
    }
    else
    {
      m_picHeader.setNoOutputOfPriorPicsFlag(false);
    }

    if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR)
    {
      m_lastNoIncorrectPicOutputFlag = m_apcSlicePilot->getNoIncorrectPicOutputFlag();
    }
  }
  if ((m_apcSlicePilot->getRapPicFlag() || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) && m_picHeader.getNoOutputOfPriorPicsFlag())
  {
    m_lastPOCNoOutputPriorPics = m_apcSlicePilot->getPOC();
    m_isNoOutputPriorPics = true;
  }
  else
  {
    m_isNoOutputPriorPics = false;
  }

  //For inference of PicOutputFlag
  if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL)
  {
    if (m_lastNoIncorrectPicOutputFlag)
    {
      m_picHeader.setPicOutputFlag(false);
    }
  }

#if JVET_P0288_PIC_OUTPUT
  {
    PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId());
    CHECK(pps == 0, "No PPS present");
    SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());
    CHECK(sps == 0, "No SPS present");
    if (sps->getVPSId() > 0)
    {
      VPS *vps = m_parameterSetManager.getVPS(sps->getVPSId());
      CHECK(vps == 0, "No VPS present");
      if ((vps->getOlsModeIdc() == 0 && vps->getGeneralLayerIdx(nalu.m_nuhLayerId) < (vps->getMaxLayers() - 1) && vps->getOlsOutputLayerFlag(vps->m_targetOlsIdx, vps->getMaxLayers() - 1) == 1) || (vps->getOlsModeIdc() == 2 && vps->getOlsOutputLayerFlag(vps->m_targetOlsIdx, vps->getGeneralLayerIdx(nalu.m_nuhLayerId)) == 0))
      {
        m_picHeader.setPicOutputFlag(false);
      }
    }
  }
#endif

  if ((m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) &&
      m_lastNoIncorrectPicOutputFlag)                     //Reset POC MSB when CRA or GDR has NoIncorrectPicOutputFlag equal to 1
  {
#if !JVET_P0101_POC_MULTILAYER
    PPS *pps = m_parameterSetManager.getPPS(m_picHeader.getPPSId());
    CHECK(pps == 0, "No PPS present");
    SPS *sps = m_parameterSetManager.getSPS(pps->getSPSId());
    CHECK(sps == 0, "No SPS present");
#endif
    int iMaxPOClsb = 1 << sps->getBitsForPOC();
    m_apcSlicePilot->setPOC( m_apcSlicePilot->getPOC() & (iMaxPOClsb - 1) );
    xUpdatePreviousTid0POC(m_apcSlicePilot);
  }

#if JVET_P0101_POC_MULTILAYER
  AccessUnitPicInfo picInfo;
  picInfo.m_nalUnitType = nalu.m_nalUnitType;
  picInfo.m_nuhLayerId  = nalu.m_nuhLayerId;
  picInfo.m_temporalId  = nalu.m_temporalId;
  picInfo.m_POC         = m_apcSlicePilot->getPOC();
  m_accessUnitPicInfo.push_back(picInfo);
#endif

  // Skip pictures due to random access

  if (isRandomAccessSkipPicture(iSkipFrame, iPOCLastDisplay))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC = m_apcSlicePilot->getPOC();
    return false;
  }
  // Skip TFD pictures associated with BLA/BLANT pictures

  // clear previous slice skipped flag
  m_prevSliceSkipped = false;

  //we should only get a different poc for a new picture (with CTU address==0)
#if JVET_P0125_EOS_LAYER_SPECIFIC
  if (m_apcSlicePilot->getPOC() != m_prevPOC && !m_bFirstSliceInSequence[nalu.m_nuhLayerId] && (m_apcSlicePilot->getFirstCtuRsAddrInSlice() != 0))
#else
  if(m_apcSlicePilot->getPOC() != m_prevPOC && !m_bFirstSliceInSequence && (m_apcSlicePilot->getFirstCtuRsAddrInSlice() != 0))
#endif
  {
    msg( WARNING, "Warning, the first slice of a picture might have been lost!\n");
  }
  m_prevLayerID = nalu.m_nuhLayerId;

  // leave when a new picture is found
  if(m_apcSlicePilot->getFirstCtuRsAddrInSlice() == 0 && !m_bFirstSliceInPicture)
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 0 ) );
      m_prevPOC = m_apcSlicePilot->getPOC();
      return true;
    }
    m_prevPOC = m_apcSlicePilot->getPOC();
  }
  else
  {
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
  }

#if FIX_TICKET891
  // actual decoding starts here
  xActivateParameterSets( nalu.m_nuhLayerId );
#endif

  //detect lost reference picture and insert copy of earlier frame.
  {
    int lostPoc;
    int refPicIndex;
#if FIX_TICKET891
    while ((lostPoc = m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPL0(), 0, true, &refPicIndex, m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_0))) > 0)
#else
    while ((lostPoc = m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPL0(), 0, true, &refPicIndex)) > 0)
#endif
    {
      if ( ( (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) || (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA) ) && m_apcSlicePilot->getNoIncorrectPicOutputFlag() )
      {
        if (m_apcSlicePilot->getRPL0()->isInterLayerRefPic(refPicIndex) == 0)
        {
          xCreateUnavailablePicture(lostPoc - 1, m_apcSlicePilot->getRPL0()->isRefPicLongterm(refPicIndex), m_apcSlicePilot->getPic()->layerId, m_apcSlicePilot->getRPL0()->isInterLayerRefPic(refPicIndex));
        }
      }
      else
      {
        xCreateLostPicture( lostPoc - 1, m_apcSlicePilot->getPic()->layerId );
      }
    }
#if FIX_TICKET891
    while ((lostPoc = m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPL1(), 0, true, &refPicIndex, m_apcSlicePilot->getNumRefIdx(REF_PIC_LIST_1))) > 0)
#else
    while ((lostPoc = m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPL1(), 0, true, &refPicIndex)) > 0)
#endif
    {
      if (((m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR) || (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)) && m_apcSlicePilot->getNoIncorrectPicOutputFlag())
      {
        if (m_apcSlicePilot->getRPL1()->isInterLayerRefPic(refPicIndex) == 0)
        {
          xCreateUnavailablePicture(lostPoc - 1, m_apcSlicePilot->getRPL1()->isRefPicLongterm(refPicIndex), m_apcSlicePilot->getPic()->layerId, m_apcSlicePilot->getRPL1()->isInterLayerRefPic(refPicIndex));
        }
      }
      else
      {
        xCreateLostPicture( lostPoc - 1, m_apcSlicePilot->getPic()->layerId );
      }
    }
  }

    m_prevPOC = m_apcSlicePilot->getPOC();

  if (m_bFirstSliceInPicture)
  {
    xUpdateRasInit(m_apcSlicePilot);
  }

#if !FIX_TICKET891  
  // actual decoding starts here
  xActivateParameterSets( nalu.m_nuhLayerId );
#endif

#if JVET_P0125_EOS_LAYER_SPECIFIC
  m_bFirstSliceInSequence[nalu.m_nuhLayerId] = false;
#else
  m_bFirstSliceInSequence = false;
#endif
  m_bFirstSliceInBitstream  = false;


  Slice* pcSlice = m_pcPic->slices[m_uiSliceSegmentIdx];
  pcSlice->setPic( m_pcPic );
  m_pcPic->poc         = pcSlice->getPOC();
  m_pcPic->layer       = pcSlice->getTLayer();
  m_pcPic->referenced  = true;
  m_pcPic->layer       = nalu.m_temporalId;
  m_pcPic->layerId    = nalu.m_nuhLayerId;
    m_pcPic->subLayerNonReferencePictureDueToSTSA = false;


  pcSlice->checkCRA(pcSlice->getRPL0(), pcSlice->getRPL1(), m_pocCRA, m_associatedIRAPType, m_cListPic);
  pcSlice->constructRefPicList(m_cListPic);
  pcSlice->checkSTSA(m_cListPic);

  pcSlice->scaleRefPicList( scaledRefPic, m_pcPic->cs->picHeader, m_parameterSetManager.getAPSs(), m_picHeader.getLmcsAPS(), m_picHeader.getScalingListAPS(), true );

    if (!pcSlice->isIntra())
    {
      bool bLowDelay = true;
      int  iCurrPOC  = pcSlice->getPOC();
      int iRefIdx = 0;

      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      if (pcSlice->isInterB())
      {
        for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
        {
          if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
          {
            bLowDelay = false;
          }
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }

    if (pcSlice->getSPS()->getUseSMVD() && pcSlice->getCheckLDC() == false
      && pcSlice->getPicHeader()->getMvdL1ZeroFlag() == false
      )
    {
      int currPOC = pcSlice->getPOC();

      int forwardPOC = currPOC;
      int backwardPOC = currPOC;
      int ref = 0;
      int refIdx0 = -1;
      int refIdx1 = -1;

      // search nearest forward POC in List 0
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_0, ref)->longTerm;
        if ( poc < currPOC && (poc > forwardPOC || refIdx0 == -1) && !isRefLongTerm )
        {
          forwardPOC = poc;
          refIdx0 = ref;
        }
      }

      // search nearest backward POC in List 1
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_1, ref)->longTerm;
        if ( poc > currPOC && (poc < backwardPOC || refIdx1 == -1) && !isRefLongTerm )
        {
          backwardPOC = poc;
          refIdx1 = ref;
        }
      }

      if ( !(forwardPOC < currPOC && backwardPOC > currPOC) )
      {
        forwardPOC = currPOC;
        backwardPOC = currPOC;
        refIdx0 = -1;
        refIdx1 = -1;

        // search nearest backward POC in List 0
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_0, ref)->longTerm;
          if ( poc > currPOC && (poc < backwardPOC || refIdx0 == -1) && !isRefLongTerm )
          {
            backwardPOC = poc;
            refIdx0 = ref;
          }
        }

        // search nearest forward POC in List 1
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic(REF_PIC_LIST_1, ref)->longTerm;
          if ( poc < currPOC && (poc > forwardPOC || refIdx1 == -1) && !isRefLongTerm )
          {
            forwardPOC = poc;
            refIdx1 = ref;
          }
        }
      }

      if ( forwardPOC < currPOC && backwardPOC > currPOC )
      {
        pcSlice->setBiDirPred( true, refIdx0, refIdx1 );
      }
      else
      {
        pcSlice->setBiDirPred( false, -1, -1 );
      }
    }
    else
    {
      pcSlice->setBiDirPred( false, -1, -1 );
    }

    //---------------
    pcSlice->setRefPOCList();

    SEIMessages drapSEIs = getSeisByType(m_pcPic->SEIs, SEI::DEPENDENT_RAP_INDICATION );
    if (!drapSEIs.empty())
    {
      msg( NOTICE, "Dependent RAP indication SEI decoded\n");
      pcSlice->setDRAP(true);
      pcSlice->setLatestDRAPPOC(pcSlice->getPOC());
    }
    pcSlice->checkConformanceForDRAP(nalu.m_temporalId);

  Quant *quant = m_cTrQuant.getQuant();

  if( pcSlice->getSPS()->getScalingListFlag() )
  {
    ScalingList scalingList;
    if( pcSlice->getPicHeader()->getScalingListPresentFlag() )
    {
      APS* scalingListAPS = pcSlice->getPicHeader()->getScalingListAPS();
      scalingList = scalingListAPS->getScalingList();
    }
    else
    {
      scalingList.setDefaultScalingList();
    }
    int scalingListAPSId = pcSlice->getPicHeader()->getScalingListAPSId();
    if (getScalingListUpdateFlag() || (scalingListAPSId != getPreScalingListAPSId()))
    {
      quant->setScalingListDec(scalingList);
      setScalingListUpdateFlag(false);
      setPreScalingListAPSId(scalingListAPSId);
    }
    quant->setUseScalingList( true );
  }
  else
  {
    quant->setUseScalingList( false );
  }


  if (pcSlice->getSPS()->getUseLmcs())
  {
    if (m_bFirstSliceInPicture)
      m_sliceLmcsApsId = -1;
    if (pcSlice->getPicHeader()->getLmcsEnabledFlag())
    {
      APS* lmcsAPS = pcSlice->getPicHeader()->getLmcsAPS();
      if (m_sliceLmcsApsId == -1)
      {
        m_sliceLmcsApsId = lmcsAPS->getAPSId();
      }
      else
      {
        CHECK(lmcsAPS->getAPSId() != m_sliceLmcsApsId, "same APS ID shall be used for all slices in one picture");
      }
      SliceReshapeInfo& sInfo = lmcsAPS->getReshaperAPSInfo();
      SliceReshapeInfo& tInfo = m_cReshaper.getSliceReshaperInfo();
      tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
      tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
      memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int)*(PIC_CODE_CW_BINS));
      tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
      tInfo.chrResScalingOffset = sInfo.chrResScalingOffset;
      tInfo.setUseSliceReshaper(pcSlice->getPicHeader()->getLmcsEnabledFlag());
      tInfo.setSliceReshapeChromaAdj(pcSlice->getPicHeader()->getLmcsChromaResidualScaleFlag());
      tInfo.setSliceReshapeModelPresentFlag(true);
    }
    else
    {
      SliceReshapeInfo& tInfo = m_cReshaper.getSliceReshaperInfo();
      tInfo.setUseSliceReshaper(false);
      tInfo.setSliceReshapeChromaAdj(false);
      tInfo.setSliceReshapeModelPresentFlag(false);
    }
    if (pcSlice->getPicHeader()->getLmcsEnabledFlag())
    {
      m_cReshaper.constructReshaper();
    }
    else
    {
      m_cReshaper.setReshapeFlag(false);
    }
    if ((pcSlice->getSliceType() == I_SLICE) && m_cReshaper.getSliceReshaperInfo().getUseSliceReshaper())
    {
      m_cReshaper.setCTUFlag(false);
      m_cReshaper.setRecReshaped(true);
    }
    else
    {
      if (m_cReshaper.getSliceReshaperInfo().getUseSliceReshaper())
      {
        m_cReshaper.setCTUFlag(true);
        m_cReshaper.setRecReshaped(true);
      }
      else
      {
        m_cReshaper.setCTUFlag(false);
        m_cReshaper.setRecReshaped(false);
      }
    }
    m_cReshaper.setVPDULoc(-1, -1);
  }
  else
  {
    m_cReshaper.setCTUFlag(false);
    m_cReshaper.setRecReshaped(false);
  }

  //  Decode a picture
  m_cSliceDecoder.decompressSlice( pcSlice, &( nalu.getBitstream() ), ( m_pcPic->poc == getDebugPOC() ? getDebugCTU() : -1 ) );

  m_bFirstSliceInPicture = false;
  m_uiSliceSegmentIdx++;

  pcSlice->freeScaledRefPicList( scaledRefPic );

  return false;
}

void DecLib::xDecodeVPS( InputNALUnit& nalu )
{
  m_vps = new VPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK( nalu.m_temporalId, "The value of TemporalId of VPS NAL units shall be equal to 0" );

  m_HLSReader.parseVPS( m_vps );
  m_parameterSetManager.storeVPS( m_vps, nalu.getBitstream().getFifo());
}

#if JVET_Q0117_PARAMETER_SETS_CLEANUP
void DecLib::xDecodeDCI(InputNALUnit& nalu)
{
  m_HLSReader.setBitstream(&nalu.getBitstream());

  CHECK(nalu.m_temporalId, "The value of TemporalId of DCI NAL units shall be equal to 0");
  if (!m_dci)
  {
    m_dci = new DCI;
    m_HLSReader.parseDCI(m_dci);
  }
  else
  {
    DCI dupDCI;
    m_HLSReader.parseDCI(&dupDCI);
    CHECK( !m_dci->IsIndenticalDCI(dupDCI), "Two signaled DCIs are different");
  }
}
#else
void DecLib::xDecodeDPS( InputNALUnit& nalu )
{
  DPS* dps = new DPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK( nalu.m_temporalId, "The value of TemporalId of DPS NAL units shall be equal to 0" );

  m_HLSReader.parseDPS( dps );
  m_parameterSetManager.storeDPS( dps, nalu.getBitstream().getFifo() );
}
#endif

void DecLib::xDecodeSPS( InputNALUnit& nalu )
{
  SPS* sps = new SPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK( nalu.m_temporalId, "The value of TemporalId of SPS NAL units shall be equal to 0" );

  m_HLSReader.parseSPS( sps );
  DTRACE( g_trace_ctx, D_QP_PER_CTU, "CTU Size: %dx%d", sps->getMaxCUWidth(), sps->getMaxCUHeight() );
  m_parameterSetManager.storeSPS( sps, nalu.getBitstream().getFifo() );
}

void DecLib::xDecodePPS( InputNALUnit& nalu )
{
  PPS* pps = new PPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePPS( pps );
  pps->setLayerId( nalu.m_nuhLayerId );
  pps->setTemporalId( nalu.m_temporalId );
  m_parameterSetManager.storePPS( pps, nalu.getBitstream().getFifo() );
}

void DecLib::xDecodeAPS(InputNALUnit& nalu)
{
  APS* aps = new APS();
  m_HLSReader.setBitstream(&nalu.getBitstream());
  m_HLSReader.parseAPS(aps);
  aps->setTemporalId(nalu.m_temporalId);
  aps->setLayerId( nalu.m_nuhLayerId );
  m_parameterSetManager.checkAuApsContent( aps, m_accessUnitApsNals );
  if (aps->getAPSType() == SCALING_LIST_APS)
  {
    setScalingListUpdateFlag(true);
  }

  // aps will be deleted if it was already stored (and did not changed),
  // thus, storing it must be last action.
  m_parameterSetManager.storeAPS(aps, nalu.getBitstream().getFifo());
}
#if JVET_P0288_PIC_OUTPUT
bool DecLib::decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay, int iTargetOlsIdx)
#else
bool DecLib::decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay)
#endif
{
  bool ret;
  // ignore all NAL units of layers > 0

  m_accessUnitNals.push_back( std::pair<NalUnitType, int>( nalu.m_nalUnitType, nalu.m_temporalId ) );

  switch (nalu.m_nalUnitType)
  {
    case NAL_UNIT_VPS:
      xDecodeVPS( nalu );
#if JVET_P0288_PIC_OUTPUT
      m_vps->m_targetOlsIdx = iTargetOlsIdx;
#endif
      return false;
#if JVET_Q0117_PARAMETER_SETS_CLEANUP
    case NAL_UNIT_DCI:
      xDecodeDCI( nalu );
      return false;
#else
    case NAL_UNIT_DPS:
      xDecodeDPS( nalu );
      return false;
#endif
    case NAL_UNIT_SPS:
      xDecodeSPS( nalu );
      return false;

    case NAL_UNIT_PPS:
      xDecodePPS( nalu );
      return false;

    case NAL_UNIT_PH:
      xDecodePicHeader(nalu);
      return !m_bFirstSliceInPicture;

    case NAL_UNIT_PREFIX_APS:
    case NAL_UNIT_SUFFIX_APS:
      xDecodeAPS(nalu);
      return false;

    case NAL_UNIT_PREFIX_SEI:
      // Buffer up prefix SEI messages until SPS of associated VCL is known.
      m_prefixSEINALUs.push_back(new InputNALUnit(nalu));
      return false;

    case NAL_UNIT_SUFFIX_SEI:
      if (m_pcPic)
      {
#if JVET_P0125_ASPECT_TID_LAYER_ID_NUH
        m_accessUnitSeiTids.push_back(nalu.m_temporalId);
#endif
        m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_pcPic->SEIs, nalu.m_nalUnitType, nalu.m_temporalId, m_parameterSetManager.getActiveSPS(), m_HRD, m_pDecodedSEIOutputStream );
      }
      else
      {
        msg( NOTICE, "Note: received suffix SEI but no picture currently active.\n");
      }
      return false;

    case NAL_UNIT_CODED_SLICE_TRAIL:
    case NAL_UNIT_CODED_SLICE_STSA:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_GDR:
    case NAL_UNIT_CODED_SLICE_RADL:
    case NAL_UNIT_CODED_SLICE_RASL:
      ret = xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay);
      return ret;

    case NAL_UNIT_EOS:
      m_associatedIRAPType = NAL_UNIT_INVALID;
      m_pocCRA = 0;
      m_pocRandomAccess = MAX_INT;
      m_prevLayerID = MAX_INT;
      m_prevPOC = MAX_INT;
      m_prevSliceSkipped = false;
      m_skippedPOC = 0;
      return false;

    case NAL_UNIT_ACCESS_UNIT_DELIMITER:
      {
        AUDReader audReader;
        uint32_t picType;
        audReader.parseAccessUnitDelimiter(&(nalu.getBitstream()),picType);
        return !m_bFirstSliceInPicture;
      }

    case NAL_UNIT_EOB:
      return false;

    case NAL_UNIT_RESERVED_IRAP_VCL_11:
    case NAL_UNIT_RESERVED_IRAP_VCL_12:
      msg( NOTICE, "Note: found reserved VCL NAL unit.\n");
      xParsePrefixSEIsForUnknownVCLNal();
      return false;
    case NAL_UNIT_RESERVED_VCL_4:
    case NAL_UNIT_RESERVED_VCL_5:
    case NAL_UNIT_RESERVED_VCL_6:
    case NAL_UNIT_RESERVED_NVCL_26:
    case NAL_UNIT_RESERVED_NVCL_27:
      msg( NOTICE, "Note: found reserved NAL unit.\n");
      return false;
    case NAL_UNIT_UNSPECIFIED_28:
    case NAL_UNIT_UNSPECIFIED_29:
    case NAL_UNIT_UNSPECIFIED_30:
    case NAL_UNIT_UNSPECIFIED_31:
      msg( NOTICE, "Note: found unspecified NAL unit.\n");
      return false;
    default:
      THROW( "Invalid NAL unit type" );
      break;
  }

  return false;
}


/** Function for checking if picture should be skipped because of random access. This function checks the skipping of pictures in the case of -s option random access.
 *  All pictures prior to the random access point indicated by the counter iSkipFrame are skipped.
 *  It also checks the type of Nal unit type at the random access point.
 *  If the random access point is CRA/CRANT/BLA/BLANT, TFD pictures with POC less than the POC of the random access point are skipped.
 *  If the random access point is IDR all pictures after the random access point are decoded.
 *  If the random access point is none of the above, a warning is issues, and decoding of pictures with POC
 *  equal to or greater than the random access point POC is attempted. For non IDR/CRA/BLA random
 *  access point there is no guarantee that the decoder will not crash.
 */
bool DecLib::isRandomAccessSkipPicture( int& iSkipFrame, int& iPOCLastDisplay )
{
  if (iSkipFrame)
  {
    iSkipFrame--;   // decrement the counter
    return true;
  }
  else if ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )
  {
    m_pocRandomAccess = -MAX_INT; // no need to skip the reordered pictures in IDR, they are decodable.
  }
  else if (m_pocRandomAccess == MAX_INT) // start of random access point, m_pocRandomAccess has not been set yet.
  {
    if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->getPOC();
    }
    else
    {
      if(!m_warningMessageSkipPicture)
      {
        msg( WARNING, "\nWarning: this is not a valid random access point and the data is discarded until the first CRA picture");
        m_warningMessageSkipPicture = true;
      }
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->getPOC() < m_pocRandomAccess && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL))
  {
    iPOCLastDisplay++;
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false;
}

void DecLib::checkNalUnitConstraints( uint32_t naluType )
{
  if (m_parameterSetManager.getActiveSPS() != NULL && m_parameterSetManager.getActiveSPS()->getProfileTierLevel() != NULL)
  {
    const ConstraintInfo *cInfo = m_parameterSetManager.getActiveSPS()->getProfileTierLevel()->getConstraintInfo();
    xCheckNalUnitConstraintFlags( cInfo, naluType );
  }

#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  if (m_parameterSetManager.getActiveDPS() != NULL)
  {
    const DPS *dps = m_parameterSetManager.getActiveDPS();
    for (int i=0; i< dps->getNumPTLs(); i++)
    { 
      ProfileTierLevel ptl = dps->getProfileTierLevel(i);
      const ConstraintInfo *cInfo = ptl.getConstraintInfo();
      xCheckNalUnitConstraintFlags( cInfo, naluType );
    }
  }
#endif
}
void DecLib::xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType )
{
  if (cInfo != NULL)
  {
    CHECK(cInfo->getNoTrailConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_TRAIL,
      "Non-conforming bitstream. no_trail_constraint_flag is equal to 1 but bitstream contains NAL unit of type TRAIL_NUT.");
    CHECK(cInfo->getNoStsaConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_STSA,
      "Non-conforming bitstream. no_stsa_constraint_flag is equal to 1 but bitstream contains NAL unit of type STSA_NUT.");
    CHECK(cInfo->getNoRaslConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_RASL,
      "Non-conforming bitstream. no_rasl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RASL_NUT.");
    CHECK(cInfo->getNoRadlConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_RADL,
      "Non-conforming bitstream. no_radl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RADL_NUT.");
    CHECK(cInfo->getNoIdrConstraintFlag() && (naluType == NAL_UNIT_CODED_SLICE_IDR_W_RADL),
      "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_W_RADL.");
    CHECK(cInfo->getNoIdrConstraintFlag() && (naluType == NAL_UNIT_CODED_SLICE_IDR_N_LP),
      "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_N_LP.");
    CHECK(cInfo->getNoCraConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_CRA,
      "Non-conforming bitstream. no_cra_constraint_flag is equal to 1 but bitstream contains NAL unit of type CRA_NUT.");
    CHECK(cInfo->getNoGdrConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_GDR,
      "Non-conforming bitstream. no_gdr_constraint_flag is equal to 1 but bitstream contains NAL unit of type GDR_NUT.");
    CHECK(cInfo->getNoApsConstraintFlag() && naluType == NAL_UNIT_PREFIX_APS,
      "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_PREFIX_NUT.");
    CHECK(cInfo->getNoApsConstraintFlag() && naluType == NAL_UNIT_SUFFIX_APS,
      "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_SUFFIX_NUT.");
  }
}

//! \}
