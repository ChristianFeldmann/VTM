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

/** \file     EncLib.cpp
    \brief    encoder class
*/

#include "EncLib.h"

#include "EncModeCtrl.h"
#include "AQp.h"
#include "EncCu.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/ChromaFormat.h"
#if ENABLE_SPLIT_PARALLELISM
#include <omp.h>
#endif

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================



EncLib::EncLib()
  : m_spsMap( MAX_NUM_SPS )
  , m_ppsMap( MAX_NUM_PPS )
  , m_apsMap( MAX_NUM_APS )
  , m_AUWriterIf( nullptr )
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
{
  m_iPOCLast          = -1;
  m_iNumPicRcvd       =  0;
  m_uiNumAllPicCoded  =  0;

  m_iMaxRefPicNum     = 0;

#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif

#if JVET_N0415_CTB_ALF
  memset(m_apss, 0, sizeof(m_apss));
#endif
}

EncLib::~EncLib()
{
}

void EncLib::create ()
{
  // initialize global variables
  initROM();
  TComHash::initBlockSizeToIndex();
  m_iPOCLast = m_compositeRefEnabled ? -2 : -1;
  // create processing unit classes
  m_cGOPEncoder.        create( );
  m_cSliceEncoder.      create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth );
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
#if ENABLE_SPLIT_PARALLELISM
  m_numCuEncStacks  = m_numSplitThreads == 1 ? 1 : NUM_RESERVERD_SPLIT_JOBS;
#else
  m_numCuEncStacks  = 1;
#endif
#if ENABLE_WPP_PARALLELISM
  m_numCuEncStacks *= ( m_numWppThreads + m_numWppExtraLines );
#endif

  m_cCuEncoder      = new EncCu              [m_numCuEncStacks];
  m_cInterSearch    = new InterSearch        [m_numCuEncStacks];
  m_cIntraSearch    = new IntraSearch        [m_numCuEncStacks];
  m_cTrQuant        = new TrQuant            [m_numCuEncStacks];
  m_CABACEncoder    = new CABACEncoder       [m_numCuEncStacks];
  m_cRdCost         = new RdCost             [m_numCuEncStacks];
  m_CtxCache        = new CtxCache           [m_numCuEncStacks];

  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cCuEncoder[jId].         create( this );
  }
#else
  m_cCuEncoder.         create( this );
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cInterSearch.cacheAssign( &m_cacheModel );
#endif
  const uint32_t widthInCtus   = (getSourceWidth()  + m_maxCUWidth  - 1)  / m_maxCUWidth;
  const uint32_t heightInCtus  = (getSourceHeight() + m_maxCUHeight - 1) / m_maxCUHeight;
  const uint32_t numCtuInFrame = widthInCtus * heightInCtus;

  if (m_bUseSAO)
  {
    m_cEncSAO.create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth, m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA], m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
    m_cEncSAO.createEncData(getSaoCtuBoundary(), numCtuInFrame);
  }

  m_cLoopFilter.create( m_maxTotalCUDepth );
  if ( !m_bLoopFilterDisable )
  {
    m_cLoopFilter.initEncPicYuvBuffer( m_chromaFormatIDC, getSourceWidth(), getSourceHeight() );
  }
  if( m_alf )
  {
#if JVET_N0242_NON_LINEAR_ALF
    m_cEncALF.create( this, getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth, m_bitDepth, m_inputBitDepth );
#else
    m_cEncALF.create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth, m_bitDepth, m_inputBitDepth );
#endif
  }

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  m_cReshaper = new EncReshape[m_numCuEncStacks];
#endif
  if (m_lumaReshapeEnable)
  {
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    for (int jId = 0; jId < m_numCuEncStacks; jId++)
    {
      m_cReshaper[jId].createEnc(getSourceWidth(), getSourceHeight(), m_maxCUWidth, m_maxCUHeight, m_bitDepth[COMPONENT_Y]);
    }
#else
    m_cReshaper.createEnc( getSourceWidth(), getSourceHeight(), m_maxCUWidth, m_maxCUHeight, m_bitDepth[COMPONENT_Y]);
#endif
  }
  if ( m_RCEnableRateControl )
  {
    m_cRateCtrl.init(m_framesToBeEncoded, m_RCTargetBitrate, (int)((double)m_iFrameRate / m_temporalSubsampleRatio + 0.5), m_iGOPSize, m_iSourceWidth, m_iSourceHeight,
      m_maxCUWidth, m_maxCUHeight, getBitDepth(CHANNEL_TYPE_LUMA), m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList);
  }

}

void EncLib::destroy ()
{
  // destroy processing unit classes
  m_cGOPEncoder.        destroy();
  m_cSliceEncoder.      destroy();
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cCuEncoder[jId].destroy();
  }
#else
  m_cCuEncoder.         destroy();
#endif
  if( m_alf )
  {
    m_cEncALF.destroy();
  }
  m_cEncSAO.            destroyEncData();
  m_cEncSAO.            destroy();
  m_cLoopFilter.        destroy();
  m_cRateCtrl.          destroy();
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for (int jId = 0; jId < m_numCuEncStacks; jId++)
  {
    m_cReshaper[jId].   destroy();
  }
#else
  m_cReshaper.          destroy();
#endif
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cInterSearch[jId].   destroy();
    m_cIntraSearch[jId].   destroy();
  }
#else
  m_cInterSearch.       destroy();
  m_cIntraSearch.       destroy();
#endif

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  delete[] m_cCuEncoder;
  delete[] m_cInterSearch;
  delete[] m_cIntraSearch;
  delete[] m_cTrQuant;
  delete[] m_CABACEncoder;
  delete[] m_cRdCost;
  delete[] m_CtxCache;
#endif




  // destroy ROM
  destroyROM();
  return;
}

void EncLib::init( bool isFieldCoding, AUWriterIf* auWriterIf )
{
  m_AUWriterIf = auWriterIf;

  SPS &sps0=*(m_spsMap.allocatePS(0)); // NOTE: implementations that use more than 1 SPS need to be aware of activation issues.
  PPS &pps0=*(m_ppsMap.allocatePS(0));
#if !JVET_N0415_CTB_ALF
  APS &aps0=*(m_apsMap.allocatePS(0));
#endif

  // initialize SPS
  xInitSPS(sps0);
#if HEVC_VPS
  xInitVPS(m_cVPS, sps0);
#endif

#if JVET_N0349_DPS
  int dpsId = getDecodingParameterSetEnabled() ? 1 : 0;
  xInitDPS(m_dps, sps0, dpsId);
  sps0.setDecodingParameterSetId(m_dps.getDecodingParameterSetId());
    
#endif
#if ENABLE_SPLIT_PARALLELISM
  if( omp_get_dynamic() )
  {
    omp_set_dynamic( false );
  }
  omp_set_nested( true );
#endif

  if (getUseCompositeRef())
  {
    sps0.setLongTermRefsPresent(true);
  }

#if U0132_TARGET_BITS_SATURATION
  if (m_RCCpbSaturationEnabled)
  {
    m_cRateCtrl.initHrdParam(sps0.getVuiParameters()->getHrdParameters(), m_iFrameRate, m_RCInitialCpbFullness);
  }
#endif
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cRdCost[jId].setCostMode ( m_costMode );
  }
#else
  m_cRdCost.setCostMode ( m_costMode );
#endif

  // initialize PPS
  xInitPPS(pps0, sps0);
  // initialize APS
#if !JVET_N0415_CTB_ALF
  xInitAPS(aps0);
#endif
  xInitRPS(sps0, isFieldCoding);

#if ER_CHROMA_QP_WCG_PPS
  if (m_wcgChromaQpControl.isEnabled())
  {
    PPS &pps1=*(m_ppsMap.allocatePS(1));
    xInitPPS(pps1, sps0);
  }
#endif
  if (getUseCompositeRef())
  {
    PPS &pps2 = *(m_ppsMap.allocatePS(2));
    xInitPPS(pps2, sps0);
    xInitPPSforLT(pps2);
  }

  // initialize processing unit classes
  m_cGOPEncoder.  init( this );
  m_cSliceEncoder.init( this, sps0 );
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    // precache a few objects
    for( int i = 0; i < 10; i++ )
    {
      auto x = m_CtxCache[jId].get();
      m_CtxCache[jId].cache( x );
    }

    m_cCuEncoder[jId].init( this, sps0, jId );

    // initialize transform & quantization class
    m_cTrQuant[jId].init( jId == 0 ? nullptr : m_cTrQuant[0].getQuant(),
#if MAX_TB_SIZE_SIGNALLING
                          1 << m_log2MaxTbSize,

#else
                          MAX_TB_SIZEY,
#endif
                          m_useRDOQ,
                          m_useRDOQTS,
#if T0196_SELECTIVE_RDOQ
                          m_useSelectiveRDOQ,
#endif
                          true,
                          m_useTransformSkipFast
    );

    // initialize encoder search class
    CABACWriter* cabacEstimator = m_CABACEncoder[jId].getCABACEstimator( &sps0 );
    m_cIntraSearch[jId].init( this,
                              &m_cTrQuant[jId],
                              &m_cRdCost[jId],
                              cabacEstimator,
                              getCtxCache( jId ), m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth
                            , &m_cReshaper[jId]
    );
    m_cInterSearch[jId].init( this,
                              &m_cTrQuant[jId],
                              m_iSearchRange,
                              m_bipredSearchRange,
                              m_motionEstimationSearchMethod,
                              getUseCompositeRef(),
                              m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth, &m_cRdCost[jId], cabacEstimator, getCtxCache( jId )
                           , &m_cReshaper[jId]
    );

    // link temporary buffets from intra search with inter search to avoid unnecessary memory overhead
    m_cInterSearch[jId].setTempBuffers( m_cIntraSearch[jId].getSplitCSBuf(), m_cIntraSearch[jId].getFullCSBuf(), m_cIntraSearch[jId].getSaveCSBuf() );
  }
#else  // ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  m_cCuEncoder.   init( this, sps0 );

  // initialize transform & quantization class
  m_cTrQuant.init( nullptr,
#if MAX_TB_SIZE_SIGNALLING
                   1 << m_log2MaxTbSize,
#else
                   MAX_TB_SIZEY,
#endif
                   m_useRDOQ,
                   m_useRDOQTS,
#if T0196_SELECTIVE_RDOQ
                   m_useSelectiveRDOQ,
#endif
                   true,
                   m_useTransformSkipFast
  );

  // initialize encoder search class
  CABACWriter* cabacEstimator = m_CABACEncoder.getCABACEstimator(&sps0);
  m_cIntraSearch.init( this,
                       &m_cTrQuant,
                       &m_cRdCost,
                       cabacEstimator,
                       getCtxCache(), m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth
                     , &m_cReshaper
  );
  m_cInterSearch.init( this,
                       &m_cTrQuant,
                       m_iSearchRange,
                       m_bipredSearchRange,
                       m_motionEstimationSearchMethod,
                       getUseCompositeRef(),
    m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth, &m_cRdCost, cabacEstimator, getCtxCache()
                     , &m_cReshaper
  );

  // link temporary buffets from intra search with inter search to avoid unneccessary memory overhead
  m_cInterSearch.setTempBuffers( m_cIntraSearch.getSplitCSBuf(), m_cIntraSearch.getFullCSBuf(), m_cIntraSearch.getSaveCSBuf() );
#endif // ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

  m_iMaxRefPicNum = 0;

#if HEVC_USE_SCALING_LISTS
#if ER_CHROMA_QP_WCG_PPS
  if( m_wcgChromaQpControl.isEnabled() )
  {
    xInitScalingLists( sps0, *m_ppsMap.getPS(1) );
    xInitScalingLists( sps0, pps0 );
  }
  else
#endif
  {
    xInitScalingLists( sps0, pps0 );
  }
#endif
#if ENABLE_WPP_PARALLELISM
  m_entropyCodingSyncContextStateVec.resize( pps0.pcv->heightInCtus );
#endif
  if (getUseCompositeRef())
  {
    Picture *picBg = new Picture;
    picBg->create(sps0.getChromaFormatIdc(), Size(sps0.getPicWidthInLumaSamples(), sps0.getPicHeightInLumaSamples()), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false);
    picBg->getRecoBuf().fill(0);
#if JVET_N0415_CTB_ALF
    picBg->finalInit(sps0, pps0, m_apss);
#else
    picBg->finalInit(sps0, pps0, aps0);
#endif
    picBg->allocateNewSlice();
    picBg->createSpliceIdx(pps0.pcv->sizeInCtus);
    m_cGOPEncoder.setPicBg(picBg);
    Picture *picOrig = new Picture;
    picOrig->create(sps0.getChromaFormatIdc(), Size(sps0.getPicWidthInLumaSamples(), sps0.getPicHeightInLumaSamples()), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false);
    picOrig->getOrigBuf().fill(0);
    m_cGOPEncoder.setPicOrig(picOrig);
  }
}

#if HEVC_USE_SCALING_LISTS
void EncLib::xInitScalingLists(SPS &sps, PPS &pps)
{
  // Initialise scaling lists
  // The encoder will only use the SPS scaling lists. The PPS will never be marked present.
  const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE] =
  {
      sps.getMaxLog2TrDynamicRange(CHANNEL_TYPE_LUMA),
      sps.getMaxLog2TrDynamicRange(CHANNEL_TYPE_CHROMA)
  };

  Quant* quant = getTrQuant()->getQuant();

  if(getUseScalingListId() == SCALING_LIST_OFF)
  {
    quant->setFlatScalingList(maxLog2TrDynamicRange, sps.getBitDepths());
    quant->setUseScalingList(false);
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setFlatScalingList( maxLog2TrDynamicRange, sps.getBitDepths() );
      getTrQuant( jId )->getQuant()->setUseScalingList( false );
    }
#endif
    sps.setScalingListPresentFlag(false);
    pps.setScalingListPresentFlag(false);
  }
  else if(getUseScalingListId() == SCALING_LIST_DEFAULT)
  {
    sps.getScalingList().setDefaultScalingList ();
    sps.setScalingListPresentFlag(false);
    pps.setScalingListPresentFlag(false);

    quant->setScalingList(&(sps.getScalingList()), maxLog2TrDynamicRange, sps.getBitDepths());
    quant->setUseScalingList(true);
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setUseScalingList( true );
    }
#endif
  }
  else if(getUseScalingListId() == SCALING_LIST_FILE_READ)
  {
    sps.getScalingList().setDefaultScalingList ();
    if(sps.getScalingList().xParseScalingList(getScalingListFileName()))
    {
      THROW( "parse scaling list");
    }
    sps.getScalingList().checkDcOfMatrix();
    sps.setScalingListPresentFlag(sps.getScalingList().checkDefaultScalingList());
    pps.setScalingListPresentFlag(false);

    quant->setScalingList(&(sps.getScalingList()), maxLog2TrDynamicRange, sps.getBitDepths());
    quant->setUseScalingList(true);
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setUseScalingList( true );
    }
#endif
  }
  else
  {
    THROW("error : ScalingList == " << getUseScalingListId() << " not supported\n");
  }

  if (getUseScalingListId() != SCALING_LIST_OFF)
  {
    // Prepare delta's:
    for(uint32_t sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
    {
      const int predListStep = (sizeId == SCALING_LIST_32x32? (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) : 1); // if 32x32, skip over chroma entries.

      for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId+=predListStep)
      {
        sps.getScalingList().checkPredMode( sizeId, listId );
      }
    }
  }
}
#endif

void EncLib::xInitPPSforLT(PPS& pps)
{
  pps.setOutputFlagPresentFlag(true);
  pps.setDeblockingFilterControlPresentFlag(true);
  pps.setPPSDeblockingFilterDisabledFlag(true);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncLib::deletePicBuffer()
{
  PicList::iterator iterPic = m_cListPic.begin();
  int iSize = int( m_cListPic.size() );

  for ( int i = 0; i < iSize; i++ )
  {
    Picture* pcPic = *(iterPic++);

    pcPic->destroy();

    // get rid of the qpadaption layer
    while( pcPic->aqlayer.size() )
    {
      delete pcPic->aqlayer.back(); pcPic->aqlayer.pop_back();
    }

    delete pcPic;
    pcPic = NULL;
  }
}

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   pcPicYuvOrg         original YUV picture
 \param   pcPicYuvTrueOrg
 \param   snrCSC
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  accessUnitsOut      list of output access units
 \retval  iNumEncoded         number of encoded pictures
 */
void EncLib::encode( bool flush, PelStorage* pcPicYuvOrg, PelStorage* cPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut,
                     int& iNumEncoded )
{
  if (m_compositeRefEnabled && m_cGOPEncoder.getPicBg()->getSpliceFull() && m_iPOCLast >= 10 && m_iNumPicRcvd == 0 && m_cGOPEncoder.getEncodedLTRef() == false)
  {
    Picture* picCurr = NULL;
    xGetNewPicBuffer(rcListPicYuvRecOut, picCurr, 2);
    const PPS *pps = m_ppsMap.getPS(2);
    const SPS *sps = m_spsMap.getPS(pps->getSPSId());

    picCurr->M_BUFS(0, PIC_ORIGINAL).copyFrom(m_cGOPEncoder.getPicBg()->getRecoBuf());
#if JVET_N0415_CTB_ALF
    picCurr->finalInit(*sps, *pps, m_apss);
#else
    APS *aps = m_apsMap.getPS(0);
    picCurr->finalInit(*sps, *pps, *aps);
#endif
    picCurr->poc = m_iPOCLast - 1;
    m_iPOCLast -= 2;
    if (getUseAdaptiveQP())
    {
      AQpPreanalyzer::preanalyze(picCurr);
    }
    if (m_RCEnableRateControl)
    {
      m_cRateCtrl.initRCGOP(m_iNumPicRcvd);
    }
    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut,
      false, false, snrCSC, m_printFrameMSE, true);
    m_cGOPEncoder.setEncodedLTRef(true);
    if (m_RCEnableRateControl)
    {
      m_cRateCtrl.destroyRCGOP();
    }

    iNumEncoded = 0;
    m_iNumPicRcvd = 0;
  }
  //PROF_ACCUM_AND_START_NEW_SET( getProfilerPic(), P_GOP_LEVEL );
  if (pcPicYuvOrg != NULL)
  {
    // get original YUV
    Picture* pcPicCurr = NULL;

#if ER_CHROMA_QP_WCG_PPS
    int ppsID=-1; // Use default PPS ID
    if (getWCGChromaQPControl().isEnabled())
    {
      ppsID = getdQPs()[m_iPOCLast / (m_compositeRefEnabled ? 2 : 1) + 1];
      ppsID+=(getSwitchPOC() != -1 && (m_iPOCLast+1 >= getSwitchPOC())?1:0);
    }
    xGetNewPicBuffer( rcListPicYuvRecOut,
                      pcPicCurr, ppsID );
#else
    xGetNewPicBuffer( rcListPicYuvRecOut,
                      pcPicCurr, -1 ); // Uses default PPS ID. However, could be modified, for example, to use a PPS ID as a function of POC (m_iPOCLast+1)
#endif

    {
      const PPS *pPPS=(ppsID<0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsID);
      const SPS *pSPS=m_spsMap.getPS(pPPS->getSPSId());

      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL ).swap( *pcPicYuvOrg );
      pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL ).swap(*cPicYuvTrueOrg );
#if JVET_N0415_CTB_ALF
      pcPicCurr->finalInit(*pSPS, *pPPS, m_apss);
#else
      APS *pAPS = m_apsMap.getPS(0);
      pcPicCurr->finalInit(*pSPS, *pPPS, *pAPS);
#endif
    }

    pcPicCurr->poc = m_iPOCLast;

    // compute image characteristics
    if ( getUseAdaptiveQP() )
    {
      AQpPreanalyzer::preanalyze( pcPicCurr );
    }
  }

  if ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0)))
  {
    iNumEncoded = 0;
    return;
  }

  if ( m_RCEnableRateControl )
  {
    m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
  }

  // compress GOP
  m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut,
                            false, false, snrCSC, m_printFrameMSE
    , false
  );

  if ( m_RCEnableRateControl )
  {
    m_cRateCtrl.destroyRCGOP();
  }

  iNumEncoded         = m_iNumPicRcvd;
  m_iNumPicRcvd       = 0;
  m_uiNumAllPicCoded += iNumEncoded;
}

/**------------------------------------------------
 Separate interlaced frame into two fields
 -------------------------------------------------**/
void separateFields(Pel* org, Pel* dstField, uint32_t stride, uint32_t width, uint32_t height, bool isTop)
{
  if (!isTop)
  {
    org += stride;
  }
  for (int y = 0; y < height>>1; y++)
  {
    for (int x = 0; x < width; x++)
    {
      dstField[x] = org[x];
    }

    dstField += stride;
    org += stride*2;
  }

}

void EncLib::encode( bool flush, PelStorage* pcPicYuvOrg, PelStorage* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut,
                     int& iNumEncoded, bool isTff )
{
  iNumEncoded = 0;

  for (int fieldNum=0; fieldNum<2; fieldNum++)
  {
    if (pcPicYuvOrg)
    {
      /* -- field initialization -- */
      const bool isTopField=isTff==(fieldNum==0);

      Picture *pcField;
      xGetNewPicBuffer( rcListPicYuvRecOut, pcField, -1 );

      for (uint32_t comp = 0; comp < ::getNumberValidComponents(pcPicYuvOrg->chromaFormat); comp++)
      {
        const ComponentID compID = ComponentID(comp);
        {
          PelBuf compBuf = pcPicYuvOrg->get( compID );
          separateFields( compBuf.buf,
                         pcField->getOrigBuf().get(compID).buf,
                         compBuf.stride,
                         compBuf.width,
                         compBuf.height,
                         isTopField);
        }
      }

      {
        int ppsID=-1; // Use default PPS ID
        const PPS *pPPS=(ppsID<0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsID);
        const SPS *pSPS=m_spsMap.getPS(pPPS->getSPSId());
#if JVET_N0415_CTB_ALF
        pcField->finalInit(*pSPS, *pPPS, m_apss);
#else
        APS *pAPS = m_apsMap.getPS(0);
        pcField->finalInit(*pSPS, *pPPS, *pAPS);
#endif
      }

      pcField->poc = m_iPOCLast;
      pcField->reconstructed = false;

      pcField->setBorderExtension(false);// where is this normally?

      pcField->topField = isTopField;                  // interlaced requirement

      // compute image characteristics
      if ( getUseAdaptiveQP() )
      {
        AQpPreanalyzer::preanalyze( pcField );
      }
    }

    if ( m_iNumPicRcvd && ((flush&&fieldNum==1) || (m_iPOCLast/2)==0 || m_iNumPicRcvd==m_iGOPSize ) )
    {
      // compress GOP
      m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, true, isTff, snrCSC, m_printFrameMSE
                              , false
      );

      iNumEncoded += m_iNumPicRcvd;
      m_uiNumAllPicCoded += m_iNumPicRcvd;
      m_iNumPicRcvd = 0;
    }
  }
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \retval rpcPic obtained picture buffer
 */
void EncLib::xGetNewPicBuffer ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, int ppsId )
{
  // rotate he output buffer
  rcListPicYuvRecOut.push_back( rcListPicYuvRecOut.front() ); rcListPicYuvRecOut.pop_front();

  rpcPic=0;

  // At this point, the SPS and PPS can be considered activated - they are copied to the new Pic.
  const PPS *pPPS=(ppsId<0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsId);
  CHECK(!(pPPS!=0), "Unspecified error");
  const PPS &pps=*pPPS;

  const SPS *pSPS=m_spsMap.getPS(pps.getSPSId());
  CHECK(!(pSPS!=0), "Unspecified error");
  const SPS &sps=*pSPS;

  Slice::sortPicList(m_cListPic);

  // use an entry in the buffered list if the maximum number that need buffering has been reached:
  if (m_cListPic.size() >= (uint32_t)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2) )
  {
    PicList::iterator iterPic  = m_cListPic.begin();
    int iSize = int( m_cListPic.size() );
    for ( int i = 0; i < iSize; i++ )
    {
      rpcPic = *iterPic;
      if( ! rpcPic->referenced )
      {
        break;
      }
      iterPic++;
    }

    // If PPS ID is the same, we will assume that it has not changed since it was last used
    // and return the old object.
    if (pps.getPPSId() != rpcPic->cs->pps->getPPSId())
    {
      // the IDs differ - free up an entry in the list, and then create a new one, as with the case where the max buffering state has not been reached.
      rpcPic->destroy();
      delete rpcPic;
      m_cListPic.erase(iterPic);
      rpcPic=0;
    }
  }

  if (rpcPic==0)
  {
    rpcPic = new Picture;

    rpcPic->create( sps.getChromaFormatIdc(), Size( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples()), sps.getMaxCUWidth(), sps.getMaxCUWidth()+16, false );
    if ( getUseAdaptiveQP() )
    {
      const uint32_t iMaxDQPLayer = pps.getCuQpDeltaSubdiv()/2+1;
      rpcPic->aqlayer.resize( iMaxDQPLayer );
      for (uint32_t d = 0; d < iMaxDQPLayer; d++)
      {
        rpcPic->aqlayer[d] = new AQpLayer( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples(), sps.getMaxCUWidth()>>d, sps.getMaxCUHeight()>>d );
      }
    }

    m_cListPic.push_back( rpcPic );
  }

  rpcPic->setBorderExtension( false );
  rpcPic->reconstructed = false;
  rpcPic->referenced = true;
  rpcPic->getHashMap()->clearAll();

  m_iPOCLast += (m_compositeRefEnabled ? 2 : 1);
  m_iNumPicRcvd++;
}


#if HEVC_VPS
void EncLib::xInitVPS(VPS &vps, const SPS &sps)
{
  // The SPS must have already been set up.
  // set the VPS profile information.
  *vps.getPTL() = *sps.getPTL();
  vps.setMaxOpSets(1);
  vps.getTimingInfo()->setTimingInfoPresentFlag       ( false );
  vps.setNumHrdParameters( 0 );

  vps.createHrdParamBuffer();
  for( uint32_t i = 0; i < vps.getNumHrdParameters(); i ++ )
  {
    vps.setHrdOpSetIdx( 0, i );
    vps.setCprmsPresentFlag( false, i );
    // Set up HrdParameters here.
  }
}
#endif

#if JVET_N0349_DPS
void EncLib::xInitDPS(DPS &dps, const SPS &sps, const int dpsId)
{
  // The SPS must have already been set up.
  // set the DPS profile information.
  dps.setDecodingParameterSetId(dpsId);
  dps.setMaxSubLayersMinus1(sps.getMaxTLayers()-1);
  dps.setProfileTierLevel(*sps.getProfileTierLevel());
}
#endif


void EncLib::xInitSPS(SPS &sps)
{
#if !JVET_M0101_HLS
  sps.setIntraOnlyConstraintFlag(m_bIntraOnlyConstraintFlag);
  sps.setMaxBitDepthConstraintIdc(m_maxBitDepthConstraintIdc);
  sps.setMaxChromaFormatConstraintIdc(m_maxChromaFormatConstraintIdc);
  sps.setFrameConstraintFlag(m_frameOnlyConstraintFlag);
  sps.setNoQtbttDualTreeIntraConstraintFlag(m_bNoQtbttDualTreeIntraConstraintFlag);
  sps.setNoSaoConstraintFlag(m_bNoSaoConstraintFlag);
  sps.setNoAlfConstraintFlag(m_bNoAlfConstraintFlag);
  sps.setNoPcmConstraintFlag(m_bNoPcmConstraintFlag);
  sps.setNoRefWraparoundConstraintFlag(m_bNoRefWraparoundConstraintFlag);
  sps.setNoTemporalMvpConstraintFlag(m_bNoTemporalMvpConstraintFlag);
  sps.setNoSbtmvpConstraintFlag(m_bNoSbtmvpConstraintFlag);
  sps.setNoAmvrConstraintFlag(m_bNoAmvrConstraintFlag);
  sps.setNoBdofConstraintFlag(m_bNoBdofConstraintFlag);
  sps.setNoCclmConstraintFlag(m_bNoCclmConstraintFlag);
  sps.setNoMtsConstraintFlag(m_bNoMtsConstraintFlag);
  sps.setNoAffineMotionConstraintFlag(m_bNoAffineMotionConstraintFlag);
  sps.setNoGbiConstraintFlag(m_bNoGbiConstraintFlag);
  sps.setNoMhIntraConstraintFlag(m_bNoMhIntraConstraintFlag);
  sps.setNoTriangleConstraintFlag(m_bNoTriangleConstraintFlag);
  sps.setNoLadfConstraintFlag(m_bNoLadfConstraintFlag);
  sps.setNoCurrPicRefConstraintFlag(m_bNoCurrPicRefConstraintFlag);
  sps.setNoQpDeltaConstraintFlag(m_bNoQpDeltaConstraintFlag);
  sps.setNoDepQuantConstraintFlag(m_bNoDepQuantConstraintFlag);
  sps.setNoSignDataHidingConstraintFlag(m_bNoSignDataHidingConstraintFlag);

  ProfileTierLevel& profileTierLevel = *sps.getPTL()->getGeneralPTL();
  profileTierLevel.setLevelIdc                    (m_level);
  profileTierLevel.setTierFlag                    (m_levelTier);
  profileTierLevel.setProfileIdc                  (m_profile);
  profileTierLevel.setProfileCompatibilityFlag    (m_profile, 1);
  profileTierLevel.setProgressiveSourceFlag       (m_progressiveSourceFlag);
  profileTierLevel.setInterlacedSourceFlag        (m_interlacedSourceFlag);
  profileTierLevel.setNonPackedConstraintFlag     (m_nonPackedConstraintFlag);
  profileTierLevel.setFrameOnlyConstraintFlag     (m_frameOnlyConstraintFlag);
  profileTierLevel.setBitDepthConstraint          (m_bitDepthConstraintValue);
  profileTierLevel.setChromaFormatConstraint      (m_chromaFormatConstraintValue);
  profileTierLevel.setIntraConstraintFlag         (m_intraConstraintFlag);
  profileTierLevel.setOnePictureOnlyConstraintFlag(m_onePictureOnlyConstraintFlag);
  profileTierLevel.setLowerBitRateConstraintFlag  (m_lowerBitRateConstraintFlag);

  if ((m_profile == Profile::MAIN10) && (m_bitDepth[CHANNEL_TYPE_LUMA] == 8) && (m_bitDepth[CHANNEL_TYPE_CHROMA] == 8))
  {
    /* The above constraint is equal to Profile::MAIN */
    profileTierLevel.setProfileCompatibilityFlag(Profile::MAIN, 1);
  }
  if (m_profile == Profile::MAIN)
  {
    /* A Profile::MAIN10 decoder can always decode Profile::MAIN */
    profileTierLevel.setProfileCompatibilityFlag( Profile::MAIN10, 1 );
  }
#else
  ProfileTierLevel* profileTierLevel = sps.getProfileTierLevel();
  ConstraintInfo* cinfo = profileTierLevel->getConstraintInfo();
  cinfo->setProgressiveSourceFlag       (m_progressiveSourceFlag);
  cinfo->setInterlacedSourceFlag        (m_interlacedSourceFlag);
  cinfo->setNonPackedConstraintFlag     (m_nonPackedConstraintFlag);
  cinfo->setFrameOnlyConstraintFlag     (m_frameOnlyConstraintFlag);
  cinfo->setIntraOnlyConstraintFlag         (m_intraConstraintFlag);
  cinfo->setMaxBitDepthConstraintIdc    (m_maxBitDepthConstraintIdc);
  cinfo->setMaxChromaFormatConstraintIdc((ChromaFormat)m_maxChromaFormatConstraintIdc);
  cinfo->setNoQtbttDualTreeIntraConstraintFlag(m_bNoQtbttDualTreeIntraConstraintFlag);
  cinfo->setNoSaoConstraintFlag(m_bNoSaoConstraintFlag);
  cinfo->setNoAlfConstraintFlag(m_bNoAlfConstraintFlag);
  cinfo->setNoPcmConstraintFlag(m_bNoPcmConstraintFlag);
  cinfo->setNoRefWraparoundConstraintFlag(m_bNoRefWraparoundConstraintFlag);
  cinfo->setNoTemporalMvpConstraintFlag(m_bNoTemporalMvpConstraintFlag);
  cinfo->setNoSbtmvpConstraintFlag(m_bNoSbtmvpConstraintFlag);
  cinfo->setNoAmvrConstraintFlag(m_bNoAmvrConstraintFlag);
  cinfo->setNoBdofConstraintFlag(m_bNoBdofConstraintFlag);
  cinfo->setNoCclmConstraintFlag(m_bNoCclmConstraintFlag);
  cinfo->setNoMtsConstraintFlag(m_bNoMtsConstraintFlag);
  cinfo->setNoAffineMotionConstraintFlag(m_bNoAffineMotionConstraintFlag);
  cinfo->setNoGbiConstraintFlag(m_bNoGbiConstraintFlag);
  cinfo->setNoMhIntraConstraintFlag(m_bNoMhIntraConstraintFlag);
  cinfo->setNoTriangleConstraintFlag(m_bNoTriangleConstraintFlag);
  cinfo->setNoLadfConstraintFlag(m_bNoLadfConstraintFlag);
  cinfo->setNoCurrPicRefConstraintFlag(m_bNoCurrPicRefConstraintFlag);
  cinfo->setNoQpDeltaConstraintFlag(m_bNoQpDeltaConstraintFlag);
  cinfo->setNoDepQuantConstraintFlag(m_bNoDepQuantConstraintFlag);
  cinfo->setNoSignDataHidingConstraintFlag(m_bNoSignDataHidingConstraintFlag);

  profileTierLevel->setLevelIdc                    (m_level);
  profileTierLevel->setTierFlag                    (m_levelTier);
  profileTierLevel->setProfileIdc                  (m_profile);

#endif
  /* XXX: should Main be marked as compatible with still picture? */
  /* XXX: may be a good idea to refactor the above into a function
   * that chooses the actual compatibility based upon options */

  sps.setPicWidthInLumaSamples  ( m_iSourceWidth      );
  sps.setPicHeightInLumaSamples ( m_iSourceHeight     );
  sps.setConformanceWindow      ( m_conformanceWindow );
  sps.setMaxCUWidth             ( m_maxCUWidth        );
  sps.setMaxCUHeight            ( m_maxCUHeight       );
  sps.setMaxCodingDepth         ( m_maxTotalCUDepth   );
  sps.setChromaFormatIdc        ( m_chromaFormatIDC   );
  sps.setLog2DiffMaxMinCodingBlockSize(m_log2DiffMaxMinCodingBlockSize);

  sps.setCTUSize                             ( m_CTUSize );
  sps.setSplitConsOverrideEnabledFlag        ( m_useSplitConsOverride );
  sps.setMinQTSizes                          ( m_uiMinQT );
  sps.setMaxBTDepth                          ( m_uiMaxBTDepth, m_uiMaxBTDepthI, m_uiMaxBTDepthIChroma );
  sps.setUseDualITree                        ( m_dualITree );
#if JVET_N0193_LFNST
  sps.setUseLFNST                            ( m_LFNST );
#endif
  sps.setSBTMVPEnabledFlag                  ( m_SubPuMvpMode );
  sps.setAMVREnabledFlag                ( m_ImvMode != IMV_OFF );
  sps.setBDOFEnabledFlag                    ( m_BIO );
  sps.setUseAffine             ( m_Affine );
  sps.setUseAffineType         ( m_AffineType );
  sps.setUseLMChroma           ( m_LMChroma ? true : false );
  sps.setCclmCollocatedChromaFlag( m_cclmCollocatedChromaFlag );
  sps.setUseMTS                ( m_IntraMTS || m_InterMTS || m_ImplicitMTS );
  sps.setUseIntraMTS           ( m_IntraMTS );
  sps.setUseInterMTS           ( m_InterMTS );
  sps.setUseSBT                             ( m_SBT );
  if( sps.getUseSBT() )
  {
    sps.setMaxSbtSize                       ( m_iSourceWidth >= 1920 ? 64 : 32 );
  }
#if JVET_N0235_SMVD_SPS
  sps.setUseSMVD                ( m_SMVD );
#endif
  sps.setUseGBi                ( m_GBi );
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  sps.setLadfEnabled           ( m_LadfEnabled );
  if ( m_LadfEnabled )
  {
    sps.setLadfNumIntervals    ( m_LadfNumIntervals );
    for ( int k = 0; k < m_LadfNumIntervals; k++ )
    {
      sps.setLadfQpOffset( m_LadfQpOffset[k], k );
      sps.setLadfIntervalLowerBound( m_LadfIntervalLowerBound[k], k );
    }
    CHECK( m_LadfIntervalLowerBound[0] != 0, "abnormal value set to LadfIntervalLowerBound[0]" );
  }
#endif

  sps.setUseMHIntra            ( m_MHIntra );
  sps.setUseTriangle           ( m_Triangle );
#if JVET_N0127_MMVD_SPS_FLAG
  sps.setUseMMVD               ( m_MMVD );
  sps.setFpelMmvdEnabledFlag   (( m_MMVD ) ? m_allowDisFracMMVD : false);
#else
  sps.setFpelMmvdEnabledFlag             ( m_allowDisFracMMVD );
#endif
  sps.setAffineAmvrEnabledFlag              ( m_AffineAmvr );
  sps.setUseDMVR                            ( m_DMVR );

  sps.setIBCFlag                            ( m_IBCMode);
  sps.setWrapAroundEnabledFlag                      ( m_wrapAround );
  sps.setWrapAroundOffset                   ( m_wrapAroundOffset );
  // ADD_NEW_TOOL : (encoder lib) set tool enabling flags and associated parameters here
#if INCLUDE_ISP_CFG_FLAG
  sps.setUseISP                             ( m_ISP );
#endif
  sps.setUseReshaper                        ( m_lumaReshapeEnable );
#if JVET_N0217_MATRIX_INTRAPRED
  sps.setUseMIP                ( m_MIP );
#endif
  int minCUSize =  sps.getMaxCUWidth() >> sps.getLog2DiffMaxMinCodingBlockSize();
  int log2MinCUSize = 0;
  while(minCUSize > 1)
  {
    minCUSize >>= 1;
    log2MinCUSize++;
  }

  sps.setLog2MinCodingBlockSize(log2MinCUSize);

  sps.setPCMLog2MinSize (m_uiPCMLog2MinSize);
  sps.setPCMEnabledFlag        ( m_usePCM           );
  sps.setPCMLog2MaxSize( m_pcmLog2MaxSize  );

  sps.setSPSTemporalMVPEnabledFlag((getTMVPModeId() == 2 || getTMVPModeId() == 1));

#if MAX_TB_SIZE_SIGNALLING
  sps.setLog2MaxTbSize   ( m_log2MaxTbSize );
#endif

  for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    sps.setBitDepth      (ChannelType(channelType), m_bitDepth[channelType] );
    sps.setQpBDOffset  (ChannelType(channelType), (6 * (m_bitDepth[channelType] - 8)));
    sps.setPCMBitDepth (ChannelType(channelType), m_PCMBitDepth[channelType]         );
  }

  sps.setSAOEnabledFlag( m_bUseSAO );

  sps.setMaxTLayers( m_maxTempLayer );
  sps.setTemporalIdNestingFlag( ( m_maxTempLayer == 1 ) ? true : false );

  for (int i = 0; i < std::min(sps.getMaxTLayers(), (uint32_t) MAX_TLAYER); i++ )
  {
    sps.setMaxDecPicBuffering(m_maxDecPicBuffering[i], i);
    sps.setNumReorderPics(m_numReorderPics[i], i);
  }

  sps.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag );
#if HEVC_USE_SCALING_LISTS
  sps.setScalingListFlag ( (m_useScalingListId == SCALING_LIST_OFF) ? 0 : 1 );
#endif
#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  sps.setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );
#endif
  sps.setALFEnabledFlag( m_alf );
  sps.setVuiParametersPresentFlag(getVuiParametersPresentFlag());

  if (sps.getVuiParametersPresentFlag())
  {
    VUI* pcVUI = sps.getVuiParameters();
    pcVUI->setAspectRatioInfoPresentFlag(getAspectRatioInfoPresentFlag());
    pcVUI->setAspectRatioIdc(getAspectRatioIdc());
    pcVUI->setSarWidth(getSarWidth());
    pcVUI->setSarHeight(getSarHeight());
    pcVUI->setOverscanInfoPresentFlag(getOverscanInfoPresentFlag());
    pcVUI->setOverscanAppropriateFlag(getOverscanAppropriateFlag());
    pcVUI->setVideoSignalTypePresentFlag(getVideoSignalTypePresentFlag());
    pcVUI->setVideoFormat(getVideoFormat());
    pcVUI->setVideoFullRangeFlag(getVideoFullRangeFlag());
    pcVUI->setColourDescriptionPresentFlag(getColourDescriptionPresentFlag());
    pcVUI->setColourPrimaries(getColourPrimaries());
    pcVUI->setTransferCharacteristics(getTransferCharacteristics());
    pcVUI->setMatrixCoefficients(getMatrixCoefficients());
    pcVUI->setChromaLocInfoPresentFlag(getChromaLocInfoPresentFlag());
    pcVUI->setChromaSampleLocTypeTopField(getChromaSampleLocTypeTopField());
    pcVUI->setChromaSampleLocTypeBottomField(getChromaSampleLocTypeBottomField());
    pcVUI->setNeutralChromaIndicationFlag(getNeutralChromaIndicationFlag());
    pcVUI->setDefaultDisplayWindow(getDefaultDisplayWindow());
    pcVUI->setFrameFieldInfoPresentFlag(getFrameFieldInfoPresentFlag());
    pcVUI->setFieldSeqFlag(false);
    pcVUI->setHrdParametersPresentFlag(false);
    pcVUI->getTimingInfo()->setPocProportionalToTimingFlag(getPocProportionalToTimingFlag());
    pcVUI->getTimingInfo()->setNumTicksPocDiffOneMinus1   (getNumTicksPocDiffOneMinus1()   );
    pcVUI->setBitstreamRestrictionFlag(getBitstreamRestrictionFlag());
    pcVUI->setTilesFixedStructureFlag(getTilesFixedStructureFlag());
    pcVUI->setMotionVectorsOverPicBoundariesFlag(getMotionVectorsOverPicBoundariesFlag());
    pcVUI->setMinSpatialSegmentationIdc(getMinSpatialSegmentationIdc());
    pcVUI->setMaxBytesPerPicDenom(getMaxBytesPerPicDenom());
    pcVUI->setMaxBitsPerMinCuDenom(getMaxBitsPerMinCuDenom());
    pcVUI->setLog2MaxMvLengthHorizontal(getLog2MaxMvLengthHorizontal());
    pcVUI->setLog2MaxMvLengthVertical(getLog2MaxMvLengthVertical());
  }

  sps.setNumLongTermRefPicSPS(NUM_LONG_TERM_REF_PIC_SPS);
  CHECK(!(NUM_LONG_TERM_REF_PIC_SPS <= MAX_NUM_LONG_TERM_REF_PICS), "Unspecified error");
  for (int k = 0; k < NUM_LONG_TERM_REF_PIC_SPS; k++)
  {
    sps.setLtRefPicPocLsbSps(k, 0);
    sps.setUsedByCurrPicLtSPSFlag(k, 0);
  }

#if U0132_TARGET_BITS_SATURATION
  if( getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() || getCpbSaturationEnabled() )
#else
  if( getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() )
#endif
  {
    xInitHrdParameters(sps);
  }
  if( getBufferingPeriodSEIEnabled() || getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() )
  {
    sps.getVuiParameters()->setHrdParametersPresentFlag( true );
  }

  // Set up SPS range extension settings
  sps.getSpsRangeExtension().setTransformSkipRotationEnabledFlag(m_transformSkipRotationEnabledFlag);
  sps.getSpsRangeExtension().setTransformSkipContextEnabledFlag(m_transformSkipContextEnabledFlag);
  for (uint32_t signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    sps.getSpsRangeExtension().setRdpcmEnabledFlag(RDPCMSignallingMode(signallingModeIndex), m_rdpcmEnabledFlag[signallingModeIndex]);
  }
  sps.getSpsRangeExtension().setExtendedPrecisionProcessingFlag(m_extendedPrecisionProcessingFlag);
  sps.getSpsRangeExtension().setIntraSmoothingDisabledFlag( m_intraSmoothingDisabledFlag );
  sps.getSpsRangeExtension().setHighPrecisionOffsetsEnabledFlag(m_highPrecisionOffsetsEnabledFlag);
  sps.getSpsRangeExtension().setPersistentRiceAdaptationEnabledFlag(m_persistentRiceAdaptationEnabledFlag);
  sps.getSpsRangeExtension().setCabacBypassAlignmentEnabledFlag(m_cabacBypassAlignmentEnabledFlag);
}

void EncLib::xInitHrdParameters(SPS &sps)
{
  m_encHRD.initHRDParameters((EncCfg*) this);
  
  VUI *vui = sps.getVuiParameters();
  
  HRDParameters *hrdParams = vui->getHrdParameters();
  *hrdParams = m_encHRD.getHRDParameters();

  TimingInfo *timingInfo = vui->getTimingInfo();
  *timingInfo = m_encHRD.getTimingInfo();
}

void EncLib::xInitPPS(PPS &pps, const SPS &sps)
{
  // pps ID already initialised.
  pps.setSPSId(sps.getSPSId());

  pps.setConstrainedIntraPred( m_bUseConstrainedIntraPred );
  bool bUseDQP = (getCuQpDeltaSubdiv() > 0)? true : false;

  if((getMaxDeltaQP() != 0 )|| getUseAdaptiveQP())
  {
    bUseDQP = true;
  }

#if SHARP_LUMA_DELTA_QP
  if ( getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    bUseDQP = true;
  }
#endif
#if ENABLE_QPA
  if (getUsePerceptQPA() && !bUseDQP)
  {
    CHECK( m_cuQpDeltaSubdiv != 0, "max. delta-QP subdiv must be zero!" );
    bUseDQP = (getBaseQP() < 38) && (getSourceWidth() > 512 || getSourceHeight() > 320);
  }
#endif

  if (m_costMode==COST_SEQUENCE_LEVEL_LOSSLESS || m_costMode==COST_LOSSLESS_CODING)
  {
    bUseDQP=false;
  }


  if ( m_RCEnableRateControl )
  {
    pps.setUseDQP(true);
    pps.setCuQpDeltaSubdiv( 0 );
  }
  else if(bUseDQP)
  {
    pps.setUseDQP(true);
    pps.setCuQpDeltaSubdiv( m_cuQpDeltaSubdiv );
  }
  else
  {
    pps.setUseDQP(false);
    pps.setCuQpDeltaSubdiv( 0 );
  }

  if ( m_cuChromaQpOffsetSubdiv >= 0 )
  {
    pps.getPpsRangeExtension().setCuChromaQpOffsetSubdiv(m_cuChromaQpOffsetSubdiv);
    pps.getPpsRangeExtension().clearChromaQpOffsetList();
    pps.getPpsRangeExtension().setChromaQpOffsetListEntry(1, 6, 6);
    /* todo, insert table entries from command line (NB, 0 should not be touched) */
  }
  else
  {
    pps.getPpsRangeExtension().setCuChromaQpOffsetSubdiv(0);
    pps.getPpsRangeExtension().clearChromaQpOffsetList();
  }
  pps.getPpsRangeExtension().setCrossComponentPredictionEnabledFlag(m_crossComponentPredictionEnabledFlag);
  pps.getPpsRangeExtension().setLog2SaoOffsetScale(CHANNEL_TYPE_LUMA,   m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA  ]);
  pps.getPpsRangeExtension().setLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA, m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA]);

  {
    int baseQp = 26;
    if( 16 == getGOPSize() )
    {
      baseQp = getBaseQP()-24;
    }
    else
    {
      baseQp = getBaseQP()-26;
    }
    const int maxDQP = 37;
    const int minDQP = -26 + sps.getQpBDOffset(CHANNEL_TYPE_LUMA);

    pps.setPicInitQPMinus26( std::min( maxDQP, std::max( minDQP, baseQp ) ));
  }

#if ER_CHROMA_QP_WCG_PPS
  if (getWCGChromaQPControl().isEnabled())
  {
    const int baseQp=m_iQP+pps.getPPSId();
    const double chromaQp = m_wcgChromaQpControl.chromaQpScale * baseQp + m_wcgChromaQpControl.chromaQpOffset;
    const double dcbQP = m_wcgChromaQpControl.chromaCbQpScale * chromaQp;
    const double dcrQP = m_wcgChromaQpControl.chromaCrQpScale * chromaQp;
    const int cbQP =(int)(dcbQP + ( dcbQP < 0 ? -0.5 : 0.5) );
    const int crQP =(int)(dcrQP + ( dcrQP < 0 ? -0.5 : 0.5) );
    pps.setQpOffset(COMPONENT_Cb, Clip3( -12, 12, min(0, cbQP) + m_chromaCbQpOffset ));
    pps.setQpOffset(COMPONENT_Cr, Clip3( -12, 12, min(0, crQP) + m_chromaCrQpOffset));
#if JVET_N0054_JOINT_CHROMA
    pps.setQpOffset(JOINT_CbCr,   Clip3( -12, 12, ( min(0, cbQP) + min(0, crQP) ) / 2 + m_chromaCbCrQpOffset));
#endif
  }
  else
  {
#endif
  pps.setQpOffset(COMPONENT_Cb, m_chromaCbQpOffset );
  pps.setQpOffset(COMPONENT_Cr, m_chromaCrQpOffset );
#if JVET_N0054_JOINT_CHROMA
  pps.setQpOffset(JOINT_CbCr, m_chromaCbCrQpOffset );
#endif
#if ER_CHROMA_QP_WCG_PPS
  }
#endif
#if W0038_CQP_ADJ
  bool bChromaDeltaQPEnabled = false;
  {
    bChromaDeltaQPEnabled = ( m_sliceChromaQpOffsetIntraOrPeriodic[0] || m_sliceChromaQpOffsetIntraOrPeriodic[1] );
    if( !bChromaDeltaQPEnabled )
    {
      for( int i=0; i<m_iGOPSize; i++ )
      {
        if( m_GOPList[i].m_CbQPoffset || m_GOPList[i].m_CrQPoffset )
        {
          bChromaDeltaQPEnabled = true;
          break;
        }
      }
    }
  }
 #if ENABLE_QPA
  if ((getUsePerceptQPA() || getSliceChromaOffsetQpPeriodicity() > 0) && (getChromaFormatIdc() != CHROMA_400))
  {
    bChromaDeltaQPEnabled = true;
  }
 #endif
  pps.setSliceChromaQpFlag(bChromaDeltaQPEnabled);
#endif
  if (
    !pps.getSliceChromaQpFlag() && sps.getUseDualITree()
    && (getChromaFormatIdc() != CHROMA_400))
  {
#if JVET_N0054_JOINT_CHROMA
    pps.setSliceChromaQpFlag(m_chromaCbQpOffsetDualTree != 0 || m_chromaCrQpOffsetDualTree != 0 || m_chromaCbCrQpOffsetDualTree != 0);
#else
    pps.setSliceChromaQpFlag(m_chromaCbQpOffsetDualTree != 0 || m_chromaCrQpOffsetDualTree != 0);
#endif
  }

  pps.setEntropyCodingSyncEnabledFlag( m_entropyCodingSyncEnabledFlag );

#if JVET_N0857_TILES_BRICKS
  pps.setSingleTileInPicFlag((m_iNumColumnsMinus1 == 0 && m_iNumRowsMinus1 == 0));
#else
  pps.setTilesEnabledFlag( (m_iNumColumnsMinus1 > 0 || m_iNumRowsMinus1 > 0) );
#endif

  pps.setUseWP( m_useWeightedPred );
  pps.setWPBiPred( m_useWeightedBiPred );
  pps.setOutputFlagPresentFlag( false );

  if ( getDeblockingFilterMetric() )
  {
    pps.setDeblockingFilterOverrideEnabledFlag(true);
    pps.setPPSDeblockingFilterDisabledFlag(false);
  }
  else
  {
    pps.setDeblockingFilterOverrideEnabledFlag( !getLoopFilterOffsetInPPS() );
    pps.setPPSDeblockingFilterDisabledFlag( getLoopFilterDisable() );
  }

  if (! pps.getPPSDeblockingFilterDisabledFlag())
  {
    pps.setDeblockingFilterBetaOffsetDiv2( getLoopFilterBetaOffset() );
    pps.setDeblockingFilterTcOffsetDiv2( getLoopFilterTcOffset() );
  }
  else
  {
    pps.setDeblockingFilterBetaOffsetDiv2(0);
    pps.setDeblockingFilterTcOffsetDiv2(0);
  }

  // deblockingFilterControlPresentFlag is true if any of the settings differ from the inferred values:
  const bool deblockingFilterControlPresentFlag = pps.getDeblockingFilterOverrideEnabledFlag() ||
                                                  pps.getPPSDeblockingFilterDisabledFlag()     ||
                                                  pps.getDeblockingFilterBetaOffsetDiv2() != 0 ||
                                                  pps.getDeblockingFilterTcOffsetDiv2() != 0;

  pps.setDeblockingFilterControlPresentFlag(deblockingFilterControlPresentFlag);

  pps.setLog2ParallelMergeLevelMinus2   (m_log2ParallelMergeLevelMinus2 );
  pps.setCabacInitPresentFlag(CABAC_INIT_PRESENT_FLAG);
  pps.setLoopFilterAcrossSlicesEnabledFlag( m_bLFCrossSliceBoundaryFlag );


  int histogram[MAX_NUM_REF + 1];
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    histogram[i]=0;
  }
  for( int i = 0; i < getGOPSize(); i++)
  {
    CHECK(!(getGOPEntry(i).m_numRefPicsActive >= 0 && getGOPEntry(i).m_numRefPicsActive <= MAX_NUM_REF), "Unspecified error");
    histogram[getGOPEntry(i).m_numRefPicsActive]++;
  }

  int maxHist=-1;
  int bestPos=0;
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    if(histogram[i]>maxHist)
    {
      maxHist=histogram[i];
      bestPos=i;
    }
  }
  CHECK(!(bestPos <= 15), "Unspecified error");
    pps.setNumRefIdxL0DefaultActive(bestPos);
  pps.setNumRefIdxL1DefaultActive(bestPos);
  pps.setTransquantBypassEnabledFlag(getTransquantBypassEnabledFlag());
  pps.setUseTransformSkip( m_useTransformSkip );
  pps.getPpsRangeExtension().setLog2MaxTransformSkipBlockSize( m_log2MaxTransformSkipBlockSize  );


  xInitPPSforTiles(pps);

#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  pps.setLoopFilterAcrossVirtualBoundariesDisabledFlag( m_loopFilterAcrossVirtualBoundariesDisabledFlag );
  pps.setNumVerVirtualBoundaries            ( m_numVerVirtualBoundaries );
  pps.setNumHorVirtualBoundaries            ( m_numHorVirtualBoundaries );
  for( unsigned int i = 0; i < m_numVerVirtualBoundaries; i++ )
  {
    pps.setVirtualBoundariesPosX            ( m_virtualBoundariesPosX[i], i );
  }
  for( unsigned int i = 0; i < m_numHorVirtualBoundaries; i++ )
  {
    pps.setVirtualBoundariesPosY            ( m_virtualBoundariesPosY[i], i );
  }
#endif

  pps.pcv = new PreCalcValues( sps, pps, true );
}

void EncLib::xInitAPS(APS &aps)
{
  //Do nothing now
}
//Function for initializing m_RPSList, a list of ReferencePictureSet, based on the GOPEntry objects read from the config file.
void EncLib::xInitRPS(SPS &sps, bool isFieldCoding)
{
  ReferencePictureSet*      rps;

  sps.createRPSList(getGOPSize() + m_extraRPSs + 1);
  RPSList* rpsList = sps.getRPSList();

  for( int i = 0; i < getGOPSize()+m_extraRPSs; i++)
  {
    const GOPEntry &ge = getGOPEntry(i);
    rps = rpsList->getReferencePictureSet(i);
    rps->setNumberOfPictures(ge.m_numRefPics);
    rps->setNumRefIdc(ge.m_numRefIdc);
    int numNeg = 0;
    int numPos = 0;
    for( int j = 0; j < ge.m_numRefPics; j++)
    {
      rps->setDeltaPOC(j,ge.m_referencePics[j]);
      rps->setUsed(j,ge.m_usedByCurrPic[j]);
      if(ge.m_referencePics[j]>0)
      {
        numPos++;
      }
      else
      {
        numNeg++;
      }
    }
    rps->setNumberOfNegativePictures(numNeg);
    rps->setNumberOfPositivePictures(numPos);

    // handle inter RPS intialization from the config file.
    rps->setInterRPSPrediction(ge.m_interRPSPrediction > 0);  // not very clean, converting anything > 0 to true.
    rps->setDeltaRIdxMinus1(0);                               // index to the Reference RPS is always the previous one.
    ReferencePictureSet*     RPSRef = i>0 ? rpsList->getReferencePictureSet(i-1): NULL;  // get the reference RPS

    if (ge.m_interRPSPrediction == 2)  // Automatic generation of the inter RPS idc based on the RIdx provided.
    {
      CHECK(!(RPSRef!=NULL), "Unspecified error");
      int deltaRPS = getGOPEntry(i-1).m_POC - ge.m_POC;  // the ref POC - current POC
      int numRefDeltaPOC = RPSRef->getNumberOfPictures();

      rps->setDeltaRPS(deltaRPS);           // set delta RPS
      rps->setNumRefIdc(numRefDeltaPOC+1);  // set the numRefIdc to the number of pictures in the reference RPS + 1.
      int count=0;
      for (int j = 0; j <= numRefDeltaPOC; j++ ) // cycle through pics in reference RPS.
      {
        int RefDeltaPOC = (j<numRefDeltaPOC)? RPSRef->getDeltaPOC(j): 0;  // if it is the last decoded picture, set RefDeltaPOC = 0
        rps->setRefIdc(j, 0);
        for (int k = 0; k < rps->getNumberOfPictures(); k++ )  // cycle through pics in current RPS.
        {
          if (rps->getDeltaPOC(k) == ( RefDeltaPOC + deltaRPS))  // if the current RPS has a same picture as the reference RPS.
          {
              rps->setRefIdc(j, (rps->getUsed(k)?1:2));
              count++;
              break;
          }
        }
      }
      if (count != rps->getNumberOfPictures())
      {
        msg( WARNING, "Warning: Unable fully predict all delta POCs using the reference RPS index given in the config file.  Setting Inter RPS to false for this RPS.\n");
        rps->setInterRPSPrediction(0);
      }
    }
    else if (ge.m_interRPSPrediction == 1)  // inter RPS idc based on the RefIdc values provided in config file.
    {
      CHECK(!(RPSRef!=NULL), "Unspecified error");
      rps->setDeltaRPS(ge.m_deltaRPS);
      rps->setNumRefIdc(ge.m_numRefIdc);
      for (int j = 0; j < ge.m_numRefIdc; j++ )
      {
        rps->setRefIdc(j, ge.m_refIdc[j]);
      }
      // the following code overwrite the deltaPOC and Used by current values read from the config file with the ones
      // computed from the RefIdc.  A warning is printed if they are not identical.
      numNeg = 0;
      numPos = 0;
      ReferencePictureSet      RPSTemp;  // temporary variable

      for (int j = 0; j < ge.m_numRefIdc; j++ )
      {
        if (ge.m_refIdc[j])
        {
          int deltaPOC = ge.m_deltaRPS + ((j < RPSRef->getNumberOfPictures())? RPSRef->getDeltaPOC(j) : 0);
          RPSTemp.setDeltaPOC((numNeg+numPos),deltaPOC);
          RPSTemp.setUsed((numNeg+numPos),ge.m_refIdc[j]==1?1:0);
          if (deltaPOC<0)
          {
            numNeg++;
          }
          else
          {
            numPos++;
          }
        }
      }
      if (numNeg != rps->getNumberOfNegativePictures())
      {
        msg( WARNING, "Warning: number of negative pictures in RPS is different between intra and inter RPS specified in the config file.\n");
        rps->setNumberOfNegativePictures(numNeg);
        rps->setNumberOfPictures(numNeg+numPos);
      }
      if (numPos != rps->getNumberOfPositivePictures())
      {
        msg( WARNING, "Warning: number of positive pictures in RPS is different between intra and inter RPS specified in the config file.\n");
        rps->setNumberOfPositivePictures(numPos);
        rps->setNumberOfPictures(numNeg+numPos);
      }
      RPSTemp.setNumberOfPictures(numNeg+numPos);
      RPSTemp.setNumberOfNegativePictures(numNeg);
      RPSTemp.sortDeltaPOC();     // sort the created delta POC before comparing
      // check if Delta POC and Used are the same
      // print warning if they are not.
      for (int j = 0; j < ge.m_numRefIdc; j++ )
      {
        if (RPSTemp.getDeltaPOC(j) != rps->getDeltaPOC(j))
        {
          msg( WARNING, "Warning: delta POC is different between intra RPS and inter RPS specified in the config file.\n");
          rps->setDeltaPOC(j,RPSTemp.getDeltaPOC(j));
        }
        if (RPSTemp.getUsed(j) != rps->getUsed(j))
        {
          msg( WARNING, "Warning: Used by Current in RPS is different between intra and inter RPS specified in the config file.\n");
          rps->setUsed(j,RPSTemp.getUsed(j));
        }
      }
    }
  }
  //In case of field coding, we need to set special parameters for the first bottom field of the sequence, since it is not specified in the cfg file.
  //The position = GOPSize + extraRPSs which is (a priori) unused is reserved for this field in the RPS.
  if (isFieldCoding)
  {
    rps = rpsList->getReferencePictureSet(getGOPSize()+m_extraRPSs);
    rps->setNumberOfPictures(1);
    rps->setNumberOfNegativePictures(1);
    rps->setNumberOfPositivePictures(0);
    rps->setNumberOfLongtermPictures(0);
    rps->setDeltaPOC(0,-1);
    rps->setPOC(0,0);
    rps->setUsed(0,true);
    rps->setInterRPSPrediction(false);
    rps->setDeltaRIdxMinus1(0);
    rps->setDeltaRPS(0);
    rps->setNumRefIdc(0);
  }
}

   // This is a function that
   // determines what Reference Picture Set to use
   // for a specific slice (with POC = POCCurr)
void EncLib::selectReferencePictureSet(Slice* slice, int POCCurr, int GOPid
                                      , int ltPoc
)
{
  bool isEncodeLtRef = (POCCurr == ltPoc);
  if (m_compositeRefEnabled && isEncodeLtRef)
  {
    POCCurr++;
  }
  int rIdx = GOPid;
  slice->setRPSidx(GOPid);

  for(int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
  {
    if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      int POCIndex = POCCurr%m_uiIntraPeriod;
      if(POCIndex == 0)
      {
        POCIndex = m_uiIntraPeriod;
      }
      if(POCIndex == m_GOPList[extraNum].m_POC)
      {
        slice->setRPSidx(extraNum);
        rIdx = extraNum;
      }
    }
    else
    {
      if(POCCurr==m_GOPList[extraNum].m_POC)
      {
        slice->setRPSidx(extraNum);
        rIdx = extraNum;
      }
    }
  }

  if(POCCurr == 1 && slice->getPic()->fieldPic)
  {
    slice->setRPSidx(m_iGOPSize+m_extraRPSs);
    rIdx = m_iGOPSize + m_extraRPSs;
  }

  ReferencePictureSet *rps = const_cast<ReferencePictureSet *>(slice->getSPS()->getRPSList()->getReferencePictureSet(slice->getRPSidx()));
  if (m_compositeRefEnabled && ltPoc != -1 && !isEncodeLtRef)
  {
    if (ltPoc != -1 && rps->getNumberOfLongtermPictures() != 1 && !isEncodeLtRef)
    {
      int idx = rps->getNumberOfPictures();
      int maxPicOrderCntLSB = 1 << slice->getSPS()->getBitsForPOC();
      int ltPocLsb = ltPoc % maxPicOrderCntLSB;

      rps->setNumberOfPictures(rps->getNumberOfPictures() + 1);
      rps->setNumberOfLongtermPictures(1);
      rps->setPOC(idx, ltPoc);
      rps->setPocLSBLT(idx, ltPocLsb);
      rps->setDeltaPOC(idx, -POCCurr + ltPoc);
      rps->setUsed(idx, true);
    }
  }
  else if (m_compositeRefEnabled && isEncodeLtRef)
  {
    ReferencePictureSet* localRPS = slice->getLocalRPS();
    (*localRPS) = ReferencePictureSet();
    int refPics = rps->getNumberOfPictures();
    localRPS->setNumberOfPictures(rps->getNumberOfPictures());
    for (int i = 0; i < refPics; i++)
    {
      localRPS->setDeltaPOC(i, rps->getDeltaPOC(i) + 1);
      localRPS->setUsed(i, rps->getUsed(i));
    }
    localRPS->setNumberOfNegativePictures(rps->getNumberOfNegativePictures());
    localRPS->setNumberOfPositivePictures(rps->getNumberOfPositivePictures());
    localRPS->setInterRPSPrediction(true);
    int deltaRPS = 1;
    int newIdc = 0;
    for (int i = 0; i < refPics; i++)
    {
      int deltaPOC = ((i != refPics) ? rps->getDeltaPOC(i) : 0);  // check if the reference abs POC is >= 0
      int refIdc = 0;
      for (int j = 0; j < localRPS->getNumberOfPictures(); j++) // loop through the  pictures in the new RPS
      {
        if ((deltaPOC + deltaRPS) == localRPS->getDeltaPOC(j))
        {
          if (localRPS->getUsed(j))
          {
            refIdc = 1;
          }
          else
          {
            refIdc = 2;
          }
        }
      }
      localRPS->setRefIdc(i, refIdc);
      newIdc++;
    }
    localRPS->setNumRefIdc(newIdc + 1);
    localRPS->setRefIdc(newIdc, 0);
    localRPS->setDeltaRPS(deltaRPS);
    localRPS->setDeltaRIdxMinus1(slice->getSPS()->getRPSList()->getNumberOfReferencePictureSets() - 1 - rIdx);
    slice->setRPS(localRPS);
    slice->setRPSidx(-1);
    return;
  }
  slice->setRPS(rps);
}

int EncLib::getReferencePictureSetIdxForSOP(int POCCurr, int GOPid )
{
  int rpsIdx = GOPid;

  for(int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
  {
    if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      int POCIndex = POCCurr%m_uiIntraPeriod;
      if(POCIndex == 0)
      {
        POCIndex = m_uiIntraPeriod;
      }
      if(POCIndex == m_GOPList[extraNum].m_POC)
      {
        rpsIdx = extraNum;
      }
    }
    else
    {
      if(POCCurr==m_GOPList[extraNum].m_POC)
      {
        rpsIdx = extraNum;
      }
    }
  }

  return rpsIdx;
}

void  EncLib::xInitPPSforTiles(PPS &pps)
{
#if JVET_N0857_TILES_BRICKS
  if ( (m_iNumColumnsMinus1==0) && (m_iNumRowsMinus1==0) )
  {
    // one, no bricks
    pps.setSingleTileInPicFlag(true);
    pps.setSingleBrickPerSliceFlag(true);
    pps.setRectSliceFlag(true);
  }
  else
  {
    pps.setSingleTileInPicFlag(false);
    pps.setSingleBrickPerSliceFlag( m_sliceMode==SINGLE_BRICK_PER_SLICE );
    pps.setRectSliceFlag( m_sliceMode==SINGLE_BRICK_PER_SLICE );
  }
#endif
  pps.setUniformTileSpacingFlag( m_tileUniformSpacingFlag );
  pps.setNumTileColumnsMinus1( m_iNumColumnsMinus1 );
  pps.setNumTileRowsMinus1( m_iNumRowsMinus1 );
  if( !m_tileUniformSpacingFlag )
  {
    pps.setTileColumnWidth( m_tileColumnWidth );
    pps.setTileRowHeight( m_tileRowHeight );
  }
  pps.setLoopFilterAcrossBricksEnabledFlag( m_loopFilterAcrossBricksEnabledFlag );

#if JVET_N0857_TILES_BRICKS
  //pps.setRectSliceFlag( m_rectSliceFlag );
  pps.setNumSlicesInPicMinus1( m_numSlicesInPicMinus1 );
  pps.setTopLeftTileIdx( m_topLeftTileIdx );
  pps.setBottomRightTileIdx( m_bottomRightTileIdx );
  pps.setLoopFilterAcrossBricksEnabledFlag( m_loopFilterAcrossBricksEnabledFlag );
  pps.setLoopFilterAcrossSlicesEnabledFlag( m_loopFilterAcrossSlicesEnabledFlag );
  pps.setSignalledSliceIdFlag( m_signalledSliceIdFlag );
  pps.setSignalledSliceIdLengthMinus1( m_signalledSliceIdLengthMinus1 );
  pps.setSignalledSliceIdFlag( m_signalledSliceIdFlag );
  pps.setSignalledSliceIdLengthMinus1( m_signalledSliceIdLengthMinus1 );
  pps.setSliceId( m_sliceId );

  int numTiles= (m_iNumColumnsMinus1 + 1) * (m_iNumRowsMinus1 + 1);
  
  if (m_brickSplitMap.empty())
  {
    pps.setBrickSplittingPresentFlag(false);
  }
  else
  {
    pps.setBrickSplittingPresentFlag(true);

    std::vector<bool> brickSplitFlag (numTiles, false);
    std::vector<bool> uniformBrickSpacingFlag (numTiles, false);
    std::vector<int>  brickHeightMinus1 (numTiles, 0);
    std::vector<int>  numBrickRowsMinus1 (numTiles, 0);
    std::vector<std::vector<int>>  brickRowHeightMinus1 (numTiles);

    for (auto &brickSplit: m_brickSplitMap)
    {
      int tileIdx = brickSplit.first;
      CHECK ( tileIdx >= numTiles, "Brick split specified for undefined tile");

      brickSplitFlag[tileIdx]           = true;
      uniformBrickSpacingFlag [tileIdx] = brickSplit.second.m_uniformSplit;
      if (uniformBrickSpacingFlag [tileIdx])
      {
        brickHeightMinus1[tileIdx]=brickSplit.second.m_uniformHeight - 1;
      }
      else
      {
        numBrickRowsMinus1[tileIdx]=brickSplit.second.m_numSplits;
        brickRowHeightMinus1[tileIdx].resize(brickSplit.second.m_numSplits);
        for (int i=0; i<brickSplit.second.m_numSplits; i++)
        {
          brickRowHeightMinus1[tileIdx][i]=brickSplit.second.m_brickHeight[i] - 1;
        }
      }
    }
    pps.setBrickSplitFlag(brickSplitFlag);
    pps.setUniformBrickSpacingFlag(uniformBrickSpacingFlag);
    pps.setBrickHeightMinus1(brickHeightMinus1);
    pps.setNumBrickRowsMinus1(numBrickRowsMinus1);
    pps.setBrickRowHeightMinus1(brickRowHeightMinus1);

    // check brick dimensions
    std::vector<uint32_t> tileRowHeight (m_iNumRowsMinus1+1);
    int picHeightInCtus = (getSourceHeight() + m_maxCUHeight - 1) / m_maxCUHeight;

    // calculate all tile row heights
    if( pps.getUniformTileSpacingFlag() )
    {
      //set width and height for each (uniform) tile
      for(int row=0; row < m_iNumRowsMinus1 + 1; row++)
      {
        tileRowHeight[row] = (row+1)*picHeightInCtus/(m_iNumRowsMinus1+1)   - (row*picHeightInCtus)/(m_iNumRowsMinus1 + 1);
      }
    }
    else
    {
      tileRowHeight[ m_iNumRowsMinus1 ] = picHeightInCtus;
      for( int j = 0; j < m_iNumRowsMinus1; j++ ) 
      {
        tileRowHeight[ j ] = pps.getTileRowHeight( j );
        tileRowHeight[ m_iNumRowsMinus1 ]  =  tileRowHeight[ m_iNumRowsMinus1 ] - pps.getTileRowHeight( j );
      }
    }

    // check brick splits for each tile
    for (int tileIdx=0; tileIdx < numTiles; tileIdx++)
    {
      if (pps.getBrickSplitFlag(tileIdx))
      {
        const int tileY = tileIdx / (m_iNumColumnsMinus1+1);

        int tileHeight = tileRowHeight [tileY];

        if (pps.getUniformBrickSpacingFlag(tileIdx))
        {
          CHECK((pps.getBrickHeightMinus1(tileIdx) + 1) >= tileHeight, "Brick height larger than or equal to tile height");
        }
        else
        {
          int cumulativeHeight=0;
          for (int i = 0; i < pps.getNumBrickRowsMinus1(tileIdx); i++)
          {
            cumulativeHeight += pps.getBrickRowHeightMinus1(tileIdx, i) + 1;
          }
          CHECK(cumulativeHeight >= tileHeight, "Cumulative brick height larger than or equal to tile height");
        }
      }
    }
  }

#endif
}

void  EncCfg::xCheckGSParameters()
{
  int   iWidthInCU = ( m_iSourceWidth%m_maxCUWidth ) ? m_iSourceWidth/m_maxCUWidth + 1 : m_iSourceWidth/m_maxCUWidth;
  int   iHeightInCU = ( m_iSourceHeight%m_maxCUHeight ) ? m_iSourceHeight/m_maxCUHeight + 1 : m_iSourceHeight/m_maxCUHeight;
  uint32_t  uiCummulativeColumnWidth = 0;
  uint32_t  uiCummulativeRowHeight = 0;

  //check the column relative parameters
  if( m_iNumColumnsMinus1 >= (1<<(LOG2_MAX_NUM_COLUMNS_MINUS1+1)) )
  {
    EXIT( "The number of columns is larger than the maximum allowed number of columns." );
  }

  if( m_iNumColumnsMinus1 >= iWidthInCU )
  {
    EXIT( "The current picture can not have so many columns." );
  }

  if( m_iNumColumnsMinus1 && !m_tileUniformSpacingFlag )
  {
    for(int i=0; i<m_iNumColumnsMinus1; i++)
    {
      uiCummulativeColumnWidth += m_tileColumnWidth[i];
    }

    if( uiCummulativeColumnWidth >= iWidthInCU )
    {
      EXIT( "The width of the column is too large." );
    }
  }

  //check the row relative parameters
  if( m_iNumRowsMinus1 >= (1<<(LOG2_MAX_NUM_ROWS_MINUS1+1)) )
  {
    EXIT( "The number of rows is larger than the maximum allowed number of rows." );
  }

  if( m_iNumRowsMinus1 >= iHeightInCU )
  {
    EXIT( "The current picture can not have so many rows." );
  }

  if( m_iNumRowsMinus1 && !m_tileUniformSpacingFlag )
  {
    for(int i=0; i<m_iNumRowsMinus1; i++)
    {
      uiCummulativeRowHeight += m_tileRowHeight[i];
    }

    if( uiCummulativeRowHeight >= iHeightInCU )
    {
      EXIT( "The height of the row is too large." );
    }
  }
}

#if JCTVC_Y0038_PARAMS
void EncLib::setParamSetChanged(int spsId, int ppsId)
{
  m_ppsMap.setChangedFlag(ppsId);
  m_spsMap.setChangedFlag(spsId);
}
#endif
bool EncLib::APSNeedsWriting(int apsId)
{
  bool isChanged = m_apsMap.getChangedFlag(apsId);
  m_apsMap.clearChangedFlag(apsId);
  return isChanged;
}

bool EncLib::PPSNeedsWriting(int ppsId)
{
  bool bChanged=m_ppsMap.getChangedFlag(ppsId);
  m_ppsMap.clearChangedFlag(ppsId);
  return bChanged;
}

bool EncLib::SPSNeedsWriting(int spsId)
{
  bool bChanged=m_spsMap.getChangedFlag(spsId);
  m_spsMap.clearChangedFlag(spsId);
  return bChanged;
}

#if X0038_LAMBDA_FROM_QP_CAPABILITY
int EncCfg::getQPForPicture(const uint32_t gopIndex, const Slice *pSlice) const
{
  const int lumaQpBDOffset = pSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  int qp;

  if (getCostMode()==COST_LOSSLESS_CODING)
  {
    qp=LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
  }
  else
  {
    const SliceType sliceType=pSlice->getSliceType();

    qp = getBaseQP();

    // switch at specific qp and keep this qp offset
    static int appliedSwitchDQQ = 0; /* TODO: MT */
    if( pSlice->getPOC() == getSwitchPOC() )
    {
      appliedSwitchDQQ = getSwitchDQP();
    }
    qp += appliedSwitchDQQ;

#if QP_SWITCHING_FOR_PARALLEL
    const int* pdQPs = getdQPs();
    if ( pdQPs )
    {
      qp += pdQPs[pSlice->getPOC() / (m_compositeRefEnabled ? 2 : 1)];
    }
#endif

    if(sliceType==I_SLICE)
    {
      qp += getIntraQPOffset();
    }
    else
    {
#if SHARP_LUMA_DELTA_QP
      // Only adjust QP when not lossless
      if (!(( getMaxDeltaQP() == 0 ) && (!getLumaLevelToDeltaQPMapping().isEnabled()) && (qp == -lumaQpBDOffset ) && (pSlice->getPPS()->getTransquantBypassEnabledFlag())))
#else
      if (!(( getMaxDeltaQP() == 0 ) && (qp == -lumaQpBDOffset ) && (pSlice->getPPS()->getTransquantBypassEnabledFlag())))
#endif

      {
        const GOPEntry &gopEntry=getGOPEntry(gopIndex);
        // adjust QP according to the QP offset for the GOP entry.
        qp +=gopEntry.m_QPOffset;

        // adjust QP according to QPOffsetModel for the GOP entry.
        double dqpOffset=qp*gopEntry.m_QPOffsetModelScale+gopEntry.m_QPOffsetModelOffset+0.5;
        int qpOffset = (int)floor(Clip3<double>(0.0, 3.0, dqpOffset));
        qp += qpOffset ;
      }
    }

#if !QP_SWITCHING_FOR_PARALLEL
    // modify QP if a fractional QP was originally specified, cause dQPs to be 0 or 1.
    const int* pdQPs = getdQPs();
    if ( pdQPs )
    {
      qp += pdQPs[ pSlice->getPOC() ];
    }
#endif
  }
  qp = Clip3( -lumaQpBDOffset, MAX_QP, qp );
  return qp;
}
#endif

//! \}
