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
  , m_apsMap(MAX_NUM_APS * MAX_NUM_APS_TYPE)
  , m_AUWriterIf( nullptr )
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
  , m_lmcsAPS(nullptr)
#if JVET_O0299_APS_SCALINGLIST
  , m_scalinglistAPS( nullptr )
#endif
#if JVET_O0119_BASE_PALETTE_444
  , m_doPlt( true )
#endif
{
  m_iPOCLast          = -1;
  m_iNumPicRcvd       =  0;
  m_uiNumAllPicCoded  =  0;

  m_iMaxRefPicNum     = 0;

#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif

  memset(m_apss, 0, sizeof(m_apss));
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
#if !JVET_O1164_PS
  m_cSliceEncoder.      create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth );
#endif
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

#if JVET_O1164_PS
  m_cLoopFilter.create( m_maxTotalCUDepth );
#else
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
    m_cEncALF.create( this, getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth, m_bitDepth, m_inputBitDepth );
  }
#endif

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
#if JVET_O0299_APS_SCALINGLIST
  APS &aps0 = *( m_apsMap.allocatePS( SCALING_LIST_APS ) );
  aps0.setAPSId( 0 );
  aps0.setAPSType( SCALING_LIST_APS );
#endif

  // initialize SPS
  xInitSPS(sps0);
  xInitVPS(m_cVPS);

  int dpsId = getDecodingParameterSetEnabled() ? 1 : 0;
  xInitDPS(m_dps, sps0, dpsId);
  sps0.setDecodingParameterSetId(m_dps.getDecodingParameterSetId());

#if ENABLE_SPLIT_PARALLELISM
  if( omp_get_dynamic() )
  {
    omp_set_dynamic( false );
  }
  omp_set_nested( true );
#endif

#if JVET_N0494_DRAP
  if (getUseCompositeRef() || getDependentRAPIndicationSEIEnabled())
#else
  if (getUseCompositeRef())
#endif
  {
    sps0.setLongTermRefsPresent(true);
  }

#if U0132_TARGET_BITS_SATURATION
  if (m_RCCpbSaturationEnabled)
  {
    m_cRateCtrl.initHrdParam(sps0.getHrdParameters(), m_iFrameRate, m_RCInitialCpbFullness);
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
#if JVET_O1164_PS
  pps0.setPicWidthInLumaSamples( m_iSourceWidth );
  pps0.setPicHeightInLumaSamples( m_iSourceHeight );
  pps0.setConformanceWindow( m_conformanceWindow );
#endif
  xInitPPS(pps0, sps0);
  // initialize APS
  xInitRPL(sps0, isFieldCoding);

#if JVET_O1164_RPR
  if( m_rprEnabled )
  {
    PPS &pps = *( m_ppsMap.allocatePS( ENC_PPS_ID_RPR ) );
#if RPR_CONF_WINDOW
    Window& inputConfWindow = pps0.getConformanceWindow();
    int scaledWidth = int((pps0.getPicWidthInLumaSamples() - (inputConfWindow.getWindowLeftOffset() + inputConfWindow.getWindowRightOffset()) * SPS::getWinUnitX(sps0.getChromaFormatIdc())) / m_scalingRatioHor);
#else
    int scaledWidth = int(pps0.getPicWidthInLumaSamples() / m_scalingRatioHor);
#endif
    int minSizeUnit = std::max(8, (int)(sps0.getMaxCUHeight() >> (sps0.getMaxCodingDepth() - 1)));
    int temp = scaledWidth / minSizeUnit;
    int width = ( scaledWidth - ( temp * minSizeUnit) > 0 ? temp + 1 : temp ) * minSizeUnit;

#if RPR_CONF_WINDOW
    int scaledHeight = int((pps0.getPicHeightInLumaSamples() - (inputConfWindow.getWindowTopOffset() + inputConfWindow.getWindowBottomOffset()) * SPS::getWinUnitY(sps0.getChromaFormatIdc())) / m_scalingRatioVer);
#else
    int scaledHeight = int(pps0.getPicHeightInLumaSamples() / m_scalingRatioVer);
#endif
    temp = scaledHeight / minSizeUnit;
    int height = ( scaledHeight - ( temp * minSizeUnit) > 0 ? temp + 1 : temp ) * minSizeUnit;

    pps.setPicWidthInLumaSamples( width );
    pps.setPicHeightInLumaSamples( height );

    Window conformanceWindow;

    conformanceWindow.setWindow( 0, ( width - scaledWidth ) / SPS::getWinUnitX( sps0.getChromaFormatIdc() ), 0, ( height - scaledHeight ) / SPS::getWinUnitY( sps0.getChromaFormatIdc() ) );

    pps.setConformanceWindow( conformanceWindow );

    xInitPPS( pps, sps0 ); // will allocate memory for and initialize pps.pcv inside
  }
#endif

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
                          true
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
                   true
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

#if ER_CHROMA_QP_WCG_PPS
  if( m_wcgChromaQpControl.isEnabled() )
  {
#if JVET_O0299_APS_SCALINGLIST  
    xInitScalingLists( sps0, *m_apsMap.getPS( 1 ) );
    xInitScalingLists( sps0, aps0 );
#else
    xInitScalingLists( sps0, *m_ppsMap.getPS(1) );
    xInitScalingLists( sps0, pps0 );
#endif
  }
  else
#endif
  {
#if JVET_O0299_APS_SCALINGLIST
    xInitScalingLists( sps0, aps0 );
#else
    xInitScalingLists( sps0, pps0 );
#endif
  }
#if JVET_O1164_RPR
  if( m_rprEnabled )
  {
#if JVET_O0299_APS_SCALINGLIST
    xInitScalingLists( sps0, *m_apsMap.getPS( ENC_PPS_ID_RPR ) );
#else
    xInitScalingLists( sps0, *m_ppsMap.getPS( ENC_PPS_ID_RPR ) );
#endif
  }
#endif
#if ENABLE_WPP_PARALLELISM
  m_entropyCodingSyncContextStateVec.resize( pps0.pcv->heightInCtus );
#endif
  if (getUseCompositeRef())
  {
    Picture *picBg = new Picture;
#if JVET_O1164_PS
    picBg->create( sps0.getChromaFormatIdc(), Size( pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples() ), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false );
#else
    picBg->create(sps0.getChromaFormatIdc(), Size(sps0.getPicWidthInLumaSamples(), sps0.getPicHeightInLumaSamples()), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false);
#endif
    picBg->getRecoBuf().fill(0);
#if JVET_O0299_APS_SCALINGLIST
    picBg->finalInit( sps0, pps0, m_apss, m_lmcsAPS, m_scalinglistAPS );
#else
    picBg->finalInit(sps0, pps0, m_apss, m_lmcsAPS);
#endif
    pps0.setNumBricksInPic((int)picBg->brickMap->bricks.size());
    picBg->allocateNewSlice();
    picBg->createSpliceIdx(pps0.pcv->sizeInCtus);
    m_cGOPEncoder.setPicBg(picBg);
    Picture *picOrig = new Picture;
#if JVET_O1164_PS
    picOrig->create( sps0.getChromaFormatIdc(), Size( pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples() ), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false );
#else
    picOrig->create(sps0.getChromaFormatIdc(), Size(sps0.getPicWidthInLumaSamples(), sps0.getPicHeightInLumaSamples()), sps0.getMaxCUWidth(), sps0.getMaxCUWidth() + 16, false);
#endif
    picOrig->getOrigBuf().fill(0);
    m_cGOPEncoder.setPicOrig(picOrig);
  }
}

#if JVET_O0299_APS_SCALINGLIST
void EncLib::xInitScalingLists( SPS &sps, APS &aps )
#else
void EncLib::xInitScalingLists(SPS &sps, PPS &pps)
#endif
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
#if !JVET_O0299_APS_SCALINGLIST
    sps.setScalingListPresentFlag(false);
    pps.setScalingListPresentFlag(false);
#endif
  }
  else if(getUseScalingListId() == SCALING_LIST_DEFAULT)
  {
#if JVET_O0299_APS_SCALINGLIST
    aps.getScalingList().setDefaultScalingList ();
    quant->setScalingList( &( aps.getScalingList() ), maxLog2TrDynamicRange, sps.getBitDepths() );
#else
    sps.getScalingList().setDefaultScalingList ();
    sps.setScalingListPresentFlag(false);
    pps.setScalingListPresentFlag(false);

    quant->setScalingList(&(sps.getScalingList()), maxLog2TrDynamicRange, sps.getBitDepths());
#endif
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
#if JVET_O0299_APS_SCALINGLIST 
    aps.getScalingList().setDefaultScalingList();
    CHECK( aps.getScalingList().xParseScalingList( getScalingListFileName() ), "Error Parsing Scaling List Input File" );
    aps.getScalingList().checkDcOfMatrix();
    if( aps.getScalingList().isNotDefaultScalingList() == false )
    {
      setUseScalingListId( SCALING_LIST_DEFAULT );
    }
    quant->setScalingList( &( aps.getScalingList() ), maxLog2TrDynamicRange, sps.getBitDepths() );
#else
    sps.getScalingList().setDefaultScalingList ();
    if(sps.getScalingList().xParseScalingList(getScalingListFileName()))
    {
      THROW( "parse scaling list");
    }
    sps.getScalingList().checkDcOfMatrix();
    sps.setScalingListPresentFlag(sps.getScalingList().isNotDefaultScalingList());
    pps.setScalingListPresentFlag(false);

    quant->setScalingList(&(sps.getScalingList()), maxLog2TrDynamicRange, sps.getBitDepths());
#endif
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

#if JVET_O0299_APS_SCALINGLIST 
  if( getUseScalingListId() == SCALING_LIST_FILE_READ )
#else
  if (getUseScalingListId() == SCALING_LIST_FILE_READ && sps.getScalingListPresentFlag())
#endif
  {
    // Prepare delta's:
    for (uint32_t sizeId = SCALING_LIST_2x2; sizeId <= SCALING_LIST_64x64; sizeId++)
    {
      for (uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        if (((sizeId == SCALING_LIST_64x64) && (listId % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0))
         || ((sizeId == SCALING_LIST_2x2) && (listId % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0)))
        {
          continue;
        }
#if JVET_O0299_APS_SCALINGLIST 
        aps.getScalingList().checkPredMode( sizeId, listId );
#else
        sps.getScalingList().checkPredMode( sizeId, listId );
#endif
      }
    }
  }
}

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
#if JVET_O0299_APS_SCALINGLIST
    picCurr->finalInit( *sps, *pps, m_apss, m_lmcsAPS, m_scalinglistAPS );
#else
    picCurr->finalInit(*sps, *pps, m_apss, m_lmcsAPS);
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
#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cGOPEncoder.getMetricTime();
#endif
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

    int ppsID=-1; // Use default PPS ID
#if ER_CHROMA_QP_WCG_PPS
    if (getWCGChromaQPControl().isEnabled())
    {
      ppsID = getdQPs()[m_iPOCLast / (m_compositeRefEnabled ? 2 : 1) + 1];
      ppsID+=(getSwitchPOC() != -1 && (m_iPOCLast+1 >= getSwitchPOC())?1:0);
    }
#endif

#if JVET_O1164_RPR
    if( m_rprEnabled && m_uiIntraPeriod == -1 )
    {
      const int poc = m_iPOCLast + ( m_compositeRefEnabled ? 2 : 1 );

      if( poc / m_switchPocPeriod % 2 )
      {
        ppsID = ENC_PPS_ID_RPR;
      }
      else
      {
        ppsID = 0;
      }
    }
#endif
    xGetNewPicBuffer( rcListPicYuvRecOut,
                      pcPicCurr, ppsID );

    {
      const PPS *pPPS=(ppsID<0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsID);
      const SPS *pSPS=m_spsMap.getPS(pPPS->getSPSId());

#if JVET_O1164_RPR
      if( m_rprEnabled )
      {
#if RPR_CTC_PRINT
        pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Y ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Y ) );
        pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Cb ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Cb ) );
        pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Cr ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Cr ) );

        pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT ).getBuf( COMPONENT_Y ).copyFrom( cPicYuvTrueOrg->getBuf( COMPONENT_Y ) );
        pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT ).getBuf( COMPONENT_Cb ).copyFrom( cPicYuvTrueOrg->getBuf( COMPONENT_Cb ) );
        pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT ).getBuf( COMPONENT_Cr ).copyFrom( cPicYuvTrueOrg->getBuf( COMPONENT_Cr ) );
#endif

        const ChromaFormat chromaFormatIDC = pSPS->getChromaFormatIdc();

#if RPR_CONF_WINDOW
        const PPS *refPPS = m_ppsMap.getPS(0);
        Picture::rescalePicture( *pcPicYuvOrg, refPPS->getConformanceWindow(), pcPicCurr->getOrigBuf(), pPPS->getConformanceWindow(), chromaFormatIDC, pSPS->getBitDepths(), true, true );
        Picture::rescalePicture( *cPicYuvTrueOrg, refPPS->getConformanceWindow(), pcPicCurr->getTrueOrigBuf(), pPPS->getConformanceWindow(), chromaFormatIDC, pSPS->getBitDepths(), true, true );
#else
        Picture::rescalePicture(*pcPicYuvOrg, pcPicCurr->getOrigBuf(), chromaFormatIDC, pSPS->getBitDepths(), true, true);
        Picture::rescalePicture(*cPicYuvTrueOrg, pcPicCurr->getTrueOrigBuf(), chromaFormatIDC, pSPS->getBitDepths(), true, true);
#endif
      }
      else
      {
        pcPicCurr->M_BUFS( 0, PIC_ORIGINAL ).swap( *pcPicYuvOrg );
        pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL ).swap( *cPicYuvTrueOrg );
      }
#else
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL ).swap( *pcPicYuvOrg );
      pcPicCurr->M_BUFS( 0, PIC_TRUE_ORIGINAL ).swap(*cPicYuvTrueOrg );
#endif

#if JVET_O0299_APS_SCALINGLIST
      pcPicCurr->finalInit( *pSPS, *pPPS, m_apss, m_lmcsAPS, m_scalinglistAPS );
#else
      pcPicCurr->finalInit(*pSPS, *pPPS, m_apss, m_lmcsAPS);
#endif
      PPS *ptrPPS = (ppsID<0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsID);
      ptrPPS->setNumBricksInPic((int)pcPicCurr->brickMap->bricks.size());
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
#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

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
#if JVET_O0299_APS_SCALINGLIST
        pcField->finalInit( *pSPS, *pPPS, m_apss, m_lmcsAPS, m_scalinglistAPS );
#else
        pcField->finalInit(*pSPS, *pPPS, m_apss, m_lmcsAPS);
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
#if JVET_O0756_CALCULATE_HDRMETRICS
      m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

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
#if JVET_O1164_PS
    rpcPic->create( sps.getChromaFormatIdc(), Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ), sps.getMaxCUWidth(), sps.getMaxCUWidth() + 16, false );
#if RPR_CTC_PRINT
    if( m_rprEnabled )
    {
      rpcPic->M_BUFS( 0, PIC_ORIGINAL_INPUT ).create( sps.getChromaFormatIdc(), Area( Position(), Size( sps.getMaxPicWidthInLumaSamples(), sps.getMaxPicHeightInLumaSamples() ) ) );
      rpcPic->M_BUFS( 0, PIC_TRUE_ORIGINAL_INPUT ).create( sps.getChromaFormatIdc(), Area( Position(), Size( sps.getMaxPicWidthInLumaSamples(), sps.getMaxPicHeightInLumaSamples() ) ) );
    }
#endif
#else
    rpcPic->create( sps.getChromaFormatIdc(), Size( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples()), sps.getMaxCUWidth(), sps.getMaxCUWidth()+16, false );
#endif
    if ( getUseAdaptiveQP() )
    {
      const uint32_t iMaxDQPLayer = pps.getCuQpDeltaSubdiv()/2+1;
      rpcPic->aqlayer.resize( iMaxDQPLayer );
      for (uint32_t d = 0; d < iMaxDQPLayer; d++)
      {
#if JVET_O1164_PS
        rpcPic->aqlayer[d] = new AQpLayer( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples(), sps.getMaxCUWidth() >> d, sps.getMaxCUHeight() >> d );
#else
        rpcPic->aqlayer[d] = new AQpLayer( sps.getPicWidthInLumaSamples(), sps.getPicHeightInLumaSamples(), sps.getMaxCUWidth()>>d, sps.getMaxCUHeight()>>d );
#endif
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


void EncLib::xInitVPS(VPS &vps)
{
  // The SPS must have already been set up.
  // set the VPS profile information.
  vps.setMaxLayers(1);
  for (uint32_t i = 0; i < vps.getMaxLayers(); i++)
  {
    vps.setVPSIncludedLayerId(0, i);
  }
}

void EncLib::xInitDPS(DPS &dps, const SPS &sps, const int dpsId)
{
  // The SPS must have already been set up.
  // set the DPS profile information.
  dps.setDecodingParameterSetId(dpsId);
  dps.setMaxSubLayersMinus1(sps.getMaxTLayers()-1);
  dps.setProfileTierLevel(*sps.getProfileTierLevel());
}


void EncLib::xInitSPS(SPS &sps)
{
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
  cinfo->setNoPartitionConstraintsOverrideConstraintFlag(m_noPartitionConstraintsOverrideConstraintFlag);
  cinfo->setNoSaoConstraintFlag(m_bNoSaoConstraintFlag);
  cinfo->setNoAlfConstraintFlag(m_bNoAlfConstraintFlag);
#if !JVET_O0525_REMOVE_PCM
  cinfo->setNoPcmConstraintFlag(m_bNoPcmConstraintFlag);
#endif
  cinfo->setNoRefWraparoundConstraintFlag(m_bNoRefWraparoundConstraintFlag);
  cinfo->setNoTemporalMvpConstraintFlag(m_bNoTemporalMvpConstraintFlag);
  cinfo->setNoSbtmvpConstraintFlag(m_bNoSbtmvpConstraintFlag);
  cinfo->setNoAmvrConstraintFlag(m_bNoAmvrConstraintFlag);
  cinfo->setNoBdofConstraintFlag(m_bNoBdofConstraintFlag);
  cinfo->setNoDmvrConstraintFlag(m_noDmvrConstraintFlag);
  cinfo->setNoCclmConstraintFlag(m_bNoCclmConstraintFlag);
  cinfo->setNoMtsConstraintFlag(m_bNoMtsConstraintFlag);
  cinfo->setNoSbtConstraintFlag(m_noSbtConstraintFlag);
  cinfo->setNoAffineMotionConstraintFlag(m_bNoAffineMotionConstraintFlag);
  cinfo->setNoGbiConstraintFlag(m_bNoGbiConstraintFlag);
  cinfo->setNoIbcConstraintFlag(m_noIbcConstraintFlag);
  cinfo->setNoMhIntraConstraintFlag(m_bNoMhIntraConstraintFlag);
  cinfo->setNoFPelMmvdConstraintFlag(m_noFPelMmvdConstraintFlag);
  cinfo->setNoTriangleConstraintFlag(m_bNoTriangleConstraintFlag);
  cinfo->setNoLadfConstraintFlag(m_bNoLadfConstraintFlag);
  cinfo->setNoTransformSkipConstraintFlag(m_noTransformSkipConstraintFlag);
#if JVET_O1136_TS_BDPCM_SIGNALLING
  cinfo->setNoBDPCMConstraintFlag(m_noBDPCMConstraintFlag);
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  cinfo->setNoJointCbCrConstraintFlag(m_noJointCbCrConstraintFlag);
#endif
  cinfo->setNoQpDeltaConstraintFlag(m_bNoQpDeltaConstraintFlag);
  cinfo->setNoDepQuantConstraintFlag(m_bNoDepQuantConstraintFlag);
  cinfo->setNoSignDataHidingConstraintFlag(m_bNoSignDataHidingConstraintFlag);

  profileTierLevel->setLevelIdc                    (m_level);
  profileTierLevel->setTierFlag                    (m_levelTier);
  profileTierLevel->setProfileIdc                  (m_profile);
#if JVET_O0044_MULTI_SUB_PROFILE
  profileTierLevel->setNumSubProfile(m_numSubProfile);
  for (int k = 0; k < m_numSubProfile; k++)
  {
    profileTierLevel->setSubProfileIdc(k, m_subProfile[k]);
  }
#else
  profileTierLevel->setSubProfileIdc(m_subProfile);
#endif
  /* XXX: should Main be marked as compatible with still picture? */
  /* XXX: may be a good idea to refactor the above into a function
   * that chooses the actual compatibility based upon options */

#if JVET_O1164_PS
  sps.setMaxPicWidthInLumaSamples( m_iSourceWidth );
  sps.setMaxPicHeightInLumaSamples( m_iSourceHeight );
#else
  sps.setPicWidthInLumaSamples  ( m_iSourceWidth      );
  sps.setPicHeightInLumaSamples ( m_iSourceHeight     );
  sps.setConformanceWindow      ( m_conformanceWindow );
#endif
  sps.setMaxCUWidth             ( m_maxCUWidth        );
  sps.setMaxCUHeight            ( m_maxCUHeight       );
  sps.setMaxCodingDepth         ( m_maxTotalCUDepth   );
  sps.setChromaFormatIdc        ( m_chromaFormatIDC   );
  sps.setLog2DiffMaxMinCodingBlockSize(m_log2DiffMaxMinCodingBlockSize);

  sps.setCTUSize                             ( m_CTUSize );
  sps.setSplitConsOverrideEnabledFlag        ( m_useSplitConsOverride );
  sps.setMinQTSizes                          ( m_uiMinQT );
  sps.setMaxMTTHierarchyDepth                ( m_uiMaxMTTHierarchyDepth, m_uiMaxMTTHierarchyDepthI, m_uiMaxMTTHierarchyDepthIChroma );
  unsigned maxBtSize[3], maxTtSize[3];
  memcpy(maxBtSize, m_uiMinQT, sizeof(maxBtSize));
  memcpy(maxTtSize, m_uiMinQT, sizeof(maxTtSize));
  if (m_uiMaxMTTHierarchyDepth)
  {
    maxBtSize[1] = std::min(m_CTUSize, (unsigned)MAX_BT_SIZE_INTER);
    maxTtSize[1] = std::min(m_CTUSize, (unsigned)MAX_TT_SIZE_INTER);
  }
  if (m_uiMaxMTTHierarchyDepthI)
  {
    maxBtSize[0] = std::min(m_CTUSize, (unsigned)MAX_BT_SIZE);
    maxTtSize[0] = std::min(m_CTUSize, (unsigned)MAX_TT_SIZE);
  }
  if (m_uiMaxMTTHierarchyDepthIChroma)
  {
    maxBtSize[2] = std::min(m_CTUSize, (unsigned)MAX_BT_SIZE_C);
    maxTtSize[2] = std::min(m_CTUSize, (unsigned)MAX_TT_SIZE_C);
  }
  sps.setMaxBTSize                           ( maxBtSize[1], maxBtSize[0], maxBtSize[2] );
  sps.setMaxTTSize                           ( maxTtSize[1], maxTtSize[0], maxTtSize[2] );
  sps.setIDRRefParamListPresent              ( m_idrRefParamList );
  sps.setUseDualITree                        ( m_dualITree );
  sps.setUseLFNST                            ( m_LFNST );
  sps.setSBTMVPEnabledFlag                  ( m_SubPuMvpMode );
  sps.setAMVREnabledFlag                ( m_ImvMode != IMV_OFF );
  sps.setBDOFEnabledFlag                    ( m_BIO );
  sps.setUseAffine             ( m_Affine );
  sps.setUseAffineType         ( m_AffineType );
#if JVET_O0070_PROF
  sps.setUsePROF               ( m_PROF );
#endif
  sps.setUseLMChroma           ( m_LMChroma ? true : false );
  sps.setCclmCollocatedChromaFlag( m_cclmCollocatedChromaFlag );
  sps.setUseMTS                ( m_IntraMTS || m_InterMTS || m_ImplicitMTS );
  sps.setUseIntraMTS           ( m_IntraMTS );
  sps.setUseInterMTS           ( m_InterMTS );
  sps.setUseSBT                             ( m_SBT );
  if( sps.getUseSBT() )
  {
#if JVET_O0545_MAX_TB_SIGNALLING
    sps.setMaxSbtSize                       ( std::min((int)(1 << m_log2MaxTbSize), m_iSourceWidth >= 1920 ? 64 : 32) );
#else
    sps.setMaxSbtSize                       ( m_iSourceWidth >= 1920 ? 64 : 32 );
#endif
  }
  sps.setUseSMVD                ( m_SMVD );
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
  sps.setUseMMVD               ( m_MMVD );
  sps.setFpelMmvdEnabledFlag   (( m_MMVD ) ? m_allowDisFracMMVD : false);
#if JVET_O1140_SLICE_DISABLE_BDOF_DMVR_FLAG
  sps.setBdofDmvrSlicePresentFlag(m_DMVR || m_BIO);
#endif
  sps.setAffineAmvrEnabledFlag              ( m_AffineAmvr );
  sps.setUseDMVR                            ( m_DMVR );
#if JVET_O0119_BASE_PALETTE_444
  sps.setPLTMode                            ( m_PLTMode);
#endif
  sps.setIBCFlag                            ( m_IBCMode);
  sps.setWrapAroundEnabledFlag                      ( m_wrapAround );
  sps.setWrapAroundOffset                   ( m_wrapAroundOffset );
  // ADD_NEW_TOOL : (encoder lib) set tool enabling flags and associated parameters here
  sps.setUseISP                             ( m_ISP );
  sps.setUseReshaper                        ( m_lumaReshapeEnable );
  sps.setUseMIP                ( m_MIP );
  int minCUSize =  sps.getMaxCUWidth() >> sps.getLog2DiffMaxMinCodingBlockSize();
  int log2MinCUSize = 0;
  while(minCUSize > 1)
  {
    minCUSize >>= 1;
    log2MinCUSize++;
  }

  sps.setLog2MinCodingBlockSize(log2MinCUSize);

#if !JVET_O0525_REMOVE_PCM
  sps.setPCMLog2MinSize (m_uiPCMLog2MinSize);
  sps.setPCMEnabledFlag        ( m_usePCM           );
  sps.setPCMLog2MaxSize( m_pcmLog2MaxSize  );
#endif

#if JVET_O1136_TS_BDPCM_SIGNALLING
  sps.setTransformSkipEnabledFlag(m_useTransformSkip);
  sps.setBDPCMEnabledFlag(m_useBDPCM);
#endif

  sps.setSPSTemporalMVPEnabledFlag((getTMVPModeId() == 2 || getTMVPModeId() == 1));

#if MAX_TB_SIZE_SIGNALLING
  sps.setLog2MaxTbSize   ( m_log2MaxTbSize );
#endif

  for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    sps.setBitDepth      (ChannelType(channelType), m_bitDepth[channelType] );
    sps.setQpBDOffset  (ChannelType(channelType), (6 * (m_bitDepth[channelType] - 8)));
#if JVET_O0919_TS_MIN_QP
    sps.setMinQpPrimeTsMinus4(ChannelType(channelType), (6 * (m_bitDepth[channelType] - m_inputBitDepth[channelType])));
#endif
#if !JVET_O0525_REMOVE_PCM
    sps.setPCMBitDepth (ChannelType(channelType), m_PCMBitDepth[channelType]         );
#endif
  }

#if JVET_O0244_DELTA_POC
  sps.setUseWP( m_useWeightedPred );
  sps.setUseWPBiPred( m_useWeightedBiPred );
#endif

  sps.setSAOEnabledFlag( m_bUseSAO );
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  sps.setJointCbCrEnabledFlag( m_JointCbCrMode );
#endif
  sps.setMaxTLayers( m_maxTempLayer );
  sps.setTemporalIdNestingFlag( ( m_maxTempLayer == 1 ) ? true : false );

  for (int i = 0; i < std::min(sps.getMaxTLayers(), (uint32_t) MAX_TLAYER); i++ )
  {
    sps.setMaxDecPicBuffering(m_maxDecPicBuffering[i], i);
    sps.setNumReorderPics(m_numReorderPics[i], i);
  }

#if !JVET_O0525_REMOVE_PCM
  sps.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag );
#endif
  sps.setScalingListFlag ( (m_useScalingListId == SCALING_LIST_OFF) ? 0 : 1 );
  sps.setALFEnabledFlag( m_alf );
  sps.setVuiParametersPresentFlag(getVuiParametersPresentFlag());

  if (sps.getVuiParametersPresentFlag())
  {
    VUI* pcVUI = sps.getVuiParameters();
    pcVUI->setAspectRatioInfoPresentFlag(getAspectRatioInfoPresentFlag());
    pcVUI->setAspectRatioIdc(getAspectRatioIdc());
    pcVUI->setSarWidth(getSarWidth());
    pcVUI->setSarHeight(getSarHeight());
    pcVUI->setColourDescriptionPresentFlag(getColourDescriptionPresentFlag());
    pcVUI->setColourPrimaries(getColourPrimaries());
    pcVUI->setTransferCharacteristics(getTransferCharacteristics());
    pcVUI->setMatrixCoefficients(getMatrixCoefficients());
    pcVUI->setFieldSeqFlag(false);
    pcVUI->setChromaLocInfoPresentFlag(getChromaLocInfoPresentFlag());
    pcVUI->setChromaSampleLocTypeTopField(getChromaSampleLocTypeTopField());
    pcVUI->setChromaSampleLocTypeBottomField(getChromaSampleLocTypeBottomField());
    pcVUI->setChromaSampleLocType(getChromaSampleLocType());
    pcVUI->setOverscanInfoPresentFlag(getOverscanInfoPresentFlag());
    pcVUI->setOverscanAppropriateFlag(getOverscanAppropriateFlag());
    pcVUI->setVideoSignalTypePresentFlag(getVideoSignalTypePresentFlag());
    pcVUI->setVideoFullRangeFlag(getVideoFullRangeFlag());
  }

  sps.setNumLongTermRefPicSPS(NUM_LONG_TERM_REF_PIC_SPS);
  CHECK(!(NUM_LONG_TERM_REF_PIC_SPS <= MAX_NUM_LONG_TERM_REF_PICS), "Unspecified error");
  for (int k = 0; k < NUM_LONG_TERM_REF_PIC_SPS; k++)
  {
    sps.setLtRefPicPocLsbSps(k, 0);
    sps.setUsedByCurrPicLtSPSFlag(k, 0);
  }
#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
  sps.setChromaQpMappingTableFromParams(m_chromaQpMappingTableParams, sps.getQpBDOffset(CHANNEL_TYPE_CHROMA));
  sps.derivedChromaQPMappingTables();
#endif

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
    sps.setHrdParametersPresentFlag( true );
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

  if (m_uiIntraPeriod < 0)
    sps.setRPL1CopyFromRPL0Flag(true);
}

void EncLib::xInitHrdParameters(SPS &sps)
{
  m_encHRD.initHRDParameters((EncCfg*) this);

  HRDParameters *hrdParams = sps.getHrdParameters();
  *hrdParams = m_encHRD.getHRDParameters();

  TimingInfo *timingInfo = sps.getTimingInfo();
  *timingInfo = m_encHRD.getTimingInfo();
}

void EncLib::xInitPPS(PPS &pps, const SPS &sps)
{
  // pps ID already initialised.
  pps.setSPSId(sps.getSPSId());

  #if JVET_O0238_PPS_OR_SLICE
  pps.setConstantSliceHeaderParamsEnabledFlag(getConstantSliceHeaderParamsEnabledFlag());
  pps.setPPSDepQuantEnabledIdc(getPPSDepQuantEnabledIdc());
  pps.setPPSRefPicListSPSIdc0(getPPSRefPicListSPSIdc0());
  pps.setPPSRefPicListSPSIdc1(getPPSRefPicListSPSIdc1());
  pps.setPPSTemporalMVPEnabledIdc(getPPSTemporalMVPEnabledIdc());
  pps.setPPSMvdL1ZeroIdc(getPPSMvdL1ZeroIdc());
  pps.setPPSCollocatedFromL0Idc(getPPSCollocatedFromL0Idc());
  pps.setPPSSixMinusMaxNumMergeCandPlus1(getPPSSixMinusMaxNumMergeCandPlus1());
  pps.setPPSFiveMinusMaxNumSubblockMergeCandPlus1(getPPSFiveMinusMaxNumSubblockMergeCandPlus1());
  pps.setPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1(getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1());
#endif

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
    pps.setCuChromaQpOffsetSubdiv(m_cuChromaQpOffsetSubdiv);
    pps.clearChromaQpOffsetList();
#if JVET_O1168_CU_CHROMA_QP_OFFSET
    pps.setChromaQpOffsetListEntry(1, 6, 6, 6);
#else
    pps.setChromaQpOffsetListEntry(1, 6, 6);
#endif
    /* todo, insert table entries from command line (NB, 0 should not be touched) */
  }
  else
  {
    pps.setCuChromaQpOffsetSubdiv(0);
    pps.clearChromaQpOffsetList();
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
    pps.setQpOffset(JOINT_CbCr,   Clip3( -12, 12, ( min(0, cbQP) + min(0, crQP) ) / 2 + m_chromaCbCrQpOffset));
  }
  else
  {
#endif
  pps.setQpOffset(COMPONENT_Cb, m_chromaCbQpOffset );
  pps.setQpOffset(COMPONENT_Cr, m_chromaCrQpOffset );
  pps.setQpOffset(JOINT_CbCr, m_chromaCbCrQpOffset );
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
    pps.setSliceChromaQpFlag(m_chromaCbQpOffsetDualTree != 0 || m_chromaCrQpOffsetDualTree != 0 || m_chromaCbCrQpOffsetDualTree != 0);
  }

  pps.setEntropyCodingSyncEnabledFlag( m_entropyCodingSyncEnabledFlag );

  pps.setSingleTileInPicFlag((m_iNumColumnsMinus1 == 0 && m_iNumRowsMinus1 == 0));

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
    CHECK(!(getRPLEntry(0, i).m_numRefPicsActive >= 0 && getRPLEntry(0, i).m_numRefPicsActive <= MAX_NUM_REF), "Unspecified error");
    histogram[getRPLEntry(0, i).m_numRefPicsActive]++;
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
#if JVET_O1136_TS_BDPCM_SIGNALLING
  pps.setLog2MaxTransformSkipBlockSize(m_log2MaxTransformSkipBlockSize);
#else
  pps.setUseTransformSkip( m_useTransformSkip );
  pps.getPpsRangeExtension().setLog2MaxTransformSkipBlockSize( m_log2MaxTransformSkipBlockSize  );
#endif


  xInitPPSforTiles(pps);

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

  pps.pcv = new PreCalcValues( sps, pps, true );
  pps.setRpl1IdxPresentFlag(sps.getRPL1IdxPresentFlag());
}

void EncLib::xInitAPS(APS &aps)
{
  //Do nothing now
}

void EncLib::xInitRPL(SPS &sps, bool isFieldCoding)
{
  ReferencePictureList*      rpl;

  int numRPLCandidates = getRPLCandidateSize(0);
  sps.createRPLList0(numRPLCandidates);
  sps.createRPLList1(numRPLCandidates);
  RPLList* rplList = 0;

  for (int i = 0; i < 2; i++)
  {
    rplList = (i == 0) ? sps.getRPLList0() : sps.getRPLList1();
    for (int j = 0; j < numRPLCandidates; j++)
    {
      const RPLEntry &ge = getRPLEntry(i, j);
      rpl = rplList->getReferencePictureList(j);
      rpl->setNumberOfShorttermPictures(ge.m_numRefPics);
      rpl->setNumberOfLongtermPictures(0);   //Hardcoded as 0 for now. need to update this when implementing LTRP
      rpl->setNumberOfActivePictures(ge.m_numRefPicsActive);
#if JVET_N0100_PROPOSAL1
      rpl->setLtrpInSliceHeaderFlag(ge.m_ltrp_in_slice_header_flag);
#endif

      for (int k = 0; k < ge.m_numRefPics; k++)
      {
        rpl->setRefPicIdentifier(k, ge.m_deltaRefPics[k], 0);
      }
    }
  }

  //Check if all delta POC of STRP in each RPL has the same sign
  //Check RPLL0 first
  const RPLList* rplList0 = sps.getRPLList0();
  const RPLList* rplList1 = sps.getRPLList1();
  uint32_t numberOfRPL = sps.getNumRPL0();

  bool isAllEntriesinRPLHasSameSignFlag = true;
  bool isFirstEntry = true;
  bool lastSign = true;        //true = positive ; false = negative
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && ii < numberOfRPL; ii++)
  {
    const ReferencePictureList* rpl = rplList0->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->getNumberOfActivePictures(); jj++)
    {
      if (!rpl->isRefPicLongterm(jj) && isFirstEntry)
      {
        lastSign = (rpl->getRefPicIdentifier(jj) >= 0) ? true : false;
        isFirstEntry = false;
      }
      else if (!rpl->isRefPicLongterm(jj) && (((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) >= 0 && lastSign == false) || ((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) < 0 && lastSign == true)))
      {
        isAllEntriesinRPLHasSameSignFlag = false;
      }
    }
  }
  //Check RPLL1. Skip it if it is already found out that this flag is not true for RPL0 or if RPL1 is the same as RPL0
  numberOfRPL = sps.getNumRPL1();
  isFirstEntry = true;
  lastSign = true;
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && !sps.getRPL1CopyFromRPL0Flag() && ii < numberOfRPL; ii++)
  {
    isFirstEntry = true;
    const ReferencePictureList* rpl = rplList1->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->getNumberOfActivePictures(); jj++)
    {
      if (!rpl->isRefPicLongterm(jj) && isFirstEntry)
      {
        lastSign = (rpl->getRefPicIdentifier(jj) >= 0) ? true : false;
        isFirstEntry = false;
      }
      else if (!rpl->isRefPicLongterm(jj) && (((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) >= 0 && lastSign == false) || ((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) < 0 && lastSign == true)))
      {
        isAllEntriesinRPLHasSameSignFlag = false;
      }
    }
  }
  sps.setAllActiveRplEntriesHasSameSignFlag(isAllEntriesinRPLHasSameSignFlag);
}

void EncLib::getActiveRefPicListNumForPOC(const SPS *sps, int POCCurr, int GOPid, uint32_t *activeL0, uint32_t *activeL1)
{
  if (m_uiIntraPeriod < 0)  //Only for RA
  {
    *activeL0 = *activeL1 = 0;
    return;
  }
  uint32_t rpl0Idx = GOPid;
  uint32_t rpl1Idx = GOPid;

  int fullListNum = m_iGOPSize;
  int partialListNum = getRPLCandidateSize(0) - m_iGOPSize;
  int extraNum = fullListNum;
  if (m_uiIntraPeriod < 0)
  {
    if (POCCurr < 10)
    {
      rpl0Idx = POCCurr + m_iGOPSize - 1;
      rpl1Idx = POCCurr + m_iGOPSize - 1;
    }
    else
    {
      rpl0Idx = (POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1;
      rpl1Idx = (POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1;
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum<fullListNum + partialListNum; extraNum++)
  {
    if (m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      int POCIndex = POCCurr%m_uiIntraPeriod;
      if (POCIndex == 0)
        POCIndex = m_uiIntraPeriod;
      if (POCIndex == m_RPLList0[extraNum].m_POC)
      {
        rpl0Idx = extraNum;
        rpl1Idx = extraNum;
        extraNum++;
      }
    }
  }

  const ReferencePictureList *rpl0 = sps->getRPLList0()->getReferencePictureList(rpl0Idx);
  *activeL0 = rpl0->getNumberOfActivePictures();
  const ReferencePictureList *rpl1 = sps->getRPLList1()->getReferencePictureList(rpl1Idx);
  *activeL1 = rpl1->getNumberOfActivePictures();
}

void EncLib::selectReferencePictureList(Slice* slice, int POCCurr, int GOPid, int ltPoc)
{
  bool isEncodeLtRef = (POCCurr == ltPoc);
  if (m_compositeRefEnabled && isEncodeLtRef)
  {
    POCCurr++;
  }

  slice->setRPL0idx(GOPid);
  slice->setRPL1idx(GOPid);

  int fullListNum = m_iGOPSize;
  int partialListNum = getRPLCandidateSize(0) - m_iGOPSize;
  int extraNum = fullListNum;
  if (m_uiIntraPeriod < 0)
  {
    if (POCCurr < 10)
    {
      slice->setRPL0idx(POCCurr + m_iGOPSize - 1);
      slice->setRPL1idx(POCCurr + m_iGOPSize - 1);
    }
    else
    {
      slice->setRPL0idx((POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1);
      slice->setRPL1idx((POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1);
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum < fullListNum + partialListNum; extraNum++)
  {
    if (m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      int POCIndex = POCCurr%m_uiIntraPeriod;
      if (POCIndex == 0)
        POCIndex = m_uiIntraPeriod;
      if (POCIndex == m_RPLList0[extraNum].m_POC)
      {
        slice->setRPL0idx(extraNum);
        slice->setRPL1idx(extraNum);
        extraNum++;
      }
    }
  }

  const ReferencePictureList *rpl0 = (slice->getSPS()->getRPLList0()->getReferencePictureList(slice->getRPL0idx()));
  const ReferencePictureList *rpl1 = (slice->getSPS()->getRPLList1()->getReferencePictureList(slice->getRPL1idx()));
  slice->setRPL0(rpl0);
  slice->setRPL1(rpl1);
}

void  EncLib::xInitPPSforTiles(PPS &pps)
{
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
    if (m_rectSliceFlag)
      pps.setRectSliceFlag(m_rectSliceFlag);
  }
  pps.setUniformTileSpacingFlag( m_tileUniformSpacingFlag );
  pps.setNumTileColumnsMinus1( m_iNumColumnsMinus1 );
  pps.setNumTileRowsMinus1( m_iNumRowsMinus1 );
  if( !m_tileUniformSpacingFlag )
  {
    pps.setTileColumnWidth( m_tileColumnWidth );
    pps.setTileRowHeight( m_tileRowHeight );
  }
#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
  else
  {
    pps.setTileColsWidthMinus1(m_uniformTileColsWidthMinus1);
    pps.setTileRowsHeightMinus1(m_uniformTileRowHeightMinus1);
  }
#endif
  pps.setLoopFilterAcrossBricksEnabledFlag( m_loopFilterAcrossBricksEnabledFlag );

  //pps.setRectSliceFlag( m_rectSliceFlag );
  pps.setNumSlicesInPicMinus1( m_numSlicesInPicMinus1 );
  pps.setTopLeftBrickIdx(m_topLeftBrickIdx);
  pps.setBottomRightBrickIdx(m_bottomRightBrickIdx);
#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
  if (m_numSlicesInPicMinus1 > 0)
  {
    std::vector<int> bottomrightdelta(m_numSlicesInPicMinus1 + 1);
    for (int i = 0; i < m_numSlicesInPicMinus1 + 1; i++)
    {
      bottomrightdelta[i] = (i == 0) ? m_bottomRightBrickIdx[i] : m_bottomRightBrickIdx[i] - m_bottomRightBrickIdx[i - 1];
    }
    pps.setBottomRightBrickIdxDelta(bottomrightdelta);
  }
#endif

  pps.setLoopFilterAcrossBricksEnabledFlag( m_loopFilterAcrossBricksEnabledFlag );
  pps.setLoopFilterAcrossSlicesEnabledFlag( m_loopFilterAcrossSlicesEnabledFlag );
  pps.setSignalledSliceIdFlag( m_signalledSliceIdFlag );
  pps.setSignalledSliceIdLengthMinus1( m_signalledSliceIdLengthMinus1 );
  pps.setSignalledSliceIdFlag( m_signalledSliceIdFlag );
  pps.setSignalledSliceIdLengthMinus1( m_signalledSliceIdLengthMinus1 );
  pps.setSliceId( m_sliceId );

  int numTiles= (m_iNumColumnsMinus1 + 1) * (m_iNumRowsMinus1 + 1);
  pps.setNumTilesInPic(numTiles);
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
  std::vector<int> tileHeight(numTiles);
#endif

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
#if JVET_O0173_O0176_O0338_NUMBRICK_M2
    std::vector<int> numBrickRowsMinus2(numTiles, 0);
#else
    std::vector<int>  numBrickRowsMinus1 (numTiles, 0);
#endif
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
#if JVET_O0173_O0176_O0338_NUMBRICK_M2
        numBrickRowsMinus2[tileIdx] = brickSplit.second.m_numSplits - 1;
#else
        numBrickRowsMinus1[tileIdx]=brickSplit.second.m_numSplits;
#endif 
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
#if JVET_O0173_O0176_O0338_NUMBRICK_M2
    pps.setNumBrickRowsMinus2(numBrickRowsMinus2);
#else
    pps.setNumBrickRowsMinus1(numBrickRowsMinus1);
#endif 
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
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
      const int tileY = tileIdx / (m_iNumColumnsMinus1 + 1);

      tileHeight[tileIdx] = tileRowHeight[tileY];

      if (tileHeight[tileIdx] <= 1)
      {
        CHECK(pps.getBrickSplitFlag(tileIdx) != 0, "The value of brick_split_flag[ i ] shall be 0 if tileHeight <= 1");
      }
#endif
      if (pps.getBrickSplitFlag(tileIdx))
      {
#if !JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
        const int tileY = tileIdx / (m_iNumColumnsMinus1+1);

        int tileHeight = tileRowHeight [tileY];
#endif
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
        if (tileHeight[tileIdx] <= 2)
        {
          CHECK(pps.getUniformBrickSpacingFlag(tileIdx) != 1, "The value of uniform_brick_spacing_flag[ i ] shall be 1 if tileHeight <= 2");
        }
#endif
        if (pps.getUniformBrickSpacingFlag(tileIdx))
        {
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
          CHECK((pps.getBrickHeightMinus1(tileIdx) + 1) >= tileHeight[tileIdx], "Brick height larger than or equal to tile height");
#else
          CHECK((pps.getBrickHeightMinus1(tileIdx) + 1) >= tileHeight, "Brick height larger than or equal to tile height");
#endif
        }
        else
        {
          int cumulativeHeight=0;
#if JVET_O0173_O0176_O0338_NUMBRICK_M2
          for (int i = 0; i <= pps.getNumBrickRowsMinus2(tileIdx); i++)
#else 
          for (int i = 0; i < pps.getNumBrickRowsMinus1(tileIdx); i++)
#endif 
          {
            cumulativeHeight += pps.getBrickRowHeightMinus1(tileIdx, i) + 1;
          }
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
          CHECK(cumulativeHeight >= tileHeight[tileIdx], "Cumulative brick height larger than or equal to tile height");
#else
          CHECK(cumulativeHeight >= tileHeight, "Cumulative brick height larger than or equal to tile height");
#endif
        }
      }
    }
  }
#if JVET_O0452_PPS_BRICK_SIGNALING_CONDITION
  pps.setTileHeight(tileHeight);
#endif
}

void  EncCfg::xCheckGSParameters()
{
  int   iWidthInCU = ( m_iSourceWidth%m_maxCUWidth ) ? m_iSourceWidth/m_maxCUWidth + 1 : m_iSourceWidth/m_maxCUWidth;
  int   iHeightInCU = ( m_iSourceHeight%m_maxCUHeight ) ? m_iSourceHeight/m_maxCUHeight + 1 : m_iSourceHeight/m_maxCUHeight;
  uint32_t  uiCummulativeColumnWidth = 0;
  uint32_t  uiCummulativeRowHeight = 0;

#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
  if (m_tileUniformSpacingFlag && m_uniformTileColsWidthMinus1 == -1)
  {
    EXIT("Uniform tiles specified with unspecified or invalid UniformTileColsWidthMinus1 value");
  }
  if (m_tileUniformSpacingFlag && m_uniformTileRowHeightMinus1 == -1)
  {
    EXIT("Uniform tiles specified with unspecified or invalid UniformTileRowHeightMinus1 value");
  }
  if (m_tileUniformSpacingFlag && m_uniformTileColsWidthMinus1 >= iWidthInCU)
  {
    EXIT("UniformTileColsWidthMinus1 too large");
  }
  if (m_tileUniformSpacingFlag && m_uniformTileRowHeightMinus1 >= iHeightInCU)
  {
    EXIT("UniformTileRowHeightMinus1 too large");
  }
#endif

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

void EncLib::setParamSetChanged(int spsId, int ppsId)
{
  m_ppsMap.setChangedFlag(ppsId);
  m_spsMap.setChangedFlag(spsId);
}
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

#if JVET_O0119_BASE_PALETTE_444
void EncLib::checkPltStats( Picture* pic )
{
  int totalArea = 0;
  int pltArea = 0;
  for (auto apu : pic->cs->pus)
  {
    for (int i = 0; i < MAX_NUM_TBLOCKS; ++i)
    {
      int puArea = apu->blocks[i].width * apu->blocks[i].height;
      if (apu->blocks[i].width > 0 && apu->blocks[i].height > 0)
      {
        totalArea += puArea;
        if (CU::isPLT(*apu->cu) || CU::isIBC(*apu->cu))
        {
          pltArea += puArea;
        }
        break;
      }

    }
  }
  if (pltArea * PLT_FAST_RATIO < totalArea)
  {
    m_doPlt = false;
  }
  else
  {
    m_doPlt = true;
  }
}
#endif

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
