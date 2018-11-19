/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2018, ITU/ISO/IEC
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

/** \file     EncSearch.cpp
 *  \brief    encoder intra search class
 */

#include "IntraSearch.h"

#include "EncModeCtrl.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

#include <math.h>
#include <limits>

 //! \ingroup EncoderLib
 //! \{

IntraSearch::IntraSearch()
  : m_pSplitCS      (nullptr)
  , m_pFullCS       (nullptr)
  , m_pBestCS       (nullptr)
  , m_pcEncCfg      (nullptr)
  , m_pcTrQuant     (nullptr)
  , m_pcRdCost      (nullptr)
  , m_CABACEstimator(nullptr)
  , m_CtxCache      (nullptr)
  , m_isInitialized (false)
{
  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = nullptr;
  }
}


void IntraSearch::destroy()
{
  CHECK( !m_isInitialized, "Not initialized" );

  if( m_pcEncCfg )
  {
    bool BTnoRQT = m_pcEncCfg->getQTBT();


    const uint32_t uiNumLayersToAllocateSplit = BTnoRQT ? 1 : m_pcEncCfg->getQuadtreeTULog2MaxSize() - m_pcEncCfg->getQuadtreeTULog2MinSize() + 1;
    const uint32_t uiNumLayersToAllocateFull  = BTnoRQT ? 1 : m_pcEncCfg->getQuadtreeTULog2MaxSize() - m_pcEncCfg->getQuadtreeTULog2MinSize() + 1;
    const int uiNumSaveLayersToAllocate = 2;

    for( uint32_t layer = 0; layer < uiNumSaveLayersToAllocate; layer++ )
    {
      m_pSaveCS[layer]->destroy();
      delete m_pSaveCS[layer];
    }

    uint32_t numWidths  = gp_sizeIdxInfo->numWidths();
    uint32_t numHeights = gp_sizeIdxInfo->numHeights();

    for( uint32_t width = 0; width < numWidths; width++ )
    {
      for( uint32_t height = 0; height < numHeights; height++ )
      {
        if( ( BTnoRQT || width == height ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
        {
          for( uint32_t layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
          {
            m_pSplitCS[width][height][layer]->destroy();

            delete m_pSplitCS[width][height][layer];
          }

          for( uint32_t layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
          {
            m_pFullCS[width][height][layer]->destroy();

            delete m_pFullCS[width][height][layer];
          }

          delete[] m_pSplitCS[width][height];
          delete[] m_pFullCS [width][height];

          m_pBestCS[width][height]->destroy();
          m_pTempCS[width][height]->destroy();

          delete m_pTempCS[width][height];
          delete m_pBestCS[width][height];
        }
      }

      delete[] m_pSplitCS[width];
      delete[] m_pFullCS [width];

      delete[] m_pTempCS[width];
      delete[] m_pBestCS[width];
    }

    delete[] m_pSplitCS;
    delete[] m_pFullCS;

    delete[] m_pBestCS;
    delete[] m_pTempCS;

    delete[] m_pSaveCS;
  }

  m_pSplitCS = m_pFullCS = nullptr;

  m_pBestCS = m_pTempCS = nullptr;

  m_pSaveCS = nullptr;

  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    delete[] m_pSharedPredTransformSkip[ch];
    m_pSharedPredTransformSkip[ch] = nullptr;
  }

  m_isInitialized = false;
}

IntraSearch::~IntraSearch()
{
  if( m_isInitialized )
  {
    destroy();
  }
}

void IntraSearch::init( EncCfg*        pcEncCfg,
                        TrQuant*       pcTrQuant,
                        RdCost*        pcRdCost,
                        CABACWriter*   CABACEstimator,
                        CtxCache*      ctxCache,
                        const uint32_t     maxCUWidth,
                        const uint32_t     maxCUHeight,
                        const uint32_t     maxTotalCUDepth
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_pcEncCfg                     = pcEncCfg;
  m_pcTrQuant                    = pcTrQuant;
  m_pcRdCost                     = pcRdCost;
  m_CABACEstimator               = CABACEstimator;
  m_CtxCache                     = ctxCache;

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();

  IntraPrediction::init( cform, pcEncCfg->getBitDepth( CHANNEL_TYPE_LUMA ) );

  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }

  uint32_t numWidths  = gp_sizeIdxInfo->numWidths();
  uint32_t numHeights = gp_sizeIdxInfo->numHeights();

  bool BTnoRQT = m_pcEncCfg->getQTBT();

  const uint32_t uiNumLayersToAllocateSplit = BTnoRQT ? 1 : pcEncCfg->getQuadtreeTULog2MaxSize() - pcEncCfg->getQuadtreeTULog2MinSize() + 1;
  const uint32_t uiNumLayersToAllocateFull  = BTnoRQT ? 1 : pcEncCfg->getQuadtreeTULog2MaxSize() - pcEncCfg->getQuadtreeTULog2MinSize() + 1;

  m_pBestCS = new CodingStructure**[numWidths];
  m_pTempCS = new CodingStructure**[numWidths];

  m_pFullCS  = new CodingStructure***[numWidths];
  m_pSplitCS = new CodingStructure***[numWidths];

  for( uint32_t width = 0; width < numWidths; width++ )
  {
    m_pBestCS[width] = new CodingStructure*[numHeights];
    m_pTempCS[width] = new CodingStructure*[numHeights];

    m_pFullCS [width] = new CodingStructure**[numHeights];
    m_pSplitCS[width] = new CodingStructure**[numHeights];

    for( uint32_t height = 0; height < numHeights; height++ )
    {
      if( ( BTnoRQT || width == height ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
      {
        m_pBestCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pTempCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pBestCS[width][height]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        m_pTempCS[width][height]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        m_pFullCS [width][height] = new CodingStructure*[uiNumLayersToAllocateFull];
        m_pSplitCS[width][height] = new CodingStructure*[uiNumLayersToAllocateSplit];

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
        {
          m_pFullCS [width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

          m_pFullCS [width][height][layer]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        }

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
        {
          m_pSplitCS[width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

          m_pSplitCS[width][height][layer]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        }
      }
      else
      {
        m_pBestCS[width][height] = nullptr;
        m_pTempCS[width][height] = nullptr;

        m_pFullCS [width][height] = nullptr;
        m_pSplitCS[width][height] = nullptr;
      }
    }
  }

  const int uiNumSaveLayersToAllocate = 2;

  m_pSaveCS = new CodingStructure*[uiNumSaveLayersToAllocate];

  for( uint32_t depth = 0; depth < uiNumSaveLayersToAllocate; depth++ )
  {
    m_pSaveCS[depth] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
    m_pSaveCS[depth]->create( UnitArea( cform, Area( 0, 0, maxCUWidth, maxCUHeight ) ), false );
  }

  m_isInitialized = true;
}


//////////////////////////////////////////////////////////////////////////
// INTRA PREDICTION
//////////////////////////////////////////////////////////////////////////

void IntraSearch::estIntraPredLumaQT( CodingUnit &cu, Partitioner &partitioner )
{
  CodingStructure       &cs            = *cu.cs;
  const SPS             &sps           = *cs.sps;
  const uint32_t             uiWidthBit    = cs.pcv->rectCUs ? g_aucLog2[partitioner.currArea().lwidth() ] : CU::getIntraSizeIdx(cu);
  const uint32_t             uiHeightBit   =                   g_aucLog2[partitioner.currArea().lheight()];

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantization divisor is 1.
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(cu.transQuantBypass) / double(1 << SCALE_BITS);


  //===== loop over partitions =====

  const TempCtx ctxStart          ( m_CtxCache, m_CABACEstimator->getCtx() );
  const TempCtx ctxStartIntraMode ( m_CtxCache, SubCtx( Ctx::IPredMode[CHANNEL_TYPE_LUMA],        m_CABACEstimator->getCtx() ) );
#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
  const TempCtx ctxStartMHIntraMode ( m_CtxCache, SubCtx( Ctx::MHIntraPredMode,        m_CABACEstimator->getCtx() ) );
#endif
#if JVET_L0283_MULTI_REF_LINE
  const TempCtx ctxStartMrlIdx      ( m_CtxCache, SubCtx( Ctx::MultiRefLineIdx,        m_CABACEstimator->getCtx() ) );
#endif

  CHECK( !cu.firstPU, "CU has no PUs" );
  const bool keepResi   = cs.pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() || KEEP_PRED_AND_RESI_SIGNALS;


  uint32_t extraModes = 0; // add two extra modes, which would be used after uiMode <= DC_IDX is removed for cu.nsstIdx == 3


  const int width   = partitioner.currArea().lwidth();
  const int height  = partitioner.currArea().lheight();

  // Marking EMT usage for faster EMT
  // 0: EMT is either not applicable for current CU (cuWidth > EMT_INTRA_MAX_CU or cuHeight > EMT_INTRA_MAX_CU), not active in the config file or the fast decision algorithm is not used in this case
  // 1: EMT fast algorithm can be applied for the current CU, and the DCT2 is being checked
  // 2: EMT is being checked for current CU. Stored results of DCT2 can be utilized for speedup
  uint8_t emtUsageFlag = 0;
  const int maxSizeEMT = cs.pcv->noRQT ? EMT_INTRA_MAX_CU_WITH_QTBT : EMT_INTRA_MAX_CU;
  if( width <= maxSizeEMT && height <= maxSizeEMT && sps.getSpsNext().getUseIntraEMT() )
  {
    emtUsageFlag = cu.emtFlag == 1 ? 2 : 1;
  }

  bool isAllIntra = m_pcEncCfg->getIntraPeriod() == 1;

  if( cs.pcv->rectCUs )
  {
    if( width * height < 64 && !isAllIntra )
    {
      emtUsageFlag = 0; //this forces the recalculation of the candidates list. Why is this necessary? (to be checked)
    }
  }

  static_vector<uint32_t,   FAST_UDI_MAX_RDMODE_NUM> uiHadModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandCostList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandHadList;

#if JVET_L0283_MULTI_REF_LINE
  static_vector<int, FAST_UDI_MAX_RDMODE_NUM> extendRefList;
  static_vector<int, FAST_UDI_MAX_RDMODE_NUM>* nullList = NULL;
#endif

  auto &pu = *cu.firstPU;
  int puIndex = 0;
  {
    CandHadList.clear();
    CandCostList.clear();
    uiHadModeList.clear();
#if JVET_L0283_MULTI_REF_LINE
    extendRefList.clear();
#endif

    CHECK(pu.cu != &cu, "PU is not contained in the CU");

    //===== determine set of modes to be tested (using prediction signal only) =====
    int numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes
    static_vector< uint32_t, FAST_UDI_MAX_RDMODE_NUM > uiRdModeList;

    int numModesForFullRD = 3;
    if( cs.pcv->rectCUs )
    {
      numModesForFullRD = g_aucIntraModeNumFast_UseMPM_2D[uiWidthBit - MIN_CU_LOG2][uiHeightBit - MIN_CU_LOG2];
    }
    else
    {
      numModesForFullRD = m_pcEncCfg->getFastUDIUseMPMEnabled() ? g_aucIntraModeNumFast_UseMPM[uiWidthBit] : g_aucIntraModeNumFast_NotUseMPM[uiWidthBit];
      numModesForFullRD -= 1;
    }

#if INTRA_FULL_SEARCH
    numModesForFullRD = numModesAvailable;
#endif


    if( emtUsageFlag != 2 )
    {
      // this should always be true
      CHECK( !pu.Y().valid(), "PU is not valid" );
#if JVET_L0283_MULTI_REF_LINE
      bool isFirstLineOfCtu = (((pu.block(COMPONENT_Y).y)&((pu.cs->sps)->getMaxCUWidth() - 1)) == 0);
      int numOfPassesExtendRef = (isFirstLineOfCtu ? 1 : MRL_NUM_REF_LINES);
      pu.multiRefIdx = 0;
#endif

      //===== init pattern for luma prediction =====
      initIntraPatternChType( cu, pu.Y(), IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, pu, false, pu ) );
      if( numModesForFullRD != numModesAvailable )
      {
        CHECK( numModesForFullRD >= numModesAvailable, "Too many modes for full RD search" );

        const CompArea &area = pu.Y();

        PelBuf piOrg         = cs.getOrgBuf(area);
        PelBuf piPred        = cs.getPredBuf(area);

        DistParam distParam;

        const bool bUseHadamard = cu.transQuantBypass == 0;

        m_pcRdCost->setDistParam(distParam, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

        distParam.applyWeight = false;

        bool bSatdChecked[NUM_INTRA_MODE];
        memset( bSatdChecked, 0, sizeof( bSatdChecked ) );

        {
          for( int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
          {
            uint32_t       uiMode = modeIdx;
            Distortion uiSad  = 0;

            // Skip checking extended Angular modes in the first round of SATD
            if( uiMode > DC_IDX && ( uiMode & 1 ) )
            {
              continue;
            }

            bSatdChecked[uiMode] = true;

            pu.intraDir[0] = modeIdx;

            if( useDPCMForFirstPassIntraEstimation( pu, uiMode ) )
            {
              encPredIntraDPCM( COMPONENT_Y, piOrg, piPred, uiMode );
            }
            else
            {
              predIntraAng( COMPONENT_Y, piPred, pu, IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, pu, true, pu ) );
            }
            // use Hadamard transform here
            uiSad += distParam.distFunc(distParam);

            // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
            m_CABACEstimator->getCtx() = SubCtx( Ctx::IPredMode[CHANNEL_TYPE_LUMA], ctxStartIntraMode );
#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
            m_CABACEstimator->getCtx() = SubCtx( Ctx::MHIntraPredMode, ctxStartMHIntraMode );
#endif
#if JVET_L0283_MULTI_REF_LINE
            m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );
#endif

            uint64_t fracModeBits = xFracModeBitsIntra(pu, uiMode, CHANNEL_TYPE_LUMA);

            double cost = ( double ) uiSad + ( double ) fracModeBits * sqrtLambdaForFirstPass;

            DTRACE( g_trace_ctx, D_INTRA_COST, "IntraHAD: %u, %llu, %f (%d)\n", uiSad, fracModeBits, cost, uiMode );

            updateCandList( uiMode, cost,  uiRdModeList, CandCostList
#if JVET_L0283_MULTI_REF_LINE
              , extendRefList, 0
#endif              
              , numModesForFullRD + extraModes );
            updateCandList(uiMode, (double) uiSad, uiHadModeList, CandHadList
#if JVET_L0283_MULTI_REF_LINE
              , *nullList, -1
#endif              
              , 3 + extraModes);
          }
        } // NSSTFlag

        // forget the extra modes
        uiRdModeList.resize( numModesForFullRD );
#if JVET_L0283_MULTI_REF_LINE
        CandCostList.resize(numModesForFullRD);
        extendRefList.resize(numModesForFullRD);
#endif
        static_vector<unsigned, FAST_UDI_MAX_RDMODE_NUM> parentCandList(FAST_UDI_MAX_RDMODE_NUM);
        std::copy_n(uiRdModeList.begin(), numModesForFullRD, parentCandList.begin());

        // Second round of SATD for extended Angular modes
        for (int modeIdx = 0; modeIdx < numModesForFullRD; modeIdx++)
        {
          unsigned parentMode = parentCandList[modeIdx];
          if (parentMode > (DC_IDX + 1) && parentMode < (NUM_LUMA_MODE - 1))
          {
            for (int subModeIdx = -1; subModeIdx <= 1; subModeIdx += 2)
            {
              unsigned mode = parentMode + subModeIdx;


              if (!bSatdChecked[mode])
              {
                pu.intraDir[0] = mode;

                if (useDPCMForFirstPassIntraEstimation(pu, mode))
                {
                  encPredIntraDPCM(COMPONENT_Y, piOrg, piPred, mode);
                }
                else
                {
                  predIntraAng(COMPONENT_Y, piPred, pu,
                               IntraPrediction::useFilteredIntraRefSamples(COMPONENT_Y, pu, true, pu));
                }
                // use Hadamard transform here
                Distortion sad = distParam.distFunc(distParam);

                // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
                m_CABACEstimator->getCtx() = SubCtx( Ctx::IPredMode[CHANNEL_TYPE_LUMA], ctxStartIntraMode );
#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
                m_CABACEstimator->getCtx() = SubCtx( Ctx::MHIntraPredMode, ctxStartMHIntraMode );
#endif
#if JVET_L0283_MULTI_REF_LINE
                m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );
#endif

                uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

                double cost = (double) sad + (double) fracModeBits * sqrtLambdaForFirstPass;

                updateCandList(mode, cost, uiRdModeList, CandCostList
#if JVET_L0283_MULTI_REF_LINE
                  , extendRefList, 0
#endif
                  , numModesForFullRD);
                updateCandList(mode, (double)sad, uiHadModeList, CandHadList
#if JVET_L0283_MULTI_REF_LINE
                  , *nullList, -1
#endif                 
                  , 3);

                bSatdChecked[mode] = true;
              }
            }
          }
        }
#if JVET_L0283_MULTI_REF_LINE
        pu.multiRefIdx = 1;
        const int  numMPMs = NUM_MOST_PROBABLE_MODES;
        unsigned  multiRefMPM [numMPMs];
        PU::getIntraMPMs(pu, multiRefMPM);
        for (int mRefNum = 1; mRefNum < numOfPassesExtendRef; mRefNum++)
        {
          int multiRefIdx = MULTI_REF_LINE_IDX[mRefNum];

          pu.multiRefIdx = multiRefIdx;
          {
            initIntraPatternChType(cu, pu.Y(), IntraPrediction::useFilteredIntraRefSamples(COMPONENT_Y, pu, false, pu));
          }
          for (int x = 0; x < numMPMs; x++)
          {
            uint32_t mode = multiRefMPM[x];
            {
              pu.intraDir[0] = mode;

              if (useDPCMForFirstPassIntraEstimation(pu, mode))
              {
                encPredIntraDPCM(COMPONENT_Y, piOrg, piPred, mode);
              }
              else
              {
                predIntraAng(COMPONENT_Y, piPred, pu, IntraPrediction::useFilteredIntraRefSamples(COMPONENT_Y, pu, true, pu));
              }

              // use Hadamard transform here
              Distortion sad = distParam.distFunc(distParam);

              // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
              m_CABACEstimator->getCtx() = SubCtx( Ctx::IPredMode[CHANNEL_TYPE_LUMA], ctxStartIntraMode );
#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MHIntraPredMode, ctxStartMHIntraMode );
#endif
#if JVET_L0283_MULTI_REF_LINE
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );
#endif

              uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

              double cost = (double)sad + (double)fracModeBits * sqrtLambdaForFirstPass;
              updateCandList(mode, cost, uiRdModeList, CandCostList, extendRefList, multiRefIdx, numModesForFullRD);
            }
          }
        }
        CandCostList.resize(numModesForFullRD);
        extendRefList.resize(numModesForFullRD);
#endif
        if( m_pcEncCfg->getFastUDIUseMPMEnabled() )
        {
          const int numMPMs = NUM_MOST_PROBABLE_MODES;
          unsigned  uiPreds[numMPMs];

#if JVET_L0283_MULTI_REF_LINE
          pu.multiRefIdx = 0;
#endif

          const int numCand = PU::getIntraMPMs( pu, uiPreds );

          for( int j = 0; j < numCand; j++ )
          {
            bool mostProbableModeIncluded = false;
            int  mostProbableMode         = uiPreds[j];


            for( int i = 0; i < numModesForFullRD; i++ )
            {
#if JVET_L0283_MULTI_REF_LINE
              mostProbableModeIncluded |= (mostProbableMode == uiRdModeList[i] && extendRefList[i] == 0);
#else
              mostProbableModeIncluded |= ( mostProbableMode == uiRdModeList[i] );
#endif
            }
            if( !mostProbableModeIncluded )
            {
#if JVET_L0283_MULTI_REF_LINE
              extendRefList.push_back(0);
#endif
              numModesForFullRD++;
              uiRdModeList.push_back( mostProbableMode );
            }
          }
        }
      }
      else
      {
        for( int i = 0; i < numModesForFullRD; i++ )
        {
          uiRdModeList.push_back( i );
        }
      }
      if( emtUsageFlag == 1 )
      {
        // Store the modes to be checked with RD
        m_savedNumRdModes[puIndex] = numModesForFullRD;
        std::copy_n( uiRdModeList.begin(), numModesForFullRD, m_savedRdModeList[puIndex] );
#if JVET_L0283_MULTI_REF_LINE
        std::copy_n(extendRefList.begin(), numModesForFullRD, m_savedExtendRefList[puIndex]);
#endif
      }
    }
    else //emtUsage = 2 (here we potentially reduce the number of modes that will be full-RD checked)
    {
      if( isAllIntra && m_pcEncCfg->getFastIntraEMT() )
      {
        double thresholdSkipMode;
        if( cs.pcv->noRQT )
        {
          thresholdSkipMode = 1.0 + 1.4 / sqrt( ( double ) ( width*height ) );
        }
        else
        {
          switch( width )
          {
          case  4: thresholdSkipMode = 1.47; break; // Skip checking   4x4 Intra modes using the R-D cost in the DCT2-pass
          case  8: thresholdSkipMode = 1.28; break; // Skip checking   8x8 Intra modes using the R-D cost in the DCT2-pass
          case 16: thresholdSkipMode = 1.12; break; // Skip checking 16x16 Intra modes using the R-D cost in the DCT2-pass
          case 32: thresholdSkipMode = 1.06; break; // Skip checking 32x32 Intra modes using the R-D cost in the DCT2-pass
          default: thresholdSkipMode = 1.06; break; // Skip checking 32x32 Intra modes using the R-D cost in the DCT2-pass
          }
        }

        numModesForFullRD = 0;

        // Skip checking the modes with much larger R-D cost than the best mode
        for( int i = 0; i < m_savedNumRdModes[puIndex]; i++ )
        {
          if( m_modeCostStore[puIndex][i] <= thresholdSkipMode * m_bestModeCostStore[puIndex] )
          {
            uiRdModeList.push_back( m_savedRdModeList[puIndex][i] );
#if JVET_L0283_MULTI_REF_LINE
            extendRefList.push_back(m_savedExtendRefList[puIndex][i]);
#endif
            numModesForFullRD++;
          }
        }
      }
      else //this is necessary because we skip the candidates list calculation, since it was already obtained for the DCT-II. Now we load it
      {
        // Restore the modes to be checked with RD
        numModesForFullRD = m_savedNumRdModes[puIndex];
        uiRdModeList.resize( numModesForFullRD );
        std::copy_n( m_savedRdModeList[puIndex], m_savedNumRdModes[puIndex], uiRdModeList.begin() );
#if JVET_L0283_MULTI_REF_LINE
        CandCostList.resize(numModesForFullRD);
        extendRefList.resize(numModesForFullRD);
        std::copy_n(m_savedExtendRefList[puIndex], m_savedNumRdModes[puIndex], extendRefList.begin());
#endif
      }
    }


    CHECK( numModesForFullRD != uiRdModeList.size(), "Inconsistent state!" );

    // after this point, don't use numModesForFullRD

    // PBINTRA fast
    if( m_pcEncCfg->getUsePbIntraFast() && !cs.slice->isIntra() && cu.partSize == SIZE_2Nx2N && uiRdModeList.size() < numModesAvailable && emtUsageFlag != 2 )
    {
      if( CandHadList.size() < 3 || CandHadList[2] > cs.interHad * PBINTRA_RATIO )
      {
        uiRdModeList.resize( std::min<size_t>( uiRdModeList.size(), 2 ) );
      }
      if( CandHadList.size() < 2 || CandHadList[1] > cs.interHad * PBINTRA_RATIO )
      {
        uiRdModeList.resize( std::min<size_t>( uiRdModeList.size(), 1 ) );
      }
      if( CandHadList.size() < 1 || CandHadList[0] > cs.interHad * PBINTRA_RATIO )
      {
        cs.dist = std::numeric_limits<Distortion>::max();
        cs.interHad = 0;

        //===== reset context models =====
        m_CABACEstimator->getCtx() = SubCtx( Ctx::IPredMode[CHANNEL_TYPE_LUMA], ctxStartIntraMode );
#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
        m_CABACEstimator->getCtx() = SubCtx( Ctx::MHIntraPredMode, ctxStartMHIntraMode );
#endif
#if JVET_L0283_MULTI_REF_LINE
        m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );
#endif

        return;
      }
    }

    //===== check modes (using r-d costs) =====
    uint32_t       uiBestPUMode  = 0;
#if JVET_L0283_MULTI_REF_LINE
    int            bestExtendRef = 0;
#endif

    CodingStructure *csTemp = m_pTempCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];
    CodingStructure *csBest = m_pBestCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];

    csTemp->slice = cs.slice;
    csBest->slice = cs.slice;
    csTemp->initStructData();
    csBest->initStructData();

    // just to be sure
    numModesForFullRD = ( int ) uiRdModeList.size();
    for (uint32_t uiMode = 0; uiMode < numModesForFullRD; uiMode++)
    {
      // set luma prediction mode
      uint32_t uiOrgMode = uiRdModeList[uiMode];

      pu.intraDir[0] = uiOrgMode;
#if JVET_L0283_MULTI_REF_LINE
      int multiRefIdx = extendRefList[uiMode];
      pu.multiRefIdx  = multiRefIdx;
      CHECK(pu.multiRefIdx && (pu.intraDir[0] == DC_IDX || pu.intraDir[0] == PLANAR_IDX), "ERL");
#endif


      // set context models
      m_CABACEstimator->getCtx() = ctxStart;

      // determine residual for partition
      cs.initSubStructure( *csTemp, partitioner.chType, cs.area, true );

      xRecurIntraCodingLumaQT( *csTemp, partitioner );

      if( emtUsageFlag == 1 && m_pcEncCfg->getFastIntraEMT() )
      {
        m_modeCostStore[puIndex][uiMode] = csTemp->cost; //cs.cost;
      }


      DTRACE( g_trace_ctx, D_INTRA_COST, "IntraCost T %f (%d) \n", csTemp->cost, uiOrgMode );

      // check r-d cost
      if( csTemp->cost < csBest->cost )
      {
        std::swap( csTemp, csBest );

        uiBestPUMode  = uiOrgMode;
#if JVET_L0283_MULTI_REF_LINE
        bestExtendRef = multiRefIdx;
#endif

        if( ( emtUsageFlag == 1 ) && m_pcEncCfg->getFastIntraEMT() )
        {
          m_bestModeCostStore[puIndex] = csBest->cost; //cs.cost;
        }
      }

      csTemp->releaseIntermediateData();
    } // Mode loop

    cs.useSubStructure( *csBest, partitioner.chType, pu.singleChan( CHANNEL_TYPE_LUMA ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );

    csBest->releaseIntermediateData();
    //=== update PU data ====
    pu.intraDir[0] = uiBestPUMode;
#if JVET_L0283_MULTI_REF_LINE
    pu.multiRefIdx = bestExtendRef;
#endif
  }

  //===== reset context models =====
  m_CABACEstimator->getCtx() = ctxStart;
}

void IntraSearch::estIntraPredChromaQT(CodingUnit &cu, Partitioner &partitioner)
{
  const ChromaFormat format   = cu.chromaFormat;
  const uint32_t    numberValidComponents = getNumberValidComponents(format);
  CodingStructure &cs = *cu.cs;
  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );

  cs.setDecomp( cs.area.Cb(), false );

  auto &pu = *cu.firstPU;

  {
    uint32_t       uiBestMode = 0;
    Distortion uiBestDist = 0;
    double     dBestCost = MAX_DOUBLE;

    //----- init mode list ----
    {
      uint32_t  uiMinMode = 0;
      uint32_t  uiMaxMode = NUM_CHROMA_MODE;

      //----- check chroma modes -----
      uint32_t chromaCandModes[ NUM_CHROMA_MODE ];
      PU::getIntraChromaCandModes( pu, chromaCandModes );

      // create a temporary CS
      CodingStructure &saveCS = *m_pSaveCS[0];
      saveCS.pcv      = cs.pcv;
      saveCS.picture  = cs.picture;
      saveCS.area.repositionTo( cs.area );
      saveCS.clearTUs();

      if( CS::isDualITree( cs ) )
      {
        if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
        {
          partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );

          do
          {
            cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType ), partitioner.chType ).depth = partitioner.currTrDepth;
          } while( partitioner.nextPart( cs ) );

          partitioner.exitCurrSplit();
        }
        else
        cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType ), partitioner.chType );
      }

      std::vector<TransformUnit*> orgTUs;


      // create a store for the TUs
      for( const auto &ptu : cs.tus )
      {
        // for split TUs in HEVC, add the TUs without Chroma parts for correct setting of Cbfs
        if( pu.contains( *ptu, CHANNEL_TYPE_CHROMA ) || ( !cs.pcv->noRQT && !ptu->Cb().valid() && !ptu->Cr().valid() ) )
        {
          saveCS.addTU( *ptu, partitioner.chType );
          orgTUs.push_back( ptu );
        }
      }
#if JVET_L0338_MDLM
      // SATD pre-selecting.
      int satdModeList[NUM_CHROMA_MODE];
      int64_t satdSortedCost[NUM_CHROMA_MODE];
      for (int i = 0; i < NUM_CHROMA_MODE; i++)
      {
        satdSortedCost[i] = 0; // for the mode not pre-select by SATD, do RDO by default, so set the initial value 0.
        satdModeList[i] = 0;
      }
      bool modeIsEnable[NUM_INTRA_MODE + 1]; // use intra mode idx to check whether enable
      for (int i = 0; i < NUM_INTRA_MODE + 1; i++)
      {
        modeIsEnable[i] = 1;
      }

      DistParam distParam;
      const bool useHadamard = true;
      pu.intraDir[1] = MDLM_L_IDX; // temporary assigned, just to indicate this is a MDLM mode. for luma down-sampling operation.

      initIntraPatternChType(cu, pu.Cb());
      initIntraPatternChType(cu, pu.Cr());
      xGetLumaRecPixels(pu, pu.Cb());

      for (int idx = uiMinMode; idx <= uiMaxMode - 1; idx++)
      {
        int mode = chromaCandModes[idx];
        satdModeList[idx] = mode;
        if (PU::isLMCMode(mode) && !PU::isLMCModeEnabled(pu, mode))
        {
          continue;
        }
        if ((mode == LM_CHROMA_IDX) || (mode == PLANAR_IDX) || (mode == DM_CHROMA_IDX)) // only pre-check regular modes and MDLM modes, not including DM ,Planar, and LM
        {
          continue;
        }
        pu.intraDir[1] = mode; // temporary assigned, for SATD checking.

        int64_t sad = 0;
        CodingStructure& cs = *(pu.cs);

        CompArea areaCb = pu.Cb();
        PelBuf orgCb = cs.getOrgBuf(areaCb);
        PelBuf predCb = cs.getPredBuf(areaCb);

        m_pcRdCost->setDistParam(distParam, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, useHadamard);
        distParam.applyWeight = false;

        if (PU::isLMCMode(mode))
        {
          predIntraChromaLM(COMPONENT_Cb, predCb, pu, areaCb, mode);
        }
        else
        {
          predIntraAng(COMPONENT_Cb, predCb, pu, false);
        }

        sad += distParam.distFunc(distParam);

        CompArea areaCr = pu.Cr();
        PelBuf orgCr = cs.getOrgBuf(areaCr);
        PelBuf predCr = cs.getPredBuf(areaCr);

        m_pcRdCost->setDistParam(distParam, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, useHadamard);
        distParam.applyWeight = false;

        if (PU::isLMCMode(mode))
        {
          predIntraChromaLM(COMPONENT_Cr, predCr, pu, areaCr, mode);
        }
        else
        {
          predIntraAng(COMPONENT_Cr, predCr, pu, false);
        }
        sad += distParam.distFunc(distParam);
        satdSortedCost[idx] = sad;
      }
      // sort the mode based on the cost from small to large.
      int tempIdx = 0;
      int64_t tempCost = 0;
      for (int i = uiMinMode; i <= uiMaxMode - 1; i++)
      {
        for (int j = i + 1; j <= uiMaxMode - 1; j++)
        {
          if (satdSortedCost[j] < satdSortedCost[i])
          {
            tempIdx = satdModeList[i];
            satdModeList[i] = satdModeList[j];
            satdModeList[j] = tempIdx;

            tempCost = satdSortedCost[i];
            satdSortedCost[i] = satdSortedCost[j];
            satdSortedCost[j] = tempCost;

          }
        }
      }
      int reducedModeNumber = 2; // reduce the number of chroma modes
      for (int i = 0; i < reducedModeNumber; i++)
      {
        modeIsEnable[satdModeList[uiMaxMode - 1 - i]] = 0; // disable the last reducedModeNumber modes
      }
#endif

      // save the dist
      Distortion baseDist = cs.dist;

      for (uint32_t uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++)
      {
        const int chromaIntraMode = chromaCandModes[uiMode];
        if( PU::isLMCMode( chromaIntraMode ) && ! PU::isLMCModeEnabled( pu, chromaIntraMode ) )
        {
          continue;
        }
#if JVET_L0338_MDLM
        if (!modeIsEnable[chromaIntraMode] && PU::isLMCModeEnabled(pu, chromaIntraMode)) // when CCLM is disable, then MDLM is disable. not use satd checking
        {
          continue;
        }
#endif
        cs.setDecomp( pu.Cb(), false );
        cs.dist = baseDist;
        //----- restore context models -----
        m_CABACEstimator->getCtx() = ctxStart;

        //----- chroma coding -----
        pu.intraDir[1] = chromaIntraMode;

        xRecurIntraChromaCodingQT( cs, partitioner );

        if (cs.pps->getUseTransformSkip())
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        uint64_t fracBits   = xGetIntraFracBitsQT( cs, partitioner, false, true );
        Distortion uiDist = cs.dist;
        double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

        //----- compare -----
        if( dCost < dBestCost )
        {
          for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
          {
            const CompArea &area = pu.blocks[i];

            saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
            saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
            cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );

            for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
            {
              saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
            }
          }

          dBestCost  = dCost;
          uiBestDist = uiDist;
          uiBestMode = chromaIntraMode;
        }
      }

      for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
      {
        const CompArea &area = pu.blocks[i];

        cs.getRecoBuf         ( area ).copyFrom( saveCS.getRecoBuf( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf         ( area ).copyFrom( saveCS.getPredBuf( area ) );
        cs.getResiBuf         ( area ).copyFrom( saveCS.getResiBuf( area ) );
#endif
        cs.picture->getRecoBuf( area ).copyFrom( cs.    getRecoBuf( area ) );

        for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
        {
          orgTUs[ j ]->copyComponentFrom( *saveCS.tus[ j ], area.compID );
        }
      }
    }

    pu.intraDir[1] = uiBestMode;
    cs.dist        = uiBestDist;
  }

  //----- restore context models -----
  m_CABACEstimator->getCtx() = ctxStart;
}

void IntraSearch::IPCMSearch(CodingStructure &cs, Partitioner& partitioner)
{
  for (uint32_t ch = 0; ch < getNumberValidTBlocks( *cs.pcv ); ch++)
  {
    const ComponentID compID = ComponentID(ch);

    xEncPCM(cs, partitioner, compID);
  }

  cs.getPredBuf().fill(0);
  cs.getResiBuf().fill(0);
  cs.getOrgResiBuf().fill(0);

  cs.dist     = 0;
  cs.fracBits = 0;
  cs.cost     = 0;

  cs.setDecomp(cs.area);
  cs.picture->getRecoBuf(cs.area).copyFrom(cs.getRecoBuf());
}

void IntraSearch::xEncPCM(CodingStructure &cs, Partitioner& partitioner, const ComponentID &compID)
{
  TransformUnit &tu = *cs.getTU( partitioner.chType );

  const int  channelBitDepth = cs.sps->getBitDepth(toChannelType(compID));
  const uint32_t uiPCMBitDepth = cs.sps->getPCMBitDepth(toChannelType(compID));

  const int pcmShiftRight = (channelBitDepth - int(uiPCMBitDepth));

  CompArea  area    = tu.blocks[compID];
  PelBuf    pcmBuf  = tu.getPcmbuf  (compID);
  PelBuf    recBuf  = cs.getRecoBuf ( area );
  CPelBuf   orgBuf  = cs.getOrgBuf  ( area );

  CHECK(pcmShiftRight < 0, "Negative shift");

  for (uint32_t uiY = 0; uiY < pcmBuf.height; uiY++)
  {
    for (uint32_t uiX = 0; uiX < pcmBuf.width; uiX++)
    {
      // Encode
      pcmBuf.at(uiX, uiY) = orgBuf.at(uiX, uiY) >> pcmShiftRight;
      // Reconstruction
      recBuf.at(uiX, uiY) = pcmBuf.at(uiX, uiY) << pcmShiftRight;
    }
  }
}

// -------------------------------------------------------------------------------------------------------------------
// Intra search
// -------------------------------------------------------------------------------------------------------------------

void IntraSearch::xEncIntraHeader(CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma)
{
  CodingUnit &cu = *cs.getCU( partitioner.chType );

  if (bLuma)
  {
    bool isFirst = partitioner.currArea().lumaPos() == cs.area.lumaPos();

    // CU header
    if( isFirst )
    {
      if( !cs.slice->isIntra() 
#if JVET_L0293_CPR
        && cu.Y().valid()
#endif
        )
      {
        if( cs.pps->getTransquantBypassEnabledFlag() )
        {
          m_CABACEstimator->cu_transquant_bypass_flag( cu );
        }
        m_CABACEstimator->cu_skip_flag( cu );
        m_CABACEstimator->pred_mode   ( cu );
      }
#if JVET_L0283_MULTI_REF_LINE
      m_CABACEstimator->extend_ref_line(cu);
#endif
      if( CU::isIntra(cu) && cu.partSize == SIZE_2Nx2N )
      {
        m_CABACEstimator->pcm_data( cu );
        if( cu.ipcm )
        {
          return;
        }
      }
    }

    PredictionUnit &pu = *cs.getPU(partitioner.currArea().lumaPos(), partitioner.chType);

    // luma prediction mode
    if (cu.partSize == SIZE_2Nx2N)
    {
      if (isFirst)
      {
#if JVET_L0293_CPR
        if ( !cu.Y().valid())
          m_CABACEstimator->pred_mode( cu );
#endif
        m_CABACEstimator->intra_luma_pred_mode( pu );
      }
    }
  }

  if (bChroma)
  {
    bool isFirst = partitioner.currArea().Cb().valid() && partitioner.currArea().chromaPos() == cs.area.chromaPos();

    PredictionUnit &pu = *cs.getPU( partitioner.currArea().chromaPos(), CHANNEL_TYPE_CHROMA );

    if( cu.partSize == SIZE_2Nx2N )
    {
      if( isFirst )
      {
        m_CABACEstimator->intra_chroma_pred_mode( pu );
      }
    }
  }
}

void IntraSearch::xEncSubdivCbfQT(CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma)
{
  const UnitArea &currArea = partitioner.currArea();
  TransformUnit &currTU    = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );
#if HM_EMT_NSST_AS_IN_JEM
  CodingUnit &currCU       = *currTU.cu;
#endif
  uint32_t currDepth           = partitioner.currTrDepth;

  const bool subdiv        = currTU.depth > currDepth;

  if( cs.pcv->noRQT )
  {
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      CHECK( !subdiv, "TU split implied" );
    }
    else
      CHECK( subdiv, "No TU subdivision is allowed with QTBT" );
  }

  if (bChroma)
  {
    const uint32_t numberValidComponents = getNumberValidComponents(currArea.chromaFormat);

    for (uint32_t ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
    {
      const ComponentID compID = ComponentID(ch);

      if( currDepth == 0 || TU::getCbfAtDepth( currTU, compID, currDepth - 1 ) )
      {
        const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth ) : false );
        m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, compID, currDepth ), currArea.blocks[compID], currDepth, prevCbf );

      }
    }
  }

  if (subdiv)
  {
#if HM_EMT_NSST_AS_IN_JEM
    if( currDepth == 0 && bLuma ) m_CABACEstimator->emt_cu_flag( currCU );
#endif

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else
    THROW( "Cannot perform an implicit split!" );

    do
    {
      xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma );
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
#if HM_EMT_NSST_AS_IN_JEM
    if( currDepth == 0 && bLuma && TU::getCbfAtDepth( currTU, COMPONENT_Y, 0 ) ) m_CABACEstimator->emt_cu_flag( currCU );

#endif
    //===== Cbfs =====
    if (bLuma)
    {
      m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, COMPONENT_Y, currDepth ), currTU.Y(), currTU.depth );
    }
  }
}

void IntraSearch::xEncCoeffQT(CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID)
{
  const UnitArea &currArea  = partitioner.currArea();
  TransformUnit &currTU     = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );
  uint32_t      currDepth       = partitioner.currTrDepth;
  const bool subdiv         = currTU.depth > currDepth;

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else
      THROW("Implicit TU split not available!");

    do
    {
      xEncCoeffQT( cs, partitioner, compID );
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else

  if( currArea.blocks[compID].valid() )
  {
    if( TU::hasCrossCompPredInfo( currTU, compID ) )
    {
      m_CABACEstimator->cross_comp_pred( currTU, compID );
    }
    if( TU::getCbf( currTU, compID ) )
    {
      m_CABACEstimator->residual_coding( currTU, compID );
    }
  }
}

uint64_t IntraSearch::xGetIntraFracBitsQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma )
{
  m_CABACEstimator->resetBits();

  xEncIntraHeader( cs, partitioner, bLuma, bChroma );
  xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma );

  if( bLuma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Y );
  }
  if( bChroma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Cb );
    xEncCoeffQT( cs, partitioner, COMPONENT_Cr );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTChroma(TransformUnit& currTU, const ComponentID &compID)
{
  m_CABACEstimator->resetBits();

  if( TU::hasCrossCompPredInfo( currTU, compID ) )
  {
    m_CABACEstimator->cross_comp_pred( currTU, compID );
  }
  if( TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

void IntraSearch::xIntraCodingTUBlock(TransformUnit &tu, const ComponentID &compID, const bool &checkCrossCPrediction, Distortion& ruiDist, const int &default0Save1Load2, uint32_t* numSig )
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }

  CodingStructure &cs                       = *tu.cs;

  const CompArea      &area                 = tu.blocks[compID];
  const SPS           &sps                  = *cs.sps;
  const PPS           &pps                  = *cs.pps;

  const ChannelType    chType               = toChannelType(compID);
  const int            bitDepth             = sps.getBitDepth(chType);

  PelBuf         piOrg                      = cs.getOrgBuf    (area);
  PelBuf         piPred                     = cs.getPredBuf   (area);
  PelBuf         piResi                     = cs.getResiBuf   (area);
  PelBuf         piOrgResi                  = cs.getOrgResiBuf(area);
  PelBuf         piReco                     = cs.getRecoBuf   (area);

  const PredictionUnit &pu                  = *cs.getPU(area.pos(), chType);
  const uint32_t           uiChFinalMode        = PU::getFinalIntraMode(pu, chType);

  const bool           bUseCrossCPrediction = pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && isChroma( compID ) && PU::isChromaIntraModeCrossCheckMode( pu ) && checkCrossCPrediction;
  const bool           ccUseRecoResi        = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();

#if !JVET_L0059_MTS_SIMP
  const uint8_t          transformIndex       = tu.cu->emtFlag && compID == COMPONENT_Y ? tu.emtIdx : DCT2_EMT ;
#endif

  //===== init availability pattern =====
  PelBuf sharedPredTS( m_pSharedPredTransformSkip[compID], area );
  if( default0Save1Load2 != 2 )
  {
    const bool bUseFilteredPredictions = IntraPrediction::useFilteredIntraRefSamples( compID, pu, true, tu );
    initIntraPatternChType( *tu.cu, area, bUseFilteredPredictions );

    //===== get prediction signal =====
    if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
    {
      {
        xGetLumaRecPixels( pu, area );
      }
      predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
    }
    else
    {
      predIntraAng( compID, piPred, pu, bUseFilteredPredictions );
    }


    // save prediction
    if( default0Save1Load2 == 1 )
    {
      sharedPredTS.copyFrom( piPred );
    }
  }
  else
  {
    // load prediction
    piPred.copyFrom( sharedPredTS );
  }


  DTRACE( g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), uiChFinalMode );
  //DTRACE_PEL_BUF( D_PRED, piPred, tu, tu.cu->predMode, COMPONENT_Y );

  //===== get residual signal =====
  piResi.copyFrom( piOrg  );
  piResi.subtract( piPred );

  if (pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && isLuma(compID))
  {
    piOrgResi.copyFrom (piResi);
  }

  if (bUseCrossCPrediction)
  {
    if (xCalcCrossComponentPredictionAlpha(tu, compID, ccUseRecoResi) == 0)
    {
      return;
    }
    CrossComponentPrediction::crossComponentPrediction(tu, compID, cs.getResiBuf(tu.Y()), piResi, piResi, false);
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  //--- transform and quantization           ---
  TCoeff uiAbsSum = 0;

  const QpParam cQP(tu, compID);

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda(compID);
#endif


  m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx());

#if !JVET_L0059_MTS_SIMP
  if( transformIndex != DCT2_EMT && ( !tu.transformSkip[COMPONENT_Y] ) ) //this can only be true if compID is luma
  {
    *numSig = 0;
    TCoeff* coeffBuffer = tu.getCoeffs(compID).buf;
    for( uint32_t uiX = 0; uiX < tu.Y().area(); uiX++ )
    {
      if( coeffBuffer[uiX] )
      {
        ( *numSig )++;
        if( *numSig > g_EmtSigNumThr )
        {
          break;
        }
      }
    }
    //if the number of significant coeffs is less than the threshold, then only the default transform (which has a 0 index, but it is the DST7) is allowed
    if( transformIndex != 0 && *numSig <= g_EmtSigNumThr && !tu.transformSkip[compID] )
    {
      return;
    }
  }
#endif

  DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), compID, uiAbsSum );


  //--- inverse transform ---
  if (uiAbsSum > 0)
  {
    m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
  }
  else
  {
    piResi.fill(0);
  }

  //===== reconstruction =====
  if (bUseCrossCPrediction)
  {
    CrossComponentPrediction::crossComponentPrediction(tu, compID, cs.getResiBuf(tu.Y()), piResi, piResi, true);
  }

  piReco.reconstruct(piPred, piResi, cs.slice->clpRng( compID ));

  //===== update distortion =====
#if WCG_EXT
  if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
    ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE_WTD, &orgLuma );
  }
  else
#endif
  {
    ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE );
  }
}

void IntraSearch::xRecurIntraCodingLumaQT( CodingStructure &cs, Partitioner &partitioner )
{
  const UnitArea &currArea = partitioner.currArea();
  const CodingUnit &cu     = *cs.getCU(currArea.lumaPos(), partitioner.chType);
  uint32_t     currDepth       = partitioner.currTrDepth;
  const PPS &pps           = *cs.pps;
  const bool keepResi      = pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() || KEEP_PRED_AND_RESI_SIGNALS;
  bool bCheckFull          = true;
  bool bCheckSplit         = false;
  bCheckFull               = cs.pcv->noRQT && !partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
  bCheckSplit              = cs.pcv->noRQT &&  partitioner.canSplit( TU_MAX_TR_SPLIT, cs );

  uint32_t    numSig           = 0;

  if( !cs.pcv->noRQT )
  {
  }

  bool    checkInitTrDepth = false, checkInitTrDepthTransformSkipWinner = false;

  double     dSingleCost                        = MAX_DOUBLE;
  Distortion uiSingleDistLuma                   = 0;
  uint64_t     singleFracBits                     = 0;
  bool       checkTransformSkip                 = pps.getUseTransformSkip();
  int        bestModeId[MAX_NUM_COMPONENT]      = {0, 0, 0};
  uint8_t      nNumTransformCands                 = cu.emtFlag ? 4 : 1; //4 is the number of transforms of emt
  bool       isAllIntra                         = m_pcEncCfg->getIntraPeriod() == 1;

  uint8_t numTransformIndexCands                  = nNumTransformCands;

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;

  if( bCheckSplit )
  {
    csSplit = &cs;
  }
  else if( bCheckFull )
  {
    csFull = &cs;
  }

  if( bCheckFull )
  {
    csFull->cost = 0.0;

    TransformUnit &tu = csFull->addTU( CS::getArea( *csFull, currArea, partitioner.chType ), partitioner.chType );
    tu.depth = currDepth;

    checkTransformSkip &= TU::hasTransformSkipFlag( *tu.cs, tu.Y() );
    checkTransformSkip &= !cu.transQuantBypass;
    checkTransformSkip &= !cu.emtFlag;

    CHECK( !tu.Y().valid(), "Invalid TU" );

    //this prevents transformSkip from being checked because we already know it's not the best mode
    checkTransformSkip = ( checkInitTrDepth && !checkInitTrDepthTransformSkipWinner ) ? false : checkTransformSkip;


    CHECK( checkInitTrDepthTransformSkipWinner && !checkTransformSkip, "Transform Skip must be enabled if it was the winner in the previous call of xRecurIntraCodingLumaQT!" );

    CodingStructure &saveCS = *m_pSaveCS[0];

    TransformUnit *tmpTU = nullptr;

    Distortion singleDistTmpLuma = 0;
    uint64_t     singleTmpFracBits = 0;
    double     singleCostTmp     = 0;
    int        firstCheckId      = 0;

    //we add the EMT candidates to the loop. TransformSkip will still be the last one to be checked (when modeId == lastCheckId) as long as checkTransformSkip is true
    int        lastCheckId       = numTransformIndexCands - ( firstCheckId + 1 ) + ( int ) checkTransformSkip;
    bool isNotOnlyOneMode        = lastCheckId != firstCheckId && !checkInitTrDepthTransformSkipWinner;

    if( isNotOnlyOneMode )
    {
      saveCS.pcv     = cs.pcv;
      saveCS.picture = cs.picture;
      saveCS.area.repositionTo(cs.area);
      saveCS.clearTUs();
      tmpTU = &saveCS.addTU(currArea, partitioner.chType);
    }

    bool cbfBestMode = false;


    for( int modeId = firstCheckId; modeId <= lastCheckId; modeId++ )
    {
      if( checkInitTrDepthTransformSkipWinner )
      {
        //If this is a full RQT call and the winner of the first call (checkFirst=true) was transformSkip, then we skip the first iteration of the loop, since transform skip always comes at the end
        if( modeId == firstCheckId )
        {
          continue;
        }
      }

      uint8_t transformIndex = modeId;


      if( ( transformIndex < lastCheckId ) || ( ( transformIndex == lastCheckId ) && !checkTransformSkip ) ) //we avoid this if the mode is transformSkip
      {
        // Skip checking other transform candidates if zero CBF is encountered and it is the best transform so far
        if( m_pcEncCfg->getFastIntraEMT() && isAllIntra && transformIndex && !cbfBestMode )
        {
          continue;
        }
      }

      if ((modeId != firstCheckId) && isNotOnlyOneMode)
      {
        m_CABACEstimator->getCtx() = ctxStart;
      }

      int default0Save1Load2 = 0;
      singleDistTmpLuma = 0;

      if (modeId == firstCheckId && modeId != lastCheckId && !checkInitTrDepthTransformSkipWinner )
      {
        default0Save1Load2 = 1;
      }
      else if (modeId != firstCheckId)
      {
        default0Save1Load2 = 2;
      }

      if (cu.emtFlag)
      {
        tu.emtIdx = transformIndex;
      }
      if( !checkTransformSkip )
      {
        tu.transformSkip[COMPONENT_Y] = false;
      }
      else
      {
        tu.transformSkip[COMPONENT_Y] = modeId == lastCheckId;
      }

      xIntraCodingTUBlock( tu, COMPONENT_Y, false, singleDistTmpLuma, default0Save1Load2, &numSig );

      //----- determine rate and r-d cost -----
      //the condition (transformIndex != DCT2_EMT) seems to be irrelevant, since DCT2_EMT=7 and the highest value of transformIndex is 4
#if JVET_L0059_MTS_SIMP
      if( ( modeId == lastCheckId && checkTransformSkip && !TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth ) ) )
#else
      if( ( modeId == lastCheckId && checkTransformSkip && !TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth ) )
        || ( tu.emtIdx > 0 && ( checkTransformSkip ? transformIndex != lastCheckId : true ) && tu.emtIdx != DCT2_EMT && numSig <= g_EmtSigNumThr ) )
#endif
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        singleCostTmp = MAX_DOUBLE;
      }
      else
      {
        singleTmpFracBits = xGetIntraFracBitsQT( *csFull, partitioner, true, false );
        singleCostTmp     = m_pcRdCost->calcRdCost( singleTmpFracBits, singleDistTmpLuma );
      }

      if (singleCostTmp < dSingleCost)
      {
        dSingleCost       = singleCostTmp;
        uiSingleDistLuma  = singleDistTmpLuma;
        singleFracBits    = singleTmpFracBits;

        bestModeId[COMPONENT_Y] = modeId;
        cbfBestMode       = TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth );


        if( bestModeId[COMPONENT_Y] != lastCheckId )
        {
#if KEEP_PRED_AND_RESI_SIGNALS
          saveCS.getPredBuf( tu.Y() ).copyFrom( csFull->getPredBuf( tu.Y() ) );
#endif
          saveCS.getRecoBuf( tu.Y() ).copyFrom( csFull->getRecoBuf( tu.Y() ) );

          if( keepResi )
          {
            saveCS.getResiBuf   ( tu.Y() ).copyFrom( csFull->getResiBuf   ( tu.Y() ) );
            saveCS.getOrgResiBuf( tu.Y() ).copyFrom( csFull->getOrgResiBuf( tu.Y() ) );
          }

          tmpTU->copyComponentFrom( tu, COMPONENT_Y );

          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }

    if( bestModeId[COMPONENT_Y] != lastCheckId )
    {
#if KEEP_PRED_AND_RESI_SIGNALS
      csFull->getPredBuf( tu.Y() ).copyFrom( saveCS.getPredBuf( tu.Y() ) );
#endif
      csFull->getRecoBuf( tu.Y() ).copyFrom( saveCS.getRecoBuf( tu.Y() ) );

      if( keepResi )
      {
        csFull->getResiBuf   ( tu.Y() ).copyFrom( saveCS.getResiBuf   ( tu.Y() ) );
        csFull->getOrgResiBuf( tu.Y() ).copyFrom( saveCS.getOrgResiBuf( tu.Y() ) );
      }

      tu.copyComponentFrom( *tmpTU, COMPONENT_Y );

      if( !bCheckSplit )
      {
        m_CABACEstimator->getCtx() = ctxBest;
      }
    }
    else if( bCheckSplit )
    {
      ctxBest = m_CABACEstimator->getCtx();
    }

    csFull->cost     += dSingleCost;
    csFull->dist     += uiSingleDistLuma;
    csFull->fracBits += singleFracBits;
  }

  if( bCheckSplit )
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if( bCheckFull )
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }
    //----- code splitted block -----
    csSplit->cost = 0;

    bool uiSplitCbfLuma  = false;
    bool splitIsSelected = true;
    if( cs.pcv->noRQT && partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }

    do
    {
      xRecurIntraCodingLumaQT( *csSplit, partitioner );

      csSplit->setDecomp( partitioner.currArea().Y() );

      uiSplitCbfLuma |= TU::getCbfAtDepth( *csSplit->getTU( partitioner.currArea().lumaPos(), partitioner.chType ), COMPONENT_Y, partitioner.currTrDepth );



    } while( partitioner.nextPart( *csSplit ) );

    partitioner.exitCurrSplit();

    if( splitIsSelected )
    {
      for( auto &ptu : csSplit->tus )
      {
        if( currArea.Y().contains( ptu->Y() ) )
        {
          TU::setCbfAtDepth( *ptu, COMPONENT_Y, currDepth, uiSplitCbfLuma ? 1 : 0 );
        }
      }

      //----- restore context states -----
      m_CABACEstimator->getCtx() = ctxStart;

      //----- determine rate and r-d cost -----
      csSplit->fracBits = xGetIntraFracBitsQT(*csSplit, partitioner, true, false);

      //--- update cost ---
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);
    }
  }

  if( csFull || csSplit )
  {
    {
      // otherwise this would've happened in useSubStructure
      cs.picture->getRecoBuf( currArea.Y() ).copyFrom( cs.getRecoBuf( currArea.Y() ) );
    }

    cs.cost = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );
  }
}

ChromaCbfs IntraSearch::xRecurIntraChromaCodingQT(CodingStructure &cs, Partitioner& partitioner)
{
  UnitArea currArea                   = partitioner.currArea();
  const bool keepResi                 = cs.sps->getSpsNext().getUseLMChroma() || KEEP_PRED_AND_RESI_SIGNALS;
  if( !currArea.Cb().valid() ) return ChromaCbfs( false );


  TransformUnit &currTU               = *cs.getTU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );
  const PredictionUnit &pu            = *cs.getPU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );
  const TransformUnit &currTULuma     = CS::isDualITree( cs ) ? *cs.picture->cs->getTU( currArea.lumaPos(), CHANNEL_TYPE_LUMA ) : currTU;

  uint32_t     currDepth                  = partitioner.currTrDepth;
  const PPS &pps                      = *cs.pps;
  ChromaCbfs cbfs                     ( false );

  if (currDepth == currTU.depth)
  {
    if (!currArea.Cb().valid() || !currArea.Cr().valid())
    {
      return cbfs;
    }

    bool checkTransformSkip = pps.getUseTransformSkip();
    checkTransformSkip &= TU::hasTransformSkipFlag( *currTU.cs, partitioner.currArea().Cb() );

    if( m_pcEncCfg->getUseTransformSkipFast() )
    {
      checkTransformSkip &= TU::hasTransformSkipFlag( *currTU.cs, partitioner.currArea().Y() );

      if( checkTransformSkip && cs.pcv->noChroma2x2 )
      {
        int nbLumaSkip = currTULuma.transformSkip[0] ? 1 : 0;

        {
          // the chroma blocks are co-located with the last luma block, so backwards references are needed
          nbLumaSkip += cs.getTU( currTULuma.Y().topLeft().offset( -1,  0 ), partitioner.chType )->transformSkip[0] ? 1 : 0;
          nbLumaSkip += cs.getTU( currTULuma.Y().topLeft().offset( -1, -1 ), partitioner.chType )->transformSkip[0] ? 1 : 0;
          nbLumaSkip += cs.getTU( currTULuma.Y().topLeft().offset(  0, -1 ), partitioner.chType )->transformSkip[0] ? 1 : 0;
        }

        checkTransformSkip &= ( nbLumaSkip > 0 );
      }
    }

    CodingStructure &saveCS = *m_pSaveCS[1];
    saveCS.pcv      = cs.pcv;
    saveCS.picture  = cs.picture;
    saveCS.area.repositionTo( cs.area );
    saveCS.initStructData( -1, false, true );

    TransformUnit &tmpTU = saveCS.addTU(currArea, partitioner.chType);


    cs.setDecomp(currArea.Cb(), true); // set in advance (required for Cb2/Cr2 in 4:2:2 video)

    const unsigned      numTBlocks  = ::getNumberValidTBlocks( *cs.pcv );

    for( uint32_t c = COMPONENT_Cb; c < numTBlocks; c++)
    {
      const ComponentID compID  = ComponentID(c);
      const CompArea&   area    = currTU.blocks[compID];

      double     dSingleCost    = MAX_DOUBLE;
      int        bestModeId     = 0;
      Distortion singleDistC    = 0;
      Distortion singleDistCTmp = 0;
      double     singleCostTmp  = 0;

      const bool checkCrossComponentPrediction = PU::isChromaIntraModeCrossCheckMode( pu ) && pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && TU::getCbf( currTU, COMPONENT_Y );

      const int  crossCPredictionModesToTest = checkCrossComponentPrediction ? 2 : 1;
      const int  transformSkipModesToTest    = checkTransformSkip ? 2 : 1;
      const int  totalModesToTest            = crossCPredictionModesToTest * transformSkipModesToTest;
      const bool isOneMode                   = (totalModesToTest == 1);

      int currModeId = 0;
      int default0Save1Load2 = 0;

      TempCtx ctxStart  ( m_CtxCache );
      TempCtx ctxBest   ( m_CtxCache );

      if (!isOneMode)
      {
        ctxStart = m_CABACEstimator->getCtx();
      }

      for (int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
      {
        for (int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
        {
          currTU.compAlpha    [compID] = 0;
          currTU.transformSkip[compID] = transformSkipModeId;

          currModeId++;

          const bool isFirstMode = (currModeId == 1);
          const bool isLastMode  = (currModeId == totalModesToTest); // currModeId is indexed from 1

          if (isOneMode)
          {
            default0Save1Load2 = 0;
          }
          else if (!isOneMode && (transformSkipModeId == 0) && (crossCPredictionModeId == 0))
          {
            default0Save1Load2 = 1; //save prediction on first mode
          }
          else
          {
            default0Save1Load2 = 2; //load it on subsequent modes
          }

          if (!isFirstMode) // if not first mode to be tested
          {
            m_CABACEstimator->getCtx() = ctxStart;
          }

          singleDistCTmp = 0;

          xIntraCodingTUBlock( currTU, compID, crossCPredictionModeId != 0, singleDistCTmp, default0Save1Load2 );

          if( ( ( crossCPredictionModeId == 1 ) && ( currTU.compAlpha[compID] == 0 ) ) || ( ( transformSkipModeId == 1 ) && !TU::getCbf( currTU, compID ) ) ) //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else if( !isOneMode )
          {
            uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma( currTU, compID );
            singleCostTmp = m_pcRdCost->calcRdCost( fracBitsTmp, singleDistCTmp );
          }

          if( singleCostTmp < dSingleCost )
          {
            dSingleCost = singleCostTmp;
            singleDistC = singleDistCTmp;
            bestModeId  = currModeId;

            if( !isLastMode )
            {
#if KEEP_PRED_AND_RESI_SIGNALS
              saveCS.getPredBuf   (area).copyFrom(cs.getPredBuf   (area));
              saveCS.getOrgResiBuf(area).copyFrom(cs.getOrgResiBuf(area));
#endif
              if( keepResi )
              {
                saveCS.getResiBuf (area).copyFrom(cs.getResiBuf   (area));
              }
              saveCS.getRecoBuf   (area).copyFrom(cs.getRecoBuf   (area));

              tmpTU.copyComponentFrom(currTU, compID);

              ctxBest = m_CABACEstimator->getCtx();
            }
          }
        }
      }

      if (bestModeId < totalModesToTest)
      {
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf   (area).copyFrom(saveCS.getPredBuf   (area));
        cs.getOrgResiBuf(area).copyFrom(saveCS.getOrgResiBuf(area));
#endif
        if( keepResi )
        {
          cs.getResiBuf (area).copyFrom(saveCS.getResiBuf   (area));
        }
        cs.getRecoBuf   (area).copyFrom(saveCS.getRecoBuf   (area));

        currTU.copyComponentFrom(tmpTU, compID);

        m_CABACEstimator->getCtx() = ctxBest;
      }

      cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));

      cbfs.cbf(compID) = TU::getCbf(currTU, compID);

      cs.dist += singleDistC;
    }
  }
  else
  {
    unsigned    numValidTBlocks   = ::getNumberValidTBlocks( *cs.pcv );
    ChromaCbfs  SplitCbfs         ( false );

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else
      THROW( "Implicit TU split not available" );

    do
    {
      ChromaCbfs subCbfs = xRecurIntraChromaCodingQT( cs, partitioner );

      for( uint32_t ch = COMPONENT_Cb; ch < numValidTBlocks; ch++ )
      {
        const ComponentID compID = ComponentID( ch );
        SplitCbfs.cbf( compID ) |= subCbfs.cbf( compID );
      }
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();

    {

      cbfs.Cb |= SplitCbfs.Cb;
      cbfs.Cr |= SplitCbfs.Cr;

      for( auto &ptu : cs.tus )
      {
        if( currArea.Cb().contains( ptu->Cb() ) || ( !ptu->Cb().valid() && currArea.Y().contains( ptu->Y() ) ) )
        {
          TU::setCbfAtDepth( *ptu, COMPONENT_Cb, currDepth, SplitCbfs.Cb );
          TU::setCbfAtDepth( *ptu, COMPONENT_Cr, currDepth, SplitCbfs.Cr );
        }
      }
    }
  }

  return cbfs;
}

uint64_t IntraSearch::xFracModeBitsIntra(PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &chType)
{
  uint32_t orgMode = uiMode;

#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
  if (!pu.mhIntraFlag)
#endif
  std::swap(orgMode, pu.intraDir[chType]);

  m_CABACEstimator->resetBits();

  if( isLuma( chType ) )
  {
#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
    if ( pu.mhIntraFlag )
      m_CABACEstimator->MHIntra_luma_pred_modes(*pu.cu);
    else
    {
#if JVET_L0283_MULTI_REF_LINE
      m_CABACEstimator->extend_ref_line(pu);
#endif
      m_CABACEstimator->intra_luma_pred_mode(pu);
    }
#else
#if JVET_L0283_MULTI_REF_LINE
    m_CABACEstimator->extend_ref_line(pu);
#endif
    m_CABACEstimator->intra_luma_pred_mode( pu );
#endif
  }
  else
  {
    m_CABACEstimator->intra_chroma_pred_mode( pu );
  }

#if JVET_L0100_MULTI_HYPOTHESIS_INTRA
  if ( !pu.mhIntraFlag )
#endif
  std::swap(orgMode, pu.intraDir[chType]);

  return m_CABACEstimator->getEstFracBits();
}



void IntraSearch::encPredIntraDPCM( const ComponentID &compID, PelBuf &pOrg, PelBuf &pDst, const uint32_t &uiDirMode )
{
  CHECK( pOrg.buf == 0, "Encoder DPCM called without original buffer" );

  const int srcStride = m_topRefLength + 1;
  CPelBuf   pSrc = CPelBuf(getPredictorPtr(compID), srcStride, m_leftRefLength + 1);

  // Sample Adaptive intra-Prediction (SAP)
  if( uiDirMode == HOR_IDX )
  {
    // left column filled with reference samples, remaining columns filled with pOrg data
    for( int y = 0; y < pDst.height; y++ )
    {
      pDst.at( 0, y ) = pSrc.at( 0, 1 + y );
    }
    CPelBuf orgRest  = pOrg.subBuf( 0, 0, pOrg.width - 1, pOrg.height );
    PelBuf  predRest = pDst.subBuf( 1, 0, pDst.width - 1, pDst.height );

    predRest.copyFrom( orgRest );
  }
  else // VER_IDX
  {
    // top row filled with reference samples, remaining rows filled with pOrg data
    for( int x = 0; x < pDst.width; x++ )
    {
      pDst.at( x, 0 ) = pSrc.at( 1 + x, 0 );
    }
    CPelBuf orgRest  = pOrg.subBuf( 0, 0, pOrg.width, pOrg.height - 1 );
    PelBuf  predRest = pDst.subBuf( 0, 1, pDst.width, pDst.height - 1 );

    predRest.copyFrom( orgRest );
  }
}

bool IntraSearch::useDPCMForFirstPassIntraEstimation( const PredictionUnit &pu, const uint32_t &uiDirMode )
{
  return CU::isRDPCMEnabled( *pu.cu ) && pu.cu->transQuantBypass && (uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
}
