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
#define PLTCtx(c) SubCtx( Ctx::Palette, c )
IntraSearch::IntraSearch()
  : m_pSplitCS      (nullptr)
  , m_pFullCS       (nullptr)
  , m_pBestCS       (nullptr)
  , m_pcEncCfg      (nullptr)
  , m_pcTrQuant     (nullptr)
  , m_pcRdCost      (nullptr)
  , m_pcReshape     (nullptr)
  , m_CABACEstimator(nullptr)
  , m_CtxCache      (nullptr)
  , m_isInitialized (false)
{
  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = nullptr;
  }
#if JVET_P0077_LINE_CG_PALETTE
  m_truncBinBits = nullptr;
  m_escapeNumBins = nullptr;
  m_minErrorIndexMap = nullptr;
  for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
  {
    m_indexError[i] = nullptr;
  }
  for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
  {
    m_statePtRDOQ[i] = nullptr;
  }
#endif
}


void IntraSearch::destroy()
{
  CHECK( !m_isInitialized, "Not initialized" );

  if( m_pcEncCfg )
  {
    const uint32_t uiNumLayersToAllocateSplit = 1;
    const uint32_t uiNumLayersToAllocateFull  = 1;
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
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
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

  m_tmpStorageLCU.destroy();
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
  m_colorTransResiBuf.destroy();
#endif 
  m_isInitialized = false;
#if JVET_P0077_LINE_CG_PALETTE
  if (m_truncBinBits != nullptr)
  {
    for (unsigned i = 0; i < m_symbolSize; i++)
    {
      delete[] m_truncBinBits[i];
      m_truncBinBits[i] = nullptr;
    }
    delete[] m_truncBinBits;
    m_truncBinBits = nullptr;
  }
  if (m_escapeNumBins != nullptr)
  {
    delete[] m_escapeNumBins;
    m_escapeNumBins = nullptr;
  }
  if (m_indexError[0] != nullptr)
  {
    for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
    {
      delete[] m_indexError[i];
      m_indexError[i] = nullptr;
    }
  }
  if (m_minErrorIndexMap != nullptr)
  {
    delete[] m_minErrorIndexMap;
    m_minErrorIndexMap = nullptr;
  }
  if (m_statePtRDOQ[0] != nullptr)
  {
    for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
    {
      delete[] m_statePtRDOQ[i];
      m_statePtRDOQ[i] = nullptr;
    }
  }
#endif
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
                       , EncReshape*   pcReshape
#if JVET_P0077_LINE_CG_PALETTE
                       , const unsigned bitDepthY
#endif
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_pcEncCfg                     = pcEncCfg;
  m_pcTrQuant                    = pcTrQuant;
  m_pcRdCost                     = pcRdCost;
  m_CABACEstimator               = CABACEstimator;
  m_CtxCache                     = ctxCache;
  m_pcReshape                    = pcReshape;

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();

  IntraPrediction::init( cform, pcEncCfg->getBitDepth( CHANNEL_TYPE_LUMA ) );
  m_tmpStorageLCU.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
  m_colorTransResiBuf.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
#endif

  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }

  uint32_t numWidths  = gp_sizeIdxInfo->numWidths();
  uint32_t numHeights = gp_sizeIdxInfo->numHeights();

  const uint32_t uiNumLayersToAllocateSplit = 1;
  const uint32_t uiNumLayersToAllocateFull  = 1;

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
      if(  gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
      {
        m_pBestCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pTempCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pBestCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
        m_pTempCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());

        m_pFullCS [width][height] = new CodingStructure*[uiNumLayersToAllocateFull];
        m_pSplitCS[width][height] = new CodingStructure*[uiNumLayersToAllocateSplit];

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
        {
          m_pFullCS [width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

          m_pFullCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
        }

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
        {
          m_pSplitCS[width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
          m_pSplitCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
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
    m_pSaveCS[depth]->create(UnitArea(cform, Area(0, 0, maxCUWidth, maxCUHeight)), false, (bool)pcEncCfg->getPLTMode());
  }

  m_isInitialized = true;
#if JVET_P0077_LINE_CG_PALETTE
  if (pcEncCfg->getPLTMode())
  {
    m_symbolSize = (1 << bitDepthY); // pixel values are within [0, SymbolSize-1] with size SymbolSize
    if (m_truncBinBits == nullptr)
    {
      m_truncBinBits = new uint16_t*[m_symbolSize];
      for (unsigned i = 0; i < m_symbolSize; i++)
      {
        m_truncBinBits[i] = new uint16_t[m_symbolSize + 1];
      }
    }
    if (m_escapeNumBins == nullptr)
    {
      m_escapeNumBins = new uint16_t[m_symbolSize];
    }
    initTBCTable(bitDepthY);
    if (m_indexError[0] == nullptr)
    {
      for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
      {
        m_indexError[i] = new double[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
      }
    }
    if (m_minErrorIndexMap == nullptr)
    {
      m_minErrorIndexMap = new uint8_t[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
    }
    if (m_statePtRDOQ[0] == nullptr)
    {
      for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
      {
        m_statePtRDOQ[i] = new uint8_t[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
      }
    }
  }
#endif
}


//////////////////////////////////////////////////////////////////////////
// INTRA PREDICTION
//////////////////////////////////////////////////////////////////////////
static constexpr double COST_UNKNOWN = -65536.0;

double IntraSearch::findInterCUCost( CodingUnit &cu )
{
  if( cu.isConsIntra() && !cu.slice->isIntra() )
  {
    //search corresponding inter CU cost
    for( int i = 0; i < m_numCuInSCIPU; i++ )
    {
      if( cu.lumaPos() == m_cuAreaInSCIPU[i].pos() && cu.lumaSize() == m_cuAreaInSCIPU[i].size() )
      {
        return m_cuCostInSCIPU[i];
      }
    }
  }
  return COST_UNKNOWN;
}

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
bool IntraSearch::estIntraPredLumaQT(CodingUnit &cu, Partitioner &partitioner, const double bestCostSoFar, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst, CodingStructure* bestCS)
#else
bool IntraSearch::estIntraPredLumaQT( CodingUnit &cu, Partitioner &partitioner, const double bestCostSoFar, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst )
#endif
{
  CodingStructure       &cs            = *cu.cs;
  const SPS             &sps           = *cs.sps;
  const uint32_t             uiWidthBit    = floorLog2(partitioner.currArea().lwidth() );
  const uint32_t             uiHeightBit   =                   floorLog2(partitioner.currArea().lheight());

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantization divisor is 1.
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( ) * FRAC_BITS_SCALE;

  //===== loop over partitions =====

  const TempCtx ctxStart          ( m_CtxCache, m_CABACEstimator->getCtx() );
  const TempCtx ctxStartMipFlag    ( m_CtxCache, SubCtx( Ctx::MipFlag,          m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartIspMode    ( m_CtxCache, SubCtx( Ctx::ISPMode,          m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartPlanarFlag ( m_CtxCache, SubCtx( Ctx::IntraLumaPlanarFlag, m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartIntraMode(m_CtxCache, SubCtx(Ctx::IntraLumaMpmFlag, m_CABACEstimator->getCtx()));
  const TempCtx ctxStartMrlIdx      ( m_CtxCache, SubCtx( Ctx::MultiRefLineIdx,        m_CABACEstimator->getCtx() ) );

  CHECK( !cu.firstPU, "CU has no PUs" );
  const bool keepResi   = cs.pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() || KEEP_PRED_AND_RESI_SIGNALS;

  // variables for saving fast intra modes scan results across multiple LFNST passes
  bool LFNSTLoadFlag = sps.getUseLFNST() && cu.lfnstIdx != 0;
  bool LFNSTSaveFlag = sps.getUseLFNST() && cu.lfnstIdx == 0;

  LFNSTSaveFlag &= sps.getUseIntraMTS() ? cu.mtsFlag == 0 : true;

  const uint32_t lfnstIdx = cu.lfnstIdx;
  double costInterCU = findInterCUCost( cu );


  const int width  = partitioner.currArea().lwidth();
  const int height = partitioner.currArea().lheight();

  // Marking MTS usage for faster MTS
  // 0: MTS is either not applicable for current CU (cuWidth > MTS_INTRA_MAX_CU_SIZE or cuHeight > MTS_INTRA_MAX_CU_SIZE), not active in the config file or the fast decision algorithm is not used in this case
  // 1: MTS fast algorithm can be applied for the current CU, and the DCT2 is being checked
  // 2: MTS is being checked for current CU. Stored results of DCT2 can be utilized for speedup
  uint8_t mtsUsageFlag = 0;
  const int maxSizeEMT = MTS_INTRA_MAX_CU_SIZE;
  if( width <= maxSizeEMT && height <= maxSizeEMT && sps.getUseIntraMTS() )
  {
    mtsUsageFlag = ( sps.getUseLFNST() && cu.mtsFlag == 1 ) ? 2 : 1;
  }

  if( width * height < 64 && !m_pcEncCfg->getUseFastLFNST() )
  {
    mtsUsageFlag = 0;
  }

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
  const bool colorTransformIsEnabled = sps.getUseColorTrans() && !CS::isDualITree(cs);
  const bool isFirstColorSpace       = colorTransformIsEnabled && ((m_pcEncCfg->getRGBFormatFlag() && cu.colorTransform) || (!m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform));
  const bool isSecondColorSpace      = colorTransformIsEnabled && ((m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform) || (!m_pcEncCfg->getRGBFormatFlag() && cu.colorTransform));
#endif

  double bestCurrentCost = bestCostSoFar;
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
  bool ispCanBeUsed   = sps.getUseISP() && cu.mtsFlag == 0 && cu.lfnstIdx == 0 && CU::canUseISP(width, height, cu.cs->sps->getMaxTbSize());
  bool saveDataForISP = ispCanBeUsed && (!colorTransformIsEnabled || isFirstColorSpace);
  bool testISP        = ispCanBeUsed && (!colorTransformIsEnabled || !cu.colorTransform);
#else
  bool testISP = sps.getUseISP() && cu.mtsFlag == 0 && cu.lfnstIdx == 0 && CU::canUseISP( width, height, cu.cs->sps->getMaxTbSize() );
#endif

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
  if ( saveDataForISP )
  {
    //reset the intra modes lists variables
    m_ispCandListHor.clear();
    m_ispCandListVer.clear();
  }
#endif
  if( testISP )
  {
    //reset the variables used for the tests
#if !JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
    m_ispCandListHor.clear();
    m_ispCandListVer.clear();
#endif
    m_regIntraRDListWithCosts.clear();
    int numTotalPartsHor = (int)width  >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_VERT_SPLIT));
    int numTotalPartsVer = (int)height >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_HORZ_SPLIT));
#if JVET_P1026_ISP_LFNST_COMBINATION
    m_ispTestedModes[0].init( numTotalPartsHor, numTotalPartsVer );
    //the total number of subpartitions is modified to take into account the cases where LFNST cannot be combined with ISP due to size restrictions
    numTotalPartsHor = sps.getUseLFNST() && CU::canUseLfnstWithISP(cu.Y(), HOR_INTRA_SUBPARTITIONS) ? numTotalPartsHor : 0;
    numTotalPartsVer = sps.getUseLFNST() && CU::canUseLfnstWithISP(cu.Y(), VER_INTRA_SUBPARTITIONS) ? numTotalPartsVer : 0;
    for (int j = 1; j < NUM_LFNST_NUM_PER_SET; j++)
    {
      m_ispTestedModes[j].init(numTotalPartsHor, numTotalPartsVer);
    }
#else
    m_ispTestedModes.init(numTotalPartsHor, numTotalPartsVer);
#endif
  }

#if JVET_P0059_CHROMA_BDPCM
  const bool testBDPCM = (sps.getBDPCMEnabled()!=0) && CU::bdpcmAllowed(cu, ComponentID(partitioner.chType)) && cu.mtsFlag == 0 && cu.lfnstIdx == 0;
#else
  const bool testBDPCM = sps.getBDPCMEnabledFlag() && CU::bdpcmAllowed( cu, ComponentID( partitioner.chType ) ) && cu.mtsFlag == 0 && cu.lfnstIdx == 0;
#endif
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> uiHadModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandCostList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandHadList;


  auto &pu = *cu.firstPU;
  bool validReturn = false;
  {
    CandHadList.clear();
    CandCostList.clear();
    uiHadModeList.clear();

    CHECK(pu.cu != &cu, "PU is not contained in the CU");

    //===== determine set of modes to be tested (using prediction signal only) =====
    int numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes
    const bool fastMip    = sps.getUseMIP() && m_pcEncCfg->getUseFastMIP();
#if JVET_P0803_COMBINED_MIP_CLEANUP
    const bool mipAllowed = sps.getUseMIP() && isLuma(partitioner.chType) && ((cu.lfnstIdx == 0) || allowLfnstWithMip(cu.firstPU->lumaSize()));
    const bool testMip = mipAllowed && !(cu.lwidth() > (8 * cu.lheight()) || cu.lheight() > (8 * cu.lwidth()));
    const bool supportedMipBlkSize = pu.lwidth() <= MIP_MAX_WIDTH && pu.lheight() <= MIP_MAX_HEIGHT; 
#else
    const bool mipAllowed = sps.getUseMIP() && isLuma(partitioner.chType) && pu.lwidth() <= cu.cs->sps->getMaxTbSize() && pu.lheight() <= cu.cs->sps->getMaxTbSize() && ((cu.lfnstIdx == 0) || allowLfnstWithMip(cu.firstPU->lumaSize()));
    const bool testMip    = mipAllowed && mipModesAvailable(pu.Y());
#endif

    static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> uiRdModeList;

    int numModesForFullRD = 3;
    numModesForFullRD = g_aucIntraModeNumFast_UseMPM_2D[uiWidthBit - MIN_CU_LOG2][uiHeightBit - MIN_CU_LOG2];

#if INTRA_FULL_SEARCH
    numModesForFullRD = numModesAvailable;
#endif

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
    if (isSecondColorSpace)
    {
      uiRdModeList.clear();
      if (m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx] > 0)
      {
        for (int i = 0; i < m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx]; i++)
        {
          uiRdModeList.push_back(m_savedRdModeFirstColorSpace[m_savedRdModeIdx][i]);
        }
      }
      else
      {
        return false;
      }
    }
    else
    {
#endif
    if( mtsUsageFlag != 2 )
    {
      // this should always be true
      CHECK( !pu.Y().valid(), "PU is not valid" );
      bool isFirstLineOfCtu = (((pu.block(COMPONENT_Y).y)&((pu.cs->sps)->getMaxCUWidth() - 1)) == 0);
      int numOfPassesExtendRef = ((!sps.getUseMRL() || isFirstLineOfCtu) ? 1 : MRL_NUM_REF_LINES);
      pu.multiRefIdx = 0;

      if( numModesForFullRD != numModesAvailable )
      {
        CHECK( numModesForFullRD >= numModesAvailable, "Too many modes for full RD search" );

        const CompArea &area = pu.Y();

        PelBuf piOrg         = cs.getOrgBuf(area);
        PelBuf piPred        = cs.getPredBuf(area);

        DistParam distParamSad;
        DistParam distParamHad;
#if JVET_P1006_PICTURE_HEADER
        if (cu.slice->getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
#else
        if (cu.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
#endif
        {
          CompArea      tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpOrg = m_tmpStorageLCU.getBuf(tmpArea);
          tmpOrg.copyFrom(piOrg);
          tmpOrg.rspSignal(m_pcReshape->getFwdLUT());
          m_pcRdCost->setDistParam(distParamSad, tmpOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false); // Use SAD cost
          m_pcRdCost->setDistParam(distParamHad, tmpOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y,  true); // Use HAD (SATD) cost
        }
        else
        {
          m_pcRdCost->setDistParam(distParamSad, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, false); // Use SAD cost
          m_pcRdCost->setDistParam(distParamHad, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y,  true); // Use HAD (SATD) cost
        }

        distParamSad.applyWeight = false;
        distParamHad.applyWeight = false;

        if( testMip && supportedMipBlkSize )
        {
          numModesForFullRD += fastMip? std::max(numModesForFullRD, floorLog2(std::min(pu.lwidth(), pu.lheight())) - 1) : numModesForFullRD;
        }
        const int numHadCand = (testMip ? 2 : 1) * 3;

        //*** Derive (regular) candidates using Hadamard
        cu.mipFlag = false;

        //===== init pattern for luma prediction =====
        initIntraPatternChType(cu, pu.Y(), true);
        bool bSatdChecked[NUM_INTRA_MODE];
        memset( bSatdChecked, 0, sizeof( bSatdChecked ) );

        if( !LFNSTLoadFlag )
        {
          for( int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
          {
            uint32_t       uiMode = modeIdx;
            Distortion minSadHad = 0;

            // Skip checking extended Angular modes in the first round of SATD
            if( uiMode > DC_IDX && ( uiMode & 1 ) )
            {
              continue;
            }

            bSatdChecked[uiMode] = true;

            pu.intraDir[0] = modeIdx;

            initPredIntraParams(pu, pu.Y(), sps);
              predIntraAng( COMPONENT_Y, piPred, pu);
            // Use the min between SAD and HAD as the cost criterion
            // SAD is scaled by 2 to align with the scaling of HAD
            minSadHad += std::min(distParamSad.distFunc(distParamSad)*2, distParamHad.distFunc(distParamHad));

            // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
            m_CABACEstimator->getCtx() = SubCtx( Ctx::MipFlag, ctxStartMipFlag );
            m_CABACEstimator->getCtx() = SubCtx( Ctx::ISPMode, ctxStartIspMode );
            m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
            m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
            m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );

            uint64_t fracModeBits = xFracModeBitsIntra(pu, uiMode, CHANNEL_TYPE_LUMA);

            double cost = ( double ) minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;

            DTRACE(g_trace_ctx, D_INTRA_COST, "IntraHAD: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, uiMode);

#if JVET_P0803_COMBINED_MIP_CLEANUP
            updateCandList( ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, uiMode ), cost,              uiRdModeList,  CandCostList, numModesForFullRD );
            updateCandList( ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, uiMode ), double(minSadHad), uiHadModeList, CandHadList,  numHadCand );
#else
            updateCandList( ModeInfo(false, 0, NOT_INTRA_SUBPARTITIONS, uiMode), cost,          uiRdModeList,  CandCostList, numModesForFullRD );
            updateCandList( ModeInfo(false, 0, NOT_INTRA_SUBPARTITIONS, uiMode), (double)minSadHad, uiHadModeList, CandHadList,  numHadCand );
#endif
          }
          if( !sps.getUseMIP() && LFNSTSaveFlag )
          {
            // save found best modes
            m_uiSavedNumRdModesLFNST   = numModesForFullRD;
            m_uiSavedRdModeListLFNST   = uiRdModeList;
            m_dSavedModeCostLFNST      = CandCostList;
            // PBINTRA fast
            m_uiSavedHadModeListLFNST  = uiHadModeList;
            m_dSavedHadListLFNST       = CandHadList;
            LFNSTSaveFlag              = false;
          }
        } // NSSTFlag
        if( !sps.getUseMIP() && LFNSTLoadFlag )
        {
          // restore saved modes
          numModesForFullRD = m_uiSavedNumRdModesLFNST;
          uiRdModeList      = m_uiSavedRdModeListLFNST;
          CandCostList      = m_dSavedModeCostLFNST;
          // PBINTRA fast
          uiHadModeList     = m_uiSavedHadModeListLFNST;
          CandHadList       = m_dSavedHadListLFNST;
        } // !LFNSTFlag

        if (!(sps.getUseMIP() && LFNSTLoadFlag))
        {
        static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> parentCandList = uiRdModeList;

        // Second round of SATD for extended Angular modes
        for (int modeIdx = 0; modeIdx < numModesForFullRD; modeIdx++)
        {
          unsigned parentMode = parentCandList[modeIdx].modeId;
          if (parentMode > (DC_IDX + 1) && parentMode < (NUM_LUMA_MODE - 1))
          {
            for (int subModeIdx = -1; subModeIdx <= 1; subModeIdx += 2)
            {
              unsigned mode = parentMode + subModeIdx;


              if (!bSatdChecked[mode])
              {
                pu.intraDir[0] = mode;

                initPredIntraParams(pu, pu.Y(), sps);
                  predIntraAng(COMPONENT_Y, piPred, pu );

                // Use the min between SAD and SATD as the cost criterion
                // SAD is scaled by 2 to align with the scaling of HAD
                Distortion minSadHad = std::min(distParamSad.distFunc(distParamSad)*2, distParamHad.distFunc(distParamHad));

                // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
                m_CABACEstimator->getCtx() = SubCtx( Ctx::MipFlag, ctxStartMipFlag );
                m_CABACEstimator->getCtx() = SubCtx( Ctx::ISPMode, ctxStartIspMode );
                m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
                m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
                m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );

                uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

                double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;

#if JVET_P0803_COMBINED_MIP_CLEANUP
                updateCandList( ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, mode ), cost,              uiRdModeList,  CandCostList, numModesForFullRD );
                updateCandList( ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, mode ), double(minSadHad), uiHadModeList, CandHadList,  numHadCand );
#else
                updateCandList( ModeInfo( false, 0, NOT_INTRA_SUBPARTITIONS, mode ), cost,        uiRdModeList,  CandCostList, numModesForFullRD );
                updateCandList( ModeInfo( false, 0, NOT_INTRA_SUBPARTITIONS, mode ), (double)minSadHad, uiHadModeList, CandHadList,  numHadCand );
#endif

                bSatdChecked[mode] = true;
              }
            }
          }
        }
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
        if ( saveDataForISP )
#else
        if ( testISP )
#endif
        {
          // we save the regular intra modes list
          m_ispCandListHor = uiRdModeList;
        }
        pu.multiRefIdx = 1;
        const int  numMPMs = NUM_MOST_PROBABLE_MODES;
        unsigned  multiRefMPM [numMPMs];
        PU::getIntraMPMs(pu, multiRefMPM);
        for (int mRefNum = 1; mRefNum < numOfPassesExtendRef; mRefNum++)
        {
          int multiRefIdx = MULTI_REF_LINE_IDX[mRefNum];

          pu.multiRefIdx = multiRefIdx;
          {
            initIntraPatternChType(cu, pu.Y(), true);
          }
          for (int x = 1; x < numMPMs; x++)
          {
            uint32_t mode = multiRefMPM[x];
            {
              pu.intraDir[0] = mode;
              initPredIntraParams(pu, pu.Y(), sps);

                predIntraAng(COMPONENT_Y, piPred, pu);

              // Use the min between SAD and SATD as the cost criterion
              // SAD is scaled by 2 to align with the scaling of HAD
              Distortion minSadHad = std::min(distParamSad.distFunc(distParamSad)*2, distParamHad.distFunc(distParamHad));

              // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MipFlag, ctxStartMipFlag );
              m_CABACEstimator->getCtx() = SubCtx( Ctx::ISPMode, ctxStartIspMode );
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );

              uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

              double cost = (double)minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;
#if JVET_P0803_COMBINED_MIP_CLEANUP
              updateCandList( ModeInfo( false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode ), cost,              uiRdModeList,  CandCostList, numModesForFullRD );
              updateCandList( ModeInfo( false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode ), double(minSadHad), uiHadModeList, CandHadList,  numHadCand );
#else
              updateCandList( ModeInfo( false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode ), cost,        uiRdModeList,  CandCostList, numModesForFullRD );
              updateCandList( ModeInfo( false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode ), (double)minSadHad, uiHadModeList, CandHadList,  numHadCand );
#endif
            }
          }
        }
        CHECKD( uiRdModeList.size() != numModesForFullRD, "Error: RD mode list size" );

        if (LFNSTSaveFlag && testMip && !allowLfnstWithMip(cu.firstPU->lumaSize())) // save a different set for the next run
        {
          // save found best modes
          m_uiSavedRdModeListLFNST = uiRdModeList;
          m_dSavedModeCostLFNST = CandCostList;
          // PBINTRA fast
          m_uiSavedHadModeListLFNST = uiHadModeList;
          m_dSavedHadListLFNST = CandHadList;
          m_uiSavedNumRdModesLFNST = g_aucIntraModeNumFast_UseMPM_2D[uiWidthBit - MIN_CU_LOG2][uiHeightBit - MIN_CU_LOG2];
          m_uiSavedRdModeListLFNST.resize(m_uiSavedNumRdModesLFNST);
          m_dSavedModeCostLFNST.resize(m_uiSavedNumRdModesLFNST);
          // PBINTRA fast
          m_uiSavedHadModeListLFNST.resize(3);
          m_dSavedHadListLFNST.resize(3);
          LFNSTSaveFlag = false;
        }
          //*** Derive MIP candidates using Hadamard
        if( testMip && ! supportedMipBlkSize )
        {
          // avoid estimation for unsupported blk sizes
          const int transpOff    = getNumModesMip( pu.Y() );
          const int numModesFull = (transpOff << 1);
          for( uint32_t uiModeFull = 0; uiModeFull < numModesFull; uiModeFull++ )
          {
            const bool     isTransposed = (uiModeFull >= transpOff ? true : false);
            const uint32_t uiMode       = (isTransposed ? uiModeFull - transpOff : uiModeFull);

            numModesForFullRD++;
            uiRdModeList.push_back( ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, uiMode) );
            CandCostList.push_back(0);
          }
        }
        else
          if (testMip)
          {
            cu.mipFlag = true;
            pu.multiRefIdx = 0;

            double mipHadCost[MAX_NUM_MIP_MODE] = { MAX_DOUBLE };

            initIntraPatternChType(cu, pu.Y());
#if JVET_P0803_COMBINED_MIP_CLEANUP
            initIntraMip( pu, pu.Y() );

            const int transpOff    = getNumModesMip( pu.Y() );
            const int numModesFull = (transpOff << 1);
            for( uint32_t uiModeFull = 0; uiModeFull < numModesFull; uiModeFull++ )
            {
              const bool     isTransposed = (uiModeFull >= transpOff ? true : false);
              const uint32_t uiMode       = (isTransposed ? uiModeFull - transpOff : uiModeFull);

              pu.mipTransposedFlag = isTransposed;
#else
            initIntraMip( pu );

            for (uint32_t uiMode = 0; uiMode < getNumModesMip(pu.Y()); uiMode++)
            {
#endif
              pu.intraDir[CHANNEL_TYPE_LUMA] = uiMode;
              predIntraMip(COMPONENT_Y, piPred, pu);

              // Use the min between SAD and HAD as the cost criterion
              // SAD is scaled by 2 to align with the scaling of HAD
              Distortion minSadHad = std::min(distParamSad.distFunc(distParamSad)*2, distParamHad.distFunc(distParamHad));

              m_CABACEstimator->getCtx() = SubCtx( Ctx::MipFlag, ctxStartMipFlag );

              uint64_t fracModeBits = xFracModeBitsIntra(pu, uiMode, CHANNEL_TYPE_LUMA);

              double cost = double(minSadHad) + double(fracModeBits) * sqrtLambdaForFirstPass;
#if JVET_P0803_COMBINED_MIP_CLEANUP
              mipHadCost[uiModeFull] = cost;
              DTRACE(g_trace_ctx, D_INTRA_COST, "IntraMIP: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, uiModeFull);

              updateCandList( ModeInfo( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, uiMode ), cost,                  uiRdModeList,  CandCostList, numModesForFullRD + 1 );
              updateCandList( ModeInfo( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, uiMode ), 0.8*double(minSadHad), uiHadModeList, CandHadList,  numHadCand );
#else
              mipHadCost[uiMode] = cost;
              DTRACE(g_trace_ctx, D_INTRA_COST, "IntraMIP: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, uiMode);

              updateCandList(ModeInfo(true, 0, NOT_INTRA_SUBPARTITIONS, uiMode), cost, uiRdModeList, CandCostList, numModesForFullRD + 1);
              updateCandList(ModeInfo(true, 0, NOT_INTRA_SUBPARTITIONS, uiMode), 0.8*double(minSadHad), uiHadModeList, CandHadList, numHadCand);
#endif
            }

            const double thresholdHadCost = 1.0 + 1.4 / sqrt((double)(pu.lwidth()*pu.lheight()));
            reduceHadCandList(uiRdModeList, CandCostList, numModesForFullRD, thresholdHadCost, mipHadCost, pu, fastMip);
          }
          if ( sps.getUseMIP() && LFNSTSaveFlag)
          {
            // save found best modes
            m_uiSavedNumRdModesLFNST = numModesForFullRD;
            m_uiSavedRdModeListLFNST = uiRdModeList;
            m_dSavedModeCostLFNST = CandCostList;
            // PBINTRA fast
            m_uiSavedHadModeListLFNST = uiHadModeList;
            m_dSavedHadListLFNST = CandHadList;
            LFNSTSaveFlag = false;
          }
        }
        else //if( sps.getUseMIP() && LFNSTLoadFlag)
        {
          // restore saved modes
          numModesForFullRD = m_uiSavedNumRdModesLFNST;
          uiRdModeList = m_uiSavedRdModeListLFNST;
          CandCostList = m_dSavedModeCostLFNST;
          // PBINTRA fast
          uiHadModeList = m_uiSavedHadModeListLFNST;
          CandHadList = m_dSavedHadListLFNST;
        }

        if( m_pcEncCfg->getFastUDIUseMPMEnabled() )
        {
          const int numMPMs = NUM_MOST_PROBABLE_MODES;
          unsigned  uiPreds[numMPMs];

          pu.multiRefIdx = 0;

          const int numCand = PU::getIntraMPMs( pu, uiPreds );

          for( int j = 0; j < numCand; j++ )
          {
            bool mostProbableModeIncluded = false;
#if JVET_P0803_COMBINED_MIP_CLEANUP
            ModeInfo mostProbableMode( false, false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j] );
#else
            ModeInfo mostProbableMode( false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j] );
#endif


            for( int i = 0; i < numModesForFullRD; i++ )
            {
              mostProbableModeIncluded |= ( mostProbableMode == uiRdModeList[i] );
            }
            if( !mostProbableModeIncluded )
            {
              numModesForFullRD++;
              uiRdModeList.push_back( mostProbableMode );
              CandCostList.push_back(0);
            }
          }
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
          if ( saveDataForISP )
#else
          if ( testISP )
#endif
          {
            // we add the MPMs to the list that contains only regular intra modes
            for (int j = 0; j < numCand; j++)
            {
              bool     mostProbableModeIncluded = false;
#if JVET_P0803_COMBINED_MIP_CLEANUP
              ModeInfo mostProbableMode( false, false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j] );
#else
              ModeInfo mostProbableMode(false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j]);
#endif

              for (int i = 0; i < m_ispCandListHor.size(); i++)
              {
                mostProbableModeIncluded |= (mostProbableMode == m_ispCandListHor[i]);
              }
              if (!mostProbableModeIncluded)
              {
                m_ispCandListHor.push_back(mostProbableMode);
              }
            }
          }
        }
      }
      else
      {
        THROW( "Full search not supported for MIP" );
      }
      if( sps.getUseLFNST() && mtsUsageFlag == 1 )
      {
        // Store the modes to be checked with RD
        m_savedNumRdModes[ lfnstIdx ]     = numModesForFullRD;
        std::copy_n( uiRdModeList.begin(),  numModesForFullRD, m_savedRdModeList[ lfnstIdx ] );
      }
    }
    else //mtsUsage = 2 (here we potentially reduce the number of modes that will be full-RD checked)
    {
      if( ( m_pcEncCfg->getUseFastLFNST() || !cu.slice->isIntra() ) && m_bestModeCostValid[ lfnstIdx ] )
      {
        numModesForFullRD = 0;

        double thresholdSkipMode = 1.0 + ( ( cu.lfnstIdx > 0 ) ? 0.1 : 1.0 ) * ( 1.4 / sqrt( ( double ) ( width*height ) ) );

        // Skip checking the modes with much larger R-D cost than the best mode
        for( int i = 0; i < m_savedNumRdModes[ lfnstIdx ]; i++ )
        {
          if( m_modeCostStore[ lfnstIdx ][ i ] <= thresholdSkipMode * m_bestModeCostStore[ lfnstIdx ] )
          {
            uiRdModeList.push_back( m_savedRdModeList[ lfnstIdx ][ i ] );
            numModesForFullRD++;
          }
        }
      }
      else //this is necessary because we skip the candidates list calculation, since it was already obtained for the DCT-II. Now we load it
      {
        // Restore the modes to be checked with RD
        numModesForFullRD = m_savedNumRdModes[ lfnstIdx ];
        uiRdModeList.resize( numModesForFullRD );
        std::copy_n( m_savedRdModeList[ lfnstIdx ], m_savedNumRdModes[ lfnstIdx ], uiRdModeList.begin() );
        CandCostList.resize( numModesForFullRD );
      }
    }

    CHECK( numModesForFullRD != uiRdModeList.size(), "Inconsistent state!" );

    // after this point, don't use numModesForFullRD

    // PBINTRA fast
    if( m_pcEncCfg->getUsePbIntraFast() && !cs.slice->isIntra() && uiRdModeList.size() < numModesAvailable && !cs.slice->getDisableSATDForRD() && ( mtsUsageFlag != 2 || lfnstIdx > 0 ) )
    {
      double pbintraRatio = (lfnstIdx > 0) ? 1.25 : PBINTRA_RATIO;
      int maxSize = -1;
      ModeInfo bestMipMode;
      int bestMipIdx = -1;
      for( int idx = 0; idx < uiRdModeList.size(); idx++ )
      {
        if( uiRdModeList[idx].mipFlg )
        {
          bestMipMode = uiRdModeList[idx];
          bestMipIdx = idx;
          break;
        }
      }
      const int numHadCand = 3;
      for (int k = numHadCand - 1; k >= 0; k--)
      {
        if (CandHadList.size() < (k + 1) || CandHadList[k] > cs.interHad * pbintraRatio) { maxSize = k; }
      }
      if (maxSize > 0)
      {
        uiRdModeList.resize(std::min<size_t>(uiRdModeList.size(), maxSize));
        if( bestMipIdx >= 0 )
        {
          if( uiRdModeList.size() <= bestMipIdx )
          {
            uiRdModeList.push_back(bestMipMode);
          }
        }
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
        if ( saveDataForISP )
#else
        if ( testISP )
#endif
        {
          m_ispCandListHor.resize(std::min<size_t>(m_ispCandListHor.size(), maxSize));
        }
      }
      if (maxSize == 0)
      {
        cs.dist = std::numeric_limits<Distortion>::max();
        cs.interHad = 0;

        //===== reset context models =====
        m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);
        m_CABACEstimator->getCtx() = SubCtx(Ctx::ISPMode, ctxStartIspMode);
        m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
        m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
        m_CABACEstimator->getCtx() = SubCtx(Ctx::MultiRefLineIdx, ctxStartMrlIdx);

        return false;
      }
    }
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
    }
#endif

    int numNonISPModes = (int)uiRdModeList.size();

    if ( testISP )
    {
      // we reserve positions for ISP in the common full RD list
#if JVET_P1026_ISP_LFNST_COMBINATION
      const int maxNumRDModesISP = sps.getUseLFNST() ? 16 * NUM_LFNST_NUM_PER_SET : 16;
      m_curIspLfnstIdx = 0;
#else
      const int maxNumRDModesISP = 16;
#endif
      for (int i = 0; i < maxNumRDModesISP; i++)
#if JVET_P0803_COMBINED_MIP_CLEANUP
        uiRdModeList.push_back( ModeInfo( false, false, 0, INTRA_SUBPARTITIONS_RESERVED, 0 ) );
#else
        uiRdModeList.push_back(ModeInfo(false, 0, INTRA_SUBPARTITIONS_RESERVED, 0));
#endif
    }

    //===== check modes (using r-d costs) =====
    ModeInfo       uiBestPUMode;
    int            bestBDPCMMode = 0;
    double         bestCostNonBDPCM = MAX_DOUBLE;

    CodingStructure *csTemp = m_pTempCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];
    CodingStructure *csBest = m_pBestCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];

    csTemp->slice = cs.slice;
    csBest->slice = cs.slice;
    csTemp->initStructData();
    csBest->initStructData();
    csTemp->picture = cs.picture;
    csBest->picture = cs.picture;

#if !JVET_P0803_COMBINED_MIP_CLEANUP
    static_vector<int, FAST_UDI_MAX_RDMODE_NUM> rdModeIdxList;
    if (testMip)
    {
    static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> rdModeListTemp;
    for( int i = 0; i < uiRdModeList.size(); i++)
    {
      if( !uiRdModeList[i].mipFlg && uiRdModeList[i].ispMod==NOT_INTRA_SUBPARTITIONS )
      {
        rdModeListTemp.push_back( uiRdModeList[i] );
        rdModeIdxList.push_back( i );
      }
    }
    for( int i = 0; i < uiRdModeList.size(); i++)
    {
      if( uiRdModeList[i].mipFlg || uiRdModeList[i].ispMod!=NOT_INTRA_SUBPARTITIONS )
      {
        rdModeListTemp.push_back( uiRdModeList[i] );
        rdModeIdxList.push_back( i );
      }
    }
      uiRdModeList.resize(rdModeListTemp.size());
    for( int i = 0; i < uiRdModeList.size(); i++)
    {
      uiRdModeList[i] = rdModeListTemp[i];
    }
    }
    else
    {
      static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> rdModeListTemp;
      for( int i = 0; i < uiRdModeList.size(); i++ )
      {
        if( !uiRdModeList[i].mipFlg  )
        {
          rdModeListTemp.push_back( uiRdModeList[i] );
        }
      }
      uiRdModeList.resize(rdModeListTemp.size());
      for( int i = 0; i < rdModeListTemp.size(); i++ )
      {
        uiRdModeList[i] = rdModeListTemp[i];
      }
    }
#endif

    // just to be sure
    numModesForFullRD = ( int ) uiRdModeList.size();
    TUIntraSubPartitioner subTuPartitioner( partitioner );
#if JVET_P1026_ISP_LFNST_COMBINATION
    if ( testISP )
    {
      m_modeCtrl->setIspCost( MAX_DOUBLE );
      m_modeCtrl->setMtsFirstPassNoIspCost( MAX_DOUBLE );
    }
    int bestLfnstIdx = cu.lfnstIdx;
#else
    if( !cu.ispMode && !cu.mtsFlag )
    {
      m_modeCtrl->setMtsFirstPassNoIspCost( MAX_DOUBLE );
    }
#endif

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
    for (int mode = isSecondColorSpace ? 0 : -2 * int(testBDPCM); mode < (int)uiRdModeList.size(); mode++)
#else
    for (int mode = -2 * int(testBDPCM); mode < (int)uiRdModeList.size(); mode++)
#endif
    {
      // set CU/PU to luma prediction mode
      ModeInfo uiOrgMode;
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
      if (sps.getUseColorTrans() && !m_pcEncCfg->getRGBFormatFlag() && isSecondColorSpace && mode)
      {
        continue;
      }

      if (mode < 0 || (isSecondColorSpace && m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx][mode]))
      {
        cu.bdpcmMode = mode < 0 ? -mode : m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx][mode];
#else
      if ( mode < 0 )
      {
        cu.bdpcmMode = -mode;
#endif
#if JVET_P0803_COMBINED_MIP_CLEANUP
        uiOrgMode = ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, cu.bdpcmMode == 2 ? VER_IDX : HOR_IDX );
#else
        uiOrgMode = ModeInfo(false, 0, NOT_INTRA_SUBPARTITIONS, cu.bdpcmMode == 2 ? VER_IDX : HOR_IDX);
#endif
#if !JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
        cu.mipFlag                     = uiOrgMode.mipFlg;
#if JVET_P0803_COMBINED_MIP_CLEANUP
        pu.mipTransposedFlag           = uiOrgMode.mipTrFlg;
#endif
        cu.ispMode                     = uiOrgMode.ispMod;
        pu.multiRefIdx                 = uiOrgMode.mRefId;
        pu.intraDir[CHANNEL_TYPE_LUMA] = uiOrgMode.modeId;
#endif
      }
      else
      {
        cu.bdpcmMode = 0;
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
        uiOrgMode = uiRdModeList[mode];
      }
      if (!cu.bdpcmMode && uiRdModeList[mode].ispMod == INTRA_SUBPARTITIONS_RESERVED)
#else      
        if (uiRdModeList[mode].ispMod == INTRA_SUBPARTITIONS_RESERVED)
#endif
        {
          if (mode == numNonISPModes) // the list needs to be sorted only once
          {
#if JVET_P1026_ISP_LFNST_COMBINATION
            if (m_pcEncCfg->getUseFastISP())
            {
              m_modeCtrl->setBestPredModeDCT2(uiBestPUMode.modeId);
            }
            if (!xSortISPCandList(bestCurrentCost, csBest->cost, uiBestPUMode))
              break;
#else
            xSortISPCandList(bestCurrentCost, csBest->cost);
#endif
          }
          xGetNextISPMode(uiRdModeList[mode], (mode > 0 ? &uiRdModeList[mode - 1] : nullptr), Size(width, height));
          if (uiRdModeList[mode].ispMod == INTRA_SUBPARTITIONS_RESERVED)
            continue;
#if JVET_P1026_ISP_LFNST_COMBINATION
          cu.lfnstIdx = m_curIspLfnstIdx;
#endif
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
          uiOrgMode = uiRdModeList[mode];
        }
#else
        }
        uiOrgMode = uiRdModeList[mode];
#endif
      cu.mipFlag                     = uiOrgMode.mipFlg;
#if JVET_P0803_COMBINED_MIP_CLEANUP
      pu.mipTransposedFlag           = uiOrgMode.mipTrFlg;
#endif
      cu.ispMode                     = uiOrgMode.ispMod;
      pu.multiRefIdx                 = uiOrgMode.mRefId;
      pu.intraDir[CHANNEL_TYPE_LUMA] = uiOrgMode.modeId;

      CHECK(cu.mipFlag && pu.multiRefIdx, "Error: combination of MIP and MRL not supported");
      CHECK(pu.multiRefIdx && (pu.intraDir[0] == PLANAR_IDX), "Error: combination of MRL and Planar mode not supported");
      CHECK(cu.ispMode && cu.mipFlag, "Error: combination of ISP and MIP not supported");
      CHECK(cu.ispMode && pu.multiRefIdx, "Error: combination of ISP and MRL not supported");
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
      CHECK(cu.ispMode&& cu.colorTransform, "Error: combination of ISP and ACT not supported");

      pu.intraDir[CHANNEL_TYPE_CHROMA] = cu.colorTransform ? DM_CHROMA_IDX : pu.intraDir[CHANNEL_TYPE_CHROMA];
#else
      }
#endif

      // set context models
      m_CABACEstimator->getCtx() = ctxStart;

      // determine residual for partition
      cs.initSubStructure( *csTemp, partitioner.chType, cs.area, true );

      bool tmpValidReturn = false;
      if( cu.ispMode )
      {
#if JVET_P1026_ISP_LFNST_COMBINATION
        if ( m_pcEncCfg->getUseFastISP() )
        {
          m_modeCtrl->setISPWasTested(true);
        }
#endif
        tmpValidReturn = xIntraCodingLumaISP(*csTemp, subTuPartitioner, bestCurrentCost);
        if (csTemp->tus.size() == 0)
        {
          // no TUs were coded
          csTemp->cost = MAX_DOUBLE;
          continue;
        }
#if JVET_P1026_ISP_LFNST_COMBINATION
        // we save the data for future tests
        m_ispTestedModes[m_curIspLfnstIdx].setModeResults((ISPType)cu.ispMode, (int)uiOrgMode.modeId, (int)csTemp->tus.size(), csTemp->cus[0]->firstTU->cbf[COMPONENT_Y] ? csTemp->cost : MAX_DOUBLE, csBest->cost);
        csTemp->cost = !tmpValidReturn ? MAX_DOUBLE : csTemp->cost;
#else
        if (!cu.mtsFlag && !cu.lfnstIdx)
        {
          // we save the data for future tests
          m_ispTestedModes.setModeResults((ISPType)cu.ispMode, (int)uiOrgMode.modeId, (int)csTemp->tus.size(), csTemp->cus[0]->firstTU->cbf[COMPONENT_Y] ? csTemp->cost : MAX_DOUBLE, csBest->cost);
        }
#endif
      }
      else
      {
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
        if (cu.colorTransform)
        {
          tmpValidReturn = xRecurIntraCodingACTQT(*csTemp, partitioner, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
        }
        else
#endif
        tmpValidReturn = xRecurIntraCodingLumaQT( *csTemp, partitioner, uiBestPUMode.ispMod ? bestCurrentCost : MAX_DOUBLE, -1, TU_NO_ISP, uiBestPUMode.ispMod,
                                                  mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst );
      }

      if (!cu.ispMode && !cu.mtsFlag && !cu.lfnstIdx && !cu.bdpcmMode && !pu.multiRefIdx && !cu.mipFlag && testISP)
      {
#if JVET_P0803_COMBINED_MIP_CLEANUP
        m_regIntraRDListWithCosts.push_back( ModeInfoWithCost( cu.mipFlag, pu.mipTransposedFlag, pu.multiRefIdx, cu.ispMode, uiOrgMode.modeId, csTemp->cost ) );
#else
        m_regIntraRDListWithCosts.push_back(ModeInfoWithCost(cu.mipFlag, pu.multiRefIdx, cu.ispMode, uiOrgMode.modeId, csTemp->cost));
#endif
      }

      if( cu.ispMode && !csTemp->cus[0]->firstTU->cbf[COMPONENT_Y] )
      {
        csTemp->cost = MAX_DOUBLE;
        csTemp->costDbOffset = 0;
        tmpValidReturn = false;
      }
      validReturn |= tmpValidReturn;

      if( sps.getUseLFNST() && mtsUsageFlag == 1 && !cu.ispMode && mode >= 0 )
      {
#if JVET_P0803_COMBINED_MIP_CLEANUP
        m_modeCostStore[lfnstIdx][mode] = tmpValidReturn ? csTemp->cost : (MAX_DOUBLE / 2.0); //(MAX_DOUBLE / 2.0) ??
#else
        m_modeCostStore[ lfnstIdx ][ testMip ? rdModeIdxList[ mode ] : mode ] = tmpValidReturn ? csTemp->cost : ( MAX_DOUBLE / 2.0 ); //(MAX_DOUBLE / 2.0) ??
#endif
      }

      DTRACE(g_trace_ctx, D_INTRA_COST, "IntraCost T [x=%d,y=%d,w=%d,h=%d] %f (%d,%d,%d,%d,%d,%d) \n", cu.blocks[0].x,
        cu.blocks[0].y, (int)width, (int)height, csTemp->cost, uiOrgMode.modeId, uiOrgMode.ispMod,
        pu.multiRefIdx, cu.mipFlag, cu.lfnstIdx, cu.mtsFlag);


      if( tmpValidReturn )
      {
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
        if (isFirstColorSpace)
        {
          if (m_pcEncCfg->getRGBFormatFlag() || !cu.ispMode)
          {
            sortRdModeListFirstColorSpace(uiOrgMode, csTemp->cost, cu.bdpcmMode, m_savedRdModeFirstColorSpace[m_savedRdModeIdx], m_savedRdCostFirstColorSpace[m_savedRdModeIdx], m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx], m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx]);
          }
        }
#endif
        // check r-d cost
        if( csTemp->cost < csBest->cost )
        {
          std::swap( csTemp, csBest );

          uiBestPUMode  = uiOrgMode;
          bestBDPCMMode = cu.bdpcmMode;
          if( sps.getUseLFNST() && mtsUsageFlag == 1 && !cu.ispMode )
          {
            m_bestModeCostStore[ lfnstIdx ] = csBest->cost; //cs.cost;
            m_bestModeCostValid[ lfnstIdx ] = true;
          }
          if( csBest->cost < bestCurrentCost )
          {
            bestCurrentCost = csBest->cost;
          }
#if JVET_P1026_ISP_LFNST_COMBINATION
          if ( cu.ispMode )
          {
            m_modeCtrl->setIspCost(csBest->cost);
            bestLfnstIdx = cu.lfnstIdx;
          }
          else if ( testISP )
          {
            m_modeCtrl->setMtsFirstPassNoIspCost(csBest->cost);
          }
#else
          if( !cu.ispMode && !cu.mtsFlag )
          {
            m_modeCtrl->setMtsFirstPassNoIspCost( csBest->cost );
          }
#endif
        }
        if( !cu.ispMode && !cu.bdpcmMode && csBest->cost < bestCostNonBDPCM )
        {
          bestCostNonBDPCM = csBest->cost;
        }
      }

      csTemp->releaseIntermediateData();
      if( m_pcEncCfg->getFastLocalDualTreeMode() )
      {
        if( cu.isConsIntra() && !cu.slice->isIntra() && csBest->cost != MAX_DOUBLE && costInterCU != COST_UNKNOWN && mode >= 0 )
        {
          if( m_pcEncCfg->getFastLocalDualTreeMode() == 2 )
          {
            //Note: only try one intra mode, which is especially useful to reduce EncT for LDB case (around 4%)
            break;
          }
          else
          {
            if( csBest->cost > costInterCU * 1.5 )
            {
              break;
            }
          }
        }
      }
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
      if (sps.getUseColorTrans() && !CS::isDualITree(cs))
      {
        if ((m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform) && csBest->cost != MAX_DOUBLE && bestCS->cost != MAX_DOUBLE && mode >= 0)
        {
          if (csBest->cost > bestCS->cost)
          {
            break;
          }
        }
      }
#endif
    } // Mode loop
    cu.ispMode = uiBestPUMode.ispMod;
#if JVET_P1026_ISP_LFNST_COMBINATION
    cu.lfnstIdx = bestLfnstIdx;
#endif

    if( validReturn )
    {
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
      if (cu.colorTransform)
      {
        cs.useSubStructure(*csBest, partitioner.chType, pu, true, true, keepResi, keepResi);
      }
      else
#endif
      cs.useSubStructure( *csBest, partitioner.chType, pu.singleChan( CHANNEL_TYPE_LUMA ), true, true, keepResi, keepResi );
    }
    csBest->releaseIntermediateData();
    if( validReturn )
    {
      //=== update PU data ====
      cu.mipFlag = uiBestPUMode.mipFlg;
#if JVET_P0803_COMBINED_MIP_CLEANUP
      pu.mipTransposedFlag             = uiBestPUMode.mipTrFlg;
#endif
      pu.multiRefIdx = uiBestPUMode.mRefId;
      pu.intraDir[ CHANNEL_TYPE_LUMA ] = uiBestPUMode.modeId;
      cu.bdpcmMode = bestBDPCMMode;
#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
      if (cu.colorTransform)
      {
        CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM mode for adaptive color transform");
      }
#endif
    }
  }

  //===== reset context models =====
  m_CABACEstimator->getCtx() = ctxStart;

  return validReturn;
}

void IntraSearch::estIntraPredChromaQT( CodingUnit &cu, Partitioner &partitioner, const double maxCostAllowed )
{
  const ChromaFormat format   = cu.chromaFormat;
  const uint32_t    numberValidComponents = getNumberValidComponents(format);
  CodingStructure &cs = *cu.cs;
  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );

  cs.setDecomp( cs.area.Cb(), false );

  double    bestCostSoFar = maxCostAllowed;
  bool      lumaUsesISP   = !cu.isSepTree() && cu.ispMode;
  PartSplit ispType       = lumaUsesISP ? CU::getISPType( cu, COMPONENT_Y ) : TU_NO_ISP;
  CHECK( cu.ispMode && bestCostSoFar < 0, "bestCostSoFar must be positive!" );

  auto &pu = *cu.firstPU;

  {
    uint32_t       uiBestMode = 0;
    Distortion uiBestDist = 0;
    double     dBestCost = MAX_DOUBLE;
#if JVET_P0059_CHROMA_BDPCM
    int32_t bestBDPCMMode = 0;
#endif

    //----- init mode list ----
    {
#if JVET_P0059_CHROMA_BDPCM
      int32_t  uiMinMode = 0;
      int32_t  uiMaxMode = NUM_CHROMA_MODE;
#else
      uint32_t  uiMinMode = 0;
      uint32_t  uiMaxMode = NUM_CHROMA_MODE;
#endif
      //----- check chroma modes -----
      uint32_t chromaCandModes[ NUM_CHROMA_MODE ];
      PU::getIntraChromaCandModes( pu, chromaCandModes );

      // create a temporary CS
      CodingStructure &saveCS = *m_pSaveCS[0];
      saveCS.pcv      = cs.pcv;
      saveCS.picture  = cs.picture;
      saveCS.area.repositionTo( cs.area );
      saveCS.clearTUs();

      if( !cu.isSepTree() && cu.ispMode )
      {
        saveCS.clearCUs();
        saveCS.clearPUs();
      }

      if( cu.isSepTree() )
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

      if( lumaUsesISP )
      {
        CodingUnit& auxCU = saveCS.addCU( cu, partitioner.chType );
        auxCU.ispMode = cu.ispMode;
        saveCS.sps = cu.cs->sps;
        saveCS.addPU( *cu.firstPU, partitioner.chType );
      }


      // create a store for the TUs
      for( const auto &ptu : cs.tus )
      {
        // for split TUs in HEVC, add the TUs without Chroma parts for correct setting of Cbfs
        if( lumaUsesISP || pu.contains( *ptu, CHANNEL_TYPE_CHROMA ) )
        {
          saveCS.addTU( *ptu, partitioner.chType );
          orgTUs.push_back( ptu );
        }
      }
      if( lumaUsesISP )
      {
        saveCS.clearCUs();
      }
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
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
      DistParam distParamSad;
      DistParam distParamSatd;
#else
      DistParam distParam;
#endif
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
#else
      const bool useHadamard = true;
#endif
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

#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
        int64_t sad = 0;
        int64_t sadCb = 0;
        int64_t satdCb = 0;
        int64_t sadCr = 0;
        int64_t satdCr = 0;
#else
        int64_t sad = 0;
#endif
        CodingStructure& cs = *(pu.cs);

        CompArea areaCb = pu.Cb();
        PelBuf orgCb = cs.getOrgBuf(areaCb);
        PelBuf predCb = cs.getPredBuf(areaCb);
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
        m_pcRdCost->setDistParam(distParamSad, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, false);
        m_pcRdCost->setDistParam(distParamSatd, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, true);
#else
        m_pcRdCost->setDistParam(distParam, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, useHadamard);
#endif
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
        distParamSad.applyWeight = false;
        distParamSatd.applyWeight = false;
#else
        distParam.applyWeight = false;
#endif
        if (PU::isLMCMode(mode))
        {
          predIntraChromaLM(COMPONENT_Cb, predCb, pu, areaCb, mode);
        }
        else
        {
          initPredIntraParams(pu, pu.Cb(), *pu.cs->sps);
          predIntraAng(COMPONENT_Cb, predCb, pu);
        }
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
        sadCb = distParamSad.distFunc(distParamSad) * 2;
        satdCb = distParamSatd.distFunc(distParamSatd);
        sad += std::min(sadCb, satdCb);
#else
        sad += distParam.distFunc(distParam);
#endif
        CompArea areaCr = pu.Cr();
        PelBuf orgCr = cs.getOrgBuf(areaCr);
        PelBuf predCr = cs.getPredBuf(areaCr);
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
        m_pcRdCost->setDistParam(distParamSad, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, false);
        m_pcRdCost->setDistParam(distParamSatd, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, true);
#else
        m_pcRdCost->setDistParam(distParam, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, useHadamard);
#endif
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
        distParamSad.applyWeight = false;
        distParamSatd.applyWeight = false;
#else
        distParam.applyWeight = false;
#endif
        if (PU::isLMCMode(mode))
        {
          predIntraChromaLM(COMPONENT_Cr, predCr, pu, areaCr, mode);
        }
        else
        {
          initPredIntraParams(pu, pu.Cr(), *pu.cs->sps);
          predIntraAng(COMPONENT_Cr, predCr, pu);
        }
#if JVET_P0058_CHROMA_TS_ENCODER_INTRA_SAD_MOD
        sadCr = distParamSad.distFunc(distParamSad) * 2;
        satdCr = distParamSatd.distFunc(distParamSatd);
        sad += std::min(sadCr, satdCr);
#else
        sad += distParam.distFunc(distParam);
#endif
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

      // save the dist
      Distortion baseDist = cs.dist;
#if JVET_P0059_CHROMA_BDPCM
      bool testBDPCM = true;
      testBDPCM = testBDPCM && CU::bdpcmAllowed(cu, COMPONENT_Cb) && cu.ispMode == 0 && cu.mtsFlag == 0 && cu.lfnstIdx == 0;
      for (int32_t uiMode = uiMinMode - (2 * int(testBDPCM)); uiMode < uiMaxMode; uiMode++)
#else
      for (uint32_t uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++)
#endif
      {
#if JVET_P0059_CHROMA_BDPCM
        int chromaIntraMode = chromaCandModes[uiMode];
#else 
        const int chromaIntraMode = chromaCandModes[uiMode];
#endif

#if JVET_P0059_CHROMA_BDPCM
        if (uiMode < 0)
        {
            cu.bdpcmModeChroma = -uiMode;
            chromaIntraMode = chromaCandModes[0];
        }
        else
        {
            cu.bdpcmModeChroma = 0;
#endif
        if( PU::isLMCMode( chromaIntraMode ) && ! PU::isLMCModeEnabled( pu, chromaIntraMode ) )
        {
          continue;
        }
        if (!modeIsEnable[chromaIntraMode] && PU::isLMCModeEnabled(pu, chromaIntraMode)) // when CCLM is disable, then MDLM is disable. not use satd checking
        {
          continue;
        }
#if JVET_P0059_CHROMA_BDPCM
        }
#endif
        cs.setDecomp( pu.Cb(), false );
        cs.dist = baseDist;
        //----- restore context models -----
        m_CABACEstimator->getCtx() = ctxStart;

        //----- chroma coding -----
        pu.intraDir[1] = chromaIntraMode;

        xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );
        if( lumaUsesISP && cs.dist == MAX_UINT )
        {
          continue;
        }

        if (cs.sps->getTransformSkipEnabledFlag())
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        uint64_t fracBits   = xGetIntraFracBitsQT( cs, partitioner, false, true, -1, ispType );
        Distortion uiDist = cs.dist;
        double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

        //----- compare -----
        if( dCost < dBestCost )
        {
          if( lumaUsesISP && dCost < bestCostSoFar )
          {
            bestCostSoFar = dCost;
          }
          for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
          {
            const CompArea &area = pu.blocks[i];

            saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
            saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   (area ) );
            cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf   (area ) );
            cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );

            for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
            {
              saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
            }
          }

          dBestCost  = dCost;
          uiBestDist = uiDist;
          uiBestMode = chromaIntraMode;
#if JVET_P0059_CHROMA_BDPCM
          bestBDPCMMode = cu.bdpcmModeChroma;
#endif
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
        cs.getPredBuf         ( area ).copyFrom( saveCS.getPredBuf( area ) );
        cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf    ( area ) );

        cs.picture->getRecoBuf( area ).copyFrom( cs.    getRecoBuf( area ) );

        for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
        {
          orgTUs[ j ]->copyComponentFrom( *saveCS.tus[ j ], area.compID );
        }
      }
    }

    pu.intraDir[1] = uiBestMode;
    cs.dist        = uiBestDist;
#if JVET_P0059_CHROMA_BDPCM
    cu.bdpcmModeChroma = bestBDPCMMode;
#endif
  }

  //----- restore context models -----
  m_CABACEstimator->getCtx() = ctxStart;
  if( lumaUsesISP && bestCostSoFar >= maxCostAllowed )
  {
    cu.ispMode = 0;
  }
}


void IntraSearch::saveCuAreaCostInSCIPU( Area area, double cost )
{
  if( m_numCuInSCIPU < NUM_INTER_CU_INFO_SAVE )
  {
    m_cuAreaInSCIPU[m_numCuInSCIPU] = area;
    m_cuCostInSCIPU[m_numCuInSCIPU] = cost;
    m_numCuInSCIPU++;
  }
}

void IntraSearch::initCuAreaCostInSCIPU()
{
  for( int i = 0; i < NUM_INTER_CU_INFO_SAVE; i++ )
  {
    m_cuAreaInSCIPU[i] = Area();
    m_cuCostInSCIPU[i] = 0;
  }
  m_numCuInSCIPU = 0;
}
void IntraSearch::PLTSearch(CodingStructure &cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;
#if !JVET_P0077_LINE_CG_PALETTE
  m_orgCtxRD = PLTCtx(m_CABACEstimator->getCtx());
#endif

#if JVET_P1006_PICTURE_HEADER
  if (m_pcEncCfg->getLmcs() && (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#else
  if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#endif
  {
    cs.getPredBuf().copyFrom(cs.getOrgBuf());
    cs.getPredBuf().Y().rspSignal(m_pcReshape->getFwdLUT());
  }
#if !JVET_P0077_LINE_CG_PALETTE
  Pel  *runLength = tu.getRunLens (compBegin);
  bool *runType   = tu.getRunTypes(compBegin);
#endif
  cu.lastPLTSize[compBegin] = cs.prevPLT.curPLTSize[compBegin];
  //derive palette
  derivePLTLossy(cs, partitioner, compBegin, numComp);
  reorderPLT(cs, partitioner, compBegin, numComp);

#if JVET_P0077_LINE_CG_PALETTE
  preCalcPLTIndexRD(cs, partitioner, compBegin, numComp); // Pre-calculate distortions for each pixel 
  double rdCost = MAX_DOUBLE;
  deriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_HORTRAV, rdCost); // Optimize palette index map (horizontal scan)
  if ((cu.curPLTSize[compBegin] + cu.useEscape[compBegin]) > 1)
  {
    deriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_VERTRAV, rdCost); // Optimize palette index map (vertical scan)
  }
#else
  //calculate palette index
  preCalcPLTIndex(cs, partitioner, compBegin, numComp);
  //derive run
  uint64_t bits = MAX_UINT64;
  deriveRunAndCalcBits(cs, partitioner, compBegin, numComp, PLT_SCAN_HORTRAV, bits);
  if ((cu.curPLTSize[compBegin] + cu.useEscape[compBegin]) > 1)
  {
    deriveRunAndCalcBits(cs, partitioner, compBegin, numComp, PLT_SCAN_VERTRAV, bits);
  }
#endif
  cu.useRotation[compBegin] = m_bestScanRotationMode;
#if JVET_P0077_LINE_CG_PALETTE
  int indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  if (indexMaxSize <= 1)
  {
    cu.useRotation[compBegin] = false;
  }
#else
  memcpy(runType, m_runTypeRD, sizeof(bool)*width*height);
  memcpy(runLength, m_runLengthRD, sizeof(Pel)*width*height);
#endif
  //reconstruct pixel
  PelBuf    curPLTIdx = tu.getcurPLTIdx(compBegin);
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      if (curPLTIdx.at(x, y) == cu.curPLTSize[compBegin])
      {
#if JVET_P0077_LINE_CG_PALETTE
        calcPixelPred(cs, partitioner, y, x, compBegin, numComp);
#endif
      }
      else
      {
        for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
        {
          CompArea area = cu.blocks[compID];
          PelBuf   recBuf = cs.getRecoBuf(area);
          uint32_t scaleX = getComponentScaleX((ComponentID)COMPONENT_Cb, cs.sps->getChromaFormatIdc());
          uint32_t scaleY = getComponentScaleY((ComponentID)COMPONENT_Cb, cs.sps->getChromaFormatIdc());
          if (compBegin != COMPONENT_Y || compID == COMPONENT_Y)
          {
            recBuf.at(x, y) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
          else if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            recBuf.at(x >> scaleX, y >> scaleY) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
        }
      }
    }
  }

  cs.getPredBuf().fill(0);
  cs.getResiBuf().fill(0);
  cs.getOrgResiBuf().fill(0);

  cs.fracBits = MAX_UINT;
  cs.cost = MAX_DOUBLE;
  Distortion distortion = 0;
  for (uint32_t comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    CPelBuf reco = cs.getRecoBuf(compID);
    CPelBuf org = cs.getOrgBuf(compID);
#if WCG_EXT
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
#if JVET_P1006_PICTURE_HEADER
      m_pcEncCfg->getLmcs() && (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
#else
      m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
#endif
    {
      const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMPONENT_Y]);

      if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
      {
        const CompArea &areaY = cu.Y();
        CompArea tmpArea1(COMPONENT_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
        PelBuf   tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
        tmpRecLuma.copyFrom(reco);
        tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
        distortion += m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
        distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
    }
    else
#endif
      distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE);
  }

  cs.dist += distortion;
  const CompArea &area = cu.blocks[compBegin];
  cs.setDecomp(area);
  cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
}
#if JVET_P0077_LINE_CG_PALETTE
void IntraSearch::calcPixelPredRD(CodingStructure& cs, Partitioner& partitioner, Pel* orgBuf, Pel* paPixelValue, Pel* paRecoValue, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    QpParam cQP(tu, ComponentID(ch));
#if JVET_P0460_PLT_TS_MIN_QP
    qp[ch] = cQP.Qp(true);
#else
    qp[ch] = cQP.Qp(false);
#endif
    qpRem[ch] = qp[ch] % 6;
    qpPer[ch] = qp[ch] / 6;
    quantiserScale[ch] = g_quantScales[0][qpRem[ch]];
    quantiserRightShift[ch] = QUANT_SHIFT + qpPer[ch];
    rightShiftOffset[ch] = 1 << (quantiserRightShift[ch] - 1);
    invquantiserRightShift[ch] = IQUANT_SHIFT;
    add[ch] = 1 << (invquantiserRightShift[ch] - 1);
  }

  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int  channelBitDepth = cu.cs->sps->getBitDepth(toChannelType((ComponentID)ch));
    paPixelValue[ch] = Pel(std::max<int>(0, ((orgBuf[ch] * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
    assert(paPixelValue[ch] < (1 << (channelBitDepth + 1)));
    paRecoValue[ch] = (((paPixelValue[ch] * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
    paRecoValue[ch] = Pel(ClipBD<int>(paRecoValue[ch], channelBitDepth));//to be checked
  }
}

void IntraSearch::preCalcPLTIndexRD(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
#if JVET_P1006_PICTURE_HEADER
    if (m_pcEncCfg->getLmcs() && (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#else
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#endif
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int rasPos;
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      rasPos = y * width + x;;
      // chroma discard
      bool discardChroma = (compBegin == COMPONENT_Y) && (y&scaleY || x&scaleX);
      Pel curPel[3];
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        uint32_t pX1 = (comp > 0 && compBegin == COMPONENT_Y) ? (x >> scaleX) : x;
        uint32_t pY1 = (comp > 0 && compBegin == COMPONENT_Y) ? (y >> scaleY) : y;
        curPel[comp] = orgBuf[comp].at(pX1, pY1);
      }

      uint8_t  pltIdx = 0;
      double minError = MAX_DOUBLE;
      uint8_t  bestIdx = 0;
      while (pltIdx < cu.curPLTSize[compBegin])
      {
        uint64_t sqrtError = 0;
        for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
        {
          int64_t tmpErr = int64_t(curPel[comp] - cu.curPLT[comp][pltIdx]);
          if (isChroma((ComponentID)comp))
          {
            sqrtError += uint64_t(tmpErr*tmpErr*ENC_CHROMA_WEIGHTING);
          }
          else
          {
            sqrtError += tmpErr*tmpErr;
          }
        }
        m_indexError[pltIdx][rasPos] = (double)sqrtError;
        if (sqrtError < minError)
        {
          minError = (double)sqrtError;
          bestIdx = pltIdx;
        }
        pltIdx++;
      }

      Pel paPixelValue[3], paRecoValue[3];
      calcPixelPredRD(cs, partitioner, curPel, paPixelValue, paRecoValue, compBegin, numComp);
      uint64_t error = 0, rate = 0;
      for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
      {
        int64_t tmpErr = int64_t(curPel[comp] - paRecoValue[comp]);
        if (isChroma((ComponentID)comp))
        {
          error += uint64_t(tmpErr*tmpErr*ENC_CHROMA_WEIGHTING);
        }
        else
        {
          error += tmpErr*tmpErr;
        }
        rate += m_escapeNumBins[paPixelValue[comp]]; // encode quantized escape color
      }
      double rdCost = (double)error + m_pcRdCost->getLambda()*(double)rate;
      m_indexError[cu.curPLTSize[compBegin]][rasPos] = rdCost;
      if (rdCost < minError) 
      {
        minError = rdCost;
        bestIdx = (uint8_t)cu.curPLTSize[compBegin];
      }
      m_minErrorIndexMap[rasPos] = bestIdx; // save the optimal index of the current pixel
    }
  }
}

void IntraSearch::deriveIndexMap(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp, PLTScanMode pltScanMode, double& dMinCost)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t      height = cu.block(compBegin).height;
  uint32_t      width = cu.block(compBegin).width;

  int   total     = height*width;
  Pel  *runIndex = tu.getPLTIndex(compBegin);
  bool *runType  = tu.getRunTypes(compBegin);
  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][pltScanMode ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
// Trellis initialization
  for (int i = 0; i < 2; i++)
  {
    memset(m_prevRunTypeRDOQ[i], 0, sizeof(Pel)*NUM_TRELLIS_STATE);
    memset(m_prevRunPosRDOQ[i],  0, sizeof(int)*NUM_TRELLIS_STATE);
    memset(m_stateCostRDOQ[i],  0, sizeof (double)*NUM_TRELLIS_STATE);
  }
  for (int state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    m_statePtRDOQ[state][0] = 0;
  }
// Context modeling
  const FracBitsAccess& fracBits = m_CABACEstimator->getCtx().getFracBitsAcess();
  BinFracBits fracBitsPltCopyFlagIndex[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(PLT_RUN_INDEX, dist);
    fracBitsPltCopyFlagIndex[dist] = fracBits.getFracBitsArray(Ctx::IdxRunModel( ctxId ) );
  }
  BinFracBits fracBitsPltCopyFlagAbove[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(PLT_RUN_COPY, dist);
    fracBitsPltCopyFlagAbove[dist] = fracBits.getFracBitsArray(Ctx::CopyRunModel( ctxId ) );
  }
  const BinFracBits fracBitsPltRunType = fracBits.getFracBitsArray( Ctx::RunTypeFlag() );

// Trellis RDO per CG
  bool contTrellisRD = true;
  for (int subSetId = 0; ( subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE ) && contTrellisRD; subSetId++)
  {
    int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
    int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
    maxSubPos = (maxSubPos > total) ? total : maxSubPos; // if last position is out of the current CU size
    contTrellisRD = deriveSubblockIndexMap(cs, partitioner, compBegin, pltScanMode, minSubPos, maxSubPos, fracBitsPltRunType, fracBitsPltCopyFlagIndex, fracBitsPltCopyFlagAbove, dMinCost, (bool)pltScanMode);
  }
  if (!contTrellisRD)
  {
    return;
  }


// best state at the last scan position
  double  sumRdCost = MAX_DOUBLE;
  uint8_t bestState = 0;
  for (uint8_t state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    if (m_stateCostRDOQ[0][state] < sumRdCost)
    {
      sumRdCost = m_stateCostRDOQ[0][state];
      bestState = state;
    }
  }

     bool checkRunTable  [MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t checkIndexTable[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t bestStateTable [MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t nextState = bestState;
// best trellis path
  for (int i = (width*height - 1); i >= 0; i--)
  {
    bestStateTable[i] = nextState;
    int rasterPos = m_scanOrder[i].idx;
    nextState = m_statePtRDOQ[nextState][rasterPos];
  }
// reconstruct index and runs based on the state pointers
  for (int i = 0; i < (width*height); i++)
  {
    int rasterPos = m_scanOrder[i].idx;
    int  abovePos = (pltScanMode == PLT_SCAN_HORTRAV) ? m_scanOrder[i].idx - width : m_scanOrder[i].idx - 1;
        nextState = bestStateTable[i];
    if ( nextState == 0 ) // same as the previous
    {
      checkRunTable[rasterPos] = checkRunTable[ m_scanOrder[i - 1].idx ];
      if ( checkRunTable[rasterPos] == PLT_RUN_INDEX )
      {
        checkIndexTable[rasterPos] = checkIndexTable[m_scanOrder[i - 1].idx];
      }
      else
      {
        checkIndexTable[rasterPos] = checkIndexTable[ abovePos ];
      }
    }
    else if (nextState == 1) // CopyAbove mode
    {
      checkRunTable[rasterPos] = PLT_RUN_COPY;
      checkIndexTable[rasterPos] = checkIndexTable[abovePos];
    }
    else if (nextState == 2) // Index mode
    {
      checkRunTable[rasterPos] = PLT_RUN_INDEX;
      checkIndexTable[rasterPos] = m_minErrorIndexMap[rasterPos];
    }
  }

// Escape flag
  m_bestEscape = false;
  for (int pos = 0; pos < (width*height); pos++)
  {
    uint8_t index = checkIndexTable[pos];
    if (index == cu.curPLTSize[compBegin])
    {
      m_bestEscape = true;
      break;
    }
  }

// Horizontal scan v.s vertical scan
  if (sumRdCost < dMinCost)
  {
    cu.useEscape[compBegin] = m_bestEscape;
    m_bestScanRotationMode = pltScanMode;
    for (int pos = 0; pos < (width*height); pos++)
    {
      runIndex[pos] = checkIndexTable[pos];
      runType[pos] = checkRunTable[pos];
    }
    dMinCost = sumRdCost;
  }
}

bool IntraSearch::deriveSubblockIndexMap(
  CodingStructure& cs,
  Partitioner&  partitioner,
  ComponentID   compBegin,
  PLTScanMode   pltScanMode,
  int           minSubPos,
  int           maxSubPos,
  const BinFracBits& fracBitsPltRunType,
  const BinFracBits* fracBitsPltIndexINDEX,
  const BinFracBits* fracBitsPltIndexCOPY,
  const double minCost,
  bool         useRotate
)
{
  CodingUnit &cu    = *cs.getCU(partitioner.chType);
  uint32_t   height = cu.block(compBegin).height;
  uint32_t   width  = cu.block(compBegin).width;
  int indexMaxValue = cu.curPLTSize[compBegin];

  int refId = 0;
  int currRasterPos, currScanPos, prevScanPos, aboveScanPos, roffset;
  int log2Width = (pltScanMode == PLT_SCAN_HORTRAV) ? floorLog2(width): floorLog2(height);
  int buffersize = (pltScanMode == PLT_SCAN_HORTRAV) ? 2*width: 2*height;
  for (int curPos = minSubPos; curPos < maxSubPos; curPos++)
  {
    currRasterPos = m_scanOrder[curPos].idx;
    prevScanPos = (curPos == 0) ? 0 : (curPos - 1) % buffersize;
    roffset = (curPos >> log2Width) << log2Width;
    aboveScanPos = roffset - (curPos - roffset + 1);
    aboveScanPos %= buffersize;
    currScanPos = curPos % buffersize;
    if ((pltScanMode == PLT_SCAN_HORTRAV && curPos < width) || (pltScanMode == PLT_SCAN_VERTRAV && curPos < height))
    {
      aboveScanPos = -1; // first column/row: above row is not valid
    }

// Trellis stats: 
// 1st state: same as previous scanned sample
// 2nd state: Copy_Above mode
// 3rd state: Index mode 
// Loop of current state
    for ( int curState = 0; curState < NUM_TRELLIS_STATE; curState++ ) 
    {
      double    minRdCost          = MAX_DOUBLE;
      int       minState           = 0; // best prevState
      uint8_t   bestRunIndex       = 0;
      bool      bestRunType        = 0;
      bool      bestPrevCodedType  = 0;
      int       bestPrevCodedPos   = 0;
      if ( ( curState == 0 && curPos == 0 ) || ( curState == 1 && aboveScanPos < 0 ) ) // state not available
      {
        m_stateCostRDOQ[1 - refId][curState] = MAX_DOUBLE;
        continue;
      }

      bool    runType  = 0;
      uint8_t runIndex = 0;
      if ( curState == 1 ) // 2nd state: Copy_Above mode
      {
        runType = PLT_RUN_COPY;
      }
      else if ( curState == 2 ) // 3rd state: Index mode 
      {
        runType = PLT_RUN_INDEX;
        runIndex = m_minErrorIndexMap[currRasterPos];
      }

// Loop of previous state
      for ( int stateID = 0; stateID < NUM_TRELLIS_STATE; stateID++ ) 
      {
        if ( m_stateCostRDOQ[refId][stateID] == MAX_DOUBLE )
        {
          continue;
        }
        if ( curState == 0 ) // 1st state: same as previous scanned sample
        {
          runType = m_runMapRDOQ[refId][stateID][prevScanPos];
          runIndex = ( runType == PLT_RUN_INDEX ) ? m_indexMapRDOQ[refId][stateID][ prevScanPos ] : m_indexMapRDOQ[refId][stateID][ aboveScanPos ];
        }
        else if ( curState == 1 ) // 2nd state: Copy_Above mode
        {
          runIndex = m_indexMapRDOQ[refId][stateID][aboveScanPos];
        }
        bool    prevRunType   = m_runMapRDOQ[refId][stateID][prevScanPos];
        uint8_t prevRunIndex  = m_indexMapRDOQ[refId][stateID][prevScanPos];
        uint8_t aboveRunIndex = (aboveScanPos >= 0) ? m_indexMapRDOQ[refId][stateID][aboveScanPos] : 0;
        int      dist = curPos - m_prevRunPosRDOQ[refId][stateID] - 1;
        double rdCost = m_stateCostRDOQ[refId][stateID];
        if ( rdCost >= minRdCost ) continue;

// Calculate Rd cost 
        bool prevCodedRunType = m_prevRunTypeRDOQ[refId][stateID];
        int  prevCodedPos     = m_prevRunPosRDOQ [refId][stateID];
        const BinFracBits* fracBitsPt = (m_prevRunTypeRDOQ[refId][stateID] == PLT_RUN_INDEX) ? fracBitsPltIndexINDEX : fracBitsPltIndexCOPY;
        rdCost += rateDistOptPLT(runType, runIndex, prevRunType, prevRunIndex, aboveRunIndex, prevCodedRunType, prevCodedPos, curPos, (pltScanMode == PLT_SCAN_HORTRAV) ? width : height, dist, indexMaxValue, fracBitsPt, fracBitsPltRunType);
        if (rdCost < minRdCost) // update minState ( minRdCost )
        {
          minRdCost    = rdCost;
          minState     = stateID;
          bestRunType  = runType;
          bestRunIndex = runIndex;
          bestPrevCodedType = prevCodedRunType;
          bestPrevCodedPos  = prevCodedPos;
        }
      }
// Update trellis info of current state
      m_stateCostRDOQ  [1 - refId][curState]  = minRdCost;
      m_prevRunTypeRDOQ[1 - refId][curState]  = bestPrevCodedType;
      m_prevRunPosRDOQ [1 - refId][curState]  = bestPrevCodedPos;
      m_statePtRDOQ[curState][currRasterPos] = minState;
      int buffer2update = std::min(buffersize, curPos);
      memcpy(m_indexMapRDOQ[1 - refId][curState], m_indexMapRDOQ[refId][minState], sizeof(uint8_t)*buffer2update);
      memcpy(m_runMapRDOQ[1 - refId][curState], m_runMapRDOQ[refId][minState], sizeof(bool)*buffer2update);
      m_indexMapRDOQ[1 - refId][curState][currScanPos] = bestRunIndex;
      m_runMapRDOQ  [1 - refId][curState][currScanPos] = bestRunType;
    }

    if (useRotate) // early terminate: Rd cost >= min cost in horizontal scan
    {
      if ((m_stateCostRDOQ[1 - refId][0] >= minCost) &&
         (m_stateCostRDOQ[1 - refId][1] >= minCost) &&
         (m_stateCostRDOQ[1 - refId][2] >= minCost) )
      {
        return 0;
      }
    }
    refId = 1 - refId;
  }
  return 1;
}

double IntraSearch::rateDistOptPLT(
  bool      runType,
  uint8_t   runIndex,
  bool      prevRunType,
  uint8_t   prevRunIndex,
  uint8_t   aboveRunIndex,
  bool&     prevCodedRunType,
  int&      prevCodedPos,
  int       scanPos,
  uint32_t  width,
  int       dist,
  int       indexMaxValue,
  const BinFracBits* IndexfracBits,
  const BinFracBits& TypefracBits)
{
  double rdCost = 0.0;
  bool identityFlag = !( (runType != prevRunType) || ( (runType == PLT_RUN_INDEX) && (runIndex != prevRunIndex) ) );

  if ( ( !identityFlag && runType == PLT_RUN_INDEX ) || scanPos == 0 ) // encode index value
  {
    uint8_t refIndex = (prevRunType == PLT_RUN_INDEX) ? prevRunIndex : aboveRunIndex;
    refIndex = (scanPos == 0) ? ( indexMaxValue + 1) : refIndex;
    if ( runIndex == refIndex )
    {
      rdCost = MAX_DOUBLE;
      return rdCost;
    }
    rdCost += m_pcRdCost->getLambda()*m_truncBinBits[(runIndex > refIndex) ? runIndex - 1 : runIndex][(scanPos == 0) ? (indexMaxValue + 1) : indexMaxValue];
  }
  rdCost += m_indexError[runIndex][m_scanOrder[scanPos].idx];
  if (scanPos > 0)
  {
    rdCost += m_pcRdCost->getLambda()*( identityFlag ? (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[1] >> SCALE_BITS) : (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[0] >> SCALE_BITS));
  }
  if ( !identityFlag && scanPos >= width && prevRunType != PLT_RUN_COPY )
  {
    rdCost += m_pcRdCost->getLambda()*(TypefracBits.intBits[runType] >> SCALE_BITS);
  }
  if (!identityFlag || scanPos == 0)
  {
    prevCodedRunType = runType;
    prevCodedPos = scanPos;
  }
  return rdCost;
}
uint32_t IntraSearch::getEpExGolombNumBins(uint32_t symbol, uint32_t count)
{
  uint32_t numBins = 0;
  while (symbol >= (uint32_t)(1 << count))
  {
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  numBins++;
  numBins += count;
  assert(numBins <= 32);
  return numBins;
}

uint32_t IntraSearch::getTruncBinBits(uint32_t symbol, uint32_t maxSymbol)
{
  uint32_t idxCodeBit = 0;
  uint32_t thresh;
  if (maxSymbol > 256)
  {
    uint32_t threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }
  uint32_t uiVal = 1 << thresh;
  assert(uiVal <= maxSymbol);
  assert((uiVal << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  uint32_t b = maxSymbol - uiVal;
  assert(b < uiVal);
  if (symbol < uiVal - b)
  {
    idxCodeBit = thresh;
  }
  else
  {
    idxCodeBit = thresh + 1;
  }
  return idxCodeBit;
}

void IntraSearch::initTBCTable(int bitDepth)
{
  for (uint32_t i = 0; i < m_symbolSize; i++)
  {
    memset(m_truncBinBits[i], 0, sizeof(uint16_t)*(m_symbolSize + 1));
  }
  for (uint32_t i = 0; i < (m_symbolSize + 1); i++)
  {
    for (uint32_t j = 0; j < i; j++)
    {
      m_truncBinBits[j][i] = getTruncBinBits(j, i);
    }
  }
  memset(m_escapeNumBins, 0, sizeof(uint16_t)*m_symbolSize);
  for (uint32_t i = 0; i < m_symbolSize; i++)
  {
    m_escapeNumBins[i] = getEpExGolombNumBins(i, 3);
  }
}
#else
void IntraSearch::deriveRunAndCalcBits(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp, PLTScanMode pltScanMode, uint64_t& minBits)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;
  Pel  *runLength = tu.getRunLens (compBegin);
  bool *runType   = tu.getRunTypes(compBegin);

  cu.useRotation[compBegin] = (pltScanMode == PLT_SCAN_VERTRAV);
  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  deriveRun(cs, partitioner, compBegin);

  m_CABACEstimator->getCtx() = PLTCtx(m_orgCtxRD);
  m_CABACEstimator->resetBits();
  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
  m_CABACEstimator->cu_palette_info(cu, compBegin, numComp, cuCtx);
  uint64_t bitsTemp = m_CABACEstimator->getEstFracBits();
  if (minBits > bitsTemp)
  {
    m_bestScanRotationMode = pltScanMode;
    memcpy(m_runTypeRD, runType, sizeof(bool)*width*height);
    memcpy(m_runLengthRD, runLength, sizeof(Pel)*width*height);
    minBits = bitsTemp;
  }
}
void IntraSearch::deriveRun(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;
  uint32_t total = height * width, idx = 0;
  uint32_t startPos = 0;
  uint64_t indexBits = 0, runBitsIndex = 0, runBitsCopy = 0;
  m_storeCtxRun = PLTCtx(m_orgCtxRD);

  PLTtypeBuf  runType = tu.getrunType(compBegin);
  PelBuf      runLength = tu.getrunLength(compBegin);
  while (idx < total)
  {
    startPos = idx;
    double aveBitsPerPix[NUM_PLT_RUN];
    uint32_t indexRun = 0;
    bool runValid = calIndexRun(cs, partitioner, startPos, total, indexRun, compBegin);
    m_CABACEstimator->getCtx() = PLTCtx(m_storeCtxRun);
    aveBitsPerPix[PLT_RUN_INDEX] = runValid ? getRunBits(cu, indexRun, startPos, PLT_RUN_INDEX, &indexBits, &runBitsIndex, compBegin) : MAX_DOUBLE;
    m_storeCtxRunIndex = PLTCtx(m_CABACEstimator->getCtx());

    uint32_t copyRun = 0;
    bool copyValid = calCopyRun(cs, partitioner, startPos, total, copyRun, compBegin);
    m_CABACEstimator->getCtx() = PLTCtx(m_storeCtxRun);
    aveBitsPerPix[PLT_RUN_COPY] = copyValid ? getRunBits(cu, copyRun, startPos, PLT_RUN_COPY, &indexBits, &runBitsCopy, compBegin) : MAX_DOUBLE;
    m_storeCtxRunCopy = PLTCtx(m_CABACEstimator->getCtx());

    if (copyValid == 0 && runValid == 0)
    {
      assert(0);
    }
    else
    {
      if (aveBitsPerPix[PLT_RUN_COPY] <= aveBitsPerPix[PLT_RUN_INDEX])
      {
        for (int runidx = 0; runidx <copyRun; runidx++)
        {
          uint32_t posy = m_scanOrder[idx + runidx].y;
          uint32_t posx = m_scanOrder[idx + runidx].x;
          runType.at(posx, posy) = PLT_RUN_COPY;
          runLength.at(posx, posy) = copyRun;
        }
        idx += copyRun;
        m_storeCtxRun = PLTCtx(m_storeCtxRunCopy);

      }
      else
      {
        for (int runidx = 0; runidx <indexRun; runidx++)
        {
          uint32_t posy = m_scanOrder[idx + runidx].y;
          uint32_t posx = m_scanOrder[idx + runidx].x;
          runType.at(posx, posy) = PLT_RUN_INDEX;
          runLength.at(posx, posy) = indexRun;
        }
        idx += indexRun;
        m_storeCtxRun = PLTCtx(m_storeCtxRunIndex);

      }
    }
  }
  assert(idx == total);
}
double IntraSearch::getRunBits(const CodingUnit&  cu, uint32_t run, uint32_t strPos, PLTRunMode paletteRunMode, uint64_t* indexBits, uint64_t* runBits, ComponentID compBegin)
{
  TransformUnit&   tu = *cu.firstTU;
  uint32_t height = cu.block(compBegin).height;
  uint32_t width  = cu.block(compBegin).width;
  uint32_t endPos = height*width;
  PLTtypeBuf runType   = tu.getrunType(compBegin);
  PelBuf     curPLTIdx = tu.getcurPLTIdx(compBegin);
  uint32_t   indexMaxSize = (cu.useEscape[compBegin]) ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];

  m_CABACEstimator->resetBits();
  ///////////////// encode Run Type
  m_CABACEstimator->encodeRunType(cu, runType, strPos, m_scanOrder, compBegin);
  uint64_t runTypeBits = m_CABACEstimator->getEstFracBits();
  uint32_t curLevel = 0;
  switch (paletteRunMode)
  {
  case PLT_RUN_INDEX:
    curLevel = m_CABACEstimator->writePLTIndex(cu, strPos, curPLTIdx, runType, indexMaxSize, compBegin);
    *indexBits = m_CABACEstimator->getEstFracBits() - runTypeBits;
    m_CABACEstimator->cu_run_val(run - 1, PLT_RUN_INDEX, curLevel, endPos - strPos - 1);
    *runBits = m_CABACEstimator->getEstFracBits() - runTypeBits - (*indexBits);
    break;
  case PLT_RUN_COPY:
    m_CABACEstimator->cu_run_val(run - 1, PLT_RUN_COPY, curLevel, endPos - strPos - 1);
    *runBits = m_CABACEstimator->getEstFracBits() - runTypeBits;
    break;
  default:
    assert(0);
  }
  assert(run >= 1);
  double costPerPixel = (double)m_CABACEstimator->getEstFracBits() / (double)run;
  return costPerPixel;
}
void IntraSearch::preCalcPLTIndex(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  const int  channelBitDepth_L = cs.sps->getBitDepth(CHANNEL_TYPE_LUMA);
  const int  channelBitDepth_C = cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA);
  const int  pcmShiftRight_L = (channelBitDepth_L - PLT_ENCBITDEPTH);
  const int  pcmShiftRight_C = (channelBitDepth_C - PLT_ENCBITDEPTH);

  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  PelBuf   curPLTIdx = tu.getcurPLTIdx(compBegin);
  int      errorLimit = numComp * g_paletteQuant[cu.qp];
  uint32_t bestIdx = 0;
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      uint32_t pltIdx = 0;
      uint32_t minError = MAX_UINT;
      while (pltIdx < cu.curPLTSize[compBegin])
      {
        uint32_t absError = 0, pX, pY;
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          pX = (comp > 0 && compBegin == COMPONENT_Y) ? (x >> scaleX) : x;
          pY = (comp > 0 && compBegin == COMPONENT_Y) ? (y >> scaleY) : y;
#if JVET_P0526_PLT_ENCODER
          if (isChroma((ComponentID) comp))
          {
            absError += int(double(abs(cu.curPLT[comp][pltIdx] - orgBuf[comp].at(pX, pY))) * PLT_CHROMA_WEIGHTING) >> pcmShiftRight_C;
          }
          else
          {
            absError += abs(cu.curPLT[comp][pltIdx] - orgBuf[comp].at(pX, pY)) >> pcmShiftRight_L;
          }
#else
          int shift = (comp > 0) ? pcmShiftRight_C : pcmShiftRight_L;
          absError += abs(cu.curPLT[comp][pltIdx] - orgBuf[comp].at(pX, pY)) >> shift;
#endif
        }

        if (absError < minError)
        {
          bestIdx = pltIdx;
          minError = absError;
          if (minError == 0)
          {
            break;
          }
        }
        pltIdx++;
      }
      curPLTIdx.at(x, y) = bestIdx;
      if (minError > errorLimit)
      {
        curPLTIdx.at(x, y) = cu.curPLTSize[compBegin];
        cu.useEscape[compBegin] = true;
        calcPixelPred(cs, partitioner, y, x, compBegin, numComp);
      }
    }
  }
}
#endif
void IntraSearch::calcPixelPred(CodingStructure& cs, Partitioner& partitioner, uint32_t yPos, uint32_t xPos, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
#if JVET_P1006_PICTURE_HEADER
    if (m_pcEncCfg->getLmcs() && (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#else
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#endif
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    QpParam cQP(tu, ComponentID(ch));
#if JVET_P0460_PLT_TS_MIN_QP
    qp[ch] = cQP.Qp(true);
#else
    qp[ch] = cQP.Qp(false);
#endif
    qpRem[ch] = qp[ch] % 6;
    qpPer[ch] = qp[ch] / 6;
    quantiserScale[ch] = g_quantScales[0][qpRem[ch]];
    quantiserRightShift[ch] = QUANT_SHIFT + qpPer[ch];
    rightShiftOffset[ch] = 1 << (quantiserRightShift[ch] - 1);
    invquantiserRightShift[ch] = IQUANT_SHIFT;
    add[ch] = 1 << (invquantiserRightShift[ch] - 1);
  }

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int channelBitDepth = cu.cs->sps->getBitDepth(toChannelType((ComponentID)ch));
    CompArea  area = cu.blocks[ch];
    PelBuf    recBuf = cs.getRecoBuf(area);
    PLTescapeBuf escapeValue = tu.getescapeValue((ComponentID)ch);
    if (compBegin != COMPONENT_Y || ch == 0)
    {
      escapeValue.at(xPos, yPos) = TCoeff(std::max<int>(0, ((orgBuf[ch].at(xPos, yPos) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
      assert(escapeValue.at(xPos, yPos) < (1 << (channelBitDepth + 1)));
      recBuf.at(xPos, yPos) = (((escapeValue.at(xPos, yPos)*g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
      recBuf.at(xPos, yPos) = Pel(ClipBD<int>(recBuf.at(xPos, yPos), channelBitDepth));//to be checked
    }
    else if (compBegin == COMPONENT_Y && ch > 0 && yPos % (1 << scaleY) == 0 && xPos % (1 << scaleX) == 0)
    {
      uint32_t yPosC = yPos >> scaleY;
      uint32_t xPosC = xPos >> scaleX;
      escapeValue.at(xPosC, yPosC) = TCoeff(std::max<int>(0, ((orgBuf[ch].at(xPosC, yPosC) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
      assert(escapeValue.at(xPosC, yPosC) < (1 << (channelBitDepth + 1)));
      recBuf.at(xPosC, yPosC) = (((escapeValue.at(xPosC, yPosC)*g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
      recBuf.at(xPosC, yPosC) = Pel(ClipBD<int>(recBuf.at(xPosC, yPosC), channelBitDepth));//to be checked
    }
  }
}
void IntraSearch::derivePLTLossy(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  const int channelBitDepth_L = cs.sps->getBitDepth(CHANNEL_TYPE_LUMA);
  const int channelBitDepth_C = cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA);
  const int pcmShiftRight_L = (channelBitDepth_L - PLT_ENCBITDEPTH);
  const int pcmShiftRight_C = (channelBitDepth_C - PLT_ENCBITDEPTH);

  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
#if JVET_P1006_PICTURE_HEADER
    if (m_pcEncCfg->getLmcs() && (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#else
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
#endif
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int errorLimit = g_paletteQuant[cu.qp];
  uint32_t totalSize = height*width;
  SortingElement *pelList = new SortingElement[totalSize];
  SortingElement  element;
  SortingElement *pelListSort = new SortingElement[MAXPLTSIZE + 1];
  uint32_t dictMaxSize = MAXPLTSIZE;
  uint32_t idx = 0;
  int last = -1;

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      uint32_t org[3], pX, pY;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        pX = (comp > 0 && compBegin == COMPONENT_Y) ? (x >> scaleX) : x;
        pY = (comp > 0 && compBegin == COMPONENT_Y) ? (y >> scaleY) : y;
        org[comp] = orgBuf[comp].at(pX, pY);
      }
      element.setAll(org, compBegin, numComp);
      int besti = last, bestSAD = (last == -1) ? MAX_UINT : pelList[last].getSAD(element, cs.sps->getBitDepths(), compBegin, numComp);
      if (bestSAD)
      {
        for (int i = idx - 1; i >= 0; i--)
        {
          uint32_t sad = pelList[i].getSAD(element, cs.sps->getBitDepths(), compBegin, numComp);
          if (sad < bestSAD)
          {
            bestSAD = sad;
            besti = i;
            if (!sad) break;
          }
        }
      }
      if (besti >= 0 && pelList[besti].almostEqualData(element, errorLimit, cs.sps->getBitDepths(), compBegin, numComp))
      {
        pelList[besti].addElement(element, compBegin, numComp);
        last = besti;
      }
      else
      {
        pelList[idx].copyDataFrom(element, compBegin, numComp);
        pelList[idx].setCnt(1);
        last = idx;
        idx++;
      }
    }
  }

  for (int i = 0; i < dictMaxSize; i++)
  {
    pelListSort[i].setCnt(0);
    pelListSort[i].resetAll(compBegin, numComp);
  }

  //bubble sorting
  dictMaxSize = 1;
  for (int i = 0; i < idx; i++)
  {
    if (pelList[i].getCnt() > pelListSort[dictMaxSize - 1].getCnt())
    {
      int j;
      for (j = dictMaxSize; j > 0; j--)
      {
        if (pelList[i].getCnt() > pelListSort[j - 1].getCnt() )
        {
          pelListSort[j].copyAllFrom(pelListSort[j - 1], compBegin, numComp);
          dictMaxSize = std::min(dictMaxSize + 1, (uint32_t)MAXPLTSIZE);
        }
        else
        {
          break;
        }
      }
      pelListSort[j].copyAllFrom(pelList[i], compBegin, numComp);
    }
  }

  uint32_t paletteSize = 0;
  uint64_t numColorBits = 0;
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    numColorBits += (comp > 0) ? channelBitDepth_C : channelBitDepth_L;
  }
#if JVET_P0526_PLT_ENCODER
  const int plt_lambda_shift = (compBegin > 0) ? pcmShiftRight_C : pcmShiftRight_L;
  double    bitCost          = m_pcRdCost->getLambda() / (double) (1 << (2 * plt_lambda_shift)) * numColorBits;
#else
  double bitCost = m_pcRdCost->getLambda()*numColorBits;
#endif
  for (int i = 0; i < MAXPLTSIZE; i++)
  {
    if (pelListSort[i].getCnt())
    {
      int half = pelListSort[i].getCnt() >> 1;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        cu.curPLT[comp][paletteSize] = (pelListSort[i].getSumData(comp) + half) / pelListSort[i].getCnt();
      }

      int best = -1;
      if (errorLimit)
      {
        double pal[MAX_NUM_COMPONENT], err = 0.0, bestCost = 0.0;
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
#if !JVET_P0526_PLT_ENCODER
          const int shift = (comp > 0) ? pcmShiftRight_C : pcmShiftRight_L;
#endif
          pal[comp] = pelListSort[i].getSumData(comp) / (double)pelListSort[i].getCnt();
          err = pal[comp] - cu.curPLT[comp][paletteSize];
#if JVET_P0526_PLT_ENCODER
          if (isChroma((ComponentID) comp))
          {
            bestCost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRight_C));
          }
          else
          {
            bestCost += (err * err) / (1 << (2 * pcmShiftRight_L));
          }
#else
          bestCost += (err*err) / (1 << (2 * shift));
#endif
        }
        bestCost = bestCost * pelListSort[i].getCnt() + bitCost;

        for (int t = 0; t < cs.prevPLT.curPLTSize[compBegin]; t++)
        {
          double cost = 0.0;
          for (int comp = compBegin; comp < (compBegin + numComp); comp++)
          {
#if !JVET_P0526_PLT_ENCODER
            const int shift = (comp > 0) ? pcmShiftRight_C : pcmShiftRight_L;
#endif
            err = pal[comp] - cs.prevPLT.curPLT[comp][t];
#if JVET_P0526_PLT_ENCODER
            if (isChroma((ComponentID) comp))
            {
              cost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRight_C));
            }
            else
            {
              cost += (err * err) / (1 << (2 * pcmShiftRight_L));
            }
#else
            cost += (err*err) / (1 << (2 * shift));
#endif
          }
          cost *= pelListSort[i].getCnt();
          if (cost < bestCost)
          {
            best = t;
            bestCost = cost;
          }
        }
        if (best != -1)
        {
          for (int comp = compBegin; comp < (compBegin + numComp); comp++)
          {
            cu.curPLT[comp][paletteSize] = cs.prevPLT.curPLT[comp][best];
          }
        }
      }

      bool duplicate = false;
      if (pelListSort[i].getCnt() == 1 && best == -1)
      {
        duplicate = true;
      }
      else
      {
        for (int t = 0; t<paletteSize; t++)
        {
          bool duplicateTmp = true;
          for (int comp = compBegin; comp < (compBegin + numComp); comp++)
          {
            duplicateTmp = duplicateTmp && (cu.curPLT[comp][paletteSize] == cu.curPLT[comp][t]);
          }
          if (duplicateTmp)
          {
            duplicate = true;
            break;
          }
        }
      }
      if (!duplicate) paletteSize++;
    }
    else
    {
      break;
    }
  }
  cu.curPLTSize[compBegin] = paletteSize;

  delete[] pelList;
  delete[] pelListSort;
}
// -------------------------------------------------------------------------------------------------------------------
// Intra search
// -------------------------------------------------------------------------------------------------------------------

void IntraSearch::xEncIntraHeader( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx )
{
  CodingUnit &cu = *cs.getCU( partitioner.chType );

  if (bLuma)
  {
    bool isFirst = cu.ispMode ? subTuIdx == 0 : partitioner.currArea().lumaPos() == cs.area.lumaPos();

    // CU header
    if( isFirst )
    {
      if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag() || cs.slice->getSPS()->getPLTMode())
      && cu.Y().valid()
      )
      {
        m_CABACEstimator->cu_skip_flag( cu );
        m_CABACEstimator->pred_mode   ( cu );
      }
      if (CU::isPLT(cu))
      {
        return;
      }
      m_CABACEstimator->bdpcm_mode  ( cu, ComponentID(partitioner.chType) );
#if JVET_P0059_CHROMA_BDPCM
      if (!CS::isDualITree(cs) && isLuma(partitioner.chType))
          m_CABACEstimator->bdpcm_mode(cu, ComponentID(CHANNEL_TYPE_CHROMA));
#endif
    }

    PredictionUnit &pu = *cs.getPU(partitioner.currArea().lumaPos(), partitioner.chType);

    // luma prediction mode
    if (isFirst)
    {
      if ( !cu.Y().valid())
        m_CABACEstimator->pred_mode( cu );
      m_CABACEstimator->intra_luma_pred_mode( pu );
    }
  }

  if (bChroma)
  {
    bool isFirst = partitioner.currArea().Cb().valid() && partitioner.currArea().chromaPos() == cs.area.chromaPos();

    PredictionUnit &pu = *cs.getPU( partitioner.currArea().chromaPos(), CHANNEL_TYPE_CHROMA );

    if( isFirst )
    {
      m_CABACEstimator->intra_chroma_pred_mode( pu );
    }
  }
}

void IntraSearch::xEncSubdivCbfQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx, const PartSplit ispType )
{
  const UnitArea &currArea = partitioner.currArea();
          int subTuCounter = subTuIdx;
  TransformUnit &currTU = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType, subTuCounter );
  CodingUnit    &currCU = *currTU.cu;
  uint32_t currDepth           = partitioner.currTrDepth;

  const bool subdiv        = currTU.depth > currDepth;
  ComponentID compID = partitioner.chType == CHANNEL_TYPE_LUMA ? COMPONENT_Y : COMPONENT_Cb;
  const bool chromaCbfISP = currArea.blocks[COMPONENT_Cb].valid() && currCU.ispMode && !subdiv;

  if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
  {
    CHECK( !subdiv, "TU split implied" );
  }
  else
  {
    CHECK( subdiv && !currCU.ispMode && isLuma( compID ), "No TU subdivision is allowed with QTBT" );
  }

  if( bChroma && ( !currCU.ispMode || chromaCbfISP ) )
  {
    const uint32_t numberValidComponents = getNumberValidComponents(currArea.chromaFormat);
    const uint32_t cbfDepth = ( chromaCbfISP ? currDepth - 1 : currDepth );

    for (uint32_t ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
    {
      const ComponentID compID = ComponentID(ch);

      if( currDepth == 0 || TU::getCbfAtDepth( currTU, compID, currDepth - 1 ) || chromaCbfISP )
      {
        const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth ) : false );
        m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, compID, currDepth ), currArea.blocks[compID], cbfDepth, prevCbf );

      }
    }
  }

  if (subdiv)
  {

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( currCU.ispMode && isLuma( compID ) )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
    THROW( "Cannot perform an implicit split!" );

    do
    {
      xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma, subTuCounter, ispType );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    //===== Cbfs =====
    if (bLuma)
    {
      bool previousCbf       = false;
      bool lastCbfIsInferred = false;
      if( ispType != TU_NO_ISP )
      {
        bool rootCbfSoFar = false;
        uint32_t nTus = currCU.ispMode == HOR_INTRA_SUBPARTITIONS ? currCU.lheight() >> floorLog2(currTU.lheight()) : currCU.lwidth() >> floorLog2(currTU.lwidth());
        if( subTuCounter == nTus - 1 )
        {
          TransformUnit* tuPointer = currCU.firstTU;
          for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
          {
            rootCbfSoFar |= TU::getCbfAtDepth( *tuPointer, COMPONENT_Y, currDepth );
            tuPointer = tuPointer->next;
          }
          if( !rootCbfSoFar )
          {
            lastCbfIsInferred = true;
          }
        }
        if( !lastCbfIsInferred )
        {
          previousCbf = TU::getPrevTuCbfAtDepth( currTU, COMPONENT_Y, partitioner.currTrDepth );
        }
      }
      if( !lastCbfIsInferred )
      {
        m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, COMPONENT_Y, currDepth ), currTU.Y(), currTU.depth, previousCbf, currCU.ispMode );
      }
    }
  }
}

#if JVET_P1026_ISP_LFNST_COMBINATION
void IntraSearch::xEncCoeffQT( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID, const int subTuIdx, const PartSplit ispType, CUCtx* cuCtx )
#else
void IntraSearch::xEncCoeffQT( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID, const int subTuIdx, const PartSplit ispType )
#endif
{
  const UnitArea &currArea  = partitioner.currArea();

       int subTuCounter     = subTuIdx;
  TransformUnit &currTU     = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType, subTuIdx );
  uint32_t      currDepth       = partitioner.currTrDepth;
  const bool subdiv         = currTU.depth > currDepth;

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if( currTU.cu->ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
      THROW("Implicit TU split not available!");

    do
    {
#if JVET_P1026_ISP_LFNST_COMBINATION
      xEncCoeffQT( cs, partitioner, compID, subTuCounter, ispType, cuCtx );
#else
      xEncCoeffQT( cs, partitioner, compID, subTuCounter, ispType );
#endif
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else

  if( currArea.blocks[compID].valid() )
  {
    if( compID == COMPONENT_Cr )
    {
      const int cbfMask = ( TU::getCbf( currTU, COMPONENT_Cb ) ? 2 : 0 ) + ( TU::getCbf( currTU, COMPONENT_Cr ) ? 1 : 0 );
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
    if( TU::hasCrossCompPredInfo( currTU, compID ) )
    {
      m_CABACEstimator->cross_comp_pred( currTU, compID );
    }
    if( TU::getCbf( currTU, compID ) )
    {
#if JVET_P1026_MTS_SIGNALLING
      if( isLuma(compID) )
      {
#if JVET_P1026_ISP_LFNST_COMBINATION
        m_CABACEstimator->residual_coding( currTU, compID, cuCtx );
        m_CABACEstimator->mts_idx( *currTU.cu, cuCtx );
#else
        CUCtx cuCtx;
        m_CABACEstimator->residual_coding( currTU, compID, &cuCtx );
        m_CABACEstimator->mts_idx( *currTU.cu, cuCtx );
#endif
      }
      else
#endif
      m_CABACEstimator->residual_coding( currTU, compID );
    }
  }
}

#if JVET_P1026_ISP_LFNST_COMBINATION
uint64_t IntraSearch::xGetIntraFracBitsQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx, const PartSplit ispType, CUCtx* cuCtx )
#else
uint64_t IntraSearch::xGetIntraFracBitsQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx, const PartSplit ispType )
#endif
{
  m_CABACEstimator->resetBits();

  xEncIntraHeader( cs, partitioner, bLuma, bChroma, subTuIdx );
  xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma, subTuIdx, ispType );


  if( bLuma )
  {
#if JVET_P1026_ISP_LFNST_COMBINATION
    xEncCoeffQT( cs, partitioner, COMPONENT_Y, subTuIdx, ispType, cuCtx );
#else
    xEncCoeffQT( cs, partitioner, COMPONENT_Y, subTuIdx, ispType );
#endif
  }
  if( bChroma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Cb, subTuIdx, ispType );
    xEncCoeffQT( cs, partitioner, COMPONENT_Cr, subTuIdx, ispType );
  }

#if JVET_P1026_ISP_LFNST_COMBINATION
  CodingUnit& cu = *cs.getCU(partitioner.chType);
  if ( cuCtx && bLuma && cu.isSepTree() && ( !cu.ispMode || ( cu.lfnstIdx && subTuIdx == 0 ) || ( !cu.lfnstIdx && subTuIdx == m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1] - 1 ) ) )
  {
    m_CABACEstimator->residual_lfnst_mode(cu, *cuCtx);
  }
#endif

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTSingleChromaComponent( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID )
{
  m_CABACEstimator->resetBits();

  if( compID == COMPONENT_Cb )
  {
    //intra mode coding
    PredictionUnit &pu = *cs.getPU( partitioner.currArea().lumaPos(), partitioner.chType );
    m_CABACEstimator->intra_chroma_pred_mode( pu );
    //xEncIntraHeader(cs, partitioner, false, true);
  }
  CHECK( partitioner.currTrDepth != 1, "error in the depth!" );
  const UnitArea &currArea = partitioner.currArea();

  TransformUnit &currTU = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );

  //cbf coding
  const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbfAtDepth( currTU, COMPONENT_Cb, partitioner.currTrDepth ) : false );
  m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, compID, partitioner.currTrDepth ), currArea.blocks[compID], partitioner.currTrDepth - 1, prevCbf );
  //coeffs coding and cross comp coding
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

uint64_t IntraSearch::xGetIntraFracBitsQTChroma(TransformUnit& currTU, const ComponentID &compID)
{
  m_CABACEstimator->resetBits();

  if( TU::hasCrossCompPredInfo( currTU, compID ) )
  {
    m_CABACEstimator->cross_comp_pred( currTU, compID );
  }

  // Include Cbf and jointCbCr flags here as we make decisions across components
  CodingStructure &cs = *currTU.cs;

  if ( currTU.jointCbCr )
  {
    const int cbfMask = ( TU::getCbf( currTU, COMPONENT_Cb ) ? 2 : 0 ) + ( TU::getCbf( currTU, COMPONENT_Cr ) ? 1 : 0 );
    m_CABACEstimator->cbf_comp( cs, cbfMask>>1, currTU.blocks[ COMPONENT_Cb ], currTU.depth, false );
    m_CABACEstimator->cbf_comp( cs, cbfMask &1, currTU.blocks[ COMPONENT_Cr ], currTU.depth, cbfMask>>1 );
    if( cbfMask )
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    if( cbfMask >> 1 )
      m_CABACEstimator->residual_coding( currTU, COMPONENT_Cb );
    if( cbfMask & 1 )
      m_CABACEstimator->residual_coding( currTU, COMPONENT_Cr );
  }
  else
  {
    if ( compID == COMPONENT_Cb )
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, compID ), currTU.blocks[ compID ], currTU.depth, false );
    else
    {
      const bool cbCbf    = TU::getCbf( currTU, COMPONENT_Cb );
      const bool crCbf    = TU::getCbf( currTU, compID );
      const int  cbfMask  = ( cbCbf ? 2 : 0 ) + ( crCbf ? 1 : 0 );
      m_CABACEstimator->cbf_comp( cs, crCbf, currTU.blocks[ compID ], currTU.depth, cbCbf );
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
  }

  if( !currTU.jointCbCr && TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

void IntraSearch::xIntraCodingTUBlock(TransformUnit &tu, const ComponentID &compID, const bool &checkCrossCPrediction, Distortion& ruiDist, const int &default0Save1Load2, uint32_t* numSig, std::vector<TrMode>* trModes, const bool loadTr)
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }

  CodingStructure &cs                       = *tu.cs;
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());

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


  //===== init availability pattern =====
  CHECK( tu.jointCbCr && compID == COMPONENT_Cr, "wrong combination of compID and jointCbCr" );
  bool jointCbCr = tu.jointCbCr && compID == COMPONENT_Cb;

#if JVET_P0059_CHROMA_BDPCM
  if (compID == COMPONENT_Y || (isChroma(compID) && tu.cu->bdpcmModeChroma))
#else
  if ( compID == COMPONENT_Y )
#endif
  {
  PelBuf sharedPredTS( m_pSharedPredTransformSkip[compID], area );
  if( default0Save1Load2 != 2 )
  {
    bool predRegDiffFromTB = CU::isPredRegDiffFromTB(*tu.cu, compID);
    bool firstTBInPredReg = CU::isFirstTBInPredReg(*tu.cu, compID, area);
    CompArea areaPredReg(COMPONENT_Y, tu.chromaFormat, area);
    if (tu.cu->ispMode && isLuma(compID))
    {
      if (predRegDiffFromTB)
      {
        if (firstTBInPredReg)
        {
          CU::adjustPredArea(areaPredReg);
          initIntraPatternChTypeISP(*tu.cu, areaPredReg, piReco);
        }
      }
      else
        initIntraPatternChTypeISP(*tu.cu, area, piReco);
    }
    else
    {
      initIntraPatternChType(*tu.cu, area);
    }

    //===== get prediction signal =====
#if JVET_P0059_CHROMA_BDPCM
    if(compID != COMPONENT_Y && !tu.cu->bdpcmModeChroma && PU::isLMCMode(uiChFinalMode))
#else
    if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
#endif
    {
      {
        xGetLumaRecPixels( pu, area );
      }
      predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
    }
    else
    {
      if( PU::isMIP( pu, chType ) )
      {
#if JVET_P0803_COMBINED_MIP_CLEANUP
        initIntraMip( pu, area );
#endif
        predIntraMip( compID, piPred, pu );
      }
      else
      {
        if (predRegDiffFromTB)
        {
          if (firstTBInPredReg)
          {
            PelBuf piPredReg = cs.getPredBuf(areaPredReg);
            predIntraAng(compID, piPredReg, pu);
          }
        }
        else
          predIntraAng(compID, piPred, pu);
      }
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
  }


  DTRACE( g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), uiChFinalMode );
  //DTRACE_PEL_BUF( D_PRED, piPred, tu, tu.cu->predMode, COMPONENT_Y );

  const Slice           &slice = *cs.slice;
#if JVET_P1006_PICTURE_HEADER
  bool flag = slice.getPicHeader()->getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
#else
  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
#endif
  if (isLuma(compID))
  {
  //===== get residual signal =====
  piResi.copyFrom( piOrg  );
#if JVET_P1006_PICTURE_HEADER
  if (slice.getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
#else
  if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
#endif
  {
    CompArea      tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
    PelBuf tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
    tmpPred.copyFrom(piPred);
    piResi.rspSignal(m_pcReshape->getFwdLUT());
    piResi.subtract(tmpPred);
  }
  else
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
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  //--- transform and quantization           ---
  TCoeff uiAbsSum = 0;

  const QpParam cQP(tu, compID);

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda(compID);
#endif

  flag =flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
#if JVET_P1006_PICTURE_HEADER
  if (flag && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() )
#else
  if (flag && isChroma(compID) && slice.getLmcsChromaResidualScaleFlag() )
#endif
  {
    int cResScaleInv = tu.getChromaAdj();
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)cResScaleInv;
    m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cResScale*cResScale));
  }

  const CompArea &crArea = tu.blocks     [ COMPONENT_Cr ];
  PelBuf          crOrg  = cs.getOrgBuf  ( crArea );
  PelBuf          crPred = cs.getPredBuf ( crArea );
  PelBuf          crResi = cs.getResiBuf ( crArea );
  PelBuf          crReco = cs.getRecoBuf ( crArea );

  if ( jointCbCr )
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
    const int    absIct = abs( TU::getICTMode(tu) );
    const double lfact  = ( absIct == 1 || absIct == 3 ? 0.8 : 0.5 );
    m_pcTrQuant->setLambda( lfact * m_pcTrQuant->getLambda() );
  }
  if ( sps.getJointCbCrEnabledFlag() && isChroma(compID) && (tu.cu->cs->slice->getSliceQp() > 18) )
  {
    m_pcTrQuant->setLambda( 1.3 * m_pcTrQuant->getLambda() );
  }

  if( isLuma(compID) )
  {
    if (trModes)
    {
#if JVET_P0273_MTSIntraMaxCand
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, m_pcEncCfg->getMTSIntraMaxCand());
#else
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, CU::isIntra(*tu.cu) ? m_pcEncCfg->getIntraMTSMaxCand() : m_pcEncCfg->getInterMTSMaxCand());
#endif
#if JVET_P0058_CHROMA_TS
      tu.mtsIdx[compID] = trModes->at(0).first;
#else
      tu.mtsIdx = trModes->at(0).first;
#endif
    }
#if JVET_AHG14_LOSSLESS
    if( !( m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && tu.mtsIdx[compID] == 0 ) || tu.cu->bdpcmMode != 0 )
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
#else
    m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
#endif


  DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), compID, uiAbsSum );

  if (tu.cu->ispMode && isLuma(compID) && CU::isISPLast(*tu.cu, area, area.compID) && CU::allLumaCBFsAreZero(*tu.cu))
  {
    // ISP has to have at least one non-zero CBF
    ruiDist = MAX_INT;
    return;
  }

#if JVET_AHG14_LOSSLESS
  if( ( m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && tu.mtsIdx[compID] == 0 ) && 0 == tu.cu->bdpcmMode )
  {
    uiAbsSum = 0;
    tu.getCoeffs( compID ).fill( 0 );
    TU::setCbfAtDepth( tu, compID, tu.depth, 0 );
  }
#endif

  //--- inverse transform ---
  if (uiAbsSum > 0)
  {
    m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
  }
  else
  {
    piResi.fill(0);
  }
  }
  else // chroma
  {
    int         codedCbfMask  = 0;
    ComponentID codeCompId    = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr) : compID);
    const QpParam qpCbCr(tu, codeCompId);

    if( tu.jointCbCr )
    {
      ComponentID otherCompId = ( codeCompId==COMPONENT_Cr ? COMPONENT_Cb : COMPONENT_Cr );
      tu.getCoeffs( otherCompId ).fill(0); // do we need that?
      TU::setCbfAtDepth (tu, otherCompId, tu.depth, false );
    }
    PelBuf& codeResi = ( codeCompId == COMPONENT_Cr ? crResi : piResi );
    uiAbsSum = 0;

#if JVET_P0058_CHROMA_TS
    if (trModes)
    {
#if JVET_P0273_MTSIntraMaxCand
        m_pcTrQuant->transformNxN(tu, compID, qpCbCr, trModes, m_pcEncCfg->getMTSIntraMaxCand());
#else
        m_pcTrQuant->transformNxN(tu, compID, qpCbCr, trModes, CU::isIntra(*tu.cu) ? m_pcEncCfg->getIntraMTSMaxCand() : m_pcEncCfg->getInterMTSMaxCand());
#endif
        tu.mtsIdx[compID] = trModes->at(0).first;
    }
#endif
#if JVET_P0058_CHROMA_TS
    // encoder bugfix: Set loadTr to aovid redundant transform process
#if JVET_AHG14_LOSSLESS
#if JVET_P0059_CHROMA_BDPCM
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && tu.mtsIdx[compID] == 0) || tu.cu->bdpcmModeChroma != 0)
#else
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && tu.mtsIdx[compID] == 0))
#endif
    {
        m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
#else
    m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
#endif
#else
    m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx());
#endif

#if JVET_AHG14_LOSSLESS
#if JVET_P0059_CHROMA_BDPCM
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && tu.mtsIdx[compID] == 0) && 0 == tu.cu->bdpcmModeChroma)
#else
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && tu.mtsIdx[compID] == 0))
#endif
    {
        uiAbsSum = 0;
        tu.getCoeffs(compID).fill(0);
        TU::setCbfAtDepth(tu, compID, tu.depth, 0);
    }
#endif

    DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), codeCompId, uiAbsSum );
    if( uiAbsSum > 0 )
    {
      m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
      codedCbfMask += ( codeCompId == COMPONENT_Cb ? 2 : 1 );
    }
    else
    {
      codeResi.fill(0);
    }

    if( tu.jointCbCr )
    {
      if( tu.jointCbCr == 3 && codedCbfMask == 2 )
      {
        codedCbfMask = 3;
        TU::setCbfAtDepth (tu, COMPONENT_Cr, tu.depth, true );
      }
      if( tu.jointCbCr != codedCbfMask )
      {
        ruiDist = std::numeric_limits<Distortion>::max();
        return;
      }
      m_pcTrQuant->invTransformICT( tu, piResi, crResi );
      uiAbsSum = codedCbfMask;
    }
  }

  //===== reconstruction =====
#if JVET_P1006_PICTURE_HEADER
  if ( flag && uiAbsSum > 0 && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() )
#else
  if ( flag && uiAbsSum > 0 && isChroma(compID) && slice.getLmcsChromaResidualScaleFlag() )
#endif
  {
    piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
    if( jointCbCr )
    {
      crResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
    }
  }
  if (bUseCrossCPrediction)
  {
    CrossComponentPrediction::crossComponentPrediction(tu, compID, cs.getResiBuf(tu.Y()), piResi, piResi, true);
    if( jointCbCr )
    {
      CrossComponentPrediction::crossComponentPrediction(tu, COMPONENT_Cr, cs.getResiBuf(tu.Y()), crResi, crResi, true);
    }
  }

#if JVET_P1006_PICTURE_HEADER
  if (slice.getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
#else
  if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
#endif
  {
    CompArea      tmpArea(COMPONENT_Y, area.chromaFormat, Position(0,0), area.size());
    PelBuf tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
    tmpPred.copyFrom(piPred);
    piReco.reconstruct(tmpPred, piResi, cs.slice->clpRng(compID));
  }
  else
  {
    piReco.reconstruct(piPred, piResi, cs.slice->clpRng( compID ));
    if( jointCbCr )
    {
      crReco.reconstruct(crPred, crResi, cs.slice->clpRng( COMPONENT_Cr ));
    }
  }


  //===== update distortion =====
#if WCG_EXT
  if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
#if JVET_P1006_PICTURE_HEADER
    && slice.getPicHeader()->getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
#else
    && slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
#endif
  {
    const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
    if (compID == COMPONENT_Y  && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
    {
      CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
      PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
      tmpRecLuma.copyFrom(piReco);
      tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
      ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
    }
    else
    {
      ruiDist += m_pcRdCost->getDistPart(piOrg, piReco, bitDepth, compID, DF_SSE_WTD, &orgLuma);
      if( jointCbCr )
      {
        ruiDist += m_pcRdCost->getDistPart(crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
      }
    }
  }
  else
#endif
  {
    ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE );
    if( jointCbCr )
    {
      ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE );
    }
  }
}

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
void IntraSearch::xIntraCodingACTTUBlock(TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, std::vector<TrMode>* trModes, const bool loadTr)
{
  if (!tu.blocks[compID].valid())
  {
    CHECK(1, "tu does not exist");
  }

  CodingStructure     &cs = *tu.cs;
  const SPS           &sps = *cs.sps;
  const Slice         &slice = *cs.slice;
  const CompArea      &area = tu.blocks[compID];
  const CompArea &crArea = tu.blocks[COMPONENT_Cr];

  PelBuf              piOrgResi = cs.getOrgResiBuf(area);
  PelBuf              piResi = cs.getResiBuf(area);
  PelBuf              crOrgResi = cs.getOrgResiBuf(crArea);
  PelBuf              crResi = cs.getResiBuf(crArea);
  TCoeff              uiAbsSum = 0;

  CHECK(tu.jointCbCr && compID == COMPONENT_Cr, "wrong combination of compID and jointCbCr");
  bool jointCbCr = tu.jointCbCr && compID == COMPONENT_Cb;

  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());

  m_pcTrQuant->lambdaAdjustColorTrans(true);

  if (jointCbCr)
  {
    ComponentID compIdCode = (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr);
    m_pcTrQuant->selectLambda(compIdCode);
  }
  else
  {
    m_pcTrQuant->selectLambda(compID);
  }

#if JVET_P1006_PICTURE_HEADER
  bool flag = slice.getPicHeader()->getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag())) && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
  if (flag && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
#else
  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag())) && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
  if (flag && isChroma(compID) && slice.getLmcsChromaResidualScaleFlag())
#endif
  {
    int    cResScaleInv = tu.getChromaAdj();
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)cResScaleInv;
    m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cResScale*cResScale));
  }

  if (jointCbCr)
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
    const int    absIct = abs(TU::getICTMode(tu));
    const double lfact = (absIct == 1 || absIct == 3 ? 0.8 : 0.5);
    m_pcTrQuant->setLambda(lfact * m_pcTrQuant->getLambda());
  }
  if (sps.getJointCbCrEnabledFlag() && isChroma(compID) && (slice.getSliceQp() > 18))
  {
    m_pcTrQuant->setLambda(1.3 * m_pcTrQuant->getLambda());
  }

  if (isLuma(compID))
  {
    QpParam cQP(tu, compID);
    for (int qpIdx = 0; qpIdx < 2; qpIdx++)
    {
      cQP.Qps[qpIdx] = cQP.Qps[qpIdx] + (compID == COMPONENT_Cr ? DELTA_QP_FOR_Co : DELTA_QP_FOR_Y_Cg);
      cQP.pers[qpIdx] = cQP.Qps[qpIdx] / 6;
      cQP.rems[qpIdx] = cQP.Qps[qpIdx] % 6;
    }

    if (trModes)
    {
#if JVET_P0273_MTSIntraMaxCand
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, m_pcEncCfg->getMTSIntraMaxCand());
#else
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, CU::isIntra(*tu.cu) ? m_pcEncCfg->getIntraMTSMaxCand() : m_pcEncCfg->getInterMTSMaxCand());
#endif
#if JVET_P0058_CHROMA_TS
      tu.mtsIdx[compID] = trModes->at(0).first;
#else
      tu.mtsIdx = trModes->at(0).first;
#endif
    }
    m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);

    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
    }
    else
    {
      piResi.fill(0);
    }
  }
  else
  {
    int         codedCbfMask = 0;
    ComponentID codeCompId = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr) : compID);
    QpParam qpCbCr(tu, codeCompId);
    for (int qpIdx = 0; qpIdx < 2; qpIdx++)
    {
      qpCbCr.Qps[qpIdx] = qpCbCr.Qps[qpIdx] + (codeCompId == COMPONENT_Cr ? DELTA_QP_FOR_Co : DELTA_QP_FOR_Y_Cg);
      qpCbCr.pers[qpIdx] = qpCbCr.Qps[qpIdx] / 6;
      qpCbCr.rems[qpIdx] = qpCbCr.Qps[qpIdx] % 6;
    }

    if (tu.jointCbCr)
    {
      ComponentID otherCompId = (codeCompId == COMPONENT_Cr ? COMPONENT_Cb : COMPONENT_Cr);
      tu.getCoeffs(otherCompId).fill(0);
      TU::setCbfAtDepth(tu, otherCompId, tu.depth, false);
    }

    PelBuf& codeResi = (codeCompId == COMPONENT_Cr ? crResi : piResi);
    uiAbsSum = 0;
    m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx());
    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
      codedCbfMask += (codeCompId == COMPONENT_Cb ? 2 : 1);
    }
    else
    {
      codeResi.fill(0);
    }

    if (tu.jointCbCr)
    {
      if (tu.jointCbCr == 3 && codedCbfMask == 2)
      {
        codedCbfMask = 3;
        TU::setCbfAtDepth(tu, COMPONENT_Cr, tu.depth, true);
      }
      if (tu.jointCbCr != codedCbfMask)
      {
        ruiDist = std::numeric_limits<Distortion>::max();
        m_pcTrQuant->lambdaAdjustColorTrans(false);
        return;
      }
      m_pcTrQuant->invTransformICT(tu, piResi, crResi);
      uiAbsSum = codedCbfMask;
    }
  }

#if JVET_P1006_PICTURE_HEADER
  if (flag && uiAbsSum > 0 && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
#else
  if (flag && uiAbsSum > 0 && isChroma(compID) && slice.getLmcsChromaResidualScaleFlag())
#endif
  {
    piResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(compID));
    if (jointCbCr)
    {
      crResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(COMPONENT_Cr));
    }
  }

  m_pcTrQuant->lambdaAdjustColorTrans(false);

  ruiDist += m_pcRdCost->getDistPart(piOrgResi, piResi, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
  if (jointCbCr)
  {
    ruiDist += m_pcRdCost->getDistPart(crOrgResi, crResi, sps.getBitDepth(toChannelType(COMPONENT_Cr)), COMPONENT_Cr, DF_SSE);
  }
}
#endif

bool IntraSearch::xIntraCodingLumaISP(CodingStructure& cs, Partitioner& partitioner, const double bestCostSoFar)
{
  int               subTuCounter = 0;
  const CodingUnit& cu = *cs.getCU(partitioner.currArea().lumaPos(), partitioner.chType);
  bool              earlySkipISP = false;
  bool              splitCbfLuma = false;
  const PartSplit   ispType = CU::getISPType(cu, COMPONENT_Y);

  cs.cost = 0;

  partitioner.splitCurrArea(ispType, cs);

#if JVET_P1026_ISP_LFNST_COMBINATION
  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
#endif

  do   // subpartitions loop
  {
    uint32_t   numSig = 0;
    Distortion singleDistTmpLuma = 0;
    uint64_t   singleTmpFracBits = 0;
    double     singleCostTmp = 0;

    TransformUnit& tu = cs.addTU(CS::getArea(cs, partitioner.currArea(), partitioner.chType), partitioner.chType);
    tu.depth = partitioner.currTrDepth;

    // Encode TU
    xIntraCodingTUBlock(tu, COMPONENT_Y, false, singleDistTmpLuma, 0, &numSig);

    if (singleDistTmpLuma == MAX_INT)   // all zero CBF skip
    {
      earlySkipISP = true;
      partitioner.exitCurrSplit();
      cs.cost = MAX_DOUBLE;
      return false;
    }

    {
      if (m_pcRdCost->calcRdCost(cs.fracBits, cs.dist + singleDistTmpLuma) > bestCostSoFar)
      {
        // The accumulated cost + distortion is already larger than the best cost so far, so it is not necessary to calculate the rate
        earlySkipISP = true;
      }
      else
      {
#if JVET_P1026_ISP_LFNST_COMBINATION
        singleTmpFracBits = xGetIntraFracBitsQT(cs, partitioner, true, false, subTuCounter, ispType, &cuCtx);
#else
        singleTmpFracBits = xGetIntraFracBitsQT(cs, partitioner, true, false, subTuCounter, ispType);
#endif
      }
      singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
    }

    cs.cost += singleCostTmp;
    cs.dist += singleDistTmpLuma;
    cs.fracBits += singleTmpFracBits;

    subTuCounter++;

    splitCbfLuma |= TU::getCbfAtDepth(*cs.getTU(partitioner.currArea().lumaPos(), partitioner.chType, subTuCounter - 1), COMPONENT_Y, partitioner.currTrDepth);
#if JVET_P1026_ISP_LFNST_COMBINATION
    int nSubPartitions = m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1];
#else
    int nSubPartitions = m_ispTestedModes.numTotalParts[cu.ispMode - 1];
#endif
    if (subTuCounter < nSubPartitions)
    {
      // exit condition if the accumulated cost is already larger than the best cost so far (no impact in RD performance)
      if (cs.cost > bestCostSoFar)
      {
        earlySkipISP = true;
        break;
      }
      else if (subTuCounter < nSubPartitions)
      {
        // more restrictive exit condition
        double threshold = nSubPartitions == 2 ? 0.95 : subTuCounter == 1 ? 0.83 : 0.91;
        if (subTuCounter < nSubPartitions && cs.cost > bestCostSoFar * threshold)
        {
          earlySkipISP = true;
          break;
        }
      }
    }
  } while (partitioner.nextPart(cs));   // subpartitions loop

  partitioner.exitCurrSplit();
  const UnitArea& currArea = partitioner.currArea();
  const uint32_t  currDepth = partitioner.currTrDepth;

  if (earlySkipISP)
  {
    cs.cost = MAX_DOUBLE;
  }
  else
  {
    cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
    // The cost check is necessary here again to avoid superfluous operations if the maximum number of coded subpartitions was reached and yet ISP did not win
    if (cs.cost < bestCostSoFar)
    {
      cs.setDecomp(cu.Y());
      cs.picture->getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));

      for (auto& ptu : cs.tus)
      {
        if (currArea.Y().contains(ptu->Y()))
        {
          TU::setCbfAtDepth(*ptu, COMPONENT_Y, currDepth, splitCbfLuma ? 1 : 0);
        }
      }
    }
    else
    {
#if !JVET_P1026_ISP_LFNST_COMBINATION
      cs.cost = MAX_DOUBLE;
#endif
      earlySkipISP = true;
    }
  }
  return !earlySkipISP;
}


bool IntraSearch::xRecurIntraCodingLumaQT( CodingStructure &cs, Partitioner &partitioner, const double bestCostSoFar, const int subTuIdx, const PartSplit ispType, const bool ispIsCurrentWinner, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst )
{
        int   subTuCounter = subTuIdx;
  const UnitArea &currArea = partitioner.currArea();
  const CodingUnit     &cu = *cs.getCU( currArea.lumaPos(), partitioner.chType );
        bool  earlySkipISP = false;
  uint32_t currDepth       = partitioner.currTrDepth;
  const SPS &sps           = *cs.sps;
  const PPS &pps           = *cs.pps;
  const bool keepResi      = pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() || KEEP_PRED_AND_RESI_SIGNALS;
  bool bCheckFull          = true;
  bool bCheckSplit         = false;
  bCheckFull               = !partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
  bCheckSplit              = partitioner.canSplit( TU_MAX_TR_SPLIT, cs );

  if( cu.ispMode )
  {
    bCheckSplit = partitioner.canSplit( ispType, cs );
    bCheckFull = !bCheckSplit;
  }
  uint32_t    numSig           = 0;

  double     dSingleCost                        = MAX_DOUBLE;
  Distortion uiSingleDistLuma                   = 0;
  uint64_t   singleFracBits                     = 0;
  bool       checkTransformSkip                 = sps.getTransformSkipEnabledFlag();
  int        bestModeId[ MAX_NUM_COMPONENT ]    = { 0, 0, 0 };
  uint8_t    nNumTransformCands                 = cu.mtsFlag ? 4 : 1;
  uint8_t    numTransformIndexCands             = nNumTransformCands;

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;

#if JVET_P1026_ISP_LFNST_COMBINATION
  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
#endif

  if( bCheckSplit )
  {
    csSplit = &cs;
  }
  else if( bCheckFull )
  {
    csFull = &cs;
  }

  bool validReturnFull = false;

  if( bCheckFull )
  {
    csFull->cost = 0.0;

    TransformUnit &tu = csFull->addTU( CS::getArea( *csFull, currArea, partitioner.chType ), partitioner.chType );
    tu.depth = currDepth;

    const bool tsAllowed  = TU::isTSAllowed( tu, COMPONENT_Y );
#if JVET_P1026_MTS_SIGNALLING
    const bool mtsAllowed = CU::isMTSAllowed( cu, COMPONENT_Y );
#else
    const bool mtsAllowed = TU::isMTSAllowed( tu, COMPONENT_Y );
#endif
    std::vector<TrMode> trModes;

    if( sps.getUseLFNST() )
    {
      checkTransformSkip &= tsAllowed;
      checkTransformSkip &= !cu.mtsFlag;
      checkTransformSkip &= !cu.lfnstIdx;

      if( !cu.mtsFlag && checkTransformSkip )
      {
        trModes.push_back( TrMode( 0, true ) ); //DCT2
        trModes.push_back( TrMode( 1, true ) ); //TS
      }
    }
    else
    {
      nNumTransformCands = 1 + ( tsAllowed ? 1 : 0 ) + ( mtsAllowed ? 4 : 0 ); // DCT + TS + 4 MTS = 6 tests

      trModes.push_back( TrMode( 0, true ) ); //DCT2
      if( tsAllowed )
      {
        trModes.push_back( TrMode( 1, true ) );
      }
      if( mtsAllowed )
      {
        for( int i = 2; i < 6; i++ )
        {
          trModes.push_back( TrMode( i, true ) );
        }
      }
    }

    CHECK( !tu.Y().valid(), "Invalid TU" );

    CodingStructure &saveCS = *m_pSaveCS[0];

    TransformUnit *tmpTU = nullptr;

    Distortion singleDistTmpLuma = 0;
    uint64_t     singleTmpFracBits = 0;
    double     singleCostTmp     = 0;
    int        firstCheckId      = ( sps.getUseLFNST() && mtsCheckRangeFlag && cu.mtsFlag ) ? mtsFirstCheckId : 0;

    //we add the MTS candidates to the loop. TransformSkip will still be the last one to be checked (when modeId == lastCheckId) as long as checkTransformSkip is true
    int        lastCheckId       = sps.getUseLFNST() ? ( ( mtsCheckRangeFlag && cu.mtsFlag ) ? ( mtsLastCheckId + ( int ) checkTransformSkip ) : ( numTransformIndexCands - ( firstCheckId + 1 ) + ( int ) checkTransformSkip ) ) :
                                   trModes[ nNumTransformCands - 1 ].first;
    bool isNotOnlyOneMode        = sps.getUseLFNST() ? lastCheckId != firstCheckId : nNumTransformCands != 1;

    if( isNotOnlyOneMode )
    {
      saveCS.pcv     = cs.pcv;
      saveCS.picture = cs.picture;
      saveCS.area.repositionTo(cs.area);
      saveCS.clearTUs();
      tmpTU = &saveCS.addTU(currArea, partitioner.chType);
    }

    bool    cbfBestMode      = false;
    bool    cbfBestModeValid = false;
    bool    cbfDCT2  = true;

    double bestDCT2cost = MAX_DOUBLE;
    double threshold = m_pcEncCfg->getUseFastISP() && !cu.ispMode && ispIsCurrentWinner && nNumTransformCands > 1 ? 1 + 1.4 / sqrt( cu.lwidth() * cu.lheight() ) : 1;
    for( int modeId = firstCheckId; modeId <= ( sps.getUseLFNST() ? lastCheckId : ( nNumTransformCands - 1 ) ); modeId++ )
    {
      uint8_t transformIndex = modeId;

      if( sps.getUseLFNST() )
      {
        if( ( transformIndex < lastCheckId ) || ( ( transformIndex == lastCheckId ) && !checkTransformSkip ) ) //we avoid this if the mode is transformSkip
        {
          // Skip checking other transform candidates if zero CBF is encountered and it is the best transform so far
          if( m_pcEncCfg->getUseFastLFNST() && transformIndex && !cbfBestMode && cbfBestModeValid )
          {
            continue;
          }
        }
      }
      else
      {
#if JVET_AHG14_LOSSLESS
        if( !( m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING ) )
        {
#endif
#if JVET_P0058_CHROMA_TS
        if( !cbfDCT2 || ( m_pcEncCfg->getUseTransformSkipFast() && bestModeId[ COMPONENT_Y ] == MTS_SKIP))
#else
        if( !cbfDCT2 || ( m_pcEncCfg->getUseTransformSkipFast() && bestModeId[ COMPONENT_Y ] == 1 ) )
#endif
        {
          break;
        }
        if( !trModes[ modeId ].second )
        {
          continue;
        }
        //we compare the DCT-II cost against the best ISP cost so far (except for TS)
#if JVET_P0058_CHROMA_TS
        if (m_pcEncCfg->getUseFastISP() && !cu.ispMode && ispIsCurrentWinner && trModes[modeId].first != MTS_DCT2_DCT2 && (trModes[modeId].first != MTS_SKIP || !tsAllowed) && bestDCT2cost > bestCostSoFar * threshold)
#else
        if( m_pcEncCfg->getUseFastISP() && !cu.ispMode && ispIsCurrentWinner && trModes[ modeId ].first != 0 && ( trModes[ modeId ].first != 1 || !tsAllowed ) && bestDCT2cost > bestCostSoFar * threshold )
#endif
        {
          continue;
        }
#if JVET_AHG14_LOSSLESS
        }
#endif
#if JVET_P0058_CHROMA_TS
        tu.mtsIdx[COMPONENT_Y] = trModes[modeId].first;
#else
        tu.mtsIdx = trModes[ modeId ].first;
#endif
      }


      if ((modeId != firstCheckId) && isNotOnlyOneMode)
      {
        m_CABACEstimator->getCtx() = ctxStart;
      }

      int default0Save1Load2 = 0;
      singleDistTmpLuma = 0;

      if( modeId == firstCheckId && ( sps.getUseLFNST() ? ( modeId != lastCheckId ) : ( nNumTransformCands > 1 ) ) )
      {
        default0Save1Load2 = 1;
      }
      else if (modeId != firstCheckId)
      {
        if( sps.getUseLFNST() && !cbfBestModeValid )
        {
          default0Save1Load2 = 1;
        }
        else
        {
          default0Save1Load2 = 2;
        }
      }
      if( cu.ispMode )
      {
        default0Save1Load2 = 0;
      }
      if( sps.getUseLFNST() )
      {
        if( cu.mtsFlag )
        {
          if( moreProbMTSIdxFirst )
          {
            const ChannelType     chType      = toChannelType( COMPONENT_Y );
            const CompArea&       area        = tu.blocks[ COMPONENT_Y ];
            const PredictionUnit& pu          = *cs.getPU( area.pos(), chType );
            uint32_t              uiIntraMode = pu.intraDir[ chType ];

            if( transformIndex == 1 )
            {
#if JVET_P0058_CHROMA_TS
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
#else
              tu.mtsIdx = ( uiIntraMode < 34 ) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
#endif
            }
            else if( transformIndex == 2 )
            {
#if JVET_P0058_CHROMA_TS
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
#else
              tu.mtsIdx = ( uiIntraMode < 34 ) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
#endif
            }
            else
            {
#if JVET_P0058_CHROMA_TS
              tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
#else
              tu.mtsIdx = MTS_DST7_DST7 + transformIndex;
#endif
            }
          }
          else
          {
#if JVET_P0058_CHROMA_TS
            tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
#else
            tu.mtsIdx = MTS_DST7_DST7 + transformIndex;
#endif
          }
        }
        else
        {
#if JVET_P0058_CHROMA_TS
          tu.mtsIdx[COMPONENT_Y] = transformIndex;
#else
          tu.mtsIdx = transformIndex;
#endif
        }

        if( !cu.mtsFlag && checkTransformSkip )
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, false, singleDistTmpLuma, default0Save1Load2, &numSig, modeId == 0 ? &trModes : nullptr, true );
          if( modeId == 0 )
          {
            for( int i = 0; i < 2; i++ )
            {
              if( trModes[ i ].second )
              {
                lastCheckId = trModes[ i ].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, false, singleDistTmpLuma, default0Save1Load2, &numSig );
        }
      }
      else
      {
        if( nNumTransformCands > 1 )
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, false, singleDistTmpLuma, default0Save1Load2, &numSig, modeId == 0 ? &trModes : nullptr, true );
          if( modeId == 0 )
          {
            for( int i = 0; i < nNumTransformCands; i++ )
            {
              if( trModes[ i ].second )
              {
                lastCheckId = trModes[ i ].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, false, singleDistTmpLuma, default0Save1Load2, &numSig );
        }
      }

      //----- determine rate and r-d cost -----
      if( ( sps.getUseLFNST() ? ( modeId == lastCheckId && modeId != 0 && checkTransformSkip ) : ( trModes[ modeId ].first != 0 ) ) && !TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth ) )
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        singleCostTmp = MAX_DOUBLE;
      }
      else
      {
        if( cu.ispMode && m_pcRdCost->calcRdCost( csFull->fracBits, csFull->dist + singleDistTmpLuma ) > bestCostSoFar )
        {
          earlySkipISP = true;
        }
        else
        {
#if JVET_P1026_ISP_LFNST_COMBINATION
          singleTmpFracBits = xGetIntraFracBitsQT( *csFull, partitioner, true, false, subTuCounter, ispType, &cuCtx );
#else
          singleTmpFracBits = xGetIntraFracBitsQT( *csFull, partitioner, true, false, subTuCounter, ispType );
#endif
        }
        singleCostTmp     = m_pcRdCost->calcRdCost( singleTmpFracBits, singleDistTmpLuma );
      }

      if ( !cu.ispMode && nNumTransformCands > 1 && modeId == firstCheckId )
      {
        bestDCT2cost = singleCostTmp;
      }

      if (singleCostTmp < dSingleCost)
      {
        dSingleCost       = singleCostTmp;
        uiSingleDistLuma  = singleDistTmpLuma;
        singleFracBits    = singleTmpFracBits;

        if( sps.getUseLFNST() )
        {
          bestModeId[ COMPONENT_Y ] = modeId;
          cbfBestMode = TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth );
          cbfBestModeValid = true;
          validReturnFull = true;
        }
        else
        {
          bestModeId[ COMPONENT_Y ] = trModes[ modeId ].first;
          if( trModes[ modeId ].first == 0 )
          {
            cbfDCT2 = TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth );
          }
        }

        if( bestModeId[COMPONENT_Y] != lastCheckId )
        {
          saveCS.getPredBuf( tu.Y() ).copyFrom( csFull->getPredBuf( tu.Y() ) );
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

    if( sps.getUseLFNST() && !validReturnFull )
    {
      csFull->cost = MAX_DOUBLE;

      if( bCheckSplit )
      {
        ctxBest = m_CABACEstimator->getCtx();
      }
    }
    else
    {
      if( bestModeId[COMPONENT_Y] != lastCheckId )
      {
        csFull->getPredBuf( tu.Y() ).copyFrom( saveCS.getPredBuf( tu.Y() ) );
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
  }

  bool validReturnSplit = false;
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
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }

    if( cu.ispMode )
    {
      partitioner.splitCurrArea( ispType, *csSplit );
    }
    do
    {
      bool tmpValidReturnSplit = xRecurIntraCodingLumaQT( *csSplit, partitioner, bestCostSoFar, subTuCounter, ispType, false, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
      if( sps.getUseLFNST() && !tmpValidReturnSplit )
      {
        splitIsSelected = false;
        break;
      }

      if( !cu.ispMode )
      {
        csSplit->setDecomp( partitioner.currArea().Y() );
      }
      else if( CU::isISPFirst( cu, partitioner.currArea().Y(), COMPONENT_Y ) )
      {
        csSplit->setDecomp( cu.Y() );
      }

      uiSplitCbfLuma |= TU::getCbfAtDepth( *csSplit->getTU( partitioner.currArea().lumaPos(), partitioner.chType, subTuCounter - 1 ), COMPONENT_Y, partitioner.currTrDepth );
      if( cu.ispMode )
      {
        //exit condition if the accumulated cost is already larger than the best cost so far (no impact in RD performance)
        if( csSplit->cost > bestCostSoFar )
        {
          earlySkipISP    = true;
          splitIsSelected = false;
          break;
        }
        else
        {
          //more restrictive exit condition
          bool tuIsDividedInRows = CU::divideTuInRows( cu );
          int nSubPartitions = tuIsDividedInRows ? cu.lheight() >> floorLog2(cu.firstTU->lheight()) : cu.lwidth() >> floorLog2(cu.firstTU->lwidth());
          double threshold = nSubPartitions == 2 ? 0.95 : subTuCounter == 1 ? 0.83 : 0.91;
          if( subTuCounter < nSubPartitions && csSplit->cost > bestCostSoFar*threshold )
          {
            earlySkipISP    = true;
            splitIsSelected = false;
            break;
          }
        }
      }



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

#if JVET_P1026_ISP_LFNST_COMBINATION
      cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] = false;
      cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
      cuCtx.lfnstLastScanPos = false;
#if JVET_P1026_MTS_SIGNALLING
      cuCtx.violatesMtsCoeffConstraint = false;
#endif
#endif

      //----- determine rate and r-d cost -----
#if JVET_P1026_ISP_LFNST_COMBINATION
      csSplit->fracBits = xGetIntraFracBitsQT( *csSplit, partitioner, true, false, cu.ispMode ? 0 : -1, ispType, &cuCtx );
#else
      csSplit->fracBits = xGetIntraFracBitsQT( *csSplit, partitioner, true, false, cu.ispMode ? 0 : -1, ispType );
#endif

      //--- update cost ---
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      validReturnSplit = true;
    }
  }

  bool retVal = false;
  if( csFull || csSplit )
  {
    if( !sps.getUseLFNST() || validReturnFull || validReturnSplit )
    {
      {
        // otherwise this would've happened in useSubStructure
        cs.picture->getRecoBuf( currArea.Y() ).copyFrom( cs.getRecoBuf( currArea.Y() ) );
        cs.picture->getPredBuf( currArea.Y() ).copyFrom( cs.getPredBuf( currArea.Y() ) );
      }

      if( cu.ispMode && earlySkipISP )
      {
        cs.cost = MAX_DOUBLE;
      }
      else
      {
        cs.cost = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );
        retVal = true;
      }
    }
  }
  return retVal;
}

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM
bool IntraSearch::xRecurIntraCodingACTQT(CodingStructure &cs, Partitioner &partitioner, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst)
{
  const UnitArea &currArea = partitioner.currArea();
  uint32_t       currDepth = partitioner.currTrDepth;
  const Slice    &slice = *cs.slice;
  const SPS      &sps = *cs.sps;

  bool bCheckFull = !partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  bool bCheckSplit = !bCheckFull;

  TempCtx ctxStart(m_CtxCache, m_CABACEstimator->getCtx());
  TempCtx ctxBest(m_CtxCache);

  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull = nullptr;
  if (bCheckSplit)
  {
    csSplit = &cs;
  }
  else if (bCheckFull)
  {
    csFull = &cs;
  }

  bool validReturnFull = false;

  if (bCheckFull)
  {
    TransformUnit        &tu = csFull->addTU(CS::getArea(*csFull, currArea, partitioner.chType), partitioner.chType);
    tu.depth = currDepth;
    const CodingUnit     &cu = *csFull->getCU(tu.Y().pos(), CHANNEL_TYPE_LUMA);
    const PredictionUnit &pu = *csFull->getPU(tu.Y().pos(), CHANNEL_TYPE_LUMA);
    CHECK(!tu.Y().valid() || !tu.Cb().valid() || !tu.Cr().valid(), "Invalid TU");
    CHECK(tu.cu != &cu, "wrong CU fetch");
    CHECK(cu.ispMode, "adaptive color transform cannot be applied to ISP");
    CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM mode for adaptive color transform");

    // 1. intra prediction and forward color transform

    PelUnitBuf orgBuf = csFull->getOrgBuf(tu);
    PelUnitBuf predBuf = csFull->getPredBuf(tu);
    PelUnitBuf resiBuf = csFull->getResiBuf(tu);
    PelUnitBuf orgResiBuf = csFull->getOrgResiBuf(tu);

    for (int i = 0; i < getNumberValidComponents(tu.chromaFormat); i++)
    {
      ComponentID          compID = (ComponentID)i;
      const CompArea       &area = tu.blocks[compID];
      const ChannelType    chType = toChannelType(compID);

      PelBuf         piOrg = orgBuf.bufs[compID];
      PelBuf         piPred = predBuf.bufs[compID];
      PelBuf         piResi = resiBuf.bufs[compID];

      initIntraPatternChType(*tu.cu, area);
      if (PU::isMIP(pu, chType))
      {
#if JVET_P0803_COMBINED_MIP_CLEANUP
        initIntraMip(pu, area);
#endif
        predIntraMip(compID, piPred, pu);
      }
      else
      {
        predIntraAng(compID, piPred, pu);
      }

      piResi.copyFrom(piOrg);
#if JVET_P1006_PICTURE_HEADER
      if (slice.getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
#else
      if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
#endif
      {
        CompArea tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
        PelBuf   tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
        tmpPred.copyFrom(piPred);
        piResi.rspSignal(m_pcReshape->getFwdLUT());
        piResi.subtract(tmpPred);
      }
      else
        piResi.subtract(piPred);
    }

    resiBuf.colorSpaceConvert(orgResiBuf, true);

    // 2. luma residual optimization 
    double     dSingleCostLuma = MAX_DOUBLE;
    bool       checkTransformSkip = sps.getTransformSkipEnabledFlag();
    int        bestLumaModeId = 0;
    uint8_t    nNumTransformCands = cu.mtsFlag ? 4 : 1;
    uint8_t    numTransformIndexCands = nNumTransformCands;

    const bool tsAllowed = TU::isTSAllowed(tu, COMPONENT_Y);
#if JVET_P1026_MTS_SIGNALLING
    const bool mtsAllowed = CU::isMTSAllowed(cu, COMPONENT_Y);
#else
    const bool mtsAllowed = TU::isMTSAllowed(tu, COMPONENT_Y);
#endif
    std::vector<TrMode> trModes;

    if (sps.getUseLFNST())
    {
      checkTransformSkip &= tsAllowed;
      checkTransformSkip &= !cu.mtsFlag;
      checkTransformSkip &= !cu.lfnstIdx;

      if (!cu.mtsFlag && checkTransformSkip)
      {
        trModes.push_back(TrMode(0, true)); //DCT2
        trModes.push_back(TrMode(1, true)); //TS
      }
    }
    else
    {
      nNumTransformCands = 1 + (tsAllowed ? 1 : 0) + (mtsAllowed ? 4 : 0); // DCT + TS + 4 MTS = 6 tests

      trModes.push_back(TrMode(0, true)); //DCT2
      if (tsAllowed)
      {
        trModes.push_back(TrMode(1, true));
      }
      if (mtsAllowed)
      {
        for (int i = 2; i < 6; i++)
        {
          trModes.push_back(TrMode(i, true));
        }
      }
    }

    CodingStructure &saveLumaCS = *m_pSaveCS[0];
    TransformUnit   *tmpTU = nullptr;
    Distortion      singleDistTmpLuma = 0;
    uint64_t        singleTmpFracBits = 0;
    double          singleCostTmp = 0;
    int             firstCheckId = (sps.getUseLFNST() && mtsCheckRangeFlag && cu.mtsFlag) ? mtsFirstCheckId : 0;
    int             lastCheckId = sps.getUseLFNST() ? ((mtsCheckRangeFlag && cu.mtsFlag) ? (mtsLastCheckId + (int)checkTransformSkip) : (numTransformIndexCands - (firstCheckId + 1) + (int)checkTransformSkip)) : trModes[nNumTransformCands - 1].first;
    bool            isNotOnlyOneMode = sps.getUseLFNST() ? lastCheckId != firstCheckId : nNumTransformCands != 1;

    if (isNotOnlyOneMode)
    {
      saveLumaCS.pcv = csFull->pcv;
      saveLumaCS.picture = csFull->picture;
      saveLumaCS.area.repositionTo(csFull->area);
      saveLumaCS.clearTUs();
      tmpTU = &saveLumaCS.addTU(currArea, partitioner.chType);
    }

    bool    cbfBestMode = false;
    bool    cbfBestModeValid = false;
    bool    cbfDCT2 = true;

    m_pcRdCost->lambdaAdjustColorTrans(true, COMPONENT_Y);

    for (int modeId = firstCheckId; modeId <= lastCheckId; modeId++)
    {
      uint8_t transformIndex = modeId;
      csFull->getResiBuf(tu.Y()).copyFrom(csFull->getOrgResiBuf(tu.Y()));

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      if (sps.getUseLFNST())
      {
        if ((transformIndex < lastCheckId) || ((transformIndex == lastCheckId) && !checkTransformSkip)) //we avoid this if the mode is transformSkip
        {
          // Skip checking other transform candidates if zero CBF is encountered and it is the best transform so far
          if (m_pcEncCfg->getUseFastLFNST() && transformIndex && !cbfBestMode && cbfBestModeValid)
          {
            continue;
          }
        }
      }
      else
      {
#if JVET_AHG14_LOSSLESS
        if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING))
        {
#endif
        if (!cbfDCT2 || (m_pcEncCfg->getUseTransformSkipFast() && bestLumaModeId == 1))
        {
          break;
        }
        if (!trModes[modeId].second)
        {
          continue;
        }
#if JVET_AHG14_LOSSLESS
        }
#endif
#if JVET_P0058_CHROMA_TS
        tu.mtsIdx[COMPONENT_Y] = trModes[modeId].first;
#else
        tu.mtsIdx = trModes[modeId].first;
#endif
      }

      singleDistTmpLuma = 0;
      if (sps.getUseLFNST())
      {
        if (cu.mtsFlag)
        {
          if (moreProbMTSIdxFirst)
          {
            uint32_t uiIntraMode = pu.intraDir[CHANNEL_TYPE_LUMA];

            if (transformIndex == 1)
            {
#if JVET_P0058_CHROMA_TS
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
#else
              tu.mtsIdx = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
#endif
            }
            else if (transformIndex == 2)
            {
#if JVET_P0058_CHROMA_TS
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
#else
              tu.mtsIdx = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
#endif
            }
            else
            {
#if JVET_P0058_CHROMA_TS
              tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
#else
              tu.mtsIdx = MTS_DST7_DST7 + transformIndex;
#endif
            }
          }
          else
          {
#if JVET_P0058_CHROMA_TS
            tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
#else
            tu.mtsIdx = MTS_DST7_DST7 + transformIndex;
#endif
          }
        }
        else
        {
#if JVET_P0058_CHROMA_TS
          tu.mtsIdx[COMPONENT_Y] = transformIndex;
#else
          tu.mtsIdx = transformIndex;
#endif
        }

        if (!cu.mtsFlag && checkTransformSkip)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, modeId == 0 ? &trModes : nullptr, true);
          if (modeId == 0)
          {
            for (int i = 0; i < 2; i++)
            {
              if (trModes[i].second)
              {
                lastCheckId = trModes[i].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma);
        }
      }
      else
      {
        if (nNumTransformCands > 1)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, modeId == 0 ? &trModes : nullptr, true);
          if (modeId == 0)
          {
            for (int i = 0; i < nNumTransformCands; i++)
            {
              if (trModes[i].second)
              {
                lastCheckId = trModes[i].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma);
        }
      }

      //----- determine rate and r-d cost -----
      if ((sps.getUseLFNST() ? (modeId == lastCheckId && modeId != 0 && checkTransformSkip) : (trModes[modeId].first != 0)) && !TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth))
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        singleCostTmp = MAX_DOUBLE;
      }
      else
      {
        singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, -1, TU_NO_ISP);
        singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
      }

      if (singleCostTmp < dSingleCostLuma)
      {
        dSingleCostLuma = singleCostTmp;
        validReturnFull = true;

        if (sps.getUseLFNST())
        {
          bestLumaModeId = modeId;
          cbfBestMode = TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth);
          cbfBestModeValid = true;
        }
        else
        {
          bestLumaModeId = trModes[modeId].first;
          if (trModes[modeId].first == 0)
          {
            cbfDCT2 = TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth);
          }
        }

        if (bestLumaModeId != lastCheckId)
        {
          saveLumaCS.getResiBuf(tu.Y()).copyFrom(csFull->getResiBuf(tu.Y()));
          tmpTU->copyComponentFrom(tu, COMPONENT_Y);
          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }

    m_pcRdCost->lambdaAdjustColorTrans(false, COMPONENT_Y);

    if (sps.getUseLFNST())
    {
      if (!validReturnFull)
      {
        csFull->cost = MAX_DOUBLE;
        return false;
      }
    }
    else
    {
      CHECK(!validReturnFull, "no transform mode was tested for luma");
    }

    csFull->setDecomp(currArea.Y(), true);
    csFull->setDecomp(currArea.Cb(), true);

    if (bestLumaModeId != lastCheckId)
    {
      csFull->getResiBuf(tu.Y()).copyFrom(saveLumaCS.getResiBuf(tu.Y()));
      tu.copyComponentFrom(*tmpTU, COMPONENT_Y);
      m_CABACEstimator->getCtx() = ctxBest;
    }

    // 3 chroma residual optimization
    CodingStructure &saveChromaCS = *m_pSaveCS[1];
    saveChromaCS.pcv = csFull->pcv;
    saveChromaCS.picture = csFull->picture;
    saveChromaCS.area.repositionTo(csFull->area);
    saveChromaCS.initStructData(MAX_INT, true);
    tmpTU = &saveChromaCS.addTU(currArea, partitioner.chType);

    CompArea&  cbArea = tu.blocks[COMPONENT_Cb];
    CompArea&  crArea = tu.blocks[COMPONENT_Cr];

    ctxStart = m_CABACEstimator->getCtx();
    m_CABACEstimator->resetBits();
    tu.jointCbCr = 0;

#if JVET_P1006_PICTURE_HEADER
    bool doReshaping = (slice.getPicHeader()->getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (slice.isIntra() || m_pcReshape->getCTUFlag()) && (cbArea.width * cbArea.height > 4));
#else
    bool doReshaping = (slice.getLmcsEnabledFlag() && slice.getLmcsChromaResidualScaleFlag() && (slice.isIntra() || m_pcReshape->getCTUFlag()) && (cbArea.width * cbArea.height > 4));
#endif
    if (doReshaping)
    {
      const Area      area = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].size()));
      const CompArea &areaY = CompArea(COMPONENT_Y, tu.chromaFormat, area);
      int             adj = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
      tu.setChromaAdj(adj);
    }

    CompStorage  orgResiCb[5], orgResiCr[5]; // 0:std, 1-3:jointCbCr (placeholder at this stage), 4:crossComp
    orgResiCb[0].create(cbArea);
    orgResiCr[0].create(crArea);
    orgResiCb[0].copyFrom(csFull->getOrgResiBuf(cbArea));
    orgResiCr[0].copyFrom(csFull->getOrgResiBuf(crArea));
    if (doReshaping)
    {
      int cResScaleInv = tu.getChromaAdj();
      orgResiCb[0].scaleSignal(cResScaleInv, 1, slice.clpRng(COMPONENT_Cb));
      orgResiCr[0].scaleSignal(cResScaleInv, 1, slice.clpRng(COMPONENT_Cr));
    }

    // 3.1 regular chroma residual coding
    csFull->getResiBuf(cbArea).copyFrom(orgResiCb[0]);
    csFull->getResiBuf(crArea).copyFrom(orgResiCr[0]);

    for (uint32_t c = COMPONENT_Cb; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
    {
      const ComponentID compID = ComponentID(c);
      Distortion singleDistChroma = 0;
      xIntraCodingACTTUBlock(tu, compID, singleDistChroma);
      xGetIntraFracBitsQTChroma(tu, compID);
    }

    Position tuPos = tu.Y();
    tuPos.relativeTo(cu.Y());
    const UnitArea relativeUnitArea(tu.chromaFormat, Area(tuPos, tu.Y().size()));
    PelUnitBuf     invColorTransResidual = m_colorTransResiBuf.getBuf(relativeUnitArea);
    csFull->getResiBuf(tu).colorSpaceConvert(invColorTransResidual, false);

    Distortion totalDist = 0;
    for (uint32_t c = COMPONENT_Y; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
    {
      const ComponentID compID = ComponentID(c);
      const CompArea&   area = tu.blocks[compID];
      PelBuf            piOrg = csFull->getOrgBuf(area);
      PelBuf            piReco = csFull->getRecoBuf(area);
      PelBuf            piPred = csFull->getPredBuf(area);
      PelBuf            piResi = invColorTransResidual.bufs[compID];

      piReco.reconstruct(piPred, piResi, cs.slice->clpRng(compID));

      if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
#if JVET_P1006_PICTURE_HEADER
        & slice.getPicHeader()->getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
#else
        & slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
#endif
      {
        const CPelBuf orgLuma = csFull->getOrgBuf(csFull->area.blocks[COMPONENT_Y]);
        if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        {
          CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
          tmpRecLuma.copyFrom(piReco);
          tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
          totalDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
        {
          totalDist += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
      }
      else
      {
        totalDist += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
      }
    }

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t totalBits = xGetIntraFracBitsQT(*csFull, partitioner, true, true, -1, TU_NO_ISP);
    double   totalCost = m_pcRdCost->calcRdCost(totalBits, totalDist);

    saveChromaCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
    saveChromaCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
    saveChromaCS.getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));
    tmpTU->copyComponentFrom(tu, COMPONENT_Cb);
    tmpTU->copyComponentFrom(tu, COMPONENT_Cr);
    ctxBest = m_CABACEstimator->getCtx();

    // 3.2 jointCbCr
    double     bestCostJointCbCr = totalCost;
    Distortion bestDistJointCbCr = totalDist;
    uint64_t   bestBitsJointCbCr = totalBits;
    int        bestJointCbCr = tu.jointCbCr; assert(!bestJointCbCr);

    bool       lastIsBest = false;
    std::vector<int>  jointCbfMasksToTest;
    if (sps.getJointCbCrEnabledFlag() && (TU::getCbf(tu, COMPONENT_Cb) || TU::getCbf(tu, COMPONENT_Cr)))
    {
      jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(tu, orgResiCb, orgResiCr);
    }

    for (int cbfMask : jointCbfMasksToTest)
    {
      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      Distortion distTmp = 0;
      tu.jointCbCr = (uint8_t)cbfMask;

      csFull->getResiBuf(cbArea).copyFrom(orgResiCb[cbfMask]);
      csFull->getResiBuf(crArea).copyFrom(orgResiCr[cbfMask]);
      xIntraCodingACTTUBlock(tu, COMPONENT_Cb, distTmp);

      double   costTmp = std::numeric_limits<double>::max();
      uint64_t bitsTmp = 0;
      if (distTmp < std::numeric_limits<Distortion>::max())
      {
        csFull->getResiBuf(tu).colorSpaceConvert(invColorTransResidual, false);
        distTmp = 0;
        for (uint32_t c = COMPONENT_Y; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
        {
          const ComponentID compID = ComponentID(c);
          const CompArea&   area = tu.blocks[compID];
          PelBuf            piOrg = csFull->getOrgBuf(area);
          PelBuf            piReco = csFull->getRecoBuf(area);
          PelBuf            piPred = csFull->getPredBuf(area);
          PelBuf            piResi = invColorTransResidual.bufs[compID];

          piReco.reconstruct(piPred, piResi, cs.slice->clpRng(compID));
          if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
#if JVET_P1006_PICTURE_HEADER
            & slice.getPicHeader()->getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
#else
            & slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
#endif
          {
            const CPelBuf orgLuma = csFull->getOrgBuf(csFull->area.blocks[COMPONENT_Y]);
            if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
            {
              CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
              PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
              tmpRecLuma.copyFrom(piReco);
              tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
              distTmp += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
            }
            else
            {
              distTmp += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
            }
          }
          else
          {
            distTmp += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
          }
        }

        bitsTmp = xGetIntraFracBitsQT(*csFull, partitioner, true, true, -1, TU_NO_ISP);
        costTmp = m_pcRdCost->calcRdCost(bitsTmp, distTmp);
      }

      if (costTmp < bestCostJointCbCr)
      {
        bestCostJointCbCr = costTmp;
        bestDistJointCbCr = distTmp;
        bestBitsJointCbCr = bitsTmp;
        bestJointCbCr = tu.jointCbCr;
        lastIsBest = (cbfMask == jointCbfMasksToTest.back());

        // store data
        if (!lastIsBest)
        {
          saveChromaCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
          saveChromaCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
          saveChromaCS.getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));
          tmpTU->copyComponentFrom(tu, COMPONENT_Cb);
          tmpTU->copyComponentFrom(tu, COMPONENT_Cr);

          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }

    if (!lastIsBest)
    {
      csFull->getResiBuf(cbArea).copyFrom(saveChromaCS.getResiBuf(cbArea));
      csFull->getResiBuf(crArea).copyFrom(saveChromaCS.getResiBuf(crArea));
      csFull->getRecoBuf(tu).copyFrom(saveChromaCS.getRecoBuf(tu));
      tu.copyComponentFrom(*tmpTU, COMPONENT_Cb);
      tu.copyComponentFrom(*tmpTU, COMPONENT_Cr);

      m_CABACEstimator->getCtx() = ctxBest;
    }
    tu.jointCbCr = bestJointCbCr;
    csFull->picture->getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));

    csFull->dist += bestDistJointCbCr;
    csFull->fracBits += bestBitsJointCbCr;
    csFull->cost = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist);
  }

  bool validReturnSplit = false;
  if (bCheckSplit)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, *csSplit))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, *csSplit);
    }

    bool splitIsSelected = true;
    do
    {
      bool tmpValidReturnSplit = xRecurIntraCodingACTQT(*csSplit, partitioner, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
      if (sps.getUseLFNST())
      {
        if (!tmpValidReturnSplit)
        {
          splitIsSelected = false;
          break;
        }
      }
      else
      {
        CHECK(!tmpValidReturnSplit, "invalid RD of sub-TU partitions for ACT");
      }
    } while (partitioner.nextPart(*csSplit));

    partitioner.exitCurrSplit();

    if (splitIsSelected)
    {
      unsigned compCbf[3] = { 0, 0, 0 };
      for (auto &currTU : csSplit->traverseTUs(currArea, partitioner.chType))
      {
        for (unsigned ch = 0; ch < getNumberValidTBlocks(*csSplit->pcv); ch++)
        {
          compCbf[ch] |= (TU::getCbfAtDepth(currTU, ComponentID(ch), currDepth + 1) ? 1 : 0);
        }
      }

      for (auto &currTU : csSplit->traverseTUs(currArea, partitioner.chType))
      {
        TU::setCbfAtDepth(currTU, COMPONENT_Y, currDepth, compCbf[COMPONENT_Y]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cb, currDepth, compCbf[COMPONENT_Cb]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cr, currDepth, compCbf[COMPONENT_Cr]);
      }

      m_CABACEstimator->getCtx() = ctxStart;
      csSplit->fracBits = xGetIntraFracBitsQT(*csSplit, partitioner, true, true, -1, TU_NO_ISP);
      csSplit->cost = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      validReturnSplit = true;
    }
  }

  bool retVal = false;
  if (csFull || csSplit)
  {
    if (sps.getUseLFNST())
    {
      if (validReturnFull || validReturnSplit)
      {
        retVal = true;
      }
    }
    else
    {
      CHECK(!validReturnFull && !validReturnSplit, "illegal TU optimization");
      retVal = true;
    }
  }
  return retVal;
}
#endif

ChromaCbfs IntraSearch::xRecurIntraChromaCodingQT( CodingStructure &cs, Partitioner& partitioner, const double bestCostSoFar, const PartSplit ispType )
{
  UnitArea currArea                   = partitioner.currArea();
  const bool keepResi                 = cs.sps->getUseLMChroma() || KEEP_PRED_AND_RESI_SIGNALS;
  if( !currArea.Cb().valid() ) return ChromaCbfs( false );


  TransformUnit &currTU               = *cs.getTU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );
  const PredictionUnit &pu            = *cs.getPU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );

  bool lumaUsesISP                    = false;
  uint32_t     currDepth                  = partitioner.currTrDepth;
  const PPS &pps                      = *cs.pps;
  ChromaCbfs cbfs                     ( false );

  if (currDepth == currTU.depth)
  {
    if (!currArea.Cb().valid() || !currArea.Cr().valid())
    {
      return cbfs;
    }


    CodingStructure &saveCS = *m_pSaveCS[1];
    saveCS.pcv      = cs.pcv;
    saveCS.picture  = cs.picture;
    saveCS.area.repositionTo( cs.area );
    saveCS.initStructData( MAX_INT, true );

    if( !currTU.cu->isSepTree() && currTU.cu->ispMode )
    {
      saveCS.clearCUs();
      CodingUnit& auxCU = saveCS.addCU( *currTU.cu, partitioner.chType );
      auxCU.ispMode = currTU.cu->ispMode;
      saveCS.sps = currTU.cs->sps;
      saveCS.clearPUs();
      saveCS.addPU( *currTU.cu->firstPU, partitioner.chType );
    }

    TransformUnit &tmpTU = saveCS.addTU(currArea, partitioner.chType);


    cs.setDecomp(currArea.Cb(), true); // set in advance (required for Cb2/Cr2 in 4:2:2 video)

    const unsigned      numTBlocks  = ::getNumberValidTBlocks( *cs.pcv );

    CompArea&  cbArea         = currTU.blocks[COMPONENT_Cb];
    CompArea&  crArea         = currTU.blocks[COMPONENT_Cr];
    double     bestCostCb     = MAX_DOUBLE;
    double     bestCostCr     = MAX_DOUBLE;
    Distortion bestDistCb     = 0;
    Distortion bestDistCr     = 0;
    int        maxModesTested = 0;
    bool       earlyExitISP   = false;

    TempCtx ctxStartTU( m_CtxCache );
    TempCtx ctxStart  ( m_CtxCache );
    TempCtx ctxBest   ( m_CtxCache );

    ctxStartTU       = m_CABACEstimator->getCtx();
    currTU.jointCbCr = 0;

    // Do predictions here to avoid repeating the "default0Save1Load2" stuff
#if JVET_P0059_CHROMA_BDPCM
    int  predMode   = pu.cu->bdpcmModeChroma ? BDPCM_IDX : PU::getFinalIntraMode(pu, CHANNEL_TYPE_CHROMA);
#else
    int  predMode   = PU::getFinalIntraMode( pu, CHANNEL_TYPE_CHROMA );
#endif

    PelBuf piPredCb = cs.getPredBuf(cbArea);
    PelBuf piPredCr = cs.getPredBuf(crArea);

    initIntraPatternChType( *currTU.cu, cbArea);
    initIntraPatternChType( *currTU.cu, crArea);

    if( PU::isLMCMode( predMode ) )
    {
      xGetLumaRecPixels( pu, cbArea );
      predIntraChromaLM( COMPONENT_Cb, piPredCb, pu, cbArea, predMode );
      predIntraChromaLM( COMPONENT_Cr, piPredCr, pu, crArea, predMode );
    }
    else
    {
      predIntraAng( COMPONENT_Cb, piPredCb, pu);
      predIntraAng( COMPONENT_Cr, piPredCr, pu);
    }

    // determination of chroma residuals including reshaping and cross-component prediction
    //----- get chroma residuals -----
    PelBuf resiCb  = cs.getResiBuf(cbArea);
    PelBuf resiCr  = cs.getResiBuf(crArea);
    resiCb.copyFrom( cs.getOrgBuf (cbArea) );
    resiCr.copyFrom( cs.getOrgBuf (crArea) );
    resiCb.subtract( piPredCb );
    resiCr.subtract( piPredCr );

    //----- get reshape parameter ----
#if JVET_P1006_PICTURE_HEADER
    bool doReshaping = ( cs.picHeader->getLmcsEnabledFlag() && cs.picHeader->getLmcsChromaResidualScaleFlag()
#else
    bool doReshaping = ( cs.slice->getLmcsEnabledFlag() && cs.slice->getLmcsChromaResidualScaleFlag()
#endif
                         && (cs.slice->isIntra() || m_pcReshape->getCTUFlag()) && (cbArea.width * cbArea.height > 4) );
    if( doReshaping )
    {
      const Area area = currTU.Y().valid() ? currTU.Y() : Area(recalcPosition(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.blocks[currTU.chType].pos()), recalcSize(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.blocks[currTU.chType].size()));
      const CompArea &areaY = CompArea(COMPONENT_Y, currTU.chromaFormat, area);
      int adj = m_pcReshape->calculateChromaAdjVpduNei(currTU, areaY);
      currTU.setChromaAdj(adj);
    }

    //----- get cross component prediction parameters -----
    bool checkCrossComponentPrediction = PU::isChromaIntraModeCrossCheckMode( pu ) && pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && TU::getCbf( currTU, COMPONENT_Y );
    int  compAlpha[MAX_NUM_COMPONENT] = { 0, 0, 0 };
    if( checkCrossComponentPrediction )
    {
      compAlpha[COMPONENT_Cb] = xCalcCrossComponentPredictionAlpha( currTU, COMPONENT_Cb, m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() );
      compAlpha[COMPONENT_Cr] = xCalcCrossComponentPredictionAlpha( currTU, COMPONENT_Cr, m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() );
      if( compAlpha[COMPONENT_Cb] == 0 && compAlpha[COMPONENT_Cr] == 0 )
      {
        checkCrossComponentPrediction = false;
      }
    }

    //===== store original residual signals (std and crossCompPred) =====
    CompStorage  orgResiCb[5], orgResiCr[5]; // 0:std, 1-3:jointCbCr (placeholder at this stage), 4:crossComp
    for( int k = 0; k < (checkCrossComponentPrediction?5:1); k+=4 )
    {
      orgResiCb[k].create( cbArea );
      orgResiCr[k].create( crArea );
      if( k >= 4 ) {
        CrossComponentPrediction::crossComponentPrediction( currTU, COMPONENT_Cb, cs.getResiBuf(currTU.Y()), resiCb, orgResiCb[k], false);
        CrossComponentPrediction::crossComponentPrediction( currTU, COMPONENT_Cr, cs.getResiBuf(currTU.Y()), resiCr, orgResiCr[k], false);
      } else {
        orgResiCb[k].copyFrom( resiCb );
        orgResiCr[k].copyFrom( resiCr );
      }
      if( doReshaping )
      {
        int cResScaleInv = currTU.getChromaAdj();
        orgResiCb[k].scaleSignal( cResScaleInv, 1, currTU.cu->cs->slice->clpRng(COMPONENT_Cb) );
        orgResiCr[k].scaleSignal( cResScaleInv, 1, currTU.cu->cs->slice->clpRng(COMPONENT_Cr) );
      }
    }

    for( uint32_t c = COMPONENT_Cb; c < numTBlocks; c++)
    {
      const ComponentID compID  = ComponentID(c);
      const CompArea&   area    = currTU.blocks[compID];

      double     dSingleCost    = MAX_DOUBLE;
      int        bestModeId     = 0;
      Distortion singleDistCTmp = 0;
      double     singleCostTmp  = 0;
      const int  crossCPredictionModesToTest = checkCrossComponentPrediction ? 2 : 1;
#if JVET_P0058_CHROMA_TS
      const bool tsAllowed = TU::isTSAllowed(currTU, compID) && (m_pcEncCfg->getUseChromaTS());
      uint8_t nNumTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
      std::vector<TrMode> trModes;
      trModes.push_back(TrMode(0, true)); // DCT2

      if (tsAllowed)
      {
          trModes.push_back(TrMode(1, true));//TS
      }
      CHECK(!currTU.Cb().valid(), "Invalid TU");
#endif

#if JVET_P0058_CHROMA_TS
      const int  totalModesToTest            = crossCPredictionModesToTest * nNumTransformCands;
      bool cbfDCT2 = true;
#else
      const int  totalModesToTest            = crossCPredictionModesToTest;
#endif
      const bool isOneMode                   = false;
      maxModesTested                         = totalModesToTest > maxModesTested ? totalModesToTest : maxModesTested;

      int currModeId = 0;
      int default0Save1Load2 = 0;


      if (!isOneMode)
      {
        ctxStart = m_CABACEstimator->getCtx();
      }

#if JVET_P0058_CHROMA_TS
      for (int modeId = 0; modeId < nNumTransformCands; modeId++)
#endif
      {
        for (int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
        {
          resiCb.copyFrom( orgResiCb[4*crossCPredictionModeId] );
          resiCr.copyFrom( orgResiCr[4*crossCPredictionModeId] );

          currTU.compAlpha    [compID] = ( crossCPredictionModeId ? compAlpha[compID] : 0 );

#if JVET_P0058_CHROMA_TS
#if JVET_P0059_CHROMA_BDPCM
          currTU.mtsIdx[compID] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : trModes[modeId].first;
#else
          currTU.mtsIdx[compID] = trModes[modeId].first;
#endif
#endif

          currModeId++;

          const bool isFirstMode = (currModeId == 1);
          const bool isLastMode  = false; // Always store output to saveCS and tmpTU

#if JVET_AHG14_LOSSLESS
          if( !( m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING ) )
          {
#endif
#if JVET_P0058_CHROMA_TS
           //if DCT2's cbf==0, skip ts search
          if (!cbfDCT2 && trModes[modeId].first == MTS_SKIP)
          {
              break;
          }
          if (!trModes[modeId].second)
          {
              continue;
          }
#endif
#if JVET_AHG14_LOSSLESS
          }
#endif

          if (!isFirstMode) // if not first mode to be tested
          {
            m_CABACEstimator->getCtx() = ctxStart;
          }

          singleDistCTmp = 0;

#if JVET_P0058_CHROMA_TS
          if (nNumTransformCands > 1)
          {
              xIntraCodingTUBlock(currTU, compID, crossCPredictionModeId != 0, singleDistCTmp, default0Save1Load2, nullptr, modeId == 0 ? &trModes : nullptr, true);
          }
          else
          {
              xIntraCodingTUBlock(currTU, compID, crossCPredictionModeId != 0, singleDistCTmp, default0Save1Load2);
          }
#else
          xIntraCodingTUBlock( currTU, compID, crossCPredictionModeId != 0, singleDistCTmp, default0Save1Load2 );
#endif

#if JVET_P0058_CHROMA_TS
#if JVET_P0059_CHROMA_BDPCM
          if (((crossCPredictionModeId == 1) && (currTU.compAlpha[compID] == 0)) || ((currTU.mtsIdx[compID] == MTS_SKIP && !currTU.cu->bdpcmModeChroma) && !TU::getCbf(currTU, compID))) //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
#else
          if (((crossCPredictionModeId == 1) && (currTU.compAlpha[compID] == 0)) || ((currTU.mtsIdx[compID] == MTS_SKIP) && !TU::getCbf(currTU, compID))) //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
#endif
#else
          if( ( ( crossCPredictionModeId == 1 ) && ( currTU.compAlpha[compID] == 0 ) ) ) //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
#endif
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else if( lumaUsesISP && bestCostSoFar != MAX_DOUBLE && c == COMPONENT_Cb )
          {
            uint64_t fracBitsTmp = xGetIntraFracBitsQTSingleChromaComponent( cs, partitioner, ComponentID( c ) );
            singleCostTmp = m_pcRdCost->calcRdCost( fracBitsTmp, singleDistCTmp );
            if( isOneMode || ( !isOneMode && !isLastMode ) )
            {
              m_CABACEstimator->getCtx() = ctxStart;
            }
          }
          else if( !isOneMode )
          {
            uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma( currTU, compID );
            singleCostTmp = m_pcRdCost->calcRdCost( fracBitsTmp, singleDistCTmp );
          }

          if( singleCostTmp < dSingleCost )
          {
            dSingleCost = singleCostTmp;
            bestModeId  = currModeId;

            if ( c == COMPONENT_Cb )
            {
              bestCostCb = singleCostTmp;
              bestDistCb = singleDistCTmp;
            }
            else
            {
              bestCostCr = singleCostTmp;
              bestDistCr = singleDistCTmp;
            }

#if JVET_P0058_CHROMA_TS
            if (currTU.mtsIdx[compID] == MTS_DCT2_DCT2)
            {
                cbfDCT2 = TU::getCbfAtDepth(currTU, compID, currDepth);
            }
#endif

            if( !isLastMode )
            {
#if KEEP_PRED_AND_RESI_SIGNALS
              saveCS.getPredBuf   (area).copyFrom(cs.getPredBuf   (area));
              saveCS.getOrgResiBuf(area).copyFrom(cs.getOrgResiBuf(area));
#endif
              saveCS.getPredBuf   (area).copyFrom(cs.getPredBuf   (area));
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

      if( lumaUsesISP && dSingleCost > bestCostSoFar && c == COMPONENT_Cb )
      {
        //Luma + Cb cost is already larger than the best cost, so we don't need to test Cr
        cs.dist = MAX_UINT;
        m_CABACEstimator->getCtx() = ctxStart;
        earlyExitISP               = true;
        break;
        //return cbfs;
      }

      // Done with one component of separate coding of Cr and Cb, just switch to the best Cb contexts if Cr coding is still to be done
      if ( c == COMPONENT_Cb && bestModeId < totalModesToTest)
      {
        m_CABACEstimator->getCtx() = ctxBest;

        currTU.copyComponentFrom(tmpTU, COMPONENT_Cb); // Cbf of Cb is needed to estimate cost for Cr Cbf
      }
    }

    if ( !earlyExitISP )
    {
      // Test using joint chroma residual coding
      double     bestCostCbCr   = bestCostCb + bestCostCr;
      Distortion bestDistCbCr   = bestDistCb + bestDistCr;
      int        bestJointCbCr  = 0;
      bool       lastIsBest     = false;
      std::vector<int>  jointCbfMasksToTest;
      if ( cs.sps->getJointCbCrEnabledFlag() && (TU::getCbf(tmpTU, COMPONENT_Cb) || TU::getCbf(tmpTU, COMPONENT_Cr)))
      {
        jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(currTU, orgResiCb, orgResiCr);
      }
      for( int cbfMask : jointCbfMasksToTest )
      {
        Distortion distTmp = 0;

        currTU.jointCbCr               = (uint8_t)cbfMask;
        currTU.compAlpha[COMPONENT_Cb] = 0;
        currTU.compAlpha[COMPONENT_Cr] = 0;
#if JVET_P0058_CHROMA_TS
        // encoder bugfix: initialize mtsIdx for chroma under JointCbCrMode.
        currTU.mtsIdx[COMPONENT_Cb] = currTU.mtsIdx[COMPONENT_Cr]  = MTS_DCT2_DCT2;
#endif
        m_CABACEstimator->getCtx() = ctxStartTU;

        resiCb.copyFrom( orgResiCb[cbfMask] );
        resiCr.copyFrom( orgResiCr[cbfMask] );
        xIntraCodingTUBlock( currTU, COMPONENT_Cb, false, distTmp, 0 );

        double costTmp = std::numeric_limits<double>::max();
        if( distTmp < std::numeric_limits<Distortion>::max() )
        {
          uint64_t bits  = xGetIntraFracBitsQTChroma( currTU, COMPONENT_Cb );
          costTmp = m_pcRdCost->calcRdCost( bits, distTmp );
        }

        if( costTmp < bestCostCbCr )
        {
          bestCostCbCr  = costTmp;
          bestDistCbCr  = distTmp;
          bestJointCbCr = currTU.jointCbCr;

          // store data
          if( cbfMask != jointCbfMasksToTest.back() )
          {
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getOrgResiBuf(cbArea).copyFrom(cs.getOrgResiBuf(cbArea));
            saveCS.getOrgResiBuf(crArea).copyFrom(cs.getOrgResiBuf(crArea));
#endif
            saveCS.getPredBuf   (cbArea).copyFrom(cs.getPredBuf   (cbArea));
            saveCS.getPredBuf   (crArea).copyFrom(cs.getPredBuf   (crArea));
            if( keepResi )
            {
              saveCS.getResiBuf (cbArea).copyFrom(cs.getResiBuf   (cbArea));
              saveCS.getResiBuf (crArea).copyFrom(cs.getResiBuf   (crArea));
            }
            saveCS.getRecoBuf   (cbArea).copyFrom(cs.getRecoBuf   (cbArea));
            saveCS.getRecoBuf   (crArea).copyFrom(cs.getRecoBuf   (crArea));

            tmpTU.copyComponentFrom(currTU, COMPONENT_Cb);
            tmpTU.copyComponentFrom(currTU, COMPONENT_Cr);

            ctxBest = m_CABACEstimator->getCtx();
          }
          else
          {
            lastIsBest = true;
          }
        }
      }

      // Retrieve the best CU data (unless it was the very last one tested)
      if ( !( maxModesTested == 1 && jointCbfMasksToTest.empty() ) && !lastIsBest )
      {
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf   (cbArea).copyFrom(saveCS.getPredBuf   (cbArea));
        cs.getOrgResiBuf(cbArea).copyFrom(saveCS.getOrgResiBuf(cbArea));
        cs.getPredBuf   (crArea).copyFrom(saveCS.getPredBuf   (crArea));
        cs.getOrgResiBuf(crArea).copyFrom(saveCS.getOrgResiBuf(crArea));
#endif
        cs.getPredBuf   (cbArea).copyFrom(saveCS.getPredBuf   (cbArea));
        cs.getPredBuf   (crArea).copyFrom(saveCS.getPredBuf   (crArea));

        if( keepResi )
        {
          cs.getResiBuf (cbArea).copyFrom(saveCS.getResiBuf   (cbArea));
          cs.getResiBuf (crArea).copyFrom(saveCS.getResiBuf   (crArea));
        }
        cs.getRecoBuf   (cbArea).copyFrom(saveCS.getRecoBuf   (cbArea));
        cs.getRecoBuf   (crArea).copyFrom(saveCS.getRecoBuf   (crArea));

        currTU.copyComponentFrom(tmpTU, COMPONENT_Cb);
        currTU.copyComponentFrom(tmpTU, COMPONENT_Cr);

        m_CABACEstimator->getCtx() = ctxBest;
      }

      // Copy results to the picture structures
      cs.picture->getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
      cs.picture->getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));
      cs.picture->getPredBuf(cbArea).copyFrom(cs.getPredBuf(cbArea));
      cs.picture->getPredBuf(crArea).copyFrom(cs.getPredBuf(crArea));

      cbfs.cbf(COMPONENT_Cb) = TU::getCbf(currTU, COMPONENT_Cb);
      cbfs.cbf(COMPONENT_Cr) = TU::getCbf(currTU, COMPONENT_Cr);

      currTU.jointCbCr = ( (cbfs.cbf(COMPONENT_Cb) + cbfs.cbf(COMPONENT_Cr)) ? bestJointCbCr : 0 );
      cs.dist         += bestDistCbCr;
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
    else if( currTU.cu->ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
      THROW( "Implicit TU split not available" );

    do
    {
      ChromaCbfs subCbfs = xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );

      for( uint32_t ch = COMPONENT_Cb; ch < numValidTBlocks; ch++ )
      {
        const ComponentID compID = ComponentID( ch );
        SplitCbfs.cbf( compID ) |= subCbfs.cbf( compID );
      }
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();

    if( lumaUsesISP && cs.dist == MAX_UINT )
    {
      return cbfs;
    }
    {

      cbfs.Cb |= SplitCbfs.Cb;
      cbfs.Cr |= SplitCbfs.Cr;

      if( !lumaUsesISP )
      {
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
  }

  return cbfs;
}

uint64_t IntraSearch::xFracModeBitsIntra(PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &chType)
{
  uint32_t orgMode = uiMode;

  if (!pu.ciipFlag)
  std::swap(orgMode, pu.intraDir[chType]);

  m_CABACEstimator->resetBits();

  if( isLuma( chType ) )
  {
    if (!pu.ciipFlag)
    {
      m_CABACEstimator->intra_luma_pred_mode(pu);
    }
  }
  else
  {
    m_CABACEstimator->intra_chroma_pred_mode( pu );
  }

  if ( !pu.ciipFlag )
  std::swap(orgMode, pu.intraDir[chType]);

  return m_CABACEstimator->getEstFracBits();
}

#if JVET_P0517_ADAPTIVE_COLOR_TRANSFORM 
void IntraSearch::sortRdModeListFirstColorSpace(ModeInfo mode, double cost, char bdpcmMode, ModeInfo* rdModeList, double* rdCostList, char* bdpcmModeList, int& candNum)
{
  if (candNum == 0)
  {
    rdModeList[0] = mode;
    rdCostList[0] = cost;
    bdpcmModeList[0] = bdpcmMode;
    candNum++;
    return;
  }

  int insertPos = -1;
  for (int pos = candNum - 1; pos >= 0; pos--)
  {
    if (cost < rdCostList[pos])
    {
      insertPos = pos;
    }
  }

  if (insertPos >= 0)
  {
    for (int i = candNum - 1; i >= insertPos; i--)
    {
      rdModeList[i + 1] = rdModeList[i];
      rdCostList[i + 1] = rdCostList[i];
      bdpcmModeList[i + 1] = bdpcmModeList[i];
    }
    rdModeList[insertPos] = mode;
    rdCostList[insertPos] = cost;
    bdpcmModeList[insertPos] = bdpcmMode;
    candNum++;
  }
  else
  {
    rdModeList[candNum] = mode;
    rdCostList[candNum] = cost;
    bdpcmModeList[candNum] = bdpcmMode;
    candNum++;
  }

  CHECK(candNum > FAST_UDI_MAX_RDMODE_NUM, "exceed intra mode candidate list capacity");

  return;
}

void IntraSearch::invalidateBestRdModeFirstColorSpace()
{
  int numSaveRdClass = 4 * NUM_LFNST_NUM_PER_SET * 2;
  int savedRdModeListSize = FAST_UDI_MAX_RDMODE_NUM;

  for (int i = 0; i < numSaveRdClass; i++)
  {
    m_numSavedRdModeFirstColorSpace[i] = 0;
    for (int j = 0; j < savedRdModeListSize; j++)
    {
#if JVET_P0803_COMBINED_MIP_CLEANUP
      m_savedRdModeFirstColorSpace[i][j] = ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, 0);
#else
      m_savedRdModeFirstColorSpace[i][j] = ModeInfo(false, 0, NOT_INTRA_SUBPARTITIONS, 0);
#endif
      m_savedBDPCMModeFirstColorSpace[i][j] = 0;
      m_savedRdCostFirstColorSpace[i][j] = MAX_DOUBLE;
    }
  }
}
#endif

template<typename T, size_t N>
void IntraSearch::reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const PredictionUnit &pu, const bool fastMip)
{
  const int maxCandPerType = numModesForFullRD >> 1;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> tempRdModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> tempCandCostList;
  const double minCost = candCostList[0];
  bool keepOneMip = candModeList.size() > numModesForFullRD;

  int numConv = 0;
  int numMip = 0;
  for (int idx = 0; idx < candModeList.size() - (keepOneMip?0:1); idx++)
  {
    bool addMode = false;
    const ModeInfo& orgMode = candModeList[idx];

    if (!orgMode.mipFlg)
    {
      addMode = (numConv < 3);
      numConv += addMode ? 1:0;
    }
    else
    {
      addMode = ( numMip < maxCandPerType || (candCostList[idx] < thresholdHadCost * minCost) || keepOneMip );
      keepOneMip = false;
      numMip += addMode ? 1:0;
    }
    if( addMode )
    {
      tempRdModeList.push_back(orgMode);
      tempCandCostList.push_back(candCostList[idx]);
    }
  }

  if ((pu.lwidth() > 8 && pu.lheight() > 8))
  {
    // Sort MIP candidates by Hadamard cost
#if JVET_P0803_COMBINED_MIP_CLEANUP
    const int transpOff = getNumModesMip( pu.Y() );
#else
    const int transpOff = getNumModesMip(pu.Y()) / 2;
#endif
    static_vector<uint8_t, FAST_UDI_MAX_RDMODE_NUM> sortedMipModes(0);
    static_vector<double, FAST_UDI_MAX_RDMODE_NUM> sortedMipCost(0);
#if JVET_P0803_COMBINED_MIP_CLEANUP
    for( uint8_t mode : { 0, 1, 2 } )
#else
    for (uint8_t mode : { 3, 4, 5 })
#endif
    {
      uint8_t candMode = mode + uint8_t((mipHadCost[mode + transpOff] < mipHadCost[mode]) ? transpOff : 0);
      updateCandList(candMode, mipHadCost[candMode], sortedMipModes, sortedMipCost, 3);
    }

    // Append MIP mode to RD mode list
    const int modeListSize = int(tempRdModeList.size());
    for (int idx = 0; idx < 3; idx++)
    {
#if JVET_P0803_COMBINED_MIP_CLEANUP
      const bool     isTransposed = (sortedMipModes[idx] >= transpOff ? true : false);
      const uint32_t mipIdx       = (isTransposed ? sortedMipModes[idx] - transpOff : sortedMipModes[idx]);
      const ModeInfo mipMode( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mipIdx );
#else
      const ModeInfo mipMode(true, 0, NOT_INTRA_SUBPARTITIONS, sortedMipModes[idx]);
#endif
      bool alreadyIncluded = false;
      for (int modeListIdx = 0; modeListIdx < modeListSize; modeListIdx++)
      {
        if (tempRdModeList[modeListIdx] == mipMode)
        {
          alreadyIncluded = true;
          break;
        }
      }

      if (!alreadyIncluded)
      {
        tempRdModeList.push_back(mipMode);
        tempCandCostList.push_back(0);
        if( fastMip ) break;
      }
    }
  }

  candModeList = tempRdModeList;
  candCostList = tempCandCostList;
  numModesForFullRD = int(candModeList.size());
}

// It decides which modes from the ISP lists can be full RD tested
void IntraSearch::xGetNextISPMode(ModeInfo& modeInfo, const ModeInfo* lastMode, const Size cuSize)
{
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>* rdModeLists[2] = { &m_ispCandListHor, &m_ispCandListVer };

#if JVET_P1026_ISP_LFNST_COMBINATION
  const int curIspLfnstIdx = m_curIspLfnstIdx;
  if (curIspLfnstIdx >= NUM_LFNST_NUM_PER_SET)
  {
    //All lfnst indices have been checked
    return;
  }
#endif

  ISPType nextISPcandSplitType;
#if JVET_P1026_ISP_LFNST_COMBINATION
  auto& ispTestedModes = m_ispTestedModes[curIspLfnstIdx];
#else
  auto& ispTestedModes = m_ispTestedModes;
#endif
  const bool horSplitIsTerminated = ispTestedModes.splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1];
  const bool verSplitIsTerminated = ispTestedModes.splitIsFinished[VER_INTRA_SUBPARTITIONS - 1];
  if (!horSplitIsTerminated && !verSplitIsTerminated)
  {
    nextISPcandSplitType = !lastMode ? HOR_INTRA_SUBPARTITIONS : lastMode->ispMod == HOR_INTRA_SUBPARTITIONS ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
  }
  else if (!horSplitIsTerminated && verSplitIsTerminated)
  {
    nextISPcandSplitType = HOR_INTRA_SUBPARTITIONS;
  }
  else if (horSplitIsTerminated && !verSplitIsTerminated)
  {
    nextISPcandSplitType = VER_INTRA_SUBPARTITIONS;
  }
  else
  {
#if JVET_P1026_ISP_LFNST_COMBINATION
    xFinishISPModes();
#endif
    return;   // no more modes will be tested
  }

  int maxNumSubPartitions = ispTestedModes.numTotalParts[nextISPcandSplitType - 1];

#if JVET_P1026_ISP_LFNST_COMBINATION
  // We try to break the split here for lfnst > 0 according to the first mode 
  if (curIspLfnstIdx > 0 && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] == 1)
  {
    int firstModeThisSplit = ispTestedModes.getTestedIntraMode(nextISPcandSplitType, 0);
    int numSubPartsFirstModeThisSplit = ispTestedModes.getNumCompletedSubParts(nextISPcandSplitType, firstModeThisSplit);
    CHECK(numSubPartsFirstModeThisSplit < 0, "wrong number of subpartitions!");
    bool stopThisSplit = false;
    bool stopThisSplitAllLfnsts = false;
    if (numSubPartsFirstModeThisSplit < maxNumSubPartitions)
    {
      stopThisSplit = true;
      if (m_pcEncCfg->getUseFastISP() && curIspLfnstIdx == 1 && numSubPartsFirstModeThisSplit < maxNumSubPartitions - 1)
      {
        stopThisSplitAllLfnsts = true;
      }
    }

    if (stopThisSplit)
    {
      ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
      if (curIspLfnstIdx == 1 && stopThisSplitAllLfnsts)
      {
        m_ispTestedModes[2].splitIsFinished[nextISPcandSplitType - 1] = true;
      }
      return;
    }
  }
#endif

#if JVET_P1026_ISP_LFNST_COMBINATION
  // We try to break the split here for lfnst = 0 or all lfnst indices according to the first two modes 
  if (curIspLfnstIdx == 0 && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] == 2)
#else
  if (ispTestedModes.numTestedModes[nextISPcandSplitType - 1] >= 2)
#endif
  {
    // Split stop criteria after checking the performance of previously tested intra modes
    const int thresholdSplit1 = maxNumSubPartitions;
    bool stopThisSplit = false;
#if JVET_P1026_ISP_LFNST_COMBINATION
    bool stopThisSplitForAllLFNSTs = false;
    const int thresholdSplit1ForAllLFNSTs = maxNumSubPartitions - 1;
#endif

    int mode1 = ispTestedModes.getTestedIntraMode((ISPType)nextISPcandSplitType, 0);
    mode1 = mode1 == DC_IDX ? -1 : mode1;
    int numSubPartsBestMode1 = mode1 != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)nextISPcandSplitType, mode1) : -1;
    int mode2 = ispTestedModes.getTestedIntraMode((ISPType)nextISPcandSplitType, 1);
    mode2 = mode2 == DC_IDX ? -1 : mode2;
    int numSubPartsBestMode2 = mode2 != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)nextISPcandSplitType, mode2) : -1;

    // 1) The 2 most promising modes do not reach a certain number of sub-partitions
    if (numSubPartsBestMode1 != -1 && numSubPartsBestMode2 != -1)
    {
      if (numSubPartsBestMode1 < thresholdSplit1 && numSubPartsBestMode2 < thresholdSplit1)
      {
        stopThisSplit = true;
#if JVET_P1026_ISP_LFNST_COMBINATION
        if (curIspLfnstIdx == 0 && numSubPartsBestMode1 < thresholdSplit1ForAllLFNSTs && numSubPartsBestMode2 < thresholdSplit1ForAllLFNSTs)
        {
          stopThisSplitForAllLFNSTs = true;
        }
#endif
      }
#if JVET_P1026_ISP_LFNST_COMBINATION
      else
      {
        //we stop also if the cost is MAX_DOUBLE for both modes
        double mode1Cost = ispTestedModes.getRDCost(nextISPcandSplitType, mode1);
        double mode2Cost = ispTestedModes.getRDCost(nextISPcandSplitType, mode2);
        if (!(mode1Cost < MAX_DOUBLE || mode2Cost < MAX_DOUBLE))
        {
          stopThisSplit = true;
        }
      }
#endif
    }

    if (!stopThisSplit)
    {
      // 2) One split type may be discarded by comparing the number of sub-partitions of the best angle modes of both splits 
      ISPType otherSplit = nextISPcandSplitType == HOR_INTRA_SUBPARTITIONS ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
      int  numSubPartsBestMode2OtherSplit = mode2 != -1 ? ispTestedModes.getNumCompletedSubParts(otherSplit, mode2) : -1;
#if JVET_P1026_ISP_LFNST_COMBINATION
      if (numSubPartsBestMode2OtherSplit != -1 && numSubPartsBestMode2 != -1 && ispTestedModes.bestSplitSoFar != nextISPcandSplitType)
#else
      if (numSubPartsBestMode2OtherSplit != -1 && numSubPartsBestMode2 != -1)
#endif
      {
        if (numSubPartsBestMode2OtherSplit > numSubPartsBestMode2)
        {
          stopThisSplit = true;
        }
#if JVET_P1026_ISP_LFNST_COMBINATION
        // both have the same number of subpartitions
        else if (numSubPartsBestMode2OtherSplit == numSubPartsBestMode2)
#else
        else if (numSubPartsBestMode2OtherSplit == numSubPartsBestMode2 && numSubPartsBestMode2OtherSplit == maxNumSubPartitions)
#endif
        {
#if JVET_P1026_ISP_LFNST_COMBINATION
          // both have the maximum number of subpartitions, so it compares RD costs to decide
          if (numSubPartsBestMode2OtherSplit == maxNumSubPartitions)
          {
#endif
            double rdCostBestMode2ThisSplit = ispTestedModes.getRDCost(nextISPcandSplitType, mode2);
            double rdCostBestMode2OtherSplit = ispTestedModes.getRDCost(otherSplit, mode2);
            double threshold = 1.3;
            if (rdCostBestMode2ThisSplit == MAX_DOUBLE || rdCostBestMode2OtherSplit < rdCostBestMode2ThisSplit * threshold)
            {
              stopThisSplit = true;
            }
#if JVET_P1026_ISP_LFNST_COMBINATION
          }
          else // none of them reached the maximum number of subpartitions with the best angle modes, so it compares the results with the the planar mode
          {
            int  numSubPartsBestMode1OtherSplit = mode1 != -1 ? ispTestedModes.getNumCompletedSubParts(otherSplit, mode1) : -1;
            if (numSubPartsBestMode1OtherSplit != -1 && numSubPartsBestMode1 != -1 && numSubPartsBestMode1OtherSplit > numSubPartsBestMode1)
            {
              stopThisSplit = true;
            }
          }
#endif
        }
      }
    }
    if (stopThisSplit)
    {
      ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
#if JVET_P1026_ISP_LFNST_COMBINATION
      if (stopThisSplitForAllLFNSTs)
      {
        for (int lfnstIdx = 1; lfnstIdx < NUM_LFNST_NUM_PER_SET; lfnstIdx++)
        {
          m_ispTestedModes[lfnstIdx].splitIsFinished[nextISPcandSplitType - 1] = true;
        }
      }
#endif
      return;
    }
  }

  // Now a new mode is retrieved from the list and it has to be decided whether it should be tested or not
  if (ispTestedModes.candIndexInList[nextISPcandSplitType - 1] < rdModeLists[nextISPcandSplitType - 1]->size())
  {
    ModeInfo candidate = rdModeLists[nextISPcandSplitType - 1]->at(ispTestedModes.candIndexInList[nextISPcandSplitType - 1]);
    ispTestedModes.candIndexInList[nextISPcandSplitType - 1]++;

    // extra modes are only tested if ISP has won so far
    if (ispTestedModes.candIndexInList[nextISPcandSplitType - 1] > ispTestedModes.numOrigModesToTest)
    {
      if (ispTestedModes.bestSplitSoFar != candidate.ispMod || ispTestedModes.bestModeSoFar == PLANAR_IDX)
      {
#if JVET_P1026_ISP_LFNST_COMBINATION
        ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
#endif
        return;
      }
    }

    bool testCandidate = true;

    // we look for a reference mode that has already been tested within the window and decide to test the new one according to the reference mode costs
#if JVET_P1026_ISP_LFNST_COMBINATION
    if (maxNumSubPartitions > 2 && (curIspLfnstIdx > 0 || (candidate.modeId >= DC_IDX && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] >= 2)))
#else
    if (candidate.modeId >= DC_IDX && maxNumSubPartitions > 2 && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] >= 2)
#endif
    {
#if JVET_P1026_ISP_LFNST_COMBINATION
      int       refLfnstIdx = -1;
#endif
      const int angWindowSize = 5;
      int       numSubPartsLeftMode, numSubPartsRightMode, numSubPartsRefMode, leftIntraMode = -1, rightIntraMode = -1;
      int       windowSize = candidate.modeId > DC_IDX ? angWindowSize : 1;
      int       numSamples = cuSize.width << floorLog2(cuSize.height);
      int       numSubPartsLimit = numSamples >= 256 ? maxNumSubPartitions - 1 : 2;

#if JVET_P1026_ISP_LFNST_COMBINATION
      xFindAlreadyTestedNearbyIntraModes(curIspLfnstIdx, (int)candidate.modeId, &refLfnstIdx, &leftIntraMode, &rightIntraMode, (ISPType)candidate.ispMod, windowSize);
#else
      xFindAlreadyTestedNearbyIntraModes((int)candidate.modeId, &leftIntraMode, &rightIntraMode, (ISPType)candidate.ispMod, windowSize);
#endif

#if JVET_P1026_ISP_LFNST_COMBINATION
      if (refLfnstIdx != -1 && refLfnstIdx != curIspLfnstIdx)
      {
        CHECK(leftIntraMode != candidate.modeId || rightIntraMode != candidate.modeId, "wrong intra mode and lfnstIdx values!");
        numSubPartsRefMode = m_ispTestedModes[refLfnstIdx].getNumCompletedSubParts((ISPType)candidate.ispMod, candidate.modeId);
        CHECK(numSubPartsRefMode <= 0, "Wrong value of the number of subpartitions completed!");
      }
      else
      {
#endif
        numSubPartsLeftMode = leftIntraMode != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)candidate.ispMod, leftIntraMode) : -1;
        numSubPartsRightMode = rightIntraMode != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)candidate.ispMod, rightIntraMode) : -1;

        numSubPartsRefMode = std::max(numSubPartsLeftMode, numSubPartsRightMode);
#if JVET_P1026_ISP_LFNST_COMBINATION
      }
#endif

      if (numSubPartsRefMode > 0)
      {
        // The mode was found. Now we check the condition
        testCandidate = numSubPartsRefMode > numSubPartsLimit;
      }
    }

    if (testCandidate)
    {
      modeInfo = candidate;
    }
  }
#if JVET_P1026_ISP_LFNST_COMBINATION
  else
  {
    //the end of the list was reached, so the split is invalidated
    ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
  }
#endif
}

#if JVET_P1026_ISP_LFNST_COMBINATION
void IntraSearch::xFindAlreadyTestedNearbyIntraModes(int lfnstIdx, int currentIntraMode, int* refLfnstIdx, int* leftIntraMode, int* rightIntraMode, ISPType ispOption, int windowSize)
#else
void IntraSearch::xFindAlreadyTestedNearbyIntraModes(int currentIntraMode, int* leftIntraMode, int* rightIntraMode, ISPType ispOption, int windowSize)
#endif
{
  bool leftModeFound = false, rightModeFound = false;
  *leftIntraMode = -1;
  *rightIntraMode = -1;
#if JVET_P1026_ISP_LFNST_COMBINATION
  *refLfnstIdx = -1;
#endif
  const unsigned st = ispOption - 1;

#if JVET_P1026_ISP_LFNST_COMBINATION
  //first we check if the exact intra mode was already tested for another lfnstIdx value
  if (lfnstIdx > 0)
  {
    bool sameIntraModeFound = false;
    if (lfnstIdx == 2 && m_ispTestedModes[1].modeHasBeenTested[currentIntraMode][st])
    {
      sameIntraModeFound = true;
      *refLfnstIdx = 1;
    }
    else if (m_ispTestedModes[0].modeHasBeenTested[currentIntraMode][st])
    {
      sameIntraModeFound = true;
      *refLfnstIdx = 0;
    }

    if (sameIntraModeFound)
    {
      *leftIntraMode = currentIntraMode;
      *rightIntraMode = currentIntraMode;
      return;
    }
  }

  //The mode has not been checked for another lfnstIdx value, so now we look for a similar mode within a window using the same lfnstIdx 
#endif
  for (int k = 1; k <= windowSize; k++)
  {
    int off = currentIntraMode - 2 - k;
    int leftMode = (off < 0) ? NUM_LUMA_MODE + off : currentIntraMode - k;
    int rightMode = currentIntraMode > DC_IDX ? (((int)currentIntraMode - 2 + k) % 65) + 2 : PLANAR_IDX;

#if JVET_P1026_ISP_LFNST_COMBINATION
    leftModeFound  = leftMode  != (int)currentIntraMode ? m_ispTestedModes[lfnstIdx].modeHasBeenTested[leftMode][st]  : false;
    rightModeFound = rightMode != (int)currentIntraMode ? m_ispTestedModes[lfnstIdx].modeHasBeenTested[rightMode][st] : false;
#else
    leftModeFound = leftMode != (int)currentIntraMode ? m_ispTestedModes.modeHasBeenTested[leftMode][st] : false;
    rightModeFound = rightMode != (int)currentIntraMode ? m_ispTestedModes.modeHasBeenTested[rightMode][st] : false;
#endif
    if (leftModeFound || rightModeFound)
    {
      *leftIntraMode = leftModeFound ? leftMode : -1;
      *rightIntraMode = rightModeFound ? rightMode : -1;
#if JVET_P1026_ISP_LFNST_COMBINATION
      *refLfnstIdx = lfnstIdx;
#endif
      break;
    }
  }
}

#if JVET_P1026_ISP_LFNST_COMBINATION
//It prepares the list of potential intra modes candidates that will be tested using RD costs
bool IntraSearch::xSortISPCandList(double bestCostSoFar, double bestNonISPCost, ModeInfo bestNonISPMode)
#else
void IntraSearch::xSortISPCandList(double bestCostSoFar, double bestNonISPCost)
#endif
{
#if JVET_P1026_ISP_LFNST_COMBINATION
  int bestISPModeInRelCU = -1;
  m_modeCtrl->setStopNonDCT2Transforms(false);

  if (m_pcEncCfg->getUseFastISP())
  {
    //we check if the ISP tests can be cancelled
    double thSkipISP = 1.4;
    if (bestNonISPCost > bestCostSoFar * thSkipISP)
    {
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
        {
          m_ispTestedModes[j].splitIsFinished[splitIdx] = true;
        }
      }
      return false;
    }
    if (!updateISPStatusFromRelCU(bestNonISPCost, bestNonISPMode, bestISPModeInRelCU))
    {
      return false;
    }
  }
#else
  if (m_pcEncCfg->getUseFastISP())
  {
    double thSkipISP = 1.4;
    if (bestNonISPCost > bestCostSoFar * thSkipISP)
    {
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        m_ispTestedModes.splitIsFinished[splitIdx] = true;
      }
      return;
    }
  }
#endif

  for (int k = 0; k < m_ispCandListHor.size(); k++)
  {
    m_ispCandListHor.at(k).ispMod = HOR_INTRA_SUBPARTITIONS; //we set the correct ISP split type value
  }

  auto origHadList = m_ispCandListHor;   // save the original hadamard list of regular intra
  bool modeIsInList[NUM_LUMA_MODE] = { false };

  m_ispCandListHor.clear();
  m_ispCandListVer.clear();

  // we sort the normal intra modes according to their full RD costs
  std::sort(m_regIntraRDListWithCosts.begin(), m_regIntraRDListWithCosts.end(), ModeInfoWithCost::compareModeInfoWithCost);

  // we get the best angle from the regular intra list
  int bestNormalIntraAngle = -1;
  for (int modeIdx = 0; modeIdx < m_regIntraRDListWithCosts.size(); modeIdx++)
  {
    if (bestNormalIntraAngle == -1 && m_regIntraRDListWithCosts.at(modeIdx).modeId > DC_IDX)
    {
      bestNormalIntraAngle = m_regIntraRDListWithCosts.at(modeIdx).modeId;
      break;
    }
  }

  int mode1 = PLANAR_IDX;
  int mode2 = bestNormalIntraAngle;

  ModeInfo refMode = origHadList.at(0);
  auto* destListPtr = &m_ispCandListHor;
  #if JVET_P1026_ISP_LFNST_COMBINATION
  //List creation 

  if (m_pcEncCfg->getUseFastISP() && bestISPModeInRelCU != -1) //RelCU intra mode
  {
#if JVET_P0803_COMBINED_MIP_CLEANUP
   destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, bestISPModeInRelCU));
#else
   destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, bestISPModeInRelCU));
#endif
    modeIsInList[bestISPModeInRelCU] = true;
  }

  // Planar
  if (!modeIsInList[mode1])
  {
#if JVET_P0803_COMBINED_MIP_CLEANUP
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode1));
#else
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, mode1));
#endif
    modeIsInList[mode1] = true;
  }
  // Best angle in regular intra
  if (mode2 != -1 && !modeIsInList[mode2])
  {
#if JVET_P0803_COMBINED_MIP_CLEANUP
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode2));
#else
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, mode2));
#endif
    modeIsInList[mode2] = true;
  }
  // Remaining regular intra modes that were full RD tested (except DC, which is added after the angles from regular intra)
  int dcModeIndex = -1;
  for (int remModeIdx = 0; remModeIdx < m_regIntraRDListWithCosts.size(); remModeIdx++)
  {
    int currentMode = m_regIntraRDListWithCosts.at(remModeIdx).modeId;
    if (currentMode != mode1 && currentMode != mode2 && !modeIsInList[currentMode])
    {
      if (currentMode > DC_IDX)
      {
#if JVET_P0803_COMBINED_MIP_CLEANUP
        destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, currentMode));
#else
        destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, currentMode));
#endif
        modeIsInList[currentMode] = true;
      }
      else if (currentMode == DC_IDX)
      {
        dcModeIndex = remModeIdx;
      }
    }
  }

  // DC is added after the angles from regular intra
  if (dcModeIndex != -1 && !modeIsInList[DC_IDX])
  {
#if JVET_P0803_COMBINED_MIP_CLEANUP
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, DC_IDX));
#else
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, DC_IDX));
#endif
    modeIsInList[DC_IDX] = true;
  }

  // We add extra candidates to the list that will only be tested if ISP is likely to win
  for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
  {
    m_ispTestedModes[j].numOrigModesToTest = (int)destListPtr->size();
  }
#else
  // 1) Planar
#if JVET_P0803_COMBINED_MIP_CLEANUP
  destListPtr->push_back( ModeInfo( refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode1 ) );
#else
  destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, mode1));
#endif
  modeIsInList[mode1] = true;
  // 2) Best angle in regular intra
  if (mode2 != -1)
  {
#if JVET_P0803_COMBINED_MIP_CLEANUP
    destListPtr->push_back( ModeInfo( refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode2 ) );
#else
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, mode2));
#endif
    modeIsInList[mode2] = true;
  }
  // 3) Remaining regular intra modes that were full RD tested (except DC, which is added after the angles from regular intra)
  int dcModeIndex = -1;
  for (int remModeIdx = 0; remModeIdx < m_regIntraRDListWithCosts.size(); remModeIdx++)
  {
    int currentMode = m_regIntraRDListWithCosts.at(remModeIdx).modeId;
    if (currentMode != mode1 && currentMode != mode2)
    {
      if (currentMode > DC_IDX)
      {
#if JVET_P0803_COMBINED_MIP_CLEANUP
        destListPtr->push_back( ModeInfo( refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, currentMode ) );
#else
        destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, currentMode));
#endif
        modeIsInList[currentMode] = true;
      }
      else if (currentMode == DC_IDX)
      {
        dcModeIndex = remModeIdx;
      }
    }
  }
  // 4) DC is added after the angles from regular intra
  if (dcModeIndex != -1)
  {
#if JVET_P0803_COMBINED_MIP_CLEANUP
    destListPtr->push_back( ModeInfo( refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, DC_IDX ) );
#else
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, DC_IDX));
#endif
    modeIsInList[DC_IDX] = true;
  }

  // 5) We add extra candidates to the list that will only be tested if ISP is likely to win
  m_ispTestedModes.numOrigModesToTest = (int)destListPtr->size();
#endif
  const int addedModesFromHadList = 3;
  int       newModesAdded = 0;

  for (int k = 0; k < origHadList.size(); k++)
  {
    if (newModesAdded == addedModesFromHadList)
    {
      break;
    }
    if (!modeIsInList[origHadList.at(k).modeId])
    {
#if JVET_P0803_COMBINED_MIP_CLEANUP
      destListPtr->push_back( ModeInfo( refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, origHadList.at(k).modeId ) );
#else
      destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mRefId, refMode.ispMod, origHadList.at(k).modeId));
#endif
      newModesAdded++;
    }
  }

#if JVET_P1026_ISP_LFNST_COMBINATION
  if (m_pcEncCfg->getUseFastISP() && bestISPModeInRelCU != -1)
  {
    destListPtr->resize(1);
  }
#endif

  // Copy modes to other split-type list
  m_ispCandListVer = m_ispCandListHor;
  for (int i = 0; i < m_ispCandListVer.size(); i++)
  {
    m_ispCandListVer[i].ispMod = VER_INTRA_SUBPARTITIONS;
  }

  // Reset the tested modes information to 0
#if JVET_P1026_ISP_LFNST_COMBINATION
  for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
  {
    for (int i = 0; i < m_ispCandListHor.size(); i++)
    {
      m_ispTestedModes[j].clearISPModeInfo(m_ispCandListHor[i].modeId);
    }
  }
  return true;
#else
  for (int i = 0; i < m_ispCandListHor.size(); i++)
  {
    m_ispTestedModes.clearISPModeInfo(m_ispCandListHor[i].modeId);
  }
#endif
}

#if JVET_P1026_ISP_LFNST_COMBINATION
void IntraSearch::xSortISPCandListLFNST()
{
  //It resorts the list of intra mode candidates for lfnstIdx > 0 by checking the RD costs for lfnstIdx = 0
  ISPTestedModesInfo& ispTestedModesRef = m_ispTestedModes[0];
  for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
  {
    ISPType ispMode = splitIdx ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
    if (!m_ispTestedModes[m_curIspLfnstIdx].splitIsFinished[splitIdx] && ispTestedModesRef.testedModes[splitIdx].size() > 1)
    {
      auto& candList   = ispMode == HOR_INTRA_SUBPARTITIONS ? m_ispCandListHor : m_ispCandListVer;
      int bestModeId   = candList[1].modeId > DC_IDX ? candList[1].modeId : -1;
      int bestSubParts = candList[1].modeId > DC_IDX ? ispTestedModesRef.getNumCompletedSubParts(ispMode, bestModeId) : -1;
      double bestCost  = candList[1].modeId > DC_IDX ? ispTestedModesRef.getRDCost(ispMode, bestModeId) : MAX_DOUBLE;
      for (int i = 0; i < candList.size(); i++)
      {
        const int candSubParts = ispTestedModesRef.getNumCompletedSubParts(ispMode, candList[i].modeId);
        const double candCost = ispTestedModesRef.getRDCost(ispMode, candList[i].modeId);
        if (candSubParts > bestSubParts || candCost < bestCost)
        {
          bestModeId = candList[i].modeId;
          bestCost = candCost;
          bestSubParts = candSubParts;
        }
      }

      if (bestModeId != -1)
      {
        if (bestModeId != candList[0].modeId)
        {
          auto prevMode = candList[0];
          candList[0].modeId = bestModeId;
          for (int i = 1; i < candList.size(); i++)
          {
            auto nextMode = candList[i];
            candList[i] = prevMode;
            if (nextMode.modeId == bestModeId)
            {
              break;
            }
            prevMode = nextMode;
          }
        }
      }
    }
  }
}

bool IntraSearch::updateISPStatusFromRelCU( double bestNonISPCostCurrCu, ModeInfo bestNonISPModeCurrCu, int& bestISPModeInRelCU )
{
  //It compares the data of a related CU with the current CU to cancel or reduce the ISP tests
  bestISPModeInRelCU = -1;
  if (m_modeCtrl->getRelatedCuIsValid())
  {
    double bestNonISPCostRelCU = m_modeCtrl->getBestDCT2NonISPCostRelCU();
    double costRatio           = bestNonISPCostCurrCu / bestNonISPCostRelCU;
    bool   bestModeRelCuIsMip  = (m_modeCtrl->getIspPredModeValRelCU() >> 5) & 0x1;
    bool   bestModeCurrCuIsMip = bestNonISPModeCurrCu.mipFlg;
    int    relatedCuIntraMode  = m_modeCtrl->getIspPredModeValRelCU() >> 9;
    bool   isSameTypeOfMode    = (bestModeRelCuIsMip && bestModeCurrCuIsMip) || (!bestModeRelCuIsMip && !bestModeCurrCuIsMip);
    bool   bothModesAreAngular = bestNonISPModeCurrCu.modeId > DC_IDX && relatedCuIntraMode > DC_IDX;
    bool   modesAreComparable  = isSameTypeOfMode && (bestModeCurrCuIsMip || bestNonISPModeCurrCu.modeId == relatedCuIntraMode || (bothModesAreAngular && abs(relatedCuIntraMode - (int)bestNonISPModeCurrCu.modeId) <= 5));
    int    status              = m_modeCtrl->getIspPredModeValRelCU();

    if ((status & 0x3) == 0x3) //ISP was not selected in the relCU
    {
      double bestNonDCT2Cost = m_modeCtrl->getBestNonDCT2Cost();
      double ratioWithNonDCT2 = bestNonDCT2Cost / bestNonISPCostRelCU;
      double margin = ratioWithNonDCT2 < 0.95 ? 0.2 : 0.1;

      if (costRatio > 1 - margin && costRatio < 1 + margin && modesAreComparable)
      {
        for (int lfnstVal = 0; lfnstVal < NUM_LFNST_NUM_PER_SET; lfnstVal++)
        {
          m_ispTestedModes[lfnstVal].splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1] = true;
          m_ispTestedModes[lfnstVal].splitIsFinished[VER_INTRA_SUBPARTITIONS - 1] = true;
        }
        return false;
      }
    }
    else if ((status & 0x3) == 0x1) //ISP was selected in the relCU
    {
      double margin = 0.05;

      if (costRatio > 1 - margin && costRatio < 1 + margin && modesAreComparable)
      {
        int  ispSplitIdx = (m_modeCtrl->getIspPredModeValRelCU() >> 2) & 0x1;
        bool lfnstIdxIsNot0 = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 3) & 0x1);
        bool lfnstIdxIs2 = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 4) & 0x1);
        int  lfnstIdx = !lfnstIdxIsNot0 ? 0 : lfnstIdxIs2 ? 2 : 1;
        bestISPModeInRelCU = (int)m_modeCtrl->getBestISPIntraModeRelCU();

        for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
        {
          for (int lfnstVal = 0; lfnstVal < NUM_LFNST_NUM_PER_SET; lfnstVal++)
          {
            if (lfnstVal == lfnstIdx && splitIdx == ispSplitIdx)
            {
              continue;
            }
            m_ispTestedModes[lfnstVal].splitIsFinished[splitIdx] = true;
          }
        }

        bool stopNonDCT2Transforms = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 6) & 0x1);
        m_modeCtrl->setStopNonDCT2Transforms(stopNonDCT2Transforms);
      }
    }
    else
    {
      THROW("Wrong ISP relCU status");
    }
  }

  return true;
}

void IntraSearch::xFinishISPModes()
{
  //Continue to the next lfnst index 
  m_curIspLfnstIdx++;

  if (m_curIspLfnstIdx < NUM_LFNST_NUM_PER_SET)
  {
    //Check if LFNST is applicable
    if (m_curIspLfnstIdx == 1)
    {
      bool canTestLFNST = false;
      for (int lfnstIdx = 1; lfnstIdx < NUM_LFNST_NUM_PER_SET; lfnstIdx++)
      {
        canTestLFNST |= !m_ispTestedModes[lfnstIdx].splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1] || !m_ispTestedModes[lfnstIdx].splitIsFinished[VER_INTRA_SUBPARTITIONS - 1];
      }
      if (canTestLFNST)
      {
        //Construct the intra modes candidates list for the lfnst > 0 cases
        xSortISPCandListLFNST();
      }
    }
  }
}
#endif

