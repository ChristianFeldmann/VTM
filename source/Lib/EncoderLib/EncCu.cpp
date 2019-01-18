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

/** \file     EncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include "EncCu.h"

#include "EncLib.h"
#include "Analyze.h"
#include "AQp.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"


#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>
#if ENABLE_WPP_PARALLELISM
#include <mutex>
extern std::recursive_mutex g_cache_mutex;
#endif



//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

void EncCu::create( EncCfg* encCfg )
{
  unsigned      uiMaxWidth    = encCfg->getMaxCUWidth();
  unsigned      uiMaxHeight   = encCfg->getMaxCUHeight();
  ChromaFormat  chromaFormat  = encCfg->getChromaFormatIdc();

  unsigned      numWidths     = gp_sizeIdxInfo->numWidths();
  unsigned      numHeights    = gp_sizeIdxInfo->numHeights();
  m_pTempCS = new CodingStructure**  [numWidths];
  m_pBestCS = new CodingStructure**  [numWidths];

  m_pTempMotLUTs = new LutMotionCand**[numWidths];
  m_pBestMotLUTs = new LutMotionCand**[numWidths];
  m_pSplitTempMotLUTs = new LutMotionCand**[numWidths];

  for( unsigned w = 0; w < numWidths; w++ )
  {
    m_pTempCS[w] = new CodingStructure*  [numHeights];
    m_pBestCS[w] = new CodingStructure*  [numHeights];
    m_pTempMotLUTs[w] = new LutMotionCand*[numHeights];
    m_pBestMotLUTs[w] = new LutMotionCand*[numHeights];
    m_pSplitTempMotLUTs[w] = new LutMotionCand*[numHeights];

    for( unsigned h = 0; h < numHeights; h++ )
    {
      unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
      unsigned height = gp_sizeIdxInfo->sizeFrom( h );

      if( gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height ) )
      {
        m_pTempCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pBestCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pTempCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
        m_pBestCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
        m_pTempMotLUTs[w][h] = new LutMotionCand ;
        m_pBestMotLUTs[w][h] = new LutMotionCand ;
        m_pSplitTempMotLUTs[w][h] = new LutMotionCand;
        m_pSplitTempMotLUTs[w][h]->currCnt = 0;
        m_pSplitTempMotLUTs[w][h]->motionCand = nullptr;
        m_pSplitTempMotLUTs[w][h]->motionCand = new MotionInfo[MAX_NUM_HMVP_CANDS];

        m_pTempMotLUTs[w][h]->currCnt = 0;
        m_pTempMotLUTs[w][h]->motionCand = nullptr;
        m_pTempMotLUTs[w][h]->motionCand = new MotionInfo[MAX_NUM_HMVP_CANDS];

        m_pBestMotLUTs[w][h]->currCnt = 0;
        m_pBestMotLUTs[w][h]->motionCand = nullptr;
        m_pBestMotLUTs[w][h]->motionCand = new MotionInfo[MAX_NUM_HMVP_CANDS];
      }
      else
      {
        m_pTempCS[w][h] = nullptr;
        m_pBestCS[w][h] = nullptr;
        m_pTempMotLUTs[w][h] = nullptr;
        m_pBestMotLUTs[w][h] = nullptr;
        m_pSplitTempMotLUTs[w][h] = nullptr;
      }
    }
  }

  // WIA: only the weight==height case is relevant without QTBT
  m_pImvTempCS = nullptr;

  m_cuChromaQpOffsetIdxPlus1 = 0;

  unsigned maxDepth = numWidths + numHeights;

  m_modeCtrl = new EncModeCtrlMTnoRQT();

#if REUSE_CU_RESULTS
  m_modeCtrl->create( *encCfg );

#endif
  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
  {
    m_acMergeBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }
  for( unsigned ui = 0; ui < TRIANGLE_MAX_NUM_CANDS; ui++ )
  {
    m_acTriangleWeightedBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }

  m_CtxBuffer.resize( maxDepth );
  m_CurrCtx = 0;
}


void EncCu::destroy()
{
  unsigned numWidths  = gp_sizeIdxInfo->numWidths();
  unsigned numHeights = gp_sizeIdxInfo->numHeights();

  for( unsigned w = 0; w < numWidths; w++ )
  {
    for( unsigned h = 0; h < numHeights; h++ )
    {
      if( m_pBestCS[w][h] ) m_pBestCS[w][h]->destroy();
      if( m_pTempCS[w][h] ) m_pTempCS[w][h]->destroy();

      delete m_pBestCS[w][h];
      delete m_pTempCS[w][h];
      if (m_pTempMotLUTs[w][h])
      {
        delete[] m_pTempMotLUTs[w][h]->motionCand;
        m_pTempMotLUTs[w][h]->motionCand = nullptr;
        delete m_pTempMotLUTs[w][h];
      }
      if (m_pBestMotLUTs[w][h])
      {
        delete[] m_pBestMotLUTs[w][h]->motionCand;
        m_pBestMotLUTs[w][h]->motionCand = nullptr;
        delete m_pBestMotLUTs[w][h];
      }

      if (m_pSplitTempMotLUTs[w][h])
      {
        delete[] m_pSplitTempMotLUTs[w][h]->motionCand;
        m_pSplitTempMotLUTs[w][h]->motionCand = nullptr;
        delete m_pSplitTempMotLUTs[w][h];
      }
    }

    delete[] m_pTempCS[w];
    delete[] m_pBestCS[w];
    delete[] m_pBestMotLUTs[w];
    delete[] m_pTempMotLUTs[w];
    delete[] m_pSplitTempMotLUTs[w];
  }

  delete[] m_pBestCS; m_pBestCS = nullptr;
  delete[] m_pTempCS; m_pTempCS = nullptr;
  delete[] m_pSplitTempMotLUTs; m_pSplitTempMotLUTs = nullptr;
  delete[] m_pBestMotLUTs; m_pBestMotLUTs = nullptr;
  delete[] m_pTempMotLUTs; m_pTempMotLUTs = nullptr;

#if REUSE_CU_RESULTS
  m_modeCtrl->destroy();

#endif
  delete m_modeCtrl;
  m_modeCtrl = nullptr;

  // WIA: only the weight==height case is relevant without QTBT
  if( m_pImvTempCS )
  {
    for( unsigned w = 0; w < numWidths; w++ )
    {
      if( m_pImvTempCS[w] )
      {
        m_pImvTempCS[w]->destroy();
        delete[] m_pImvTempCS[w];
      }
    }

    delete[] m_pImvTempCS;
    m_pImvTempCS = nullptr;
  }

  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
  {
    m_acMergeBuffer[ui].destroy();
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].destroy();
  }
  for( unsigned ui = 0; ui < TRIANGLE_MAX_NUM_CANDS; ui++ )
  {
    m_acTriangleWeightedBuffer[ui].destroy();
  }
}



EncCu::~EncCu()
{
}



/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init( EncLib* pcEncLib, const SPS& sps PARL_PARAM( const int tId ) )
{
  m_pcEncCfg           = pcEncLib;
  m_pcIntraSearch      = pcEncLib->getIntraSearch( PARL_PARAM0( tId ) );
  m_pcInterSearch      = pcEncLib->getInterSearch( PARL_PARAM0( tId ) );
  m_pcTrQuant          = pcEncLib->getTrQuant( PARL_PARAM0( tId ) );
  m_pcRdCost           = pcEncLib->getRdCost ( PARL_PARAM0( tId ) );
  m_CABACEstimator     = pcEncLib->getCABACEncoder( PARL_PARAM0( tId ) )->getCABACEstimator( &sps );
  m_CABACEstimator->setEncCu(this);
  m_CtxCache           = pcEncLib->getCtxCache( PARL_PARAM0( tId ) );
  m_pcRateCtrl         = pcEncLib->getRateCtrl();
  m_pcSliceEncoder     = pcEncLib->getSliceEncoder();
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  m_pcEncLib           = pcEncLib;
  m_dataId             = tId;
#endif

#if REUSE_CU_RESULTS
  DecCu::init( m_pcTrQuant, m_pcIntraSearch, m_pcInterSearch );

#endif
  m_modeCtrl->init( m_pcEncCfg, m_pcRateCtrl, m_pcRdCost );

  m_pcInterSearch->setModeCtrl( m_modeCtrl );
  ::memset(m_subMergeBlkSize, 0, sizeof(m_subMergeBlkSize));
  ::memset(m_subMergeBlkNum, 0, sizeof(m_subMergeBlkNum));
  m_prevPOC = MAX_UINT;
#if !JVET_L0198_L0468_L0104_ATMVP_8x8SUB_BLOCK
  m_clearSubMergeStatic = false;
#endif

#if JVET_L0293_CPR
  if (m_pcEncCfg->getCPRHashSearch() && m_pcEncCfg->getCPRMode())
  {
    m_cprHashMap.init(m_pcEncCfg->getSourceWidth(), m_pcEncCfg->getSourceHeight());
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] )
{
#if JVET_L0293_CPR
  if (m_pcEncCfg->getCPRHashSearch() && ctuRsAddr == 0 && cs.slice->getSPS()->getSpsNext().getCPRMode())
  {
    m_cprHashMap.rebuildPicHashMap(cs.picture->getOrigBuf());
  }
#endif
  m_modeCtrl->initCTUEncoding( *cs.slice );

#if ENABLE_SPLIT_PARALLELISM
  if( m_pcEncCfg->getNumSplitThreads() > 1 )
  {
    for( int jId = 1; jId < NUM_RESERVERD_SPLIT_JOBS; jId++ )
    {
      EncCu*            jobEncCu  = m_pcEncLib->getCuEncoder( cs.picture->scheduler.getSplitDataId( jId ) );
      CacheBlkInfoCtrl* cacheCtrl = dynamic_cast< CacheBlkInfoCtrl* >( jobEncCu->m_modeCtrl );
      if( cacheCtrl )
      {
        cacheCtrl->init( *cs.slice );
      }
    }
  }

  if( auto* cacheCtrl = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) ) { cacheCtrl->tick(); }
#endif
  // init the partitioning manager
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );
  partitioner->initCtu( area, CH_L, *cs.slice );
#if JVET_L0293_CPR
  if (m_pcEncCfg->getCPRMode())
  {
    m_pcInterSearch->resetCtuRecord();
    m_ctuCprSearchRangeX = m_pcEncCfg->getCPRLocalSearchRangeX();
    m_ctuCprSearchRangeY = m_pcEncCfg->getCPRLocalSearchRangeY();
  }
  if (m_pcEncCfg->getCPRMode() && m_pcEncCfg->getCPRHashSearch() && (m_pcEncCfg->getCPRFastMethod() & CPR_FAST_METHOD_ADAPTIVE_SEARCHRANGE))
  {
    const int hashHitRatio = m_cprHashMap.getHashHitRatio(area.Y()); // in percent
    if (hashHitRatio < 5) // 5%
    {
      m_ctuCprSearchRangeX >>= 1;
      m_ctuCprSearchRangeY >>= 1;
    }
    if (cs.slice->getNumRefIdx(REF_PIC_LIST_0) > 1)
    {
      m_ctuCprSearchRangeX >>= 1;
      m_ctuCprSearchRangeY >>= 1;
    }
  }
#endif //CPR
  // init current context pointer
  m_CurrCtx = m_CtxBuffer.data();

  CodingStructure *tempCS = m_pTempCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];
  CodingStructure *bestCS = m_pBestCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];
  LutMotionCand *tempMotCandLUTs = m_pTempMotLUTs[gp_sizeIdxInfo->idxFrom(area.lumaSize().width)][gp_sizeIdxInfo->idxFrom(area.lumaSize().height)];
  LutMotionCand *bestMotCandLUTs = m_pBestMotLUTs[gp_sizeIdxInfo->idxFrom(area.lumaSize().width)][gp_sizeIdxInfo->idxFrom(area.lumaSize().height)];
  cs.slice->copyMotionLUTs(cs.slice->getMotionLUTs(), tempMotCandLUTs);
  cs.slice->copyMotionLUTs(cs.slice->getMotionLUTs(), bestMotCandLUTs);

  cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false );
  cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false );
  tempCS->currQP[CH_L] = bestCS->currQP[CH_L] =
  tempCS->baseQP       = bestCS->baseQP       = currQP[CH_L];
  tempCS->prevQP[CH_L] = bestCS->prevQP[CH_L] = prevQP[CH_L];

  xCompressCU( tempCS, bestCS, *partitioner
    , tempMotCandLUTs
    , bestMotCandLUTs
  );

  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
  cs.useSubStructure( *bestCS, partitioner->chType, CS::getArea( *bestCS, area, partitioner->chType ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );
  cs.slice->copyMotionLUTs(bestMotCandLUTs, cs.slice->getMotionLUTs());

  if( !cs.pcv->ISingleTree && cs.slice->isIRAP() && cs.pcv->chrFormat != CHROMA_400 )
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner->initCtu( area, CH_C, *cs.slice );

    cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false );
    cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false );
    tempCS->currQP[CH_C] = bestCS->currQP[CH_C] =
    tempCS->baseQP       = bestCS->baseQP       = currQP[CH_C];
    tempCS->prevQP[CH_C] = bestCS->prevQP[CH_C] = prevQP[CH_C];

    xCompressCU( tempCS, bestCS, *partitioner
      , tempMotCandLUTs
      , bestMotCandLUTs
    );

    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
    cs.useSubStructure( *bestCS, partitioner->chType, CS::getArea( *bestCS, area, partitioner->chType ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );
  }

  if (m_pcEncCfg->getUseRateCtrl())
  {
    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_actualMSE = (double)bestCS->dist / (double)m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr).m_numberOfPixel;
  }
  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;
  delete partitioner;

#if ENABLE_SPLIT_PARALLELISM && ENABLE_WPP_PARALLELISM
  if( m_pcEncCfg->getNumSplitThreads() > 1 && m_pcEncCfg->getNumWppThreads() > 1 )
  {
    cs.picture->finishCtuPart( area );
  }
#endif

  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

static int xCalcHADs8x8_ISlice(const Pel *piOrg, const int iStrideOrg)
{
  int k, i, j, jj;
  int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for (k = 0; k < 64; k += 8)
  {
    diff[k + 0] = piOrg[0];
    diff[k + 1] = piOrg[1];
    diff[k + 2] = piOrg[2];
    diff[k + 3] = piOrg[3];
    diff[k + 4] = piOrg[4];
    diff[k + 5] = piOrg[5];
    diff[k + 6] = piOrg[6];
    diff[k + 7] = piOrg[7];

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j = 0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj    ] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj    ] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i = 0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad = (iSumHad + 2) >> 2;
  return(iSumHad);
}

int  EncCu::updateCtuDataISlice(const CPelBuf buf)
{
  int  xBl, yBl;
  const int iBlkSize = 8;
  const Pel* pOrgInit = buf.buf;
  int  iStrideOrig = buf.stride;

  int iSumHad = 0;
  for( yBl = 0; ( yBl + iBlkSize ) <= buf.height; yBl += iBlkSize )
  {
    for( xBl = 0; ( xBl + iBlkSize ) <= buf.width; xBl += iBlkSize )
    {
      const Pel* pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice( pOrg, iStrideOrig );
    }
  }
  return( iSumHad );
}

bool EncCu::xCheckBestMode( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  bool bestCSUpdated = false;

  if( !tempCS->cus.empty() )
  {
    if( tempCS->cus.size() == 1 )
    {
      const CodingUnit& cu = *tempCS->cus.front();
      CHECK( cu.skip && !cu.firstPU->mergeFlag, "Skip flag without a merge flag is not allowed!" );
    }

#if WCG_EXT
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda() );
#endif

    if( m_modeCtrl->useModeResult( encTestMode, tempCS, partitioner ) )
    {
      if( tempCS->cus.size() == 1 )
      {
        // if tempCS is not a split-mode
        CodingUnit &cu = *tempCS->cus.front();

        if( CU::isLosslessCoded( cu ) && !cu.ipcm )
        {
          xFillPCMBuffer( cu );
        }
      }

      std::swap( tempCS, bestCS );
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
      bestCSUpdated = true;
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  return bestCSUpdated;

}

void EncCu::xCompressCU( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner
  , LutMotionCand *&tempMotCandLUTs
  , LutMotionCand *&bestMotCandLUTs
)
{
#if ENABLE_SPLIT_PARALLELISM
  CHECK( m_dataId != tempCS->picture->scheduler.getDataId(), "Working in the wrong dataId!" );

  if( m_pcEncCfg->getNumSplitThreads() != 1 && tempCS->picture->scheduler.getSplitJobId() == 0 )
  {
    if( m_modeCtrl->isParallelSplit( *tempCS, partitioner ) )
    {
      m_modeCtrl->setParallelSplit( true );
      xCompressCUParallel( tempCS, bestCS, partitioner );
      return;
    }
  }

#endif

  Slice&   slice      = *tempCS->slice;
  const PPS &pps      = *tempCS->pps;
  const SPS &sps      = *tempCS->sps;
  const uint32_t uiLPelX  = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY  = tempCS->area.Y().lumaPos().y;

  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth()  );

  const UnitArea currCsArea = clipArea( CS::getArea( *bestCS, bestCS->area, partitioner.chType ), *tempCS->picture );
  if( m_pImvTempCS && !slice.isIntra() )
  {
    tempCS->initSubStructure( *m_pImvTempCS[wIdx], partitioner.chType, partitioner.currArea(), false );
  }

#if JVET_L0293_CPR
  tempCS->chType = partitioner.chType;
  bestCS->chType = partitioner.chType;
#endif
  m_modeCtrl->initCULevel( partitioner, *tempCS );

  m_CurrCtx->start = m_CABACEstimator->getCtx();

  m_cuChromaQpOffsetIdxPlus1 = 0;

  if( slice.getUseChromaQpAdj() )
  {
    int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
      std::max<int>( 0, sps.getLog2DiffMaxMinCodingBlockSize() - int( pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() ) );
    m_cuChromaQpOffsetIdxPlus1 = ( ( uiLPelX >> lgMinCuSize ) + ( uiTPelY >> lgMinCuSize ) ) % ( pps.getPpsRangeExtension().getChromaQpOffsetListLen() + 1 );
  }

  if( !m_modeCtrl->anyMode() )
  {
    m_modeCtrl->finishCULevel( partitioner );
    return;
  }
  if (!slice.isIntra()
#if JVET_L0293_CPR
    && tempCS->chType == CHANNEL_TYPE_LUMA
#endif
    )
  {
    tempCS->slice->copyMotionLUTs(tempMotCandLUTs, tempCS->slice->getMotionLUTs());
  }

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cux", uiLPelX ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuy", uiTPelY ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuw", tempCS->area.lwidth() ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuh", tempCS->area.lheight() ) );
  DTRACE( g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight() );

  do
  {
    EncTestMode currTestMode = m_modeCtrl->currTestMode();

#if JVET_L0428_DQP_SEP_TREE
    if (tempCS->pps->getUseDQP() && CS::isDualITree(*tempCS) && isChroma(partitioner.chType))
    {
      const Position chromaCentral(tempCS->area.Cb().chromaPos().offset(tempCS->area.Cb().chromaSize().width >> 1, tempCS->area.Cb().chromaSize().height >> 1));
      const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, tempCS->area.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, tempCS->area.chromaFormat));
      const CodingStructure* baseCS = bestCS->picture->cs;
      const CodingUnit* colLumaCu = baseCS->getCU(lumaRefPos, CHANNEL_TYPE_LUMA);
      const TransformUnit*  tu = baseCS->getTU(lumaRefPos, CHANNEL_TYPE_LUMA);

      if (colLumaCu && tu)
      {
        currTestMode.qp = colLumaCu->qp;
      }
    }
#endif

#if SHARP_LUMA_DELTA_QP
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() && partitioner.currDepth <= pps.getMaxCuDQPDepth() )
    {
#if ENABLE_SPLIT_PARALLELISM
      CHECK( tempCS->picture->scheduler.getSplitJobId() > 0, "Changing lambda is only allowed in the master thread!" );
#endif
      if (currTestMode.qp >= 0)
      {
        updateLambda(&slice, currTestMode.qp);
      }
    }
#endif

    if( currTestMode.type == ETM_INTER_ME )
    {
      if( ( currTestMode.opts & ETO_IMV ) != 0 )
      {
        xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode);
      }
      else
      {
        xCheckRDCostInter( tempCS, bestCS, partitioner, currTestMode );
      }

    }
    else if( currTestMode.type == ETM_AFFINE )
    {
      xCheckRDCostAffineMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
#if REUSE_CU_RESULTS
    else if( currTestMode.type == ETM_RECO_CACHED )
    {
      xReuseCachedResult( tempCS, bestCS, partitioner );
    }
#endif
    else if( currTestMode.type == ETM_MERGE_SKIP )
    {
      xCheckRDCostMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
      CodingUnit* cu = bestCS->getCU(partitioner.chType);
#if JVET_L0293_CPR
      if (cu)
#endif
      cu->mmvdSkip = cu->skip == false ? false : cu->mmvdSkip;
    }
    else if( currTestMode.type == ETM_MERGE_TRIANGLE )
    {
      xCheckRDCostMergeTriangle2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( currTestMode.type == ETM_INTRA )
    {
      xCheckRDCostIntra( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( currTestMode.type == ETM_IPCM )
    {
      xCheckIntraPCM( tempCS, bestCS, partitioner, currTestMode );
    }
#if JVET_L0293_CPR
    else if (currTestMode.type == ETM_CPR)
    {
      xCheckRDCostCPRMode(tempCS, bestCS, partitioner, currTestMode);
    }
    else if (currTestMode.type == ETM_CPR_MERGE)
    {
      xCheckRDCostCPRModeMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
    }
#endif // CPR
    else if( isModeSplit( currTestMode ) )
    {

      xCheckModeSplit( tempCS, bestCS, partitioner, currTestMode
        , tempMotCandLUTs
        , bestMotCandLUTs
        , partitioner.currArea()
      );
    }
    else
    {
      THROW( "Don't know how to handle mode: type = " << currTestMode.type << ", options = " << currTestMode.opts );
    }
  } while( m_modeCtrl->nextMode( *tempCS, partitioner ) );

  //////////////////////////////////////////////////////////////////////////
  // Finishing CU
#if ENABLE_SPLIT_PARALLELISM
  if( bestCS->cus.empty() )
  {
    CHECK( bestCS->cost != MAX_DOUBLE, "Cost should be maximal if no encoding found" );
    CHECK( bestCS->picture->scheduler.getSplitJobId() == 0, "Should always get a result in serial case" );

    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

#endif
  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
  bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;
  if (!slice.isIntra() 
#if JVET_L0293_CPR
    && bestCS->chType == CHANNEL_TYPE_LUMA
#endif
    && bestCS->cus.size() == 1 && bestCS->cus.back()->predMode == MODE_INTER 
#if JVET_L0293_CPR
    && bestCS->area.Y() == (*bestCS->cus.back()).Y()
#else
    && bestCS->area == *bestCS->cus.back()
#endif
    )
  {
    bestCS->slice->updateMotionLUTs(bestMotCandLUTs, (*bestCS->cus.back()));
  }
  bestCS->picture->getRecoBuf( currCsArea ).copyFrom( bestCS->getRecoBuf( currCsArea ) );
  m_modeCtrl->finishCULevel( partitioner );

#if ENABLE_SPLIT_PARALLELISM
  if( tempCS->picture->scheduler.getSplitJobId() == 0 && m_pcEncCfg->getNumSplitThreads() != 1 )
  {
    tempCS->picture->finishParallelPart( currCsArea );
  }

#endif
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

#if SHARP_LUMA_DELTA_QP
void EncCu::updateLambda( Slice* slice, double dQP )
{
#if WCG_EXT
  int    NumberBFrames = ( m_pcEncCfg->getGOPSize() - 1 );
  int    SHIFT_QP = 12;
  double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)(slice->getPic()->fieldPic ? NumberBFrames/2 : NumberBFrames) );

  int bitdepth_luma_qp_scale = 6
                               * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                                  - DISTORTION_PRECISION_ADJUSTMENT(slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
  double qp_temp = (double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;

  double dQPFactor = m_pcEncCfg->getGOPEntry( m_pcSliceEncoder->getGopId() ).m_QPFactor;

  if( slice->getSliceType() == I_SLICE )
  {
    if( m_pcEncCfg->getIntraQpFactor() >= 0.0 /*&& m_pcEncCfg->getGOPEntry( m_pcSliceEncoder->getGopId() ).m_sliceType != I_SLICE*/ )
    {
      dQPFactor = m_pcEncCfg->getIntraQpFactor();
    }
    else
    {
      if( m_pcEncCfg->getLambdaFromQPEnable() )
      {
        dQPFactor = 0.57;
      }
      else
      {
        dQPFactor = 0.57*dLambda_scale;
      }
    }
  }
  else if( m_pcEncCfg->getLambdaFromQPEnable() )
  {
    dQPFactor = 0.57*dQPFactor;
  }

  double dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
  int depth = slice->getDepth();

  if( !m_pcEncCfg->getLambdaFromQPEnable() && depth>0 )
  {
    int qp_temp_slice = slice->getSliceQp() + bitdepth_luma_qp_scale - SHIFT_QP; // avoid lambda  over adjustment,  use slice_qp here
    dLambda *= Clip3( 2.00, 4.00, (qp_temp_slice / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
  }
  if( !m_pcEncCfg->getUseHADME() && slice->getSliceType( ) != I_SLICE )
  {
    dLambda *= 0.95;
  }

  const int temporalId = m_pcEncCfg->getGOPEntry( m_pcSliceEncoder->getGopId() ).m_temporalId;
  const std::vector<double> &intraLambdaModifiers = m_pcEncCfg->getIntraLambdaModifier();
  double lambdaModifier;
  if( slice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcEncCfg->getLambdaModifier(temporalId);
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[(temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size() - 1)];
  }
  dLambda *= lambdaModifier;

  int qpBDoffset = slice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  int iQP = Clip3(-qpBDoffset, MAX_QP, (int)floor(dQP + 0.5));
  m_pcSliceEncoder->setUpLambda(slice, dLambda, iQP);

#else
  int iQP = (int)dQP;
  const double oldQP     = (double)slice->getSliceQpBase();
  const double oldLambda = m_pcSliceEncoder->calculateLambda (slice, m_pcSliceEncoder->getGopId(), slice->getDepth(), oldQP, oldQP, iQP);
  const double newLambda = oldLambda * pow (2.0, (dQP - oldQP) / 3.0);
#if RDOQ_CHROMA_LAMBDA
  const double chromaLambda = newLambda / m_pcRdCost->getChromaWeight();
  const double lambdaArray[MAX_NUM_COMPONENT] = {newLambda, chromaLambda, chromaLambda};
  m_pcTrQuant->setLambdas (lambdaArray);
#else
  m_pcTrQuant->setLambda (newLambda);
#endif
  m_pcRdCost->setLambda( newLambda, slice->getSPS()->getBitDepths() );
#endif
}
#endif

#if ENABLE_SPLIT_PARALLELISM
//#undef DEBUG_PARALLEL_TIMINGS
//#define DEBUG_PARALLEL_TIMINGS 1
void EncCu::xCompressCUParallel( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth() );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  Picture* picture = tempCS->picture;

  int numJobs = m_modeCtrl->getNumParallelJobs( *bestCS, partitioner );

  bool    jobUsed                            [NUM_RESERVERD_SPLIT_JOBS];
  std::fill( jobUsed, jobUsed + NUM_RESERVERD_SPLIT_JOBS, false );

  const UnitArea currArea = CS::getArea( *tempCS, partitioner.currArea(), partitioner.chType );
#if ENABLE_WPP_PARALLELISM
  const int      wppTId   = picture->scheduler.getWppThreadId();
#endif
  const bool doParallel   = !m_pcEncCfg->getForceSingleSplitThread();
#if _MSC_VER && ENABLE_WPP_PARALLELISM
#pragma omp parallel for schedule(dynamic,1) num_threads(NUM_SPLIT_THREADS_IF_MSVC) if(doParallel)
#else
  omp_set_num_threads( m_pcEncCfg->getNumSplitThreads() );

#pragma omp parallel for schedule(dynamic,1) if(doParallel)
#endif
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    // thread start
#if ENABLE_WPP_PARALLELISM
    picture->scheduler.setWppThreadId( wppTId );
#endif
    picture->scheduler.setSplitThreadId();
    picture->scheduler.setSplitJobId( jId );

    Partitioner* jobPartitioner = PartitionerFactory::get( *tempCS->slice );
    EncCu*       jobCuEnc       = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );
    auto*        jobBlkCache    = dynamic_cast<CacheBlkInfoCtrl*>( jobCuEnc->m_modeCtrl );

    jobPartitioner->copyState( partitioner );
    jobCuEnc      ->copyState( this, *jobPartitioner, currArea, true );

    if( jobBlkCache )
    {
      jobBlkCache->tick();
    }

    CodingStructure *&jobBest = jobCuEnc->m_pBestCS[wIdx][hIdx];
    CodingStructure *&jobTemp = jobCuEnc->m_pTempCS[wIdx][hIdx];

    jobUsed[jId] = true;

    jobCuEnc->xCompressCU( jobTemp, jobBest, *jobPartitioner );

    delete jobPartitioner;

    picture->scheduler.setSplitJobId( 0 );
    // thread stop
  }
  picture->scheduler.setSplitThreadId( 0 );

  int    bestJId  = 0;
  double bestCost = bestCS->cost;
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    EncCu* jobCuEnc = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );

    if( jobUsed[jId] && jobCuEnc->m_pBestCS[wIdx][hIdx]->cost < bestCost )
    {
      bestCost = jobCuEnc->m_pBestCS[wIdx][hIdx]->cost;
      bestJId  = jId;
    }
  }

  if( bestJId > 0 )
  {
    copyState( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( bestJId ) ), partitioner, currArea, false );
    m_CurrCtx->best = m_CABACEstimator->getCtx();

    tempCS = m_pTempCS[wIdx][hIdx];
    bestCS = m_pBestCS[wIdx][hIdx];
  }

  const int      bitDepthY = tempCS->sps->getBitDepth( CH_L );
  const UnitArea clipdArea = clipArea( currArea, *picture );

  CHECK( calcCheckSum( picture->getRecoBuf( clipdArea.Y() ), bitDepthY ) != calcCheckSum( bestCS->getRecoBuf( clipdArea.Y() ), bitDepthY ), "Data copied incorrectly!" );

  picture->finishParallelPart( currArea );

  if( auto *blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) )
  {
    for( int jId = 1; jId <= numJobs; jId++ )
    {
      if( !jobUsed[jId] || jId == bestJId ) continue;

      auto *jobBlkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) )->m_modeCtrl );
      CHECK( !jobBlkCache, "If own mode controller has blk info cache capability so should all other mode controllers!" );
      blkCache->CacheBlkInfoCtrl::copyState( *jobBlkCache, partitioner.currArea() );
    }

    blkCache->tick();
  }

}

void EncCu::copyState( EncCu* other, Partitioner& partitioner, const UnitArea& currArea, const bool isDist )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth () );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  if( isDist )
  {
    other->m_pBestCS[wIdx][hIdx]->initSubStructure( *m_pBestCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
    other->m_pTempCS[wIdx][hIdx]->initSubStructure( *m_pTempCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
  }
  else
  {
          CodingStructure* dst =        m_pBestCS[wIdx][hIdx];
    const CodingStructure *src = other->m_pBestCS[wIdx][hIdx];
    bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;

    dst->useSubStructure( *src, partitioner.chType, currArea, KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );
    dst->cost           =  src->cost;
    dst->dist           =  src->dist;
    dst->fracBits       =  src->fracBits;
    dst->features       =  src->features;
  }

  if( isDist )
  {
    m_CurrCtx = m_CtxBuffer.data();
  }

  m_pcInterSearch->copyState( *other->m_pcInterSearch );
  m_modeCtrl     ->copyState( *other->m_modeCtrl, partitioner.currArea() );
  m_pcRdCost     ->copyState( *other->m_pcRdCost );
  m_pcTrQuant    ->copyState( *other->m_pcTrQuant );

  m_CABACEstimator->getCtx() = other->m_CABACEstimator->getCtx();
}
#endif

void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode
  , LutMotionCand* &tempMotCandLUTs
  , LutMotionCand* &bestMotCandLUTs
  , UnitArea  parArea
)
{
  const int qp                = encTestMode.qp;
  const PPS &pps              = *tempCS->pps;
  const Slice &slice          = *tempCS->slice;
  const bool bIsLosslessMode  = false; // False at this level. Next level down may set it to true.
  const int oldPrevQp         = tempCS->prevQP[partitioner.chType];
  const uint32_t currDepth        = partitioner.currDepth;

  const unsigned wParIdx = gp_sizeIdxInfo->idxFrom(parArea.lwidth());
  const unsigned hParIdx = gp_sizeIdxInfo->idxFrom(parArea.lheight());
#if JVET_L0293_CPR
  if (tempCS->chType == CHANNEL_TYPE_LUMA)
#endif
  tempCS->slice->copyMotionLUTs(tempMotCandLUTs, m_pSplitTempMotLUTs[wParIdx][hParIdx]);

  const PartSplit split = getPartSplit( encTestMode );

  CHECK( split == CU_DONT_SPLIT, "No proper split provided!" );

  tempCS->initStructData( qp, bIsLosslessMode );

  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  const TempCtx ctxStartSP( m_CtxCache, SubCtx( Ctx::SplitFlag,   m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartBT( m_CtxCache, SubCtx( Ctx::BTSplitFlag, m_CABACEstimator->getCtx() ) );

  m_CABACEstimator->resetBits();

  if( partitioner.getImplicitSplit( *tempCS ) != CU_QUAD_SPLIT )
  {
    if( partitioner.canSplit( CU_QUAD_SPLIT, *tempCS ) )
    {
      m_CABACEstimator->split_cu_flag( split == CU_QUAD_SPLIT, *tempCS, partitioner );
    }
    if( split != CU_QUAD_SPLIT )
    {
      m_CABACEstimator->split_cu_mode_mt( split, *tempCS, partitioner );
    }
  }

  const double factor = ( tempCS->currQP[partitioner.chType] > 30 ? 1.1 : 1.075 );
  const double cost   = m_pcRdCost->calcRdCost( uint64_t( m_CABACEstimator->getEstFracBits() + ( ( bestCS->fracBits ) / factor ) ), Distortion( bestCS->dist / factor ) );

  m_CABACEstimator->getCtx() = SubCtx( Ctx::SplitFlag,   ctxStartSP );
  m_CABACEstimator->getCtx() = SubCtx( Ctx::BTSplitFlag, ctxStartBT );

  if( cost > bestCS->cost )
  {
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
    return;
  }

  partitioner.splitCurrArea( split, *tempCS );

  m_CurrCtx++;

  tempCS->getRecoBuf().fill( 0 );
  AffineMVInfo tmpMVInfo;
  bool isAffMVInfoSaved;
  m_pcInterSearch->savePrevAffMVInfo(0, tmpMVInfo, isAffMVInfoSaved);

  do
  {
    const auto &subCUArea  = partitioner.currArea();

    if( tempCS->picture->Y().contains( subCUArea.lumaPos() ) )
    {
      const unsigned wIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lwidth () );
      const unsigned hIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lheight() );

      CodingStructure *tempSubCS = m_pTempCS[wIdx][hIdx];
      CodingStructure *bestSubCS = m_pBestCS[wIdx][hIdx];

      tempCS->initSubStructure( *tempSubCS, partitioner.chType, subCUArea, false );
      tempCS->initSubStructure( *bestSubCS, partitioner.chType, subCUArea, false );
      LutMotionCand *tempSubMotCandLUTs = m_pTempMotLUTs[wIdx][hIdx];
      LutMotionCand *bestSubMotCandLUTs = m_pBestMotLUTs[wIdx][hIdx];
#if JVET_L0293_CPR
      if (tempCS->chType == CHANNEL_TYPE_LUMA)
      {
#endif
        tempCS->slice->copyMotionLUTs(tempMotCandLUTs, tempSubMotCandLUTs);
        tempCS->slice->copyMotionLUTs(tempMotCandLUTs, bestSubMotCandLUTs);
#if JVET_L0293_CPR
      }
#endif

      xCompressCU( tempSubCS, bestSubCS, partitioner
        , tempSubMotCandLUTs
        , bestSubMotCandLUTs
      );

      if( bestSubCS->cost == MAX_DOUBLE )
      {
        CHECK( split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split" );
        tempCS->cost = MAX_DOUBLE;
        m_CurrCtx--;
        partitioner.exitCurrSplit();
        bool bestCSUpdated =
        xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

#if JVET_L0293_CPR
        if (tempCS->chType == CHANNEL_TYPE_LUMA)
#endif
        if (bestCSUpdated)
        {
          std::swap(tempMotCandLUTs, bestMotCandLUTs);
        }
        return;
      }

      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      tempCS->useSubStructure( *bestSubCS, partitioner.chType, CS::getArea( *tempCS, subCUArea, partitioner.chType ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );
#if JVET_L0293_CPR
      if (tempCS->chType == CHANNEL_TYPE_LUMA)
#endif
      tempCS->slice->copyMotionLUTs(bestSubMotCandLUTs, tempMotCandLUTs);

      if(currDepth < pps.getMaxCuDQPDepth())
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }

      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
    }
  } while( partitioner.nextPart( *tempCS ) );

  partitioner.exitCurrSplit();

  m_CurrCtx--;

  // Finally, generate split-signaling bits for RD-cost check
  const PartSplit implicitSplit = partitioner.getImplicitSplit( *tempCS );

  {
    bool enforceQT = implicitSplit == CU_QUAD_SPLIT;
#if HM_QTBT_REPRODUCE_FAST_LCTU_BUG

    // LARGE CTU bug
    if( m_pcEncCfg->getUseFastLCTU() )
    {
      unsigned minDepth = 0;
#if JVET_L0217_L0678_SPS_CLEANUP
      unsigned maxDepth = g_aucLog2[tempCS->sps->getCTUSize()] - g_aucLog2[tempCS->sps->getMinQTSize(slice.getSliceType(), partitioner.chType)];
#else
      unsigned maxDepth = g_aucLog2[tempCS->sps->getSpsNext().getCTUSize()] - g_aucLog2[tempCS->sps->getSpsNext().getMinQTSize( slice.getSliceType(), partitioner.chType )];
#endif

      if( auto ad = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner ) )
      {
        ad->setMaxMinDepth( minDepth, maxDepth, *tempCS );
      }

      if( minDepth > partitioner.currQtDepth )
      {
        // enforce QT
        enforceQT = true;
      }
    }
#endif

    if( !enforceQT )
    {
      m_CABACEstimator->resetBits();

      if( partitioner.canSplit( CU_QUAD_SPLIT, *tempCS ) )
      {
        m_CABACEstimator->split_cu_flag( split == CU_QUAD_SPLIT, *tempCS, partitioner );
      }
      if( split != CU_QUAD_SPLIT )
      {
        m_CABACEstimator->split_cu_mode_mt( split, *tempCS, partitioner );
      }

      tempCS->fracBits += m_CABACEstimator->getEstFracBits(); // split bits
    }
  }

  tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

  // Check Delta QP bits for splitted structure
  xCheckDQP( *tempCS, partitioner, true );

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE)
  {
#if HEVC_TILES_WPP
    const TileMap& tileMap = *tempCS->picture->tileMap;
#endif
#if HEVC_TILES_WPP || HEVC_DEPENDENT_SLICES
    const uint32_t CtuAddr             = CU::getCtuAddr( *bestCS->getCU( partitioner.chType ) );
#endif
    const bool isEndOfSlice        =    slice.getSliceMode() == FIXED_NUMBER_OF_BYTES
                                      && ((slice.getSliceBits() + CS::getEstBits(*bestCS)) > slice.getSliceArgument() << 3)
#if HEVC_TILES_WPP
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceCurStartCtuTsAddr())
#endif
#if HEVC_DEPENDENT_SLICES
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceSegmentCurStartCtuTsAddr());
#else
                                      ;
#endif

#if HEVC_DEPENDENT_SLICES
    const bool isEndOfSliceSegment =    slice.getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES
                                      && ((slice.getSliceSegmentBits() + CS::getEstBits(*bestCS)) > slice.getSliceSegmentArgument() << 3)
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceSegmentCurStartCtuTsAddr());
                                          // Do not need to check slice condition for slice-segment since a slice-segment is a subset of a slice.
    if (isEndOfSlice || isEndOfSliceSegment)
#else
    if(isEndOfSlice)
#endif
    {
      bestCS->cost = MAX_DOUBLE;
    }
  }


  // RD check for sub partitioned coding structure.
  bool bestCSUpdated =
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

  if (isAffMVInfoSaved)
    m_pcInterSearch->addAffMVInfo(tmpMVInfo);

  if (!slice.isIntra()
#if JVET_L0293_CPR
    && tempCS->chType == CHANNEL_TYPE_LUMA
#endif
    )
  {
    if (bestCSUpdated)
    {
      std::swap(tempMotCandLUTs, bestMotCandLUTs);
    }
    tempCS->slice->copyMotionLUTs(m_pSplitTempMotLUTs[wParIdx][hParIdx], tempMotCandLUTs);
  }

  tempCS->releaseIntermediateData();

  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}


void EncCu::xCheckRDCostIntra( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  double bestInterCost             = m_modeCtrl->getBestInterCost();
  double costSize2Nx2NemtFirstPass = m_modeCtrl->getEmtSize2Nx2NFirstPassCost();
  bool skipSecondEmtPass           = m_modeCtrl->getSkipSecondEMTPass();
  const SPS &sps                   = *tempCS->sps;
  const PPS &pps              = *tempCS->pps;
  const CodingUnit *bestCU    = bestCS->getCU( partitioner.chType );
  const int maxSizeEMT        = EMT_INTRA_MAX_CU_WITH_QTBT;
  uint8_t considerEmtSecondPass = ( sps.getSpsNext().getUseIntraEMT() && isLuma( partitioner.chType ) && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ) ? 1 : 0;

  Distortion interHad = m_modeCtrl->getInterHad();



  for( uint8_t emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
  {
    //Possible early EMT tests interruptions
    //2) Second EMT pass. This "if clause" is necessary because of the NSST and PDPC "for loops".
    if( emtCuFlag && skipSecondEmtPass )
    {
      continue;
    }
    //3) if interHad is 0, only try further modes if some intra mode was already better than inter
    if( m_pcEncCfg->getUsePbIntraFast() && !tempCS->slice->isIntra() && bestCU && CU::isInter( *bestCS->getCU( partitioner.chType ) ) && interHad == 0 )
    {
      continue;
    }

    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

    CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
    cu.skip             = false;
    cu.mmvdSkip = false;
    cu.predMode         = MODE_INTRA;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;
  //cu.ipcm             = false;
    cu.emtFlag          = emtCuFlag;

    CU::addPUs( cu );

    tempCS->interHad    = interHad;

    if( isLuma( partitioner.chType ) )
    {
      m_pcIntraSearch->estIntraPredLumaQT( cu, partitioner );

      if (m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == std::numeric_limits<Distortion>::max()
          && tempCS->interHad == 0)
      {
        interHad = 0;
        // JEM assumes only perfect reconstructions can from now on beat the inter mode
        m_modeCtrl->enforceInterHad( 0 );
        continue;
      }

      if( !CS::isDualITree( *tempCS ) )
      {
        cu.cs->picture->getRecoBuf( cu.Y() ).copyFrom( cu.cs->getRecoBuf( COMPONENT_Y ) );
      }
    }

    if( tempCS->area.chromaFormat != CHROMA_400 && ( partitioner.chType == CHANNEL_TYPE_CHROMA || !CS::isDualITree( *tempCS ) ) )
    {
      m_pcIntraSearch->estIntraPredChromaQT( cu, partitioner );
    }

    cu.rootCbf = false;

    for( uint32_t t = 0; t < getNumberValidTBlocks( *cu.cs->pcv ); t++ )
    {
      cu.rootCbf |= cu.firstTU->cbf[t] != 0;
    }

    // Get total bits for current mode: encode CU
    m_CABACEstimator->resetBits();

    if( pps.getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    if( !cu.cs->slice->isIntra() 
#if JVET_L0293_CPR 
      && cu.Y().valid()
#endif
      )
    {
      m_CABACEstimator->cu_skip_flag ( cu );
    }
    m_CABACEstimator->pred_mode      ( cu );
    m_CABACEstimator->extend_ref_line( cu );
    m_CABACEstimator->cu_pred_data   ( cu );
    m_CABACEstimator->pcm_data       ( cu, partitioner );

    // Encode Coefficients
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual( cu, partitioner, cuCtx );

    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );


    // we save the cost of the modes for the first EMT pass
    if( !emtCuFlag ) static_cast< double& >( costSize2Nx2NemtFirstPass ) = tempCS->cost;

#if WCG_EXT
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );


    //now we check whether the second pass of SIZE_2Nx2N and the whole Intra SIZE_NxN should be skipped or not
    if( !emtCuFlag && !tempCS->slice->isIntra() && bestCU && bestCU->predMode != MODE_INTRA && m_pcEncCfg->getFastInterEMT() )
    {
      const double thEmtInterFastSkipIntra = 1.4; // Skip checking Intra if "2Nx2N using DCT2" is worse than best Inter mode
      if( costSize2Nx2NemtFirstPass > thEmtInterFastSkipIntra * bestInterCost )
      {
        skipSecondEmtPass = true;
        m_modeCtrl->setSkipSecondEMTPass( true );
        break;
      }
    }

  } //for emtCuFlag
}

void EncCu::xCheckIntraPCM(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
  cu.skip             = false;
  cu.mmvdSkip = false;
  cu.predMode         = MODE_INTRA;
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
  cu.ipcm             = true;

  tempCS->addPU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

  tempCS->addTU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

  m_pcIntraSearch->IPCMSearch(*tempCS, partitioner);

  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  m_CABACEstimator->resetBits();

  if( tempCS->pps->getTransquantBypassEnabledFlag() )
  {
    m_CABACEstimator->cu_transquant_bypass_flag( cu );
  }

  if( !cu.cs->slice->isIntra() 
#if JVET_L0293_CPR 
    && cu.Y().valid()
#endif
    )
  {
    m_CABACEstimator->cu_skip_flag ( cu );
  }
  m_CABACEstimator->pred_mode      ( cu );
  m_CABACEstimator->pcm_data       ( cu, partitioner );


  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit( *tempCS, partitioner );

  xCheckDQP( *tempCS, partitioner );

#if WCG_EXT
  DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
  DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
}

void EncCu::xCheckDQP( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx )
{
  CHECK( bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit( cs ) == CU_DONT_SPLIT, "bKeepCtx should only be set in split case" );
  CHECK( !bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case" );

  if( !cs.pps->getUseDQP() )
  {
    return;
  }

#if JVET_L0428_DQP_SEP_TREE
  if (CS::isDualITree(cs) && isChroma(partitioner.chType))
  {
    return;
  }
#endif

  if( bKeepCtx && partitioner.currDepth != cs.pps->getMaxCuDQPDepth() )
  {
    return;
  }

  if( !bKeepCtx && partitioner.currDepth > cs.pps->getMaxCuDQPDepth() )
  {
    return;
  }

  CodingUnit* cuFirst = cs.getCU( partitioner.chType );

  CHECK( !cuFirst, "No CU available" );

  bool hasResidual = false;
  for( const auto &cu : cs.cus )
  {
    if( cu->rootCbf )
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP( *cuFirst, cs.prevQP[partitioner.chType] );

  if( hasResidual )
  {
    TempCtx ctxTemp( m_CtxCache );
    if( !bKeepCtx ) ctxTemp = SubCtx( Ctx::DeltaQP, m_CABACEstimator->getCtx() );

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta( *cuFirst, predQP, cuFirst->qp );

    cs.fracBits += m_CABACEstimator->getEstFracBits(); // dQP bits
    cs.cost      = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);


    if( !bKeepCtx ) m_CABACEstimator->getCtx() = SubCtx( Ctx::DeltaQP, ctxTemp );

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for( const auto &cu : cs.cus )
    {
      if( cu->rootCbf )
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for( const auto &cu : cs.cus )
    {
      cu->qp = predQP;
    }
  }
}

void EncCu::xFillPCMBuffer( CodingUnit &cu )
{
  const ChromaFormat format        = cu.chromaFormat;
  const uint32_t numberValidComponents = getNumberValidComponents(format);

  for( auto &tu : CU::traverseTUs( cu ) )
  {
    for( uint32_t ch = 0; ch < numberValidComponents; ch++ )
    {
      const ComponentID compID = ComponentID( ch );

      const CompArea &compArea = tu.blocks[ compID ];

      const CPelBuf source      = tu.cs->getOrgBuf( compArea );
             PelBuf destination = tu.getPcmbuf( compID );

      destination.copyFrom( source );
    }
  }
}


void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;

  if( sps.getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mergeCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
  }

  setMergeBestSATDCost( MAX_DOUBLE );

  {
    // first get merge candidates
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx  = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;
    PU::getInterMergeCandidates(pu, mergeCtx
      , 0
    );
    PU::restrictBiPredMergeCands(pu, mergeCtx);
    PU::getInterMMVDMergeCandidates(pu, mergeCtx);
  }
  bool candHasNoResidual[MRG_MAX_NUM_CANDS + MMVD_ADD_NUM];
  for (uint32_t ui = 0; ui < MRG_MAX_NUM_CANDS + MMVD_ADD_NUM; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip = false;
  bool                                        bestIsMMVDSkip = true;
  PelUnitBuf                                  acMergeBuffer[MRG_MAX_NUM_CANDS];
  PelUnitBuf                                  acMergeRealBuffer[MMVD_MRG_MAX_RD_BUF_NUM];
  PelUnitBuf *                                acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM];
  PelUnitBuf *                                singleMergeTempBuffer;
  int                                         insertPos;
  unsigned                                    uiNumMrgSATDCand = mergeCtx.numValidMergeCand + MMVD_ADD_NUM;

  static_vector<unsigned, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  RdModeList;
  bool                                        mrgTempBufSet = false;

  for (unsigned i = 0; i < MRG_MAX_NUM_CANDS + MMVD_ADD_NUM; i++)
  {
    RdModeList.push_back(i);
  }

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  for (unsigned i = 0; i < MMVD_MRG_MAX_RD_BUF_NUM; i++)
  {
    acMergeRealBuffer[i] = m_acMergeBuffer[i].getBuf(localUnitArea);
    if (i < MMVD_MRG_MAX_RD_NUM)
    {
      acMergeTempBuffer[i] = acMergeRealBuffer + i;
    }
    else
    {
      singleMergeTempBuffer = acMergeRealBuffer + i;
    }
  }

  static_vector<unsigned, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  RdModeList2; // store the Intra mode for Intrainter
  RdModeList2.clear();
  bool isIntrainterEnabled = sps.getSpsNext().getUseMHIntra();
  if (bestCS->area.lwidth() * bestCS->area.lheight() < 64 || bestCS->area.lwidth() >= MAX_CU_SIZE || bestCS->area.lheight() >= MAX_CU_SIZE)
  {
    isIntrainterEnabled = false;
  }
  bool isTestSkipMerge[MRG_MAX_NUM_CANDS]; // record if the merge candidate has tried skip mode 
  for (uint32_t idx = 0; idx < MRG_MAX_NUM_CANDS; idx++)
  {
    isTestSkipMerge[idx] = false;
  }
  if( m_pcEncCfg->getUseFastMerge() || isIntrainterEnabled)
  {
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;
    if (isIntrainterEnabled)
    {
      uiNumMrgSATDCand += 1;
    }
    bestIsSkip       = false;

    if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
    {
#if JVET_L0293_CPR
      if (slice.getSPS()->getSpsNext().getCPRMode())
      {
        ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
        bestIsSkip = blkCache->isSkip(tempCS->area) && cuECtx.bestCU;
      }
      else
#endif
      bestIsSkip = blkCache->isSkip( tempCS->area );
      bestIsMMVDSkip = blkCache->isMMVDSkip(tempCS->area);
    }

    if (isIntrainterEnabled) // always perform low complexity check
    {
      bestIsSkip = false;
    }

    static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if( !bestIsSkip )
    {
      RdModeList.clear();
      mrgTempBufSet       = true;
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( encTestMode.lossless );

      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );
      const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda(cu.transQuantBypass) / double(1 << SCALE_BITS);

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
      cu.skip             = false;
      cu.mmvdSkip = false;
      cu.triangle         = false;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
    //cu.emtFlag  is set below

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;
      const bool bUseHadamard= !encTestMode.lossless;
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );
#if JVET_L0293_CPR
      uint32_t cprCand = 0;
      uint32_t numValidMv = mergeCtx.numValidMergeCand;
#endif
      for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
      {
#if JVET_L0293_CPR
        if ((mergeCtx.interDirNeighbours[uiMergeCand] == 1 || mergeCtx.interDirNeighbours[uiMergeCand] == 3) && tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[uiMergeCand << 1].refIdx)->getPOC() == tempCS->slice->getPOC())
        {
          cprCand++;
          numValidMv--;
          continue;
        }
#endif
        mergeCtx.setMergeInfo( pu, uiMergeCand );

        PU::spanMotionInfo( pu, mergeCtx );
        distParam.cur = singleMergeTempBuffer->Y();
        m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
        acMergeBuffer[uiMergeCand] = m_acRealMergeBuffer[uiMergeCand].getBuf(localUnitArea);
        acMergeBuffer[uiMergeCand].copyFrom(*singleMergeTempBuffer);
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
        {
          mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
          mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
        }

        Distortion uiSad = distParam.distFunc(distParam);
        uint32_t uiBitsCand = uiMergeCand + 1;
        if( uiMergeCand == tempCS->slice->getMaxNumMergeCand() - 1 )
        {
          uiBitsCand--;
        }
        uiBitsCand++; // for mmvd_flag
        double cost     = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;
        insertPos = -1;
        updateDoubleCandList(uiMergeCand, cost, RdModeList, candCostList, RdModeList2, (uint32_t)NUM_LUMA_MODE, uiNumMrgSATDCand, &insertPos);
        if (insertPos != -1)
        {
          if (insertPos == RdModeList.size() - 1)
          {
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
          else
          {
            for (uint32_t i = uint32_t(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
#if JVET_L0293_CPR
        CHECK(std::min(uiMergeCand + 1 - cprCand, uiNumMrgSATDCand) != RdModeList.size(), "");
#else        
        CHECK( std::min( uiMergeCand + 1, uiNumMrgSATDCand ) != RdModeList.size(), "" );
#endif
      }
#if JVET_L0293_CPR
      if (numValidMv < uiNumMrgSATDCand)
        uiNumMrgSATDCand = numValidMv;
      if (numValidMv == 0)
        return;
#endif


      if (isIntrainterEnabled)
      {
        int numTestIntraMode = 4;
        // prepare for Intra bits calculation
        const TempCtx ctxStart(m_CtxCache, m_CABACEstimator->getCtx());
        const TempCtx ctxStartIntraMode(m_CtxCache, SubCtx(Ctx::MHIntraPredMode, m_CABACEstimator->getCtx()));

        // for Intrainter fast, recored the best intra mode during the first round for mrege 0
        int bestMHIntraMode = -1;
        double bestMHIntraCost = MAX_DOUBLE;

        pu.mhIntraFlag = true;

        // save the to-be-tested merge candidates
        uint32_t MHIntraMergeCand[NUM_MRG_SATD_CAND];
#if JVET_L0293_CPR
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(NUM_MRG_SATD_CAND, (const int) uiNumMrgSATDCand); mergeCnt++)
#else
        for (uint32_t mergeCnt = 0; mergeCnt < NUM_MRG_SATD_CAND; mergeCnt++)
#endif
        {
          MHIntraMergeCand[mergeCnt] = RdModeList[mergeCnt];
        }
#if JVET_L0293_CPR
        for (uint32_t mergeCnt = 0; mergeCnt < std::min( std::min(NUM_MRG_SATD_CAND, (const int)uiNumMrgSATDCand), 4); mergeCnt++)
#else
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(NUM_MRG_SATD_CAND, 4); mergeCnt++)
#endif
        {
          uint32_t mergeCand = MHIntraMergeCand[mergeCnt];
          acMergeBuffer[mergeCand] = m_acRealMergeBuffer[mergeCand].getBuf(localUnitArea);

          // estimate merge bits
          uint32_t bitsCand = mergeCand + 1;
          if (mergeCand == pu.cs->slice->getMaxNumMergeCand() - 1)
          {
            bitsCand--;
          }

          // first round
          for (uint32_t intraCnt = 0; intraCnt < numTestIntraMode; intraCnt++)
          {
            pu.intraDir[0] = (intraCnt < 2) ? intraCnt : ((intraCnt == 2) ? HOR_IDX : VER_IDX);

            // fast 2
            if (mergeCnt > 0 && bestMHIntraMode != pu.intraDir[0])
            {
              continue;
            }
            int narrowCase = PU::getNarrowShape(pu.lwidth(), pu.lheight());
            if (narrowCase == 1 && pu.intraDir[0] == HOR_IDX)
            {
              continue;
            }
            if (narrowCase == 2 && pu.intraDir[0] == VER_IDX)
            {
              continue;
            }
            // generate intrainter Y prediction
            if (mergeCnt == 0)
            {
              bool isUseFilter = IntraPrediction::useFilteredIntraRefSamples(COMPONENT_Y, pu, true, pu);
              m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y(), isUseFilter);
              m_pcIntraSearch->predIntraAng(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu, isUseFilter);
              m_pcIntraSearch->switchBuffer(pu, COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));
            }
            pu.cs->getPredBuf(pu).copyFrom(acMergeBuffer[mergeCand]);
            m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));

            // calculate cost
            distParam.cur = pu.cs->getPredBuf(pu).Y();
            Distortion sadValue = distParam.distFunc(distParam);
            m_CABACEstimator->getCtx() = SubCtx(Ctx::MHIntraPredMode, ctxStartIntraMode);
            uint64_t fracModeBits = m_pcIntraSearch->xFracModeBitsIntra(pu, pu.intraDir[0], CHANNEL_TYPE_LUMA);
            double cost = (double)sadValue + (double)(bitsCand + 1) * sqrtLambdaForFirstPass + (double)fracModeBits * sqrtLambdaForFirstPassIntra;
            insertPos = -1;
            updateDoubleCandList(mergeCand + MRG_MAX_NUM_CANDS + MMVD_ADD_NUM, cost, RdModeList, candCostList, RdModeList2, pu.intraDir[0], uiNumMrgSATDCand, &insertPos);
            if (insertPos != -1)
            {
              for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
              {
                swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
              }
              swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
            }
            // fast 2
            if (mergeCnt == 0 && cost < bestMHIntraCost)
            {
              bestMHIntraMode = pu.intraDir[0];
              bestMHIntraCost = cost;
            }
          }
        }
        pu.mhIntraFlag = false;
        m_CABACEstimator->getCtx() = ctxStart;
      }

      cu.mmvdSkip = true;
      int tempNum = 0;
      tempNum = MMVD_ADD_NUM;
      bool allowDirection[4] = { true, true, true, true };
      for (uint32_t mergeCand = mergeCtx.numValidMergeCand; mergeCand < mergeCtx.numValidMergeCand + tempNum; mergeCand++)
      {
        const int mmvdMergeCand = mergeCand - mergeCtx.numValidMergeCand;
        int bitsBaseIdx = 0;
        int bitsRefineStep = 0;
        int bitsDirection = 2;
        int bitsCand = 0;
        int baseIdx;
        int refineStep;
        int direction;
        baseIdx = mmvdMergeCand / MMVD_MAX_REFINE_NUM;
        refineStep = (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM)) / 4;
        direction = (mmvdMergeCand - baseIdx * MMVD_MAX_REFINE_NUM - refineStep * 4) % 4;
        if (refineStep == 0)
        {
          allowDirection[direction] = true;
        }
        if (allowDirection[direction] == false)
        {
          continue;
        }
        bitsBaseIdx = baseIdx + 1;
        if (baseIdx == MMVD_BASE_MV_NUM - 1)
        {
          bitsBaseIdx--;
        }

        bitsRefineStep = refineStep + 1;
        if (refineStep == MMVD_REFINE_STEP - 1)
        {
          bitsRefineStep--;
        }

        bitsCand = bitsBaseIdx + bitsRefineStep + bitsDirection;
        bitsCand++; // for mmvd_flag

        mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);

        PU::spanMotionInfo(pu, mergeCtx);
        distParam.cur = singleMergeTempBuffer->Y();
        m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);

        Distortion uiSad = distParam.distFunc(distParam);


        double cost = (double)uiSad + (double)bitsCand * sqrtLambdaForFirstPass;
        allowDirection[direction] = cost >  1.3 * candCostList[0] ? 0 : 1;
        insertPos = -1;
        updateDoubleCandList(mergeCand, cost, RdModeList, candCostList, RdModeList2, (uint32_t)NUM_LUMA_MODE, uiNumMrgSATDCand, &insertPos);
        if (insertPos != -1)
        {
          for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
          {
            swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
          }
          swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
        }
      }

      // Try to limit number of candidates using SATD-costs
      for( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      setMergeBestSATDCost( candCostList[0] );

      if (isIntrainterEnabled)
      {
        pu.mhIntraFlag = true;
        for (uint32_t mergeCnt = 0; mergeCnt < uiNumMrgSATDCand; mergeCnt++)
        {
          if (RdModeList[mergeCnt] >= (MRG_MAX_NUM_CANDS + MMVD_ADD_NUM))
          {
            pu.intraDir[0] = RdModeList2[mergeCnt];
            pu.intraDir[1] = DM_CHROMA_IDX;
            uint32_t bufIdx = (pu.intraDir[0] > 1) ? (pu.intraDir[0] == HOR_IDX ? 2 : 3) : pu.intraDir[0];
            bool isUseFilter = IntraPrediction::useFilteredIntraRefSamples(COMPONENT_Cb, pu, true, pu);
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb(), isUseFilter);
            m_pcIntraSearch->predIntraAng(COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), pu, isUseFilter);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));
            isUseFilter = IntraPrediction::useFilteredIntraRefSamples(COMPONENT_Cr, pu, true, pu);
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr(), isUseFilter);
            m_pcIntraSearch->predIntraAng(COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), pu, isUseFilter);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
          }
        }
        pu.mhIntraFlag = false;
      }

      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }
    else
    {
      if (bestIsMMVDSkip)
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand + MMVD_ADD_NUM;
      }
      else
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
      }
    }
  }

  const uint32_t iteration = encTestMode.lossless ? 1 : 2;

  // 2. Pass: check candidates using full RD test
  for( uint32_t uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
  {
    for( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx];


#if JVET_L0293_CPR
      if(uiMergeCand < mergeCtx.numValidMergeCand)
        if ((mergeCtx.interDirNeighbours[uiMergeCand] == 1 || mergeCtx.interDirNeighbours[uiMergeCand] == 3) && tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[uiMergeCand << 1].refIdx)->getPOC() == tempCS->slice->getPOC())
        {
          continue;
        }
#endif

      if (uiNoResidualPass != 0 && uiMergeCand >= (MRG_MAX_NUM_CANDS + MMVD_ADD_NUM)) // intrainter does not support skip mode
      {
        uiMergeCand -= (MRG_MAX_NUM_CANDS + MMVD_ADD_NUM); // for skip, map back to normal merge candidate idx and try RDO
        if (isTestSkipMerge[uiMergeCand])
        {
          continue;
        }
      }

      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMrgHADIdx])
       || ( (uiNoResidualPass == 0) && bestIsSkip ) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
      cu.skip             = false;
      cu.mmvdSkip = false;
      cu.triangle         = false;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      if (uiNoResidualPass == 0 && uiMergeCand >= (MRG_MAX_NUM_CANDS + MMVD_ADD_NUM))
      {
        uiMergeCand -= (MRG_MAX_NUM_CANDS + MMVD_ADD_NUM);
        cu.mmvdSkip = false;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
        pu.mhIntraFlag = true;
        pu.intraDir[0] = RdModeList2[uiMrgHADIdx];
        CHECK(pu.intraDir[0]<0 || pu.intraDir[0]>(NUM_LUMA_MODE - 1), "out of intra mode");
        pu.intraDir[1] = DM_CHROMA_IDX;
      }

      else if (uiMergeCand >= mergeCtx.numValidMergeCand && uiMergeCand < MRG_MAX_NUM_CANDS + MMVD_ADD_NUM)
      {
        cu.mmvdSkip = true;
        mergeCtx.setMmvdMergeCandiInfo(pu, uiMergeCand - mergeCtx.numValidMergeCand);
      }
      else
      {
        cu.mmvdSkip = false;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
      }
      PU::spanMotionInfo( pu, mergeCtx );

      if( mrgTempBufSet )
      {
        if (pu.mhIntraFlag)
        {
          uint32_t bufIdx = (pu.intraDir[0] > 1) ? (pu.intraDir[0] == HOR_IDX ? 2 : 3) : pu.intraDir[0];
          PelBuf tmpBuf = tempCS->getPredBuf(pu).Y();
          tmpBuf.copyFrom(acMergeBuffer[uiMergeCand].Y());
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, bufIdx));
          tmpBuf = tempCS->getPredBuf(pu).Cb();
          tmpBuf.copyFrom(acMergeBuffer[uiMergeCand].Cb());
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Cb, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));
          tmpBuf = tempCS->getPredBuf(pu).Cr();
          tmpBuf.copyFrom(acMergeBuffer[uiMergeCand].Cr());
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Cr, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
        }
        else
        {
          if (uiNoResidualPass != 0 && uiMergeCand < mergeCtx.numValidMergeCand && RdModeList[uiMrgHADIdx] >= (MRG_MAX_NUM_CANDS + MMVD_ADD_NUM))
          {
            tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand]);
          }
          else
          {
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
          }
        }
      }
      else
      {
        m_pcInterSearch->motionCompensation( pu );
        
      }
      if (!cu.mmvdSkip && !pu.mhIntraFlag && uiNoResidualPass != 0)
      {
        CHECK(uiMergeCand >= mergeCtx.numValidMergeCand, "out of normal merge");
        isTestSkipMerge[uiMergeCand] = true;
      }

      xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass
        , NULL
        , 1
        , uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL);

      if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip && !pu.mhIntraFlag)
      {
        bestIsSkip = bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }// end loop uiMrgHADIdx

    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if( bestCU.rootCbf == 0 )
      {
        if( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
}

void EncCu::xCheckRDCostMergeTriangle2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;
  const SPS &sps = *tempCS->sps;

  CHECK( slice.getSliceType() != B_SLICE, "Triangle mode is only applied to B-slices" );
  
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
  
  bool trianglecandHasNoResidual[TRIANGLE_MAX_NUM_CANDS];
  for( int mergeCand = 0; mergeCand < TRIANGLE_MAX_NUM_CANDS; mergeCand++ )
  {
    trianglecandHasNoResidual[mergeCand] = false;
  }

#if JVET_L0293_CPR
  bool bestIsSkip;
  CodingUnit* cuTemp = bestCS->getCU(partitioner.chType);
  if (cuTemp)
    bestIsSkip = m_pcEncCfg->getUseFastDecisionForMerge() ? bestCS->getCU(partitioner.chType)->rootCbf == 0 : false;
  else
    bestIsSkip = false;
#else
  bool                                            bestIsSkip             = m_pcEncCfg->getUseFastDecisionForMerge() ? bestCS->getCU( partitioner.chType )->rootCbf == 0 : false;
#endif
  uint8_t                                         numTriangleCandidate   = TRIANGLE_MAX_NUM_CANDS;
  uint8_t                                         triangleNumMrgSATDCand = TRIANGLE_MAX_NUM_SATD_CANDS;
  PelUnitBuf                                      triangleBuffer[TRIANGLE_MAX_NUM_UNI_CANDS];
  PelUnitBuf                                      triangleWeightedBuffer[TRIANGLE_MAX_NUM_CANDS];
  static_vector<uint8_t, TRIANGLE_MAX_NUM_CANDS> triangleRdModeList;
  static_vector<double,  TRIANGLE_MAX_NUM_CANDS> tianglecandCostList;

  if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
  {
    bestIsSkip |= blkCache->isSkip( tempCS->area );
  }

  DistParam distParam;
  const bool useHadamard = !encTestMode.lossless;
  m_pcRdCost->setDistParam( distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, useHadamard );

  const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );

  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(encTestMode.lossless);

  MergeCtx triangleMrgCtx;
  {
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
    cu.triangle = true;
    cu.mmvdSkip = false;
#if JVET_L0646_GBI
    cu.GBiIdx   = GBI_DEFAULT;
#endif

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;


    PU::getTriangleMergeCandidates( pu, triangleMrgCtx );
    for( uint8_t mergeCand = 0; mergeCand < TRIANGLE_MAX_NUM_UNI_CANDS; mergeCand++ )
    {
      triangleBuffer[mergeCand] = m_acMergeBuffer[mergeCand].getBuf(localUnitArea);
      triangleMrgCtx.setMergeInfo( pu, mergeCand );
      PU::spanMotionInfo( pu, triangleMrgCtx );
      
      m_pcInterSearch->motionCompensation( pu, triangleBuffer[mergeCand] );
    }
  }

  bool tempBufSet = bestIsSkip ? false : true;
  triangleNumMrgSATDCand = bestIsSkip ? TRIANGLE_MAX_NUM_CANDS : TRIANGLE_MAX_NUM_SATD_CANDS;
  if( bestIsSkip )
  {
    for( uint8_t i = 0; i < TRIANGLE_MAX_NUM_CANDS; i++ )
    {
      triangleRdModeList.push_back(i);
    }
  }
  else
  {
    CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );
      
    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
    cu.skip             = false;
    cu.predMode         = MODE_INTER;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;
    cu.triangle         = true;
    cu.mmvdSkip         = false;
#if JVET_L0646_GBI
    cu.GBiIdx           = GBI_DEFAULT;
#endif

    PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );
      
    if( abs(g_aucLog2[cu.lwidth()] - g_aucLog2[cu.lheight()]) >= 2 )
    {
      numTriangleCandidate = 30;
    }
    else
    {
      numTriangleCandidate = TRIANGLE_MAX_NUM_CANDS;
    }

    for( uint8_t mergeCand = 0; mergeCand < numTriangleCandidate; mergeCand++ )
    {
      bool    splitDir = g_triangleCombination[mergeCand][0];
      uint8_t candIdx0 = g_triangleCombination[mergeCand][1];
      uint8_t candIdx1 = g_triangleCombination[mergeCand][2];

      pu.mergeIdx  = mergeCand;
      pu.mergeFlag = true;
      triangleWeightedBuffer[mergeCand] = m_acTriangleWeightedBuffer[mergeCand].getBuf( localUnitArea );
      triangleBuffer[candIdx0] = m_acMergeBuffer[candIdx0].getBuf( localUnitArea );
      triangleBuffer[candIdx1] = m_acMergeBuffer[candIdx1].getBuf( localUnitArea );

      m_pcInterSearch->weightedTriangleBlk( pu, PU::getTriangleWeights(pu, triangleMrgCtx, candIdx0, candIdx1), splitDir, CHANNEL_TYPE_LUMA, triangleWeightedBuffer[mergeCand], triangleBuffer[candIdx0], triangleBuffer[candIdx1] );
      
      distParam.cur = triangleWeightedBuffer[mergeCand].Y();

      Distortion uiSad = distParam.distFunc( distParam );

      uint32_t uiBitsCand = g_triangleIdxBins[mergeCand];

      double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;

      static_vector<int, TRIANGLE_MAX_NUM_CANDS> * nullList = nullptr;
      updateCandList( mergeCand, cost, triangleRdModeList, tianglecandCostList
        , *nullList, -1
        , triangleNumMrgSATDCand );
    }
        
    // limit number of candidates using SATD-costs
    for( uint8_t i = 0; i < triangleNumMrgSATDCand; i++ )
    {
      if( tianglecandCostList[i] > MRG_FAST_RATIO * tianglecandCostList[0] || tianglecandCostList[i] > getMergeBestSATDCost() )
      {
        triangleNumMrgSATDCand = i;
        break;
      }
    }

    // perform chroma weighting process
    for( uint8_t i = 0; i < triangleNumMrgSATDCand; i++ )
    {
      uint8_t  mergeCand = triangleRdModeList[i];
      bool     splitDir  = g_triangleCombination[mergeCand][0];
      uint8_t  candIdx0  = g_triangleCombination[mergeCand][1];
      uint8_t  candIdx1  = g_triangleCombination[mergeCand][2];
        
      pu.mergeIdx  = mergeCand;
      pu.mergeFlag = true;
                
      m_pcInterSearch->weightedTriangleBlk( pu, PU::getTriangleWeights(pu, triangleMrgCtx, candIdx0, candIdx1), splitDir, CHANNEL_TYPE_CHROMA, triangleWeightedBuffer[mergeCand], triangleBuffer[candIdx0], triangleBuffer[candIdx1] );
    }

    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
  }

  {
    const uint8_t iteration = encTestMode.lossless ? 1 : 2;
    for( uint8_t noResidualPass = 0; noResidualPass < iteration; noResidualPass++ )
    {
      for( uint8_t mrgHADIdx = 0; mrgHADIdx < triangleNumMrgSATDCand; mrgHADIdx++ )
      {
        uint8_t mergeCand = triangleRdModeList[mrgHADIdx];

        if ( ( (noResidualPass != 0) && trianglecandHasNoResidual[mergeCand] )
          || ( (noResidualPass == 0) && bestIsSkip ) )
        {
          continue;
        }

        bool    splitDir = g_triangleCombination[mergeCand][0];
        uint8_t candIdx0 = g_triangleCombination[mergeCand][1];
        uint8_t candIdx1 = g_triangleCombination[mergeCand][2];

        CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

        partitioner.setCUData(cu);
        cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
        cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
        cu.skip = false;
        cu.predMode = MODE_INTER;
        cu.transQuantBypass = encTestMode.lossless;
        cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
        cu.qp = encTestMode.qp;
        cu.triangle = true;
        cu.mmvdSkip = false;
#if JVET_L0646_GBI
        cu.GBiIdx   = GBI_DEFAULT;
#endif
        PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

        pu.mergeIdx = mergeCand;
        pu.mergeFlag = true;

        PU::spanTriangleMotionInfo(pu, triangleMrgCtx, mergeCand, splitDir, candIdx0, candIdx1 );

        if( tempBufSet )
        {
          tempCS->getPredBuf().copyFrom( triangleWeightedBuffer[mergeCand] );
        }
        else
        {
          triangleBuffer[candIdx0] = m_acMergeBuffer[candIdx0].getBuf( localUnitArea );
          triangleBuffer[candIdx1] = m_acMergeBuffer[candIdx1].getBuf( localUnitArea );
          PelUnitBuf predBuf         = tempCS->getPredBuf();
          m_pcInterSearch->weightedTriangleBlk( pu, PU::getTriangleWeights(pu, triangleMrgCtx, candIdx0, candIdx1), splitDir, MAX_NUM_CHANNEL_TYPE, predBuf, triangleBuffer[candIdx0], triangleBuffer[candIdx1] );
        }
        
        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, noResidualPass, NULL, true, ( (noResidualPass == 0 ) ? &trianglecandHasNoResidual[mergeCand] : NULL ) );

        if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
        {
          bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
        }
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
      }// end loop mrgHADIdx
    }   
  }
}

void EncCu::xCheckRDCostAffineMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( m_modeCtrl->getFastDeltaQp() )
  {
    return;
  }

  if ( bestCS->area.lumaSize().width < 8 || bestCS->area.lumaSize().height < 8 )
  {
    return;
  }

#if JVET_L0632_AFFINE_MERGE
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Affine Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  AffineMergeCtx affineMergeCtx;
  const SPS &sps = *tempCS->sps;

#if JVET_L0369_SUBBLOCK_MERGE
  MergeCtx mrgCtx;
  if ( sps.getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mrgCtx.subPuMvpMiBuf = MotionBuf( m_SubPuMiBuf, bufSize );
    affineMergeCtx.mrgCtx = &mrgCtx;
  }
#endif

  {
    // first get merge candidates
    CodingUnit cu( tempCS->area );
    cu.cs = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
    cu.mmvdSkip = false;

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;

    PU::getAffineMergeCand( pu, affineMergeCtx );

    if ( affineMergeCtx.numValidMergeCand <= 0 )
    {
      return;
    }
  }

  bool candHasNoResidual[AFFINE_MRG_MAX_NUM_CANDS];
  for ( uint32_t ui = 0; ui < affineMergeCtx.numValidMergeCand; ui++ )
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip = false;
  uint32_t                                    uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
  PelUnitBuf                                  acMergeBuffer[AFFINE_MRG_MAX_NUM_CANDS];
  static_vector<uint32_t, AFFINE_MRG_MAX_NUM_CANDS>  RdModeList;
  bool                                        mrgTempBufSet = false;

  for ( uint32_t i = 0; i < AFFINE_MRG_MAX_NUM_CANDS; i++ )
  {
    RdModeList.push_back( i );
  }

  if ( m_pcEncCfg->getUseFastMerge() )
  {
    uiNumMrgSATDCand = std::min( NUM_AFF_MRG_SATD_CAND, affineMergeCtx.numValidMergeCand );
    bestIsSkip = false;

    if ( auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl) )
    {
      bestIsSkip = blkCache->isSkip( tempCS->area );
    }

    static_vector<double, AFFINE_MRG_MAX_NUM_CANDS> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if ( !bestIsSkip )
    {
      RdModeList.clear();
      mrgTempBufSet = true;
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( encTestMode.lossless );

      CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
      cu.skip = false;
      cu.affine = true;
      cu.predMode = MODE_INTER;
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;

      PredictionUnit &pu = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;
      const bool bUseHadamard = !encTestMode.lossless;
      m_pcRdCost->setDistParam( distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, bUseHadamard );

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height ) );

      for ( uint32_t uiMergeCand = 0; uiMergeCand < affineMergeCtx.numValidMergeCand; uiMergeCand++ )
      {
        acMergeBuffer[uiMergeCand] = m_acMergeBuffer[uiMergeCand].getBuf( localUnitArea );

        // set merge information
        pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
        pu.mergeFlag = true;
        pu.mergeIdx = uiMergeCand;
        cu.affineType = affineMergeCtx.affineType[uiMergeCand];
#if JVET_L0646_GBI
        cu.GBiIdx = affineMergeCtx.GBiIdx[uiMergeCand];
#endif

#if JVET_L0369_SUBBLOCK_MERGE
        pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
        if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP )
        {
          pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
          pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
          PU::spanMotionInfo( pu, mrgCtx );
        }
        else
        {
#endif
          PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0 );
          PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1 );

          PU::spanMotionInfo( pu );
#if JVET_L0369_SUBBLOCK_MERGE
        }
#endif

        distParam.cur = acMergeBuffer[uiMergeCand].Y();

        m_pcInterSearch->motionCompensation( pu, acMergeBuffer[uiMergeCand] );

        Distortion uiSad = distParam.distFunc( distParam );
        uint32_t   uiBitsCand = uiMergeCand + 1;
        if ( uiMergeCand == tempCS->slice->getMaxNumAffineMergeCand() - 1 )
        {
          uiBitsCand--;
        }
        double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;
        static_vector<int, AFFINE_MRG_MAX_NUM_CANDS> * nullList = nullptr;
        updateCandList( uiMergeCand, cost, RdModeList, candCostList
          , *nullList, -1
          , uiNumMrgSATDCand );

        CHECK( std::min( uiMergeCand + 1, uiNumMrgSATDCand ) != RdModeList.size(), "" );
      }

      // Try to limit number of candidates using SATD-costs
      for ( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if ( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }
    else
    {
      uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
    }
  }

  const uint32_t iteration = encTestMode.lossless ? 1 : 2;

  // 2. Pass: check candidates using full RD test
  for ( uint32_t uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
  {
    for ( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx];

      if ( ((uiNoResidualPass != 0) && candHasNoResidual[uiMergeCand])
        || ((uiNoResidualPass == 0) && bestIsSkip) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
      cu.skip = false;
      cu.affine = true;
      cu.predMode = MODE_INTER;
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      PredictionUnit &pu = tempCS->addPU( cu, partitioner.chType );

      // set merge information
      pu.mergeFlag = true;
      pu.mergeIdx = uiMergeCand;
      pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
      cu.affineType = affineMergeCtx.affineType[uiMergeCand];
#if JVET_L0646_GBI
      cu.GBiIdx = affineMergeCtx.GBiIdx[uiMergeCand];
#endif

#if JVET_L0369_SUBBLOCK_MERGE
      pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
      if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP )
      {
        pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
        pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
        PU::spanMotionInfo( pu, mrgCtx );
      }
      else
      {
#endif
        PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0 );
        PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1 );

        PU::spanMotionInfo( pu );
#if JVET_L0369_SUBBLOCK_MERGE
      }
#endif

      if ( mrgTempBufSet )
      {
        tempCS->getPredBuf().copyFrom( acMergeBuffer[uiMergeCand] );
      }
      else
      {
        m_pcInterSearch->motionCompensation( pu );
      }

      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, NULL, true, ((uiNoResidualPass == 0) ? &candHasNoResidual[uiMergeCand] : NULL) );

      if ( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
      {
        bestIsSkip = bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }// end loop uiMrgHADIdx

    if ( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if ( bestCU.rootCbf == 0 )
      {
        if ( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if ( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if ( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
#else
  MvField       affineMvField[2][3];
  unsigned char interDirNeighbours;
  int           numValidMergeCand;
  bool          hasNoResidual = false;
#if JVET_L0646_GBI
  uint8_t       gbiIdx = GBI_DEFAULT;
#endif


  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
  cu.skip             = false;
  cu.mmvdSkip = false;
  cu.partSize         = encTestMode.partSize;
  cu.affine           = true;
  cu.predMode         = MODE_INTER;
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;

  CU::addPUs( cu );

  cu.firstPU->mergeFlag = true;
  cu.firstPU->mergeIdx  = 0;
#if JVET_L0646_GBI
  PU::getAffineMergeCand( *cu.firstPU, affineMvField, interDirNeighbours, gbiIdx, numValidMergeCand );
#else
  PU::getAffineMergeCand( *cu.firstPU, affineMvField, interDirNeighbours, numValidMergeCand );
#endif
  if( numValidMergeCand == -1 )
  {
    return;
  }

  cu.firstPU->interDir = interDirNeighbours;
  PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_0], REF_PIC_LIST_0 );
  PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_1], REF_PIC_LIST_1 );
#if JVET_L0646_GBI
  cu.GBiIdx = gbiIdx;
#endif

  PU::spanMotionInfo( *cu.firstPU );

  m_pcInterSearch->motionCompensation( cu );

  xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
    , NULL
    , 1
    , &hasNoResidual);

  if( ! (encTestMode.lossless || hasNoResidual) )
  {
    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    tempCS->copyStructure( *bestCS, partitioner.chType );
    tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );

    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 1
      , NULL
      , 1
      , &hasNoResidual);
  }
#endif
}
#if JVET_L0293_CPR
//////////////////////////////////////////////////////////////////////////////////////////////
// cpr merge/skip mode check
void EncCu::xCheckRDCostCPRModeMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  assert(tempCS->chType != CHANNEL_TYPE_CHROMA); // chroma CPR is derived

  if (tempCS->area.lwidth() > CPR_MAX_CAND_SIZE || tempCS->area.lheight() > CPR_MAX_CAND_SIZE) // currently only check 32x32 and below block for cpr merge/skip
  {
    return;
  }
  const SPS &sps = *tempCS->sps;

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  MergeCtx mergeCtx;


  if (sps.getSpsNext().getUseSubPuMvp())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
  }

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_INTER;
    cu.cpr = true;
    cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    cu.mmvdSkip = false;
    pu.mmvdMergeFlag = false;
    cu.triangle = false;
    PU::getInterMergeCandidates(pu, mergeCtx
      , 0
    );
  }

  int candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (unsigned int ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = 0;
  }

  bool                                        bestIsSkip = false;
  unsigned                                    numMrgSATDCand = mergeCtx.numValidMergeCand;
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList(MRG_MAX_NUM_CANDS);
  for (unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++)
  {
    RdModeList[i] = i;
  }

  //{
    static_vector<double, MRG_MAX_NUM_CANDS>  candCostList(MRG_MAX_NUM_CANDS, MAX_DOUBLE);
    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    {
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(encTestMode.lossless);

      CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
      cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
      cu.skip = false;
      cu.predMode = MODE_INTER;
      cu.cpr = true;
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      cu.mmvdSkip = false;
      cu.triangle = false;
      DistParam distParam;
      const bool bUseHadamard = !encTestMode.lossless;
      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType); //tempCS->addPU(cu);
      pu.mmvdMergeFlag = false;
      Picture* refPic = pu.cu->slice->getPic();
      const CPelBuf refBuf = refPic->getRecoBuf(pu.blocks[COMPONENT_Y]);
      const Pel*        piRefSrch = refBuf.buf;
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
      int refStride = refBuf.stride;
      const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
      int numValidBv = mergeCtx.numValidMergeCand;
      for (unsigned int mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
      {
        if (mergeCtx.interDirNeighbours[mergeCand] != 1)
        {
          numValidBv--;
          continue;
        }
        if (tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[mergeCand << 1].refIdx)->getPOC() != tempCS->slice->getPOC())
        {
          numValidBv--;
          continue;
        }
        mergeCtx.setMergeInfo(pu, mergeCand); // set bv info in merge mode
        const int cuPelX = pu.Y().x;
        const int cuPelY = pu.Y().y;
        int roiWidth = pu.lwidth();
        int roiHeight = pu.lheight();
        const int picWidth = pu.cs->slice->getSPS()->getPicWidthInLumaSamples();
        const int picHeight = pu.cs->slice->getSPS()->getPicHeightInLumaSamples();
        const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
        int xPred = pu.bv.getHor();
        int yPred = pu.bv.getVer();

        if (!PU::isBlockVectorValid(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, 0, xPred, yPred, lcuWidth)) // not valid bv derived
        {
          numValidBv--;
          continue;
        }
        PU::spanMotionInfo(pu, mergeCtx);

        distParam.cur.buf = piRefSrch + refStride * yPred + xPred;

        Distortion sad = distParam.distFunc(distParam);
        unsigned int bitsCand = mergeCand + 1;
        if (mergeCand == tempCS->slice->getMaxNumMergeCand() - 1)
        {
          bitsCand--;
        }
        double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;
        static_vector<int, MRG_MAX_NUM_CANDS> * nullList = nullptr;

        updateCandList(mergeCand, cost, RdModeList, candCostList
          , *nullList, -1
         , numMrgSATDCand);
      }

      // Try to limit number of candidates using SATD-costs
      if (numValidBv)
      {
        numMrgSATDCand = numValidBv;
        for (unsigned int i = 1; i < numValidBv; i++)
        {
          if (candCostList[i] > MRG_FAST_RATIO*candCostList[0])
          {
            numMrgSATDCand = i;
            break;
          }
        }
      }
      else
      {
        tempCS->dist = 0;
        tempCS->fracBits = 0;
        tempCS->cost = MAX_DOUBLE;
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
        return;
      }

      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    }
  //}


  const unsigned int iteration = encTestMode.lossless ? 1 : 2;

  // 2. Pass: check candidates using full RD test
  for (unsigned int numResidualPass = 0; numResidualPass < iteration; numResidualPass++)
  {
    for (unsigned int mrgHADIdx = 0; mrgHADIdx < numMrgSATDCand; mrgHADIdx++)
    {
      unsigned int mergeCand = RdModeList[mrgHADIdx];
      if (mergeCtx.interDirNeighbours[mergeCand] != 1)
      {
        continue;
      }
      if (tempCS->slice->getRefPic(REF_PIC_LIST_0, mergeCtx.mvFieldNeighbours[mergeCand << 1].refIdx)->getPOC() != tempCS->slice->getPOC())
      {
        continue;
      }
      if (!(numResidualPass == 1 && candHasNoResidual[mergeCand] == 1))
      {
        if (!(bestIsSkip && (numResidualPass == 0)))
        {
          unsigned char considerEmtSecondPass = 0;
          bool skipSecondEmtPass = true;
          bool hasResidual[2] = { false, false };
          double emtCost[2] = { MAX_DOUBLE, MAX_DOUBLE };

          // CU-level optimization
          for (unsigned char emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++)
          {
            if (m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass)
            {
              continue;
            }

            // first get merge candidates
            CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

            partitioner.setCUData(cu);
            cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
            cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
            cu.skip = false;
            cu.predMode = MODE_INTER;
            cu.cpr = true;
            cu.transQuantBypass = encTestMode.lossless;
            cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
            cu.qp = encTestMode.qp;

            cu.emtFlag = false;

            PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);// tempCS->addPU(cu);
            pu.intraDir[0] = DC_IDX; // set intra pred for cpr block
            pu.intraDir[1] = PLANAR_IDX; // set intra pred for cpr block
            cu.mmvdSkip = false;
            pu.mmvdMergeFlag = false;
            cu.triangle = false;
            mergeCtx.setMergeInfo(pu, mergeCand);
            PU::spanMotionInfo(pu, mergeCtx);

            assert(mergeCtx.mrgTypeNeighbours[mergeCand] == MRG_TYPE_CPR); //  should be CPR candidate at this round
            const bool chroma = !(CS::isDualITree(*tempCS));

            //  MC
            m_pcInterSearch->motionCompensation(pu,REF_PIC_LIST_0, true, chroma);
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

            m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0), true, chroma);
            xEncodeDontSplit(*tempCS, partitioner);

            if (tempCS->pps->getUseDQP() && (partitioner.currDepth) <= tempCS->pps->getMaxCuDQPDepth())
            {
              xCheckDQP(*tempCS, partitioner);
            }

            hasResidual[emtCuFlag] = cu.rootCbf;
            emtCost[emtCuFlag] = tempCS->cost;

            DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

            tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
          }
          if (numResidualPass == 0 && (emtCost[0] <= emtCost[1] ? !hasResidual[0] : !hasResidual[1]))

            {
              // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
              candHasNoResidual[mergeCand] = 1;
            }

            if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
            {
              if (bestCS->getCU(partitioner.chType) == NULL)
                bestIsSkip = 0;
              else
              bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
            }
        }
      }
    }
  }

}

void EncCu::xCheckRDCostCPRMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

    CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());
#endif
    cu.skip = false;
    cu.predMode = MODE_INTER;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp = encTestMode.qp;
    cu.cpr = true;
    cu.imv = 0;

    CU::addPUs(cu);

    PredictionUnit& pu = *cu.firstPU;
    cu.mmvdSkip = false;
    pu.mmvdMergeFlag = false;

    pu.intraDir[0] = DC_IDX; // set intra pred for cpr block
    pu.intraDir[1] = PLANAR_IDX; // set intra pred for cpr block

    pu.interDir = 1; // use list 0 for CPR mode
    pu.refIdx[REF_PIC_LIST_0] = pu.cs->slice->getNumRefIdx(REF_PIC_LIST_0) - 1; // last idx in the list


    if (partitioner.chType == CHANNEL_TYPE_LUMA)
    {
      bool bValid = m_pcInterSearch->predCPRSearch(cu, partitioner, m_ctuCprSearchRangeX, m_ctuCprSearchRangeY, m_cprHashMap);

      if (bValid)
      {
        PU::spanMotionInfo(pu);
        const bool chroma = !(CS::isDualITree(*tempCS));
        //  MC
        m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, true, chroma);

        double    bestCost = bestCS->cost;
        unsigned char    considerEmtSecondPass = 0;
        bool      skipSecondEmtPass = true;
        double    emtFirstPassCost = MAX_DOUBLE;

        // CU-level optimization

        for (unsigned char emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++)
        {
          if (m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass)
          {
            continue;
          }

          tempCS->getCU(tempCS->chType)->emtFlag = emtCuFlag;

          m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, true, chroma);

          if (m_pcEncCfg->getFastInterEMT())
          {
            emtFirstPassCost = (!emtCuFlag) ? tempCS->cost : emtFirstPassCost;
          }
          xEncodeDontSplit(*tempCS, partitioner);

          if (tempCS->pps->getUseDQP() && (partitioner.currDepth) <= tempCS->pps->getMaxCuDQPDepth())
          {
            xCheckDQP(*tempCS, partitioner);
          }

          DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
          xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

          //now we check whether the second pass should be skipped or not
          if (!emtCuFlag && considerEmtSecondPass)
          {
            static const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
            if (m_pcEncCfg->getFastInterEMT() && (!cu.firstTU->cbf[COMPONENT_Y] || emtFirstPassCost > bestCost * thresholdToSkipEmtSecondPass))
            {
              skipSecondEmtPass = true;
            }
            else //EMT will be checked
            {
              if (bestCost == bestCS->cost) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
              {
                tempCS->clearTUs();
              }
              else
              {
                tempCS->initStructData(bestCS->currQP[bestCS->chType], bestCS->isLossless);

                tempCS->copyStructure(*bestCS, partitioner.chType);
                tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
              }

              //we need to restart the distortion for the new tempCS, the bit count and the cost
              tempCS->dist = 0;
              tempCS->fracBits = 0;
              tempCS->cost = MAX_DOUBLE;
            }
          }
        }

      } // bValid
      else
      {
        tempCS->dist = 0;
        tempCS->fracBits = 0;
        tempCS->cost = MAX_DOUBLE;
      }
    }
 // chroma CU cpr comp
    else
    {
      bool success = true;
      // chroma tree, reuse luma bv at minimal block level
      // enabled search only when each chroma sub-block has a BV from its luma sub-block
      assert(tempCS->getCprLumaCoverage(pu.Cb()) == CPR_LUMA_COVERAGE_FULL);
      // check if each BV for the chroma sub-block is valid
      //static const UInt unitArea = MIN_PU_SIZE * MIN_PU_SIZE;
      const CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
      PredictionUnit subPu;
      subPu.cs = pu.cs;
      subPu.cu = pu.cu;
      const ComponentID compID = COMPONENT_Cb; // use Cb to represent both Cb and CR, as their structures are the same
      int shiftHor = ::getComponentScaleX(compID, pu.chromaFormat);
      int shiftVer = ::getComponentScaleY(compID, pu.chromaFormat);
      //const ChromaFormat  chFmt = pu.chromaFormat;

      for (int y = lumaArea.y; y < lumaArea.y + lumaArea.height; y += MIN_PU_SIZE)
      {
        for (int x = lumaArea.x; x < lumaArea.x + lumaArea.width; x += MIN_PU_SIZE)
        {
          const MotionInfo &curMi = pu.cs->picture->cs->getMotionInfo(Position{ x, y });

          subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, MIN_PU_SIZE, MIN_PU_SIZE)));
          Position offsetRef = subPu.blocks[compID].pos().offset((curMi.bv.getHor() >> shiftHor), (curMi.bv.getVer() >> shiftVer));
          Position refEndPos(offsetRef.x + subPu.blocks[compID].size().width - 1, offsetRef.y + subPu.blocks[compID].size().height - 1 );

          if (!subPu.cs->isDecomp(refEndPos, toChannelType(compID)) || !subPu.cs->isDecomp(offsetRef, toChannelType(compID))) // ref block is not yet available for this chroma sub-block
          {
            success = false;
            break;
          }
        }
        if (!success)
          break;
      }
      ////////////////////////////////////////////////////////////////////////////

      if (success)
      {
        //pu.mergeType = MRG_TYPE_CPR;
        m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, false, true); // luma=0, chroma=1
        m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, false, true);

        xEncodeDontSplit(*tempCS, partitioner);

        xCheckDQP(*tempCS, partitioner);

        DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());

        xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
      }
      else
      {
        tempCS->dist = 0;
        tempCS->fracBits = 0;
        tempCS->cost = MAX_DOUBLE;
      }
    }
  }
  // check cpr mode in encoder RD
  //////////////////////////////////////////////////////////////////////////////////////////////
#endif // CPR

void EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

#if JVET_L0646_GBI
  
  m_pcInterSearch->setAffineModeSelected(false);

  if( tempCS->slice->getCheckLDC() )
  {
    m_bestGbiCost[0] = m_bestGbiCost[1] = std::numeric_limits<double>::max();
    m_bestGbiIdx[0] = m_bestGbiIdx[1] = -1;
  }

  m_pcInterSearch->resetBufferedUniMotions();
  int gbiLoopNum = (tempCS->slice->isInterB() ? GBI_NUM : 1);
  gbiLoopNum = (tempCS->sps->getSpsNext().getUseGBi() ? gbiLoopNum : 1);

  if( tempCS->area.lwidth() * tempCS->area.lheight() < GBI_SIZE_CONSTRAINT )
  {
    gbiLoopNum = 1;
  }

  double curBestCost = bestCS->cost;
  double equGBiCost = MAX_DOUBLE;

  for( int gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
  {
    if( m_pcEncCfg->getUseGBiFast() )
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if( blkCache )
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestGBiIdx = blkCache->getGbiIdx(bestCS->area);

        if( isBestInter && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT && g_GbiSearchOrder[gbiLoopIdx] != bestGBiIdx )
        {
          continue;
        }
      }
    }
    if( !tempCS->slice->getCheckLDC() )
    {
      if( gbiLoopIdx != 0 && gbiLoopIdx != 3 && gbiLoopIdx != 4 )
      {
        continue;
      }
    }
#endif

  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
  cu.skip             = false;
  cu.mmvdSkip = false;
//cu.affine
  cu.predMode         = MODE_INTER;
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
  CU::addPUs( cu );

#if JVET_L0646_GBI
  cu.GBiIdx = g_GbiSearchOrder[gbiLoopIdx];
  uint8_t gbiIdx = cu.GBiIdx;
  bool  testGbi = (gbiIdx != GBI_DEFAULT);
#endif

  m_pcInterSearch->predInterSearch( cu, partitioner );

  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );

#if JVET_L0646_GBI
  gbiIdx = CU::getValidGbiIdx(cu);
  if( testGbi && gbiIdx == GBI_DEFAULT ) // Enabled GBi but the search results is uni.
  {
    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    continue;
  }
  CHECK(!(testGbi || (!testGbi && gbiIdx == GBI_DEFAULT)), " !( bTestGbi || (!bTestGbi && gbiIdx == GBI_DEFAULT ) )");

  bool isEqualUni = false;
  if( m_pcEncCfg->getUseGBiFast() )
  {
    if( cu.firstPU->interDir != 3 && testGbi == 0 )
    {
      isEqualUni = true;
    }
  }
#endif

  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
    , m_pImvTempCS ? m_pImvTempCS[wIdx] : NULL
    , 1
    , 0
#if JVET_L0646_GBI
    , &equGBiCost
#endif
  );

#if JVET_L0646_GBI
  if( g_GbiSearchOrder[gbiLoopIdx] == GBI_DEFAULT )
    m_pcInterSearch->setAffineModeSelected((bestCS->cus.front()->affine && !(bestCS->cus.front()->firstPU->mergeFlag)));

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  double skipTH = MAX_DOUBLE;
  skipTH = (m_pcEncCfg->getUseGBiFast() ? 1.05 : MAX_DOUBLE);
  if( equGBiCost > curBestCost * skipTH )
  {
    break;
  }

  if( m_pcEncCfg->getUseGBiFast() )
  {
    if( isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1 )
    {
      break;
    }
  }
  if( g_GbiSearchOrder[gbiLoopIdx] == GBI_DEFAULT && xIsGBiSkip(cu) && m_pcEncCfg->getUseGBiFast() )
  {
    break;
  }
 }  // for( UChar gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
#endif
}





bool EncCu::xCheckRDCostInterIMV( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  int iIMV = int( ( encTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT );
#if JVET_L0646_GBI
  m_pcInterSearch->setAffineModeSelected(false);
#endif
  // Only int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK( iIMV != 1 && iIMV != 2 && iIMV != 3, "Unsupported IMV Mode" );
  // Fast 4-Pel Mode

  EncTestMode encTestModeBase = encTestMode;                                        // copy for clearing non-IMV options
  encTestModeBase.opts        = EncTestModeOpts( encTestModeBase.opts & ETO_IMV );  // clear non-IMV options (is that intended?)

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingStructure* pcCUInfo2Reuse = nullptr;

#if JVET_L0646_GBI
  m_pcInterSearch->resetBufferedUniMotions();
  int gbiLoopNum = (tempCS->slice->isInterB() ? GBI_NUM : 1);
  gbiLoopNum = (pcCUInfo2Reuse != NULL ? 1 : gbiLoopNum);
  gbiLoopNum = (tempCS->slice->getSPS()->getSpsNext().getUseGBi() ? gbiLoopNum : 1);

  if( tempCS->area.lwidth() * tempCS->area.lheight() < GBI_SIZE_CONSTRAINT )
  {
    gbiLoopNum = 1;
  }

  double curBestCost = bestCS->cost;
  double equGBiCost = MAX_DOUBLE;

  for( int gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
  {
    if( m_pcEncCfg->getUseGBiFast() )
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if( blkCache )
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestGBiIdx = blkCache->getGbiIdx(bestCS->area);

        if( isBestInter && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT && g_GbiSearchOrder[gbiLoopIdx] != bestGBiIdx )
        {
          continue;
        }
      }
    }

    if( !tempCS->slice->getCheckLDC() )
    {
      if( gbiLoopIdx != 0 && gbiLoopIdx != 3 && gbiLoopIdx != 4 )
      {
        continue;
      }
    }

    if( m_pcEncCfg->getUseGBiFast() && tempCS->slice->getCheckLDC() && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT
      && (m_bestGbiIdx[0] >= 0 && g_GbiSearchOrder[gbiLoopIdx] != m_bestGbiIdx[0])
      && (m_bestGbiIdx[1] >= 0 && g_GbiSearchOrder[gbiLoopIdx] != m_bestGbiIdx[1]))
    {
      continue;
    }
#endif

  CodingUnit &cu = ( pcCUInfo2Reuse != nullptr ) ? *tempCS->getCU( partitioner.chType ) : tempCS->addCU( tempCS->area, partitioner.chType );

  if( pcCUInfo2Reuse == nullptr )
  {
    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
#if HEVC_TILES_WPP
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
#endif
    cu.skip             = false;
    cu.mmvdSkip = false;
  //cu.affine
    cu.predMode         = MODE_INTER;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;

    CU::addPUs( cu );
  }
  else
  {
    CHECK( cu.skip,                                "Mismatch" );
    CHECK( cu.qtDepth  != partitioner.currQtDepth, "Mismatch" );
    CHECK( cu.btDepth  != partitioner.currBtDepth, "Mismatch" );
    CHECK( cu.mtDepth  != partitioner.currMtDepth, "Mismatch" );
    CHECK( cu.depth    != partitioner.currDepth,   "Mismatch" );
  }

  cu.imv      = iIMV > 1 ? 2 : 1;
  cu.emtFlag  = false;

#if JVET_L0646_GBI
  bool testGbi;
  uint8_t gbiIdx;
#endif
  
  if( pcCUInfo2Reuse != nullptr )
  {
    // reuse the motion info from pcCUInfo2Reuse
    CU::resetMVDandMV2Int( cu, m_pcInterSearch );

#if JVET_L0646_GBI
    CHECK(cu.GBiIdx < 0 || cu.GBiIdx >= GBI_NUM, "cu.GBiIdx < 0 || cu.GBiIdx >= GBI_NUM");
    gbiIdx = CU::getValidGbiIdx(cu);
    testGbi = (gbiIdx != GBI_DEFAULT);
#endif

    if( !CU::hasSubCUNonZeroMVd( cu ) )
    {
#if JVET_L0293_CPR
      if (m_modeCtrl->useModeResult(encTestModeBase, tempCS, partitioner))
      {
        std::swap(tempCS, bestCS);
        // store temp best CI for next CU coding
        m_CurrCtx->best = m_CABACEstimator->getCtx();
      }
#else
      m_modeCtrl->useModeResult( encTestModeBase, tempCS, partitioner );
#endif
      return false;
    }
    else
    {
      m_pcInterSearch->motionCompensation( cu );
    }
  }
  else
  {
#if JVET_L0646_GBI 
    cu.GBiIdx = g_GbiSearchOrder[gbiLoopIdx];
    gbiIdx = cu.GBiIdx;
    testGbi = (gbiIdx != GBI_DEFAULT);
#endif

    m_pcInterSearch->predInterSearch( cu, partitioner );

#if JVET_L0646_GBI
    gbiIdx = CU::getValidGbiIdx(cu);
#endif
  }

#if JVET_L0646_GBI
  if( testGbi && gbiIdx == GBI_DEFAULT ) // Enabled GBi but the search results is uni.
  {
    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    continue;
  }
  CHECK(!(testGbi || (!testGbi && gbiIdx == GBI_DEFAULT)), " !( bTestGbi || (!bTestGbi && gbiIdx == GBI_DEFAULT ) )");

  bool isEqualUni = false;
  if( m_pcEncCfg->getUseGBiFast() )
  {
    if( cu.firstPU->interDir != 3 && testGbi == 0 )
    {
      isEqualUni = true;
    }
  }
#endif

  if( !CU::hasSubCUNonZeroMVd( cu ) )
  {
#if JVET_L0293_CPR
    if (m_modeCtrl->useModeResult(encTestModeBase, tempCS, partitioner))
    {
      std::swap(tempCS, bestCS);
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
    }
#else
    m_modeCtrl->useModeResult( encTestModeBase, tempCS, partitioner );
#endif
    return false;
  }

  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestModeBase, 0
    , NULL
    , true
    , 0
#if JVET_L0646_GBI
    , &equGBiCost
#endif
  );

#if JVET_L0646_GBI
  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  double skipTH = MAX_DOUBLE;
  skipTH = (m_pcEncCfg->getUseGBiFast() ? 1.05 : MAX_DOUBLE);
  if( equGBiCost > curBestCost * skipTH )
  {
    break;
  }

  if( m_pcEncCfg->getUseGBiFast() )
  {
    if( isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1 )
    {
      break;
    }
  }
  if( g_GbiSearchOrder[gbiLoopIdx] == GBI_DEFAULT && xIsGBiSkip(cu) && m_pcEncCfg->getUseGBiFast() )
  {
    break;
  }
 } // for( UChar gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
#endif

  return true;
}

void EncCu::xEncodeInterResidual( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, int residualPass
  , CodingStructure* imvCS
  , int emtMode
  , bool* bestHasNonResi
#if JVET_L0646_GBI
  , double* equGBiCost
#endif
  )
{
  if( residualPass == 1 && encTestMode.lossless )
  {
    return;
  }

  CodingUnit*            cu        = tempCS->getCU( partitioner.chType );
  double   bestCostInternal        = MAX_DOUBLE;
  double           bestCost        = bestCS->cost;
  const SPS&            sps        = *tempCS->sps;
  const int      maxSizeEMT        = EMT_INTER_MAX_CU_WITH_QTBT;
  bool              swapped        = false; // avoid unwanted data copy
  bool             reloadCU        = false;
  const bool considerEmtSecondPass = emtMode && sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT;

  int minEMTMode = 0;
  int maxEMTMode = (considerEmtSecondPass?1:0);

  // Not allow very big |MVd| to avoid CABAC crash caused by too large MVd. Normally no impact on coding performance.
  const int maxMvd = 1 << 15;
  const PredictionUnit& pu = *cu->firstPU;
  if (!cu->affine)
  {
    if ((pu.refIdx[0] >= 0 && (pu.mvd[0].getAbsHor() >= maxMvd || pu.mvd[0].getAbsVer() >= maxMvd))
      || (pu.refIdx[1] >= 0 && (pu.mvd[1].getAbsHor() >= maxMvd || pu.mvd[1].getAbsVer() >= maxMvd)))
    {
      return;
    }
  }
  else
  {
    for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
    {
      if (pu.refIdx[refList] >= 0)
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          if (pu.mvdAffi[refList][ctrlP].getAbsHor() >= maxMvd || pu.mvdAffi[refList][ctrlP].getAbsVer() >= maxMvd)
          {
            return;
          }
        }
      }
    }
  }

  if( emtMode == 2 )
  {
    minEMTMode = maxEMTMode = (cu->emtFlag?1:0);
  }

  for( int curEmtMode = minEMTMode; curEmtMode <= maxEMTMode; curEmtMode++ )
  {
    if( reloadCU )
    {
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( false == swapped )
      {
        tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
        tempCS->copyStructure( *bestCS, partitioner.chType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist     = 0;
      tempCS->fracBits = 0;
      tempCS->cost     = MAX_DOUBLE;
    }

    reloadCU    = true; // enable cu reloading
    cu->skip    = false;
    cu->emtFlag = curEmtMode;

    const bool skipResidual = residualPass == 1;
    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );

    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );

#if JVET_L0646_GBI
    if( ETM_INTER_ME == encTestMode.type )
    {
      if( equGBiCost != NULL )
      {
        if( tempCS->cost < (*equGBiCost) && cu->GBiIdx == GBI_DEFAULT )
        {
          (*equGBiCost) = tempCS->cost;
        }
      }
      else
      {
        CHECK(equGBiCost == NULL, "equGBiCost == NULL");
      }
      if( tempCS->slice->getCheckLDC() && !cu->imv && cu->GBiIdx != GBI_DEFAULT && tempCS->cost < m_bestGbiCost[1] )
      {
        if( tempCS->cost < m_bestGbiCost[0] )
        {
          m_bestGbiCost[1] = m_bestGbiCost[0];
          m_bestGbiCost[0] = tempCS->cost;
          m_bestGbiIdx[1] = m_bestGbiIdx[0];
          m_bestGbiIdx[0] = cu->GBiIdx;
        }
        else
        {
          m_bestGbiCost[1] = tempCS->cost;
          m_bestGbiIdx[1] = cu->GBiIdx;
        }
      }
    }
#endif

    double emtFirstPassCost = tempCS->cost;
    if( imvCS && (tempCS->cost < imvCS->cost) )
    {
      if( imvCS->cost != MAX_DOUBLE )
      {
        imvCS->initStructData( encTestMode.qp, encTestMode.lossless );
      }
      imvCS->copyStructure( *tempCS, partitioner.chType );
    }
    if( NULL != bestHasNonResi && (bestCostInternal > tempCS->cost) )
    {
      bestCostInternal = tempCS->cost;
      if (!(tempCS->getPU(partitioner.chType)->mhIntraFlag))
      *bestHasNonResi  = !cu->rootCbf;
    }

    if (cu->rootCbf == false)
    {
      if (tempCS->getPU(partitioner.chType)->mhIntraFlag)
      {
        tempCS->cost = MAX_DOUBLE;
        return;
      }
    }

#if WCG_EXT
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    //now we check whether the second pass should be skipped or not
    if( !curEmtMode && maxEMTMode )
    {
      const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
      const bool bCond1 = !cu->firstTU->cbf[COMPONENT_Y];

      const bool bCond3 = emtFirstPassCost > ( bestCost * thresholdToSkipEmtSecondPass );

      if( m_pcEncCfg->getFastInterEMT() && (bCond1 || bCond3 ) ) 
      {
        maxEMTMode = 0; // do not test EMT
      }
    }
  } //end emt loop
}


void EncCu::xEncodeDontSplit( CodingStructure &cs, Partitioner &partitioner )
{
  m_CABACEstimator->resetBits();

  {
    if( partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_flag( false, cs, partitioner );
    }
    if( partitioner.canSplit( CU_MT_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_mode_mt( CU_DONT_SPLIT, cs, partitioner );
    }
  }

  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost      = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );

}

#if REUSE_CU_RESULTS
void EncCu::xReuseCachedResult( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  const SPS &sps = *tempCS->sps;

  BestEncInfoCache* bestEncCache = dynamic_cast<BestEncInfoCache*>( m_modeCtrl );
  CHECK( !bestEncCache, "If this mode is chosen, mode controller has to implement the mode caching capabilities" );
  EncTestMode cachedMode;

  if( bestEncCache->setCsFrom( *tempCS, cachedMode, partitioner ) )
  {
    CodingUnit& cu = *tempCS->cus.front();
    partitioner.setCUData( cu );

    if( CU::isIntra( cu ) )
    {
      xReconIntraQT( cu );
    }
    else
    {
      xDeriveCUMV( cu );
      xReconInter( cu );
    }

    Distortion finalDistortion = 0;
    const int  numValidComponents = getNumberValidComponents( tempCS->area.chromaFormat );

    for( int comp = 0; comp < numValidComponents; comp++ )
    {
      const ComponentID compID = ComponentID( comp );

      if( CS::isDualITree( *tempCS ) && toChannelType( compID ) != partitioner.chType )
      {
        continue;
      }

      CPelBuf reco = tempCS->getRecoBuf( compID );
      CPelBuf org  = tempCS->getOrgBuf ( compID );

#if WCG_EXT
      if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
      {
        const CPelBuf orgLuma = tempCS->getOrgBuf(tempCS->area.blocks[COMPONENT_Y]);
        finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
      }
      else
#endif
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }

    m_CABACEstimator->getCtx() = m_CurrCtx->start;
    m_CABACEstimator->resetBits();

    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->coding_unit( cu, partitioner, cuCtx );

    tempCS->dist     = finalDistortion;
    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

    xEncodeDontSplit( *tempCS,         partitioner );
    xCheckDQP       ( *tempCS,         partitioner );
    xCheckBestMode  (  tempCS, bestCS, partitioner, cachedMode );
  }
  else
  {
    THROW( "Should never happen!" );
  }
}

#endif


//! \}
