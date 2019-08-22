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

/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#ifndef __INTRASEARCH__
#define __INTRASEARCH__

// Include files

#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/IntraPrediction.h"
#include "CommonLib/CrossCompPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"
#include "EncReshape.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class EncModeCtrl;

#if JVET_O0119_BASE_PALETTE_444
enum PLTScanMode
{
  PLT_SCAN_HORTRAV = 0,
  PLT_SCAN_VERTRAV = 1,
  NUM_PLT_SCAN = 2
};
class SortingElement
{
public:
  inline bool operator<(const SortingElement &other) const
  {
    return cnt > other.cnt;
  }
  SortingElement() {
    cnt = shift = lastCnt = 0;
    data[0] = data[1] = data[2] = 0;
    sumData[0] = sumData[1] = sumData[2] = 0;
  }
  uint32_t  getCnt() const        { return cnt; }
  void      setCnt(uint32_t val)  { cnt = val; }
  int       getSumData (int id) const   { return sumData[id]; }

  void resetAll(ComponentID compBegin, uint32_t numComp)
  {
    shift = lastCnt = 0;
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch] = 0;
      sumData[ch] = 0;
    }
  }
  void setAll(uint32_t* ui, ComponentID compBegin, uint32_t numComp)
  {
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch] = ui[ch];
    }
  }
  bool almostEqualData(SortingElement element, int errorLimit, const BitDepths& bitDepths, ComponentID compBegin, uint32_t numComp)
  {
    bool almostEqual = true;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      ChannelType chType = (comp > 0) ? CHANNEL_TYPE_CHROMA : CHANNEL_TYPE_LUMA;
      if ((std::abs(data[comp] - element.data[comp]) >> (bitDepths.recon[chType] - PLT_ENCBITDEPTH)) > errorLimit)
      {
        almostEqual = false;
        break;
      }
    }
    return almostEqual;
  }
  uint32_t getSAD(SortingElement element, const BitDepths& bitDepths, ComponentID compBegin, uint32_t numComp)
  {
    uint32_t sumAd = 0;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      ChannelType chType = (comp > 0) ? CHANNEL_TYPE_CHROMA : CHANNEL_TYPE_LUMA;
      sumAd += (std::abs(data[comp] - element.data[comp]) >> (bitDepths.recon[chType] - PLT_ENCBITDEPTH));
    }
    return sumAd;
  }
  void copyDataFrom(SortingElement element, ComponentID compBegin, uint32_t numComp)
  {
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      data[comp] = element.data[comp];
      sumData[comp] = data[comp];
    }
    shift = 0; lastCnt = 1;
  }
  void copyAllFrom(SortingElement element, ComponentID compBegin, uint32_t numComp)
  {
    copyDataFrom(element, compBegin, numComp);
    cnt = element.cnt;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      sumData[comp] = element.sumData[comp];
    }
    lastCnt = element.lastCnt; shift = element.shift;
  }
  void addElement(const SortingElement& element, ComponentID compBegin, uint32_t numComp)
  {
    cnt++;
    for (int i = compBegin; i<(compBegin + numComp); i++)
    {
      sumData[i] += element.data[i];
    }
    if (cnt>1 && cnt == 2 * lastCnt)
    {
      uint32_t rnd = 1 << shift;
      shift++;
      for (int i = compBegin; i<(compBegin + numComp); i++)
      {
        data[i] = (sumData[i] + rnd) >> shift;
      }
      lastCnt = cnt;
    }
  }
private:
  uint32_t cnt;
  int shift, lastCnt, data[3], sumData[3];
};
#endif
/// encoder search class
class IntraSearch : public IntraPrediction, CrossComponentPrediction
{
private:
  EncModeCtrl    *m_modeCtrl;
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_TBLOCKS];

  XUCache         m_unitCache;

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure ***m_pTempCS;
  CodingStructure ***m_pBestCS;

  CodingStructure **m_pSaveCS;

#if JVET_O0050_LOCAL_DUAL_TREE
  bool            m_saveCuCostInSCIPU;
  uint8_t         m_numCuInSCIPU;
  Area            m_cuAreaInSCIPU[NUM_INTER_CU_INFO_SAVE];
  double          m_cuCostInSCIPU[NUM_INTER_CU_INFO_SAVE];
#endif

  struct ModeInfo
  {
    bool     mipFlg; // CU::mipFlag
    int      mRefId; // PU::multiRefIdx
    uint8_t  ispMod; // CU::ispMode
    uint32_t modeId; // PU::intraDir[CHANNEL_TYPE_LUMA]

    ModeInfo() : mipFlg(false), mRefId(0), ispMod(NOT_INTRA_SUBPARTITIONS), modeId(0) {}
    ModeInfo(const bool mipf, const int mrid, const uint8_t ispm, const uint32_t mode) : mipFlg(mipf), mRefId(mrid), ispMod(ispm), modeId(mode) {}
    bool operator==(const ModeInfo cmp) const { return (mipFlg == cmp.mipFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId); }
  };
#if JVET_O0502_ISP_CLEANUP
  struct ModeInfoWithCost : public ModeInfo
  {
    double rdCost;
    ModeInfoWithCost() : ModeInfo(), rdCost(MAX_DOUBLE) {}
    ModeInfoWithCost(const bool mipf, const int mrid, const uint8_t ispm, const uint32_t mode, double cost) : ModeInfo(mipf, mrid, ispm, mode), rdCost(cost) {}
    bool operator==(const ModeInfoWithCost cmp) const { return (mipFlg == cmp.mipFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId && rdCost == cmp.rdCost); }
    static bool compareModeInfoWithCost(ModeInfoWithCost a, ModeInfoWithCost b) { return a.rdCost < b.rdCost; }
  };

  struct ISPTestedModeInfo
  {
    int    numCompSubParts;
    double rdCost;

    ISPTestedModeInfo() {}

    void setMode(int numParts, double cost)
    {
      numCompSubParts = numParts;
      rdCost = cost;
    }
    void clear()
    {
      numCompSubParts = -1;
      rdCost = MAX_DOUBLE;
    }
  };
  struct ISPTestedModesInfo
  {
    ISPTestedModeInfo                           intraMode[NUM_LUMA_MODE][2];
    bool                                        modeHasBeenTested[NUM_LUMA_MODE][2];
    int                                         numTotalParts[2];
    static_vector<int, FAST_UDI_MAX_RDMODE_NUM> testedModes[2];
    int                                         bestModeSoFar;
    ISPType                                     bestSplitSoFar;
    int                                         bestMode[2];
    double                                      bestCost[2];
    int                                         numTestedModes[2];
    int                                         candIndexInList[2];
    bool                                        stopTestingHorSplit;
    bool                                        stopTestingVerSplit;
    int                                         numOrigModesToTest;

    // set a tested mode results
    void setModeResults(ISPType splitType, int iModeIdx, int numCompletedParts, double rdCost, double currentBestCost)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      const int maxNumParts = numTotalParts[st];
      intraMode[iModeIdx][st].setMode(numCompletedParts, numCompletedParts == maxNumParts ? rdCost : MAX_DOUBLE);
      testedModes[st].push_back(iModeIdx);
      numTestedModes[st]++;
      modeHasBeenTested[iModeIdx][st] = true;
      if (numCompletedParts == maxNumParts && rdCost < bestCost[st])   // best mode update
      {
        bestMode[st] = iModeIdx;
        bestCost[st] = rdCost;
      }
      if (numCompletedParts == maxNumParts && rdCost < currentBestCost)   // best mode update
      {
        bestModeSoFar = iModeIdx;
        bestSplitSoFar = splitType;
      }
    }

    int getNumCompletedSubParts(ISPType splitType, int iModeIdx)
    {
      const unsigned st = splitType - 1;
      CHECK(st < 0 || st > 1, "The split type is invalid!");
      CHECK(iModeIdx < 0 || iModeIdx >(NUM_LUMA_MODE - 1), "The modeIdx is invalid");
      return modeHasBeenTested[iModeIdx][st] ? intraMode[iModeIdx][st].numCompSubParts : -1;
    }

    double getRDCost(ISPType splitType, int iModeIdx, int maxNumSubParts)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      return modeHasBeenTested[iModeIdx][st] && intraMode[iModeIdx][st].numCompSubParts == maxNumSubParts
        ? intraMode[iModeIdx][st].rdCost
        : -1;
    }

    // get a tested intra mode index
    int getTestedIntraMode(ISPType splitType, int pos)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      return pos < testedModes[st].size() ? testedModes[st].at(pos) : -1;
    }

    // set everything to default values
    void clear()
    {
      numTestedModes[0] = numTestedModes[1] = 0;
      candIndexInList[0] = candIndexInList[1] = 0;
      stopTestingHorSplit = false;
      stopTestingVerSplit = false;
      testedModes[0].clear();
      testedModes[1].clear();
      bestCost[0] = MAX_DOUBLE;
      bestCost[1] = MAX_DOUBLE;
      bestMode[0] = -1;
      bestMode[1] = -1;
      bestModeSoFar = -1;
      bestSplitSoFar = NOT_INTRA_SUBPARTITIONS;
      numOrigModesToTest = -1;
      memset(modeHasBeenTested, 0, sizeof(modeHasBeenTested));
    }
    void clearISPModeInfo(int idx)
    {
      intraMode[idx][0].clear();
      intraMode[idx][1].clear();
    }
  };

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_ispCandListHor, m_ispCandListVer;
  static_vector<ModeInfoWithCost, FAST_UDI_MAX_RDMODE_NUM> m_regIntraRDListWithCosts;

  ISPTestedModesInfo m_ispTestedModes;
#else
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrl;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlHor;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlVer;
#endif

  //cost variables for the EMT algorithm and new modes list
  double     m_bestModeCostStore[ NUM_LFNST_NUM_PER_SET ];                                    // RD cost of the best mode for each PU using DCT2
  bool       m_bestModeCostValid[ NUM_LFNST_NUM_PER_SET ];
  double     m_modeCostStore[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];                   // RD cost of each mode for each PU using DCT2
  ModeInfo   m_savedRdModeList[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];
  int32_t    m_savedNumRdModes[ NUM_LFNST_NUM_PER_SET ];

#if !JVET_O0502_ISP_CLEANUP
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_intraModeDiagRatio;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_intraModeHorVerRatio;
  static_vector<int,    FAST_UDI_MAX_RDMODE_NUM> m_intraModeTestedNormalIntra;
#endif

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedRdModeListLFNST;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedHadModeListLFNST;
  uint32_t                                         m_uiSavedNumRdModesLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedModeCostLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedHadListLFNST;

  PelStorage      m_tmpStorageLCU;
protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;
  RdCost*         m_pcRdCost;
  EncReshape*     m_pcReshape;

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;

  bool            m_isInitialized;

public:

  IntraSearch();
  ~IntraSearch();

  void init                       ( EncCfg*        pcEncCfg,
                                    TrQuant*       pcTrQuant,
                                    RdCost*        pcRdCost,
                                    CABACWriter*   CABACEstimator,
                                    CtxCache*      ctxCache,
                                    const uint32_t     maxCUWidth,
                                    const uint32_t     maxCUHeight,
                                    const uint32_t     maxTotalCUDepth
                                  , EncReshape*   m_pcReshape
                                  );

  void destroy                    ();

  CodingStructure****getSplitCSBuf() { return m_pSplitCS; }
  CodingStructure****getFullCSBuf () { return m_pFullCS; }
  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

  void setModeCtrl                ( EncModeCtrl *modeCtrl ) { m_modeCtrl = modeCtrl; }

#if JVET_O0050_LOCAL_DUAL_TREE
  bool getSaveCuCostInSCIPU       ()               { return m_saveCuCostInSCIPU; }
  void setSaveCuCostInSCIPU       ( bool b )       { m_saveCuCostInSCIPU = b;  }
  void setNumCuInSCIPU            ( uint8_t i )    { m_numCuInSCIPU = i; }
  void saveCuAreaCostInSCIPU      ( Area area, double cost );
  void initCuAreaCostInSCIPU      ();
  double findInterCUCost          ( CodingUnit &cu );
#endif

public:

  bool estIntraPredLumaQT         ( CodingUnit &cu, Partitioner& pm, const double bestCostSoFar  = MAX_DOUBLE, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false );
  void estIntraPredChromaQT       ( CodingUnit &cu, Partitioner& pm, const double maxCostAllowed = MAX_DOUBLE );
#if !JVET_O0525_REMOVE_PCM
  void IPCMSearch                 (CodingStructure &cs, Partitioner& partitioner);
#endif
#if JVET_O0119_BASE_PALETTE_444
  void PLTSearch                  ( CodingStructure &cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  void deriveRunAndCalcBits       ( CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp, PLTScanMode pltScanMode, uint64_t& bits);
#endif
  uint64_t xFracModeBitsIntra     (PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &compID);
  void invalidateBestModeCost     () { for( int i = 0; i < NUM_LFNST_NUM_PER_SET; i++ ) m_bestModeCostValid[ i ] = false; };

protected:

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------

#if !JVET_O0525_REMOVE_PCM
  void xEncPCM                    (CodingStructure &cs, Partitioner& partitioner, const ComponentID &compID);
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  void     xEncIntraHeader                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1 );
  void     xEncSubdivCbfQT                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
  uint64_t xGetIntraFracBitsQT                     ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
  uint64_t xGetIntraFracBitsQTSingleChromaComponent( CodingStructure &cs, Partitioner& pm, const ComponentID compID );

  uint64_t xGetIntraFracBitsQTChroma(TransformUnit& tu, const ComponentID &compID);
  void xEncCoeffQT                                 ( CodingStructure &cs, Partitioner& pm, const ComponentID compID, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );


  void xIntraCodingTUBlock        (TransformUnit &tu, const ComponentID &compID, const bool &checkCrossCPrediction, Distortion& ruiDist, const int &default0Save1Load2 = 0, uint32_t* numSig = nullptr, std::vector<TrMode>* trModes=nullptr, const bool loadTr=false );

  ChromaCbfs xRecurIntraChromaCodingQT( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE,                          const PartSplit ispType = TU_NO_ISP );
  bool       xRecurIntraCodingLumaQT  ( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, const bool ispIsCurrentWinner = false, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false );
#if JVET_O0502_ISP_CLEANUP
  bool       xIntraCodingLumaISP      ( CodingStructure& cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE );
#endif

  void encPredIntraDPCM( const ComponentID &compID, PelBuf &pOrg, PelBuf &pDst, const uint32_t &uiDirMode );
  static bool useDPCMForFirstPassIntraEstimation( const PredictionUnit &pu, const uint32_t &uiDirMode );

  template<typename T, size_t N>
#if JVET_O0925_MIP_SIMPLIFICATIONS
  void reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const PredictionUnit &pu, const bool fastMip);
#else
  void reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double thresholdHadCostConv);

  double m_bestCostNonMip;
#endif
#if JVET_O0119_BASE_PALETTE_444
  void   deriveRun       (      CodingStructure &cs, Partitioner& partitioner, ComponentID compBegin);
  double getRunBits      (const CodingUnit&      cu, uint32_t     run,         uint32_t    strPos,    PLTRunMode paletteRunMode, uint64_t*   indexBits, uint64_t* runBits, ComponentID compBegin);
  void   derivePLTLossy  (      CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  void   calcPixelPred   (      CodingStructure& cs, Partitioner& partitioner, uint32_t    yPos,      uint32_t xPos,             ComponentID compBegin, uint32_t  numComp);
  void   preCalcPLTIndex (      CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
#endif
#if JVET_O0502_ISP_CLEANUP
  void xGetNextISPMode                    ( ModeInfo& modeInfo, const ModeInfo* lastMode, const Size cuSize );
  void xFindAlreadyTestedNearbyIntraModes ( int currentIntraMode, int* leftIntraMode, int* rightIntraMode, ISPType ispOption, int windowSize );
  void xSortISPCandList                   ( double bestCostSoFar, double bestNonISPCost );
#endif
};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__
