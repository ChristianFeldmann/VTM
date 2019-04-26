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

#if JVET_N0217_MATRIX_INTRAPRED
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

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrl;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlHor;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlVer;
#else
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrl;
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlHor;
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlVer;
#endif

  //cost variables for the EMT algorithm and new modes list
#if JVET_N0193_LFNST
  double     m_bestModeCostStore[ NUM_LFNST_NUM_PER_SET ];                                    // RD cost of the best mode for each PU using DCT2
  double     m_modeCostStore[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];                   // RD cost of each mode for each PU using DCT2
#if JVET_N0217_MATRIX_INTRAPRED
  ModeInfo   m_savedRdModeList[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];
  int32_t    m_savedNumRdModes[ NUM_LFNST_NUM_PER_SET ];
#else
  uint32_t   m_savedRdModeList[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ], m_savedNumRdModes[ NUM_LFNST_NUM_PER_SET ];
#endif
#if !JVET_N0217_MATRIX_INTRAPRED
  int        m_savedExtendRefList[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];
#endif
#endif

  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_intraModeDiagRatio;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_intraModeHorVerRatio;
  static_vector<int,    FAST_UDI_MAX_RDMODE_NUM> m_intraModeTestedNormalIntra;

#if JVET_N0193_LFNST
#if JVET_N0217_MATRIX_INTRAPRED
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedRdModeListLFNST;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedHadModeListLFNST;
#else
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedRdModeListLFNST;
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedHadModeListLFNST;
#endif
  uint32_t                                         m_uiSavedNumRdModesLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedModeCostLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedHadListLFNST;
#if !JVET_N0217_MATRIX_INTRAPRED
  static_vector<int,      FAST_UDI_MAX_RDMODE_NUM> m_iSavedExtendRefListLFNST;
#endif
#endif

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

public:

#if JVET_N0193_LFNST
  bool estIntraPredLumaQT         ( CodingUnit &cu, Partitioner& pm, const double bestCostSoFar  = MAX_DOUBLE, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false );
#else
  void estIntraPredLumaQT         ( CodingUnit &cu, Partitioner& pm, const double bestCostSoFar  = MAX_DOUBLE );
#endif
  void estIntraPredChromaQT       ( CodingUnit &cu, Partitioner& pm, const double maxCostAllowed = MAX_DOUBLE );
  void IPCMSearch                 (CodingStructure &cs, Partitioner& partitioner);
  uint64_t xFracModeBitsIntra     (PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &compID);

protected:

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------

  void xEncPCM                    (CodingStructure &cs, Partitioner& partitioner, const ComponentID &compID);

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
#if JVET_N0193_LFNST
  bool       xRecurIntraCodingLumaQT  ( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, const bool ispIsCurrentWinner = false, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false );
#else
  void       xRecurIntraCodingLumaQT  ( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, const bool ispIsCurrentWinner = false );
#endif


  void encPredIntraDPCM( const ComponentID &compID, PelBuf &pOrg, PelBuf &pDst, const uint32_t &uiDirMode );
  static bool useDPCMForFirstPassIntraEstimation( const PredictionUnit &pu, const uint32_t &uiDirMode );

#if JVET_N0217_MATRIX_INTRAPRED
  template<typename T, size_t N>
  void reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double thresholdHadCostConv);

  double m_bestCostNonMip;
#endif
};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__
