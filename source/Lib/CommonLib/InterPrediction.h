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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__


// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
#include "ContextModelling.h"
// forward declaration
class Mv;

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// Class definition
// ====================================================================================================================

class InterPrediction : public WeightPrediction
{
private:
  int m_shareState;

#if !JVET_O0055_INT_DMVR_DIS_BDOF
  Distortion  m_bioDistThres;
  Distortion  m_bioSubBlkDistThres;
  Distortion  m_bioPredSubBlkDist[MAX_NUM_PARTS_IN_CTU];
#endif

#if !JVET_O0304_SIMPLIFIED_BDOF
  int m_dotProduct1[BIO_TEMP_BUFFER_SIZE];
  int m_dotProduct2[BIO_TEMP_BUFFER_SIZE];
  int m_dotProduct3[BIO_TEMP_BUFFER_SIZE];
  int m_dotProduct5[BIO_TEMP_BUFFER_SIZE];
  int m_dotProduct6[BIO_TEMP_BUFFER_SIZE];
#endif

protected:
  InterpolationFilter  m_if;

  Pel*                 m_acYuvPred            [NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlock        [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlockTmp     [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMPONENT];


  ChromaFormat         m_currChromaFormat;

  ComponentID          m_maxCompIDToPred;      ///< tells the predictor to only process the components up to (inklusive) this one - useful to skip chroma components during RD-search

  RdCost*              m_pcRdCost;

  int                  m_iRefListIdx;
  PelStorage           m_triangleBuf;
  Mv*                  m_storedMv;
 /*buffers for bilinear Filter data for DMVR refinement*/
  Pel*                 m_cYuvPredTempDMVRL0;
  Pel*                 m_cYuvPredTempDMVRL1;
  int                  m_biLinearBufStride;
  /*buffers for padded data*/
  PelUnitBuf           m_cYuvRefBuffDMVRL0;
  PelUnitBuf           m_cYuvRefBuffDMVRL1;
  Pel*                 m_cRefSamplesDMVRL0[MAX_NUM_COMPONENT];
  Pel*                 m_cRefSamplesDMVRL1[MAX_NUM_COMPONENT];
  Mv m_pSearchOffset[25] = { Mv(-2,-2), Mv(-1,-2), Mv(0,-2), Mv(1,-2), Mv(2,-2),
                             Mv(-2,-1), Mv(-1,-1), Mv(0,-1), Mv(1,-1), Mv(2,-1),
                             Mv(-2, 0), Mv(-1, 0), Mv(0, 0), Mv(1, 0), Mv(2, 0),
                             Mv(-2, 1), Mv(-1, 1), Mv(0, 1), Mv(1, 1), Mv(2, 1),
                             Mv(-2, 2), Mv(-1, 2), Mv(0, 2), Mv(1, 2), Mv(2, 2) };
  uint64_t m_SADsArray[((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)];

#if JVET_O0070_PROF
  Pel                  m_gradBuf[2][2][(MAX_CU_SIZE + 2) * (MAX_CU_SIZE + 2)];
  int                  m_dMvBuf[2][16 * 2];
  bool                 m_applyPROF[2];
  bool                 m_skipPROF;
  bool                 m_encOnly;
  bool                 m_isBi;
#endif

  Pel*                 m_gradX0;
  Pel*                 m_gradY0;
  Pel*                 m_gradX1;
  Pel*                 m_gradY1;
  bool                 m_subPuMC;

#if JVET_O1170_IBC_VIRTUAL_BUFFER
  int                  m_IBCBufferWidth;
  PelStorage           m_IBCBuffer;
  void xIntraBlockCopy          (PredictionUnit &pu, PelUnitBuf &predBuf, const ComponentID compID);
#endif
  int             rightShiftMSB(int numer, int    denom);
  void            applyBiOptFlow(const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths);
#if !JVET_O0055_INT_DMVR_DIS_BDOF
  bool            xCalcBiPredSubBlkDist(const PredictionUnit &pu, const Pel* yuvSrc0, const int src0Stride, const Pel* yuvSrc1, const int src1Stride, const BitDepths &clipBitDepths);
#endif
  void xPredInterUni            ( const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi
                                  , const bool& bioApplied
                                  , const bool luma, const bool chroma
  );
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
  void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred, PelUnitBuf* yuvPredTmp = NULL );
#else
  void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred );
#endif
  void xPredInterBlk            ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
                                 , const bool& bioApplied
                                 , bool isIBC
#if JVET_O1164_RPR
                                 , const std::pair<int, int> scalingRatio = SCALE_1X
#endif
                                 , SizeType dmvrWidth = 0
                                 , SizeType dmvrHeight = 0
                                 , bool bilinearMC = false
                                 , Pel *srcPadBuf = NULL
                                 , int32_t srcPadStride = 0
                                 );

  void xAddBIOAvg4              (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng);
  void xBioGradFilter           (Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, int bitDepth);
  void xCalcBIOPar              (const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, int bitDepth);
  void xCalcBlkGradient         (int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize);
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
  void xWeightedAverage         ( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied, PelUnitBuf* yuvDstTmp = NULL );
#else
  void xWeightedAverage         ( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied );
#endif
#if JVET_O0070_PROF
  void xApplyBiPROF             (const PredictionUnit& pu, const CPelBuf& pcYuvSrc0, const CPelBuf& pcYuvSrc1, PelBuf& pcYuvDst, const ClpRng& clpRng);
#endif
#if JVET_O1164_RPR
  void xPredAffineBlk           ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng, const std::pair<int, int> scalingRatio = SCALE_1X );
#else
  void xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng );
#endif

  void xWeightedTriangleBlk     ( const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const bool splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 );

  static bool xCheckIdenticalMotion( const PredictionUnit& pu );

  void xSubPuMC(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X);
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
  void xSubPuBio(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X, PelUnitBuf* yuvDstTmp = NULL);
#else
  void xSubPuBio(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X);
#endif
  void destroy();


  MotionInfo      m_SubPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#if !JVET_O0258_REMOVE_CHROMA_IBC_FOR_DUALTREE
  void xChromaMC(PredictionUnit &pu, PelUnitBuf& pcYuvPred);
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel      *m_cacheModel;
#endif
public:
  InterPrediction();
  virtual ~InterPrediction();

#if JVET_O1170_IBC_VIRTUAL_BUFFER
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize);
#else
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC);
#endif

  // inter
  void    motionCompensation  (PredictionUnit &pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X
    , const bool luma = true, const bool chroma = true
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
    , PelUnitBuf* predBufWOBIO = NULL
#endif
  );
  void    motionCompensation  (PredictionUnit &pu, const RefPicList &eRefPicList = REF_PIC_LIST_X
    , const bool luma = true, const bool chroma = true
  );
  void    motionCompensation  (CodingUnit &cu,     const RefPicList &eRefPicList = REF_PIC_LIST_X
    , const bool luma = true, const bool chroma = true
  );

  void    motionCompensation4Triangle( CodingUnit &cu, MergeCtx &triangleMrgCtx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1 );
  void    weightedTriangleBlk        ( PredictionUnit &pu, const bool splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 );
#if JVET_O0297_DMVR_PADDING // For Dec speedup
  void xPrefetch(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma);
  void xPad(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId);
#else
  void xPrefetchPad(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId);
#endif
  void xFinalPaddedMCForDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1, PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied
    , const Mv startMV[NUM_REF_PIC_LIST_01]
#if JVET_O0297_DMVR_PADDING // For Dec speedup
    , bool blockMoved
#endif
  );
  void xBIPMVRefine(int bd, Pel *pRefL0, Pel *pRefL1, uint64_t& minCost, int16_t *deltaMV, uint64_t *pSADsArray, int width, int height);
  uint64_t xDMVRCost(int bitDepth, Pel* pRef, uint32_t refStride, const Pel* pOrg, uint32_t orgStride, int width, int height);
  void xinitMC(PredictionUnit& pu, const ClpRngs &clpRngs);
  void xProcessDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied );

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void    cacheAssign( CacheModel *cache );
#endif
  void    setShareState(int shareStateIn) {m_shareState = shareStateIn;}
#if ENABLE_SPLIT_PARALLELISM
  int     getShareState() const { return m_shareState; }
#endif
  static bool isSubblockVectorSpreadOverLimit( int a, int b, int c, int d, int predType );
#if JVET_O1170_IBC_VIRTUAL_BUFFER
  void xFillIBCBuffer(CodingUnit &cu);
#if JVET_O1170_CHECK_BV_AT_DECODER
  void resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize);
  void resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos);
  bool isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv);
#endif
#endif

#if JVET_O1164_RPR
  bool xPredInterBlkRPR( const std::pair<int, int>& scalingRatio, const PPS& pps, const CompArea &blk, const Picture* refPic, const Mv& mv, Pel* dst, const int dstStride, const bool bi, const bool wrapRef, const ClpRng& clpRng, const int filterIndex, const bool useAltHpelIf = false );
#endif
};

//! \}

#endif // __INTERPREDICTION__
