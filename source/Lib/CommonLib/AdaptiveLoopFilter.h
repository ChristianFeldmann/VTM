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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#include "CommonDef.h"

#include "Unit.h"
#include "UnitTools.h"

struct AlfClassifier
{
  AlfClassifier() {}
  AlfClassifier( uint8_t cIdx, uint8_t tIdx )
    : classIdx( cIdx ), transposeIdx( tIdx )
  {
  }

  uint8_t classIdx;
  uint8_t transposeIdx;
};

enum Direction
{
  HOR,
  VER,
  DIAG0,
  DIAG1,
  NUM_DIRECTIONS
};

class AdaptiveLoopFilter
{
public:
#if JVET_N0242_NON_LINEAR_ALF
  static inline int clipALF(const int clip, const short ref, const short val0, const short val1)
  {
    return Clip3<int>(-clip, +clip, val0-ref) + Clip3<int>(-clip, +clip, val1-ref);
  }

  static constexpr int AlfNumClippingValues[MAX_NUM_CHANNEL_TYPE] = { 4, 4 };
  static constexpr int MaxAlfNumClippingValues = 4;

#endif
  static constexpr int   m_NUM_BITS = 8;
  static constexpr int   m_CLASSIFICATION_BLK_SIZE = 32;  //non-normative, local buffer size
  static constexpr int m_ALF_UNUSED_CLASSIDX = 255;
  static constexpr int m_ALF_UNUSED_TRANSPOSIDX = 255;

  AdaptiveLoopFilter();
  virtual ~AdaptiveLoopFilter() {}
#if JVET_N0415_CTB_ALF
  void reconstructCoeffAPSs(CodingStructure& cs, bool luma, bool chroma, bool isRdo);
  void reconstructCoeff(AlfSliceParam& alfSliceParam, ChannelType channel, const bool isRdo, const bool isRedo = false);
  void ALFProcess(CodingStructure& cs);
#else
  void reconstructCoeff(AlfSliceParam& alfSliceParam, ChannelType channel, const bool isRedo = false);
  void ALFProcess( CodingStructure& cs, AlfSliceParam& alfSliceParam );
#endif
  void create( const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] );
  void destroy();
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
  static void deriveClassificationBlk(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift, int vbCTUHeight, int vbPos);
#else
  static void deriveClassificationBlk(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift);
#endif
  void deriveClassification( AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk );
#else
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
  static void deriveClassificationBlk(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blk, const int shift, int vbCTUHeight, int vbPos);
#else
  static void deriveClassificationBlk(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blk, const int shift);
#endif
  void deriveClassification( AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blk );
#endif
  void resetPCMBlkClassInfo(CodingStructure & cs, AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blk);
  template<AlfFilterType filtType>
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#else
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#else
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#endif
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs);
#else
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs);
#else
  static void filterBlk(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs);
#endif
#endif
#endif
  inline static int getMaxGolombIdx( AlfFilterType filterType )
  {
    return filterType == ALF_FILTER_5 ? 2 : 3;
  }

#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  void(*m_deriveClassificationBlk)(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift, int vbCTUHeight, int vbPos);
#else
  void(*m_deriveClassificationBlk)(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blk, const int shift, int vbCTUHeight, int vbPos);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  void(*m_deriveClassificationBlk)(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift);
#else
  void(*m_deriveClassificationBlk)(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blk, const int shift);
#endif
#endif

#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#else
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#else
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs, int vbCTUHeight, int vbPos);
#endif
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs);
#else
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, short* fClipSet, const ClpRng& clpRng, CodingStructure& cs);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs);
#else
  void(*m_filter5x5Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs);
  void(*m_filter7x7Blk)(AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng, CodingStructure& cs);
#endif
#endif
#endif

#ifdef TARGET_SIMD_X86
  void initAdaptiveLoopFilterX86();
  template <X86_VEXT vext>
  void _initAdaptiveLoopFilterX86();
#endif

protected:
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  bool isCrossedByVirtualBoundaries( const int xPos, const int yPos, const int width, const int height, bool& clipTop, bool& clipBottom, bool& clipLeft, bool& clipRight, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], const PPS* pps );
#endif
#if JVET_N0415_CTB_ALF
  static const int             m_classToFilterMapping[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES];
  static const int             m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF];
  short                        m_fixedFilterSetCoeffDec[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_coeffApsLuma[6][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#if JVET_N0242_NON_LINEAR_ALF
  short                        m_clippApsLuma[6][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
  short                        m_clipDefault[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
  bool                         m_created = false;
  short                        m_chromaCoeffFinal[MAX_NUM_ALF_LUMA_COEFF];
#endif
#if JVET_N0242_NON_LINEAR_ALF
  Pel                          m_alfClippingValues[MAX_NUM_CHANNEL_TYPE][MaxAlfNumClippingValues];
#endif
  std::vector<AlfFilterShape>  m_filterShapes[MAX_NUM_CHANNEL_TYPE];
  AlfClassifier**              m_classifier;
  short                        m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#if JVET_N0242_NON_LINEAR_ALF
  short                        m_clippFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_chromaClippFinal[MAX_NUM_ALF_LUMA_COEFF];
#endif
  int**                        m_laplacian[NUM_DIRECTIONS];
  uint8_t*                     m_ctuEnableFlag[MAX_NUM_COMPONENT];
  PelStorage                   m_tempBuf;
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  PelStorage                   m_tempBuf2;
#endif
  int                          m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];
  int                          m_picWidth;
  int                          m_picHeight;
  int                          m_maxCUWidth;
  int                          m_maxCUHeight;
  int                          m_maxCUDepth;
  int                          m_numCTUsInWidth;
  int                          m_numCTUsInHeight;
  int                          m_numCTUsInPic;
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
  int                          m_alfVBLumaPos;
  int                          m_alfVBChmaPos;
  int                          m_alfVBLumaCTUHeight;
  int                          m_alfVBChmaCTUHeight;
#endif
  ChromaFormat                 m_chromaFormat;
  ClpRngs                      m_clpRngs;
};

#endif
