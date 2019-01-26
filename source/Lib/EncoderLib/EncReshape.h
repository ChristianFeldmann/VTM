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

 /** \file     EncReshape.h
     \brief    encoder reshaping header and class (header)
 */

#ifndef __ENCRESHAPE__
#define __ENCRESHAPE__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include "CommonLib/Reshape.h"
#if JVET_M0427_INLOOP_RESHAPER

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct ModelInfo
{
  unsigned FullRangeInputFlag;
  unsigned ScaleIntPrec;
  unsigned ScaleInt;
  unsigned ScaleFracPrec;
  unsigned ScaleFrac;
  unsigned ScaleSign;
  unsigned OffsetIntPrec;
  unsigned OffsetInt;
  unsigned OffsetFracPrec;
  unsigned OffsetFrac;
  unsigned OffsetSign;
  unsigned MinMaxQPAbsPrec;
  unsigned MaxQPAbs;
  unsigned MaxQPSign;
  unsigned MinQPAbs;
  unsigned MinQPSign;
  unsigned ScaleAbs;
  unsigned OffsetAbs;
};

class EncReshape : public Reshape
{
private:
  bool                    m_bSrcReshaped;
  int                     m_picWidth;
  int                     m_picHeight;
  uint32_t                m_maxCUWidth;
  uint32_t                m_maxCUHeight;
  uint32_t                m_widthInCtus;
  uint32_t                m_heightInCtus;
  uint32_t                m_numCtuInFrame;
  bool                    m_bExceedSTD;
  std::vector<uint32_t>   m_uiBinImportance;
  int                     m_tcase;
  int                     m_rateAdpMode;
  bool                    m_bUseAdpCW;
  uint16_t                m_initCWAnalyze;
  ModelInfo               m_dQPModel;
  ReshapeCW               m_reshapeCW;
  Pel                     m_cwLumaWeight[PIC_CODE_CW_BINS];
  double                  m_chromaWeight;
  int                     m_chromaAdj;
public:

  EncReshape();
  ~EncReshape();

  void createEnc( int picWidth, int picHeight, uint32_t maxCUWidth, uint32_t maxCUHeight, int bitDepth);
  void destroy();

  bool getSrcReshaped() { return m_bSrcReshaped; }
  void setSrcReshaped(bool bPicReshaped) { m_bSrcReshaped = bPicReshaped; }

  void preAnalyzerSDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isCPR);
  void preAnalyzerHDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isCPR);
  void bubbleSortDsd(double *array, int * idx, int n);
  void swap(int *xp, int *yp) { int temp = *xp;  *xp = *yp;  *yp = temp; }
  void swap(double *xp, double *yp) { double temp = *xp;  *xp = *yp;  *yp = temp; }
  void deriveReshapeParametersSDRfromStats(uint32_t *, double*, double* dReshapeTH1, double* dReshapeTH2, bool *bIntraAdp, bool *bInterAdp);
  void deriveReshapeParameters(double *array, int start, int end, ReshapeCW respCW, double &alpha, double &beta);
  void initLUTfromdQPModel();
  int  calcEXP2(int val);
  void constructReshaperSDR();
  ReshapeCW *        getReshapeCW() { return &m_reshapeCW; }
  Pel * getWeightTable() { return m_cwLumaWeight; }
  double getCWeight() { return m_chromaWeight; }

  void initModelParam(double dScale = 0.015, double dOffset = -7.5, int QPMax = 6, int QPMin = -3)
  {
    /// dQP model:  dQP = clip3(QPMin, QPMax, dScale*Y+dOffset);
    m_dQPModel.FullRangeInputFlag = 0;
    m_dQPModel.ScaleIntPrec = 0;
    m_dQPModel.ScaleFracPrec = 16;
    m_dQPModel.OffsetIntPrec = 3;
    m_dQPModel.OffsetFracPrec = 1;
    m_dQPModel.MinMaxQPAbsPrec = 3;
    m_dQPModel.ScaleSign = dScale < 0 ? 1 : 0;
    m_dQPModel.ScaleAbs = unsigned((dScale < 0 ? -dScale : dScale) * (1 << m_dQPModel.ScaleFracPrec));
    m_dQPModel.ScaleInt = m_dQPModel.ScaleAbs >> m_dQPModel.ScaleFracPrec;
    m_dQPModel.ScaleFrac = m_dQPModel.ScaleAbs - (m_dQPModel.ScaleInt << m_dQPModel.ScaleFracPrec);
    m_dQPModel.OffsetSign = dOffset < 0 ? 1 : 0;
    m_dQPModel.OffsetAbs = unsigned((dOffset < 0 ? -dOffset : dOffset) * (1 << m_dQPModel.OffsetFracPrec));
    m_dQPModel.OffsetInt = m_dQPModel.OffsetAbs >> m_dQPModel.OffsetFracPrec;
    m_dQPModel.OffsetFrac = m_dQPModel.OffsetAbs - (m_dQPModel.OffsetInt << m_dQPModel.OffsetFracPrec);
    m_dQPModel.MaxQPSign = QPMax < 0 ? 1 : 0;
    m_dQPModel.MaxQPAbs = m_dQPModel.MaxQPSign ? -QPMax : QPMax;
    m_dQPModel.MinQPSign = QPMin < 0 ? 1 : 0;
    m_dQPModel.MinQPAbs = m_dQPModel.MinQPSign ? -QPMin : QPMin;
  }
};// END CLASS DEFINITION EncReshape

//! \}
#endif
#endif
