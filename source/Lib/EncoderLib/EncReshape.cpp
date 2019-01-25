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

/** \file     EncReshape.cpp
\brief    encoder reshaper class
*/
#include "EncReshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#if JVET_M0427_INLOOP_RESHAPER
//! \ingroup EncLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncReshape::EncReshape()
{
  m_bCTUFlag     = false;
  m_bSrcReshaped = false;
  m_bRecReshaped = false;
  m_bReshape     = true;
  m_bExceedSTD   = false;
  m_uiCWOrgAnalyze = MAX_LUMA_RESHAPING_LUT_SIZE / PIC_ANALYZE_CW_BINS;
  m_uiCWOrg = MAX_LUMA_RESHAPING_LUT_SIZE / PIC_CODE_CW_BINS;
  m_tcase        = 0;
  m_rateAdpMode  = 0;
  m_chromaAdj    = 0;
}

EncReshape::~EncReshape()
{
}

void  EncReshape::create_enc(int picWidth, int picHeight, uint32_t maxCUWidth, uint32_t maxCUHeight)
{
  if (forwardReshapingLUT.empty())
    forwardReshapingLUT.resize(MAX_LUMA_RESHAPING_LUT_SIZE, 0);
  if (inverseReshapingLUT.empty())
    inverseReshapingLUT.resize(MAX_LUMA_RESHAPING_LUT_SIZE,0);
  if (m_uiBinCWAll.empty())
    m_uiBinCWAll.resize(PIC_ANALYZE_CW_BINS);
  if (m_uiBinImportance.empty())
    m_uiBinImportance.resize(PIC_ANALYZE_CW_BINS);
  if (m_ReshapePivot.empty())
    m_ReshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (ChromaAdjHelpLUT.empty())
    ChromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 2048);

  m_sliceReshapeInfo.setUseSliceReshaper(true);
  m_sliceReshapeInfo.setSliceReshapeChromaAdj(true);
  m_sliceReshapeInfo.setSliceReshapeModelPresentFlag(true);
  m_sliceReshapeInfo.reshape_model_min_bin_idx = 0;
  m_sliceReshapeInfo.reshape_model_max_bin_idx = PIC_CODE_CW_BINS - 1;
  memset(m_sliceReshapeInfo.reshape_model_bin_CW_delta, 0, (PIC_CODE_CW_BINS) * sizeof(int));

  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;
  m_widthInCtus = (m_picWidth + m_maxCUWidth - 1) / m_maxCUWidth;
  m_heightInCtus = (m_picHeight + m_maxCUHeight - 1) / m_maxCUHeight;
  m_numCtuInFrame = m_widthInCtus * m_heightInCtus;
}

void  EncReshape::destroy()
{
}

/**
-Perform HDR set up
\param   pcPic describe pointer of current coding picture
\param   sliceType describe the slice type
*/
void EncReshape::preAnalyzerHDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isCPR)
{
  m_sliceReshapeInfo.slice_reshaper_enable_flag = true;
  if (reshapeCW.RspIntraPeriod == 1)
  {
    if (pcPic->getPOC() == 0)          { m_sliceReshapeInfo.slice_reshaper_model_present_flag = true;  }
    else                               { m_sliceReshapeInfo.slice_reshaper_model_present_flag = false; }
  }
  else
  {
    if (sliceType == I_SLICE || (sliceType==P_SLICE && isCPR) )             { m_sliceReshapeInfo.slice_reshaper_model_present_flag = true;  }
    else                                                                    { m_sliceReshapeInfo.slice_reshaper_model_present_flag = false; }
  }
  if ((sliceType == I_SLICE || (sliceType == P_SLICE && isCPR)) && isDualT) { m_sliceReshapeInfo.uiReshapeChromaAdj = 0;                    }
  else                                                                      { m_sliceReshapeInfo.uiReshapeChromaAdj = 1;                    }
}

/**
-Perform picture analysis for SDR
\param   pcPic describe pointer of current coding picture
\param   sliceType describe the slice type
\param   reshapeCW describe some input info
*/
void EncReshape::preAnalyzerSDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isCPR)
{
  m_sliceReshapeInfo.slice_reshaper_model_present_flag = true;
  m_sliceReshapeInfo.slice_reshaper_enable_flag = true;

  int modIP = pcPic->getPOC() - pcPic->getPOC() / reshapeCW.RspFpsToIp * reshapeCW.RspFpsToIp;

  if (sliceType == I_SLICE || (reshapeCW.RspIntraPeriod == -1 && modIP == 0) || (sliceType== P_SLICE && isCPR))
  {
    if (m_sliceReshapeInfo.slice_reshaper_model_present_flag == true)
    {
      uint32_t uiStdMin = 16 * 4;
      uint32_t uiStdMax = 235 * 4;
      int  bin_len = MAX_LUMA_RESHAPING_LUT_SIZE / PIC_ANALYZE_CW_BINS;
      uint32_t min_start_bin_idx, max_end_bin_idx;

      m_reshapeCW = reshapeCW;

      for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
      {
        m_uiBinImportance[b] = 0;
        m_uiBinCWAll[b] = bin_len;
      }

      min_start_bin_idx = int(floor((double(uiStdMin) / double(bin_len))));
      max_end_bin_idx = int(floor((double(uiStdMax) / double(bin_len))));

      m_sliceReshapeInfo.reshape_model_min_bin_idx = min_start_bin_idx;
      m_sliceReshapeInfo.reshape_model_max_bin_idx = max_end_bin_idx;

      PelBuf picY = pcPic->getOrigBuf(COMPONENT_Y);
      const int iWidth = picY.width;
      const int iHeight = picY.height;
      const int iStride = picY.stride;

      double dBlockBinVarSum[PIC_ANALYZE_CW_BINS] = { 0.0 };
      uint32_t   dBlockBinCnt[PIC_ANALYZE_CW_BINS] = { 0 };

      const uint32_t uiWinSize = PIC_ANALYZE_WIN_SIZE;
      const uint32_t uiWinLens = (uiWinSize - 1) >> 1;

      int64_t tempSq = 0;
      int64_t leftSum = 0, leftSumSq = 0;
      int64_t *leftColSum = new int64_t[iWidth];
      int64_t *leftColSumSq = new int64_t[iWidth];
      memset(leftColSum, 0, iWidth * sizeof(int64_t));
      memset(leftColSumSq, 0, iWidth * sizeof(int64_t));
      int64_t topSum = 0, topSumSq = 0;
      int64_t *topRowSum = new int64_t[iHeight];
      int64_t *topRowSumSq = new int64_t[iHeight];
      memset(topRowSum, 0, iHeight * sizeof(int64_t));
      memset(topRowSumSq, 0, iHeight * sizeof(int64_t));
      int64_t *topColSum = new int64_t[iWidth];
      int64_t *topColSumSq = new int64_t[iWidth];
      memset(topColSum, 0, iWidth * sizeof(int64_t));
      memset(topColSumSq, 0, iWidth * sizeof(int64_t));

      for (uint32_t y = 0; y < iHeight; y++)
      {
        for (uint32_t x = 0; x < iWidth; x++)
        {
          const Pel pPxlY = picY.buf[x];
          int64_t uiSum = 0;
          int64_t uiSumSq = 0;
          uint32_t uiNumPixInPart = 0;

          uint32_t y1 = std::max((int)(y - uiWinLens), 0);
          uint32_t y2 = std::min((int)(y + uiWinLens), (iHeight - 1));
          uint32_t x1 = std::max((int)(x - uiWinLens), 0);
          uint32_t x2 = std::min((int)(x + uiWinLens), (iWidth - 1));


          uint32_t bx = 0, by = 0;
          const Pel *pWinY = &picY.buf[0];
          uiNumPixInPart = (x2 - x1 + 1) * (y2 - y1 + 1);

          if (x == 0 && y == 0)           // for the 1st Pixel, calc all points
          {
            for (by = y1; by <= y2; by++)
            {
              for (bx = x1; bx <= x2; bx++)
              {
                tempSq = pWinY[bx] * pWinY[bx];
                leftSum += pWinY[bx];
                leftSumSq += tempSq;
                leftColSum[bx] += pWinY[bx];
                leftColSumSq[bx] += tempSq;
                topColSum[bx] += pWinY[bx];
                topColSumSq[bx] += tempSq;
                topRowSum[by] += pWinY[bx];
                topRowSumSq[by] += tempSq;
              }
              pWinY += iStride;
            }
            topSum = leftSum;
            topSumSq = leftSumSq;
            uiSum = leftSum;
            uiSumSq = leftSumSq;
          }
          else if (x == 0 && y > 0)       // for the 1st column, calc the bottom stripe
          {
            if (y < iHeight - uiWinLens)
            {
              pWinY += uiWinLens*iStride;
              topRowSum[y + uiWinLens] = 0;
              topRowSumSq[y + uiWinLens] = 0;
              for (bx = x1; bx <= x2; bx++)
              {
                topRowSum[y + uiWinLens] += pWinY[bx];
                topRowSumSq[y + uiWinLens] += pWinY[bx] * pWinY[bx];
              }
              topSum += topRowSum[y + uiWinLens];
              topSumSq += topRowSumSq[y + uiWinLens];
            }
            if (y > uiWinLens)
            {
              topSum -= topRowSum[y - 1 - uiWinLens];
              topSumSq -= topRowSumSq[y - 1 - uiWinLens];
            }

            memset(leftColSum, 0, iWidth * sizeof(int64_t));
            memset(leftColSumSq, 0, iWidth * sizeof(int64_t));
            pWinY = &picY.buf[0];
            pWinY -= (y <= uiWinLens ? y : uiWinLens)*iStride;
            for (by = y1; by <= y2; by++)
            {
              for (bx = x1; bx <= x2; bx++)
              {
                leftColSum[bx] += pWinY[bx];
                leftColSumSq[bx] += pWinY[bx] * pWinY[bx];
              }
              pWinY += iStride;
            }

            leftSum = topSum;
            leftSumSq = topSumSq;
            uiSum = topSum;
            uiSumSq = topSumSq;
          }

          else if (x > 0)
          {
            if (x < iWidth - uiWinLens)
            {
              pWinY -= (y <= uiWinLens ? y : uiWinLens)*iStride;
              if (y == 0)                 // for the 1st row, calc the right stripe
              {
                leftColSum[x + uiWinLens] = 0;
                leftColSumSq[x + uiWinLens] = 0;
                for (by = y1; by <= y2; by++)
                {
                  leftColSum[x + uiWinLens] += pWinY[x + uiWinLens];
                  leftColSumSq[x + uiWinLens] += pWinY[x + uiWinLens] * pWinY[x + uiWinLens];
                  pWinY += iStride;
                }
              }
              else                        // for the main area, calc the B-R point 
              {
                leftColSum[x + uiWinLens] = topColSum[x + uiWinLens];
                leftColSumSq[x + uiWinLens] = topColSumSq[x + uiWinLens];
                if (y < iHeight - uiWinLens)
                {
                  pWinY = &picY.buf[0];
                  pWinY += uiWinLens * iStride;
                  leftColSum[x + uiWinLens] += pWinY[x + uiWinLens];
                  leftColSumSq[x + uiWinLens] += pWinY[x + uiWinLens] * pWinY[x + uiWinLens];
                }
                if (y > uiWinLens)
                {
                  pWinY = &picY.buf[0];
                  pWinY -= (uiWinLens + 1) * iStride;
                  leftColSum[x + uiWinLens] -= pWinY[x + uiWinLens];
                  leftColSumSq[x + uiWinLens] -= pWinY[x + uiWinLens] * pWinY[x + uiWinLens];
                }
              }
              topColSum[x + uiWinLens] = leftColSum[x + uiWinLens];
              topColSumSq[x + uiWinLens] = leftColSumSq[x + uiWinLens];
              leftSum += leftColSum[x + uiWinLens];
              leftSumSq += leftColSumSq[x + uiWinLens];
            }
            if (x > uiWinLens)
            {
              leftSum -= leftColSum[x - 1 - uiWinLens];
              leftSumSq -= leftColSumSq[x - 1 - uiWinLens];
            }
            uiSum = leftSum;
            uiSumSq = leftSumSq;
          }

          double dAverage = double(uiSum) / uiNumPixInPart;
          double dVariance = double(uiSumSq) / uiNumPixInPart - dAverage * dAverage;
          double dVarLog10 = log10(dVariance + 1.0);

          uint32_t uiBinNum = (uint32_t)floor((double)pPxlY / (double)PIC_ANALYZE_CW_BINS);
          dBlockBinVarSum[uiBinNum] += dVarLog10;
          dBlockBinCnt[uiBinNum]++;
        }
        picY.buf += iStride;
      }

      delete[] topColSum;
      delete[] topColSumSq;
      delete[] topRowSum;
      delete[] topRowSumSq;
      delete[] leftColSum;
      delete[] leftColSumSq;

      for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
      {
        if (dBlockBinCnt[b] > 0)
          dBlockBinVarSum[b] = dBlockBinVarSum[b] / dBlockBinCnt[b];
      }

      m_bReshape = true;
      m_bExceedSTD = false;
      m_bUseAdpCW = false;
      m_chromaWeight = 1.0;
      m_sliceReshapeInfo.uiReshapeChromaAdj = 1;
      bool   bIntraAdp = false;
      bool   bInterAdp = true;
      double dReshapeTH1 = 0.0;
      double dReshapeTH2 = 5.0;
      deriveReshapeParametersSDRfromStats(dBlockBinCnt, dBlockBinVarSum, &dReshapeTH1, &dReshapeTH2, &bIntraAdp, &bInterAdp);

      if (m_rateAdpMode == 2 && reshapeCW.RspBaseQP <= 22)
      {
        bIntraAdp = false;
        bInterAdp = false;
      }

      m_sliceReshapeInfo.slice_reshaper_enable_flag = bIntraAdp;

      if (!bIntraAdp && !bInterAdp)
      {
        m_sliceReshapeInfo.slice_reshaper_model_present_flag = false;
        m_bReshape = false;
        return;
      }

      if (m_bExceedSTD)
      {
        min_start_bin_idx = 2;
        max_end_bin_idx = 29;
        for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
        {
          if (dBlockBinCnt[b] > 0 && b < min_start_bin_idx)
            min_start_bin_idx = b;
          if (dBlockBinCnt[b] > 0 && b > max_end_bin_idx)
            max_end_bin_idx = b;
        }
        m_sliceReshapeInfo.reshape_model_min_bin_idx = min_start_bin_idx;
        m_sliceReshapeInfo.reshape_model_max_bin_idx = max_end_bin_idx;
      }

      if (reshapeCW.RspBaseQP <= 22 && m_rateAdpMode == 1)
      {
        for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
        {
          if (i >= min_start_bin_idx && i <= max_end_bin_idx)
            m_uiBinCWAll[i] = m_uiCWOrgAnalyze + 1;
          else
            m_uiBinCWAll[i] = 0;
        }
      }
      else if (m_bUseAdpCW)
      {
        double Alpha = 1.0, Beta = 0.0;
        deriveReshapeParameters(dBlockBinVarSum, min_start_bin_idx, max_end_bin_idx, m_reshapeCW, Alpha, Beta);
        for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
        {
          if (i >= min_start_bin_idx && i <= max_end_bin_idx)
            m_uiBinCWAll[i] = (uint32_t)round(Alpha*dBlockBinVarSum[i] + Beta);
          else
            m_uiBinCWAll[i] = 0;
        }
      }
      else
      {
        for (int b = min_start_bin_idx; b <= max_end_bin_idx; b++)
        {
          if (dBlockBinVarSum[b] < dReshapeTH1)
            m_uiBinImportance[b] = 2;
          else if (dBlockBinVarSum[b] > dReshapeTH2)
            m_uiBinImportance[b] = 3;
          else
            m_uiBinImportance[b] = 1;
        }

        for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
        {
          if (m_uiBinImportance[i] == 0)
            m_uiBinCWAll[i] = 0;
          else if (m_uiBinImportance[i] == 1)
            m_uiBinCWAll[i] = m_uiCWOrgAnalyze + 1;
          else if (m_uiBinImportance[i] == 2)
            m_uiBinCWAll[i] = m_reshapeCW.BinCW[0];
          else if (m_uiBinImportance[i] == 3)
            m_uiBinCWAll[i] = m_reshapeCW.BinCW[1];
          else
            THROW("SDR Reshape Bin Importance not supported");
        }
      }
      if (m_reshapeCW.RspPicSize <= 1497600 && reshapeCW.RspIntraPeriod == -1 && modIP == 0 && sliceType != I_SLICE)
      {
        m_sliceReshapeInfo.slice_reshaper_enable_flag = false;
      }

    }
    m_chromaAdj = m_sliceReshapeInfo.uiReshapeChromaAdj;
    if ((sliceType == I_SLICE || (sliceType == P_SLICE && isCPR)) && isDualT)
    {
        m_sliceReshapeInfo.uiReshapeChromaAdj = 0;
    }
  }
  else // Inter slices
  {
    m_sliceReshapeInfo.slice_reshaper_model_present_flag = false;
    m_sliceReshapeInfo.uiReshapeChromaAdj = m_chromaAdj;

    if (!m_bReshape)
    {
      m_sliceReshapeInfo.slice_reshaper_enable_flag = false;
    }
    else
    {
      const int cTid = m_reshapeCW.Tid;
      bool enableRsp = m_tcase == 5 ? false : (m_tcase < 5 ? (cTid < m_tcase + 1 ? false : true) : (cTid <= 10 - m_tcase ? true : false));
      m_sliceReshapeInfo.slice_reshaper_enable_flag = enableRsp;
    }
  }
}

// Bubble Sort to  descending order with index
void EncReshape::bubbleSortDsd(double* array, int * idx, int n)
{
  int i, j;
  bool swapped;
  for (i = 0; i < n - 1; i++)
  {
    swapped = false;
    for (j = 0; j < n - i - 1; j++)
    {
      if (array[j] < array[j + 1])
      {
        swap(&array[j], &array[j + 1]);
        swap(&idx[j], &idx[j + 1]);
        swapped = true;
      }
    }
    if (swapped == false)
      break;
  }
}

void EncReshape::deriveReshapeParametersSDRfromStats(uint32_t * dBlockBinCnt, double *dBlockBinVarSum, double* dReshapeTH1, double* dReshapeTH2, bool *bIntraAdp, bool *bInterAdp)
{
  int    BinIdxSortDsd[PIC_ANALYZE_CW_BINS] = { 0 };
  double BinVarSortDsd[PIC_ANALYZE_CW_BINS] = { 0.0 };
  double BinHist[PIC_ANALYZE_CW_BINS] = { 0.0 };
  double BinVarSortDsdCDF[PIC_ANALYZE_CW_BINS] = { 0.0 };

  double maxBinVar = 0.0, meanBinVar = 0.0, minBinVar = 5.0;
  int    nonZeroBinCt = 0;
  for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
  {
    BinHist[b] = (double)dBlockBinCnt[b] / (double)(m_reshapeCW.RspPicSize);
    if (BinHist[b] > 0.001)
    {
      nonZeroBinCt++;
      meanBinVar += dBlockBinVarSum[b];
      if (dBlockBinVarSum[b] > maxBinVar)        {        maxBinVar = dBlockBinVarSum[b];      }
      if (dBlockBinVarSum[b] < minBinVar)        {        minBinVar = dBlockBinVarSum[b];      }
    }
    BinVarSortDsd[b] = dBlockBinVarSum[b];
    BinIdxSortDsd[b] = b;
  }
  if ((BinHist[0] + BinHist[1] + BinHist[PIC_ANALYZE_CW_BINS - 2] + BinHist[PIC_ANALYZE_CW_BINS - 1]) > 0.01)   {    m_bExceedSTD = true;  }
  if ((BinHist[PIC_ANALYZE_CW_BINS - 2] + BinHist[PIC_ANALYZE_CW_BINS - 1]) > 0.01)   {    *bInterAdp = false;    return;   }
  else                                                                                {    *bInterAdp = true;               }

  meanBinVar = meanBinVar / (double)nonZeroBinCt;
  bubbleSortDsd(BinVarSortDsd, BinIdxSortDsd, PIC_ANALYZE_CW_BINS);
  BinVarSortDsdCDF[0] = BinHist[BinIdxSortDsd[0]];
  for (int b = 1; b < PIC_ANALYZE_CW_BINS; b++)
  {
    BinVarSortDsdCDF[b] = BinVarSortDsdCDF[b - 1] + BinHist[BinIdxSortDsd[b]];
  }

  int firstBinVarLessThanVal1 = 0; // Val1 = 3.5
  int firstBinVarLessThanVal2 = 0; // Val2 = 3.0
  int firstBinVarLessThanVal3 = 0; // Val3 = 2.5
  int firstBinVarLessThanVal4 = 0; // Val4 = 2.0
  for (int b = 0; b < PIC_ANALYZE_CW_BINS - 1; b++)
  {
    if (BinVarSortDsd[b] > 3.5)     {      firstBinVarLessThanVal1 = b + 1;    }
    if (BinVarSortDsd[b] > 3.0)     {      firstBinVarLessThanVal2 = b + 1;    }
    if (BinVarSortDsd[b] > 2.5)     {      firstBinVarLessThanVal3 = b + 1;    }
    if (BinVarSortDsd[b] > 2.0)     {      firstBinVarLessThanVal4 = b + 1;    }
  }

  m_reshapeCW.BinCW[0] = 38;
  m_reshapeCW.BinCW[1] = 28;

  if (m_reshapeCW.RspIntraPeriod == -1)
  {
    *bIntraAdp = true;
    if (m_reshapeCW.RspPicSize > 1497600)
    {
      m_reshapeCW.BinCW[0] = 36;
      *dReshapeTH1 = 2.4;
      *dReshapeTH2 = 4.5;
      m_rateAdpMode = 2;

      if (meanBinVar >= 2.52)
      {
        if (BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5)
        {
          *dReshapeTH1 = 2.5;
          *dReshapeTH2 = 3.0;
        }
        else if (BinVarSortDsdCDF[firstBinVarLessThanVal2] < 0.1 && BinVarSortDsdCDF[firstBinVarLessThanVal1] > 0.02)
        {
          *dReshapeTH1 = 2.2;
        }
        else if (BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.25)
        {
          m_reshapeCW.BinCW[1] = 30;
          *dReshapeTH1 = 2.0;
          m_rateAdpMode = 0;
        }
        else
        {
          m_reshapeCW.BinCW[1] = 30;
          m_rateAdpMode = 1;
        }
      }
    }
    else if (m_reshapeCW.RspPicSize > 660480)
    {
      m_reshapeCW.BinCW[0] = 34;
      *dReshapeTH1 = 3.4;
      *dReshapeTH2 = 4.0;
      m_rateAdpMode = 2;

      if (BinVarSortDsdCDF[firstBinVarLessThanVal4] > 0.6)
      {
        if (maxBinVar < 3.5)
        {
          m_bUseAdpCW = true;
          m_reshapeCW.BinCW[0] = 38;
        }
        else
        {
          m_reshapeCW.BinCW[0] = 40;
          *dReshapeTH1 = 2.2;
          *dReshapeTH2 = 4.5;
          m_rateAdpMode = 0;
        }
      }
      else
      {
        if (maxBinVar > 3.3)
        {
          m_reshapeCW.BinCW[1] = 30;
        }
        else
        {
          m_reshapeCW.BinCW[1] = 28;
        }
      }
    }
    else if (m_reshapeCW.RspPicSize > 249600)
    {
      m_reshapeCW.BinCW[0] = 36;
      *dReshapeTH1 = 2.5;
      *dReshapeTH2 = 4.5;

      if (m_bExceedSTD)
      {
        m_reshapeCW.BinCW[0] = 36;
        m_reshapeCW.BinCW[1] = 30;
      }
      if (minBinVar > 2.6)
      {
        *dReshapeTH1 = 3.0;
      }
      else {
        double diff1 = BinVarSortDsdCDF[firstBinVarLessThanVal4] - BinVarSortDsdCDF[firstBinVarLessThanVal3];
        double diff2 = BinVarSortDsdCDF[firstBinVarLessThanVal2] - BinVarSortDsdCDF[firstBinVarLessThanVal1];
        if (diff1 > 0.4 || BinVarSortDsdCDF[firstBinVarLessThanVal1] > 0.1)
        {
          m_bUseAdpCW = true;
          m_rateAdpMode = 1;
        }
        else if (diff2 <= 0.1 && BinVarSortDsdCDF[firstBinVarLessThanVal4] > 0.99 && BinVarSortDsdCDF[firstBinVarLessThanVal3] > 0.642 && BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.03)
        {
          m_bUseAdpCW = true;
          m_rateAdpMode = 1;
        }
        else
        {
          m_rateAdpMode = 2;
        }
      }
    }
    else
    {
      m_reshapeCW.BinCW[0] = 36;
      *dReshapeTH1 = 2.6;
      *dReshapeTH2 = 4.5;

      if (BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5 && maxBinVar < 4.7)
      {
        *dReshapeTH1 = 3.2;
        m_rateAdpMode = 1;
      }
    }
  }
  else if (m_reshapeCW.RspIntraPeriod == 1)
  {
    *bIntraAdp = true;
    if (m_reshapeCW.RspPicSize > 5184000)
    {
      *dReshapeTH1 = 2.0;
      *dReshapeTH2 = 3.0;
      m_rateAdpMode = 2;

      if (maxBinVar > 2.4)
      {
        if (BinVarSortDsdCDF[firstBinVarLessThanVal4] > 0.88) 
        {
          if (maxBinVar < 2.695)
          {
            *dReshapeTH2 = 2.2;
          }
          else 
          {
            if (BinVarSortDsdCDF[firstBinVarLessThanVal3] < 0.45)
            {
              *dReshapeTH1 = 2.5;
              *dReshapeTH2 = 4.0;
              m_reshapeCW.BinCW[0] = 36;
              m_sliceReshapeInfo.uiReshapeChromaAdj = 0;
              m_rateAdpMode = 0;
            }
            else
            {
              m_bUseAdpCW = true;
              m_reshapeCW.BinCW[0] = 36;
              m_reshapeCW.BinCW[1] = 30;
            }
          }
        }
        else 
        {
          if (maxBinVar > 2.8)
          {
            *dReshapeTH1 = 2.2;
            *dReshapeTH2 = 4.0;
            m_reshapeCW.BinCW[0] = 36;
            m_sliceReshapeInfo.uiReshapeChromaAdj = 0;
          }
          else
          {
            m_bUseAdpCW = true;
            m_reshapeCW.BinCW[0] = 38;
            m_reshapeCW.BinCW[1] = 28;
          }
        }
      }
      else
      {
        if (maxBinVar > 2.24)
        {
          m_bUseAdpCW = true;
          m_reshapeCW.BinCW[0] = 34;
          m_reshapeCW.BinCW[1] = 30;
        }
      }
    }
    else if (m_reshapeCW.RspPicSize > 1497600)
    {
      *dReshapeTH1 = 2.0;
      *dReshapeTH2 = 4.5;
      m_rateAdpMode = 2;

      if (BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.25)
      {
        int firstVarCDFLargerThanVal = 1;
        for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
        {
          if (BinVarSortDsdCDF[b] > 0.7)
          {
            firstVarCDFLargerThanVal = b;
            break;
          }
        }
        if (meanBinVar < 2.52 || BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5)
        {
          *dReshapeTH1 = 2.2;
          *dReshapeTH2 = (BinVarSortDsd[firstVarCDFLargerThanVal] + BinVarSortDsd[firstVarCDFLargerThanVal - 1]) / 2.0;
        }
        else
        {
          m_reshapeCW.BinCW[1] = 30;
          *dReshapeTH2 = 2.8;
        }
      }
      else if (BinVarSortDsdCDF[firstBinVarLessThanVal2] < 0.1 && BinVarSortDsdCDF[firstBinVarLessThanVal1] > 0.02)
      {
        m_reshapeCW.BinCW[0] = 36;
        *dReshapeTH1 = 3.5;
        m_rateAdpMode = 1;
      }
    }
    else if (m_reshapeCW.RspPicSize > 660480)
    {
      *dReshapeTH1 = 2.5;
      *dReshapeTH2 = 4.5;
      m_rateAdpMode = 1;

      if (BinVarSortDsdCDF[firstBinVarLessThanVal4] > 0.6)
      {
        if (maxBinVar < 3.5)
        {
          *dReshapeTH1 = 2.0;
        }
      }
      else
      {
        if (maxBinVar > 3.3)
        {
          m_reshapeCW.BinCW[0] = 35;
        }
        else
        {
          *dReshapeTH1 = 2.8;
          m_reshapeCW.BinCW[0] = 35;
        }
      }
    }
    else if (m_reshapeCW.RspPicSize > 249600)
    {
      m_rateAdpMode = 1;
      m_reshapeCW.BinCW[0] = 36;
      *dReshapeTH1 = 2.5;
      *dReshapeTH2 = 4.5;
    }
    else
    {
      if (BinVarSortDsdCDF[firstBinVarLessThanVal2] < 0.33 && m_reshapeCW.RspFps>40)
      {
        *bIntraAdp = false;
        *bInterAdp = false;
      }
      else
      {
        m_rateAdpMode = 1;
        m_reshapeCW.BinCW[0] = 36;
        *dReshapeTH1 = 3.0;
        *dReshapeTH2 = 4.0;
      }
    }
  }
  else
  {
    if (m_reshapeCW.RspPicSize > 5184000)
    {
      m_reshapeCW.BinCW[0] = 40;
      *dReshapeTH2 = 4.0;
      m_rateAdpMode = 2;

      if (maxBinVar < 2.4)
      {
        *dReshapeTH1 = 3.0;
        if (m_reshapeCW.RspBaseQP <= 22)
          m_tcase = 3;
      }
      else if (maxBinVar > 3.0)
      {
        if (minBinVar > 1)
        {
          m_reshapeCW.BinCW[0] = 36;
          *dReshapeTH1 = 2.8;
          *dReshapeTH2 = 3.5;
          m_sliceReshapeInfo.uiReshapeChromaAdj = 0;
          m_chromaWeight = 1.05;
          m_rateAdpMode = 0;
        }
        else
        {
          m_reshapeCW.BinCW[0] = 36;
          *dReshapeTH1 = 2.2;
          *dReshapeTH2 = 3.5;
          m_sliceReshapeInfo.uiReshapeChromaAdj = 0;
          m_chromaWeight = 0.95;
          if (m_reshapeCW.RspBaseQP <= 27 && m_reshapeCW.RspBaseQP >=25)
            m_tcase = 3;
        }
      }
      else
      {
        *dReshapeTH1 = 1.5;
      }
    }
    else if (m_reshapeCW.RspPicSize > 1497600)
    {
      *dReshapeTH1 = 2.5;
      *dReshapeTH2 = 4.5;
      m_rateAdpMode = 1; 

      if (meanBinVar < 2.52)
      {
        *bIntraAdp = true;
        m_rateAdpMode = 0;
        m_tcase = 9;
      }
      else
      {
        if (BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5)
        {
          *dReshapeTH2 = 3.0;
          *bIntraAdp = true;
        }
        else if (BinVarSortDsdCDF[firstBinVarLessThanVal2] < 0.1 && BinVarSortDsdCDF[firstBinVarLessThanVal1] > 0.02)
        {
          *dReshapeTH1 = 3.0;
          *bIntraAdp = true;
          m_rateAdpMode = 0;
          m_tcase = 9;
        }
        else if (BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.25)
        {
          *dReshapeTH1 = 2.4;
          m_reshapeCW.BinCW[0] = 36;
        }
        else
        {
          *dReshapeTH1 = 2.4;
          m_reshapeCW.BinCW[0] = 36;
        }
      }
    }
    else if (m_reshapeCW.RspPicSize > 660480)
    {
      *bIntraAdp = true;
      m_rateAdpMode = 1;  

      if (BinVarSortDsdCDF[firstBinVarLessThanVal4] > 0.6)
      {
        if (maxBinVar < 3.5)
        {
          *dReshapeTH1 = 2.1;
          *dReshapeTH2 = 3.5;
        }
        else
        {
          *dReshapeTH1 = 2.4;
          *dReshapeTH2 = 4.5;
          m_reshapeCW.BinCW[0] = 40;
          m_rateAdpMode = 0;  
        }
      }
      else
      {
        if (maxBinVar > 3.3)
        {
          *dReshapeTH1 = 3.5;
          *dReshapeTH2 = 3.8;
        }
        else
        {
          *dReshapeTH1 = 3.0;
          *dReshapeTH2 = 4.0;
          m_reshapeCW.BinCW[1] = 30;
        }
      }
    }
    else if (m_reshapeCW.RspPicSize > 249600)
    {
      m_reshapeCW.BinCW[1] = 30;
      *dReshapeTH1 = 2.5;
      *dReshapeTH2 = 4.5;
      *bIntraAdp = true;
      m_rateAdpMode = 1;

      if (minBinVar > 2.6)
      {
        *dReshapeTH1 = 3.2;
        m_rateAdpMode = 0;
        m_tcase = 9;
      }
      else {
        double diff1 = BinVarSortDsdCDF[firstBinVarLessThanVal4] - BinVarSortDsdCDF[firstBinVarLessThanVal3];
        double diff2 = BinVarSortDsdCDF[firstBinVarLessThanVal2] - BinVarSortDsdCDF[firstBinVarLessThanVal1];
        if (diff1 > 0.4 || BinVarSortDsdCDF[firstBinVarLessThanVal1] > 0.1)
        {
          *dReshapeTH1 = 2.9;
          *bIntraAdp = false;
        }
        else
        {
          if (diff2 > 0.1)
          {
            *dReshapeTH1 = 2.5;
          }
          else
          {
            *dReshapeTH1 = 2.9;
            if (BinVarSortDsdCDF[firstBinVarLessThanVal4] > 0.99 && BinVarSortDsdCDF[firstBinVarLessThanVal3] > 0.642 && BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.03)
            {
              m_rateAdpMode = 0;
              m_tcase = 9;
            }
          }
        }
      }
    }
    else
    {
      m_reshapeCW.BinCW[0] = 36;
      m_reshapeCW.BinCW[1] = 30;
      *dReshapeTH1 = 2.6;
      *dReshapeTH2 = 4.5;
      *bIntraAdp = true;
      m_rateAdpMode = 1;
      if (BinVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5 && maxBinVar < 4.7)
      {
        *dReshapeTH1 = 3.4;
      }
    }
  }
}

void EncReshape::deriveReshapeParameters(double *array, int start, int end, ReshapeCW respCW, double &alpha, double &beta)
{
  double minVar = 10.0, maxVar = 0.0;
  for (int b = start; b <= end; b++)
  {
    if (array[b] < minVar)       minVar = array[b];
    if (array[b] > maxVar)       maxVar = array[b];
  }
  double maxCW = (double)respCW.BinCW[0];
  double minCW = (double)respCW.BinCW[1];
  alpha = (minCW - maxCW) / (maxVar - minVar);
  beta = (maxCW*maxVar - minCW*minVar) / (maxVar - minVar);
}

/**
-Init reshaping LUT  from dQP model
*/
void EncReshape::initLUTfromdQPModel()
{
  initModelParam();
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = MAX_LUMA_RESHAPING_LUT_SIZE / PIC_CODE_CW_BINS;
  int p1 = m_DftModel.ScaleFracPrec; //=16, precision of 0.015
  int p2 = m_DftModel.OffsetFracPrec; //=1, precision of 7.5
  int total_shift = p1 + p2;
  int scaleFP = (1 - 2 * m_DftModel.ScaleSign)  * m_DftModel.ScaleAbs;
  int offsetFP = (1 - 2 * m_DftModel.OffsetSign) * m_DftModel.OffsetAbs;
  int maxQP = (1 - 2 * m_DftModel.MaxQPSign)  * m_DftModel.MaxQPAbs;
  int minQP = (1 - 2 * m_DftModel.MinQPSign)  * m_DftModel.MinQPAbs;
  int maxFP = maxQP * (1 << total_shift);
  int minFP = minQP * (1 << total_shift);
  int temp, signval, absval;
  int dQPDIV6_FP;
  int32_t * SlopeLUT = new int32_t[MAX_LUMA_RESHAPING_LUT_SIZE]();
  int32_t * fLUT_HP = new int32_t[MAX_LUMA_RESHAPING_LUT_SIZE]();

  for (int i = 0; i < LUMA_LEVEL_TO_DQP_LUT_MAXSIZE; i++)
  {
    temp = int64_t((scaleFP*i) * (1 << p2)) + int64_t(offsetFP * (1 << p1));
    temp = temp > maxFP ? maxFP : temp < minFP ? minFP : temp;
    signval = temp >= 0 ? 1 : -1;
    absval = signval * temp;
    dQPDIV6_FP = signval * (((absval + 3) / 6 + (1 << (total_shift - 17))) >> (total_shift - 16));
    SlopeLUT[i] = calcEXP2(dQPDIV6_FP);
  }

  if (m_DftModel.FullRangeInputFlag == 0)  
  {
    for (int i = 0; i < 64; i++)                               {      SlopeLUT[i] = 0;    }
    for (int i = 940; i < MAX_LUMA_RESHAPING_LUT_SIZE; i++)    {      SlopeLUT[i] = 0;    }
  }

  for (int i = 0; i < MAX_LUMA_RESHAPING_LUT_SIZE - 1; i++)
    fLUT_HP[i + 1] = fLUT_HP[i] + SlopeLUT[i];
  if (SlopeLUT != nullptr)   {    delete[] SlopeLUT;    SlopeLUT = nullptr;  }

  int max_Y = (fLUT_HP[MAX_LUMA_RESHAPING_LUT_SIZE - 1] + (1 << 7)) >> 8;
  int Roffset = max_Y >> 1;
  for (int i = 0; i < MAX_LUMA_RESHAPING_LUT_SIZE; i++)
  {
    forwardReshapingLUT[i] = (short)(((fLUT_HP[i] >> 8) * (MAX_LUMA_RESHAPING_LUT_SIZE - 1) + Roffset) / max_Y);
  }

  if (fLUT_HP != nullptr)   {    delete[] fLUT_HP;    fLUT_HP = nullptr;  }
  m_sliceReshapeInfo.reshape_model_min_bin_idx = 1;
  m_sliceReshapeInfo.reshape_model_max_bin_idx = 14;

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    int16_t X1 = i * pwlFwdBinLen;
    m_ReshapePivot[i] = forwardReshapingLUT[X1];
  }
  m_ReshapePivot[pwlFwdLUTsize] = 1023;

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_uiBinCWAll[i] = m_ReshapePivot[i + 1] - m_ReshapePivot[i];
  }

  int maxAbsDeltaCW = 0, AbsDeltaCW = 0, DeltaCW = 0;
  for (int i = m_sliceReshapeInfo.reshape_model_min_bin_idx; i <= m_sliceReshapeInfo.reshape_model_max_bin_idx; i++)
  {
    DeltaCW = (int)m_uiBinCWAll[i] - (int)m_uiCWOrg;
    m_sliceReshapeInfo.reshape_model_bin_CW_delta[i] = DeltaCW;
    AbsDeltaCW = (DeltaCW < 0) ? (-DeltaCW) : DeltaCW;
    if (AbsDeltaCW > maxAbsDeltaCW)     {      maxAbsDeltaCW = AbsDeltaCW;    }
  }
  m_sliceReshapeInfo.maxNbitsNeededDeltaCW = g_aucLog2[maxAbsDeltaCW << 1];

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    int16_t Y1 = m_ReshapePivot[i];
    int16_t Y2 = m_ReshapePivot[i + 1];
    forwardReshapingLUT[i*pwlFwdBinLen] = Clip3((Pel)0, (Pel)1023, (Pel)Y1);
    int log2_pwlFwdBinLen = log2_MAX_LUMA_RESHAPING_LUT_SIZE - log2_PIC_CODE_CW_BINS;
    int32_t scale = ((int32_t)(Y2 - Y1) * (1 << FP_PREC) + (1 << (log2_pwlFwdBinLen - 1))) >> (log2_pwlFwdBinLen);
    for (int j = 1; j < pwlFwdBinLen; j++)
    {
      int tempVal = Y1 + (((int32_t)scale * (int32_t)j + (1 << (FP_PREC - 1))) >> FP_PREC);
      forwardReshapingLUT[i*pwlFwdBinLen + j] = Clip3((Pel)0, (Pel)1023, (Pel)tempVal);
    }
  }
  ReverseLUT(forwardReshapingLUT, inverseReshapingLUT, MAX_LUMA_RESHAPING_LUT_SIZE);
  updateChromaDQPLUT();
}


/**
-Perform fixe point exp2 calculation
\param   val  input value
\retval  output value = exp2(val)
*/
int EncReshape::calcEXP2(int val)
{
  int32_t i, f, r, s;
  r = 0x00000e20;

  i = ((int32_t)(val)+0x8000) & ~0xffff;
  f = (int32_t)(val)-i;
  s = ((15 << 16) - i) >> 16;

  r = (r * f + 0x3e1cc333) >> 17;
  r = (r * f + 0x58bd46a6) >> 16;
  r = r * f + 0x7ffde4a3;
  return (uint32_t)r >> s;
}

void EncReshape::constructReshaperSDR()
{
  int used_codewords;
  int tot_cw = MAX_LUMA_RESHAPING_LUT_SIZE;
  int hist_bins = PIC_ANALYZE_CW_BINS;
  int log2_hist_lens = log2_MAX_LUMA_RESHAPING_LUT_SIZE - log2_PIC_ANALYZE_CW_BINS;
  int hist_lens = m_uiCWOrgAnalyze;
  int16_t *Y_LUT_all = new int16_t[MAX_LUMA_RESHAPING_LUT_SIZE + 1]();
  int i, j;
  int cw_scale_bins1, cw_scale_bins2;
  int max_allow_cw = tot_cw;

  cw_scale_bins1 = m_reshapeCW.BinCW[0];
  cw_scale_bins2 = m_reshapeCW.BinCW[1];

  used_codewords = 0;
  for (i = 0; i < hist_bins; i++)
    used_codewords += m_uiBinCWAll[i];

  if (used_codewords > max_allow_cw)
  {
    int cnt0 = 0, cnt1 = 0, cnt2 = 0;
    for (i = 0; i < hist_bins; i++)
    {
      if (m_uiBinCWAll[i] == hist_lens + 1)               cnt0++;
      else if (m_uiBinCWAll[i] == cw_scale_bins1)         cnt1++;
      else if (m_uiBinCWAll[i] == cw_scale_bins2)         cnt2++;
    }

    int delta_cw = used_codewords - max_allow_cw;
    int cw_reduce1 = (cw_scale_bins1 - hist_lens - 1) * cnt1;
    int cw_reduce2 = (hist_lens + 1 - cw_scale_bins2) * cnt0;

    if (delta_cw <= cw_reduce1)
    {
      int idx = 0;
      while (delta_cw > 0)
      {
        if (m_uiBinCWAll[idx] > (hist_lens + 1))
        {
          m_uiBinCWAll[idx]--;
          delta_cw--;
        }
        idx++;
        if (idx == hist_bins)
          idx = 0;
      }
    }
    else if (delta_cw > cw_reduce1 && delta_cw <= (cw_reduce1 + cw_reduce2))
    {
      delta_cw -= cw_reduce1;
      int idx = 0;
      while (delta_cw > 0)
      {
        if (m_uiBinCWAll[idx] > cw_scale_bins2 && m_uiBinCWAll[idx] < cw_scale_bins1)
        {
          m_uiBinCWAll[idx]--;
          delta_cw--;
        }
        idx++;
        if (idx == hist_bins)
          idx = 0;
      }
      for (i = 0; i < hist_bins; i++)
      {
        if (m_uiBinCWAll[i] == cw_scale_bins1)
          m_uiBinCWAll[i] = hist_lens + 1;
      }
    }
    else if (delta_cw > (cw_reduce1 + cw_reduce2))
    {
      delta_cw -= (cw_reduce1 + cw_reduce2);
      int idx = 0;
      while (delta_cw > 0)
      {
        if (m_uiBinCWAll[idx] > 0 && m_uiBinCWAll[idx] < (hist_lens + 1))
        {
          m_uiBinCWAll[idx]--;
          delta_cw--;
        }
        idx++;
        if (idx == hist_bins)
          idx = 0;
      }
      for (i = 0; i < hist_bins; i++)
      {
        if (m_uiBinCWAll[i] == m_uiCWOrgAnalyze + 1)
          m_uiBinCWAll[i] = cw_scale_bins2;
        if (m_uiBinCWAll[i] == cw_scale_bins1)
          m_uiBinCWAll[i] = m_uiCWOrgAnalyze + 1;
      }
    }
  }

  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    m_uiBinCWAll[i] = m_uiBinCWAll[2 * i] + m_uiBinCWAll[2 * i + 1];
  }
  m_sliceReshapeInfo.reshape_model_min_bin_idx = 0;
  m_sliceReshapeInfo.reshape_model_max_bin_idx = PIC_CODE_CW_BINS - 1;
  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    if (m_uiBinCWAll[i] > 0)
    {
      m_sliceReshapeInfo.reshape_model_min_bin_idx = i;
      break;
    }
  }
  for (int i = PIC_CODE_CW_BINS - 1; i >= 0; i--)
  {
    if (m_uiBinCWAll[i] > 0)
    {
      m_sliceReshapeInfo.reshape_model_max_bin_idx = i;
      break;
    }
  }

  int maxAbsDeltaCW = 0, AbsDeltaCW = 0, DeltaCW = 0;
  for (int i = m_sliceReshapeInfo.reshape_model_min_bin_idx; i <= m_sliceReshapeInfo.reshape_model_max_bin_idx; i++)
  {
    DeltaCW = (int)m_uiBinCWAll[i] - (int)m_uiCWOrg;
    m_sliceReshapeInfo.reshape_model_bin_CW_delta[i] = DeltaCW;
    AbsDeltaCW = (DeltaCW < 0) ? (-DeltaCW) : DeltaCW;
    if (AbsDeltaCW > maxAbsDeltaCW)      {      maxAbsDeltaCW = AbsDeltaCW;    }
  }
  m_sliceReshapeInfo.maxNbitsNeededDeltaCW = g_aucLog2[maxAbsDeltaCW << 1];

  hist_bins = PIC_CODE_CW_BINS;
  log2_hist_lens = log2_MAX_LUMA_RESHAPING_LUT_SIZE - log2_PIC_CODE_CW_BINS;
  hist_lens = m_uiCWOrg;

  int sum_bins = 0;
  for (i = 0; i < hist_bins; i++)   {    sum_bins += m_uiBinCWAll[i];  }

  CHECK(sum_bins > max_allow_cw, "SDR CW assignment is wrong!!");

  memset(Y_LUT_all, 0, (MAX_LUMA_RESHAPING_LUT_SIZE + 1) * sizeof(int16_t));
  Y_LUT_all[0] = 0;

  for (i = 0; i < hist_bins; i++)
  {
    Y_LUT_all[(i + 1)*hist_lens] = Y_LUT_all[i*hist_lens] + m_uiBinCWAll[i];
    int16_t Y1 = Y_LUT_all[i*hist_lens];
    int16_t Y2 = Y_LUT_all[(i + 1)*hist_lens];
    m_ReshapePivot[i + 1] = Y2;
    int32_t scale = ((int32_t)(Y2 - Y1) * (1 << FP_PREC) + (1 << (log2_hist_lens - 1))) >> (log2_hist_lens);
    forwardReshapingLUT[i*hist_lens] = Clip3((Pel)0, (Pel)1023, (Pel)Y1);
    for (j = 1; j < hist_lens; j++)
    {
      Y_LUT_all[i*hist_lens + j] = Y1 + (((int32_t)scale * (int32_t)j + (1 << (FP_PREC - 1))) >> FP_PREC);
      forwardReshapingLUT[i*hist_lens + j] = Clip3((Pel)0, (Pel)1023, (Pel)Y_LUT_all[i*hist_lens + j]);
    }
  }

  for (i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    int i_start = i*hist_lens;
    int i_end = (i + 1)*hist_lens - 1;
    m_cwLumaWeight[i] = forwardReshapingLUT[i_end] - forwardReshapingLUT[i_start];
  }

  if (Y_LUT_all != nullptr)   {     delete[] Y_LUT_all;    Y_LUT_all = nullptr;  }

  ReverseLUT(forwardReshapingLUT, inverseReshapingLUT, MAX_LUMA_RESHAPING_LUT_SIZE);  
  updateChromaDQPLUT();
}

#endif
//
//! \}
