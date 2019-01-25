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

/** \file     Reshape.cpp
    \brief    common reshaper class
*/
#include "Reshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#if JVET_M0427_INLOOP_RESHAPER
 //! \ingroup CommonLib
 //! \{

 // ====================================================================================================================
 // Constructor / destructor / create / destroy
 // ====================================================================================================================

Reshape::Reshape()
{
  m_bCTUFlag = false;
  m_bRecReshaped = false;
  m_bReshape = true;
  m_uiCWOrg = MAX_LUMA_RESHAPING_LUT_SIZE / PIC_CODE_CW_BINS;
}

Reshape::~Reshape()
{
}

void  Reshape::create_dec()
{
  if (forwardReshapingLUT.empty())
    forwardReshapingLUT.resize(MAX_LUMA_RESHAPING_LUT_SIZE, 0);
  if (inverseReshapingLUT.empty())
    inverseReshapingLUT.resize(MAX_LUMA_RESHAPING_LUT_SIZE, 0);
  if (m_uiBinCWAll.empty())
    m_uiBinCWAll.resize(PIC_CODE_CW_BINS, 0);
  if (m_ReshapePivot.empty())
    m_ReshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (ChromaAdjHelpLUT.empty())
    ChromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 2048);
}

void  Reshape::destroy()
{
}

/**
-Perform inverse of a one dimension LUT
\param   InputLUT  describing the input LUT
\retval  OutputLUT describing the inversed LUT of InputLUT
\param   lut_size  size of LUT in number of samples
*/
void Reshape::ReverseLUT(std::vector<Pel>& InputLUT, std::vector<Pel>& OutputLUT, uint16_t lut_size)
{
  int i, j;
  OutputLUT[m_ReshapePivot[m_sliceReshapeInfo.reshape_model_min_bin_idx]] = m_sliceReshapeInfo.reshape_model_min_bin_idx*m_uiCWOrg;
  for (i = m_sliceReshapeInfo.reshape_model_min_bin_idx; i <= m_sliceReshapeInfo.reshape_model_max_bin_idx; i++)
  {
    int16_t X1 = m_ReshapePivot[i];
    int16_t X2 = m_ReshapePivot[i + 1];
    OutputLUT[X2] = (i + 1)*m_uiCWOrg;
    int16_t Y1 = OutputLUT[X1];
    int16_t Y2 = OutputLUT[X2];

    if (X2 !=X1)
    {
      int32_t scale = (int32_t)(Y2 - Y1) * (1 << FP_PREC) / (int32_t)(X2 - X1);
      for (j = X1 + 1; j < X2; j++)
      {
        OutputLUT[j] = (Pel)((scale*(int32_t)(j - X1) + (1 << (FP_PREC - 1))) >> FP_PREC) + Y1;
      }
    }
  }

  for (i = 0; i < m_ReshapePivot[m_sliceReshapeInfo.reshape_model_min_bin_idx]; i++)
    OutputLUT[i] = OutputLUT[m_ReshapePivot[m_sliceReshapeInfo.reshape_model_min_bin_idx]];
  for (i = m_ReshapePivot[m_sliceReshapeInfo.reshape_model_max_bin_idx + 1]; i < MAX_LUMA_RESHAPING_LUT_SIZE; i++)
    OutputLUT[i] = OutputLUT[m_ReshapePivot[m_sliceReshapeInfo.reshape_model_max_bin_idx + 1]];

  bool clipRange = ((m_sliceReshapeInfo.reshape_model_min_bin_idx > 0) && (m_sliceReshapeInfo.reshape_model_max_bin_idx < (PIC_CODE_CW_BINS - 1)));
  for (i = 0; i < lut_size; i++)
  {
    if (clipRange) OutputLUT[i] = Clip3((Pel)64, (Pel)940, OutputLUT[i]);
    else           OutputLUT[i] = Clip3((Pel)0, (Pel)1023, OutputLUT[i]);
  }
}


/** compute chroma residuce scale for TU
* \param average luma pred of TU
* \return chroma residue scale
*/
int  Reshape::calculateChromaAdj(Pel avgLuma)
{
  int lumaIdx = Clip3<int>(0, int(LUMA_LEVEL_TO_DQP_LUT_MAXSIZE) - 1, avgLuma);
  int iAdj = ChromaAdjHelpLUT[getPWLIdxInv(lumaIdx)];
  return(iAdj);
}


/** find inx of PWL for inverse mapping
* \param average luma pred of TU
* \return idx of PWL for inverse mapping
*/
int Reshape::getPWLIdxInv(int lumaVal)
{
  int idxS = 0;
  if (lumaVal < m_ReshapePivot[m_sliceReshapeInfo.reshape_model_min_bin_idx + 1])
    return m_sliceReshapeInfo.reshape_model_min_bin_idx;
  else if (lumaVal >= m_ReshapePivot[m_sliceReshapeInfo.reshape_model_max_bin_idx])
    return m_sliceReshapeInfo.reshape_model_max_bin_idx;
  else
  {
    for (idxS = m_sliceReshapeInfo.reshape_model_min_bin_idx; (idxS < m_sliceReshapeInfo.reshape_model_max_bin_idx); idxS++)
    {
      if (lumaVal < m_ReshapePivot[idxS + 1])     break;
    }
    return idxS;
  }
}

/**
-copy Slice reshaper info structure
\param   tInfo describing the target Slice reshaper info structure
\param   sInfo describing the source Slice reshaper info structure
*/
void Reshape::copySliceReshaperInfo(sliceReshapeInfo& tInfo, sliceReshapeInfo& sInfo)
{
  tInfo.slice_reshaper_model_present_flag = sInfo.slice_reshaper_model_present_flag;
  if (sInfo.slice_reshaper_model_present_flag)
  {
    tInfo.reshape_model_max_bin_idx = sInfo.reshape_model_max_bin_idx;
    tInfo.reshape_model_min_bin_idx = sInfo.reshape_model_min_bin_idx;
    memcpy(tInfo.reshape_model_bin_CW_delta, sInfo.reshape_model_bin_CW_delta, sizeof(int)*(PIC_CODE_CW_BINS));
    tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
  }
  tInfo.slice_reshaper_enable_flag = sInfo.slice_reshaper_enable_flag;
  if (sInfo.slice_reshaper_enable_flag)
    tInfo.uiReshapeChromaAdj = sInfo.uiReshapeChromaAdj;
  else
    tInfo.uiReshapeChromaAdj = 0;
}

/** Construct reshaper from syntax
* \param void
* \return void
*/
void Reshape::constructReshaper()
{
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = MAX_LUMA_RESHAPING_LUT_SIZE / PIC_CODE_CW_BINS;

  for (int i = 0; i < m_sliceReshapeInfo.reshape_model_min_bin_idx; i++)
    m_uiBinCWAll[i] = 0;
  for (int i = m_sliceReshapeInfo.reshape_model_max_bin_idx + 1; i < PIC_CODE_CW_BINS; i++)
    m_uiBinCWAll[i] = 0;
  for (int i = m_sliceReshapeInfo.reshape_model_min_bin_idx; i <= m_sliceReshapeInfo.reshape_model_max_bin_idx; i++)
    m_uiBinCWAll[i] = (uint16_t)(m_sliceReshapeInfo.reshape_model_bin_CW_delta[i] + (int)m_uiCWOrg);

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_ReshapePivot[i + 1] = m_ReshapePivot[i] + m_uiBinCWAll[i];
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

/** generate chroma residue scaling LUT
* \param void
* \return void
*/
void Reshape::updateChromaDQPLUT()
{
  const int16_t  CW_bin_SC_LUT[2 * PIC_ANALYZE_CW_BINS] = { 16384, 16384, 16384, 16384, 16384, 16384, 16384, 8192, 8192, 8192, 8192, 5461, 5461, 5461, 5461, 4096, 4096, 4096, 4096, 3277, 3277, 3277, 3277, 2731, 2731, 2731, 2731, 2341, 2341, 2341, 2048, 2048, 2048, 1820, 1820, 1820, 1638, 1638, 1638, 1638, 1489, 1489, 1489, 1489, 1365, 1365, 1365, 1365, 1260, 1260, 1260, 1260, 1170, 1170, 1170, 1170, 1092, 1092, 1092, 1092, 1024, 1024, 1024, 1024 }; //p=11
  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    if ((i < m_sliceReshapeInfo.reshape_model_min_bin_idx) || (i > m_sliceReshapeInfo.reshape_model_max_bin_idx))
      ChromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    else
      ChromaAdjHelpLUT[i] = CW_bin_SC_LUT[Clip3((uint16_t)1, (uint16_t)64, (uint16_t)(m_uiBinCWAll[i] >> 1)) - 1];
  }
}
#endif


//
//! \}
