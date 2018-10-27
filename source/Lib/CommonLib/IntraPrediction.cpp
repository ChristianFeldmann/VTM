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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>

#if JVET_L0628_4TAP_INTRA
#include "CommonLib/InterpolationFilter.h"
#endif //JVET_L0628_4TAP_INTRA

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t IntraPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    20, //   1xn
    20, //   2xn
    20, //   4xn
    14, //   8xn
    2,  //  16xn
    0,  //  32xn
#if HM_MDIS_AS_IN_JEM && !JVET_L0628_4TAP_INTRA
    20, //  64xn
#else
    0,  //  64xn
#endif
    0,  // 128xn
  },
  { // Chroma
    40, //   1xn
    40, //   2xn
    40, //   4xn
    28, //   8xn
    4,  //  16xn
    0,  //  32xn
#if HM_MDIS_AS_IN_JEM && !JVET_L0628_4TAP_INTRA
    40, //  64xn
#else
    0,  //  64xn
#endif
    0,  // 128xn
  }
};

#if JVET_L0628_4TAP_INTRA
const TFilterCoeff g_intraGaussFilter[32][4] = {
  { 16, 32, 16, 0 },
  { 15, 29, 17, 3 },
  { 15, 29, 17, 3 },
  { 14, 29, 18, 3 },
  { 13, 29, 18, 4 },
  { 13, 28, 19, 4 },
  { 13, 28, 19, 4 },
  { 12, 28, 20, 4 },
  { 11, 28, 20, 5 },
  { 11, 27, 21, 5 },
  { 10, 27, 22, 5 },
  { 9, 27, 22, 6 },
  { 9, 26, 23, 6 },
  { 9, 26, 23, 6 },
  { 8, 25, 24, 7 },
  { 8, 25, 24, 7 },
  { 8, 24, 24, 8 },
  { 7, 24, 25, 8 },
  { 7, 24, 25, 8 },
  { 6, 23, 26, 9 },
  { 6, 23, 26, 9 },
  { 6, 22, 27, 9 },
  { 5, 22, 27, 10 },
  { 5, 21, 27, 11 },
  { 5, 20, 28, 11 },
  { 4, 20, 28, 12 },
  { 4, 19, 28, 13 },
  { 4, 19, 28, 13 },
  { 4, 18, 29, 13 },
  { 3, 18, 29, 14 },
  { 3, 17, 29, 15 },
  { 3, 17, 29, 15 } 
};
#endif

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      m_piYuvExt[ch][buf] = nullptr;
    }
  }

  m_piTemp = nullptr;
#if JVET_L0338_MDLM
  m_pMdlmTemp = nullptr;
#endif
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

void IntraPrediction::destroy()
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      delete[] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = nullptr;
    }
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;
#if JVET_L0338_MDLM
  delete[] m_pMdlmTemp;
  m_pMdlmTemp = nullptr;
#endif
}

void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_iYuvExtSize = (MAX_CU_SIZE * 2 + 1) * (MAX_CU_SIZE * 2 + 1);

    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[m_iYuvExtSize];
      }
    }
  }

  int shift = bitDepthY + 4;
  for (int i = 32; i < 64; i++)
  {
    m_auShiftLM[i - 32] = ((1 << shift) + i / 2) / i;
  }
  if (m_piTemp == nullptr)
  {
    m_piTemp = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }
#if JVET_L0338_MDLM
  if (m_pMdlmTemp == nullptr)
  {
    m_pMdlmTemp = new Pel[(2 * MAX_CU_SIZE + 1)*(2 * MAX_CU_SIZE + 1)];//MDLM will use top-above and left-below samples.
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc( const CPelBuf &pSrc, const Size &dstSize )
{
  CHECK( dstSize.width == 0 || dstSize.height == 0, "Empty area provided" );

  int idx, sum = 0;
  Pel dcVal;
  const int width  = dstSize.width;
  const int height = dstSize.height;
  const auto denom     = (width == height) ? (width << 1) : std::max(width,height);
  const auto divShift  = g_aucLog2[denom];
  const auto divOffset = (denom >> 1);

  if ( width >= height )
  {
  for( idx = 0; idx < width; idx++ )
  {
    sum += pSrc.at( 1 + idx, 0 );
  }
  }
  if ( width <= height )
  {   
  for( idx = 0; idx < height; idx++ )
  {
    sum += pSrc.at( 0, 1 + idx );
  }  
  }

  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}

  int IntraPrediction::getWideAngle( int width, int height, int predMode )
  {
    if ( predMode > DC_IDX && predMode <= VDIA_IDX )
    {
      int modeShift = (std::min(2, abs(g_aucLog2[width] - g_aucLog2[height])) << 2) + 2;
      if ( width > height && predMode < 2 + modeShift )
      {
        predMode += (VDIA_IDX - 1);
      }
      else if ( height > width && predMode > VDIA_IDX - modeShift )
      {
        predMode -= (VDIA_IDX - 1);
      }
    }
    return predMode;
  }

  void IntraPrediction::setReferenceArrayLengths( const CompArea &area )
  {
    // set Top and Left reference samples length
    const int  width    = area.width;
    const int  height   = area.height;
    int blockShapeRatio = std::min(2, abs(g_aucLog2[width] - g_aucLog2[height]));

    m_leftRefLength     = (height << 1);
    m_topRefLength      = (width << 1);
    if( width > height )
    {
      m_leftRefLength  += (width >> blockShapeRatio) - height + ((width + 31) >> 5);
    }
    else if( height > width )
    {
      m_topRefLength   += (height >> blockShapeRatio) - width + ((height + 31) >> 5);
    }

  }

void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool useFilteredPredSamples )
{
  const ComponentID    compID       = MAP_CHROMA( compId );
  const ChannelType    channelType  = toChannelType( compID );
  const int            iWidth       = piPred.width;
  const int            iHeight      = piPred.height;
  const uint32_t           uiDirMode    = PU::getFinalIntraMode( pu, channelType );


  CHECK( g_aucLog2[iWidth] < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed" );
  CHECK( g_aucLog2[iWidth] > 7, "Size not allowed" );
  CHECK( iWidth != iHeight && !pu.cs->pcv->rectCUs, "Rectangular block are only allowed with QTBT" );

  const int  srcStride  = m_topRefLength  + 1;
  const int  srcHStride = m_leftRefLength + 1;

  Pel *ptrSrc = getPredictorPtr(compID, useFilteredPredSamples);
  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(compID));

  switch (uiDirMode)
  {
    case(PLANAR_IDX): xPredIntraPlanar(CPelBuf(ptrSrc, srcStride, srcHStride), piPred, *pu.cs->sps); break;
    case(DC_IDX):     xPredIntraDc(CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType, false); break;
#if JVET_L0628_4TAP_INTRA
    case(2): 
    case(DIA_IDX):
    case(VDIA_IDX):
      if (getWideAngle(iWidth, iHeight, uiDirMode) == static_cast<int>(uiDirMode)) // check if uiDirMode is not wide-angle
      {
        xPredIntraAng(CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType, uiDirMode, clpRng, *pu.cs->sps, useFilteredPredSamples); 
        break;
      }
    default:          xPredIntraAng(CPelBuf(getPredictorPtr(compID, false), srcStride, srcHStride), piPred, channelType, uiDirMode, clpRng, *pu.cs->sps, useFilteredPredSamples); break;
#else //JVET_L0628_4TAP_INTRA
    default:          xPredIntraAng(CPelBuf(ptrSrc, srcStride, srcHStride), piPred, channelType, uiDirMode, clpRng, *pu.cs->sps, false); break;
#endif //JVET_L0628_4TAP_INTRA
  }

  bool pdpcCondition = (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX || uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
  if (pdpcCondition)
  {
    const CPelBuf srcBuf = CPelBuf(ptrSrc, srcStride, srcStride);
    PelBuf dstBuf = piPred;
    const int scale = ((g_aucLog2[iWidth] - 2 + g_aucLog2[iHeight] - 2 + 2) >> 2);
    CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

    if (uiDirMode == PLANAR_IDX)
    {
      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top + (64 - wL - wT) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
    else if (uiDirMode == DC_IDX)
    {
      const Pel topLeft = srcBuf.at(0, 0);
      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          int wTL = (wL >> 4) + (wT >> 4);
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top - wTL * topLeft + (64 - wL - wT + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
    else if (uiDirMode == HOR_IDX)
    {
      const Pel topLeft = srcBuf.at(0, 0);
      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wTL = wT;
          dstBuf.at(x, y) = ClipPel((wT * top - wTL * topLeft + (64 - wT + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
    else if (uiDirMode == VER_IDX)
    {
      const Pel topLeft = srcBuf.at(0, 0);
      for (int y = 0; y < iHeight; y++)
      {
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          int wTL = wL;
          dstBuf.at(x, y) = ClipPel((wL * left - wTL * topLeft + (64 - wL + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
  }
}
void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir)
{
  int  iLumaStride = 0;
  PelBuf Temp;
#if JVET_L0338_MDLM
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX))
  {
    iLumaStride = 2 * MAX_CU_SIZE + 1;
    Temp = PelBuf(m_pMdlmTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  else
  {
#endif
  iLumaStride = MAX_CU_SIZE + 1;
  Temp = PelBuf(m_piTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
#if JVET_L0338_MDLM
  }
#endif
  int a, b, iShift;
  xGetLMParameters(pu, compID, chromaArea, a, b, iShift);

  ////// final prediction
  piPred.copyFrom(Temp);
  piPred.linearTransform(a, iShift, b, true, pu.cs->slice->clpRng(compID));
}

void IntraPrediction::xFilterGroup(Pel* pMulDst[], int i, Pel const * const piSrc, int iRecStride, bool bAboveAvaillable, bool bLeftAvaillable)
{
  pMulDst[0][i] = (piSrc[1] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[1][i] = (piSrc[iRecStride] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[3][i] = (piSrc[0] + piSrc[1] + 1) >> 1;

  pMulDst[2][i] = (piSrc[0] + piSrc[1] + piSrc[iRecStride] + piSrc[iRecStride + 1] + 2) >> 2;

}



/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */

//NOTE: Bit-Limit - 24-bit source
void IntraPrediction::xPredIntraPlanar( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps )
{
  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;
  const uint32_t log2W  = g_aucLog2[ width ];
  const uint32_t log2H  = g_aucLog2[ height ];

  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const uint32_t offset = width * height;

  // Get left and above reference column and row
  for( int k = 0; k < width + 1; k++ )
  {
    topRow[k] = pSrc.at( k + 1, 0 );
  }

  for( int k = 0; k < height + 1; k++ )
  {
    leftColumn[k] = pSrc.at( 0, k + 1 );
  }

  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight = topRow[width];

  for( int k = 0; k < width; k++ )
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] << log2H;
  }

  for( int k = 0; k < height; k++ )
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] << log2W;
  }

  const uint32_t finalShift = 1 + log2W + log2H;
  const uint32_t stride     = pDst.stride;
  Pel*       pred       = pDst.buf;
  for( int y = 0; y < height; y++, pred += stride )
  {
    int horPred = leftColumn[y];

    for( int x = 0; x < width; x++ )
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      int vertPred = topRow[x];
      pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
    }
  }
}




void IntraPrediction::xPredIntraDc( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter )
{
  const Pel dcval = xGetPredValDc( pSrc, pDst );
  pDst.fill( dcval );

#if HEVC_USE_DC_PREDFILTERING
  if( enableBoundaryFilter )
  {
    xDCPredFiltering( pSrc, pDst, channelType );
  }
#endif
}

#if HEVC_USE_DC_PREDFILTERING
/** Function for filtering intra DC predictor. This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
void IntraPrediction::xDCPredFiltering(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType &channelType)
{
  uint32_t iWidth = pDst.width;
  uint32_t iHeight = pDst.height;
  int x, y;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst.at(0, 0) = (Pel)((pSrc.at(1, 0) + pSrc.at(0, 1) + 2 * pDst.at(0, 0) + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst.at(x, 0) = (Pel)((pSrc.at(x + 1, 0)  +  3 * pDst.at(x, 0) + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1; y < iHeight; y++ )
    {
      pDst.at(0, y) = (Pel)((pSrc.at(0, y + 1) + 3 * pDst.at(0, y) + 2) >> 2);
    }
  }

  return;
}
#endif

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
*
* This function derives the prediction samples for the angular mode based on the prediction direction indicated by
* the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
* the reference row above the block in the case of vertical prediction or displacement of the rightmost column
* of the block and reference column left from the block in the case of the horizontal prediction. The displacement
* is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
* the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
* from the extended main reference.
*/
//NOTE: Bit-Limit - 25-bit source
#if HEVC_USE_HOR_VER_PREDFILTERING
void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const uint32_t dirMode, const ClpRng& clpRng, const bool bEnableEdgeFilters, const SPS& sps, const bool enableBoundaryFilter )
#elif JVET_L0628_4TAP_INTRA
void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const uint32_t dirMode, const ClpRng& clpRng, const SPS& sps, const bool useFilteredPredSamples )
#else
void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const uint32_t dirMode, const ClpRng& clpRng, const SPS& sps, const bool enableBoundaryFilter )
#endif
{
  int width =int(pDst.width);
  int height=int(pDst.height);

  CHECK( !( dirMode > DC_IDX && dirMode < NUM_LUMA_MODE ), "Invalid intra dir" );

  int              predMode           = getWideAngle(width, height, dirMode);
  const bool       bIsModeVer         = predMode >= DIA_IDX;
  const int        intraPredAngleMode = (bIsModeVer) ? predMode - VER_IDX : -(predMode - HOR_IDX);
  const int        absAngMode         = abs(intraPredAngleMode);
  const int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
#if HEVC_USE_HOR_VER_PREDFILTERING
  const bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);
#endif

  // Set bitshifts and scale the angle parameter to block size

  static const int angTable[27]    = { 0,    1,    2,    3,    5,    7,    9,   11,   13,   15,   17,   19,   21,   23,   26,   29,   32,   35,  39,  45,  49,  54,  60,  68,  79,  93, 114 };
  static const int invAngTable[27] = { 0, 8192, 4096, 2731, 1638, 1170,  910,  745,  630,  546,  482,  431,  390,  356,  315,  282,  256,  234, 210, 182, 167, 152, 137, 120, 104,  88,  72 }; // (256 * 32) / Angle

  int invAngle                    = invAngTable[absAngMode];
  int absAng                      = angTable   [absAngMode];
  int intraPredAngle              = signAng * absAng;

  Pel* refMain;
  Pel* refSide;

  Pel  refAbove[2 * MAX_CU_SIZE + 3];
  Pel  refLeft [2 * MAX_CU_SIZE + 3];


  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
#if JVET_L0628_4TAP_INTRA
    auto width    = int(pDst.width) +1;
    auto height   = int(pDst.height)+1;  
    auto lastIdx  = bIsModeVer ? width : height;
    auto firstIdx = ( ((bIsModeVer ? height : width) -1) * intraPredAngle ) >> 5;
#endif //JVET_L0628_4TAP_INTRA
    for( int x = 0; x < width + 1; x++ )
    {
      refAbove[x + height - 1] = pSrc.at( x, 0 );
    }
    for( int y = 0; y < height + 1; y++ )
    {
      refLeft[y + width - 1] = pSrc.at( 0, y );
    }
    refMain = (bIsModeVer ? refAbove + height : refLeft  + width ) - 1;
    refSide = (bIsModeVer ? refLeft  + width  : refAbove + height) - 1;

    // Extend the Main reference to the left.
    int invAngleSum    = 128;       // rounding for (shift by 8)
#if JVET_L0628_4TAP_INTRA
    for( int k = -1; k > firstIdx; k-- )
#else //JVET_L0628_4TAP_INTRA
    const int refMainOffsetPreScale = bIsModeVer ? height : width;
    for( int k = -1; k > (refMainOffsetPreScale * intraPredAngle) >> 5; k-- )
#endif //JVET_L0628_4TAP_INTRA
    {
      invAngleSum += invAngle;
      refMain[k] = refSide[invAngleSum>>8];
    }
#if JVET_L0628_4TAP_INTRA
    refMain[lastIdx] = refMain[lastIdx-1];
    refMain[firstIdx] = refMain[firstIdx+1];
#endif //JVET_L0628_4TAP_INTRA
  }
  else
  {
    for( int x = 0; x < m_topRefLength + 1; x++ )
    {
#if JVET_L0628_4TAP_INTRA
      refAbove[x+1] = pSrc.at(x, 0);
#else //JVET_L0628_4TAP_INTRA
      refAbove[x] = pSrc.at(x, 0);
#endif //JVET_L0628_4TAP_INTRA
    }
    for( int y = 0; y < m_leftRefLength + 1; y++ )
    {
#if JVET_L0628_4TAP_INTRA
      refLeft[y+1]  = pSrc.at(0, y);
#else //JVET_L0628_4TAP_INTRA
      refLeft[y]  = pSrc.at(0, y);
#endif //JVET_L0628_4TAP_INTRA
    }
    refMain = bIsModeVer ? refAbove : refLeft ;
    refSide = bIsModeVer ? refLeft  : refAbove;

#if JVET_L0628_4TAP_INTRA
    refMain++;
    refSide++;
    refMain[-1] = refMain[0];
    auto lastIdx = 1 + ((bIsModeVer) ? m_topRefLength : m_leftRefLength);
    refMain[lastIdx] = refMain[lastIdx-1];
#endif //JVET_L0628_4TAP_INTRA
  }

  // swap width/height if we are doing a horizontal mode:
  Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
  const int dstStride = bIsModeVer ? pDst.stride : MAX_CU_SIZE;
  Pel *pDstBuf = bIsModeVer ? pDst.buf : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }


  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        pDstBuf[y*dstStride + x] = refMain[x + 1];
      }
    }
#if HEVC_USE_HOR_VER_PREDFILTERING
    if (edgeFilter)
    {
      for( int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ( ( refSide[y + 1] - refSide[0] ) >> 1 ), clpRng );
      }
    }
#endif
  }
  else
  {
    Pel *pDsty=pDstBuf;

    for (int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & (32 - 1);

#if JVET_L0628_4TAP_INTRA
      if( (absAng & (32 - 1)) != 0 ) // use 4-tap interpolation only for intra prediction modes with fractional displacements
#elif HM_4TAPIF_AS_IN_JEM
      if( deltaFract )
#else //JVET_L0628_4TAP_INTRA
      if( absAng < 32 )
#endif
      {
#if JVET_L0628_4TAP_INTRA
        if( isLuma(channelType) )
        {
          Pel                        p[4];
          const bool                 useCubicFilter = !useFilteredPredSamples;
          TFilterCoeff const * const f              = (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : g_intraGaussFilter[deltaFract];

          int         refMainIndex   = deltaInt + 1;

          for( int x = 0; x < width; x++, refMainIndex++ )
          {
            p[0] = refMain[refMainIndex - 1];
            p[1] = refMain[refMainIndex];
            p[2] = refMain[refMainIndex + 1];
            p[3] = refMain[refMainIndex + 2];

            pDstBuf[y*dstStride + x] = static_cast<Pel>((static_cast<int>(f[0] * p[0]) + static_cast<int>(f[1] * p[1]) + static_cast<int>(f[2] * p[2]) + static_cast<int>(f[3] * p[3]) + 32) >> 6);

            if( useCubicFilter ) // only cubic filter has negative coefficients and requires clipping
            {
              pDstBuf[y*dstStride + x] = ClipPel( pDstBuf[y*dstStride + x], clpRng );
            }
          }
        }
        else
#endif //JVET_L0628_4TAP_INTRA
        {
          // Do linear filtering
          const Pel *pRM = refMain + deltaInt + 1;
          int lastRefMainPel = *pRM++;
          for( int x = 0; x < width; pRM++, x++ )
          {
            int thisRefMainPel = *pRM;
            pDsty[x + 0] = ( Pel ) ( ( ( 32 - deltaFract )*lastRefMainPel + deltaFract*thisRefMainPel + 16 ) >> 5 );
            lastRefMainPel = thisRefMainPel;
          }
        }
      }
      else
      {
        // Just copy the integer samples
        for( int x = 0; x < width; x++ )
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
      const int numModes = 8;
      const int scale = ((g_aucLog2[width] - 2 + g_aucLog2[height] - 2 + 2) >> 2);
      CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

      if (predMode == 2 || predMode == VDIA_IDX)
      {
        int wT = 16 >> std::min(31, ((y << 1) >> scale));

        for (int x = 0; x < width; x++)
        {
          int wL = 16 >> std::min(31, ((x << 1) >> scale));
          if (wT + wL == 0) break;

          int c = x + y + 1;
          const Pel left = (wL != 0) ? refSide[c + 1] : 0;
          const Pel top  = (wT != 0) ? refMain[c + 1] : 0;

          pDsty[x] = ClipPel((wL * left + wT * top + (64 - wL - wT) * pDsty[x] + 32) >> 6, clpRng);
        }
      }
      else if ((predMode >= VDIA_IDX - numModes && predMode != VDIA_IDX) || (predMode != 2 && predMode <= (2 + numModes)))
      {
        int invAngleSum0 = 2;
        for (int x = 0; x < width; x++)
        {
          invAngleSum0 += invAngle;
          int deltaPos0 = invAngleSum0 >> 2;
          int deltaFrac0 = deltaPos0 & 63;
          int deltaInt0 = deltaPos0 >> 6;

          int deltay = y + deltaInt0 + 1;
          if (deltay >(bIsModeVer ? m_leftRefLength : m_topRefLength) - 1) break;

          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          if (wL == 0) break;
          Pel *p = refSide + deltay;

          Pel left = (((64 - deltaFrac0) * p[0] + deltaFrac0 * p[1] + 32) >> 6);
          pDsty[x] = ClipPel((wL * left + (64 - wL) * pDsty[x] + 32) >> 6, clpRng);
        }
      }
    }
#if HEVC_USE_HOR_VER_PREDFILTERING
    if( edgeFilter && absAng <= 1 )
    {
      for( int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ((refSide[y + 1] - refSide[0]) >> 2), clpRng );
      }
    }
#endif
  }

  // Flip the block if this is the horizontal mode
  if( !bIsModeVer )
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        pDst.at( y, x ) = pDstBuf[x];
      }
      pDstBuf += dstStride;
    }
  }
}


bool IntraPrediction::useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const uint32_t &uiDirMode)
{
  return CU::isRDPCMEnabled(*pu.cu) && pu.cu->transQuantBypass && (uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
}

inline bool isAboveLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT );
inline int  isAboveAvailable      ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags );
inline int  isLeftAvailable       ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags );
inline int  isAboveRightAvailable ( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );
inline int  isBelowLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );

void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool bFilterRefSamples)
{
  const CodingStructure& cs   = *cu.cs;

  Pel *refBufUnfiltered   = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered     = m_piYuvExt[area.compID][PRED_BUF_FILTERED];

  setReferenceArrayLengths(area);

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
  // ----- Step 2: filtered reference samples -----
  if( bFilterRefSamples )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps );
  }
}

void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu )
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  const int  predSize           = m_topRefLength;
  const int  predHSize          = m_leftRefLength;
  const int  predStride         = predSize + 1;

  const bool noShift            = pcv.noChroma2x2 && area.width == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth          = pcv.minCUWidth  >> (noShift ? 0 : getComponentScaleX( area.compID, sps.getChromaFormatIdc() ));
  const int  unitHeight         = pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY( area.compID, sps.getChromaFormatIdc() ));

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area;
  const Position posRT          = area.topRight();
  const Position posLB          = area.bottomLeft();

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int   numIntraNeighbor = 0;

  memset( neighborFlags, 0, totalUnits );

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable( cu, chType, posLT );
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT, numAboveUnits,      unitWidth,  (neighborFlags + totalLeftUnits + 1) );
  numIntraNeighbor += isAboveRightAvailable( cu, chType, posRT, numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + numAboveUnits) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT, numLeftUnits,       unitHeight, (neighborFlags + totalLeftUnits - 1) );
  numIntraNeighbor += isBelowLeftAvailable ( cu, chType, posLB, numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );

  // ----- Step 2: fill reference samples (depending on neighborhood) -----
  CHECK((predHSize + 1) * predStride > m_iYuvExtSize, "Reference sample area not supported");

  const Pel*  srcBuf    = recoBuf.buf;
  const int   srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);


  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j]            = valueDC; }
    for( int i = 1; i <= predHSize; i++ ) { ptrDst[i*predStride] = valueDC; }
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - srcStride - 1;
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j] = ptrSrc[j]; }
    // Fill left and below left border with rec. samples
    ptrSrc = srcBuf - 1;
    for( int i = 1; i <= predHSize; i++ ) { ptrDst[i*predStride] = *(ptrSrc); ptrSrc += srcStride; }
  }
  else // reference samples are partially available
  {
    // BB: old implementation using tmpLineBuf
    // ---------------------------------------
    Pel  tmpLineBuf[5 * MAX_CU_SIZE];
    Pel* ptrTmp;
    int  unitIdx;

    // Initialize
    const int totalSamples = (totalLeftUnits * unitHeight) + ((totalAboveUnits + 1) * unitWidth); // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    for( int k = 0; k < totalSamples; k++ ) { tmpLineBuf[k] = valueDC; }

    // Fill top-left sample
    ptrSrc = srcBuf - srcStride - 1;
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight);
    unitIdx = totalLeftUnits;
    if( neighborFlags[unitIdx] )
    {
      Pel topLeftVal = ptrSrc[0];
      for( int j = 0; j < unitWidth; j++ ) { ptrTmp[j] = topLeftVal; }
    }

    // Fill left & below-left samples (downwards)
    ptrSrc += srcStride;
    ptrTmp--;
    unitIdx--;

    for( int k = 0; k < totalLeftUnits; k++ )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int i = 0; i < unitHeight; i++ ) { ptrTmp[-i] = ptrSrc[i*srcStride]; }
      }
      ptrSrc += unitHeight*srcStride;
      ptrTmp -= unitHeight;
      unitIdx--;
    }

    // Fill above & above-right samples (left-to-right) (each unit has "unitWidth" samples)
    ptrSrc = srcBuf - srcStride;
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight) + unitWidth; // offset line buffer by totalLeftUnits*unitHeight (for left/below-left) + unitWidth (for above-left)
    unitIdx = totalLeftUnits + 1;
    for( int k = 0; k < totalAboveUnits; k++ )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int j = 0; j < unitWidth; j++ ) { ptrTmp[j] = ptrSrc[j]; }
      }
      ptrSrc += unitWidth;
      ptrTmp += unitWidth;
      unitIdx++;
    }

    // Pad reference samples when necessary
    int  currUnit       = 0;
    Pel* ptrTmpCurrUnit = tmpLineBuf;

    if( !neighborFlags[0] )
    {
      int nextUnit = 1;
      while( nextUnit < totalUnits && !neighborFlags[nextUnit] )
      {
        nextUnit++;
      }
      Pel* ptrTmpRef = tmpLineBuf + ((nextUnit < totalLeftUnits) ? (nextUnit * unitHeight) : ((totalLeftUnits * (unitHeight - unitWidth)) + (nextUnit * unitWidth)));
      const Pel refSample = *ptrTmpRef;
      // Pad unavailable samples with new value
      // fill left column
      while( currUnit < std::min<int>( nextUnit, totalLeftUnits ) )
      {
        for( int i = 0; i < unitHeight; i++ ) { ptrTmpCurrUnit[i] = refSample; }
        ptrTmpCurrUnit += unitHeight;
        currUnit++;
      }
      // fill top row
      while( currUnit < nextUnit )
      {
        for( int j = 0; j < unitWidth; j++ ) { ptrTmpCurrUnit[j] = refSample; }
        ptrTmpCurrUnit += unitWidth;
        currUnit++;
      }
    }

    // pad all other reference samples.
    while( currUnit < totalUnits )
    {
      const int numSamplesInCurrUnit = (currUnit >= totalLeftUnits) ? unitWidth : unitHeight;
      if( !neighborFlags[currUnit] ) // samples not available
      {
        const Pel refSample = *(ptrTmpCurrUnit - 1);
        for( int k = 0; k < numSamplesInCurrUnit; k++ ) { ptrTmpCurrUnit[k] = refSample; }

      }
      ptrTmpCurrUnit += numSamplesInCurrUnit;
      currUnit++;
    }

    // Copy processed samples
    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight) + (unitWidth - 1);
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j] = ptrTmp[j]; } // top left, top and top right samples

    ptrTmp = tmpLineBuf + (totalLeftUnits * unitHeight);
    for( int i = 1; i <= predHSize; i++ ) { ptrDst[i*predStride] = ptrTmp[-i]; }
  }
}

void IntraPrediction::xFilterReferenceSamples( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps )
{
  const int  predSize   = m_topRefLength;
  const int  predHSize  = m_leftRefLength;
  const int  predStride = predSize + 1;


#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  // Strong intra smoothing
  ChannelType chType = toChannelType( area.compID );
  if( sps.getUseStrongIntraSmoothing() && isLuma( chType ) )
  {
    const Pel bottomLeft = refBufUnfiltered[predStride * predHSize];
    const Pel topLeft    = refBufUnfiltered[0];
    const Pel topRight   = refBufUnfiltered[predSize];

    const int  threshold     = 1 << (sps.getBitDepth( chType ) - 5);
    const bool bilinearLeft  = abs( (bottomLeft + topLeft)  - (2 * refBufUnfiltered[predStride * tuHeight]) ) < threshold; //difference between the
    const bool bilinearAbove = abs( (topLeft    + topRight) - (2 * refBufUnfiltered[             tuWidth ]) ) < threshold; //ends and the middle

    if( tuWidth >= 32 && tuHeight >= 32 && bilinearLeft && bilinearAbove )
#if !HEVC_USE_INTRA_SMOOTHING_T32
    if( tuWidth > 32 && tuHeight > 32 )
#endif
#if !HEVC_USE_INTRA_SMOOTHING_T64
    if( tuWidth < 64 && tuHeight < 64 )
#endif
    {
      Pel *piDestPtr = refBufFiltered + (predStride * predHSize); // bottom left

      // apply strong intra smoothing
      for (int i = 0; i < predHSize; i++, piDestPtr -= predStride) //left column (bottom to top)
      {
        *piDestPtr = (((predHSize - i) * bottomLeft) + (i * topLeft) + predHSize / 2) / predHSize;
      }
      for( uint32_t i = 0; i <= predSize; i++, piDestPtr++ )            //full top row (left-to-right)
      {
        *piDestPtr = (((predSize - i) * topLeft) + (i * topRight) + predSize / 2) / predSize;
      }

      return;
    }
  }
#endif

  // Regular reference sample filter
  const Pel *piSrcPtr  = refBufUnfiltered + (predStride * predHSize); // bottom left
        Pel *piDestPtr = refBufFiltered   + (predStride * predHSize); // bottom left

  // bottom left (not filtered)
  *piDestPtr = *piSrcPtr;
  piDestPtr -= predStride;
  piSrcPtr  -= predStride;
  //left column (bottom to top)
  for( int i = 1; i < predHSize; i++, piDestPtr -= predStride, piSrcPtr -= predStride)
  {
    *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[-predStride] + 2) >> 2;
  }
  //top-left
  *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[1] + 2) >> 2;
  piDestPtr++;
  piSrcPtr++;
  //top row (left-to-right)
  for( uint32_t i=1; i < predSize; i++, piDestPtr++, piSrcPtr++ )
  {
    *piDestPtr = (piSrcPtr[1] + 2 * piSrcPtr[0] + piSrcPtr[-1] + 2) >> 2;
  }
  // top right (not filtered)
  *piDestPtr=*piSrcPtr;
}

bool IntraPrediction::useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, bool modeSpecific, const UnitArea &tuArea )
{
  const SPS         &sps    = *pu.cs->sps;
  const ChannelType  chType = toChannelType( compID );

  // high level conditions
  if( sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag() )                                       { return false; }
  if( !isLuma( chType ) && pu.chromaFormat != CHROMA_444 )                                               { return false; }


  if( !modeSpecific )                                                                                    { return true; }

  // pred. mode related conditions
  const int dirMode = PU::getFinalIntraMode( pu, chType );
  int predMode = getWideAngle(tuArea.blocks[compID].width, tuArea.blocks[compID].height, dirMode);
  if (predMode != dirMode && (predMode < 2 || predMode > VDIA_IDX))                                      { return true; }
  if (dirMode == DC_IDX)                                                                                 { return false; }
  if (dirMode == PLANAR_IDX)
  {
    return tuArea.blocks[compID].width * tuArea.blocks[compID].height > 32 ? true : false;
  }

  int diff = std::min<int>( abs( dirMode - HOR_IDX ), abs( dirMode - VER_IDX ) );
  int log2Size = ((g_aucLog2[tuArea.blocks[compID].width] + g_aucLog2[tuArea.blocks[compID].height]) >> 1);
  CHECK( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
  return (diff > m_aucIntraFilter[chType][log2Size]);
}


bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure& cs = *cu.cs;
  const Position refPos = posLT.offset(-1, -1);
  const CodingUnit* pcCUAboveLeft = cs.isDecomp( refPos, chType ) ? cs.getCURestricted( refPos, cu, chType ) : nullptr;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool bAboveLeftFlag;

  if (isConstrained)
  {
    bAboveLeftFlag = pcCUAboveLeft && CU::isIntra(*pcCUAboveLeft);
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }

  return bAboveLeftFlag;
}

int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;
  int maxDx = uiNumUnitsInPU * unitWidth;

  for (uint32_t dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }
  return iNumIntra;
}

int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;
  int maxDy = uiNumUnitsInPU * unitHeight;

  for (uint32_t dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;

  uint32_t maxDx = uiNumUnitsInPU * unitWidth;

  for (uint32_t dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }

  return iNumIntra;
}

int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;
  int maxDy = uiNumUnitsInPU * unitHeight;

  for (uint32_t dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-1, unitHeight + dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if ( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}
// LumaRecPixels
void IntraPrediction::xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea)
{
  int iDstStride = 0;
  Pel* pDst0 = 0;
#if JVET_L0338_MDLM
  int curChromaMode = pu.intraDir[1];
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    iDstStride = 2 * MAX_CU_SIZE + 1;
    pDst0 = m_pMdlmTemp + iDstStride + 1;
  }
  else
  {
#endif
  iDstStride = MAX_CU_SIZE + 1;
  pDst0 = m_piTemp + iDstStride + 1; //MMLM_SAMPLE_NEIGHBOR_LINES;
#if JVET_L0338_MDLM
  }
#endif
  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, pu.chromaFormat, chromaArea.lumaPos(), recalcSize( pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)


  CHECK( lumaArea.width  == chromaArea.width, "" );
  CHECK( lumaArea.height == chromaArea.height, "" );

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CPelBuf Src = pu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  int iRecStride        = Src.stride;
  int iRecStride2       = iRecStride << 1;

  CodingStructure&      cs = *pu.cs;
  const CodingUnit& lumaCU = isChroma( pu.chType ) ? *pu.cs->picture->cs->getCU( lumaArea.pos(), CH_L ) : *pu.cu;
  const CodingUnit&     cu = *pu.cu;

  const CompArea& area = isChroma( pu.chType ) ? chromaArea : lumaArea;

  const SPS &sps = *cs.sps;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  if( !cs.pcv->rectCUs )
  {
    iBaseUnitSize = sps.getMaxCUWidth() >> sps.getMaxCodingDepth();
  }

  const int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const int  iUnitHeight      = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const int  iAboveUnits      = iTUWidthInUnits;
  const int  iLeftUnits       = iTUHeightInUnits;
#if JVET_L0338_MDLM
  const int  chromaUnitWidth = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, area.chromaFormat);
  const int  chromaUnitHeight = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, area.chromaFormat);
  const int  topTemplateSampNum = 2 * uiCWidth; // for MDLM, the number of template samples is 2W or 2H.
  const int  leftTemplateSampNum = 2 * uiCHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  const int  totalAboveUnits = (topTemplateSampNum + (chromaUnitWidth - 1)) / chromaUnitWidth;
  const int  totalLeftUnits = (leftTemplateSampNum + (chromaUnitHeight - 1)) / chromaUnitHeight;
  const int  totalUnits = totalLeftUnits + totalAboveUnits + 1;
  const int  aboveRightUnits = totalAboveUnits - iAboveUnits;
  const int  leftBelowUnits = totalLeftUnits - iLeftUnits;

  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
#endif
  bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
#if JVET_L0338_MDLM
  memset(bNeighborFlags, 0, totalUnits);
#else
  memset( bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits );
#endif
  bool bAboveAvaillable, bLeftAvaillable;

  int availlableUnit = isLeftAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iLeftUnits, iUnitHeight,
#if JVET_L0338_MDLM
  ( bNeighborFlags + iLeftUnits + leftBelowUnits - 1 ) );
#else
  ( bNeighborFlags + iLeftUnits - 1 ) );
#endif

  if( lumaCU.cs->pcv->rectCUs )
  {
    bLeftAvaillable = availlableUnit == iTUHeightInUnits;
  }
  else
  {
    bLeftAvaillable = availlableUnit == iTUWidthInUnits;
  }

  availlableUnit = isAboveAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iAboveUnits, iUnitWidth,
#if JVET_L0338_MDLM
  ( bNeighborFlags + iLeftUnits + leftBelowUnits + 1 ) );
#else
  ( bNeighborFlags + iLeftUnits + 1 ) );
#endif

  if( lumaCU.cs->pcv->rectCUs )
  {
    bAboveAvaillable = availlableUnit == iTUWidthInUnits;
  }
  else
  {
    bAboveAvaillable = availlableUnit == iTUHeightInUnits;
  }
#if JVET_L0338_MDLM
  if (bLeftAvaillable) // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.bottomLeftComp(area.compID), leftBelowUnits, iUnitHeight, (bNeighborFlags + leftBelowUnits - 1));
  }

  if (bAboveAvaillable) // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.topRightComp(area.compID), aboveRightUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1));
  }
#endif

  Pel*       pDst  = nullptr;
  Pel const* piSrc = nullptr;

  if( bAboveAvaillable )
  {
    pDst  = pDst0    - iDstStride;
    piSrc = pRecSrc0 - iRecStride2;
#if JVET_L0338_MDLM
    int addedAboveRight = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedAboveRight = avaiAboveRightUnits*chromaUnitWidth;
    }
    for (int i = 0; i < uiCWidth + addedAboveRight; i++)
#else
    for( int i = 0; i < uiCWidth; i++ )
#endif
    {
      if( i == 0 && !bLeftAvaillable )
      {
        pDst[i] = ( piSrc[2 * i] + piSrc[2 * i + iRecStride] + 1 ) >> 1;
      }
      else
      {
        pDst[i] = ( ( ( piSrc[2 * i             ] * 2 ) + piSrc[2 * i - 1             ] + piSrc[2 * i + 1             ] )
                  + ( ( piSrc[2 * i + iRecStride] * 2 ) + piSrc[2 * i - 1 + iRecStride] + piSrc[2 * i + 1 + iRecStride] )
                  + 4 ) >> 3;
      }
    }
  }

  if( bLeftAvaillable )
  {
    pDst  = pDst0    - 1;
    piSrc = pRecSrc0 - 3;
#if JVET_L0338_MDLM
    int addedLeftBelow = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedLeftBelow = avaiLeftBelowUnits*chromaUnitHeight;
    }
    for (int j = 0; j < uiCHeight + addedLeftBelow; j++)
#else
    for( int j = 0; j < uiCHeight; j++ )
#endif
    {
      pDst[0] = ( ( piSrc[1             ] * 2 + piSrc[0         ] + piSrc[2             ] )
                + ( piSrc[1 + iRecStride] * 2 + piSrc[iRecStride] + piSrc[2 + iRecStride] )
                + 4 ) >> 3;

      piSrc += iRecStride2;
      pDst  += iDstStride;
    }
  }


  // inner part from reconstructed picture buffer
  for( int j = 0; j < uiCHeight; j++ )
  {
    for( int i = 0; i < uiCWidth; i++ )
    {
      if( i == 0 && !bLeftAvaillable )
      {
        pDst0[i] = ( pRecSrc0[2 * i] + pRecSrc0[2 * i + iRecStride] + 1 ) >> 1;
      }
      else
      {
        pDst0[i] = ( pRecSrc0[2 * i             ] * 2 + pRecSrc0[2 * i + 1             ] + pRecSrc0[2 * i - 1             ]
                   + pRecSrc0[2 * i + iRecStride] * 2 + pRecSrc0[2 * i + 1 + iRecStride] + pRecSrc0[2 * i - 1 + iRecStride]
                   + 4 ) >> 3;
      }
    }

    pDst0    += iDstStride;
    pRecSrc0 += iRecStride2;
  }
}
#if JVET_L0338_MDLM && !JVET_L0191_LM_WO_LMS
void IntraPrediction::xPadMdlmTemplateSample(Pel*pSrc, Pel*pCur, int cWidth, int cHeight, int existSampNum, int targetSampNum)
{
  int sampNumToBeAdd = targetSampNum - existSampNum;
  Pel*pTempSrc = pSrc + existSampNum;
  Pel*pTempCur = pCur + existSampNum;
  for (int i = 0; i < sampNumToBeAdd; i++)
  {
    pTempSrc[i] = pSrc[existSampNum - 1];
    pTempCur[i] = pCur[existSampNum - 1];
  }
}
#endif
#if JVET_L0191_LM_WO_LMS
void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID,
                                              const CompArea &chromaArea,
                                              int &a, int &b, int &iShift)
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure & cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const SPS &        sps           = *cs.sps;
  const uint32_t     tuWidth     = chromaArea.width;
  const uint32_t     tuHeight    = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int baseUnitSize = 1 << MIN_CU_LOG2;
  const int unitWidth    = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const int unitHeight   = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);

  const int tuWidthInUnits  = tuWidth / unitWidth;
  const int tuHeightInUnits = tuHeight / unitHeight;
  const int aboveUnits      = tuWidthInUnits;
  const int leftUnits       = tuHeightInUnits;
#if JVET_L0338_MDLM
  int topTemplateSampNum = 2 * cWidth; // for MDLM, the template sample number is 2W or 2H;
  int leftTemplateSampNum = 2 * cHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  int totalAboveUnits = (topTemplateSampNum + (unitWidth - 1)) / unitWidth;
  int totalLeftUnits = (leftTemplateSampNum + (unitHeight - 1)) / unitHeight;
  int totalUnits = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits = totalAboveUnits - aboveUnits;
  int leftBelowUnits = totalLeftUnits - leftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  int avaiAboveUnits = 0;
  int avaiLeftUnits = 0;

  int curChromaMode = pu.intraDir[1];
#endif
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
#if JVET_L0338_MDLM
  memset(neighborFlags, 0, totalUnits);
#else
  memset(neighborFlags, 0, 1 + leftUnits + aboveUnits);
#endif

  bool aboveAvailable, leftAvailable;

  int availableUnit =
    isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, aboveUnits, unitWidth, 
#if JVET_L0338_MDLM
    (neighborFlags + leftUnits + leftBelowUnits + 1));
#else
    (neighborFlags + leftUnits + 1));
#endif
  aboveAvailable = availableUnit == tuWidthInUnits;

  availableUnit =
    isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, leftUnits, unitHeight, 
#if JVET_L0338_MDLM
    (neighborFlags + leftUnits + leftBelowUnits - 1));
#else
    (neighborFlags + leftUnits - 1));
#endif
  leftAvailable = availableUnit == tuHeightInUnits;
#if JVET_L0338_MDLM
  if (leftAvailable) // if left is not available, then the below left is not available
  {
    avaiLeftUnits = tuHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.bottomLeftComp(chromaArea.compID), leftBelowUnits, unitHeight, (neighborFlags + leftBelowUnits - 1));
  }
  if (aboveAvailable) // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = tuWidthInUnits;
    avaiAboveRightUnits = isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.topRightComp(chromaArea.compID), aboveRightUnits, unitWidth, (neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1));
  }
#endif
  Pel *srcColor0, *curChroma0;
  int  srcStride, curStride;

  PelBuf temp;
#if JVET_L0338_MDLM
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp = PelBuf(m_pMdlmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
#endif
  srcStride = MAX_CU_SIZE + 1;
  temp        = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
#if JVET_L0338_MDLM
  }
#endif
  srcColor0 = temp.bufAt(0, 0);
  curChroma0 = getPredictorPtr(compID);

  curStride = m_topRefLength + 1;

  curChroma0 += curStride + 1;

  unsigned internalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  int minLuma[2] = { MAX_INT, 0 };
  int maxLuma[2] = { -MAX_INT, 0 };

  Pel *src = srcColor0 - srcStride;
  Pel *cur = curChroma0 - curStride;
#if JVET_L0338_MDLM
  int minDim = 1;
  int actualTopTemplateSampNum = 0;
  int actualLeftTemplateSampNum = 0;
  if (curChromaMode == MDLM_T_IDX)
  {
    leftAvailable = 0;
    actualTopTemplateSampNum = unitWidth*(avaiAboveUnits + avaiAboveRightUnits);
    minDim = actualTopTemplateSampNum;
  }
  else if (curChromaMode == MDLM_L_IDX)
  {
    aboveAvailable = 0;
    actualLeftTemplateSampNum = unitHeight*(avaiLeftUnits + avaiLeftBelowUnits);
    minDim = actualLeftTemplateSampNum;
  }
  else if (curChromaMode == LM_CHROMA_IDX)
  {
    actualTopTemplateSampNum = cWidth;
    actualLeftTemplateSampNum = cHeight;
    minDim = leftAvailable && aboveAvailable ? 1 << g_aucPrevLog2[std::min(actualLeftTemplateSampNum, actualTopTemplateSampNum)]
      : 1 << g_aucPrevLog2[leftAvailable ? actualLeftTemplateSampNum : actualTopTemplateSampNum];
  }
#endif
#if !JVET_L0338_MDLM
  int minDim = leftAvailable && aboveAvailable ? 1 << g_aucPrevLog2[std::min(cHeight, cWidth)]
                                                   : 1 << g_aucPrevLog2[leftAvailable ? cHeight : cWidth];
#endif
  int numSteps = minDim;

  if (aboveAvailable)
  {
    for (int j = 0; j < numSteps; j++)
    {
#if JVET_L0338_MDLM
      int idx = (j * actualTopTemplateSampNum) / minDim;
#else
      int idx = (j * cWidth) / minDim;
#endif

      if (minLuma[0] > src[idx])
      {
        minLuma[0] = src[idx];
        minLuma[1] = cur[idx];
      }
      if (maxLuma[0] < src[idx])
      {
        maxLuma[0] = src[idx];
        maxLuma[1] = cur[idx];
      }
    }
  }

  if (leftAvailable)
  {
    src = srcColor0 - 1;
    cur = curChroma0 - 1;

    for (int i = 0; i < numSteps; i++)
    {
#if JVET_L0338_MDLM
      int idx = (i * actualLeftTemplateSampNum) / minDim;
#else
      int idx = (i * cHeight) / minDim;
#endif

      if (minLuma[0] > src[srcStride * idx])
      {
        minLuma[0] = src[srcStride * idx];
        minLuma[1] = cur[curStride * idx];
      }
      if (maxLuma[0] < src[srcStride * idx])
      {
        maxLuma[0] = src[srcStride * idx];
        maxLuma[1] = cur[curStride * idx];
      }
    }
  }

  if ((leftAvailable || aboveAvailable))
  {
    a         = 0;
    iShift    = 16;
    int shift = (internalBitDepth > 8) ? internalBitDepth - 9 : 0;
    int add   = shift ? 1 << (shift - 1) : 0;
    int diff  = (maxLuma[0] - minLuma[0] + add) >> shift;
    if (diff > 0)
    {
      int div = ((maxLuma[1] - minLuma[1]) * g_aiLMDivTableLow[diff - 1] + 32768) >> 16;
      a       = (((maxLuma[1] - minLuma[1]) * g_aiLMDivTableHigh[diff - 1] + div + add) >> shift);
    }
    b = minLuma[1] - ((a * minLuma[0]) >> iShift);
  }
  else
  {
    a = 0;

    b = 1 << (internalBitDepth - 1);

    iShift = 0;
  }
}
#else
static int GetFloorLog2( unsigned x )
{
  int bits = -1;
  while( x > 0 )
  {
    bits++;
    x >>= 1;
  }
  return bits;
}


void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea,
  int& a, int&  b, int& iShift)
{
  CHECK( compID == COMPONENT_Y, "" );

  const SizeType uiCWidth  = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure&  cs = *(pu.cs);
  const CodingUnit& cu = *(pu.cu);

  const SPS &sps        = *cs.sps;
  const uint32_t uiTuWidth  = chromaArea.width;
  const uint32_t uiTuHeight = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int iBaseUnitSize = 1 << MIN_CU_LOG2;
  const int  iUnitWidth   = iBaseUnitSize >> getComponentScaleX( chromaArea.compID, nChromaFormat );
  const int  iUnitHeight  = iBaseUnitSize >> getComponentScaleX( chromaArea.compID, nChromaFormat );


  const int  iTUWidthInUnits  = uiTuWidth / iUnitWidth;
  const int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const int  iAboveUnits      = iTUWidthInUnits;
  const int  iLeftUnits       = iTUHeightInUnits;
#if JVET_L0338_MDLM
  int topTemplateSampNum = 2 * uiCWidth; // for MDLM, the template sample number is 2W or 2H;
  int leftTemplateSampNum = 2 * uiCHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  int totalAboveUnits = (topTemplateSampNum + (iUnitWidth - 1)) / iUnitWidth;
  int totalLeftUnits = (leftTemplateSampNum + (iUnitHeight - 1)) / iUnitHeight;
  int totalUnits = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits = totalAboveUnits - iAboveUnits;
  int leftBelowUnits = totalLeftUnits - iLeftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  int avaiAboveUnits = 0;
  int avaiLeftUnits = 0;

  int curChromaMode = pu.intraDir[1];
#endif
  bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
#if JVET_L0338_MDLM
  memset(bNeighborFlags, 0, totalUnits);
#else
  memset( bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits );
#endif

  bool bAboveAvaillable, bLeftAvaillable;

  int availlableUnit = isAboveAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, iAboveUnits, iUnitWidth, 
#if JVET_L0338_MDLM
    (bNeighborFlags + iLeftUnits + leftBelowUnits + 1 ) );
#else
    ( bNeighborFlags + iLeftUnits + 1 ) );
#endif
  bAboveAvaillable = availlableUnit == iTUWidthInUnits;

  availlableUnit = isLeftAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, iLeftUnits, iUnitHeight, 
#if JVET_L0338_MDLM
    (bNeighborFlags + iLeftUnits + leftBelowUnits - 1 ) );
#else
    ( bNeighborFlags + iLeftUnits - 1 ) );
#endif
  bLeftAvaillable = availlableUnit == iTUHeightInUnits;
#if JVET_L0338_MDLM
  if (bLeftAvaillable) // if left is not available, then the below left is not available
  {
    avaiLeftUnits = iTUHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.bottomLeftComp(chromaArea.compID), leftBelowUnits, iUnitHeight, (bNeighborFlags + leftBelowUnits - 1));
  }
  if (bAboveAvaillable) // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = iTUWidthInUnits;
    avaiAboveRightUnits = isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.topRightComp(chromaArea.compID), aboveRightUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1));
  }
#endif
  Pel *pSrcColor0, *pCurChroma0;
  int  iSrcStride,  iCurStride;

  PelBuf Temp;  
#if JVET_L0338_MDLM
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    iSrcStride = 2 * MAX_CU_SIZE + 1;
    Temp = PelBuf(m_pMdlmTemp + iSrcStride + 1, iSrcStride, Size(chromaArea));
  }
  else
  {
#endif
  iSrcStride = MAX_CU_SIZE + 1;
  Temp = PelBuf(m_piTemp + iSrcStride + 1, iSrcStride, Size(chromaArea));
#if JVET_L0338_MDLM
  }
#endif
  pSrcColor0 = Temp.bufAt(0, 0);
  pCurChroma0 = getPredictorPtr(compID);
  iCurStride = m_topRefLength + 1;
  pCurChroma0 += iCurStride + 1;
  int x = 0, y = 0, xx = 0, xy = 0;
  int iCountShift = 0;
  unsigned uiInternalBitDepth = sps.getBitDepth( CHANNEL_TYPE_CHROMA );

  Pel *pSrc = pSrcColor0  - iSrcStride;
  Pel *pCur = pCurChroma0 - iCurStride;
#if JVET_L0338_MDLM
  //get the temp buffer to store the downsampled luma and chroma
  Pel* pTempBufferSrc = new Pel[2 * MAX_CU_SIZE]; // for MDLM, use tempalte size 2W or 2H,
  Pel* pTempBufferCur = new Pel[2 * MAX_CU_SIZE];

  int actualTopTemplateSampNum = iUnitWidth*(avaiAboveUnits + avaiAboveRightUnits);
  int actualLeftTemplateSampNum = iUnitHeight*(avaiLeftUnits + avaiLeftBelowUnits);

  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    if (curChromaMode == MDLM_T_IDX)
    {
      if (bAboveAvaillable)
      {
        for (int j = 0; j < actualTopTemplateSampNum; j++)
        {
          pTempBufferSrc[j] = pSrc[j];
          pTempBufferCur[j] = pCur[j];
        }
      }
    }
    else
    {
      if (bLeftAvaillable)
      {
        pSrc = pSrcColor0 - 1;
        pCur = pCurChroma0 - 1;
        for (int i = 0; i < actualLeftTemplateSampNum; i++)
        {
          pTempBufferSrc[i] = pSrc[iSrcStride *i];
          pTempBufferCur[i] = pCur[iCurStride *i];
        }
      }
    }
    //pad the temple sample to targetSampNum.
    int orgNumSample = (curChromaMode == MDLM_T_IDX) ? (avaiAboveUnits*iUnitWidth) : (avaiLeftUnits*iUnitHeight);
    int existSampNum = (curChromaMode == MDLM_T_IDX) ? actualTopTemplateSampNum : actualLeftTemplateSampNum;
    int targetSampNum = 1 << (g_aucLog2[existSampNum - 1] + 1);

    if (orgNumSample == 0)
    {
      delete[] pTempBufferSrc;
      delete[] pTempBufferCur;
      pTempBufferSrc = nullptr;
      pTempBufferCur = nullptr;

      a = 0;
      b = 1 << (uiInternalBitDepth - 1);
      iShift = 0;
      return;
    }
    if (targetSampNum != existSampNum)//if existSampNum not a value of power of 2
    {
      xPadMdlmTemplateSample(pTempBufferSrc, pTempBufferCur, uiCWidth, uiCHeight, existSampNum, targetSampNum);
    }
    for (int j = 0; j < targetSampNum; j++)
    {
      x += pTempBufferSrc[j];
      y += pTempBufferCur[j];
      xx += pTempBufferSrc[j] * pTempBufferSrc[j];
      xy += pTempBufferSrc[j] * pTempBufferCur[j];
    }
    iCountShift = g_aucLog2[targetSampNum];
  }
  else
  {
#endif
  int       minDim        = bLeftAvaillable && bAboveAvaillable ? 1 << g_aucPrevLog2[std::min( uiCHeight, uiCWidth )] : 1 << g_aucPrevLog2[bLeftAvaillable ? uiCHeight : uiCWidth];
  int       minStep       = 1;
  int       numSteps      = cs.pcv->rectCUs ? minDim / minStep : minDim;

  if( bAboveAvaillable )
  {
    for( int j = 0; j < numSteps; j++ )
    {
      int idx = ( j * minStep * uiCWidth ) / minDim;

      x  += pSrc[idx];
      y  += pCur[idx];
      xx += pSrc[idx] * pSrc[idx];
      xy += pSrc[idx] * pCur[idx];
    }

    iCountShift = g_aucLog2[minDim / minStep];
  }

  if( bLeftAvaillable )
  {
    pSrc = pSrcColor0  - 1;
    pCur = pCurChroma0 - 1;

    for( int i = 0; i < numSteps; i++ )
    {
      int idx = ( i * uiCHeight * minStep ) / minDim;
      x  += pSrc[iSrcStride * idx];
      y  += pCur[iCurStride * idx];
      xx += pSrc[iSrcStride * idx] * pSrc[iSrcStride * idx];
      xy += pSrc[iSrcStride * idx] * pCur[iCurStride * idx];
    }

    iCountShift += bAboveAvaillable ? 1 : g_aucLog2[minDim / minStep];
  }
#if JVET_L0338_MDLM
  }

  delete[] pTempBufferSrc;
  delete[] pTempBufferCur;
  pTempBufferSrc = nullptr;
  pTempBufferCur = nullptr;
#endif
#if JVET_L0338_MDLM
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    if ((curChromaMode == MDLM_L_IDX) ? (!bLeftAvaillable) : (!bAboveAvaillable))
    {
      a = 0;
      b = 1 << (uiInternalBitDepth - 1);
      iShift = 0;
      return;
    }
  }
  else
  {
#endif
  if( !bLeftAvaillable && !bAboveAvaillable )
  {
    a = 0;
      b = 1 << ( uiInternalBitDepth - 1 );
    iShift = 0;
    return;
  }
#if JVET_L0338_MDLM
  }
#endif
  int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if( iTempShift > 0 )
  {
    x  = ( x  + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    y  = ( y  + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xx = ( xx + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xy = ( xy + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    iCountShift -= iTempShift;
  }


  /////// xCalcLMParameters

  int avgX = x >> iCountShift;
  int avgY = y >> iCountShift;

  int RErrX = x & ( ( 1 << iCountShift ) - 1 );
  int RErrY = y & ( ( 1 << iCountShift ) - 1 );

  int iB = 7;
  iShift = 13 - iB;


  if( iCountShift == 0 )
  {
    a = 0;
    b = 1 << ( uiInternalBitDepth - 1 );
    iShift = 0;
  }
  else
  {
    int a1 = xy - ( avgX * avgY << iCountShift ) -     avgX * RErrY - avgY * RErrX;
    int a2 = xx - ( avgX * avgX << iCountShift ) - 2 * avgX * RErrX;
    const int iShiftA1 = uiInternalBitDepth - 2;
    const int iShiftA2 = 5;
    const int iAccuracyShift = uiInternalBitDepth + 4;

    int iScaleShiftA2 = 0;
    int iScaleShiftA1 = 0;
    int a1s = a1;
    int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2( abs( a1 ) ) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2( abs( a2 ) ) - iShiftA2;

    if( iScaleShiftA1 < 0 )
    {
      iScaleShiftA1 = 0;
    }

    if( iScaleShiftA2 < 0 )
    {
      iScaleShiftA2 = 0;
    }

    int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if( a2s >= 32 )
    {
      uint32_t a2t = m_auShiftLM[a2s - 32];
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if( iScaleShiftA < 0 )
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3( -( 1 << ( 15 - iB ) ), ( 1 << ( 15 - iB ) ) - 1, a );
    a = a << iB;

    int16_t n = 0;
    if( a != 0 )
    {
      n = GetFloorLog2( abs( a ) + ( ( a < 0 ? -1 : 1 ) - 1 ) / 2 ) - 5;
    }

    iShift = ( iShift + iB ) - n;
    a = a >> n;

    b = avgY - ( ( a * avgX ) >> iShift );
  }
}
#endif

//! \}
