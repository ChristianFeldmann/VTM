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

/** \file     Rom.cpp
    \brief    global variables & functions
*/

#include "Rom.h"
#include "UnitTools.h"

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

#if ENABLE_TRACING
CDTrace *g_trace_ctx = NULL;
#endif
bool g_mctsDecCheckEnabled = false;


//! \ingroup CommonLib
//! \{

MsgLevel g_verbosity = VERBOSE;

const char* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
#if JVET_N0067_NAL_Unit_Header
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  case NAL_UNIT_APS:                    return "APS";
  case NAL_UNIT_CODED_SLICE_TRAIL:      return "TRAIL";
  case NAL_UNIT_CODED_SLICE_STSA:       return "STSA";
  case NAL_UNIT_CODED_SLICE_RADL:       return "RADL";
  case NAL_UNIT_CODED_SLICE_RASL:       return "RASL";
  case NAL_UNIT_DPS:                    return "DPS";
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_VPS:                    return "VPS";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA"; 
  case NAL_UNIT_CODED_SLICE_GRA:        return "GRA";
  default:                              return "UNK";
#else
#if JVET_M0101_HLS
  case NAL_UNIT_CODED_SLICE_TRAIL:      return "TRAIL";
  case NAL_UNIT_CODED_SLICE_STSA:       return "STSA";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL:       return "RADL";
  case NAL_UNIT_CODED_SLICE_RASL:       return "RASL";
#if HEVC_VPS
  case NAL_UNIT_VPS:                    return "VPS";
#endif
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_APS:                    return "APS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
#else
  case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
  case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
  case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
  case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
  case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
  case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
  case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
  case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
  case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
  case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
  case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
  case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
#if HEVC_VPS
  case NAL_UNIT_VPS:                    return "VPS";
#endif
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_APS:                    return "APS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
#endif
#endif
  }
}

class ScanGenerator
{
private:
  uint32_t m_line, m_column;
  const uint32_t m_blockWidth, m_blockHeight;
  const uint32_t m_stride;
  const CoeffScanType m_scanType;

public:
  ScanGenerator(uint32_t blockWidth, uint32_t blockHeight, uint32_t stride, CoeffScanType scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  uint32_t GetCurrentX() const { return m_column; }
  uint32_t GetCurrentY() const { return m_line; }

  uint32_t GetNextIndex(uint32_t blockOffsetX, uint32_t blockOffsetY)
  {
    const uint32_t rtn = ((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:

        if ((m_column == m_blockWidth - 1) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
        {
          m_line += m_column + 1;
          m_column = 0;

          if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
          {
            m_column += m_line - (m_blockHeight - 1);
            m_line = m_blockHeight - 1;
          }
        }
        else
        {
          m_column++;
          m_line--;
        }
        break;

#if HEVC_USE_MDCS
      //------------------------------------------------
      case SCAN_HOR:

        if (m_column == m_blockWidth - 1)
        {
          m_line++;
          m_column = 0;
        }
        else
        {
          m_column++;
        }
        break;

      //------------------------------------------------

      case SCAN_VER:

        if (m_line == m_blockHeight - 1)
        {
          m_column++;
          m_line = 0;
        }
        else
        {
          m_line++;
        }
        break;

#endif
      //------------------------------------------------

      default:

        THROW("ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex");
        break;
    }

    return rtn;
  }
};
const int8_t g_GbiLog2WeightBase = 3;
const int8_t g_GbiWeightBase = (1 << g_GbiLog2WeightBase);
const int8_t g_GbiWeights[GBI_NUM] = { -2, 3, 4, 5, 10 };
const int8_t g_GbiSearchOrder[GBI_NUM] = { GBI_DEFAULT, GBI_DEFAULT - 2, GBI_DEFAULT + 2, GBI_DEFAULT - 1, GBI_DEFAULT + 1 };
int8_t g_GbiCodingOrder[GBI_NUM];
int8_t g_GbiParsingOrder[GBI_NUM];

int8_t getGbiWeight(uint8_t gbiIdx, uint8_t uhRefFrmList)
{
  // Weghts for the model: P0 + w * (P1 - P0) = (1-w) * P0 + w * P1
  // Retuning  1-w for P0 or w for P1
  return (uhRefFrmList == REF_PIC_LIST_0 ? g_GbiWeightBase - g_GbiWeights[gbiIdx] : g_GbiWeights[gbiIdx]);
}

void resetGbiCodingOrder(bool bRunDecoding, const CodingStructure &cs)
{
  // Form parsing order: { GBI_DEFAULT, GBI_DEFAULT+1, GBI_DEFAULT-1, GBI_DEFAULT+2, GBI_DEFAULT-2, ... }
  g_GbiParsingOrder[0] = GBI_DEFAULT;
  for (int i = 1; i <= (GBI_NUM >> 1); ++i)
  {
    g_GbiParsingOrder[2 * i - 1] = GBI_DEFAULT + (int8_t)i;
    g_GbiParsingOrder[2 * i] = GBI_DEFAULT - (int8_t)i;
  }

  // Form encoding order
  if (!bRunDecoding)
  {
    for (int i = 0; i < GBI_NUM; ++i)
    {
      g_GbiCodingOrder[(uint32_t)g_GbiParsingOrder[i]] = i;
    }
  }
}

uint32_t deriveWeightIdxBits(uint8_t gbiIdx) // Note: align this with TEncSbac::codeGbiIdx and TDecSbac::parseGbiIdx
{
  uint32_t numBits = 1;
  uint8_t  gbiCodingIdx = (uint8_t)g_GbiCodingOrder[gbiIdx];

  if (GBI_NUM > 2 && gbiCodingIdx != 0)
  {
    uint32_t prefixNumBits = GBI_NUM - 2;
    uint32_t step = 1;
    uint8_t  prefixSymbol = gbiCodingIdx;

    // Truncated unary code
    uint8_t idx = 1;
    for (int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (prefixSymbol == idx)
      {
        ++numBits;
        break;
      }
      else
      {
        ++numBits;
        idx += step;
      }
    }
  }
  return numBits;
}

#if JVET_N0103_CGSIZE_HARMONIZATION
uint32_t g_log2SbbSize[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][2] =
//===== luma/chroma =====
{
  { { 0,0 },{ 0,1 },{ 0,2 },{ 0,3 },{ 0,4 },{ 0,4 },{ 0,4 },{ 0,4 } },
  { { 1,0 },{ 1,1 },{ 1,1 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 } },
  { { 2,0 },{ 1,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 3,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } }
};
#else
uint32_t g_log2SbbSize[2][MAX_CU_DEPTH+1][MAX_CU_DEPTH+1][2] =
{
  //===== luma =====
  {
    { {0,0}, {0,1}, {0,2}, {0,3}, {0,4}, {0,4}, {0,4}, {0,4} },
    { {1,0}, {1,1}, {1,2}, {1,3}, {1,3}, {1,3}, {1,3}, {1,3} },
    { {2,0}, {2,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {3,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} }
  },
  //===== chroma =====
  {
    { {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0} },
    { {0,0}, {1,1}, {1,1}, {1,1}, {1,1}, {1,1}, {1,1}, {1,1} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} }
  },
};
#endif
// initialize ROM variables
void initROM()
{
  int c;

  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
  ::memset(g_aucLog2, 0, sizeof(g_aucLog2));
  c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
    g_aucNextLog2[i] = i <= 1 ? 0 : c + 1;

    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

    g_aucPrevLog2[i] = c;
    g_aucLog2    [i] = c;
  }


  gp_sizeIdxInfo = new SizeIndexInfoLog2();
  gp_sizeIdxInfo->init(MAX_CU_SIZE);


  SizeIndexInfoLog2 sizeInfo;
  sizeInfo.init(MAX_CU_SIZE);

#if !JVET_N0103_CGSIZE_HARMONIZATION
  for( int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ )
  {
#endif
  // initialize scan orders
  for (uint32_t blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++)
  {
    for (uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
    {
      const uint32_t blockWidth  = sizeInfo.sizeFrom(blockWidthIdx);
      const uint32_t blockHeight = sizeInfo.sizeFrom(blockHeightIdx);
      const uint32_t totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);
        ScanElement *       scan     = nullptr;

        if (blockWidthIdx < sizeInfo.numWidths() && blockHeightIdx < sizeInfo.numHeights())
        {
          scan = new ScanElement[totalValues];
        }

#if JVET_N0103_CGSIZE_HARMONIZATION
        g_scanOrder[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx] = scan;
#else
        g_scanOrder[ch][SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx] = scan;
#endif

        if (scan == nullptr)
        {
          continue;
        }

        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);

        for (uint32_t scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
          const int posY      = rasterPos / blockWidth;
          const int posX      = rasterPos - ( posY * blockWidth );

          scan[scanPosition].idx = rasterPos;
          scan[scanPosition].x   = posX;
          scan[scanPosition].y   = posY;
        }
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
#if JVET_N0103_CGSIZE_HARMONIZATION
      const uint32_t* log2Sbb        = g_log2SbbSize[g_aucLog2[blockWidth]][g_aucLog2[blockHeight]];
#else
      const uint32_t* log2Sbb        = g_log2SbbSize[ch][ g_aucLog2[blockWidth] ][ g_aucLog2[blockHeight] ];
#endif
      const uint32_t  log2CGWidth    = log2Sbb[0];
      const uint32_t  log2CGHeight   = log2Sbb[1];

      const uint32_t  groupWidth     = 1 << log2CGWidth;
      const uint32_t  groupHeight    = 1 << log2CGHeight;
      const uint32_t  widthInGroups = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockWidth) >> log2CGWidth;
      const uint32_t  heightInGroups = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockHeight) >> log2CGHeight;

      const uint32_t  groupSize      = groupWidth    * groupHeight;
      const uint32_t  totalGroups    = widthInGroups * heightInGroups;

      for (uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        ScanElement *scan = new ScanElement[totalValues];

#if JVET_N0103_CGSIZE_HARMONIZATION
        g_scanOrder[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx] = scan;
#else
        g_scanOrder[ch][SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx] = scan;
#endif

        if ( blockWidth > JVET_C0024_ZERO_OUT_TH || blockHeight > JVET_C0024_ZERO_OUT_TH )
        {
          for (uint32_t i = 0; i < totalValues; i++)
          {
            scan[i].idx = totalValues - 1;
            scan[i].x   = blockWidth - 1;
            scan[i].y   = blockHeight - 1;
          }
        }

        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (uint32_t groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const uint32_t groupPositionY  = fullBlockScan.GetCurrentY();
          const uint32_t groupPositionX  = fullBlockScan.GetCurrentX();
          const uint32_t groupOffsetX    = groupPositionX * groupWidth;
          const uint32_t groupOffsetY    = groupPositionY * groupHeight;
          const uint32_t groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (uint32_t scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
            const int posY      = rasterPos / blockWidth;
            const int posX      = rasterPos - ( posY * blockWidth );

            scan[groupOffsetScan + scanPosition].idx = rasterPos;
            scan[groupOffsetScan + scanPosition].x   = posX;
            scan[groupOffsetScan + scanPosition].y   = posY;
          }

          fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }
#if !JVET_N0103_CGSIZE_HARMONIZATION
  }
#endif

#if JVET_N0193_LFNST
  // initialize CoefTopLeftDiagScan8x8 for LFNST
  for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++ )
  {
    const uint32_t blockWidth = sizeInfo.sizeFrom( blockWidthIdx );

    const static uint8_t g_auiXYDiagScan8x8[ 64 ][ 2 ] =
    {
      { 0, 0 }, { 0, 1 }, { 1, 0 }, { 0, 2 }, { 1, 1 }, { 2, 0 }, { 0, 3 }, { 1, 2 },
      { 2, 1 }, { 3, 0 }, { 1, 3 }, { 2, 2 }, { 3, 1 }, { 2, 3 }, { 3, 2 }, { 3, 3 },
      { 0, 4 }, { 0, 5 }, { 1, 4 }, { 0, 6 }, { 1, 5 }, { 2, 4 }, { 0, 7 }, { 1, 6 },
      { 2, 5 }, { 3, 4 }, { 1, 7 }, { 2, 6 }, { 3, 5 }, { 2, 7 }, { 3, 6 }, { 3, 7 },
      { 4, 0 }, { 4, 1 }, { 5, 0 }, { 4, 2 }, { 5, 1 }, { 6, 0 }, { 4, 3 }, { 5, 2 },
      { 6, 1 }, { 7, 0 }, { 5, 3 }, { 6, 2 }, { 7, 1 }, { 6, 3 }, { 7, 2 }, { 7, 3 },
      { 4, 4 }, { 4, 5 }, { 5, 4 }, { 4, 6 }, { 5, 5 }, { 6, 4 }, { 4, 7 }, { 5, 6 },
      { 6, 5 }, { 7, 4 }, { 5, 7 }, { 6, 6 }, { 7, 5 }, { 6, 7 }, { 7, 6 }, { 7, 7 }
    };
    for( int i = 0; i < 64; i++ )
    {
      g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ].idx = g_auiXYDiagScan8x8[ i ][ 0 ] + g_auiXYDiagScan8x8[ i ][ 1 ] * blockWidth;
      g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ].x   = g_auiXYDiagScan8x8[ i ][ 0 ];
      g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ].y   = g_auiXYDiagScan8x8[ i ][ 1 ];
    }
  }
#endif

  for( int idxH = MAX_CU_DEPTH - MIN_CU_LOG2; idxH >= 0; --idxH )
  {
    for( int idxW = MAX_CU_DEPTH - MIN_CU_LOG2; idxW >= 0; --idxW )
    {
      int numW   = 1 << idxW;
      int numH   = 1 << idxH;
      int ratioW = std::max( 0, idxW - idxH );
      int ratioH = std::max( 0, idxH - idxW );
      int sum    = std::max( (numW >> ratioW), (numH >> ratioH) ) - 1;
      for( int y = 0; y < numH; y++ )
      {
        int idxY = y >> ratioH;
        for( int x = 0; x < numW; x++ )
        {
          int idxX = x >> ratioW;
          g_triangleMvStorage[TRIANGLE_DIR_135][idxH][idxW][y][x] = (idxX == idxY) ? 2 : (idxX > idxY ? 0 : 1);
          g_triangleMvStorage[TRIANGLE_DIR_45][idxH][idxW][y][x] = (idxX + idxY == sum) ? 2 : (idxX + idxY > sum ? 1 : 0);
        }
      }
    }
  }
}

void destroyROM()
{
  unsigned numWidths = gp_sizeIdxInfo->numAllWidths();
  unsigned numHeights = gp_sizeIdxInfo->numAllHeights();

#if JVET_N0103_CGSIZE_HARMONIZATION
  for (uint32_t groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (uint32_t scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
      for (uint32_t blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++)
      {
        for (uint32_t blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++)
        {
          delete[] g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
          g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;
        }
      }
    }
  }
#else
  for( uint32_t ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ )
  {
    for( uint32_t groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++ )
    {
      for( uint32_t scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++ )
      {
        for( uint32_t blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++ )
        {
          for( uint32_t blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++ )
          {
            delete[] g_scanOrder[ch][groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
            g_scanOrder[ch][groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;
          }
        }
      }
    }
  }
#endif

  delete gp_sizeIdxInfo;
  gp_sizeIdxInfo = nullptr;
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

#if JVET_N0246_MODIFIED_QUANTSCALES
const int g_quantScales[2][SCALING_LIST_REM_NUM] = // can be represented as a 9 element table
{
    { 26214,23302,20560,18396,16384,14564 },
    { 18396,16384,14564,13107,11651,10280 } // Note: last 3 values of second row == half of the first 3 values of the first row
};

const int g_invQuantScales[2][SCALING_LIST_REM_NUM] = // can be represented as a 9 element table
{
  { 40,45,51,57,64,72 },
  { 57,64,72,80,90,102 } // Note: last 3 values of second row == double of the first 3 values of the first row
};
#else
const int g_quantScales[SCALING_LIST_REM_NUM] =
{
  26214,23302,20560,18396,16384,14564
};

const int g_invQuantScales[SCALING_LIST_REM_NUM] =
{
  40,45,51,57,64,72
};
#endif

//--------------------------------------------------------------------------------------------------
//structures
//--------------------------------------------------------------------------------------------------
//coefficients
//--------------------------------------------------------------------------------------------------

const uint8_t g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize] =
{
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,34,35,35,36,36,37,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,63,63,63,63,63,63 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,63,63,63,63,63,63 }
};

// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const uint8_t g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1] =
{
  {3, 3, 3, 3, 2, 2},  //   4x4,   4x8,   4x16,   4x32,   4x64,   4x128,
  {3, 3, 3, 3, 3, 2},  //   8x4,   8x8,   8x16,   8x32,   8x64,   8x128,
  {3, 3, 3, 3, 3, 2},  //  16x4,  16x8,  16x16,  16x32,  16x64,  16x128,
  {3, 3, 3, 3, 3, 2},  //  32x4,  32x8,  32x16,  32x32,  32x64,  32x128,
  {2, 3, 3, 3, 3, 2},  //  64x4,  64x8,  64x16,  64x32,  64x64,  64x128,
  {2, 2, 2, 2, 2, 3},  // 128x4, 128x8, 128x16, 128x32, 128x64, 128x128,
};

const uint8_t g_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3,  //  64x64
  3   // 128x128
};
const uint8_t g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5,  //  64x64   33
  5   // 128x128
};

const uint8_t g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 2, 2, 2, 2, 2, 3,  4,  6,  8, 10, 12, 13, 14, 16, 18, 20, 22, 23, 24, 26, 28, 30, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 44, 44, 45, 46, 46, 46, 47, 48, 48, 48, 49, 50, 51, 52, 52, 52, 53, 54, 54, 54, 55, 56, 56, 56, 57, 58, 59, 60, DM_CHROMA_IDX };

#if JVET_N0217_MATRIX_INTRAPRED
extern const uint8_t  g_intraMode65to33AngMapping[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 3, 3, 4, 4, 5, 5,  6,  6,  7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, DM_CHROMA_IDX };

const uint8_t g_mapMipToAngular65[3][MAX_NUM_MIP_MODE] =
{
  {  0, 18, 18,  0, 18,  0, 12,  0, 18,  2, 18, 12, 18, 18,  1, 18, 18,  0,  0, 50,  0, 50,  0, 56,  0, 50, 66, 50, 56, 50, 50,  1, 50, 50, 50 },
  {  0,  1,  0,  1,  0, 22, 18, 18,  1,  0,  1,  0,  1,  0, 44,  0, 50,  1,  0 },
  {  1,  1,  1,  1, 18,  0,  1,  0,  1, 50,  0 },
};
const uint8_t g_mapAngular33ToMip[3][35] =
{
  { 17, 17, 17,  9,  9,  9,  9, 17, 17, 17, 17, 17, 17, 17,  5,  5,  5,  5, 34, 22, 22, 22, 22, 34, 34, 34, 34, 34, 34, 34, 26, 26, 26, 26, 26 },
  {  0,  0, 10, 10, 10, 10, 10,  4,  6,  7,  7,  7,  5,  5,  0,  0,  3,  3, 12, 12, 12, 12, 14, 14, 14, 16, 16, 16, 15, 13,  1,  1,  1,  1,  1 },
  {  5,  1,  3,  3,  3,  3,  0,  0,  0,  4,  4,  4,  5,  1,  1,  1,  1,  1,  6,  6,  6,  6,  6, 10, 10,  9,  9,  9,  9,  9,  8,  8,  8,  8,  8 },
};

const int g_sortedMipMpms[3][NUM_MPM_MIP] =
{
  { 17, 34,  5 },
  {  0,  7, 16 },
  {  1,  4,  6 },
};
#endif



// ====================================================================================================================
// Misc.
// ====================================================================================================================
SizeIndexInfo*           gp_sizeIdxInfo = NULL;
int8_t                    g_aucLog2    [MAX_CU_SIZE + 1];
int8_t                    g_aucNextLog2[MAX_CU_SIZE + 1];
int8_t                    g_aucPrevLog2[MAX_CU_SIZE + 1];

UnitScale g_miScaling( MIN_CU_LOG2, MIN_CU_LOG2 );


// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
#if JVET_N0103_CGSIZE_HARMONIZATION
ScanElement *g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
#else
ScanElement *g_scanOrder[2][SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
#endif
#if JVET_N0193_LFNST
ScanElement  g_coefTopLeftDiagScan8x8[ MAX_CU_SIZE / 2 + 1 ][ 64 ];
#endif

const uint32_t g_uiMinInGroup[LAST_SIGNIFICANT_GROUPS] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96 };
const uint32_t g_uiGroupIdx[MAX_TB_SIZEY] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11 };
const uint32_t g_auiGoRiceParsCoeff[32] =
{
  0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3
};
const uint32_t g_auiGoRicePosCoeff0[3][32] =
{
  {0, 0, 0, 0, 0, 1, 2,    2, 2, 2, 2, 2, 4, 4,    4, 4, 4, 4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,     8,  8,  8,  8},
  {1, 1, 1, 1, 2, 3, 4,    4, 4, 6, 6, 6, 8, 8,    8, 8, 8, 8, 12, 12, 12, 12, 12, 12, 12, 12, 16, 16,    16, 16, 16, 16},
  {1, 1, 2, 2, 2, 3, 4,    4, 4, 6, 6, 6, 8, 8,    8, 8, 8, 8, 12, 12, 12, 12, 12, 12, 12, 16, 16, 16,    16, 16, 16, 16}
};

#if HEVC_USE_SCALING_LISTS
const char *MatrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
    "INTRA2X2_LUMA",
    "INTRA2X2_CHROMAU",
    "INTRA2X2_CHROMAV",
    "INTER2X2_LUMA",
    "INTER2X2_CHROMAU",
    "INTER2X2_CHROMAV"
  },
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
   "INTRA32X32_LUMA",
   "INTRA32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTRA32X32_CHROMAV_FROM16x16_CHROMAV",
   "INTER32X32_LUMA",
   "INTER32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTER32X32_CHROMAV_FROM16x16_CHROMAV"
  },
};

const char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
  },
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTRA32X32_CHROMAV_DC_FROM16x16_CHROMAV",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTER32X32_CHROMAV_DC_FROM16x16_CHROMAV"
  },
};

const int g_quantTSDefault4x4[4 * 4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const int g_quantIntraDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,21,24,
  16,16,16,16,17,19,22,25,
  16,16,17,18,20,22,25,29,
  16,16,18,21,24,27,31,36,
  17,17,20,24,30,35,41,47,
  18,19,22,27,35,44,54,65,
  21,22,25,31,41,54,70,88,
  24,25,29,36,47,65,88,115
};

const int g_quantInterDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,20,24,
  16,16,16,17,18,20,24,25,
  16,16,17,18,20,24,25,28,
  16,17,18,20,24,25,28,33,
  17,18,20,24,25,28,33,41,
  18,20,24,25,28,33,41,54,
  20,24,25,28,33,41,54,71,
  24,25,28,33,41,54,71,91
};

const uint32_t g_scalingListSize [SCALING_LIST_SIZE_NUM] = { 4, 16, 64, 256, 1024, 4096, 16384 };
const uint32_t g_scalingListSizeX[SCALING_LIST_SIZE_NUM] = { 2,  4,  8,  16,   32,   64,   128 };
#endif


uint8_t g_triangleMvStorage[TRIANGLE_DIR_NUM][MAX_CU_DEPTH - MIN_CU_LOG2 + 1][MAX_CU_DEPTH - MIN_CU_LOG2 + 1][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];

//! \}
