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

/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>
#include <limits>



const uint8_t ProbModelTables::m_NextState[128][2] =
{
  {   2,  1 },{   0,  3 },{   4,  0 },{   1,  5 },{   6,  2 },{   3,  7 },{   8,  4 },{   5,  9 },
  {  10,  4 },{   5, 11 },{  12,  8 },{   9, 13 },{  14,  8 },{   9, 15 },{  16, 10 },{  11, 17 },
  {  18, 12 },{  13, 19 },{  20, 14 },{  15, 21 },{  22, 16 },{  17, 23 },{  24, 18 },{  19, 25 },
  {  26, 18 },{  19, 27 },{  28, 22 },{  23, 29 },{  30, 22 },{  23, 31 },{  32, 24 },{  25, 33 },
  {  34, 26 },{  27, 35 },{  36, 26 },{  27, 37 },{  38, 30 },{  31, 39 },{  40, 30 },{  31, 41 },
  {  42, 32 },{  33, 43 },{  44, 32 },{  33, 45 },{  46, 36 },{  37, 47 },{  48, 36 },{  37, 49 },
  {  50, 38 },{  39, 51 },{  52, 38 },{  39, 53 },{  54, 42 },{  43, 55 },{  56, 42 },{  43, 57 },
  {  58, 44 },{  45, 59 },{  60, 44 },{  45, 61 },{  62, 46 },{  47, 63 },{  64, 48 },{  49, 65 },
  {  66, 48 },{  49, 67 },{  68, 50 },{  51, 69 },{  70, 52 },{  53, 71 },{  72, 52 },{  53, 73 },
  {  74, 54 },{  55, 75 },{  76, 54 },{  55, 77 },{  78, 56 },{  57, 79 },{  80, 58 },{  59, 81 },
  {  82, 58 },{  59, 83 },{  84, 60 },{  61, 85 },{  86, 60 },{  61, 87 },{  88, 60 },{  61, 89 },
  {  90, 62 },{  63, 91 },{  92, 64 },{  65, 93 },{  94, 64 },{  65, 95 },{  96, 66 },{  67, 97 },
  {  98, 66 },{  67, 99 },{ 100, 66 },{  67,101 },{ 102, 68 },{  69,103 },{ 104, 68 },{  69,105 },
  { 106, 70 },{  71,107 },{ 108, 70 },{  71,109 },{ 110, 70 },{  71,111 },{ 112, 72 },{  73,113 },
  { 114, 72 },{  73,115 },{ 116, 72 },{  73,117 },{ 118, 74 },{  75,119 },{ 120, 74 },{  75,121 },
  { 122, 74 },{  75,123 },{ 124, 76 },{  77,125 },{ 124, 76 },{  77,125 },{ 126,126 },{ 127,127 }
};

const uint32_t ProbModelTables::m_EstFracBits[128] =
{
  0x07b23, 0x085f9, 0x074a0, 0x08cbc, 0x06ee4, 0x09354, 0x067f4, 0x09c1b, 0x060b0, 0x0a62a, 0x05a9c, 0x0af5b, 0x0548d, 0x0b955, 0x04f56, 0x0c2a9,
  0x04a87, 0x0cbf7, 0x045d6, 0x0d5c3, 0x04144, 0x0e01b, 0x03d88, 0x0e937, 0x039e0, 0x0f2cd, 0x03663, 0x0fc9e, 0x03347, 0x10600, 0x03050, 0x10f95,
  0x02d4d, 0x11a02, 0x02ad3, 0x12333, 0x0286e, 0x12cad, 0x02604, 0x136df, 0x02425, 0x13f48, 0x021f4, 0x149c4, 0x0203e, 0x1527b, 0x01e4d, 0x15d00,
  0x01c99, 0x166de, 0x01b18, 0x17017, 0x019a5, 0x17988, 0x01841, 0x18327, 0x016df, 0x18d50, 0x015d9, 0x19547, 0x0147c, 0x1a083, 0x0138e, 0x1a8a3,
  0x01251, 0x1b418, 0x01166, 0x1bd27, 0x01068, 0x1c77b, 0x00f7f, 0x1d18e, 0x00eda, 0x1d91a, 0x00e19, 0x1e254, 0x00d4f, 0x1ec9a, 0x00c90, 0x1f6e0,
  0x00c01, 0x1fef8, 0x00b5f, 0x208b1, 0x00ab6, 0x21362, 0x00a15, 0x21e46, 0x00988, 0x2285d, 0x00934, 0x22ea8, 0x008a8, 0x239b2, 0x0081d, 0x24577,
  0x007c9, 0x24ce6, 0x00763, 0x25663, 0x00710, 0x25e8f, 0x006a0, 0x26a26, 0x00672, 0x26f23, 0x005e8, 0x27ef8, 0x005ba, 0x284b5, 0x0055e, 0x29057,
  0x0050c, 0x29bab, 0x004c1, 0x2a674, 0x004a7, 0x2aa5e, 0x0046f, 0x2b32f, 0x0041f, 0x2c0ad, 0x003e7, 0x2ca8d, 0x003ba, 0x2d323, 0x0010c, 0x3bfbb
};

const BinFracBits ProbModelTables::m_BinFracBits_128[128] =
{
  {{0x07b23, 0x085f9}}, {{0x085f9, 0x07b23}},   {{0x074a0, 0x08cbc}}, {{0x08cbc, 0x074a0}},   {{0x06ee4, 0x09354}}, {{0x09354, 0x06ee4}},   {{0x067f4, 0x09c1b}}, {{0x09c1b, 0x067f4}},
  {{0x060b0, 0x0a62a}}, {{0x0a62a, 0x060b0}},   {{0x05a9c, 0x0af5b}}, {{0x0af5b, 0x05a9c}},   {{0x0548d, 0x0b955}}, {{0x0b955, 0x0548d}},   {{0x04f56, 0x0c2a9}}, {{0x0c2a9, 0x04f56}},
  {{0x04a87, 0x0cbf7}}, {{0x0cbf7, 0x04a87}},   {{0x045d6, 0x0d5c3}}, {{0x0d5c3, 0x045d6}},   {{0x04144, 0x0e01b}}, {{0x0e01b, 0x04144}},   {{0x03d88, 0x0e937}}, {{0x0e937, 0x03d88}},
  {{0x039e0, 0x0f2cd}}, {{0x0f2cd, 0x039e0}},   {{0x03663, 0x0fc9e}}, {{0x0fc9e, 0x03663}},   {{0x03347, 0x10600}}, {{0x10600, 0x03347}},   {{0x03050, 0x10f95}}, {{0x10f95, 0x03050}},
  {{0x02d4d, 0x11a02}}, {{0x11a02, 0x02d4d}},   {{0x02ad3, 0x12333}}, {{0x12333, 0x02ad3}},   {{0x0286e, 0x12cad}}, {{0x12cad, 0x0286e}},   {{0x02604, 0x136df}}, {{0x136df, 0x02604}},
  {{0x02425, 0x13f48}}, {{0x13f48, 0x02425}},   {{0x021f4, 0x149c4}}, {{0x149c4, 0x021f4}},   {{0x0203e, 0x1527b}}, {{0x1527b, 0x0203e}},   {{0x01e4d, 0x15d00}}, {{0x15d00, 0x01e4d}},
  {{0x01c99, 0x166de}}, {{0x166de, 0x01c99}},   {{0x01b18, 0x17017}}, {{0x17017, 0x01b18}},   {{0x019a5, 0x17988}}, {{0x17988, 0x019a5}},   {{0x01841, 0x18327}}, {{0x18327, 0x01841}},
  {{0x016df, 0x18d50}}, {{0x18d50, 0x016df}},   {{0x015d9, 0x19547}}, {{0x19547, 0x015d9}},   {{0x0147c, 0x1a083}}, {{0x1a083, 0x0147c}},   {{0x0138e, 0x1a8a3}}, {{0x1a8a3, 0x0138e}},
  {{0x01251, 0x1b418}}, {{0x1b418, 0x01251}},   {{0x01166, 0x1bd27}}, {{0x1bd27, 0x01166}},   {{0x01068, 0x1c77b}}, {{0x1c77b, 0x01068}},   {{0x00f7f, 0x1d18e}}, {{0x1d18e, 0x00f7f}},
  {{0x00eda, 0x1d91a}}, {{0x1d91a, 0x00eda}},   {{0x00e19, 0x1e254}}, {{0x1e254, 0x00e19}},   {{0x00d4f, 0x1ec9a}}, {{0x1ec9a, 0x00d4f}},   {{0x00c90, 0x1f6e0}}, {{0x1f6e0, 0x00c90}},
  {{0x00c01, 0x1fef8}}, {{0x1fef8, 0x00c01}},   {{0x00b5f, 0x208b1}}, {{0x208b1, 0x00b5f}},   {{0x00ab6, 0x21362}}, {{0x21362, 0x00ab6}},   {{0x00a15, 0x21e46}}, {{0x21e46, 0x00a15}},
  {{0x00988, 0x2285d}}, {{0x2285d, 0x00988}},   {{0x00934, 0x22ea8}}, {{0x22ea8, 0x00934}},   {{0x008a8, 0x239b2}}, {{0x239b2, 0x008a8}},   {{0x0081d, 0x24577}}, {{0x24577, 0x0081d}},
  {{0x007c9, 0x24ce6}}, {{0x24ce6, 0x007c9}},   {{0x00763, 0x25663}}, {{0x25663, 0x00763}},   {{0x00710, 0x25e8f}}, {{0x25e8f, 0x00710}},   {{0x006a0, 0x26a26}}, {{0x26a26, 0x006a0}},
  {{0x00672, 0x26f23}}, {{0x26f23, 0x00672}},   {{0x005e8, 0x27ef8}}, {{0x27ef8, 0x005e8}},   {{0x005ba, 0x284b5}}, {{0x284b5, 0x005ba}},   {{0x0055e, 0x29057}}, {{0x29057, 0x0055e}},
  {{0x0050c, 0x29bab}}, {{0x29bab, 0x0050c}},   {{0x004c1, 0x2a674}}, {{0x2a674, 0x004c1}},   {{0x004a7, 0x2aa5e}}, {{0x2aa5e, 0x004a7}},   {{0x0046f, 0x2b32f}}, {{0x2b32f, 0x0046f}},
  {{0x0041f, 0x2c0ad}}, {{0x2c0ad, 0x0041f}},   {{0x003e7, 0x2ca8d}}, {{0x2ca8d, 0x003e7}},   {{0x003ba, 0x2d323}}, {{0x2d323, 0x003ba}},   {{0x0010c, 0x3bfbb}}, {{0x3bfbb, 0x0010c}}
};

const uint32_t ProbModelTables::m_EstFracProb[128] =
{
  0x041b5, 0x03df6, 0x04410, 0x03bbc, 0x04636, 0x039a3, 0x048e6, 0x036f6, 0x04bd3, 0x0340c, 0x04e5d, 0x03185, 0x050fa, 0x02eeb, 0x0534b, 0x02c9b,
  0x0557e, 0x02a6a, 0x057b1, 0x02839, 0x059e4, 0x02608, 0x05bba, 0x02433, 0x05d8f, 0x0225e, 0x05f58, 0x02097, 0x060f7, 0x01efa, 0x06288, 0x01d69,
  0x06427, 0x01bcb, 0x06581, 0x01a72, 0x066d4, 0x0191f, 0x0682f, 0x017c6, 0x0693e, 0x016b7, 0x06a80, 0x01576, 0x06b7e, 0x01478, 0x06ca1, 0x01356,
  0x06da2, 0x01255, 0x06e88, 0x01170, 0x06f67, 0x01092, 0x0703e, 0x00fba, 0x07116, 0x00ee3, 0x071b7, 0x00e42, 0x0728f, 0x00d6a, 0x07323, 0x00cd6,
  0x073e9, 0x00c11, 0x0747d, 0x00b7d, 0x0751e, 0x00add, 0x075b2, 0x00a49, 0x0761b, 0x009e0, 0x07697, 0x00964, 0x07719, 0x008e2, 0x07794, 0x00867,
  0x077f1, 0x0080b, 0x0785b, 0x007a1, 0x078c9, 0x00733, 0x07932, 0x006ca, 0x0798f, 0x0066d, 0x079c6, 0x00636, 0x07a23, 0x005da, 0x07a7f, 0x0057d,
  0x07ab7, 0x00546, 0x07afb, 0x00502, 0x07b32, 0x004cb, 0x07b7d, 0x00480, 0x07b9c, 0x00461, 0x07bf8, 0x00405, 0x07c17, 0x003e6, 0x07c55, 0x003a9,
  0x07c8c, 0x00371, 0x07cbf, 0x0033f, 0x07cd0, 0x0032e, 0x07cf6, 0x00308, 0x07d2c, 0x002d1, 0x07d52, 0x002ab, 0x07d71, 0x0028c, 0x07f46, 0x000b5
};

const uint8_t ProbModelTables::m_LPSTable_64_4[64][4] =
{
  { 128, 176, 208, 240 },
  { 128, 167, 197, 227 },
  { 128, 158, 187, 216 },
  { 123, 150, 178, 205 },
  { 116, 142, 169, 195 },
  { 111, 135, 160, 185 },
  { 105, 128, 152, 175 },
  { 100, 122, 144, 166 },
  {  95, 116, 137, 158 },
  {  90, 110, 130, 150 },
  {  85, 104, 123, 142 },
  {  81,  99, 117, 135 },
  {  77,  94, 111, 128 },
  {  73,  89, 105, 122 },
  {  69,  85, 100, 116 },
  {  66,  80,  95, 110 },
  {  62,  76,  90, 104 },
  {  59,  72,  86,  99 },
  {  56,  69,  81,  94 },
  {  53,  65,  77,  89 },
  {  51,  62,  73,  85 },
  {  48,  59,  69,  80 },
  {  46,  56,  66,  76 },
  {  43,  53,  63,  72 },
  {  41,  50,  59,  69 },
  {  39,  48,  56,  65 },
  {  37,  45,  54,  62 },
  {  35,  43,  51,  59 },
  {  33,  41,  48,  56 },
  {  32,  39,  46,  53 },
  {  30,  37,  43,  50 },
  {  29,  35,  41,  48 },
  {  27,  33,  39,  45 },
  {  26,  31,  37,  43 },
  {  24,  30,  35,  41 },
  {  23,  28,  33,  39 },
  {  22,  27,  32,  37 },
  {  21,  26,  30,  35 },
  {  20,  24,  29,  33 },
  {  19,  23,  27,  31 },
  {  18,  22,  26,  30 },
  {  17,  21,  25,  28 },
  {  16,  20,  23,  27 },
  {  15,  19,  22,  25 },
  {  14,  18,  21,  24 },
  {  14,  17,  20,  23 },
  {  13,  16,  19,  22 },
  {  12,  15,  18,  21 },
  {  12,  14,  17,  20 },
  {  11,  14,  16,  19 },
  {  11,  13,  15,  18 },
  {  10,  12,  15,  17 },
  {  10,  12,  14,  16 },
  {   9,  11,  13,  15 },
  {   9,  11,  12,  14 },
  {   8,  10,  12,  14 },
  {   8,   9,  11,  13 },
  {   7,   9,  11,  12 },
  {   7,   9,  10,  12 },
  {   7,   8,  10,  11 },
  {   6,   8,   9,  11 },
  {   6,   7,   9,  10 },
  {   6,   7,   8,   9 },
  {   2,   2,   2,   2 }
};

const uint8_t ProbModelTables::m_RenormTable_32[32] =
{
  6,  5,  4,  4,
  3,  3,  3,  3,
  2,  2,  2,  2,
  2,  2,  2,  2,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1
};





void BinProbModel_Std::init( int qp, int initId )
{
  int slope     = ( ( initId >>  4 )  * 5 ) - 45;
  int offset    = ( ( initId  & 15 ) << 3 ) - 16;
  int inistate  = ( ( slope   * qp ) >> 4 ) + offset;
  if( inistate >= 64 )
  {
    m_State     = ( std::min( 62, inistate - 64 ) << 1 ) + 1;
  }
  else
  {
    m_State     = ( std::min( 62, 63 - inistate ) << 1 );
  }
}




CtxSet::CtxSet( std::initializer_list<CtxSet> ctxSets )
{
  uint16_t  minOffset = std::numeric_limits<uint16_t>::max();
  uint16_t  maxOffset = 0;
  for( auto iter = ctxSets.begin(); iter != ctxSets.end(); iter++ )
  {
    minOffset = std::min<uint16_t>( minOffset, (*iter).Offset              );
    maxOffset = std::max<uint16_t>( maxOffset, (*iter).Offset+(*iter).Size );
  }
  Offset  = minOffset;
  Size    = maxOffset - minOffset;
}





const std::vector<uint8_t>& ContextSetCfg::getInitTable( unsigned initId )
{
  CHECK( initId >= (unsigned)sm_InitTables.size(),
         "Invalid initId (" << initId << "), only " << sm_InitTables.size() << " tables defined." );
  return sm_InitTables[initId];
}


CtxSet ContextSetCfg::addCtxSet( std::initializer_list<std::initializer_list<uint8_t>> initSet2d )
{
  const std::size_t startIdx  = sm_InitTables[0].size();
  const std::size_t numValues = ( *initSet2d.begin() ).size();
        std::size_t setId     = 0;
  for( auto setIter = initSet2d.begin(); setIter != initSet2d.end() && setId < sm_InitTables.size(); setIter++, setId++ )
  {
    const std::initializer_list<uint8_t>& initSet   = *setIter;
    std::vector<uint8_t>&           initTable = sm_InitTables[setId];
    CHECK( initSet.size() != numValues,
           "Number of init values do not match for all sets (" << initSet.size() << " != " << numValues << ")." );
    initTable.resize( startIdx + numValues );
    std::size_t elemId = startIdx;
    for( auto elemIter = ( *setIter ).begin(); elemIter != ( *setIter ).end(); elemIter++, elemId++ )
    {
      initTable[elemId] = *elemIter;
    }
  }
  return CtxSet( (uint16_t)startIdx, (uint16_t)numValues );
}



#define CNU 154 // dummy initialization value for unused context models 'Context model Not Used'
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables( NUMBER_OF_SLICE_TYPES );

const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 137, 125, 127, 107, 138, 140, },
  { 138, 111, 143, 107, 138, 140, },
  { 138, 141, 158, 151, 124, 126, },
#else
  {  107, 139, 126, 107, 139, 126, },
  {  107, 139, 126, 107, 139, 126, },
  {  139, 141, 157, 139, 141, 157, },
#endif
});

const CtxSet ContextSetCfg::BTSplitFlag = ContextSetCfg::addCtxSet
({
  // |-------- 1st bin, 9 ctx for luma + 3 ctx for chroma------| |--2nd bin--| |3rd bin|
#if TRAINED_CABAC_INIT_TABLES
  { 137, 125, 141, 123, 125, 141, 78, 124, 140, CNU, CNU, CNU, 169, 155, 154, 154, },
  { 123, 140, 156, 138, 125, 141, 122, 124, 140, CNU, CNU, CNU, 169, 155, 139, 169, },
  { 139, 141, 157, 139, 155, 142, 153, 125, 141, 154, 154, 154, 154, 154, 154, 140, },
#else
  {  107, 139, 126, 107, 139, 126, 107, 139, 126, 107, 139, 126, 154, 154, 154, 154,},
  {  107, 139, 126, 107, 139, 126, 107, 139, 126, 107, 139, 126, 154, 154, 154, 154,},
  {  139, 141, 157, 139, 141, 157, 139, 141, 157, 139, 141, 157, 154, 154, 154, 154,},
#endif
});

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 183, 185, 186, },
  { 168, 199, 200, },
  { CNU, CNU, CNU, },
#else
  {  197, 185, 201,},
  {  197, 185, 201,},
  {  CNU, CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 125, },
  { 110, },
  { CNU, },
#else
  {  154,},
  {  110,},
  {  CNU,},
#endif
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 167, },
  { 138, },
  { CNU, },
#else
  { 137,},
  { 122,},
  { CNU,},
#endif
});
const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 136, },
  { 167, },
  { CNU, },
#else
  { 151, },
  { CNU, },
  { CNU, },
#endif
  });

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 154, },
  { 154, },
  { CNU, },
#else
  { CNU, },
  { CNU, },
  { CNU, },
#endif
  });

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 213, },
  { 169, },
  { CNU, },
#else
  { 184, },
  { CNU, },
  { CNU, },
#endif
  });
const CtxSet ContextSetCfg::PartSize = ContextSetCfg::addCtxSet
({
  {  154, 139, 154, 154,},
  {  154, 139, 154, 154,},
  {  184, CNU, CNU, CNU,},
});

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 178, },
  { 194, },
  { CNU, },
#else
  {  134,},
  {  149,},
  {  CNU,},
#endif
});

#if JVET_L0283_MULTI_REF_LINE
const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 151, 183, CNU, },
  { 165, 183, CNU, },
  { 122, 184, CNU, },
#else
  { 154, 154, 154 },
  { 154, 154, 154 },
  { CNU, CNU, CNU },
#endif
  });
#endif

const CtxSet ContextSetCfg::IPredMode[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 183, },
    { 154, },
    { 156, },
#else
    { 183 },
    { 154 },
    { 184 },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { CNU, 152, 139, 154, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
    { CNU, 138, 139, 169, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
    { CNU, 109, 139, 154, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
#else
    {  139, 152, 139, 154, 154, 154, 154, 154, 154, 154, 154, 154,  },
    {  139, 152, 139, 154, 154, 154, 154, 154, 154, 154, 154, 154,  },
    {  139,  63, 139, 154, 154, 154, 154, 154, 154, 154, 154, 154,  },
#endif
  }),
};

const CtxSet ContextSetCfg::PdpcFlag = ContextSetCfg::addCtxSet
({
  {  107,},
  {  107,},
  {  139,},
});

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  {  154, 154, 154,},
  {  154, 154, 154,},
  {  154, 154, 154,},
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 111, 110, 95, 78, 193, },
  { 126, 111, 95, 93, 194, },
  { CNU, CNU, CNU, CNU, CNU, },
#else
  {   95,  79,  63,  31,  31,},
  {   95,  79,  63,  31,  31,},
  {  CNU, CNU, CNU, CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 139, 139, },
  { 138, 168, },
  { CNU, CNU, },
#else
  {  153, 153,},
  {  153, 153,},
  {  CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 196, 184, 171, },
  { 181, 169, 185, },
  { CNU, CNU, CNU, },
#else
  {  197, 185, 201,},
  {  197, 185, 201,},
  {  CNU, CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 123, },
  { 138, },
  { CNU, },
#else
  { 92,  },
  { 77,  },
  { CNU, },
#endif
});

#if JVET_L0632_AFFINE_MERGE
const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
( {
#if TRAINED_CABAC_INIT_TABLES
  { 123, 154, 154, 168, CNU, },
  { 109, 154, 139, 168, CNU, },
  { CNU, CNU, CNU, CNU, CNU, },
#else
  { 137, CNU, CNU, CNU, CNU, },
  { 122, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, },
#endif
} );
#endif

#if JVET_L0646_GBI
const CtxSet ContextSetCfg::GBiIdx = ContextSetCfg::addCtxSet
({
  // 4 ctx for 1st bin; 1 ctx for each of rest bins
#if TRAINED_CABAC_INIT_TABLES
  { 199, CNU, CNU, CNU, 124, 169, 127, },
  { 154, CNU, CNU, CNU, 124, 185, 143, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
#else
  { 95,  79,  63,  31,  31,  31,  31, },
  { 95,  79,  63,  31,  31,  31,  31, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
#endif
  });
#endif

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 169, 183, },
  { 155, 198, },
  { CNU, CNU, },
#else
  {  169, 198,},
  {  140, 198,},
  {  CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::TransSubdivFlag = ContextSetCfg::addCtxSet
({
  {  224, 167, 122, 122, 122},
  {  124, 138,  94,  94,  94},
  {  153, 138, 138, 138, 138},
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 94, },
  { 95, },
  { CNU, },
#else
  {   79,},
  {   79,},
  {  CNU,},
#endif
});

const CtxSet ContextSetCfg::QtCbf[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 140, 141, },
    { 155, 127, },
    { CNU, 126, },
#else
    {  153, 111,  },
    {  153, 111,  },
    {  111, 141,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 149, 168, CNU, CNU, CNU, },
    { 164, 154, CNU, CNU, CNU, },
    { 109, CNU, CNU, CNU, CNU, },
#else
    {  149,  92, 167, 154, 154,  },
    {  149, 107, 167, 154, 154,  },
    {   94, 138, 182, 154, 154,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 192, 153, },
    { 178, 139, },
    { 122, 140, },
#else
    { 149, 149, },
    { 149, 149, },
    {  94,  94, },
#endif
  }),
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 106, 170, },
    { 121, 141, },
    { 107, 158, },
#else
    {  121, 140,  },
    {  121, 140,  },
    {   91, 171,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 91, 140, },
    { 105, 155, },
    { 105, 126, },
#else
    {   61, 154,  },
    {   61, 154,  },
    {  134, 141,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
    {  122, 143,  },
    {   78, 111,  },
    {  135, 155,  },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   91, 141,  },
    {   60, 140,  },
    {  104, 139,  },
  }),
};

const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 105, 152, 167, 153, 168, 169, 104, 167, 182, 183, 183, 170, 209, 213, 183, 183, 169, 185, },
    { 119, 152, 167, 168, 183, 140, 134, 182, 168, 183, 169, 185, 166, 228, 183, 198, 184, 156, },
    { 105, 138, 153, 154, 125, 111, 105, 139, 154, 155, 155, 127, 137, 185, 169, 185, 171, 159, },
#else
    {  120, 152, 167, 153, 168, 169, 119, 167, 197, 183, 183, 170, 209, 213, 183, 183, 169, 185, },
    {  149, 152, 167, 168, 183, 140, 149, 182, 168, 183, 169, 170, 195, 213, 183, 198, 184, 156, },
    {  120, 138, 153, 154, 140, 126, 120, 139, 154, 155, 155, 142, 137, 185, 169, 185, 171, 159, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 148, 167, 153, 168, 154, 140, 166, 199, 183, 199, 199, 172, },
    { 134, 168, 168, 169, 169, 170, 196, 244, 184, 244, 200, 172, },
    { 104, 168, 168, 169, 140, 141, 167, 215, 155, 172, 171, 158, },
#else
    {  148, 167, 153, 139, 154, 140, 166, 199, 183, 184, 184, 157,  },
    {  134, 168, 168, 139, 169, 155, 166, 229, 198, 229, 185, 157,  },
    {  119, 168, 153, 140, 140, 141, 167, 200, 155, 172, 142, 158,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 152, 127, 173, 201, 187, 173, 197, 203, 188, 217, 188, 189, 182, 223, 223, 223, 223, 223, },
    { 123, 142, 202, 172, 172, 203, 138, 188, 233, 203, 203, 191, 139, 223, 223, 223, 223, 223, },
    { 108, 157, 158, 158, 218, 189, 123, 191, 159, 190, 205, 236, 79, 223, 253, 223, 223, 253, },
#else
    {  152, 127, 173, 201, 187, 173, 226, 188, 188, 217, 188, 174, 182, 223, 223, 223, 223, 223, },
    {  123, 142, 202, 172, 157, 203, 138, 173, 218, 188, 173, 175, 168, 223, 223, 223, 223, 223, },
    {  108, 157, 173, 173, 218, 189, 123, 175, 159, 175, 190, 251,  79, 223, 223, 223, 223, 223, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 182, 171, 143, 158, 172, 202, 168, 223, 223, 223, 223, 223, },
    { 168, 156, 173, 201, 157, 203, 198, 223, 223, 223, 223, 223, },
    { 152, 173, 157, 187, 189, 251, 170, 223, 223, 253, 223, 223, },
#else
    {  196, 156, 143, 158, 172, 216, 168, 223, 223, 223, 191, 223,  },
    {  182, 141, 158, 186, 142, 173, 183, 223, 223, 223, 222, 223,  },
    {  152, 158, 157, 187, 204, 175, 170, 223, 223, 237, 223, 223,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 137, 142, 190, 188, 202, 189, 241, 191, 191, 189, 189, 190, 195, 223, 223, 223, 223, 223, },
    { 123, 187, 191, 173, 173, 248, 138, 191, 191, 191, 203, 191, 196, 223, 223, 223, 223, 223, },
    { 107, 143, 205, 188, 233, 205, 63, 251, 191, 253, 206, 252, 62, 223, 223, 223, 223, 223, },
#else
    {  137, 142, 189, 173, 187, 174, 241, 175, 175, 174, 174, 204, 210, 223, 223, 223, 223, 223, },
    {  123, 172, 175, 158, 158, 233, 138, 175, 190, 175, 188, 175, 196, 223, 223, 223, 223, 223, },
    {  107, 143, 219, 188, 233, 190,  63, 250, 205, 252, 220, 251,  63, 223, 223, 223, 223, 253, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 167, 200, 175, 188, 174, 175, 196, 223, 223, 223, 223, 223, },
    { 167, 156, 237, 158, 188, 205, 182, 223, 223, 223, 223, 223, },
    { 166, 174, 159, 247, 188, 189, 168, 223, 223, 223, 238, 223, },
#else
    {  167, 185, 159, 158, 159, 189, 196, 223, 223, 223, 223, 223,  },
    {  167, 141, 175, 143, 172, 159, 182, 223, 223, 223, 223, 223,  },
    {  166, 159, 158, 232, 158, 174, 183, 238, 223, 223, 223, 223,  },
#endif
  }),
};


const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 91, 104, 136, 152, 153, 153, 105, 137, 167, 153, 168, 121, 167, 153, 168, 139, 151, 153, 139, 168, 154, },
    { 106, 134, 151, 152, 138, 168, 120, 137, 138, 153, 139, 136, 138, 153, 168, 139, 137, 153, 168, 139, 139, },
    { 121, 135, 137, 138, 153, 153, 136, 123, 138, 153, 139, 152, 153, 153, 139, 139, 138, 168, 139, 154, 139, },
#else
    {  105, 119, 151, 152, 153, 153, 135, 152, 182, 153, 168, 136, 182, 153, 168, 139, 166, 168, 139, 168, 154,  },
    {  120, 119, 151, 167, 138, 168, 135, 152, 153, 153, 139, 136, 153, 153, 168, 139, 137, 168, 168, 139, 139,  },
    {  135, 150, 152, 138, 153, 153, 151, 123, 153, 168, 139, 152, 153, 153, 139, 139, 138, 168, 139, 154, 139,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 135, 135, 152, 138, 153, 124, 151, 168, 169, 153, 139, },
    { 120, 150, 152, 153, 153, 153, 166, 168, 168, 139, 154, },
    { 136, 121, 167, 168, 138, 153, 137, 139, 154, 139, 154, },
#else
    {  105, 135, 152, 167, 153, 124, 151, 168, 169, 153, 124,  },
    {  134, 150, 152, 153, 153, 153, 166, 168, 168, 139, 139,  },
    {  135, 121, 167, 168, 138, 153, 167, 139, 154, 139, 154,  },
#endif
  }),
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 30, 0, 102, 104, 106, 152, 57, 44, 120, 136, 123, 87, 134, 151, 152, 153, 89, 121, 152, 153, 125, },
    { 88, 0, 102, 149, 150, 152, 101, 103, 150, 151, 138, 102, 105, 122, 167, 153, 90, 107, 123, 153, 154, },
    { 90, 41, 149, 121, 122, 123, 58, 105, 92, 108, 109, 104, 92, 123, 109, 124, 151, 138, 139, 154, 140, },
#else
    {  73,   0,  58, 119, 150, 137,  42,  73, 120, 136, 123,  58, 149, 151, 152, 153, 134, 136, 152, 153, 125,  },
    {  88,   0, 102, 104, 150, 122, 101,  89, 150, 151, 138,  88, 120, 122, 152, 153, 105, 107, 123, 153, 154,  },
    { 134, 161, 149, 121, 122, 138,  88, 120, 107, 108, 109, 105, 107, 123, 109, 124, 151, 138, 139, 154, 140,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 102, 101, 90, 107, 122, 93, 118, 121, 153, 125, 140, },
    { 0, 0, 105, 151, 107, 93, 103, 136, 138, 154, 125, },
    { 165, 11, 120, 122, 137, 138, 75, 106, 138, 154, 155, },
#else
    {  87,  57,  90, 107, 107,  63, 119,  91, 152, 124, 140,  },
    { 101,   0, 105, 121, 107,  93, 118, 106, 108, 124, 154,  },
    { 179,  72,  90, 121, 122, 123,  75,  76, 123, 139, 170,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 89, 132, 151, 138, 124, 125, 119, 152, 153, 154, 140, 135, 153, 139, 169, 155, 151, 168, 169, 170, 171, },
    { 118, 101, 137, 138, 139, 140, 149, 138, 139, 154, 155, 136, 153, 154, 140, 170, 152, 139, 140, 155, 186, },
    { 135, 120, 108, 153, 139, 140, 151, 153, 139, 125, 140, 123, 154, 140, 155, 126, 139, 140, 170, 156, 142, },
#else
    {  89, 103, 121, 137, 138, 139, 119, 137, 138, 139, 125, 135, 167, 168, 154, 140, 136, 153, 183, 155, 185,  },
    { 118,   0, 136, 152, 153, 154, 134, 152, 153, 139, 140, 150, 138, 139, 154, 155, 151, 153, 169, 140, 200,  },
    { 164, 149, 137, 153, 124, 125, 151, 138, 139, 125, 125, 152, 139, 140, 140, 111, 153, 154, 155, 170, 127,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 102, 164, 138, 139, 154, 140, 181, 155, 171, 157, 143, },
    { 132, 136, 153, 154, 140, 155, 167, 155, 156, 142, 173, },
    { 165, 151, 153, 154, 125, 126, 168, 155, 186, 172, 143, },
#else
    {  27, 149, 137, 153, 139, 125, 151, 154, 170, 127, 127,  },
    { 132, 135, 152, 139, 139, 125, 151, 154, 155, 141, 142,  },
    { 165, 121, 138, 139, 139, 125, 138, 154, 156, 171, 127,  },
#endif
  }),
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 111, 125, 124, 111, 111, 109, 111, 111, 125, 109, 140, 126, 111, 111, 139, 140, 111, 125, 95, 138, CNU, CNU, CNU, CNU, CNU, },
    { 125, 110, 109, 111, 125, 123, 111, 111, 95, 123, 140, 126, 125, 95, 169, 125, 140, 110, 124, 152, CNU, CNU, CNU, CNU, CNU, },
    { 140, 140, 124, 140, 126, 109, 140, 141, 125, 94, 111, 127, 111, 140, 93, 141, 186, 141, 125, 197, CNU, CNU, CNU, CNU, CNU, },
#else
    {  125, 110, 124, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79, 126, 111, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  125, 110,  94, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94, 111, 111,  95,  94, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79, 143, 127, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 123, 109, 63, CNU, },
    { 138, 123, 92, CNU, },
    { 123, 108, 62, CNU, },
#else
    {  108, 123,  93, 154,  },
    {  108, 123, 108, 154,  },
    {  108, 123,  63, 154,  },
#endif
  }),
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 125, 110, 139, 125, 125, 109, 111, 111, 110, 109, 140, 126, 110, 110, 154, 140, 111, 125, 109, 181, CNU, CNU, CNU, CNU, CNU, },
    { 110, 95, 94, 125, 110, 123, 140, 111, 95, 123, 125, 111, 110, 95, 154, 125, 111, 95, 94, 137, CNU, CNU, CNU, CNU, CNU, },
    { 110, 110, 109, 125, 111, 123, 111, 141, 95, 108, 111, 142, 111, 95, 63, 140, 157, 141, 110, 152, CNU, CNU, CNU, CNU, CNU, },
#else
    {  125, 110, 124, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79, 126, 111, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  125, 110,  94, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94, 111, 111,  95,  94, CNU, CNU, CNU, CNU, CNU, CNU,  },
    {  110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79, 143, 127, 111,  79, CNU, CNU, CNU, CNU, CNU, CNU,  },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if TRAINED_CABAC_INIT_TABLES
    { 108, 94, 122, CNU, },
    { 108, 93, 92, CNU, },
    { 108, 123, 77, CNU, },
#else
    {  108, 123,  93, 154,  },
    {  108, 123, 108, 154,  },
    {  108, 123,  63, 154,  },
#endif
  }),
};


const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 168, },
  { 168, },
  { CNU, },
#else
  {  168,},
  {  168,},
  {  CNU,},
#endif
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 92, },
  { 214, },
  { 184, },
#else
  {  153,},
  {  153,},
  {  153,},
#endif
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 77, },
  { 111, },
  { 110, },
#else
  {  160,},
  {  185,},
  {  200,},
#endif
});

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 124, 61, },
  { 138, 46, },
  { 109, 42, },
#else
  {  139, 139,},
  {  139, 139,},
  {  139, 139,},
#endif
});

const CtxSet ContextSetCfg::TransquantBypassFlag = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
});

const CtxSet ContextSetCfg::RdpcmFlag = ContextSetCfg::addCtxSet
({
  {  139, 139,},
  {  139, 139,},
  {  CNU, CNU,},
});

const CtxSet ContextSetCfg::RdpcmDir = ContextSetCfg::addCtxSet
({
  {  139, 139,},
  {  139, 139,},
  {  CNU, CNU,},
});

const CtxSet ContextSetCfg::EMTTuIndex = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 153, 138, CNU, CNU, },
  { 138, 167, CNU, CNU, },
  { 167, 123, CNU, CNU, },
#else
  {  CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::EMTCuFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 155, 141, 155, 155, 140, CNU, },
  { 141, 141, 141, 126, 155, CNU, },
  { CNU, CNU, 140, 155, 155, CNU, },
#else
  {  CNU, CNU, CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU, CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::CrossCompPred = ContextSetCfg::addCtxSet
({
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 212, 214, 230, 182, },
  { 212, 214, 230, 182, },
  { CNU, CNU, CNU, CNU, },
#else
  {  197, 185, 201, 185,},
  {  197, 185, 201, 185,},
  {  CNU, CNU, CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::ctbAlfFlag =
{
  ContextSetCfg::addCtxSet
  ( {
#if TRAINED_CABAC_INIT_TABLES
    { 138, 141, 173, 122, 170, 203, 151, 170, 203, },
    { 153, 156, 188, 137, 185, 218, 152, 185, 218, },
    { 155, 205, 253, 168, 187, 234, 168, 187, 220, },
#else
    { 100, 153, 200, 100, 153, 200, 100, 153, 200 },
    { 100, 153, 200, 100, 153, 200, 100, 153, 200 },
    { 100, 153, 200, 100, 153, 200, 100, 153, 200 },
#endif
    } )
};

const CtxSet ContextSetCfg::MHIntraFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 226, },
  { 227, },
  { CNU, },
#else
  { 154, },
  { 110, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::MHIntraPredMode = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 155, CNU, CNU, CNU, },
  { 141, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, },
#else
  { 183, CNU, CNU, CNU, },
  { 154, CNU, CNU, CNU, },
  { 184, CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::TriangleFlag = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 165, 137, 153, },
  { 106, 122, 138, },
  { CNU, CNU, CNU, },
#else
  { 151, 137, 154, },
  { 151, 137, 154, },
  { CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::TriangleIdx = ContextSetCfg::addCtxSet
({
#if TRAINED_CABAC_INIT_TABLES
  { 155, },
  { 126, },
  { CNU, },
#else
  { 140, },
  { 140, },
  { CNU, },
#endif
});

const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();


// combined sets
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };



template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore()
  : m_CtxBuffer ()
  , m_Ctx       ( nullptr )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( bool dummy )
  : m_CtxBuffer ( ContextSetCfg::NumberOfContexts )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( const CtxStore<BinProbModel>& ctxStore )
  : m_CtxBuffer ( ctxStore.m_CtxBuffer )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

template <class BinProbModel>
void CtxStore<BinProbModel>::init( int qp, int initId )
{
  const std::vector<uint8_t>& initTable = ContextSetCfg::getInitTable( initId );
  CHECK( m_CtxBuffer.size() != initTable.size(),
        "Size of init table (" << initTable.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  int clippedQP = Clip3( 0, MAX_QP, qp );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].init( clippedQP, initTable[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::setWinSizes( const std::vector<uint8_t>& log2WindowSizes )
{
  CHECK( m_CtxBuffer.size() != log2WindowSizes.size(),
        "Size of window size table (" << log2WindowSizes.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setLog2WindowSize( log2WindowSizes[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadPStates( const std::vector<uint16_t>& probStates )
{
  CHECK( m_CtxBuffer.size() != probStates.size(),
        "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setState( probStates[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::savePStates( std::vector<uint16_t>& probStates ) const
{
  probStates.resize( m_CtxBuffer.size(), uint16_t(0) );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    probStates[k] = m_CtxBuffer[k].getState();
  }
}





template class CtxStore<BinProbModel_Std>;





Ctx::Ctx()                                  : m_BPMType( BPM_Undefined )                        {}
Ctx::Ctx( const BinProbModel_Std*   dummy ) : m_BPMType( BPM_Std   ), m_CtxStore_Std  ( true )  {}

Ctx::Ctx( const Ctx& ctx )
  : m_BPMType         ( ctx.m_BPMType )
  , m_CtxStore_Std    ( ctx.m_CtxStore_Std    )
{
  ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
}

