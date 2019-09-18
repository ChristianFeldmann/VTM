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

/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>
#include <limits>


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

const BinFracBits ProbModelTables::m_binFracBits[256] = {
  { { 0x0005c, 0x48000 } }, { { 0x00116, 0x3b520 } }, { { 0x001d0, 0x356cb } }, { { 0x0028b, 0x318a9 } },
  { { 0x00346, 0x2ea40 } }, { { 0x00403, 0x2c531 } }, { { 0x004c0, 0x2a658 } }, { { 0x0057e, 0x28beb } },
  { { 0x0063c, 0x274ce } }, { { 0x006fc, 0x26044 } }, { { 0x007bc, 0x24dc9 } }, { { 0x0087d, 0x23cfc } },
  { { 0x0093f, 0x22d96 } }, { { 0x00a01, 0x21f60 } }, { { 0x00ac4, 0x2122e } }, { { 0x00b89, 0x205dd } },
  { { 0x00c4e, 0x1fa51 } }, { { 0x00d13, 0x1ef74 } }, { { 0x00dda, 0x1e531 } }, { { 0x00ea2, 0x1db78 } },
  { { 0x00f6a, 0x1d23c } }, { { 0x01033, 0x1c970 } }, { { 0x010fd, 0x1c10b } }, { { 0x011c8, 0x1b903 } },
  { { 0x01294, 0x1b151 } }, { { 0x01360, 0x1a9ee } }, { { 0x0142e, 0x1a2d4 } }, { { 0x014fc, 0x19bfc } },
  { { 0x015cc, 0x19564 } }, { { 0x0169c, 0x18f06 } }, { { 0x0176d, 0x188de } }, { { 0x0183f, 0x182e8 } },
  { { 0x01912, 0x17d23 } }, { { 0x019e6, 0x1778a } }, { { 0x01abb, 0x1721c } }, { { 0x01b91, 0x16cd5 } },
  { { 0x01c68, 0x167b4 } }, { { 0x01d40, 0x162b6 } }, { { 0x01e19, 0x15dda } }, { { 0x01ef3, 0x1591e } },
  { { 0x01fcd, 0x15480 } }, { { 0x020a9, 0x14fff } }, { { 0x02186, 0x14b99 } }, { { 0x02264, 0x1474e } },
  { { 0x02343, 0x1431b } }, { { 0x02423, 0x13f01 } }, { { 0x02504, 0x13afd } }, { { 0x025e6, 0x1370f } },
  { { 0x026ca, 0x13336 } }, { { 0x027ae, 0x12f71 } }, { { 0x02894, 0x12bc0 } }, { { 0x0297a, 0x12821 } },
  { { 0x02a62, 0x12494 } }, { { 0x02b4b, 0x12118 } }, { { 0x02c35, 0x11dac } }, { { 0x02d20, 0x11a51 } },
  { { 0x02e0c, 0x11704 } }, { { 0x02efa, 0x113c7 } }, { { 0x02fe9, 0x11098 } }, { { 0x030d9, 0x10d77 } },
  { { 0x031ca, 0x10a63 } }, { { 0x032bc, 0x1075c } }, { { 0x033b0, 0x10461 } }, { { 0x034a5, 0x10173 } },
  { { 0x0359b, 0x0fe90 } }, { { 0x03693, 0x0fbb9 } }, { { 0x0378c, 0x0f8ed } }, { { 0x03886, 0x0f62b } },
  { { 0x03981, 0x0f374 } }, { { 0x03a7e, 0x0f0c7 } }, { { 0x03b7c, 0x0ee23 } }, { { 0x03c7c, 0x0eb89 } },
  { { 0x03d7d, 0x0e8f9 } }, { { 0x03e7f, 0x0e671 } }, { { 0x03f83, 0x0e3f2 } }, { { 0x04088, 0x0e17c } },
  { { 0x0418e, 0x0df0e } }, { { 0x04297, 0x0dca8 } }, { { 0x043a0, 0x0da4a } }, { { 0x044ab, 0x0d7f3 } },
  { { 0x045b8, 0x0d5a5 } }, { { 0x046c6, 0x0d35d } }, { { 0x047d6, 0x0d11c } }, { { 0x048e7, 0x0cee3 } },
  { { 0x049fa, 0x0ccb0 } }, { { 0x04b0e, 0x0ca84 } }, { { 0x04c24, 0x0c85e } }, { { 0x04d3c, 0x0c63f } },
  { { 0x04e55, 0x0c426 } }, { { 0x04f71, 0x0c212 } }, { { 0x0508d, 0x0c005 } }, { { 0x051ac, 0x0bdfe } },
  { { 0x052cc, 0x0bbfc } }, { { 0x053ee, 0x0b9ff } }, { { 0x05512, 0x0b808 } }, { { 0x05638, 0x0b617 } },
  { { 0x0575f, 0x0b42a } }, { { 0x05888, 0x0b243 } }, { { 0x059b4, 0x0b061 } }, { { 0x05ae1, 0x0ae83 } },
  { { 0x05c10, 0x0acaa } }, { { 0x05d41, 0x0aad6 } }, { { 0x05e74, 0x0a907 } }, { { 0x05fa9, 0x0a73c } },
  { { 0x060e0, 0x0a575 } }, { { 0x06219, 0x0a3b3 } }, { { 0x06354, 0x0a1f5 } }, { { 0x06491, 0x0a03b } },
  { { 0x065d1, 0x09e85 } }, { { 0x06712, 0x09cd4 } }, { { 0x06856, 0x09b26 } }, { { 0x0699c, 0x0997c } },
  { { 0x06ae4, 0x097d6 } }, { { 0x06c2f, 0x09634 } }, { { 0x06d7c, 0x09495 } }, { { 0x06ecb, 0x092fa } },
  { { 0x0701d, 0x09162 } }, { { 0x07171, 0x08fce } }, { { 0x072c7, 0x08e3e } }, { { 0x07421, 0x08cb0 } },
  { { 0x0757c, 0x08b26 } }, { { 0x076da, 0x089a0 } }, { { 0x0783b, 0x0881c } }, { { 0x0799f, 0x0869c } },
  { { 0x07b05, 0x0851f } }, { { 0x07c6e, 0x083a4 } }, { { 0x07dd9, 0x0822d } }, { { 0x07f48, 0x080b9 } },
  { { 0x080b9, 0x07f48 } }, { { 0x0822d, 0x07dd9 } }, { { 0x083a4, 0x07c6e } }, { { 0x0851f, 0x07b05 } },
  { { 0x0869c, 0x0799f } }, { { 0x0881c, 0x0783b } }, { { 0x089a0, 0x076da } }, { { 0x08b26, 0x0757c } },
  { { 0x08cb0, 0x07421 } }, { { 0x08e3e, 0x072c7 } }, { { 0x08fce, 0x07171 } }, { { 0x09162, 0x0701d } },
  { { 0x092fa, 0x06ecb } }, { { 0x09495, 0x06d7c } }, { { 0x09634, 0x06c2f } }, { { 0x097d6, 0x06ae4 } },
  { { 0x0997c, 0x0699c } }, { { 0x09b26, 0x06856 } }, { { 0x09cd4, 0x06712 } }, { { 0x09e85, 0x065d1 } },
  { { 0x0a03b, 0x06491 } }, { { 0x0a1f5, 0x06354 } }, { { 0x0a3b3, 0x06219 } }, { { 0x0a575, 0x060e0 } },
  { { 0x0a73c, 0x05fa9 } }, { { 0x0a907, 0x05e74 } }, { { 0x0aad6, 0x05d41 } }, { { 0x0acaa, 0x05c10 } },
  { { 0x0ae83, 0x05ae1 } }, { { 0x0b061, 0x059b4 } }, { { 0x0b243, 0x05888 } }, { { 0x0b42a, 0x0575f } },
  { { 0x0b617, 0x05638 } }, { { 0x0b808, 0x05512 } }, { { 0x0b9ff, 0x053ee } }, { { 0x0bbfc, 0x052cc } },
  { { 0x0bdfe, 0x051ac } }, { { 0x0c005, 0x0508d } }, { { 0x0c212, 0x04f71 } }, { { 0x0c426, 0x04e55 } },
  { { 0x0c63f, 0x04d3c } }, { { 0x0c85e, 0x04c24 } }, { { 0x0ca84, 0x04b0e } }, { { 0x0ccb0, 0x049fa } },
  { { 0x0cee3, 0x048e7 } }, { { 0x0d11c, 0x047d6 } }, { { 0x0d35d, 0x046c6 } }, { { 0x0d5a5, 0x045b8 } },
  { { 0x0d7f3, 0x044ab } }, { { 0x0da4a, 0x043a0 } }, { { 0x0dca8, 0x04297 } }, { { 0x0df0e, 0x0418e } },
  { { 0x0e17c, 0x04088 } }, { { 0x0e3f2, 0x03f83 } }, { { 0x0e671, 0x03e7f } }, { { 0x0e8f9, 0x03d7d } },
  { { 0x0eb89, 0x03c7c } }, { { 0x0ee23, 0x03b7c } }, { { 0x0f0c7, 0x03a7e } }, { { 0x0f374, 0x03981 } },
  { { 0x0f62b, 0x03886 } }, { { 0x0f8ed, 0x0378c } }, { { 0x0fbb9, 0x03693 } }, { { 0x0fe90, 0x0359b } },
  { { 0x10173, 0x034a5 } }, { { 0x10461, 0x033b0 } }, { { 0x1075c, 0x032bc } }, { { 0x10a63, 0x031ca } },
  { { 0x10d77, 0x030d9 } }, { { 0x11098, 0x02fe9 } }, { { 0x113c7, 0x02efa } }, { { 0x11704, 0x02e0c } },
  { { 0x11a51, 0x02d20 } }, { { 0x11dac, 0x02c35 } }, { { 0x12118, 0x02b4b } }, { { 0x12494, 0x02a62 } },
  { { 0x12821, 0x0297a } }, { { 0x12bc0, 0x02894 } }, { { 0x12f71, 0x027ae } }, { { 0x13336, 0x026ca } },
  { { 0x1370f, 0x025e6 } }, { { 0x13afd, 0x02504 } }, { { 0x13f01, 0x02423 } }, { { 0x1431b, 0x02343 } },
  { { 0x1474e, 0x02264 } }, { { 0x14b99, 0x02186 } }, { { 0x14fff, 0x020a9 } }, { { 0x15480, 0x01fcd } },
  { { 0x1591e, 0x01ef3 } }, { { 0x15dda, 0x01e19 } }, { { 0x162b6, 0x01d40 } }, { { 0x167b4, 0x01c68 } },
  { { 0x16cd5, 0x01b91 } }, { { 0x1721c, 0x01abb } }, { { 0x1778a, 0x019e6 } }, { { 0x17d23, 0x01912 } },
  { { 0x182e8, 0x0183f } }, { { 0x188de, 0x0176d } }, { { 0x18f06, 0x0169c } }, { { 0x19564, 0x015cc } },
  { { 0x19bfc, 0x014fc } }, { { 0x1a2d4, 0x0142e } }, { { 0x1a9ee, 0x01360 } }, { { 0x1b151, 0x01294 } },
  { { 0x1b903, 0x011c8 } }, { { 0x1c10b, 0x010fd } }, { { 0x1c970, 0x01033 } }, { { 0x1d23c, 0x00f6a } },
  { { 0x1db78, 0x00ea2 } }, { { 0x1e531, 0x00dda } }, { { 0x1ef74, 0x00d13 } }, { { 0x1fa51, 0x00c4e } },
  { { 0x205dd, 0x00b89 } }, { { 0x2122e, 0x00ac4 } }, { { 0x21f60, 0x00a01 } }, { { 0x22d96, 0x0093f } },
  { { 0x23cfc, 0x0087d } }, { { 0x24dc9, 0x007bc } }, { { 0x26044, 0x006fc } }, { { 0x274ce, 0x0063c } },
  { { 0x28beb, 0x0057e } }, { { 0x2a658, 0x004c0 } }, { { 0x2c531, 0x00403 } }, { { 0x2ea40, 0x00346 } },
  { { 0x318a9, 0x0028b } }, { { 0x356cb, 0x001d0 } }, { { 0x3b520, 0x00116 } }, { { 0x48000, 0x0005c } },
};
#if !JVET_O0065_CABAC_INIT
const uint16_t ProbModelTables::m_inistateToCount[128] = {
  614,   647,   681,   718,   756,   797,   839,   884,   932,   982,   1034,  1089,  1148,  1209,  1274,  1342,
  1414,  1490,  1569,  1653,  1742,  1835,  1933,  2037,  2146,  2261,  2382,  2509,  2643,  2785,  2934,  3091,
  3256,  3430,  3614,  3807,  4011,  4225,  4452,  4690,  4941,  5205,  5483,  5777,  6086,  6412,  6755,  7116,
  7497,  7898,  8320,  8766,  9235,  9729,  10249, 10798, 11375, 11984, 12625, 13300, 14012, 14762, 15551, 16384,
  16384, 17216, 18005, 18755, 19467, 20142, 20783, 21392, 21969, 22518, 23038, 23532, 24001, 24447, 24869, 25270,
  25651, 26012, 26355, 26681, 26990, 27284, 27562, 27826, 28077, 28315, 28542, 28756, 28960, 29153, 29337, 29511,
  29676, 29833, 29982, 30124, 30258, 30385, 30506, 30621, 30730, 30834, 30932, 31025, 31114, 31198, 31277, 31353,
  31425, 31493, 31558, 31619, 31678, 31733, 31785, 31835, 31883, 31928, 31970, 32011, 32049, 32086, 32120, 32153
};
#endif
void BinProbModel_Std::init( int qp, int initId )
{
#if JVET_O0065_CABAC_INIT
  int slope = (initId >> 3) - 4;
  int offset = ((initId & 7) * 18) + 1;
  int inistate = ((slope   * (qp - 16)) >> 1) + offset;
  int state_clip = inistate < 1 ? 1 : inistate > 127 ? 127 : inistate;
  const int p1 = (state_clip << 8);
#else
  int slope     = ( ( initId >>  4 )  * 5 ) - 45;
  int offset    = ( ( initId  & 15 ) << 3 ) - 16;
  int inistate  = ( ( slope   * qp ) >> 4 ) + offset;
  const int p1 = m_inistateToCount[inistate < 0 ? 0 : inistate > 127 ? 127 : inistate];
#endif
  m_state[0]   = p1 & MASK_0;
  m_state[1]   = p1 & MASK_1;
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


#if JVET_O0065_CABAC_INIT
#define CNU 35
#else
#define CNU 154 // dummy initialization value for unused context models 'Context model Not Used'
#endif
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables(NUMBER_OF_SLICE_TYPES + 1);

// clang-format off
#if JVET_O0065_CABAC_INIT
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
  {  18,  27,  15,  11,  28,  30,  19,  22,  23, },
  {  18,  27,  53,  12,   6,  30,  13,  15,  31, },
  {  19,  28,  38,  12,  29,  38,  28,  38,  31, },
  {  12,  13,   8,   8,  13,  12,   5,  10,   9, },
});

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
  {  26,  36,  38,  33,  34,  21, },
  {  20,   7,  23,  18,  19,   6, },
  {  12,   6,  15,  33,  27,  22, },
  {   0,   8,   8,  12,  12,  12, },
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
  {  43,  42,  37,  35,  44, },
  {  36,  35,  37,  27,  52, },
  {  43,  42,  29,  27,  44, },
  {   9,   9,   9,   8,   8, },
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
  {  28,  29,  28,  29, },
  {  36,  37,  28,  22, },
  {  51,  37,  51,  37, },
  {  12,  12,  12,  13, },
});

#if JVET_O0050_LOCAL_DUAL_TREE
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
({
  {  40,  28, },
  {  25,  12, },
  { CNU, CNU, },
  {   1,   0, },
});
#endif

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
  {  57,  60,  53, },
  {  57,  59,  45, },
  {   0,  34,  36, },
  {   5,   4,   8, },
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
  {   6, },
  {   6, },
  {  19, },
  {   5, },
});

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
({
  {  31,  15, },
  {  38,   7, },
  { CNU, CNU, },
  {   5,   5, },
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
  {  41, },
  {  43, },
  {  34, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
  {  48, },
  {  26, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
  {  43, },
  {  43, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  51, },
  {  60, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
  {  40,  50, },
  {  40,  35, },
  { CNU, CNU, },
  {   5,   2, },
});

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
  {  25,  50, },
  {  25,  57, },
  {  25,  51, },
  {   6,   8, },
});

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
({
  {  36, },
  {  36, },
  {  45, },
  {   6, },
});

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
({
  {  13,  21, },
  {  12,  13, },
  {  13,  28, },
  {   4,   5, },
});

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
({
  {  19, },
  {  42, },
  {  59, },
  {   4, },
});

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
  {  25, },
  {  33, },
  {  19, },
  {   6, },
});

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
({
  {  41,  49,  50,  26, },
  {  41,  57,  58,  26, },
  {  33,  41,  42,  25, },
  {   9,  10,  10,   5, },
});

#if !JVET_O0925_MIP_SIMPLIFICATIONS
const CtxSet ContextSetCfg::MipMode = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
  {   6,  13,   5,   4,  25, },
  {   7,   6,   5,   4,  33, },
  { CNU, CNU, CNU, CNU, CNU, },
  {   0,   0,   1,   4,   0, },
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
  {  13,  20, },
  {  27,  35, },
  { CNU, CNU, },
  {   0,   4, },
});

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
({
  {  56,  59,  60, },
  {  56,  50,  37, },
  { CNU, CNU, CNU, },
  {   4,   4,   4, },
});

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
  {  27,  28,  29, },
  {  12,  20,   6, },
  { CNU, CNU, CNU, },
  {   4,   1,   0, },
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
  {  35, },
  {  35, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
  {   4, },
  {   5, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::GBiIdx = ContextSetCfg::addCtxSet
({
  {  20, },
  {   5, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
  {  51,  58, },
  {  44,  43, },
  {  14,  45, },
  {   9,   5, },
});

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
  {  12, },
  {   5, },
  {   6, },
  {   4, },
});

const CtxSet ContextSetCfg::QtCbf[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  15, CNU,   5,  14, },
    {  15, CNU,  20,   7, },
    {   7, CNU,   5,   7, },
    {   5, DWS,   8,   8, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25, },
    {  25, },
    {  12, },
    {   5, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  44, },
    {  25,  29, },
    {  33,  21, },
    {   2,   1, },
  })
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,  37, },
    {  25,  30, },
    {  18,  31, },
    {   8,   5, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,  37, },
    {  25,  52, },
    {  25,   7, },
    {   5,   8, },
  })
};

const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  17,  41,  49,  51,   1,  49,  50,  37,  48,  51,  58,  45, },
    {  17,  41,  42,  29,  25,  49,  43,  37,  33,  51,  51,  30, },
    {  25,  19,  28,  14,  25,  20,  29,  30,  19,  52,  30,  38, },
    {  12,   9,   9,  10,   9,   9,   9,  10,   8,   8,   8,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  49,  42,  21,  48,  59,  59,  53, },
    {  17,  19,  20,  29,  41,  59,  60,  38, },
    {  25,  27,  28,  37,  49,  53,  53,  46, },
    {   9,   9,   9,  13,   5,   5,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  26,  45,  53,  46,  49,  54,  61,  39,  42,  39,  39,  39, },
    {  19,  38,  38,  46,  34,  54,  54,  39,   6,  39,  39,  39, },
    {  11,  38,  46,  54,  27,  39,  39,  39,  28,  39,  39,  39, },
    {   9,  12,   8,   8,   8,   8,   8,   5,   8,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  45,  38,  31,  58,  39,  39,  39, },
    {  35,  45,  53,  54,  51,  39,  39,  39, },
    {  19,  46,  38,  39,  52,  39,  39,  39, },
    {   8,  12,   8,   8,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  19,  54,  39,  39,  50,  39,  39,  39,   0,  39,  39,  39, },
    {  19,  39,  54,  39,  19,  39,  39,  39,  56,  39,  39,  39, },
    {  18,  39,  39,  39,  11,  39,  39,  39,   0,  39,  39,  39, },
    {   8,   8,   8,   8,   8,   0,   4,   4,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  38,  54,  39,  41,  39,  39,  39, },
    {  34,  38,  62,  39,  26,  39,  39,  39, },
    {  26,  39,  39,  39,  19,  39,  39,  39, },
    {   8,   8,   8,   8,   4,   0,   0,   0, },
  })
};

const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  33,  40,  25,  41,  26,  42,  25,  33,  26,  34,  27,  25,  41,  42,  42,  35,  33,  27,  35,  42,  43, },
    {  18,  17,  33,  18,  34,  42,  25,  33,  26,  42,  27,  25,  34,  42,  42,  20,  26,  27,  42,  20,  20, },
    {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  20,  34,  42,  20,  43,  20, },
    {   8,   9,  12,  13,  13,  13,  10,  13,  13,  13,  13,  13,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  33,  25,  26,  19,  19,  27,  33,  42,  43,  27,  43, },
    {  25,  25,  26,  11,  19,  27,  33,  42,  50,  20,  43, },
    {  33,  25,  26,  42,  19,  27,  26,  50,  43,  20,  43, },
    {   9,  13,  12,  12,  13,  13,  13,  13,  13,  13,  13, },
  })
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,   0,   0,  17,  25,  18,   0,   9,  25,  33,  19,   0,  25,  33,  26,  20,  25,  33,  34,  42,  29, },
    {  17,   0,   1,  17,  25,  18,   0,   9,  25,  33,  34,   9,  25,  18,  26,  20,  25,  18,  19,  27,  21, },
    {  25,   1,  40,  25,  33,  11,  17,  25,  25,  18,   4,  17,  33,  11,   4,   5,  33,  19,  20,  28,  22, },
    {   1,   5,   9,   9,   9,   6,   5,   9,  10,  10,   9,   9,   9,   9,   9,   9,   6,   8,   9,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,   1,  40,  33,  18,   4,  25,  33,  27,  36,  37, },
    {  17,   9,  25,  10,   3,   4,  17,  33,  19,  28,  29, },
    {  48,   9,  25,  18,  26,  27,  25,  26,  35,  28,  37, },
    {   1,   5,   8,   8,   8,   6,   6,   9,   8,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,   0,  33,  34,  35,  36,  25,  49,  35,  28,  29,  40,  42,  43,  36,  37,  56,  58,  59,  45,  38, },
    {   0,  17,  26,  19,  20,  21,  25,  34,  20,  28,  29,  33,  27,  28,  29,  37,  34,  28,  44,  37,  38, },
    {  25,  25,  11,  27,  20,  21,  18,  12,  28,  21,  22,  34,  28,  29,  29,  30,  28,  29,  45,  30,  23, },
    {   9,   5,  10,  13,  13,  10,   9,  10,  13,  13,  13,   9,  10,  10,  10,  10,   8,   9,   8,  10,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,  40,  42,  20,  21,  29,  49,  52,  53,  38,  46, },
    {   0,  25,  27,  20,  13,   6,  57,  52,  30,  38,  31, },
    {  40,  33,  27,  28,  21,  37,  51,  37,  53,  38,  46, },
    {   9,   9,  10,  12,  12,  10,   5,   9,   9,   9,   9, },
  })
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  14,   6,   5,   7,   7,  12,   7,   7,   6,  12,  22,   7,   6,  14,  20,  28,   7,  13,  13,  20, },
    {   6,  13,  12,   6,   6,   4,  14,  14,   5,  12,  29,  14,  13,   5,  36,  28,  14,  13,  20,  19, },
    {   6,  13,  12,   6,  14,  12,  14,  14,  29,   4,  14,   7,  14,  29,   4,  29,  30,  37,  29,  58, },
    {   8,   5,   4,   5,   4,   4,   5,   4,   1,   0,   5,   1,   0,   0,   0,   1,   1,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  11,   5,   3, },
    {  19,   4,  18, },
    {  12,  11,   3, },
    {   2,   1,   1, },
  })
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  13,  13,  20,   6,   6,  12,  14,  14,   5,  13,  14,   7,   5,  12,  21,  13,   7,  13,  12,  41, },
    {   5,   5,  12,   6,   6,  19,   6,  14,   5,  19,  29,   7,  13,   5,  36,  21,   7,  13,   5,  27, },
    {  13,   5,   4,   6,   6,  11,  14,  14,   5,  11,  14,   7,  14,   5,   3,  21,  45,  45,  21,  34, },
    {   8,   5,   8,   5,   5,   4,   5,   5,   4,   0,   5,   5,   1,   0,   0,   1,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  11,   5,  19, },
    {  11,   4,  18, },
    {  12,   4,   3, },
    {   6,   2,   2, },
  })
};

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
  {  34, },
  {  49, },
  {  42, },
  {  12, },
});

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
({
  {  50, },
  {  28, },
  { CNU, },
  {   5, },
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
  {   2, },
  {  60, },
  {  59, },
  {   0, },
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
  {  10, },
  {   5, },
  {   5, },
  {   0, },
});

const CtxSet ContextSetCfg::TransquantBypassFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
({
  {  45,  37, },
  {  38,  45, },
  { CNU,  45, },
  {   8,   8, },
});

#if JVET_O0119_BASE_PALETTE_444
const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, },
  { DWS, DWS, DWS, DWS, DWS, },
});

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { DWS, DWS, DWS, },
});
#endif

const CtxSet ContextSetCfg::RdpcmFlag = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::RdpcmDir = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::MTSIndex = ContextSetCfg::addCtxSet
({
  {  29, CNU, CNU, CNU, CNU, CNU,  33,  18,  27,   0, CNU, },
  {  29, CNU, CNU, CNU, CNU, CNU,  18,  33,  27,   0, CNU, },
  {  20, CNU, CNU, CNU, CNU, CNU,  33,   0,  42,   0, CNU, },
  {   8, DWS, DWS, DWS, DWS, DWS,   1,   0,   9,   0, DWS, },
});

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
  {  48,  43, },
  {  33,  43, },
  {  33,  43, },
  {   9,   2, },
});

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
({
  {  57,  58, },
  {  57,  58, },
  { CNU, CNU, },
  {   1,   5, },
});

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
({
  {  42, },
  {  42, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
({
  {  35,  51,  20, },
  {  20,  43,  12, },
  { CNU, CNU, CNU, },
  {   8,   4,   4, },
});

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {  13, },
});

const CtxSet ContextSetCfg::CrossCompPred = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  {  58,  33,  50,  59,  52, },
  {  59,  48,  58,  60,  60, },
  { CNU,  34, CNU, CNU, CNU, },
  {   0,   5,   1,   0,   1, },
});

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
({
  {  26,  45,  46,  33,  61,  54,  33,  61,  54, },
  {   6,  23,  46,  27,  61,  54,  20,  46,  54, },
  {  39,  39,  39,  54,  39,  39,  31,  62,  39, },
  {   0,   0,   0,   0,   0,   0,   0,   0,   0, },
});

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
({
  {  18,  18, },
  {  20,  12, },
  {  44,  44, },
  {   0,   0, },
});

const CtxSet ContextSetCfg::AlfUseLatestFilt = ContextSetCfg::addCtxSet
({
  {  58, },
  {  50, },
  {  31, },
  {   0, },
});

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
({
  {  53, },
  {  53, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::MHIntraFlag = ContextSetCfg::addCtxSet
({
  {  58, },
  {  58, },
  { CNU, },
  {   1, },
});

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
  {   0,  43,  30, },
  {   0,  42,  37, },
  {  17,  27,  36, },
  {   1,   5,   8, },
});

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
({
  {  51,  44,  45, },
  {  36,  44,  45, },
  {  43,  29,  51, },
  {   1,   1,   0, },
});

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
({
  {  18,  35,  37, },
  {  18,  12,  29, },
  {  11,   5,  38, },
  {   5,   5,   5, },
});

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
({
  {  25,  50,  37, },
  {  40,  35,  44, },
  {  25,  28,  38, },
  {  13,  13,   8, },
});

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
({
  {  11, },
  {   3, },
  {  11, },
  {   6, },
});

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
({
  { CNU,  10,   4,   4,   5, },
  { CNU,   2,   3,   3,   4, },
  { CNU,   3,   3,   3,   3, },
  { DWS,   1,   1,   1,   1, },
});

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
({
  {  19,  11,  12, CNU, },
  {  18,  11,  12, CNU, },
  {  11,   5,  13, CNU, },
  {   4,   2,   1, DWS, },
});

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
({
  {  28,  25,  53, CNU, CNU, CNU, },
  {   5,  10,  53, CNU, CNU, CNU, },
  {  20,   2,  46, CNU, CNU, CNU, },
  {   1,   4,   4, DWS, DWS, DWS, },
});
#else
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
  { 136, 153, 141, 122, 125, 156, 138, 126, 143, },
  { 122, 139, 186, 124, 125, 141, 139, 141, 158, },
  { 138, 154, 172, 124, 140, 142, 154, 142, 175, },
  {  12,  12,   8,   8,  13,  12,   5,  10,  12, },
});

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
  { 123, 140, 142, 136, 138, 140, },
  { 139, 126, 142, 136, 138, 125, },
  { 124, 125, 127, 136, 153, 126, },
  {   0,   8,   8,  12,  12,  12, },
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
  { 154, 168, 155, 139, 155, },
  { 169, 168, 170, 153, 170, },
  { 154, 168, 155, 153, 155, },
  {  10,   9,   9,   8,   8, },
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
  { 154, 155, 154, 140, },
  { 169, 141, 154, 155, },
  { 169, 170, 169, 170, },
  {  12,  12,  12,  12, },
});

#if JVET_O0050_LOCAL_DUAL_TREE
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
({
  { 192, 168, },
  { 179, 139, },
  { CNU, CNU, },
  {   5,   2, },
});
#endif

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
  { 197, 214, 216, },
  { 211, 198, 185, },
  {   0, 152, 154, },
  {   5,   8,   8, },
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
  { 111, },
  { 111, },
  { 138, },
  {   5, },
});

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
({
#if JVET_O0249_MERGE_SYNTAX
  { 159, 126 },
  { 143, 111 },
  { 154, 154 },
  {   6,   5 }
#else
  { 142, 125, },
  { 141, 110, },
  { CNU, CNU, },
  {   4,   4, },
#endif
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
  { 137, },
  { 154, },
  { 138, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
#if JVET_O0249_MERGE_SYNTAX
  { 108, },
  { 108, },
  { 154, },
  {   5, },
#else
  { 109, },
  { 124, },
  { CNU, },
  {   5, },
#endif
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
  { 154, },
  { 154, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  { 213, },
  { 244, },
  { CNU, },
  {   1, },
});

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
  { 192, 168, },
  { 179, 139, },
  { CNU, CNU, },
  {   5,   2, },
});

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
  {  90, 212, },
  { 118, 212, },
  { 134, 169, },
  {   8,   8, },
});

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
({
  { 154, },
  { 154, },
  { 170, },
  {   6, },
});

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
({
  { 125, 125, },
  { 139, 139, },
  { 110, 154, },
  {   4,   5, },
});
#if JVET_O1153_INTRA_CHROMAMODE_CODING
const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
({
  { 153,},
  { 168,},
  { 169,},
  {   4,},
  });
const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
  { 149,},
  { 166,},
  { 138,},
  {   5,},
  });
#else
const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
  { 137, 139, 140, },
  { 138, 139, 169, },
  { 154, 154, 154, },
  {   5,   8,   8, },
});
#endif

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
({
  { 181, 182, 183, 152, },
  { 181, 197, 183, 152, },
  { 165, 196, 168,  40, },
  {   9,  10,  10,   1, },
});

#if !JVET_O0925_MIP_SIMPLIFICATIONS
const CtxSet ContextSetCfg::MipMode = ContextSetCfg::addCtxSet
({
  { 196, },
  { 196, },
  { 182, },
  {   9, },
});
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
  { 111, 125,  95,  94, 164, },
  { 126, 111, 110, 109, 136, },
  { CNU, CNU, CNU, CNU, CNU, },
  {   0,   0,   1,   4,   0, },
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
  { 125, 139, },
  { 138, 168, },
  { CNU, CNU, },
  {   4,   5, },
});

#if JVET_O0500_SEP_CTX_AFFINE_SUBBLOCK_MRG
const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
({
  { 183, 185, 187, },
  { 168, 169, 171, },
  { CNU, CNU, CNU, },
  {   4,   4,   4, },
  });
#endif

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
#if JVET_O0249_MERGE_SYNTAX
  { 180, 169, 186 },
  { 195, 168, 155 },
  { 154, 154, 154 },
  {   5,   4,   4 }
#else
  { 183, 185, 187, },
  { 168, 169, 171, },
  { CNU, CNU, CNU, },
  {   4,   4,   4, },
#endif
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
  { 153, },
  { 153, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
  { 110, },
  { 110, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::GBiIdx = ContextSetCfg::addCtxSet
({
  { 228, },
  { 228, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
  { 169, 183, },
  { 155, 154, },
  { 126, 156, },
  {   9,   5, },
});

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
({
  { 148, 154, },
  {   0, 140, },
  {  40, 169, },
  {   1,   4, },
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
  {  94, },
  {  95, },
  { 110, },
  {   4, },
});

const CtxSet ContextSetCfg::QtCbf[] =
{
  ContextSetCfg::addCtxSet
  ({
#if JVET_O0193_REMOVE_TR_DEPTH_IN_CBF_CTX
    { 127, 111, 124, 140 },
    { 127,  79, 139, 126 },
    { 126, 138, 124, 111 },
    {   5,   1,   8,   8 },
#else
    { 142, 127, 124, 140, 111, },
    { 143, 127, 139, 126,  79, },
    { CNU, 126, 124, 111, 138, },
    {   1,   5,   8,   8,   1, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_O0193_REMOVE_TR_DEPTH_IN_CBF_CTX
    { 163, },
    { 150, },
    { 124, },
    {   5, },
#else
    { 163, 135, },
    { 150, 121, },
    { 124, CNU, },
    {   5,   0, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
    { 161, 155, },
    { 163, 155, },
    { 151, 141, },
    {   2,   2, },
  })
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
    { 105, 170, },
    { 106, 156, },
    { 107, 158, },
    {   8,   5, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 105, 155, },
    { 105, 155, },
    {  90, 126, },
    {   5,   8, },
  }),
};

const CtxSet ContextSetCfg::SigFlag[] =
{
#if JVET_O0617_SIG_FLAG_CONTEXT_REDUCTION
  ContextSetCfg::addCtxSet
  ({
    {  88, 166, 182, 169, 101, 167, 168, 155, 194, 213, 183, 156, },
    { 132, 152, 168, 140, 177, 182, 154, 155, 151, 213, 169, 156, },
    {  89, 138, 139, 140, 150, 139, 140, 141, 138, 185, 141, 157, },
    {  12,   9,   9,  10,   9,   9,   9,   9,   8,   8,   8,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  27, 167, 168, 140, 180, 199, 214, 186, },
    { 133, 138, 139, 140, 181, 214, 200, 157, },
    { 134, 153, 154, 155, 167, 186, 186, 143, },
    {   9,   9,   9,  13,   5,   5,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 152, 156, 186, 202, 182, 249, 247, 207, 182, 223, 223, 223, },
    { 123, 142, 172, 218, 138, 250, 248, 223, 125, 223, 223, 223, },
    {  93, 142, 143, 175, 153, 223, 223, 238, 154, 223, 223, 223, },
    {   9,  12,   8,   8,   8,   8,   8,   5,   8,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 182, 171, 143, 190, 183, 223, 223, 223, },
    { 168, 156, 216, 249, 169, 223, 223, 223, },
    { 138, 173, 157, 223, 170, 223, 223, 223, },
    {   8,  12,   8,   8,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 123, 175, 223, 223, 212, 223, 223, 223,   0, 223, 223, 223, },
    { 123, 223, 205, 223, 138, 223, 223, 223, 196, 223, 223, 223, },
    { 107, 206, 223, 223,  93, 223, 223, 238,  55, 223, 223, 223, },
    {   8,   8,   8,   8,   8,   0,   4,   4,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 167, 187, 249, 207, 181, 223, 223, 223, },
    { 167, 157, 191, 223, 152, 223, 223, 223, },
    { 152, 236, 223, 223, 123, 223, 223, 223, },
    {   8,   8,   8,   8,   4,   0,   0,   0, },
  }),
#else
  ContextSetCfg::addCtxSet
  ({
    {  88, 166, 152, 182, 168, 154, 116, 167, 182, 168, 183, 155, 208, 213, 183, 183, 169, 185, },
    { 132, 152, 167, 168, 183, 140, 177, 182, 168, 154, 169, 155, 151, 213, 183, 169, 184, 156, },
    {  89, 138, 153, 139, 154, 140, 135, 139, 139, 140, 140, 141, 123, 185, 140, 170, 141, 157, },
    {  12,   9,   9,   9,   9,  10,   9,   9,   9,   9,   9,   9,   8,   8,   8,   8,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  27, 167, 153, 168, 139, 140, 180, 199, 183, 199, 199, 186, },
    { 133, 138, 153, 139, 154, 140, 181, 229, 169, 229, 170, 157, },
    { 104, 153, 168, 154, 154, 155, 167, 186, 170, 201, 171, 143, },
    {   9,   9,  12,   9,  12,  13,   5,   5,   8,   8,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 152, 156, 201, 186, 186, 187, 182, 249, 188, 232, 188, 191, 182, 223, 223, 223, 223, 223, },
    { 123, 142, 157, 172, 172, 218, 138, 250, 248, 248, 234, 223, 125, 223, 223, 223, 223, 223, },
    {  93, 142, 157, 143, 188, 175, 153, 223, 251, 223, 223, 238, 154, 223, 223, 223, 223, 223, },
    {   9,  12,   9,   8,   8,   8,   8,   8,   8,   8,   8,   5,   8,   0,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 182, 171, 157, 143, 172, 189, 183, 223, 223, 223, 223, 223, },
    { 168, 156, 173, 216, 172, 234, 169, 223, 223, 223, 223, 223, },
    { 138, 173, 142, 172, 189, 223, 170, 223, 223, 223, 223, 223, },
    {   8,   9,  12,   8,   8,   8,   4,   0,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 123, 159, 223, 223, 247, 237, 212, 223, 223, 237, 237, 223, 176, 223, 223, 223, 223, 223, },
    { 123, 191, 223, 190, 218, 223, 138, 223, 223, 223, 223, 223, 196, 223, 223, 223, 223, 223, },
    { 107, 175, 223, 223, 252, 223,  78, 223, 223, 238, 223, 238,  25, 223, 223, 223, 223, 223, },
    {   8,   8,   4,   8,   8,   8,   8,   0,   0,   4,   8,   4,   4,   0,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 167, 201, 223, 248, 219, 207, 181, 223, 223, 223, 223, 223, },
    { 167, 171, 223, 190, 248, 223, 152, 223, 223, 223, 223, 223, },
    { 137, 250, 223, 237, 234, 223, 123, 223, 223, 223, 223, 223, },
    {   8,   8,   1,   8,   8,   8,   4,   0,   0,   0,   0,   0, },
  })
#endif
};

const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    { 121, 119, 121, 137, 152, 153, 119, 151, 152, 138, 168, 135, 152, 153, 168, 139, 151, 153, 139, 168, 154, },
    { 121, 104, 121, 137, 138, 153, 104, 122, 123, 153, 124, 106, 138, 153, 168, 139, 137, 153, 168, 139, 139, },
    { 121, 135, 137, 152, 138, 153,  91, 137, 138, 153, 139, 151, 138, 153, 139, 139, 138, 168, 139, 154, 139, },
    {   8,   9,  12,  13,  13,  13,  10,  13,  13,  13,  13,  13,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 151, 120, 152, 138, 138, 138, 151, 168, 154, 153, 154, },
    { 150, 120, 137, 123, 138, 153, 136, 153, 168, 139, 154, },
    { 151, 135, 152, 153, 138, 153, 137, 168, 154, 139, 154, },
    {   9,  13,  12,  12,  13,  13,  10,  13,  13,  13,  13, },
  })
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    { 104,   0, 102,  89, 150, 122,   0,  58, 134, 136, 138,   0, 148, 136, 152, 124, 133, 136, 138, 153, 140, },
    {  73,   0,  26, 104, 120, 137,   0,  57, 105, 136, 138, 116,  90, 107, 152, 153, 104, 107, 123, 153, 125, },
    { 119,  41, 148, 135, 136, 123,  43,  60, 106, 122, 109,  73, 106, 108, 109, 124, 136, 138, 139, 154, 111, },
    {   4,   5,   9,   9,   9,   6,   5,   9,  10,   9,   9,   9,   9,   9,   8,   9,   9,   8,   9,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 149,  56, 134, 136,  92, 123, 104, 122, 124, 140, 126, },
    { 103,   0,  90,  91,  92,  93, 103, 136, 138, 154, 140, },
    { 165,  87, 120, 122, 122, 123, 120, 137, 168, 154, 155, },
    {   2,   5,   8,   8,   8,   6,   6,   9,   8,   8,   8, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0, 176, 151, 138, 139, 140, 162, 167, 139, 154, 155, 164, 153, 154, 169, 170, 181, 183, 184, 156, 157, },
    {   0, 133, 137, 138, 139, 125, 134, 138, 139, 154, 155, 136, 153, 154, 140, 170, 138, 154, 155, 170, 157, },
    { 134, 120, 123, 153, 139, 140, 121, 109, 139, 125, 111, 138, 154, 140, 155, 141, 154, 140, 185, 156, 143, },
    {   8,   5,   9,  13,  13,  10,   9,  10,  13,  13,  13,   9,  10,  10,  10,  10,   8,   9,   8,   9,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 161, 178, 153, 154, 140, 140, 211, 170, 186, 157, 188, },
    {   0, 135, 153, 139, 125, 140, 182, 170, 156, 142, 159, },
    { 164, 151, 153, 154, 125, 140, 154, 170, 186, 172, 159, },
    {   6,   9,  10,  12,  12,  10,   5,   9,   9,   9,  12, },
  })
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
    { 111, 111,  95, 111, 111, 124, 111, 126, 111, 124, 126, 126, 111, 111, 154, 154, 111, 110, 110, 124, },
    { 125, 110, 109, 125, 125, 123, 111, 111,  95, 123, 140, 111, 110,  95, 169, 125, 140, 139, 139, 138, },
    { 125, 110, 109, 140, 111, 109, 111, 111, 140, 123, 111, 126, 111, 140,  79, 155, 142, 141, 140, 198, },
    {   8,   5,   4,   5,   4,   4,   5,   4,   1,   0,   5,   1,   0,   0,   0,   1,   1,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 137,  95,  63, },
    { 138, 123,  92, },
    { 109, 108,  77, },
    {   2,   1,   1, },
  })
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
    { 125, 110, 139, 125, 111, 124, 111, 111,  95, 110, 140, 126, 110, 124, 155, 139, 111, 110, 124, 181, },
    {  95,  95, 109, 110, 110, 123, 125, 111, 124, 123, 140, 111, 110, 124, 154, 125, 126, 110, 124, 153, },
    { 110,  95,  94, 125, 125, 108, 111, 111,  95, 108, 111, 141, 111,  95,  78, 140, 186, 156, 125, 138, },
    {   8,   5,   8,   5,   5,   4,   5,   5,   4,   0,   5,   5,   1,   0,   0,   1,   4,   1,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    { 108, 124, 138, },
    { 108, 123,  92, },
    { 109,  94,  92, },
    {   3,   2,   2, },
  })
};

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
  { 153, },
  { 168, },
  { 153, },
  {  10, },
});

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
({
  { 154, },
  { 139, },
  { CNU, },
  {   5, },
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
  {  31, },
  { 244, },
  { 214, },
  {   0, },
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
  {  46, },
  {  95, },
  {  95, },
  {   0, },
});

const CtxSet ContextSetCfg::TransquantBypassFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
( {
  { 184, CNU, },
  { 155, CNU, },
  { 169, 155, },
  {   8,   8, },
} );

#if JVET_O0119_BASE_PALETTE_444
const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
({
  { 146 },
  { 146 },
  { 147 },
  { 1 }
});

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
({
  { 153 },
  { 138 },
  { 168 },
  { 5 }
});

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
({
  { 167 },
  { 167 },
  { 167 },
  { 8 }
});

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
({
  { 157, 169, 138, 170, 155 },
  { 186, 169, 166, 186, 185 },
  { 155, 168, 167, 155, 154 },
  { 5,   6,   5,   9,  10 }
});

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
({
  { 187, 172, 156 },
  { 187, 187, 185 },
  { 201, 171, 155 },
  { 0,   5,   5 }
});
#endif

const CtxSet ContextSetCfg::RdpcmFlag = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::RdpcmDir = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::MTSIndex = ContextSetCfg::addCtxSet
({
  { CNU, 141, 141, 140, 140, CNU, 218, 137, 153,   0, CNU, },
  { CNU, 141, 141, 126, 155, CNU, 250, 166, 153,   0, CNU, },
  { CNU, CNU, 139, 139, 154, CNU, 220,   0, 182, 161, CNU, },
  { DWS,   8,   8,   9,   8, DWS,   1,   0,   9,   0, DWS, },
});

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
  { 165, 154, },
  { 150, 169, },
  { 151, 169, },
  {   9,   1, },
});

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
({
  { 197, 183, },
  { 211, 183, },
  { CNU, CNU, },
  {   4,   5, },
});

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
({
  { 168, },
  { 168, },
  { CNU, },
  {   9, },
});

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
({
  { 139, 169, 139, },
  { 139, 154, 124, },
  { CNU, CNU, CNU, },
  {   8,   4,   4, },
});

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
({
  { 154, },
  { 154, },
  { CNU, },
  {  13, },
});

const CtxSet ContextSetCfg::CrossCompPred = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

#if JVET_O0057_ALTHPELIF
const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  { 212, 180, 183, 242, 242, },
  { 213, 166, 198, 244, 244, },
  { CNU, 182, CNU, CNU, CNU, },
  {   1,   5,   1,   0,   0, },
});
#else
const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  { 212, 180, 183, 242, },
  { 213, 166, 198, 244, },
  { CNU, 182, CNU, CNU, },
  {   1,   5,   1,   0, },
});
#endif

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
({
  { 167, 200, 174, 211, 247, 249, 151, 247, 249, },
  { 110, 142, 203, 168, 246, 248, 168, 246, 248, },
  { 223, 223, 223, 202, 204, 250, 202, 204, 221, },
  {   0,   0,   0,   0,   0,   0,   0,   0,   0, },
});

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
({ // Cb,  Cr
  { 100, 100, }, // B / P (cabac_init_flag)
  { 153, 153, }, // P / B (cabac_init_flag)
  { 200, 200, }, // I
  {   0,   0, }, // shiftIdx
});

#endif
const CtxSet ContextSetCfg::AlfUseLatestFilt = ContextSetCfg::addCtxSet
({
  { 169, },
  { 183, },
  { 159, },
  {   0, },
});

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
({
  { 201, },
  { 200, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::MHIntraFlag = ContextSetCfg::addCtxSet
({
#if JVET_O0249_MERGE_SYNTAX
  { 184, },
  { 184, },
  { 154, },
  {   1, },
#else
  { 184, },
  { 185, },
  { CNU, },
  {   0, },
#endif
});



const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
  {   0, 154, 156, },
  {   0, 153, 155, },
  { 133, 153, 154, },
  {   1,   5,   8, },
});

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
({
#if JVET_O0105_ICT
  { 156, 156, 156, },
  { 156, 156, 156, },
  { 184, 184, 184, },
  {   1,   1,   1, },
#else
  { 156, },
  { 156, },
  { 184, },
  {   1, },
#endif
});

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
({
  { 123, 139, 155, },
  { 123, 124, 140, },
  { 123, 139, 156, },
  {   5,   5,   5, },
});

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
({
  { 150, 168, 140, },
  { 136, 124, 140, },
  { 135, 139, 126, },
  {  13,  13,   9, },
});

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
({
  { 123, },
  { 123, },
  { 138, },
  {   5, },
});

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
({
  { 124,  63,  79,  79,  95, },
  { 138,  47,  63,  63,  63, },
  { 124,  63,  63,  63,  63, },
  {   4,   1,   1,   1,   1, },
});

#if JVET_O0122_TS_SIGN_LEVEL
const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
({
  { 139, 108, 124, 111 },
  { 122, 138, 139, 110 },
  { 123, 139, 110, 125 },
  {   4,   2,   1,   5 }
  });
#endif

#if JVET_O0122_TS_SIGN_LEVEL

const CtxSet ContextSetCfg::TsResidualSign =
{
  ContextSetCfg::addCtxSet
  ({
  { 139,  92, 201, 139, 122, 171 },
  { 124,  77, 171, 169, 121, 187 },
  { 124,  61, 187, 154, 121, 187 },
  {   1,   4,   1,   5,   5,   5 }
    }),
};

#else
const CtxSet ContextSetCfg::TsResidualSign =
{
  ContextSetCfg::addCtxSet
  ({
    { 154, 154, },
    { 139, 154, },
    { 124, 139, },
    {   1,   2, },
   }),
};
#endif
#endif
// clang-format on

const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();


// combined sets
#if JVET_O0119_BASE_PALETTE_444
const CtxSet ContextSetCfg::Palette = { ContextSetCfg::RotationFlag, ContextSetCfg::RunTypeFlag, ContextSetCfg::IdxRunModel, ContextSetCfg::CopyRunModel };
#endif
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
const CtxSet ContextSetCfg::Alf = { ContextSetCfg::ctbAlfFlag, ContextSetCfg::ctbAlfAlternative, ContextSetCfg::AlfUseLatestFilt, ContextSetCfg::AlfUseTemporalFilt };
#else
const CtxSet ContextSetCfg::Alf = { ContextSetCfg::ctbAlfFlag, ContextSetCfg::AlfUseLatestFilt, ContextSetCfg::AlfUseTemporalFilt };
#endif

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
  const std::vector<uint8_t> &rateInitTable = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES);
  CHECK(m_CtxBuffer.size() != rateInitTable.size(),
        "Size of rate init table (" << rateInitTable.size() << ") does not match size of context buffer ("
                                    << m_CtxBuffer.size() << ").");
  int clippedQP = Clip3( 0, MAX_QP, qp );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].init( clippedQP, initTable[k] );
    m_CtxBuffer[k].setLog2WindowSize(rateInitTable[k]);
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

