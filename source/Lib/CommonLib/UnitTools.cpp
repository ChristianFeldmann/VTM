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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools


uint64_t CS::getEstBits(const CodingStructure &cs)
{
  return cs.fracBits >> SCALE_BITS;
}



bool CS::isDualITree( const CodingStructure &cs )
{
  return cs.slice->isIRAP() && !cs.pcv->ISingleTree;
}

UnitArea CS::getArea( const CodingStructure &cs, const UnitArea &area, const ChannelType chType )
{
  return isDualITree( cs ) ? area.singleChan( chType ) : area;
}
void CS::setRefinedMotionField(CodingStructure &cs)
{
  for (CodingUnit *cu : cs.cus)
  {
    for (auto &pu : CU::traversePUs(*cu))
    {
      PredictionUnit subPu = pu;
      int dx, dy, x, y, num = 0;
      dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
      dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
      Position puPos = pu.lumaPos();
      if (PU::checkDMVRCondition(pu))
      {
        for (y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy)
        {
          for (x = puPos.x; x < (puPos.x + pu.lumaSize().width); x = x + dx)
          {
            subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
            subPu.mv[0] = pu.mv[0];
            subPu.mv[1] = pu.mv[1];
            subPu.mv[REF_PIC_LIST_0] += pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_1] -= pu.mvdL0SubPu[num];
#if JVET_N0334_MVCLIPPING
            subPu.mv[REF_PIC_LIST_0].clipToStorageBitDepth();
            subPu.mv[REF_PIC_LIST_1].clipToStorageBitDepth();
#endif
            pu.mvdL0SubPu[num].setZero();
            num++;
            PU::spanMotionInfo(subPu);
          }
        }
      }
    }
  }
}
// CU tools

bool CU::isIntra(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTRA;
}

bool CU::isInter(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTER;
}

bool CU::isIBC(const CodingUnit &cu)
{
  return cu.predMode == MODE_IBC;
}

bool CU::isRDPCMEnabled(const CodingUnit& cu)
{
  return cu.cs->sps->getSpsRangeExtension().getRdpcmEnabledFlag(cu.predMode == MODE_INTRA ? RDPCM_SIGNAL_IMPLICIT : RDPCM_SIGNAL_EXPLICIT);
}

bool CU::isLosslessCoded(const CodingUnit &cu)
{
  return cu.cs->pps->getTransquantBypassEnabledFlag() && cu.transQuantBypass;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return ( cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx() ) && ( cu.tileIdx == cu2.tileIdx );
}

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  uint32_t ctuSizeBit = g_aucLog2[cu.cs->sps->getMaxCUWidth()];

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

uint32_t CU::getIntraSizeIdx(const CodingUnit &cu)
{
  uint8_t uiWidth = cu.lumaSize().width;

  uint32_t  uiCnt   = 0;

  while (uiWidth)
  {
    uiCnt++;
    uiWidth >>= 1;
  }

  uiCnt -= 2;

  return uiCnt > 6 ? 6 : uiCnt;
}

bool CU::isLastSubCUOfCtu( const CodingUnit &cu )
{
  const SPS &sps      = *cu.cs->sps;
  const Area cuAreaY = CS::isDualITree( *cu.cs ) ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : ( const Area& ) cu.Y();

  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == sps.getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == sps.getPicHeightInLumaSamples() ) );
}

uint32_t CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType].lumaPos(), *cu.cs->pcv );
}

int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const CodingStructure &cs = *cu.cs;

  if ( !cu.blocks[cu.chType].x && !( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) && ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType) != NULL ) )
  {
    return ( ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType ) )->qp );
  }
  else
  {
    const int a = ( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType))->qp : prevQP;
    const int b = ( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( -1, 0 ), cu.chType))->qp : prevQP;

    return ( a + b + 1 ) >> 1;
  }
}


uint32_t CU::getNumPUs( const CodingUnit& cu )
{
  uint32_t cnt = 0;
  PredictionUnit *pu = cu.firstPU;

  do
  {
    cnt++;
  } while( ( pu != cu.lastPU ) && ( pu = pu->next ) );

  return cnt;
}

void CU::addPUs( CodingUnit& cu )
{
  cu.cs->addPU( CS::getArea( *cu.cs, cu, cu.chType ), cu.chType );
}


PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  if( depth >= cu.depth ) return CU_DONT_SPLIT;

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if     ( cuSplitType == CU_QUAD_SPLIT    ) return CU_QUAD_SPLIT;

  else if( cuSplitType == CU_HORZ_SPLIT    ) return CU_HORZ_SPLIT;

  else if( cuSplitType == CU_VERT_SPLIT    ) return CU_VERT_SPLIT;

  else if( cuSplitType == CU_TRIH_SPLIT    ) return CU_TRIH_SPLIT;
  else if( cuSplitType == CU_TRIV_SPLIT    ) return CU_TRIV_SPLIT;
  else   { THROW( "Unknown split mode"    ); return CU_QUAD_SPLIT; }
}

bool CU::hasNonTsCodedBlock( const CodingUnit& cu )
{
  bool hasAnyNonTSCoded = false;

  for( auto &currTU : traverseTUs( cu ) )
  {
    for( uint32_t i = 0; i < ::getNumberValidTBlocks( *cu.cs->pcv ); i++ )
    {
      hasAnyNonTSCoded |= ( currTU.blocks[i].valid() && ( isLuma(ComponentID(i)) ? currTU.mtsIdx != MTS_SKIP : true ) && TU::getCbf( currTU, ComponentID( i ) ) );
    }
  }

  return hasAnyNonTSCoded;
}

#if JVET_N0193_LFNST
uint32_t CU::getNumNonZeroCoeffNonTs( const CodingUnit& cu, const bool lumaFlag, const bool chromaFlag )
#else
uint32_t CU::getNumNonZeroCoeffNonTs( const CodingUnit& cu )
#endif
{
  uint32_t count = 0;
  for( auto &currTU : traverseTUs( cu ) )
  {
#if JVET_N0193_LFNST
    count += TU::getNumNonZeroCoeffsNonTS( currTU, lumaFlag, chromaFlag );
#else
    count += TU::getNumNonZeroCoeffsNonTS( currTU );
#endif
  }

  return count;
}

#if JVET_N0193_LFNST
uint32_t CU::getNumNonZeroCoeffNonTsCorner8x8( const CodingUnit& cu, const bool lumaFlag, const bool chromaFlag )
{
  uint32_t count = 0;
  for( auto &currTU : traverseTUs( cu ) )
  {
    count += TU::getNumNonZeroCoeffsNonTSCorner8x8( currTU, lumaFlag, chromaFlag );
  }

  return count;
}
#endif

bool CU::divideTuInRows( const CodingUnit &cu )
{
  CHECK( cu.ispMode != HOR_INTRA_SUBPARTITIONS && cu.ispMode != VER_INTRA_SUBPARTITIONS, "Intra Subpartitions type not recognized!" );
  return cu.ispMode == HOR_INTRA_SUBPARTITIONS ? true : false;
}

bool CU::firstTestISPHorSplit( const int width, const int height, const ComponentID compID, const CodingUnit *cuLeft, const CodingUnit *cuAbove )
{
  //this function decides which split mode (horizontal or vertical) is tested first (encoder only)
  //we check the logarithmic aspect ratios of the block
  int aspectRatio = g_aucLog2[width] - g_aucLog2[height];
  if( aspectRatio > 0 )
  {
    return true;
  }
  else if( aspectRatio < 0 )
  {
    return false;
  }
  else //if (aspectRatio == 0)
  {
    //we gather data from the neighboring CUs
    const int cuLeftWidth    = cuLeft  != nullptr                                    ? cuLeft->blocks[compID].width   : -1;
    const int cuLeftHeight   = cuLeft  != nullptr                                    ? cuLeft->blocks[compID].height  : -1;
    const int cuAboveWidth   = cuAbove != nullptr                                    ? cuAbove->blocks[compID].width  : -1;
    const int cuAboveHeight  = cuAbove != nullptr                                    ? cuAbove->blocks[compID].height : -1;
    const int cuLeft1dSplit  = cuLeft  != nullptr &&  cuLeft->predMode == MODE_INTRA ? cuLeft->ispMode                :  0;
    const int cuAbove1dSplit = cuAbove != nullptr && cuAbove->predMode == MODE_INTRA ? cuAbove->ispMode               :  0;
    if( cuLeftWidth != -1 && cuAboveWidth == -1 )
    {
      int cuLeftAspectRatio = g_aucLog2[cuLeftWidth] - g_aucLog2[cuLeftHeight];
      return cuLeftAspectRatio < 0 ? false : cuLeftAspectRatio > 0 ? true : cuLeft1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
    }
    else if( cuLeftWidth == -1 && cuAboveWidth != -1 )
    {
      int cuAboveAspectRatio = g_aucLog2[cuAboveWidth] - g_aucLog2[cuAboveHeight];
      return cuAboveAspectRatio < 0 ? false : cuAboveAspectRatio > 0 ? true : cuAbove1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
    }
    else if( cuLeftWidth != -1 && cuAboveWidth != -1 )
    {
      int cuLeftAspectRatio = g_aucLog2[cuLeftWidth] - g_aucLog2[cuLeftHeight];
      int cuAboveAspectRatio = g_aucLog2[cuAboveWidth] - g_aucLog2[cuAboveHeight];
      if( cuLeftAspectRatio < 0 && cuAboveAspectRatio < 0 )
      {
        return false;
      }
      else if( cuLeftAspectRatio > 0 && cuAboveAspectRatio > 0 )
      {
        return true;
      }
      else if( cuLeftAspectRatio == 0 && cuAboveAspectRatio == 0 )
      {
        if( cuLeft1dSplit != 0 && cuAbove1dSplit != 0 )
        {
          return cuLeft1dSplit == VER_INTRA_SUBPARTITIONS && cuAbove1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
        }
        else if( cuLeft1dSplit != 0 && cuAbove1dSplit == 0 )
        {
          return cuLeft1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
        }
        else if( cuLeft1dSplit == 0 && cuAbove1dSplit != 0 )
        {
          return cuAbove1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
        }
        return true;
      }
      else
      {
        return cuLeftAspectRatio > cuAboveAspectRatio ? cuLeftAspectRatio > 0 : cuAboveAspectRatio > 0;
      }
      //return true;
    }
    return true;
  }
}

PartSplit CU::getISPType( const CodingUnit &cu, const ComponentID compID )
{
  if( cu.ispMode && isLuma( compID ) )
  {
    const bool tuIsDividedInRows = CU::divideTuInRows( cu );

    return tuIsDividedInRows ? TU_1D_HORZ_SPLIT : TU_1D_VERT_SPLIT;
  }
  return TU_NO_ISP;
}

bool CU::isISPLast( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  PartSplit partitionType = CU::getISPType( cu, compID );

  Area originalArea = cu.blocks[compID];
  switch( partitionType )
  {
    case TU_1D_HORZ_SPLIT:
      return tuArea.y + tuArea.height == originalArea.y + originalArea.height;
    case TU_1D_VERT_SPLIT:
      return tuArea.x + tuArea.width == originalArea.x + originalArea.width;
    default:
      THROW( "Unknown ISP processing order type!" );
      return false;
  }
}

bool CU::isISPFirst( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  return tuArea == cu.firstTU->blocks[compID];
}

ISPType CU::canUseISPSplit( const CodingUnit &cu, const ComponentID compID )
{
  const int width     = cu.blocks[compID].width;
  const int height    = cu.blocks[compID].height;
#if MAX_TB_SIZE_SIGNALLING
  const int maxTrSize = cu.cs->sps->getMaxTbSize();
#else
  const int maxTrSize = MAX_TB_SIZEY;
#endif
  return CU::canUseISPSplit( width, height, maxTrSize );
}

ISPType CU::canUseISPSplit( const int width, const int height, const int maxTrSize )
{
  bool widthCannotBeUsed = false, heightCannotBeUsed = false;

  const uint32_t minTuSizeForISP = MIN_TB_SIZEY;
  bool  notEnoughSamplesToSplit = ( g_aucLog2[width] + g_aucLog2[height] <= ( g_aucLog2[minTuSizeForISP] << 1 ) );
#if JVET_N0308_MAX_CU_SIZE_FOR_ISP
  bool  cuSizeLargerThanMaxTrSize = width  > maxTrSize || height > maxTrSize;
  widthCannotBeUsed  = cuSizeLargerThanMaxTrSize || notEnoughSamplesToSplit;
  heightCannotBeUsed = cuSizeLargerThanMaxTrSize || notEnoughSamplesToSplit;
#else
  widthCannotBeUsed  = width  > maxTrSize || notEnoughSamplesToSplit;
  heightCannotBeUsed = height > maxTrSize || notEnoughSamplesToSplit;
#endif

  if( !widthCannotBeUsed && !heightCannotBeUsed )
  {
    return CAN_USE_VER_AND_HORL_SPLITS; //both splits can be used
  }
  else if( widthCannotBeUsed && !heightCannotBeUsed )
  {
    return VER_INTRA_SUBPARTITIONS; //only the vertical split can be performed
  }
  else if( !widthCannotBeUsed && heightCannotBeUsed )
  {
    return HOR_INTRA_SUBPARTITIONS; //only the horizontal split can be performed
  }
  else
  {
    return NOT_INTRA_SUBPARTITIONS; //neither of the splits can be used
  }
}

uint32_t CU::getISPSplitDim( const int width, const int height, const PartSplit ispType )
{
  bool divideTuInRows = ispType == TU_1D_HORZ_SPLIT;
  uint32_t splitDimensionSize, nonSplitDimensionSize, partitionSize, divShift = 2;

  if( divideTuInRows )
  {
    splitDimensionSize    = height;
    nonSplitDimensionSize = width;
  }
  else
  {
    splitDimensionSize    = width;
    nonSplitDimensionSize = height;
  }

  const int minNumberOfSamplesPerCu = 1 << ( ( g_aucLog2[MIN_TB_SIZEY] << 1 ) );
  const int factorToMinSamples = nonSplitDimensionSize < minNumberOfSamplesPerCu ? minNumberOfSamplesPerCu >> g_aucLog2[nonSplitDimensionSize] : 1;
  partitionSize = ( splitDimensionSize >> divShift ) < factorToMinSamples ? factorToMinSamples : ( splitDimensionSize >> divShift );

  CHECK( g_aucLog2[partitionSize] + g_aucLog2[nonSplitDimensionSize] < g_aucLog2[minNumberOfSamplesPerCu], "A partition has less than the minimum amount of samples!" );
  return partitionSize;
}


PUTraverser CU::traversePUs( CodingUnit& cu )
{
  return PUTraverser( cu.firstPU, cu.lastPU->next );
}

TUTraverser CU::traverseTUs( CodingUnit& cu )
{
  return TUTraverser( cu.firstTU, cu.lastTU->next );
}

cPUTraverser CU::traversePUs( const CodingUnit& cu )
{
  return cPUTraverser( cu.firstPU, cu.lastPU->next );
}

cTUTraverser CU::traverseTUs( const CodingUnit& cu )
{
  return cTUTraverser( cu.firstTU, cu.lastTU->next );
}

// PU tools

int PU::getIntraMPMs( const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/ )
{
  const int numMPMs = NUM_MOST_PROBABLE_MODES;
#if !JVET_N0185_UNIFIED_MPM
  const int extendRefLine = (channelType == CHANNEL_TYPE_LUMA) ? pu.multiRefIdx : 0;
  const ISPType ispType = isLuma( channelType ) ? ISPType( pu.cu->ispMode ) : NOT_INTRA_SUBPARTITIONS;
  const bool isHorSplit = ispType == HOR_INTRA_SUBPARTITIONS;
#endif
  {
#if JVET_N0217_MATRIX_INTRAPRED
    CHECK(channelType != CHANNEL_TYPE_LUMA, "Not harmonized yet");
#endif
    int numCand      = -1;
    int leftIntraDir = PLANAR_IDX, aboveIntraDir = PLANAR_IDX;

    const CompArea &area = pu.block(getFirstComponentOfChannel(channelType));
    const Position posRT = area.topRight();
    const Position posLB = area.bottomLeft();

    // Get intra direction of left PU
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType);
    if (puLeft && CU::isIntra(*puLeft->cu))
    {
#if JVET_N0217_MATRIX_INTRAPRED
      leftIntraDir = PU::getIntraDirLuma( *puLeft );
#else
      leftIntraDir = puLeft->intraDir[channelType];
#endif
    }

    // Get intra direction of above PU
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType);
    if (puAbove && CU::isIntra(*puAbove->cu) && CU::isSameCtu(*pu.cu, *puAbove->cu))
    {
#if JVET_N0217_MATRIX_INTRAPRED
      aboveIntraDir = PU::getIntraDirLuma( *puAbove );
#else
      aboveIntraDir = puAbove->intraDir[channelType];
#endif
    }

    CHECK(2 >= numMPMs, "Invalid number of most probable modes");

    const int offset = (int)NUM_LUMA_MODE - 6;
    const int mod = offset + 3;

#if !JVET_N0185_UNIFIED_MPM
    if (extendRefLine)
    {
      int modeIdx = 0;
      int angularMode[2] = { 0, 0 };

      if (leftIntraDir > DC_IDX)
      {
        angularMode[modeIdx++] = leftIntraDir;
      }
      if (aboveIntraDir > DC_IDX && aboveIntraDir != leftIntraDir)
      {
        angularMode[modeIdx++] = aboveIntraDir;
      }
      if (modeIdx == 0)
      {
        mpm[0] = VER_IDX;
        mpm[1] = HOR_IDX;
        mpm[2] = 2;
        mpm[3] = DIA_IDX;
        mpm[4] = VDIA_IDX;
        mpm[5] = 26;
      }
      else if (modeIdx == 1)
      {
        mpm[0] = angularMode[0];
        mpm[1] = ((angularMode[0] + offset) % mod) + 2;
        mpm[2] = ((angularMode[0] - 1) % mod) + 2;
        mpm[3] = ((angularMode[0] + offset - 1) % mod) + 2;
        mpm[4] = (angularMode[0] % mod) + 2;
        mpm[5] = ((angularMode[0] + offset - 2) % mod) + 2;
      }
      else
      {
        mpm[0] = angularMode[0];
        mpm[1] = angularMode[1];
        int maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;
        int minCandModeIdx = 1 - maxCandModeIdx;
        if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 1)
        {
          mpm[2] = ((angularMode[minCandModeIdx] + offset) % mod) + 2;
          mpm[3] = ((angularMode[maxCandModeIdx] - 1) % mod) + 2;
          mpm[4] = ((angularMode[minCandModeIdx] + offset - 1) % mod) + 2;
          mpm[5] = ( angularMode[maxCandModeIdx] % mod) + 2;
        }
        else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] >= 62)
        {
          mpm[2] = ((angularMode[minCandModeIdx] - 1) % mod) + 2;
          mpm[3] = ((angularMode[maxCandModeIdx] + offset) % mod) + 2;
          mpm[4] = ((angularMode[minCandModeIdx]) % mod) + 2;
          mpm[5] = ((angularMode[maxCandModeIdx] + offset - 1) % mod) + 2;
        }
        else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 2)
        {
          mpm[2] = ((angularMode[minCandModeIdx] - 1) % mod) + 2;
          mpm[3] = ((angularMode[minCandModeIdx] + offset) % mod) + 2;
          mpm[4] = ((angularMode[maxCandModeIdx] - 1) % mod) + 2;
          mpm[5] = ((angularMode[minCandModeIdx] + offset - 1) % mod) + 2;
        }
        else
        {
          mpm[2] = ((angularMode[minCandModeIdx] + offset) % mod) + 2;
          mpm[3] = ((angularMode[minCandModeIdx] - 1) % mod) + 2;
          mpm[4] = ((angularMode[maxCandModeIdx] + offset) % mod) + 2;
          mpm[5] = ((angularMode[maxCandModeIdx] - 1) % mod) + 2;
        }
      }
    }
    else if( ispType != NOT_INTRA_SUBPARTITIONS )
    {
      //default case
      mpm[0] = PLANAR_IDX;
      if( isHorSplit )
      {
        mpm[1] = HOR_IDX;
        mpm[2] = 25;
        mpm[3] = 10;
        mpm[4] = 65;
        mpm[5] = VER_IDX;
      }
      else
      {
        mpm[1] = VER_IDX;
        mpm[2] = 43;
        mpm[3] = 60;
        mpm[4] = 3;
        mpm[5] = HOR_IDX;
      }
      int canonicalMode = mpm[1];
      if( leftIntraDir == aboveIntraDir ) //L=A
      {
        numCand = 1;
        if( leftIntraDir > DC_IDX )
        {
          mpm[0] =     leftIntraDir;
          mpm[1] = ( ( leftIntraDir + offset ) % mod ) + 2;
          mpm[2] = ( ( leftIntraDir - 1 ) % mod ) + 2;
          if( ( isHorSplit && leftIntraDir < DIA_IDX ) || ( !isHorSplit && leftIntraDir >= DIA_IDX ) )
          {
            mpm[3] = ( ( leftIntraDir + offset - 1 ) % mod ) + 2;
            mpm[4] =   ( leftIntraDir                % mod ) + 2;
            mpm[5] = ( ( leftIntraDir + offset - 2 ) % mod ) + 2;;
          }
          else
          {
            if( isHorSplit )
            {
              mpm[3] = HOR_IDX;
              mpm[4] = 5;
            }
            else
            {
              mpm[3] = VER_IDX;
              mpm[4] = VDIA_IDX - 3;
            }
            mpm[5] = PLANAR_IDX;
          }
        }
      }
      else //L!=A
      {
        numCand = 2;
        if( ( leftIntraDir > DC_IDX ) && ( aboveIntraDir > DC_IDX ) )
        {
          int distLeftToCanonicalMode  = abs( leftIntraDir - canonicalMode );
          int distAboveToCanonicalMode = abs( aboveIntraDir - canonicalMode );
          mpm[0] = aboveIntraDir;
          mpm[1] = leftIntraDir;
          if( distLeftToCanonicalMode <= distAboveToCanonicalMode )
          {
            mpm[0] = leftIntraDir;
            mpm[1] = aboveIntraDir;
          }
          int maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;
          int minCandModeIdx = 1 - maxCandModeIdx;
          if( mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 1 )
          {
            mpm[2] = ( ( mpm[minCandModeIdx] + offset )     % mod ) + 2;
            mpm[3] = ( ( mpm[maxCandModeIdx] - 1 )          % mod ) + 2;
            mpm[4] = ( ( mpm[minCandModeIdx] + offset - 1 ) % mod ) + 2;
            mpm[5] =   ( mpm[maxCandModeIdx]                % mod ) + 2;
          }
          else if( mpm[maxCandModeIdx] - mpm[minCandModeIdx] >= 62 )
          {
            mpm[2] = ( ( mpm[minCandModeIdx] - 1 )          % mod ) + 2;
            mpm[3] = ( ( mpm[maxCandModeIdx] + offset )     % mod ) + 2;
            mpm[4] = ( ( mpm[minCandModeIdx] )              % mod ) + 2;
            mpm[5] = ( ( mpm[maxCandModeIdx] + offset - 1 ) % mod ) + 2;
          }
          else if( mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 2 )
          {
            mpm[2] = ( ( mpm[minCandModeIdx] - 1 )          % mod ) + 2;
            mpm[3] = ( ( mpm[minCandModeIdx] + offset )     % mod ) + 2;
            mpm[4] = ( ( mpm[maxCandModeIdx] - 1 )          % mod ) + 2;
            mpm[5] = ( ( mpm[minCandModeIdx] + offset - 1 ) % mod ) + 2;
          }
          else
          {
            mpm[2] = ( ( mpm[minCandModeIdx] + offset )     % mod ) + 2;
            mpm[3] = ( ( mpm[minCandModeIdx] - 1 )          % mod ) + 2;
            mpm[4] = ( ( mpm[maxCandModeIdx] + offset )     % mod ) + 2;
            mpm[5] = ( ( mpm[maxCandModeIdx] - 1 )          % mod ) + 2;
          }
        }
        else if( leftIntraDir + aboveIntraDir > 2 )
        {
          //mpm[0] = PLANAR_IDX;
          int angMode = leftIntraDir > DC_IDX ? leftIntraDir : aboveIntraDir;
          mpm[1] = angMode;
          mpm[2] = ( ( angMode + offset )     % mod ) + 2;
          mpm[3] = ( ( angMode - 1 )          % mod ) + 2;
          mpm[4] = ( ( angMode + offset - 1 ) % mod ) + 2;
          mpm[5] = ( ( angMode )              % mod ) + 2;
        }
      }
    }
    else
#endif
    {
#if JVET_N0185_UNIFIED_MPM
      mpm[0] = PLANAR_IDX;
      mpm[1] = DC_IDX;
#else
      mpm[0] = leftIntraDir;
      mpm[1] = (mpm[0] == PLANAR_IDX) ? DC_IDX : PLANAR_IDX;
#endif
      mpm[2] = VER_IDX;
      mpm[3] = HOR_IDX;
      mpm[4] = VER_IDX - 4;
      mpm[5] = VER_IDX + 4;

      if (leftIntraDir == aboveIntraDir)
      {
        numCand = 1;
        if (leftIntraDir > DC_IDX)
        {
#if JVET_N0185_UNIFIED_MPM
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = ((leftIntraDir + offset) % mod) + 2;
          mpm[3] = ((leftIntraDir - 1) % mod) + 2;
          mpm[4] = DC_IDX;
          mpm[5] = ((leftIntraDir + offset - 1) % mod) + 2;
#else
          mpm[0] = leftIntraDir;
          mpm[1] = PLANAR_IDX;
          mpm[2] = DC_IDX;
          mpm[3] = ((leftIntraDir + offset) % mod) + 2;
          mpm[4] = ((leftIntraDir - 1) % mod) + 2;
          mpm[5] = ((leftIntraDir + offset - 1) % mod) + 2;
#endif
        }
      }
      else //L!=A
      {
        numCand = 2;
#if !JVET_N0185_UNIFIED_MPM
        mpm[0] = leftIntraDir;
        mpm[1] = aboveIntraDir;
#endif
#if JVET_N0185_UNIFIED_MPM
        int  maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;
#else
        bool maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;
#endif

        if ((leftIntraDir > DC_IDX) && (aboveIntraDir > DC_IDX))
        {
#if JVET_N0185_UNIFIED_MPM
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = aboveIntraDir;
          maxCandModeIdx = mpm[1] > mpm[2] ? 1 : 2;
          int minCandModeIdx = mpm[1] > mpm[2] ? 2 : 1;
#else
          mpm[2] = PLANAR_IDX;
#endif
          mpm[3] = DC_IDX;
#if JVET_N0185_UNIFIED_MPM
          if ((mpm[maxCandModeIdx] - mpm[minCandModeIdx] < 63) && (mpm[maxCandModeIdx] - mpm[minCandModeIdx] > 1))
#else
          if ((mpm[maxCandModeIdx] - mpm[!maxCandModeIdx] < 63) && (mpm[maxCandModeIdx] - mpm[!maxCandModeIdx] > 1))
#endif
          {
            mpm[4] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] - 1) % mod) + 2;
          }
          else
          {
            mpm[4] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx]) % mod) + 2;
          }
        }
        else if (leftIntraDir + aboveIntraDir >= 2)
        {
#if JVET_N0185_UNIFIED_MPM
          mpm[0] = PLANAR_IDX;
          mpm[1] = (leftIntraDir < aboveIntraDir) ? aboveIntraDir : leftIntraDir;
          maxCandModeIdx = 1;
          mpm[2] = DC_IDX;
#else
          mpm[2] = (mpm[!maxCandModeIdx] == PLANAR_IDX) ? DC_IDX : PLANAR_IDX;
#endif
          mpm[3] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] - 1) % mod) + 2;
          mpm[5] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
        }
      }
    }
    for (int i = 0; i < numMPMs; i++)
    {
      CHECK(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
    }
    CHECK(numCand == 0, "No candidates found");
    return numCand;
  }
}

#if JVET_N0217_MATRIX_INTRAPRED
bool PU::isMIP(const PredictionUnit &pu, const ChannelType &chType)
{
  return (chType == CHANNEL_TYPE_LUMA && pu.cu->mipFlag);
}

int PU::getMipSizeId(const PredictionUnit &pu)
{
  if ((pu.lwidth() == 4) && (pu.lheight() == 4))
  {
    return 0; // MIP with 16x4 matrix
  }
  else if (pu.lwidth() <= 8 && pu.lheight() <= 8)
  {
    return 1; // MIP with 16x8 matrix
  }
  else
  {
    return 2; // MIP with 64x8 matrix
  }
}

int PU::getMipMPMs(const PredictionUnit &pu, unsigned *mpm)
{
  const CompArea &area = pu.block( getFirstComponentOfChannel( CHANNEL_TYPE_LUMA ) );
  const Position &pos = area.pos();

  bool realMode = false;

  // Get intra mode of left PU
  int leftIntraMode = -1;
  const PredictionUnit *puLeft = pu.cs->getPURestricted( pos.offset( -1, 0 ), pu, CHANNEL_TYPE_LUMA );

  if( puLeft && CU::isIntra( *puLeft->cu ) )
  {
    if( PU::isMIP( *puLeft ) )
    {
      if (getMipSizeId(*puLeft) == getMipSizeId(pu))
      {
        leftIntraMode = puLeft->intraDir[CHANNEL_TYPE_LUMA];
        realMode = true;
      }
    }
    else
    {
      leftIntraMode = g_mapAngular33ToMip[getMipSizeId(pu)][g_intraMode65to33AngMapping[puLeft->intraDir[CHANNEL_TYPE_LUMA]]];
    }
  }

  // Get intra mode of above PU
  int aboveIntraMode = -1;
  const PredictionUnit *puAbove = pu.cs->getPURestricted( pos.offset( 0, -1 ), pu, CHANNEL_TYPE_LUMA );

  if( puAbove && CU::isIntra( *puAbove->cu ) && CU::isSameCtu(*pu.cu, *puAbove->cu) )
  {
    if( PU::isMIP( *puAbove ) )
    {
      if (getMipSizeId(*puAbove) == getMipSizeId(pu))
      {
        aboveIntraMode = puAbove->intraDir[CHANNEL_TYPE_LUMA];
        realMode = true;
      }
    }
    else
    {
      aboveIntraMode = g_mapAngular33ToMip[getMipSizeId(pu)][g_intraMode65to33AngMapping[puAbove->intraDir[CHANNEL_TYPE_LUMA]]];
    }
  }

  // derive MPMs
  CHECKD(NUM_MPM_MIP != 3, "Error: wrong number of MPMs for MIP");

  const int* modeList = g_sortedMipMpms[getMipSizeId(pu)];

  int numCand = 0;
  if( leftIntraMode == aboveIntraMode )
  {
    if( leftIntraMode > -1 )
    {
      mpm[0] = leftIntraMode;
      numCand = 1;

      if( leftIntraMode != modeList[0] )
      {
        mpm[1] = modeList[0];
        mpm[2] = (leftIntraMode != modeList[1]) ? modeList[1] : modeList[2];
      }
      else
      {
        mpm[1] = modeList[1];
        mpm[2] = modeList[2];
      }
    }
    else
    {
      mpm[0] = modeList[0];
      mpm[1] = modeList[1];
      mpm[2] = modeList[2];
    }
  }
  else
  {
    if( leftIntraMode > -1 && aboveIntraMode > -1 )
    {
      mpm[0] = leftIntraMode;
      mpm[1] = aboveIntraMode;
      numCand = 2;

      int index = 0;
      for( int i = 0; i < 3; i++ )
      {
        if( (leftIntraMode != modeList[i]) && (aboveIntraMode != modeList[i]) )
        {
          index = i;
          break;
        }
      }
      CHECK( index > 2, "Error" );
      mpm[2] = modeList[index];
    }
    else
    {
      mpm[0] = leftIntraMode > -1 ? leftIntraMode : aboveIntraMode;
      numCand = 1;

      if( mpm[0] != modeList[0] )
      {
        mpm[1] = modeList[0];
        mpm[2] = (mpm[0] != modeList[1]) ? modeList[1] : modeList[2];
      }
      else
      {
        mpm[1] = modeList[1];
        mpm[2] = modeList[2];
      }
    }
  }

  return (realMode ? numCand : 0);
}

uint32_t PU::getIntraDirLuma( const PredictionUnit &pu )
{
  if (isMIP(pu))
  {
    return g_mapMipToAngular65[getMipSizeId(pu)][pu.intraDir[CHANNEL_TYPE_LUMA]];
  }
  else
  {
    return pu.intraDir[CHANNEL_TYPE_LUMA];
  }
}

AvailableInfo PU::getAvailableInfoLuma(const PredictionUnit &pu)
{
  const Area puArea = pu.Y();
  const CodingStructure &cs = *pu.cs;
  CHECK(cs.pps->getConstrainedIntraPred(), "Error: constrained intra prediction not supported");

  AvailableInfo availInfo(0, 0);

  // above
  const int unitWidth = cs.pcv->minCUWidth;
  const int numAboveUnits = (puArea.width + (unitWidth - 1)) / unitWidth;
  for (int uX = 0; uX < numAboveUnits; uX++)
  {
    const Position topPos = puArea.offset(availInfo.maxPosTop, -1);
    const CodingUnit* pcCUAbove = cs.isDecomp(topPos, CHANNEL_TYPE_LUMA) ? cs.getCURestricted(topPos, *(pu.cu), CHANNEL_TYPE_LUMA) : nullptr;
    if (!pcCUAbove) { break; }

    availInfo.maxPosTop += unitWidth;
  }

  // left
  const int unitHeight = cs.pcv->minCUHeight;
  const int numLeftUnits = (puArea.height + (unitHeight - 1)) / unitHeight;
  for (int uY = 0; uY < numLeftUnits; uY++)
  {
    const Position leftPos = puArea.offset(-1, availInfo.maxPosLeft);
    const CodingUnit* pcCULeft = cs.isDecomp(leftPos, CHANNEL_TYPE_LUMA) ? cs.getCURestricted(leftPos, *(pu.cu), CHANNEL_TYPE_LUMA) : nullptr;
    if (!pcCULeft) { break; }

    availInfo.maxPosLeft += unitHeight;
  }

  CHECKD(availInfo.maxPosTop > puArea.width || availInfo.maxPosLeft > puArea.height, "Error");
  return availInfo;
}
#endif

void PU::getIntraChromaCandModes( const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE] )
{
  {
    modeList[  0 ] = PLANAR_IDX;
    modeList[  1 ] = VER_IDX;
    modeList[  2 ] = HOR_IDX;
    modeList[  3 ] = DC_IDX;
    modeList[4] = LM_CHROMA_IDX;
    modeList[5] = MDLM_L_IDX;
    modeList[6] = MDLM_T_IDX;
    modeList[7] = DM_CHROMA_IDX;

    Position topLeftPos = pu.blocks[pu.chType].lumaPos();
    Position refPos = topLeftPos.offset( pu.blocks[pu.chType].lumaSize().width >> 1, pu.blocks[pu.chType].lumaSize().height >> 1 );
    const PredictionUnit *lumaPU = CS::isDualITree( *pu.cs ) ? pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : &pu;
#if JVET_N0217_MATRIX_INTRAPRED
    const uint32_t lumaMode = PU::getIntraDirLuma( *lumaPU );
#else
    const uint32_t lumaMode = lumaPU->intraDir[CHANNEL_TYPE_LUMA];
#endif
    for( int i = 0; i < 4; i++ )
    {
      if( lumaMode == modeList[i] )
      {
        modeList[i] = VDIA_IDX;
        break;
      }
    }
  }
}


bool PU::isLMCMode(unsigned mode)
{
  return (mode >= LM_CHROMA_IDX && mode <= MDLM_T_IDX);
}
bool PU::isLMCModeEnabled(const PredictionUnit &pu, unsigned mode)
{
  if ( pu.cs->sps->getUseLMChroma() )
  {
    return true;
  }
  return false;
}

int PU::getLMSymbolList(const PredictionUnit &pu, int *pModeList)
{
  int iIdx = 0;

  pModeList[ iIdx++ ] = LM_CHROMA_IDX;
    pModeList[ iIdx++ ] = -1;
  pModeList[iIdx++] = MDLM_L_IDX;
  pModeList[iIdx++] = MDLM_T_IDX;
  return iIdx;
}



bool PU::isChromaIntraModeCrossCheckMode( const PredictionUnit &pu )
{
  return pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX;
}

#if !JVET_N0302_SIMPLFIED_CIIP
int PU::getMHIntraMPMs(const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/, const bool isChromaMDMS /*= false*/, const unsigned startIdx /*= 0*/)
{
  const int numMPMs = 3; // Multi-hypothesis intra uses only 3 MPM
  {
    int numCand = -1;
    uint32_t leftIntraDir = DC_IDX, aboveIntraDir = DC_IDX;

    const CompArea& area = pu.block(getFirstComponentOfChannel(channelType));
    const Position& pos = area.pos();

    // Get intra direction of left PU
    const PredictionUnit *puLeft = pu.cs->getPURestricted(pos.offset(-1, 0), pu, channelType);

    if (puLeft && (CU::isIntra(*puLeft->cu) || puLeft->mhIntraFlag))
    {
      leftIntraDir = puLeft->intraDir[channelType];

      if (isChroma(channelType) && leftIntraDir == DM_CHROMA_IDX)
      {
        leftIntraDir = puLeft->intraDir[0];
      }
    }

    // Get intra direction of above PU
    const PredictionUnit* puAbove = pu.cs->getPURestricted(pos.offset(0, -1), pu, channelType);

    if (puAbove && (CU::isIntra(*puAbove->cu) || puAbove->mhIntraFlag) && CU::isSameCtu(*pu.cu, *puAbove->cu))
    {
      aboveIntraDir = puAbove->intraDir[channelType];

      if (isChroma(channelType) && aboveIntraDir == DM_CHROMA_IDX)
      {
        aboveIntraDir = puAbove->intraDir[0];
      }
    }

    CHECK(2 >= numMPMs, "Invalid number of most probable modes");

    uint32_t leftIntraDir2 = leftIntraDir;
    uint32_t aboveIntraDir2 = aboveIntraDir;

    leftIntraDir2 = (leftIntraDir2 > DC_IDX) ? ((leftIntraDir2 <= DIA_IDX) ? HOR_IDX : VER_IDX) : leftIntraDir2;
    aboveIntraDir2 = (aboveIntraDir2 > DC_IDX) ? ((aboveIntraDir2 <= DIA_IDX) ? HOR_IDX : VER_IDX) : aboveIntraDir2;

    if (leftIntraDir2 == aboveIntraDir2)
    {
      numCand = 1;

      if (leftIntraDir2 > DC_IDX) // angular modes
      {
        mpm[0] = leftIntraDir2;
        mpm[1] = PLANAR_IDX;
        mpm[2] = DC_IDX;
      }
      else //non-angular
      {
        mpm[0] = PLANAR_IDX;
        mpm[1] = DC_IDX;
        mpm[2] = VER_IDX;
      }
    }
    else
    {
      numCand = 2;

      mpm[0] = leftIntraDir2;
      mpm[1] = aboveIntraDir2;

      if (leftIntraDir2 && aboveIntraDir2) //both modes are non-planar
      {
        mpm[2] = PLANAR_IDX;
      }
      else
      {
        mpm[2] = (leftIntraDir2 + aboveIntraDir2) < 2 ? VER_IDX : DC_IDX;
      }
    }
    int narrowCase = getNarrowShape(pu.lwidth(), pu.lheight());
    if (narrowCase > 0)
    {
      bool isMPM[NUM_LUMA_MODE];
      for (int idx = 0; idx < NUM_LUMA_MODE; idx++)
      {
        isMPM[idx] = false;
      }
      for (int idx = 0; idx < numMPMs; idx++)
      {
        isMPM[mpm[idx]] = true;
      }
      if (narrowCase == 1 && isMPM[HOR_IDX])
      {
        for (int idx = 0; idx < numMPMs; idx++)
        {
          if (mpm[idx] == HOR_IDX)
          {
            if (!isMPM[PLANAR_IDX])
              mpm[idx] = PLANAR_IDX;
            else if (!isMPM[DC_IDX])
              mpm[idx] = DC_IDX;
            else if (!isMPM[VER_IDX])
              mpm[idx] = VER_IDX;
            break;
          }
        }
      }
      if (narrowCase == 2 && isMPM[VER_IDX])
      {
        for (int idx = 0; idx < numMPMs; idx++)
        {
          if (mpm[idx] == VER_IDX)
          {
            if (!isMPM[PLANAR_IDX])
              mpm[idx] = PLANAR_IDX;
            else if (!isMPM[DC_IDX])
              mpm[idx] = DC_IDX;
            else if (!isMPM[HOR_IDX])
              mpm[idx] = HOR_IDX;
            break;
          }
        }
      }
    }
    CHECK(numCand == 0, "No candidates found");
    CHECK(mpm[0] == mpm[1] || mpm[0] == mpm[2] || mpm[2] == mpm[1], "redundant MPM");
    return numCand;
  }
}
#endif
int PU::getNarrowShape(const int width, const int height)
{
  int longSide = (width > height) ? width : height;
  int shortSide = (width > height) ? height : width;
  if (longSide > (2 * shortSide))
  {
    if (longSide == width)
      return 1;
    else
      return 2;
  }
  else
  {
    return 0;
  }
}

uint32_t PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
  uint32_t uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
    Position topLeftPos = pu.blocks[pu.chType].lumaPos();
    Position refPos = topLeftPos.offset( pu.blocks[pu.chType].lumaSize().width >> 1, pu.blocks[pu.chType].lumaSize().height >> 1 );
    const PredictionUnit &lumaPU = CS::isDualITree( *pu.cs ) ? *pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : *pu.cs->getPU( topLeftPos, CHANNEL_TYPE_LUMA );

#if JVET_N0217_MATRIX_INTRAPRED
    uiIntraMode = PU::getIntraDirLuma( lumaPU );
#else
    uiIntraMode = lumaPU.intraDir[0];
#endif
  }
#if JVET_N0671_CHROMA_FORMAT_422
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) && uiIntraMode < NUM_LUMA_MODE ) // map directional, planar and dc
#else
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) )
#endif //JVET_N0671_CHROMA_FORMAT_422
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
  return uiIntraMode;
}

#if JVET_N0193_LFNST
int PU::getWideAngIntraMode( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID )
{
  if( dirMode < 2 )
  {
    return ( int ) dirMode;
  }

  CodingStructure& cs           = *tu.cs;
  const CompArea&  area         = tu.blocks[ compID ];
  PelBuf           pred         = cs.getPredBuf( area );
  int              width        = int( pred.width );
  int              height       = int( pred.height );
  int              modeShift[ ] = { 0, 6, 10, 12, 14, 15 };
  int              deltaSize    = abs( g_aucLog2[ width ] - g_aucLog2[ height ] );
  int              predMode     = dirMode;

  if( width > height && dirMode < 2 + modeShift[ deltaSize ] )
  {
    predMode += ( VDIA_IDX - 1 );
  }
  else if( height > width && predMode > VDIA_IDX - modeShift[ deltaSize ] )
  {
    predMode -= ( VDIA_IDX + 1 );
  }

  return predMode;
}
#endif

bool PU::xCheckSimilarMotion(const int mergeCandIndex, const int prevCnt, const MergeCtx mergeCandList, bool hasPruned[MRG_MAX_NUM_CANDS])
{
  for (uint32_t ui = 0; ui < prevCnt; ui++)
  {
    if (hasPruned[ui])
    {
      continue;
    }
    if (mergeCandList.interDirNeighbours[ui] == mergeCandList.interDirNeighbours[mergeCandIndex])
    {
      if (mergeCandList.interDirNeighbours[ui] == 3)
      {
        int offset0 = (ui * 2);
        int offset1 = (mergeCandIndex * 2);
        if (mergeCandList.mvFieldNeighbours[offset0].refIdx == mergeCandList.mvFieldNeighbours[offset1].refIdx &&
            mergeCandList.mvFieldNeighbours[offset0 + 1].refIdx == mergeCandList.mvFieldNeighbours[offset1 + 1].refIdx &&
            mergeCandList.mvFieldNeighbours[offset0].mv == mergeCandList.mvFieldNeighbours[offset1].mv &&
            mergeCandList.mvFieldNeighbours[offset0 + 1].mv == mergeCandList.mvFieldNeighbours[offset1 + 1].mv
          )
        {
          hasPruned[ui] = true;
          return true;
        }
      }
      else
      {
        int offset0 = (ui * 2) + mergeCandList.interDirNeighbours[ui] - 1;
        int offset1 = (mergeCandIndex * 2) + mergeCandList.interDirNeighbours[ui] - 1;
        if (mergeCandList.mvFieldNeighbours[offset0].refIdx == mergeCandList.mvFieldNeighbours[offset1].refIdx &&
            mergeCandList.mvFieldNeighbours[offset0].mv == mergeCandList.mvFieldNeighbours[offset1].mv
          )
        {
          hasPruned[ui] = true;
          return true;
        }
      }
    }
  }

  return false;
}

#if JVET_L0090_PAIR_AVG

bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
  , bool ibcFlag
  , bool isShared
)
#else

bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, bool isCandInter[MRG_MAX_NUM_CANDS], bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
  , int mmvdList
)
#endif
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;
  bool hasPruned[MRG_MAX_NUM_CANDS];
  memset(hasPruned, 0, MRG_MAX_NUM_CANDS * sizeof(bool));
  if (isAvailableSubPu)
  {
    hasPruned[subPuMvpPos] = true;
  }
#if JVET_N0266_SMALL_BLOCKS
  auto &lut = ibcFlag ? ( isShared ? cs.motionLut.lutShareIbc : cs.motionLut.lutIbc ) : cs.motionLut.lut;
#else
  auto &lut = ibcFlag ? ( isShared ? cs.motionLut.lutShareIbc : cs.motionLut.lutIbc ) : ( isShared ? cs.motionLut.lutShare : cs.motionLut.lut );
#endif
  int num_avai_candInLUT = (int) lut.size();

  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];
    mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
    }
#if JVET_N0843_BVP_SIMPLIFICATION
    if (mrgIdx > 2 || (mrgIdx > 1 && ibcFlag) || !xCheckSimilarMotion(cnt, prevCnt, mrgCtx, hasPruned))
#else
    if (mrgIdx > 2 || !xCheckSimilarMotion(cnt, prevCnt, mrgCtx, hasPruned))
#endif
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? miNeighbor.GBiIdx : GBI_DEFAULT;
      if (mrgCandIdx == cnt && canFastExit)
      {
        return true;
      }
      cnt ++;
      if (cnt  == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }
  return false;
}

void PU::getIBCMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumMergeCand = slice.getMaxNumMergeCand();
  const bool canFastExit = pu.cs->pps->getLog2ParallelMergeLevelMinus2() == 0;

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.GBiIdx[ui] = GBI_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours[ui] = MRG_TYPE_IBC;
    mrgCtx.mvFieldNeighbours[ui * 2].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui * 2 + 1].refIdx = NOT_VALID;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

#if JVET_N0843_BVP_SIMPLIFICATION==0
  const Position posLT = pu.shareParentPos;
#endif
  const Position posRT = pu.shareParentPos.offset(pu.shareParentSize.width - 1, 0);
  const Position posLB = pu.shareParentPos.offset(0, pu.shareParentSize.height - 1);

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  const bool isAvailableA1 = puLeft && isDiffMER(pu, *puLeft) && pu.cu != puLeft->cu && CU::isIBC(*puLeft->cu);
  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
    if (mrgCandIdx == cnt && canFastExit)
    {
      return;
    }
    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }


  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool isAvailableB1 = puAbove && isDiffMER(pu, *puAbove) && pu.cu != puAbove->cu && CU::isIBC(*puAbove->cu);
  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    if (!isAvailableA1 || (miAbove != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Above
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int spatialCandPos = cnt;

#if JVET_N0843_BVP_SIMPLIFICATION==0
  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);
  bool isAvailableB0 = puAboveRight && isDiffMER(pu, *puAboveRight) && CU::isIBC(*puAboveRight->cu);
  if (isAvailableB0)
  {
    miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));

#if HM_JEM_MERGE_CANDS
    if ((!isAvailableB1 || (miAbove != miAboveRight)) && (!isAvailableA1 || (miLeft != miAboveRight)))
#else
    if (!isAvailableB1 || (miAbove != miAboveRight))
#endif
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      // get Mv from Above-right
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);

      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);
  bool isAvailableA0 = puLeftBottom && isDiffMER(pu, *puLeftBottom) && CU::isIBC(*puLeftBottom->cu);
  if (isAvailableA0)
  {
    miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));

#if HM_JEM_MERGE_CANDS
    if ((!isAvailableA1 || (miBelowLeft != miLeft)) && (!isAvailableB1 || (miBelowLeft != miAbove)) && (!isAvailableB0 || (miBelowLeft != miAboveRight)))
#else
    if (!isAvailableA1 || (miBelowLeft != miLeft))
#endif
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above left
  if (cnt < 4)
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);
    bool isAvailableB2 = puAboveLeft && isDiffMER(pu, *puAboveLeft) && CU::isIBC(*puAboveLeft->cu);
    if (isAvailableB2)
    {
      miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));

#if HM_JEM_MERGE_CANDS
      if ((!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)) && (!isAvailableA0 || (miBelowLeft != miAboveLeft)) && (!isAvailableB0 || (miAboveRight != miAboveLeft)))
#else
      if ((!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)))
#endif
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);
        if (mrgCandIdx == cnt && canFastExit)
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }
#endif

#if JVET_N0843_BVP_SIMPLIFICATION
  int maxNumMergeCandMin1 = maxNumMergeCand;
#else
  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
#endif
  if (cnt != maxNumMergeCandMin1)
  {
    bool isAvailableSubPu = false;
    unsigned subPuMvpPos = 0;

    bool  isShared = ((pu.Y().lumaSize().width != pu.shareParentSize.width) || (pu.Y().lumaSize().height != pu.shareParentSize.height));

#if JVET_L0090_PAIR_AVG
    bool bFound = addMergeHMVPCand(cs, mrgCtx, canFastExit
      , mrgCandIdx
      , maxNumMergeCandMin1, cnt
      , spatialCandPos
      , isAvailableSubPu, subPuMvpPos
      , true
      , isShared
    );
#else
    bool bFound = addMergeHMVPCand(slice, mrgCtx, isCandInter, canFastExit
      , mrgCandIdx
      , maxNumMergeCandMin1, cnt, cnt, isAvailableSubPu, subPuMvpPos
    );
#endif
    if (bFound)
    {
      return;
    }
  }

#if JVET_L0090_PAIR_AVG && JVET_N0843_BVP_SIMPLIFICATION==0
  // pairwise-average candidates
    if (cnt>1 && cnt <maxNumMergeCand)
    {
       mrgCtx.mvFieldNeighbours[cnt * 2    ].setMvField(Mv(0, 0), NOT_VALID);
       mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField(Mv(0, 0), NOT_VALID);

       const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2].mv;
       const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2].mv;
       // average two MVs
       Mv avgMv = MvI;

       avgMv += MvJ;
       mrgCtx.mrgTypeNeighbours[cnt] = MRG_TYPE_IBC;
       roundAffineMv(avgMv.hor, avgMv.ver, 1);
       avgMv.roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      mrgCtx.mvFieldNeighbours[cnt * 2 ].setMvField(avgMv, MAX_NUM_REF);
      mrgCtx.interDirNeighbours[cnt] = 1;
      cnt++;
    }

    // early termination
    if (cnt == maxNumMergeCand)
    {
      return;
    }
#endif

#if JVET_N0317_ADD_ZERO_BV
    while (cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), MAX_NUM_REF);
      mrgCtx.interDirNeighbours[cnt] = 1;
      cnt++;
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }
    }
#endif

  mrgCtx.numValidMergeCand = cnt;

}

void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx,
                                 int mmvdList,
                                 const int& mrgCandIdx )
{
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
  const uint32_t maxNumMergeCand = slice.getMaxNumMergeCand();
  const bool canFastExit     = pu.cs->pps->getLog2ParallelMergeLevelMinus2() == 0;

#if !JVET_L0090_PAIR_AVG
  // this variable is unused if remove HEVC combined candidates
  bool isCandInter[MRG_MAX_NUM_CANDS];
#endif

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
#if !JVET_L0090_PAIR_AVG
    isCandInter[ui] = false;
#endif
    mrgCtx.GBiIdx[ui] = GBI_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours [ui] = MRG_TYPE_DEFAULT_N;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

#if JVET_N0266_SMALL_BLOCKS
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
#else
  const Position posLT = pu.shareParentPos;
  const Position posRT = pu.shareParentPos.offset(pu.shareParentSize.width - 1, 0);
  const Position posLB = pu.shareParentPos.offset(0, pu.shareParentSize.height - 1);
#endif
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );

  const bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );

  if( isAvailableA1 )
  {
    miLeft = puLeft->getMotionInfo( posLB.offset(-1, 0) );

#if !JVET_L0090_PAIR_AVG
    isCandInter[cnt] = true;
#endif

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->GBiIdx : GBI_DEFAULT;
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
    }
    if (mrgCandIdx == cnt && canFastExit)
    {
      return;
    }

    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }


  // above
  const PredictionUnit *puAbove = cs.getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );

  bool isAvailableB1 = puAbove && isDiffMER( pu, *puAbove ) && pu.cu != puAbove->cu && CU::isInter( *puAbove->cu );

  if( isAvailableB1 )
  {
    miAbove = puAbove->getMotionInfo( posRT.offset( 0, -1 ) );

    if( !isAvailableA1 || ( miAbove != miLeft ) )
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Above
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->GBiIdx : GBI_DEFAULT;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAbove.mv[0], miAbove.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAbove.mv[1], miAbove.refIdx[1] );
      }
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  int spatialCandPos = cnt;

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );

  bool isAvailableB0 = puAboveRight && isDiffMER( pu, *puAboveRight ) && CU::isInter( *puAboveRight->cu );

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

#if HM_JEM_MERGE_CANDS
    if( ( !isAvailableB1 || ( miAbove != miAboveRight ) ) && ( !isAvailableA1 || ( miLeft != miAboveRight ) ) )
#else
    if( !isAvailableB1 || ( miAbove != miAboveRight ) )
#endif
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      // get Mv from Above-right
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->GBiIdx : GBI_DEFAULT;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
      }

      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );

  bool isAvailableA0 = puLeftBottom && isDiffMER( pu, *puLeftBottom ) && CU::isInter( *puLeftBottom->cu );

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

#if HM_JEM_MERGE_CANDS
    if( ( !isAvailableA1 || ( miBelowLeft != miLeft ) ) && ( !isAvailableB1 || ( miBelowLeft != miAbove ) ) && ( !isAvailableB0 || ( miBelowLeft != miAboveRight ) ) )
#else
    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )
#endif
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->GBiIdx : GBI_DEFAULT;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
      }

      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }


  // above left
  if ( cnt < 4 )
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

    bool isAvailableB2 = puAboveLeft && isDiffMER( pu, *puAboveLeft ) && CU::isInter( *puAboveLeft->cu );

    if( isAvailableB2 )
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

#if HM_JEM_MERGE_CANDS
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) && ( !isAvailableA0 || ( miBelowLeft != miAboveLeft ) ) && ( !isAvailableB0 || ( miAboveRight != miAboveLeft ) ) )
#else
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )
#endif
      {
#if !JVET_L0090_PAIR_AVG
        isCandInter[cnt] = true;
#endif

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->GBiIdx : GBI_DEFAULT;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
        }

        if (mrgCandIdx == cnt && canFastExit)
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

#if JVET_N0213_TMVP_REMOVAL
  if (slice.getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
#else
  if (slice.getEnableTMVPFlag())
#endif
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
#if JVET_N0266_SMALL_BLOCKS
    Position posRB = pu.Y().bottomRight().offset( -3, -3 );
#else
    Position posRB = pu.shareParentPos.offset(pu.shareParentSize.width-3, pu.shareParentSize.height - 3);
#endif
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
#if JVET_N0266_SMALL_BLOCKS
    Position posC1 = pu.Y().center();
#else
    Position posC1 = pu.shareParentPos.offset((pu.shareParentSize.width/2), (pu.shareParentSize.height/2));
#endif
    bool C0Avail = false;
#if !JVET_N0266_SMALL_BLOCKS
    bool C1Avail = (posC1.x < pcv.lumaWidth) && (posC1.y  < pcv.lumaHeight);
#endif
    if (((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight))
    {
      {
        Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

        if( ( posInCtu.x + 4 < pcv.maxCUWidth ) &&           // is not at the last column of CTU
            ( posInCtu.y + 4 < pcv.maxCUHeight ) )           // is not at the last row    of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else if( posInCtu.x + 4 < pcv.maxCUWidth )           // is not at the last column of CTU But is last row of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        }
        else if( posInCtu.y + 4 < pcv.maxCUHeight )          // is not at the last row of CTU But is last column of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else //is the right bottom corner of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // same as for last column but not last row
        }
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx ) )
#if JVET_N0266_SMALL_BLOCKS
                              || getColocatedMVP( pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx );
#else
                                      || ( C1Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx ));
#endif
    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx ) )
#if JVET_N0266_SMALL_BLOCKS
                   || getColocatedMVP( pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx );
#else
                           || (C1Avail &&  getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx ) );
#endif
      if (bExistMV)
      {
        dir     |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if( dir != 0 )
    {
      bool addTMvp = true;
#if HM_JEM_MERGE_CANDS
      int iSpanCand = cnt;
      for( int i = 0; i < iSpanCand; i++ )
      {
        if( mrgCtx.interDirNeighbours[  i           ] == dir &&
            mrgCtx.mvFieldNeighbours [  i << 1      ] == mrgCtx.mvFieldNeighbours[  uiArrayAddr << 1      ] &&
            mrgCtx.mvFieldNeighbours [( i << 1 ) + 1] == mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 ) + 1] )
        {
          addTMvp = false;
        }
      }
#endif
      if( addTMvp )
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
#if !JVET_L0090_PAIR_AVG
        isCandInter              [uiArrayAddr] = true;
#endif
        mrgCtx.GBiIdx[uiArrayAddr] = GBI_DEFAULT;
        if (mrgCandIdx == cnt && canFastExit)
        {
          return;
        }

        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
  if (cnt != maxNumMergeCandMin1)
  {
    bool isAvailableSubPu = false;
    unsigned subPuMvpPos = 0;
#if JVET_L0090_PAIR_AVG
#if JVET_N0266_SMALL_BLOCKS
    bool isShared = false;
#else
    bool  isShared = ((pu.Y().lumaSize().width != pu.shareParentSize.width) || (pu.Y().lumaSize().height != pu.shareParentSize.height));
#endif
    bool bFound = addMergeHMVPCand(cs, mrgCtx, canFastExit
      , mrgCandIdx
      , maxNumMergeCandMin1, cnt
      , spatialCandPos
      , isAvailableSubPu, subPuMvpPos
      , CU::isIBC(*pu.cu)
      , isShared
    );
#else
    bool bFound = addMergeHMVPCand(slice, mrgCtx, isCandInter, canFastExit
      , (mmvdList != 0 && mrgCandIdx != -1) ? (const int)mrgCandIdxIBC : mrgCandIdx
      , maxNumMergeCandMin1, cnt, cnt, isAvailableSubPu, subPuMvpPos
      , mmvdList
    );
#endif
    if (bFound)
    {
      return;
    }
  }

#if JVET_L0090_PAIR_AVG
  // pairwise-average candidates
  {

    if (cnt > 1 && cnt < maxNumMergeCand)
    {

      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField( Mv( 0, 0 ), NOT_VALID );
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField( Mv( 0, 0 ), NOT_VALID );
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;


      for( int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++ )
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

        // both MVs are invalid, skip
        if( (refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID) )
        {
          continue;
        }

        interDir += 1 << refListId;
        // both MVs are valid, average these two MVs
        if( (refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID) )
        {
          const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

          // average two MVs
          Mv avgMv = MvI;
          avgMv += MvJ;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);

          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( avgMv, refIdxI );
        }
        // only one MV is valid, take the only one MV
        else if( refIdxI != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxI );
        }
        else if( refIdxJ != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxJ );
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if( interDir > 0 )
      {
        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }
#endif

  uint32_t uiArrayAddr = cnt;
#if !JVET_L0090_PAIR_AVG
  uint32_t uiCutoff    = std::min( uiArrayAddr, 3u );
  if (slice.isInterB())
  {
    static const uint32_t NUM_PRIORITY_LIST = 12;
    static const uint32_t uiPriorityList0[NUM_PRIORITY_LIST] = { 0 , 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3 };
    static const uint32_t uiPriorityList1[NUM_PRIORITY_LIST] = { 1 , 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2 };

    for (int idx = 0; idx < uiCutoff * (uiCutoff - 1) && uiArrayAddr != maxNumMergeCand; idx++)
    {
      CHECK( idx >= NUM_PRIORITY_LIST, "Invalid priority list number" );
      int i = uiPriorityList0[idx];
      int j = uiPriorityList1[idx];
      if (isCandInter[i] && isCandInter[j] && (mrgCtx.interDirNeighbours[i] & 0x1) && (mrgCtx.interDirNeighbours[j] & 0x2))
      {
        isCandInter[uiArrayAddr] = true;
        mrgCtx.interDirNeighbours[uiArrayAddr] = 3;
        mrgCtx.GBiIdx[uiArrayAddr] = ((mrgCtx.interDirNeighbours[uiArrayAddr] == 3)) ? CU::deriveGbiIdx(mrgCtx.GBiIdx[i], mrgCtx.GBiIdx[j]) : GBI_DEFAULT;

        // get Mv from cand[i] and cand[j]
        mrgCtx.mvFieldNeighbours[ uiArrayAddr << 1     ].setMvField(mrgCtx.mvFieldNeighbours[ i << 1     ].mv, mrgCtx.mvFieldNeighbours[ i << 1     ].refIdx);
        mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].setMvField(mrgCtx.mvFieldNeighbours[(j << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(j << 1) + 1].refIdx);

        int iRefPOCL0 = slice.getRefPOC(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1)    ].refIdx);
        int iRefPOCL1 = slice.getRefPOC(REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].refIdx);

        if( iRefPOCL0 == iRefPOCL1 && mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 )].mv == mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 ) + 1].mv )
        {
          isCandInter[uiArrayAddr] = false;
        }
        else
        {
          uiArrayAddr++;
        }
      }
    }
  }

  // early termination
  if (uiArrayAddr == maxNumMergeCand)
  {
    return;
  }
#endif

  int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);

  int r = 0;
  int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)
  {
#if !JVET_L0090_PAIR_AVG
    isCandInter               [uiArrayAddr     ] = true;
#endif
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.GBiIdx             [uiArrayAddr     ] = GBI_DEFAULT;
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    if ( mrgCtx.interDirNeighbours[uiArrayAddr] == 1 && pu.cs->slice->getRefPic(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[uiArrayAddr << 1].refIdx)->getPOC() == pu.cs->slice->getPOC())
    {
      mrgCtx.mrgTypeNeighbours[uiArrayAddr] = MRG_TYPE_IBC;
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  mrgCtx.numValidMergeCand = uiArrayAddr;
}
bool PU::checkDMVRCondition(const PredictionUnit& pu)
{
#if JVET_N0146_DMVR_BDOF_CONDITION
    WPScalingParam *wp0;
    WPScalingParam *wp1;
    int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    int refIdx1 = pu.refIdx[REF_PIC_LIST_1];
    pu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0, wp0);
    pu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1, wp1);
#endif
  if (pu.cs->sps->getUseDMVR())
  {
    return pu.mergeFlag
      && pu.mergeType == MRG_TYPE_DEFAULT_N
      && !pu.cu->affine
      && !pu.mmvdMergeFlag
      && !pu.cu->mmvdSkip
      && PU::isBiPredFromDifferentDirEqDistPoc(pu)
      && (pu.lheight() >= 8)
#if JVET_N0407_DMVR_CU_SIZE_RESTRICTION
      && (pu.lwidth() >= 8)
      && ((pu.lheight() * pu.lwidth()) >= 128)
#else
      && ((pu.lheight() * pu.lwidth()) >= 64)
#endif
#if JVET_N0146_DMVR_BDOF_CONDITION
      && (pu.cu->GBiIdx == GBI_DEFAULT)
      && ((!wp0[COMPONENT_Y].bPresentFlag) && (!wp1[COMPONENT_Y].bPresentFlag))
#endif
      ;
  }
  else
  {
    return false;
  }
}
// for ibc pu validation
bool PU::isBlockVectorValid(PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xStartInCU, int yStartInCU, int xBv, int yBv, int ctuSize)
{
  const int ctuSizeLog2 = g_aucLog2[ctuSize];

  int refRightX = xPos + xBv + width - 1;
  int refBottomY = yPos + yBv + height - 1;

  int refLeftX = xPos + xBv;
  int refTopY = yPos + yBv;

  if ((xPos + xBv) < 0)
  {
    return false;
  }
  if (refRightX >= picWidth)
  {
    return false;
  }

  if ((yPos + yBv) < 0)
  {
    return false;
  }
  if (refBottomY >= picHeight)
  {
    return false;
  }
  if ((xBv + width) > 0 && (yBv + height) > 0)
  {
    return false;
  }

  // cannot be in the above CTU row
  if (refTopY >> ctuSizeLog2 < yPos >> ctuSizeLog2)
    return false;

  // cannot be in the below CTU row
  if (refBottomY >> ctuSizeLog2 > yPos >> ctuSizeLog2)
  {
    return false;
  }

  // in the same CTU line
#if JVET_N0175_N0251_N0384_IBC_SMALL_CTU
  int numLeftCTUs = (1 << ((7 - ctuSizeLog2) << 1)) - ((ctuSizeLog2 < 7) ? 1 : 0);
  if ((refRightX >> ctuSizeLog2 <= xPos >> ctuSizeLog2) && (refLeftX >> ctuSizeLog2 >= (xPos >> ctuSizeLog2) - numLeftCTUs))
#else
  if ((refRightX >> ctuSizeLog2 <= xPos >> ctuSizeLog2) && (refLeftX >> ctuSizeLog2 >= (xPos >> ctuSizeLog2) - 1))
#endif
  {

    // in the same CTU, or left CTU
    // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
#if JVET_N0175_N0251_N0384_IBC_SMALL_CTU
    if (((refLeftX >> ctuSizeLog2) == ((xPos >> ctuSizeLog2) - 1)) && (ctuSizeLog2 == 7))
#else
    if ((refLeftX >> ctuSizeLog2) == ((xPos >> ctuSizeLog2) - 1))
#endif
    {
      // ref block's collocated block in current CTU
      const Position refPosCol = pu.Y().topLeft().offset(xBv + ctuSize, yBv);
      int offset64x = (refPosCol.x >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      int offset64y = (refPosCol.y >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      const Position refPosCol64x64 = {offset64x, offset64y};
      if (pu.cs->isDecomp(refPosCol64x64, toChannelType(COMPONENT_Y)))
        return false;
#if JVET_N0383_N0251_IBC_COL_VPDU_REMOVE
      if (refPosCol64x64 == pu.Y().topLeft())
        return false;
#endif
    }
  }
  else
    return false;

  // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
  const Position refPosLT = pu.Y().topLeft().offset(xBv, yBv);
  const Position refPosBR = pu.Y().bottomRight().offset(xBv, yBv);
  const ChannelType      chType = toChannelType(COMPONENT_Y);
  if (!pu.cs->isDecomp(refPosBR, chType))
    return false;
  if (!pu.cs->isDecomp(refPosLT, chType))
    return false;
  return true;

}// for ibc pu validation

static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  int iDiffPocD = iColPOC - iColRefPOC;
  int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    int iTDB = Clip3(-128, 127, iDiffPocB);
    int iTDD = Clip3(-128, 127, iDiffPocD);
    int iX = (0x4000 + abs(iTDD / 2)) / iTDD;
    int iScale = Clip3(-4096, 4095, (iTDB * iX + 32) >> 6);
    return iScale;
  }
}

int convertMvFixedToFloat(int32_t val)
{
  int sign  = val >> 31;
  int scale = floorLog2((val ^ sign) | MV_MANTISSA_UPPER_LIMIT) - (MV_MANTISSA_BITCOUNT - 1);

  int exponent;
  int mantissa;
  if (scale >= 0)
  {
    int round = (1 << scale) >> 1;
    int n     = (val + round) >> scale;
    exponent  = scale + ((n ^ sign) >> (MV_MANTISSA_BITCOUNT - 1));
    mantissa  = (n & MV_MANTISSA_UPPER_LIMIT) | (sign << (MV_MANTISSA_BITCOUNT - 1));
  }
  else
  {
    exponent = 0;
    mantissa = val;
  }

  return exponent | (mantissa << MV_EXPONENT_BITCOUNT);
}

int convertMvFloatToFixed(int val)
{
  int exponent = val & MV_EXPONENT_MASK;
  int mantissa = val >> MV_EXPONENT_BITCOUNT;
  return exponent == 0 ? mantissa : (mantissa ^ MV_MANTISSA_LIMIT) << (exponent - 1);
}

int roundMvComp(int x)
{
  return convertMvFloatToFixed(convertMvFixedToFloat(x));
}

int PU::getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC)
{
  return xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);
}

void PU::getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  int refIdxList0, refIdxList1;
  int k;
  int currBaseNum = 0;
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;

  for (k = 0; k < maxNumMergeCand; k++)
  {
    if (mrgCtx.mrgTypeNeighbours[k] == MRG_TYPE_DEFAULT_N)
    {
      refIdxList0 = mrgCtx.mvFieldNeighbours[(k << 1)].refIdx;
      refIdxList1 = mrgCtx.mvFieldNeighbours[(k << 1) + 1].refIdx;

      if ((refIdxList0 >= 0) && (refIdxList1 >= 0))
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      else if (refIdxList0 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = MvField(Mv(0, 0), -1);
      }
      else if (refIdxList1 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = MvField(Mv(0, 0), -1);
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }

      currBaseNum++;

      if (currBaseNum == MMVD_BASE_MV_NUM)
        break;
    }
  }
#if !JVET_N0448_N0380
  if (currBaseNum < MMVD_BASE_MV_NUM)
  {
    for (k = currBaseNum; k < MMVD_BASE_MV_NUM; k++)
    {
      mrgCtx.mmvdBaseMv[k][0] = MvField(Mv(0, 0), 0);
      const Slice &slice = *pu.cs->slice;
      mrgCtx.mmvdBaseMv[k][1] = MvField(Mv(0, 0), (slice.isInterB() ? 0 : -1));
      mrgCtx.GBiIdx[k] = GBI_DEFAULT;
      mrgCtx.interDirNeighbours[k] = (mrgCtx.mmvdBaseMv[k][0].refIdx >= 0) + (mrgCtx.mmvdBaseMv[k][1].refIdx >= 0) * 2;
    }
  }
#endif
}
bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv, const int &refIdx )
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~( scale - 1 );

  const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

  const Slice &slice = *pu.cs->slice;

  // use coldir.
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

  if( !pColPic )
  {
    return false;
  }

  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());

  const MotionInfo& mi = pColPic->cs->getMotionInfo( pos );

  if( !mi.isInter )
  {
    return false;
  }
  if (mi.isIBCmot)
  {
    return false;
  }
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (iColRefIdx < 0)
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = mi.refIdx[eColRefPicList];

    if (iColRefIdx < 0)
    {
      return false;
    }
  }

  const Slice *pColSlice = nullptr;

  for( const auto s : pColPic->slices )
  {
    if( s->getIndependentSliceIdx() == mi.sliceIdx )
    {
      pColSlice = s;
      break;
    }
  }

  CHECK( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;

  const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->longTerm;
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];
  cColMv.setHor(roundMvComp(cColMv.getHor()));
  cColMv.setVer(roundMvComp(cColMv.getVer()));

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
  }
  else
  {
    const int currPOC    = slice.getPOC();
    const int colPOC     = colSlice.getPOC();
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->getPOC();
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
    }
    else
    {
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER(const PredictionUnit &pu1, const PredictionUnit &pu2)
{
  const unsigned xN = pu1.lumaPos().x;
  const unsigned yN = pu1.lumaPos().y;
  const unsigned xP = pu2.lumaPos().x;
  const unsigned yP = pu2.lumaPos().y;

  unsigned plevel = pu1.cs->pps->getLog2ParallelMergeLevelMinus2() + 2;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

#if JVET_N0329_IBC_SEARCH_IMP
bool PU::isAddNeighborMv(const Mv& currMv, Mv* neighborMvs, int numNeighborMv)
{
  bool existed = false;
  for (uint32_t cand = 0; cand < numNeighborMv && !existed; cand++)
  {
    if (currMv == neighborMvs[cand])
    {
      existed = true;
    }
  }

  if (!existed)
  {
    return true;
  }
  else
  {
    return false;
  }
}
#endif

#if JVET_N0329_IBC_SEARCH_IMP
void PU::getIbcMVPsEncOnly(PredictionUnit &pu, Mv* mvPred, int& nbPred)
#else
void PU::getIbcMVPsEncOnly(PredictionUnit &pu, Mv* MvPred, int& nbPred)
#endif
{
#if JVET_N0329_IBC_SEARCH_IMP
  const PreCalcValues   &pcv = *pu.cs->pcv;
  const int  cuWidth = pu.blocks[COMPONENT_Y].width;
  const int  cuHeight = pu.blocks[COMPONENT_Y].height;
  const int  log2UnitWidth = g_aucLog2[pcv.minCUWidth];
  const int  log2UnitHeight = g_aucLog2[pcv.minCUHeight];
  const int  totalAboveUnits = (cuWidth >> log2UnitWidth) + 1;
  const int  totalLeftUnits = (cuHeight >> log2UnitHeight) + 1;

  nbPred = 0;
  Position posLT = pu.Y().topLeft();

  // above-left
  const PredictionUnit *aboveLeftPU = pu.cs->getPURestricted(posLT.offset(-1, -1), pu, CHANNEL_TYPE_LUMA);
  if (aboveLeftPU && CU::isIBC(*aboveLeftPU->cu))
  {
    if (isAddNeighborMv(aboveLeftPU->bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = aboveLeftPU->bv;
    }
  }

  // above neighbors
  for (uint32_t dx = 0; dx < totalAboveUnits && nbPred < IBC_NUM_CANDIDATES; dx++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset((dx << log2UnitWidth), -1), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  // left neighbors
  for (uint32_t dy = 0; dy < totalLeftUnits && nbPred < IBC_NUM_CANDIDATES; dy++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset(-1, (dy << log2UnitHeight)), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  size_t numAvaiCandInLUT = pu.cs->motionLut.lutIbc.size();
  for (uint32_t cand = 0; cand < numAvaiCandInLUT && nbPred < IBC_NUM_CANDIDATES; cand++)
  {
    MotionInfo neibMi = pu.cs->motionLut.lutIbc[cand];
    if (isAddNeighborMv(neibMi.bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = neibMi.bv;
    }
  }

  bool isBvCandDerived[IBC_NUM_CANDIDATES];
  ::memset(isBvCandDerived, false, IBC_NUM_CANDIDATES);

  int curNbPred = nbPred;
  if (curNbPred < IBC_NUM_CANDIDATES)
  {
    do
    {
      curNbPred = nbPred;
      for (uint32_t idx = 0; idx < curNbPred && nbPred < IBC_NUM_CANDIDATES; idx++)
      {
        if (!isBvCandDerived[idx])
        {
          Mv derivedBv;
          if (getDerivedBV(pu, mvPred[idx], derivedBv))
          {
            if (isAddNeighborMv(derivedBv, mvPred, nbPred))
            {
              mvPred[nbPred++] = derivedBv;
            }
          }
          isBvCandDerived[idx] = true;
        }
      }
    } while (nbPred > curNbPred && nbPred < IBC_NUM_CANDIDATES);
  }
#else
  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  unsigned int left = 0, above = 0;

  //left
  const PredictionUnit *neibLeftPU = NULL;
  neibLeftPU = pu.cs->getPURestricted(posLB.offset(-1, 0), pu, CHANNEL_TYPE_LUMA);
  left = (neibLeftPU) ? CU::isIBC(*neibLeftPU->cu) : 0;

  if (left)
  {
    MvPred[nbPred++] = neibLeftPU->bv;
    if (getDerivedBV(pu, MvPred[nbPred - 1], MvPred[nbPred]))
      nbPred++;
  }

  //above
  const PredictionUnit *neibAbovePU = NULL;
  neibAbovePU = pu.cs->getPURestricted(posRT.offset(0, -1), pu, CHANNEL_TYPE_LUMA);
  above = (neibAbovePU) ? CU::isIBC(*neibAbovePU->cu) : 0;

  if (above)
  {
    MvPred[nbPred++] = neibAbovePU->bv;
    if (getDerivedBV(pu, MvPred[nbPred - 1], MvPred[nbPred]))
      nbPred++;
  }

  // Below Left predictor search
  const PredictionUnit *neibBelowLeftPU = NULL;
  neibBelowLeftPU = pu.cs->getPURestricted(posLB.offset(-1, 1), pu, CHANNEL_TYPE_LUMA);
  unsigned int belowLeft = (neibBelowLeftPU) ? CU::isIBC(*neibBelowLeftPU->cu) : 0;

  if (belowLeft)
  {
    MvPred[nbPred++] = neibBelowLeftPU->bv;
    if (getDerivedBV(pu, MvPred[nbPred - 1], MvPred[nbPred]))
      nbPred++;
  }


  // Above Right predictor search
  const PredictionUnit *neibAboveRightPU = NULL;
  neibAboveRightPU = pu.cs->getPURestricted(posRT.offset(1, -1), pu, CHANNEL_TYPE_LUMA);
  unsigned int aboveRight = (neibAboveRightPU) ? CU::isIBC(*neibAboveRightPU->cu) : 0;

  if (aboveRight)
  {
    MvPred[nbPred++] = neibAboveRightPU->bv;
    if (getDerivedBV(pu, MvPred[nbPred - 1], MvPred[nbPred]))
      nbPred++;
  }


  // Above Left predictor search
  const PredictionUnit *neibAboveLeftPU = NULL;
  neibAboveLeftPU = pu.cs->getPURestricted(posLT.offset(-1, -1), pu, CHANNEL_TYPE_LUMA);
  unsigned int aboveLeft = (neibAboveLeftPU) ? CU::isIBC(*neibAboveLeftPU->cu) : 0;

  if (aboveLeft)
  {
    MvPred[nbPred++] = neibAboveLeftPU->bv;
    if (getDerivedBV(pu, MvPred[nbPred - 1], MvPred[nbPred]))
      nbPred++;
  }
#endif
}

bool PU::getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv)
{
  int   cuPelX = pu.lumaPos().x;
  int   cuPelY = pu.lumaPos().y;
  int rX = cuPelX + currentMv.getHor();
  int rY = cuPelY + currentMv.getVer();
  int offsetX = currentMv.getHor();
  int offsetY = currentMv.getVer();


  if (rX < 0 || rY < 0 || rX >= pu.cs->slice->getSPS()->getPicWidthInLumaSamples() || rY >= pu.cs->slice->getSPS()->getPicHeightInLumaSamples())
  {
    return false;
  }

  const PredictionUnit *neibRefPU = NULL;
  neibRefPU = pu.cs->getPURestricted(pu.lumaPos().offset(offsetX, offsetY), pu, CHANNEL_TYPE_LUMA);

  bool isIBC = (neibRefPU) ? CU::isIBC(*neibRefPU->cu) : 0;
  if (isIBC)
  {
    derivedMv = neibRefPU->bv;
    derivedMv += currentMv;
  }
  return isIBC;
}

/**
 * Constructs a list of candidates for IBC AMVP (See specification, section "Derivation process for motion vector predictor candidates")
 */
void PU::fillIBCMvpCand(PredictionUnit &pu, AMVPInfo &amvpInfo)
{
#if JVET_N0843_BVP_SIMPLIFICATION==0
  CodingStructure &cs = *pu.cs;
#endif

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

#if JVET_N0843_BVP_SIMPLIFICATION==0
  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  bool isScaledFlagLX = false; /// variable name from specification; true when the PUs below left or left are available (availableA0 || availableA1).

  const PredictionUnit* tmpPU = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType); // getPUBelowLeft(idx, partIdxLB);
  isScaledFlagLX = tmpPU != NULL && CU::isIBC(*tmpPU->cu);
  if (!isScaledFlagLX)
  {
    tmpPU = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
    isScaledFlagLX = tmpPU != NULL && CU::isIBC(*tmpPU->cu);
  }

  // Left predictor search
  if (isScaledFlagLX)
  {
    bool isAdded = addIBCMVPCand(pu, posLB, MD_BELOW_LEFT, *pInfo);

    if (!isAdded)
    {
      isAdded = addIBCMVPCand(pu, posLB, MD_LEFT, *pInfo);
    }
  }

  // Above predictor search
  bool isAdded = addIBCMVPCand(pu, posRT, MD_ABOVE_RIGHT, *pInfo);

  if (!isAdded)
  {
    isAdded = addIBCMVPCand(pu, posRT, MD_ABOVE, *pInfo);

    if (!isAdded)
    {
      addIBCMVPCand(pu, posLT, MD_ABOVE_LEFT, *pInfo);
    }
  }

  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }

  if (pInfo->numCand == 2)
  {
    if (pInfo->mvCand[0] == pInfo->mvCand[1])
    {
      pInfo->numCand = 1;
    }
  }

  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    addAMVPHMVPCand(pu, REF_PIC_LIST_0, REF_PIC_LIST_1, cs.slice->getPOC(), *pInfo, pu.cu->imv);
  }

  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv(0, 0);
    pInfo->numCand++;
  }
#endif

#if JVET_N0843_BVP_SIMPLIFICATION
  MergeCtx mergeCtx;
  PU::getIBCMergeCandidates(pu, mergeCtx, AMVP_MAX_NUM_CANDS - 1);
  int candIdx = 0;
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;;
    pInfo->numCand++;
    candIdx++;
  }
#endif

  for (Mv &mv : pInfo->mvCand)
  {
#if JVET_N0843_BVP_SIMPLIFICATION
    mv.roundIbcPrecInternal2Amvr(pu.cu->imv);
#endif
  }
}


/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo)
{
  CodingStructure &cs = *pu.cs;

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  bool isScaledFlagLX = false; /// variable name from specification; true when the PUs below left or left are available (availableA0 || availableA1).

  {
    const PredictionUnit* tmpPU = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType ); // getPUBelowLeft(idx, partIdxLB);
    isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );

    if( !isScaledFlagLX )
    {
      tmpPU = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
      isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );
    }
  }

  // Left predictor search
  if( isScaledFlagLX )
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );

      if( !bAdded )
      {
        bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

        if( !bAdded )
        {
          addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );
        }
      }
    }
  }

  // Above predictor search
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

  if( !isScaledFlagLX )
  {
    bool bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandWithScaling( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

#if JVET_N0213_TMVP_REMOVAL
  if (cs.slice->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && (pu.lumaSize().width + pu.lumaSize().height > 12))
#else
  if( cs.slice->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS )
#endif
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool C0Avail = false;
    Position posC1 = pu.Y().center();
#if !JVET_N0266_SMALL_BLOCKS
    bool C1Avail =  ( posC1.x  < pcv.lumaWidth ) && ( posC1.y < pcv.lumaHeight ) ;
#endif
    Mv cColMv;

    if( ( ( posRB.x + pcv.minCUWidth ) < pcv.lumaWidth ) && ( ( posRB.y + pcv.minCUHeight ) < pcv.lumaHeight ) )
    {
      Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

      if ((posInCtu.x + 4 < pcv.maxCUWidth) &&           // is not at the last column of CTU
          (posInCtu.y + 4 < pcv.maxCUHeight))             // is not at the last row    of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else if (posInCtu.x + 4 < pcv.maxCUWidth)           // is not at the last column of CTU But is last row of CTU
      {
        // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        posC0 = posRB.offset(4, 4);
      }
      else if (posInCtu.y + 4 < pcv.maxCUHeight)          // is not at the last row of CTU But is last column of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else //is the right bottom corner of CTU
      {
        // same as for last column but not last row
        posC0 = posRB.offset(4, 4);
      }
    }
#if JVET_N0266_SMALL_BLOCKS
    if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdx_Col ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdx_Col ) )
#else
    if ((C0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdx_Col)) || (C1Avail && getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdx_Col)))
#endif
    {
      cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }

  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    const int        currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
    const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;
    addAMVPHMVPCand(pu, eRefPicList, eRefPicList2nd, currRefPOC, *pInfo, pu.cu->imv);
  }

  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
    pInfo->numCand++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundTransPrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo )
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  switch ( dir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1, 0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset( 0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset( 1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1, 1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if ( neibPU == NULL || !CU::isInter( *neibPU->cu ) || !neibPU->cu->affine
    || neibPU->mergeType != MRG_TYPE_DEFAULT_N
    )
  {
    return false;
  }

  Mv outputAffineMv[3];
  const MotionInfo& neibMi = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC = cs.slice->getRefPic( refPicList, refIdx )->getPOC();
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for ( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

    if ( ((neibPU->interDir & (eRefPicListIndex + 1)) == 0) || pu.cu->slice->getRefPOC( eRefPicListIndex, neibRefIdx ) != currRefPOC )
    {
      continue;
    }

    xInheritedAffineMv( pu, neibPU, eRefPicListIndex, outputAffineMv );
    outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
    outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    }
    affiAMVPInfo.numCand++;
    return true;
  }

  return false;
}

void PU::xInheritedAffineMv( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;

  Mv mvLT, mvRT, mvLB;
  mvLT = puNeighbour->mvAffi[eRefPicList][0];
  mvRT = puNeighbour->mvAffi[eRefPicList][1];
  mvLB = puNeighbour->mvAffi[eRefPicList][2];

  bool isTopCtuBoundary = false;
  if ( (posNeiY + neiH) % pu.cs->sps->getCTUSize() == 0 && (posNeiY + neiH) == posCurY )
  {
    // use bottom-left and bottom-right sub-block MVs for inheritance
    const Position posRB = puNeighbour->Y().bottomRight();
    const Position posLB = puNeighbour->Y().bottomLeft();
    mvLT = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];
    mvRT = puNeighbour->getMotionInfo( posRB ).mv[eRefPicList];
    posNeiY += neiH;
    isTopCtuBoundary = true;
  }

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - g_aucLog2[neiW]);
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - g_aucLog2[neiW]);
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM && !isTopCtuBoundary )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - g_aucLog2[neiH]);
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - g_aucLog2[neiH]);
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;
  rcMv[0].clipToStorageBitDepth();

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;
  rcMv[1].clipToStorageBitDepth();

  // v2
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2].hor = horTmp;
    rcMv[2].ver = verTmp;
    rcMv[2].clipToStorageBitDepth();
  }
}


void PU::fillAffineMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }


  // insert inherited affine candidates
  Mv outputAffineMv[3];
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  // check left neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo ) )
  {
    addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo );
  }

  // check above neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo ) )
  {
    if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo ) )
    {
      addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo );
    }
  }

  if ( affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS )
  {
    for (int i = 0; i < affiAMVPInfo.numCand; i++)
    {
      affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    }
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0 );
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0 );
  }
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0 );
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1 );
  if ( amvpInfo1.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2 );
  if ( amvpInfo2.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo2.numCand << 2);

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);

  if ( cornerMVPattern == 7 || (cornerMVPattern == 3 && pu.cu->affineType == AFFINEMODEL_4PARAM) )
  {
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    affiAMVPInfo.numCand++;
  }


  if ( affiAMVPInfo.numCand < 2 )
  {
    // check corner MVs
    for ( int i = 2; i >= 0 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; i-- )
    {
      if ( cornerMVPattern & (1 << i) ) // MV i exist
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.numCand++;
      }
    }

    // Get Temporal Motion Predictor
    if ( affiAMVPInfo.numCand < 2 && pu.cs->slice->getEnableTMVPFlag() )
    {
      const int refIdxCol = refIdx;

      Position posRB = pu.Y().bottomRight().offset( -3, -3 );

      const PreCalcValues& pcv = *pu.cs->pcv;

      Position posC0;
      bool C0Avail = false;
      Position posC1 = pu.Y().center();
#if !JVET_N0266_SMALL_BLOCKS
      bool C1Avail =  ( posC1.x  < pcv.lumaWidth ) && ( posC1.y < pcv.lumaHeight ) ;
#endif
      Mv cColMv;
      if ( ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight) )
      {
        Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

        if ( (posInCtu.x + 4 < pcv.maxCUWidth) &&           // is not at the last column of CTU
          (posInCtu.y + 4 < pcv.maxCUHeight) )             // is not at the last row    of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else if ( posInCtu.x + 4 < pcv.maxCUWidth )           // is not at the last column of CTU But is last row of CTU
        {
          // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
          posC0 = posRB.offset( 4, 4 );
        }
        else if ( posInCtu.y + 4 < pcv.maxCUHeight )          // is not at the last row of CTU But is last column of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else //is the right bottom corner of CTU
        {
          // same as for last column but not last row
          posC0 = posRB.offset( 4, 4 );
        }
      }
#if JVET_N0266_SMALL_BLOCKS
      if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdxCol ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdxCol ) )
#else
      if ( (C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdxCol )) || (C1Avail && getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdxCol ) ) )
#endif
      {
        cColMv.roundAffinePrecInternal2Amvr(pu.cu->imv);
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.numCand++;
      }
    }

    if ( affiAMVPInfo.numCand < 2 )
    {
      // add zero MV
      for ( int i = affiAMVPInfo.numCand; i < AMVP_MAX_NUM_CANDS; i++ )
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.numCand++;
      }
    }
  }

  for (int i = 0; i < affiAMVPInfo.numCand; i++)
  {
    affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addIBCMVPCand(const PredictionUnit &pu, const Position &pos, const MvpDir &eDir, AMVPInfo &info)
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset(-1, 0);
    break;
  case MD_ABOVE:
    neibPos = pos.offset(0, -1);
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(1, -1);
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset(-1, 1);
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset(-1, -1);
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted(neibPos, pu, pu.chType);

  if (neibPU == NULL || CU::isIBC(*neibPU->cu)==false)
  {
    return false;
  }

  const MotionInfo& neibMi = neibPU->getMotionInfo(neibPos);
  info.mvCand[info.numCand++] = neibMi.mv[REF_PIC_LIST_0];
  return true;
}

bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC     = cs.slice->getRefPic( eRefPicList, iRefIdx )->getPOC();
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if( neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
      info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
      return true;
    }
  }

  return false;
}

/**
* \param pInfo
* \param eRefPicList
* \param iRefIdx
* \param uiPartUnitIdx
* \param eDir
* \returns bool
*/
bool PU::addMVPCandWithScaling( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const Slice &slice           = *cs.slice;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch( eDir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if (neibPU == NULL || !CU::isInter(*neibPU->cu) || !CU::isInter(*pu.cu))
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  const int  currPOC            = slice.getPOC();
  const int  currRefPOC         = slice.getRefPic( eRefPicList, iRefIdx )->poc;
  const bool bIsCurrRefLongTerm = slice.getRefPic( eRefPicList, iRefIdx )->longTerm;
  const int  neibPOC            = currPOC;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];
    if( neibRefIdx >= 0 )
    {
      const bool bIsNeibRefLongTerm = slice.getRefPic(eRefPicListIndex, neibRefIdx)->longTerm;

      if (bIsCurrRefLongTerm == bIsNeibRefLongTerm)
      {
        Mv cMv = neibMi.mv[eRefPicListIndex];

        if( !( bIsCurrRefLongTerm /* || bIsNeibRefLongTerm*/) )
        {
          const int neibRefPOC = slice.getRefPOC( eRefPicListIndex, neibRefIdx );
          const int scale      = xGetDistScaleFactor( currPOC, currRefPOC, neibPOC, neibRefPOC );

          if( scale != 4096 )
          {
            cMv = cMv.scaleMv( scale );
          }
        }

        info.mvCand[info.numCand++] = cMv;
        return true;
      }
    }
  }

  return false;
}

void PU::addAMVPHMVPCand(const PredictionUnit &pu, const RefPicList eRefPicList, const RefPicList eRefPicList2nd, const int currRefPOC, AMVPInfo &info, uint8_t imv)
{
  const Slice &slice = *(*pu.cs).slice;

  MotionInfo neibMi;
  auto &lut = CU::isIBC(*pu.cu) ? pu.cs->motionLut.lutIbc : pu.cs->motionLut.lut;
  int num_avai_candInLUT = (int) lut.size();
  int num_allowedCand = std::min(MAX_NUM_HMVP_AVMPCANDS, num_avai_candInLUT);

  for (int mrgIdx = 1; mrgIdx <= num_allowedCand; mrgIdx++)
  {
    if (info.numCand >= AMVP_MAX_NUM_CANDS)
    {
      return;
    }
    neibMi = lut[mrgIdx - 1];

    for (int predictorSource = 0; predictorSource < 2; predictorSource++)
    {
      const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
      const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

      if (neibRefIdx >= 0 && (CU::isIBC(*pu.cu) || (currRefPOC == slice.getRefPOC(eRefPicListIndex, neibRefIdx))))
      {
        Mv pmv = neibMi.mv[eRefPicListIndex];
        pmv.roundTransPrecInternal2Amvr(pu.cu->imv);

        info.mvCand[info.numCand++] = pmv;
        if (info.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return;
        }
      }
    }
  }
}

bool PU::isBipredRestriction(const PredictionUnit &pu)
{
  if(pu.cu->lumaSize().width == 4 && pu.cu->lumaSize().height ==4 )
  {
    return true;
  }
#if JVET_N0266_SMALL_BLOCKS
  /* disable bi-prediction for 4x8/8x4 */
  if ( pu.cu->lumaSize().width + pu.cu->lumaSize().height == 12 )
  {
    return true;
  }
#endif
  return false;
}
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
void PU::getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], int8_t neighGbi[4], bool isAvailable[4], int verIdx[4], int modelIdx, int verNum, AffineMergeCtx& affMrgType)
#else
void PU::getAffineControlPointCand( const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int modelIdx, int verNum, AffineMergeCtx& affMrgType )
#endif
{
  int cuW = pu.Y().width;
  int cuH = pu.Y().height;
  int vx, vy;
  int shift = MAX_CU_DEPTH;
  int shiftHtoW = shift + g_aucLog2[cuW] - g_aucLog2[cuH];

  // motion info
  Mv cMv[2][4];
  int refIdx[2] = { -1, -1 };
  int dir = 0;
#if  JVET_N0481_BCW_CONSTRUCTED_AFFINE
  int8_t gbiIdx = GBI_DEFAULT;
#endif
  EAffineModel curType = (verNum == 2) ? AFFINEMODEL_4PARAM : AFFINEMODEL_6PARAM;

  if ( verNum == 2 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1];
    if ( !isAvailable[idx0] || !isAvailable[idx1] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
    if (dir == 3)
    {
      if (neighGbi[idx0] == neighGbi[idx1])
      {
        gbiIdx = neighGbi[idx0];
      }
    }

#endif
  }
  else if ( verNum == 3 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1], idx2 = verIdx[2];
    if ( !isAvailable[idx0] || !isAvailable[idx1] || !isAvailable[idx2] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 && mi[idx2].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l] && mi[idx0].refIdx[l] == mi[idx2].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
    int gbiClass[5] = { -1,0,0,0,1 };
    if (dir == 3)
    {
      if (neighGbi[idx0] == neighGbi[idx1] && gbiClass[neighGbi[idx0]] == gbiClass[neighGbi[idx2]])
      {
        gbiIdx = neighGbi[idx0];
      }
      else if (neighGbi[idx0] == neighGbi[idx2] && gbiClass[neighGbi[idx0]] == gbiClass[neighGbi[idx1]])
      {
        gbiIdx = neighGbi[idx0];

      }
      else if (neighGbi[idx1] == neighGbi[idx2] && gbiClass[neighGbi[idx0]] == gbiClass[neighGbi[idx1]])
      {
        gbiIdx = neighGbi[idx1];
      }
      else
      {
        gbiIdx = GBI_DEFAULT;
      }

    }

#endif
  }

  if ( dir == 0 )
  {
    return;
  }


  for ( int l = 0; l < 2; l++ )
  {
    if ( dir & (l + 1) )
    {
      for ( int i = 0; i < verNum; i++ )
      {
        cMv[l][verIdx[i]] = mi[verIdx[i]].mv[l];
      }

      // convert to LT, RT[, [LB]]
      switch ( modelIdx )
      {
      case 0: // 0 : LT, RT, LB
        break;

      case 1: // 1 : LT, RT, RB
        cMv[l][2].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][1].hor;
        cMv[l][2].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][1].ver;
#if JVET_N0334_MVCLIPPING
        cMv[l][2].clipToStorageBitDepth();
#endif
        break;

      case 2: // 2 : LT, LB, RB
        cMv[l][1].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][2].hor;
        cMv[l][1].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][2].ver;
#if JVET_N0334_MVCLIPPING
        cMv[l][1].clipToStorageBitDepth();
#endif
        break;

      case 3: // 3 : RT, LB, RB
        cMv[l][0].hor = cMv[l][1].hor + cMv[l][2].hor - cMv[l][3].hor;
        cMv[l][0].ver = cMv[l][1].ver + cMv[l][2].ver - cMv[l][3].ver;
#if JVET_N0334_MVCLIPPING
        cMv[l][0].clipToStorageBitDepth();
#endif
        break;

      case 4: // 4 : LT, RT
        break;

      case 5: // 5 : LT, LB
        vx = (cMv[l][0].hor << shift) + ((cMv[l][2].ver - cMv[l][0].ver) << shiftHtoW);
        vy = (cMv[l][0].ver << shift) - ((cMv[l][2].hor - cMv[l][0].hor) << shiftHtoW);
        roundAffineMv( vx, vy, shift );
        cMv[l][1].set( vx, vy );
#if JVET_N0334_MVCLIPPING
        cMv[l][1].clipToStorageBitDepth();
#endif
        break;

      default:
        CHECK( 1, "Invalid model index!\n" );
        break;
      }
    }
    else
    {
      for ( int i = 0; i < 4; i++ )
      {
        cMv[l][i].hor = 0;
        cMv[l][i].ver = 0;
      }
    }
  }

  for ( int i = 0; i < 3; i++ )
  {
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].mv = cMv[0][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].refIdx = refIdx[0];

    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].mv = cMv[1][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].refIdx = refIdx[1];
  }
  affMrgType.interDirNeighbours[affMrgType.numValidMergeCand] = dir;
  affMrgType.affineType[affMrgType.numValidMergeCand] = curType;
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
  affMrgType.GBiIdx[affMrgType.numValidMergeCand] = gbiIdx;
#endif
  affMrgType.numValidMergeCand++;


  return;
}

const int getAvailableAffineNeighboursForLeftPredictor( const PredictionUnit &pu, const PredictionUnit* npu[] )
{
  const Position posLB = pu.Y().bottomLeft();
  int num = 0;

  const PredictionUnit *puLeftBottom = pu.cs->getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  if ( puLeftBottom && puLeftBottom->cu->affine
    && puLeftBottom->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puLeftBottom;
    return num;
  }

  const PredictionUnit* puLeft = pu.cs->getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  if ( puLeft && puLeft->cu->affine
    && puLeft->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puLeft;
    return num;
  }

  return num;
}

const int getAvailableAffineNeighboursForAbovePredictor( const PredictionUnit &pu, const PredictionUnit* npu[], int numAffNeighLeft )
{
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  int num = numAffNeighLeft;

  const PredictionUnit* puAboveRight = pu.cs->getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  if ( puAboveRight && puAboveRight->cu->affine
    && puAboveRight->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puAboveRight;
    return num;
  }

  const PredictionUnit* puAbove = pu.cs->getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  if ( puAbove && puAbove->cu->affine
    && puAbove->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puAbove;
    return num;
  }

  const PredictionUnit *puAboveLeft = pu.cs->getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  if ( puAboveLeft && puAboveLeft->cu->affine
    && puAboveLeft->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puAboveLeft;
    return num;
  }

  return num;
}

void PU::getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx )
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumAffineMergeCand = slice.getMaxNumAffineMergeCand();

  for ( int i = 0; i < maxNumAffineMergeCand; i++ )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField( Mv(), -1 );
      affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField( Mv(), -1 );
    }
    affMrgCtx.interDirNeighbours[i] = 0;
    affMrgCtx.affineType[i] = AFFINEMODEL_4PARAM;
    affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affMrgCtx.GBiIdx[i] = GBI_DEFAULT;
  }

  affMrgCtx.numValidMergeCand = 0;
  affMrgCtx.maxNumMergeCand = maxNumAffineMergeCand;

  bool enableSubPuMvp = slice.getSPS()->getSBTMVPEnabledFlag() && !(slice.getPOC() == slice.getRefPic(REF_PIC_LIST_0, 0)->getPOC() && slice.isIRAP());
  bool isAvailableSubPu = false;
  if ( enableSubPuMvp && slice.getEnableTMVPFlag() )
  {
    MergeCtx mrgCtx = *affMrgCtx.mrgCtx;
    bool tmpLICFlag = false;

    CHECK( mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized" );
    mrgCtx.subPuMvpMiBuf.fill( MotionInfo() );

    int pos = 0;
    // Get spatial MV
    const Position posCurLB = pu.Y().bottomLeft();
    MotionInfo miLeft;

    //left
    const PredictionUnit* puLeft = cs.getPURestricted( posCurLB.offset( -1, 0 ), pu, pu.chType );
    const bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );
    if ( isAvailableA1 )
    {
      miLeft = puLeft->getMotionInfo( posCurLB.offset( -1, 0 ) );
      // get Inter Dir
      mrgCtx.interDirNeighbours[pos] = miLeft.interDir;

      // get Mv from Left
      mrgCtx.mvFieldNeighbours[pos << 1].setMvField( miLeft.mv[0], miLeft.refIdx[0] );

      if ( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[(pos << 1) + 1].setMvField( miLeft.mv[1], miLeft.refIdx[1] );
      }
      pos++;
    }

    mrgCtx.numValidMergeCand = pos;

    isAvailableSubPu = getInterMergeSubPuMvpCand( pu, mrgCtx, tmpLICFlag, pos
      , 0
    );
    if ( isAvailableSubPu )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 0].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 0].refIdx );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 1].refIdx );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = mrgCtx.interDirNeighbours[pos];

      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINE_MODEL_NUM;
      affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_SUBPU_ATMVP;
      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      affMrgCtx.numValidMergeCand++;

      // early termination
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
  }

  if ( slice.getSPS()->getUseAffine() )
  {
    ///> Start: inherited affine candidates
    const PredictionUnit* npu[5];
    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor( pu, npu );
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor( pu, npu, numAffNeighLeft );
    for ( int idx = 0; idx < numAffNeigh; idx++ )
    {
      // derive Mv from Neigh affine PU
      Mv cMv[2][3];
      const PredictionUnit* puNeigh = npu[idx];
      pu.cu->affineType = puNeigh->cu->affineType;
      if ( puNeigh->interDir != 2 )
      {
        xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_0, cMv[0] );
      }
      if ( slice.isInterB() )
      {
        if ( puNeigh->interDir != 1 )
        {
          xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_1, cMv[1] );
        }
      }

      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( cMv[0][mvNum], puNeigh->refIdx[0] );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( cMv[1][mvNum], puNeigh->refIdx[1] );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir;
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = (EAffineModel)(puNeigh->cu->affineType);
      affMrgCtx.GBiIdx[affMrgCtx.numValidMergeCand] = puNeigh->cu->GBiIdx;

      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool isAvailable[4] = { false };
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
      int8_t neighGbi[4] = { GBI_DEFAULT };
#endif
      // control point: LT B2->B3->A2
      const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
      for ( int i = 0; i < 3; i++ )
      {
        const Position pos = posLT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if ( puNeigh && CU::isInter( *puNeigh->cu )
          )
        {
          isAvailable[0] = true;
          mi[0] = puNeigh->getMotionInfo( pos );
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
          neighGbi[0] = puNeigh->cu->GBiIdx;
#endif
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { pu.Y().topRight().offset( 0, -1 ), pu.Y().topRight().offset( 1, -1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posRT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );


        if ( puNeigh && CU::isInter( *puNeigh->cu )
          )
        {
          isAvailable[1] = true;
          mi[1] = puNeigh->getMotionInfo( pos );
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
          neighGbi[1] = puNeigh->cu->GBiIdx;
#endif
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { pu.Y().bottomLeft().offset( -1, 0 ), pu.Y().bottomLeft().offset( -1, 1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posLB[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );


        if ( puNeigh && CU::isInter( *puNeigh->cu )
          )
        {
          isAvailable[2] = true;
          mi[2] = puNeigh->getMotionInfo( pos );
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
          neighGbi[2] = puNeigh->cu->GBiIdx;
#endif
          break;
        }
      }

      // control point: RB
      if ( slice.getEnableTMVPFlag() )
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = pu.Y().bottomRight().offset( -3, -3 );

        const PreCalcValues& pcv = *cs.pcv;
        Position posC0;
        bool C0Avail = false;

        if ( ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight) )
        {
          Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

          if ( (posInCtu.x + 4 < pcv.maxCUWidth) &&  // is not at the last column of CTU
            (posInCtu.y + 4 < pcv.maxCUHeight) )     // is not at the last row    of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            C0Avail = true;
          }
          else if ( posInCtu.x + 4 < pcv.maxCUWidth ) // is not at the last column of CTU But is last row of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
          }
          else if ( posInCtu.y + 4 < pcv.maxCUHeight ) // is not at the last row of CTU But is last column of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            C0Avail = true;
          }
          else //is the right bottom corner of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            // same as for last column but not last row
          }
        }

        Mv        cColMv;
        int       refIdx = 0;
        bool      bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_0, posC0, cColMv, refIdx );
        if ( bExistMV )
        {
          mi[3].mv[0] = cColMv;
          mi[3].refIdx[0] = refIdx;
          mi[3].interDir = 1;
          isAvailable[3] = true;
        }

        if ( slice.isInterB() )
        {
          bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_1, posC0, cColMv, refIdx );
          if ( bExistMV )
          {
            mi[3].mv[1] = cColMv;
            mi[3].refIdx[1] = refIdx;
            mi[3].interDir |= 2;
            isAvailable[3] = true;
          }
        }
      }

      //-------------------  insert model  -------------------//
      int order[6] = { 0, 1, 2, 3, 4, 5 };
      int modelNum = 6;
      int model[6][4] = {
        { 0, 1, 2 },          // 0:  LT, RT, LB
        { 0, 1, 3 },          // 1:  LT, RT, RB
        { 0, 2, 3 },          // 2:  LT, LB, RB
        { 1, 2, 3 },          // 3:  RT, LB, RB
        { 0, 1 },             // 4:  LT, RT
        { 0, 2 },             // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx = pu.cs->sps->getUseAffineType() ? 0 : 4;
      for ( int idx = startIdx; idx < modelNum; idx++ )
      {
        int modelIdx = order[idx];
#if JVET_N0481_BCW_CONSTRUCTED_AFFINE
        getAffineControlPointCand(pu, mi, neighGbi, isAvailable, model[modelIdx], modelIdx, verNum[modelIdx], affMrgCtx);
#else
        getAffineControlPointCand( pu, mi, isAvailable, model[modelIdx], modelIdx, verNum[modelIdx], affMrgCtx );
#endif
        if ( affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx )
        {
          return;
        }

        // early termination
        if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
        {
          return;
        }
      }
    }
    ///> End: Constructed affine candidates
  }

  ///> zero padding
  int cnt = affMrgCtx.numValidMergeCand;
  while ( cnt < maxNumAffineMergeCand )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField( Mv( 0, 0 ), 0 );
    }
    affMrgCtx.interDirNeighbours[cnt] = 1;

    if ( slice.isInterB() )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField( Mv( 0, 0 ), 0 );
      }
      affMrgCtx.interDirNeighbours[cnt] = 3;
    }
    affMrgCtx.affineType[cnt] = AFFINEMODEL_4PARAM;

    if ( cnt == mrgCandIdx )
    {
      return;
    }
    cnt++;
    affMrgCtx.numValidMergeCand++;
  }
}

void PU::setAllAffineMvField( PredictionUnit &pu, MvField *mvField, RefPicList eRefList )
{
  // Set Mv
  Mv mv[3];
  for ( int i = 0; i < 3; i++ )
  {
    mv[i] = mvField[i].mv;
  }
  setAllAffineMv( pu, mv[0], mv[1], mv[2], eRefList );

  // Set RefIdx
  CHECK( mvField[0].refIdx != mvField[1].refIdx || mvField[0].refIdx != mvField[2].refIdx, "Affine mv corners don't have the same refIdx." );
  pu.refIdx[eRefList] = mvField[0].refIdx;
}

#if JVET_N0334_MVCLIPPING
void PU::setAllAffineMv(PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs)
#else
void PU::setAllAffineMv( PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList)
#endif
{
  int width  = pu.Y().width;
  int shift = MAX_CU_DEPTH;
#if JVET_N0334_MVCLIPPING
  if (clipCPMVs)
  {
    affLT.mvCliptoStorageBitDepth();
    affRT.mvCliptoStorageBitDepth();
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      affLB.mvCliptoStorageBitDepth();
    }
  }
#endif
  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
  deltaMvHorX = (affRT - affLT).getHor() << (shift - g_aucLog2[width]);
  deltaMvHorY = (affRT - affLT).getVer() << (shift - g_aucLog2[width]);
  int height = pu.Y().height;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = (affLB - affLT).getHor() << (shift - g_aucLog2[height]);
    deltaMvVerY = (affLB - affLT).getVer() << (shift - g_aucLog2[height]);
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY = deltaMvHorX;
  }

  int mvScaleHor = affLT.getHor() << shift;
  int mvScaleVer = affLT.getVer() << shift;

  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
  const int halfBW = blockWidth >> 1;
  const int halfBH = blockHeight >> 1;

  MotionBuf mb = pu.getMotionBuf();
  int mvScaleTmpHor, mvScaleTmpVer;
#if JVET_N0068_AFFINE_MEM_BW
  const bool subblkMVSpreadOverLimit = InterPrediction::isSubblockVectorSpreadOverLimit( deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, pu.interDir );
#endif
  for ( int h = 0; h < pu.Y().height; h += blockHeight )
  {
    for ( int w = 0; w < pu.Y().width; w += blockWidth )
    {
#if JVET_N0068_AFFINE_MEM_BW
      if ( !subblkMVSpreadOverLimit )
      {
#endif
        mvScaleTmpHor = mvScaleHor + deltaMvHorX * (halfBW + w) + deltaMvVerX * (halfBH + h);
        mvScaleTmpVer = mvScaleVer + deltaMvHorY * (halfBW + w) + deltaMvVerY * (halfBH + h);

#if JVET_N0068_AFFINE_MEM_BW
      }
      else
      {
        mvScaleTmpHor = mvScaleHor + deltaMvHorX * ( pu.Y().width >> 1 ) + deltaMvVerX * ( pu.Y().height >> 1 );
        mvScaleTmpVer = mvScaleVer + deltaMvHorY * ( pu.Y().width >> 1 ) + deltaMvVerY * ( pu.Y().height >> 1 );
      }
#endif
      roundAffineMv( mvScaleTmpHor, mvScaleTmpVer, shift );
      Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
      curMv.clipToStorageBitDepth();

      for ( int y = (h >> MIN_CU_LOG2); y < ((h + blockHeight) >> MIN_CU_LOG2); y++ )
      {
        for ( int x = (w >> MIN_CU_LOG2); x < ((w + blockWidth) >> MIN_CU_LOG2); x++ )
        {
          mb.at(x, y).mv[eRefList] = curMv;
        }
      }
    }
  }

  pu.mvAffi[eRefList][0] = affLT;
  pu.mvAffi[eRefList][1] = affRT;
  pu.mvAffi[eRefList][2] = affLB;
}

static bool deriveScaledMotionTemporal( const Slice&      slice,
                                        const Position&   colPos,
                                        const Picture*    pColPic,
                                        const RefPicList  eCurrRefPicList,
                                        Mv&         cColMv,
                                        const RefPicList  eFetchRefPicList)
{
  const MotionInfo &mi = pColPic->cs->getMotionInfo(colPos);
  const Slice *pColSlice = nullptr;

  for (const auto &pSlice : pColPic->slices)
  {
    if (pSlice->getIndependentSliceIdx() == mi.sliceIdx)
    {
      pColSlice = pSlice;
      break;
    }
  }

  CHECK(pColSlice == nullptr, "Couldn't find the colocated slice");

  int iColPOC, iColRefPOC, iCurrPOC, iCurrRefPOC, iScale;
  bool bAllowMirrorMV = true;
  RefPicList eColRefPicList = slice.getCheckLDC() ? eCurrRefPicList : RefPicList(1 - eFetchRefPicList);
  if (pColPic == slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx()))
  {
    eColRefPicList = eCurrRefPicList;   //67 -> disable, 64 -> enable
    bAllowMirrorMV = false;
  }

  // Although it might make sense to keep the unavailable motion field per direction still be unavailable, I made the MV prediction the same way as in TMVP
  // So there is an interaction between MV0 and MV1 of the corresponding blocks identified by TV.

  // Grab motion and do necessary scaling.{{
  iCurrPOC = slice.getPOC();

  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (iColRefIdx < 0 && (slice.getCheckLDC() || bAllowMirrorMV))
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = mi.refIdx[eColRefPicList];

    if (iColRefIdx < 0)
    {
      return false;
    }
  }

  if (iColRefIdx >= 0 && slice.getNumRefIdx(eCurrRefPicList) > 0)
  {
    iColPOC = pColSlice->getPOC();
    iColRefPOC = pColSlice->getRefPOC(eColRefPicList, iColRefIdx);
    if (iColPOC == iColRefPOC)
      return false;
    ///////////////////////////////////////////////////////////////
    // Set the target reference index to 0, may be changed later //
    ///////////////////////////////////////////////////////////////
    iCurrRefPOC = slice.getRefPic(eCurrRefPicList, 0)->getPOC();
    // Scale the vector.
    cColMv = mi.mv[eColRefPicList];
    cColMv.setHor(roundMvComp(cColMv.getHor()));
    cColMv.setVer(roundMvComp(cColMv.getVer()));
    //pcMvFieldSP[2*iPartition + eCurrRefPicList].getMv();
    // Assume always short-term for now
    iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);

    if (iScale != 4096)
    {

      cColMv = cColMv.scaleMv(iScale);
    }

    return true;
  }
  return false;
}

void clipColPos(int& posX, int& posY, const PredictionUnit& pu)
{
  Position puPos = pu.lumaPos();
  int log2CtuSize = g_aucLog2[pu.cs->sps->getCTUSize()];
  int ctuX = ((puPos.x >> log2CtuSize) << log2CtuSize);
  int ctuY = ((puPos.y >> log2CtuSize) << log2CtuSize);
  int horMax = std::min((int)pu.cs->sps->getPicWidthInLumaSamples() - 1, ctuX + (int)pu.cs->sps->getCTUSize() + 3);
  int horMin = std::max((int)0, ctuX);
  int verMax = std::min((int)pu.cs->sps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.cs->sps->getCTUSize() - 1);
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
}

bool PU::getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count
  , int mmvdList
)
{
  const Slice   &slice = *pu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);

  const Picture *pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
  Mv cTMv;
  RefPicList fetchRefPicList = RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0);

  bool terminate = false;
  for (unsigned currRefListId = 0; currRefListId < (slice.getSliceType() == B_SLICE ? 2 : 1) && !terminate; currRefListId++)
  {
    if ( count )
    {
      RefPicList currRefPicList = RefPicList(slice.getCheckLDC() ? (slice.getColFromL0Flag() ? currRefListId : 1 - currRefListId) : currRefListId);

      if ((mrgCtx.interDirNeighbours[0] & (1 << currRefPicList)) && slice.getRefPic(currRefPicList, mrgCtx.mvFieldNeighbours[0 * 2 + currRefPicList].refIdx) == pColPic)
      {
        cTMv = mrgCtx.mvFieldNeighbours[0 * 2 + currRefPicList].mv;
        terminate = true;
        fetchRefPicList = currRefPicList;
        break;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////
  int mvPrec = MV_FRACTIONAL_BITS_INTERNAL;

  Mv cTempVector = cTMv;
  bool  tempLICFlag = false;

  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();
  int numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;

  Mv cColMv;
  // use coldir.
  bool     bBSlice = slice.isInterB();

  Position centerPos;

  bool found = false;
  cTempVector = cTMv;
  int tempX = cTempVector.getHor() >> mvPrec;
  int tempY = cTempVector.getVer() >> mvPrec;

  centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
  centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

  clipColPos(centerPos.x, centerPos.y, pu);

  centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

  // derivation of center motion parameters from the collocated CU
  const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

  if (mi.isInter && mi.isIBCmot == false)
  {
    mrgCtx.interDirNeighbours[count] = 0;

    for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList  currRefPicList = RefPicList(currRefListId);

      if (deriveScaledMotionTemporal(slice, centerPos, pColPic, currRefPicList, cColMv, fetchRefPicList))
      {
        // set as default, for further motion vector field spanning
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, 0);
        mrgCtx.interDirNeighbours[count] |= (1 << currRefListId);
        LICFlag = tempLICFlag;
        mrgCtx.GBiIdx[count] = GBI_DEFAULT;
        found = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(Mv(), NOT_VALID);
        mrgCtx.interDirNeighbours[count] &= ~(1 << currRefListId);
      }
    }
  }

  if (!found)
  {
    return false;
  }
  if (mmvdList != 1)
  {
  int xOff = (puWidth >> 1) + tempX;
  int yOff = (puHeight >> 1) + tempY;

  MotionBuf& mb = mrgCtx.subPuMvpMiBuf;

  const bool isBiPred = isBipredRestriction(pu);

  for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
  {
    for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
    {
      Position colPos{ x + xOff, y + yOff };

      clipColPos(colPos.x, colPos.y, pu);

      colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };

      const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);

      MotionInfo mi;

      found = false;
      mi.isInter = true;
      mi.sliceIdx = slice.getIndependentSliceIdx();
      mi.isIBCmot = false;
      if (colMi.isInter && colMi.isIBCmot == false)
      {
        for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
        {
          RefPicList currRefPicList = RefPicList(currRefListId);
          if (deriveScaledMotionTemporal(slice, colPos, pColPic, currRefPicList, cColMv, fetchRefPicList))
          {
            mi.refIdx[currRefListId] = 0;
            mi.mv[currRefListId] = cColMv;
            found = true;
          }
        }
      }
      if (!found)
      {
        mi.mv[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mv;
        mi.mv[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mv;
        mi.refIdx[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].refIdx;
        mi.refIdx[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].refIdx;
      }

      mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);

      if (isBiPred && mi.interDir == 3)
      {
        mi.interDir = 1;
        mi.mv[1] = Mv();
        mi.refIdx[1] = NOT_VALID;
      }

      mb.subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight))).fill(mi);
      }
    }
  }
  return true;
  }

void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
{
  MotionBuf mb = pu.getMotionBuf();

  if( !pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N
    || pu.mergeType == MRG_TYPE_IBC
    )
  {
    MotionInfo mi;

    mi.isInter = !CU::isIntra(*pu.cu);
    mi.isIBCmot = CU::isIBC(*pu.cu);
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;

      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = pu.bv;
      }
    }

    if( pu.cu->affine )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &dest = mb.at( x, y );
          dest.isInter  = mi.isInter;
          dest.isIBCmot = false;
          dest.interDir = mi.interDir;
          dest.sliceIdx = mi.sliceIdx;
          for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
          {
            if( mi.refIdx[i] == -1 )
            {
              dest.mv[i] = Mv();
            }
            dest.refIdx[i] = mi.refIdx[i];
          }
        }
      }
    }
    else
    {
      mb.fill( mi );
    }
  }
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
  }
  else
  {

    if( isBipredRestriction( pu ) )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &mi = mb.at( x, y );
          if( mi.interDir == 3 )
          {
            mi.interDir  = 1;
            mi.mv    [1] = Mv();
            mi.refIdx[1] = NOT_VALID;
          }
        }
      }
    }
  }
}

void PU::applyImv( PredictionUnit& pu, MergeCtx &mrgCtx, InterPrediction *interPred )
{
  if( !pu.mergeFlag )
  {
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      pu.mvd[0].changeTransPrecAmvr2Internal(pu.cu->imv);
      unsigned mvp_idx = pu.mvpIdx[0];
      AMVPInfo amvpInfo;
      if (CU::isIBC(*pu.cu))
      {
        PU::fillIBCMvpCand(pu, amvpInfo);
      }
      else
      PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo);
      pu.mvpNum[0] = amvpInfo.numCand;
      pu.mvpIdx[0] = mvp_idx;
      pu.mv    [0] = amvpInfo.mvCand[mvp_idx] + pu.mvd[0];
#if JVET_N0334_MVCLIPPING
      pu.mv[0].mvCliptoStorageBitDepth();
#endif
    }

    if (pu.interDir != 1 /* PRED_L0 */)
    {
      if( !( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 ) && pu.cu->imv )/* PRED_BI */
      {
        pu.mvd[1].changeTransPrecAmvr2Internal(pu.cu->imv);
      }
      unsigned mvp_idx = pu.mvpIdx[1];
      AMVPInfo amvpInfo;
      PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo);
      pu.mvpNum[1] = amvpInfo.numCand;
      pu.mvpIdx[1] = mvp_idx;
      pu.mv    [1] = amvpInfo.mvCand[mvp_idx] + pu.mvd[1];
#if JVET_N0334_MVCLIPPING
      pu.mv[1].mvCliptoStorageBitDepth();
#endif
    }
  }
  else
  {
    // this function is never called for merge
    THROW("unexpected");
    PU::getInterMergeCandidates ( pu, mrgCtx
      , 0
    );

    mrgCtx.setMergeInfo( pu, pu.mergeIdx );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}

bool PU::isBiPredFromDifferentDir( const PredictionUnit& pu )
{
  if ( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
  {
    const int iPOC0 = pu.cu->slice->getRefPOC( REF_PIC_LIST_0, pu.refIdx[0] );
    const int iPOC1 = pu.cu->slice->getRefPOC( REF_PIC_LIST_1, pu.refIdx[1] );
    const int iPOC  = pu.cu->slice->getPOC();
    if ( (iPOC - iPOC0)*(iPOC - iPOC1) < 0 )
    {
      return true;
    }
  }

  return false;
}
bool PU::isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu)
{
  if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
  {
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, pu.refIdx[0]);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, pu.refIdx[1]);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}
void PU::restrictBiPredMergeCands( const PredictionUnit &pu, MergeCtx& mergeCtx )
{
  if( PU::isBipredRestriction( pu ) )
  {
    for( uint32_t mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; ++mergeCand )
    {
      if( mergeCtx.interDirNeighbours[ mergeCand ] == 3 )
      {
        mergeCtx.interDirNeighbours[ mergeCand ] = 1;
        mergeCtx.mvFieldNeighbours[( mergeCand << 1 ) + 1].setMvField( Mv( 0, 0 ), -1 );
        mergeCtx.GBiIdx[mergeCand] = GBI_DEFAULT;
      }
    }
  }
}

void PU::restrictBiPredMergeCandsOne(PredictionUnit &pu)
{
  if (PU::isBipredRestriction(pu))
  {
    if (pu.interDir == 3)
    {
      pu.interDir = 1;
      pu.refIdx[1] = -1;
      pu.mv[1] = Mv(0, 0);
      pu.cu->GBiIdx = GBI_DEFAULT;
    }
  }
}

void PU::getTriangleMergeCandidates( const PredictionUnit &pu, MergeCtx& triangleMrgCtx )
{
#if JVET_N0340_TRI_MERGE_CAND
  MergeCtx tmpMergeCtx;

  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumMergeCand = slice.getMaxNumMergeCand();

  triangleMrgCtx.numValidMergeCand = 0;

  for (int32_t i = 0; i < TRIANGLE_MAX_NUM_UNI_CANDS; i++)
  {
    triangleMrgCtx.GBiIdx[i] = GBI_DEFAULT;
    triangleMrgCtx.interDirNeighbours[i] = 0;
    triangleMrgCtx.mrgTypeNeighbours[i] = MRG_TYPE_DEFAULT_N;
    triangleMrgCtx.mvFieldNeighbours[(i << 1)].refIdx = NOT_VALID;
    triangleMrgCtx.mvFieldNeighbours[(i << 1) + 1].refIdx = NOT_VALID;
    triangleMrgCtx.mvFieldNeighbours[(i << 1)].mv = Mv();
    triangleMrgCtx.mvFieldNeighbours[(i << 1) + 1].mv = Mv();
  }

  PU::getInterMergeCandidates(pu, tmpMergeCtx, 0);

  for (int32_t i = 0; i < maxNumMergeCand; i++)
  {
    int parity = i & 1;
    if (tmpMergeCtx.interDirNeighbours[i] & (0x01 + parity))
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 1 + parity;
      triangleMrgCtx.mrgTypeNeighbours[triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].mv;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].refIdx = -1;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].refIdx;
      triangleMrgCtx.numValidMergeCand++;
      if (triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS)
      {
        return;
      }
      continue;
    }

    if (tmpMergeCtx.interDirNeighbours[i] & (0x02 - parity))
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 2 - parity;
      triangleMrgCtx.mrgTypeNeighbours[triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].refIdx;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].refIdx = -1;
      triangleMrgCtx.numValidMergeCand++;
      if (triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS)
      {
        return;
      }
    }
  }
#else

  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
  const int32_t maxNumMergeCand = TRIANGLE_MAX_NUM_UNI_CANDS;
  triangleMrgCtx.numValidMergeCand = 0;

  for( int32_t i = 0; i < maxNumMergeCand; i++ )
  {
    triangleMrgCtx.interDirNeighbours[i] = 0;
    triangleMrgCtx.mrgTypeNeighbours [i] = MRG_TYPE_DEFAULT_N;
    triangleMrgCtx.mvFieldNeighbours[(i << 1)    ].refIdx = NOT_VALID;
    triangleMrgCtx.mvFieldNeighbours[(i << 1) + 1].refIdx = NOT_VALID;
    triangleMrgCtx.mvFieldNeighbours[(i << 1)    ].mv = Mv();
    triangleMrgCtx.mvFieldNeighbours[(i << 1) + 1].mv = Mv();
  }

  MotionInfo candidate[TRIANGLE_MAX_NUM_CANDS_MEM];
  int32_t candCount = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  const bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu )
    ;
  if( isAvailableA1 )
  {
    miLeft = puLeft->getMotionInfo( posLB.offset(-1, 0) );
    candidate[candCount].isInter   = true;
    candidate[candCount].interDir  = miLeft.interDir;
    candidate[candCount].mv[0]     = miLeft.mv[0];
    candidate[candCount].mv[1]     = miLeft.mv[1];
    candidate[candCount].refIdx[0] = miLeft.refIdx[0];
    candidate[candCount].refIdx[1] = miLeft.refIdx[1];
    candCount++;
  }

  // above
  const PredictionUnit *puAbove = cs.getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  bool isAvailableB1 = puAbove && isDiffMER( pu, *puAbove ) && pu.cu != puAbove->cu && CU::isInter( *puAbove->cu )
    ;
  if( isAvailableB1 )
  {
    miAbove = puAbove->getMotionInfo( posRT.offset( 0, -1 ) );

    if( !isAvailableA1 || ( miAbove != miLeft ) )
    {
      candidate[candCount].isInter   = true;
      candidate[candCount].interDir  = miAbove.interDir;
      candidate[candCount].mv[0]     = miAbove.mv[0];
      candidate[candCount].mv[1]     = miAbove.mv[1];
      candidate[candCount].refIdx[0] = miAbove.refIdx[0];
      candidate[candCount].refIdx[1] = miAbove.refIdx[1];
      candCount++;
    }
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  bool isAvailableB0 = puAboveRight && isDiffMER( pu, *puAboveRight ) && CU::isInter( *puAboveRight->cu )
    ;

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

    if( ( !isAvailableB1 || ( miAbove != miAboveRight ) ) && ( !isAvailableA1 || ( miLeft != miAboveRight ) ) )
    {
      candidate[candCount].isInter   = true;
      candidate[candCount].interDir  = miAboveRight.interDir;
      candidate[candCount].mv[0]     = miAboveRight.mv[0];
      candidate[candCount].mv[1]     = miAboveRight.mv[1];
      candidate[candCount].refIdx[0] = miAboveRight.refIdx[0];
      candidate[candCount].refIdx[1] = miAboveRight.refIdx[1];
      candCount++;
    }
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  bool isAvailableA0 = puLeftBottom && isDiffMER( pu, *puLeftBottom ) && CU::isInter( *puLeftBottom->cu )
    ;
  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

    if( ( !isAvailableA1 || ( miBelowLeft != miLeft ) ) && ( !isAvailableB1 || ( miBelowLeft != miAbove ) ) && ( !isAvailableB0 || ( miBelowLeft != miAboveRight ) ) )
    {
      candidate[candCount].isInter   = true;
      candidate[candCount].interDir  = miBelowLeft.interDir;
      candidate[candCount].mv[0]     = miBelowLeft.mv[0];
      candidate[candCount].mv[1]     = miBelowLeft.mv[1];
      candidate[candCount].refIdx[0] = miBelowLeft.refIdx[0];
      candidate[candCount].refIdx[1] = miBelowLeft.refIdx[1];
      candCount++;
    }
  }

  // above left
  const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  bool isAvailableB2 = puAboveLeft && isDiffMER( pu, *puAboveLeft ) && CU::isInter( *puAboveLeft->cu )
    ;

  if( isAvailableB2 )
  {
    miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

    if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) && ( !isAvailableA0 || ( miBelowLeft != miAboveLeft ) ) && ( !isAvailableB0 || ( miAboveRight != miAboveLeft ) ) )
    {
      candidate[candCount].isInter   = true;
      candidate[candCount].interDir  = miAboveLeft.interDir;
      candidate[candCount].mv[0]     = miAboveLeft.mv[0];
      candidate[candCount].mv[1]     = miAboveLeft.mv[1];
      candidate[candCount].refIdx[0] = miAboveLeft.refIdx[0];
      candidate[candCount].refIdx[1] = miAboveLeft.refIdx[1];
      candCount++;
    }
  }

  if( slice.getEnableTMVPFlag() )
  {
    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool isAvailableC0 = false;
#if !JVET_N0266_SMALL_BLOCKS
    bool isAvailableC1 = (posC1.x < pcv.lumaWidth) && (posC1.y < pcv.lumaHeight);
#endif
    if (((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight))
    {
      Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

      if( ( posInCtu.x + 4 < pcv.maxCUWidth ) &&           // is not at the last column of CTU
          ( posInCtu.y + 4 < pcv.maxCUHeight ) )           // is not at the last row    of CTU
      {
        posC0 = posRB.offset( 4, 4 );
        isAvailableC0 = true;
      }
      else if( posInCtu.x + 4 < pcv.maxCUWidth )           // is not at the last column of CTU But is last row of CTU
      {
        posC0 = posRB.offset( 4, 4 );
        // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
      }
      else if( posInCtu.y + 4 < pcv.maxCUHeight )          // is not at the last row of CTU But is last column of CTU
      {
        posC0 = posRB.offset( 4, 4 );
        isAvailableC0 = true;
      }
      else //is the right bottom corner of CTU
      {
        posC0 = posRB.offset( 4, 4 );
        // same as for last column but not last row
      }
    }

    // C0
    Mv        cColMv;
    int32_t   refIdx     = 0;
    bool      existMV    = ( isAvailableC0 && getColocatedMVP( pu, REF_PIC_LIST_0, posC0, cColMv, refIdx ) );
    MotionInfo temporalMv;
    temporalMv.interDir  = 0;
    if( existMV )
    {
      temporalMv.isInter   = true;
      temporalMv.interDir |= 1;
      temporalMv.mv[0]     = cColMv;
      temporalMv.refIdx[0] = refIdx;
    }
    existMV = ( isAvailableC0 && getColocatedMVP( pu, REF_PIC_LIST_1, posC0, cColMv, refIdx ) );
    if( existMV )
    {
      temporalMv.interDir |= 2;
      temporalMv.mv[1]     = cColMv;
      temporalMv.refIdx[1] = refIdx;
    }

    if( temporalMv.interDir != 0 )
    {
      candidate[candCount].isInter   = true;
      candidate[candCount].interDir  = temporalMv.interDir;
      candidate[candCount].mv[0]     = temporalMv.mv[0];
      candidate[candCount].mv[1]     = temporalMv.mv[1];
      candidate[candCount].refIdx[0] = temporalMv.refIdx[0];
      candidate[candCount].refIdx[1] = temporalMv.refIdx[1];
      candCount++;
    }

    // C1
    temporalMv.interDir = 0;
#if JVET_N0266_SMALL_BLOCKS
    existMV = getColocatedMVP( pu, REF_PIC_LIST_0, posC1, cColMv, refIdx );
#else
    existMV    = isAvailableC1 && getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv, refIdx );
#endif
    if( existMV )
    {
      temporalMv.isInter   = true;
      temporalMv.interDir |= 1;
      temporalMv.mv[0]     = cColMv;
      temporalMv.refIdx[0] = refIdx;
    }
#if JVET_N0266_SMALL_BLOCKS
    existMV = getColocatedMVP( pu, REF_PIC_LIST_1, posC1, cColMv, refIdx );
#else
    existMV    = isAvailableC1 && getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv, refIdx );
#endif
    if( existMV )
    {
      temporalMv.interDir |= 2;
      temporalMv.mv[1]     = cColMv;
      temporalMv.refIdx[1] = refIdx;
    }

    if( temporalMv.interDir != 0 )
    {
      candidate[candCount].isInter   = true;
      candidate[candCount].interDir  = temporalMv.interDir;
      candidate[candCount].mv[0]     = temporalMv.mv[0];
      candidate[candCount].mv[1]     = temporalMv.mv[1];
      candidate[candCount].refIdx[0] = temporalMv.refIdx[0];
      candidate[candCount].refIdx[1] = temporalMv.refIdx[1];
      candCount++;
    }
  }
  // put uni-prediction candidate to the triangle candidate list
  for( int32_t i = 0; i < candCount; i++ )
  {
    if( candidate[i].interDir != 3 )
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = candidate[i].interDir;
      triangleMrgCtx.mrgTypeNeighbours [triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].mv = candidate[i].mv[0];
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].mv = candidate[i].mv[1];
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].refIdx = candidate[i].refIdx[0];
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].refIdx = candidate[i].refIdx[1];
      triangleMrgCtx.numValidMergeCand += isUniqueTriangleCandidates(pu, triangleMrgCtx);
      if( triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS )
      {
        return;
      }
    }
  }

  // put L0 mv of bi-prediction candidate to the triangle candidate list
  for( int32_t i = 0; i < candCount; i++ )
  {
    if( candidate[i].interDir == 3 )
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 1;
      triangleMrgCtx.mrgTypeNeighbours [triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].mv = candidate[i].mv[0];
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].mv = Mv(0, 0);
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].refIdx = candidate[i].refIdx[0];
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].refIdx = -1;
      triangleMrgCtx.numValidMergeCand += isUniqueTriangleCandidates(pu, triangleMrgCtx);
      if( triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS )
      {
        return;
      }
    }
  }

  // put L1 mv of bi-prediction candidate to the triangle candidate list
  for( int32_t i = 0; i < candCount; i++ )
  {
    if( candidate[i].interDir == 3 )
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 2;
      triangleMrgCtx.mrgTypeNeighbours [triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].mv = Mv(0, 0);
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].mv = candidate[i].mv[1];
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].refIdx = -1;
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].refIdx = candidate[i].refIdx[1];
      triangleMrgCtx.numValidMergeCand += isUniqueTriangleCandidates(pu, triangleMrgCtx);
      if( triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS )
      {
        return;
      }
    }
  }

  // put average of L0 and L1 mvs of bi-prediction candidate to the triangle candidate list
  for( int32_t i = 0; i < candCount; i++ )
  {
    if( candidate[i].interDir == 3 )
    {
      int32_t curPicPoc   = slice.getPOC();
      int32_t refPicPocL0 = slice.getRefPOC(REF_PIC_LIST_0, candidate[i].refIdx[0]);
      int32_t refPicPocL1 = slice.getRefPOC(REF_PIC_LIST_1, candidate[i].refIdx[1]);
      Mv aveMv = candidate[i].mv[1];
      int32_t distscale = xGetDistScaleFactor( curPicPoc, refPicPocL0, curPicPoc, refPicPocL1 );
      if( distscale != 4096 )
      {
        aveMv = aveMv.scaleMv( distscale ); // scaling to L0
      }
      aveMv = aveMv + candidate[i].mv[0];
      roundAffineMv(aveMv.hor, aveMv.ver, 1);
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 1;
      triangleMrgCtx.mrgTypeNeighbours [triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].mv = aveMv;
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].mv = Mv(0, 0);
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1)    ].refIdx = candidate[i].refIdx[0];
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1].refIdx = -1;
      triangleMrgCtx.numValidMergeCand += isUniqueTriangleCandidates(pu, triangleMrgCtx);
      if( triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS )
      {
        return;
      }
    }
  }

  // fill with Mv(0, 0)
  int32_t numRefIdx = std::min( slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1) );
  int32_t cnt = 0;
  while( triangleMrgCtx.numValidMergeCand < TRIANGLE_MAX_NUM_UNI_CANDS )
  {
    if( cnt < numRefIdx )
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 1;
      triangleMrgCtx.mvFieldNeighbours[triangleMrgCtx.numValidMergeCand << 1].setMvField(Mv(0, 0), cnt);
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + 1].refIdx = NOT_VALID;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + 1].mv = Mv();
      triangleMrgCtx.numValidMergeCand++;

      if( triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS )
      {
        return;
      }

      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 2;
      triangleMrgCtx.mvFieldNeighbours [(triangleMrgCtx.numValidMergeCand << 1) + 1 ].setMvField(Mv(0, 0), cnt);
      triangleMrgCtx.mvFieldNeighbours[triangleMrgCtx.numValidMergeCand << 1].refIdx = NOT_VALID;
      triangleMrgCtx.mvFieldNeighbours[triangleMrgCtx.numValidMergeCand << 1].mv = Mv();
      triangleMrgCtx.numValidMergeCand++;

      cnt = (cnt + 1) % numRefIdx;
    }
  }
#endif
}

bool PU::isUniqueTriangleCandidates( const PredictionUnit &pu, MergeCtx& triangleMrgCtx )
{
  int newCand = triangleMrgCtx.numValidMergeCand;
  for( int32_t i = 0; i < newCand; i++ )
  {
    int32_t predFlagCur  = triangleMrgCtx.interDirNeighbours[i] == 1 ? 0 : 1;
    int32_t predFlagNew  = triangleMrgCtx.interDirNeighbours[newCand] == 1 ? 0 : 1;
    int32_t refPicPocCur = pu.cs->slice->getRefPOC( (RefPicList)predFlagCur, triangleMrgCtx.mvFieldNeighbours[(i << 1) + predFlagCur].refIdx );
    int32_t refPicPocNew = pu.cs->slice->getRefPOC( (RefPicList)predFlagNew, triangleMrgCtx.mvFieldNeighbours[(newCand << 1) + predFlagNew].refIdx);
    if( refPicPocCur == refPicPocNew && triangleMrgCtx.mvFieldNeighbours[(i << 1) + predFlagCur].mv == triangleMrgCtx.mvFieldNeighbours[(newCand << 1) + predFlagNew].mv )
    {
      return false;
    }
  }
  return true;
}


void PU::spanTriangleMotionInfo( PredictionUnit &pu, MergeCtx &triangleMrgCtx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1 )
{
  pu.triangleSplitDir = splitDir;
  pu.triangleMergeIdx0 = candIdx0;
  pu.triangleMergeIdx1 = candIdx1;
  MotionBuf mb = pu.getMotionBuf();

  MotionInfo biMv;
  biMv.isInter  = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if( triangleMrgCtx.interDirNeighbours[candIdx0] == 1 && triangleMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
    biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].refIdx;
    biMv.refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }
  else if( triangleMrgCtx.interDirNeighbours[candIdx0] == 2 && triangleMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
    biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
    biMv.refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].refIdx;
    biMv.refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
  }
  else if( triangleMrgCtx.interDirNeighbours[candIdx0] == 1 && triangleMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    int32_t refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_0, triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx ), REF_PIC_LIST_1 );
    if( refIdx != -1 )
    {
      biMv.interDir  = 3;
      biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
      biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
      biMv.refIdx[0] = triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx;
      biMv.refIdx[1] = refIdx;
    }
    else
    {
      refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_0, triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx), REF_PIC_LIST_1 );
      biMv.interDir  = ( refIdx != -1 ) ? 3 : 1;
      biMv.mv[0]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv : triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
      biMv.mv[1]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv : Mv(0, 0);
      biMv.refIdx[0] = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx : triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx;
      biMv.refIdx[1] = ( refIdx != -1 ) ? refIdx : -1;
    }
  }
  else if( triangleMrgCtx.interDirNeighbours[candIdx0] == 2 && triangleMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    int32_t refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_1, triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx ), REF_PIC_LIST_0 );
    if( refIdx != -1 )
    {
      biMv.interDir  = 3;
      biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
      biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
      biMv.refIdx[0] = refIdx;
      biMv.refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
    }
    else
    {
      refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_1, triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx ), REF_PIC_LIST_0 );
      biMv.interDir  = ( refIdx != -1 ) ? 3 : 2;
      biMv.mv[0]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv : Mv(0, 0);
      biMv.mv[1]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv : triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
      biMv.refIdx[0] = ( refIdx != -1 ) ? refIdx : -1;
      biMv.refIdx[1] = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx : triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
    }
  }

  int32_t idxW  = (int32_t)(g_aucLog2[pu.lwidth() ] - MIN_CU_LOG2);
  int32_t idxH  = (int32_t)(g_aucLog2[pu.lheight()] - MIN_CU_LOG2);
  for( int32_t y = 0; y < mb.height; y++ )
  {
    for( int32_t x = 0; x < mb.width; x++ )
    {
      if( g_triangleMvStorage[splitDir][idxH][idxW][y][x] == 2 )
      {
        mb.at( x, y ).isInter   = true;
        mb.at( x, y ).interDir  = biMv.interDir;
        mb.at( x, y ).refIdx[0] = biMv.refIdx[0];
        mb.at( x, y ).refIdx[1] = biMv.refIdx[1];
        mb.at( x, y ).mv    [0] = biMv.mv    [0];
        mb.at( x, y ).mv    [1] = biMv.mv    [1];
        mb.at( x, y ).sliceIdx  = biMv.sliceIdx;
      }
      else if( g_triangleMvStorage[splitDir][idxH][idxW][y][x] == 0 )
      {
        mb.at( x, y ).isInter   = true;
        mb.at( x, y ).interDir  = triangleMrgCtx.interDirNeighbours[candIdx0];
        mb.at( x, y ).refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].refIdx;
        mb.at( x, y ).refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
        mb.at( x, y ).mv    [0] = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
        mb.at( x, y ).mv    [1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
        mb.at( x, y ).sliceIdx  = biMv.sliceIdx;
      }
      else
      {
        mb.at( x, y ).isInter   = true;
        mb.at( x, y ).interDir  = triangleMrgCtx.interDirNeighbours[candIdx1];
        mb.at( x, y ).refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].refIdx;
        mb.at( x, y ).refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
        mb.at( x, y ).mv    [0] = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
        mb.at( x, y ).mv    [1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
        mb.at( x, y ).sliceIdx  = biMv.sliceIdx;
      }
    }
  }
}

int32_t PU::mappingRefPic( const PredictionUnit &pu, int32_t refPicPoc, bool targetRefPicList )
{
  int32_t numRefIdx = pu.cs->slice->getNumRefIdx( (RefPicList)targetRefPicList );

  for( int32_t i = 0; i < numRefIdx; i++ )
  {
    if( pu.cs->slice->getRefPOC( (RefPicList)targetRefPicList, i ) == refPicPoc )
    {
      return i;
    }
  }
  return -1;
}

void CU::resetMVDandMV2Int( CodingUnit& cu, InterPrediction *interPred )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    MergeCtx mrgCtx;

    if( !pu.mergeFlag )
    {
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        Mv mv        = pu.mv[0];
        Mv mvPred;
        AMVPInfo amvpInfo;
        if (CU::isIBC(*pu.cu))
          PU::fillIBCMvpCand(pu, amvpInfo);
        else
        PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo);
        pu.mvpNum[0] = amvpInfo.numCand;

        mvPred       = amvpInfo.mvCand[pu.mvpIdx[0]];
        mv.roundTransPrecInternal2Amvr(cu.imv);
        pu.mv[0]     = mv;
        Mv mvDiff    = mv - mvPred;
        pu.mvd[0]    = mvDiff;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        Mv mv        = pu.mv[1];
        Mv mvPred;
        AMVPInfo amvpInfo;
        PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo);
        pu.mvpNum[1] = amvpInfo.numCand;

        mvPred       = amvpInfo.mvCand[pu.mvpIdx[1]];
        mv.roundTransPrecInternal2Amvr(cu.imv);
        Mv mvDiff    = mv - mvPred;

        if( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 /* PRED_BI */ )
        {
          pu.mvd[1] = Mv();
          mv = mvPred;
        }
        else
        {
          pu.mvd[1] = mvDiff;
        }
        pu.mv[1] = mv;
      }

    }
    else
    {
        PU::getInterMergeCandidates ( pu, mrgCtx
          , 0
        );

        mrgCtx.setMergeInfo( pu, pu.mergeIdx );
    }

    PU::spanMotionInfo( pu, mrgCtx );
  }
}

bool CU::hasSubCUNonZeroMVd( const CodingUnit& cu )
{
  bool bNonZeroMvd = false;

  for( const auto &pu : CU::traversePUs( cu ) )
  {
    if( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getHor() != 0;
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getVer() != 0;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        if( !pu.cu->cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getHor() != 0;
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getVer() != 0;
        }
      }
    }
  }

  return bNonZeroMvd;
}

bool CU::hasSubCUNonZeroAffineMVd( const CodingUnit& cu )
{
  bool nonZeroAffineMvd = false;

  if ( !cu.affine || cu.firstPU->mergeFlag )
  {
    return false;
  }

  for ( const auto &pu : CU::traversePUs( cu ) )
  {
    if ( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if ( pu.interDir != 2 /* PRED_L1 */ )
      {
        for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
        {
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getHor() != 0;
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getVer() != 0;
        }
      }

      if ( pu.interDir != 1 /* PRED_L0 */ )
      {
        if ( !pu.cu->cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
          {
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getHor() != 0;
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getVer() != 0;
          }
        }
      }
    }
  }

  return nonZeroAffineMvd;
}

int CU::getMaxNeighboriMVCandNum( const CodingStructure& cs, const Position& pos )
{
  const int  numDefault     = 0;
  int        maxImvNumCand  = 0;

  // Get BCBP of left PU
#if JVET_N0857_TILES_BRICKS
#if JVET_N0150_ONE_CTU_DELAY_WPP
  const CodingUnit *cuLeft  = cs.getCURestricted( pos.offset( -1, 0 ), pos, cs.slice->getIndependentSliceIdx(), cs.picture->brickMap->getBrickIdxRsMap( pos ), CH_L );
#else
  const CodingUnit *cuLeft  = cs.getCURestricted( pos.offset( -1, 0 ), cs.slice->getIndependentSliceIdx(), cs.picture->brickMap->getBrickIdxRsMap( pos ), CH_L );
#endif
#else
#if JVET_N0150_ONE_CTU_DELAY_WPP
  const CodingUnit *cuLeft  = cs.getCURestricted( pos.offset( -1, 0 ), pos, cs.slice->getIndependentSliceIdx(), cs.picture->tileMap->getTileIdxMap( pos ), CH_L );
#else
  const CodingUnit *cuLeft  = cs.getCURestricted( pos.offset( -1, 0 ), cs.slice->getIndependentSliceIdx(), cs.picture->tileMap->getTileIdxMap( pos ), CH_L );
#endif
#endif
  maxImvNumCand = ( cuLeft ) ? cuLeft->imvNumCand : numDefault;

  // Get BCBP of above PU
#if JVET_N0857_TILES_BRICKS
#if JVET_N0150_ONE_CTU_DELAY_WPP
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, cs.slice->getIndependentSliceIdx(), cs.picture->brickMap->getBrickIdxRsMap( pos ), CH_L );
#else
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), cs.slice->getIndependentSliceIdx(), cs.picture->brickMap->getBrickIdxRsMap( pos ), CH_L );
#endif
#else
#if JVET_N0150_ONE_CTU_DELAY_WPP
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, cs.slice->getIndependentSliceIdx(), cs.picture->tileMap->getTileIdxMap( pos ), CH_L );
#else
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), cs.slice->getIndependentSliceIdx(), cs.picture->tileMap->getTileIdxMap( pos ), CH_L );
#endif
#endif
  maxImvNumCand = std::max( maxImvNumCand, ( cuAbove ) ? cuAbove->imvNumCand : numDefault );

  return maxImvNumCand;
}

uint8_t CU::getSbtInfo( uint8_t idx, uint8_t pos )
{
  return ( pos << 4 ) + ( idx << 0 );
}

uint8_t CU::getSbtIdx( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 0 ) & 0xf;
}

uint8_t CU::getSbtPos( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 4 ) & 0x3;
}

uint8_t CU::getSbtMode( uint8_t sbtIdx, uint8_t sbtPos )
{
  uint8_t sbtMode = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: sbtMode = sbtPos + SBT_VER_H0;  break;
  case SBT_HOR_HALF: sbtMode = sbtPos + SBT_HOR_H0;  break;
  case SBT_VER_QUAD: sbtMode = sbtPos + SBT_VER_Q0;  break;
  case SBT_HOR_QUAD: sbtMode = sbtPos + SBT_HOR_Q0;  break;
  default:           assert( 0 );
  }

  assert( sbtMode < NUMBER_SBT_MODE );
  return sbtMode;
}

uint8_t CU::getSbtIdxFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
    return SBT_VER_HALF;
  else if( sbtMode <= SBT_HOR_H1 )
    return SBT_HOR_HALF;
  else if( sbtMode <= SBT_VER_Q1 )
    return SBT_VER_QUAD;
  else if( sbtMode <= SBT_HOR_Q1 )
    return SBT_HOR_QUAD;
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::getSbtPosFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
    return sbtMode - SBT_VER_H0;
  else if( sbtMode <= SBT_HOR_H1 )
    return sbtMode - SBT_HOR_H0;
  else if( sbtMode <= SBT_VER_Q1 )
    return sbtMode - SBT_VER_Q0;
  else if( sbtMode <= SBT_HOR_Q1 )
    return sbtMode - SBT_HOR_Q0;
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::targetSbtAllowed( uint8_t sbtIdx, uint8_t sbtAllowed )
{
  uint8_t val = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: val = ( ( sbtAllowed >> SBT_VER_HALF ) & 0x1 ); break;
  case SBT_HOR_HALF: val = ( ( sbtAllowed >> SBT_HOR_HALF ) & 0x1 ); break;
  case SBT_VER_QUAD: val = ( ( sbtAllowed >> SBT_VER_QUAD ) & 0x1 ); break;
  case SBT_HOR_QUAD: val = ( ( sbtAllowed >> SBT_HOR_QUAD ) & 0x1 ); break;
  default:           CHECK( 1, "unknown SBT type" );
  }
  return val;
}

uint8_t CU::numSbtModeRdo( uint8_t sbtAllowed )
{
  uint8_t num = 0;
  uint8_t sum = 0;
  num = targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) + targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  num = targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  return sum;
}

bool CU::isMtsMode( const uint8_t sbtInfo )
{
  return getSbtIdx( sbtInfo ) == SBT_OFF_MTS;
}

bool CU::isSbtMode( const uint8_t sbtInfo )
{
  uint8_t sbtIdx = getSbtIdx( sbtInfo );
  return sbtIdx >= SBT_VER_HALF && sbtIdx <= SBT_HOR_QUAD;
}

bool CU::isSameSbtSize( const uint8_t sbtInfo1, const uint8_t sbtInfo2 )
{
  uint8_t sbtIdx1 = getSbtIdxFromSbtMode( sbtInfo1 );
  uint8_t sbtIdx2 = getSbtIdxFromSbtMode( sbtInfo2 );
  if( sbtIdx1 == SBT_HOR_HALF || sbtIdx1 == SBT_VER_HALF )
    return sbtIdx2 == SBT_HOR_HALF || sbtIdx2 == SBT_VER_HALF;
  else if( sbtIdx1 == SBT_HOR_QUAD || sbtIdx1 == SBT_VER_QUAD )
    return sbtIdx2 == SBT_HOR_QUAD || sbtIdx2 == SBT_VER_QUAD;
  else
    return false;
}

bool CU::isGBiIdxCoded( const CodingUnit &cu )
{
  if( cu.cs->sps->getUseGBi() == false )
  {
    CHECK(cu.GBiIdx != GBI_DEFAULT, "Error: cu.GBiIdx != GBI_DEFAULT");
    return false;
  }

  if (cu.predMode == MODE_IBC)
  {
    return false;
  }

  if( cu.predMode == MODE_INTRA || cu.cs->slice->isInterP() )
  {
    return false;
  }

  if( cu.lwidth() * cu.lheight() < GBI_SIZE_CONSTRAINT )
  {
    return false;
  }

  if( !cu.firstPU->mergeFlag )
  {
    if( cu.firstPU->interDir == 3 )
    {
		WPScalingParam *wp0;
		WPScalingParam *wp1;
		int refIdx0 = cu.firstPU->refIdx[REF_PIC_LIST_0];
		int refIdx1 = cu.firstPU->refIdx[REF_PIC_LIST_1];

		cu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0, wp0);
		cu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1, wp1);
		if ((wp0[COMPONENT_Y].bPresentFlag || wp0[COMPONENT_Cb].bPresentFlag || wp0[COMPONENT_Cr].bPresentFlag
			|| wp1[COMPONENT_Y].bPresentFlag || wp1[COMPONENT_Cb].bPresentFlag || wp1[COMPONENT_Cr].bPresentFlag)
			)
		{
			return false;
		}
      return true;
    }
  }

  return false;
}

uint8_t CU::getValidGbiIdx( const CodingUnit &cu )
{
  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    return cu.GBiIdx;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    CHECK(cu.GBiIdx != GBI_DEFAULT, " cu.GBiIdx != GBI_DEFAULT ");
  }
  else
  {
    CHECK(cu.GBiIdx != GBI_DEFAULT, " cu.GBiIdx != GBI_DEFAULT ");
  }

  return GBI_DEFAULT;
}

void CU::setGbiIdx( CodingUnit &cu, uint8_t uh )
{
  int8_t uhCnt = 0;

  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    cu.GBiIdx = uh;
    ++uhCnt;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    cu.GBiIdx = GBI_DEFAULT;
  }
  else
  {
    cu.GBiIdx = GBI_DEFAULT;
  }

  CHECK(uhCnt <= 0, " uhCnt <= 0 ");
}

uint8_t CU::deriveGbiIdx( uint8_t gbiLO, uint8_t gbiL1 )
{
  if( gbiLO == gbiL1 )
  {
    return gbiLO;
  }
  const int8_t w0 = getGbiWeight(gbiLO, REF_PIC_LIST_0);
  const int8_t w1 = getGbiWeight(gbiL1, REF_PIC_LIST_1);
  const int8_t th = g_GbiWeightBase >> 1;
  const int8_t off = 1;

  if( w0 == w1 || (w0 < (th - off) && w1 < (th - off)) || (w0 >(th + off) && w1 >(th + off)) )
  {
    return GBI_DEFAULT;
  }
  else
  {
    if( w0 > w1 )
    {
      return ( w0 >= th ? gbiLO : gbiL1 );
    }
    else
    {
      return ( w1 >= th ? gbiL1 : gbiLO );
    }
  }
}

#if JVET_N0413_RDPCM
bool CU::bdpcmAllowed( const CodingUnit& cu, const ComponentID compID )
{
  bool bdpcmAllowed = compID == COMPONENT_Y;
       bdpcmAllowed &= CU::isIntra( cu );
       bdpcmAllowed &= ( cu.lwidth() <= 32 && cu.lheight() <= 32 );

  return bdpcmAllowed;
}
#endif
// TU tools

bool TU::isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID)
{
  return tu.cs->sps->getSpsRangeExtension().getTransformSkipRotationEnabledFlag() && tu.blocks[compID].width == 4 && tu.cu->predMode == MODE_INTRA;
}

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
  return getCbfAtDepth( tu, compID, tu.depth );
}

bool TU::getCbfAtDepth(const TransformUnit &tu, const ComponentID &compID, const unsigned &depth)
{
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1  << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}

bool TU::isTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  bool    tsAllowed = compID == COMPONENT_Y;
  const int maxSize = tu.cs->pps->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize();

  tsAllowed &= tu.cs->pps->getUseTransformSkip();
  tsAllowed &= !tu.cu->transQuantBypass;
  tsAllowed &= ( !tu.cu->ispMode || !isLuma(compID) );
#if JVET_N0413_RDPCM
  tsAllowed &= !( tu.cu->bdpcmMode && tu.lwidth() <= BDPCM_MAX_CU_SIZE && tu.lheight() <= BDPCM_MAX_CU_SIZE );
#endif
  SizeType transformSkipMaxSize = 1 << maxSize;
  tsAllowed &= tu.lwidth() <= transformSkipMaxSize && tu.lheight() <= transformSkipMaxSize;
  tsAllowed &= !tu.cu->sbtInfo;

  return tsAllowed;
}

bool TU::isMTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  bool   mtsAllowed = compID == COMPONENT_Y;
  const int maxSize = CU::isIntra( *tu.cu ) ? MTS_INTRA_MAX_CU_SIZE : MTS_INTER_MAX_CU_SIZE;

  mtsAllowed &= CU::isIntra( *tu.cu ) ? tu.cs->sps->getUseIntraMTS() : tu.cs->sps->getUseInterMTS() && CU::isInter( *tu.cu );
  mtsAllowed &= ( tu.lwidth() <= maxSize && tu.lheight() <= maxSize );
  mtsAllowed &= !tu.cu->ispMode;
  mtsAllowed &= !tu.cu->sbtInfo;
#if JVET_N0413_RDPCM
  mtsAllowed &= !( tu.cu->bdpcmMode && tu.lwidth() <= BDPCM_MAX_CU_SIZE && tu.lheight() <= BDPCM_MAX_CU_SIZE );
#endif
  return mtsAllowed;
}

uint32_t TU::getGolombRiceStatisticsIndex(const TransformUnit &tu, const ComponentID &compID)
{
  const bool transformSkip    = tu.mtsIdx==MTS_SKIP;
  const bool transquantBypass = tu.cu->transQuantBypass;

  //--------

  const uint32_t channelTypeOffset = isChroma(compID) ? 2 : 0;
  const uint32_t nonTransformedOffset = (transformSkip || transquantBypass) ? 1 : 0;

  //--------

  const uint32_t selectedIndex = channelTypeOffset + nonTransformedOffset;
  CHECK( selectedIndex >= RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS, "Invalid golomb rice adaptation statistics set" );

  return selectedIndex;
}

#if HEVC_USE_MDCS
uint32_t TU::getCoefScanIdx(const TransformUnit &tu, const ComponentID &compID)
{
  //------------------------------------------------

  //this mechanism is available for intra only

  if( !CU::isIntra( *tu.cu ) )
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //check that MDCS can be used for this TU


  const CompArea &area      = tu.blocks[compID];
  const SPS &sps            = *tu.cs->sps;
  const ChromaFormat format = sps.getChromaFormatIdc();


  const uint32_t maximumWidth  = MDCS_MAXIMUM_WIDTH  >> getComponentScaleX(compID, format);
  const uint32_t maximumHeight = MDCS_MAXIMUM_HEIGHT >> getComponentScaleY(compID, format);

  if ((area.width > maximumWidth) || (area.height > maximumHeight))
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //otherwise, select the appropriate mode

  const PredictionUnit &pu = *tu.cs->getPU( area.pos(), toChannelType( compID ) );

  uint32_t uiDirMode = PU::getFinalIntraMode(pu, toChannelType(compID));

  //------------------

       if (abs((int) uiDirMode - VER_IDX) <= MDCS_ANGLE_LIMIT)
  {
    return SCAN_HOR;
  }
  else if (abs((int) uiDirMode - HOR_IDX) <= MDCS_ANGLE_LIMIT)
  {
    return SCAN_VER;
  }
  else
  {
    return SCAN_DIAG;
  }
}

#endif
bool TU::hasCrossCompPredInfo( const TransformUnit &tu, const ComponentID &compID )
{
  return (isChroma(compID) && tu.cs->pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && TU::getCbf(tu, COMPONENT_Y) &&
    (!CU::isIntra(*tu.cu) || PU::isChromaIntraModeCrossCheckMode(*tu.cs->getPU(tu.blocks[compID].pos(), toChannelType(compID)))));
}

uint32_t TU::getNumNonZeroCoeffsNonTS( const TransformUnit& tu, const bool bLuma, const bool bChroma )
{
  uint32_t count = 0;
  for( uint32_t i = 0; i < ::getNumberValidTBlocks( *tu.cs->pcv ); i++ )
  {
#if JVET_N0193_LFNST
    if( tu.blocks[ i ].valid() && tu.mtsIdx != MTS_SKIP && TU::getCbf( tu, ComponentID( i ) ) )
#else
    if( tu.blocks[i].valid() && ( isLuma(ComponentID(i)) ? tu.mtsIdx !=MTS_SKIP : true ) && TU::getCbf( tu, ComponentID( i ) ) )
#endif
    {
      if( isLuma  ( tu.blocks[i].compID ) && !bLuma   ) continue;
      if( isChroma( tu.blocks[i].compID ) && !bChroma ) continue;

      uint32_t area = tu.blocks[i].area();
      const TCoeff* coeff = tu.getCoeffs( ComponentID( i ) ).buf;
      for( uint32_t j = 0; j < area; j++ )
      {
        count += coeff[j] != 0;
      }
    }
  }
  return count;
}

#if JVET_N0193_LFNST
uint32_t TU::getNumNonZeroCoeffsNonTSCorner8x8( const TransformUnit& tu, const bool lumaFlag, const bool chromaFlag )
{
  const uint32_t lumaWidth       = tu.blocks[ 0 ].width,  chromaWidth  = tu.blocks[ 1 ].width;
  const uint32_t lumaHeight      = tu.blocks[ 0 ].height, chromaHeight = tu.blocks[ 1 ].height;
  bool           luma4x4TUFlag   = lumaWidth     == 4 && lumaHeight   == 4;
  bool           chroma4x4TUFlag = chromaWidth   == 4 && chromaHeight == 4;
  bool           luma8x8TUFlag   = lumaWidth     == 8 && lumaHeight   == 8;
  bool           chroma8x8TUFlag = chromaWidth   == 8 && chromaHeight == 8;
  bool           lumaCountFlag   = ( lumaWidth   >= 8 && lumaHeight   >= 8 ) || luma4x4TUFlag;
  bool           chromaCountFlag = ( chromaWidth >= 8 && chromaHeight >= 8 ) || chroma4x4TUFlag;

  uint32_t count = 0;
  for( uint32_t i = 0; i < ::getNumberValidTBlocks( *tu.cs->pcv ); i++ )
  {
    if( tu.blocks[ i ].valid() && tu.mtsIdx != MTS_SKIP && TU::getCbf( tu, ComponentID( i ) ) )
    {
      if(   isLuma( tu.blocks[ i ].compID ) && (   !lumaFlag ||   !lumaCountFlag ) ) continue;
      if( isChroma( tu.blocks[ i ].compID ) && ( !chromaFlag || !chromaCountFlag ) ) continue;

      const ScanElement * scan  = g_coefTopLeftDiagScan8x8[ gp_sizeIdxInfo->idxFrom( tu.blocks[ i ].width ) ];
      const TCoeff*       coeff = tu.getCoeffs( ComponentID( i ) ).buf;

      int startPos = MAX_LFNST_COEF_NUM, endPos = 47;
      if( ( isLuma( tu.blocks[ i ].compID ) && luma4x4TUFlag ) || ( isChroma( tu.blocks[ i ].compID ) && chroma4x4TUFlag ) )
      {
        startPos = 8; endPos = 15;
      }
      else if( ( isLuma( tu.blocks[ i ].compID ) && luma8x8TUFlag ) || ( isChroma( tu.blocks[ i ].compID ) && chroma8x8TUFlag ) )
      {
        startPos = 8; endPos = 47;
      }
      const ScanElement *scanPtr = scan + startPos;
      for( uint32_t j = startPos; j <= endPos; j++ )
      {
        count += coeff[ scanPtr->idx ] != 0;
        scanPtr++;
      }
    }
  }
  return count;
}
#endif

bool TU::needsSqrt2Scale( const TransformUnit &tu, const ComponentID &compID )
{
  const Size &size=tu.blocks[compID];
  const bool isTransformSkip = tu.mtsIdx==MTS_SKIP && isLuma(compID);
  return (!isTransformSkip) && (((g_aucLog2[size.width] + g_aucLog2[size.height]) & 1) == 1);
}

#if HM_QTBT_AS_IN_JEM_QUANT

bool TU::needsBlockSizeTrafoScale( const TransformUnit &tu, const ComponentID &compID )
{
  return needsSqrt2Scale( tu, compID ) || isNonLog2BlockSize( tu.blocks[compID] );
}
#else
bool TU::needsQP3Offset(const TransformUnit &tu, const ComponentID &compID)
{
  if( !tu.transformSkip[compID] )
  {
    return ( ( ( g_aucLog2[tu.blocks[compID].width] + g_aucLog2[tu.blocks[compID].height] ) & 1 ) == 1 );
  }
  return false;
}
#endif


TransformUnit* TU::getPrevTU( const TransformUnit &tu, const ComponentID compID )
{
  TransformUnit* prevTU = tu.prev;

  if( prevTU != nullptr && ( prevTU->cu != tu.cu || !prevTU->blocks[compID].valid() ) )
  {
    prevTU = nullptr;
  }

  return prevTU;
}

bool TU::getPrevTuCbfAtDepth( const TransformUnit &currentTu, const ComponentID compID, const int trDepth )
{
  const TransformUnit* prevTU = getPrevTU( currentTu, compID );
  return ( prevTU != nullptr ) ? TU::getCbfAtDepth( *prevTU, compID, trDepth ) : false;
}

#if !JVET_N0866_UNIF_TRFM_SEL_IMPL_MTS_ISP
void TU::getTransformTypeISP( const TransformUnit &tu, const ComponentID compID, int &typeH, int &typeV )
{
  typeH = DCT2, typeV = DCT2;
  const int uiChFinalMode = PU::getFinalIntraMode( *tu.cu->firstPU, toChannelType( compID ) );
  bool intraModeIsEven = uiChFinalMode % 2 == 0;

  if( uiChFinalMode == DC_IDX || uiChFinalMode == 33 || uiChFinalMode == 35 )
  {
    typeH = DCT2;
    typeV = typeH;
  }
  else if( uiChFinalMode == PLANAR_IDX || ( uiChFinalMode >= 31 && uiChFinalMode <= 37 ) )
  {
    typeH = DST7;
    typeV = typeH;
  }
  else if( ( intraModeIsEven && uiChFinalMode >= 2 && uiChFinalMode <= 30 ) || ( !intraModeIsEven && uiChFinalMode >= 39 && uiChFinalMode <= 65 ) )
  {
    typeH = DST7;
    typeV = DCT2;
  }
  else if( ( !intraModeIsEven && uiChFinalMode >= 3 && uiChFinalMode <= 29 ) || ( intraModeIsEven && uiChFinalMode >= 38 && uiChFinalMode <= 66 ) )
  {
    typeH = DCT2;
    typeV = DST7;
  }
  //Size restriction for non-DCT-II transforms
  Area tuArea = tu.blocks[compID];
  typeH = tuArea.width  <= 2 || tuArea.width  >= 32 ? DCT2 : typeH;
  typeV = tuArea.height <= 2 || tuArea.height >= 32 ? DCT2 : typeV;
}

#endif

// other tools

uint32_t getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}

#if JVET_N0217_MATRIX_INTRAPRED
int getNumModesMip(const Size& block)
{
  if (block.width > (4 * block.height) || block.height > (4 * block.width))
  {
    return 0;
  }

  if( block.width == 4 && block.height == 4 )
  {
    return 35;
  }
  else if (block.width <= 8 && block.height <= 8)
  {
    return 19;
  }
  else
  {
    return 11;
  }
}

int getNumEpBinsMip(const Size& block)
{
  int numModes = getNumModesMip(block);
  return int(std::ceil((std::log2(numModes - NUM_MPM_MIP - 1))));
}

bool mipModesAvailable(const Size& block)
{
  return (getNumModesMip(block));
}
#endif



