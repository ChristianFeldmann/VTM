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

/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include "Unit.h"

#include "Buffer.h"
#include "Picture.h"
#include "ChromaFormat.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"

#include "ChromaFormat.h"

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

void CompArea::xRecalcLumaToChroma()
{
  const uint32_t csx = getComponentScaleX(compID, chromaFormat);
  const uint32_t csy = getComponentScaleY(compID, chromaFormat);

  x      >>= csx;
  y      >>= csy;
  width  >>= csx;
  height >>= csy;
}

Position CompArea::chromaPos() const
{
  if (isLuma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width << scaleX, height << scaleY );
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize() const
{
  if( isLuma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width >> scaleX, height >> scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Position( x << scaleX, y << scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::compPos( const ComponentID compID ) const
{
  return isLuma( compID ) ? lumaPos() : chromaPos();
}

Position CompArea::chanPos( const ChannelType chType ) const
{
  return isLuma( chType ) ? lumaPos() : chromaPos();
}

// ---------------------------------------------------------------------------
// unit method definitions
// ---------------------------------------------------------------------------

UnitArea::UnitArea(const ChromaFormat _chromaFormat) : chromaFormat(_chromaFormat) { }

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const Area &_area) : chromaFormat(_chromaFormat), blocks(getNumberValidComponents(_chromaFormat))
{
  const uint32_t numCh = getNumberValidComponents(chromaFormat);

  for (uint32_t i = 0; i < numCh; i++)
  {
    blocks[i] = CompArea(ComponentID(i), chromaFormat, _area, true);
  }
}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY) : chromaFormat(_chromaFormat), blocks { blkY } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY) } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY, const CompArea &blkCb, const CompArea &blkCr)  : chromaFormat(_chromaFormat), blocks { blkY, blkCb, blkCr } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,      CompArea &&blkCb,      CompArea &&blkCr) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY), std::forward<CompArea>(blkCb), std::forward<CompArea>(blkCr) } {}

bool UnitArea::contains(const UnitArea& other) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

bool UnitArea::contains( const UnitArea& other, const ChannelType chType ) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( toChannelType( blk.compID ) == chType && blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
void UnitArea::resizeTo( const UnitArea& unitArea )
{
  for( uint32_t i = 0; i < blocks.size(); i++ )
  {
    blocks[i].resizeTo( unitArea.blocks[i] );
  }
}
#endif

void UnitArea::repositionTo(const UnitArea& unitArea)
{
  for(uint32_t i = 0; i < blocks.size(); i++)
  {
    blocks[i].repositionTo(unitArea.blocks[i]);
  }
}

const UnitArea UnitArea::singleComp(const ComponentID compID) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (blk.compID == compID)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

const UnitArea UnitArea::singleChan(const ChannelType chType) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (toChannelType(blk.compID) == chType)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

// ---------------------------------------------------------------------------
// coding unit method definitions
// ---------------------------------------------------------------------------

CodingUnit::CodingUnit(const UnitArea &unit)                                : UnitArea(unit),                 cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }
CodingUnit::CodingUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }

CodingUnit& CodingUnit::operator=( const CodingUnit& other )
{
  slice             = other.slice;
  predMode          = other.predMode;
  qtDepth           = other.qtDepth;
  depth             = other.depth;
  btDepth           = other.btDepth;
  mtDepth           = other.mtDepth;
  splitSeries       = other.splitSeries;
  skip              = other.skip;
  mmvdSkip = other.mmvdSkip;
  affine            = other.affine;
  affineType        = other.affineType;
  triangle          = other.triangle;
  transQuantBypass  = other.transQuantBypass;
#if JVET_N0413_RDPCM
  bdpcmMode         = other.bdpcmMode;
#endif
  ipcm              = other.ipcm;
  qp                = other.qp;
  chromaQpAdj       = other.chromaQpAdj;
  rootCbf           = other.rootCbf;
  sbtInfo           = other.sbtInfo;
#if JVET_N0193_LFNST
  mtsFlag           = other.mtsFlag;
  lfnstIdx          = other.lfnstIdx;
#endif
  tileIdx           = other.tileIdx;
  imv               = other.imv;
  imvNumCand        = other.imvNumCand;
  GBiIdx            = other.GBiIdx;
  for (int i = 0; i<2; i++)
    refIdxBi[i] = other.refIdxBi[i];

  shareParentPos    = other.shareParentPos;
  shareParentSize   = other.shareParentSize;
  smvdMode        = other.smvdMode;
  ispMode           = other.ispMode;
#if JVET_N0217_MATRIX_INTRAPRED
  mipFlag           = other.mipFlag;
#endif

  return *this;
}

void CodingUnit::initData()
{
  predMode          = NUMBER_OF_PREDICTION_MODES;
  qtDepth           = 0;
  depth             = 0;
  btDepth           = 0;
  mtDepth           = 0;
  splitSeries       = 0;
  skip              = false;
  mmvdSkip = false;
  affine            = false;
  affineType        = 0;
  triangle          = false;
  transQuantBypass  = false;
#if JVET_N0413_RDPCM
  bdpcmMode         = 0;
#endif
  ipcm              = false;
  qp                = 0;
  chromaQpAdj       = 0;
  rootCbf           = true;
  sbtInfo           = 0;
#if JVET_N0193_LFNST
  mtsFlag           = 0;
  lfnstIdx          = 0;
#endif
  tileIdx           = 0;
  imv               = 0;
  imvNumCand        = 0;
  GBiIdx            = GBI_DEFAULT;
  for (int i = 0; i < 2; i++)
    refIdxBi[i] = -1;
  shareParentPos = Position(-1, -1);
  shareParentSize.width = -1;
  shareParentSize.height = -1;
  smvdMode        = 0;
  ispMode           = 0;
#if JVET_N0217_MATRIX_INTRAPRED
  mipFlag           = false;
#endif
}

const uint8_t CodingUnit::checkAllowedSbt() const
{
  if( !slice->getSPS()->getUseSBT() )
  {
    return 0;
  }

  //check on prediction mode
  if( predMode == MODE_INTRA || predMode == MODE_IBC ) //intra or IBC
  {
    return 0;
  }
  if( firstPU->mhIntraFlag )
  {
    return 0;
  }
#if JVET_N0483_DISABLE_SBT_FOR_TPM
  if( triangle )
  {
    return 0;
  }
#endif

  uint8_t sbtAllowed = 0;
  int cuWidth  = lwidth();
  int cuHeight = lheight();
  bool allow_type[NUMBER_SBT_IDX];
  memset( allow_type, false, NUMBER_SBT_IDX * sizeof( bool ) );

  //parameter
  int maxSbtCUSize = cs->sps->getMaxSbtSize();
  int minSbtCUSize = 1 << ( MIN_CU_LOG2 + 1 );

  //check on size
  if( cuWidth > maxSbtCUSize || cuHeight > maxSbtCUSize )
  {
    return 0;
  }

  allow_type[SBT_VER_HALF] = cuWidth  >= minSbtCUSize;
  allow_type[SBT_HOR_HALF] = cuHeight >= minSbtCUSize;
  allow_type[SBT_VER_QUAD] = cuWidth  >= ( minSbtCUSize << 1 );
  allow_type[SBT_HOR_QUAD] = cuHeight >= ( minSbtCUSize << 1 );

  for( int i = 0; i < NUMBER_SBT_IDX; i++ )
  {
    sbtAllowed += (uint8_t)allow_type[i] << i;
  }

  return sbtAllowed;
}

uint8_t CodingUnit::getSbtTuSplit() const
{
  uint8_t sbtTuSplitType = 0;

  switch( getSbtIdx() )
  {
  case SBT_VER_HALF: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_VER_HALF_POS0_SPLIT; break;
  case SBT_HOR_HALF: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_HOR_HALF_POS0_SPLIT; break;
  case SBT_VER_QUAD: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_VER_QUAD_POS0_SPLIT; break;
  case SBT_HOR_QUAD: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_HOR_QUAD_POS0_SPLIT; break;
  default: assert( 0 );  break;
  }

  assert( sbtTuSplitType <= SBT_HOR_QUAD_POS1_SPLIT && sbtTuSplitType >= SBT_VER_HALF_POS0_SPLIT );
  return sbtTuSplitType;
}

// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

PredictionUnit::PredictionUnit(const UnitArea &unit)                                : UnitArea(unit)                , cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }
PredictionUnit::PredictionUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }

void PredictionUnit::initData()
{
  // intra data - need this default initialization for PCM
  intraDir[0] = DC_IDX;
  intraDir[1] = PLANAR_IDX;
  multiRefIdx = 0;

  // inter data
  mergeFlag   = false;
#if JVET_N0324_REGULAR_MRG_FLAG
  regularMergeFlag = false;
#endif
  mergeIdx    = MAX_UCHAR;
  triangleSplitDir  = MAX_UCHAR;
  triangleMergeIdx0 = MAX_UCHAR;
  triangleMergeIdx1 = MAX_UCHAR;
  mmvdMergeFlag = false;
  mmvdMergeIdx = MAX_UINT;
  interDir    = MAX_UCHAR;
  mergeType   = MRG_TYPE_DEFAULT_N;
  bv.setZero();
  bvd.setZero();
  mvRefine = false;
  for (uint32_t i = 0; i < MAX_NUM_SUBCU_DMVR; i++)
  {
    mvdL0SubPu[i].setZero();
  }
  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i] = MAX_UCHAR;
    mvpNum[i] = MAX_UCHAR;
    refIdx[i] = -1;
    mv[i]     .setZero();
    mvd[i]    .setZero();
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j].setZero();
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j].setZero();
    }
  }
  mhIntraFlag = false;
  shareParentPos = Position(-1, -1);
  shareParentSize.width = -1;
  shareParentSize.height = -1;
  mmvdEncOptMode = 0;
}

PredictionUnit& PredictionUnit::operator=(const IntraPredictionData& predData)
{
  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    intraDir[i] = predData.intraDir[i];
  }
  multiRefIdx = predData.multiRefIdx;

  return *this;
}

PredictionUnit& PredictionUnit::operator=(const InterPredictionData& predData)
{
  mergeFlag   = predData.mergeFlag;
#if JVET_N0324_REGULAR_MRG_FLAG
  regularMergeFlag = predData.regularMergeFlag;
#endif
  mergeIdx    = predData.mergeIdx;
  triangleSplitDir  = predData.triangleSplitDir  ;
  triangleMergeIdx0 = predData.triangleMergeIdx0 ;
  triangleMergeIdx1 = predData.triangleMergeIdx1 ;
  mmvdMergeFlag = predData.mmvdMergeFlag;
  mmvdMergeIdx = predData.mmvdMergeIdx;
  interDir    = predData.interDir;
  mergeType   = predData.mergeType;
  bv          = predData.bv;
  bvd         = predData.bvd;
  mvRefine = predData.mvRefine;
  for (uint32_t i = 0; i < MAX_NUM_SUBCU_DMVR; i++)
  {
    mvdL0SubPu[i] = predData.mvdL0SubPu[i];
  }
  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i]   = predData.mvpIdx[i];
    mvpNum[i]   = predData.mvpNum[i];
    mv[i]       = predData.mv[i];
    mvd[i]      = predData.mvd[i];
    refIdx[i]   = predData.refIdx[i];
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j] = predData.mvdAffi[i][j];
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j] = predData.mvAffi[i][j];
    }
  }
  mhIntraFlag = predData.mhIntraFlag;
  shareParentPos = predData.shareParentPos;
  shareParentSize = predData.shareParentSize;
  return *this;
}

PredictionUnit& PredictionUnit::operator=( const PredictionUnit& other )
{
  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    intraDir[ i ] = other.intraDir[ i ];
  }
  multiRefIdx = other.multiRefIdx;

  mergeFlag   = other.mergeFlag;
#if JVET_N0324_REGULAR_MRG_FLAG
  regularMergeFlag = other.regularMergeFlag;
#endif
  mergeIdx    = other.mergeIdx;
  triangleSplitDir  = other.triangleSplitDir  ;
  triangleMergeIdx0 = other.triangleMergeIdx0 ;
  triangleMergeIdx1 = other.triangleMergeIdx1 ;
  mmvdMergeFlag = other.mmvdMergeFlag;
  mmvdMergeIdx = other.mmvdMergeIdx;
  interDir    = other.interDir;
  mergeType   = other.mergeType;
  bv          = other.bv;
  bvd         = other.bvd;
  mvRefine = other.mvRefine;
  for (uint32_t i = 0; i < MAX_NUM_SUBCU_DMVR; i++)
  {
    mvdL0SubPu[i] = other.mvdL0SubPu[i];
  }
  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i]   = other.mvpIdx[i];
    mvpNum[i]   = other.mvpNum[i];
    mv[i]       = other.mv[i];
    mvd[i]      = other.mvd[i];
    refIdx[i]   = other.refIdx[i];
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j] = other.mvdAffi[i][j];
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j] = other.mvAffi[i][j];
    }
  }
  mhIntraFlag = other.mhIntraFlag;
  shareParentPos = other.shareParentPos;
  shareParentSize = other.shareParentSize;
  return *this;
}

PredictionUnit& PredictionUnit::operator=( const MotionInfo& mi )
{
  interDir = mi.interDir;

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    refIdx[i] = mi.refIdx[i];
    mv    [i] = mi.mv[i];
  }

  return *this;
}

const MotionInfo& PredictionUnit::getMotionInfo() const
{
  return cs->getMotionInfo( lumaPos() );
}

const MotionInfo& PredictionUnit::getMotionInfo( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getMotionInfo( pos );
}

MotionBuf PredictionUnit::getMotionBuf()
{
  return cs->getMotionBuf( *this );
}

CMotionBuf PredictionUnit::getMotionBuf() const
{
  return cs->getMotionBuf( *this );
}


// ---------------------------------------------------------------------------
// transform unit method definitions
// ---------------------------------------------------------------------------

TransformUnit::TransformUnit(const UnitArea& unit) : UnitArea(unit), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
    m_pcmbuf[i] = nullptr;
  }

  initData();
}

TransformUnit::TransformUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
    m_pcmbuf[i] = nullptr;
  }

  initData();
}

void TransformUnit::initData()
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    cbf[i]           = 0;
    rdpcm[i]         = NUMBER_OF_RDPCM_MODES;
    compAlpha[i]     = 0;
  }
  depth              = 0;
  mtsIdx             = MTS_DCT2_DCT2;
  noResidual         = false;
#if JVET_N0054_JOINT_CHROMA
  jointCbCr          = 0;
#endif
  m_chromaResScaleInv = 0;
}

void TransformUnit::init(TCoeff **coeffs, Pel **pcmbuf)
{
  uint32_t numBlocks = getNumberValidTBlocks(*cs->pcv);

  for (uint32_t i = 0; i < numBlocks; i++)
  {
    m_coeffs[i] = coeffs[i];
    m_pcmbuf[i] = pcmbuf[i];
  }
}

TransformUnit& TransformUnit::operator=(const TransformUnit& other)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  unsigned numBlocks = ::getNumberValidTBlocks(*cs->pcv);
  for( unsigned i = 0; i < numBlocks; i++ )
  {
    CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

    uint32_t area = blocks[i].area();

    if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
    if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);

    cbf[i]           = other.cbf[i];
    rdpcm[i]         = other.rdpcm[i];
    compAlpha[i]     = other.compAlpha[i];
  }
  depth              = other.depth;
  mtsIdx             = other.mtsIdx;
  noResidual         = other.noResidual;
#if JVET_N0054_JOINT_CHROMA
  jointCbCr          = other.jointCbCr;
#endif
  return *this;
}

void TransformUnit::copyComponentFrom(const TransformUnit& other, const ComponentID i)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

  uint32_t area = blocks[i].area();

  if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
  if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);

  cbf[i]           = other.cbf[i];
  rdpcm[i]         = other.rdpcm[i];
  compAlpha[i]     = other.compAlpha[i];

  depth            = other.depth;
  mtsIdx           = isLuma( i ) ? other.mtsIdx : mtsIdx;
  noResidual       = other.noResidual;
#if JVET_N0054_JOINT_CHROMA
  jointCbCr        = isChroma( i ) ? other.jointCbCr : jointCbCr;
#endif
}

       CoeffBuf TransformUnit::getCoeffs(const ComponentID id)       { return  CoeffBuf(m_coeffs[id], blocks[id]); }
const CCoeffBuf TransformUnit::getCoeffs(const ComponentID id) const { return CCoeffBuf(m_coeffs[id], blocks[id]); }

       PelBuf   TransformUnit::getPcmbuf(const ComponentID id)       { return  PelBuf  (m_pcmbuf[id], blocks[id]); }
const CPelBuf   TransformUnit::getPcmbuf(const ComponentID id) const { return CPelBuf  (m_pcmbuf[id], blocks[id]); }

void TransformUnit::checkTuNoResidual( unsigned idx )
{
  if( CU::getSbtIdx( cu->sbtInfo ) == SBT_OFF_DCT )
  {
    return;
  }

  if( ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS0 && idx == 1 ) || ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS1 && idx == 0 ) )
  {
    noResidual = true;
  }
}
int          TransformUnit::getChromaAdj()                     const { return m_chromaResScaleInv; }
void         TransformUnit::setChromaAdj(int i)                      { m_chromaResScaleInv = i;    }
