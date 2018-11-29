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

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"

#include <memory.h>
#include <algorithm>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
, m_maxCompIDToPred ( MAX_NUM_COMPONENT )
, m_pcRdCost        ( nullptr )
#if JVET_L0265_AFF_MINIMUM4X4
, m_storedMv        ( nullptr )
#endif
#if JVET_L0256_BIO
, m_gradX0(nullptr)
, m_gradY0(nullptr)
, m_gradX1(nullptr)
, m_gradY1(nullptr)
, m_subPuMC(false)
#endif
{
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      xFree( m_acYuvPred[i][c] );
      m_acYuvPred[i][c] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

#if JVET_L0124_L0208_TRIANGLE
  m_triangleBuf.destroy();
#endif

#if JVET_L0265_AFF_MINIMUM4X4
  if (m_storedMv != nullptr)
  {
    delete[]m_storedMv;
    m_storedMv = nullptr;
  }
#endif

#if JVET_L0256_BIO
  xFree(m_gradX0);   m_gradX0 = nullptr;
  xFree(m_gradY0);   m_gradY0 = nullptr;
  xFree(m_gradX1);   m_gradX1 = nullptr;
  xFree(m_gradY1);   m_gradY1 = nullptr;
#endif
}

void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chromaFormatIDC )
{
  m_pcRdCost = pcRdCost;


  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] != nullptr && m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] == nullptr ) // check if first is null (in which case, nothing initialised yet)
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
#if JVET_L0256_BIO
      int extWidth = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 16;
      int extHeight = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 1;
#else
      int extWidth  = MAX_CU_SIZE + 16;
      int extHeight = MAX_CU_SIZE + 1;
#endif
      for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++ )
      {
        m_filteredBlockTmp[i][c] = ( Pel* ) xMalloc( Pel, ( extWidth + 4 ) * ( extHeight + 7 + 4 ) );

        for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++ )
        {
          m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
        }
      }

      // new structure
      for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        m_acYuvPred[i][c] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
    }

#if JVET_L0124_L0208_TRIANGLE
    m_triangleBuf.create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
#endif

    m_iRefListIdx = -1;
  
#if JVET_L0256_BIO
    m_gradX0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradX1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
#endif
  }

#if !JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_if.initInterpolationFilter( true );
#endif

#if JVET_L0265_AFF_MINIMUM4X4
  if (m_storedMv == nullptr)
  {
    const int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;
    m_storedMv = new Mv[MVBUFFER_SIZE*MVBUFFER_SIZE];
  }
#endif
}

bool checkIdenticalMotion( const PredictionUnit &pu, bool checkAffine )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !pu.cu->affine )
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
        else
        {
          CHECK( !checkAffine, "In this case, checkAffine should be on." );
#if JVET_L0694_AFFINE_LINEBUFFER_CLEANUP
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]) && (pu.mvAffi[0][2] == pu.mvAffi[1][2])) )
#else
          const CMotionBuf &mb = pu.getMotionBuf();
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1])) )
#endif
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !pu.cu->affine )
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
        else
        {
#if JVET_L0694_AFFINE_LINEBUFFER_CLEANUP
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]) && (pu.mvAffi[0][2] == pu.mvAffi[1][2])) )
#else
          const CMotionBuf &mb = pu.getMotionBuf();
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (mb.at( 0, 0 ).mv[0] == mb.at( 0, 0 ).mv[1]) && (mb.at( mb.width - 1, 0 ).mv[0] == mb.at( mb.width - 1, 0 ).mv[1]) && (mb.at( 0, mb.height - 1 ).mv[0] == mb.at( 0, mb.height - 1 ).mv[1])) )
#endif
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void InterPrediction::xSubPuMC( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ )
{

  // compute the location of the current PU
  Position puPos    = pu.lumaPos();
  Size puSize       = pu.lumaSize();

  int numPartLine, numPartCol, puHeight, puWidth;
  {
#if JVET_L0198_L0468_L0104_ATMVP_8x8SUB_BLOCK
    numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
    numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
    puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
    puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
#else 
    const Slice& slice = *pu.cs->slice;
    numPartLine = std::max(puSize.width >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    numPartCol  = std::max(puSize.height >> slice.getSubPuMvpSubblkLog2Size(), 1u);
    puHeight    = numPartCol == 1 ? puSize.height : 1 << slice.getSubPuMvpSubblkLog2Size();
    puWidth     = numPartLine == 1 ? puSize.width : 1 << slice.getSubPuMvpSubblkLog2Size();
#endif 
  }

  PredictionUnit subPu;

  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

#if JVET_L0369_SUBBLOCK_MERGE
  bool isAffine = pu.cu->affine;
  subPu.cu->affine = false;
#endif

  // join sub-pus containing the same motion
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

#if JVET_L0256_BIO
  m_subPuMC = true;
#endif

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later  = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

      motionCompensation(subPu, subPredBuf, eRefPicList);
      secDim = later - secStep;
    }
  }
#if JVET_L0256_BIO
  m_subPuMC = false;
#endif

#if JVET_L0369_SUBBLOCK_MERGE
  pu.cu->affine = isAffine;
#endif
}

#if JVET_L0293_CPR
void InterPrediction::xChromaMC(PredictionUnit &pu, PelUnitBuf& pcYuvPred)
{
  // separated tree, chroma
  const CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
  PredictionUnit subPu;
  subPu.cs = pu.cs;
  subPu.cu = pu.cu;

  Picture * refPic = pu.cu->slice->getPic();
  for (int y = lumaArea.y; y < lumaArea.y + lumaArea.height; y += MIN_PU_SIZE)
  {
    for (int x = lumaArea.x; x < lumaArea.x + lumaArea.width; x += MIN_PU_SIZE)
    {
      const MotionInfo &curMi = pu.cs->picture->cs->getMotionInfo(Position{ x, y });

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, MIN_PU_SIZE, MIN_PU_SIZE)));
      PelUnitBuf subPredBuf = pcYuvPred.subBuf(UnitAreaRelative(pu, subPu));

      xPredInterBlk(COMPONENT_Cb, subPu, refPic, curMi.mv[0], subPredBuf, false, pu.cu->slice->clpRng(COMPONENT_Cb)
#if JVET_L0256_BIO
                    , false
#endif
                    , true);
      xPredInterBlk(COMPONENT_Cr, subPu, refPic, curMi.mv[0], subPredBuf, false, pu.cu->slice->clpRng(COMPONENT_Cr)
#if JVET_L0256_BIO
                    , false
#endif
                    , true);
    }
  }
}
#endif


void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi 
#if JVET_L0256_BIO
                                   ,const bool& bioApplied /*=false*/
#endif
#if JVET_L0293_CPR
  , const bool luma, const bool chroma
#endif
)
{
  const SPS &sps = *pu.cs->sps;

  int iRefIdx = pu.refIdx[eRefPicList];
  Mv mv[3];
#if JVET_L0293_CPR 
  bool isCPR = false;
  if (pu.cs->slice->getRefPic(eRefPicList, iRefIdx)->getPOC() == pu.cs->slice->getPOC())
  {
    isCPR = true;
  }
#endif
  if( pu.cu->affine )
  {
    CHECK( iRefIdx < 0, "iRefIdx incorrect." );

#if JVET_L0694_AFFINE_LINEBUFFER_CLEANUP
    mv[0] = pu.mvAffi[eRefPicList][0];
    mv[1] = pu.mvAffi[eRefPicList][1];
    mv[2] = pu.mvAffi[eRefPicList][2];
#else
    const CMotionBuf &mb = pu.getMotionBuf();
    mv[0] = mb.at( 0,            0             ).mv[eRefPicList];
    mv[1] = mb.at( mb.width - 1, 0             ).mv[eRefPicList];
    mv[2] = mb.at( 0,            mb.height - 1 ).mv[eRefPicList];
#endif
  }
  else
  {
    mv[0] = pu.mv[eRefPicList];
  }
  if ( !pu.cu->affine )
  clipMv(mv[0], pu.cu->lumaPos(), sps);


  for( uint32_t comp = COMPONENT_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );
#if JVET_L0293_CPR
    if (compID == COMPONENT_Y && !luma)
      continue;
    if (compID != COMPONENT_Y && !chroma)
      continue;
#endif
    if ( pu.cu->affine )
    {
#if JVET_L0256_BIO
      CHECK( bioApplied, "BIO is not allowed with affine" );
#endif
      xPredAffineBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv, pcYuvPred, bi, pu.cu->slice->clpRng( compID ) );
    }
    else
    {
      xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng( compID )
#if JVET_L0256_BIO
                    , bioApplied
#endif
#if JVET_L0293_CPR
                    , isCPR
#endif
                    );

    }
  }
}

void InterPrediction::xPredInterBi(PredictionUnit& pu, PelUnitBuf &pcYuvPred)
{
  const PPS   &pps   = *pu.cs->pps;
  const Slice &slice = *pu.cs->slice;

#if JVET_L0256_BIO
  bool bioApplied = false;
  if (pu.cs->sps->getSpsNext().getUseBIO())
  {
    if (pu.cu->affine || m_subPuMC)
    {
      bioApplied = false;
    }
    else
    {
      const bool biocheck0 = !(pps.getWPBiPred() && slice.getSliceType() == B_SLICE);
      const bool biocheck1 = !(pps.getUseWP() && slice.getSliceType() == P_SLICE);
      if (biocheck0
        && biocheck1
        && PU::isBiPredFromDifferentDir(pu)
        && !(pu.Y().height == 4 || (pu.Y().width == 4 && pu.Y().height == 8))
       )
      {
        bioApplied = true;
      }
    }

#if JVET_L0646_GBI
    if (pu.cu->cs->sps->getSpsNext().getUseGBi() && bioApplied && pu.cu->GBiIdx != GBI_DEFAULT)
    {
      bioApplied = false;
    }
#endif
  }
#endif

  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( pu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    CHECK( pu.refIdx[refList] >= slice.getNumRefIdx( eRefPicList ), "Invalid reference index" );
    m_iRefListIdx = refList;

    PelUnitBuf pcMbBuf = ( pu.chromaFormat == CHROMA_400 ?
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y())) :
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[refList][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[refList][2], pcYuvPred.Cr())) );

    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
      xPredInterUni ( pu, eRefPicList, pcMbBuf, true
#if JVET_L0256_BIO
                     ,bioApplied 
#endif
                     );
    }
    else
    {
      if( ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, true );
      }
      else
      {
#if JVET_L0124_L0208_TRIANGLE
        xPredInterUni( pu, eRefPicList, pcMbBuf, pu.cu->triangle );
#else
        xPredInterUni ( pu, eRefPicList, pcMbBuf, false );
#endif
      }
    }
  }


  CPelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[0][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvPred.Cr())) );
  CPelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[1][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvPred.Cr())) );
  if( pps.getWPBiPred() && slice.getSliceType() == B_SLICE )
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred );
  }
  else if( pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred );
  }
  else
  {
#if JVET_L0256_BIO
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs(), bioApplied );
#else
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs() );
#endif
  }
}

void InterPrediction::xPredInterBlk ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
#if JVET_L0256_BIO
                                     , const bool& bioApplied
#endif
#if JVET_L0293_CPR
                                     , bool isCPR
#endif
                                    )
{
  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat  chFmt = pu.chromaFormat;
  const bool          rndRes = !bi;

  int iAddPrecShift = 0;

#if !REMOVE_MV_ADAPT_PREC
  if (_mv.highPrec)
  {
    CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv(), "Found a high-precision motion vector, but the high-precision MV extension is disabled!");
#endif
    iAddPrecShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#if !REMOVE_MV_ADAPT_PREC
  }
#endif

  int shiftHor = 2 + iAddPrecShift + ::getComponentScaleX(compID, chFmt);
  int shiftVer = 2 + iAddPrecShift + ::getComponentScaleY(compID, chFmt);

  int xFrac = _mv.hor & ((1 << shiftHor) - 1);
  int yFrac = _mv.ver & ((1 << shiftVer) - 1);
#if JVET_L0293_CPR 
  if (isCPR)
  {
    xFrac = yFrac = 0;
    JVET_J0090_SET_CACHE_ENABLE( false );
  }
#endif
  xFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
  yFrac <<= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE - iAddPrecShift;
#if !REMOVE_MV_ADAPT_PREC
  CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv() && ((xFrac & 3) != 0), "Invalid fraction");
  CHECKD(!pu.cs->sps->getSpsNext().getUseHighPrecMv() && ((yFrac & 3) != 0), "Invalid fraction");
#endif

  PelBuf &dstBuf  = dstPic.bufs[compID];
  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  CPelBuf refBuf;
  {
    Position offset = pu.blocks[compID].pos().offset( _mv.getHor() >> shiftHor, _mv.getVer() >> shiftVer );
    refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, pu.blocks[compID].size() ) );
  }

#if JVET_L0256_BIO
  // backup data
  int backupWidth = width;
  int backupHeight = height;
  Pel *backupDstBufPtr = dstBuf.buf;
  int backupDstBufStride = dstBuf.stride;

  if (bioApplied && compID == COMPONENT_Y)
  {
    width = width + 2 * BIO_EXTEND_SIZE + 2;
    height = height + 2 * BIO_EXTEND_SIZE + 2;

    // change MC output
    dstBuf.stride = width;
    dstBuf.buf = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 2;
  }
#endif

  if( yFrac == 0 )
  {
#if JVET_L0256_BIO
    m_if.filterHor(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, xFrac, rndRes, chFmt, clpRng);
#else
    m_if.filterHor(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, rndRes, chFmt, clpRng);
#endif
  }
  else if( xFrac == 0 )
  {
#if JVET_L0256_BIO
    m_if.filterVer(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, true, rndRes, chFmt, clpRng);
#else
    m_if.filterVer(compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, rndRes, chFmt, clpRng);
#endif
  }
  else
  {
    PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
#if JVET_L0256_BIO
    tmpBuf.stride = dstBuf.stride;
#endif

    int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
#if JVET_L0256_BIO
    m_if.filterHor(compID, (Pel*)refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, backupWidth, backupHeight + vFilterSize - 1, xFrac, false, chFmt, clpRng);
#else
    m_if.filterHor(compID, (Pel*) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, width, height + vFilterSize - 1, xFrac, false,         chFmt, clpRng);
#endif
    JVET_J0090_SET_CACHE_ENABLE( false );
#if JVET_L0256_BIO
    m_if.filterVer(compID, (Pel*)tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, false, rndRes, chFmt, clpRng);
#else
    m_if.filterVer(compID, (Pel*) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height,                   yFrac, false, rndRes, chFmt, clpRng);
#endif
  }
  JVET_J0090_SET_CACHE_ENABLE( true );
#if JVET_L0256_BIO
  if (bioApplied && compID == COMPONENT_Y)
  {
    refBuf.buf = refBuf.buf - refBuf.stride - 1;
    dstBuf.buf = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + dstBuf.stride + 1;
    bioSampleExtendBilinearFilter(refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width - 2, height - 2, 1, xFrac, yFrac, rndRes, chFmt, clpRng);

    // restore data 
    width = backupWidth;
    height = backupHeight;
    dstBuf.buf = backupDstBufPtr;
    dstBuf.stride = backupDstBufStride;
  }
#endif
}

void InterPrediction::xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng )
{
  if ( (pu.cu->affineType == AFFINEMODEL_6PARAM && _mv[0] == _mv[1] && _mv[0] == _mv[2])
    || (pu.cu->affineType == AFFINEMODEL_4PARAM && _mv[0] == _mv[1])
    )
  {
    Mv mvTemp = _mv[0];
    clipMv( mvTemp, pu.cu->lumaPos(), *pu.cs->sps );
    xPredInterBlk( compID, pu, refPic, mvTemp, dstPic, bi, clpRng
#if JVET_L0256_BIO
                  , false
#endif
#if JVET_L0293_CPR
                  , false
#endif
                  );
    return;
  }

  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat chFmt = pu.chromaFormat;
  int iScaleX = ::getComponentScaleX( compID, chFmt );
  int iScaleY = ::getComponentScaleY( compID, chFmt );

  Mv mvLT =_mv[0];
  Mv mvRT =_mv[1];
  Mv mvLB =_mv[2];

#if !REMOVE_MV_ADAPT_PREC
  mvLT.setHighPrec();
  mvRT.setHighPrec();
  mvLB.setHighPrec();
#endif

  // get affine sub-block width and height
  const int width  = pu.Y().width;
  const int height = pu.Y().height;
  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;

  blockWidth  >>= iScaleX;
  blockHeight >>= iScaleY;

 #if JVET_L0265_AFF_MINIMUM4X4
  blockWidth =  std::max(blockWidth, AFFINE_MIN_BLOCK_SIZE);
  blockHeight = std::max(blockHeight, AFFINE_MIN_BLOCK_SIZE);

  CHECK(blockWidth  > (width >> iScaleX ), "Sub Block width  > Block width");
  CHECK(blockHeight > (height >> iScaleX), "Sub Block height > Block height");
  const int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;
#endif

  const int cxWidth  = width  >> iScaleX;
  const int cxHeight = height >> iScaleY;
  const int iHalfBW  = blockWidth  >> 1;
  const int iHalfBH  = blockHeight >> 1;

  const int iBit = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
  iDMvHorX = (mvRT - mvLT).getHor() << (iBit - g_aucLog2[cxWidth]);
  iDMvHorY = (mvRT - mvLT).getVer() << (iBit - g_aucLog2[cxWidth]);
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (iBit - g_aucLog2[cxHeight]);
    iDMvVerY = (mvLB - mvLT).getVer() << (iBit - g_aucLog2[cxHeight]);
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << iBit;
  int iMvScaleVer = mvLT.getVer() << iBit;
  const SPS &sps    = *pu.cs->sps;
  const int iMvShift = 4;
  const int iOffset  = 8;
  const int iHorMax = ( sps.getPicWidthInLumaSamples()     + iOffset -      pu.Y().x - 1 ) << iMvShift;
  const int iHorMin = (      -(int)pu.cs->pcv->maxCUWidth  - iOffset - (int)pu.Y().x + 1 ) << iMvShift;
  const int iVerMax = ( sps.getPicHeightInLumaSamples()    + iOffset -      pu.Y().y - 1 ) << iMvShift;
  const int iVerMin = (      -(int)pu.cs->pcv->maxCUHeight - iOffset - (int)pu.Y().y + 1 ) << iMvShift;

  PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
  const int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

  const int shift = iBit - 4 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE + 2;

  // get prediction block by block
  for ( int h = 0; h < cxHeight; h += blockHeight )
  {
    for ( int w = 0; w < cxWidth; w += blockWidth )
    {

#if JVET_L0265_AFF_MINIMUM4X4
      int iMvScaleTmpHor, iMvScaleTmpVer;
      if(compID == COMPONENT_Y)
      {
        iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (iHalfBW + w) + iDMvVerX * (iHalfBH + h);
        iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (iHalfBW + w) + iDMvVerY * (iHalfBH + h);
        roundAffineMv(iMvScaleTmpHor, iMvScaleTmpVer, shift);

        // clip and scale
        iMvScaleTmpHor = std::min<int>(iHorMax, std::max<int>(iHorMin, iMvScaleTmpHor));
        iMvScaleTmpVer = std::min<int>(iVerMax, std::max<int>(iVerMin, iMvScaleTmpVer));

        m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(iMvScaleTmpHor, iMvScaleTmpVer);
      }
      else
      {
        Mv curMv = (m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE) * MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)] +
          Mv(2, 2));
        curMv.set(curMv.getHor() >> 2, curMv.getVer() >> 2);     
        iMvScaleTmpHor = curMv.hor;
        iMvScaleTmpVer = curMv.ver;
      }
#else
      int iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (iHalfBW + w) + iDMvVerX * (iHalfBH + h);
      int iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (iHalfBW + w) + iDMvVerY * (iHalfBH + h);
      roundAffineMv( iMvScaleTmpHor, iMvScaleTmpVer, shift );

      // clip and scale
      iMvScaleTmpHor = std::min<int>( iHorMax, std::max<int>( iHorMin, iMvScaleTmpHor ) );
      iMvScaleTmpVer = std::min<int>( iVerMax, std::max<int>( iVerMin, iMvScaleTmpVer ) );
#endif
      // get the MV in high precision
      int xFrac, yFrac, xInt, yInt;

      if (!iScaleX)
      {
        xInt  = iMvScaleTmpHor >> 4;
        xFrac = iMvScaleTmpHor & 15;
      }
      else
      {
        xInt  = iMvScaleTmpHor >> 5;
        xFrac = iMvScaleTmpHor & 31;
      }
      if (!iScaleY)
      {
        yInt  = iMvScaleTmpVer >> 4;
        yFrac = iMvScaleTmpVer & 15;
      }
      else
      {
        yInt  = iMvScaleTmpVer >> 5;
        yFrac = iMvScaleTmpVer & 31;
      }

      const CPelBuf refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, pu.blocks[compID].offset(xInt + w, yInt + h), pu.blocks[compID] ) );
      PelBuf &dstBuf = dstPic.bufs[compID];

      if ( yFrac == 0 )
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, xFrac, !bi, chFmt, clpRng );
      }
      else if ( xFrac == 0 )
      {
        m_if.filterVer( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, true, !bi, chFmt, clpRng );
      }
      else
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf - ((vFilterSize>>1) -1)*refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, blockWidth, blockHeight+vFilterSize-1, xFrac, false,      chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( false );
        m_if.filterVer( compID, tmpBuf.buf + ((vFilterSize>>1) -1)*tmpBuf.stride, tmpBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, false, !bi, chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( true );
      }
    }
  }
}

int getMSB( unsigned x )
{
  int msb = 0, bits = ( sizeof(int) << 3 ), y = 1;
  while( x > 1u )
  {
    bits >>= 1;
    y      = x >> bits;
    if( y )
    {
      x    = y;
      msb += bits;
    }
  }
  msb += y;
  return msb;
}

#if JVET_L0256_BIO
void InterPrediction::applyBiOptFlow(const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths)
{
  const int     height = yuvDst.Y().height;
  const int     width = yuvDst.Y().width;
  int           heightG = height + 2 * BIO_EXTEND_SIZE;
  int           widthG = width + 2 * BIO_EXTEND_SIZE;
  int           offsetPos = widthG*BIO_EXTEND_SIZE + BIO_EXTEND_SIZE;

  Pel*          gradX0 = m_gradX0;
  Pel*          gradX1 = m_gradX1;
  Pel*          gradY0 = m_gradY0;
  Pel*          gradY1 = m_gradY1;

  int           stridePredMC = widthG + 2;
  const Pel*    srcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + stridePredMC + 1;
  const Pel*    srcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + stridePredMC + 1;
  const int     src0Stride = stridePredMC;
  const int     src1Stride = stridePredMC;

  Pel*          dstY = yuvDst.Y().buf;
  const int     dstStride = yuvDst.Y().stride;
  const Pel*    srcY0Temp = srcY0;
  const Pel*    srcY1Temp = srcY1;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    Pel* dstTempPtr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + stridePredMC + 1;
    Pel* gradY = (refList == 0) ? m_gradY0 : m_gradY1;
    Pel* gradX = (refList == 0) ? m_gradX0 : m_gradX1;

    xBioGradFilter(dstTempPtr, stridePredMC, widthG, heightG, widthG, gradX, gradY);
    Pel* padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 2;
    for (int y = 0; y< height; y++)
    {
      padStr[-1] = padStr[0];
      padStr[width] = padStr[width - 1];
      padStr += stridePredMC;
    }

    padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 1;
    ::memcpy(padStr - stridePredMC, padStr, sizeof(Pel)*(widthG));
    ::memcpy(padStr + height*stridePredMC, padStr + (height - 1)*stridePredMC, sizeof(Pel)*(widthG));
  }

  const ClpRng& clpRng = pu.cu->cs->slice->clpRng(COMPONENT_Y);
  const int   bitDepth = clipBitDepths.recon[toChannelType(COMPONENT_Y)];
  const int   shiftNum = IF_INTERNAL_PREC + 1 - bitDepth;
  const int   offset = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;
  const int   limit = ((int)1 << (4 + IF_INTERNAL_PREC - bitDepth - 5));

  int*     dotProductTemp1 = m_dotProduct1;
  int*     dotProductTemp2 = m_dotProduct2;
  int*     dotProductTemp3 = m_dotProduct3;
  int*     dotProductTemp5 = m_dotProduct5;
  int*     dotProductTemp6 = m_dotProduct6;

  xCalcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, widthG, widthG, heightG);

  int xUnit = (width >> 2);
  int yUnit = (height >> 2);

  Pel *dstY0 = dstY;
  gradX0 = m_gradX0; gradX1 = m_gradX1;
  gradY0 = m_gradY0; gradY1 = m_gradY1;

  for (int yu = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++)
    {
      if (m_bioPredSubBlkDist[yu*xUnit + xu] < m_bioSubBlkDistThres)
      {
        srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
        srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*src1Stride + xu) << 2);
        dstY0 = dstY + ((yu*dstStride + xu) << 2);
        PelBuf dstPelBuf(dstY0, dstStride, Size(4, 4));
        dstPelBuf.addAvg(CPelBuf(srcY0Temp, src0Stride, Size(4, 4)), CPelBuf(srcY1Temp, src1Stride, Size(4, 4)), clpRng);
        continue;
      }

      int     sGxdI = 0, sGydI = 0, sGxGy = 0, sGx2 = 0, sGy2 = 0;
      int     tmpx = 0, tmpy = 0;

      dotProductTemp1 = m_dotProduct1 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp2 = m_dotProduct2 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp3 = m_dotProduct3 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp5 = m_dotProduct5 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp6 = m_dotProduct6 + offsetPos + ((yu*widthG + xu) << 2);

      xCalcBlkGradient(xu << 2, yu << 2, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, sGx2, sGy2, sGxGy, sGxdI, sGydI, widthG, heightG, (1 << 2));

      if (sGx2 > 0)
      {
        tmpx = rightShiftMSB(sGxdI << 3, sGx2);
        tmpx = Clip3(-limit, limit, tmpx);
      }
      if (sGy2 > 0)
      {
        int     mainsGxGy = sGxGy >> 12;
        int     secsGxGy = sGxGy & ((1 << 12) - 1);
        int     tmpData = tmpx * mainsGxGy;
        tmpData = ((tmpData << 12) + tmpx*secsGxGy) >> 1;
        tmpy = rightShiftMSB(((sGydI << 3) - tmpData), sGy2);
        tmpy = Clip3(-limit, limit, tmpy);
      }

      srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      gradX0 = m_gradX0 + offsetPos + ((yu*widthG + xu) << 2);
      gradX1 = m_gradX1 + offsetPos + ((yu*widthG + xu) << 2);
      gradY0 = m_gradY0 + offsetPos + ((yu*widthG + xu) << 2);
      gradY1 = m_gradY1 + offsetPos + ((yu*widthG + xu) << 2);

      dstY0 = dstY + ((yu*dstStride + xu) << 2);
      xAddBIOAvg4(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY0, dstStride, gradX0, gradX1, gradY0, gradY1, widthG, (1 << 2), (1 << 2), (int)tmpx, (int)tmpy, shiftNum, offset, clpRng);
    }  // xu
  }  // yu
}

void InterPrediction::bioSampleExtendBilinearFilter(Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int dim, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng)
{
  Pel const* pSrc = NULL;
  Pel*       pDst = NULL;

  int vFilterSize = NTAPS_LUMA;
  int widthTmp = 0;
  int heightTmp = 0;

  for (int cand = 0; cand < 4; cand++)  // top, left, bottom and right
  {

    if (cand == 0)  // top
    {
      pSrc = src;
      pDst = dst;
      widthTmp = width;
      heightTmp = dim;
    }
    else if (cand == 1)  // left
    {
      pSrc = src + dim*srcStride;
      pDst = dst + dim*dstStride;
      widthTmp = dim;
      heightTmp = height - 2 * dim;
    }
    else if (cand == 2)  // bottom
    {
      pSrc = src + (height - dim)*srcStride;
      pDst = dst + (height - dim)*dstStride;
      widthTmp = width;
      heightTmp = dim;
    }
    else if (cand == 3)  // right
    {
      pSrc = src + dim*srcStride + width - dim;
      pDst = dst + dim*dstStride + width - dim;
      widthTmp = dim;
      heightTmp = height - 2 * dim;
    }

    if (fracY == 0)
    {
      m_if.filterHor(COMPONENT_Y, pSrc, srcStride, pDst, dstStride, widthTmp, heightTmp, fracX, isLast, fmt, clpRng, 1);
    }
    else if (fracX == 0)
    {
      m_if.filterVer(COMPONENT_Y, pSrc, srcStride, pDst, dstStride, widthTmp, heightTmp, fracY, true, isLast, fmt, clpRng, 1);
    }
    else
    {
      PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][COMPONENT_Y], Size(width, height));
      tmpBuf.stride = width;

      m_if.filterHor(COMPONENT_Y, pSrc - ((vFilterSize >> 1) - 1) * srcStride, srcStride, tmpBuf.buf, tmpBuf.stride, widthTmp, heightTmp + vFilterSize - 1, fracX, false, fmt, clpRng, 1);
      JVET_J0090_SET_CACHE_ENABLE( false );
      m_if.filterVer(COMPONENT_Y, tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, pDst, dstStride, widthTmp, heightTmp, fracY, false, isLast, fmt, clpRng, 1);
      JVET_J0090_SET_CACHE_ENABLE( true );
    }
  }
}

bool InterPrediction::xCalcBiPredSubBlkDist(const PredictionUnit &pu, const Pel* pYuvSrc0, const int src0Stride, const Pel* pYuvSrc1, const int src1Stride, const BitDepths &clipBitDepths)
{
  const int     width = pu.lwidth();
  const int     height = pu.lheight();
  const int     clipbd = clipBitDepths.recon[toChannelType(COMPONENT_Y)];
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(clipbd);
  const int     shift = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int     xUnit = (width >> 2);
  const int     yUnit = (height >> 2);

  m_bioDistThres = (shift <= 5) ? (((32 << (clipbd - 8))*width*height) >> (5 - shift)) : (((32 << (clipbd - 8))*width*height) << (shift - 5));
  m_bioSubBlkDistThres = (shift <= 5) ? (((64 << (clipbd - 8)) << 4) >> (5 - shift)) : (((64 << (clipbd - 8)) << 4) << (shift - 5));

  m_bioDistThres >>= distortionShift;
  m_bioSubBlkDistThres >>= distortionShift;

  DistParam cDistParam;
  Distortion dist = 0;
  for (int yu = 0, blkIdx = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++, blkIdx++)
    {
      const Pel* pPred0 = pYuvSrc0 + ((yu*src0Stride + xu) << 2);
      const Pel* pPred1 = pYuvSrc1 + ((yu*src1Stride + xu) << 2);

      m_pcRdCost->setDistParam(cDistParam, pPred0, pPred1, src0Stride, src1Stride, clipbd, COMPONENT_Y, (1 << 2), (1 << 2), 0, 1, false, true);
      m_bioPredSubBlkDist[blkIdx] = cDistParam.distFunc(cDistParam);
      dist += m_bioPredSubBlkDist[blkIdx];
    }
  }

  return (dist >= m_bioDistThres);
}

void InterPrediction::xAddBIOAvg4(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
#if ENABLE_SIMD_OPT_BIO
  g_pelBufOP.addBIOAvg4(src0, src0Stride, src1, src1Stride, dst, dstStride, gradX0, gradX1, gradY0, gradY1, gradStride, width, height, tmpx, tmpy, shift, offset, clpRng);
#else
  int b = 0;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 4)
    {
      b = tmpx * (gradX0[x] - gradX1[x]) + tmpy * (gradY0[x] - gradY1[x]);
      b = ((b + 1) >> 1);
      dst[x] = ClipPel((int16_t)rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 1] - gradX1[x + 1]) + tmpy * (gradY0[x + 1] - gradY1[x + 1]);
      b = ((b + 1) >> 1);
      dst[x + 1] = ClipPel((int16_t)rightShift((src0[x + 1] + src1[x + 1] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 2] - gradX1[x + 2]) + tmpy * (gradY0[x + 2] - gradY1[x + 2]);
      b = ((b + 1) >> 1);
      dst[x + 2] = ClipPel((int16_t)rightShift((src0[x + 2] + src1[x + 2] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 3] - gradX1[x + 3]) + tmpy * (gradY0[x + 3] - gradY1[x + 3]);
      b = ((b + 1) >> 1);
      dst[x + 3] = ClipPel((int16_t)rightShift((src0[x + 3] + src1[x + 3] + b + offset), shift), clpRng);
    }
    dst += dstStride;       src0 += src0Stride;     src1 += src1Stride;
    gradX0 += gradStride; gradX1 += gradStride; gradY0 += gradStride; gradY1 += gradStride;
  }
#endif
}

void InterPrediction::xBioGradFilter(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY)
{
#if ENABLE_SIMD_OPT_BIO
  g_pelBufOP.bioGradFilter(pSrc, srcStride, width, height, gradStride, gradX, gradY);
#else
  Pel* srcTmp = pSrc + srcStride + 1;
  Pel* gradXTmp = gradX + gradStride + 1;
  Pel* gradYTmp = gradY + gradStride + 1;

  for (int y = 0; y < (height - 2 * BIO_EXTEND_SIZE); y++)
  {
    for (int x = 0; x < (width - 2 * BIO_EXTEND_SIZE); x++)
    {
      gradYTmp[x] = (srcTmp[x + srcStride] - srcTmp[x - srcStride]) >> 4;
      gradXTmp[x] = (srcTmp[x + 1] - srcTmp[x - 1]) >> 4;
    }
    gradXTmp += gradStride;
    gradYTmp += gradStride;
    srcTmp += srcStride;
  }

  gradXTmp = gradX + gradStride + 1;
  gradYTmp = gradY + gradStride + 1;
  for (int y = 0; y < (height - 2 * BIO_EXTEND_SIZE); y++)
  {
    gradXTmp[-1] = gradXTmp[0];
    gradXTmp[width - 2 * BIO_EXTEND_SIZE] = gradXTmp[width - 2 * BIO_EXTEND_SIZE - 1];
    gradXTmp += gradStride;

    gradYTmp[-1] = gradYTmp[0];
    gradYTmp[width - 2 * BIO_EXTEND_SIZE] = gradYTmp[width - 2 * BIO_EXTEND_SIZE - 1];
    gradYTmp += gradStride;
  }

  gradXTmp = gradX + gradStride;
  gradYTmp = gradY + gradStride;
  ::memcpy(gradXTmp - gradStride, gradXTmp, sizeof(Pel)*(width));
  ::memcpy(gradXTmp + (height - 2 * BIO_EXTEND_SIZE)*gradStride, gradXTmp + (height - 2 * BIO_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
  ::memcpy(gradYTmp - gradStride, gradYTmp, sizeof(Pel)*(width));
  ::memcpy(gradYTmp + (height - 2 * BIO_EXTEND_SIZE)*gradStride, gradYTmp + (height - 2 * BIO_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
#endif
}

void InterPrediction::xCalcBIOPar(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG)
{
#if ENABLE_SIMD_OPT_BIO 
  g_pelBufOP.calcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, gradStride, widthG, heightG);
#else
  for (int y = 0; y < heightG; y++)
  {
    for (int x = 0; x < widthG; x++)
    {
      int temp = (srcY0Temp[x] >> 6) - (srcY1Temp[x] >> 6);
      int tempX = (gradX0[x] + gradX1[x]) >> 3;
      int tempY = (gradY0[x] + gradY1[x]) >> 3;
      dotProductTemp1[x] = tempX * tempX;
      dotProductTemp2[x] = tempX * tempY;
      dotProductTemp3[x] = -tempX * temp;
      dotProductTemp5[x] = tempY * tempY;
      dotProductTemp6[x] = -tempY * temp;
    }
    srcY0Temp += src0Stride;
    srcY1Temp += src1Stride;
    gradX0 += gradStride;
    gradX1 += gradStride;
    gradY0 += gradStride;
    gradY1 += gradStride;
    dotProductTemp1 += widthG;
    dotProductTemp2 += widthG;
    dotProductTemp3 += widthG;
    dotProductTemp5 += widthG;
    dotProductTemp6 += widthG;
  }
#endif
}

void InterPrediction::xCalcBlkGradient(int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize)
{
#if ENABLE_SIMD_OPT_BIO
  g_pelBufOP.calcBlkGradient(sx, sy, arraysGx2, arraysGxGy, arraysGxdI, arraysGy2, arraysGydI, sGx2, sGy2, sGxGy, sGxdI, sGydI, width, height, unitSize);
#else
  int     *Gx2 = arraysGx2;
  int     *Gy2 = arraysGy2;
  int     *GxGy = arraysGxGy;
  int     *GxdI = arraysGxdI;
  int     *GydI = arraysGydI;

  // set to the above row due to JVET_K0485_BIO_EXTEND_SIZE
  Gx2 -= (BIO_EXTEND_SIZE*width);
  Gy2 -= (BIO_EXTEND_SIZE*width);
  GxGy -= (BIO_EXTEND_SIZE*width);
  GxdI -= (BIO_EXTEND_SIZE*width);
  GydI -= (BIO_EXTEND_SIZE*width);

  for (int y = -BIO_EXTEND_SIZE; y < unitSize + BIO_EXTEND_SIZE; y++)
  {
    for (int x = -BIO_EXTEND_SIZE; x < unitSize + BIO_EXTEND_SIZE; x++)
    {
      sGx2 += Gx2[x];
      sGy2 += Gy2[x];
      sGxGy += GxGy[x];
      sGxdI += GxdI[x];
      sGydI += GydI[x];
    }
    Gx2 += width;
    Gy2 += width;
    GxGy += width;
    GxdI += width;
    GydI += width;
  }
#endif
}
#endif

#if JVET_L0256_BIO
void InterPrediction::xWeightedAverage(const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied )
#else
void InterPrediction::xWeightedAverage( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs )
#endif
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
#if JVET_L0646_GBI
    if( pu.cu->GBiIdx != GBI_DEFAULT )
    {
#if JVET_L0256_BIO
      CHECK(bioApplied, "GBi is disallowed with BIO");
#endif
      pcYuvDst.addWeightedAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, pu.cu->GBiIdx);
      return;
    }
#endif
#if JVET_L0256_BIO
    if (bioApplied)
    {
      const int  src0Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const int  src1Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const Pel* pSrcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + 2 * src0Stride + 2;
      const Pel* pSrcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + 2 * src1Stride + 2;

      bool bioEnabled = xCalcBiPredSubBlkDist(pu, pSrcY0, src0Stride, pSrcY1, src1Stride, clipBitDepths);
      if (bioEnabled)
      {
        applyBiOptFlow(pu, pcYuvSrc0, pcYuvSrc1, iRefIdx0, iRefIdx1, pcYuvDst, clipBitDepths);
      }
      else
      {
        pcYuvDst.bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, pu.lumaSize()), CPelBuf(pSrcY1, src1Stride, pu.lumaSize()), clpRngs.comp[0]);
      }
    }
    pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, bioApplied);
#else
    pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs );
#endif
  }
  else if( iRefIdx0 >= 0 && iRefIdx1 < 0 )
  {
#if JVET_L0124_L0208_TRIANGLE
    if( pu.cu->triangle )
    {
      pcYuvDst.copyFrom( pcYuvSrc0 );
    }
    else
#endif 
    pcYuvDst.copyClip( pcYuvSrc0, clpRngs );
  }
  else if( iRefIdx0 < 0 && iRefIdx1 >= 0 )
  {
#if JVET_L0124_L0208_TRIANGLE
    if( pu.cu->triangle )
    {
      pcYuvDst.copyFrom( pcYuvSrc1 );
    }
    else
#endif
    pcYuvDst.copyClip( pcYuvSrc1, clpRngs );
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList 
#if JVET_L0293_CPR
  , const bool luma, const bool chroma
#endif
)
{
#if JVET_L0293_CPR
  // dual tree handling for CPR as the only ref
  if (!luma || !chroma)
  {
    if (!luma && chroma)
    {
      xChromaMC(pu, predBuf);
      return;
    }
    else // (luma && !chroma)
    {
      xPredInterUni(pu, eRefPicList, predBuf, false
#if JVET_L0256_BIO
        , false
#endif
        , luma, chroma);
      return;
    }
  }
  // else, go with regular MC below
#endif
        CodingStructure &cs = *pu.cs;
  const PPS &pps            = *cs.pps;
  const SliceType sliceType =  cs.slice->getSliceType();

  if( eRefPicList != REF_PIC_LIST_X )
  {
    if( ( ( sliceType == P_SLICE && pps.getUseWP() ) || ( sliceType == B_SLICE && pps.getWPBiPred() ) ) )
    {
      xPredInterUni         ( pu,          eRefPicList, predBuf, true );
      xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred );
    }
    else
    {
      xPredInterUni( pu, eRefPicList, predBuf, false );
    }
  }
  else
  {
#if JVET_L0293_CPR
    if (pu.mergeType != MRG_TYPE_DEFAULT_N && pu.mergeType != MRG_TYPE_CPR)
#else
    if( pu.mergeType != MRG_TYPE_DEFAULT_N )
#endif
    {
      xSubPuMC( pu, predBuf, eRefPicList );
    }
    else if( xCheckIdenticalMotion( pu ) )
    {
      xPredInterUni( pu, REF_PIC_LIST_0, predBuf, false );
    }
    else
    {
      xPredInterBi( pu, predBuf );
    }
  }
  return;
}

void InterPrediction::motionCompensation( CodingUnit &cu, const RefPicList &eRefPicList 
#if JVET_L0293_CPR
  , const bool luma, const bool chroma
#endif
)
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    PelUnitBuf predBuf = cu.cs->getPredBuf( pu );
    motionCompensation( pu, predBuf, eRefPicList 
#if JVET_L0293_CPR
      , luma, chroma
#endif
    );
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ 
#if JVET_L0293_CPR
  , const bool luma, const bool chroma
#endif
)
{
  PelUnitBuf predBuf = pu.cs->getPredBuf( pu );
  motionCompensation( pu, predBuf, eRefPicList 
#if JVET_L0293_CPR
    , luma, chroma
#endif
  );
}

#if JVET_L0256_BIO
int InterPrediction::rightShiftMSB(int numer, int denom)
{
  int     d;
  int msbIdx = 0;
  for (msbIdx = 0; msbIdx<32; msbIdx++)
  {
    if (denom < ((int)1 << msbIdx))
    {
      break;
    }
  }

  int shiftIdx = msbIdx - 1;
  d = (numer >> shiftIdx);

  return d;
}
#endif

#if JVET_L0124_L0208_TRIANGLE
void InterPrediction::motionCompensation4Triangle( CodingUnit &cu, MergeCtx &triangleMrgCtx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1 )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    const UnitArea localUnitArea( cu.cs->area.chromaFormat, Area( 0, 0, pu.lwidth(), pu.lheight() ) );
    PelUnitBuf tmpTriangleBuf = m_triangleBuf.getBuf( localUnitArea );
    PelUnitBuf predBuf        = cu.cs->getPredBuf( pu );
     
    triangleMrgCtx.setMergeInfo( pu, candIdx0 );
    PU::spanMotionInfo( pu );
    motionCompensation( pu, tmpTriangleBuf );
   
    triangleMrgCtx.setMergeInfo( pu, candIdx1 );
    PU::spanMotionInfo( pu );
    motionCompensation( pu, predBuf );

    weightedTriangleBlk( pu, PU::getTriangleWeights(pu, triangleMrgCtx, candIdx0, candIdx1), splitDir, MAX_NUM_CHANNEL_TYPE, predBuf, tmpTriangleBuf, predBuf );
  }
}

void InterPrediction::weightedTriangleBlk( PredictionUnit &pu, bool weights, const bool splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 )
{
  if( channel == CHANNEL_TYPE_LUMA )
  {
    xWeightedTriangleBlk( pu, pu.lumaSize().width, pu.lumaSize().height, COMPONENT_Y, splitDir, weights, predDst, predSrc0, predSrc1 );
  }
  else if( channel == CHANNEL_TYPE_CHROMA )
  {
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, weights, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, weights, predDst, predSrc0, predSrc1 );
  }
  else
  {
    xWeightedTriangleBlk( pu, pu.lumaSize().width,   pu.lumaSize().height,   COMPONENT_Y,  splitDir, weights, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, weights, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, weights, predDst, predSrc0, predSrc1 );
  }
}

void InterPrediction::xWeightedTriangleBlk( const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const bool splitDir, const bool weights, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 )
{
  Pel*    dst        = predDst .get(compIdx).buf;
  Pel*    src0       = predSrc0.get(compIdx).buf;
  Pel*    src1       = predSrc1.get(compIdx).buf;
  int32_t strideDst  = predDst .get(compIdx).stride  - width;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride  - width;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride  - width;

  const char    log2WeightBase    = 3;
  const ClpRng  clipRng           = pu.cu->slice->clpRngs().comp[compIdx];
  const int32_t clipbd            = clipRng.bd;
  const int32_t shiftDefault      = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int32_t offsetDefault     = (1<<(shiftDefault-1)) + IF_INTERNAL_OFFS;
  const int32_t shiftWeighted     = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
  const int32_t offsetWeighted    = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
                                  
  const int32_t ratioWH           = (width > height) ? (width / height) : 1;
  const int32_t ratioHW           = (width > height) ? 1 : (height / width);
  const Pel*    pelWeighted       = (compIdx == COMPONENT_Y) ? g_trianglePelWeightedLuma[splitDir][weights] : g_trianglePelWeightedChroma[predDst.chromaFormat == CHROMA_444 ? 0 : 1][splitDir][weights];
  const int32_t weightedLength    = (compIdx == COMPONENT_Y) ? g_triangleWeightLengthLuma[weights] : g_triangleWeightLengthChroma[predDst.chromaFormat == CHROMA_444 ? 0 : 1][weights];
        int32_t weightedStartPos  = ( splitDir == 0 ) ? ( 0 - (weightedLength >> 1) * ratioWH ) : ( width - ((weightedLength + 1) >> 1) * ratioWH );
        int32_t weightedEndPos    = weightedStartPos + weightedLength * ratioWH - 1;
        int32_t weightedPosoffset =( splitDir == 0 ) ? ratioWH : -ratioWH;
  
  const Pel*    tmpPelWeighted;
        int32_t x, y, tmpX, tmpY, tmpWeightedStart, tmpWeightedEnd;
  
  for( y = 0; y < height; y+= ratioHW )
  {
    for( tmpY = ratioHW; tmpY > 0; tmpY-- )
    {
      for( x = 0; x < weightedStartPos; x++ )
      {
        *dst++ = ClipPel( rightShift( (splitDir == 0 ? *src1 : *src0) + offsetDefault, shiftDefault), clipRng );
        src0++;
        src1++;
      }

      tmpWeightedStart = std::max((int32_t)0, weightedStartPos);
      tmpWeightedEnd   = std::min(weightedEndPos, (int32_t)(width - 1));
      tmpPelWeighted   = pelWeighted;
      if( weightedStartPos < 0 )
      {
        tmpPelWeighted += abs(weightedStartPos) / ratioWH;
      }
      for( x = tmpWeightedStart; x <= tmpWeightedEnd; x+= ratioWH )
      {
        for( tmpX = ratioWH; tmpX > 0; tmpX-- )
        {
          *dst++ = ClipPel( rightShift( ((*tmpPelWeighted)*(*src0++) + ((8 - (*tmpPelWeighted)) * (*src1++)) + offsetWeighted), shiftWeighted ), clipRng );
        }
        tmpPelWeighted++;
      }

      for( x = weightedEndPos + 1; x < width; x++ )
      {
        *dst++ = ClipPel( rightShift( (splitDir == 0 ? *src0 : *src1) + offsetDefault, shiftDefault ), clipRng );
        src0++;
        src1++;
      }

      dst  += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
    }
    weightedStartPos += weightedPosoffset;
    weightedEndPos   += weightedPosoffset;
  }
}
#endif

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void InterPrediction::cacheAssign( CacheModel *cache )
{
  m_cacheModel = cache;
  m_if.cacheAssign( cache );
  m_if.initInterpolationFilter( !cache->isCacheEnable() );
}
#endif

//! \}
