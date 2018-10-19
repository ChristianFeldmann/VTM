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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"


#if HEVC_USE_SIGN_HIDING
CoeffCodingContext::CoeffCodingContext(const TransformUnit& tu, ComponentID component, bool signHide)
#else
CoeffCodingContext::CoeffCodingContext(const TransformUnit& tu, ComponentID component )
#endif
  : m_compID                    (component)
  , m_chType                    (toChannelType(m_compID))
  , m_width                     (tu.block(m_compID).width)
  , m_height                    (tu.block(m_compID).height)
  , m_log2CGWidth               ((m_width & 3) || (m_height & 3) ? 1 : 2)
  , m_log2CGHeight              ((m_width & 3) || (m_height & 3) ? 1 : 2)
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups             (m_width  >> m_log2CGWidth)
  , m_heightInGroups            (m_height >> m_log2CGHeight)
  , m_log2BlockWidth            (g_aucLog2[m_width])
  , m_log2BlockHeight           (g_aucLog2[m_height])
  , m_log2BlockSize             ((m_log2BlockWidth + m_log2BlockHeight)>>1)
  , m_maxNumCoeff               (m_width * m_height)
#if HEVC_USE_SIGN_HIDING
  , m_signHiding                (signHide)
#endif
  , m_extendedPrecision         (tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  , m_maxLog2TrDynamicRange     (tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
#if HEVC_USE_MDCS
  , m_scanType                  (CoeffScanType(TU::getCoefScanIdx( tu, m_compID)))
#else
  , m_scanType                  (SCAN_DIAG)
#endif
  , m_scan                      (g_scanOrder     [SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )])
  , m_scanPosX                  (g_scanOrderPosXY[SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )][0])
  , m_scanPosY                  (g_scanOrderPosXY[SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )][1])
  , m_scanCG                    (g_scanOrder[SCAN_UNGROUPED  ][m_scanType][gp_sizeIdxInfo->idxFrom(m_widthInGroups)][gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
  , m_CtxSetLastX               (Ctx::LastX[m_chType])
  , m_CtxSetLastY               (Ctx::LastY[m_chType])
  , m_maxLastPosX               (g_uiGroupIdx[m_width - 1])
  , m_maxLastPosY               (g_uiGroupIdx[m_height - 1])
  , m_lastOffsetX               (0)
  , m_lastOffsetY               (0)
  , m_lastShiftX                (0)
  , m_lastShiftY                (0)
  , m_TrafoBypass               (tu.cs->sps->getSpsRangeExtension().getTransformSkipContextEnabledFlag() &&  (tu.cu->transQuantBypass || tu.transformSkip[m_compID]))
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
  , m_tmplCpSum1                (-1)
  , m_tmplCpDiag                (-1)
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType+4] }
  , m_parFlagCtxSet             ( Ctx::ParFlag[m_chType] )
  , m_gtxFlagCtxSet             { Ctx::GtxFlag[m_chType], Ctx::GtxFlag[m_chType+2] }
  , m_sigCoeffGroupFlag         ()
  , m_emtNumSigCoeff            (0)
{
  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
#if HEVC_USE_MDCS
  if (m_scanType == SCAN_VER)
  {
    std::swap(log2sizeX, log2sizeY);
    std::swap(const_cast<unsigned&>(m_maxLastPosX), const_cast<unsigned&>(m_maxLastPosY));
  }
#endif
  if (m_chType == CHANNEL_TYPE_CHROMA)
  {
    if( tu.cs->pcv->rectCUs )
    {
#if HEVC_USE_MDCS
      const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( ( m_scanType == SCAN_VER ? m_height : m_width  ) >> 3) );
      const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( ( m_scanType == SCAN_VER ? m_width  : m_height ) >> 3) );
#else
      const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( m_width  >> 3) );
      const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( m_height >> 3) );
#endif
    }
    else
    {
      const_cast<int&>(m_lastShiftX) = log2sizeX - 2;
      const_cast<int&>(m_lastShiftY) = log2sizeY - 2;
    }
  }
  else
  {
    if( tu.cs->pcv->rectCUs )
    {
      static const int prefix_ctx[8]  = { 0, 0, 0, 3, 6, 10, 15, 21 };
      const_cast<int&>(m_lastOffsetX) = prefix_ctx[ log2sizeX ];
      const_cast<int&>(m_lastOffsetY) = prefix_ctx[ log2sizeY ];;
    }
    else
    {
      const_cast<int&>(m_lastOffsetX) = 3 * (log2sizeX - 2) + ((log2sizeX - 1) >> 2);
      const_cast<int&>(m_lastOffsetY) = 3 * (log2sizeY - 2) + ((log2sizeY - 1) >> 2);
    }
    const_cast<int&>(m_lastShiftX)  = (log2sizeX + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (log2sizeY + 1) >> 2;
  }
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[ m_subSetId ];
  m_subSetPosY              = m_subSetPos / m_widthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
}




unsigned DeriveCtx::CtxCUsplit( const CodingStructure& cs, Partitioner& partitioner )
{
  auto adPartitioner = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner );

  if( !adPartitioner )
  {
    return 0;
  }

  const Position pos         = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned curTileIdx  = cs.picture->tileMap->getTileIdxMap( partitioner.currArea().lumaPos() );
#endif
  unsigned ctxId = 0;

  // get left depth
#if HEVC_TILES_WPP
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), curSliceIdx, curTileIdx, partitioner.chType );
#else
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), curSliceIdx, partitioner.chType );
#endif
  ctxId = ( cuLeft && cuLeft->qtDepth > partitioner.currQtDepth ) ? 1 : 0;

  // get above depth
#if HEVC_TILES_WPP
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), curSliceIdx, curTileIdx, partitioner.chType );
#else
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), curSliceIdx, partitioner.chType );
#endif

  ctxId += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;

  if( cs.sps->getSpsNext().getUseLargeCTU() )
  {
    unsigned minDepth = 0;
    unsigned maxDepth = 0;
    adPartitioner->setMaxMinDepth( minDepth, maxDepth, cs );
    if( partitioner.currDepth < minDepth )
    {
      ctxId = 3;
    }
    else if( partitioner.currDepth >= maxDepth + 1 )
    {
      ctxId = 4;
    }
  }

  return ctxId;
}

#if ENABLE_BMS
unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const unsigned trDepth, const bool prevCbCbf )
#else
unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const bool prevCbCbf )
#endif
{
  if( compID == COMPONENT_Cr )
  {
    return ( prevCbCbf ? 1 : 0 );
  }
#if ENABLE_BMS
  if( isChroma( compID ) )
  {
    return trDepth;
  }
  else
  {
    return ( trDepth == 0 ? 1 : 0 );
  }
#else
  return isChroma( compID ) ? 0 : 1;
#endif
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
  if( pu.cs->sps->getSpsNext().getUseLargeCTU() )
  {
    if( pu.cs->pcv->rectCUs )
    {
      return Clip3( 0, 3, 7 - ( ( g_aucLog2[pu.lumaSize().width] + g_aucLog2[pu.lumaSize().height] + 1 ) >> 1 ) );    // VG-ASYMM DONE
    }
    return Clip3( 0, 3, 6 - g_aucLog2[pu.cu->lumaSize().width] );
  }
  return pu.cu->qtDepth;
}

unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->affine ) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->affine ) ? 1 : 0;

  return ctxId;
}
unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->skip ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->skip ) ? 1 : 0;

  return ctxId;
}


unsigned DeriveCtx::CtxIMVFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->imv ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->imv ) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxBTsplit(const CodingStructure& cs, Partitioner& partitioner)
{
  const Position pos          = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx  = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned curTileIdx   = cs.picture->tileMap->getTileIdxMap( pos );
#endif

  unsigned ctx                = 0;

#if HEVC_TILES_WPP
  const CodingUnit *cuLeft    = cs.getCURestricted( pos.offset( -1,  0 ), curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit *cuAbove   = cs.getCURestricted( pos.offset(  0, -1 ), curSliceIdx, curTileIdx, partitioner.chType );
#else
  const CodingUnit *cuLeft    = cs.getCURestricted( pos.offset( -1,  0 ), curSliceIdx, partitioner.chType );
  const CodingUnit *cuAbove   = cs.getCURestricted( pos.offset(  0, -1 ), curSliceIdx, partitioner.chType );
#endif

  {
    const unsigned currDepth = partitioner.currQtDepth * 2 + partitioner.currBtDepth;

    if( cuLeft )  ctx += ( ( 2 * cuLeft->qtDepth  + cuLeft->btDepth  ) > currDepth ? 1 : 0 );
    if( cuAbove ) ctx += ( ( 2 * cuAbove->qtDepth + cuAbove->btDepth ) > currDepth ? 1 : 0 );
  }
  return ctx;
}


void MergeCtx::setMergeInfo( PredictionUnit& pu, int candIdx )
{
  CHECK( candIdx >= numValidMergeCand, "Merge candidate does not exist" );

  pu.mergeFlag               = true;
  pu.interDir                = interDirNeighbours[candIdx];
  pu.mergeIdx                = candIdx;
  pu.mergeType               = mrgTypeNeighbours[candIdx];
  pu.mv     [REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  pu.mv     [REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
  pu.mvd    [REF_PIC_LIST_0] = Mv();
  pu.mvd    [REF_PIC_LIST_1] = Mv();
  pu.refIdx [REF_PIC_LIST_0] = mvFieldNeighbours[( candIdx << 1 ) + 0].refIdx;
  pu.refIdx [REF_PIC_LIST_1] = mvFieldNeighbours[( candIdx << 1 ) + 1].refIdx;
  pu.mvpIdx [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx [REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_1] = NOT_VALID;

#if JVET_L0646_GBI 
  pu.cu->GBiIdx = ( interDirNeighbours[candIdx] == 3 ) ? GBiIdx[candIdx] : GBI_DEFAULT;
#endif

}
