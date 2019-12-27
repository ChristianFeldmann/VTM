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

/** \file     Slice.cpp
    \brief    slice header and SPS class
*/

#include "CommonDef.h"
#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "dtrace_next.h"

#include "UnitTools.h"

//! \ingroup CommonLib
//! \{

Slice::Slice()
#if !JVET_P1006_PICTURE_HEADER
: m_iPPSId                        ( -1 )
, m_PicOutputFlag                 ( true )
, m_iPOC                          ( 0 )
#else
: m_iPOC                          ( 0 )
#endif
, m_iLastIDR                      ( 0 )
, m_iAssociatedIRAP               ( 0 )
, m_iAssociatedIRAPType           ( NAL_UNIT_INVALID )
, m_rpl0Idx                       ( -1 )
, m_rpl1Idx                       ( -1 )
, m_eNalUnitType                  ( NAL_UNIT_CODED_SLICE_IDR_W_RADL )
, m_eSliceType                    ( I_SLICE )
, m_iSliceQp                      ( 0 )
, m_ChromaQpAdjEnabled            ( false )
, m_deblockingFilterDisable       ( false )
, m_deblockingFilterOverrideFlag  ( false )
, m_deblockingFilterBetaOffsetDiv2( 0 )
, m_deblockingFilterTcOffsetDiv2  ( 0 )
, m_pendingRasInit                ( false )
#if !JVET_P1006_PICTURE_HEADER
, m_depQuantEnabledFlag           ( false )
, m_signDataHidingEnabledFlag     ( false )
#endif
, m_bCheckLDC                     ( false )
, m_biDirPred                    ( false )
, m_iSliceQpDelta                 ( 0 )
, m_iDepth                        ( 0 )
, m_dps                           ( nullptr )
, m_pcSPS                         ( NULL )
, m_pcPPS                         ( NULL )
, m_pcPic                         ( NULL )
#if JVET_P1006_PICTURE_HEADER
, m_pcPicHeader                   ( NULL )
#endif
, m_colFromL0Flag                 ( true )
#if !JVET_P1006_PICTURE_HEADER
, m_noOutputPriorPicsFlag         ( false )
#endif
, m_noIncorrectPicOutputFlag      ( false )
, m_handleCraAsCvsStartFlag            ( false )
, m_colRefIdx                     ( 0 )
#if !JVET_P1006_PICTURE_HEADER
, m_maxNumMergeCand               ( 0 )
, m_maxNumAffineMergeCand         ( 0 )
, m_maxNumTriangleCand            ( 0 )
, m_maxNumIBCMergeCand            ( 0 )
, m_disFracMMVD                   ( false )
, m_disBdofDmvrFlag               ( false )
#endif
, m_uiTLayer                      ( 0 )
, m_bTLayerSwitchingFlag          ( false )
#if !JVET_P1004_REMOVE_BRICKS
, m_sliceMode                     ( NO_SLICES )
, m_sliceArgument                 ( 0 )
, m_sliceCurStartCtuTsAddr        ( 0 )
, m_sliceCurEndCtuTsAddr          ( 0 )
#endif
, m_independentSliceIdx           ( 0 )
, m_nextSlice                     ( false )
, m_sliceBits                     ( 0 )
, m_bFinalized                    ( false )
#if !JVET_P1004_REMOVE_BRICKS
, m_sliceCurStartBrickIdx         ( 0 )
, m_sliceCurEndBrickIdx           ( 0 )
, m_sliceNumBricks                ( 0 )
, m_sliceIdx                      ( 0 )
#endif
, m_bTestWeightPred               ( false )
, m_bTestWeightBiPred             ( false )
, m_substreamSizes                ( )
#if JVET_P1004_REMOVE_BRICKS
, m_numEntryPoints                ( 0 )
#endif
, m_cabacInitFlag                 ( false )
#if JVET_P0126_SIGNALLING_SUBPICID
 , m_sliceSubPicId               ( 0 )
#endif
#if !JVET_P1006_PICTURE_HEADER
, m_jointCbCrSignFlag             ( false )
, m_bLMvdL1Zero                   ( false )
, m_LFCrossSliceBoundaryFlag      ( false )
, m_enableTMVPFlag                ( true )
#endif
, m_encCABACTableIdx              (I_SLICE)
, m_iProcessingStartTime          ( 0 )
, m_dProcessingTime               ( 0 )
#if !JVET_P1006_PICTURE_HEADER
, m_splitConsOverrideFlag         ( false )
, m_uiMinQTSize                   ( 0 )
, m_uiMaxMTTHierarchyDepth                  ( 0 )
, m_uiMaxTTSize                   ( 0 )
, m_uiMinQTSizeIChroma            ( 0 )
, m_uiMaxMTTHierarchyDepthIChroma           ( 0 )
, m_uiMaxBTSizeIChroma            ( 0 )
, m_uiMaxTTSizeIChroma            ( 0 )
, m_uiMaxBTSize                   ( 0 )
, m_lmcsApsId                    ( -1 )
, m_lmcsAps                      (nullptr)
, m_tileGroupLmcsEnabledFlag     (false)
, m_tileGroupLmcsChromaResidualScaleFlag (false)
, m_scalingListApsId             ( -1 )
, m_scalingListAps               ( nullptr )
, m_tileGroupscalingListPresentFlag ( false )
, m_nonReferencePicFlag          ( 0 )
#endif
{
  for(uint32_t i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i] = 0;
  }

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_lambdas            [component] = 0.0;
    m_iSliceChromaQpDelta[component] = 0;
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = 0;

  initEqualRef();

  for ( int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    m_list1IdxToList0Idx[idx] = -1;
  }

  for(int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
  {
    for(uint32_t i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_apcRefPicList [i][iNumCount] = NULL;
      m_aiRefPOCList  [i][iNumCount] = 0;
    }
  }

  resetWpScaling();
  initWpAcDcParam();

  for(int ch=0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = false;
  }

  memset(m_alfApss, 0, sizeof(m_alfApss));
#if JVET_P1004_REMOVE_BRICKS

  m_sliceMap.initSliceMap();
#endif
}

Slice::~Slice()
{
#if JVET_P1004_REMOVE_BRICKS
  m_sliceMap.initSliceMap();
#endif
}


void Slice::initSlice()
{
  for(uint32_t i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]      = 0;
  }
  m_colFromL0Flag = true;

  m_colRefIdx = 0;
  initEqualRef();

  m_bCheckLDC = false;

  m_biDirPred = false;
  m_symRefIdx[0] = -1;
  m_symRefIdx[1] = -1;

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = 0;
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = 0;

#if !JVET_P1006_PICTURE_HEADER
  m_maxNumMergeCand = MRG_MAX_NUM_CANDS;
  m_maxNumAffineMergeCand = AFFINE_MRG_MAX_NUM_CANDS;
  m_maxNumIBCMergeCand = IBC_MRG_MAX_NUM_CANDS;
#endif

  m_bFinalized=false;

#if !JVET_P1006_PICTURE_HEADER
  m_disFracMMVD          = false;
  m_disBdofDmvrFlag      = false;
#endif
  m_substreamSizes.clear();
  m_cabacInitFlag        = false;
#if !JVET_P1006_PICTURE_HEADER
  m_jointCbCrSignFlag    = false;
  m_enableTMVPFlag       = true;
#endif
  m_enableDRAPSEI        = false;
  m_useLTforDRAP         = false;
  m_isDRAP               = false;
  m_latestDRAPPOC        = MAX_INT;
  resetTileGroupAlfEnabledFlag();
}

#if JVET_P1006_PICTURE_HEADER
void Slice::inheritFromPicHeader( PicHeader *picHeader, const PPS *pps, const SPS *sps )
{ 
  if(picHeader->getPicRplPresentFlag())
  {
    setRPL0idx( picHeader->getRPL0idx() );
    *getLocalRPL0() = *picHeader->getLocalRPL0();
    if(getRPL0idx() != -1)
    {
      setRPL0(sps->getRPLList0()->getReferencePictureList(getRPL0idx()));
    }
    else
    {
      setRPL0(getLocalRPL0());
    }
    
    setRPL1idx( picHeader->getRPL1idx() );
    *getLocalRPL1() = *picHeader->getLocalRPL1();
    if(getRPL1idx() != -1)
    {
      setRPL1(sps->getRPLList1()->getReferencePictureList(getRPL1idx()));
    }
    else
    {
      setRPL1(getLocalRPL1());
    }
  }

  setDeblockingFilterDisable( picHeader->getDeblockingFilterDisable() );
  setDeblockingFilterBetaOffsetDiv2( picHeader->getDeblockingFilterBetaOffsetDiv2() );
  setDeblockingFilterTcOffsetDiv2( picHeader->getDeblockingFilterTcOffsetDiv2() );

  setSaoEnabledFlag(CHANNEL_TYPE_LUMA,     picHeader->getSaoEnabledFlag(CHANNEL_TYPE_LUMA));
  setSaoEnabledFlag(CHANNEL_TYPE_CHROMA,   picHeader->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA));

  setTileGroupAlfEnabledFlag(COMPONENT_Y,  picHeader->getAlfEnabledFlag(COMPONENT_Y));
  setTileGroupAlfEnabledFlag(COMPONENT_Cb, picHeader->getAlfEnabledFlag(COMPONENT_Cb));
  setTileGroupAlfEnabledFlag(COMPONENT_Cr, picHeader->getAlfEnabledFlag(COMPONENT_Cr));
  setTileGroupNumAps(picHeader->getNumAlfAps());
  setAlfAPSs(picHeader->getAlfAPSs());
  setTileGroupApsIdChroma(picHeader->getAlfApsIdChroma());   
}

#endif
#if JVET_P1004_REMOVE_BRICKS
void  Slice::setNumEntryPoints( const PPS *pps ) 
{
  uint32_t ctuAddr, ctuX, ctuY;
  m_numEntryPoints = 0;

  // count the number of CTUs that align with either the start of a tile, or with an entropy coding sync point
  // ignore the first CTU since it doesn't count as an entry point
  for( uint32_t i = 1; i < m_sliceMap.getNumCtuInSlice(); i++ ) 
  {
    ctuAddr = m_sliceMap.getCtuAddrInSlice( i );
    ctuX = ( ctuAddr % pps->getPicWidthInCtu() );
    ctuY = ( ctuAddr / pps->getPicWidthInCtu() );
    if( pps->ctuIsTileColBd( ctuX ) && (pps->ctuIsTileRowBd( ctuY ) || pps->getEntropyCodingSyncEnabledFlag() ) ) 
    {
      m_numEntryPoints++;
    }
  }
}

#endif
void Slice::setDefaultClpRng( const SPS& sps )
{
  m_clpRngs.comp[COMPONENT_Y].min = m_clpRngs.comp[COMPONENT_Cb].min  = m_clpRngs.comp[COMPONENT_Cr].min = 0;
  m_clpRngs.comp[COMPONENT_Y].max                                     = (1<< sps.getBitDepth(CHANNEL_TYPE_LUMA))-1;
  m_clpRngs.comp[COMPONENT_Y].bd  = sps.getBitDepth(CHANNEL_TYPE_LUMA);
  m_clpRngs.comp[COMPONENT_Y].n   = 0;
  m_clpRngs.comp[COMPONENT_Cb].max = m_clpRngs.comp[COMPONENT_Cr].max = (1<< sps.getBitDepth(CHANNEL_TYPE_CHROMA))-1;
  m_clpRngs.comp[COMPONENT_Cb].bd  = m_clpRngs.comp[COMPONENT_Cr].bd  = sps.getBitDepth(CHANNEL_TYPE_CHROMA);
  m_clpRngs.comp[COMPONENT_Cb].n   = m_clpRngs.comp[COMPONENT_Cr].n   = 0;
  m_clpRngs.used = m_clpRngs.chroma = false;
}


bool Slice::getRapPicFlag() const
{
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}


void  Slice::sortPicList        (PicList& rcListPic)
{
  Picture*    pcPicExtract;
  Picture*    pcPicInsert;

  PicList::iterator    iterPicExtract;
  PicList::iterator    iterPicExtract_1;
  PicList::iterator    iterPicInsert;

  for (int i = 1; i < (int)(rcListPic.size()); i++)
  {
    iterPicExtract = rcListPic.begin();
    for (int j = 0; j < i; j++)
    {
      iterPicExtract++;
    }
    pcPicExtract = *(iterPicExtract);

    iterPicInsert = rcListPic.begin();
    while (iterPicInsert != iterPicExtract)
    {
      pcPicInsert = *(iterPicInsert);
      if (pcPicInsert->getPOC() >= pcPicExtract->getPOC())
      {
        break;
      }

      iterPicInsert++;
    }

    iterPicExtract_1 = iterPicExtract;    iterPicExtract_1++;

    //  swap iterPicExtract and iterPicInsert, iterPicExtract = curr. / iterPicInsert = insertion position
    rcListPic.insert( iterPicInsert, iterPicExtract, iterPicExtract_1 );
    rcListPic.erase( iterPicExtract );
  }
}

#if JVET_N0278_FIXES
Picture* Slice::xGetRefPic( PicList& rcListPic, int poc, const int layerId )
#else
Picture* Slice::xGetRefPic (PicList& rcListPic, int poc)
#endif
{
  PicList::iterator  iterPic = rcListPic.begin();
  Picture*           pcPic   = *(iterPic);

  while ( iterPic != rcListPic.end() )
  {
#if JVET_N0278_FIXES
    if( pcPic->getPOC() == poc && pcPic->layerId == layerId )
#else
    if(pcPic->getPOC() == poc)
#endif
    {
      break;
    }
    iterPic++;
    pcPic = *(iterPic);
  }
  return  pcPic;
}

#if JVET_N0278_FIXES
Picture* Slice::xGetLongTermRefPic( PicList& rcListPic, int poc, bool pocHasMsb, const int layerId )
#else
Picture* Slice::xGetLongTermRefPic( PicList& rcListPic, int poc, bool pocHasMsb)
#endif
{
  PicList::iterator  iterPic = rcListPic.begin();
  Picture*           pcPic   = *(iterPic);
  Picture*           pcStPic = pcPic;

  int pocCycle = 1 << getSPS()->getBitsForPOC();
  if (!pocHasMsb)
  {
    poc = poc & (pocCycle - 1);
  }

  while ( iterPic != rcListPic.end() )
  {
    pcPic = *(iterPic);
#if JVET_N0278_FIXES
    if( pcPic && pcPic->getPOC() != this->getPOC() && pcPic->referenced && pcPic->layerId == layerId )
#else
    if (pcPic && pcPic->getPOC()!=this->getPOC() && pcPic->referenced)
#endif
    {
      int picPoc = pcPic->getPOC();
      if (!pocHasMsb)
      {
        picPoc = picPoc & (pocCycle - 1);
      }

      if (poc == picPoc)
      {
        if(pcPic->longTerm)
        {
          return pcPic;
        }
        else
        {
          pcStPic = pcPic;
        }
        break;
      }
    }

    iterPic++;
  }

  return pcStPic;
}

void Slice::setRefPOCList       ()
{
  for (int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (int iNumRefIdx = 0; iNumRefIdx < m_aiNumRefIdx[iDir]; iNumRefIdx++)
    {
      m_aiRefPOCList[iDir][iNumRefIdx] = m_apcRefPicList[iDir][iNumRefIdx]->getPOC();
    }
  }

}

void Slice::setList1IdxToList0Idx()
{
  int idxL0, idxL1;
  for ( idxL1 = 0; idxL1 < getNumRefIdx( REF_PIC_LIST_1 ); idxL1++ )
  {
    m_list1IdxToList0Idx[idxL1] = -1;
    for ( idxL0 = 0; idxL0 < getNumRefIdx( REF_PIC_LIST_0 ); idxL0++ )
    {
      if ( m_apcRefPicList[REF_PIC_LIST_0][idxL0]->getPOC() == m_apcRefPicList[REF_PIC_LIST_1][idxL1]->getPOC() )
      {
        m_list1IdxToList0Idx[idxL1] = idxL0;
        break;
      }
    }
  }
}

void Slice::constructRefPicList(PicList& rcListPic)
{
  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));
  if (m_eSliceType == I_SLICE)
  {
    ::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
    ::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));
    return;
  }

  Picture*  pcRefPic = NULL;
  uint32_t numOfActiveRef = 0;
  //construct L0
  numOfActiveRef = getNumRefIdx(REF_PIC_LIST_0);
#if JVET_O1159_SCALABILITY
  int layerIdx = m_pcPic->cs->vps == nullptr ? 0 : m_pcPic->cs->vps->getGeneralLayerIdx( m_pcPic->layerId );
#endif

  for (int ii = 0; ii < numOfActiveRef; ii++)
  {
#if JVET_O1159_SCALABILITY
    if( m_pRPL0->isInterLayerRefPic( ii ) )
    {
      CHECK( m_pRPL0->getInterLayerRefPicIdx( ii ) == NOT_VALID, "Wrong ILRP index" );

      int refLayerIdx = m_pcPic->cs->vps->getDirectRefLayerIdx( layerIdx, m_pRPL0->getInterLayerRefPicIdx( ii ) );

      pcRefPic = xGetRefPic( rcListPic, getPOC(), refLayerIdx );
      pcRefPic->longTerm = true;
    }
    else
#endif
    if (!m_pRPL0->isRefPicLongterm(ii))
    {
#if JVET_N0278_FIXES
      pcRefPic = xGetRefPic( rcListPic, getPOC() - m_pRPL0->getRefPicIdentifier( ii ), m_pcPic->layerId );
#else
      pcRefPic = xGetRefPic(rcListPic, getPOC() - m_pRPL0->getRefPicIdentifier(ii));
#endif
      pcRefPic->longTerm = false;
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = m_pRPL0->getRefPicIdentifier(ii) & pocMask;
      ltrpPoc += m_localRPL0.getDeltaPocMSBPresentFlag(ii) ? (pocMask + 1) * m_localRPL0.getDeltaPocMSBCycleLT(ii) : 0;
#if JVET_N0278_FIXES
      pcRefPic = xGetLongTermRefPic( rcListPic, ltrpPoc, m_localRPL0.getDeltaPocMSBPresentFlag( ii ), m_pcPic->layerId );
#else
      pcRefPic = xGetLongTermRefPic(rcListPic, ltrpPoc, m_localRPL0.getDeltaPocMSBPresentFlag(ii));
#endif
      pcRefPic->longTerm = true;
    }
    pcRefPic->extendPicBorder();
    m_apcRefPicList[REF_PIC_LIST_0][ii] = pcRefPic;
    m_bIsUsedAsLongTerm[REF_PIC_LIST_0][ii] = pcRefPic->longTerm;
  }

  //construct L1
  numOfActiveRef = getNumRefIdx(REF_PIC_LIST_1);
  for (int ii = 0; ii < numOfActiveRef; ii++)
  {
#if JVET_O1159_SCALABILITY
    if( m_pRPL1->isInterLayerRefPic( ii ) )
    {
      CHECK( m_pRPL1->getInterLayerRefPicIdx( ii ) == NOT_VALID, "Wrong ILRP index" );

      int refLayerIdx = m_pcPic->cs->vps->getDirectRefLayerIdx( layerIdx, m_pRPL1->getInterLayerRefPicIdx( ii ) );

      pcRefPic = xGetRefPic( rcListPic, getPOC(), refLayerIdx );
      pcRefPic->longTerm = true;
    }
    else
#endif
    if (!m_pRPL1->isRefPicLongterm(ii))
    {
#if JVET_N0278_FIXES
      pcRefPic = xGetRefPic( rcListPic, getPOC() - m_pRPL1->getRefPicIdentifier( ii ), m_pcPic->layerId );
#else
      pcRefPic = xGetRefPic(rcListPic, getPOC() - m_pRPL1->getRefPicIdentifier(ii));
#endif
      pcRefPic->longTerm = false;
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = m_pRPL1->getRefPicIdentifier(ii) & pocMask;
      ltrpPoc += m_localRPL1.getDeltaPocMSBPresentFlag(ii) ? (pocMask + 1) * m_localRPL1.getDeltaPocMSBCycleLT(ii) : 0;
#if JVET_N0278_FIXES
      pcRefPic = xGetLongTermRefPic( rcListPic, ltrpPoc, m_localRPL1.getDeltaPocMSBPresentFlag( ii ), m_pcPic->layerId );
#else
      pcRefPic = xGetLongTermRefPic(rcListPic, ltrpPoc, m_localRPL1.getDeltaPocMSBPresentFlag(ii));
#endif
      pcRefPic->longTerm = true;
    }
    pcRefPic->extendPicBorder();
    m_apcRefPicList[REF_PIC_LIST_1][ii] = pcRefPic;
    m_bIsUsedAsLongTerm[REF_PIC_LIST_1][ii] = pcRefPic->longTerm;
  }
}

void Slice::initEqualRef()
{
  for (int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (int iRefIdx1 = 0; iRefIdx1 < MAX_NUM_REF; iRefIdx1++)
    {
      for (int iRefIdx2 = iRefIdx1; iRefIdx2 < MAX_NUM_REF; iRefIdx2++)
      {
        m_abEqualRef[iDir][iRefIdx1][iRefIdx2] = m_abEqualRef[iDir][iRefIdx2][iRefIdx1] = (iRefIdx1 == iRefIdx2? true : false);
      }
    }
  }
}

void Slice::checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic)
{
  int i;
  Slice* curSlice = pic->slices[curSliceSegmentIdx];
  int currColRefPOC =  curSlice->getRefPOC( RefPicList(1 - curSlice->getColFromL0Flag()), curSlice->getColRefIdx());

  for(i=curSliceSegmentIdx-1; i>=0; i--)
  {
    const Slice* preSlice = pic->slices[i];
    if(preSlice->getSliceType() != I_SLICE)
    {
      const int preColRefPOC  = preSlice->getRefPOC( RefPicList(1 - preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
      if(currColRefPOC != preColRefPOC)
      {
        THROW("Collocated_ref_idx shall always be the same for all slices of a coded picture!");
      }
      else
      {
        break;
      }
    }
  }
}

void Slice::checkCRA(const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, int& pocCRA, NalUnitType& associatedIRAPType, PicList& rcListPic)
{
  if (pocCRA < MAX_UINT && getPOC() > pocCRA)
  {
    uint32_t numRefPic = pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures();
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL0->isRefPicLongterm(i))
      {
        CHECK(getPOC() - pRPL0->getRefPicIdentifier(i) < pocCRA, "Invalid state");
      }
      else
      {
#if JVET_N0278_FIXES
        CHECK( xGetLongTermRefPic( rcListPic, pRPL0->getRefPicIdentifier( i ), pRPL0->getDeltaPocMSBPresentFlag( i ), m_pcPic->layerId )->getPOC() < pocCRA, "Invalid state" );
#else
        CHECK(xGetLongTermRefPic(rcListPic, pRPL0->getRefPicIdentifier(i), pRPL0->getDeltaPocMSBPresentFlag(i))->getPOC() < pocCRA, "Invalid state");
#endif
      }
    }
    numRefPic = pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures();
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL1->isRefPicLongterm(i))
      {
        CHECK(getPOC() - pRPL1->getRefPicIdentifier(i) < pocCRA, "Invalid state");
      }
#if JVET_O1159_SCALABILITY
      else if( !pRPL1->isInterLayerRefPic( i ) )
#else
      else
#endif
      {
#if JVET_N0278_FIXES
        CHECK( xGetLongTermRefPic( rcListPic, pRPL1->getRefPicIdentifier( i ), pRPL1->getDeltaPocMSBPresentFlag( i ), m_pcPic->layerId )->getPOC() < pocCRA, "Invalid state" );
#else
        CHECK(xGetLongTermRefPic(rcListPic, pRPL1->getRefPicIdentifier(i), pRPL1->getDeltaPocMSBPresentFlag(i))->getPOC() < pocCRA, "Invalid state");
#endif
      }
    }
  }
  if (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP) // IDR picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
  else if (getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA) // CRA picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
}

#if JVET_O0235_NAL_UNIT_TYPE_CONSTRAINTS
void Slice::checkSTSA(PicList& rcListPic)
{
  int ii;
  Picture* pcRefPic = NULL;
  int numOfActiveRef = getNumRefIdx(REF_PIC_LIST_0);

  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    pcRefPic = m_apcRefPicList[REF_PIC_LIST_0][ii];

    // Checking this: "When the current picture is an STSA picture, there shall be no active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that has TemporalId equal to that of the current picture"
    if (getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA)
    {
      CHECK(pcRefPic->layer == m_uiTLayer, "When the current picture is an STSA picture, there shall be no active entry in the RPL that has TemporalId equal to that of the current picture");
    }

    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has TemporalId equal to that of the current picture, there shall be no
    // picture that has TemporalId equal to that of the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA picture in decoding order."
    CHECK(pcRefPic->subLayerNonReferencePictureDueToSTSA, "The RPL of the current picture contains a picture that is not allowed in this temporal layer due to an earlier STSA picture");
  }

  numOfActiveRef = getNumRefIdx(REF_PIC_LIST_1);
  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    pcRefPic = m_apcRefPicList[REF_PIC_LIST_1][ii];

    // Checking this: "When the current picture is an STSA picture, there shall be no active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that has TemporalId equal to that of the current picture"
    if (getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA)
    {
      CHECK(pcRefPic->layer == m_uiTLayer, "When the current picture is an STSA picture, there shall be no active entry in the RPL that has TemporalId equal to that of the current picture");
    }

    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has TemporalId equal to that of the current picture, there shall be no
    // picture that has TemporalId equal to that of the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA picture in decoding order."
    CHECK(pcRefPic->subLayerNonReferencePictureDueToSTSA, "The active RPL part of the current picture contains a picture that is not allowed in this temporal layer due to an earlier STSA picture");
  }

  // If the current picture is an STSA picture, make all reference pictures in the DPB with temporal
  // id equal to the temproal id of the current picture sub-layer non-reference pictures. The flag
  // subLayerNonReferencePictureDueToSTSA equal to true means that the picture may not be used for
  // reference by a picture that follows the current STSA picture in decoding order
  if (getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA)
  {
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      pcRefPic = *(iterPic++);
      if (!pcRefPic->referenced || pcRefPic->getPOC() == m_iPOC)
      {
        continue;
      }

      if (pcRefPic->layer == m_uiTLayer)
      {
        pcRefPic->subLayerNonReferencePictureDueToSTSA = true;
      }
    }
  }
}
#endif


/** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
 * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
 * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
 * \param rcListPic reference to the reference picture list
 * This function marks the reference pictures as "unused for reference" in the following conditions.
 * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
 * are marked as "unused for reference"
 *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
 * Otherwise
 *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
 *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
 *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
 *    the bRefreshPending flag to false.
 *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
 *    reference of the current picture.
 * Note that the current picture is already placed in the reference list and its marking is not changed.
 * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
 */
void Slice::decodingRefreshMarking(int& pocCRA, bool& bRefreshPending, PicList& rcListPic, const bool bEfficientFieldIRAPEnabled)
{
  Picture* rpcPic;
  int      pocCurr = getPOC();

  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP)  // IDR picture
  {
    // mark all pictures as not used for reference
    PicList::iterator        iterPic       = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic);
      if (rpcPic->getPOC() != pocCurr)
      {
        rpcPic->referenced = false;
        rpcPic->getHashMap()->clearAll();
      }
      iterPic++;
    }
    if (bEfficientFieldIRAPEnabled)
    {
      bRefreshPending = true;
    }
  }
  else // CRA or No DR
  {
    if(bEfficientFieldIRAPEnabled && (getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
    {
      if (bRefreshPending==true && pocCurr > m_iLastIDR) // IDR reference marking pending
      {
        PicList::iterator        iterPic       = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != m_iLastIDR)
          {
            rpcPic->referenced = false;
            rpcPic->getHashMap()->clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    else
    {
      if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending
      {
        PicList::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
          {
            rpcPic->referenced = false;
            rpcPic->getHashMap()->clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
    {
      bRefreshPending = true;
      pocCRA = pocCurr;
    }
  }
}

void Slice::copySliceInfo(Slice *pSrc, bool cpyAlmostAll)
{
  CHECK(!pSrc, "Source is NULL");

  int i, j, k;

  m_iPOC                 = pSrc->m_iPOC;
  m_eNalUnitType         = pSrc->m_eNalUnitType;
  m_eSliceType           = pSrc->m_eSliceType;
  m_iSliceQp             = pSrc->m_iSliceQp;
  m_iSliceQpBase         = pSrc->m_iSliceQpBase;
  m_ChromaQpAdjEnabled              = pSrc->m_ChromaQpAdjEnabled;
  m_deblockingFilterDisable         = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag    = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2  = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2    = pSrc->m_deblockingFilterTcOffsetDiv2;

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]     = pSrc->m_aiNumRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  }

  m_bCheckLDC             = pSrc->m_bCheckLDC;
  m_iSliceQpDelta        = pSrc->m_iSliceQpDelta;

  m_biDirPred = pSrc->m_biDirPred;
  m_symRefIdx[0] = pSrc->m_symRefIdx[0];
  m_symRefIdx[1] = pSrc->m_symRefIdx[1];

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = pSrc->m_iSliceChromaQpDelta[component];
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = pSrc->m_iSliceChromaQpDelta[JOINT_CbCr];

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_apcRefPicList[i][j]  = pSrc->m_apcRefPicList[i][j];
      m_aiRefPOCList[i][j]   = pSrc->m_aiRefPOCList[i][j];
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];
    }
    m_bIsUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_bIsUsedAsLongTerm[i][MAX_NUM_REF];
  }
  if( cpyAlmostAll ) m_iDepth = pSrc->m_iDepth;

  // access channel
  if (cpyAlmostAll) m_pRPL0 = pSrc->m_pRPL0;
  if (cpyAlmostAll) m_pRPL1 = pSrc->m_pRPL1;
  m_iLastIDR             = pSrc->m_iLastIDR;

  if( cpyAlmostAll ) m_pcPic  = pSrc->m_pcPic;

#if JVET_P1006_PICTURE_HEADER
  m_pcPicHeader          = pSrc->m_pcPicHeader;
#endif
  m_colFromL0Flag        = pSrc->m_colFromL0Flag;
  m_colRefIdx            = pSrc->m_colRefIdx;

  if( cpyAlmostAll ) setLambdas(pSrc->getLambdas());

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      for (k =0; k < MAX_NUM_REF; k++)
      {
        m_abEqualRef[i][j][k] = pSrc->m_abEqualRef[i][j][k];
      }
    }
  }

  m_uiTLayer                      = pSrc->m_uiTLayer;
  m_bTLayerSwitchingFlag          = pSrc->m_bTLayerSwitchingFlag;

#if JVET_P1004_REMOVE_BRICKS
  m_sliceMap                      = pSrc->m_sliceMap;
#else
  m_sliceMode                     = pSrc->m_sliceMode;
  m_sliceArgument                 = pSrc->m_sliceArgument;
  m_sliceCurStartCtuTsAddr        = pSrc->m_sliceCurStartCtuTsAddr;
  m_sliceCurEndCtuTsAddr          = pSrc->m_sliceCurEndCtuTsAddr;
#endif
  m_independentSliceIdx           = pSrc->m_independentSliceIdx;
  m_nextSlice                     = pSrc->m_nextSlice;
  m_clpRngs                       = pSrc->m_clpRngs;
  m_pendingRasInit                = pSrc->m_pendingRasInit;

  for ( uint32_t e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( uint32_t n=0 ; n<MAX_NUM_REF ; n++ )
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(WPScalingParam)*MAX_NUM_COMPONENT );
    }
  }

  for( uint32_t ch = 0 ; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = pSrc->m_saoEnabledFlag[ch];
  }

  m_cabacInitFlag                 = pSrc->m_cabacInitFlag;
#if !JVET_P1006_PICTURE_HEADER
  m_jointCbCrSignFlag             = pSrc->m_jointCbCrSignFlag;
#endif
  memcpy(m_alfApss, pSrc->m_alfApss, sizeof(m_alfApss)); // this might be quite unsafe
  memcpy( m_tileGroupAlfEnabledFlag, pSrc->m_tileGroupAlfEnabledFlag, sizeof(m_tileGroupAlfEnabledFlag));
  m_tileGroupNumAps               = pSrc->m_tileGroupNumAps;
  m_tileGroupLumaApsId            = pSrc->m_tileGroupLumaApsId;
  m_tileGroupChromaApsId          = pSrc->m_tileGroupChromaApsId;
  m_disableSATDForRd              = pSrc->m_disableSATDForRd;

#if !JVET_P1006_PICTURE_HEADER
  m_bLMvdL1Zero                   = pSrc->m_bLMvdL1Zero;
  m_LFCrossSliceBoundaryFlag      = pSrc->m_LFCrossSliceBoundaryFlag;
  m_enableTMVPFlag                = pSrc->m_enableTMVPFlag;
  m_maxNumMergeCand               = pSrc->m_maxNumMergeCand;
  m_maxNumAffineMergeCand         = pSrc->m_maxNumAffineMergeCand;
  m_maxNumTriangleCand            = pSrc->m_maxNumTriangleCand;
  m_maxNumIBCMergeCand            = pSrc->m_maxNumIBCMergeCand;
  m_disFracMMVD                   = pSrc->m_disFracMMVD;
  m_disBdofDmvrFlag               = pSrc->m_disBdofDmvrFlag;
#endif
  if( cpyAlmostAll ) m_encCABACTableIdx  = pSrc->m_encCABACTableIdx;
#if !JVET_P1006_PICTURE_HEADER
  m_splitConsOverrideFlag         = pSrc->m_splitConsOverrideFlag;
  m_uiMinQTSize                   = pSrc->m_uiMinQTSize;
  m_uiMaxMTTHierarchyDepth                  = pSrc->m_uiMaxMTTHierarchyDepth;
  m_uiMaxTTSize                   = pSrc->m_uiMaxTTSize;
  m_uiMinQTSizeIChroma            = pSrc->m_uiMinQTSizeIChroma;
  m_uiMaxMTTHierarchyDepthIChroma           = pSrc->m_uiMaxMTTHierarchyDepthIChroma;
  m_uiMaxBTSizeIChroma            = pSrc->m_uiMaxBTSizeIChroma;
  m_uiMaxTTSizeIChroma            = pSrc->m_uiMaxTTSizeIChroma;
  m_uiMaxBTSize                   = pSrc->m_uiMaxBTSize;

  m_depQuantEnabledFlag           = pSrc->m_depQuantEnabledFlag;
  m_signDataHidingEnabledFlag     = pSrc->m_signDataHidingEnabledFlag;

  m_tileGroupLmcsEnabledFlag = pSrc->m_tileGroupLmcsEnabledFlag;
  m_tileGroupLmcsChromaResidualScaleFlag = pSrc->m_tileGroupLmcsChromaResidualScaleFlag;
  m_lmcsAps = pSrc->m_lmcsAps;
  m_lmcsApsId = pSrc->m_lmcsApsId;

  m_tileGroupscalingListPresentFlag = pSrc->m_tileGroupscalingListPresentFlag;
  m_scalingListAps                  = pSrc->m_scalingListAps;
  m_scalingListApsId                = pSrc->m_scalingListApsId;
#endif
  for( int i = 0; i < NUM_REF_PIC_LIST_01; i ++ )
  {
    for (int j = 0; j < MAX_NUM_REF_PICS; j ++ )
    {
      m_scalingRatio[i][j]          = pSrc->m_scalingRatio[i][j];
    }
  }
}


/** Function for checking if this is a switching-point
*/
bool Slice::isTemporalLayerSwitchingPoint(PicList& rcListPic) const
{
  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    const Picture* pcPic = *(iterPic++);
    if( pcPic->referenced && pcPic->poc != getPOC())
    {
      if( pcPic->layer >= getTLayer())
      {
        return false;
      }
    }
  }
  return true;
}

/** Function for checking if this is a STSA candidate
 */
bool Slice::isStepwiseTemporalLayerSwitchingPointCandidate(PicList& rcListPic) const
{
  PicList::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    const Picture* pcPic = *(iterPic++);
    if( pcPic->referenced && pcPic->poc != getPOC())
    {
      if( pcPic->layer >= getTLayer())
      {
        return false;
      }
    }
  }
  return true;
}


void Slice::checkLeadingPictureRestrictions(PicList& rcListPic) const
{
  int nalUnitType = this->getNalUnitType();

  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() > this->getPOC())
  {
    // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
    if (nalUnitType < NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
        nalUnitType > NAL_UNIT_CODED_SLICE_CRA)
    {
      CHECK(nalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
            nalUnitType != NAL_UNIT_CODED_SLICE_RADL, "Invalid NAL unit type");
    }
  }

  // When a picture is a trailing picture, it shall not be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() < this->getPOC())
  {
    CHECK(nalUnitType == NAL_UNIT_CODED_SLICE_RASL ||
          nalUnitType == NAL_UNIT_CODED_SLICE_RADL, "Invalid NAL unit type");
  }


  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL)
  {
    CHECK( this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL, "Invalid NAL unit type");
  }

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
  {
    CHECK (this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP, "Invalid NAL unit type");
  }

  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  int numLeadingPicsFound = 0;
  while ( iterPic != rcListPic.end())
  {
    Picture* pcPic = *(iterPic++);
    if( ! pcPic->reconstructed)
    {
      continue;
    }
    if( pcPic->poc == this->getPOC())
    {
      continue;
    }
    const Slice* pcSlice = pcPic->slices[0];

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede the IRAP picture in output order.
    // (Note that any picture following in output order would be present in the DPB)
#if JVET_P1006_PICTURE_HEADER
    if(pcSlice->getPicHeader()->getPicOutputFlag() == 1 && !this->getPicHeader()->getNoOutputOfPriorPicsFlag())
#else
    if(pcSlice->getPicOutputFlag() == 1 && !this->getNoOutputPriorPicsFlag())
#endif
    {
      if (nalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
          nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
      {
        CHECK(pcPic->poc >= this->getPOC(), "Invalid POC");
      }
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede any RADL picture associated with the IRAP
    // picture in output order.
#if JVET_P1006_PICTURE_HEADER
    if(pcSlice->getPicHeader()->getPicOutputFlag() == 1)
#else
    if(pcSlice->getPicOutputFlag() == 1)
#endif
    {
      if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
      {
        // rpcPic precedes the IRAP in decoding order
        if(this->getAssociatedIRAPPOC() > pcSlice->getAssociatedIRAPPOC())
        {
          // rpcPic must not be the IRAP picture
          if(this->getAssociatedIRAPPOC() != pcPic->poc)
          {
            CHECK( pcPic->poc >= this->getPOC(), "Invalid POC");
          }
        }
      }
    }

    // When a picture is a leading picture, it shall precede, in decoding order,
    // all trailing pictures that are associated with the same IRAP picture.
    if ((nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL) &&
        (pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL && pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL)  )
    {
      if (pcSlice->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC())
      {
        numLeadingPicsFound++;
        int limitNonLP = 0;
        if (pcSlice->getSPS()->getVuiParameters() && pcSlice->getSPS()->getVuiParameters()->getFieldSeqFlag())
          limitNonLP = 1;
        CHECK(pcPic->poc > this->getAssociatedIRAPPOC() && numLeadingPicsFound > limitNonLP, "Invalid POC");
      }
    }

    // Any RASL picture associated with a CRA or BLA picture shall precede any
    // RADL picture associated with the CRA or BLA picture in output order
    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL)
    {
      if ((this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA) &&
          this->getAssociatedIRAPPOC() == pcSlice->getAssociatedIRAPPOC())
      {
        if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL)
        {
          CHECK( pcPic->poc <= this->getPOC(), "Invalid POC");
        }
      }
    }

    // Any RASL picture associated with a CRA picture shall follow, in output
    // order, any IRAP picture that precedes the CRA picture in decoding order.
    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL)
    {
      if(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)
      {
        if(pcSlice->getPOC() < this->getAssociatedIRAPPOC() &&
          (
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA))
        {
          CHECK(this->getPOC() <= pcSlice->getPOC(), "Invalid POC");
        }
      }
    }
  }
}




//Function for applying picture marking based on the Reference Picture List
#if JVET_O1159_SCALABILITY
void Slice::applyReferencePictureListBasedMarking( PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, const int layerId ) const
#else
void Slice::applyReferencePictureListBasedMarking(PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1) const
#endif
{
  int i, isReference;
  checkLeadingPictureRestrictions(rcListPic);

  bool isNeedToCheck = (this->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL) ? false : true;

  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    Picture* pcPic = *(iterPic++);

    if (!pcPic->referenced)
      continue;

    isReference = 0;
    // loop through all pictures in the Reference Picture Set
    // to see if the picture should be kept as reference picture
#if JVET_O1159_SCALABILITY
    for( i = 0; isNeedToCheck && !isReference && i < pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures() + pRPL0->getNumberOfInterLayerPictures(); i++ )
    {
      if( pRPL0->isInterLayerRefPic( i ) )
      {
        // Diagonal inter-layer prediction is not allowed
        CHECK( pRPL0->getRefPicIdentifier( i ), "ILRP identifier should be 0" );

        if( pcPic->poc == m_iPOC )
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
      else if( pcPic->layerId == layerId )
#else
    for (i = 0; isNeedToCheck && !isReference && i<pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures(); i++)
    {
#endif
      if (!(pRPL0->isRefPicLongterm(i)))
      {
        if (pcPic->poc == this->getPOC() - pRPL0->getRefPicIdentifier(i))
        {
          isReference = 1;
          pcPic->longTerm = false;
        }
      }
      else
      {
        int pocCycle = 1 << (pcPic->cs->sps->getBitsForPOC());
        int curPoc = pcPic->poc & (pocCycle - 1);
        if (pcPic->longTerm && curPoc == pRPL0->getRefPicIdentifier(i))
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
    }

#if JVET_O1159_SCALABILITY
    for( i = 0; isNeedToCheck && !isReference && i < pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures() + pRPL1->getNumberOfInterLayerPictures(); i++ )
    {
      if( pRPL1->isInterLayerRefPic( i ) )
      {
        // Diagonal inter-layer prediction is not allowed
        CHECK( pRPL1->getRefPicIdentifier( i ), "ILRP identifier should be 0" );

        if( pcPic->poc == m_iPOC )
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
      else if( pcPic->layerId == layerId )
#else
    for (i = 0; isNeedToCheck && !isReference && i<pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures(); i++)
    {
#endif
      if (!(pRPL1->isRefPicLongterm(i)))
      {
        if (pcPic->poc == this->getPOC() - pRPL1->getRefPicIdentifier(i))
        {
          isReference = 1;
          pcPic->longTerm = false;
        }
      }
      else
      {
        int pocCycle = 1 << (pcPic->cs->sps->getBitsForPOC());
        int curPoc = pcPic->poc & (pocCycle - 1);
        if (pcPic->longTerm && curPoc == pRPL1->getRefPicIdentifier(i))
        {
          isReference = 1;
          pcPic->longTerm = true;
        }
      }
    }
    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture List
#if JVET_O1159_SCALABILITY
    if( pcPic->layerId == layerId && pcPic->poc != m_iPOC && isReference == 0 )
#else
    if (pcPic->poc != this->getPOC() && isReference == 0)
#endif
    {
      pcPic->referenced = false;
      pcPic->longTerm = false;
    }

    // sanity checks
    if (pcPic->referenced)
    {
      //check that pictures of higher temporal layers are not used
      CHECK(pcPic->usedByCurr && !(pcPic->layer <= this->getTLayer()), "Invalid state");
    }
  }
}

int Slice::checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx, bool printErrors) const
{
  Picture* rpcPic;
  int isAvailable = 0;
  int notPresentPoc = 0;

  if (this->isIDRorBLA()) return 0; //Assume that all pic in the DPB will be flushed anyway so no need to check.

#if JVET_O1159_SCALABILITY
  int numberOfPictures = pRPL->getNumberOfLongtermPictures() + pRPL->getNumberOfShorttermPictures() + pRPL->getNumberOfInterLayerPictures();
#else
  int numberOfPictures = pRPL->getNumberOfLongtermPictures() + pRPL->getNumberOfShorttermPictures();
#endif
  //Check long term ref pics
  for (int ii = 0; pRPL->getNumberOfLongtermPictures() > 0 && ii < numberOfPictures; ii++)
  {
#if JVET_O1159_SCALABILITY
    if( !pRPL->isRefPicLongterm( ii ) || pRPL->isInterLayerRefPic( ii ) )
    {
      continue;
    }
#else
    if (!pRPL->isRefPicLongterm(ii))
      continue;
#endif

    notPresentPoc = pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
      int curPoc = rpcPic->getPOC() & (pocCycle - 1);
      int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
      if (rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);
        int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
        int curPoc = rpcPic->getPOC() & (pocCycle - 1);
        int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
        if (!rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
        {
          isAvailable = 1;
          rpcPic->longTerm = true;
          break;
        }
      }
    }
    if (!isAvailable)
    {
      if (printErrors)
      {
        msg(ERROR, "\nCurrent picture: %d Long-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      return notPresentPoc;
    }
  }
  //report that a picture is lost if it is in the Reference Picture List but not in the DPB

  isAvailable = 0;
  //Check short term ref pics
  for (int ii = 0; ii < numberOfPictures; ii++)
  {
    if (pRPL->isRefPicLongterm(ii))
      continue;

    notPresentPoc = this->getPOC() - pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if (!rpcPic->longTerm && rpcPic->getPOC() == this->getPOC() - pRPL->getRefPicIdentifier(ii) && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    //report that a picture is lost if it is in the Reference Picture List but not in the DPB
    if (isAvailable == 0 && pRPL->getNumberOfShorttermPictures() > 0)
    {
      if (printErrors)
      {
        msg(ERROR, "\nCurrent picture: %d Short-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      return notPresentPoc;
    }
  }
  return 0;
}

int Slice::checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx, bool printErrors, int *refPicIndex) const
{
  Picture* rpcPic;
  int isAvailable = 0;
  int notPresentPoc = 0;
  *refPicIndex = 0;

  if (this->isIDRorBLA()) return 0; //Assume that all pic in the DPB will be flushed anyway so no need to check.

#if JVET_O1159_SCALABILITY
  int numberOfPictures = pRPL->getNumberOfLongtermPictures() + pRPL->getNumberOfShorttermPictures() + pRPL->getNumberOfInterLayerPictures();
#else
  int numberOfPictures = pRPL->getNumberOfLongtermPictures() + pRPL->getNumberOfShorttermPictures();
#endif
  //Check long term ref pics
  for (int ii = 0; pRPL->getNumberOfLongtermPictures() > 0 && ii < numberOfPictures; ii++)
  {
#if JVET_O1159_SCALABILITY
    if( !pRPL->isRefPicLongterm( ii ) || pRPL->isInterLayerRefPic( ii ) )
    {
      continue;
    }
#else
    if (!pRPL->isRefPicLongterm(ii))
      continue;
#endif

    notPresentPoc = pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
      int curPoc = rpcPic->getPOC() & (pocCycle - 1);
      int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
      if (rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);
        int pocCycle = 1 << (rpcPic->cs->sps->getBitsForPOC());
        int curPoc = rpcPic->getPOC() & (pocCycle - 1);
        int refPoc = pRPL->getRefPicIdentifier(ii) & (pocCycle - 1);
        if (!rpcPic->longTerm && curPoc == refPoc && rpcPic->referenced)
        {
          isAvailable = 1;
          rpcPic->longTerm = true;
          break;
        }
      }
    }
    if (!isAvailable)
    {
      if (printErrors)
      {
        msg(ERROR, "\nCurrent picture: %d Long-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      *refPicIndex = ii;
      return notPresentPoc;
    }
  }
  //report that a picture is lost if it is in the Reference Picture List but not in the DPB

  isAvailable = 0;
  //Check short term ref pics
  for (int ii = 0; ii < numberOfPictures; ii++)
  {
    if (pRPL->isRefPicLongterm(ii))
      continue;

    notPresentPoc = this->getPOC() - pRPL->getRefPicIdentifier(ii);
    isAvailable = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if (!rpcPic->longTerm && rpcPic->getPOC() == this->getPOC() - pRPL->getRefPicIdentifier(ii) && rpcPic->referenced)
      {
        isAvailable = 1;
        break;
      }
    }
    //report that a picture is lost if it is in the Reference Picture List but not in the DPB
    if (isAvailable == 0 && pRPL->getNumberOfShorttermPictures() > 0)
    {
      if (printErrors)
      {
        msg(ERROR, "\nCurrent picture: %d Short-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC(), notPresentPoc);
      }
      *refPicIndex = ii;
      return notPresentPoc;
    }
  }
  return 0;
}

bool Slice::isPOCInRefPicList(const ReferencePictureList *rpl, int poc )
{
#if JVET_O1159_SCALABILITY
  for( int i = 0; i < rpl->getNumberOfLongtermPictures() + rpl->getNumberOfShorttermPictures() + rpl->getNumberOfInterLayerPictures(); i++ )
  {
    if( rpl->isInterLayerRefPic( i ) )
    {
      // Diagonal inter-layer prediction is not allowed
      CHECK( rpl->getRefPicIdentifier( i ), "ILRP identifier should be 0" );

      if( poc == m_iPOC )
      {
        return true;
      }
    }
    else
#else
  for (int i = 0; i < rpl->getNumberOfLongtermPictures() + rpl->getNumberOfShorttermPictures(); i++)
  {
#endif
    if (rpl->isRefPicLongterm(i))
    {
      if (poc == rpl->getRefPicIdentifier(i))
      {
        return true;
      }
    }
    else
    {
      if (poc == getPOC() - rpl->getRefPicIdentifier(i))
      {
        return true;
      }
    }
  }
  return false;
}

bool Slice::isPocRestrictedByDRAP( int poc, bool precedingDRAPInDecodingOrder )
{
  if (!getEnableDRAPSEI())
  {
    return false;
  }
  return ( isDRAP() && poc != getAssociatedIRAPPOC() ) ||
         ( cvsHasPreviousDRAP() && getPOC() > getLatestDRAPPOC() && (precedingDRAPInDecodingOrder || poc < getLatestDRAPPOC()) );
}

void Slice::checkConformanceForDRAP( uint32_t temporalId )
{
  if (!(isDRAP() || cvsHasPreviousDRAP()))
  {
    return;
  }

  if (isDRAP())
  {
    if (!(getNalUnitType() == NalUnitType::NAL_UNIT_CODED_SLICE_TRAIL ||
          getNalUnitType() == NalUnitType::NAL_UNIT_CODED_SLICE_STSA))
    {
      msg( WARNING, "Warning, non-conforming bitstream. The DRAP picture should be a trailing picture.\n");
    }
    if ( temporalId != 0)
    {
      msg( WARNING, "Warning, non-conforming bitstream. The DRAP picture shall have a temporal sublayer identifier equal to 0.\n");
    }
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_0); i++)
    {
      if (getRefPic(REF_PIC_LIST_0,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg( WARNING, "Warning, non-conforming bitstream. The DRAP picture shall not include any pictures in the active "
                      "entries of its reference picture lists except the preceding IRAP picture in decoding order.\n");
      }
    }
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_1); i++)
    {
      if (getRefPic(REF_PIC_LIST_1,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg( WARNING, "Warning, non-conforming bitstream. The DRAP picture shall not include any pictures in the active "
                      "entries of its reference picture lists except the preceding IRAP picture in decoding order.\n");
      }
    }
  }

  if (cvsHasPreviousDRAP() && getPOC() > getLatestDRAPPOC())
  {
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_0); i++)
    {
      if (getRefPic(REF_PIC_LIST_0,i)->getPOC() < getLatestDRAPPOC() && getRefPic(REF_PIC_LIST_0,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg( WARNING, "Warning, non-conforming bitstream. Any picture that follows the DRAP picture in both decoding order "
                    "and output order shall not include, in the active entries of its reference picture lists, any picture "
                    "that precedes the DRAP picture in decoding order or output order, with the exception of the preceding "
                    "IRAP picture in decoding order. Problem is POC %d in RPL0.\n", getRefPic(REF_PIC_LIST_0,i)->getPOC());
      }
    }
    for (int i = 0; i < getNumRefIdx(REF_PIC_LIST_1); i++)
    {
      if (getRefPic(REF_PIC_LIST_1,i)->getPOC() < getLatestDRAPPOC() && getRefPic(REF_PIC_LIST_1,i)->getPOC() != getAssociatedIRAPPOC())
      {
        msg( WARNING, "Warning, non-conforming bitstream. Any picture that follows the DRAP picture in both decoding order "
                    "and output order shall not include, in the active entries of its reference picture lists, any picture "
                    "that precedes the DRAP picture in decoding order or output order, with the exception of the preceding "
                    "IRAP picture in decoding order. Problem is POC %d in RPL1", getRefPic(REF_PIC_LIST_1,i)->getPOC());
      }
    }
  }
}

#if !JVET_O1159_SCALABILITY
void Slice::createExplicitReferencePictureSetFromReference(PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1)
{
  Picture* rpcPic;
  int pocCycle = 0;


  ReferencePictureList* pLocalRPL0 = this->getLocalRPL0();
  (*pLocalRPL0) = ReferencePictureList();

  uint32_t numOfSTRPL0 = 0;
  uint32_t numOfLTRPL0 = 0;
  uint32_t numOfRefPic = pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures();
  uint32_t refPicIdxL0 = 0;
  for (int ii = 0; ii < numOfRefPic; ii++)
  {
    // loop through all pictures in the reference picture buffer
    PicList::iterator iterPic = rcListPic.begin();
    bool isAvailable = false;

    pocCycle = 1 << (this->getSPS()->getBitsForPOC());
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if (!pRPL0->isRefPicLongterm(ii) && rpcPic->referenced && rpcPic->getPOC() == this->getPOC() - pRPL0->getRefPicIdentifier(ii) && !isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
      {
        isAvailable = true;
        break;
      }
      else if (pRPL0->isRefPicLongterm(ii) && rpcPic->referenced && (rpcPic->getPOC() & (pocCycle - 1)) == pRPL0->getRefPicIdentifier(ii) && !isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
      {
        isAvailable = true;
        break;
      }
    }
    if (isAvailable)
    {
      pLocalRPL0->setRefPicIdentifier(refPicIdxL0, pRPL0->getRefPicIdentifier(ii), pRPL0->isRefPicLongterm(ii));
      refPicIdxL0++;
      numOfSTRPL0 = numOfSTRPL0 + ((pRPL0->isRefPicLongterm(ii)) ? 0 : 1);
      numOfLTRPL0 = numOfLTRPL0 + ((pRPL0->isRefPicLongterm(ii)) ? 1 : 0);
      isAvailable = false;
    }
  }

  if (getEnableDRAPSEI())
  {
    pLocalRPL0->setNumberOfShorttermPictures(numOfSTRPL0);
    pLocalRPL0->setNumberOfLongtermPictures(numOfLTRPL0);
    if (!isIRAP() && !isPOCInRefPicList(pLocalRPL0, getAssociatedIRAPPOC()))
    {
      if (getUseLTforDRAP() && !isPOCInRefPicList(pRPL1, getAssociatedIRAPPOC()))
      {
        // Adding associated IRAP as longterm picture
        pLocalRPL0->setRefPicIdentifier(refPicIdxL0, getAssociatedIRAPPOC(), true);
        refPicIdxL0++;
        numOfLTRPL0++;
      }
      else
      {
        // Adding associated IRAP as shortterm picture
        pLocalRPL0->setRefPicIdentifier(refPicIdxL0, this->getPOC() - getAssociatedIRAPPOC(), false);
        refPicIdxL0++;
        numOfSTRPL0++;
      }
    }
  }

  ReferencePictureList* pLocalRPL1 = this->getLocalRPL1();
  (*pLocalRPL1) = ReferencePictureList();

  uint32_t numOfSTRPL1 = 0;
  uint32_t numOfLTRPL1 = 0;
  numOfRefPic = pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures();
  uint32_t refPicIdxL1 = 0;
  for (int ii = 0; ii < numOfRefPic; ii++)
  {
    // loop through all pictures in the reference picture buffer
    PicList::iterator iterPic = rcListPic.begin();
    bool isAvailable = false;
    pocCycle = 1 << (this->getSPS()->getBitsForPOC());
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if (!pRPL1->isRefPicLongterm(ii) && rpcPic->referenced && rpcPic->getPOC() == this->getPOC() - pRPL1->getRefPicIdentifier(ii) && !isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
      {
        isAvailable = true;
        break;
      }
      else if (pRPL1->isRefPicLongterm(ii) && rpcPic->referenced && (rpcPic->getPOC() & (pocCycle - 1)) == pRPL1->getRefPicIdentifier(ii) && !isPocRestrictedByDRAP(rpcPic->getPOC(), rpcPic->precedingDRAP))
      {
        isAvailable = true;
        break;
      }
    }
    if (isAvailable)
    {
      pLocalRPL1->setRefPicIdentifier(refPicIdxL1, pRPL1->getRefPicIdentifier(ii), pRPL1->isRefPicLongterm(ii));
      refPicIdxL1++;
      numOfSTRPL1 = numOfSTRPL1 + ((pRPL1->isRefPicLongterm(ii)) ? 0 : 1);
      numOfLTRPL1 = numOfLTRPL1 + ((pRPL1->isRefPicLongterm(ii)) ? 1 : 0);
      isAvailable = false;
    }
  }

  //Copy from L1 if we have less than active ref pic
  int numOfNeedToFill = pRPL0->getNumberOfActivePictures() - (numOfLTRPL0 + numOfSTRPL0);
  bool isDisallowMixedRefPic = (this->getSPS()->getAllActiveRplEntriesHasSameSignFlag()) ? true : false;
  int originalL0StrpNum = numOfSTRPL0;
  int originalL0LtrpNum = numOfLTRPL0;
  for (int ii = 0; numOfNeedToFill > 0 && ii < (pLocalRPL1->getNumberOfLongtermPictures() + pLocalRPL1->getNumberOfShorttermPictures()); ii++)
  {
    if (ii <= (numOfLTRPL1 + numOfSTRPL1 - 1))
    {
      //Make sure this copy is not already in L0
      bool canIncludeThis = true;
      for (int jj = 0; jj < refPicIdxL0; jj++)
      {
        if ((pLocalRPL1->getRefPicIdentifier(ii) == pLocalRPL0->getRefPicIdentifier(jj)) && (pLocalRPL1->isRefPicLongterm(ii) == pLocalRPL0->isRefPicLongterm(jj)))
          canIncludeThis = false;
        bool sameSign = (pLocalRPL1->getRefPicIdentifier(ii) > 0) == (pLocalRPL0->getRefPicIdentifier(0) > 0);
        if (isDisallowMixedRefPic && canIncludeThis && !pLocalRPL1->isRefPicLongterm(ii) && !sameSign)
          canIncludeThis = false;
      }
      if (canIncludeThis)
      {
        pLocalRPL0->setRefPicIdentifier(refPicIdxL0, pLocalRPL1->getRefPicIdentifier(ii), pLocalRPL1->isRefPicLongterm(ii));
        refPicIdxL0++;
        numOfSTRPL0 = numOfSTRPL0 + ((pRPL1->isRefPicLongterm(ii)) ? 0 : 1);
        numOfLTRPL0 = numOfLTRPL0 + ((pRPL1->isRefPicLongterm(ii)) ? 1 : 0);

        numOfNeedToFill--;
      }
    }
  }
  pLocalRPL0->setNumberOfLongtermPictures(numOfLTRPL0);
  pLocalRPL0->setNumberOfShorttermPictures(numOfSTRPL0);
  pLocalRPL0->setNumberOfActivePictures((numOfLTRPL0 + numOfSTRPL0 < pRPL0->getNumberOfActivePictures()) ? numOfLTRPL0 + numOfSTRPL0 : pRPL0->getNumberOfActivePictures());
  pLocalRPL0->setLtrpInSliceHeaderFlag(pRPL0->getLtrpInSliceHeaderFlag());
  this->setRPL0idx(-1);
  this->setRPL0(pLocalRPL0);

  //Copy from L0 if we have less than active ref pic
  numOfNeedToFill = pLocalRPL0->getNumberOfActivePictures() - (numOfLTRPL1 + numOfSTRPL1);
  for (int ii = 0; numOfNeedToFill > 0 && ii < (pLocalRPL0->getNumberOfLongtermPictures() + pLocalRPL0->getNumberOfShorttermPictures()); ii++)
  {
    if (ii <= (originalL0StrpNum + originalL0LtrpNum - 1))
    {
      //Make sure this copy is not already in L0
      bool canIncludeThis = true;
      for (int jj = 0; jj < refPicIdxL1; jj++)
      {
        if ((pLocalRPL0->getRefPicIdentifier(ii) == pLocalRPL1->getRefPicIdentifier(jj)) && (pLocalRPL0->isRefPicLongterm(ii) == pLocalRPL1->isRefPicLongterm(jj)))
          canIncludeThis = false;
        bool sameSign = (pLocalRPL0->getRefPicIdentifier(ii) > 0) == (pLocalRPL1->getRefPicIdentifier(0) > 0);
        if (isDisallowMixedRefPic && canIncludeThis && !pLocalRPL0->isRefPicLongterm(ii) && !sameSign)
          canIncludeThis = false;
      }
      if (canIncludeThis)
      {
        pLocalRPL1->setRefPicIdentifier(refPicIdxL1, pLocalRPL0->getRefPicIdentifier(ii), pLocalRPL0->isRefPicLongterm(ii));
        refPicIdxL1++;
        numOfSTRPL1 = numOfSTRPL1 + ((pLocalRPL0->isRefPicLongterm(ii)) ? 0 : 1);
        numOfLTRPL1 = numOfLTRPL1 + ((pLocalRPL0->isRefPicLongterm(ii)) ? 1 : 0);
        numOfNeedToFill--;
      }
    }
  }
  pLocalRPL1->setNumberOfLongtermPictures(numOfLTRPL1);
  pLocalRPL1->setNumberOfShorttermPictures(numOfSTRPL1);
  pLocalRPL1->setNumberOfActivePictures((isDisallowMixedRefPic) ? (numOfLTRPL1 + numOfSTRPL1) : (((numOfLTRPL1 + numOfSTRPL1) < pRPL1->getNumberOfActivePictures()) ? (numOfLTRPL1 + numOfSTRPL1) : pRPL1->getNumberOfActivePictures()));
  pLocalRPL1->setLtrpInSliceHeaderFlag(pRPL1->getLtrpInSliceHeaderFlag());
  this->setRPL1idx(-1);
  this->setRPL1(pLocalRPL1);
}
#endif

//! get AC and DC values for weighted pred
void  Slice::getWpAcDcParam(const WPACDCParam *&wp) const
{
  wp = m_weightACDCParam;
}

//! init AC and DC values for weighted pred
void  Slice::initWpAcDcParam()
{
  for(int iComp = 0; iComp < MAX_NUM_COMPONENT; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}

//! get tables for weighted prediction
void  Slice::getWpScaling( RefPicList e, int iRefIdx, WPScalingParam *&wp ) const
{
  CHECK(e>=NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  wp = (WPScalingParam*) m_weightPredTable[e][iRefIdx];
}

//! reset Default WP tables settings : no weight.
void  Slice::resetWpScaling()
{
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;
      }
    }
  }
}

//! init WP table
void  Slice::initWpScaling(const SPS *sps)
{
  const bool bUseHighPrecisionPredictionWeighting = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        if ( !pwp->bPresentFlag )
        {
          // Inferring values not present :
          pwp->iWeight = (1 << pwp->uiLog2WeightDenom);
          pwp->iOffset = 0;
        }

        const int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (sps->getBitDepth(toChannelType(ComponentID(yuv)))-8));

        pwp->w      = pwp->iWeight;
        pwp->o      = pwp->iOffset * offsetScalingFactor; //NOTE: This value of the ".o" variable is never used - .o is set immediately before it gets used
        pwp->shift  = pwp->uiLog2WeightDenom;
        pwp->round  = (pwp->uiLog2WeightDenom>=1) ? (1 << (pwp->uiLog2WeightDenom-1)) : (0);
      }
    }
  }
}


void Slice::startProcessingTimer()
{
  m_iProcessingStartTime = clock();
}

void Slice::stopProcessingTimer()
{
  m_dProcessingTime += (double)(clock()-m_iProcessingStartTime) / CLOCKS_PER_SEC;
  m_iProcessingStartTime = 0;
}

unsigned Slice::getMinPictureDistance() const
{
  int minPicDist = MAX_INT;
  if (getSPS()->getIBCFlag())
  {
    minPicDist = 0;
  }
  else
  if( ! isIntra() )
  {
    const int currPOC  = getPOC();
    for (int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_0); refIdx++)
    {
      minPicDist = std::min( minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_0, refIdx)->getPOC()));
    }
    if( getSliceType() == B_SLICE )
    {
      for (int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_1); refIdx++)
      {
        minPicDist = std::min(minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_1, refIdx)->getPOC()));
      }
    }
  }
  return (unsigned) minPicDist;
}

// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
VPS::VPS()
  : m_VPSId(0)
  , m_uiMaxLayers(1)
  , m_vpsExtensionFlag()
{
#if JVET_O1159_SCALABILITY
  m_vpsAllLayersSameNumSubLayersFlag = true;
  m_vpsAllIndependentLayersFlag = true;
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_vpsLayerId[i] = 0;
    m_vpsIndependentLayerFlag[i] = 1;
    for (int j = 0; j < MAX_VPS_LAYERS; j++)
    {
      m_vpsDirectRefLayerFlag[i][j] = 0;
      m_directRefLayerIdx[i][j] = MAX_VPS_LAYERS;
      m_interLayerRefIdx[i][i] = NOT_VALID;
    }
  }
  m_vpsEachLayerIsAnOlsFlag = 1;
  m_vpsOlsModeIdc = 0;
  m_vpsNumOutputLayerSets = 1;
  for (int i = 0; i < MAX_NUM_OLSS; i++)
  {
    for (int j = 0; j < MAX_VPS_LAYERS; j++)
    {
      m_vpsOlsOutputLayerFlag[i][j] = 0;
    }
  }
#else
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_vpsIncludedLayerId[i] = 0;
  }
#endif
}

VPS::~VPS()
{
}

#if JVET_P1006_PICTURE_HEADER
// ------------------------------------------------------------------------------------------------
// Picture Header
// ------------------------------------------------------------------------------------------------

PicHeader::PicHeader()
: m_valid                                         ( 0 )
, m_nonReferencePictureFlag                       ( 0 )
, m_gdrPicFlag                                    ( 0 )
, m_noOutputOfPriorPicsFlag                       ( 0 )
, m_recoveryPocCnt                                ( 0 )
, m_spsId                                         ( -1 )
, m_ppsId                                         ( -1 )
, m_subPicIdSignallingPresentFlag                 ( 0 )
, m_subPicIdLen                                   ( 0 )
, m_loopFilterAcrossVirtualBoundariesDisabledFlag ( 0 )
, m_numVerVirtualBoundaries                       ( 0 )
, m_numHorVirtualBoundaries                       ( 0 )
, m_colourPlaneId                                 ( 0 )
, m_picOutputFlag                                 ( true )
, m_picRplPresentFlag                             ( 0 )
, m_pRPL0                                         ( 0 )
, m_pRPL1                                         ( 0 )
, m_rpl0Idx                                       ( 0 )
, m_rpl1Idx                                       ( 0 )
, m_splitConsOverrideFlag                         ( 0 )
, m_cuQpDeltaSubdivIntra                          ( 0 )
, m_cuQpDeltaSubdivInter                          ( 0 )
, m_cuChromaQpOffsetSubdivIntra                   ( 0 )
, m_cuChromaQpOffsetSubdivInter                   ( 0 )
, m_enableTMVPFlag                                ( true )
, m_mvdL1ZeroFlag                                 ( 0 )
, m_maxNumMergeCand                               ( MRG_MAX_NUM_CANDS )
, m_maxNumAffineMergeCand                         ( AFFINE_MRG_MAX_NUM_CANDS )
, m_disFracMMVD                                   ( 0 )
#if JVET_P0314_PROF_BDOF_DMVR_HLS
, m_disBdofFlag                                   ( 0 )
, m_disDmvrFlag                                   ( 0 )
, m_disProfFlag                                   ( 0 )
#else
, m_disBdofDmvrFlag                               ( 0 )
#endif
, m_maxNumTriangleCand                            ( 0 )
, m_maxNumIBCMergeCand                            ( IBC_MRG_MAX_NUM_CANDS )
, m_jointCbCrSignFlag                             ( 0 )
, m_saoEnabledPresentFlag                         ( 0 )
, m_alfEnabledPresentFlag                         ( 0 )
, m_numAlfAps                                     ( 0 )
, m_alfApsId                                      ( 0 )
, m_alfChromaApsId                                ( 0 )
, m_depQuantEnabledFlag                           ( 0 )
, m_signDataHidingEnabledFlag                     ( 0 )
, m_deblockingFilterOverridePresentFlag           ( 0 )
, m_deblockingFilterOverrideFlag                  ( 0 )
, m_deblockingFilterDisable                       ( 0 )
, m_deblockingFilterBetaOffsetDiv2                ( 0 )
, m_deblockingFilterTcOffsetDiv2                  ( 0 )
, m_lmcsEnabledFlag                               ( 0 )
, m_lmcsApsId                                     ( -1 )
, m_lmcsAps                                       ( nullptr )
, m_lmcsChromaResidualScaleFlag                   ( 0 )
, m_scalingListPresentFlag                        ( 0 )
, m_scalingListApsId                              ( -1 )
, m_scalingListAps                                ( nullptr )
{
  memset(m_subPicId,                                0,    sizeof(m_subPicId));
  memset(m_virtualBoundariesPosX,                   0,    sizeof(m_virtualBoundariesPosX));
  memset(m_virtualBoundariesPosY,                   0,    sizeof(m_virtualBoundariesPosY));
  memset(m_saoEnabledFlag,                          0,    sizeof(m_saoEnabledFlag));
  memset(m_alfEnabledFlag,                          0,    sizeof(m_alfEnabledFlag));
  memset(m_minQT,                                   0,    sizeof(m_minQT));
  memset(m_maxMTTHierarchyDepth,                    0,    sizeof(m_maxMTTHierarchyDepth));
  memset(m_maxBTSize,                               0,    sizeof(m_maxBTSize));
  memset(m_maxTTSize,                               0,    sizeof(m_maxTTSize));

  m_localRPL0.setNumberOfActivePictures(0);
  m_localRPL0.setNumberOfShorttermPictures(0);
  m_localRPL0.setNumberOfLongtermPictures(0);
  m_localRPL0.setLtrpInSliceHeaderFlag(0);
#if JVET_O1159_SCALABILITY
  m_localRPL0.setNumberOfInterLayerPictures( 0 );
#endif

  m_localRPL1.setNumberOfActivePictures(0);
  m_localRPL1.setNumberOfShorttermPictures(0);
  m_localRPL1.setNumberOfLongtermPictures(0);
  m_localRPL1.setLtrpInSliceHeaderFlag(0);
#if JVET_O1159_SCALABILITY
  m_localRPL1.setNumberOfInterLayerPictures( 0 );
#endif

  m_alfApsId.resize(0);
}

PicHeader::~PicHeader()
{
  m_alfApsId.resize(0);
}

/**
 - initialize picture header to defaut state
 */
void PicHeader::initPicHeader()
{
  m_valid                                         = 0;
  m_nonReferencePictureFlag                       = 0;
  m_gdrPicFlag                                    = 0;
  m_noOutputOfPriorPicsFlag                       = 0;
  m_recoveryPocCnt                                = 0;
  m_spsId                                         = -1;
  m_ppsId                                         = -1;
  m_subPicIdSignallingPresentFlag                 = 0;
  m_subPicIdLen                                   = 0;
  m_loopFilterAcrossVirtualBoundariesDisabledFlag = 0;
  m_numVerVirtualBoundaries                       = 0;
  m_numHorVirtualBoundaries                       = 0;
  m_colourPlaneId                                 = 0;
  m_picOutputFlag                                 = true;
  m_picRplPresentFlag                             = 0;
  m_pRPL0                                         = 0;
  m_pRPL1                                         = 0;
  m_rpl0Idx                                       = 0;
  m_rpl1Idx                                       = 0;
  m_splitConsOverrideFlag                         = 0;
  m_cuQpDeltaSubdivIntra                          = 0;
  m_cuQpDeltaSubdivInter                          = 0;
  m_cuChromaQpOffsetSubdivIntra                   = 0;
  m_cuChromaQpOffsetSubdivInter                   = 0;
  m_enableTMVPFlag                                = true;
  m_mvdL1ZeroFlag                                 = 0;
  m_maxNumMergeCand                               = MRG_MAX_NUM_CANDS;
  m_maxNumAffineMergeCand                         = AFFINE_MRG_MAX_NUM_CANDS;
  m_disFracMMVD                                   = 0;
#if JVET_P0314_PROF_BDOF_DMVR_HLS
  m_disBdofFlag                                   = 0;
  m_disDmvrFlag                                   = 0;
  m_disProfFlag                                   = 0;
#else
  m_disBdofDmvrFlag                               = 0;
#endif
  m_maxNumTriangleCand                            = 0;
  m_maxNumIBCMergeCand                            = IBC_MRG_MAX_NUM_CANDS;
  m_jointCbCrSignFlag                             = 0;
  m_saoEnabledPresentFlag                         = 0;
  m_alfEnabledPresentFlag                         = 0;
  m_numAlfAps                                     = 0;
  m_alfChromaApsId                                = 0;
  m_depQuantEnabledFlag                           = 0;
  m_signDataHidingEnabledFlag                     = 0;
  m_deblockingFilterOverridePresentFlag           = 0;
  m_deblockingFilterOverrideFlag                  = 0;
  m_deblockingFilterDisable                       = 0;
  m_deblockingFilterBetaOffsetDiv2                = 0;
  m_deblockingFilterTcOffsetDiv2                  = 0;
  m_lmcsEnabledFlag                               = 0;
  m_lmcsApsId                                     = -1;
  m_lmcsAps                                       = nullptr;
  m_lmcsChromaResidualScaleFlag                   = 0;
  m_scalingListPresentFlag                        = 0;
  m_scalingListApsId                              = -1;
  m_scalingListAps                                = nullptr;
  memset(m_subPicId,                                0,    sizeof(m_subPicId));
  memset(m_virtualBoundariesPosX,                   0,    sizeof(m_virtualBoundariesPosX));
  memset(m_virtualBoundariesPosY,                   0,    sizeof(m_virtualBoundariesPosY));
  memset(m_saoEnabledFlag,                          0,    sizeof(m_saoEnabledFlag));
  memset(m_alfEnabledFlag,                          0,    sizeof(m_alfEnabledFlag));
  memset(m_minQT,                                   0,    sizeof(m_minQT));
  memset(m_maxMTTHierarchyDepth,                    0,    sizeof(m_maxMTTHierarchyDepth));
  memset(m_maxBTSize,                               0,    sizeof(m_maxBTSize));
  memset(m_maxTTSize,                               0,    sizeof(m_maxTTSize));

  m_localRPL0.setNumberOfActivePictures(0);
  m_localRPL0.setNumberOfShorttermPictures(0);
  m_localRPL0.setNumberOfLongtermPictures(0);
  m_localRPL0.setLtrpInSliceHeaderFlag(0);

  m_localRPL1.setNumberOfActivePictures(0);
  m_localRPL1.setNumberOfShorttermPictures(0);
  m_localRPL1.setNumberOfLongtermPictures(0);
  m_localRPL1.setLtrpInSliceHeaderFlag(0);

  m_alfApsId.resize(0);
}

#endif
// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------
SPSRExt::SPSRExt()
 : m_transformSkipRotationEnabledFlag   (false)
 , m_transformSkipContextEnabledFlag    (false)
// m_rdpcmEnabledFlag initialized below
 , m_extendedPrecisionProcessingFlag    (false)
 , m_intraSmoothingDisabledFlag         (false)
 , m_highPrecisionOffsetsEnabledFlag    (false)
 , m_persistentRiceAdaptationEnabledFlag(false)
 , m_cabacBypassAlignmentEnabledFlag    (false)
{
  for (uint32_t signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_rdpcmEnabledFlag[signallingModeIndex] = false;
  }
}


SPS::SPS()
: m_SPSId                     (  0)
, m_decodingParameterSetId    (  0 )
#if JVET_P0205_VPS_ID_0
, m_VPSId                     ( 0 )
#endif
, m_affineAmvrEnabledFlag     ( false )
, m_DMVR                      ( false )
, m_MMVD                      ( false )
, m_SBT                       ( false )
#if !JVET_P0983_REMOVE_SPS_SBT_MAX_SIZE_FLAG
, m_MaxSbtSize                ( 32 )
#endif
, m_ISP                       ( false )
, m_chromaFormatIdc           (CHROMA_420)
#if JVET_P1006_PICTURE_HEADER
, m_separateColourPlaneFlag(0)
#endif
, m_uiMaxTLayers              (  1)
// Structure
, m_maxWidthInLumaSamples     (352)
, m_maxHeightInLumaSamples    (288)
#if JVET_P0126_SIGNALLING_SUBPICID
, m_subPicPresentFlag         (0)
#endif
#if JVET_P1006_PICTURE_HEADER
, m_numSubPics(1)
, m_subPicIdPresentFlag(0)
, m_subPicIdSignallingPresentFlag(0)
, m_subPicIdLen(16)
#endif
, m_log2MinCodingBlockSize    (  0)
, m_log2DiffMaxMinCodingBlockSize(0)
, m_CTUSize(0)
, m_minQT{ 0, 0, 0 }
, m_maxMTTHierarchyDepth{ MAX_BT_DEPTH, MAX_BT_DEPTH_INTER, MAX_BT_DEPTH_C }
, m_maxBTSize{ MAX_BT_SIZE,  MAX_BT_SIZE_INTER,  MAX_BT_SIZE_C }
, m_maxTTSize{ MAX_TT_SIZE,  MAX_TT_SIZE_INTER,  MAX_TT_SIZE_C }
, m_uiMaxCUWidth              ( 32)
, m_uiMaxCUHeight             ( 32)
, m_uiMaxCodingDepth          (  3)
, m_numRPL0                   ( 0 )
, m_numRPL1                   ( 0 )
, m_rpl1CopyFromRpl0Flag      ( false )
, m_rpl1IdxPresentFlag        ( false )
, m_allRplEntriesHasSameSignFlag ( true )
, m_bLongTermRefsPresent      (false)
// Tool list
, m_transformSkipEnabledFlag  (false)
#if JVET_P0059_CHROMA_BDPCM
, m_BDPCMEnabled              (0)
#else
, m_BDPCMEnabledFlag          (false)
#endif
, m_JointCbCrEnabledFlag      (false)
, m_sbtmvpEnabledFlag         (false)
, m_bdofEnabledFlag           (false)
, m_fpelMmvdEnabledFlag       ( false )
#if JVET_P0314_PROF_BDOF_DMVR_HLS
, m_BdofControlPresentFlag    ( false )
, m_DmvrControlPresentFlag    ( false )
, m_ProfControlPresentFlag    ( false )
#else
, m_BdofDmvrSlicePresentFlag  ( false )
#endif
, m_uiBitsForPOC              (  8)
, m_numLongTermRefPicSPS      (  0)
, m_log2MaxTbSize             (  6)
, m_useWeightPred             (false)
, m_useWeightedBiPred         (false)
, m_saoEnabledFlag            (false)
, m_bTemporalIdNestingFlag    (false)
, m_scalingListEnabledFlag    (false)
#if JVET_P1006_PICTURE_HEADER
, m_loopFilterAcrossVirtualBoundariesDisabledFlag(0)
, m_numVerVirtualBoundaries(0)
, m_numHorVirtualBoundaries(0)
#endif
, m_hrdParametersPresentFlag  (false)
, m_vuiParametersPresentFlag  (false)
, m_vuiParameters             ()
, m_wrapAroundEnabledFlag     (false)
, m_wrapAroundOffset          (  0)
, m_IBCFlag                   (  0)
, m_PLTMode                   (  0)
, m_lmcsEnabled               (false)
, m_AMVREnabledFlag                       ( false )
, m_LMChroma                  ( false )
#if JVET_P0592_CHROMA_PHASE
, m_horCollocatedChromaFlag   ( true )
, m_verCollocatedChromaFlag   ( false )
#else
, m_cclmCollocatedChromaFlag  ( false )
#endif
, m_IntraMTS                  ( false )
, m_InterMTS                  ( false )
, m_LFNST                     ( false )
, m_Affine                    ( false )
, m_AffineType                ( false )
, m_PROF                      ( false )
, m_ciip                   ( false )
, m_Triangle                  ( false )
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
, m_LadfEnabled               ( false )
, m_LadfNumIntervals          ( 0 )
, m_LadfQpOffset              { 0 }
, m_LadfIntervalLowerBound    { 0 }
#endif
#if JVET_P2001_SYNTAX_ORDER_MISMATCHES
, m_MRL                       ( false )
#endif
, m_MIP                       ( false )
, m_GDREnabledFlag            ( true )
, m_SubLayerCbpParametersPresentFlag ( true )
#if JVET_P0590_SCALING_WINDOW
, m_rprEnabledFlag            ( false )
#endif

{
  for(int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_bitDepths.recon[ch] = 8;
    m_qpBDOffset   [ch] = 0;
  }

  for ( int i = 0; i < MAX_TLAYER; i++ )
  {
    m_uiMaxLatencyIncreasePlus1[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_numReorderPics[i]       = 0;
  }

  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_usedByCurrPicLtSPSFlag, 0, sizeof(m_usedByCurrPicLtSPSFlag));
#if JVET_P1006_PICTURE_HEADER
  ::memset(m_virtualBoundariesPosX, 0, sizeof(m_virtualBoundariesPosX));
  ::memset(m_virtualBoundariesPosY, 0, sizeof(m_virtualBoundariesPosY));
#endif
}

SPS::~SPS()
{
}

void  SPS::createRPLList0(int numRPL)
{
  m_RPLList0.destroy();
  m_RPLList0.create(numRPL + 1);
  m_numRPL0 = numRPL;
  m_rpl1IdxPresentFlag = (m_numRPL0 != m_numRPL1) ? true : false;
}
void  SPS::createRPLList1(int numRPL)
{
  m_RPLList1.destroy();
  m_RPLList1.create(numRPL + 1);
  m_numRPL1 = numRPL;

  m_rpl1IdxPresentFlag = (m_numRPL0 != m_numRPL1) ? true : false;
}


const int SPS::m_winUnitX[]={1,2,2,1};
const int SPS::m_winUnitY[]={1,2,1,1};

void ChromaQpMappingTable::setParams(const ChromaQpMappingTableParams &params, const int qpBdOffset)
{
  m_qpBdOffset = qpBdOffset;
  m_sameCQPTableForAllChromaFlag = params.m_sameCQPTableForAllChromaFlag;
#if JVET_P0667_QP_OFFSET_TABLE_SIGNALING_JCCR
  m_numQpTables = params.m_numQpTables;
#endif

  for (int i = 0; i < MAX_NUM_CQP_MAPPING_TABLES; i++)
  {
    m_numPtsInCQPTableMinus1[i] = params.m_numPtsInCQPTableMinus1[i];
    m_deltaQpInValMinus1[i] = params.m_deltaQpInValMinus1[i];
#if JVET_P0410_CHROMA_QP_MAPPING
    m_qpTableStartMinus26[i] = params.m_qpTableStartMinus26[i];
#endif
    m_deltaQpOutVal[i] = params.m_deltaQpOutVal[i];
  }
}
void ChromaQpMappingTable::derivedChromaQPMappingTables()
{
#if JVET_P0667_QP_OFFSET_TABLE_SIGNALING_JCCR
  for (int i = 0; i < getNumQpTables(); i++)
#else
  for (int i = 0; i < (getSameCQPTableForAllChromaFlag() ? 1 : 3); i++)
#endif
  {
    const int qpBdOffsetC = m_qpBdOffset;
    const int numPtsInCQPTableMinus1 = getNumPtsInCQPTableMinus1(i);
#if JVET_P0410_CHROMA_QP_MAPPING
    std::vector<int> qpInVal(numPtsInCQPTableMinus1 + 2), qpOutVal(numPtsInCQPTableMinus1 + 2);

    qpInVal[0] = getQpTableStartMinus26(i) + 26;
    qpOutVal[0] = qpInVal[0];
    for (int j = 0; j <= getNumPtsInCQPTableMinus1(i); j++)
    {
      qpInVal[j + 1] = qpInVal[j] + getDeltaQpInValMinus1(i, j) + 1;
      qpOutVal[j + 1] = qpOutVal[j] + getDeltaQpOutVal(i, j);
    }
#else
    std::vector<int> qpInVal(numPtsInCQPTableMinus1 + 1), qpOutVal(numPtsInCQPTableMinus1 + 1);

    qpInVal[0] = -qpBdOffsetC + getDeltaQpInValMinus1(i, 0);
    qpOutVal[0] = -qpBdOffsetC + getDeltaQpOutVal(i, 0);
    for (int j = 1; j <= getNumPtsInCQPTableMinus1(i); j++)
    {
      qpInVal[j] = qpInVal[j - 1] + getDeltaQpInValMinus1(i, j) + 1;
      qpOutVal[j] = qpOutVal[j - 1] + getDeltaQpOutVal(i, j);
    }
#endif

    for (int j = 0; j <= getNumPtsInCQPTableMinus1(i); j++)
    {
      CHECK(qpInVal[j]  < -qpBdOffsetC || qpInVal[j]  > MAX_QP, "qpInVal out of range");
      CHECK(qpOutVal[j] < -qpBdOffsetC || qpOutVal[j] > MAX_QP, "qpOutVal out of range");
    }

    m_chromaQpMappingTables[i][qpInVal[0]] = qpOutVal[0];
    for (int k = qpInVal[0] - 1; k >= -qpBdOffsetC; k--)
    {
      m_chromaQpMappingTables[i][k] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k + 1] - 1);
    }
#if JVET_P0410_CHROMA_QP_MAPPING
    for (int j = 0; j <= numPtsInCQPTableMinus1; j++)
#else
    for (int j = 0; j < numPtsInCQPTableMinus1; j++)
#endif
    {
#if JVET_P0410_CHROMA_QP_MAPPING
      int sh = (getDeltaQpInValMinus1(i, j) + 1) >> 1;
#else
      int sh = (getDeltaQpInValMinus1(i, j + 1) + 1) >> 1;
#endif
      for (int k = qpInVal[j] + 1, m = 1; k <= qpInVal[j + 1]; k++, m++)
      {
#if JVET_P0410_CHROMA_QP_MAPPING
        m_chromaQpMappingTables[i][k] = m_chromaQpMappingTables[i][qpInVal[j]]
          + ((qpOutVal[j + 1] - qpOutVal[j]) * m + sh) / (getDeltaQpInValMinus1(i, j) + 1);
#else
        m_chromaQpMappingTables[i][k] = m_chromaQpMappingTables[i][qpInVal[j]]
          + (getDeltaQpOutVal(i, j + 1) * m + sh) / (getDeltaQpInValMinus1(i, j + 1) + 1);
#endif
      }
    }
#if JVET_P0410_CHROMA_QP_MAPPING
    for (int k = qpInVal[numPtsInCQPTableMinus1 + 1] + 1; k <= MAX_QP; k++)
#else
    for (int k = qpInVal[numPtsInCQPTableMinus1]+1; k <= MAX_QP; k++)
#endif
    {
      m_chromaQpMappingTables[i][k] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k - 1] + 1);
    }
  }
}

#if JVET_P1004_REMOVE_BRICKS
SliceMap::SliceMap()
: m_sliceID              (0)
, m_numTilesInSlice      (0)
, m_numCtuInSlice        (0)
{
  m_ctuAddrInSlice.clear();
}

SliceMap::~SliceMap()
{
  m_numCtuInSlice = 0;
  m_ctuAddrInSlice.clear();
}

RectSlice::RectSlice()
: m_tileIdx            (0)
, m_sliceWidthInTiles  (0)
, m_sliceHeightInTiles (0)
, m_numSlicesInTile    (0)
, m_sliceHeightInCtu   (0)
{
}

RectSlice::~RectSlice()
{
}

#endif
PPSRExt::PPSRExt()
: m_crossComponentPredictionEnabledFlag(false)
// m_log2SaoOffsetScale initialized below
{
  for(int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_log2SaoOffsetScale[ch] = 0;
  }
}

PPS::PPS()
: m_PPSId                            (0)
, m_SPSId                            (0)
, m_picInitQPMinus26                 (0)
, m_useDQP                           (false)
, m_bSliceChromaQpFlag               (false)
#if !JVET_P1006_PICTURE_HEADER
, m_cuQpDeltaSubdiv                  (0)
#endif
, m_chromaCbQpOffset                 (0)
, m_chromaCrQpOffset                 (0)
, m_chromaCbCrQpOffset               (0)
#if !JVET_P1006_PICTURE_HEADER
, m_cuChromaQpOffsetSubdiv             (0)
#endif
, m_chromaQpOffsetListLen              (0)
, m_numRefIdxL0DefaultActive         (1)
, m_numRefIdxL1DefaultActive         (1)
, m_rpl1IdxPresentFlag               (false)
#if JVET_P1006_PICTURE_HEADER
, m_numSubPics                       (1)
, m_subPicIdSignallingPresentFlag    (0)
, m_subPicIdLen                      (16)
#endif
#if JVET_P1004_REMOVE_BRICKS
, m_noPicPartitionFlag               (1)
, m_log2CtuSize                      (0)
, m_ctuSize                          (0)
, m_picWidthInCtu                    (0)
, m_picHeightInCtu                   (0)
, m_numTileCols                      (1)
, m_numTileRows                      (1)
, m_rectSliceFlag                    (1)  
, m_numSlicesInPic                   (1)
, m_tileIdxDeltaPresentFlag          (0)
, m_loopFilterAcrossTilesEnabledFlag (1)
, m_loopFilterAcrossSlicesEnabledFlag(0)
#endif
#if !JVET_P2001_REMOVE_TRANSQUANT_BYPASS
, m_TransquantBypassEnabledFlag      (false)
#endif
, m_log2MaxTransformSkipBlockSize    (2)
, m_entropyCodingSyncEnabledFlag     (false)
#if !JVET_P1004_REMOVE_BRICKS
, m_loopFilterAcrossBricksEnabledFlag (true)
, m_uniformTileSpacingFlag           (false)
, m_numTileColumnsMinus1             (0)
, m_numTileRowsMinus1                (0)
, m_singleTileInPicFlag              (true)
, m_tileColsWidthMinus1              (0)
, m_tileRowsHeightMinus1             (0)
, m_brickSplittingPresentFlag        (false)
, m_singleBrickPerSliceFlag          (true)
, m_rectSliceFlag                    (true)
, m_numSlicesInPicMinus1             (0)
, m_numTilesInPic                    (1)
, m_numBricksInPic                   (1)
, m_signalledSliceIdFlag             (false)
,m_signalledSliceIdLengthMinus1      (0)
#endif
, m_constantSliceHeaderParamsEnabledFlag (false)
, m_PPSDepQuantEnabledIdc            (0)
, m_PPSRefPicListSPSIdc0             (0)
, m_PPSRefPicListSPSIdc1             (0)
#if !JVET_P0206_TMVP_flags
, m_PPSTemporalMVPEnabledIdc         (0)
#endif
, m_PPSMvdL1ZeroIdc                  (0)
, m_PPSCollocatedFromL0Idc           (0)
, m_PPSSixMinusMaxNumMergeCandPlus1  (0)
#if !JVET_P0152_REMOVE_PPS_NUM_SUBBLOCK_MERGE_CAND
, m_PPSFiveMinusMaxNumSubblockMergeCandPlus1 (0)
#endif
, m_PPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1 (0)
, m_cabacInitPresentFlag             (false)
#if JVET_P1006_PICTURE_HEADER
, m_pictureHeaderExtensionPresentFlag(0)
#endif
, m_sliceHeaderExtensionPresentFlag  (false)
#if !JVET_P1004_REMOVE_BRICKS
, m_loopFilterAcrossSlicesEnabledFlag(false)
#endif
, m_listsModificationPresentFlag     (0)
#if !JVET_P1006_PICTURE_HEADER
, m_loopFilterAcrossVirtualBoundariesDisabledFlag(false)
, m_numVerVirtualBoundaries          (0)
, m_numHorVirtualBoundaries          (0)
#endif
, m_picWidthInLumaSamples( 352 )
, m_picHeightInLumaSamples( 288 )
, m_ppsRangeExtension                ()
, pcv                                (NULL)
{
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset = 0; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here and never subsequently changed.
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.JointCbCrOffset = 0;
#if !JVET_P1006_PICTURE_HEADER
  ::memset(m_virtualBoundariesPosX, 0, sizeof(m_virtualBoundariesPosX));
  ::memset(m_virtualBoundariesPosY, 0, sizeof(m_virtualBoundariesPosY));
#endif
#if JVET_P1004_REMOVE_BRICKS
  m_tileColWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();
#endif
}

PPS::~PPS()
{
#if JVET_P1004_REMOVE_BRICKS
  m_tileColWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();

#endif
  delete pcv;
}

#if JVET_P1004_REMOVE_BRICKS
/**
 - reset tile and slice parameters and lists
 */
void PPS::resetTileSliceInfo()
{
  m_numExpTileCols = 0;
  m_numExpTileRows = 0;
  m_numTileCols    = 0;
  m_numTileRows    = 0;
  m_numSlicesInPic = 0;
  m_tileColWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();
}

/**
 - initialize tile row/column sizes and boundaries
 */
void PPS::initTiles()
{
  int       colIdx, rowIdx;
  int       ctuX, ctuY;
  
  // check explicit tile column sizes
  uint32_t  remainingWidthInCtu  = m_picWidthInCtu;
  for( colIdx = 0; colIdx < m_numExpTileCols; colIdx++ )
  {
    CHECK(m_tileColWidth[colIdx] > remainingWidthInCtu,    "Tile column width exceeds picture width");
    remainingWidthInCtu -= m_tileColWidth[colIdx];
  }

  // divide remaining picture width into uniform tile columns
  uint32_t  uniformTileColWidth = m_tileColWidth[colIdx-1];
  while( remainingWidthInCtu > 0 ) 
  {
    CHECK(colIdx >= MAX_TILE_COLS, "Number of tile columns exceeds valid range");
    uniformTileColWidth = std::min(remainingWidthInCtu, uniformTileColWidth);
    m_tileColWidth.push_back( uniformTileColWidth );
    remainingWidthInCtu -= uniformTileColWidth;
    colIdx++;
  }
  m_numTileCols = colIdx;
    
  // check explicit tile row sizes
  uint32_t  remainingHeightInCtu  = m_picHeightInCtu;
  for( rowIdx = 0; rowIdx < m_numExpTileRows; rowIdx++ )
  {
    CHECK(m_tileRowHeight[rowIdx] > remainingHeightInCtu,     "Tile row height exceeds picture height");
    remainingHeightInCtu -= m_tileRowHeight[rowIdx];
  }
    
  // divide remaining picture height into uniform tile rows
  uint32_t  uniformTileRowHeight = m_tileRowHeight[rowIdx - 1];
  while( remainingHeightInCtu > 0 ) 
  {
    CHECK(rowIdx >= MAX_TILE_ROWS, "Number of tile rows exceeds valid range");
    uniformTileRowHeight = std::min(remainingHeightInCtu, uniformTileRowHeight);
    m_tileRowHeight.push_back( uniformTileRowHeight );
    remainingHeightInCtu -= uniformTileRowHeight;
    rowIdx++;
  }
  m_numTileRows = rowIdx;

  // set left column bounaries
  m_tileColBd.push_back( 0 );
  for( colIdx = 0; colIdx < m_numTileCols; colIdx++ )
  {
    m_tileColBd.push_back( m_tileColBd[ colIdx ] + m_tileColWidth[ colIdx ] );
  }
  
  // set top row bounaries
  m_tileRowBd.push_back( 0 );
  for( rowIdx = 0; rowIdx < m_numTileRows; rowIdx++ )
  {
    m_tileRowBd.push_back( m_tileRowBd[ rowIdx ] + m_tileRowHeight[ rowIdx ] );
  }

  // set mapping between horizontal CTU address and tile column index
  colIdx = 0;
  for( ctuX = 0; ctuX <= m_picWidthInCtu; ctuX++ ) 
  {
    if( ctuX == m_tileColBd[ colIdx + 1 ] )
    {
      colIdx++;
    }
    m_ctuToTileCol.push_back( colIdx );
  }
  
  // set mapping between vertical CTU address and tile row index
  rowIdx = 0;
  for( ctuY = 0; ctuY <= m_picHeightInCtu; ctuY++ ) 
  {
    if( ctuY == m_tileRowBd[ rowIdx + 1 ] )
    {
      rowIdx++;
    }
    m_ctuToTileRow.push_back( rowIdx );
  }
}

/**
 - initialize memory for rectangular slice parameters
 */
void PPS::initRectSlices()
{ 
  CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_rectSlices.resize(m_numSlicesInPic);
}

/**
 - initialize mapping between rectangular slices and CTUs
 */
void PPS::initRectSliceMap()
{
  uint32_t  ctuY;
  uint32_t  tileX, tileY;
    
  // allocate new memory for slice list
  CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_sliceMap.resize( m_numSlicesInPic );

  // generate CTU maps for all rectangular slices in picture
  for( uint32_t i = 0; i < m_numSlicesInPic; i++ )
  {
    m_sliceMap[ i ].initSliceMap();

    // get position of first tile in slice
    tileX =  m_rectSlices[ i ].getTileIdx() % m_numTileCols;
    tileY =  m_rectSlices[ i ].getTileIdx() / m_numTileCols;
    
    // infer slice size for last slice in picture
    if( i == m_numSlicesInPic-1 ) 
    {
      m_rectSlices[ i ].setSliceWidthInTiles ( m_numTileCols - tileX );
      m_rectSlices[ i ].setSliceHeightInTiles( m_numTileRows - tileY );
      m_rectSlices[ i ].setNumSlicesInTile( 1 );
    }

    // set slice index
    m_sliceMap[ i ].setSliceID(i);
    
    // complete tiles within a single slice case
    if( m_rectSlices[ i ].getSliceWidthInTiles( ) > 1 || m_rectSlices[ i ].getSliceHeightInTiles( ) > 1)
    {
      for( uint32_t j = 0; j < m_rectSlices[ i ].getSliceHeightInTiles( ); j++ )
      {
        for( uint32_t k = 0; k < m_rectSlices[ i ].getSliceWidthInTiles( ); k++ )
        {
          m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX + k), getTileColumnBd(tileX + k +1),
                                          getTileRowBd(tileY + j), getTileRowBd(tileY + j +1), m_picWidthInCtu);
        }
      }
    }
    // multiple slices within a single tile case
    else 
    {
      uint32_t  numSlicesInTile = m_rectSlices[ i ].getNumSlicesInTile( );

      ctuY = getTileRowBd( tileY );
      for( uint32_t j = 0; j < numSlicesInTile-1; j++ ) 
      {
        m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX), getTileColumnBd(tileX+1),
                                        ctuY, ctuY + m_rectSlices[ i ].getSliceHeightInCtu(), m_picWidthInCtu);
        ctuY += m_rectSlices[ i ].getSliceHeightInCtu();
        i++;
        m_sliceMap[ i ].setSliceID(i);
      }

      // infer slice height for last slice in tile
      CHECK( ctuY >= getTileRowBd( tileY + 1 ), "Invalid rectangular slice signalling");
      m_rectSlices[ i ].setSliceHeightInCtu( getTileRowBd( tileY + 1 ) - ctuY );
      m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX), getTileColumnBd(tileX+1),
                                      ctuY, getTileRowBd( tileY + 1 ), m_picWidthInCtu);
    } 
  }

  // check for valid rectangular slice map
  checkSliceMap();
}

void PPS::initRasterSliceMap( std::vector<uint32_t> numTilesInSlice )
{
  uint32_t tileIdx = 0;
  setNumSlicesInPic( (uint32_t) numTilesInSlice.size() );

  // allocate new memory for slice list
  CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_sliceMap.resize( m_numSlicesInPic );

  for( uint32_t sliceIdx = 0; sliceIdx < numTilesInSlice.size(); sliceIdx++ ) 
  {
    m_sliceMap[sliceIdx].initSliceMap();
    m_sliceMap[sliceIdx].setSliceID( tileIdx );
    m_sliceMap[sliceIdx].setNumTilesInSlice( numTilesInSlice[sliceIdx] );
    for( uint32_t idx = 0; idx < numTilesInSlice[sliceIdx]; idx++ )
    {
      uint32_t tileX = tileIdx % getNumTileColumns();
      uint32_t tileY = tileIdx / getNumTileColumns();
      CHECK(tileY >= getNumTileRows(), "Number of tiles in slice exceeds the remaining number of tiles in picture");

      m_sliceMap[sliceIdx].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1),
                                          getTileRowBd(tileY), getTileRowBd(tileY + 1), 
                                          getPicWidthInCtu());
      tileIdx++;
    }
  }

  // check for valid raster-scan slice map
  checkSliceMap();
}

/**
 - check if slice map covers the entire picture without skipping or duplicating any CTU positions
 */
void PPS::checkSliceMap()
{
  uint32_t i;
  std::vector<uint32_t>  ctuList, sliceList;
  uint32_t picSizeInCtu = getPicWidthInCtu() * getPicHeightInCtu();
  for( i = 0; i < m_numSlicesInPic; i++ )
  {
    sliceList = m_sliceMap[ i ].getCtuAddrList();
    ctuList.insert( ctuList.end(), sliceList.begin(), sliceList.end() );
  }  
  CHECK( ctuList.size() < picSizeInCtu, "Slice map contains too few CTUs");
  CHECK( ctuList.size() > picSizeInCtu, "Slice map contains too many CTUs");
  std::sort( ctuList.begin(), ctuList.end() );   
  for( i = 1; i < ctuList.size(); i++ )
  {
    CHECK( ctuList[i] > ctuList[i-1]+1, "CTU missing in slice map");
    CHECK( ctuList[i] == ctuList[i-1],  "CTU duplicated in slice map");
  }
}

#endif
APS::APS()
: m_APSId(0)
, m_temporalId( 0 )
#if JVET_N0278_FIXES
, m_layerId( 0 )
#endif
{
}

APS::~APS()
{
}

#if JVET_O1159_SCALABILITY
ReferencePictureList::ReferencePictureList( const bool interLayerPicPresentFlag )
#else
ReferencePictureList::ReferencePictureList()
#endif
  : m_numberOfShorttermPictures(0)
  , m_numberOfLongtermPictures(0)
  , m_numberOfActivePictures(MAX_INT)
  , m_ltrp_in_slice_header_flag(0)
#if JVET_O1159_SCALABILITY
  , m_interLayerPresentFlag( interLayerPicPresentFlag )
  , m_numberOfInterLayerPictures( 0 )
#endif
{
  ::memset(m_isLongtermRefPic, 0, sizeof(m_isLongtermRefPic));
  ::memset(m_refPicIdentifier, 0, sizeof(m_refPicIdentifier));
  ::memset(m_POC, 0, sizeof(m_POC));
#if JVET_O1159_SCALABILITY
  ::memset( m_isInterLayerRefPic, 0, sizeof( m_isInterLayerRefPic ) );
  ::memset( m_interLayerRefPicIdx, 0, sizeof( m_interLayerRefPicIdx ) );
#endif
}

ReferencePictureList::~ReferencePictureList()
{
}

#if JVET_O1159_SCALABILITY
void ReferencePictureList::setRefPicIdentifier( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx )
#else
void ReferencePictureList::setRefPicIdentifier(int idx, int identifier, bool isLongterm)
#endif
{
  m_refPicIdentifier[idx] = identifier;
  m_isLongtermRefPic[idx] = isLongterm;

  m_deltaPocMSBPresentFlag[idx] = false;
  m_deltaPOCMSBCycleLT[idx] = 0;

#if JVET_O1159_SCALABILITY
  m_isInterLayerRefPic[idx] = isInterLayerRefPic;
  m_interLayerRefPicIdx[idx] = interLayerIdx;
#endif
}

int ReferencePictureList::getRefPicIdentifier(int idx) const
{
  return m_refPicIdentifier[idx];
}


bool ReferencePictureList::isRefPicLongterm(int idx) const
{
  return m_isLongtermRefPic[idx];
}

void ReferencePictureList::setNumberOfShorttermPictures(int numberOfStrp)
{
  m_numberOfShorttermPictures = numberOfStrp;
}

int ReferencePictureList::getNumberOfShorttermPictures() const
{
  return m_numberOfShorttermPictures;
}

void ReferencePictureList::setNumberOfLongtermPictures(int numberOfLtrp)
{
  m_numberOfLongtermPictures = numberOfLtrp;
}

int ReferencePictureList::getNumberOfLongtermPictures() const
{
  return m_numberOfLongtermPictures;
}

void ReferencePictureList::setPOC(int idx, int POC)
{
  m_POC[idx] = POC;
}

int ReferencePictureList::getPOC(int idx) const
{
  return m_POC[idx];
}

void ReferencePictureList::setNumberOfActivePictures(int numberActive)
{
  m_numberOfActivePictures = numberActive;
}

int ReferencePictureList::getNumberOfActivePictures() const
{
  return m_numberOfActivePictures;
}

void ReferencePictureList::printRefPicInfo() const
{
  //DTRACE(g_trace_ctx, D_RPSINFO, "RefPics = { ");
  printf("RefPics = { ");
  int numRefPic = getNumberOfShorttermPictures() + getNumberOfLongtermPictures();
  for (int ii = 0; ii < numRefPic; ii++)
  {
    //DTRACE(g_trace_ctx, D_RPSINFO, "%d%s ", m_refPicIdentifier[ii], (m_isLongtermRefPic[ii] == 1) ? "[LT]" : "[ST]");
    printf("%d%s ", m_refPicIdentifier[ii], (m_isLongtermRefPic[ii] == 1) ? "[LT]" : "[ST]");
  }
  //DTRACE(g_trace_ctx, D_RPSINFO, "}\n");
  printf("}\n");
}

ScalingList::ScalingList()
{
#if JVET_P0365_SCALING_MATRIX_LFNST
  m_disableScalingMatrixForLfnstBlks = true;
#endif
#if JVET_P01034_PRED_1D_SCALING_LIST
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    m_scalingListCoef[scalingListId].resize(matrixSize*matrixSize);
  }
#else
  for(uint32_t sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      m_scalingListCoef[sizeId][listId].resize(std::min<int>(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]));
    }
  }
#endif
}

/** set default quantization matrix to array
*/
void ScalingList::setDefaultScalingList()
{
#if JVET_P01034_PRED_1D_SCALING_LIST
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    processDefaultMatrix(scalingListId);
  }
#else
  for(uint32_t sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(uint32_t listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      processDefaultMatrix(sizeId, listId);
    }
  }
#endif
}
/** check if use default quantization matrix
 * \returns true if the scaling list is not equal to the default quantization matrix
*/
bool ScalingList::isNotDefaultScalingList()
{
  bool isAllDefault = true;
#if JVET_P01034_PRED_1D_SCALING_LIST
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    if (scalingListId < SCALING_LIST_1D_START_16x16)
    {
      if (::memcmp(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId), sizeof(int) * matrixSize * matrixSize))
      {
        isAllDefault = false;
        break;
      }
    }
    else
    {
      if ((::memcmp(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId), sizeof(int) * MAX_MATRIX_COEF_NUM)) || (getScalingListDC(scalingListId) != 16))
      {
        isAllDefault = false;
        break;
      }
    }
    if (!isAllDefault) break;
  }
#else
  for ( uint32_t sizeId = SCALING_LIST_2x2; sizeId <= SCALING_LIST_64x64; sizeId++)
  {
    for(uint32_t listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      if (((sizeId == SCALING_LIST_64x64) && (listId % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0))
          || ((sizeId == SCALING_LIST_2x2) && (listId % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0)))
      {
        continue;
      }
      if (sizeId < SCALING_LIST_16x16)
      {
        if (::memcmp(getScalingListAddress(sizeId, listId), getScalingListDefaultAddress(sizeId, listId), sizeof(int) * (int)g_scalingListSize[sizeId]))
        {
          isAllDefault = false;
          break;
        }
      }
      else
      {
        if ((::memcmp(getScalingListAddress(sizeId, listId), getScalingListDefaultAddress(sizeId, listId), sizeof(int) * MAX_MATRIX_COEF_NUM)) || (getScalingListDC(sizeId, listId) != 16))
        {
          isAllDefault = false;
          break;
        }
      }
    }
    if (!isAllDefault) break;
  }
#endif

  return !isAllDefault;
}

/** get scaling matrix from RefMatrixID
 * \param sizeId    size index
 * \param listId    index of input matrix
 * \param refListId index of reference matrix
 */
#if JVET_P01034_PRED_1D_SCALING_LIST
int ScalingList::lengthUvlc(int uiCode)
{
  if (uiCode < 0) printf("Error UVLC! \n");

  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}
int ScalingList::lengthSvlc(int uiCode)
{
  uint32_t uiCode2 = uint32_t(uiCode <= 0 ? (-uiCode) << 1 : (uiCode << 1) - 1);
  int uiLength = 1;
  int uiTemp = ++uiCode2;

  CHECK(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}
void ScalingList::codePredScalingList(int* scalingList, const int* scalingListPred, int scalingListDC, int scalingListPredDC, int scalingListId, int& bitsCost) //sizeId, listId is current to-be-coded matrix idx
{
  int deltaValue = 0;
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  int coefNum = matrixSize*matrixSize;
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)][gp_sizeIdxInfo->idxFrom(matrixSize)];
  int nextCoef = 0;

  int8_t data;
  const int *src = scalingList;
  const int *srcPred = scalingListPred;
  if (scalingListDC!=-1 && scalingListPredDC!=-1)
  {
    bitsCost += lengthSvlc((int8_t)(scalingListDC - scalingListPredDC - nextCoef));
    nextCoef =  scalingListDC - scalingListPredDC;
  }
  else if ((scalingListDC != -1 && scalingListPredDC == -1))
  {
    bitsCost += lengthSvlc((int8_t)(scalingListDC - srcPred[scan[0].idx] - nextCoef));
    nextCoef =  scalingListDC - srcPred[scan[0].idx];
  }
  else if ((scalingListDC == -1 && scalingListPredDC == -1))
  {
  }
  else
  {
    printf("Predictor DC mismatch! \n");
  }
  for (int i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
      continue;
    deltaValue = (src[scan[i].idx] - srcPred[scan[i].idx]);
    data = (int8_t)(deltaValue - nextCoef);
    nextCoef = deltaValue;

    bitsCost += lengthSvlc(data);
  }
}
void ScalingList::codeScalingList(int* scalingList, int scalingListDC, int scalingListId, int& bitsCost) //sizeId, listId is current to-be-coded matrix idx
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  int coefNum = matrixSize * matrixSize;
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)][gp_sizeIdxInfo->idxFrom(matrixSize)];
  int nextCoef = SCALING_LIST_START_VALUE;
  int8_t data;
  const int *src = scalingList;

  if (scalingListId >= SCALING_LIST_1D_START_16x16)
  {
    bitsCost += lengthSvlc(int8_t(getScalingListDC(scalingListId) - nextCoef));
    nextCoef = getScalingListDC(scalingListId);
  }

  for (int i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
      continue;
    data = int8_t(src[scan[i].idx] - nextCoef);
    nextCoef = src[scan[i].idx];

    bitsCost += lengthSvlc(data);
  }
}
void ScalingList::CheckBestPredScalingList(int scalingListId, int predListId, int& BitsCount)
{
  //check previously coded matrix as a predictor, code "lengthUvlc" function
  int *scalingList = getScalingListAddress(scalingListId);
  const int *scalingListPred = (scalingListId == predListId) ? ((predListId < SCALING_LIST_1D_START_8x8) ? g_quantTSDefault4x4 : g_quantIntraDefault8x8) : getScalingListAddress(predListId);
  int scalingListDC = (scalingListId >= SCALING_LIST_1D_START_16x16) ? getScalingListDC(scalingListId) : -1;
  int scalingListPredDC = (predListId >= SCALING_LIST_1D_START_16x16) ? ((scalingListId == predListId) ? 16 : getScalingListDC(predListId)) : -1;

  int bitsCost = 0;
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  int predMatrixSize = (predListId < SCALING_LIST_1D_START_4x4) ? 2 : (predListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;

  if (matrixSize != predMatrixSize) printf("Predictor size mismatch! \n");

  bitsCost = 2 + lengthUvlc(scalingListId - predListId);
  //copy-flag + predictor-mode-flag + deltaListId
  codePredScalingList(scalingList, scalingListPred, scalingListDC, scalingListPredDC, scalingListId, bitsCost);
  BitsCount = bitsCost;
}
void ScalingList::processRefMatrix(uint32_t scalinListId, uint32_t refListId)
{
  int matrixSize = (scalinListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalinListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  ::memcpy(getScalingListAddress(scalinListId), ((scalinListId == refListId) ? getScalingListDefaultAddress(refListId) : getScalingListAddress(refListId)), sizeof(int)*matrixSize*matrixSize);
}
void ScalingList::checkPredMode(uint32_t scalingListId)
{
  int bestBitsCount = MAX_INT;
  int BitsCount = 2;
  setScalingListPreditorModeFlag(scalingListId, false);
  codeScalingList(getScalingListAddress(scalingListId), ((scalingListId >= SCALING_LIST_1D_START_16x16) ? getScalingListDC(scalingListId) : -1), scalingListId, BitsCount);
  bestBitsCount = BitsCount;

  for (int predListIdx = (int)scalingListId; predListIdx >= 0; predListIdx--)
  {

    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    int predMatrixSize = (predListIdx < SCALING_LIST_1D_START_4x4) ? 2 : (predListIdx < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    if (((scalingListId == SCALING_LIST_1D_START_2x2 || scalingListId == SCALING_LIST_1D_START_4x4 || scalingListId == SCALING_LIST_1D_START_8x8) && predListIdx != (int)scalingListId) || matrixSize != predMatrixSize)
      continue;
    const int* refScalingList = (scalingListId == predListIdx) ? getScalingListDefaultAddress(predListIdx) : getScalingListAddress(predListIdx);
    const int refDC = (predListIdx < SCALING_LIST_1D_START_16x16) ? refScalingList[0] : (scalingListId == predListIdx) ? 16 : getScalingListDC(predListIdx);
    if (!::memcmp(getScalingListAddress(scalingListId), refScalingList, sizeof(int)*matrixSize*matrixSize) // check value of matrix
      // check DC value
      && (scalingListId < SCALING_LIST_1D_START_16x16 || getScalingListDC(scalingListId) == refDC))
    {
      //copy mode
      setRefMatrixId(scalingListId, predListIdx);
      setScalingListCopyModeFlag(scalingListId, true);
      setScalingListPreditorModeFlag(scalingListId, false);
      return;
    }
    else
    {
      //predictor mode
      //use previously coded matrix as a predictor
      CheckBestPredScalingList(scalingListId, predListIdx, BitsCount);
      if (BitsCount < bestBitsCount)
      {
        bestBitsCount = BitsCount;
        setScalingListCopyModeFlag(scalingListId, false);
        setScalingListPreditorModeFlag(scalingListId, true);
        setRefMatrixId(scalingListId, predListIdx);
      }
    }
  }
  setScalingListCopyModeFlag(scalingListId, false);
}
#else
void ScalingList::processRefMatrix(uint32_t sizeId, uint32_t listId, uint32_t refListId)
{
  ::memcpy(getScalingListAddress(sizeId, listId), ((listId == refListId) ? getScalingListDefaultAddress(sizeId, refListId) : getScalingListAddress(sizeId, refListId)), sizeof(int)*std::min(MAX_MATRIX_COEF_NUM, (int)g_scalingListSize[sizeId]));
}
void ScalingList::checkPredMode(uint32_t sizeId, uint32_t listId)
{
  for (int predListIdx = (int)listId; predListIdx >= 0; predListIdx--)
  {
    if ((sizeId == SCALING_LIST_64x64 && ((listId % 3) != 0 || (predListIdx % 3) != 0)) || (sizeId == SCALING_LIST_2x2 && ((listId % 3) == 0 || (predListIdx % 3) == 0)))
      continue;
    if( !::memcmp(getScalingListAddress(sizeId,listId),((listId == predListIdx) ?
      getScalingListDefaultAddress(sizeId, predListIdx): getScalingListAddress(sizeId, predListIdx)),sizeof(int)*std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId])) // check value of matrix
      && ((sizeId < SCALING_LIST_16x16) || listId == predListIdx ? getScalingListDefaultAddress(sizeId, predListIdx)[0] == getScalingListDC(sizeId, predListIdx) : (getScalingListDC(sizeId, listId) == getScalingListDC(sizeId, predListIdx)))) // check DC value
    {
      setRefMatrixId(sizeId, listId, predListIdx);
      setScalingListPredModeFlag(sizeId, listId, false);
      return;
    }
  }
  setScalingListPredModeFlag(sizeId, listId, true);
}
#endif

static void outputScalingListHelp(std::ostream &os)
{
  os << "The scaling list file specifies all matrices and their DC values; none can be missing,\n"
         "but their order is arbitrary.\n\n"
         "The matrices are specified by:\n"
         "<matrix name><unchecked data>\n"
         "  <value>,<value>,<value>,....\n\n"
         "  Line-feeds can be added arbitrarily between values, and the number of values needs to be\n"
         "  at least the number of entries for the matrix (superfluous entries are ignored).\n"
         "  The <unchecked data> is text on the same line as the matrix that is not checked\n"
         "  except to ensure that the matrix name token is unique. It is recommended that it is ' ='\n"
         "  The values in the matrices are the absolute values (0-255), not the delta values as\n"
         "  exchanged between the encoder and decoder\n\n"
         "The DC values (for matrix sizes larger than 8x8) are specified by:\n"
         "<matrix name>_DC<unchecked data>\n"
         "  <value>\n";

  os << "The permitted matrix names are:\n";
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)
  {
    for (uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if (!(((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0)) || ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0))))
      {
        os << "  " << MatrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}

void ScalingList::outputScalingLists(std::ostream &os) const
{
#if JVET_P01034_PRED_1D_SCALING_LIST
  int scalingListId = 0;
#endif
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)
  {
    const uint32_t size = (sizeIdc == 1) ? 2 : ((sizeIdc == 2) ? 4 : 8);
    for(uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
#if JVET_P01034_PRED_1D_SCALING_LIST
      if (!((sizeIdc== SCALING_LIST_64x64 && listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0) || (sizeIdc == SCALING_LIST_2x2 && listIdc < 4)))
#else
      if (!(((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0)) || ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0))))
#endif
      {
#if JVET_P01034_PRED_1D_SCALING_LIST
        const int *src = getScalingListAddress(scalingListId);
#else
        const int *src = getScalingListAddress(sizeIdc, listIdc);
#endif
        os << (MatrixType[sizeIdc][listIdc]) << " =\n  ";
        for(uint32_t y=0; y<size; y++)
        {
          for(uint32_t x=0; x<size; x++, src++)
          {
            os << std::setw(3) << (*src) << ", ";
          }
          os << (y+1<size?"\n  ":"\n");
        }
        if(sizeIdc > SCALING_LIST_8x8)
        {
#if JVET_P01034_PRED_1D_SCALING_LIST
          os << MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw(3) << getScalingListDC(scalingListId) << "\n";
#else
          os << MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw(3) << getScalingListDC(sizeIdc, listIdc) << "\n";
#endif
        }
        os << "\n";
#if JVET_P01034_PRED_1D_SCALING_LIST
        scalingListId++;
#endif
      }
    }
  }
}

bool ScalingList::xParseScalingList(const std::string &fileName)
{
  static const int LINE_SIZE=1024;
  FILE *fp = NULL;
  char line[LINE_SIZE];

  if (fileName.empty())
  {
    msg( ERROR, "Error: no scaling list file specified. Help on scaling lists being output\n");
    outputScalingListHelp(std::cout);
    std::cout << "\n\nExample scaling list file using default values:\n\n";
    outputScalingLists(std::cout);
    return true;
  }
  else if ((fp = fopen(fileName.c_str(),"r")) == (FILE*)NULL)
  {
    msg( ERROR, "Error: cannot open scaling list file %s for reading\n", fileName.c_str());
    return true;
  }

#if JVET_P01034_PRED_1D_SCALING_LIST
  int scalingListId = 0;
#endif
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)//2x2-128x128
  {
    const uint32_t size = std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeIdc]);

    for(uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
#if !JVET_P01034_PRED_1D_SCALING_LIST
      int * const src = getScalingListAddress(sizeIdc, listIdc);
#endif

#if JVET_P01034_PRED_1D_SCALING_LIST
      if ((sizeIdc == SCALING_LIST_64x64 && listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0) || (sizeIdc == SCALING_LIST_2x2 && listIdc < 4))
#else
      if (((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0)) || ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0)))
#endif
      {
        continue;
      }
      else
      {
#if JVET_P01034_PRED_1D_SCALING_LIST
        int * const src = getScalingListAddress(scalingListId);
#endif
        {
          fseek(fp, 0, SEEK_SET);
          bool bFound=false;
          while ((!feof(fp)) && (!bFound))
          {
            char *ret = fgets(line, LINE_SIZE, fp);
            char *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType[sizeIdc][listIdc]);
            // This could be a match against the DC string as well, so verify it isn't
            if (findNamePosition!= NULL && (MatrixType_DC[sizeIdc][listIdc]==NULL || strstr(line, MatrixType_DC[sizeIdc][listIdc])==NULL))
            {
              bFound=true;
            }
          }
          if (!bFound)
          {
            msg( ERROR, "Error: cannot find Matrix %s from scaling list file %s\n", MatrixType[sizeIdc][listIdc], fileName.c_str());
            return true;

          }
        }
        for (uint32_t i=0; i<size; i++)
        {
          int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            msg( ERROR, "Error: cannot read value #%d for Matrix %s from scaling list file %s at file position %ld\n", i, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            msg( ERROR, "Error: QMatrix entry #%d of value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", i, data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          src[i] = data;
        }

        //set DC value for default matrix check
#if JVET_P01034_PRED_1D_SCALING_LIST
        setScalingListDC(scalingListId, src[0]);
#else
        setScalingListDC(sizeIdc, listIdc, src[0]);
#endif

        if(sizeIdc > SCALING_LIST_8x8)
        {
          {
            fseek(fp, 0, SEEK_SET);
            bool bFound=false;
            while ((!feof(fp)) && (!bFound))
            {
              char *ret = fgets(line, LINE_SIZE, fp);
              char *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType_DC[sizeIdc][listIdc]);
              if (findNamePosition!= NULL)
              {
                // This won't be a match against the non-DC string.
                bFound=true;
              }
            }
            if (!bFound)
            {
              msg( ERROR, "Error: cannot find DC Matrix %s from scaling list file %s\n", MatrixType_DC[sizeIdc][listIdc], fileName.c_str());
              return true;
            }
          }
          int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            msg( ERROR, "Error: cannot read DC %s from scaling list file %s at file position %ld\n", MatrixType_DC[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            msg( ERROR, "Error: DC value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          //overwrite DC value when size of matrix is larger than 16x16
#if JVET_P01034_PRED_1D_SCALING_LIST
          setScalingListDC(scalingListId, data);
#else
          setScalingListDC(sizeIdc, listIdc, data);
#endif
        }
      }
#if JVET_P01034_PRED_1D_SCALING_LIST
      scalingListId++;
#endif
    }
  }
//  std::cout << "\n\nRead scaling lists of:\n\n";
//  outputScalingLists(std::cout);

  fclose(fp);
  return false;
}


/** get default address of quantization matrix
 * \param sizeId size index
 * \param listId list index
 * \returns pointer of quantization matrix
 */
#if JVET_P01034_PRED_1D_SCALING_LIST
const int* ScalingList::getScalingListDefaultAddress(uint32_t scalingListId)
#else
const int* ScalingList::getScalingListDefaultAddress(uint32_t sizeId, uint32_t listId)
#endif
{
  const int *src = 0;
#if JVET_P01034_PRED_1D_SCALING_LIST
  int sizeId = (scalingListId < SCALING_LIST_1D_START_8x8) ? 2 : 3;
  switch (sizeId)
#else
  switch(sizeId)
#endif
  {
    case SCALING_LIST_1x1:
    case SCALING_LIST_2x2:
    case SCALING_LIST_4x4:
      src = g_quantTSDefault4x4;
      break;
    case SCALING_LIST_8x8:
    case SCALING_LIST_16x16:
    case SCALING_LIST_32x32:
    case SCALING_LIST_64x64:
    case SCALING_LIST_128x128:
#if JVET_P01034_PRED_1D_SCALING_LIST
      src = g_quantInterDefault8x8;
#else
      src = (listId < (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES)) ? g_quantIntraDefault8x8 : g_quantInterDefault8x8;
#endif
      break;
    default:
      THROW( "Invalid scaling list" );
      src = NULL;
      break;
  }
  return src;
}

/** process of default matrix
 * \param sizeId size index
 * \param listId index of input matrix
 */
#if JVET_P01034_PRED_1D_SCALING_LIST
void ScalingList::processDefaultMatrix(uint32_t scalingListId)
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  ::memcpy(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId), sizeof(int)*matrixSize*matrixSize);
  setScalingListDC(scalingListId, SCALING_LIST_DC);
}

/** check DC value of matrix for default matrix signaling
 */
void ScalingList::checkDcOfMatrix()
{
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    //check default matrix?
    if (getScalingListDC(scalingListId) == 0)
    {
      processDefaultMatrix(scalingListId);
    }
  }
}
#else
void ScalingList::processDefaultMatrix(uint32_t sizeId, uint32_t listId)
{
  ::memcpy(getScalingListAddress(sizeId, listId),getScalingListDefaultAddress(sizeId,listId),sizeof(int)*std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]));
  setScalingListDC(sizeId,listId,SCALING_LIST_DC);
}

/** check DC value of matrix for default matrix signaling
 */
void ScalingList::checkDcOfMatrix()
{
  for(uint32_t sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      //check default matrix?
      if(getScalingListDC(sizeId,listId) == 0)
      {
        processDefaultMatrix(sizeId, listId);
      }
    }
  }
}
#endif

ParameterSetManager::ParameterSetManager()
: m_spsMap(MAX_NUM_SPS)
, m_ppsMap(MAX_NUM_PPS)
, m_apsMap(MAX_NUM_APS * MAX_NUM_APS_TYPE)
, m_dpsMap(MAX_NUM_DPS)
#if JVET_P0205_VPS_ID_0
, m_vpsMap(MAX_NUM_VPS)
#endif
, m_activeDPSId(-1)
, m_activeSPSId(-1)
#if JVET_P0205_VPS_ID_0
, m_activeVPSId(-1)
#endif
{
}


ParameterSetManager::~ParameterSetManager()
{
}

//! activate a SPS from a active parameter sets SEI message
//! \returns true, if activation is successful
//bool ParameterSetManager::activateSPSWithSEI(int spsId)
//{
//  SPS *sps = m_spsMap.getPS(spsId);
//  if (sps)
//  {
//    int vpsId = sps->getVPSId();
//    VPS *vps = m_vpsMap.getPS(vpsId);
//    if (vps)
//    {
//      m_activeVPS = *(vps);
//      m_activeSPS = *(sps);
//      return true;
//    }
//    else
//    {
//     msg( WARNING, "Warning: tried to activate SPS using an Active parameter sets SEI message. Referenced VPS does not exist.");
//    }
//  }
//  else
//  {
//    msg( WARNING, "Warning: tried to activate non-existing SPS using an Active parameter sets SEI message.");
//  }
//  return false;
//}

//! activate a PPS and depending on isIDR parameter also SPS
//! \returns true, if activation is successful
bool ParameterSetManager::activatePPS(int ppsId, bool isIRAP)
{
  PPS *pps = m_ppsMap.getPS(ppsId);
  if (pps)
  {
    int spsId = pps->getSPSId();
    if (!isIRAP && (spsId != m_activeSPSId ))
    {
      msg( WARNING, "Warning: tried to activate PPS referring to a inactive SPS at non-IDR.");
    }
    else
    {
      SPS *sps = m_spsMap.getPS(spsId);
      if (sps)
      {
        int dpsId = sps->getDecodingParameterSetId();
        if ((m_activeDPSId!=-1) && (dpsId != m_activeDPSId ))
        {
          msg( WARNING, "Warning: tried to activate DPS with different ID than the currently active DPS. This should not happen within the same bitstream!");
        }
        else
        {
          if (dpsId != 0)
          {
            DPS *dps =m_dpsMap.getPS(dpsId);
            if (dps)
            {
              m_activeDPSId = dpsId;
              m_dpsMap.setActive(dpsId);
            }
            else
            {
              msg( WARNING, "Warning: tried to activate PPS that refers to a non-existing DPS.");
            }
          }
          else
          {
            // set zero as active DPS ID (special reserved value, no actual DPS)
            m_activeDPSId = dpsId;
            m_dpsMap.setActive(dpsId);
          }
        }

#if JVET_P0205_VPS_ID_0
        int vpsId = sps->getVPSId();
        if(vpsId != 0)
        {
          VPS *vps = m_vpsMap.getPS(vpsId);
          if(vps)
          {
            m_activeVPSId = vpsId;
            m_vpsMap.setActive(vpsId);
          }  
          else
          {
            msg( WARNING, "Warning: tried to activate PPS that refers to non-existing VPS." );
          }
        }
        else
        {
          //No actual VPS
          m_activeVPSId = -1;
          m_vpsMap.clear();
        }
#endif

          m_spsMap.clear();
          m_spsMap.setActive(spsId);
        m_activeSPSId = spsId;
        m_ppsMap.clear();
        m_ppsMap.setActive(ppsId);
        return true;
      }
      else
      {
        msg( WARNING, "Warning: tried to activate a PPS that refers to a non-existing SPS.");
      }
    }
  }
  else
  {
    msg( WARNING, "Warning: tried to activate non-existing PPS.");
  }

  // Failed to activate if reach here.
  m_activeSPSId=-1;
  m_activeDPSId=-1;
  return false;
}

bool ParameterSetManager::activateAPS(int apsId, int apsType)
{
  APS *aps = m_apsMap.getPS(apsId + (MAX_NUM_APS * apsType));
  if (aps)
  {
    m_apsMap.setActive(apsId + (MAX_NUM_APS * apsType));
    return true;
  }
  else
  {
    msg(WARNING, "Warning: tried to activate non-existing APS.");
  }
  return false;
}

template <>
void ParameterSetMap<APS>::setID(APS* parameterSet, const int psId)
{
  parameterSet->setAPSId(psId);
}
template <>
void ParameterSetMap<PPS>::setID(PPS* parameterSet, const int psId)
{
  parameterSet->setPPSId(psId);
}

template <>
void ParameterSetMap<SPS>::setID(SPS* parameterSet, const int psId)
{
  parameterSet->setSPSId(psId);
}

ProfileTierLevel::ProfileTierLevel()
  : m_tierFlag        (Level::MAIN)
  , m_profileIdc      (Profile::NONE)
  , m_numSubProfile(0)
  , m_subProfileIdc(0)
  , m_levelIdc        (Level::NONE)
{
  ::memset(m_subLayerLevelPresentFlag,   0, sizeof(m_subLayerLevelPresentFlag  ));
  ::memset(m_subLayerLevelIdc, Level::NONE, sizeof(m_subLayerLevelIdc          ));
}

void calculateParameterSetChangedFlag(bool &bChanged, const std::vector<uint8_t> *pOldData, const std::vector<uint8_t> *pNewData)
{
  if (!bChanged)
  {
    if ((pOldData==0 && pNewData!=0) || (pOldData!=0 && pNewData==0))
    {
      bChanged=true;
    }
    else if (pOldData!=0 && pNewData!=0)
    {
      // compare the two
      if (pOldData->size() != pNewData->size())
      {
        bChanged=true;
      }
      else
      {
        const uint8_t *pNewDataArray=&(*pNewData)[0];
        const uint8_t *pOldDataArray=&(*pOldData)[0];
        if (memcmp(pOldDataArray, pNewDataArray, pOldData->size()))
        {
          bChanged=true;
        }
      }
    }
  }
}

//! \}

uint32_t PreCalcValues::getValIdx( const Slice &slice, const ChannelType chType ) const
{
  return slice.isIntra() ? ( ISingleTree ? 0 : ( chType << 1 ) ) : 1;
}

uint32_t PreCalcValues::getMaxBtDepth( const Slice &slice, const ChannelType chType ) const
{
#if JVET_P1006_PICTURE_HEADER
  if ( slice.getPicHeader()->getSplitConsOverrideFlag() )    
    return slice.getPicHeader()->getMaxMTTHierarchyDepth( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
#else
  if ( slice.getSplitConsOverrideFlag() )
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMaxMTTHierarchyDepth() : slice.getMaxMTTHierarchyDepthIChroma();
#endif
  else
  return maxBtDepth[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMinBtSize( const Slice &slice, const ChannelType chType ) const
{
  return minBtSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMaxBtSize( const Slice &slice, const ChannelType chType ) const
{
#if JVET_P1006_PICTURE_HEADER
  if (slice.getPicHeader()->getSplitConsOverrideFlag())
    return slice.getPicHeader()->getMaxBTSize( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
#else
  if (slice.getSplitConsOverrideFlag())
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMaxBTSize() : slice.getMaxBTSizeIChroma();
#endif
  else
    return maxBtSize[getValIdx(slice, chType)];
}

uint32_t PreCalcValues::getMinTtSize( const Slice &slice, const ChannelType chType ) const
{
  return minTtSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMaxTtSize( const Slice &slice, const ChannelType chType ) const
{
#if JVET_P1006_PICTURE_HEADER
  if (slice.getPicHeader()->getSplitConsOverrideFlag())
    return slice.getPicHeader()->getMaxTTSize( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
#else
  if ( slice.getSplitConsOverrideFlag() )
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMaxTTSize() : slice.getMaxTTSizeIChroma();
#endif
  else
  return maxTtSize[getValIdx( slice, chType )];
}
uint32_t PreCalcValues::getMinQtSize( const Slice &slice, const ChannelType chType ) const
{
#if JVET_P1006_PICTURE_HEADER
  if (slice.getPicHeader()->getSplitConsOverrideFlag())
    return slice.getPicHeader()->getMinQTSize( slice.getSliceType(), ISingleTree ? CHANNEL_TYPE_LUMA : chType);
#else
  if ( slice.getSplitConsOverrideFlag() )
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMinQTSize() : slice.getMinQTSizeIChroma();
#endif
  else
  return minQtSize[getValIdx( slice, chType )];
}

#if JVET_P1006_PICTURE_HEADER
void Slice::scaleRefPicList( Picture *scaledRefPic[ ], PicHeader *picHeader, APS** apss, APS* lmcsAps, APS* scalingListAps, const bool isDecoder )
#else
void Slice::scaleRefPicList( Picture *scaledRefPic[ ], APS** apss, APS* lmcsAps, APS* scalingListAps, const bool isDecoder )
#endif
{
  int i;
  const SPS* sps = getSPS();
  const PPS* pps = getPPS();

#if JVET_P0206_TMVP_flags
  bool refPicIsSameRes = false;
#endif
   
  // this is needed for IBC
  m_pcPic->unscaledPic = m_pcPic;

  if( m_eSliceType == I_SLICE )
  {
    return;
  }

  freeScaledRefPicList( scaledRefPic );

  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      // if rescaling is needed, otherwise just reuse the original picture pointer; it is needed for motion field, otherwise motion field requires a copy as well
      // reference resampling for the whole picture is not applied at decoder

      int xScale, yScale;
      CU::getRprScaling( sps, pps, m_apcRefPicList[refList][rIdx], xScale, yScale );
      m_scalingRatio[refList][rIdx] = std::pair<int, int>( xScale, yScale );

#if JVET_P0206_TMVP_flags
#if JVET_P0590_SCALING_WINDOW
      if( m_scalingRatio[refList][rIdx] == SCALE_1X && pps->getPicWidthInLumaSamples() == m_apcRefPicList[refList][rIdx]->getPicWidthInLumaSamples() && pps->getPicHeightInLumaSamples() == m_apcRefPicList[refList][rIdx]->getPicHeightInLumaSamples() )
#else
      if( m_scalingRatio[refList][rIdx] == SCALE_1X )
#endif
      {
        refPicIsSameRes = true;
      }
#endif

      if( m_scalingRatio[refList][rIdx] == SCALE_1X || isDecoder )
      {
        m_scaledRefPicList[refList][rIdx] = m_apcRefPicList[refList][rIdx];
      }
      else
      {
        int poc = m_apcRefPicList[refList][rIdx]->getPOC();
        // check whether the reference picture has already been scaled
        for( i = 0; i < MAX_NUM_REF; i++ )
        {
          if( scaledRefPic[i] != nullptr && scaledRefPic[i]->poc == poc )
          {
            break;
          }
        }

        if( i == MAX_NUM_REF )
        {
          int j;
          // search for unused Picture structure in scaledRefPic
          for( j = 0; j < MAX_NUM_REF; j++ )
          {
            if( scaledRefPic[j] == nullptr )
            {
              break;
            }
          }

          CHECK( j >= MAX_NUM_REF, "scaledRefPic can not hold all reference pictures!" );

          if( j >= MAX_NUM_REF )
          {
            j = 0;
          }

          if( scaledRefPic[j] == nullptr )
          {
            scaledRefPic[j] = new Picture;

            scaledRefPic[j]->setBorderExtension( false );
            scaledRefPic[j]->reconstructed = false;
            scaledRefPic[j]->referenced = true;

#if JVET_P1006_PICTURE_HEADER
#if JVET_O1159_SCALABILITY
            scaledRefPic[j]->finalInit( m_pcPic->cs->vps, *sps, *pps, picHeader, apss, lmcsAps, scalingListAps );
#else
            scaledRefPic[j]->finalInit( *sps, *pps, picHeader, apss, lmcsAps, scalingListAps );
#endif
#else
            scaledRefPic[ j ]->finalInit( *sps, *pps, apss, lmcsAps, scalingListAps );
#endif

            scaledRefPic[j]->poc = -1;

#if JVET_N0278_FIXES
            scaledRefPic[j]->create( sps->getChromaFormatIdc(), Size( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples() ), sps->getMaxCUWidth(), sps->getMaxCUWidth() + 16, isDecoder, m_pcPic->layerId );
#else
            scaledRefPic[j]->create( sps->getChromaFormatIdc(), Size( pps->getPicWidthInLumaSamples(), pps->getPicHeightInLumaSamples() ), sps->getMaxCUWidth(), sps->getMaxCUWidth() + 16, isDecoder );
#endif
          }

          scaledRefPic[j]->poc = poc;
          scaledRefPic[j]->longTerm = m_apcRefPicList[refList][rIdx]->longTerm;

          // rescale the reference picture
          const bool downsampling = m_apcRefPicList[refList][rIdx]->getRecoBuf().Y().width >= scaledRefPic[j]->getRecoBuf().Y().width && m_apcRefPicList[refList][rIdx]->getRecoBuf().Y().height >= scaledRefPic[j]->getRecoBuf().Y().height;
#if JVET_P0590_SCALING_WINDOW
#if JVET_P0592_CHROMA_PHASE
          Picture::rescalePicture( m_scalingRatio[refList][rIdx], 
                                   m_apcRefPicList[refList][rIdx]->getRecoBuf(), m_apcRefPicList[refList][rIdx]->slices[0]->getPPS()->getScalingWindow(), 
                                   scaledRefPic[j]->getRecoBuf(), pps->getScalingWindow(), 
                                   sps->getChromaFormatIdc(), sps->getBitDepths(), true, downsampling,
                                   sps->getHorCollocatedChromaFlag(), sps->getVerCollocatedChromaFlag() );
#else
          Picture::rescalePicture( m_scalingRatio[refList][rIdx], m_apcRefPicList[refList][rIdx]->getRecoBuf(), m_apcRefPicList[refList][rIdx]->slices[0]->getPPS()->getScalingWindow(), scaledRefPic[j]->getRecoBuf(), pps->getScalingWindow(), sps->getChromaFormatIdc(), sps->getBitDepths(), true, downsampling );
#endif
#elif JVET_P0592_CHROMA_PHASE
          Picture::rescalePicture( m_scalingRatio[refList][rIdx], m_apcRefPicList[refList][rIdx]->getRecoBuf(),
                                   m_apcRefPicList[refList][rIdx]->slices[0]->getPPS()->getConformanceWindow(),
                                   scaledRefPic[j]->getRecoBuf(), pps->getConformanceWindow(),
                                   sps->getChromaFormatIdc(), sps->getBitDepths(), true, downsampling,
                                   sps->getHorCollocatedChromaFlag(), sps->getVerCollocatedChromaFlag() );
#else
          Picture::rescalePicture( m_apcRefPicList[refList][rIdx]->getRecoBuf(), m_apcRefPicList[refList][rIdx]->slices[0]->getPPS()->getConformanceWindow(), scaledRefPic[j]->getRecoBuf(), pps->getConformanceWindow(), sps->getChromaFormatIdc(), sps->getBitDepths(), true, downsampling );
#endif
          scaledRefPic[j]->extendPicBorder();

          m_scaledRefPicList[refList][rIdx] = scaledRefPic[j];
        }
        else
        {
          m_scaledRefPicList[refList][rIdx] = scaledRefPic[i];
        }
      }
    }
  }

  // make the scaled reference picture list as the default reference picture list
  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      m_savedRefPicList[refList][rIdx] = m_apcRefPicList[refList][rIdx];
      m_apcRefPicList[refList][rIdx] = m_scaledRefPicList[refList][rIdx];

      // allow the access of the unscaled version in xPredInterBlk()
      m_apcRefPicList[refList][rIdx]->unscaledPic = m_savedRefPicList[refList][rIdx];
    }
  }
  
#if JVET_P0206_TMVP_flags
  //Make sure that TMVP is disabled when there are no reference pictures with the same resolution
  if(!refPicIsSameRes)
  {
#if JVET_P1006_PICTURE_HEADER
    CHECK(getPicHeader()->getEnableTMVPFlag() != 0, "TMVP cannot be enabled in pictures that have no reference pictures with the same resolution")
#else
    CHECK(m_enableTMVPFlag != 0, "TMVP cannot be enabled in slices that have no reference pictures with the same resolution")
#endif
  }
#endif
}

void Slice::freeScaledRefPicList( Picture *scaledRefPic[] )
{
  if( m_eSliceType == I_SLICE )
  {
    return;
  }
  for( int i = 0; i < MAX_NUM_REF; i++ )
  {
    if( scaledRefPic[i] != nullptr )
    {
      scaledRefPic[i]->destroy();
      scaledRefPic[i] = nullptr;
    }
  }
}

bool Slice::checkRPR()
{
  const PPS* pps = getPPS();

  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {

    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      if( m_scaledRefPicList[refList][rIdx]->cs->pcv->lumaWidth != pps->getPicWidthInLumaSamples() || m_scaledRefPicList[refList][rIdx]->cs->pcv->lumaHeight != pps->getPicHeightInLumaSamples() )
      {
        return true;
      }
    }
  }

  return false;
}

#if ENABLE_TRACING
void xTraceVPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Video Parameter Set     ===========\n" );
}

void xTraceDPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Decoding Parameter Set     ===========\n" );
}

void xTraceSPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Sequence Parameter Set  ===========\n" );
}

void xTracePPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Parameter Set  ===========\n" );
}

void xTraceAPSHeader()
{
  DTRACE(g_trace_ctx, D_HEADER, "=========== Adaptation Parameter Set  ===========\n");
}

#if JVET_P1006_PICTURE_HEADER
void xTracePictureHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Header ===========\n" );
}

#endif
void xTraceSliceHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Slice ===========\n" );
}

void xTraceAccessUnitDelimiter()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n" );
}
#endif
