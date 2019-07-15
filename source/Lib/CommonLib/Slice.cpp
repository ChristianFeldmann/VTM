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
: m_iPPSId                        ( -1 )
, m_PicOutputFlag                 ( true )
, m_iPOC                          ( 0 )
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
, m_depQuantEnabledFlag           ( false )
, m_signDataHidingEnabledFlag     ( false )
, m_bCheckLDC                     ( false )
, m_biDirPred                    ( false )
, m_iSliceQpDelta                 ( 0 )
, m_iDepth                        ( 0 )
#if HEVC_VPS
, m_pcVPS                         ( NULL )
#endif
, m_dps                           ( nullptr )
, m_pcSPS                         ( NULL )
, m_pcPPS                         ( NULL )
, m_pcPic                         ( NULL )
, m_colFromL0Flag                 ( true )
, m_noOutputPriorPicsFlag         ( false )
, m_noRaslOutputFlag              ( false )
, m_handleCraAsCvsStartFlag            ( false )
, m_colRefIdx                     ( 0 )
, m_maxNumMergeCand               ( 0 )
, m_maxNumAffineMergeCand         ( 0 )
, m_maxNumTriangleCand            ( 0 )
, m_disFracMMVD                   ( false )
, m_uiTLayer                      ( 0 )
, m_bTLayerSwitchingFlag          ( false )
, m_sliceMode                     ( NO_SLICES )
, m_sliceArgument                 ( 0 )
, m_sliceCurStartCtuTsAddr        ( 0 )
, m_sliceCurEndCtuTsAddr          ( 0 )
, m_independentSliceIdx           ( 0 )
, m_nextSlice                     ( false )
, m_sliceBits                     ( 0 )
, m_bFinalized                    ( false )
, m_bTestWeightPred               ( false )
, m_bTestWeightBiPred             ( false )
, m_substreamSizes                ( )
, m_cabacInitFlag                 ( false )
, m_bLMvdL1Zero                   ( false )
, m_LFCrossSliceBoundaryFlag      ( false )
, m_enableTMVPFlag                ( true )
, m_encCABACTableIdx              (I_SLICE)
, m_iProcessingStartTime          ( 0 )
, m_dProcessingTime               ( 0 )
, m_splitConsOverrideFlag         ( false )
, m_uiMinQTSize                   ( 0 )
, m_uiMaxBTDepth                  ( 0 )
, m_uiMaxTTSize                   ( 0 )
, m_uiMinQTSizeIChroma            ( 0 )
, m_uiMaxBTDepthIChroma           ( 0 )
, m_uiMaxBTSizeIChroma            ( 0 )
, m_uiMaxTTSizeIChroma            ( 0 )
, m_uiMaxBTSize                   ( 0 )
, m_lmcsApsId                    ( -1 )
, m_lmcsAps                      (nullptr)
, m_tileGroupLmcsEnabledFlag     (false)
, m_tileGroupLmcsChromaResidualScaleFlag (false)
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
}

Slice::~Slice()
{
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

  m_maxNumMergeCand = MRG_MAX_NUM_CANDS;
  m_maxNumAffineMergeCand = AFFINE_MRG_MAX_NUM_CANDS;

  m_bFinalized=false;

  m_disFracMMVD          = false;
  m_substreamSizes.clear();
  m_cabacInitFlag        = false;
  m_enableTMVPFlag       = true;
}

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

Picture* Slice::xGetRefPic (PicList& rcListPic, int poc)
{
  PicList::iterator  iterPic = rcListPic.begin();
  Picture*           pcPic   = *(iterPic);

  while ( iterPic != rcListPic.end() )
  {
    if(pcPic->getPOC() == poc)
    {
      break;
    }
    iterPic++;
    pcPic = *(iterPic);
  }
  return  pcPic;
}


Picture* Slice::xGetLongTermRefPic( PicList& rcListPic, int poc, bool pocHasMsb)
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
    if (pcPic && pcPic->getPOC()!=this->getPOC() && pcPic->referenced)
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
  for (int ii = 0; ii < numOfActiveRef; ii++)
  {
    if (!m_pRPL0->isRefPicLongterm(ii))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC() - m_pRPL0->getRefPicIdentifier(ii));
      pcRefPic->longTerm = false;
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = m_pRPL0->getRefPicIdentifier(ii) & pocMask;
      ltrpPoc += m_localRPL0.getDeltaPocMSBPresentFlag(ii) ? (pocMask + 1) * m_localRPL0.getDeltaPocMSBCycleLT(ii) : 0;
      pcRefPic = xGetLongTermRefPic(rcListPic, ltrpPoc, m_localRPL0.getDeltaPocMSBPresentFlag(ii));
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
    if (!m_pRPL1->isRefPicLongterm(ii))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC() - m_pRPL1->getRefPicIdentifier(ii));
      pcRefPic->longTerm = false;
    }
    else
    {
      int pocBits = getSPS()->getBitsForPOC();
      int pocMask = (1 << pocBits) - 1;
      int ltrpPoc = m_pRPL1->getRefPicIdentifier(ii) & pocMask;
      ltrpPoc += m_localRPL1.getDeltaPocMSBPresentFlag(ii) ? (pocMask + 1) * m_localRPL1.getDeltaPocMSBCycleLT(ii) : 0;
      pcRefPic = xGetLongTermRefPic(rcListPic, ltrpPoc, m_localRPL1.getDeltaPocMSBPresentFlag(ii));
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
        CHECK(xGetLongTermRefPic(rcListPic, pRPL0->getRefPicIdentifier(i), pRPL0->getDeltaPocMSBPresentFlag(i))->getPOC() < pocCRA, "Invalid state");
      }
    }
    numRefPic = pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures();
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL1->isRefPicLongterm(i))
      {
        CHECK(getPOC() - pRPL1->getRefPicIdentifier(i) < pocCRA, "Invalid state");
      }
      else
      {
        CHECK(xGetLongTermRefPic(rcListPic, pRPL1->getRefPicIdentifier(i), pRPL1->getDeltaPocMSBPresentFlag(i))->getPOC() < pocCRA, "Invalid state");
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

  m_sliceMode                     = pSrc->m_sliceMode;
  m_sliceArgument                 = pSrc->m_sliceArgument;
  m_sliceCurStartCtuTsAddr        = pSrc->m_sliceCurStartCtuTsAddr;
  m_sliceCurEndCtuTsAddr          = pSrc->m_sliceCurEndCtuTsAddr;
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
  memcpy(m_alfApss, pSrc->m_alfApss, sizeof(m_alfApss)); // this might be quite unsafe
  memcpy( m_tileGroupAlfEnabledFlag, pSrc->m_tileGroupAlfEnabledFlag, sizeof(m_tileGroupAlfEnabledFlag));
  m_tileGroupNumAps               = pSrc->m_tileGroupNumAps;
  m_tileGroupLumaApsId            = pSrc->m_tileGroupLumaApsId;
  m_tileGroupChromaApsId          = pSrc->m_tileGroupChromaApsId;
  m_disableSATDForRd              = pSrc->m_disableSATDForRd;

  m_bLMvdL1Zero                   = pSrc->m_bLMvdL1Zero;
  m_LFCrossSliceBoundaryFlag      = pSrc->m_LFCrossSliceBoundaryFlag;
  m_enableTMVPFlag                = pSrc->m_enableTMVPFlag;
  m_maxNumMergeCand               = pSrc->m_maxNumMergeCand;
  m_maxNumAffineMergeCand         = pSrc->m_maxNumAffineMergeCand;
  m_maxNumTriangleCand            = pSrc->m_maxNumTriangleCand;
  m_disFracMMVD                   = pSrc->m_disFracMMVD;
  if( cpyAlmostAll ) m_encCABACTableIdx  = pSrc->m_encCABACTableIdx;
  m_splitConsOverrideFlag         = pSrc->m_splitConsOverrideFlag;
  m_uiMinQTSize                   = pSrc->m_uiMinQTSize;
  m_uiMaxBTDepth                  = pSrc->m_uiMaxBTDepth;
  m_uiMaxTTSize                   = pSrc->m_uiMaxTTSize;
  m_uiMinQTSizeIChroma            = pSrc->m_uiMinQTSizeIChroma;
  m_uiMaxBTDepthIChroma           = pSrc->m_uiMaxBTDepthIChroma;
  m_uiMaxBTSizeIChroma            = pSrc->m_uiMaxBTSizeIChroma;
  m_uiMaxTTSizeIChroma            = pSrc->m_uiMaxTTSizeIChroma;
  m_uiMaxBTSize                   = pSrc->m_uiMaxBTSize;

  m_depQuantEnabledFlag           = pSrc->m_depQuantEnabledFlag;
  m_signDataHidingEnabledFlag     = pSrc->m_signDataHidingEnabledFlag;

  m_tileGroupLmcsEnabledFlag = pSrc->m_tileGroupLmcsEnabledFlag;
  m_tileGroupLmcsChromaResidualScaleFlag = pSrc->m_tileGroupLmcsChromaResidualScaleFlag;
  m_lmcsAps = pSrc->m_lmcsAps;
  m_lmcsApsId = pSrc->m_lmcsApsId;
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
    if( pcPic->referenced &&  pcPic->usedByCurr && pcPic->poc != getPOC())
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
    if(pcSlice->getPicOutputFlag() == 1 && !this->getNoOutputPriorPicsFlag())
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
    if(pcSlice->getPicOutputFlag() == 1)
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
    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL ||
        nalUnitType == NAL_UNIT_CODED_SLICE_RADL )
      {
        if(pcSlice->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC())
        {
          // rpcPic is a picture that preceded the leading in decoding order since it exist in the DPB
          // rpcPic would violate the constraint if it was a trailing picture
          CHECK( pcPic->poc > this->getAssociatedIRAPPOC(), "Invalid POC");
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
void Slice::applyReferencePictureListBasedMarking(PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1) const
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
    for (i = 0; isNeedToCheck && !isReference && i<pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures(); i++)
    {
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
    for (i = 0; isNeedToCheck && !isReference && i<pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures(); i++)
    {
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
    if (pcPic->poc != this->getPOC() && isReference == 0)
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

  int numberOfPictures = pRPL->getNumberOfLongtermPictures() + pRPL->getNumberOfShorttermPictures();
  //Check long term ref pics
  for (int ii = 0; pRPL->getNumberOfLongtermPictures() > 0 && ii < numberOfPictures; ii++)
  {
    if (!pRPL->isRefPicLongterm(ii))
      continue;

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
      if (!pRPL0->isRefPicLongterm(ii) && rpcPic->referenced && rpcPic->getPOC() == this->getPOC() - pRPL0->getRefPicIdentifier(ii))
      {
        isAvailable = true;
        break;
      }
      else if (pRPL0->isRefPicLongterm(ii) && rpcPic->referenced && (rpcPic->getPOC() & (pocCycle - 1)) == pRPL0->getRefPicIdentifier(ii))
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
      if (!pRPL1->isRefPicLongterm(ii) && rpcPic->referenced && rpcPic->getPOC() == this->getPOC() - pRPL1->getRefPicIdentifier(ii))
      {
        isAvailable = true;
        break;
      }
      else if (pRPL1->isRefPicLongterm(ii) && rpcPic->referenced && (rpcPic->getPOC() & (pocCycle - 1)) == pRPL1->getRefPicIdentifier(ii))
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
        numOfSTRPL1 = numOfSTRPL1 + ((pRPL0->isRefPicLongterm(ii)) ? 0 : 1);
        numOfLTRPL1 = numOfLTRPL1 + ((pRPL0->isRefPicLongterm(ii)) ? 1 : 0);

        numOfNeedToFill--;
      }
    }
  }
  pLocalRPL1->setNumberOfLongtermPictures(numOfLTRPL1);
  pLocalRPL1->setNumberOfShorttermPictures(numOfSTRPL1);
  pLocalRPL1->setNumberOfActivePictures((isDisallowMixedRefPic) ? (numOfLTRPL1 + numOfSTRPL1) : (((numOfLTRPL1 + numOfSTRPL1) < pRPL1->getNumberOfActivePictures()) ? (numOfLTRPL1 + numOfSTRPL1) : pRPL1->getNumberOfActivePictures()));
  this->setRPL1idx(-1);
  this->setRPL1(pLocalRPL1);
}

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

#if HEVC_VPS
// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
VPS::VPS()
: m_VPSId                     (  0)
, m_uiMaxTLayers              (  1)
, m_uiMaxLayers               (  1)
, m_bTemporalIdNestingFlag    (false)
, m_numHrdParameters          (  0)
, m_maxNuhReservedZeroLayerId (  0)
, m_hrdParameters             ()
, m_hrdOpSetIdx               ()
, m_cprmsPresentFlag          ()
{

  for( int i = 0; i < MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_uiMaxLatencyIncrease[i] = 0;
  }
}

VPS::~VPS()
{
}
#else
// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
VPS::VPS()
  : m_VPSId(0)
  , m_uiMaxLayers(1)
  , m_vpsExtensionFlag()
{
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_vpsIncludedLayerId[i] = 0;
  }
}

VPS::~VPS()
{
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
, m_affineAmvrEnabledFlag     ( false )
, m_DMVR                      ( false )
, m_MMVD                      ( false )
, m_SBT                       ( false )
, m_MaxSbtSize                ( 32 )
#if INCLUDE_ISP_CFG_FLAG
, m_ISP                       ( false )
#endif
#if HEVC_VPS
, m_VPSId                     (  0)
#endif
, m_chromaFormatIdc           (CHROMA_420)
, m_uiMaxTLayers              (  1)
// Structure
, m_picWidthInLumaSamples     (352)
, m_picHeightInLumaSamples    (288)
, m_log2MinCodingBlockSize    (  0)
, m_log2DiffMaxMinCodingBlockSize(0)
, m_CTUSize(0)
, m_minQT{ 0, 0, 0 }
, m_maxBTDepth{ MAX_BT_DEPTH, MAX_BT_DEPTH_INTER, MAX_BT_DEPTH_C }
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
, m_pcmEnabledFlag            (false)
, m_pcmLog2MaxSize            (  5)
, m_uiPCMLog2MinSize          (  7)
, m_bPCMFilterDisableFlag     (false)
, m_sbtmvpEnabledFlag         (false)
, m_bdofEnabledFlag           (false)
, m_fpelMmvdEnabledFlag       ( false )
, m_uiBitsForPOC              (  8)
, m_numLongTermRefPicSPS      (  0)
#if MAX_TB_SIZE_SIGNALLING
, m_log2MaxTbSize             (  6)
#endif
, m_saoEnabledFlag            (false)
, m_bTemporalIdNestingFlag    (false)
, m_scalingListEnabledFlag    (false)
, m_vuiParametersPresentFlag  (false)
, m_vuiParameters             ()
, m_wrapAroundEnabledFlag     (false)
, m_wrapAroundOffset          (  0)
, m_IBCFlag                   (  0)
, m_lumaReshapeEnable         (false)
, m_AMVREnabledFlag                       ( false )
, m_LMChroma                  ( false )
, m_cclmCollocatedChromaFlag  ( false )
, m_IntraMTS                  ( false )
, m_InterMTS                  ( false )
, m_LFNST                     ( false )
, m_Affine                    ( false )
, m_AffineType                ( false )
, m_MHIntra                   ( false )
, m_Triangle                  ( false )
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
, m_LadfEnabled               ( false )
, m_LadfNumIntervals          ( 0 )
, m_LadfQpOffset              { 0 }
, m_LadfIntervalLowerBound    { 0 }
#endif
, m_MIP                       ( false )
{
  for(int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_bitDepths.recon[ch] = 8;
    m_pcmBitDepths[ch] = 8;
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
}

SPS::~SPS()
{
}

void  SPS::createRPLList0(int numRPL)
{
  m_RPLList0.destroy();
  m_RPLList0.create(numRPL);
  m_numRPL0 = numRPL;
  m_rpl1IdxPresentFlag = (m_numRPL0 != m_numRPL1) ? true : false;
}
void  SPS::createRPLList1(int numRPL)
{
  m_RPLList1.destroy();
  m_RPLList1.create(numRPL);
  m_numRPL1 = numRPL;

  m_rpl1IdxPresentFlag = (m_numRPL0 != m_numRPL1) ? true : false;
}


const int SPS::m_winUnitX[]={1,2,2,1};
const int SPS::m_winUnitY[]={1,2,1,1};

PPSRExt::PPSRExt()
: m_log2MaxTransformSkipBlockSize      (2)
, m_crossComponentPredictionEnabledFlag(false)
, m_cuChromaQpOffsetSubdiv             (0)
, m_chromaQpOffsetListLen              (0)
// m_ChromaQpAdjTableIncludingNullEntry initialized below
// m_log2SaoOffsetScale initialized below
{
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset = 0; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here and never subsequently changed.
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
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
, m_bConstrainedIntraPred            (false)
, m_bSliceChromaQpFlag               (false)
, m_cuQpDeltaSubdiv                  (0)
, m_chromaCbQpOffset                 (0)
, m_chromaCrQpOffset                 (0)
, m_chromaCbCrQpOffset               (0)
, m_numRefIdxL0DefaultActive         (1)
, m_numRefIdxL1DefaultActive         (1)
, m_rpl1IdxPresentFlag               (false)
, m_TransquantBypassEnabledFlag      (false)
, m_useTransformSkip                 (false)
, m_entropyCodingSyncEnabledFlag     (false)
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
, m_cabacInitPresentFlag             (false)
, m_sliceHeaderExtensionPresentFlag  (false)
, m_loopFilterAcrossSlicesEnabledFlag(false)
, m_listsModificationPresentFlag     (0)
, m_numExtraSliceHeaderBits          (0)
, m_loopFilterAcrossVirtualBoundariesDisabledFlag(false)
, m_numVerVirtualBoundaries          (0)
, m_numHorVirtualBoundaries          (0)
, m_ppsRangeExtension                ()
, pcv                                (NULL)
{
  ::memset(m_virtualBoundariesPosX, 0, sizeof(m_virtualBoundariesPosX));
  ::memset(m_virtualBoundariesPosY, 0, sizeof(m_virtualBoundariesPosY));
}

PPS::~PPS()
{
  delete pcv;
}

APS::APS()
: m_APSId(0)
{
}

APS::~APS()
{
}


ReferencePictureList::ReferencePictureList()
  : m_numberOfShorttermPictures(0)
  , m_numberOfLongtermPictures(0)
  , m_numberOfActivePictures(MAX_INT)
{
  ::memset(m_isLongtermRefPic, 0, sizeof(m_isLongtermRefPic));
  ::memset(m_refPicIdentifier, 0, sizeof(m_refPicIdentifier));
  ::memset(m_POC, 0, sizeof(m_POC));
}

ReferencePictureList::~ReferencePictureList()
{
}

void ReferencePictureList::setRefPicIdentifier(int idx, int identifier, bool isLongterm)
{
  m_refPicIdentifier[idx] = identifier;
  m_isLongtermRefPic[idx] = isLongterm;

  m_deltaPocMSBPresentFlag[idx] = false;
  m_deltaPOCMSBCycleLT[idx] = 0;
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
  for(uint32_t sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      m_scalingListCoef[sizeId][listId].resize(std::min<int>(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]));
    }
  }
}

/** set default quantization matrix to array
*/
void ScalingList::setDefaultScalingList()
{
  for(uint32_t sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(uint32_t listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      processDefaultMatrix(sizeId, listId);
    }
  }
}
/** check if use default quantization matrix
 * \returns true if use default quantization matrix in all size
*/
bool ScalingList::checkDefaultScalingList()
{
  bool isAllDefault = true;
  for ( uint32_t sizeId = SCALING_LIST_2x2; sizeId <= SCALING_LIST_64x64; sizeId++)
  {
    for(uint32_t listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      if (((sizeId == SCALING_LIST_64x64) && (listId % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) != 0))
       || ((sizeId == SCALING_LIST_2x2) && (listId % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) == 0)))
      {
        continue;
      }
      if( !::memcmp(getScalingListAddress(sizeId,listId), getScalingListDefaultAddress(sizeId, listId),sizeof(int)*std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId])) // check value of matrix
         && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == 16))) // check DC value
      {
        isAllDefault = false;
        break;
      }
    }
  }

  return !isAllDefault;
}

/** get scaling matrix from RefMatrixID
 * \param sizeId    size index
 * \param listId    index of input matrix
 * \param refListId index of reference matrix
 */
void ScalingList::processRefMatrix( uint32_t sizeId, uint32_t listId , uint32_t refListId )
{
  ::memcpy(getScalingListAddress(sizeId, listId),((listId == refListId)? getScalingListDefaultAddress(sizeId, refListId): getScalingListAddress(sizeId, refListId)),sizeof(int)*std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]));
}

void ScalingList::checkPredMode(uint32_t sizeId, uint32_t listId)
{
  for (int predListIdx = (int)listId; predListIdx >= 0; predListIdx--)
  {
    if ((sizeId == SCALING_LIST_64x64 && (listId % 3) != 0) || (sizeId == SCALING_LIST_2x2 && (listId % 3) == 0))
      continue;
    if( !::memcmp(getScalingListAddress(sizeId,listId),((listId == predListIdx) ?
      getScalingListDefaultAddress(sizeId, predListIdx): getScalingListAddress(sizeId, predListIdx)),sizeof(int)*std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId])) // check value of matrix
      && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == getScalingListDC(sizeId,predListIdx)))) // check DC value
    {
      setRefMatrixId(sizeId, listId, predListIdx);
      setScalingListPredModeFlag(sizeId, listId, false);
      return;
    }
  }
  setScalingListPredModeFlag(sizeId, listId, true);
}

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
      if (!(((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) != 0)) || ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) == 0))))
      {
        os << "  " << MatrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}

void ScalingList::outputScalingLists(std::ostream &os) const
{
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)
  {
    const uint32_t size = (sizeIdc == 1) ? 2 : ((sizeIdc == 2) ? 4 : 8);
    for(uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if (!(((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) != 0)) || ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) == 0))))
      {
        const int *src = getScalingListAddress(sizeIdc, listIdc);
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
          os << MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw(3) << getScalingListDC(sizeIdc, listIdc) << "\n";
        }
        os << "\n";
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

  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)//2x2-128x128
  {
    const uint32_t size = std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeIdc]);

    for(uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      int * const src = getScalingListAddress(sizeIdc, listIdc);

      if (((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) != 0)) || ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1)) == 0)))
      {
        continue;
      }
      else
      {
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
        setScalingListDC(sizeIdc,listIdc,src[0]);

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
          setScalingListDC(sizeIdc,listIdc,data);
        }
      }
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
const int* ScalingList::getScalingListDefaultAddress(uint32_t sizeId, uint32_t listId)
{
  const int *src = 0;
  switch(sizeId)
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
      src = (listId < (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES - 1))) ? g_quantIntraDefault8x8 : g_quantInterDefault8x8;
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

ParameterSetManager::ParameterSetManager()
#if HEVC_VPS
: m_vpsMap(MAX_NUM_VPS)
, m_spsMap(MAX_NUM_SPS)
#else
: m_spsMap(MAX_NUM_SPS)
#endif
, m_ppsMap(MAX_NUM_PPS)
, m_apsMap(MAX_NUM_APS * MAX_NUM_APS_TYPE)
, m_dpsMap(MAX_NUM_DPS)
#if HEVC_VPS
, m_activeVPSId(-1)
#endif
, m_activeDPSId(-1)
, m_activeSPSId(-1)
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

#if HEVC_VPS
//! activate a PPS and depending on isIDR parameter also SPS and VPS
#else
//! activate a PPS and depending on isIDR parameter also SPS
#endif
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

#if HEVC_VPS
        int vpsId = sps->getVPSId();
        if (!isIRAP && (vpsId != m_activeVPSId ))
        {
          msg( WARNING, "Warning: tried to activate PPS referring to a inactive VPS at non-IDR.");
        }
        else
        {
#endif
          m_spsMap.clear();
          m_spsMap.setActive(spsId);
#if HEVC_VPS
          VPS *vps =m_vpsMap.getPS(vpsId);
          if (vps)
          {
            m_activeVPSId = vpsId;
            m_activeSPSId = spsId;
            m_ppsMap.setActive(ppsId);
            return true;
          }
          else
          {
            msg( WARNING, "Warning: tried to activate PPS that refers to a non-existing VPS.");
          }
        }
#else
        m_activeSPSId = spsId;
        m_ppsMap.clear();
        m_ppsMap.setActive(ppsId);
        return true;
#endif
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
#if HEVC_VPS
  m_activeVPSId=-1;
#endif
  m_activeDPSId=-1;
  return false;
}

bool ParameterSetManager::activateAPS(int apsId, int apsType)
{
  APS *aps = m_apsMap.getPS((apsId << NUM_APS_TYPE_LEN) + apsType);
  if (aps)
  {
    m_apsMap.setActive((apsId << NUM_APS_TYPE_LEN) + apsType);
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
  , m_subProfileIdc   (0)
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
  if ( slice.getSplitConsOverrideFlag() )
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMaxBTDepth() : slice.getMaxBTDepthIChroma();
  else
  return maxBtDepth[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMinBtSize( const Slice &slice, const ChannelType chType ) const
{
  return minBtSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMaxBtSize( const Slice &slice, const ChannelType chType ) const
{
  if (slice.getSplitConsOverrideFlag())
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMaxBTSize() : slice.getMaxBTSizeIChroma();
  else
    return maxBtSize[getValIdx(slice, chType)];
}

uint32_t PreCalcValues::getMinTtSize( const Slice &slice, const ChannelType chType ) const
{
  return minTtSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMaxTtSize( const Slice &slice, const ChannelType chType ) const
{
  if ( slice.getSplitConsOverrideFlag() )
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMaxTTSize() : slice.getMaxTTSizeIChroma();
  else
  return maxTtSize[getValIdx( slice, chType )];
}
uint32_t PreCalcValues::getMinQtSize( const Slice &slice, const ChannelType chType ) const
{
  if ( slice.getSplitConsOverrideFlag() )
    return (!slice.isIntra() || isLuma(chType) || ISingleTree) ? slice.getMinQTSize() : slice.getMinQTSizeIChroma();
  else
  return minQtSize[getValIdx( slice, chType )];
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

void xTraceSliceHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Slice ===========\n" );
}

void xTraceAccessUnitDelimiter()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n" );
}
#endif
