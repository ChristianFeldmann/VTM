/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ITU/ISO/IEC
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

/** \file     Slice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __SLICE__
#define __SLICE__

#include <cstring>
#include <list>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "Rom.h"
#include "ChromaFormat.h"
#include "Common.h"
#include "HRD.h"
#include <unordered_map>
#include "AlfParameters.h"

//! \ingroup CommonLib
//! \{
#include "CommonLib/MotionInfo.h"
struct MotionInfo;


struct Picture;
class Pic;
class TrQuant;
// ====================================================================================================================
// Constants
// ====================================================================================================================
class PreCalcValues;
static const uint32_t REF_PIC_LIST_NUM_IDX=32;

typedef std::list<Picture*> PicList;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if JVET_Q0814_DPB
struct DpbParameters
{
  int m_maxDecPicBuffering[MAX_TLAYER] = { 0 };
  int m_numReorderPics[MAX_TLAYER] = { 0 };
  int m_maxLatencyIncreasePlus1[MAX_TLAYER] = { 0 };
};
#endif

class ReferencePictureList
{
private:
  int   m_numberOfShorttermPictures;
  int   m_numberOfLongtermPictures;
  int   m_isLongtermRefPic[MAX_NUM_REF_PICS];
  int   m_refPicIdentifier[MAX_NUM_REF_PICS];  //This can be delta POC for STRP or POC LSB for LTRP
  int   m_POC[MAX_NUM_REF_PICS];
  int   m_numberOfActivePictures;
  bool  m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];
  int   m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  bool  m_ltrp_in_slice_header_flag;
  bool  m_interLayerPresentFlag;
  bool  m_isInterLayerRefPic[MAX_NUM_REF_PICS];
  int   m_interLayerRefPicIdx[MAX_NUM_REF_PICS];
  int   m_numberOfInterLayerPictures;

public:
  ReferencePictureList( const bool interLayerPicPresentFlag = false );
  virtual ~ReferencePictureList();

  void    setRefPicIdentifier( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx );
  int     getRefPicIdentifier(int idx) const;
  bool    isRefPicLongterm(int idx) const;

  void    setNumberOfShorttermPictures(int numberOfStrp);
  int     getNumberOfShorttermPictures() const;

  void    setNumberOfLongtermPictures(int numberOfLtrp);
  int     getNumberOfLongtermPictures() const;

  void    setLtrpInSliceHeaderFlag(bool flag) { m_ltrp_in_slice_header_flag = flag; }
  bool    getLtrpInSliceHeaderFlag() const { return m_ltrp_in_slice_header_flag; }

  void    setNumberOfInterLayerPictures( const int numberOfIlrp ) { m_numberOfInterLayerPictures = numberOfIlrp; }
  int     getNumberOfInterLayerPictures() const { return m_numberOfInterLayerPictures; }

  int     getNumRefEntries() const { return m_numberOfShorttermPictures + m_numberOfLongtermPictures + m_numberOfInterLayerPictures; }

  void    setPOC(int idx, int POC);
  int     getPOC(int idx) const;

  void    setNumberOfActivePictures(int numberOfLtrp);
  int     getNumberOfActivePictures() const;

  int     getDeltaPocMSBCycleLT(int i) const { return m_deltaPOCMSBCycleLT[i]; }
  void    setDeltaPocMSBCycleLT(int i, int x) { m_deltaPOCMSBCycleLT[i] = x; }
  bool    getDeltaPocMSBPresentFlag(int i) const { return m_deltaPocMSBPresentFlag[i]; }
  void    setDeltaPocMSBPresentFlag(int i, bool x) { m_deltaPocMSBPresentFlag[i] = x; }

  void    printRefPicInfo() const;

  bool      getInterLayerPresentFlag()                   const { return m_interLayerPresentFlag; }
  void      setInterLayerPresentFlag( bool b )                 { m_interLayerPresentFlag = b; }
  bool      isInterLayerRefPic( int idx )                const { return m_isInterLayerRefPic[idx]; }
  int       getInterLayerRefPicIdx( int idx )            const { return m_interLayerRefPicIdx[idx]; }
  void      setInterLayerRefPicIdx( int idx, int layerIdc )    { m_interLayerRefPicIdx[idx] = layerIdc; }
};

/// Reference Picture List set class
class RPLList
{
private:
  std::vector<ReferencePictureList> m_referencePictureLists;

public:
  RPLList() { }
  virtual                        ~RPLList() { }

  void                           create(int numberOfEntries) { m_referencePictureLists.resize(numberOfEntries); }
  void                           destroy() { }


  ReferencePictureList*          getReferencePictureList(int referencePictureListIdx) { return &m_referencePictureLists[referencePictureListIdx]; }
  const ReferencePictureList*    getReferencePictureList(int referencePictureListIdx) const { return &m_referencePictureLists[referencePictureListIdx]; }

  int                            getNumberOfReferencePictureLists() const { return int(m_referencePictureLists.size()); }
};

/// SCALING_LIST class
class ScalingList
{
public:
             ScalingList();
  virtual    ~ScalingList()                                                 { }
  bool       getDisableScalingMatrixForLfnstBlks() const     { return m_disableScalingMatrixForLfnstBlks;}
  void       setDisableScalingMatrixForLfnstBlks(bool flag)  { m_disableScalingMatrixForLfnstBlks = flag;}
  int*       getScalingListAddress(uint32_t scalingListId)                    { return &(m_scalingListCoef[scalingListId][0]);            } //!< get matrix coefficient
  const int* getScalingListAddress(uint32_t scalingListId) const              { return &(m_scalingListCoef[scalingListId][0]);            } //!< get matrix coefficient
  void       checkPredMode(uint32_t scalingListId);

  void       setRefMatrixId(uint32_t scalingListId, uint32_t u)               { m_refMatrixId[scalingListId] = u;                         } //!< set reference matrix ID
  uint32_t       getRefMatrixId(uint32_t scalingListId) const                 { return m_refMatrixId[scalingListId];                      } //!< get reference matrix ID
  
  static const int* getScalingListDefaultAddress(uint32_t scalinListId);                                                                           //!< get default matrix coefficient
  void       processDefaultMatrix(uint32_t scalinListId);

  void       setScalingListDC(uint32_t scalinListId, uint32_t u)              { m_scalingListDC[scalinListId] = u;                        } //!< set DC value
  int        getScalingListDC(uint32_t scalinListId) const                    { return m_scalingListDC[scalinListId];                     } //!< get DC value

  void       setScalingListCopyModeFlag(uint32_t scalinListId, bool bIsCopy)  { m_scalingListPredModeFlagIsCopy[scalinListId] = bIsCopy;  }
  bool       getScalingListCopyModeFlag(uint32_t scalinListId) const          { return m_scalingListPredModeFlagIsCopy[scalinListId];     } //getScalingListPredModeFlag
  void       processRefMatrix(uint32_t scalingListId, uint32_t refListId);

  int        lengthUvlc(int uiCode);
  int        lengthSvlc(int uiCode);
  void       CheckBestPredScalingList(int scalingListId, int predListIdx, int& BitsCount);
  void       codePredScalingList(int* scalingList, const int* scalingListPred, int scalingListDC, int scalingListPredDC, int scalinListId, int& bitsCost);
  void       codeScalingList(int* scalingList, int scalingListDC, int scalinListId, int& bitsCost);
  void       setScalingListPreditorModeFlag(uint32_t scalingListId, bool bIsPred) { m_scalingListPreditorModeFlag[scalingListId] = bIsPred; }
  bool       getScalingListPreditorModeFlag(uint32_t scalingListId) const { return m_scalingListPreditorModeFlag[scalingListId]; }
#if JVET_Q0505_CHROAM_QM_SIGNALING_400
  bool       getChromaScalingListPresentFlag() const {return m_chromaScalingListPresentFlag;}
  void       setChromaScalingListPresentFlag( bool flag) { m_chromaScalingListPresentFlag = flag;}
  bool       isLumaScalingList( int scalingListId) const;
#endif
  void       checkDcOfMatrix();
  bool       xParseScalingList(const std::string &fileName);
  void       setDefaultScalingList();
  bool       isNotDefaultScalingList();

  bool operator==( const ScalingList& other )
  {
    if (memcmp(m_scalingListPredModeFlagIsCopy, other.m_scalingListPredModeFlagIsCopy, sizeof(m_scalingListPredModeFlagIsCopy)))
    {
      return false;
    }
    if( memcmp( m_scalingListDC, other.m_scalingListDC, sizeof( m_scalingListDC ) ) )
    {
      return false;
    }
    if( memcmp( m_refMatrixId, other.m_refMatrixId, sizeof( m_refMatrixId ) ) )
    {
      return false;
    }
    if( memcmp( m_scalingListCoef, other.m_scalingListCoef, sizeof( m_scalingListCoef ) ) )
    {
      return false;
    }

    return true;
  }

  bool operator!=( const ScalingList& other )
  {
    return !( *this == other );
  }

private:
  void             outputScalingLists(std::ostream &os) const;
  bool             m_disableScalingMatrixForLfnstBlks;
  bool             m_scalingListPredModeFlagIsCopy [30]; //!< reference list index
  int              m_scalingListDC                 [30]; //!< the DC value of the matrix coefficient for 16x16
  uint32_t         m_refMatrixId                   [30]; //!< RefMatrixID
  bool             m_scalingListPreditorModeFlag   [30]; //!< reference list index
  std::vector<int> m_scalingListCoef               [30]; //!< quantization matrix
#if JVET_Q0505_CHROAM_QM_SIGNALING_400
  bool             m_chromaScalingListPresentFlag;
#endif
};

class ConstraintInfo
{
  bool              m_progressiveSourceFlag;
  bool              m_interlacedSourceFlag;
  bool              m_nonPackedConstraintFlag;
#if JVET_Q0114_CONSTRAINT_FLAGS
  bool              m_nonProjectedConstraintFlag;
  bool              m_noResChangeInClvsConstraintFlag;
  bool              m_oneTilePerPicConstraintFlag;
  bool              m_oneSlicePerPicConstraintFlag;
  bool              m_oneSubpicPerPicConstraintFlag;
#endif
  bool              m_frameOnlyConstraintFlag;
  bool              m_intraOnlyConstraintFlag;
  uint32_t          m_maxBitDepthConstraintIdc;
  ChromaFormat      m_maxChromaFormatConstraintIdc;
  bool              m_onePictureOnlyConstraintFlag;
  bool              m_lowerBitRateConstraintFlag;

  bool              m_noQtbttDualTreeIntraConstraintFlag;
  bool              m_noPartitionConstraintsOverrideConstraintFlag;
  bool              m_noSaoConstraintFlag;
  bool              m_noAlfConstraintFlag;
#if JVET_Q0795_CCALF
  bool              m_noCCAlfConstraintFlag;
#endif
  bool              m_noRefWraparoundConstraintFlag;
  bool              m_noTemporalMvpConstraintFlag;
  bool              m_noSbtmvpConstraintFlag;
  bool              m_noAmvrConstraintFlag;
  bool              m_noBdofConstraintFlag;
  bool              m_noDmvrConstraintFlag;
  bool              m_noCclmConstraintFlag;
  bool              m_noMtsConstraintFlag;
  bool              m_noSbtConstraintFlag;
  bool              m_noAffineMotionConstraintFlag;
  bool              m_noBcwConstraintFlag;
  bool              m_noIbcConstraintFlag;
  bool              m_noCiipConstraintFlag;
  bool              m_noFPelMmvdConstraintFlag;
#if !JVET_Q0806
  bool              m_noTriangleConstraintFlag;
#else
  bool              m_noGeoConstraintFlag;
#endif
  bool              m_noLadfConstraintFlag;
  bool              m_noTransformSkipConstraintFlag;
  bool              m_noBDPCMConstraintFlag;
  bool              m_noJointCbCrConstraintFlag;
  bool              m_noQpDeltaConstraintFlag;
  bool              m_noDepQuantConstraintFlag;
  bool              m_noSignDataHidingConstraintFlag;
  bool              m_noTrailConstraintFlag;
  bool              m_noStsaConstraintFlag;
  bool              m_noRaslConstraintFlag;
  bool              m_noRadlConstraintFlag;
  bool              m_noIdrConstraintFlag;
  bool              m_noCraConstraintFlag;
  bool              m_noGdrConstraintFlag;
  bool              m_noApsConstraintFlag;

public:
  ConstraintInfo()
    : m_progressiveSourceFlag (false)
    , m_interlacedSourceFlag (false)
    , m_nonPackedConstraintFlag (false)
#if JVET_Q0114_CONSTRAINT_FLAGS
    , m_nonProjectedConstraintFlag(false)
    , m_noResChangeInClvsConstraintFlag(false)
    , m_oneTilePerPicConstraintFlag(false)
    , m_oneSlicePerPicConstraintFlag(false)
    , m_oneSubpicPerPicConstraintFlag(false)
#endif
    , m_frameOnlyConstraintFlag  (false)
    , m_intraOnlyConstraintFlag  (false)
    , m_maxBitDepthConstraintIdc  (  0)
    , m_maxChromaFormatConstraintIdc(CHROMA_420)
    , m_onePictureOnlyConstraintFlag (false)
    , m_lowerBitRateConstraintFlag (false )
    , m_noQtbttDualTreeIntraConstraintFlag(false)
    , m_noPartitionConstraintsOverrideConstraintFlag(false)
    , m_noSaoConstraintFlag      (false)
    , m_noAlfConstraintFlag      (false)
#if JVET_Q0795_CCALF
    , m_noCCAlfConstraintFlag      (false)
#endif
    , m_noRefWraparoundConstraintFlag(false)
    , m_noTemporalMvpConstraintFlag(false)
    , m_noSbtmvpConstraintFlag   (false)
    , m_noAmvrConstraintFlag     (false)
    , m_noBdofConstraintFlag     (false)
    , m_noDmvrConstraintFlag     (false)
    , m_noCclmConstraintFlag     (false)
    , m_noMtsConstraintFlag      (false)
    , m_noSbtConstraintFlag      (false)
    , m_noAffineMotionConstraintFlag(false)
    , m_noBcwConstraintFlag      (false)
    , m_noIbcConstraintFlag      (false)
    , m_noCiipConstraintFlag  (false)
    , m_noFPelMmvdConstraintFlag (false)
#if !JVET_Q0806
    , m_noTriangleConstraintFlag (false)
#else
    , m_noGeoConstraintFlag      (false)
#endif
    , m_noLadfConstraintFlag     (false)
    , m_noTransformSkipConstraintFlag(false)
    , m_noBDPCMConstraintFlag    (false)
    , m_noJointCbCrConstraintFlag (false)
    , m_noQpDeltaConstraintFlag  (false)
    , m_noDepQuantConstraintFlag (false)
    , m_noSignDataHidingConstraintFlag(false)
    , m_noTrailConstraintFlag (false)
    , m_noStsaConstraintFlag (false)
    , m_noRaslConstraintFlag (false)
    , m_noRadlConstraintFlag (false)
    , m_noIdrConstraintFlag (false)
    , m_noCraConstraintFlag (false)
    , m_noGdrConstraintFlag (false)
    , m_noApsConstraintFlag (false)
  {}

  bool          getProgressiveSourceFlag() const { return m_progressiveSourceFlag; }
  void          setProgressiveSourceFlag(bool b) { m_progressiveSourceFlag = b; }

  bool          getInterlacedSourceFlag() const { return m_interlacedSourceFlag; }
  void          setInterlacedSourceFlag(bool b) { m_interlacedSourceFlag = b; }

  bool          getNonPackedConstraintFlag() const { return m_nonPackedConstraintFlag; }
  void          setNonPackedConstraintFlag(bool b) { m_nonPackedConstraintFlag = b; }

  bool          getFrameOnlyConstraintFlag() const { return m_frameOnlyConstraintFlag; }
  void          setFrameOnlyConstraintFlag(bool b) { m_frameOnlyConstraintFlag = b; }

  uint32_t      getMaxBitDepthConstraintIdc() const { return m_maxBitDepthConstraintIdc; }
  void          setMaxBitDepthConstraintIdc(uint32_t bitDepth) { m_maxBitDepthConstraintIdc = bitDepth; }

  ChromaFormat  getMaxChromaFormatConstraintIdc() const { return m_maxChromaFormatConstraintIdc; }
  void          setMaxChromaFormatConstraintIdc(ChromaFormat fmt) { m_maxChromaFormatConstraintIdc = fmt; }

#if JVET_Q0114_CONSTRAINT_FLAGS
  bool          getNonProjectedConstraintFlag() const { return m_nonProjectedConstraintFlag; }
  void          setNonProjectedConstraintFlag(bool b) { m_nonProjectedConstraintFlag = b; }

  bool          getNoResChangeInClvsConstraintFlag() const { return m_noResChangeInClvsConstraintFlag; }
  void          setNoResChangeInClvsConstraintFlag(bool b) { m_noResChangeInClvsConstraintFlag = b; }

  bool          getOneTilePerPicConstraintFlag() const { return m_oneTilePerPicConstraintFlag; }
  void          setOneTilePerPicConstraintFlag(bool b) { m_oneTilePerPicConstraintFlag = b; }

  bool          getOneSlicePerPicConstraintFlag() const { return m_oneSlicePerPicConstraintFlag; }
  void          setOneSlicePerPicConstraintFlag(bool b) { m_oneSlicePerPicConstraintFlag = b; }

  bool          getOneSubpicPerPicConstraintFlag() const { return m_oneSubpicPerPicConstraintFlag; }
  void          setOneSubpicPerPicConstraintFlag(bool b) { m_oneSubpicPerPicConstraintFlag = b; }
#endif

  bool          getIntraOnlyConstraintFlag() const { return m_intraOnlyConstraintFlag; }
  void          setIntraOnlyConstraintFlag(bool b) { m_intraOnlyConstraintFlag = b; }

  bool          getOnePictureOnlyConstraintFlag() const { return m_onePictureOnlyConstraintFlag; }
  void          setOnePictureOnlyConstraintFlag(bool b) { m_onePictureOnlyConstraintFlag = b; }

  bool          getLowerBitRateConstraintFlag() const { return m_lowerBitRateConstraintFlag; }
  void          setLowerBitRateConstraintFlag(bool b) { m_lowerBitRateConstraintFlag = b; }

  bool          getNoQtbttDualTreeIntraConstraintFlag() const { return m_noQtbttDualTreeIntraConstraintFlag; }
  void          setNoQtbttDualTreeIntraConstraintFlag(bool bVal) { m_noQtbttDualTreeIntraConstraintFlag = bVal; }
  bool          getNoPartitionConstraintsOverrideConstraintFlag() const { return m_noPartitionConstraintsOverrideConstraintFlag; }
  void          setNoPartitionConstraintsOverrideConstraintFlag(bool bVal) { m_noPartitionConstraintsOverrideConstraintFlag = bVal; }
  bool          getNoSaoConstraintFlag() const { return m_noSaoConstraintFlag; }
  void          setNoSaoConstraintFlag(bool bVal) { m_noSaoConstraintFlag = bVal; }
  bool          getNoAlfConstraintFlag() const { return m_noAlfConstraintFlag; }
  void          setNoAlfConstraintFlag(bool bVal) { m_noAlfConstraintFlag = bVal; }
#if JVET_Q0795_CCALF
  bool          getNoCCAlfConstraintFlag() const { return m_noCCAlfConstraintFlag; }
  void          setNoCCAlfConstraintFlag(bool val) { m_noCCAlfConstraintFlag = val; }
#endif
  bool          getNoJointCbCrConstraintFlag() const { return m_noJointCbCrConstraintFlag; }
  void          setNoJointCbCrConstraintFlag(bool bVal) { m_noJointCbCrConstraintFlag = bVal; }
  bool          getNoRefWraparoundConstraintFlag() const { return m_noRefWraparoundConstraintFlag; }
  void          setNoRefWraparoundConstraintFlag(bool bVal) { m_noRefWraparoundConstraintFlag = bVal; }
  bool          getNoTemporalMvpConstraintFlag() const { return m_noTemporalMvpConstraintFlag; }
  void          setNoTemporalMvpConstraintFlag(bool bVal) { m_noTemporalMvpConstraintFlag = bVal; }
  bool          getNoSbtmvpConstraintFlag() const { return m_noSbtmvpConstraintFlag; }
  void          setNoSbtmvpConstraintFlag(bool bVal) { m_noSbtmvpConstraintFlag = bVal; }
  bool          getNoAmvrConstraintFlag() const { return m_noAmvrConstraintFlag; }
  void          setNoAmvrConstraintFlag(bool bVal) { m_noAmvrConstraintFlag = bVal; }
  bool          getNoBdofConstraintFlag() const { return m_noBdofConstraintFlag; }
  void          setNoBdofConstraintFlag(bool bVal) { m_noBdofConstraintFlag = bVal; }
  bool          getNoDmvrConstraintFlag() const { return m_noDmvrConstraintFlag; }
  void          setNoDmvrConstraintFlag(bool bVal) { m_noDmvrConstraintFlag = bVal; }
  bool          getNoCclmConstraintFlag() const { return m_noCclmConstraintFlag; }
  void          setNoCclmConstraintFlag(bool bVal) { m_noCclmConstraintFlag = bVal; }
  bool          getNoMtsConstraintFlag() const { return m_noMtsConstraintFlag; }
  void          setNoMtsConstraintFlag(bool bVal) { m_noMtsConstraintFlag = bVal; }
  bool          getNoSbtConstraintFlag() const { return m_noSbtConstraintFlag; }
  void          setNoSbtConstraintFlag(bool bVal) { m_noSbtConstraintFlag = bVal; }
  bool          getNoAffineMotionConstraintFlag() const { return m_noAffineMotionConstraintFlag; }
  void          setNoAffineMotionConstraintFlag(bool bVal) { m_noAffineMotionConstraintFlag = bVal; }
  bool          getNoBcwConstraintFlag() const { return m_noBcwConstraintFlag; }
  void          setNoBcwConstraintFlag(bool bVal) { m_noBcwConstraintFlag = bVal; }
  bool          getNoIbcConstraintFlag() const { return m_noIbcConstraintFlag; }
  void          setNoIbcConstraintFlag(bool bVal) { m_noIbcConstraintFlag = bVal; }
  bool          getNoCiipConstraintFlag() const { return m_noCiipConstraintFlag; }
  void          setNoCiipConstraintFlag(bool bVal) { m_noCiipConstraintFlag = bVal; }
  bool          getNoFPelMmvdConstraintFlag() const { return m_noFPelMmvdConstraintFlag; }
  void          setNoFPelMmvdConstraintFlag(bool bVal) { m_noFPelMmvdConstraintFlag = bVal; }
#if !JVET_Q0806
  bool          getNoTriangleConstraintFlag() const { return m_noTriangleConstraintFlag; }
  void          setNoTriangleConstraintFlag(bool bVal) { m_noTriangleConstraintFlag = bVal; }
#else
  bool          getNoGeoConstraintFlag() const { return m_noGeoConstraintFlag; }
  void          setNoGeoConstraintFlag(bool bVal) { m_noGeoConstraintFlag = bVal; }
#endif
  bool          getNoLadfConstraintFlag() const { return m_noLadfConstraintFlag; }
  void          setNoLadfConstraintFlag(bool bVal) { m_noLadfConstraintFlag = bVal; }
  bool          getNoTransformSkipConstraintFlag() const { return m_noTransformSkipConstraintFlag; }
  void          setNoTransformSkipConstraintFlag(bool bVal) { m_noTransformSkipConstraintFlag = bVal; }
  bool          getNoBDPCMConstraintFlag() const { return m_noBDPCMConstraintFlag; }
  void          setNoBDPCMConstraintFlag(bool bVal) { m_noBDPCMConstraintFlag = bVal; }
  bool          getNoQpDeltaConstraintFlag() const { return m_noQpDeltaConstraintFlag; }
  void          setNoQpDeltaConstraintFlag(bool bVal) { m_noQpDeltaConstraintFlag = bVal; }
  bool          getNoDepQuantConstraintFlag() const { return m_noDepQuantConstraintFlag; }
  void          setNoDepQuantConstraintFlag(bool bVal) { m_noDepQuantConstraintFlag = bVal; }
  bool          getNoSignDataHidingConstraintFlag() const { return m_noSignDataHidingConstraintFlag; }
  void          setNoSignDataHidingConstraintFlag(bool bVal) { m_noSignDataHidingConstraintFlag = bVal; }
  bool          getNoTrailConstraintFlag() const { return m_noTrailConstraintFlag; }
  void          setNoTrailConstraintFlag(bool bVal) { m_noTrailConstraintFlag = bVal; }
  bool          getNoStsaConstraintFlag() const { return m_noStsaConstraintFlag; }
  void          setNoStsaConstraintFlag(bool bVal) { m_noStsaConstraintFlag = bVal; }
  bool          getNoRaslConstraintFlag() const { return m_noRaslConstraintFlag; }
  void          setNoRaslConstraintFlag(bool bVal) { m_noRaslConstraintFlag = bVal; }
  bool          getNoRadlConstraintFlag() const { return m_noRadlConstraintFlag; }
  void          setNoRadlConstraintFlag(bool bVal) { m_noRadlConstraintFlag = bVal; }
  bool          getNoIdrConstraintFlag() const { return m_noIdrConstraintFlag; }
  void          setNoIdrConstraintFlag(bool bVal) { m_noIdrConstraintFlag = bVal; }
  bool          getNoCraConstraintFlag() const { return m_noCraConstraintFlag; }
  void          setNoCraConstraintFlag(bool bVal) { m_noCraConstraintFlag = bVal; }
  bool          getNoGdrConstraintFlag() const { return m_noGdrConstraintFlag; }
  void          setNoGdrConstraintFlag(bool bVal) { m_noGdrConstraintFlag = bVal; }
  bool          getNoApsConstraintFlag() const { return m_noApsConstraintFlag; }
  void          setNoApsConstraintFlag(bool bVal) { m_noApsConstraintFlag = bVal; }

#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  friend bool             operator == (const ConstraintInfo& op1, const ConstraintInfo& op2);
  friend bool             operator != (const ConstraintInfo& op1, const ConstraintInfo& op2);
#endif
};

class ProfileTierLevel
{
  Level::Tier       m_tierFlag;
  Profile::Name     m_profileIdc;
  uint8_t           m_numSubProfile;
  std::vector<uint32_t>          m_subProfileIdc;
  Level::Name       m_levelIdc;

  ConstraintInfo    m_constraintInfo;
  bool              m_subLayerLevelPresentFlag[MAX_TLAYER - 1];
  Level::Name       m_subLayerLevelIdc[MAX_TLAYER];

public:
                ProfileTierLevel();

  Level::Tier   getTierFlag() const                         { return m_tierFlag;                    }
  void          setTierFlag(Level::Tier x)                  { m_tierFlag = x;                       }

  Profile::Name getProfileIdc() const                       { return m_profileIdc;                  }
  void          setProfileIdc(Profile::Name x)              { m_profileIdc = x;                     }

  uint32_t      getSubProfileIdc(int i) const               { return m_subProfileIdc[i]; }
  void          setSubProfileIdc(int i, uint32_t x)         { m_subProfileIdc[i] = x; }

  uint8_t       getNumSubProfile() const                    { return m_numSubProfile; }
  void          setNumSubProfile(uint8_t x)                 { m_numSubProfile = x; m_subProfileIdc.resize(m_numSubProfile); }

  Level::Name   getLevelIdc() const                         { return m_levelIdc;                    }
  void          setLevelIdc(Level::Name x)                  { m_levelIdc = x;                       }


  ConstraintInfo*         getConstraintInfo()              { return &m_constraintInfo; }
  const ConstraintInfo*   getConstraintInfo() const        { return &m_constraintInfo; }

  bool                    getSubLayerLevelPresentFlag(int i) const     { return m_subLayerLevelPresentFlag[i];   }
  void                    setSubLayerLevelPresentFlag(int i, bool x)   { m_subLayerLevelPresentFlag[i] = x;      }

  Level::Name             getSubLayerLevelIdc(int i) const             { return m_subLayerLevelIdc[i];   }
  void                    setSubLayerLevelIdc(int i, Level::Name x)    { m_subLayerLevelIdc[i] = x;      }
#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  friend bool             operator == (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
  friend bool             operator != (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
#endif
};



class SliceReshapeInfo
{
public:
  bool      sliceReshaperEnableFlag;
  bool      sliceReshaperModelPresentFlag;
  unsigned  enableChromaAdj;
  uint32_t  reshaperModelMinBinIdx;
  uint32_t  reshaperModelMaxBinIdx;
  int       reshaperModelBinCWDelta[PIC_CODE_CW_BINS];
  int       maxNbitsNeededDeltaCW;
  int       chrResScalingOffset;
  void      setUseSliceReshaper(bool b)                                { sliceReshaperEnableFlag = b;            }
  bool      getUseSliceReshaper() const                                { return sliceReshaperEnableFlag;         }
  void      setSliceReshapeModelPresentFlag(bool b)                    { sliceReshaperModelPresentFlag = b;      }
  bool      getSliceReshapeModelPresentFlag() const                    { return   sliceReshaperModelPresentFlag; }
  void      setSliceReshapeChromaAdj(unsigned adj)                     { enableChromaAdj = adj;                  }
  unsigned  getSliceReshapeChromaAdj() const                           { return enableChromaAdj;                 }

  bool operator==( const SliceReshapeInfo& other )
  {
    if( sliceReshaperEnableFlag != other.sliceReshaperEnableFlag )
    {
      return false;
    }
    if( sliceReshaperModelPresentFlag != other.sliceReshaperModelPresentFlag )
    {
      return false;
    }
    if( enableChromaAdj != other.enableChromaAdj )
    {
      return false;
    }
    if( reshaperModelMinBinIdx != other.reshaperModelMinBinIdx )
    {
      return false;
    }
    if( reshaperModelMaxBinIdx != other.reshaperModelMaxBinIdx )
    {
      return false;
    }
    if( maxNbitsNeededDeltaCW != other.maxNbitsNeededDeltaCW )
    {
      return false;
    }
    if (chrResScalingOffset != other.chrResScalingOffset)
    {
      return false;
    }
    if( memcmp( reshaperModelBinCWDelta, other.reshaperModelBinCWDelta, sizeof( reshaperModelBinCWDelta ) ) )
    {
      return false;
    }

    return true;
  }

  bool operator!=( const SliceReshapeInfo& other )
  {
    return !( *this == other );
  }
};

struct ReshapeCW
{
  std::vector<uint32_t> binCW;
  int       updateCtrl;
  int       adpOption;
  uint32_t  initialCW;
  int rspPicSize;
  int rspFps;
  int rspBaseQP;
  int rspTid;
  int rspSliceQP;
  int rspFpsToIp;
};

struct ChromaQpAdj
{
  union
  {
    struct {
      int CbOffset;
      int CrOffset;
      int JointCbCrOffset;
    } comp;
    int offset[3];
  } u;
};
struct ChromaQpMappingTableParams {
  int               m_qpBdOffset;
  bool              m_sameCQPTableForAllChromaFlag;
  int               m_numQpTables;
  int               m_qpTableStartMinus26[MAX_NUM_CQP_MAPPING_TABLES];
  int               m_numPtsInCQPTableMinus1[MAX_NUM_CQP_MAPPING_TABLES];
  std::vector<int>  m_deltaQpInValMinus1[MAX_NUM_CQP_MAPPING_TABLES];
  std::vector<int>  m_deltaQpOutVal[MAX_NUM_CQP_MAPPING_TABLES];

  ChromaQpMappingTableParams()
  {
    m_qpBdOffset = 12;
    m_sameCQPTableForAllChromaFlag = true;
    m_numQpTables = 1;
    m_numPtsInCQPTableMinus1[0] = 0;
    m_qpTableStartMinus26[0] = 0;
    m_deltaQpInValMinus1[0] = { 0 };
    m_deltaQpOutVal[0] = { 0 };
  }

  void      setSameCQPTableForAllChromaFlag(bool b)                             { m_sameCQPTableForAllChromaFlag = b; }
  bool      getSameCQPTableForAllChromaFlag()                             const { return m_sameCQPTableForAllChromaFlag; }
  void      setNumQpTables(int n)                                     { m_numQpTables = n; }
  int       getNumQpTables()                                     const { return m_numQpTables; }
  void      setQpTableStartMinus26(int tableIdx, int n)                         { m_qpTableStartMinus26[tableIdx] = n; }
  int       getQpTableStartMinus26(int tableIdx)                          const { return m_qpTableStartMinus26[tableIdx]; }
  void      setNumPtsInCQPTableMinus1(int tableIdx, int n)                      { m_numPtsInCQPTableMinus1[tableIdx] = n; }
  int       getNumPtsInCQPTableMinus1(int tableIdx)                       const { return m_numPtsInCQPTableMinus1[tableIdx]; }
  void      setDeltaQpInValMinus1(int tableIdx, std::vector<int> &inVals)       { m_deltaQpInValMinus1[tableIdx] = inVals; }
  void      setDeltaQpInValMinus1(int tableIdx, int idx, int n)                 { m_deltaQpInValMinus1[tableIdx][idx] = n; }
  int       getDeltaQpInValMinus1(int tableIdx, int idx)                  const { return m_deltaQpInValMinus1[tableIdx][idx]; }
  void      setDeltaQpOutVal(int tableIdx, std::vector<int> &outVals)           { m_deltaQpOutVal[tableIdx] = outVals; }
  void      setDeltaQpOutVal(int tableIdx, int idx, int n)                      { m_deltaQpOutVal[tableIdx][idx] = n; }
  int       getDeltaQpOutVal(int tableIdx, int idx)                       const { return m_deltaQpOutVal[tableIdx][idx]; }
};
struct ChromaQpMappingTable : ChromaQpMappingTableParams
{
  std::map<int, int> m_chromaQpMappingTables[MAX_NUM_CQP_MAPPING_TABLES];

  int       getMappedChromaQpValue(ComponentID compID, const int qpVal)  const { return m_chromaQpMappingTables[m_sameCQPTableForAllChromaFlag ? 0 : (int)compID - 1].at(qpVal); }
  void      derivedChromaQPMappingTables();
  void      setParams(const ChromaQpMappingTableParams &params, const int qpBdOffset);
};

class SliceMap
{
private:
  uint32_t               m_sliceID;                           //!< slice identifier (slice index for rectangular slices, slice address for raser-scan slices)
  uint32_t               m_numTilesInSlice;                   //!< number of tiles in slice (raster-scan slices only)
  uint32_t               m_numCtuInSlice;                     //!< number of CTUs in the slice
  std::vector<uint32_t>  m_ctuAddrInSlice;                    //!< raster-scan addresses of all the CTUs in the slice

public:
  SliceMap();
  virtual ~SliceMap();

  void                   setSliceID( uint32_t u )             { m_sliceID = u;            }
  uint32_t               getSliceID() const                   { return m_sliceID;         }
  void                   setNumTilesInSlice( uint32_t u )     { m_numTilesInSlice = u;    }
  uint32_t               getNumTilesInSlice() const           { return m_numTilesInSlice; }
  void                   setNumCtuInSlice( uint32_t u )       { m_numCtuInSlice = u;      }
  uint32_t               getNumCtuInSlice() const             { return m_numCtuInSlice;   }
  std::vector<uint32_t>  getCtuAddrList( ) const              { return m_ctuAddrInSlice;  }
  uint32_t               getCtuAddrInSlice( int idx ) const   { CHECK(idx >= m_ctuAddrInSlice.size(), "CTU index exceeds number of CTUs in slice."); return m_ctuAddrInSlice[idx]; }
  void                   pushToCtuAddrInSlice( uint32_t u )   { m_ctuAddrInSlice.push_back(u); m_numCtuInSlice++;}

  void  initSliceMap() 
  {
    m_sliceID = 0;
    m_numTilesInSlice = 0;
    m_numCtuInSlice = 0;
    m_ctuAddrInSlice.clear();
  }

  void  addCtusToSlice( uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY ) 
  {
    CHECK( startX >= stopX || startY >= stopY, "Invalid slice definition");
    for( uint32_t ctbY = startY; ctbY < stopY; ctbY++ ) 
    {
      for( uint32_t ctbX = startX; ctbX < stopX; ctbX++ ) 
      {
        m_ctuAddrInSlice.push_back( ctbY * picWidthInCtbsY + ctbX );
        m_numCtuInSlice++;
      }
    }
  }
};

class RectSlice
{
private:
  uint32_t         m_tileIdx;                           //!< tile index corresponding to the first CTU in the slice
  uint32_t         m_sliceWidthInTiles;                 //!< slice width in units of tiles
  uint32_t         m_sliceHeightInTiles;                //!< slice height in units of tiles
  uint32_t         m_numSlicesInTile;                   //!< number of slices in current tile for the special case of multiple slices inside a single tile
  uint32_t         m_sliceHeightInCtu;                  //!< slice height in units of CTUs for the special case of multiple slices inside a single tile
  
public:
  RectSlice();
  virtual ~RectSlice();
  
  void             setSliceWidthInTiles( uint32_t u )   { m_sliceWidthInTiles = u;      }
  uint32_t         getSliceWidthInTiles( ) const        { return  m_sliceWidthInTiles;  }
  void             setSliceHeightInTiles( uint32_t u )  { m_sliceHeightInTiles = u;     }
  uint32_t         getSliceHeightInTiles( ) const       { return  m_sliceHeightInTiles; }
  void             setNumSlicesInTile( uint32_t u )     { m_numSlicesInTile = u;        }
  uint32_t         getNumSlicesInTile( ) const          { return  m_numSlicesInTile;    }
  void             setSliceHeightInCtu( uint32_t u )    { m_sliceHeightInCtu = u;       }
  uint32_t         getSliceHeightInCtu( ) const         { return  m_sliceHeightInCtu;   }
  void             setTileIdx( uint32_t u )             { m_tileIdx = u;                }
  uint32_t         getTileIdx( ) const                  { return  m_tileIdx;            }

};

#if JVET_O1143_SUBPIC_BOUNDARY
class SubPic
{
private:
  uint32_t         m_subPicID;                                  //!< ID of subpicture
  uint32_t         m_subPicIdx;                                 //!< Index of subpicture
  uint32_t         m_numCTUsInSubPic;                           //!< number of CTUs contained in this sub-picture
  uint32_t         m_subPicCtuTopLeftX;                         //!< horizontal position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicCtuTopLeftY;                         //!< vertical position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicWidth;                               //!< the width of subpicture in units of CTU
  uint32_t         m_subPicHeight;                              //!< the height of subpicture in units of CTU
  uint32_t         m_subPicWidthInLumaSample;                   //!< the width of subpicture in units of luma sample
  uint32_t         m_subPicHeightInLumaSample;                  //!< the height of subpicture in units of luma sample
  uint32_t         m_firstCtuInSubPic;                          //!< the raster scan index of the first CTU in a subpicture
  uint32_t         m_lastCtuInSubPic;                           //!< the raster scan index of the last CTU in a subpicture
  uint32_t         m_subPicLeft;                                //!< the position of left boundary 
  uint32_t         m_subPicRight;                               //!< the position of right boundary
  uint32_t         m_subPicTop;                                 //!< the position of top boundary
  uint32_t         m_subPicBottom;                              //!< the position of bottom boundary
  std::vector<uint32_t> m_ctuAddrInSubPic;                      //!< raster scan addresses of all the CTUs in the slice

  bool             m_treatedAsPicFlag;                          //!< whether the subpicture is treated as a picture in the decoding process excluding in-loop filtering operations
  bool             m_loopFilterAcrossSubPicEnabledFlag;         //!< whether in-loop filtering operations may be performed across the boundaries of the subpicture
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  uint32_t         m_numSlicesInSubPic;                         //!< Number of slices contained in this subpicture
#endif
  
public:
  SubPic();
  virtual ~SubPic();

  void             setSubPicID (uint32_t u)                {         m_subPicID = u;       }
  uint32_t         getSubPicID   ()                  const { return  m_subPicID;           }
  void             setSubPicIdx (uint32_t u)               {         m_subPicIdx = u;      }
  uint32_t         getSubPicIdx ()                   const { return  m_subPicIdx;          }
  void             setNumCTUsInSubPic   (uint32_t u)       {         m_numCTUsInSubPic = u;       }
  uint32_t         getNumCTUsInSubPic   ()           const { return  m_numCTUsInSubPic;           }
  void             setSubPicCtuTopLeftX (uint32_t u)       {         m_subPicCtuTopLeftX = u;     }
  uint32_t         getSubPicCtuTopLeftX ()           const { return  m_subPicCtuTopLeftX;         }
  void             setSubPicCtuTopLeftY (uint32_t u)       {         m_subPicCtuTopLeftY = u;     }
  uint32_t         getSubPicCtuTopLeftY ()           const { return  m_subPicCtuTopLeftY;         }
  void             setSubPicWidthInCTUs (uint32_t u)       {         m_subPicWidth = u;           }
  uint32_t         getSubPicWidthInCTUs ()           const { return  m_subPicWidth;               }
  void             setSubPicHeightInCTUs(uint32_t u)       {         m_subPicHeight = u;          }
  uint32_t         getSubPicHeightInCTUs()           const { return  m_subPicHeight;              }
  void             setFirstCTUInSubPic  (uint32_t u)       {         m_firstCtuInSubPic = u;      }
  uint32_t         getFirstCTUInSubPic  ()           const { return  m_firstCtuInSubPic;          }
  void             setLastCTUInSubPic   (uint32_t u)       {         m_lastCtuInSubPic = u;       }
  uint32_t         getLastCTUInSubPic   ()           const { return  m_lastCtuInSubPic;           }
  void             setSubPicLeft        (uint32_t u)       {         m_subPicLeft = u;            }
  uint32_t         getSubPicLeft        ()           const { return  m_subPicLeft;                }
  void             setSubPicRight       (uint32_t u)       {         m_subPicRight = u;           }
  uint32_t         getSubPicRight       ()           const { return  m_subPicRight;               }
  void             setSubPicTop         (uint32_t u)       {         m_subPicTop = u;             }
  uint32_t         getSubPicTop         ()           const { return  m_subPicTop;                 }
  void             setSubPicBottom      (uint32_t u)       {         m_subPicBottom = u;          }
  uint32_t         getSubPicBottom      ()           const { return  m_subPicBottom;              }

  void             setSubPicWidthInLumaSample (uint32_t u) {         m_subPicWidthInLumaSample = u;   }
  uint32_t         getSubPicWidthInLumaSample()      const { return  m_subPicWidthInLumaSample;       }
  void             setSubPicHeightInLumaSample(uint32_t u) {         m_subPicHeightInLumaSample = u;  }
  uint32_t         getSubPicHeightInLumaSample()     const { return  m_subPicHeightInLumaSample;      }

  std::vector<uint32_t> getCtuAddrList  ()           const { return  m_ctuAddrInSubPic;           }
  void                  clearCTUAddrList()                 { m_ctuAddrInSubPic.clear(); }
  void                  addCTUsToSubPic(std::vector<uint32_t> ctuAddrInSlice)
  {
    for (auto ctu:ctuAddrInSlice)
      m_ctuAddrInSubPic.push_back(ctu);    
  }
  void  addAllCtusInPicToSubPic(uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY)
  {
    CHECK(startX >= stopX || startY >= stopY, "Invalid slice definition");
    for (uint32_t ctbY = startY; ctbY < stopY; ctbY++)
    {
      for (uint32_t ctbX = startX; ctbX < stopX; ctbX++)
      {
        m_ctuAddrInSubPic.push_back(ctbY * picWidthInCtbsY + ctbX);
      }
    }
  }
  bool                 isContainingPos(const Position& pos) const
  {
    return pos.x >= m_subPicLeft && pos.x <= m_subPicRight && pos.y >= m_subPicTop  && pos.y <= m_subPicBottom;
  }
  void             setTreatedAsPicFlag           (bool u)  {         m_treatedAsPicFlag = u;   }
  bool             getTreatedAsPicFlag           ()  const { return  m_treatedAsPicFlag;       }
  void             setloopFilterAcrossEnabledFlag(bool u)  {         m_loopFilterAcrossSubPicEnabledFlag = u; }
  bool             getloopFilterAcrossEnabledFlag()  const { return  m_loopFilterAcrossSubPicEnabledFlag;     }

  bool             isFirstCTUinSubPic(uint32_t ctuAddr)    { return  ctuAddr == m_firstCtuInSubPic;  }
  bool              isLastCTUinSubPic(uint32_t ctuAddr)    { return  ctuAddr == m_lastCtuInSubPic;   }
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  void             setNumSlicesInSubPic( uint32_t val )    { m_numSlicesInSubPic = val; }
  uint32_t         getNumSlicesInSubPic() const            { return m_numSlicesInSubPic; }
  bool             containsCtu(const Position& pos) const
  {
    return pos.x >= m_subPicCtuTopLeftX && pos.x < m_subPicCtuTopLeftX + m_subPicWidth &&
           pos.y >= m_subPicCtuTopLeftY && pos.y < m_subPicCtuTopLeftY + m_subPicHeight;
  }
#endif
};
#endif

#if JVET_Q0117_PARAMETER_SETS_CLEANUP // Rename DPS to DCI (decoding_capability_information)
class DCI
{
private:
  int m_maxSubLayersMinus1;
  std::vector<ProfileTierLevel> m_profileTierLevel;

public:
  DCI()
    : m_maxSubLayersMinus1(0)
  {};

  virtual ~DCI() {};

  int  getMaxSubLayersMinus1() const { return m_maxSubLayersMinus1; }
  void setMaxSubLayersMinus1(int val) { m_maxSubLayersMinus1 = val; }

  size_t getNumPTLs() const { return m_profileTierLevel.size(); }
  void  setProfileTierLevel(const std::vector<ProfileTierLevel>& val) { m_profileTierLevel = val; }
  const ProfileTierLevel& getProfileTierLevel(int idx) const { return m_profileTierLevel[idx]; }
  bool  IsIndenticalDCI(const DCI& comparedDCI) const
  {
    if(m_maxSubLayersMinus1 != comparedDCI.m_maxSubLayersMinus1) return false;
    if(m_profileTierLevel != comparedDCI.m_profileTierLevel) return false;
    return true;
  }
};
#else
class DPS
{
private:
  int m_decodingParameterSetId;
  int m_maxSubLayersMinus1;
  std::vector<ProfileTierLevel> m_profileTierLevel;

public:
  DPS()
    : m_decodingParameterSetId(-1)
    , m_maxSubLayersMinus1 (0)
  {};

  virtual ~DPS() {};

  int  getDecodingParameterSetId() const { return m_decodingParameterSetId; }
  void setDecodingParameterSetId(int val) { m_decodingParameterSetId = val; }
  int  getMaxSubLayersMinus1() const { return m_maxSubLayersMinus1; }
  void setMaxSubLayersMinus1(int val) { m_maxSubLayersMinus1 = val; }

  size_t getNumPTLs() const { return m_profileTierLevel.size(); }
  void  setProfileTierLevel(const std::vector<ProfileTierLevel> &val)   { m_profileTierLevel = val; }
  const ProfileTierLevel& getProfileTierLevel(int idx) const            { return m_profileTierLevel[idx]; }
};

#endif


class VPS
{
private:
  int                   m_VPSId;
  uint32_t              m_uiMaxLayers;

  uint32_t              m_vpsMaxSubLayers;
  uint32_t              m_vpsLayerId[MAX_VPS_LAYERS];
  bool                  m_vpsAllLayersSameNumSubLayersFlag;
  bool                  m_vpsAllIndependentLayersFlag;
  bool                  m_vpsIndependentLayerFlag[MAX_VPS_LAYERS];
  bool                  m_vpsDirectRefLayerFlag[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  bool                  m_vpsEachLayerIsAnOlsFlag;
  uint32_t              m_vpsOlsModeIdc;
  uint32_t              m_vpsNumOutputLayerSets;
  bool                  m_vpsOlsOutputLayerFlag[MAX_NUM_OLSS][MAX_VPS_LAYERS];
  uint32_t              m_directRefLayerIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  uint32_t              m_generalLayerIdx[MAX_VPS_LAYERS];

#if JVET_Q0786_PTL_only
  uint32_t              m_vpsNumPtls;
  bool                  m_ptPresentFlag[MAX_NUM_OLSS];
  uint32_t              m_ptlMaxTemporalId[MAX_NUM_OLSS];
  std::vector<ProfileTierLevel> m_vpsProfileTierLevel;
  uint32_t              m_olsPtlIdx[MAX_NUM_OLSS];
#endif

  // stores index ( ilrp_idx within 0 .. NumDirectRefLayers ) of the dependent reference layers 
  uint32_t              m_interLayerRefIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  bool                  m_vpsExtensionFlag;

#if JVET_Q0814_DPB
  std::vector<Size>             m_olsDpbPicSize;
  std::vector<int>              m_olsDpbParamsIdx;
  std::vector<std::vector<int>> m_outputLayerIdInOls;
public:
  int                           m_totalNumOLSs;
  int                           m_numDpbParams;
  std::vector<DpbParameters>    m_dpbParameters;
  bool                          m_sublayerDpbParamsPresentFlag;
  std::vector<int>              m_dpbMaxTemporalId;
  std::vector<int>              m_targetOutputLayerIdSet;          ///< set of LayerIds to be outputted
  std::vector<int>              m_targetLayerIdSet;                ///< set of LayerIds to be included in the sub-bitstream extraction process.
  int                           m_targetOlsIdx;
  std::vector<int>              m_numOutputLayersInOls;
  std::vector<int>              m_numLayersInOls;
  std::vector<std::vector<int>> m_layerIdInOls;
#elif JVET_P0288_PIC_OUTPUT
public:
  int                   m_targetOlsIdx;
#endif

public:
                    VPS();

  virtual           ~VPS();

  int               getVPSId() const                                     { return m_VPSId;                                                  }
  void              setVPSId(int i)                                      { m_VPSId = i;                                                     }

  uint32_t          getMaxLayers() const                                 { return m_uiMaxLayers;                                            }
  void              setMaxLayers(uint32_t l)                             { m_uiMaxLayers = l;                                               }

  uint32_t          getMaxSubLayers() const                              { return m_vpsMaxSubLayers;                                        }
  void              setMaxSubLayers(uint32_t value)                      { m_vpsMaxSubLayers = value;                                       }
  bool              getAllLayersSameNumSublayersFlag() const { return m_vpsAllLayersSameNumSubLayersFlag; }
  void              setAllLayersSameNumSublayersFlag(bool t) { m_vpsAllLayersSameNumSubLayersFlag = t; }

  uint32_t          getLayerId(uint32_t layerIdx) const { return m_vpsLayerId[layerIdx]; }
  void              setLayerId(uint32_t layerIdx, uint32_t layerId) { m_vpsLayerId[layerIdx] = layerId; }

  bool              getAllIndependentLayersFlag() const { return m_vpsAllIndependentLayersFlag; }
  void              setAllIndependentLayersFlag(bool t) { m_vpsAllIndependentLayersFlag = t; }

  bool              getIndependentLayerFlag(uint32_t layerIdx) const { return m_vpsIndependentLayerFlag[layerIdx]; }
  void              setIndependentLayerFlag(uint32_t layerIdx, bool t) { m_vpsIndependentLayerFlag[layerIdx] = t; }

  bool              getDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx) const { return m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx]; }
  void              setDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx, bool t) { m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx] = t; }

  uint32_t          getDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc ) const { return m_directRefLayerIdx[layerIdx][refLayerIdc]; }
  void              setDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc, uint32_t refLayerIdx ) { m_directRefLayerIdx[layerIdx][refLayerIdc] = refLayerIdx; }

  uint32_t          getInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx ) const { return m_interLayerRefIdx[layerIdx][refLayerIdx]; }
  void              setInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx, uint32_t refLayerIdc ) { m_interLayerRefIdx[layerIdx][refLayerIdx] = refLayerIdc; }

  uint32_t          getGeneralLayerIdx(uint32_t layerId) const { return m_generalLayerIdx[layerId]; }
  void              setGeneralLayerIdx(uint32_t layerId, uint32_t layerIdc) { m_generalLayerIdx[layerId] = layerIdc; }

  bool              getEachLayerIsAnOlsFlag() const { return m_vpsEachLayerIsAnOlsFlag; }
  void              setEachLayerIsAnOlsFlag(bool t) { m_vpsEachLayerIsAnOlsFlag = t; }

  uint32_t          getOlsModeIdc() const { return m_vpsOlsModeIdc; }
  void              setOlsModeIdc(uint32_t t) { m_vpsOlsModeIdc = t; }

  uint32_t          getNumOutputLayerSets() const { return m_vpsNumOutputLayerSets; }
  void              setNumOutputLayerSets(uint8_t t) { m_vpsNumOutputLayerSets = t; }

  bool              getOlsOutputLayerFlag(uint32_t ols, uint32_t layer) const { return m_vpsOlsOutputLayerFlag[ols][layer]; }
  void              setOlsOutputLayerFlag(uint32_t ols, uint32_t layer, bool t) { m_vpsOlsOutputLayerFlag[ols][layer] = t; }

#if JVET_Q0786_PTL_only
  uint32_t          getNumPtls()                                   const { return m_vpsNumPtls; }
  void              setNumPtls(uint32_t val)                             { m_vpsNumPtls = val; }

  bool              getPtPresentFlag(int idx)                      const { return m_ptPresentFlag[idx]; }
  void              setPtPresentFlag(int idx, bool val)                  { m_ptPresentFlag[idx] = val; }

  uint32_t          getPtlMaxTemporalId(int idx)                   const { return m_ptlMaxTemporalId[idx]; }
  void              setPtlMaxTemporalId(int idx, uint32_t val)           { m_ptlMaxTemporalId[idx] = val; }

  void              setProfileTierLevel(const std::vector<ProfileTierLevel> &val)   { m_vpsProfileTierLevel = val; }
  const ProfileTierLevel& getProfileTierLevel(int idx)             const { return m_vpsProfileTierLevel[idx]; }

  uint32_t          getOlsPtlIdx(int idx)                          const { return m_olsPtlIdx[idx]; }
  void              setOlsPtlIdx(int idx, uint32_t val)                  { m_olsPtlIdx[idx] = val; }
#endif

  bool              getVPSExtensionFlag() const                          { return m_vpsExtensionFlag;                                 }
  void              setVPSExtensionFlag(bool t)                          { m_vpsExtensionFlag = t;                                    }

#if JVET_P0288_PIC_OUTPUT
  int               getTargetOlsIdx() { return m_targetOlsIdx; }
  void              setTargetOlsIdx(uint32_t t) { m_targetOlsIdx = t; }
#endif

#if JVET_Q0814_DPB
  int               getMaxDecPicBuffering( int temporalId ) const        { return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].m_maxDecPicBuffering[temporalId]; }
  int               getNumReorderPics( int temporalId ) const            { return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].m_numReorderPics[temporalId]; }
  int               getTotalNumOLSs() const                              { return m_totalNumOLSs; }
  Size              getOlsDpbPicSize( int olsIdx ) const                 { return m_olsDpbPicSize[olsIdx];          }
  void              setOlsDpbPicSize( int olsIdx, Size size )            { m_olsDpbPicSize[olsIdx] = size;          }
  void              setOlsDpbPicWidth( int olsIdx, int width )           { m_olsDpbPicSize[olsIdx].width = width;   }
  void              setOlsDpbPicHeight( int olsIdx, int height )         { m_olsDpbPicSize[olsIdx].height = height; }

  int               getOlsDpbParamsIdx( int olsIdx ) const               { return m_olsDpbParamsIdx[olsIdx];        }
  void              setOlsDpbParamsIdx( int olsIdx, int paramIdx )       { m_olsDpbParamsIdx[olsIdx] = paramIdx;    }
  
  void              deriveOutputLayerSets();
  void              deriveTargetOutputLayerSet( int targetOlsIdx );
#endif
};

class Window
{
private:
  bool m_enabledFlag;
  int  m_winLeftOffset;
  int  m_winRightOffset;
  int  m_winTopOffset;
  int  m_winBottomOffset;
public:
  Window()
  : m_enabledFlag    (false)
  , m_winLeftOffset  (0)
  , m_winRightOffset (0)
  , m_winTopOffset   (0)
  , m_winBottomOffset(0)
  { }

  bool getWindowEnabledFlag() const   { return m_enabledFlag;                          }
  int  getWindowLeftOffset() const    { return m_enabledFlag ? m_winLeftOffset : 0;    }
  void setWindowLeftOffset(int val)   { m_winLeftOffset = val; m_enabledFlag = true;   }
  int  getWindowRightOffset() const   { return m_enabledFlag ? m_winRightOffset : 0;   }
  void setWindowRightOffset(int val)  { m_winRightOffset = val; m_enabledFlag = true;  }
  int  getWindowTopOffset() const     { return m_enabledFlag ? m_winTopOffset : 0;     }
  void setWindowTopOffset(int val)    { m_winTopOffset = val; m_enabledFlag = true;    }
  int  getWindowBottomOffset() const  { return m_enabledFlag ? m_winBottomOffset: 0;   }
  void setWindowBottomOffset(int val) { m_winBottomOffset = val; m_enabledFlag = true; }

  void setWindow(int offsetLeft, int offsetLRight, int offsetLTop, int offsetLBottom)
  {
    m_enabledFlag     = true;
    m_winLeftOffset   = offsetLeft;
    m_winRightOffset  = offsetLRight;
    m_winTopOffset    = offsetLTop;
    m_winBottomOffset = offsetLBottom;
  }
};


class VUI
{
private:
  bool       m_aspectRatioInfoPresentFlag;
  bool       m_aspectRatioConstantFlag;
  int        m_aspectRatioIdc;
  int        m_sarWidth;
  int        m_sarHeight;
  bool       m_colourDescriptionPresentFlag;
  int        m_colourPrimaries;
  int        m_transferCharacteristics;
  int        m_matrixCoefficients;
#if !JVET_Q0042_VUI
  bool       m_fieldSeqFlag;
#endif
  bool       m_chromaLocInfoPresentFlag;
  int        m_chromaSampleLocTypeTopField;
  int        m_chromaSampleLocTypeBottomField;
  int        m_chromaSampleLocType;
  bool       m_overscanInfoPresentFlag;
  bool       m_overscanAppropriateFlag;
  bool       m_videoFullRangeFlag;

public:
  VUI()
    : m_aspectRatioInfoPresentFlag        (false) //TODO: This initialiser list contains magic numbers
    , m_aspectRatioConstantFlag           (true)
    , m_aspectRatioIdc                    (0)
    , m_sarWidth                          (0)
    , m_sarHeight                         (0)
    , m_colourDescriptionPresentFlag      (false)
    , m_colourPrimaries                   (2)
    , m_transferCharacteristics           (2)
    , m_matrixCoefficients                (2)
#if !JVET_Q0042_VUI
    , m_fieldSeqFlag                      (false)
#endif
    , m_chromaLocInfoPresentFlag          (false)
    , m_chromaSampleLocTypeTopField       (0)
    , m_chromaSampleLocTypeBottomField    (0)
    , m_chromaSampleLocType               (0)
    , m_overscanInfoPresentFlag           (false)
    , m_overscanAppropriateFlag           (false)
    , m_videoFullRangeFlag                (false)
  {}

  virtual           ~VUI() {}

  bool              getAspectRatioInfoPresentFlag() const                  { return m_aspectRatioInfoPresentFlag;           }
  void              setAspectRatioInfoPresentFlag(bool i)                  { m_aspectRatioInfoPresentFlag = i;              }
  bool              getAspectRatioConstantFlag() const                     { return m_aspectRatioConstantFlag;           }
  void              setAspectRatioConstantFlag(bool b)                     { m_aspectRatioConstantFlag = b;              }

  int               getAspectRatioIdc() const                              { return m_aspectRatioIdc;                       }
  void              setAspectRatioIdc(int i)                               { m_aspectRatioIdc = i;                          }

  int               getSarWidth() const                                    { return m_sarWidth;                             }
  void              setSarWidth(int i)                                     { m_sarWidth = i;                                }

  int               getSarHeight() const                                   { return m_sarHeight;                            }
  void              setSarHeight(int i)                                    { m_sarHeight = i;                               }

  bool              getColourDescriptionPresentFlag() const                { return m_colourDescriptionPresentFlag;         }
  void              setColourDescriptionPresentFlag(bool i)                { m_colourDescriptionPresentFlag = i;            }

  int               getColourPrimaries() const                             { return m_colourPrimaries;                      }
  void              setColourPrimaries(int i)                              { m_colourPrimaries = i;                         }

  int               getTransferCharacteristics() const                     { return m_transferCharacteristics;              }
  void              setTransferCharacteristics(int i)                      { m_transferCharacteristics = i;                 }

  int               getMatrixCoefficients() const                          { return m_matrixCoefficients;                   }
  void              setMatrixCoefficients(int i)                           { m_matrixCoefficients = i;                      }

#if !JVET_Q0042_VUI
  bool              getFieldSeqFlag() const                                { return m_fieldSeqFlag;                         }
  void              setFieldSeqFlag(bool i)                                { m_fieldSeqFlag = i;                            }
#endif

  bool              getChromaLocInfoPresentFlag() const                    { return m_chromaLocInfoPresentFlag;             }
  void              setChromaLocInfoPresentFlag(bool i)                    { m_chromaLocInfoPresentFlag = i;                }

  int               getChromaSampleLocTypeTopField() const                 { return m_chromaSampleLocTypeTopField;          }
  void              setChromaSampleLocTypeTopField(int i)                  { m_chromaSampleLocTypeTopField = i;             }

  int               getChromaSampleLocTypeBottomField() const              { return m_chromaSampleLocTypeBottomField;       }
  void              setChromaSampleLocTypeBottomField(int i)               { m_chromaSampleLocTypeBottomField = i;          }

  int               getChromaSampleLocType() const                         { return m_chromaSampleLocType;          }
  void              setChromaSampleLocType(int i)                          { m_chromaSampleLocType = i;             }

  bool              getOverscanInfoPresentFlag() const                     { return m_overscanInfoPresentFlag;              }
  void              setOverscanInfoPresentFlag(bool i)                     { m_overscanInfoPresentFlag = i;                 }

  bool              getOverscanAppropriateFlag() const                     { return m_overscanAppropriateFlag;              }
  void              setOverscanAppropriateFlag(bool i)                     { m_overscanAppropriateFlag = i;                 }

  bool              getVideoFullRangeFlag() const                          { return m_videoFullRangeFlag;                   }
  void              setVideoFullRangeFlag(bool i)                          { m_videoFullRangeFlag = i;                      }

};

/// SPS RExt class
class SPSRExt // Names aligned to text specification
{
private:
  bool             m_transformSkipRotationEnabledFlag;
  bool             m_transformSkipContextEnabledFlag;
  bool             m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];
  bool             m_extendedPrecisionProcessingFlag;
  bool             m_intraSmoothingDisabledFlag;
  bool             m_highPrecisionOffsetsEnabledFlag;
  bool             m_persistentRiceAdaptationEnabledFlag;
  bool             m_cabacBypassAlignmentEnabledFlag;

public:
  SPSRExt();

  bool settingsDifferFromDefaults() const
  {
    return getTransformSkipRotationEnabledFlag()
        || getTransformSkipContextEnabledFlag()
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT)
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT)
        || getExtendedPrecisionProcessingFlag()
        || getIntraSmoothingDisabledFlag()
        || getHighPrecisionOffsetsEnabledFlag()
        || getPersistentRiceAdaptationEnabledFlag()
        || getCabacBypassAlignmentEnabledFlag();
  }


  bool getTransformSkipRotationEnabledFlag() const                                     { return m_transformSkipRotationEnabledFlag;     }
  void setTransformSkipRotationEnabledFlag(const bool value)                           { m_transformSkipRotationEnabledFlag = value;    }

  bool getTransformSkipContextEnabledFlag() const                                      { return m_transformSkipContextEnabledFlag;      }
  void setTransformSkipContextEnabledFlag(const bool value)                            { m_transformSkipContextEnabledFlag = value;     }

  bool getRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode) const             { return m_rdpcmEnabledFlag[signallingMode];     }
  void setRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode, const bool value) { m_rdpcmEnabledFlag[signallingMode] = value;    }

  bool getExtendedPrecisionProcessingFlag() const                                      { return m_extendedPrecisionProcessingFlag;      }
  void setExtendedPrecisionProcessingFlag(bool value)                                  { m_extendedPrecisionProcessingFlag = value;     }

  bool getIntraSmoothingDisabledFlag() const                                           { return m_intraSmoothingDisabledFlag;           }
  void setIntraSmoothingDisabledFlag(bool bValue)                                      { m_intraSmoothingDisabledFlag=bValue;           }

  bool getHighPrecisionOffsetsEnabledFlag() const                                      { return m_highPrecisionOffsetsEnabledFlag;      }
  void setHighPrecisionOffsetsEnabledFlag(bool value)                                  { m_highPrecisionOffsetsEnabledFlag = value;     }

  bool getPersistentRiceAdaptationEnabledFlag() const                                  { return m_persistentRiceAdaptationEnabledFlag;  }
  void setPersistentRiceAdaptationEnabledFlag(const bool value)                        { m_persistentRiceAdaptationEnabledFlag = value; }

  bool getCabacBypassAlignmentEnabledFlag() const                                      { return m_cabacBypassAlignmentEnabledFlag;      }
  void setCabacBypassAlignmentEnabledFlag(const bool value)                            { m_cabacBypassAlignmentEnabledFlag = value;     }
};


/// SPS class
class SPS
{
private:
  int               m_SPSId;
#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  int               m_decodingParameterSetId;
#endif
  int               m_VPSId;

  bool              m_affineAmvrEnabledFlag;
  bool              m_DMVR;
  bool              m_MMVD;
  bool              m_SBT;
  bool              m_ISP;
  ChromaFormat      m_chromaFormatIdc;
  bool              m_separateColourPlaneFlag;     //!< separate colour plane flag

  uint32_t              m_uiMaxTLayers;           // maximum number of temporal layers

#if JVET_P0117_PTL_SCALABILITY
  bool              m_ptlDpbHrdParamsPresentFlag;
  bool              m_SubLayerDpbParamsFlag;
#endif

  // Structure
  uint32_t              m_maxWidthInLumaSamples;
  uint32_t              m_maxHeightInLumaSamples;
#if JVET_Q0260_CONFORMANCE_WINDOW_IN_SPS
  Window                m_conformanceWindow;
#endif
#if JVET_Q0119_CLEANUPS
  bool                  m_subPicInfoPresentFlag;                // indicates the presence of sub-picture info
#else
  bool                  m_subPicPresentFlag;                    // indicates the presence of sub-pictures
#endif
  uint8_t               m_numSubPics;                        //!< number of sub-pictures used
  uint32_t              m_subPicCtuTopLeftX[MAX_NUM_SUB_PICS];                
  uint32_t              m_subPicCtuTopLeftY[MAX_NUM_SUB_PICS];                
  uint32_t              m_SubPicWidth[MAX_NUM_SUB_PICS];                      
  uint32_t              m_SubPicHeight[MAX_NUM_SUB_PICS];                     
  bool                  m_subPicTreatedAsPicFlag[MAX_NUM_SUB_PICS];           
  bool                  m_loopFilterAcrossSubpicEnabledFlag[MAX_NUM_SUB_PICS];
#if JVET_Q0119_CLEANUPS
  bool                  m_subPicIdMappingExplicitlySignalledFlag;
  bool                  m_subPicIdMappingInSpsFlag;
#else
  bool                  m_subPicIdPresentFlag;               //!< indicates the presence of sub-picture IDs
  bool                  m_subPicIdSignallingPresentFlag;     //!< indicates the presence of sub-picture ID signalling in the SPS
#endif
  uint32_t              m_subPicIdLen;                       //!< sub-picture ID length in bits
  uint8_t               m_subPicId[MAX_NUM_SUB_PICS];        //!< sub-picture ID for each sub-picture in the sequence

  int               m_log2MinCodingBlockSize;
#if !JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
  int               m_log2DiffMaxMinCodingBlockSize;
#endif
  unsigned    m_CTUSize;
  unsigned    m_partitionOverrideEnalbed;       // enable partition constraints override function
  unsigned    m_minQT[3];   // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned    m_maxMTTHierarchyDepth[3];
  unsigned    m_maxBTSize[3];
  unsigned    m_maxTTSize[3];
  bool        m_idrRefParamList;
  unsigned    m_dualITree;
  uint32_t              m_uiMaxCUWidth;
  uint32_t              m_uiMaxCUHeight;
#if !JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
  uint32_t              m_uiMaxCodingDepth; ///< Total CU depth, relative to the smallest possible transform block size.
#endif

  RPLList           m_RPLList0;
  RPLList           m_RPLList1;
  uint32_t          m_numRPL0;
  uint32_t          m_numRPL1;
  bool              m_rpl1CopyFromRpl0Flag;
  bool              m_rpl1IdxPresentFlag;
  bool              m_allRplEntriesHasSameSignFlag;
  bool              m_bLongTermRefsPresent;
  bool              m_SPSTemporalMVPEnabledFlag;
  int               m_numReorderPics[MAX_TLAYER];

  // Tool list

  bool              m_transformSkipEnabledFlag;
#if JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
  int               m_log2MaxTransformSkipBlockSize;
#endif
#if JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
  bool              m_BDPCMEnabledFlag;
#else
  int               m_BDPCMEnabled;
#endif
  bool              m_JointCbCrEnabledFlag;
  // Parameter
  BitDepths         m_bitDepths;
#if JVET_Q0151_Q0205_ENTRYPOINTS
  bool              m_entropyCodingSyncEnabledFlag;                    //!< Flag for enabling WPP
  bool              m_entropyCodingSyncEntryPointPresentFlag;          //!< Flag for indicating the presence of WPP entry points
 #endif
  int               m_qpBDOffset[MAX_NUM_CHANNEL_TYPE];
  int               m_minQpMinus4[MAX_NUM_CHANNEL_TYPE]; //  QP_internal - QP_input;

  bool              m_sbtmvpEnabledFlag;
  bool              m_bdofEnabledFlag;
  bool              m_fpelMmvdEnabledFlag;
  bool              m_BdofControlPresentFlag;
  bool              m_DmvrControlPresentFlag;
  bool              m_ProfControlPresentFlag;
  uint32_t          m_uiBitsForPOC;
#if JVET_P0116_POC_MSB
  bool              m_pocMsbFlag;
  uint32_t          m_pocMsbLen;
#endif
#if JVET_Q0400_EXTRA_BITS
  int               m_numExtraPHBitsBytes;
  int               m_numExtraSHBitsBytes;
  std::vector<bool> m_extraPHBitPresentFlag;
  std::vector<bool> m_extraSHBitPresentFlag;
#endif
  uint32_t          m_numLongTermRefPicSPS;
  uint32_t          m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  bool              m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS];
  uint32_t          m_log2MaxTbSize;
  bool              m_useWeightPred;                     //!< Use of Weighting Prediction (P_SLICE)
  bool              m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)

  bool              m_saoEnabledFlag;

  bool              m_bTemporalIdNestingFlag; // temporal_id_nesting_flag

  bool              m_scalingListEnabledFlag;
  bool              m_loopFilterAcrossVirtualBoundariesDisabledFlag;   //!< disable loop filtering across virtual boundaries
  unsigned          m_numVerVirtualBoundaries;                         //!< number of vertical virtual boundaries
  unsigned          m_numHorVirtualBoundaries;                         //!< number of horizontal virtual boundaries
  unsigned          m_virtualBoundariesPosX[3];                        //!< horizontal position of each vertical virtual boundary
  unsigned          m_virtualBoundariesPosY[3];                        //!< vertical position of each horizontal virtual boundary
  uint32_t          m_uiMaxDecPicBuffering[MAX_TLAYER];
  uint32_t          m_uiMaxLatencyIncreasePlus1[MAX_TLAYER];


  TimingInfo        m_timingInfo;
  bool              m_hrdParametersPresentFlag;
  HRDParameters     m_hrdParameters;

#if JVET_Q0042_VUI
  bool              m_fieldSeqFlag;
#endif
  bool              m_vuiParametersPresentFlag;
  VUI               m_vuiParameters;

  SPSRExt           m_spsRangeExtension;

  static const int  m_winUnitX[NUM_CHROMA_FORMAT];
  static const int  m_winUnitY[NUM_CHROMA_FORMAT];
  ProfileTierLevel  m_profileTierLevel;

  bool              m_alfEnabledFlag;
#if JVET_Q0795_CCALF
  bool              m_ccalfEnabledFlag;
#endif
  bool              m_wrapAroundEnabledFlag;
  unsigned          m_wrapAroundOffset;
  unsigned          m_IBCFlag;
  bool              m_useColorTrans;
  unsigned          m_PLTMode;

  bool              m_lmcsEnabled;
  bool              m_AMVREnabledFlag;
  bool              m_LMChroma;
  bool              m_horCollocatedChromaFlag;
  bool              m_verCollocatedChromaFlag;
  bool              m_MTS;
  bool              m_IntraMTS;                   // 18
  bool              m_InterMTS;                   // 19
  bool              m_LFNST;
  bool              m_SMVD;
  bool              m_Affine;
  bool              m_AffineType;
  bool              m_PROF;
  bool              m_bcw;                        //
  bool              m_ciip;
#if !JVET_Q0806
  bool              m_Triangle;
#else
  bool              m_Geo;
#endif
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool              m_LadfEnabled;
  int               m_LadfNumIntervals;
  int               m_LadfQpOffset[MAX_LADF_INTERVALS];
  int               m_LadfIntervalLowerBound[MAX_LADF_INTERVALS];
#endif
  bool              m_MRL;
  bool              m_MIP;
  ChromaQpMappingTable m_chromaQpMappingTable;
  bool m_GDREnabledFlag;
  bool              m_SubLayerCbpParametersPresentFlag;

  bool              m_rprEnabledFlag;
  bool              m_interLayerPresentFlag;

#if JVET_Q0297_MER
  uint32_t          m_log2ParallelMergeLevelMinus2;
#endif
#if JVET_Q0179_SCALING_WINDOW_SIZE_CONSTRAINT
  bool              m_ppsValidFlag[64];
  Size              m_scalingWindowSizeInPPS[64];
#endif

public:

  SPS();
  virtual                 ~SPS();

  int                     getSPSId() const                                                                { return m_SPSId;                                                      }
  void                    setSPSId(int i)                                                                 { m_SPSId = i;                                                         }
#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  void                    setDecodingParameterSetId(int val)                                              { m_decodingParameterSetId = val; }
  int                     getDecodingParameterSetId() const                                               { return m_decodingParameterSetId; }
#endif
  int                     getVPSId() const                                                                { return m_VPSId; }
  void                    setVPSId(int i)                                                                 { m_VPSId = i; }

  ChromaFormat            getChromaFormatIdc () const                                                     { return m_chromaFormatIdc;                                            }
  void                    setChromaFormatIdc (ChromaFormat i)                                             { m_chromaFormatIdc = i;                                               }
  void                    setSeparateColourPlaneFlag ( bool b )                                           { m_separateColourPlaneFlag = b;                                       }
  bool                    getSeparateColourPlaneFlag () const                                             { return m_separateColourPlaneFlag;                                    }

  static int              getWinUnitX (int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitX[chromaFormatIdc]; }
  static int              getWinUnitY (int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitY[chromaFormatIdc]; }

  // structure
  void                    setMaxPicWidthInLumaSamples( uint32_t u )                                       { m_maxWidthInLumaSamples = u; }
  uint32_t                getMaxPicWidthInLumaSamples() const                                             { return  m_maxWidthInLumaSamples; }
  void                    setMaxPicHeightInLumaSamples( uint32_t u )                                      { m_maxHeightInLumaSamples = u; }
  uint32_t                getMaxPicHeightInLumaSamples() const                                            { return  m_maxHeightInLumaSamples; }
#if JVET_Q0260_CONFORMANCE_WINDOW_IN_SPS
  Window&                 getConformanceWindow()                                                          { return  m_conformanceWindow; }
  const Window&           getConformanceWindow() const                                                    { return  m_conformanceWindow; }
  void                    setConformanceWindow( Window& conformanceWindow )                               { m_conformanceWindow = conformanceWindow; }
#endif

#if JVET_Q0119_CLEANUPS
  void                    setSubPicInfoPresentFlag(bool b)                                                { m_subPicInfoPresentFlag = b;            }
  bool                    getSubPicInfoPresentFlag() const                                                { return m_subPicInfoPresentFlag;         }
#else
  void                    setSubPicPresentFlag(bool b)                                                    { m_subPicPresentFlag = b;                }
  bool                    getSubPicPresentFlag() const                                                    { return m_subPicPresentFlag;             }
#endif

  void                    setNumSubPics( uint8_t u )                                                      { m_numSubPics = u;                        }
  uint8_t                 getNumSubPics( ) const                                                          { return  m_numSubPics;                    }
  void                    setSubPicCtuTopLeftX( int i, uint32_t u )                                       { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicCtuTopLeftX[i] = u;                     }
  uint32_t                getSubPicCtuTopLeftX( int i ) const                                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicCtuTopLeftX[i];                 }
  void                    setSubPicCtuTopLeftY( int i, uint32_t u )                                       { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicCtuTopLeftY[i] = u;                     }
  uint32_t                getSubPicCtuTopLeftY( int i ) const                                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicCtuTopLeftY[i];                 }
  void                    setSubPicWidth( int i, uint32_t u )                                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_SubPicWidth[i] = u;                           }
  uint32_t                getSubPicWidth( int i ) const                                                   { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_SubPicWidth[i];                       }
  void                    setSubPicHeight( int i, uint32_t u )                                            { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_SubPicHeight[i] = u;                          }
  uint32_t                getSubPicHeight( int i ) const                                                  { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_SubPicHeight[i];                      }
  void                    setSubPicTreatedAsPicFlag( int i, bool u )                                      { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicTreatedAsPicFlag[i] = u;                }
  bool                    getSubPicTreatedAsPicFlag( int i ) const                                        { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicTreatedAsPicFlag[i];            }
  void                    setLoopFilterAcrossSubpicEnabledFlag( int i, bool u )                           { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_loopFilterAcrossSubpicEnabledFlag[i] = u;     }
  bool                    getLoopFilterAcrossSubpicEnabledFlag( int i ) const                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_loopFilterAcrossSubpicEnabledFlag[i]; }
#if JVET_Q0119_CLEANUPS
  void                    setSubPicIdMappingExplicitlySignalledFlag( bool b )                             { m_subPicIdMappingExplicitlySignalledFlag = b;    }
  bool                    getSubPicIdMappingExplicitlySignalledFlag() const                               { return m_subPicIdMappingExplicitlySignalledFlag; }
  void                    setSubPicIdMappingInSpsFlag( bool b )                                           { m_subPicIdMappingInSpsFlag = b;                  }
  bool                    getSubPicIdMappingInSpsFlag() const                                             { return  m_subPicIdMappingInSpsFlag;              }
#else
  void                    setSubPicIdPresentFlag( bool b )                                                { m_subPicIdPresentFlag = b;               }
  bool                    getSubPicIdPresentFlag() const                                                  { return  m_subPicIdPresentFlag;           }
  void                    setSubPicIdSignallingPresentFlag( bool b )                                      { m_subPicIdSignallingPresentFlag = b;     }
  bool                    getSubPicIdSignallingPresentFlag() const                                        { return  m_subPicIdSignallingPresentFlag; }
#endif
  void                    setSubPicIdLen( uint32_t u )                                                    { m_subPicIdLen = u;                       }
  uint32_t                getSubPicIdLen() const                                                          { return  m_subPicIdLen;                   }
  void                    setSubPicId( int i, uint8_t u )                                                 { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicId[i] = u;     }
  uint8_t                 getSubPicId( int i ) const                                                      { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicId[i]; }

  uint32_t                    getNumLongTermRefPicSPS() const                                                 { return m_numLongTermRefPicSPS;                                       }
  void                    setNumLongTermRefPicSPS(uint32_t val)                                               { m_numLongTermRefPicSPS = val;                                        }

  uint32_t                    getLtRefPicPocLsbSps(uint32_t index) const                                          { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_ltRefPicPocLsbSps[index]; }
  void                    setLtRefPicPocLsbSps(uint32_t index, uint32_t val)                                      { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_ltRefPicPocLsbSps[index] = val;  }

  bool                    getUsedByCurrPicLtSPSFlag(int i) const                                          { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_usedByCurrPicLtSPSFlag[i];    }
  void                    setUsedByCurrPicLtSPSFlag(int i, bool x)                                        { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_usedByCurrPicLtSPSFlag[i] = x;       }

  int                     getLog2MinCodingBlockSize() const                                               { return m_log2MinCodingBlockSize;                                     }
  void                    setLog2MinCodingBlockSize(int val)                                              { m_log2MinCodingBlockSize = val;                                      }
#if !JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
  int                     getLog2DiffMaxMinCodingBlockSize() const                                        { return m_log2DiffMaxMinCodingBlockSize;                              }
  void                    setLog2DiffMaxMinCodingBlockSize(int val)                                       { m_log2DiffMaxMinCodingBlockSize = val;                               }
#endif
  void                    setCTUSize(unsigned    ctuSize)                                                 { m_CTUSize = ctuSize; }
  unsigned                getCTUSize()                                                              const { return  m_CTUSize; }
  void                    setSplitConsOverrideEnabledFlag(bool b)                                         { m_partitionOverrideEnalbed = b; }
  bool                    getSplitConsOverrideEnabledFlag()                                         const { return m_partitionOverrideEnalbed; }
  void                    setMinQTSizes(unsigned*   minQT)                                                { m_minQT[0] = minQT[0]; m_minQT[1] = minQT[1]; m_minQT[2] = minQT[2]; }
  unsigned                getMinQTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA)
                                                                                                    const { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_minQT[0] : m_minQT[2]) : m_minQT[1]; }
  void                    setMaxMTTHierarchyDepth(unsigned    maxMTTHierarchyDepth,
                                        unsigned    maxMTTHierarchyDepthI,
                                        unsigned    maxMTTHierarchyDepthIChroma)
                                                                                                          { m_maxMTTHierarchyDepth[1] = maxMTTHierarchyDepth; m_maxMTTHierarchyDepth[0] = maxMTTHierarchyDepthI; m_maxMTTHierarchyDepth[2] = maxMTTHierarchyDepthIChroma; }
  unsigned                getMaxMTTHierarchyDepth()                                                 const { return m_maxMTTHierarchyDepth[1]; }
  unsigned                getMaxMTTHierarchyDepthI()                                                const { return m_maxMTTHierarchyDepth[0]; }
  unsigned                getMaxMTTHierarchyDepthIChroma()                                          const { return m_maxMTTHierarchyDepth[2]; }
  void                    setMaxBTSize(unsigned    maxBTSize,
                                       unsigned    maxBTSizeI,
                                       unsigned    maxBTSizeC)
                                                                                                          { m_maxBTSize[1] = maxBTSize; m_maxBTSize[0] = maxBTSizeI; m_maxBTSize[2] = maxBTSizeC; }
  unsigned                getMaxBTSize()                                                            const { return m_maxBTSize[1]; }
  unsigned                getMaxBTSizeI()                                                           const { return m_maxBTSize[0]; }
  unsigned                getMaxBTSizeIChroma()                                                     const { return m_maxBTSize[2]; }
  void                    setMaxTTSize(unsigned    maxTTSize,
                                       unsigned    maxTTSizeI,
                                       unsigned    maxTTSizeC)
                                                                                                          { m_maxTTSize[1] = maxTTSize; m_maxTTSize[0] = maxTTSizeI; m_maxTTSize[2] = maxTTSizeC; }
  unsigned                getMaxTTSize()                                                            const { return m_maxTTSize[1]; }
  unsigned                getMaxTTSizeI()                                                           const { return m_maxTTSize[0]; }
  unsigned                getMaxTTSizeIChroma()                                                     const { return m_maxTTSize[2]; }
  unsigned*               getMinQTSizes()                                                          const { return (unsigned *)m_minQT;                }
  unsigned*               getMaxMTTHierarchyDepths()                                               const { return (unsigned *)m_maxMTTHierarchyDepth; }
  unsigned*               getMaxBTSizes()                                                          const { return (unsigned *)m_maxBTSize;            }
  unsigned*               getMaxTTSizes()                                                          const { return (unsigned *)m_maxTTSize;            }
  void                    setIDRRefParamListPresent(bool b)                             { m_idrRefParamList = b; }
  bool                    getIDRRefParamListPresent()                             const { return m_idrRefParamList; }
  void                    setUseDualITree(bool b) { m_dualITree = b; }
  bool                    getUseDualITree()                                      const { return m_dualITree; }

  void                    setMaxCUWidth( uint32_t u )                                                         { m_uiMaxCUWidth = u;                                                  }
  uint32_t                    getMaxCUWidth() const                                                           { return  m_uiMaxCUWidth;                                              }
  void                    setMaxCUHeight( uint32_t u )                                                        { m_uiMaxCUHeight = u;                                                 }
  uint32_t                    getMaxCUHeight() const                                                          { return  m_uiMaxCUHeight;                                             }
#if !JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
  void                    setMaxCodingDepth( uint32_t u )                                                     { m_uiMaxCodingDepth = u;                                              }
  uint32_t                    getMaxCodingDepth() const                                                       { return  m_uiMaxCodingDepth;                                          }
#endif
  bool                    getTransformSkipEnabledFlag() const                                                 { return m_transformSkipEnabledFlag;                                   }
  void                    setTransformSkipEnabledFlag( bool b )                                               { m_transformSkipEnabledFlag = b;                                      }
#if JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
  uint32_t                getLog2MaxTransformSkipBlockSize() const                                            { return m_log2MaxTransformSkipBlockSize;                              }
  void                    setLog2MaxTransformSkipBlockSize(uint32_t u)                                        { m_log2MaxTransformSkipBlockSize = u;                                 }
#endif
#if JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
  bool                    getBDPCMEnabledFlag() const                                                         { return m_BDPCMEnabledFlag;                                           }
  void                    setBDPCMEnabledFlag( bool b )                                                       { m_BDPCMEnabledFlag = b;                                              }
#else
  int                     getBDPCMEnabled() const                                                             { return m_BDPCMEnabled;                                               }
  void                    setBDPCMEnabled(int val)                                                            { m_BDPCMEnabled = val;                                                }
#endif
  void                    setBitsForPOC( uint32_t u )                                                         { m_uiBitsForPOC = u;                                                  }
  uint32_t                    getBitsForPOC() const                                                           { return m_uiBitsForPOC;                                               }
#if JVET_P0116_POC_MSB
  void                    setPocMsbFlag(bool b)                                                               { m_pocMsbFlag = b;                                                    }
  bool                    getPocMsbFlag() const                                                               { return m_pocMsbFlag;                                                 }
  void                    setPocMsbLen(uint32_t u)                                                            { m_pocMsbLen = u;                                                     }
  uint32_t                getPocMsbLen() const                                                                { return m_pocMsbLen;                                                  }
#endif
#if JVET_Q0400_EXTRA_BITS
  void                    setNumExtraPHBitsBytes(int i)                                                       { m_numExtraPHBitsBytes = i;                                           }
  int                     getNumExtraPHBitsBytes() const                                                      { return m_numExtraPHBitsBytes;                                        }
  void                    setNumExtraSHBitsBytes(int i)                                                       { m_numExtraSHBitsBytes = i;                                           }
  int                     getNumExtraSHBitsBytes() const                                                      { return m_numExtraSHBitsBytes;                                        }
  void                    setExtraPHBitPresentFlags(const std::vector<bool> &b)                               { m_extraPHBitPresentFlag = b;                                         }
  const std::vector<bool> getExtraPHBitPresentFlags() const                                                   { return m_extraPHBitPresentFlag;                                      }
  void                    setExtraSHBitPresentFlags(const std::vector<bool> &b)                               { m_extraSHBitPresentFlag = b;                                         }
  const std::vector<bool> getExtraSHBitPresentFlags() const                                                   { return m_extraSHBitPresentFlag;                                      }
#endif
  void                    setNumReorderPics(int i, uint32_t tlayer)                                           { m_numReorderPics[tlayer] = i;                                        }
  int                     getNumReorderPics(uint32_t tlayer) const                                            { return m_numReorderPics[tlayer];                                     }
  void                    createRPLList0(int numRPL);
  void                    createRPLList1(int numRPL);
  const RPLList*          getRPLList( bool b ) const                                                          { return b==1 ? &m_RPLList1 : &m_RPLList0;                             }
  RPLList*                getRPLList( bool b )                                                                { return b==1 ? &m_RPLList1 : &m_RPLList0;                             }
  uint32_t                getNumRPL( bool b ) const                                                           { return b==1 ? m_numRPL1   : m_numRPL0;                               }
  const RPLList*          getRPLList0() const                                                                 { return &m_RPLList0;                                                  }
  RPLList*                getRPLList0()                                                                       { return &m_RPLList0;                                                  }
  const RPLList*          getRPLList1() const                                                                 { return &m_RPLList1;                                                  }
  RPLList*                getRPLList1()                                                                       { return &m_RPLList1;                                                  }
  uint32_t                getNumRPL0() const                                                                  { return m_numRPL0;                                                    }
  uint32_t                getNumRPL1() const                                                                  { return m_numRPL1;                                                    }
  void                    setRPL1CopyFromRPL0Flag(bool isCopy)                                                { m_rpl1CopyFromRpl0Flag = isCopy;                                     }
  bool                    getRPL1CopyFromRPL0Flag() const                                                     { return m_rpl1CopyFromRpl0Flag;                                       }
  bool                    getRPL1IdxPresentFlag() const                                                       { return m_rpl1IdxPresentFlag;                                         }
  void                    setAllActiveRplEntriesHasSameSignFlag(bool isAllSame)                               { m_allRplEntriesHasSameSignFlag = isAllSame;                          }
  bool                    getAllActiveRplEntriesHasSameSignFlag() const                                       { return m_allRplEntriesHasSameSignFlag;                               }
  bool                    getLongTermRefsPresent() const                                                  { return m_bLongTermRefsPresent;                                       }
  void                    setLongTermRefsPresent(bool b)                                                  { m_bLongTermRefsPresent=b;                                            }
  bool                    getSPSTemporalMVPEnabledFlag() const                                            { return m_SPSTemporalMVPEnabledFlag;                                  }
  void                    setSPSTemporalMVPEnabledFlag(bool b)                                            { m_SPSTemporalMVPEnabledFlag=b;                                       }
  void                    setLog2MaxTbSize( uint32_t u )                                                  { m_log2MaxTbSize = u;                                                 }
  uint32_t                getLog2MaxTbSize() const                                                        { return  m_log2MaxTbSize;                                             }
  uint32_t                getMaxTbSize() const                                                            { return  1 << m_log2MaxTbSize;                                        }
  // Bit-depth
  int                     getBitDepth(ChannelType type) const                                             { return m_bitDepths.recon[type];                                      }
  void                    setBitDepth(ChannelType type, int u )                                           { m_bitDepths.recon[type] = u;                                         }
  const BitDepths&        getBitDepths() const                                                            { return m_bitDepths;                                                  }

#if JVET_Q0151_Q0205_ENTRYPOINTS
  bool                    getEntropyCodingSyncEnabledFlag() const                                         { return m_entropyCodingSyncEnabledFlag;                               }
  void                    setEntropyCodingSyncEnabledFlag(bool val)                                       { m_entropyCodingSyncEnabledFlag = val;                                }
  bool                    getEntropyCodingSyncEntryPointsPresentFlag() const                              { return m_entropyCodingSyncEntryPointPresentFlag;                     }
  void                    setEntropyCodingSyncEntryPointsPresentFlag(bool val)                            { m_entropyCodingSyncEntryPointPresentFlag = val;                      }
#endif
  int                     getMaxLog2TrDynamicRange(ChannelType channelType) const                         { return getSpsRangeExtension().getExtendedPrecisionProcessingFlag() ? std::max<int>(15, int(m_bitDepths.recon[channelType] + 6)) : 15; }

  int                     getDifferentialLumaChromaBitDepth() const                                       { return int(m_bitDepths.recon[CHANNEL_TYPE_LUMA]) - int(m_bitDepths.recon[CHANNEL_TYPE_CHROMA]); }
  int                     getQpBDOffset(ChannelType type) const                                           { return m_qpBDOffset[type];                                           }
  void                    setQpBDOffset(ChannelType type, int i)                                          { m_qpBDOffset[type] = i;                                              }
  int                     getMinQpPrimeTsMinus4(ChannelType type) const                                         { return m_minQpMinus4[type];                                           }
  void                    setMinQpPrimeTsMinus4(ChannelType type, int i)                                        { m_minQpMinus4[type] = i;                                              }

  void                    setSAOEnabledFlag(bool bVal)                                                    { m_saoEnabledFlag = bVal;                                                    }
  bool                    getSAOEnabledFlag() const                                                       { return m_saoEnabledFlag;                                                    }

  bool                    getALFEnabledFlag() const                                                       { return m_alfEnabledFlag; }
  void                    setALFEnabledFlag( bool b )                                                     { m_alfEnabledFlag = b; }
#if JVET_Q0795_CCALF
bool                    getCCALFEnabledFlag() const                                                       { return m_ccalfEnabledFlag; }
void                    setCCALFEnabledFlag( bool b )                                                     { m_ccalfEnabledFlag = b; }
#endif
  void                    setJointCbCrEnabledFlag(bool bVal)                                              { m_JointCbCrEnabledFlag = bVal; }
  bool                    getJointCbCrEnabledFlag() const                                                 { return m_JointCbCrEnabledFlag; }

  bool                    getSBTMVPEnabledFlag() const                                                    { return m_sbtmvpEnabledFlag; }
  void                    setSBTMVPEnabledFlag(bool b)                                                    { m_sbtmvpEnabledFlag = b; }

  void                    setBDOFEnabledFlag(bool b)                                                      { m_bdofEnabledFlag = b; }
  bool                    getBDOFEnabledFlag() const                                                      { return m_bdofEnabledFlag; }

  bool                    getFpelMmvdEnabledFlag() const                                                  { return m_fpelMmvdEnabledFlag; }
  void                    setFpelMmvdEnabledFlag( bool b )                                                { m_fpelMmvdEnabledFlag = b;    }
  bool                    getUseDMVR()const                                                               { return m_DMVR; }
  void                    setUseDMVR(bool b)                                                              { m_DMVR = b;    }
  bool                    getUseMMVD()const                                                               { return m_MMVD; }
  void                    setUseMMVD(bool b)                                                              { m_MMVD = b;    }
  bool                    getBdofControlPresentFlag()const                                                { return m_BdofControlPresentFlag; }
  void                    setBdofControlPresentFlag(bool b)                                               { m_BdofControlPresentFlag = b;    }

  bool                    getDmvrControlPresentFlag()const                                                { return m_DmvrControlPresentFlag; }
  void                    setDmvrControlPresentFlag(bool b)                                               { m_DmvrControlPresentFlag = b;    }

  bool                    getProfControlPresentFlag()const                                                { return m_ProfControlPresentFlag; }
  void                    setProfControlPresentFlag(bool b)                                               { m_ProfControlPresentFlag = b;    }
  uint32_t                getMaxTLayers() const                                                           { return m_uiMaxTLayers; }
  void                    setMaxTLayers( uint32_t uiMaxTLayers )                                          { CHECK( uiMaxTLayers > MAX_TLAYER, "Invalid number T-layers" ); m_uiMaxTLayers = uiMaxTLayers; }

#if JVET_P0117_PTL_SCALABILITY
  bool                    getPtlDpbHrdParamsPresentFlag()  const                                          { return m_ptlDpbHrdParamsPresentFlag;     }
  void                    setPtlDpbHrdParamsPresentFlag(bool b)                                           {        m_ptlDpbHrdParamsPresentFlag = b; }
  bool                    getSubLayerDpbParamsFlag()  const                                               { return m_SubLayerDpbParamsFlag;          }
  void                    setSubLayerDpbParamsFlag(bool b)                                                {        m_SubLayerDpbParamsFlag = b;      }
#endif
  bool                    getTemporalIdNestingFlag() const                                                { return m_bTemporalIdNestingFlag;                                     }
  void                    setTemporalIdNestingFlag( bool bValue )                                         { m_bTemporalIdNestingFlag = bValue;                                   }

  bool                    getScalingListFlag() const                                                      { return m_scalingListEnabledFlag;                                     }
  void                    setScalingListFlag( bool b )                                                    { m_scalingListEnabledFlag  = b;                                       }
  void                    setLoopFilterAcrossVirtualBoundariesDisabledFlag(bool b)                        { m_loopFilterAcrossVirtualBoundariesDisabledFlag = b;                 }
  bool                    getLoopFilterAcrossVirtualBoundariesDisabledFlag() const                        { return m_loopFilterAcrossVirtualBoundariesDisabledFlag;              }
  void                    setNumVerVirtualBoundaries(unsigned u)                                          { m_numVerVirtualBoundaries = u;                                       }
  unsigned                getNumVerVirtualBoundaries() const                                              { return m_numVerVirtualBoundaries;                                    }
  void                    setNumHorVirtualBoundaries(unsigned u)                                          { m_numHorVirtualBoundaries = u;                                       }
  unsigned                getNumHorVirtualBoundaries() const                                              { return m_numHorVirtualBoundaries;                                    }
  void                    setVirtualBoundariesPosX(unsigned u, unsigned idx)                              { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;    }
  unsigned                getVirtualBoundariesPosX(unsigned idx) const                                    { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx]; }
  void                    setVirtualBoundariesPosY(unsigned u, unsigned idx)                              { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;    }
  unsigned                getVirtualBoundariesPosY(unsigned idx) const                                    { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx]; }
  uint32_t                getMaxDecPicBuffering(uint32_t tlayer) const                                    { return m_uiMaxDecPicBuffering[tlayer];                               }
  void                    setMaxDecPicBuffering( uint32_t ui, uint32_t tlayer )                           { CHECK(tlayer >= MAX_TLAYER, "Invalid T-layer"); m_uiMaxDecPicBuffering[tlayer] = ui;    }
  uint32_t                getMaxLatencyIncreasePlus1(uint32_t tlayer) const                               { return m_uiMaxLatencyIncreasePlus1[tlayer];                          }
  void                    setMaxLatencyIncreasePlus1( uint32_t ui , uint32_t tlayer)                      { m_uiMaxLatencyIncreasePlus1[tlayer] = ui;                            }

  void                    setAffineAmvrEnabledFlag( bool val )                                            { m_affineAmvrEnabledFlag = val;                                       }
  bool                    getAffineAmvrEnabledFlag() const                                                { return m_affineAmvrEnabledFlag;                                      }
  TimingInfo*             getTimingInfo()                                                                 { return &m_timingInfo; }
  const TimingInfo*       getTimingInfo() const                                                           { return &m_timingInfo; }
  bool                    getHrdParametersPresentFlag() const                                             { return m_hrdParametersPresentFlag; }
  void                    setHrdParametersPresentFlag(bool b)                                             { m_hrdParametersPresentFlag = b; }
  HRDParameters*          getHrdParameters()                                                              { return &m_hrdParameters; }
  const HRDParameters*    getHrdParameters() const                                                        { return &m_hrdParameters; }
#if JVET_Q0042_VUI
  bool                    getFieldSeqFlag() const                                                         { return m_fieldSeqFlag;                         }
  void                    setFieldSeqFlag(bool i)                                                         { m_fieldSeqFlag = i;                            }
#endif
  bool                    getVuiParametersPresentFlag() const                                             { return m_vuiParametersPresentFlag;                                   }
  void                    setVuiParametersPresentFlag(bool b)                                             { m_vuiParametersPresentFlag = b;                                      }
  VUI*                    getVuiParameters()                                                              { return &m_vuiParameters;                                             }
  const VUI*              getVuiParameters() const                                                        { return &m_vuiParameters;                                             }
  const ProfileTierLevel* getProfileTierLevel() const                                                     { return &m_profileTierLevel; }
  ProfileTierLevel*       getProfileTierLevel()                                                           { return &m_profileTierLevel; }

  const SPSRExt&          getSpsRangeExtension() const                                                    { return m_spsRangeExtension;                                          }
  SPSRExt&                getSpsRangeExtension()                                                          { return m_spsRangeExtension;                                          }

  void                    setWrapAroundEnabledFlag(bool b)                                                { m_wrapAroundEnabledFlag = b;                                         }
  bool                    getWrapAroundEnabledFlag() const                                                { return m_wrapAroundEnabledFlag;                                      }
  void                    setWrapAroundOffset(unsigned offset)                                            { m_wrapAroundOffset = offset;                                         }
  unsigned                getWrapAroundOffset() const                                                     { return m_wrapAroundOffset;                                           }
  void                    setUseLmcs(bool b)                                                              { m_lmcsEnabled = b;                                                   }
  bool                    getUseLmcs() const                                                              { return m_lmcsEnabled;                                                }
  void                    setIBCFlag(unsigned IBCFlag)                                                    { m_IBCFlag = IBCFlag; }
  unsigned                getIBCFlag() const                                                              { return m_IBCFlag; }
  void                    setUseColorTrans(bool value) { m_useColorTrans = value; }
  bool                    getUseColorTrans() const { return m_useColorTrans; }
  void                    setPLTMode(unsigned PLTMode)                                                    { m_PLTMode = PLTMode; }
  unsigned                getPLTMode() const                                                              { return m_PLTMode; }
  void                    setUseSBT( bool b )                                                             { m_SBT = b; }
  bool                    getUseSBT() const                                                               { return m_SBT; }
  void                    setUseISP( bool b )                                                             { m_ISP = b; }
  bool                    getUseISP() const                                                               { return m_ISP; }

  void      setAMVREnabledFlag    ( bool b )                                        { m_AMVREnabledFlag = b; }
  bool      getAMVREnabledFlag    ()                                      const     { return m_AMVREnabledFlag; }
  void      setUseAffine          ( bool b )                                        { m_Affine = b; }
  bool      getUseAffine          ()                                      const     { return m_Affine; }
  void      setUseAffineType      ( bool b )                                        { m_AffineType = b; }
  bool      getUseAffineType      ()                                      const     { return m_AffineType; }
  void      setUsePROF            ( bool b )                                        { m_PROF = b; }
  bool      getUsePROF            ()                                      const     { return m_PROF; }
  void      setUseLMChroma        ( bool b )                                        { m_LMChroma = b; }
  bool      getUseLMChroma        ()                                      const     { return m_LMChroma; }
  void      setHorCollocatedChromaFlag( bool b )                                    { m_horCollocatedChromaFlag = b;    }
  bool      getHorCollocatedChromaFlag()                                  const     { return m_horCollocatedChromaFlag; }
  void      setVerCollocatedChromaFlag( bool b )                                    { m_verCollocatedChromaFlag = b;    }
  bool      getVerCollocatedChromaFlag()                                  const     { return m_verCollocatedChromaFlag; }
  bool      getCclmCollocatedChromaFlag()                                 const     { return m_verCollocatedChromaFlag; }
  void      setUseMTS             ( bool b )                                        { m_MTS = b; }
  bool      getUseMTS             ()                                      const     { return m_MTS; }
  bool      getUseImplicitMTS     ()                                      const     { return m_MTS && !m_IntraMTS; }
  void      setUseIntraMTS        ( bool b )                                        { m_IntraMTS = b; }
  bool      getUseIntraMTS        ()                                      const     { return m_IntraMTS; }
  void      setUseInterMTS        ( bool b )                                        { m_InterMTS = b; }
  bool      getUseInterMTS        ()                                      const     { return m_InterMTS; }
  void      setUseLFNST           ( bool b )                                        { m_LFNST = b; }
  bool      getUseLFNST           ()                                      const     { return m_LFNST; }
  void      setUseSMVD(bool b)                                                      { m_SMVD = b; }
  bool      getUseSMVD()                                                  const     { return m_SMVD; }
  void      setUseBcw             ( bool b )                                        { m_bcw = b; }
  bool      getUseBcw             ()                                      const     { return m_bcw; }
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  void      setLadfEnabled        ( bool b )                                        { m_LadfEnabled = b; }
  bool      getLadfEnabled        ()                                      const     { return m_LadfEnabled; }
  void      setLadfNumIntervals   ( int i )                                         { m_LadfNumIntervals = i; }
  int       getLadfNumIntervals   ()                                      const     { return m_LadfNumIntervals; }
  void      setLadfQpOffset       ( int value, int idx )                            { m_LadfQpOffset[ idx ] = value; }
  int       getLadfQpOffset       ( int idx )                             const     { return m_LadfQpOffset[ idx ]; }
  void      setLadfIntervalLowerBound( int value, int idx )                         { m_LadfIntervalLowerBound[ idx ] = value; }
  int       getLadfIntervalLowerBound( int idx )                          const     { return m_LadfIntervalLowerBound[ idx ]; }
#endif

  void      setUseCiip         ( bool b )                                        { m_ciip = b; }
  bool      getUseCiip         ()                                      const     { return m_ciip; }
#if !JVET_Q0806
  void      setUseTriangle        ( bool b )                                        { m_Triangle = b; }
  bool      getUseTriangle        ()                                      const     { return m_Triangle; }
#else
  void      setUseGeo             ( bool b )                                        { m_Geo = b; }
  bool      getUseGeo             ()                                      const     { return m_Geo; }
#endif
  void      setUseMRL             ( bool b )                                        { m_MRL = b; }
  bool      getUseMRL             ()                                      const     { return m_MRL; }
  void      setUseMIP             ( bool b )                                        { m_MIP = b; }
  bool      getUseMIP             ()                                      const     { return m_MIP; }

  bool      getUseWP              ()                                      const     { return m_useWeightPred; }
  bool      getUseWPBiPred        ()                                      const     { return m_useWeightedBiPred; }
  void      setUseWP              ( bool b )                                        { m_useWeightPred = b; }
  void      setUseWPBiPred        ( bool b )                                        { m_useWeightedBiPred = b; }
  void      setChromaQpMappingTableFromParams(const ChromaQpMappingTableParams &params, const int qpBdOffset)   { m_chromaQpMappingTable.setParams(params, qpBdOffset); }
  void      derivedChromaQPMappingTables()                                          { m_chromaQpMappingTable.derivedChromaQPMappingTables(); }
  const ChromaQpMappingTable& getChromaQpMappingTable()                   const     { return m_chromaQpMappingTable;}
  int       getMappedChromaQpValue(ComponentID compID, int qpVal)         const     { return m_chromaQpMappingTable.getMappedChromaQpValue(compID, qpVal); }
  void setGDREnabledFlag(bool flag) { m_GDREnabledFlag = flag; }
  bool getGDREnabledFlag() const { return m_GDREnabledFlag; }
  void      setSubLayerParametersPresentFlag(bool flag)                             { m_SubLayerCbpParametersPresentFlag = flag; }
  bool      getSubLayerParametersPresentFlag()                            const     { return m_SubLayerCbpParametersPresentFlag;  }

  bool      getRprEnabledFlag()                                           const     { return m_rprEnabledFlag; }
  void      setRprEnabledFlag( bool flag )                                          { m_rprEnabledFlag = flag; }
  bool      getInterLayerPresentFlag()                                        const { return m_interLayerPresentFlag; }
  void      setInterLayerPresentFlag( bool b )                                      { m_interLayerPresentFlag = b; }

#if JVET_Q0297_MER
  uint32_t  getLog2ParallelMergeLevelMinus2() const { return m_log2ParallelMergeLevelMinus2; }
  void      setLog2ParallelMergeLevelMinus2(uint32_t mrgLevel) { m_log2ParallelMergeLevelMinus2 = mrgLevel; }
#endif
#if JVET_Q0179_SCALING_WINDOW_SIZE_CONSTRAINT
  void          setPPSValidFlag(int i, bool b) { m_ppsValidFlag[i] = b; }
  bool          getPPSValidFlag(int i)         { return m_ppsValidFlag[i]; }
  void          setScalingWindowSizeInPPS(int i, int scWidth, int scHeight) { m_scalingWindowSizeInPPS[i].width = scWidth; m_scalingWindowSizeInPPS[i].height = scHeight;}
  const Size&   getScalingWindowSizeInPPS(int i)                            { return m_scalingWindowSizeInPPS[i]; }
#endif
};


/// Reference Picture Lists class


/// PPS RExt class
class PPSRExt // TODO: remove
{
private:
  bool             m_crossComponentPredictionEnabledFlag;

#if !JVET_Q0441_SAO_MOD_12_BIT
  uint32_t             m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];
#endif

public:
  PPSRExt();

  bool settingsDifferFromDefaults(const bool bTransformSkipEnabledFlag) const
  {
#if JVET_Q0441_SAO_MOD_12_BIT
    return (getCrossComponentPredictionEnabledFlag() );
#else
    return (getCrossComponentPredictionEnabledFlag() )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA) !=0 )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA) !=0 );
#endif
  }

  bool                   getCrossComponentPredictionEnabledFlag() const                   { return m_crossComponentPredictionEnabledFlag;   }
  void                   setCrossComponentPredictionEnabledFlag(bool value)               { m_crossComponentPredictionEnabledFlag = value;  }

#if !JVET_Q0441_SAO_MOD_12_BIT
  // Now: getPpsRangeExtension().getLog2SaoOffsetScale and getPpsRangeExtension().setLog2SaoOffsetScale
  uint32_t                   getLog2SaoOffsetScale(ChannelType type) const                    { return m_log2SaoOffsetScale[type];             }
  void                   setLog2SaoOffsetScale(ChannelType type, uint32_t uiBitShift)         { m_log2SaoOffsetScale[type] = uiBitShift;       }
#endif
};


/// PPS class
class PPS
{
private:
  int              m_PPSId;                    // pic_parameter_set_id
  int              m_SPSId;                    // seq_parameter_set_id
  int              m_picInitQPMinus26;
  bool             m_useDQP;
#if JVET_Q0420_PPS_CHROMA_TOOL_FLAG
  bool             m_usePPSChromaTool;
#endif
  bool             m_bSliceChromaQpFlag;       // slicelevel_chroma_qp_flag

  int              m_layerId;
  int              m_temporalId;

  // access channel

  int              m_chromaCbQpOffset;
  int              m_chromaCrQpOffset;
  bool             m_chromaJointCbCrQpOffsetPresentFlag;
  int              m_chromaCbCrQpOffset;

  // Chroma QP Adjustments
  int              m_chromaQpOffsetListLen; // size (excludes the null entry used in the following array).
  ChromaQpAdj      m_ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  uint32_t             m_numRefIdxL0DefaultActive;
  uint32_t             m_numRefIdxL1DefaultActive;

  bool             m_rpl1IdxPresentFlag;

  bool             m_bUseWeightPred;                    //!< Use of Weighting Prediction (P_SLICE)
  bool             m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)
  bool             m_OutputFlagPresentFlag;             //!< Indicates the presence of output_flag in slice header
  uint8_t          m_numSubPics;                        //!< number of sub-pictures used - must match SPS
#if JVET_Q0119_CLEANUPS
  bool             m_subPicIdMappingInPpsFlag;
#else
  bool             m_subPicIdSignallingPresentFlag;     //!< indicates the presence of sub-picture ID signalling in the PPS
#endif
  uint32_t         m_subPicIdLen;                       //!< sub-picture ID length in bits
  uint8_t          m_subPicId[MAX_NUM_SUB_PICS];        //!< sub-picture ID for each sub-picture in the sequence
  bool             m_noPicPartitionFlag;                //!< no picture partitioning flag - single slice, single tile
  uint8_t          m_log2CtuSize;                       //!< log2 of the CTU size - required to match corresponding value in SPS
  uint8_t          m_ctuSize;                           //!< CTU size
  uint32_t         m_picWidthInCtu;                     //!< picture width in units of CTUs
  uint32_t         m_picHeightInCtu;                    //!< picture height in units of CTUs
  uint32_t         m_numExpTileCols;                    //!< number of explicitly specified tile columns
  uint32_t         m_numExpTileRows;                    //!< number of explicitly specified tile rows
  uint32_t         m_numTileCols;                       //!< number of tile columns
  uint32_t         m_numTileRows;                       //!< number of tile rows
  std::vector<uint32_t> m_tileColWidth;                 //!< tile column widths in units of CTUs
  std::vector<uint32_t> m_tileRowHeight;                //!< tile row heights in units of CTUs
  std::vector<uint32_t> m_tileColBd;                    //!< tile column left-boundaries in units of CTUs
  std::vector<uint32_t> m_tileRowBd;                    //!< tile row top-boundaries in units of CTUs  
  std::vector<uint32_t> m_ctuToTileCol;                 //!< mapping between CTU horizontal address and tile column index
  std::vector<uint32_t> m_ctuToTileRow;                 //!< mapping between CTU vertical address and tile row index
  bool             m_rectSliceFlag;                     //!< rectangular slice flag  
  bool             m_singleSlicePerSubPicFlag;          //!< single slice per sub-picture flag
  std::vector<uint32_t> m_ctuToSubPicIdx;               //!< mapping between CTU and Sub-picture index
  uint32_t         m_numSlicesInPic;                    //!< number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool             m_tileIdxDeltaPresentFlag;           //!< tile index delta present flag
  std::vector<RectSlice> m_rectSlices;                  //!< list of rectangular slice signalling parameters
  std::vector<SliceMap>  m_sliceMap;                    //!< list of CTU maps for each slice in the picture
#if JVET_O1143_SUBPIC_BOUNDARY
  std::vector<SubPic>      m_subPics;                   //!< list of subpictures in the picture
#endif
  bool             m_loopFilterAcrossTilesEnabledFlag;  //!< loop filtering applied across tiles flag
  bool             m_loopFilterAcrossSlicesEnabledFlag; //!< loop filtering applied across slices flag
#if !JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
  int              m_log2MaxTransformSkipBlockSize;
#endif
#if !JVET_Q0151_Q0205_ENTRYPOINTS
  bool             m_entropyCodingSyncEnabledFlag;      //!< Indicates the presence of wavefronts
#endif

#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
  bool              m_constantSliceHeaderParamsEnabledFlag;
  int               m_PPSDepQuantEnabledIdc;
  int               m_PPSRefPicListSPSIdc0;
  int               m_PPSRefPicListSPSIdc1;
  int               m_PPSMvdL1ZeroIdc;
  int               m_PPSCollocatedFromL0Idc;
  uint32_t          m_PPSSixMinusMaxNumMergeCandPlus1;
#if !JVET_Q0806
  uint32_t          m_PPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1;
#else
  uint32_t          m_PPSMaxNumMergeCandMinusMaxNumGeoCandPlus1;
#endif
#endif

  bool             m_cabacInitPresentFlag;

  bool             m_pictureHeaderExtensionPresentFlag;   //< picture header extension flags present in picture headers or not
  bool             m_sliceHeaderExtensionPresentFlag;
  bool             m_deblockingFilterControlPresentFlag;
  bool             m_deblockingFilterOverrideEnabledFlag;
  bool             m_ppsDeblockingFilterDisabledFlag;
  int              m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  int              m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
  int              m_deblockingFilterCbBetaOffsetDiv2;    //< beta offset for Cb deblocking filter
  int              m_deblockingFilterCbTcOffsetDiv2;      //< tc offset for Cb deblocking filter
  int              m_deblockingFilterCrBetaOffsetDiv2;    //< beta offset for Cr deblocking filter
  int              m_deblockingFilterCrTcOffsetDiv2;      //< tc offset for Cr deblocking filter
#endif
  bool             m_listsModificationPresentFlag;

#if JVET_Q0819_PH_CHANGES
  bool             m_rplInfoInPhFlag;
  bool             m_dbfInfoInPhFlag;
  bool             m_saoInfoInPhFlag;
  bool             m_alfInfoInPhFlag;
  bool             m_wpInfoInPhFlag;
  bool             m_qpDeltaInfoInPhFlag;
#endif
#if SPS_ID_CHECK
  bool             m_mixedNaluTypesInPicFlag;
#endif

  uint32_t         m_picWidthInLumaSamples;
  uint32_t         m_picHeightInLumaSamples;
  Window           m_conformanceWindow;
  Window           m_scalingWindow;

  PPSRExt          m_ppsRangeExtension;

public:
  PreCalcValues   *pcv;

public:
                         PPS();
  virtual                ~PPS();

  int                    getPPSId() const                                                 { return m_PPSId;                               }
  void                   setPPSId(int i)                                                  { m_PPSId = i;                                  }
  int                    getSPSId() const                                                 { return m_SPSId;                               }
  void                   setSPSId(int i)                                                  { m_SPSId = i;                                  }

  void                   setTemporalId( int i )                                           { m_temporalId = i;                             }
  int                    getTemporalId()                                            const { return m_temporalId;                          }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }

  int                    getPicInitQPMinus26() const                                      { return  m_picInitQPMinus26;                   }
  void                   setPicInitQPMinus26( int i )                                     { m_picInitQPMinus26 = i;                       }
  bool                   getUseDQP() const                                                { return m_useDQP;                              }
  void                   setUseDQP( bool b )                                              { m_useDQP   = b;                               }
#if JVET_Q0420_PPS_CHROMA_TOOL_FLAG
  bool                   getPPSChromaToolFlag()                                     const { return  m_usePPSChromaTool;                   }
  void                   setPPSChromaToolFlag(bool b)                                     { m_usePPSChromaTool = b;                       }
#endif
  bool                   getSliceChromaQpFlag() const                                     { return  m_bSliceChromaQpFlag;                 }
  void                   setSliceChromaQpFlag( bool b )                                   { m_bSliceChromaQpFlag = b;                     }


  bool                   getJointCbCrQpOffsetPresentFlag() const                          { return m_chromaJointCbCrQpOffsetPresentFlag;   }
  void                   setJointCbCrQpOffsetPresentFlag(bool b)                          { m_chromaJointCbCrQpOffsetPresentFlag = b;      } 

  void                   setQpOffset(ComponentID compID, int i )
  {
    if      (compID==COMPONENT_Cb)
    {
      m_chromaCbQpOffset = i;
    }
    else if (compID==COMPONENT_Cr)
    {
      m_chromaCrQpOffset = i;
    }
    else if (compID==JOINT_CbCr)
    {
      m_chromaCbCrQpOffset = i;
    }
    else
    {
      THROW( "Invalid chroma QP offset" );
    }
  }
  int                    getQpOffset(ComponentID compID) const
  {
    return (compID==COMPONENT_Y) ? 0 : (compID==COMPONENT_Cb ? m_chromaCbQpOffset : compID==COMPONENT_Cr ? m_chromaCrQpOffset : m_chromaCbCrQpOffset );
  }

  bool                   getCuChromaQpOffsetListEnabledFlag() const                       { return getChromaQpOffsetListLen()>0;            }
  int                    getChromaQpOffsetListLen() const                                 { return m_chromaQpOffsetListLen;                 }
  void                   clearChromaQpOffsetList()                                        { m_chromaQpOffsetListLen = 0;                    }

  const ChromaQpAdj&     getChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1 ) const
  {
    CHECK(cuChromaQpOffsetIdxPlus1 >= m_chromaQpOffsetListLen+1, "Invalid chroma QP offset");
    return m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }

  void                   setChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1, int cbOffset, int crOffset, int jointCbCrOffset )
  {
    CHECK(cuChromaQpOffsetIdxPlus1 == 0 || cuChromaQpOffsetIdxPlus1 > MAX_QP_OFFSET_LIST_SIZE, "Invalid chroma QP offset");
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CbOffset = cbOffset; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CrOffset = crOffset;
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.JointCbCrOffset = jointCbCrOffset;
    m_chromaQpOffsetListLen = std::max(m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
  }

  void                   setNumRefIdxL0DefaultActive(uint32_t ui)                             { m_numRefIdxL0DefaultActive=ui;                }
  uint32_t                   getNumRefIdxL0DefaultActive() const                              { return m_numRefIdxL0DefaultActive;            }
  void                   setNumRefIdxL1DefaultActive(uint32_t ui)                             { m_numRefIdxL1DefaultActive=ui;                }
  uint32_t                   getNumRefIdxL1DefaultActive() const                              { return m_numRefIdxL1DefaultActive;            }

  void                   setRpl1IdxPresentFlag(bool isPresent)                            { m_rpl1IdxPresentFlag = isPresent;             }
  uint32_t               getRpl1IdxPresentFlag() const                                    { return m_rpl1IdxPresentFlag;                  }

  bool                   getUseWP() const                                                 { return m_bUseWeightPred;                      }
  bool                   getWPBiPred() const                                              { return m_useWeightedBiPred;                   }
  void                   setUseWP( bool b )                                               { m_bUseWeightPred = b;                         }
  void                   setWPBiPred( bool b )                                            { m_useWeightedBiPred = b;                      }

  void                   setOutputFlagPresentFlag( bool b )                               { m_OutputFlagPresentFlag = b;                  }
  bool                   getOutputFlagPresentFlag() const                                 { return m_OutputFlagPresentFlag;               }
  void                   setNumSubPics( uint8_t u )                                       { m_numSubPics = u;                             }
  uint8_t                getNumSubPics( ) const                                           { return  m_numSubPics;                         }
#if JVET_Q0119_CLEANUPS
  void                   setSubPicIdMappingInPpsFlag( bool b )                            { m_subPicIdMappingInPpsFlag = b;               }
  bool                   getSubPicIdMappingInPpsFlag() const                              { return m_subPicIdMappingInPpsFlag;            }
#else
  void                   setSubPicIdSignallingPresentFlag( bool b )                       { m_subPicIdSignallingPresentFlag = b;          }
  bool                   getSubPicIdSignallingPresentFlag() const                         { return  m_subPicIdSignallingPresentFlag;      }
#endif
  void                   setSubPicIdLen( uint32_t u )                                     { m_subPicIdLen = u;                            }
  uint32_t               getSubPicIdLen() const                                           { return  m_subPicIdLen;                        }
  void                   setSubPicId( int i, uint8_t u )                                  { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicId[i] = u;     }
  uint8_t                getSubPicId( int i ) const                                       { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicId[i]; }
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  uint32_t               getSubPicIdxFromSubPicId( uint32_t subPicId ) const;
#endif
  void                   setNoPicPartitionFlag( bool b )                                  { m_noPicPartitionFlag = b;                     }
  bool                   getNoPicPartitionFlag( ) const                                   { return  m_noPicPartitionFlag;                 }
  void                   setLog2CtuSize( uint8_t u )                                      { m_log2CtuSize = u; m_ctuSize = 1 << m_log2CtuSize; 
                                                                                            m_picWidthInCtu = (m_picWidthInLumaSamples  + m_ctuSize - 1) / m_ctuSize;  
                                                                                            m_picHeightInCtu = (m_picHeightInLumaSamples  + m_ctuSize - 1) / m_ctuSize; }
  uint8_t                getLog2CtuSize( ) const                                          { return  m_log2CtuSize;                        }
  uint8_t                getCtuSize( ) const                                              { return  m_ctuSize;                            }
  uint8_t                getPicWidthInCtu( ) const                                        { return  m_picWidthInCtu;                      }
  uint8_t                getPicHeightInCtu( ) const                                       { return  m_picHeightInCtu;                     }
  void                   setNumExpTileColumns( uint32_t u )                               { m_numExpTileCols = u;                         }
  uint32_t               getNumExpTileColumns( ) const                                    { return  m_numExpTileCols;                     }
  void                   setNumExpTileRows( uint32_t u )                                  { m_numExpTileRows = u;                         }
  uint32_t               getNumExpTileRows( ) const                                       { return  m_numExpTileRows;                     }
  void                   setNumTileColumns( uint32_t u )                                  { m_numTileCols = u;                            }
  uint32_t               getNumTileColumns( ) const                                       { return  m_numTileCols;                        }
  void                   setNumTileRows( uint32_t u )                                     { m_numTileRows = u;                            }
  uint32_t               getNumTileRows( ) const                                          { return  m_numTileRows;                        }
  uint32_t               getNumTiles( ) const                                             { return  m_numTileCols * m_numTileRows;        }
  void                   setTileColumnWidths( std::vector<uint32_t> widths )              { m_tileColWidth = widths;                      }
  void                   setTileRowHeights( std::vector<uint32_t> heights )               { m_tileRowHeight = heights;                    }
  void                   addTileColumnWidth( uint32_t u )                                 { CHECK( m_tileColWidth.size()  >= MAX_TILE_COLS, "Number of tile columns exceeds valid range" ); m_tileColWidth.push_back(u);    }
  void                   addTileRowHeight( uint32_t u )                                   { CHECK( m_tileRowHeight.size() >= MAX_TILE_ROWS, "Number of tile rows exceeds valid range" );    m_tileRowHeight.push_back(u);   }
  uint32_t               getTileColumnWidth( int idx ) const                              { CHECK( idx >= m_tileColWidth.size(), "Tile column index exceeds valid range" );                 return  m_tileColWidth[idx];    }
  uint32_t               getTileRowHeight( int idx ) const                                { CHECK( idx >= m_tileRowHeight.size(), "Tile row index exceeds valid range" );                   return  m_tileRowHeight[idx];   }  
  uint32_t               getTileColumnBd( int idx ) const                                 { CHECK( idx >= m_tileColBd.size(), "Tile column index exceeds valid range" );                    return  m_tileColBd[idx];       }
  uint32_t               getTileRowBd( int idx ) const                                    { CHECK( idx >= m_tileRowBd.size(), "Tile row index exceeds valid range" );                       return  m_tileRowBd[idx];       }
  uint32_t               ctuToTileCol( int ctuX ) const                                   { CHECK( ctuX >= m_ctuToTileCol.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileCol[ctuX];                  }
  uint32_t               ctuToTileRow( int ctuY ) const                                   { CHECK( ctuY >= m_ctuToTileRow.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileRow[ctuY];                  }
  uint32_t               ctuToTileColBd( int ctuX ) const                                 { return  getTileColumnBd(ctuToTileCol( ctuX ));                                                                                  }
  uint32_t               ctuToTileRowBd( int ctuY ) const                                 { return  getTileRowBd(ctuToTileRow( ctuY ));                                                                                     }
  bool                   ctuIsTileColBd( int ctuX ) const                                 { return  ctuX == ctuToTileColBd( ctuX );                                                                                         }
  bool                   ctuIsTileRowBd( int ctuY ) const                                 { return  ctuY == ctuToTileRowBd( ctuY );                                                                                         }
  uint32_t               getTileIdx( uint32_t ctuX, uint32_t ctuY ) const                 { return (ctuToTileRow( ctuY ) * getNumTileColumns()) + ctuToTileCol( ctuX );                                                     }
  uint32_t               getTileIdx( uint32_t ctuRsAddr) const                            { return getTileIdx( ctuRsAddr % m_picWidthInCtu,  ctuRsAddr / m_picWidthInCtu );                                                 }
  uint32_t               getTileIdx( const Position& pos ) const                          { return getTileIdx( pos.x / m_ctuSize, pos.y / m_ctuSize );                                                                      }
  void                   setRectSliceFlag( bool b )                                       { m_rectSliceFlag = b;                                                                                                            }
  bool                   getRectSliceFlag( ) const                                        { return  m_rectSliceFlag;                                                                                                        }
  void                   setSingleSlicePerSubPicFlag( bool b )                            { m_singleSlicePerSubPicFlag = b;                                                                                                 }
  bool                   getSingleSlicePerSubPicFlag( ) const                             { return  m_singleSlicePerSubPicFlag;                                                                                             }
#if JVET_Q0817
  uint32_t               getCtuToSubPicIdx( int idx ) const                               { CHECK( idx >= m_ctuToSubPicIdx.size(), "CTU address index exceeds valid range" ); CHECK( getNumSubPics() < 1, "Number of subpicture cannot be 0" ); return  m_ctuToSubPicIdx[ idx ]; }
#else
  uint32_t               getCtuToSubPicIdx( int idx ) const                               { CHECK( idx >= m_ctuToSubPicIdx.size(), "CTU address index exceeds valid range" ); return  m_ctuToSubPicIdx[idx];                }
#endif
  void                   setNumSlicesInPic( uint32_t u )                                  { CHECK( u > MAX_SLICES, "Number of slices in picture exceeds valid range" ); m_numSlicesInPic = u;                               }
  uint32_t               getNumSlicesInPic( ) const                                       { return  m_numSlicesInPic;                                                                                                       }
  void                   setTileIdxDeltaPresentFlag( bool b )                             { m_tileIdxDeltaPresentFlag = b;                                                                                                  }
  bool                   getTileIdxDeltaPresentFlag( ) const                              { return  m_tileIdxDeltaPresentFlag;                                                                                              }
  void                   setSliceWidthInTiles( int idx, uint32_t u )                      { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceWidthInTiles( u );            }
  uint32_t               getSliceWidthInTiles( int idx ) const                            { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceWidthInTiles( );      }
  void                   setSliceHeightInTiles( int idx, uint32_t u )                     { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInTiles( u );           }
  uint32_t               getSliceHeightInTiles( int idx ) const                           { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInTiles( );     }
  void                   setNumSlicesInTile( int idx, uint32_t u )                        { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setNumSlicesInTile( u );              }
  uint32_t               getNumSlicesInTile( int idx ) const                              { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getNumSlicesInTile( );        }
  void                   setSliceHeightInCtu( int idx, uint32_t u )                       { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInCtu( u );             }
  uint32_t               getSliceHeightInCtu( int idx ) const                             { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInCtu( );       }
  void                   setSliceTileIdx(  int idx, uint32_t u )                          { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setTileIdx( u );                      }
  uint32_t               getSliceTileIdx( int idx ) const                                 { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getTileIdx( );                }
  void                   setRectSlices( std::vector<RectSlice> rectSlices )               { m_rectSlices = rectSlices;                                                                                                      }
  void                   setLoopFilterAcrossTilesEnabledFlag( bool b )                    { m_loopFilterAcrossTilesEnabledFlag = b;                                                                                         }
  bool                   getLoopFilterAcrossTilesEnabledFlag( ) const                     { return  m_loopFilterAcrossTilesEnabledFlag;                                                                                     }
  void                   setLoopFilterAcrossSlicesEnabledFlag( bool b )                   { m_loopFilterAcrossSlicesEnabledFlag = b;                                                                                        }
  bool                   getLoopFilterAcrossSlicesEnabledFlag( ) const                    { return  m_loopFilterAcrossSlicesEnabledFlag;                                                                                    }
  void                   resetTileSliceInfo();
  void                   initTiles();
  void                   initRectSlices();
  void                   initRectSliceMap();
#if JVET_O1143_SUBPIC_BOUNDARY
  std::vector<SubPic>    getSubPics()  const                                              {return m_subPics;          };
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  SubPic                 getSubPic(uint32_t idx) const                                    { return m_subPics[idx]; }
#endif
  void                   initSubPic(const SPS &sps);
  SubPic                 getSubPicFromPos(const Position& pos)  const;
  SubPic                 getSubPicFromCU (const CodingUnit& cu) const;
#endif
  void                   initRasterSliceMap( std::vector<uint32_t> sizes );
  void                   checkSliceMap(); 
  SliceMap               getSliceMap( int idx ) const                                     { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return m_sliceMap[idx];                             }
#if !JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL
  uint32_t               getLog2MaxTransformSkipBlockSize() const                         { return m_log2MaxTransformSkipBlockSize; }
  void                   setLog2MaxTransformSkipBlockSize(uint32_t u)                     { m_log2MaxTransformSkipBlockSize = u; }
#endif
#if !JVET_Q0151_Q0205_ENTRYPOINTS
  bool                   getEntropyCodingSyncEnabledFlag() const                          { return m_entropyCodingSyncEnabledFlag;        }
  void                   setEntropyCodingSyncEnabledFlag(bool val)                        { m_entropyCodingSyncEnabledFlag = val;         }
#endif


#if !JVET_Q0482_REMOVE_CONSTANT_PARAMS
  bool                    getConstantSliceHeaderParamsEnabledFlag() const                 { return m_constantSliceHeaderParamsEnabledFlag; }
  void                    setConstantSliceHeaderParamsEnabledFlag(bool b)                 { m_constantSliceHeaderParamsEnabledFlag = b;   }
  int                     getPPSDepQuantEnabledIdc() const                                { return m_PPSDepQuantEnabledIdc;               }
  void                    setPPSDepQuantEnabledIdc(int u)                                 { m_PPSDepQuantEnabledIdc = u;                  }
  int                     getPPSRefPicListSPSIdc( bool b ) const                          { return b==1 ? m_PPSRefPicListSPSIdc1: m_PPSRefPicListSPSIdc0; }
  int                     getPPSRefPicListSPSIdc0() const                                 { return m_PPSRefPicListSPSIdc0;                }
  void                    setPPSRefPicListSPSIdc0(int u)                                  { m_PPSRefPicListSPSIdc0 = u;                   }
  int                     getPPSRefPicListSPSIdc1() const                                 { return m_PPSRefPicListSPSIdc1;                }
  void                    setPPSRefPicListSPSIdc1(int u)                                  { m_PPSRefPicListSPSIdc1 = u;                   }
  int                     getPPSMvdL1ZeroIdc() const                                      { return m_PPSMvdL1ZeroIdc;                     }
  void                    setPPSMvdL1ZeroIdc(int u)                                       { m_PPSMvdL1ZeroIdc = u;                        }
  int                     getPPSCollocatedFromL0Idc() const                               { return m_PPSCollocatedFromL0Idc;              }
  void                    setPPSCollocatedFromL0Idc(int u)                                { m_PPSCollocatedFromL0Idc = u;                 }
  uint32_t                getPPSSixMinusMaxNumMergeCandPlus1() const                      { return m_PPSSixMinusMaxNumMergeCandPlus1;     }
  void                    setPPSSixMinusMaxNumMergeCandPlus1(uint32_t u)                  { m_PPSSixMinusMaxNumMergeCandPlus1 = u;        }
#if !JVET_Q0806
  uint32_t                getPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1() const       { return m_PPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1; }
  void                    setPPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1(uint32_t u)   { m_PPSMaxNumMergeCandMinusMaxNumTriangleCandPlus1 = u; }
#else
  uint32_t                getPPSMaxNumMergeCandMinusMaxNumGeoCandPlus1() const            { return m_PPSMaxNumMergeCandMinusMaxNumGeoCandPlus1; }
  void                    setPPSMaxNumMergeCandMinusMaxNumGeoCandPlus1(uint32_t u)        { m_PPSMaxNumMergeCandMinusMaxNumGeoCandPlus1 = u; }
#endif
#endif

  void                   setCabacInitPresentFlag( bool flag )                             { m_cabacInitPresentFlag = flag;                }
  bool                   getCabacInitPresentFlag() const                                  { return m_cabacInitPresentFlag;                }
  void                   setDeblockingFilterControlPresentFlag( bool val )                { m_deblockingFilterControlPresentFlag = val;   }
  bool                   getDeblockingFilterControlPresentFlag() const                    { return m_deblockingFilterControlPresentFlag;  }
  void                   setDeblockingFilterOverrideEnabledFlag( bool val )               { m_deblockingFilterOverrideEnabledFlag = val;  }
  bool                   getDeblockingFilterOverrideEnabledFlag() const                   { return m_deblockingFilterOverrideEnabledFlag; }
  void                   setPPSDeblockingFilterDisabledFlag(bool val)                     { m_ppsDeblockingFilterDisabledFlag = val;      } //!< set offset for deblocking filter disabled
  bool                   getPPSDeblockingFilterDisabledFlag() const                       { return m_ppsDeblockingFilterDisabledFlag;     } //!< get offset for deblocking filter disabled
  void                   setDeblockingFilterBetaOffsetDiv2(int val)                       { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  int                    getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  void                   setDeblockingFilterTcOffsetDiv2(int val)                         { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  int                    getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
  void                   setDeblockingFilterCbBetaOffsetDiv2(int val)                     { m_deblockingFilterCbBetaOffsetDiv2 = val;     } //!< set beta offset for Cb deblocking filter
  int                    getDeblockingFilterCbBetaOffsetDiv2() const                      { return m_deblockingFilterCbBetaOffsetDiv2;    } //!< get beta offset for Cb deblocking filter
  void                   setDeblockingFilterCbTcOffsetDiv2(int val)                       { m_deblockingFilterCbTcOffsetDiv2 = val;       } //!< set tc offset for Cb deblocking filter
  int                    getDeblockingFilterCbTcOffsetDiv2() const                        { return m_deblockingFilterCbTcOffsetDiv2;      } //!< get tc offset for Cb deblocking filter
  void                   setDeblockingFilterCrBetaOffsetDiv2(int val)                     { m_deblockingFilterCrBetaOffsetDiv2 = val;     } //!< set beta offset for Cr deblocking filter
  int                    getDeblockingFilterCrBetaOffsetDiv2() const                      { return m_deblockingFilterCrBetaOffsetDiv2;    } //!< get beta offset for Cr deblocking filter
  void                   setDeblockingFilterCrTcOffsetDiv2(int val)                       { m_deblockingFilterCrTcOffsetDiv2 = val;       } //!< set tc offset for Cr deblocking filter
  int                    getDeblockingFilterCrTcOffsetDiv2() const                        { return m_deblockingFilterCrTcOffsetDiv2;      } //!< get tc offset for Cr deblocking filter
#endif
  bool                   getListsModificationPresentFlag() const                          { return m_listsModificationPresentFlag;        }
  void                   setListsModificationPresentFlag( bool b )                        { m_listsModificationPresentFlag = b;           }
  bool                   getPictureHeaderExtensionPresentFlag() const                     { return m_pictureHeaderExtensionPresentFlag;     }
  void                   setPictureHeaderExtensionPresentFlag(bool val)                   { m_pictureHeaderExtensionPresentFlag = val;      }
  bool                   getSliceHeaderExtensionPresentFlag() const                       { return m_sliceHeaderExtensionPresentFlag;     }
  void                   setSliceHeaderExtensionPresentFlag(bool val)                     { m_sliceHeaderExtensionPresentFlag = val;      }

#if JVET_Q0819_PH_CHANGES 
  void                   setRplInfoInPhFlag(bool flag)                                    { m_rplInfoInPhFlag = flag;                     }
  bool                   getRplInfoInPhFlag() const                                       { return m_rplInfoInPhFlag;                     }
  void                   setDbfInfoInPhFlag(bool flag)                                    { m_dbfInfoInPhFlag = flag;                     }
  bool                   getDbfInfoInPhFlag() const                                       { return m_dbfInfoInPhFlag;                     }
  void                   setSaoInfoInPhFlag(bool flag)                                    { m_saoInfoInPhFlag = flag;                     }
  bool                   getSaoInfoInPhFlag() const                                       { return m_saoInfoInPhFlag;                     }
  void                   setAlfInfoInPhFlag(bool flag)                                    { m_alfInfoInPhFlag = flag;                     }
  bool                   getAlfInfoInPhFlag() const                                       { return m_alfInfoInPhFlag;                     }
  void                   setWpInfoInPhFlag(bool flag)                                     { m_wpInfoInPhFlag = flag;                      }
  bool                   getWpInfoInPhFlag() const                                        { return m_wpInfoInPhFlag;                      }
  void                   setQpDeltaInfoInPhFlag(bool flag)                                { m_qpDeltaInfoInPhFlag = flag;                 }
  bool                   getQpDeltaInfoInPhFlag() const                                   { return m_qpDeltaInfoInPhFlag; }
#endif

  const PPSRExt&         getPpsRangeExtension() const                                     { return m_ppsRangeExtension;                   }
  PPSRExt&               getPpsRangeExtension()                                           { return m_ppsRangeExtension;                   }

  void                    setPicWidthInLumaSamples( uint32_t u )                          { m_picWidthInLumaSamples = u; }
  uint32_t                getPicWidthInLumaSamples() const                                { return  m_picWidthInLumaSamples; }
  void                    setPicHeightInLumaSamples( uint32_t u )                         { m_picHeightInLumaSamples = u; }
  uint32_t                getPicHeightInLumaSamples() const                               { return  m_picHeightInLumaSamples; }

  Window&                 getConformanceWindow()                                          { return  m_conformanceWindow; }
  const Window&           getConformanceWindow() const                                    { return  m_conformanceWindow; }
  void                    setConformanceWindow( Window& conformanceWindow )               { m_conformanceWindow = conformanceWindow; }

  Window&                 getScalingWindow()                                              { return  m_scalingWindow; }
  const Window&           getScalingWindow()                                        const { return  m_scalingWindow; }
  void                    setScalingWindow( Window& scalingWindow )                       { m_scalingWindow = scalingWindow; }

#if SPS_ID_CHECK
  int                     getMixedNaluTypesInPicFlag() const                              { return m_mixedNaluTypesInPicFlag; }
  void                    setMixedNaluTypesInPicFlag( const bool flag )                   { m_mixedNaluTypesInPicFlag = flag; }
#endif
};

class APS
{
private:
  int                    m_APSId;                    // adaptation_parameter_set_id
  int                    m_temporalId;
  int                    m_layerId;
  ApsType                m_APSType;                  // aps_params_type
  AlfParam               m_alfAPSParam;
  SliceReshapeInfo       m_reshapeAPSInfo;
  ScalingList            m_scalingListApsInfo;
#if JVET_Q0795_CCALF
  CcAlfFilterParam       m_ccAlfAPSParam;
#endif

public:
  APS();
  virtual                ~APS();

  int                    getAPSId() const                                                 { return m_APSId;                               }
  void                   setAPSId(int i)                                                  { m_APSId = i;                                  }

  ApsType                getAPSType() const                                               { return m_APSType;                             }
  void                   setAPSType( ApsType type )                                       { m_APSType = type;                             }

  void                   setAlfAPSParam(AlfParam& alfAPSParam)                            { m_alfAPSParam = alfAPSParam;                  }
  void                   setTemporalId( int i )                                           { m_temporalId = i;                             }
  int                    getTemporalId()                                            const { return m_temporalId;                          }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }
  AlfParam&              getAlfAPSParam()  { return m_alfAPSParam; }

  void                   setReshaperAPSInfo(SliceReshapeInfo& reshapeAPSInfo)             { m_reshapeAPSInfo = reshapeAPSInfo;            }
  SliceReshapeInfo&      getReshaperAPSInfo()                                             { return m_reshapeAPSInfo;                      }
  void                   setScalingList( ScalingList& scalingListAPSInfo )                { m_scalingListApsInfo = scalingListAPSInfo;    }
  ScalingList&           getScalingList()                                                 { return m_scalingListApsInfo;                  }
#if JVET_Q0795_CCALF
  void                   setCcAlfAPSParam(CcAlfFilterParam& ccAlfAPSParam)                { m_ccAlfAPSParam = ccAlfAPSParam;              }
  CcAlfFilterParam&      getCcAlfAPSParam()  { return m_ccAlfAPSParam; }
#endif
};
struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  bool bPresentFlag;
  uint32_t uiLog2WeightDenom;
  int  iWeight;
  int  iOffset;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  int  w;
  int  o;
  int  offset;
  int  shift;
  int  round;

};
struct WPACDCParam
{
  int64_t iAC;
  int64_t iDC;
};

// picture header class
class PicHeader
{
private:
  bool                        m_valid;                                                  //!< picture header is valid yet or not
  Picture*                    m_pcPic;                                                  //!< pointer to picture structure
#if JVET_Q0819_PH_CHANGES
  int                         m_pocLsb;                                                 //!< least significant bits of picture order count
#endif
  bool                        m_nonReferencePictureFlag;                                //!< non-reference picture flag
  bool                        m_gdrOrIrapPicFlag;                                       //!< gdr or irap picture flag
  bool                        m_gdrPicFlag;                                             //!< gradual decoding refresh picture flag
  bool                        m_noOutputOfPriorPicsFlag;                                //!< no output of prior pictures flag
  uint32_t                    m_recoveryPocCnt;                                         //!< recovery POC count
  int                         m_spsId;                                                  //!< sequence parameter set ID
  int                         m_ppsId;                                                  //!< picture parameter set ID
#if JVET_P0116_POC_MSB
  bool                        m_pocMsbPresentFlag;                                      //!< ph_poc_msb_present_flag
  int                         m_pocMsbVal;                                              //!< poc_msb_val
#endif
#if !JVET_Q0119_CLEANUPS
  bool                        m_subPicIdSignallingPresentFlag;                          //!< indicates the presence of sub-picture ID signalling in the SPS
  uint32_t                    m_subPicIdLen;                                            //!< sub-picture ID length in bits
  uint8_t                     m_subPicId[MAX_NUM_SUB_PICS];                             //!< sub-picture ID for each sub-picture in the sequence
#endif
  bool                        m_loopFilterAcrossVirtualBoundariesDisabledFlag;          //!< loop filtering across virtual boundaries disabled
  unsigned                    m_numVerVirtualBoundaries;                                //!< number of vertical virtual boundaries
  unsigned                    m_numHorVirtualBoundaries;                                //!< number of horizontal virtual boundaries
  unsigned                    m_virtualBoundariesPosX[3];                               //!< horizontal virtual boundary positions
  unsigned                    m_virtualBoundariesPosY[3];                               //!< vertical virtual boundary positions
#if !JVET_Q0155_COLOUR_ID
  unsigned                    m_colourPlaneId;                                          //!< 4:4:4 colour plane ID
#endif
  bool                        m_picOutputFlag;                                          //!< picture output flag
#if !JVET_Q0819_PH_CHANGES  
  bool                        m_picRplPresentFlag;                                      //!< reference lists present in picture header or not
#endif
  const ReferencePictureList
    *m_pRPL0;   //!< pointer to RPL for L0, either in the SPS or the local RPS in the picture header
  const ReferencePictureList* m_pRPL1;                                                  //!< pointer to RPL for L1, either in the SPS or the local RPS in the picture header
  ReferencePictureList        m_localRPL0;                                              //!< RPL for L0 when present in picture header
  ReferencePictureList        m_localRPL1;                                              //!< RPL for L1 when present in picture header
  int                         m_rpl0Idx;                                                //!< index of used RPL in the SPS or -1 for local RPL in the picture header
  int                         m_rpl1Idx;                                                //!< index of used RPL in the SPS or -1 for local RPL in the picture header
#if JVET_Q0819_PH_CHANGES
  bool                        m_picInterSliceAllowedFlag;                               //!< inter slice allowed flag in PH
  bool                        m_picIntraSliceAllowedFlag;                               //!< intra slice allowed flag in PH
#endif
  bool                        m_splitConsOverrideFlag;                                  //!< partitioning constraint override flag  
  uint32_t                    m_cuQpDeltaSubdivIntra;                                   //!< CU QP delta maximum subdivision for intra slices
  uint32_t                    m_cuQpDeltaSubdivInter;                                   //!< CU QP delta maximum subdivision for inter slices 
  uint32_t                    m_cuChromaQpOffsetSubdivIntra;                            //!< CU chroma QP offset maximum subdivision for intra slices 
  uint32_t                    m_cuChromaQpOffsetSubdivInter;                            //!< CU chroma QP offset maximum subdivision for inter slices 
  bool                        m_enableTMVPFlag;                                         //!< enable temporal motion vector prediction
#if JVET_Q0482_REMOVE_CONSTANT_PARAMS
  bool                        m_picColFromL0Flag;                                       //!< syntax element collocated_from_l0_flag
#endif
#if JVET_Q0259_COLLOCATED_PIC_IN_PH
  uint32_t                    m_colRefIdx;
#endif
  bool                        m_mvdL1ZeroFlag;                                          //!< L1 MVD set to zero flag  
  uint32_t                    m_maxNumMergeCand;                                        //!< max number of merge candidates
  uint32_t                    m_maxNumAffineMergeCand;                                  //!< max number of sub-block merge candidates
  bool                        m_disFracMMVD;                                            //!< fractional MMVD offsets disabled flag
  bool                        m_disBdofFlag;                                            //!< picture level BDOF disable flag
  bool                        m_disDmvrFlag;                                            //!< picture level DMVR disable flag
  bool                        m_disProfFlag;                                            //!< picture level PROF disable flag
#if !JVET_Q0806
  uint32_t                    m_maxNumTriangleCand;                                     //!< max number of triangle merge candidates
#else
  uint32_t                    m_maxNumGeoCand;                                          //!< max number of geometric merge candidates
#endif
  uint32_t                    m_maxNumIBCMergeCand;                                     //!< max number of IBC merge candidates
  bool                        m_jointCbCrSignFlag;                                      //!< joint Cb/Cr residual sign flag  
#if JVET_Q0819_PH_CHANGES 
  int                         m_qpDelta;                                                //!< value of Qp delta
#else
  bool                        m_saoEnabledPresentFlag;                                  //!< sao enabled flags present in the picture header
#endif
  bool                        m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];                   //!< sao enabled flags for each channel
#if !JVET_Q0819_PH_CHANGES 
  bool                        m_alfEnabledPresentFlag;                                  //!< alf enabled flags present in the picture header
#endif
  bool                        m_alfEnabledFlag[MAX_NUM_COMPONENT];                      //!< alf enabled flags for each component
  int                         m_numAlfAps;                                              //!< number of alf aps active for the picture
  std::vector<int>            m_alfApsId;                                               //!< list of alf aps for the picture
  int                         m_alfChromaApsId;                                         //!< chroma alf aps ID
#if JVET_Q0795_CCALF
  bool m_ccalfEnabledFlag[MAX_NUM_COMPONENT];
  int  m_ccalfCbApsId;
  int  m_ccalfCrApsId;
#endif
  bool                        m_depQuantEnabledFlag;                                    //!< dependent quantization enabled flag
  bool                        m_signDataHidingEnabledFlag;                              //!< sign data hiding enabled flag
#if !JVET_Q0819_PH_CHANGES  
  bool                        m_deblockingFilterOverridePresentFlag;                    //!< deblocking filter override controls present in picture header
#endif
  bool                        m_deblockingFilterOverrideFlag;                           //!< deblocking filter override controls enabled
  bool                        m_deblockingFilterDisable;                                //!< deblocking filter disabled flag
  int                         m_deblockingFilterBetaOffsetDiv2;                         //!< beta offset for deblocking filter
  int                         m_deblockingFilterTcOffsetDiv2;                           //!< tc offset for deblocking filter
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
  int                         m_deblockingFilterCbBetaOffsetDiv2;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCbTcOffsetDiv2;                         //!< tc offset for deblocking filter
  int                         m_deblockingFilterCrBetaOffsetDiv2;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCrTcOffsetDiv2;                         //!< tc offset for deblocking filter
#endif
  bool                        m_lmcsEnabledFlag;                                        //!< lmcs enabled flag
  int                         m_lmcsApsId;                                              //!< lmcs APS ID
  APS*                        m_lmcsAps;                                                //!< lmcs APS
  bool                        m_lmcsChromaResidualScaleFlag;                            //!< lmcs chroma residual scale flag  
  bool                        m_scalingListPresentFlag;                                 //!< quantization scaling lists present
  int                         m_scalingListApsId;                                       //!< quantization scaling list APS ID
  APS*                        m_scalingListAps;                                         //!< quantization scaling list APS
  unsigned                    m_minQT[3];                                               //!< minimum quad-tree size  0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned                    m_maxMTTHierarchyDepth[3];                                //!< maximum MTT depth
  unsigned                    m_maxBTSize[3];                                           //!< maximum BT size
  unsigned                    m_maxTTSize[3];                                           //!< maximum TT size

#if JVET_Q0819_PH_CHANGES  
  WPScalingParam              m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT];   // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  int                         m_numL0Weights;                                           //!< number of weights for L0 list
  int                         m_numL1Weights;                                           //!< number of weights for L1 list
#endif

public:
                              PicHeader();
  virtual                     ~PicHeader();
  void                        initPicHeader();
  bool                        isValid()                                                 { return m_valid;                                                                              }
  void                        setValid()                                                { m_valid = true;                                                                              }
  void                        setPic( Picture* p )                                      { m_pcPic = p;                                                                                 }
  Picture*                    getPic()                                                  { return m_pcPic;                                                                              }
  const Picture*              getPic() const                                            { return m_pcPic;                                                                              }
#if JVET_Q0819_PH_CHANGES
  void                        setPocLsb(int i)                                          { m_pocLsb = i;                                                                                }
  int                         getPocLsb()                                               { return m_pocLsb;                                                                             }
#endif
  void                        setNonReferencePictureFlag( bool b )                      { m_nonReferencePictureFlag = b;                                                               }
  bool                        getNonReferencePictureFlag() const                        { return m_nonReferencePictureFlag;                                                            }
  void                        setGdrOrIrapPicFlag( bool b )                             { m_gdrOrIrapPicFlag = b;                                                                      }
  bool                        getGdrOrIrapPicFlag() const                               { return m_gdrOrIrapPicFlag;                                                                   }
  void                        setGdrPicFlag( bool b )                                   { m_gdrPicFlag = b;                                                                            }
  bool                        getGdrPicFlag() const                                     { return m_gdrPicFlag;                                                                         }
  void                        setNoOutputOfPriorPicsFlag( bool b )                      { m_noOutputOfPriorPicsFlag = b;                                                               }
  bool                        getNoOutputOfPriorPicsFlag() const                        { return m_noOutputOfPriorPicsFlag;                                                            }
  void                        setRecoveryPocCnt( uint32_t u )                           { m_recoveryPocCnt = u;                                                                        }
  bool                        getRecoveryPocCnt() const                                 { return m_recoveryPocCnt;                                                                     }
  void                        setSPSId( uint32_t u )                                    { m_spsId = u;                                                                                 }
  uint32_t                    getSPSId() const                                          { return m_spsId;                                                                              }
  void                        setPPSId( uint32_t u )                                    { m_ppsId = u;                                                                                 }
  uint32_t                    getPPSId() const                                          { return m_ppsId;                                                                              }
#if JVET_P0116_POC_MSB
  void                        setPocMsbPresentFlag(bool b)                              { m_pocMsbPresentFlag = b;                                                                     }
  bool                        getPocMsbPresentFlag() const                              { return m_pocMsbPresentFlag;                                                                  }
  void                        setPocMsbVal(int i)                                       { m_pocMsbVal = i;                                                                             }
  int                         getPocMsbVal()                                            { return m_pocMsbVal;                                                                          }
#endif
#if !JVET_Q0119_CLEANUPS
  void                        setSubPicIdSignallingPresentFlag( bool b )                { m_subPicIdSignallingPresentFlag = b;                                                         }
  bool                        getSubPicIdSignallingPresentFlag() const                  { return  m_subPicIdSignallingPresentFlag;                                                     }
  void                        setSubPicIdLen( uint32_t u )                              { m_subPicIdLen = u;                                                                           }
  uint32_t                    getSubPicIdLen() const                                    { return  m_subPicIdLen;                                                                       }
  void                        setSubPicId( int i, uint8_t u )                           { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-pic index exceeds valid range" ); m_subPicId[i] = u;      }
  uint8_t                     getSubPicId( int i ) const                                { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-pic index exceeds valid range" ); return  m_subPicId[i];  }
#endif
  void                        setLoopFilterAcrossVirtualBoundariesDisabledFlag(bool b)  { m_loopFilterAcrossVirtualBoundariesDisabledFlag = b;                                         }
  bool                        getLoopFilterAcrossVirtualBoundariesDisabledFlag() const  { return m_loopFilterAcrossVirtualBoundariesDisabledFlag;                                      }
  void                        setNumVerVirtualBoundaries(unsigned u)                    { m_numVerVirtualBoundaries = u;                                                               }
  unsigned                    getNumVerVirtualBoundaries() const                        { return m_numVerVirtualBoundaries;                                                            }
  void                        setNumHorVirtualBoundaries(unsigned u)                    { m_numHorVirtualBoundaries = u;                                                               }
  unsigned                    getNumHorVirtualBoundaries() const                        { return m_numHorVirtualBoundaries;                                                            }
  void                        setVirtualBoundariesPosX(unsigned u, unsigned idx)        { CHECK( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;   }
  unsigned                    getVirtualBoundariesPosX(unsigned idx) const              { CHECK( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx];}
  void                        setVirtualBoundariesPosY(unsigned u, unsigned idx)        { CHECK( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;   }
  unsigned                    getVirtualBoundariesPosY(unsigned idx) const              { CHECK( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx];}
#if !JVET_Q0155_COLOUR_ID
  void                        setColourPlaneId(unsigned u)                              { m_colourPlaneId = u;                                                                         }
  unsigned                    getColourPlaneId() const                                  { return m_colourPlaneId;                                                                      }
#endif
  void                        setPicOutputFlag( bool b )                                { m_picOutputFlag = b;                                                                         }
  bool                        getPicOutputFlag() const                                  { return m_picOutputFlag;                                                                      }
#if !JVET_Q0819_PH_CHANGES 
  void                        setPicRplPresentFlag( bool b )                            { m_picRplPresentFlag = b;                                                                     }
  bool                        getPicRplPresentFlag() const                              { return m_picRplPresentFlag;                                                                  }
#endif
  void                        setRPL( bool b, const ReferencePictureList *pcRPL)        { if(b==1) { m_pRPL1 = pcRPL; } else { m_pRPL0 = pcRPL; }                                      }
  const ReferencePictureList* getRPL( bool b )                                          { return b==1 ? m_pRPL1 : m_pRPL0;                                                             }
  ReferencePictureList*       getLocalRPL( bool b )                                     { return b==1 ? &m_localRPL1 : &m_localRPL0;                                                   }
  void                        setRPLIdx( bool b, int rplIdx)                            { if(b==1) { m_rpl1Idx = rplIdx; } else { m_rpl0Idx = rplIdx; }                                }
  int                         getRPLIdx( bool b ) const                                 { return b==1 ? m_rpl1Idx : m_rpl0Idx;                                                         }
  void                        setRPL0(const ReferencePictureList *pcRPL)                { m_pRPL0 = pcRPL;                                                                             }
  void                        setRPL1(const ReferencePictureList *pcRPL)                { m_pRPL1 = pcRPL;                                                                             }
  const ReferencePictureList* getRPL0()                                                 { return m_pRPL0;                                                                              }
  const ReferencePictureList* getRPL1()                                                 { return m_pRPL1;                                                                              }
  ReferencePictureList*       getLocalRPL0()                                            { return &m_localRPL0;                                                                         }
  ReferencePictureList*       getLocalRPL1()                                            { return &m_localRPL1;                                                                         }
  void                        setRPL0idx(int rplIdx)                                    { m_rpl0Idx = rplIdx;                                                                          }
  void                        setRPL1idx(int rplIdx)                                    { m_rpl1Idx = rplIdx;                                                                          }
  int                         getRPL0idx() const                                        { return m_rpl0Idx;                                                                            }
  int                         getRPL1idx() const                                        { return m_rpl1Idx;                                                                            }
#if JVET_Q0819_PH_CHANGES
  void                        setPicInterSliceAllowedFlag(bool b)                       { m_picInterSliceAllowedFlag = b; }
  bool                        getPicInterSliceAllowedFlag() const                       { return m_picInterSliceAllowedFlag; }
  void                        setPicIntraSliceAllowedFlag(bool b)                       { m_picIntraSliceAllowedFlag = b; }
  bool                        getPicIntraSliceAllowedFlag() const                       { return m_picIntraSliceAllowedFlag; }
#endif 
  void                        setSplitConsOverrideFlag( bool b )                        { m_splitConsOverrideFlag = b;                                                                 }
  bool                        getSplitConsOverrideFlag() const                          { return m_splitConsOverrideFlag;                                                              }  
  void                        setCuQpDeltaSubdivIntra( uint32_t u )                     { m_cuQpDeltaSubdivIntra = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivIntra() const                           { return m_cuQpDeltaSubdivIntra;                                                               }
  void                        setCuQpDeltaSubdivInter( uint32_t u )                     { m_cuQpDeltaSubdivInter = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivInter() const                           { return m_cuQpDeltaSubdivInter;                                                               }
  void                        setCuChromaQpOffsetSubdivIntra( uint32_t u )              { m_cuChromaQpOffsetSubdivIntra = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivIntra() const                    { return m_cuChromaQpOffsetSubdivIntra;                                                        }
  void                        setCuChromaQpOffsetSubdivInter( uint32_t u )              { m_cuChromaQpOffsetSubdivInter = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivInter() const                    { return m_cuChromaQpOffsetSubdivInter;                                                        }
  void                        setEnableTMVPFlag( bool b )                               { m_enableTMVPFlag = b;                                                                        }
  bool                        getEnableTMVPFlag() const                                 { return m_enableTMVPFlag;                                                                     }
#if JVET_Q0482_REMOVE_CONSTANT_PARAMS
  void                        setPicColFromL0Flag(bool val)                             { m_picColFromL0Flag = val;                                                                     }
  bool                        getPicColFromL0Flag() const                               { return m_picColFromL0Flag;                                                                    }
#endif
#if JVET_Q0259_COLLOCATED_PIC_IN_PH
  void                        setColRefIdx( uint32_t refIdx)                             { m_colRefIdx = refIdx;                                                                       }
  uint32_t                    getColRefIdx()                                             { return m_colRefIdx;                                                                         }
#endif
  void                        setMvdL1ZeroFlag( bool b )                                { m_mvdL1ZeroFlag = b;                                                                         }
  bool                        getMvdL1ZeroFlag() const                                  { return m_mvdL1ZeroFlag;                                                                      }  
  void                        setMaxNumMergeCand(uint32_t val )                         { m_maxNumMergeCand = val;                                                                     }
  uint32_t                    getMaxNumMergeCand() const                                { return m_maxNumMergeCand;                                                                    }  
  void                        setMaxNumAffineMergeCand( uint32_t val )                  { m_maxNumAffineMergeCand = val;                                                               }
  uint32_t                    getMaxNumAffineMergeCand() const                          { return m_maxNumAffineMergeCand;                                                              }
  void                        setDisFracMMVD( bool val )                                { m_disFracMMVD = val;                                                                         }
  bool                        getDisFracMMVD() const                                    { return m_disFracMMVD;                                                                        }  
  void                        setDisBdofFlag( bool val )                                { m_disBdofFlag = val;                                                                         }
  bool                        getDisBdofFlag() const                                    { return m_disBdofFlag;                                                                        }
  void                        setDisDmvrFlag( bool val )                                { m_disDmvrFlag = val;                                                                         }
  bool                        getDisDmvrFlag() const                                    { return m_disDmvrFlag;                                                                        }
  void                        setDisProfFlag( bool val )                                { m_disProfFlag = val;                                                                         }
  bool                        getDisProfFlag() const                                    { return m_disProfFlag;                                                                        }
#if !JVET_Q0806
  void                        setMaxNumTriangleCand(uint32_t b)                         { m_maxNumTriangleCand = b;                                                                    }
  uint32_t                    getMaxNumTriangleCand() const                             { return m_maxNumTriangleCand;                                                                 }
#else
  void                        setMaxNumGeoCand(uint32_t b)                              { m_maxNumGeoCand = b; }
  uint32_t                    getMaxNumGeoCand() const                                  { return m_maxNumGeoCand; }
#endif
  void                        setMaxNumIBCMergeCand( uint32_t b )                       { m_maxNumIBCMergeCand = b;                                                                    }
  uint32_t                    getMaxNumIBCMergeCand() const                             { return m_maxNumIBCMergeCand;                                                                 } 
  void                        setJointCbCrSignFlag( bool b )                            { m_jointCbCrSignFlag = b;                                                                     }
  bool                        getJointCbCrSignFlag() const                              { return m_jointCbCrSignFlag;                                                                  }
#if JVET_Q0819_PH_CHANGES 
  void                        setQpDelta(int b)                                         { m_qpDelta = b;                                                                               }
  int                         getQpDelta() const                                        { return m_qpDelta;                                                                            }
#else
  void                        setSaoEnabledPresentFlag( bool b )                        { m_saoEnabledPresentFlag = b;                                                                 }
  bool                        getSaoEnabledPresentFlag() const                          { return m_saoEnabledPresentFlag;                                                              }
#endif
  void                        setSaoEnabledFlag(ChannelType chType, bool b)             { m_saoEnabledFlag[chType] = b;                                                                }
  bool                        getSaoEnabledFlag(ChannelType chType) const               { return m_saoEnabledFlag[chType];                                                             }  
#if !JVET_Q0819_PH_CHANGES   
  void                        setAlfEnabledPresentFlag( bool b )                        { m_alfEnabledPresentFlag = b;                                                                 }
  bool                        getAlfEnabledPresentFlag() const                          { return m_alfEnabledPresentFlag;                                                              }
#endif
  void                        setAlfEnabledFlag(ComponentID compId, bool b)             { m_alfEnabledFlag[compId] = b;                                                                }
  bool                        getAlfEnabledFlag(ComponentID compId) const               { return m_alfEnabledFlag[compId];                                                             }
  void                        setNumAlfAps(int i)                                       { m_numAlfAps = i;                                                                             }
  int                         getNumAlfAps() const                                      { return m_numAlfAps;                                                                          }
  void                        setAlfApsIdChroma(int i)                                  { m_alfChromaApsId = i;                                                                        }
  int                         getAlfApsIdChroma() const                                 { return m_alfChromaApsId;                                                                     }
#if JVET_Q0795_CCALF
  void setCcAlfEnabledFlag(ComponentID compId, bool b) { m_ccalfEnabledFlag[compId] = b; }
  bool getCcAlfEnabledFlag(ComponentID compId) const { return m_ccalfEnabledFlag[compId]; }

  void setCcAlfCbApsId(int i) { m_ccalfCbApsId = i; }
  int  getCcAlfCbApsId() const { return m_ccalfCbApsId; }
  void setCcAlfCrApsId(int i) { m_ccalfCrApsId = i; }
  int  getCcAlfCrApsId() const { return m_ccalfCrApsId; }
#endif
  void                        setDepQuantEnabledFlag( bool b )                          { m_depQuantEnabledFlag = b;                                                                   }
  bool                        getDepQuantEnabledFlag() const                            { return m_depQuantEnabledFlag;                                                                }  
  void                        setSignDataHidingEnabledFlag( bool b )                    { m_signDataHidingEnabledFlag = b;                                                             }
  bool                        getSignDataHidingEnabledFlag() const                      { return m_signDataHidingEnabledFlag;                                                          }  
#if !JVET_Q0819_PH_CHANGES 
  void                        setDeblockingFilterOverridePresentFlag( bool b )          { m_deblockingFilterOverridePresentFlag = b;                                                   }
  bool                        getDeblockingFilterOverridePresentFlag() const            { return m_deblockingFilterOverridePresentFlag;                                                }
#endif
  void                        setDeblockingFilterOverrideFlag( bool b )                 { m_deblockingFilterOverrideFlag = b;                                                          }
  bool                        getDeblockingFilterOverrideFlag() const                   { return m_deblockingFilterOverrideFlag;                                                       }    
  void                        setDeblockingFilterDisable( bool b )                      { m_deblockingFilterDisable= b;                                                                }  
  bool                        getDeblockingFilterDisable() const                        { return m_deblockingFilterDisable;                                                            }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )                { m_deblockingFilterBetaOffsetDiv2 = i;                                                        }  
  int                         getDeblockingFilterBetaOffsetDiv2()const                  { return m_deblockingFilterBetaOffsetDiv2;                                                     }
  void                        setDeblockingFilterTcOffsetDiv2( int i )                  { m_deblockingFilterTcOffsetDiv2 = i;                                                          }  
  int                         getDeblockingFilterTcOffsetDiv2() const                   { return m_deblockingFilterTcOffsetDiv2;                                                       }    
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )              { m_deblockingFilterCbBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCbBetaOffsetDiv2()const                { return m_deblockingFilterCbBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )                { m_deblockingFilterCbTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCbTcOffsetDiv2() const                 { return m_deblockingFilterCbTcOffsetDiv2;                                                     }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )              { m_deblockingFilterCrBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const                { return m_deblockingFilterCrBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )                { m_deblockingFilterCrTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCrTcOffsetDiv2() const                 { return m_deblockingFilterCrTcOffsetDiv2;                                                     }
#endif
  void                        setLmcsEnabledFlag(bool b)                                { m_lmcsEnabledFlag = b;                                                                       }
  bool                        getLmcsEnabledFlag()                                      { return m_lmcsEnabledFlag;                                                                    }
  const bool                  getLmcsEnabledFlag() const                                { return m_lmcsEnabledFlag;                                                                    }
  void                        setLmcsAPS(APS* aps)                                      { m_lmcsAps = aps; m_lmcsApsId = (aps) ? aps->getAPSId() : -1;                                 }
  APS*                        getLmcsAPS() const                                        { return m_lmcsAps;                                                                            }
  void                        setLmcsAPSId(int id)                                      { m_lmcsApsId = id;                                                                            }
  int                         getLmcsAPSId() const                                      { return m_lmcsApsId;                                                                          }
  void                        setLmcsChromaResidualScaleFlag(bool b)                    { m_lmcsChromaResidualScaleFlag = b;                                                           }
  bool                        getLmcsChromaResidualScaleFlag()                          { return m_lmcsChromaResidualScaleFlag;                                                        }
  const bool                  getLmcsChromaResidualScaleFlag() const                    { return m_lmcsChromaResidualScaleFlag;                                                        }
  void                        setScalingListAPS( APS* aps )                             { m_scalingListAps = aps; m_scalingListApsId = ( aps ) ? aps->getAPSId() : -1;                 }
  APS*                        getScalingListAPS() const                                 { return m_scalingListAps;                                                                     }
  void                        setScalingListAPSId( int id )                             { m_scalingListApsId = id;                                                                     }
  int                         getScalingListAPSId() const                               { return m_scalingListApsId;                                                                   }
  void                        setScalingListPresentFlag( bool b )                       { m_scalingListPresentFlag = b;                                                                }
  bool                        getScalingListPresentFlag()                               { return m_scalingListPresentFlag;                                                             }
  const bool                  getScalingListPresentFlag() const                         { return m_scalingListPresentFlag;                                                             }
  
  unsigned*                   getMinQTSizes() const                                     { return (unsigned *)m_minQT;                                                                  }
  unsigned*                   getMaxMTTHierarchyDepths() const                          { return (unsigned *)m_maxMTTHierarchyDepth;                                                   }
  unsigned*                   getMaxBTSizes() const                                     { return (unsigned *)m_maxBTSize;                                                              }
  unsigned*                   getMaxTTSizes() const                                     { return (unsigned *)m_maxTTSize;                                                              }
  
  void                        setMinQTSize(unsigned idx, unsigned minQT)                { m_minQT[idx] = minQT;                                                                        }
  void                        setMaxMTTHierarchyDepth(unsigned idx, unsigned maxMTT)    { m_maxMTTHierarchyDepth[idx] = maxMTT;                                                        }
  void                        setMaxBTSize(unsigned idx, unsigned maxBT)                { m_maxBTSize[idx] = maxBT;                                                                    }
  void                        setMaxTTSize(unsigned idx, unsigned maxTT)                { m_maxTTSize[idx] = maxTT;                                                                    }

  void                        setMinQTSizes(unsigned*   minQT)                          { m_minQT[0] = minQT[0]; m_minQT[1] = minQT[1]; m_minQT[2] = minQT[2];                                                 }
  void                        setMaxMTTHierarchyDepths(unsigned*   maxMTT)              { m_maxMTTHierarchyDepth[0] = maxMTT[0]; m_maxMTTHierarchyDepth[1] = maxMTT[1]; m_maxMTTHierarchyDepth[2] = maxMTT[2]; }
  void                        setMaxBTSizes(unsigned*   maxBT)                          { m_maxBTSize[0] = maxBT[0]; m_maxBTSize[1] = maxBT[1]; m_maxBTSize[2] = maxBT[2];                                     }
  void                        setMaxTTSizes(unsigned*   maxTT)                          { m_maxTTSize[0] = maxTT[0]; m_maxTTSize[1] = maxTT[1]; m_maxTTSize[2] = maxTT[2];                                     }
    
  unsigned                    getMinQTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_minQT[0] : m_minQT[2]) : m_minQT[1];                                              }
  unsigned                    getMaxMTTHierarchyDepth(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxMTTHierarchyDepth[0] : m_maxMTTHierarchyDepth[2]) : m_maxMTTHierarchyDepth[1]; }
  unsigned                    getMaxBTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxBTSize[0] : m_maxBTSize[2]) : m_maxBTSize[1];                                  }
  unsigned                    getMaxTTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxTTSize[0] : m_maxTTSize[2]) : m_maxTTSize[1];                                  }
  
  void                        setAlfAPSs(std::vector<int> apsIDs)                       { m_alfApsId.resize(m_numAlfAps);
                                                                                          for (int i = 0; i < m_numAlfAps; i++)
                                                                                          {
                                                                                            m_alfApsId[i] = apsIDs[i];
                                                                                          }
                                                                                        }

  std::vector<int>            getAlfAPSs() const                                        { return m_alfApsId; }

#if JVET_Q0819_PH_CHANGES 
  void                        setWpScaling(WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT);
  }
  void                        getWpScaling(RefPicList e, int iRefIdx, WPScalingParam *&wp) const;
  WPScalingParam*             getWpScalingAll()                                        { return (WPScalingParam *) m_weightPredTable; }
  void                        setNumL0Weights(int b)                                   { m_numL0Weights = b;                          }
  int                         getNumL0Weights()                                        { return m_numL0Weights;                       }
  void                        setNumL1Weights(int b)                                   { m_numL1Weights = b;                          }
  int                         getNumL1Weights()                                        { return m_numL1Weights;                       }
#endif
};

/// slice header class
class Slice
{

private:
  //  Bitstream writing
  bool                       m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];
  int                        m_iPOC;
  int                        m_iLastIDR;
  int                        m_iAssociatedIRAP;
  NalUnitType                m_iAssociatedIRAPType;
  bool                       m_enableDRAPSEI;
  bool                       m_useLTforDRAP;
  bool                       m_isDRAP;
  int                        m_latestDRAPPOC;
  const ReferencePictureList* m_pRPL0;                //< pointer to RPL for L0, either in the SPS or the local RPS in the same slice header
  const ReferencePictureList* m_pRPL1;                //< pointer to RPL for L1, either in the SPS or the local RPS in the same slice header
  ReferencePictureList        m_localRPL0;            //< RPL for L0 when present in slice header
  ReferencePictureList        m_localRPL1;            //< RPL for L1 when present in slice header
  int                         m_rpl0Idx;              //< index of used RPL in the SPS or -1 for local RPL in the slice header
  int                         m_rpl1Idx;              //< index of used RPL in the SPS or -1 for local RPL in the slice header
#if JVET_Q0155_COLOUR_ID
  int                        m_colourPlaneId;                         //!< 4:4:4 colour plane ID
#endif
  NalUnitType                m_eNalUnitType;         ///< Nal unit type for the slice
#if JVET_Q0775_PH_IN_SH
  bool                       m_pictureHeaderInSliceHeader;
#endif
#if JVET_Q0118_CLEANUPS
  uint32_t                   m_nuhLayerId;           ///< Nal unit layer id
#endif
  SliceType                  m_eSliceType;
  int                        m_iSliceQp;
  int                        m_iSliceQpBase;
  bool                       m_ChromaQpAdjEnabled;
  bool                       m_deblockingFilterDisable;
  bool                       m_deblockingFilterOverrideFlag;      //< offsets for deblocking filter inherit from PPS
  int                        m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  int                        m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
  int                        m_deblockingFilterCbBetaOffsetDiv2;  //< beta offset for deblocking filter
  int                        m_deblockingFilterCbTcOffsetDiv2;    //< tc offset for deblocking filter
  int                        m_deblockingFilterCrBetaOffsetDiv2;  //< beta offset for deblocking filter
  int                        m_deblockingFilterCrTcOffsetDiv2;    //< tc offset for deblocking filter
#endif
#if JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
  bool                       m_tsResidualCodingDisabledFlag;
#endif
  int                        m_list1IdxToList0Idx[MAX_NUM_REF];
  int                        m_aiNumRefIdx   [NUM_REF_PIC_LIST_01];    //  for multiple reference of current slice
  bool                       m_pendingRasInit;

  bool                       m_bCheckLDC;

  bool                       m_biDirPred;
  int                        m_symRefIdx[2];

  //  Data
  int                        m_iSliceQpDelta;
  int                        m_iSliceChromaQpDelta[MAX_NUM_COMPONENT+1];
  Picture*                   m_apcRefPicList [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                        m_aiRefPOCList  [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  bool                       m_bIsUsedAsLongTerm[NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                        m_iDepth;
  Picture*                   m_scaledRefPicList[NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
  Picture*                   m_savedRefPicList[NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
  std::pair<int, int>        m_scalingRatio[NUM_REF_PIC_LIST_01][MAX_NUM_REF_PICS];


  // access channel
#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  const DPS*                 m_dps;
#endif
  const VPS*                 m_pcVPS;
  const SPS*                 m_pcSPS;
  const PPS*                 m_pcPPS;
  Picture*                   m_pcPic;
  const PicHeader*           m_pcPicHeader;    //!< pointer to picture header structure
  bool                       m_colFromL0Flag;  // collocated picture from List0 flag

  bool                       m_noIncorrectPicOutputFlag;
  bool                       m_handleCraAsCvsStartFlag;

  uint32_t                       m_colRefIdx;
  double                     m_lambdas[MAX_NUM_COMPONENT];

  bool                       m_abEqualRef  [NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_REF];
  uint32_t                       m_uiTLayer;
  bool                       m_bTLayerSwitchingFlag;

  SliceMap                   m_sliceMap;                     //!< list of CTUs in current slice - raster scan CTU addresses
  uint32_t                       m_independentSliceIdx;
  bool                       m_nextSlice;
  uint32_t                       m_sliceBits;
  bool                       m_bFinalized;


  bool                       m_bTestWeightPred;
  bool                       m_bTestWeightBiPred;
  WPScalingParam             m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT]; // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  WPACDCParam                m_weightACDCParam[MAX_NUM_COMPONENT];
  ClpRngs                    m_clpRngs;
  std::vector<uint32_t>          m_substreamSizes;
  uint32_t                   m_numEntryPoints;
#if JVET_Q0151_Q0205_ENTRYPOINTS
  uint32_t                   m_numSubstream;
#endif

  bool                       m_cabacInitFlag;

  uint32_t                   m_sliceSubPicId;

  
  


  SliceType                  m_encCABACTableIdx;           // Used to transmit table selection across slices.

  clock_t                    m_iProcessingStartTime;
  double                     m_dProcessingTime;
  
  int                        m_rpPicOrderCntVal;
  APS*                       m_alfApss[ALF_CTB_MAX_NUM_APS];
  bool                       m_tileGroupAlfEnabledFlag[MAX_NUM_COMPONENT];
  int                        m_tileGroupNumAps;
  std::vector<int>           m_tileGroupLumaApsId;
  int                        m_tileGroupChromaApsId;
#if JVET_Q0795_CCALF
  bool                       m_tileGroupCcAlfCbEnabledFlag;
  bool                       m_tileGroupCcAlfCrEnabledFlag;
  int                        m_tileGroupCcAlfCbApsId;
  int                        m_tileGroupCcAlfCrApsId;
#endif
  bool                       m_disableSATDForRd;
public:
                              Slice();
  virtual                     ~Slice();
  void                        initSlice();
  void                        inheritFromPicHeader( PicHeader *picHeader, const PPS *pps, const SPS *sps );
  void                        setPicHeader( const PicHeader* pcPicHeader )           { m_pcPicHeader = pcPicHeader;                                  }
  const PicHeader*            getPicHeader() const                                   { return m_pcPicHeader;                                         }
  int                         getRefIdx4MVPair( RefPicList eCurRefPicList, int nCurRefIdx );

#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  void                        setDPS( DPS* dps )                                     { m_dps = dps;                                              }
  const DPS*                  getDPS() const                                         { return m_dps;                                               }
#endif

  void                        setSPS( const SPS* pcSPS )                             { m_pcSPS = pcSPS;                                              }
  const SPS*                  getSPS() const                                         { return m_pcSPS;                                               }
  void                        setVPS( const VPS* pcVPS )                             { m_pcVPS = pcVPS;                                              }
  const VPS*                  getVPS() const                                         { return m_pcVPS;                                               }

  void                        setPPS( const PPS* pcPPS )                             { m_pcPPS = pcPPS;                                              }
  const PPS*                  getPPS() const                                         { return m_pcPPS;                                               }

  void                        setAlfAPSs(APS** apss)                                 { memcpy(m_alfApss, apss, sizeof(m_alfApss));                   }
  APS**                       getAlfAPSs()                                           { return m_alfApss;                                             }
  void                        setSaoEnabledFlag(ChannelType chType, bool s)          {m_saoEnabledFlag[chType] =s;                                   }
  bool                        getSaoEnabledFlag(ChannelType chType) const            { return m_saoEnabledFlag[chType];                              }
  void                        setRPL0(const ReferencePictureList *pcRPL)             { m_pRPL0 = pcRPL;                                             }
  void                        setRPL1(const ReferencePictureList *pcRPL)             { m_pRPL1 = pcRPL;                                             }
  const ReferencePictureList* getRPL0()                                              { return m_pRPL0;                                              }
  const ReferencePictureList* getRPL1()                                              { return m_pRPL1;                                              }
  ReferencePictureList*       getLocalRPL0()                                         { return &m_localRPL0;                                         }
  ReferencePictureList*       getLocalRPL1()                                         { return &m_localRPL1;                                         }
  void                        setRPL0idx(int rplIdx)                                 { m_rpl0Idx = rplIdx;                                          }
  void                        setRPL1idx(int rplIdx)                                 { m_rpl1Idx = rplIdx;                                          }
  int                         getRPL0idx() const                                     { return m_rpl0Idx;                                            }
  int                         getRPL1idx() const                                     { return m_rpl1Idx;                                            }
  void                        setLastIDR(int iIDRPOC)                                { m_iLastIDR = iIDRPOC;                                         }
  int                         getLastIDR() const                                     { return m_iLastIDR;                                            }
  void                        setAssociatedIRAPPOC(int iAssociatedIRAPPOC)           { m_iAssociatedIRAP = iAssociatedIRAPPOC;                       }
  int                         getAssociatedIRAPPOC() const                           { return m_iAssociatedIRAP;                                     }
  void                        setAssociatedIRAPType(NalUnitType associatedIRAPType)  { m_iAssociatedIRAPType = associatedIRAPType;                   }
  NalUnitType                 getAssociatedIRAPType() const                          { return m_iAssociatedIRAPType;                                 }
  SliceType                   getSliceType() const                                   { return m_eSliceType;                                          }
  int                         getPOC() const                                         { return m_iPOC;                                                }
  int                         getSliceQp() const                                     { return m_iSliceQp;                                            }
  bool                        getUseWeightedPrediction() const                       { return( (m_eSliceType==P_SLICE && testWeightPred()) || (m_eSliceType==B_SLICE && testWeightBiPred()) ); }
  int                         getSliceQpDelta() const                                { return m_iSliceQpDelta;                                       }
  int                         getSliceChromaQpDelta(ComponentID compID) const        { return isLuma(compID) ? 0 : m_iSliceChromaQpDelta[compID];    }
  bool                        getUseChromaQpAdj() const                              { return m_ChromaQpAdjEnabled;                                  }
  bool                        getDeblockingFilterDisable() const                     { return m_deblockingFilterDisable;                             }
  bool                        getDeblockingFilterOverrideFlag() const                { return m_deblockingFilterOverrideFlag;                        }
  int                         getDeblockingFilterBetaOffsetDiv2()const               { return m_deblockingFilterBetaOffsetDiv2;                      }
  int                         getDeblockingFilterTcOffsetDiv2() const                { return m_deblockingFilterTcOffsetDiv2;                        }
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
  int                         getDeblockingFilterCbBetaOffsetDiv2()const             { return m_deblockingFilterCbBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCbTcOffsetDiv2() const              { return m_deblockingFilterCbTcOffsetDiv2;                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const             { return m_deblockingFilterCrBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCrTcOffsetDiv2() const              { return m_deblockingFilterCrTcOffsetDiv2;                      }
#endif
  bool                        getPendingRasInit() const                              { return m_pendingRasInit;                                      }
  void                        setPendingRasInit( bool val )                          { m_pendingRasInit = val;                                       }

  int                         getNumRefIdx( RefPicList e ) const                     { return m_aiNumRefIdx[e];                                      }
  Picture*                    getPic()                                               { return m_pcPic;                                               }
  const Picture*              getPic() const                                         { return m_pcPic;                                               }
#if JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY
        Picture*              getRefPic( RefPicList e, int iRefIdx) const            { return m_apcRefPicList[e][iRefIdx];                           }
#else
  const Picture*              getRefPic( RefPicList e, int iRefIdx) const            { return m_apcRefPicList[e][iRefIdx];                           }
#endif
  int                         getRefPOC( RefPicList e, int iRefIdx) const            { return m_aiRefPOCList[e][iRefIdx];                            }
  int                         getDepth() const                                       { return m_iDepth;                                              }
  bool                        getColFromL0Flag() const                               { return m_colFromL0Flag;                                       }
  uint32_t                    getColRefIdx() const                                   { return m_colRefIdx;                                           }
  void                        checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic);
  bool                        getIsUsedAsLongTerm(int i, int j) const                { return m_bIsUsedAsLongTerm[i][j];                             }
  void                        setIsUsedAsLongTerm(int i, int j, bool value)          { m_bIsUsedAsLongTerm[i][j] = value;                            }
  bool                        getCheckLDC() const                                    { return m_bCheckLDC;                                           }
  int                         getList1IdxToList0Idx( int list1Idx ) const            { return m_list1IdxToList0Idx[list1Idx];                        }
  void                        setPOC( int i )                                        { m_iPOC              = i;                                      }
#if JVET_Q0775_PH_IN_SH
  bool                        getPictureHeaderInSliceHeader() const                  { return m_pictureHeaderInSliceHeader;                         }
  void                        setPictureHeaderInSliceHeader( bool e )                { m_pictureHeaderInSliceHeader = e;                            }
#endif
  void                        setNalUnitType( NalUnitType e )                        { m_eNalUnitType      = e;                                      }
  NalUnitType                 getNalUnitType() const                                 { return m_eNalUnitType;                                        }
#if JVET_Q0118_CLEANUPS
  void                        setNalUnitLayerId( uint32_t i )                        { m_nuhLayerId = i;                                             }
  uint32_t                    getNalUnitLayerId() const                              { return m_nuhLayerId;                                          }
#endif
  bool                        getRapPicFlag() const;
  bool                        getIdrPicFlag() const                                  { return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  bool                        isIRAP() const { return (getNalUnitType() >= NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA); }
#if SPS_ID_CHECK
  bool                        isClvssPu() const                                      { return m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_GDR && !m_pcPPS->getMixedNaluTypesInPicFlag(); }
#endif
  bool                        isIDRorBLA() const { return (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP); }
  void                        checkCRA(const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, int& pocCRA, NalUnitType& associatedIRAPType, PicList& rcListPic);
  void                        checkSTSA(PicList& rcListPic);
  void                        decodingRefreshMarking(int& pocCRA, bool& bRefreshPending, PicList& rcListPic, const bool bEfficientFieldIRAPEnabled);
  void                        setSliceType( SliceType e )                            { m_eSliceType        = e;                                      }
  void                        setSliceQp( int i )                                    { m_iSliceQp          = i;                                      }
  void                        setSliceQpDelta( int i )                               { m_iSliceQpDelta     = i;                                      }
  void                        setSliceChromaQpDelta( ComponentID compID, int i )     { m_iSliceChromaQpDelta[compID] = isLuma(compID) ? 0 : i;       }
  void                        setUseChromaQpAdj( bool b )                            { m_ChromaQpAdjEnabled = b;                                     }
  void                        setDeblockingFilterDisable( bool b )                   { m_deblockingFilterDisable= b;                                 }
  void                        setDeblockingFilterOverrideFlag( bool b )              { m_deblockingFilterOverrideFlag = b;                           }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )             { m_deblockingFilterBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterTcOffsetDiv2( int i )               { m_deblockingFilterTcOffsetDiv2 = i;                           }
#if JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )           { m_deblockingFilterCbBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )             { m_deblockingFilterCbTcOffsetDiv2 = i;                           }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )           { m_deblockingFilterCrBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )             { m_deblockingFilterCrTcOffsetDiv2 = i;                           }
#endif
#if JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
  void                        setTSResidualCodingDisabledFlag(bool b) { m_tsResidualCodingDisabledFlag = b; }
  bool                        getTSResidualCodingDisabledFlag() const { return m_tsResidualCodingDisabledFlag; }
#endif

  void                        setNumRefIdx( RefPicList e, int i )                    { m_aiNumRefIdx[e]    = i;                                      }
  void                        setPic( Picture* p )                                   { m_pcPic             = p;                                      }
  void                        setDepth( int iDepth )                                 { m_iDepth            = iDepth;                                 }

  void                        constructRefPicList(PicList& rcListPic);
  void                        setRefPOCList();

  void                        setColFromL0Flag( bool colFromL0 )                     { m_colFromL0Flag = colFromL0;                                  }
  void                        setColRefIdx( uint32_t refIdx)                             { m_colRefIdx = refIdx;                                         }
  void                        setCheckLDC( bool b )                                  { m_bCheckLDC = b;                                              }

  void                        setBiDirPred( bool b, int refIdx0, int refIdx1 ) { m_biDirPred = b; m_symRefIdx[0] = refIdx0; m_symRefIdx[1] = refIdx1; }
  bool                        getBiDirPred() const { return m_biDirPred; }
  int                         getSymRefIdx( int refList ) const { return m_symRefIdx[refList]; }

  bool                        isIntra() const                                        { return m_eSliceType == I_SLICE;                               }
  bool                        isInterB() const                                       { return m_eSliceType == B_SLICE;                               }
  bool                        isInterP() const                                       { return m_eSliceType == P_SLICE;                               }

  bool                        getEnableDRAPSEI () const                              { return m_enableDRAPSEI;                                       }
  void                        setEnableDRAPSEI ( bool b )                            { m_enableDRAPSEI = b;                                          }
  bool                        getUseLTforDRAP () const                               { return m_useLTforDRAP;                                        }
  void                        setUseLTforDRAP ( bool b )                             { m_useLTforDRAP = b;                                           }
  bool                        isDRAP () const                                        { return m_isDRAP;                                              }
  void                        setDRAP ( bool b )                                     { m_isDRAP = b;                                                 }
  void                        setLatestDRAPPOC ( int i )                             { m_latestDRAPPOC = i;                                          }
  int                         getLatestDRAPPOC () const                              { return m_latestDRAPPOC;                                       }
  bool                        cvsHasPreviousDRAP() const                             { return m_latestDRAPPOC != MAX_INT;                            }
  bool                        isPocRestrictedByDRAP( int poc, bool precedingDRAPinDecodingOrder );
  bool                        isPOCInRefPicList( const ReferencePictureList *rpl, int poc );
  void                        checkConformanceForDRAP( uint32_t temporalId );
#if JVET_Q0155_COLOUR_ID
  void                        setColourPlaneId( int id )                             { m_colourPlaneId = id;                                         }
  int                         getColourPlaneId() const                               { return m_colourPlaneId;                                       }
#endif

  void                        setLambdas( const double lambdas[MAX_NUM_COMPONENT] )  { for (int component = 0; component < MAX_NUM_COMPONENT; component++) m_lambdas[component] = lambdas[component]; }
  const double*               getLambdas() const                                     { return m_lambdas;                                             }

  void                        setSliceSubPicId(int i)                               { m_sliceSubPicId = i;   }
  uint32_t                    getSliceSubPicId() const                              { return m_sliceSubPicId; }
  uint32_t                    getCuQpDeltaSubdiv() const                             { return this->isIntra() ? m_pcPicHeader->getCuQpDeltaSubdivIntra() : m_pcPicHeader->getCuQpDeltaSubdivInter(); }
  uint32_t                    getCuChromaQpOffsetSubdiv() const                      { return this->isIntra() ? m_pcPicHeader->getCuChromaQpOffsetSubdivIntra() : m_pcPicHeader->getCuChromaQpOffsetSubdivInter(); }
  void                        initEqualRef();
  bool                        isEqualRef( RefPicList e, int iRefIdx1, int iRefIdx2 )
  {
    CHECK(e>=NUM_REF_PIC_LIST_01, "Invalid reference picture list");
    if (iRefIdx1 < 0 || iRefIdx2 < 0)
    {
      return false;
    }
    else
    {
      return m_abEqualRef[e][iRefIdx1][iRefIdx2];
    }
  }

  void                        setEqualRef( RefPicList e, int iRefIdx1, int iRefIdx2, bool b)
  {
    CHECK( e >= NUM_REF_PIC_LIST_01, "Invalid reference picture list" );
    m_abEqualRef[e][iRefIdx1][iRefIdx2] = m_abEqualRef[e][iRefIdx2][iRefIdx1] = b;
  }

  static void                 sortPicList( PicList& rcListPic );
  void                        setList1IdxToList0Idx();

  uint32_t                        getTLayer() const                                      { return m_uiTLayer;                                            }
  void                        setTLayer( uint32_t uiTLayer )                             { m_uiTLayer = uiTLayer;                                        }

  void                        checkLeadingPictureRestrictions( PicList& rcListPic )                                         const;
#if FIX_TICKET891
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList* pRPL, int rplIdx, bool printErrors, int* refPicIndex, int numActiveRefPics) const;
#else
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList* pRPL, int rplIdx, bool printErrors, int* refPicIndex) const;
#endif
  void                        applyReferencePictureListBasedMarking( PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, const int layerId )  const;
  bool                        isTemporalLayerSwitchingPoint( PicList& rcListPic )                                           const;
  bool                        isStepwiseTemporalLayerSwitchingPointCandidate( PicList& rcListPic )                          const;
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx, bool printErrors)                const;
  void                        setNoIncorrectPicOutputFlag(bool val)                  { m_noIncorrectPicOutputFlag = val;                             }
  bool                        getNoIncorrectPicOutputFlag() const                    { return m_noIncorrectPicOutputFlag;                                    }

  void                        setHandleCraAsCvsStartFlag( bool val )                 { m_handleCraAsCvsStartFlag = val;                                   }
  bool                        getHandleCraAsCvsStartFlag() const                     { return m_handleCraAsCvsStartFlag;                                  }

  void                        setNumTilesInSlice( uint32_t u )                       { m_sliceMap.setNumTilesInSlice( u );                                       }
  uint32_t                    getNumTilesInSlice() const                             { return m_sliceMap.getNumTilesInSlice();                                   }
  void                        setSliceMap( SliceMap map )                            { m_sliceMap = map;                                                         }
  uint32_t                    getFirstCtuRsAddrInSlice() const                       { return m_sliceMap.getCtuAddrInSlice(0);                                   }
  void                        setSliceID( uint32_t u )                               { m_sliceMap.setSliceID( u );                                               }
  uint32_t                    getSliceID() const                                     { return m_sliceMap.getSliceID();                                           }
  uint32_t                    getNumCtuInSlice() const                               { return m_sliceMap.getNumCtuInSlice();                                     }
  uint32_t                    getCtuAddrInSlice( int idx ) const                     { return m_sliceMap.getCtuAddrInSlice( idx );                               }
  void                        initSliceMap()                                         { m_sliceMap.initSliceMap();                                                }
  void                        addCtusToSlice( uint32_t startX, uint32_t stopX, 
                                              uint32_t startY, uint32_t stopY, 
                                              uint32_t picWidthInCtbsY )             { m_sliceMap.addCtusToSlice(startX, stopX, startY, stopY, picWidthInCtbsY); }
  void                        setIndependentSliceIdx( uint32_t i)                        { m_independentSliceIdx = i;                                    }
  uint32_t                        getIndependentSliceIdx() const                         { return  m_independentSliceIdx;                                }
  void                        copySliceInfo(Slice *pcSliceSrc, bool cpyAlmostAll = true);
  void                        setSliceBits( uint32_t uiVal )                             { m_sliceBits = uiVal;                                          }
  uint32_t                        getSliceBits() const                                   { return m_sliceBits;                                           }
  void                        setFinalized( bool uiVal )                             { m_bFinalized = uiVal;                                         }
  bool                        getFinalized() const                                   { return m_bFinalized;                                          }
  bool                        testWeightPred( ) const                                { return m_bTestWeightPred;                                     }
  void                        setTestWeightPred( bool bValue )                       { m_bTestWeightPred = bValue;                                   }
  bool                        testWeightBiPred( ) const                              { return m_bTestWeightBiPred;                                   }
  void                        setTestWeightBiPred( bool bValue )                     { m_bTestWeightBiPred = bValue;                                 }
  void                        setWpScaling( WPScalingParam  wp[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT] )
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam)*NUM_REF_PIC_LIST_01*MAX_NUM_REF*MAX_NUM_COMPONENT);
  }
#if JVET_Q0819_PH_CHANGES  
  void                        setWpScaling(WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT);
  }
  WPScalingParam *            getWpScalingAll()                                      { return (WPScalingParam *) m_weightPredTable;                  }
#endif
  void                        getWpScaling( RefPicList e, int iRefIdx, WPScalingParam *&wp) const;

  void                        resetWpScaling();
  void                        initWpScaling(const SPS *sps);

  void                        setWpAcDcParam( WPACDCParam wp[MAX_NUM_COMPONENT] )    { memcpy(m_weightACDCParam, wp, sizeof(WPACDCParam)*MAX_NUM_COMPONENT); }

  void                        getWpAcDcParam( const WPACDCParam *&wp ) const;
  void                        initWpAcDcParam();

  void                        clearSubstreamSizes( )                                 { return m_substreamSizes.clear();                              }
  uint32_t                        getNumberOfSubstreamSizes( )                           { return (uint32_t) m_substreamSizes.size();                        }
  void                        addSubstreamSize( uint32_t size )                          { m_substreamSizes.push_back(size);                             }
  uint32_t                        getSubstreamSize( uint32_t idx )                           { CHECK(idx>=getNumberOfSubstreamSizes(),"Invalid index"); return m_substreamSizes[idx]; }
#if JVET_Q0151_Q0205_ENTRYPOINTS
  void                        resetNumberOfSubstream()                               { m_numSubstream = 0;                                           }
  uint32_t                    getNumberOfSubstream()                                 { return (uint32_t) m_numSubstream;                             }
  void                        increaseNumberOfSubstream()                            { m_numSubstream++;                                             }
#endif

  void                        setCabacInitFlag( bool val )                           { m_cabacInitFlag = val;                                        } //!< set CABAC initial flag
  bool                        getCabacInitFlag()                               const { return m_cabacInitFlag;                                       } //!< get CABAC initial flag

  void                        setEncCABACTableIdx( SliceType idx )                   { m_encCABACTableIdx = idx;                                     }
  SliceType                   getEncCABACTableIdx() const                            { return m_encCABACTableIdx;                                    }


  void                        setSliceQpBase( int i )                                { m_iSliceQpBase = i;                                           }
  int                         getSliceQpBase()                                 const { return m_iSliceQpBase;                                        }

  void                        setDefaultClpRng( const SPS& sps );
  const ClpRngs&              clpRngs()                                         const { return m_clpRngs;}
  const ClpRng&               clpRng( ComponentID id)                           const { return m_clpRngs.comp[id];}
  ClpRngs&                    getClpRngs()                                            { return m_clpRngs;}
  unsigned                    getMinPictureDistance()                           const ;
  void startProcessingTimer();
  void stopProcessingTimer();
  void resetProcessingTime()       { m_dProcessingTime = m_iProcessingStartTime = 0; }
  double getProcessingTime() const { return m_dProcessingTime; }

  void                        resetTileGroupAlfEnabledFlag() { memset(m_tileGroupAlfEnabledFlag, 0, sizeof(m_tileGroupAlfEnabledFlag)); }
  bool                        getTileGroupAlfEnabledFlag(ComponentID compId) const { return m_tileGroupAlfEnabledFlag[compId]; }
  void                        setTileGroupAlfEnabledFlag(ComponentID compId, bool b) { m_tileGroupAlfEnabledFlag[compId] = b; }
  int                         getTileGroupNumAps() const { return m_tileGroupNumAps; }
  void                        setTileGroupNumAps(int i) { m_tileGroupNumAps = i; }
  int                         getTileGroupApsIdChroma() const { return m_tileGroupChromaApsId; }
  void                        setTileGroupApsIdChroma(int i) { m_tileGroupChromaApsId = i; }
  std::vector<int32_t>        getTileGroupApsIdLuma() const { return m_tileGroupLumaApsId; }
  void                        setAlfAPSs(std::vector<int> ApsIDs)
  {
    m_tileGroupLumaApsId.resize(m_tileGroupNumAps);
    for (int i = 0; i < m_tileGroupNumAps; i++)
    {
      m_tileGroupLumaApsId[i] = ApsIDs[i];
    }
  }
#if JVET_Q0795_CCALF
  void resetTileGroupCcAlCbfEnabledFlag() { m_tileGroupCcAlfCbEnabledFlag = 0; }
  void resetTileGroupCcAlCrfEnabledFlag() { m_tileGroupCcAlfCrEnabledFlag = 0; }

  void setTileGroupCcAlfCbEnabledFlag(bool b) { m_tileGroupCcAlfCbEnabledFlag = b; }
  void setTileGroupCcAlfCrEnabledFlag(bool b) { m_tileGroupCcAlfCrEnabledFlag = b; }
  void setTileGroupCcAlfCbApsId(int i) { m_tileGroupCcAlfCbApsId = i; }
  void setTileGroupCcAlfCrApsId(int i) { m_tileGroupCcAlfCrApsId = i; }

  bool getTileGroupCcAlfCbEnabledFlag() { return m_tileGroupCcAlfCbEnabledFlag; }
  bool getTileGroupCcAlfCrEnabledFlag() { return m_tileGroupCcAlfCrEnabledFlag; }
  int  getTileGroupCcAlfCbApsId() { return m_tileGroupCcAlfCbApsId; }
  int  getTileGroupCcAlfCrApsId() { return m_tileGroupCcAlfCrApsId; }
#endif
  void                        setDisableSATDForRD(bool b) { m_disableSATDForRd = b; }
  bool                        getDisableSATDForRD() { return m_disableSATDForRd; }
  void                        scaleRefPicList( Picture *scaledRefPic[ ], PicHeader *picHeader, APS** apss, APS* lmcsAps, APS* scalingListAps, const bool isDecoder );
  void                        freeScaledRefPicList( Picture *scaledRefPic[] );
  bool                        checkRPR();
  const std::pair<int, int>&  getScalingRatio( const RefPicList refPicList, const int refIdx )  const { CHECK( refIdx < 0, "Invalid reference index" ); return m_scalingRatio[refPicList][refIdx]; }
#if JVET_Q0151_Q0205_ENTRYPOINTS
  void                        setNumSubstream( const SPS *sps, const PPS *pps );
  void                        setNumEntryPoints( const SPS *sps, const PPS *pps );
#else
  void                        setNumEntryPoints( const PPS *pps );
#endif
  uint32_t                    getNumEntryPoints( ) const { return m_numEntryPoints;  }

#if JVET_Q0795_CCALF
  CcAlfFilterParam            m_ccAlfFilterParam;
  uint8_t*                    m_ccAlfFilterControl[2];
#endif

protected:
  Picture*              xGetRefPic( PicList& rcListPic, int poc, const int layerId );
  Picture*              xGetLongTermRefPic( PicList& rcListPic, int poc, bool pocHasMsb, const int layerId );
public:
#if JVET_Q0504_PLT_NON444
  std::unordered_map< Position, std::unordered_map< Size, double> > m_mapPltCost[2];
#else
  std::unordered_map< Position, std::unordered_map< Size, double> > m_mapPltCost;
#endif
private:
};// END CLASS DEFINITION Slice

void calculateParameterSetChangedFlag(bool &bChanged, const std::vector<uint8_t> *pOldData, const std::vector<uint8_t> *pNewData);

template <class T> class ParameterSetMap
{
public:
  template <class Tm>
  struct MapData
  {
    bool                  bChanged;
    std::vector<uint8_t>   *pNaluData; // Can be null
    Tm*                   parameterSet;
  };

  ParameterSetMap(int maxId)
  :m_maxId (maxId)
  ,m_lastActiveParameterSet(NULL)
  {
    m_activePsId.clear();
  }

  ~ParameterSetMap()
  {
    for (typename std::map<int,MapData<T> >::iterator i = m_paramsetMap.begin(); i!= m_paramsetMap.end(); i++)
    {
      delete (*i).second.pNaluData;
      delete (*i).second.parameterSet;
    }
    delete m_lastActiveParameterSet; m_lastActiveParameterSet = NULL;
  }

  T *allocatePS(const int psId)
  {
    CHECK( psId >= m_maxId, "Invalid PS id" );
    if ( m_paramsetMap.find(psId) == m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged = true;
      m_paramsetMap[psId].pNaluData=0;
      m_paramsetMap[psId].parameterSet = new T;
      setID(m_paramsetMap[psId].parameterSet, psId);
    }
    return m_paramsetMap[psId].parameterSet;
  }

  void clearMap()
  {
    m_paramsetMap.clear();
  }

  void storePS(int psId, T *ps, const std::vector<uint8_t> *pNaluData)
  {
    CHECK( psId >= m_maxId, "Invalid PS id" );
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      MapData<T> &mapData=m_paramsetMap[psId];

      // work out changed flag
      calculateParameterSetChangedFlag(mapData.bChanged, mapData.pNaluData, pNaluData);

      if( ! mapData.bChanged )
      {
        // just keep the old one
        delete ps;
        return;
      }

      if (find(m_activePsId.begin(), m_activePsId.end(), psId) != m_activePsId.end())
      {
        std::swap( m_paramsetMap[psId].parameterSet, m_lastActiveParameterSet );
      }
      delete m_paramsetMap[psId].pNaluData;
      delete m_paramsetMap[psId].parameterSet;

      m_paramsetMap[psId].parameterSet = ps;
    }
    else
    {
      m_paramsetMap[psId].parameterSet = ps;
      m_paramsetMap[psId].bChanged = false;
    }
    if (pNaluData != 0)
    {
      m_paramsetMap[psId].pNaluData=new std::vector<uint8_t>;
      *(m_paramsetMap[psId].pNaluData) = *pNaluData;
    }
    else
    {
      m_paramsetMap[psId].pNaluData=0;
    }
  }

  void checkAuApsContent( APS *aps, std::vector<int>& accessUnitApsNals )
  {
    int apsId = ( aps->getAPSId() << NUM_APS_TYPE_LEN ) + (int)aps->getAPSType();

    if( std::find( accessUnitApsNals.begin(), accessUnitApsNals.end(), apsId ) != accessUnitApsNals.end() )
    {
      CHECK( m_paramsetMap.find( apsId ) == m_paramsetMap.end(), "APS does not exist" );
      APS* existedAPS = m_paramsetMap[apsId].parameterSet;

      if( aps->getAPSType() == LMCS_APS )
      {
        CHECK( aps->getReshaperAPSInfo() != existedAPS->getReshaperAPSInfo(), "All APS NAL units with a particular value of adaptation_parameter_set_id and a particular value of aps_params_type within an access unit shall have the same content" );
      }
      else if( aps->getAPSType() == ALF_APS )
      {
        CHECK( aps->getAlfAPSParam() != existedAPS->getAlfAPSParam(), "All APS NAL units with a particular value of adaptation_parameter_set_id and a particular value of aps_params_type within an access unit shall have the same content" );
      }
      else if( aps->getAPSType() == SCALING_LIST_APS )
      {
        CHECK( aps->getScalingList() != existedAPS->getScalingList(), "All APS NAL units with a particular value of adaptation_parameter_set_id and a particular value of aps_params_type within an access unit shall have the same content" );
      }
      else
      {
        CHECK( true, "Wrong APS type" );
      }
    }
    else
    {
      accessUnitApsNals.push_back( apsId );
    }
  }


  void setChangedFlag(int psId, bool bChanged=true)
  {
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged=bChanged;
    }
  }

  void clearChangedFlag(int psId)
  {
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged=false;
    }
  }

  bool getChangedFlag(int psId) const
  {
    const typename std::map<int,MapData<T> >::const_iterator constit=m_paramsetMap.find(psId);
    if ( constit != m_paramsetMap.end() )
    {
      return constit->second.bChanged;
    }
    return false;
  }

  T* getPS(int psId)
  {
    typename std::map<int,MapData<T> >::iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  const T* getPS(int psId) const
  {
    typename std::map<int,MapData<T> >::const_iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  T* getFirstPS()
  {
    return (m_paramsetMap.begin() == m_paramsetMap.end() ) ? NULL : m_paramsetMap.begin()->second.parameterSet;
  }

  void setActive(int psId) { m_activePsId.push_back(psId); }
  void clear() { m_activePsId.clear(); }

private:
  std::map<int,MapData<T> > m_paramsetMap;
  int                       m_maxId;
  std::vector<int>          m_activePsId;
  T*                        m_lastActiveParameterSet;
  static void setID(T* parameterSet, const int psId);
};

class ParameterSetManager
{
public:
                 ParameterSetManager();
  virtual        ~ParameterSetManager();

  void           storeVPS(VPS *vps, const std::vector<uint8_t> &naluData)    { m_vpsMap.storePS(vps->getVPSId(), vps, &naluData); }
  VPS*           getVPS( int vpsId )                                         { return m_vpsMap.getPS( vpsId ); };

#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  void           storeDPS(DPS *dps, const std::vector<uint8_t> &naluData)    { m_dpsMap.storePS( dps->getDecodingParameterSetId(), dps, &naluData); };
  //! get pointer to existing video parameter set
  DPS*           getDPS(int dpsId)                                           { return m_dpsMap.getPS(dpsId); };
  bool           getDPSChangedFlag(int dpsId) const                          { return m_dpsMap.getChangedFlag(dpsId); }
  void           clearDPSChangedFlag(int dpsId)                              { m_dpsMap.clearChangedFlag(dpsId); }
  DPS*           getFirstDPS()                                               { return m_dpsMap.getFirstPS(); };
#endif
  //! store sequence parameter set and take ownership of it
  void           storeSPS(SPS *sps, const std::vector<uint8_t> &naluData) { m_spsMap.storePS( sps->getSPSId(), sps, &naluData); };
  //! get pointer to existing sequence parameter set
  SPS*           getSPS(int spsId)                                           { return m_spsMap.getPS(spsId); };
  bool           getSPSChangedFlag(int spsId) const                          { return m_spsMap.getChangedFlag(spsId); }
  void           clearSPSChangedFlag(int spsId)                              { m_spsMap.clearChangedFlag(spsId); }
  SPS*           getFirstSPS()                                               { return m_spsMap.getFirstPS(); };

  //! store picture parameter set and take ownership of it
  void           storePPS(PPS *pps, const std::vector<uint8_t> &naluData) { m_ppsMap.storePS( pps->getPPSId(), pps, &naluData); };
  //! get pointer to existing picture parameter set
  PPS*           getPPS(int ppsId)                                           { return m_ppsMap.getPS(ppsId); };
  bool           getPPSChangedFlag(int ppsId) const                          { return m_ppsMap.getChangedFlag(ppsId); }
  void           clearPPSChangedFlag(int ppsId)                              { m_ppsMap.clearChangedFlag(ppsId); }
  PPS*           getFirstPPS()                                               { return m_ppsMap.getFirstPS(); };

  //! activate a SPS from a active parameter sets SEI message
  //! \returns true, if activation is successful
  // bool           activateSPSWithSEI(int SPSId);

  //! activate a PPS and depending on isIDR parameter also SPS
  //! \returns true, if activation is successful
  bool           activatePPS(int ppsId, bool isIRAP);
  APS**          getAPSs() { return &m_apss[0]; }
  ParameterSetMap<APS>* getApsMap() { return &m_apsMap; }
  void           storeAPS(APS *aps, const std::vector<uint8_t> &naluData)    { m_apsMap.storePS(aps->getAPSId() + (MAX_NUM_APS * aps->getAPSType()), aps, &naluData); };
  APS*           getAPS(int apsId, int apsType)                              { return m_apsMap.getPS(apsId + (MAX_NUM_APS * apsType)); };
  bool           getAPSChangedFlag(int apsId, int apsType) const             { return m_apsMap.getChangedFlag(apsId + (MAX_NUM_APS * apsType)); }
  void           clearAPSChangedFlag(int apsId, int apsType)                 { m_apsMap.clearChangedFlag(apsId + ( MAX_NUM_APS * apsType)); }
  APS*           getFirstAPS()                                               { return m_apsMap.getFirstPS(); };
  bool           activateAPS(int apsId, int apsType);
  const SPS*     getActiveSPS()const                                         { return m_spsMap.getPS(m_activeSPSId); };
#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  const DPS*     getActiveDPS()const                                         { return m_dpsMap.getPS(m_activeDPSId); };
#endif
  void           checkAuApsContent( APS *aps, std::vector<int>& accessUnitApsNals ) { m_apsMap.checkAuApsContent( aps, accessUnitApsNals ); }

protected:
  ParameterSetMap<SPS> m_spsMap;
  ParameterSetMap<PPS> m_ppsMap;
  ParameterSetMap<APS> m_apsMap;
#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  ParameterSetMap<DPS> m_dpsMap;
#endif
  ParameterSetMap<VPS> m_vpsMap;

  APS* m_apss[ALF_CTB_MAX_NUM_APS];
#if !JVET_Q0117_PARAMETER_SETS_CLEANUP
  int m_activeDPSId; // -1 for nothing active
#endif
  int m_activeSPSId; // -1 for nothing active
  int m_activeVPSId; // -1 for nothing active
};

class PreCalcValues
{
public:
  PreCalcValues( const SPS& sps, const PPS& pps, bool _isEncoder )
    : chrFormat           ( sps.getChromaFormatIdc() )
    , multiBlock422       ( false )
    , maxCUWidth          ( sps.getMaxCUWidth() )
    , maxCUHeight         ( sps.getMaxCUHeight() )
    , maxCUWidthMask      ( maxCUWidth  - 1 )
    , maxCUHeightMask     ( maxCUHeight - 1 )
    , maxCUWidthLog2      ( floorLog2( maxCUWidth  ) )
    , maxCUHeightLog2     ( floorLog2( maxCUHeight ) )
#if JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
    , minCUWidth          ( 1 << MIN_CU_LOG2 )
    , minCUHeight         ( 1 << MIN_CU_LOG2 )
#else
    , minCUWidth          ( sps.getMaxCUWidth()  >> sps.getMaxCodingDepth() )
    , minCUHeight         ( sps.getMaxCUHeight() >> sps.getMaxCodingDepth() )
#endif
    , minCUWidthLog2      ( floorLog2( minCUWidth  ) )
    , minCUHeightLog2     ( floorLog2( minCUHeight ) )
#if JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
    , partsInCtuWidth     ( maxCUWidth >> MIN_CU_LOG2)
    , partsInCtuHeight    ( maxCUHeight >> MIN_CU_LOG2)
    , partsInCtu          ( partsInCtuWidth * partsInCtuHeight )
#else
    , partsInCtuWidth     ( 1 << sps.getMaxCodingDepth() )
    , partsInCtuHeight    ( 1 << sps.getMaxCodingDepth() )
    , partsInCtu          ( 1 << (sps.getMaxCodingDepth() << 1) )
#endif
    , widthInCtus         ( (pps.getPicWidthInLumaSamples () + sps.getMaxCUWidth () - 1) / sps.getMaxCUWidth () )
    , heightInCtus        ( (pps.getPicHeightInLumaSamples() + sps.getMaxCUHeight() - 1) / sps.getMaxCUHeight() )
    , sizeInCtus          ( widthInCtus * heightInCtus )
    , lumaWidth           ( pps.getPicWidthInLumaSamples() )
    , lumaHeight          ( pps.getPicHeightInLumaSamples() )
#if JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX
    , fastDeltaQPCuMaxSize( Clip3(1u << sps.getLog2MinCodingBlockSize(), sps.getMaxCUHeight(), 32u) )
#else
    , fastDeltaQPCuMaxSize( Clip3(sps.getMaxCUHeight() >> (sps.getLog2DiffMaxMinCodingBlockSize()), sps.getMaxCUHeight(), 32u) )
#endif
    , noChroma2x2         (  false )
    , isEncoder           ( _isEncoder )
    , ISingleTree         ( !sps.getUseDualITree() )
    , maxBtDepth          { sps.getMaxMTTHierarchyDepthI(), sps.getMaxMTTHierarchyDepth(), sps.getMaxMTTHierarchyDepthIChroma() }
    , minBtSize           { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() }
    , maxBtSize           { sps.getMaxBTSizeI(), sps.getMaxBTSize(), sps.getMaxBTSizeIChroma() }
    , minTtSize           { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() }
    , maxTtSize           { sps.getMaxTTSizeI(), sps.getMaxTTSize(), sps.getMaxTTSizeIChroma() }
    , minQtSize           { sps.getMinQTSize(I_SLICE, CHANNEL_TYPE_LUMA), sps.getMinQTSize(B_SLICE, CHANNEL_TYPE_LUMA), sps.getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA) }
  {}

  const ChromaFormat chrFormat;
  const bool         multiBlock422;
  const unsigned     maxCUWidth;
  const unsigned     maxCUHeight;
  // to get CTU position, use (x & maxCUWidthMask) rather than (x % maxCUWidth)
  const unsigned     maxCUWidthMask;
  const unsigned     maxCUHeightMask;
  const unsigned     maxCUWidthLog2;
  const unsigned     maxCUHeightLog2;
  const unsigned     minCUWidth;
  const unsigned     minCUHeight;
  const unsigned     minCUWidthLog2;
  const unsigned     minCUHeightLog2;
  const unsigned     partsInCtuWidth;
  const unsigned     partsInCtuHeight;
  const unsigned     partsInCtu;
  const unsigned     widthInCtus;
  const unsigned     heightInCtus;
  const unsigned     sizeInCtus;
  const unsigned     lumaWidth;
  const unsigned     lumaHeight;
  const unsigned     fastDeltaQPCuMaxSize;
  const bool         noChroma2x2;
  const bool         isEncoder;
  const bool         ISingleTree;

private:
  const unsigned     maxBtDepth[3];
  const unsigned     minBtSize [3];
  const unsigned     maxBtSize [3];
  const unsigned     minTtSize [3];
  const unsigned     maxTtSize [3];
  const unsigned     minQtSize [3];

  unsigned getValIdx    ( const Slice &slice, const ChannelType chType ) const;

public:
  unsigned getMaxBtDepth( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinQtSize ( const Slice &slice, const ChannelType chType ) const;
};

#if ENABLE_TRACING
void xTraceVPSHeader();
void xTraceDPSHeader();
void xTraceSPSHeader();
void xTracePPSHeader();
void xTraceAPSHeader();
void xTracePictureHeader();
void xTraceSliceHeader();
void xTraceAccessUnitDelimiter();
#endif

#endif // __SLICE__
