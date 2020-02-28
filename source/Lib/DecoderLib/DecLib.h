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

/** \file     DecLib.h
    \brief    decoder class (header)
*/

#ifndef __DECLIB__
#define __DECLIB__

#include "DecSlice.h"
#include "CABACReader.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CacheModel.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Reshape.h"

class InputNALUnit;

//! \ingroup DecoderLib
//! \{

bool tryDecodePicture( Picture* pcPic, const int expectedPoc, const std::string& bitstreamFileName, bool bDecodeUntilPocFound = false, int debugCTU = -1, int debugPOC = -1 );
// Class definition
// ====================================================================================================================

/// decoder class
class DecLib
{
private:
  int                     m_iMaxRefPicNum;

  NalUnitType             m_associatedIRAPType; ///< NAL unit type of the associated IRAP picture
  int                     m_pocCRA;            ///< POC number of the latest CRA picture
  int                     m_pocRandomAccess;   ///< POC number of the random access point (the first IDR or CRA picture)
  int                     m_lastRasPoc;

  PicList                 m_cListPic;         //  Dynamic buffer
  ParameterSetManager     m_parameterSetManager;  // storage for parameter sets
  PicHeader               m_picHeader;            // picture header
  Slice*                  m_apcSlicePilot;


  SEIMessages             m_SEIs; ///< List of SEI messages that have been received before the first slice and between slices, excluding prefix SEIs...


  // functional classes
  IntraPrediction         m_cIntraPred;
  InterPrediction         m_cInterPred;
  TrQuant                 m_cTrQuant;
  DecSlice                m_cSliceDecoder;
  TrQuant                 m_cTrQuantScalingList;
  DecCu                   m_cCuDecoder;
  HLSyntaxReader          m_HLSReader;
  CABACDecoder            m_CABACDecoder;
  SEIReader               m_seiReader;
  LoopFilter              m_cLoopFilter;
  SampleAdaptiveOffset    m_cSAO;
  AdaptiveLoopFilter      m_cALF;
  Reshape                 m_cReshaper;                        ///< reshaper class
  HRD                     m_HRD;
  // decoder side RD cost computation
  RdCost                  m_cRdCost;                      ///< RD cost computation class
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel              m_cacheModel;
#endif
  bool isRandomAccessSkipPicture(int& iSkipFrame,  int& iPOCLastDisplay);
  Picture*                m_pcPic;
  uint32_t                m_uiSliceSegmentIdx;
  uint32_t                m_prevLayerID;
  int                     m_prevPOC;
  int                     m_prevTid0POC;
  bool                    m_bFirstSliceInPicture;
#if JVET_P0125_EOS_LAYER_SPECIFIC
  bool                    m_bFirstSliceInSequence[MAX_VPS_LAYERS];
#else
  bool                    m_bFirstSliceInSequence;
#endif
  bool                    m_prevSliceSkipped;
  int                     m_skippedPOC;
  bool                    m_bFirstSliceInBitstream;
  int                     m_lastPOCNoOutputPriorPics;
  bool                    m_isNoOutputPriorPics;
  bool                    m_lastNoIncorrectPicOutputFlag;    //value of variable NoIncorrectPicOutputFlag of the last CRA / GDR pic
  int                     m_sliceLmcsApsId;         //value of LmcsApsId, constraint is same id for all slices in one picture
  std::ostream           *m_pDecodedSEIOutputStream;

  int                     m_decodedPictureHashSEIEnabled;  ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  uint32_t                    m_numberOfChecksumErrorsDetected;

  bool                    m_warningMessageSkipPicture;

  std::list<InputNALUnit*> m_prefixSEINALUs; /// Buffered up prefix SEI NAL Units.
  int                     m_debugPOC;
  int                     m_debugCTU;

  std::vector<std::pair<NalUnitType, int>> m_accessUnitNals;
  #if JVET_P0101_POC_MULTILAYER
  struct AccessUnitPicInfo
  {
    NalUnitType     m_nalUnitType; ///< nal_unit_type
    uint32_t        m_temporalId;  ///< temporal_id
    uint32_t        m_nuhLayerId;  ///< nuh_layer_id
    int             m_POC;
  };
  std::vector<AccessUnitPicInfo> m_accessUnitPicInfo;
  #endif
  std::vector<int> m_accessUnitApsNals;
#if JVET_P0125_ASPECT_TID_LAYER_ID_NUH
  std::vector<int> m_accessUnitSeiTids;
#endif

  VPS*                    m_vps;
  bool                    m_scalingListUpdateFlag;
  int                     m_PreScalingListAPSId;
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  int                     m_maxDecSubPicIdx;
  int                     m_maxDecSliceAddrInSubPic;
#endif

#if JVET_O1143_SUBPIC_BOUNDARY
public:
  int                     m_targetSubPicIdx;
#endif

#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  DCI*                    m_dci;
#endif
public:
  DecLib();
  virtual ~DecLib();

  void  create  ();
  void  destroy ();

  void  setDecodedPictureHashSEIEnabled(int enabled) { m_decodedPictureHashSEIEnabled=enabled; }

  void  init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    const std::string& cacheCfgFileName
#endif
  );
#if JVET_P0288_PIC_OUTPUT
  bool  decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay, int iTargetOlsIdx);
#else
  bool  decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay);
#endif
  void  deletePicBuffer();

  void  executeLoopFilters();
  void  finishPicture(int& poc, PicList*& rpcListPic, MsgLevel msgl = INFO);
  void  finishPictureLight(int& poc, PicList*& rpcListPic );
  void  checkNoOutputPriorPics (PicList* rpcListPic);
  void  checkNalUnitConstraints( uint32_t naluType );


  bool  getNoOutputPriorPicsFlag () const   { return m_isNoOutputPriorPics; }
  void  setNoOutputPriorPicsFlag (bool val) { m_isNoOutputPriorPics = val; }
  void  setFirstSliceInPicture (bool val)  { m_bFirstSliceInPicture = val; }
  bool  getFirstSliceInPicture () const  { return m_bFirstSliceInPicture; }
#if JVET_P0125_EOS_LAYER_SPECIFIC
  bool  getFirstSliceInSequence(int layer) const { return m_bFirstSliceInSequence[layer]; }
  void  setFirstSliceInSequence(bool val, int layer) { m_bFirstSliceInSequence[layer] = val; }
#else
  bool  getFirstSliceInSequence () const   { return m_bFirstSliceInSequence; }
  void  setFirstSliceInSequence (bool val) { m_bFirstSliceInSequence = val; }
#endif
  void  setDecodedSEIMessageOutputStream(std::ostream *pOpStream) { m_pDecodedSEIOutputStream = pOpStream; }
  uint32_t  getNumberOfChecksumErrorsDetected() const { return m_numberOfChecksumErrorsDetected; }

  int  getDebugCTU( )               const { return m_debugCTU; }
  void setDebugCTU( int debugCTU )        { m_debugCTU = debugCTU; }
  int  getDebugPOC( )               const { return m_debugPOC; };
  void setDebugPOC( int debugPOC )        { m_debugPOC = debugPOC; };
  void resetAccessUnitNals()              { m_accessUnitNals.clear();    }
#if JVET_P0101_POC_MULTILAYER
  void resetAccessUnitPicInfo()              { m_accessUnitPicInfo.clear();    }
#endif
  void resetAccessUnitApsNals()           { m_accessUnitApsNals.clear(); }
#if JVET_P0125_ASPECT_TID_LAYER_ID_NUH
  void resetAccessUnitSeiTids()           { m_accessUnitSeiTids.clear(); }
  void checkTidLayerIdInAccessUnit();
#endif
  bool isSliceNaluFirstInAU( bool newPicture, InputNALUnit &nalu );

  const VPS* getVPS()                     { return m_vps; }
#if JVET_Q0814_DPB
  void deriveTargetOutputLayerSet( const int targetOlsIdx ) { if( m_vps != nullptr ) m_vps->deriveTargetOutputLayerSet( targetOlsIdx ); }
#endif

  void  initScalingList()
  {
    m_cTrQuantScalingList.init(nullptr, MAX_TB_SIZEY, false, false, false, false);
  }
  bool  getScalingListUpdateFlag() { return m_scalingListUpdateFlag; }
  void  setScalingListUpdateFlag(bool b) { m_scalingListUpdateFlag = b; }
  int   getPreScalingListAPSId() { return m_PreScalingListAPSId; }
  void  setPreScalingListAPSId(int id) { m_PreScalingListAPSId = id; }

protected:
  void  xUpdateRasInit(Slice* slice);

  Picture * xGetNewPicBuffer( const SPS &sps, const PPS &pps, const uint32_t temporalLayer, const int layerId );
  void  xCreateLostPicture( int iLostPOC, const int layerId );
  void  xCreateUnavailablePicture(int iUnavailablePoc, bool longTermFlag, const int layerId, const bool interLayerRefPicFlag);
  void  xActivateParameterSets( const int layerId );
  void  xCheckParameterSetConstraints( const int layerId );
  void      xDecodePicHeader( InputNALUnit& nalu );
  bool      xDecodeSlice(InputNALUnit &nalu, int &iSkipFrame, int iPOCLastDisplay);
  void      xDecodeVPS( InputNALUnit& nalu );
#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  void      xDecodeDCI( InputNALUnit& nalu );
#else
  void      xDecodeDPS( InputNALUnit& nalu );
#endif
  void      xDecodeSPS( InputNALUnit& nalu );
  void      xDecodePPS( InputNALUnit& nalu );
  void      xDecodeAPS(InputNALUnit& nalu);
  void      xUpdatePreviousTid0POC(Slice *pSlice) { if ((pSlice->getTLayer() == 0) && (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RASL) && (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RADL))  { m_prevTid0POC = pSlice->getPOC(); }  }
  void      xParsePrefixSEImessages();
  void      xParsePrefixSEIsForUnknownVCLNal();

  void  xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType );

};// END CLASS DEFINITION DecLib


//! \}

#endif // __DECTOP__

