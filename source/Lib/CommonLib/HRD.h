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


#ifndef __HRD__
#define __HRD__

#include "Common.h"
#include "SEI.h"

class TimingInfo
{
protected:
  bool     m_timingInfoPresentFlag;
  uint32_t m_numUnitsInTick;
  uint32_t m_timeScale;
  int      m_numTicksPocDiffOneMinus1;

public:
  TimingInfo()
    : m_timingInfoPresentFlag      (false)
    , m_numUnitsInTick             (1001)
    , m_timeScale                  (60000)
    , m_numTicksPocDiffOneMinus1   (0)
  {}

  void     setTimingInfoPresentFlag( bool flag )   { m_timingInfoPresentFlag = flag;       }
  bool     getTimingInfoPresentFlag( ) const       { return m_timingInfoPresentFlag;       }

  void     setNumUnitsInTick( uint32_t value )     { m_numUnitsInTick = value;             }
  uint32_t getNumUnitsInTick( ) const              { return m_numUnitsInTick;              }
  void     setTimeScale( uint32_t value )          { m_timeScale = value;                  }
  uint32_t getTimeScale( ) const                   { return m_timeScale;                   }

  void     setNumTicksPocDiffOneMinus1(int x)      { m_numTicksPocDiffOneMinus1 = x;       }
  int      getNumTicksPocDiffOneMinus1( ) const    { return m_numTicksPocDiffOneMinus1;    }
};

struct HrdSubLayerInfo
{
  bool     fixedPicRateFlag;
  bool     fixedPicRateWithinCvsFlag;
  uint32_t picDurationInTcMinus1;
  bool     lowDelayHrdFlag;
  uint32_t cpbCntMinus1;
  uint32_t bitRateValueMinus1[MAX_CPB_CNT][2];
  uint32_t cpbSizeValue      [MAX_CPB_CNT][2];
  uint32_t ducpbSizeValue    [MAX_CPB_CNT][2];
  bool     cbrFlag           [MAX_CPB_CNT][2];
  uint32_t duBitRateValue    [MAX_CPB_CNT][2];
};

class HRDParameters
{
private:
  bool     m_nalHrdParametersPresentFlag;
  bool     m_vclHrdParametersPresentFlag;
  uint32_t m_tickDivisorMinus2;
  bool     m_generalDecodingUnitHrdParamsPresentFlag;
  uint32_t m_bitRateScale;
  uint32_t m_cpbSizeScale;
  uint32_t m_cpbSizeDuScale;
  HrdSubLayerInfo m_HRD[MAX_TLAYER];

public:
  HRDParameters()
    :m_nalHrdParametersPresentFlag       (false)
    ,m_vclHrdParametersPresentFlag       (false)
    ,m_tickDivisorMinus2                 (0)
    ,m_generalDecodingUnitHrdParamsPresentFlag  (false)
    ,m_bitRateScale                      (0)
    ,m_cpbSizeScale                      (0)
    ,m_cpbSizeDuScale                    (0)
  {}

  virtual ~HRDParameters() {}

  void      setNalHrdParametersPresentFlag( bool flag )                                { m_nalHrdParametersPresentFlag = flag;                      }
  bool      getNalHrdParametersPresentFlag( ) const                                    { return m_nalHrdParametersPresentFlag;                      }

  void      setVclHrdParametersPresentFlag( bool flag )                                { m_vclHrdParametersPresentFlag = flag;                      }
  bool      getVclHrdParametersPresentFlag( ) const                                    { return m_vclHrdParametersPresentFlag;                      }


  void      setTickDivisorMinus2( uint32_t value )                                     { m_tickDivisorMinus2 = value;                               }
  uint32_t  getTickDivisorMinus2( ) const                                              { return m_tickDivisorMinus2;                                }


  void      setGeneralDecodingUnitHrdParamsPresentFlag( bool flag)                     { m_generalDecodingUnitHrdParamsPresentFlag = flag;                 }
  bool      getGeneralDecodingUnitHrdParamsPresentFlag( ) const                        { return m_generalDecodingUnitHrdParamsPresentFlag;                 }

  void      setBitRateScale( uint32_t value )                                          { m_bitRateScale = value;                                    }
  uint32_t  getBitRateScale( ) const                                                   { return m_bitRateScale;                                     }

  void      setCpbSizeScale( uint32_t value )                                          { m_cpbSizeScale = value;                                    }
  uint32_t  getCpbSizeScale( ) const                                                   { return m_cpbSizeScale;                                     }
  void      setCpbSizeDuScale( uint32_t value )                                        { m_cpbSizeDuScale = value;                                  }
  uint32_t  getCpbSizeDuScale( ) const                                                 { return m_cpbSizeDuScale;                                   }


  void      setFixedPicRateFlag( int layer, bool flag )                                { m_HRD[layer].fixedPicRateFlag = flag;                      }
  bool      getFixedPicRateFlag( int layer ) const                                     { return m_HRD[layer].fixedPicRateFlag;                      }

  void      setFixedPicRateWithinCvsFlag( int layer, bool flag )                       { m_HRD[layer].fixedPicRateWithinCvsFlag = flag;             }
  bool      getFixedPicRateWithinCvsFlag( int layer ) const                            { return m_HRD[layer].fixedPicRateWithinCvsFlag;             }

  void      setPicDurationInTcMinus1( int layer, uint32_t value )                      { m_HRD[layer].picDurationInTcMinus1 = value;                }
  uint32_t  getPicDurationInTcMinus1( int layer ) const                                { return m_HRD[layer].picDurationInTcMinus1;                 }

  void      setLowDelayHrdFlag( int layer, bool flag )                                 { m_HRD[layer].lowDelayHrdFlag = flag;                       }
  bool      getLowDelayHrdFlag( int layer ) const                                      { return m_HRD[layer].lowDelayHrdFlag;                       }

  void      setCpbCntMinus1( int layer, uint32_t value )                               { m_HRD[layer].cpbCntMinus1 = value;                         }
  uint32_t  getCpbCntMinus1( int layer ) const                                         { return m_HRD[layer].cpbCntMinus1;                          }

  void      setBitRateValueMinus1( int layer, int cpbcnt, int nalOrVcl, uint32_t value )   { m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl] = value; }
  uint32_t  getBitRateValueMinus1( int layer, int cpbcnt, int nalOrVcl ) const             { return m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl];  }

  void      setCpbSizeValueMinus1( int layer, int cpbcnt, int nalOrVcl, uint32_t value )   { m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl] = value;       }
  uint32_t  getCpbSizeValueMinus1( int layer, int cpbcnt, int nalOrVcl ) const             { return m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl];        }
  void      setDuCpbSizeValueMinus1( int layer, int cpbcnt, int nalOrVcl, uint32_t value ) { m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl] = value;     }
  uint32_t  getDuCpbSizeValueMinus1( int layer, int cpbcnt, int nalOrVcl ) const           { return m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl];      }
  void      setDuBitRateValueMinus1( int layer, int cpbcnt, int nalOrVcl, uint32_t value ) { m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl] = value;     }
  uint32_t  getDuBitRateValueMinus1(int layer, int cpbcnt, int nalOrVcl ) const            { return m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl];      }
  void      setCbrFlag( int layer, int cpbcnt, int nalOrVcl, bool value )                  { m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl] = value;            }
  bool      getCbrFlag( int layer, int cpbcnt, int nalOrVcl ) const                        { return m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl];             }

  bool      getCpbDpbDelaysPresentFlag( ) const                                            { return getNalHrdParametersPresentFlag() || getVclHrdParametersPresentFlag(); }
};

class HRD
{
public:
  HRD()
  :m_bufferingPeriodInitialized (false)
#if JVET_Q0818_PT_SEI
  , m_pictureTimingAvailable    (false)	
#endif
  {};

  virtual ~HRD()
  {};

  void                 setHRDParameters(HRDParameters &hrdParam)    { m_hrdParams=hrdParam; }
  HRDParameters        getHRDParameters() const                     { return m_hrdParams; }
  const HRDParameters& getHRDParameters()                           { return m_hrdParams; }

  void                 setTimingInfo(TimingInfo &timingInfo)        { m_timingInfo=timingInfo; }
  TimingInfo           getTimingInfo() const                        { return m_timingInfo; }
  const TimingInfo&    getTimingInfo()                              { return m_timingInfo; }

  void                       setBufferingPeriodSEI(const SEIBufferingPeriod* bp)  { bp->copyTo(m_bufferingPeriodSEI); m_bufferingPeriodInitialized = true; }
  const SEIBufferingPeriod*  getBufferingPeriodSEI() const                        { return m_bufferingPeriodInitialized ? &m_bufferingPeriodSEI : nullptr; }

#if JVET_Q0818_PT_SEI
  void                       setPictureTimingSEI(const SEIPictureTiming* pt)  { pt->copyTo(m_pictureTimingSEI); m_pictureTimingAvailable = true; }
  const SEIPictureTiming*    getPictureTimingSEI() const                      { return m_pictureTimingAvailable ? &m_pictureTimingSEI : nullptr; }
#endif

protected:
  HRDParameters m_hrdParams;
  TimingInfo    m_timingInfo;
  bool               m_bufferingPeriodInitialized;
  SEIBufferingPeriod m_bufferingPeriodSEI;
#if JVET_Q0818_PT_SEI
  bool               m_pictureTimingAvailable;
  SEIPictureTiming   m_pictureTimingSEI;
#endif
};

#endif //__HRD__
