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


#ifndef __HRD__
#define __HRD__

#include "Common.h"

class TimingInfo
{
protected:
  bool     m_timingInfoPresentFlag;
  uint32_t m_numUnitsInTick;
  uint32_t m_timeScale;
  bool     m_pocProportionalToTimingFlag;
  int      m_numTicksPocDiffOneMinus1;

public:
  TimingInfo()
    : m_timingInfoPresentFlag      (false)
    , m_numUnitsInTick             (1001)
    , m_timeScale                  (60000)
    , m_pocProportionalToTimingFlag(false)
    , m_numTicksPocDiffOneMinus1   (0)
  {}

  void     setTimingInfoPresentFlag( bool flag )   { m_timingInfoPresentFlag = flag;       }
  bool     getTimingInfoPresentFlag( ) const       { return m_timingInfoPresentFlag;       }

  void     setNumUnitsInTick( uint32_t value )     { m_numUnitsInTick = value;             }
  uint32_t getNumUnitsInTick( ) const              { return m_numUnitsInTick;              }
  void     setTimeScale( uint32_t value )          { m_timeScale = value;                  }
  uint32_t getTimeScale( ) const                   { return m_timeScale;                   }

  void     setPocProportionalToTimingFlag(bool x)  { m_pocProportionalToTimingFlag = x;    }
  bool     getPocProportionalToTimingFlag( ) const { return m_pocProportionalToTimingFlag; }

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
  bool     m_subPicCpbParamsPresentFlag;
  uint32_t m_tickDivisorMinus2;
  uint32_t m_duCpbRemovalDelayLengthMinus1;
  bool     m_subPicCpbParamsInPicTimingSEIFlag;
  uint32_t m_dpbOutputDelayDuLengthMinus1;
  uint32_t m_bitRateScale;
  uint32_t m_cpbSizeScale;
  uint32_t m_ducpbSizeScale;
  uint32_t m_initialCpbRemovalDelayLengthMinus1;
  uint32_t m_cpbRemovalDelayLengthMinus1;
  uint32_t m_dpbOutputDelayLengthMinus1;
  HrdSubLayerInfo m_HRD[MAX_TLAYER];

public:
  HRDParameters()
    :m_nalHrdParametersPresentFlag       (false)
    ,m_vclHrdParametersPresentFlag       (false)
    ,m_subPicCpbParamsPresentFlag        (false)
    ,m_tickDivisorMinus2                 (0)
    ,m_duCpbRemovalDelayLengthMinus1     (0)
    ,m_subPicCpbParamsInPicTimingSEIFlag (false)
    ,m_dpbOutputDelayDuLengthMinus1      (0)
    ,m_bitRateScale                      (0)
    ,m_cpbSizeScale                      (0)
    ,m_initialCpbRemovalDelayLengthMinus1(23)
    ,m_cpbRemovalDelayLengthMinus1       (23)
    ,m_dpbOutputDelayLengthMinus1        (23)
  {}

  virtual ~HRDParameters() {}

  void      setNalHrdParametersPresentFlag( bool flag )                                { m_nalHrdParametersPresentFlag = flag;                      }
  bool      getNalHrdParametersPresentFlag( ) const                                    { return m_nalHrdParametersPresentFlag;                      }

  void      setVclHrdParametersPresentFlag( bool flag )                                { m_vclHrdParametersPresentFlag = flag;                      }
  bool      getVclHrdParametersPresentFlag( ) const                                    { return m_vclHrdParametersPresentFlag;                      }

  void      setSubPicCpbParamsPresentFlag( bool flag )                                 { m_subPicCpbParamsPresentFlag = flag;                       }
  bool      getSubPicCpbParamsPresentFlag( ) const                                     { return m_subPicCpbParamsPresentFlag;                       }

  void      setTickDivisorMinus2( uint32_t value )                                     { m_tickDivisorMinus2 = value;                               }
  uint32_t  getTickDivisorMinus2( ) const                                              { return m_tickDivisorMinus2;                                }

  void      setDuCpbRemovalDelayLengthMinus1( uint32_t value )                         { m_duCpbRemovalDelayLengthMinus1 = value;                   }
  uint32_t  getDuCpbRemovalDelayLengthMinus1( ) const                                  { return m_duCpbRemovalDelayLengthMinus1;                    }

  void      setSubPicCpbParamsInPicTimingSEIFlag( bool flag)                           { m_subPicCpbParamsInPicTimingSEIFlag = flag;                }
  bool      getSubPicCpbParamsInPicTimingSEIFlag( ) const                              { return m_subPicCpbParamsInPicTimingSEIFlag;                }

  void      setDpbOutputDelayDuLengthMinus1(uint32_t value )                           { m_dpbOutputDelayDuLengthMinus1 = value;                    }
  uint32_t  getDpbOutputDelayDuLengthMinus1( ) const                                   { return m_dpbOutputDelayDuLengthMinus1;                     }

  void      setBitRateScale( uint32_t value )                                          { m_bitRateScale = value;                                    }
  uint32_t  getBitRateScale( ) const                                                   { return m_bitRateScale;                                     }

  void      setCpbSizeScale( uint32_t value )                                          { m_cpbSizeScale = value;                                    }
  uint32_t  getCpbSizeScale( ) const                                                   { return m_cpbSizeScale;                                     }
  void      setDuCpbSizeScale( uint32_t value )                                        { m_ducpbSizeScale = value;                                  }
  uint32_t  getDuCpbSizeScale( ) const                                                 { return m_ducpbSizeScale;                                   }

  void      setInitialCpbRemovalDelayLengthMinus1( uint32_t value )                    { m_initialCpbRemovalDelayLengthMinus1 = value;              }
  uint32_t  getInitialCpbRemovalDelayLengthMinus1( ) const                             { return m_initialCpbRemovalDelayLengthMinus1;               }

  void      setCpbRemovalDelayLengthMinus1( uint32_t value )                           { m_cpbRemovalDelayLengthMinus1 = value;                     }
  uint32_t  getCpbRemovalDelayLengthMinus1( ) const                                    { return m_cpbRemovalDelayLengthMinus1;                      }

  void      setDpbOutputDelayLengthMinus1( uint32_t value )                            { m_dpbOutputDelayLengthMinus1 = value;                      }
  uint32_t  getDpbOutputDelayLengthMinus1( ) const                                     { return m_dpbOutputDelayLengthMinus1;                       }

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
  {};

  virtual ~HRD()
  {};

  void                 setHRDParameters(HRDParameters &hrdParam)    { m_hrdParams=hrdParam; }
  HRDParameters        getHRDParameters() const                     { return m_hrdParams; }
  const HRDParameters& getHRDParameters()                           { return m_hrdParams; }

  void                 setTimingInfo(TimingInfo &timingInfo)        { m_timingInfo=timingInfo; }
  TimingInfo           getTimingInfo() const                        { return m_timingInfo; }
  const TimingInfo&    getTimingInfo()                              { return m_timingInfo; }

protected:
  HRDParameters m_hrdParams;
  TimingInfo    m_timingInfo;
};

#endif //__HRD__