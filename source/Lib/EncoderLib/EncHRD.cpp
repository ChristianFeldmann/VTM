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

#include "EncHRD.h"

#if U0132_TARGET_BITS_SATURATION

// calculate scale value of bitrate and initial delay
int EncHRD::xCalcScale(int x)
{
  if (x==0)
  {
    return 0;
  }
  uint32_t mask = 0xffffffff;
  int scaleValue = 32;

  while ((x&mask) != 0)
  {
    scaleValue--;
    mask = (mask >> 1);
  }

  return scaleValue;
}
#endif

void EncHRD::initHRDParameters (EncCfg* encCfg)
{
  bool useSubCpbParams = (encCfg->getSliceMode() > 0) || (encCfg->getSliceSegmentMode() > 0);
  int  bitRate         = encCfg->getTargetBitrate();
  bool isRandomAccess  = encCfg->getIntraPeriod() > 0;
# if U0132_TARGET_BITS_SATURATION
  int cpbSize          = encCfg->getCpbSize();
  CHECK(!(cpbSize!=0), "Unspecified error");  // CPB size may not be equal to zero. ToDo: have a better default and check for level constraints
  if( !encCfg->getVuiParametersPresentFlag() && !encCfg->getCpbSaturationEnabled() )
#else
  if( !encCfg.getVuiParametersPresentFlag() )
#endif
  {
    return;
  }

  m_timingInfo.setTimingInfoPresentFlag( true );
  switch( encCfg->getFrameRate() )
  {
  case 24:
    m_timingInfo.setNumUnitsInTick( 1125000 );    m_timingInfo.setTimeScale    ( 27000000 );
    break;
  case 25:
    m_timingInfo.setNumUnitsInTick( 1080000 );    m_timingInfo.setTimeScale    ( 27000000 );
    break;
  case 30:
    m_timingInfo.setNumUnitsInTick( 900900 );     m_timingInfo.setTimeScale    ( 27000000 );
    break;
  case 50:
    m_timingInfo.setNumUnitsInTick( 540000 );     m_timingInfo.setTimeScale    ( 27000000 );
    break;
  case 60:
    m_timingInfo.setNumUnitsInTick( 450450 );     m_timingInfo.setTimeScale    ( 27000000 );
    break;
  default:
    m_timingInfo.setNumUnitsInTick( 1001 );       m_timingInfo.setTimeScale    ( 60000 );
    break;
  }

  if (encCfg->getTemporalSubsampleRatio()>1)
  {
    uint32_t temporalSubsampleRatio = encCfg->getTemporalSubsampleRatio();
    if ( double(m_timingInfo.getNumUnitsInTick()) * temporalSubsampleRatio > std::numeric_limits<uint32_t>::max() )
    {
      m_timingInfo.setTimeScale( m_timingInfo.getTimeScale() / temporalSubsampleRatio );
    }
    else
    {
      m_timingInfo.setNumUnitsInTick( m_timingInfo.getNumUnitsInTick() * temporalSubsampleRatio );
    }
  }

  bool rateCnt = ( bitRate > 0 );
  m_hrdParams.setNalHrdParametersPresentFlag( rateCnt );
  m_hrdParams.setVclHrdParametersPresentFlag( rateCnt );
  m_hrdParams.setSubPicCpbParamsPresentFlag( useSubCpbParams );

  if( m_hrdParams.getSubPicCpbParamsPresentFlag() )
  {
    m_hrdParams.setTickDivisorMinus2( 100 - 2 );                          //
    m_hrdParams.setDuCpbRemovalDelayLengthMinus1( 7 );                    // 8-bit precision ( plus 1 for last DU in AU )
    m_hrdParams.setSubPicCpbParamsInPicTimingSEIFlag( true );
    m_hrdParams.setDpbOutputDelayDuLengthMinus1( 5 + 7 );                 // With sub-clock tick factor of 100, at least 7 bits to have the same value as AU dpb delay
  }
  else
  {
    m_hrdParams.setSubPicCpbParamsInPicTimingSEIFlag( false );
  }

#if U0132_TARGET_BITS_SATURATION
  if (xCalcScale(bitRate) <= 6)
  {
    m_hrdParams.setBitRateScale(0);
  }
  else
  {
    m_hrdParams.setBitRateScale(xCalcScale(bitRate) - 6);
  }

  if (xCalcScale(cpbSize) <= 4)
  {
    m_hrdParams.setCpbSizeScale(0);
  }
  else
  {
    m_hrdParams.setCpbSizeScale(xCalcScale(cpbSize) - 4);
  }
#else
  m_hrdParams.setBitRateScale( 4 );                                       // in units of 2^( 6 + 4 ) = 1,024 bps
  m_hrdParams.setCpbSizeScale( 6 );                                       // in units of 2^( 4 + 6 ) = 1,024 bit
#endif

  m_hrdParams.setDuCpbSizeScale( 6 );                                     // in units of 2^( 4 + 6 ) = 1,024 bit

  m_hrdParams.setInitialCpbRemovalDelayLengthMinus1(15);                  // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  if( isRandomAccess )
  {
    m_hrdParams.setCpbRemovalDelayLengthMinus1(5);                        // 32 = 2^5 (plus 1)
    m_hrdParams.setDpbOutputDelayLengthMinus1 (5);                        // 32 + 3 = 2^6
  }
  else
  {
    m_hrdParams.setCpbRemovalDelayLengthMinus1(9);                        // max. 2^10
    m_hrdParams.setDpbOutputDelayLengthMinus1 (9);                        // max. 2^10
  }

  // Note: parameters for all temporal layers are initialized with the same values
  int i, j;
  uint32_t bitrateValue, cpbSizeValue;
  uint32_t duCpbSizeValue;
  uint32_t duBitRateValue = 0;

  for( i = 0; i < MAX_TLAYER; i ++ )
  {
    m_hrdParams.setFixedPicRateFlag( i, 1 );
    m_hrdParams.setPicDurationInTcMinus1( i, 0 );
    m_hrdParams.setLowDelayHrdFlag( i, 0 );
    m_hrdParams.setCpbCntMinus1( i, 0 );

    //! \todo check for possible PTL violations
    // BitRate[ i ] = ( bit_rate_value_minus1[ i ] + 1 ) * 2^( 6 + bit_rate_scale )
    bitrateValue = bitRate / (1 << (6 + m_hrdParams.getBitRateScale()) );      // bitRate is in bits, so it needs to be scaled down
                                                                        // CpbSize[ i ] = ( cpb_size_value_minus1[ i ] + 1 ) * 2^( 4 + cpb_size_scale )
#if U0132_TARGET_BITS_SATURATION
    cpbSizeValue = cpbSize / (1 << (4 + m_hrdParams.getCpbSizeScale()) );      // using bitRate results in 1 second CPB size
#else
    cpbSizeValue = bitRate / (1 << (4 + m_hrdParams.getCpbSizeScale()) );      // using bitRate results in 1 second CPB size
#endif


                                                                        // DU CPB size could be smaller (i.e. bitrateValue / number of DUs), but we don't know
                                                                        // in how many DUs the slice segment settings will result
    duCpbSizeValue = bitrateValue;
    duBitRateValue = cpbSizeValue;

    for( j = 0; j < ( m_hrdParams.getCpbCntMinus1( i ) + 1 ); j ++ )
    {
      m_hrdParams.setBitRateValueMinus1( i, j, 0, ( bitrateValue - 1 ) );
      m_hrdParams.setCpbSizeValueMinus1( i, j, 0, ( cpbSizeValue - 1 ) );
      m_hrdParams.setDuCpbSizeValueMinus1( i, j, 0, ( duCpbSizeValue - 1 ) );
      m_hrdParams.setDuBitRateValueMinus1( i, j, 0, ( duBitRateValue - 1 ) );
      m_hrdParams.setCbrFlag( i, j, 0, false );

      m_hrdParams.setBitRateValueMinus1( i, j, 1, ( bitrateValue - 1) );
      m_hrdParams.setCpbSizeValueMinus1( i, j, 1, ( cpbSizeValue - 1 ) );
      m_hrdParams.setDuCpbSizeValueMinus1( i, j, 1, ( duCpbSizeValue - 1 ) );
      m_hrdParams.setDuBitRateValueMinus1( i, j, 1, ( duBitRateValue - 1 ) );
      m_hrdParams.setCbrFlag( i, j, 1, false );
    }
  }
}

