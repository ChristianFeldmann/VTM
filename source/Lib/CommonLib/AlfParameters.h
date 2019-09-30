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

/** \file     AlfParameters.h
    \brief    Define types for storing ALF parameters
*/

#ifndef __ALFPARAMETERS__
#define __ALFPARAMETERS__

#include <vector>
#include "CommonDef.h"

//! \ingroup AlfParameters
//! \{

enum AlfFilterType
{
  ALF_FILTER_5,
  ALF_FILTER_7,
  ALF_NUM_OF_FILTER_TYPES
};

struct AlfFilterShape
{
  AlfFilterShape( int size )
    : filterLength( size ),
    numCoeff( size * size / 4 + 1 ),
    filterSize( size * size / 2 + 1 )
  {
    if( size == 5 )
    {
      pattern = {
                 0,
             1,  2,  3,
         4,  5,  6,  5,  4,
             3,  2,  1,
                 0
      };

      weights = {
                 2,
              2, 2, 2,
           2, 2, 1, 1
      };
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
      golombIdx = {
                 0,
              0, 1, 0,
           0, 1, 2, 2
      };
#endif

      filterType = ALF_FILTER_5;
    }
    else if( size == 7 )
    {
      pattern = {
                     0,
                 1,  2,  3,
             4,  5,  6,  7,  8,
         9, 10, 11, 12, 11, 10, 9,
             8,  7,  6,  5,  4,
                 3,  2,  1,
                     0
      };

      weights = {
                    2,
                2,  2,  2,
            2,  2,  2,  2,  2,
        2,  2,  2,  1,  1
      };
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
      golombIdx = {
                    0,
                 0, 1, 0,
              0, 1, 2, 1, 0,
           0, 1, 2, 3, 3
      };
#endif

      filterType = ALF_FILTER_7;
    }
    else
    {
      filterType = ALF_NUM_OF_FILTER_TYPES;
      CHECK( 0, "Wrong ALF filter shape" );
    }
  }

  AlfFilterType filterType;
  int filterLength;
  int numCoeff;      //TO DO: check whether we need both numCoeff and filterSize
  int filterSize;
  std::vector<int> pattern;
  std::vector<int> weights;
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
  std::vector<int> golombIdx;
#endif
};

struct AlfParam
{
  bool                         enabledFlag[MAX_NUM_COMPONENT];                          // alf_slice_enable_flag, alf_chroma_idc
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  bool                         nonLinearFlag[MAX_NUM_CHANNEL_TYPE][MAX_NUM_ALF_ALTERNATIVES_CHROMA]; // alf_[luma/chroma]_clip_flag
#else
  bool                         nonLinearFlag[MAX_NUM_CHANNEL_TYPE];                     // alf_nonlinear_enable_flag[Luma/Chroma]
#endif
  short                        lumaCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_coeff_luma_delta[i][j]
  short                        lumaClipp[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_clipp_luma_[i][j]
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  int                          numAlternativesChroma;                                                  // alf_chroma_num_alts_minus_one + 1
  short                        chromaCoeff[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF]; // alf_coeff_chroma[i]
  short                        chromaClipp[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF]; // alf_clipp_chroma[i]
#else
  short                        chromaCoeff[MAX_NUM_ALF_CHROMA_COEFF];                   // alf_coeff_chroma[i]
  short                        chromaClipp[MAX_NUM_ALF_CHROMA_COEFF];                   // alf_clipp_chroma[i]
#endif
  short                        filterCoeffDeltaIdx[MAX_NUM_ALF_CLASSES];                // filter_coeff_delta[i]
  bool                         alfLumaCoeffFlag[MAX_NUM_ALF_CLASSES];                   // alf_luma_coeff_flag[i]
  int                          numLumaFilters;                                          // number_of_filters_minus1 + 1
  bool                         alfLumaCoeffDeltaFlag;                                   // alf_luma_coeff_delta_flag
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
  bool                         alfLumaCoeffDeltaPredictionFlag;                         // alf_luma_coeff_delta_prediction_flag
#endif
  std::vector<AlfFilterShape>* filterShapes;
#if !JVET_O0245_VPS_DPS_APS
  int                          tLayer;
#endif
  bool                         newFilterFlag[MAX_NUM_CHANNEL_TYPE];
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
  int                          fixedFilterPattern;
  int                          fixedFilterIdx[MAX_NUM_ALF_CLASSES];
  int                          fixedFilterSetIndex;
#endif

  AlfParam()
  {
    reset();
  }

  void reset()
  {
    std::memset( enabledFlag, false, sizeof( enabledFlag ) );
    std::memset( nonLinearFlag, false, sizeof( nonLinearFlag ) );
    std::memset( lumaCoeff, 0, sizeof( lumaCoeff ) );
    std::memset( lumaClipp, 0, sizeof( lumaClipp ) );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    numAlternativesChroma = 1;
#endif
    std::memset( chromaCoeff, 0, sizeof( chromaCoeff ) );
    std::memset( chromaClipp, 0, sizeof( chromaClipp ) );
    std::memset( filterCoeffDeltaIdx, 0, sizeof( filterCoeffDeltaIdx ) );
    std::memset( alfLumaCoeffFlag, true, sizeof( alfLumaCoeffFlag ) );
    numLumaFilters = 1;
    alfLumaCoeffDeltaFlag = false;
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    alfLumaCoeffDeltaPredictionFlag = false;
#endif
#if !JVET_O0245_VPS_DPS_APS
    tLayer = 0;
#endif
    memset(newFilterFlag, 0, sizeof(newFilterFlag));
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    fixedFilterPattern = 0;
    std::memset(fixedFilterIdx, 0, sizeof(fixedFilterIdx));
    fixedFilterSetIndex = 0;
#endif
  }

  const AlfParam& operator = ( const AlfParam& src )
  {
    std::memcpy( enabledFlag, src.enabledFlag, sizeof( enabledFlag ) );
    std::memcpy( nonLinearFlag, src.nonLinearFlag, sizeof( nonLinearFlag ) );
    std::memcpy( lumaCoeff, src.lumaCoeff, sizeof( lumaCoeff ) );
    std::memcpy( lumaClipp, src.lumaClipp, sizeof( lumaClipp ) );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    numAlternativesChroma = src.numAlternativesChroma;
#endif
    std::memcpy( chromaCoeff, src.chromaCoeff, sizeof( chromaCoeff ) );
    std::memcpy( chromaClipp, src.chromaClipp, sizeof( chromaClipp ) );
    std::memcpy( filterCoeffDeltaIdx, src.filterCoeffDeltaIdx, sizeof( filterCoeffDeltaIdx ) );
    std::memcpy( alfLumaCoeffFlag, src.alfLumaCoeffFlag, sizeof( alfLumaCoeffFlag ) );
    numLumaFilters = src.numLumaFilters;
    alfLumaCoeffDeltaFlag = src.alfLumaCoeffDeltaFlag;
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    alfLumaCoeffDeltaPredictionFlag = src.alfLumaCoeffDeltaPredictionFlag;
#endif
    filterShapes = src.filterShapes;
#if !JVET_O0245_VPS_DPS_APS
    tLayer = src.tLayer;
#endif
    std::memcpy(newFilterFlag, src.newFilterFlag, sizeof(newFilterFlag));
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    fixedFilterPattern = src.fixedFilterPattern;
    std::memcpy(fixedFilterIdx, src.fixedFilterIdx, sizeof(fixedFilterIdx));
    fixedFilterSetIndex = src.fixedFilterSetIndex;
#endif
    return *this;
  }

#if JVET_O0245_VPS_DPS_APS
  bool operator==( const AlfParam& other )
  {
    if( memcmp( enabledFlag, other.enabledFlag, sizeof( enabledFlag ) ) )
    {
      return false;
    }
    if( memcmp( nonLinearFlag, other.nonLinearFlag, sizeof( nonLinearFlag ) ) )
    {
      return false;
    }
    if( memcmp( lumaCoeff, other.lumaCoeff, sizeof( lumaCoeff ) ) )
    {
      return false;
    }
    if( memcmp( lumaClipp, other.lumaClipp, sizeof( lumaClipp ) ) )
    {
      return false;
    }
    if( memcmp( chromaCoeff, other.chromaCoeff, sizeof( chromaCoeff ) ) )
    {
      return false;
    }
    if( memcmp( chromaClipp, other.chromaClipp, sizeof( chromaClipp ) ) )
    {
      return false;
    }
    if( memcmp( filterCoeffDeltaIdx, other.filterCoeffDeltaIdx, sizeof( filterCoeffDeltaIdx ) ) )
    {
      return false;
    }
    if( memcmp( alfLumaCoeffFlag, other.alfLumaCoeffFlag, sizeof( alfLumaCoeffFlag ) ) )
    {
      return false;
    }
    if( memcmp( newFilterFlag, other.newFilterFlag, sizeof( newFilterFlag ) ) )
    {
      return false;
    }    
    if( numAlternativesChroma != other.numAlternativesChroma )
    {
      return false;
    }
    if( numLumaFilters != other.numLumaFilters )
    {
      return false;
    }
    if( alfLumaCoeffDeltaFlag != other.alfLumaCoeffDeltaFlag )
    {
      return false;
    }

    return true;
  }

  bool operator!=( const AlfParam& other )
  {
    return !( *this == other );
  }
#endif
};

//! \}

#endif  // end of #ifndef  __ALFPARAMETERS__