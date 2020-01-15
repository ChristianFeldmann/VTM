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

/** \file     MatrixIntraPrediction.h
\brief    matrix-based intra prediction class (header)
*/

#ifndef __MATRIXINTRAPPREDICTION__
#define __MATRIXINTRAPPREDICTION__


#include "Unit.h"

static const int MIP_MAX_INPUT_SIZE             =  8;
static const int MIP_MAX_REDUCED_OUTPUT_SAMPLES = 64;


class MatrixIntraPrediction
{
public:
  MatrixIntraPrediction();

  void prepareInputForPred(const CPelBuf &pSrc, const Area& block, const int bitDepth);
#if JVET_P0803_COMBINED_MIP_CLEANUP
  void predBlock(int* const result, const int modeIdx, const bool transpose, const int bitDepth);
#else
  void predBlock(int* const result, const int modeIdx, const int bitDepth);
#endif

  private:
    static_vector<int, MIP_MAX_INPUT_SIZE> m_reducedBoundary;           // downsampled             boundary of a block
    static_vector<int, MIP_MAX_INPUT_SIZE> m_reducedBoundaryTransposed; // downsampled, transposed boundary of a block
    int                                    m_inputOffset;
    int                                    m_inputOffsetTransp;
    static_vector<int, MIP_MAX_WIDTH>      m_refSamplesTop;             // top  reference samples for upsampling
    static_vector<int, MIP_MAX_HEIGHT>     m_refSamplesLeft;            // left reference samples for upsampling

    Size m_blockSize;
#if JVET_P0803_COMBINED_MIP_CLEANUP || JVET_P0199_P0289_P0303_MIP_FULLMATRIX
    int  m_sizeId;
    int  m_reducedBdrySize;
#else
    int  m_numModes;
    Size m_reducedBoundarySize;
#endif
#if JVET_P0199_P0289_P0303_MIP_FULLMATRIX
    int  m_reducedPredSize;
#else
    Size m_reducedPredictionSize;
#endif
    unsigned int m_upsmpFactorHor;
    unsigned int m_upsmpFactorVer;

    void initPredBlockParams(const Size& block);

    static void boundaryDownsampling1D(int* reducedDst, const int* const fullSrc, const SizeType srcLen, const SizeType dstLen);
#if !JVET_P0803_COMBINED_MIP_CLEANUP
    static void doDownsampling( int* dst, const int* src, const SizeType srcLen, const SizeType dstLen );
#endif

#if JVET_P0803_COMBINED_MIP_CLEANUP && JVET_P0199_P0289_P0303_MIP_FULLMATRIX
    void predictionUpsampling( int* const dst, const int* const src ) const;
#else
    void predictionUpsampling( int* const dst, const int* const src, const bool transpose ) const;
#endif
    static void predictionUpsampling1D( int* const dst, const int* const src, const int* const bndry,
                                        const SizeType srcSizeUpsmpDim, const SizeType srcSizeOrthDim,
                                        const SizeType srcStep, const SizeType srcStride,
                                        const SizeType dstStep, const SizeType dstStride,
                                        const SizeType bndryStep,
                                        const unsigned int upsmpFactor );

    void getMatrixData(const uint8_t*& matrix, int &shiftMatrix, int &offsetMatrix, const int modeIdx) const;

#if !JVET_P0803_COMBINED_MIP_CLEANUP
    bool isTransposed( const int modeIdx ) const;
    int  getWeightIdx( const int modeIdx ) const;
#endif

#if JVET_P0199_P0289_P0303_MIP_FULLMATRIX
    void computeReducedPred( int*const result, const int* const input, 
                             const uint8_t*matrix, const int shiftMatrix, const int offsetMatrix,
                             const bool transpose, const int bitDepth );
#else
    void computeReducedPred( int*const result, const int* const input, const uint8_t*matrix,
                             const bool leaveHorOut, const bool leaveVerOut,
                             const int shiftMatrix, const int offsetMatrix,
#if JVET_P0803_COMBINED_MIP_CLEANUP
                             const bool transpose, const int bitDepth );
#else
                             const bool transpose, const bool needUpsampling, const int bitDepth );
#endif
#endif
  };

#endif //__MATRIXINTRAPPREDICTION__
