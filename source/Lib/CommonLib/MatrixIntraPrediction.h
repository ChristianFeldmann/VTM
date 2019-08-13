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

#if JVET_O0925_MIP_SIMPLIFICATIONS
  void prepareInputForPred(const CPelBuf &pSrc, const Area& block, const int bitDepth);
#else
  void prepareInputForPred(const CPelBuf &src, const Area& block, const int bitDepth, const AvailableInfo &availInfo);
#endif
  void predBlock(int* const result, const int modeIdx, const int bitDepth);

  private:
    static_vector<int, MIP_MAX_INPUT_SIZE> m_reducedBoundary;           // downsampled             boundary of a block
    static_vector<int, MIP_MAX_INPUT_SIZE> m_reducedBoundaryTransposed; // downsampled, transposed boundary of a block
#if JVET_O0925_MIP_SIMPLIFICATIONS
    int                                    m_inputOffset;
    int                                    m_inputOffsetTransp;
    static_vector<int, MIP_MAX_WIDTH>      m_refSamplesTop;             // top  reference samples for upsampling
    static_vector<int, MIP_MAX_HEIGHT>     m_refSamplesLeft;            // left reference samples for upsampling
#else
    static_vector<int, MIP_MAX_WIDTH>      m_boundaryForUpsamplingTop;  // top  boundary samples for upsampling
    static_vector<int, MIP_MAX_HEIGHT>     m_boundaryForUpsamplingLeft; // left boundary samples for upsampling
#endif

    Size m_blockSize;
    int  m_numModes;
    Size m_reducedBoundarySize;
    Size m_reducedPredictionSize;
#if !JVET_O0925_MIP_SIMPLIFICATIONS
    Size m_boundarySizeForUpsampling;
#endif
    unsigned int m_upsmpFactorHor;
    unsigned int m_upsmpFactorVer;

    void initPredBlockParams(const Size& block);

#if JVET_O0925_MIP_SIMPLIFICATIONS
    static void boundaryDownsampling1D(int* reducedDst, const int* const fullSrc, const SizeType srcLen, const SizeType dstLen);
#else
    static void boundaryDownsampling1D( int* reducedDst, int* fullSrcAndIntermediateDst, const SizeType srcLen, const SizeType dstLen, const bool saveIntermediate, const SizeType intermediateLen );
#endif
    static void doDownsampling( int* dst, const int* src, const SizeType srcLen, const SizeType dstLen );

    void predictionUpsampling( int* const dst, const int* const src, const bool transpose ) const;
    static void predictionUpsampling1D( int* const dst, const int* const src, const int* const bndry,
                                        const SizeType srcSizeUpsmpDim, const SizeType srcSizeOrthDim,
                                        const SizeType srcStep, const SizeType srcStride,
                                        const SizeType dstStep, const SizeType dstStride,
#if JVET_O0925_MIP_SIMPLIFICATIONS
                                        const SizeType bndryStep,
#endif
                                        const unsigned int upsmpFactor );

#if JVET_O0925_MIP_SIMPLIFICATIONS
    void getMatrixData(const uint8_t*& matrix, int &shiftMatrix, int &offsetMatrix, const int modeIdx) const;
#else
    void getMatrixBias( const short*& matrix, const short*& bias, const int modeIdx ) const;
    void getShifts( int &shiftMatrix, int &shiftBias, const int modeIdx, const int bitDepth ) const;
#endif

    bool isTransposed( const int modeIdx ) const;
    int  getWeightIdx( const int modeIdx ) const;

#if JVET_O0925_MIP_SIMPLIFICATIONS
    void computeReducedPred( int*const result, const int* const input, const uint8_t*matrix,
                             const bool leaveHorOut, const bool leaveVerOut,
                             const int shiftMatrix, const int offsetMatrix,
                             const bool transpose, const bool needUpsampling, const int bitDepth );
#else
    void xComputeMatrixTimesRedBndryPlusBias( int*const result, const int* const input,
                                              const short*matrix, const short*bias,
                                              const bool leaveHorOut, const bool leaveVerOut,
                                              const int shiftMatrix, const int shiftBias,
                                              const bool transpose, const bool needUpsampling );
#endif
  };

#endif //__MATRIXINTRAPPREDICTION__
