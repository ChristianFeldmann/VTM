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

/** \file     MatrixIntraPrediction.cpp
\brief    matrix-based intra prediction class
*/

#include "MatrixIntraPrediction.h"
#include "dtrace_next.h"

#if JVET_N0217_MATRIX_INTRAPRED
#include "UnitTools.h"
#include "MipData.h"


namespace Mip
{
  PredictorMIP::PredictorMIP():
    m_reducedBoundary          (MIP_MAX_INPUT_SIZE),
    m_reducedBoundaryTransposed(MIP_MAX_INPUT_SIZE),
    m_boundaryForUpsamplingTop (MIP_MAX_WIDTH),
    m_boundaryForUpsamplingLeft(MIP_MAX_HEIGHT),
    m_blockSize( 0, 0 ),
    m_numModes( 0 ),
    m_reducedBoundarySize( 0, 0 ),
    m_reducedPredictionSize( 0, 0 ),
    m_boundarySizeForUpsampling( 0, 0 ),
    m_upsmpFactorHor( 0 ),
    m_upsmpFactorVer( 0 )
  {
  }


  void PredictorMIP::deriveBoundaryData(const CPelBuf& src, const Area& block, const int bitDepth, const AvailableInfo &availInfo)
  {
    // Step 1: Save block size and calculate dependent values
    initPredBlockParams(block);

    // Step 2: Get the input data (left and top reference samples)
    const int defaultPad = int(1 << (bitDepth - 1));

    // TOP (save top boundary since we might need it for upsampling)
    m_boundaryForUpsamplingTop.resize( block.width );
    const int availPosTop = availInfo.maxPosTop;
    CHECKD(availPosTop > block.width, "Error: availPosTop out of range");

    if (availPosTop > 0)
    {
      // top available
      const Position posT0(block.x, block.y - 1);
      for (int x = 0; x < availPosTop; x++)
      {
        m_boundaryForUpsamplingTop[ x ] = src.at( posT0.offset( x, 0 ) );
      }
      // top unavailable
      const int padVal = m_boundaryForUpsamplingTop[ availPosTop - 1 ];
      for( int x = availPosTop; x < m_boundaryForUpsamplingTop.size(); x++ )
      {
        m_boundaryForUpsamplingTop[ x ] = padVal;
      }
    }
    else
    {
      std::fill( m_boundaryForUpsamplingTop.begin(), m_boundaryForUpsamplingTop.end(), defaultPad );
    }

    // LEFT (save left boundary since we might need it for upsampling)
    m_boundaryForUpsamplingLeft.resize( block.height );
    const int availPosLeft = availInfo.maxPosLeft;
    CHECKD(availPosLeft > block.height, "Error: availPosLeft out of range");

    if (availPosLeft > 0)
    {
      // left available
      const Position posL0(block.x - 1, block.y);
      for (int y = 0; y < availPosLeft; y++)
      {
        m_boundaryForUpsamplingLeft[ y ] = src.at( posL0.offset( 0, y ) );
      }
      // left unavailable
      const int padVal = m_boundaryForUpsamplingLeft[ availPosLeft - 1 ];
      for( int y = availPosLeft; y < m_boundaryForUpsamplingLeft.size(); y++ )
      {
        m_boundaryForUpsamplingLeft[ y ] = padVal;
      }
    }
    else
    {
      std::fill( m_boundaryForUpsamplingLeft.begin(), m_boundaryForUpsamplingLeft.end(), defaultPad );
    }

    // Step 3: Compute the reduced boundary via Haar-downsampling (input for the prediction and intermediate boundary for upsampling)
    m_reducedBoundary          .resize( m_reducedBoundarySize.width + m_reducedBoundarySize.height );
    m_reducedBoundaryTransposed.resize( m_reducedBoundarySize.width + m_reducedBoundarySize.height );

    const bool needVerticalUpsampling = ( m_upsmpFactorVer > 1 );
    int* const topReduced = m_reducedBoundary.data();
    boundaryDownsampling1D( topReduced, m_boundaryForUpsamplingTop.data(), block.width, m_reducedBoundarySize.width, needVerticalUpsampling, m_boundarySizeForUpsampling.width );
    m_boundaryForUpsamplingTop.resize( needVerticalUpsampling ? m_boundarySizeForUpsampling.width : 0 );

    const bool needHorizontalUpsampling = ( m_upsmpFactorHor > 1 );
    int* const leftReduced = m_reducedBoundary.data() + m_reducedBoundarySize.width;
    boundaryDownsampling1D( leftReduced, m_boundaryForUpsamplingLeft.data(), block.height, m_reducedBoundarySize.height, needHorizontalUpsampling, m_boundarySizeForUpsampling.height );
    m_boundaryForUpsamplingLeft.resize( needHorizontalUpsampling ? m_boundarySizeForUpsampling.height : 0 );

    int* const leftReducedTransposed = m_reducedBoundaryTransposed.data();
    int* const topReducedTransposed  = m_reducedBoundaryTransposed.data() + m_reducedBoundarySize.height;
    for( int x = 0; x < m_reducedBoundarySize.width; x++ )
    {
      topReducedTransposed[ x ] = topReduced[ x ];
    }
    for( int y = 0; y < m_reducedBoundarySize.height; y++ )
    {
      leftReducedTransposed[ y ] = leftReduced[ y ];
    }
  }

  void PredictorMIP::getPrediction(int* const result, const int modeIdx, const int bitDepth)
  {
    const bool transpose = isTransposed( modeIdx );
    const bool needUpsampling = ( m_upsmpFactorHor > 1 ) || ( m_upsmpFactorVer > 1 );

    const short* matrix;
    const short* bias;
    getMatrixBias( matrix, bias, modeIdx );

    int shiftMatrix = 0;
    int shiftBias = 0;
    getShifts(shiftMatrix, shiftBias, modeIdx, bitDepth );

    bool leaveHorOut = ( m_blockSize.width == 4 && m_blockSize.height >= 16 );
    bool leaveVerOut = ( m_blockSize.height == 4 && m_blockSize.width >= 16 );
    if (transpose)
    {
      std::swap(leaveHorOut, leaveVerOut);
    }
    static_vector<int, MIP_MAX_REDUCED_OUTPUT_SAMPLES> bufReducedPred( m_reducedPredictionSize.area() );
    int* const       reducedPred     = needUpsampling ? bufReducedPred.data() : result;
    const int* const reducedBoundary = transpose ? m_reducedBoundaryTransposed.data() : m_reducedBoundary.data();
    xComputeMatrixTimesRedBndryPlusBias( reducedPred, reducedBoundary, matrix, bias,
                                         leaveHorOut, leaveVerOut,
                                         shiftMatrix, shiftBias,
                                         transpose, needUpsampling );
    // Reduced prediction is transposed if ( transpose && needUpsampling ).

    if( needUpsampling )
    {
      predictionUpsampling( result, reducedPred, transpose );
    }
  }

  bool PredictorMIP::isTransposed( const int modeIdx ) const
  {
    return ( modeIdx > ( m_numModes / 2 ) );
  }

  int PredictorMIP::getWeightIdx( const int modeIdx ) const
  {
    if( modeIdx > m_numModes / 2 )
    {
      return modeIdx - m_numModes / 2;
    }
    else
    {
      return modeIdx;
    }
  }

  void PredictorMIP::initPredBlockParams(const Size& block)
  {
    m_blockSize = block;
    m_numModes  = getNumModesMip(m_blockSize);

    // init reduced boundary size
    if (m_blockSize.width > 4 || m_blockSize.height > 4)
    {
      m_reducedBoundarySize = Size(4, 4);
    }
    else
    {
      m_reducedBoundarySize = Size(2, 2);
    }

    // init reduced prediction size
    if (m_blockSize.width <= 8 && m_blockSize.height <= 8)
    {
      m_reducedPredictionSize = Size(4, 4);
    }
    else
    {
      m_reducedPredictionSize = Size(std::min<SizeType>(m_blockSize.width, 8), std::min<SizeType>(m_blockSize.height, 8));
    }

    // init boundary size for upsampling
    if (m_blockSize.height > m_blockSize.width)
    {
      m_boundarySizeForUpsampling = Size(m_blockSize.width, m_reducedPredictionSize.height);
    }
    else
    {
      m_boundarySizeForUpsampling = Size(m_reducedPredictionSize.width, m_blockSize.height);
    }

    // init upsampling factors
    m_upsmpFactorHor = m_blockSize.width / m_reducedPredictionSize.width;
    CHECKD(m_reducedPredictionSize.width * m_upsmpFactorHor != m_blockSize.width, "Need integer horizontal upsampling factor.");
    CHECKD((m_upsmpFactorHor < 1) || ((m_upsmpFactorHor & (m_upsmpFactorHor - 1)) != 0), "Need power of two horizontal upsampling factor.");

    m_upsmpFactorVer = m_blockSize.height / m_reducedPredictionSize.height;
    CHECKD(m_reducedPredictionSize.height * m_upsmpFactorVer != m_blockSize.height, "Need integer vertical upsampling factor.");
    CHECKD((m_upsmpFactorVer < 1) || ((m_upsmpFactorVer & (m_upsmpFactorVer - 1)) != 0), "Need power of two vertical upsampling factor.");
  }

  void PredictorMIP::doDownsampling( int* dst, const int* src, const SizeType srcLen, const SizeType dstLen )
  {
    // TODO: Check if src and dst can ever be negative. If not assign unsigned type and simplify rounding.
    const SizeType downsmpFactor = srcLen / dstLen;
    CHECKD( srcLen != dstLen * downsmpFactor, "Need integer downsampling factor." );
    CHECKD( ( downsmpFactor & ( downsmpFactor - 1 ) ) != 0, "Need power of two downsampling factor." );
    const int log2DownsmpFactor = g_aucLog2[ downsmpFactor ];
    const int roundingOffsetPositive = ( 1 << ( log2DownsmpFactor - 1 ) );

    for( SizeType srcIdx = 0, dstIdx = 0; dstIdx < dstLen; ++dstIdx )
    {
      int sum = 0;
      for( SizeType blockIdx = 0; blockIdx < downsmpFactor; ++blockIdx, ++srcIdx )
      {
        sum += src[ srcIdx ];
      }
      const int roundingOffset = roundingOffsetPositive - ( sum < 0 ? 1 : 0 );
      dst[ dstIdx ] = ( sum + roundingOffset ) >> log2DownsmpFactor;
    }
  }


  void PredictorMIP::boundaryDownsampling1D( int* reducedDst, int* fullSrcAndIntermediateDst,
                                              const SizeType srcLen, const SizeType dstLen,
                                              const bool saveIntermediate, const SizeType intermediateLen )
  {
    SizeType currLen = srcLen;

    // Create intermediate boundary if needed.
    if( saveIntermediate && intermediateLen < srcLen )
    {
      CHECKD( intermediateLen < dstLen, "Intermediate length must not be less than target length." );
      doDownsampling( fullSrcAndIntermediateDst, fullSrcAndIntermediateDst, currLen, intermediateLen );
      currLen = intermediateLen;
    }

    if( dstLen < currLen )
    {
      // Create reduced boundary by downsampling.
      doDownsampling( reducedDst, fullSrcAndIntermediateDst, currLen, dstLen );
    }
    else
    {
      // Copy reduced boundary if no downsampling is needed.
      for( SizeType i = 0; i < dstLen; ++i )
      {
        reducedDst[ i ] = fullSrcAndIntermediateDst[ i ];
      }
    }
  }


  void PredictorMIP::predictionUpsampling1D( int* const dst, const int* const src, const int* const bndry,
                                              const SizeType srcSizeUpsmpDim, const SizeType srcSizeOrthDim,
                                              const SizeType srcStep, const SizeType srcStride,
                                              const SizeType dstStep, const SizeType dstStride,
                                              const unsigned int upsmpFactor )
  {
    // TODO: Check if src and dst can ever be negative. If not assign unsigned type and simplify rounding.
    const int log2UpsmpFactor = g_aucLog2[ upsmpFactor ];
    CHECKD( upsmpFactor <= 1, "Upsampling factor must be at least 2." );

    SizeType idxOrthDim = 0;
    const int* srcLine = src;
    int* dstLine = dst;
    while( idxOrthDim < srcSizeOrthDim )
    {
      SizeType idxUpsmpDim = 0;
      const int* before = bndry + idxOrthDim;
      const int* behind = srcLine;
      int* currDst = dstLine;
      while( idxUpsmpDim < srcSizeUpsmpDim )
      {
        SizeType pos = 1;
        int scaledBefore = ( *before ) << log2UpsmpFactor;
        int scaledBehind = 0;
        while( pos <= upsmpFactor )
        {
          scaledBefore -= *before;
          scaledBehind += *behind;
          *currDst = scaledBefore + scaledBehind;
          *currDst = ( *currDst + ( 1 << ( log2UpsmpFactor - 1 ) ) -
            ( *currDst < 0 ? 1 : 0 ) ) >> log2UpsmpFactor;

          pos++;
          currDst += dstStep;
        }

        idxUpsmpDim++;
        before = behind;
        behind += srcStep;
      }

      idxOrthDim++;
      srcLine += srcStride;
      dstLine += dstStride;
    }
  }


  void PredictorMIP::predictionUpsampling( int* const dst, const int* const src, const bool transpose ) const
  {
    // shorter side is upsampled first
    if( m_blockSize.height > m_blockSize.width )
    {
      const int* verSrc       = nullptr;
      SizeType   verSrcStep   = 0;
      SizeType   verSrcStride = 0;
      if( m_upsmpFactorHor > 1 )
      {
        const SizeType horSrcStep   = transpose ? m_reducedPredictionSize.height : 1;
        const SizeType horSrcStride = transpose ? 1 : m_reducedPredictionSize.width;

        int* const     horDst       = dst + ( m_upsmpFactorVer - 1 ) * m_blockSize.width;
        const SizeType horDstStride = m_upsmpFactorVer * m_blockSize.width;

        predictionUpsampling1D( horDst, src, m_boundaryForUpsamplingLeft.data(),
                                m_reducedPredictionSize.width, m_reducedPredictionSize.height,
                                horSrcStep, horSrcStride, 1, horDstStride,
                                m_upsmpFactorHor );

        verSrc       = horDst;
        verSrcStep   = horDstStride;
        verSrcStride = 1;
      }
      else
      {
        verSrc       = src;
        verSrcStep   = transpose ? 1 : m_blockSize.width;
        verSrcStride = transpose ? m_reducedPredictionSize.height : 1;
      }
      predictionUpsampling1D( dst, verSrc, m_boundaryForUpsamplingTop.data(),
                              m_reducedPredictionSize.height, m_blockSize.width,
                              verSrcStep, verSrcStride, m_blockSize.width, 1,
                              m_upsmpFactorVer );
    }
    else
    {
      const int* horSrc = nullptr;
      SizeType   horSrcStep = 0;
      SizeType   horSrcStride = 0;
      if( m_upsmpFactorVer > 1 )
      {
        const SizeType verSrcStep   = transpose ? 1 : m_reducedPredictionSize.width;
        const SizeType verSrcStride = transpose ? m_reducedPredictionSize.height : 1;

        int* const     verDst       = dst + ( m_upsmpFactorHor - 1 );
        const SizeType verDstStep   = m_blockSize.width;
        const SizeType verDstStride = m_upsmpFactorHor;

        predictionUpsampling1D( verDst, src, m_boundaryForUpsamplingTop.data(),
                                m_reducedPredictionSize.height, m_reducedPredictionSize.width,
                                verSrcStep, verSrcStride, verDstStep, verDstStride,
                                m_upsmpFactorVer );

        horSrc = verDst;
        horSrcStep = verDstStride;
        horSrcStride = verDstStep;
      }
      else
      {
        horSrc       = src;
        horSrcStep   = transpose ? m_blockSize.height : 1;
        horSrcStride = transpose ? 1 : m_reducedPredictionSize.width;
      }
      predictionUpsampling1D( dst, horSrc, m_boundaryForUpsamplingLeft.data(),
                              m_reducedPredictionSize.width, m_blockSize.height,
                              horSrcStep, horSrcStride, 1, m_blockSize.width,
                              m_upsmpFactorHor );
    }
  }

  void PredictorMIP::getMatrixBias( const short*& matrix, const short*& bias, const int modeIdx ) const
  {
    const int idx = getWeightIdx( modeIdx );

    if( m_blockSize.width == 4 && m_blockSize.height == 4 )
    {
      matrix = &mipMatrix4x4[idx][0][0];
      bias   = &mipBias4x4  [idx][0];
    }
    else if( m_blockSize.width <= 8 && m_blockSize.height <= 8 )
    {
      matrix = &mipMatrix8x8[idx][0][0];
      bias   = &mipBias8x8  [idx][0];
    }
    else
    {
      matrix = &mipMatrix16x16[idx][0][0];
      bias   = &mipBias16x16  [idx][0];
    }
  }

  void PredictorMIP::getShifts( int &shiftMatrix, int &shiftBias, const int modeIdx, const int bitDepth ) const
  {
    const int idx = getWeightIdx( modeIdx );

    if( m_blockSize.width == 4 && m_blockSize.height == 4 )
    {
      shiftMatrix = mipShiftMatrix4x4[idx];
      shiftBias   = mipShiftBias4x4  [idx] + (bitDepth - 10);
    }
    else if( m_blockSize.width <= 8 && m_blockSize.height <= 8 )
    {
      shiftMatrix = mipShiftMatrix8x8[idx];
      shiftBias   = mipShiftBias8x8  [idx] + (bitDepth - 10);
    }
    else
    {
      shiftMatrix = mipShiftMatrix16x16[idx];
      shiftBias   = mipShiftBias16x16  [idx] + (bitDepth - 10);
    }
  }

  void PredictorMIP::xComputeMatrixTimesRedBndryPlusBias( int*const result, const int* const input,
                                                          const short*matrix, const short*bias,
                                                          const bool leaveHorOut, const bool leaveVerOut,
                                                          const int shiftMatrix, const int shiftBias,
                                                          const bool transpose, const bool needUpsampling )
  {
    const int inputSize = m_reducedBoundarySize.width + m_reducedBoundarySize.height;

    // Use local buffer for transposed result if no upsampling will be done.
    static_vector<int, MIP_MAX_REDUCED_OUTPUT_SAMPLES> resBufTransposed( m_reducedPredictionSize.area() );
    int*const resPtr = (transpose && !needUpsampling) ? resBufTransposed.data() : result;

    const int offset = 1 << (shiftMatrix - 1);
    CHECK(inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four");

    const short *weight = matrix;

    const int intermediateWidth  = transpose ? m_reducedPredictionSize.height : m_reducedPredictionSize.width;
    const int intermediateHeight = transpose ? m_reducedPredictionSize.width : m_reducedPredictionSize.height;
    const int xStep = leaveHorOut ? 2 : 1;
    const int yStep = leaveVerOut ? intermediateWidth : 0;

    int posRes  = 0;
    int posBias = 0;
    for (int y = 0; y < intermediateHeight; y++)
    {
      for (int x = 0; x < intermediateWidth; x++)
      {
        int tmp0 = 0;
        int tmp1 = 0;
        int tmp2 = 0;
        int tmp3 = 0;
        for (int i = 0; i < inputSize - 1; i += 4)
        {
          tmp0 += input[i]     * weight[i];
          tmp1 += input[i + 1] * weight[i + 1];
          tmp2 += input[i + 2] * weight[i + 2];
          tmp3 += input[i + 3] * weight[i + 3];
        }
        resPtr[posRes++] = ((tmp0 + tmp1 + tmp2 + tmp3) + (bias[posBias] << shiftBias) + offset) >> shiftMatrix;

        weight  += xStep * inputSize;
        posBias += xStep;
      }
      weight  += yStep * inputSize;
      posBias += yStep;
    }

    // Re-transpose if no upsampling will be done.
    if( transpose && !needUpsampling )
    {
      for( int y = 0; y < m_reducedPredictionSize.height; y++ )
      {
        for( int x = 0; x < m_reducedPredictionSize.width; x++ )
        {
          CHECKD( x * m_reducedPredictionSize.height + y >= m_reducedPredictionSize.area(), "error" );
          result[ y * m_reducedPredictionSize.width + x ] = resPtr[ x * m_reducedPredictionSize.height + y ];
        }
      }
    }
  }


} // namespace Mip

MatrixIntraPrediction::MatrixIntraPrediction()
{
}

void MatrixIntraPrediction::prepareInputForPred(const CPelBuf &src, const Area& puArea, const int bitDepth, const AvailableInfo &availInfo)
{
  m_predictorMip.deriveBoundaryData(src, puArea, bitDepth, availInfo);
}

void MatrixIntraPrediction::predBlock( const Size &puSize, const int intraMode, PelBuf& dst, const int bitDepth )
{
  static_vector<int, MIP_MAX_WIDTH * MIP_MAX_HEIGHT> predMip(puSize.area());
  int* const resultMip = predMip.data();
  m_predictorMip.getPrediction(resultMip, intraMode, bitDepth);

  for (int y = 0; y < puSize.height; y++)
  {
    for (int x = 0; x < puSize.width; x++)
    {
      dst.at(x, y) = Pel(ClipBD<int>(resultMip[y * puSize.width + x], bitDepth));
    }
  }
}

#endif
