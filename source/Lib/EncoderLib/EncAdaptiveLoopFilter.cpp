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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilter.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"

#define AlfCtx(c) SubCtx( Ctx::Alf, c)
std::vector<double> EncAdaptiveLoopFilter::m_lumaLevelToWeightPLUT;

void AlfCovariance::getClipMax(const AlfFilterShape& alfShape, int *clip_max) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    clip_max[k] = 0;

    bool inc = true;
    while( inc && clip_max[k]+1 < numBins && y[clip_max[k]+1][k] == y[clip_max[k]][k] )
    {
      for( int l = 0; inc && l < numCoeff; ++l )
        if( E[clip_max[k]][0][k][l] != E[clip_max[k]+1][0][k][l] )
        {
          inc = false;
        }
      if( inc )
      {
        ++clip_max[k];
      }
    }
  }
  clip_max[numCoeff-1] = 0;
}

void AlfCovariance::reduceClipCost(const AlfFilterShape& alfShape, int *clip) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    bool dec = true;
    while( dec && clip[k] > 0 && y[clip[k]-1][k] == y[clip[k]][k] )
    {
      for( int l = 0; dec && l < numCoeff; ++l )
        if( E[clip[k]][clip[l]][k][l] != E[clip[k]-1][clip[l]][k][l] )
        {
          dec = false;
        }
      if( dec )
      {
        --clip[k];
      }
    }
  }
}

double AlfCovariance::optimizeFilter(const AlfFilterShape& alfShape, int* clip, double *f, bool optimize_clip) const
{
  const int size = alfShape.numCoeff;
  int clip_max[MAX_NUM_ALF_LUMA_COEFF];

  double err_best, err_last;

  TE kE;
  Ty ky;

  if( optimize_clip )
  {
    // Start by looking for min clipping that has no impact => max_clipping
    getClipMax(alfShape, clip_max);
    for (int k=0; k<size; ++k)
    {
      clip[k] = std::max(clip_max[k], clip[k]);
      clip[k] = std::min(clip[k], numBins-1);
    }
  }

  setEyFromClip( clip, kE, ky, size );

  gnsSolveByChol( kE, ky, f, size );
  err_best = calculateError( clip, f, size );

  int step = optimize_clip ? (numBins+1)/2 : 0;

  while( step > 0 )
  {
    double err_min = err_best;
    int idx_min = -1;
    int inc_min = 0;

    for( int k = 0; k < size-1; ++k )
    {
      if( clip[k] - step >= clip_max[k] )
      {
        clip[k] -= step;
        ky[k] = y[clip[k]][k];
        for( int l = 0; l < size; l++ )
        {
          kE[k][l] = E[clip[k]][clip[l]][k][l];
          kE[l][k] = E[clip[l]][clip[k]][l][k];
        }

        gnsSolveByChol( kE, ky, f, size );
        err_last = calculateError( clip, f, size );

        if( err_last < err_min )
        {
          err_min = err_last;
          idx_min = k;
          inc_min = -step;
        }
        clip[k] += step;
      }
      if( clip[k] + step < numBins )
      {
        clip[k] += step;
        ky[k] = y[clip[k]][k];
        for( int l = 0; l < size; l++ )
        {
          kE[k][l] = E[clip[k]][clip[l]][k][l];
          kE[l][k] = E[clip[l]][clip[k]][l][k];
        }

        gnsSolveByChol( kE, ky, f, size );
        err_last = calculateError( clip, f, size );

        if( err_last < err_min )
        {
          err_min = err_last;
          idx_min = k;
          inc_min = step;
        }
        clip[k] -= step;

      }
      ky[k] = y[clip[k]][k];
      for( int l = 0; l < size; l++ )
      {
        kE[k][l] = E[clip[k]][clip[l]][k][l];
        kE[l][k] = E[clip[l]][clip[k]][l][k];
      }
    }

    if( idx_min >= 0 )
    {
      err_best = err_min;
      clip[idx_min] += inc_min;
      ky[idx_min] = y[clip[idx_min]][idx_min];
      for( int l = 0; l < size; l++ )
      {
        kE[idx_min][l] = E[clip[idx_min]][clip[l]][idx_min][l];
        kE[l][idx_min] = E[clip[l]][clip[idx_min]][l][idx_min];
      }
    }
    else
    {
      --step;
    }
  }

  if( optimize_clip ) {
    // test all max
    for( int k = 0; k < size-1; ++k )
    {
      clip_max[k] = 0;
    }
    TE kE_max;
    Ty ky_max;
    setEyFromClip( clip_max, kE_max, ky_max, size );

    gnsSolveByChol( kE_max, ky_max, f, size );
    err_last = calculateError( clip_max, f, size );
    if( err_last < err_best )
    {
      err_best = err_last;
      for (int k=0; k<size; ++k)
      {
        clip[k] = clip_max[k];
      }
    }
    else
    {
      // update clip to reduce coding cost
      reduceClipCost(alfShape, clip);

      // update f with best solution
      gnsSolveByChol( kE, ky, f, size );
    }
  }

  return err_best;
}

double AlfCovariance::calcErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const int bitDepth ) const
{
  double factor = 1 << ( bitDepth - 1 );
  double error = 0;

  for( int i = 0; i < numCoeff; i++ )   //diagonal
  {
    double sum = 0;
    for( int j = i + 1; j < numCoeff; j++ )
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E[clip[i]][clip[j]][i][j] * coeff[j];
    }
    error += ( ( E[clip[i]][clip[i]][i][i] * coeff[i] + sum * 2 ) / factor - 2 * y[clip[i]][i] ) * coeff[i];
  }

  return error / factor;
}

double AlfCovariance::calculateError( const int *clip, const double *coeff, const int numCoeff ) const
{
  double sum = 0;
  for( int i = 0; i < numCoeff; i++ )
  {
    sum += coeff[i] * y[clip[i]][i];
  }

  return pixAcc - sum;
}

double AlfCovariance::calculateError( const int *clip ) const
{
  Ty c;

  return optimizeFilter( clip, c, numCoeff );
}
//********************************
// Cholesky decomposition
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//Find filter coeff related
int AlfCovariance::gnsCholeskyDec( TE inpMatr, TE outMatr, int numEq ) const
{
  Ty invDiag;  /* Vector of the inverse of diagonal entries of outMatr */

  for( int i = 0; i < numEq; i++ )
  {
    for( int j = i; j < numEq; j++ )
    {
      /* Compute the scaling factor */
      double scale = inpMatr[i][j];
      if( i > 0 )
      {
        for( int k = i - 1; k >= 0; k-- )
        {
          scale -= outMatr[k][j] * outMatr[k][i];
        }
      }

      /* Compute i'th row of outMatr */
      if( i == j )
      {
        if( scale <= REG_SQR ) // if(scale <= 0 )  /* If inpMatr is singular */
        {
          return 0;
        }
        else              /* Normal operation */
          invDiag[i] = 1.0 / ( outMatr[i][i] = sqrt( scale ) );
      }
      else
      {
        outMatr[i][j] = scale * invDiag[i]; /* Upper triangular part          */
        outMatr[j][i] = 0.0;              /* Lower triangular part set to 0 */
      }
    }
  }
  return 1; /* Signal that Cholesky factorization is successfully performed */
}

void AlfCovariance::gnsTransposeBacksubstitution( TE U, double* rhs, double* x, int order ) const
{
  /* Backsubstitution starts */
  x[0] = rhs[0] / U[0][0];               /* First row of U'                   */
  for( int i = 1; i < order; i++ )
  {         /* For the rows 1..order-1           */

    double sum = 0; //Holds backsubstitution from already handled rows

    for( int j = 0; j < i; j++ ) /* Backsubst already solved unknowns */
    {
      sum += x[j] * U[j][i];
    }

    x[i] = ( rhs[i] - sum ) / U[i][i];       /* i'th component of solution vect.  */
  }
}

void AlfCovariance::gnsBacksubstitution( TE R, double* z, int size, double* A ) const
{
  size--;
  A[size] = z[size] / R[size][size];

  for( int i = size - 1; i >= 0; i-- )
  {
    double sum = 0;

    for( int j = i + 1; j <= size; j++ )
    {
      sum += R[i][j] * A[j];
    }

    A[i] = ( z[i] - sum ) / R[i][i];
  }
}

int AlfCovariance::gnsSolveByChol( const int *clip, double *x, int numEq ) const
{
  TE LHS;
  Ty rhs;

  setEyFromClip( clip, LHS, rhs, numEq );
  return gnsSolveByChol( LHS, rhs, x, numEq );
}

int AlfCovariance::gnsSolveByChol( TE LHS, double* rhs, double *x, int numEq ) const
{
  Ty aux;     /* Auxiliary vector */
  TE U;    /* Upper triangular Cholesky factor of LHS */

  int res = 1;  // Signal that Cholesky factorization is successfully performed

                /* The equation to be solved is LHSx = rhs */

                /* Compute upper triangular U such that U'*U = LHS */
  if( gnsCholeskyDec( LHS, U, numEq ) ) /* If Cholesky decomposition has been successful */
  {
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
    * Solve U'*aux = rhs for aux
    */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution( U, aux, numEq, x );

  }
  else /* LHS was singular */
  {
    res = 0;

    /* Regularize LHS */
    for( int i = 0; i < numEq; i++ )
    {
      LHS[i][i] += REG;
    }

    /* Compute upper triangular U such that U'*U = regularized LHS */
    res = gnsCholeskyDec( LHS, U, numEq );

    if( !res )
    {
      std::memset( x, 0, sizeof( double )*numEq );
      return 0;
    }

    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* Solve U*x = aux for x */
    gnsBacksubstitution( U, aux, numEq, x );
  }
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////

EncAdaptiveLoopFilter::EncAdaptiveLoopFilter()
  : m_CABACEstimator( nullptr )
{
  for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_alfCovariance[i] = nullptr;
  }
  for( int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_alfCovarianceFrame[i] = nullptr;
  }
  m_filterCoeffSet = nullptr;
  m_filterClippSet = nullptr;
  m_diffFilterCoeff = nullptr;

  m_alfWSSD = 0;
}

void EncAdaptiveLoopFilter::create( const EncCfg* encCfg, const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] )
{
  AdaptiveLoopFilter::create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth );
  CHECK( encCfg == nullptr, "encCfg must not be null" );
  m_encCfg = encCfg;

  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    ChannelType chType = (ChannelType)channelIdx;
    int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
    m_alfCovarianceFrame[chType] = new AlfCovariance*[m_filterShapes[chType].size()];
    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovarianceFrame[chType][i] = new AlfCovariance[numClasses];
      for( int k = 0; k < numClasses; k++ )
      {
        m_alfCovarianceFrame[chType][i][k].create( m_filterShapes[chType][i].numCoeff );
      }
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlagTmp[compIdx] = new uint8_t[m_numCTUsInPic];
    ChannelType chType = toChannelType( ComponentID( compIdx ) );
    int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

    m_alfCovariance[compIdx] = new AlfCovariance**[m_filterShapes[chType].size()];

    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovariance[compIdx][i] = new AlfCovariance*[m_numCTUsInPic];
      for( int j = 0; j < m_numCTUsInPic; j++ )
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance[numClasses];
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovariance[compIdx][i][j][k].create( m_filterShapes[chType][i].numCoeff );
        }
      }
    }
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].create( m_filterShapes[COMPONENT_Y][i].numCoeff );
    }
  }

  m_filterCoeffSet = new int*[MAX_NUM_ALF_CLASSES];
  m_filterClippSet = new int*[MAX_NUM_ALF_CLASSES];
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];

  for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_filterClippSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }

  m_apsIdStart = (int)MAX_NUM_APS;
  m_ctbDistortionFixedFilter = new double[m_numCTUsInPic];
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    m_ctbDistortionUnfilter[comp] = new double[m_numCTUsInPic];
  }
  m_alfCtbFilterSetIndexTmp.resize(m_numCTUsInPic);
  memset(m_clipDefaultEnc, 0, sizeof(m_clipDefaultEnc));
}

void EncAdaptiveLoopFilter::destroy()
{
  if (!m_created)
  {
    return;
  }
  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    if( m_alfCovarianceFrame[channelIdx] )
    {
      ChannelType chType = (ChannelType)channelIdx;
      int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovarianceFrame[channelIdx][i][k].destroy();
        }
        delete[] m_alfCovarianceFrame[channelIdx][i];
        m_alfCovarianceFrame[channelIdx][i] = nullptr;
      }
      delete[] m_alfCovarianceFrame[channelIdx];
      m_alfCovarianceFrame[channelIdx] = nullptr;
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    if( m_ctuEnableFlagTmp[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp[compIdx];
      m_ctuEnableFlagTmp[compIdx] = nullptr;
    }

    if( m_alfCovariance[compIdx] )
    {
      ChannelType chType = toChannelType( ComponentID( compIdx ) );
      int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int j = 0; j < m_numCTUsInPic; j++ )
        {
          for( int k = 0; k < numClasses; k++ )
          {
            m_alfCovariance[compIdx][i][j][k].destroy();
          }
          delete[] m_alfCovariance[compIdx][i][j];
          m_alfCovariance[compIdx][i][j] = nullptr;

        }
        delete[] m_alfCovariance[compIdx][i];
        m_alfCovariance[compIdx][i] = nullptr;

      }
      delete[] m_alfCovariance[compIdx];
      m_alfCovariance[compIdx] = nullptr;
    }
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].destroy();
    }
  }

  if( m_filterCoeffSet )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if( m_filterClippSet )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_filterClippSet[i];
      m_filterClippSet[i] = nullptr;
    }
    delete[] m_filterClippSet;
    m_filterClippSet = nullptr;
  }

  if( m_diffFilterCoeff )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_diffFilterCoeff[i];
      m_diffFilterCoeff[i] = nullptr;
    }
    delete[] m_diffFilterCoeff;
    m_diffFilterCoeff = nullptr;
  }


  delete[] m_ctbDistortionFixedFilter;
  m_ctbDistortionFixedFilter = nullptr;
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    delete[] m_ctbDistortionUnfilter[comp];
    m_ctbDistortionUnfilter[comp] = nullptr;
  }
  AdaptiveLoopFilter::destroy();
}
void EncAdaptiveLoopFilter::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice
, ParameterSetMap<APS>* apsMap )
{
  m_apsMap = apsMap;
  m_CABACEstimator = cabacEncoder->getCABACEstimator( pcSlice->getSPS() );
  m_CtxCache = ctxCache;
  m_CABACEstimator->initCtxModels( *pcSlice );
  m_CABACEstimator->resetBits();
}

void EncAdaptiveLoopFilter::ALFProcess(CodingStructure& cs, const double *lambdas
#if ENABLE_QPA
                                       , const double lambdaChromaWeight
#endif
                                      )
{
  if (cs.slice->getPendingRasInit() || cs.slice->isIDRorBLA())
  {
    memset(cs.slice->getAlfAPSs(), 0, sizeof(*cs.slice->getAlfAPSs())*MAX_NUM_APS);
    m_apsIdStart = (int)MAX_NUM_APS;
    m_apsMap->clear();
    for (int i = 0; i < MAX_NUM_APS; i++)
    {
      APS* alfAPS = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsMap->clearChangedFlag((i << NUM_APS_TYPE_LEN) + ALF_APS);
      if (alfAPS)
        alfAPS = nullptr;
    }
  }
  AlfSliceParam alfSliceParam;
  alfSliceParam.reset();
  const TempCtx  ctxStart(m_CtxCache, AlfCtx(m_CABACEstimator->getCtx()));
  // set available filter shapes
  alfSliceParam.filterShapes = m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU ALF enable flags, it was already reset before ALF process
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
  }

  // reset ALF parameters
  alfSliceParam.reset();
  int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_LUMA]);
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_CHROMA]);
  m_lambda[COMPONENT_Y] = lambdas[COMPONENT_Y] * double(1 << shiftLuma);
  m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb] * double(1 << shiftChroma);
  m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr] * double(1 << shiftChroma);

  PelUnitBuf orgYuv = cs.getOrgBuf();

  m_tempBuf.copyFrom( cs.getRecoBuf() );
  PelUnitBuf recYuv = m_tempBuf.getBuf( cs.area );
  recYuv.extendBorderPel( MAX_ALF_FILTER_LENGTH >> 1 );

  // derive classification
  const CPelBuf& recLuma = recYuv.get( COMPONENT_Y );
  const PreCalcValues& pcv = *cs.pcv;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
      const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;

      if( isCrossedByVirtualBoundaries( xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS() ) )
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );

          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );

            const int wBuf = w + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
            const int hBuf = h + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
            PelUnitBuf buf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
            buf.copyFrom( recYuv.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf ) ) ) );
            buf.extendBorderPel( MAX_ALF_PADDING_SIZE );
            buf = buf.subBuf( UnitArea ( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

            const Area blkSrc( 0, 0, w, h );
            const Area blkDst( xStart, yStart, w, h );
            deriveClassification( m_classifier, buf.get(COMPONENT_Y), blkDst, blkSrc );
            Area blkPCM( xStart, yStart, w, h );
            resetPCMBlkClassInfo( cs, m_classifier, buf.get(COMPONENT_Y), blkPCM );

            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {
        Area blk( xPos, yPos, width, height );
        deriveClassification( m_classifier, recLuma, blk, blk );
        Area blkPCM( xPos, yPos, width, height );
        resetPCMBlkClassInfo( cs, m_classifier, recLuma, blkPCM );
      }
    }
  }

  // get CTB stats for filtering
  deriveStatsForFiltering( orgYuv, recYuv, cs );

  // derive filter (luma)
  alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_LUMA
#if ENABLE_QPA
            , lambdaChromaWeight
#endif
            );

  // derive filter (chroma)
  {
    alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_CHROMA
#if ENABLE_QPA
              , lambdaChromaWeight
#endif
              );
  }

  m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
  alfEncoderCtb(cs, alfSliceParam
#if ENABLE_QPA
    , lambdaChromaWeight
#endif
  );

  alfReconstructor(cs, recYuv);
}

double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel,
#if ENABLE_QPA
                                                       const double chromaWeight,
#endif
                                                       const int numClasses, const int numCoeff, double& distUnfilter )
{
  TempCtx        ctxTempStart( m_CtxCache );
  TempCtx        ctxTempBest( m_CtxCache );
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;

  double cost = 0;
  distUnfilter = 0;

  setEnableFlag(m_alfSliceParamTemp, channel, true);
#if ENABLE_QPA
  CHECK ((chromaWeight > 0.0) && (cs.slice->getSliceCurStartCtuTsAddr() != 0), "incompatible start CTU address, must be 0");
#endif

  reconstructCoeff(m_alfSliceParamTemp, channel, true, isLuma(channel));
  for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
  {
    for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
    {
      m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaCoeffFinal[i];
      m_filterClippSet[classIdx][i] = isLuma(channel) ? m_clippFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaClippFinal[i];
    }
  }

  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    for( int compID = compIDFirst; compID <= compIDLast; compID++ )
    {
      double distUnfilterCtu = getUnfilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses );

      ctxTempStart = AlfCtx( m_CABACEstimator->getCtx() );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 1;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfSliceParamTemp );
      double costOn = distUnfilterCtu + getFilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfSliceParamTemp.numLumaFilters - 1, numCoeff );
#if ENABLE_QPA
      const double ctuLambda = chromaWeight > 0.0 ? (isLuma (channel) ? cs.picture->m_uEnerHpCtu[ctuIdx] : cs.picture->m_uEnerHpCtu[ctuIdx] / chromaWeight) : m_lambda[compID];
#else
      const double ctuLambda = m_lambda[compID];
#endif
      costOn += ctuLambda * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
      ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );

      m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 0;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfSliceParamTemp);
      double costOff = distUnfilterCtu + ctuLambda * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();

      if( costOn < costOff )
      {
        cost += costOn;
        m_CABACEstimator->getCtx() = AlfCtx( ctxTempBest );
        m_ctuEnableFlag[compID][ctuIdx] = 1;
      }
      else
      {
        cost += costOff;
        m_ctuEnableFlag[compID][ctuIdx] = 0;
        distUnfilter += distUnfilterCtu;
      }
    }
  }

  if( isChroma( channel ) )
  {
    setEnableFlag(m_alfSliceParamTemp, channel, m_ctuEnableFlag);
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    cost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[channel];
  }

  return cost;
}

void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfSliceParam& alfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel
#if ENABLE_QPA
                                      , const double lambdaChromaWeight // = 0.0
#endif
                                      )
{
  const TempCtx  ctxStart( m_CtxCache, AlfCtx( m_CABACEstimator->getCtx() ) );
  TempCtx        ctxBest( m_CtxCache );

  double costMin = MAX_DOUBLE;

  std::vector<AlfFilterShape>& alfFilterShape = alfSliceParam.filterShapes[channel];
  m_bitsNewFilter[channel] = 0;
  const int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;

  for( int iShapeIdx = 0; iShapeIdx < alfFilterShape.size(); iShapeIdx++ )
  {
    m_alfSliceParamTemp = alfSliceParam;
    //1. get unfiltered distortion
    double cost = getUnfilteredDistortion( m_alfCovarianceFrame[channel][iShapeIdx], channel );
    cost /= 1.001; // slight preference for unfiltered choice

    if( cost < costMin )
    {
      costMin = cost;
      setEnableFlag( alfSliceParam, channel, false );
      // no CABAC signalling
      ctxBest = AlfCtx( ctxStart );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 0 );
    }

    const int nonLinearFlagMax =
      ( isLuma( channel ) ? m_encCfg->getUseNonLinearAlfLuma() : m_encCfg->getUseNonLinearAlfChroma() )
      ? 2 : 1;

    for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
    {
    //2. all CTUs are on
    setEnableFlag( m_alfSliceParamTemp, channel, true );
    m_alfSliceParamTemp.nonLinearFlag[channel] = nonLinearFlag;
    m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
    setCtuEnableFlag( m_ctuEnableFlag, channel, 1 );
    cost = getFilterCoeffAndCost( cs, 0, channel, nonLinearFlag != 0, iShapeIdx, uiCoeffBits );

    if( cost < costMin )
    {
      m_bitsNewFilter[channel] = uiCoeffBits;
      costMin = cost;
      copyAlfSliceParam( alfSliceParam, m_alfSliceParamTemp, channel );
      ctxBest = AlfCtx( m_CABACEstimator->getCtx() );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 1 );
    }

    //3. CTU decision
    double distUnfilter = 0;
    const int iterNum = isLuma(channel) ? (2 * 4 + 1) : (2 * 2 + 1);

    for( int iter = 0; iter < iterNum; iter++ )
    {
      if ((iter & 0x01) == 0)
      {
        m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
        cost = m_lambda[channel] * uiCoeffBits;
        cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, channel,
#if ENABLE_QPA
                                        lambdaChromaWeight,
#endif
                                        numClasses, alfFilterShape[iShapeIdx].numCoeff, distUnfilter );
        if (cost < costMin)
        {
          m_bitsNewFilter[channel] = uiCoeffBits;
          costMin = cost;
          ctxBest = AlfCtx(m_CABACEstimator->getCtx());
          copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
          copyAlfSliceParam(alfSliceParam, m_alfSliceParamTemp, channel);
        }
      }
      else
      {
        // unfiltered distortion is added due to some CTBs may not use filter
        cost = getFilterCoeffAndCost(cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits);
      }
    }//for iter
    }// for nonLineaFlag
  }//for shapeIdx
  m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
}

void EncAdaptiveLoopFilter::copyAlfSliceParam( AlfSliceParam& alfSliceParamDst, AlfSliceParam& alfSliceParamSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( &alfSliceParamDst, &alfSliceParamSrc, sizeof( AlfSliceParam ) );
  }
  else
  {
    alfSliceParamDst.nonLinearFlag[channel] = alfSliceParamSrc.nonLinearFlag[channel];
    alfSliceParamDst.enabledFlag[COMPONENT_Cb] = alfSliceParamSrc.enabledFlag[COMPONENT_Cb];
    alfSliceParamDst.enabledFlag[COMPONENT_Cr] = alfSliceParamSrc.enabledFlag[COMPONENT_Cr];
    memcpy( alfSliceParamDst.chromaCoeff, alfSliceParamSrc.chromaCoeff, sizeof( alfSliceParamDst.chromaCoeff ) );
    memcpy( alfSliceParamDst.chromaClipp, alfSliceParamSrc.chromaClipp, sizeof( alfSliceParamDst.chromaClipp ) );
  }
}
double EncAdaptiveLoopFilter::getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, bool onlyFilterCost )
{
  //collect stat based on CTU decision
  if( bReCollectStat )
  {
    getFrameStats( channel, iShapeIdx );
  }

  double dist = distUnfilter;
  uiCoeffBits = 0;
  int uiSliceFlag = 0;
  AlfFilterShape& alfFilterShape = m_alfSliceParamTemp.filterShapes[channel][iShapeIdx];
  //get filter coeff
  if( isLuma( channel ) )
  {
    std::fill_n(m_alfClipMerged[iShapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfSliceParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    // Reset Merge Tmp Cov
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES].reset(AlfNumClippingValues[channel]);
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES + 1].reset(AlfNumClippingValues[channel]);
    //distortion
    dist += mergeFiltersAndCost( m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], m_alfClipMerged[iShapeIdx], uiCoeffBits );
  }
  else
  {
    //distortion
    assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][iShapeIdx][0].numCoeff);
    std::fill_n(m_filterClippSet[0], MAX_NUM_ALF_CHROMA_COEFF, m_alfSliceParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues[CHANNEL_TYPE_CHROMA] / 2 : 0);
    dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterClippSet[0], m_filterCoeffSet[0], m_alfCovarianceFrame[channel][iShapeIdx][0], alfFilterShape, m_NUM_BITS, m_alfSliceParamTemp.nonLinearFlag[channel] );
    //setEnableFlag( m_alfSliceParamTemp, channel, m_ctuEnableFlag );
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
    {
      m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffSet[0][i];
      m_alfSliceParamTemp.chromaClipp[i] = m_filterClippSet[0][i];
    }
    uiCoeffBits += getCoeffRate( m_alfSliceParamTemp, true );
    uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  }
  if (onlyFilterCost)
  {
    return dist + m_lambda[channel] * uiCoeffBits;
  }
  double rate = uiCoeffBits + uiSliceFlag;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->codeAlfCtuEnableFlags( cs, channel, &m_alfSliceParamTemp);
  rate += FracBitsScale * (double)m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[channel] * rate;
}

int EncAdaptiveLoopFilter::getCoeffRate( AlfSliceParam& alfSliceParam, bool isChroma )
{
  int iBits = 0;
  assert( isChroma );

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  AlfFilterShape alfShape( 5 );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  const int numFilters = 1;

  // vlc for all
  for( int i = 0; i < alfShape.numCoeff - 1; i++ )
  {
    int coeffVal = abs( alfSliceParam.chromaCoeff[i] );

    for( int k = 1; k < 15; k++ )
    {
      m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Golomb parameters
  iBits += lengthUvlc( kMin - 1 );  // "min_golomb_order"
  int golombOrderIncreaseFlag = 0;

  for( int idx = 0; idx < maxGolombIdx; idx++ )
  {
    golombOrderIncreaseFlag = ( m_kMinTab[idx] != kMin ) ? 1 : 0;
    CHECK( !( m_kMinTab[idx] <= kMin + 1 ), "ALF Golomb parameter not consistent" );
    iBits += golombOrderIncreaseFlag;                           //golomb_order_increase_flag
    kMin = m_kMinTab[idx];
  }

  // Filter coefficients
  for( int i = 0; i < alfShape.numCoeff - 1; i++ )
  {
    iBits += lengthGolomb( alfSliceParam.chromaCoeff[i], m_kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
  }

  if( m_alfSliceParamTemp.nonLinearFlag[isChroma] )
  {
    memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
    // vlc for all
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      if( !abs( alfSliceParam.chromaCoeff[i] ) )
        continue;
      int coeffVal = abs( alfSliceParam.chromaClipp[i] );

      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k, false );
      }
    }

    kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

    // Golomb parameters
    iBits += lengthUvlc( kMin - 1 );  // "min_golomb_order"
    golombOrderIncreaseFlag = 0;

    for( int idx = 0; idx < maxGolombIdx; idx++ )
    {
      golombOrderIncreaseFlag = ( m_kMinTab[idx] != kMin ) ? 1 : 0;
      CHECK( !( m_kMinTab[idx] <= kMin + 1 ), "ALF Golomb parameter not consistent" );
      iBits += golombOrderIncreaseFlag;                           //golomb_order_increase_flag
      kMin = m_kMinTab[idx];
    }

    // Filter coefficients
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      if( !abs( alfSliceParam.chromaCoeff[i] ) )
        continue;
      iBits += lengthGolomb( alfSliceParam.chromaClipp[i], m_kMinTab[alfShape.golombIdx[i]], false );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
    }
  }
  return iBits;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel )
{
  double dist = 0;
  if( isLuma( channel ) )
  {
    dist = getUnfilteredDistortion( cov, MAX_NUM_ALF_CLASSES );
  }
  else
  {
    dist = getUnfilteredDistortion( cov, 1 ) + lengthTruncatedUnary( 0, 3 ) * m_lambda[COMPONENT_Cb];
  }
  return dist;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, const int numClasses )
{
  double dist = 0;
  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    dist += cov[classIdx].pixAcc;
  }
  return dist;
}

double EncAdaptiveLoopFilter::getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff )
{
  double dist = 0;

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    dist += cov[classIdx].calcErrorForCoeffs(m_filterClippSet[classIdx], m_filterCoeffSet[classIdx], numCoeff, m_NUM_BITS);
  }

  return dist;
}

double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits )
{
  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  static bool codedVarBins[MAX_NUM_ALF_CLASSES];
  static double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = MAX_DOUBLE;
  int predMode = 0, bestPredMode = 0, coeffBits, coeffBitsForce0;

  mergeClasses( alfShape, covFrame, covMerged, clipMerged, MAX_NUM_ALF_CLASSES, m_filterIndices );

  while( numFilters >= 1 )
  {
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfSliceParam);
    // filter coeffs are stored in m_filterCoeffSet
    distForce0 = getDistForce0( alfShape, numFilters, errorForce0CoeffTab, codedVarBins );
    coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters, predMode );
    coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFilters, codedVarBins );

    cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

    if( cost0 < cost )
    {
      cost = cost0;
    }

    if (alfSliceParam.fixedFilterSetIndex > 0)
    {
      int len = 0;
      len += getTBlength(alfSliceParam.fixedFilterSetIndex - 1, NUM_FIXED_FILTER_SETS);
      len += 1; //fixed filter flag pattern
      if (alfSliceParam.fixedFilterPattern > 0)
      {
        len += MAX_NUM_ALF_CLASSES;  //"fixed_filter_flag" for each class
      }
      cost += m_lambda[COMPONENT_Y] * len;
    }

    if( cost <= costMin )
    {
      costMin = cost;
      numFiltersBest = numFilters;
      bestPredMode = predMode;
    }
    numFilters--;
  }

  dist = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab, alfSliceParam );
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest, predMode );
  distForce0 = getDistForce0( alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins );
  coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins );

  cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

  alfSliceParam.numLumaFilters = numFiltersBest;
  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfSliceParam.alfLumaCoeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
    alfSliceParam.alfLumaCoeffDeltaPredictionFlag = bestPredMode;
  }
  else
  {
    distReturn = distForce0;
    alfSliceParam.alfLumaCoeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    memcpy( alfSliceParam.alfLumaCoeffFlag, codedVarBins, sizeof( codedVarBins ) );
    alfSliceParam.alfLumaCoeffDeltaPredictionFlag = 0;

    for( int varInd = 0; varInd < numFiltersBest; varInd++ )
    {
      if( codedVarBins[varInd] == 0 )
      {
        memset( m_filterCoeffSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
        memset( m_filterClippSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
      }
    }
  }

  for( int ind = 0; ind < alfSliceParam.numLumaFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff; i++ )
    {
      if( alfSliceParam.alfLumaCoeffDeltaPredictionFlag )
      {
        alfSliceParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_diffFilterCoeff[ind][i];
      }
      else
      {
        alfSliceParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      }
      alfSliceParam.lumaClipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterClippSet[ind][i];
    }
  }

  memcpy( alfSliceParam.filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof( short ) * MAX_NUM_ALF_CLASSES );
  uiCoeffBits += getNonFilterCoeffRate( alfSliceParam );
  return distReturn;
}

int EncAdaptiveLoopFilter::getNonFilterCoeffRate( AlfSliceParam& alfSliceParam )
{
  int len = 1   // alf_coefficients_delta_flag
          + lengthTruncatedUnary( 0, 3 )    // chroma_idc = 0, it is signalled when ALF is enabled for luma
          + getTBlength( alfSliceParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES );   //numLumaFilters

  if( alfSliceParam.numLumaFilters > 1 )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      len += getTBlength( (int)alfSliceParam.filterCoeffDeltaIdx[i], alfSliceParam.numLumaFilters );  //filter_coeff_delta[i]
    }
  }
  len++; //fixed filter set flag
  if (alfSliceParam.fixedFilterSetIndex > 0)
  {
    len += getTBlength(alfSliceParam.fixedFilterSetIndex - 1, NUM_FIXED_FILTER_SETS);
    len += 1; //fixed filter flag pattern
    if (alfSliceParam.fixedFilterPattern > 0)
      len += MAX_NUM_ALF_CLASSES;  //"fixed_filter_flag" for each class
  }
  return len;
}

int EncAdaptiveLoopFilter::lengthTruncatedUnary( int symbol, int maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return 0;
  }

  bool codeLast = ( maxSymbol > symbol );
  int numBins = 0;
  while( symbol-- )
  {
    numBins++;
  }
  if( codeLast )
  {
    numBins++;
  }

  return numBins;
}

int EncAdaptiveLoopFilter::getTBlength( int uiSymbol, const int uiMaxSymbol )
{
  int uiThresh;
  if( uiMaxSymbol > 256 )
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while( uiThreshVal <= uiMaxSymbol )
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_tbMax[uiMaxSymbol];
  }

  int uiVal = 1 << uiThresh;
  assert( uiVal <= uiMaxSymbol );
  assert( ( uiVal << 1 ) > uiMaxSymbol );
  assert( uiSymbol < uiMaxSymbol );
  int b = uiMaxSymbol - uiVal;
  assert( b < uiVal );
  if( uiSymbol < uiVal - b )
  {
    return uiThresh;
  }
  else
  {
    return uiThresh + 1;
  }
}

int EncAdaptiveLoopFilter::getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins )
{
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );
  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !codedVarBins[ind] )
    {
      continue;
    }
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( pDiffQFilterCoeffIntPP[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Coding parameters
  int len = kMin           //min_golomb_order
          + maxGolombIdx   //golomb_order_increase_flag
          + numFilters;    //filter_coefficient_flag[i]

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( codedVarBins[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        len += lengthGolomb( abs( pDiffQFilterCoeffIntPP[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] ); // alf_coeff_luma_delta[i][j]
      }
    }
  }

  if( m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
  {
    memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );

    for( int ind = 0; ind < numFilters; ++ind )
    {
      if( !codedVarBins[ind] )
      {
        continue;
      }
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        if( !abs( pDiffQFilterCoeffIntPP[ind][i] ) )
          continue;
        int coeffVal = abs( m_filterClippSet[ind][i] );
        for( int k = 1; k < 15; k++ )
        {
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k, false );
        }
      }
    }

    kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

    // Coding parameters
    len += kMin           //min_golomb_order
        + maxGolombIdx   //golomb_order_increase_flag
      ;

    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {
      if( codedVarBins[ind] )
      {
        for( int i = 0; i < alfShape.numCoeff - 1; i++ )
        {
          if( !abs( pDiffQFilterCoeffIntPP[ind][i] ) )
            continue;
          len += lengthGolomb( abs( m_filterClippSet[ind][i] ), m_kMinTab[alfShape.golombIdx[i]], false ); // alf_coeff_luma_delta[i][j]
        }
      }
    }
  }

  return len;
}

int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode )
{
  int ratePredMode0 = getCostFilterCoeff( alfShape, filterSet, numFilters );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( ind == 0 )
    {
      memcpy( filterCoeffDiff[ind], filterSet[ind], sizeof( int ) * alfShape.numCoeff );
    }
    else
    {
      for( int i = 0; i < alfShape.numCoeff; i++ )
      {
        filterCoeffDiff[ind][i] = filterSet[ind][i] - filterSet[ind - 1][i];
      }
    }
  }

  int ratePredMode1 = getCostFilterCoeff( alfShape, filterCoeffDiff, numFilters );

  predMode = ( ratePredMode1 < ratePredMode0 && numFilters > 1 ) ? 1 : 0;

  int rateClipp = m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? getCostFilterClipp( alfShape, filterSet, numFilters ) : 0;

  return ( numFilters > 1 ? 1 : 0 )        // coeff_delta_pred_mode_flag
       + rateClipp
       + ( predMode ? ratePredMode1 : ratePredMode0 ); // min_golomb_order, golomb_order_increase_flag, alf_coeff_luma_delta
}

int EncAdaptiveLoopFilter::getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( pDiffQFilterCoeffIntPP[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Coding parameters
  int len = kMin           //min_golomb_order
          + maxGolombIdx;  //golomb_order_increase_flag

  // Filter coefficients
  len += lengthFilterCoeffs( alfShape, numFilters, pDiffQFilterCoeffIntPP, m_kMinTab );  // alf_coeff_luma_delta[i][j]

  return len;
}

int EncAdaptiveLoopFilter::getCostFilterClipp( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  for( int filterIdx = 0; filterIdx < numFilters; ++filterIdx )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      if( !abs( pDiffQFilterCoeffIntPP[filterIdx][i] ) )
        continue;
      int clippVal = abs( m_filterClippSet[filterIdx][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( clippVal, k );
      }
    }
  }
  int len = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );
  return len           //min_golomb_order
          + getMaxGolombIdx( alfShape.filterType ) //golomb_order_increase_flag
          + lengthFilterClipps( alfShape, numFilters, pDiffQFilterCoeffIntPP, m_kMinTab ); // Filter clippings
}

int EncAdaptiveLoopFilter::lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab )
{
  int bitCnt = 0;

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitCnt += lengthGolomb( abs( FilterCoeff[ind][i] ), kMinTab[alfShape.golombIdx[i]] );
    }
  }
  return bitCnt;
}

int EncAdaptiveLoopFilter::lengthFilterClipps( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab )
{
  int bitCnt = 0;

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      if( !abs( FilterCoeff[ind][i] ) )
        continue;
      bitCnt += lengthGolomb( abs( m_filterClippSet[ind][i] ), kMinTab[alfShape.golombIdx[i]], false );
    }
  }
  return bitCnt;
}

double EncAdaptiveLoopFilter::getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins )
{
  static int bitsVarBin[MAX_NUM_ALF_CLASSES];

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( m_filterCoeffSet[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    bitsVarBin[ind] = 0;
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitsVarBin[ind] += lengthGolomb( abs( m_filterCoeffSet[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] );
    }
  }

  if( m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
  {
    memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
    for( int ind = 0; ind < numFilters; ++ind )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        if( !abs( m_filterCoeffSet[ind][i] ) )
          continue;
        int coeffVal = abs( m_filterClippSet[ind][i] );
        for( int k = 1; k < 15; k++ )
        {
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k, false );
        }
      }
    }

    getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

    for( int ind = 0; ind < numFilters; ++ind )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        if( !abs( m_filterCoeffSet[ind][i] ) )
          continue;
        bitsVarBin[ind] += lengthGolomb( abs( m_filterClippSet[ind][i] ), m_kMinTab[alfShape.golombIdx[i]], false );
      }
    }
  }

  double distForce0 = getDistCoeffForce0( codedVarBins, errorTabForce0Coeff, bitsVarBin, numFilters );

  return distForce0;
}

int EncAdaptiveLoopFilter::getGolombKMin( AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB] )
{
  int kStart;
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );

  int minBitsKStart = MAX_INT;
  int minKStart = -1;

  for( int k = 1; k < 8; k++ )
  {
    int bitsKStart = 0; kStart = k;
    for( int scanPos = 0; scanPos < maxGolombIdx; scanPos++ )
    {
      int kMin = kStart;
      int minBits = bitsCoeffScan[scanPos][kMin];

      if( bitsCoeffScan[scanPos][kStart + 1] < minBits )
      {
        kMin = kStart + 1;
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if( bitsKStart < minBitsKStart )
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for( int scanPos = 0; scanPos < maxGolombIdx; scanPos++ )
  {
    int kMin = kStart;
    int minBits = bitsCoeffScan[scanPos][kMin];

    if( bitsCoeffScan[scanPos][kStart + 1] < minBits )
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  return minKStart;
}

double EncAdaptiveLoopFilter::getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters )
{
  double distForce0 = 0;
  std::memset( codedVarBins, 0, sizeof( *codedVarBins ) * MAX_NUM_ALF_CLASSES );

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    double costDiff = errorForce0CoeffTab[filtIdx][0] - ( errorForce0CoeffTab[filtIdx][1] + m_lambda[COMPONENT_Y] * bitsVarBin[filtIdx] );
    codedVarBins[filtIdx] = costDiff > 0 ? true : false;
    distForce0 += errorForce0CoeffTab[filtIdx][codedVarBins[filtIdx] ? 1 : 0];
  }

  return distForce0;
}

int EncAdaptiveLoopFilter::lengthUvlc( int uiCode )
{
  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  return ( uiLength >> 1 ) + ( ( uiLength + 1 ) >> 1 );
}

int EncAdaptiveLoopFilter::lengthGolomb( int coeffVal, int k, bool signed_coeff )
{
  int m = 2 << ( k - 1 );
  int q = coeffVal / m;
  if( signed_coeff && coeffVal != 0 )
  {
    return q + 2 + k;
  }
  else
  {
    return q + 1 + k;
  }
}

double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfSliceParam& alfSliceParam )
{
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

  alfSliceParam.fixedFilterSetIndex = 0;
  AlfCovariance& tmpCovFf = covMerged[MAX_NUM_ALF_CLASSES + 1];
  double factor = 1 << (m_NUM_BITS - 1);
  double errorMin = 0;
  double errorMinPerClass[MAX_NUM_ALF_CLASSES] = { 0 };
  double errorCurSetPerClass[MAX_NUM_ALF_CLASSES] = { 0 };
  int    fixedFilterFlagPerClass[MAX_NUM_ALF_CLASSES] = { 0 };

  if (!alfSliceParam.nonLinearFlag[CHANNEL_TYPE_LUMA])
  {
    alfSliceParam.fixedFilterSetIndex = 0;
    for (int filterSetIdx = 0; filterSetIdx < NUM_FIXED_FILTER_SETS; filterSetIdx++)
    {
      double errorCur = 0;
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
        int fixedFilterIdx = m_classToFilterMapping[filterSetIdx][classIdx];
        errorCurSetPerClass[classIdx] = cov[classIdx].calcErrorForCoeffs(clipMerged[numFilters - 1][filterIndices[classIdx]], m_fixedFilterSetCoeff[fixedFilterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);

        if (errorCurSetPerClass[classIdx] >= 0)
        {
          errorCurSetPerClass[classIdx] = 0;
          fixedFilterFlagPerClass[classIdx] = 0;
        }
        else
        {
          errorCur += errorCurSetPerClass[classIdx];
          fixedFilterFlagPerClass[classIdx] = 1;
        }
      }

      if (errorCur < errorMin)
      {
        memcpy(alfSliceParam.fixedFilterIdx, fixedFilterFlagPerClass, sizeof(fixedFilterFlagPerClass));
        alfSliceParam.fixedFilterSetIndex = filterSetIdx + 1;
        errorMin = errorCur;
        std::memcpy(errorMinPerClass, errorCurSetPerClass, sizeof(errorMinPerClass));
      }
    }

    alfSliceParam.fixedFilterPattern = 0;
    if (alfSliceParam.fixedFilterSetIndex > 0)
    {
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
        if (alfSliceParam.fixedFilterIdx[classIdx] == 0)
        {
          alfSliceParam.fixedFilterPattern = 1;
          break;
        }
      }
    }
  }

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    tmpCov.reset();
    bool found_clip = false;
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      if( filterIndices[classIdx] == filtIdx )
      {
        //adjust stat
        tmpCovFf = cov[classIdx];
        if (alfSliceParam.fixedFilterSetIndex > 0 && alfSliceParam.fixedFilterIdx[classIdx] > 0
          && alfSliceParam.nonLinearFlag[CHANNEL_TYPE_LUMA] == false
          )
        {
          int fixedFilterIdx = m_classToFilterMapping[alfSliceParam.fixedFilterSetIndex - 1][classIdx];
          tmpCovFf.pixAcc += errorMinPerClass[classIdx];
          for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
          {
            double sum = 0;
            for (int j = 0; j < MAX_NUM_ALF_LUMA_COEFF; j++)
            {
              sum += tmpCovFf.E[clipMerged[numFilters - 1][classIdx][i]][clipMerged[numFilters - 1][classIdx][j]][i][j] * m_fixedFilterSetCoeff[fixedFilterIdx][j];
            }
            sum /= factor;
            tmpCovFf.y[clipMerged[numFilters - 1][classIdx][i]][i] -= sum;
          }
        }
        tmpCov += tmpCovFf;
        if( !found_clip )
        {
          found_clip = true; // clip should be at the adress of shortest one
          memcpy(m_filterClippSet[filtIdx], clipMerged[numFilters-1][classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
        }
      }
    }

    // Find coeffcients
    assert(alfShape.numCoeff == tmpCov.numCoeff);
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, false );
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];
  }
  return error;
}

double EncAdaptiveLoopFilter::deriveCoeffQuant( int *filterClipp, int *filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip )
{
  const int factor = 1 << ( bitDepth - 1 );
  const int max_value = factor - 1;
  const int min_value = -factor + 1;

const int numCoeff = shape.numCoeff;
  static double filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip );
  roundFiltCoeff( filterCoeffQuant, filterCoeff, numCoeff, factor );

  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    filterCoeffQuant[i] = std::min( max_value, std::max( min_value, filterCoeffQuant[i] ) );
  }
  filterCoeffQuant[numCoeff - 1] = 0;

  int modified=1;

  double errRef=cov.calcErrorForCoeffs( filterClipp, filterCoeffQuant, numCoeff, bitDepth );
  while( modified )
  {
    modified=0;
    for( int sign: {1, -1} )
    {
      double errMin = MAX_DOUBLE;
      int minInd = -1;

      for( int k = 0; k < numCoeff-1; k++ )
      {
        if( filterCoeffQuant[k] - sign > max_value || filterCoeffQuant[k] - sign < min_value )
          continue;

        filterCoeffQuant[k] -= sign;

        double error = cov.calcErrorForCoeffs( filterClipp, filterCoeffQuant, numCoeff, bitDepth );
        if( error < errMin )
        {
          errMin = error;
          minInd = k;
        }
        filterCoeffQuant[k] += sign;
      }
      if( errMin < errRef )
      {
        filterCoeffQuant[minInd] -= sign;
        modified++;
        errRef = errMin;
      }
    }
  }

  return errRef;
}

void EncAdaptiveLoopFilter::roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor )
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    filterCoeffQuant[i] = int( filterCoeff[i] * sign * factor + 0.5 ) * sign;
  }
}

void EncAdaptiveLoopFilter::mergeClasses( const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] )
{
  static int tmpClip[MAX_NUM_ALF_LUMA_COEFF];
  static int bestMergeClip[MAX_NUM_ALF_LUMA_COEFF];
  static double err[MAX_NUM_ALF_CLASSES];
  static double bestMergeErr;
  static bool availableClass[MAX_NUM_ALF_CLASSES];
  static uint8_t indexList[MAX_NUM_ALF_CLASSES];
  static uint8_t indexListTemp[MAX_NUM_ALF_CLASSES];
  int numRemaining = numClasses;

  memset( filterIndices, 0, sizeof( short ) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES );

  for( int i = 0; i < numClasses; i++ )
  {
    filterIndices[numRemaining - 1][i] = i;
    indexList[i] = i;
    availableClass[i] = true;
    covMerged[i] = cov[i];
    covMerged[i].numBins = m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[COMPONENT_Y] : 1;
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  tmpCov.numBins = m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[COMPONENT_Y] : 1;

  // init Clip
  for( int i = 0; i < numClasses; i++ )
  {
    std::fill_n(clipMerged[numRemaining-1][i], MAX_NUM_ALF_LUMA_COEFF, m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    if ( m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
    {
      err[i] = covMerged[i].optimizeFilterClip( alfShape, clipMerged[numRemaining-1][i] );
    }
    else
    {
      err[i] = covMerged[i].calculateError( clipMerged[numRemaining-1][i] );
    }
  }

  while( numRemaining >= 2 )
  {
    double errorMin = std::numeric_limits<double>::max();
    int bestToMergeIdx1 = 0, bestToMergeIdx2 = 1;

    for( int i = 0; i < numClasses - 1; i++ )
    {
      if( availableClass[i] )
      {
        for( int j = i + 1; j < numClasses; j++ )
        {
          if( availableClass[j] )
          {
            double error1 = err[i];
            double error2 = err[j];

            tmpCov.add( covMerged[i], covMerged[j] );
            for( int l = 0; l < MAX_NUM_ALF_LUMA_COEFF; ++l )
            {
              tmpClip[l] = (clipMerged[numRemaining-1][i][l] + clipMerged[numRemaining-1][j][l] + 1 ) >> 1;
            }
            double errorMerged = m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? tmpCov.optimizeFilterClip( alfShape, tmpClip ) : tmpCov.calculateError( tmpClip );
            double error = errorMerged - error1 - error2;

            if( error < errorMin )
            {
              bestMergeErr = errorMerged;
              memcpy(bestMergeClip, tmpClip, sizeof(bestMergeClip));
              errorMin = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
    memcpy(clipMerged[numRemaining-2], clipMerged[numRemaining-1], sizeof(int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF]));
    memcpy(clipMerged[numRemaining-2][bestToMergeIdx1], bestMergeClip, sizeof(bestMergeClip));
    err[bestToMergeIdx1] = bestMergeErr;
    availableClass[bestToMergeIdx2] = false;

    for( int i = 0; i < numClasses; i++ )
    {
      if( indexList[i] == bestToMergeIdx2 )
      {
        indexList[i] = bestToMergeIdx1;
      }
    }

    numRemaining--;
    if( numRemaining <= numClasses )
    {
      std::memcpy( indexListTemp, indexList, sizeof( uint8_t ) * numClasses );

      bool exist = false;
      int ind = 0;

      for( int j = 0; j < numClasses; j++ )
      {
        exist = false;
        for( int i = 0; i < numClasses; i++ )
        {
          if( indexListTemp[i] == j )
          {
            exist = true;
            break;
          }
        }

        if( exist )
        {
          for( int i = 0; i < numClasses; i++ )
          {
            if( indexListTemp[i] == j )
            {
              filterIndices[numRemaining - 1][i] = ind;
              indexListTemp[i] = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::getFrameStats( ChannelType channel, int iShapeIdx )
{
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  for( int i = 0; i < numClasses; i++ )
  {
    m_alfCovarianceFrame[channel][iShapeIdx][i].reset(AlfNumClippingValues[channel]);
  }
  if( isLuma( channel ) )
  {
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], numClasses );
  }
  else
  {
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], numClasses );
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], numClasses );
  }
}

void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses )
{
  for( int i = 0; i < m_numCTUsInPic; i++ )
  {
    if( ctbEnableFlags[i] )
    {
      for( int j = 0; j < numClasses; j++ )
      {
        frameCov[j] += ctbCov[i][j];
      }
    }
  }
}

void EncAdaptiveLoopFilter::deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv, CodingStructure& cs )
{
  int ctuRsAddr = 0;
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );

  // init CTU stats buffers
  for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int shape = 0; shape != m_filterShapes[toChannelType( compID )].size(); shape++ )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
        {
          m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset(AlfNumClippingValues[toChannelType( compID )]);
        }
      }
    }
  }

  // init Frame stats buffers
  const int numberOfChannels = getNumberValidChannels( m_chromaFormat );
  for( int channelIdx = 0; channelIdx < numberOfChannels; channelIdx++ )
  {
    const ChannelType channelID = ChannelType( channelIdx );
    const int numClasses = isLuma( channelID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int shape = 0; shape != m_filterShapes[channelIdx].size(); shape++ )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset(AlfNumClippingValues[channelID]);
      }
    }
  }

  const PreCalcValues& pcv = *cs.pcv;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
      const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
      if( isCrossedByVirtualBoundaries( xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS() ) )
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );

          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );

            const int wBuf = w + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
            const int hBuf = h + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
            PelUnitBuf recBuf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
            recBuf.copyFrom( recYuv.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf ) ) ) );
            recBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
            recBuf = recBuf.subBuf( UnitArea ( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

            const UnitArea area( m_chromaFormat, Area( 0, 0, w, h ) );
            const UnitArea areaDst( m_chromaFormat, Area( xStart, yStart, w, h ) );
            for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
            {
              const ComponentID compID = ComponentID( compIdx );
              const CompArea& compArea = area.block( compID );

              int  recStride = recBuf.get( compID ).stride;
              Pel* rec = recBuf.get( compID ).bufAt( compArea );

              int  orgStride = orgYuv.get(compID).stride;
              Pel* org = orgYuv.get(compID).bufAt(xStart >> ::getComponentScaleX(compID, m_chromaFormat), yStart >> ::getComponentScaleY(compID, m_chromaFormat));

              ChannelType chType = toChannelType( compID );

              for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
              {
              const CompArea& compAreaDst = areaDst.block( compID );
                getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compAreaDst, compArea, chType
                  , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
                  , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))
                );
              }
            }

            xStart = xEnd;
          }

          yStart = yEnd;
        }

        for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
        {
          const ComponentID compID = ComponentID( compIdx );

          ChannelType chType = toChannelType( compID );

          for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
          {
            const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

            for( int classIdx = 0; classIdx < numClasses; classIdx++ )
            {
              m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
            }
          }
        }
      }
      else
      {
      const UnitArea area( m_chromaFormat, Area( xPos, yPos, width, height ) );

      for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
      {
        const ComponentID compID = ComponentID( compIdx );
        const CompArea& compArea = area.block( compID );

        int  recStride = recYuv.get( compID ).stride;
        Pel* rec = recYuv.get( compID ).bufAt( compArea );

        int  orgStride = orgYuv.get( compID ).stride;
        Pel* org = orgYuv.get( compID ).bufAt( compArea );

        ChannelType chType = toChannelType( compID );

        for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
        {
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, compArea, chType
            , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
            , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))        
          );


          const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

          for( int classIdx = 0; classIdx < numClasses; classIdx++ )
          {
            m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
          }
        }
      }
      }
      ctuRsAddr++;
    }
  }
}

void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos)



{
  static int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues];

  const int numBins = AlfNumClippingValues[channel];

  int transposeIdx = 0;
  int classIdx = 0;

  for( int i = 0; i < area.height; i++ )
  {
    int vbDistance = ((areaDst.y + i) % vbCTUHeight) - vbPos;
    for( int j = 0; j < area.width; j++ )
    {
      if( classifier && classifier[areaDst.y + i][areaDst.x + j].classIdx == m_ALF_UNUSED_CLASSIDX && classifier[areaDst.y + i][areaDst.x + j].transposeIdx == m_ALF_UNUSED_TRANSPOSIDX )
      {
        continue;
      }
      std::memset( ELocal, 0, sizeof( ELocal ) );
      if( classifier )
      {
        AlfClassifier& cl = classifier[areaDst.y + i][areaDst.x + j];
        transposeIdx = cl.transposeIdx;
        classIdx = cl.classIdx;
      }

      double weight = 1.0;
      if (m_alfWSSD)
      {
        weight = m_lumaLevelToWeightPLUT[org[j]];
      }
      int yLocal = org[j] - rec[j];
      calcCovariance(ELocal, rec + j, recStride, shape, transposeIdx, channel, vbDistance);
      for( int k = 0; k < shape.numCoeff; k++ )
      {
        for( int l = k; l < shape.numCoeff; l++ )
        {
          for( int b0 = 0; b0 < numBins; b0++ )
          {
            for( int b1 = 0; b1 < numBins; b1++ )
            {
              if (m_alfWSSD)
              {
                alfCovariance[classIdx].E[b0][b1][k][l] += weight * (double)(ELocal[k][b0] * ELocal[l][b1]);
              }
              else
              {
                alfCovariance[classIdx].E[b0][b1][k][l] += ELocal[k][b0] * ELocal[l][b1];
              }
            }
          }
        }
        for( int b = 0; b < numBins; b++ )
        {
          if (m_alfWSSD)
          {
            alfCovariance[classIdx].y[b][k] += weight * (double)(ELocal[k][b] * yLocal);
          }
          else
          {
            alfCovariance[classIdx].y[b][k] += ELocal[k][b] * yLocal;
          }
        }
      }
      if (m_alfWSSD)
      {
        alfCovariance[classIdx].pixAcc += weight * (double)(yLocal * yLocal);
      }
      else
      {
        alfCovariance[classIdx].pixAcc += yLocal * yLocal;
      }
    }
    org += orgStride;
    rec += recStride;
  }

  int numClasses = classifier ? MAX_NUM_ALF_CLASSES : 1;
  for( classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    for( int k = 1; k < shape.numCoeff; k++ )
    {
      for( int l = 0; l < k; l++ )
      {
        for( int b0 = 0; b0 < numBins; b0++ )
        {
          for( int b1 = 0; b1 < numBins; b1++ )
          {
            alfCovariance[classIdx].E[b0][b1][k][l] = alfCovariance[classIdx].E[b1][b0][l][k];
          }
        }
      }
    }
  }
}
void EncAdaptiveLoopFilter::calcCovariance(int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance)

{
  int clipTopRow = -4;
  int clipBotRow = 4;
  if (vbDistance >= -3 && vbDistance < 0)
  {
    clipBotRow = -vbDistance - 1;
    clipTopRow = -clipBotRow; // symmetric
  }
  else if (vbDistance >= 0 && vbDistance < 3)
  {
    clipTopRow = -vbDistance;
    clipBotRow = -clipTopRow; // symmetric
  }

  const int *filterPattern = shape.pattern.data();
  const int halfFilterLength = shape.filterLength >> 1;
  const Pel* clip = m_alfClippingValues[channel];
  const int numBins = AlfNumClippingValues[channel];

  int k = 0;

  const short curr = rec[0];

  if( transposeIdx == 0 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;
      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++ )
      {
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
        }
    }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
      }
    }
  }
  else if( transposeIdx == 2 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;

      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++ )
      {
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for (int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
        }
      }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
      }
    }

  }
  for( int b = 0; b < numBins; b++ )
  {
    ELocal[filterPattern[k]][b] += curr;
  }
}



void EncAdaptiveLoopFilter::setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, bool val )
{
  if( channel == CHANNEL_TYPE_LUMA )
  {
    alfSlicePara.enabledFlag[COMPONENT_Y] = val;
  }
  else
  {
    alfSlicePara.enabledFlag[COMPONENT_Cb] = alfSlicePara.enabledFlag[COMPONENT_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags )
{
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
  for( int compId = compIDFirst; compId <= compIDLast; compId++ )
  {
    alfSlicePara.enabledFlag[compId] = false;
    for( int i = 0; i < m_numCTUsInPic; i++ )
    {
      if( ctuFlags[compId][i] )
      {
        alfSlicePara.enabledFlag[compId] = true;
        break;
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( ctuFlagsDst[COMPONENT_Y], ctuFlagsSrc[COMPONENT_Y], sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memcpy( ctuFlagsDst[COMPONENT_Cb], ctuFlagsSrc[COMPONENT_Cb], sizeof( uint8_t ) * m_numCTUsInPic );
    memcpy( ctuFlagsDst[COMPONENT_Cr], ctuFlagsSrc[COMPONENT_Cr], sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val )
{
  if( isLuma( channel ) )
  {
    memset( ctuFlags[COMPONENT_Y], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memset( ctuFlags[COMPONENT_Cb], val, sizeof( uint8_t ) * m_numCTUsInPic );
    memset( ctuFlags[COMPONENT_Cr], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

std::vector<int> EncAdaptiveLoopFilter::getAvaiApsIdsLuma(CodingStructure& cs, int &newApsId)
{
  APS** apss = cs.slice->getAlfAPSs();
  for (int i = 0; i < MAX_NUM_APS; i++)
  {
    apss[i] = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
  }

  std::vector<int> result;
  int apsIdChecked = 0, curApsId = m_apsIdStart;
  if (curApsId < int(MAX_NUM_APS))
  {
    while (apsIdChecked < MAX_NUM_APS && !cs.slice->isIntra() && result.size() < (ALF_CTB_MAX_NUM_APS - 1) && !cs.slice->getPendingRasInit() && !cs.slice->isIDRorBLA())
    {
      APS* curAPS = cs.slice->getAlfAPSs()[curApsId];
      if (curAPS && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA])
      {
        result.push_back(curApsId);
      }
      apsIdChecked++;
      curApsId = (curApsId + 1) % MAX_NUM_APS;
    }
  }
  cs.slice->setTileGroupNumAps((int)result.size());
  cs.slice->setAlfAPSs(result);
  newApsId = m_apsIdStart - 1;
  if (newApsId < 0)
  {
    newApsId = (int)MAX_NUM_APS - 1;
  }

  CHECK(newApsId >= (int)MAX_NUM_APS, "Wrong APS index assignment in getAvaiApsIdsLuma");
  return result;
}
void  EncAdaptiveLoopFilter::initDistortion()
{
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      m_ctbDistortionUnfilter[comp][ctbIdx] = getUnfilteredDistortion(m_alfCovariance[comp][0][ctbIdx], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);
    }
  }
}
void  EncAdaptiveLoopFilter::alfEncoderCtb(CodingStructure& cs, AlfSliceParam& alfSliceParamNewFilters
#if ENABLE_QPA
  , const double lambdaChromaWeight
#endif
)
{
  TempCtx        ctxStart(m_CtxCache, AlfCtx(m_CABACEstimator->getCtx()));
  TempCtx        ctxBest(m_CtxCache);
  TempCtx        ctxTempStart(m_CtxCache);
  TempCtx        ctxTempBest(m_CtxCache);
  AlfSliceParam  alfSliceParamNewFiltersBest = alfSliceParamNewFilters;
  APS**          apss = cs.slice->getAlfAPSs();
  short*     alfCtbFilterSetIndex = cs.picture->getAlfCtbFilterIndex();
  bool     hasNewFilters[2] = { alfSliceParamNewFilters.enabledFlag[COMPONENT_Y] , alfSliceParamNewFilters.enabledFlag[COMPONENT_Cb] || alfSliceParamNewFilters.enabledFlag[COMPONENT_Cr] };
  initDistortion();

  //luma
  setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_LUMA, 1);
  getFrameStats(CHANNEL_TYPE_LUMA, 0);
  setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_LUMA, 0);
  double costOff = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][0], CHANNEL_TYPE_LUMA);

  int newApsId;
  std::vector<int> apsIds = getAvaiApsIdsLuma(cs, newApsId);
  std::vector<int> bestApsIds;
  double costMin = MAX_DOUBLE;
  reconstructCoeffAPSs(cs, true, false, true);

  int numLoops = hasNewFilters[CHANNEL_TYPE_LUMA] ? 2 : 1;
  for (int useNewFilter = 0; useNewFilter < numLoops; useNewFilter++)
  {
    int bitsNewFilter = 0;
    if (useNewFilter == 1)
    {
      if (!hasNewFilters[CHANNEL_TYPE_LUMA])
      {
        continue;
      }
      else
      {
        bitsNewFilter = m_bitsNewFilter[CHANNEL_TYPE_LUMA];
        reconstructCoeff(alfSliceParamNewFilters, CHANNEL_TYPE_LUMA, true, true);
      }
    }
    int numIter = useNewFilter ? 2 : 1;
    for (int numTemporalAps = 0; numTemporalAps <= apsIds.size(); numTemporalAps++)
    {
      cs.slice->setTileGroupNumAps(numTemporalAps + useNewFilter);
      int numFilterSet = NUM_FIXED_FILTER_SETS + numTemporalAps + useNewFilter;
      if (numTemporalAps == apsIds.size() && numTemporalAps > 0 && useNewFilter && newApsId == apsIds.back()) //last temporalAPS is occupied by new filter set and this temporal APS becomes unavailable
      {
        continue;
      }
      for (int iter = 0; iter < numIter; iter++)
      {
        m_alfSliceParamTemp = alfSliceParamNewFilters;
        m_alfSliceParamTemp.enabledFlag[CHANNEL_TYPE_LUMA] = true;
        double curCost = getTBlength(numTemporalAps + useNewFilter, ALF_CTB_MAX_NUM_APS + 1) * m_lambda[CHANNEL_TYPE_LUMA];
        if (iter > 0)  //re-derive new filter-set
        {
          double dDistOrgNewFilter = 0;
          int blocksUsingNewFilter = 0;
          for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
          {
            if (m_ctuEnableFlag[COMPONENT_Y][ctbIdx] && alfCtbFilterSetIndex[ctbIdx] != NUM_FIXED_FILTER_SETS)
            {
              m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 0;
            }
            else if (m_ctuEnableFlag[COMPONENT_Y][ctbIdx] && alfCtbFilterSetIndex[ctbIdx] == NUM_FIXED_FILTER_SETS)
            {
              blocksUsingNewFilter++;
              dDistOrgNewFilter += m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
              for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
              {
                short* pCoeff = m_coeffFinal;
                short* pClipp = m_clippFinal;
                for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                {
                  m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                  m_clipTmp[i] = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                }
                dDistOrgNewFilter += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
              }
            }
          }
          if (blocksUsingNewFilter > 0 && blocksUsingNewFilter < m_numCTUsInPic)
          {
            int bitNL[2] = { 0, 0 };
            double errNL[2] = { 0.0, 0.0 };
            m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 1;
            if (m_encCfg->getUseNonLinearAlfLuma())
            {
              errNL[1] = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, 0, bitNL[1], true);
              m_alfSliceParamTempNL = m_alfSliceParamTemp;
            }
            else
            {
              errNL[1] = MAX_DOUBLE;
            }
            m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 0;
            errNL[0] = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, 0, bitNL[0], true);

            int bitsNewFilterTempLuma = bitNL[0];
            double err = errNL[0];
            if (errNL[1]  < errNL[0])
            {
              err = errNL[1];
              bitsNewFilterTempLuma = bitNL[1];
              m_alfSliceParamTemp = m_alfSliceParamTempNL;
            }
            if (dDistOrgNewFilter + m_lambda[CHANNEL_TYPE_LUMA] * m_bitsNewFilter[CHANNEL_TYPE_LUMA] < err) //re-derived filter is not good, skip
            {
              continue;
            }
            reconstructCoeff(m_alfSliceParamTemp, CHANNEL_TYPE_LUMA, true, true);
            bitsNewFilter = bitsNewFilterTempLuma;
          }
          else //no blocks using new filter, skip
          {
            continue;
          }
        }

        m_CABACEstimator->getCtx() = ctxStart;
        for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
        {
          double distUnfilterCtb = m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
          //ctb on
          m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 1;
          double         costOn = MAX_DOUBLE;
          ctxTempStart = AlfCtx(m_CABACEstimator->getCtx());
          int iBestFilterSetIdx = 0;
          for (int filterSetIdx = 0; filterSetIdx < numFilterSet; filterSetIdx++)
          {
            //rate
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
            m_CABACEstimator->resetBits();
            m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMPONENT_Y, &m_alfSliceParamTemp);
            alfCtbFilterSetIndex[ctbIdx] = filterSetIdx;
            m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctbIdx, &m_alfSliceParamTemp.enabledFlag[COMPONENT_Y]);
            double rateOn = FracBitsScale *(double)m_CABACEstimator->getEstFracBits();
            //distortion
            double dist = distUnfilterCtb;
            for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
              if (filterSetIdx < NUM_FIXED_FILTER_SETS)
              {
                int filterIdx = m_classToFilterMapping[filterSetIdx][classIdx];
                dist += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipDefaultEnc, m_fixedFilterSetCoeff[filterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
              }
              else
              {
                short *pCoeff;
                short *pClipp;
                if (useNewFilter && filterSetIdx == NUM_FIXED_FILTER_SETS)
                {
                  pCoeff = m_coeffFinal;
                  pClipp = m_clippFinal;
                }
                else if (useNewFilter)
                {
                  pCoeff = m_coeffApsLuma[filterSetIdx - 1 - NUM_FIXED_FILTER_SETS];
                  pClipp = m_clippApsLuma[filterSetIdx - 1 - NUM_FIXED_FILTER_SETS];
                }
                else
                {
                  pCoeff = m_coeffApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS];
                  pClipp = m_clippApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS];
                }
                for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                {
                  m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                  m_clipTmp[i] = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                }
                dist += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
              }
            }
            //cost
            double costOnTmp = dist + m_lambda[COMPONENT_Y] * rateOn;
            if (costOnTmp < costOn)
            {
              ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
              costOn = costOnTmp;
              iBestFilterSetIdx = filterSetIdx;
            }
          }
          //ctb off
          m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 0;
          //rate
          m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
          m_CABACEstimator->resetBits();
          m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMPONENT_Y, &m_alfSliceParamTemp);
          //cost
          double costOff = distUnfilterCtb + m_lambda[COMPONENT_Y] * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
          if (costOn < costOff)
          {
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
            m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 1;
            alfCtbFilterSetIndex[ctbIdx] = iBestFilterSetIdx;
            curCost += costOn;
          }
          else
          {
            m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 0;
            curCost += costOff;
          }
        } //for(ctbIdx)
        int tmpBits = bitsNewFilter + 5 * (numFilterSet - NUM_FIXED_FILTER_SETS) + (cs.slice->isIntra() ? 1 : getTBlength(numFilterSet - NUM_FIXED_FILTER_SETS, ALF_CTB_MAX_NUM_APS + 1));
        curCost += tmpBits * m_lambda[COMPONENT_Y];
        if (curCost < costMin)
        {
          costMin = curCost;
          bestApsIds.resize(numFilterSet - NUM_FIXED_FILTER_SETS);
          for (int i = 0; i < bestApsIds.size(); i++)
          {
            if (i == 0 && useNewFilter)
            {
              bestApsIds[i] = newApsId;
            }
            else
            {
              bestApsIds[i] = apsIds[i - useNewFilter];
            }
          }
          alfSliceParamNewFiltersBest = m_alfSliceParamTemp;
          ctxBest = AlfCtx(m_CABACEstimator->getCtx());
          copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
          for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
          {
            m_alfCtbFilterSetIndexTmp[ctuIdx] = alfCtbFilterSetIndex[ctuIdx];
          }
          alfSliceParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] = useNewFilter;
        }
      }//for (int iter = 0; iter < numIter; iter++)
    }// for (int numTemporalAps = 0; numTemporalAps < apsIds.size(); numTemporalAps++)
  }//for (int useNewFilter = 0; useNewFilter <= 1; useNewFilter++)

  if (costOff <= costMin)
  {
    cs.slice->resetTileGroupAlfEnabledFlag();
    cs.slice->setTileGroupNumAps(0);
    setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_LUMA, 0);
    setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, 0);
    return;
  }
  else
  {
    alfSliceParamNewFiltersBest.tLayer = cs.slice->getTLayer();
    cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Y, true);
    cs.slice->setTileGroupNumAps((int)bestApsIds.size());
    cs.slice->setAlfAPSs(bestApsIds);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
    for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
    {
      alfCtbFilterSetIndex[ctuIdx] = m_alfCtbFilterSetIndexTmp[ctuIdx];
    }
    if (alfSliceParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA])
    {
      APS* newAPS = m_apsMap->getPS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
        newAPS->setAPSId(newApsId);
        newAPS->setAPSType(ALF_APS);
      }
      newAPS->setAlfAPSParam(alfSliceParamNewFiltersBest);
      m_apsMap->setChangedFlag((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsIdStart = newApsId;
    }

    std::vector<int> apsIds = cs.slice->getTileGroupApsIdLuma();
    for (int i = 0; i < (int)cs.slice->getTileGroupNumAps(); i++)
    {
      apss[apsIds[i]] = m_apsMap->getPS((apsIds[i] << NUM_APS_TYPE_LEN) + ALF_APS);
    }
  }

  //chroma
  setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, 1);
  getFrameStats(CHANNEL_TYPE_CHROMA, 0);
  costOff = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA) + m_lambda[CHANNEL_TYPE_CHROMA] * 1.0;
  costMin = MAX_DOUBLE;
  m_CABACEstimator->getCtx() = AlfCtx(ctxBest);
  ctxStart = AlfCtx(m_CABACEstimator->getCtx());
  int newApsIdChroma = -1;
  if (alfSliceParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] && (alfSliceParamNewFiltersBest.enabledFlag[COMPONENT_Cb] || alfSliceParamNewFiltersBest.enabledFlag[COMPONENT_Cr]))
  {
    newApsIdChroma = newApsId;
  }
  else if (alfSliceParamNewFiltersBest.enabledFlag[COMPONENT_Cb] || alfSliceParamNewFiltersBest.enabledFlag[COMPONENT_Cr])
  {
    int curId = m_apsIdStart;
    while (newApsIdChroma < 0)
    {
      curId--;
      if (curId < 0)
      {
        curId = (int)MAX_NUM_APS - 1;
      }
      if (std::find(bestApsIds.begin(), bestApsIds.end(), curId) == bestApsIds.end())
      {
        newApsIdChroma = curId;
      }
    }
  }
  for (int curApsId = 0; curApsId < MAX_NUM_APS; curApsId++)
  {
    if ((cs.slice->getPendingRasInit() || cs.slice->isIDRorBLA() || cs.slice->isIntra()) && curApsId != newApsIdChroma)
    {
      continue;
    }
    APS* curAPS = m_apsMap->getPS((curApsId << NUM_APS_TYPE_LEN) + ALF_APS);
    double curCost = (cs.slice->isIntra() && cs.slice->getTileGroupNumAps() == 1) ? 0 : (m_lambda[CHANNEL_TYPE_CHROMA] * 5);
    if (curApsId == newApsIdChroma)
    {
      m_alfSliceParamTemp = alfSliceParamNewFilters;
      curCost += m_lambda[CHANNEL_TYPE_CHROMA] * m_bitsNewFilter[CHANNEL_TYPE_CHROMA];
    }
    else if (curAPS && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA])
    {
      m_alfSliceParamTemp = curAPS->getAlfAPSParam();
    }
    else
    {
      continue;
    }
    reconstructCoeff(m_alfSliceParamTemp, CHANNEL_TYPE_CHROMA, true, true);
    m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
    for (int compId = 1; compId < MAX_NUM_COMPONENT; compId++)
    {
      m_alfSliceParamTemp.enabledFlag[compId] = true;
      for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
      {
        double distUnfilterCtu = m_ctbDistortionUnfilter[compId][ctbIdx];
        //cost on
        m_ctuEnableFlag[compId][ctbIdx] = 1;
        ctxTempStart = AlfCtx(m_CABACEstimator->getCtx());
        //rate
        m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
        m_CABACEstimator->resetBits();
        //ctb flag
        m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, compId, &m_alfSliceParamTemp);
        double rateOn = FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
        //distortion
        for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
        {
          m_filterTmp[i] = m_chromaCoeffFinal[i];
          m_clipTmp[i] = m_chromaClippFinal[i];
        }
        double dist = distUnfilterCtu + m_alfCovariance[compId][0][ctbIdx][0].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, m_NUM_BITS);
        double costOn = dist + m_lambda[compId] * rateOn;
        ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
        //cost off
        m_ctuEnableFlag[compId][ctbIdx] = 0;
        //rate
        m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
        m_CABACEstimator->resetBits();
        m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, compId, &m_alfSliceParamTemp);
        //cost
        double costOff = distUnfilterCtu + m_lambda[compId] * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
        if (costOn < costOff)
        {
          m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
          m_ctuEnableFlag[compId][ctbIdx] = 1;
          curCost += costOn;
        }
        else
        {
          m_ctuEnableFlag[compId][ctbIdx] = 0;
          curCost += costOff;
        }
      }
    }
    //chroma idc
    setEnableFlag(m_alfSliceParamTemp, CHANNEL_TYPE_CHROMA, m_ctuEnableFlag);
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    curCost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[CHANNEL_TYPE_CHROMA];

    if (curCost < costMin)
    {
      costMin = curCost;
      cs.slice->setTileGroupApsIdChroma(curApsId);
      cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Cb, m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb]);
      cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Cr, m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr]);
      copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);
    }
  }
  if (costOff < costMin)
  {
    cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Cb, false);
    cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Cr, false);
    setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, 0);
  }
  else
  {
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_CHROMA);
    if (cs.slice->getTileGroupApsIdChroma() == newApsIdChroma)  //new filter
    {
      APS* newAPS = m_apsMap->getPS((newApsIdChroma << NUM_APS_TYPE_LEN) + ALF_APS);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS((newApsIdChroma << NUM_APS_TYPE_LEN) + ALF_APS);
        newAPS->setAPSType(ALF_APS);
        newAPS->setAPSId(newApsIdChroma);
        newAPS->getAlfAPSParam().reset();
      }
      newAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] = true;
      newAPS->getAlfAPSParam().nonLinearFlag[CHANNEL_TYPE_CHROMA] = alfSliceParamNewFilters.nonLinearFlag[CHANNEL_TYPE_CHROMA];
      newAPS->getAlfAPSParam().tLayer = cs.slice->getTLayer();
      for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
      {
        newAPS->getAlfAPSParam().chromaCoeff[i] = alfSliceParamNewFilters.chromaCoeff[i];
        newAPS->getAlfAPSParam().chromaClipp[i] = alfSliceParamNewFilters.chromaClipp[i];
      }
      m_apsMap->setChangedFlag((newApsIdChroma << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsIdStart = newApsIdChroma;
    }
    apss[cs.slice->getTileGroupApsIdChroma()] = m_apsMap->getPS((cs.slice->getTileGroupApsIdChroma() << NUM_APS_TYPE_LEN) + ALF_APS);
  }
}

void EncAdaptiveLoopFilter::alfReconstructor(CodingStructure& cs, const PelUnitBuf& recExtBuf)
{
  if (!cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y))
  {
    return;
  }
  reconstructCoeffAPSs(cs, true, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) || cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr), false);
  short* alfCtuFilterIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  PelUnitBuf& recBuf = cs.getRecoBufRef();
  const PreCalcValues& pcv = *cs.pcv;

  int ctuIdx = 0;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const int width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

      bool ctuEnableFlag = m_ctuEnableFlag[COMPONENT_Y][ctuIdx];
      for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        ctuEnableFlag |= m_ctuEnableFlag[compIdx][ctuIdx] > 0;
      }
      if (ctuEnableFlag && isCrossedByVirtualBoundaries(xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS()))
      {
        int yStart = yPos;
        for (int i = 0; i <= numHorVirBndry; i++)
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = (i == 0 && clipTop) || (i > 0) || (yStart == 0);
          const bool clipB = (i == numHorVirBndry && clipBottom) || (i < numHorVirBndry ) || (yEnd == pcv.lumaHeight);

          int xStart = xPos;
          for (int j = 0; j <= numVerVirBndry; j++)
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = (j == 0 && clipLeft) || (j > 0) || (xStart == 0);
            const bool clipR = (j == numVerVirBndry && clipRight) || (j < numVerVirBndry ) || (xEnd == pcv.lumaWidth);

            const int wBuf = w + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
            const int hBuf = h + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
            PelUnitBuf buf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
            buf.copyFrom(recExtBuf.subBuf(UnitArea(cs.area.chromaFormat, Area(xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
            buf.extendBorderPel(MAX_ALF_PADDING_SIZE);
            buf = buf.subBuf(UnitArea(cs.area.chromaFormat, Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h)));

            if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
            {
              const Area blkSrc(0, 0, w, h);
              const Area blkDst(xStart, yStart, w, h);
              short filterSetIndex = alfCtuFilterIndex[ctuIdx];
              short *coeff;
              short *clip;
              if (filterSetIndex >= NUM_FIXED_FILTER_SETS)
              {
                coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
                clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
              }
              else
              {
                coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
                clip = m_clipDefault;
              }
              m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs
                , m_alfVBLumaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
              );
            }

            for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
            {
              ComponentID compID = ComponentID(compIdx);
              const int chromaScaleX = getComponentScaleX(compID, recBuf.chromaFormat);
              const int chromaScaleY = getComponentScaleY(compID, recBuf.chromaFormat);
              if (m_ctuEnableFlag[compIdx][ctuIdx])
              {
                const Area blkSrc(0, 0, w >> chromaScaleX, h >> chromaScaleY);
                const Area blkDst(xStart >> chromaScaleX, yStart >> chromaScaleY, w >> chromaScaleX, h >> chromaScaleY);
                m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs
                  , m_alfVBChmaCTUHeight
                  , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
                );
              }
            }
            
            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {

      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
      {
        Area blk(xPos, yPos, width, height);
        short filterSetIndex = alfCtuFilterIndex[ctuIdx];
        short *coeff;
        short *clip;
        if (filterSetIndex >= NUM_FIXED_FILTER_SETS)
        {
          coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
          clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
        }
        else
        {
          coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
          clip = m_clipDefault;
        }
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs
          , m_alfVBLumaCTUHeight
          , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
        );
      }

      for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        ComponentID compID = ComponentID(compIdx);
        const int chromaScaleX = getComponentScaleX(compID, recBuf.chromaFormat);
        const int chromaScaleY = getComponentScaleY(compID, recBuf.chromaFormat);
        if (m_ctuEnableFlag[compIdx][ctuIdx])
        {
          Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
            , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
          );
        }
      }
      }
      ctuIdx++;
    }
  }
}
