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

#if JVET_N0415_CTB_ALF
#define AlfCtx(c) SubCtx( Ctx::Alf, c)
#else
#define AlfCtx(c) SubCtx( Ctx::ctbAlfFlag, c )
#endif
std::vector<double> EncAdaptiveLoopFilter::m_lumaLevelToWeightPLUT;

#if JVET_N0242_NON_LINEAR_ALF
void AlfCovariance::getClipMax(const AlfFilterShape& alfShape, int *clip_max) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    clip_max[k] = 0;

    bool inc = true;
    while (clip_max[k]+1 < numBins && y[clip_max[k]+1][k] == y[clip_max[k]][k])
    {
      for (int l = 0; l < numCoeff; ++l)
        if (E[clip_max[k]][0][k][l] != E[clip_max[k]+1][0][k][l])
        {
          inc = false;
          break;
        }
      if (!inc)
      {
        break;
      }
      ++clip_max[k];
    }
  }
  clip_max[numCoeff-1] = 0;
}

void AlfCovariance::reduceClipCost(const AlfFilterShape& alfShape, int *clip) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    bool dec = true;
    while (clip[k] > 0 && y[clip[k]-1][k] == y[clip[k]][k])
    {
      for (int l=0; l<numCoeff; ++l)
        if (E[clip[k]][clip[l]][k][l] != E[clip[k]-1][clip[l]][k][l])
        {
          dec = false;
          break;
        }
      if (!dec)
      {
        break;
      }
      --clip[k];
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

#endif
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
#if !JVET_N0242_NON_LINEAR_ALF
  m_filterCoeffQuant = nullptr;
#endif
  m_filterCoeffSet = nullptr;
#if JVET_N0242_NON_LINEAR_ALF
  m_filterClippSet = nullptr;
#endif
  m_diffFilterCoeff = nullptr;

  m_alfWSSD = 0;
}

#if JVET_N0242_NON_LINEAR_ALF
void EncAdaptiveLoopFilter::create( const EncCfg* encCfg, const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] )
#else
void EncAdaptiveLoopFilter::create( const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] )
#endif
{
  AdaptiveLoopFilter::create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth );
#if JVET_N0242_NON_LINEAR_ALF
  CHECK( encCfg == nullptr, "encCfg must not be null" );
  m_encCfg = encCfg;
#endif

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
#if JVET_N0415_CTB_ALF
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
#else
    for( int j = 0; j <= MAX_NUM_ALF_CLASSES; j++ )
#endif
    {
      m_alfCovarianceMerged[i][j].create( m_filterShapes[COMPONENT_Y][i].numCoeff );
    }
  }

#if !JVET_N0242_NON_LINEAR_ALF
  m_filterCoeffQuant = new int[MAX_NUM_ALF_LUMA_COEFF];
#endif
  m_filterCoeffSet = new int*[MAX_NUM_ALF_CLASSES];
#if JVET_N0242_NON_LINEAR_ALF
  m_filterClippSet = new int*[MAX_NUM_ALF_CLASSES];
#endif
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];

  for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
#if JVET_N0242_NON_LINEAR_ALF
    m_filterClippSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
#endif
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }

#if JVET_N0415_CTB_ALF
  m_apsIdStart = (int)MAX_NUM_APS;
  m_ctbDistortionFixedFilter = new double[m_numCTUsInPic];
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    m_ctbDistortionUnfilter[comp] = new double[m_numCTUsInPic];
  }
  m_alfCtbFilterSetIndexTmp.resize(m_numCTUsInPic);
  memset(m_clipDefaultEnc, 0, sizeof(m_clipDefaultEnc));
#endif
}

void EncAdaptiveLoopFilter::destroy()
{
#if JVET_N0415_CTB_ALF
  if (!m_created)
  {
    return;
  }
#endif
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
#if JVET_N0415_CTB_ALF
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
#else
    for( int j = 0; j <= MAX_NUM_ALF_CLASSES; j++ )
#endif
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

#if JVET_N0242_NON_LINEAR_ALF
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

#endif
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

#if !JVET_N0242_NON_LINEAR_ALF
  delete[] m_filterCoeffQuant;
  m_filterCoeffQuant = nullptr;
#endif

#if JVET_N0415_CTB_ALF
  delete[] m_ctbDistortionFixedFilter;
  m_ctbDistortionFixedFilter = nullptr;
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    delete[] m_ctbDistortionUnfilter[comp];
    m_ctbDistortionUnfilter[comp] = nullptr;
  }
#endif
  AdaptiveLoopFilter::destroy();
}
#if JVET_N0415_CTB_ALF
void EncAdaptiveLoopFilter::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice
, ParameterSetMap<APS>* apsMap )
#else
void EncAdaptiveLoopFilter::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice )
#endif
{
#if JVET_N0415_CTB_ALF
  m_apsMap = apsMap;
#endif
  m_CABACEstimator = cabacEncoder->getCABACEstimator( pcSlice->getSPS() );
  m_CtxCache = ctxCache;
  m_CABACEstimator->initCtxModels( *pcSlice );
  m_CABACEstimator->resetBits();
}

#if JVET_N0415_CTB_ALF
void EncAdaptiveLoopFilter::ALFProcess(CodingStructure& cs, const double *lambdas
#if ENABLE_QPA
                                       , const double lambdaChromaWeight
#endif
                                      )
#else
void EncAdaptiveLoopFilter::ALFProcess( CodingStructure& cs, const double *lambdas,
#if ENABLE_QPA
                                        const double lambdaChromaWeight,
#endif
                                        AlfSliceParam& alfSliceParam )
#endif
{
#if JVET_N0415_CTB_ALF
  if (cs.slice->getPendingRasInit() || cs.slice->isIDRorBLA())
  {
    memset(cs.slice->getAPSs(), 0, sizeof(*cs.slice->getAPSs())*MAX_NUM_APS);
    m_apsMap->clearMap();
  }
  AlfSliceParam alfSliceParam;
  alfSliceParam.reset();
  const TempCtx  ctxStart(m_CtxCache, AlfCtx(m_CABACEstimator->getCtx()));
#endif
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
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
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
#else
  Area blk( 0, 0, recLuma.width, recLuma.height );
  deriveClassification( m_classifier, recLuma, blk );
  Area blkPCM(0, 0, recLuma.width, recLuma.height);
  resetPCMBlkClassInfo(cs, m_classifier, recLuma, blkPCM);
#endif

  // get CTB stats for filtering
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  deriveStatsForFiltering( orgYuv, recYuv, cs );
#else
  deriveStatsForFiltering( orgYuv, recYuv );
#endif

  // derive filter (luma)
  alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_LUMA
#if ENABLE_QPA
            , lambdaChromaWeight
#endif
            );

  // derive filter (chroma)
#if !JVET_N0415_CTB_ALF
  if ( alfSliceParam.enabledFlag[COMPONENT_Y] )
#endif
  {
    alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_CHROMA
#if ENABLE_QPA
              , lambdaChromaWeight
#endif
              );
  }

#if JVET_N0415_CTB_ALF
  m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
  alfEncoderCtb(cs, alfSliceParam
#if ENABLE_QPA
    , lambdaChromaWeight
#endif
  );

  alfReconstructor(cs, recYuv);
#endif
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

#if JVET_N0415_CTB_ALF
  reconstructCoeff(m_alfSliceParamTemp, channel, true, isLuma(channel));
  for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
  {
    for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
    {
      m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaCoeffFinal[i];
#if JVET_N0242_NON_LINEAR_ALF
      m_filterClippSet[classIdx][i] = isLuma(channel) ? m_clippFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaClippFinal[i];
#endif
    }
  }
#endif

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
#if JVET_N0415_CTB_ALF
  m_bitsNewFilter[channel] = 0;
#else
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
#endif
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

#if JVET_N0242_NON_LINEAR_ALF
    const int nonLinearFlagMax =
      ( isLuma( channel ) ? m_encCfg->getUseNonLinearAlfLuma() : m_encCfg->getUseNonLinearAlfChroma() )
      ? 2 : 1;

    for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
    {
#endif
    //2. all CTUs are on
    setEnableFlag( m_alfSliceParamTemp, channel, true );
#if JVET_N0242_NON_LINEAR_ALF
    m_alfSliceParamTemp.nonLinearFlag[channel] = nonLinearFlag;
#endif
    m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
    setCtuEnableFlag( m_ctuEnableFlag, channel, 1 );
#if JVET_N0242_NON_LINEAR_ALF
    cost = getFilterCoeffAndCost( cs, 0, channel, nonLinearFlag != 0, iShapeIdx, uiCoeffBits );
#else
    cost = getFilterCoeffAndCost( cs, 0, channel, false, iShapeIdx, uiCoeffBits );
#endif

    if( cost < costMin )
    {
#if JVET_N0415_CTB_ALF
      m_bitsNewFilter[channel] = uiCoeffBits;
#endif
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
#if JVET_N0415_CTB_ALF
          m_bitsNewFilter[channel] = uiCoeffBits;
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
    }// for nonLineaFlag
#endif
  }//for shapeIdx
  m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
#if !JVET_N0415_CTB_ALF
  copyCtuEnableFlag( m_ctuEnableFlag, m_ctuEnableFlagTmp, channel );

  //filtering
  reconstructCoeff( alfSliceParam, channel, isLuma( channel ) );

  for( int compIdx = compIDFirst; compIdx <= compIDLast; compIdx++ )
  {
    ComponentID compID = (ComponentID)compIdx;
    if( alfSliceParam.enabledFlag[compID] )
    {
      const PreCalcValues& pcv = *cs.pcv;
      int ctuIdx = 0;
      const int chromaScaleX = getComponentScaleX( compID, recBuf.chromaFormat );
      const int chromaScaleY = getComponentScaleY( compID, recBuf.chromaFormat );
      AlfFilterType filterType = isLuma( compID ) ? ALF_FILTER_7 : ALF_FILTER_5;
      short* coeff = isLuma( compID ) ? m_coeffFinal : alfSliceParam.chromaCoeff;
#if JVET_N0242_NON_LINEAR_ALF
      short* clipp = isLuma( compID ) ? m_clippFinal : m_chromaClippFinal; //alfSliceParam.chromaClipp;
#endif

#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
      bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
      int numHorVirBndry = 0, numVerVirBndry = 0;
      int horVirBndryPos[] = { 0, 0, 0 };
      int verVirBndryPos[] = { 0, 0, 0 };
#endif
      for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
      {
        for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
        {
          const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
          const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
          Area blk( xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY );

          if( m_ctuEnableFlag[compID][ctuIdx] )
          {
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
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
                  buf.copyFrom( recExtBuf.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf ) ) ) );
                  buf.extendBorderPel( MAX_ALF_PADDING_SIZE );
                  buf = buf.subBuf( UnitArea ( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

                  const Area blkSrc( 0, 0, w >> chromaScaleX, h >> chromaScaleY );
                  const Area blkDst( xStart >> chromaScaleX, yStart >> chromaScaleY, w >> chromaScaleX, h >> chromaScaleY );
                  if( filterType == ALF_FILTER_5 )
                  {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
                    m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs
                      , m_alfVBChmaCTUHeight
                      , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
                    );
#else
                    m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, m_clpRngs.comp[compIdx], cs
                      , m_alfVBChmaCTUHeight
                      , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
                    );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
                    m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs);
#else
                    m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, m_clpRngs.comp[compIdx], cs);
#endif
#endif
                  }
                  else if( filterType == ALF_FILTER_7 )
                  {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
                    m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs
                      , m_alfVBLumaCTUHeight
                      , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
                    );
#else
                    m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, m_clpRngs.comp[compIdx], cs
                      , m_alfVBLumaCTUHeight
                      , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
                    );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
                    m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs);
#else
                    m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, coeff, m_clpRngs.comp[compIdx], cs);
#endif
#endif
                  }
                  else
                  {
                    CHECK( 0, "Wrong ALF filter type" );
                  }

                  xStart = xEnd;
                }

                yStart = yEnd;
              }
            }
            else
            {
#endif
            if( filterType == ALF_FILTER_5 )
            {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs
                , m_alfVBChmaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
              );
#else
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs
                , m_alfVBChmaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
              );
#endif			  
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, m_clpRngs.comp[compIdx], cs
                , m_alfVBChmaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
              );
#else
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx], cs
                , m_alfVBChmaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
              );
#endif
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs);
#else
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, m_clpRngs.comp[compIdx], cs);
#else
              m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx], cs);
#endif
#endif
#endif

            }
            else if( filterType == ALF_FILTER_7 )
            {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs
                , m_alfVBLumaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
              );
#else
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs
                , m_alfVBLumaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
              );
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, m_clpRngs.comp[compIdx], cs
                , m_alfVBLumaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
              );
#else
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx], cs
                , m_alfVBLumaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
              );
#endif
#endif
#else

#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs);
#else
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, clipp, m_clpRngs.comp[compIdx], cs);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, coeff, m_clpRngs.comp[compIdx], cs);
#else
              m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx], cs);
#endif
#endif
#endif
           }
            else
            {
              CHECK( 0, "Wrong ALF filter type" );
            }
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
            }
#endif
          }
          ctuIdx++;
        }
      }
    }
  }
#endif
}

void EncAdaptiveLoopFilter::copyAlfSliceParam( AlfSliceParam& alfSliceParamDst, AlfSliceParam& alfSliceParamSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( &alfSliceParamDst, &alfSliceParamSrc, sizeof( AlfSliceParam ) );
  }
  else
  {
#if JVET_N0242_NON_LINEAR_ALF
    alfSliceParamDst.nonLinearFlag[channel] = alfSliceParamSrc.nonLinearFlag[channel];
#endif
    alfSliceParamDst.enabledFlag[COMPONENT_Cb] = alfSliceParamSrc.enabledFlag[COMPONENT_Cb];
    alfSliceParamDst.enabledFlag[COMPONENT_Cr] = alfSliceParamSrc.enabledFlag[COMPONENT_Cr];
    memcpy( alfSliceParamDst.chromaCoeff, alfSliceParamSrc.chromaCoeff, sizeof( alfSliceParamDst.chromaCoeff ) );
#if JVET_N0242_NON_LINEAR_ALF
    memcpy( alfSliceParamDst.chromaClipp, alfSliceParamSrc.chromaClipp, sizeof( alfSliceParamDst.chromaClipp ) );
#endif
  }
}
#if JVET_N0415_CTB_ALF
double EncAdaptiveLoopFilter::getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, bool onlyFilterCost )
#else
double EncAdaptiveLoopFilter::getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits )
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
    std::fill_n(m_alfClipMerged[iShapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfSliceParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    // Reset Merge Tmp Cov
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES].reset(AlfNumClippingValues[channel]);
#if JVET_N0415_CTB_ALF
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES + 1].reset(AlfNumClippingValues[channel]);
#endif
    //distortion
    dist += mergeFiltersAndCost( m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], m_alfClipMerged[iShapeIdx], uiCoeffBits );
#else
    dist += mergeFiltersAndCost( m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits );
#endif
  }
  else
  {
    //distortion
#if JVET_N0242_NON_LINEAR_ALF
    assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][iShapeIdx][0].numCoeff);
    std::fill_n(m_filterClippSet[0], MAX_NUM_ALF_CHROMA_COEFF, m_alfSliceParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues[CHANNEL_TYPE_CHROMA] / 2 : 0);
    dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterClippSet[0], m_filterCoeffSet[0], m_alfCovarianceFrame[channel][iShapeIdx][0], alfFilterShape, m_NUM_BITS, m_alfSliceParamTemp.nonLinearFlag[channel] );
#else
    dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterCoeffQuant, m_alfCovarianceFrame[channel][iShapeIdx][0].E, m_alfCovarianceFrame[channel][iShapeIdx][0].y, alfFilterShape.numCoeff, alfFilterShape.weights, m_NUM_BITS, true );
    memcpy( m_filterCoeffSet[0], m_filterCoeffQuant, sizeof( *m_filterCoeffQuant ) * alfFilterShape.numCoeff );
#endif
    //setEnableFlag( m_alfSliceParamTemp, channel, m_ctuEnableFlag );
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
    {
#if JVET_N0242_NON_LINEAR_ALF
      m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffSet[0][i];
      m_alfSliceParamTemp.chromaClipp[i] = m_filterClippSet[0][i];
#else
      m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffQuant[i];
#endif
    }
    uiCoeffBits += getCoeffRate( m_alfSliceParamTemp, true );
    uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  }
#if JVET_N0415_CTB_ALF
  if (onlyFilterCost)
  {
    return dist + m_lambda[channel] * uiCoeffBits;
  }
#endif
  double rate = uiCoeffBits + uiSliceFlag;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->codeAlfCtuEnableFlags( cs, channel, &m_alfSliceParamTemp);
  rate += FracBitsScale * (double)m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[channel] * rate;
}

int EncAdaptiveLoopFilter::getCoeffRate( AlfSliceParam& alfSliceParam, bool isChroma )
{
  int iBits = 0;
#if JVET_N0242_NON_LINEAR_ALF
  assert( isChroma );
#else
  if( !isChroma )
  {
    iBits++;                                               // alf_coefficients_delta_flag
    if( !alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      if( alfSliceParam.numLumaFilters > 1 )
      {
        iBits++;                                           // coeff_delta_pred_mode_flag
      }
    }
  }
#endif

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
#if JVET_N0242_NON_LINEAR_ALF
  AlfFilterShape alfShape( 5 );
#else
  AlfFilterShape alfShape( isChroma ? 5 : 7 );
#endif
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
#if JVET_N0242_NON_LINEAR_ALF
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
#else
  const short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;

  // vlc for all
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( isChroma || !alfSliceParam.alfLumaCoeffDeltaFlag || alfSliceParam.alfLumaCoeffFlag[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        int coeffVal = abs( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] );

        for( int k = 1; k < 15; k++ )
        {
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
        }
      }
    }
  }
#endif

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

#if JVET_N0242_NON_LINEAR_ALF
  // Filter coefficients
  for( int i = 0; i < alfShape.numCoeff - 1; i++ )
  {
    iBits += lengthGolomb( alfSliceParam.chromaCoeff[i], m_kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
  }
#else
  if( !isChroma )
  {
    if( alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      iBits += numFilters;             //filter_coefficient_flag[i]
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfSliceParam.alfLumaCoeffFlag[ind] && alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      iBits += lengthGolomb( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], m_kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
    }
  }
#endif

#if JVET_N0242_NON_LINEAR_ALF
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
#endif
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
#if JVET_N0415_CTB_ALF
#if JVET_N0242_NON_LINEAR_ALF
    dist += cov[classIdx].calcErrorForCoeffs(m_filterClippSet[classIdx], m_filterCoeffSet[classIdx], numCoeff, m_NUM_BITS);
#else
    dist += calcErrorForCoeffs(cov[classIdx].E, cov[classIdx].y, m_filterCoeffSet[classIdx], numCoeff, m_NUM_BITS);
#endif
#else
    int filterIdx = numClasses == 1 ? 0 : m_filterIndices[numFiltersMinus1][classIdx];
#if JVET_N0242_NON_LINEAR_ALF
    dist += cov[classIdx].calcErrorForCoeffs( m_filterClippSet[filterIdx], m_filterCoeffSet[filterIdx], numCoeff, m_NUM_BITS );
#else
    dist += calcErrorForCoeffs( cov[classIdx].E, cov[classIdx].y, m_filterCoeffSet[filterIdx], numCoeff, m_NUM_BITS );
#endif
#endif
  }

  return dist;
}

#if JVET_N0242_NON_LINEAR_ALF
double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits )
#else
double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits )
#endif
{
  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  static bool codedVarBins[MAX_NUM_ALF_CLASSES];
  static double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = MAX_DOUBLE;
  int predMode = 0, bestPredMode = 0, coeffBits, coeffBitsForce0;

#if JVET_N0242_NON_LINEAR_ALF
  mergeClasses( alfShape, covFrame, covMerged, clipMerged, MAX_NUM_ALF_CLASSES, m_filterIndices );
#else
  mergeClasses( covFrame, covMerged, MAX_NUM_ALF_CLASSES, m_filterIndices );
#endif

  while( numFilters >= 1 )
  {
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0415_CTB_ALF
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfSliceParam);
#else
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab);
#endif
#else
#if JVET_N0415_CTB_ALF
    dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfSliceParam );
#else
    dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab );
#endif
#endif
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

#if JVET_N0415_CTB_ALF
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
#endif

    if( cost <= costMin )
    {
      costMin = cost;
      numFiltersBest = numFilters;
      bestPredMode = predMode;
    }
    numFilters--;
  }

#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0415_CTB_ALF
  dist = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab, alfSliceParam );
#else
  dist = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab );
#endif
#else
#if JVET_N0415_CTB_ALF
  dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab, alfSliceParam );
#else
  dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab );
#endif
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
        memset( m_filterClippSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
      alfSliceParam.lumaClipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterClippSet[ind][i];
#endif
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
#if JVET_N0415_CTB_ALF
  len++; //fixed filter set flag
  if (alfSliceParam.fixedFilterSetIndex > 0)
  {
    len += getTBlength(alfSliceParam.fixedFilterSetIndex - 1, NUM_FIXED_FILTER_SETS);
    len += 1; //fixed filter flag pattern
    if (alfSliceParam.fixedFilterPattern > 0)
      len += MAX_NUM_ALF_CLASSES;  //"fixed_filter_flag" for each class
  }
#endif
  return len;
}

int EncAdaptiveLoopFilter::lengthTruncatedUnary( int symbol, int maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return 0;
  }

  bool codeLast = ( maxSymbol > symbol );
  int bins = 0;
  int numBins = 0;
  while( symbol-- )
  {
    bins <<= 1;
    bins++;
    numBins++;
  }
  if( codeLast )
  {
    bins <<= 1;
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

#if JVET_N0242_NON_LINEAR_ALF
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

#endif
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

#if JVET_N0242_NON_LINEAR_ALF
  int rateClipp = m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? getCostFilterClipp( alfShape, filterSet, numFilters ) : 0;

  return ( numFilters > 1 ? 1 : 0 )        // coeff_delta_pred_mode_flag
       + rateClipp
       + ( predMode ? ratePredMode1 : ratePredMode0 ); // min_golomb_order, golomb_order_increase_flag, alf_coeff_luma_delta
#else
  return ( numFilters > 1 ? 1 : 0 )        // coeff_delta_pred_mode_flag
       + ( predMode ? ratePredMode1 : ratePredMode0 ); // min_golomb_order, golomb_order_increase_flag, alf_coeff_luma_delta
#endif
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

#if JVET_N0242_NON_LINEAR_ALF
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

#endif
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

#if JVET_N0242_NON_LINEAR_ALF
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

#endif
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

#if JVET_N0242_NON_LINEAR_ALF
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

#endif
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

#if JVET_N0242_NON_LINEAR_ALF
int EncAdaptiveLoopFilter::lengthGolomb( int coeffVal, int k, bool signed_coeff )
#else
int EncAdaptiveLoopFilter::lengthGolomb( int coeffVal, int k )
#endif
{
  int m = 2 << ( k - 1 );
  int q = coeffVal / m;
#if JVET_N0242_NON_LINEAR_ALF
  if( signed_coeff && coeffVal != 0 )
#else
  if( coeffVal != 0 )
#endif
  {
    return q + 2 + k;
  }
  else
  {
    return q + 1 + k;
  }
}

#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0415_CTB_ALF
double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfSliceParam& alfSliceParam )
#else
double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2] )
#endif
#else
#if JVET_N0415_CTB_ALF
double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfSliceParam& alfSliceParam )
#else
double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2] )
#endif
#endif
{
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

#if JVET_N0415_CTB_ALF
  alfSliceParam.fixedFilterSetIndex = 0;
  AlfCovariance& tmpCovFf = covMerged[MAX_NUM_ALF_CLASSES + 1];
  double factor = 1 << (m_NUM_BITS - 1);
  double errorMin = 0;
  double errorMinPerClass[MAX_NUM_ALF_CLASSES] = { 0 };
  double errorCurSetPerClass[MAX_NUM_ALF_CLASSES] = { 0 };
  int    fixedFilterFlagPerClass[MAX_NUM_ALF_CLASSES] = { 0 };

#if JVET_N0242_NON_LINEAR_ALF
  if (!alfSliceParam.nonLinearFlag[CHANNEL_TYPE_LUMA])
#endif
  {
    alfSliceParam.fixedFilterSetIndex = 0;
    for (int filterSetIdx = 0; filterSetIdx < NUM_FIXED_FILTER_SETS; filterSetIdx++)
    {
      double errorCur = 0;
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
        int fixedFilterIdx = m_classToFilterMapping[filterSetIdx][classIdx];
#if JVET_N0242_NON_LINEAR_ALF
        errorCurSetPerClass[classIdx] = cov[classIdx].calcErrorForCoeffs(clipMerged[numFilters - 1][filterIndices[classIdx]], m_fixedFilterSetCoeff[fixedFilterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#else
        errorCurSetPerClass[classIdx] = calcErrorForCoeffs(cov[classIdx].E, cov[classIdx].y, m_fixedFilterSetCoeff[fixedFilterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#endif

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
#endif

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    tmpCov.reset();
#if JVET_N0242_NON_LINEAR_ALF
    bool found_clip = false;
#endif
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      if( filterIndices[classIdx] == filtIdx )
      {
#if JVET_N0415_CTB_ALF
        //adjust stat
        tmpCovFf = cov[classIdx];
        if (alfSliceParam.fixedFilterSetIndex > 0 && alfSliceParam.fixedFilterIdx[classIdx] > 0
#if JVET_N0242_NON_LINEAR_ALF
          && alfSliceParam.nonLinearFlag[CHANNEL_TYPE_LUMA] == false
#endif
          )
        {
          int fixedFilterIdx = m_classToFilterMapping[alfSliceParam.fixedFilterSetIndex - 1][classIdx];
          tmpCovFf.pixAcc += errorMinPerClass[classIdx];
          for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
          {
            double sum = 0;
            for (int j = 0; j < MAX_NUM_ALF_LUMA_COEFF; j++)
            {
#if JVET_N0242_NON_LINEAR_ALF
              sum += tmpCovFf.E[clipMerged[numFilters - 1][classIdx][i]][clipMerged[numFilters - 1][classIdx][j]][i][j] * m_fixedFilterSetCoeff[fixedFilterIdx][j];
#else
              sum += tmpCovFf.E[i][j] * m_fixedFilterSetCoeff[fixedFilterIdx][j];
#endif
            }
            sum /= factor;
#if JVET_N0242_NON_LINEAR_ALF
            tmpCovFf.y[clipMerged[numFilters - 1][classIdx][i]][i] -= sum;
#else
            tmpCovFf.y[i] -= sum;
#endif
          }
        }
        tmpCov += tmpCovFf;
#else
        tmpCov += cov[classIdx];
#endif
#if JVET_N0242_NON_LINEAR_ALF
        if( !found_clip )
        {
          found_clip = true; // clip should be at the adress of shortest one
          memcpy(m_filterClippSet[filtIdx], clipMerged[numFilters-1][classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
        }
#endif
      }
    }

    // Find coeffcients
#if JVET_N0242_NON_LINEAR_ALF
    assert(alfShape.numCoeff == tmpCov.numCoeff);
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, false );
#else
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterCoeffQuant, tmpCov.E, tmpCov.y, alfShape.numCoeff, alfShape.weights, m_NUM_BITS );
#endif
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];
#if !JVET_N0242_NON_LINEAR_ALF

    // store coeff
    memcpy( m_filterCoeffSet[filtIdx], m_filterCoeffQuant, sizeof( int )*alfShape.numCoeff );
#endif
  }
  return error;
}

#if JVET_N0242_NON_LINEAR_ALF
double EncAdaptiveLoopFilter::deriveCoeffQuant( int *filterClipp, int *filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip )
#else
double EncAdaptiveLoopFilter::deriveCoeffQuant( int *filterCoeffQuant, double **E, double *y, const int numCoeff, std::vector<int>& weights, const int bitDepth, const bool bChroma )
#endif
{
  const int factor = 1 << ( bitDepth - 1 );
#if JVET_N0242_NON_LINEAR_ALF
const int numCoeff = shape.numCoeff;
#else
  static int filterCoeffQuantMod[MAX_NUM_ALF_LUMA_COEFF];
#endif
  static double filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

#if JVET_N0242_NON_LINEAR_ALF
  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip );
#else
  gnsSolveByChol( E, y, filterCoeff, numCoeff );
#endif
  roundFiltCoeff( filterCoeffQuant, filterCoeff, numCoeff, factor );
#if JVET_N0242_NON_LINEAR_ALF

  const int max_value = factor - 1;
  const int min_value = -factor;

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
#else
  const int targetCoeffSumInt = 0;
  int quantCoeffSum = 0;
  for( int i = 0; i < numCoeff; i++ )
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
  }
  int count = 0;
  while( quantCoeffSum != targetCoeffSumInt && count < 10 )
  {
    int sign = quantCoeffSum > targetCoeffSumInt ? 1 : -1;
    int diff = ( quantCoeffSum - targetCoeffSumInt ) * sign;

    double errMin = MAX_DOUBLE;
    int minInd = -1;

    for( int k = 0; k < numCoeff; k++ )
    {
      if( weights[k] <= diff )
      {
        memcpy( filterCoeffQuantMod, filterCoeffQuant, sizeof( int ) * numCoeff );

        filterCoeffQuantMod[k] -= sign;
        double error = calcErrorForCoeffs( E, y, filterCoeffQuantMod, numCoeff, bitDepth );

        if( error < errMin )
        {
          errMin = error;
          minInd = k;
        }
      }
    }

    if( minInd != -1 )
    {
      filterCoeffQuant[minInd] -= sign;
    }

    quantCoeffSum = 0;
    for( int i = 0; i < numCoeff; i++ )
    {
      quantCoeffSum += weights[i] * filterCoeffQuant[i];
    }
    ++count;
  }
  if( count == 10 )
  {
    memset( filterCoeffQuant, 0, sizeof( int ) * numCoeff );
  }

  int max_value = factor - 1;
  int min_value = -factor;

  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    filterCoeffQuant[i] = std::min( max_value, std::max( min_value, filterCoeffQuant[i] ) );
    filterCoeff[i] = filterCoeffQuant[i] / double( factor );
  }

  quantCoeffSum = 0;
  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
    filterCoeff[i] = filterCoeffQuant[i] / double(factor);
  }
  filterCoeffQuant[numCoeff - 1] = -quantCoeffSum;
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);


  //Restrict the range of the center coefficient
  int max_value_center = (2 * factor - 1) - factor;
  int min_value_center = 0 - factor;

  filterCoeffQuant[numCoeff - 1] = std::min(max_value_center, std::max(min_value_center, filterCoeffQuant[numCoeff - 1]));
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);

  int coeffQuantAdjust[MAX_NUM_ALF_LUMA_COEFF];
  int adjustedTotalCoeff = (numCoeff - 1) << 1;

  count = 0;
  quantCoeffSum += filterCoeffQuant[numCoeff - 1];
  while (quantCoeffSum != targetCoeffSumInt && count < 15)
  {
    int sign = quantCoeffSum > targetCoeffSumInt ? 1 : -1;
    int diff = (quantCoeffSum - targetCoeffSumInt) * sign;

    if (diff > 4 * adjustedTotalCoeff)     sign = sign * 8;
    else if (diff > 2 * adjustedTotalCoeff)     sign = sign * 4;
    else if (diff >     adjustedTotalCoeff)     sign = sign * 2;

    double errMin = MAX_DOUBLE;
    int    minInd = -1;

    for (int k = 0; k < numCoeff - 1; k++)
    {
      memcpy(coeffQuantAdjust, filterCoeffQuant, sizeof(int) * numCoeff);

      coeffQuantAdjust[k] -= sign;

      if (coeffQuantAdjust[k] <= max_value && coeffQuantAdjust[k] >= min_value)
      {
        double error = calcErrorForCoeffs(E, y, coeffQuantAdjust, numCoeff, bitDepth);

        if (error < errMin)
        {
          errMin = error;
          minInd = k;
        }
      }
    }

    if (minInd != -1)
    {
      filterCoeffQuant[minInd] -= sign;
      quantCoeffSum -= (weights[minInd] * sign);
    }

    ++count;
  }

  if (quantCoeffSum != targetCoeffSumInt)
  {
    memset(filterCoeffQuant, 0, sizeof(int) * numCoeff);
  }

  for (int i = 0; i < numCoeff - 1; i++)
  {
    CHECK(filterCoeffQuant[i] > max_value || filterCoeffQuant[i] < min_value, "filterCoeffQuant[i]>max_value || filterCoeffQuant[i]<min_value");
    filterCoeff[i] = filterCoeffQuant[i] / double(factor);
  }
  CHECK(filterCoeffQuant[numCoeff - 1] > max_value_center || filterCoeffQuant[numCoeff - 1] < min_value_center, "filterCoeffQuant[numCoeff-1]>max_value_center || filterCoeffQuant[numCoeff-1]<min_value_center");
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);


  double error = calcErrorForCoeffs( E, y, filterCoeffQuant, numCoeff, bitDepth );
  return error;
#endif
}

#if !JVET_N0242_NON_LINEAR_ALF
double EncAdaptiveLoopFilter::calcErrorForCoeffs( double **E, double *y,
#if JVET_N0415_CTB_ALF
  const
#endif
  int *coeff, const int numCoeff, const int bitDepth )
{
  double factor = 1 << ( bitDepth - 1 );
  double error = 0;

  for( int i = 0; i < numCoeff; i++ )   //diagonal
  {
    double sum = 0;
    for( int j = i + 1; j < numCoeff; j++ )
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E[i][j] * coeff[j];
    }
    error += ( ( E[i][i] * coeff[i] + sum * 2 ) / factor - 2 * y[i] ) * coeff[i];
  }

  return error / factor;
}

#endif
void EncAdaptiveLoopFilter::roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor )
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    filterCoeffQuant[i] = int( filterCoeff[i] * sign * factor + 0.5 ) * sign;
  }
}

#if JVET_N0242_NON_LINEAR_ALF
void EncAdaptiveLoopFilter::mergeClasses( const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] )
#else
void EncAdaptiveLoopFilter::mergeClasses( AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] )
#endif
{
#if JVET_N0242_NON_LINEAR_ALF
  static int tmpClip[MAX_NUM_ALF_LUMA_COEFF];
  static int bestMergeClip[MAX_NUM_ALF_LUMA_COEFF];
  static double err[MAX_NUM_ALF_CLASSES];
  static double bestMergeErr;
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
    covMerged[i].numBins = m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#endif
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
#if JVET_N0242_NON_LINEAR_ALF
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
#endif

#if JVET_N0242_NON_LINEAR_ALF
  while( numRemaining >= 2 )
#else
  while( numRemaining > 2 )
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
            double error1 = err[i];
            double error2 = err[j];
#else
            double error1 = calculateError( covMerged[i] );
            double error2 = calculateError( covMerged[j] );
#endif

            tmpCov.add( covMerged[i], covMerged[j] );
#if JVET_N0242_NON_LINEAR_ALF
            for( int l = 0; l < MAX_NUM_ALF_LUMA_COEFF; ++l )
            {
              tmpClip[l] = (clipMerged[numRemaining-1][i][l] + clipMerged[numRemaining-1][j][l] + 1 ) >> 1;
            }
            double errorMerged = m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? tmpCov.optimizeFilterClip( alfShape, tmpClip ) : tmpCov.calculateError( tmpClip );
            double error = errorMerged - error1 - error2;
#else
            double error = calculateError( tmpCov ) - error1 - error2;
#endif

            if( error < errorMin )
            {
#if JVET_N0242_NON_LINEAR_ALF
              bestMergeErr = errorMerged;
              memcpy(bestMergeClip, tmpClip, sizeof(bestMergeClip));
#endif
              errorMin = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
#if JVET_N0242_NON_LINEAR_ALF
    memcpy(clipMerged[numRemaining-2], clipMerged[numRemaining-1], sizeof(int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF]));
    memcpy(clipMerged[numRemaining-2][bestToMergeIdx1], bestMergeClip, sizeof(bestMergeClip));
    err[bestToMergeIdx1] = bestMergeErr;
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
    m_alfCovarianceFrame[channel][iShapeIdx][i].reset(AlfNumClippingValues[channel]);
#else
    m_alfCovarianceFrame[channel][iShapeIdx][i].reset();
#endif
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

#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
void EncAdaptiveLoopFilter::deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv, CodingStructure& cs )
#else
void EncAdaptiveLoopFilter::deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv )
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
          m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset(AlfNumClippingValues[toChannelType( compID )]);
#else
          m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset();
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset(AlfNumClippingValues[channelID]);
#else
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset();
#endif
      }
    }
  }

#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  const PreCalcValues& pcv = *cs.pcv;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
#endif
  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
      const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
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
              const CompArea& compAreaDst = areaDst.block( compID );

              int  recStride = recBuf.get( compID ).stride;
              Pel* rec = recBuf.get( compID ).bufAt( compArea );

              int  orgStride = orgYuv.get(compID).stride;
              Pel* org = orgYuv.get(compID).bufAt(xStart >> ::getComponentScaleX(compID, m_chromaFormat), yStart >> ::getComponentScaleY(compID, m_chromaFormat));

              ChannelType chType = toChannelType( compID );

              for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
              {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
                getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compAreaDst, compArea, chType
                  , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
                  , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))
                );
#else
                getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, compArea
                  , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
                  , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))
                );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
                getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compAreaDst, compArea, chType);
#else
                getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compAreaDst, compArea);
#endif
#endif
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
#endif
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
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, compArea, chType
            , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
            , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))        
          );
#else
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, chType
            , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
            , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))        
          );
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, compArea
            , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
            , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))
          );
#else
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea
            , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
            , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))
          );
#endif
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, compArea, chType);
#else
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, chType);
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, compArea);
#else
          getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea);
#endif
#endif
#endif


          const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

          for( int classIdx = 0; classIdx < numClasses; classIdx++ )
          {
            m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
          }
        }
      }
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
      }
#endif
      ctuRsAddr++;
    }
  }
}

#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos)
#else
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos)
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, int vbCTUHeight, int vbPos)
#else
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area, int vbCTUHeight, int vbPos)
#endif
#endif

#else
#if JVET_N0242_NON_LINEAR_ALF
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel)
#else
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area, const ChannelType channel)
#endif
#else
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area)
#else
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area)
#endif
#endif
#endif


{
#if JVET_N0242_NON_LINEAR_ALF
  static int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues];

  const int numBins = AlfNumClippingValues[channel];
#else
  static int ELocal[MAX_NUM_ALF_LUMA_COEFF];
#endif

  int transposeIdx = 0;
  int classIdx = 0;

  for( int i = 0; i < area.height; i++ )
  {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
    int vbDistance = ((areaDst.y + i) % vbCTUHeight) - vbPos;
#else
    int vbDistance = (i % vbCTUHeight) - vbPos;
#endif
#endif
    for( int j = 0; j < area.width; j++ )
    {
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
      if( classifier && classifier[areaDst.y + i][areaDst.x + j].classIdx == m_ALF_UNUSED_CLASSIDX && classifier[areaDst.y + i][areaDst.x + j].transposeIdx == m_ALF_UNUSED_TRANSPOSIDX )
#else
      if( classifier && classifier[area.y + i][area.x + j].classIdx == m_ALF_UNUSED_CLASSIDX && classifier[area.y + i][area.x + j].transposeIdx == m_ALF_UNUSED_TRANSPOSIDX )
#endif
      {
        continue;
      }
#if JVET_N0242_NON_LINEAR_ALF
      std::memset( ELocal, 0, sizeof( ELocal ) );
#else
      std::memset( ELocal, 0, shape.numCoeff * sizeof( int ) );
#endif
      if( classifier )
      {
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
        AlfClassifier& cl = classifier[areaDst.y + i][areaDst.x + j];
#else
        AlfClassifier& cl = classifier[area.y + i][area.x + j];
#endif
        transposeIdx = cl.transposeIdx;
        classIdx = cl.classIdx;
      }

      double weight = 1.0;
      if (m_alfWSSD)
      {
        weight = m_lumaLevelToWeightPLUT[org[j]];
      }
      int yLocal = org[j] - rec[j];
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
      calcCovariance(ELocal, rec + j, recStride, shape, transposeIdx, channel, vbDistance);
#else
      calcCovariance(ELocal, rec + j, recStride, shape.pattern.data(), shape.filterLength >> 1, transposeIdx, vbDistance);
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
      calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel );
#else
      calcCovariance( ELocal, rec + j, recStride, shape.pattern.data(), shape.filterLength >> 1, transposeIdx );
#endif
#endif
      for( int k = 0; k < shape.numCoeff; k++ )
      {
        for( int l = k; l < shape.numCoeff; l++ )
        {
#if JVET_N0242_NON_LINEAR_ALF
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
#else
          if (m_alfWSSD)
          {
            alfCovariace[classIdx].E[k][l] += weight * (double)(ELocal[k] * ELocal[l]);
          }
          else
          alfCovariace[classIdx].E[k][l] += ELocal[k] * ELocal[l];
#endif
        }
#if JVET_N0242_NON_LINEAR_ALF
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
#else
        if (m_alfWSSD)
        {
          alfCovariace[classIdx].y[k] += weight * (double)(ELocal[k] * yLocal);
        }
        else
        alfCovariace[classIdx].y[k] += ELocal[k] * yLocal;
#endif
      }
      if (m_alfWSSD)
      {
#if JVET_N0242_NON_LINEAR_ALF
        alfCovariance[classIdx].pixAcc += weight * (double)(yLocal * yLocal);
#else
        alfCovariace[classIdx].pixAcc += weight * (double)(yLocal * yLocal);
#endif
      }
      else
#if JVET_N0242_NON_LINEAR_ALF
      {
        alfCovariance[classIdx].pixAcc += yLocal * yLocal;
      }
#else
      alfCovariace[classIdx].pixAcc += yLocal * yLocal;
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
        for( int b0 = 0; b0 < numBins; b0++ )
        {
          for( int b1 = 0; b1 < numBins; b1++ )
          {
            alfCovariance[classIdx].E[b0][b1][k][l] = alfCovariance[classIdx].E[b1][b0][l][k];
          }
        }
#else
        alfCovariace[classIdx].E[k][l] = alfCovariace[classIdx].E[l][k];
#endif
      }
    }
  }
}
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
void EncAdaptiveLoopFilter::calcCovariance(int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance)
#else
void EncAdaptiveLoopFilter::calcCovariance(int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx, int vbDistance)
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
void EncAdaptiveLoopFilter::calcCovariance(int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel)
#else
void EncAdaptiveLoopFilter::calcCovariance(int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx)
#endif
#endif

{
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
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
#endif

#if JVET_N0242_NON_LINEAR_ALF
  const int *filterPattern = shape.pattern.data();
  const int halfFilterLength = shape.filterLength >> 1;
  const Pel* clip = m_alfClippingValues[channel];
  const int numBins = AlfNumClippingValues[channel];

#endif
  int k = 0;

#if JVET_N0242_NON_LINEAR_ALF
  const short curr = rec[0];
#endif

  if( transposeIdx == 0 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;
#else
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;
#endif
#if JVET_N0242_NON_LINEAR_ALF
      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++ )
      {
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
#else
      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++ )
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
#endif
    }
#if JVET_N0242_NON_LINEAR_ALF
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
#else
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
#endif
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
        }
    }
#else
      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++)
      {
        ELocal[filterPattern[k++]] += rec0[std::max(i, clipTopRow) * stride] + rec1[-std::max(i, -clipBotRow) * stride];
      }
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[i * stride], rec1[-i * stride]);
        }
      }
#else
      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++)
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
#endif
#endif
    }
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
      }
    }
#else
    for (int i = -halfFilterLength; i < 0; i++)
    {
      ELocal[filterPattern[k++]] += rec[std::max(i, clipTopRow) * stride] + rec[-std::max(i, -clipBotRow) * stride];
    }
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
    for( int i = -halfFilterLength; i < 0; i++, k++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[i*stride], rec[-i * stride]);
      }
    }
#else
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
#endif
#endif
  }
  else if( transposeIdx == 2 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;
#else
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;
#endif

#if JVET_N0242_NON_LINEAR_ALF
      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++ )
      {
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
#else
      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j-- )
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
#endif
    }
#if JVET_N0242_NON_LINEAR_ALF
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
#else
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
#endif
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
      for (int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
        }
      }
#else
      for (int i = halfFilterLength + j; i >= -halfFilterLength - j; i--)
      {
        ELocal[filterPattern[k++]] += rec0[std::max(i, clipTopRow) * stride] + rec1[-std::max(i, -clipBotRow) * stride];
      }
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
      for( int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++ )
      {
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[i * stride], rec1[-i * stride]);
        }
      }
#else
      for( int i = halfFilterLength + j; i >= -halfFilterLength - j; i-- )
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
#endif
#endif
    }
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
      }
    }
#else
    for (int i = -halfFilterLength; i < 0; i++)
    {
      ELocal[filterPattern[k++]] += rec[std::max(i, clipTopRow) * stride] + rec[-std::max(i, -clipBotRow) * stride];
    }
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[i*stride], rec[-i * stride]);
      }
    }
#else
    for (int i = -halfFilterLength; i < 0; i++)
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
#endif
#endif

  }
#if JVET_N0242_NON_LINEAR_ALF
  for( int b = 0; b < numBins; b++ )
  {
    ELocal[filterPattern[k]][b] += curr;
  }
#else
  ELocal[filterPattern[k++]] += rec[0];
#endif
}



#if !JVET_N0242_NON_LINEAR_ALF
double EncAdaptiveLoopFilter::calculateError( AlfCovariance& cov )
{
  static double c[MAX_NUM_ALF_COEFF];

  gnsSolveByChol( cov.E, cov.y, c, cov.numCoeff );

  double sum = 0;
  for( int i = 0; i < cov.numCoeff; i++ )
  {
    sum += c[i] * cov.y[i];
  }

  return cov.pixAcc - sum;
}

//********************************
// Cholesky decomposition
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//Find filter coeff related
int EncAdaptiveLoopFilter::gnsCholeskyDec( double **inpMatr, double outMatr[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], int numEq )
{
  static double invDiag[MAX_NUM_ALF_COEFF];  /* Vector of the inverse of diagonal entries of outMatr */

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

void EncAdaptiveLoopFilter::gnsTransposeBacksubstitution( double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* rhs, double* x, int order )
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

void EncAdaptiveLoopFilter::gnsBacksubstitution( double R[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* z, int size, double* A )
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

int EncAdaptiveLoopFilter::gnsSolveByChol( double **LHS, double *rhs, double *x, int numEq )
{
  static double aux[MAX_NUM_ALF_COEFF];     /* Auxiliary vector */
  static double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF];    /* Upper triangular Cholesky factor of LHS */
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
#endif
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

#if JVET_N0415_CTB_ALF
std::vector<int> EncAdaptiveLoopFilter::getAvaiApsIdsLuma(CodingStructure& cs, int &newApsId)
{
  APS** apss = cs.slice->getAPSs();
  for (int i = 0; i < MAX_NUM_APS; i++)
  {
    apss[i] = m_apsMap->getPS(i);
  }

  std::vector<int> result;
  int apsIdChecked = 0, curApsId = m_apsIdStart;
  if (curApsId < int(MAX_NUM_APS))
  {
    while (apsIdChecked < MAX_NUM_APS && !cs.slice->isIntra() && result.size() < (ALF_CTB_MAX_NUM_APS - 1) && !cs.slice->getPendingRasInit() && !cs.slice->isIDRorBLA())
    {
      APS* curAPS = cs.slice->getAPSs()[curApsId];
      if (curAPS && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA])
      {
        result.push_back(curApsId);
      }
      apsIdChecked++;
      curApsId = (curApsId + 1) % MAX_NUM_APS;
    }
  }
  cs.slice->setTileGroupNumAps((int)result.size());
  cs.slice->setAPSs(result);
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
  APS**          apss = cs.slice->getAPSs();
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
#if JVET_N0242_NON_LINEAR_ALF
                short* pClipp = m_clippFinal;
#endif
                for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                {
                  m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
#if JVET_N0242_NON_LINEAR_ALF
                  m_clipTmp[i] = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
#endif
                }
#if JVET_N0242_NON_LINEAR_ALF
                dDistOrgNewFilter += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#else
                dDistOrgNewFilter += calcErrorForCoeffs(m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].E, m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].y, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#endif
              }
            }
          }
          if (blocksUsingNewFilter > 0 && blocksUsingNewFilter < m_numCTUsInPic)
          {
            int bitNL[2] = { 0, 0 };
            double errNL[2] = { 0.0, 0.0 };
#if JVET_N0242_NON_LINEAR_ALF
            m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 1;
            if (m_encCfg->getUseNonLinearAlfLuma())
            {
              errNL[1] = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, 0, bitNL[1], true);
              m_alfSliceParamTempNL = m_alfSliceParamTemp;
            }
            else
#endif
            {
              errNL[1] = MAX_DOUBLE;
            }
#if JVET_N0242_NON_LINEAR_ALF
            m_alfSliceParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 0;
#endif
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
#if JVET_N0242_NON_LINEAR_ALF
                dist += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipDefaultEnc, m_fixedFilterSetCoeff[filterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#else
                dist += calcErrorForCoeffs(m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].E, m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].y, m_fixedFilterSetCoeff[filterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#endif
              }
              else
              {
                short *pCoeff;
#if JVET_N0242_NON_LINEAR_ALF
                short *pClipp;
#endif
                if (useNewFilter && filterSetIdx == NUM_FIXED_FILTER_SETS)
                {
                  pCoeff = m_coeffFinal;
#if JVET_N0242_NON_LINEAR_ALF
                  pClipp = m_clippFinal;
#endif
                }
                else if (useNewFilter)
                {
                  pCoeff = m_coeffApsLuma[filterSetIdx - 1 - NUM_FIXED_FILTER_SETS];
#if JVET_N0242_NON_LINEAR_ALF
                  pClipp = m_clippApsLuma[filterSetIdx - 1 - NUM_FIXED_FILTER_SETS];
#endif
                }
                else
                {
                  pCoeff = m_coeffApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS];
#if JVET_N0242_NON_LINEAR_ALF
                  pClipp = m_clippApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS];
#endif
                }
                for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                {
                  m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
#if JVET_N0242_NON_LINEAR_ALF
                  m_clipTmp[i] = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
#endif
                }
#if JVET_N0242_NON_LINEAR_ALF
                dist += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#else
                dist += calcErrorForCoeffs(m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].E, m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].y, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#endif
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
    cs.slice->setAPSs(bestApsIds);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
    for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
    {
      alfCtbFilterSetIndex[ctuIdx] = m_alfCtbFilterSetIndexTmp[ctuIdx];
    }
    if (alfSliceParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA])
    {
      APS* newAPS = m_apsMap->getPS(newApsId);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS(newApsId);
      }
      newAPS->setAlfAPSParam(alfSliceParamNewFiltersBest);
      m_apsMap->setChangedFlag(newApsId);
      m_apsIdStart = newApsId;
    }

    std::vector<int> apsIds = cs.slice->getTileGroupApsIdLuma();
    for (int i = 0; i < (int)cs.slice->getTileGroupNumAps(); i++)
    {
      apss[apsIds[i]] = m_apsMap->getPS(apsIds[i]);
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
    APS* curAPS = m_apsMap->getPS(curApsId);
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
#if JVET_N0242_NON_LINEAR_ALF
          m_clipTmp[i] = m_chromaClippFinal[i];
#endif
        }
#if JVET_N0242_NON_LINEAR_ALF
        double dist = distUnfilterCtu + m_alfCovariance[compId][0][ctbIdx][0].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, m_NUM_BITS);
#else
        double dist = distUnfilterCtu + calcErrorForCoeffs(m_alfCovariance[compId][0][ctbIdx][0].E, m_alfCovariance[compId][0][ctbIdx][0].y, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, m_NUM_BITS);
#endif
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
      APS* newAPS = m_apsMap->getPS(newApsIdChroma);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS(newApsIdChroma);
        newAPS->getAlfAPSParam().reset();
      }
      newAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] = true;
#if JVET_N0242_NON_LINEAR_ALF
      newAPS->getAlfAPSParam().nonLinearFlag[CHANNEL_TYPE_CHROMA] = alfSliceParamNewFilters.nonLinearFlag[CHANNEL_TYPE_CHROMA];
#endif
      newAPS->getAlfAPSParam().tLayer = cs.slice->getTLayer();
      for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
      {
        newAPS->getAlfAPSParam().chromaCoeff[i] = alfSliceParamNewFilters.chromaCoeff[i];
#if JVET_N0242_NON_LINEAR_ALF
        newAPS->getAlfAPSParam().chromaClipp[i] = alfSliceParamNewFilters.chromaClipp[i];
#endif
      }
      m_apsMap->setChangedFlag(newApsIdChroma);
      m_apsIdStart = newApsIdChroma;
    }
    apss[cs.slice->getTileGroupApsIdChroma()] = m_apsMap->getPS(cs.slice->getTileGroupApsIdChroma());
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
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
#endif
  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const int width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
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
#if JVET_N0242_NON_LINEAR_ALF
              short *clip;
#endif
              if (filterSetIndex >= NUM_FIXED_FILTER_SETS)
              {
                coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#if JVET_N0242_NON_LINEAR_ALF
                clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#endif
              }
              else
              {
                coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
#if JVET_N0242_NON_LINEAR_ALF
                clip = m_clipDefault;
#endif
              }
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
              m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs
                , m_alfVBLumaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
              );
#else
              m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, m_clpRngs.comp[COMPONENT_Y], cs
                , m_alfVBLumaCTUHeight
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
              );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
              m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs);
#else
              m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, m_clpRngs.comp[COMPONENT_Y], cs);
#endif
#endif
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
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
                m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs
                  , m_alfVBChmaCTUHeight
                  , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
                );
#else
                m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal, m_clpRngs.comp[compIdx], cs
                  , m_alfVBChmaCTUHeight
                  , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
                );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
                m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs);
#else
                m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal, m_clpRngs.comp[compIdx], cs);
#endif
#endif
              }
            }
            
            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {
#endif

      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
      {
        Area blk(xPos, yPos, width, height);
        short filterSetIndex = alfCtuFilterIndex[ctuIdx];
        short *coeff;
#if JVET_N0242_NON_LINEAR_ALF
        short *clip;
#endif
        if (filterSetIndex >= NUM_FIXED_FILTER_SETS)
        {
          coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#if JVET_N0242_NON_LINEAR_ALF
          clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#endif
        }
        else
        {
          coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
#if JVET_N0242_NON_LINEAR_ALF
          clip = m_clipDefault;
#endif
        }
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs
          , m_alfVBLumaCTUHeight
          , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
        );
#else
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, m_clpRngs.comp[COMPONENT_Y], cs
          , m_alfVBLumaCTUHeight
          , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
        );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs);
#else
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, m_clpRngs.comp[COMPONENT_Y], cs);
#endif
#endif
#else
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs
          , m_alfVBLumaCTUHeight
          , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
        );
#else
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, COMPONENT_Y, coeff, m_clpRngs.comp[COMPONENT_Y], cs
          , m_alfVBLumaCTUHeight
          , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
        );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs);
#else
        m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, COMPONENT_Y, coeff, m_clpRngs.comp[COMPONENT_Y], cs);
#endif
#endif       
#endif
      }

      for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        ComponentID compID = ComponentID(compIdx);
        const int chromaScaleX = getComponentScaleX(compID, recBuf.chromaFormat);
        const int chromaScaleY = getComponentScaleY(compID, recBuf.chromaFormat);
        if (m_ctuEnableFlag[compIdx][ctuIdx])
        {
          Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
            , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
          );
#else
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal, m_clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
            , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
          );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs);
#else
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal, m_clpRngs.comp[compIdx], cs);
#endif
#endif
#else		  
#if JVET_N0180_ALF_LINE_BUFFER_REDUCTION
#if JVET_N0242_NON_LINEAR_ALF
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
            , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
          );
#else
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, m_chromaCoeffFinal, m_clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
            , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
          );
#endif
#else
#if JVET_N0242_NON_LINEAR_ALF
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs);
#else
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, compID, m_chromaCoeffFinal, m_clpRngs.comp[compIdx], cs);
#endif
#endif
#endif
        }
      }
#if JVET_N0438_LOOP_FILTER_DISABLED_ACROSS_VIR_BOUND
      }
#endif
      ctuIdx++;
    }
  }
}
#endif