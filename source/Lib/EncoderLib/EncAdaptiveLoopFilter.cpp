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
#if JVET_O_MAX_NUM_ALF_APS_8
  , m_apsIdStart( ALF_CTB_MAX_NUM_APS )
#else
  , m_apsIdStart( MAX_NUM_APS )
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    int numClasses = channelIdx ? MAX_NUM_ALF_ALTERNATIVES_CHROMA : MAX_NUM_ALF_CLASSES;
#else
    int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    m_ctuEnableFlagTmp2[compIdx] = new uint8_t[m_numCTUsInPic];
    if( isLuma( ComponentID(compIdx) ) )
    {
      m_ctuAlternativeTmp[compIdx] = nullptr;
    }
    else
    {
      m_ctuAlternativeTmp[compIdx] = new uint8_t[m_numCTUsInPic];
      std::fill_n( m_ctuAlternativeTmp[compIdx], m_numCTUsInPic, 0 );
    }
#endif
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

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  m_filterCoeffSet = new int*[std::max(MAX_NUM_ALF_CLASSES, MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
  m_filterClippSet = new int*[std::max(MAX_NUM_ALF_CLASSES, MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
#else
  m_filterCoeffSet = new int*[MAX_NUM_ALF_CLASSES];
  m_filterClippSet = new int*[MAX_NUM_ALF_CLASSES];
#endif
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];

  for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_filterClippSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }

#if !JVET_O1164_PS
#if JVET_O_MAX_NUM_ALF_APS_8
  m_apsIdStart = ALF_CTB_MAX_NUM_APS;
#else
  m_apsIdStart = (int)MAX_NUM_APS;
#endif
#endif

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

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    if( m_ctuEnableFlagTmp2[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp2[compIdx];
      m_ctuEnableFlagTmp2[compIdx] = nullptr;
    }

    if( m_ctuAlternativeTmp[compIdx] )
    {
      delete[] m_ctuAlternativeTmp[compIdx];
      m_ctuAlternativeTmp[compIdx] = nullptr;
    }

#endif
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
#if JVET_O_MAX_NUM_ALF_APS_8
    memset(cs.slice->getAlfAPSs(), 0, sizeof(*cs.slice->getAlfAPSs())*ALF_CTB_MAX_NUM_APS);
    m_apsIdStart = ALF_CTB_MAX_NUM_APS;
#else
    memset(cs.slice->getAlfAPSs(), 0, sizeof(*cs.slice->getAlfAPSs())*MAX_NUM_APS);
    m_apsIdStart = (int)MAX_NUM_APS;
#endif
    m_apsMap->clear();
#if JVET_O_MAX_NUM_ALF_APS_8
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
#else
    for (int i = 0; i < MAX_NUM_APS; i++)
#endif
    {
      APS* alfAPS = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsMap->clearChangedFlag((i << NUM_APS_TYPE_LEN) + ALF_APS);
      if (alfAPS)
      {
        alfAPS->getAlfAPSParam().reset();
        alfAPS = nullptr;
      }
    }
  }
  AlfParam alfParam;
  alfParam.reset();
  const TempCtx  ctxStart(m_CtxCache, AlfCtx(m_CABACEstimator->getCtx()));
  // set available filter shapes
  alfParam.filterShapes = m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU ALF enable flags, it was already reset before ALF process
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    m_ctuAlternative[compIdx] = cs.picture->getAlfCtuAlternativeData( compIdx );
#endif
  }

  // reset ALF parameters
  alfParam.reset();
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
#if !JVET_O0625_ALF_PADDING
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
#endif
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
#if JVET_O0625_ALF_PADDING
  int alfBryList[4] = { ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY }; // 0 - top, 1 - bottom, 2 - left, 3 - right.
#endif

  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
      const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
#if JVET_O0625_ALF_PADDING
      if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, alfBryList[0], alfBryList[1], alfBryList[2], alfBryList[3], numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS() ) )
#else
      if( isCrossedByVirtualBoundaries( xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS() ) )
#endif
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
#if JVET_O0625_ALF_PADDING
          const bool clipT = ( i == 0 && alfBryList[0] != ALF_NONE_BOUNDARY ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && alfBryList[1] != ALF_NONE_BOUNDARY ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
#else
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
#endif
          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
#if JVET_O0625_ALF_PADDING
            const bool clipL = ( j == 0 && alfBryList[2] != ALF_NONE_BOUNDARY ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && alfBryList[3] != ALF_NONE_BOUNDARY ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
#else
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
#endif
            const int wBuf = w + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
            const int hBuf = h + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
            PelUnitBuf buf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
            buf.copyFrom( recYuv.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf ) ) ) );
            buf.extendBorderPel( MAX_ALF_PADDING_SIZE );
            buf = buf.subBuf( UnitArea ( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

            const Area blkSrc( 0, 0, w, h );
            const Area blkDst( xStart, yStart, w, h );
#if JVET_O0625_ALF_PADDING
            deriveClassification( m_classifier, buf.get(COMPONENT_Y), blkDst, blkSrc, alfBryList );
#else
            deriveClassification( m_classifier, buf.get(COMPONENT_Y), blkDst, blkSrc );
#endif
#if !JVET_O0525_REMOVE_PCM
            Area blkPCM( xStart, yStart, w, h );
            resetPCMBlkClassInfo( cs, m_classifier, buf.get(COMPONENT_Y), blkPCM );
#endif

            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {
        Area blk( xPos, yPos, width, height );
#if JVET_O0625_ALF_PADDING
        deriveClassification( m_classifier, recLuma, blk, blk, alfBryList );
#else
        deriveClassification( m_classifier, recLuma, blk, blk );
#endif
#if !JVET_O0525_REMOVE_PCM
        Area blkPCM( xPos, yPos, width, height );
        resetPCMBlkClassInfo( cs, m_classifier, recLuma, blkPCM );
#endif
      }
    }
  }

  // get CTB stats for filtering
  deriveStatsForFiltering( orgYuv, recYuv, cs );

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    cs.slice->getPic()->getAlfCtbFilterIndex()[ctbIdx] = NUM_FIXED_FILTER_SETS;
  }
  // consider using new filter (only)
  alfParam.newFilterFlag[CHANNEL_TYPE_LUMA] = true;
  alfParam.newFilterFlag[CHANNEL_TYPE_CHROMA] = true;
  cs.slice->setTileGroupNumAps(1); // Only new filter for RD cost optimization
#endif
  // derive filter (luma)
  alfEncoder( cs, alfParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_LUMA
#if ENABLE_QPA
            , lambdaChromaWeight
#endif
            );

  // derive filter (chroma)
  {
    alfEncoder( cs, alfParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_CHROMA
#if ENABLE_QPA
              , lambdaChromaWeight
#endif
              );
  }

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  // let alfEncoderCtb decide now
  alfParam.newFilterFlag[CHANNEL_TYPE_LUMA] = false;
  alfParam.newFilterFlag[CHANNEL_TYPE_CHROMA] = false;
  cs.slice->setTileGroupNumAps(0);
#endif
  m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
  alfEncoderCtb(cs, alfParam
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  TempCtx        ctxTempAltStart( m_CtxCache );
  TempCtx        ctxTempAltBest( m_CtxCache );
#endif
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  const int numAlts = isLuma( channel ) ? 1 : m_alfParamTemp.numAlternativesChroma;
#endif

  double cost = 0;
  distUnfilter = 0;

  setEnableFlag(m_alfParamTemp, channel, true);
#if ENABLE_QPA
  CHECK ((chromaWeight > 0.0) && (cs.slice->getSliceCurStartCtuTsAddr() != 0), "incompatible start CTU address, must be 0");
#endif

  reconstructCoeff(m_alfParamTemp, channel, true, isLuma(channel));
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  for( int altIdx = 0; altIdx < (isLuma(channel) ? 1 : MAX_NUM_ALF_ALTERNATIVES_CHROMA); altIdx++)
  {
    for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
    {
      for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
      {
        m_filterCoeffSet[isLuma(channel) ? classIdx : altIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaCoeffFinal[altIdx][i];
        m_filterClippSet[isLuma(channel) ? classIdx : altIdx][i] = isLuma(channel) ? m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaClippFinal[altIdx][i];
      }
    }
  }
#else
  for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
  {
    for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
    {
      m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaCoeffFinal[i];
      m_filterClippSet[classIdx][i] = isLuma(channel) ? m_clippFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaClippFinal[i];
    }
  }
#endif

  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    for( int compID = compIDFirst; compID <= compIDLast; compID++ )
    {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
#if ENABLE_QPA
      const double ctuLambda = chromaWeight > 0.0 ? (isLuma (channel) ? cs.picture->m_uEnerHpCtu[ctuIdx] : cs.picture->m_uEnerHpCtu[ctuIdx] / chromaWeight) : m_lambda[compID];
#else
      const double ctuLambda = m_lambda[compID];
#endif
#endif

      double distUnfilterCtu = getUnfilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses );

      ctxTempStart = AlfCtx( m_CABACEstimator->getCtx() );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 1;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfParamTemp );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if( isLuma( channel ) )
      {
        // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
        assert( cs.slice->getPic()->getAlfCtbFilterIndex()[ctuIdx] == NUM_FIXED_FILTER_SETS );
        assert( cs.slice->getTileGroupNumAps() == 1 );
        m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctuIdx, &m_alfParamTemp.enabledFlag[COMPONENT_Y]);
      }
      double costOn = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

#else
      double costOn = distUnfilterCtu + getFilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfParamTemp.numLumaFilters - 1, numCoeff );
#if ENABLE_QPA
      const double ctuLambda = chromaWeight > 0.0 ? (isLuma (channel) ? cs.picture->m_uEnerHpCtu[ctuIdx] : cs.picture->m_uEnerHpCtu[ctuIdx] / chromaWeight) : m_lambda[compID];
#else
      const double ctuLambda = m_lambda[compID];
#endif
      costOn += ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif
      ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if( isLuma( channel ) )
      {
        costOn += getFilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfParamTemp.numLumaFilters - 1, numCoeff );
      }
      else
      {
        double bestAltCost = MAX_DOUBLE;
        int bestAltIdx = -1;
        ctxTempAltStart = AlfCtx( ctxTempBest );
        for( int altIdx = 0; altIdx < numAlts; ++altIdx )
        {
          if( altIdx )
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempAltStart );
          m_CABACEstimator->resetBits();
          m_ctuAlternative[compID][ctuIdx] = altIdx;
          m_CABACEstimator->codeAlfCtuAlternative( cs, ctuIdx, compID, &m_alfParamTemp );
          double r_altCost = ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

          double altDist = 0.;
          altDist += m_alfCovariance[compID][iShapeIdx][ctuIdx][0].calcErrorForCoeffs(  m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, m_NUM_BITS );

          double altCost = altDist + r_altCost;
          if( altCost < bestAltCost )
          {
            bestAltCost = altCost;
            bestAltIdx = altIdx;
            ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
          }
        }
        m_ctuAlternative[compID][ctuIdx] = bestAltIdx;
        costOn += bestAltCost;
      }
#endif

      m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 0;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfParamTemp);
      double costOff = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

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
    setEnableFlag(m_alfParamTemp, channel, m_ctuEnableFlag);
#if !JVET_O0491_HLS_CLEANUP
    const int alfChromaIdc = m_alfParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfParamTemp.enabledFlag[COMPONENT_Cr];
    cost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[channel];
#endif
  }

  return cost;
}

void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfParam& alfParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel
#if ENABLE_QPA
                                      , const double lambdaChromaWeight // = 0.0
#endif
                                      )
{
  const TempCtx  ctxStart( m_CtxCache, AlfCtx( m_CABACEstimator->getCtx() ) );
  TempCtx        ctxBest( m_CtxCache );

  double costMin = MAX_DOUBLE;

  std::vector<AlfFilterShape>& alfFilterShape = alfParam.filterShapes[channel];
  m_bitsNewFilter[channel] = 0;
  const int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;

  for( int iShapeIdx = 0; iShapeIdx < alfFilterShape.size(); iShapeIdx++ )
  {
    m_alfParamTemp = alfParam;
    //1. get unfiltered distortion
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    if( isChroma(channel) )
      m_alfParamTemp.numAlternativesChroma = 1;
#endif
    double cost = getUnfilteredDistortion( m_alfCovarianceFrame[channel][iShapeIdx], channel );
    cost /= 1.001; // slight preference for unfiltered choice

    if( cost < costMin )
    {
      costMin = cost;
      setEnableFlag( alfParam, channel, false );
      // no CABAC signalling
      ctxBest = AlfCtx( ctxStart );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 0 );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if( isChroma(channel) )
        setCtuAlternativeChroma( m_ctuAlternativeTmp, 0 );
#endif
    }

    const int nonLinearFlagMax =
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      ( isLuma( channel ) ? m_encCfg->getUseNonLinearAlfLuma() : 0 ) // For Chroma non linear flag is check for each alternative filter
#else
      ( isLuma( channel ) ? m_encCfg->getUseNonLinearAlfLuma() : m_encCfg->getUseNonLinearAlfChroma() )
#endif
      ? 2 : 1;

    for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
    {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    for( int numAlternatives = isLuma( channel ) ? 1 : getMaxNumAlternativesChroma(); numAlternatives > 0; numAlternatives-- )
    {
      if( isChroma( channel ) )
        m_alfParamTemp.numAlternativesChroma = numAlternatives;
#endif
      //2. all CTUs are on
      setEnableFlag( m_alfParamTemp, channel, true );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if( isLuma( channel ) )
        m_alfParamTemp.nonLinearFlag[channel][0] = nonLinearFlag;
#else
      m_alfParamTemp.nonLinearFlag[channel] = nonLinearFlag;
#endif
      m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
      setCtuEnableFlag( m_ctuEnableFlag, channel, 1 );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      // all alternatives are on
      if( isChroma( channel ) )
        initCtuAlternativeChroma( m_ctuAlternative );
      cost = getFilterCoeffAndCost( cs, 0, channel, true, iShapeIdx, uiCoeffBits );
#else
    cost = getFilterCoeffAndCost( cs, 0, channel, nonLinearFlag != 0, iShapeIdx, uiCoeffBits );
#endif

      if( cost < costMin )
      {
        m_bitsNewFilter[channel] = uiCoeffBits;
        costMin = cost;
        copyAlfParam( alfParam, m_alfParamTemp, channel );
        ctxBest = AlfCtx( m_CABACEstimator->getCtx() );
        setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 1 );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
        if( isChroma(channel) )
          copyCtuAlternativeChroma( m_ctuAlternativeTmp, m_ctuAlternative );
#endif
      }

      //3. CTU decision
      double distUnfilter = 0;
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      double prevItCost = MAX_DOUBLE;
#endif
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      const int iterNum = isLuma(channel) ? (2 * 4 + 1) : (2 * (2 + m_alfParamTemp.numAlternativesChroma - 1) + 1);
#else
    const int iterNum = isLuma(channel) ? (2 * 4 + 1) : (2 * 2 + 1);
#endif

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
                                          numClasses, alfFilterShape[iShapeIdx].numCoeff, distUnfilter);
          if (cost < costMin)
          {
            m_bitsNewFilter[channel] = uiCoeffBits;
            costMin = cost;
            ctxBest = AlfCtx(m_CABACEstimator->getCtx());
            copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
            if( isChroma(channel) )
              copyCtuAlternativeChroma( m_ctuAlternativeTmp, m_ctuAlternative );
#endif
            copyAlfParam(alfParam, m_alfParamTemp, channel);
          }
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
          else if ( cost >= prevItCost  )
          {
            // High probability that we have converged or we are diverging
            break;
          }
          prevItCost = cost;
#endif
        }
        else
        {
          // unfiltered distortion is added due to some CTBs may not use filter
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
          // no need to reset CABAC here, since uiCoeffBits is not affected
          /*cost = */getFilterCoeffAndCost( cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits );
#else
        cost = getFilterCoeffAndCost(cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits);
#endif
        }
      }//for iter
      // Decrease number of alternatives and reset ctu params and filters
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    }
#endif
    }// for nonLineaFlag
  }//for shapeIdx
  m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  if( isChroma(channel) )
    copyCtuAlternativeChroma( m_ctuAlternative, m_ctuAlternativeTmp );
  copyCtuEnableFlag( m_ctuEnableFlag, m_ctuEnableFlagTmp, channel );
#endif
}

void EncAdaptiveLoopFilter::copyAlfParam( AlfParam& alfParamDst, AlfParam& alfParamSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( &alfParamDst, &alfParamSrc, sizeof( AlfParam ) );
  }
  else
  {
#if !JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    alfParamDst.nonLinearFlag[channel] = alfParamSrc.nonLinearFlag[channel];
#endif
    alfParamDst.enabledFlag[COMPONENT_Cb] = alfParamSrc.enabledFlag[COMPONENT_Cb];
    alfParamDst.enabledFlag[COMPONENT_Cr] = alfParamSrc.enabledFlag[COMPONENT_Cr];
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    alfParamDst.numAlternativesChroma = alfParamSrc.numAlternativesChroma;
    memcpy( alfParamDst.nonLinearFlag[CHANNEL_TYPE_CHROMA], alfParamSrc.nonLinearFlag[CHANNEL_TYPE_CHROMA], sizeof( alfParamDst.nonLinearFlag[CHANNEL_TYPE_CHROMA] ) );
#endif
    memcpy( alfParamDst.chromaCoeff, alfParamSrc.chromaCoeff, sizeof( alfParamDst.chromaCoeff ) );
    memcpy( alfParamDst.chromaClipp, alfParamSrc.chromaClipp, sizeof( alfParamDst.chromaClipp ) );
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
#if !JVET_O0491_HLS_CLEANUP
  int uiSliceFlag = 0;
#endif
  AlfFilterShape& alfFilterShape = m_alfParamTemp.filterShapes[channel][iShapeIdx];
  //get filter coeff
  if( isLuma( channel ) )
  {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    std::fill_n(m_alfClipMerged[iShapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfParamTemp.nonLinearFlag[channel][0] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
#else
    std::fill_n(m_alfClipMerged[iShapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
#endif
    // Reset Merge Tmp Cov
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES].reset(AlfNumClippingValues[channel]);
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES + 1].reset(AlfNumClippingValues[channel]);
    //distortion
    dist += mergeFiltersAndCost( m_alfParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], m_alfClipMerged[iShapeIdx], uiCoeffBits );
  }
  else
  {
    //distortion
#if !JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][iShapeIdx][0].numCoeff);
    std::fill_n(m_filterClippSet[0], MAX_NUM_ALF_CHROMA_COEFF, m_alfParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues[CHANNEL_TYPE_CHROMA] / 2 : 0);
    dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterClippSet[0], m_filterCoeffSet[0], m_alfCovarianceFrame[channel][iShapeIdx][0], alfFilterShape, m_NUM_BITS, m_alfParamTemp.nonLinearFlag[channel] );
#endif
#if !JVET_O0491_HLS_CLEANUP
    //setEnableFlag( m_alfParamTemp, channel, m_ctuEnableFlag );
    const int alfChromaIdc = m_alfParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfParamTemp.enabledFlag[COMPONENT_Cr];
#endif
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    for( int altIdx = 0; altIdx < m_alfParamTemp.numAlternativesChroma; ++altIdx )
    {
      assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][iShapeIdx][altIdx].numCoeff);
      AlfParam bestSliceParam;
      double bestCost = MAX_DOUBLE;
      double bestDist = MAX_DOUBLE;
      int bestCoeffBits = 0;
      const int nonLinearFlagMax = m_encCfg->getUseNonLinearAlfChroma() ? 2 : 1;

      for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
      {
        m_alfParamTemp.nonLinearFlag[channel][altIdx] = nonLinearFlag;

        std::fill_n(m_filterClippSet[altIdx], MAX_NUM_ALF_CHROMA_COEFF, nonLinearFlag ? AlfNumClippingValues[CHANNEL_TYPE_CHROMA] / 2 : 0 );
        double dist = m_alfCovarianceFrame[channel][iShapeIdx][altIdx].pixAcc + deriveCoeffQuant( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], m_alfCovarianceFrame[channel][iShapeIdx][altIdx], alfFilterShape, m_NUM_BITS, nonLinearFlag );
        for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
        {
          m_alfParamTemp.chromaCoeff[altIdx][i] = m_filterCoeffSet[altIdx][i];
          m_alfParamTemp.chromaClipp[altIdx][i] = m_filterClippSet[altIdx][i];
        }
        int coeffBits = getChromaCoeffRate( m_alfParamTemp, altIdx );
        double cost = dist + m_lambda[channel] * coeffBits;
        if( cost < bestCost )
        {
          bestCost = cost;
          bestDist = dist;
          bestCoeffBits = coeffBits;
          bestSliceParam = m_alfParamTemp;
        }
      }
      uiCoeffBits += bestCoeffBits;
      dist += bestDist;
      m_alfParamTemp = bestSliceParam;
    }
    uiCoeffBits += lengthUvlc( m_alfParamTemp.numAlternativesChroma-1 );
    uiCoeffBits += m_alfParamTemp.numAlternativesChroma; // non-linear flags
#if !JVET_O0491_HLS_CLEANUP
    uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3)
                    - lengthTruncatedUnary( 0, 3 );  // rate already put on Luma
#endif
#else
    for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
    {
      m_alfParamTemp.chromaCoeff[i] = m_filterCoeffSet[0][i];
      m_alfParamTemp.chromaClipp[i] = m_filterClippSet[0][i];
    }
    uiCoeffBits += getCoeffRate( m_alfParamTemp, true );
    uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
#endif
  }
  if (onlyFilterCost)
  {
    return dist + m_lambda[channel] * uiCoeffBits;
  }
#if !JVET_O0491_HLS_CLEANUP
  double rate = uiCoeffBits + uiSliceFlag;
#else
  double rate = uiCoeffBits;
#endif
  m_CABACEstimator->resetBits();
  m_CABACEstimator->codeAlfCtuEnableFlags( cs, channel, &m_alfParamTemp);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    if( isLuma( channel ) )
    {
      // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
      assert( cs.slice->getPic()->getAlfCtbFilterIndex()[ctuIdx] == NUM_FIXED_FILTER_SETS );
      assert( cs.slice->getTileGroupNumAps() == 1 );
      m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctuIdx, &m_alfParamTemp.enabledFlag[COMPONENT_Y]);
    }
  }
  m_CABACEstimator->codeAlfCtuAlternatives( cs, channel, &m_alfParamTemp );
#endif
  rate += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[channel] * rate;
}

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
int EncAdaptiveLoopFilter::getChromaCoeffRate( AlfParam& alfParam, int altIdx )
#else
int EncAdaptiveLoopFilter::getCoeffRate( AlfParam& alfParam, bool isChroma )
#endif
{
  int iBits = 0;
#if !JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  assert( isChroma );
#endif

  AlfFilterShape alfShape(5);
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  const int numFilters = 1;
#endif
#if !JVET_O0216_ALF_COEFF_EG3
  // vlc for all
  for( int i = 0; i < alfShape.numCoeff - 1; i++ )
  {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    int coeffVal = abs( alfParam.chromaCoeff[altIdx][i] );
#else
    int coeffVal = abs( alfParam.chromaCoeff[i] );
#endif

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
#endif
  // Filter coefficients
  for( int i = 0; i < alfShape.numCoeff - 1; i++ )
  {
#if JVET_O0216_ALF_COEFF_EG3
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    iBits += lengthGolomb( alfParam.chromaCoeff[altIdx][i], 3 );  // alf_coeff_chroma[altIdx][i]
#else
    iBits += lengthGolomb( alfParam.chromaCoeff[i], 3 );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#endif
#else
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    iBits += lengthGolomb( alfParam.chromaCoeff[altIdx][i], m_kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[altIdx][i]
#else
    iBits += lengthGolomb( alfParam.chromaCoeff[i], m_kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#endif
#endif
  }

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx] )
#else
  if( m_alfParamTemp.nonLinearFlag[isChroma] )
#endif
  {
#if JVET_O0064_SIMP_ALF_CLIP_CODING
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if( !abs( alfParam.chromaCoeff[altIdx][i] ) )
#else
      if( !abs( alfParam.chromaCoeff[i] ) )
#endif
      {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
        alfParam.chromaClipp[altIdx][i] = 0;
#else
        alfParam.chromaClipp[i] = 0;
#endif
      }
    }
    iBits += ((alfShape.numCoeff - 1) << 1);
#else
    memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
    // vlc for all
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if( !abs( alfParam.chromaCoeff[altIdx][i] ) )
#else
      if( !abs( alfParam.chromaCoeff[i] ) )
#endif
        continue;
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      int coeffVal = abs( alfParam.chromaClipp[altIdx][i] );
#else
      int coeffVal = abs( alfParam.chromaClipp[i] );
#endif

      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k, false );
      }
    }
#if JVET_O0216_ALF_COEFF_EG3
    int kMin = getGolombKMin(alfShape, numFilters, m_kMinTab, m_bitsCoeffScan);
#else
    kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );
#endif

    // Golomb parameters
    iBits += lengthUvlc( kMin - 1 );  // "min_golomb_order"
#if JVET_O0216_ALF_COEFF_EG3
    int golombOrderIncreaseFlag = 0;
#else
    golombOrderIncreaseFlag = 0;
#endif

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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      if( !abs( alfParam.chromaCoeff[altIdx][i] ) )
#else
      if( !abs( alfParam.chromaCoeff[i] ) )
#endif
        continue;
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      iBits += lengthGolomb( alfParam.chromaClipp[altIdx][i], m_kMinTab[alfShape.golombIdx[i]], false );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#else
      iBits += lengthGolomb( alfParam.chromaClipp[i], m_kMinTab[alfShape.golombIdx[i]], false );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#endif
    }
#endif
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
#if !JVET_O0491_HLS_CLEANUP
    dist = getUnfilteredDistortion( cov, 1 ) + lengthTruncatedUnary( 0, 3 ) * m_lambda[COMPONENT_Cb];
#else
    dist = getUnfilteredDistortion( cov, 1 );
#endif
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

double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits )
{
  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  static bool codedVarBins[MAX_NUM_ALF_CLASSES];
  static double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = MAX_DOUBLE;
#if JVET_O0669_REMOVE_ALF_COEFF_PRED
  int coeffBits, coeffBitsForce0;
#else
  int predMode = 0, bestPredMode = 0, coeffBits, coeffBitsForce0;
#endif

  mergeClasses( alfShape, covFrame, covMerged, clipMerged, MAX_NUM_ALF_CLASSES, m_filterIndices );

  while( numFilters >= 1 )
  {
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfParam);
    // filter coeffs are stored in m_filterCoeffSet
    distForce0 = getDistForce0( alfShape, numFilters, errorForce0CoeffTab, codedVarBins );
#if JVET_O0669_REMOVE_ALF_COEFF_PRED
    coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters );
#else
    coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters, predMode );
#endif
    coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFilters, codedVarBins );

    cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

    if( cost0 < cost )
    {
      cost = cost0;
    }
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    if (alfParam.fixedFilterSetIndex > 0)
    {
      int len = 0;
      len += getTBlength(alfParam.fixedFilterSetIndex - 1, NUM_FIXED_FILTER_SETS);
      len += 1; //fixed filter flag pattern
      if (alfParam.fixedFilterPattern > 0)
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
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
      bestPredMode = predMode;
#endif
    }
    numFilters--;
  }

  dist = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab, alfParam );
#if JVET_O0669_REMOVE_ALF_COEFF_PRED
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest );
#else
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest, predMode );
#endif
  distForce0 = getDistForce0( alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins );
  coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins );

  cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

  alfParam.numLumaFilters = numFiltersBest;
  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfParam.alfLumaCoeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    alfParam.alfLumaCoeffDeltaPredictionFlag = bestPredMode;
#endif
  }
  else
  {
    distReturn = distForce0;
    alfParam.alfLumaCoeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    memcpy( alfParam.alfLumaCoeffFlag, codedVarBins, sizeof( codedVarBins ) );
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
    alfParam.alfLumaCoeffDeltaPredictionFlag = 0;
#endif

    for( int varInd = 0; varInd < numFiltersBest; varInd++ )
    {
      if( codedVarBins[varInd] == 0 )
      {
        memset( m_filterCoeffSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
        memset( m_filterClippSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
      }
    }
  }

  for( int ind = 0; ind < alfParam.numLumaFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff; i++ )
    {
#if JVET_O0669_REMOVE_ALF_COEFF_PRED
      alfParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
#else
      if( alfParam.alfLumaCoeffDeltaPredictionFlag )
      {
        alfParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_diffFilterCoeff[ind][i];
      }
      else
      {
        alfParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      }
#endif
      alfParam.lumaClipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterClippSet[ind][i];
    }
  }

  memcpy( alfParam.filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof( short ) * MAX_NUM_ALF_CLASSES );
  uiCoeffBits += getNonFilterCoeffRate( alfParam );
  return distReturn;
}

int EncAdaptiveLoopFilter::getNonFilterCoeffRate( AlfParam& alfParam )
{
  int len = 1   // alf_coefficients_delta_flag
#if !JVET_O0491_HLS_CLEANUP
          + lengthTruncatedUnary( 0, 3 )    // chroma_idc = 0, it is signalled when ALF is enabled for luma
          + getTBlength( alfParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES );   //numLumaFilters
#else
          + 2                                          // slice_alf_chroma_idc                     u(2)
          + lengthUvlc (alfParam.numLumaFilters - 1);  // alf_luma_num_filters_signalled_minus1   ue(v)
#endif

  if( alfParam.numLumaFilters > 1 )
  {
#if JVET_O0491_HLS_CLEANUP
    const int coeffLength = ceilLog2(alfParam.numLumaFilters);
#endif
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
#if !JVET_O0491_HLS_CLEANUP
      len += getTBlength( (int)alfParam.filterCoeffDeltaIdx[i], alfParam.numLumaFilters );  //filter_coeff_delta[i]
#else
      len += coeffLength;                              // alf_luma_coeff_delta_idx   u(v)
#endif
    }
  }
#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
  len++; //fixed filter set flag
  if (alfParam.fixedFilterSetIndex > 0)
  {
    len += getTBlength(alfParam.fixedFilterSetIndex - 1, NUM_FIXED_FILTER_SETS);
    len += 1; //fixed filter flag pattern
    if (alfParam.fixedFilterPattern > 0)
      len += MAX_NUM_ALF_CLASSES;  //"fixed_filter_flag" for each class
  }
#endif
  return len;
}

#if !JVET_O0491_HLS_CLEANUP
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
#endif

int EncAdaptiveLoopFilter::getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins )
{
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );
  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
#endif
#if JVET_O0216_ALF_COEFF_EG3
  int len = numFilters; //filter_coefficient_flag[i]
#else
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
#endif
  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( codedVarBins[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
#if JVET_O0216_ALF_COEFF_EG3
        len += lengthGolomb( abs( pDiffQFilterCoeffIntPP[ind][i] ), 3 ); // alf_coeff_luma_delta[i][j]
#else
        len += lengthGolomb( abs( pDiffQFilterCoeffIntPP[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] ); // alf_coeff_luma_delta[i][j]
#endif
      }
    }
  }

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] )
#else
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
#endif
  {
#if JVET_O0064_SIMP_ALF_CLIP_CODING
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(pDiffQFilterCoeffIntPP[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
        len += 2;
      }
    }
#else
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
#if JVET_O0216_ALF_COEFF_EG3
    int kMin = getGolombKMin(alfShape, numFilters, m_kMinTab, m_bitsCoeffScan);
#else
    kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );
#endif

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
#endif
  }

  return len;
}

#if JVET_O0669_REMOVE_ALF_COEFF_PRED
int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters )
{
  return (m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? getCostFilterClipp(alfShape, filterSet, numFilters) : 0) + getCostFilterCoeff(alfShape, filterSet, numFilters);
#else
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

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  int rateClipp = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] ? getCostFilterClipp( alfShape, filterSet, numFilters ) : 0;
#else
  int rateClipp = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? getCostFilterClipp( alfShape, filterSet, numFilters ) : 0;
#endif

  return ( numFilters > 1 ? 1 : 0 )        // coeff_delta_pred_mode_flag
       + rateClipp
       + ( predMode ? ratePredMode1 : ratePredMode0 ); // min_golomb_order, golomb_order_increase_flag, alf_coeff_luma_delta
#endif
}

int EncAdaptiveLoopFilter::getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
#if JVET_O0216_ALF_COEFF_EG3
  return lengthFilterCoeffs( alfShape, numFilters, pDiffQFilterCoeffIntPP );  // alf_coeff_luma_delta[i][j];
#else
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
#endif
}

int EncAdaptiveLoopFilter::getCostFilterClipp( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
#if JVET_O0064_SIMP_ALF_CLIP_CODING
  for (int filterIdx = 0; filterIdx < numFilters; ++filterIdx)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      if (!abs(pDiffQFilterCoeffIntPP[filterIdx][i]))
      {
        m_filterClippSet[filterIdx][i] = 0;
      }
    }
  }
  return (numFilters * (alfShape.numCoeff - 1)) << 1;
#else
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
#endif
}

#if JVET_O0216_ALF_COEFF_EG3
int EncAdaptiveLoopFilter::lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff )
#else
int EncAdaptiveLoopFilter::lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab )
#endif
{
  int bitCnt = 0;

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
#if JVET_O0216_ALF_COEFF_EG3
      bitCnt += lengthGolomb( abs( FilterCoeff[ind][i] ), 3 );
#else
      bitCnt += lengthGolomb( abs( FilterCoeff[ind][i] ), kMinTab[alfShape.golombIdx[i]] );
#endif
    }
  }
  return bitCnt;
}

#if !JVET_O0064_SIMP_ALF_CLIP_CODING
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
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
#endif
#if !JVET_O0216_ALF_COEFF_EG3
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
#endif

  for( int ind = 0; ind < numFilters; ++ind )
  {
    bitsVarBin[ind] = 0;
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
#if JVET_O0216_ALF_COEFF_EG3
      bitsVarBin[ind] += lengthGolomb( abs( m_filterCoeffSet[ind][i] ), 3 );
#else
      bitsVarBin[ind] += lengthGolomb( abs( m_filterCoeffSet[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] );
#endif
    }
  }

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] )
#else
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
#endif
#if JVET_O0064_SIMP_ALF_CLIP_CODING
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(m_filterCoeffSet[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
      }
    }
  }
#else
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
#if !JVET_O0216_ALF_COEFF_EG3 || !JVET_O0064_SIMP_ALF_CLIP_CODING
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
#endif
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
  int numBins = 0;
  unsigned int symbol = abs(coeffVal);
  while (symbol >= (unsigned int)(1 << k))
  {
    numBins++;
    symbol -= 1 << k;
    k++;
  }
  numBins += ( k + 1) ;
  if (signed_coeff && coeffVal != 0)
  {
    numBins++;
  }
  return numBins;
}

double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam )
{
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

#if !JVET_O0669_REMOVE_ALF_COEFF_PRED
  alfParam.fixedFilterSetIndex = 0;
  AlfCovariance& tmpCovFf = covMerged[MAX_NUM_ALF_CLASSES + 1];
  double factor = 1 << (m_NUM_BITS - 1);
  double errorMin = 0;
  double errorMinPerClass[MAX_NUM_ALF_CLASSES] = { 0 };
  double errorCurSetPerClass[MAX_NUM_ALF_CLASSES] = { 0 };
  int    fixedFilterFlagPerClass[MAX_NUM_ALF_CLASSES] = { 0 };
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  if (!alfParam.nonLinearFlag[CHANNEL_TYPE_LUMA][0])
#else
  if (!alfParam.nonLinearFlag[CHANNEL_TYPE_LUMA])
#endif
  {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
#else
    alfParam.fixedFilterSetIndex = 0;
#endif
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
        memcpy(alfParam.fixedFilterIdx, fixedFilterFlagPerClass, sizeof(fixedFilterFlagPerClass));
        alfParam.fixedFilterSetIndex = filterSetIdx + 1;
        errorMin = errorCur;
        std::memcpy(errorMinPerClass, errorCurSetPerClass, sizeof(errorMinPerClass));
      }
    }

    alfParam.fixedFilterPattern = 0;
    if (alfParam.fixedFilterSetIndex > 0)
    {
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
        if (alfParam.fixedFilterIdx[classIdx] == 0)
        {
          alfParam.fixedFilterPattern = 1;
          break;
        }
      }
    }
  }
#endif


  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    tmpCov.reset();
    bool found_clip = false;
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      if( filterIndices[classIdx] == filtIdx )
      {
#if JVET_O0669_REMOVE_ALF_COEFF_PRED
        tmpCov += cov[classIdx];
#else
        //adjust stat
        tmpCovFf = cov[classIdx];
        if (alfParam.fixedFilterSetIndex > 0 && alfParam.fixedFilterIdx[classIdx] > 0
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
          && alfParam.nonLinearFlag[CHANNEL_TYPE_LUMA][0] == false
#else
          && alfParam.nonLinearFlag[CHANNEL_TYPE_LUMA] == false
#endif
          )
        {
          int fixedFilterIdx = m_classToFilterMapping[alfParam.fixedFilterSetIndex - 1][classIdx];
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
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    covMerged[i].numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#else
    covMerged[i].numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#endif
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  tmpCov.numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#else
  tmpCov.numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#endif

  // init Clip
  for( int i = 0; i < numClasses; i++ )
  {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    std::fill_n(clipMerged[numRemaining-1][i], MAX_NUM_ALF_LUMA_COEFF, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    if ( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] )
#else
    std::fill_n(clipMerged[numRemaining-1][i], MAX_NUM_ALF_LUMA_COEFF, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    if ( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
            double errorMerged = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] ? tmpCov.optimizeFilterClip( alfShape, tmpClip ) : tmpCov.calculateError( tmpClip );
#else
            double errorMerged = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? tmpCov.optimizeFilterClip( alfShape, tmpClip ) : tmpCov.calculateError( tmpClip );
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  int numAlternatives = isLuma( channel ) ? 1 : m_alfParamTemp.numAlternativesChroma;
  // When calling this function m_ctuEnableFlag shall be set to 0 for CTUs using alternative APS
  // Here we compute frame stats for building new alternative filters
  for( int altIdx = 0; altIdx < numAlternatives; ++altIdx )
  {
    for( int i = 0; i < numClasses; i++ )
    {
      m_alfCovarianceFrame[channel][iShapeIdx][isLuma( channel ) ? i : altIdx].reset(AlfNumClippingValues[channel]);
    }
    if( isLuma( channel ) )
    {
      getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], nullptr, numClasses, altIdx );
    }
    else
    {
      getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], m_ctuAlternative[COMPONENT_Cb], numClasses, altIdx );
      getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], m_ctuAlternative[COMPONENT_Cr], numClasses, altIdx );
    }
  }
#else
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
#endif
}

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx )
#else
void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses )
#endif
{
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  const ChannelType channel = (!ctbAltIdx ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA);
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    if( ctbEnableFlags[ctuIdx]  )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        if( isLuma( channel ) || altIdx == ctbAltIdx[ctuIdx] )
        {
          frameCov[isLuma( channel ) ? classIdx : altIdx] += ctbCov[ctuIdx][classIdx];
        }
      }
    }
  }
#else
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
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    const int numAlts = channelID == CHANNEL_TYPE_LUMA ? 1 : MAX_NUM_ALF_ALTERNATIVES_CHROMA;
#endif
    const int numClasses = isLuma( channelID ) ? MAX_NUM_ALF_CLASSES : 1;

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    for( int altIdx = 0; altIdx < numAlts; ++altIdx )
#endif
    for( int shape = 0; shape != m_filterShapes[channelIdx].size(); shape++ )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
        m_alfCovarianceFrame[channelIdx][shape][isLuma( channelID ) ? classIdx : altIdx].reset(AlfNumClippingValues[channelID]);
#else
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset(AlfNumClippingValues[channelID]);
#endif
      }
    }
  }

  const PreCalcValues& pcv = *cs.pcv;
#if !JVET_O0625_ALF_PADDING
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
#endif
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
#if JVET_O0625_ALF_PADDING
  int alfBryList[4] = { ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY }; // 0 - top, 1 - bottom, 2 - left, 3 - right.
#endif

  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
      const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
#if JVET_O0625_ALF_PADDING
      if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, alfBryList[0], alfBryList[1], alfBryList[2], alfBryList[3], numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS() ) )
#else
      if( isCrossedByVirtualBoundaries( xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS() ) )
#endif
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
#if JVET_O0625_ALF_PADDING
          const bool clipT = ( i == 0 && alfBryList[0] != ALF_NONE_BOUNDARY ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && alfBryList[1] != ALF_NONE_BOUNDARY ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
#else
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
#endif
          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
#if JVET_O0625_ALF_PADDING
            const bool clipL = ( j == 0 && alfBryList[2] != ALF_NONE_BOUNDARY ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && alfBryList[3] != ALF_NONE_BOUNDARY ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
            int alfBryListChroma[4];
#else
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
#endif
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
#if JVET_O0625_ALF_PADDING
              alfBryListChroma[0] = alfBryList[0] != ALF_NONE_BOUNDARY ? alfBryList[0] >> ::getComponentScaleY( compID, m_chromaFormat ) : ALF_NONE_BOUNDARY;
              alfBryListChroma[1] = alfBryList[1] != ALF_NONE_BOUNDARY ? alfBryList[1] >> ::getComponentScaleY( compID, m_chromaFormat ) : ALF_NONE_BOUNDARY;
              alfBryListChroma[2] = alfBryList[2] != ALF_NONE_BOUNDARY ? alfBryList[2] >> ::getComponentScaleX( compID, m_chromaFormat ) : ALF_NONE_BOUNDARY;
              alfBryListChroma[3] = alfBryList[3] != ALF_NONE_BOUNDARY ? alfBryList[3] >> ::getComponentScaleX( compID, m_chromaFormat ) : ALF_NONE_BOUNDARY;
#endif
              ChannelType chType = toChannelType( compID );

              for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
              {
              const CompArea& compAreaDst = areaDst.block( compID );
                getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compAreaDst, compArea, chType
                  , ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight)
                  , ((yPos + m_maxCUHeight >= m_picHeight) ? m_picHeight : ((compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos))
#if JVET_O0625_ALF_PADDING
                  , compIdx ? alfBryListChroma : alfBryList
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
              m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#else
              m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#endif
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
#if JVET_O0625_ALF_PADDING
            , alfBryList
#endif
          );


          const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

          for( int classIdx = 0; classIdx < numClasses; classIdx++ )
          {
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
            m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#else
            m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#endif
          }
        }
      }
      }
      ctuRsAddr++;
    }
  }
}

#if JVET_O0625_ALF_PADDING
void EncAdaptiveLoopFilter::getBlkStats( AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, 
  Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos, const int alfBryList[4] )
#else
void EncAdaptiveLoopFilter::getBlkStats(AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos)
#endif


{
  static int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues];

  const int numBins = AlfNumClippingValues[channel];
#if JVET_O0625_ALF_PADDING
  const int chromaScaleY = getComponentScaleY( channel == CHANNEL_TYPE_LUMA ? COMPONENT_Y : COMPONENT_Cb, area.chromaFormat );
  const int vbHeight = 4 >> chromaScaleY;
  const int vbHalfHeight = vbHeight >> 1;
  int alfBryDist[4] = { ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY };
#endif
  int transposeIdx = 0;
  int classIdx = 0;

  for( int i = 0; i < area.height; i++ )
  {
    int vbDistance = ((areaDst.y + i) % vbCTUHeight) - vbPos;
#if JVET_O0625_ALF_PADDING
    alfBryDist[0] = alfBryList[0] != ALF_NONE_BOUNDARY ? ( i + areaDst.y - alfBryList[0] ) : ALF_NONE_BOUNDARY;
    alfBryDist[1] = alfBryList[1] != ALF_NONE_BOUNDARY ? ( i + areaDst.y - alfBryList[1] ) : ALF_NONE_BOUNDARY;

    if ( vbDistance >= 0 && vbDistance < vbHeight && alfBryDist[1] >= -vbHeight && alfBryDist[1] < 0 )
    { 
      /* between bottom boundary and ALF virtual boundary */
      if ( alfBryDist[1] < -vbHalfHeight )
      {
        alfBryDist[1] = ALF_NONE_BOUNDARY; /* closer to bottom boundary */
      }
      else
      {
        vbDistance = ALF_NONE_BOUNDARY;   /* closer to ALF virtual boundary */
      }
    }
#endif
    for( int j = 0; j < area.width; j++ )
    {
      if( classifier && classifier[areaDst.y + i][areaDst.x + j].classIdx == m_ALF_UNUSED_CLASSIDX && classifier[areaDst.y + i][areaDst.x + j].transposeIdx == m_ALF_UNUSED_TRANSPOSIDX )
      {
        continue;
      }
#if JVET_O0625_ALF_PADDING
      alfBryDist[2] = alfBryList[2] != ALF_NONE_BOUNDARY ? ( j + areaDst.x - alfBryList[2] ) : ALF_NONE_BOUNDARY;
      alfBryDist[3] = alfBryList[3] != ALF_NONE_BOUNDARY ? ( j + areaDst.x - alfBryList[3] ) : ALF_NONE_BOUNDARY;
#endif
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
#if JVET_O0625_ALF_PADDING
      calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel, vbDistance, alfBryDist );
#else
      calcCovariance(ELocal, rec + j, recStride, shape, transposeIdx, channel, vbDistance);
#endif
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

#if JVET_O0625_ALF_PADDING
void EncAdaptiveLoopFilter::calcCovariance( int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, 
  const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance, const int alfBryDist[4] )
#else
void EncAdaptiveLoopFilter::calcCovariance(int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance)
#endif
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
#if JVET_O0625_ALF_PADDING
  else if ( alfBryDist[0] >= 0 && alfBryDist[0] < 3 )
  {
    clipTopRow = -alfBryDist[0];
    clipBotRow = -clipTopRow;
  }
  else if ( alfBryDist[1] >= -3 && alfBryDist[1] < 0 )
  {
    clipBotRow = -alfBryDist[1] - 1;
    clipTopRow = -clipBotRow;
  }

  int clipLeft  = -4;
  int clipRight = 4;

  if ( alfBryDist[2] >= 0 && alfBryDist[2] < 3 )
  {
    clipLeft  = -alfBryDist[2];
    clipRight = -clipLeft;
  }
  else if ( alfBryDist[3] >= -3 && alfBryDist[3] < 0 )
  {
    clipRight = -alfBryDist[3] - 1;
    clipLeft  = -clipRight;
  }
#endif
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
#if JVET_O0625_ALF_PADDING
        int clipj = Clip3( clipLeft, clipRight, j );
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF( clip[b], curr, rec0[clipj], rec1[-clipj] );
        }
#else
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
#endif
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
#if JVET_O0625_ALF_PADDING
      int clipj = Clip3( clipLeft, clipRight, j );
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF( clip[b], curr, rec[clipj], rec[-clipj] );
      }
#else
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
#endif
    }
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
#if JVET_O0625_ALF_PADDING
      int clipj = Clip3( clipLeft, clipRight, j );
      const Pel* rec0 = rec + clipj;
      const Pel* rec1 = rec - clipj;
#else
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
#endif
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
#if JVET_O0625_ALF_PADDING
        int clipj = Clip3( clipLeft, clipRight, j );
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF( clip[b], curr, rec0[clipj], rec1[-clipj] );
        }
#else
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
#endif
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
#if JVET_O0625_ALF_PADDING
      int clipj = Clip3( clipLeft, clipRight, j );
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF( clip[b], curr, rec[clipj], rec[-clipj] );
      }
#else
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
#endif
    }
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
#if JVET_O0625_ALF_PADDING
      int clipj = Clip3( clipLeft, clipRight, j );
      const Pel* rec0 = rec + clipj;
      const Pel* rec1 = rec - clipj;
#else
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
#endif
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



void EncAdaptiveLoopFilter::setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, bool val )
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

void EncAdaptiveLoopFilter::setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags )
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
#if JVET_O_MAX_NUM_ALF_APS_8
  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
#else
  for (int i = 0; i < MAX_NUM_APS; i++)
#endif
  {
    apss[i] = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
  }

  std::vector<int> result;
  int apsIdChecked = 0, curApsId = m_apsIdStart;
#if JVET_O_MAX_NUM_ALF_APS_8
  if (curApsId < ALF_CTB_MAX_NUM_APS)
#else
  if (curApsId < int(MAX_NUM_APS))
#endif
  {
#if JVET_O_MAX_NUM_ALF_APS_8
    while (apsIdChecked < ALF_CTB_MAX_NUM_APS && !cs.slice->isIntra() && result.size() < ALF_CTB_MAX_NUM_APS && !cs.slice->getPendingRasInit() && !cs.slice->isIDRorBLA())
#else
    while (apsIdChecked < MAX_NUM_APS && !cs.slice->isIntra() && result.size() < (ALF_CTB_MAX_NUM_APS - 1) && !cs.slice->getPendingRasInit() && !cs.slice->isIDRorBLA())
#endif
    {
      APS* curAPS = cs.slice->getAlfAPSs()[curApsId];
      if (curAPS && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA])
      {
        result.push_back(curApsId);
      }
      apsIdChecked++;
#if JVET_O_MAX_NUM_ALF_APS_8
      curApsId = (curApsId + 1) % ALF_CTB_MAX_NUM_APS;
#else
      curApsId = (curApsId + 1) % MAX_NUM_APS;
#endif
    }
  }
  cs.slice->setTileGroupNumAps((int)result.size());
  cs.slice->setAlfAPSs(result);
  newApsId = m_apsIdStart - 1;
  if (newApsId < 0)
  {
#if JVET_O_MAX_NUM_ALF_APS_8
    newApsId = ALF_CTB_MAX_NUM_APS - 1;
#else
    newApsId = (int)MAX_NUM_APS - 1;
#endif
  }
#if JVET_O_MAX_NUM_ALF_APS_8
  CHECK(newApsId >= ALF_CTB_MAX_NUM_APS, "Wrong APS index assignment in getAvaiApsIdsLuma");
#else
  CHECK(newApsId >= (int)MAX_NUM_APS, "Wrong APS index assignment in getAvaiApsIdsLuma");
#endif
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
void  EncAdaptiveLoopFilter::alfEncoderCtb(CodingStructure& cs, AlfParam& alfParamNewFilters
#if ENABLE_QPA
  , const double lambdaChromaWeight
#endif
)
{
  TempCtx        ctxStart(m_CtxCache, AlfCtx(m_CABACEstimator->getCtx()));
  TempCtx        ctxBest(m_CtxCache);
  TempCtx        ctxTempStart(m_CtxCache);
  TempCtx        ctxTempBest(m_CtxCache);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  TempCtx        ctxTempAltStart( m_CtxCache );
  TempCtx        ctxTempAltBest( m_CtxCache );
#endif
  AlfParam  alfParamNewFiltersBest = alfParamNewFilters;
  APS**          apss = cs.slice->getAlfAPSs();
  short*     alfCtbFilterSetIndex = cs.picture->getAlfCtbFilterIndex();
  bool     hasNewFilters[2] = { alfParamNewFilters.enabledFlag[COMPONENT_Y] , alfParamNewFilters.enabledFlag[COMPONENT_Cb] || alfParamNewFilters.enabledFlag[COMPONENT_Cr] };
  initDistortion();

  //luma
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  m_alfParamTemp = alfParamNewFilters;
#endif
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
        reconstructCoeff(alfParamNewFilters, CHANNEL_TYPE_LUMA, true, true);
      }
    }
    int numIter = useNewFilter ? 2 : 1;
    for (int numTemporalAps = 0; numTemporalAps <= apsIds.size(); numTemporalAps++)
    {
#if JVET_O_MAX_NUM_ALF_APS_8
      if (numTemporalAps + useNewFilter >= ALF_CTB_MAX_NUM_APS)
      {
        continue;
      }
#endif
      cs.slice->setTileGroupNumAps(numTemporalAps + useNewFilter);
      int numFilterSet = NUM_FIXED_FILTER_SETS + numTemporalAps + useNewFilter;
      if (numTemporalAps == apsIds.size() && numTemporalAps > 0 && useNewFilter && newApsId == apsIds.back()) //last temporalAPS is occupied by new filter set and this temporal APS becomes unavailable
      {
        continue;
      }
      for (int iter = 0; iter < numIter; iter++)
      {
        m_alfParamTemp = alfParamNewFilters;
        m_alfParamTemp.enabledFlag[CHANNEL_TYPE_LUMA] = true;
#if JVET_O_MAX_NUM_ALF_APS_8
        double curCost = 3 * m_lambda[CHANNEL_TYPE_LUMA];
#else
        double curCost = getTBlength(numTemporalAps + useNewFilter, ALF_CTB_MAX_NUM_APS + 1) * m_lambda[CHANNEL_TYPE_LUMA];
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
            m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] = 1;
#else
            m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 1;
#endif
            if (m_encCfg->getUseNonLinearAlfLuma())
            {
              errNL[1] = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, 0, bitNL[1], true);
              m_alfParamTempNL = m_alfParamTemp;
            }
            else
            {
              errNL[1] = MAX_DOUBLE;
            }
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
            m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][0] = 0;
#else
            m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 0;
#endif
            errNL[0] = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, 0, bitNL[0], true);

            int bitsNewFilterTempLuma = bitNL[0];
            double err = errNL[0];
            if (errNL[1]  < errNL[0])
            {
              err = errNL[1];
              bitsNewFilterTempLuma = bitNL[1];
              m_alfParamTemp = m_alfParamTempNL;
            }
            if (dDistOrgNewFilter + m_lambda[CHANNEL_TYPE_LUMA] * m_bitsNewFilter[CHANNEL_TYPE_LUMA] < err) //re-derived filter is not good, skip
            {
              continue;
            }
            reconstructCoeff(m_alfParamTemp, CHANNEL_TYPE_LUMA, true, true);
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
            m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMPONENT_Y, &m_alfParamTemp);
            alfCtbFilterSetIndex[ctbIdx] = filterSetIdx;
            m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctbIdx, &m_alfParamTemp.enabledFlag[COMPONENT_Y]);
            double rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
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
          m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMPONENT_Y, &m_alfParamTemp);
          //cost
          double costOff =
            distUnfilterCtb + m_lambda[COMPONENT_Y] * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
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
#if JVET_O0288_UNIFY_ALF_SLICE_TYPE_REMOVAL
#if JVET_O_MAX_NUM_ALF_APS_8
        int tmpBits = bitsNewFilter + 3 * (numFilterSet - NUM_FIXED_FILTER_SETS);
#else
        int tmpBits = bitsNewFilter + 5 * (numFilterSet - NUM_FIXED_FILTER_SETS) + getTBlength(numFilterSet - NUM_FIXED_FILTER_SETS, ALF_CTB_MAX_NUM_APS + 1);
#endif
#else
#if JVET_O_MAX_NUM_ALF_APS_8
        int tmpBits = bitsNewFilter + 3 * (numFilterSet - NUM_FIXED_FILTER_SETS) + (cs.slice->isIntra() ? 1 : 3);
#else
        int tmpBits = bitsNewFilter + 5 * (numFilterSet - NUM_FIXED_FILTER_SETS) + (cs.slice->isIntra() ? 1 : getTBlength(numFilterSet - NUM_FIXED_FILTER_SETS, ALF_CTB_MAX_NUM_APS + 1));
#endif
#endif
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
          alfParamNewFiltersBest = m_alfParamTemp;
          ctxBest = AlfCtx(m_CABACEstimator->getCtx());
          copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
          for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
          {
            m_alfCtbFilterSetIndexTmp[ctuIdx] = alfCtbFilterSetIndex[ctuIdx];
          }
          alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] = useNewFilter;
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
#if !JVET_O0245_VPS_DPS_APS
    alfParamNewFiltersBest.tLayer = cs.slice->getTLayer();
#endif
    cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Y, true);
    cs.slice->setTileGroupNumAps((int)bestApsIds.size());
    cs.slice->setAlfAPSs(bestApsIds);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
    for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
    {
      alfCtbFilterSetIndex[ctuIdx] = m_alfCtbFilterSetIndexTmp[ctuIdx];
    }
    if (alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA])
    {
      APS* newAPS = m_apsMap->getPS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
        newAPS->setAPSId(newApsId);
        newAPS->setAPSType(ALF_APS);
      }
      newAPS->setAlfAPSParam(alfParamNewFiltersBest);
#if JVET_O0245_VPS_DPS_APS
      newAPS->setTemporalId( cs.slice->getTLayer() );
#endif
      newAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] = false;
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  m_alfParamTemp = alfParamNewFiltersBest;
  if( m_alfParamTemp.numAlternativesChroma < 1 )
  {
    m_alfParamTemp.numAlternativesChroma = 1;
  }
  setCtuAlternativeChroma( m_ctuAlternative, 0 );
#endif
  setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, 1);
  getFrameStats(CHANNEL_TYPE_CHROMA, 0);
  costOff = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA);
  costMin = MAX_DOUBLE;
  m_CABACEstimator->getCtx() = AlfCtx(ctxBest);
  ctxStart = AlfCtx(m_CABACEstimator->getCtx());
  int newApsIdChroma = -1;
  if (alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] && (alfParamNewFiltersBest.enabledFlag[COMPONENT_Cb] || alfParamNewFiltersBest.enabledFlag[COMPONENT_Cr]))
  {
    newApsIdChroma = newApsId;
  }
  else if (alfParamNewFiltersBest.enabledFlag[COMPONENT_Cb] || alfParamNewFiltersBest.enabledFlag[COMPONENT_Cr])
  {
    int curId = m_apsIdStart;
    while (newApsIdChroma < 0)
    {
      curId--;
      if (curId < 0)
      {
#if JVET_O_MAX_NUM_ALF_APS_8
        curId = ALF_CTB_MAX_NUM_APS - 1;
#else
        curId = (int)MAX_NUM_APS - 1;
#endif
      }
      if (std::find(bestApsIds.begin(), bestApsIds.end(), curId) == bestApsIds.end())
      {
        newApsIdChroma = curId;
      }
    }
  }
#if JVET_O_MAX_NUM_ALF_APS_8
  for (int curApsId = 0; curApsId < ALF_CTB_MAX_NUM_APS; curApsId++)
#else
  for (int curApsId = 0; curApsId < MAX_NUM_APS; curApsId++)
#endif
  {
    if ((cs.slice->getPendingRasInit() || cs.slice->isIDRorBLA() || cs.slice->isIntra()) && curApsId != newApsIdChroma)
    {
      continue;
    }
    APS* curAPS = m_apsMap->getPS((curApsId << NUM_APS_TYPE_LEN) + ALF_APS);
#if JVET_O0288_UNIFY_ALF_SLICE_TYPE_REMOVAL
#if JVET_O_MAX_NUM_ALF_APS_8
    double curCost = m_lambda[CHANNEL_TYPE_CHROMA] * 3;
#else
    double curCost = m_lambda[CHANNEL_TYPE_CHROMA] * 5;
#endif
#else
#if JVET_O_MAX_NUM_ALF_APS_8
    double curCost = (cs.slice->isIntra() && cs.slice->getTileGroupNumAps() == 1) ? 0 : (m_lambda[CHANNEL_TYPE_CHROMA] * 3);
#else
    double curCost = (cs.slice->isIntra() && cs.slice->getTileGroupNumAps() == 1) ? 0 : (m_lambda[CHANNEL_TYPE_CHROMA] * 5);
#endif
#endif
    if (curApsId == newApsIdChroma)
    {
      m_alfParamTemp = alfParamNewFilters;
      curCost += m_lambda[CHANNEL_TYPE_CHROMA] * m_bitsNewFilter[CHANNEL_TYPE_CHROMA];
    }
    else if (curAPS && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA])
    {
      m_alfParamTemp = curAPS->getAlfAPSParam();
    }
    else
    {
      continue;
    }
    reconstructCoeff(m_alfParamTemp, CHANNEL_TYPE_CHROMA, true, true);
    m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
    for (int compId = 1; compId < MAX_NUM_COMPONENT; compId++)
    {
      m_alfParamTemp.enabledFlag[compId] = true;
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
        m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, compId, &m_alfParamTemp);
        double rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
#if ENABLE_QPA
        const double ctuLambda = lambdaChromaWeight > 0.0 ? cs.picture->m_uEnerHpCtu[ctbIdx] / lambdaChromaWeight : m_lambda[compId];
#else
        const double ctuLambda = m_lambda[compId];
#endif
        double dist = MAX_DOUBLE;
        int numAlts = m_alfParamTemp.numAlternativesChroma;
        ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
        double bestAltRate = 0;
        double bestAltCost = MAX_DOUBLE;
        int bestAltIdx = -1;
        ctxTempAltStart = AlfCtx( ctxTempBest );
        for( int altIdx = 0; altIdx < numAlts; ++altIdx )
        {
          if( altIdx )
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempAltStart );
          m_CABACEstimator->resetBits();
          m_ctuAlternative[compId][ctbIdx] = altIdx;
          m_CABACEstimator->codeAlfCtuAlternative( cs, ctbIdx, compId, &m_alfParamTemp );
          double altRate   = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          double r_altCost = ctuLambda * altRate;

          //distortion
          for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
          {
            m_filterTmp[i] = m_chromaCoeffFinal[altIdx][i];
            m_clipTmp[i] = m_chromaClippFinal[altIdx][i];
          }
          double altDist = m_alfCovariance[compId][0][ctbIdx][0].calcErrorForCoeffs( m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, m_NUM_BITS );
          double altCost = altDist + r_altCost;
          if( altCost < bestAltCost )
          {
            bestAltCost = altCost;
            bestAltIdx = altIdx;
            bestAltRate = altRate;
            ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
            dist = altDist;
          }
        }
        m_ctuAlternative[compId][ctbIdx] = bestAltIdx;
        rateOn += bestAltRate;
        dist += distUnfilterCtu;
        //cost
        double costOn = dist + ctuLambda * rateOn;
#else
        //distortion
        for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
        {
          m_filterTmp[i] = m_chromaCoeffFinal[i];
          m_clipTmp[i] = m_chromaClippFinal[i];
        }
        double dist = distUnfilterCtu + m_alfCovariance[compId][0][ctbIdx][0].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, m_NUM_BITS);
        double costOn = dist + m_lambda[compId] * rateOn;
        ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
#endif
        //cost off
        m_ctuEnableFlag[compId][ctbIdx] = 0;
        //rate
        m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
        m_CABACEstimator->resetBits();
        m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, compId, &m_alfParamTemp);
        //cost
        double costOff = distUnfilterCtu + m_lambda[compId] * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
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
    setEnableFlag(m_alfParamTemp, CHANNEL_TYPE_CHROMA, m_ctuEnableFlag);
#if !JVET_O0491_HLS_CLEANUP
    const int alfChromaIdc = m_alfParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfParamTemp.enabledFlag[COMPONENT_Cr];
#endif
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
#if !JVET_O0491_HLS_CLEANUP
    curCost += (lengthTruncatedUnary(alfChromaIdc, 3) - lengthTruncatedUnary(0, 3)) * m_lambda[CHANNEL_TYPE_CHROMA];
#endif
#else
    curCost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[CHANNEL_TYPE_CHROMA];
#endif

    if (curCost < costMin)
    {
      costMin = curCost;
      cs.slice->setTileGroupApsIdChroma(curApsId);
      cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Cb, m_alfParamTemp.enabledFlag[COMPONENT_Cb]);
      cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Cr, m_alfParamTemp.enabledFlag[COMPONENT_Cr]);
      copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      copyCtuAlternativeChroma(m_ctuAlternativeTmp, m_ctuAlternative);
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    copyCtuAlternativeChroma(m_ctuAlternative, m_ctuAlternativeTmp);
#endif
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
      if (!alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA])
      {
        newAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] = false;
      }
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      newAPS->getAlfAPSParam().numAlternativesChroma = alfParamNewFilters.numAlternativesChroma;
      for( int altIdx = 0; altIdx < MAX_NUM_ALF_ALTERNATIVES_CHROMA; ++altIdx )
        newAPS->getAlfAPSParam().nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx] = alfParamNewFilters.nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx];
#else
      newAPS->getAlfAPSParam().nonLinearFlag[CHANNEL_TYPE_CHROMA] = alfParamNewFilters.nonLinearFlag[CHANNEL_TYPE_CHROMA];
#endif
#if JVET_O0245_VPS_DPS_APS
      newAPS->setTemporalId( cs.slice->getTLayer() );
#else
      newAPS->getAlfAPSParam().tLayer = cs.slice->getTLayer();
#endif
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
      for (int altIdx = 0; altIdx  < MAX_NUM_ALF_ALTERNATIVES_CHROMA; ++altIdx )
      for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
      {
        newAPS->getAlfAPSParam().chromaCoeff[altIdx][i] = alfParamNewFilters.chromaCoeff[altIdx][i];
        newAPS->getAlfAPSParam().chromaClipp[altIdx][i] = alfParamNewFilters.chromaClipp[altIdx][i];
      }
#else
      for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
      {
        newAPS->getAlfAPSParam().chromaCoeff[i] = alfParamNewFilters.chromaCoeff[i];
        newAPS->getAlfAPSParam().chromaClipp[i] = alfParamNewFilters.chromaClipp[i];
      }
#endif
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
#if !JVET_O0625_ALF_PADDING
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
#endif
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
#if JVET_O0625_ALF_PADDING
  int alfBryList[4] = { ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY, ALF_NONE_BOUNDARY }; // 0 - top, 1 - bottom, 2 - left, 3 - right.
#endif
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
#if JVET_O0625_ALF_PADDING
      if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, alfBryList[0], alfBryList[1], alfBryList[2], alfBryList[3], numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS() ) )
#else
      if (ctuEnableFlag && isCrossedByVirtualBoundaries(xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.slice->getPPS()))
#endif
      {
        int yStart = yPos;
        for (int i = 0; i <= numHorVirBndry; i++)
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
#if JVET_O0625_ALF_PADDING
          const bool clipT = ( i == 0 && alfBryList[0] != ALF_NONE_BOUNDARY ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && alfBryList[1] != ALF_NONE_BOUNDARY ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
#else
          const bool clipT = (i == 0 && clipTop) || (i > 0) || (yStart == 0);
          const bool clipB = (i == numHorVirBndry && clipBottom) || (i < numHorVirBndry ) || (yEnd == pcv.lumaHeight);
#endif
          int xStart = xPos;
          for (int j = 0; j <= numVerVirBndry; j++)
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
#if JVET_O0625_ALF_PADDING
            const bool clipL = ( j == 0 && alfBryList[2] != ALF_NONE_BOUNDARY ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && alfBryList[3] != ALF_NONE_BOUNDARY ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
#else
            const bool clipL = (j == 0 && clipLeft) || (j > 0) || (xStart == 0);
            const bool clipR = (j == numVerVirBndry && clipRight) || (j < numVerVirBndry ) || (xEnd == pcv.lumaWidth);
#endif
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
#if JVET_O0625_ALF_PADDING
                , ( ( yPos + pcv.maxCUHeight >= pcv.lumaHeight ) ? pcv.lumaHeight : m_alfVBLumaPos ), alfBryList
#else
                , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
                const int alt_num = m_ctuAlternative[compID][ctuIdx];
                m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs
                  , m_alfVBChmaCTUHeight
#if JVET_O0625_ALF_PADDING
                  , ( ( yPos + pcv.maxCUHeight >= pcv.lumaHeight ) ? pcv.lumaHeight : m_alfVBChmaPos ), alfBryList
#else
                  , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
#endif
#else
                m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs
                  , m_alfVBChmaCTUHeight
                  , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
#endif
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
#if JVET_O0625_ALF_PADDING
          , ( ( yPos + pcv.maxCUHeight >= pcv.lumaHeight ) ? pcv.lumaHeight : m_alfVBLumaPos ), alfBryList
#else
          , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBLumaPos)
#endif
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
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
          const int alt_num = m_ctuAlternative[compID][ctuIdx];
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
#if JVET_O0625_ALF_PADDING
            , ( ( yPos + pcv.maxCUHeight >= pcv.lumaHeight ) ? pcv.lumaHeight : m_alfVBChmaPos ), alfBryList
#else
            , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
#endif
#else
          m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal, m_chromaClippFinal, m_clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
            , ((yPos + pcv.maxCUHeight >= pcv.lumaHeight) ? pcv.lumaHeight : m_alfVBChmaPos)
#endif
          );
        }
      }
      }
      ctuIdx++;
    }
  }
}
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB

void EncAdaptiveLoopFilter::copyCtuAlternativeChroma( uint8_t* ctuAltsDst[MAX_NUM_COMPONENT], uint8_t* ctuAltsSrc[MAX_NUM_COMPONENT] )
{
  std::copy_n( ctuAltsSrc[COMPONENT_Cb], m_numCTUsInPic, ctuAltsDst[COMPONENT_Cb] );
  std::copy_n( ctuAltsSrc[COMPONENT_Cr], m_numCTUsInPic, ctuAltsDst[COMPONENT_Cr] );
}

void EncAdaptiveLoopFilter::setCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMPONENT], uint8_t val )
{
  std::fill_n( ctuAlts[COMPONENT_Cb], m_numCTUsInPic, val );
  std::fill_n( ctuAlts[COMPONENT_Cr], m_numCTUsInPic, val );
}

void EncAdaptiveLoopFilter::initCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMPONENT] )
{
  uint8_t altIdx = 0;
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx )
  {
    ctuAlts[COMPONENT_Cb][ctuIdx] = altIdx;
    if( (ctuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numCTUsInPic )
      ++altIdx;
  }
  altIdx = 0;
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx )
  {
    ctuAlts[COMPONENT_Cr][ctuIdx] = altIdx;
    if( (ctuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numCTUsInPic )
      ++altIdx;
  }
}

int EncAdaptiveLoopFilter::getMaxNumAlternativesChroma( )
{
  return std::min<int>( m_numCTUsInPic * 2, m_encCfg->getMaxNumAlfAlternativesChroma() );
}
#endif
