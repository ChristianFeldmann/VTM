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

/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTER__
#define __ENCADAPTIVELOOPFILTER__

#include "CommonLib/AdaptiveLoopFilter.h"

#include "CABACWriter.h"
#include "EncCfg.h"

struct AlfCovariance
{
  static constexpr int MaxAlfNumClippingValues = AdaptiveLoopFilter::MaxAlfNumClippingValues;
  using TE = double[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = double[MAX_NUM_ALF_LUMA_COEFF];
  using TKE = TE[AdaptiveLoopFilter::MaxAlfNumClippingValues][AdaptiveLoopFilter::MaxAlfNumClippingValues];
  using TKy = Ty[AdaptiveLoopFilter::MaxAlfNumClippingValues];

  int numCoeff;
  int numBins;
  TKy y;
  TKE E;
  double pixAcc;

  AlfCovariance() {}
  ~AlfCovariance() {}

  void create( int size, int num_bins = MaxAlfNumClippingValues )
  {
    numCoeff = size;
    numBins = num_bins;
    std::memset( y, 0, sizeof( y ) );
    std::memset( E, 0, sizeof( E ) );
  }

  void destroy()
  {
  }

  void reset( int num_bins = -1 )
  {
    if ( num_bins > 0 )
      numBins = num_bins;
    pixAcc = 0;
    std::memset( y, 0, sizeof( y ) );
    std::memset( E, 0, sizeof( E ) );
  }

  const AlfCovariance& operator=( const AlfCovariance& src )
  {
    numCoeff = src.numCoeff;
    numBins = src.numBins;
    std::memcpy( E, src.E, sizeof( E ) );
    std::memcpy( y, src.y, sizeof( y ) );
    pixAcc = src.pixAcc;

    return *this;
  }

  void add( const AlfCovariance& lhs, const AlfCovariance& rhs )
  {
    numCoeff = lhs.numCoeff;
    numBins = lhs.numBins;
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            E[b0][b1][j][i] = lhs.E[b0][b1][j][i] + rhs.E[b0][b1][j][i];
          }
        }
      }
    }
    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        y[b][j] = lhs.y[b][j] + rhs.y[b][j];
      }
    }
    pixAcc = lhs.pixAcc + rhs.pixAcc;
  }

  const AlfCovariance& operator+= ( const AlfCovariance& src )
  {
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            E[b0][b1][j][i] += src.E[b0][b1][j][i];
          }
        }
      }
    }
    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        y[b][j] += src.y[b][j];
      }
    }
    pixAcc += src.pixAcc;

    return *this;
  }

  const AlfCovariance& operator-= ( const AlfCovariance& src )
  {
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            E[b0][b1][j][i] -= src.E[b0][b1][j][i];
          }
        }
      }
    }
    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        y[b][j] -= src.y[b][j];
      }
    }
    pixAcc -= src.pixAcc;

    return *this;
  }

  void setEyFromClip(const int* clip, TE _E, Ty _y, int size) const
  {
    for (int k=0; k<size; k++)
    {
      _y[k] = y[clip[k]][k];
      for (int l=0; l<size; l++)
      {
        _E[k][l] = E[clip[k]][clip[l]][k][l];
      }
    }
  }

  double optimizeFilter(const int* clip, double *f, int size) const
  {
    gnsSolveByChol( clip, f, size );
    return calculateError( clip, f );
  }

  double optimizeFilter(const AlfFilterShape& alfShape, int* clip, double *f, bool optimize_clip) const;
  double optimizeFilterClip(const AlfFilterShape& alfShape, int* clip) const
  {
    Ty f;
    return optimizeFilter(alfShape, clip, f, true);
  }

  double calculateError( const int *clip ) const;
  double calculateError( const int *clip, const double *coeff ) const { return calculateError(clip, coeff, numCoeff); }
  double calculateError( const int *clip, const double *coeff, const int numCoeff ) const;
  double calcErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const int bitDepth ) const;

  void getClipMax(const AlfFilterShape& alfShape, int *clip_max) const;
  void reduceClipCost(const AlfFilterShape& alfShape, int *clip) const;

private:
  // Cholesky decomposition

  int  gnsSolveByChol( const int *clip, double *x, int numEq ) const;
  int  gnsSolveByChol( TE LHS, double* rhs, double *x, int numEq ) const;
  void gnsBacksubstitution( TE R, double* z, int size, double* A ) const;
  void gnsTransposeBacksubstitution( TE U, double* rhs, double* x, int order ) const;
  int  gnsCholeskyDec( TE inpMatr, TE outMatr, int numEq ) const;
};

class EncAdaptiveLoopFilter : public AdaptiveLoopFilter
{
public:
  static constexpr int   m_MAX_SCAN_VAL = 11;
  static constexpr int   m_MAX_EXP_GOLOMB = 16;
  int m_alfWSSD;
  inline void           setAlfWSSD(int alfWSSD) { m_alfWSSD = alfWSSD; }
  static std::vector<double>  m_lumaLevelToWeightPLUT;
  inline std::vector<double>& getLumaLevelWeightTable() { return m_lumaLevelToWeightPLUT; }

private:
  const EncCfg*          m_encCfg;
  AlfCovariance***       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][classIdx]
  AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_CHANNEL_TYPE];   // [CHANNEL][shapeIdx][classIdx]
  uint8_t*                 m_ctuEnableFlagTmp[MAX_NUM_COMPONENT];

  //for RDO
  AlfSliceParam          m_alfSliceParamTemp;
  ParameterSetMap<APS>*  m_apsMap;
  AlfCovariance          m_alfCovarianceMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES + 2];
  int                    m_alfClipMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_CtxCache;
  double                 m_lambda[MAX_NUM_COMPONENT];
  const double           FracBitsScale = 1.0 / double( 1 << SCALE_BITS );

  int**                  m_filterCoeffSet;
  int**                  m_filterClippSet;
  int**                  m_diffFilterCoeff;
  int                    m_kMinTab[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB];
  short                  m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];
  unsigned               m_bitsNewFilter[MAX_NUM_CHANNEL_TYPE];
  int                    m_apsIdStart;
  double                 *m_ctbDistortionFixedFilter;
  double                 *m_ctbDistortionUnfilter[MAX_NUM_COMPONENT];
  std::vector<short>     m_alfCtbFilterSetIndexTmp;
  AlfSliceParam          m_alfSliceParamTempNL;
  int                    m_clipDefaultEnc[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_filterTmp[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_clipTmp[MAX_NUM_ALF_LUMA_COEFF];

public:
  EncAdaptiveLoopFilter();
  virtual ~EncAdaptiveLoopFilter() {}
  void  initDistortion();
  std::vector<int> getAvaiApsIdsLuma(CodingStructure& cs, int &newApsId);
  void  alfEncoderCtb(CodingStructure& cs, AlfSliceParam& alfSliceParamNewFilters
#if ENABLE_QPA
    , const double lambdaChromaWeight
#endif
  );
  void   alfReconstructor(CodingStructure& cs, const PelUnitBuf& recExtBuf);
  void ALFProcess(CodingStructure& cs, const double *lambdas
#if ENABLE_QPA
    , const double lambdaChromaWeight
#endif
  );
  void initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice, ParameterSetMap<APS>* apsMap );
  void create( const EncCfg* encCfg, const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] );
  void destroy();
  static int lengthGolomb( int coeffVal, int k, bool signed_coeff=true );
  static int getGolombKMin( AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB] );

private:
  void   alfEncoder( CodingStructure& cs, AlfSliceParam& alfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel
#if ENABLE_QPA
                   , const double lambdaChromaWeight = 0.0
#endif
                   );

  void   copyAlfSliceParam( AlfSliceParam& alfSliceParamDst, AlfSliceParam& alfSliceParamSrc, ChannelType channel );
  double mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits );

  void   getFrameStats( ChannelType channel, int iShapeIdx );
  void   getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses );
  void   deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv, CodingStructure& cs );
  void   getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos);
  void   calcCovariance(int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance);
  void   mergeClasses(const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]);


  double getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, bool onlyFilterCost = false );
  double deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfSliceParam& alfSliceParam);
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode );
  double deriveCoeffQuant( int *filterClipp, int *filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip );
  double deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel,
#if ENABLE_QPA
                                  const double chromaWeight,
#endif
                                  const int numClasses, const int numCoeff, double& distUnfilter );
  void   roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor );

  double getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters );
  int    lengthTruncatedUnary( int symbol, int maxSymbol );
  int    lengthUvlc( int uiCode );
  int    getNonFilterCoeffRate( AlfSliceParam& alfSliceParam );
  int    getTBlength( int uiSymbol, const int uiMaxSymbol );

  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins );
  int    getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    getCostFilterClipp( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab );
  int    lengthFilterClipps( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab );
  double getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins );
  int    getCoeffRate( AlfSliceParam& alfSliceParam, bool isChroma );

  double getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel );
  double getUnfilteredDistortion( AlfCovariance* cov, const int numClasses );
  double getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff );

  void setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, bool val );
  void setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags );
  void setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val );
  void copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel );
};



#endif
