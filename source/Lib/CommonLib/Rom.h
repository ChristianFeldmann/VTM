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

/** \file     Rom.h
    \brief    global variables & functions (header)
*/

#ifndef __ROM__
#define __ROM__

#include "CommonDef.h"
#include "Common.h"

#include <stdio.h>
#include <iostream>


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

void         initROM();
void         destroyROM();

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

// flexible conversion from relative to absolute index
struct ScanElement
{
  uint32_t idx;
  uint16_t x;
  uint16_t y;
};

#if JVET_N0103_CGSIZE_HARMONIZATION
extern       uint32_t   g_log2SbbSize[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][2];
extern ScanElement
  *g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
#else
extern       uint32_t   g_log2SbbSize   [2][MAX_CU_DEPTH+1][MAX_CU_DEPTH+1][2];
extern ScanElement
  *g_scanOrder[2][SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
#endif
#if JVET_N0193_LFNST
extern       ScanElement   g_coefTopLeftDiagScan8x8[ MAX_CU_SIZE / 2 + 1 ][ 64 ];
#endif

#if JVET_N0246_MODIFIED_QUANTSCALES
extern const int g_quantScales   [2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // Q(QP%6)
extern const int g_invQuantScales[2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // IQ(QP%6)
#else
extern const int g_quantScales   [SCALING_LIST_REM_NUM];          // Q(QP%6)
extern const int g_invQuantScales[SCALING_LIST_REM_NUM];          // IQ(QP%6)
#endif

static const int g_numTransformMatrixSizes = 6;
static const int g_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = {  6, 6 };

#if JVET_N0217_MATRIX_INTRAPRED
extern const uint8_t g_intraMode65to33AngMapping[NUM_INTRA_MODE];
extern const uint8_t g_mapMipToAngular65[3][MAX_NUM_MIP_MODE];
extern const uint8_t g_mapAngular33ToMip[3][35];
extern const int     g_sortedMipMpms    [3][NUM_MPM_MIP];
#endif

// ====================================================================================================================
// Luma QP to Chroma QP mapping
// ====================================================================================================================
static const int chromaQPMappingTableSize = (MAX_QP + 7);

extern const uint8_t  g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize];


// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

extern const uint32_t   g_uiGroupIdx[ MAX_TB_SIZEY ];
extern const uint32_t   g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];
extern const uint32_t   g_auiGoRiceParsCoeff     [ 32 ];
extern const uint32_t   g_auiGoRicePosCoeff0[ 3 ][ 32 ];

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const uint8_t  g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1];
extern const uint8_t  g_aucIntraModeNumFast_UseMPM   [MAX_CU_DEPTH];
extern const uint8_t  g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH];

extern const uint8_t  g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];


// ====================================================================================================================
// Mode-Dependent DST Matrices
// ====================================================================================================================


extern const TMatrixCoeff g_trCoreDCT2P2  [TRANSFORM_NUMBER_OF_DIRECTIONS][  2][  2];
extern const TMatrixCoeff g_trCoreDCT2P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDCT2P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDCT2P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT2P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];
extern const TMatrixCoeff g_trCoreDCT2P64 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 64][ 64];

extern const TMatrixCoeff g_trCoreDCT8P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDCT8P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDCT8P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT8P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];

extern const TMatrixCoeff g_trCoreDST7P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDST7P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDST7P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDST7P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];

#if JVET_N0193_LFNST
extern const     int8_t   g_lfnst8x8[ 4 ][ 2 ][ 16 ][ 48 ];
extern const     int8_t   g_lfnst4x4[ 4 ][ 2 ][ 16 ][ 16 ];

extern const     uint8_t  g_lfnstLut[ NUM_INTRA_MODE + NUM_EXT_LUMA_MODE - 1 ];
#endif

// ====================================================================================================================
// Misc.
// ====================================================================================================================
extern SizeIndexInfo* gp_sizeIdxInfo;
extern int8_t          g_aucLog2                       [MAX_CU_SIZE + 1];
extern int8_t          g_aucNextLog2        [MAX_CU_SIZE + 1];
extern int8_t          g_aucPrevLog2        [MAX_CU_SIZE + 1];

inline bool is34( const SizeType& size )
{
  return ( size & ( ( int64_t ) 1 << ( g_aucLog2[size] - 1 ) ) );
}

inline bool is58( const SizeType& size )
{
  return ( size & ( ( int64_t ) 1 << ( g_aucLog2[size] - 2 ) ) );
}

inline bool isNonLog2BlockSize( const Size& size )
{
  return ( ( 1 << g_aucLog2[size.width] ) != size.width ) || ( ( 1 << g_aucLog2[size.height] ) != size.height );
}

inline bool isNonLog2Size( const SizeType& size )
{
  return ( ( 1 << g_aucLog2[size] ) != size );
}

extern UnitScale     g_miScaling; // scaling object for motion scaling

/*! Sophisticated Trace-logging */
#if ENABLE_TRACING
#include "dtrace.h"
extern CDTrace* g_trace_ctx;
#endif

const char* nalUnitTypeToString(NalUnitType type);

#if HEVC_USE_SCALING_LISTS
extern const char *MatrixType   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];
extern const char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern const int g_quantTSDefault4x4   [4*4];
extern const int g_quantIntraDefault8x8[8*8];
extern const int g_quantInterDefault8x8[8*8];

extern const uint32_t g_scalingListSize [SCALING_LIST_SIZE_NUM];
extern const uint32_t g_scalingListSizeX[SCALING_LIST_SIZE_NUM];
#endif

extern MsgLevel g_verbosity;


extern const int8_t g_GbiLog2WeightBase;
extern const int8_t g_GbiWeightBase;
extern const int8_t g_GbiWeights[GBI_NUM];
extern const int8_t g_GbiSearchOrder[GBI_NUM];
extern       int8_t g_GbiCodingOrder[GBI_NUM];
extern       int8_t g_GbiParsingOrder[GBI_NUM];

class CodingStructure;
int8_t getGbiWeight(uint8_t gbiIdx, uint8_t uhRefFrmList);
void resetGbiCodingOrder(bool bRunDecoding, const CodingStructure &cs);
uint32_t deriveWeightIdxBits(uint8_t gbiIdx);

constexpr uint8_t g_tbMax[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8 };

//! \}

extern       uint8_t g_triangleMvStorage[TRIANGLE_DIR_NUM][MAX_CU_DEPTH - MIN_CU_LOG2 + 1][MAX_CU_DEPTH - MIN_CU_LOG2 + 1][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];

extern bool g_mctsDecCheckEnabled;

#endif  //__TCOMROM__

