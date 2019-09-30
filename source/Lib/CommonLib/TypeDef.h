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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#ifndef __TYPEDEF__
#define __TYPEDEF__

#ifndef __COMMONDEF__
#error Include CommonDef.h not TypeDef.h
#endif

#include <vector>
#include <utility>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <assert.h>
#include <cassert>

#define JVET_O0245_VPS_DPS_APS                            1 // JVET-O0245: constraints for VPS, DPS, and APS

#define JVET_O0044_MULTI_SUB_PROFILE                      1 // JVET-O0044: Zero or more sub profile fields

#define JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA           1 // JVET-O0143: Remove signaling of top_right_brick_idx

#define JVET_O0236_PPS_PARSING_DEPENDENCY                 1 // JVET-O0236: Resolves a PPS parsing dependency

#define JVET_O0148_NUM_ACTIVE_REF_PIC_CHECK               1 // JVET-O0148: Constraint that num active entries in RPL 0 and RPL 1 for P and B pictures

#define JVET_O0145_ENTRYPOINT_SIGNALLING                  0 // JVET-O0145: Not signalling num_entry_point_offsets but derive it at decoder

#define JVET_N0494_DRAP                                   1 // JVET-N0494: Dependent random access point indication SEI

#define JVET_O0173_O0176_O0338_NUMBRICK_M2                1 // JVET-O0173, O0176, O0338 : Replacing num_brick_rows_minus1 with num_brick_rows_minus2

#define JVET_OO152_BP_SEI_GDR                             1 // JVET-O0152: Allowing association of a buffering period SEI with GDR picture

#define JVET_OO147_LEADING_PIC_CHECKING                   1 // JVET-O0147: Constraints on leading pictures

#define JVET_O0299_APS_SCALINGLIST                        1 // JVET-O0299: Scaling List Matrices Support in APS

#define JVET_O1164_RPR                                    1  // JVET-O1164: Reference picture resampling
#if JVET_O1164_RPR
#define JVET_O1164_PS                                     1
#define RPR_BUFFER                                        1  // lossless
#define RPR_CTC_PRINT                                     1
#define RPR_CONF_WINDOW                                   1
#endif

#define JVET_O0625_ALF_PADDING                            1 // JVET-O0625/O0654/O0662: Unified padding method in ALF

#define JVET_O0238_PPS_OR_SLICE                           1 // JVET-O0238: Signal certain parameters either in PPS or per slice
#define SUPPORT_FOR_RECT_SLICES_WITH_VARYING_NUMBER_OF_TILES 1

#define JVET_O0452_PPS_BRICK_SIGNALING_CONDITION          1  // JVET-O0452: brick_split_flag signaled as RowHeight > 1 and uniform_brick_spacing_flag signaled as RowHeight > 2

#define FLATTEN_BUFFERS                                                                                                \
  (1 && JVET_O0502_ISP_CLEANUP && JVET_O0364_PADDING && JVET_O0426_MRL_REF_SAMPLES_DC_MODE                             \
   && JVET_O0925_MIP_SIMPLIFICATIONS && JVET_O0364_PDPC_DC)

#define JVET_O0119_BASE_PALETTE_444                       1 // JVET-O0119: Palette mode in HEVC and palette mode signaling in JVET-N0258. Only enabled for YUV444.

#define JVET_O0304_SIMPLIFIED_BDOF                        1 // JVET-O0304: Reduction of number of multiplications in BDOF

#define JVET_O0455_IBC_MAX_MERGE_NUM                      1 // JVET-O0455: Control the max number of IBC merge candidates independently from regular merge candidates

#define JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE          1 // JVET-O0650: Signal chroma QP mapping tables and move chroma PPS/slice offsets after mapping table

#define JVET_O0502_ISP_CLEANUP                            1 // JVET-O0502: Enable PDPC and all 67 intra modes and apply the cubic filter always (also included in JVET-O0341) for ISP

#define JVET_O0640_PICTURE_SIZE_CONSTRAINT                1 // JVET-O0640: Picture width and height shall be a multiple of Max(8, minCU size)

#define JVET_O_MAX_NUM_ALF_APS_8                          1 // JVET-O: number of ALF APSs is reduced to 8

#define JVET_O0925_MIP_SIMPLIFICATIONS                    1 // JVET-O0925: Simplifications of MIP

#define JVET_O0070_PROF                                   1 // JVET-O0070 method 4-2.1a: Prediction refinement with optical flow for affine mode

#define JVET_O0570_GRAD_SIMP                              1 // JVET-O0570/JVET-O0211, SMID friendly spatial gradient calculation

#define JVET_O1170_IBC_VIRTUAL_BUFFER                     1 // JVET-O1170/O1171: IBC virtual buffer
#if JVET_O1170_IBC_VIRTUAL_BUFFER
#define JVET_O1170_CHECK_BV_AT_DECODER                    1 // For decoder to check if a BV is valid or not
#endif

#define JVET_O0538_SPS_CONTROL_ISP_SBT                    1 // JVET-O0538: SPS control for ISP and SBT transform

#define JVET_O0634_BDOF_SIZE_CONSTRAINT                   1 // JVET-O0634: BDOF applied CU size align with DMVR

#define JVET_O0213_RESTRICT_LFNST_TO_MAX_TB_SIZE          1 // JVET-O0213: Block size restriction of LFNST to maximum transform size

#define JVET_O0617_SIG_FLAG_CONTEXT_REDUCTION             1 // JVET-O0617: Significant flag context reduction

#define JVET_O0244_DELTA_POC                              1 // JVET-O0244: weighted prediction in SPS and delta POC

#define JVET_O1153_INTRA_CHROMAMODE_CODING                1  //JVET-O1153: simplified intra chromamode coding

#define JVET_O0159_10BITTCTABLE_DEBLOCKING                1 // tc table for 10-bit video

#define JVET_O0061_MV_THR_DEBLOCKING                      1 // a deblocking mv threshold of half pel

#define JVET_O0220_METHOD1_SUBBLK_FLAG_PARSING            1 // JVET-O0220 method-1: Parse merge_subblock_flag conditioned on MaxNumSubblockMergeCand

#define JVET_O0263_O0220_SUBBLOCK_SYNTAX_CLEANUP          1 // JVET-O0263/ JVET-O0220: Syntax cleanup on subblock merge

#define JVET_O0060_4x4_deblocking                         1 // deblock on 4x4 grid

#define JVET_O0046_DQ_SIGNALLING                          1 // JVET-O0046: Move delta-QP earlier for 64x64 VPDU processing, applied to CUs >64x64 only

#define JVET_O0616_400_CHROMA_SUPPORT                     1 // JVET-O0616: Various chroma format support in VVC

#define JVET_O0265_TPM_SIMPLIFICATION                     1 // JVET-O0265/JVET-O0629/JVET-O0418/JVET-O0329/JVET-O0378/JVET-O0411/JVET-O0279:Simplified motion field storage for TPM

#define JVET_O0409_EXCLUDE_CODED_SUB_BLK_FLAG_FROM_COUNT  1 // JVET-O0409: exclude coded_subblock_flag from counting context-coded bins in transform skip

#define JVET_O0057_ALTHPELIF                              1  //AMVR_HPEL

#define JVET_O1136_TS_BDPCM_SIGNALLING                    1 // JVET-O1136: Unified syntax for JVET-O0165/O0200/O0783 on TS and BDPCM signalling

#define JVET_O0219_LFNST_TRANSFORM_SET_FOR_LMCMODE        1

#define JVET_O0426_MRL_REF_SAMPLES_DC_MODE                1 // JVET-O0426: align MRL reference samples used for DC intra mode prediction

#define JVET_O0366_AFFINE_BCW                             1 // JVET-O0366: Simplifications on BCW index derivation process

#define JVET_O0919_TS_MIN_QP                              1 // JVET-O0919: Minimum QP for Transform Skip Mode

#define JVET_O1168_CU_CHROMA_QP_OFFSET                    1 // JVET-O1168: cu chroma QP offset

#define JVET_O0368_LFNST_WITH_DCT2_ONLY                   1 // JVET-O0368/O0292/O0521/O0466: disable LFNST for non-DCT2 MTS candidates normatively

#define JVET_O0106_ISP_4xN_PREDREG_FOR_1xN_2xN            1 // JVET-O0106: use 4xN prediction regions for 1xN and 2xN subblocks

#define JVET_O0500_SEP_CTX_AFFINE_SUBBLOCK_MRG            1 // JVET-O0500: Different ctx models for inter affine flag and subblock merge flag

#define JVET_O0414_SMVD_LTRP                              1 // JVET-O0414: long-term reference picture restriction for SMVD

#define JVET_O0258_REMOVE_CHROMA_IBC_FOR_DUALTREE         1 // JVET-O0258 Remove chroma IBC when dualtree is used

#define JVET_O1161_IBC_MAX_SIZE                           1 // Limit largest IBC luma CU size to 64x64 per discussion of JVET-O1161

#define JVET_O0315_RDPCM_INTRAMODE_ALIGN                  1 // JVET-O0200/O0205/O0296/O0342/O0463/O0542: Intra prediction mode alignment for BDPCM

#define JVET_O0284_CONDITION_SMVD_MVDL1ZEROFLAG           1 // JVET-O0284: condition sym_mvd_flag on mvd_l1_zero_flag

#define JVET_O0122_TS_SIGN_LEVEL                          1 // JVET-O0122: Sign context and level mapping of TS residual coding.

#define JVET_O0438_SPS_AFFINE_AMVR_FLAG                   1 // JVET-O0438: affine AMVR control flag conditioned on affine control flag in SPS

#define JVET_O0065_CABAC_INIT                             1 // JVET-O0065: CABAC initialization

#define JVET_O0052_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT      1 // JVET-O0052 Method-1: TU-level context coded bin constraint

#define JVET_O0105_ICT                                    1 // JVET-O0105: inter-chroma transform (ICT) as extension of joint chroma coding (JCC)
#define JVET_O0543_ICT_ICU_ONLY                           1 // JVET-O0543: ICT only in Intra CUs (was Intra slices, modified during adoption)
#define JVET_N0288_PROPOSAL1                              1   // JVET-N0288 Proposal 1

#define JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB     1 // JVET-O0090 test 2: CTB selection of ALF alternative chroma filters
#define JVET_O0050_LOCAL_DUAL_TREE                        1 // JVET-O0050: avoid small intra chroma block by a "local dual-tree" technique

#define JVET_O0216_ALF_COEFF_EG3                          1 // JVET-O0216/O0302/O0648: using EG3 for ALF coefficients coding

#define JVET_O0256_ADJUST_THD_DEPQUANT                    1 // JVET-O0256: Fast encoder with adjusted threshold in dependent quantization

#define JVET_O0619_GTX_SINGLE_PASS_TS_RESIDUAL_CODING     1 // JVET-O0619/O0623 : Single pass coding of abs_level_gtx_flag[x] for TS residual coding

#define JVET_O0272_LMCS_SIMP_INVERSE_MAPPING              1 // JVET-O0272: LMCS simplified inverse mapping

#define JVET_O0247_ALF_CTB_CODING_REDUNDANCY_REMOVAL      1 // JVET-O0247: not signal APS index when number APS is 2

#define JVET_O0297_DMVR_PADDING                           1 // JVET-O0297 DMVR Padding

#define JVET_O0637_CHROMA_GRADIENT_LINE_SELECTION         1 // Choose line0 and line3 for gradient computation when chroma is same size as luma

#define JVET_O0288_UNIFY_ALF_SLICE_TYPE_REMOVAL           1 // JVET-O0288: remove slice type dependency in ALF

#define JVET_O0064_SIMP_ALF_CLIP_CODING                   1 // JVET-O0047/O0058/O0064/O0067/O0290/O0301/O0430: use FLC for alf clipping indices, always signal alf clipping indices

#define JVET_O0529_IMPLICIT_MTS_HARMONIZE                 1 // JVET-O0529/O0540: Harmonization of LFNST, MIP and implicit MTS

#define JVET_O0669_REMOVE_ALF_COEFF_PRED                  1 // JVET-O0425/O0427/O0669: remove prediction in ALF coefficients coding

#define  JVET_O0526_MIN_CTU_SIZE                          1 // JVET-O0526: Minimum CTU size 32x32

#define JVET_O0545_MAX_TB_SIGNALLING                      1 // JVET-O0545: Configurable maximum transform size

#define JVET_O0525_REMOVE_PCM                             1 // JVET-O0525: Removal of PCM mode

#define JVET_O0541_IMPLICIT_MTS_CONDITION                 1 // JVET_O0541: Decouple the intra implicit transform selection from an inter MTS related SPS flag
#define JVET_O0163_REMOVE_SWITCHING_TMV                   1 // JVET-O0163/JVET-O0588: Remove switching between L0 and L1 for temporal MV
#define JVET_O0655_422_CHROMA_DM_MAPPING_FIX              1 // JVET-O0655: modify chroma DM derivation table for 4:2:2 chroma format

#define JVET_O1109_UNFIY_CRS                              1 // JVET-O1109: Unified CRS derivation

#define JVET_O0590_REDUCE_DMVR_ORIG_MV_COST               1 // Reduce the DMVR cost of the original MV

#define JVET_O0432_LMCS_ENCODER                           1 // JVET-O0432: LMCS encoder improvement

#define JVET_O0429_CRS_LAMBDA_FIX                         1 // JVET-O0429: fix encoder lambda rounding used in CRS

#define JVET_O0428_LMCS_CLEANUP                           1 // JVET-O0428: LMCS cleanups

#define JVET_O0164_REMOVE_AMVP_SPATIAL_SCALING            1 // JVET-O0164/JVET-O0587: remove spatial AMVP candidate scaling

#define JVET_O0162_IBC_MVP_FLAG                           1 // JVET-O0162/O0331/O0480/O0574: IBC mvp flag conditioned on MaxNumMergeCand>1

#define JVET_O0055_INT_DMVR_DIS_BDOF                      1 // integer-distance DMVR cost to disable BDOF and disable BDOF early termination

#define JVET_O0277_INTRA_SMALL_BLOCK_DCTIF                1 // JVET-O0277: DCT-IF interpolation filter is always used for 4x4, 4x8, and 8x4 luma CB

#define JVET_O0267_IBC_SCALING_LIST                       1

#define JVET_O0280_SIMD_TRIANGLE_WEIGHTING                1 // JVET-O0280: SIMD implementation for weighted sample prediction process of triangle prediction mode

#define JVET_O0379_SPEEDUP_TPM_ENCODER                    1 // JVET_O0379: Speedup mode decision process for triangle prediction mode

#define JVET_O0364_PADDING                                1 // JVET-O0364 Part 2: clean up padding process in intra prediction
#define JVET_O0364_PDPC_DC                                1 // JVET-O0364 Part 4: align PDPC process for DC with the one for Planar
#define JVET_O0364_PDPC_ANGULAR                           1 // JVET-O0364 Part 5: simplify PDPC process for angular modes

#define JVET_O0094_LFNST_ZERO_PRIM_COEFFS                 1 // JVET-O0094: CE6-2.1a, LFNST involves zeroing of primary only coefficient positions

#define JVET_O0294_TRANSFORM_CLEANUP                      1 // JVET-O0294: Context modelling for MTS index

#define JVET_O1124_ALLOW_CCLM_COND                        1 // JVET-O1124/JVET-O0196: CCLM restriction to reduce luma-chroma latency for chroma separate tree

#define JVET_O0078_SINGLE_HMVPLUT                         1 // JVET-O0078Single HMVP table for all CUs inside the shared merge list region for IBC

#define JVET_O0126_BPWA_INDEX_CODING_FIX                  1 // JVET-O0126 align BPWA index coding with specification

#define JVET_O0592_ENC_ME_IMP                             1 // JVET-O0592 encoder ME improvement

#define JVET_O0108_DIS_DMVR_BDOF_CIIP                     1 // JVET_O0108 CE9-2.2: disable DMVR and BDOF for CIIP

#define JVET_O1140_SLICE_DISABLE_BDOF_DMVR_FLAG           1 // JVET-O1140 slice level disable flag for BDOF and DMVR

#define JVET_O0567_MVDRange_Constraint                    1 // JVET-O0567: constrain the signalled MVD value to the range of [-2^17, 2^17-1]

#define JVET_O0596_CBF_SIG_ALIGN_TO_SPEC                  1 // JVET-O0596 align cbf signaling with specification
#define JVET_O0193_REMOVE_TR_DEPTH_IN_CBF_CTX             1 // JVET-O0193/JVET-O0375: remove transform depth in cbf context modeling
#define JVET_O0681_DIS_BPWA_CIIP                          1 // JVET-O0681 disable BCW for CIIP, method 2 inherit BCW index
#define JVET_O0249_MERGE_SYNTAX                           1 // JVET-O0249: merge syntax change
#define JVET_O0594_BDOF_REF_SAMPLE_PADDING                1 // JVET-O0594/O0252/O0506/O0615/O0624: BDOF reference sample padding using the nearest integer sample position

#define JVET_O0610_CFG                                    1 // config default change for "Adopt to mandate the presence of AU delimiter for each AU", config parameter should be removed later
#define JVET_O0610_DETECT_AUD                             1 // JVET-O0610: detect presence of mandatory access unit delimiters

#define JVET_O0491_HLS_CLEANUP                            1
#define JVET_O0041_FRAME_FIELD_SEI                        1

#define JVET_O0376_SPS_JOINTCBCR_FLAG                          1 // JVET-O0376: add the JointCbCr control flag in SPS
#define JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS         1 // JVET-O0472: LFNST index signalling depends on the position of last significant coefficient

#define JVET_O0189_DU                                     1
#if JVET_O0189_DU
#define FIX_SEI_O0189                                     1
#define FIX_HRD_O0189                                     1
#endif
#define JVET_N0353_INDEP_BUFF_TIME_SEI                    1 // JVET-N0353 independent parsing of buffering and timing info SEIs
#define JVET_N0867_TEMP_SCAL_HRD                          1 // JVET-N0867 temporal scalability support for HRD in buffering period and picture timing SEIs

#define FIX_DB_MAX_TRANSFORM_SIZE                         1

#define MRG_SHARELIST_SHARSIZE                            32

#define JVET_M0497_MATRIX_MULT                            0 // 0: Fast method; 1: Matrix multiplication

#define APPLY_SBT_SL_ON_MTS                               1 // apply save & load fast algorithm on inter MTS when SBT is on
#if !JVET_O0525_REMOVE_PCM
#define FIX_PCM                                           1 // Fix PCM bugs in VTM3
#endif

#if JVET_O0545_MAX_TB_SIGNALLING
#define MAX_TB_SIZE_SIGNALLING                            1
#else
#define MAX_TB_SIZE_SIGNALLING                            0
#endif

#define JVET_N0100_PROPOSAL1                              1 // JVET-N0100: ltrp picture signalling

#define JVET_N0865_SYNTAX                                 1 // JVET_N0865 syntax elements
#define JVET_N0865_NONSYNTAX                              1 // JVET_N0865 other than syntax parts
#define JVET_N0865_GRA2GDR                                1 // Changing the name of GRA to GDR

#define JVET_O0178                                        1 // JVET_O0178 - conditionally signal sps_sub_layer_ordering_info_present_flag

#define RPL_IN_IDR_FIX                                    1 // JVET_N0865 syntax elements

#define JVET_O0179                                        1 // JVET_O0179: Implement NAL unit header syntax proposal B

#define JVET_O0176_PROPOSAL3                              1 // JVET_O0176 check for rect_slice_flag 

#define JVET_O0181                                        1 // JVET_O0181: non_reference_picture_flag

#define JVET_O0177_PROPOSAL1                              1 // JVET_O0177 proposal 1: Allow signaling of HRD parameters for temporal sub-layers optionally controlled by a flag

#define JVET_O0241                                        1 // JVET_O0241 : decoding process for generating unavailable reference pictures


#define HEVC_SEI                                          0 // SEI messages that are defined in HEVC, but not in VVC

typedef std::pair<int, bool> TrMode;
typedef std::pair<int, int>  TrCost;

// clang-format off
#define ENABLE_JVET_L0283_MRL                             1 // 1: Enable MRL, 0: Disable MRL
#define JVET_L0090_PAIR_AVG                               1 // Add pairwise average candidates, replace HEVC combined candidates
#define REUSE_CU_RESULTS                                  1
#if REUSE_CU_RESULTS
#define REUSE_CU_RESULTS_WITH_MULTIPLE_TUS                1
#if !JVET_O0545_MAX_TB_SIGNALLING
#define MAX_NUM_TUS                                       4
#endif
#endif
// clang-format on


#ifndef JVET_J0090_MEMORY_BANDWITH_MEASURE
#define JVET_J0090_MEMORY_BANDWITH_MEASURE                0
#endif

#ifndef EXTENSION_360_VIDEO
#define EXTENSION_360_VIDEO                               0   ///< extension for 360/spherical video coding support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#ifndef EXTENSION_HDRTOOLS
#define EXTENSION_HDRTOOLS                                0 //< extension for HDRTools/Metrics support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#define JVET_O0756_CONFIG_HDRMETRICS                      1
#if EXTENSION_HDRTOOLS
#define JVET_O0756_CALCULATE_HDRMETRICS                   1
#endif

#ifndef ENABLE_WPP_PARALLELISM
#define ENABLE_WPP_PARALLELISM                            0
#endif
#if ENABLE_WPP_PARALLELISM
#ifndef ENABLE_WPP_STATIC_LINK
#define ENABLE_WPP_STATIC_LINK                            0 // bug fix static link
#endif
#define PARL_WPP_MAX_NUM_THREADS                         16

#endif
#ifndef ENABLE_SPLIT_PARALLELISM
#define ENABLE_SPLIT_PARALLELISM                          0
#endif
#if ENABLE_SPLIT_PARALLELISM
#define PARL_SPLIT_MAX_NUM_JOBS                           6                             // number of parallel jobs that can be defined and need memory allocated
#define NUM_RESERVERD_SPLIT_JOBS                        ( PARL_SPLIT_MAX_NUM_JOBS + 1 )  // number of all data structures including the merge thread (0)
#define PARL_SPLIT_MAX_NUM_THREADS                        PARL_SPLIT_MAX_NUM_JOBS
#define NUM_SPLIT_THREADS_IF_MSVC                         4

#endif


// ====================================================================================================================
// General settings
// ====================================================================================================================

#ifndef ENABLE_TRACING
#define ENABLE_TRACING                                    0 // DISABLE by default (enable only when debugging, requires 15% run-time in decoding) -- see documentation in 'doc/DTrace for NextSoftware.pdf'
#endif

#if ENABLE_TRACING
#define K0149_BLOCK_STATISTICS                            1 // enables block statistics, which can be analysed with YUView (https://github.com/IENT/YUView)
#if K0149_BLOCK_STATISTICS
#define BLOCK_STATS_AS_CSV                                0 // statistics will be written in a comma separated value format. this is not supported by YUView
#endif
#endif

#define WCG_EXT                                           1
#define WCG_WPSNR                                         WCG_EXT

#define KEEP_PRED_AND_RESI_SIGNALS                        0

// ====================================================================================================================
// Debugging
// ====================================================================================================================

// most debugging tools are now bundled within the ENABLE_TRACING macro -- see documentation to see how to use

#define PRINT_MACRO_VALUES                                1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

#define INTRA_FULL_SEARCH                                 0 ///< enables full mode search for intra estimation

// TODO: rename this macro to DECODER_DEBUG_BIT_STATISTICS (may currently cause merge issues with other branches)
// This can be enabled by the makefile
#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS                0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run time by up to ~10%)
#endif

#ifndef RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
#define RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS         (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 (default) = decoder reports as normal, 1 = decoder produces max frame bit usage statistics
#endif

#define TR_ONLY_COEFF_STATS                              (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 combine TS and non-TS decoder debug statistics. 1 = separate TS and non-TS decoder debug statistics.
#define EPBINCOUNT_FIX                                   (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 use count to represent number of calls to decodeBins. 1 = count and bins for EP bins are the same.

#ifndef RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_TOOL_STATISTICS               0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces tool usage statistics
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS || RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_STATISTICS                    1
#endif

// ====================================================================================================================
// Tool Switches - transitory (these macros are likely to be removed in future revisions)
// ====================================================================================================================

#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES  1 ///< TODO: integrate this macro into a broader conformance checking system.
#define T0196_SELECTIVE_RDOQ                              1 ///< selective RDOQ
#define U0040_MODIFIED_WEIGHTEDPREDICTION_WITH_BIPRED_AND_CLIPPING 1
#define U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI    1 ///< Alternative transfer characteristics SEI message (JCTVC-U0033, with syntax naming from V1005)
#define X0038_LAMBDA_FROM_QP_CAPABILITY                   1 ///< This approach derives lambda from QP+QPoffset+QPoffset2. QPoffset2 is derived from QP+QPoffset using a linear model that is clipped between 0 and 3.
                                                            // To use this capability enable config parameter LambdaFromQpEnable

// ====================================================================================================================
// Tool Switches
// ====================================================================================================================


// This can be enabled by the makefile
#ifndef RExt__HIGH_BIT_DEPTH_SUPPORT
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      0 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#endif

// SIMD optimizations
#define SIMD_ENABLE                                       1
#define ENABLE_SIMD_OPT                                 ( SIMD_ENABLE && !RExt__HIGH_BIT_DEPTH_SUPPORT )    ///< SIMD optimizations, no impact on RD performance
#define ENABLE_SIMD_OPT_MCIF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the interpolation filter, no impact on RD performance
#define ENABLE_SIMD_OPT_BUFFER                          ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the buffer operations, no impact on RD performance
#define ENABLE_SIMD_OPT_DIST                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the distortion calculations(SAD,SSE,HADAMARD), no impact on RD performance
#define ENABLE_SIMD_OPT_AFFINE_ME                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for affine ME, no impact on RD performance
#define ENABLE_SIMD_OPT_ALF                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for ALF
#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_GBI                               1                                                 ///< SIMD optimization for GBi
#endif

// End of SIMD optimizations


#define ME_ENABLE_ROUNDING_OF_MVS                         1 ///< 0 (default) = disables rounding of motion vectors when right shifted,  1 = enables rounding

#define RDOQ_CHROMA_LAMBDA                                1 ///< F386: weighting of chroma for RDOQ

#define U0132_TARGET_BITS_SATURATION                      1 ///< Rate control with target bits saturation method
#ifdef  U0132_TARGET_BITS_SATURATION
#define V0078_ADAPTIVE_LOWER_BOUND                        1 ///< Target bits saturation with adaptive lower bound
#endif
#define W0038_DB_OPT                                      1 ///< adaptive DB parameter selection, LoopFilterOffsetInPPS and LoopFilterDisable are set to 0 and DeblockingFilterMetric=2;
#define W0038_CQP_ADJ                                     1 ///< chroma QP adjustment based on TL, CQPTLAdjustEnabled is set to 1;

#define SHARP_LUMA_DELTA_QP                               1 ///< include non-normative LCU deltaQP and normative chromaQP change
#define ER_CHROMA_QP_WCG_PPS                              1 ///< Chroma QP model for WCG used in Anchor 3.2
#define ENABLE_QPA                                        1 ///< Non-normative perceptual QP adaptation according to JVET-H0047 and JVET-K0206. Deactivated by default, activated using encoder arguments --PerceptQPA=1 --SliceChromaQPOffsetPeriodicity=1
#define ENABLE_QPA_SUB_CTU                              ( 1 && ENABLE_QPA ) ///< when maximum delta-QP depth is greater than zero, use sub-CTU QPA


#define RDOQ_CHROMA                                       1 ///< use of RDOQ in chroma

#define QP_SWITCHING_FOR_PARALLEL                         1 ///< Replace floating point QP with a source-file frame number. After switching POC, increase base QP instead of frame level QP.

#define LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET         1 /// JVET-L0414 (CE11.2.2) with explicit signalling of num interval, threshold and qpOffset
// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            1 ///< 0 use original 6-bit transform matrices for both forward and inverse transform, 1 (default) = use original matrices for inverse transform and high precision matrices for forward transform
#else
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            0 ///< 0 (default) use original 6-bit transform matrices for both forward and inverse transform, 1 = use original matrices for inverse transform and high precision matrices for forward transform
#endif

#if FULL_NBIT
#define DISTORTION_PRECISION_ADJUSTMENT(x)                0
#else
#define DISTORTION_ESTIMATION_BITS                        8
#define DISTORTION_PRECISION_ADJUSTMENT(x)                ((x>DISTORTION_ESTIMATION_BITS)? ((x)-DISTORTION_ESTIMATION_BITS) : 0)
#endif

// ====================================================================================================================
// Error checks
// ====================================================================================================================

#if ((RExt__HIGH_PRECISION_FORWARD_TRANSFORM != 0) && (RExt__HIGH_BIT_DEPTH_SUPPORT == 0))
#error ERROR: cannot enable RExt__HIGH_PRECISION_FORWARD_TRANSFORM without RExt__HIGH_BIT_DEPTH_SUPPORT
#endif

// ====================================================================================================================
// Named numerical types
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
typedef       int             Pel;               ///< pixel type
typedef       int64_t           TCoeff;            ///< transform coefficient
typedef       int             TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int64_t           Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint64_t          Intermediate_UInt; ///< used as intermediate value in calculations
#else
typedef       int16_t           Pel;               ///< pixel type
typedef       int             TCoeff;            ///< transform coefficient
typedef       int16_t           TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int             Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint32_t            Intermediate_UInt; ///< used as intermediate value in calculations
#endif

typedef       uint64_t          SplitSeries;       ///< used to encoded the splits that caused a particular CU size
#if JVET_O0050_LOCAL_DUAL_TREE
typedef       uint64_t          ModeTypeSeries;    ///< used to encoded the ModeType at different split depth
#endif

typedef       uint64_t        Distortion;        ///< distortion measurement

// ====================================================================================================================
// Enumeration
// ====================================================================================================================
#if JVET_O0245_VPS_DPS_APS
enum ApsType
#else
enum ApsTypeValues
#endif
{
  ALF_APS = 0,
  LMCS_APS = 1,
#if JVET_O0299_APS_SCALINGLIST
  SCALING_LIST_APS = 2,
#endif
};

enum QuantFlags
{
  Q_INIT           = 0x0,
  Q_USE_RDOQ       = 0x1,
  Q_RDOQTS         = 0x2,
  Q_SELECTIVE_RDOQ = 0x4,
};

//EMT transform tags
enum TransType
{
  DCT2 = 0,
  DCT8 = 1,
  DST7 = 2,
  NUM_TRANS_TYPE = 3,
  DCT2_EMT = 4
};

enum MTSIdx
{
  MTS_DCT2_DCT2 = 0,
  MTS_SKIP = 1,
  MTS_DST7_DST7 = 2,
  MTS_DCT8_DST7 = 3,
  MTS_DST7_DCT8 = 4,
  MTS_DCT8_DCT8 = 5
};

enum ISPType
{
  NOT_INTRA_SUBPARTITIONS       = 0,
  HOR_INTRA_SUBPARTITIONS       = 1,
  VER_INTRA_SUBPARTITIONS       = 2,
#if JVET_O0502_ISP_CLEANUP
  NUM_INTRA_SUBPARTITIONS_MODES = 3,
  INTRA_SUBPARTITIONS_RESERVED  = 4
#else
  NUM_INTRA_SUBPARTITIONS_MODES = 3
#endif
};

enum SbtIdx
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  NUMBER_SBT_IDX,
  SBT_OFF_MTS, //note: must be after all SBT modes, only used in fast algorithm to discern the best mode is inter EMT
};

enum SbtPos
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

enum SbtMode
{
  SBT_VER_H0 = 0,
  SBT_VER_H1 = 1,
  SBT_HOR_H0 = 2,
  SBT_HOR_H1 = 3,
  SBT_VER_Q0 = 4,
  SBT_VER_Q1 = 5,
  SBT_HOR_Q0 = 6,
  SBT_HOR_Q1 = 7,
  NUMBER_SBT_MODE
};

enum RDPCMMode
{
  RDPCM_OFF             = 0,
  RDPCM_HOR             = 1,
  RDPCM_VER             = 2,
  NUMBER_OF_RDPCM_MODES = 3
};

enum RDPCMSignallingMode
{
  RDPCM_SIGNAL_IMPLICIT            = 0,
  RDPCM_SIGNAL_EXPLICIT            = 1,
  NUMBER_OF_RDPCM_SIGNALLING_MODES = 2
};

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
};

#if JVET_O0050_LOCAL_DUAL_TREE
enum TreeType
{
  TREE_D = 0, //default tree status (for single-tree slice, TREE_D means joint tree; for dual-tree I slice, TREE_D means TREE_L for luma and TREE_C for chroma)
  TREE_L = 1, //separate tree only contains luma (may split)
  TREE_C = 2, //separate tree only contains chroma (not split), to avoid small chroma block
};

enum ModeType
{
  MODE_TYPE_ALL = 0, //all modes can try
  MODE_TYPE_INTER = 1, //can try inter
  MODE_TYPE_INTRA = 2, //can try intra, ibc, palette
};
#endif

#define CH_L CHANNEL_TYPE_LUMA
#define CH_C CHANNEL_TYPE_CHROMA

enum ComponentID
{
  COMPONENT_Y         = 0,
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT   = 3,
  JOINT_CbCr          = MAX_NUM_COMPONENT,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
};

#define MAP_CHROMA(c) (ComponentID(c))

enum InputColourSpaceConversion // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1, // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2, // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};

enum MATRIX_COEFFICIENTS // Table E.5 (Matrix coefficients)
{
  MATRIX_COEFFICIENTS_RGB                           = 0,
  MATRIX_COEFFICIENTS_BT709                         = 1,
  MATRIX_COEFFICIENTS_UNSPECIFIED                   = 2,
  MATRIX_COEFFICIENTS_RESERVED_BY_ITUISOIEC         = 3,
  MATRIX_COEFFICIENTS_USFCCT47                      = 4,
  MATRIX_COEFFICIENTS_BT601_625                     = 5,
  MATRIX_COEFFICIENTS_BT601_525                     = 6,
  MATRIX_COEFFICIENTS_SMPTE240                      = 7,
  MATRIX_COEFFICIENTS_YCGCO                         = 8,
  MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE = 9,
  MATRIX_COEFFICIENTS_BT2020_CONSTANT_LUMINANCE     = 10,
};

enum DeblockEdgeDir
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR = 2
};

/// supported prediction type
enum PredMode
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  MODE_IBC                   = 2,     ///< ibc-prediction mode
#if JVET_O0119_BASE_PALETTE_444
  MODE_PLT                   = 3,     ///< plt-prediction mode
  NUMBER_OF_PREDICTION_MODES = 4,
#else
  NUMBER_OF_PREDICTION_MODES = 3,
#endif
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0               = 0,   ///< reference list 0
  REF_PIC_LIST_1               = 1,   ///< reference list 1
  NUM_REF_PIC_LIST_01          = 2,
  REF_PIC_LIST_X               = 100  ///< special mark
};

#define L0 REF_PIC_LIST_0
#define L1 REF_PIC_LIST_1

/// distortion function index
enum DFunc
{
  DF_SSE             = 0,             ///< general size SSE
  DF_SSE2            = DF_SSE+1,      ///<   2xM SSE
  DF_SSE4            = DF_SSE+2,      ///<   4xM SSE
  DF_SSE8            = DF_SSE+3,      ///<   8xM SSE
  DF_SSE16           = DF_SSE+4,      ///<  16xM SSE
  DF_SSE32           = DF_SSE+5,      ///<  32xM SSE
  DF_SSE64           = DF_SSE+6,      ///<  64xM SSE
  DF_SSE16N          = DF_SSE+7,      ///< 16NxM SSE

  DF_SAD             = 8,             ///< general size SAD
  DF_SAD2            = DF_SAD+1,      ///<   2xM SAD
  DF_SAD4            = DF_SAD+2,      ///<   4xM SAD
  DF_SAD8            = DF_SAD+3,      ///<   8xM SAD
  DF_SAD16           = DF_SAD+4,      ///<  16xM SAD
  DF_SAD32           = DF_SAD+5,      ///<  32xM SAD
  DF_SAD64           = DF_SAD+6,      ///<  64xM SAD
  DF_SAD16N          = DF_SAD+7,      ///< 16NxM SAD

  DF_HAD             = 16,            ///< general size Hadamard
  DF_HAD2            = DF_HAD+1,      ///<   2xM HAD
  DF_HAD4            = DF_HAD+2,      ///<   4xM HAD
  DF_HAD8            = DF_HAD+3,      ///<   8xM HAD
  DF_HAD16           = DF_HAD+4,      ///<  16xM HAD
  DF_HAD32           = DF_HAD+5,      ///<  32xM HAD
  DF_HAD64           = DF_HAD+6,      ///<  64xM HAD
  DF_HAD16N          = DF_HAD+7,      ///< 16NxM HAD

  DF_SAD12           = 24,
  DF_SAD24           = 25,
  DF_SAD48           = 26,

  DF_MRSAD           = 27,            ///< general size MR SAD
  DF_MRSAD2          = DF_MRSAD+1,    ///<   2xM MR SAD
  DF_MRSAD4          = DF_MRSAD+2,    ///<   4xM MR SAD
  DF_MRSAD8          = DF_MRSAD+3,    ///<   8xM MR SAD
  DF_MRSAD16         = DF_MRSAD+4,    ///<  16xM MR SAD
  DF_MRSAD32         = DF_MRSAD+5,    ///<  32xM MR SAD
  DF_MRSAD64         = DF_MRSAD+6,    ///<  64xM MR SAD
  DF_MRSAD16N        = DF_MRSAD+7,    ///< 16NxM MR SAD

  DF_MRHAD           = 35,            ///< general size MR Hadamard
  DF_MRHAD2          = DF_MRHAD+1,    ///<   2xM MR HAD
  DF_MRHAD4          = DF_MRHAD+2,    ///<   4xM MR HAD
  DF_MRHAD8          = DF_MRHAD+3,    ///<   8xM MR HAD
  DF_MRHAD16         = DF_MRHAD+4,    ///<  16xM MR HAD
  DF_MRHAD32         = DF_MRHAD+5,    ///<  32xM MR HAD
  DF_MRHAD64         = DF_MRHAD+6,    ///<  64xM MR HAD
  DF_MRHAD16N        = DF_MRHAD+7,    ///< 16NxM MR HAD

  DF_MRSAD12         = 43,
  DF_MRSAD24         = 44,
  DF_MRSAD48         = 45,

  DF_SAD_FULL_NBIT    = 46,
  DF_SAD_FULL_NBIT2   = DF_SAD_FULL_NBIT+1,    ///<   2xM SAD with full bit usage
  DF_SAD_FULL_NBIT4   = DF_SAD_FULL_NBIT+2,    ///<   4xM SAD with full bit usage
  DF_SAD_FULL_NBIT8   = DF_SAD_FULL_NBIT+3,    ///<   8xM SAD with full bit usage
  DF_SAD_FULL_NBIT16  = DF_SAD_FULL_NBIT+4,    ///<  16xM SAD with full bit usage
  DF_SAD_FULL_NBIT32  = DF_SAD_FULL_NBIT+5,    ///<  32xM SAD with full bit usage
  DF_SAD_FULL_NBIT64  = DF_SAD_FULL_NBIT+6,    ///<  64xM SAD with full bit usage
  DF_SAD_FULL_NBIT16N = DF_SAD_FULL_NBIT+7,    ///< 16NxM SAD with full bit usage

  DF_SSE_WTD          = 54,                ///< general size SSE
  DF_SSE2_WTD         = DF_SSE_WTD+1,      ///<   4xM SSE
  DF_SSE4_WTD         = DF_SSE_WTD+2,      ///<   4xM SSE
  DF_SSE8_WTD         = DF_SSE_WTD+3,      ///<   8xM SSE
  DF_SSE16_WTD        = DF_SSE_WTD+4,      ///<  16xM SSE
  DF_SSE32_WTD        = DF_SSE_WTD+5,      ///<  32xM SSE
  DF_SSE64_WTD        = DF_SSE_WTD+6,      ///<  64xM SSE
  DF_SSE16N_WTD       = DF_SSE_WTD+7,      ///< 16NxM SSE
  DF_DEFAULT_ORI      = DF_SSE_WTD+8,

  DF_SAD_INTERMEDIATE_BITDEPTH = 63,

  DF_TOTAL_FUNCTIONS = 64
};

/// motion vector predictor direction used in AMVP
enum MvpDir
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

enum TransformDirection
{
  TRANSFORM_FORWARD              = 0,
  TRANSFORM_INVERSE              = 1,
  TRANSFORM_NUMBER_OF_DIRECTIONS = 2
};

/// supported ME search methods
enum MESearchMethod
{
  MESEARCH_FULL              = 0,
  MESEARCH_DIAMOND           = 1,
  MESEARCH_SELECTIVE         = 2,
  MESEARCH_DIAMOND_ENHANCED  = 3,
  MESEARCH_NUMBER_OF_METHODS = 4
};

/// coefficient scanning type used in ACS
enum CoeffScanType
{
  SCAN_DIAG = 0,        ///< up-right diagonal scan
#if JVET_O0119_BASE_PALETTE_444
  SCAN_TRAV_HOR = 1,
  SCAN_TRAV_VER = 2,
#endif
  SCAN_NUMBER_OF_TYPES
};

enum CoeffScanGroupType
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum ScalingListMode
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
};

enum ScalingListSize
{
  SCALING_LIST_1x1 = 0,
  SCALING_LIST_2x2,
  SCALING_LIST_4x4,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_64x64,
  SCALING_LIST_128x128,
  SCALING_LIST_SIZE_NUM,
  //for user define matrix
  SCALING_LIST_FIRST_CODED = SCALING_LIST_2x2,
  SCALING_LIST_LAST_CODED = SCALING_LIST_64x64
};

// Slice / Slice segment encoding modes
enum SliceConstraint
{
  NO_SLICES              = 0,          ///< don't use slices / slice segments
  FIXED_NUMBER_OF_CTU    = 1,          ///< Limit maximum number of largest coding tree units in a slice / slice segments
  FIXED_NUMBER_OF_BYTES  = 2,          ///< Limit maximum number of bytes in a slice / slice segment
  FIXED_NUMBER_OF_TILES  = 3,          ///< slices / slice segments span an integer number of tiles
  SINGLE_BRICK_PER_SLICE = 4,          ///< each brick is coded as separate NAL unit (slice)
  NUMBER_OF_SLICE_CONSTRAINT_MODES = 5
};

// For use with decoded picture hash SEI messages, generated by encoder.
enum HashType
{
  HASHTYPE_MD5             = 0,
  HASHTYPE_CRC             = 1,
  HASHTYPE_CHECKSUM        = 2,
  HASHTYPE_NONE            = 3,
  NUMBER_OF_HASHTYPES      = 4
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,

  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};

#define NUM_SAO_BO_CLASSES_LOG2  5
#define NUM_SAO_BO_CLASSES       (1<<NUM_SAO_BO_CLASSES_LOG2)

namespace Profile
{
  enum Name
  {
    NONE = 0,
    MAIN = 1,
    MAIN10 = 2,
    MAINSTILLPICTURE = 3,
    MAINREXT = 4,
    HIGHTHROUGHPUTREXT = 5,
    NEXT = 6
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
  };

  enum Name
  {
    // code = (level * 30)
    NONE     = 0,
    LEVEL1   = 30,
    LEVEL2   = 60,
    LEVEL2_1 = 63,
    LEVEL3   = 90,
    LEVEL3_1 = 93,
    LEVEL4   = 120,
    LEVEL4_1 = 123,
    LEVEL5   = 150,
    LEVEL5_1 = 153,
    LEVEL5_2 = 156,
    LEVEL6   = 180,
    LEVEL6_1 = 183,
    LEVEL6_2 = 186,
    LEVEL8_5 = 255,
  };
}

enum CostMode
{
  COST_STANDARD_LOSSY              = 0,
  COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  COST_LOSSLESS_CODING             = 2,
  COST_MIXED_LOSSLESS_LOSSY_CODING = 3
};

enum WeightedPredictionMethod
{
  WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT                          =0,
  WP_PER_PICTURE_WITH_SIMPLE_DC_PER_COMPONENT                               =1,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT                           =2,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING              =3,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING_AND_EXTENSION=4
};

enum FastInterSearchMode
{
  FASTINTERSEARCH_DISABLED = 0,
  FASTINTERSEARCH_MODE1    = 1, // TODO: assign better names to these.
  FASTINTERSEARCH_MODE2    = 2,
  FASTINTERSEARCH_MODE3    = 3
};

enum SPSExtensionFlagIndex
{
  SPS_EXT__REXT           = 0,
//SPS_EXT__MVHEVC         = 1, //for use in future versions
//SPS_EXT__SHVC           = 2, //for use in future versions
  SPS_EXT__NEXT           = 3,
  NUM_SPS_EXTENSION_FLAGS = 8
};

enum PPSExtensionFlagIndex
{
  PPS_EXT__REXT           = 0,
//PPS_EXT__MVHEVC         = 1, //for use in future versions
//PPS_EXT__SHVC           = 2, //for use in future versions
  NUM_PPS_EXTENSION_FLAGS = 8
};

// TODO: Existing names used for the different NAL unit types can be altered to better reflect the names in the spec.
//       However, the names in the spec are not yet stable at this point. Once the names are stable, a cleanup
//       effort can be done without use of macros to alter the names used to indicate the different NAL unit types.
enum NalUnitType
{
#if JVET_O0179
  NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  NAL_UNIT_CODED_SLICE_STSA,        // 1
  NAL_UNIT_CODED_SLICE_RASL,        // 2
  NAL_UNIT_CODED_SLICE_RADL,        // 3

  NAL_UNIT_RESERVED_VCL_4, 
  NAL_UNIT_RESERVED_VCL_5,
  NAL_UNIT_RESERVED_VCL_6,
  NAL_UNIT_RESERVED_VCL_7,

  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 8
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 9
  NAL_UNIT_CODED_SLICE_CRA,         // 10
#if JVET_N0865_GRA2GDR
  NAL_UNIT_CODED_SLICE_GDR,         // 11
#else
  NAL_UNIT_CODED_SLICE_GRA,         // 11
#endif

  NAL_UNIT_RESERVED_IRAP_VCL_12,
  NAL_UNIT_RESERVED_IRAP_VCL_13,

  NAL_UNIT_RESERVED_VCL_14,
  NAL_UNIT_RESERVED_VCL_15,

  NAL_UNIT_SPS,                     // 16
  NAL_UNIT_PPS,                     // 17
  NAL_UNIT_APS,                     // 18
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 19
  NAL_UNIT_EOS,                     // 20
  NAL_UNIT_EOB,                     // 21
  NAL_UNIT_PREFIX_SEI,              // 22
  NAL_UNIT_SUFFIX_SEI,              // 23
  NAL_UNIT_DPS,                     // 24
  NAL_UNIT_VPS,                     // 25

  NAL_UNIT_RESERVED_NVCL_26,
  NAL_UNIT_RESERVED_NVCL_27,

  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
#else
  NAL_UNIT_PPS = 0,                     // 0
  NAL_UNIT_ACCESS_UNIT_DELIMITER,       // 1
  NAL_UNIT_PREFIX_SEI,                  // 2
  NAL_UNIT_SUFFIX_SEI,                  // 3
  NAL_UNIT_APS,                         // 4
  NAL_UNIT_RESERVED_NVCL_5,             // 5
  NAL_UNIT_RESERVED_NVCL_6,             // 6
  NAL_UNIT_RESERVED_NVCL_7,             // 7
  NAL_UNIT_CODED_SLICE_TRAIL,           // 8
  NAL_UNIT_CODED_SLICE_STSA,            // 9
  NAL_UNIT_CODED_SLICE_RADL,            // 10
  NAL_UNIT_CODED_SLICE_RASL,            // 11
  NAL_UNIT_RESERVED_VCL_12,             // 12
  NAL_UNIT_RESERVED_VCL_13,             // 13
  NAL_UNIT_RESERVED_VCL_14,             // 14
  NAL_UNIT_RESERVED_VCL_15,             // 15
  NAL_UNIT_DPS,                         // 16
  NAL_UNIT_SPS,                         // 17
  NAL_UNIT_EOS,                         // 18
  NAL_UNIT_EOB,                         // 19
  NAL_UNIT_VPS,                         // 20
  NAL_UNIT_RESERVED_NVCL_21,            // 21
  NAL_UNIT_RESERVED_NVCL_22,            // 22
  NAL_UNIT_RESERVED_NVCL_23,            // 23
  NAL_UNIT_CODED_SLICE_IDR_W_RADL,      // 24
  NAL_UNIT_CODED_SLICE_IDR_N_LP,        // 25
  NAL_UNIT_CODED_SLICE_CRA,             // 26
#if JVET_N0865_GRA2GDR
  NAL_UNIT_CODED_SLICE_GDR,             // 27
#else
  NAL_UNIT_CODED_SLICE_GRA,             // 27
#endif
  NAL_UNIT_UNSPECIFIED_28,              // 29
  NAL_UNIT_UNSPECIFIED_29,              // 30
  NAL_UNIT_UNSPECIFIED_30,              // 31
  NAL_UNIT_UNSPECIFIED_31,              // 32
  NAL_UNIT_INVALID
#endif
};

#if SHARP_LUMA_DELTA_QP
enum LumaLevelToDQPMode
{
  LUMALVL_TO_DQP_DISABLED   = 0,
  LUMALVL_TO_DQP_AVG_METHOD = 1, // use average of CTU to determine luma level
  LUMALVL_TO_DQP_NUM_MODES  = 2
};
#endif

enum MergeType
{
  MRG_TYPE_DEFAULT_N        = 0, // 0
  MRG_TYPE_SUBPU_ATMVP,
  MRG_TYPE_IBC,
  NUM_MRG_TYPE                   // 5
};

enum TriangleSplit
{
  TRIANGLE_DIR_135 = 0,
  TRIANGLE_DIR_45,
  TRIANGLE_DIR_NUM
};

enum SharedMrgState
{
  NO_SHARE            = 0,
  GEN_ON_SHARED_BOUND = 1,
  SHARING             = 2
};
//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////

enum EncModeFeature
{
  ENC_FT_FRAC_BITS = 0,
  ENC_FT_DISTORTION,
  ENC_FT_RD_COST,
  ENC_FT_ENC_MODE_TYPE,
  ENC_FT_ENC_MODE_OPTS,
  ENC_FT_ENC_MODE_PART,
  NUM_ENC_FEATURES
};

enum ImvMode
{
  IMV_OFF = 0,
#if JVET_O0057_ALTHPELIF
  IMV_FPEL,
  IMV_4PEL,
  IMV_HPEL,
#else
  IMV_DEFAULT,
  IMV_4PEL,
#endif
  NUM_IMV_MODES
};


// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for adaptive loop filter
class PicSym;

#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc; // NEW, MERGE, OFF
  int typeIdc;     // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  int typeAuxInfo; // BO: starting band index
  int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  void reset();
  const SAOBlkParam& operator= (const SAOBlkParam& src);
  SAOOffset& operator[](int compIdx){ return offsetParam[compIdx];}
  const SAOOffset& operator[](int compIdx) const { return offsetParam[compIdx];}
private:
  SAOOffset offsetParam[MAX_NUM_COMPONENT];

};



struct BitDepths
{
  int recon[MAX_NUM_CHANNEL_TYPE]; ///< the bit depth as indicated in the SPS
};

#if JVET_O0119_BASE_PALETTE_444
enum PLTRunMode
{
  PLT_RUN_INDEX = 0,
  PLT_RUN_COPY  = 1,
  NUM_PLT_RUN   = 2
};
#endif
/// parameters for deblocking filter
struct LFCUParam
{
  bool internalEdge;                     ///< indicates internal edge
  bool leftEdge;                         ///< indicates left edge
  bool topEdge;                          ///< indicates top edge
};



struct PictureHash
{
  std::vector<uint8_t> hash;

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }
};

struct SEITimeSet
{
  SEITimeSet() : clockTimeStampFlag(false),
                     numUnitFieldBasedFlag(false),
                     countingType(0),
                     fullTimeStampFlag(false),
                     discontinuityFlag(false),
                     cntDroppedFlag(false),
                     numberOfFrames(0),
                     secondsValue(0),
                     minutesValue(0),
                     hoursValue(0),
                     secondsFlag(false),
                     minutesFlag(false),
                     hoursFlag(false),
                     timeOffsetLength(0),
                     timeOffsetValue(0)
  { }
  bool clockTimeStampFlag;
  bool numUnitFieldBasedFlag;
  int  countingType;
  bool fullTimeStampFlag;
  bool discontinuityFlag;
  bool cntDroppedFlag;
  int  numberOfFrames;
  int  secondsValue;
  int  minutesValue;
  int  hoursValue;
  bool secondsFlag;
  bool minutesFlag;
  bool hoursFlag;
  int  timeOffsetLength;
  int  timeOffsetValue;
};

struct SEIMasteringDisplay
{
  bool      colourVolumeSEIEnabled;
  uint32_t      maxLuminance;
  uint32_t      minLuminance;
  uint16_t    primaries[3][2];
  uint16_t    whitePoint[2];
};

#if SHARP_LUMA_DELTA_QP
struct LumaLevelToDeltaQPMapping
{
  LumaLevelToDQPMode                 mode;             ///< use deltaQP determined by block luma level
  double                             maxMethodWeight;  ///< weight of max luma value when mode = 2
  std::vector< std::pair<int, int> > mapping;          ///< first=luma level, second=delta QP.
#if ENABLE_QPA
  bool isEnabled() const { return (mode != LUMALVL_TO_DQP_DISABLED && mode != LUMALVL_TO_DQP_NUM_MODES); }
#else
  bool isEnabled() const { return mode!=LUMALVL_TO_DQP_DISABLED; }
#endif
};
#endif

#if ER_CHROMA_QP_WCG_PPS
struct WCGChromaQPControl
{
  bool isEnabled() const { return enabled; }
  bool   enabled;         ///< Enabled flag (0:default)
  double chromaCbQpScale; ///< Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale; ///< Chroma Cr QP Scale (1.0:default)
  double chromaQpScale;   ///< Chroma QP Scale (0.0:default)
  double chromaQpOffset;  ///< Chroma QP Offset (0.0:default)
};
#endif

class ChromaCbfs
{
public:
  ChromaCbfs()
    : Cb(true), Cr(true)
  {}
  ChromaCbfs( bool _cbf )
    : Cb( _cbf ), Cr( _cbf )
  {}
public:
  bool sigChroma( ChromaFormat chromaFormat ) const
  {
    if( chromaFormat == CHROMA_400 )
    {
      return false;
    }
    return   ( Cb || Cr );
  }
  bool& cbf( ComponentID compID )
  {
    bool *cbfs[MAX_NUM_TBLOCKS] = { nullptr, &Cb, &Cr };

    return *cbfs[compID];
  }
public:
  bool Cb;
  bool Cr;
};


enum MsgLevel
{
  SILENT  = 0,
  ERROR   = 1,
  WARNING = 2,
  INFO    = 3,
  NOTICE  = 4,
  VERBOSE = 5,
  DETAILS = 6
};
enum RESHAPE_SIGNAL_TYPE
{
  RESHAPE_SIGNAL_SDR = 0,
  RESHAPE_SIGNAL_PQ  = 1,
  RESHAPE_SIGNAL_HLG = 2,
  RESHAPE_SIGNAL_NULL = 100,
};


// ---------------------------------------------------------------------------
// exception class
// ---------------------------------------------------------------------------

class Exception : public std::exception
{
public:
  Exception( const std::string& _s ) : m_str( _s ) { }
  Exception( const Exception& _e ) : std::exception( _e ), m_str( _e.m_str ) { }
  virtual ~Exception() noexcept { };
  virtual const char* what() const noexcept { return m_str.c_str(); }
  Exception& operator=( const Exception& _e ) { std::exception::operator=( _e ); m_str = _e.m_str; return *this; }
  template<typename T> Exception& operator<<( T t ) { std::ostringstream oss; oss << t; m_str += oss.str(); return *this; }
private:
  std::string m_str;
};

// if a check fails with THROW or CHECK, please check if ported correctly from assert in revision 1196)
#define THROW(x)            throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define CHECK(c,x)          if(c){ THROW(x); }
#define EXIT(x)             throw( Exception( "\n" ) << x << "\n" )
#define CHECK_NULLPTR(_ptr) CHECK( !( _ptr ), "Accessing an empty pointer pointer!" )

#if !NDEBUG  // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in debug
#ifndef _DEBUG
#define _DEBUG 1
#endif
#endif

#if defined( _DEBUG )
#define CHECKD(c,x)         if(c){ THROW(x); }
#else
#define CHECKD(c,x)
#endif // _DEBUG

// ---------------------------------------------------------------------------
// static vector
// ---------------------------------------------------------------------------

template<typename T, size_t N>
class static_vector
{
  T _arr[ N ];
  size_t _size;

public:

  typedef T         value_type;
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T&        reference;
  typedef T const&  const_reference;
  typedef T*        pointer;
  typedef T const*  const_pointer;
  typedef T*        iterator;
  typedef T const*  const_iterator;

  static const size_type max_num_elements = N;

  static_vector() : _size( 0 )                                 { }
  static_vector( size_t N_ ) : _size( N_ )                     { }
  static_vector( size_t N_, const T& _val ) : _size( 0 )       { resize( N_, _val ); }
  template<typename It>
  static_vector( It _it1, It _it2 ) : _size( 0 )               { while( _it1 < _it2 ) _arr[ _size++ ] = *_it1++; }
  static_vector( std::initializer_list<T> _il ) : _size( 0 )
  {
    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }
  static_vector& operator=( std::initializer_list<T> _il )
  {
    _size = 0;

    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }

  void resize( size_t N_ )                      { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = T() ; _size = N_; }
  void resize( size_t N_, const T& _val )       { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = _val; _size = N_; }
  void reserve( size_t N_ )                     { CHECKD( N_ > N, "capacity exceeded" ); }
  void push_back( const T& _val )               { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = _val; }
  void push_back( T&& val )                     { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = std::forward<T>( val ); }
  void pop_back()                               { CHECKD( _size == 0, "calling pop_back on an empty vector" ); _size--; }
  void pop_front()                              { CHECKD( _size == 0, "calling pop_front on an empty vector" ); _size--; for( int i = 0; i < _size; i++ ) _arr[i] = _arr[i + 1]; }
  void clear()                                  { _size = 0; }
  reference       at( size_t _i )               { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference at( size_t _i ) const         { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       operator[]( size_t _i )       { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference operator[]( size_t _i ) const { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       front()                       { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  const_reference front() const                 { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  reference       back()                        { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  const_reference back() const                  { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  pointer         data()                        { return _arr; }
  const_pointer   data() const                  { return _arr; }
  iterator        begin()                       { return _arr; }
  const_iterator  begin() const                 { return _arr; }
  const_iterator  cbegin() const                { return _arr; }
  iterator        end()                         { return _arr + _size; }
  const_iterator  end() const                   { return _arr + _size; };
  const_iterator  cend() const                  { return _arr + _size; };
  size_type       size() const                  { return _size; };
  size_type       byte_size() const             { return _size * sizeof( T ); }
  bool            empty() const                 { return _size == 0; }

  size_type       capacity() const              { return N; }
  size_type       max_size() const              { return N; }
  size_type       byte_capacity() const         { return sizeof(_arr); }

  iterator        insert( const_iterator _pos, const T& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = _val;
                                                  _size++;
                                                  return const_cast<iterator>( _pos ); }

  iterator        insert( const_iterator _pos, T&& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = std::forward<T>( _val );
                                                  _size++; return const_cast<iterator>( _pos ); }
  template<class InputIt>
  iterator        insert( const_iterator _pos, InputIt first, InputIt last )
                                                { const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = const_cast<iterator>( _pos ); _size += numEl;
                                                  while( first != last ) *it++ = *first++;
                                                  return const_cast<iterator>( _pos ); }

  iterator        insert( const_iterator _pos, size_t numEl, const T& val )
                                                { //const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = const_cast<iterator>( _pos ); _size += numEl;
                                                  for ( int k = 0; k < numEl; k++) *it++ = val;
                                                  return const_cast<iterator>( _pos ); }

  void            erase( const_iterator _pos )  { iterator it   = const_cast<iterator>( _pos ) - 1;
                                                  iterator last = end() - 1;
                                                  while( ++it != last ) *it = *( it + 1 );
                                                  _size--; }
};


// ---------------------------------------------------------------------------
// dynamic cache
// ---------------------------------------------------------------------------

template<typename T>
class dynamic_cache
{
  std::vector<T*> m_cache;
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  int64_t         m_cacheId;
#endif

public:

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  dynamic_cache()
  {
    static int cacheId = 0;
    m_cacheId = cacheId++;
  }

#endif
  ~dynamic_cache()
  {
    deleteEntries();
  }

  void deleteEntries()
  {
    for( auto &p : m_cache )
    {
      delete p;
      p = nullptr;
    }

    m_cache.clear();
  }

  T* get()
  {
    T* ret;

    if( !m_cache.empty() )
    {
      ret = m_cache.back();
      m_cache.pop_back();
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
      CHECK( ret->cacheId != m_cacheId, "Putting item into wrong cache!" );
      CHECK( !ret->cacheUsed,           "Fetched an element that should've been in cache!!" );
#endif
    }
    else
    {
      ret = new T;
    }

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    ret->cacheId   = m_cacheId;
    ret->cacheUsed = false;

#endif
    return ret;
  }

  void cache( T* el )
  {
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    CHECK( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
    CHECK( el->cacheUsed,            "Putting cached item back into cache!" );

    el->cacheUsed = true;

#endif
    m_cache.push_back( el );
  }

  void cache( std::vector<T*>& vel )
  {
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    for( auto el : vel )
    {
      CHECK( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
      CHECK( el->cacheUsed,            "Putting cached item back into cache!" );

      el->cacheUsed = true;
    }

#endif
    m_cache.insert( m_cache.end(), vel.begin(), vel.end() );
    vel.clear();
  }
};

typedef dynamic_cache<struct CodingUnit    > CUCache;
typedef dynamic_cache<struct PredictionUnit> PUCache;
typedef dynamic_cache<struct TransformUnit > TUCache;

struct XUCache
{
  CUCache cuCache;
  PUCache puCache;
  TUCache tuCache;
};

#define SIGN(x) ( (x) >= 0 ? 1 : -1 )

//! \}

#endif


