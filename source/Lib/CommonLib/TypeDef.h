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

#define CABAC_RETRAIN                                     1 // CABAC retraining based on VTM8rc1

#define FIX_TICKET891                                     1

#define JVET_Q0044_SLICE_IDX_WITH_SUBPICS                 1 // JVET-Q0044: slice index with subpictures

#define JVET_P0125_ASPECT_TID_LAYER_ID_NUH                1 // JVET-P0125: Aspects of constraints on TemporalId and nuh_layer_id

#define JVET_Q0471_CHROMA_QT_SPLIT                        1 // JVET-Q0471: Chroma QT split

#define JVET_P0117_PTL_SCALABILITY                        1 // JVET-P0117: sps_ptl_dpb_hrd_params_present_flag related syntax change, others in JVET-Q0786

#define JVET_Q0118_CLEANUPS                               1 // JVET-Q0118: AHG8/AHG9: Scalability HLS cleanups

#define JVET_Q0416_WRAPAROUND_OFFSET                      1  //JVET-Q0416: subtract £¨CtbSizeY / MinCbSizeY + 2£© from wraparound offset before signaling

#define JVET_P0125_EOS_LAYER_SPECIFIC                     1 // JVET-P0125: Specify EOS NAL units to be layer specific

#define JVET_Q0482_REMOVE_CONSTANT_PARAMS                 1 // JVET-Q0482: Remove constant slice header parameter settings in PPS

#if JVET_Q0482_REMOVE_CONSTANT_PARAMS
#define JVET_Q0259_COLLOCATED_PIC_IN_PH                   1 // JVET-Q0259 aspect 5: Include collocated pic in PH when TMVP enabled and rpl_info_in_ph_flag is 1
#endif

#define JVET_Q0505_CHROAM_QM_SIGNALING_400                1  //JVET-Q0505: Cleanup of chroma quantization matrix signaling for 400 color format

#define JVET_Q0786_PTL_only                               1 // JVET-Q0786: modifications to VPS syntax - PTL part only (signal PTL for single layer OLSs)

#define JVET_P0116_POC_MSB                                1 // JVET-P0116: use POC MSB cycle signalling for independent layers,to support of mixed IRAP and non-IRAP pictures within an AU

#define JVET_Q0468_Q0469_MIN_LUMA_CB_AND_MIN_QT_FIX       1 // JVET-Q0468: add support of min Luma coding block size; JVET-Q0469: fix for signaling of Intra Chroma Min QT size

#define JVET_Q0260_CONFORMANCE_WINDOW_IN_SPS              1 // JVET-Q0260: Conformance cropping window in the SPS that applies to the max picture size

#define JVET_Q0210_UEK_REMOVAL                            1 // JVET-Q0210 Aspect 8: Replace uek signalling in alf_data with ue(v).

#define JVET_Q0449_RPR_NO_SMOOTHING                       1 // JVET-Q0449: Disable smoothing half-sample interpolation filter in conjunction with RPR

#define JVET_Q0493_PLT_ENCODER_LOSSLESS                   1 // JVET-Q0493: Palette encoder improvements for lossless coding

#define JVET_Q0629_REMOVAL_PLT_4X4                        1 // JVET-Q0629: Removal of 4x4 blocks in palette mode

#define JVET_Q0291_REDUCE_DUALTREE_PLT_SIZE               1 // JVET-Q0291: reduce palette size of dual tree from 31 to 15 and palette predictor size of dual tree from 63 to 31

#define JVET_Q0330_BLOCK_PARTITION                        1 // JVET-Q0330: fix the block partitioning at picture boundary

#define JVET_Q0417_CONSTRAINT_SPS_VB_PRESENT_FLAG         1 // JVET_Q0417: a constraint on the value of sps_virtual_boundaries_present_flag based on res_change_in_clvs_allowed_flag

#define JVET_Q0179_SCALING_WINDOW_SIZE_CONSTRAINT         1 // JVET-Q0179: Scaling window size constraint for constraining worst case memory bandwidth

#define JVET_P2008_OUTPUT_LOG                             1 // Output log file for conformance tests

#define JVET_P0097_REMOVE_VPS_DEP_NONSCALABLE_LAYER       1 // Removing dependencies on VPS from the decoding process of a non-scalable bitstream

#define JVET_Q0420_PPS_CHROMA_TOOL_FLAG                   1 // JVET-Q0420: add pps_chroma_tool_offsets_present_flag in PPS

#define JVET_Q0172_CHROMA_FORMAT_BITDEPTH_CONSTRAINT      1 // JVET-Q0172: Disallow differing chroma format and different bit depths for cross-layer prediction. 

#define JVET_Q0491_PLT_ESCAPE                             1 // JVET-Q0491: Palette escape binarization

#define JVET_Q414_CONSTRAINT_ON_GDR_PIC_FLAG              1  //JVET-Q0414: when gdr_enabled_flag is equal to 0, gdr_pic_flag shall be 0

#define JVET_Q0817                                        1 // JVET_Q0817: Remove the constraint on single_slice_per_subpic_flag

#define JVET_Q0413_SKIP_LAST_SUBPIC_SIG                   1  //JVET-Q0413 modification 2: skip the width and height signaling of last subpicture

#define JVET_Q0169_SUBPIC_LEN_CONFORM                     1 // JVET-Q0169: add bitstream conformance check on subpic length

#define JVET_Q0504_PLT_NON444                             1 // JVET-Q0504: enable palette mode for non 444 color format

#define JVET_Q0183_SPS_TRANSFORM_SKIP_MODE_CONTROL        1 // JVET-Q0183: Signal the max block size in SPS and conditionally signal min_qp_prime_ts_minus4

#define JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM     1 // JVET-Q0089: RRC slice-level switch for lossless coding and one SPS flag for luma and chroma BDPCM.

#define JVET_Q0816                                        1 // JVET_Q0816: Omit the signalling of subpic layout when there is only one subpicture

#define JVET_Q0438_MONOCHROME_BUGFIXES                    1 // JVET-Q0438: Monochrome bug fixes

#define JVET_Q0441_SAO_MOD_12_BIT                         1 // JVET-Q0441: SAO modification for 12 bit. Also removes old HEVC RExt SAO modification, which was broken.

#define JVET_Q0043_RPR_and_Subpics                        1 // JVET-Q0043: Disallow for both RPR and subpics to be used together

#define JVET_Q0818_PT_SEI                                 1 // JVET-Q0818: add display_elemental_periods_minus1 to picture timing SEI message

#define JVET_Q0110_Q0785_CHROMA_BDPCM_420                 1 // JVET-Q0110/Q0785: Enable chroma BDPCM for 420, separate contexts for chroma BDPCM and bug-fixes.

#define JVET_Q0042_VUI                                    1 // Modifications to VUI syntax

#define JVET_Q0512_ENC_CHROMA_TS_ACT                      1 // JVET-Q0512: encoder-side improvement on enabling chroma transform-skip for ACT

#define JVET_P0101_POC_MULTILAYER                         1 // POC derivation for pictures in dependent layers

#define JVET_Q0446_MIP_CONST_SHIFT_OFFSET                 1 // JVET-Q0446: MIP with constant shift and offset

#define JVET_Q0265                                        1 // JVET-Q0265: Cleanup for monochrome and independently coded color planes

#define JVET_Q0447_WP_PARAM_ESTIM                         1 // JVET-Q0447: Add search iterations for method 2,3 and 4

#define JVET_Q0119_CLEANUPS                               1 // JVET-Q0119: AHG12: Cleanups on signalling of subpictures, tiles, and rectangular slices

#define JVET_Q0114_CONSTRAINT_FLAGS                       1 // JVET-Q0114: AHG9: A few more general constraints flags

#define JVET_Q0820_ACT                                    1 // JVET-Q0820: ACT bug fixes and reversible ACT transform 

#define JVET_Q0814_DPB                                    1 // JVET-Q0814: DPB capacity is based on picture units regardless of the resoltuion
#define ENABLING_MULTI_SPS                                1 // Bug fix to enable multiple SPS
#define SPS_ID_CHECK                                      1 // add SPS id check to be the same within CLVS, related to mixed_nalu_types_in_pic_flag
#define JVET_Q0117_PARAMETER_SETS_CLEANUP                 1 // JVET-Q0117: cleanups on parameter sets


#define JVET_Q0353_ACT_SW_FIX                             1 // JVET-Q0353: Bug fix of ACT 

#define JVET_P0288_PIC_OUTPUT                             1 // JVET-P0288: Set the value of PictureOutputFlag

#define JVET_Q0695_CHROMA_TS_JCCR                         1 // JVET-Q0695: Enabling the RD checking of chroma transform-skip mode for JCCR at encoder
#define JVET_Q0500_CCLM_REF_PADDING                       1 // JVET-Q0500: Reference samples padding for CCLM

#define JVET_Q0128_DMVR_BDOF_ENABLING_CONDITION           1 // JVET-Q0128: Cleanup of enabling condition for DMVR and BDOF 

#define JVET_Q0784_LFNST_COMBINATION                      1 // lfnst signaling, latency reduction and a bugfix for scaling from Q0106, Q0686, Q0133

#define JVET_Q0501_PALETTE_WPP_INIT_ABOVECTU              1 // JVET-Q0501: Initialize palette predictor from above CTU row in WPP 

#define JVET_Q0503_Q0712_PLT_ENCODER_IMPROV_BUGFIX        1 // JVET-Q0503/Q0712: Platte encoder improvement/bugfix

#define JVET_Q0819_PH_CHANGES                             1 // JVET-Q0819: Combination of PH related syntax changes

#define JVET_Q0481_PARTITION_CONSTRAINTS_ORDER            1 // JVET-Q0481: Ordering of partition constraints syntax elements in the SPS

#define JVET_Q0155_COLOUR_ID                              1 // JVET-Q0155: move colour_plane_id from PH to SH

#define JVET_Q0444_AMVR_SIGNALLING                        1 // JVET-Q0444: Conditional signaling of sps_affine_amvr_enabled_flag based on sps_amvr_enabled_flag

#define JVET_Q0147_JCCR_SIGNALLING                        1 // JVET-Q0147: Conditional signaling of sps_joint_cbcr_enabled_flag based on ChromaArrayType

#define JVET_Q0267_RESET_CHROMA_QP_OFFSET                 1 // JVET-Q0267: Reset chroma QP offsets at the start of each chroma QP offset group

#define JVET_Q0293_REMOVAL_PDPC_CHROMA_NX2                1 // JVET-Q0293: Removal of chroma Nx2 blocks in PDPC 

#define JVET_Q0121_DEBLOCKING_CONTROL_PARAMETERS          1 // JVET-Q0121: Add deblocking control parameters for Cb and Cr and extend the parameter ranges

#define JVET_Q0517_RPR_AFFINE_DS                          1 // JVET-Q0517: affine down-sampling filters for RPR

#define JVET_Q0151_Q0205_ENTRYPOINTS                      1 // JVET-Q0151 & JVET-Q0205: Make mandatory the tile offsets signalling and move the entropy_coding_sync_enabled_flag entry_point_offsets_present_flag syntax elements to the SPS from the PPS

#define JVET_Q0203_MULTI_SLICE_IN_TILE                    1 // JVET-Q0203: Signalling of multiple rectangular slices within a tile

#define JVET_O1143_SUBPIC_BOUNDARY                        1 // treat subpicture boundary as picture boundary

#if JVET_O1143_SUBPIC_BOUNDARY
#define JVET_O1143_LPF_ACROSS_SUBPIC_BOUNDARY             1 
#define JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY              1
#endif

#define JVET_Q0495_NLALF_CLIP_CLEANUP                     1 // JVET-Q0495: Cleanup of clipping table for NL-ALF

#define JVET_Q0156_STSA                                   1 // JVET-Q0156: Enable inter-layer prediction for STSA pictures

#define JVET_Q0249_ALF_CHROMA_CLIPFLAG                    1 // JVET-Q0249: Cleanup of chroma clipping flags for ALF
#define JVET_Q0150                                        1 // fix for ALF virtual horizontal CTU boundary processing
#define JVET_Q0054                                        1 // fix for long luma deblocking decision
#define JVET_Q0795_CCALF                                  1 // Cross-component ALF

#define JVET_Q0297_MER                                    1 // JVET_Q0297: Merge estimation region

#define JVET_Q0483_CLIP_TMVP                              1 // JVET-Q0483: Clip TMVP when no scaling is applied

#define JVET_Q0516_MTS_SIGNALLING_DC_ONLY_COND            1 // JVET-Q0516/Q0685: disable MTS when there is only DC coefficient 

#define JVET_Q0806                                        1 // Geo related adoptions (JVET-Q0059, JVET-Q0077, JVET-Q0123, JVET-Q0188, JVET-Q0242_GEO, JVET-Q0309, JVET-Q0365 and JVET-Q0370)

#define JVET_Q0055_MTS_SIGNALLING                         1 // JVET-Q0055: Check for transform coefficients outside the 16x16 area
#define JVET_Q0480_RASTER_RECT_SLICES                     1 // JVET-Q0480: Eliminate redundant slice height syntax when in raster rectangular slice mode (tile_idx_delta_present_flag == 0)

#define JVET_Q0775_PH_IN_SH                               1 // JVET-Q0755: Allow picture header in slice header

#define JVET_Q0433_MODIFIED_CHROMA_DIST_WEIGHT            1 // modification of chroma distortion weight (as agreed during presentation of JVET-Q0433)

#define JVET_P0188_MINCR                                  1 // JVET-P0188: Add MinCR checking in encoder.
#define JVET_Q0436_CABAC_ZERO_WORD                        1 // JVET-Q0436: Add modified CABAC zero word insertion in encoder.

#define JVET_Q0487_SCALING_WINDOW_ISSUES                  1 // JVET-Q0487: Fix scaling window issues when scaling ratio is 1:1

#define JVET_Q0787_SUBPIC                                 1 // JVET-Q0787: fix subpicture location signalling

#define JVET_Q0400_EXTRA_BITS                             1 // JVET-Q0400: reserved bits for future extensions

#define JVET_AHG14_LOSSLESS                               1
#define JVET_AHG14_LOSSLESS_ENC_QP_FIX                    1 && JVET_AHG14_LOSSLESS

#define JVET_M0497_MATRIX_MULT                            0 // 0: Fast method; 1: Matrix multiplication

#define APPLY_SBT_SL_ON_MTS                               1 // apply save & load fast algorithm on inter MTS when SBT is on


typedef std::pair<int, bool> TrMode;
typedef std::pair<int, int>  TrCost;

// clang-format off
#define REUSE_CU_RESULTS                                  1 
#if REUSE_CU_RESULTS
#define REUSE_CU_RESULTS_WITH_MULTIPLE_TUS                1
#endif
// clang-format on

#define JVET_P0190_SCALABLE_NESTING_SEI                   1 // JVET-P0190 scalable nesting sei message

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
#define ENABLE_SIMD_OPT_BCW                               1                                                 ///< SIMD optimization for Bcw
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
typedef       uint64_t          ModeTypeSeries;    ///< used to encoded the ModeType at different split depth

typedef       uint64_t        Distortion;        ///< distortion measurement

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

#if !JVET_Q0089_SLICE_LOSSLESS_CODING_CHROMA_BDPCM
enum BDPCMControl
{
  BDPCM_INACTIVE = 0,
  BDPCM_LUMAONLY = 1,
  BDPCM_LUMACHROMA = 2,
};
#endif

enum ApsType
{
  ALF_APS = 0,
  LMCS_APS = 1,
  SCALING_LIST_APS = 2,
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
  NUM_INTRA_SUBPARTITIONS_MODES = 3,
  INTRA_SUBPARTITIONS_RESERVED  = 4
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

#if JVET_Q0265
/// chroma formats (according to how the monochrome or the color planes are intended to be coded)
#else
/// chroma formats (according to semantics of chroma_format_idc)
#endif
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
  MODE_PLT                   = 3,     ///< plt-prediction mode
  NUMBER_OF_PREDICTION_MODES = 4,
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

#if JVET_Q0806
  DF_SAD_WITH_MASK   = 64,
  DF_TOTAL_FUNCTIONS = 65
#else
  DF_TOTAL_FUNCTIONS = 64
#endif
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
  SCAN_TRAV_HOR = 1,
  SCAN_TRAV_VER = 2,
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

enum ScalingList1dStartIdx
{
  SCALING_LIST_1D_START_2x2    = 0,
  SCALING_LIST_1D_START_4x4    = 2,
  SCALING_LIST_1D_START_8x8    = 8,
  SCALING_LIST_1D_START_16x16  = 14,
  SCALING_LIST_1D_START_32x32  = 20,
  SCALING_LIST_1D_START_64x64  = 26,
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
    NONE        = 0,
    MAIN_10     = 1,
    MAIN_444_10 = 2
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
    NUMBER_OF_TIERS=2
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
  NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  NAL_UNIT_CODED_SLICE_STSA,        // 1
  NAL_UNIT_CODED_SLICE_RADL,        // 2
  NAL_UNIT_CODED_SLICE_RASL,        // 3

  NAL_UNIT_RESERVED_VCL_4,
  NAL_UNIT_RESERVED_VCL_5,
  NAL_UNIT_RESERVED_VCL_6,

  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  NAL_UNIT_CODED_SLICE_CRA,         // 9
  NAL_UNIT_CODED_SLICE_GDR,         // 10

  NAL_UNIT_RESERVED_IRAP_VCL_11,
  NAL_UNIT_RESERVED_IRAP_VCL_12,

#if JVET_Q0117_PARAMETER_SETS_CLEANUP
  NAL_UNIT_DCI,                     // 13
#else
  NAL_UNIT_DPS,                     // 13
#endif
  NAL_UNIT_VPS,                     // 14
  NAL_UNIT_SPS,                     // 15
  NAL_UNIT_PPS,                     // 16
  NAL_UNIT_PREFIX_APS,              // 17
  NAL_UNIT_SUFFIX_APS,              // 18
  NAL_UNIT_PH,                      // 19
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  NAL_UNIT_EOS,                     // 21
  NAL_UNIT_EOB,                     // 22
  NAL_UNIT_PREFIX_SEI,              // 23
  NAL_UNIT_SUFFIX_SEI,              // 24
  NAL_UNIT_FD,                      // 25

  NAL_UNIT_RESERVED_NVCL_26,
  NAL_UNIT_RESERVED_NVCL_27,

  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
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

#if !JVET_Q0806
enum TriangleSplit
{
  TRIANGLE_DIR_135 = 0,
  TRIANGLE_DIR_45,
  TRIANGLE_DIR_NUM
};
#endif

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
  IMV_FPEL,
  IMV_4PEL,
  IMV_HPEL,
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

enum PLTRunMode
{
  PLT_RUN_INDEX = 0,
  PLT_RUN_COPY  = 1,
  NUM_PLT_RUN   = 2
};
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
#if ENABLE_SPLIT_PARALLELISM
  int64_t         m_cacheId;
#endif

public:

#if ENABLE_SPLIT_PARALLELISM
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
#if ENABLE_SPLIT_PARALLELISM
      CHECK( ret->cacheId != m_cacheId, "Putting item into wrong cache!" );
      CHECK( !ret->cacheUsed,           "Fetched an element that should've been in cache!!" );
#endif
    }
    else
    {
      ret = new T;
    }

#if ENABLE_SPLIT_PARALLELISM
    ret->cacheId   = m_cacheId;
    ret->cacheUsed = false;

#endif
    return ret;
  }

  void cache( T* el )
  {
#if ENABLE_SPLIT_PARALLELISM
    CHECK( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
    CHECK( el->cacheUsed,            "Putting cached item back into cache!" );

    el->cacheUsed = true;

#endif
    m_cache.push_back( el );
  }

  void cache( std::vector<T*>& vel )
  {
#if ENABLE_SPLIT_PARALLELISM
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


