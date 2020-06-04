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

 /** \file     libVTMEncoderDefaultConfigs.cpp
     \brief    Encoder library default configs
 */

#include "libVTMEncoderDefaultConfigs.h"

std::string getRandomAccessParameters(vtm_settings_t *settings)
{
    std::string parameters;

    // Todo: Apply the settings

    //#======== Profile ================
    parameters.append("Profile                       : auto\n");

    //#======== Unit definition ================
    parameters.append("MaxCUWidth                    : 64          # Maximum coding unit width in pixel\n");
    parameters.append("MaxCUHeight                   : 64          # Maximum coding unit height in pixel\n");

    //#======== Coding Structure =============
    parameters.append("IntraPeriod                   : 32          # Period of I-Frame ( -1 = only first)\n");
    parameters.append("DecodingRefreshType           : 1           # Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI\n");
    parameters.append("GOPSize                       : 16          # GOP Size (number of B slice = GOPSize-1)\n");

    parameters.append("IntraQPOffset                 : -3\n");
    parameters.append("LambdaFromQpEnable            : 1           # see JCTVC-X0038 for suitable parameters for IntraQPOffset, QPoffset, QPOffsetModelOff, QPOffsetModelScale when enabled\n");
    //#        Type POC QPoffset QPOffsetModelOff QPOffsetModelScale CbQPoffset CrQPoffset QPfactor tcOffsetDiv2 betaOffsetDiv2 CbTcOffsetDiv2 CbBetaOffsetDiv2 CrTcOffsetDiv2 CrBetaOffsetDiv2 temporal_id #ref_pics_active_L0 #ref_pics_L0   reference_pictures_L0 #ref_pics_active_L1 #ref_pics_L1   reference_pictures_L1
    parameters.append("Frame1:   B   16   1        0.0                      0.0            0          0          1.0      0            0                0             0                0               0              0             2                3          16 32 24                    2                2           16 32\n");
    parameters.append("Frame2:   B    8   1       -4.8848                   0.2061         0          0          1.0      0            0                0             0                0               0              1             2                2          8 16                        2                2           -8 8\n");
    parameters.append("Frame3:   B    4   4       -5.7476                   0.2286         0          0          1.0      0            0                0             0                0               0              2             2                2          4 12                        2                2           -4 -12\n");
    parameters.append("Frame4:   B    2   5       -5.90                     0.2333         0          0          1.0      0            0                0             0                0               0              3             2                2          2 10                        2                3           -2 -6 -14\n");
    parameters.append("Frame5:   B    1   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                2          1 -1                        2                4           -1 -3 -7 -15\n");
    parameters.append("Frame6:   B    3   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                2          1 3                         2                3           -1 -5 -13\n");
    parameters.append("Frame7:   B    6   5       -5.90                     0.2333         0          0          1.0      0            0                0             0                0               0              3             2                2          2 6                         2                2           -2 -10\n");
    parameters.append("Frame8:   B    5   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                2          1 5                         2                3           -1 -3 -11\n");
    parameters.append("Frame9:   B    7   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                3          1 3 7                       2                2           -1 -9\n");
    parameters.append("Frame10:  B   12   4       -5.7476                   0.2286         0          0          1.0      0            0                0             0                0               0              2             2                2          4 12                        2                2           -4 4\n");
    parameters.append("Frame11:  B   10   5       -5.90                     0.2333         0          0          1.0      0            0                0             0                0               0              3             2                2          2 10                        2                2           -2 -6\n");
    parameters.append("Frame12:  B    9   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                2          1 9                         2                3           -1 -3 -7\n");
    parameters.append("Frame13:  B   11   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                3          1 3 11                      2                2           -1 -5\n");
    parameters.append("Frame14:  B   14   5       -5.90                     0.2333         0          0          1.0      0            0                0             0                0               0              3             2                3          2 6 14                      2                2           -2 2\n");
    parameters.append("Frame15:  B   13   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                3          1 5 13                      2                2           -1 -3\n");
    parameters.append("Frame16:  B   15   6       -7.1444                   0.3            0          0          1.0      0            0                0             0                0               0              4             2                4          1 3 7 15                    2                2           -1 1\n");

    //#=========== Motion Search =============
    parameters.append("FastSearch                    : 1           # 0:Full search  1:TZ search\n");
    parameters.append("SearchRange                   : 384         # (0: Search range is a Full frame)\n");
    parameters.append("ASR                           : 1           # Adaptive motion search range\n");
    parameters.append("MinSearchWindow               : 96          # Minimum motion search window size for the adaptive window ME\n");
    parameters.append("BipredSearchRange             : 4           # Search range for bi-prediction refinement\n");
    parameters.append("HadamardME                    : 1           # Use of hadamard measure for fractional ME\n");
    parameters.append("FEN                           : 1           # Fast encoder decision\n");
    parameters.append("FDM                           : 1           # Fast Decision for Merge RD cost\n");

    //#======== Quantization =============
    parameters.append("QP                            : 32          # Quantization parameter(0-51)\n");
    parameters.append("MaxDeltaQP                    : 0           # CU-based multi-QP optimization\n");
    parameters.append("MaxCuDQPSubdiv                : 0           # Maximum subdiv for CU luma Qp adjustment\n");
    parameters.append("DeltaQpRD                     : 0           # Slice-based multi-QP optimization\n");
    parameters.append("RDOQ                          : 1           # RDOQ\n");
    parameters.append("RDOQTS                        : 1           # RDOQ for transform skip\n");

    //#=========== Deblock Filter ============
    parameters.append("LoopFilterOffsetInPPS         : 1           # Dbl params: 0=varying params in SliceHeader, param = base_param + GOP_offset_param; 1 (default) =constant params in PPS, param = base_param)\n");
    parameters.append("LoopFilterDisable             : 0           # Disable deblocking filter (0=Filter, 1=No Filter)\n");
    parameters.append("LoopFilterBetaOffset_div2     : 0           # base_param: -12 ~ 12\n");
    parameters.append("LoopFilterTcOffset_div2       : 0           # base_param: -12 ~ 12\n");
    parameters.append("LoopFilterCbBetaOffset_div2   : 0           # base_param: -12 ~ 12\n");
    parameters.append("LoopFilterCbTcOffset_div2     : 0           # base_param: -12 ~ 12\n");
    parameters.append("LoopFilterCrBetaOffset_div2   : 0           # base_param: -12 ~ 12\n");
    parameters.append("LoopFilterCrTcOffset_div2     : 0           # base_param: -12 ~ 12\n");
    parameters.append("DeblockingFilterMetric        : 0           # blockiness metric (automatically configures deblocking parameters in bitstream). Applies slice-level loop filter offsets (LoopFilterOffsetInPPS and LoopFilterDisable must be 0)\n");

    //#=========== Misc. ============
    parameters.append("InternalBitDepth              : 10          # codec operating bit-depth\n");

    //#=========== Coding Tools =================
    parameters.append("SAO                           : 1           # Sample adaptive offset  (0: OFF, 1: ON)\n");
    parameters.append("TransformSkip                 : 1           # Transform skipping (0: OFF, 1: ON)\n");
    parameters.append("TransformSkipFast             : 1           # Fast Transform skipping (0: OFF, 1: ON)\n");
    parameters.append("TransformSkipLog2MaxSize      : 5\n");
    parameters.append("SAOLcuBoundary                : 0           # SAOLcuBoundary using non-deblocked pixels (0: OFF, 1: ON)\n");

    //#=========== TemporalFilter =================
    parameters.append("TemporalFilter                : 0           # Enable/disable GOP Based Temporal Filter\n");
    parameters.append("TemporalFilterFutureReference : 1           # Enable/disable reading future frames\n");
    parameters.append("TemporalFilterStrengthFrame8  : 0.95        # Enable filter at every 8th frame with given strength\n");
    parameters.append("TemporalFilterStrengthFrame16 : 1.5         # Enable filter at every 16th frame with given strength, longer intervals has higher priority\n");

    //#============ Rate Control ======================
    parameters.append("RateControl                         : 0                # Rate control: enable rate control\n");
    parameters.append("TargetBitrate                       : 1000000          # Rate control: target bitrate, in bps\n");
    parameters.append("KeepHierarchicalBit                 : 2                # Rate control: 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit allocation\n");
    parameters.append("LCULevelRateControl                 : 1                # Rate control: 1: LCU level RC; 0: picture level RC\n");
    parameters.append("RCLCUSeparateModel                  : 1                # Rate control: use LCU level separate R-lambda model\n");
    parameters.append("InitialQP                           : 0                # Rate control: initial QP\n");
    parameters.append("RCForceIntraQP                      : 0                # Rate control: force intra QP to be equal to initial QP\n");

    //#============ VTM settings ======================
    parameters.append("SEIDecodedPictureHash               : 0\n");
    parameters.append("CbQpOffset                          : 0\n");
    parameters.append("CrQpOffset                          : 0\n");
    parameters.append("SameCQPTablesForAllChroma           : 1\n");
    parameters.append("QpInValCb                           : 17 22 34 42\n");
    parameters.append("QpOutValCb                          : 17 23 35 39\n");
    parameters.append("ReWriteParamSets                    : 1\n");
    //#============ NEXT ====================

    //# General
    parameters.append("CTUSize                      : 128\n");
    parameters.append("LCTUFast                     : 1\n");

    parameters.append("DualITree                    : 1      # separate partitioning of luma and chroma channels for I-slices\n");
    parameters.append("MinQTLumaISlice              : 8\n");
    parameters.append("MinQTChromaISliceInChromaSamples: 4      # minimum QT size in chroma samples for chroma separate tree\n");
    parameters.append("MinQTNonISlice               : 8\n");
    parameters.append("MaxMTTHierarchyDepth         : 3\n");
    parameters.append("MaxMTTHierarchyDepthISliceL  : 3\n");
    parameters.append("MaxMTTHierarchyDepthISliceC  : 3\n");

    parameters.append("MTS                          : 1\n");
    parameters.append("MTSIntraMaxCand              : 4\n");
    parameters.append("MTSInterMaxCand              : 4\n");
    parameters.append("SBT                          : 1\n");
    parameters.append("LFNST                        : 1\n");
    parameters.append("ISP                          : 1\n");
    parameters.append("MMVD                         : 1\n");
    parameters.append("Affine                       : 1\n");
    parameters.append("SubPuMvp                     : 1\n");
    parameters.append("MaxNumMergeCand              : 6\n");
    parameters.append("LMChroma                     : 1      # use CCLM only\n");
    parameters.append("DepQuant                     : 1\n");
    parameters.append("IMV                          : 1\n");
    parameters.append("ALF                          : 1\n");
    parameters.append("BCW                          : 1\n");
    parameters.append("BcwFast                      : 1\n");
    parameters.append("BIO                          : 1\n");
    parameters.append("CIIP                         : 1\n");
    parameters.append("Geo                          : 1\n");
    parameters.append("IBC                          : 0      # turned off in CTC\n");
    parameters.append("AllowDisFracMMVD             : 1\n");
    parameters.append("AffineAmvr                   : 1\n");
    parameters.append("LMCSEnable                   : 1      # LMCS: 0: disable, 1:enable\n");
    parameters.append("LMCSSignalType               : 0      # Input signal type: 0:SDR, 1:HDR-PQ, 2:HDR-HLG\n");
    parameters.append("LMCSUpdateCtrl               : 0      # LMCS model update control: 0:RA, 1:AI, 2:LDB/LDP\n");
    parameters.append("LMCSOffset                   : 6      # chroma residual scaling offset\n");
    parameters.append("MRL                          : 1\n");
    parameters.append("MIP                          : 1\n");
    parameters.append("DMVR                         : 1\n");
    parameters.append("SMVD                         : 1\n");
    parameters.append("JointCbCr                    : 1      # joint coding of chroma residuals (if available): 0: disable, 1: enable\n");
    parameters.append("PROF                         : 1\n");

    //# Fast tools
    parameters.append("PBIntraFast                  : 1\n");
    parameters.append("ISPFast                      : 0\n");
    parameters.append("FastMrg                      : 1\n");
    parameters.append("AMaxBT                       : 1\n");
    parameters.append("FastMIP                      : 0\n");
    parameters.append("FastLFNST                    : 0\n");
    parameters.append("FastLocalDualTreeMode        : 1\n");
    parameters.append("ChromaTS                     : 1\n");

    //# Encoder optimization tools
    parameters.append("AffineAmvrEncOpt             : 1\n");
    parameters.append("MmvdDisNum                   : 6\n");

    return parameters;
}