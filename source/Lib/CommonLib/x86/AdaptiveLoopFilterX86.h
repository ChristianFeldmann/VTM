/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

/** \file     AdaptiveLoopFilterX86.h
    \brief    adaptive loop filter class
*/
#include "CommonDefX86.h"
#include "../AdaptiveLoopFilter.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86
#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

template<X86_VEXT vext>
static void simdDeriveClassificationBlk(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS],
                                        const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift,
                                        const int vbCTUHeight, int vbPos)
{
  CHECK((blk.height & 7) != 0, "Block height must be a multiple of 8");
  CHECK((blk.width & 7) != 0, "Block width must be a multiple of 8");
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  const size_t imgStride = srcLuma.stride;
  const Pel *  srcExt    = srcLuma.buf;

  const int imgHExtended = blk.height + 4;
  const int imgWExtended = blk.width + 4;

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;

  // 18x40 array
  uint16_t colSums[(AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4) >> 1]
                  [AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 8];

  for (int i = 0; i < imgHExtended; i += 2)
  {
    const size_t offset = (i + posY - 3) * imgStride + posX - 3;

    const Pel *imgY0 = &srcExt[offset];
    const Pel *imgY1 = &srcExt[offset + imgStride];
    const Pel *imgY2 = &srcExt[offset + imgStride * 2];
    const Pel *imgY3 = &srcExt[offset + imgStride * 3];

    // pixel padding for gradient calculation
    int pos      = blkDst.pos().y - 2 + i;
    int posInCTU = pos & (vbCTUHeight - 1);
    if (pos > 0 && posInCTU == vbPos - 2)
    {
      imgY3 = imgY2;
    }
    else if (pos > 0 && posInCTU == vbPos)
    {
      imgY0 = imgY1;
    }

    __m128i prev = _mm_setzero_si128();

    for (int j = 0; j < imgWExtended; j += 8)
    {
      const __m128i x0 = _mm_loadu_si128((const __m128i *) (imgY0 + j));
      const __m128i x1 = _mm_loadu_si128((const __m128i *) (imgY1 + j));
      const __m128i x2 = _mm_loadu_si128((const __m128i *) (imgY2 + j));
      const __m128i x3 = _mm_loadu_si128((const __m128i *) (imgY3 + j));

      const __m128i x4 = _mm_loadu_si128((const __m128i *) (imgY0 + j + 2));
      const __m128i x5 = _mm_loadu_si128((const __m128i *) (imgY1 + j + 2));
      const __m128i x6 = _mm_loadu_si128((const __m128i *) (imgY2 + j + 2));
      const __m128i x7 = _mm_loadu_si128((const __m128i *) (imgY3 + j + 2));

      const __m128i nw = _mm_blend_epi16(x0, x1, 0xaa);
      const __m128i n  = _mm_blend_epi16(x0, x5, 0x55);
      const __m128i ne = _mm_blend_epi16(x4, x5, 0xaa);
      const __m128i w  = _mm_blend_epi16(x1, x2, 0xaa);
      const __m128i e  = _mm_blend_epi16(x5, x6, 0xaa);
      const __m128i sw = _mm_blend_epi16(x2, x3, 0xaa);
      const __m128i s  = _mm_blend_epi16(x2, x7, 0x55);
      const __m128i se = _mm_blend_epi16(x6, x7, 0xaa);

      __m128i c = _mm_blend_epi16(x1, x6, 0x55);
      c         = _mm_add_epi16(c, c);
      __m128i d = _mm_shuffle_epi8(c, _mm_setr_epi8(2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13));

      const __m128i ver = _mm_abs_epi16(_mm_sub_epi16(c, _mm_add_epi16(n, s)));
      const __m128i hor = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(w, e)));
      const __m128i di0 = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(nw, se)));
      const __m128i di1 = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(ne, sw)));

      const __m128i hv  = _mm_hadd_epi16(ver, hor);
      const __m128i di  = _mm_hadd_epi16(di0, di1);
      const __m128i all = _mm_hadd_epi16(hv, di);

      const __m128i t = _mm_blend_epi16(all, prev, 0xaa);
      _mm_storeu_si128((__m128i *) &colSums[i >> 1][j], _mm_hadd_epi16(t, all));
      prev = all;
    }
  }

  for (int i = 0; i < (blk.height >> 1); i += 4)
  {
    for (int j = 0; j < blk.width; j += 8)
    {
      __m128i x0, x1, x2, x3, x4, x5, x6, x7;

      const uint32_t z  = (2 * i + blkDst.pos().y) & (vbCTUHeight - 1);
      const uint32_t z2 = (2 * i + 4 + blkDst.pos().y) & (vbCTUHeight - 1);

      x0 = (z == vbPos) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 0][j + 4]);
      x1 = _mm_loadu_si128((__m128i *) &colSums[i + 1][j + 4]);
      x2 = _mm_loadu_si128((__m128i *) &colSums[i + 2][j + 4]);
      x3 = (z == vbPos - 4) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 3][j + 4]);

      x4 = (z2 == vbPos) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 2][j + 4]);
      x5 = _mm_loadu_si128((__m128i *) &colSums[i + 3][j + 4]);
      x6 = _mm_loadu_si128((__m128i *) &colSums[i + 4][j + 4]);
      x7 = (z2 == vbPos - 4) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 5][j + 4]);

      x0 = _mm_add_epi16(x0, x1);
      x2 = _mm_add_epi16(x2, x3);
      x0 = _mm_add_epi16(x0, x2);

      x4 = _mm_add_epi16(x4, x5);
      x6 = _mm_add_epi16(x6, x7);
      x4 = _mm_add_epi16(x4, x6);

      x1 = _mm_unpacklo_epi16(x0, x4);
      x5 = _mm_unpackhi_epi16(x0, x4);
      x0 = _mm_unpacklo_epi16(x1, x5);
      x4 = _mm_unpackhi_epi16(x1, x5);

      __m128i sumV  = _mm_cvtepu16_epi32(x0);
      __m128i sumH  = _mm_unpackhi_epi16(x0, _mm_setzero_si128());
      __m128i sumD0 = _mm_cvtepu16_epi32(x4);
      __m128i sumD1 = _mm_unpackhi_epi16(x4, _mm_setzero_si128());

      //      uint32_t tempAct = sumV + sumH;
      __m128i tempAct = _mm_add_epi32(sumV, sumH);

      //      const uint32_t activity = std::min<uint32_t>(15, tempAct * scale >> shift);
      //      static const uint8_t th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
      //      uint8_t classIdx = th[activity];
      const uint32_t scale  = (z == vbPos - 4 || z == vbPos) ? 96 : 64;
      const uint32_t scale2 = (z2 == vbPos - 4 || z2 == vbPos) ? 96 : 64;
      __m128i activity = _mm_mullo_epi32(tempAct, _mm_unpacklo_epi64(_mm_set1_epi32(scale), _mm_set1_epi32(scale2)));
      activity         = _mm_srl_epi32(activity, _mm_cvtsi32_si128(shift));
      activity         = _mm_min_epi32(activity, _mm_set1_epi32(15));
      __m128i classIdx = _mm_shuffle_epi8(_mm_setr_epi8(0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4), activity);

      //      if (sumV > sumH)
      //      {
      //        hv1       = sumV;
      //        hv0       = sumH;
      //        dirTempHV = 0;
      //      }
      //      else
      //      {
      //        hv1       = sumH;
      //        hv0       = sumV;
      //        dirTempHV = 1;
      //      }
      __m128i dirTempHVMinus1 = _mm_cmpgt_epi32(sumV, sumH);
      __m128i hv1             = _mm_max_epi32(sumV, sumH);
      __m128i hv0             = _mm_min_epi32(sumV, sumH);

      //      if (sumD0 > sumD1)
      //      {
      //        d1       = sumD0;
      //        d0       = sumD1;
      //        dirTempD = 0;
      //      }
      //      else
      //      {
      //        d1       = sumD1;
      //        d0       = sumD0;
      //        dirTempD = 1;
      //      }
      __m128i dirTempDMinus1 = _mm_cmpgt_epi32(sumD0, sumD1);
      __m128i d1             = _mm_max_epi32(sumD0, sumD1);
      __m128i d0             = _mm_min_epi32(sumD0, sumD1);

      //      int dirIdx;
      //      if (d1 * hv0 > hv1 * d0)
      //      {
      //        hvd1   = d1;
      //        hvd0   = d0;
      //        dirIdx = 0;
      //      }
      //      else
      //      {
      //        hvd1   = hv1;
      //        hvd0   = hv0;
      //        dirIdx = 2;
      //      }
      __m128i a      = _mm_xor_si128(_mm_mullo_epi32(d1, hv0), _mm_set1_epi32(0x80000000));
      __m128i b      = _mm_xor_si128(_mm_mullo_epi32(hv1, d0), _mm_set1_epi32(0x80000000));
      __m128i dirIdx = _mm_cmpgt_epi32(a, b);
      __m128i hvd1   = _mm_blendv_epi8(hv1, d1, dirIdx);
      __m128i hvd0   = _mm_blendv_epi8(hv0, d0, dirIdx);

      //      if (hvd1 * 2 > 9 * hvd0)
      //      {
      //        classIdx += (dirIdx + 2) * 5;
      //      }
      //      else if (hvd1 > 2 * hvd0)
      //      {
      //        classIdx += (dirIdx + 1) * 5;
      //      }
      __m128i strength1 = _mm_cmpgt_epi32(hvd1, _mm_add_epi32(hvd0, hvd0));
      __m128i strength2 = _mm_cmpgt_epi32(_mm_add_epi32(hvd1, hvd1), _mm_add_epi32(hvd0, _mm_slli_epi32(hvd0, 3)));
      __m128i offset    = _mm_and_si128(strength1, _mm_set1_epi32(5));
      classIdx          = _mm_add_epi32(classIdx, offset);
      classIdx          = _mm_add_epi32(classIdx, _mm_and_si128(strength2, _mm_set1_epi32(5)));
      offset            = _mm_andnot_si128(dirIdx, offset);
      offset            = _mm_add_epi32(offset, offset);
      classIdx          = _mm_add_epi32(classIdx, offset);

      //      uint8_t transposeIdx = 2 * dirTempD + dirTempHV;
      __m128i transposeIdx = _mm_set1_epi32(3);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempHVMinus1);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempDMinus1);

      int yOffset = 2 * i + blkDst.pos().y;
      int xOffset = j + blkDst.pos().x;

      static_assert(sizeof(AlfClassifier) == 2, "ALFClassifier type must be 16 bits wide");
      __m128i v;
      v = _mm_unpacklo_epi8(classIdx, transposeIdx);
      v = _mm_shuffle_epi8(v, _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 8, 9, 8, 9, 8, 9, 8, 9));
      _mm_storeu_si128((__m128i *) (classifier[yOffset] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 1] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 2] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 3] + xOffset), v);
      v = _mm_unpackhi_epi8(classIdx, transposeIdx);
      v = _mm_shuffle_epi8(v, _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 8, 9, 8, 9, 8, 9, 8, 9));
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 4] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 5] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 6] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 7] + xOffset), v);
    }
  }
}

template<X86_VEXT vext>
static void simdFilter5x5Blk(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc,
                             const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
                             const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
                             int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  const bool bChroma = isChroma( compId );

  const SPS*     sps = cs.slice->getSPS();
  bool isDualTree = CS::isDualITree(cs);
  bool isPCMFilterDisabled = sps->getPCMFilterDisableFlag();
  ChromaFormat nChromaFormat = sps->getChromaFormatIdc();

  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const int numBitsMinus1 = AdaptiveLoopFilter::m_NUM_BITS - 1;
  const int offset = ( 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 2 ) );

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4;

  const short *coef[2] = { filterSet, filterSet };
  const short *clip[2] = { fClipSet, fClipSet };

  int transposeIdx[2] = {0, 0};
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  bool pcmFlags2x2[8] = {0,0,0,0,0,0,0,0};
  Pel  pcmRec2x2[32];

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  const __m128i mmOffset = _mm_set1_epi32( offset );
  const __m128i mmMin = _mm_set1_epi16( clpRng.min );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max );

  const unsigned char *filterCoeffIdx[2];
  Pel filterCoeff[MAX_NUM_ALF_LUMA_COEFF][2];
  Pel filterClipp[MAX_NUM_ALF_LUMA_COEFF][2];

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;

  Pel* pRec0 = dst + blkDst.x;
  Pel* pRec1;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    if( !bChroma )
    {
      pClass = classifier[blkDst.y + i] + blkDst.x;
    }

    for( int j = 0; j < endWidth - startWidth; j += 8 )
    {
      for( int k = 0; k < 2; ++k )
      {
        if( !bChroma )
        {
          const AlfClassifier& cl = pClass[j+4*k];
          transposeIdx[k] = cl.transposeIdx;
          coef[k] = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          clip[k] = fClipSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          if ( isPCMFilterDisabled && cl.classIdx == AdaptiveLoopFilter::m_ALF_UNUSED_CLASSIDX && transposeIdx[k] == AdaptiveLoopFilter::m_ALF_UNUSED_TRANSPOSIDX )
          {
            // Note that last one (i.e. filterCoeff[6][k]) is not unused with JVET_N0242_NON_LINEAR_ALF; could be simplified
            static const unsigned char _filterCoeffIdx[7] = { 0, 0, 0, 0, 0, 0, 0 };
            static short _identityFilterCoeff[] = { 0 };
            static short _identityFilterClipp[] = { 0 };
            filterCoeffIdx[k] = _filterCoeffIdx;
            coef[k] = _identityFilterCoeff;
            clip[k] = _identityFilterClipp;
          }
          else if( transposeIdx[k] == 1 )
          {
            static const unsigned char _filterCoeffIdx[7] = { 4, 1, 5, 3, 0, 2, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 2 )
          {
            static const unsigned char _filterCoeffIdx[7] = { 0, 3, 2, 1, 4, 5, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 3 )
          {
            static const unsigned char _filterCoeffIdx[7] = { 4, 3, 5, 1, 0, 2, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else
          {
            static const unsigned char _filterCoeffIdx[7] = { 0, 1, 2, 3, 4, 5, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
        }
        else
        {
          static const unsigned char _filterCoeffIdx[7] = { 0, 1, 2, 3, 4, 5, 6 };
          filterCoeffIdx[k] = _filterCoeffIdx;
        }

        for ( int i=0; i < 7; ++i )
        {
          filterCoeff[i][k] = coef[k][filterCoeffIdx[k][i]];
          filterClipp[i][k] = clip[k][filterCoeffIdx[k][i]];
        }
      }


      pRec1 = pRec0 + j;

      if ( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;

        // check which chroma 2x2 blocks use PCM
        // chroma PCM may not be aligned with 4x4 ALF processing grid
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            Position pos(j + blkDst.x + blkX, i + blkDst.y + blkY);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB && !JVET_O0050_LOCAL_DUAL_TREE
            const CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#else
            CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
            cu = cu->isSepTree() ? cs.getCU( pos, CH_C ) : cu;
#endif
            if(cu != NULL)
            {
              *flags++ = cu->ipcm ? 1 : 0;
            }
            else
            {
              *flags++ = 0;
            }

            // save original samples from 2x2 PCM blocks
            if( cu != NULL && cu->ipcm )
            {
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+1)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+1)];
            }
          }
        }
      }

      __m128i xmmNull = _mm_setzero_si128();

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;

        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - (bChroma ? 2 : 4)))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + (bChroma ? 1 : 3)))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
        }
        __m128i clipp, clipm;
        __m128i coeffa, coeffb;
        __m128i xmmCur = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 0 ) );

        // coeff 0 and 1
        __m128i xmm00 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 + 0 ) );
        xmm00 = _mm_sub_epi16( xmm00, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[0][0], filterClipp[0][0], filterClipp[0][0], filterClipp[0][0],
                                filterClipp[0][1], filterClipp[0][1], filterClipp[0][1], filterClipp[0][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm00 = _mm_min_epi16( xmm00, clipp );
        xmm00 = _mm_max_epi16( xmm00, clipm );

        __m128i xmm01 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 + 0 ) );
        xmm01 = _mm_sub_epi16( xmm01, xmmCur );
        xmm01 = _mm_min_epi16( xmm01, clipp );
        xmm01 = _mm_max_epi16( xmm01, clipm );

        xmm00 = _mm_add_epi16( xmm00, xmm01 );

        __m128i xmm10 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 1 ) );
        xmm10 = _mm_sub_epi16( xmm10, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[1][0], filterClipp[1][0], filterClipp[1][0], filterClipp[1][0],
                                filterClipp[1][1], filterClipp[1][1], filterClipp[1][1], filterClipp[1][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm10 = _mm_min_epi16( xmm10, clipp );
        xmm10 = _mm_max_epi16( xmm10, clipm );

        __m128i xmm11 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 1 ) );
        xmm11 = _mm_sub_epi16( xmm11, xmmCur );
        xmm11 = _mm_min_epi16( xmm11, clipp );
        xmm11 = _mm_max_epi16( xmm11, clipm );

        xmm10 = _mm_add_epi16( xmm10, xmm11 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[0][0] );
        coeffb = _mm_set1_epi16( filterCoeff[1][0] );
        __m128i xmm0 = _mm_unpacklo_epi16( xmm00, xmm10 );
        __m128i xmmS0 = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[0][1] );
        coeffb = _mm_set1_epi16( filterCoeff[1][1] );
        __m128i xmm1 = _mm_unpackhi_epi16( xmm00, xmm10 );
        __m128i xmmS1 = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // coeff 2 and 3
        __m128i xmm20 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 0 ) );
        xmm20 = _mm_sub_epi16( xmm20, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[2][0], filterClipp[2][0], filterClipp[2][0], filterClipp[2][0],
                                filterClipp[2][1], filterClipp[2][1], filterClipp[2][1], filterClipp[2][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm20 = _mm_min_epi16( xmm20, clipp );
        xmm20 = _mm_max_epi16( xmm20, clipm );

        __m128i xmm21 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 0 ) );
        xmm21 = _mm_sub_epi16( xmm21, xmmCur );
        xmm21 = _mm_min_epi16( xmm21, clipp );
        xmm21 = _mm_max_epi16( xmm21, clipm );

        xmm20 = _mm_add_epi16( xmm20, xmm21 );

        __m128i xmm30 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 1 ) );
        xmm30 = _mm_sub_epi16( xmm30, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[3][0], filterClipp[3][0], filterClipp[3][0], filterClipp[3][0],
                                filterClipp[3][1], filterClipp[3][1], filterClipp[3][1], filterClipp[3][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm30 = _mm_min_epi16( xmm30, clipp );
        xmm30 = _mm_max_epi16( xmm30, clipm );

        __m128i xmm31 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 1 ) );
        xmm31 = _mm_sub_epi16( xmm31, xmmCur );
        xmm31 = _mm_min_epi16( xmm31, clipp );
        xmm31 = _mm_max_epi16( xmm31, clipm );

        xmm30 = _mm_add_epi16( xmm30, xmm31 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[2][0] );
        coeffb = _mm_set1_epi16( filterCoeff[3][0] );
        xmm0 = _mm_unpacklo_epi16( xmm20, xmm30 );
        __m128i xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[2][1] );
        coeffb = _mm_set1_epi16( filterCoeff[3][1] );
        xmm1 = _mm_unpackhi_epi16( xmm20, xmm30 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // coeff 4 and 5
        __m128i xmm40 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 2 ) );
        xmm40 = _mm_sub_epi16( xmm40, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[4][0], filterClipp[4][0], filterClipp[4][0], filterClipp[4][0],
                                filterClipp[4][1], filterClipp[4][1], filterClipp[4][1], filterClipp[4][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm40 = _mm_min_epi16( xmm40, clipp );
        xmm40 = _mm_max_epi16( xmm40, clipm );

        __m128i xmm41 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 2 ) );
        xmm41 = _mm_sub_epi16( xmm41, xmmCur );
        xmm41 = _mm_min_epi16( xmm41, clipp );
        xmm41 = _mm_max_epi16( xmm41, clipm );

        xmm40 = _mm_add_epi16( xmm40, xmm41 );

        __m128i xmm50 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 1 ) );
        xmm50 = _mm_sub_epi16( xmm50, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[5][0], filterClipp[5][0], filterClipp[5][0], filterClipp[5][0],
                                filterClipp[5][1], filterClipp[5][1], filterClipp[5][1], filterClipp[5][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm50 = _mm_min_epi16( xmm50, clipp );
        xmm50 = _mm_max_epi16( xmm50, clipm );

        __m128i xmm51 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 1 ) );
        xmm51 = _mm_sub_epi16( xmm51, xmmCur );
        xmm51 = _mm_min_epi16( xmm51, clipp );
        xmm51 = _mm_max_epi16( xmm51, clipm );

        xmm50 = _mm_add_epi16( xmm50, xmm51 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[4][0] );
        coeffb = _mm_set1_epi16( filterCoeff[5][0] );
        xmm0 = _mm_unpacklo_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[4][1] );
        coeffb = _mm_set1_epi16( filterCoeff[5][1] );
        xmm1 = _mm_unpackhi_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // finish
        xmmS0 = _mm_add_epi32( xmmS0, mmOffset );
        xmmS0 = _mm_srai_epi32( xmmS0, numBitsMinus1 );
        xmmS1 = _mm_add_epi32( xmmS1, mmOffset );
        xmmS1 = _mm_srai_epi32( xmmS1, numBitsMinus1 );

        xmmS0 = _mm_packs_epi32( xmmS0, xmmS1 );
        // coeff 6
        xmmS0 = _mm_add_epi16(xmmS0, xmmCur);
        xmmS0 = _mm_min_epi16( mmMax, _mm_max_epi16( xmmS0, mmMin ) );

        if( j + 8 <= endWidth - startWidth )
        {
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 6 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xC0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 4 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xF0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xFC );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }

        pRec1 += dstStride;
      }
      pRec1 -= dstStride2;
      // restore 2x2 PCM chroma blocks
      if( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            if( *flags++ )
            {
              pRec1[(blkY+0)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+0)*dstStride + (blkX+1)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+1)] = *pcmRec++;
            }
          }
        }
      }

    }

    pRec0 += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
  }
}

template<X86_VEXT vext>
static void simdFilter7x7Blk(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc,
                             const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
                             const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
                             int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  const bool bChroma = isChroma( compId );

  const SPS*     sps = cs.slice->getSPS();
  bool isDualTree = CS::isDualITree(cs);
  bool isPCMFilterDisabled = sps->getPCMFilterDisableFlag();
  ChromaFormat nChromaFormat = sps->getChromaFormatIdc();
  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const int numBitsMinus1 = AdaptiveLoopFilter::m_NUM_BITS - 1;
  const int offset = ( 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 2 ) );

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

  const short *coef[2] = { filterSet, filterSet };
  const short *clip[2] = { fClipSet, fClipSet };

  int transposeIdx[2] = {0, 0};
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  bool pcmFlags2x2[8] = {0,0,0,0,0,0,0,0};
  Pel  pcmRec2x2[32];

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  const __m128i mmOffset = _mm_set1_epi32( offset );
  const __m128i mmMin = _mm_set1_epi16( clpRng.min );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max );

  const unsigned char *filterCoeffIdx[2];
  Pel filterCoeff[MAX_NUM_ALF_LUMA_COEFF][2];
  Pel filterClipp[MAX_NUM_ALF_LUMA_COEFF][2];

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;

  Pel* pRec0 = dst + blkDst.x;
  Pel* pRec1;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    if( !bChroma )
    {
      pClass = classifier[blkDst.y + i] + blkDst.x;
    }

    for( int j = 0; j < endWidth - startWidth; j += 8 )
    {
      for (int k = 0; k < 2; ++k)
      {
        if( !bChroma )
        {
          const AlfClassifier& cl = pClass[j+4*k];
          transposeIdx[k] = cl.transposeIdx;
          coef[k] = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          clip[k] = fClipSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          if ( isPCMFilterDisabled && cl.classIdx == AdaptiveLoopFilter::m_ALF_UNUSED_CLASSIDX && transposeIdx[k] == AdaptiveLoopFilter::m_ALF_UNUSED_TRANSPOSIDX )
          {
            // Note that last one (i.e. filterCoeff[12][k]) is not unused with JVET_N0242_NON_LINEAR_ALF; could be simplified
            static const unsigned char _filterCoeffIdx[13] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            static short _identityFilterCoeff[] = { 0 };
            static short _identityFilterClipp[] = { 0 };
            filterCoeffIdx[k] = _filterCoeffIdx;
            coef[k] = _identityFilterCoeff;
            clip[k] = _identityFilterClipp;
          }
          else if( transposeIdx[k] == 1 )
          {
            static const unsigned char _filterCoeffIdx[13] = { 9, 4, 10, 8, 1, 5, 11, 7, 3, 0, 2, 6, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 2 )
          {
            static const unsigned char _filterCoeffIdx[13] = { 0, 3, 2, 1, 8, 7, 6, 5, 4, 9, 10, 11, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 3 )
          {
            static const unsigned char _filterCoeffIdx[13] = { 9, 8, 10, 4, 3, 7, 11, 5, 1, 0, 2, 6, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else
          {
            static const unsigned char _filterCoeffIdx[13] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
        }
        else
        {
          static const unsigned char _filterCoeffIdx[13] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
          filterCoeffIdx[k] = _filterCoeffIdx;
        }

        for ( int i=0; i < 13; ++i )
        {
          filterCoeff[i][k] = coef[k][filterCoeffIdx[k][i]];
          filterClipp[i][k] = clip[k][filterCoeffIdx[k][i]];
        }
      }

      pRec1 = pRec0 + j;

      if ( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;

        // check which chroma 2x2 blocks use PCM
        // chroma PCM may not be aligned with 4x4 ALF processing grid
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            Position pos(j + blkDst.x + blkX, i + blkDst.y + blkY);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB && !JVET_O0050_LOCAL_DUAL_TREE
            const CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#else
            CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
            cu = cu->isSepTree() ? cs.getCU( pos, CH_C ) : cu;
#endif
            if( cu != NULL)
            {
              *flags++ = cu->ipcm ? 1 : 0;
            }
            else
            {
              *flags++ = 0;
            }

            // save original samples from 2x2 PCM blocks
            if( cu != NULL && cu->ipcm )
            {
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+1)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+1)];
            }
          }
        }
      }

      __m128i xmmNull = _mm_setzero_si128();

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;
        pImg5 = pImgYPad5 + j + ii * srcStride;
        pImg6 = pImgYPad6 + j + ii * srcStride;

        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - (bChroma ? 2 : 4)))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;
          pImg5 = (yVb >= vbPos - 3) ? pImg3 : pImg5;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
          pImg6 = (yVb >= vbPos - 3) ? pImg4 : pImg6;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + (bChroma ? 1 : 3)))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;
          pImg6 = (yVb <= vbPos + 2) ? pImg4 : pImg6;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
          pImg5 = (yVb <= vbPos + 2) ? pImg3 : pImg5;
        }
        __m128i clipp, clipm;
        __m128i coeffa, coeffb;
        __m128i xmmCur = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 0 ) );

        // coeff 0 and 1
        __m128i xmm00 = _mm_lddqu_si128( ( __m128i* ) ( pImg5 + 0 ) );
        xmm00 = _mm_sub_epi16( xmm00, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[0][0], filterClipp[0][0], filterClipp[0][0], filterClipp[0][0],
                                filterClipp[0][1], filterClipp[0][1], filterClipp[0][1], filterClipp[0][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm00 = _mm_min_epi16( xmm00, clipp );
        xmm00 = _mm_max_epi16( xmm00, clipm );

        __m128i xmm01 = _mm_lddqu_si128( ( __m128i* ) ( pImg6 + 0 ) );
        xmm01 = _mm_sub_epi16( xmm01, xmmCur );
        xmm01 = _mm_min_epi16( xmm01, clipp );
        xmm01 = _mm_max_epi16( xmm01, clipm );

        xmm00 = _mm_add_epi16( xmm00, xmm01 );

        __m128i xmm10 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 + 1 ) );
        xmm10 = _mm_sub_epi16( xmm10, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[1][0], filterClipp[1][0], filterClipp[1][0], filterClipp[1][0],
                                filterClipp[1][1], filterClipp[1][1], filterClipp[1][1], filterClipp[1][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm10 = _mm_min_epi16( xmm10, clipp );
        xmm10 = _mm_max_epi16( xmm10, clipm );

        __m128i xmm11 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 - 1 ) );
        xmm11 = _mm_sub_epi16( xmm11, xmmCur );
        xmm11 = _mm_min_epi16( xmm11, clipp );
        xmm11 = _mm_max_epi16( xmm11, clipm );

        xmm10 = _mm_add_epi16( xmm10, xmm11 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[0][0] );
        coeffb = _mm_set1_epi16( filterCoeff[1][0] );
        __m128i xmm0 = _mm_unpacklo_epi16( xmm00, xmm10 );
        __m128i xmmS0 = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[0][1] );
        coeffb = _mm_set1_epi16( filterCoeff[1][1] );
        __m128i xmm1 = _mm_unpackhi_epi16( xmm00, xmm10 );
        __m128i xmmS1 = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // coeff 2 and 3
        __m128i xmm20 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 + 0 ) );
        xmm20 = _mm_sub_epi16( xmm20, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[2][0], filterClipp[2][0], filterClipp[2][0], filterClipp[2][0],
                                filterClipp[2][1], filterClipp[2][1], filterClipp[2][1], filterClipp[2][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm20 = _mm_min_epi16( xmm20, clipp );
        xmm20 = _mm_max_epi16( xmm20, clipm );

        __m128i xmm21 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 + 0 ) );
        xmm21 = _mm_sub_epi16( xmm21, xmmCur );
        xmm21 = _mm_min_epi16( xmm21, clipp );
        xmm21 = _mm_max_epi16( xmm21, clipm );

        xmm20 = _mm_add_epi16( xmm20, xmm21 );

        __m128i xmm30 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 - 1 ) );
        xmm30 = _mm_sub_epi16( xmm30, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[3][0], filterClipp[3][0], filterClipp[3][0], filterClipp[3][0],
                                filterClipp[3][1], filterClipp[3][1], filterClipp[3][1], filterClipp[3][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm30 = _mm_min_epi16( xmm30, clipp );
        xmm30 = _mm_max_epi16( xmm30, clipm );

        __m128i xmm31 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 + 1 ) );
        xmm31 = _mm_sub_epi16( xmm31, xmmCur );
        xmm31 = _mm_min_epi16( xmm31, clipp );
        xmm31 = _mm_max_epi16( xmm31, clipm );

        xmm30 = _mm_add_epi16( xmm30, xmm31 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[2][0] );
        coeffb = _mm_set1_epi16( filterCoeff[3][0] );
        xmm0 = _mm_unpacklo_epi16( xmm20, xmm30 );
        __m128i xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[2][1] );
        coeffb = _mm_set1_epi16( filterCoeff[3][1] );
        xmm1 = _mm_unpackhi_epi16( xmm20, xmm30 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // coeff 4 and 5
        __m128i xmm40 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 2 ) );
        xmm40 = _mm_sub_epi16( xmm40, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[4][0], filterClipp[4][0], filterClipp[4][0], filterClipp[4][0],
                                filterClipp[4][1], filterClipp[4][1], filterClipp[4][1], filterClipp[4][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm40 = _mm_min_epi16( xmm40, clipp );
        xmm40 = _mm_max_epi16( xmm40, clipm );

        __m128i xmm41 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 2 ) );
        xmm41 = _mm_sub_epi16( xmm41, xmmCur );
        xmm41 = _mm_min_epi16( xmm41, clipp );
        xmm41 = _mm_max_epi16( xmm41, clipm );

        xmm40 = _mm_add_epi16( xmm40, xmm41 );

        __m128i xmm50 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 1 ) );
        xmm50 = _mm_sub_epi16( xmm50, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[5][0], filterClipp[5][0], filterClipp[5][0], filterClipp[5][0],
                                filterClipp[5][1], filterClipp[5][1], filterClipp[5][1], filterClipp[5][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm50 = _mm_min_epi16( xmm50, clipp );
        xmm50 = _mm_max_epi16( xmm50, clipm );

        __m128i xmm51 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 1 ) );
        xmm51 = _mm_sub_epi16( xmm51, xmmCur );
        xmm51 = _mm_min_epi16( xmm51, clipp );
        xmm51 = _mm_max_epi16( xmm51, clipm );

        xmm50 = _mm_add_epi16( xmm50, xmm51 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[4][0] );
        coeffb = _mm_set1_epi16( filterCoeff[5][0] );
        xmm0 = _mm_unpacklo_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[4][1] );
        coeffb = _mm_set1_epi16( filterCoeff[5][1] );
        xmm1 = _mm_unpackhi_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);


        // coeff 6 and 7
        __m128i xmm60 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 0 ) );
        xmm60 = _mm_sub_epi16( xmm60, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[6][0], filterClipp[6][0], filterClipp[6][0], filterClipp[6][0],
                                filterClipp[6][1], filterClipp[6][1], filterClipp[6][1], filterClipp[6][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm60 = _mm_min_epi16( xmm60, clipp );
        xmm60 = _mm_max_epi16( xmm60, clipm );

        __m128i xmm61 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 0 ) );
        xmm61 = _mm_sub_epi16( xmm61, xmmCur );
        xmm61 = _mm_min_epi16( xmm61, clipp );
        xmm61 = _mm_max_epi16( xmm61, clipm );

        xmm60 = _mm_add_epi16( xmm60, xmm61 );

        __m128i xmm70 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 1 ) );
        xmm70 = _mm_sub_epi16( xmm70, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[7][0], filterClipp[7][0], filterClipp[7][0], filterClipp[7][0],
                                filterClipp[7][1], filterClipp[7][1], filterClipp[7][1], filterClipp[7][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm70 = _mm_min_epi16( xmm70, clipp );
        xmm70 = _mm_max_epi16( xmm70, clipm );

        __m128i xmm71 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 1 ) );
        xmm71 = _mm_sub_epi16( xmm71, xmmCur );
        xmm71 = _mm_min_epi16( xmm71, clipp );
        xmm71 = _mm_max_epi16( xmm71, clipm );

        xmm70 = _mm_add_epi16( xmm70, xmm71 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[6][0] );
        coeffb = _mm_set1_epi16( filterCoeff[7][0] );
        xmm0 = _mm_unpacklo_epi16( xmm60, xmm70 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[6][1] );
        coeffb = _mm_set1_epi16( filterCoeff[7][1] );
        xmm1 = _mm_unpackhi_epi16( xmm60, xmm70 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);


        // coeff 8 and 9
        __m128i xmm80 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 2 ) );
        xmm80 = _mm_sub_epi16( xmm80, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[8][0], filterClipp[8][0], filterClipp[8][0], filterClipp[8][0],
                                filterClipp[8][1], filterClipp[8][1], filterClipp[8][1], filterClipp[8][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm80 = _mm_min_epi16( xmm80, clipp );
        xmm80 = _mm_max_epi16( xmm80, clipm );

        __m128i xmm81 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 2 ) );
        xmm81 = _mm_sub_epi16( xmm81, xmmCur );
        xmm81 = _mm_min_epi16( xmm81, clipp );
        xmm81 = _mm_max_epi16( xmm81, clipm );

        xmm80 = _mm_add_epi16( xmm80, xmm81 );

        __m128i xmm90 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 3 ) );
        xmm90 = _mm_sub_epi16( xmm90, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[9][0], filterClipp[9][0], filterClipp[9][0], filterClipp[9][0],
                                filterClipp[9][1], filterClipp[9][1], filterClipp[9][1], filterClipp[9][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm90 = _mm_min_epi16( xmm90, clipp );
        xmm90 = _mm_max_epi16( xmm90, clipm );

        __m128i xmm91 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 3 ) );
        xmm91 = _mm_sub_epi16( xmm91, xmmCur );
        xmm91 = _mm_min_epi16( xmm91, clipp );
        xmm91 = _mm_max_epi16( xmm91, clipm );

        xmm90 = _mm_add_epi16( xmm90, xmm91 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[8][0] );
        coeffb = _mm_set1_epi16( filterCoeff[9][0] );
        xmm0 = _mm_unpacklo_epi16( xmm80, xmm90 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[8][1] );
        coeffb = _mm_set1_epi16( filterCoeff[9][1] );
        xmm1 = _mm_unpackhi_epi16( xmm80, xmm90 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);


        // coeff 10 and 11
        __m128i xmm100 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 2 ) );
        xmm100 = _mm_sub_epi16( xmm100, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[10][0], filterClipp[10][0], filterClipp[10][0], filterClipp[10][0],
                                filterClipp[10][1], filterClipp[10][1], filterClipp[10][1], filterClipp[10][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm100 = _mm_min_epi16( xmm100, clipp );
        xmm100 = _mm_max_epi16( xmm100, clipm );

        __m128i xmm101 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 2 ) );
        xmm101 = _mm_sub_epi16( xmm101, xmmCur );
        xmm101 = _mm_min_epi16( xmm101, clipp );
        xmm101 = _mm_max_epi16( xmm101, clipm );

        xmm100 = _mm_add_epi16( xmm100, xmm101 );

        __m128i xmm110 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 1 ) );
        xmm110 = _mm_sub_epi16( xmm110, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[11][0], filterClipp[11][0], filterClipp[11][0], filterClipp[11][0],
                                filterClipp[11][1], filterClipp[11][1], filterClipp[11][1], filterClipp[11][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm110 = _mm_min_epi16( xmm110, clipp );
        xmm110 = _mm_max_epi16( xmm110, clipm );

        __m128i xmm111 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 1 ) );
        xmm111 = _mm_sub_epi16( xmm111, xmmCur );
        xmm111 = _mm_min_epi16( xmm111, clipp );
        xmm111 = _mm_max_epi16( xmm111, clipm );

        xmm110 = _mm_add_epi16( xmm110, xmm111 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[10][0] );
        coeffb = _mm_set1_epi16( filterCoeff[11][0] );
        xmm0 = _mm_unpacklo_epi16( xmm100, xmm110 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[10][1] );
        coeffb = _mm_set1_epi16( filterCoeff[11][1] );
        xmm1 = _mm_unpackhi_epi16( xmm100, xmm110 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // finish
        xmmS0 = _mm_add_epi32( xmmS0, mmOffset );
        xmmS0 = _mm_srai_epi32( xmmS0, numBitsMinus1 );
        xmmS1 = _mm_add_epi32( xmmS1, mmOffset );
        xmmS1 = _mm_srai_epi32( xmmS1, numBitsMinus1 );

        xmmS0 = _mm_packs_epi32( xmmS0, xmmS1 );
        // coeff 12
        xmmS0 = _mm_add_epi16(xmmS0, xmmCur);
        xmmS0 = _mm_min_epi16( mmMax, _mm_max_epi16( xmmS0, mmMin ) );

        if( j + 8 <= endWidth - startWidth )
        {
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 6 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xC0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 4 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xF0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xFC );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }


        pRec1 += dstStride;
      }
      pRec1 -= dstStride2;
      // restore 2x2 PCM chroma blocks
      if( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            if( *flags++ )
            {
              pRec1[(blkY+0)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+0)*dstStride + (blkX+1)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+1)] = *pcmRec++;
            }
          }
        }
      }

    }

    pRec0 += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
  }
}

template <X86_VEXT vext>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86()
{
  m_deriveClassificationBlk = simdDeriveClassificationBlk<vext>;
  m_filter5x5Blk = simdFilter5x5Blk<vext>;
  m_filter7x7Blk = simdFilter7x7Blk<vext>;
}

template void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86<SIMDX86>();
#endif //#ifdef TARGET_SIMD_X86
//! \}
