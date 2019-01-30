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

 /** \file     Hash.h
     \brief    Hash class (header)
 */


#ifndef __HASH__
#define __HASH__

 // Include files

#include "CommonLib/Buffer.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include <vector>


struct BlockHash
{
  short x;
  short y;
  unsigned int hashValue2;
};

typedef std::vector<BlockHash>::iterator MapIterator;

// ====================================================================================================================
// Class definitions
// ====================================================================================================================


struct TCRCCalculatorLight
{
public:
  TCRCCalculatorLight(unsigned int bits, unsigned int truncPoly);
  ~TCRCCalculatorLight();

public:
  void processData(unsigned char* curData, unsigned int dataLength);
  void reset() { m_remainder = 0; }
  unsigned int getCRC() { return m_remainder & m_finalResultMask; }

private:
  void xInitTable();

private:
  unsigned int m_remainder;
  unsigned int m_truncPoly;
  unsigned int m_bits;
  unsigned int m_table[256];
  unsigned int m_finalResultMask;
};


struct TComHash
{
public:
  TComHash();
  ~TComHash();
  void create();
  void clearAll();
  void addToTable(unsigned int hashValue, const BlockHash& blockHash);
  int count(unsigned int hashValue);
  int count(unsigned int hashValue) const;
  MapIterator getFirstIterator(unsigned int hashValue);
  const MapIterator getFirstIterator(unsigned int hashValue) const;
  bool hasExactMatch(unsigned int hashValue1, unsigned int hashValue2);

  void generateBlock2x2HashValue(const PelUnitBuf &curPicBuf, int picWidth, int picHeight, const BitDepths bitDepths, unsigned int* picBlockHash[2], bool* picBlockSameInfo[3]);
  void generateBlockHashValue(int picWidth, int picHeight, int width, int height, unsigned int* srcPicBlockHash[2], unsigned int* dstPicBlockHash[2], bool* srcPicBlockSameInfo[3], bool* dstPicBlockSameInfo[3]);
  void generateRectangleHashValue(int picWidth, int picHeight, int width, int height, unsigned int* srcPicBlockHash[2], unsigned int* dstPicBlockHash[2], bool* srcPicBlockSameInfo[3], bool* dstPicBlockSameInfo[3]);
  void addToHashMapByRowWithPrecalData(unsigned int* srcHash[2], bool* srcIsSame, int picWidth, int picHeight, int width, int height);
  bool isInitial() { return tableHasContent; }
  void setInitial() { tableHasContent = true; }



public:
  static unsigned int   getCRCValue1(unsigned char* p, int length);
  static unsigned int   getCRCValue2(unsigned char* p, int length);
  static void getPixelsIn1DCharArrayByBlock2x2(const PelUnitBuf &curPicBuf, unsigned char* pixelsIn1D, int xStart, int yStart, const BitDepths& bitDepths, bool includeAllComponent = true);
  static bool isBlock2x2RowSameValue(unsigned char* p, bool includeAllComponent = true);
  static bool isBlock2x2ColSameValue(unsigned char* p, bool includeAllComponent = true);
  static bool getBlockHashValue(const PelUnitBuf &curPicBuf, int width, int height, int xStart, int yStart, const BitDepths bitDepths, unsigned int& hashValue1, unsigned int& hashValue2);
  static void initBlockSizeToIndex();

private:
  std::vector<BlockHash>** m_lookupTable;
  bool tableHasContent;

private:
  static const int m_CRCBits = 16;
  static const int m_blockSizeBits = 3;
  static int m_blockSizeToIndex[65][65];

  static TCRCCalculatorLight m_crcCalculator1;
  static TCRCCalculatorLight m_crcCalculator2;
};

#endif // __HASH__
