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

/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"
#include "ChromaFormat.h"
#if ENABLE_WPP_PARALLELISM
#if ENABLE_WPP_STATIC_LINK
#include <atomic>
#else
#include <condition_variable>
#endif
#endif


#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
#if ENABLE_WPP_STATIC_LINK
class SyncObj
{
public:
  SyncObj() : m_Val(-1) {}
  ~SyncObj()            {}

  void reset()
  {
    m_Val = -1;
  }

  bool isReady( int64_t val ) const
  {
//    std::cout << "is ready m_Val " << m_Val << " val " << val << std::endl;
    return m_Val >= val;
  }

  void wait( int64_t idx, int ctuPosY  )
  {
    while( ! isReady( idx ) )
    {
    }
  }

  void set( int64_t val, int ctuPosY)
  {
    m_Val = val;
  }

private:
  std::atomic<int>         m_Val;
};
#else
class SyncObj
{
public:
  SyncObj() : m_Val(-1) {}
  ~SyncObj()            {}

  void reset()
  {
    std::unique_lock< std::mutex > lock( m_mutex );

    m_Val = -1;
  }

  bool isReady( int64_t val ) const
  {
    return m_Val >= val;
  }

  void wait( int64_t idx, int ctuPosY  )
  {
    std::unique_lock< std::mutex > lock( m_mutex );

    while( ! isReady( idx ) )
    {
      m_cv.wait( lock );
    }
  }

  void set( int64_t val, int ctuPosY)
  {
    std::unique_lock< std::mutex > lock( m_mutex );
    m_Val = val;
    m_cv.notify_all();
  }

private:
  int64_t                 m_Val;
  std::condition_variable m_cv;
  std::mutex              m_mutex;
};
#endif
#endif

int g_wppThreadId( 0 );
#pragma omp threadprivate(g_wppThreadId)

#if ENABLE_SPLIT_PARALLELISM
int g_splitThreadId( 0 );
#pragma omp threadprivate(g_splitThreadId)

int g_splitJobId( 0 );
#pragma omp threadprivate(g_splitJobId)
#endif

Scheduler::Scheduler() :
#if ENABLE_WPP_PARALLELISM
  m_numWppThreads( 1 ),
  m_numWppDataInstances( 1 )
#endif
#if ENABLE_SPLIT_PARALLELISM && ENABLE_WPP_PARALLELISM
  ,
#endif
#if ENABLE_SPLIT_PARALLELISM
  m_numSplitThreads( 1 )
#endif
{
}

Scheduler::~Scheduler()
{
#if ENABLE_WPP_PARALLELISM
  for( auto & so : m_SyncObjs )
  {
    delete so;
  }
  m_SyncObjs.clear();
#endif
}

#if ENABLE_SPLIT_PARALLELISM
unsigned Scheduler::getSplitDataId( int jobId ) const
{
  if( m_numSplitThreads > 1 && m_hasParallelBuffer )
  {
    int splitJobId = jobId == CURR_THREAD_ID ? g_splitJobId : jobId;

    return ( g_wppThreadId * NUM_RESERVERD_SPLIT_JOBS ) + splitJobId;
  }
  else
  {
    return 0;
  }
}

unsigned Scheduler::getSplitPicId( int tId /*= CURR_THREAD_ID */ ) const
{
  if( m_numSplitThreads > 1 && m_hasParallelBuffer )
  {
    int threadId = tId == CURR_THREAD_ID ? g_splitThreadId : tId;

    return ( g_wppThreadId * m_numSplitThreads ) + threadId;
  }
  else
  {
    return 0;
  }
}

unsigned Scheduler::getSplitJobId() const
{
  if( m_numSplitThreads > 1 )
  {
    return g_splitJobId;
  }
  else
  {
    return 0;
  }
}

void Scheduler::setSplitJobId( const int jobId )
{
  CHECK( g_splitJobId != 0 && jobId != 0, "Need to reset the jobId after usage!" );
  g_splitJobId = jobId;
}

void Scheduler::startParallel()
{
  m_hasParallelBuffer = true;
}

void Scheduler::finishParallel()
{
  m_hasParallelBuffer = false;
}

void Scheduler::setSplitThreadId( const int tId )
{
  g_splitThreadId = tId == CURR_THREAD_ID ? omp_get_thread_num() : tId;
}

#endif


#if ENABLE_WPP_PARALLELISM
unsigned Scheduler::getWppDataId( int lID ) const
{
  const int tId = lID == CURR_THREAD_ID ? g_wppThreadId : lID;

#if ENABLE_SPLIT_PARALLELISM
  if( m_numSplitThreads > 1 )
  {
    return tId * NUM_RESERVERD_SPLIT_JOBS;
  }
  else
  {
    return tId;
  }
#else
  return tId;
#endif
}

unsigned Scheduler::getWppThreadId() const
{
  return g_wppThreadId;
}

void Scheduler::setWppThreadId( const int tId )
{
  g_wppThreadId = tId == CURR_THREAD_ID ? omp_get_thread_num() : tId;

  CHECK( g_wppThreadId >= PARL_WPP_MAX_NUM_THREADS, "The WPP thread ID " << g_wppThreadId << " is invalid!" );
}
#endif

unsigned Scheduler::getDataId() const
{
#if ENABLE_SPLIT_PARALLELISM
  if( m_numSplitThreads > 1 )
  {
    return getSplitDataId();
  }
#endif
#if ENABLE_WPP_PARALLELISM
  if( m_numWppThreads > 1 )
  {
    return getWppDataId();
  }
#endif
  return 0;
}

bool Scheduler::init( const int ctuYsize, const int ctuXsize, const int numWppThreadsRunning, const int numWppExtraLines, const int numSplitThreads )
{
#if ENABLE_SPLIT_PARALLELISM
  m_numSplitThreads = numSplitThreads;
#endif
#if ENABLE_WPP_PARALLELISM
  m_firstNonFinishedLine    = 0;
  m_numWppThreadsRunning    = 1;
  m_numWppDataInstances     = numWppThreadsRunning+numWppExtraLines;
  m_numWppThreads           = numWppThreadsRunning;
  m_ctuYsize                = ctuYsize;
  m_ctuXsize                = ctuXsize;

  if( m_SyncObjs.size() == 0 )
  {
    m_SyncObjs.reserve( ctuYsize );
    for( int i = (int)m_SyncObjs.size(); i < ctuYsize; i++ )
    {
      m_SyncObjs.push_back( new SyncObj );
    }
  }
  else
  {
    CHECK( m_SyncObjs.size() != ctuYsize, "");
  }

  for( int i = 0; i < ctuYsize; i++ )
  {
    m_SyncObjs[i]->reset();
  }

  if( m_numWppThreads != m_numWppDataInstances )
  {
    m_LineDone.clear();
    m_LineDone.resize(ctuYsize, -1);

    m_LineProc.clear();
    m_LineProc.resize(ctuYsize, false);

    m_SyncObjs[0]->set(0,0);
    m_LineProc[0]=true;
  }
#endif

  return true;
}


int Scheduler::getNumPicInstances() const
{
#if !ENABLE_SPLIT_PARALLELISM
  return 1;
#elif !ENABLE_WPP_PARALLELISM
  return ( m_numSplitThreads > 1 ? m_numSplitThreads : 1 );
#else
  return m_numSplitThreads > 1 ? m_numWppDataInstances * m_numSplitThreads : 1;
#endif
}

#if ENABLE_WPP_PARALLELISM
void Scheduler::wait( const int ctuPosX, const int ctuPosY )
{
  if( m_numWppThreads == m_numWppDataInstances )
  {
    if( ctuPosY > 0 && ctuPosX+1 < m_ctuXsize)
    {
      m_SyncObjs[ctuPosY-1]->wait( ctuPosX+1, ctuPosY-1 );
    }
    return;
  }

  m_SyncObjs[ctuPosY]->wait( ctuPosX, ctuPosY );
}

void Scheduler::setReady(const int ctuPosX, const int ctuPosY)
{
  if( m_numWppThreads == m_numWppDataInstances )
  {
    m_SyncObjs[ctuPosY]->set( ctuPosX, ctuPosY);
    return;
  }

  std::unique_lock< std::mutex > lock( m_mutex );

  if( ctuPosX+1 == m_ctuXsize )
  {
    m_LineProc[ctuPosY] = true; //prevent line from be further evaluated
    m_LineDone[ctuPosY] = std::numeric_limits<int>::max();
    m_firstNonFinishedLine = ctuPosY+1;
  }
  else
  {
    m_LineDone[ctuPosY] = ctuPosX;
    m_LineProc[ctuPosY] = false;    // mark currently not processed
  }

  int lastLine = m_firstNonFinishedLine + m_numWppDataInstances;
  lastLine = std::min( m_ctuYsize, lastLine )-1-m_firstNonFinishedLine;

  m_numWppThreadsRunning--;

  Position pos;
  //if the current encoder is the last
  const bool c1 = (ctuPosY == m_firstNonFinishedLine + m_numWppThreads - 1);
  const bool c2 = (ctuPosY+1 <= m_firstNonFinishedLine+lastLine);
  const bool c3 = (ctuPosX >= m_ctuXsize/4);
  if( c1 && c2 && c3 && getNextCtu( pos, ctuPosY+1, 4 ) )
  {
    //  try to continue in the next row
    // go on in the current line
    m_SyncObjs[pos.y]->set(pos.x, pos.y);
    m_numWppThreadsRunning++;
  }
  else if( getNextCtu( pos, ctuPosY, 1 ) )
  {
    //  try to continue in the same row
    // go on in the current line
    m_SyncObjs[pos.y]->set(pos.x, pos.y);
    m_numWppThreadsRunning++;
  }
  for( int i = m_numWppThreadsRunning; i < m_numWppThreads; i++ )
  {
   // just go and get a job
    for( int y = 0; y <= lastLine; y++ )
    {
      if( getNextCtu( pos, m_firstNonFinishedLine+y, 1 ))
      {
        m_SyncObjs[pos.y]->set(pos.x, pos.y);
        m_numWppThreadsRunning++;
        break;
      }
    }
  }
}


bool Scheduler::getNextCtu( Position& pos, int ctuLine, int offset)
{
  int x = m_LineDone[ctuLine] + 1;
  if( ! m_LineProc[ctuLine] )
  {
    int maxXOffset = x+offset >= m_ctuXsize ? m_ctuXsize-1 : x+offset;
    if( (ctuLine == 0 || m_LineDone[ctuLine-1]>=maxXOffset) && (x==0 || m_LineDone[ctuLine]>=+x-1))
    {
      m_LineProc[ctuLine] = true;
      pos.x = x; pos.y = ctuLine;
      return true;
    }
  }
  return false;
}

#endif
#endif


// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------


Brick::Brick()
: m_widthInCtus     (0)
, m_heightInCtus    (0)
, m_colBd           (0)
, m_rowBd           (0)
, m_firstCtuRsAddr  (0)
{
}

Brick::~Brick()
{
}


BrickMap::BrickMap()
  : pcv(nullptr)
  , numTiles(0)
  , numTileColumns(0)
  , numTileRows(0)
  , brickIdxRsMap(nullptr)
  , brickIdxBsMap(nullptr)
  , ctuBsToRsAddrMap(nullptr)
  , ctuRsToBsAddrMap(nullptr)
{
}

void BrickMap::create( const SPS& sps, const PPS& pps )
{
  pcv = pps.pcv;

  numTileColumns = pps.getNumTileColumnsMinus1() + 1;
  numTileRows    = pps.getNumTileRowsMinus1() + 1;
  numTiles       = numTileColumns * numTileRows;

  const size_t numCtusInFrame = pcv->sizeInCtus;

  brickIdxRsMap    = new uint32_t[numCtusInFrame];
  brickIdxBsMap    = new uint32_t[numCtusInFrame + 1];
  ctuBsToRsAddrMap = new uint32_t[numCtusInFrame + 1];
  ctuRsToBsAddrMap = new uint32_t[numCtusInFrame + 1];

  brickIdxBsMap[numCtusInFrame] = ~0u;   // Initialize last element to some large value

  initBrickMap( sps, pps );

  numTiles = (uint32_t) bricks.size();
}

void BrickMap::destroy()
{
  bricks.clear();

  if ( brickIdxRsMap )
  {
    delete[] brickIdxRsMap;
    brickIdxRsMap = nullptr;
  }

  if ( brickIdxBsMap )
  {
    delete[] brickIdxBsMap;
    brickIdxBsMap = nullptr;
  }

  if ( ctuBsToRsAddrMap )
  {
    delete[] ctuBsToRsAddrMap;
    ctuBsToRsAddrMap = nullptr;
  }

  if ( ctuRsToBsAddrMap )
  {
    delete[] ctuRsToBsAddrMap;
    ctuRsToBsAddrMap = nullptr;
  }
}

void BrickMap::initBrickMap( const SPS& sps, const PPS& pps )
{
  const uint32_t frameWidthInCtus  = pcv->widthInCtus;
  const uint32_t frameHeightInCtus = pcv->heightInCtus;

#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
  std::vector<uint32_t> tileRowHeight;
  std::vector<uint32_t> tileColWidth;
  if (pps.getUniformTileSpacingFlag())
  {
    int tileWidthInCTUs = pps.getTileColsWidthMinus1() + 1;
    int tileHeightInCTUs = pps.getTileRowsHeightMinus1() + 1;
    int tileWidthInLumaSamples = tileWidthInCTUs * sps.getCTUSize();
    int tileHeightInLumaSamples = tileHeightInCTUs * sps.getCTUSize();

    numTileColumns = (pps.getPicWidthInLumaSamples() + tileWidthInLumaSamples - 1) / tileWidthInLumaSamples;
    numTileRows = (pps.getPicHeightInLumaSamples() + tileHeightInLumaSamples - 1) / tileHeightInLumaSamples;
    numTiles = numTileColumns * numTileRows;

    int remainingHeightInCtbsY = frameHeightInCtus;
    while (remainingHeightInCtbsY > tileHeightInCTUs)
    {
      tileRowHeight.push_back(tileHeightInCTUs);
      remainingHeightInCtbsY -= tileHeightInCTUs;
    }
    tileRowHeight.push_back(remainingHeightInCtbsY);

    int remainingWidthInCtbsY = frameWidthInCtus;
    while (remainingWidthInCtbsY > tileWidthInCTUs)
    {
      tileColWidth.push_back(tileWidthInCTUs);
      remainingWidthInCtbsY -= tileWidthInCTUs;
    }
    tileColWidth.push_back(remainingWidthInCtbsY);
#else
  std::vector<uint32_t> tileRowHeight (numTileRows);
  std::vector<uint32_t> tileColWidth (numTileColumns);

  if( pps.getUniformTileSpacingFlag() )
  {
    //set width and height for each (uniform) tile
    for(int row=0; row < numTileRows; row++)
    {
      tileRowHeight[row] = (row+1)*frameHeightInCtus/numTileRows   - (row*frameHeightInCtus)/numTileRows;
    }
    for(int col=0; col < numTileColumns; col++)
    {
      tileColWidth[col] = (col+1)*frameWidthInCtus/numTileColumns - (col*frameWidthInCtus)/numTileColumns;
    }
#endif
  }
  else
  {
#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
    tileColWidth.resize(numTileColumns);
    tileRowHeight.resize(numTileRows);
#endif
    tileColWidth[ numTileColumns - 1 ] = frameWidthInCtus;
    for( int i = 0; i < numTileColumns - 1; i++ )
    {
      tileColWidth[ i ] = pps.getTileColumnWidth(i);
      tileColWidth[ numTileColumns - 1 ]  =  tileColWidth[ numTileColumns - 1 ] - pps.getTileColumnWidth(i);
    }


    tileRowHeight[ numTileRows-1 ] = frameHeightInCtus;
    for( int j = 0; j < numTileRows-1; j++ )
    {
      tileRowHeight[ j ] = pps.getTileRowHeight( j );
      tileRowHeight[ numTileRows-1 ]  =  tileRowHeight[ numTileRows-1 ] - pps.getTileRowHeight( j );
    }
  }


  //initialize each tile of the current picture
  std::vector<uint32_t> tileRowBd (numTileRows);
  std::vector<uint32_t> tileColBd (numTileColumns);

  tileColBd[ 0 ] = 0;
  for( int i = 0; i  <  numTileColumns - 1; i++ )
  {
    tileColBd[ i + 1 ] = tileColBd[ i ] + tileColWidth[ i ];
  }

  tileRowBd[ 0 ] = 0;
  for( int j = 0; j  <  numTileRows - 1; j++ )
  {
    tileRowBd[ j + 1 ] = tileRowBd[ j ] + tileRowHeight[ j ];
  }

  int brickIdx = 0;
  for(int tileIdx=0; tileIdx< numTiles; tileIdx++)
  {
#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
    int tileX = tileIdx % numTileColumns;
    int tileY = tileIdx / numTileColumns;
#else
    int tileX = tileIdx % ( pps.getNumTileColumnsMinus1() + 1 );
    int tileY = tileIdx / ( pps.getNumTileColumnsMinus1() + 1 );
#endif
    if ( !pps.getBrickSplittingPresentFlag() || !pps.getBrickSplitFlag(tileIdx))
    {
      bricks.resize(bricks.size()+1);
      bricks[ brickIdx ].setColBd (tileColBd[ tileX ]);
      bricks[ brickIdx ].setRowBd (tileRowBd[ tileY ]);
      bricks[ brickIdx ].setWidthInCtus (tileColWidth[ tileX ]);
      bricks[ brickIdx ].setHeightInCtus(tileRowHeight[ tileY ]);
      bricks[ brickIdx ].setFirstCtuRsAddr(bricks[ brickIdx ].getColBd() + bricks[ brickIdx ].getRowBd() * frameWidthInCtus);
      brickIdx++;
    }
    else
    {
      std::vector<uint32_t> rowHeight2;
      std::vector<uint32_t> rowBd2;
#if JVET_O0173_O0176_O0338_NUMBRICK_M2
      int numBrickRowsMinus2 = 0;
      if (pps.getUniformBrickSpacingFlag(tileIdx))
      {
        int brickHeight            = pps.getBrickHeightMinus1(tileIdx) + 1;
        int remainingHeightInCtbsY = tileRowHeight[tileY];
        int brickInTile            = 0;
        while (remainingHeightInCtbsY > brickHeight)
        {
          rowHeight2.resize(brickInTile + 1);
          rowHeight2[brickInTile++] = brickHeight;
          remainingHeightInCtbsY -= brickHeight;
        }
        rowHeight2.resize(brickInTile + 1);
        rowHeight2[brickInTile] = remainingHeightInCtbsY;
        numBrickRowsMinus2      = brickInTile - 1;
      }
      else
      {
        numBrickRowsMinus2 = pps.getNumBrickRowsMinus2(tileIdx);
        rowHeight2.resize(numBrickRowsMinus2 + 2);
        rowHeight2[numBrickRowsMinus2 + 1] = tileRowHeight[tileY];
        for (int j = 0; j < numBrickRowsMinus2 + 1; j++)
        {
          rowHeight2[j] = pps.getBrickRowHeightMinus1(tileIdx, j) + 1;
          rowHeight2[numBrickRowsMinus2 + 1] -= rowHeight2[j];
        }
      }
      rowBd2.resize(numBrickRowsMinus2 + 2);
      rowBd2[0] = 0;
      for (int j = 0; j < numBrickRowsMinus2 + 1; j++)
      {
        rowBd2[j + 1] = rowBd2[j] + rowHeight2[j];
      }
      for (int j = 0; j < numBrickRowsMinus2 + 2; j++)
      {
        bricks.resize(bricks.size() + 1);
        bricks[brickIdx].setColBd(tileColBd[tileX]);
        bricks[brickIdx].setRowBd(tileRowBd[tileY] + rowBd2[j]);
        bricks[brickIdx].setWidthInCtus(tileColWidth[tileX]);
        bricks[brickIdx].setHeightInCtus(rowHeight2[j]);
        bricks[brickIdx].setFirstCtuRsAddr(bricks[brickIdx].getColBd() + bricks[brickIdx].getRowBd() * frameWidthInCtus);
        brickIdx++;
      }
#else
      int numBrickRowsMinus1 = 0;
      if (pps.getUniformBrickSpacingFlag(tileIdx))
      {
        int brickHeight= pps.getBrickHeightMinus1(tileIdx) + 1;
        int remainingHeightInCtbsY  = tileRowHeight[ tileY ];
        int brickInTile = 0;
        while( remainingHeightInCtbsY > brickHeight )
        {
          rowHeight2.resize(brickInTile+1);
          rowHeight2[ brickInTile++ ] = brickHeight;
          remainingHeightInCtbsY -= brickHeight;
        }
        rowHeight2.resize(brickInTile+1);
        rowHeight2[ brickInTile ] = remainingHeightInCtbsY;
        numBrickRowsMinus1 = brickInTile;
      }
      else
      {
        numBrickRowsMinus1 = pps.getNumBrickRowsMinus1(tileIdx);
        rowHeight2.resize(numBrickRowsMinus1 + 1);
        rowHeight2[ numBrickRowsMinus1 ] = tileRowHeight[ tileY ];
        for(int j = 0; j < numBrickRowsMinus1; j++ )
        {
          rowHeight2[ j ] = pps.getBrickRowHeightMinus1 ( tileIdx, j )+ 1;
          rowHeight2[ numBrickRowsMinus1 ]  -=  rowHeight2[ j ];
        }
      }
      rowBd2.resize(numBrickRowsMinus1 + 1);
      rowBd2[ 0 ] = 0;
      for( int j = 0; j  <  numBrickRowsMinus1; j++ )
      {
        rowBd2[ j + 1 ] = rowBd2[ j ] + rowHeight2[ j ];
      }
      for( int j = 0; j < numBrickRowsMinus1 + 1; j++ )
      {
        bricks.resize(bricks.size()+1);
        bricks[ brickIdx ].setColBd (tileColBd[ tileX ]);
        bricks[ brickIdx ].setRowBd (tileRowBd[ tileY ] + rowBd2[ j ]);
        bricks[ brickIdx ].setWidthInCtus (tileColWidth[ tileX ]);
        bricks[ brickIdx ].setHeightInCtus(rowHeight2[ j ]);
        bricks[ brickIdx ].setFirstCtuRsAddr(bricks[ brickIdx ].getColBd() + bricks[ brickIdx ].getRowBd() * frameWidthInCtus);
        brickIdx++;
      }
      #endif
    }
  }

  initCtuBsRsAddrMap();

  for( int i = 0; i < (int)bricks.size(); i++ )
  {
    for( int y = bricks[i].getRowBd(); y < bricks[i].getRowBd() + bricks[i].getHeightInCtus(); y++ )
    {
      for( int x = bricks[i].getColBd(); x < bricks[i].getColBd() + bricks[i].getWidthInCtus(); x++ )
      {
        // brickIdxBsMap in BS scan is brickIdxMap as defined in the draft text
        brickIdxBsMap[ ctuRsToBsAddrMap[ y * frameWidthInCtus+ x ] ] = i;
        // brickIdxRsMap in RS scan is usually required in the software
        brickIdxRsMap[ y * frameWidthInCtus+ x ] = i;
      }
    }
  }
#if JVET_O0143_BOTTOM_RIGHT_BRICK_IDX_DELTA
  if (pps.getRectSliceFlag())
  {
    int numSlicesInPic = (pps.getNumSlicesInPicMinus1() + 1);
    int numBricksInPic = (int)bricks.size();

    std::vector<int> bricksToSliceMap(numBricksInPic);
    std::vector<int> numBricksInSlice(numSlicesInPic);

    m_topLeftBrickIdx.resize(numSlicesInPic);
    m_bottomRightBrickIdx.resize(numSlicesInPic);

    if (numSlicesInPic == 1)
    {
      m_topLeftBrickIdx[0] = 0;
      m_bottomRightBrickIdx[0] = numBricksInPic - 1;
    }
    else
    {
      for (int i = 0; i < numSlicesInPic; i++)
      {
        for (int j = 0; i == 0 && j < numBricksInPic; j++)
        {
          bricksToSliceMap[j] = -1;
        }
        numBricksInSlice[i] = 0;
        m_bottomRightBrickIdx[i] = pps.getBottomRightBrickIdxDelta(i) + ((i == 0) ? 0 : m_bottomRightBrickIdx[i - 1]);
        for (int j = m_bottomRightBrickIdx[i]; j >= 0; j--)
        {
          if (bricks[j].getColBd() <= bricks[m_bottomRightBrickIdx[i]].getColBd() &&
            bricks[j].getRowBd() <= bricks[m_bottomRightBrickIdx[i]].getRowBd() &&
            bricksToSliceMap[j] == -1)
          {
            m_topLeftBrickIdx[i] = j;
            numBricksInSlice[i]++;
            bricksToSliceMap[j] = i;
          }
        }
      }
    }
  }
#endif

}


void BrickMap::initCtuBsRsAddrMap()
{
  const uint32_t picWidthInCtbsY  = pcv->widthInCtus;
  const uint32_t picHeightInCtbsY = pcv->heightInCtus;
  const uint32_t picSizeInCtbsY    = picWidthInCtbsY * picHeightInCtbsY;
  const int numBricksInPic         = (int) bricks.size();

  for( uint32_t ctbAddrRs = 0; ctbAddrRs < picSizeInCtbsY; ctbAddrRs++ )
  {
    const uint32_t tbX = ctbAddrRs % picWidthInCtbsY;
    const uint32_t tbY = ctbAddrRs / picWidthInCtbsY;
    bool brickFound = false;
    int bkIdx = (numBricksInPic - 1);
    for( int i = 0; i < (numBricksInPic - 1)  &&  !brickFound; i++ )
    {
      brickFound = tbX  <  ( bricks[i].getColBd() + bricks[i].getWidthInCtus() )  &&
                   tbY  <  ( bricks[i].getRowBd() + bricks[i].getHeightInCtus() );
      if( brickFound )
      {
        bkIdx = i;
      }
    }
    ctuRsToBsAddrMap[ ctbAddrRs ] = 0;

    for( uint32_t i = 0; i < bkIdx; i++ )
    {
      ctuRsToBsAddrMap[ ctbAddrRs ]  +=  bricks[i].getHeightInCtus() * bricks[i].getWidthInCtus();
    }
    ctuRsToBsAddrMap[ ctbAddrRs ]  += ( tbY - bricks[ bkIdx ].getRowBd() ) * bricks[ bkIdx ].getWidthInCtus() + tbX - bricks[ bkIdx ].getColBd();
  }


  for( uint32_t ctbAddrRs = 0; ctbAddrRs < picSizeInCtbsY; ctbAddrRs++ )
  {
    ctuBsToRsAddrMap[ ctuRsToBsAddrMap[ ctbAddrRs ] ] = ctbAddrRs;
  }
}

uint32_t BrickMap::getSubstreamForCtuAddr(const uint32_t ctuAddr, const bool addressInRaster, Slice *slice) const
{
  const bool wppEnabled = slice->getPPS()->getEntropyCodingSyncEnabledFlag();
  uint32_t subStrm;

  if( (wppEnabled && pcv->heightInCtus > 1) || (numTiles > 1) ) // wavefronts, and possibly tiles being used.
  {
    // needs to be checked
    CHECK (false, "bricks and WPP needs to be checked");

    const uint32_t ctuRsAddr = addressInRaster ? ctuAddr : getCtuBsToRsAddrMap(ctuAddr);
    const uint32_t brickIndex = getBrickIdxRsMap(ctuRsAddr);

    if (wppEnabled)
    {
      const uint32_t firstCtuRsAddrOfTile     = bricks[brickIndex].getFirstCtuRsAddr();
      const uint32_t tileYInCtus              = firstCtuRsAddrOfTile / pcv->widthInCtus;
      const uint32_t ctuLine                  = ctuRsAddr / pcv->widthInCtus;
      const uint32_t startingSubstreamForTile = (tileYInCtus * numTileColumns) + (bricks[brickIndex].getHeightInCtus() * (brickIndex % numTileColumns));
      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      subStrm = brickIndex;
    }
  }
  else
  {
    subStrm = 0;
  }
  return subStrm;
}


Picture::Picture()
{
  brickMap             = nullptr;
  cs                   = nullptr;
  m_bIsBorderExtended  = false;
  usedByCurr           = false;
  longTerm             = false;
  reconstructed        = false;
  neededForOutput      = false;
  referenced           = false;
  layer                = std::numeric_limits<uint32_t>::max();
  fieldPic             = false;
  topField             = false;
#if JVET_N0494_DRAP
  precedingDRAP        = false;
#endif
  for( int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_prevQP[i] = -1;
  }
  m_spliceIdx = NULL;
  m_ctuNums = 0;
}

void Picture::create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned _margin, const bool _decoder)
{
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            =  _margin;
  const Area a      = Area( Position(), size );
  M_BUFS( 0, PIC_RECONSTRUCTION ).create( _chromaFormat, a, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );
  M_BUFS( 0, PIC_RECON_WRAP ).create( _chromaFormat, a, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );

  if( !_decoder )
  {
    M_BUFS( 0, PIC_ORIGINAL ).    create( _chromaFormat, a );
    M_BUFS( 0, PIC_TRUE_ORIGINAL ). create( _chromaFormat, a );
  }
#if !KEEP_PRED_AND_RESI_SIGNALS

  m_ctuArea = UnitArea( _chromaFormat, Area( Position{ 0, 0 }, Size( _maxCUSize, _maxCUSize ) ) );
#endif
  m_hashMap.clearAll();
}

void Picture::destroy()
{
#if ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
  for( int jId = 0; jId < ( PARL_SPLIT_MAX_NUM_THREADS * PARL_WPP_MAX_NUM_THREADS ); jId++ )
#else
  for( int jId = 0; jId < PARL_SPLIT_MAX_NUM_THREADS; jId++ )
#endif
#endif
  for (uint32_t t = 0; t < NUM_PIC_TYPES; t++)
  {
    M_BUFS( jId, t ).destroy();
  }
  m_hashMap.clearAll();
  if( cs )
  {
    cs->destroy();
    delete cs;
    cs = nullptr;
  }

  for( auto &ps : slices )
  {
    delete ps;
  }
  slices.clear();

  for( auto &psei : SEIs )
  {
    delete psei;
  }
  SEIs.clear();

  if ( brickMap )
  {
    brickMap->destroy();
    delete brickMap;
    brickMap = nullptr;
  }
  if (m_spliceIdx)
  {
    delete[] m_spliceIdx;
    m_spliceIdx = NULL;
  }
}

void Picture::createTempBuffers( const unsigned _maxCUSize )
{
#if KEEP_PRED_AND_RESI_SIGNALS
  const Area a( Position{ 0, 0 }, lumaSize() );
#else
  const Area a = m_ctuArea.Y();
#endif

#if ENABLE_SPLIT_PARALLELISM
  scheduler.startParallel();

  for( int jId = 0; jId < scheduler.getNumPicInstances(); jId++ )
#endif
  {
    M_BUFS( jId, PIC_PREDICTION                   ).create( chromaFormat, a,   _maxCUSize );
    M_BUFS( jId, PIC_RESIDUAL                     ).create( chromaFormat, a,   _maxCUSize );
#if ENABLE_SPLIT_PARALLELISM
    if( jId > 0 ) M_BUFS( jId, PIC_RECONSTRUCTION ).create( chromaFormat, Y(), _maxCUSize, margin, MEMORY_ALIGN_DEF_SIZE );
#endif
  }

  if( cs ) cs->rebindPicBufs();
}

void Picture::destroyTempBuffers()
{
#if ENABLE_SPLIT_PARALLELISM
  scheduler.finishParallel();

  for( int jId = 0; jId < scheduler.getNumPicInstances(); jId++ )
#endif
  for( uint32_t t = 0; t < NUM_PIC_TYPES; t++ )
  {
    if( t == PIC_RESIDUAL || t == PIC_PREDICTION ) M_BUFS( jId, t ).destroy();
#if ENABLE_SPLIT_PARALLELISM
    if( t == PIC_RECONSTRUCTION &&       jId > 0 ) M_BUFS( jId, t ).destroy();
#endif
  }

  if( cs ) cs->rebindPicBufs();
}

       PelBuf     Picture::getOrigBuf(const CompArea &blk)        { return getBuf(blk,  PIC_ORIGINAL); }
const CPelBuf     Picture::getOrigBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_ORIGINAL); }
       PelUnitBuf Picture::getOrigBuf(const UnitArea &unit)       { return getBuf(unit, PIC_ORIGINAL); }
const CPelUnitBuf Picture::getOrigBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORIGINAL); }
       PelUnitBuf Picture::getOrigBuf()                           { return M_BUFS(0,    PIC_ORIGINAL); }
const CPelUnitBuf Picture::getOrigBuf()                     const { return M_BUFS(0,    PIC_ORIGINAL); }

       PelBuf     Picture::getOrigBuf(const ComponentID compID)       { return getBuf(compID, PIC_ORIGINAL); }
const CPelBuf     Picture::getOrigBuf(const ComponentID compID) const { return getBuf(compID, PIC_ORIGINAL); }
       PelUnitBuf Picture::getTrueOrigBuf()                           { return M_BUFS(0, PIC_TRUE_ORIGINAL); }
const CPelUnitBuf Picture::getTrueOrigBuf()                     const { return M_BUFS(0, PIC_TRUE_ORIGINAL); }
       PelBuf     Picture::getTrueOrigBuf(const CompArea &blk)        { return getBuf(blk, PIC_TRUE_ORIGINAL); }
const CPelBuf     Picture::getTrueOrigBuf(const CompArea &blk)  const { return getBuf(blk, PIC_TRUE_ORIGINAL); }
       PelBuf     Picture::getPredBuf(const CompArea &blk)        { return getBuf(blk,  PIC_PREDICTION); }
const CPelBuf     Picture::getPredBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_PREDICTION); }
       PelUnitBuf Picture::getPredBuf(const UnitArea &unit)       { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf Picture::getPredBuf(const UnitArea &unit) const { return getBuf(unit, PIC_PREDICTION); }

       PelBuf     Picture::getResiBuf(const CompArea &blk)        { return getBuf(blk,  PIC_RESIDUAL); }
const CPelBuf     Picture::getResiBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_RESIDUAL); }
       PelUnitBuf Picture::getResiBuf(const UnitArea &unit)       { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf Picture::getResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_RESIDUAL); }

       PelBuf     Picture::getRecoBuf(const ComponentID compID, bool wrap)       { return getBuf(compID,                    wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const ComponentID compID, bool wrap) const { return getBuf(compID,                    wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelBuf     Picture::getRecoBuf(const CompArea &blk, bool wrap)            { return getBuf(blk,                       wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const CompArea &blk, bool wrap)      const { return getBuf(blk,                       wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap)           { return getBuf(unit,                      wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap)     const { return getBuf(unit,                      wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf(bool wrap)                                 { return M_BUFS(scheduler.getSplitPicId(), wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelUnitBuf Picture::getRecoBuf(bool wrap)                           const { return M_BUFS(scheduler.getSplitPicId(), wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }

#if JVET_O0299_APS_SCALINGLIST
void Picture::finalInit( const SPS& sps, const PPS& pps, APS** alfApss, APS* lmcsAps, APS* scalingListAps )
#else
void Picture::finalInit(const SPS& sps, const PPS& pps, APS** alfApss, APS* lmcsAps)
#endif
{
  for( auto &sei : SEIs )
  {
    delete sei;
  }
  SEIs.clear();
  clearSliceBuffer();

  if( brickMap )
  {
    brickMap->destroy();
    delete brickMap;
    brickMap = nullptr;
  }

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
#if JVET_O1164_PS
  const int          iWidth = pps.getPicWidthInLumaSamples();
  const int          iHeight = pps.getPicHeightInLumaSamples();
#else
  const int          iWidth = sps.getPicWidthInLumaSamples();
  const int          iHeight = sps.getPicHeightInLumaSamples();
#endif

  if( cs )
  {
    cs->initStructData();
  }
  else
  {
    cs = new CodingStructure( g_globalUnitCache.cuCache, g_globalUnitCache.puCache, g_globalUnitCache.tuCache );
    cs->sps = &sps;
    cs->create( chromaFormatIDC, Area( 0, 0, iWidth, iHeight ), true );
  }

  cs->picture = this;
  cs->slice   = nullptr;  // the slices for this picture have not been set at this point. update cs->slice after swapSliceObject()
  cs->pps     = &pps;
  memcpy(cs->alfApss, alfApss, sizeof(cs->alfApss));
  cs->lmcsAps = lmcsAps;
#if JVET_O0299_APS_SCALINGLIST  
  cs->scalinglistAps = scalingListAps;
#endif

  cs->pcv     = pps.pcv;

  brickMap = new BrickMap;
  brickMap->create( sps, pps );
  if (m_spliceIdx == NULL)
  {
    m_ctuNums = cs->pcv->sizeInCtus;
    m_spliceIdx = new int[m_ctuNums];
    memset(m_spliceIdx, 0, m_ctuNums * sizeof(int));
  }
}

void Picture::allocateNewSlice()
{
  slices.push_back(new Slice);
  Slice& slice = *slices.back();
  memcpy(slice.getAlfAPSs(), cs->alfApss, sizeof(cs->alfApss));

  slice.setLmcsAPS(cs->lmcsAps);
#if JVET_O0299_APS_SCALINGLIST
  slice.setscalingListAPS( cs->scalinglistAps );
#endif

  slice.setPPS( cs->pps);
  slice.setSPS( cs->sps);
  if(slices.size()>=2)
  {
    slice.copySliceInfo( slices[slices.size()-2] );
    slice.initSlice();
  }
}

Slice *Picture::swapSliceObject(Slice * p, uint32_t i)
{
  p->setSPS(cs->sps);
  p->setPPS(cs->pps);
  p->setAlfAPSs(cs->alfApss);

  if(cs->lmcsAps != nullptr)
    p->setLmcsAPS(cs->lmcsAps);
#if JVET_O0299_APS_SCALINGLIST
  if(cs->scalinglistAps != nullptr)
    p->setscalingListAPS( cs->scalinglistAps );
#endif

  Slice * pTmp = slices[i];
  slices[i] = p;
  pTmp->setSPS(0);
  pTmp->setPPS(0);
#if JVET_O_MAX_NUM_ALF_APS_8
  memset(pTmp->getAlfAPSs(), 0, sizeof(*pTmp->getAlfAPSs())*ALF_CTB_MAX_NUM_APS);
#else
  memset(pTmp->getAlfAPSs(), 0, sizeof(*pTmp->getAlfAPSs())*MAX_NUM_APS);
#endif

  pTmp->setLmcsAPS(0);
#if JVET_O0299_APS_SCALINGLIST
  pTmp->setscalingListAPS( 0 );
#endif
  return pTmp;
}

void Picture::clearSliceBuffer()
{
  for (uint32_t i = 0; i < uint32_t(slices.size()); i++)
  {
    delete slices[i];
  }
  slices.clear();
}

#if ENABLE_SPLIT_PARALLELISM

void Picture::finishParallelPart( const UnitArea& area )
{
  const UnitArea clipdArea = clipArea( area, *this );
  const int      sourceID  = scheduler.getSplitPicId( 0 );
  CHECK( scheduler.getSplitJobId() > 0, "Finish-CU cannot be called from within a mode- or split-parallelized block!" );

  // distribute the reconstruction across all of the parallel workers
  for( int tId = 1; tId < scheduler.getNumSplitThreads(); tId++ )
  {
    const int destID = scheduler.getSplitPicId( tId );

    M_BUFS( destID, PIC_RECONSTRUCTION ).subBuf( clipdArea ).copyFrom( M_BUFS( sourceID, PIC_RECONSTRUCTION ).subBuf( clipdArea ) );
  }
}

#if ENABLE_WPP_PARALLELISM
void Picture::finishCtuPart( const UnitArea& ctuArea )
{
  const UnitArea clipdArea = clipArea( ctuArea, *this );
  const int      sourceID  = scheduler.getSplitPicId( 0 );
  // distribute the reconstruction across all of the parallel workers
  for( int dataId = 0; dataId < scheduler.getNumPicInstances(); dataId++ )
  {
    if( dataId == sourceID ) continue;

    M_BUFS( dataId, PIC_RECONSTRUCTION ).subBuf( clipdArea ).copyFrom( M_BUFS( sourceID, PIC_RECONSTRUCTION ).subBuf( clipdArea ) );
  }
}
#endif

#endif

#if JVET_O1164_RPR
const TFilterCoeff DownsamplingFilterSRC[8][16][12] =
{
    { // D = 1
      {   0,   0,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0 },
      {   0,   0,   0,   2,  -6, 127,   7,  -2,   0,   0,   0,   0 },
      {   0,   0,   0,   3, -12, 125,  16,  -5,   1,   0,   0,   0 },
      {   0,   0,   0,   4, -16, 120,  26,  -7,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -18, 114,  36, -10,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -20, 107,  46, -12,   2,   0,   0,   0 },
      {   0,   0,   0,   5, -21,  99,  57, -15,   3,   0,   0,   0 },
      {   0,   0,   0,   5, -20,  89,  68, -18,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -19,  79,  79, -19,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -18,  68,  89, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   3, -15,  57,  99, -21,   5,   0,   0,   0 },
      {   0,   0,   0,   2, -12,  46, 107, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   1, -10,  36, 114, -18,   5,   0,   0,   0 },
      {   0,   0,   0,   1,  -7,  26, 120, -16,   4,   0,   0,   0 },
      {   0,   0,   0,   1,  -5,  16, 125, -12,   3,   0,   0,   0 },
      {   0,   0,   0,   0,  -2,   7, 127,  -6,   2,   0,   0,   0 }
    },
    { // D = 1.5
      {   0,   2,   0, -14,  33,  86,  33, -14,   0,   2,   0,   0 },
      {   0,   1,   1, -14,  29,  85,  38, -13,  -1,   2,   0,   0 },
      {   0,   1,   2, -14,  24,  84,  43, -12,  -2,   2,   0,   0 },
      {   0,   1,   2, -13,  19,  83,  48, -11,  -3,   2,   0,   0 },
      {   0,   0,   3, -13,  15,  81,  53, -10,  -4,   3,   0,   0 },
      {   0,   0,   3, -12,  11,  79,  57,  -8,  -5,   3,   0,   0 },
      {   0,   0,   3, -11,   7,  76,  62,  -5,  -7,   3,   0,   0 },
      {   0,   0,   3, -10,   3,  73,  65,  -2,  -7,   3,   0,   0 },
      {   0,   0,   3,  -9,   0,  70,  70,   0,  -9,   3,   0,   0 },
      {   0,   0,   3,  -7,  -2,  65,  73,   3, -10,   3,   0,   0 },
      {   0,   0,   3,  -7,  -5,  62,  76,   7, -11,   3,   0,   0 },
      {   0,   0,   3,  -5,  -8,  57,  79,  11, -12,   3,   0,   0 },
      {   0,   0,   3,  -4, -10,  53,  81,  15, -13,   3,   0,   0 },
      {   0,   0,   2,  -3, -11,  48,  83,  19, -13,   2,   1,   0 },
      {   0,   0,   2,  -2, -12,  43,  84,  24, -14,   2,   1,   0 },
      {   0,   0,   2,  -1, -13,  38,  85,  29, -14,   1,   1,   0 }
    },
    { // D = 2
      {   0,   5,   -6,  -10,  37,  76,   37,  -10,  -6,    5,  0,   0}, //0
      {   0,   5,   -4,  -11,  33,  76,   40,  -9,    -7,    5,  0,   0}, //1
      //{   0,   5,   -3,  -12,  28,  75,   44,  -7,    -8,    5,  1,   0}, //2
      {  -1,   5,   -3,  -12,  29,  75,   45,  -7,    -8,   5,  0,   0}, //2 new coefficients in m24499
      {  -1,   4,   -2,  -13,  25,  75,   48,  -5,    -9,    5,  1,   0}, //3
      {  -1,   4,   -1,  -13,  22,  73,   52,  -3,    -10,  4,  1,   0}, //4
      {  -1,   4,   0,    -13,  18,  72,   55,  -1,    -11,  4,  2,  -1}, //5
      {  -1,   4,   1,    -13,  14,  70,   59,  2,    -12,  3,  2,  -1}, //6
      {  -1,   3,   1,    -13,  11,  68,   62,  5,    -12,  3,  2,  -1}, //7
      {  -1,   3,   2,    -13,  8,  65,   65,  8,    -13,  2,  3,  -1}, //8
      {  -1,   2,   3,    -12,  5,  62,   68,  11,    -13,  1,  3,  -1}, //9
      {  -1,   2,   3,    -12,  2,  59,   70,  14,    -13,  1,  4,  -1}, //10
      {  -1,   2,   4,    -11,  -1,  55,   72,  18,    -13,  0,  4,  -1}, //11
      {   0,   1,   4,    -10,  -3,  52,   73,  22,    -13,  -1,  4,  -1}, //12
      {   0,   1,   5,    -9,    -5,  48,   75,  25,    -13,  -2,  4,  -1}, //13
      //{   0,   1,   5,    -8,    -7,  44,   75,  28,    -12,  -3,  5,   0}, //14
      {    0,   0,   5,    -8,   -7,  45,   75,  29,    -12,  -3,  5,  -1}  , //14 new coefficients in m24499  
      {   0,   0,   5,    -7,    -9,  40,   76,  33,    -11,  -4,  5,   0}, //15
    },
    { // D = 2.5
      {   2,  -3,   -9,  6,   39,  58,   39,  6,   -9,  -3,    2,    0}, // 0
      {   2,  -3,   -9,  4,   38,  58,   43,  7,   -9,  -4,    1,    0}, // 1
      {   2,  -2,   -9,  2,   35,  58,   44,  9,   -8,  -4,    1,    0}, // 2
      {   1,  -2,   -9,  1,   34,  58,   46,  11,   -8,  -5,    1,    0}, // 3
      //{   1,  -1,   -8,  -1,   31,  57,   48,  13,   -8,  -5,    1,    0}, // 4
      {   1,  -1,   -8,  -1,   31,  57,   47,  13,   -7,  -5,    1,    0},  // 4 new coefficients in m24499  
      {   1,  -1,   -8,  -2,   29,  56,   49,  15,   -7,  -6,    1,    1}, // 5
      {   1,  0,   -8,  -3,   26,  55,   51,  17,   -7,  -6,    1,    1}, // 6
      {   1,  0,   -7,  -4,   24,  54,   52,  19,   -6,  -7,    1,    1}, // 7
      {   1,  0,   -7,  -5,   22,  53,   53,  22,   -5,  -7,    0,    1}, // 8
      {   1,  1,   -7,  -6,   19,  52,   54,  24,   -4,  -7,    0,    1}, // 9
      {   1,  1,   -6,  -7,   17,  51,   55,  26,   -3,  -8,    0,    1}, // 10
      {   1,  1,   -6,  -7,   15,  49,   56,  29,   -2,  -8,    -1,    1}, // 11
      //{   0,  1,   -5,  -8,   13,  48,   57,  31,   -1,  -8,    -1,    1}, // 12 new coefficients in m24499
      {   0,  1,   -5,  -7,   13,  47,  57,  31,  -1,    -8,   -1,    1}, // 12   
      {   0,  1,   -5,  -8,   11,  46,   58,  34,   1,    -9,    -2,    1}, // 13
      {   0,  1,   -4,  -8,   9,    44,   58,  35,   2,    -9,    -2,    2}, // 14
      {   0,  1,   -4,  -9,   7,    43,   58,  38,   4,    -9,    -3,    2}, // 15
    },
    { // D = 3
      {  -2,  -7,   0,  17,  35,  43,  35,  17,   0,  -7,  -5,   2 },
      {  -2,  -7,  -1,  16,  34,  43,  36,  18,   1,  -7,  -5,   2 },
      {  -1,  -7,  -1,  14,  33,  43,  36,  19,   1,  -6,  -5,   2 },
      {  -1,  -7,  -2,  13,  32,  42,  37,  20,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  12,  31,  42,  38,  21,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  11,  30,  42,  39,  23,   4,  -6,  -6,   1 },
      {   0,  -7,  -4,  10,  29,  42,  40,  24,   5,  -6,  -6,   1 },
      {   1,  -7,  -4,   9,  27,  41,  40,  25,   6,  -5,  -6,   1 },
      {   1,  -6,  -5,   7,  26,  41,  41,  26,   7,  -5,  -6,   1 },
      {   1,  -6,  -5,   6,  25,  40,  41,  27,   9,  -4,  -7,   1 },
      {   1,  -6,  -6,   5,  24,  40,  42,  29,  10,  -4,  -7,   0 },
      {   1,  -6,  -6,   4,  23,  39,  42,  30,  11,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  21,  38,  42,  31,  12,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  20,  37,  42,  32,  13,  -2,  -7,  -1 },
      {   2,  -5,  -6,   1,  19,  36,  43,  33,  14,  -1,  -7,  -1 },
      {   2,  -5,  -7,   1,  18,  36,  43,  34,  16,  -1,  -7,  -2 }
    },
    { // D = 3.5
      {  -6,  -3,   5,  19,  31,  36,  31,  19,   5,  -3,  -6,   0 },
      {  -6,  -4,   4,  18,  31,  37,  32,  20,   6,  -3,  -6,  -1 },
      {  -6,  -4,   4,  17,  30,  36,  33,  21,   7,  -3,  -6,  -1 },
      {  -5,  -5,   3,  16,  30,  36,  33,  22,   8,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  29,  36,  34,  23,   9,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  28,  36,  34,  24,  10,  -2,  -6,  -3 },
      {  -4,  -5,   1,  14,  27,  36,  35,  24,  10,  -1,  -6,  -3 },
      {  -4,  -5,   0,  13,  26,  35,  35,  25,  11,   0,  -5,  -3 },
      {  -4,  -6,   0,  12,  26,  36,  36,  26,  12,   0,  -6,  -4 },
      {  -3,  -5,   0,  11,  25,  35,  35,  26,  13,   0,  -5,  -4 },
      {  -3,  -6,  -1,  10,  24,  35,  36,  27,  14,   1,  -5,  -4 },
      {  -3,  -6,  -2,  10,  24,  34,  36,  28,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   9,  23,  34,  36,  29,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   8,  22,  33,  36,  30,  16,   3,  -5,  -5 },
      {  -1,  -6,  -3,   7,  21,  33,  36,  30,  17,   4,  -4,  -6 },
      {  -1,  -6,  -3,   6,  20,  32,  37,  31,  18,   4,  -4,  -6 }
    },
    { // D = 4
      {  -9,   0,   9,  20,  28,  32,  28,  20,   9,   0,  -9,   0 },
      {  -9,   0,   8,  19,  28,  32,  29,  20,  10,   0,  -4,  -5 },
      {  -9,  -1,   8,  18,  28,  32,  29,  21,  10,   1,  -4,  -5 },
      {  -9,  -1,   7,  18,  27,  32,  30,  22,  11,   1,  -4,  -6 },
      {  -8,  -2,   6,  17,  27,  32,  30,  22,  12,   2,  -4,  -6 },
      {  -8,  -2,   6,  16,  26,  32,  31,  23,  12,   2,  -4,  -6 },
      {  -8,  -2,   5,  16,  26,  31,  31,  23,  13,   3,  -3,  -7 },
      {  -8,  -3,   5,  15,  25,  31,  31,  24,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  25,  31,  31,  25,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  24,  31,  31,  25,  15,   5,  -3,  -8 },
      {  -7,  -3,   3,  13,  23,  31,  31,  26,  16,   5,  -2,  -8 },
      {  -6,  -4,   2,  12,  23,  31,  32,  26,  16,   6,  -2,  -8 },
      {  -6,  -4,   2,  12,  22,  30,  32,  27,  17,   6,  -2,  -8 },
      {  -6,  -4,   1,  11,  22,  30,  32,  27,  18,   7,  -1,  -9 },
      {  -5,  -4,   1,  10,  21,  29,  32,  28,  18,   8,  -1,  -9 },
      {  -5,  -4,   0,  10,  20,  29,  32,  28,  19,   8,   0,  -9 }
    },
    { // D = 5.5
      {  -8,   7,  13,  18,  22,  24,  22,  18,  13,   7,   2, -10 },
      {  -8,   7,  13,  18,  22,  23,  22,  19,  13,   7,   2, -10 },
      {  -8,   6,  12,  18,  22,  23,  22,  19,  14,   8,   2, -10 },
      {  -9,   6,  12,  17,  22,  23,  23,  19,  14,   8,   3, -10 },
      {  -9,   6,  12,  17,  21,  23,  23,  19,  14,   9,   3, -10 },
      {  -9,   5,  11,  17,  21,  23,  23,  20,  15,   9,   3, -10 },
      {  -9,   5,  11,  16,  21,  23,  23,  20,  15,   9,   4, -10 },
      {  -9,   5,  10,  16,  21,  23,  23,  20,  15,  10,   4, -10 },
      { -10,   5,  10,  16,  20,  23,  23,  20,  16,  10,   5, -10 },
      { -10,   4,  10,  15,  20,  23,  23,  21,  16,  10,   5,  -9 },
      { -10,   4,   9,  15,  20,  23,  23,  21,  16,  11,   5,  -9 },
      { -10,   3,   9,  15,  20,  23,  23,  21,  17,  11,   5,  -9 },
      { -10,   3,   9,  14,  19,  23,  23,  21,  17,  12,   6,  -9 },
      { -10,   3,   8,  14,  19,  23,  23,  22,  17,  12,   6,  -9 },
      { -10,   2,   8,  14,  19,  22,  23,  22,  18,  12,   6,  -8 },
      { -10,   2,   7,  13,  19,  22,  23,  22,  18,  13,   7,  -8 }
    }
};

void Picture::sampleRateConv( const Pel* orgSrc, SizeType orgWidth, SizeType orgHeight, SizeType orgStride, Pel* scaledSrc, SizeType scaledWidth, SizeType scaledHeight, SizeType paddedWidth, SizeType paddedHeight, SizeType scaledStride, const int bitDepth, const bool useLumaFilter, const bool downsampling )
{
  if( orgWidth == scaledWidth && orgHeight == scaledHeight )
  {
    for( int j = 0; j < orgHeight; j++ )
    {
      memcpy( scaledSrc + j * scaledStride, orgSrc + j * orgStride, sizeof( Pel ) * orgWidth );
    }

    return;
  }

  const TFilterCoeff* filterHor = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff* filterVer = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const int numFracPositions = useLumaFilter ? 15 : 31;
  const int numFracShift = useLumaFilter ? 4 : 5;

  if( downsampling )
  {
    int verFilter = 0;
    int horFilter = 0;
    if( 4 * orgHeight > 15 * scaledHeight )   verFilter = 7;
    else if( 7 * orgHeight > 20 * scaledHeight )   verFilter = 6;
    else if( 2 * orgHeight > 5 * scaledHeight )   verFilter = 5;
    else if( 1 * orgHeight > 2 * scaledHeight )   verFilter = 4;
    else if( 3 * orgHeight > 5 * scaledHeight )   verFilter = 3;
    else if( 4 * orgHeight > 5 * scaledHeight )   verFilter = 2;
    else if( 19 * orgHeight > 20 * scaledHeight )   verFilter = 1;

    if( 4 * orgWidth > 15 * scaledWidth )   horFilter = 7;
    else if( 7 * orgWidth > 20 * scaledWidth )   horFilter = 6;
    else if( 2 * orgWidth > 5 * scaledWidth )   horFilter = 5;
    else if( 1 * orgWidth > 2 * scaledWidth )   horFilter = 4;
    else if( 3 * orgWidth > 5 * scaledWidth )   horFilter = 3;
    else if( 4 * orgWidth > 5 * scaledWidth )   horFilter = 2;
    else if( 19 * orgWidth > 20 * scaledWidth )   horFilter = 1;

    filterHor = &DownsamplingFilterSRC[horFilter][0][0];
    filterVer = &DownsamplingFilterSRC[verFilter][0][0];
  }

  const int filerLength = downsampling ? 12 : ( useLumaFilter ? NTAPS_LUMA : NTAPS_CHROMA );
  const int log2Norm = downsampling ? 14 : 12;

  int *buf = new int[orgHeight * paddedWidth];
  int maxVal = ( 1 << bitDepth ) - 1;

  CHECK( bitDepth > 17, "Overflow may happen!" );

  for( int i = 0; i < paddedWidth; i++ )
  {
    const Pel* org = orgSrc;
    int integer = ( i * orgWidth ) / scaledWidth;
    int frac = ( ( i * orgWidth << numFracShift ) / scaledWidth ) & numFracPositions;

    int* tmp = buf + i;

    for( int j = 0; j < orgHeight; j++ )
    {
      int sum = 0;
      const TFilterCoeff* f = filterHor + frac * filerLength;

      for( int k = 0; k < filerLength; k++ )
      {
        int xInt = std::min<int>( std::max( 0, integer + k - filerLength / 2 + 1 ), orgWidth - 1 );
        sum += f[k] * org[xInt]; // postpone horizontal filtering gain removal after vertical filtering
      }

      *tmp = sum;

      tmp += paddedWidth;
      org += orgStride;
    }
  }

  Pel* dst = scaledSrc;

  for( int j = 0; j < paddedHeight; j++ )
  {
    int integer = ( j * orgHeight ) / scaledHeight;
    int frac = ( ( j * orgHeight << numFracShift ) / scaledHeight ) & numFracPositions;

    for( int i = 0; i < paddedWidth; i++ )
    {
      int sum = 0;
      int* tmp = buf + i;
      const TFilterCoeff* f = filterVer + frac * filerLength;

      for( int k = 0; k < filerLength; k++ )
      {
        int yInt = std::min<int>( std::max( 0, integer + k - filerLength / 2 + 1 ), orgHeight - 1 );
        sum += f[k] * tmp[yInt*paddedWidth];
      }

      dst[i] = std::min<int>( std::max( 0, ( sum + ( 1 << ( log2Norm - 1 ) ) ) >> log2Norm ), maxVal );
    }

    dst += scaledStride;
  }

  delete[] buf;
}

#if RPR_CONF_WINDOW
void Picture::rescalePicture( const CPelUnitBuf& beforeScaling, const Window& confBefore, const PelUnitBuf& afterScaling, const Window& confAfter, const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling )
{
  for( int comp = 0; comp < ::getNumberValidComponents( chromaFormatIDC ); comp++ )
  {
    ComponentID compID = ComponentID( comp );
    const CPelBuf& beforeScale = beforeScaling.get( compID );
    const PelBuf& afterScale = afterScaling.get( compID );
    int widthBefore = beforeScale.width - (((confBefore.getWindowLeftOffset() + confBefore.getWindowRightOffset()) * SPS::getWinUnitX(chromaFormatIDC)) >> getChannelTypeScaleX((ChannelType)(comp > 0), chromaFormatIDC));
    int heightBefore = beforeScale.height - (((confBefore.getWindowTopOffset() + confBefore.getWindowBottomOffset()) * SPS::getWinUnitY(chromaFormatIDC)) >> getChannelTypeScaleY((ChannelType)(comp > 0), chromaFormatIDC));
    int widthAfter = afterScale.width - (((confAfter.getWindowLeftOffset() + confAfter.getWindowRightOffset()) * SPS::getWinUnitX(chromaFormatIDC)) >> getChannelTypeScaleX((ChannelType)(comp > 0), chromaFormatIDC));
    int heightAfter = afterScale.height - (((confAfter.getWindowTopOffset() + confAfter.getWindowBottomOffset()) * SPS::getWinUnitY(chromaFormatIDC)) >> getChannelTypeScaleY((ChannelType)(comp > 0), chromaFormatIDC));

    Picture::sampleRateConv( beforeScale.buf,  widthBefore, heightBefore, beforeScale.stride, afterScale.buf, widthAfter, heightAfter, afterScale.width, afterScale.height, afterScale.stride, bitDepths.recon[comp], downsampling || useLumaFilter ? true : isLuma(compID), downsampling );
  }
}
#else
void Picture::rescalePicture(const CPelUnitBuf& beforeScaling, const PelUnitBuf& afterScaling, const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling)
{
  for (int comp = 0; comp < ::getNumberValidComponents(chromaFormatIDC); comp++)
  {
    ComponentID compID = ComponentID(comp);
    const CPelBuf& beforeScale = beforeScaling.get(compID);
    const PelBuf& afterScale = afterScaling.get(compID);

    Picture::sampleRateConv(beforeScale.buf, beforeScale.width, beforeScale.height, beforeScale.stride, afterScale.buf, afterScale.width, afterScale.height, afterScale.width, afterScale.height, afterScale.stride, bitDepths.recon[comp], downsampling || useLumaFilter ? true : isLuma(compID), downsampling);
  }
}
#endif
#endif

void Picture::extendPicBorder()
{
  if ( m_bIsBorderExtended )
  {
    return;
  }

  for(int comp=0; comp<getNumberValidComponents( cs->area.chromaFormat ); comp++)
  {
    ComponentID compID = ComponentID( comp );
    PelBuf p = M_BUFS( 0, PIC_RECONSTRUCTION ).get( compID );
    Pel *piTxt = p.bufAt(0,0);
    int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

    Pel*  pi = piTxt;
    // do left and right margins
      for (int y = 0; y < p.height; y++)
      {
        for (int x = 0; x < xmargin; x++ )
        {
          pi[ -xmargin + x ] = pi[0];
          pi[  p.width + x ] = pi[p.width-1];
        }
        pi += p.stride;
      }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (p.stride + xmargin);
    // pi is now the (-marginX, height-1)
    for (int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
    }

    // pi is still (-marginX, height-1)
    pi -= ((p.height-1) * p.stride);
    // pi is now (-marginX, 0)
    for (int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
    }

    // reference picture with horizontal wrapped boundary
    if (cs->sps->getWrapAroundEnabledFlag())
    {
      p = M_BUFS( 0, PIC_RECON_WRAP ).get( compID );
      p.copyFrom(M_BUFS( 0, PIC_RECONSTRUCTION ).get( compID ));
      piTxt = p.bufAt(0,0);
      pi = piTxt;
      int xoffset = cs->sps->getWrapAroundOffset() >> getComponentScaleX( compID, cs->area.chromaFormat );
      for (int y = 0; y < p.height; y++)
      {
        for (int x = 0; x < xmargin; x++ )
        {
          if( x < xoffset )
          {
            pi[ -x - 1 ] = pi[ -x - 1 + xoffset ];
            pi[  p.width + x ] = pi[ p.width + x - xoffset ];
          }
          else
          {
            pi[ -x - 1 ] = pi[ 0 ];
            pi[  p.width + x ] = pi[ p.width - 1 ];
          }
        }
        pi += p.stride;
      }
      pi -= (p.stride + xmargin);
      for (int y = 0; y < ymargin; y++ )
      {
        ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
      }
      pi -= ((p.height-1) * p.stride);
      for (int y = 0; y < ymargin; y++ )
      {
        ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
      }
    }
  }

  m_bIsBorderExtended = true;
}

PelBuf Picture::getBuf( const ComponentID compID, const PictureType &type )
{
#if JVET_O1164_RPR
  return M_BUFS( ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL || type == PIC_ORIGINAL_INPUT || type == PIC_TRUE_ORIGINAL_INPUT ) ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
#else
  return M_BUFS( ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL ) ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
#endif
}

const CPelBuf Picture::getBuf( const ComponentID compID, const PictureType &type ) const
{
#if JVET_O1164_RPR
  return M_BUFS( ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL || type == PIC_ORIGINAL_INPUT || type == PIC_TRUE_ORIGINAL_INPUT ) ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
#else
  return M_BUFS( ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL ) ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
#endif
}

PelBuf Picture::getBuf( const CompArea &blk, const PictureType &type )
{
  if( !blk.valid() )
  {
    return PelBuf();
  }

#if ENABLE_SPLIT_PARALLELISM
#if JVET_O1164_RPR
  const int jId = ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL || type == PIC_ORIGINAL_INPUT || type == PIC_TRUE_ORIGINAL_INPUT ) ? 0 : scheduler.getSplitPicId();
#else
  const int jId = ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL ) ? 0 : scheduler.getSplitPicId();
#endif
#endif
#if !KEEP_PRED_AND_RESI_SIGNALS
  if( type == PIC_RESIDUAL || type == PIC_PREDICTION )
  {
    CompArea localBlk = blk;
    localBlk.x &= ( cs->pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    localBlk.y &= ( cs->pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );

    return M_BUFS( jId, type ).getBuf( localBlk );
  }
#endif

  return M_BUFS( jId, type ).getBuf( blk );
}

const CPelBuf Picture::getBuf( const CompArea &blk, const PictureType &type ) const
{
  if( !blk.valid() )
  {
    return PelBuf();
  }

#if ENABLE_SPLIT_PARALLELISM
  const int jId = ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL ) ? 0 : scheduler.getSplitPicId();

#endif
#if !KEEP_PRED_AND_RESI_SIGNALS
  if( type == PIC_RESIDUAL || type == PIC_PREDICTION )
  {
    CompArea localBlk = blk;
    localBlk.x &= ( cs->pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    localBlk.y &= ( cs->pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );

    return M_BUFS( jId, type ).getBuf( localBlk );
  }
#endif

  return M_BUFS( jId, type ).getBuf( blk );
}

PelUnitBuf Picture::getBuf( const UnitArea &unit, const PictureType &type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getBuf( const UnitArea &unit, const PictureType &type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

Pel* Picture::getOrigin( const PictureType &type, const ComponentID compID ) const
{
#if ENABLE_SPLIT_PARALLELISM
  const int jId = ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL ) ? 0 : scheduler.getSplitPicId();
#endif
  return M_BUFS( jId, type ).getOrigin( compID );

}

void Picture::createSpliceIdx(int nums)
{
  m_ctuNums = nums;
  m_spliceIdx = new int[m_ctuNums];
  memset(m_spliceIdx, 0, m_ctuNums * sizeof(int));
}

bool Picture::getSpliceFull()
{
  int count = 0;
  for (int i = 0; i < m_ctuNums; i++)
  {
    if (m_spliceIdx[i] != 0)
      count++;
  }
  if (count < m_ctuNums * 0.25)
    return false;
  return true;
}

void Picture::addPictureToHashMapForInter()
{
#if JVET_O1164_PS
  int picWidth = slices[0]->getPPS()->getPicWidthInLumaSamples();
  int picHeight = slices[0]->getPPS()->getPicHeightInLumaSamples();
#else
  int picWidth = slices[0]->getSPS()->getPicWidthInLumaSamples();
  int picHeight = slices[0]->getSPS()->getPicHeightInLumaSamples();
#endif
  uint32_t* blockHashValues[2][2];
  bool* bIsBlockSame[2][3];

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      blockHashValues[i][j] = new uint32_t[picWidth*picHeight];
    }

    for (int j = 0; j < 3; j++)
    {
      bIsBlockSame[i][j] = new bool[picWidth*picHeight];
    }
  }
  m_hashMap.create(picWidth, picHeight);
  m_hashMap.generateBlock2x2HashValue(getOrigBuf(), picWidth, picHeight, slices[0]->getSPS()->getBitDepths(), blockHashValues[0], bIsBlockSame[0]);//2x2
  m_hashMap.generateBlockHashValue(picWidth, picHeight, 4, 4, blockHashValues[0], blockHashValues[1], bIsBlockSame[0], bIsBlockSame[1]);//4x4
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[1], bIsBlockSame[1][2], picWidth, picHeight, 4, 4);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 8, 8, blockHashValues[1], blockHashValues[0], bIsBlockSame[1], bIsBlockSame[0]);//8x8
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[0], bIsBlockSame[0][2], picWidth, picHeight, 8, 8);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 16, 16, blockHashValues[0], blockHashValues[1], bIsBlockSame[0], bIsBlockSame[1]);//16x16
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[1], bIsBlockSame[1][2], picWidth, picHeight, 16, 16);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 32, 32, blockHashValues[1], blockHashValues[0], bIsBlockSame[1], bIsBlockSame[0]);//32x32
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[0], bIsBlockSame[0][2], picWidth, picHeight, 32, 32);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 64, 64, blockHashValues[0], blockHashValues[1], bIsBlockSame[0], bIsBlockSame[1]);//64x64
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[1], bIsBlockSame[1][2], picWidth, picHeight, 64, 64);

  m_hashMap.setInitial();

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      delete[] blockHashValues[i][j];
    }

    for (int j = 0; j < 3; j++)
    {
      delete[] bIsBlockSame[i][j];
    }
  }
}
