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

  const uint32_t numCtusInFrame = pcv->sizeInCtus;
  brickIdxRsMap    = new uint32_t[numCtusInFrame];
  brickIdxBsMap    = new uint32_t[numCtusInFrame];
  ctuBsToRsAddrMap = new uint32_t[numCtusInFrame+1];
  ctuRsToBsAddrMap = new uint32_t[numCtusInFrame+1];

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
  }
  else
  {
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
    int tileX = tileIdx % ( pps.getNumTileColumnsMinus1() + 1 );
    int tileY = tileIdx / ( pps.getNumTileColumnsMinus1() + 1 );

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

void Picture::finalInit(const SPS& sps, const PPS& pps, APS** alfApss, APS& lmcsAps)
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
  const int          iWidth = sps.getPicWidthInLumaSamples();
  const int          iHeight = sps.getPicHeightInLumaSamples();

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
  cs->lmcsAps = &lmcsAps;

#if HEVC_VPS
  cs->vps     = nullptr;
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

  p->setLmcsAPS(cs->lmcsAps);

  Slice * pTmp = slices[i];
  slices[i] = p;
  pTmp->setSPS(0);
  pTmp->setPPS(0);
  memset(pTmp->getAlfAPSs(), 0, sizeof(*pTmp->getAlfAPSs())*MAX_NUM_APS);

  pTmp->setLmcsAPS(0);
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
  return M_BUFS( ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL ) ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
}

const CPelBuf Picture::getBuf( const ComponentID compID, const PictureType &type ) const
{
  return M_BUFS( ( type == PIC_ORIGINAL || type == PIC_TRUE_ORIGINAL ) ? 0 : scheduler.getSplitPicId(), type ).getBuf( compID );
}

PelBuf Picture::getBuf( const CompArea &blk, const PictureType &type )
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
  int picWidth = slices[0]->getSPS()->getPicWidthInLumaSamples();
  int picHeight = slices[0]->getSPS()->getPicHeightInLumaSamples();
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
