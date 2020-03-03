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

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "CommonLib/CommonDef.h"
#include "BitstreamExtractorApp.h"
#include "DecoderLib/AnnexBread.h"
#include "DecoderLib/NALread.h"
#include "EncoderLib/NALwrite.h"
#include "EncoderLib/VLCWriter.h"
#include "EncoderLib/AnnexBwrite.h"

BitstreamExtractorApp::BitstreamExtractorApp()
{
}

void BitstreamExtractorApp::xPrintVPSInfo (VPS *vps)
{
  msg (VERBOSE, "VPS Info: \n");
  msg (VERBOSE, "  VPS ID         : %d\n", vps->getVPSId());
  msg (VERBOSE, "  Max layers     : %d\n", vps->getMaxLayers());
  msg (VERBOSE, "  Max sub-layers : %d\n", vps->getMaxSubLayers());
  msg (VERBOSE, "  Number of OLS  : %d\n", vps->getNumOutputLayerSets());
  for (int olsIdx=0; olsIdx < vps->getNumOutputLayerSets(); olsIdx++)
  {
    vps->deriveTargetOutputLayerSet(olsIdx);
    msg (VERBOSE, "    OLS # %d\n", olsIdx);
    msg (VERBOSE, "      Output layers: ");
    for( int i = 0; i < vps->m_targetOutputLayerIdSet.size(); i++ )
    {
      msg (VERBOSE, "%d  ", vps->m_targetOutputLayerIdSet[i]);
    }
    msg (VERBOSE, "\n");

    msg (VERBOSE, "      Target layers: ");
    for( int i = 0; i < vps->m_targetLayerIdSet.size(); i++ )
    {
      msg (VERBOSE, "%d  ", vps->m_targetLayerIdSet[i]);
    }
    msg (VERBOSE, "\n");
  }
}

void BitstreamExtractorApp::xWriteVPS(VPS *vps, std::ostream& out, int layerId, int temporalId)
{
  // create a new NAL unit for output
  OutputNALUnit naluOut (NAL_UNIT_VPS, layerId, temporalId);
  CHECK( naluOut.m_temporalId, "The value of TemporalId of VPS NAL units shall be equal to 0" );

  // write the VPS to the newly created NAL unit buffer
  m_hlSyntaxWriter.setBitstream( &naluOut.m_Bitstream );
  m_hlSyntaxWriter.codeVPS( vps );

  // create a dummy AU
  AccessUnit tmpAu;
  // convert to EBSP (this adds emulation prevention!) and add into NAL unit
  tmpAu.push_back(new NALUnitEBSP(naluOut));

  // write the dummy AU
  // note: The first NAL unit in an access unit will be written with a 4-byte start code
  //       Parameter sets are also coded with a 4-byte start code, so writing the dummy
  //       AU works without chaning the start code length.
  //       This cannot be done for VLC NAL units!
  writeAnnexB (out, tmpAu);
}

void BitstreamExtractorApp::xWriteSPS(SPS *sps, std::ostream& out, int layerId, int temporalId)
{
  // create a new NAL unit for output
  OutputNALUnit naluOut (NAL_UNIT_SPS, layerId, temporalId);
  CHECK( naluOut.m_temporalId, "The value of TemporalId of SPS NAL units shall be equal to 0" );

  // write the SPS to the newly created NAL unit buffer
  m_hlSyntaxWriter.setBitstream( &naluOut.m_Bitstream );
  m_hlSyntaxWriter.codeSPS( sps );

  // create a dummy AU
  AccessUnit tmpAu;
  // convert to EBSP (this adds emulation prevention!) and add into NAL unit
  tmpAu.push_back(new NALUnitEBSP(naluOut));

  // write the dummy AU
  // note: The first NAL unit in an access unit will be written with a 4-byte start code
  //       Parameter sets are also coded with a 4-byte start code, so writing the dummy
  //       AU works without chaning the start code length.
  //       This cannot be done for VLC NAL units!
  writeAnnexB (out, tmpAu);
}

void BitstreamExtractorApp::xWritePPS(PPS *pps, std::ostream& out, int layerId, int temporalId)
{
  // create a new NAL unit for output
  OutputNALUnit naluOut (NAL_UNIT_PPS, layerId, temporalId);

  // write the PPS to the newly created NAL unit buffer
  m_hlSyntaxWriter.setBitstream( &naluOut.m_Bitstream );
  m_hlSyntaxWriter.codePPS( pps );

  // create a dummy AU
  AccessUnit tmpAu;
  // convert to EBSP (this adds emulation prevention!) and add into NAL unit
  tmpAu.push_back(new NALUnitEBSP(naluOut));

  // write the dummy AU
  // note: The first NAL unit in an access unit will be written with a 4-byte start code
  //       Parameter sets are also coded with a 4-byte start code, so writing the dummy
  //       AU works without chaning the start code length.
  //       This cannot be done for VLC NAL units!
  writeAnnexB (out, tmpAu);
}


uint32_t BitstreamExtractorApp::decode()
{
  std::ifstream bitstreamFileIn(m_bitstreamFileNameIn.c_str(), std::ifstream::in | std::ifstream::binary);
  if (!bitstreamFileIn)
  {
    EXIT( "failed to open bitstream file " << m_bitstreamFileNameIn.c_str() << " for reading" ) ;
  }

  std::ofstream bitstreamFileOut(m_bitstreamFileNameOut.c_str(), std::ifstream::out | std::ifstream::binary);

  InputByteStream bytestream(bitstreamFileIn);

  bitstreamFileIn.clear();
  bitstreamFileIn.seekg( 0, std::ios::beg );

  int unitCnt = 0;

  while (!!bitstreamFileIn)
  {
    AnnexBStats stats = AnnexBStats();

    InputNALUnit nalu;
    byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);

    // call actual decoding function
    if (nalu.getBitstream().getFifo().empty())
    {
      /* this can happen if the following occur:
       *  - empty input file
       *  - two back-to-back start_code_prefixes
       *  - start_code_prefix immediately followed by EOF
       */
      msg(WARNING, "Warning: Attempt to decode an empty NAL unit" );
    }
    else
    {
      read(nalu);

      bool writeInpuNalUnitToStream = true;

      // filte temporal layers
      writeInpuNalUnitToStream &= ( m_maxTemporalLayer < 0  ) || ( nalu.m_temporalId <= m_maxTemporalLayer );

      if( nalu.m_nalUnitType == NAL_UNIT_VPS )
      {
        VPS* vps = new VPS();
        m_hlSynaxReader.setBitstream( &nalu.getBitstream() );
        m_hlSynaxReader.parseVPS( vps );
        m_parameterSetManager.storeVPS( vps, nalu.getBitstream().getFifo() );
        xPrintVPSInfo(vps);

        // example: just write the parsed VPS back to the stream
        // *** add modifications here ***
        // only write, if not dropped earlier
        if (writeInpuNalUnitToStream)
        {
          xWriteVPS(vps, bitstreamFileOut, nalu.m_nuhLayerId, nalu.m_temporalId);
          writeInpuNalUnitToStream = false;
        }
      }

      if( nalu.m_nalUnitType == NAL_UNIT_SPS )
      {
        SPS* sps = new SPS();
        m_hlSynaxReader.setBitstream( &nalu.getBitstream() );
        m_hlSynaxReader.parseSPS( sps );
        m_parameterSetManager.storeSPS( sps, nalu.getBitstream().getFifo() );

        msg (VERBOSE, "SPS Info: SPS ID = %d\n", sps->getSPSId());
        // example: just write the parsed SPS back to the stream
        // *** add modifications here ***
        // only write, if not dropped earlier
        if (writeInpuNalUnitToStream)
        {
          xWriteSPS(sps, bitstreamFileOut, nalu.m_nuhLayerId, nalu.m_temporalId);
          writeInpuNalUnitToStream = false;
        }
      }

      if( nalu.m_nalUnitType == NAL_UNIT_PPS )
      {
        PPS* pps = new PPS();
        m_hlSynaxReader.setBitstream( &nalu.getBitstream() );
        m_hlSynaxReader.parsePPS( pps );
        m_parameterSetManager.storePPS( pps, nalu.getBitstream().getFifo() );
        msg (VERBOSE, "PPS Info: PPS ID = %d\n", pps->getPPSId());

        // example: just write the parsed PPS back to the stream
        // *** add modifications here ***
        // only write, if not dropped earlier
        if (writeInpuNalUnitToStream)
        {
          xWritePPS(pps, bitstreamFileOut, nalu.m_nuhLayerId, nalu.m_temporalId);
          writeInpuNalUnitToStream = false;
        }
      }

      unitCnt++;

      if( writeInpuNalUnitToStream )
      {
        int numZeros = stats.m_numLeadingZero8BitsBytes + stats.m_numZeroByteBytes + stats.m_numStartCodePrefixBytes -1;
        // write start code
        char ch = 0;
        for( int i = 0 ; i < numZeros; i++ )
        {
          bitstreamFileOut.write( &ch, 1 );
        }
        ch = 1;
        bitstreamFileOut.write( &ch, 1 );
        // write input NAL unit
        bitstreamFileOut.write( (const char*)nalu.getBitstream().getFifo().data(), nalu.getBitstream().getFifo().size() );
      }
    }
  }

  return 0;
}

