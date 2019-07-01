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

 /** \file     StreamMergeAppCfg.cpp
     \brief    Decoder configuration class
 */

#include <cstdio>
#include <cstring>
#include <string>
#include "StreamMergeAppCfg.h"
#include "Utilities/program_options_lite.h"

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param argc number of arguments
    \param argv array of arguments
 */
bool StreamMergeAppCfg::parseCfg(int argc, char* argv[])
{
#if 1
  int i;

  m_numInputStreams = argc - 2;

  for (i = 0; i < m_numInputStreams; i++)
  {
    m_bitstreamFileNameIn[i] = argv[i + 1];
  }

  m_bitstreamFileNameOut = argv[i + 1];
#else
  bool do_help = false;
  int warnUnknowParameter = 0;
  po::Options opts;
  opts.addOptions()

  ("help", do_help, false, "this help text")
  ("BitstreamFileIn0,-a", m_bitstreamFileNameIn[0], string(""), "bitstream input file name")
  ("BitstreamFileIn1,-b", m_bitstreamFileNameIn[1], string(""), "bitstream input file name")
  ("BitstreamFileOut,o", m_bitstreamFileNameOut, string(""), "bitstream output file name")
  ;

  po::setDefaults(opts);
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled = po::scanArgv(opts, argc, (const char**)argv, err);

  for (list<const char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++)
  {
    std::cerr << "Unhandled argument ignored: " << *it << std::endl;
  }

  if (argc == 1 || do_help)
  {
    po::doHelp(cout, opts);
    return false;
  }

  if (err.is_errored)
  {
    if (!warnUnknowParameter)
    {
      /* errors have already been reported to stderr */
      return false;
    }
  }

  m_numInputStreams = 0;
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    if (!m_bitstreamFileNameIn[i].empty())
      m_numInputStreams++;
  }

  if (m_numInputStreams < 2)
  {
    std::cerr << "Need at least two input bitstreams, aborting" << std::endl;
    return false;
  }

  if (m_bitstreamFileNameOut.empty())
  {
    std::cerr << "No output file specified, aborting" << std::endl;
    return false;
  }
#endif

  return true;
}

StreamMergeAppCfg::StreamMergeAppCfg()
  : m_bitstreamFileNameOut()
  , m_numInputStreams(0)
{
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
    m_bitstreamFileNameIn[i] = "";
}

StreamMergeAppCfg::~StreamMergeAppCfg()
{

}

//! \}
