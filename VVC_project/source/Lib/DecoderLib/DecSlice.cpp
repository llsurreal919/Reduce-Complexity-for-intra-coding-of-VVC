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

/** \file     DecSlice.cpp
    \brief    slice decoder class
*/

#include "DecSlice.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"

#include <vector>

//! \ingroup DecoderLib
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

DecSlice::DecSlice()
{
}

DecSlice::~DecSlice()
{
}

void DecSlice::create()
{
}

void DecSlice::destroy()
{
}

void DecSlice::init( CABACDecoder* cabacDecoder, DecCu* pcCuDecoder )
{
  m_CABACDecoder    = cabacDecoder;
  m_pcCuDecoder     = pcCuDecoder;
}

void DecSlice::decompressSlice( Slice* slice, InputBitstream* bitstream, int debugCTU )
{
  //-- For time output for each slice
  slice->startProcessingTimer();

  const SPS*     sps          = slice->getSPS();
  Picture*       pic          = slice->getPic();
  const BrickMap& tileMap     = *pic->brickMap;
  CABACReader&   cabacReader  = *m_CABACDecoder->getCABACReader( 0 );

  // setup coding structure
  CodingStructure& cs = *pic->cs;
  cs.slice            = slice;
  cs.sps              = sps;
  cs.pps              = slice->getPPS();
  memcpy(cs.alfApss, slice->getAlfAPSs(), sizeof(cs.alfApss));

  cs.lmcsAps = slice->getLmcsAPS();

  cs.pcv              = slice->getPPS()->pcv;
  cs.chromaQpAdj      = 0;

  cs.picture->resizeSAO(cs.pcv->sizeInCtus, 0);

#if JVET_O0119_BASE_PALETTE_444
  cs.resetPrevPLT(cs.prevPLT);
#endif

  if (slice->getSliceCurStartCtuTsAddr() == 0)
  {
    cs.picture->resizeAlfCtuEnableFlag( cs.pcv->sizeInCtus );
    cs.picture->resizeAlfCtbFilterIndex(cs.pcv->sizeInCtus);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
    cs.picture->resizeAlfCtuAlternative( cs.pcv->sizeInCtus );
#endif
  }

  const unsigned numSubstreams = slice->getNumberOfSubstreamSizes() + 1;

  // init each couple {EntropyDecoder, Substream}
  // Table of extracted substreams.
  std::vector<InputBitstream*> ppcSubstreams( numSubstreams );
  for( unsigned idx = 0; idx < numSubstreams; idx++ )
  {
    ppcSubstreams[idx] = bitstream->extractSubstream( idx+1 < numSubstreams ? ( slice->getSubstreamSize(idx) << 3 ) : bitstream->getNumBitsLeft() );
  }

  const int       startCtuTsAddr          = slice->getSliceCurStartCtuTsAddr();

  const unsigned  numCtusInFrame          = cs.pcv->sizeInCtus;
  const unsigned  widthInCtus             = cs.pcv->widthInCtus;
  const bool      wavefrontsEnabled       = cs.pps->getEntropyCodingSyncEnabledFlag();

  cabacReader.initBitstream( ppcSubstreams[0] );
  cabacReader.initCtxModels( *slice );

  // Quantization parameter
    pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
  CHECK( pic->m_prevQP[0] == std::numeric_limits<int>::max(), "Invalid previous QP" );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", slice->getPOC() );


  // for every CTU in the slice segment...
  bool isLastCtuOfSliceSegment = false;
  uint32_t startSliceRsRow = tileMap.getCtuBsToRsAddrMap(startCtuTsAddr) / widthInCtus;
  uint32_t startSliceRsCol = tileMap.getCtuBsToRsAddrMap(startCtuTsAddr) % widthInCtus;
  uint32_t endSliceRsRow = tileMap.getCtuBsToRsAddrMap(slice->getSliceCurEndCtuTsAddr() - 1) / widthInCtus;
  uint32_t endSliceRsCol = tileMap.getCtuBsToRsAddrMap(slice->getSliceCurEndCtuTsAddr() - 1) % widthInCtus;
  unsigned subStrmId = 0;
  for( unsigned ctuTsAddr = startCtuTsAddr; !isLastCtuOfSliceSegment && ctuTsAddr < numCtusInFrame; ctuTsAddr++ )
  {
    const unsigned  ctuRsAddr             = tileMap.getCtuBsToRsAddrMap(ctuTsAddr);
    const Brick&  currentTile             = tileMap.bricks[ tileMap.getBrickIdxRsMap(ctuRsAddr) ];
    if (slice->getPPS()->getRectSliceFlag() &&
      ((ctuRsAddr / widthInCtus) < startSliceRsRow || (ctuRsAddr / widthInCtus) > endSliceRsRow ||
      (ctuRsAddr % widthInCtus) < startSliceRsCol || (ctuRsAddr % widthInCtus) > endSliceRsCol))
      continue;
    const unsigned  firstCtuRsAddrOfTile  = currentTile.getFirstCtuRsAddr();
    const unsigned  tileXPosInCtus        = firstCtuRsAddrOfTile % widthInCtus;
    const unsigned  tileYPosInCtus        = firstCtuRsAddrOfTile / widthInCtus;
    const unsigned  ctuXPosInCtus         = ctuRsAddr % widthInCtus;
    const unsigned  ctuYPosInCtus         = ctuRsAddr / widthInCtus;
    const unsigned  maxCUSize             = sps->getMaxCUWidth();
    Position pos( ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize) ;
    UnitArea ctuArea(cs.area.chromaFormat, Area( pos.x, pos.y, maxCUSize, maxCUSize ) );

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    cabacReader.initBitstream( ppcSubstreams[subStrmId] );

    // set up CABAC contexts' state for this CTU
    if( ctuRsAddr == firstCtuRsAddrOfTile )
    {
      if( ctuTsAddr != startCtuTsAddr ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
#if JVET_O0119_BASE_PALETTE_444
        cs.resetPrevPLT(cs.prevPLT);
#endif
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }
    else if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if( ctuTsAddr != startCtuTsAddr ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
#if JVET_O0119_BASE_PALETTE_444
        cs.resetPrevPLT(cs.prevPLT);
#endif
      }
      if( cs.getCURestricted( pos.offset(0, -1), pos, slice->getIndependentSliceIdx(), tileMap.getBrickIdxRsMap( pos ), CH_L ) )
      {
        // Top-right is available, so use it.
        cabacReader.getCtx() = m_entropyCodingSyncContextState;
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }

    bool updateGbiCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuTsAddr == startCtuTsAddr;
    if(updateGbiCodingOrder)
    {
      resetGbiCodingOrder(true, cs);
    }

    if ((cs.slice->getSliceType() != I_SLICE || cs.sps->getIBCFlag()) && ctuXPosInCtus == tileXPosInCtus)
    {
      cs.motionLut.lut.resize(0);
      cs.motionLut.lutIbc.resize(0);
#if JVET_O1170_CHECK_BV_AT_DECODER
      cs.resetIBCBuffer = true;
#endif
#if !JVET_O0078_SINGLE_HMVPLUT
      cs.motionLut.lutShareIbc.resize(0);
#endif
    }

    if( !cs.slice->isIntra() )
    {
      pic->mctsInfo.init( &cs, getCtuAddr( ctuArea.lumaPos(), *( cs.pcv ) ) );
    }

    if( ctuRsAddr == debugCTU )
    {
      isLastCtuOfSliceSegment = true; // get out here
      break;
    }
    isLastCtuOfSliceSegment = cabacReader.coding_tree_unit( cs, ctuArea, pic->m_prevQP, ctuRsAddr );

    m_pcCuDecoder->decompressCtu( cs, ctuArea );

    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = cabacReader.getCtx();
    }


    if( isLastCtuOfSliceSegment )
    {
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      cabacReader.remaining_bytes( false );
#endif
        slice->setSliceCurEndCtuTsAddr( ctuTsAddr+1 );
    }
    else if( ( ctuXPosInCtus + 1 == tileXPosInCtus + currentTile.getWidthInCtus () ) &&
             ( ctuYPosInCtus + 1 == tileYPosInCtus + currentTile.getHeightInCtus() || wavefrontsEnabled ) )
    {
      // The sub-stream/stream should be terminated after this CTU.
      // (end of slice-segment, end of tile, end of wavefront-CTU-row)
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      cabacReader.remaining_bytes( true );
#endif
      subStrmId++;
    }
  }
  CHECK( !isLastCtuOfSliceSegment, "Last CTU of slice segment not signalled as such" );

  // deallocate all created substreams, including internal buffers.
  for( auto substr: ppcSubstreams )
  {
    delete substr;
  }
  slice->stopProcessingTimer();
}

//! \}
