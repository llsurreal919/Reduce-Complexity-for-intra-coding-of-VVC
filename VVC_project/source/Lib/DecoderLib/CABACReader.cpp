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

/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)           const CodingStatisticsClassType CSCT(x);                       m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)        const CodingStatisticsClassType CSCT(x,y);                     m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)    const CodingStatisticsClassType CSCT(x, s.width, s.height);    m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z) const CodingStatisticsClassType CSCT(x, s.width, s.height, z); m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)                  m_BinDecoder.set( x );
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)
#endif


void CABACReader::initCtxModels( Slice& slice )
{
  SliceType sliceType  = slice.getSliceType();
  int       qp         = slice.getSliceQp();
  if( slice.getPPS()->getCabacInitPresentFlag() && slice.getCabacInitFlag() )
  {
    switch( sliceType )
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      THROW( "Invalid slice type" );
      break;
    }
  }
  m_BinDecoder.reset( qp, (int)sliceType );
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if( m_BinDecoder.decodeBinTrm() )
  {
    m_BinDecoder.finish();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::IncrementStatisticEP( STATS__TRAILING_BITS, m_Bitstream->readOutTrailingBits(), 0 );
#else
    m_Bitstream->readOutTrailingBits();
#endif
    return true;
  }
  return false;
}

void CABACReader::remaining_bytes( bool noTrailingBytesExpected )
{
  if( noTrailingBytesExpected )
  {
    CHECK( 0 != m_Bitstream->getNumBitsLeft(), "Bits left when not supposed" );
  }
  else
  {
    while( m_Bitstream->getNumBitsLeft() )
    {
      unsigned trailingNullByte = m_Bitstream->readByte();
      if( trailingNullByte != 0 )
      {
        THROW( "Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n" );
      }
    }
  }
}

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qpL, qpC, ctuRsAddr )
//================================================================================

bool CABACReader::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr )
{
  CUCtx cuCtx( qps[CH_L] );
  QTBTPartitioner partitioner;

  partitioner.initCtu(area, CH_L, *cs.slice);
#if JVET_O0050_LOCAL_DUAL_TREE
  cs.treeType = partitioner.treeType = TREE_D;
  cs.modeType = partitioner.modeType = MODE_TYPE_ALL;
#endif


  sao( cs, ctuRsAddr );
  if (cs.sps->getALFEnabledFlag() && (cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y)))
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
    const uint32_t          curTileIdx = cs.picture->brickMap->getBrickIdxRsMap( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      if (cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx))
      {
        uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
        int ctx = 0;
        ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
        ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;

        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__ALF);
        ctbAlfFlag[ctuRsAddr] = m_BinDecoder.decodeBin( Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );

        if (isLuma((ComponentID)compIdx) && ctbAlfFlag[ctuRsAddr])
        {
          readAlfCtuFilterIndex(cs, ctuRsAddr);
        }
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
        if( isChroma( (ComponentID)compIdx ) )
        {
          int apsIdx = cs.slice->getTileGroupApsIdChroma();
          CHECK(cs.slice->getAlfAPSs()[apsIdx] == nullptr, "APS not initialized");
          const AlfParam& alfParam = cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();
          const int numAlts = alfParam.numAlternativesChroma;
          uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData( compIdx );
          ctbAlfAlternative[ctuRsAddr] = 0;
          if( ctbAlfFlag[ctuRsAddr] )
          {
            uint8_t decoded = 0;
            while( decoded < numAlts-1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( compIdx-1 ) ) )
              ++ decoded;
            ctbAlfAlternative[ctuRsAddr] = decoded;
          }
        }
#endif
      }
    }
  }

  bool isLast = false;

  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
  {
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, CH_C, *cs.slice);
    CUCtx cuCtxChroma(qps[CH_C]);
    isLast    = coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &cuCtxChroma);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = cuCtxChroma.qp;
  }
  else
  {
    isLast    = coding_tree(cs, partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( !isLast && CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner.initCtu(area, CH_C, *cs.slice);
      isLast    = coding_tree(cs, partitioner, cuCtxChroma);
      qps[CH_C] = cuCtxChroma.qp;
    }
  }

  DTRACE_COND( ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->poc, cs.slice->getSliceQpBase() );
  DTRACE     (                 g_trace_ctx, D_QP_PER_CTU, " %3d",           qps[CH_L] - cs.slice->getSliceQpBase() );

  return isLast;
}

void CABACReader::readAlfCtuFilterIndex(CodingStructure& cs, unsigned ctuRsAddr)
{
  short* alfCtbFilterSetIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  unsigned numAps = cs.slice->getTileGroupNumAps();
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  uint32_t filtIndex = 0;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    int useLatestFilt = m_BinDecoder.decodeBin(Ctx::AlfUseLatestFilt());
    if (useLatestFilt)
    {
      filtIndex = NUM_FIXED_FILTER_SETS;
    }
    else
    {
      if (numAps == 1)
      {
        xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
      }
      else
      {
        unsigned usePrevFilt = m_BinDecoder.decodeBin(Ctx::AlfUseTemporalFilt());
        if (usePrevFilt)
        {
#if JVET_O0247_ALF_CTB_CODING_REDUNDANCY_REMOVAL
          if (numAps > 2)
          {
#endif
            xReadTruncBinCode(filtIndex, numAvailableFiltSets - (NUM_FIXED_FILTER_SETS + 1));
#if JVET_O0247_ALF_CTB_CODING_REDUNDANCY_REMOVAL
          }
#endif
          filtIndex += (unsigned)(NUM_FIXED_FILTER_SETS + 1);
        }
        else
        {
          xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
        }
      }
    }
  }
  else
  {
    xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
  }
  alfCtbFilterSetIndex[ctuRsAddr] = filtIndex;
}
//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao( slice, ctuRsAddr )
//================================================================================

void CABACReader::sao( CodingStructure& cs, unsigned ctuRsAddr )
{
  const SPS&   sps   = *cs.sps;

  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  const Slice& slice                        = *cs.slice;
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool              slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool              slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  sao_ctu_pars[ COMPONENT_Y  ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cb ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cr ].modeIdc      = SAO_MODE_OFF;
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  // merge
  int             frame_width_in_ctus     = cs.pcv->widthInCtus;
  int             ry                      = ctuRsAddr      / frame_width_in_ctus;
  int             rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  int             sao_merge_type          = -1;
  const Position  pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned  curSliceIdx = cs.slice->getIndependentSliceIdx();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SAO );

  const unsigned  curTileIdx  = cs.picture->brickMap->getBrickIdxRsMap( pos );
  if( cs.getCURestricted( pos.offset(-(int)cs.pcv->maxCUWidth, 0), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_left_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) );
  }

  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(int)cs.pcv->maxCUHeight), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_above_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) ) << 1;
  }
  if( sao_merge_type >= 0 )
  {
    if( slice_sao_luma_flag || slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Y  ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Y  ].typeIdc  = sao_merge_type;
    }
    if( slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Cb ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cr ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cb ].typeIdc  = sao_merge_type;
      sao_ctu_pars[ COMPONENT_Cr ].typeIdc  = sao_merge_type;
    }
    return;
  }

  // explicit parameters
  ComponentID firstComp = ( slice_sao_luma_flag   ? COMPONENT_Y  : COMPONENT_Cb );
  ComponentID lastComp  = ( slice_sao_chroma_flag ? COMPONENT_Cr : COMPONENT_Y  );
  for( ComponentID compID = firstComp; compID <= lastComp; compID = ComponentID( compID + 1 ) )
  {
    SAOOffset& sao_pars = sao_ctu_pars[ compID ];

    // sao_type_idx_luma / sao_type_idx_chroma
    if( compID != COMPONENT_Cr )
    {
      if( m_BinDecoder.decodeBin( Ctx::SaoTypeIdx() ) )
      {
        if( m_BinDecoder.decodeBinEP( ) )
        {
          // edge offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_EO;
        }
        else
        {
          // band offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_BO;
        }
      }
    }
    else //Cr, follow Cb SAO type
    {
      sao_pars.modeIdc = sao_ctu_pars[ COMPONENT_Cb ].modeIdc;
      sao_pars.typeIdc = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    if( sao_pars.modeIdc == SAO_MODE_OFF )
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( sps.getBitDepth( toChannelType(compID) ) );
    offset    [0]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [1]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [2]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [3]           = (int)unary_max_eqprob( maxOffsetQVal );

    // band offset mode
    if( sao_pars.typeIdc == SAO_TYPE_START_BO )
    {
      // sao_offset_sign
      for( int k = 0; k < 4; k++ )
      {
        if( offset[k] && m_BinDecoder.decodeBinEP( ) )
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      sao_pars.typeAuxInfo = m_BinDecoder.decodeBinsEP( NUM_SAO_BO_CLASSES_LOG2 );
      for( int k = 0; k < 4; k++ )
      {
        sao_pars.offset[ ( sao_pars.typeAuxInfo + k ) % MAX_NUM_SAO_CLASSES ] = offset[k];
      }
      continue;
    }

    // edge offset mode
    sao_pars.typeAuxInfo = 0;
    if( compID != COMPONENT_Cr )
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      sao_pars.typeIdc += m_BinDecoder.decodeBinsEP( NUM_SAO_EO_TYPES_LOG2 );
    }
    else
    {
      sao_pars.typeIdc  = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    sao_pars.offset[ SAO_CLASS_EO_FULL_VALLEY ] =  offset[0];
    sao_pars.offset[ SAO_CLASS_EO_HALF_VALLEY ] =  offset[1];
    sao_pars.offset[ SAO_CLASS_EO_PLAIN       ] =  0;
    sao_pars.offset[ SAO_CLASS_EO_HALF_PEAK   ] = -offset[2];
    sao_pars.offset[ SAO_CLASS_EO_FULL_PEAK   ] = -offset[3];
  }
}

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    bool  coding_tree       ( cs, partitioner, cuCtx )
//    bool  split_cu_flag     ( cs, partitioner )
//    split split_cu_mode_mt  ( cs, partitioner )
//================================================================================

bool CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  bool           lastSegment  = false;

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
#if JVET_O0050_LOCAL_DUAL_TREE
  //Note: do not reset qg at chroma CU
  if( pps.getUseDQP() && partitioner.currQgEnable() && !isChroma(partitioner.chType) )
#else
  if( pps.getUseDQP() && partitioner.currQgEnable() )
#endif
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
  }

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.getUseDQP() && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->getUseChromaQpAdj() && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
    }
  }
  int startShareThisLevel = 0;

  const PartSplit splitMode = split_cu_mode( cs, partitioner );

  CHECK( !partitioner.canSplit( splitMode, cs ), "Got an invalid split!" );

  if( splitMode != CU_DONT_SPLIT )
  {
      const PartSplit split = splitMode;
      int splitRatio = 1;
      CHECK(!(split == CU_QUAD_SPLIT || split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT
        || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT), "invalid split type");
      splitRatio = (split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT) ? 1 : 2;

      bool isOneChildSmall = (((partitioner.currArea().lwidth())*(partitioner.currArea().lheight())) >> splitRatio) < MRG_SHARELIST_SHARSIZE;

      if ((((partitioner.currArea().lwidth())*(partitioner.currArea().lheight())) > (MRG_SHARELIST_SHARSIZE * 1)))
      {
        shareStateDec = NO_SHARE;
      }

      if (shareStateDec == NO_SHARE)//init state
      {
        if (isOneChildSmall)
        {
          shareStateDec = SHARING;//share start state
          startShareThisLevel = 1;

          shareParentPos = partitioner.currArea().lumaPos();
          shareParentSize.width = partitioner.currArea().lwidth();
          shareParentSize.height = partitioner.currArea().lheight();
        }
      }
      if (CS::isDualITree(cs) && pPartitionerChroma != nullptr && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
      {
        partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
        pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
        bool beContinue = true;
        bool lumaContinue = true;
        bool chromaContinue = true;
        bool lastSegmentC = false;

        while (beContinue)
        {
          if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
          {
            if (!lastSegmentC && cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              lastSegmentC = coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
            }
            lumaContinue = partitioner.nextPart(cs);
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
          else
          {
            //dual tree coding under 64x64 block
            if (!lastSegment && cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              lastSegment = coding_tree(cs, partitioner, cuCtx);
            }
            lumaContinue = partitioner.nextPart(cs);
            if (!lastSegmentC && cs.area.blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
            {
              lastSegmentC = coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
            }
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            CHECK(lastSegment == true, "luma should not be the last segment");
            beContinue = lumaContinue;
          }
        }
        partitioner.exitCurrSplit();
        pPartitionerChroma->exitCurrSplit();

        //cat the chroma CUs together
        CodingUnit* currentCu = cs.getCU(partitioner.currArea().lumaPos(), CHANNEL_TYPE_LUMA);
        CodingUnit* nextCu = nullptr;
        CodingUnit* tempLastLumaCu = nullptr;
        CodingUnit* tempLastChromaCu = nullptr;
        ChannelType currentChType = currentCu->chType;
        while (currentCu->next != nullptr)
        {
          nextCu = currentCu->next;
          if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_LUMA)
          {
            tempLastLumaCu = currentCu;
            if (tempLastChromaCu != nullptr) //swap
            {
              tempLastChromaCu->next = nextCu;
            }
          }
          else if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_CHROMA)
          {
            tempLastChromaCu = currentCu;
            if (tempLastLumaCu != nullptr) //swap
            {
              tempLastLumaCu->next = nextCu;
            }
          }
          currentCu = nextCu;
          currentChType = currentCu->chType;
        }

        CodingUnit* chromaFirstCu = cs.getCU(pPartitionerChroma->currArea().chromaPos(), CHANNEL_TYPE_CHROMA);
        tempLastLumaCu->next = chromaFirstCu;

        lastSegment = lastSegmentC;
      }
      else
      {
#if JVET_O0050_LOCAL_DUAL_TREE
        const ModeType modeTypeParent = partitioner.modeType;
        cs.modeType = partitioner.modeType = mode_constraint( cs, partitioner, splitMode ); //change for child nodes
        //decide chroma split or not
        bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && partitioner.modeType == MODE_TYPE_INTRA;
        CHECK( chromaNotSplit && partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma" );
        if( partitioner.treeType == TREE_D )
        {
          cs.treeType = partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
        }
#endif
      partitioner.splitCurrArea( splitMode, cs );
      do
      {
        if( !lastSegment && cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          lastSegment = coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
#if JVET_O0050_LOCAL_DUAL_TREE
      if( chromaNotSplit )
      {
        CHECK( partitioner.chType != CHANNEL_TYPE_LUMA, "must be luma status" );
        partitioner.chType = CHANNEL_TYPE_CHROMA;
        cs.treeType = partitioner.treeType = TREE_C;

        if( !lastSegment && cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          lastSegment = coding_tree( cs, partitioner, cuCtx );
        }

        //recover treeType
        partitioner.chType = CHANNEL_TYPE_LUMA;
        cs.treeType = partitioner.treeType = TREE_D;
      }

      //recover ModeType
      cs.modeType = partitioner.modeType = modeTypeParent;
#endif
      }
      if (startShareThisLevel == 1)
        shareStateDec = NO_SHARE;
      return lastSegment;
  }

  CodingUnit& cu = cs.addCU( CS::getArea( cs, currArea, partitioner.chType ), partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice   = cs.slice;
  cu.tileIdx = cs.picture->brickMap->getBrickIdxRsMap( currArea.lumaPos() );
#if JVET_O0050_LOCAL_DUAL_TREE
  CHECK( cu.cs->treeType != partitioner.treeType, "treeType mismatch" );
  int lumaQPinLocalDualTree = -1;
#endif

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }

#if JVET_O0050_LOCAL_DUAL_TREE
  if (pps.getUseDQP() && partitioner.isSepTree(cs) && isChroma(cu.chType))
#else
  if (pps.getUseDQP() && CS::isDualITree(cs) && isChroma(cu.chType))
#endif
  {
    const Position chromaCentral(cu.chromaPos().offset(cu.chromaSize().width >> 1, cu.chromaSize().height >> 1));
    const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, cu.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, cu.chromaFormat));
#if JVET_O0050_LOCAL_DUAL_TREE
    //derive chroma qp, but the chroma qp is saved in cuCtx.qp which is used for luma qp
    //therefore, after decoding the chroma CU, the cuCtx.qp shall be recovered to luma qp in order to decode next luma cu qp
    const CodingUnit* colLumaCu = cs.getLumaCU( lumaRefPos );
    CHECK( colLumaCu == nullptr, "colLumaCU shall exist" );
    lumaQPinLocalDualTree = cuCtx.qp;
#else
    const CodingUnit* colLumaCu = cs.getCU(lumaRefPos, CHANNEL_TYPE_LUMA);
#endif

    if (colLumaCu) cuCtx.qp = colLumaCu->qp;
  }

  cu.qp = cuCtx.qp;                 //NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;  //NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level

  // coding unit
    cu.shareParentPos = (shareStateDec == SHARING) ? shareParentPos : partitioner.currArea().lumaPos();
    cu.shareParentSize = (shareStateDec == SHARING) ? shareParentSize : partitioner.currArea().lumaSize();

  bool isLastCtu = coding_unit( cu, partitioner, cuCtx );
#if JVET_O0050_LOCAL_DUAL_TREE
  //recover cuCtx.qp to luma qp after decoding the chroma CU
  if( pps.getUseDQP() && partitioner.isSepTree( cs ) && isChroma( cu.chType ) )
  {
    cuCtx.qp = lumaQPinLocalDualTree;
  }
#endif

#if JVET_O0119_BASE_PALETTE_444
  uint32_t compBegin;
  uint32_t numComp;
  bool jointPLT = false;
#if JVET_O0050_LOCAL_DUAL_TREE
  if (cu.isSepTree())
#else
  if (CS::isDualITree(*cu.cs))
#endif
  {
    if (isLuma(partitioner.chType))
    {
      compBegin = COMPONENT_Y;
      numComp = 1;
    }
    else
    {
      compBegin = COMPONENT_Cb;
      numComp = 2;
    }
  }
  else
  {
    compBegin = COMPONENT_Y;
    numComp = 3;
    jointPLT = true;
  }
  if (CU::isPLT(cu))
  {
    cs.reorderPrevPLT(cs.prevPLT, cu.curPLTSize, cu.curPLT, cu.reuseflag, compBegin, numComp, jointPLT);
  }
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
  if( cu.chType == CHANNEL_TYPE_CHROMA )
  {
    DTRACE( g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
#endif
  DTRACE( g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
#if JVET_O0050_LOCAL_DUAL_TREE
  }
#endif
  if (startShareThisLevel == 1)
    shareStateDec = NO_SHARE;
  return isLastCtu;
}

#if JVET_O0050_LOCAL_DUAL_TREE
ModeType CABACReader::mode_constraint( CodingStructure& cs, Partitioner &partitioner, PartSplit splitMode )
{
  int val = cs.signalModeCons( splitMode, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    int ctxIdx = DeriveCtx::CtxModeConsFlag( cs, partitioner );
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MODE_CONSTRAINT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );
    bool flag = m_BinDecoder.decodeBin( Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
    return flag ? MODE_TYPE_INTRA : MODE_TYPE_INTER;
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    return MODE_TYPE_INTRA;
  }
  else
  {
    return partitioner.modeType;
  }
}
#endif

PartSplit CABACReader::split_cu_mode( CodingStructure& cs, Partitioner &partitioner )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SPLIT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );

  PartSplit mode = CU_DONT_SPLIT;

  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  bool isSplit = canBh || canBv || canTh || canTv || canQt;

  if( canNo && isSplit )
  {
    isSplit = m_BinDecoder.decodeBin( Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, isSplit );

  if( !isSplit )
  {
    return CU_DONT_SPLIT;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  bool       isQt   = canQt;

  if( isQt && canBtt )
  {
    isQt = m_BinDecoder.decodeBin( Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return CU_QUAD_SPLIT;
  }

  const bool canHor = canBh || canTh;
  bool        isVer = canBv || canTv;

  if( isVer && canHor )
  {
    isVer = m_BinDecoder.decodeBin( Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  bool        is12 = isVer ? canBv : canBh;

  if( is12 && can14 )
  {
    is12 = m_BinDecoder.decodeBin( Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  if     ( isVer && is12 )  mode = CU_VERT_SPLIT;
  else if( isVer && !is12 ) mode = CU_TRIV_SPLIT;
  else if( !isVer && is12 ) mode = CU_HORZ_SPLIT;
  else                      mode = CU_TRIH_SPLIT;

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, mode );

  return mode;
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    bool  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_transquant_bypass_flag ( cu )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
#if !JVET_O0525_REMOVE_PCM
//    void  pcm_flag                  ( cu )
//    void  pcm_samples               ( tu )
#endif
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    bool  end_of_ctu                ( cu, cuCtx )
//================================================================================

bool CABACReader::coding_unit( CodingUnit &cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;
#if JVET_O0050_LOCAL_DUAL_TREE
  CHECK( cu.treeType != partitioner.treeType || cu.modeType != partitioner.modeType, "treeType or modeType mismatch" );
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
#endif
  // transquant bypass flag
  if( cs.pps->getTransquantBypassEnabledFlag() )
  {
    cu_transquant_bypass_flag( cu );
  }
  PredictionUnit&    pu = cs.addPU(cu, partitioner.chType);
  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
  {
    cu_skip_flag( cu );
  }

  // skip data
  if( cu.skip )
  {
    cs.addTU         ( cu, partitioner.chType );
    pu.shareParentPos = cu.shareParentPos;
    pu.shareParentSize = cu.shareParentSize;
    MergeCtx           mrgCtx;
    prediction_unit  ( pu, mrgCtx );
    return end_of_ctu( cu, cuCtx );
  }

  // prediction mode and partitioning data
  pred_mode ( cu );
#if JVET_O0119_BASE_PALETTE_444
  if (CU::isPLT(cu))
  {
    cs.addTU(cu, partitioner.chType);
#if JVET_O0050_LOCAL_DUAL_TREE
    if (cu.isSepTree())
#else
    if (CS::isDualITree(*cu.cs))
#endif
    {
      if (isLuma(partitioner.chType))
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
      if (cu.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
      {
        cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
      }
    }
    else
    {
      cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
    }
    return end_of_ctu(cu, cuCtx);
  }
#endif
  bdpcm_mode( cu, ComponentID( partitioner.chType ) );

  // --> create PUs
#if !JVET_O0525_REMOVE_PCM
  // pcm samples
  if( CU::isIntra(cu) )
  {
    pcm_flag( cu, partitioner );
    if( cu.ipcm )
    {
      TransformUnit& tu = cs.addTU( cu, partitioner.chType );
      pcm_samples( tu );
      return end_of_ctu( cu, cuCtx );
    }
  }

#endif

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // check end of cu
  return end_of_ctu( cu, cuCtx );
}


void CABACReader::cu_transquant_bypass_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__TQ_BYPASS_FLAG );

  cu.transQuantBypass = ( m_BinDecoder.decodeBin( Ctx::TransquantBypassFlag() ) );
}


void CABACReader::cu_skip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SKIP_FLAG );

#if JVET_O0050_LOCAL_DUAL_TREE
  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getSPS()->getIBCFlag())
#else
  if (cu.slice->isIntra() && cu.cs->slice->getSPS()->getIBCFlag())
#endif
  {
    cu.skip = false;
    cu.rootCbf = false;
    cu.predMode = MODE_INTRA;
    cu.mmvdSkip = false;
#if JVET_O1161_IBC_MAX_SIZE
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
    if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
    {
    unsigned ctxId = DeriveCtx::CtxSkipFlag(cu);
    unsigned skip = m_BinDecoder.decodeBin(Ctx::SkipFlag(ctxId));
    DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );
    if (skip)
    {
      cu.skip = true;
      cu.rootCbf = false;
      cu.predMode = MODE_IBC;
      cu.mmvdSkip = false;
    }
    }
    return;
  }
  if ( !cu.cs->slice->getSPS()->getIBCFlag() && cu.lwidth() == 4 && cu.lheight() == 4 )
  {
    return;
  }
#if JVET_O0050_LOCAL_DUAL_TREE
  if( !cu.cs->slice->getSPS()->getIBCFlag() && cu.isConsIntra() )
  {
    return;
  }
#endif
  unsigned ctxId  = DeriveCtx::CtxSkipFlag(cu);
  unsigned skip   = m_BinDecoder.decodeBin( Ctx::SkipFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

  if (skip && cu.cs->slice->getSPS()->getIBCFlag())
  {
#if JVET_O1161_IBC_MAX_SIZE
#if JVET_O0050_LOCAL_DUAL_TREE
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !cu.isConsInter()) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#else
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#endif
#else
#if JVET_O0050_LOCAL_DUAL_TREE
    if ((cu.lwidth() < 128 || cu.lheight() < 128) && !cu.isConsInter()) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#else
    if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
#endif
    {
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
        return;
      }
    unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
    if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
    {
      cu.skip = true;
      cu.rootCbf = false;
      cu.predMode = MODE_IBC;
      cu.mmvdSkip = false;
      cu.firstPU->regularMergeFlag = false;
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
  }
  if ((skip && CU::isInter(cu) && cu.cs->slice->getSPS()->getIBCFlag()) ||
    (skip && !cu.cs->slice->getSPS()->getIBCFlag()))
  {
#if !JVET_O0249_MERGE_SYNTAX
    if (!cu.cs->slice->getSPS()->getUseMMVD() && (cu.firstPU->lwidth() * cu.firstPU->lheight() == 32))
    {
      cu.firstPU->regularMergeFlag = true;
    }
    else
    {
      unsigned regularMergeFlag = (m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(0)));
      DTRACE(g_trace_ctx, D_SYNTAX, "regular_merge_flag() ctx=%d regularMergeFlag=%d\n", 0, regularMergeFlag?1:0);
      cu.firstPU->regularMergeFlag = regularMergeFlag;
    }
    if (cu.firstPU->regularMergeFlag)
    {
      cu.mmvdSkip = false;
      cu.firstPU->mmvdMergeFlag = false;
      cu.firstPU->mhIntraFlag = false;
      cu.affine = false;
      cu.triangle = false;
    }
    else
    {
      if (cu.cs->slice->getSPS()->getUseMMVD())
      {
        bool isCUWithOnlyRegularAndMMVD=((cu.firstPU->lwidth() == 8 && cu.firstPU->lheight() == 4) || (cu.firstPU->lwidth() == 4 && cu.firstPU->lheight() == 8));
        if (isCUWithOnlyRegularAndMMVD)
        {
          cu.mmvdSkip = !(cu.firstPU->regularMergeFlag);
        }
        else
        {
          unsigned mmvdSkip = m_BinDecoder.decodeBin(Ctx::MmvdFlag(0));
          cu.mmvdSkip = mmvdSkip;
          DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, mmvdSkip ? 1 : 0);
        }
      }
      else
      {
        cu.mmvdSkip = false;
      }
    }
#endif
    cu.skip     = true;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTER;
  }
}

void CABACReader::imv_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.cs->sps->getAMVREnabledFlag() )
  {
    return;
  }

  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  if ( cu.affine )
  {
    return;
  }

  const SPS *sps = cu.cs->sps;

  unsigned value = 0;
  if (CU::isIBC(cu))
    value = 1;
  else
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 0 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 0 );

#if JVET_O0057_ALTHPELIF
    cu.imv = value;
#endif
  if( sps->getAMVREnabledFlag() && value )
  {
#if JVET_O0057_ALTHPELIF
    if (!CU::isIBC(cu))
    {
      value = m_BinDecoder.decodeBin(Ctx::ImvFlag(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 4);
      cu.imv = value ? 1 : IMV_HPEL;
    }
    if (value)
    {
#endif
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 1 );
    value++;
#if JVET_O0057_ALTHPELIF
      cu.imv = value;
    }
#endif
  }

#if !JVET_O0057_ALTHPELIF
  cu.imv = value;
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::affine_amvr_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  unsigned value = 0;
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 2 );

  if( value )
  {
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 3 );
    value++;
  }

  cu.imv = value;
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::pred_mode( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__PRED_MODE );
#if JVET_O0258_REMOVE_CHROMA_IBC_FOR_DUALTREE
  if (cu.cs->slice->getSPS()->getIBCFlag() && cu.chType != CHANNEL_TYPE_CHROMA)
#else
  if (cu.cs->slice->getSPS()->getIBCFlag())
#endif
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    if( cu.isConsInter() )
    {
      cu.predMode = MODE_INTER;
      return;
    }
#endif

#if JVET_O0050_LOCAL_DUAL_TREE
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
#else
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) )
#endif
    {
      cu.predMode = MODE_INTRA;
#if JVET_O1161_IBC_MAX_SIZE
      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
      if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
      {
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
      {
        cu.predMode = MODE_IBC;
      }
      }
#if JVET_O0119_BASE_PALETTE_444
      if (!CU::isIBC(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
#endif
    }
    else
    {
      if (m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))))
      {
        cu.predMode = MODE_INTRA;
#if JVET_O0119_BASE_PALETTE_444
        if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
        {
          if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
          {
            cu.predMode = MODE_PLT;
          }
        }
#endif
      }
      else
      {
        cu.predMode = MODE_INTER;
#if JVET_O1161_IBC_MAX_SIZE
        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
        if (cu.lwidth() < 128 || cu.lheight() < 128) // disable 128x128 IBC mode
#endif
        {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
        {
          cu.predMode = MODE_IBC;
        }
        }
      }
    }
  }
  else
  {
#if JVET_O0119_BASE_PALETTE_444
#if JVET_O0050_LOCAL_DUAL_TREE
    if( cu.isConsInter() )
    {
      cu.predMode = MODE_INTER;
      return;
    }
#endif

#if JVET_O0050_LOCAL_DUAL_TREE
    if ( cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4) || cu.isConsIntra() )
#else
    if ( cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4) )
#endif
    {
      cu.predMode = MODE_INTRA;
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      cu.predMode = m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))) ? MODE_INTRA : MODE_INTER;
      if (!CU::isIntra(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64)
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
#else
#if JVET_O0050_LOCAL_DUAL_TREE
    if( cu.isConsIntra() || cu.isConsInter() )
    {
      cu.predMode = cu.isConsIntra() ? MODE_INTRA : MODE_INTER;
      return;
    }
#endif
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || m_BinDecoder.decodeBin( Ctx::PredMode( DeriveCtx::CtxPredModeFlag( cu ) ) ) )
    {
      cu.predMode = MODE_INTRA;
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
#endif
  }
}
void CABACReader::bdpcm_mode( CodingUnit& cu, const ComponentID compID )
{
  cu.bdpcmMode = 0;

#if JVET_O1136_TS_BDPCM_SIGNALLING
  if( !cu.cs->sps->getBDPCMEnabledFlag() ) return;
#endif
  if( !CU::bdpcmAllowed( cu, compID ) ) return;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__BDPCM_MODE, cu.block(compID).lumaSize(), compID );

  cu.bdpcmMode = m_BinDecoder.decodeBin( Ctx::BDPCMMode( 0 ) );

  if( cu.bdpcmMode )
  {
    cu.bdpcmMode += m_BinDecoder.decodeBin( Ctx::BDPCMMode( 1 ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "bdpcm_mode() x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode );
}
#if !JVET_O0525_REMOVE_PCM
void CABACReader::pcm_flag( CodingUnit& cu, Partitioner &partitioner )
{
  const SPS& sps = *cu.cs->sps;
  if( !sps.getPCMEnabledFlag() || partitioner.currArea().lwidth() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lwidth() < (1 << sps.getPCMLog2MinSize())
      || partitioner.currArea().lheight() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lheight() < (1 << sps.getPCMLog2MinSize()) )
  {
    cu.ipcm = false;
    return;
  }
  cu.ipcm = ( m_BinDecoder.decodeBinTrm() );
}

#endif

void CABACReader::cu_pred_data( CodingUnit &cu )
{
  if( CU::isIntra( cu ) )
  {
    intra_luma_pred_modes( cu );
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    cu.predMode = MODE_IBC;
    return;
  }
  MergeCtx mrgCtx;

  for( auto &pu : CU::traversePUs( cu ) )
  {
    pu.shareParentPos = cu.shareParentPos;
    pu.shareParentSize = cu.shareParentSize;
    prediction_unit( pu, mrgCtx );
  }

  imv_mode   ( cu, mrgCtx );
  affine_amvr_mode( cu, mrgCtx );
  cu_gbi_flag( cu );

}

void CABACReader::cu_gbi_flag(CodingUnit& cu)
{
  if(!CU::isGBiIdxCoded(cu))
  {
    return;
  }

  CHECK(!(GBI_NUM > 1 && (GBI_NUM == 2 || (GBI_NUM & 0x01) == 1)), " !( GBI_NUM > 1 && ( GBI_NUM == 2 || ( GBI_NUM & 0x01 ) == 1 ) ) ");

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__GBI_IDX);

  uint32_t idx = 0;

  uint32_t symbol = m_BinDecoder.decodeBin(Ctx::GBiIdx(0));

  int32_t numGBi = (cu.slice->getCheckLDC()) ? 5 : 3;
#if JVET_O0126_BPWA_INDEX_CODING_FIX
  if(symbol == 1)
#else
  if(symbol == 0)
#endif
  {
    uint32_t prefixNumBits = numGBi - 2;
    uint32_t step = 1;

    idx = 1;

    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      symbol = m_BinDecoder.decodeBinEP();
#if JVET_O0126_BPWA_INDEX_CODING_FIX
      if (symbol == 0)
#else
      if (symbol == 1)
#endif
      {
        break;
      }
      idx += step;
    }
  }

  uint8_t gbiIdx = (uint8_t)g_GbiParsingOrder[idx];
  CU::setGbiIdx(cu, gbiIdx);

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_gbi_flag() gbi_idx=%d\n", cu.GBiIdx ? 1 : 0);
}

void CABACReader::xReadTruncBinCode(uint32_t& symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  int b = maxSymbol - val;
  symbol = m_BinDecoder.decodeBinsEP(thresh);
  if (symbol >= val - b)
  {
    uint32_t altSymbol;
    altSymbol = m_BinDecoder.decodeBinEP();
    symbol <<= 1;
    symbol += altSymbol;
    symbol -= (val - b);
  }
}

void CABACReader::extend_ref_line(CodingUnit& cu)
{
#if !ENABLE_JVET_L0283_MRL
  return;
#endif
#if !JVET_O0525_REMOVE_PCM
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.ipcm || cu.bdpcmMode )
#else
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode )
#endif
  {
    cu.firstPU->multiRefIdx = 0;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MULTI_REF_LINE);

  const int numBlocks = CU::getNumPUs(cu);
  PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
    if (isFirstLineOfCtu)
    {
      pu->multiRefIdx = 0;
      continue;
    }
    int multiRefIdx = 0;

    if (MRL_NUM_REF_LINES > 1)
    {
      multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
        multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
      }

    }
    pu->multiRefIdx = multiRefIdx;
    pu = pu->next;
  }
}

void CABACReader::intra_luma_pred_modes( CodingUnit &cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  if( cu.bdpcmMode )
  {
#if JVET_O0315_RDPCM_INTRAMODE_ALIGN
    cu.firstPU->intraDir[0] = cu.bdpcmMode == 2? VER_IDX : HOR_IDX;
#else
    PredictionUnit *pu = cu.firstPU;
    unsigned mpm_pred[NUM_MOST_PROBABLE_MODES];
    PU::getIntraMPMs(*pu, mpm_pred);
    cu.firstPU->intraDir[0] = mpm_pred[0];
#endif
    return;
  }

  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
  extend_ref_line( cu );
  isp_mode( cu );

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, cu.lumaSize(), CHANNEL_TYPE_LUMA );

  // prev_intra_luma_pred_flag
  int numBlocks = CU::getNumPUs( cu );
  int mpmFlag[4];
  for( int k = 0; k < numBlocks; k++ )
  {
    CHECK(numBlocks != 1, "not supported yet");
#if JVET_O0502_ISP_CLEANUP
    if ( cu.firstPU->multiRefIdx )
#else
    if( cu.firstPU->multiRefIdx || ( cu.ispMode && isLuma( cu.chType ) ) )
#endif
    {
      mpmFlag[0] = true;
    }
    else
    {
      mpmFlag[k] = m_BinDecoder.decodeBin(Ctx::IntraLumaMpmFlag());
    }
  }

  PredictionUnit *pu = cu.firstPU;

  unsigned mpm_pred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    PU::getIntraMPMs( *pu, mpm_pred );

    if( mpmFlag[k] )
    {
      uint32_t ipred_idx = 0;
      {
        unsigned ctx = (pu->cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
        if (pu->multiRefIdx == 0)
          ipred_idx = m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(ctx));
        else
          ipred_idx = 1;
        if( ipred_idx )
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 1)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 2)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 3)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
      }
      pu->intraDir[0] = mpm_pred[ipred_idx];
    }
    else
    {
      unsigned ipred_mode = 0;

      {
        xReadTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);
      }
      //postponed sorting of MPMs (only in remaining branch)
      std::sort( mpm_pred, mpm_pred + NUM_MOST_PROBABLE_MODES );

      for( uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
      {
        ipred_mode += (ipred_mode >= mpm_pred[i]);
      }

      pu->intraDir[0] = ipred_mode;
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}

void CABACReader::intra_chroma_pred_modes( CodingUnit& cu )
{
#if JVET_O0050_LOCAL_DUAL_TREE
  if( cu.chromaFormat == CHROMA_400 || ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_LUMA ) )
#else
  if( cu.chromaFormat == CHROMA_400 || ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_LUMA ) )
#endif
  {
    return;
  }

  PredictionUnit *pu = cu.firstPU;

  {
    CHECK( pu->cu != &cu, "Inkonsistent PU-CU mapping" );
    intra_chroma_pred_mode( *pu );
  }
}
#if JVET_O1153_INTRA_CHROMAMODE_CODING
bool CABACReader::intra_chroma_lmc_mode(PredictionUnit& pu)
{
  int lmModeList[10];
  PU::getLMSymbolList(pu, lmModeList);

  int symbol = m_BinDecoder.decodeBin(Ctx::IntraChromaPredMode(0));

  if (symbol == 0)
  {
    pu.intraDir[1] = lmModeList[symbol];
    CHECK(pu.intraDir[1] != LM_CHROMA_IDX, "should be LM_CHROMA");
  }
  else
  {
    symbol += m_BinDecoder.decodeBinEP();
    pu.intraDir[1] = lmModeList[symbol];
  }
  return true; //it will only enter this function for LMC modes, so always return true ;
}

void CABACReader::intra_chroma_pred_mode(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIR_ANG, pu.cu->blocks[pu.chType].lumaSize(), CHANNEL_TYPE_CHROMA);
  // LM chroma mode
#if JVET_O1124_ALLOW_CCLM_COND
  if (pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed())
#else
  if (pu.cs->sps->getUseLMChroma())
#endif
  {
    bool isLMCMode = m_BinDecoder.decodeBin(Ctx::CclmModeFlag(0)) ? true : false;
    if (isLMCMode)
    {
      intra_chroma_lmc_mode(pu);
      return;
    }
  }

  if (m_BinDecoder.decodeBin(Ctx::IntraChromaPredMode(0)) == 0)
  {
    pu.intraDir[1] = DM_CHROMA_IDX;
    return;
  }

  unsigned candId = m_BinDecoder.decodeBinsEP(2);

  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(pu, chromaCandModes);

  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(PU::isLMCMode(chromaCandModes[candId]), "The intra dir cannot be LM_CHROMA for this path");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");

  pu.intraDir[1] = chromaCandModes[candId];
}
#else
bool CABACReader::intra_chroma_lmc_mode( PredictionUnit& pu )
{
  int lmModeList[10];
  int maxSymbol = PU::getLMSymbolList(pu, lmModeList);
  int symbol    = unary_max_symbol(Ctx::IntraChromaPredMode(1), Ctx::IntraChromaPredMode(2), maxSymbol - 1);
  if (lmModeList[symbol] != -1)
  {
    pu.intraDir[1] = lmModeList[symbol];
    return true;
  }
  return false;
}

void CABACReader::intra_chroma_pred_mode( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, pu.cu->blocks[pu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  if (m_BinDecoder.decodeBin(Ctx::IntraChromaPredMode(0)) == 0)
  {
    pu.intraDir[1] = DM_CHROMA_IDX;
    return;
  }

  // LM chroma mode
#if JVET_O1124_ALLOW_CCLM_COND
  if( pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed() )
#else
  if( pu.cs->sps->getUseLMChroma() )
#endif
  {
    if( intra_chroma_lmc_mode( pu ) )
    {
      return;
    }
  }
  unsigned candId = m_BinDecoder.decodeBinsEP( 2 );

  unsigned chromaCandModes[ NUM_CHROMA_MODE ];
  PU::getIntraChromaCandModes( pu, chromaCandModes );

  CHECK( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  CHECK( PU::isLMCMode( chromaCandModes[ candId ] ), "The intra dir cannot be LM_CHROMA for this path" );
  CHECK( chromaCandModes[ candId ] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );

  pu.intraDir[1] = chromaCandModes[ candId ];
}
#endif
void CABACReader::cu_residual( CodingUnit& cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    PredictionUnit& pu = *cu.firstPU;
    if( !pu.mergeFlag )
    {
      rqt_root_cbf( cu );
    }
    else
    {
      cu.rootCbf = true;
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }
    if( !cu.rootCbf )
    {
      TransformUnit& tu = cu.cs->addTU(cu, partitioner.chType);
      tu.depth = 0;
      for( unsigned c = 0; c < tu.blocks.size(); c++ )
      {
        tu.cbf[c]             = 0;
        ComponentID   compID  = ComponentID(c);
        tu.getCoeffs( compID ).fill( 0 );
        tu.getPcmbuf( compID ).fill( 0 );
      }
      return;
    }
  }
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA]   = false;
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
#endif
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  cuCtx.lfnstLastScanPos = false;
#endif

  ChromaCbfs chromaCbfs;
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    transform_tree( *cu.cs, subTuPartitioner, cuCtx, CU::getISPType(cu, getFirstComponentOfChannel(partitioner.chType)), 0 );
#else
    transform_tree( *cu.cs, subTuPartitioner, cuCtx, chromaCbfs, CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType ) ), 0 );
#endif
  }
  else
  {
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    transform_tree( *cu.cs, partitioner, cuCtx             );
#else
    transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
#endif
  }
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  residual_lfnst_mode( cu, cuCtx );
#else
  residual_lfnst_mode( cu );
#endif
}

void CABACReader::rqt_root_cbf( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__QT_ROOT_CBF );

  cu.rootCbf = ( m_BinDecoder.decodeBin( Ctx::QtRootCbf() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACReader::sbt_mode( CodingUnit& cu )
{
  const uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SBT_MODE );
  //bin - flag
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  bool sbtFlag = m_BinDecoder.decodeBin( Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );

  //bin - type
  bool sbtQuadFlag = false;
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    sbtQuadFlag = m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    sbtQuadFlag = 0;
  }

  //bin - dir
  bool sbtHorFlag = false;
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    sbtHorFlag = m_BinDecoder.decodeBin( Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    sbtHorFlag = ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow );
  }
  cu.setSbtIdx( sbtHorFlag ? ( sbtQuadFlag ? SBT_HOR_QUAD : SBT_HOR_HALF ) : ( sbtQuadFlag ? SBT_VER_QUAD : SBT_VER_HALF ) );

  //bin - pos
  bool sbtPosFlag = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 0 ) );
  cu.setSbtPos( sbtPosFlag ? SBT_POS1 : SBT_POS0 );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}


bool CABACReader::end_of_ctu( CodingUnit& cu, CUCtx& cuCtx )
{
#if !JVET_O1164_PS
  const SPS     &sps   = *cu.cs->sps;
#endif
  const Position rbPos = recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].bottomRight().offset( 1, 1 ) );

#if JVET_O1164_PS
  if( ( ( rbPos.x & cu.cs->pcv->maxCUWidthMask ) == 0 || rbPos.x == cu.cs->pps->getPicWidthInLumaSamples() )
  && ( ( rbPos.y & cu.cs->pcv->maxCUHeightMask ) == 0 || rbPos.y == cu.cs->pps->getPicHeightInLumaSamples() )
#else
  if ( ( ( rbPos.x & cu.cs->pcv->maxCUWidthMask  ) == 0 || rbPos.x == sps.getPicWidthInLumaSamples () )
    && ( ( rbPos.y & cu.cs->pcv->maxCUHeightMask ) == 0 || rbPos.y == sps.getPicHeightInLumaSamples() )
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
    && ( !cu.isSepTree() || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
#else
    && ( !CS::isDualITree( *cu.cs ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
#endif
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

    return terminating_bit();
  }

  return false;
}

#if JVET_O0119_BASE_PALETTE_444
void CABACReader::cu_palette_info(CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  const SPS&      sps = *(cu.cs->sps);
  TransformUnit&   tu = *cu.firstTU;
  int curPLTidx = 0;

  cu.lastPLTSize[compBegin] = cu.cs->prevPLT.curPLTSize[compBegin];

  if (cu.lastPLTSize[compBegin])
  {
    xDecodePLTPredIndicator(cu, MAXPLTSIZE, compBegin);
  }

  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        cu.curPLT[comp][curPLTidx] = cu.cs->prevPLT.curPLT[comp][idx];
      }
      curPLTidx++;
    }
  }

  uint32_t recievedPLTnum = 0;

  if (curPLTidx < MAXPLTSIZE)
  {
    recievedPLTnum = exp_golomb_eqprob(0);
  }

  cu.curPLTSize[compBegin] = curPLTidx + recievedPLTnum;
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = curPLTidx; idx < cu.curPLTSize[compBegin]; idx++)
    {
      ComponentID compID = (ComponentID)comp;
      const int  channelBitDepth = sps.getBitDepth(toChannelType(compID));
      cu.curPLT[compID][idx] = m_BinDecoder.decodeBinsEP(channelBitDepth);
    }
  }
  cu.useEscape[compBegin] = true; // JC
  if (cu.curPLTSize[compBegin] > 0)
  {
    uint32_t escCode = 0;
    escCode = m_BinDecoder.decodeBinEP();
    cu.useEscape[compBegin] = (escCode != 0);
  }
  uint32_t    indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  //encode index map
  PLTtypeBuf  runType = tu.getrunType(compBegin);
  PelBuf      runLength = tu.getrunLength(compBegin);
  PelBuf      curPLTIdx = tu.getcurPLTIdx(compBegin);
  uint32_t    height = cu.block(compBegin).height;
  uint32_t    width = cu.block(compBegin).width;

  int       numCopyIndexRuns = -1;
  bool      lastRunType = 0;
  uint32_t  numIndices = 0;
  uint32_t  adjust = 0;
  uint32_t  symbol = 0;
  std::list<int> parsedIdxList;
  if (indexMaxSize > 1)
  {
    uint32_t currParam = 3 + ((indexMaxSize) >> 3);
    numIndices = m_BinDecoder.decodeRemAbsEP(currParam, false, MAX_NUM_CHANNEL_TYPE); // JC: number of indices (INDEX RUN)
    numIndices++;
    numCopyIndexRuns = numIndices;
    while (numIndices--)
    {
      xReadTruncBinCode(symbol, indexMaxSize - adjust);
      adjust = 1;
      parsedIdxList.push_back(symbol);
    }
    lastRunType = m_BinDecoder.decodeBin(Ctx::RunTypeFlag());
    parseScanRotationModeFlag(cu, compBegin);
    adjust = 0;
  }
  else
  {
    cu.useRotation[compBegin] = false;
  }

  if (cu.useEscape[compBegin] && cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    if (!cu.isSepTree() || isLuma(tu.chType))
#else
    if (!CS::isDualITree(*tu.cs) || isLuma(tu.chType))
#endif
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
#if JVET_O1168_CU_CHROMA_QP_OFFSET
  if (cu.useEscape[compBegin] && cu.cs->slice->getUseChromaQpAdj() && !cuCtx.isChromaQpAdjCoded)
#else
  if (cu.cs->slice->getUseChromaQpAdj() && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded)
#endif
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    if (!cu.isSepTree() || isChroma(tu.chType))
#else
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))
#endif
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }


  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t strPos = 0;
  uint32_t endPos = height * width;
  while (strPos < endPos)
  {
    uint32_t posy = m_scanOrder[strPos].y;
    uint32_t posx = m_scanOrder[strPos].x;
    uint32_t posyprev = strPos == 0 ? 0 : m_scanOrder[strPos - 1].y;
    uint32_t posxprev = strPos == 0 ? 0 : m_scanOrder[strPos - 1].x;

    if (indexMaxSize > 1)
    {
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else if (strPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else
      {
        if (numCopyIndexRuns && strPos < endPos - 1) // JC: if numIndices (decoder will know this value) == 0 - > only CopyAbove, if strPos == endPos - 1, the last RunType was already coded
        {
          runType.at(posx, posy) = (m_BinDecoder.decodeBin(Ctx::RunTypeFlag()));
        }
        else
        {
          if (strPos == endPos - 1 && numCopyIndexRuns)
          {
            runType.at(posx, posy) = PLT_RUN_INDEX;
          }
          else
          {
            runType.at(posx, posy) = PLT_RUN_COPY;
          }
        }
      }
    }
    else
    {
      runType.at(posx, posy) = PLT_RUN_INDEX;
    }

    Pel curLevel = 0;
    if (runType.at(posx, posy) == PLT_RUN_INDEX)
    {
      if (!parsedIdxList.empty())
      {
        curLevel = parsedIdxList.front();
        parsedIdxList.pop_front();
      }
      else
      {
        curLevel = 0;
      }
      xAdjustPLTIndex(cu, curLevel, strPos, curPLTIdx, runType, indexMaxSize, compBegin);
    }

    if (indexMaxSize > 1)
    {
      bool lastRun;
      numCopyIndexRuns -= (runType.at(posx, posy) == PLT_RUN_INDEX);
      lastRun = numCopyIndexRuns == 0 && runType.at(posx, posy) == lastRunType;
      if (!lastRun)
      {
        runLength.at(posx, posy) = cu_run_val((PLTRunMode)runType.at(posx, posy), curLevel, endPos - strPos - numCopyIndexRuns - 1 - lastRunType) + 1;
      }
      else
      {
        runLength.at(posx, posy) = endPos - strPos;
      }

    }
    else
    {
      runLength.at(posx, posy) = endPos - strPos;
    }

    //assign run information
    for (int runidx = 1; runidx < runLength.at(posx, posy); runidx++)
    {
      int posYrun, posXrun;
      posYrun = m_scanOrder[strPos + runidx].y;
      posXrun = m_scanOrder[strPos + runidx].x;
      runType.at(posXrun, posYrun) = runType.at(posx, posy);
      runLength.at(posXrun, posYrun) = runLength.at(posx, posy);
    }

    uint32_t posYrun;
    uint32_t posXrun;
    if (runType.at(posx, posy) == PLT_RUN_INDEX)
    {
      for (uint32_t idx = 1; idx < runLength.at(posx, posy); idx++)
      {
        posYrun = m_scanOrder[strPos + idx].y;
        posXrun = m_scanOrder[strPos + idx].x;
        curPLTIdx.at(posXrun, posYrun) = curPLTIdx.at(posx, posy);
      }
    }
    else if (runType.at(posx, posy) == PLT_RUN_COPY)
    {
      for (uint32_t idx = 0; idx < runLength.at(posx, posy); idx++)
      {
        posYrun = m_scanOrder[strPos + idx].y;
        posXrun = m_scanOrder[strPos + idx].x;
        curPLTIdx.at(posXrun, posYrun) = (cu.useRotation[compBegin]) ? curPLTIdx.at(posXrun - 1, posYrun) : curPLTIdx.at(posXrun, posYrun - 1);
      }
    }
    strPos += (runLength.at(posx, posy));
  }
  assert(strPos == endPos);

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, sps.getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, sps.getChromaFormatIdc());
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    ComponentID compID = (ComponentID)comp;
    for (strPos = 0; strPos < endPos; strPos++)
    {
      uint32_t posy = m_scanOrder[strPos].y;
      uint32_t posx = m_scanOrder[strPos].x;
      if (curPLTIdx.at(posx, posy) == cu.curPLTSize[compBegin])
      {
        {
          PLTescapeBuf    escapeValue = tu.getescapeValue((ComponentID)comp);
          if (compID == COMPONENT_Y || compBegin != COMPONENT_Y)
          {
            escapeValue.at(posx, posy) = exp_golomb_eqprob(3);
            assert(escapeValue.at(posx, posy) < (1 << (cu.cs->sps->getBitDepth(toChannelType((ComponentID)comp)) + 1)));
          }
          if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
          {
            uint32_t posxC = posx >> scaleX;
            uint32_t posyC = posy >> scaleY;
            escapeValue.at(posxC, posyC) = exp_golomb_eqprob(3);
            assert(escapeValue.at(posxC, posyC) < (1 << (cu.cs->sps->getBitDepth(toChannelType(compID)) + 1)));
          }
        }
      }
    }
  }

}
void CABACReader::parseScanRotationModeFlag(CodingUnit& cu, ComponentID compBegin)
{
  cu.useRotation[compBegin] = m_BinDecoder.decodeBin(Ctx::RotationFlag());
}
void CABACReader::xDecodePLTPredIndicator(CodingUnit& cu, uint32_t maxPLTSize, ComponentID compBegin)
{
  uint32_t symbol, numPltPredicted = 0, idx = 0;

  symbol = exp_golomb_eqprob(0);

  if (symbol != 1)
  {
    while (idx < cu.lastPLTSize[compBegin] && numPltPredicted < maxPLTSize)
    {
      if (idx > 0)
      {
        symbol = exp_golomb_eqprob(0);
      }
      if (symbol == 1)
      {
        break;
      }

      if (symbol)
      {
        idx += symbol - 1;
      }
      cu.reuseflag[compBegin][idx] = 1;
      numPltPredicted++;
      idx++;
    }
  }
}
void CABACReader::xAdjustPLTIndex(CodingUnit& cu, Pel curLevel, uint32_t idx, PelBuf& paletteIdx, PLTtypeBuf& paletteRunType, int maxSymbol, ComponentID compBegin)
{
  uint32_t symbol;
  int refLevel = MAX_INT;
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  if (idx)
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      refLevel = paletteIdx.at(prevposx, prevposy);
      if (paletteIdx.at(prevposx, prevposy) == cu.curPLTSize[compBegin]) // escape
      {
        refLevel = maxSymbol - 1;
      }
    }
    else
    {
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        refLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin]) // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        refLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin]) // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
    }
    maxSymbol--;
  }
  symbol = curLevel;
  if (curLevel >= refLevel) // include escape mode
  {
    symbol++;
  }
  paletteIdx.at(posx, posy) = symbol;
}
uint32_t  CABACReader::cu_run_val(PLTRunMode runtype, const uint32_t paletteIdx, const uint32_t maxRun)
{
  uint32_t symbol = 0;
  if (runtype == PLT_RUN_COPY)
  {
  }
  else
  {
    g_paletteRunLeftLut[0] = (paletteIdx < PLT_RUN_MSB_IDX_CTX_T1 ? 0 : (paletteIdx < PLT_RUN_MSB_IDX_CTX_T2 ? 1 : 2));
  }
  symbol = xReadTruncMsbP1RefinementBits(runtype, maxRun, PLT_RUN_MSB_IDX_CABAC_BYPASS_THRE);
  return symbol;
}
uint32_t CABACReader::xReadTruncUnarySymbol(PLTRunMode runtype, uint32_t maxVal, uint32_t ctxT)
{
  if (maxVal == 0)
    return 0;

  uint8_t *ctxLut;
  ctxLut = (runtype == PLT_RUN_INDEX) ? g_paletteRunLeftLut : g_paletteRunTopLut;
  uint32_t bin, idx = 0;
  do
  {
    if (idx > ctxT)
      bin = m_BinDecoder.decodeBinEP();
    else
    {
      bin = m_BinDecoder.decodeBin(
        (idx <= ctxT)
        ? ((runtype == PLT_RUN_INDEX) ? Ctx::IdxRunModel(ctxLut[idx]) : Ctx::CopyRunModel(ctxLut[idx]))
        : ((runtype == PLT_RUN_INDEX) ? Ctx::IdxRunModel(ctxLut[ctxT]) : Ctx::CopyRunModel(ctxLut[ctxT])));
      //        idx <= ctxT? pcSCModel[ctxLut[idx]] : pcSCModel[ctxLut[ctxT]] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    }
    idx++;
  } while (bin && idx < maxVal);

  return (bin && idx == maxVal) ? maxVal : idx - 1;
}
uint32_t CABACReader::xReadTruncMsbP1RefinementBits(PLTRunMode runtype, uint32_t maxVal, uint32_t ctxT)
{
  if (maxVal == 0)
  {
    return 0;
  }
  uint32_t symbol;
  uint32_t msbP1 = xReadTruncUnarySymbol(runtype, floorLog2(maxVal) + 1, ctxT);
  if (msbP1 > 1)
  {
    uint32_t numBins = floorLog2(maxVal) + 1;
    if (msbP1 < numBins)
    {
      uint32_t bits = msbP1 - 1;
      symbol = m_BinDecoder.decodeBinsEP(bits);
      symbol |= (1 << bits);
    }
    else
    {
      uint32_t curValue = 1 << (numBins - 1);
      xReadTruncBinCode(symbol, maxVal + 1 - curValue);
      symbol += curValue;
    }
  }
  else
    symbol = msbP1;

  return symbol;
}
#endif

//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu, mrgCtx );
//    void  merge_flag      ( pu );
//    void  merge_data      ( pu, mrgCtx );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACReader::prediction_unit( PredictionUnit& pu, MergeCtx& mrgCtx )
{
  if( pu.cu->skip )
  {
    pu.mergeFlag = true;
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
#if JVET_O0249_MERGE_SYNTAX
    merge_data(pu);
#else
    if (CU::isIBC(*pu.cu))
    {
      merge_idx(pu);
    }
    else
    {
      if (pu.regularMergeFlag)
      {
        merge_idx(pu);
      }
      else
      {
        subblock_merge_flag( *pu.cu );
        MHIntra_flag(pu);
        if (pu.mhIntraFlag)
        {
          pu.intraDir[0] = PLANAR_IDX;
          pu.intraDir[1] = DM_CHROMA_IDX;
        }
        else
        {
          pu.cu->triangle = pu.cu->cs->slice->getSPS()->getUseTriangle() && pu.cu->cs->slice->isInterB() && !pu.cu->affine && !pu.mmvdMergeFlag && !pu.cu->mmvdSkip && pu.cs->slice->getMaxNumTriangleCand() >= 2;
        }
        if (pu.mmvdMergeFlag)
        {
          mmvd_merge_idx(pu);
        }
        else
          merge_data   ( pu );
      }
    }
#endif
  }
  else if (CU::isIBC(*pu.cu))
  {
    pu.interDir = 1;
    pu.cu->affine = false;
    pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
    mvd_coding(pu.mvd[REF_PIC_LIST_0]);
#if JVET_O0162_IBC_MVP_FLAG
#if JVET_O0455_IBC_MAX_MERGE_NUM
    if ( pu.cu->slice->getMaxNumIBCMergeCand() == 1 )
#else
    if ( pu.cu->slice->getMaxNumMergeCand() == 1 )
#endif
    {
      pu.mvpIdx[REF_PIC_LIST_0] = 0;
    }
    else
#endif
    mvp_flag(pu, REF_PIC_LIST_0);
  }
  else
  {
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
    smvd_mode( pu );

    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_0 );
      if( pu.cu->affine )
      {
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][0] );
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][1] );
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][2] );
        }
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_0] );
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }

    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      if ( pu.cu->smvdMode != 1 )
      {
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 /* PRED_BI */ )
      {
        pu.mvd[ REF_PIC_LIST_1 ] = Mv();
        pu.mvdAffi[REF_PIC_LIST_1][0] = Mv();
        pu.mvdAffi[REF_PIC_LIST_1][1] = Mv();
        pu.mvdAffi[REF_PIC_LIST_1][2] = Mv();
      }
      else if( pu.cu->affine )
      {
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][0] );
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][1] );
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][2] );
        }
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_1] );
      }
      }
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
  }
  if( pu.interDir == 3 /* PRED_BI */ && PU::isBipredRestriction(pu) )
  {
    pu.mv    [REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
    pu.interDir               =  1;
    pu.cu->GBiIdx = GBI_DEFAULT;
  }

  if ( pu.cu->smvdMode )
  {
    RefPicList eCurRefList = (RefPicList)(pu.cu->smvdMode - 1);
    pu.mvd[1 - eCurRefList].set( -pu.mvd[eCurRefList].hor, -pu.mvd[eCurRefList].ver );
#if JVET_O0567_MVDRange_Constraint
    CHECK(!((pu.mvd[1 - eCurRefList].getHor() >= MVD_MIN) && (pu.mvd[1 - eCurRefList].getHor() <= MVD_MAX)) || !((pu.mvd[1 - eCurRefList].getVer() >= MVD_MIN) && (pu.mvd[1 - eCurRefList].getVer() <= MVD_MAX)), "Illegal MVD value");
#endif
    pu.refIdx[1 - eCurRefList] = pu.cs->slice->getSymRefIdx( 1 - eCurRefList );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}

void CABACReader::smvd_mode( PredictionUnit& pu )
{
  pu.cu->smvdMode = 0;
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }

  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SYMMVD_FLAG );

  pu.cu->smvdMode = m_BinDecoder.decodeBin( Ctx::SmvdFlag() ) ? 1 : 0;

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

void CABACReader::subblock_merge_flag( CodingUnit& cu )
{
#if JVET_O0249_MERGE_SYNTAX
  cu.affine = false;
#else
  if ( cu.firstPU->mergeFlag && (cu.firstPU->mmvdMergeFlag || cu.mmvdSkip) )
  {
    return;
  }
#endif

#if JVET_O0220_METHOD1_SUBBLK_FLAG_PARSING
  if ( !cu.cs->slice->isIntra() && (cu.slice->getMaxNumAffineMergeCand() > 0) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
#else
  if ( !cu.cs->slice->isIntra() && (cu.cs->sps->getUseAffine() || cu.cs->sps->getSBTMVPEnabledFlag()) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
#endif
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__AFFINE_FLAG );

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
#if JVET_O0500_SEP_CTX_AFFINE_SUBBLOCK_MRG
    cu.affine = m_BinDecoder.decodeBin( Ctx::SubblockMergeFlag( ctxId ) );
#else
    cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );
#endif
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACReader::affine_flag( CodingUnit& cu )
{
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__AFFINE_FLAG );

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      ctxId = 0;
      cu.affineType = m_BinDecoder.decodeBin( Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
    else
    {
      cu.affineType = AFFINEMODEL_4PARAM;
    }
  }
}

void CABACReader::merge_flag( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MERGE_FLAG );

  pu.mergeFlag = ( m_BinDecoder.decodeBin( Ctx::MergeFlag() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

  if (pu.mergeFlag && CU::isIBC(*pu.cu))
  {
    pu.mmvdMergeFlag = false;
    pu.regularMergeFlag = false;
    return;
  }
#if !JVET_O0249_MERGE_SYNTAX
  if (pu.mergeFlag)
  {
    if (!pu.cs->sps->getUseMMVD() && (pu.lwidth() * pu.lheight() == 32))
    {
      pu.regularMergeFlag = true;
    }
    else
    {
      pu.regularMergeFlag = (m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(1)));
      DTRACE(g_trace_ctx, D_SYNTAX, "regular_merge_flag() ctx=%d pu.regularMergeFlag=%d\n", 1, pu.regularMergeFlag?1:0);
    }
    if (pu.regularMergeFlag)
    {
      pu.mmvdMergeFlag = false;
      pu.mhIntraFlag = false;
      pu.cu->affine = false;
      pu.cu->triangle = false;
    }
    else
    {
      if (pu.cs->sps->getUseMMVD())
      {
        bool isCUWithOnlyRegularAndMMVD=((pu.lwidth() == 8 && pu.lheight() == 4) || (pu.lwidth() == 4 && pu.lheight() == 8));
        if (isCUWithOnlyRegularAndMMVD)
        {
          pu.mmvdMergeFlag = !(pu.regularMergeFlag);
        }
        else
        {
          pu.mmvdMergeFlag = (m_BinDecoder.decodeBin(Ctx::MmvdFlag(0)));
          DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
        }
      }
      else
      {
        pu.mmvdMergeFlag = false;
      }
    }
  }
#endif
}


void CABACReader::merge_data( PredictionUnit& pu )
{
#if JVET_O0249_MERGE_SYNTAX
  if (CU::isIBC(*pu.cu))
  {
    merge_idx(pu);
    return;
  }
  else
  {
    CodingUnit cu = *pu.cu;
    subblock_merge_flag(*pu.cu);
    if (pu.cu->affine)
    {
      merge_idx(pu);
      cu.firstPU->regularMergeFlag = false;
      return;
    }

    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MERGE_FLAG );
    const bool triangleAvailable = pu.cu->cs->slice->getSPS()->getUseTriangle() && pu.cu->cs->slice->isInterB() && pu.cu->cs->slice->getMaxNumTriangleCand() > 1;
    const bool ciipAvailable = pu.cs->sps->getUseMHIntra() && !pu.cu->skip && pu.cu->lwidth() < MAX_CU_SIZE && pu.cu->lheight() < MAX_CU_SIZE;
    if (pu.cu->lwidth() * pu.cu->lheight() >= 64
      && (triangleAvailable || ciipAvailable))
    {
      cu.firstPU->regularMergeFlag = m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(cu.skip ? 0 : 1));
    }
    else
    {
      cu.firstPU->regularMergeFlag = true;
    }
    if (cu.firstPU->regularMergeFlag)
    {
      if (cu.cs->slice->getSPS()->getUseMMVD())
      {
        cu.firstPU->mmvdMergeFlag = m_BinDecoder.decodeBin(Ctx::MmvdFlag(0));
      }
      else
      {
        cu.firstPU->mmvdMergeFlag = false;
      }
      if (cu.skip)
      {
        cu.mmvdSkip = cu.firstPU->mmvdMergeFlag;
      }
    }
    else
    {
      pu.mmvdMergeFlag = false;
      pu.cu->mmvdSkip = false;
      if (triangleAvailable && ciipAvailable)
      {
        MHIntra_flag(pu);
      }
      else if (ciipAvailable)
      {
        pu.mhIntraFlag = true;
      }
      else
      {
        pu.mhIntraFlag = false;
      }
      if (pu.mhIntraFlag)
      {
        pu.intraDir[0] = PLANAR_IDX;
        pu.intraDir[1] = DM_CHROMA_IDX;
      }
      else
      {
        pu.cu->triangle = true;
      }
    }
  }
  if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
#else
  if (pu.cu->mmvdSkip)
#endif
  {
    mmvd_merge_idx(pu);
  }
  else
  {
    merge_idx(pu);
  }
}


void CABACReader::merge_idx( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MERGE_INDEX );

  if ( pu.cu->affine )
  {
    int numCandminus1 = int( pu.cs->slice->getMaxNumAffineMergeCand() ) - 1;
    pu.mergeIdx = 0;
    if ( numCandminus1 > 0 )
    {
      if ( m_BinDecoder.decodeBin( Ctx::AffMergeIdx() ) )
      {
        pu.mergeIdx++;
        for ( ; pu.mergeIdx < numCandminus1; pu.mergeIdx++ )
        {
            if ( !m_BinDecoder.decodeBinEP() )
            {
              break;
            }
        }
      }
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
  }
  else
  {
  int numCandminus1 = int( pu.cs->slice->getMaxNumMergeCand() ) - 1;
  pu.mergeIdx       = 0;

  if( pu.cu->triangle )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__TRIANGLE_INDEX );
    bool    splitDir;
    uint8_t candIdx0;
    uint8_t candIdx1;
    splitDir = m_BinDecoder.decodeBinEP();
    auto decodeOneIdx = [this](int numCandminus1) -> uint8_t
    {
      uint8_t decIdx = 0;
      if( numCandminus1 > 0 )
      {
        if( this->m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
        {
          decIdx++;
          for( ; decIdx < numCandminus1; decIdx++ )
          {
            if( !this->m_BinDecoder.decodeBinEP() )
              break;
          }
        }
      }
      return decIdx;
    };
    const int maxNumTriangleCand = pu.cs->slice->getMaxNumTriangleCand();
    CHECK(maxNumTriangleCand < 2, "Incorrect max number of triangle candidates");
    candIdx0 = decodeOneIdx(maxNumTriangleCand - 1);
    candIdx1 = decodeOneIdx(maxNumTriangleCand - 2);
    candIdx1 += candIdx1 >= candIdx0 ? 1 : 0;
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_split_dir=%d\n", splitDir );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx0=%d\n", candIdx0 );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx1=%d\n", candIdx1 );
    pu.triangleSplitDir = splitDir;
    pu.triangleMergeIdx0 = candIdx0;
    pu.triangleMergeIdx1 = candIdx1;
    return;
  }

#if JVET_O0455_IBC_MAX_MERGE_NUM
  if (pu.cu->predMode == MODE_IBC)
  {
    numCandminus1 = int(pu.cs->slice->getMaxNumIBCMergeCand()) - 1;
  }
#endif
  if( numCandminus1 > 0 )
  {
    if( m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
    {
      pu.mergeIdx++;
      for( ; pu.mergeIdx < numCandminus1; pu.mergeIdx++ )
      {
          if( !m_BinDecoder.decodeBinEP() )
          {
            break;
          }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
  }
}

void CABACReader::mmvd_merge_idx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MERGE_INDEX);

  int var0 = 0;
  if (pu.cs->slice->getMaxNumMergeCand() > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    var0 = m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);
  int numCandminus1_step = MMVD_REFINE_STEP - 1;
  int var1 = 0;
  if (m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdx()))
  {
    var1++;
    for (; var1 < numCandminus1_step; var1++)
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);
  int var2 = 0;
  if (m_BinDecoder.decodeBinEP())
  {
    var2 += 2;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  else
  {
    var2 += 0;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  int mvpIdx = (var0 * MMVD_MAX_REFINE_NUM + var1 * 4 + var2);
  pu.mmvdMergeIdx = mvpIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
}

void CABACReader::inter_pred_idc( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__INTER_DIR );

  if( pu.cs->slice->isInterP() )
  {
    pu.interDir = 1;
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( m_BinDecoder.decodeBin( Ctx::InterDir(ctxId) ) )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, pu.lumaPos().x, pu.lumaPos().y );
      pu.interDir = 3;
      return;
    }
  }
  if( m_BinDecoder.decodeBin( Ctx::InterDir(4) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", 2, pu.lumaPos().x, pu.lumaPos().y );
    pu.interDir = 2;
    return;
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
  pu.interDir = 1;
  return;
}


void CABACReader::ref_idx( PredictionUnit &pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__REF_FRM_IDX );

  if ( pu.cu->smvdMode )
  {
    pu.refIdx[eRefList] = pu.cs->slice->getSymRefIdx( eRefList );
    return;
  }

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::RefPic() ) )
  {
    if( numRef > 1 )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, pu.lumaPos().x, pu.lumaPos().y );
    }
    pu.refIdx[eRefList] = 0;
    return;
  }
  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::RefPic(1) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
    pu.refIdx[eRefList] = 1;
    return;
  }
  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      pu.refIdx[eRefList] = (signed char)( idx - 1 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx-1, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
  }
}



void CABACReader::mvp_flag( PredictionUnit& pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MVP_IDX );

  unsigned mvp_idx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvp_idx, pu.lumaPos().x, pu.lumaPos().y );
  pu.mvpIdx [eRefList] = mvp_idx;
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvp_idx );
}


void CABACReader::MHIntra_flag(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseMHIntra())
  {
    pu.mhIntraFlag = false;
    return;
  }
  if (pu.cu->skip)
  {
    pu.mhIntraFlag = false;
    return;
  }

#if !JVET_O0249_MERGE_SYNTAX
  if (pu.mmvdMergeFlag)
  {
    pu.mhIntraFlag = false;
    return;
  }
  if (pu.cu->affine)
  {
    pu.mhIntraFlag = false;
    return;
  }
  if (pu.cu->lwidth() * pu.cu->lheight() < 64 || pu.cu->lwidth() >= MAX_CU_SIZE || pu.cu->lheight() >= MAX_CU_SIZE)
  {
    pu.mhIntraFlag = false;
    return;
  }
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MH_INTRA_FLAG);

  pu.mhIntraFlag = (m_BinDecoder.decodeBin(Ctx::MHIntraFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "MHIntra_flag() MHIntra=%d pos=(%d,%d) size=%dx%d\n", pu.mhIntraFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
}



#if !JVET_O0525_REMOVE_PCM
//================================================================================
//  clause 7.3.8.7
//--------------------------------------------------------------------------------
//    void  pcm_samples( tu )
//================================================================================

void CABACReader::pcm_samples( TransformUnit& tu )
{
  CHECK( !tu.cu->ipcm, "pcm mode expected" );
#if !JVET_O0050_LOCAL_DUAL_TREE
  const CodingStructure *cs = tu.cs;
#endif
  const ChannelType chType = tu.chType;

  const SPS&        sps       = *tu.cu->cs->sps;
  tu.depth                    = 0;

#if JVET_O0050_LOCAL_DUAL_TREE
  ComponentID compStr = (tu.cu->isSepTree() && !isLuma( chType )) ? COMPONENT_Cb : COMPONENT_Y;
  ComponentID compEnd = (tu.cu->isSepTree() && isLuma( chType )) ? COMPONENT_Y : COMPONENT_Cr;
#else
  ComponentID compStr = (CS::isDualITree(*cs) && !isLuma(chType)) ? COMPONENT_Cb: COMPONENT_Y;
  ComponentID compEnd = (CS::isDualITree(*cs) && isLuma(chType)) ? COMPONENT_Y : COMPONENT_Cr;
#endif
  for( ComponentID compID = compStr; compID <= compEnd; compID = ComponentID(compID+1) )
  {
    PelBuf          samples     = tu.getPcmbuf( compID );
    const unsigned  sampleBits  = sps.getPCMBitDepth( toChannelType(compID) );
    for( unsigned y = 0; y < samples.height; y++ )
    {
      for( unsigned x = 0; x < samples.width; x++ )
      {
        samples.at(x, y) = m_BinDecoder.decodeBinsPCM( sampleBits );
      }
    }
  }
  m_BinDecoder.start();
}
#endif

//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================

#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
void CABACReader::transform_tree( CodingStructure &cs, Partitioner &partitioner, CUCtx& cuCtx,                         const PartSplit ispType, const int subTuIdx )
#else
void CABACReader::transform_tree( CodingStructure &cs, Partitioner &partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs, const PartSplit ispType, const int subTuIdx )
#endif
{
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  const UnitArea&   area = partitioner.currArea();
  CodingUnit&         cu = *cs.getCU(area.blocks[partitioner.chType], partitioner.chType);
  int       subTuCounter = subTuIdx;

  // split_transform_flag
  bool split = partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  const unsigned  trDepth = partitioner.currTrDepth;
#else
  ChromaCbfs chromaCbfsLastDepth;
  chromaCbfsLastDepth.Cb        = chromaCbfs.Cb;
  chromaCbfsLastDepth.Cr        = chromaCbfs.Cr;
  const UnitArea& area          = partitioner.currArea();

  CodingUnit&     cu            = *cs.getCU( area.blocks[partitioner.chType], partitioner.chType );
  const unsigned  trDepth       = partitioner.currTrDepth;
        int       subTuCounter  = subTuIdx;

  // split_transform_flag
  bool split = false;

  split = partitioner.canSplit( TU_MAX_TR_SPLIT, cs );

  bool max_tu_split = split;
#endif

  if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
  {
    split = true;
  }

  if( !split && cu.ispMode )
  {
    split = partitioner.canSplit( ispType, cs );
  }
#if !JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  const bool chromaCbfISP = area.blocks[COMPONENT_Cb].valid() && cu.ispMode && !split;

  // cbf_cb & cbf_cr
#if JVET_O0050_LOCAL_DUAL_TREE
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA ) && ( !cu.ispMode || chromaCbfISP ) )
#else
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) && ( !cu.ispMode || chromaCbfISP ) )
#endif
  {
    const int cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
    if (!max_tu_split)
    {
      {
        if (!(cu.sbtInfo && trDepth == 1))
          chromaCbfs.Cb &= cbf_comp(cs, area.blocks[COMPONENT_Cb], cbfDepth);
      }
      {
        if (!(cu.sbtInfo && trDepth == 1))
          chromaCbfs.Cr &= cbf_comp(cs, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb);
      }
    }
  }
#if JVET_O0050_LOCAL_DUAL_TREE
  else if( cu.isSepTree() )
#else
  else if( CS::isDualITree( cs ) )
#endif
  {
    chromaCbfs = ChromaCbfs( false );
  }
#endif

  if( split )
  {
    {

      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
      {
#if ENABLE_TRACING
        const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
        DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n", partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
        partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
      }
      else if( cu.ispMode )
      {
        partitioner.splitCurrArea( ispType, cs );
      }
      else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
      {
        partitioner.splitCurrArea( PartSplit( cu.getSbtTuSplit() ), cs );
      }
      else
        THROW( "Implicit TU split not available!" );
    }

    do
    {
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
      transform_tree( cs, partitioner, cuCtx,          ispType, subTuCounter );
#else
      ChromaCbfs subCbfs = chromaCbfs;
      transform_tree( cs, partitioner, cuCtx, subCbfs, ispType, subTuCounter );
#endif
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();

#if !JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    const UnitArea &currArea  = partitioner.currArea();
    const unsigned  currDepth = partitioner.currTrDepth;
    const unsigned numTBlocks = getNumberValidTBlocks( *cs.pcv );

    unsigned        compCbf[3] = { 0, 0, 0 };
    unsigned        cbfDepth   = 0;
    for( auto &currTU : cs.traverseTUs( currArea, partitioner.chType ) )
    {
      for( unsigned ch = 0; ch < numTBlocks; ch++ )
      {
        cbfDepth     = !isLuma( ComponentID( ch ) ) && cu.ispMode ? currDepth : currDepth + 1;
        compCbf[ch] |= ( TU::getCbfAtDepth( currTU, ComponentID( ch ), cbfDepth ) ? 1 : 0 );
      }
    }

    for (auto &currTU: cs.traverseTUs(currArea, partitioner.chType))
    {
      TU::setCbfAtDepth(currTU, COMPONENT_Y, currDepth, compCbf[COMPONENT_Y]);
      if (currArea.chromaFormat != CHROMA_400)
      {
        TU::setCbfAtDepth(currTU, COMPONENT_Cb, currDepth, compCbf[COMPONENT_Cb]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cr, currDepth, compCbf[COMPONENT_Cr]);
      }
    }
#endif
  }
  else
  {
    TransformUnit &tu = cs.addTU( CS::getArea( cs, area, partitioner.chType ), partitioner.chType );
    unsigned numBlocks = ::getNumberValidTBlocks( *cs.pcv );
    tu.checkTuNoResidual( partitioner.currPartIdx() );
#if !JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    chromaCbfs.Cb &= !tu.noResidual;
    chromaCbfs.Cr &= !tu.noResidual;
#endif

    for( unsigned compID = COMPONENT_Y; compID < numBlocks; compID++ )
    {
      if( tu.blocks[compID].valid() )
      {
        tu.getCoeffs( ComponentID( compID ) ).fill( 0 );
        tu.getPcmbuf( ComponentID( compID ) ).fill( 0 );
      }
    }
    tu.depth = trDepth;
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
    transform_unit(tu, cuCtx, partitioner, subTuCounter);
#else
    if( !isChroma( partitioner.chType ) )
    {
      if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, 1 );
      }
      else if( cu.sbtInfo && tu.noResidual )
      {
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, 0 );
      }
      else if( cu.sbtInfo && !chromaCbfsLastDepth.sigChroma( area.chromaFormat ) )
      {
        assert( !tu.noResidual );
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, 1 );
      }
      else
      {
        bool previousCbf       = false;
        bool rootCbfSoFar      = false;
        bool lastCbfIsInferred = false;
        if( cu.ispMode )
        {
          uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
          if( subTuCounter == nTus - 1 )
          {
            TransformUnit* tuPointer = cu.firstTU;
            for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
            {
              rootCbfSoFar |= TU::getCbfAtDepth( *tuPointer, COMPONENT_Y, trDepth );
              tuPointer = tuPointer->next;
            }
            if( !rootCbfSoFar )
            {
              lastCbfIsInferred = true;
            }
          }
          if( !lastCbfIsInferred )
          {
            previousCbf = TU::getPrevTuCbfAtDepth( tu, COMPONENT_Y, trDepth );
          }
        }
        bool cbfY = lastCbfIsInferred ? true : cbf_comp( cs, tu.Y(), trDepth, previousCbf, cu.ispMode );
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, ( cbfY ? 1 : 0 ) );
      }
    }
    if( area.chromaFormat != CHROMA_400 && ( !cu.ispMode || chromaCbfISP ) )
    {
      TU::setCbfAtDepth( tu, COMPONENT_Cb, trDepth, ( chromaCbfs.Cb ? 1 : 0 ) );
      TU::setCbfAtDepth( tu, COMPONENT_Cr, trDepth, ( chromaCbfs.Cr ? 1 : 0 ) );
    }

    transform_unit(tu, cuCtx, chromaCbfs);
#endif
  }
}

bool CABACReader::cbf_comp( CodingStructure& cs, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
#if JVET_O0193_REMOVE_TR_DEPTH_IN_CBF_CTX
  const unsigned  ctxId = DeriveCtx::CtxQtCbf( area.compID, prevCbf, useISP && isLuma( area.compID ) );
#else
  const unsigned  ctxId = DeriveCtx::CtxQtCbf( area.compID, depth, prevCbf, useISP && isLuma( area.compID ) );
#endif
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID ];

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__QT_CBF, area.size(), area.compID);

  unsigned  cbf = 0;
  if( area.compID == COMPONENT_Y && cs.getCU( area.pos(), ChannelType( area.compID ) )->bdpcmMode )
  {
#if JVET_O0193_REMOVE_TR_DEPTH_IN_CBF_CTX
    cbf = m_BinDecoder.decodeBin( ctxSet( 1 ) );
#else
    cbf = m_BinDecoder.decodeBin( ctxSet( 4 ) );
#endif
  }
  else
  {
    cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
  return cbf;
}

//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================

void CABACReader::mvd_coding( Mv &rMvd )
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_mvd    ( STATS__CABAC_BITS__MVD );
  CodingStatisticsClassType ctype_mvd_ep ( STATS__CABAC_BITS__MVD_EP );
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_mvd );

  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_mvd_ep );

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += exp_golomb_eqprob(1 );
    }
    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += exp_golomb_eqprob(1 );
    }
    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
#if JVET_O0567_MVDRange_Constraint
  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal MVD value");
#endif
}


//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
void CABACReader::transform_unit( TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter)
#else
void CABACReader::transform_unit( TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
#endif
{
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;

  CodingStructure&  cs = *tu.cs;
  CodingUnit&       cu = *tu.cu;
  ChromaCbfs        chromaCbfs;
  chromaCbfs.Cb = chromaCbfs.Cr = false;

  const bool chromaCbfISP = area.blocks[COMPONENT_Cb].valid() && cu.ispMode;

  // cbf_cb & cbf_cr
#if JVET_O0050_LOCAL_DUAL_TREE
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#else
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!CS::isDualITree(cs) || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#endif
  {
    const int cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
    {
      if (!(cu.sbtInfo && tu.noResidual))
        chromaCbfs.Cb = cbf_comp(cs, area.blocks[COMPONENT_Cb], cbfDepth);

      if (!(cu.sbtInfo && tu.noResidual))
        chromaCbfs.Cr = cbf_comp(cs, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb);
    }
  }
#if JVET_O0050_LOCAL_DUAL_TREE
  else if (cu.isSepTree())
#else
  else if (CS::isDualITree(cs))
#endif
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 1);
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 0);
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 1);
    }
    else
    {
      bool previousCbf = false;
      bool rootCbfSoFar = false;
      bool lastCbfIsInferred = false;
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < nTus - 1; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, trDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMPONENT_Y, trDepth);
        }
      }
      bool cbfY = lastCbfIsInferred ? true : cbf_comp(cs, tu.Y(), trDepth, previousCbf, cu.ispMode);
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, (cbfY ? 1 : 0));
    }
  }
  if (area.chromaFormat != CHROMA_400 && (!cu.ispMode || chromaCbfISP))
  {
    TU::setCbfAtDepth(tu, COMPONENT_Cb, trDepth, (chromaCbfs.Cb ? 1 : 0));
    TU::setCbfAtDepth(tu, COMPONENT_Cr, trDepth, (chromaCbfs.Cr ? 1 : 0));
  }
#else
  CodingUnit& cu         = *tu.cu;
#endif
  bool        lumaOnly   = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbfLuma    = ( tu.cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma  = ( lumaOnly ? false : ( chromaCbfs.Cb || chromaCbfs.Cr ) );

#if JVET_O0046_DQ_SIGNALLING
  if( ( cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma ) &&
#else
  if( ( cbfLuma || cbfChroma ) &&
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
    (!tu.cu->isSepTree() || isLuma(tu.chType)) )
#else
    (!CS::isDualITree(*tu.cs) || isLuma(tu.chType)) )
#endif
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
#if JVET_O1168_CU_CHROMA_QP_OFFSET
    if (cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cuCtx.isChromaQpAdjCoded)
#else
    if( cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded )
#endif
    {
      cu_chroma_qp_offset( cu );
      cuCtx.isChromaQpAdjCoded = true;
    }

#if JVET_O0105_ICT
  if( !lumaOnly )
  {
    joint_cb_cr( tu, ( tu.cbf[COMPONENT_Cb] ? 2 : 0 ) + ( tu.cbf[COMPONENT_Cr] ? 1 : 0 ) );
  }

#endif
    if( cbfLuma )
    {
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
      residual_coding( tu, COMPONENT_Y, cuCtx );
#else
      residual_coding( tu, COMPONENT_Y );
#endif
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( TU::hasCrossCompPredInfo( tu, compID ) )
        {
          cross_comp_pred( tu, compID );
        }
        if( tu.cbf[ compID ] )
        {
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
          residual_coding( tu, compID, cuCtx );
#else
          residual_coding( tu, compID );
#endif
      }
    }
  }
}

void CABACReader::cu_qp_delta( CodingUnit& cu, int predQP, int8_t& qp )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__DELTA_QP_EP );

  CHECK( predQP == std::numeric_limits<int>::max(), "Invalid predicted QP" );
  int qpY = predQP;
  int DQp = unary_max_symbol( Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( DQp >= CU_DQP_TU_CMAX )
  {
    DQp += exp_golomb_eqprob( CU_DQP_EG_k  );
  }
  if( DQp > 0 )
  {
    if( m_BinDecoder.decodeBinEP( ) )
    {
      DQp = -DQp;
    }
    int     qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
    qpY = ( (predQP + DQp + (MAX_QP + 1) + 2 * qpBdOffsetY) % ((MAX_QP + 1) + qpBdOffsetY)) - qpBdOffsetY;
  }
  qp = (int8_t)qpY;

  DTRACE( g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACReader::cu_chroma_qp_offset( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT, cu.blocks[cu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  // cu_chroma_qp_offset_flag
  int       length  = cu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListLen();
  unsigned  qpAdj   = m_BinDecoder.decodeBin( Ctx::ChromaQpAdjFlag() );
  if( qpAdj && length > 1 )
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol( Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
}

//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    RDPCMMode   explicit_rdpcm_mode     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

#if JVET_O0105_ICT
void CABACReader::joint_cb_cr( TransformUnit& tu, const int cbfMask )
{
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  if ( !tu.cu->slice->getSPS()->getJointCbCrEnabledFlag() )
  {
    return;
  }
#endif

#if JVET_O0543_ICT_ICU_ONLY
  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
#else
  if( cbfMask )
#endif
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__JOINT_CB_CR, tu.blocks[COMPONENT_Cr].lumaSize(), CHANNEL_TYPE_CHROMA );
    tu.jointCbCr = ( m_BinDecoder.decodeBin( Ctx::JointCbCrFlag( cbfMask-1 ) ) ? cbfMask : 0 );
  }
}
#else
void CABACReader::joint_cb_cr( TransformUnit& tu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__JOINT_CB_CR, tu.blocks[COMPONENT_Cr].lumaSize(), CHANNEL_TYPE_CHROMA );
  tu.jointCbCr = m_BinDecoder.decodeBin( Ctx::JointCbCrFlag( 0 ) );
}
#endif

#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID, CUCtx& cuCtx )
#else
void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID )
#endif
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

#if JVET_O0105_ICT
  if( compID == COMPONENT_Cr && tu.jointCbCr == 3 )
    return;
#else
  // Joint Cb-Cr residual mode is signalled if both Cb and Cr cbfs are true
  if ( compID == COMPONENT_Cr && TU::getCbf( tu, COMPONENT_Cb ) )
  {
    joint_cb_cr( tu );

    // No Cr residual in bitstream in joint Cb-Cr residual mode
    if ( tu.jointCbCr )
      return;
  }
#endif

  // parse transform skip and explicit rdpcm mode
  mts_coding         ( tu, compID );
  explicit_rdpcm_mode( tu, compID );

  if( isLuma( compID ) && ( tu.mtsIdx == MTS_SKIP || tu.cu->bdpcmMode ) )
  {
    residual_codingTS( tu, compID );
    return;
  }

  // determine sign hiding
  bool signHiding  = ( cu.cs->slice->getSignDataHidingEnabledFlag() && !cu.transQuantBypass && tu.rdpcm[compID] == RDPCM_OFF );
  if(  signHiding && CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && tu.mtsIdx==MTS_SKIP )
  {
    const ChannelType chType    = toChannelType( compID );
    const unsigned    intraMode = PU::getFinalIntraMode( *cu.cs->getPU( tu.blocks[compID].pos(), chType ), chType );
    if( intraMode == HOR_IDX || intraMode == VER_IDX )
    {
      signHiding = false;
    }
  }

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;

  // parse last coeff position
  cctx.setScanPosLast( last_sig_coeff( cctx, tu, compID ) );
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS
  if( tu.mtsIdx != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
    cuCtx.violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
#endif
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  if( tu.mtsIdx != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx.lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
#endif
  // parse subblocks
  const int stateTransTab = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
  int       state         = 0;

#if JVET_O0052_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT
  int ctxBinSampleRatio = (compID == COMPONENT_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;
#endif

    for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
    {
      cctx.initSubblock       ( subSetId );
#if JVET_O0538_SPS_CONTROL_ISP_SBT
      if( ( tu.mtsIdx > MTS_SKIP || ( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
      if( ( tu.mtsIdx > MTS_SKIP || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
      {
        if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) ) || ( tu.blocks[ compID ].width == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth() ) ) )
        {
          continue;
        }
      }
      residual_coding_subblock( cctx, coeff, stateTransTab, state );
    }

}

void CABACReader::mts_coding( TransformUnit& tu, ComponentID compID )
{
#if !JVET_O0294_TRANSFORM_CLEANUP
  const CodingUnit  &cu = *tu.cu;
#endif
  const bool  tsAllowed = TU::isTSAllowed ( tu, compID );
  const bool mtsAllowed = TU::isMTSAllowed( tu, compID );

  if( tu.cu->bdpcmMode ) tu.mtsIdx = MTS_SKIP;
  if( !mtsAllowed && !tsAllowed ) return;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[compID], compID );

  int symbol = 0;
  int ctxIdx = 0;

  if( tsAllowed )
  {
    ctxIdx = 6;
    symbol = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );
#if JVET_O0294_TRANSFORM_CLEANUP
    tu.mtsIdx = symbol ? MTS_SKIP : MTS_DCT2_DCT2;
#else
    tu.mtsIdx = symbol ? MTS_DCT2_DCT2 : MTS_SKIP;
#endif
  }

  if( tu.mtsIdx != MTS_SKIP )
  {
    if( mtsAllowed )
    {
#if JVET_O0294_TRANSFORM_CLEANUP
      ctxIdx = 0;
#else
      ctxIdx = std::min( (int)cu.qtDepth, 5 );
#endif
      symbol = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );

      if( symbol )
      {
        ctxIdx    = 7;
        tu.mtsIdx = MTS_DST7_DST7; // mtsIdx = 2 -- 4
        for( int i = 0; i < 3; i++, ctxIdx++ )
        {
          symbol = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );
          tu.mtsIdx += symbol;

          if( !symbol )
          {
            break;
          }
        }
      }
    }
  }
#if JVET_O0294_TRANSFORM_CLEANUP
  DTRACE(g_trace_ctx, D_SYNTAX, "mts_coding() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), tu.mtsIdx);
#else
  DTRACE( g_trace_ctx, D_SYNTAX, "mts_coding() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), tu.mtsIdx );
#endif
}

void CABACReader::isp_mode( CodingUnit& cu )
{
#if !JVET_O0525_REMOVE_PCM
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || cu.ipcm || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) )
#else
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) )
#endif
  {
    cu.ispMode = NOT_INTRA_SUBPARTITIONS;
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__ISP_MODE_FLAG);

  int symbol = m_BinDecoder.decodeBin(Ctx::ISPMode(0));

  if( symbol )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__ISP_SPLIT_FLAG );
    cu.ispMode = 1 + m_BinDecoder.decodeBin( Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACReader::explicit_rdpcm_mode( TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;

  tu.rdpcm[compID] = RDPCM_OFF;

  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.mtsIdx==MTS_SKIP || cu.transQuantBypass ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__EXPLICIT_RDPCM_BITS, tu.blocks[tu.chType].lumaSize() );

    ChannelType chType = toChannelType( compID );
    if( m_BinDecoder.decodeBin( Ctx::RdpcmFlag( chType ) ) )
    {
      if( m_BinDecoder.decodeBin( Ctx::RdpcmDir( chType ) ) )
      {
        tu.rdpcm[compID] = RDPCM_VER;
      }
      else
      {
        tu.rdpcm[compID] = RDPCM_HOR;
      }
    }
  }
}

#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
void CABACReader::residual_lfnst_mode( CodingUnit& cu,  CUCtx& cuCtx  )
#else
void CABACReader::residual_lfnst_mode( CodingUnit& cu )
#endif
{
#if JVET_O0213_RESTRICT_LFNST_TO_MAX_TB_SIZE
  int chIdx = CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#endif
  if( cu.ispMode != NOT_INTRA_SUBPARTITIONS ||
#if JVET_O0925_MIP_SIMPLIFICATIONS
      (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize())) ||
#else
      cu.mipFlag == true ||
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
    ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
#else
    ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
#endif
#if JVET_O0213_RESTRICT_LFNST_TO_MAX_TB_SIZE
#if JVET_O0545_MAX_TB_SIGNALLING
    || ( cu.blocks[ chIdx ].lumaSize().width > cu.cs->sps->getMaxTbSize() || cu.blocks[ chIdx ].lumaSize().height > cu.cs->sps->getMaxTbSize() )
#else
    || ( cu.blocks[ chIdx ].lumaSize().width > MAX_TB_SIZEY || cu.blocks[ chIdx ].lumaSize().height > MAX_TB_SIZEY )
#endif
#endif
    )
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__LFNST );

  if( cu.cs->sps->getUseLFNST() && CU::isIntra( cu ) && !CU::isLosslessCoded( cu ) )
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    const bool lumaFlag              = cu.isSepTree() ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag            = cu.isSepTree() ? ( isChroma( cu.chType ) ? true : false ) : true;
#else
    const bool lumaFlag              = CS::isDualITree( *cu.cs ) ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag            = CS::isDualITree( *cu.cs ) ? ( isChroma( cu.chType ) ? true : false ) : true;
#endif
#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
    bool nonZeroCoeffNonTs;
#endif
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS
    bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] );
#else
    bool nonZeroCoeffNonTsCorner8x8 = CU::getNumNonZeroCoeffNonTsCorner8x8( cu, lumaFlag, chromaFlag ) > 0;
#endif
#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
#if JVET_O0050_LOCAL_DUAL_TREE
    const int  nonZeroCoeffThr       = cu.isSepTree() ? ( isLuma( cu.chType ) ? LFNST_SIG_NZ_LUMA : LFNST_SIG_NZ_CHROMA ) : LFNST_SIG_NZ_LUMA + LFNST_SIG_NZ_CHROMA;
#else
    const int  nonZeroCoeffThr       = CS::isDualITree( *cu.cs ) ? ( isLuma( cu.chType ) ? LFNST_SIG_NZ_LUMA : LFNST_SIG_NZ_CHROMA ) : LFNST_SIG_NZ_LUMA + LFNST_SIG_NZ_CHROMA;
#endif
    nonZeroCoeffNonTs = CU::getNumNonZeroCoeffNonTs( cu, lumaFlag, chromaFlag ) > nonZeroCoeffThr;
#endif
#if JVET_O0368_LFNST_WITH_DCT2_ONLY
    const bool isNonDCT2 = (TU::getCbf(*cu.firstTU, ComponentID(COMPONENT_Y)) && cu.firstTU->mtsIdx != MTS_DCT2_DCT2);
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
    if( !cuCtx.lfnstLastScanPos || nonZeroCoeffNonTsCorner8x8 || isNonDCT2 )
#else
    if (!nonZeroCoeffNonTs || nonZeroCoeffNonTsCorner8x8 || isNonDCT2)
#endif
#else
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
    if( !cuCtx.lfnstLastScanPos || nonZeroCoeffNonTsCorner8x8 )
#else
    if( !nonZeroCoeffNonTs || nonZeroCoeffNonTsCorner8x8 )
#endif
#endif
    {
      cu.lfnstIdx = 0;
      return;
    }
  }
  else
  {
    cu.lfnstIdx = 0;
    return;
  }


  unsigned cctx = 0;
#if JVET_O0368_LFNST_WITH_DCT2_ONLY
#if JVET_O0050_LOCAL_DUAL_TREE
  if ( cu.isSepTree() ) cctx++;
#else
  if ( CS::isDualITree(*cu.cs) ) cctx++;
#endif
#else
#if JVET_O0050_LOCAL_DUAL_TREE
  if( cu.firstTU->mtsIdx < MTS_DST7_DST7 && cu.isSepTree() ) cctx++;
#else
  if( cu.firstTU->mtsIdx < MTS_DST7_DST7 && CS::isDualITree( *cu.cs ) ) cctx++;
#endif
#endif

  uint32_t idxLFNST = m_BinDecoder.decodeBin( Ctx::LFNSTIdx( cctx ) );
  if( idxLFNST )
  {
    idxLFNST += m_BinDecoder.decodeBinEP();
  }
  cu.lfnstIdx = idxLFNST;

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}

int CABACReader::last_sig_coeff( CoeffCodingContext& cctx, TransformUnit& tu, ComponentID compID )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__LAST_SIG_X_Y, Size( cctx.width(), cctx.height() ), cctx.compID() );

  unsigned PosLastX = 0, PosLastY = 0;
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

#if JVET_O0538_SPS_CONTROL_ISP_SBT
  if( ( tu.mtsIdx > MTS_SKIP || ( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
  if( ( tu.mtsIdx > MTS_SKIP || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
  {
    maxLastPosX = ( tu.blocks[ compID ].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[ compID ].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }

  for( ; PosLastX < maxLastPosX; PosLastX++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastXCtxId( PosLastX ) ) )
    {
      break;
    }
  }
  for( ; PosLastY < maxLastPosY; PosLastY++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastYCtxId( PosLastY ) ) )
    {
      break;
    }
  }
  if( PosLastX > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastX - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastX = g_uiMinInGroup[ PosLastX ] + uiTemp;
  }
  if( PosLastY > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastY - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastY = g_uiMinInGroup[ PosLastY ] + uiTemp;
  }

  int blkPos;
  {
    blkPos = PosLastX + ( PosLastY * cctx.width() );
  }

  int scanPos = 0;
  for( ; scanPos < cctx.maxNumCoeff() - 1; scanPos++ )
  {
    if( blkPos == cctx.blockPos( scanPos ) )
    {
      break;
    }
  }
  return scanPos;
}



void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, const int stateTransTable, int& state )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = ( isLast || !minSubPos );
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  uint8_t   ctxOffset[16];

  //===== decode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       numNonZero    =  0;
#if JVET_O0052_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT
  int       remRegBins    = cctx.regBinLimit;
#else
  bool      is2x2subblock = ( cctx.log2CGSize() == 2 );
  int       remRegBins    = ( is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK );
#endif
  int       firstPosMode2 = minSubPos - 1;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      sigFlag = m_BinDecoder.decodeBin( sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
      cctx.sigCtxIdAbs( nextSigPos, coeff, state ); // required for setting variables that are needed for gtx/par context selection
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff           = cctx.ctxOffsetAbs();
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>( lastNZPos, nextSigPos );

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      unsigned gt1Flag = m_BinDecoder.decodeBin( cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      unsigned parFlag = 0;
      unsigned gt2Flag = 0;
      if( gt1Flag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbs( ctxOff ) );

        remRegBins--;
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);
        gt2Flag = m_BinDecoder.decodeBin( cctx.greater2CtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2Flag, cctx.greater2CtxIdAbs( ctxOff ) );
        remRegBins--;
      }
      coeff[ blkPos ] += 1 + parFlag + gt1Flag + (gt2Flag << 1);
    }

    state = ( stateTransTable >> ((state<<2)+((coeff[blkPos]&1)<<1)) ) & 3;
  }
  firstPosMode2 = nextSigPos;
#if JVET_O0052_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT
  cctx.regBinLimit = remRegBins;
#endif


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
    ricePar = g_auiGoRiceParsCoeff[sumAll];
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    if( tcoeff >= 4 )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
      int       rem     = m_BinDecoder.decodeRemAbsEP( ricePar, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      tcoeff += (rem<<1);
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0[std::max(0, state - 1)][sumAll];
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_escs);
    int       rem       = m_BinDecoder.decodeRemAbsEP( rice, cctx.extPrec(), cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    TCoeff    tcoeff  = ( rem == pos0 ? 0 : rem < pos0 ? rem+1 : rem );
    state = ( stateTransTable >> ((state<<2)+((tcoeff&1)<<1)) ) & 3;
    if( tcoeff )
    {
      int        blkPos         = cctx.blockPos( scanPos );
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = scanPos;
      lastNZPos  = std::max<int>( lastNZPos, scanPos );
      coeff[blkPos] = tcoeff;
    }
  }

  //===== decode sign's =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
  const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
  unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );

  //===== set final coefficents =====
  int sumAbs = 0;
  for( unsigned k = 0; k < numSigns; k++ )
  {
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
    signPattern         <<= 1;
  }
  if( numNonZero > numSigns )
  {
    int k                 = numSigns;
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
  }
}

void CABACReader::residual_codingTS( TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, tu.cu->bdpcmMode );
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;

  cctx.setNumCtxBins( 2 * tu.lwidth()*tu.lheight() );

  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId );
    residual_coding_subblockTS( cctx, coeff );
  }
}

void CABACReader::residual_coding_subblockTS( CoeffCodingContext& cctx, TCoeff* coeff )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
#if TR_ONLY_COEFF_STATS
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG_TS, cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS_TS,        cctx.width(), cctx.height(), cctx.compID() );
#else
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

#endif

  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;
  unsigned    signPattern = 0;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = cctx.isLastSubSet() && cctx.noneSigGroup();
  if( !sigGroup )
  {
#if !JVET_O0409_EXCLUDE_CODED_SUB_BLK_FLAG_FROM_COUNT
    if( cctx.isContextCoded() )
    {
#endif
      sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId( true ) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", sigGroup, cctx.sigGroupCtxId() );
#if !JVET_O0409_EXCLUDE_CODED_SUB_BLK_FLAG_FROM_COUNT
    }
    else
    {
      sigGroup = m_BinDecoder.decodeBinEP( );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() EPbin=%d\n", sigGroup );
    }
#endif
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode absolute values =====
  const int inferSigPos   = minSubPos;
  int       numNonZero    =  0;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  for( ; nextSigPos <= minSubPos; nextSigPos++ )
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      if( cctx.isContextCoded() )
      {
        const unsigned sigCtxId = cctx.sigCtxIdAbsTS( nextSigPos, coeff );
        sigFlag = m_BinDecoder.decodeBin( sigCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      }
      else
      {
        sigFlag = m_BinDecoder.decodeBinEP( );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() EPbin=%d\n", sigFlag );
      }
    }

    if( sigFlag )
    {
      //===== decode sign's =====
#if TR_ONLY_COEFF_STATS
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT_TS, Size(cctx.width(), cctx.height()), cctx.compID());
#else
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
#endif
      int sign;
      if( cctx.isContextCoded() )
      {
#if JVET_O0122_TS_SIGN_LEVEL
        const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
        sign = m_BinDecoder.decodeBin(signCtxId);
#else
        sign = m_BinDecoder.decodeBin( Ctx::TsResidualSign(  cctx.bdpcm() ? 1 : 0 ) );
#endif
      }
      else
      {
        sign = m_BinDecoder.decodeBinEP( );
      }

      signPattern += ( sign << numNonZero );

      sigBlkPos[numNonZero++] = blkPos;

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      unsigned gt1Flag;
#if JVET_O0122_TS_SIGN_LEVEL
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      if( cctx.isContextCoded() )
      {
        gt1Flag = m_BinDecoder.decodeBin(gt1CtxId);
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, gt1CtxId );
      }
      else
      {
        gt1Flag = m_BinDecoder.decodeBinEP( );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() EPbin=%d\n", gt1Flag );
      }
#else
      if( cctx.isContextCoded() )
      {
        gt1Flag = m_BinDecoder.decodeBin( cctx.greaterXCtxIdAbsTS(0) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greaterXCtxIdAbsTS(0) );
      }
      else
      {
        gt1Flag = m_BinDecoder.decodeBinEP( );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() EPbin=%d\n", gt1Flag );
      }
#endif

      unsigned parFlag = 0;
      if( gt1Flag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        if( cctx.isContextCoded() )
        {
          parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbsTS() );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbsTS() );
        }
        else
        {
          parFlag = m_BinDecoder.decodeBinEP( );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() EPbin=%d\n", parFlag );
        }
      }
#if JVET_O0122_TS_SIGN_LEVEL
      coeff[ blkPos ] = (sign ? -1 : 1 ) * (1 + parFlag + gt1Flag);
#else
      coeff[ blkPos ] += 1 + parFlag + gt1Flag;
#endif
    }
  }

  int cutoffVal = 2;
  const int numGtBins = 4;

  //===== 2nd PASS: gt2 =====
#if JVET_O0619_GTX_SINGLE_PASS_TS_RESIDUAL_CODING
  for (int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++)
  {
    TCoeff& tcoeff = coeff[cctx.blockPos(scanPos)];
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
#if JVET_O0122_TS_SIGN_LEVEL
      if( tcoeff < 0)
      {
        tcoeff = -tcoeff;
      }
#endif
       if (tcoeff >= cutoffVal)
       {
          RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);
          unsigned gt2Flag;
          if (cctx.isContextCoded())
          {
            gt2Flag = m_BinDecoder.decodeBin(cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
            tcoeff += (gt2Flag << 1);
            DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2Flag, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, tcoeff);
          }
          else
          {
            gt2Flag = m_BinDecoder.decodeBinEP();
            tcoeff += (gt2Flag << 1);
            DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() EPbin=%d sp=%d coeff=%d\n", i, gt2Flag, scanPos, tcoeff);
          }
       }
       cutoffVal += 2;
    }
  }
#else
  for( int i = 0; i < numGtBins; i++ )
  {
    for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
    {
      TCoeff& tcoeff = coeff[cctx.blockPos( scanPos )];
#if JVET_O0122_TS_SIGN_LEVEL
      if( tcoeff < 0)
      {
        tcoeff = -tcoeff;
      }
#endif
      if( tcoeff >= cutoffVal )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt2 );
        unsigned gt2Flag;
        if( cctx.isContextCoded() )
        {
          gt2Flag = m_BinDecoder.decodeBin( cctx.greaterXCtxIdAbsTS(cutoffVal>>1) );
          tcoeff += ( gt2Flag << 1 );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2Flag, cctx.greaterXCtxIdAbsTS(cutoffVal>>1), scanPos, tcoeff );
        }
        else
        {
          gt2Flag = m_BinDecoder.decodeBinEP( );
          tcoeff += ( gt2Flag << 1 );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() EPbin=%d sp=%d coeff=%d\n", i, gt2Flag, scanPos, tcoeff );
        }
      }
    }
    cutoffVal += 2;
  }
#endif
  //===== 3rd PASS: Go-rice codes =====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
    if( tcoeff >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      int       rem  = m_BinDecoder.decodeRemAbsEP( rice, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );
      tcoeff += ( rem << 1 );
    }
#if JVET_O0122_TS_SIGN_LEVEL
    if (!cctx.bdpcm())
    {
      if (tcoeff > 0)
      {
        int rightPixel, belowPixel;
        cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
        tcoeff = cctx.decDeriveModCoeff(rightPixel, belowPixel, tcoeff);
      }
    }
#endif
  }

  //===== set final coefficents =====
  for( unsigned k = 0; k < numNonZero; k++ )
  {
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    coeff[ sigBlkPos[k] ] = ( signPattern & 1 ? -AbsCoeff : AbsCoeff );
    signPattern         >>= 1;
  }
}


//================================================================================
//  clause 7.3.8.12
//--------------------------------------------------------------------------------
//    void  cross_comp_pred( tu, compID )
//================================================================================

void CABACReader::cross_comp_pred( TransformUnit& tu, ComponentID compID )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__CROSS_COMPONENT_PREDICTION, tu.blocks[compID], compID);

  signed char alpha   = 0;
  unsigned    ctxBase = ( compID == COMPONENT_Cr ? 5 : 0 );
  unsigned    symbol  = m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase) );
  if( symbol )
  {
    // Cross-component prediction alpha is non-zero.
    symbol = m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase+1) );
    if( symbol )
    {
      // alpha is 2 (symbol=1), 4(symbol=2) or 8(symbol=3).
      // Read up to two more bits
      symbol += unary_max_symbol( Ctx::CrossCompPred(ctxBase+2), Ctx::CrossCompPred(ctxBase+3), 2 );
    }
    alpha = ( 1 << symbol );
    if( m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase+4) ) )
    {
      alpha = -alpha;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
  tu.compAlpha[compID] = alpha;
}



//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol  )
{
  unsigned onesRead = 0;
  while( onesRead < maxSymbol && m_BinDecoder.decodeBin( onesRead == 0 ? ctxId0 : ctxIdN ) == 1 )
  {
    ++onesRead;
  }
  return onesRead;
}


unsigned CABACReader::unary_max_eqprob( unsigned maxSymbol )
{
  for( unsigned k = 0; k < maxSymbol; k++ )
  {
    if( !m_BinDecoder.decodeBinEP() )
    {
      return k;
    }
  }
  return maxSymbol;
}


unsigned CABACReader::exp_golomb_eqprob( unsigned count )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while( bit )
  {
    bit     = m_BinDecoder.decodeBinEP( );
    symbol += bit << count++;
  }
  if( --count )
  {
    symbol += m_BinDecoder.decodeBinsEP( count );
  }
  return symbol;
}

unsigned CABACReader::code_unary_fixed( unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  unsigned idx;
  bool unary = m_BinDecoder.decodeBin( ctxId );
  if( unary )
  {
    idx = unary_max_eqprob( unary_max );
  }
  else
  {
    idx = unary_max + 1 + m_BinDecoder.decodeBinsEP( fixed );
  }
  return idx;
}

void CABACReader::mip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->getUseMIP() )
  {
    cu.mipFlag = false;
    return;
  }
#if JVET_O0545_MAX_TB_SIGNALLING
  if( cu.lwidth() > cu.cs->sps->getMaxTbSize() || cu.lheight() > cu.cs->sps->getMaxTbSize())
#else
  if( cu.lwidth() > MIP_MAX_WIDTH || cu.lheight() > MIP_MAX_HEIGHT )
#endif
  {
    cu.mipFlag = false;
    return;
  }
  if( !mipModesAvailable( cu.Y() ) )
  {
    cu.mipFlag = false;
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  cu.mipFlag = m_BinDecoder.decodeBin( Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACReader::mip_pred_modes( CodingUnit &cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.Y().valid() )
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    mip_pred_mode( pu );
  }
}

void CABACReader::mip_pred_mode( PredictionUnit &pu )
{
#if JVET_O0545_MAX_TB_SIGNALLING
  CHECK( pu.lwidth() > pu.cs->sps->getMaxTbSize() || pu.lheight() > pu.cs->sps->getMaxTbSize(), "Error: block size not supported" );
#else
  CHECK( pu.lwidth() > MIP_MAX_WIDTH || pu.lheight() > MIP_MAX_HEIGHT, "Error: block size not supported" );
#endif

  const int numModes   = getNumModesMip( pu.Y() ); CHECKD( numModes > MAX_NUM_MIP_MODE, "Error: too many MIP modes" );

#if JVET_O0925_MIP_SIMPLIFICATIONS
  uint32_t mipMode;
  xReadTruncBinCode( mipMode, numModes );
  pu.intraDir[CHANNEL_TYPE_LUMA] = mipMode;
#else
  int      unaryMax    = NUM_MPM_MIP - 1;
  int      fixedLength = getNumEpBinsMip( pu.Y() );
  unsigned modeIdx     = code_unary_fixed( Ctx::MipMode( 0 ), unaryMax, fixedLength );
  CHECK( modeIdx >= numModes, "modeIdx out of range" );

  // derive true MIP mode from modeIdx
  unsigned mpm[NUM_MPM_MIP];
  PU::getMipMPMs(pu, mpm);

  if (modeIdx < NUM_MPM_MIP)
  {
    pu.intraDir[CHANNEL_TYPE_LUMA] = mpm[modeIdx];
  }
  else
  {
    modeIdx -= NUM_MPM_MIP;
    std::sort(mpm, mpm + NUM_MPM_MIP);

    for( unsigned i = 0; i < NUM_MPM_MIP; i++ )
    {
      modeIdx += (modeIdx >= mpm[i]);
    }
    pu.intraDir[CHANNEL_TYPE_LUMA] = modeIdx;
  }
#endif
  CHECKD(pu.intraDir[CHANNEL_TYPE_LUMA] < 0 || pu.intraDir[CHANNEL_TYPE_LUMA] >= numModes, "Invalid MIP mode");

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA] );
}
