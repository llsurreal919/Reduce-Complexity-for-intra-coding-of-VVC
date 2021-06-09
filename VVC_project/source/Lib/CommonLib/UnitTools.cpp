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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools


uint64_t CS::getEstBits(const CodingStructure &cs)
{
  return cs.fracBits >> SCALE_BITS;
}



bool CS::isDualITree( const CodingStructure &cs )
{
  return cs.slice->isIntra() && !cs.pcv->ISingleTree;
}

UnitArea CS::getArea( const CodingStructure &cs, const UnitArea &area, const ChannelType chType )
{
#if JVET_O0050_LOCAL_DUAL_TREE
  return isDualITree( cs ) || cs.treeType != TREE_D ? area.singleChan( chType ) : area;
#else
  return isDualITree( cs ) ? area.singleChan( chType ) : area;
#endif
}
void CS::setRefinedMotionField(CodingStructure &cs)
{
  for (CodingUnit *cu : cs.cus)
  {
    for (auto &pu : CU::traversePUs(*cu))
    {
      PredictionUnit subPu = pu;
      int dx, dy, x, y, num = 0;
      dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
      dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
      Position puPos = pu.lumaPos();
      if (PU::checkDMVRCondition(pu))
      {
        for (y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy)
        {
          for (x = puPos.x; x < (puPos.x + pu.lumaSize().width); x = x + dx)
          {
            subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
            subPu.mv[0] = pu.mv[0];
            subPu.mv[1] = pu.mv[1];
            subPu.mv[REF_PIC_LIST_0] += pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_1] -= pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_0].clipToStorageBitDepth();
            subPu.mv[REF_PIC_LIST_1].clipToStorageBitDepth();
            pu.mvdL0SubPu[num].setZero();
            num++;
            PU::spanMotionInfo(subPu);
          }
        }
      }
    }
  }
}
// CU tools

#if JVET_O1164_RPR
bool CU::getRprScaling( const SPS* sps, const PPS* curPPS, const PPS* refPPS, int& xScale, int& yScale )
{
#if RPR_CONF_WINDOW
  const Window& curConfWindow = curPPS->getConformanceWindow();
  int curPicWidth = curPPS->getPicWidthInLumaSamples() - (curConfWindow.getWindowLeftOffset() + curConfWindow.getWindowRightOffset()) * SPS::getWinUnitY(sps->getChromaFormatIdc());
  int curPicHeight = curPPS->getPicHeightInLumaSamples() - (curConfWindow.getWindowTopOffset() + curConfWindow.getWindowBottomOffset()) * SPS::getWinUnitY(sps->getChromaFormatIdc());
  const Window& refConfWindow = refPPS->getConformanceWindow();
  int refPicWidth = refPPS->getPicWidthInLumaSamples() - (refConfWindow.getWindowLeftOffset() + refConfWindow.getWindowRightOffset()) * SPS::getWinUnitY(sps->getChromaFormatIdc());
  int refPicHeight = refPPS->getPicHeightInLumaSamples() - (refConfWindow.getWindowTopOffset() + refConfWindow.getWindowBottomOffset()) * SPS::getWinUnitY(sps->getChromaFormatIdc());
#else
  int curPicWidth = curPPS->getPicWidthInLumaSamples();
  int curPicHeight = curPPS->getPicHeightInLumaSamples();
  int refPicWidth = refPPS->getPicWidthInLumaSamples();
  int refPicHeight = refPPS->getPicHeightInLumaSamples();
#endif

  xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
  yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;

  return refPicWidth != curPicWidth || refPicHeight != curPicHeight;
}
#endif

bool CU::isIntra(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTRA;
}

bool CU::isInter(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTER;
}

bool CU::isIBC(const CodingUnit &cu)
{
  return cu.predMode == MODE_IBC;
}

#if JVET_O0119_BASE_PALETTE_444
bool CU::isPLT(const CodingUnit &cu)
{
  return cu.predMode == MODE_PLT;
}
#endif

bool CU::isRDPCMEnabled(const CodingUnit& cu)
{
  return cu.cs->sps->getSpsRangeExtension().getRdpcmEnabledFlag(cu.predMode == MODE_INTRA ? RDPCM_SIGNAL_IMPLICIT : RDPCM_SIGNAL_EXPLICIT);
}

bool CU::isLosslessCoded(const CodingUnit &cu)
{
  return cu.cs->pps->getTransquantBypassEnabledFlag() && cu.transQuantBypass;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return ( cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx() ) && ( cu.tileIdx == cu2.tileIdx );
}

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  uint32_t ctuSizeBit = floorLog2(cu.cs->sps->getMaxCUWidth());

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

bool CU::isLastSubCUOfCtu( const CodingUnit &cu )
{
#if !JVET_O1164_PS
  const SPS &sps      = *cu.cs->sps;
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
  const Area cuAreaY = cu.isSepTree() ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : (const Area&)cu.Y();
#else
  const Area cuAreaY = CS::isDualITree( *cu.cs ) ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : ( const Area& ) cu.Y();
#endif


#if JVET_O1164_PS
  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == cu.cs->pps->getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == cu.cs->pps->getPicHeightInLumaSamples() ) );
#else
  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == sps.getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == sps.getPicHeightInLumaSamples() ) );
#endif
}

uint32_t CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType].lumaPos(), *cu.cs->pcv );
}

int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const CodingStructure &cs = *cu.cs;

  if ( !cu.blocks[cu.chType].x && !( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) && ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType) != NULL ) && CU::isSameSliceAndTile( *cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType), cu ) )
  {
    return ( ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType ) )->qp );
  }
  else
  {
    const int a = ( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType))->qp : prevQP;
    const int b = ( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( -1, 0 ), cu.chType))->qp : prevQP;

    return ( a + b + 1 ) >> 1;
  }
}


uint32_t CU::getNumPUs( const CodingUnit& cu )
{
  uint32_t cnt = 0;
  PredictionUnit *pu = cu.firstPU;

  do
  {
    cnt++;
  } while( ( pu != cu.lastPU ) && ( pu = pu->next ) );

  return cnt;
}

void CU::addPUs( CodingUnit& cu )
{
  cu.cs->addPU( CS::getArea( *cu.cs, cu, cu.chType ), cu.chType );
}


PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  if( depth >= cu.depth ) return CU_DONT_SPLIT;

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if     ( cuSplitType == CU_QUAD_SPLIT    ) return CU_QUAD_SPLIT;

  else if( cuSplitType == CU_HORZ_SPLIT    ) return CU_HORZ_SPLIT;

  else if( cuSplitType == CU_VERT_SPLIT    ) return CU_VERT_SPLIT;

  else if( cuSplitType == CU_TRIH_SPLIT    ) return CU_TRIH_SPLIT;
  else if( cuSplitType == CU_TRIV_SPLIT    ) return CU_TRIV_SPLIT;
  else   { THROW( "Unknown split mode"    ); return CU_QUAD_SPLIT; }
}

#if JVET_O0050_LOCAL_DUAL_TREE
ModeType CU::getModeTypeAtDepth( const CodingUnit& cu, const unsigned depth )
{
  ModeType modeType = ModeType( (cu.modeTypeSeries >> (depth * 3)) & 0x07 );
  CHECK( depth > cu.depth, " depth is wrong" );
  return modeType;
}
#endif

#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
uint32_t CU::getNumNonZeroCoeffNonTs( const CodingUnit& cu, const bool lumaFlag, const bool chromaFlag )
{
  uint32_t count = 0;
  for( auto &currTU : traverseTUs( cu ) )
  {
    count += TU::getNumNonZeroCoeffsNonTS( currTU, lumaFlag, chromaFlag );
  }

  return count;
}
#endif

#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
uint32_t CU::getNumNonZeroCoeffNonTsCorner8x8( const CodingUnit& cu, const bool lumaFlag, const bool chromaFlag )
{
  uint32_t count = 0;
  for( auto &currTU : traverseTUs( cu ) )
  {
    count += TU::getNumNonZeroCoeffsNonTSCorner8x8( currTU, lumaFlag, chromaFlag );
  }

  return count;
}
#endif

bool CU::divideTuInRows( const CodingUnit &cu )
{
  CHECK( cu.ispMode != HOR_INTRA_SUBPARTITIONS && cu.ispMode != VER_INTRA_SUBPARTITIONS, "Intra Subpartitions type not recognized!" );
  return cu.ispMode == HOR_INTRA_SUBPARTITIONS ? true : false;
}

#if !JVET_O0502_ISP_CLEANUP
bool CU::firstTestISPHorSplit( const int width, const int height, const ComponentID compID, const CodingUnit *cuLeft, const CodingUnit *cuAbove )
{
  //this function decides which split mode (horizontal or vertical) is tested first (encoder only)
  //we check the logarithmic aspect ratios of the block
  int aspectRatio = floorLog2(width) - floorLog2(height);
  if( aspectRatio > 0 )
  {
    return true;
  }
  else if( aspectRatio < 0 )
  {
    return false;
  }
  else //if (aspectRatio == 0)
  {
    //we gather data from the neighboring CUs
    const int cuLeftWidth    = cuLeft  != nullptr                                    ? cuLeft->blocks[compID].width   : -1;
    const int cuLeftHeight   = cuLeft  != nullptr                                    ? cuLeft->blocks[compID].height  : -1;
    const int cuAboveWidth   = cuAbove != nullptr                                    ? cuAbove->blocks[compID].width  : -1;
    const int cuAboveHeight  = cuAbove != nullptr                                    ? cuAbove->blocks[compID].height : -1;
    const int cuLeft1dSplit  = cuLeft  != nullptr &&  cuLeft->predMode == MODE_INTRA ? cuLeft->ispMode                :  0;
    const int cuAbove1dSplit = cuAbove != nullptr && cuAbove->predMode == MODE_INTRA ? cuAbove->ispMode               :  0;
    if( cuLeftWidth != -1 && cuAboveWidth == -1 )
    {
      int cuLeftAspectRatio = floorLog2(cuLeftWidth) - floorLog2(cuLeftHeight);
      return cuLeftAspectRatio < 0 ? false : cuLeftAspectRatio > 0 ? true : cuLeft1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
    }
    else if( cuLeftWidth == -1 && cuAboveWidth != -1 )
    {
      int cuAboveAspectRatio = floorLog2(cuAboveWidth) - floorLog2(cuAboveHeight);
      return cuAboveAspectRatio < 0 ? false : cuAboveAspectRatio > 0 ? true : cuAbove1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
    }
    else if( cuLeftWidth != -1 && cuAboveWidth != -1 )
    {
      int cuLeftAspectRatio = floorLog2(cuLeftWidth) - floorLog2(cuLeftHeight);
      int cuAboveAspectRatio = floorLog2(cuAboveWidth) - floorLog2(cuAboveHeight);
      if( cuLeftAspectRatio < 0 && cuAboveAspectRatio < 0 )
      {
        return false;
      }
      else if( cuLeftAspectRatio > 0 && cuAboveAspectRatio > 0 )
      {
        return true;
      }
      else if( cuLeftAspectRatio == 0 && cuAboveAspectRatio == 0 )
      {
        if( cuLeft1dSplit != 0 && cuAbove1dSplit != 0 )
        {
          return cuLeft1dSplit == VER_INTRA_SUBPARTITIONS && cuAbove1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
        }
        else if( cuLeft1dSplit != 0 && cuAbove1dSplit == 0 )
        {
          return cuLeft1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
        }
        else if( cuLeft1dSplit == 0 && cuAbove1dSplit != 0 )
        {
          return cuAbove1dSplit == VER_INTRA_SUBPARTITIONS ? false : true;
        }
        return true;
      }
      else
      {
        return cuLeftAspectRatio > cuAboveAspectRatio ? cuLeftAspectRatio > 0 : cuAboveAspectRatio > 0;
      }
      //return true;
    }
    return true;
  }
}
#endif

PartSplit CU::getISPType( const CodingUnit &cu, const ComponentID compID )
{
  if( cu.ispMode && isLuma( compID ) )
  {
    const bool tuIsDividedInRows = CU::divideTuInRows( cu );

    return tuIsDividedInRows ? TU_1D_HORZ_SPLIT : TU_1D_VERT_SPLIT;
  }
  return TU_NO_ISP;
}

bool CU::isISPLast( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  PartSplit partitionType = CU::getISPType( cu, compID );

  Area originalArea = cu.blocks[compID];
  switch( partitionType )
  {
    case TU_1D_HORZ_SPLIT:
      return tuArea.y + tuArea.height == originalArea.y + originalArea.height;
    case TU_1D_VERT_SPLIT:
      return tuArea.x + tuArea.width == originalArea.x + originalArea.width;
    default:
      THROW( "Unknown ISP processing order type!" );
      return false;
  }
}

bool CU::isISPFirst( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  return tuArea == cu.firstTU->blocks[compID];
}

bool CU::canUseISP( const CodingUnit &cu, const ComponentID compID )
{
  const int width     = cu.blocks[compID].width;
  const int height    = cu.blocks[compID].height;
#if MAX_TB_SIZE_SIGNALLING
  const int maxTrSize = cu.cs->sps->getMaxTbSize();
#else
  const int maxTrSize = MAX_TB_SIZEY;
#endif
  return CU::canUseISP( width, height, maxTrSize );
}

bool CU::canUseISP( const int width, const int height, const int maxTrSize )
{
  bool  notEnoughSamplesToSplit = ( floorLog2(width) + floorLog2(height) <= ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  bool  cuSizeLargerThanMaxTrSize = width > maxTrSize || height > maxTrSize;
  if ( notEnoughSamplesToSplit || cuSizeLargerThanMaxTrSize )
  {
    return false;
  }
  return true;
}

uint32_t CU::getISPSplitDim( const int width, const int height, const PartSplit ispType )
{
  bool divideTuInRows = ispType == TU_1D_HORZ_SPLIT;
  uint32_t splitDimensionSize, nonSplitDimensionSize, partitionSize, divShift = 2;

  if( divideTuInRows )
  {
    splitDimensionSize    = height;
    nonSplitDimensionSize = width;
  }
  else
  {
    splitDimensionSize    = width;
    nonSplitDimensionSize = height;
  }

  const int minNumberOfSamplesPerCu = 1 << ( ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  const int factorToMinSamples = nonSplitDimensionSize < minNumberOfSamplesPerCu ? minNumberOfSamplesPerCu >> floorLog2(nonSplitDimensionSize) : 1;
  partitionSize = ( splitDimensionSize >> divShift ) < factorToMinSamples ? factorToMinSamples : ( splitDimensionSize >> divShift );

  CHECK( floorLog2(partitionSize) + floorLog2(nonSplitDimensionSize) < floorLog2(minNumberOfSamplesPerCu), "A partition has less than the minimum amount of samples!" );
  return partitionSize;
}

#if JVET_O0502_ISP_CLEANUP
bool CU::allLumaCBFsAreZero(const CodingUnit& cu)
{
  if (!cu.ispMode)
  {
    return TU::getCbf(*cu.firstTU, COMPONENT_Y) == false;
  }
  else
  {
    int numTotalTUs = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(cu.firstTU->lheight()) : cu.lwidth() >> floorLog2(cu.firstTU->lwidth());
    TransformUnit* tuPtr = cu.firstTU;
    for (int tuIdx = 0; tuIdx < numTotalTUs; tuIdx++)
    {
      if (TU::getCbf(*tuPtr, COMPONENT_Y) == true)
      {
        return false;
      }
      tuPtr = tuPtr->next;
    }
    return true;
  }
}
#endif


PUTraverser CU::traversePUs( CodingUnit& cu )
{
  return PUTraverser( cu.firstPU, cu.lastPU->next );
}

TUTraverser CU::traverseTUs( CodingUnit& cu )
{
  return TUTraverser( cu.firstTU, cu.lastTU->next );
}

cPUTraverser CU::traversePUs( const CodingUnit& cu )
{
  return cPUTraverser( cu.firstPU, cu.lastPU->next );
}

cTUTraverser CU::traverseTUs( const CodingUnit& cu )
{
  return cTUTraverser( cu.firstTU, cu.lastTU->next );
}

// PU tools

int PU::getIntraMPMs( const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/ )
{
  const int numMPMs = NUM_MOST_PROBABLE_MODES;
  {
    CHECK(channelType != CHANNEL_TYPE_LUMA, "Not harmonized yet");
    int numCand      = -1;
    int leftIntraDir = PLANAR_IDX, aboveIntraDir = PLANAR_IDX;

    const CompArea &area = pu.block(getFirstComponentOfChannel(channelType));
    const Position posRT = area.topRight();
    const Position posLB = area.bottomLeft();

    // Get intra direction of left PU
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType);
    if (puLeft && CU::isIntra(*puLeft->cu))
    {
      leftIntraDir = PU::getIntraDirLuma( *puLeft );
    }

    // Get intra direction of above PU
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType);
    if (puAbove && CU::isIntra(*puAbove->cu) && CU::isSameCtu(*pu.cu, *puAbove->cu))
    {
      aboveIntraDir = PU::getIntraDirLuma( *puAbove );
    }

    CHECK(2 >= numMPMs, "Invalid number of most probable modes");

    const int offset = (int)NUM_LUMA_MODE - 6;
    const int mod = offset + 3;

    {
      mpm[0] = PLANAR_IDX;
      mpm[1] = DC_IDX;
      mpm[2] = VER_IDX;
      mpm[3] = HOR_IDX;
      mpm[4] = VER_IDX - 4;
      mpm[5] = VER_IDX + 4;

      if (leftIntraDir == aboveIntraDir)
      {
        numCand = 1;
        if (leftIntraDir > DC_IDX)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = ((leftIntraDir + offset) % mod) + 2;
          mpm[3] = ((leftIntraDir - 1) % mod) + 2;
#if JVET_O0925_MIP_SIMPLIFICATIONS
          mpm[4] = ((leftIntraDir + offset - 1) % mod) + 2;
          mpm[5] = ( leftIntraDir               % mod) + 2;
#else
          mpm[4] = DC_IDX;
          mpm[5] = ((leftIntraDir + offset - 1) % mod) + 2;
#endif
        }
      }
      else //L!=A
      {
        numCand = 2;
        int  maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;

        if ((leftIntraDir > DC_IDX) && (aboveIntraDir > DC_IDX))
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = aboveIntraDir;
          maxCandModeIdx = mpm[1] > mpm[2] ? 1 : 2;
          int minCandModeIdx = mpm[1] > mpm[2] ? 2 : 1;
#if JVET_O0925_MIP_SIMPLIFICATIONS
          if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 1)
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset)     % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
            mpm[5] = ((mpm[minCandModeIdx] + offset - 1) % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] >= 62)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ( mpm[minCandModeIdx]           % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 2)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] - 1)      % mod) + 2;
          }
          else
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
          }
#else
          mpm[3] = DC_IDX;
          if ((mpm[maxCandModeIdx] - mpm[minCandModeIdx] < 63) && (mpm[maxCandModeIdx] - mpm[minCandModeIdx] > 1))
          {
            mpm[4] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] - 1) % mod) + 2;
          }
          else
          {
            mpm[4] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx]) % mod) + 2;
          }
#endif
        }
        else if (leftIntraDir + aboveIntraDir >= 2)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = (leftIntraDir < aboveIntraDir) ? aboveIntraDir : leftIntraDir;
          maxCandModeIdx = 1;
#if JVET_O0925_MIP_SIMPLIFICATIONS
          mpm[2] = ((mpm[maxCandModeIdx] + offset)     % mod) + 2;
          mpm[3] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
          mpm[5] = ( mpm[maxCandModeIdx]               % mod) + 2;
#else
          mpm[2] = DC_IDX;
          mpm[3] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] - 1) % mod) + 2;
          mpm[5] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
#endif
        }
      }
    }
    for (int i = 0; i < numMPMs; i++)
    {
      CHECK(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
    }
    CHECK(numCand == 0, "No candidates found");
    return numCand;
  }
}

bool PU::isMIP(const PredictionUnit &pu, const ChannelType &chType)
{
  return (chType == CHANNEL_TYPE_LUMA && pu.cu->mipFlag);
}

int PU::getMipSizeId(const PredictionUnit &pu)
{
  if ((pu.lwidth() == 4) && (pu.lheight() == 4))
  {
    return 0; // MIP with 16x4 matrix
  }
  else if (pu.lwidth() <= 8 && pu.lheight() <= 8)
  {
    return 1; // MIP with 16x8 matrix
  }
  else
  {
    return 2; // MIP with 64x8 matrix
  }
}

#if !JVET_O0925_MIP_SIMPLIFICATIONS
int PU::getMipMPMs(const PredictionUnit &pu, unsigned *mpm)
{
  const CompArea &area = pu.block( getFirstComponentOfChannel( CHANNEL_TYPE_LUMA ) );
  const Position &pos = area.pos();

  bool realMode = false;

  // Get intra mode of left PU
  int leftIntraMode = -1;
  const PredictionUnit *puLeft = pu.cs->getPURestricted( pos.offset( -1, 0 ), pu, CHANNEL_TYPE_LUMA );

  if( puLeft && CU::isIntra( *puLeft->cu ) )
  {
    if( PU::isMIP( *puLeft ) )
    {
      if (getMipSizeId(*puLeft) == getMipSizeId(pu))
      {
        leftIntraMode = puLeft->intraDir[CHANNEL_TYPE_LUMA];
        realMode = true;
      }
    }
    else
    {
      leftIntraMode = g_mapAngular33ToMip[getMipSizeId(pu)][g_intraMode65to33AngMapping[puLeft->intraDir[CHANNEL_TYPE_LUMA]]];
    }
  }

  // Get intra mode of above PU
  int aboveIntraMode = -1;
  const PredictionUnit *puAbove = pu.cs->getPURestricted( pos.offset( 0, -1 ), pu, CHANNEL_TYPE_LUMA );

  if( puAbove && CU::isIntra( *puAbove->cu ) && CU::isSameCtu(*pu.cu, *puAbove->cu) )
  {
    if( PU::isMIP( *puAbove ) )
    {
      if (getMipSizeId(*puAbove) == getMipSizeId(pu))
      {
        aboveIntraMode = puAbove->intraDir[CHANNEL_TYPE_LUMA];
        realMode = true;
      }
    }
    else
    {
      aboveIntraMode = g_mapAngular33ToMip[getMipSizeId(pu)][g_intraMode65to33AngMapping[puAbove->intraDir[CHANNEL_TYPE_LUMA]]];
    }
  }

  // derive MPMs
  CHECKD(NUM_MPM_MIP != 3, "Error: wrong number of MPMs for MIP");

  const int* modeList = g_sortedMipMpms[getMipSizeId(pu)];

  int numCand = 0;
  if( leftIntraMode == aboveIntraMode )
  {
    if( leftIntraMode > -1 )
    {
      mpm[0] = leftIntraMode;
      numCand = 1;

      if( leftIntraMode != modeList[0] )
      {
        mpm[1] = modeList[0];
        mpm[2] = (leftIntraMode != modeList[1]) ? modeList[1] : modeList[2];
      }
      else
      {
        mpm[1] = modeList[1];
        mpm[2] = modeList[2];
      }
    }
    else
    {
      mpm[0] = modeList[0];
      mpm[1] = modeList[1];
      mpm[2] = modeList[2];
    }
  }
  else
  {
    if( leftIntraMode > -1 && aboveIntraMode > -1 )
    {
      mpm[0] = leftIntraMode;
      mpm[1] = aboveIntraMode;
      numCand = 2;

      int index = 0;
      for( int i = 0; i < 3; i++ )
      {
        if( (leftIntraMode != modeList[i]) && (aboveIntraMode != modeList[i]) )
        {
          index = i;
          break;
        }
      }
      CHECK( index > 2, "Error" );
      mpm[2] = modeList[index];
    }
    else
    {
      mpm[0] = leftIntraMode > -1 ? leftIntraMode : aboveIntraMode;
      numCand = 1;

      if( mpm[0] != modeList[0] )
      {
        mpm[1] = modeList[0];
        mpm[2] = (mpm[0] != modeList[1]) ? modeList[1] : modeList[2];
      }
      else
      {
        mpm[1] = modeList[1];
        mpm[2] = modeList[2];
      }
    }
  }

  return (realMode ? numCand : 0);
}
#endif

uint32_t PU::getIntraDirLuma( const PredictionUnit &pu )
{
  if (isMIP(pu))
  {
#if JVET_O0925_MIP_SIMPLIFICATIONS
    return PLANAR_IDX;
#else
    return g_mapMipToAngular65[getMipSizeId(pu)][pu.intraDir[CHANNEL_TYPE_LUMA]];
#endif
  }
  else
  {
    return pu.intraDir[CHANNEL_TYPE_LUMA];
  }
}

#if !JVET_O0925_MIP_SIMPLIFICATIONS
AvailableInfo PU::getAvailableInfoLuma(const PredictionUnit &pu)
{
  const Area puArea = pu.Y();
  const CodingStructure &cs = *pu.cs;
  CHECK(cs.pps->getConstrainedIntraPred(), "Error: constrained intra prediction not supported");

  AvailableInfo availInfo(0, 0);

  // above
  const int unitWidth = cs.pcv->minCUWidth;
  const int numAboveUnits = (puArea.width + (unitWidth - 1)) / unitWidth;
  for (int uX = 0; uX < numAboveUnits; uX++)
  {
    const Position topPos = puArea.offset(availInfo.maxPosTop, -1);
    const CodingUnit* pcCUAbove = cs.isDecomp(topPos, CHANNEL_TYPE_LUMA) ? cs.getCURestricted(topPos, *(pu.cu), CHANNEL_TYPE_LUMA) : nullptr;
    if (!pcCUAbove) { break; }

    availInfo.maxPosTop += unitWidth;
  }

  // left
  const int unitHeight = cs.pcv->minCUHeight;
  const int numLeftUnits = (puArea.height + (unitHeight - 1)) / unitHeight;
  for (int uY = 0; uY < numLeftUnits; uY++)
  {
    const Position leftPos = puArea.offset(-1, availInfo.maxPosLeft);
    const CodingUnit* pcCULeft = cs.isDecomp(leftPos, CHANNEL_TYPE_LUMA) ? cs.getCURestricted(leftPos, *(pu.cu), CHANNEL_TYPE_LUMA) : nullptr;
    if (!pcCULeft) { break; }

    availInfo.maxPosLeft += unitHeight;
  }

  CHECKD(availInfo.maxPosTop > puArea.width || availInfo.maxPosLeft > puArea.height, "Error");
  return availInfo;
}
#endif

void PU::getIntraChromaCandModes( const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE] )
{
  {
    modeList[  0 ] = PLANAR_IDX;
    modeList[  1 ] = VER_IDX;
    modeList[  2 ] = HOR_IDX;
    modeList[  3 ] = DC_IDX;
    modeList[4] = LM_CHROMA_IDX;
    modeList[5] = MDLM_L_IDX;
    modeList[6] = MDLM_T_IDX;
    modeList[7] = DM_CHROMA_IDX;

#if JVET_O0219_LFNST_TRANSFORM_SET_FOR_LMCMODE
    const uint32_t lumaMode = getCoLocatedIntraLumaMode(pu);
#else
    Position topLeftPos = pu.blocks[pu.chType].lumaPos();
    Position refPos = topLeftPos.offset( pu.blocks[pu.chType].lumaSize().width >> 1, pu.blocks[pu.chType].lumaSize().height >> 1 );
#if JVET_O0050_LOCAL_DUAL_TREE
    const PredictionUnit *lumaPU = pu.cu->isSepTree() ? pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : &pu;
#else
    const PredictionUnit *lumaPU = CS::isDualITree( *pu.cs ) ? pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : &pu;
#endif
    const uint32_t lumaMode = PU::getIntraDirLuma( *lumaPU );
#endif
    for( int i = 0; i < 4; i++ )
    {
      if( lumaMode == modeList[i] )
      {
        modeList[i] = VDIA_IDX;
        break;
      }
    }
  }
}

bool PU::isLMCMode(unsigned mode)
{
  return (mode >= LM_CHROMA_IDX && mode <= MDLM_T_IDX);
}

bool PU::isLMCModeEnabled(const PredictionUnit &pu, unsigned mode)
{
#if JVET_O1124_ALLOW_CCLM_COND
  if ( pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed() )
#else
  if ( pu.cs->sps->getUseLMChroma() )
#endif
  {
    return true;
  }
  return false;
}

#if JVET_O1153_INTRA_CHROMAMODE_CODING
int PU::getLMSymbolList(const PredictionUnit &pu, int *modeList)
{
  int idx = 0;

  modeList[idx++] = LM_CHROMA_IDX;
  modeList[idx++] = MDLM_L_IDX;
  modeList[idx++] = MDLM_T_IDX;
  return idx;
}
#else
int PU::getLMSymbolList(const PredictionUnit &pu, int *modeList)
{
  int idx = 0;

  modeList[idx++] = LM_CHROMA_IDX;
  modeList[idx++] = -1;
  modeList[idx++] = MDLM_L_IDX;
  modeList[idx++] = MDLM_T_IDX;
  return idx;
}
#endif

bool PU::isChromaIntraModeCrossCheckMode( const PredictionUnit &pu )
{
  return pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX;
}

uint32_t PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
  uint32_t uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
#if JVET_O0219_LFNST_TRANSFORM_SET_FOR_LMCMODE
    uiIntraMode = getCoLocatedIntraLumaMode(pu);
#else
    Position topLeftPos = pu.blocks[pu.chType].lumaPos();
    Position refPos = topLeftPos.offset( pu.blocks[pu.chType].lumaSize().width >> 1, pu.blocks[pu.chType].lumaSize().height >> 1 );
#if JVET_O0050_LOCAL_DUAL_TREE
    const PredictionUnit &lumaPU = pu.cu->isSepTree() ? *pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : *pu.cs->getPU( topLeftPos, CHANNEL_TYPE_LUMA );
#else
    const PredictionUnit &lumaPU = CS::isDualITree( *pu.cs ) ? *pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : *pu.cs->getPU( topLeftPos, CHANNEL_TYPE_LUMA );
#endif

    uiIntraMode = PU::getIntraDirLuma( lumaPU );
#endif
  }
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) && uiIntraMode < NUM_LUMA_MODE ) // map directional, planar and dc
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
  return uiIntraMode;
}

#if JVET_O0219_LFNST_TRANSFORM_SET_FOR_LMCMODE
uint32_t PU::getCoLocatedIntraLumaMode( const PredictionUnit &pu )
{
  Position topLeftPos = pu.blocks[pu.chType].lumaPos();
  Position refPos = topLeftPos.offset( pu.blocks[pu.chType].lumaSize().width >> 1, pu.blocks[pu.chType].lumaSize().height >> 1 );
#if JVET_O0050_LOCAL_DUAL_TREE
  const PredictionUnit &lumaPU = pu.cu->isSepTree() ? *pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : *pu.cs->getPU( topLeftPos, CHANNEL_TYPE_LUMA );
#else
  const PredictionUnit &lumaPU = CS::isDualITree( *pu.cs ) ? *pu.cs->picture->cs->getPU( refPos, CHANNEL_TYPE_LUMA ) : *pu.cs->getPU( topLeftPos, CHANNEL_TYPE_LUMA );
#endif

  return PU::getIntraDirLuma( lumaPU );
}
#endif

int PU::getWideAngIntraMode( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID )
{
  if( dirMode < 2 )
  {
    return ( int ) dirMode;
  }

  CodingStructure& cs           = *tu.cs;
  const CompArea&  area         = tu.blocks[ compID ];
  PelBuf           pred         = cs.getPredBuf( area );
  int              width        = int( pred.width );
  int              height       = int( pred.height );
  int              modeShift[ ] = { 0, 6, 10, 12, 14, 15 };
  int              deltaSize    = abs( floorLog2( width ) - floorLog2( height ) );
  int              predMode     = dirMode;

  if( width > height && dirMode < 2 + modeShift[ deltaSize ] )
  {
    predMode += ( VDIA_IDX - 1 );
  }
  else if( height > width && predMode > VDIA_IDX - modeShift[ deltaSize ] )
  {
    predMode -= ( VDIA_IDX + 1 );
  }

  return predMode;
}

bool PU::xCheckSimilarMotion(const int mergeCandIndex, const int prevCnt, const MergeCtx mergeCandList, bool hasPruned[MRG_MAX_NUM_CANDS])
{
  for (uint32_t ui = 0; ui < prevCnt; ui++)
  {
    if (hasPruned[ui])
    {
      continue;
    }
    if (mergeCandList.interDirNeighbours[ui] == mergeCandList.interDirNeighbours[mergeCandIndex])
    {
      if (mergeCandList.interDirNeighbours[ui] == 3)
      {
        int offset0 = (ui * 2);
        int offset1 = (mergeCandIndex * 2);
        if (mergeCandList.mvFieldNeighbours[offset0].refIdx == mergeCandList.mvFieldNeighbours[offset1].refIdx &&
            mergeCandList.mvFieldNeighbours[offset0 + 1].refIdx == mergeCandList.mvFieldNeighbours[offset1 + 1].refIdx &&
            mergeCandList.mvFieldNeighbours[offset0].mv == mergeCandList.mvFieldNeighbours[offset1].mv &&
            mergeCandList.mvFieldNeighbours[offset0 + 1].mv == mergeCandList.mvFieldNeighbours[offset1 + 1].mv
          )
        {
          hasPruned[ui] = true;
          return true;
        }
      }
      else
      {
        int offset0 = (ui * 2) + mergeCandList.interDirNeighbours[ui] - 1;
        int offset1 = (mergeCandIndex * 2) + mergeCandList.interDirNeighbours[ui] - 1;
        if (mergeCandList.mvFieldNeighbours[offset0].refIdx == mergeCandList.mvFieldNeighbours[offset1].refIdx &&
            mergeCandList.mvFieldNeighbours[offset0].mv == mergeCandList.mvFieldNeighbours[offset1].mv
          )
        {
          hasPruned[ui] = true;
          return true;
        }
      }
    }
  }

  return false;
}

#if JVET_L0090_PAIR_AVG
bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
  , bool ibcFlag
  , bool isShared
)
#else
bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, bool isCandInter[MRG_MAX_NUM_CANDS], bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
  , int mmvdList
)
#endif
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;
  bool hasPruned[MRG_MAX_NUM_CANDS];
  memset(hasPruned, 0, MRG_MAX_NUM_CANDS * sizeof(bool));
  if (isAvailableSubPu)
  {
    hasPruned[subPuMvpPos] = true;
  }
#if JVET_O0078_SINGLE_HMVPLUT
  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
#else
  auto &lut = ibcFlag ? ( isShared ? cs.motionLut.lutShareIbc : cs.motionLut.lutIbc ) : cs.motionLut.lut;
#endif
  int num_avai_candInLUT = (int) lut.size();

  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];
    mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
#if JVET_O0057_ALTHPELIF
    mrgCtx.useAltHpelIf[cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
#endif
    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
    }
    if (mrgIdx > 2 || (mrgIdx > 1 && ibcFlag) || !xCheckSimilarMotion(cnt, prevCnt, mrgCtx, hasPruned))
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? miNeighbor.GBiIdx : GBI_DEFAULT;
      if (mrgCandIdx == cnt && canFastExit)
      {
        return true;
      }
      cnt ++;
      if (cnt  == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }
#if JVET_O0057_ALTHPELIF
  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }
#endif
  return false;
}

void PU::getIBCMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
#if JVET_O0455_IBC_MAX_MERGE_NUM
  const uint32_t maxNumMergeCand = slice.getMaxNumIBCMergeCand();
#else
  const uint32_t maxNumMergeCand = slice.getMaxNumMergeCand();
#endif
  const bool canFastExit = pu.cs->pps->getLog2ParallelMergeLevelMinus2() == 0;

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.GBiIdx[ui] = GBI_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours[ui] = MRG_TYPE_IBC;
    mrgCtx.mvFieldNeighbours[ui * 2].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui * 2 + 1].refIdx = NOT_VALID;
#if JVET_O0057_ALTHPELIF
    mrgCtx.useAltHpelIf[ui] = false;
#endif
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posRT = pu.shareParentPos.offset(pu.shareParentSize.width - 1, 0);
  const Position posLB = pu.shareParentPos.offset(0, pu.shareParentSize.height - 1);

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  const bool isAvailableA1 = puLeft && isDiffMER(pu, *puLeft) && pu.cu != puLeft->cu && CU::isIBC(*puLeft->cu);
  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
    if (mrgCandIdx == cnt && canFastExit)
    {
      return;
    }
    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }


  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool isAvailableB1 = puAbove && isDiffMER(pu, *puAbove) && pu.cu != puAbove->cu && CU::isIBC(*puAbove->cu);
  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    if (!isAvailableA1 || (miAbove != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Above
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int spatialCandPos = cnt;


  int maxNumMergeCandMin1 = maxNumMergeCand;
  if (cnt != maxNumMergeCandMin1)
  {
    bool isAvailableSubPu = false;
    unsigned subPuMvpPos = 0;

    bool  isShared = ((pu.Y().lumaSize().width != pu.shareParentSize.width) || (pu.Y().lumaSize().height != pu.shareParentSize.height));

#if JVET_L0090_PAIR_AVG
    bool bFound = addMergeHMVPCand(cs, mrgCtx, canFastExit
      , mrgCandIdx
      , maxNumMergeCandMin1, cnt
      , spatialCandPos
      , isAvailableSubPu, subPuMvpPos
      , true
      , isShared
    );
#else
    bool bFound = addMergeHMVPCand(slice, mrgCtx, isCandInter, canFastExit
      , mrgCandIdx
      , maxNumMergeCandMin1, cnt, cnt, isAvailableSubPu, subPuMvpPos
    );
#endif
    if (bFound)
    {
      return;
    }
  }


    while (cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), MAX_NUM_REF);
      mrgCtx.interDirNeighbours[cnt] = 1;
      cnt++;
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }
    }

  mrgCtx.numValidMergeCand = cnt;

}

void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx,
                                 int mmvdList,
                                 const int& mrgCandIdx )
{
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
  const uint32_t maxNumMergeCand = slice.getMaxNumMergeCand();
  const bool canFastExit     = pu.cs->pps->getLog2ParallelMergeLevelMinus2() == 0;

#if !JVET_L0090_PAIR_AVG
  // this variable is unused if remove HEVC combined candidates
  bool isCandInter[MRG_MAX_NUM_CANDS];
#endif

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
#if !JVET_L0090_PAIR_AVG
    isCandInter[ui] = false;
#endif
    mrgCtx.GBiIdx[ui] = GBI_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours [ui] = MRG_TYPE_DEFAULT_N;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
#if JVET_O0057_ALTHPELIF
    mrgCtx.useAltHpelIf[ui] = false;
#endif
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );

  const bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );

  if( isAvailableA1 )
  {
    miLeft = puLeft->getMotionInfo( posLB.offset(-1, 0) );

#if !JVET_L0090_PAIR_AVG
    isCandInter[cnt] = true;
#endif

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
#if JVET_O0057_ALTHPELIF
    mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
#endif
    mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->GBiIdx : GBI_DEFAULT;
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
    }
    if (mrgCandIdx == cnt && canFastExit)
    {
      return;
    }

    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }


  // above
  const PredictionUnit *puAbove = cs.getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );

  bool isAvailableB1 = puAbove && isDiffMER( pu, *puAbove ) && pu.cu != puAbove->cu && CU::isInter( *puAbove->cu );

  if( isAvailableB1 )
  {
    miAbove = puAbove->getMotionInfo( posRT.offset( 0, -1 ) );

    if( !isAvailableA1 || ( miAbove != miLeft ) )
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
#if JVET_O0057_ALTHPELIF
      mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
#endif
      // get Mv from Above
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->GBiIdx : GBI_DEFAULT;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAbove.mv[0], miAbove.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAbove.mv[1], miAbove.refIdx[1] );
      }
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  int spatialCandPos = cnt;

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );

  bool isAvailableB0 = puAboveRight && isDiffMER( pu, *puAboveRight ) && CU::isInter( *puAboveRight->cu );

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

    if( !isAvailableB1 || ( miAbove != miAboveRight ) )
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
#if JVET_O0057_ALTHPELIF
      mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
#endif
      // get Mv from Above-right
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->GBiIdx : GBI_DEFAULT;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
      }

      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );

  bool isAvailableA0 = puLeftBottom && isDiffMER( pu, *puLeftBottom ) && CU::isInter( *puLeftBottom->cu );

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )
    {
#if !JVET_L0090_PAIR_AVG
      isCandInter[cnt] = true;
#endif

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
#if JVET_O0057_ALTHPELIF
      mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
#endif
      mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->GBiIdx : GBI_DEFAULT;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
      }

      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }


  // above left
  if ( cnt < 4 )
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

    bool isAvailableB2 = puAboveLeft && isDiffMER( pu, *puAboveLeft ) && CU::isInter( *puAboveLeft->cu );

    if( isAvailableB2 )
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )
      {
#if !JVET_L0090_PAIR_AVG
        isCandInter[cnt] = true;
#endif

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
#if JVET_O0057_ALTHPELIF
        mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
#endif
        mrgCtx.GBiIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->GBiIdx : GBI_DEFAULT;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
        }

        if (mrgCandIdx == cnt && canFastExit)
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (slice.getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset( -3, -3 );
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool C0Avail = false;
    if (((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight))
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx, false ) )
                              || getColocatedMVP( pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx, false );
    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx, false ) )
                   || getColocatedMVP( pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx, false );
      if (bExistMV)
      {
        dir     |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if( dir != 0 )
    {
      bool addTMvp = true;
      if( addTMvp )
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
#if !JVET_L0090_PAIR_AVG
        isCandInter              [uiArrayAddr] = true;
#endif
        mrgCtx.GBiIdx[uiArrayAddr] = GBI_DEFAULT;
#if JVET_O0057_ALTHPELIF
        mrgCtx.useAltHpelIf[uiArrayAddr] = false;
#endif
        if (mrgCandIdx == cnt && canFastExit)
        {
          return;
        }

        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
  if (cnt != maxNumMergeCandMin1)
  {
    bool isAvailableSubPu = false;
    unsigned subPuMvpPos = 0;
#if JVET_L0090_PAIR_AVG
    bool isShared = false;
    bool bFound = addMergeHMVPCand(cs, mrgCtx, canFastExit
      , mrgCandIdx
      , maxNumMergeCandMin1, cnt
      , spatialCandPos
      , isAvailableSubPu, subPuMvpPos
      , CU::isIBC(*pu.cu)
      , isShared
    );
#else
    bool bFound = addMergeHMVPCand(slice, mrgCtx, isCandInter, canFastExit
      , (mmvdList != 0 && mrgCandIdx != -1) ? (const int)mrgCandIdxIBC : mrgCandIdx
      , maxNumMergeCandMin1, cnt, cnt, isAvailableSubPu, subPuMvpPos
      , mmvdList
    );
#endif
    if (bFound)
    {
      return;
    }
  }

#if JVET_L0090_PAIR_AVG
  // pairwise-average candidates
  {

    if (cnt > 1 && cnt < maxNumMergeCand)
    {

      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField( Mv( 0, 0 ), NOT_VALID );
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField( Mv( 0, 0 ), NOT_VALID );
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;


#if JVET_O0057_ALTHPELIF
      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
#endif
      for( int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++ )
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

        // both MVs are invalid, skip
        if( (refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID) )
        {
          continue;
        }

        interDir += 1 << refListId;
        // both MVs are valid, average these two MVs
        if( (refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID) )
        {
          const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

          // average two MVs
          Mv avgMv = MvI;
          avgMv += MvJ;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);

          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( avgMv, refIdxI );
        }
        // only one MV is valid, take the only one MV
        else if( refIdxI != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxI );
        }
        else if( refIdxJ != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxJ );
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if( interDir > 0 )
      {
        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }
#endif

  uint32_t uiArrayAddr = cnt;
#if !JVET_L0090_PAIR_AVG
  uint32_t uiCutoff    = std::min( uiArrayAddr, 3u );
  if (slice.isInterB())
  {
    static const uint32_t NUM_PRIORITY_LIST = 12;
    static const uint32_t uiPriorityList0[NUM_PRIORITY_LIST] = { 0 , 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3 };
    static const uint32_t uiPriorityList1[NUM_PRIORITY_LIST] = { 1 , 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2 };

    for (int idx = 0; idx < uiCutoff * (uiCutoff - 1) && uiArrayAddr != maxNumMergeCand; idx++)
    {
      CHECK( idx >= NUM_PRIORITY_LIST, "Invalid priority list number" );
      int i = uiPriorityList0[idx];
      int j = uiPriorityList1[idx];
      if (isCandInter[i] && isCandInter[j] && (mrgCtx.interDirNeighbours[i] & 0x1) && (mrgCtx.interDirNeighbours[j] & 0x2))
      {
        isCandInter[uiArrayAddr] = true;
        mrgCtx.interDirNeighbours[uiArrayAddr] = 3;
        mrgCtx.GBiIdx[uiArrayAddr] = ((mrgCtx.interDirNeighbours[uiArrayAddr] == 3)) ? CU::deriveGbiIdx(mrgCtx.GBiIdx[i], mrgCtx.GBiIdx[j]) : GBI_DEFAULT;

        // get Mv from cand[i] and cand[j]
        mrgCtx.mvFieldNeighbours[ uiArrayAddr << 1     ].setMvField(mrgCtx.mvFieldNeighbours[ i << 1     ].mv, mrgCtx.mvFieldNeighbours[ i << 1     ].refIdx);
        mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].setMvField(mrgCtx.mvFieldNeighbours[(j << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(j << 1) + 1].refIdx);

        int iRefPOCL0 = slice.getRefPOC(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1)    ].refIdx);
        int iRefPOCL1 = slice.getRefPOC(REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[(uiArrayAddr << 1) + 1].refIdx);

        if( iRefPOCL0 == iRefPOCL1 && mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 )].mv == mrgCtx.mvFieldNeighbours[( uiArrayAddr << 1 ) + 1].mv )
        {
          isCandInter[uiArrayAddr] = false;
        }
        else
        {
          uiArrayAddr++;
        }
      }
    }
  }

  // early termination
  if (uiArrayAddr == maxNumMergeCand)
  {
    return;
  }
#endif

  int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);

  int r = 0;
  int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)
  {
#if !JVET_L0090_PAIR_AVG
    isCandInter               [uiArrayAddr     ] = true;
#endif
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.GBiIdx             [uiArrayAddr     ] = GBI_DEFAULT;
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);
#if JVET_O0057_ALTHPELIF
    mrgCtx.useAltHpelIf[uiArrayAddr] = false;
#endif

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    if ( mrgCtx.interDirNeighbours[uiArrayAddr] == 1 && pu.cs->slice->getRefPic(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[uiArrayAddr << 1].refIdx)->getPOC() == pu.cs->slice->getPOC())
    {
      mrgCtx.mrgTypeNeighbours[uiArrayAddr] = MRG_TYPE_IBC;
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  mrgCtx.numValidMergeCand = uiArrayAddr;
}
bool PU::checkDMVRCondition(const PredictionUnit& pu)
{
  WPScalingParam *wp0;
  WPScalingParam *wp1;
  int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
  int refIdx1 = pu.refIdx[REF_PIC_LIST_1];
  pu.cu->slice->getWpScaling(REF_PIC_LIST_0, refIdx0, wp0);
  pu.cu->slice->getWpScaling(REF_PIC_LIST_1, refIdx1, wp1);
#if JVET_O1140_SLICE_DISABLE_BDOF_DMVR_FLAG
  if (pu.cs->sps->getUseDMVR() && (!pu.cs->slice->getDisBdofDmvrFlag()))
#else
  if (pu.cs->sps->getUseDMVR())
#endif
  {
    return pu.mergeFlag
      && pu.mergeType == MRG_TYPE_DEFAULT_N
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
      && !pu.mhIntraFlag
#endif
      && !pu.cu->affine
      && !pu.mmvdMergeFlag
      && !pu.cu->mmvdSkip
      && PU::isBiPredFromDifferentDirEqDistPoc(pu)
      && (pu.lheight() >= 8)
      && (pu.lwidth() >= 8)
      && ((pu.lheight() * pu.lwidth()) >= 128)
      && (pu.cu->GBiIdx == GBI_DEFAULT)
      && ((!wp0[COMPONENT_Y].bPresentFlag) && (!wp1[COMPONENT_Y].bPresentFlag))
#if JVET_O1164_RPR
      && PU::isRefPicSameSize( pu )
#endif
      ;
  }
  else
  {
    return false;
  }
}

#if !JVET_O1170_IBC_VIRTUAL_BUFFER
// for ibc pu validation
bool PU::isBlockVectorValid(PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xStartInCU, int yStartInCU, int xBv, int yBv, int ctuSize)
{
  const int ctuSizeLog2 = floorLog2(ctuSize);

  int refRightX = xPos + xBv + width - 1;
  int refBottomY = yPos + yBv + height - 1;

  int refLeftX = xPos + xBv;
  int refTopY = yPos + yBv;

  if ((xPos + xBv) < 0)
  {
    return false;
  }
  if (refRightX >= picWidth)
  {
    return false;
  }

  if ((yPos + yBv) < 0)
  {
    return false;
  }
  if (refBottomY >= picHeight)
  {
    return false;
  }
  if ((xBv + width) > 0 && (yBv + height) > 0)
  {
    return false;
  }

  // cannot be in the above CTU row
  if (refTopY >> ctuSizeLog2 < yPos >> ctuSizeLog2)
    return false;

  // cannot be in the below CTU row
  if (refBottomY >> ctuSizeLog2 > yPos >> ctuSizeLog2)
  {
    return false;
  }

  // in the same CTU line
  int numLeftCTUs = (1 << ((7 - ctuSizeLog2) << 1)) - ((ctuSizeLog2 < 7) ? 1 : 0);
  if ((refRightX >> ctuSizeLog2 <= xPos >> ctuSizeLog2) && (refLeftX >> ctuSizeLog2 >= (xPos >> ctuSizeLog2) - numLeftCTUs))
  {

    // in the same CTU, or left CTU
    // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
    if (((refLeftX >> ctuSizeLog2) == ((xPos >> ctuSizeLog2) - 1)) && (ctuSizeLog2 == 7))
    {
      // ref block's collocated block in current CTU
      const Position refPosCol = pu.Y().topLeft().offset(xBv + ctuSize, yBv);
      int offset64x = (refPosCol.x >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      int offset64y = (refPosCol.y >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      const Position refPosCol64x64 = {offset64x, offset64y};
      if (pu.cs->isDecomp(refPosCol64x64, toChannelType(COMPONENT_Y)))
        return false;
      if (refPosCol64x64 == pu.Y().topLeft())
        return false;
    }
  }
  else
    return false;

  // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
  const Position refPosLT = pu.Y().topLeft().offset(xBv, yBv);
  const Position refPosBR = pu.Y().bottomRight().offset(xBv, yBv);
  const ChannelType      chType = toChannelType(COMPONENT_Y);
  if (!pu.cs->isDecomp(refPosBR, chType))
    return false;
  if (!pu.cs->isDecomp(refPosLT, chType))
    return false;
  return true;

}// for ibc pu validation
#endif

static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  int iDiffPocD = iColPOC - iColRefPOC;
  int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    int iTDB = Clip3(-128, 127, iDiffPocB);
    int iTDD = Clip3(-128, 127, iDiffPocD);
    int iX = (0x4000 + abs(iTDD / 2)) / iTDD;
    int iScale = Clip3(-4096, 4095, (iTDB * iX + 32) >> 6);
    return iScale;
  }
}

int convertMvFixedToFloat(int32_t val)
{
  int sign  = val >> 31;
  int scale = floorLog2((val ^ sign) | MV_MANTISSA_UPPER_LIMIT) - (MV_MANTISSA_BITCOUNT - 1);

  int exponent;
  int mantissa;
  if (scale >= 0)
  {
    int round = (1 << scale) >> 1;
    int n     = (val + round) >> scale;
    exponent  = scale + ((n ^ sign) >> (MV_MANTISSA_BITCOUNT - 1));
    mantissa  = (n & MV_MANTISSA_UPPER_LIMIT) | (sign << (MV_MANTISSA_BITCOUNT - 1));
  }
  else
  {
    exponent = 0;
    mantissa = val;
  }

  return exponent | (mantissa << MV_EXPONENT_BITCOUNT);
}

int convertMvFloatToFixed(int val)
{
  int exponent = val & MV_EXPONENT_MASK;
  int mantissa = val >> MV_EXPONENT_BITCOUNT;
  return exponent == 0 ? mantissa : (mantissa ^ MV_MANTISSA_LIMIT) << (exponent - 1);
}

int roundMvComp(int x)
{
  return convertMvFloatToFixed(convertMvFixedToFloat(x));
}

int PU::getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC)
{
  return xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);
}

void PU::getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  int refIdxList0, refIdxList1;
  int k;
  int currBaseNum = 0;
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;

  for (k = 0; k < maxNumMergeCand; k++)
  {
    if (mrgCtx.mrgTypeNeighbours[k] == MRG_TYPE_DEFAULT_N)
    {
      refIdxList0 = mrgCtx.mvFieldNeighbours[(k << 1)].refIdx;
      refIdxList1 = mrgCtx.mvFieldNeighbours[(k << 1) + 1].refIdx;

      if ((refIdxList0 >= 0) && (refIdxList1 >= 0))
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      else if (refIdxList0 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = MvField(Mv(0, 0), -1);
      }
      else if (refIdxList1 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = MvField(Mv(0, 0), -1);
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
#if JVET_O0057_ALTHPELIF
      mrgCtx.mmvdUseAltHpelIf[currBaseNum] = mrgCtx.useAltHpelIf[k];
#endif

      currBaseNum++;

      if (currBaseNum == MMVD_BASE_MV_NUM)
        break;
    }
  }
}
bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv, const int &refIdx, bool sbFlag)
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~( scale - 1 );

  const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

  const Slice &slice = *pu.cs->slice;

  // use coldir.
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

  if( !pColPic )
  {
    return false;
  }

  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());

  const MotionInfo& mi = pColPic->cs->getMotionInfo( pos );

  if( !mi.isInter )
  {
    return false;
  }
  if (mi.isIBCmot)
  {
    return false;
  }
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (sbFlag && !slice.getCheckLDC())
  {
    eColRefPicList = eRefPicList;
    iColRefIdx = mi.refIdx[eColRefPicList];
    if (iColRefIdx < 0)
    {
      return false;
    }
  }
  else
  {
    if (iColRefIdx < 0)
    {
      eColRefPicList = RefPicList(1 - eColRefPicList);
      iColRefIdx = mi.refIdx[eColRefPicList];

      if (iColRefIdx < 0)
      {
        return false;
      }
    }
  }

  const Slice *pColSlice = nullptr;

  for( const auto s : pColPic->slices )
  {
    if( s->getIndependentSliceIdx() == mi.sliceIdx )
    {
      pColSlice = s;
      break;
    }
  }

  CHECK( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;

  const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->longTerm;
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];
  cColMv.setHor(roundMvComp(cColMv.getHor()));
  cColMv.setVer(roundMvComp(cColMv.getVer()));

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
  }
  else
  {
    const int currPOC    = slice.getPOC();
    const int colPOC     = colSlice.getPOC();
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->getPOC();
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
    }
    else
    {
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER(const PredictionUnit &pu1, const PredictionUnit &pu2)
{
  const unsigned xN = pu1.lumaPos().x;
  const unsigned yN = pu1.lumaPos().y;
  const unsigned xP = pu2.lumaPos().x;
  const unsigned yP = pu2.lumaPos().y;

  unsigned plevel = pu1.cs->pps->getLog2ParallelMergeLevelMinus2() + 2;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

bool PU::isAddNeighborMv(const Mv& currMv, Mv* neighborMvs, int numNeighborMv)
{
  bool existed = false;
  for (uint32_t cand = 0; cand < numNeighborMv && !existed; cand++)
  {
    if (currMv == neighborMvs[cand])
    {
      existed = true;
    }
  }

  if (!existed)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PU::getIbcMVPsEncOnly(PredictionUnit &pu, Mv* mvPred, int& nbPred)
{
  const PreCalcValues   &pcv = *pu.cs->pcv;
  const int  cuWidth = pu.blocks[COMPONENT_Y].width;
  const int  cuHeight = pu.blocks[COMPONENT_Y].height;
  const int  log2UnitWidth = floorLog2(pcv.minCUWidth);
  const int  log2UnitHeight = floorLog2(pcv.minCUHeight);
  const int  totalAboveUnits = (cuWidth >> log2UnitWidth) + 1;
  const int  totalLeftUnits = (cuHeight >> log2UnitHeight) + 1;

  nbPred = 0;
  Position posLT = pu.Y().topLeft();

  // above-left
  const PredictionUnit *aboveLeftPU = pu.cs->getPURestricted(posLT.offset(-1, -1), pu, CHANNEL_TYPE_LUMA);
  if (aboveLeftPU && CU::isIBC(*aboveLeftPU->cu))
  {
    if (isAddNeighborMv(aboveLeftPU->bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = aboveLeftPU->bv;
    }
  }

  // above neighbors
  for (uint32_t dx = 0; dx < totalAboveUnits && nbPred < IBC_NUM_CANDIDATES; dx++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset((dx << log2UnitWidth), -1), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  // left neighbors
  for (uint32_t dy = 0; dy < totalLeftUnits && nbPred < IBC_NUM_CANDIDATES; dy++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset(-1, (dy << log2UnitHeight)), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  size_t numAvaiCandInLUT = pu.cs->motionLut.lutIbc.size();
  for (uint32_t cand = 0; cand < numAvaiCandInLUT && nbPred < IBC_NUM_CANDIDATES; cand++)
  {
    MotionInfo neibMi = pu.cs->motionLut.lutIbc[cand];
    if (isAddNeighborMv(neibMi.bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = neibMi.bv;
    }
  }

  bool isBvCandDerived[IBC_NUM_CANDIDATES];
  ::memset(isBvCandDerived, false, IBC_NUM_CANDIDATES);

  int curNbPred = nbPred;
  if (curNbPred < IBC_NUM_CANDIDATES)
  {
    do
    {
      curNbPred = nbPred;
      for (uint32_t idx = 0; idx < curNbPred && nbPred < IBC_NUM_CANDIDATES; idx++)
      {
        if (!isBvCandDerived[idx])
        {
          Mv derivedBv;
          if (getDerivedBV(pu, mvPred[idx], derivedBv))
          {
            if (isAddNeighborMv(derivedBv, mvPred, nbPred))
            {
              mvPred[nbPred++] = derivedBv;
            }
          }
          isBvCandDerived[idx] = true;
        }
      }
    } while (nbPred > curNbPred && nbPred < IBC_NUM_CANDIDATES);
  }
}

bool PU::getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv)
{
  int   cuPelX = pu.lumaPos().x;
  int   cuPelY = pu.lumaPos().y;
  int rX = cuPelX + currentMv.getHor();
  int rY = cuPelY + currentMv.getVer();
  int offsetX = currentMv.getHor();
  int offsetY = currentMv.getVer();


#if JVET_O1164_PS
  if( rX < 0 || rY < 0 || rX >= pu.cs->slice->getPPS()->getPicWidthInLumaSamples() || rY >= pu.cs->slice->getPPS()->getPicHeightInLumaSamples() )
#else
  if (rX < 0 || rY < 0 || rX >= pu.cs->slice->getSPS()->getPicWidthInLumaSamples() || rY >= pu.cs->slice->getSPS()->getPicHeightInLumaSamples())
#endif
  {
    return false;
  }

  const PredictionUnit *neibRefPU = NULL;
  neibRefPU = pu.cs->getPURestricted(pu.lumaPos().offset(offsetX, offsetY), pu, CHANNEL_TYPE_LUMA);

  bool isIBC = (neibRefPU) ? CU::isIBC(*neibRefPU->cu) : 0;
  if (isIBC)
  {
    derivedMv = neibRefPU->bv;
    derivedMv += currentMv;
  }
  return isIBC;
}

/**
 * Constructs a list of candidates for IBC AMVP (See specification, section "Derivation process for motion vector predictor candidates")
 */
void PU::fillIBCMvpCand(PredictionUnit &pu, AMVPInfo &amvpInfo)
{

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;


  MergeCtx mergeCtx;
  PU::getIBCMergeCandidates(pu, mergeCtx, AMVP_MAX_NUM_CANDS - 1);
  int candIdx = 0;
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;;
    pInfo->numCand++;
    candIdx++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundIbcPrecInternal2Amvr(pu.cu->imv);
  }
}


/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo)
{
  CodingStructure &cs = *pu.cs;

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

#if !JVET_O0164_REMOVE_AMVP_SPATIAL_SCALING
  bool isScaledFlagLX = false; /// variable name from specification; true when the PUs below left or left are available (availableA0 || availableA1).

  {
    const PredictionUnit* tmpPU = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType ); // getPUBelowLeft(idx, partIdxLB);
    isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );

    if( !isScaledFlagLX )
    {
      tmpPU = cs.getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
      isScaledFlagLX = tmpPU != NULL && CU::isInter( *tmpPU->cu );
    }
  }

  // Left predictor search
  if( isScaledFlagLX )
#endif
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );

#if !JVET_O0164_REMOVE_AMVP_SPATIAL_SCALING
      if( !bAdded )
      {
        bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

        if( !bAdded )
        {
          addMVPCandWithScaling( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );
        }
      }
#endif
    }
  }

  // Above predictor search
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

#if !JVET_O0164_REMOVE_AMVP_SPATIAL_SCALING
  if( !isScaledFlagLX )
  {
    bool bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandWithScaling( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandWithScaling( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }
#endif

  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

  if (cs.slice->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && (pu.lumaSize().width + pu.lumaSize().height > 12))
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool C0Avail = false;
    Position posC1 = pu.Y().center();
    Mv cColMv;

    if( ( ( posRB.x + pcv.minCUWidth ) < pcv.lumaWidth ) && ( ( posRB.y + pcv.minCUHeight ) < pcv.lumaHeight ) )
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }
    if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdx_Col, false ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdx_Col, false ) )
    {
      cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }

  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    const int        currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
    const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;
    addAMVPHMVPCand(pu, eRefPicList, eRefPicList2nd, currRefPOC, *pInfo, pu.cu->imv);
  }

  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
    pInfo->numCand++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundTransPrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo )
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  switch ( dir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1, 0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset( 0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset( 1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1, 1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if ( neibPU == NULL || !CU::isInter( *neibPU->cu ) || !neibPU->cu->affine
    || neibPU->mergeType != MRG_TYPE_DEFAULT_N
    )
  {
    return false;
  }

  Mv outputAffineMv[3];
  const MotionInfo& neibMi = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC = cs.slice->getRefPic( refPicList, refIdx )->getPOC();
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for ( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

    if ( ((neibPU->interDir & (eRefPicListIndex + 1)) == 0) || pu.cu->slice->getRefPOC( eRefPicListIndex, neibRefIdx ) != currRefPOC )
    {
      continue;
    }

    xInheritedAffineMv( pu, neibPU, eRefPicListIndex, outputAffineMv );
    outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
    outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    }
    affiAMVPInfo.numCand++;
    return true;
  }

  return false;
}

void PU::xInheritedAffineMv( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;

  Mv mvLT, mvRT, mvLB;
  mvLT = puNeighbour->mvAffi[eRefPicList][0];
  mvRT = puNeighbour->mvAffi[eRefPicList][1];
  mvLB = puNeighbour->mvAffi[eRefPicList][2];

  bool isTopCtuBoundary = false;
  if ( (posNeiY + neiH) % pu.cs->sps->getCTUSize() == 0 && (posNeiY + neiH) == posCurY )
  {
    // use bottom-left and bottom-right sub-block MVs for inheritance
    const Position posRB = puNeighbour->Y().bottomRight();
    const Position posLB = puNeighbour->Y().bottomLeft();
    mvLT = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];
    mvRT = puNeighbour->getMotionInfo( posRB ).mv[eRefPicList];
    posNeiY += neiH;
    isTopCtuBoundary = true;
  }

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(neiW));
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(neiW));
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM && !isTopCtuBoundary )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - floorLog2(neiH));
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - floorLog2(neiH));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;
  rcMv[0].clipToStorageBitDepth();

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;
  rcMv[1].clipToStorageBitDepth();

  // v2
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2].hor = horTmp;
    rcMv[2].ver = verTmp;
    rcMv[2].clipToStorageBitDepth();
  }
}


void PU::fillAffineMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }


  // insert inherited affine candidates
  Mv outputAffineMv[3];
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  // check left neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo ) )
  {
    addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo );
  }

  // check above neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo ) )
  {
    if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo ) )
    {
      addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo );
    }
  }

  if ( affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS )
  {
    for (int i = 0; i < affiAMVPInfo.numCand; i++)
    {
      affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    }
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0 );
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0 );
  }
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0 );
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1 );
  if ( amvpInfo1.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2 );
  if ( amvpInfo2.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo2.numCand << 2);

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);

  if ( cornerMVPattern == 7 || (cornerMVPattern == 3 && pu.cu->affineType == AFFINEMODEL_4PARAM) )
  {
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    affiAMVPInfo.numCand++;
  }


  if ( affiAMVPInfo.numCand < 2 )
  {
    // check corner MVs
    for ( int i = 2; i >= 0 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; i-- )
    {
      if ( cornerMVPattern & (1 << i) ) // MV i exist
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.numCand++;
      }
    }

    // Get Temporal Motion Predictor
    if ( affiAMVPInfo.numCand < 2 && pu.cs->slice->getEnableTMVPFlag() )
    {
      const int refIdxCol = refIdx;

      Position posRB = pu.Y().bottomRight().offset( -3, -3 );

      const PreCalcValues& pcv = *pu.cs->pcv;

      Position posC0;
      bool C0Avail = false;
      Position posC1 = pu.Y().center();
      Mv cColMv;
      if ( ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight) )
      {
        int posYInCtu = posRB.y & pcv.maxCUHeightMask;
        if (posYInCtu + 4 < pcv.maxCUHeight)
        {
          posC0 = posRB.offset(4, 4);
          C0Avail = true;
        }
      }
      if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdxCol, false ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdxCol, false ) )
      {
        cColMv.roundAffinePrecInternal2Amvr(pu.cu->imv);
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.numCand++;
      }
    }

    if ( affiAMVPInfo.numCand < 2 )
    {
      // add zero MV
      for ( int i = affiAMVPInfo.numCand; i < AMVP_MAX_NUM_CANDS; i++ )
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.numCand++;
      }
    }
  }

  for (int i = 0; i < affiAMVPInfo.numCand; i++)
  {
    affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC     = cs.slice->getRefPic( eRefPicList, iRefIdx )->getPOC();
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if( neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
      info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
      return true;
    }
  }

  return false;
}

#if !JVET_O0164_REMOVE_AMVP_SPATIAL_SCALING
/**
* \param pInfo
* \param eRefPicList
* \param iRefIdx
* \param uiPartUnitIdx
* \param eDir
* \returns bool
*/
bool PU::addMVPCandWithScaling( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const Slice &slice           = *cs.slice;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch( eDir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if (neibPU == NULL || !CU::isInter(*neibPU->cu) || !CU::isInter(*pu.cu))
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  const int  currPOC            = slice.getPOC();
  const int  currRefPOC         = slice.getRefPic( eRefPicList, iRefIdx )->poc;
  const bool bIsCurrRefLongTerm = slice.getRefPic( eRefPicList, iRefIdx )->longTerm;
  const int  neibPOC            = currPOC;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];
    if( neibRefIdx >= 0 )
    {
      const bool bIsNeibRefLongTerm = slice.getRefPic(eRefPicListIndex, neibRefIdx)->longTerm;

      if (bIsCurrRefLongTerm == bIsNeibRefLongTerm)
      {
        Mv cMv = neibMi.mv[eRefPicListIndex];

        if( !( bIsCurrRefLongTerm /* || bIsNeibRefLongTerm*/) )
        {
          const int neibRefPOC = slice.getRefPOC( eRefPicListIndex, neibRefIdx );
          const int scale      = xGetDistScaleFactor( currPOC, currRefPOC, neibPOC, neibRefPOC );

          if( scale != 4096 )
          {
            cMv = cMv.scaleMv( scale );
          }
        }

        info.mvCand[info.numCand++] = cMv;
        return true;
      }
    }
  }

  return false;
}
#endif

void PU::addAMVPHMVPCand(const PredictionUnit &pu, const RefPicList eRefPicList, const RefPicList eRefPicList2nd, const int currRefPOC, AMVPInfo &info, uint8_t imv)
{
  const Slice &slice = *(*pu.cs).slice;

  MotionInfo neibMi;
  auto &lut = CU::isIBC(*pu.cu) ? pu.cs->motionLut.lutIbc : pu.cs->motionLut.lut;
  int num_avai_candInLUT = (int) lut.size();
  int num_allowedCand = std::min(MAX_NUM_HMVP_AVMPCANDS, num_avai_candInLUT);

  for (int mrgIdx = 1; mrgIdx <= num_allowedCand; mrgIdx++)
  {
    if (info.numCand >= AMVP_MAX_NUM_CANDS)
    {
      return;
    }
    neibMi = lut[mrgIdx - 1];

    for (int predictorSource = 0; predictorSource < 2; predictorSource++)
    {
      const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
      const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

      if (neibRefIdx >= 0 && (CU::isIBC(*pu.cu) || (currRefPOC == slice.getRefPOC(eRefPicListIndex, neibRefIdx))))
      {
        Mv pmv = neibMi.mv[eRefPicListIndex];
        pmv.roundTransPrecInternal2Amvr(pu.cu->imv);

        info.mvCand[info.numCand++] = pmv;
        if (info.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return;
        }
      }
    }
  }
}

bool PU::isBipredRestriction(const PredictionUnit &pu)
{
  if(pu.cu->lumaSize().width == 4 && pu.cu->lumaSize().height ==4 )
  {
    return true;
  }
  /* disable bi-prediction for 4x8/8x4 */
  if ( pu.cu->lumaSize().width + pu.cu->lumaSize().height == 12 )
  {
    return true;
  }
  return false;
}

#if JVET_O0366_AFFINE_BCW
void PU::getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t gbiIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgType)
#else
void PU::getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], int8_t neighGbi[4], bool isAvailable[4], int verIdx[4], int modelIdx, int verNum, AffineMergeCtx& affMrgType)
#endif
{
  int cuW = pu.Y().width;
  int cuH = pu.Y().height;
  int vx, vy;
  int shift = MAX_CU_DEPTH;
  int shiftHtoW = shift + floorLog2(cuW) - floorLog2(cuH);

  // motion info
  Mv cMv[2][4];
  int refIdx[2] = { -1, -1 };
  int dir = 0;
#if !JVET_O0366_AFFINE_BCW
  int8_t gbiIdx = GBI_DEFAULT;
#endif
  EAffineModel curType = (verNum == 2) ? AFFINEMODEL_4PARAM : AFFINEMODEL_6PARAM;

  if ( verNum == 2 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1];
    if ( !isAvailable[idx0] || !isAvailable[idx1] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }
#if !JVET_O0366_AFFINE_BCW
    if (dir == 3)
    {
      if (neighGbi[idx0] == neighGbi[idx1])
      {
        gbiIdx = neighGbi[idx0];
      }
    }
#endif

  }
  else if ( verNum == 3 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1], idx2 = verIdx[2];
    if ( !isAvailable[idx0] || !isAvailable[idx1] || !isAvailable[idx2] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 && mi[idx2].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l] && mi[idx0].refIdx[l] == mi[idx2].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }
#if !JVET_O0366_AFFINE_BCW
    int gbiClass[5] = { -1,0,0,0,1 };
    if (dir == 3)
    {
      if (neighGbi[idx0] == neighGbi[idx1] && gbiClass[neighGbi[idx0]] == gbiClass[neighGbi[idx2]])
      {
        gbiIdx = neighGbi[idx0];
      }
      else if (neighGbi[idx0] == neighGbi[idx2] && gbiClass[neighGbi[idx0]] == gbiClass[neighGbi[idx1]])
      {
        gbiIdx = neighGbi[idx0];

      }
      else if (neighGbi[idx1] == neighGbi[idx2] && gbiClass[neighGbi[idx0]] == gbiClass[neighGbi[idx1]])
      {
        gbiIdx = neighGbi[idx1];
      }
      else
      {
        gbiIdx = GBI_DEFAULT;
      }

    }
#endif

  }

  if ( dir == 0 )
  {
    return;
  }


  for ( int l = 0; l < 2; l++ )
  {
    if ( dir & (l + 1) )
    {
      for ( int i = 0; i < verNum; i++ )
      {
        cMv[l][verIdx[i]] = mi[verIdx[i]].mv[l];
      }

      // convert to LT, RT[, [LB]]
      switch ( modelIdx )
      {
      case 0: // 0 : LT, RT, LB
        break;

      case 1: // 1 : LT, RT, RB
        cMv[l][2].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][1].hor;
        cMv[l][2].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][1].ver;
        cMv[l][2].clipToStorageBitDepth();
        break;

      case 2: // 2 : LT, LB, RB
        cMv[l][1].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][2].hor;
        cMv[l][1].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][2].ver;
        cMv[l][1].clipToStorageBitDepth();
        break;

      case 3: // 3 : RT, LB, RB
        cMv[l][0].hor = cMv[l][1].hor + cMv[l][2].hor - cMv[l][3].hor;
        cMv[l][0].ver = cMv[l][1].ver + cMv[l][2].ver - cMv[l][3].ver;
        cMv[l][0].clipToStorageBitDepth();
        break;

      case 4: // 4 : LT, RT
        break;

      case 5: // 5 : LT, LB
        vx = (cMv[l][0].hor << shift) + ((cMv[l][2].ver - cMv[l][0].ver) << shiftHtoW);
        vy = (cMv[l][0].ver << shift) - ((cMv[l][2].hor - cMv[l][0].hor) << shiftHtoW);
        roundAffineMv( vx, vy, shift );
        cMv[l][1].set( vx, vy );
        cMv[l][1].clipToStorageBitDepth();
        break;

      default:
        CHECK( 1, "Invalid model index!\n" );
        break;
      }
    }
    else
    {
      for ( int i = 0; i < 4; i++ )
      {
        cMv[l][i].hor = 0;
        cMv[l][i].ver = 0;
      }
    }
  }

  for ( int i = 0; i < 3; i++ )
  {
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].mv = cMv[0][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].refIdx = refIdx[0];

    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].mv = cMv[1][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].refIdx = refIdx[1];
  }
  affMrgType.interDirNeighbours[affMrgType.numValidMergeCand] = dir;
  affMrgType.affineType[affMrgType.numValidMergeCand] = curType;
#if JVET_O0366_AFFINE_BCW
  affMrgType.GBiIdx[affMrgType.numValidMergeCand] = (dir == 3) ? gbiIdx : GBI_DEFAULT;
#else
  affMrgType.GBiIdx[affMrgType.numValidMergeCand] = gbiIdx;
#endif
  affMrgType.numValidMergeCand++;


  return;
}

const int getAvailableAffineNeighboursForLeftPredictor( const PredictionUnit &pu, const PredictionUnit* npu[] )
{
  const Position posLB = pu.Y().bottomLeft();
  int num = 0;

  const PredictionUnit *puLeftBottom = pu.cs->getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  if ( puLeftBottom && puLeftBottom->cu->affine
    && puLeftBottom->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puLeftBottom;
    return num;
  }

  const PredictionUnit* puLeft = pu.cs->getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  if ( puLeft && puLeft->cu->affine
    && puLeft->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puLeft;
    return num;
  }

  return num;
}

const int getAvailableAffineNeighboursForAbovePredictor( const PredictionUnit &pu, const PredictionUnit* npu[], int numAffNeighLeft )
{
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  int num = numAffNeighLeft;

  const PredictionUnit* puAboveRight = pu.cs->getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  if ( puAboveRight && puAboveRight->cu->affine
    && puAboveRight->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puAboveRight;
    return num;
  }

  const PredictionUnit* puAbove = pu.cs->getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  if ( puAbove && puAbove->cu->affine
    && puAbove->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puAbove;
    return num;
  }

  const PredictionUnit *puAboveLeft = pu.cs->getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  if ( puAboveLeft && puAboveLeft->cu->affine
    && puAboveLeft->mergeType == MRG_TYPE_DEFAULT_N
    )
  {
    npu[num++] = puAboveLeft;
    return num;
  }

  return num;
}

void PU::getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx )
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumAffineMergeCand = slice.getMaxNumAffineMergeCand();

  for ( int i = 0; i < maxNumAffineMergeCand; i++ )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField( Mv(), -1 );
      affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField( Mv(), -1 );
    }
    affMrgCtx.interDirNeighbours[i] = 0;
    affMrgCtx.affineType[i] = AFFINEMODEL_4PARAM;
    affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affMrgCtx.GBiIdx[i] = GBI_DEFAULT;
  }

  affMrgCtx.numValidMergeCand = 0;
  affMrgCtx.maxNumMergeCand = maxNumAffineMergeCand;

  bool enableSubPuMvp = slice.getSPS()->getSBTMVPEnabledFlag() && !(slice.getPOC() == slice.getRefPic(REF_PIC_LIST_0, 0)->getPOC() && slice.isIRAP());
  bool isAvailableSubPu = false;
  if ( enableSubPuMvp && slice.getEnableTMVPFlag() )
  {
    MergeCtx mrgCtx = *affMrgCtx.mrgCtx;
    bool tmpLICFlag = false;

    CHECK( mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized" );
    mrgCtx.subPuMvpMiBuf.fill( MotionInfo() );

    int pos = 0;
    // Get spatial MV
    const Position posCurLB = pu.Y().bottomLeft();
    MotionInfo miLeft;

    //left
    const PredictionUnit* puLeft = cs.getPURestricted( posCurLB.offset( -1, 0 ), pu, pu.chType );
    const bool isAvailableA1 = puLeft && isDiffMER( pu, *puLeft ) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );
    if ( isAvailableA1 )
    {
      miLeft = puLeft->getMotionInfo( posCurLB.offset( -1, 0 ) );
      // get Inter Dir
      mrgCtx.interDirNeighbours[pos] = miLeft.interDir;

      // get Mv from Left
      mrgCtx.mvFieldNeighbours[pos << 1].setMvField( miLeft.mv[0], miLeft.refIdx[0] );

      if ( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[(pos << 1) + 1].setMvField( miLeft.mv[1], miLeft.refIdx[1] );
      }
      pos++;
    }

    mrgCtx.numValidMergeCand = pos;

    isAvailableSubPu = getInterMergeSubPuMvpCand( pu, mrgCtx, tmpLICFlag, pos
      , 0
    );
    if ( isAvailableSubPu )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 0].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 0].refIdx );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 1].refIdx );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = mrgCtx.interDirNeighbours[pos];

      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINE_MODEL_NUM;
      affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_SUBPU_ATMVP;
      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      affMrgCtx.numValidMergeCand++;

      // early termination
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
  }

  if ( slice.getSPS()->getUseAffine() )
  {
    ///> Start: inherited affine candidates
    const PredictionUnit* npu[5];
    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor( pu, npu );
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor( pu, npu, numAffNeighLeft );
    for ( int idx = 0; idx < numAffNeigh; idx++ )
    {
      // derive Mv from Neigh affine PU
      Mv cMv[2][3];
      const PredictionUnit* puNeigh = npu[idx];
      pu.cu->affineType = puNeigh->cu->affineType;
      if ( puNeigh->interDir != 2 )
      {
        xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_0, cMv[0] );
      }
      if ( slice.isInterB() )
      {
        if ( puNeigh->interDir != 1 )
        {
          xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_1, cMv[1] );
        }
      }

      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( cMv[0][mvNum], puNeigh->refIdx[0] );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( cMv[1][mvNum], puNeigh->refIdx[1] );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir;
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = (EAffineModel)(puNeigh->cu->affineType);
      affMrgCtx.GBiIdx[affMrgCtx.numValidMergeCand] = puNeigh->cu->GBiIdx;

      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool isAvailable[4] = { false };

#if JVET_O0366_AFFINE_BCW
      int8_t neighGbi[2] = { GBI_DEFAULT, GBI_DEFAULT };
#else
      int8_t neighGbi[4] = { GBI_DEFAULT, GBI_DEFAULT, GBI_DEFAULT, GBI_DEFAULT };
#endif
      // control point: LT B2->B3->A2
      const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
      for ( int i = 0; i < 3; i++ )
      {
        const Position pos = posLT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if ( puNeigh && CU::isInter( *puNeigh->cu )
          )
        {
          isAvailable[0] = true;
          mi[0] = puNeigh->getMotionInfo( pos );
          neighGbi[0] = puNeigh->cu->GBiIdx;
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { pu.Y().topRight().offset( 0, -1 ), pu.Y().topRight().offset( 1, -1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posRT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );


        if ( puNeigh && CU::isInter( *puNeigh->cu )
          )
        {
          isAvailable[1] = true;
          mi[1] = puNeigh->getMotionInfo( pos );
          neighGbi[1] = puNeigh->cu->GBiIdx;
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { pu.Y().bottomLeft().offset( -1, 0 ), pu.Y().bottomLeft().offset( -1, 1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posLB[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );


        if ( puNeigh && CU::isInter( *puNeigh->cu )
          )
        {
          isAvailable[2] = true;
          mi[2] = puNeigh->getMotionInfo( pos );
#if !JVET_O0366_AFFINE_BCW
          neighGbi[2] = puNeigh->cu->GBiIdx;
#endif
          break;
        }
      }

      // control point: RB
      if ( slice.getEnableTMVPFlag() )
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = pu.Y().bottomRight().offset( -3, -3 );

        const PreCalcValues& pcv = *cs.pcv;
        Position posC0;
        bool C0Avail = false;

        if ( ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight) )
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + 4 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(4, 4);
            C0Avail = true;
          }
        }

        Mv        cColMv;
        int       refIdx = 0;
        bool      bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_0, posC0, cColMv, refIdx, false );
        if ( bExistMV )
        {
          mi[3].mv[0] = cColMv;
          mi[3].refIdx[0] = refIdx;
          mi[3].interDir = 1;
          isAvailable[3] = true;
        }

        if ( slice.isInterB() )
        {
          bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_1, posC0, cColMv, refIdx, false );
          if ( bExistMV )
          {
            mi[3].mv[1] = cColMv;
            mi[3].refIdx[1] = refIdx;
            mi[3].interDir |= 2;
            isAvailable[3] = true;
          }
        }
      }

      //-------------------  insert model  -------------------//
      int order[6] = { 0, 1, 2, 3, 4, 5 };
      int modelNum = 6;
      int model[6][4] = {
        { 0, 1, 2 },          // 0:  LT, RT, LB
        { 0, 1, 3 },          // 1:  LT, RT, RB
        { 0, 2, 3 },          // 2:  LT, LB, RB
        { 1, 2, 3 },          // 3:  RT, LB, RB
        { 0, 1 },             // 4:  LT, RT
        { 0, 2 },             // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx = pu.cs->sps->getUseAffineType() ? 0 : 4;
      for ( int idx = startIdx; idx < modelNum; idx++ )
      {
        int modelIdx = order[idx];
#if JVET_O0366_AFFINE_BCW
        getAffineControlPointCand(pu, mi, isAvailable, model[modelIdx], ((modelIdx == 3) ? neighGbi[1] : neighGbi[0]), modelIdx, verNum[modelIdx], affMrgCtx);
#else
        getAffineControlPointCand(pu, mi, neighGbi, isAvailable, model[modelIdx], modelIdx, verNum[modelIdx], affMrgCtx);
#endif
        if ( affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx )
        {
          return;
        }

        // early termination
        if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
        {
          return;
        }
      }
    }
    ///> End: Constructed affine candidates
  }

  ///> zero padding
  int cnt = affMrgCtx.numValidMergeCand;
  while ( cnt < maxNumAffineMergeCand )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField( Mv( 0, 0 ), 0 );
    }
    affMrgCtx.interDirNeighbours[cnt] = 1;

    if ( slice.isInterB() )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField( Mv( 0, 0 ), 0 );
      }
      affMrgCtx.interDirNeighbours[cnt] = 3;
    }
    affMrgCtx.affineType[cnt] = AFFINEMODEL_4PARAM;

    if ( cnt == mrgCandIdx )
    {
      return;
    }
    cnt++;
    affMrgCtx.numValidMergeCand++;
  }
}

void PU::setAllAffineMvField( PredictionUnit &pu, MvField *mvField, RefPicList eRefList )
{
  // Set Mv
  Mv mv[3];
  for ( int i = 0; i < 3; i++ )
  {
    mv[i] = mvField[i].mv;
  }
  setAllAffineMv( pu, mv[0], mv[1], mv[2], eRefList );

  // Set RefIdx
  CHECK( mvField[0].refIdx != mvField[1].refIdx || mvField[0].refIdx != mvField[2].refIdx, "Affine mv corners don't have the same refIdx." );
  pu.refIdx[eRefList] = mvField[0].refIdx;
}

void PU::setAllAffineMv(PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs)
{
  int width  = pu.Y().width;
  int shift = MAX_CU_DEPTH;
  if (clipCPMVs)
  {
    affLT.mvCliptoStorageBitDepth();
    affRT.mvCliptoStorageBitDepth();
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      affLB.mvCliptoStorageBitDepth();
    }
  }
  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
  deltaMvHorX = (affRT - affLT).getHor() << (shift - floorLog2(width));
  deltaMvHorY = (affRT - affLT).getVer() << (shift - floorLog2(width));
  int height = pu.Y().height;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = (affLB - affLT).getHor() << (shift - floorLog2(height));
    deltaMvVerY = (affLB - affLT).getVer() << (shift - floorLog2(height));
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY = deltaMvHorX;
  }

  int mvScaleHor = affLT.getHor() << shift;
  int mvScaleVer = affLT.getVer() << shift;

  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
  const int halfBW = blockWidth >> 1;
  const int halfBH = blockHeight >> 1;

  MotionBuf mb = pu.getMotionBuf();
  int mvScaleTmpHor, mvScaleTmpVer;
  const bool subblkMVSpreadOverLimit = InterPrediction::isSubblockVectorSpreadOverLimit( deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, pu.interDir );
  for ( int h = 0; h < pu.Y().height; h += blockHeight )
  {
    for ( int w = 0; w < pu.Y().width; w += blockWidth )
    {
      if ( !subblkMVSpreadOverLimit )
      {
        mvScaleTmpHor = mvScaleHor + deltaMvHorX * (halfBW + w) + deltaMvVerX * (halfBH + h);
        mvScaleTmpVer = mvScaleVer + deltaMvHorY * (halfBW + w) + deltaMvVerY * (halfBH + h);

      }
      else
      {
        mvScaleTmpHor = mvScaleHor + deltaMvHorX * ( pu.Y().width >> 1 ) + deltaMvVerX * ( pu.Y().height >> 1 );
        mvScaleTmpVer = mvScaleVer + deltaMvHorY * ( pu.Y().width >> 1 ) + deltaMvVerY * ( pu.Y().height >> 1 );
      }
      roundAffineMv( mvScaleTmpHor, mvScaleTmpVer, shift );
      Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
      curMv.clipToStorageBitDepth();

      for ( int y = (h >> MIN_CU_LOG2); y < ((h + blockHeight) >> MIN_CU_LOG2); y++ )
      {
        for ( int x = (w >> MIN_CU_LOG2); x < ((w + blockWidth) >> MIN_CU_LOG2); x++ )
        {
          mb.at(x, y).mv[eRefList] = curMv;
        }
      }
    }
  }

  pu.mvAffi[eRefList][0] = affLT;
  pu.mvAffi[eRefList][1] = affRT;
  pu.mvAffi[eRefList][2] = affLB;
}

void clipColPos(int& posX, int& posY, const PredictionUnit& pu)
{
  Position puPos = pu.lumaPos();
  int log2CtuSize = floorLog2(pu.cs->sps->getCTUSize());
  int ctuX = ((puPos.x >> log2CtuSize) << log2CtuSize);
  int ctuY = ((puPos.y >> log2CtuSize) << log2CtuSize);
#if JVET_O1164_PS
  int horMax = std::min( (int)pu.cs->pps->getPicWidthInLumaSamples() - 1, ctuX + (int)pu.cs->sps->getCTUSize() + 3 );
#else
  int horMax = std::min((int)pu.cs->sps->getPicWidthInLumaSamples() - 1, ctuX + (int)pu.cs->sps->getCTUSize() + 3);
#endif
  int horMin = std::max((int)0, ctuX);
#if JVET_O1164_PS
  int verMax = std::min( (int)pu.cs->pps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.cs->sps->getCTUSize() - 1 );
#else
  int verMax = std::min((int)pu.cs->sps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.cs->sps->getCTUSize() - 1);
#endif
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
}

bool PU::getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count
  , int mmvdList
)
{
  const Slice   &slice = *pu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);

  const Picture *pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
  Mv cTMv;

#if JVET_O0163_REMOVE_SWITCHING_TMV
  if ( count )
  {
    if ( (mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_0)) && slice.getRefPic( REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].refIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].mv;
    }
    else if ( slice.isInterB() && (mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_1)) && slice.getRefPic( REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].refIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].mv;
    }
  }
#else
  bool terminate = false;
  for (unsigned currRefListId = 0; currRefListId < (slice.getSliceType() == B_SLICE ? 2 : 1) && !terminate; currRefListId++)
  {
    if ( count )
    {
      RefPicList currRefPicList = RefPicList(slice.getCheckLDC() ? (slice.getColFromL0Flag() ? currRefListId : 1 - currRefListId) : currRefListId);

      if ((mrgCtx.interDirNeighbours[0] & (1 << currRefPicList)) && slice.getRefPic(currRefPicList, mrgCtx.mvFieldNeighbours[0 * 2 + currRefPicList].refIdx) == pColPic)
      {
        cTMv = mrgCtx.mvFieldNeighbours[0 * 2 + currRefPicList].mv;
        terminate = true;
        break;
      }
    }
  }
#endif

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////
  int mvPrec = MV_FRACTIONAL_BITS_INTERNAL;

  Mv cTempVector = cTMv;
  bool  tempLICFlag = false;

  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();
  int numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;

  Mv cColMv;
  int refIdx = 0;
  // use coldir.
  bool     bBSlice = slice.isInterB();

  Position centerPos;

  bool found = false;
  cTempVector = cTMv;
  int tempX = cTempVector.getHor() >> mvPrec;
  int tempY = cTempVector.getVer() >> mvPrec;

  centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
  centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

  clipColPos(centerPos.x, centerPos.y, pu);

  centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

  // derivation of center motion parameters from the collocated CU
  const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

  if (mi.isInter && mi.isIBCmot == false)
  {
    mrgCtx.interDirNeighbours[count] = 0;

    for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList  currRefPicList = RefPicList(currRefListId);

      if (getColocatedMVP(pu, currRefPicList, centerPos, cColMv, refIdx, true))
      {
        // set as default, for further motion vector field spanning
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, 0);
        mrgCtx.interDirNeighbours[count] |= (1 << currRefListId);
        LICFlag = tempLICFlag;
        mrgCtx.GBiIdx[count] = GBI_DEFAULT;
        found = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(Mv(), NOT_VALID);
        mrgCtx.interDirNeighbours[count] &= ~(1 << currRefListId);
      }
    }
  }

  if (!found)
  {
    return false;
  }
  if (mmvdList != 1)
  {
  int xOff = (puWidth >> 1) + tempX;
  int yOff = (puHeight >> 1) + tempY;

  MotionBuf& mb = mrgCtx.subPuMvpMiBuf;

  const bool isBiPred = isBipredRestriction(pu);

  for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
  {
    for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
    {
      Position colPos{ x + xOff, y + yOff };

      clipColPos(colPos.x, colPos.y, pu);

      colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };

      const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);

      MotionInfo mi;

      found = false;
      mi.isInter = true;
      mi.sliceIdx = slice.getIndependentSliceIdx();
      mi.isIBCmot = false;
      if (colMi.isInter && colMi.isIBCmot == false)
      {
        for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
        {
          RefPicList currRefPicList = RefPicList(currRefListId);
          if (getColocatedMVP(pu, currRefPicList, colPos, cColMv, refIdx, true))
          {
            mi.refIdx[currRefListId] = 0;
            mi.mv[currRefListId] = cColMv;
            found = true;
          }
        }
      }
      if (!found)
      {
        mi.mv[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mv;
        mi.mv[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mv;
        mi.refIdx[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].refIdx;
        mi.refIdx[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].refIdx;
      }

      mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);

      if (isBiPred && mi.interDir == 3)
      {
        mi.interDir = 1;
        mi.mv[1] = Mv();
        mi.refIdx[1] = NOT_VALID;
      }

      mb.subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight))).fill(mi);
      }
    }
  }
  return true;
}

void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
{
  MotionBuf mb = pu.getMotionBuf();

  if( !pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N
    || pu.mergeType == MRG_TYPE_IBC
    )
  {
    MotionInfo mi;

    mi.isInter = !CU::isIntra(*pu.cu);
    mi.isIBCmot = CU::isIBC(*pu.cu);
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;
#if JVET_O0057_ALTHPELIF
      mi.useAltHpelIf = pu.cu->imv == IMV_HPEL;
#endif

      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = pu.bv;
      }
    }

    if( pu.cu->affine )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &dest = mb.at( x, y );
          dest.isInter  = mi.isInter;
          dest.isIBCmot = false;
          dest.interDir = mi.interDir;
          dest.sliceIdx = mi.sliceIdx;
          for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
          {
            if( mi.refIdx[i] == -1 )
            {
              dest.mv[i] = Mv();
            }
            dest.refIdx[i] = mi.refIdx[i];
          }
        }
      }
    }
    else
    {
      mb.fill( mi );
    }
  }
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
  }
  else
  {

    if( isBipredRestriction( pu ) )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &mi = mb.at( x, y );
          if( mi.interDir == 3 )
          {
            mi.interDir  = 1;
            mi.mv    [1] = Mv();
            mi.refIdx[1] = NOT_VALID;
          }
        }
      }
    }
  }
}

void PU::applyImv( PredictionUnit& pu, MergeCtx &mrgCtx, InterPrediction *interPred )
{
  if( !pu.mergeFlag )
  {
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      pu.mvd[0].changeTransPrecAmvr2Internal(pu.cu->imv);
      unsigned mvp_idx = pu.mvpIdx[0];
      AMVPInfo amvpInfo;
      if (CU::isIBC(*pu.cu))
      {
        PU::fillIBCMvpCand(pu, amvpInfo);
      }
      else
      PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo);
      pu.mvpNum[0] = amvpInfo.numCand;
      pu.mvpIdx[0] = mvp_idx;
      pu.mv    [0] = amvpInfo.mvCand[mvp_idx] + pu.mvd[0];
      pu.mv[0].mvCliptoStorageBitDepth();
    }

    if (pu.interDir != 1 /* PRED_L0 */)
    {
      if( !( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 ) && pu.cu->imv )/* PRED_BI */
      {
        pu.mvd[1].changeTransPrecAmvr2Internal(pu.cu->imv);
      }
      unsigned mvp_idx = pu.mvpIdx[1];
      AMVPInfo amvpInfo;
      PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo);
      pu.mvpNum[1] = amvpInfo.numCand;
      pu.mvpIdx[1] = mvp_idx;
      pu.mv    [1] = amvpInfo.mvCand[mvp_idx] + pu.mvd[1];
      pu.mv[1].mvCliptoStorageBitDepth();
    }
  }
  else
  {
    // this function is never called for merge
    THROW("unexpected");
    PU::getInterMergeCandidates ( pu, mrgCtx
      , 0
    );

    mrgCtx.setMergeInfo( pu, pu.mergeIdx );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}

bool PU::isBiPredFromDifferentDir( const PredictionUnit& pu )
{
  if ( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
  {
    const int iPOC0 = pu.cu->slice->getRefPOC( REF_PIC_LIST_0, pu.refIdx[0] );
    const int iPOC1 = pu.cu->slice->getRefPOC( REF_PIC_LIST_1, pu.refIdx[1] );
    const int iPOC  = pu.cu->slice->getPOC();
    if ( (iPOC - iPOC0)*(iPOC - iPOC1) < 0 )
    {
      return true;
    }
  }

  return false;
}

bool PU::isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu)
{
  if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
  {
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, pu.refIdx[0]);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, pu.refIdx[1]);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}

void PU::restrictBiPredMergeCandsOne(PredictionUnit &pu)
{
  if (PU::isBipredRestriction(pu))
  {
    if (pu.interDir == 3)
    {
      pu.interDir = 1;
      pu.refIdx[1] = -1;
      pu.mv[1] = Mv(0, 0);
      pu.cu->GBiIdx = GBI_DEFAULT;
    }
  }
}

void PU::getTriangleMergeCandidates( const PredictionUnit &pu, MergeCtx& triangleMrgCtx )
{
  MergeCtx tmpMergeCtx;

  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumMergeCand = slice.getMaxNumMergeCand();

  triangleMrgCtx.numValidMergeCand = 0;

  for (int32_t i = 0; i < TRIANGLE_MAX_NUM_UNI_CANDS; i++)
  {
    triangleMrgCtx.GBiIdx[i] = GBI_DEFAULT;
    triangleMrgCtx.interDirNeighbours[i] = 0;
    triangleMrgCtx.mrgTypeNeighbours[i] = MRG_TYPE_DEFAULT_N;
    triangleMrgCtx.mvFieldNeighbours[(i << 1)].refIdx = NOT_VALID;
    triangleMrgCtx.mvFieldNeighbours[(i << 1) + 1].refIdx = NOT_VALID;
    triangleMrgCtx.mvFieldNeighbours[(i << 1)].mv = Mv();
    triangleMrgCtx.mvFieldNeighbours[(i << 1) + 1].mv = Mv();
#if JVET_O0057_ALTHPELIF
    triangleMrgCtx.useAltHpelIf[i] = false;
#endif
  }

  PU::getInterMergeCandidates(pu, tmpMergeCtx, 0);

  for (int32_t i = 0; i < maxNumMergeCand; i++)
  {
    int parity = i & 1;
    if (tmpMergeCtx.interDirNeighbours[i] & (0x01 + parity))
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 1 + parity;
      triangleMrgCtx.mrgTypeNeighbours[triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].mv;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].refIdx = -1;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].refIdx;
      triangleMrgCtx.numValidMergeCand++;
      if (triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS)
      {
        return;
      }
      continue;
    }

    if (tmpMergeCtx.interDirNeighbours[i] & (0x02 - parity))
    {
      triangleMrgCtx.interDirNeighbours[triangleMrgCtx.numValidMergeCand] = 2 - parity;
      triangleMrgCtx.mrgTypeNeighbours[triangleMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + !parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].refIdx;
      triangleMrgCtx.mvFieldNeighbours[(triangleMrgCtx.numValidMergeCand << 1) + parity].refIdx = -1;
      triangleMrgCtx.numValidMergeCand++;
      if (triangleMrgCtx.numValidMergeCand == TRIANGLE_MAX_NUM_UNI_CANDS)
      {
        return;
      }
    }
  }
}

void PU::spanTriangleMotionInfo( PredictionUnit &pu, MergeCtx &triangleMrgCtx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1 )
{
  pu.triangleSplitDir = splitDir;
  pu.triangleMergeIdx0 = candIdx0;
  pu.triangleMergeIdx1 = candIdx1;
  MotionBuf mb = pu.getMotionBuf();

  MotionInfo biMv;
  biMv.isInter  = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if( triangleMrgCtx.interDirNeighbours[candIdx0] == 1 && triangleMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
    biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].refIdx;
    biMv.refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }
  else if( triangleMrgCtx.interDirNeighbours[candIdx0] == 2 && triangleMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
    biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
    biMv.refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].refIdx;
    biMv.refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
  }
  else if( triangleMrgCtx.interDirNeighbours[candIdx0] == 1 && triangleMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
#if JVET_O0265_TPM_SIMPLIFICATION
    biMv.interDir = 1;
    biMv.mv[0] = triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
    biMv.mv[1] = Mv(0, 0);
    biMv.refIdx[0] = triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx;
    biMv.refIdx[1] = -1;
#else
    int32_t refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_0, triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx ), REF_PIC_LIST_1 );
    if( refIdx != -1 )
    {
      biMv.interDir  = 3;
      biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
      biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
      biMv.refIdx[0] = triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx;
      biMv.refIdx[1] = refIdx;
    }
    else
    {
      refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_0, triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx), REF_PIC_LIST_1 );
      biMv.interDir  = ( refIdx != -1 ) ? 3 : 1;
      biMv.mv[0]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv : triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
      biMv.mv[1]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv : Mv(0, 0);
      biMv.refIdx[0] = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx : triangleMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx;
      biMv.refIdx[1] = ( refIdx != -1 ) ? refIdx : -1;
    }
#endif
  }
  else if( triangleMrgCtx.interDirNeighbours[candIdx0] == 2 && triangleMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
#if JVET_O0265_TPM_SIMPLIFICATION
    biMv.interDir = 2;
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = -1;
    biMv.refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
#else
    int32_t refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_1, triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx ), REF_PIC_LIST_0 );
    if( refIdx != -1 )
    {
      biMv.interDir  = 3;
      biMv.mv[0]     = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
      biMv.mv[1]     = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
      biMv.refIdx[0] = refIdx;
      biMv.refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
    }
    else
    {
      refIdx = mappingRefPic( pu, pu.cs->slice->getRefPOC( REF_PIC_LIST_1, triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx ), REF_PIC_LIST_0 );
      biMv.interDir  = ( refIdx != -1 ) ? 3 : 2;
      biMv.mv[0]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv : Mv(0, 0);
      biMv.mv[1]     = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv : triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
      biMv.refIdx[0] = ( refIdx != -1 ) ? refIdx : -1;
      biMv.refIdx[1] = ( refIdx != -1 ) ? triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx : triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
    }
#endif
  }

  int32_t idxW  = (int32_t)(floorLog2(pu.lwidth() ) - MIN_CU_LOG2);
  int32_t idxH  = (int32_t)(floorLog2(pu.lheight()) - MIN_CU_LOG2);
  for( int32_t y = 0; y < mb.height; y++ )
  {
    for( int32_t x = 0; x < mb.width; x++ )
    {
      if( g_triangleMvStorage[splitDir][idxH][idxW][y][x] == 2 )
      {
        mb.at( x, y ).isInter   = true;
        mb.at( x, y ).interDir  = biMv.interDir;
        mb.at( x, y ).refIdx[0] = biMv.refIdx[0];
        mb.at( x, y ).refIdx[1] = biMv.refIdx[1];
        mb.at( x, y ).mv    [0] = biMv.mv    [0];
        mb.at( x, y ).mv    [1] = biMv.mv    [1];
        mb.at( x, y ).sliceIdx  = biMv.sliceIdx;
      }
      else if( g_triangleMvStorage[splitDir][idxH][idxW][y][x] == 0 )
      {
        mb.at( x, y ).isInter   = true;
        mb.at( x, y ).interDir  = triangleMrgCtx.interDirNeighbours[candIdx0];
        mb.at( x, y ).refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].refIdx;
        mb.at( x, y ).refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
        mb.at( x, y ).mv    [0] = triangleMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
        mb.at( x, y ).mv    [1] = triangleMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
        mb.at( x, y ).sliceIdx  = biMv.sliceIdx;
      }
      else
      {
        mb.at( x, y ).isInter   = true;
        mb.at( x, y ).interDir  = triangleMrgCtx.interDirNeighbours[candIdx1];
        mb.at( x, y ).refIdx[0] = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].refIdx;
        mb.at( x, y ).refIdx[1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
        mb.at( x, y ).mv    [0] = triangleMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
        mb.at( x, y ).mv    [1] = triangleMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
        mb.at( x, y ).sliceIdx  = biMv.sliceIdx;
      }
    }
  }
}

int32_t PU::mappingRefPic( const PredictionUnit &pu, int32_t refPicPoc, bool targetRefPicList )
{
  int32_t numRefIdx = pu.cs->slice->getNumRefIdx( (RefPicList)targetRefPicList );

  for( int32_t i = 0; i < numRefIdx; i++ )
  {
    if( pu.cs->slice->getRefPOC( (RefPicList)targetRefPicList, i ) == refPicPoc )
    {
      return i;
    }
  }
  return -1;
}

bool CU::hasSubCUNonZeroMVd( const CodingUnit& cu )
{
  bool bNonZeroMvd = false;

  for( const auto &pu : CU::traversePUs( cu ) )
  {
    if( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getHor() != 0;
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getVer() != 0;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        if( !pu.cu->cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getHor() != 0;
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getVer() != 0;
        }
      }
    }
  }

  return bNonZeroMvd;
}

bool CU::hasSubCUNonZeroAffineMVd( const CodingUnit& cu )
{
  bool nonZeroAffineMvd = false;

  if ( !cu.affine || cu.firstPU->mergeFlag )
  {
    return false;
  }

  for ( const auto &pu : CU::traversePUs( cu ) )
  {
    if ( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if ( pu.interDir != 2 /* PRED_L1 */ )
      {
        for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
        {
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getHor() != 0;
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getVer() != 0;
        }
      }

      if ( pu.interDir != 1 /* PRED_L0 */ )
      {
        if ( !pu.cu->cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
          {
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getHor() != 0;
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getVer() != 0;
          }
        }
      }
    }
  }

  return nonZeroAffineMvd;
}

uint8_t CU::getSbtInfo( uint8_t idx, uint8_t pos )
{
  return ( pos << 4 ) + ( idx << 0 );
}

uint8_t CU::getSbtIdx( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 0 ) & 0xf;
}

uint8_t CU::getSbtPos( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 4 ) & 0x3;
}

uint8_t CU::getSbtMode( uint8_t sbtIdx, uint8_t sbtPos )
{
  uint8_t sbtMode = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: sbtMode = sbtPos + SBT_VER_H0;  break;
  case SBT_HOR_HALF: sbtMode = sbtPos + SBT_HOR_H0;  break;
  case SBT_VER_QUAD: sbtMode = sbtPos + SBT_VER_Q0;  break;
  case SBT_HOR_QUAD: sbtMode = sbtPos + SBT_HOR_Q0;  break;
  default:           assert( 0 );
  }

  assert( sbtMode < NUMBER_SBT_MODE );
  return sbtMode;
}

uint8_t CU::getSbtIdxFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
    return SBT_VER_HALF;
  else if( sbtMode <= SBT_HOR_H1 )
    return SBT_HOR_HALF;
  else if( sbtMode <= SBT_VER_Q1 )
    return SBT_VER_QUAD;
  else if( sbtMode <= SBT_HOR_Q1 )
    return SBT_HOR_QUAD;
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::getSbtPosFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
    return sbtMode - SBT_VER_H0;
  else if( sbtMode <= SBT_HOR_H1 )
    return sbtMode - SBT_HOR_H0;
  else if( sbtMode <= SBT_VER_Q1 )
    return sbtMode - SBT_VER_Q0;
  else if( sbtMode <= SBT_HOR_Q1 )
    return sbtMode - SBT_HOR_Q0;
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::targetSbtAllowed( uint8_t sbtIdx, uint8_t sbtAllowed )
{
  uint8_t val = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: val = ( ( sbtAllowed >> SBT_VER_HALF ) & 0x1 ); break;
  case SBT_HOR_HALF: val = ( ( sbtAllowed >> SBT_HOR_HALF ) & 0x1 ); break;
  case SBT_VER_QUAD: val = ( ( sbtAllowed >> SBT_VER_QUAD ) & 0x1 ); break;
  case SBT_HOR_QUAD: val = ( ( sbtAllowed >> SBT_HOR_QUAD ) & 0x1 ); break;
  default:           CHECK( 1, "unknown SBT type" );
  }
  return val;
}

uint8_t CU::numSbtModeRdo( uint8_t sbtAllowed )
{
  uint8_t num = 0;
  uint8_t sum = 0;
  num = targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) + targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  num = targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  return sum;
}

bool CU::isSbtMode( const uint8_t sbtInfo )
{
  uint8_t sbtIdx = getSbtIdx( sbtInfo );
  return sbtIdx >= SBT_VER_HALF && sbtIdx <= SBT_HOR_QUAD;
}

bool CU::isSameSbtSize( const uint8_t sbtInfo1, const uint8_t sbtInfo2 )
{
  uint8_t sbtIdx1 = getSbtIdxFromSbtMode( sbtInfo1 );
  uint8_t sbtIdx2 = getSbtIdxFromSbtMode( sbtInfo2 );
  if( sbtIdx1 == SBT_HOR_HALF || sbtIdx1 == SBT_VER_HALF )
    return sbtIdx2 == SBT_HOR_HALF || sbtIdx2 == SBT_VER_HALF;
  else if( sbtIdx1 == SBT_HOR_QUAD || sbtIdx1 == SBT_VER_QUAD )
    return sbtIdx2 == SBT_HOR_QUAD || sbtIdx2 == SBT_VER_QUAD;
  else
    return false;
}

#if JVET_O0106_ISP_4xN_PREDREG_FOR_1xN_2xN
bool CU::isPredRegDiffFromTB(const CodingUnit &cu, const ComponentID compID)
{
  return (compID == COMPONENT_Y)
    && (cu.ispMode == VER_INTRA_SUBPARTITIONS &&
      CU::isMinWidthPredEnabledForBlkSize(cu.blocks[compID].width, cu.blocks[compID].height)
      );
}

bool CU::isMinWidthPredEnabledForBlkSize(const int w, const int h)
{
  return ((w == 8 && h > 4) || w == 4);
}

bool CU::isFirstTBInPredReg(const CodingUnit& cu, const ComponentID compID, const CompArea &area)
{
  return (compID == COMPONENT_Y) && cu.ispMode && ((area.topLeft().x - cu.Y().topLeft().x) % PRED_REG_MIN_WIDTH == 0);
}

void CU::adjustPredArea(CompArea &area)
{
  area.width = std::max<int>(PRED_REG_MIN_WIDTH, area.width);
}
#endif

bool CU::isGBiIdxCoded( const CodingUnit &cu )
{
  if( cu.cs->sps->getUseGBi() == false )
  {
    CHECK(cu.GBiIdx != GBI_DEFAULT, "Error: cu.GBiIdx != GBI_DEFAULT");
    return false;
  }

  if (cu.predMode == MODE_IBC)
  {
    return false;
  }

  if( cu.predMode == MODE_INTRA || cu.cs->slice->isInterP() )
  {
    return false;
  }

  if( cu.lwidth() * cu.lheight() < GBI_SIZE_CONSTRAINT )
  {
    return false;
  }

  if( !cu.firstPU->mergeFlag )
  {
    if( cu.firstPU->interDir == 3 )
    {
      WPScalingParam *wp0;
      WPScalingParam *wp1;
      int refIdx0 = cu.firstPU->refIdx[REF_PIC_LIST_0];
      int refIdx1 = cu.firstPU->refIdx[REF_PIC_LIST_1];

      cu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0, wp0);
      cu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1, wp1);
      if ((wp0[COMPONENT_Y].bPresentFlag || wp0[COMPONENT_Cb].bPresentFlag || wp0[COMPONENT_Cr].bPresentFlag
        || wp1[COMPONENT_Y].bPresentFlag || wp1[COMPONENT_Cb].bPresentFlag || wp1[COMPONENT_Cr].bPresentFlag))
      {
        return false;
      }
      return true;
    }
  }

  return false;
}

uint8_t CU::getValidGbiIdx( const CodingUnit &cu )
{
  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    return cu.GBiIdx;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    CHECK(cu.GBiIdx != GBI_DEFAULT, " cu.GBiIdx != GBI_DEFAULT ");
  }
  else
  {
    CHECK(cu.GBiIdx != GBI_DEFAULT, " cu.GBiIdx != GBI_DEFAULT ");
  }

  return GBI_DEFAULT;
}

void CU::setGbiIdx( CodingUnit &cu, uint8_t uh )
{
  int8_t uhCnt = 0;

  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    cu.GBiIdx = uh;
    ++uhCnt;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    cu.GBiIdx = GBI_DEFAULT;
  }
  else
  {
    cu.GBiIdx = GBI_DEFAULT;
  }

  CHECK(uhCnt <= 0, " uhCnt <= 0 ");
}

uint8_t CU::deriveGbiIdx( uint8_t gbiLO, uint8_t gbiL1 )
{
  if( gbiLO == gbiL1 )
  {
    return gbiLO;
  }
  const int8_t w0 = getGbiWeight(gbiLO, REF_PIC_LIST_0);
  const int8_t w1 = getGbiWeight(gbiL1, REF_PIC_LIST_1);
  const int8_t th = g_GbiWeightBase >> 1;
  const int8_t off = 1;

  if( w0 == w1 || (w0 < (th - off) && w1 < (th - off)) || (w0 >(th + off) && w1 >(th + off)) )
  {
    return GBI_DEFAULT;
  }
  else
  {
    if( w0 > w1 )
    {
      return ( w0 >= th ? gbiLO : gbiL1 );
    }
    else
    {
      return ( w1 >= th ? gbiL1 : gbiLO );
    }
  }
}

bool CU::bdpcmAllowed( const CodingUnit& cu, const ComponentID compID )
{
#if JVET_O1136_TS_BDPCM_SIGNALLING
  SizeType transformSkipMaxSize = 1 << cu.cs->pps->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize();
#endif

  bool bdpcmAllowed = compID == COMPONENT_Y;
       bdpcmAllowed &= CU::isIntra( cu );
#if JVET_O1136_TS_BDPCM_SIGNALLING
       bdpcmAllowed &= ( cu.lwidth() <= transformSkipMaxSize && cu.lheight() <= transformSkipMaxSize );
#else
       bdpcmAllowed &= ( cu.lwidth() <= 32 && cu.lheight() <= 32 );
#endif

  return bdpcmAllowed;
}
// TU tools

bool TU::isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID)
{
  return tu.cs->sps->getSpsRangeExtension().getTransformSkipRotationEnabledFlag() && tu.blocks[compID].width == 4 && tu.cu->predMode == MODE_INTRA;
}

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
  return getCbfAtDepth( tu, compID, tu.depth );
}

bool TU::getCbfAtDepth(const TransformUnit &tu, const ComponentID &compID, const unsigned &depth)
{
#if JVET_O0050_LOCAL_DUAL_TREE
  if( !tu.blocks[compID].valid() )
    CHECK( tu.cbf[compID] != 0, "cbf must be 0 if the component is not available" );
#endif
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1  << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}

bool TU::isTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  bool    tsAllowed = compID == COMPONENT_Y;
  const int maxSize = tu.cs->pps->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize();

#if JVET_O1136_TS_BDPCM_SIGNALLING
  tsAllowed &= tu.cs->sps->getTransformSkipEnabledFlag();
#else
  tsAllowed &= tu.cs->pps->getUseTransformSkip();
#endif
  tsAllowed &= !tu.cu->transQuantBypass;
  tsAllowed &= ( !tu.cu->ispMode || !isLuma(compID) );
#if JVET_O1136_TS_BDPCM_SIGNALLING
  SizeType transformSkipMaxSize = 1 << maxSize;
  tsAllowed &= !(tu.cu->bdpcmMode && tu.lwidth() <= transformSkipMaxSize && tu.lheight() <= transformSkipMaxSize);
#else
  tsAllowed &= !( tu.cu->bdpcmMode && tu.lwidth() <= BDPCM_MAX_CU_SIZE && tu.lheight() <= BDPCM_MAX_CU_SIZE );
  SizeType transformSkipMaxSize = 1 << maxSize;
#endif
  tsAllowed &= tu.lwidth() <= transformSkipMaxSize && tu.lheight() <= transformSkipMaxSize;
  tsAllowed &= !tu.cu->sbtInfo;

  return tsAllowed;
}

bool TU::isMTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  bool   mtsAllowed = compID == COMPONENT_Y;
  const int maxSize = CU::isIntra( *tu.cu ) ? MTS_INTRA_MAX_CU_SIZE : MTS_INTER_MAX_CU_SIZE;

  mtsAllowed &= CU::isIntra( *tu.cu ) ? tu.cs->sps->getUseIntraMTS() : tu.cs->sps->getUseInterMTS() && CU::isInter( *tu.cu );
  mtsAllowed &= ( tu.lwidth() <= maxSize && tu.lheight() <= maxSize );
  mtsAllowed &= !tu.cu->ispMode;
  mtsAllowed &= !tu.cu->sbtInfo;
#if JVET_O1136_TS_BDPCM_SIGNALLING
  SizeType transformSkipMaxSize = 1 << tu.cs->pps->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize();
  mtsAllowed &= !( tu.cu->bdpcmMode && tu.lwidth() <= transformSkipMaxSize && tu.lheight() <= transformSkipMaxSize);
#else
  mtsAllowed &= !( tu.cu->bdpcmMode && tu.lwidth() <= BDPCM_MAX_CU_SIZE && tu.lheight() <= BDPCM_MAX_CU_SIZE );
#endif
  return mtsAllowed;
}

#if JVET_O0105_ICT
int TU::getICTMode( const TransformUnit& tu, int jointCbCr )
{
  if( jointCbCr < 0 )
  {
    jointCbCr = tu.jointCbCr;
  }
  return g_ictModes[ tu.cs->slice->getJointCbCrSignFlag() ][ jointCbCr ];
}
#endif

bool TU::hasCrossCompPredInfo( const TransformUnit &tu, const ComponentID &compID )
{
  return (isChroma(compID) && tu.cs->pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && TU::getCbf(tu, COMPONENT_Y) &&
    (!CU::isIntra(*tu.cu) || PU::isChromaIntraModeCrossCheckMode(*tu.cs->getPU(tu.blocks[compID].pos(), toChannelType(compID)))));
}

#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
uint32_t TU::getNumNonZeroCoeffsNonTS( const TransformUnit& tu, const bool bLuma, const bool bChroma )
{
  uint32_t count = 0;
  for( uint32_t i = 0; i < ::getNumberValidTBlocks( *tu.cs->pcv ); i++ )
  {
    if( tu.blocks[ i ].valid() && tu.mtsIdx != MTS_SKIP && TU::getCbf( tu, ComponentID( i ) ) )
    {
      if( isLuma  ( tu.blocks[i].compID ) && !bLuma   ) continue;
      if( isChroma( tu.blocks[i].compID ) && !bChroma ) continue;

      uint32_t area = tu.blocks[i].area();
      const TCoeff* coeff = tu.getCoeffs( ComponentID( i ) ).buf;
      for( uint32_t j = 0; j < area; j++ )
      {
        count += coeff[j] != 0;
      }
    }
  }
  return count;
}
#endif
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
uint32_t TU::getNumNonZeroCoeffsNonTSCorner8x8( const TransformUnit& tu, const bool lumaFlag, const bool chromaFlag )
{
  const uint32_t lumaWidth       = tu.blocks[ 0 ].width,  chromaWidth  = tu.blocks[ 1 ].width;
  const uint32_t lumaHeight      = tu.blocks[ 0 ].height, chromaHeight = tu.blocks[ 1 ].height;
  bool           luma4x4TUFlag   = lumaWidth     == 4 && lumaHeight   == 4;
  bool           chroma4x4TUFlag = chromaWidth   == 4 && chromaHeight == 4;
  bool           luma8x8TUFlag   = lumaWidth     == 8 && lumaHeight   == 8;
  bool           chroma8x8TUFlag = chromaWidth   == 8 && chromaHeight == 8;
  bool           lumaCountFlag   = ( lumaWidth   >= 8 && lumaHeight   >= 8 ) || luma4x4TUFlag;
  bool           chromaCountFlag = ( chromaWidth >= 8 && chromaHeight >= 8 ) || chroma4x4TUFlag;

  uint32_t count = 0;
  for( uint32_t i = 0; i < ::getNumberValidTBlocks( *tu.cs->pcv ); i++ )
  {
    if( tu.blocks[ i ].valid() && tu.mtsIdx != MTS_SKIP && TU::getCbf( tu, ComponentID( i ) ) )
    {
      if(   isLuma( tu.blocks[ i ].compID ) && (   !lumaFlag ||   !lumaCountFlag ) ) continue;
      if( isChroma( tu.blocks[ i ].compID ) && ( !chromaFlag || !chromaCountFlag ) ) continue;

      const ScanElement * scan  = g_coefTopLeftDiagScan8x8[ gp_sizeIdxInfo->idxFrom( tu.blocks[ i ].width ) ];
      const TCoeff*       coeff = tu.getCoeffs( ComponentID( i ) ).buf;

      int startPos = MAX_LFNST_COEF_NUM, endPos = 47;
      if( ( isLuma( tu.blocks[ i ].compID ) && luma4x4TUFlag ) || ( isChroma( tu.blocks[ i ].compID ) && chroma4x4TUFlag ) )
      {
        startPos = 8; endPos = 15;
      }
      else if( ( isLuma( tu.blocks[ i ].compID ) && luma8x8TUFlag ) || ( isChroma( tu.blocks[ i ].compID ) && chroma8x8TUFlag ) )
      {
        startPos = 8; endPos = 47;
      }
      const ScanElement *scanPtr = scan + startPos;
      for( uint32_t j = startPos; j <= endPos; j++ )
      {
        count += coeff[ scanPtr->idx ] != 0;
        scanPtr++;
      }
    }
  }
  return count;
}
#endif

bool TU::needsSqrt2Scale( const TransformUnit &tu, const ComponentID &compID )
{
  const Size &size=tu.blocks[compID];
  const bool isTransformSkip = tu.mtsIdx==MTS_SKIP && isLuma(compID);
  return (!isTransformSkip) && (((floorLog2(size.width) + floorLog2(size.height)) & 1) == 1);
}

bool TU::needsBlockSizeTrafoScale( const TransformUnit &tu, const ComponentID &compID )
{
  return needsSqrt2Scale( tu, compID ) || isNonLog2BlockSize( tu.blocks[compID] );
}

TransformUnit* TU::getPrevTU( const TransformUnit &tu, const ComponentID compID )
{
  TransformUnit* prevTU = tu.prev;

  if( prevTU != nullptr && ( prevTU->cu != tu.cu || !prevTU->blocks[compID].valid() ) )
  {
    prevTU = nullptr;
  }

  return prevTU;
}

bool TU::getPrevTuCbfAtDepth( const TransformUnit &currentTu, const ComponentID compID, const int trDepth )
{
  const TransformUnit* prevTU = getPrevTU( currentTu, compID );
  return ( prevTU != nullptr ) ? TU::getCbfAtDepth( *prevTU, compID, trDepth ) : false;
}


// other tools

uint32_t getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}

int getNumModesMip(const Size& block)
{
  if (block.width > (4 * block.height) || block.height > (4 * block.width))
  {
    return 0;
  }

  if( block.width == 4 && block.height == 4 )
  {
    return 35;
  }
  else if (block.width <= 8 && block.height <= 8)
  {
    return 19;
  }
  else
  {
    return 11;
  }
}

#if !JVET_O0925_MIP_SIMPLIFICATIONS
int getNumEpBinsMip(const Size& block)
{
  int numModes = getNumModesMip(block);
  return int(std::ceil((std::log2(numModes - NUM_MPM_MIP - 1))));
}
#endif

bool mipModesAvailable(const Size& block)
{
  return (getNumModesMip(block));
}

#if JVET_O0925_MIP_SIMPLIFICATIONS
bool allowLfnstWithMip(const Size& block)
{
  if (block.width >= 16 && block.height >= 16)
  {
    return true;
  }
  return false;
}
#endif

#if JVET_O1164_RPR
bool PU::isRefPicSameSize( const PredictionUnit& pu )
{
  bool samePicSize = true;
  int curPicWidth = pu.cs->pps->getPicWidthInLumaSamples();
  int curPicHeight = pu.cs->pps->getPicHeightInLumaSamples();

  if( pu.refIdx[0] >= 0 )
  {
    int refPicWidth = pu.cu->slice->getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->unscaledPic->cs->pps->getPicWidthInLumaSamples();
    int refPicHeight = pu.cu->slice->getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->unscaledPic->cs->pps->getPicHeightInLumaSamples();

    samePicSize = refPicWidth == curPicWidth && refPicHeight == curPicHeight;
  }

  if( pu.refIdx[1] >= 0 )
  {
    int refPicWidth = pu.cu->slice->getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->unscaledPic->cs->pps->getPicWidthInLumaSamples();
    int refPicHeight = pu.cu->slice->getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->unscaledPic->cs->pps->getPicHeightInLumaSamples();

    samePicSize = samePicSize && ( refPicWidth == curPicWidth && refPicHeight == curPicHeight );
  }

  return samePicSize;
}
#endif



