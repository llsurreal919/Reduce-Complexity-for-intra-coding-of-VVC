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

/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#ifndef __UNITTOOLS__
#define __UNITTOOLS__

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

// CS tools
namespace CS
{
  uint64_t getEstBits                   ( const CodingStructure &cs );
  UnitArea getArea                    ( const CodingStructure &cs, const UnitArea &area, const ChannelType chType );
  bool   isDualITree                  ( const CodingStructure &cs );
  void   setRefinedMotionField(CodingStructure &cs);
}


// CU tools
namespace CU
{
  bool isIntra                        (const CodingUnit &cu);
  bool isInter                        (const CodingUnit &cu);
  bool isIBC                          (const CodingUnit &cu);
#if JVET_O0119_BASE_PALETTE_444
  bool isPLT                          (const CodingUnit &cu);
#endif
  bool isRDPCMEnabled                 (const CodingUnit &cu);
  bool isLosslessCoded                (const CodingUnit &cu);

  bool isSameCtu                      (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSlice                    (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameTile                     (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSliceAndTile             (const CodingUnit &cu, const CodingUnit &cu2);
  bool isLastSubCUOfCtu               (const CodingUnit &cu);
  uint32_t getCtuAddr                     (const CodingUnit &cu);

  int  predictQP                      (const CodingUnit& cu, const int prevQP );

  uint32_t getNumPUs                      (const CodingUnit& cu);
  void addPUs                         (      CodingUnit& cu);

  PartSplit getSplitAtDepth           (const CodingUnit& cu, const unsigned depth);
#if JVET_O0050_LOCAL_DUAL_TREE
  ModeType  getModeTypeAtDepth        (const CodingUnit& cu, const unsigned depth);
#endif

#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  uint32_t getNumNonZeroCoeffNonTs         ( const CodingUnit& cu, const bool lumaFlag = true, const bool chromaFlag = true );
#endif
  uint32_t getNumNonZeroCoeffNonTsCorner8x8( const CodingUnit& cu, const bool lumaFlag = true, const bool chromaFlag = true );
#if JVET_O0106_ISP_4xN_PREDREG_FOR_1xN_2xN
  bool  isPredRegDiffFromTB(const CodingUnit& cu, const ComponentID compID);
  bool  isFirstTBInPredReg(const CodingUnit& cu, const ComponentID compID, const CompArea &area);
  bool  isMinWidthPredEnabledForBlkSize(const int w, const int h);
  void  adjustPredArea(CompArea &area);
#endif
  bool  isGBiIdxCoded                 (const CodingUnit& cu);
  uint8_t getValidGbiIdx              (const CodingUnit& cu);
  void  setGbiIdx                     (CodingUnit& cu, uint8_t uh);
  uint8_t deriveGbiIdx                (uint8_t gbiLO, uint8_t gbiL1);
  bool bdpcmAllowed                   (const CodingUnit& cu, const ComponentID compID);


  bool      divideTuInRows            ( const CodingUnit &cu );
#if !JVET_O0502_ISP_CLEANUP
  bool      firstTestISPHorSplit      ( const int width, const int height,            const ComponentID compID, const CodingUnit *cuLeft = nullptr, const CodingUnit *cuAbove = nullptr );
#endif
  PartSplit getISPType                ( const CodingUnit &cu,                         const ComponentID compID );
  bool      isISPLast                 ( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID );
  bool      isISPFirst                ( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID );
  bool      canUseISP                 ( const CodingUnit &cu,                         const ComponentID compID );
  bool      canUseISP                 ( const int width, const int height, const int maxTrSize = MAX_TB_SIZEY );
  uint32_t  getISPSplitDim            ( const int width, const int height, const PartSplit ispType );
#if JVET_O0502_ISP_CLEANUP
  bool      allLumaCBFsAreZero        ( const CodingUnit& cu );
#endif

  PUTraverser traversePUs             (      CodingUnit& cu);
  TUTraverser traverseTUs             (      CodingUnit& cu);
  cPUTraverser traversePUs            (const CodingUnit& cu);
  cTUTraverser traverseTUs            (const CodingUnit& cu);

  bool  hasSubCUNonZeroMVd            (const CodingUnit& cu);
  bool  hasSubCUNonZeroAffineMVd      ( const CodingUnit& cu );

  uint8_t getSbtInfo                  (uint8_t idx, uint8_t pos);
  uint8_t getSbtIdx                   (const uint8_t sbtInfo);
  uint8_t getSbtPos                   (const uint8_t sbtInfo);
  uint8_t getSbtMode                  (const uint8_t sbtIdx, const uint8_t sbtPos);
  uint8_t getSbtIdxFromSbtMode        (const uint8_t sbtMode);
  uint8_t getSbtPosFromSbtMode        (const uint8_t sbtMode);
  uint8_t targetSbtAllowed            (uint8_t idx, uint8_t sbtAllowed);
  uint8_t numSbtModeRdo               (uint8_t sbtAllowed);
  bool    isSbtMode                   (const uint8_t sbtInfo);
  bool    isSameSbtSize               (const uint8_t sbtInfo1, const uint8_t sbtInfo2);
#if JVET_O1164_RPR
  bool    getRprScaling               ( const SPS* sps, const PPS* curPPS, const PPS* refPPS, int& xScale, int& yScale );
#endif
}
// PU tools
namespace PU
{
  int  getLMSymbolList(const PredictionUnit &pu, int *modeList);
  int  getIntraMPMs(const PredictionUnit &pu, unsigned *mpm, const ChannelType &channelType = CHANNEL_TYPE_LUMA);
  bool          isMIP                 (const PredictionUnit &pu, const ChannelType &chType = CHANNEL_TYPE_LUMA);
#if !JVET_O0925_MIP_SIMPLIFICATIONS
  int           getMipMPMs            (const PredictionUnit &pu, unsigned *mpm);
#endif
  int           getMipSizeId          (const PredictionUnit &pu);
  uint32_t      getIntraDirLuma       (const PredictionUnit &pu);
#if !JVET_O0925_MIP_SIMPLIFICATIONS
  AvailableInfo getAvailableInfoLuma  (const PredictionUnit &pu);
#endif
  void getIntraChromaCandModes        (const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE]);
  uint32_t getFinalIntraMode              (const PredictionUnit &pu, const ChannelType &chType);
#if JVET_O0219_LFNST_TRANSFORM_SET_FOR_LMCMODE
  uint32_t getCoLocatedIntraLumaMode      (const PredictionUnit &pu);
#endif
  int getWideAngIntraMode             ( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID );
  void getInterMergeCandidates        (const PredictionUnit &pu, MergeCtx& mrgCtx,
    int mmvdList,
    const int& mrgCandIdx = -1 );
  void getIBCMergeCandidates          (const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
  void getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
  int getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC);
  bool isDiffMER                      (const PredictionUnit &pu, const PredictionUnit &pu2);
  bool getColocatedMVP                (const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &pos, Mv& rcMv, const int &refIdx, bool sbFlag);
  void fillMvpCand                    (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo );
  void fillIBCMvpCand                 (PredictionUnit &pu, AMVPInfo &amvpInfo);
  void fillAffineMvpCand              (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo);
  bool addMVPCandUnscaled             (const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
#if !JVET_O0164_REMOVE_AMVP_SPATIAL_SCALING
  bool addMVPCandWithScaling          (const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
#endif
  void xInheritedAffineMv             ( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] );
  bool xCheckSimilarMotion(const int mergeCandIndex, const int prevCnt, const MergeCtx mergeCandList, bool hasPruned[MRG_MAX_NUM_CANDS]);
#if JVET_L0090_PAIR_AVG
  bool addMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
    , bool ibcFlag
    , bool isShared
  );
#else
  bool addMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, bool isCandInter[MRG_MAX_NUM_CANDS], bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
  );
#endif
  void addAMVPHMVPCand(const PredictionUnit &pu, const RefPicList eRefPicList, const RefPicList eRefPicList2nd, const int currRefPOC, AMVPInfo &info, uint8_t imv);
  bool addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAmvpInfo );
  bool isBipredRestriction            (const PredictionUnit &pu);
  void spanMotionInfo                 (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx() );
  void applyImv                       (      PredictionUnit &pu, MergeCtx &mrgCtx, InterPrediction *interPred = NULL );
#if JVET_O0366_AFFINE_BCW
  void getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t gbiIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgCtx);
#else
  void getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], int8_t neighGbi[4], bool isAvailable[4], int verIdx[4], int modelIdx, int verNum, AffineMergeCtx& affMrgCtx);
#endif
  void getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx = -1 );
  void setAllAffineMvField            (      PredictionUnit &pu, MvField *mvField, RefPicList eRefList );
  void setAllAffineMv                 (      PredictionUnit &pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList
    , bool clipCPMVs = false
  );
  bool getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx &mrgCtx, bool& LICFlag, const int count
    , int mmvdList
  );
  bool getInterMergeSubPuRecurCand(const PredictionUnit &pu, MergeCtx &mrgCtx, const int count);
  bool isBiPredFromDifferentDir       (const PredictionUnit &pu);
  bool isBiPredFromDifferentDirEqDistPoc(const PredictionUnit &pu);
  void restrictBiPredMergeCandsOne    (PredictionUnit &pu);

  bool isLMCMode                      (                          unsigned mode);
  bool isLMCModeEnabled               (const PredictionUnit &pu, unsigned mode);
  bool isChromaIntraModeCrossCheckMode(const PredictionUnit &pu);
  void getTriangleMergeCandidates     (const PredictionUnit &pu, MergeCtx &triangleMrgCtx);
  void spanTriangleMotionInfo         (      PredictionUnit &pu, MergeCtx &triangleMrgCtx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1);
  int32_t mappingRefPic               (const PredictionUnit &pu, int32_t refPicPoc, bool targetRefPicList);
  bool isAddNeighborMv  (const Mv& currMv, Mv* neighborMvs, int numNeighborMv);
  void getIbcMVPsEncOnly(PredictionUnit &pu, Mv* mvPred, int& nbPred);
  bool getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv);
#if !JVET_O1170_IBC_VIRTUAL_BUFFER
  bool isBlockVectorValid(PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xStartInCU, int yStartInCU, int xBv, int yBv, int ctuSize);
#endif
  bool checkDMVRCondition(const PredictionUnit& pu);

#if JVET_O1164_RPR
  bool isRefPicSameSize( const PredictionUnit& pu );
#endif
}

// TU tools
namespace TU
{
#if !JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  uint32_t getNumNonZeroCoeffsNonTS       (const TransformUnit &tu, const bool bLuma = true, const bool bChroma = true);
#endif
  uint32_t getNumNonZeroCoeffsNonTSCorner8x8( const TransformUnit &tu, const bool bLuma = true, const bool bChroma = true );
  bool isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID);
  bool getCbf                         (const TransformUnit &tu, const ComponentID &compID);
  bool getCbfAtDepth                  (const TransformUnit &tu, const ComponentID &compID, const unsigned &depth);
  void setCbfAtDepth                  (      TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf);
  bool isTSAllowed                    (const TransformUnit &tu, const ComponentID  compID);
  bool isMTSAllowed                   (const TransformUnit &tu, const ComponentID  compID);
  bool hasCrossCompPredInfo           (const TransformUnit &tu, const ComponentID &compID);


  bool needsSqrt2Scale                ( const TransformUnit &tu, const ComponentID &compID );
  bool needsBlockSizeTrafoScale       ( const TransformUnit &tu, const ComponentID &compID );
  TransformUnit* getPrevTU          ( const TransformUnit &tu, const ComponentID compID );
  bool           getPrevTuCbfAtDepth( const TransformUnit &tu, const ComponentID compID, const int trDepth );
#if JVET_O0105_ICT
  int            getICTMode         ( const TransformUnit &tu, int jointCbCr = -1 );
#endif
}

uint32_t getCtuAddr        (const Position& pos, const PreCalcValues &pcv);
int  getNumModesMip   (const Size& block);
#if !JVET_O0925_MIP_SIMPLIFICATIONS
int  getNumEpBinsMip  (const Size& block);
#endif
bool mipModesAvailable(const Size& block);
#if JVET_O0925_MIP_SIMPLIFICATIONS
bool allowLfnstWithMip(const Size& block);
#endif

template<typename T, size_t N>
uint32_t updateCandList(T uiMode, double uiCost, static_vector<T, N>& candModeList, static_vector<double, N>& candCostList
  , size_t uiFastCandNum = N, int* iserttPos = nullptr)
{
  CHECK( std::min( uiFastCandNum, candModeList.size() ) != std::min( uiFastCandNum, candCostList.size() ), "Sizes do not match!" );
  CHECK( uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!" );

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min( uiFastCandNum, candCostList.size() );

  while( shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift] )
  {
    shift++;
  }

  if( candModeList.size() >= uiFastCandNum && shift != 0 )
  {
    for( i = 1; i < shift; i++ )
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    candCostList[currSize - shift] = uiCost;
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if( currSize < uiFastCandNum )
  {
    candModeList.insert( candModeList.end() - shift, uiMode );
    candCostList.insert( candCostList.end() - shift, uiCost );
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }
  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}

#endif
