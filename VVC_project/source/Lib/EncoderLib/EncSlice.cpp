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

/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"

#include "EncLib.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Picture.h"
#if K0149_BLOCK_STATISTICS
#include "CommonLib/dtrace_blockstatistics.h"
#endif

#if ENABLE_WPP_PARALLELISM
#include <mutex>
extern recursive_mutex g_cache_mutex;
#endif

#include <math.h>

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
 : m_encCABACTableIdx(I_SLICE)
#if ENABLE_QPA
 , m_adaptedLumaQP(-1)
#endif
{
}

EncSlice::~EncSlice()
{
  destroy();
}

void EncSlice::create( int iWidth, int iHeight, ChromaFormat chromaFormat, uint32_t iMaxCUWidth, uint32_t iMaxCUHeight, uint8_t uhTotalDepth )
{
}

void EncSlice::destroy()
{
  // free lambda and QP arrays
  m_vdRdPicLambda.clear();
  m_vdRdPicQp.clear();
  m_viRdPicQp.clear();
}

void EncSlice::init( EncLib* pcEncLib, const SPS& sps )
{
  m_pcCfg             = pcEncLib;
  m_pcLib             = pcEncLib;
  m_pcListPic         = pcEncLib->getListPic();

  m_pcGOPEncoder      = pcEncLib->getGOPEncoder();
  m_pcCuEncoder       = pcEncLib->getCuEncoder();
  m_pcInterSearch     = pcEncLib->getInterSearch();
  m_CABACWriter       = pcEncLib->getCABACEncoder()->getCABACWriter   (&sps);
  m_CABACEstimator    = pcEncLib->getCABACEncoder()->getCABACEstimator(&sps);
  m_pcTrQuant         = pcEncLib->getTrQuant();
  m_pcRdCost          = pcEncLib->getRdCost();

  // create lambda and QP arrays
  m_vdRdPicLambda.resize(m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_vdRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_viRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pcRateCtrl        = pcEncLib->getRateCtrl();
}

void
EncSlice::setUpLambda( Slice* slice, const double dLambda, int iQP)
{
  // store lambda
  m_pcRdCost ->setLambda( dLambda, slice->getSPS()->getBitDepths() );

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
  for( uint32_t compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    int chromaQPOffset       = slice->getPPS()->getQpOffset( compID ) + slice->getSliceChromaQpDelta( compID );
#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
    int qpc = slice->getSPS()->getMappedChromaQpValue(compID, iQP) + chromaQPOffset;
#else
    int qpc                  = ( iQP + chromaQPOffset < 0 ) ? iQP : getScaledChromaQP( iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc() );
#endif
    double tmpWeight         = pow( 2.0, ( iQP - qpc ) / 3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
    if( m_pcCfg->getDepQuantEnabledFlag() && !( m_pcCfg->getLFNST() ) )
    {
      tmpWeight *= ( m_pcCfg->getGOPSize() >= 8 ? pow( 2.0, 0.1/3.0 ) : pow( 2.0, 0.2/3.0 ) );  // increase chroma weight for dependent quantization (in order to reduce bit rate shift from chroma to luma)
    }
    m_pcRdCost->setDistortionWeight( compID, tmpWeight );
#if ENABLE_WPP_PARALLELISM
    for( int jId = 1; jId < ( m_pcLib->getNumWppThreads() + m_pcLib->getNumWppExtraLines() ); jId++ )
    {
      m_pcLib->getRdCost( slice->getPic()->scheduler.getWppDataId( jId ) )->setDistortionWeight( compID, tmpWeight );
    }
#endif
    dLambdas[compIdx] = dLambda / tmpWeight;
  }

#if RDOQ_CHROMA_LAMBDA
  // for RDOQ
  m_pcTrQuant->setLambdas( dLambdas );
#else
  m_pcTrQuant->setLambda( dLambda );
#endif

  // for SAO
  slice->setLambdas( dLambdas );
}

#if ENABLE_QPA

static inline int apprI3Log2 (const double d) // rounded 3*log2(d)
{
  return d < 1.5e-13 ? -128 : int (floor (3.0 * log (d) / log (2.0) + 0.5));
}

static inline int lumaDQPOffset (const uint32_t avgLumaValue, const int bitDepth)
{
  return (1 - int ((3 * uint64_t (avgLumaValue * avgLumaValue)) >> uint64_t (2 * bitDepth - 1)));
}

static void filterAndCalculateAverageEnergies (const Pel* pSrc, const int  iSrcStride,
                                               double &hpEner,  const int  iHeight,    const int iWidth,
                                               const uint32_t uBitDepth /* luma bit-depth (4-16) */)
{
  uint64_t saAct = 0;

  // skip first row as there may be a black border frame
  pSrc += iSrcStride;
  // center rows
  for (int y = 1; y < iHeight - 1; y++)
  {
    // skip column as there may be a black border frame

    for (int x = 1; x < iWidth - 1; x++) // and columns
    {
      const int f = 12 * (int)pSrc[x  ] - 2 * ((int)pSrc[x-1] + (int)pSrc[x+1] + (int)pSrc[x  -iSrcStride] + (int)pSrc[x  +iSrcStride])
                       - (int)pSrc[x-1-iSrcStride] - (int)pSrc[x+1-iSrcStride] - (int)pSrc[x-1+iSrcStride] - (int)pSrc[x+1+iSrcStride];
      saAct += abs (f);
    }
    // skip column as there may be a black border frame
    pSrc += iSrcStride;
  }
  // skip last row as there may be a black border frame

  hpEner = double(saAct) / double((iWidth - 2) * (iHeight - 2));

  // lower limit, compensate for highpass amplification
  if (hpEner < double(1 << (uBitDepth - 4))) hpEner = double(1 << (uBitDepth - 4));
}

#ifndef GLOBAL_AVERAGING
  #define GLOBAL_AVERAGING 1 // "global" averaging of a_k across a set instead of one picture
#endif

#if GLOBAL_AVERAGING
static double getAveragePictureEnergy (const CPelBuf picOrig, const uint32_t uBitDepth)
{
  const double hpEnerPic = 16.0 * sqrt ((3840.0 * 2160.0) / double(picOrig.width * picOrig.height)) * double(1 << uBitDepth);

  return sqrt (hpEnerPic); // square-root of a_pic value
}
#endif

static int getGlaringColorQPOffset (Picture* const pcPic, const int ctuAddr, const uint32_t startAddr, const uint32_t boundingAddr,
                                    const int bitDepth,   uint32_t &avgLumaValue)
{
  const PreCalcValues& pcv  = *pcPic->cs->pcv;
  const ChromaFormat chrFmt = pcPic->chromaFormat;
  const uint32_t chrWidth   = pcv.maxCUWidth  >> getChannelTypeScaleX (CH_C, chrFmt);
  const uint32_t chrHeight  = pcv.maxCUHeight >> getChannelTypeScaleY (CH_C, chrFmt);
  const int      midLevel   = 1 << (bitDepth - 1);
  int chrValue = MAX_INT;
  avgLumaValue = (startAddr < boundingAddr) ? 0 : (uint32_t)pcPic->getOrigBuf().Y().computeAvg();

  if (ctuAddr >= 0) // luma
  {
    avgLumaValue = (uint32_t)pcPic->m_iOffsetCtu[ctuAddr];
  }
  else if (startAddr < boundingAddr)
  {
    for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr = pcPic->brickMap->getCtuBsToRsAddrMap (ctuTsAddr);
      avgLumaValue += pcPic->m_iOffsetCtu[ctuRsAddr];
    }
    avgLumaValue = (avgLumaValue + ((boundingAddr - startAddr) >> 1)) / (boundingAddr - startAddr);
  }

  for (uint32_t comp = COMPONENT_Cb; comp < MAX_NUM_COMPONENT; comp++)
  {
    const ComponentID compID = (ComponentID)comp;
    int avgCompValue;

    if (ctuAddr >= 0) // chroma
    {
      const CompArea chrArea = clipArea (CompArea (compID, chrFmt, Area ((ctuAddr % pcv.widthInCtus) * chrWidth, (ctuAddr / pcv.widthInCtus) * chrHeight, chrWidth, chrHeight)), pcPic->block (compID));

      avgCompValue = pcPic->getOrigBuf (chrArea).computeAvg();
    }
    else avgCompValue = pcPic->getOrigBuf (pcPic->block (compID)).computeAvg();

    if (chrValue > avgCompValue) chrValue = avgCompValue; // minimum of the DC offsets
  }
  CHECK (chrValue < 0, "DC offset cannot be negative!");

  chrValue = (int)avgLumaValue - chrValue;

  if (chrValue > midLevel) return apprI3Log2 (double (chrValue * chrValue) / double (midLevel * midLevel));

  return 0;
}

static int applyQPAdaptationChroma (Picture* const pcPic, Slice* const pcSlice, EncCfg* const pcEncCfg, const int sliceQP)
{
  const int bitDepth               = pcSlice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA); // overall image bit-depth
  double hpEner[MAX_NUM_COMPONENT] = {0.0, 0.0, 0.0};
  int    optSliceChromaQpOffset[2] = {0, 0};
  int    savedLumaQP               = -1;
  uint32_t meanLuma                = MAX_UINT;

  for (uint32_t comp = 0; comp < getNumberValidComponents (pcPic->chromaFormat); comp++)
  {
    const ComponentID compID = (ComponentID)comp;
    const CPelBuf    picOrig = pcPic->getOrigBuf (pcPic->block (compID));

    filterAndCalculateAverageEnergies (picOrig.buf,    picOrig.stride, hpEner[comp],
                                       picOrig.height, picOrig.width,  bitDepth - (isChroma (compID) ? 1 : 0));
    if (isChroma (compID))
    {
      const int  adaptChromaQPOffset = 2.0 * hpEner[comp] <= hpEner[0] ? 0 : apprI3Log2 (2.0 * hpEner[comp] / hpEner[0]);

      if (savedLumaQP < 0)
      {
#if GLOBAL_AVERAGING
        int     averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEner[0] / getAveragePictureEnergy (pcPic->getOrigBuf().Y(), bitDepth)));
#else
        int     averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP); // mean slice QP
#endif

        averageAdaptedLumaQP += getGlaringColorQPOffset (pcPic, -1 /*ctuRsAddr*/, 0 /*startAddr*/, 0 /*boundingAddr*/, bitDepth, meanLuma);

        if (averageAdaptedLumaQP > MAX_QP
#if SHARP_LUMA_DELTA_QP
            && (pcEncCfg->getLumaLevelToDeltaQPMapping().mode != LUMALVL_TO_DQP_NUM_MODES)
#endif
            ) averageAdaptedLumaQP = MAX_QP;
#if SHARP_LUMA_DELTA_QP

        // change mean picture QP index based on picture's average luma value (Sharp)
        if (pcEncCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES)
        {
          if (meanLuma == MAX_UINT) meanLuma = pcPic->getOrigBuf().Y().computeAvg();

          averageAdaptedLumaQP = Clip3 (0, MAX_QP, averageAdaptedLumaQP + lumaDQPOffset (meanLuma, bitDepth));
        }
#endif

        savedLumaQP = averageAdaptedLumaQP;
      } // savedLumaQP < 0

#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
      const int lumaChromaMappingDQP = savedLumaQP - pcSlice->getSPS()->getMappedChromaQpValue(compID, savedLumaQP);
#else
      const int lumaChromaMappingDQP = savedLumaQP - getScaledChromaQP (savedLumaQP, pcEncCfg->getChromaFormatIdc());
#endif

      optSliceChromaQpOffset[comp-1] = std::min (3 + lumaChromaMappingDQP, adaptChromaQPOffset + lumaChromaMappingDQP);
    }
  }

  pcEncCfg->setSliceChromaOffsetQpIntraOrPeriodic (pcEncCfg->getSliceChromaOffsetQpPeriodicity(), optSliceChromaQpOffset);

  return savedLumaQP;
}

#endif // ENABLE_QPA

/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param iNumPicRcvd   number of received pictures
 \param iGOPid        POC offset for hierarchical structure
 \param rpcSlice      slice header class
 \param isField       true for field coding
 */
void EncSlice::initEncSlice(Picture* pcPic, const int pocLast, const int pocCurr, const int iGOPid, Slice*& rpcSlice, const bool isField
  , bool isEncodeLtRef
)
{
  double dQP;
  double dLambda;
#if JVET_O0119_BASE_PALETTE_444
  pcPic->cs->resetPrevPLT(pcPic->cs->prevPLT);
#endif

  rpcSlice = pcPic->slices[0];
  rpcSlice->setSliceBits(0);
  rpcSlice->setPic( pcPic );
  rpcSlice->initSlice();
  int multipleFactor = m_pcCfg->getUseCompositeRef() ? 2 : 1;
  if (m_pcCfg->getUseCompositeRef() && isEncodeLtRef)
  {
    rpcSlice->setPicOutputFlag(false);
  }
  else
  {
    rpcSlice->setPicOutputFlag(true);
  }
  rpcSlice->setPOC( pocCurr );
  rpcSlice->setDepQuantEnabledFlag( m_pcCfg->getDepQuantEnabledFlag() );
  rpcSlice->setSignDataHidingEnabledFlag( m_pcCfg->getSignDataHidingEnabledFlag() );

#if SHARP_LUMA_DELTA_QP
  pcPic->fieldPic = isField;
  m_gopID = iGOPid;
#endif

  // depth computation based on GOP size
  int depth;
  {
    int poc = rpcSlice->getPOC();
    if(isField)
    {
      poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
    }
    else
    {
      poc = poc % (m_pcCfg->getGOPSize() * multipleFactor);
    }

    if ( poc == 0 )
    {
      depth = 0;
    }
    else
    {
      int step = m_pcCfg->getGOPSize() * multipleFactor;
      depth    = 0;
      for( int i=step>>1; i>=1; i>>=1 )
      {
        for (int j = i; j<(m_pcCfg->getGOPSize() * multipleFactor); j += step)
        {
          if ( j == poc )
          {
            i=0;
            break;
          }
        }
        step >>= 1;
        depth++;
      }
    }

    if(m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
    {
      if (isField && ((rpcSlice->getPOC() % 2) == 1))
      {
        depth++;
      }
    }
  }

  // slice type
  SliceType eSliceType;

  eSliceType=B_SLICE;
  if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    if(m_pcCfg->getDecodingRefreshType() == 3)
    {
      eSliceType = (pocLast == 0 || pocCurr % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
    else
    {
      eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
  }

  rpcSlice->setSliceType    ( eSliceType );

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking
  // ------------------------------------------------------------------------------------------------------------------

  pcPic->referenced = true;

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice);
#else
  dQP = m_pcCfg->getBaseQP();
  if(eSliceType!=I_SLICE)
  {
#if SHARP_LUMA_DELTA_QP
    if (!(( m_pcCfg->getMaxDeltaQP() == 0) && (!m_pcCfg->getLumaLevelToDeltaQPMapping().isEnabled()) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#else
    if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#endif
    {
      dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
    }
  }

  // modify QP
  const int* pdQPs = m_pcCfg->getdQPs();
  if ( pdQPs )
  {
    dQP += pdQPs[ rpcSlice->getPOC() ];
  }

  if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
  {
    dQP=LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
    m_pcCfg->setDeltaQpRD(0);
  }
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
#if !SHARP_LUMA_DELTA_QP
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif
#endif
  int iQP;
  double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for ( int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
#if SHARP_LUMA_DELTA_QP
    dLambda = calculateLambda(rpcSlice, iGOPid, depth, dQP, dQP, iQP );
#else
    // compute lambda value
    int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
    int    SHIFT_QP = 12;

    int    bitdepth_luma_qp_scale =
      6
      * (rpcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
         - DISTORTION_PRECISION_ADJUSTMENT(rpcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
    double qp_temp = (double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
    double qp_temp_orig = (double) dQP - SHIFT_QP;
#endif
    // Case #1: I or P-slices (key-frame)
    double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
    if ( eSliceType==I_SLICE )
    {
      if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(iGOPid).m_sliceType != I_SLICE)
      {
        dQPFactor=m_pcCfg->getIntraQpFactor();
      }
      else
      {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        if(m_pcCfg->getLambdaFromQPEnable())
        {
          dQPFactor=0.57;
        }
        else
        {
#endif
        double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? NumberBFrames/2 : NumberBFrames) );

        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        }
#endif
      }
    }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
    else if( m_pcCfg->getLambdaFromQPEnable() )
    {
      dQPFactor=0.57;
    }
#endif

    dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    if(!m_pcCfg->getLambdaFromQPEnable() && depth>0)
#else
    if ( depth>0 )
#endif
    {
#if FULL_NBIT
        dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
        dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
    }

    // if hadamard is used in ME process
    if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
    {
      dLambda *= 0.95;
    }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    double lambdaModifier;
    if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
    {
      lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
    }
    else
    {
      lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
    }
    dLambda *= lambdaModifier;
#endif

    iQP = Clip3( -rpcSlice->getSPS()->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, (int) floor( dQP + 0.5 ) );
#endif

    m_vdRdPicLambda[iDQpIdx] = dLambda;
    m_vdRdPicQp    [iDQpIdx] = dQP;
    m_viRdPicQp    [iDQpIdx] = iQP;
  }

  // obtain dQP = 0 case
  dLambda = m_vdRdPicLambda[0];
  dQP     = m_vdRdPicQp    [0];
  iQP     = m_viRdPicQp    [0];

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

#if W0038_CQP_ADJ
 #if ENABLE_QPA
  m_adaptedLumaQP = -1;

  if ((m_pcCfg->getUsePerceptQPA() || m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0) && !m_pcCfg->getUseRateCtrl() && rpcSlice->getPPS()->getSliceChromaQpFlag() &&
      (rpcSlice->isIntra() || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0)))
  {
    m_adaptedLumaQP = applyQPAdaptationChroma (pcPic, rpcSlice, m_pcCfg, iQP);
  }
 #endif
  if(rpcSlice->getPPS()->getSliceChromaQpFlag())
  {
    const bool bUseIntraOrPeriodicOffset = (rpcSlice->isIntra() && !rpcSlice->getSPS()->getIBCFlag()) || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0);
    int cbQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(false) : m_pcCfg->getGOPEntry(iGOPid).m_CbQPoffset;
    int crQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(true)  : m_pcCfg->getGOPEntry(iGOPid).m_CrQPoffset;

    cbQP = Clip3( -12, 12, cbQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb);
    crQP = Clip3( -12, 12, crQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr);
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, Clip3( -12, 12, cbQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)>=-12), "Unspecified error");
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, Clip3( -12, 12, crQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)>=-12), "Unspecified error");
  }
  else
  {
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setSliceChromaQpDelta( JOINT_CbCr, 0 );
  }
#endif

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  double lambdaModifier;
  if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }

  dLambda *= lambdaModifier;
#endif

  setUpLambda(rpcSlice, dLambda, iQP);

#if WCG_EXT
  // cost = Distortion + Lambda*R,
  // when QP is adjusted by luma, distortion is changed, so we have to adjust lambda to match the distortion, then the cost function becomes
  // costA = Distortion + AdjustedLambda * R          -- currently, costA is still used when calculating intermediate cost of using SAD, HAD, resisual etc.
  // an alternative way is to weight the distortion to before the luma QP adjustment, then the cost function becomes
  // costB = weightedDistortion + Lambda * R          -- currently, costB is used to calculat final cost, and when DF_FUNC is DF_DEFAULT
  m_pcRdCost->saveUnadjustedLambda();
#endif

  if (m_pcCfg->getFastMEForGenBLowDelayEnabled())
  {
    // restore original slice type

    if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        eSliceType = (pocLast == 0 || (pocCurr) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
      else
      {
        eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
    }

    rpcSlice->setSliceType        ( eSliceType );
  }

  if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
  {
    dQP = xGetQPValueAccordingToLambda( dLambda );
    iQP = Clip3( -rpcSlice->getSPS()->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, (int) floor( dQP + 0.5 ) );
  }

  rpcSlice->setSliceQp           ( iQP );
  rpcSlice->setSliceQpDelta      ( 0 );
#if !W0038_CQP_ADJ
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
  rpcSlice->setSliceChromaQpDelta( JOINT_CbCr,   0 );
#endif
  rpcSlice->setUseChromaQpAdj( rpcSlice->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag() );
  rpcSlice->setNumRefIdx(REF_PIC_LIST_0, m_pcCfg->getRPLEntry(0, iGOPid).m_numRefPicsActive);
  rpcSlice->setNumRefIdx(REF_PIC_LIST_1, m_pcCfg->getRPLEntry(1, iGOPid).m_numRefPicsActive);

  if ( m_pcCfg->getDeblockingFilterMetric() )
  {
    rpcSlice->setDeblockingFilterOverrideFlag(true);
    rpcSlice->setDeblockingFilterDisable(false);
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }
  else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
  {
    rpcSlice->setDeblockingFilterOverrideFlag( rpcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() );
    rpcSlice->setDeblockingFilterDisable( rpcSlice->getPPS()->getPPSDeblockingFilterDisabledFlag() );
    if ( !rpcSlice->getDeblockingFilterDisable())
    {
      if ( rpcSlice->getDeblockingFilterOverrideFlag() && eSliceType!=I_SLICE)
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
      }
      else
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
      }
    }
  }
  else
  {
    rpcSlice->setDeblockingFilterOverrideFlag( false );
    rpcSlice->setDeblockingFilterDisable( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }

  rpcSlice->setDepth            ( depth );

  pcPic->layer =  temporalId;
  if(eSliceType==I_SLICE)
  {
    pcPic->layer = 0;
  }
  rpcSlice->setTLayer( pcPic->layer );

  rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
  rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
  rpcSlice->setMaxNumMergeCand      ( m_pcCfg->getMaxNumMergeCand()      );
  rpcSlice->setMaxNumAffineMergeCand( m_pcCfg->getMaxNumAffineMergeCand() );
  rpcSlice->setMaxNumTriangleCand   ( m_pcCfg->getMaxNumTriangleCand() );
#if JVET_O0455_IBC_MAX_MERGE_NUM
  rpcSlice->setMaxNumIBCMergeCand   ( m_pcCfg->getMaxNumIBCMergeCand() );
#endif
  rpcSlice->setSplitConsOverrideFlag(false);
  rpcSlice->setMinQTSize( rpcSlice->getSPS()->getMinQTSize(eSliceType));
  rpcSlice->setMaxBTDepth( rpcSlice->isIntra() ? rpcSlice->getSPS()->getMaxBTDepthI() : rpcSlice->getSPS()->getMaxBTDepth() );
  rpcSlice->setMaxBTSize( rpcSlice->isIntra() ? rpcSlice->getSPS()->getMaxBTSizeI() : rpcSlice->getSPS()->getMaxBTSize() );
  rpcSlice->setMaxTTSize( rpcSlice->isIntra() ? rpcSlice->getSPS()->getMaxTTSizeI() : rpcSlice->getSPS()->getMaxTTSize() );
  if ( eSliceType == I_SLICE && rpcSlice->getSPS()->getUseDualITree() )
  {
    rpcSlice->setMinQTSizeIChroma( rpcSlice->getSPS()->getMinQTSize(eSliceType, CHANNEL_TYPE_CHROMA) );
    rpcSlice->setMaxBTDepthIChroma( rpcSlice->getSPS()->getMaxBTDepthIChroma() );
    rpcSlice->setMaxBTSizeIChroma( rpcSlice->getSPS()->getMaxBTSizeIChroma() );
    rpcSlice->setMaxTTSizeIChroma( rpcSlice->getSPS()->getMaxTTSizeIChroma() );
  }
  rpcSlice->setDisableSATDForRD(false);

#if JVET_O1164_PS
  if( ( m_pcCfg->getIBCHashSearch() && m_pcCfg->getIBCMode() ) || m_pcCfg->getAllowDisFracMMVD() )
  {
    m_pcCuEncoder->getIbcHashMap().destroy();
    m_pcCuEncoder->getIbcHashMap().init( pcPic->cs->pps->getPicWidthInLumaSamples(), pcPic->cs->pps->getPicHeightInLumaSamples() );
  }
#endif
}


#if SHARP_LUMA_DELTA_QP
double EncSlice::calculateLambda( const Slice*     slice,
                                  const int        GOPid, // entry in the GOP table
                                  const int        depth, // slice GOP hierarchical depth.
                                  const double     refQP, // initial slice-level QP
                                  const double     dQP,   // initial double-precision QP
                                        int       &iQP )  // returned integer QP.
{
  enum   SliceType eSliceType    = slice->getSliceType();
  const  bool      isField       = slice->getPic()->fieldPic;
  const  int       NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
  const  int       SHIFT_QP      = 12;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(GOPid).m_temporalId;
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

  int bitdepth_luma_qp_scale = 6
                               * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                                  - DISTORTION_PRECISION_ADJUSTMENT(slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
  double qp_temp = dQP + bitdepth_luma_qp_scale - SHIFT_QP;
  // Case #1: I or P-slices (key-frame)
  double dQPFactor = m_pcCfg->getGOPEntry(GOPid).m_QPFactor;
  if ( eSliceType==I_SLICE )
  {
    if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(GOPid).m_sliceType != I_SLICE)
    {
      dQPFactor=m_pcCfg->getIntraQpFactor();
    }
    else
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      if(m_pcCfg->getLambdaFromQPEnable())
      {
        dQPFactor=0.57;
      }
      else
      {
#endif
        double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? NumberBFrames/2 : NumberBFrames) );
        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      }
#endif
    }
  }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  else if( m_pcCfg->getLambdaFromQPEnable() )
  {
    dQPFactor=0.57;
  }
#endif

  double dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  if( !(m_pcCfg->getLambdaFromQPEnable()) && depth>0 )
#else
  if ( depth>0 )
#endif
  {
    double qp_temp_ref = refQP + bitdepth_luma_qp_scale - SHIFT_QP;
    dLambda *= Clip3(2.00, 4.00, (qp_temp_ref / 6.0));   // (j == B_SLICE && p_cur_frm->layer != 0 )
  }

  // if hadamard is used in ME process
  if ( !m_pcCfg->getUseHADME() && slice->getSliceType( ) != I_SLICE )
  {
    dLambda *= 0.95;
  }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  double lambdaModifier;
  if( eSliceType != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }
  dLambda *= lambdaModifier;
#endif

  iQP = Clip3( -slice->getSPS()->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, (int) floor( dQP + 0.5 ) );

  if( m_pcCfg->getDepQuantEnabledFlag() )
  {
    dLambda *= pow( 2.0, 0.25/3.0 ); // slight lambda adjustment for dependent quantization (due to different slope of quantizer)
  }

  // NOTE: the lambda modifiers that are sometimes applied later might be best always applied in here.
  return dLambda;
}
#endif

void EncSlice::resetQP( Picture* pic, int sliceQP, double lambda )
{
  Slice* slice = pic->slices[0];

  // store lambda
  slice->setSliceQp( sliceQP );
  setUpLambda(slice, lambda, sliceQP);
}

#if ENABLE_QPA
static bool applyQPAdaptation (Picture* const pcPic,       Slice* const pcSlice,        const PreCalcValues& pcv,
                               const uint32_t startAddr,   const uint32_t boundingAddr, const bool useSharpLumaDQP,
                               const bool useFrameWiseQPA, const int previouslyAdaptedLumaQP = -1)
{
  const int  bitDepth    = pcSlice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA);
  const int  iQPIndex    = pcSlice->getSliceQp(); // initial QP index for current slice, used in following loops
  const BrickMap& tileMap = *pcPic->brickMap;
  bool   sliceQPModified = false;
  uint32_t   meanLuma    = MAX_UINT;
  double     hpEnerAvg   = 0.0;

#if GLOBAL_AVERAGING
  if (!useFrameWiseQPA || previouslyAdaptedLumaQP < 0)  // mean visual activity value and luma value in each CTU
#endif
  {
    for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr  = tileMap.getCtuBsToRsAddrMap (ctuTsAddr);
      const Position pos ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);
      const CompArea ctuArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
      const CompArea fltArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0, pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))), pcPic->Y());
      const CPelBuf  picOrig    = pcPic->getOrigBuf (fltArea);
      double hpEner = 0.0;

      filterAndCalculateAverageEnergies (picOrig.buf,    picOrig.stride, hpEner,
                                         picOrig.height, picOrig.width,  bitDepth);
      hpEnerAvg += hpEner;
      pcPic->m_uEnerHpCtu[ctuRsAddr] = hpEner;
      pcPic->m_iOffsetCtu[ctuRsAddr] = pcPic->getOrigBuf (ctuArea).computeAvg();
    }

    hpEnerAvg /= double (boundingAddr - startAddr);
  }
#if GLOBAL_AVERAGING
  const double hpEnerPic = 1.0 / getAveragePictureEnergy (pcPic->getOrigBuf().Y(), bitDepth);  // inverse, speed
#else
  const double hpEnerPic = 1.0 / hpEnerAvg; // speedup: multiply instead of divide in loop below; 1.0 for tuning
#endif

  if (useFrameWiseQPA || (iQPIndex >= MAX_QP))
  {
    int iQPFixed = (previouslyAdaptedLumaQP < 0) ? Clip3 (0, MAX_QP, iQPIndex + apprI3Log2 (hpEnerAvg * hpEnerPic)) : previouslyAdaptedLumaQP;

    if (isChromaEnabled (pcPic->chromaFormat) && (iQPIndex < MAX_QP) && (previouslyAdaptedLumaQP < 0))
    {
      iQPFixed += getGlaringColorQPOffset (pcPic, -1 /*ctuRsAddr*/, startAddr, boundingAddr, bitDepth, meanLuma);

      if (iQPFixed > MAX_QP
#if SHARP_LUMA_DELTA_QP
          && !useSharpLumaDQP
#endif
          ) iQPFixed = MAX_QP;
    }
#if SHARP_LUMA_DELTA_QP

    // change new fixed QP based on average CTU luma value (Sharp)
    if (useSharpLumaDQP && (iQPIndex < MAX_QP) && (previouslyAdaptedLumaQP < 0))
    {
      if (meanLuma == MAX_UINT) // collect picture mean luma value
      {
        meanLuma = 0;

        for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
        {
          const uint32_t ctuRsAddr = tileMap.getCtuBsToRsAddrMap (ctuTsAddr);

          meanLuma += pcPic->m_iOffsetCtu[ctuRsAddr];  // CTU mean
        }
        meanLuma = (meanLuma + ((boundingAddr - startAddr) >> 1)) / (boundingAddr - startAddr);
      }
      iQPFixed = Clip3 (0, MAX_QP, iQPFixed + lumaDQPOffset (meanLuma, bitDepth));
    }
#endif

    if (iQPIndex >= MAX_QP) iQPFixed = MAX_QP;
    else
    if (iQPFixed != iQPIndex)
    {
      const double* oldLambdas = pcSlice->getLambdas();
      const double  corrFactor = pow (2.0, double(iQPFixed - iQPIndex) / 3.0);
      const double  newLambdas[MAX_NUM_COMPONENT] = {oldLambdas[0] * corrFactor, oldLambdas[1] * corrFactor, oldLambdas[2] * corrFactor};

      CHECK (iQPIndex != pcSlice->getSliceQpBase(), "Invalid slice QP!");
      pcSlice->setLambdas (newLambdas);
      pcSlice->setSliceQp (iQPFixed); // update the slice/base QPs
      pcSlice->setSliceQpBase (iQPFixed);

      sliceQPModified = true;
    }

    for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr = tileMap.getCtuBsToRsAddrMap (ctuTsAddr);

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPFixed; // fixed QPs
    }
  }
  else // CTU-wise QPA
  {
    for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr = tileMap.getCtuBsToRsAddrMap (ctuTsAddr);

      int iQPAdapt = Clip3 (0, MAX_QP, iQPIndex + apprI3Log2 (pcPic->m_uEnerHpCtu[ctuRsAddr] * hpEnerPic));

      if (pcv.widthInCtus > 1) // try to enforce CTU SNR greater than zero dB
      {
        meanLuma = (uint32_t)pcPic->m_iOffsetCtu[ctuRsAddr];

        if (isChromaEnabled (pcPic->chromaFormat))
        {
          iQPAdapt += getGlaringColorQPOffset (pcPic, (int)ctuRsAddr, startAddr, boundingAddr, bitDepth, meanLuma);

          if (iQPAdapt > MAX_QP
#if SHARP_LUMA_DELTA_QP
              && !useSharpLumaDQP
#endif
              ) iQPAdapt = MAX_QP;
          CHECK (meanLuma != (uint32_t)pcPic->m_iOffsetCtu[ctuRsAddr], "luma DC offsets don't match");
        }
#if SHARP_LUMA_DELTA_QP

        // change adaptive QP based on mean CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
 #if ENABLE_QPA_SUB_CTU
          pcPic->m_uEnerHpCtu[ctuRsAddr] = (double)meanLuma; // for sub-CTU QPA
 #endif
          iQPAdapt = Clip3 (0, MAX_QP, iQPAdapt + lumaDQPOffset (meanLuma, bitDepth));
        }

#endif
        const uint32_t uRefScale  = g_invQuantScales[0][iQPAdapt % 6] << ((iQPAdapt / 6) + bitDepth - 4);
        const CompArea subArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
        const Pel*     pSrc       = pcPic->getOrigBuf (subArea).buf;
        const SizeType iSrcStride = pcPic->getOrigBuf (subArea).stride;
        const SizeType iSrcHeight = pcPic->getOrigBuf (subArea).height;
        const SizeType iSrcWidth  = pcPic->getOrigBuf (subArea).width;
        uint32_t uAbsDCless = 0;

        // compute sum of absolute DC-less (high-pass) luma values
        for (SizeType h = 0; h < iSrcHeight; h++)
        {
          for (SizeType w = 0; w < iSrcWidth; w++)
          {
            uAbsDCless += (uint32_t)abs (pSrc[w] - (Pel)meanLuma);
          }
          pSrc += iSrcStride;
        }

        if (iSrcHeight >= 64 || iSrcWidth >= 64)  // normalization
        {
          const uint64_t blockSize = uint64_t(iSrcWidth * iSrcHeight);

          uAbsDCless = uint32_t((uint64_t(uAbsDCless) * 64*64 + (blockSize >> 1)) / blockSize);
        }

        if (uAbsDCless < 64*64) uAbsDCless = 64*64;  // limit to 1

        // reduce QP index if CTU would be fully quantized to zero
        if (uAbsDCless < uRefScale)
        {
          const int limit  = std::min (0, ((iQPIndex + 4) >> 3) - 6);
          const int redVal = std::max (limit, apprI3Log2 ((double)uAbsDCless / (double)uRefScale));

          iQPAdapt = std::max (0, iQPAdapt + redVal);
        }
      }

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt; // adapted QPs

#if ENABLE_QPA_SUB_CTU
      if (pcv.widthInCtus > 1 && pcSlice->getPPS()->getCuQpDeltaSubdiv() == 0)  // reduce local DQP rate peaks
#elif ENABLE_QPA_SUB_CTU
      if (pcv.widthInCtus > 1 && pcSlice->getPPS()->getMaxCuDQPDepth() == 0)  // reduce local DQP rate peaks
#else
      if (pcv.widthInCtus > 1) // try to reduce local bitrate peaks via minimum smoothing of the adapted QPs
#endif
      {
        iQPAdapt = ctuRsAddr % pcv.widthInCtus; // horizontal offset
        if (iQPAdapt == 0)
        {
          iQPAdapt = (ctuRsAddr > 1) ? pcPic->m_iOffsetCtu[ctuRsAddr - 2] : 0;
        }
        else // iQPAdapt >= 1
        {
          iQPAdapt = (iQPAdapt > 1) ? std::min (pcPic->m_iOffsetCtu[ctuRsAddr - 2], pcPic->m_iOffsetCtu[ctuRsAddr]) : pcPic->m_iOffsetCtu[ctuRsAddr];
        }
        if (ctuRsAddr > pcv.widthInCtus)
        {
          iQPAdapt = std::min (iQPAdapt, (int)pcPic->m_iOffsetCtu[ctuRsAddr - 1 - pcv.widthInCtus]);
        }
        if ((ctuRsAddr > 0) && (pcPic->m_iOffsetCtu[ctuRsAddr - 1] < (Pel)iQPAdapt))
        {
          pcPic->m_iOffsetCtu[ctuRsAddr - 1] = (Pel)iQPAdapt;
        }
        if ((ctuTsAddr == boundingAddr - 1) && (ctuRsAddr > pcv.widthInCtus)) // last CTU in the given slice
        {
          iQPAdapt = std::min (pcPic->m_iOffsetCtu[ctuRsAddr - 1], pcPic->m_iOffsetCtu[ctuRsAddr - pcv.widthInCtus]);
          if (pcPic->m_iOffsetCtu[ctuRsAddr] < (Pel)iQPAdapt)
          {
            pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt;
          }
        }
      }
    } // end iteration over all CTUs in current slice
  }

  return sliceQPModified;
}

#if ENABLE_QPA_SUB_CTU
static int applyQPAdaptationSubCtu (CodingStructure &cs, const UnitArea ctuArea, const uint32_t ctuAddr, const bool useSharpLumaDQP)
{
  const PreCalcValues &pcv = *cs.pcv;
  const Picture     *pcPic = cs.picture;
  const int       bitDepth = cs.slice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA); // overall image bit-depth
  const int   adaptedCtuQP = pcPic ? pcPic->m_iOffsetCtu[ctuAddr] : cs.slice->getSliceQpBase();

  if (!pcPic || cs.pps->getCuQpDeltaSubdiv() == 0) return adaptedCtuQP;

  for (unsigned addr = 0; addr < cs.picture->m_subCtuQP.size(); addr++)
  {
    cs.picture->m_subCtuQP[addr] = (int8_t)adaptedCtuQP;
  }
  if (cs.slice->getSliceQp() < MAX_QP && pcv.widthInCtus > 1)
  {
#if SHARP_LUMA_DELTA_QP
    const int   lumaCtuDQP = useSharpLumaDQP ? lumaDQPOffset ((uint32_t)pcPic->m_uEnerHpCtu[ctuAddr], bitDepth) : 0;
#endif
#if MAX_TB_SIZE_SIGNALLING
    const unsigned     mts = std::min (cs.sps->getMaxTbSize(), pcv.maxCUWidth);
#else
    const unsigned     mts = std::min<uint32_t> (MAX_TB_SIZEY, pcv.maxCUWidth);
#endif
    const unsigned mtsLog2 = (unsigned)floorLog2(mts);
    const unsigned  stride = pcv.maxCUWidth >> mtsLog2;
    unsigned numAct = 0;    // number of block activities
    double   sumAct = 0.0; // sum of all block activities
    double   subAct[16];   // individual block activities
#if SHARP_LUMA_DELTA_QP
    uint32_t subMLV[16];   // individual mean luma values
#endif

    CHECK (mts * 4 < pcv.maxCUWidth || mts * 4 < pcv.maxCUHeight, "max. transform size is too small for given CTU size");

    for (unsigned h = 0; h < (pcv.maxCUHeight >> mtsLog2); h++)
    {
      for (unsigned w = 0; w < stride; w++)
      {
        const unsigned addr    = w + h * stride;
        const PosType  x       = ctuArea.lx() + w * mts;
        const PosType  y       = ctuArea.ly() + h * mts;
        const CompArea fltArea = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (x > 0 ? x - 1 : 0, y > 0 ? y - 1 : 0, mts + (x > 0 ? 2 : 1), mts + (y > 0 ? 2 : 1))), pcPic->Y());
        const CPelBuf  picOrig = pcPic->getOrigBuf (fltArea);

        if (x >= pcPic->lwidth() || y >= pcPic->lheight())
        {
          continue;
        }
        filterAndCalculateAverageEnergies (picOrig.buf,    picOrig.stride, subAct[addr],
                                           picOrig.height, picOrig.width,  bitDepth);
        numAct++;
        sumAct += subAct[addr];
#if SHARP_LUMA_DELTA_QP

        if (useSharpLumaDQP)
        {
          const CompArea subArea = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (x, y, mts, mts)), pcPic->Y());

          subMLV[addr] = pcPic->getOrigBuf (subArea).computeAvg();
        }
#endif
      }
    }
    if (sumAct <= 0.0) return adaptedCtuQP;

    sumAct = double(numAct) / sumAct; // 1.0 / (average CTU activity)

    for (unsigned h = 0; h < (pcv.maxCUHeight >> mtsLog2); h++)
    {
      for (unsigned w = 0; w < stride; w++)
      {
        const unsigned addr = w + h * stride;

        if (ctuArea.lx() + w * mts >= pcPic->lwidth() || ctuArea.ly() + h * mts >= pcPic->lheight())
        {
          continue;
        }
        cs.picture->m_subCtuQP[addr] = (int8_t)Clip3 (0, MAX_QP, adaptedCtuQP + apprI3Log2 (subAct[addr] * sumAct));
#if SHARP_LUMA_DELTA_QP

        // change adapted QP based on mean sub-CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
          cs.picture->m_subCtuQP[addr] = (int8_t)Clip3 (0, MAX_QP, (int)cs.picture->m_subCtuQP[addr] - lumaCtuDQP + lumaDQPOffset (subMLV[addr], bitDepth));
        }
#endif
      }
    }
  }

  return adaptedCtuQP;
}
#endif // ENABLE_QPA_SUB_CTU
#endif // ENABLE_QPA

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

//! set adaptive search range based on poc difference
void EncSlice::setSearchRange( Slice* pcSlice )
{
  int iCurrPOC = pcSlice->getPOC();
  int iRefPOC;
  int iGOPSize = m_pcCfg->getGOPSize();
  int iOffset = (iGOPSize >> 1);
  int iMaxSR = m_pcCfg->getSearchRange();
  int iNumPredDir = pcSlice->isInterP() ? 1 : 2;

  for (int iDir = 0; iDir < iNumPredDir; iDir++)
  {
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
    {
      iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
      int newSearchRange = Clip3(m_pcCfg->getMinSearchWindow(), iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_pcInterSearch->setAdaptiveSearchRange(iDir, iRefIdx, newSearchRange);
#if ENABLE_WPP_PARALLELISM
      for( int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++ )
      {
        m_pcLib->getInterSearch( jId )->setAdaptiveSearchRange( iDir, iRefIdx, newSearchRange );
      }
#endif
    }
  }
}

/**
 Multi-loop slice encoding for different slice QP

 \param pcPic    picture class
 */
void EncSlice::precompressSlice( Picture* pcPic )
{
  // if deltaQP RD is not used, simply return
  if ( m_pcCfg->getDeltaQpRD() == 0 )
  {
    return;
  }

  if ( m_pcCfg->getUseRateCtrl() )
  {
    THROW("\nMultiple QP optimization is not allowed when rate control is enabled." );
  }

  Slice* pcSlice        = pcPic->slices[getSliceSegmentIdx()];


  if (pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES)
  {
    // TODO: investigate use of average cost per CTU so that this Slice Mode can be used.
    THROW( "Unable to optimise Slice-level QP if Slice Mode is set to FIXED_NUMBER_OF_BYTES\n" );
  }

  double     dPicRdCostBest = MAX_DOUBLE;
  uint32_t       uiQpIdxBest = 0;

  double dFrameLambda;
  int SHIFT_QP = 12
                 + 6
                     * (pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                        - DISTORTION_PRECISION_ADJUSTMENT(pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));

  // set frame lambda
  if (m_pcCfg->getGOPSize() > 1)
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0] - SHIFT_QP) / 3.0);
  }

  // for each QP candidate
  for ( uint32_t uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
  {
    pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdx] );
    setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdx], m_viRdPicQp    [uiQpIdx]);

    // try compress
    compressSlice   ( pcPic, true, m_pcCfg->getFastDeltaQp());

    uint64_t uiPicDist        = m_uiPicDist; // Distortion, as calculated by compressSlice.
    // NOTE: This distortion is the chroma-weighted SSE distortion for the slice.
    //       Previously a standard SSE distortion was calculated (for the entire frame).
    //       Which is correct?
#if W0038_DB_OPT
    // TODO: Update loop filter, SAO and distortion calculation to work on one slice only.
    // uiPicDist = m_pcGOPEncoder->preLoopFilterPicAndCalcDist( pcPic );
#endif
    // compute RD cost and choose the best
    double dPicRdCost = double( uiPicDist ) + dFrameLambda * double( m_uiPicTotalBits );

    if ( dPicRdCost < dPicRdCostBest )
    {
      uiQpIdxBest    = uiQpIdx;
      dPicRdCostBest = dPicRdCost;
    }
  }

  // set best values
  pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdxBest] );
  setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdxBest], m_viRdPicQp    [uiQpIdxBest]);
}

void EncSlice::calCostSliceI(Picture* pcPic) // TODO: this only analyses the first slice segment. What about the others?
{
  double         iSumHadSlice      = 0;
  Slice * const  pcSlice           = pcPic->slices[getSliceSegmentIdx()];
  const BrickMap &tileMap          = *pcPic->brickMap;
  const PreCalcValues& pcv         = *pcPic->cs->pcv;
  const SPS     &sps               = *(pcSlice->getSPS());
  const int      shift             = sps.getBitDepth(CHANNEL_TYPE_LUMA)-8;
  const int      offset            = (shift>0)?(1<<(shift-1)):0;


  uint32_t startCtuTsAddr, boundingCtuTsAddr;
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );
  for( uint32_t ctuTsAddr = startCtuTsAddr, ctuRsAddr = tileMap.getCtuBsToRsAddrMap( startCtuTsAddr);
       ctuTsAddr < boundingCtuTsAddr;
       ctuRsAddr = tileMap.getCtuBsToRsAddrMap(++ctuTsAddr) )
  {
    Position pos( (ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);

    const int height  = std::min( pcv.maxCUHeight, pcv.lumaHeight - pos.y );
    const int width   = std::min( pcv.maxCUWidth,  pcv.lumaWidth  - pos.x );
    const CompArea blk( COMPONENT_Y, pcv.chrFormat, pos, Size( width, height));
    int iSumHad = m_pcCuEncoder->updateCtuDataISlice( pcPic->getOrigBuf( blk ) );

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra=(iSumHad+offset)>>shift;
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;

  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

/** \param pcPic   picture class
 */
void EncSlice::compressSlice( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP )
{
  // if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
  //   effectively disabling the slice-segment-mode.

  Slice* const pcSlice    = pcPic->slices[getSliceSegmentIdx()];
  uint32_t  startCtuTsAddr;
  uint32_t  boundingCtuTsAddr;

  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );
  if (bCompressEntireSlice)
  {
    boundingCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr();
  }

  // initialize cost values - these are used by precompressSlice (they should be parameters).
  m_uiPicTotalBits  = 0;
  m_uiPicDist       = 0;

  pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

  m_CABACEstimator->initCtxModels( *pcSlice );

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for( int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++ )
  {
    CABACWriter* cw = m_pcLib->getCABACEncoder( jId )->getCABACEstimator( pcSlice->getSPS() );
    cw->initCtxModels( *pcSlice );
  }

#endif
  m_pcCuEncoder->getModeCtrl()->setFastDeltaQp(bFastDeltaQP);


  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
  {
    xCalcACDCParamSlice(pcSlice);
  }

  const bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

  if ( bWp_explicit )
  {
    //------------------------------------------------------------------------------
    //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
    //------------------------------------------------------------------------------
    if(pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES)
    {
      EXIT("Weighted Prediction is not yet supported with slice mode determined by max number of bins.");
    }

    xEstimateWPParamSlice( pcSlice, m_pcCfg->getWeightedPredictionMethod() );
    pcSlice->initWpScaling(pcSlice->getSPS());

    // check WP on/off
    xCheckWPEnable( pcSlice );
  }



    pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();

  CHECK( pcPic->m_prevQP[0] == std::numeric_limits<int>::max(), "Invalid previous QP" );

  CodingStructure&  cs          = *pcPic->cs;
  cs.slice    = pcSlice;
  cs.pcv      = pcSlice->getPPS()->pcv;
  cs.fracBits = 0;

  if( startCtuTsAddr == 0 && ( pcSlice->getPOC() != m_pcCfg->getSwitchPOC() || -1 == m_pcCfg->getDebugCTU() ) )
  {
    cs.initStructData (pcSlice->getSliceQp(), pcSlice->getPPS()->getTransquantBypassEnabledFlag());
  }

#if ENABLE_QPA
  if (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && (boundingCtuTsAddr > startCtuTsAddr))
  {
    if (applyQPAdaptation (pcPic, pcSlice, *cs.pcv, startCtuTsAddr, boundingCtuTsAddr, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES,
                           (m_pcCfg->getBaseQP() >= 38) || (m_pcCfg->getSourceWidth() <= 512 && m_pcCfg->getSourceHeight() <= 320), m_adaptedLumaQP))
    {
      m_CABACEstimator->initCtxModels (*pcSlice);
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
      for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
      {
        CABACWriter* cw = m_pcLib->getCABACEncoder (jId)->getCABACEstimator (pcSlice->getSPS());
        cw->initCtxModels (*pcSlice);
      }
#endif
        pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
      if (startCtuTsAddr == 0)
      {
        cs.currQP[0] = cs.currQP[1] = pcSlice->getSliceQp(); // cf code above
      }
    }
  }
#endif // ENABLE_QPA

#if JVET_O0119_BASE_PALETTE_444
  bool checkPLTRatio = m_pcCfg->getIntraPeriod() != 1 && pcSlice->isIRAP();
  if (checkPLTRatio)
  {
    m_pcCuEncoder->getModeCtrl()->setPltEnc(true);
  }
  else
  {
    bool doPlt = m_pcLib->getPltEnc();
    m_pcCuEncoder->getModeCtrl()->setPltEnc(doPlt);
  }
#endif

#if ENABLE_WPP_PARALLELISM
  bool bUseThreads = m_pcCfg->getNumWppThreads() > 1;
  if( bUseThreads )
  {
    CHECK( startCtuTsAddr != 0 || boundingCtuTsAddr != pcPic->cs->pcv->sizeInCtus, "not intended" );

    pcPic->cs->allocateVectorsAtPicLevel();

    omp_set_num_threads( m_pcCfg->getNumWppThreads() + m_pcCfg->getNumWppExtraLines() );

    #pragma omp parallel for schedule(static,1) if(bUseThreads)
    for( int ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr += widthInCtus )
    {
      // wpp thread start
      pcPic->scheduler.setWppThreadId();
#if ENABLE_SPLIT_PARALLELISM
      pcPic->scheduler.setSplitThreadId( 0 );
#endif
      encodeCtus( pcPic, bCompressEntireSlice, bFastDeltaQP, ctuTsAddr, ctuTsAddr + widthInCtus, m_pcLib );
      // wpp thread stop
    }
  }
  else
#endif
#if K0149_BLOCK_STATISTICS
  const SPS *sps = pcSlice->getSPS();
  CHECK(sps == 0, "No SPS present");
  writeBlockStatisticsHeader(sps);
#endif
  m_pcInterSearch->resetAffineMVList();
#if JVET_O0592_ENC_ME_IMP
  m_pcInterSearch->resetUniMvList();
#endif
  encodeCtus( pcPic, bCompressEntireSlice, bFastDeltaQP, startCtuTsAddr, boundingCtuTsAddr, m_pcLib );
#if JVET_O0119_BASE_PALETTE_444
  if (checkPLTRatio) m_pcLib->checkPltStats( pcPic );
#endif
}

void EncSlice::checkDisFracMmvd( Picture* pcPic, uint32_t startCtuTsAddr, uint32_t boundingCtuTsAddr )
{
  CodingStructure&  cs            = *pcPic->cs;
  Slice* pcSlice                  = cs.slice;
  const PreCalcValues& pcv        = *cs.pcv;
  const uint32_t    widthInCtus   = pcv.widthInCtus;
  const BrickMap&  tileMap         = *pcPic->brickMap;
  const uint32_t hashThreshold    = 20;
  uint32_t totalCtu               = 0;
  uint32_t hashRatio              = 0;

  if ( !pcSlice->getSPS()->getFpelMmvdEnabledFlag() )
  {
    return;
  }

  for ( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
    const uint32_t ctuRsAddr = tileMap.getCtuBsToRsAddrMap( ctuTsAddr );
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos ( ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight );
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight ) );

    hashRatio += m_pcCuEncoder->getIbcHashMap().getHashHitRatio( ctuArea.Y() );
    totalCtu++;
  }

  if ( hashRatio > totalCtu * hashThreshold )
  {
    pcSlice->setDisFracMMVD( true );
  }
  if (!pcSlice->getDisFracMMVD()) {
    bool useIntegerMVD = (pcPic->lwidth()*pcPic->lheight() > 1920 * 1080);
    pcSlice->setDisFracMMVD( useIntegerMVD );
  }
}


#if JVET_O0105_ICT
void setJointCbCrModes( CodingStructure& cs, const Position topLeftLuma, const Size sizeLuma )
{
  bool              sgnFlag = true;

  if( isChromaEnabled( cs.picture->chromaFormat) )
  {
    const CompArea  cbArea  = CompArea( COMPONENT_Cb, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CompArea  crArea  = CompArea( COMPONENT_Cr, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CPelBuf   orgCb   = cs.picture->getOrigBuf( cbArea );
    const CPelBuf   orgCr   = cs.picture->getOrigBuf( crArea );
    const int       x0      = ( cbArea.x > 0 ? 0 : 1 );
    const int       y0      = ( cbArea.y > 0 ? 0 : 1 );
    const int       x1      = ( cbArea.x + cbArea.width  < cs.picture->Cb().width  ? cbArea.width  : cbArea.width  - 1 );
    const int       y1      = ( cbArea.y + cbArea.height < cs.picture->Cb().height ? cbArea.height : cbArea.height - 1 );
    const int       cbs     = orgCb.stride;
    const int       crs     = orgCr.stride;
    const Pel*      pCb     = orgCb.buf + y0 * cbs;
    const Pel*      pCr     = orgCr.buf + y0 * crs;
    int64_t         sumCbCr = 0;

    // determine inter-chroma transform sign from correlation between high-pass filtered (i.e., zero-mean) Cb and Cr planes
    for( int y = y0; y < y1; y++, pCb += cbs, pCr += crs )
    {
      for( int x = x0; x < x1; x++ )
      {
        int cb = ( 12*(int)pCb[x] - 2*((int)pCb[x-1] + (int)pCb[x+1] + (int)pCb[x-cbs] + (int)pCb[x+cbs]) - ((int)pCb[x-1-cbs] + (int)pCb[x+1-cbs] + (int)pCb[x-1+cbs] + (int)pCb[x+1+cbs]) );
        int cr = ( 12*(int)pCr[x] - 2*((int)pCr[x-1] + (int)pCr[x+1] + (int)pCr[x-crs] + (int)pCr[x+crs]) - ((int)pCr[x-1-crs] + (int)pCr[x+1-crs] + (int)pCr[x-1+crs] + (int)pCr[x+1+crs]) );
        sumCbCr += cb*cr;
      }
    }

    sgnFlag = ( sumCbCr < 0 );
  }

  cs.slice->setJointCbCrSignFlag( sgnFlag );
}
#endif


void EncSlice::encodeCtus( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP, uint32_t startCtuTsAddr, uint32_t boundingCtuTsAddr, EncLib* pEncLib )
{
  CodingStructure&  cs            = *pcPic->cs;
  Slice* pcSlice                  = cs.slice;
  const PreCalcValues& pcv        = *cs.pcv;
  const uint32_t        widthInCtus   = pcv.widthInCtus;
  const BrickMap&  tileMap        = *pcPic->brickMap;
#if ENABLE_QPA
  const int iQPIndex              = pcSlice->getSliceQpBase();
#endif

#if ENABLE_WPP_PARALLELISM
  const int       dataId          = pcPic->scheduler.getWppDataId();
#elif ENABLE_SPLIT_PARALLELISM
  const int       dataId          = 0;
#endif
  CABACWriter*    pCABACWriter    = pEncLib->getCABACEncoder( PARL_PARAM0( dataId ) )->getCABACEstimator( pcSlice->getSPS() );
  TrQuant*        pTrQuant        = pEncLib->getTrQuant( PARL_PARAM0( dataId ) );
  RdCost*         pRdCost         = pEncLib->getRdCost( PARL_PARAM0( dataId ) );
  EncCfg*         pCfg            = pEncLib;
  RateCtrl*       pRateCtrl       = pEncLib->getRateCtrl();
#if ENABLE_WPP_PARALLELISM
  // first version dont use ctx from above
  pCABACWriter->initCtxModels( *pcSlice );
#endif
#if RDOQ_CHROMA_LAMBDA
  pTrQuant    ->setLambdas( pcSlice->getLambdas() );
#else
  pTrQuant    ->setLambda ( pcSlice->getLambdas()[0] );
#endif
  pRdCost     ->setLambda ( pcSlice->getLambdas()[0], pcSlice->getSPS()->getBitDepths() );

  int prevQP[2];
  int currQP[2];
  prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
  currQP[0] = currQP[1] = pcSlice->getSliceQp();

    prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
  if ( pcSlice->getSPS()->getFpelMmvdEnabledFlag() ||
      (pcSlice->getSPS()->getIBCFlag() && m_pcCuEncoder->getEncCfg()->getIBCHashSearch()))
  {
    m_pcCuEncoder->getIbcHashMap().rebuildPicHashMap(cs.picture->getTrueOrigBuf());
    if (m_pcCfg->getIntraPeriod() != -1)
    {
      int hashBlkHitPerc = m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y());
      cs.slice->setDisableSATDForRD(hashBlkHitPerc > 59);
    }
  }
  checkDisFracMmvd( pcPic, startCtuTsAddr, boundingCtuTsAddr );

#if JVET_O0105_ICT
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  if (pcSlice->getSPS()->getJointCbCrEnabledFlag())
  {
    setJointCbCrModes(cs, Position(0, 0), cs.area.lumaSize());
  }
#else
  setJointCbCrModes(cs, Position(0, 0), cs.area.lumaSize());
#endif
#endif

  // for every CTU in the slice segment (may terminate sooner if there is a byte limit on the slice-segment)
  uint32_t startSliceRsRow = tileMap.getCtuBsToRsAddrMap(startCtuTsAddr) / widthInCtus;
  uint32_t startSliceRsCol = tileMap.getCtuBsToRsAddrMap(startCtuTsAddr) % widthInCtus;
  uint32_t endSliceRsRow = tileMap.getCtuBsToRsAddrMap(boundingCtuTsAddr - 1) / widthInCtus;
  uint32_t endSliceRsCol = tileMap.getCtuBsToRsAddrMap(boundingCtuTsAddr - 1) % widthInCtus;
  for( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
    const int32_t ctuRsAddr = tileMap.getCtuBsToRsAddrMap( ctuTsAddr );
    if (pcSlice->getPPS()->getRectSliceFlag() &&
      ((ctuRsAddr / widthInCtus) < startSliceRsRow || (ctuRsAddr / widthInCtus) > endSliceRsRow ||
      (ctuRsAddr % widthInCtus) < startSliceRsCol || (ctuRsAddr % widthInCtus) > endSliceRsCol))
      continue;

    // update CABAC state
    const uint32_t firstCtuRsAddrOfTile = tileMap.bricks[tileMap.getBrickIdxRsMap(ctuRsAddr)].getFirstCtuRsAddr();
    const uint32_t tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight ) );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    if( pCfg->getSwitchPOC() != pcPic->poc || -1 == pCfg->getDebugCTU() )
    if ((cs.slice->getSliceType() != I_SLICE || cs.sps->getIBCFlag()) && ctuXPosInCtus == tileXPosInCtus)
    {
      cs.motionLut.lut.resize(0);
      cs.motionLut.lutIbc.resize(0);
#if !JVET_O0078_SINGLE_HMVPLUT
      cs.motionLut.lutShareIbc.resize(0);
#endif
    }

#if ENABLE_WPP_PARALLELISM
    pcPic->scheduler.wait( ctuXPosInCtus, ctuYPosInCtus );
#endif

    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      pCABACWriter->initCtxModels( *pcSlice );
#if JVET_O0119_BASE_PALETTE_444
      cs.resetPrevPLT(cs.prevPLT);
#endif
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }
    else if (ctuXPosInCtus == tileXPosInCtus && pEncLib->getEntropyCodingSyncEnabledFlag())
    {
      // reset and then update contexts to the state at the end of the top-right CTU (if within current slice and tile).
      pCABACWriter->initCtxModels( *pcSlice );
#if JVET_O0119_BASE_PALETTE_444
      cs.resetPrevPLT(cs.prevPLT);
#endif
      if( cs.getCURestricted( pos.offset(0, -1), pos, pcSlice->getIndependentSliceIdx(), tileMap.getBrickIdxRsMap( pos ), CH_L ) )
      {
        // Top-right is available, we use it.
        pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextState;
      }
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }

#if ENABLE_WPP_PARALLELISM
    if( ctuXPosInCtus == 0 && ctuYPosInCtus > 0 && widthInCtus > 1 && ( pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual() ) )
    {
      pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus-1];  // last line
    }
#else
#endif

#if RDOQ_CHROMA_LAMBDA && ENABLE_QPA && !ENABLE_QPA_SUB_CTU
    double oldLambdaArray[MAX_NUM_COMPONENT] = {0.0};
#endif
    const double oldLambda = pRdCost->getLambda();
    if ( pCfg->getUseRateCtrl() )
    {
      int estQP        = pcSlice->getSliceQp();
      double estLambda = -1.0;
      double bpp       = -1.0;

      if( ( pcPic->slices[0]->isIRAP() && pCfg->getForceIntraQP() ) || !pCfg->getLCULevelRC() )
      {
        estQP = pcSlice->getSliceQp();
      }
      else
      {
        bpp = pRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->isIRAP());
        if ( pcPic->slices[0]->isIRAP())
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
        }
        else
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambda( bpp );
          estQP     = pRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
        }

        estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );

#if WCG_EXT
        pRdCost->saveUnadjustedLambda();
#endif
        pRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());

#if RDOQ_CHROMA_LAMBDA
        // set lambda for RDOQ
        const double chromaLambda = estLambda / pRdCost->getChromaWeight();
        const double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
        pTrQuant->setLambdas( lambdaArray );
#else
        pTrQuant->setLambda( estLambda );
#endif
      }

      pRateCtrl->setRCQP( estQP );
    }
#if ENABLE_QPA
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
#if ENABLE_QPA_SUB_CTU
      const int adaptedQP    = applyQPAdaptationSubCtu (cs, ctuArea, ctuRsAddr, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES);
#else
      const int adaptedQP    = pcPic->m_iOffsetCtu[ctuRsAddr];
#endif
      const double newLambda = pcSlice->getLambdas()[0] * pow (2.0, double (adaptedQP - iQPIndex) / 3.0);
      pcPic->m_uEnerHpCtu[ctuRsAddr] = newLambda; // for ALF and SAO
#if !ENABLE_QPA_SUB_CTU
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->getLambdas (oldLambdaArray); // save the old lambdas
      const double chromaLambda = newLambda / pRdCost->getChromaWeight();
      const double lambdaArray[MAX_NUM_COMPONENT] = {newLambda, chromaLambda, chromaLambda};
      pTrQuant->setLambdas (lambdaArray);
#else
      pTrQuant->setLambda (newLambda);
#endif
      pRdCost->setLambda (newLambda, pcSlice->getSPS()->getBitDepths());
#endif
      currQP[0] = currQP[1] = adaptedQP;
    }
#endif

    bool updateGbiCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuTsAddr == startCtuTsAddr;
    if( updateGbiCodingOrder )
    {
      resetGbiCodingOrder(false, cs);
      m_pcInterSearch->initWeightIdxBits();
    }
    if (pcSlice->getSPS()->getUseReshaper())
    {
      m_pcCuEncoder->setDecCuReshaperInEncCU(m_pcLib->getReshaper(), pcSlice->getSPS()->getChromaFormatIdc());

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
      for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
      {
        m_pcLib->getCuEncoder(jId)->setDecCuReshaperInEncCU(m_pcLib->getReshaper(jId), pcSlice->getSPS()->getChromaFormatIdc());
      }
#endif
    }
    if( !cs.slice->isIntra() && pCfg->getMCTSEncConstraint() )
    {
      pcPic->mctsInfo.init( &cs, ctuRsAddr );
    }

  if (pCfg->getSwitchPOC() != pcPic->poc || ctuRsAddr >= pCfg->getDebugCTU())
#if ENABLE_WPP_PARALLELISM
    pEncLib->getCuEncoder( dataId )->compressCtu( cs, ctuArea, ctuRsAddr, prevQP, currQP );
#else
    m_pcCuEncoder->compressCtu( cs, ctuArea, ctuRsAddr, prevQP, currQP );
#endif

#if K0149_BLOCK_STATISTICS
    getAndStoreBlockStatistics(cs, ctuArea);
#endif

    pCABACWriter->resetBits();
    pCABACWriter->coding_tree_unit( cs, ctuArea, prevQP, ctuRsAddr, true, true );
    const int numberOfWrittenBits = int( pCABACWriter->getEstFracBits() >> SCALE_BITS );

    // Calculate if this CTU puts us over slice bit size.
    // cannot terminate if current slice/slice-segment would be 0 Ctu in size,
    const uint32_t validEndOfSliceCtuTsAddr = ctuTsAddr + (ctuTsAddr == startCtuTsAddr ? 1 : 0);
    // Set slice end parameter
    if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceBits()+numberOfWrittenBits > (pcSlice->getSliceArgument()<<3))
    {
      pcSlice->setSliceCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;
    }
    if (boundingCtuTsAddr <= ctuTsAddr)
    {
      break;
    }

#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
    pcSlice->setSliceBits( ( uint32_t ) ( pcSlice->getSliceBits() + numberOfWrittenBits ) );
#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif

    // Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
    if( ctuXPosInCtus == tileXPosInCtus && pEncLib->getEntropyCodingSyncEnabledFlag() )
    {
      pEncLib->m_entropyCodingSyncContextState = pCABACWriter->getCtx();
    }
#if ENABLE_WPP_PARALLELISM
    if( ctuXPosInCtus == 1 && ( pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual() ) )
    {
      pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus] = pCABACWriter->getCtx();
    }
#endif

#if !ENABLE_WPP_PARALLELISM
    int actualBits = int(cs.fracBits >> SCALE_BITS);
    actualBits    -= (int)m_uiPicTotalBits;
#endif
    if ( pCfg->getUseRateCtrl() )
    {
#if ENABLE_WPP_PARALLELISM
      int actualBits      = int( cs.fracBits >> SCALE_BITS );
      actualBits         -= (int)m_uiPicTotalBits;
#endif
      int actualQP        = g_RCInvalidQPValue;
      double actualLambda = pRdCost->getLambda();
      int numberOfEffectivePixels    = 0;

      int numberOfSkipPixel = 0;
      for (auto &cu : cs.traverseCUs(ctuArea, CH_L))
      {
        numberOfSkipPixel += cu.skip*cu.lumaSize().area();
      }

      for( auto &cu : cs.traverseCUs( ctuArea, CH_L ) )
      {
        if( !cu.skip || cu.rootCbf )
        {
          numberOfEffectivePixels += cu.lumaSize().area();
          break;
        }
      }
      double skipRatio = (double)numberOfSkipPixel / ctuArea.lumaSize().area();
      CodingUnit* cu = cs.getCU( ctuArea.lumaPos(), CH_L );

      if ( numberOfEffectivePixels == 0 )
      {
        actualQP = g_RCInvalidQPValue;
      }
      else
      {
        actualQP = cu->qp;
      }
      pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
      pRateCtrl->getRCPic()->updateAfterCTU(pRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda, skipRatio,
        pcSlice->isIRAP() ? 0 : pCfg->getLCULevelRC());
    }
#if ENABLE_QPA && !ENABLE_QPA_SUB_CTU
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->setLambdas (oldLambdaArray);
#else
      pTrQuant->setLambda (oldLambda);
#endif
      pRdCost->setLambda (oldLambda, pcSlice->getSPS()->getBitDepths());
    }
#endif

#if !ENABLE_WPP_PARALLELISM
    m_uiPicTotalBits += actualBits;
    m_uiPicDist       = cs.dist;
#endif
#if ENABLE_WPP_PARALLELISM
    pcPic->scheduler.setReady( ctuXPosInCtus, ctuYPosInCtus );
#endif
  }

  // this is wpp exclusive section

//  m_uiPicTotalBits += actualBits;
//  m_uiPicDist       = cs.dist;

}

void EncSlice::encodeSlice   ( Picture* pcPic, OutputBitstream* pcSubstreams, uint32_t &numBinsCoded )
{

  Slice *const pcSlice               = pcPic->slices[getSliceSegmentIdx()];
  const BrickMap& tileMap            = *pcPic->brickMap;
  const uint32_t startCtuTsAddr          = pcSlice->getSliceCurStartCtuTsAddr();
  const uint32_t boundingCtuTsAddr       = pcSlice->getSliceCurEndCtuTsAddr();
  const bool wavefrontsEnabled       = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();


  // setup coding structure
  CodingStructure& cs = *pcPic->cs;
  cs.slice            = pcSlice;
  // initialise entropy coder for the slice
  m_CABACWriter->initCtxModels( *pcSlice );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", pcSlice->getPOC() );

    pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();

  const PreCalcValues& pcv = *cs.pcv;
  const uint32_t widthInCtus   = pcv.widthInCtus;
  uint32_t startSliceRsRow = tileMap.getCtuBsToRsAddrMap(startCtuTsAddr) / widthInCtus;
  uint32_t startSliceRsCol = tileMap.getCtuBsToRsAddrMap(startCtuTsAddr) % widthInCtus;
  uint32_t endSliceRsRow = tileMap.getCtuBsToRsAddrMap(boundingCtuTsAddr - 1) / widthInCtus;
  uint32_t endSliceRsCol = tileMap.getCtuBsToRsAddrMap(boundingCtuTsAddr - 1) % widthInCtus;
  uint32_t uiSubStrm = 0;

  // for every CTU in the slice segment...

  for( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
    const uint32_t ctuRsAddr            = tileMap.getCtuBsToRsAddrMap(ctuTsAddr);
    const Brick& currentTile            = tileMap.bricks[tileMap.getBrickIdxRsMap(ctuRsAddr)];
    if (pcSlice->getPPS()->getRectSliceFlag() &&
      ((ctuRsAddr / widthInCtus) < startSliceRsRow || (ctuRsAddr / widthInCtus) > endSliceRsRow ||
      (ctuRsAddr % widthInCtus) < startSliceRsCol || (ctuRsAddr % widthInCtus) > endSliceRsCol))
      continue;
    const uint32_t firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();
    const uint32_t tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
    m_CABACWriter->initBitstream( &pcSubstreams[uiSubStrm] );

    // set up CABAC contexts' state for this CTU
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice );
#if JVET_O0119_BASE_PALETTE_444
        cs.resetPrevPLT(cs.prevPLT);
#endif
      }
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice );
#if JVET_O0119_BASE_PALETTE_444
        cs.resetPrevPLT(cs.prevPLT);
#endif
      }
      if( cs.getCURestricted( pos.offset( 0, -1 ), pos, pcSlice->getIndependentSliceIdx(), tileMap.getBrickIdxRsMap( pos ), CH_L ) )
      {
        // Top-right is available, so use it.
        m_CABACWriter->getCtx() = m_entropyCodingSyncContextState;
      }
    }

    bool updateGbiCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuTsAddr == startCtuTsAddr;
    if( updateGbiCodingOrder )
    {
      resetGbiCodingOrder(false, cs);
    }

    m_CABACWriter->coding_tree_unit( cs, ctuArea, pcPic->m_prevQP, ctuRsAddr );

    // store probabilities of second CTU in line into buffer
    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = m_CABACWriter->getCtx();
    }

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
    bool isLastCTUinBrick = tileMap.getBrickIdxBsMap(ctuTsAddr) != tileMap.getBrickIdxBsMap(ctuTsAddr + 1);
    bool isLastCTUinWPP = wavefrontsEnabled && ((ctuRsAddr + 1 % widthInCtus) == tileXPosInCtus);
    bool isMoreCTUsinSlice = ctuRsAddr != tileMap.getCtuBsToRsAddrMap(boundingCtuTsAddr - 1);
    if (isLastCTUinBrick || isLastCTUinWPP || !isMoreCTUsinSlice)         // this the the last CTU of either tile/brick/WPP/slice
    {
      m_CABACWriter->end_of_slice();  //This is actually end_of_brick_one_bit or end_of_subset_one_bit

      // Byte-alignment in slice_data() when new tile
      pcSubstreams[uiSubStrm].writeByteAlignment();

      if (isMoreCTUsinSlice) //Byte alignment only when it is not the last substream in the slice
      {
        // write sub-stream size
        pcSlice->addSubstreamSize((pcSubstreams[uiSubStrm].getNumberOfWrittenBits() >> 3) + pcSubstreams[uiSubStrm].countStartCodeEmulations());
      }
      uiSubStrm++;
    }
  } // CTU-loop


  if(pcSlice->getPPS()->getCabacInitPresentFlag())
  {
    m_encCABACTableIdx = m_CABACWriter->getCtxInitId( *pcSlice );
  }
  else
  {
    m_encCABACTableIdx = pcSlice->getSliceType();
  }
  numBinsCoded = m_CABACWriter->getNumBins();

}

void EncSlice::calculateBoundingCtuTsAddrForSlice(uint32_t &startCtuTSAddrSlice, uint32_t &boundingCtuTSAddrSlice, bool &haveReachedTileBoundary,
                                                   Picture* pcPic, const int sliceMode, const int sliceArgument)
{
  Slice* pcSlice = pcPic->slices[getSliceSegmentIdx()];
  const BrickMap& tileMap = *( pcPic->brickMap );
  const PPS &pps         = *( pcSlice->getPPS() );
  const uint32_t numberOfCtusInFrame = pcPic->cs->pcv->sizeInCtus;
  boundingCtuTSAddrSlice=0;
  haveReachedTileBoundary=false;

  switch (sliceMode)
  {
    case FIXED_NUMBER_OF_CTU:
      {
        uint32_t ctuAddrIncrement    = sliceArgument;
        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    case FIXED_NUMBER_OF_BYTES:
      boundingCtuTSAddrSlice  = numberOfCtusInFrame; // This will be adjusted later if required.
      break;
    case FIXED_NUMBER_OF_TILES:
      {
      const uint32_t startBrickIdx = tileMap.getBrickIdxBsMap(startCtuTSAddrSlice);
      uint32_t endBrickIdx = -1;
      if (pps.getRectSliceFlag())  //rectangular slice
      {
        uint32_t sliceIdx = 0;
        while (endBrickIdx == -1 && sliceIdx <= pps.getNumSlicesInPicMinus1())
        {
          if (pps.getTopLeftBrickIdx(sliceIdx) == startBrickIdx)
            endBrickIdx = pps.getBottomRightBrickIdx(sliceIdx);
          sliceIdx++;
        }
        if (endBrickIdx == -1)
          EXIT("Incorrect rectangular slice definition");
    }
      else   //raster-scan slice
      {
        endBrickIdx = startBrickIdx + sliceArgument - 1;
      }

      uint32_t currentTileIdx = startBrickIdx;
      int tmpAddr = -1;
      for (int i = startCtuTSAddrSlice; i < numberOfCtusInFrame; i++)
      {
        currentTileIdx = tileMap.getBrickIdxBsMap(i);
        if (currentTileIdx == endBrickIdx)
          tmpAddr = i;
      }
      boundingCtuTSAddrSlice = (tmpAddr != -1) ? tmpAddr : numberOfCtusInFrame - 1;
      boundingCtuTSAddrSlice++;
      break;
      }
      break;
    case SINGLE_BRICK_PER_SLICE:
      {
        const uint32_t brickIdx           = tileMap.getBrickIdxRsMap( tileMap.getCtuBsToRsAddrMap(startCtuTSAddrSlice) );
        const uint32_t brickWidthInCtus   = tileMap.bricks[brickIdx].getWidthInCtus();
        const uint32_t brickHeightInCtus  = tileMap.bricks[brickIdx].getHeightInCtus();
        const uint32_t ctuAddrIncrement   = brickWidthInCtus * brickHeightInCtus;
        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    default:
      boundingCtuTSAddrSlice    = numberOfCtusInFrame;
      break;
  }

  // Adjust for tiles and wavefronts.
  const bool wavefrontsAreEnabled = pps.getEntropyCodingSyncEnabledFlag();

  if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
      (pps.getNumTileRowsMinus1() > 0 || pps.getNumTileColumnsMinus1() > 0))
  {
    const uint32_t  ctuRsAddr                  = tileMap.getCtuBsToRsAddrMap(startCtuTSAddrSlice);
    const uint32_t  startTileIdx               = tileMap.getBrickIdxRsMap(ctuRsAddr);
    const Brick&    startingTile               = tileMap.bricks[startTileIdx];
    const uint32_t  tileStartTsAddr            = tileMap.getCtuRsToBsAddrMap(startingTile.getFirstCtuRsAddr());
    const uint32_t  tileStartWidth             = startingTile.getWidthInCtus();
    const uint32_t  tileStartHeight            = startingTile.getHeightInCtus();
    const uint32_t tileLastTsAddr_excl        = tileStartTsAddr + tileStartWidth*tileStartHeight;
    const uint32_t tileBoundingCtuTsAddrSlice = tileLastTsAddr_excl;
    const uint32_t ctuColumnOfStartingTile     = ((startCtuTSAddrSlice-tileStartTsAddr)%tileStartWidth);
    if (wavefrontsAreEnabled && ctuColumnOfStartingTile!=0)
    {
      // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
      const uint32_t numberOfCTUsToEndOfRow            = tileStartWidth - ctuColumnOfStartingTile;
      const uint32_t wavefrontTileBoundingCtuAddrSlice = startCtuTSAddrSlice + numberOfCTUsToEndOfRow;
      if (wavefrontTileBoundingCtuAddrSlice < boundingCtuTSAddrSlice)
      {
        boundingCtuTSAddrSlice = wavefrontTileBoundingCtuAddrSlice;
      }
    }

    if (tileBoundingCtuTsAddrSlice < boundingCtuTSAddrSlice)
    {
      boundingCtuTSAddrSlice = tileBoundingCtuTsAddrSlice;
      haveReachedTileBoundary = true;
    }
  }
  else if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) && wavefrontsAreEnabled && ((startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) != 0))
  {
    // Adjust for wavefronts (no tiles).
    // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
    boundingCtuTSAddrSlice = std::min(boundingCtuTSAddrSlice, startCtuTSAddrSlice - (startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) + (pcPic->cs->pcv->widthInCtus));
  }
}

/** Determines the starting and bounding CTU address of current slice / dependent slice
 * \param [out] startCtuTsAddr
 * \param [out] boundingCtuTsAddr
 * \param [in]  pcPic

 * Updates startCtuTsAddr, boundingCtuTsAddr with appropriate CTU address
 */
void EncSlice::xDetermineStartAndBoundingCtuTsAddr  ( uint32_t& startCtuTsAddr, uint32_t& boundingCtuTsAddr, Picture* pcPic )
{
  Slice* pcSlice                 = pcPic->slices[getSliceSegmentIdx()];

  // Non-dependent slice
  uint32_t startCtuTsAddrSlice           = pcSlice->getSliceCurStartCtuTsAddr();
  bool haveReachedTileBoundarySlice  = false;
  uint32_t boundingCtuTsAddrSlice;
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, haveReachedTileBoundarySlice, pcPic,
                                     m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument());
  pcSlice->setSliceCurEndCtuTsAddr(   boundingCtuTsAddrSlice );
  pcSlice->setSliceCurStartCtuTsAddr( startCtuTsAddrSlice    );

  const BrickMap& tileMap = *(pcPic->brickMap);
  pcSlice->setSliceCurStartBrickIdx(tileMap.getBrickIdxBsMap(startCtuTsAddrSlice));
  if (pcSlice->getPPS()->getRectSliceFlag())
    pcSlice->setSliceCurEndBrickIdx(tileMap.getBrickIdxBsMap(boundingCtuTsAddrSlice - 1));
  else
    pcSlice->setSliceNumBricks(tileMap.getBrickIdxBsMap(boundingCtuTsAddrSlice - 1) - tileMap.getBrickIdxBsMap(startCtuTsAddrSlice) + 1);

  startCtuTsAddr = startCtuTsAddrSlice;
  boundingCtuTsAddr = boundingCtuTsAddrSlice;
}

double EncSlice::xGetQPValueAccordingToLambda ( double lambda )
{
  return 4.2005*log(lambda) + 13.7122;
}

//! \}
