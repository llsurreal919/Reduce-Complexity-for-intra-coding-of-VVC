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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CrossCompPrediction.h"


#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>

#include "QuantRDOQ.h"
#include "DepQuant.h"

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

struct coeffGroupRDStats
{
  int    iNNZbeforePos0;
  double d64CodedLevelandDist; // distortion and level cost only
  double d64UncodedDist;    // all zero coded block distortion
  double d64SigCost;
  double d64SigCost_0;
};

FwdTrans *fastFwdTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32, fastForwardDCT2_B64 },
  { nullptr,            fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32, nullptr },
  { nullptr,            fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32, nullptr },
};

InvTrans *fastInvTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64 },
  { nullptr,            fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, nullptr },
  { nullptr,            fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, nullptr },
};

//! \ingroup CommonLib
//! \{

#if JVET_O0105_ICT
static inline int64_t square( const int d ) { return d * (int64_t)d; }

template<int signedMode> std::pair<int64_t,int64_t> fwdTransformCbCr( const PelBuf &resCb, const PelBuf &resCr, PelBuf& resC1, PelBuf& resC2 )
{
  const Pel*  cb  = resCb.buf;
  const Pel*  cr  = resCr.buf;
  Pel*        c1  = resC1.buf;
  Pel*        c2  = resC2.buf;
  int64_t     d1  = 0;
  int64_t     d2  = 0;
  for( SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride, c1 += resC1.stride, c2 += resC2.stride )
  {
    for( SizeType x = 0; x < resCb.width; x++ )
    {
      int cbx = cb[x], crx = cr[x];
      if      ( signedMode ==  1 )
      {
        c1[x] = Pel( ( 4*cbx + 2*crx ) / 5 );
        d1   += square( cbx - c1[x] ) + square( crx - (c1[x]>>1) );
      }
      else if ( signedMode == -1 )
      {
        c1[x] = Pel( ( 4*cbx - 2*crx ) / 5 );
        d1   += square( cbx - c1[x] ) + square( crx - (-c1[x]>>1) );
      }
      else if ( signedMode ==  2 )
      {
        c1[x] = Pel( ( cbx + crx ) / 2 );
        d1   += square( cbx - c1[x] ) + square( crx - c1[x] );
      }
      else if ( signedMode == -2 )
      {
        c1[x] = Pel( ( cbx - crx ) / 2 );
        d1   += square( cbx - c1[x] ) + square( crx + c1[x] );
      }
      else if ( signedMode ==  3 )
      {
        c2[x] = Pel( ( 4*crx + 2*cbx ) / 5 );
        d1   += square( cbx - (c2[x]>>1) ) + square( crx - c2[x] );
      }
      else if ( signedMode == -3 )
      {
        c2[x] = Pel( ( 4*crx - 2*cbx ) / 5 );
        d1   += square( cbx - (-c2[x]>>1) ) + square( crx - c2[x] );
      }
      else
      {
        d1   += square( cbx );
        d2   += square( crx );
      }
    }
  }
  return std::make_pair(d1,d2);
}

template<int signedMode> void invTransformCbCr( PelBuf &resCb, PelBuf &resCr )
{
  Pel*  cb  = resCb.buf;
  Pel*  cr  = resCr.buf;
  for( SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride )
  {
    for( SizeType x = 0; x < resCb.width; x++ )
    {
      if      ( signedMode ==  1 )  { cr[x] =  cb[x] >> 1;  }
      else if ( signedMode == -1 )  { cr[x] = -cb[x] >> 1;  }
      else if ( signedMode ==  2 )  { cr[x] =  cb[x]; }
      else if ( signedMode == -2 )  { cr[x] = -cb[x]; }
      else if ( signedMode ==  3 )  { cb[x] =  cr[x] >> 1; }
      else if ( signedMode == -3 )  { cb[x] = -cr[x] >> 1; }
    }
  }
}
#endif

// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================
TrQuant::TrQuant() : m_quant( nullptr )
{
  // allocate temporary buffers
#if JVET_O0105_ICT
  {
    m_invICT      = m_invICTMem + maxAbsIctMode;
    m_invICT[ 0]  = invTransformCbCr< 0>;
    m_invICT[ 1]  = invTransformCbCr< 1>;
    m_invICT[-1]  = invTransformCbCr<-1>;
    m_invICT[ 2]  = invTransformCbCr< 2>;
    m_invICT[-2]  = invTransformCbCr<-2>;
    m_invICT[ 3]  = invTransformCbCr< 3>;
    m_invICT[-3]  = invTransformCbCr<-3>;
    m_fwdICT      = m_fwdICTMem + maxAbsIctMode;
    m_fwdICT[ 0]  = fwdTransformCbCr< 0>;
    m_fwdICT[ 1]  = fwdTransformCbCr< 1>;
    m_fwdICT[-1]  = fwdTransformCbCr<-1>;
    m_fwdICT[ 2]  = fwdTransformCbCr< 2>;
    m_fwdICT[-2]  = fwdTransformCbCr<-2>;
    m_fwdICT[ 3]  = fwdTransformCbCr< 3>;
    m_fwdICT[-3]  = fwdTransformCbCr<-3>;
  }
#endif
}

TrQuant::~TrQuant()
{
  if( m_quant )
  {
    delete m_quant;
    m_quant = nullptr;
  }
}

#if ENABLE_SPLIT_PARALLELISM
void TrQuant::copyState( const TrQuant& other )
{
  m_quant->copyState( *other.m_quant );
}
#endif

void TrQuant::xDeQuant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  m_quant->dequant( tu, dstCoeff, compID, cQP );
}

void TrQuant::init( const Quant* otherQuant,
                    const uint32_t uiMaxTrSize,
                    const bool bUseRDOQ,
                    const bool bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                    const bool useSelectiveRDOQ,
#endif
                    const bool bEnc,
                    const bool useTransformSkipFast
)
{
  m_uiMaxTrSize          = uiMaxTrSize;
  m_bEnc                 = bEnc;
  m_useTransformSkipFast = useTransformSkipFast;

  delete m_quant;
  m_quant = nullptr;

  {
    m_quant = new DepQuant( otherQuant, bEnc );
  }

  if( m_quant )
  {
    m_quant->init( uiMaxTrSize, bUseRDOQ, bUseRDOQTS, useSelectiveRDOQ );
  }
}

void TrQuant::fwdLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  const int8_t* trMat  = ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ];
  const int     trSize = ( size > 4 ) ? 48 : 16;
  int           coef;
  int*          out    = dst;

  assert( index < 3 );

  for( int j = 0; j < zeroOutSize; j++ )
  {
    int*          srcPtr   = src;
    const int8_t* trMatTmp = trMat;
    coef = 0;
    for( int i = 0; i < trSize; i++ )
    {
      coef += *srcPtr++ * *trMatTmp++;
    }
    *out++ = ( coef + 64 ) >> 7;
    trMat += trSize;
  }

  ::memset( out, 0, ( trSize - zeroOutSize ) * sizeof( int ) );
}

void TrQuant::invLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  int             maxLog2TrDynamicRange =  15;
  const TCoeff    outputMinimum         = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum         =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const int8_t*   trMat                 =  ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ];
  const int       trSize                =  ( size > 4 ) ? 48 : 16;
  int             resi;
  int*            out                   =  dst;

  assert( index < 3 );

  for( int j = 0; j < trSize; j++ )
  {
    resi = 0;
    const int8_t* trMatTmp = trMat;
    int*          srcPtr   = src;
    for( int i = 0; i < zeroOutSize; i++ )
    {
      resi += *srcPtr++ * *trMatTmp;
      trMatTmp += trSize;
    }
    *out++ = Clip3( outputMinimum, outputMaximum, ( int ) ( resi + 64 ) >> 7 );
    trMat++;
  }
}

uint32_t TrQuant::getLFNSTIntraMode( int wideAngPredMode )
{
  uint32_t intraMode;

  if( wideAngPredMode < 0 )
  {
    intraMode = ( uint32_t ) ( wideAngPredMode + ( NUM_EXT_LUMA_MODE >> 1 ) + NUM_LUMA_MODE );
  }
  else if( wideAngPredMode >= NUM_LUMA_MODE )
  {
    intraMode = ( uint32_t ) ( wideAngPredMode + ( NUM_EXT_LUMA_MODE >> 1 ) );
  }
  else
  {
    intraMode = ( uint32_t ) wideAngPredMode;
  }

  return intraMode;
}

bool TrQuant::getTransposeFlag( uint32_t intraMode )
{
  return ( ( intraMode >= NUM_LUMA_MODE ) && ( intraMode >= ( NUM_LUMA_MODE + ( NUM_EXT_LUMA_MODE >> 1 ) ) ) ) ||
         ( ( intraMode <  NUM_LUMA_MODE ) && ( intraMode >  DIA_IDX ) );
}

void TrQuant::xInvLfnst( const TransformUnit &tu, const ComponentID compID )
{
  const CompArea& area     = tu.blocks[ compID ];
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx;

  if( lfnstIdx && tu.mtsIdx != MTS_SKIP && width >= 4 && height >= 4 )
  {
    const bool whge3 = width >= 8 && height >= 8;
    const ScanElement * scan = whge3 ? g_coefTopLeftDiagScan8x8[ gp_sizeIdxInfo->idxFrom( width ) ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ gp_sizeIdxInfo->idxFrom( width ) ][ gp_sizeIdxInfo->idxFrom( height ) ];
    uint32_t intraMode = PU::getFinalIntraMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) );

    if( PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
    {
#if JVET_O0219_LFNST_TRANSFORM_SET_FOR_LMCMODE
      intraMode = PU::getCoLocatedIntraLumaMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ) );
#else
      intraMode = PLANAR_IDX;
#endif
    }
#if JVET_O0925_MIP_SIMPLIFICATIONS
    if (PU::isMIP(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID)))
    {
      intraMode = PLANAR_IDX;
    }
#endif
    CHECK( intraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode" );

    if( lfnstIdx < 3 )
    {
      intraMode = getLFNSTIntraMode( PU::getWideAngIntraMode( tu, intraMode, compID ) );
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
      CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType { STATS__TOOL_LFNST, width, height, compID } );
#endif
      bool          transposeFlag   = getTransposeFlag( intraMode );
      const int     sbSize          = whge3 ? 8 : 4;
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
      const int     subGrpXMax      = ( height == 4 && width  > 8 ) ? 2 : 1;
      const int     subGrpYMax      = ( width  == 4 && height > 8 ) ? 2 : 1;
#endif
      bool          tu4x4Flag       = ( width == 4 && height == 4 );
      bool          tu8x8Flag       = ( width == 8 && height == 8 );
      TCoeff*       lfnstTemp;
      TCoeff*       coeffTemp;
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
      for( int subGroupX = 0; subGroupX < subGrpXMax; subGroupX++ )
      {
        for( int subGroupY = 0; subGroupY < subGrpYMax; subGroupY++ )
        {
          const int offsetX = sbSize * subGroupX;
          const int offsetY = sbSize * subGroupY * width;
          int y;
          lfnstTemp = m_tempInMatrix; // inverse spectral rearrangement
          coeffTemp = m_tempCoeff + offsetX + offsetY;
#else
          int y;
          lfnstTemp = m_tempInMatrix; // inverse spectral rearrangement
          coeffTemp = m_tempCoeff;
#endif
          TCoeff * dst = lfnstTemp;
          const ScanElement * scanPtr = scan;
          for( y = 0; y < 16; y++ )
          {
            *dst++ = coeffTemp[ scanPtr->idx ];
            scanPtr++;
          }

          invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, ( tu4x4Flag || tu8x8Flag ) ? 8 : 16 );

          lfnstTemp = m_tempOutMatrix; // inverse spectral rearrangement

          if( transposeFlag )
          {
            if( sbSize == 4 )
            {
              for( y = 0; y < 4; y++ )
              {
                coeffTemp[ 0 ] = lfnstTemp[ 0 ];  coeffTemp[ 1 ] = lfnstTemp[  4 ];
                coeffTemp[ 2 ] = lfnstTemp[ 8 ];  coeffTemp[ 3 ] = lfnstTemp[ 12 ];
                lfnstTemp++;
                coeffTemp += width;
              }
            }
            else // ( sbSize == 8 )
            {
              for( y = 0; y < 8; y++ )
              {
                coeffTemp[ 0 ] = lfnstTemp[  0 ];  coeffTemp[ 1 ] = lfnstTemp[  8 ];
                coeffTemp[ 2 ] = lfnstTemp[ 16 ];  coeffTemp[ 3 ] = lfnstTemp[ 24 ];
                if( y < 4 )
                {
                  coeffTemp[ 4 ] = lfnstTemp[ 32 ];  coeffTemp[ 5 ] = lfnstTemp[ 36 ];
                  coeffTemp[ 6 ] = lfnstTemp[ 40 ];  coeffTemp[ 7 ] = lfnstTemp[ 44 ];
                }
                lfnstTemp++;
                coeffTemp += width;
              }
            }
          }
          else
          {
            for( y = 0; y < sbSize; y++ )
            {
              uint32_t uiStride = ( y < 4 ) ? sbSize : 4;
              ::memcpy( coeffTemp, lfnstTemp, uiStride * sizeof( TCoeff ) );
              lfnstTemp += uiStride;
              coeffTemp += width;
            }
          }
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
        }
      } // subGroupX
#endif
    }
  }
}

void TrQuant::xFwdLfnst( const TransformUnit &tu, const ComponentID compID, const bool loadTr )
{
  const CompArea& area     = tu.blocks[ compID ];
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx;

  if( lfnstIdx && tu.mtsIdx != MTS_SKIP && width >= 4 && height >= 4 )
  {
    const bool whge3 = width >= 8 && height >= 8;
    const ScanElement * scan = whge3 ? g_coefTopLeftDiagScan8x8[ gp_sizeIdxInfo->idxFrom( width ) ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ gp_sizeIdxInfo->idxFrom( width ) ][ gp_sizeIdxInfo->idxFrom( height ) ];
    uint32_t intraMode = PU::getFinalIntraMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) );

    if( PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
    {
#if JVET_O0219_LFNST_TRANSFORM_SET_FOR_LMCMODE
      intraMode = PU::getCoLocatedIntraLumaMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ) );
#else
      intraMode = PLANAR_IDX;
#endif
    }
#if JVET_O0925_MIP_SIMPLIFICATIONS
    if (PU::isMIP(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID)))
    {
      intraMode = PLANAR_IDX;
    }
#endif
    CHECK( intraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode" );

    if( lfnstIdx < 3 )
    {
      intraMode = getLFNSTIntraMode( PU::getWideAngIntraMode( tu, intraMode, compID ) );

      bool            transposeFlag   = getTransposeFlag( intraMode );
      const int       sbSize          = whge3 ? 8 : 4;
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
      const int       subGrpXMax      = ( height == 4 && width  > 8 ) ? 2 : 1;
      const int       subGrpYMax      = ( width  == 4 && height > 8 ) ? 2 : 1;
#endif
      bool            tu4x4Flag       = ( width == 4 && height == 4 );
      bool            tu8x8Flag       = ( width == 8 && height == 8 );
      TCoeff*         lfnstTemp;
      TCoeff*         coeffTemp;
      TCoeff *        tempCoeff = loadTr ? m_mtsCoeffs[tu.mtsIdx] : m_tempCoeff;

#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
      for( int subGroupX = 0; subGroupX < subGrpXMax; subGroupX++ )
      {
        for( int subGroupY = 0; subGroupY < subGrpYMax; subGroupY++ )
        {
          const int offsetX = sbSize * subGroupX;
          const int offsetY = sbSize * subGroupY * width;
          int y;
          lfnstTemp = m_tempInMatrix; // forward low frequency non-separable transform
          coeffTemp = tempCoeff + offsetX + offsetY;
#else
          int y;
          lfnstTemp = m_tempInMatrix; // forward low frequency non-separable transform
          coeffTemp = tempCoeff;
#endif

          if( transposeFlag )
          {
            if( sbSize == 4 )
            {
              for( y = 0; y < 4; y++ )
              {
                lfnstTemp[ 0 ] = coeffTemp[ 0 ];  lfnstTemp[  4 ] = coeffTemp[ 1 ];
                lfnstTemp[ 8 ] = coeffTemp[ 2 ];  lfnstTemp[ 12 ] = coeffTemp[ 3 ];
                lfnstTemp++;
                coeffTemp += width;
              }
            }
            else // ( sbSize == 8 )
            {
              for( y = 0; y < 8; y++ )
              {
                lfnstTemp[  0 ] = coeffTemp[ 0 ];  lfnstTemp[  8 ] = coeffTemp[ 1 ];
                lfnstTemp[ 16 ] = coeffTemp[ 2 ];  lfnstTemp[ 24 ] = coeffTemp[ 3 ];
                if( y < 4 )
                {
                  lfnstTemp[ 32 ] = coeffTemp[ 4 ];  lfnstTemp[ 36 ] = coeffTemp[ 5 ];
                  lfnstTemp[ 40 ] = coeffTemp[ 6 ];  lfnstTemp[ 44 ] = coeffTemp[ 7 ];
                }
                lfnstTemp++;
                coeffTemp += width;
              }
            }
          }
          else
          {
            for( y = 0; y < sbSize; y++ )
            {
              uint32_t uiStride = ( y < 4 ) ? sbSize : 4;
              ::memcpy( lfnstTemp, coeffTemp, uiStride * sizeof( TCoeff ) );
              lfnstTemp += uiStride;
              coeffTemp += width;
            }
          }

          fwdLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, ( tu4x4Flag || tu8x8Flag ) ? 8 : 16 );

          lfnstTemp = m_tempOutMatrix; // forward spectral rearrangement
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
          coeffTemp = tempCoeff + offsetX + offsetY;
#else
          coeffTemp = tempCoeff;
#endif
          const ScanElement * scanPtr = scan;
          int lfnstCoeffNum = ( sbSize == 4 ) ? sbSize * sbSize : 48;
          for( y = 0; y < lfnstCoeffNum; y++ )
          {
            coeffTemp[ scanPtr->idx ] = *lfnstTemp++;
            scanPtr++;
          }
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
        }
      } // subGroupX
#endif
    }
  }
}


void TrQuant::invTransformNxN( TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQP )
{
  const CompArea &area    = tu.blocks[compID];
  const uint32_t uiWidth      = area.width;
  const uint32_t uiHeight     = area.height;

#if MAX_TB_SIZE_SIGNALLING
  CHECK( uiWidth > tu.cs->sps->getMaxTbSize() || uiHeight > tu.cs->sps->getMaxTbSize(), "Maximal allowed transformation size exceeded!" );
#else
  CHECK( uiWidth > MAX_TB_SIZEY || uiHeight > MAX_TB_SIZEY, "Maximal allowed transformation size exceeded!" );
#endif
  if (tu.cu->transQuantBypass)
  {
    // where should this logic go?
    const bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
    const CCoeffBuf pCoeff    = tu.getCoeffs(compID);

    for (uint32_t y = 0, coefficientIndex = 0; y < uiHeight; y++)
    {
      for (uint32_t x = 0; x < uiWidth; x++, coefficientIndex++)
      {
        pResi.at(x, y) = rotateResidual ? pCoeff.at(pCoeff.width - x - 1, pCoeff.height - y - 1) : pCoeff.at(x, y);
      }
    }
  }
  else
  {
    CoeffBuf tempCoeff = CoeffBuf(m_tempCoeff, area);
    xDeQuant( tu, tempCoeff, compID, cQP );

    DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

    if( tu.cs->sps->getUseLFNST() )
    {
      xInvLfnst( tu, compID );
    }

    if( isLuma(compID) && tu.mtsIdx == MTS_SKIP )
    {
      xITransformSkip( tempCoeff, pResi, tu, compID );
    }
    else
    {
      xIT( tu, compID, tempCoeff, pResi );
    }
  }

  //DTRACE_BLOCK_COEFF(tu.getCoeffs(compID), tu, tu.cu->predMode, compID);
  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode, compID);
  invRdpcmNxN(tu, compID, pResi);
}

void TrQuant::invRdpcmNxN(TransformUnit& tu, const ComponentID &compID, PelBuf &pcResidual)
{
  const CompArea &area    = tu.blocks[compID];

  if (CU::isRDPCMEnabled(*tu.cu) && (tu.mtsIdx==MTS_SKIP || tu.cu->transQuantBypass))
  {
    const uint32_t uiWidth  = area.width;
    const uint32_t uiHeight = area.height;

    RDPCMMode rdpcmMode = RDPCM_OFF;

    if (tu.cu->predMode == MODE_INTRA)
    {
      const ChannelType chType = toChannelType(compID);
      const uint32_t uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), chType), chType);

      if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
      {
        rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      }
    }
    else  // not intra case
    {
      rdpcmMode = RDPCMMode(tu.rdpcm[compID]);
    }

    const TCoeff pelMin = (TCoeff) std::numeric_limits<Pel>::min();
    const TCoeff pelMax = (TCoeff) std::numeric_limits<Pel>::max();

    if (rdpcmMode == RDPCM_VER)
    {
      for (uint32_t uiX = 0; uiX < uiWidth; uiX++)
      {
        TCoeff accumulator = pcResidual.at(uiX, 0); // 32-bit accumulator

        for (uint32_t uiY = 1; uiY < uiHeight; uiY++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
    else if (rdpcmMode == RDPCM_HOR)
    {
      for (uint32_t uiY = 0; uiY < uiHeight; uiY++)
      {
        TCoeff accumulator = pcResidual.at(0, uiY);

        for (uint32_t uiX = 1; uiX < uiWidth; uiX++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
  }
}

#if JVET_O0105_ICT

std::pair<int64_t,int64_t> TrQuant::fwdTransformICT( const TransformUnit &tu, const PelBuf &resCb, const PelBuf &resCr, PelBuf &resC1, PelBuf &resC2, int jointCbCr )
{
  CHECK( Size(resCb) != Size(resCr), "resCb and resCr have different sizes" );
  CHECK( Size(resCb) != Size(resC1), "resCb and resC1 have different sizes" );
  CHECK( Size(resCb) != Size(resC2), "resCb and resC2 have different sizes" );
  return (*m_fwdICT[ TU::getICTMode(tu, jointCbCr) ])( resCb, resCr, resC1, resC2 );
}

void TrQuant::invTransformICT( const TransformUnit &tu, PelBuf &resCb, PelBuf &resCr )
{
  CHECK( Size(resCb) != Size(resCr), "resCb and resCr have different sizes" );
  (*m_invICT[ TU::getICTMode(tu) ])( resCb, resCr );
}

std::vector<int> TrQuant::selectICTCandidates( const TransformUnit &tu, CompStorage* resCb, CompStorage* resCr )
{
  CHECK( !resCb[0].valid() || !resCr[0].valid(), "standard components are not valid" );

#if JVET_O0543_ICT_ICU_ONLY
  if( !CU::isIntra( *tu.cu ) )
  {
    int cbfMask = 3;
    resCb[cbfMask].create( tu.blocks[COMPONENT_Cb] );
    resCr[cbfMask].create( tu.blocks[COMPONENT_Cr] );
    fwdTransformICT( tu, resCb[0], resCr[0], resCb[cbfMask], resCr[cbfMask], cbfMask );
    std::vector<int> cbfMasksToTest;
    cbfMasksToTest.push_back( cbfMask );
    return cbfMasksToTest;
  }
#endif

  std::pair<int64_t,int64_t> pairDist[4];
  for( int cbfMask = 0; cbfMask < 4; cbfMask++ )
  {
    if( cbfMask )
    {
      CHECK( resCb[cbfMask].valid() || resCr[cbfMask].valid(), "target components for cbfMask=" << cbfMask << " are already present" );
      resCb[cbfMask].create( tu.blocks[COMPONENT_Cb] );
      resCr[cbfMask].create( tu.blocks[COMPONENT_Cr] );
    }
    pairDist[cbfMask] = fwdTransformICT( tu, resCb[0], resCr[0], resCb[cbfMask], resCr[cbfMask], cbfMask );
  }

  std::vector<int> cbfMasksToTest;
  int64_t minDist1  = std::min<int64_t>( pairDist[0].first, pairDist[0].second );
  int64_t minDist2  = std::numeric_limits<int64_t>::max();
  int     cbfMask1  = 0;
  int     cbfMask2  = 0;
  for( int cbfMask : { 1, 2, 3 } )
  {
    if( pairDist[cbfMask].first < minDist1 )
    {
      cbfMask2  = cbfMask1; minDist2  = minDist1;
      cbfMask1  = cbfMask;  minDist1  = pairDist[cbfMask1].first;
    }
    else if( pairDist[cbfMask].first < minDist2 )
    {
      cbfMask2  = cbfMask;  minDist2  = pairDist[cbfMask2].first;
    }
  }
  if( cbfMask1 )
  {
    cbfMasksToTest.push_back( cbfMask1 );
  }
  if( cbfMask2 && ( ( minDist2 < (9*minDist1)/8 ) || ( !cbfMask1 && minDist2 < (3*minDist1)/2 ) ) )
  {
    cbfMasksToTest.push_back( cbfMask2 );
  }

  return cbfMasksToTest;
}

#endif


// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

void TrQuant::getTrTypes(const TransformUnit tu, const ComponentID compID, int &trTypeHor, int &trTypeVer)
{
  const bool isExplicitMTS = (CU::isIntra(*tu.cu) ? tu.cs->sps->getUseIntraMTS() : tu.cs->sps->getUseInterMTS() && CU::isInter(*tu.cu)) && isLuma(compID);
#if JVET_O0529_IMPLICIT_MTS_HARMONIZE
  const bool isImplicitMTS = CU::isIntra(*tu.cu) && tu.cs->sps->getUseImplicitMTS() && isLuma(compID) && tu.cu->lfnstIdx == 0 && tu.cu->mipFlag == 0;
#else
  const bool isImplicitMTS = CU::isIntra(*tu.cu) && tu.cs->sps->getUseImplicitMTS() && isLuma(compID);
#endif
  const bool isISP = CU::isIntra(*tu.cu) && tu.cu->ispMode && isLuma(compID);
  const bool isSBT = CU::isInter(*tu.cu) && tu.cu->sbtInfo && isLuma(compID);

  trTypeHor = DCT2;
  trTypeVer = DCT2;

#if JVET_O0538_SPS_CONTROL_ISP_SBT
  if (!tu.cs->sps->getUseMTS())
    return;
#endif

  if (isImplicitMTS || isISP)
  {
    int  width = tu.blocks[compID].width;
    int  height = tu.blocks[compID].height;
    bool widthDstOk = width >= 4 && width <= 16;
    bool heightDstOk = height >= 4 && height <= 16;

    if (widthDstOk)
      trTypeHor = DST7;
    if (heightDstOk)
      trTypeVer = DST7;
    return;
  }


  if (isSBT)
  {
    uint8_t sbtIdx = tu.cu->getSbtIdx();
    uint8_t sbtPos = tu.cu->getSbtPos();

    if( sbtIdx == SBT_VER_HALF || sbtIdx == SBT_VER_QUAD )
    {
      assert( tu.lwidth() <= MTS_INTER_MAX_CU_SIZE );
      if( tu.lheight() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DCT8;  trTypeVer = DST7; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
    else
    {
      assert( tu.lheight() <= MTS_INTER_MAX_CU_SIZE );
      if( tu.lwidth() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DST7;  trTypeVer = DCT8; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
    return;
  }

  if (isExplicitMTS)
  {
    if (tu.mtsIdx > MTS_SKIP)
    {
      int indHor = (tu.mtsIdx - MTS_DST7_DST7) & 1;
      int indVer = (tu.mtsIdx - MTS_DST7_DST7) >> 1;

      trTypeHor = indHor ? DCT8 : DST7;
      trTypeVer = indVer ? DCT8 : DST7;
    }
  }
}



void TrQuant::xT( const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff, const int width, const int height )
{
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->getBitDepth(              toChannelType( compID ) );
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
  const uint32_t transformWidthIndex    = floorLog2(width ) - 1;  // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = floorLog2(height) - 1;  // nLog2HeightMinus1, since transform start from 2-point


  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  getTrTypes ( tu, compID, trTypeHor, trTypeVer );

#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
  const int      skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  const int      skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#else
  int  skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int  skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
  if( tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx )
  {
    if( (width == 4 && height > 4) || (width > 4 && height == 4) )
    {
      skipWidth  = width  - 4;
      skipHeight = height - 4;
    }
    else if( (width >= 8 && height >= 8) )
    {
      skipWidth  = width  - 8;
      skipHeight = height - 8;
    }
  }
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  if ( trTypeHor != DCT2 )
  {
    CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType{ STATS__TOOL_EMT, uint32_t( width ), uint32_t( height ), compID } );
  }
#endif

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TB_SIZEY * MAX_TB_SIZEY] );

  const Pel *resiBuf    = resi.buf;
  const int  resiStride = resi.stride;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      block[( y * width ) + x] = resiBuf[( y * resiStride ) + x];
    }
  }

  if( width > 1 && height > 1 ) // 2-D transform
  {
    const int      shift_1st              = ((floorLog2(width )) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    const int      shift_2nd              =  (floorLog2(height))            + TRANSFORM_MATRIX_SHIFT                          + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
  TCoeff *tmp = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );

  fastFwdTrans[trTypeHor][transformWidthIndex ](block,        tmp, shift_1st, height,        0, skipWidth);
  fastFwdTrans[trTypeVer][transformHeightIndex](tmp, dstCoeff.buf, shift_2nd, width, skipWidth, skipHeight);
  }
  else if( height == 1 ) //1-D horizontal transform
  {
    const int      shift              = ((floorLog2(width )) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECKD( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastFwdTrans[trTypeHor][transformWidthIndex]( block, dstCoeff.buf, shift, 1, 0, skipWidth );
  }
  else //if (iWidth == 1) //1-D vertical transform
  {
    int shift = ( ( floorLog2(height) ) + bitDepth + TRANSFORM_MATRIX_SHIFT ) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECKD( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastFwdTrans[trTypeVer][transformHeightIndex]( block, dstCoeff.buf, shift, 1, 0, skipHeight );
  }
}

void TrQuant::xIT( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual )
{
  const int      width                  = pCoeff.width;
  const int      height                 = pCoeff.height;
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->getBitDepth(              toChannelType( compID ) );
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const TCoeff   clipMinimum            = -( 1 << maxLog2TrDynamicRange );
  const TCoeff   clipMaximum            =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const uint32_t transformWidthIndex    = floorLog2(width ) - 1;                                // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = floorLog2(height) - 1;                                // nLog2HeightMinus1, since transform start from 2-point


  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  getTrTypes ( tu, compID, trTypeHor, trTypeVer );
#if !JVET_O0094_LFNST_ZERO_PRIM_COEFFS
  const int      skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  const int      skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#else
  int skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
  if( tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx )
  {
    if( (width == 4 && height > 4) || (width > 4 && height == 4) )
    {
      skipWidth  = width  - 4;
      skipHeight = height - 4;
    }
    else if( (width >= 8 && height >= 8) )
    {
      skipWidth  = width  - 8;
      skipHeight = height - 8;
    }
  }
#endif

  TCoeff *block = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );

  if( width > 1 && height > 1 ) //2-D transform
  {
    const int      shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; // 1 has been added to shift_1st at the expense of shift_2nd
    const int      shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
    TCoeff *tmp = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );
  fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, tmp, shift_1st, width, skipWidth, skipHeight, clipMinimum, clipMaximum);
  fastInvTrans[trTypeHor][transformWidthIndex] (tmp,      block, shift_2nd, height,         0, skipWidth, clipMinimum, clipMaximum);
  }
  else if( width == 1 ) //1-D vertical transform
  {
    int shift = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastInvTrans[trTypeVer][transformHeightIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipHeight, clipMinimum, clipMaximum );
  }
  else //if(iHeight == 1) //1-D horizontal transform
  {
    const int      shift              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastInvTrans[trTypeHor][transformWidthIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipWidth, clipMinimum, clipMaximum );
  }

  Pel *resiBuf    = pResidual.buf;
  int  resiStride = pResidual.stride;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      resiBuf[( y * resiStride ) + x] = Pel( block[( y * width ) + x] );
    }
  }
}

/** Wrapper function between HM interface and core NxN transform skipping
 */
void TrQuant::xITransformSkip(const CCoeffBuf     &pCoeff,
                                    PelBuf        &pResidual,
                              const TransformUnit &tu,
                              const ComponentID   &compID)
{
  const CompArea &area      = tu.blocks[compID];
  const int width           = area.width;
  const int height          = area.height;
  const int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const int channelBitDepth = tu.cs->sps->getBitDepth(toChannelType(compID));

  int iTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  if( tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<int>( 0, iTransformShift );
  }

  int iWHScale = 1;

  const bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

  if( iTransformShift >= 0 )
  {
    const TCoeff offset = iTransformShift == 0 ? 0 : ( 1 << ( iTransformShift - 1 ) );

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) ) * iWHScale + offset ) >> iTransformShift );
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) )  * iWHScale << iTransformShift );
      }
    }
  }
}

void TrQuant::xQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx)
{
  m_quant->quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
}

#if JVET_O0502_ISP_CLEANUP
void TrQuant::transformNxN( TransformUnit& tu, const ComponentID& compID, const QpParam& cQP, std::vector<TrMode>* trModes, const int maxCand )
#else
void TrQuant::transformNxN( TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, std::vector<TrMode>* trModes, const int maxCand, double* diagRatio, double* horVerRatio )
#endif
{
        CodingStructure &cs = *tu.cs;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t width      = rect.width;
  const uint32_t height     = rect.height;

  const CPelBuf  resiBuf    = cs.getResiBuf(rect);

#if MAX_TB_SIZE_SIGNALLING
  CHECK( cs.sps->getMaxTbSize() < width, "Unsupported transformation size" );
#else
  CHECK( MAX_TB_SIZEY < width, "Unsupported transformation size" );
#endif

  int pos = 0;
  std::vector<TrCost> trCosts;
  std::vector<TrMode>::iterator it = trModes->begin();
  const double facBB[] = { 1.2, 1.3, 1.3, 1.4, 1.5 };
  while( it != trModes->end() )
  {
    tu.mtsIdx = it->first;
    CoeffBuf tempCoeff( m_mtsCoeffs[tu.mtsIdx], rect );
    if( tu.noResidual )
    {
      int sumAbs = 0;
      trCosts.push_back( TrCost( sumAbs, pos++ ) );
      it++;
      continue;
    }

    if( isLuma(compID) && tu.mtsIdx == MTS_SKIP )
    {
      xTransformSkip( tu, compID, resiBuf, tempCoeff.buf );
    }
    else
    {
      xT( tu, compID, resiBuf, tempCoeff, width, height );
    }

    int sumAbs = 0;
    for( int pos = 0; pos < width*height; pos++ )
    {
      sumAbs += abs( tempCoeff.buf[pos] );
    }

    double scaleSAD=1.0;
    if (isLuma(compID) && tu.mtsIdx==MTS_SKIP && ((floorLog2(width) + floorLog2(height)) & 1) == 1 )
    {
      scaleSAD=1.0/1.414213562; // compensate for not scaling transform skip coefficients by 1/sqrt(2)
    }
    trCosts.push_back( TrCost( int(sumAbs*scaleSAD), pos++ ) );
    it++;
  }

#if !JVET_O0502_ISP_CLEANUP
  // it gets the distribution of the DCT-II coefficients energy, which will be useful to discard ISP tests
  CoeffBuf coeffsDCT( m_mtsCoeffs[0], rect );
  xGetCoeffEnergy( tu, compID, coeffsDCT, diagRatio, horVerRatio );
#endif
  int numTests = 0;
  std::vector<TrCost>::iterator itC = trCosts.begin();
  const double fac   = facBB[floorLog2(std::max(width, height))-2];
  const double thr   = fac * trCosts.begin()->first;
  const double thrTS = trCosts.begin()->first;
  while( itC != trCosts.end() )
  {
    const bool testTr = itC->first <= ( itC->second == 1 ? thrTS : thr ) && numTests <= maxCand;
    trModes->at( itC->second ).second = testTr;
    numTests += testTr;
    itC++;
  }
}

#if JVET_O0502_ISP_CLEANUP
void TrQuant::transformNxN( TransformUnit& tu, const ComponentID& compID, const QpParam& cQP, TCoeff& uiAbsSum, const Ctx& ctx, const bool loadTr )
#else
void TrQuant::transformNxN( TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx, const bool loadTr, double* diagRatio, double* horVerRatio )
#endif
{
        CodingStructure &cs = *tu.cs;
  const SPS &sps            = *cs.sps;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;

  const CPelBuf resiBuf     = cs.getResiBuf(rect);
        CoeffBuf rpcCoeff   = tu.getCoeffs(compID);

  if( tu.noResidual )
  {
    uiAbsSum = 0;
    TU::setCbfAtDepth( tu, compID, tu.depth, uiAbsSum > 0 );
    return;
  }

  RDPCMMode rdpcmMode = RDPCM_OFF;
  rdpcmNxN(tu, compID, cQP, uiAbsSum, rdpcmMode);

  if( tu.cu->bdpcmMode && isLuma(compID) )
  {
    tu.mtsIdx = MTS_SKIP;
  }

  if (rdpcmMode == RDPCM_OFF)
  {
    uiAbsSum = 0;

    // transform and quantize
    if (CU::isLosslessCoded(*tu.cu))
    {
      const bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

      for( uint32_t y = 0; y < uiHeight; y++ )
      {
        for( uint32_t x = 0; x < uiWidth; x++ )
        {
          const Pel currentSample = resiBuf.at( x, y );

          if( rotateResidual )
          {
            rpcCoeff.at( uiWidth - x - 1, uiHeight - y - 1 ) = currentSample;
          }
          else
          {
            rpcCoeff.at( x, y ) = currentSample;
          }

          uiAbsSum += TCoeff( abs( currentSample ) );
        }
      }
    }
    else
    {
#if MAX_TB_SIZE_SIGNALLING
      CHECK( cs.sps->getMaxTbSize() < uiWidth, "Unsupported transformation size" );

#else
      CHECK( MAX_TB_SIZEY < uiWidth, "Unsupported transformation size" );
#endif

      CoeffBuf tempCoeff(loadTr ? m_mtsCoeffs[tu.mtsIdx] : m_tempCoeff, rect);

      DTRACE_PEL_BUF( D_RESIDUALS, resiBuf, tu, tu.cu->predMode, compID );

      if( !loadTr )
      {
        if( isLuma(compID) && tu.mtsIdx == MTS_SKIP )
      {
        xTransformSkip( tu, compID, resiBuf, tempCoeff.buf );
      }
      else
      {
        xT( tu, compID, resiBuf, tempCoeff, uiWidth, uiHeight );
      }
      }

#if !JVET_O0502_ISP_CLEANUP
      //we do this only with the DCT-II coefficients
      if( isLuma(compID) &&
        !loadTr && tu.mtsIdx == MTS_DCT2_DCT2
        )
      {
        //it gets the distribution of the coefficients energy, which will be useful to discard ISP tests
        xGetCoeffEnergy( tu, compID, tempCoeff, diagRatio, horVerRatio );
      }
#endif

      if( sps.getUseLFNST() )
      {
        xFwdLfnst( tu, compID, loadTr );
      }

      DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

      xQuant( tu, compID, tempCoeff, uiAbsSum, cQP, ctx );

      DTRACE_COEFF_BUF( D_TCOEFF, tu.getCoeffs( compID ), tu, tu.cu->predMode, compID );
    }
  }

  // set coded block flag (CBF)
  TU::setCbfAtDepth (tu, compID, tu.depth, uiAbsSum > 0);
}

#if !JVET_O0502_ISP_CLEANUP
void TrQuant::xGetCoeffEnergy( TransformUnit &tu, const ComponentID &compID, const CoeffBuf& coeffs, double* diagRatio, double* horVerRatio )
{
  if( nullptr == diagRatio || nullptr == horVerRatio ) return;
  if( tu.cu->predMode == MODE_INTRA && !tu.cu->ispMode && isLuma( compID ) && tu.cs->sps->getUseISP() && CU::canUseISP( *tu.cu, compID ) )
  {
    const int width   = tu.cu->blocks[compID].width;
    const int height  = tu.cu->blocks[compID].height;
    const int log2Sl  = width <= height ? floorLog2(height >> floorLog2(width)) : floorLog2(width >> floorLog2(height));
    const int diPos1  = width <= height ? width  : height;
    const int diPos2  = width <= height ? height : width;
    const int ofsPos1 = width <= height ? 1 : coeffs.stride;
    const int ofsPos2 = width <= height ? coeffs.stride : 1;

    int wdtE = 0, hgtE = 0, diaE = 0;
    int* gtE = width <= height ? &wdtE : &hgtE;
    int* stE = width <= height ? &hgtE : &wdtE;

    for( int pos1 = 0; pos1 < diPos1; pos1++ )
    {
      const int posN = pos1 << log2Sl;
      for( int pos2 = 0; pos2 < diPos2; pos2++ )
      {
        const int blkP = pos1 * ofsPos1 + pos2 * ofsPos2;
        if( posN  > pos2 ) *gtE += abs( coeffs.buf[ blkP ] );
        if( posN  < pos2 ) *stE += abs( coeffs.buf[ blkP ] );
        if( posN == pos2 ) diaE += abs( coeffs.buf[ blkP ] );
      }
    }

    *horVerRatio = 0 == wdtE && 0 == hgtE ? 1 : double( wdtE ) / double( hgtE );
    *diagRatio   = 0 == wdtE && 0 == hgtE && 0 == diaE ? 1 : double( diaE ) / double( wdtE + hgtE );
  }
}
#endif

void TrQuant::applyForwardRDPCM(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const RDPCMMode &mode)
{
  const bool bLossless      = tu.cu->transQuantBypass;
  const uint32_t uiWidth        = tu.blocks[compID].width;
  const uint32_t uiHeight       = tu.blocks[compID].height;
  const bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
  const uint32_t uiSizeMinus1   = (uiWidth * uiHeight) - 1;

  const CPelBuf pcResidual  = tu.cs->getResiBuf(tu.blocks[compID]);
  const CoeffBuf pcCoeff    = tu.getCoeffs(compID);

  uint32_t uiX = 0;
  uint32_t uiY = 0;

  uint32_t &majorAxis            = (mode == RDPCM_VER) ? uiX      : uiY;
  uint32_t &minorAxis            = (mode == RDPCM_VER) ? uiY      : uiX;
  const uint32_t  majorAxisLimit = (mode == RDPCM_VER) ? uiWidth  : uiHeight;
  const uint32_t  minorAxisLimit = (mode == RDPCM_VER) ? uiHeight : uiWidth;

  const bool bUseHalfRoundingPoint = (mode != RDPCM_OFF);

  uiAbsSum = 0;

  for (majorAxis = 0; majorAxis < majorAxisLimit; majorAxis++)
  {
    TCoeff accumulatorValue = 0; // 32-bit accumulator

    for (minorAxis = 0; minorAxis < minorAxisLimit; minorAxis++)
    {
      const uint32_t sampleIndex        = (uiY * uiWidth) + uiX;
      const uint32_t coefficientIndex   = (rotateResidual ? (uiSizeMinus1-sampleIndex) : sampleIndex);
      const Pel  currentSample      = pcResidual.at(uiX, uiY);
      const TCoeff encoderSideDelta = TCoeff(currentSample) - accumulatorValue;

      Pel reconstructedDelta;

      if (bLossless)
      {
        pcCoeff.buf[coefficientIndex] = encoderSideDelta;
        reconstructedDelta            = (Pel) encoderSideDelta;
      }
      else
      {
        m_quant->transformSkipQuantOneSample(tu, compID, encoderSideDelta, pcCoeff.buf[coefficientIndex],   coefficientIndex, cQP, bUseHalfRoundingPoint);
        m_quant->invTrSkipDeQuantOneSample  (tu, compID, pcCoeff.buf[coefficientIndex], reconstructedDelta, coefficientIndex, cQP);
      }

      uiAbsSum += abs(pcCoeff.buf[coefficientIndex]);

      if (mode != RDPCM_OFF)
      {
        accumulatorValue += reconstructedDelta;
      }
    }
  }
}

void TrQuant::rdpcmNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, RDPCMMode &rdpcmMode)
{
  if (!CU::isRDPCMEnabled(*tu.cu) || (tu.mtsIdx!=MTS_SKIP && !tu.cu->transQuantBypass))
  {
    rdpcmMode = RDPCM_OFF;
  }
  else if (CU::isIntra(*tu.cu))
  {
    const ChannelType chType = toChannelType(compID);
    const uint32_t uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(tu.blocks[compID].pos(), chType), chType);

    if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
    {
      rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);
    }
    else
    {
      rdpcmMode = RDPCM_OFF;
    }
  }
  else // not intra, need to select the best mode
  {
    const CompArea &area = tu.blocks[compID];
    const uint32_t uiWidth   = area.width;
    const uint32_t uiHeight  = area.height;

    RDPCMMode bestMode = NUMBER_OF_RDPCM_MODES;
    TCoeff    bestAbsSum = std::numeric_limits<TCoeff>::max();
    TCoeff    bestCoefficients[MAX_TB_SIZEY * MAX_TB_SIZEY];

    for (uint32_t modeIndex = 0; modeIndex < NUMBER_OF_RDPCM_MODES; modeIndex++)
    {
      const RDPCMMode mode = RDPCMMode(modeIndex);

      TCoeff currAbsSum = 0;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);

      if (currAbsSum < bestAbsSum)
      {
        bestMode = mode;
        bestAbsSum = currAbsSum;

        if (mode != RDPCM_OFF)
        {
          CoeffBuf(bestCoefficients, uiWidth, uiHeight).copyFrom(tu.getCoeffs(compID));
        }
      }
    }

    rdpcmMode = bestMode;
    uiAbsSum = bestAbsSum;

    if (rdpcmMode != RDPCM_OFF) //the TU is re-transformed and quantized if DPCM_OFF is returned, so there is no need to preserve it here
    {
      tu.getCoeffs(compID).copyFrom(CoeffBuf(bestCoefficients, uiWidth, uiHeight));
    }
  }

  tu.rdpcm[compID] = rdpcmMode;
}

void TrQuant::xTransformSkip(const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t width          = rect.width;
  const uint32_t height         = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const int channelBitDepth = sps.getBitDepth(chType);
  const int maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);
  int iTransformShift       = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if( sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<int>( 0, iTransformShift );
  }

  int iWHScale = 1;

  const bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );
  const uint32_t uiSizeMinus1 = ( width * height ) - 1;

  if( iTransformShift >= 0 )
  {
    for( uint32_t y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale ) << iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const TCoeff offset = 1 << ( iTransformShift - 1 );

    for( uint32_t y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale + offset ) >> iTransformShift;
      }
    }
  }
}

//! \}
