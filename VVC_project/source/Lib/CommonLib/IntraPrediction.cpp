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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>

#include "CommonLib/InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t IntraPrediction::m_aucIntraFilter[MAX_INTRA_FILTER_DEPTHS] =
{
#if JVET_O0277_INTRA_SMALL_BLOCK_DCTIF
  24, //   1xn
  24, //   2xn
  24, //   4xn
#else
  20, //   1xn
  20, //   2xn
  20, //   4xn
#endif
  14, //   8xn
  2,  //  16xn
  0,  //  32xn
  0,  //  64xn
  0   // 128xn
};

const TFilterCoeff g_intraGaussFilter[32][4] = {
  { 16, 32, 16, 0 },
  { 15, 29, 17, 3 },
  { 15, 29, 17, 3 },
  { 14, 29, 18, 3 },
  { 13, 29, 18, 4 },
  { 13, 28, 19, 4 },
  { 13, 28, 19, 4 },
  { 12, 28, 20, 4 },
  { 11, 28, 20, 5 },
  { 11, 27, 21, 5 },
  { 10, 27, 22, 5 },
  { 9, 27, 22, 6 },
  { 9, 26, 23, 6 },
  { 9, 26, 23, 6 },
  { 8, 25, 24, 7 },
  { 8, 25, 24, 7 },
  { 8, 24, 24, 8 },
  { 7, 24, 25, 8 },
  { 7, 24, 25, 8 },
  { 6, 23, 26, 9 },
  { 6, 23, 26, 9 },
  { 6, 22, 27, 9 },
  { 5, 22, 27, 10 },
  { 5, 21, 27, 11 },
  { 5, 20, 28, 11 },
  { 4, 20, 28, 12 },
  { 4, 19, 28, 13 },
  { 4, 19, 28, 13 },
  { 4, 18, 29, 13 },
  { 3, 18, 29, 14 },
  { 3, 17, 29, 15 },
  { 3, 17, 29, 15 }
};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      m_piYuvExt[ch][buf] = nullptr;
    }
  }
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  m_piTemp = nullptr;
  m_pMdlmTemp = nullptr;

#if JVET_O0119_BASE_PALETTE_444
  m_runTypeRD   = nullptr;
  m_runLengthRD = nullptr;
#endif
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

void IntraPrediction::destroy()
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      delete[] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = nullptr;
    }
  }
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      delete[] m_yuvExt2[ch][buf];
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;
  delete[] m_pMdlmTemp;
  m_pMdlmTemp = nullptr;
#if JVET_O0119_BASE_PALETTE_444
  if (m_runTypeRD)   { xFree( m_runTypeRD  );   m_runTypeRD = NULL; }
  if (m_runLengthRD) { xFree( m_runLengthRD); m_runLengthRD = NULL; }
#endif
}

void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  if (m_yuvExt2[COMPONENT_Y][0] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
#if JVET_O0364_PADDING
    m_iYuvExtSize = (MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * (MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX);
#else
    m_iYuvExtSize = (MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX * 33) * (MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX * 33);
#endif

    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[m_iYuvExtSize];
      }
    }
  }

  if (m_yuvExt2[COMPONENT_Y][0] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_yuvExtSize2 = (MAX_CU_SIZE) * (MAX_CU_SIZE);

    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (uint32_t buf = 0; buf < 4; buf++)
      {
        m_yuvExt2[ch][buf] = new Pel[m_yuvExtSize2];
      }
    }
  }

  if (m_piTemp == nullptr)
  {
    m_piTemp = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }
  if (m_pMdlmTemp == nullptr)
  {
    m_pMdlmTemp = new Pel[(2 * MAX_CU_SIZE + 1)*(2 * MAX_CU_SIZE + 1)];//MDLM will use top-above and left-below samples.
  }
#if JVET_O0119_BASE_PALETTE_444
  if (m_runTypeRD == nullptr)
  {
    m_runTypeRD = (bool *) xMalloc(bool, MAX_CU_SIZE * MAX_CU_SIZE);
  }
  if (m_runLengthRD == nullptr)
  {
    m_runLengthRD = (Pel *) xMalloc(Pel, MAX_CU_SIZE * MAX_CU_SIZE);
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc( const CPelBuf &pSrc, const Size &dstSize )
{
  CHECK( dstSize.width == 0 || dstSize.height == 0, "Empty area provided" );

  int idx, sum = 0;
  Pel dcVal;
  const int width  = dstSize.width;
  const int height = dstSize.height;
  const auto denom     = (width == height) ? (width << 1) : std::max(width,height);
  const auto divShift  = floorLog2(denom);
  const auto divOffset = (denom >> 1);

  if ( width >= height )
  {
    for( idx = 0; idx < width; idx++ )
    {
#if JVET_O0426_MRL_REF_SAMPLES_DC_MODE
      sum += pSrc.at(m_ipaParam.multiRefIndex + 1 + idx, 0);
#else
      sum += pSrc.at( 1 + idx, 0 );
#endif
    }
  }
  if ( width <= height )
  {
    for( idx = 0; idx < height; idx++ )
    {
#if JVET_O0426_MRL_REF_SAMPLES_DC_MODE
      sum += pSrc.at(0, m_ipaParam.multiRefIndex + 1 + idx);
#else
      sum += pSrc.at( 0, 1 + idx );
#endif
    }
  }

  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}

int IntraPrediction::getWideAngle( int width, int height, int predMode )
{
  if ( predMode > DC_IDX && predMode <= VDIA_IDX )
  {
    int modeShift[] = { 0, 6, 10, 12, 14, 15 };
    int deltaSize = abs(floorLog2(width) - floorLog2(height));
    if (width > height && predMode < 2 + modeShift[deltaSize])
    {
      predMode += (VDIA_IDX - 1);
    }
    else if (height > width && predMode > VDIA_IDX - modeShift[deltaSize])
    {
      predMode -= (VDIA_IDX - 1);
    }
  }
  return predMode;
}

void IntraPrediction::setReferenceArrayLengths( const CompArea &area )
{
  // set Top and Left reference samples length
  const int  width    = area.width;
  const int  height   = area.height;

  m_leftRefLength     = (height << 1);
  m_topRefLength      = (width << 1);

}

void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu)
{
  const ComponentID    compID       = MAP_CHROMA( compId );
  const ChannelType    channelType  = toChannelType( compID );
  const int            iWidth       = piPred.width;
  const int            iHeight      = piPred.height;
  const uint32_t       uiDirMode    = isLuma( compId ) && pu.cu->bdpcmMode ? BDPCM_IDX : PU::getFinalIntraMode( pu, channelType );

  CHECK( floorLog2(iWidth) < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed" );
  CHECK( floorLog2(iWidth) > 7, "Size not allowed" );

  const int multiRefIdx = m_ipaParam.multiRefIndex;
#if JVET_O0364_PADDING
  const int srcStride  = m_topRefLength + 1 + multiRefIdx;
  const int srcHStride = m_leftRefLength + 1 + multiRefIdx;
#else
  const int whRatio     = m_ipaParam.whRatio;
  const int hwRatio     = m_ipaParam.hwRatio;

  const int  srcStride  = m_topRefLength  + 1 + (whRatio + 1) * multiRefIdx;
  const int  srcHStride = m_leftRefLength + 1 + (hwRatio + 1) * multiRefIdx;
#endif

#if JVET_O0502_ISP_CLEANUP
  const CPelBuf& srcBuf = pu.cu->ispMode && isLuma(compID) ? getISPBuffer() : CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
#else
  const CPelBuf & srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
#endif
  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(compID));

  switch (uiDirMode)
  {
    case(PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred); break;
    case(DC_IDX):     xPredIntraDc(srcBuf, piPred, channelType, false); break;
    case(BDPCM_IDX):  xPredIntraBDPCM(srcBuf, piPred, pu.cu->bdpcmMode, clpRng); break;
    default:          xPredIntraAng(srcBuf, piPred, channelType, clpRng); break;
  }

  if (m_ipaParam.applyPDPC)
  {
    PelBuf dstBuf = piPred;
    const int scale = ((floorLog2(iWidth) - 2 + floorLog2(iHeight) - 2 + 2) >> 2);
    CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

#if JVET_O0364_PDPC_DC
    if (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX)
#else
    if (uiDirMode == PLANAR_IDX)
#endif
    {
      for (int y = 0; y < iHeight; y++)
      {
        const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
          const Pel top   = srcBuf.at(x + 1, 0);
          const Pel val   = dstBuf.at(x, y);
          dstBuf.at(x, y) = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
        }
      }
    }
#if !JVET_O0364_PDPC_DC
    else if (uiDirMode == DC_IDX)
    {
      const Pel topLeft = srcBuf.at(0, 0);
      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = srcBuf.at(0, y + 1);
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          int wTL = (wL >> 4) + (wT >> 4);
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top - wTL * topLeft + (64 - wL - wT + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
#endif
  }
}

void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir)
{
  int  iLumaStride = 0;
  PelBuf Temp;
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX))
  {
    iLumaStride = 2 * MAX_CU_SIZE + 1;
    Temp = PelBuf(m_pMdlmTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  else
  {
    iLumaStride = MAX_CU_SIZE + 1;
    Temp = PelBuf(m_piTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  int a, b, iShift;
  xGetLMParameters(pu, compID, chromaArea, a, b, iShift);

  ////// final prediction
  piPred.copyFrom(Temp);
  piPred.linearTransform(a, iShift, b, true, pu.cs->slice->clpRng(compID));
}

/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */

//NOTE: Bit-Limit - 24-bit source
void IntraPrediction::xPredIntraPlanar( const CPelBuf &pSrc, PelBuf &pDst )
{
  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;
  const uint32_t log2W  = floorLog2(width  < 2 ? 2 : width);
  const uint32_t log2H  = floorLog2(height < 2 ? 2 : height);

  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const uint32_t offset = 1 << (log2W + log2H);

  // Get left and above reference column and row
  for( int k = 0; k < width + 1; k++ )
  {
    topRow[k] = pSrc.at( k + 1, 0 );
  }

  for( int k = 0; k < height + 1; k++ )
  {
    leftColumn[k] = pSrc.at( 0, k + 1 );
  }

  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight = topRow[width];

  for( int k = 0; k < width; k++ )
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] << log2H;
  }

  for( int k = 0; k < height; k++ )
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] << log2W;
  }

  const uint32_t finalShift = 1 + log2W + log2H;
  const uint32_t stride     = pDst.stride;
  Pel*       pred       = pDst.buf;
  for( int y = 0; y < height; y++, pred += stride )
  {
    int horPred = leftColumn[y];

    for( int x = 0; x < width; x++ )
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      int vertPred = topRow[x];
      pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
    }
  }
}
void IntraPrediction::xPredIntraDc( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter )
{
  const Pel dcval = xGetPredValDc( pSrc, pDst );
  pDst.fill( dcval );
}

// Function for initialization of intra prediction parameters
void IntraPrediction::initPredIntraParams(const PredictionUnit & pu, const CompArea area, const SPS& sps)
{
  const ComponentID compId = area.compID;
  const ChannelType chType = toChannelType(compId);

  const bool        useISP = NOT_INTRA_SUBPARTITIONS != pu.cu->ispMode && isLuma( chType );

  const Size   cuSize    = Size( pu.cu->blocks[compId].width, pu.cu->blocks[compId].height );
  const Size   puSize    = Size( area.width, area.height );
  const Size&  blockSize = useISP ? cuSize : puSize;
  const int      dirMode = PU::getFinalIntraMode(pu, chType);
  const int     predMode = getWideAngle( blockSize.width, blockSize.height, dirMode );

#if !JVET_O0364_PADDING
  m_ipaParam.whRatio              = std::max( unsigned( 1 ), blockSize.width  / blockSize.height ) ;
  m_ipaParam.hwRatio              = std::max( unsigned( 1 ), blockSize.height / blockSize.width  ) ;
#endif
  m_ipaParam.isModeVer            = predMode >= DIA_IDX;
  m_ipaParam.multiRefIndex        = isLuma (chType) ? pu.multiRefIdx : 0 ;
  m_ipaParam.refFilterFlag        = false;
  m_ipaParam.interpolationFlag    = false;
#if JVET_O0502_ISP_CLEANUP
  m_ipaParam.applyPDPC            = ((puSize.width >= MIN_TB_SIZEY && puSize.height >= MIN_TB_SIZEY) || !isLuma(compId)) && m_ipaParam.multiRefIndex == 0;
#else
  m_ipaParam.applyPDPC            = !useISP && m_ipaParam.multiRefIndex == 0;
#endif

  const int    intraPredAngleMode = (m_ipaParam.isModeVer) ? predMode - VER_IDX : -(predMode - HOR_IDX);


  int absAng = 0;
  if (dirMode > DC_IDX && dirMode < NUM_LUMA_MODE) // intraPredAngle for directional modes
  {
    static const int angTable[32]    = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };
#if JVET_O0364_PADDING
    static const int invAngTable[32] = {
      0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565,
      512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16
    };   // (512 * 32) / Angle
#else
    static const int invAngTable[32] = { 0, 8192, 4096, 2731, 2048, 1365,  1024,  819,  683,  585,  512,  455,  410,  356,  315,  282,  256,  234,  210, 182, 161, 144, 128, 112,  95,  80,  64,  48,  32,  24,  16,    8 }; // (256 * 32) / Angle
#endif

    const int     absAngMode         = abs(intraPredAngleMode);
    const int     signAng            = intraPredAngleMode < 0 ? -1 : 1;
                  absAng             = angTable  [absAngMode];

    m_ipaParam.invAngle              = invAngTable[absAngMode];
    m_ipaParam.intraPredAngle        = signAng * absAng;
#if JVET_O0364_PDPC_ANGULAR
    if (intraPredAngleMode < 0)
    {
      m_ipaParam.applyPDPC = false;
    }
    else if (intraPredAngleMode > 0)
    {
      const int sideSize = m_ipaParam.isModeVer ? puSize.height : puSize.width;
      const int maxScale = 2;

#if JVET_O0364_PADDING
      m_ipaParam.angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * m_ipaParam.invAngle - 2) - 8));
#else
      m_ipaParam.angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * m_ipaParam.invAngle - 2) - 7));
#endif
      m_ipaParam.applyPDPC &= m_ipaParam.angularScale >= 0;
    }
#else
    m_ipaParam.applyPDPC            &= m_ipaParam.intraPredAngle == 0 || m_ipaParam.intraPredAngle >= 12; // intra prediction modes: HOR, VER, x, where x>=VDIA-8 or x<=2+8
#endif
  }

  // high level conditions and DC intra prediction
  if(   sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag()
    || !isLuma( chType )
    || useISP
#if JVET_O0925_MIP_SIMPLIFICATIONS
    || PU::isMIP( pu, chType )
#endif
    || m_ipaParam.multiRefIndex
    || DC_IDX == dirMode
    )
  {
#if !JVET_O0502_ISP_CLEANUP
    if (useISP)
    {
      m_ipaParam.interpolationFlag = (m_ipaParam.isModeVer ? puSize.width : puSize.height) > 8 ? true : false ;
    }
#endif
  }
  else if (isLuma( chType ) && pu.cu->bdpcmMode) // BDPCM
  {
    m_ipaParam.refFilterFlag = false;
  }
  else if (dirMode == PLANAR_IDX) // Planar intra prediction
  {
    m_ipaParam.refFilterFlag = puSize.width * puSize.height > 32 ? true : false;
  }
  else if (!useISP)// HOR, VER and angular modes (MDIS)
  {
    bool filterFlag = false;
#if !JVET_O0277_INTRA_SMALL_BLOCK_DCTIF
    if (predMode != dirMode ) // wide-anlge mode
    {
      filterFlag = true;
    }
    else
#endif
    {
#if JVET_O0277_INTRA_SMALL_BLOCK_DCTIF
      const int diff = std::min<int>( abs( predMode - HOR_IDX ), abs( predMode - VER_IDX ) );
#else
      const int diff = std::min<int>( abs( dirMode - HOR_IDX ), abs( dirMode - VER_IDX ) );
#endif
      const int log2Size = ((floorLog2(puSize.width) + floorLog2(puSize.height)) >> 1);
      CHECK( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
      filterFlag = (diff > m_aucIntraFilter[log2Size]);
    }

    // Selelection of either ([1 2 1] / 4 ) refrence filter OR Gaussian 4-tap interpolation filter
    if (filterFlag)
    {
      const bool isRefFilter       =  isIntegerSlope(absAng);
#if JVET_O0277_INTRA_SMALL_BLOCK_DCTIF
      CHECK( puSize.width * puSize.height <= 32, "DCT-IF interpolation filter is always used for 4x4, 4x8, and 8x4 luma CB" );
      m_ipaParam.refFilterFlag     =  isRefFilter;
#else
      m_ipaParam.refFilterFlag = isRefFilter && puSize.width * puSize.height > 32;
#endif
      m_ipaParam.interpolationFlag = !isRefFilter;
    }
  }
}


/** Function for deriving the simplified angular intra predictions.
*
* This function derives the prediction samples for the angular mode based on the prediction direction indicated by
* the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
* the reference row above the block in the case of vertical prediction or displacement of the rightmost column
* of the block and reference column left from the block in the case of the horizontal prediction. The displacement
* is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
* the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
* from the extended main reference.
*/
//NOTE: Bit-Limit - 25-bit source

void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng)
{
  int width =int(pDst.width);
  int height=int(pDst.height);

  const bool bIsModeVer     = m_ipaParam.isModeVer;
#if !JVET_O0364_PADDING
  const int  whRatio        = m_ipaParam.whRatio;
  const int  hwRatio        = m_ipaParam.hwRatio;
#endif
  const int  multiRefIdx    = m_ipaParam.multiRefIndex;
  const int  intraPredAngle = m_ipaParam.intraPredAngle;
  const int  invAngle       = m_ipaParam.invAngle;

  Pel* refMain;
  Pel* refSide;

  Pel  refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel  refLeft [2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
#if JVET_O0364_PADDING
    for (int x = 0; x <= width + 1 + multiRefIdx; x++)
    {
      refAbove[x + height] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= height + 1 + multiRefIdx; y++)
    {
      refLeft[y + width] = pSrc.at(0, y);
    }
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft + width : refAbove + height;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * invAngle + 256) >> 9, sizeSide)];
    }
#else
    const int width    = pDst.width + 1;
    const int height   = pDst.height + 1;
    const int lastIdx  = (bIsModeVer ? width : height) + multiRefIdx;
    const int firstIdx = (((bIsModeVer ? height : width) - 1) * intraPredAngle) >> 5;

    for (int x = 0; x < width + 1 + multiRefIdx; x++)
    {
      refAbove[x + height - 1] = pSrc.at( x, 0 );
    }
    for (int y = 0; y < height + 1 + multiRefIdx; y++)
    {
      refLeft[y + width - 1] = pSrc.at( 0, y );
    }
    refMain = (bIsModeVer ? refAbove + height : refLeft  + width ) - 1;
    refSide = (bIsModeVer ? refLeft  + width  : refAbove + height) - 1;

    // Extend the Main reference to the left.
    int invAngleSum    = 128;       // rounding for (shift by 8)
    for( int k = -1; k > firstIdx; k-- )
    {
      invAngleSum += invAngle;
      refMain[k] = refSide[invAngleSum>>8];
    }
    refMain[lastIdx] = refMain[lastIdx-1];
    refMain[firstIdx] = refMain[firstIdx+1];
#endif
  }
  else
  {
#if JVET_O0364_PADDING
    for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(0, y);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (multiRefIdx << s) + 2;
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
#else
    for (int x = 0; x < m_topRefLength + 1 + (whRatio + 1) * multiRefIdx; x++)
    {
      refAbove[x+1] = pSrc.at(x, 0);
    }
    for (int y = 0; y < m_leftRefLength + 1 + (hwRatio + 1) * multiRefIdx; y++)
    {
      refLeft[y+1]  = pSrc.at(0, y);
    }
    refMain = bIsModeVer ? refAbove : refLeft ;
    refSide = bIsModeVer ? refLeft  : refAbove;

    refMain++;
    refSide++;
    refMain[-1] = refMain[0];
    auto lastIdx = 1 + ((bIsModeVer) ? m_topRefLength + (whRatio + 1) * multiRefIdx : m_leftRefLength +  (hwRatio + 1) * multiRefIdx);
    refMain[lastIdx] = refMain[lastIdx-1];
#endif
  }

  // swap width/height if we are doing a horizontal mode:
  Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
  const int dstStride = bIsModeVer ? pDst.stride : MAX_CU_SIZE;
  Pel *pDstBuf = bIsModeVer ? pDst.buf : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }

  // compensate for line offset in reference line buffers
  refMain += multiRefIdx;
  refSide += multiRefIdx;

  Pel *pDsty = pDstBuf;

  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        pDsty[x] = refMain[x + 1];
      }

      if (m_ipaParam.applyPDPC)
      {
        const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        const Pel topLeft = refMain[0];
        const Pel left    = refSide[1 + y];
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          const int wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }

      pDsty += dstStride;
    }
  }
  else
  {
    for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y<height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;

      if ( !isIntegerSlope( abs(intraPredAngle) ) )
      {
        if( isLuma(channelType) )
        {
          const bool useCubicFilter = !m_ipaParam.interpolationFlag;

          const TFilterCoeff *const f =
            (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : g_intraGaussFilter[deltaFract];

          for (int x = 0; x < width; x++)
          {
            Pel p[4];

            p[0] = refMain[deltaInt + x];
            p[1] = refMain[deltaInt + x + 1];
            p[2] = refMain[deltaInt + x + 2];
#if JVET_O0364_PADDING
            p[3] = refMain[deltaInt + x + 3];
#else
            p[3] = f[3] != 0 ? refMain[deltaInt + x + 3] : 0;
#endif

            Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;

            pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
          }
        }
        else
        {
          // Do linear filtering
          for (int x = 0; x < width; x++)
          {
            Pel p[2];

            p[0] = refMain[deltaInt + x + 1];
            p[1] = refMain[deltaInt + x + 2];

            pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
          }
        }
      }
      else
      {
        // Just copy the integer samples
        for( int x = 0; x < width; x++ )
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
#if JVET_O0364_PDPC_ANGULAR
      if (m_ipaParam.applyPDPC)
      {
        const int scale       = m_ipaParam.angularScale;
#if JVET_O0364_PADDING
        int       invAngleSum = 256;
#else
        int       invAngleSum = 128;
#endif

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          invAngleSum += invAngle;

          int wL   = 32 >> (2 * x >> scale);
#if JVET_O0364_PADDING
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
#else
          Pel left = refSide[y + (invAngleSum >> 8) + 1];
#endif
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }
#else
      const int scale = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
      CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");
      if (m_ipaParam.applyPDPC)
      {
        if (m_ipaParam.intraPredAngle == 32) // intra prediction modes: 2 and VDIA
        {
          int wT = 16 >> std::min(31, ((y << 1) >> scale));

          for (int x = 0; x < width; x++)
          {
            int wL = 16 >> std::min(31, ((x << 1) >> scale));
            if (wT + wL == 0) break;

            int c = x + y + 1;
            if (c >= 2 * height) { wL = 0; }
            if (c >= 2 * width)  { wT = 0; }
            const Pel left = (wL != 0) ? refSide[c + 1] : 0;
            const Pel top  = (wT != 0) ? refMain[c + 1] : 0;

            pDsty[x] = ClipPel((wL * left + wT * top + (64 - wL - wT) * pDsty[x] + 32) >> 6, clpRng);
          }
        }
        else
        {
#if JVET_O0364_PADDING
          int invAngleSum0 = 4;
#else
          int invAngleSum0 = 2;
#endif
          for (int x = 0; x < width; x++)
          {
            invAngleSum0 += invAngle;
#if JVET_O0364_PADDING
            int deltaPos0 = invAngleSum0 >> 3;
#else
            int deltaPos0 = invAngleSum0 >> 2;
#endif
            int deltaFrac0 = deltaPos0 & 63;
            int deltaInt0 = deltaPos0 >> 6;

            int deltay = y + deltaInt0 + 1;
            if (deltay >(bIsModeVer ? m_leftRefLength : m_topRefLength) - 1) break;

            int wL = 32 >> std::min(31, ((x << 1) >> scale));
            if (wL == 0) break;
            Pel *p = refSide + deltay;

            Pel left = p[deltaFrac0 >> 5];
            pDsty[x] = ClipPel((wL * left + (64 - wL) * pDsty[x] + 32) >> 6, clpRng);
          }
        }
      }
#endif
    }
  }

  // Flip the block if this is the horizontal mode
  if( !bIsModeVer )
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        pDst.at( y, x ) = pDstBuf[x];
      }
      pDstBuf += dstStride;
    }
  }
}

void IntraPrediction::xPredIntraBDPCM(const CPelBuf &pSrc, PelBuf &pDst, const uint32_t dirMode, const ClpRng& clpRng )
{
  const int wdt = pDst.width;
  const int hgt = pDst.height;

  const int strideP = pDst.stride;
  const int strideS = pSrc.stride;

  CHECK( !( dirMode == 1 || dirMode == 2 ), "Incorrect BDPCM mode parameter." );

  Pel* pred = &pDst.buf[0];
  if( dirMode == 1 )
  {
    Pel  val;
    for( int y = 0; y < hgt; y++ )
    {
      val = pSrc.buf[(y + 1) * strideS];
      for( int x = 0; x < wdt; x++ )
      {
        pred[x] = val;
      }
      pred += strideP;
    }
  }
  else
  {
    for( int y = 0; y < hgt; y++ )
    {
      for( int x = 0; x < wdt; x++ )
      {
        pred[x] = pSrc.buf[x + 1];
      }
      pred += strideP;
    }
  }
}

bool IntraPrediction::useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const uint32_t &uiDirMode)
{
  return CU::isRDPCMEnabled(*pu.cu) && pu.cu->transQuantBypass && (uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
}

void IntraPrediction::geneWeightedPred(const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf)
{
  const int            width = pred.width;
  const int            height = pred.height;
  const int            srcStride = width;
  const int            dstStride = pred.stride;

  Pel*                 dstBuf = pred.buf;
  int wIntra, wMerge;

  const Position posBL = pu.Y().bottomLeft();
  const Position posTR = pu.Y().topRight();
  const PredictionUnit *neigh0 = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CHANNEL_TYPE_LUMA);
  const PredictionUnit *neigh1 = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CHANNEL_TYPE_LUMA);
  bool isNeigh0Intra = neigh0 && (CU::isIntra(*neigh0->cu));
  bool isNeigh1Intra = neigh1 && (CU::isIntra(*neigh1->cu));

  if (isNeigh0Intra && isNeigh1Intra)
  {
    wIntra = 3; wMerge = 1;
  }
  else
  {
    if (!isNeigh0Intra && !isNeigh1Intra)
    {
      wIntra = 1; wMerge = 3;
    }
    else
    {
      wIntra = 2; wMerge = 2;
    }
  }
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      dstBuf[y*dstStride + x] = (wMerge * dstBuf[y*dstStride + x] + wIntra * srcBuf[y*srcStride + x] + 2) >> 2;
    }
  }
}
void IntraPrediction::switchBuffer(const PredictionUnit &pu, ComponentID compID, PelBuf srcBuff, Pel *dst)
{
  Pel  *src = srcBuff.bufAt(0, 0);
  int compWidth = compID == COMPONENT_Y ? pu.Y().width : pu.Cb().width;
  int compHeight = compID == COMPONENT_Y ? pu.Y().height : pu.Cb().height;
  for (int i = 0; i < compHeight; i++)
  {
    memcpy(dst, src, compWidth * sizeof(Pel));
    src += srcBuff.stride;
    dst += compWidth;
  }
}

void IntraPrediction::geneIntrainterPred(const CodingUnit &cu)
{
  if (!cu.firstPU->mhIntraFlag)
  {
    return;
  }

  const PredictionUnit* pu = cu.firstPU;

  initIntraPatternChType(cu, pu->Y());
  predIntraAng(COMPONENT_Y, cu.cs->getPredBuf(*pu).Y(), *pu);

  initIntraPatternChType(cu, pu->Cb());
  predIntraAng(COMPONENT_Cb, cu.cs->getPredBuf(*pu).Cb(), *pu);

  initIntraPatternChType(cu, pu->Cr());
  predIntraAng(COMPONENT_Cr, cu.cs->getPredBuf(*pu).Cr(), *pu);

  for (int currCompID = 0; currCompID < 3; currCompID++)
  {
    ComponentID currCompID2 = (ComponentID)currCompID;
    PelBuf tmpBuf = currCompID == 0 ? cu.cs->getPredBuf(*pu).Y() : (currCompID == 1 ? cu.cs->getPredBuf(*pu).Cb() : cu.cs->getPredBuf(*pu).Cr());
    switchBuffer(*pu, currCompID2, tmpBuf, getPredictorPtr2(currCompID2, 0));
  }
}

inline bool isAboveLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT );
inline int  isAboveAvailable      ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags );
inline int  isLeftAvailable       ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags );
inline int  isAboveRightAvailable ( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );
inline int  isBelowLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );

void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag)
{
  const CodingStructure& cs   = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
  }

  Pel *refBufUnfiltered   = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered     = m_piYuvExt[area.compID][PRED_BUF_FILTERED];

#if JVET_O0502_ISP_CLEANUP
  setReferenceArrayLengths( area );
#else
  setReferenceArrayLengths( cu.ispMode && isLuma( area.compID ) ? cu.blocks[area.compID] : area );
#endif

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
  // ----- Step 2: filtered reference samples -----
  if( m_ipaParam.refFilterFlag || forceRefFilterFlag )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx );
  }
}

#if JVET_O0502_ISP_CLEANUP
void IntraPrediction::initIntraPatternChTypeISP(const CodingUnit& cu, const CompArea& area, PelBuf& recBuf, const bool forceRefFilterFlag)
{
  const CodingStructure& cs = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
  }

  const Position posLT = area;
  bool           isLeftAvail = cs.isDecomp(posLT.offset(-1, 0), CHANNEL_TYPE_LUMA);
  bool           isAboveAvail = cs.isDecomp(posLT.offset(0, -1), CHANNEL_TYPE_LUMA);
  // ----- Step 1: unfiltered reference samples -----
#if JVET_O0106_ISP_4xN_PREDREG_FOR_1xN_2xN
  if (cu.blocks[area.compID].x == area.x && cu.blocks[area.compID].y == area.y)
#else
  if (CU::isISPFirst(cu, area, area.compID))
#endif
  {
    Pel* refBufUnfiltered = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
    // With the first subpartition all the CU reference samples are fetched at once in a single call to xFillReferenceSamples
    if (cu.ispMode == HOR_INTRA_SUBPARTITIONS)
    {
      m_leftRefLength = cu.Y().height << 1;
      m_topRefLength = cu.Y().width + area.width;
    }
    else //if (cu.ispMode == VER_INTRA_SUBPARTITIONS)
    {
      m_leftRefLength = cu.Y().height + area.height;
      m_topRefLength = cu.Y().width << 1;
    }

    const int srcStride = m_topRefLength + 1;
    const int srcHStride = m_leftRefLength + 1;

    m_pelBufISP[0] = m_pelBufISPBase[0] = PelBuf(m_piYuvExt[area.compID][PRED_BUF_UNFILTERED], srcStride, srcHStride);
    m_pelBufISP[1] = m_pelBufISPBase[1] = PelBuf(m_piYuvExt[area.compID][PRED_BUF_FILTERED], srcStride, srcHStride);

    xFillReferenceSamples(cs.picture->getRecoBuf(cu.Y()), refBufUnfiltered, cu.Y(), cu);

    // After having retrieved all the CU reference samples, the number of reference samples is now adjusted for the current subpartition
    m_topRefLength = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;
  }
  else
  {
    //Now we only need to fetch the newly available reconstructed samples from the previously coded TU
    Position tuPos = area;
    tuPos.relativeTo(cu.Y());
    m_pelBufISP[0] = m_pelBufISPBase[0].subBuf(tuPos, area.size());
    m_pelBufISP[1] = m_pelBufISPBase[1].subBuf(tuPos, area.size());

    PelBuf& dstBuf = m_pelBufISP[0];

    m_topRefLength = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;

    const int predSizeHor = m_topRefLength;
    const int predSizeVer = m_leftRefLength;
    if (cu.ispMode == HOR_INTRA_SUBPARTITIONS)
    {
      Pel* src = recBuf.bufAt(0, -1);
      Pel* dst = dstBuf.bufAt(1, 0);
      for (int i = 0; i < area.width; i++)
      {
        dst[i] = src[i];
      }
      Pel sample = src[area.width - 1];
      dst += area.width;
      for (int i = 0; i < predSizeHor - area.width; i++)
      {
        dst[i] = sample;
      }
      if (!isLeftAvail) //if left is not avaible, then it is necessary to fetch these samples for each subpartition
      {
        Pel* dst = dstBuf.bufAt(0, 0);
        Pel  sample = src[0];
        for (int i = 0; i < predSizeVer + 1; i++)
        {
          *dst = sample;
          dst += dstBuf.stride;
        }
      }
    }
    else
    {
      Pel* src = recBuf.bufAt(-1, 0);
      Pel* dst = dstBuf.bufAt(0, 1);
      for (int i = 0; i < area.height; i++)
      {
        *dst = *src;
        src += recBuf.stride;
        dst += dstBuf.stride;
      }
      Pel sample = src[-recBuf.stride];
      for (int i = 0; i < predSizeVer - area.height; i++)
      {
        *dst = sample;
        dst += dstBuf.stride;
      }

      if (!isAboveAvail) //if above is not avaible, then it is necessary to fetch these samples for each subpartition
      {
        Pel* dst = dstBuf.bufAt(0, 0);
        Pel  sample = recBuf.at(-1, 0);
        for (int i = 0; i < predSizeHor + 1; i++)
        {
          dst[i] = sample;
        }
      }
    }
  }
  // ----- Step 2: filtered reference samples -----
  if (m_ipaParam.refFilterFlag || forceRefFilterFlag)
  {
    Pel* refBufUnfiltered = m_pelBufISP[0].buf;
    Pel* refBufFiltered = m_pelBufISP[1].buf;
    xFilterReferenceSamples(refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx, m_pelBufISP[0].stride);
  }
}
#endif


void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu )
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int multiRefIdx         = (area.compID == COMPONENT_Y) ? cu.firstPU->multiRefIdx : 0;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  const int  predSize           = m_topRefLength;
  const int  predHSize          = m_leftRefLength;
#if JVET_O0364_PADDING
  const int predStride = predSize + 1 + multiRefIdx;
#else
  const int  cuWidth            = cu.blocks[area.compID].width;
  const int  cuHeight           = cu.blocks[area.compID].height;
  const int  whRatio            = cu.ispMode && isLuma(area.compID) ? std::max(1, cuWidth / cuHeight) : std::max(1, tuWidth / tuHeight);
  const int  hwRatio            = cu.ispMode && isLuma(area.compID) ? std::max(1, cuHeight / cuWidth) : std::max(1, tuHeight / tuWidth);
  const int  predStride         = predSize + 1 + (whRatio + 1) * multiRefIdx;
#endif

  const bool noShift            = pcv.noChroma2x2 && area.width == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth          = tuWidth  <= 2 && cu.ispMode && isLuma(area.compID) ? tuWidth  : pcv.minCUWidth  >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const int  unitHeight         = tuHeight <= 2 && cu.ispMode && isLuma(area.compID) ? tuHeight : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area;
  const Position posRT          = area.topRight();
  const Position posLB          = area.bottomLeft();

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int   numIntraNeighbor = 0;

  memset( neighborFlags, 0, totalUnits );

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable( cu, chType, posLT );
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT, numAboveUnits,      unitWidth,  (neighborFlags + totalLeftUnits + 1) );
  numIntraNeighbor += isAboveRightAvailable( cu, chType, posRT, numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + numAboveUnits) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT, numLeftUnits,       unitHeight, (neighborFlags + totalLeftUnits - 1) );
  numIntraNeighbor += isBelowLeftAvailable ( cu, chType, posLB, numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );

  // ----- Step 2: fill reference samples (depending on neighborhood) -----
  CHECK((predHSize + 1) * predStride > m_iYuvExtSize, "Reference sample area not supported");

  const Pel*  srcBuf    = recoBuf.buf;
  const int   srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);


  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize + multiRefIdx; j++) { ptrDst[j] = valueDC; }
    for (int i = 1; i <= predHSize + multiRefIdx; i++) { ptrDst[i*predStride] = valueDC; }
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    for (int j = 0; j <= predSize + multiRefIdx; j++) { ptrDst[j] = ptrSrc[j]; }
    ptrSrc = srcBuf - multiRefIdx * srcStride - (1 + multiRefIdx);
    for (int i = 1; i <= predHSize + multiRefIdx; i++) { ptrDst[i*predStride] = *(ptrSrc); ptrSrc += srcStride; }
  }
  else // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits])
    {
      ptrDst[0] = ptrSrc[0];
      for (int i = 1; i <= multiRefIdx; i++)
      {
        ptrDst[i] = ptrSrc[i];
        ptrDst[i*predStride] = ptrSrc[i*srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + multiRefIdx) * srcStride;
    ptrDst += (1 + multiRefIdx) * predStride;
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i*predStride] = ptrSrc[i*srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight * predStride;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = (predHSize % unitHeight == 0) ? unitHeight : predHSize % unitHeight;
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i*predStride] = ptrSrc[i*srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
    ptrDst = refBufUnfiltered + 1 + multiRefIdx;
    for (int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = (predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth;
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = 0;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + multiRefIdx;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = multiRefIdx;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - 1) * unitWidth + 1 + multiRefIdx;
      }
      const Pel firstAvailSample = ptrDst[firstAvailCol + firstAvailRow * predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize + multiRefIdx;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i*predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit]) // samples not available
      {
        // last available sample
        int lastAvailRow = 0;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + multiRefIdx + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = multiRefIdx;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits) * unitWidth + multiRefIdx;
        }
        const Pel lastAvailSample = ptrDst[lastAvailCol + lastAvailRow * predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i*predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 1; i < multiRefIdx + 1; i++)
          {
            ptrDst[i*predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit = (currUnit == totalUnits - 1) ? ((predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
#if !JVET_O0364_PADDING
  // padding of extended samples above right with the last sample
  int lastSample = multiRefIdx + predSize;
  for (int j = 1; j <= whRatio * multiRefIdx; j++) { ptrDst[lastSample + j] = ptrDst[lastSample]; }
  // padding of extended samples below left with the last sample
  lastSample = multiRefIdx + predHSize;
  for (int i = 1; i <= hwRatio * multiRefIdx; i++) { ptrDst[(lastSample + i)*predStride] = ptrDst[lastSample*predStride]; }
#endif
}

void IntraPrediction::xFilterReferenceSamples( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps
  , int multiRefIdx
#if JVET_O0502_ISP_CLEANUP
  , int stride
#endif
)
{
  if (area.compID != COMPONENT_Y)
  {
    multiRefIdx = 0;
  }
#if JVET_O0364_PADDING
  const int predSize = m_topRefLength + multiRefIdx;
  const int predHSize = m_leftRefLength + multiRefIdx;
#else
  int whRatio          = std::max(1, int(area.width  / area.height));
  int hwRatio          = std::max(1, int(area.height / area.width));
  const int  predSize  = m_topRefLength  + (whRatio + 1) * multiRefIdx;
  const int  predHSize = m_leftRefLength + (hwRatio + 1) * multiRefIdx;
#endif
#if JVET_O0502_ISP_CLEANUP
  const int predStride = stride == 0 ? predSize + 1 : stride;
#else
  const int  predStride = predSize + 1;
#endif



  // Regular reference sample filter
  const Pel *piSrcPtr  = refBufUnfiltered + (predStride * predHSize); // bottom left
        Pel *piDestPtr = refBufFiltered   + (predStride * predHSize); // bottom left

  // bottom left (not filtered)
  *piDestPtr = *piSrcPtr;
  piDestPtr -= predStride;
  piSrcPtr  -= predStride;
  //left column (bottom to top)
  for( int i = 1; i < predHSize; i++, piDestPtr -= predStride, piSrcPtr -= predStride)
  {
    *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[-predStride] + 2) >> 2;
  }
  //top-left
  *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[1] + 2) >> 2;
  piDestPtr++;
  piSrcPtr++;
  //top row (left-to-right)
  for( uint32_t i=1; i < predSize; i++, piDestPtr++, piSrcPtr++ )
  {
    *piDestPtr = (piSrcPtr[1] + 2 * piSrcPtr[0] + piSrcPtr[-1] + 2) >> 2;
  }
  // top right (not filtered)
  *piDestPtr=*piSrcPtr;
}

bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure& cs = *cu.cs;
  const Position refPos = posLT.offset(-1, -1);
  const CodingUnit* pcCUAboveLeft = cs.isDecomp( refPos, chType ) ? cs.getCURestricted( refPos, cu, chType ) : nullptr;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool bAboveLeftFlag;

  if (isConstrained)
  {
    bAboveLeftFlag = pcCUAboveLeft && CU::isIntra(*pcCUAboveLeft);
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }

  return bAboveLeftFlag;
}

int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;
  int maxDx = uiNumUnitsInPU * unitWidth;

  for (uint32_t dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }
  return iNumIntra;
}

int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;
  int maxDy = uiNumUnitsInPU * unitHeight;

  for (uint32_t dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;

  uint32_t maxDx = uiNumUnitsInPU * unitWidth;

  for (uint32_t dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }

  return iNumIntra;
}

int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const bool isConstrained = cs.pps->getConstrainedIntraPred();
  bool *pbValidFlags = bValidFlags;
  int iNumIntra = 0;
  int maxDy = uiNumUnitsInPU * unitHeight;

  for (uint32_t dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-1, unitHeight + dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if ( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

// LumaRecPixels
void IntraPrediction::xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea)
{
  int iDstStride = 0;
  Pel* pDst0 = 0;
  int curChromaMode = pu.intraDir[1];
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    iDstStride = 2 * MAX_CU_SIZE + 1;
    pDst0 = m_pMdlmTemp + iDstStride + 1;
  }
  else
  {
    iDstStride = MAX_CU_SIZE + 1;
    pDst0 = m_piTemp + iDstStride + 1; //MMLM_SAMPLE_NEIGHBOR_LINES;
  }
  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, pu.chromaFormat, chromaArea.lumaPos(), recalcSize( pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)

  CHECK(lumaArea.width == chromaArea.width && CHROMA_444 != pu.chromaFormat, "");
  CHECK(lumaArea.height == chromaArea.height && CHROMA_444 != pu.chromaFormat && CHROMA_422 != pu.chromaFormat, "");

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CPelBuf Src = pu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  int iRecStride        = Src.stride;
  int logSubWidthC  = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
  int logSubHeightC = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.chromaFormat);

  int iRecStride2       = iRecStride << logSubHeightC;
  const int mult        =          1 << logSubWidthC ;

  const CodingUnit& lumaCU = isChroma( pu.chType ) ? *pu.cs->picture->cs->getCU( lumaArea.pos(), CH_L ) : *pu.cu;
  const CodingUnit&     cu = *pu.cu;

  const CompArea& area = isChroma( pu.chType ) ? chromaArea : lumaArea;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  const int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const int  iUnitHeight = iBaseUnitSize >> getComponentScaleY(area.compID, area.chromaFormat);

  const int  iTUWidthInUnits = uiTuWidth / iUnitWidth;
  const int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const int  iAboveUnits      = iTUWidthInUnits;
  const int  iLeftUnits       = iTUHeightInUnits;
  const int  chromaUnitWidth = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, area.chromaFormat);
  const int  chromaUnitHeight = iBaseUnitSize >> getComponentScaleY(COMPONENT_Cb, area.chromaFormat);
  const int  topTemplateSampNum = 2 * uiCWidth; // for MDLM, the number of template samples is 2W or 2H.
  const int  leftTemplateSampNum = 2 * uiCHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  const int  totalAboveUnits = (topTemplateSampNum + (chromaUnitWidth - 1)) / chromaUnitWidth;
  const int  totalLeftUnits = (leftTemplateSampNum + (chromaUnitHeight - 1)) / chromaUnitHeight;
  const int  totalUnits = totalLeftUnits + totalAboveUnits + 1;
  const int  aboveRightUnits = totalAboveUnits - iAboveUnits;
  const int  leftBelowUnits = totalLeftUnits - iLeftUnits;

  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(bNeighborFlags, 0, totalUnits);
  bool bAboveAvaillable, bLeftAvaillable;

  int availlableUnit = isLeftAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iLeftUnits, iUnitHeight,
  ( bNeighborFlags + iLeftUnits + leftBelowUnits - 1 ) );

  bLeftAvaillable = availlableUnit == iTUHeightInUnits;

  availlableUnit = isAboveAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iAboveUnits, iUnitWidth,
  ( bNeighborFlags + iLeftUnits + leftBelowUnits + 1 ) );

  bAboveAvaillable = availlableUnit == iTUWidthInUnits;

  if (bLeftAvaillable) // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.bottomLeftComp(area.compID), leftBelowUnits, iUnitHeight, (bNeighborFlags + leftBelowUnits - 1));
  }

  if (bAboveAvaillable) // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.topRightComp(area.compID), aboveRightUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1));
  }

  Pel*       pDst  = nullptr;
  Pel const* piSrc = nullptr;

  bool isFirstRowOfCtu = ((pu.block(COMPONENT_Cb).y)&(((pu.cs->sps)->getMaxCUWidth() >> 1) - 1)) == 0;
  const int strOffset = (CHROMA_444 == pu.chromaFormat) ? 0 : iRecStride;

  int c0_2tap = 1, c1_2tap = 1,                                                     offset_2tap = 1, shift_2tap = 1; //sum = 2
  int c0_3tap = 2, c1_3tap = 1, c2_3tap = 1,                                        offset_3tap = 2, shift_3tap = 2; //sum = 4
  int c0_5tap = 1, c1_5tap = 4, c2_5tap = 1, c3_5tap = 1, c4_5tap = 1,              offset_5tap = 4, shift_5tap = 3; //sum = 8
  int c0_6tap = 2, c1_6tap = 1, c2_6tap = 1, c3_6tap = 2, c4_6tap = 1, c5_6tap = 1, offset_6tap = 4, shift_6tap = 3; //sum = 8

  switch (pu.chromaFormat)
  {
    case CHROMA_422: //overwrite filter coefficient values for 422
      c0_2tap = 1, c1_2tap = 0,                                                     offset_2tap = 0, shift_2tap = 0; //sum = 1
      c0_3tap = 2, c1_3tap = 1, c2_3tap = 1,                                        offset_3tap = 2, shift_3tap = 2; //sum = 4
      c0_5tap = 0, c1_5tap = 1, c2_5tap = 0, c3_5tap = 0, c4_5tap = 0,              offset_5tap = 0, shift_5tap = 0; //sum = 1
      c0_6tap = 2, c1_6tap = 1, c2_6tap = 1, c3_6tap = 0, c4_6tap = 0, c5_6tap = 0, offset_6tap = 2, shift_6tap = 2; //sum = 4
      break;

    case CHROMA_444:  //overwrite filter coefficient values for 422
      c0_2tap = 1, c1_2tap = 0,                                                     offset_2tap = 0, shift_2tap = 0; //sum = 1
      c0_3tap = 1, c1_3tap = 0, c2_3tap = 0,                                        offset_3tap = 0, shift_3tap = 0; //sum = 1
      c0_5tap = 0, c1_5tap = 1, c2_5tap = 0, c3_5tap = 0, c4_5tap = 0,              offset_5tap = 0, shift_5tap = 0; //sum = 1
      c0_6tap = 1, c1_6tap = 0, c2_6tap = 0, c3_6tap = 0, c4_6tap = 0, c5_6tap = 0, offset_6tap = 0, shift_6tap = 0; //sum = 1
      break;

    default:
      break;
  }

  if( bAboveAvaillable )
  {
    pDst  = pDst0    - iDstStride;
    int addedAboveRight = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedAboveRight = avaiAboveRightUnits*chromaUnitWidth;
    }
    for (int i = 0; i < uiCWidth + addedAboveRight; i++)
    {
      if (isFirstRowOfCtu)
      {
        piSrc = pRecSrc0 - iRecStride;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + addedAboveRight - 1 + logSubWidthC))
        {
          pDst[i] = piSrc[mult * i];
        }
        else
        {
          pDst[i] = (piSrc[mult * i] * c0_3tap + piSrc[mult * i - 1] * c1_3tap + piSrc[mult * i + 1] * c2_3tap + offset_3tap) >> shift_3tap;
        }
      }
      else if( pu.cs->sps->getCclmCollocatedChromaFlag() )
      {
        piSrc = pRecSrc0 - iRecStride2;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + addedAboveRight - 1 + logSubWidthC))
        {
          pDst[i] = (piSrc[mult * i] * c0_3tap + piSrc[mult * i - strOffset] * c1_3tap + piSrc[mult * i + strOffset] * c2_3tap + offset_3tap) >> shift_3tap;
        }
        else
        {
          pDst[i] = (piSrc[mult * i - strOffset] * c0_5tap
                  +  piSrc[mult * i]             * c1_5tap + piSrc[mult * i - 1] * c2_5tap + piSrc[mult * i + 1] * c3_5tap
                  +  piSrc[mult * i + strOffset] * c4_5tap
                  +  offset_5tap) >> shift_5tap;
        }
      }
      else
      {
        piSrc = pRecSrc0 - iRecStride2;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + addedAboveRight - 1 + logSubWidthC))
        {
          pDst[i] = (piSrc[mult * i] * c0_2tap + piSrc[mult * i + strOffset] * c1_2tap + offset_2tap) >> shift_2tap;
        }
        else
        {
          pDst[i] = ((piSrc[mult * i]            * c0_6tap + piSrc[mult * i - 1]             * c1_6tap + piSrc[mult * i + 1]             * c2_6tap)
                  + (piSrc[mult * i + strOffset] * c3_6tap + piSrc[mult * i - 1 + strOffset] * c4_6tap + piSrc[mult * i + 1 + strOffset] * c5_6tap)
                  + offset_6tap) >> shift_6tap;
        }
      }
    }
  }

  if( bLeftAvaillable )
  {
    pDst  = pDst0    - 1;

    piSrc = pRecSrc0 - 2 - logSubWidthC;

    int addedLeftBelow = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedLeftBelow = avaiLeftBelowUnits*chromaUnitHeight;
    }

    for (int j = 0; j < uiCHeight + addedLeftBelow; j++)
    {
      if( pu.cs->sps->getCclmCollocatedChromaFlag() )
      {
        if ((j == 0 && !bAboveAvaillable) || (j == uiCHeight + addedLeftBelow - 1 + logSubWidthC))
        {
          pDst[0] = ( piSrc[1] * c0_3tap + piSrc[0] * c1_3tap + piSrc[2] * c2_3tap + offset_3tap) >> shift_3tap;
        }
        else
        {
          pDst[0] = ( piSrc[1 - strOffset] * c0_5tap
                    + piSrc[1            ] * c1_5tap + piSrc[0] * c2_5tap + piSrc[2] * c3_5tap
                    + piSrc[1 + strOffset] * c4_5tap
                    + offset_5tap ) >> shift_5tap;
        }
      }
      else
      {
        pDst[0] = ((piSrc[1]             * c0_6tap + piSrc[0]         * c1_6tap + piSrc[2]             * c2_6tap)
                +  (piSrc[1 + strOffset] * c3_6tap + piSrc[strOffset] * c4_6tap + piSrc[2 + strOffset] * c5_6tap)
                +   offset_6tap) >> shift_6tap;
      }

      piSrc += iRecStride2;
      pDst  += iDstStride;
    }
  }

  // inner part from reconstructed picture buffer
  for( int j = 0; j < uiCHeight; j++ )
  {
    for( int i = 0; i < uiCWidth; i++ )
    {
      if( pu.cs->sps->getCclmCollocatedChromaFlag() )
      {
        if( i == 0 && !bLeftAvaillable )
        {
          if ( j == 0 && !bAboveAvaillable )
          {
            pDst0[i] = pRecSrc0[mult * i];
          }
          else
          {
            pDst0[i] = (pRecSrc0[mult * i] * c0_3tap + pRecSrc0[mult * i - strOffset] * c1_3tap + pRecSrc0[mult * i + strOffset] * c2_3tap + offset_3tap) >> shift_3tap;
          }
        }
        else if ( j == 0 && !bAboveAvaillable )
        {
          pDst0[i] = (pRecSrc0[mult * i] * c0_3tap + pRecSrc0[mult * i - 1] * c1_3tap + pRecSrc0[mult * i + 1] * c2_3tap + offset_3tap) >> shift_3tap;
        }
        else
        {
          pDst0[i] = (pRecSrc0[mult * i - strOffset] * c0_5tap
                   +  pRecSrc0[mult * i]             * c1_5tap + pRecSrc0[mult * i - 1] * c2_5tap + pRecSrc0[mult * i + 1] * c3_5tap
                   +  pRecSrc0[mult * i + strOffset] * c4_5tap
                   +  offset_5tap) >> shift_5tap;
        }
      }
      else
      {

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth - 1 + logSubWidthC))
        {
          pDst0[i] = (pRecSrc0[mult * i] * c0_2tap + pRecSrc0[mult * i + strOffset] * c1_2tap + offset_2tap) >> shift_2tap;
        }
        else
        {
          pDst0[i] = (pRecSrc0[mult * i]             * c0_6tap + pRecSrc0[mult * i + 1]             * c1_6tap + pRecSrc0[mult * i - 1]             * c2_6tap
                    + pRecSrc0[mult * i + strOffset] * c3_6tap + pRecSrc0[mult * i + 1 + strOffset] * c4_6tap + pRecSrc0[mult * i - 1 + strOffset] * c5_6tap
                    + offset_6tap) >> shift_6tap;
        }
      }
    }

    pDst0    += iDstStride;
    pRecSrc0 += iRecStride2;
  }
}
void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID,
                                              const CompArea &chromaArea,
                                              int &a, int &b, int &iShift)
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure & cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const SPS &        sps           = *cs.sps;
  const uint32_t     tuWidth     = chromaArea.width;
  const uint32_t     tuHeight    = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int baseUnitSize = 1 << MIN_CU_LOG2;
  const int unitWidth    = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const int unitHeight   = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);

  const int tuWidthInUnits  = tuWidth / unitWidth;
  const int tuHeightInUnits = tuHeight / unitHeight;
  const int aboveUnits      = tuWidthInUnits;
  const int leftUnits       = tuHeightInUnits;
  int topTemplateSampNum = 2 * cWidth; // for MDLM, the template sample number is 2W or 2H;
  int leftTemplateSampNum = 2 * cHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  int totalAboveUnits = (topTemplateSampNum + (unitWidth - 1)) / unitWidth;
  int totalLeftUnits = (leftTemplateSampNum + (unitHeight - 1)) / unitHeight;
  int totalUnits = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits = totalAboveUnits - aboveUnits;
  int leftBelowUnits = totalLeftUnits - leftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  int avaiAboveUnits = 0;
  int avaiLeftUnits = 0;

  int curChromaMode = pu.intraDir[1];
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  bool aboveAvailable, leftAvailable;

  int availableUnit =
    isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, aboveUnits, unitWidth,
    (neighborFlags + leftUnits + leftBelowUnits + 1));
  aboveAvailable = availableUnit == tuWidthInUnits;

  availableUnit =
    isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, leftUnits, unitHeight,
    (neighborFlags + leftUnits + leftBelowUnits - 1));
  leftAvailable = availableUnit == tuHeightInUnits;
  if (leftAvailable) // if left is not available, then the below left is not available
  {
    avaiLeftUnits = tuHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.bottomLeftComp(chromaArea.compID), leftBelowUnits, unitHeight, (neighborFlags + leftBelowUnits - 1));
  }
  if (aboveAvailable) // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = tuWidthInUnits;
    avaiAboveRightUnits = isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.topRightComp(chromaArea.compID), aboveRightUnits, unitWidth, (neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1));
  }
  Pel *srcColor0, *curChroma0;
  int  srcStride, curStride;

  PelBuf temp;
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp = PelBuf(m_pMdlmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_CU_SIZE + 1;
    temp        = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  srcColor0 = temp.bufAt(0, 0);
  curChroma0 = getPredictorPtr(compID);

  curStride = m_topRefLength + 1;

  curChroma0 += curStride + 1;

  unsigned internalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  int minLuma[2] = {  MAX_INT, 0 };
  int maxLuma[2] = { -MAX_INT, 0 };

  Pel *src = srcColor0 - srcStride;
  Pel *cur = curChroma0 - curStride;
  int actualTopTemplateSampNum = 0;
  int actualLeftTemplateSampNum = 0;
  if (curChromaMode == MDLM_T_IDX)
  {
    leftAvailable = 0;
    avaiAboveRightUnits = avaiAboveRightUnits > (cHeight/unitWidth) ?  cHeight/unitWidth : avaiAboveRightUnits;
    actualTopTemplateSampNum = unitWidth*(avaiAboveUnits + avaiAboveRightUnits);
  }
  else if (curChromaMode == MDLM_L_IDX)
  {
    aboveAvailable = 0;
    avaiLeftBelowUnits = avaiLeftBelowUnits > (cWidth/unitHeight) ? cWidth/unitHeight : avaiLeftBelowUnits;
    actualLeftTemplateSampNum = unitHeight*(avaiLeftUnits + avaiLeftBelowUnits);
  }
  else if (curChromaMode == LM_CHROMA_IDX)
  {
    actualTopTemplateSampNum = cWidth;
    actualLeftTemplateSampNum = cHeight;
  }
  int startPos[2]; //0:Above, 1: Left
  int pickStep[2];

  int aboveIs4 = leftAvailable  ? 0 : 1;
  int leftIs4 =  aboveAvailable ? 0 : 1;

  startPos[0] = actualTopTemplateSampNum >> (2 + aboveIs4);
  pickStep[0] = std::max(1, actualTopTemplateSampNum >> (1 + aboveIs4));

  startPos[1] = actualLeftTemplateSampNum >> (2 + leftIs4);
  pickStep[1] = std::max(1, actualLeftTemplateSampNum >> (1 + leftIs4));

  Pel selectLumaPix[4] = { 0, 0, 0, 0 };
  Pel selectChromaPix[4] = { 0, 0, 0, 0 };

  int cntT, cntL;
  cntT = cntL = 0;
  int cnt = 0;
  if (aboveAvailable)
  {
    cntT = std::min(actualTopTemplateSampNum, (1 + aboveIs4) << 1);
    src = srcColor0 - srcStride;
    cur = curChroma0 - curStride;
    for (int pos = startPos[0]; cnt < cntT; pos += pickStep[0], cnt++)
    {
      selectLumaPix[cnt] = src[pos];
      selectChromaPix[cnt] = cur[pos];
    }
  }

  if (leftAvailable)
  {
    cntL = std::min(actualLeftTemplateSampNum, ( 1 + leftIs4 ) << 1 );
    src = srcColor0 - 1;
    cur = curChroma0 - 1;
    for (int pos = startPos[1], cnt = 0; cnt < cntL; pos += pickStep[1], cnt++)
    {
      selectLumaPix[cnt + cntT] = src[pos * srcStride];
      selectChromaPix[cnt+ cntT] = cur[pos * curStride];
    }
  }
  cnt = cntL + cntT;

  if (cnt == 2)
  {
    selectLumaPix[3] = selectLumaPix[0]; selectChromaPix[3] = selectChromaPix[0];
    selectLumaPix[2] = selectLumaPix[1]; selectChromaPix[2] = selectChromaPix[1];
    selectLumaPix[0] = selectLumaPix[1]; selectChromaPix[0] = selectChromaPix[1];
    selectLumaPix[1] = selectLumaPix[3]; selectChromaPix[1] = selectChromaPix[3];
  }

  int minGrpIdx[2] = { 0, 2 };
  int maxGrpIdx[2] = { 1, 3 };
  int *tmpMinGrp = minGrpIdx;
  int *tmpMaxGrp = maxGrpIdx;
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMinGrp[1]]) std::swap(tmpMinGrp[0], tmpMinGrp[1]);
  if (selectLumaPix[tmpMaxGrp[0]] > selectLumaPix[tmpMaxGrp[1]]) std::swap(tmpMaxGrp[0], tmpMaxGrp[1]);
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMaxGrp[1]]) std::swap(tmpMinGrp, tmpMaxGrp);
  if (selectLumaPix[tmpMinGrp[1]] > selectLumaPix[tmpMaxGrp[0]]) std::swap(tmpMinGrp[1], tmpMaxGrp[0]);

  minLuma[0] = (selectLumaPix[tmpMinGrp[0]] + selectLumaPix[tmpMinGrp[1]] + 1 )>>1;
  minLuma[1] = (selectChromaPix[tmpMinGrp[0]] + selectChromaPix[tmpMinGrp[1]] + 1) >> 1;
  maxLuma[0] = (selectLumaPix[tmpMaxGrp[0]] + selectLumaPix[tmpMaxGrp[1]] + 1 )>>1;
  maxLuma[1] = (selectChromaPix[tmpMaxGrp[0]] + selectChromaPix[tmpMaxGrp[1]] + 1) >> 1;

  if (leftAvailable || aboveAvailable)
  {
    int diff = maxLuma[0] - minLuma[0];
    if (diff > 0)
    {
      int diffC = maxLuma[1] - minLuma[1];
      int x = floorLog2( diff );
      static const uint8_t DivSigTable[1 << 4] = {
        // 4bit significands - 8 ( MSB is omitted )
        0,  7,  6,  5,  5,  4,  4,  3,  3,  2,  2,  1,  1,  1,  1,  0
      };
      int normDiff = (diff << 4 >> x) & 15;
      int v = DivSigTable[normDiff] | 8;
      x += normDiff != 0;

      int y = floorLog2( abs( diffC ) ) + 1;
      int add = 1 << y >> 1;
      a = (diffC * v + add) >> y;
      iShift = 3 + x - y;
      if ( iShift < 1 )
      {
        iShift = 1;
        a = ( (a == 0)? 0: (a < 0)? -15 : 15 );   // a=Sign(a)*15
      }
      b = minLuma[1] - ((a * minLuma[0]) >> iShift);
    }
    else
    {
      a = 0;
      b = minLuma[1];
      iShift = 0;
    }
  }
  else
  {
    a = 0;

    b = 1 << (internalBitDepth - 1);

    iShift = 0;
  }
}

void IntraPrediction::initIntraMip( const PredictionUnit &pu )
{
#if JVET_O0545_MAX_TB_SIGNALLING
  CHECK( pu.lwidth() > pu.cs->sps->getMaxTbSize() || pu.lheight() > pu.cs->sps->getMaxTbSize(), "Error: block size not supported for MIP" );
#else
  CHECK( pu.lwidth() > MIP_MAX_WIDTH || pu.lheight() > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );
#endif

#if JVET_O0925_MIP_SIMPLIFICATIONS
  // prepare input (boundary) data for prediction
  CHECK(m_ipaParam.refFilterFlag, "ERROR: unfiltered refs expected for MIP");
  Pel *ptrSrc = getPredictorPtr(COMPONENT_Y);
  const int srcStride  = m_topRefLength  + 1;
  const int srcHStride = m_leftRefLength + 1;

  m_matrixIntraPred.prepareInputForPred(CPelBuf(ptrSrc, srcStride, srcHStride), pu.Y(), pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
#else
  // derive above and left availability
  AvailableInfo availInfo = PU::getAvailableInfoLuma(pu);

  // prepare input (boundary) data for prediction
  m_matrixIntraPred.prepareInputForPred(pu.cs->picture->getRecoBuf(COMPONENT_Y), pu.Y(), pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA), availInfo);
#endif
}

void IntraPrediction::predIntraMip( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu )
{
  CHECK( compId != COMPONENT_Y, "Error: chroma not supported" );
#if JVET_O0545_MAX_TB_SIGNALLING
  CHECK( pu.lwidth() > pu.cs->sps->getMaxTbSize() || pu.lheight() > pu.cs->sps->getMaxTbSize(), "Error: block size not supported for MIP" );
#else
  CHECK( pu.lwidth() > MIP_MAX_WIDTH || pu.lheight() > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );
#endif
  CHECK( pu.lwidth() != (1 << floorLog2(pu.lwidth())) || pu.lheight() != (1 << floorLog2(pu.lheight())), "Error: expecting blocks of size 2^M x 2^N" );

  // generate mode-specific prediction
  const int bitDepth = pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  static_vector<int, MIP_MAX_WIDTH * MIP_MAX_HEIGHT> predMip(pu.Y().area());
  m_matrixIntraPred.predBlock(predMip.data(), pu.intraDir[CHANNEL_TYPE_LUMA], bitDepth);

  for (int y = 0; y < pu.lheight(); y++)
  {
    for (int x = 0; x < pu.lwidth(); x++)
    {
#if JVET_O0925_MIP_SIMPLIFICATIONS
      piPred.at(x, y) = Pel(predMip[y * pu.lwidth() + x]);
#else
      piPred.at(x, y) = Pel(ClipBD<int>(predMip[y * pu.lwidth() + x], bitDepth));
#endif
    }
  }
}

#if JVET_O0119_BASE_PALETTE_444
bool IntraPrediction::calCopyRun(CodingStructure &cs, Partitioner& partitioner, uint32_t startPos, uint32_t total, uint32_t &run, ComponentID compBegin)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  PelBuf     curPLTIdx = tu.getcurPLTIdx(compBegin);
  PLTtypeBuf runType   = tu.getrunType(compBegin);

  uint32_t idx = startPos;
  uint32_t xPos;
  uint32_t yPos;
  bool valid = false;
  run = 0;
  while (idx < total)
  {
    xPos = m_scanOrder[idx].x;
    yPos = m_scanOrder[idx].y;
    runType.at(xPos, yPos) = PLT_RUN_COPY;

    if (yPos == 0 && !cu.useRotation[compBegin])
    {
      return false;
    }
    if (xPos == 0 && cu.useRotation[compBegin])
    {
      return false;
    }
    if (!cu.useRotation[compBegin] && curPLTIdx.at(xPos, yPos) == curPLTIdx.at(xPos, yPos - 1))
    {
      run++;
      valid = true;
    }
    else if (cu.useRotation[compBegin] && curPLTIdx.at(xPos, yPos) == curPLTIdx.at(xPos - 1, yPos))
    {
      run++;
      valid = true;
    }
    else
    {
      break;
    }
    idx++;
  }
  return valid;
}
bool IntraPrediction::calIndexRun(CodingStructure &cs, Partitioner& partitioner, uint32_t startPos, uint32_t total, uint32_t &run, ComponentID compBegin)
{
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  PelBuf     curPLTIdx = tu.getcurPLTIdx(compBegin);
  PLTtypeBuf runType   = tu.getrunType(compBegin);

  run = 1;
  uint32_t idx = startPos;
  while (idx < total)
  {
    uint32_t xPos = m_scanOrder[idx].x;
    uint32_t yPos = m_scanOrder[idx].y;
    runType.at(xPos, yPos) = PLT_RUN_INDEX;

    uint32_t xPrev = idx == 0 ? 0 : m_scanOrder[idx - 1].x;
    uint32_t yPrev = idx == 0 ? 0 : m_scanOrder[idx - 1].y;
    if (idx > startPos && curPLTIdx.at(xPos, yPos) == curPLTIdx.at(xPrev, yPrev))
    {
      run++;
    }
    else if (idx > startPos)
    {
      break;
    }
    idx++;
  }
  return true;
}
void IntraPrediction::reorderPLT(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);

  uint32_t       reusePLTSizetmp = 0;
  uint32_t       pltSizetmp = 0;
  Pel            curPLTtmp[MAX_NUM_COMPONENT][MAXPLTSIZE];
  bool           curPLTpred[MAXPLTPREDSIZE];

  for (int idx = 0; idx < MAXPLTPREDSIZE; idx++)
  {
    curPLTpred[idx] = false;
    cu.reuseflag[compBegin][idx] = false;
  }
  for (int idx = 0; idx < MAXPLTSIZE; idx++)
  {
    curPLTpred[idx] = false;
  }

  for (int predidx = 0; predidx < cs.prevPLT.curPLTSize[compBegin]; predidx++)
  {
    bool match = false;
    int curidx = 0;

    for (curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
    {
      bool matchTmp = true;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        matchTmp = matchTmp && (cu.curPLT[comp][curidx] == cs.prevPLT.curPLT[comp][predidx]);
      }
      if (matchTmp)
      {
        match = true;
        break;
      }
    }

    if (match)
    {
      cu.reuseflag[compBegin][predidx] = true;
      curPLTpred[curidx] = true;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        curPLTtmp[comp][reusePLTSizetmp] = cs.prevPLT.curPLT[comp][predidx];
      }
      reusePLTSizetmp++;
      pltSizetmp++;
    }
  }
  cu.reusePLTSize[compBegin] = reusePLTSizetmp;
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
    if (!curPLTpred[curidx])
    {
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        curPLTtmp[comp][pltSizetmp] = cu.curPLT[comp][curidx];
      }
      pltSizetmp++;
    }
  }
  assert(pltSizetmp == cu.curPLTSize[compBegin]);
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      cu.curPLT[comp][curidx] = curPLTtmp[comp][curidx];
    }
  }
}
#endif

//! \}
