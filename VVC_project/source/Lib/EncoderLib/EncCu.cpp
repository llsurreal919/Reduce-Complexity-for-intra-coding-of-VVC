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

 /** \file     EncCu.cpp
     \brief    Coding Unit (CU) encoder class
 */

#include "EncCu.h"

#include "EncLib.h"
#include "Analyze.h"
#include "AQp.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "MCTS.h"


#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>
#if ENABLE_WPP_PARALLELISM
#include <mutex>
extern std::recursive_mutex g_cache_mutex;
#endif
#include <opencv2/opencv.hpp>
#include <Python.h>
#include <iostream>



//! \ingroup EncoderLib
//! \{

// ====================================================================================================================

//my code
//Start
#if FAST_ALGORITHM
cv::Mat EncCu::get_madp(int height, int width, cv::Mat pixel)
{
  cv::Mat MADP = cv::Mat::eye(height, width, CV_32S);
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if ((i == 0) && (j == 0))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(0, 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(1, 0)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(1, 1)) - int(pixel.at<uchar>(i, j)))) / 3;
      }
      else if ((i == 0) && (1 <= j && j <= width - 2))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i, j - 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i + 1, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i + 1, j)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i + 1, j + 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i, j + 1)) - int(pixel.at<uchar>(i, j)))) / 5;
      }
      else if ((i == 0) && (j == width - 1))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i, j - 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i + 1, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i + 1, j)) - int(pixel.at<uchar>(i, j)))) / 3;
      }
      else if ((j == 0) && (1 <= i && i <= height - 2))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i - 1, j)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i - 1, j + 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i, j + 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i + 1, j + 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i + 1, j)) - int(pixel.at<uchar>(i, j)))) / 5;
      }
      else if ((j == width - 1) && (1 <= i && i <= height - 2))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i - 1, j)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i - 1, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i, j - 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i + 1, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i + 1, j)) - int(pixel.at<uchar>(i, j)))) / 5;
      }
      else if ((i == height - 1) && (j == 0))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i - 1, j)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i - 1, j + 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i, j + 1)) - int(pixel.at<uchar>(i, j)))) / 3;
      }
      else if ((i == height - 1) && (1 <= j && j <= width - 2))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i, j - 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i - 1, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i - 1, j)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i - 1, j + 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i, j + 1)) - int(pixel.at<uchar>(i, j)))) / 5;
      }
      else if ((i == height - 1) && (j == width - 1))
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i, j - 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i - 1, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i - 1, j)) - int(pixel.at<uchar>(i, j)))) / 3;
      }
      else
      {
        MADP.at<int>(i, j) = (abs(int(pixel.at<uchar>(i - 1, j - 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i - 1, j)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i - 1, j + 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i, j + 1)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i + 1, j - 1)) - int(pixel.at<uchar>(i, j)))
          + abs(int(pixel.at<uchar>(i + 1, j)) - int(pixel.at<uchar>(i, j))) + abs(int(pixel.at<uchar>(i + 1, j + 1)) - int(pixel.at<uchar>(i, j)))) / 8;
      }
    }
  }
  return MADP;
}

//function to get the variance and depth of neighboring CU
std::vector<int> EncCu::get_context(const CodingUnit* neighbor_cu, AreaBuf<Pel>& pic_Y)
{
  std::vector<int> pixel_array;
  std::vector<int> Info;
  for (int i = neighbor_cu->ly(); i < (neighbor_cu->ly() + neighbor_cu->lheight()); i++)
  {
    for (int j = neighbor_cu->lx(); j < (neighbor_cu->lx() + neighbor_cu->lwidth()); j++)
    {
      pixel_array.push_back(pic_Y.at(j, i));
    }
  }

  cv::Mat pixel_mat = cv::Mat(pixel_array);
  cv::Mat pixel_copy = pixel_mat.reshape(0, neighbor_cu->lheight());
  cv::Mat Pixel;
  pixel_copy.convertTo(Pixel, CV_8U);

  cv::Scalar     mean;
  cv::Scalar     stddev;
  cv::meanStdDev(Pixel, mean, stddev);
  double var = stddev[0] * stddev[0];

  Info.push_back(int(var));
  Info.push_back(int(neighbor_cu->qtDepth));
  Info.push_back(int(neighbor_cu->mtDepth));
  return Info;
}
#endif
//End


EncCu::EncCu() : m_triangleModeTest
{
  TriangleMotionInfo(0, 1, 0), TriangleMotionInfo(1, 0, 1), TriangleMotionInfo(1, 0, 2), TriangleMotionInfo(0, 0, 1), TriangleMotionInfo(0, 2, 0),
  TriangleMotionInfo(1, 0, 3), TriangleMotionInfo(1, 0, 4), TriangleMotionInfo(1, 1, 0), TriangleMotionInfo(0, 3, 0), TriangleMotionInfo(0, 4, 0),
  TriangleMotionInfo(0, 0, 2), TriangleMotionInfo(0, 1, 2), TriangleMotionInfo(1, 1, 2), TriangleMotionInfo(0, 0, 4), TriangleMotionInfo(0, 0, 3),
  TriangleMotionInfo(0, 1, 3), TriangleMotionInfo(0, 1, 4), TriangleMotionInfo(1, 1, 4), TriangleMotionInfo(1, 1, 3), TriangleMotionInfo(1, 2, 1),
  TriangleMotionInfo(1, 2, 0), TriangleMotionInfo(0, 2, 1), TriangleMotionInfo(0, 4, 3), TriangleMotionInfo(1, 3, 0), TriangleMotionInfo(1, 3, 2),
  TriangleMotionInfo(1, 3, 4), TriangleMotionInfo(1, 4, 0), TriangleMotionInfo(1, 3, 1), TriangleMotionInfo(1, 2, 3), TriangleMotionInfo(1, 4, 1),
  TriangleMotionInfo(0, 4, 1), TriangleMotionInfo(0, 2, 3), TriangleMotionInfo(1, 4, 2), TriangleMotionInfo(0, 3, 2), TriangleMotionInfo(1, 4, 3),
  TriangleMotionInfo(0, 3, 1), TriangleMotionInfo(0, 2, 4), TriangleMotionInfo(1, 2, 4), TriangleMotionInfo(0, 4, 2), TriangleMotionInfo(0, 3, 4),
}
{}

void EncCu::create(EncCfg* encCfg)
{
  unsigned      uiMaxWidth = encCfg->getMaxCUWidth();
  unsigned      uiMaxHeight = encCfg->getMaxCUHeight();
  ChromaFormat  chromaFormat = encCfg->getChromaFormatIdc();

  unsigned      numWidths = gp_sizeIdxInfo->numWidths();
  unsigned      numHeights = gp_sizeIdxInfo->numHeights();
  m_pTempCS = new CodingStructure**[numWidths];
  m_pBestCS = new CodingStructure**[numWidths];
#if JVET_O0050_LOCAL_DUAL_TREE
  m_pTempCS2 = new CodingStructure**[numWidths];
  m_pBestCS2 = new CodingStructure**[numWidths];
#endif

  for (unsigned w = 0; w < numWidths; w++)
  {
    m_pTempCS[w] = new CodingStructure*[numHeights];
    m_pBestCS[w] = new CodingStructure*[numHeights];
#if JVET_O0050_LOCAL_DUAL_TREE
    m_pTempCS2[w] = new CodingStructure*[numHeights];
    m_pBestCS2[w] = new CodingStructure*[numHeights];
#endif

    for (unsigned h = 0; h < numHeights; h++)
    {
      unsigned width = gp_sizeIdxInfo->sizeFrom(w);
      unsigned height = gp_sizeIdxInfo->sizeFrom(h);

      if (gp_sizeIdxInfo->isCuSize(width) && gp_sizeIdxInfo->isCuSize(height))
      {
        m_pTempCS[w][h] = new CodingStructure(m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache);
        m_pBestCS[w][h] = new CodingStructure(m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache);

        m_pTempCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false);
        m_pBestCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false);

#if JVET_O0050_LOCAL_DUAL_TREE
        m_pTempCS2[w][h] = new CodingStructure(m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache);
        m_pBestCS2[w][h] = new CodingStructure(m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache);

        m_pTempCS2[w][h]->create(chromaFormat, Area(0, 0, width, height), false);
        m_pBestCS2[w][h]->create(chromaFormat, Area(0, 0, width, height), false);
#endif
      }
      else
      {
        m_pTempCS[w][h] = nullptr;
        m_pBestCS[w][h] = nullptr;
#if JVET_O0050_LOCAL_DUAL_TREE
        m_pTempCS2[w][h] = nullptr;
        m_pBestCS2[w][h] = nullptr;
#endif
      }
    }
  }

  m_cuChromaQpOffsetIdxPlus1 = 0;

  unsigned maxDepth = numWidths + numHeights;

  m_modeCtrl = new EncModeCtrlMTnoRQT();

  m_modeCtrl->create(*encCfg);

  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
  {
    m_acMergeBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
    m_acMergeTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#endif
  }
  const unsigned maxNumTriangleCand = encCfg->getMaxNumTriangleCand();
  for (unsigned i = 0; i < maxNumTriangleCand; i++)
  {
    for (unsigned j = 0; j < maxNumTriangleCand; j++)
    {
      if (i == j)
        continue;
      uint8_t idxBits0 = i + (i == maxNumTriangleCand - 1 ? 0 : 1);
      uint8_t candIdx1Enc = j - (j > i ? 1 : 0);
      uint8_t idxBits1 = candIdx1Enc + (candIdx1Enc == maxNumTriangleCand - 2 ? 0 : 1);
      m_triangleIdxBins[1][i][j] = m_triangleIdxBins[0][i][j] = 1 + idxBits0 + idxBits1;
    }
  }
  if (maxNumTriangleCand != 5)
  {
    // update the table
    int index = 0;
    for (unsigned i = 0; i < maxNumTriangleCand; i++)
    {
      for (unsigned j = 0; j < maxNumTriangleCand; j++)
      {
        if (i == j)
          continue;
        for (unsigned dir = 0; dir < 2; dir++, index++)
        {
          m_triangleModeTest[index].m_splitDir = dir;
          m_triangleModeTest[index].m_candIdx0 = i;
          m_triangleModeTest[index].m_candIdx1 = j;
        }
      }
    }
  }
  for (unsigned ui = 0; ui < TRIANGLE_MAX_NUM_CANDS; ui++)
  {
    m_acTriangleWeightedBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }

  m_CtxBuffer.resize(maxDepth);
  m_CurrCtx = 0;
}


void EncCu::destroy()
{
  unsigned numWidths = gp_sizeIdxInfo->numWidths();
  unsigned numHeights = gp_sizeIdxInfo->numHeights();

  for (unsigned w = 0; w < numWidths; w++)
  {
    for (unsigned h = 0; h < numHeights; h++)
    {
      if (m_pBestCS[w][h]) m_pBestCS[w][h]->destroy();
      if (m_pTempCS[w][h]) m_pTempCS[w][h]->destroy();

      delete m_pBestCS[w][h];
      delete m_pTempCS[w][h];

#if JVET_O0050_LOCAL_DUAL_TREE
      if (m_pBestCS2[w][h]) m_pBestCS2[w][h]->destroy();
      if (m_pTempCS2[w][h]) m_pTempCS2[w][h]->destroy();

      delete m_pBestCS2[w][h];
      delete m_pTempCS2[w][h];
#endif
    }

    delete[] m_pTempCS[w];
    delete[] m_pBestCS[w];
#if JVET_O0050_LOCAL_DUAL_TREE
    delete[] m_pTempCS2[w];
    delete[] m_pBestCS2[w];
#endif
  }

  delete[] m_pBestCS; m_pBestCS = nullptr;
  delete[] m_pTempCS; m_pTempCS = nullptr;
#if JVET_O0050_LOCAL_DUAL_TREE
  delete[] m_pBestCS2; m_pBestCS2 = nullptr;
  delete[] m_pTempCS2; m_pTempCS2 = nullptr;
#endif

#if REUSE_CU_RESULTS
  if (m_tmpStorageLCU)
  {
    m_tmpStorageLCU->destroy();
    delete m_tmpStorageLCU;  m_tmpStorageLCU = nullptr;
  }
#endif

#if REUSE_CU_RESULTS
  m_modeCtrl->destroy();

#endif
  delete m_modeCtrl;
  m_modeCtrl = nullptr;

  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
  {
    m_acMergeBuffer[ui].destroy();
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].destroy();
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
    m_acMergeTmpBuffer[ui].destroy();
#endif
  }
  for (unsigned ui = 0; ui < TRIANGLE_MAX_NUM_CANDS; ui++)
  {
    m_acTriangleWeightedBuffer[ui].destroy();
  }
}



EncCu::~EncCu()
{
}



/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init(EncLib* pcEncLib, const SPS& sps PARL_PARAM(const int tId))
{
  m_pcEncCfg = pcEncLib;
  m_pcIntraSearch = pcEncLib->getIntraSearch(PARL_PARAM0(tId));
  m_pcInterSearch = pcEncLib->getInterSearch(PARL_PARAM0(tId));
  m_pcTrQuant = pcEncLib->getTrQuant(PARL_PARAM0(tId));
  m_pcRdCost = pcEncLib->getRdCost(PARL_PARAM0(tId));
  m_CABACEstimator = pcEncLib->getCABACEncoder(PARL_PARAM0(tId))->getCABACEstimator(&sps);
  m_CABACEstimator->setEncCu(this);
  m_CtxCache = pcEncLib->getCtxCache(PARL_PARAM0(tId));
  m_pcRateCtrl = pcEncLib->getRateCtrl();
  m_pcSliceEncoder = pcEncLib->getSliceEncoder();
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  m_pcEncLib = pcEncLib;
  m_dataId = tId;
#endif
  m_pcLoopFilter = pcEncLib->getLoopFilter();
  m_shareState = NO_SHARE;
  m_pcInterSearch->setShareState(0);
  setShareStateDec(0);

  m_shareBndPosX = -1;
  m_shareBndPosY = -1;
  m_shareBndSizeW = 0;
  m_shareBndSizeH = 0;

  DecCu::init(m_pcTrQuant, m_pcIntraSearch, m_pcInterSearch);

  m_modeCtrl->init(m_pcEncCfg, m_pcRateCtrl, m_pcRdCost);

  m_pcInterSearch->setModeCtrl(m_modeCtrl);
#if JVET_O0592_ENC_ME_IMP
  m_modeCtrl->setInterSearch(m_pcInterSearch);
#endif
  m_pcIntraSearch->setModeCtrl(m_modeCtrl);

#if !JVET_O1164_PS
  if ((m_pcEncCfg->getIBCHashSearch() && m_pcEncCfg->getIBCMode()) || m_pcEncCfg->getAllowDisFracMMVD())
  {
    m_ibcHashMap.init(m_pcEncCfg->getSourceWidth(), m_pcEncCfg->getSourceHeight());
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu(CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[])
{
  m_modeCtrl->initCTUEncoding(*cs.slice);
#if JVET_O0050_LOCAL_DUAL_TREE
  cs.treeType = TREE_D;
#endif

#if JVET_O0119_BASE_PALETTE_444
  cs.slice->m_mapPltCost.clear();
#endif
#if ENABLE_SPLIT_PARALLELISM
  if (m_pcEncCfg->getNumSplitThreads() > 1)
  {
    for (int jId = 1; jId < NUM_RESERVERD_SPLIT_JOBS; jId++)
    {
      EncCu*            jobEncCu = m_pcEncLib->getCuEncoder(cs.picture->scheduler.getSplitDataId(jId));
      CacheBlkInfoCtrl* cacheCtrl = dynamic_cast<CacheBlkInfoCtrl*>(jobEncCu->m_modeCtrl);
#if REUSE_CU_RESULTS
      BestEncInfoCache* bestCache = dynamic_cast<BestEncInfoCache*>(jobEncCu->m_modeCtrl);
#endif
      SaveLoadEncInfoSbt *sbtCache = dynamic_cast<SaveLoadEncInfoSbt*>(jobEncCu->m_modeCtrl);
      if (cacheCtrl)
      {
        cacheCtrl->init(*cs.slice);
      }
#if REUSE_CU_RESULTS
      if (bestCache)
      {
        bestCache->init(*cs.slice);
      }
#endif
      if (sbtCache)
      {
        sbtCache->init(*cs.slice);
      }
    }
  }

#if REUSE_CU_RESULTS
  if (auto* cacheCtrl = dynamic_cast<BestEncInfoCache*>(m_modeCtrl)) { cacheCtrl->tick(); }
#endif
  if (auto* cacheCtrl = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl)) { cacheCtrl->tick(); }
#endif
  // init the partitioning manager
  QTBTPartitioner partitioner;
  partitioner.initCtu(area, CH_L, *cs.slice);
  if (m_pcEncCfg->getIBCMode())
  {
    if (area.lx() == 0 && area.ly() == 0)
    {
      m_pcInterSearch->resetIbcSearch();
    }
    m_pcInterSearch->resetCtuRecord();
    m_ctuIbcSearchRangeX = m_pcEncCfg->getIBCLocalSearchRangeX();
    m_ctuIbcSearchRangeY = m_pcEncCfg->getIBCLocalSearchRangeY();
  }
  if (m_pcEncCfg->getIBCMode() && m_pcEncCfg->getIBCHashSearch() && (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE))
  {
    const int hashHitRatio = m_ibcHashMap.getHashHitRatio(area.Y()); // in percent
    if (hashHitRatio < 5) // 5%
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
    if (cs.slice->getNumRefIdx(REF_PIC_LIST_0) > 0)
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
  }
  // init current context pointer
  m_CurrCtx = m_CtxBuffer.data();

  CodingStructure *tempCS = m_pTempCS[gp_sizeIdxInfo->idxFrom(area.lumaSize().width)][gp_sizeIdxInfo->idxFrom(area.lumaSize().height)];
  CodingStructure *bestCS = m_pBestCS[gp_sizeIdxInfo->idxFrom(area.lumaSize().width)][gp_sizeIdxInfo->idxFrom(area.lumaSize().height)];

  cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
  cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
  tempCS->currQP[CH_L] = bestCS->currQP[CH_L] =
    tempCS->baseQP = bestCS->baseQP = currQP[CH_L];
  tempCS->prevQP[CH_L] = bestCS->prevQP[CH_L] = prevQP[CH_L];

  xCompressCU(tempCS, bestCS, partitioner);
#if JVET_O0119_BASE_PALETTE_444
  cs.slice->m_mapPltCost.clear();
#endif
  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
  cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType), copyUnsplitCTUSignals,
    false, false, copyUnsplitCTUSignals);

  if (CS::isDualITree(cs) && isChromaEnabled(cs.pcv->chrFormat))
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner.initCtu(area, CH_C, *cs.slice);

    cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
    cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
    tempCS->currQP[CH_C] = bestCS->currQP[CH_C] =
      tempCS->baseQP = bestCS->baseQP = currQP[CH_C];
    tempCS->prevQP[CH_C] = bestCS->prevQP[CH_C] = prevQP[CH_C];

    xCompressCU(tempCS, bestCS, partitioner);

    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
    cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType),
      copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals);
  }

  if (m_pcEncCfg->getUseRateCtrl())
  {
    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_actualMSE = (double)bestCS->dist / (double)m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr).m_numberOfPixel;
  }
  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx = 0;

#if ENABLE_SPLIT_PARALLELISM && ENABLE_WPP_PARALLELISM
  if (m_pcEncCfg->getNumSplitThreads() > 1 && m_pcEncCfg->getNumWppThreads() > 1)
  {
    cs.picture->finishCtuPart(area);
  }
#endif

  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK(bestCS->cus.empty(), "No possible encoding found");
  CHECK(bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found");
  CHECK(bestCS->cost == MAX_DOUBLE, "No possible encoding found");
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

static int xCalcHADs8x8_ISlice(const Pel *piOrg, const int iStrideOrg)
{
  int k, i, j, jj;
  int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for (k = 0; k < 64; k += 8)
  {
    diff[k + 0] = piOrg[0];
    diff[k + 1] = piOrg[1];
    diff[k + 2] = piOrg[2];
    diff[k + 3] = piOrg[3];
    diff[k + 4] = piOrg[4];
    diff[k + 5] = piOrg[5];
    diff[k + 6] = piOrg[6];
    diff[k + 7] = piOrg[7];

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j = 0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i = 0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad = (iSumHad + 2) >> 2;
  return(iSumHad);
}

int  EncCu::updateCtuDataISlice(const CPelBuf buf)
{
  int  xBl, yBl;
  const int iBlkSize = 8;
  const Pel* pOrgInit = buf.buf;
  int  iStrideOrig = buf.stride;

  int iSumHad = 0;
  for (yBl = 0; (yBl + iBlkSize) <= buf.height; yBl += iBlkSize)
  {
    for (xBl = 0; (xBl + iBlkSize) <= buf.width; xBl += iBlkSize)
    {
      const Pel* pOrg = pOrgInit + iStrideOrig * yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return(iSumHad);
}

bool EncCu::xCheckBestMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  bool bestCSUpdated = false;

  if (!tempCS->cus.empty())
  {
    if (tempCS->cus.size() == 1)
    {
      const CodingUnit& cu = *tempCS->cus.front();
      CHECK(cu.skip && !cu.firstPU->mergeFlag, "Skip flag without a merge flag is not allowed!");
    }

#if WCG_EXT
    DTRACE_BEST_MODE(tempCS, bestCS, m_pcRdCost->getLambda(true));
#else
    DTRACE_BEST_MODE(tempCS, bestCS, m_pcRdCost->getLambda());
#endif

    if (m_modeCtrl->useModeResult(encTestMode, tempCS, partitioner))
    {
      if (tempCS->cus.size() == 1)
      {
        // if tempCS is not a split-mode
        CodingUnit &cu = *tempCS->cus.front();

#if !JVET_O0525_REMOVE_PCM
        if (CU::isLosslessCoded(cu) && !cu.ipcm)
#else
        if (CU::isLosslessCoded(cu))
#endif
        {
          xFillPCMBuffer(cu);
        }
      }

      std::swap(tempCS, bestCS);
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
      m_bestModeUpdated = true;
      bestCSUpdated = true;
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  return bestCSUpdated;

}

#if JVET_O0502_ISP_CLEANUP
void EncCu::xCompressCU(CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& partitioner, double maxCostAllowed)
#else
void EncCu::xCompressCU(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner)
#endif
{
#if JVET_O0502_ISP_CLEANUP
  CHECK(maxCostAllowed < 0, "Wrong value of maxCostAllowed!");
#endif
  if (m_shareState == NO_SHARE)
  {
    tempCS->sharedBndPos = tempCS->area.Y().lumaPos();
    tempCS->sharedBndSize.width = tempCS->area.lwidth();
    tempCS->sharedBndSize.height = tempCS->area.lheight();
    bestCS->sharedBndPos = bestCS->area.Y().lumaPos();
    bestCS->sharedBndSize.width = bestCS->area.lwidth();
    bestCS->sharedBndSize.height = bestCS->area.lheight();
  }
#if ENABLE_SPLIT_PARALLELISM
  CHECK(m_dataId != tempCS->picture->scheduler.getDataId(), "Working in the wrong dataId!");

  if (m_pcEncCfg->getNumSplitThreads() != 1 && tempCS->picture->scheduler.getSplitJobId() == 0)
  {
    if (m_modeCtrl->isParallelSplit(*tempCS, partitioner))
    {
      m_modeCtrl->setParallelSplit(true);
      xCompressCUParallel(tempCS, bestCS, partitioner);
      return;
    }
  }

#endif
#if JVET_O0119_BASE_PALETTE_444
  uint32_t compBegin;
  uint32_t numComp;
  bool jointPLT = false;
#if JVET_O0050_LOCAL_DUAL_TREE
  if (partitioner.isSepTree(*tempCS))
#else
  if (CS::isDualITree(*bestCS))
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
  SplitSeries splitmode = -1;
  uint32_t  bestLastPLTSize[MAX_NUM_COMPONENT];
  Pel       bestLastPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE]; // store LastPLT for partition
  uint32_t  curLastPLTSize[MAX_NUM_COMPONENT];
  Pel       curLastPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE]; // store LastPLT if no partition
  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
    bestLastPLTSize[comID] = 0;
    curLastPLTSize[comID] = tempCS->prevPLT.curPLTSize[comID];
    memcpy(curLastPLT[i], tempCS->prevPLT.curPLT[i], tempCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
  }
#endif

  Slice&   slice = *tempCS->slice;
  const PPS &pps = *tempCS->pps;
  const SPS &sps = *tempCS->sps;
  const uint32_t uiLPelX = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY = tempCS->area.Y().lumaPos().y;

#if JVET_O0050_LOCAL_DUAL_TREE
  const ModeType modeTypeParent = partitioner.modeType;
  const TreeType treeTypeParent = partitioner.treeType;
  const ChannelType chTypeParent = partitioner.chType;
#endif
  const UnitArea currCsArea = clipArea(CS::getArea(*bestCS, bestCS->area, partitioner.chType), *tempCS->picture);

  m_modeCtrl->initCULevel(partitioner, *tempCS);



#if FAST_ALGORITHM
  int res = -2;
  clock_t start, finish;
  start = clock();
  //Our algorithm only reduce the complexity of luma component
  if (partitioner.chType == CHANNEL_TYPE_LUMA)
  {
    using namespace std;
    using namespace cv;
    int x = currCsArea.lx();
    int y = currCsArea.ly();
    int height = currCsArea.lumaSize().height;
    int width = currCsArea.lumaSize().width;
    int CU_size = height * width;
    int depth_QT = int(partitioner.currQtDepth);
    int depth_MT = int(partitioner.currMtDepth);
    int video_width = 416;
    int video_height = 240;

    if (height < 128 && ((x + width <= video_width) && (y + height <= video_height)))
    {
      if (depth_MT == 3)
      {
      }
      else
      {
        if (height == 4 && width == 4)
        {
        }
        else
        {
          Picture *pcPic = tempCS->picture;
          PelUnitBuf picbuf = pcPic->getOrigBuf();
          AreaBuf<Pel>& picbuf_Y = picbuf.Y();
          vector<int> PixelArray;
          vector<int> PixelLeft;
          vector<int> PixelLeftUp;
          vector<int> PixelLeftDown;
          vector<int> PixelUp;
          vector<int> PixelRightUp;

          const CodingUnit* cuLeft = tempCS->getCU(currCsArea.Y().pos().offset(-1, 0), CHANNEL_TYPE_LUMA);      //Left
          const CodingUnit* cuUp = tempCS->getCU(currCsArea.Y().pos().offset(0, -1), CHANNEL_TYPE_LUMA);        //Up      
          const CodingUnit* cuLeftUp = tempCS->getCU(currCsArea.Y().pos().offset(-1, -1), CHANNEL_TYPE_LUMA);   //LeftUp

          int H = y + height;
          int W = x + width;
          for (int i = y; i < H; i++)
          {
            for (int j = x; j < W; j++)
            {
              PixelArray.push_back(picbuf_Y.at(j, i));
            }
          }

          vector<int> NCC_array;
          vector<int> NCD_QT_array;
          vector<int> NCD_MT_array;
          int valid_num = 0;

          if (cuLeft != NULL)  //Left
          {
            vector<int> Info_left = get_context(cuLeft, picbuf_Y);
            valid_num += 1;
            NCC_array.push_back(Info_left[0]);
            NCD_QT_array.push_back(Info_left[1]);
            NCD_MT_array.push_back(Info_left[2]);
            const CodingUnit* cuLeftDown = tempCS->getCU(currCsArea.Y().pos().offset(-1, int(cuLeft->lheight()) + 1), CHANNEL_TYPE_LUMA); //LeftDown
            if (cuLeftDown != NULL)
            {
              if (cuLeftDown->ly() <= (y + height))
              {
                vector<int> Info_leftdown = get_context(cuLeftDown, picbuf_Y);
                valid_num += 1;
                NCC_array.push_back(Info_leftdown[0]);
                NCD_QT_array.push_back(Info_leftdown[1]);
                NCD_MT_array.push_back(Info_leftdown[2]);
              }
            }
          }

          if (cuUp != NULL) //Up
          {
            vector<int> Info_up = get_context(cuUp, picbuf_Y);
            valid_num += 1;
            NCC_array.push_back(Info_up[0]);
            NCD_QT_array.push_back(Info_up[1]);
            NCD_MT_array.push_back(Info_up[2]);
            const CodingUnit* cuRightUp = tempCS->getCU(currCsArea.Y().pos().offset(int(cuUp->lwidth()) + 1, -1), CHANNEL_TYPE_LUMA); //RightUp
            if (cuRightUp != NULL)
            {
              if (cuRightUp->lx() < (x + width)) 
              {
                vector<int> Info_rightup = get_context(cuRightUp, picbuf_Y);
                valid_num += 1;
                NCC_array.push_back(Info_rightup[0]);
                NCD_QT_array.push_back(Info_rightup[1]);
                NCD_MT_array.push_back(Info_rightup[2]);
              }
            }
          }

          if (cuLeftUp != NULL) //LeftUp
          {
            if (((cuLeftUp->ly() + cuLeftUp->lheight()) > y) || ((cuLeftUp->lx() + cuLeftUp->lwidth()) > x))
            {
            }
            else
            {
              vector<int> Info_leftup = get_context(cuLeftUp, picbuf_Y);
              valid_num += 1;
              NCC_array.push_back(Info_leftup[0]);
              NCD_QT_array.push_back(Info_leftup[1]);
              NCD_MT_array.push_back(Info_leftup[2]);
            }
          }

          if (valid_num >= 3)
          {
            Mat pixel = Mat(PixelArray);
            PixelArray.clear();
            Mat pixel_copy = pixel.reshape(0, height);
            Mat Pixel;
            pixel_copy.convertTo(Pixel, CV_8U);
            Mat MADP = Mat::eye(height, width, CV_32S);
            MADP = get_madp(height, width, Pixel);

            Scalar     mean;
            Scalar     stddev;
            cv::meanStdDev(Pixel, mean, stddev);
            double Var = stddev[0] * stddev[0];        
            Scalar     MADP_mean;
            Scalar     MADP_std;
            cv::meanStdDev(MADP, MADP_mean, MADP_std);
            double NMSE = MADP_std[0] * MADP_std[0];

            int QTD = 0;
            int QTMTD = 0;
            int NCC_max = 0;
            int NCC_min = 0;
            int NCC_avg = 0;
            int NCC_sum = 0;
            int NCD_QT_max = 0;
            int NCD_QT_min = 0;
            int NCD_QT_avg = 0;
            int NCD_QT_sum = 0;
            int NCD_MT_max = 0;
            int NCD_MT_min = 0;
            int NCD_MT_avg = 0;
            int NCD_MT_sum = 0;

            std::sort(NCC_array.begin(), NCC_array.end());
            for (int i = 0; i < int(NCC_array.size()); i++)
            {
              NCC_sum += NCC_array[i];
            }
            int size = int(NCC_array.size());
            NCC_min = NCC_array[0];
            NCC_max = NCC_array[size - 1];       
            NCC_avg = NCC_sum / size;
            std::sort(NCD_QT_array.begin(), NCD_QT_array.end());
            std::sort(NCD_MT_array.begin(), NCD_MT_array.end());

            for (int i = 0; i < int(NCD_QT_array.size()); i++)
            {
              NCD_QT_sum += NCD_QT_array[i];
            }
            size = int(NCD_QT_array.size());
            NCD_QT_min = NCD_QT_array[0];
            NCD_QT_max = NCD_QT_array[size - 1];           
            NCD_QT_avg = NCD_QT_sum / size;

            for (int i = 0; i < int(NCD_MT_array.size()); i++)
            {
              NCD_MT_sum += NCD_MT_array[i];
            }
            size = int(NCD_MT_array.size());
            NCD_MT_min = NCD_MT_array[0];
            NCD_MT_max = NCD_MT_array[size - 1];             
            NCD_MT_avg = NCD_MT_sum / size;

            Mat kern_H = (cv::Mat_<char>(3, 3) << -1, 0, 1,
              -2, 0, 2,
              -1, 0, 1);
            Mat kern_V = (cv::Mat_<char>(3, 3) << 1, 2, 1,
              0, 0, 0,
              -1, -2, -1);
            Mat Gra_H;
            Mat Gra_V;
            cv::filter2D(Pixel, Gra_H, Pixel.depth(), kern_H);
            cv::filter2D(Pixel, Gra_V, Pixel.depth(), kern_V);

            Mat kern_45 = (Mat_<char>(3, 3) << 0, 1, 2,
              -1, 0, 1,
              -2, -1, 0);
            Mat kern_135 = (Mat_<char>(3, 3) << 2, 1, 0,
              1, 0, -1,
              0, -1, -2);
            Mat Gra_45;
            Mat Gra_135;
            cv::filter2D(Pixel, Gra_45, Pixel.depth(), kern_45);
            cv::filter2D(Pixel, Gra_135, Pixel.depth(), kern_135);

            double G_H = sum(Gra_H)[0] / CU_size;          
            double G_V = sum(Gra_V)[0] / CU_size;         
            double G_45 = sum(Gra_45)[0] / CU_size;
            double G_135 = sum(Gra_135)[0] / CU_size;
            double Gra = (G_H + G_V + G_45 + G_135) / 4;
            double minv = 0.0, maxv = 0.0;
            double* Gra_Min = &minv;
            double* Gra_Max = &maxv;
            cv::minMaxIdx(Gra_H / 4 + Gra_V / 4 + Gra_45 / 4 + Gra_135 / 4, Gra_Min, Gra_Max);   


            Mat Pixel_BH_Up = Pixel.rowRange(0, height / 2).clone();          
            Mat Pixel_BH_Down = Pixel.rowRange(height / 2, height).clone();  

            Mat Pixel_BV_Left = Pixel.colRange(0, width / 2).clone();        
            Mat Pixel_BV_Right = Pixel.colRange(width / 2, width).clone();    

            Mat Pixel_TH_Up = Pixel.rowRange(0, height / 4).clone();                         
            Mat Pixel_TH_Mid = Pixel.rowRange(height / 4, 3 * height / 4).clone();          
            Mat Pixel_TH_Down = Pixel.rowRange(3 * height / 4, height).clone();            

            Mat Pixel_TV_Left = Pixel.colRange(0, width / 4).clone();                      
            Mat Pixel_TV_Mid = Pixel.colRange(width / 4, 3 * width / 4).clone();          
            Mat Pixel_TV_Right = Pixel.colRange(3 * width / 4, width).clone();             

            Mat Pixel_QT_1 = Pixel(Range(0, height / 2), Range(0, width / 2));           
            Mat Pixel_QT_2 = Pixel(Range(0, height / 2), Range(width / 2, width));          
            Mat Pixel_QT_3 = Pixel(Range(height / 2, height), Range(0, width / 2));        
            Mat Pixel_QT_4 = Pixel(Range(height / 2, height), Range(width / 2, width));    



            int* features = new int[27];

            cv::meanStdDev(Pixel_BH_Up, mean, stddev);    
            int B1 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_BH_Down, mean, stddev);
            int B2 = int(stddev[0] * stddev[0]);
            int B_mean = (B1 + B2) / 2;
            int SCCD_BH = ((B1 - B_mean)*(B1 - B_mean) + (B2 - B_mean)*(B2 - B_mean)) / 2;

            cv::meanStdDev(Pixel_BV_Left, mean, stddev); 
            B1 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_BV_Right, mean, stddev);
            B2 = int(stddev[0] * stddev[0]);
            B_mean = (B1 + B2) / 2;
            int SCCD_BV = ((B1 - B_mean)*(B1 - B_mean) + (B2 - B_mean)*(B2 - B_mean)) / 2;

            cv::meanStdDev(Pixel_TH_Up, mean, stddev);     
            int T1 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_TH_Mid, mean, stddev);
            int T2 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_TH_Down, mean, stddev);
            int T3 = int(stddev[0] * stddev[0]);
            int T_mean = (T1 + T2 + T3) / 3;
            int SCCD_TH = ((T1 - T_mean)*(T1 - T_mean) + (T2 - T_mean)*(T2 - T_mean) + (T3 - T_mean)*(T3 - T_mean)) / 3;


            cv::meanStdDev(Pixel_TV_Left, mean, stddev);  
            T1 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_TV_Mid, mean, stddev);
            T2 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_TV_Right, mean, stddev);
            T3 = int(stddev[0] * stddev[0]);
            T_mean = (T1 + T2 + T3) / 3;
            int SCCD_TV = ((T1 - T_mean)*(T1 - T_mean) + (T2 - T_mean)*(T2 - T_mean) + (T3 - T_mean)*(T3 - T_mean)) / 3;

            cv::meanStdDev(Pixel_QT_1, mean, stddev);
            int Q1 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_QT_2, mean, stddev);
            int Q2 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_QT_3, mean, stddev);
            int Q3 = int(stddev[0] * stddev[0]);
            cv::meanStdDev(Pixel_QT_4, mean, stddev);
            int Q4 = int(stddev[0] * stddev[0]);
            int Q_mean = (Q1 + Q2 + Q3 + Q4) / 4;
            int SCCD_QT = ((Q1 - Q_mean)*(Q1 - Q_mean) + (Q2 - Q_mean)*(Q2 - Q_mean) + (Q3 - Q_mean)*(Q3 - Q_mean) + (Q4 - Q_mean)*(Q4 - Q_mean)) / 4;


            features[0] = int(height);        
            features[1] = int(width);
            features[2] = depth_QT;
            features[3] = depth_MT;
            features[4] = int(G_H);          
            features[5] = int(G_V);
            features[6] = int(G_45);
            features[7] = int(G_135);
            features[8] = int(Gra);
            features[9] = int(*Gra_Max);
            features[10] = int(Var);          
            features[11] = int(NMSE);
            features[12] = NCC_max;        
            features[13] = NCC_min;
            features[14] = NCC_avg;
            features[15] = NCD_QT_max;
            features[16] = NCD_QT_min;
            features[17] = NCD_QT_avg;
            features[18] = NCD_MT_max;
            features[19] = NCD_MT_min;
            features[20] = NCD_MT_avg;
            features[21] = SCCD_BH;       
            features[22] = SCCD_BV;
            features[23] = SCCD_TH;
            features[24] = SCCD_TV;
            features[25] = SCCD_QT;


            vector<EncTestMode> partition_mode;
            if (features[10] < NCC_min)
            {
              features[26] = 0;     //simple
            }
            else if (features[10] > NCC_max)
            {
              features[26] = 2;    //complex
            }
            else
            {
              features[26] = 1;    //fuzzy
            }

            Py_SetPythonHome(L"C:\\Users\\hequan\\Miniconda3\\"); //The location of python environment
            Py_Initialize();                                      //Initialize the python environment
            if (!Py_IsInitialized())
            {
              cout << "Failed to initialization.";

            }

            PyRun_SimpleString("import sys");
            PyRun_SimpleString("sys.path.append('./')");
            PyObject * pModule = NULL;
            PyObject * pFunc = NULL;
            pModule = PyImport_ImportModule("TEST");// The name of python file
            if (pModule == NULL)
            {
              cout << "The python file is not founded" << endl;
            }

            pFunc = PyObject_GetAttrString(pModule, "GetPartition");//The name of python function
            PyObject* args = Py_BuildValue("(iiiiiiiiiiiiiiiiiiiiiiiiiii)", features[0], features[1], features[2], features[3], features[4], features[5], features[6], features[7], features[8]
              , features[9], features[10], features[11], features[12], features[13], features[14], features[15], features[16], features[17], features[18], features[19], features[20], features[21], features[22], features[23], features[24], features[25], 2);     
            PyObject* pRet = PyObject_CallObject(pFunc, args);

            PyArg_Parse(pRet, "i", &res);
            if (res <= -2 || res >= 6)
            {
              cout << "The result is wrong" << endl;
            }
            else
            {
              partition_mode.push_back(m_modeCtrl->currTestMode());
              bool Valid = false;
              if (res == 0)
              {
                partition_mode.back().type = ETM_INTRA;
              }
              else if (res == 1)
              {
                partition_mode.back().type = ETM_SPLIT_QT;
              }
              else if (res == 2)
              {
                partition_mode.back().type = ETM_SPLIT_BT_H;
              }
              else if (res == 3)
              {
                partition_mode.back().type = ETM_SPLIT_BT_V;
              }
              else if (res == 4)
              {
                partition_mode.back().type = ETM_SPLIT_TT_H;
              }
              else if (res == 5)
              {
                partition_mode.back().type = ETM_SPLIT_TT_V;
              }

              if (res != -1)
              {
                Valid = m_modeCtrl->tryModeMaster(partition_mode.back(), *tempCS, partitioner);
                if (features[26] >= 1)
                {
                  if (res == 0)
                  {
                    Valid = false;
                  }
                }
              }
              else
              {
                Valid = false;
              }

              if (Valid == true)
              {
                m_modeCtrl->ChangeTestMode(res);
              }
            }
          }
        }
      }
    }
  }
  finish = clock();
  if ((double)(finish - start) != 0)
  {
    double* time = new double[1];
    time[0] = (double)(finish - start + 1);
    fwrite(time, 8, 1, Time);
    delete[] time;
    fflush(Time);
    time_p += 1;
  }
#endif



  if (partitioner.currQtDepth == 0 && partitioner.currMtDepth == 0 && !tempCS->slice->isIntra() && (sps.getUseSBT() || sps.getUseInterMTS()))
  {
    auto slsSbt = dynamic_cast<SaveLoadEncInfoSbt*>(m_modeCtrl);
    int maxSLSize = sps.getUseSBT() ? tempCS->slice->getSPS()->getMaxSbtSize() : MTS_INTER_MAX_CU_SIZE;
    slsSbt->resetSaveloadSbt(maxSLSize);
#if ENABLE_SPLIT_PARALLELISM
    CHECK(tempCS->picture->scheduler.getSplitJobId() != 0, "The SBT search reset need to happen in sequential region.");
    if (m_pcEncCfg->getNumSplitThreads() > 1)
    {
      for (int jId = 1; jId < NUM_RESERVERD_SPLIT_JOBS; jId++)
      {
        auto slsSbt = dynamic_cast<SaveLoadEncInfoSbt *>(m_pcEncLib->getCuEncoder(jId)->m_modeCtrl);
        slsSbt->resetSaveloadSbt(maxSLSize);
      }
    }
#endif
  }
  m_sbtCostSave[0] = m_sbtCostSave[1] = MAX_DOUBLE;

  m_CurrCtx->start = m_CABACEstimator->getCtx();

  m_cuChromaQpOffsetIdxPlus1 = 0;

  if (slice.getUseChromaQpAdj())
  {
    // TODO M0133 : double check encoder decisions with respect to chroma QG detection and actual encode
    int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
      std::max<int>(0, sps.getLog2DiffMaxMinCodingBlockSize() - int(pps.getPpsRangeExtension().getCuChromaQpOffsetSubdiv() / 2));
    m_cuChromaQpOffsetIdxPlus1 = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pps.getPpsRangeExtension().getChromaQpOffsetListLen() + 1);
  }

  if (!m_modeCtrl->anyMode())
  {
    m_modeCtrl->finishCULevel(partitioner);
    return;
  }

  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cux", uiLPelX));
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cuy", uiTPelY));
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cuw", tempCS->area.lwidth()));
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cuh", tempCS->area.lheight()));
  DTRACE(g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight());


  int startShareThisLevel = 0;
  m_pcInterSearch->resetSavedAffineMotion();

#if JVET_O0057_ALTHPELIF
  double bestIntPelCost = MAX_DOUBLE;
#endif
  do
  {
#if JVET_O0119_BASE_PALETTE_444
    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
      tempCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
      memcpy(tempCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
    }
#endif
    EncTestMode currTestMode = m_modeCtrl->currTestMode();
#if JVET_O0502_ISP_CLEANUP
    currTestMode.maxCostAllowed = maxCostAllowed;
#endif

#if JVET_O0050_LOCAL_DUAL_TREE
    if (pps.getUseDQP() && partitioner.isSepTree(*tempCS) && isChroma(partitioner.chType))
#else
    if (pps.getUseDQP() && CS::isDualITree(*tempCS) && isChroma(partitioner.chType))
#endif
    {
      const Position chromaCentral(tempCS->area.Cb().chromaPos().offset(tempCS->area.Cb().chromaSize().width >> 1, tempCS->area.Cb().chromaSize().height >> 1));
      const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, tempCS->area.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, tempCS->area.chromaFormat));
      const CodingStructure* baseCS = bestCS->picture->cs;
      const CodingUnit* colLumaCu = baseCS->getCU(lumaRefPos, CHANNEL_TYPE_LUMA);

      if (colLumaCu)
      {
        currTestMode.qp = colLumaCu->qp;
      }
    }

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
    if (partitioner.currQgEnable() && (
#if SHARP_LUMA_DELTA_QP
    (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()) ||
#endif
#if ENABLE_QPA_SUB_CTU
    (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && pps.getUseDQP())
#else
      false
#endif
      ))
    {
#if ENABLE_SPLIT_PARALLELISM
      CHECK(tempCS->picture->scheduler.getSplitJobId() > 0, "Changing lambda is only allowed in the master thread!");
#endif
      if (currTestMode.qp >= 0)
      {
        updateLambda(&slice, currTestMode.qp,
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
          m_pcEncCfg->getWCGChromaQPControl().isEnabled(),
#endif
          CS::isDualITree(*tempCS) || (partitioner.currDepth == 0));
      }
    }
#endif

    if (currTestMode.type == ETM_INTER_ME)
    {
      if ((currTestMode.opts & ETO_IMV) != 0)
      {
#if JVET_O0057_ALTHPELIF
        const bool skipAltHpelIF = (int((currTestMode.opts & ETO_IMV) >> ETO_IMV_SHIFT) == 4) && (bestIntPelCost > 1.25 * bestCS->cost);
        if (!skipAltHpelIF)
        {
#endif
          tempCS->bestCS = bestCS;
#if JVET_O0057_ALTHPELIF
          xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode, bestIntPelCost);
#else
          xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode);
#endif
          tempCS->bestCS = nullptr;
#if JVET_O0057_ALTHPELIF
        }
#endif
      }
      else
      {
        tempCS->bestCS = bestCS;
        xCheckRDCostInter(tempCS, bestCS, partitioner, currTestMode);
        tempCS->bestCS = nullptr;
      }

    }
    else if (currTestMode.type == ETM_HASH_INTER)
    {
      xCheckRDCostHashInter(tempCS, bestCS, partitioner, currTestMode);
    }
    else if (currTestMode.type == ETM_AFFINE)
    {
      xCheckRDCostAffineMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
    }
#if REUSE_CU_RESULTS
    else if (currTestMode.type == ETM_RECO_CACHED)
    {
      xReuseCachedResult(tempCS, bestCS, partitioner);
    }
#endif
    else if (currTestMode.type == ETM_MERGE_SKIP)
    {
      xCheckRDCostMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
      CodingUnit* cu = bestCS->getCU(partitioner.chType);
      if (cu)
        cu->mmvdSkip = cu->skip == false ? false : cu->mmvdSkip;
    }
    else if (currTestMode.type == ETM_MERGE_TRIANGLE)
    {
      xCheckRDCostMergeTriangle2Nx2N(tempCS, bestCS, partitioner, currTestMode);
    }
    else if (currTestMode.type == ETM_INTRA)
    {
      xCheckRDCostIntra(tempCS, bestCS, partitioner, currTestMode);
    }
#if !JVET_O0525_REMOVE_PCM
    else if (currTestMode.type == ETM_IPCM)
    {
      xCheckIntraPCM(tempCS, bestCS, partitioner, currTestMode);
    }
#endif
#if JVET_O0119_BASE_PALETTE_444
    else if (currTestMode.type == ETM_PALETTE)
    {
      xCheckPLT(tempCS, bestCS, partitioner, currTestMode);
    }
#endif
    else if (currTestMode.type == ETM_IBC)
    {
      xCheckRDCostIBCMode(tempCS, bestCS, partitioner, currTestMode);
    }
    else if (currTestMode.type == ETM_IBC_MERGE)
    {
      xCheckRDCostIBCModeMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
    }
    else if (isModeSplit(currTestMode))
    {
#if JVET_O0119_BASE_PALETTE_444
      if (bestCS->cus.size() != 0)
      {
        splitmode = bestCS->cus[0]->splitSeries;
      }
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
      assert(partitioner.modeType == tempCS->modeType);
      int signalModeConsVal = tempCS->signalModeCons(getPartSplit(currTestMode), partitioner, modeTypeParent);
      int numRoundRdo = signalModeConsVal == LDT_MODE_TYPE_SIGNAL ? 2 : 1;
      bool skipInterPass = false;
      for (int i = 0; i < numRoundRdo; i++)
      {
        //change cons modes
        if (signalModeConsVal == LDT_MODE_TYPE_SIGNAL)
        {
          CHECK(numRoundRdo != 2, "numRoundRdo shall be 2 - [LDT_MODE_TYPE_SIGNAL]");
          tempCS->modeType = partitioner.modeType = (i == 0) ? MODE_TYPE_INTER : MODE_TYPE_INTRA;
        }
        else if (signalModeConsVal == LDT_MODE_TYPE_INFER)
        {
          CHECK(numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INFER]");
          tempCS->modeType = partitioner.modeType = MODE_TYPE_INTRA;
        }
        else if (signalModeConsVal == LDT_MODE_TYPE_INHERIT)
        {
          CHECK(numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INHERIT]");
          tempCS->modeType = partitioner.modeType = modeTypeParent;
        }

        //for lite intra encoding fast algorithm, set the status to save inter coding info
        if (modeTypeParent == MODE_TYPE_ALL && tempCS->modeType == MODE_TYPE_INTER)
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU(true);
          m_pcIntraSearch->setNumCuInSCIPU(0);
        }
        else if (modeTypeParent == MODE_TYPE_ALL && tempCS->modeType != MODE_TYPE_INTER)
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU(false);
          if (tempCS->modeType == MODE_TYPE_ALL)
          {
            m_pcIntraSearch->setNumCuInSCIPU(0);
          }
        }

        xCheckModeSplit(tempCS, bestCS, partitioner, currTestMode, modeTypeParent, skipInterPass);
#else
      xCheckModeSplit(tempCS, bestCS, partitioner, currTestMode);
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
      //recover cons modes
      tempCS->modeType = partitioner.modeType = modeTypeParent;
      tempCS->treeType = partitioner.treeType = treeTypeParent;
      partitioner.chType = chTypeParent;
      if (modeTypeParent == MODE_TYPE_ALL)
      {
        m_pcIntraSearch->setSaveCuCostInSCIPU(false);
        if (numRoundRdo == 2 && tempCS->modeType == MODE_TYPE_INTRA)
        {
          m_pcIntraSearch->initCuAreaCostInSCIPU();
        }
      }
      if (skipInterPass)
      {
        break;
      }
      }
#endif
#if JVET_O0119_BASE_PALETTE_444
    if (splitmode != bestCS->cus[0]->splitSeries)
    {
      splitmode = bestCS->cus[0]->splitSeries;
      const CodingUnit&     cu = *bestCS->cus.front();
      cu.cs->prevPLT = bestCS->prevPLT;
      for (int i = compBegin; i < (compBegin + numComp); i++)
      {
        ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
        bestLastPLTSize[comID] = bestCS->cus[0]->cs->prevPLT.curPLTSize[comID];
        memcpy(bestLastPLT[i], bestCS->cus[0]->cs->prevPLT.curPLT[i], bestCS->cus[0]->cs->prevPLT.curPLTSize[comID] * sizeof(Pel));
      }
    }
#endif
    }
  else
  {
    THROW("Don't know how to handle mode: type = " << currTestMode.type << ", options = " << currTestMode.opts);
  }
  } while (m_modeCtrl->nextMode(*tempCS, partitioner));

  if (startShareThisLevel == 1)
  {
    m_shareState = NO_SHARE;
    m_pcInterSearch->setShareState(m_shareState);
    setShareStateDec(m_shareState);
  }

  //////////////////////////////////////////////////////////////////////////
  // Finishing CU
#if ENABLE_SPLIT_PARALLELISM
  if (bestCS->cus.empty())
  {
    CHECK(bestCS->cost != MAX_DOUBLE, "Cost should be maximal if no encoding found");
    CHECK(bestCS->picture->scheduler.getSplitJobId() == 0, "Should always get a result in serial case");

    m_modeCtrl->finishCULevel(partitioner);
    return;
  }

#endif
  if (tempCS->cost == MAX_DOUBLE && bestCS->cost == MAX_DOUBLE)
  {
    //although some coding modes were planned to be tried in RDO, no coding mode actually finished encoding due to early termination
    //thus tempCS->cost and bestCS->cost are both MAX_DOUBLE; in this case, skip the following process for normal case
    m_modeCtrl->finishCULevel(partitioner);
    return;
  }

  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
#if JVET_O0050_LOCAL_DUAL_TREE
  //copy the qp of the last non-chroma CU
  int numCUInThisNode = (int)bestCS->cus.size();
  if (numCUInThisNode > 1 && bestCS->cus.back()->chType == CHANNEL_TYPE_CHROMA && !CS::isDualITree(*bestCS))
  {
    CHECK(bestCS->cus[numCUInThisNode - 2]->chType != CHANNEL_TYPE_LUMA, "wrong chType");
    bestCS->prevQP[partitioner.chType] = bestCS->cus[numCUInThisNode - 2]->qp;
  }
  else
  {
#endif
    bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;
#if JVET_O0050_LOCAL_DUAL_TREE
  }
#endif
  if ((!slice.isIntra() || slice.getSPS()->getIBCFlag())
    && partitioner.chType == CHANNEL_TYPE_LUMA
    && bestCS->cus.size() == 1 && (bestCS->cus.back()->predMode == MODE_INTER || bestCS->cus.back()->predMode == MODE_IBC)
    && bestCS->area.Y() == (*bestCS->cus.back()).Y()
    )
  {
    const CodingUnit&     cu = *bestCS->cus.front();
    const PredictionUnit& pu = *cu.firstPU;

#if JVET_O0078_SINGLE_HMVPLUT
    bool isShare = ((CU::isIBC(cu) && m_shareState == 2) ? true : false);
    if (!cu.affine && !cu.triangle && !isShare)
#else
    if (!cu.affine && !cu.triangle)
#endif
    {
      MotionInfo mi = pu.getMotionInfo();
      mi.GBiIdx = (mi.interDir == 3) ? cu.GBiIdx : GBI_DEFAULT;
      cu.cs->addMiToLut(CU::isIBC(cu) ? cu.cs->motionLut.lutIbc : cu.cs->motionLut.lut, mi);
    }
  }
  bestCS->picture->getPredBuf(currCsArea).copyFrom(bestCS->getPredBuf(currCsArea));
  bestCS->picture->getRecoBuf(currCsArea).copyFrom(bestCS->getRecoBuf(currCsArea));
  m_modeCtrl->finishCULevel(partitioner);
#if JVET_O0050_LOCAL_DUAL_TREE
  if (m_pcIntraSearch->getSaveCuCostInSCIPU() && bestCS->cus.size() == 1)
  {
    m_pcIntraSearch->saveCuAreaCostInSCIPU(Area(partitioner.currArea().lumaPos(), partitioner.currArea().lumaSize()), bestCS->cost);
  }
#endif

#if ENABLE_SPLIT_PARALLELISM
  if (tempCS->picture->scheduler.getSplitJobId() == 0 && m_pcEncCfg->getNumSplitThreads() != 1)
  {
    tempCS->picture->finishParallelPart(currCsArea);
  }

#endif
#if JVET_O0119_BASE_PALETTE_444
  if (bestCS->cus.size() == 1) // no partition
  {
    if (bestCS->cus[0]->predMode == MODE_PLT)
    {
      for (int i = compBegin; i < (compBegin + numComp); i++)
      {
        ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
        bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
        memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
      }
      bestCS->reorderPrevPLT(bestCS->prevPLT, bestCS->cus[0]->curPLTSize, bestCS->cus[0]->curPLT, bestCS->cus[0]->reuseflag, compBegin, numComp, jointPLT);
    }
    else
    {
      for (int i = compBegin; i < (compBegin + numComp); i++)
      {
        ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
        bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
        memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
      }
    }
  }
  else
  {
    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
      bestCS->prevPLT.curPLTSize[comID] = bestLastPLTSize[comID];
      memcpy(bestCS->prevPLT.curPLT[i], bestLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
    }
  }
  const CodingUnit&     cu = *bestCS->cus.front();
  cu.cs->prevPLT = bestCS->prevPLT;
#endif
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK(bestCS->cus.empty(), "No possible encoding found");
  CHECK(bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found");
  CHECK(bestCS->cost == MAX_DOUBLE, "No possible encoding found");
}

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
void EncCu::updateLambda(Slice* slice, const int dQP,
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
  const bool useWCGChromaControl,
#endif
  const bool updateRdCostLambda)
{
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
  if (useWCGChromaControl)
  {
    int    NumberBFrames = (m_pcEncCfg->getGOPSize() - 1);
    int    SHIFT_QP = 12;
    double dLambda_scale = 1.0 - Clip3(0.0, 0.5, 0.05*(double)(slice->getPic()->fieldPic ? NumberBFrames / 2 : NumberBFrames));

    int bitdepth_luma_qp_scale = 6
      * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
        - DISTORTION_PRECISION_ADJUSTMENT(slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
    double qp_temp = (double)dQP + bitdepth_luma_qp_scale - SHIFT_QP;

    double dQPFactor = m_pcEncCfg->getGOPEntry(m_pcSliceEncoder->getGopId()).m_QPFactor;

    if (slice->getSliceType() == I_SLICE)
    {
      if (m_pcEncCfg->getIntraQpFactor() >= 0.0 && m_pcEncCfg->getGOPEntry(m_pcSliceEncoder->getGopId()).m_sliceType != I_SLICE)
      {
        dQPFactor = m_pcEncCfg->getIntraQpFactor();
      }
      else
      {
        if (m_pcEncCfg->getLambdaFromQPEnable())
        {
          dQPFactor = 0.57;
        }
        else
        {
          dQPFactor = 0.57*dLambda_scale;
        }
      }
    }
    else if (m_pcEncCfg->getLambdaFromQPEnable())
    {
      dQPFactor = 0.57;
    }

    double dLambda = dQPFactor * pow(2.0, qp_temp / 3.0);
    int depth = slice->getDepth();

    if (!m_pcEncCfg->getLambdaFromQPEnable() && depth > 0)
    {
      int qp_temp_slice = slice->getSliceQp() + bitdepth_luma_qp_scale - SHIFT_QP; // avoid lambda  over adjustment,  use slice_qp here
      dLambda *= Clip3(2.00, 4.00, (qp_temp_slice / 6.0)); // (j == B_SLICE && p_cur_frm->layer != 0 )
    }
    if (!m_pcEncCfg->getUseHADME() && slice->getSliceType() != I_SLICE)
    {
      dLambda *= 0.95;
    }

    const int temporalId = m_pcEncCfg->getGOPEntry(m_pcSliceEncoder->getGopId()).m_temporalId;
    const std::vector<double> &intraLambdaModifiers = m_pcEncCfg->getIntraLambdaModifier();
    double lambdaModifier;
    if (slice->getSliceType() != I_SLICE || intraLambdaModifiers.empty())
    {
      lambdaModifier = m_pcEncCfg->getLambdaModifier(temporalId);
    }
    else
    {
      lambdaModifier = intraLambdaModifiers[(temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size() - 1)];
    }
    dLambda *= lambdaModifier;

    int qpBDoffset = slice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
    int iQP = Clip3(-qpBDoffset, MAX_QP, (int)floor((double)dQP + 0.5));
    m_pcSliceEncoder->setUpLambda(slice, dLambda, iQP);

    return;
  }
#endif
  int iQP = dQP;
  const double oldQP = (double)slice->getSliceQpBase();
#if ENABLE_QPA_SUB_CTU
  const double oldLambda = (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && slice->getPPS()->getUseDQP()) ? slice->getLambdas()[0] :
    m_pcSliceEncoder->calculateLambda(slice, m_pcSliceEncoder->getGopId(), slice->getDepth(), oldQP, oldQP, iQP);
#else
  const double oldLambda = m_pcSliceEncoder->calculateLambda(slice, m_pcSliceEncoder->getGopId(), slice->getDepth(), oldQP, oldQP, iQP);
#endif
  const double newLambda = oldLambda * pow(2.0, ((double)dQP - oldQP) / 3.0);
#if RDOQ_CHROMA_LAMBDA
  const double chromaLambda = newLambda / m_pcRdCost->getChromaWeight();
  const double lambdaArray[MAX_NUM_COMPONENT] = { newLambda, chromaLambda, chromaLambda };
  m_pcTrQuant->setLambdas(lambdaArray);
#else
  m_pcTrQuant->setLambda(newLambda);
#endif
  if (updateRdCostLambda)
  {
    m_pcRdCost->setLambda(newLambda, slice->getSPS()->getBitDepths());
  }
}
#endif // SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU

#if ENABLE_SPLIT_PARALLELISM
//#undef DEBUG_PARALLEL_TIMINGS
//#define DEBUG_PARALLEL_TIMINGS 1
void EncCu::xCompressCUParallel(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner)
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lwidth());
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lheight());

  Picture* picture = tempCS->picture;

  int numJobs = m_modeCtrl->getNumParallelJobs(*bestCS, partitioner);

  bool    jobUsed[NUM_RESERVERD_SPLIT_JOBS];
  std::fill(jobUsed, jobUsed + NUM_RESERVERD_SPLIT_JOBS, false);

  const UnitArea currArea = CS::getArea(*tempCS, partitioner.currArea(), partitioner.chType);
#if ENABLE_WPP_PARALLELISM
  const int      wppTId = picture->scheduler.getWppThreadId();
#endif
  const bool doParallel = !m_pcEncCfg->getForceSingleSplitThread();
#if _MSC_VER && ENABLE_WPP_PARALLELISM
#pragma omp parallel for schedule(dynamic,1) num_threads(NUM_SPLIT_THREADS_IF_MSVC) if(doParallel)
#else
  omp_set_num_threads(m_pcEncCfg->getNumSplitThreads());

#pragma omp parallel for schedule(dynamic,1) if(doParallel)
#endif
  for (int jId = 1; jId <= numJobs; jId++)
  {
    // thread start
#if ENABLE_WPP_PARALLELISM
    picture->scheduler.setWppThreadId(wppTId);
#endif
    picture->scheduler.setSplitThreadId();
    picture->scheduler.setSplitJobId(jId);

    Partitioner* jobPartitioner = PartitionerFactory::get(*tempCS->slice);
    EncCu*       jobCuEnc = m_pcEncLib->getCuEncoder(picture->scheduler.getSplitDataId(jId));
    auto*        jobBlkCache = dynamic_cast<CacheBlkInfoCtrl*>(jobCuEnc->m_modeCtrl);
#if REUSE_CU_RESULTS
    auto*        jobBestCache = dynamic_cast<BestEncInfoCache*>(jobCuEnc->m_modeCtrl);
#endif

    jobPartitioner->copyState(partitioner);
    jobCuEnc->copyState(this, *jobPartitioner, currArea, true);

    if (jobBlkCache) { jobBlkCache->tick(); }
#if REUSE_CU_RESULTS
    if (jobBestCache) { jobBestCache->tick(); }

#endif
    CodingStructure *&jobBest = jobCuEnc->m_pBestCS[wIdx][hIdx];
    CodingStructure *&jobTemp = jobCuEnc->m_pTempCS[wIdx][hIdx];

    jobUsed[jId] = true;

    jobCuEnc->xCompressCU(jobTemp, jobBest, *jobPartitioner);

    delete jobPartitioner;

    picture->scheduler.setSplitJobId(0);
    // thread stop
  }
  picture->scheduler.setSplitThreadId(0);

  int    bestJId = 0;
  double bestCost = bestCS->cost;
  for (int jId = 1; jId <= numJobs; jId++)
  {
    EncCu* jobCuEnc = m_pcEncLib->getCuEncoder(picture->scheduler.getSplitDataId(jId));

    if (jobUsed[jId] && jobCuEnc->m_pBestCS[wIdx][hIdx]->cost < bestCost)
    {
      bestCost = jobCuEnc->m_pBestCS[wIdx][hIdx]->cost;
      bestJId = jId;
    }
  }

  if (bestJId > 0)
  {
    copyState(m_pcEncLib->getCuEncoder(picture->scheduler.getSplitDataId(bestJId)), partitioner, currArea, false);
    m_CurrCtx->best = m_CABACEstimator->getCtx();

    tempCS = m_pTempCS[wIdx][hIdx];
    bestCS = m_pBestCS[wIdx][hIdx];
  }

  const int      bitDepthY = tempCS->sps->getBitDepth(CH_L);
  const UnitArea clipdArea = clipArea(currArea, *picture);

  CHECK(calcCheckSum(picture->getRecoBuf(clipdArea.Y()), bitDepthY) != calcCheckSum(bestCS->getRecoBuf(clipdArea.Y()), bitDepthY), "Data copied incorrectly!");

  picture->finishParallelPart(currArea);

  if (auto *blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl))
  {
    for (int jId = 1; jId <= numJobs; jId++)
    {
      if (!jobUsed[jId] || jId == bestJId) continue;

      auto *jobBlkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_pcEncLib->getCuEncoder(picture->scheduler.getSplitDataId(jId))->m_modeCtrl);
      CHECK(!jobBlkCache, "If own mode controller has blk info cache capability so should all other mode controllers!");
      blkCache->CacheBlkInfoCtrl::copyState(*jobBlkCache, partitioner.currArea());
    }

    blkCache->tick();
  }
#if REUSE_CU_RESULTS

  if (auto *blkCache = dynamic_cast<BestEncInfoCache*>(m_modeCtrl))
  {
    for (int jId = 1; jId <= numJobs; jId++)
    {
      if (!jobUsed[jId] || jId == bestJId) continue;

      auto *jobBlkCache = dynamic_cast<BestEncInfoCache*>(m_pcEncLib->getCuEncoder(picture->scheduler.getSplitDataId(jId))->m_modeCtrl);
      CHECK(!jobBlkCache, "If own mode controller has blk info cache capability so should all other mode controllers!");
      blkCache->BestEncInfoCache::copyState(*jobBlkCache, partitioner.currArea());
    }

    blkCache->tick();
  }
#endif
}

void EncCu::copyState(EncCu* other, Partitioner& partitioner, const UnitArea& currArea, const bool isDist)
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lwidth());
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lheight());

  if (isDist)
  {
    other->m_pBestCS[wIdx][hIdx]->initSubStructure(*m_pBestCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false);
    other->m_pTempCS[wIdx][hIdx]->initSubStructure(*m_pTempCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false);
  }
  else
  {
    CodingStructure* dst = m_pBestCS[wIdx][hIdx];
    const CodingStructure* src = other->m_pBestCS[wIdx][hIdx];
    bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
    bool keepPred = true;

    dst->useSubStructure(*src, partitioner.chType, currArea, keepPred, true, keepResi, keepResi);

    dst->cost = src->cost;
    dst->dist = src->dist;
    dst->fracBits = src->fracBits;
    dst->features = src->features;
  }

  if (isDist)
  {
    m_CurrCtx = m_CtxBuffer.data();
  }

  m_pcInterSearch->copyState(*other->m_pcInterSearch);
  m_modeCtrl->copyState(*other->m_modeCtrl, partitioner.currArea());
  m_pcRdCost->copyState(*other->m_pcRdCost);
  m_pcTrQuant->copyState(*other->m_pcTrQuant);
  if (m_pcEncCfg->getReshaper())
  {
    EncReshape *encReshapeThis = dynamic_cast<EncReshape*>(m_pcReshape);
    EncReshape *encReshapeOther = dynamic_cast<EncReshape*>(other->m_pcReshape);
    encReshapeThis->copyState(*encReshapeOther);
  }
  m_shareState = other->m_shareState;
  m_shareBndPosX = other->m_shareBndPosX;
  m_shareBndPosY = other->m_shareBndPosY;
  m_shareBndSizeW = other->m_shareBndSizeW;
  m_shareBndSizeH = other->m_shareBndSizeH;
  setShareStateDec(other->getShareStateDec());
  m_pcInterSearch->setShareState(other->m_pcInterSearch->getShareState());

  m_CABACEstimator->getCtx() = other->m_CABACEstimator->getCtx();
}
#endif

#if JVET_O0050_LOCAL_DUAL_TREE
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool &skipInterPass)
#else
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
#endif
{
  const int qp = encTestMode.qp;
  const Slice &slice = *tempCS->slice;
  const bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.
  const int oldPrevQp = tempCS->prevQP[partitioner.chType];
  const auto oldMotionLut = tempCS->motionLut;
#if ENABLE_QPA_SUB_CTU
  const PPS &pps = *tempCS->pps;
  const uint32_t currDepth = partitioner.currDepth;
#endif
#if JVET_O0119_BASE_PALETTE_444
  const auto oldPLT = tempCS->prevPLT;
#endif

  const PartSplit split = getPartSplit(encTestMode);
#if JVET_O0050_LOCAL_DUAL_TREE
  const ModeType modeTypeChild = partitioner.modeType;
#endif

  CHECK(split == CU_DONT_SPLIT, "No proper split provided!");

  tempCS->initStructData(qp, bIsLosslessMode);

  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  const TempCtx ctxStartSP(m_CtxCache, SubCtx(Ctx::SplitFlag, m_CABACEstimator->getCtx()));
  const TempCtx ctxStartQt(m_CtxCache, SubCtx(Ctx::SplitQtFlag, m_CABACEstimator->getCtx()));
  const TempCtx ctxStartHv(m_CtxCache, SubCtx(Ctx::SplitHvFlag, m_CABACEstimator->getCtx()));
  const TempCtx ctxStart12(m_CtxCache, SubCtx(Ctx::Split12Flag, m_CABACEstimator->getCtx()));
#if JVET_O0050_LOCAL_DUAL_TREE
  const TempCtx ctxStartMC(m_CtxCache, SubCtx(Ctx::ModeConsFlag, m_CABACEstimator->getCtx()));
#endif
  m_CABACEstimator->resetBits();

  m_CABACEstimator->split_cu_mode(split, *tempCS, partitioner);
#if JVET_O0050_LOCAL_DUAL_TREE
  m_CABACEstimator->mode_constraint(split, *tempCS, partitioner, modeTypeChild);
#endif

  const double factor = (tempCS->currQP[partitioner.chType] > 30 ? 1.1 : 1.075);
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
  if (!tempCS->useDbCost)
    CHECK(bestCS->costDbOffset != 0, "error");
  const double cost = m_pcRdCost->calcRdCost(uint64_t(m_CABACEstimator->getEstFracBits() + ((bestCS->fracBits) / factor)), Distortion(bestCS->dist / factor)) + bestCS->costDbOffset / factor;

  m_CABACEstimator->getCtx() = SubCtx(Ctx::SplitFlag, ctxStartSP);
  m_CABACEstimator->getCtx() = SubCtx(Ctx::SplitQtFlag, ctxStartQt);
  m_CABACEstimator->getCtx() = SubCtx(Ctx::SplitHvFlag, ctxStartHv);
  m_CABACEstimator->getCtx() = SubCtx(Ctx::Split12Flag, ctxStart12);
#if JVET_O0050_LOCAL_DUAL_TREE
  m_CABACEstimator->getCtx() = SubCtx(Ctx::ModeConsFlag, ctxStartMC);
#endif
  if (cost > bestCS->cost + bestCS->costDbOffset
#if ENABLE_QPA_SUB_CTU
    || (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && pps.getUseDQP() && (pps.getCuQpDeltaSubdiv() > 0) && (split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT) &&
    (currDepth == 0)) // force quad-split or no split at CTU level
#endif
    )
  {
    xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
    return;
  }

#if JVET_O0050_LOCAL_DUAL_TREE
  const bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA ? true : false;
  if (partitioner.treeType != TREE_D)
  {
    tempCS->treeType = TREE_L;
  }
  else
  {
    if (chromaNotSplit)
    {
      CHECK(partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma");
      tempCS->treeType = partitioner.treeType = TREE_L;
    }
    else
    {
      tempCS->treeType = partitioner.treeType = TREE_D;
    }
  }
#endif

  int startShareThisLevel = 0;
  const uint32_t uiLPelX = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY = tempCS->area.Y().lumaPos().y;

  int splitRatio = 1;
  CHECK(!(split == CU_QUAD_SPLIT || split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT
    || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT), "invalid split type");
  splitRatio = (split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT) ? 1 : 2;

  bool isOneChildSmall = ((tempCS->area.lwidth())*(tempCS->area.lheight()) >> splitRatio) < MRG_SHARELIST_SHARSIZE;

  if ((((tempCS->area.lwidth())*(tempCS->area.lheight())) > (MRG_SHARELIST_SHARSIZE * 1)))
  {
    m_shareState = NO_SHARE;
  }

  if (m_shareState == NO_SHARE)//init state
  {
    if (isOneChildSmall)
    {
      m_shareState = GEN_ON_SHARED_BOUND;//share start state
      startShareThisLevel = 1;
    }
  }
  if (m_shareState == GEN_ON_SHARED_BOUND && slice.getSPS()->getIBCFlag())
  {
#if !JVET_O0078_SINGLE_HMVPLUT
    tempCS->motionLut.lutShareIbc = tempCS->motionLut.lutIbc;
#endif
    m_shareBndPosX = uiLPelX;
    m_shareBndPosY = uiTPelY;
    m_shareBndSizeW = tempCS->area.lwidth();
    m_shareBndSizeH = tempCS->area.lheight();
    m_shareState = SHARING;
  }


  m_pcInterSearch->setShareState(m_shareState);
  setShareStateDec(m_shareState);

  partitioner.splitCurrArea(split, *tempCS);
  bool qgEnableChildren = partitioner.currQgEnable(); // QG possible at children level

  m_CurrCtx++;

  tempCS->getRecoBuf().fill(0);

  tempCS->getPredBuf().fill(0);
  AffineMVInfo tmpMVInfo;
  bool isAffMVInfoSaved;
  m_pcInterSearch->savePrevAffMVInfo(0, tmpMVInfo, isAffMVInfoSaved);
#if JVET_O0592_ENC_ME_IMP
  BlkUniMvInfo tmpUniMvInfo;
  bool         isUniMvInfoSaved = false;
  if (!tempCS->slice->isIntra())
  {
    m_pcInterSearch->savePrevUniMvInfo(tempCS->area.Y(), tmpUniMvInfo, isUniMvInfoSaved);
  }
#endif

  do
  {
    const auto &subCUArea = partitioner.currArea();

    if (tempCS->picture->Y().contains(subCUArea.lumaPos()))
    {
      const unsigned wIdx = gp_sizeIdxInfo->idxFrom(subCUArea.lwidth());
      const unsigned hIdx = gp_sizeIdxInfo->idxFrom(subCUArea.lheight());

      CodingStructure *tempSubCS = m_pTempCS[wIdx][hIdx];
      CodingStructure *bestSubCS = m_pBestCS[wIdx][hIdx];

      tempCS->initSubStructure(*tempSubCS, partitioner.chType, subCUArea, false);
      tempCS->initSubStructure(*bestSubCS, partitioner.chType, subCUArea, false);
      tempSubCS->sharedBndPos.x = (m_shareState == SHARING) ? m_shareBndPosX : tempSubCS->area.Y().lumaPos().x;
      tempSubCS->sharedBndPos.y = (m_shareState == SHARING) ? m_shareBndPosY : tempSubCS->area.Y().lumaPos().y;
      tempSubCS->sharedBndSize.width = (m_shareState == SHARING) ? m_shareBndSizeW : tempSubCS->area.lwidth();
      tempSubCS->sharedBndSize.height = (m_shareState == SHARING) ? m_shareBndSizeH : tempSubCS->area.lheight();
      bestSubCS->sharedBndPos.x = (m_shareState == SHARING) ? m_shareBndPosX : tempSubCS->area.Y().lumaPos().x;
      bestSubCS->sharedBndPos.y = (m_shareState == SHARING) ? m_shareBndPosY : tempSubCS->area.Y().lumaPos().y;
      bestSubCS->sharedBndSize.width = (m_shareState == SHARING) ? m_shareBndSizeW : tempSubCS->area.lwidth();
      bestSubCS->sharedBndSize.height = (m_shareState == SHARING) ? m_shareBndSizeH : tempSubCS->area.lheight();
#if JVET_O0070_PROF
      tempSubCS->bestParent = bestSubCS->bestParent = bestCS;
#endif
#if JVET_O0502_ISP_CLEANUP
      double newMaxCostAllowed = isLuma(partitioner.chType) ? std::min(encTestMode.maxCostAllowed, bestCS->cost - m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist)) : MAX_DOUBLE;
      newMaxCostAllowed = std::max(0.0, newMaxCostAllowed);
      xCompressCU(tempSubCS, bestSubCS, partitioner, newMaxCostAllowed);
#else
      xCompressCU(tempSubCS, bestSubCS, partitioner);
#endif
#if JVET_O0070_PROF
      tempSubCS->bestParent = bestSubCS->bestParent = nullptr;
#endif

      if (bestSubCS->cost == MAX_DOUBLE)
      {
        CHECK(split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split");
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
        m_CurrCtx--;
        partitioner.exitCurrSplit();
        xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
#if JVET_O0050_LOCAL_DUAL_TREE //early terminate
        if (partitioner.chType == CHANNEL_TYPE_LUMA)
        {
          tempCS->motionLut = oldMotionLut;
        }
        if (startShareThisLevel == 1)
        {
          m_shareState = NO_SHARE;
          m_pcInterSearch->setShareState(m_shareState);
          setShareStateDec(m_shareState);
        }
#endif
        return;
      }

      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      tempCS->useSubStructure(*bestSubCS, partitioner.chType, CS::getArea(*tempCS, subCUArea, partitioner.chType), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi);

      if (partitioner.currQgEnable())
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }
#if JVET_O0050_LOCAL_DUAL_TREE
      if (partitioner.isConsInter())
      {
        for (int i = 0; i < bestSubCS->cus.size(); i++)
        {
          CHECK(bestSubCS->cus[i]->predMode != MODE_INTER, "all CUs must be inter mode in an Inter coding region (SCIPU)");
        }
      }
      else if (partitioner.isConsIntra())
      {
        for (int i = 0; i < bestSubCS->cus.size(); i++)
        {
          CHECK(bestSubCS->cus[i]->predMode == MODE_INTER, "all CUs must not be inter mode in an Intra coding region (SCIPU)");
        }
      }
#endif

      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
#if JVET_O0050_LOCAL_DUAL_TREE //early terminate
      if (!tempCS->slice->isIntra() && partitioner.isConsIntra())
      {
        tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);
        if (tempCS->cost > bestCS->cost)
        {
          tempCS->cost = MAX_DOUBLE;
          tempCS->costDbOffset = 0;
          tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
          m_CurrCtx--;
          partitioner.exitCurrSplit();
          if (partitioner.chType == CHANNEL_TYPE_LUMA)
          {
            tempCS->motionLut = oldMotionLut;
          }
          if (startShareThisLevel == 1)
          {
            m_shareState = NO_SHARE;
            m_pcInterSearch->setShareState(m_shareState);
            setShareStateDec(m_shareState);
          }
          return;
        }
      }
#endif
    }
  } while (partitioner.nextPart(*tempCS));

  partitioner.exitCurrSplit();

  if (startShareThisLevel == 1)
  {
    m_shareState = NO_SHARE;
    m_pcInterSearch->setShareState(m_shareState);
    setShareStateDec(m_shareState);
  }

  m_CurrCtx--;

#if JVET_O0050_LOCAL_DUAL_TREE
  if (chromaNotSplit)
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    //Note: In local dual tree region, the chroma CU refers to the central luma CU's QP. 
    //If the luma CU QP shall be predQP (no residual in it and before it in the QG), it must be revised to predQP before encoding the chroma CU
    //Otherwise, the chroma CU uses predQP+deltaQP in encoding but is decoded as using predQP, thus causing encoder-decoded mismatch on chroma qp.
    if (tempCS->pps->getUseDQP())
    {
      //find parent CS that including all coded CUs in the QG before this node
      CodingStructure* qgCS = tempCS;
      bool deltaQpCodedBeforeThisNode = false;
      if (partitioner.currArea().lumaPos() != partitioner.currQgPos)
      {
        int numParentNodeToQgCS = 0;
        while (qgCS->area.lumaPos() != partitioner.currQgPos)
        {
          CHECK(qgCS->parent == nullptr, "parent of qgCS shall exsit");
          qgCS = qgCS->parent;
          numParentNodeToQgCS++;
        }

        //check whether deltaQP has been coded (in luma CU or luma&chroma CU) before this node
        CodingStructure* parentCS = tempCS->parent;
        for (int i = 0; i < numParentNodeToQgCS; i++)
        {
          //checking each parent
          CHECK(parentCS == nullptr, "parentCS shall exsit");
          for (const auto &cu : parentCS->cus)
          {
            if (cu->rootCbf && !isChroma(cu->chType))
            {
              deltaQpCodedBeforeThisNode = true;
              break;
            }
          }
          parentCS = parentCS->parent;
        }
      }

      //revise luma CU qp before the first luma CU with residual in the SCIPU to predQP
      if (!deltaQpCodedBeforeThisNode)
      {
        //get pred QP of the QG
        const CodingUnit* cuFirst = qgCS->getCU(CHANNEL_TYPE_LUMA);
        CHECK(cuFirst->lumaPos() != partitioner.currQgPos, "First cu of the Qg is wrong");
        int predQp = CU::predictQP(*cuFirst, qgCS->prevQP[CHANNEL_TYPE_LUMA]);

        //revise to predQP
        int firstCuHasResidual = (int)tempCS->cus.size();
        for (int i = 0; i < tempCS->cus.size(); i++)
        {
          if (tempCS->cus[i]->rootCbf)
          {
            firstCuHasResidual = i;
            break;
          }
        }

        for (int i = 0; i < firstCuHasResidual; i++)
        {
          tempCS->cus[i]->qp = predQp;
        }
      }
    }
#endif
    assert(tempCS->treeType == TREE_L);
    uint32_t numCuPuTu[6];
    tempCS->picture->cs->getNumCuPuTuOffset(numCuPuTu);
    tempCS->picture->cs->useSubStructure(*tempCS, partitioner.chType, CS::getArea(*tempCS, partitioner.currArea(), partitioner.chType), false, true, false, false);

    partitioner.chType = CHANNEL_TYPE_CHROMA;
    tempCS->treeType = partitioner.treeType = TREE_C;

    m_CurrCtx++;

    const unsigned wIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lwidth());
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lheight());
    CodingStructure *tempCSChroma = m_pTempCS2[wIdx][hIdx];
    CodingStructure *bestCSChroma = m_pBestCS2[wIdx][hIdx];
    tempCS->initSubStructure(*tempCSChroma, partitioner.chType, partitioner.currArea(), false);
    tempCS->initSubStructure(*bestCSChroma, partitioner.chType, partitioner.currArea(), false);
    tempCS->treeType = TREE_D;
    xCompressCU(tempCSChroma, bestCSChroma, partitioner);

    //attach chromaCS to luma CS and update cost
    bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
    //bestCSChroma->treeType = tempCSChroma->treeType = TREE_C;
    CHECK(bestCSChroma->treeType != TREE_C || tempCSChroma->treeType != TREE_C, "wrong treeType for chroma CS");
    tempCS->useSubStructure(*bestCSChroma, partitioner.chType, CS::getArea(*bestCSChroma, partitioner.currArea(), partitioner.chType), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, true);

    //release tmp resource
    tempCSChroma->releaseIntermediateData();
    bestCSChroma->releaseIntermediateData();
    //tempCS->picture->cs->releaseIntermediateData();
    tempCS->picture->cs->clearCuPuTuIdxMap(partitioner.currArea(), numCuPuTu[0], numCuPuTu[1], numCuPuTu[2], numCuPuTu + 3);

    m_CurrCtx--;

    //recover luma tree status
    partitioner.chType = CHANNEL_TYPE_LUMA;
    partitioner.treeType = TREE_D;
    partitioner.modeType = MODE_TYPE_ALL;
  }
#endif

  // Finally, generate split-signaling bits for RD-cost check
  const PartSplit implicitSplit = partitioner.getImplicitSplit(*tempCS);

  {
    bool enforceQT = implicitSplit == CU_QUAD_SPLIT;

    // LARGE CTU bug
    if (m_pcEncCfg->getUseFastLCTU())
    {
      unsigned minDepth = 0;
      unsigned maxDepth = floorLog2(tempCS->sps->getCTUSize()) - floorLog2(tempCS->sps->getMinQTSize(slice.getSliceType(), partitioner.chType));

      if (auto ad = dynamic_cast<AdaptiveDepthPartitioner*>(&partitioner))
      {
        ad->setMaxMinDepth(minDepth, maxDepth, *tempCS);
      }

      if (minDepth > partitioner.currQtDepth)
      {
        // enforce QT
        enforceQT = true;
      }
    }

    if (!enforceQT)
    {
      m_CABACEstimator->resetBits();

      m_CABACEstimator->split_cu_mode(split, *tempCS, partitioner);
#if JVET_O0050_LOCAL_DUAL_TREE
      partitioner.modeType = modeTypeParent;
      m_CABACEstimator->mode_constraint(split, *tempCS, partitioner, modeTypeChild);
#endif
      tempCS->fracBits += m_CABACEstimator->getEstFracBits(); // split bits
    }
  }

  tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  // Check Delta QP bits for splitted structure
  if (!qgEnableChildren) // check at deepest QG level only
    xCheckDQP(*tempCS, partitioner, true);

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE)
  {
    const BrickMap& tileMap = *tempCS->picture->brickMap;
    const uint32_t CtuAddr = CU::getCtuAddr(*bestCS->getCU(partitioner.chType));
    const bool isEndOfSlice = slice.getSliceMode() == FIXED_NUMBER_OF_BYTES
      && ((slice.getSliceBits() + CS::getEstBits(*bestCS)) > slice.getSliceArgument() << 3)
      && CtuAddr != tileMap.getCtuBsToRsAddrMap(slice.getSliceCurStartCtuTsAddr());

    if (isEndOfSlice)
    {
      bestCS->cost = MAX_DOUBLE;
      bestCS->costDbOffset = 0;
    }
  }
  else
  {
    bestCS->costDbOffset = 0;
  }
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
#if JVET_O0050_LOCAL_DUAL_TREE
  if (tempCS->cus.size() > 0 && modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTER)
  {
    int areaSizeNoResiCu = 0;
    for (int k = 0; k < tempCS->cus.size(); k++)
    {
      areaSizeNoResiCu += (tempCS->cus[k]->rootCbf == false) ? tempCS->cus[k]->lumaSize().area() : 0;
    }
    if (areaSizeNoResiCu >= (tempCS->area.lumaSize().area() >> 1))
    {
      skipInterPass = true;
    }
  }
#endif

  // RD check for sub partitioned coding structure.
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

  if (isAffMVInfoSaved)
    m_pcInterSearch->addAffMVInfo(tmpMVInfo);
#if JVET_O0592_ENC_ME_IMP
  if (!tempCS->slice->isIntra() && isUniMvInfoSaved)
  {
    m_pcInterSearch->addUniMvInfo(tmpUniMvInfo);
  }
#endif

  tempCS->motionLut = oldMotionLut;

#if JVET_O0119_BASE_PALETTE_444
  tempCS->prevPLT = oldPLT;
#endif

  tempCS->releaseIntermediateData();

  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}


void EncCu::xCheckRDCostIntra(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  double          bestInterCost = m_modeCtrl->getBestInterCost();
  double          costSize2Nx2NmtsFirstPass = m_modeCtrl->getMtsSize2Nx2NFirstPassCost();
  bool            skipSecondMtsPass = m_modeCtrl->getSkipSecondMTSPass();
  const SPS&      sps = *tempCS->sps;
  const int       maxSizeMTS = MTS_INTRA_MAX_CU_SIZE;
  uint8_t         considerMtsSecondPass = (sps.getUseIntraMTS() && isLuma(partitioner.chType) && partitioner.currArea().lwidth() <= maxSizeMTS && partitioner.currArea().lheight() <= maxSizeMTS) ? 1 : 0;
  const PPS &pps = *tempCS->pps;

  bool   useIntraSubPartitions = false;
  double maxCostAllowedForChroma = MAX_DOUBLE;
  const  CodingUnit *bestCU = bestCS->getCU(partitioner.chType);
  Distortion interHad = m_modeCtrl->getInterHad();


  double dct2Cost = MAX_DOUBLE;
  double trGrpBestCost[4] = { MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE };
  double globalBestCost = MAX_DOUBLE;
  bool   bestSelFlag[4] = { false, false, false, false };
  bool   trGrpCheck[4] = { true, true, true, true };
  int    startMTSIdx[4] = { 0, 1, 2, 3 };
  int    endMTSIdx[4] = { 0, 1, 2, 3 };
  double trGrpStopThreshold[3] = { 1.001, 1.001, 1.001 };
  int    bestMtsFlag = 0;
  int    bestLfnstIdx = 0;

#if JVET_O0213_RESTRICT_LFNST_TO_MAX_TB_SIZE
#if JVET_O0050_LOCAL_DUAL_TREE
  const int  maxLfnstIdx = (partitioner.isSepTree(*tempCS) && partitioner.chType == CHANNEL_TYPE_CHROMA && (partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8))
#else
  const int  maxLfnstIdx = (CS::isDualITree(*tempCS) && partitioner.chType == CHANNEL_TYPE_CHROMA && (partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8))
#endif
#if JVET_O0545_MAX_TB_SIGNALLING
    || (partitioner.currArea().lwidth() > sps.getMaxTbSize() || partitioner.currArea().lheight() > sps.getMaxTbSize()) ? 0 : 2;
#else
    || (partitioner.currArea().lwidth() > MAX_TB_SIZEY || partitioner.currArea().lheight() > MAX_TB_SIZEY) ? 0 : 2;
#endif
#else
#if JVET_O0050_LOCAL_DUAL_TREE
  const int  maxLfnstIdx = partitioner.isSepTree(*tempCS) && partitioner.chType == CHANNEL_TYPE_CHROMA && (partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8) ? 0 : 2;
#else
  const int  maxLfnstIdx = CS::isDualITree(*tempCS) && partitioner.chType == CHANNEL_TYPE_CHROMA && (partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8) ? 0 : 2;
#endif
#endif
  bool       skipOtherLfnst = false;
  int        startLfnstIdx = 0;
  int        endLfnstIdx = sps.getUseLFNST() ? maxLfnstIdx : 0;

  int grpNumMax = sps.getUseLFNST() ? 4 : 1;
  m_pcIntraSearch->invalidateBestModeCost();
  for (int trGrpIdx = 0; trGrpIdx < grpNumMax; trGrpIdx++)
  {
    const uint8_t startMtsFlag = trGrpIdx > 0;
    const uint8_t endMtsFlag = sps.getUseLFNST() ? considerMtsSecondPass : 0;

    if ((trGrpIdx == 0 || (!skipSecondMtsPass && considerMtsSecondPass)) && trGrpCheck[trGrpIdx])
    {
      for (int lfnstIdx = startLfnstIdx; lfnstIdx <= endLfnstIdx; lfnstIdx++)
      {
        for (uint8_t mtsFlag = startMtsFlag; mtsFlag <= endMtsFlag; mtsFlag++)
        {
#if JVET_O0368_LFNST_WITH_DCT2_ONLY
          if (mtsFlag > 0 && lfnstIdx > 0)
          {
            continue;
          }
#endif
          //3) if interHad is 0, only try further modes if some intra mode was already better than inter
          if (sps.getUseLFNST() && m_pcEncCfg->getUsePbIntraFast() && !tempCS->slice->isIntra() && bestCU && CU::isInter(*bestCS->getCU(partitioner.chType)) && interHad == 0)
          {
            continue;
          }

          tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

          CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

          partitioner.setCUData(cu);
          cu.slice = tempCS->slice;
          cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
          cu.skip = false;
          cu.mmvdSkip = false;
          cu.predMode = MODE_INTRA;
          cu.transQuantBypass = encTestMode.lossless;
          cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
          cu.qp = encTestMode.qp;
#if !JVET_O0525_REMOVE_PCM
          //cu.ipcm             = false;
#endif
          cu.lfnstIdx = lfnstIdx;
          cu.mtsFlag = mtsFlag;
          cu.ispMode = NOT_INTRA_SUBPARTITIONS;

          CU::addPUs(cu);

          tempCS->interHad = interHad;

          m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

          bool validCandRet = false;
          if (isLuma(partitioner.chType))
          {
#if JVET_O0502_ISP_CLEANUP
            //ISP uses the value of the best cost so far (luma if it is the fast version) to avoid test non-necessary subpartitions
#if JVET_O0050_LOCAL_DUAL_TREE
            double bestCostSoFar = partitioner.isSepTree(*tempCS) ? m_modeCtrl->getBestCostWithoutSplitFlags() : bestCU && bestCU->predMode == MODE_INTRA ? bestCS->lumaCost : bestCS->cost;
            if (partitioner.isSepTree(*tempCS) && encTestMode.maxCostAllowed < bestCostSoFar)
#else
            double bestCostSoFar = CS::isDualITree(*tempCS) ? m_modeCtrl->getBestCostWithoutSplitFlags() : bestCU && bestCU->predMode == MODE_INTRA ? bestCS->lumaCost : bestCS->cost;
            if (CS::isDualITree(*tempCS) && encTestMode.maxCostAllowed < bestCostSoFar)
#endif
            {
              bestCostSoFar = encTestMode.maxCostAllowed;
            }
#else
            //the Intra SubPartitions mode uses the value of the best cost so far (luma if it is the fast version) to avoid test non-necessary lines
#if JVET_O0050_LOCAL_DUAL_TREE
            const double bestCostSoFar = partitioner.isSepTree(*tempCS) ? m_modeCtrl->getBestCostWithoutSplitFlags() : bestCU && bestCU->predMode == MODE_INTRA ? bestCS->lumaCost : bestCS->cost;
#else
            const double bestCostSoFar = CS::isDualITree(*tempCS) ? m_modeCtrl->getBestCostWithoutSplitFlags() : bestCU && bestCU->predMode == MODE_INTRA ? bestCS->lumaCost : bestCS->cost;
#endif
#endif
            validCandRet = m_pcIntraSearch->estIntraPredLumaQT(cu, partitioner, bestCostSoFar, mtsFlag, startMTSIdx[trGrpIdx], endMTSIdx[trGrpIdx], (trGrpIdx > 0));
            if (sps.getUseLFNST() && (!validCandRet || (cu.ispMode && cu.firstTU->cbf[COMPONENT_Y] == 0)))
            {
              continue;
            }

            useIntraSubPartitions = cu.ispMode != NOT_INTRA_SUBPARTITIONS;
#if JVET_O0050_LOCAL_DUAL_TREE
            if (!partitioner.isSepTree(*tempCS))
#else
            if (!CS::isDualITree(*tempCS))
#endif
            {
              tempCS->lumaCost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);
              if (useIntraSubPartitions)
              {
                //the difference between the best cost so far and the current luma cost is stored to avoid testing the Cr component if the cost of luma + Cb is larger than the best cost
                maxCostAllowedForChroma = bestCS->cost < MAX_DOUBLE ? bestCS->cost - tempCS->lumaCost : MAX_DOUBLE;
              }
            }

            if (m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == std::numeric_limits<Distortion>::max()
              && tempCS->interHad == 0)
            {
              interHad = 0;
              // JEM assumes only perfect reconstructions can from now on beat the inter mode
              m_modeCtrl->enforceInterHad(0);
              continue;
            }

#if JVET_O0050_LOCAL_DUAL_TREE
            if (!partitioner.isSepTree(*tempCS))
#else
            if (!CS::isDualITree(*tempCS))
#endif
            {
              cu.cs->picture->getRecoBuf(cu.Y()).copyFrom(cu.cs->getRecoBuf(COMPONENT_Y));
              cu.cs->picture->getPredBuf(cu.Y()).copyFrom(cu.cs->getPredBuf(COMPONENT_Y));
            }
          }

#if JVET_O0050_LOCAL_DUAL_TREE
          if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA || !cu.isSepTree()))
#else
          if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA || !CS::isDualITree(*tempCS)))
#endif
          {
            TUIntraSubPartitioner subTuPartitioner(partitioner);
#if JVET_O0050_LOCAL_DUAL_TREE
            m_pcIntraSearch->estIntraPredChromaQT(cu, (!useIntraSubPartitions || (cu.isSepTree() && !isLuma(CHANNEL_TYPE_CHROMA))) ? partitioner : subTuPartitioner, maxCostAllowedForChroma);
#else
            m_pcIntraSearch->estIntraPredChromaQT(cu, (!useIntraSubPartitions || (CS::isDualITree(*cu.cs) && !isLuma(CHANNEL_TYPE_CHROMA))) ? partitioner : subTuPartitioner, maxCostAllowedForChroma);
#endif
            if (useIntraSubPartitions && !cu.ispMode)
            {
              //At this point the temp cost is larger than the best cost. Therefore, we can already skip the remaining calculations
              continue;
            }
          }

          cu.rootCbf = false;

          for (uint32_t t = 0; t < getNumberValidTBlocks(*cu.cs->pcv); t++)
          {
            cu.rootCbf |= cu.firstTU->cbf[t] != 0;
          }

          // Get total bits for current mode: encode CU
          m_CABACEstimator->resetBits();

          if (pps.getTransquantBypassEnabledFlag())
          {
            m_CABACEstimator->cu_transquant_bypass_flag(cu);
          }

          if ((!cu.cs->slice->isIntra() || cu.cs->slice->getSPS()->getIBCFlag())
            && cu.Y().valid()
            )
          {
            m_CABACEstimator->cu_skip_flag(cu);
          }
          m_CABACEstimator->pred_mode(cu);
#if !JVET_O0525_REMOVE_PCM
          m_CABACEstimator->pcm_data(cu, partitioner);
#endif
          m_CABACEstimator->cu_pred_data(cu);
          m_CABACEstimator->bdpcm_mode(cu, ComponentID(partitioner.chType));

          // Encode Coefficients
          CUCtx cuCtx;
          cuCtx.isDQPCoded = true;
          cuCtx.isChromaQpAdjCoded = true;
          m_CABACEstimator->cu_residual(cu, partitioner, cuCtx);

          tempCS->fracBits = m_CABACEstimator->getEstFracBits();
          tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

#if JVET_O0050_LOCAL_DUAL_TREE
          double bestIspCost = cu.ispMode ? cu.isSepTree() ? tempCS->cost : tempCS->lumaCost : MAX_DOUBLE;
#else
          double bestIspCost = cu.ispMode ? CS::isDualITree(*tempCS) ? tempCS->cost : tempCS->lumaCost : MAX_DOUBLE;
#endif

          const double tmpCostWithoutSplitFlags = tempCS->cost;
          xEncodeDontSplit(*tempCS, partitioner);

          xCheckDQP(*tempCS, partitioner);

          // Check if low frequency non-separable transform (LFNST) is too expensive
#if JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
          if (lfnstIdx && !cuCtx.lfnstLastScanPos)
          {
#if JVET_O0050_LOCAL_DUAL_TREE
            bool cbfAtZeroDepth = cu.isSepTree() ? cu.rootCbf : std::min(cu.firstTU->blocks[1].width, cu.firstTU->blocks[1].height) < 4 ? TU::getCbfAtDepth(*cu.firstTU, COMPONENT_Y, 0) : cu.rootCbf;
#else
            bool cbfAtZeroDepth = CS::isDualITree(*tempCS) ? cu.rootCbf : std::min(cu.firstTU->blocks[1].width, cu.firstTU->blocks[1].height) < 4 ? TU::getCbfAtDepth(*cu.firstTU, COMPONENT_Y, 0) : cu.rootCbf;
#endif
            if (cbfAtZeroDepth)
            {
              tempCS->cost = MAX_DOUBLE;
            }
          }
#else
#if JVET_O0050_LOCAL_DUAL_TREE
          const int nonZeroCoeffThr = cu.isSepTree() ? (isLuma(partitioner.chType) ? LFNST_SIG_NZ_LUMA : LFNST_SIG_NZ_CHROMA) : LFNST_SIG_NZ_LUMA + LFNST_SIG_NZ_CHROMA;
#else
          const int nonZeroCoeffThr = CS::isDualITree(*tempCS) ? (isLuma(partitioner.chType) ? LFNST_SIG_NZ_LUMA : LFNST_SIG_NZ_CHROMA) : LFNST_SIG_NZ_LUMA + LFNST_SIG_NZ_CHROMA;
#endif
          if (lfnstIdx && cuCtx.numNonZeroCoeffNonTs <= nonZeroCoeffThr)
          {
            if (cuCtx.numNonZeroCoeffNonTs > 0)
            {
              tempCS->cost = MAX_DOUBLE;
            }
          }
#endif

          if (mtsFlag == 0 && lfnstIdx == 0)
          {
            dct2Cost = tempCS->cost;
          }

          if (tempCS->cost < bestCS->cost)
          {
            m_modeCtrl->setBestCostWithoutSplitFlags(tmpCostWithoutSplitFlags);
          }

          if (!mtsFlag) static_cast<double&>(costSize2Nx2NmtsFirstPass) = tempCS->cost;

          if (sps.getUseLFNST() && !tempCS->cus.empty())
          {
            skipOtherLfnst = m_modeCtrl->checkSkipOtherLfnst(encTestMode, tempCS, partitioner);
          }

          xCalDebCost(*tempCS, partitioner);
          tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();


#if WCG_EXT
          DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
          DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
          if (!sps.getUseLFNST())
          {
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
          }
          else
          {
            if (xCheckBestMode(tempCS, bestCS, partitioner, encTestMode))
            {
              trGrpBestCost[trGrpIdx] = globalBestCost = bestCS->cost;
              bestSelFlag[trGrpIdx] = true;
              bestMtsFlag = mtsFlag;
              bestLfnstIdx = lfnstIdx;
              if (bestCS->cus.size() == 1)
              {
                CodingUnit &cu = *bestCS->cus.front();
                if (cu.firstTU->mtsIdx == MTS_SKIP)
                {
                  if ((floorLog2(cu.firstTU->blocks[COMPONENT_Y].width) + floorLog2(cu.firstTU->blocks[COMPONENT_Y].height)) >= 6)
                  {
                    endLfnstIdx = 0;
                  }
                }
              }
            }

#if JVET_O0502_ISP_CLEANUP
            //we decide to skip the non-DCT-II transforms and LFNST according to the ISP results
            if ((endMtsFlag > 0 || endLfnstIdx > 0) && cu.ispMode && !mtsFlag && !lfnstIdx && tempCS->slice->isIntra() && m_pcEncCfg->getUseFastISP())
#else
            //we decide to skip the second emt pass or not according to the ISP results
            if (considerMtsSecondPass && cu.ispMode && !mtsFlag && tempCS->slice->isIntra())
#endif
            {
              double bestCostDct2NoIsp = m_modeCtrl->getMtsFirstPassNoIspCost();
              CHECKD(bestCostDct2NoIsp <= bestIspCost, "wrong cost!");
#if JVET_O0502_ISP_CLEANUP
              double threshold = 1.4;
#else
              double nSamples = (double)(cu.lwidth() << floorLog2(cu.lheight()));
              double threshold = 1 + 1.4 / sqrt(nSamples);
#endif

              double lfnstThreshold = 1.01 * threshold;
              if (bestCostDct2NoIsp > bestIspCost*lfnstThreshold)
              {
                endLfnstIdx = lfnstIdx;
              }

              if (bestCostDct2NoIsp > bestIspCost*threshold)
              {
                skipSecondMtsPass = true;
                m_modeCtrl->setSkipSecondMTSPass(true);
                break;
              }
            }
            //now we check whether the second pass of SIZE_2Nx2N and the whole Intra SIZE_NxN should be skipped or not
            if (!mtsFlag && !tempCS->slice->isIntra() && bestCU && bestCU->predMode != MODE_INTRA)
            {
              const double thEmtInterFastSkipIntra = 1.4; // Skip checking Intra if "2Nx2N using DCT2" is worse than best Inter mode
              if (costSize2Nx2NmtsFirstPass > thEmtInterFastSkipIntra * bestInterCost)
              {
                skipSecondMtsPass = true;
                m_modeCtrl->setSkipSecondMTSPass(true);
                break;
              }
            }
          }

        } //for emtCuFlag
        if (skipOtherLfnst)
        {
          startLfnstIdx = lfnstIdx;
          endLfnstIdx = lfnstIdx;
          break;
        }
      } //for lfnstIdx
    } //if (!skipSecondMtsPass && considerMtsSecondPass && trGrpCheck[iGrpIdx])

    if (sps.getUseLFNST() && trGrpIdx < 3)
    {
      trGrpCheck[trGrpIdx + 1] = false;

      if (bestSelFlag[trGrpIdx] && considerMtsSecondPass)
      {
        double dCostRatio = dct2Cost / trGrpBestCost[trGrpIdx];
        trGrpCheck[trGrpIdx + 1] = (bestMtsFlag != 0 || bestLfnstIdx != 0) && dCostRatio < trGrpStopThreshold[trGrpIdx];
      }
    }
  } //trGrpIdx
}

#if !JVET_O0525_REMOVE_PCM
void EncCu::xCheckIntraPCM(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
  cu.skip = false;
  cu.mmvdSkip = false;
  cu.predMode = MODE_INTRA;
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  cu.ipcm = true;
  cu.bdpcmMode = 0;

  tempCS->addPU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

  tempCS->addTU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

  m_pcIntraSearch->IPCMSearch(*tempCS, partitioner);

  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  m_CABACEstimator->resetBits();

  if (tempCS->pps->getTransquantBypassEnabledFlag())
  {
    m_CABACEstimator->cu_transquant_bypass_flag(cu);
  }

  if ((!cu.cs->slice->isIntra() || cu.cs->slice->getSPS()->getIBCFlag())
    && cu.Y().valid()
    )
  {
    m_CABACEstimator->cu_skip_flag(cu);
  }
  m_CABACEstimator->pred_mode(cu);
  m_CABACEstimator->pcm_data(cu, partitioner);


  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit(*tempCS, partitioner);

  xCheckDQP(*tempCS, partitioner);

  xCalDebCost(*tempCS, partitioner);
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();

#if WCG_EXT
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
}
#endif

#if JVET_O0119_BASE_PALETTE_444
void EncCu::xCheckPLT(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
  cu.skip = false;
  cu.mmvdSkip = false;
  cu.predMode = MODE_PLT;

  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
#if !JVET_O0525_REMOVE_PCM
  cu.ipcm = false;
#endif
  cu.bdpcmMode = 0;

  tempCS->addPU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  tempCS->addTU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  // Search
  tempCS->dist = 0;
#if JVET_O0050_LOCAL_DUAL_TREE
  if (cu.isSepTree())
#else
  if (CS::isDualITree(*tempCS))
#endif
  {
    if (isLuma(partitioner.chType))
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 1);
    }
    if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Cb, 2);
    }
  }
  else
  {
    m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 3);
  }


  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CABACEstimator->resetBits();
  if (tempCS->pps->getTransquantBypassEnabledFlag())
  {
    m_CABACEstimator->cu_transquant_bypass_flag(cu);
  }

  if ((!cu.cs->slice->isIntra() || cu.cs->slice->getSPS()->getIBCFlag())
    && cu.Y().valid())
  {
    m_CABACEstimator->cu_skip_flag(cu);
  }
  m_CABACEstimator->pred_mode(cu);

  // signaling
  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
#if JVET_O0050_LOCAL_DUAL_TREE
  if (cu.isSepTree())
#else
  if (CS::isDualITree(*tempCS))
#endif
  {
    if (isLuma(partitioner.chType))
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
    }
    if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
    }
  }
  else
  {
    m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
  }
  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit(*tempCS, partitioner);
  xCheckDQP(*tempCS, partitioner);
  xCalDebCost(*tempCS, partitioner);
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();

  const Area currCuArea = cu.block(getFirstComponentOfChannel(partitioner.chType));
  cu.slice->m_mapPltCost[currCuArea.pos()][currCuArea.size()] = tempCS->cost;
#if WCG_EXT
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
}
#endif

void EncCu::xCheckDQP(CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx)
{
  CHECK(bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit(cs) == CU_DONT_SPLIT, "bKeepCtx should only be set in split case");
  CHECK(!bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case");

  if (!cs.pps->getUseDQP())
  {
    return;
  }

#if JVET_O0050_LOCAL_DUAL_TREE
  if (partitioner.isSepTree(cs) && isChroma(partitioner.chType))
#else
  if (CS::isDualITree(cs) && isChroma(partitioner.chType))
#endif
  {
    return;
  }

  if (!partitioner.currQgEnable()) // do not consider split or leaf/not leaf QG condition (checked by caller)
  {
    return;
  }


  CodingUnit* cuFirst = cs.getCU(partitioner.chType);

  CHECK(!cuFirst, "No CU available");

  bool hasResidual = false;
  for (const auto &cu : cs.cus)
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
    if (cu->rootCbf && !isChroma(cu->chType))
#else
    if (cu->rootCbf)
#endif
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP(*cuFirst, cs.prevQP[partitioner.chType]);

  if (hasResidual)
  {
    TempCtx ctxTemp(m_CtxCache);
    if (!bKeepCtx) ctxTemp = SubCtx(Ctx::DeltaQP, m_CABACEstimator->getCtx());

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta(*cuFirst, predQP, cuFirst->qp);

    cs.fracBits += m_CABACEstimator->getEstFracBits(); // dQP bits
    cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);


    if (!bKeepCtx) m_CABACEstimator->getCtx() = SubCtx(Ctx::DeltaQP, ctxTemp);

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for (const auto &cu : cs.cus)
    {
#if JVET_O0050_LOCAL_DUAL_TREE
      //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
      if (cu->rootCbf && !isChroma(cu->chType))
#else
      if (cu->rootCbf)
#endif
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for (const auto &cu : cs.cus)
    {
      cu->qp = predQP;
    }
  }
}

void EncCu::xFillPCMBuffer(CodingUnit &cu)
{
  const ChromaFormat format = cu.chromaFormat;
  const uint32_t numberValidComponents = getNumberValidComponents(format);

  for (auto &tu : CU::traverseTUs(cu))
  {
    for (uint32_t ch = 0; ch < numberValidComponents; ch++)
    {
      const ComponentID compID = ComponentID(ch);

      const CompArea &compArea = tu.blocks[compID];

      const CPelBuf source = tu.cs->getOrgBuf(compArea);
      PelBuf destination = tu.getPcmbuf(compID);
      if (tu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
      {
        CompArea    tmpArea(COMPONENT_Y, compArea.chromaFormat, Position(0, 0), compArea.size());
        PelBuf tempOrgBuf = m_tmpStorageLCU->getBuf(tmpArea);
        tempOrgBuf.copyFrom(source);
        tempOrgBuf.rspSignal(m_pcReshape->getFwdLUT());
        destination.copyFrom(tempOrgBuf);
      }
      else
        destination.copyFrom(source);
    }
  }
}
void EncCu::xCheckRDCostHashInter(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  bool isPerfectMatch = false;

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  m_pcInterSearch->resetBufferedUniMotions();
  m_pcInterSearch->setAffineModeSelected(false);
  CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.skip = false;
  cu.predMode = MODE_INTER;
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  CU::addPUs(cu);
  cu.mmvdSkip = false;
  cu.firstPU->mmvdMergeFlag = false;

  if (m_pcInterSearch->predInterHashSearch(cu, partitioner, isPerfectMatch))
  {
    double equGBiCost = MAX_DOUBLE;

    m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
      , 0
      , &equGBiCost
    );

    if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
    {
      xCalDebCost(*bestCS, partitioner);
    }
  }
  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  int minSize = min(cu.lwidth(), cu.lheight());
  if (minSize < 64)
  {
    isPerfectMatch = false;
  }
  m_modeCtrl->setIsHashPerfectMatch(isPerfectMatch);
}

void EncCu::xCheckRDCostMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  const Slice &slice = *tempCS->slice;

  CHECK(slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices");

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;

  if (sps.getSBTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
  }

  Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS];
  setMergeBestSATDCost(MAX_DOUBLE);

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());

    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    pu.shareParentPos = tempCS->sharedBndPos;
    pu.shareParentSize = tempCS->sharedBndSize;
    PU::getInterMergeCandidates(pu, mergeCtx
      , 0
    );
    PU::getInterMMVDMergeCandidates(pu, mergeCtx);
    pu.regularMergeFlag = true;
  }
  bool candHasNoResidual[MRG_MAX_NUM_CANDS + MMVD_ADD_NUM];
  for (uint32_t ui = 0; ui < MRG_MAX_NUM_CANDS + MMVD_ADD_NUM; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip = false;
  bool                                        bestIsMMVDSkip = true;
  PelUnitBuf                                  acMergeBuffer[MRG_MAX_NUM_CANDS];
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
  PelUnitBuf                                  acMergeTmpBuffer[MRG_MAX_NUM_CANDS];
#endif
  PelUnitBuf                                  acMergeRealBuffer[MMVD_MRG_MAX_RD_BUF_NUM];
  PelUnitBuf *                                acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM];
  PelUnitBuf *                                singleMergeTempBuffer;
  int                                         insertPos;
  unsigned                                    uiNumMrgSATDCand = mergeCtx.numValidMergeCand + MMVD_ADD_NUM;

  struct ModeInfo
  {
    uint32_t mergeCand;
    bool     isRegularMerge;
    bool     isMMVD;
    bool     isCIIP;
    ModeInfo() : mergeCand(0), isRegularMerge(false), isMMVD(false), isCIIP(false) {}
    ModeInfo(const uint32_t mergeCand, const bool isRegularMerge, const bool isMMVD, const bool isCIIP) :
      mergeCand(mergeCand), isRegularMerge(isRegularMerge), isMMVD(isMMVD), isCIIP(isCIIP) {}
  };

  static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  RdModeList;
  bool                                        mrgTempBufSet = false;

  for (int i = 0; i < MRG_MAX_NUM_CANDS + MMVD_ADD_NUM; i++)
  {
    if (i < mergeCtx.numValidMergeCand)
    {
      RdModeList.push_back(ModeInfo(i, true, false, false));
    }
    else
    {
      RdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false));
    }
  }

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  for (unsigned i = 0; i < MMVD_MRG_MAX_RD_BUF_NUM; i++)
  {
    acMergeRealBuffer[i] = m_acMergeBuffer[i].getBuf(localUnitArea);
    if (i < MMVD_MRG_MAX_RD_NUM)
    {
      acMergeTempBuffer[i] = acMergeRealBuffer + i;
    }
    else
    {
      singleMergeTempBuffer = acMergeRealBuffer + i;
    }
  }

  bool isIntrainterEnabled = sps.getUseMHIntra();
  if (bestCS->area.lwidth() * bestCS->area.lheight() < 64 || bestCS->area.lwidth() >= MAX_CU_SIZE || bestCS->area.lheight() >= MAX_CU_SIZE)
  {
    isIntrainterEnabled = false;
  }
  bool isTestSkipMerge[MRG_MAX_NUM_CANDS]; // record if the merge candidate has tried skip mode
  for (uint32_t idx = 0; idx < MRG_MAX_NUM_CANDS; idx++)
  {
    isTestSkipMerge[idx] = false;
  }
  if (m_pcEncCfg->getUseFastMerge() || isIntrainterEnabled)
  {
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;
    if (isIntrainterEnabled)
    {
      uiNumMrgSATDCand += 1;
    }
    bestIsSkip = false;

    if (auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl))
    {
      if (slice.getSPS()->getIBCFlag())
      {
        ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
        bestIsSkip = blkCache->isSkip(tempCS->area) && cuECtx.bestCU;
      }
      else
        bestIsSkip = blkCache->isSkip(tempCS->area);
      bestIsMMVDSkip = blkCache->isMMVDSkip(tempCS->area);
    }

    if (isIntrainterEnabled) // always perform low complexity check
    {
      bestIsSkip = false;
    }

    static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if (!bestIsSkip)
    {
      RdModeList.clear();
      mrgTempBufSet = true;
      const TempCtx ctxStart(m_CtxCache, m_CABACEstimator->getCtx());

      CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);
      const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda(cu.transQuantBypass) * FRAC_BITS_SCALE;
      partitioner.setCUData(cu);
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
      cu.skip = false;
      cu.mmvdSkip = false;
      cu.triangle = false;
      //cu.affine
      cu.predMode = MODE_INTER;
      //cu.LICFlag
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      //cu.emtFlag  is set below

      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

      DistParam distParam;
      const bool bUseHadamard = !encTestMode.lossless && !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
      for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++)
      {
        mergeCtx.setMergeInfo(pu, uiMergeCand);

        PU::spanMotionInfo(pu, mergeCtx);
        pu.mvRefine = true;
        distParam.cur = singleMergeTempBuffer->Y();
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
        acMergeTmpBuffer[uiMergeCand] = m_acMergeTmpBuffer[uiMergeCand].getBuf(localUnitArea);
        m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true, &(acMergeTmpBuffer[uiMergeCand]));
#else
        m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
#endif
        acMergeBuffer[uiMergeCand] = m_acRealMergeBuffer[uiMergeCand].getBuf(localUnitArea);
        acMergeBuffer[uiMergeCand].copyFrom(*singleMergeTempBuffer);
        pu.mvRefine = false;
        if (mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_DEFAULT_N)
        {
          mergeCtx.mvFieldNeighbours[2 * uiMergeCand].mv = pu.mv[0];
          mergeCtx.mvFieldNeighbours[2 * uiMergeCand + 1].mv = pu.mv[1];
          {
            int dx, dy, i, j, num = 0;
            dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
            dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
            if (PU::checkDMVRCondition(pu))
            {
              for (i = 0; i < (pu.lumaSize().height); i += dy)
              {
                for (j = 0; j < (pu.lumaSize().width); j += dx)
                {
                  refinedMvdL0[num][uiMergeCand] = pu.mvdL0SubPu[num];
                  num++;
                }
              }
            }
          }
        }

        Distortion uiSad = distParam.distFunc(distParam);
        m_CABACEstimator->getCtx() = ctxStart;
        uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
        double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
        insertPos = -1;
        updateCandList(ModeInfo(uiMergeCand, true, false, false), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
        if (insertPos != -1)
        {
          if (insertPos == RdModeList.size() - 1)
          {
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
          else
          {
            for (uint32_t i = uint32_t(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
        CHECK(std::min(uiMergeCand + 1, uiNumMrgSATDCand) != RdModeList.size(), "");
      }

      if (isIntrainterEnabled)
      {
        // prepare for Intra bits calculation
        pu.mhIntraFlag = true;

        // save the to-be-tested merge candidates
        uint32_t MHIntraMergeCand[NUM_MRG_SATD_CAND];
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand); mergeCnt++)
        {
          MHIntraMergeCand[mergeCnt] = RdModeList[mergeCnt].mergeCand;
        }
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand), 4); mergeCnt++)
        {
          uint32_t mergeCand = MHIntraMergeCand[mergeCnt];
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
          acMergeTmpBuffer[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);
#else
          acMergeBuffer[mergeCand] = m_acRealMergeBuffer[mergeCand].getBuf(localUnitArea);
#endif

          // estimate merge bits
          mergeCtx.setMergeInfo(pu, mergeCand);

          // first round
          pu.intraDir[0] = PLANAR_IDX;
          uint32_t intraCnt = 0;
          // generate intrainter Y prediction
          if (mergeCnt == 0)
          {
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
            m_pcIntraSearch->predIntraAng(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));
          }
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
          pu.cs->getPredBuf(pu).copyFrom(acMergeTmpBuffer[mergeCand]);
#else
          pu.cs->getPredBuf(pu).copyFrom(acMergeBuffer[mergeCand]);
#endif
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getFwdLUT());
          }
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));

          // calculate cost
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getInvLUT());
          }
          distParam.cur = pu.cs->getPredBuf(pu).Y();
          Distortion sadValue = distParam.distFunc(distParam);
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getFwdLUT());
          }
          m_CABACEstimator->getCtx() = ctxStart;
#if JVET_O0249_MERGE_SYNTAX
          pu.regularMergeFlag = false;
#endif
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
          double cost = (double)sadValue + (double)fracBits * sqrtLambdaForFirstPassIntra;
          insertPos = -1;
          updateCandList(ModeInfo(mergeCand, false, false, true), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
          if (insertPos != -1)
          {
            for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
        pu.mhIntraFlag = false;
      }
      if (pu.cs->sps->getUseMMVD())
      {
        cu.mmvdSkip = true;
#if JVET_O0249_MERGE_SYNTAX
        pu.regularMergeFlag = true;
#endif
        const int tempNum = (mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1;
        for (int mmvdMergeCand = 0; mmvdMergeCand < tempNum; mmvdMergeCand++)
        {
          int baseIdx = mmvdMergeCand / MMVD_MAX_REFINE_NUM;
          int refineStep = (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM)) / 4;
          if (refineStep >= m_pcEncCfg->getMmvdDisNum())
            continue;
          mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);

          PU::spanMotionInfo(pu, mergeCtx);
          pu.mvRefine = true;
          distParam.cur = singleMergeTempBuffer->Y();
          pu.mmvdEncOptMode = (refineStep > 2 ? 2 : 1);
          CHECK(!pu.mmvdMergeFlag, "MMVD merge should be set");
          // Don't do chroma MC here
          m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
          pu.mmvdEncOptMode = 0;
          pu.mvRefine = false;
          Distortion uiSad = distParam.distFunc(distParam);

          m_CABACEstimator->getCtx() = ctxStart;
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
          double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
          insertPos = -1;
          updateCandList(ModeInfo(mmvdMergeCand, false, true, false), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
          if (insertPos != -1)
          {
            for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
      }
      // Try to limit number of candidates using SATD-costs
      for (uint32_t i = 1; i < uiNumMrgSATDCand; i++)
      {
        if (candCostList[i] > MRG_FAST_RATIO * candCostList[0])
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      setMergeBestSATDCost(candCostList[0]);

      if (isIntrainterEnabled)
      {
        pu.mhIntraFlag = true;
        for (uint32_t mergeCnt = 0; mergeCnt < uiNumMrgSATDCand; mergeCnt++)
        {
          if (RdModeList[mergeCnt].isCIIP)
          {
            pu.intraDir[0] = PLANAR_IDX;
            pu.intraDir[1] = DM_CHROMA_IDX;
            uint32_t bufIdx = 0;
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
            m_pcIntraSearch->predIntraAng(COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));

            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
            m_pcIntraSearch->predIntraAng(COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
          }
        }
        pu.mhIntraFlag = false;
      }

      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
      m_CABACEstimator->getCtx() = ctxStart;
    }
    else
    {
      if (bestIsMMVDSkip)
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand + ((mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1);
      }
      else
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
      }
    }
  }
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  uint32_t iteration;
  uint32_t iterationBegin = 0;
  if (encTestMode.lossless)
  {
    iteration = 1;
  }
  else
  {
    iteration = 2;
  }
  for (uint32_t uiNoResidualPass = iterationBegin; uiNoResidualPass < iteration; ++uiNoResidualPass)
  {
    for (uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++)
    {
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx].mergeCand;

      if (uiNoResidualPass != 0 && RdModeList[uiMrgHADIdx].isCIIP) // intrainter does not support skip mode
      {
        if (isTestSkipMerge[uiMergeCand])
        {
          continue;
        }
      }

      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMrgHADIdx])
        || ((uiNoResidualPass == 0) && bestIsSkip))
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
      cu.skip = false;
      cu.mmvdSkip = false;
      cu.triangle = false;
      //cu.affine
      cu.predMode = MODE_INTER;
      //cu.LICFlag
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

      if (uiNoResidualPass == 0 && RdModeList[uiMrgHADIdx].isCIIP)
      {
        cu.mmvdSkip = false;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
        pu.mhIntraFlag = true;
        pu.regularMergeFlag = false;
        pu.intraDir[0] = PLANAR_IDX;
        CHECK(pu.intraDir[0]<0 || pu.intraDir[0]>(NUM_LUMA_MODE - 1), "out of intra mode");
        pu.intraDir[1] = DM_CHROMA_IDX;
      }
      else if (RdModeList[uiMrgHADIdx].isMMVD)
      {
        cu.mmvdSkip = true;
#if JVET_O0249_MERGE_SYNTAX
        pu.regularMergeFlag = true;
#endif
        mergeCtx.setMmvdMergeCandiInfo(pu, uiMergeCand);
      }
      else
      {
        cu.mmvdSkip = false;
#if JVET_O0249_MERGE_SYNTAX
        pu.regularMergeFlag = true;
#endif
        mergeCtx.setMergeInfo(pu, uiMergeCand);
      }
      PU::spanMotionInfo(pu, mergeCtx);

      if (m_pcEncCfg->getMCTSEncConstraint())
      {
        bool isDMVR = PU::checkDMVRCondition(pu);
        if ((isDMVR && MCTSHelper::isRefBlockAtRestrictedTileBoundary(pu)) || (!isDMVR && !(MCTSHelper::checkMvBufferForMCTSConstraint(pu))))
        {
          // Do not use this mode
          tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
          continue;
        }
      }
      if (mrgTempBufSet)
      {
        {
          int dx, dy, i, j, num = 0;
          dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
          dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
          if (PU::checkDMVRCondition(pu))
          {
            for (i = 0; i < (pu.lumaSize().height); i += dy)
            {
              for (j = 0; j < (pu.lumaSize().width); j += dx)
              {
                pu.mvdL0SubPu[num] = refinedMvdL0[num][uiMergeCand];
                num++;
              }
            }
          }
        }
        if (pu.mhIntraFlag)
        {
          uint32_t bufIdx = 0;
          PelBuf tmpBuf = tempCS->getPredBuf(pu).Y();
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
          tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Y());
#else
          tmpBuf.copyFrom(acMergeBuffer[uiMergeCand].Y());
#endif
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            tmpBuf.rspSignal(m_pcReshape->getFwdLUT());
          }
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, bufIdx));
          tmpBuf = tempCS->getPredBuf(pu).Cb();
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
          tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cb());
#else
          tmpBuf.copyFrom(acMergeBuffer[uiMergeCand].Cb());
#endif
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Cb, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));
          tmpBuf = tempCS->getPredBuf(pu).Cr();
#if JVET_O0108_DIS_DMVR_BDOF_CIIP
          tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cr());
#else
          tmpBuf.copyFrom(acMergeBuffer[uiMergeCand].Cr());
#endif
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Cr, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
        }
        else
        {
          if (RdModeList[uiMrgHADIdx].isMMVD)
          {
            pu.mmvdEncOptMode = 0;
            m_pcInterSearch->motionCompensation(pu);
          }
          else if (uiNoResidualPass != 0 && RdModeList[uiMrgHADIdx].isCIIP)
          {
            tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand]);
          }
          else
          {
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
          }
        }
      }
      else
      {
        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation(pu);
        pu.mvRefine = false;
      }
      if (!cu.mmvdSkip && !pu.mhIntraFlag && uiNoResidualPass != 0)
      {
        CHECK(uiMergeCand >= mergeCtx.numValidMergeCand, "out of normal merge");
        isTestSkipMerge[uiMergeCand] = true;
      }

      xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL);

      if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip && !pu.mhIntraFlag)
      {
        bestIsSkip = !bestCS->cus.empty() && bestCS->getCU(partitioner.chType)->rootCbf == 0;
      }
      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    }// end loop uiMrgHADIdx

    if (uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection())
    {
      const CodingUnit     &bestCU = *bestCS->getCU(partitioner.chType);
      const PredictionUnit &bestPU = *bestCS->getPU(partitioner.chType);

      if (bestCU.rootCbf == 0)
      {
        if (bestPU.mergeFlag)
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if (m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE)
        {
          int absolute_MV = 0;

          for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
          {
            if (slice.getNumRefIdx(RefPicList(uiRefListIdx)) > 0)
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if (absolute_MV == 0)
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, partitioner);
  }
}

void EncCu::xCheckRDCostMergeTriangle2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  const Slice &slice = *tempCS->slice;
  const SPS &sps = *tempCS->sps;

  if (slice.getMaxNumTriangleCand() < 2)
    return;

  CHECK(slice.getSliceType() != B_SLICE, "Triangle mode is only applied to B-slices");

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  bool trianglecandHasNoResidual[TRIANGLE_MAX_NUM_CANDS];
  for (int mergeCand = 0; mergeCand < TRIANGLE_MAX_NUM_CANDS; mergeCand++)
  {
    trianglecandHasNoResidual[mergeCand] = false;
  }

#if JVET_O0379_SPEEDUP_TPM_ENCODER
  bool bestIsSkip = false;
#else
  bool bestIsSkip;
  CodingUnit* cuTemp = bestCS->getCU(partitioner.chType);
  if (cuTemp)
    bestIsSkip = m_pcEncCfg->getUseFastDecisionForMerge() ? bestCS->getCU(partitioner.chType)->rootCbf == 0 : false;
  else
    bestIsSkip = false;
#endif
  uint8_t                                         numTriangleCandidate = TRIANGLE_MAX_NUM_CANDS;
  uint8_t                                         triangleNumMrgSATDCand = TRIANGLE_MAX_NUM_SATD_CANDS;
  PelUnitBuf                                      triangleBuffer[TRIANGLE_MAX_NUM_UNI_CANDS];
  PelUnitBuf                                      triangleWeightedBuffer[TRIANGLE_MAX_NUM_CANDS];
  static_vector<uint8_t, TRIANGLE_MAX_NUM_CANDS> triangleRdModeList;
  static_vector<double, TRIANGLE_MAX_NUM_CANDS> tianglecandCostList;
  uint8_t                                         numTriangleCandComb = slice.getMaxNumTriangleCand() * (slice.getMaxNumTriangleCand() - 1) * 2;

#if !JVET_O0379_SPEEDUP_TPM_ENCODER
  if (auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl))
  {
    bestIsSkip |= blkCache->isSkip(tempCS->area);
  }
#endif

  DistParam distParam;
  const bool useHadamard = !encTestMode.lossless && !tempCS->slice->getDisableSATDForRD();
  m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, useHadamard);

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));

  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(encTestMode.lossless);

  MergeCtx triangleMrgCtx;
  {
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
    cu.triangle = true;
    cu.mmvdSkip = false;
    cu.GBiIdx = GBI_DEFAULT;

    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    pu.regularMergeFlag = false;

    PU::getTriangleMergeCandidates(pu, triangleMrgCtx);
    const uint8_t maxNumTriangleCand = pu.cs->slice->getMaxNumTriangleCand();
    for (uint8_t mergeCand = 0; mergeCand < maxNumTriangleCand; mergeCand++)
    {
      triangleBuffer[mergeCand] = m_acMergeBuffer[mergeCand].getBuf(localUnitArea);
      triangleMrgCtx.setMergeInfo(pu, mergeCand);
      PU::spanMotionInfo(pu, triangleMrgCtx);

      if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(pu))))
      {
        // Do not use this mode
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
        return;
      }
      m_pcInterSearch->motionCompensation(pu, triangleBuffer[mergeCand]);
    }
  }

#if JVET_O0379_SPEEDUP_TPM_ENCODER
  triangleNumMrgSATDCand = min(triangleNumMrgSATDCand, numTriangleCandComb);
#else
  bool tempBufSet = bestIsSkip ? false : true;
  triangleNumMrgSATDCand = bestIsSkip ? TRIANGLE_MAX_NUM_CANDS : TRIANGLE_MAX_NUM_SATD_CANDS;
  triangleNumMrgSATDCand = min(triangleNumMrgSATDCand, numTriangleCandComb);
  if (bestIsSkip)
  {
    for (uint8_t i = 0; i < TRIANGLE_MAX_NUM_CANDS; i++)
    {
      triangleRdModeList.push_back(i);
    }
  }
  else
#endif
  {
    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
    cu.skip = false;
    cu.predMode = MODE_INTER;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp = encTestMode.qp;
    cu.triangle = true;
    cu.mmvdSkip = false;
    cu.GBiIdx = GBI_DEFAULT;

    PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

    if (abs(floorLog2(cu.lwidth()) - floorLog2(cu.lheight())) >= 2)
    {
      numTriangleCandidate = 30;
    }
    else
    {
      numTriangleCandidate = TRIANGLE_MAX_NUM_CANDS;
    }

    numTriangleCandidate = min(numTriangleCandidate, numTriangleCandComb);

    for (uint8_t mergeCand = 0; mergeCand < numTriangleCandidate; mergeCand++)
    {
      bool    splitDir = m_triangleModeTest[mergeCand].m_splitDir;
      uint8_t candIdx0 = m_triangleModeTest[mergeCand].m_candIdx0;
      uint8_t candIdx1 = m_triangleModeTest[mergeCand].m_candIdx1;

      pu.triangleSplitDir = splitDir;
      pu.triangleMergeIdx0 = candIdx0;
      pu.triangleMergeIdx1 = candIdx1;
      pu.mergeFlag = true;
      pu.regularMergeFlag = false;
      triangleWeightedBuffer[mergeCand] = m_acTriangleWeightedBuffer[mergeCand].getBuf(localUnitArea);
      triangleBuffer[candIdx0] = m_acMergeBuffer[candIdx0].getBuf(localUnitArea);
      triangleBuffer[candIdx1] = m_acMergeBuffer[candIdx1].getBuf(localUnitArea);

      m_pcInterSearch->weightedTriangleBlk(pu, splitDir, CHANNEL_TYPE_LUMA, triangleWeightedBuffer[mergeCand], triangleBuffer[candIdx0], triangleBuffer[candIdx1]);
      distParam.cur = triangleWeightedBuffer[mergeCand].Y();

      Distortion uiSad = distParam.distFunc(distParam);

      uint32_t uiBitsCand = m_triangleIdxBins[splitDir][candIdx0][candIdx1];

      double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;

      updateCandList(mergeCand, cost, triangleRdModeList, tianglecandCostList
        , triangleNumMrgSATDCand);
    }

    // limit number of candidates using SATD-costs
    for (uint8_t i = 0; i < triangleNumMrgSATDCand; i++)
    {
      if (tianglecandCostList[i] > MRG_FAST_RATIO * tianglecandCostList[0] || tianglecandCostList[i] > getMergeBestSATDCost())
      {
        triangleNumMrgSATDCand = i;
        break;
      }
    }

    // perform chroma weighting process
    for (uint8_t i = 0; i < triangleNumMrgSATDCand; i++)
    {
      uint8_t  mergeCand = triangleRdModeList[i];
      bool     splitDir = m_triangleModeTest[mergeCand].m_splitDir;
      uint8_t  candIdx0 = m_triangleModeTest[mergeCand].m_candIdx0;
      uint8_t  candIdx1 = m_triangleModeTest[mergeCand].m_candIdx1;

      pu.triangleSplitDir = splitDir;
      pu.triangleMergeIdx0 = candIdx0;
      pu.triangleMergeIdx1 = candIdx1;
      pu.mergeFlag = true;
      pu.regularMergeFlag = false;
      m_pcInterSearch->weightedTriangleBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, triangleWeightedBuffer[mergeCand], triangleBuffer[candIdx0], triangleBuffer[candIdx1]);
    }

    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  }

  triangleNumMrgSATDCand = min(triangleNumMrgSATDCand, (uint8_t)triangleRdModeList.size());

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  {
    uint8_t iteration;
    uint8_t iterationBegin = 0;
    if (encTestMode.lossless)
    {
      iteration = 1;
    }
    else
    {
      iteration = 2;
    }
    for (uint8_t noResidualPass = iterationBegin; noResidualPass < iteration; ++noResidualPass)
    {
      for (uint8_t mrgHADIdx = 0; mrgHADIdx < triangleNumMrgSATDCand; mrgHADIdx++)
      {
        uint8_t mergeCand = triangleRdModeList[mrgHADIdx];

        if (((noResidualPass != 0) && trianglecandHasNoResidual[mergeCand])
          || ((noResidualPass == 0) && bestIsSkip))
        {
          continue;
        }

        bool    splitDir = m_triangleModeTest[mergeCand].m_splitDir;
        uint8_t candIdx0 = m_triangleModeTest[mergeCand].m_candIdx0;
        uint8_t candIdx1 = m_triangleModeTest[mergeCand].m_candIdx1;

        CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

        partitioner.setCUData(cu);
        cu.slice = tempCS->slice;
        cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
        cu.skip = false;
        cu.predMode = MODE_INTER;
        cu.transQuantBypass = encTestMode.lossless;
        cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
        cu.qp = encTestMode.qp;
        cu.triangle = true;
        cu.mmvdSkip = false;
        cu.GBiIdx = GBI_DEFAULT;
        PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

        pu.triangleSplitDir = splitDir;
        pu.triangleMergeIdx0 = candIdx0;
        pu.triangleMergeIdx1 = candIdx1;
        pu.mergeFlag = true;
        pu.regularMergeFlag = false;
        PU::spanTriangleMotionInfo(pu, triangleMrgCtx, splitDir, candIdx0, candIdx1);

        if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(*cu.firstPU))))
        {
          // Do not use this mode
          tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
          return;
        }
#if JVET_O0379_SPEEDUP_TPM_ENCODER
        tempCS->getPredBuf().copyFrom(triangleWeightedBuffer[mergeCand]);
#else
        if (tempBufSet)
        {
          tempCS->getPredBuf().copyFrom(triangleWeightedBuffer[mergeCand]);
        }
        else
        {
          triangleBuffer[candIdx0] = m_acMergeBuffer[candIdx0].getBuf(localUnitArea);
          triangleBuffer[candIdx1] = m_acMergeBuffer[candIdx1].getBuf(localUnitArea);
          PelUnitBuf predBuf = tempCS->getPredBuf();
          m_pcInterSearch->weightedTriangleBlk(pu, splitDir, MAX_NUM_CHANNEL_TYPE, predBuf, triangleBuffer[candIdx0], triangleBuffer[candIdx1]);
        }
#endif
        xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, noResidualPass, (noResidualPass == 0 ? &trianglecandHasNoResidual[mergeCand] : NULL));

        if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
        {
          bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
        }
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
      }// end loop mrgHADIdx
    }
  }
  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, partitioner);
  }
}

void EncCu::xCheckRDCostAffineMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  if (m_modeCtrl->getFastDeltaQp())
  {
    return;
  }

  if (bestCS->area.lumaSize().width < 8 || bestCS->area.lumaSize().height < 8)
  {
    return;
  }
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  const Slice &slice = *tempCS->slice;

  CHECK(slice.getSliceType() == I_SLICE, "Affine Merge modes not available for I-slices");

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  AffineMergeCtx affineMergeCtx;
  const SPS &sps = *tempCS->sps;

  MergeCtx mrgCtx;
  if (sps.getSBTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
    mrgCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
    affineMergeCtx.mrgCtx = &mrgCtx;
  }

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
    cu.mmvdSkip = false;

    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    pu.regularMergeFlag = false;
    PU::getAffineMergeCand(pu, affineMergeCtx);

    if (affineMergeCtx.numValidMergeCand <= 0)
    {
      return;
    }
  }

  bool candHasNoResidual[AFFINE_MRG_MAX_NUM_CANDS];
  for (uint32_t ui = 0; ui < affineMergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip = false;
  uint32_t                                    uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
  PelUnitBuf                                  acMergeBuffer[AFFINE_MRG_MAX_NUM_CANDS];
  static_vector<uint32_t, AFFINE_MRG_MAX_NUM_CANDS>  RdModeList;
  bool                                        mrgTempBufSet = false;

  for (uint32_t i = 0; i < AFFINE_MRG_MAX_NUM_CANDS; i++)
  {
    RdModeList.push_back(i);
  }

  if (m_pcEncCfg->getUseFastMerge())
  {
    uiNumMrgSATDCand = std::min(NUM_AFF_MRG_SATD_CAND, affineMergeCtx.numValidMergeCand);
    bestIsSkip = false;

    if (auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl))
    {
      bestIsSkip = blkCache->isSkip(tempCS->area);
    }

    static_vector<double, AFFINE_MRG_MAX_NUM_CANDS> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if (!bestIsSkip)
    {
      RdModeList.clear();
      mrgTempBufSet = true;
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(encTestMode.lossless);

      CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
      cu.skip = false;
      cu.affine = true;
      cu.predMode = MODE_INTER;
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;

      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

      DistParam distParam;
      const bool bUseHadamard = !encTestMode.lossless && !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));

      for (uint32_t uiMergeCand = 0; uiMergeCand < affineMergeCtx.numValidMergeCand; uiMergeCand++)
      {
        acMergeBuffer[uiMergeCand] = m_acMergeBuffer[uiMergeCand].getBuf(localUnitArea);

        // set merge information
        pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
        pu.mergeFlag = true;
        pu.regularMergeFlag = false;
        pu.mergeIdx = uiMergeCand;
        cu.affineType = affineMergeCtx.affineType[uiMergeCand];
        cu.GBiIdx = affineMergeCtx.GBiIdx[uiMergeCand];

        pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
        if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
        {
          pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
          pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
          PU::spanMotionInfo(pu, mrgCtx);
        }
        else
        {
          PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0);
          PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1);

          PU::spanMotionInfo(pu);
        }

        distParam.cur = acMergeBuffer[uiMergeCand].Y();

        m_pcInterSearch->motionCompensation(pu, acMergeBuffer[uiMergeCand]);

        Distortion uiSad = distParam.distFunc(distParam);
        uint32_t   uiBitsCand = uiMergeCand + 1;
        if (uiMergeCand == tempCS->slice->getMaxNumAffineMergeCand() - 1)
        {
          uiBitsCand--;
        }
        double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;
        updateCandList(uiMergeCand, cost, RdModeList, candCostList
          , uiNumMrgSATDCand);

        CHECK(std::min(uiMergeCand + 1, uiNumMrgSATDCand) != RdModeList.size(), "");
      }

      // Try to limit number of candidates using SATD-costs
      for (uint32_t i = 1; i < uiNumMrgSATDCand; i++)
      {
        if (candCostList[i] > MRG_FAST_RATIO * candCostList[0])
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    }
    else
    {
      uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
    }
  }

  uint32_t iteration;
  uint32_t iterationBegin = 0;
  if (encTestMode.lossless)
  {
    iteration = 1;
  }
  else
  {
    iteration = 2;
  }
  for (uint32_t uiNoResidualPass = iterationBegin; uiNoResidualPass < iteration; ++uiNoResidualPass)
  {
    for (uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++)
    {
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx];

      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMergeCand])
        || ((uiNoResidualPass == 0) && bestIsSkip))
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
      cu.skip = false;
      cu.affine = true;
      cu.predMode = MODE_INTER;
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

      // set merge information
      pu.mergeFlag = true;
      pu.mergeIdx = uiMergeCand;
      pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
      cu.affineType = affineMergeCtx.affineType[uiMergeCand];
      cu.GBiIdx = affineMergeCtx.GBiIdx[uiMergeCand];

      pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
      if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
      {
        pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
        pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
        PU::spanMotionInfo(pu, mrgCtx);
      }
      else
      {
        PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0);
        PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1);

        PU::spanMotionInfo(pu);
      }

      if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(*cu.firstPU))))
      {
        // Do not use this mode
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
        return;
      }
      if (mrgTempBufSet)
      {
        tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand]);
      }
      else
      {
        m_pcInterSearch->motionCompensation(pu);
      }

      xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, (uiNoResidualPass == 0 ? &candHasNoResidual[uiMergeCand] : NULL));

      if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
      {
        bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
      }
      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    }// end loop uiMrgHADIdx

    if (uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection())
    {
      const CodingUnit     &bestCU = *bestCS->getCU(partitioner.chType);
      const PredictionUnit &bestPU = *bestCS->getPU(partitioner.chType);

      if (bestCU.rootCbf == 0)
      {
        if (bestPU.mergeFlag)
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if (m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE)
        {
          int absolute_MV = 0;

          for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
          {
            if (slice.getNumRefIdx(RefPicList(uiRefListIdx)) > 0)
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if (absolute_MV == 0)
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, partitioner);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////
// ibc merge/skip mode check
void EncCu::xCheckRDCostIBCModeMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  assert(partitioner.chType != CHANNEL_TYPE_CHROMA); // chroma IBC is derived
#if JVET_O1161_IBC_MAX_SIZE
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128) // disable IBC mode larger than 64x64
#else
  if (tempCS->area.lwidth() == 128 && tempCS->area.lheight() == 128) // disable 128x128 IBC mode
#endif
  {
    return;
  }
  const SPS &sps = *tempCS->sps;

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  MergeCtx mergeCtx;


  if (sps.getSBTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
  }

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_IBC;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    cu.mmvdSkip = false;
    pu.mmvdMergeFlag = false;
    pu.regularMergeFlag = false;
    cu.triangle = false;
    pu.shareParentPos = tempCS->sharedBndPos;
    pu.shareParentSize = tempCS->sharedBndSize;
    PU::getIBCMergeCandidates(pu, mergeCtx);
  }

  int candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (unsigned int ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = 0;
  }

  bool                                        bestIsSkip = false;
  unsigned                                    numMrgSATDCand = mergeCtx.numValidMergeCand;
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList(MRG_MAX_NUM_CANDS);
  for (unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++)
  {
    RdModeList[i] = i;
  }

  //{
  static_vector<double, MRG_MAX_NUM_CANDS>  candCostList(MRG_MAX_NUM_CANDS, MAX_DOUBLE);
  // 1. Pass: get SATD-cost for selected candidates and reduce their count
  {
    const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(encTestMode.lossless);

    CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
    cu.skip = false;
    cu.predMode = MODE_IBC;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp = encTestMode.qp;
    cu.mmvdSkip = false;
    cu.triangle = false;
    DistParam distParam;
    const bool bUseHadamard = !encTestMode.lossless && !cu.slice->getDisableSATDForRD();
    PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType); //tempCS->addPU(cu);
    pu.mmvdMergeFlag = false;
    pu.regularMergeFlag = false;
    Picture* refPic = pu.cu->slice->getPic();
    const CPelBuf refBuf = refPic->getRecoBuf(pu.blocks[COMPONENT_Y]);
    const Pel*        piRefSrch = refBuf.buf;
    if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      const CompArea &area = cu.blocks[COMPONENT_Y];
      CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
      PelBuf tmpLuma = m_tmpStorageLCU->getBuf(tmpArea);
      tmpLuma.copyFrom(tempCS->getOrgBuf().Y());
      tmpLuma.rspSignal(m_pcReshape->getFwdLUT());
      m_pcRdCost->setDistParam(distParam, tmpLuma, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
    }
    else
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
    int refStride = refBuf.stride;
    const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
    int numValidBv = mergeCtx.numValidMergeCand;
    for (unsigned int mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
    {
      mergeCtx.setMergeInfo(pu, mergeCand); // set bv info in merge mode
      const int cuPelX = pu.Y().x;
      const int cuPelY = pu.Y().y;
      int roiWidth = pu.lwidth();
      int roiHeight = pu.lheight();
#if JVET_O1164_PS
      const int picWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
      const int picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
#else
      const int picWidth = pu.cs->slice->getSPS()->getPicWidthInLumaSamples();
      const int picHeight = pu.cs->slice->getSPS()->getPicHeightInLumaSamples();
#endif
      const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
      int xPred = pu.bv.getHor();
      int yPred = pu.bv.getVer();

#if JVET_O1170_IBC_VIRTUAL_BUFFER
      if (!m_pcInterSearch->searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth)) // not valid bv derived
#else
      if (!PU::isBlockVectorValid(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, 0, xPred, yPred, lcuWidth)) // not valid bv derived
#endif
      {
        numValidBv--;
        continue;
      }
      PU::spanMotionInfo(pu, mergeCtx);

      distParam.cur.buf = piRefSrch + refStride * yPred + xPred;

      Distortion sad = distParam.distFunc(distParam);
      unsigned int bitsCand = mergeCand + 1;
      if (mergeCand == tempCS->slice->getMaxNumMergeCand() - 1)
      {
        bitsCand--;
      }
      double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;

      updateCandList(mergeCand, cost, RdModeList, candCostList
        , numMrgSATDCand);
    }

    // Try to limit number of candidates using SATD-costs
    if (numValidBv)
    {
      numMrgSATDCand = numValidBv;
      for (unsigned int i = 1; i < numValidBv; i++)
      {
        if (candCostList[i] > MRG_FAST_RATIO*candCostList[0])
        {
          numMrgSATDCand = i;
          break;
        }
      }
    }
    else
    {
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
      return;
    }

    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
  }
  //}


  const unsigned int iteration = encTestMode.lossless ? 1 : 2;
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  // 2. Pass: check candidates using full RD test
  for (unsigned int numResidualPass = 0; numResidualPass < iteration; numResidualPass++)
  {
    for (unsigned int mrgHADIdx = 0; mrgHADIdx < numMrgSATDCand; mrgHADIdx++)
    {
      unsigned int mergeCand = RdModeList[mrgHADIdx];
      if (!(numResidualPass == 1 && candHasNoResidual[mergeCand] == 1))
      {
        if (!(bestIsSkip && (numResidualPass == 0)))
        {
          {

            // first get merge candidates
            CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

            partitioner.setCUData(cu);
            cu.slice = tempCS->slice;
            cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
            cu.skip = false;
            cu.predMode = MODE_IBC;
            cu.transQuantBypass = encTestMode.lossless;
            cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
            cu.qp = encTestMode.qp;
            cu.sbtInfo = 0;

            PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);// tempCS->addPU(cu);
            pu.intraDir[0] = DC_IDX; // set intra pred for ibc block
            pu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block
            cu.mmvdSkip = false;
            pu.mmvdMergeFlag = false;
            pu.regularMergeFlag = false;
            cu.triangle = false;
            mergeCtx.setMergeInfo(pu, mergeCand);
            PU::spanMotionInfo(pu, mergeCtx);

            assert(mergeCtx.mrgTypeNeighbours[mergeCand] == MRG_TYPE_IBC); //  should be IBC candidate at this round
#if JVET_O0050_LOCAL_DUAL_TREE
            const bool chroma = !pu.cu->isSepTree();
#else
            const bool chroma = !(CS::isDualITree(*tempCS));
#endif

            //  MC
            m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, true, chroma);
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

            m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0), true, chroma);
            xEncodeDontSplit(*tempCS, partitioner);

#if ENABLE_QPA_SUB_CTU
            xCheckDQP(*tempCS, partitioner);
#else
            // this if-check is redundant
            if (tempCS->pps->getUseDQP() && partitioner.currQgEnable())
            {
              xCheckDQP(*tempCS, partitioner);
            }
#endif


            DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

            tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
          }

          if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
          {
            if (bestCS->getCU(partitioner.chType) == NULL)
              bestIsSkip = 0;
            else
              bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
          }
        }
      }
    }
  }
  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, partitioner);
  }
}

void EncCu::xCheckRDCostIBCMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
#if JVET_O1161_IBC_MAX_SIZE
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128) // disable IBC mode larger than 64x64
#else
  if (tempCS->area.lwidth() == 128 && tempCS->area.lheight() == 128) // disable 128x128 IBC mode
#endif
  {
    return;
  }

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
  cu.skip = false;
  cu.predMode = MODE_IBC;
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  cu.imv = 0;
  cu.sbtInfo = 0;

  CU::addPUs(cu);

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  PredictionUnit& pu = *cu.firstPU;
  cu.mmvdSkip = false;
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
  pu.shareParentPos = tempCS->sharedBndPos;
  pu.shareParentSize = tempCS->sharedBndSize;

  pu.intraDir[0] = DC_IDX; // set intra pred for ibc block
  pu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block

  pu.interDir = 1; // use list 0 for IBC mode
  pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF; // last idx in the list
#if !JVET_O0258_REMOVE_CHROMA_IBC_FOR_DUALTREE
  if (partitioner.chType == CHANNEL_TYPE_LUMA)
  {
#endif
    bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap);

    if (bValid)
    {
      PU::spanMotionInfo(pu);
#if JVET_O0050_LOCAL_DUAL_TREE
      const bool chroma = !pu.cu->isSepTree();
#else
      const bool chroma = !(CS::isDualITree(*tempCS));
#endif
      //  MC
      m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, true, chroma);

      {

        m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, true, chroma);

        xEncodeDontSplit(*tempCS, partitioner);

#if ENABLE_QPA_SUB_CTU
        xCheckDQP(*tempCS, partitioner);
#else
        // this if-check is redundant
        if (tempCS->pps->getUseDQP() && partitioner.currQgEnable())
        {
          xCheckDQP(*tempCS, partitioner);
        }
#endif

        tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
        if (m_bestModeUpdated)
        {
          xCalDebCost(*tempCS, partitioner);
        }

        DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
        xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

      }

    } // bValid
    else
    {
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
    }
#if !JVET_O0258_REMOVE_CHROMA_IBC_FOR_DUALTREE
  }
  // chroma CU ibc comp
  else
  {
    bool success = true;
    // chroma tree, reuse luma bv at minimal block level
    // enabled search only when each chroma sub-block has a BV from its luma sub-block
    assert(tempCS->getIbcLumaCoverage(pu.Cb()) == IBC_LUMA_COVERAGE_FULL);
    // check if each BV for the chroma sub-block is valid
    //static const UInt unitArea = MIN_PU_SIZE * MIN_PU_SIZE;
    const CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
    PredictionUnit subPu;
    subPu.cs = pu.cs;
    subPu.cu = pu.cu;
    const ComponentID compID = COMPONENT_Cb; // use Cb to represent both Cb and CR, as their structures are the same
    int shiftHor = ::getComponentScaleX(compID, pu.chromaFormat);
    int shiftVer = ::getComponentScaleY(compID, pu.chromaFormat);
    //const ChromaFormat  chFmt = pu.chromaFormat;

    for (int y = lumaArea.y; y < lumaArea.y + lumaArea.height; y += MIN_PU_SIZE)
    {
      for (int x = lumaArea.x; x < lumaArea.x + lumaArea.width; x += MIN_PU_SIZE)
      {
        const MotionInfo &curMi = pu.cs->picture->cs->getMotionInfo(Position{ x, y });

        subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, MIN_PU_SIZE, MIN_PU_SIZE)));
        Position offsetRef = subPu.blocks[compID].pos().offset((curMi.bv.getHor() >> shiftHor), (curMi.bv.getVer() >> shiftVer));
        Position refEndPos(offsetRef.x + subPu.blocks[compID].size().width - 1, offsetRef.y + subPu.blocks[compID].size().height - 1);

        if (!subPu.cs->isDecomp(refEndPos, toChannelType(compID)) || !subPu.cs->isDecomp(offsetRef, toChannelType(compID))) // ref block is not yet available for this chroma sub-block
        {
          success = false;
          break;
        }
      }
      if (!success)
        break;
    }
    ////////////////////////////////////////////////////////////////////////////

    if (success)
    {
      //pu.mergeType = MRG_TYPE_IBC;
      m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, false, true); // luma=0, chroma=1
      m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, false, true);

      xEncodeDontSplit(*tempCS, partitioner);

      xCheckDQP(*tempCS, partitioner);
      tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
      if (m_bestModeUpdated)
      {
        xCalDebCost(*tempCS, partitioner);
      }

      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());

      xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
    }
    else
    {
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
    }
  }
#endif
}
// check ibc mode in encoder RD
//////////////////////////////////////////////////////////////////////////////////////////////

void EncCu::xCheckRDCostInter(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);


  m_pcInterSearch->setAffineModeSelected(false);

  if (tempCS->slice->getCheckLDC())
  {
    m_bestGbiCost[0] = m_bestGbiCost[1] = std::numeric_limits<double>::max();
    m_bestGbiIdx[0] = m_bestGbiIdx[1] = -1;
  }

  m_pcInterSearch->resetBufferedUniMotions();
  int gbiLoopNum = (tempCS->slice->isInterB() ? GBI_NUM : 1);
  gbiLoopNum = (tempCS->sps->getUseGBi() ? gbiLoopNum : 1);

  if (tempCS->area.lwidth() * tempCS->area.lheight() < GBI_SIZE_CONSTRAINT)
  {
    gbiLoopNum = 1;
  }

  double curBestCost = bestCS->cost;
  double equGBiCost = MAX_DOUBLE;

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  for (int gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++)
  {
    if (m_pcEncCfg->getUseGBiFast())
    {
      auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl);

      if (blkCache)
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestGBiIdx = blkCache->getGbiIdx(bestCS->area);

        if (isBestInter && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT && g_GbiSearchOrder[gbiLoopIdx] != bestGBiIdx)
        {
          continue;
        }
      }
    }
    if (!tempCS->slice->getCheckLDC())
    {
      if (gbiLoopIdx != 0 && gbiLoopIdx != 3 && gbiLoopIdx != 4)
      {
        continue;
      }
    }

    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
    cu.skip = false;
    cu.mmvdSkip = false;
    //cu.affine
    cu.predMode = MODE_INTER;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp = encTestMode.qp;
    CU::addPUs(cu);

    cu.GBiIdx = g_GbiSearchOrder[gbiLoopIdx];
    uint8_t gbiIdx = cu.GBiIdx;
    bool  testGbi = (gbiIdx != GBI_DEFAULT);

    m_pcInterSearch->predInterSearch(cu, partitioner);

    gbiIdx = CU::getValidGbiIdx(cu);
    if (testGbi && gbiIdx == GBI_DEFAULT) // Enabled GBi but the search results is uni.
    {
      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
      continue;
    }
    CHECK(!(testGbi || (!testGbi && gbiIdx == GBI_DEFAULT)), " !( bTestGbi || (!bTestGbi && gbiIdx == GBI_DEFAULT ) )");

    bool isEqualUni = false;
    if (m_pcEncCfg->getUseGBiFast())
    {
      if (cu.firstPU->interDir != 3 && testGbi == 0)
      {
        isEqualUni = true;
      }
    }

    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
      , 0
      , &equGBiCost
    );

    if (g_GbiSearchOrder[gbiLoopIdx] == GBI_DEFAULT)
      m_pcInterSearch->setAffineModeSelected((bestCS->cus.front()->affine && !(bestCS->cus.front()->firstPU->mergeFlag)));

    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

    double skipTH = MAX_DOUBLE;
    skipTH = (m_pcEncCfg->getUseGBiFast() ? 1.05 : MAX_DOUBLE);
    if (equGBiCost > curBestCost * skipTH)
    {
      break;
    }

    if (m_pcEncCfg->getUseGBiFast())
    {
      if (isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1)
      {
        break;
      }
    }
    if (g_GbiSearchOrder[gbiLoopIdx] == GBI_DEFAULT && xIsGBiSkip(cu) && m_pcEncCfg->getUseGBiFast())
    {
      break;
    }
  }  // for( UChar gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )
  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, partitioner);
  }
}




#if JVET_O0057_ALTHPELIF
bool EncCu::xCheckRDCostInterIMV(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, double &bestIntPelCost)
#else
bool EncCu::xCheckRDCostInterIMV(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
#endif
{
  int iIMV = int((encTestMode.opts & ETO_IMV) >> ETO_IMV_SHIFT);
  m_pcInterSearch->setAffineModeSelected(false);
#if JVET_O0057_ALTHPELIF
  // Only Half-Pel, int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK(iIMV < 1 || iIMV > 4, "Unsupported IMV Mode");
  const bool testAltHpelFilter = iIMV == 4;
#else
  // Only int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK(iIMV != 1 && iIMV != 2 && iIMV != 3, "Unsupported IMV Mode");
#endif
  // Fast 4-Pel Mode

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  EncTestMode encTestModeBase = encTestMode;                                        // copy for clearing non-IMV options
  encTestModeBase.opts = EncTestModeOpts(encTestModeBase.opts & ETO_IMV);  // clear non-IMV options (is that intended?)

  tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

  m_pcInterSearch->resetBufferedUniMotions();
  int gbiLoopNum = (tempCS->slice->isInterB() ? GBI_NUM : 1);
  gbiLoopNum = (tempCS->slice->getSPS()->getUseGBi() ? gbiLoopNum : 1);

  if (tempCS->area.lwidth() * tempCS->area.lheight() < GBI_SIZE_CONSTRAINT)
  {
    gbiLoopNum = 1;
  }

  bool validMode = false;
  double curBestCost = bestCS->cost;
  double equGBiCost = MAX_DOUBLE;

  for (int gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++)
  {
    if (m_pcEncCfg->getUseGBiFast())
    {
      auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl);

      if (blkCache)
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestGBiIdx = blkCache->getGbiIdx(bestCS->area);

        if (isBestInter && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT && g_GbiSearchOrder[gbiLoopIdx] != bestGBiIdx)
        {
          continue;
        }
      }
    }

    if (!tempCS->slice->getCheckLDC())
    {
      if (gbiLoopIdx != 0 && gbiLoopIdx != 3 && gbiLoopIdx != 4)
      {
        continue;
      }
    }

    if (m_pcEncCfg->getUseGBiFast() && tempCS->slice->getCheckLDC() && g_GbiSearchOrder[gbiLoopIdx] != GBI_DEFAULT
      && (m_bestGbiIdx[0] >= 0 && g_GbiSearchOrder[gbiLoopIdx] != m_bestGbiIdx[0])
      && (m_bestGbiIdx[1] >= 0 && g_GbiSearchOrder[gbiLoopIdx] != m_bestGbiIdx[1]))
    {
      continue;
    }

    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->picture->brickMap->getBrickIdxRsMap(tempCS->area.lumaPos());
    cu.skip = false;
    cu.mmvdSkip = false;
    //cu.affine
    cu.predMode = MODE_INTER;
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp = encTestMode.qp;

    CU::addPUs(cu);

#if JVET_O0057_ALTHPELIF
    if (testAltHpelFilter)
    {
      cu.imv = IMV_HPEL;
    }
    else
    {
      cu.imv = iIMV == 1 ? IMV_FPEL : IMV_4PEL;
    }
#else
    cu.imv = iIMV > 1 ? 2 : 1;
#endif

    bool testGbi;
    uint8_t gbiIdx;
#if JVET_O0057_ALTHPELIF
    bool affineAmvrEanbledFlag = !testAltHpelFilter && cu.slice->getSPS()->getAffineAmvrEnabledFlag();
#else
    bool affineAmvrEanbledFlag = cu.slice->getSPS()->getAffineAmvrEnabledFlag();
#endif

    cu.GBiIdx = g_GbiSearchOrder[gbiLoopIdx];
    gbiIdx = cu.GBiIdx;
    testGbi = (gbiIdx != GBI_DEFAULT);

    cu.firstPU->interDir = 10;

    m_pcInterSearch->predInterSearch(cu, partitioner);

    if (cu.firstPU->interDir <= 3)
    {
      gbiIdx = CU::getValidGbiIdx(cu);
    }
    else
    {
      return false;
    }

    if (m_pcEncCfg->getMCTSEncConstraint() && ((cu.firstPU->refIdx[L0] < 0 && cu.firstPU->refIdx[L1] < 0) || (!(MCTSHelper::checkMvBufferForMCTSConstraint(*cu.firstPU)))))
    {
      // Do not use this mode
      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
      continue;
    }
    if (testGbi && gbiIdx == GBI_DEFAULT) // Enabled GBi but the search results is uni.
    {
      tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
      continue;
    }
    CHECK(!(testGbi || (!testGbi && gbiIdx == GBI_DEFAULT)), " !( bTestGbi || (!bTestGbi && gbiIdx == GBI_DEFAULT ) )");

    bool isEqualUni = false;
    if (m_pcEncCfg->getUseGBiFast())
    {
      if (cu.firstPU->interDir != 3 && testGbi == 0)
      {
        isEqualUni = true;
      }
    }

    if (!CU::hasSubCUNonZeroMVd(cu) && !CU::hasSubCUNonZeroAffineMVd(cu))
    {
      if (m_modeCtrl->useModeResult(encTestModeBase, tempCS, partitioner))
      {
        std::swap(tempCS, bestCS);
        // store temp best CI for next CU coding
        m_CurrCtx->best = m_CABACEstimator->getCtx();
      }
      if (affineAmvrEanbledFlag)
      {
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
        continue;
      }
      else
      {
        return false;
      }
    }

    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestModeBase, 0
      , 0
      , &equGBiCost
    );

#if JVET_O0057_ALTHPELIF
    if (cu.imv == IMV_FPEL && tempCS->cost < bestIntPelCost)
    {
      bestIntPelCost = tempCS->cost;
    }
#endif
    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);

    double skipTH = MAX_DOUBLE;
    skipTH = (m_pcEncCfg->getUseGBiFast() ? 1.05 : MAX_DOUBLE);
    if (equGBiCost > curBestCost * skipTH)
    {
      break;
    }

    if (m_pcEncCfg->getUseGBiFast())
    {
      if (isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1)
      {
        break;
      }
    }
    if (g_GbiSearchOrder[gbiLoopIdx] == GBI_DEFAULT && xIsGBiSkip(cu) && m_pcEncCfg->getUseGBiFast())
    {
      break;
    }
    validMode = true;
  } // for( UChar gbiLoopIdx = 0; gbiLoopIdx < gbiLoopNum; gbiLoopIdx++ )

  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, partitioner);
  }

  return tempCS->slice->getSPS()->getAffineAmvrEnabledFlag() ? validMode : true;
}

void EncCu::xCalDebCost(CodingStructure &cs, Partitioner &partitioner, bool calDist)
{
  if (cs.cost == MAX_DOUBLE)
  {
    cs.costDbOffset = 0;
  }

  if (cs.slice->getDeblockingFilterDisable() || (!m_pcEncCfg->getUseEncDbOpt() && !calDist))
  {
    return;
  }

  m_pcLoopFilter->setEnc(true);
  const ChromaFormat format = cs.area.chromaFormat;
  CodingUnit*                cu = cs.getCU(partitioner.chType);
  const Position lumaPos = cu->Y().valid() ? cu->Y().pos() : recalcPosition(format, cu->chType, CHANNEL_TYPE_LUMA, cu->blocks[cu->chType].pos());
#if JVET_O0060_4x4_deblocking
  bool topEdgeAvai = lumaPos.y > 0 && ((lumaPos.y % 4) == 0);
  bool leftEdgeAvai = lumaPos.x > 0 && ((lumaPos.x % 4) == 0);
#else
  bool topEdgeAvai = lumaPos.y > 0 && ((lumaPos.y % 8) == 0);
  bool leftEdgeAvai = lumaPos.x > 0 && ((lumaPos.x % 8) == 0);
#endif
  bool anyEdgeAvai = topEdgeAvai || leftEdgeAvai;
  cs.costDbOffset = 0;

  if (calDist)
  {
    const UnitArea currCsArea = clipArea(CS::getArea(cs, cs.area, partitioner.chType), *cs.picture);
#if JVET_O0050_LOCAL_DUAL_TREE
    ComponentID compStr = (cu->isSepTree() && !isLuma(partitioner.chType)) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = (cu->isSepTree() && isLuma(partitioner.chType)) ? COMPONENT_Y : COMPONENT_Cr;
#else
    ComponentID compStr = (CS::isDualITree(cs) && !isLuma(partitioner.chType)) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = (CS::isDualITree(cs) && isLuma(partitioner.chType)) ? COMPONENT_Y : COMPONENT_Cr;
#endif
    Distortion finalDistortion = 0;
    for (int comp = compStr; comp <= compEnd; comp++)
    {
      const ComponentID compID = ComponentID(comp);
      CPelBuf org = cs.getOrgBuf(compID);
      CPelBuf reco = cs.getRecoBuf(compID);
      finalDistortion += getDistortionDb(cs, org, reco, compID, currCsArea.block(compID), false);
    }
    //updated distortion
    cs.dist = finalDistortion;
  }

  if (anyEdgeAvai && m_pcEncCfg->getUseEncDbOpt())
  {
#if JVET_O0050_LOCAL_DUAL_TREE
    ComponentID compStr = (cu->isSepTree() && !isLuma(partitioner.chType)) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = (cu->isSepTree() && isLuma(partitioner.chType)) ? COMPONENT_Y : COMPONENT_Cr;
#else
    ComponentID compStr = (CS::isDualITree(cs) && !isLuma(partitioner.chType)) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = (CS::isDualITree(cs) && isLuma(partitioner.chType)) ? COMPONENT_Y : COMPONENT_Cr;
#endif

    const UnitArea currCsArea = clipArea(CS::getArea(cs, cs.area, partitioner.chType), *cs.picture);

    PelStorage&          picDbBuf = m_pcLoopFilter->getDbEncPicYuvBuffer();

    //deblock neighbour pixels
    const Size     lumaSize = cu->Y().valid() ? cu->Y().size() : recalcSize(format, cu->chType, CHANNEL_TYPE_LUMA, cu->blocks[cu->chType].size());

    const int verOffset = lumaPos.y > 7 ? 8 : 4;
    const int horOffset = lumaPos.x > 7 ? 8 : 4;
    const UnitArea areaTop(format, Area(lumaPos.x, lumaPos.y - verOffset, lumaSize.width, verOffset));
    const UnitArea areaLeft(format, Area(lumaPos.x - horOffset, lumaPos.y, horOffset, lumaSize.height));
    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      ComponentID compId = (ComponentID)compIdx;

      //Copy current CU's reco to Deblock Pic Buffer
      const CompArea&  curCompArea = currCsArea.block(compId);
      picDbBuf.getBuf(curCompArea).copyFrom(cs.getRecoBuf(curCompArea));
      if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma(compId))
      {
        picDbBuf.getBuf(curCompArea).rspSignal(m_pcReshape->getInvLUT());
      }

      //left neighbour
      if (leftEdgeAvai)
      {
        const CompArea&  compArea = areaLeft.block(compId);
        picDbBuf.getBuf(compArea).copyFrom(cs.picture->getRecoBuf(compArea));
        if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma(compId))
        {
          picDbBuf.getBuf(compArea).rspSignal(m_pcReshape->getInvLUT());
        }
      }
      //top neighbour
      if (topEdgeAvai)
      {
        const CompArea&  compArea = areaTop.block(compId);
        picDbBuf.getBuf(compArea).copyFrom(cs.picture->getRecoBuf(compArea));
        if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma(compId))
        {
          picDbBuf.getBuf(compArea).rspSignal(m_pcReshape->getInvLUT());
        }
      }
    }

    //deblock
    if (leftEdgeAvai)
    {
      m_pcLoopFilter->xDeblockCU(*cu, EDGE_VER);
    }

    if (topEdgeAvai)
    {
      m_pcLoopFilter->xDeblockCU(*cu, EDGE_HOR);
    }

    //update current CU SSE
    Distortion distCur = 0;
    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      ComponentID compId = (ComponentID)compIdx;
      CPelBuf reco = picDbBuf.getBuf(currCsArea.block(compId));
      CPelBuf org = cs.getOrgBuf(compId);
      distCur += getDistortionDb(cs, org, reco, compId, currCsArea.block(compId), true);
    }

    //calculate difference between DB_before_SSE and DB_after_SSE for neighbouring CUs
    Distortion distBeforeDb = 0, distAfterDb = 0;
    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      ComponentID compId = (ComponentID)compIdx;
      if (leftEdgeAvai)
      {
        const CompArea&  compArea = areaLeft.block(compId);
        CPelBuf org = cs.picture->getOrigBuf(compArea);
        CPelBuf reco = cs.picture->getRecoBuf(compArea);
        CPelBuf recoDb = picDbBuf.getBuf(compArea);
        distBeforeDb += getDistortionDb(cs, org, reco, compId, compArea, false);
        distAfterDb += getDistortionDb(cs, org, recoDb, compId, compArea, true);
      }
      if (topEdgeAvai)
      {
        const CompArea&  compArea = areaTop.block(compId);
        CPelBuf org = cs.picture->getOrigBuf(compArea);
        CPelBuf reco = cs.picture->getRecoBuf(compArea);
        CPelBuf recoDb = picDbBuf.getBuf(compArea);
        distBeforeDb += getDistortionDb(cs, org, reco, compId, compArea, false);
        distAfterDb += getDistortionDb(cs, org, recoDb, compId, compArea, true);
      }
    }

    //updated cost
    int64_t distTmp = distCur - cs.dist + distAfterDb - distBeforeDb;
    int sign = distTmp < 0 ? -1 : 1;
    distTmp = distTmp < 0 ? -distTmp : distTmp;
    cs.costDbOffset = sign * m_pcRdCost->calcRdCost(0, distTmp);
  }

  m_pcLoopFilter->setEnc(false);
}

Distortion EncCu::getDistortionDb(CodingStructure &cs, CPelBuf org, CPelBuf reco, ComponentID compID, const CompArea& compArea, bool afterDb)
{
  Distortion dist = 0;
#if WCG_EXT
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());
  CPelBuf orgLuma = cs.picture->getOrigBuf(cs.area.blocks[COMPONENT_Y]);
  if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
    m_pcEncCfg->getReshaper() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
  {
    if (compID == COMPONENT_Y && !afterDb && !m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
    {
      CompArea    tmpArea(COMPONENT_Y, cs.area.chromaFormat, Position(0, 0), compArea.size());
      PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf(tmpArea);
      tmpRecLuma.copyFrom(reco);
      tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
      dist += m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
    }
    else
    {
      dist += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
    }
  }
  else if (m_pcEncCfg->getReshaper() && cs.slice->getLmcsEnabledFlag() && cs.slice->isIntra()) //intra slice
  {
    if (compID == COMPONENT_Y && afterDb)
    {
      CompArea    tmpArea(COMPONENT_Y, cs.area.chromaFormat, Position(0, 0), compArea.size());
      PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf(tmpArea);
      tmpRecLuma.copyFrom(reco);
      tmpRecLuma.rspSignal(m_pcReshape->getFwdLUT());
      dist += m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE);
    }
    else
    {
      dist += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE);
    }
  }
  else
#endif
  {
    dist = m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE);
  }
  return dist;
}

void EncCu::xEncodeInterResidual(CodingStructure *&tempCS
  , CodingStructure *&bestCS
  , Partitioner &partitioner
  , const EncTestMode& encTestMode
  , int residualPass
  , bool* bestHasNonResi
  , double* equGBiCost
)
{
  if (residualPass == 1 && encTestMode.lossless)
  {
    return;
  }

  CodingUnit*            cu = tempCS->getCU(partitioner.chType);
  double   bestCostInternal = MAX_DOUBLE;
  double           bestCost = bestCS->cost;
  double           bestCostBegin = bestCS->cost;
  CodingUnit*      prevBestCU = bestCS->getCU(partitioner.chType);
  uint8_t          prevBestSbt = (prevBestCU == nullptr) ? 0 : prevBestCU->sbtInfo;
  bool              swapped = false; // avoid unwanted data copy
  bool             reloadCU = false;

#if JVET_O0567_MVDRange_Constraint
  const PredictionUnit& pu = *cu->firstPU;

  // clang-format off
  const int affineShiftTab[3] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_SIXTEENTH,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT
  };

  const int normalShiftTab[NUM_IMV_MODES] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT,
    MV_PRECISION_INTERNAL - MV_PRECISION_4PEL,
#if JVET_O0057_ALTHPELIF
    MV_PRECISION_INTERNAL - MV_PRECISION_HALF,
#endif
  };
  // clang-format on

  int mvShift;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      if (!cu->affine)
      {
        mvShift = normalShiftTab[cu->imv];
        Mv signaledmvd(pu.mvd[refList].getHor() >> mvShift, pu.mvd[refList].getVer() >> mvShift);
        if (!((signaledmvd.getHor() >= MVD_MIN) && (signaledmvd.getHor() <= MVD_MAX)) || !((signaledmvd.getVer() >= MVD_MIN) && (signaledmvd.getVer() <= MVD_MAX)))
          return;
      }
      else
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          mvShift = affineShiftTab[cu->imv];
          Mv signaledmvd(pu.mvdAffi[refList][ctrlP].getHor() >> mvShift, pu.mvdAffi[refList][ctrlP].getVer() >> mvShift);
          if (!((signaledmvd.getHor() >= MVD_MIN) && (signaledmvd.getHor() <= MVD_MAX)) || !((signaledmvd.getVer() >= MVD_MIN) && (signaledmvd.getVer() <= MVD_MAX)))
            return;
        }
      }
    }
  }
#else
  // Not allow very big |MVd| to avoid CABAC crash caused by too large MVd. Normally no impact on coding performance.
  const int maxMvd = 1 << 15;
  const PredictionUnit& pu = *cu->firstPU;
  if (!cu->affine)
  {
    if ((pu.refIdx[0] >= 0 && (pu.mvd[0].getAbsHor() >= maxMvd || pu.mvd[0].getAbsVer() >= maxMvd))
      || (pu.refIdx[1] >= 0 && (pu.mvd[1].getAbsHor() >= maxMvd || pu.mvd[1].getAbsVer() >= maxMvd)))
    {
      return;
    }
  }
  else
  {
    for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
    {
      if (pu.refIdx[refList] >= 0)
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          if (pu.mvdAffi[refList][ctrlP].getAbsHor() >= maxMvd || pu.mvdAffi[refList][ctrlP].getAbsVer() >= maxMvd)
          {
            return;
          }
        }
      }
    }
  }
#endif
  // avoid MV exceeding 18-bit dynamic range
  const int maxMv = 1 << 17;
  if (!cu->affine && !pu.mergeFlag)
  {
    if ((pu.refIdx[0] >= 0 && (pu.mv[0].getAbsHor() >= maxMv || pu.mv[0].getAbsVer() >= maxMv))
      || (pu.refIdx[1] >= 0 && (pu.mv[1].getAbsHor() >= maxMv || pu.mv[1].getAbsVer() >= maxMv)))
    {
      return;
    }
  }
  if (cu->affine && !pu.mergeFlag)
  {
    for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
    {
      if (pu.refIdx[refList] >= 0)
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          if (pu.mvAffi[refList][ctrlP].getAbsHor() >= maxMv || pu.mvAffi[refList][ctrlP].getAbsVer() >= maxMv)
          {
            return;
          }
        }
      }
    }
  }
  const bool mtsAllowed = tempCS->sps->getUseInterMTS() && CU::isInter(*cu) && partitioner.currArea().lwidth() <= MTS_INTER_MAX_CU_SIZE && partitioner.currArea().lheight() <= MTS_INTER_MAX_CU_SIZE;
  uint8_t sbtAllowed = cu->checkAllowedSbt();
  uint8_t numRDOTried = 0;
  Distortion sbtOffDist = 0;
  bool    sbtOffRootCbf = 0;
  double  sbtOffCost = MAX_DOUBLE;
  double  currBestCost = MAX_DOUBLE;
  bool    doPreAnalyzeResi = (sbtAllowed || mtsAllowed) && residualPass == 0;

  m_pcInterSearch->initTuAnalyzer();
  if (doPreAnalyzeResi)
  {
    m_pcInterSearch->calcMinDistSbt(*tempCS, *cu, sbtAllowed);
  }

  auto    slsSbt = dynamic_cast<SaveLoadEncInfoSbt*>(m_modeCtrl);
  int     slShift = 4 + std::min((int)gp_sizeIdxInfo->idxFrom(cu->lwidth()) + (int)gp_sizeIdxInfo->idxFrom(cu->lheight()), 9);
  Distortion curPuSse = m_pcInterSearch->getEstDistSbt(NUMBER_SBT_MODE);
  uint8_t currBestSbt = 0;
  uint8_t currBestTrs = MAX_UCHAR;
  uint8_t histBestSbt = MAX_UCHAR;
  uint8_t histBestTrs = MAX_UCHAR;
  m_pcInterSearch->setHistBestTrs(MAX_UCHAR, MAX_UCHAR);
  if (doPreAnalyzeResi)
  {
    if (m_pcInterSearch->getSkipSbtAll() && !mtsAllowed) //emt is off
    {
      histBestSbt = 0; //try DCT2
      m_pcInterSearch->setHistBestTrs(histBestSbt, histBestTrs);
    }
    else
    {
      assert(curPuSse != std::numeric_limits<uint64_t>::max());
      uint16_t compositeSbtTrs = slsSbt->findBestSbt(cu->cs->area, (uint32_t)(curPuSse >> slShift));
      histBestSbt = (compositeSbtTrs >> 0) & 0xff;
      histBestTrs = (compositeSbtTrs >> 8) & 0xff;
      if (m_pcInterSearch->getSkipSbtAll() && CU::isSbtMode(histBestSbt)) //special case, skip SBT when loading SBT
      {
        histBestSbt = 0; //try DCT2
      }
      m_pcInterSearch->setHistBestTrs(histBestSbt, histBestTrs);
    }
  }

  {
    if (reloadCU)
    {
      if (bestCost == bestCS->cost) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if (false == swapped)
      {
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
        tempCS->copyStructure(*bestCS, partitioner.chType);
        tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
        bestCost = bestCS->cost;
        cu = tempCS->getCU(partitioner.chType);
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu = tempCS->getCU(partitioner.chType);
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
    }

    reloadCU = true; // enable cu reloading
    cu->skip = false;
    cu->sbtInfo = 0;

    const bool skipResidual = residualPass == 1;
    if (skipResidual || histBestSbt == MAX_UCHAR || !CU::isSbtMode(histBestSbt))
    {
      m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, skipResidual);
      numRDOTried += mtsAllowed ? 2 : 1;
      xEncodeDontSplit(*tempCS, partitioner);

      xCheckDQP(*tempCS, partitioner);


      if (NULL != bestHasNonResi && (bestCostInternal > tempCS->cost))
      {
        bestCostInternal = tempCS->cost;
        if (!(tempCS->getPU(partitioner.chType)->mhIntraFlag))
          *bestHasNonResi = !cu->rootCbf;
      }

      if (cu->rootCbf == false)
      {
        if (tempCS->getPU(partitioner.chType)->mhIntraFlag)
        {
          tempCS->cost = MAX_DOUBLE;
          tempCS->costDbOffset = 0;
          return;
        }
      }
      currBestCost = tempCS->cost;
      sbtOffCost = tempCS->cost;
      sbtOffDist = tempCS->dist;
      sbtOffRootCbf = cu->rootCbf;
      currBestSbt = CU::getSbtInfo(cu->firstTU->mtsIdx > MTS_SKIP ? SBT_OFF_MTS : SBT_OFF_DCT, 0);
      currBestTrs = cu->firstTU->mtsIdx;
#if !JVET_O0545_MAX_TB_SIGNALLING
      if (cu->lwidth() <= MAX_TB_SIZEY && cu->lheight() <= MAX_TB_SIZEY)
      {
        CHECK(tempCS->tus.size() != 1, "tu must be only one");
      }
#endif

#if WCG_EXT
      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
      xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

    }

    uint8_t numSbtRdo = CU::numSbtModeRdo(sbtAllowed);
    //early termination if all SBT modes are not allowed
    //normative
    if (!sbtAllowed || skipResidual)
    {
      numSbtRdo = 0;
    }
    //fast algorithm
    if ((histBestSbt != MAX_UCHAR && !CU::isSbtMode(histBestSbt)) || m_pcInterSearch->getSkipSbtAll())
    {
      numSbtRdo = 0;
    }
    if (bestCost != MAX_DOUBLE && sbtOffCost != MAX_DOUBLE)
    {
      double th = 1.07;
      if (!(prevBestSbt == 0 || m_sbtCostSave[0] == MAX_DOUBLE))
      {
        assert(m_sbtCostSave[1] <= m_sbtCostSave[0]);
        th *= (m_sbtCostSave[0] / m_sbtCostSave[1]);
      }
      if (sbtOffCost > bestCost * th)
      {
        numSbtRdo = 0;
      }
    }
    if (!sbtOffRootCbf && sbtOffCost != MAX_DOUBLE)
    {
      double th = Clip3(0.05, 0.55, (27 - cu->qp) * 0.02 + 0.35);
      if (sbtOffCost < m_pcRdCost->calcRdCost((cu->lwidth() * cu->lheight()) << SCALE_BITS, 0) * th)
      {
        numSbtRdo = 0;
      }
    }

    if (histBestSbt != MAX_UCHAR && numSbtRdo != 0)
    {
      numSbtRdo = 1;
      m_pcInterSearch->initSbtRdoOrder(CU::getSbtMode(CU::getSbtIdx(histBestSbt), CU::getSbtPos(histBestSbt)));
    }

    for (int sbtModeIdx = 0; sbtModeIdx < numSbtRdo; sbtModeIdx++)
    {
      uint8_t sbtMode = m_pcInterSearch->getSbtRdoOrder(sbtModeIdx);
      uint8_t sbtIdx = CU::getSbtIdxFromSbtMode(sbtMode);
      uint8_t sbtPos = CU::getSbtPosFromSbtMode(sbtMode);

      //fast algorithm (early skip, save & load)
      if (histBestSbt == MAX_UCHAR)
      {
        uint8_t skipCode = m_pcInterSearch->skipSbtByRDCost(cu->lwidth(), cu->lheight(), cu->mtDepth, sbtIdx, sbtPos, bestCS->cost, sbtOffDist, sbtOffCost, sbtOffRootCbf);
        if (skipCode != MAX_UCHAR)
        {
          continue;
        }

        if (sbtModeIdx > 0)
        {
          uint8_t prevSbtMode = m_pcInterSearch->getSbtRdoOrder(sbtModeIdx - 1);
          //make sure the prevSbtMode is the same size as the current SBT mode (otherwise the estimated dist may not be comparable)
          if (CU::isSameSbtSize(prevSbtMode, sbtMode))
          {
            Distortion currEstDist = m_pcInterSearch->getEstDistSbt(sbtMode);
            Distortion prevEstDist = m_pcInterSearch->getEstDistSbt(prevSbtMode);
            if (currEstDist > prevEstDist * 1.15)
            {
              continue;
            }
          }
        }
      }

      //init tempCS and TU
      if (bestCost == bestCS->cost) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if (false == swapped)
      {
        tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
        tempCS->copyStructure(*bestCS, partitioner.chType);
        tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
        bestCost = bestCS->cost;
        cu = tempCS->getCU(partitioner.chType);
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu = tempCS->getCU(partitioner.chType);
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      cu->skip = false;

      //set SBT info
      cu->setSbtIdx(sbtIdx);
      cu->setSbtPos(sbtPos);

      //try residual coding
      m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, skipResidual);
      numRDOTried++;

      xEncodeDontSplit(*tempCS, partitioner);

      xCheckDQP(*tempCS, partitioner);

      if (NULL != bestHasNonResi && (bestCostInternal > tempCS->cost))
      {
        bestCostInternal = tempCS->cost;
        if (!(tempCS->getPU(partitioner.chType)->mhIntraFlag))
          *bestHasNonResi = !cu->rootCbf;
      }

      if (tempCS->cost < currBestCost)
      {
        currBestSbt = cu->sbtInfo;
        currBestTrs = tempCS->tus[cu->sbtInfo ? cu->getSbtPos() : 0]->mtsIdx;
        assert(currBestTrs == 0 || currBestTrs == 1);
        currBestCost = tempCS->cost;
      }

#if WCG_EXT
      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
      xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
    }

    if (bestCostBegin != bestCS->cost)
    {
      m_sbtCostSave[0] = sbtOffCost;
      m_sbtCostSave[1] = currBestCost;
    }
  } //end emt loop

  if (histBestSbt == MAX_UCHAR && doPreAnalyzeResi && numRDOTried > 1)
  {
    slsSbt->saveBestSbt(cu->cs->area, (uint32_t)(curPuSse >> slShift), currBestSbt, currBestTrs);
  }
  tempCS->cost = currBestCost;
  if (ETM_INTER_ME == encTestMode.type)
  {
    if (equGBiCost != NULL)
    {
      if (tempCS->cost < (*equGBiCost) && cu->GBiIdx == GBI_DEFAULT)
      {
        (*equGBiCost) = tempCS->cost;
      }
    }
    else
    {
      CHECK(equGBiCost == NULL, "equGBiCost == NULL");
    }
    if (tempCS->slice->getCheckLDC() && !cu->imv && cu->GBiIdx != GBI_DEFAULT && tempCS->cost < m_bestGbiCost[1])
    {
      if (tempCS->cost < m_bestGbiCost[0])
      {
        m_bestGbiCost[1] = m_bestGbiCost[0];
        m_bestGbiCost[0] = tempCS->cost;
        m_bestGbiIdx[1] = m_bestGbiIdx[0];
        m_bestGbiIdx[0] = cu->GBiIdx;
      }
      else
      {
        m_bestGbiCost[1] = tempCS->cost;
        m_bestGbiIdx[1] = cu->GBiIdx;
      }
    }
  }
}


void EncCu::xEncodeDontSplit(CodingStructure &cs, Partitioner &partitioner)
{
  m_CABACEstimator->resetBits();

  m_CABACEstimator->split_cu_mode(CU_DONT_SPLIT, cs, partitioner);
#if JVET_O0050_LOCAL_DUAL_TREE
  if (partitioner.treeType == TREE_C)
    CHECK(m_CABACEstimator->getEstFracBits() != 0, "must be 0 bit");
#endif

  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

}

#if REUSE_CU_RESULTS
void EncCu::xReuseCachedResult(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner)
{
  m_pcRdCost->setChromaFormat(tempCS->sps->getChromaFormatIdc());
  BestEncInfoCache* bestEncCache = dynamic_cast<BestEncInfoCache*>(m_modeCtrl);
  CHECK(!bestEncCache, "If this mode is chosen, mode controller has to implement the mode caching capabilities");
  EncTestMode cachedMode;

  if (bestEncCache->setCsFrom(*tempCS, cachedMode, partitioner))
  {
    CodingUnit& cu = *tempCS->cus.front();
    cu.shareParentPos = tempCS->sharedBndPos;
    cu.shareParentSize = tempCS->sharedBndSize;
    partitioner.setCUData(cu);

    if (CU::isIntra(cu)
#if JVET_O0119_BASE_PALETTE_444
      || CU::isPLT(cu)
#endif
      )
    {
      xReconIntraQT(cu);
    }
    else
    {
      xDeriveCUMV(cu);
      xReconInter(cu);
    }

    Distortion finalDistortion = 0;
    tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
    if (m_pcEncCfg->getUseEncDbOpt())
    {
      xCalDebCost(*tempCS, partitioner, true);
      finalDistortion = tempCS->dist;
    }
    else
    {
      const SPS &sps = *tempCS->sps;
      const int  numValidComponents = getNumberValidComponents(tempCS->area.chromaFormat);

      for (int comp = 0; comp < numValidComponents; comp++)
      {
        const ComponentID compID = ComponentID(comp);

#if JVET_O0050_LOCAL_DUAL_TREE
        if (partitioner.isSepTree(*tempCS) && toChannelType(compID) != partitioner.chType)
#else
        if (CS::isDualITree(*tempCS) && toChannelType(compID) != partitioner.chType)
#endif
        {
          continue;
        }

        CPelBuf reco = tempCS->getRecoBuf(compID);
        CPelBuf org = tempCS->getOrgBuf(compID);

#if WCG_EXT
        if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
          m_pcEncCfg->getReshaper() && (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
        {
          const CPelBuf orgLuma = tempCS->getOrgBuf(tempCS->area.blocks[COMPONENT_Y]);
          if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
          {
            const CompArea &area = cu.blocks[COMPONENT_Y];
            CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
            PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf(tmpArea);
            tmpRecLuma.copyFrom(reco);
            tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
            finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
          }
          else
            finalDistortion += m_pcRdCost->getDistPart(org, reco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
#endif
          finalDistortion += m_pcRdCost->getDistPart(org, reco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
      }
    }

    m_CABACEstimator->getCtx() = m_CurrCtx->start;
    m_CABACEstimator->resetBits();

    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->coding_unit(cu, partitioner, cuCtx);


    tempCS->dist = finalDistortion;
    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

    xEncodeDontSplit(*tempCS, partitioner);
    xCheckDQP(*tempCS, partitioner);
    xCheckBestMode(tempCS, bestCS, partitioner, cachedMode);
  }
  else
  {
    THROW("Should never happen!");
  }
}
#endif


//! \}
