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

/** \file     CABACWriter.h
 *  \brief    Writer for low level syntax
 */

#ifndef __CABACWRITER__
#define __CABACWRITER__

#include "CommonLib/BitStream.h"
#include "CommonLib/ContextModelling.h"
#include "BinEncoder.h"
#include <opencv2/opencv.hpp>

//! \ingroup EncoderLib
//! \{


class EncCu;
class CABACWriter
{
public:

#if GET_TRAINING_SET
  cv::Mat get_madp(int height, int width, cv::Mat pixel);     //function to get MADP matrix
  std::vector<int> get_context(const CodingUnit* neighbor_cu, AreaBuf<Pel>& pic_Y);  //function to get the context information

  //files to save training set
  FILE * P_data = fopen("Data_Partition.dat", "a+b");   
  FILE * P_label = fopen("Label_Partition.dat", "a+b");
  FILE * T_data = fopen("Data_Termination.dat", "a+b");
  FILE * T_label = fopen("Label_Termination.dat", "a+b");
#endif


  CABACWriter(BinEncIf& binEncoder)   : m_BinEncoder(binEncoder), m_Bitstream(0) { m_TestCtx = m_BinEncoder.getCtx(); m_EncCu = NULL; }
  virtual ~CABACWriter() {}

public:
  void        initCtxModels             ( const Slice&                  slice );
  void        setEncCu(EncCu* pcEncCu) { m_EncCu = pcEncCu; }
  SliceType   getCtxInitId              ( const Slice&                  slice );
  void        initBitstream             ( OutputBitstream*              bitstream )           { m_Bitstream = bitstream; m_BinEncoder.init( m_Bitstream ); }

  const Ctx&  getCtx                    ()                                            const   { return m_BinEncoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinEncoder.getCtx();  }

  void        start                     ()                                                    { m_BinEncoder.start(); }
  void        resetBits                 ()                                                    { m_BinEncoder.resetBits(); }
  uint64_t    getEstFracBits            ()                                            const   { return m_BinEncoder.getEstFracBits(); }
  uint32_t    getNumBins                ()                                                    { return m_BinEncoder.getNumBins(); }
  bool        isEncoding                ()                                                    { return m_BinEncoder.isEncoding(); }

public:
  // slice segment data (clause 7.3.8.1)
  void        end_of_slice              ();

  // coding tree unit (clause 7.3.8.2)
  void        coding_tree_unit          (       CodingStructure&        cs,       const UnitArea&   area,       int (&qps)[2],  unsigned ctuRsAddr,  bool skipSao = false, bool skipAlf = false );

  // sao (clause 7.3.8.3)
  void        sao                       ( const Slice&                  slice,    unsigned          ctuRsAddr );
  void        sao_block_pars            ( const SAOBlkParam&            saoPars,  const BitDepths&  bitDepths,  bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo );
  void        sao_offset_pars           ( const SAOOffset&              ctbPars,  ComponentID       compID,     bool sliceEnabled,  int bitDepth );
  // coding (quad)tree (clause 7.3.8.4)
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx, Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
  void        split_cu_mode             ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );
#if JVET_O0050_LOCAL_DUAL_TREE
  void        mode_constraint           ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm,    const ModeType modeType );
#endif

  // coding unit (clause 7.3.8.5)
  void        coding_unit               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        cu_transquant_bypass_flag ( const CodingUnit&             cu );
  void        cu_skip_flag              ( const CodingUnit&             cu );
  void        pred_mode                 ( const CodingUnit&             cu );
  void        bdpcm_mode                ( const CodingUnit&             cu,       const ComponentID compID );

#if !JVET_O0525_REMOVE_PCM
  void        pcm_data                  ( const CodingUnit&             cu,       Partitioner&      pm );
  void        pcm_flag                  ( const CodingUnit&             cu,       Partitioner&      pm );
#endif
  void        cu_pred_data              ( const CodingUnit&             cu );
  void        cu_gbi_flag               ( const CodingUnit&             cu );
  void        extend_ref_line           (const PredictionUnit&          pu );
  void        extend_ref_line           (const CodingUnit&              cu );
  void        intra_luma_pred_modes     ( const CodingUnit&             cu );
  void        intra_luma_pred_mode      ( const PredictionUnit&         pu );
  void        intra_chroma_pred_modes   ( const CodingUnit&             cu );
  void        intra_chroma_lmc_mode     ( const PredictionUnit&         pu );
  void        intra_chroma_pred_mode    ( const PredictionUnit&         pu );
  void        cu_residual               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        rqt_root_cbf              ( const CodingUnit&             cu );
  void        sbt_mode                  ( const CodingUnit&             cu );
  void        end_of_ctu                ( const CodingUnit&             cu,       CUCtx&            cuCtx );
  void        mip_flag                  ( const CodingUnit&             cu );
  void        mip_pred_modes            ( const CodingUnit&             cu );
  void        mip_pred_mode             ( const PredictionUnit&         pu );
#if JVET_O0119_BASE_PALETTE_444
  void        cu_palette_info           ( const CodingUnit&             cu,       ComponentID       compBegin,     uint32_t numComp,          CUCtx&       cuCtx);
  void        cu_run_val                (       uint32_t                run,      PLTRunMode        runtype, const uint32_t paletteIdx, const uint32_t     maxRun);
  void        encodeRunType             ( const CodingUnit&             cu,       PLTtypeBuf&       runType,       uint32_t idx,              ScanElement *refScanOrder,   ComponentID compBegin);
  Pel         writePLTIndex             ( const CodingUnit&             cu,       uint32_t          idx,           PelBuf&  paletteIdx,       PLTtypeBuf&  paletteRunType, int         maxSymbol,   ComponentID compBegin );
#endif
  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( const PredictionUnit&         pu );
  void        merge_flag                ( const PredictionUnit&         pu );
#if JVET_O0249_MERGE_SYNTAX
  void        merge_data                ( const PredictionUnit&         pu );
#endif
  void        affine_flag               ( const CodingUnit&             cu );
  void        subblock_merge_flag       ( const CodingUnit&             cu );
  void        merge_idx                 ( const PredictionUnit&         pu );
  void        mmvd_merge_idx(const PredictionUnit&         pu);
  void        imv_mode                  ( const CodingUnit&             cu );
  void        affine_amvr_mode          ( const CodingUnit&             cu );
  void        inter_pred_idc            ( const PredictionUnit&         pu );
  void        ref_idx                   ( const PredictionUnit&         pu,       RefPicList        eRefList );
  void        mvp_flag                  ( const PredictionUnit&         pu,       RefPicList        eRefList );

  void        MHIntra_flag              ( const PredictionUnit&         pu );
  void        MHIntra_luma_pred_modes   ( const CodingUnit&             cu );
  void        smvd_mode              ( const PredictionUnit&         pu );

#if !JVET_O0525_REMOVE_PCM
  // pcm samples (clause 7.3.8.7)
  void        pcm_samples               ( const TransformUnit&          tu );
#endif

  // transform tree (clause 7.3.8.8)
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  void        transform_tree            ( const CodingStructure&        cs,       Partitioner&      pm,     CUCtx& cuCtx,                         const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1 );
#else
  void        transform_tree            ( const CodingStructure&        cs,       Partitioner&      pm,     CUCtx& cuCtx, ChromaCbfs& chromaCbfs, const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1 );
#endif
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, unsigned depth, const bool prevCbf = false, const bool useISP = false );

  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( const Mv &rMvd, int8_t imv );
  // transform unit (clause 7.3.8.10)
#if JVET_O0596_CBF_SIG_ALIGN_TO_SPEC
  void        transform_unit            ( const TransformUnit&          tu,       CUCtx&            cuCtx,  Partitioner& pm,       const int subTuCounter = -1 );
#else
  void        transform_unit            ( const TransformUnit&          tu,       CUCtx&            cuCtx,  ChromaCbfs& chromaCbfs );
#endif
  void        cu_qp_delta               ( const CodingUnit&             cu,       int               predQP, const int8_t qp );
  void        cu_chroma_qp_offset       ( const CodingUnit&             cu );

  // residual coding (clause 7.3.8.11)
#if JVET_O0094_LFNST_ZERO_PRIM_COEFFS || JVET_O0472_LFNST_SIGNALLING_LAST_SCAN_POS
  void        residual_coding           ( const TransformUnit&          tu,       ComponentID       compID, CUCtx* cuCtx = nullptr );
#else
  void        residual_coding           ( const TransformUnit&          tu,       ComponentID       compID );
#endif
  void        mts_coding                ( const TransformUnit&          tu,       ComponentID       compID );
  void        residual_lfnst_mode       ( const CodingUnit&             cu,       CUCtx&            cuCtx );
  void        isp_mode                  ( const CodingUnit&             cu );
  void        explicit_rdpcm_mode       ( const TransformUnit&          tu,       ComponentID       compID );
  void        last_sig_coeff            ( CoeffCodingContext&           cctx,     const TransformUnit& tu, ComponentID       compID );
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,     const TCoeff*     coeff, const int stateTransTable, int& state );
  void        residual_codingTS         ( const TransformUnit&          tu,       ComponentID       compID );
  void        residual_coding_subblockTS( CoeffCodingContext&           cctx,     const TCoeff*     coeff  );
#if JVET_O0105_ICT
  void        joint_cb_cr               ( const TransformUnit&          tu,       const int cbfMask );
#else
  void        joint_cb_cr               ( const TransformUnit&          tu );
#endif

  // cross component prediction (clause 7.3.8.12)
  void        cross_comp_pred           ( const TransformUnit&          tu,       ComponentID       compID );

  void        codeAlfCtuEnableFlags     ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam);
  void        codeAlfCtuEnableFlags     ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam);
  void        codeAlfCtuEnableFlag      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfParam* alfParam );
  void        codeAlfCtuFilterIndex(CodingStructure& cs, uint32_t ctuRsAddr, bool alfEnableLuma);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB

  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam);
  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam);
  void        codeAlfCtuAlternative      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, const AlfParam* alfParam = NULL );
#endif

private:
  void        unary_max_symbol          ( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  void        unary_max_eqprob          ( unsigned symbol,                                   unsigned maxSymbol );
  void        exp_golomb_eqprob         ( unsigned symbol, unsigned count );
  void        code_unary_fixed          ( unsigned symbol, unsigned ctxId, unsigned unary_max, unsigned fixed );

  // statistic
  unsigned    get_num_written_bits()    { return m_BinEncoder.getNumWrittenBits(); }

  void  xWriteTruncBinCode(uint32_t uiSymbol, uint32_t uiMaxSymbol);
#if JVET_O0119_BASE_PALETTE_444
  void        codeScanRotationModeFlag   ( const CodingUnit& cu,     ComponentID compBegin);
  void        xEncodePLTPredIndicator    ( const CodingUnit& cu,     uint32_t    maxPltSize, ComponentID compBegin);
  uint32_t    xWriteTruncMsbP1           (       uint32_t    symbol, PLTRunMode  runtype,    uint32_t    maxVal,   uint32_t ctxT);
  void        xWriteTruncMsbP1RefinementBits(    uint32_t    symbol, PLTRunMode  runtype,    uint32_t    maxVal,   uint32_t ctxT);
#endif
private:
  BinEncIf&         m_BinEncoder;
  OutputBitstream*  m_Bitstream;
  Ctx               m_TestCtx;
  EncCu*            m_EncCu;
#if JVET_O0119_BASE_PALETTE_444
  ScanElement*      m_scanOrder;
#endif
};



class CABACEncoder
{
public:
  CABACEncoder()
    : m_CABACWriterStd      ( m_BinEncoderStd )
    , m_CABACEstimatorStd   ( m_BitEstimatorStd )
    , m_CABACWriter         { &m_CABACWriterStd,   }
    , m_CABACEstimator      { &m_CABACEstimatorStd }
  {}

  CABACWriter*                getCABACWriter          ( const SPS*   sps   )        { return m_CABACWriter   [0]; }
  CABACWriter*                getCABACEstimator       ( const SPS*   sps   )        { return m_CABACEstimator[0]; }
private:
  BinEncoder_Std      m_BinEncoderStd;
  BitEstimator_Std    m_BitEstimatorStd;
  CABACWriter         m_CABACWriterStd;
  CABACWriter         m_CABACEstimatorStd;
  CABACWriter*        m_CABACWriter   [BPM_NUM-1];
  CABACWriter*        m_CABACEstimator[BPM_NUM-1];
};

//! \}

#endif //__CABACWRITER__
