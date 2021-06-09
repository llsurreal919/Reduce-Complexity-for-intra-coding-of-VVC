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

/** \file     EncAppCfg.h
    \brief    Handle encoder configuration parameters (header)
*/

#ifndef __ENCAPPCFG__
#define __ENCAPPCFG__

#include "CommonLib/CommonDef.h"

#include "EncoderLib/EncCfg.h"
#if EXTENSION_360_VIDEO
#include "AppEncHelper360/TExt360AppEncCfg.h"
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
#include "HDRLib/inc/DistortionMetric.H"
#endif

#include <sstream>
#include <vector>
//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class EncAppCfg
{
#if QP_SWITCHING_FOR_PARALLEL
public:
  template <class T>
  struct OptionalValue
  {
    bool bPresent;
    T    value;
    OptionalValue() : bPresent(false), value() { }
  };
#endif

protected:
  // file I/O
  std::string m_inputFileName;                                ///< source file name
  std::string m_bitstreamFileName;                            ///< output bitstream file
  std::string m_reconFileName;                                ///< output reconstruction file

  // Lambda modifiers
  double    m_adLambdaModifier[ MAX_TLAYER ];                 ///< Lambda modifier array for each temporal layer
  std::vector<double> m_adIntraLambdaModifier;                ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  double    m_dIntraQpFactor;                                 ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  // source specification
  int       m_iFrameRate;                                     ///< source frame-rates (Hz)
  uint32_t      m_FrameSkip;                                      ///< number of skipped frames from the beginning
  uint32_t      m_temporalSubsampleRatio;                         ///< temporal subsample ratio, 2 means code every two frames
  int       m_iSourceWidth;                                   ///< source width in pixel
  int       m_iSourceHeight;                                  ///< source height in pixel (when interlaced = field height)
#if EXTENSION_360_VIDEO
  int       m_inputFileWidth;                                 ///< width of image in input file  (this is equivalent to sourceWidth,  if sourceWidth  is not subsequently altered due to padding)
  int       m_inputFileHeight;                                ///< height of image in input file (this is equivalent to sourceHeight, if sourceHeight is not subsequently altered due to padding)
#endif
  int       m_iSourceHeightOrg;                               ///< original source height in pixel (when interlaced = frame height)

  bool      m_isField;                                        ///< enable field coding
  bool      m_isTopFieldFirst;
  bool      m_bEfficientFieldIRAPEnabled;                     ///< enable an efficient field IRAP structure.
  bool      m_bHarmonizeGopFirstFieldCoupleEnabled;

  int       m_conformanceWindowMode;
  int       m_confWinLeft;
  int       m_confWinRight;
  int       m_confWinTop;
  int       m_confWinBottom;
  int       m_framesToBeEncoded;                              ///< number of encoded frames
  int       m_aiPad[2];                                       ///< number of padded pixels for width and height
  bool      m_AccessUnitDelimiter;                            ///< add Access Unit Delimiter NAL units
  InputColourSpaceConversion m_inputColourSpaceConvert;       ///< colour space conversion to apply to input video
  bool      m_snrInternalColourSpace;                       ///< if true, then no colour space conversion is applied for snr calculation, otherwise inverse of input is applied.
  bool      m_outputInternalColourSpace;                    ///< if true, then no colour space conversion is applied for reconstructed video, otherwise inverse of input is applied.
  ChromaFormat m_InputChromaFormatIDC;

  bool      m_printMSEBasedSequencePSNR;
  bool      m_printHexPsnr;
  bool      m_printFrameMSE;
  bool      m_printSequenceMSE;
  bool      m_cabacZeroWordPaddingEnabled;
  bool      m_bClipInputVideoToRec709Range;
  bool      m_bClipOutputVideoToRec709Range;
  bool      m_packedYUVMode;                                  ///< If true, output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data

  bool      m_bIntraOnlyConstraintFlag;
  uint32_t  m_maxBitDepthConstraintIdc;
  uint32_t  m_maxChromaFormatConstraintIdc;
  bool      m_bFrameConstraintFlag;
  bool      m_bNoQtbttDualTreeIntraConstraintFlag;
  bool      m_noPartitionConstraintsOverrideConstraintFlag;
  bool      m_bNoSaoConstraintFlag;
  bool      m_bNoAlfConstraintFlag;
#if !JVET_O0525_REMOVE_PCM
  bool      m_bNoPcmConstraintFlag;
#endif
  bool      m_bNoRefWraparoundConstraintFlag;
  bool      m_bNoTemporalMvpConstraintFlag;
  bool      m_bNoSbtmvpConstraintFlag;
  bool      m_bNoAmvrConstraintFlag;
  bool      m_bNoBdofConstraintFlag;
  bool      m_noDmvrConstraintFlag;
  bool      m_bNoCclmConstraintFlag;
  bool      m_bNoMtsConstraintFlag;
  bool      m_noSbtConstraintFlag;
  bool      m_bNoAffineMotionConstraintFlag;
  bool      m_bNoGbiConstraintFlag;
  bool      m_noIbcConstraintFlag;
  bool      m_bNoMhIntraConstraintFlag;
  bool      m_noFPelMmvdConstraintFlag;
  bool      m_bNoTriangleConstraintFlag;
  bool      m_bNoLadfConstraintFlag;
  bool      m_noTransformSkipConstraintFlag;
#if JVET_O1136_TS_BDPCM_SIGNALLING
  bool      m_noBDPCMConstraintFlag;
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  bool      m_noJointCbCrConstraintFlag;
#endif
  bool      m_bNoQpDeltaConstraintFlag;
  bool      m_bNoDepQuantConstraintFlag;
  bool      m_bNoSignDataHidingConstraintFlag;

  // profile/level
  Profile::Name m_profile;
  Level::Tier   m_levelTier;
  Level::Name   m_level;
  uint32_t      m_subProfile;
  uint32_t          m_bitDepthConstraint;
  ChromaFormat  m_chromaFormatConstraint;
  bool          m_intraConstraintFlag;
  bool          m_onePictureOnlyConstraintFlag;
  bool          m_lowerBitRateConstraintFlag;
  bool          m_progressiveSourceFlag;
  bool          m_interlacedSourceFlag;
  bool          m_nonPackedConstraintFlag;
  bool          m_frameOnlyConstraintFlag;

  // coding structure
  int       m_iIntraPeriod;                                   ///< period of I-slice (random access period)
  int       m_iDecodingRefreshType;                           ///< random access type
  int       m_iGOPSize;                                       ///< GOP size of hierarchical structure
  bool      m_rewriteParamSets;                              ///< Flag to enable rewriting of parameter sets at random access points
  RPLEntry  m_RPLList0[MAX_GOP];                               ///< the RPL entries from the config file
  RPLEntry  m_RPLList1[MAX_GOP];                               ///< the RPL entries from the config file
  bool      m_idrRefParamList;                                ///< indicates if reference picture list syntax elements are present in slice headers of IDR pictures
  GOPEntry  m_GOPList[MAX_GOP];                               ///< the coding structure entries from the config file
  BrickSplit    m_brickSplits[MAX_TILES];
  BrickSplitMap m_brickSplitMap;
  int       m_numReorderPics[MAX_TLAYER];                     ///< total number of reorder pictures
  int       m_maxDecPicBuffering[MAX_TLAYER];                 ///< total number of pictures in the decoded picture buffer
  bool      m_crossComponentPredictionEnabledFlag;            ///< flag enabling the use of cross-component prediction
  bool      m_reconBasedCrossCPredictionEstimate;             ///< causes the alpha calculation in encoder search to be based on the decoded residual rather than the pre-transform encoder-side residual
  uint32_t      m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];       ///< number of bits for the upward bit shift operation on the decoded SAO offsets
  bool      m_useTransformSkip;                               ///< flag for enabling intra transform skipping
  bool      m_useTransformSkipFast;                           ///< flag for enabling fast intra transform skipping
#if JVET_O1136_TS_BDPCM_SIGNALLING
  bool      m_useBDPCM;
#endif
  uint32_t      m_log2MaxTransformSkipBlockSize;                  ///< transform-skip maximum size (minimum of 2)
  bool      m_transformSkipRotationEnabledFlag;               ///< control flag for transform-skip/transquant-bypass residual rotation
  bool      m_transformSkipContextEnabledFlag;                ///< control flag for transform-skip/transquant-bypass single significance map context
  bool      m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];///< control flags for residual DPCM
  bool      m_persistentRiceAdaptationEnabledFlag;            ///< control flag for Golomb-Rice parameter adaptation over each slice
  bool      m_cabacBypassAlignmentEnabledFlag;
  bool      m_ISP;
  bool      m_useFastISP;                                    ///< flag for enabling fast methods for ISP

  // coding quality
#if QP_SWITCHING_FOR_PARALLEL
  OptionalValue<uint32_t> m_qpIncrementAtSourceFrame;             ///< Optional source frame number at which all subsequent frames are to use an increased internal QP.
#else
  double    m_fQP;                                            ///< QP value of key-picture (floating point)
#endif
  int       m_iQP;                                            ///< QP value of key-picture (integer)
#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
  bool      m_useIdentityTableForNon420Chroma;
  ChromaQpMappingTableParams m_chromaQpMappingTableParams;
#endif
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  int       m_intraQPOffset;                                  ///< QP offset for intra slice (integer)
  bool      m_lambdaFromQPEnable;                             ///< enable flag for QP:lambda fix
#endif
  std::string m_dQPFileName;                                  ///< QP offset for each slice (initialized from external file)
  int*      m_aidQP;                                          ///< array of slice QP values
  int       m_iMaxDeltaQP;                                    ///< max. |delta QP|
  uint32_t      m_uiDeltaQpRD;                                    ///< dQP range for multi-pass slice QP optimization
  int       m_cuQpDeltaSubdiv;                                ///< Maximum subdiv for CU luma Qp adjustment (0:default)
  int       m_cuChromaQpOffsetSubdiv;                         ///< If negative, then do not apply chroma qp offsets.
  bool      m_bFastDeltaQP;                                   ///< Fast Delta QP (false:default)

  int       m_cbQpOffset;                                     ///< Chroma Cb QP Offset (0:default)
  int       m_crQpOffset;                                     ///< Chroma Cr QP Offset (0:default)
  int       m_cbQpOffsetDualTree;                             ///< Chroma Cb QP Offset for dual tree (overwrite m_cbQpOffset for dual tree)
  int       m_crQpOffsetDualTree;                             ///< Chroma Cr QP Offset for dual tree (overwrite m_crQpOffset for dual tree)
  int       m_cbCrQpOffset;                                   ///< QP Offset for joint Cb-Cr mode
  int       m_cbCrQpOffsetDualTree;                           ///< QP Offset for joint Cb-Cr mode (overwrite m_cbCrQpOffset for dual tree)
#if ER_CHROMA_QP_WCG_PPS
  WCGChromaQPControl m_wcgChromaQpControl;                    ///< Wide-colour-gamut chroma QP control.
#endif
#if W0038_CQP_ADJ
  uint32_t      m_sliceChromaQpOffsetPeriodicity;                 ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  int       m_sliceChromaQpOffsetIntraOrPeriodic[2/*Cb,Cr*/]; ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
#endif
#if SHARP_LUMA_DELTA_QP
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping;      ///< mapping from luma level to Delta QP.
#endif
  SEIMasteringDisplay m_masteringDisplay;

  bool      m_bUseAdaptiveQP;                                 ///< Flag for enabling QP adaptation based on a psycho-visual model
  int       m_iQPAdaptationRange;                             ///< dQP range by QP adaptation
#if ENABLE_QPA
  bool      m_bUsePerceptQPA;                                 ///< Flag to enable perceptually motivated input-adaptive QP modification
  bool      m_bUseWPSNR;                                      ///< Flag to output perceptually weighted peak SNR (WPSNR) instead of PSNR
#endif
  int       m_maxTempLayer;                                   ///< Max temporal layer

  // coding unit (CU) definition
  unsigned  m_uiCTUSize;
  bool      m_SplitConsOverrideEnabledFlag;
  unsigned  m_uiMinQT[3]; // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned  m_uiMaxBTDepth;
  unsigned  m_uiMaxBTDepthI;
  unsigned  m_uiMaxBTDepthIChroma;
  bool      m_dualTree;
  bool      m_LFNST;
  bool      m_useFastLFNST;
  int       m_SubPuMvpMode;
  bool      m_Affine;
  bool      m_AffineType;
#if JVET_O0070_PROF
  bool      m_PROF;
#endif
  bool      m_BIO;
  int       m_LMChroma;
  bool      m_cclmCollocatedChromaFlag;
  int       m_MTS;                                            ///< XZ: Multiple Transform Set
  int       m_MTSIntraMaxCand;                                ///< XZ: Number of additional candidates to test
  int       m_MTSInterMaxCand;                                ///< XZ: Number of additional candidates to test
  int       m_MTSImplicit;
  bool      m_SBT;                                            ///< Sub-Block Transform for inter blocks

  bool      m_SMVD;
  bool      m_compositeRefEnabled;
  bool      m_GBi;
  bool      m_GBiFast;
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool      m_LadfEnabed;
  int       m_LadfNumIntervals;
  std::vector<int> m_LadfQpOffset;
  int       m_LadfIntervalLowerBound[MAX_LADF_INTERVALS];
#endif

  bool      m_MHIntra;
  bool      m_Triangle;
  bool      m_HashME;
  bool      m_allowDisFracMMVD;
  bool      m_AffineAmvr;
  bool      m_AffineAmvrEncOpt;
  bool      m_DMVR;
  bool      m_MMVD;
  int       m_MmvdDisNum;
#if !JVET_O1136_TS_BDPCM_SIGNALLING  
  bool      m_RdpcmMode;
#endif
#if JVET_O0119_BASE_PALETTE_444
  unsigned  m_PLTMode;
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  bool      m_JointCbCrMode;
#endif
  unsigned  m_IBCMode;
  unsigned  m_IBCLocalSearchRangeX;
  unsigned  m_IBCLocalSearchRangeY;
  unsigned  m_IBCHashSearch;
  unsigned  m_IBCHashSearchMaxCand;
  unsigned  m_IBCHashSearchRange4SmallBlk;
  unsigned  m_IBCFastMethod;

  bool      m_wrapAround;
  unsigned  m_wrapAroundOffset;

  // ADD_NEW_TOOL : (encoder app) add tool enabling flags and associated parameters here
  bool      m_loopFilterAcrossVirtualBoundariesDisabledFlag;
  unsigned  m_numVerVirtualBoundaries;
  unsigned  m_numHorVirtualBoundaries;
  std::vector<unsigned> m_virtualBoundariesPosX;
  std::vector<unsigned> m_virtualBoundariesPosY;
  bool      m_lumaReshapeEnable;
  uint32_t  m_reshapeSignalType;
  uint32_t  m_intraCMD;
  ReshapeCW m_reshapeCW;
#if JVET_O0432_LMCS_ENCODER
  int       m_updateCtrl;
  int       m_adpOption;
  uint32_t  m_initialCW;
#endif
  bool      m_encDbOpt;
  unsigned  m_uiMaxCUWidth;                                   ///< max. CU width in pixel
  unsigned  m_uiMaxCUHeight;                                  ///< max. CU height in pixel
  unsigned  m_uiMaxCUDepth;                                   ///< max. CU depth (as specified by command line)
  unsigned  m_uiMaxCodingDepth;                               ///< max. total CU depth - includes depth of transform-block structure
  unsigned  m_uiLog2DiffMaxMinCodingBlockSize;                ///< difference between largest and smallest CU depth

  bool      m_useFastLCTU;
  bool      m_usePbIntraFast;
  bool      m_useAMaxBT;
  bool      m_useFastMrg;
  bool      m_e0023FastEnc;
  bool      m_contentBasedFastQtbt;
  bool      m_useNonLinearAlfLuma;
  bool      m_useNonLinearAlfChroma;
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  unsigned  m_maxNumAlfAlternativesChroma;
#endif
  bool      m_MIP;
  bool      m_useFastMIP;
#if JVET_O0050_LOCAL_DUAL_TREE
  int       m_fastLocalDualTreeMode;
#endif


  int       m_numSplitThreads;
  bool      m_forceSplitSequential;
  int       m_numWppThreads;
  int       m_numWppExtraLines;
  bool      m_ensureWppBitEqual;

#if MAX_TB_SIZE_SIGNALLING
  int       m_log2MaxTbSize;
#endif
  // coding tools (bit-depth)
  int       m_inputBitDepth   [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of input file
  int       m_outputBitDepth  [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of output file
  int       m_MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE];      ///< bit-depth of input samples after MSB extension
  int       m_internalBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth codec operates at (input/output files will be converted)
  bool      m_extendedPrecisionProcessingFlag;
  bool      m_highPrecisionOffsetsEnabledFlag;

  //coding tools (chroma format)
  ChromaFormat m_chromaFormatIDC;

#if !JVET_O0525_REMOVE_PCM
  // coding tools (PCM bit-depth)
  bool      m_bPCMInputBitDepthFlag;                          ///< 0: PCM bit-depth is internal bit-depth. 1: PCM bit-depth is input bit-depth.
#endif

  // coding tool (SAO)
  bool      m_bUseSAO;
  bool      m_bTestSAODisableAtPictureLevel;
  double    m_saoEncodingRate;                                ///< When >0 SAO early picture termination is enabled for luma and chroma
  double    m_saoEncodingRateChroma;                          ///< The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  int       m_maxNumOffsetsPerPic;                            ///< SAO maximun number of offset per picture
  bool      m_saoCtuBoundary;                                 ///< SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
  bool      m_saoGreedyMergeEnc;                              ///< SAO greedy merge encoding algorithm
  // coding tools (loop filter)
  bool      m_bLoopFilterDisable;                             ///< flag for using deblocking filter
  bool      m_loopFilterOffsetInPPS;                         ///< offset for deblocking filter in 0 = slice header, 1 = PPS
  int       m_loopFilterBetaOffsetDiv2;                     ///< beta offset for deblocking filter
  int       m_loopFilterTcOffsetDiv2;                       ///< tc offset for deblocking filter
#if W0038_DB_OPT
  int       m_deblockingFilterMetric;                         ///< blockiness metric in encoder
#else
  bool      m_DeblockingFilterMetric;                         ///< blockiness metric in encoder
#endif
#if !JVET_O0525_REMOVE_PCM
  // coding tools (PCM)
  bool      m_usePCM;                                         ///< flag for using IPCM
  uint32_t      m_pcmLog2MaxSize;                                 ///< log2 of maximum PCM block size
  uint32_t      m_uiPCMLog2MinSize;                               ///< log2 of minimum PCM block size
  bool      m_bPCMFilterDisableFlag;                          ///< PCM filter disable flag
#endif
  bool      m_enableIntraReferenceSmoothing;                  ///< flag for enabling(default)/disabling intra reference smoothing/filtering

  // coding tools (encoder-only parameters)
  bool      m_bUseASR;                                        ///< flag for using adaptive motion search range
  bool      m_bUseHADME;                                      ///< flag for using HAD in sub-pel ME
  bool      m_useRDOQ;                                       ///< flag for using RD optimized quantization
  bool      m_useRDOQTS;                                     ///< flag for using RD optimized quantization for transform skip
#if T0196_SELECTIVE_RDOQ
  bool      m_useSelectiveRDOQ;                               ///< flag for using selective RDOQ
#endif
  int       m_rdPenalty;                                      ///< RD-penalty for 32x32 TU for intra in non-intra slices (0: no RD-penalty, 1: RD-penalty, 2: maximum RD-penalty)
  bool      m_bDisableIntraPUsInInterSlices;                  ///< Flag for disabling intra predicted PUs in inter slices.
  MESearchMethod m_motionEstimationSearchMethod;
  bool      m_bRestrictMESampling;                            ///< Restrict sampling for the Selective ME
  int       m_iSearchRange;                                   ///< ME search range
  int       m_bipredSearchRange;                              ///< ME search range for bipred refinement
  int       m_minSearchWindow;                                ///< ME minimum search window size for the Adaptive Window ME
  bool      m_bClipForBiPredMeEnabled;                        ///< Enables clipping for Bi-Pred ME.
  bool      m_bFastMEAssumingSmootherMVEnabled;               ///< Enables fast ME assuming a smoother MV.
  FastInterSearchMode m_fastInterSearchMode;                  ///< Parameter that controls fast encoder settings
  bool      m_bUseEarlyCU;                                    ///< flag for using Early CU setting
  bool      m_useFastDecisionForMerge;                        ///< flag for using Fast Decision Merge RD-Cost
  bool      m_bUseCbfFastMode;                                ///< flag for using Cbf Fast PU Mode Decision
  bool      m_useEarlySkipDetection;                          ///< flag for using Early SKIP Detection
  SliceConstraint m_sliceMode;
  int             m_sliceArgument;                            ///< argument according to selected slice mode

  bool      m_bLFCrossSliceBoundaryFlag;  ///< 1: filter across slice boundaries 0: do not filter across slice boundaries

  bool      m_bLFCrossTileBoundaryFlag;   ///< 1: filter across tile boundaries  0: do not filter across tile boundaries
  bool      m_tileUniformSpacingFlag;
  int       m_numTileColumnsMinus1;
  int       m_numTileRowsMinus1;
  std::vector<int> m_tileColumnWidth;
  std::vector<int> m_tileRowHeight;
  bool      m_entropyCodingSyncEnabledFlag;

  bool      m_rectSliceFlag;
  int       m_numSlicesInPicMinus1;
  std::vector<int> m_topLeftBrickIdx;
  std::vector<int> m_bottomRightBrickIdx;
  bool      m_loopFilterAcrossSlicesEnabledFlag;
  bool      m_signalledSliceIdFlag;
  int       m_signalledSliceIdLengthMinus1;
  std::vector<int> m_sliceId;

  bool      m_bUseConstrainedIntraPred;                       ///< flag for using constrained intra prediction
  bool      m_bFastUDIUseMPMEnabled;
  bool      m_bFastMEForGenBLowDelayEnabled;
  bool      m_bUseBLambdaForNonKeyLowDelayPictures;

  HashType  m_decodedPictureHashSEIType;                      ///< Checksum mode for decoded picture hash SEI message
  bool      m_recoveryPointSEIEnabled;
  bool      m_bufferingPeriodSEIEnabled;
  bool      m_pictureTimingSEIEnabled;
  bool      m_toneMappingInfoSEIEnabled;
  bool      m_chromaResamplingFilterSEIenabled;
  int       m_chromaResamplingHorFilterIdc;
  int       m_chromaResamplingVerFilterIdc;
  int       m_toneMapId;
  bool      m_toneMapCancelFlag;
  bool      m_toneMapPersistenceFlag;
  int       m_toneMapCodedDataBitDepth;
  int       m_toneMapTargetBitDepth;
  int       m_toneMapModelId;
  int       m_toneMapMinValue;
  int       m_toneMapMaxValue;
  int       m_sigmoidMidpoint;
  int       m_sigmoidWidth;
  int       m_numPivots;
  int       m_cameraIsoSpeedIdc;
  int       m_cameraIsoSpeedValue;
  int       m_exposureIndexIdc;
  int       m_exposureIndexValue;
  bool      m_exposureCompensationValueSignFlag;
  int       m_exposureCompensationValueNumerator;
  int       m_exposureCompensationValueDenomIdc;
  int       m_refScreenLuminanceWhite;
  int       m_extendedRangeWhiteLevel;
  int       m_nominalBlackLevelLumaCodeValue;
  int       m_nominalWhiteLevelLumaCodeValue;
  int       m_extendedWhiteLevelLumaCodeValue;
  int*      m_startOfCodedInterval;
  int*      m_codedPivotValue;
  int*      m_targetPivotValue;
  bool      m_framePackingSEIEnabled;
  int       m_framePackingSEIType;
  int       m_framePackingSEIId;
  int       m_framePackingSEIQuincunx;
  int       m_framePackingSEIInterpretation;
  bool      m_segmentedRectFramePackingSEIEnabled;
  bool      m_segmentedRectFramePackingSEICancel;
  int       m_segmentedRectFramePackingSEIType;
  bool      m_segmentedRectFramePackingSEIPersistence;
  int       m_displayOrientationSEIAngle;
  bool      m_temporalLevel0IndexSEIEnabled;
  bool      m_gradualDecodingRefreshInfoEnabled;
  int       m_noDisplaySEITLayer;
  bool      m_decodingUnitInfoSEIEnabled;
  bool      m_SOPDescriptionSEIEnabled;
  bool      m_scalableNestingSEIEnabled;
  bool      m_tmctsSEIEnabled;
  bool      m_timeCodeSEIEnabled;
  int       m_timeCodeSEINumTs;
  SEITimeSet m_timeSetArray[MAX_TIMECODE_SEI_SETS];
  bool      m_kneeSEIEnabled;
  int       m_kneeSEIId;
  bool      m_kneeSEICancelFlag;
  bool      m_kneeSEIPersistenceFlag;
  int       m_kneeSEIInputDrange;
  int       m_kneeSEIInputDispLuminance;
  int       m_kneeSEIOutputDrange;
  int       m_kneeSEIOutputDispLuminance;
  int       m_kneeSEINumKneePointsMinus1;
  int*      m_kneeSEIInputKneePoint;
  int*      m_kneeSEIOutputKneePoint;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  int       m_preferredTransferCharacteristics;
#endif
  uint32_t      m_greenMetadataType;
  uint32_t      m_xsdMetricType;

  bool      m_MCTSEncConstraint;

  // weighted prediction
  bool      m_useWeightedPred;                    ///< Use of weighted prediction in P slices
  bool      m_useWeightedBiPred;                  ///< Use of bi-directional weighted prediction in B slices
  WeightedPredictionMethod m_weightedPredictionMethod;

  uint32_t      m_log2ParallelMergeLevel;                         ///< Parallel merge estimation region
  uint32_t      m_maxNumMergeCand;                                ///< Max number of merge candidates
  uint32_t      m_maxNumAffineMergeCand;                          ///< Max number of affine merge candidates
  uint32_t      m_maxNumTriangleCand;
#if JVET_O0455_IBC_MAX_MERGE_NUM
  uint32_t      m_maxNumIBCMergeCand;                             ///< Max number of IBC merge candidates
#endif

  int       m_TMVPModeId;
  bool      m_depQuantEnabledFlag;
  bool      m_signDataHidingEnabledFlag;
  bool      m_RCEnableRateControl;                ///< enable rate control or not
  int       m_RCTargetBitrate;                    ///< target bitrate when rate control is enabled
  int       m_RCKeepHierarchicalBit;              ///< 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit allocation
  bool      m_RCLCULevelRC;                       ///< true: LCU level rate control; false: picture level rate control NOTE: code-tidy - rename to m_RCCtuLevelRC
  bool      m_RCUseLCUSeparateModel;              ///< use separate R-lambda model at LCU level                        NOTE: code-tidy - rename to m_RCUseCtuSeparateModel
  int       m_RCInitialQP;                        ///< inital QP for rate control
  bool      m_RCForceIntraQP;                     ///< force all intra picture to use initial QP or not
#if U0132_TARGET_BITS_SATURATION
  bool      m_RCCpbSaturationEnabled;             ///< enable target bits saturation to avoid CPB overflow and underflow
  uint32_t      m_RCCpbSize;                          ///< CPB size
  double    m_RCInitialCpbFullness;               ///< initial CPB fullness
#endif
  ScalingListMode m_useScalingListId;                         ///< using quantization matrix
  std::string m_scalingListFileName;                          ///< quantization matrix file name
  bool      m_TransquantBypassEnabledFlag;                    ///< transquant_bypass_enabled_flag setting in PPS.
  bool      m_CUTransquantBypassFlagForce;                    ///< if transquant_bypass_enabled_flag, then, if true, all CU transquant bypass flags will be set to true.
  CostMode  m_costMode;                                       ///< Cost mode to use

  bool      m_recalculateQPAccordingToLambda;                 ///< recalculate QP value according to the lambda value
  int       m_activeParameterSetsSEIEnabled;

  bool      m_decodingParameterSetEnabled;                   ///< enable decoding parameter set

  bool      m_vuiParametersPresentFlag;                       ///< enable generation of VUI parameters
  bool      m_aspectRatioInfoPresentFlag;                     ///< Signals whether aspect_ratio_idc is present
  int       m_aspectRatioIdc;                                 ///< aspect_ratio_idc
  int       m_sarWidth;                                       ///< horizontal size of the sample aspect ratio
  int       m_sarHeight;                                      ///< vertical size of the sample aspect ratio
  bool      m_colourDescriptionPresentFlag;                   ///< Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  int       m_colourPrimaries;                                ///< Indicates chromaticity coordinates of the source primaries
  int       m_transferCharacteristics;                        ///< Indicates the opto-electronic transfer characteristics of the source
  int       m_matrixCoefficients;                             ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool      m_chromaLocInfoPresentFlag;                       ///< Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  int       m_chromaSampleLocTypeTopField;                    ///< Specifies the location of chroma samples for top field
  int       m_chromaSampleLocTypeBottomField;                 ///< Specifies the location of chroma samples for bottom field
  int       m_chromaSampleLocType;                            ///< Specifies the location of chroma samples for progressive content
  bool      m_overscanInfoPresentFlag;                        ///< Signals whether overscan_appropriate_flag is present
  bool      m_overscanAppropriateFlag;                        ///< Indicates whether conformant decoded pictures are suitable for display using overscan
  bool      m_videoSignalTypePresentFlag;                     ///< Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present
  bool      m_videoFullRangeFlag;                             ///< Indicates the black level and range of luma and chroma signals
  int       m_ImvMode;                                        ///< imv mode
  int       m_Imv4PelFast;                                    ///< imv 4-Pel fast mode
  std::string m_colourRemapSEIFileRoot;

  std::string m_summaryOutFilename;                           ///< filename to use for producing summary output file.
  std::string m_summaryPicFilenameBase;                       ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  uint32_t        m_summaryVerboseness;                           ///< Specifies the level of the verboseness of the text output.

  int         m_verbosity;

  std::string m_decodeBitstreams[2];                          ///< filename for decode bitstreams.
  int         m_debugCTU;
  int         m_switchPOC;                                    ///< dbg poc.
  int         m_switchDQP;                                    ///< switch DQP.
  int         m_fastForwardToPOC;                             ///< get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC
  bool        m_stopAfterFFtoPOC;
  bool        m_bs2ModPOCAndType;
  bool        m_forceDecodeBitstream1;

  bool        m_alf;                                          ///< Adaptive Loop Filter

#if JVET_O1164_RPR
  double      m_scalingRatioHor;
  double      m_scalingRatioVer;
  bool        m_rprEnabled;
  double      m_fractionOfFrames;                             ///< encode a fraction of the frames as specified in FramesToBeEncoded
  int         m_switchPocPeriod;
  int         m_upscaledOutput;                               ////< Output upscaled (2), decoded cropped but in full resolution buffer (1) or decoded cropped (0, default) picture for RPR.
#endif

#if EXTENSION_360_VIDEO
  TExt360AppEncCfg m_ext360;
  friend class TExt360AppEncCfg;
  friend class TExt360AppEncTop;
#endif

#if JVET_O0756_CONFIG_HDRMETRICS || JVET_O0756_CALCULATE_HDRMETRICS
#if JVET_O0756_CALCULATE_HDRMETRICS
  double      m_whitePointDeltaE[hdrtoolslib::NB_REF_WHITE];
#else
  double      m_whitePointDeltaE[3];
#endif
  double      m_maxSampleValue;
  int         m_sampleRange;
  int         m_colorPrimaries;
  bool        m_enableTFunctionLUT;
  int         m_chromaLocation;
  int         m_chromaUPFilter;
  int         m_cropOffsetLeft;
  int         m_cropOffsetTop;
  int         m_cropOffsetRight;
  int         m_cropOffsetBottom;
  bool        m_calculateHdrMetrics;
#endif

  // internal member functions
  bool  xCheckParameter ();                                   ///< check validity of configuration values
  void  xPrintParameter ();                                   ///< print configuration values
  void  xPrintUsage     ();                                   ///< print usage
public:
  EncAppCfg();
  virtual ~EncAppCfg();

public:
  void  create    ();                                         ///< create option handling class
  void  destroy   ();                                         ///< destroy option handling class
  bool  parseCfg  ( int argc, char* argv[] );                ///< parse configuration file to fill member variables

};// END CLASS DEFINITION EncAppCfg

//! \}

#endif // __ENCAPPCFG__

