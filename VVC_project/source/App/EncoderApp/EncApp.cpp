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

/** \file     EncApp.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <iomanip>

#include "EncApp.h"
#include "EncoderLib/AnnexBwrite.h"
#if EXTENSION_360_VIDEO
#include "AppEncHelper360/TExt360AppEncTop.h"
#endif

using namespace std;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

EncApp::EncApp()
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif
}

EncApp::~EncApp()
{
}

void EncApp::xInitLibCfg()
{
  VPS vps;

  vps.setMaxLayers                                               ( 1 );
  for(int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setVPSIncludedLayerId                                    ( 0, i );
  }
  vps.setVPSExtensionFlag                                        ( false );
  m_cEncLib.setVPS(&vps);
  m_cEncLib.setProfile                                           ( m_profile);
  m_cEncLib.setLevel                                             ( m_levelTier, m_level);
  m_cEncLib.setSubProfile                                        ( m_subProfile );
  m_cEncLib.setProgressiveSourceFlag                             ( m_progressiveSourceFlag);
  m_cEncLib.setInterlacedSourceFlag                              ( m_interlacedSourceFlag);
  m_cEncLib.setNonPackedConstraintFlag                           ( m_nonPackedConstraintFlag);
  m_cEncLib.setFrameOnlyConstraintFlag                           ( m_frameOnlyConstraintFlag);
  m_cEncLib.setBitDepthConstraintValue                           ( m_bitDepthConstraint );
  m_cEncLib.setChromaFormatConstraintValue                       ( m_chromaFormatConstraint );
  m_cEncLib.setIntraConstraintFlag                               ( m_intraConstraintFlag );
  m_cEncLib.setOnePictureOnlyConstraintFlag                      ( m_onePictureOnlyConstraintFlag );
  m_cEncLib.setLowerBitRateConstraintFlag                        ( m_lowerBitRateConstraintFlag );

  m_cEncLib.setPrintMSEBasedSequencePSNR                         ( m_printMSEBasedSequencePSNR);
  m_cEncLib.setPrintFrameMSE                                     ( m_printFrameMSE);
  m_cEncLib.setPrintHexPsnr(m_printHexPsnr);
  m_cEncLib.setPrintSequenceMSE                                  ( m_printSequenceMSE);
  m_cEncLib.setCabacZeroWordPaddingEnabled                       ( m_cabacZeroWordPaddingEnabled );

  m_cEncLib.setFrameRate                                         ( m_iFrameRate );
  m_cEncLib.setFrameSkip                                         ( m_FrameSkip );
  m_cEncLib.setTemporalSubsampleRatio                            ( m_temporalSubsampleRatio );
  m_cEncLib.setSourceWidth                                       ( m_iSourceWidth );
  m_cEncLib.setSourceHeight                                      ( m_iSourceHeight );
#if JVET_O1164_RPR
  m_cEncLib.setConformanceWindow                                 ( m_confWinLeft / SPS::getWinUnitX( m_InputChromaFormatIDC ), m_confWinRight / SPS::getWinUnitX( m_InputChromaFormatIDC ), m_confWinTop / SPS::getWinUnitY( m_InputChromaFormatIDC ), m_confWinBottom / SPS::getWinUnitY( m_InputChromaFormatIDC ) );
  m_cEncLib.setScalingRatio                                      ( m_scalingRatioHor, m_scalingRatioVer );
  m_cEncLib.setRPREnabled                                        ( m_rprEnabled );
  m_cEncLib.setSwitchPocPeriod                                   ( m_switchPocPeriod );
  m_cEncLib.setUpscaledOutput                                    ( m_upscaledOutput );
#else
  m_cEncLib.setConformanceWindow                                 ( m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
#endif
  m_cEncLib.setFramesToBeEncoded                                 ( m_framesToBeEncoded );

  //====== SPS constraint flags =======
  m_cEncLib.setIntraOnlyConstraintFlag                           ( m_intraConstraintFlag );
  m_cEncLib.setMaxBitDepthConstraintIdc                          ( m_bitDepthConstraint - 8 );
  m_cEncLib.setMaxChromaFormatConstraintIdc                      ( m_chromaFormatConstraint );
  m_cEncLib.setFrameConstraintFlag                               ( m_bFrameConstraintFlag );
  m_cEncLib.setNoQtbttDualTreeIntraConstraintFlag                ( !m_dualTree );
  m_cEncLib.setNoPartitionConstraintsOverrideConstraintFlag      ( !m_SplitConsOverrideEnabledFlag );
  m_cEncLib.setNoSaoConstraintFlag                               ( !m_bUseSAO );
  m_cEncLib.setNoAlfConstraintFlag                               ( !m_alf );
#if !JVET_O0525_REMOVE_PCM
  m_cEncLib.setNoPcmConstraintFlag                               ( !m_usePCM );
#endif
  m_cEncLib.setNoRefWraparoundConstraintFlag                     ( m_bNoRefWraparoundConstraintFlag );
  m_cEncLib.setNoTemporalMvpConstraintFlag                       ( m_TMVPModeId ? false : true );
  m_cEncLib.setNoSbtmvpConstraintFlag                            ( m_SubPuMvpMode ? false : true );
  m_cEncLib.setNoAmvrConstraintFlag                              ( m_bNoAmvrConstraintFlag );
  m_cEncLib.setNoBdofConstraintFlag                              ( !m_BIO );
  m_cEncLib.setNoDmvrConstraintFlag                              ( !m_DMVR );
  m_cEncLib.setNoCclmConstraintFlag                              ( m_LMChroma ? false : true );
  m_cEncLib.setNoMtsConstraintFlag                               ( (m_MTS || m_MTSImplicit) ? false : true );
  m_cEncLib.setNoSbtConstraintFlag                               ( !m_SBT );
  m_cEncLib.setNoAffineMotionConstraintFlag                      ( !m_Affine );
  m_cEncLib.setNoGbiConstraintFlag                               ( !m_GBi );
  m_cEncLib.setNoIbcConstraintFlag                               ( m_IBCMode ? false : true );
  m_cEncLib.setNoMhIntraConstraintFlag                           ( !m_MHIntra );
  m_cEncLib.setNoFPelMmvdConstraintFlag                          ( !(m_MMVD && m_allowDisFracMMVD) );
  m_cEncLib.setNoTriangleConstraintFlag                          ( !m_Triangle );
  m_cEncLib.setNoLadfConstraintFlag                              ( !m_LadfEnabed );
  m_cEncLib.setNoTransformSkipConstraintFlag                     ( !m_useTransformSkip );
#if JVET_O1136_TS_BDPCM_SIGNALLING
  m_cEncLib.setNoBDPCMConstraintFlag                             ( !m_useBDPCM );
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  m_cEncLib.setNoJointCbCrConstraintFlag                         (!m_JointCbCrMode);
#endif
  m_cEncLib.setNoQpDeltaConstraintFlag                           ( m_bNoQpDeltaConstraintFlag );
  m_cEncLib.setNoDepQuantConstraintFlag                          ( !m_depQuantEnabledFlag);
  m_cEncLib.setNoSignDataHidingConstraintFlag                    ( !m_signDataHidingEnabledFlag );

  //====== Coding Structure ========
  m_cEncLib.setIntraPeriod                                       ( m_iIntraPeriod );
  m_cEncLib.setDecodingRefreshType                               ( m_iDecodingRefreshType );
  m_cEncLib.setGOPSize                                           ( m_iGOPSize );
  m_cEncLib.setReWriteParamSets                                  ( m_rewriteParamSets );
  m_cEncLib.setRPLList0                                          ( m_RPLList0);
  m_cEncLib.setRPLList1                                          ( m_RPLList1);
  m_cEncLib.setIDRRefParamListPresent                            ( m_idrRefParamList );
  m_cEncLib.setGopList                                           ( m_GOPList );

  for(int i = 0; i < MAX_TLAYER; i++)
  {
    m_cEncLib.setNumReorderPics                                  ( m_numReorderPics[i], i );
    m_cEncLib.setMaxDecPicBuffering                              ( m_maxDecPicBuffering[i], i );
  }
  for( uint32_t uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cEncLib.setLambdaModifier                                  ( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cEncLib.setIntraLambdaModifier                               ( m_adIntraLambdaModifier );
  m_cEncLib.setIntraQpFactor                                     ( m_dIntraQpFactor );

  m_cEncLib.setBaseQP                                            ( m_iQP );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_cEncLib.setIntraQPOffset                                     ( m_intraQPOffset );
  m_cEncLib.setLambdaFromQPEnable                                ( m_lambdaFromQPEnable );
#endif
#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
  m_cEncLib.setChromaQpMappingTableParams                         (m_chromaQpMappingTableParams);

#endif
  m_cEncLib.setPad                                               ( m_aiPad );

  m_cEncLib.setAccessUnitDelimiter                               ( m_AccessUnitDelimiter );

  m_cEncLib.setMaxTempLayer                                      ( m_maxTempLayer );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
  m_cEncLib.setLoopFilterDisable                                 ( m_bLoopFilterDisable       );
  m_cEncLib.setLoopFilterOffsetInPPS                             ( m_loopFilterOffsetInPPS );
  m_cEncLib.setLoopFilterBetaOffset                              ( m_loopFilterBetaOffsetDiv2  );
  m_cEncLib.setLoopFilterTcOffset                                ( m_loopFilterTcOffsetDiv2    );
#if W0038_DB_OPT
  m_cEncLib.setDeblockingFilterMetric                            ( m_deblockingFilterMetric );
#else
  m_cEncLib.setDeblockingFilterMetric                            ( m_DeblockingFilterMetric );
#endif

  //====== Motion search ========
  m_cEncLib.setDisableIntraPUsInInterSlices                      ( m_bDisableIntraPUsInInterSlices );
  m_cEncLib.setMotionEstimationSearchMethod                      ( m_motionEstimationSearchMethod  );
  m_cEncLib.setSearchRange                                       ( m_iSearchRange );
  m_cEncLib.setBipredSearchRange                                 ( m_bipredSearchRange );
  m_cEncLib.setClipForBiPredMeEnabled                            ( m_bClipForBiPredMeEnabled );
  m_cEncLib.setFastMEAssumingSmootherMVEnabled                   ( m_bFastMEAssumingSmootherMVEnabled );
  m_cEncLib.setMinSearchWindow                                   ( m_minSearchWindow );
  m_cEncLib.setRestrictMESampling                                ( m_bRestrictMESampling );

  //====== Quality control ========
  m_cEncLib.setMaxDeltaQP                                        ( m_iMaxDeltaQP  );
  m_cEncLib.setCuQpDeltaSubdiv                                   ( m_cuQpDeltaSubdiv );
  m_cEncLib.setCuChromaQpOffsetSubdiv                            ( m_cuChromaQpOffsetSubdiv );
  m_cEncLib.setChromaCbQpOffset                                  ( m_cbQpOffset     );
  m_cEncLib.setChromaCrQpOffset                                  ( m_crQpOffset  );
  m_cEncLib.setChromaCbQpOffsetDualTree                          ( m_cbQpOffsetDualTree );
  m_cEncLib.setChromaCrQpOffsetDualTree                          ( m_crQpOffsetDualTree );
  m_cEncLib.setChromaCbCrQpOffset                                ( m_cbCrQpOffset         );
  m_cEncLib.setChromaCbCrQpOffsetDualTree                        ( m_cbCrQpOffsetDualTree );
#if ER_CHROMA_QP_WCG_PPS
  m_cEncLib.setWCGChromaQpControl                                ( m_wcgChromaQpControl );
#endif
#if W0038_CQP_ADJ
  m_cEncLib.setSliceChromaOffsetQpIntraOrPeriodic                ( m_sliceChromaQpOffsetPeriodicity, m_sliceChromaQpOffsetIntraOrPeriodic );
#endif
  m_cEncLib.setChromaFormatIdc                                   ( m_chromaFormatIDC  );
  m_cEncLib.setUseAdaptiveQP                                     ( m_bUseAdaptiveQP  );
  m_cEncLib.setQPAdaptationRange                                 ( m_iQPAdaptationRange );
#if ENABLE_QPA
  m_cEncLib.setUsePerceptQPA                                     ( m_bUsePerceptQPA && !m_bUseAdaptiveQP );
  m_cEncLib.setUseWPSNR                                          ( m_bUseWPSNR );
#endif
  m_cEncLib.setExtendedPrecisionProcessingFlag                   ( m_extendedPrecisionProcessingFlag );
  m_cEncLib.setHighPrecisionOffsetsEnabledFlag                   ( m_highPrecisionOffsetsEnabledFlag );

  m_cEncLib.setWeightedPredictionMethod( m_weightedPredictionMethod );

  //====== Tool list ========
#if SHARP_LUMA_DELTA_QP
  m_cEncLib.setLumaLevelToDeltaQPControls                        ( m_lumaLevelToDeltaQPMapping );
#endif
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_cEncLib.setDeltaQpRD( (m_costMode==COST_LOSSLESS_CODING) ? 0 : m_uiDeltaQpRD );
#else
  m_cEncLib.setDeltaQpRD                                         ( m_uiDeltaQpRD  );
#endif
  m_cEncLib.setFastDeltaQp                                       ( m_bFastDeltaQP  );
  m_cEncLib.setUseASR                                            ( m_bUseASR      );
  m_cEncLib.setUseHADME                                          ( m_bUseHADME    );
  m_cEncLib.setdQPs                                              ( m_aidQP        );
  m_cEncLib.setUseRDOQ                                           ( m_useRDOQ     );
  m_cEncLib.setUseRDOQTS                                         ( m_useRDOQTS   );
#if T0196_SELECTIVE_RDOQ
  m_cEncLib.setUseSelectiveRDOQ                                  ( m_useSelectiveRDOQ );
#endif
  m_cEncLib.setRDpenalty                                         ( m_rdPenalty );
  m_cEncLib.setCTUSize                                           ( m_uiCTUSize );
  m_cEncLib.setUseSplitConsOverride                              ( m_SplitConsOverrideEnabledFlag );
  m_cEncLib.setMinQTSizes                                        ( m_uiMinQT );
  m_cEncLib.setMaxBTDepth                                        ( m_uiMaxBTDepth, m_uiMaxBTDepthI, m_uiMaxBTDepthIChroma );
  m_cEncLib.setDualITree                                         ( m_dualTree );
  m_cEncLib.setLFNST                                             ( m_LFNST );
  m_cEncLib.setUseFastLFNST                                      ( m_useFastLFNST );
  m_cEncLib.setSubPuMvpMode                                      ( m_SubPuMvpMode );
  m_cEncLib.setAffine                                            ( m_Affine );
  m_cEncLib.setAffineType                                        ( m_AffineType );
#if JVET_O0070_PROF
  m_cEncLib.setPROF                                              ( m_PROF );
#endif
  m_cEncLib.setBIO                                               (m_BIO);
  m_cEncLib.setUseLMChroma                                       ( m_LMChroma );
  m_cEncLib.setCclmCollocatedChromaFlag                          ( m_cclmCollocatedChromaFlag );
  m_cEncLib.setIntraMTS                                          ( m_MTS & 1 );
  m_cEncLib.setIntraMTSMaxCand                                   ( m_MTSIntraMaxCand );
  m_cEncLib.setInterMTS                                          ( ( m_MTS >> 1 ) & 1 );
  m_cEncLib.setInterMTSMaxCand                                   ( m_MTSInterMaxCand );
  m_cEncLib.setImplicitMTS                                       ( m_MTSImplicit );
  m_cEncLib.setUseSBT                                            ( m_SBT );
  m_cEncLib.setUseCompositeRef                                   ( m_compositeRefEnabled );
  m_cEncLib.setUseSMVD                                           ( m_SMVD );
  m_cEncLib.setUseGBi                                            ( m_GBi );
  m_cEncLib.setUseGBiFast                                        ( m_GBiFast );
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  m_cEncLib.setUseLadf                                           ( m_LadfEnabed );
  if ( m_LadfEnabed )
  {
    m_cEncLib.setLadfNumIntervals                                ( m_LadfNumIntervals);
    for ( int k = 0; k < m_LadfNumIntervals; k++ )
    {
      m_cEncLib.setLadfQpOffset( m_LadfQpOffset[k], k );
      m_cEncLib.setLadfIntervalLowerBound(m_LadfIntervalLowerBound[k], k);
    }
  }
#endif
  m_cEncLib.setUseMHIntra                                        ( m_MHIntra );
  m_cEncLib.setUseTriangle                                       ( m_Triangle );
  m_cEncLib.setUseHashME                                         ( m_HashME );

  m_cEncLib.setAllowDisFracMMVD                                  ( m_allowDisFracMMVD );
  m_cEncLib.setUseAffineAmvr                                     ( m_AffineAmvr );
  m_cEncLib.setUseAffineAmvrEncOpt                               ( m_AffineAmvrEncOpt );
  m_cEncLib.setDMVR                                              ( m_DMVR );
  m_cEncLib.setMMVD                                              ( m_MMVD );
  m_cEncLib.setMmvdDisNum                                        (m_MmvdDisNum);
#if !JVET_O1136_TS_BDPCM_SIGNALLING 
  m_cEncLib.setRDPCM                                             ( m_RdpcmMode );
#endif  
#if JVET_O0119_BASE_PALETTE_444
  m_cEncLib.setPLTMode                                           ( m_PLTMode );
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  m_cEncLib.setJointCbCr                                         ( m_JointCbCrMode );
#endif
  m_cEncLib.setIBCMode                                           ( m_IBCMode );
  m_cEncLib.setIBCLocalSearchRangeX                              ( m_IBCLocalSearchRangeX );
  m_cEncLib.setIBCLocalSearchRangeY                              ( m_IBCLocalSearchRangeY );
  m_cEncLib.setIBCHashSearch                                     ( m_IBCHashSearch );
  m_cEncLib.setIBCHashSearchMaxCand                              ( m_IBCHashSearchMaxCand );
  m_cEncLib.setIBCHashSearchRange4SmallBlk                       ( m_IBCHashSearchRange4SmallBlk );
  m_cEncLib.setIBCFastMethod                                     ( m_IBCFastMethod );

  m_cEncLib.setUseWrapAround                                     ( m_wrapAround );
  m_cEncLib.setWrapAroundOffset                                  ( m_wrapAroundOffset );

  // ADD_NEW_TOOL : (encoder app) add setting of tool enabling flags and associated parameters here

  m_cEncLib.setLoopFilterAcrossVirtualBoundariesDisabledFlag     ( m_loopFilterAcrossVirtualBoundariesDisabledFlag );
  m_cEncLib.setNumVerVirtualBoundaries                           ( m_numVerVirtualBoundaries );
  m_cEncLib.setNumHorVirtualBoundaries                           ( m_numHorVirtualBoundaries );
  for( unsigned i = 0; i < m_numVerVirtualBoundaries; i++ )
  {
    m_cEncLib.setVirtualBoundariesPosX                           ( m_virtualBoundariesPosX[ i ], i );
  }
  for( unsigned i = 0; i < m_numHorVirtualBoundaries; i++ )
  {
    m_cEncLib.setVirtualBoundariesPosY                           ( m_virtualBoundariesPosY[ i ], i );
  }

  m_cEncLib.setMaxCUWidth                                        ( m_uiCTUSize );
  m_cEncLib.setMaxCUHeight                                       ( m_uiCTUSize );
  m_cEncLib.setMaxCodingDepth                                    ( m_uiMaxCodingDepth );
  m_cEncLib.setLog2DiffMaxMinCodingBlockSize                     ( m_uiLog2DiffMaxMinCodingBlockSize );
#if MAX_TB_SIZE_SIGNALLING
  m_cEncLib.setLog2MaxTbSize                                     ( m_log2MaxTbSize );
#endif
  m_cEncLib.setUseEncDbOpt(m_encDbOpt);
  m_cEncLib.setUseFastLCTU                                       ( m_useFastLCTU );
  m_cEncLib.setFastInterSearchMode                               ( m_fastInterSearchMode );
  m_cEncLib.setUseEarlyCU                                        ( m_bUseEarlyCU  );
  m_cEncLib.setUseFastDecisionForMerge                           ( m_useFastDecisionForMerge  );
  m_cEncLib.setUseCbfFastMode                                    ( m_bUseCbfFastMode  );
  m_cEncLib.setUseEarlySkipDetection                             ( m_useEarlySkipDetection );
  m_cEncLib.setUseFastMerge                                      ( m_useFastMrg );
  m_cEncLib.setUsePbIntraFast                                    ( m_usePbIntraFast );
  m_cEncLib.setUseAMaxBT                                         ( m_useAMaxBT );
  m_cEncLib.setUseE0023FastEnc                                   ( m_e0023FastEnc );
  m_cEncLib.setUseContentBasedFastQtbt                           ( m_contentBasedFastQtbt );
  m_cEncLib.setUseNonLinearAlfLuma                               ( m_useNonLinearAlfLuma );
  m_cEncLib.setUseNonLinearAlfChroma                             ( m_useNonLinearAlfChroma );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  m_cEncLib.setMaxNumAlfAlternativesChroma                       ( m_maxNumAlfAlternativesChroma );
#endif
  m_cEncLib.setUseMIP                                            ( m_MIP );
  m_cEncLib.setUseFastMIP                                        ( m_useFastMIP );
#if JVET_O0050_LOCAL_DUAL_TREE
  m_cEncLib.setFastLocalDualTreeMode                             ( m_fastLocalDualTreeMode );
#endif
  m_cEncLib.setCrossComponentPredictionEnabledFlag               ( m_crossComponentPredictionEnabledFlag );
  m_cEncLib.setUseReconBasedCrossCPredictionEstimate             ( m_reconBasedCrossCPredictionEstimate );
  m_cEncLib.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_LUMA  , m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA]   );
  m_cEncLib.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_CHROMA, m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
  m_cEncLib.setUseTransformSkip                                  ( m_useTransformSkip      );
  m_cEncLib.setUseTransformSkipFast                              ( m_useTransformSkipFast  );
#if JVET_O1136_TS_BDPCM_SIGNALLING
  m_cEncLib.setUseBDPCM                                          ( m_useBDPCM );
#endif
  m_cEncLib.setTransformSkipRotationEnabledFlag                  ( m_transformSkipRotationEnabledFlag );
  m_cEncLib.setTransformSkipContextEnabledFlag                   ( m_transformSkipContextEnabledFlag   );
  m_cEncLib.setPersistentRiceAdaptationEnabledFlag               ( m_persistentRiceAdaptationEnabledFlag );
  m_cEncLib.setCabacBypassAlignmentEnabledFlag                   ( m_cabacBypassAlignmentEnabledFlag );
  m_cEncLib.setLog2MaxTransformSkipBlockSize                     ( m_log2MaxTransformSkipBlockSize  );
  for (uint32_t signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_cEncLib.setRdpcmEnabledFlag                                ( RDPCMSignallingMode(signallingModeIndex), m_rdpcmEnabledFlag[signallingModeIndex]);
  }
  m_cEncLib.setUseConstrainedIntraPred                           ( m_bUseConstrainedIntraPred );
  m_cEncLib.setFastUDIUseMPMEnabled                              ( m_bFastUDIUseMPMEnabled );
  m_cEncLib.setFastMEForGenBLowDelayEnabled                      ( m_bFastMEForGenBLowDelayEnabled );
  m_cEncLib.setUseBLambdaForNonKeyLowDelayPictures               ( m_bUseBLambdaForNonKeyLowDelayPictures );
#if !JVET_O0525_REMOVE_PCM
  m_cEncLib.setPCMLog2MinSize                                    ( m_uiPCMLog2MinSize);
  m_cEncLib.setUsePCM                                            ( m_usePCM );
#endif
  m_cEncLib.setUseISP                                            ( m_ISP );
  m_cEncLib.setUseFastISP                                        ( m_useFastISP );

  // set internal bit-depth and constants
  for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    m_cEncLib.setBitDepth((ChannelType)channelType, m_internalBitDepth[channelType]);
    m_cEncLib.setInputBitDepth((ChannelType)channelType, m_inputBitDepth[channelType]);
#if !JVET_O0525_REMOVE_PCM
    m_cEncLib.setPCMBitDepth((ChannelType)channelType, m_bPCMInputBitDepthFlag ? m_MSBExtendedBitDepth[channelType] : m_internalBitDepth[channelType]);
#endif
  }

#if !JVET_O0525_REMOVE_PCM
  m_cEncLib.setPCMLog2MaxSize                                    ( m_pcmLog2MaxSize);
#endif
  m_cEncLib.setMaxNumMergeCand                                   ( m_maxNumMergeCand );
  m_cEncLib.setMaxNumAffineMergeCand                             ( m_maxNumAffineMergeCand );
  m_cEncLib.setMaxNumTriangleCand                                ( m_maxNumTriangleCand );
#if JVET_O0455_IBC_MAX_MERGE_NUM
  m_cEncLib.setMaxNumIBCMergeCand                                ( m_maxNumIBCMergeCand );
#endif

  //====== Weighted Prediction ========
  m_cEncLib.setUseWP                                             ( m_useWeightedPred     );
  m_cEncLib.setWPBiPred                                          ( m_useWeightedBiPred   );

  //====== Parallel Merge Estimation ========
  m_cEncLib.setLog2ParallelMergeLevelMinus2                      ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cEncLib.setSliceMode                                         ( m_sliceMode );
  m_cEncLib.setSliceArgument                                     ( m_sliceArgument );


  if(m_sliceMode == NO_SLICES )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cEncLib.setLFCrossSliceBoundaryFlag                          ( m_bLFCrossSliceBoundaryFlag );
  m_cEncLib.setUseSAO                                            ( m_bUseSAO );
  m_cEncLib.setTestSAODisableAtPictureLevel                      ( m_bTestSAODisableAtPictureLevel );
  m_cEncLib.setSaoEncodingRate                                   ( m_saoEncodingRate );
  m_cEncLib.setSaoEncodingRateChroma                             ( m_saoEncodingRateChroma );
  m_cEncLib.setMaxNumOffsetsPerPic                               ( m_maxNumOffsetsPerPic);

  m_cEncLib.setSaoCtuBoundary                                    ( m_saoCtuBoundary);
#if !JVET_O0525_REMOVE_PCM
  m_cEncLib.setPCMInputBitDepthFlag                              ( m_bPCMInputBitDepthFlag);
  m_cEncLib.setPCMFilterDisableFlag                              ( m_bPCMFilterDisableFlag);
#endif

  m_cEncLib.setSaoGreedyMergeEnc                                 ( m_saoGreedyMergeEnc);
  m_cEncLib.setIntraSmoothingDisabledFlag                        (!m_enableIntraReferenceSmoothing );
  m_cEncLib.setDecodedPictureHashSEIType                         ( m_decodedPictureHashSEIType );
  m_cEncLib.setRecoveryPointSEIEnabled                           ( m_recoveryPointSEIEnabled );
  m_cEncLib.setBufferingPeriodSEIEnabled                         ( m_bufferingPeriodSEIEnabled );
  m_cEncLib.setPictureTimingSEIEnabled                           ( m_pictureTimingSEIEnabled );
  m_cEncLib.setToneMappingInfoSEIEnabled                         ( m_toneMappingInfoSEIEnabled );
  m_cEncLib.setTMISEIToneMapId                                   ( m_toneMapId );
  m_cEncLib.setTMISEIToneMapCancelFlag                           ( m_toneMapCancelFlag );
  m_cEncLib.setTMISEIToneMapPersistenceFlag                      ( m_toneMapPersistenceFlag );
  m_cEncLib.setTMISEICodedDataBitDepth                           ( m_toneMapCodedDataBitDepth );
  m_cEncLib.setTMISEITargetBitDepth                              ( m_toneMapTargetBitDepth );
  m_cEncLib.setTMISEIModelID                                     ( m_toneMapModelId );
  m_cEncLib.setTMISEIMinValue                                    ( m_toneMapMinValue );
  m_cEncLib.setTMISEIMaxValue                                    ( m_toneMapMaxValue );
  m_cEncLib.setTMISEISigmoidMidpoint                             ( m_sigmoidMidpoint );
  m_cEncLib.setTMISEISigmoidWidth                                ( m_sigmoidWidth );
  m_cEncLib.setTMISEIStartOfCodedInterva                         ( m_startOfCodedInterval );
  m_cEncLib.setTMISEINumPivots                                   ( m_numPivots );
  m_cEncLib.setTMISEICodedPivotValue                             ( m_codedPivotValue );
  m_cEncLib.setTMISEITargetPivotValue                            ( m_targetPivotValue );
  m_cEncLib.setTMISEICameraIsoSpeedIdc                           ( m_cameraIsoSpeedIdc );
  m_cEncLib.setTMISEICameraIsoSpeedValue                         ( m_cameraIsoSpeedValue );
  m_cEncLib.setTMISEIExposureIndexIdc                            ( m_exposureIndexIdc );
  m_cEncLib.setTMISEIExposureIndexValue                          ( m_exposureIndexValue );
  m_cEncLib.setTMISEIExposureCompensationValueSignFlag           ( m_exposureCompensationValueSignFlag );
  m_cEncLib.setTMISEIExposureCompensationValueNumerator          ( m_exposureCompensationValueNumerator );
  m_cEncLib.setTMISEIExposureCompensationValueDenomIdc           ( m_exposureCompensationValueDenomIdc );
  m_cEncLib.setTMISEIRefScreenLuminanceWhite                     ( m_refScreenLuminanceWhite );
  m_cEncLib.setTMISEIExtendedRangeWhiteLevel                     ( m_extendedRangeWhiteLevel );
  m_cEncLib.setTMISEINominalBlackLevelLumaCodeValue              ( m_nominalBlackLevelLumaCodeValue );
  m_cEncLib.setTMISEINominalWhiteLevelLumaCodeValue              ( m_nominalWhiteLevelLumaCodeValue );
  m_cEncLib.setTMISEIExtendedWhiteLevelLumaCodeValue             ( m_extendedWhiteLevelLumaCodeValue );
  m_cEncLib.setChromaResamplingFilterHintEnabled                 ( m_chromaResamplingFilterSEIenabled );
  m_cEncLib.setChromaResamplingHorFilterIdc                      ( m_chromaResamplingHorFilterIdc );
  m_cEncLib.setChromaResamplingVerFilterIdc                      ( m_chromaResamplingVerFilterIdc );
  m_cEncLib.setFramePackingArrangementSEIEnabled                 ( m_framePackingSEIEnabled );
  m_cEncLib.setFramePackingArrangementSEIType                    ( m_framePackingSEIType );
  m_cEncLib.setFramePackingArrangementSEIId                      ( m_framePackingSEIId );
  m_cEncLib.setFramePackingArrangementSEIQuincunx                ( m_framePackingSEIQuincunx );
  m_cEncLib.setFramePackingArrangementSEIInterpretation          ( m_framePackingSEIInterpretation );
  m_cEncLib.setSegmentedRectFramePackingArrangementSEIEnabled    ( m_segmentedRectFramePackingSEIEnabled );
  m_cEncLib.setSegmentedRectFramePackingArrangementSEICancel     ( m_segmentedRectFramePackingSEICancel );
  m_cEncLib.setSegmentedRectFramePackingArrangementSEIType       ( m_segmentedRectFramePackingSEIType );
  m_cEncLib.setSegmentedRectFramePackingArrangementSEIPersistence( m_segmentedRectFramePackingSEIPersistence );
  m_cEncLib.setDisplayOrientationSEIAngle                        ( m_displayOrientationSEIAngle );
  m_cEncLib.setTemporalLevel0IndexSEIEnabled                     ( m_temporalLevel0IndexSEIEnabled );
  m_cEncLib.setGradualDecodingRefreshInfoEnabled                 ( m_gradualDecodingRefreshInfoEnabled );
  m_cEncLib.setNoDisplaySEITLayer                                ( m_noDisplaySEITLayer );
  m_cEncLib.setDecodingUnitInfoSEIEnabled                        ( m_decodingUnitInfoSEIEnabled );
  m_cEncLib.setSOPDescriptionSEIEnabled                          ( m_SOPDescriptionSEIEnabled );
  m_cEncLib.setScalableNestingSEIEnabled                         ( m_scalableNestingSEIEnabled );
  m_cEncLib.setTMCTSSEIEnabled                                   ( m_tmctsSEIEnabled );
  m_cEncLib.setMCTSEncConstraint                                 ( m_MCTSEncConstraint);
  m_cEncLib.setTimeCodeSEIEnabled                                ( m_timeCodeSEIEnabled );
  m_cEncLib.setNumberOfTimeSets                                  ( m_timeCodeSEINumTs );
  for(int i = 0; i < m_timeCodeSEINumTs; i++)
  {
    m_cEncLib.setTimeSet(m_timeSetArray[i], i);
  }
  m_cEncLib.setKneeSEIEnabled                                    ( m_kneeSEIEnabled );
  m_cEncLib.setKneeSEIId                                         ( m_kneeSEIId );
  m_cEncLib.setKneeSEICancelFlag                                 ( m_kneeSEICancelFlag );
  m_cEncLib.setKneeSEIPersistenceFlag                            ( m_kneeSEIPersistenceFlag );
  m_cEncLib.setKneeSEIInputDrange                                ( m_kneeSEIInputDrange );
  m_cEncLib.setKneeSEIInputDispLuminance                         ( m_kneeSEIInputDispLuminance );
  m_cEncLib.setKneeSEIOutputDrange                               ( m_kneeSEIOutputDrange );
  m_cEncLib.setKneeSEIOutputDispLuminance                        ( m_kneeSEIOutputDispLuminance );
  m_cEncLib.setKneeSEINumKneePointsMinus1                        ( m_kneeSEINumKneePointsMinus1 );
  m_cEncLib.setKneeSEIInputKneePoint                             ( m_kneeSEIInputKneePoint );
  m_cEncLib.setKneeSEIOutputKneePoint                            ( m_kneeSEIOutputKneePoint );
  m_cEncLib.setColourRemapInfoSEIFileRoot                        ( m_colourRemapSEIFileRoot );
  m_cEncLib.setMasteringDisplaySEI                               ( m_masteringDisplay );
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  m_cEncLib.setSEIAlternativeTransferCharacteristicsSEIEnable    ( m_preferredTransferCharacteristics>=0     );
  m_cEncLib.setSEIPreferredTransferCharacteristics               ( uint8_t(m_preferredTransferCharacteristics) );
#endif
  m_cEncLib.setSEIGreenMetadataInfoSEIEnable                     ( m_greenMetadataType > 0 );
  m_cEncLib.setSEIGreenMetadataType                              ( uint8_t(m_greenMetadataType) );
  m_cEncLib.setSEIXSDMetricType                                  ( uint8_t(m_xsdMetricType) );

  m_cEncLib.setTileUniformSpacingFlag                            ( m_tileUniformSpacingFlag );
  m_cEncLib.setNumColumnsMinus1                                  ( m_numTileColumnsMinus1 );
  m_cEncLib.setNumRowsMinus1                                     ( m_numTileRowsMinus1 );
  if(!m_tileUniformSpacingFlag)
  {
    m_cEncLib.setColumnWidth                                     ( m_tileColumnWidth );
    m_cEncLib.setRowHeight                                       ( m_tileRowHeight );
  }
  m_cEncLib.setRectSliceFlag                                     ( m_rectSliceFlag );
  m_cEncLib.setNumSlicesInPicMinus1                              ( m_numSlicesInPicMinus1 );
  m_cEncLib.setTopLeftBrickIdx                                   ( m_topLeftBrickIdx );
  m_cEncLib.setBottomRightBrickIdx                               ( m_bottomRightBrickIdx);
  m_cEncLib.setLoopFilterAcrossSlicesEnabledFlag                 ( m_loopFilterAcrossSlicesEnabledFlag );
  m_cEncLib.setSignalledSliceIdFlag                              ( m_signalledSliceIdFlag ),
  m_cEncLib.setSignalledSliceIdLengthMinus1                      ( m_signalledSliceIdLengthMinus1 );
  m_cEncLib.setSliceId                                           ( m_sliceId );
  m_cEncLib.setBrickSplitMap                                     (m_brickSplitMap);

  m_cEncLib.xCheckGSParameters();
  int uiTilesCount = (m_numTileRowsMinus1+1) * (m_numTileColumnsMinus1+1);
  if(uiTilesCount == 1)
  {
    m_bLFCrossTileBoundaryFlag = true;
  }
  m_cEncLib.setLFCrossTileBoundaryFlag                           ( m_bLFCrossTileBoundaryFlag );
  m_cEncLib.setEntropyCodingSyncEnabledFlag                      ( m_entropyCodingSyncEnabledFlag );
  m_cEncLib.setTMVPModeId                                        ( m_TMVPModeId );
  m_cEncLib.setUseScalingListId                                  ( m_useScalingListId  );
  m_cEncLib.setScalingListFileName                               ( m_scalingListFileName );
  m_cEncLib.setDepQuantEnabledFlag                               ( m_depQuantEnabledFlag);
  m_cEncLib.setSignDataHidingEnabledFlag                         ( m_signDataHidingEnabledFlag);
  m_cEncLib.setUseRateCtrl                                       ( m_RCEnableRateControl );
  m_cEncLib.setTargetBitrate                                     ( m_RCTargetBitrate );
  m_cEncLib.setKeepHierBit                                       ( m_RCKeepHierarchicalBit );
  m_cEncLib.setLCULevelRC                                        ( m_RCLCULevelRC );
  m_cEncLib.setUseLCUSeparateModel                               ( m_RCUseLCUSeparateModel );
  m_cEncLib.setInitialQP                                         ( m_RCInitialQP );
  m_cEncLib.setForceIntraQP                                      ( m_RCForceIntraQP );
#if U0132_TARGET_BITS_SATURATION
  m_cEncLib.setCpbSaturationEnabled                              ( m_RCCpbSaturationEnabled );
  m_cEncLib.setCpbSize                                           ( m_RCCpbSize );
  m_cEncLib.setInitialCpbFullness                                ( m_RCInitialCpbFullness );
#endif
  m_cEncLib.setTransquantBypassEnabledFlag                       ( m_TransquantBypassEnabledFlag );
  m_cEncLib.setCUTransquantBypassFlagForceValue                  ( m_CUTransquantBypassFlagForce );
  m_cEncLib.setCostMode                                          ( m_costMode );
  m_cEncLib.setUseRecalculateQPAccordingToLambda                 ( m_recalculateQPAccordingToLambda );
  m_cEncLib.setDecodingParameterSetEnabled                       ( m_decodingParameterSetEnabled );
  m_cEncLib.setActiveParameterSetsSEIEnabled                     ( m_activeParameterSetsSEIEnabled );
  m_cEncLib.setVuiParametersPresentFlag                          ( m_vuiParametersPresentFlag );
  m_cEncLib.setAspectRatioInfoPresentFlag                        ( m_aspectRatioInfoPresentFlag);
  m_cEncLib.setAspectRatioIdc                                    ( m_aspectRatioIdc );
  m_cEncLib.setSarWidth                                          ( m_sarWidth );
  m_cEncLib.setSarHeight                                         ( m_sarHeight );
  m_cEncLib.setColourDescriptionPresentFlag                      ( m_colourDescriptionPresentFlag );
  m_cEncLib.setColourPrimaries                                   ( m_colourPrimaries );
  m_cEncLib.setTransferCharacteristics                           ( m_transferCharacteristics );
  m_cEncLib.setMatrixCoefficients                                ( m_matrixCoefficients );
  m_cEncLib.setChromaLocInfoPresentFlag                          ( m_chromaLocInfoPresentFlag );
  m_cEncLib.setChromaSampleLocTypeTopField                       ( m_chromaSampleLocTypeTopField );
  m_cEncLib.setChromaSampleLocTypeBottomField                    ( m_chromaSampleLocTypeBottomField );
  m_cEncLib.setChromaSampleLocType                               ( m_chromaSampleLocType );
  m_cEncLib.setOverscanInfoPresentFlag                           ( m_overscanInfoPresentFlag );
  m_cEncLib.setOverscanAppropriateFlag                           ( m_overscanAppropriateFlag );
  m_cEncLib.setVideoSignalTypePresentFlag                        ( m_videoSignalTypePresentFlag );
  m_cEncLib.setVideoFullRangeFlag                                ( m_videoFullRangeFlag );
  m_cEncLib.setEfficientFieldIRAPEnabled                         ( m_bEfficientFieldIRAPEnabled );
  m_cEncLib.setHarmonizeGopFirstFieldCoupleEnabled               ( m_bHarmonizeGopFirstFieldCoupleEnabled );
  m_cEncLib.setSummaryOutFilename                                ( m_summaryOutFilename );
  m_cEncLib.setSummaryPicFilenameBase                            ( m_summaryPicFilenameBase );
  m_cEncLib.setSummaryVerboseness                                ( m_summaryVerboseness );
  m_cEncLib.setIMV                                               ( m_ImvMode );
  m_cEncLib.setIMV4PelFast                                       ( m_Imv4PelFast );
  m_cEncLib.setDecodeBitstream                                   ( 0, m_decodeBitstreams[0] );
  m_cEncLib.setDecodeBitstream                                   ( 1, m_decodeBitstreams[1] );
  m_cEncLib.setSwitchPOC                                         ( m_switchPOC );
  m_cEncLib.setSwitchDQP                                         ( m_switchDQP );
  m_cEncLib.setFastForwardToPOC                                  ( m_fastForwardToPOC );
  m_cEncLib.setForceDecodeBitstream1                             ( m_forceDecodeBitstream1 );
  m_cEncLib.setStopAfterFFtoPOC                                  ( m_stopAfterFFtoPOC );
  m_cEncLib.setBs2ModPOCAndType                                  ( m_bs2ModPOCAndType );
  m_cEncLib.setDebugCTU                                          ( m_debugCTU );
#if ENABLE_SPLIT_PARALLELISM
  m_cEncLib.setNumSplitThreads                                   ( m_numSplitThreads );
  m_cEncLib.setForceSingleSplitThread                            ( m_forceSplitSequential );
#endif
#if ENABLE_WPP_PARALLELISM
  m_cEncLib.setNumWppThreads                                     ( m_numWppThreads );
  m_cEncLib.setNumWppExtraLines                                  ( m_numWppExtraLines );
  m_cEncLib.setEnsureWppBitEqual                                 ( m_ensureWppBitEqual );

#endif
  m_cEncLib.setUseALF                                            ( m_alf );
  m_cEncLib.setReshaper                                          ( m_lumaReshapeEnable );
  m_cEncLib.setReshapeSignalType                                 ( m_reshapeSignalType );
  m_cEncLib.setReshapeIntraCMD                                   ( m_intraCMD );
  m_cEncLib.setReshapeCW                                         ( m_reshapeCW );

#if JVET_O0756_CALCULATE_HDRMETRICS
  for (int i=0; i<hdrtoolslib::NB_REF_WHITE; i++)
  {
    m_cEncLib.setWhitePointDeltaE                                (i, m_whitePointDeltaE[i] );
  }
  m_cEncLib.setMaxSampleValue                                    (m_maxSampleValue);
  m_cEncLib.setSampleRange                                       (m_sampleRange);
  m_cEncLib.setColorPrimaries                                    (m_colorPrimaries);
  m_cEncLib.setEnableTFunctionLUT                                (m_enableTFunctionLUT);
  for (int i=0; i<2; i++)
  {
    m_cEncLib.setChromaLocation                                    (i, m_chromaLocation);
    m_cEncLib.setChromaUPFilter                                    (m_chromaUPFilter);
  }
  m_cEncLib.setCropOffsetLeft                                    (m_cropOffsetLeft);
  m_cEncLib.setCropOffsetTop                                     (m_cropOffsetTop);
  m_cEncLib.setCropOffsetRight                                   (m_cropOffsetRight);
  m_cEncLib.setCropOffsetBottom                                  (m_cropOffsetBottom);
  m_cEncLib.setCalculateHdrMetrics                               (m_calculateHdrMetrics);
#endif
}

void EncApp::xCreateLib( std::list<PelUnitBuf*>& recBufList
                        )
{
  // Video I/O
  m_cVideoIOYuvInputFile.open( m_inputFileName,     false, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth );  // read  mode
#if EXTENSION_360_VIDEO
  m_cVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_inputFileWidth, m_inputFileHeight, m_InputChromaFormatIDC);
#else
  m_cVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1], m_InputChromaFormatIDC);
#endif
  if (!m_reconFileName.empty())
  {
    if (m_packedYUVMode && ((m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12)
        || ((m_iSourceWidth & (1 + (m_outputBitDepth[CH_L] & 3))) != 0)))
    {
      EXIT ("Invalid output bit-depth or image width for packed YUV output, aborting\n");
    }
    if (m_packedYUVMode && (m_chromaFormatIDC != CHROMA_400) && ((m_outputBitDepth[CH_C] != 10 && m_outputBitDepth[CH_C] != 12)
        || (((m_iSourceWidth / SPS::getWinUnitX (m_chromaFormatIDC)) & (1 + (m_outputBitDepth[CH_C] & 3))) != 0)))
    {
      EXIT ("Invalid chroma output bit-depth or image width for packed YUV output, aborting\n");
    }

    m_cVideoIOYuvReconFile.open(m_reconFileName, true, m_outputBitDepth, m_outputBitDepth, m_internalBitDepth);  // write mode
  }

  // create the encoder
  m_cEncLib.create();

  // create the output buffer
  for( int i = 0; i < (m_iGOPSize + 1 + (m_isField ? 1 : 0)); i++ )
  {
    recBufList.push_back( new PelUnitBuf );
  }
}

void EncApp::xDestroyLib()
{
  // Video I/O
  m_cVideoIOYuvInputFile.close();
  m_cVideoIOYuvReconFile.close();

  // Neo Decoder
  m_cEncLib.destroy();
}

void EncApp::xInitLib(bool isFieldCoding)
{
  m_cEncLib.init(isFieldCoding, this );
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal variable
 - until the end of input YUV file, call encoding function in EncLib class
 - delete allocated buffers
 - destroy internal class
 .
 */
void EncApp::encode()
{
  m_bitstream.open(m_bitstreamFileName.c_str(), fstream::binary | fstream::out);
  if (!m_bitstream)
  {
    EXIT( "Failed to open bitstream file " << m_bitstreamFileName.c_str() << " for writing\n");
  }

  std::list<PelUnitBuf*> recBufList;
  // initialize internal class & member variables
  xInitLibCfg();
  xCreateLib( recBufList
             );
  xInitLib(m_isField);

  printChromaFormat();

  // main encoder loop
  int   iNumEncoded = 0;
  bool  bEos = false;

  const InputColourSpaceConversion ipCSC  =  m_inputColourSpaceConvert;
  const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

  PelStorage trueOrgPic;
  PelStorage orgPic;
  const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
  UnitArea unitArea( m_chromaFormatIDC, Area( 0, 0, m_iSourceWidth, sourceHeight ) );

  orgPic.create( unitArea );
  trueOrgPic.create( unitArea );
#if EXTENSION_360_VIDEO
  TExt360AppEncTop           ext360(*this, m_cEncLib.getGOPEncoder()->getExt360Data(), *(m_cEncLib.getGOPEncoder()), orgPic);
#endif

  while ( !bEos )
  {
    // read input YUV file
#if EXTENSION_360_VIDEO
    if (ext360.isEnabled())
    {
      ext360.read(m_cVideoIOYuvInputFile, orgPic, trueOrgPic, ipCSC);
    }
    else
    {
      m_cVideoIOYuvInputFile.read(orgPic, trueOrgPic, ipCSC, m_aiPad, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range);
    }
#else
    m_cVideoIOYuvInputFile.read( orgPic, trueOrgPic, ipCSC, m_aiPad, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
#endif

    // increase number of received frames
    m_iFrameRcvd++;

    bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );

    bool flush = 0;
    // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
    if (m_cVideoIOYuvInputFile.isEof())
    {
      flush = true;
      bEos = true;
      m_iFrameRcvd--;
      m_cEncLib.setFramesToBeEncoded(m_iFrameRcvd);
    }

    // call encoding function for one frame
    if ( m_isField )
    {
      m_cEncLib.encode( bEos, flush ? 0 : &orgPic, flush ? 0 : &trueOrgPic, snrCSC, recBufList,
                        iNumEncoded, m_isTopFieldFirst );
#if JVET_O0756_CALCULATE_HDRMETRICS
      m_metricTime = m_cEncLib.getMetricTime();
#endif
    }
    else
    {
      m_cEncLib.encode( bEos, flush ? 0 : &orgPic, flush ? 0 : &trueOrgPic, snrCSC, recBufList,
                        iNumEncoded );
#if JVET_O0756_CALCULATE_HDRMETRICS
      m_metricTime = m_cEncLib.getMetricTime();
#endif
    }

    // write bistream to file if necessary
    if ( iNumEncoded > 0 )
    {
      xWriteOutput( iNumEncoded, recBufList
      );
    }
    // temporally skip frames
    if( m_temporalSubsampleRatio > 1 )
    {
#if EXTENSION_360_VIDEO
      m_cVideoIOYuvInputFile.skipFrames(m_temporalSubsampleRatio - 1, m_inputFileWidth, m_inputFileHeight, m_InputChromaFormatIDC);
#else
      m_cVideoIOYuvInputFile.skipFrames(m_temporalSubsampleRatio-1, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1], m_InputChromaFormatIDC);
#endif
    }
  }

  m_cEncLib.printSummary(m_isField);


  // delete used buffers in encoder class
  m_cEncLib.deletePicBuffer();

  for( auto &p : recBufList )
  {
    delete p;
  }
  recBufList.clear();

  xDestroyLib();

  m_bitstream.close();

  printRateSummary();

  return;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
  Write access units to output file.
  \param bitstreamFile  target bitstream file
  \param iNumEncoded    number of encoded frames
  \param accessUnits    list of access units to be written
 */
void EncApp::xWriteOutput( int iNumEncoded, std::list<PelUnitBuf*>& recBufList
                          )
{
  const InputColourSpaceConversion ipCSC = (!m_outputInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  std::list<PelUnitBuf*>::iterator iterPicYuvRec = recBufList.end();
  int i;

  for ( i = 0; i < iNumEncoded; i++ )
  {
    --iterPicYuvRec;
  }

  if (m_isField)
  {
    //Reinterlace fields
    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      const PelUnitBuf*  pcPicYuvRecTop     = *(iterPicYuvRec++);
      const PelUnitBuf*  pcPicYuvRecBottom  = *(iterPicYuvRec++);

      if (!m_reconFileName.empty())
      {
        m_cVideoIOYuvReconFile.write( *pcPicYuvRecTop, *pcPicYuvRecBottom,
                                      ipCSC,
                                      false, // TODO: m_packedYUVMode,
                                      m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_isTopFieldFirst );
      }
    }
  }
  else
  {
    for ( i = 0; i < iNumEncoded; i++ )
    {
      const PelUnitBuf* pcPicYuvRec = *(iterPicYuvRec++);
      if (!m_reconFileName.empty())
      {
#if JVET_O1164_RPR
        if( m_cEncLib.isRPREnabled() && m_cEncLib.getUpscaledOutput() )
        {
          const SPS& sps = *m_cEncLib.getSPS( 0 );
          const PPS& pps = *m_cEncLib.getPPS( ( sps.getMaxPicWidthInLumaSamples() != pcPicYuvRec->get( COMPONENT_Y ).width || sps.getMaxPicHeightInLumaSamples() != pcPicYuvRec->get( COMPONENT_Y ).height ) ? ENC_PPS_ID_RPR : 0 );

          m_cVideoIOYuvReconFile.writeUpscaledPicture( sps, pps, *pcPicYuvRec, ipCSC, m_packedYUVMode, m_cEncLib.getUpscaledOutput(), NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
        }
        else
        {
          m_cVideoIOYuvReconFile.write( pcPicYuvRec->get( COMPONENT_Y ).width, pcPicYuvRec->get( COMPONENT_Y ).height, *pcPicYuvRec, ipCSC, m_packedYUVMode,
            m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
        }
#else
        m_cVideoIOYuvReconFile.write( *pcPicYuvRec,
                                      ipCSC,
                                      m_packedYUVMode,
                                      m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
#endif
      }
    }
  }
}


void EncApp::outputAU( const AccessUnit& au )
{
  const vector<uint32_t>& stats = writeAnnexB(m_bitstream, au);
  rateStatsAccum(au, stats);
  m_bitstream.flush();
}


/**
 *
 */
void EncApp::rateStatsAccum(const AccessUnit& au, const std::vector<uint32_t>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<uint32_t>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL:
    case NAL_UNIT_CODED_SLICE_STSA:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_RADL:
    case NAL_UNIT_CODED_SLICE_RASL:
    case NAL_UNIT_DPS:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
    case NAL_UNIT_APS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

void EncApp::printRateSummary()
{
  double time = (double) m_iFrameRcvd / m_iFrameRate * m_temporalSubsampleRatio;
  msg( DETAILS,"Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if (m_summaryVerboseness > 0)
  {
    msg(DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
  }
}

void EncApp::printChromaFormat()
{
  if( g_verbosity >= DETAILS )
  {
    std::cout << std::setw(43) << "Input ChromaFormatIDC = ";
    switch (m_InputChromaFormatIDC)
    {
    case CHROMA_400:  std::cout << "  4:0:0"; break;
    case CHROMA_420:  std::cout << "  4:2:0"; break;
    case CHROMA_422:  std::cout << "  4:2:2"; break;
    case CHROMA_444:  std::cout << "  4:4:4"; break;
    default:
      THROW( "invalid chroma fomat");
    }
    std::cout << std::endl;

    std::cout << std::setw(43) << "Output (internal) ChromaFormatIDC = ";
    switch (m_cEncLib.getChromaFormatIdc())
    {
    case CHROMA_400:  std::cout << "  4:0:0"; break;
    case CHROMA_420:  std::cout << "  4:2:0"; break;
    case CHROMA_422:  std::cout << "  4:2:2"; break;
    case CHROMA_444:  std::cout << "  4:4:4"; break;
    default:
      THROW( "invalid chroma fomat");
    }
    std::cout << "\n" << std::endl;
  }
}

//! \}
