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

/** \file     EncAppCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include "EncAppCfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>
#include <limits>

#include "Utilities/program_options_lite.h"
#include "CommonLib/Rom.h"
#include "EncoderLib/RateCtrl.h"

#include "CommonLib/dtrace_next.h"

#define MACRO_TO_STRING_HELPER(val) #val
#define MACRO_TO_STRING(val) MACRO_TO_STRING_HELPER(val)

using namespace std;
namespace po = df::program_options_lite;



enum ExtendedProfileName // this is used for determining profile strings, where multiple profiles map to a single profile idc with various constraint flag combinations
{
  NONE = 0,
  MAIN = 1,
  MAIN10 = 2,
  MAINSTILLPICTURE = 3,
  MAINREXT = 4,
  HIGHTHROUGHPUTREXT = 5, // Placeholder profile for development
  // The following are RExt profiles, which would map to the MAINREXT profile idc.
  // The enumeration indicates the bit-depth constraint in the bottom 2 digits
  //                           the chroma format in the next digit
  //                           the intra constraint in the next digit
  //                           If it is a RExt still picture, there is a '1' for the top digit.
  MONOCHROME_8      = 1008,
  MONOCHROME_12     = 1012,
  MONOCHROME_16     = 1016,
  MAIN_12           = 1112,
  MAIN_422_10       = 1210,
  MAIN_422_12       = 1212,
  MAIN_444          = 1308,
  MAIN_444_10       = 1310,
  MAIN_444_12       = 1312,
  MAIN_444_16       = 1316, // non-standard profile definition, used for development purposes
  MAIN_INTRA        = 2108,
  MAIN_10_INTRA     = 2110,
  MAIN_12_INTRA     = 2112,
  MAIN_422_10_INTRA = 2210,
  MAIN_422_12_INTRA = 2212,
  MAIN_444_INTRA    = 2308,
  MAIN_444_10_INTRA = 2310,
  MAIN_444_12_INTRA = 2312,
  MAIN_444_16_INTRA = 2316,
  MAIN_444_STILL_PICTURE = 11308,
  MAIN_444_16_STILL_PICTURE = 12316,
  NEXT          = 6
};


//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

EncAppCfg::EncAppCfg()
: m_inputColourSpaceConvert(IPCOLOURSPACE_UNCHANGED)
, m_snrInternalColourSpace(false)
, m_outputInternalColourSpace(false)
, m_packedYUVMode(false)
, m_bIntraOnlyConstraintFlag(false)
, m_maxBitDepthConstraintIdc(0)
, m_maxChromaFormatConstraintIdc(CHROMA_420)
, m_bFrameConstraintFlag(false)
, m_bNoQtbttDualTreeIntraConstraintFlag(false)
, m_noPartitionConstraintsOverrideConstraintFlag(false)
, m_bNoSaoConstraintFlag(false)
, m_bNoAlfConstraintFlag(false)
#if !JVET_O0525_REMOVE_PCM
, m_bNoPcmConstraintFlag(false)
#endif
, m_bNoRefWraparoundConstraintFlag(false)
, m_bNoTemporalMvpConstraintFlag(false)
, m_bNoSbtmvpConstraintFlag(false)
, m_bNoAmvrConstraintFlag(false)
, m_bNoBdofConstraintFlag(false)
, m_noDmvrConstraintFlag(false)
, m_bNoCclmConstraintFlag(false)
, m_bNoMtsConstraintFlag(false)
, m_noSbtConstraintFlag(false)
, m_bNoAffineMotionConstraintFlag(false)
, m_bNoGbiConstraintFlag(false)
, m_noIbcConstraintFlag(false)
, m_bNoMhIntraConstraintFlag(false)
, m_noFPelMmvdConstraintFlag(false)
, m_bNoTriangleConstraintFlag(false)
, m_bNoLadfConstraintFlag(false)
, m_noTransformSkipConstraintFlag(false)
#if JVET_O1136_TS_BDPCM_SIGNALLING
, m_noBDPCMConstraintFlag(false)
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
, m_noJointCbCrConstraintFlag(false)
#endif
, m_bNoQpDeltaConstraintFlag(false)
, m_bNoDepQuantConstraintFlag(false)
, m_bNoSignDataHidingConstraintFlag(false)
#if EXTENSION_360_VIDEO
, m_ext360(*this)
#endif
{
  m_aidQP = NULL;
  m_startOfCodedInterval = NULL;
  m_codedPivotValue = NULL;
  m_targetPivotValue = NULL;
}

EncAppCfg::~EncAppCfg()
{
  if ( m_aidQP )
  {
    delete[] m_aidQP;
  }
  if ( m_startOfCodedInterval )
  {
    delete[] m_startOfCodedInterval;
    m_startOfCodedInterval = NULL;
  }
   if ( m_codedPivotValue )
  {
    delete[] m_codedPivotValue;
    m_codedPivotValue = NULL;
  }
  if ( m_targetPivotValue )
  {
    delete[] m_targetPivotValue;
    m_targetPivotValue = NULL;
  }

#if ENABLE_TRACING
  tracing_uninit(g_trace_ctx);
#endif
}

void EncAppCfg::create()
{
}

void EncAppCfg::destroy()
{
}

std::istringstream &operator>>(std::istringstream &in, GOPEntry &entry)     //input
{
  in>>entry.m_sliceType;
  in>>entry.m_POC;
  in>>entry.m_QPOffset;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  in>>entry.m_QPOffsetModelOffset;
  in>>entry.m_QPOffsetModelScale;
#endif
#if W0038_CQP_ADJ
  in>>entry.m_CbQPoffset;
  in>>entry.m_CrQPoffset;
#endif
  in>>entry.m_QPFactor;
  in>>entry.m_tcOffsetDiv2;
  in>>entry.m_betaOffsetDiv2;
  in>>entry.m_temporalId;
  in >> entry.m_numRefPicsActive0;
  in >> entry.m_numRefPics0;
  for (int i = 0; i < entry.m_numRefPics0; i++)
  {
    in >> entry.m_deltaRefPics0[i];
  }
  in >> entry.m_numRefPicsActive1;
  in >> entry.m_numRefPics1;
  for (int i = 0; i < entry.m_numRefPics1; i++)
  {
    in >> entry.m_deltaRefPics1[i];
  }
  return in;
}

std::istringstream &operator>>(std::istringstream &in, BrickSplit &entry)     //input
{
  in>>entry.m_tileIdx;
  in>>entry.m_uniformSplit;
  if (entry.m_uniformSplit)
  {
    in>>entry.m_uniformHeight;
  }
  else
  {
    in>>entry.m_numSplits;
    for ( int i = 0; i < entry.m_numSplits; i++ )
    {
      in>>entry.m_brickHeight[i];
    }
  }
  return in;
}


bool confirmPara(bool bflag, const char* message);

static inline ChromaFormat numberToChromaFormat(const int val)
{
  switch (val)
  {
    case 400: return CHROMA_400; break;
    case 420: return CHROMA_420; break;
    case 422: return CHROMA_422; break;
    case 444: return CHROMA_444; break;
    default:  return NUM_CHROMA_FORMAT;
  }
}

static const struct MapStrToProfile
{
  const char* str;
  Profile::Name value;
}
strToProfile[] =
{
  {"none",                 Profile::NONE               },
  {"main",                 Profile::MAIN               },
  {"main10",               Profile::MAIN10             },
  {"main-still-picture",   Profile::MAINSTILLPICTURE   },
  {"main-RExt",            Profile::MAINREXT           },
  {"high-throughput-RExt", Profile::HIGHTHROUGHPUTREXT },
  {"next",                 Profile::NEXT           }
};

static const struct MapStrToExtendedProfile
{
  const char* str;
  ExtendedProfileName value;
}
strToExtendedProfile[] =
{
    {"none",                      NONE             },
    {"main",                      MAIN             },
    {"main10",                    MAIN10           },
    {"main_still_picture",        MAINSTILLPICTURE },
    {"main-still-picture",        MAINSTILLPICTURE },
    {"main_RExt",                 MAINREXT         },
    {"main-RExt",                 MAINREXT         },
    {"main_rext",                 MAINREXT         },
    {"main-rext",                 MAINREXT         },
    {"high_throughput_RExt",      HIGHTHROUGHPUTREXT },
    {"high-throughput-RExt",      HIGHTHROUGHPUTREXT },
    {"high_throughput_rext",      HIGHTHROUGHPUTREXT },
    {"high-throughput-rext",      HIGHTHROUGHPUTREXT },
    {"monochrome",                MONOCHROME_8     },
    {"monochrome12",              MONOCHROME_12    },
    {"monochrome16",              MONOCHROME_16    },
    {"main12",                    MAIN_12          },
    {"main_422_10",               MAIN_422_10      },
    {"main_422_12",               MAIN_422_12      },
    {"main_444",                  MAIN_444         },
    {"main_444_10",               MAIN_444_10      },
    {"main_444_12",               MAIN_444_12      },
    {"main_444_16",               MAIN_444_16      },
    {"main_intra",                MAIN_INTRA       },
    {"main_10_intra",             MAIN_10_INTRA    },
    {"main_12_intra",             MAIN_12_INTRA    },
    {"main_422_10_intra",         MAIN_422_10_INTRA},
    {"main_422_12_intra",         MAIN_422_12_INTRA},
    {"main_444_intra",            MAIN_444_INTRA   },
    {"main_444_still_picture",    MAIN_444_STILL_PICTURE },
    {"main_444_10_intra",         MAIN_444_10_INTRA},
    {"main_444_12_intra",         MAIN_444_12_INTRA},
    {"main_444_16_intra",         MAIN_444_16_INTRA},
    {"main_444_16_still_picture", MAIN_444_16_STILL_PICTURE },
    {"next",                      NEXT }
};

static const ExtendedProfileName validRExtProfileNames[2/* intraConstraintFlag*/][4/* bit depth constraint 8=0, 10=1, 12=2, 16=3*/][4/*chroma format*/]=
{
    {
        { MONOCHROME_8,  NONE,          NONE,              MAIN_444          }, // 8-bit  inter for 400, 420, 422 and 444
        { NONE,          NONE,          MAIN_422_10,       MAIN_444_10       }, // 10-bit inter for 400, 420, 422 and 444
        { MONOCHROME_12, MAIN_12,       MAIN_422_12,       MAIN_444_12       }, // 12-bit inter for 400, 420, 422 and 444
        { MONOCHROME_16, NONE,          NONE,              MAIN_444_16       }  // 16-bit inter for 400, 420, 422 and 444 (the latter is non standard used for development)
    },
    {
        { NONE,          MAIN_INTRA,    NONE,              MAIN_444_INTRA    }, // 8-bit  intra for 400, 420, 422 and 444
        { NONE,          MAIN_10_INTRA, MAIN_422_10_INTRA, MAIN_444_10_INTRA }, // 10-bit intra for 400, 420, 422 and 444
        { NONE,          MAIN_12_INTRA, MAIN_422_12_INTRA, MAIN_444_12_INTRA }, // 12-bit intra for 400, 420, 422 and 444
        { NONE,          NONE,          NONE,              MAIN_444_16_INTRA }  // 16-bit intra for 400, 420, 422 and 444
    }
};

static const struct MapStrToTier
{
  const char* str;
  Level::Tier value;
}
strToTier[] =
{
  {"main", Level::MAIN},
  {"high", Level::HIGH},
};

static const struct MapStrToLevel
{
  const char* str;
  Level::Name value;
}
strToLevel[] =
{
  {"none",Level::NONE},
  {"1",   Level::LEVEL1},
  {"2",   Level::LEVEL2},
  {"2.1", Level::LEVEL2_1},
  {"3",   Level::LEVEL3},
  {"3.1", Level::LEVEL3_1},
  {"4",   Level::LEVEL4},
  {"4.1", Level::LEVEL4_1},
  {"5",   Level::LEVEL5},
  {"5.1", Level::LEVEL5_1},
  {"5.2", Level::LEVEL5_2},
  {"6",   Level::LEVEL6},
  {"6.1", Level::LEVEL6_1},
  {"6.2", Level::LEVEL6_2},
  {"8.5", Level::LEVEL8_5},
};

#if U0132_TARGET_BITS_SATURATION
uint32_t g_uiMaxCpbSize[2][21] =
{
  //         LEVEL1,        LEVEL2,LEVEL2_1,     LEVEL3, LEVEL3_1,      LEVEL4, LEVEL4_1,       LEVEL5,  LEVEL5_1,  LEVEL5_2,    LEVEL6,  LEVEL6_1,  LEVEL6_2
  { 0, 0, 0, 350000, 0, 0, 1500000, 3000000, 0, 6000000, 10000000, 0, 12000000, 20000000, 0,  25000000,  40000000,  60000000,  60000000, 120000000, 240000000 },
  { 0, 0, 0,      0, 0, 0,       0,       0, 0,       0,        0, 0, 30000000, 50000000, 0, 100000000, 160000000, 240000000, 240000000, 480000000, 800000000 }
};
#endif

static const struct MapStrToCostMode
{
  const char* str;
  CostMode    value;
}
strToCostMode[] =
{
  {"lossy",                     COST_STANDARD_LOSSY},
  {"sequence_level_lossless",   COST_SEQUENCE_LEVEL_LOSSLESS},
  {"lossless",                  COST_LOSSLESS_CODING},
  {"mixed_lossless_lossy",      COST_MIXED_LOSSLESS_LOSSY_CODING}
};

static const struct MapStrToScalingListMode
{
  const char* str;
  ScalingListMode value;
}
strToScalingListMode[] =
{
  {"0",       SCALING_LIST_OFF},
  {"1",       SCALING_LIST_DEFAULT},
  {"2",       SCALING_LIST_FILE_READ},
  {"off",     SCALING_LIST_OFF},
  {"default", SCALING_LIST_DEFAULT},
  {"file",    SCALING_LIST_FILE_READ}
};

template<typename T, typename P>
static std::string enumToString(P map[], uint32_t mapLen, const T val)
{
  for (uint32_t i = 0; i < mapLen; i++)
  {
    if (val == map[i].value)
    {
      return map[i].str;
    }
  }
  return std::string();
}

template<typename T, typename P>
static istream& readStrToEnum(P map[], uint32_t mapLen, istream &in, T &val)
{
  string str;
  in >> str;

  for (uint32_t i = 0; i < mapLen; i++)
  {
    if (str == map[i].str)
    {
      val = map[i].value;
      goto found;
    }
  }
  /* not found */
  in.setstate(ios::failbit);
found:
  return in;
}

//inline to prevent compiler warnings for "unused static function"

static inline istream& operator >> (istream &in, ExtendedProfileName &profile)
{
  return readStrToEnum(strToExtendedProfile, sizeof(strToExtendedProfile)/sizeof(*strToExtendedProfile), in, profile);
}

namespace Level
{
  static inline istream& operator >> (istream &in, Tier &tier)
  {
    return readStrToEnum(strToTier, sizeof(strToTier)/sizeof(*strToTier), in, tier);
  }

  static inline istream& operator >> (istream &in, Name &level)
  {
    return readStrToEnum(strToLevel, sizeof(strToLevel)/sizeof(*strToLevel), in, level);
  }
}

static inline istream& operator >> (istream &in, CostMode &mode)
{
  return readStrToEnum(strToCostMode, sizeof(strToCostMode)/sizeof(*strToCostMode), in, mode);
}

static inline istream& operator >> (istream &in, ScalingListMode &mode)
{
  return readStrToEnum(strToScalingListMode, sizeof(strToScalingListMode)/sizeof(*strToScalingListMode), in, mode);
}

template <class T>
struct SMultiValueInput
{
  const T              minValIncl;
  const T              maxValIncl;
  const std::size_t    minNumValuesIncl;
  const std::size_t    maxNumValuesIncl; // Use 0 for unlimited
        std::vector<T> values;
  SMultiValueInput() : minValIncl(0), maxValIncl(0), minNumValuesIncl(0), maxNumValuesIncl(0), values() { }
  SMultiValueInput(std::vector<T> &defaults) : minValIncl(0), maxValIncl(0), minNumValuesIncl(0), maxNumValuesIncl(0), values(defaults) { }
  SMultiValueInput(const T &minValue, const T &maxValue, std::size_t minNumberValues=0, std::size_t maxNumberValues=0)
    : minValIncl(minValue), maxValIncl(maxValue), minNumValuesIncl(minNumberValues), maxNumValuesIncl(maxNumberValues), values()  { }
  SMultiValueInput(const T &minValue, const T &maxValue, std::size_t minNumberValues, std::size_t maxNumberValues, const T* defValues, const uint32_t numDefValues)
    : minValIncl(minValue), maxValIncl(maxValue), minNumValuesIncl(minNumberValues), maxNumValuesIncl(maxNumberValues), values(defValues, defValues+numDefValues)  { }
  SMultiValueInput<T> &operator=(const std::vector<T> &userValues) { values=userValues; return *this; }
  SMultiValueInput<T> &operator=(const SMultiValueInput<T> &userValues) { values=userValues.values; return *this; }

  T readValue(const char *&pStr, bool &bSuccess);

  istream& readValues(std::istream &in);
};

template <class T>
static inline istream& operator >> (std::istream &in, SMultiValueInput<T> &values)
{
  return values.readValues(in);
}

template<>
uint32_t SMultiValueInput<uint32_t>::readValue(const char *&pStr, bool &bSuccess)
{
  char *eptr;
  uint32_t val=strtoul(pStr, &eptr, 0);
  pStr=eptr;
  bSuccess=!(*eptr!=0 && !isspace(*eptr) && *eptr!=',') && !(val<minValIncl || val>maxValIncl);
  return val;
}

template<>
int SMultiValueInput<int>::readValue(const char *&pStr, bool &bSuccess)
{
  char *eptr;
  int val=strtol(pStr, &eptr, 0);
  pStr=eptr;
  bSuccess=!(*eptr!=0 && !isspace(*eptr) && *eptr!=',') && !(val<minValIncl || val>maxValIncl);
  return val;
}

template<>
double SMultiValueInput<double>::readValue(const char *&pStr, bool &bSuccess)
{
  char *eptr;
  double val=strtod(pStr, &eptr);
  pStr=eptr;
  bSuccess=!(*eptr!=0 && !isspace(*eptr) && *eptr!=',') && !(val<minValIncl || val>maxValIncl);
  return val;
}

template<>
bool SMultiValueInput<bool>::readValue(const char *&pStr, bool &bSuccess)
{
  char *eptr;
  int val=strtol(pStr, &eptr, 0);
  pStr=eptr;
  bSuccess=!(*eptr!=0 && !isspace(*eptr) && *eptr!=',') && !(val<int(minValIncl) || val>int(maxValIncl));
  return val!=0;
}

template <class T>
istream& SMultiValueInput<T>::readValues(std::istream &in)
{
  values.clear();
  string str;
  while (!in.eof())
  {
    string tmp; in >> tmp; str+=" " + tmp;
  }
  if (!str.empty())
  {
    const char *pStr=str.c_str();
    // soak up any whitespace
    for(;isspace(*pStr);pStr++);

    while (*pStr != 0)
    {
      bool bSuccess=true;
      T val=readValue(pStr, bSuccess);
      if (!bSuccess)
      {
        in.setstate(ios::failbit);
        break;
      }

      if (maxNumValuesIncl != 0 && values.size() >= maxNumValuesIncl)
      {
        in.setstate(ios::failbit);
        break;
      }
      values.push_back(val);
      // soak up any whitespace and up to 1 comma.
      for(;isspace(*pStr);pStr++);
      if (*pStr == ',')
      {
        pStr++;
      }
      for(;isspace(*pStr);pStr++);
    }
  }
  if (values.size() < minNumValuesIncl)
  {
    in.setstate(ios::failbit);
  }
  return in;
}

#if QP_SWITCHING_FOR_PARALLEL
template <class T>
static inline istream& operator >> (std::istream &in, EncAppCfg::OptionalValue<T> &value)
{
  in >> std::ws;
  if (in.eof())
  {
    value.bPresent = false;
  }
  else
  {
    in >> value.value;
    value.bPresent = true;
  }
  return in;
}
#endif

static void
automaticallySelectRExtProfile(const bool bUsingGeneralRExtTools,
                               const bool bUsingChromaQPAdjustment,
                               const bool bUsingExtendedPrecision,
                               const bool bIntraConstraintFlag,
                               uint32_t &bitDepthConstraint,
                               ChromaFormat &chromaFormatConstraint,
                               const int  maxBitDepth,
                               const ChromaFormat chromaFormat)
{
  // Try to choose profile, according to table in Q1013.
  uint32_t trialBitDepthConstraint=maxBitDepth;
  if (trialBitDepthConstraint<8)
  {
    trialBitDepthConstraint=8;
  }
  else if (trialBitDepthConstraint==9 || trialBitDepthConstraint==11)
  {
    trialBitDepthConstraint++;
  }
  else if (trialBitDepthConstraint>12)
  {
    trialBitDepthConstraint=16;
  }

  // both format and bit depth constraints are unspecified
  if (bUsingExtendedPrecision || trialBitDepthConstraint==16)
  {
    bitDepthConstraint = 16;
    chromaFormatConstraint = (!bIntraConstraintFlag && chromaFormat==CHROMA_400) ? CHROMA_400 : CHROMA_444;
  }
  else if (bUsingGeneralRExtTools)
  {
    if (chromaFormat == CHROMA_400 && !bIntraConstraintFlag)
    {
      bitDepthConstraint = 16;
      chromaFormatConstraint = CHROMA_400;
    }
    else
    {
      bitDepthConstraint = trialBitDepthConstraint;
      chromaFormatConstraint = CHROMA_444;
    }
  }
  else if (chromaFormat == CHROMA_400)
  {
    if (bIntraConstraintFlag)
    {
      chromaFormatConstraint = CHROMA_420; // there is no intra 4:0:0 profile.
      bitDepthConstraint     = trialBitDepthConstraint;
    }
    else
    {
      chromaFormatConstraint = CHROMA_400;
      bitDepthConstraint     = trialBitDepthConstraint == 8 ? 8 : 12;
    }
  }
  else
  {
    bitDepthConstraint = trialBitDepthConstraint;
    chromaFormatConstraint = chromaFormat;
    if (bUsingChromaQPAdjustment && chromaFormat == CHROMA_420)
    {
      chromaFormatConstraint = CHROMA_422; // 4:2:0 cannot use the chroma qp tool.
    }
    if (chromaFormatConstraint == CHROMA_422 && bitDepthConstraint == 8)
    {
      bitDepthConstraint = 10; // there is no 8-bit 4:2:2 profile.
    }
    if (chromaFormatConstraint == CHROMA_420 && !bIntraConstraintFlag)
    {
      bitDepthConstraint = 12; // there is no 8 or 10-bit 4:2:0 inter RExt profile.
    }
  }
}
// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
bool EncAppCfg::parseCfg( int argc, char* argv[] )
{
  bool do_help = false;

  int tmpChromaFormat;
  int tmpInputChromaFormat;
  int tmpConstraintChromaFormat;
  int tmpWeightedPredictionMethod;
  int tmpFastInterSearchMode;
  int tmpMotionEstimationSearchMethod;
  int tmpSliceMode;
  int tmpDecodedPictureHashSEIMappedType;
  string inputColourSpaceConvert;
  string inputPathPrefix;
  ExtendedProfileName extendedProfile;
  int saoOffsetBitShift[MAX_NUM_CHANNEL_TYPE];

  // Multi-value input fields:                                // minval, maxval (incl), min_entries, max_entries (incl) [, default values, number of default values]
  SMultiValueInput<uint32_t> cfg_ColumnWidth                     (0, std::numeric_limits<uint32_t>::max(), 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_RowHeight                       (0, std::numeric_limits<uint32_t>::max(), 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<int>  cfg_startOfCodedInterval            (std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, 1<<16);
  SMultiValueInput<int>  cfg_codedPivotValue                 (std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, 1<<16);
  SMultiValueInput<int>  cfg_targetPivotValue                (std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, 1<<16);

  SMultiValueInput<uint32_t> cfg_SliceIdx                    (0, std::numeric_limits<uint32_t>::max(), 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_SignalledSliceId            (0, std::numeric_limits<uint32_t>::max(), 0, std::numeric_limits<uint32_t>::max());

  SMultiValueInput<double> cfg_adIntraLambdaModifier         (0, std::numeric_limits<double>::max(), 0, MAX_TLAYER); ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.

#if SHARP_LUMA_DELTA_QP
  const int defaultLumaLevelTodQp_QpChangePoints[]   =  {-3,  -2,  -1,   0,   1,   2,   3,   4,   5,   6};
  const int defaultLumaLevelTodQp_LumaChangePoints[] =  { 0, 301, 367, 434, 501, 567, 634, 701, 767, 834};
  SMultiValueInput<int>  cfg_lumaLeveltoDQPMappingQP         (-MAX_QP, MAX_QP,                    0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE, defaultLumaLevelTodQp_QpChangePoints,   sizeof(defaultLumaLevelTodQp_QpChangePoints  )/sizeof(int));
  SMultiValueInput<int>  cfg_lumaLeveltoDQPMappingLuma       (0, std::numeric_limits<int>::max(), 0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE, defaultLumaLevelTodQp_LumaChangePoints, sizeof(defaultLumaLevelTodQp_LumaChangePoints)/sizeof(int));
  uint32_t lumaLevelToDeltaQPMode;
#endif
#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
  const int qpInVals[] = { 25, 33, 43 };                // qpInVal values used to derive the chroma QP mapping table used in VTM-5.0
  const int qpOutVals[] = { 25, 32, 37 };               // qpOutVal values used to derive the chroma QP mapping table used in VTM-5.0
  SMultiValueInput<int> cfg_qpInValCb                   (MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, qpInVals, sizeof(qpInVals)/sizeof(int));
  SMultiValueInput<int> cfg_qpOutValCb                  (MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, qpOutVals, sizeof(qpOutVals) / sizeof(int));
  const int zeroVector[] = { 0 };
  SMultiValueInput<int> cfg_qpInValCr                   (MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
  SMultiValueInput<int> cfg_qpOutValCr                  (MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
  SMultiValueInput<int> cfg_qpInValCbCr                 (MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
  SMultiValueInput<int> cfg_qpOutValCbCr                (MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
#endif
  const uint32_t defaultInputKneeCodes[3]  = { 600, 800, 900 };
  const uint32_t defaultOutputKneeCodes[3] = { 100, 250, 450 };
  SMultiValueInput<uint32_t> cfg_kneeSEIInputKneePointValue      (1,  999, 0, 999, defaultInputKneeCodes,  sizeof(defaultInputKneeCodes )/sizeof(uint32_t));
  SMultiValueInput<uint32_t> cfg_kneeSEIOutputKneePointValue     (0, 1000, 0, 999, defaultOutputKneeCodes, sizeof(defaultOutputKneeCodes)/sizeof(uint32_t));
  const int defaultPrimaryCodes[6]     = { 0,50000, 0,0, 50000,0 };
  const int defaultWhitePointCode[2]   = { 16667, 16667 };
  SMultiValueInput<int>  cfg_DisplayPrimariesCode            (0, 50000, 6, 6, defaultPrimaryCodes,   sizeof(defaultPrimaryCodes  )/sizeof(int));
  SMultiValueInput<int>  cfg_DisplayWhitePointCode           (0, 50000, 2, 2, defaultWhitePointCode, sizeof(defaultWhitePointCode)/sizeof(int));

  SMultiValueInput<bool> cfg_timeCodeSeiTimeStampFlag        (0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<bool> cfg_timeCodeSeiNumUnitFieldBasedFlag(0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<int>  cfg_timeCodeSeiCountingType         (0,  6, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<bool> cfg_timeCodeSeiFullTimeStampFlag    (0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<bool> cfg_timeCodeSeiDiscontinuityFlag    (0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<bool> cfg_timeCodeSeiCntDroppedFlag       (0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<int>  cfg_timeCodeSeiNumberOfFrames       (0,511, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<int>  cfg_timeCodeSeiSecondsValue         (0, 59, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<int>  cfg_timeCodeSeiMinutesValue         (0, 59, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<int>  cfg_timeCodeSeiHoursValue           (0, 23, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<bool> cfg_timeCodeSeiSecondsFlag          (0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<bool> cfg_timeCodeSeiMinutesFlag          (0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<bool> cfg_timeCodeSeiHoursFlag            (0,  1, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<int>  cfg_timeCodeSeiTimeOffsetLength     (0, 31, 0, MAX_TIMECODE_SEI_SETS);
  SMultiValueInput<int>  cfg_timeCodeSeiTimeOffsetValue      (std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, MAX_TIMECODE_SEI_SETS);
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  const int defaultLadfQpOffset[3] = { 1, 0, 1 };
  const int defaultLadfIntervalLowerBound[2] = { 350, 833 };
  SMultiValueInput<int>  cfg_LadfQpOffset                    ( -MAX_QP, MAX_QP, 2, MAX_LADF_INTERVALS, defaultLadfQpOffset, 3 );
  SMultiValueInput<int>  cfg_LadfIntervalLowerBound          ( 0, std::numeric_limits<int>::max(), 1, MAX_LADF_INTERVALS - 1, defaultLadfIntervalLowerBound, 2 );
#endif
  SMultiValueInput<unsigned> cfg_virtualBoundariesPosX       (0, std::numeric_limits<uint32_t>::max(), 0, 3);
  SMultiValueInput<unsigned> cfg_virtualBoundariesPosY       (0, std::numeric_limits<uint32_t>::max(), 0, 3);
  int warnUnknowParameter = 0;

#if ENABLE_TRACING
  string sTracingRule;
  string sTracingFile;
  bool   bTracingChannelsList = false;
#endif
#if ENABLE_SIMD_OPT
  std::string ignore;
#endif

  bool sdr = false;

  po::Options opts;
  opts.addOptions()
  ("help",                                            do_help,                                          false, "this help text")
  ("c",    po::parseConfigFile, "configuration file name")
  ("WarnUnknowParameter,w",                           warnUnknowParameter,                                  0, "warn for unknown configuration parameters instead of failing")
  ("isSDR",                                           sdr,                                              false, "compatibility")
#if ENABLE_SIMD_OPT
  ("SIMD",                                            ignore,                                      string(""), "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension\n")
#endif
  // File, I/O and source parameters
  ("InputFile,i",                                     m_inputFileName,                             string(""), "Original YUV input file name")
  ("InputPathPrefix,-ipp",                            inputPathPrefix,                             string(""), "pathname to prepend to input filename")
  ("BitstreamFile,b",                                 m_bitstreamFileName,                         string(""), "Bitstream output file name")
  ("ReconFile,o",                                     m_reconFileName,                             string(""), "Reconstructed YUV output file name")
  ("SourceWidth,-wdt",                                m_iSourceWidth,                                       0, "Source picture width")
  ("SourceHeight,-hgt",                               m_iSourceHeight,                                      0, "Source picture height")
  ("InputBitDepth",                                   m_inputBitDepth[CHANNEL_TYPE_LUMA],                   8, "Bit-depth of input file")
  ("OutputBitDepth",                                  m_outputBitDepth[CHANNEL_TYPE_LUMA],                  0, "Bit-depth of output file (default:InternalBitDepth)")
  ("MSBExtendedBitDepth",                             m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA],             0, "bit depth of luma component after addition of MSBs of value 0 (used for synthesising High Dynamic Range source material). (default:InputBitDepth)")
  ("InternalBitDepth",                                m_internalBitDepth[CHANNEL_TYPE_LUMA],                0, "Bit-depth the codec operates at. (default: MSBExtendedBitDepth). If different to MSBExtendedBitDepth, source data will be converted")
  ("InputBitDepthC",                                  m_inputBitDepth[CHANNEL_TYPE_CHROMA],                 0, "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("OutputBitDepthC",                                 m_outputBitDepth[CHANNEL_TYPE_CHROMA],                0, "As per OutputBitDepth but for chroma component. (default: use luma output bit-depth)")
  ("MSBExtendedBitDepthC",                            m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA],           0, "As per MSBExtendedBitDepth but for chroma component. (default:MSBExtendedBitDepth)")
  ("InternalBitDepthC",                               m_internalBitDepth[CHANNEL_TYPE_CHROMA],              0, "As per InternalBitDepth but for chroma component. (default:InternalBitDepth)")
  ("ExtendedPrecision",                               m_extendedPrecisionProcessingFlag,                false, "Increased internal accuracies to support high bit depths (not valid in V1 profiles)")
  ("HighPrecisionPredictionWeighting",                m_highPrecisionOffsetsEnabledFlag,                false, "Use high precision option for weighted prediction (not valid in V1 profiles)")
  ("InputColourSpaceConvert",                         inputColourSpaceConvert,                     string(""), "Colour space conversion to apply to input video. Permitted values are (empty string=UNCHANGED) " + getListOfColourSpaceConverts(true))
  ("SNRInternalColourSpace",                          m_snrInternalColourSpace,                         false, "If true, then no colour space conversion is applied prior to SNR, otherwise inverse of input is applied.")
  ("OutputInternalColourSpace",                       m_outputInternalColourSpace,                      false, "If true, then no colour space conversion is applied for reconstructed video, otherwise inverse of input is applied.")
  ("InputChromaFormat",                               tmpInputChromaFormat,                               420, "InputChromaFormatIDC")
  ("MSEBasedSequencePSNR",                            m_printMSEBasedSequencePSNR,                      false, "0 (default) emit sequence PSNR only as a linear average of the frame PSNRs, 1 = also emit a sequence PSNR based on an average of the frame MSEs")
  ("PrintHexPSNR",                                    m_printHexPsnr,                                   false, "0 (default) don't emit hexadecimal PSNR for each frame, 1 = also emit hexadecimal PSNR values")
  ("PrintFrameMSE",                                   m_printFrameMSE,                                  false, "0 (default) emit only bit count and PSNRs for each frame, 1 = also emit MSE values")
  ("PrintSequenceMSE",                                m_printSequenceMSE,                               false, "0 (default) emit only bit rate and PSNRs for the whole sequence, 1 = also emit MSE values")
  ("CabacZeroWordPaddingEnabled",                     m_cabacZeroWordPaddingEnabled,                     true, "0 do not add conforming cabac-zero-words to bit streams, 1 (default) = add cabac-zero-words as required")
  ("ChromaFormatIDC,-cf",                             tmpChromaFormat,                                      0, "ChromaFormatIDC (400|420|422|444 or set 0 (default) for same as InputChromaFormat)")
  ("ConformanceMode",                                 m_conformanceWindowMode,                              0, "Deprecated alias of ConformanceWindowMode")
  ("ConformanceWindowMode",                           m_conformanceWindowMode,                              0, "Window conformance mode (0: no window, 1:automatic padding, 2:padding, 3:conformance")
  ("HorizontalPadding,-pdx",                          m_aiPad[0],                                           0, "Horizontal source padding for conformance window mode 2")
  ("VerticalPadding,-pdy",                            m_aiPad[1],                                           0, "Vertical source padding for conformance window mode 2")
  ("ConfLeft",                                        m_confWinLeft,                                        0, "Deprecated alias of ConfWinLeft")
  ("ConfRight",                                       m_confWinRight,                                       0, "Deprecated alias of ConfWinRight")
  ("ConfTop",                                         m_confWinTop,                                         0, "Deprecated alias of ConfWinTop")
  ("ConfBottom",                                      m_confWinBottom,                                      0, "Deprecated alias of ConfWinBottom")
  ("ConfWinLeft",                                     m_confWinLeft,                                        0, "Left offset for window conformance mode 3")
  ("ConfWinRight",                                    m_confWinRight,                                       0, "Right offset for window conformance mode 3")
  ("ConfWinTop",                                      m_confWinTop,                                         0, "Top offset for window conformance mode 3")
  ("ConfWinBottom",                                   m_confWinBottom,                                      0, "Bottom offset for window conformance mode 3")
#if JVET_O0610_CFG
  ("AccessUnitDelimiter",                             m_AccessUnitDelimiter,                             true, "Enable Access Unit Delimiter NALUs")
#else
  ("AccessUnitDelimiter",                             m_AccessUnitDelimiter,                            false, "Enable Access Unit Delimiter NALUs")
#endif
  ("FrameRate,-fr",                                   m_iFrameRate,                                         0, "Frame rate")
  ("FrameSkip,-fs",                                   m_FrameSkip,                                         0u, "Number of frames to skip at start of input YUV")
  ("TemporalSubsampleRatio,-ts",                      m_temporalSubsampleRatio,                            1u, "Temporal sub-sample ratio when reading input YUV")
  ("FramesToBeEncoded,f",                             m_framesToBeEncoded,                                  0, "Number of frames to be encoded (default=all)")
  ("ClipInputVideoToRec709Range",                     m_bClipInputVideoToRec709Range,                   false, "If true then clip input video to the Rec. 709 Range on loading when InternalBitDepth is less than MSBExtendedBitDepth")
  ("ClipOutputVideoToRec709Range",                    m_bClipOutputVideoToRec709Range,                  false, "If true then clip output video to the Rec. 709 Range on saving when OutputBitDepth is less than InternalBitDepth")
  ("PYUV",                                            m_packedYUVMode,                                  false, "If true then output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")
  ("SummaryOutFilename",                              m_summaryOutFilename,                          string(), "Filename to use for producing summary output file. If empty, do not produce a file.")
  ("SummaryPicFilenameBase",                          m_summaryPicFilenameBase,                      string(), "Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended. If empty, do not produce a file.")
  ("SummaryVerboseness",                              m_summaryVerboseness,                                0u, "Specifies the level of the verboseness of the text output")
  ("Verbosity,v",                                     m_verbosity,                               (int)VERBOSE, "Specifies the level of the verboseness")

#if JVET_O0756_CONFIG_HDRMETRICS || JVET_O0756_CALCULATE_HDRMETRICS
  ( "WhitePointDeltaE1",                              m_whitePointDeltaE[0],                            100.0, "1st reference white point value")
  ( "WhitePointDeltaE2",                              m_whitePointDeltaE[1],                           1000.0, "2nd reference white point value")
  ( "WhitePointDeltaE3",                              m_whitePointDeltaE[2],                           5000.0, "3rd reference white point value")
  ( "MaxSampleValue",                                 m_maxSampleValue,                               10000.0, "Maximum sample value for floats")
  ( "InputSampleRange",                               m_sampleRange,                                        0, "Sample Range")
  ( "InputColorPrimaries",                            m_colorPrimaries,                                     1, "Input Color Primaries")
  ( "EnableTFunctionLUT",                             m_enableTFunctionLUT,                             false, "Input Color Primaries")
  ( "ChromaLocation",                                 m_chromaLocation,                                     2, "Location of Chroma Samples")
  ( "ChromaUpsampleFilter",                           m_chromaUPFilter,                                     1, "420 to 444 conversion filters")
  ( "CropOffsetLeft",                                 m_cropOffsetLeft,                                     0, "Crop Offset Left position")
  ( "CropOffsetTop",                                  m_cropOffsetTop,                                      0, "Crop Offset Top position")
  ( "CropOffsetRight",                                m_cropOffsetRight,                                    0, "Crop Offset Right position")
  ( "CropOffsetBottom",                               m_cropOffsetBottom,                                   0, "Crop Offset Bottom position")
  ( "CalculateHdrMetrics",                            m_calculateHdrMetrics,                            false, "Enable HDR metric calculation")
#endif

  //Field coding parameters
  ("FieldCoding",                                     m_isField,                                        false, "Signals if it's a field based coding")
  ("TopFieldFirst, Tff",                              m_isTopFieldFirst,                                false, "In case of field based coding, signals whether if it's a top field first or not")
  ("EfficientFieldIRAPEnabled",                       m_bEfficientFieldIRAPEnabled,                      true, "Enable to code fields in a specific, potentially more efficient, order.")
  ("HarmonizeGopFirstFieldCoupleEnabled",             m_bHarmonizeGopFirstFieldCoupleEnabled,            true, "Enables harmonization of Gop first field couple")

  // Profile and level
  ("Profile",                                         extendedProfile,                                   NONE, "Profile name to use for encoding. Use main (for main), main10 (for main10), main-still-picture, main-RExt (for Range Extensions profile), any of the RExt specific profile names, or none")
  ("Level",                                           m_level,                                    Level::NONE, "Level limit to be used, eg 5.1, or none")
  ("Tier",                                            m_levelTier,                                Level::MAIN, "Tier to use for interpretation of --Level (main or high only)")
  ("SubProfile",                                      m_subProfile,                                        0u, "Sub-profile idc")
  ("EnableDecodingParameterSet",                      m_decodingParameterSetEnabled,                    false, "Enables writing of Decoding Parameter Set")
  ("MaxBitDepthConstraint",                           m_bitDepthConstraint,                                0u, "Bit depth to use for profile-constraint for RExt profiles. 0=automatically choose based upon other parameters")
  ("MaxChromaFormatConstraint",                       tmpConstraintChromaFormat,                            0, "Chroma-format to use for the profile-constraint for RExt profiles. 0=automatically choose based upon other parameters")
  ("IntraConstraintFlag",                             m_intraConstraintFlag,                            false, "Value of general_intra_constraint_flag to use for RExt profiles (not used if an explicit RExt sub-profile is specified)")
  ("OnePictureOnlyConstraintFlag",                    m_onePictureOnlyConstraintFlag,                   false, "Value of general_one_picture_only_constraint_flag to use for RExt profiles (not used if an explicit RExt sub-profile is specified)")
  ("LowerBitRateConstraintFlag",                      m_lowerBitRateConstraintFlag,                      true, "Value of general_lower_bit_rate_constraint_flag to use for RExt profiles")

  ("ProgressiveSource",                               m_progressiveSourceFlag,                          false, "Indicate that source is progressive")
  ("InterlacedSource",                                m_interlacedSourceFlag,                           false, "Indicate that source is interlaced")
  ("NonPackedSource",                                 m_nonPackedConstraintFlag,                        false, "Indicate that source does not contain frame packing")
  ("FrameOnly",                                       m_frameOnlyConstraintFlag,                        false, "Indicate that the bitstream contains only frames")
  ("CTUSize",                                         m_uiCTUSize,                                       128u, "CTUSize (specifies the CTU size if QTBT is on) [default: 128]")
  ("EnablePartitionConstraintsOverride",              m_SplitConsOverrideEnabledFlag,                    true, "Enable partition constraints override")
  ("MinQTISlice",                                     m_uiMinQT[0],                                        8u, "MinQTISlice")
  ("MinQTLumaISlice",                                 m_uiMinQT[0],                                        8u, "MinQTLumaISlice")
  ("MinQTChromaISlice",                               m_uiMinQT[2],                                        4u, "MinQTChromaISlice")
  ("MinQTNonISlice",                                  m_uiMinQT[1],                                        8u, "MinQTNonISlice")
  ("MaxBTDepth",                                      m_uiMaxBTDepth,                                      3u, "MaxBTDepth")
  ("MaxBTDepthI",                                     m_uiMaxBTDepthI,                                     3u, "MaxBTDepthI")
  ("MaxBTDepthISliceL",                               m_uiMaxBTDepthI,                                     3u, "MaxBTDepthISliceL")
  ("MaxBTDepthISliceC",                               m_uiMaxBTDepthIChroma,                               3u, "MaxBTDepthISliceC")
  ("DualITree",                                       m_dualTree,                                       false, "Use separate QTBT trees for intra slice luma and chroma channel types")
  ( "LFNST",                                          m_LFNST,                                          false, "Enable LFNST (0:off, 1:on)  [default: off]" )
  ( "FastLFNST",                                      m_useFastLFNST,                                   false, "Fast methods for LFNST" )
  ("SubPuMvp",                                       m_SubPuMvpMode,                                       0, "Enable Sub-PU temporal motion vector prediction (0:off, 1:ATMVP, 2:STMVP, 3:ATMVP+STMVP)  [default: off]")
  ("MMVD",                                           m_MMVD,                                            true, "Enable Merge mode with Motion Vector Difference (0:off, 1:on)  [default: 1]")
  ("Affine",                                         m_Affine,                                         false, "Enable affine prediction (0:off, 1:on)  [default: off]")
  ("AffineType",                                     m_AffineType,                                     true,  "Enable affine type prediction (0:off, 1:on)  [default: on]" )
#if JVET_O0070_PROF
  ("PROF",                                           m_PROF,                                           false, "Enable Prediction refinement with optical flow for affine mode (0:off, 1:on)  [default: off]")
#endif
  ("BIO",                                            m_BIO,                                             false, "Enable bi-directional optical flow")
  ("IMV",                                             m_ImvMode,                                            1, "Adaptive MV precision Mode (IMV)\n"
                                                                                                               "\t0: disabled\n"
                                                                                                               "\t1: enabled (1/2-Pel, Full-Pel and 4-PEL)\n")
  ("IMV4PelFast",                                     m_Imv4PelFast,                                        1, "Fast 4-Pel Adaptive MV precision Mode 0:disabled, 1:enabled)  [default: 1]")
  ("LMChroma",                                        m_LMChroma,                                           1, " LMChroma prediction "
                                                                                                               "\t0:  Disable LMChroma\n"
                                                                                                               "\t1:  Enable LMChroma\n")
  ("CclmCollocatedChroma",                            m_cclmCollocatedChromaFlag,                       false, "Specifies the location of the top-left downsampled luma sample in cross-component linear model intra prediction relative to the top-left luma sample\n"
                                                                                                               "\t0:  horizontally co-sited, vertically shifted by 0.5 units of luma samples\n"
                                                                                                               "\t1:  collocated\n")
  ("MTS",                                             m_MTS,                                                0, "Multiple Transform Set (MTS)\n"
    "\t0:  Disable MTS\n"
    "\t1:  Enable only Intra MTS\n"
    "\t2:  Enable only Inter MTS\n"
    "\t3:  Enable both Intra & Inter MTS\n")
  ("MTSIntraMaxCand",                                 m_MTSIntraMaxCand,                                    3, "Number of additional candidates to test in encoder search for MTS in intra slices\n")
  ("MTSInterMaxCand",                                 m_MTSInterMaxCand,                                    4, "Number of additional candidates to test in encoder search for MTS in inter slices\n")
  ("MTSImplicit",                                     m_MTSImplicit,                                        0, "Enable implicit MTS (when explicit MTS is off)\n")
  ( "SBT",                                            m_SBT,                                            false, "Enable Sub-Block Transform for inter blocks\n" )
  ( "ISP",                                            m_ISP,                                            false, "Enable Intra Sub-Partitions\n" )
  ("SMVD",                                            m_SMVD,                                           false, "Enable Symmetric MVD\n")
  ("CompositeLTReference",                            m_compositeRefEnabled,                            false, "Enable Composite Long Term Reference Frame")
  ("GBi",                                             m_GBi,                                            false, "Enable Generalized Bi-prediction(GBi)")
  ("GBiFast",                                         m_GBiFast,                                        false, "Fast methods for Generalized Bi-prediction(GBi)\n")
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  ("LADF",                                            m_LadfEnabed,                                     false, "Luma adaptive deblocking filter QP Offset(L0414)")
  ("LadfNumIntervals",                                m_LadfNumIntervals,                                   3, "LADF number of intervals (2-5, inclusive)")
  ("LadfQpOffset",                                    cfg_LadfQpOffset,                      cfg_LadfQpOffset, "LADF QP offset")
  ("LadfIntervalLowerBound",                          cfg_LadfIntervalLowerBound,  cfg_LadfIntervalLowerBound, "LADF lower bound for 2nd lowest interval")
#endif
  ("MHIntra",                                         m_MHIntra,                                        false, "Enable MHIntra mode")
  ("Triangle",                                        m_Triangle,                                       false, "Enable triangular shape motion vector prediction (0:off, 1:on)")
  ("HashME",                                          m_HashME,                                         false, "Enable hash motion estimation (0:off, 1:on)")

  ("AllowDisFracMMVD",                                m_allowDisFracMMVD,                               false, "Disable fractional MVD in MMVD mode adaptively")
  ("AffineAmvr",                                      m_AffineAmvr,                                     false, "Eanble AMVR for affine inter mode")
  ("AffineAmvrEncOpt",                                m_AffineAmvrEncOpt,                               false, "Enable encoder optimization of affine AMVR")
  ("DMVR",                                            m_DMVR,                                           false, "Decoder-side Motion Vector Refinement")
  ("MmvdDisNum",                                      m_MmvdDisNum,                                     8,     "Number of MMVD Distance Entries")
#if !JVET_O1136_TS_BDPCM_SIGNALLING    
  ( "RDPCM",                                          m_RdpcmMode,                                       false, "RDPCM")
#endif    
#if JVET_O0119_BASE_PALETTE_444
  ("PLT",                                             m_PLTMode,                                           0u, "PLTMode (0x1:enabled, 0x0:disabled)  [default: disabled]")
#endif
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  ("JointCbCr",                                       m_JointCbCrMode,                                  false, "Enable joint coding of chroma residuals (JointCbCr, 0:off, 1:on)")
#endif
  ( "IBC",                                            m_IBCMode,                                           0u, "IBCMode (0x1:enabled, 0x0:disabled)  [default: disabled]")
  ( "IBCLocalSearchRangeX",                           m_IBCLocalSearchRangeX,                            128u, "Search range of IBC local search in x direction")
  ( "IBCLocalSearchRangeY",                           m_IBCLocalSearchRangeY,                            128u, "Search range of IBC local search in y direction")
  ( "IBCHashSearch",                                  m_IBCHashSearch,                                     1u, "Hash based IBC search")
  ( "IBCHashSearchMaxCand",                           m_IBCHashSearchMaxCand,                            256u, "Max candidates for hash based IBC search")
  ( "IBCHashSearchRange4SmallBlk",                    m_IBCHashSearchRange4SmallBlk,                     256u, "Small block search range in based IBC search")
  ( "IBCFastMethod",                                  m_IBCFastMethod,                                     6u, "Fast methods for IBC")

  ("WrapAround",                                      m_wrapAround,                                     false, "Enable horizontal wrap-around motion compensation for inter prediction (0:off, 1:on)  [default: off]")
  ("WrapAroundOffset",                                m_wrapAroundOffset,                                  0u, "Offset in luma samples used for computing the horizontal wrap-around position")

  // ADD_NEW_TOOL : (encoder app) add parsing parameters here
  ("LoopFilterAcrossVirtualBoundariesDisabledFlag",   m_loopFilterAcrossVirtualBoundariesDisabledFlag,  false, "Disable in-loop filtering operations across the virtual boundaries (0:off, 1:on)  [default: off]")
  ("NumVerVirtualBoundaries",                         m_numVerVirtualBoundaries,                           0u, "Number of vertical virtual boundaries (0-3, inclusive)")
  ("NumHorVirtualBoundaries",                         m_numHorVirtualBoundaries,                           0u, "Number of horizontal virtual boundaries (0-3, inclusive)")
  ("VirtualBoundariesPosX",                           cfg_virtualBoundariesPosX,    cfg_virtualBoundariesPosX, "Locations of the vertical virtual boundaries in units of luma samples")
  ("VirtualBoundariesPosY",                           cfg_virtualBoundariesPosY,    cfg_virtualBoundariesPosY, "Locations of the horizontal virtual boundaries in units of luma samples")
  ("EncDbOpt",                                        m_encDbOpt,                                       false, "Encoder optimization with deblocking filter")
#if JVET_O0432_LMCS_ENCODER
  ("LMCSEnable",                                      m_lumaReshapeEnable,                              false, "Enable LMCS (luma mapping with chroma scaling")
  ("LMCSSignalType",                                  m_reshapeSignalType,                                 0u, "Input signal type: 0:SDR, 1:HDR-PQ, 2:HDR-HLG")
  ("LMCSUpdateCtrl",                                  m_updateCtrl,                                         0, "LMCS model update control: 0:RA, 1:AI, 2:LDB/LDP")
  ("LMCSAdpOption",                                   m_adpOption,                                          0, "LMCS adaptation options: 0:automatic(default),"
                                                                                                               "1: rsp both (CW66 for QP<=22), 2: rsp TID0 (for all QP),"
                                                                                                               "3: rsp inter(CW66 for QP<=22), 4: rsp inter(for all QP).")
  ("LMCSInitialCW",                                   m_initialCW,                                         0u, "LMCS initial total codeword (0~1023) when LMCSAdpOption > 0")
#else
  ("LumaReshapeEnable",                               m_lumaReshapeEnable,                              false, "Enable Reshaping for Luma Channel")
  ("ReshapeSignalType",                               m_reshapeSignalType,                                 0u, "Input signal type: 0: SDR, 1:PQ, 2:HLG")
#endif
  ("IntraCMD",                                        m_intraCMD,                                          0u, "IntraChroma MD: 0: none, 1:fixed to default wPSNR weight")
  ("LCTUFast",                                        m_useFastLCTU,                                    false, "Fast methods for large CTU")
  ("FastMrg",                                         m_useFastMrg,                                     false, "Fast methods for inter merge")
  ("PBIntraFast",                                     m_usePbIntraFast,                                 false, "Fast assertion if the intra mode is probable")
  ("AMaxBT",                                          m_useAMaxBT,                                      false, "Adaptive maximal BT-size")
  ("E0023FastEnc",                                    m_e0023FastEnc,                                    true, "Fast encoding setting for QTBT (proposal E0023)")
  ("ContentBasedFastQtbt",                            m_contentBasedFastQtbt,                           false, "Signal based QTBT speed-up")
  ("UseNonLinearAlfLuma",                             m_useNonLinearAlfLuma,                             true, "Non-linear adaptive loop filters for Luma Channel")
  ("UseNonLinearAlfChroma",                           m_useNonLinearAlfChroma,                           true, "Non-linear adaptive loop filters for Chroma Channels")
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  ("MaxNumAlfAlternativesChroma",                     m_maxNumAlfAlternativesChroma,
                                                                    (unsigned)MAX_NUM_ALF_ALTERNATIVES_CHROMA, std::string("Maximum number of alternative Chroma filters (1-") + std::to_string(MAX_NUM_ALF_ALTERNATIVES_CHROMA) + std::string (", inclusive)") )
#endif
  ("MIP",                                             m_MIP,                                             true,  "Enable MIP (matrix-based intra prediction)")
  ("FastMIP",                                         m_useFastMIP,                                     false,  "Fast encoder search for MIP (matrix-based intra prediction)")
#if JVET_O0050_LOCAL_DUAL_TREE
  ("FastLocalDualTreeMode",                           m_fastLocalDualTreeMode,                              0,  "Fast intra pass coding for local dual-tree in intra coding region, 0: off, 1: use threshold, 2: one intra mode only")
#endif
  // Unit definition parameters
  ("MaxCUWidth",                                      m_uiMaxCUWidth,                                     64u)
  ("MaxCUHeight",                                     m_uiMaxCUHeight,                                    64u)
  // todo: remove defaults from MaxCUSize
  ("MaxCUSize,s",                                     m_uiMaxCUWidth,                                     64u, "Maximum CU size")
  ("MaxCUSize,s",                                     m_uiMaxCUHeight,                                    64u, "Maximum CU size")
  ("MaxPartitionDepth,h",                             m_uiMaxCUDepth,                                      4u, "CU depth")

#if MAX_TB_SIZE_SIGNALLING
  ("Log2MaxTbSize",                                   m_log2MaxTbSize,                                      6, "Maximum transform block size in logarithm base 2 (Default: 6)")
#endif

  // Coding structure paramters
  ("IntraPeriod,-ip",                                 m_iIntraPeriod,                                      -1, "Intra period in frames, (-1: only first frame)")
  ("DecodingRefreshType,-dr",                         m_iDecodingRefreshType,                               0, "Intra refresh type (0:none 1:CRA 2:IDR 3:RecPointSEI)")
  ("GOPSize,g",                                       m_iGOPSize,                                           1, "GOP size of temporal structure")
  ("ReWriteParamSets",                                m_rewriteParamSets,                           false, "Enable rewriting of Parameter sets before every (intra) random access point")
  //Alias with same name as in HM
  ("ReWriteParamSetsFlag",                            m_rewriteParamSets,                           false, "Alias for ReWriteParamSets")
  ("IDRRefParamList",                                 m_idrRefParamList,                            false, "Enable indication of reference picture list syntax elements in slice headers of IDR pictures")
  // motion search options
  ("DisableIntraInInter",                             m_bDisableIntraPUsInInterSlices,                  false, "Flag to disable intra PUs in inter slices")
  ("FastSearch",                                      tmpMotionEstimationSearchMethod,  int(MESEARCH_DIAMOND), "0:Full search 1:Diamond 2:Selective 3:Enhanced Diamond")
  ("SearchRange,-sr",                                 m_iSearchRange,                                      96, "Motion search range")
  ("BipredSearchRange",                               m_bipredSearchRange,                                  4, "Motion search range for bipred refinement")
  ("MinSearchWindow",                                 m_minSearchWindow,                                    8, "Minimum motion search window size for the adaptive window ME")
  ("RestrictMESampling",                              m_bRestrictMESampling,                            false, "Restrict ME Sampling for selective inter motion search")
  ("ClipForBiPredMEEnabled",                          m_bClipForBiPredMeEnabled,                        false, "Enables clipping in the Bi-Pred ME. It is disabled to reduce encoder run-time")
  ("FastMEAssumingSmootherMVEnabled",                 m_bFastMEAssumingSmootherMVEnabled,                true, "Enables fast ME assuming a smoother MV.")

  ("HadamardME",                                      m_bUseHADME,                                       true, "Hadamard ME for fractional-pel")
  ("ASR",                                             m_bUseASR,                                        false, "Adaptive motion search range");
  opts.addOptions()

  // Mode decision parameters
  ("LambdaModifier0,-LM0",                            m_adLambdaModifier[ 0 ],                  ( double )1.0, "Lambda modifier for temporal layer 0. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier1,-LM1",                            m_adLambdaModifier[ 1 ],                  ( double )1.0, "Lambda modifier for temporal layer 1. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier2,-LM2",                            m_adLambdaModifier[ 2 ],                  ( double )1.0, "Lambda modifier for temporal layer 2. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier3,-LM3",                            m_adLambdaModifier[ 3 ],                  ( double )1.0, "Lambda modifier for temporal layer 3. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier4,-LM4",                            m_adLambdaModifier[ 4 ],                  ( double )1.0, "Lambda modifier for temporal layer 4. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier5,-LM5",                            m_adLambdaModifier[ 5 ],                  ( double )1.0, "Lambda modifier for temporal layer 5. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier6,-LM6",                            m_adLambdaModifier[ 6 ],                  ( double )1.0, "Lambda modifier for temporal layer 6. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifierI,-LMI",                            cfg_adIntraLambdaModifier,    cfg_adIntraLambdaModifier, "Lambda modifiers for Intra pictures, comma separated, up to one the number of temporal layer. If entry for temporalLayer exists, then use it, else if some are specified, use the last, else use the standard LambdaModifiers.")
  ("IQPFactor,-IQF",                                  m_dIntraQpFactor,                                  -1.0, "Intra QP Factor for Lambda Computation. If negative, the default will scale lambda based on GOP size (unless LambdaFromQpEnable then IntraQPOffset is used instead)")

  /* Quantization parameters */
#if QP_SWITCHING_FOR_PARALLEL
  ("QP,q",                                            m_iQP,                                               30, "Qp value")
  ("QPIncrementFrame,-qpif",                          m_qpIncrementAtSourceFrame,   OptionalValue<uint32_t>(), "If a source file frame number is specified, the internal QP will be incremented for all POCs associated with source frames >= frame number. If empty, do not increment.")
#else
  ("QP,q",                                            m_fQP,                                             30.0, "Qp value, if value is float, QP is switched once during encoding")
#endif
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  ("IntraQPOffset",                                   m_intraQPOffset,                                      0, "Qp offset value for intra slice, typically determined based on GOP size")
  ("LambdaFromQpEnable",                              m_lambdaFromQPEnable,                             false, "Enable flag for derivation of lambda from QP")
#endif
  ("DeltaQpRD,-dqr",                                  m_uiDeltaQpRD,                                       0u, "max dQp offset for slice")
  ("MaxDeltaQP,d",                                    m_iMaxDeltaQP,                                        0, "max dQp offset for block")
  ("MaxCuDQPSubdiv,-dqd",                             m_cuQpDeltaSubdiv,                                    0, "Maximum subdiv for CU luma Qp adjustment")
  ("MaxCuChromaQpOffsetSubdiv",                       m_cuChromaQpOffsetSubdiv,                            -1, "Maximum subdiv for CU chroma Qp adjustment - set less than 0 to disable")
  ("FastDeltaQP",                                     m_bFastDeltaQP,                                   false, "Fast Delta QP Algorithm")
#if SHARP_LUMA_DELTA_QP
  ("LumaLevelToDeltaQPMode",                          lumaLevelToDeltaQPMode,                              0u, "Luma based Delta QP 0(default): not used. 1: Based on CTU average, 2: Based on Max luma in CTU")
#if !WCG_EXT
  ("LumaLevelToDeltaQPMaxValWeight",                  m_lumaLevelToDeltaQPMapping.maxMethodWeight,        1.0, "Weight of block max luma val when LumaLevelToDeltaQPMode = 2")
#endif
  ("LumaLevelToDeltaQPMappingLuma",                   cfg_lumaLeveltoDQPMappingLuma,  cfg_lumaLeveltoDQPMappingLuma, "Luma to Delta QP Mapping - luma thresholds")
  ("LumaLevelToDeltaQPMappingDQP",                    cfg_lumaLeveltoDQPMappingQP,  cfg_lumaLeveltoDQPMappingQP, "Luma to Delta QP Mapping - DQP values")
#endif
#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE
  ("UseIdentityTableForNon420Chroma",                 m_useIdentityTableForNon420Chroma,                 true, "True: Indicates that 422/444 chroma uses identity chroma QP mapping tables; False: explicit Qp table may be specified in config")
  ("SameCQPTablesForAllChroma",                       m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag,                        true, "0: Different tables for Cb, Cr and joint Cb-Cr components, 1 (default): Same tables for all three chroma components")
  ("QpInValCb",                                       cfg_qpInValCb,                            cfg_qpInValCb, "Input coordinates for the QP table for Cb component")
  ("QpOutValCb",                                      cfg_qpOutValCb,                          cfg_qpOutValCb, "Output coordinates for the QP table for Cb component")
  ("QpInValCr",                                       cfg_qpInValCr,                            cfg_qpInValCr, "Input coordinates for the QP table for Cr component")
  ("QpOutValCr",                                      cfg_qpOutValCr,                          cfg_qpOutValCr, "Output coordinates for the QP table for Cr component")
  ("QpInValCbCr",                                     cfg_qpInValCbCr,                        cfg_qpInValCbCr, "Input coordinates for the QP table for joint Cb-Cr component")
  ("QpOutValCbCr",                                    cfg_qpOutValCbCr,                      cfg_qpOutValCbCr, "Output coordinates for the QP table for joint Cb-Cr component")
#endif
  ("CbQpOffset,-cbqpofs",                             m_cbQpOffset,                                         0, "Chroma Cb QP Offset")
  ("CrQpOffset,-crqpofs",                             m_crQpOffset,                                         0, "Chroma Cr QP Offset")
  ("CbQpOffsetDualTree",                              m_cbQpOffsetDualTree,                                 0, "Chroma Cb QP Offset for dual tree")
  ("CrQpOffsetDualTree",                              m_crQpOffsetDualTree,                                 0, "Chroma Cr QP Offset for dual tree")
  ("CbCrQpOffset,-cbcrqpofs",                         m_cbCrQpOffset,                                      -1, "QP Offset for joint Cb-Cr mode")
  ("CbCrQpOffsetDualTree",                            m_cbCrQpOffsetDualTree,                               0, "QP Offset for joint Cb-Cr mode in dual tree")
#if ER_CHROMA_QP_WCG_PPS
  ("WCGPPSEnable",                                    m_wcgChromaQpControl.enabled,                     false, "1: Enable the WCG PPS chroma modulation scheme. 0 (default) disabled")
  ("WCGPPSCbQpScale",                                 m_wcgChromaQpControl.chromaCbQpScale,               1.0, "WCG PPS Chroma Cb QP Scale")
  ("WCGPPSCrQpScale",                                 m_wcgChromaQpControl.chromaCrQpScale,               1.0, "WCG PPS Chroma Cr QP Scale")
  ("WCGPPSChromaQpScale",                             m_wcgChromaQpControl.chromaQpScale,                 0.0, "WCG PPS Chroma QP Scale")
  ("WCGPPSChromaQpOffset",                            m_wcgChromaQpControl.chromaQpOffset,                0.0, "WCG PPS Chroma QP Offset")
#endif
#if W0038_CQP_ADJ
  ("SliceChromaQPOffsetPeriodicity",                  m_sliceChromaQpOffsetPeriodicity,                    0u, "Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.")
  ("SliceCbQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[0],              0, "Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
  ("SliceCrQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[1],              0, "Chroma Cr QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
#endif

  ("AdaptiveQP,-aq",                                  m_bUseAdaptiveQP,                                 false, "QP adaptation based on a psycho-visual model")
  ("MaxQPAdaptationRange,-aqr",                       m_iQPAdaptationRange,                                 6, "QP adaptation range")
#if ENABLE_QPA
  ("PerceptQPA,-qpa",                                 m_bUsePerceptQPA,                                 false, "perceptually motivated input-adaptive QP modification (default: 0 = off, ignored if -aq is set)")
  ("WPSNR,-wpsnr",                                    m_bUseWPSNR,                                      false, "output perceptually weighted peak SNR (WPSNR) instead of PSNR")
#endif
  ("dQPFile,m",                                       m_dQPFileName,                               string(""), "dQP file name")
  ("RDOQ",                                            m_useRDOQ,                                         true)
  ("RDOQTS",                                          m_useRDOQTS,                                       true)
#if T0196_SELECTIVE_RDOQ
  ("SelectiveRDOQ",                                   m_useSelectiveRDOQ,                               false, "Enable selective RDOQ")
#endif
  ("RDpenalty",                                       m_rdPenalty,                                          0, "RD-penalty for 32x32 TU for intra in non-intra slices. 0:disabled  1:RD-penalty  2:maximum RD-penalty")

  // Deblocking filter parameters
  ("LoopFilterDisable",                               m_bLoopFilterDisable,                             false)
  ("LoopFilterOffsetInPPS",                           m_loopFilterOffsetInPPS,                           true)
  ("LoopFilterBetaOffset_div2",                       m_loopFilterBetaOffsetDiv2,                           0)
  ("LoopFilterTcOffset_div2",                         m_loopFilterTcOffsetDiv2,                             0)
#if W0038_DB_OPT
  ("DeblockingFilterMetric",                          m_deblockingFilterMetric,                             0)
#else
  ("DeblockingFilterMetric",                          m_DeblockingFilterMetric,                         false)
#endif
  // Coding tools
  ("CrossComponentPrediction",                        m_crossComponentPredictionEnabledFlag,            false, "Enable the use of cross-component prediction (not valid in V1 profiles)")
  ("ReconBasedCrossCPredictionEstimate",              m_reconBasedCrossCPredictionEstimate,             false, "When determining the alpha value for cross-component prediction, use the decoded residual rather than the pre-transform encoder-side residual")
  ("SaoLumaOffsetBitShift",                           saoOffsetBitShift[CHANNEL_TYPE_LUMA],                 0, "Specify the luma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
  ("SaoChromaOffsetBitShift",                         saoOffsetBitShift[CHANNEL_TYPE_CHROMA],               0, "Specify the chroma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
  ("TransformSkip",                                   m_useTransformSkip,                               false, "Intra transform skipping")
  ("TransformSkipFast",                               m_useTransformSkipFast,                           false, "Fast encoder search for transform skipping, winner takes it all mode.")
  ("TransformSkipLog2MaxSize",                        m_log2MaxTransformSkipBlockSize,                     5U, "Specify transform-skip maximum size. Minimum 2, Maximum 5. (not valid in V1 profiles)")
#if JVET_O1136_TS_BDPCM_SIGNALLING
  ("BDPCM",                                           m_useBDPCM,                                       false, "BDPCM")
#endif
  ("ISPFast",                                         m_useFastISP,                                     false, "Fast encoder search for ISP")
  ("ImplicitResidualDPCM",                            m_rdpcmEnabledFlag[RDPCM_SIGNAL_IMPLICIT],        false, "Enable implicitly signalled residual DPCM for intra (also known as sample-adaptive intra predict) (not valid in V1 profiles)")
  ("ExplicitResidualDPCM",                            m_rdpcmEnabledFlag[RDPCM_SIGNAL_EXPLICIT],        false, "Enable explicitly signalled residual DPCM for inter (not valid in V1 profiles)")
  ("ResidualRotation",                                m_transformSkipRotationEnabledFlag,               false, "Enable rotation of transform-skipped and transquant-bypassed TUs through 180 degrees prior to entropy coding (not valid in V1 profiles)")
  ("SingleSignificanceMapContext",                    m_transformSkipContextEnabledFlag,                false, "Enable, for transform-skipped and transquant-bypassed TUs, the selection of a single significance map context variable for all coefficients (not valid in V1 profiles)")
  ("GolombRiceParameterAdaptation",                   m_persistentRiceAdaptationEnabledFlag,            false, "Enable the adaptation of the Golomb-Rice parameter over the course of each slice")
  ("AlignCABACBeforeBypass",                          m_cabacBypassAlignmentEnabledFlag,                false, "Align the CABAC engine to a defined fraction of a bit prior to coding bypass data. Must be 1 in high bit rate profile, 0 otherwise")
  ("SAO",                                             m_bUseSAO,                                         true, "Enable Sample Adaptive Offset")
  ("TestSAODisableAtPictureLevel",                    m_bTestSAODisableAtPictureLevel,                  false, "Enables the testing of disabling SAO at the picture level after having analysed all blocks")
  ("SaoEncodingRate",                                 m_saoEncodingRate,                                 0.75, "When >0 SAO early picture termination is enabled for luma and chroma")
  ("SaoEncodingRateChroma",                           m_saoEncodingRateChroma,                            0.5, "The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma")
  ("MaxNumOffsetsPerPic",                             m_maxNumOffsetsPerPic,                             2048, "Max number of SAO offset per picture (Default: 2048)")
  ("SAOLcuBoundary",                                  m_saoCtuBoundary,                                 false, "0: right/bottom CTU boundary areas skipped from SAO parameter estimation, 1: non-deblocked pixels are used for those areas")
  ("SAOGreedyEnc",                                    m_saoGreedyMergeEnc,                              false, "SAO greedy merge encoding algorithm")
  ("SliceMode",                                       tmpSliceMode,                            int(NO_SLICES), "0: Disable all Recon slice limits, 1: (deprecated #CTU), 2: (deprecated #bytes), 3:specify tiles per slice, 4: one brick per slice")
  ("SliceArgument",                                   m_sliceArgument,                                      0, "Depending on SliceMode being:"
                                                                                                               "\t1: max number of CTUs per slice"
                                                                                                               "\t2: max number of bytes per slice"
                                                                                                               "\t3: max number of tiles per slice")
  ("LFCrossSliceBoundaryFlag",                        m_bLFCrossSliceBoundaryFlag,                       true)

  ("ConstrainedIntraPred",                            m_bUseConstrainedIntraPred,                       false, "Constrained Intra Prediction")
  ("FastUDIUseMPMEnabled",                            m_bFastUDIUseMPMEnabled,                           true, "If enabled, adapt intra direction search, accounting for MPM")
  ("FastMEForGenBLowDelayEnabled",                    m_bFastMEForGenBLowDelayEnabled,                   true, "If enabled use a fast ME for generalised B Low Delay slices")
  ("UseBLambdaForNonKeyLowDelayPictures",             m_bUseBLambdaForNonKeyLowDelayPictures,            true, "Enables use of B-Lambda for non-key low-delay pictures")
#if !JVET_O0525_REMOVE_PCM
  ("PCMEnabledFlag",                                  m_usePCM,                                         false)
  ("PCMLog2MaxSize",                                  m_pcmLog2MaxSize,                                    5u)
  ("PCMLog2MinSize",                                  m_uiPCMLog2MinSize,                                  3u)

  ("PCMInputBitDepthFlag",                            m_bPCMInputBitDepthFlag,                           true)
  ("PCMFilterDisableFlag",                            m_bPCMFilterDisableFlag,                          false)
#endif
  ("IntraReferenceSmoothing",                         m_enableIntraReferenceSmoothing,                   true, "0: Disable use of intra reference smoothing (not valid in V1 profiles). 1: Enable use of intra reference smoothing (same as V1)")
  ("WeightedPredP,-wpP",                              m_useWeightedPred,                                false, "Use weighted prediction in P slices")
  ("WeightedPredB,-wpB",                              m_useWeightedBiPred,                              false, "Use weighted (bidirectional) prediction in B slices")
  ("WeightedPredMethod,-wpM",                         tmpWeightedPredictionMethod, int(WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT), "Weighted prediction method")
  ("Log2ParallelMergeLevel",                          m_log2ParallelMergeLevel,                            2u, "Parallel merge estimation region")
    //deprecated copies of renamed tile parameters
  ("UniformSpacingIdc",                               m_tileUniformSpacingFlag,                         false,      "deprecated alias of TileUniformSpacing")
  ("TileUniformSpacing",                              m_tileUniformSpacingFlag,                         false,      "Indicates that tile columns and rows are distributed uniformly")
  ("NumTileColumnsMinus1",                            m_numTileColumnsMinus1,                               0,          "Number of tile columns in a picture minus 1")
  ("NumTileRowsMinus1",                               m_numTileRowsMinus1,                                  0,          "Number of rows in a picture minus 1")
  ("TileColumnWidthArray",                            cfg_ColumnWidth,                        cfg_ColumnWidth, "Array containing tile column width values in units of CTU")
  ("TileRowHeightArray",                              cfg_RowHeight,                            cfg_RowHeight, "Array containing tile row height values in units of CTU")
  ("LFCrossTileBoundaryFlag",                         m_bLFCrossTileBoundaryFlag,                        true, "1: cross-tile-boundary loop filtering. 0:non-cross-tile-boundary loop filtering")
  ("WaveFrontSynchro",                                m_entropyCodingSyncEnabledFlag,                   false, "0: entropy coding sync disabled; 1 entropy coding sync enabled")

  ("RectSliceFlag",                                   m_rectSliceFlag,                                  true, "Rectangular slice flag")
  ("NumRectSlicesInPicMinus1",                        m_numSlicesInPicMinus1,                              0, "Number slices in pic minus 1")
  ("LoopFilterAcrossTileGroupsEnabledFlag",           m_loopFilterAcrossSlicesEnabledFlag,              false, "Loop Filter Across Tile Groups Flag")
  ("SignalledIdFlag",                                 m_signalledSliceIdFlag,                           false, "Signalled Slice ID Flag")
  ("SignalledSliceIdLengthMinus1",                    m_signalledSliceIdLengthMinus1,                       0, "Signalled Tile Group Length minus 1")
  ("RectSlicesBoundaryArray",                         cfg_SliceIdx,                              cfg_SliceIdx, "Rectangular slices boundaries in Pic")
  ("SignalledSliceId",                                cfg_SignalledSliceId,                       cfg_SliceIdx, "Signalled rectangular slice ID")

  ("ScalingList",                                     m_useScalingListId,                    SCALING_LIST_OFF, "0/off: no scaling list, 1/default: default scaling lists, 2/file: scaling lists specified in ScalingListFile")
  ("ScalingListFile",                                 m_scalingListFileName,                       string(""), "Scaling list file name. Use an empty string to produce help.")
  ("DepQuant",                                        m_depQuantEnabledFlag,                                          true )
  ("SignHideFlag,-SBH",                               m_signDataHidingEnabledFlag,                                    false )
  ("MaxNumMergeCand",                                 m_maxNumMergeCand,                                   5u, "Maximum number of merge candidates")
  ("MaxNumAffineMergeCand",                           m_maxNumAffineMergeCand,                             5u, "Maximum number of affine merge candidates")
  ("MaxNumTriangleCand",                              m_maxNumTriangleCand,                                5u, "Maximum number of triangle candidates")
#if JVET_O0455_IBC_MAX_MERGE_NUM
  ("MaxNumIBCMergeCand",                              m_maxNumIBCMergeCand,                                6u, "Maximum number of IBC merge candidates")
#endif
    /* Misc. */
  ("SEIDecodedPictureHash,-dph",                      tmpDecodedPictureHashSEIMappedType,                   0, "Control generation of decode picture hash SEI messages\n"
                                                                                                               "\t3: checksum\n"
                                                                                                               "\t2: CRC\n"
                                                                                                               "\t1: use MD5\n"
                                                                                                               "\t0: disable")
  ("TMVPMode",                                        m_TMVPModeId,                                         1, "TMVP mode 0: TMVP disable for all slices. 1: TMVP enable for all slices (default) 2: TMVP enable for certain slices only")
  ("FEN",                                             tmpFastInterSearchMode,   int(FASTINTERSEARCH_DISABLED), "fast encoder setting")
  ("ECU",                                             m_bUseEarlyCU,                                    false, "Early CU setting")
  ("FDM",                                             m_useFastDecisionForMerge,                         true, "Fast decision for Merge RD Cost")
  ("CFM",                                             m_bUseCbfFastMode,                                false, "Cbf fast mode setting")
  ("ESD",                                             m_useEarlySkipDetection,                          false, "Early SKIP detection setting")
  ( "RateControl",                                    m_RCEnableRateControl,                            false, "Rate control: enable rate control" )
  ( "TargetBitrate",                                  m_RCTargetBitrate,                                    0, "Rate control: target bit-rate" )
  ( "KeepHierarchicalBit",                            m_RCKeepHierarchicalBit,                              0, "Rate control: 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit allocation" )
  ( "LCULevelRateControl",                            m_RCLCULevelRC,                                    true, "Rate control: true: CTU level RC; false: picture level RC" )
  ( "RCLCUSeparateModel",                             m_RCUseLCUSeparateModel,                           true, "Rate control: use CTU level separate R-lambda model" )
  ( "InitialQP",                                      m_RCInitialQP,                                        0, "Rate control: initial QP" )
  ( "RCForceIntraQP",                                 m_RCForceIntraQP,                                 false, "Rate control: force intra QP to be equal to initial QP" )
#if U0132_TARGET_BITS_SATURATION
  ( "RCCpbSaturation",                                m_RCCpbSaturationEnabled,                         false, "Rate control: enable target bits saturation to avoid CPB overflow and underflow" )
  ( "RCCpbSize",                                      m_RCCpbSize,                                         0u, "Rate control: CPB size" )
  ( "RCInitialCpbFullness",                           m_RCInitialCpbFullness,                             0.9, "Rate control: initial CPB fullness" )
#endif
  ("TransquantBypassEnable",                          m_TransquantBypassEnabledFlag,                    false, "transquant_bypass_enabled_flag indicator in PPS")
  ("TransquantBypassEnableFlag",                      m_TransquantBypassEnabledFlag,                    false, "deprecated and obsolete, but still needed for compatibility reasons")
  ("CUTransquantBypassFlagForce",                     m_CUTransquantBypassFlagForce,                    false, "Force transquant bypass mode, when transquant_bypass_enabled_flag is enabled")
  ("CostMode",                                        m_costMode,                         COST_STANDARD_LOSSY, "Use alternative cost functions: choose between 'lossy', 'sequence_level_lossless', 'lossless' (which forces QP to " MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP) ") and 'mixed_lossless_lossy' (which used QP'=" MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME) " for pre-estimates of transquant-bypass blocks).")
  ("RecalculateQPAccordingToLambda",                  m_recalculateQPAccordingToLambda,                 false, "Recalculate QP values according to lambda values. Do not suggest to be enabled in all intra case")
  ("SEIActiveParameterSets",                          m_activeParameterSetsSEIEnabled,                      0, "Enable generation of active parameter sets SEI messages");
  opts.addOptions()
  ("VuiParametersPresent,-vui",                       m_vuiParametersPresentFlag,                       false, "Enable generation of vui_parameters()")
  ("AspectRatioInfoPresent",                          m_aspectRatioInfoPresentFlag,                     false, "Signals whether aspect_ratio_idc is present")
  ("AspectRatioIdc",                                  m_aspectRatioIdc,                                     0, "aspect_ratio_idc")
  ("SarWidth",                                        m_sarWidth,                                           0, "horizontal size of the sample aspect ratio")
  ("SarHeight",                                       m_sarHeight,                                          0, "vertical size of the sample aspect ratio")
  ("ColourDescriptionPresent",                        m_colourDescriptionPresentFlag,                   false, "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
  ("ColourPrimaries",                                 m_colourPrimaries,                                    2, "Indicates chromaticity coordinates of the source primaries")
  ("TransferCharacteristics",                         m_transferCharacteristics,                            2, "Indicates the opto-electronic transfer characteristics of the source")
  ("MatrixCoefficients",                              m_matrixCoefficients,                                 2, "Describes the matrix coefficients used in deriving luma and chroma from RGB primaries")
  ("ChromaLocInfoPresent",                            m_chromaLocInfoPresentFlag,                       false, "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
  ("ChromaSampleLocTypeTopField",                     m_chromaSampleLocTypeTopField,                        0, "Specifies the location of chroma samples for top field")
  ("ChromaSampleLocTypeBottomField",                  m_chromaSampleLocTypeBottomField,                     0, "Specifies the location of chroma samples for bottom field")
  ("ChromaSampleLocType",                             m_chromaSampleLocType,                                0, "Specifies the location of chroma samples for progressive content")
  ("OverscanInfoPresent",                             m_overscanInfoPresentFlag,                        false, "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("OverscanAppropriate",                             m_overscanAppropriateFlag,                        false, "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("VideoSignalTypePresent",                          m_videoSignalTypePresentFlag,                     false, "Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present")
  ("VideoFullRange",                                  m_videoFullRangeFlag,                             false, "Indicates the black level and range of luma and chroma signals");
  opts.addOptions()
  ("SEIColourRemappingInfoFileRoot,-cri",             m_colourRemapSEIFileRoot,                    string(""), "Colour Remapping Information SEI parameters root file name (wo num ext)")
  ("SEIRecoveryPoint",                                m_recoveryPointSEIEnabled,                        false, "Control generation of recovery point SEI messages")
  ("SEIBufferingPeriod",                              m_bufferingPeriodSEIEnabled,                      false, "Control generation of buffering period SEI messages")
  ("SEIPictureTiming",                                m_pictureTimingSEIEnabled,                        false, "Control generation of picture timing SEI messages")
  ("SEIToneMappingInfo",                              m_toneMappingInfoSEIEnabled,                      false, "Control generation of Tone Mapping SEI messages")
  ("SEIToneMapId",                                    m_toneMapId,                                          0, "Specifies Id of Tone Mapping SEI message for a given session")
  ("SEIToneMapCancelFlag",                            m_toneMapCancelFlag,                              false, "Indicates that Tone Mapping SEI message cancels the persistence or follows")
  ("SEIToneMapPersistenceFlag",                       m_toneMapPersistenceFlag,                          true, "Specifies the persistence of the Tone Mapping SEI message")
  ("SEIToneMapCodedDataBitDepth",                     m_toneMapCodedDataBitDepth,                           8, "Specifies Coded Data BitDepth of Tone Mapping SEI messages")
  ("SEIToneMapTargetBitDepth",                        m_toneMapTargetBitDepth,                              8, "Specifies Output BitDepth of Tone mapping function")
  ("SEIToneMapModelId",                               m_toneMapModelId,                                     0, "Specifies Model utilized for mapping coded data into target_bit_depth range\n"
                                                                                                               "\t0:  linear mapping with clipping\n"
                                                                                                               "\t1:  sigmoidal mapping\n"
                                                                                                               "\t2:  user-defined table mapping\n"
                                                                                                               "\t3:  piece-wise linear mapping\n"
                                                                                                               "\t4:  luminance dynamic range information ")
  ("SEIToneMapMinValue",                              m_toneMapMinValue,                                    0, "Specifies the minimum value in mode 0")
  ("SEIToneMapMaxValue",                              m_toneMapMaxValue,                                 1023, "Specifies the maximum value in mode 0")
  ("SEIToneMapSigmoidMidpoint",                       m_sigmoidMidpoint,                                  512, "Specifies the centre point in mode 1")
  ("SEIToneMapSigmoidWidth",                          m_sigmoidWidth,                                     960, "Specifies the distance between 5% and 95% values of the target_bit_depth in mode 1")
  ("SEIToneMapStartOfCodedInterval",                  cfg_startOfCodedInterval,      cfg_startOfCodedInterval, "Array of user-defined mapping table")
  ("SEIToneMapNumPivots",                             m_numPivots,                                          0, "Specifies the number of pivot points in mode 3")
  ("SEIToneMapCodedPivotValue",                       cfg_codedPivotValue,                cfg_codedPivotValue, "Array of pivot point")
  ("SEIToneMapTargetPivotValue",                      cfg_targetPivotValue,              cfg_targetPivotValue, "Array of pivot point")
  ("SEIToneMapCameraIsoSpeedIdc",                     m_cameraIsoSpeedIdc,                                  0, "Indicates the camera ISO speed for daylight illumination")
  ("SEIToneMapCameraIsoSpeedValue",                   m_cameraIsoSpeedValue,                              400, "Specifies the camera ISO speed for daylight illumination of Extended_ISO")
  ("SEIToneMapExposureIndexIdc",                      m_exposureIndexIdc,                                   0, "Indicates the exposure index setting of the camera")
  ("SEIToneMapExposureIndexValue",                    m_exposureIndexValue,                               400, "Specifies the exposure index setting of the camera of Extended_ISO")
  ("SEIToneMapExposureCompensationValueSignFlag",     m_exposureCompensationValueSignFlag,               false, "Specifies the sign of ExposureCompensationValue")
  ("SEIToneMapExposureCompensationValueNumerator",    m_exposureCompensationValueNumerator,                 0, "Specifies the numerator of ExposureCompensationValue")
  ("SEIToneMapExposureCompensationValueDenomIdc",     m_exposureCompensationValueDenomIdc,                  2, "Specifies the denominator of ExposureCompensationValue")
  ("SEIToneMapRefScreenLuminanceWhite",               m_refScreenLuminanceWhite,                          350, "Specifies reference screen brightness setting in units of candela per square metre")
  ("SEIToneMapExtendedRangeWhiteLevel",               m_extendedRangeWhiteLevel,                          800, "Indicates the luminance dynamic range")
  ("SEIToneMapNominalBlackLevelLumaCodeValue",        m_nominalBlackLevelLumaCodeValue,                    16, "Specifies luma sample value of the nominal black level assigned decoded pictures")
  ("SEIToneMapNominalWhiteLevelLumaCodeValue",        m_nominalWhiteLevelLumaCodeValue,                   235, "Specifies luma sample value of the nominal white level assigned decoded pictures")
  ("SEIToneMapExtendedWhiteLevelLumaCodeValue",       m_extendedWhiteLevelLumaCodeValue,                  300, "Specifies luma sample value of the extended dynamic range assigned decoded pictures")
  ("SEIChromaResamplingFilterHint",                   m_chromaResamplingFilterSEIenabled,               false, "Control generation of the chroma sampling filter hint SEI message")
  ("SEIChromaResamplingHorizontalFilterType",         m_chromaResamplingHorFilterIdc,                       2, "Defines the Index of the chroma sampling horizontal filter\n"
                                                                                                               "\t0: unspecified  - Chroma filter is unknown or is determined by the application"
                                                                                                               "\t1: User-defined - Filter coefficients are specified in the chroma sampling filter hint SEI message"
                                                                                                               "\t2: Standards-defined - ITU-T Rec. T.800 | ISO/IEC15444-1, 5/3 filter")
  ("SEIChromaResamplingVerticalFilterType",           m_chromaResamplingVerFilterIdc,                         2, "Defines the Index of the chroma sampling vertical filter\n"
                                                                                                               "\t0: unspecified  - Chroma filter is unknown or is determined by the application"
                                                                                                               "\t1: User-defined - Filter coefficients are specified in the chroma sampling filter hint SEI message"
                                                                                                               "\t2: Standards-defined - ITU-T Rec. T.800 | ISO/IEC15444-1, 5/3 filter")
  ("SEIFramePacking",                                 m_framePackingSEIEnabled,                         false, "Control generation of frame packing SEI messages")
  ("SEIFramePackingType",                             m_framePackingSEIType,                                0, "Define frame packing arrangement\n"
                                                                                                               "\t3: side by side - frames are displayed horizontally\n"
                                                                                                               "\t4: top bottom - frames are displayed vertically\n"
                                                                                                               "\t5: frame alternation - one frame is alternated with the other")
  ("SEIFramePackingId",                               m_framePackingSEIId,                                  0, "Id of frame packing SEI message for a given session")
  ("SEIFramePackingQuincunx",                         m_framePackingSEIQuincunx,                            0, "Indicate the presence of a Quincunx type video frame")
  ("SEIFramePackingInterpretation",                   m_framePackingSEIInterpretation,                      0, "Indicate the interpretation of the frame pair\n"
                                                                                                               "\t0: unspecified\n"
                                                                                                               "\t1: stereo pair, frame0 represents left view\n"
                                                                                                               "\t2: stereo pair, frame0 represents right view")
  ("SEISegmentedRectFramePacking",                    m_segmentedRectFramePackingSEIEnabled,            false, "Controls generation of segmented rectangular frame packing SEI messages")
  ("SEISegmentedRectFramePackingCancel",              m_segmentedRectFramePackingSEICancel,             false, "If equal to 1, cancels the persistence of any previous SRFPA SEI message")
  ("SEISegmentedRectFramePackingType",                m_segmentedRectFramePackingSEIType,                   0, "Specifies the arrangement of the frames in the reconstructed picture")
  ("SEISegmentedRectFramePackingPersistence",         m_segmentedRectFramePackingSEIPersistence,        false, "If equal to 0, the SEI applies to the current frame only")
  ("SEIDisplayOrientation",                           m_displayOrientationSEIAngle,                         0, "Control generation of display orientation SEI messages\n"
                                                                                                               "\tN: 0 < N < (2^16 - 1) enable display orientation SEI message with anticlockwise_rotation = N and display_orientation_repetition_period = 1\n"
                                                                                                               "\t0: disable")
  ("SEITemporalLevel0Index",                          m_temporalLevel0IndexSEIEnabled,                  false, "Control generation of temporal level 0 index SEI messages")
  ("SEIGradualDecodingRefreshInfo",                   m_gradualDecodingRefreshInfoEnabled,              false, "Control generation of gradual decoding refresh information SEI message")
  ("SEINoDisplay",                                    m_noDisplaySEITLayer,                                 0, "Control generation of no display SEI message\n"
                                                                                                               "\tN: 0 < N enable no display SEI message for temporal layer N or higher\n"
                                                                                                               "\t0: disable")
  ("SEIDecodingUnitInfo",                             m_decodingUnitInfoSEIEnabled,                     false, "Control generation of decoding unit information SEI message.")
  ("SEISOPDescription",                               m_SOPDescriptionSEIEnabled,                       false, "Control generation of SOP description SEI messages")
  ("SEIScalableNesting",                              m_scalableNestingSEIEnabled,                      false, "Control generation of scalable nesting SEI messages")
  ("SEITempMotionConstrainedTileSets",                m_tmctsSEIEnabled,                                false, "Control generation of temporal motion constrained tile sets SEI message")
  ("SEITimeCodeEnabled",                              m_timeCodeSEIEnabled,                             false, "Control generation of time code information SEI message")
  ("SEITimeCodeNumClockTs",                           m_timeCodeSEINumTs,                                   0, "Number of clock time sets [0..3]")
  ("SEITimeCodeTimeStampFlag",                        cfg_timeCodeSeiTimeStampFlag,          cfg_timeCodeSeiTimeStampFlag,         "Time stamp flag associated to each time set")
  ("SEITimeCodeFieldBasedFlag",                       cfg_timeCodeSeiNumUnitFieldBasedFlag,  cfg_timeCodeSeiNumUnitFieldBasedFlag, "Field based flag associated to each time set")
  ("SEITimeCodeCountingType",                         cfg_timeCodeSeiCountingType,           cfg_timeCodeSeiCountingType,          "Counting type associated to each time set")
  ("SEITimeCodeFullTsFlag",                           cfg_timeCodeSeiFullTimeStampFlag,      cfg_timeCodeSeiFullTimeStampFlag,     "Full time stamp flag associated to each time set")
  ("SEITimeCodeDiscontinuityFlag",                    cfg_timeCodeSeiDiscontinuityFlag,      cfg_timeCodeSeiDiscontinuityFlag,     "Discontinuity flag associated to each time set")
  ("SEITimeCodeCntDroppedFlag",                       cfg_timeCodeSeiCntDroppedFlag,         cfg_timeCodeSeiCntDroppedFlag,        "Counter dropped flag associated to each time set")
  ("SEITimeCodeNumFrames",                            cfg_timeCodeSeiNumberOfFrames,         cfg_timeCodeSeiNumberOfFrames,        "Number of frames associated to each time set")
  ("SEITimeCodeSecondsValue",                         cfg_timeCodeSeiSecondsValue,           cfg_timeCodeSeiSecondsValue,          "Seconds value for each time set")
  ("SEITimeCodeMinutesValue",                         cfg_timeCodeSeiMinutesValue,           cfg_timeCodeSeiMinutesValue,          "Minutes value for each time set")
  ("SEITimeCodeHoursValue",                           cfg_timeCodeSeiHoursValue,             cfg_timeCodeSeiHoursValue,            "Hours value for each time set")
  ("SEITimeCodeSecondsFlag",                          cfg_timeCodeSeiSecondsFlag,            cfg_timeCodeSeiSecondsFlag,           "Flag to signal seconds value presence in each time set")
  ("SEITimeCodeMinutesFlag",                          cfg_timeCodeSeiMinutesFlag,            cfg_timeCodeSeiMinutesFlag,           "Flag to signal minutes value presence in each time set")
  ("SEITimeCodeHoursFlag",                            cfg_timeCodeSeiHoursFlag,              cfg_timeCodeSeiHoursFlag,             "Flag to signal hours value presence in each time set")
  ("SEITimeCodeOffsetLength",                         cfg_timeCodeSeiTimeOffsetLength,       cfg_timeCodeSeiTimeOffsetLength,      "Time offset length associated to each time set")
  ("SEITimeCodeTimeOffset",                           cfg_timeCodeSeiTimeOffsetValue,        cfg_timeCodeSeiTimeOffsetValue,       "Time offset associated to each time set")
  ("SEIKneeFunctionInfo",                             m_kneeSEIEnabled,                                 false, "Control generation of Knee function SEI messages")
  ("SEIKneeFunctionId",                               m_kneeSEIId,                                          0, "Specifies Id of Knee function SEI message for a given session")
  ("SEIKneeFunctionCancelFlag",                       m_kneeSEICancelFlag,                              false, "Indicates that Knee function SEI message cancels the persistence or follows")
  ("SEIKneeFunctionPersistenceFlag",                  m_kneeSEIPersistenceFlag,                          true, "Specifies the persistence of the Knee function SEI message")
  ("SEIKneeFunctionInputDrange",                      m_kneeSEIInputDrange,                              1000, "Specifies the peak luminance level for the input picture of Knee function SEI messages")
  ("SEIKneeFunctionInputDispLuminance",               m_kneeSEIInputDispLuminance,                        100, "Specifies the expected display brightness for the input picture of Knee function SEI messages")
  ("SEIKneeFunctionOutputDrange",                     m_kneeSEIOutputDrange,                             4000, "Specifies the peak luminance level for the output picture of Knee function SEI messages")
  ("SEIKneeFunctionOutputDispLuminance",              m_kneeSEIOutputDispLuminance,                       800, "Specifies the expected display brightness for the output picture of Knee function SEI messages")
  ("SEIKneeFunctionNumKneePointsMinus1",              m_kneeSEINumKneePointsMinus1,                         2, "Specifies the number of knee points - 1")
  ("SEIKneeFunctionInputKneePointValue",              cfg_kneeSEIInputKneePointValue,   cfg_kneeSEIInputKneePointValue, "Array of input knee point")
  ("SEIKneeFunctionOutputKneePointValue",             cfg_kneeSEIOutputKneePointValue, cfg_kneeSEIOutputKneePointValue, "Array of output knee point")
  ("SEIMasteringDisplayColourVolume",                 m_masteringDisplay.colourVolumeSEIEnabled,         false, "Control generation of mastering display colour volume SEI messages")
  ("SEIMasteringDisplayMaxLuminance",                 m_masteringDisplay.maxLuminance,                  10000u, "Specifies the mastering display maximum luminance value in units of 1/10000 candela per square metre (32-bit code value)")
  ("SEIMasteringDisplayMinLuminance",                 m_masteringDisplay.minLuminance,                      0u, "Specifies the mastering display minimum luminance value in units of 1/10000 candela per square metre (32-bit code value)")
  ("SEIMasteringDisplayPrimaries",                    cfg_DisplayPrimariesCode,       cfg_DisplayPrimariesCode, "Mastering display primaries for all three colour planes in CIE xy coordinates in increments of 1/50000 (results in the ranges 0 to 50000 inclusive)")
  ("SEIMasteringDisplayWhitePoint",                   cfg_DisplayWhitePointCode,     cfg_DisplayWhitePointCode, "Mastering display white point CIE xy coordinates in normalised increments of 1/50000 (e.g. 0.333 = 16667)")
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  ("SEIPreferredTransferCharacterisics",              m_preferredTransferCharacteristics,                   -1, "Value for the preferred_transfer_characteristics field of the Alternative transfer characteristics SEI which will override the corresponding entry in the VUI. If negative, do not produce the respective SEI message")
#endif
  ("SEIGreenMetadataType",                            m_greenMetadataType,                                  0u, "Value for the green_metadata_type specifies the type of metadata that is present in the SEI message. If green_metadata_type is 1, then metadata enabling quality recovery after low-power encoding is present")
  ("SEIXSDMetricType",                                m_xsdMetricType,                                      0u, "Value for the xsd_metric_type indicates the type of the objective quality metric. PSNR is the only type currently supported")
  ("MCTSEncConstraint",                               m_MCTSEncConstraint,                               false, "For MCTS, constrain motion vectors at tile boundaries")
#if ENABLE_TRACING
  ("TraceChannelsList",                               bTracingChannelsList,                              false, "List all available tracing channels")
  ("TraceRule",                                       sTracingRule,                               string( "" ), "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
  ("TraceFile",                                       sTracingFile,                               string( "" ), "Tracing file")
#endif

  ("DebugBitstream",                                  m_decodeBitstreams[0],             string( "" ), "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DebugPOC",                                        m_switchPOC,                                 -1, "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("DecodeBitstream1",                                m_decodeBitstreams[0],             string( "" ), "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream2",                                m_decodeBitstreams[1],             string( "" ), "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("SwitchPOC",                                       m_switchPOC,                                 -1, "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchDQP",                                       m_switchDQP,                                  0, "delta QP applied to picture with switchPOC and subsequent pictures." )
  ("FastForwardToPOC",                                m_fastForwardToPOC,                          -1, "Get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC." )
  ("StopAfterFFtoPOC",                                m_stopAfterFFtoPOC,                       false, "If using fast forward to POC, after the POC of interest has been hit, stop further encoding.")
  ("ForceDecodeBitstream1",                           m_forceDecodeBitstream1,                  false, "force decoding of bitstream 1 - use this only if you are realy sure about what you are doing ")
  ("DecodeBitstream2ModPOCAndType",                   m_bs2ModPOCAndType,                       false, "Modify POC and NALU-type of second input bitstream, to use second BS as closing I-slice")
  ("NumSplitThreads",                                 m_numSplitThreads,                            1, "Number of threads used to parallelize splitting")
  ("ForceSingleSplitThread",                          m_forceSplitSequential,                   false, "Force single thread execution even if taking the parallelized path")
  ("NumWppThreads",                                   m_numWppThreads,                              1, "Number of threads used to run WPP-style parallelization")
  ("NumWppExtraLines",                                m_numWppExtraLines,                           0, "Number of additional wpp lines to switch when threads are blocked")
  ("DebugCTU",                                        m_debugCTU,                                  -1, "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC-frame at CTUline containin debug CTU.")
#if ENABLE_WPP_PARALLELISM
  ("EnsureWppBitEqual",                               m_ensureWppBitEqual,                       true, "Ensure the results are equal to results with WPP-style parallelism, even if WPP is off")
#else
  ("EnsureWppBitEqual",                               m_ensureWppBitEqual,                      false, "Ensure the results are equal to results with WPP-style parallelism, even if WPP is off")
#endif
  ( "ALF",                                             m_alf,                                    true, "Adpative Loop Filter\n" )
#if JVET_O1164_RPR
  ( "ScalingRatioHor",                                m_scalingRatioHor,                          1.0, "Scaling ratio in hor direction" )
  ( "ScalingRatioVer",                                m_scalingRatioVer,                          1.0, "Scaling ratio in ver direction" )
  ( "FractionNumFrames",                              m_fractionOfFrames,                         1.0, "Encode a fraction of the specified in FramesToBeEncoded frames" )
  ( "SwitchPocPeriod",                                m_switchPocPeriod,                            0, "Switch POC period for RPR" )
  ( "UpscaledOutput",                                 m_upscaledOutput,                             0, "Output upscaled (2), decoded but in full resolution buffer (1) or decoded cropped (0, default) picture for RPR" )
#endif
    ;

#if EXTENSION_360_VIDEO
  TExt360AppEncCfg::TExt360AppEncCfgContext ext360CfgContext;
  m_ext360.addOptions(opts, ext360CfgContext);
#endif

  for(int i=1; i<MAX_GOP+1; i++)
  {
    std::ostringstream cOSS;
    cOSS<<"Frame"<<i;
    opts.addOptions()(cOSS.str(), m_GOPList[i-1], GOPEntry());
  }

  for(int i=1; i<MAX_TILES+1; i++)
  {
    std::ostringstream cOSS;
    cOSS<<"BrickSplit"<<i;
    opts.addOptions()(cOSS.str(), m_brickSplits[i-1], BrickSplit());
  }

  po::setDefaults(opts);
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled = po::scanArgv(opts, argc, (const char**) argv, err);
   
#if JVET_O1164_RPR
  m_rprEnabled = m_scalingRatioHor != 1.0 || m_scalingRatioVer != 1.0;
  if( m_fractionOfFrames != 1.0 )
  {
    m_framesToBeEncoded = int( m_framesToBeEncoded * m_fractionOfFrames );
  }

  if( m_rprEnabled && !m_switchPocPeriod )
  {
    m_switchPocPeriod = m_iFrameRate / 2 / m_iGOPSize * m_iGOPSize;
  }
#endif

  for (int i = 0; m_GOPList[i].m_POC != -1 && i < MAX_GOP + 1; i++)
  {
    m_RPLList0[i].m_POC = m_RPLList1[i].m_POC = m_GOPList[i].m_POC;
    m_RPLList0[i].m_temporalId = m_RPLList1[i].m_temporalId = m_GOPList[i].m_temporalId;
    m_RPLList0[i].m_refPic = m_RPLList1[i].m_refPic = m_GOPList[i].m_refPic;
    m_RPLList0[i].m_sliceType = m_RPLList1[i].m_sliceType = m_GOPList[i].m_sliceType;
    m_RPLList0[i].m_isEncoded = m_RPLList1[i].m_isEncoded = m_GOPList[i].m_isEncoded;

    m_RPLList0[i].m_numRefPicsActive = m_GOPList[i].m_numRefPicsActive0;
    m_RPLList1[i].m_numRefPicsActive = m_GOPList[i].m_numRefPicsActive1;
    m_RPLList0[i].m_numRefPics = m_GOPList[i].m_numRefPics0;
    m_RPLList1[i].m_numRefPics = m_GOPList[i].m_numRefPics1;
    for (int j = 0; j < m_GOPList[i].m_numRefPics0; j++)
      m_RPLList0[i].m_deltaRefPics[j] = m_GOPList[i].m_deltaRefPics0[j];
    for (int j = 0; j < m_GOPList[i].m_numRefPics1; j++)
      m_RPLList1[i].m_deltaRefPics[j] = m_GOPList[i].m_deltaRefPics1[j];
  }

  if (m_compositeRefEnabled)
  {
    for (int i = 0; i < m_iGOPSize; i++)
    {
      m_GOPList[i].m_POC *= 2;
      m_RPLList0[i].m_POC *= 2;
      m_RPLList1[i].m_POC *= 2;
      for (int j = 0; j < m_RPLList0[i].m_numRefPics; j++)
      {
        m_RPLList0[i].m_deltaRefPics[j] *= 2;
      }
      for (int j = 0; j < m_RPLList1[i].m_numRefPics; j++)
      {
        m_RPLList1[i].m_deltaRefPics[j] *= 2;
      }
    }
  }

  for (list<const char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++)
  {
    msg( ERROR, "Unhandled argument ignored: `%s'\n", *it);
  }

  if (argc == 1 || do_help)
  {
    /* argc == 1: no options have been specified */
    po::doHelp(cout, opts);
    return false;
  }

  if (err.is_errored)
  {
    if (!warnUnknowParameter)
    {
      /* error report has already been printed on stderr */
      return false;
    }
  }

  g_verbosity = MsgLevel( m_verbosity );


  /*
   * Set any derived parameters
   */
#if EXTENSION_360_VIDEO
  m_inputFileWidth = m_iSourceWidth;
  m_inputFileHeight = m_iSourceHeight;
  m_ext360.setMaxCUInfo(m_uiCTUSize, 1 << MIN_CU_LOG2);
#endif

  if (!inputPathPrefix.empty() && inputPathPrefix.back() != '/' && inputPathPrefix.back() != '\\' )
  {
    inputPathPrefix += "/";
  }
  m_inputFileName   = inputPathPrefix + m_inputFileName;
  m_framesToBeEncoded = ( m_framesToBeEncoded + m_temporalSubsampleRatio - 1 ) / m_temporalSubsampleRatio;
  m_adIntraLambdaModifier = cfg_adIntraLambdaModifier.values;
  if(m_isField)
  {
    //Frame height
    m_iSourceHeightOrg = m_iSourceHeight;
    //Field height
    m_iSourceHeight = m_iSourceHeight >> 1;
    //number of fields to encode
    m_framesToBeEncoded *= 2;
  }

  if( !m_tileUniformSpacingFlag && m_numTileColumnsMinus1 > 0 )
  {
    if (cfg_ColumnWidth.values.size() > m_numTileColumnsMinus1)
    {
      EXIT( "Error: The number of columns whose width are defined is larger than the allowed number of columns." );
    }
    else if (cfg_ColumnWidth.values.size() < m_numTileColumnsMinus1)
    {
      EXIT( "Error: The width of some columns is not defined." );
    }
    else
    {
      m_tileColumnWidth.resize(m_numTileColumnsMinus1);
      for(uint32_t i=0; i<cfg_ColumnWidth.values.size(); i++)
      {
        m_tileColumnWidth[i]=cfg_ColumnWidth.values[i];
      }
    }
  }
  else
  {
    m_tileColumnWidth.clear();
  }

  if( !m_tileUniformSpacingFlag && m_numTileRowsMinus1 > 0 )
  {
    if (cfg_RowHeight.values.size() > m_numTileRowsMinus1)
    {
      EXIT( "Error: The number of rows whose height are defined is larger than the allowed number of rows." );
    }
    else if (cfg_RowHeight.values.size() < m_numTileRowsMinus1)
    {
      EXIT( "Error: The height of some rows is not defined." );
    }
    else
    {
      m_tileRowHeight.resize(m_numTileRowsMinus1);
      for(uint32_t i=0; i<cfg_RowHeight.values.size(); i++)
      {
        m_tileRowHeight[i]=cfg_RowHeight.values[i];
      }
    }
  }
  else
  {
    m_tileRowHeight.clear();
  }

  /* rules for input, output and internal bitdepths as per help text */
  if (m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA  ] == 0)
  {
    m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA  ] = m_inputBitDepth      [CHANNEL_TYPE_LUMA  ];
  }
  if (m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] == 0)
  {
    m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] = m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA  ];
  }
  if (m_internalBitDepth   [CHANNEL_TYPE_LUMA  ] == 0)
  {
    m_internalBitDepth   [CHANNEL_TYPE_LUMA  ] = m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA  ];
  }
  if (m_internalBitDepth   [CHANNEL_TYPE_CHROMA] == 0)
  {
    m_internalBitDepth   [CHANNEL_TYPE_CHROMA] = m_internalBitDepth   [CHANNEL_TYPE_LUMA  ];
  }
  if (m_inputBitDepth      [CHANNEL_TYPE_CHROMA] == 0)
  {
    m_inputBitDepth      [CHANNEL_TYPE_CHROMA] = m_inputBitDepth      [CHANNEL_TYPE_LUMA  ];
  }
  if (m_outputBitDepth     [CHANNEL_TYPE_LUMA  ] == 0)
  {
    m_outputBitDepth     [CHANNEL_TYPE_LUMA  ] = m_internalBitDepth   [CHANNEL_TYPE_LUMA  ];
  }
  if (m_outputBitDepth     [CHANNEL_TYPE_CHROMA] == 0)
  {
    m_outputBitDepth     [CHANNEL_TYPE_CHROMA] = m_outputBitDepth     [CHANNEL_TYPE_LUMA  ];
  }

  m_InputChromaFormatIDC = numberToChromaFormat(tmpInputChromaFormat);
  m_chromaFormatIDC      = ((tmpChromaFormat == 0) ? (m_InputChromaFormatIDC) : (numberToChromaFormat(tmpChromaFormat)));
#if EXTENSION_360_VIDEO
  m_ext360.processOptions(ext360CfgContext);
#endif

  CHECK( !( tmpWeightedPredictionMethod >= 0 && tmpWeightedPredictionMethod <= WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING_AND_EXTENSION ), "Error in cfg" );
  m_weightedPredictionMethod = WeightedPredictionMethod(tmpWeightedPredictionMethod);

  CHECK( tmpFastInterSearchMode<0 || tmpFastInterSearchMode>FASTINTERSEARCH_MODE3, "Error in cfg" );
  m_fastInterSearchMode = FastInterSearchMode(tmpFastInterSearchMode);

  CHECK( tmpMotionEstimationSearchMethod < 0 || tmpMotionEstimationSearchMethod >= MESEARCH_NUMBER_OF_METHODS, "Error in cfg" );
  m_motionEstimationSearchMethod=MESearchMethod(tmpMotionEstimationSearchMethod);

  if (extendedProfile >= 1000 && extendedProfile <= 12316)
  {
    m_profile = Profile::MAINREXT;
    if (m_bitDepthConstraint != 0 || tmpConstraintChromaFormat != 0)
    {
      EXIT( "Error: The bit depth and chroma format constraints are not used when an explicit RExt profile is specified");
    }
    m_bitDepthConstraint           = (extendedProfile%100);
    m_intraConstraintFlag          = ((extendedProfile%10000)>=2000);
    m_onePictureOnlyConstraintFlag = (extendedProfile >= 10000);
    switch ((extendedProfile/100)%10)
    {
      case 0:  tmpConstraintChromaFormat=400; break;
      case 1:  tmpConstraintChromaFormat=420; break;
      case 2:  tmpConstraintChromaFormat=422; break;
      default: tmpConstraintChromaFormat=444; break;
    }
  }
  else
  {
    m_profile = Profile::Name(extendedProfile);
  }

  if (m_profile == Profile::HIGHTHROUGHPUTREXT )
  {
    if (m_bitDepthConstraint == 0)
    {
      m_bitDepthConstraint = 16;
    }
    m_chromaFormatConstraint = (tmpConstraintChromaFormat == 0) ? CHROMA_444 : numberToChromaFormat(tmpConstraintChromaFormat);
  }
  else if (m_profile == Profile::MAINREXT)
  {
    if (m_bitDepthConstraint == 0 && tmpConstraintChromaFormat == 0)
    {
      // produce a valid combination, if possible.
      const bool bUsingGeneralRExtTools  = m_transformSkipRotationEnabledFlag        ||
                                           m_transformSkipContextEnabledFlag         ||
                                           m_rdpcmEnabledFlag[RDPCM_SIGNAL_IMPLICIT] ||
                                           m_rdpcmEnabledFlag[RDPCM_SIGNAL_EXPLICIT] ||
                                           !m_enableIntraReferenceSmoothing          ||
                                           m_persistentRiceAdaptationEnabledFlag     ||
                                           m_log2MaxTransformSkipBlockSize!=2;
      const bool bUsingChromaQPAdjustment= m_cuChromaQpOffsetSubdiv >= 0;
      const bool bUsingExtendedPrecision = m_extendedPrecisionProcessingFlag;
      if (m_onePictureOnlyConstraintFlag)
      {
        m_chromaFormatConstraint = CHROMA_444;
        if (m_intraConstraintFlag != true)
        {
          EXIT( "Error: Intra constraint flag must be true when one_picture_only_constraint_flag is true");
        }
        const int maxBitDepth = m_chromaFormatIDC==CHROMA_400 ? m_internalBitDepth[CHANNEL_TYPE_LUMA] : std::max(m_internalBitDepth[CHANNEL_TYPE_LUMA], m_internalBitDepth[CHANNEL_TYPE_CHROMA]);
        m_bitDepthConstraint = maxBitDepth>8 ? 16:8;
      }
      else
      {
        m_chromaFormatConstraint = NUM_CHROMA_FORMAT;
        automaticallySelectRExtProfile(bUsingGeneralRExtTools,
                                       bUsingChromaQPAdjustment,
                                       bUsingExtendedPrecision,
                                       m_intraConstraintFlag,
                                       m_bitDepthConstraint,
                                       m_chromaFormatConstraint,
                                       m_chromaFormatIDC==CHROMA_400 ? m_internalBitDepth[CHANNEL_TYPE_LUMA] : std::max(m_internalBitDepth[CHANNEL_TYPE_LUMA], m_internalBitDepth[CHANNEL_TYPE_CHROMA]),
                                       m_chromaFormatIDC);
      }
    }
    else if (m_bitDepthConstraint == 0 || tmpConstraintChromaFormat == 0)
    {
      EXIT( "Error: The bit depth and chroma format constraints must either both be specified or both be configured automatically");
    }
    else
    {
      m_chromaFormatConstraint = numberToChromaFormat(tmpConstraintChromaFormat);
    }
  }
  else
  {
    m_chromaFormatConstraint = (tmpConstraintChromaFormat == 0) ? m_chromaFormatIDC : numberToChromaFormat(tmpConstraintChromaFormat);
    m_bitDepthConstraint     = ( ( m_profile == Profile::MAIN10 || m_profile == Profile::NEXT ) ? 10 : 8 );
  }


  m_inputColourSpaceConvert = stringToInputColourSpaceConvert(inputColourSpaceConvert, true);

  switch (m_conformanceWindowMode)
  {
  case 0:
    {
      // no conformance or padding
      m_confWinLeft = m_confWinRight = m_confWinTop = m_confWinBottom = 0;
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  case 1:
    {
      // automatic padding to minimum CU size
      int minCuSize = m_uiMaxCUHeight >> (m_uiMaxCUDepth - 1);
      if (m_iSourceWidth % minCuSize)
      {
        m_aiPad[0] = m_confWinRight  = ((m_iSourceWidth / minCuSize) + 1) * minCuSize - m_iSourceWidth;
        m_iSourceWidth  += m_confWinRight;
      }
      if (m_iSourceHeight % minCuSize)
      {
        m_aiPad[1] = m_confWinBottom = ((m_iSourceHeight / minCuSize) + 1) * minCuSize - m_iSourceHeight;
        m_iSourceHeight += m_confWinBottom;
        if ( m_isField )
        {
          m_iSourceHeightOrg += m_confWinBottom << 1;
          m_aiPad[1] = m_confWinBottom << 1;
        }
      }
      if (m_aiPad[0] % SPS::getWinUnitX(m_chromaFormatIDC) != 0)
      {
        EXIT( "Error: picture width is not an integer multiple of the specified chroma subsampling");
      }
      if (m_aiPad[1] % SPS::getWinUnitY(m_chromaFormatIDC) != 0)
      {
        EXIT( "Error: picture height is not an integer multiple of the specified chroma subsampling");
      }
      break;
    }
  case 2:
    {
      //padding
      m_iSourceWidth  += m_aiPad[0];
      m_iSourceHeight += m_aiPad[1];
      m_confWinRight  = m_aiPad[0];
      m_confWinBottom = m_aiPad[1];
      break;
    }
  case 3:
    {
      // conformance
      if ((m_confWinLeft == 0) && (m_confWinRight == 0) && (m_confWinTop == 0) && (m_confWinBottom == 0))
      {
        msg( ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((m_aiPad[1] != 0) || (m_aiPad[0]!=0))
      {
        msg( ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  }

  if (tmpSliceMode<0 || tmpSliceMode>=int(NUMBER_OF_SLICE_CONSTRAINT_MODES))
  {
    EXIT( "Error: bad slice mode");
  }
  m_sliceMode = SliceConstraint(tmpSliceMode);
  if (m_sliceMode==FIXED_NUMBER_OF_CTU || m_sliceMode==FIXED_NUMBER_OF_BYTES)
  {
    // note: slice mode 2 can be re-enabled using scan order tiles
    EXIT( "Error: slice mode 1 (fixed number of CTUs) and 2 (fixed number of bytes) are no longer supported");
  }



  m_topLeftBrickIdx.clear();
  m_bottomRightBrickIdx.clear();
  m_sliceId.clear();

  bool singleTileInPicFlag = (m_numTileRowsMinus1 == 0 && m_numTileColumnsMinus1 == 0);

  if (!singleTileInPicFlag)
  {
    //if (!m_singleBrickPerSliceFlag && m_rectSliceFlag)
    if (m_sliceMode != 0 && m_sliceMode != 4 && m_rectSliceFlag)
    {
      int numSlicesInPic = m_numSlicesInPicMinus1 + 1;

      if (cfg_SliceIdx.values.size() > numSlicesInPic * 2)
      {
        EXIT("Error: The number of slice indices (RectSlicesBoundaryInPic) is greater than the NumSlicesInPicMinus1.");
      }
      else if (cfg_SliceIdx.values.size() < numSlicesInPic * 2)
      {
        EXIT("Error: The number of slice indices (RectSlicesBoundaryInPic) is less than the NumSlicesInPicMinus1.");
      }
      else
      {
        m_topLeftBrickIdx.resize(numSlicesInPic);
        m_bottomRightBrickIdx.resize(numSlicesInPic);
        for (uint32_t i = 0; i < numSlicesInPic; ++i)
        {
          m_topLeftBrickIdx[i] = cfg_SliceIdx.values[i * 2];
          m_bottomRightBrickIdx[i] = cfg_SliceIdx.values[i * 2 + 1];
        }
        //Validating the correctness of rectangular slice structure
        int **brickToSlice = (int **)malloc(sizeof(int *) * (m_numTileRowsMinus1 + 1));
        for (int i = 0; i <= m_numTileRowsMinus1; i++)
        {
          brickToSlice[i] = (int *)malloc(sizeof(int) * (m_numTileColumnsMinus1 + 1));
          memset(brickToSlice[i], -1, sizeof(int) * ((m_numTileColumnsMinus1 + 1)));
        }

        //Check overlap case
        for (int sliceIdx = 0; sliceIdx < numSlicesInPic; sliceIdx++)
        {
          int sliceStartRow = m_topLeftBrickIdx[sliceIdx] / (m_numTileColumnsMinus1 + 1);
          int sliceEndRow   = m_bottomRightBrickIdx[sliceIdx] / (m_numTileColumnsMinus1 + 1);
          int sliceStartCol = m_topLeftBrickIdx[sliceIdx] % (m_numTileColumnsMinus1 + 1);
          int sliceEndCol   = m_bottomRightBrickIdx[sliceIdx] % (m_numTileColumnsMinus1 + 1);
          for (int i = 0; i <= m_numTileRowsMinus1; i++)
          {
            for (int j = 0; j <= m_numTileColumnsMinus1; j++)
            {
              if (i >= sliceStartRow && i <= sliceEndRow && j >= sliceStartCol && j <= sliceEndCol)
              {
                if (brickToSlice[i][j] != -1)
                {
                  msg(ERROR, "Error: Values given in RectSlicesBoundaryInPic have conflict! Rectangular slice shall not have overlapped tile(s)\n");
                  EXIT(1);
                }
                else
                {
                  brickToSlice[i][j] = sliceIdx;
                }
              }
            }
          }
          //Check violation to number of tiles per slice
          if (m_sliceMode == 3 && m_rectSliceFlag)
          {
            if ((sliceEndRow - sliceStartRow + 1) * (sliceEndCol - sliceStartCol + 1) > m_sliceArgument)
            {
              EXIT("Error: One or more slices contain more tiles than the defined number of tiles per slice");
            }
            if ((sliceEndRow - sliceStartRow + 1) * (sliceEndCol - sliceStartCol + 1) < m_sliceArgument)
            {
              //Allow less number of tiles only when the rectangular slice is at the right most or bottom most of the picture
              if (sliceEndRow != m_numTileRowsMinus1 || sliceEndCol != m_numTileColumnsMinus1)
              {
                EXIT("Error: One or more slices that is not at the picture boundary contain less tiles than the defined number of tiles per slice");
              }
            }
          }
        }
        //Check gap case
        for (int i = 0; i <= m_numTileRowsMinus1; i++)
        {
          for (int j = 0; j <= m_numTileColumnsMinus1; j++)
          {
            if (brickToSlice[i][j] == -1)
            {
              EXIT("Error: Values given in RectSlicesBoundaryInPic have conflict! Rectangular slice shall not have gap");
            }
          }
        }

        for (int i = 0; i <= m_numTileRowsMinus1; i++)
        {
          free(brickToSlice[i]);
          brickToSlice[i] = 0;
        }
        free(brickToSlice);
        brickToSlice = 0;
      }
    }      // (!m_singleBrickPerSliceFlag && m_rectSliceFlag)
  }        // !singleTileInPicFlag

  if (m_rectSliceFlag && m_signalledSliceIdFlag)
  {
    int numSlicesInPic = m_numSlicesInPicMinus1 + 1;

    if (cfg_SignalledSliceId.values.size() > numSlicesInPic)
    {
      EXIT("Error: The number of Slice Ids are greater than the m_signalledTileGroupIdLengthMinus1.");
    }
    else if (cfg_SignalledSliceId.values.size() < numSlicesInPic)
    {
      EXIT("Error: The number of Slice Ids are less than the m_signalledTileGroupIdLengthMinus1.");
    }
    else
    {
      m_sliceId.resize(numSlicesInPic);
      for (uint32_t i = 0; i < cfg_SignalledSliceId.values.size(); ++i)
      {
        m_sliceId[i] = cfg_SignalledSliceId.values[i];
      }
    }
  }
  else if (m_rectSliceFlag)
  {
    int numSlicesInPic = m_numSlicesInPicMinus1 + 1;
    m_sliceId.resize(numSlicesInPic);
    for (uint32_t i = 0; i < numSlicesInPic; ++i)
    {
      m_sliceId[i] = i;
    }
  }

  if (tmpDecodedPictureHashSEIMappedType<0 || tmpDecodedPictureHashSEIMappedType>=int(NUMBER_OF_HASHTYPES))
  {
    EXIT( "Error: bad checksum mode");
  }
  // Need to map values to match those of the SEI message:
  if (tmpDecodedPictureHashSEIMappedType==0)
  {
    m_decodedPictureHashSEIType=HASHTYPE_NONE;
  }
  else
  {
    m_decodedPictureHashSEIType=HashType(tmpDecodedPictureHashSEIMappedType-1);
  }

  // allocate slice-based dQP values
  m_aidQP = new int[ m_framesToBeEncoded + m_iGOPSize + 1 ];
  ::memset( m_aidQP, 0, sizeof(int)*( m_framesToBeEncoded + m_iGOPSize + 1 ) );

#if QP_SWITCHING_FOR_PARALLEL
  if (m_qpIncrementAtSourceFrame.bPresent)
  {
    uint32_t switchingPOC = 0;
    if (m_qpIncrementAtSourceFrame.value > m_FrameSkip)
    {
      // if switch source frame (ssf) = 10, and frame skip (fs)=2 and temporal subsample ratio (tsr) =1, then
      //    for this simulation switch at POC 8 (=10-2).
      // if ssf=10, fs=2, tsr=2, then for this simulation, switch at POC 4 (=(10-2)/2): POC0=Src2, POC1=Src4, POC2=Src6, POC3=Src8, POC4=Src10
      switchingPOC = (m_qpIncrementAtSourceFrame.value - m_FrameSkip) / m_temporalSubsampleRatio;
    }
    for (uint32_t i = switchingPOC; i<(m_framesToBeEncoded + m_iGOPSize + 1); i++)
    {
      m_aidQP[i] = 1;
    }
  }
#else
  // handling of floating-point QP values
  // if QP is not integer, sequence is split into two sections having QP and QP+1
  m_iQP = (int)( m_fQP );
  if ( m_iQP < m_fQP )
  {
    int iSwitchPOC = (int)( m_framesToBeEncoded - (m_fQP - m_iQP)*m_framesToBeEncoded + 0.5 );

    iSwitchPOC = (int)( (double)iSwitchPOC / m_iGOPSize + 0.5 )*m_iGOPSize;
    for ( int i=iSwitchPOC; i<m_framesToBeEncoded + m_iGOPSize + 1; i++ )
    {
      m_aidQP[i] = 1;
    }
  }
#endif

  for(uint32_t ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (saoOffsetBitShift[ch]<0)
    {
      if (m_internalBitDepth[ch]>10)
      {
        m_log2SaoOffsetScale[ch]=uint32_t(Clip3<int>(0, m_internalBitDepth[ch]-10, int(m_internalBitDepth[ch]-10 + 0.165*m_iQP - 3.22 + 0.5) ) );
      }
      else
      {
        m_log2SaoOffsetScale[ch]=0;
      }
    }
    else
    {
      m_log2SaoOffsetScale[ch]=uint32_t(saoOffsetBitShift[ch]);
    }
  }

#if SHARP_LUMA_DELTA_QP
  CHECK( lumaLevelToDeltaQPMode >= LUMALVL_TO_DQP_NUM_MODES, "Error in cfg" );

  m_lumaLevelToDeltaQPMapping.mode=LumaLevelToDQPMode(lumaLevelToDeltaQPMode);

  if (m_lumaLevelToDeltaQPMapping.mode)
  {
    CHECK(  cfg_lumaLeveltoDQPMappingLuma.values.size() != cfg_lumaLeveltoDQPMappingQP.values.size(), "Error in cfg" );
    m_lumaLevelToDeltaQPMapping.mapping.resize(cfg_lumaLeveltoDQPMappingLuma.values.size());
    for(uint32_t i=0; i<cfg_lumaLeveltoDQPMappingLuma.values.size(); i++)
    {
      m_lumaLevelToDeltaQPMapping.mapping[i]=std::pair<int,int>(cfg_lumaLeveltoDQPMappingLuma.values[i], cfg_lumaLeveltoDQPMappingQP.values[i]);
    }
  }
#endif
#if JVET_O0650_SIGNAL_CHROMAQP_MAPPING_TABLE

  CHECK(cfg_qpInValCb.values.size() != cfg_qpOutValCb.values.size(), "Chroma QP table for Cb is incomplete.");
  CHECK(cfg_qpInValCr.values.size() != cfg_qpOutValCr.values.size(), "Chroma QP table for Cr is incomplete.");
  CHECK(cfg_qpInValCbCr.values.size() != cfg_qpOutValCbCr.values.size(), "Chroma QP table for CbCr is incomplete.");
  if (m_useIdentityTableForNon420Chroma && m_chromaFormatIDC != CHROMA_420)
  {
    m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = true;
    cfg_qpInValCb.values = { 0 };
    cfg_qpInValCr.values = { 0 };
    cfg_qpInValCbCr.values = { 0 };
    cfg_qpOutValCb.values = { 0 };
    cfg_qpOutValCr.values = { 0 };
    cfg_qpOutValCbCr.values = { 0 };
  }
  m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0].resize(cfg_qpInValCb.values.size());
  m_chromaQpMappingTableParams.m_deltaQpOutVal[0].resize(cfg_qpOutValCb.values.size());
  m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[0] = (int)cfg_qpOutValCb.values.size()-1;
  int qpBdOffsetC = 6 * (m_internalBitDepth[CHANNEL_TYPE_CHROMA] - 8);
  for (int i = 0; i < cfg_qpInValCb.values.size(); i++)
  {
    CHECK(cfg_qpInValCb.values[i] < -qpBdOffsetC || cfg_qpInValCb.values[i] > MAX_QP, "Some entries cfg_qpInValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    CHECK(cfg_qpOutValCb.values[i] < -qpBdOffsetC || cfg_qpOutValCb.values[i] > MAX_QP, "Some entries cfg_qpOutValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0][i] = (i == 0) ? cfg_qpInValCb.values[i] + qpBdOffsetC : cfg_qpInValCb.values[i] - cfg_qpInValCb.values[i - 1] - 1;
    m_chromaQpMappingTableParams.m_deltaQpOutVal[0][i] = (i == 0) ? cfg_qpOutValCb.values[i] + qpBdOffsetC : cfg_qpOutValCb.values[i] - cfg_qpOutValCb.values[i - 1];
  }
  if (!m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag)
  {
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1].resize(cfg_qpInValCr.values.size());
    m_chromaQpMappingTableParams.m_deltaQpOutVal[1].resize(cfg_qpOutValCr.values.size());
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[1] = (int)cfg_qpOutValCr.values.size()-1;
    for (int i = 0; i < cfg_qpInValCr.values.size(); i++)
    {
      CHECK(cfg_qpInValCr.values[i] < -qpBdOffsetC || cfg_qpInValCr.values[i] > MAX_QP, "Some entries cfg_qpInValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      CHECK(cfg_qpOutValCr.values[i] < -qpBdOffsetC || cfg_qpOutValCr.values[i] > MAX_QP, "Some entries cfg_qpOutValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1][i] = (i == 0) ? cfg_qpInValCr.values[i] + qpBdOffsetC : cfg_qpInValCr.values[i] - cfg_qpInValCr.values[i - 1] - 1;
      m_chromaQpMappingTableParams.m_deltaQpOutVal[1][i] = (i == 0) ? cfg_qpOutValCr.values[i] + qpBdOffsetC : cfg_qpOutValCr.values[i] - cfg_qpOutValCr.values[i - 1];
    }
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2].resize(cfg_qpInValCbCr.values.size());
    m_chromaQpMappingTableParams.m_deltaQpOutVal[2].resize(cfg_qpOutValCbCr.values.size());
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[2] = (int)cfg_qpOutValCbCr.values.size()-1;
    for (int i = 0; i < cfg_qpInValCbCr.values.size(); i++)
    {
      CHECK(cfg_qpInValCbCr.values[i] < -qpBdOffsetC || cfg_qpInValCbCr.values[i] > MAX_QP, "Some entries cfg_qpInValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      CHECK(cfg_qpOutValCbCr.values[i] < -qpBdOffsetC || cfg_qpOutValCbCr.values[i] > MAX_QP, "Some entries cfg_qpOutValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2][i] = (i == 0) ? cfg_qpInValCbCr.values[i] + qpBdOffsetC : cfg_qpInValCbCr.values[i] - cfg_qpInValCbCr.values[i - 1] - 1;
      m_chromaQpMappingTableParams.m_deltaQpOutVal[2][i] = (i == 0) ? cfg_qpOutValCbCr.values[i] + qpBdOffsetC : cfg_qpOutValCbCr.values[i] - cfg_qpOutValCbCr.values[i - 1];
    }
  }
#endif

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  if ( m_LadfEnabed )
  {
    CHECK( m_LadfNumIntervals != cfg_LadfQpOffset.values.size(), "size of LadfQpOffset must be equal to LadfNumIntervals");
    CHECK( m_LadfNumIntervals - 1 != cfg_LadfIntervalLowerBound.values.size(), "size of LadfIntervalLowerBound must be equal to LadfNumIntervals - 1");
    m_LadfQpOffset = cfg_LadfQpOffset.values;
    m_LadfIntervalLowerBound[0] = 0;
    for (int k = 1; k < m_LadfNumIntervals; k++)
    {
      m_LadfIntervalLowerBound[k] = cfg_LadfIntervalLowerBound.values[k - 1];
    }
  }
#endif

#if JVET_O0610_CFG
  if (m_AccessUnitDelimiter == false)
  {
    printf ("Warning: Access unit delimiters are disabled. VVC requires the presence of access unit delimiters\n");
  }
#endif

#if JVET_O0756_CONFIG_HDRMETRICS && !JVET_O0756_CALCULATE_HDRMETRICS
  if ( m_calculateHdrMetrics == true)
  {
    printf ("Warning: Configuration enables HDR metric calculations.  However, HDR metric support was not linked when compiling the VTM.\n");
    m_calculateHdrMetrics = false;
  }
#endif

  if ( m_loopFilterAcrossVirtualBoundariesDisabledFlag )
  {
    CHECK( m_numVerVirtualBoundaries > 3, "Number of vertical virtual boundaries must be comprised between 0 and 3 included" );
    CHECK( m_numHorVirtualBoundaries > 3, "Number of horizontal virtual boundaries must be comprised between 0 and 3 included" );
    CHECK( m_numVerVirtualBoundaries != cfg_virtualBoundariesPosX.values.size(), "Size of VirtualBoundariesPosX must be equal to NumVerVirtualBoundaries");
    CHECK( m_numHorVirtualBoundaries != cfg_virtualBoundariesPosY.values.size(), "Size of VirtualBoundariesPosY must be equal to NumHorVirtualBoundaries");
    m_virtualBoundariesPosX = cfg_virtualBoundariesPosX.values;
    if (m_numVerVirtualBoundaries > 1)
    {
      sort(m_virtualBoundariesPosX.begin(), m_virtualBoundariesPosX.end());
    }
    for (unsigned i = 0; i < m_numVerVirtualBoundaries; i++)
    {
      CHECK( m_virtualBoundariesPosX[i] == 0 || m_virtualBoundariesPosX[i] >= m_iSourceWidth, "The vertical virtual boundary must be within the picture" );
      CHECK( m_virtualBoundariesPosX[i] % 8, "The vertical virtual boundary must be a multiple of 8 luma samples" );
      if (i > 0)
      {
        CHECK( m_virtualBoundariesPosX[i] - m_virtualBoundariesPosX[i-1] < m_uiCTUSize, "The distance between any two vertical virtual boundaries shall be greater than or equal to the CTU size" );
      }
    }
    m_virtualBoundariesPosY = cfg_virtualBoundariesPosY.values;
    if (m_numHorVirtualBoundaries > 1)
    {
      sort(m_virtualBoundariesPosY.begin(), m_virtualBoundariesPosY.end());
    }
    for (unsigned i = 0; i < m_numHorVirtualBoundaries; i++)
    {
      CHECK( m_virtualBoundariesPosY[i] == 0 || m_virtualBoundariesPosY[i] >= m_iSourceHeight, "The horizontal virtual boundary must be within the picture" );
      CHECK( m_virtualBoundariesPosY[i] % 8, "The horizontal virtual boundary must be a multiple of 8 luma samples" );
      if (i > 0)
      {
        CHECK( m_virtualBoundariesPosY[i] - m_virtualBoundariesPosY[i-1] < m_uiCTUSize, "The distance between any two horizontal virtual boundaries shall be greater than or equal to the CTU size" );
      }
    }
  }

#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  if ( m_alf )
  {
    CHECK( m_maxNumAlfAlternativesChroma < 1 || m_maxNumAlfAlternativesChroma > MAX_NUM_ALF_ALTERNATIVES_CHROMA, std::string("The maximum number of ALF Chroma filter alternatives must be in the range (1-") + std::to_string(MAX_NUM_ALF_ALTERNATIVES_CHROMA) + std::string (", inclusive)") );
  }

#endif
  // reading external dQP description from file
  if ( !m_dQPFileName.empty() )
  {
    FILE* fpt=fopen( m_dQPFileName.c_str(), "r" );
    if ( fpt )
    {
      int iValue;
      int iPOC = 0;
      while ( iPOC < m_framesToBeEncoded )
      {
        if ( fscanf(fpt, "%d", &iValue ) == EOF )
        {
          break;
        }
        m_aidQP[ iPOC ] = iValue;
        iPOC++;
      }
      fclose(fpt);
    }
  }

  if( m_masteringDisplay.colourVolumeSEIEnabled )
  {
    for(uint32_t idx=0; idx<6; idx++)
    {
      m_masteringDisplay.primaries[idx/2][idx%2] = uint16_t((cfg_DisplayPrimariesCode.values.size() > idx) ? cfg_DisplayPrimariesCode.values[idx] : 0);
    }
    for(uint32_t idx=0; idx<2; idx++)
    {
      m_masteringDisplay.whitePoint[idx] = uint16_t((cfg_DisplayWhitePointCode.values.size() > idx) ? cfg_DisplayWhitePointCode.values[idx] : 0);
    }
  }

  if( m_toneMappingInfoSEIEnabled && !m_toneMapCancelFlag )
  {
    if( m_toneMapModelId == 2 && !cfg_startOfCodedInterval.values.empty() )
    {
      const uint32_t num = 1u<< m_toneMapTargetBitDepth;
      m_startOfCodedInterval = new int[num];
      for(uint32_t i=0; i<num; i++)
      {
        m_startOfCodedInterval[i] = cfg_startOfCodedInterval.values.size() > i ? cfg_startOfCodedInterval.values[i] : 0;
      }
    }
    else
    {
      m_startOfCodedInterval = NULL;
    }
    if( ( m_toneMapModelId == 3 ) && ( m_numPivots > 0 ) )
    {
      if( !cfg_codedPivotValue.values.empty() && !cfg_targetPivotValue.values.empty() )
      {
        m_codedPivotValue  = new int[m_numPivots];
        m_targetPivotValue = new int[m_numPivots];
        for(uint32_t i=0; i<m_numPivots; i++)
        {
          m_codedPivotValue[i]  = cfg_codedPivotValue.values.size()  > i ? cfg_codedPivotValue.values [i] : 0;
          m_targetPivotValue[i] = cfg_targetPivotValue.values.size() > i ? cfg_targetPivotValue.values[i] : 0;
        }
      }
    }
    else
    {
      m_codedPivotValue = NULL;
      m_targetPivotValue = NULL;
    }
  }

  if( m_kneeSEIEnabled && !m_kneeSEICancelFlag )
  {
    CHECK(!( m_kneeSEINumKneePointsMinus1 >= 0 && m_kneeSEINumKneePointsMinus1 < 999 ), "Inconsistent config");
    m_kneeSEIInputKneePoint  = new int[m_kneeSEINumKneePointsMinus1+1];
    m_kneeSEIOutputKneePoint = new int[m_kneeSEINumKneePointsMinus1+1];
    for(int i=0; i<(m_kneeSEINumKneePointsMinus1+1); i++)
    {
      m_kneeSEIInputKneePoint[i]  = cfg_kneeSEIInputKneePointValue.values.size()  > i ? cfg_kneeSEIInputKneePointValue.values[i]  : 1;
      m_kneeSEIOutputKneePoint[i] = cfg_kneeSEIOutputKneePointValue.values.size() > i ? cfg_kneeSEIOutputKneePointValue.values[i] : 0;
    }
  }

  if(m_timeCodeSEIEnabled)
  {
    for(int i = 0; i < m_timeCodeSEINumTs && i < MAX_TIMECODE_SEI_SETS; i++)
    {
      m_timeSetArray[i].clockTimeStampFlag    = cfg_timeCodeSeiTimeStampFlag        .values.size()>i ? cfg_timeCodeSeiTimeStampFlag        .values [i] : false;
      m_timeSetArray[i].numUnitFieldBasedFlag = cfg_timeCodeSeiNumUnitFieldBasedFlag.values.size()>i ? cfg_timeCodeSeiNumUnitFieldBasedFlag.values [i] : 0;
      m_timeSetArray[i].countingType          = cfg_timeCodeSeiCountingType         .values.size()>i ? cfg_timeCodeSeiCountingType         .values [i] : 0;
      m_timeSetArray[i].fullTimeStampFlag     = cfg_timeCodeSeiFullTimeStampFlag    .values.size()>i ? cfg_timeCodeSeiFullTimeStampFlag    .values [i] : 0;
      m_timeSetArray[i].discontinuityFlag     = cfg_timeCodeSeiDiscontinuityFlag    .values.size()>i ? cfg_timeCodeSeiDiscontinuityFlag    .values [i] : 0;
      m_timeSetArray[i].cntDroppedFlag        = cfg_timeCodeSeiCntDroppedFlag       .values.size()>i ? cfg_timeCodeSeiCntDroppedFlag       .values [i] : 0;
      m_timeSetArray[i].numberOfFrames        = cfg_timeCodeSeiNumberOfFrames       .values.size()>i ? cfg_timeCodeSeiNumberOfFrames       .values [i] : 0;
      m_timeSetArray[i].secondsValue          = cfg_timeCodeSeiSecondsValue         .values.size()>i ? cfg_timeCodeSeiSecondsValue         .values [i] : 0;
      m_timeSetArray[i].minutesValue          = cfg_timeCodeSeiMinutesValue         .values.size()>i ? cfg_timeCodeSeiMinutesValue         .values [i] : 0;
      m_timeSetArray[i].hoursValue            = cfg_timeCodeSeiHoursValue           .values.size()>i ? cfg_timeCodeSeiHoursValue           .values [i] : 0;
      m_timeSetArray[i].secondsFlag           = cfg_timeCodeSeiSecondsFlag          .values.size()>i ? cfg_timeCodeSeiSecondsFlag          .values [i] : 0;
      m_timeSetArray[i].minutesFlag           = cfg_timeCodeSeiMinutesFlag          .values.size()>i ? cfg_timeCodeSeiMinutesFlag          .values [i] : 0;
      m_timeSetArray[i].hoursFlag             = cfg_timeCodeSeiHoursFlag            .values.size()>i ? cfg_timeCodeSeiHoursFlag            .values [i] : 0;
      m_timeSetArray[i].timeOffsetLength      = cfg_timeCodeSeiTimeOffsetLength     .values.size()>i ? cfg_timeCodeSeiTimeOffsetLength     .values [i] : 0;
      m_timeSetArray[i].timeOffsetValue       = cfg_timeCodeSeiTimeOffsetValue      .values.size()>i ? cfg_timeCodeSeiTimeOffsetValue      .values [i] : 0;
    }
  }

  m_reshapeCW.binCW.resize(3);
  m_reshapeCW.rspFps = m_iFrameRate;
#if !JVET_O0432_LMCS_ENCODER
  m_reshapeCW.rspIntraPeriod = m_iIntraPeriod;
#endif
  m_reshapeCW.rspPicSize = m_iSourceWidth*m_iSourceHeight;
  m_reshapeCW.rspFpsToIp = std::max(16, 16 * (int)(round((double)m_iFrameRate /16.0)));
  m_reshapeCW.rspBaseQP = m_iQP;
#if JVET_O0432_LMCS_ENCODER
  m_reshapeCW.updateCtrl = m_updateCtrl;
  m_reshapeCW.adpOption = m_adpOption;
  m_reshapeCW.initialCW = m_initialCW;
#endif
#if ENABLE_TRACING
  g_trace_ctx = tracing_init(sTracingFile, sTracingRule);
  if( bTracingChannelsList && g_trace_ctx )
  {
    std::string sChannelsList;
    g_trace_ctx->getChannelsList( sChannelsList );
    msg( INFO, "\n Using tracing channels:\n\n%s\n", sChannelsList.c_str() );
  }
#endif

#if ENABLE_QPA
  if (m_bUsePerceptQPA && !m_bUseAdaptiveQP && m_dualTree && (m_cbQpOffsetDualTree != 0 || m_crQpOffsetDualTree != 0))
  {
    msg( WARNING, "*************************************************************************\n" );
    msg( WARNING, "* WARNING: chroma QPA on, ignoring nonzero dual-tree chroma QP offsets! *\n" );
    msg( WARNING, "*************************************************************************\n" );
  }

#if ENABLE_QPA_SUB_CTU
 #if QP_SWITCHING_FOR_PARALLEL
  if ((m_iQP < 38) && m_bUsePerceptQPA && !m_bUseAdaptiveQP && (m_iSourceWidth <= 2048) && (m_iSourceHeight <= 1280)
 #else
  if (((int)m_fQP < 38) && m_bUsePerceptQPA && !m_bUseAdaptiveQP && (m_iSourceWidth <= 2048) && (m_iSourceHeight <= 1280)
 #endif
 #if WCG_EXT && ER_CHROMA_QP_WCG_PPS
      && (!m_wcgChromaQpControl.enabled)
 #endif
#if MAX_TB_SIZE_SIGNALLING
      && ((1 << (m_log2MaxTbSize + 1)) == m_uiCTUSize) && (m_iSourceWidth > 512 || m_iSourceHeight > 320))
#else
    && ((1 << (MAX_TB_LOG2_SIZEY + 1)) == m_uiCTUSize) && (m_iSourceWidth > 512 || m_iSourceHeight > 320))
#endif
  {
    m_cuQpDeltaSubdiv = 2;
  }
#else
 #if QP_SWITCHING_FOR_PARALLEL
  if( ( m_iQP < 38 ) && ( m_iGOPSize > 4 ) && m_bUsePerceptQPA && !m_bUseAdaptiveQP && ( m_iSourceHeight <= 1280 ) && ( m_iSourceWidth <= 2048 ) )
 #else
  if( ( ( int ) m_fQP < 38 ) && ( m_iGOPSize > 4 ) && m_bUsePerceptQPA && !m_bUseAdaptiveQP && ( m_iSourceHeight <= 1280 ) && ( m_iSourceWidth <= 2048 ) )
 #endif
  {
    msg( WARNING, "*************************************************************************\n" );
    msg( WARNING, "* WARNING: QPA on with large CTU for <=HD sequences, limiting CTU size! *\n" );
    msg( WARNING, "*************************************************************************\n" );

    m_uiCTUSize = m_uiMaxCUWidth;
#if MAX_TB_SIZE_SIGNALLING
    if( ( 1u << m_log2MaxTbSize         ) > m_uiCTUSize ) m_log2MaxTbSize--;
#endif
  }
#endif
#endif // ENABLE_QPA

  const int minCuSize = 1 << MIN_CU_LOG2;
  m_uiMaxCodingDepth = 0;
  while( ( m_uiCTUSize >> m_uiMaxCodingDepth ) > minCuSize )
  {
    m_uiMaxCodingDepth++;
  }
  m_uiLog2DiffMaxMinCodingBlockSize = m_uiMaxCodingDepth;
  m_uiMaxCUWidth = m_uiMaxCUHeight = m_uiCTUSize;
  m_uiMaxCUDepth = m_uiMaxCodingDepth;

  // check validity of input parameters
  if( xCheckParameter() )
  {
    // return check failed
    return false;
  }

  // print-out parameters
  xPrintParameter();

  return true;
}


// ====================================================================================================================
// Private member functions
// ====================================================================================================================

bool EncAppCfg::xCheckParameter()
{
  msg( NOTICE, "\n" );
  if (m_decodedPictureHashSEIType==HASHTYPE_NONE)
  {
    msg( DETAILS, "******************************************************************\n");
    msg( DETAILS, "** WARNING: --SEIDecodedPictureHash is now disabled by default. **\n");
    msg( DETAILS, "**          Automatic verification of decoded pictures by a     **\n");
    msg( DETAILS, "**          decoder requires this option to be enabled.         **\n");
    msg( DETAILS, "******************************************************************\n");
  }
  if( m_profile==Profile::NONE )
  {
    msg( DETAILS, "***************************************************************************\n");
    msg( DETAILS, "** WARNING: For conforming bitstreams a valid Profile value must be set! **\n");
    msg( DETAILS, "***************************************************************************\n");
  }
  if( m_level==Level::NONE )
  {
    msg( DETAILS, "***************************************************************************\n");
    msg( DETAILS, "** WARNING: For conforming bitstreams a valid Level value must be set!   **\n");
    msg( DETAILS, "***************************************************************************\n");
  }

  bool check_failed = false; /* abort if there is a fatal configuration problem */
#define xConfirmPara(a,b) check_failed |= confirmPara(a,b)


  if( m_profile != Profile::NEXT )
  {
    THROW( "Next profile with an alternative partitioner has to be enabled if HEVC_USE_RQT is off!" );
#if ENABLE_WPP_PARALLELISM
    xConfirmPara( m_numWppThreads > 1, "WPP-style parallelization only supported with NEXT profile" );
#endif
    xConfirmPara( m_LMChroma, "LMChroma only allowed with NEXT profile" );
    xConfirmPara( m_ImvMode, "IMV is only allowed with NEXT profile" );
#if JVET_O0119_BASE_PALETTE_444
    xConfirmPara( m_PLTMode, "PLT Mode only allowed with NEXT profile");
#endif
    xConfirmPara(m_IBCMode, "IBC Mode only allowed with NEXT profile");
    xConfirmPara( m_HashME, "Hash motion estimation only allowed with NEXT profile" );
    xConfirmPara( m_useFastLCTU, "Fast large CTU can only be applied when encoding with NEXT profile" );
    xConfirmPara( m_MTS, "MTS only allowed with NEXT profile" );
    xConfirmPara( m_MTSIntraMaxCand, "MTS only allowed with NEXT profile" );
    xConfirmPara( m_MTSInterMaxCand, "MTS only allowed with NEXT profile" );
    xConfirmPara( m_SMVD, "SMVD is only allowed with NEXT profile" );
    xConfirmPara( m_compositeRefEnabled, "Composite Reference Frame is only allowed with NEXT profile" );
    xConfirmPara( m_GBi, "GBi is only allowed with NEXT profile" );
    xConfirmPara( m_GBiFast, "GBiFast is only allowed with NEXT profile" );
    xConfirmPara( m_Triangle, "Triangle is only allowed with NEXT profile" );
    xConfirmPara(m_DMVR, "DMVR only allowed with NEXT profile");
    xConfirmPara(m_MmvdDisNum, "Number of distance MMVD entry setting only allowed with NEXT profile");
#if !JVET_O1136_TS_BDPCM_SIGNALLING    
    xConfirmPara(m_RdpcmMode, "RDPCM only allowed with NEXT profile");
#endif    
#if JVET_O0376_SPS_JOINTCBCR_FLAG
    xConfirmPara(m_JointCbCrMode, "JointCbCr only allowed with NEXT profile");
#endif
    // ADD_NEW_TOOL : (parameter check) add a check for next tools here
  }
  else
  {
    if( m_depQuantEnabledFlag )
    {
      xConfirmPara( !m_useRDOQ || !m_useRDOQTS, "RDOQ and RDOQTS must be equal to 1 if dependent quantization is enabled" );
      xConfirmPara( m_signDataHidingEnabledFlag, "SignHideFlag must be equal to 0 if dependent quantization is enabled" );
    }

  }

  if( m_wrapAround )
  {
    xConfirmPara( m_wrapAroundOffset <= m_uiCTUSize + (m_uiMaxCUWidth >> m_uiLog2DiffMaxMinCodingBlockSize), "Wrap-around offset must be greater than CtbSizeY + MinCbSize" );
    xConfirmPara( m_wrapAroundOffset > m_iSourceWidth, "Wrap-around offset must not be greater than the source picture width" );
    int minCUSize =  m_uiCTUSize >> m_uiLog2DiffMaxMinCodingBlockSize;
    xConfirmPara( m_wrapAroundOffset % minCUSize != 0, "Wrap-around offset must be an integer multiple of the specified minimum CU size" );
  }

#if ENABLE_SPLIT_PARALLELISM
  xConfirmPara( m_numSplitThreads < 1, "Number of used threads cannot be smaller than 1" );
  xConfirmPara( m_numSplitThreads > PARL_SPLIT_MAX_NUM_THREADS, "Number of used threads cannot be higher than the number of actual jobs" );
#if _MSC_VER && ENABLE_WPP_PARALLELISM
  xConfirmPara( m_numSplitThreads > 1 && m_numSplitThreads != NUM_SPLIT_THREADS_IF_MSVC, "Due to poor implementation by Microsoft, NumSplitThreads cannot be set dynamically on runtime!" );
#endif
#else
  xConfirmPara( m_numSplitThreads != 1, "ENABLE_SPLIT_PARALLELISM is disabled, numSplitThreads has to be 1" );
#endif

#if ENABLE_WPP_PARALLELISM
  xConfirmPara( m_numWppThreads < 1, "Number of threads used for WPP-style parallelization cannot be smaller than 1" );
  xConfirmPara( m_numWppThreads > PARL_WPP_MAX_NUM_THREADS, "Number of threads used for WPP-style parallelization cannot be bigger than PARL_WPP_MAX_NUM_THREADS" );
  xConfirmPara( !m_ensureWppBitEqual && m_numWppThreads > 1, "WPP bit equality is implied when using WPP-style parallelism" );
#if ENABLE_WPP_STATIC_LINK
  xConfirmPara( m_numWppExtraLines != 0, "WPP-style extra lines out of range" );
#else
  xConfirmPara( m_numWppExtraLines < 0, "WPP-style extra lines out of range" );
#endif
#else
  xConfirmPara( m_numWppThreads != 1, "ENABLE_WPP_PARALLELISM is disabled, numWppThreads has to be 1" );
  xConfirmPara( m_ensureWppBitEqual, "ENABLE_WPP_PARALLELISM is disabled, cannot ensure being WPP bit-equal" );
#endif


#if SHARP_LUMA_DELTA_QP && ENABLE_QPA
  xConfirmPara( m_bUsePerceptQPA && m_lumaLevelToDeltaQPMapping.mode >= 2, "QPA and SharpDeltaQP mode 2 cannot be used together" );
  if( m_bUsePerceptQPA && m_lumaLevelToDeltaQPMapping.mode == LUMALVL_TO_DQP_AVG_METHOD )
  {
    msg( WARNING, "*********************************************************************************\n" );
    msg( WARNING, "** WARNING: Applying custom luma-based QPA with activity-based perceptual QPA! **\n" );
    msg( WARNING, "*********************************************************************************\n" );

    m_lumaLevelToDeltaQPMapping.mode = LUMALVL_TO_DQP_NUM_MODES; // special QPA mode
  }
#endif


  xConfirmPara( m_useAMaxBT && !m_SplitConsOverrideEnabledFlag, "AMaxBt can only be used with PartitionConstriantsOverride enabled" );


  xConfirmPara(m_bitstreamFileName.empty(), "A bitstream file name must be specified (BitstreamFile)");
  const uint32_t maxBitDepth=(m_chromaFormatIDC==CHROMA_400) ? m_internalBitDepth[CHANNEL_TYPE_LUMA] : std::max(m_internalBitDepth[CHANNEL_TYPE_LUMA], m_internalBitDepth[CHANNEL_TYPE_CHROMA]);
  xConfirmPara(m_bitDepthConstraint<maxBitDepth, "The internalBitDepth must not be greater than the bitDepthConstraint value");
  xConfirmPara(m_chromaFormatConstraint<m_chromaFormatIDC, "The chroma format used must not be greater than the chromaFormatConstraint value");

  if (m_profile==Profile::MAINREXT || m_profile==Profile::HIGHTHROUGHPUTREXT)
  {
    xConfirmPara(m_lowerBitRateConstraintFlag==false && m_intraConstraintFlag==false, "The lowerBitRateConstraint flag cannot be false when intraConstraintFlag is false");
    xConfirmPara(m_cabacBypassAlignmentEnabledFlag && m_profile!=Profile::HIGHTHROUGHPUTREXT, "AlignCABACBeforeBypass must not be enabled unless the high throughput profile is being used.");
    if (m_profile == Profile::MAINREXT)
    {
      const uint32_t intraIdx = m_intraConstraintFlag ? 1:0;
      const uint32_t bitDepthIdx = (m_bitDepthConstraint == 8 ? 0 : (m_bitDepthConstraint ==10 ? 1 : (m_bitDepthConstraint == 12 ? 2 : (m_bitDepthConstraint == 16 ? 3 : 4 ))));
      const uint32_t chromaFormatIdx = uint32_t(m_chromaFormatConstraint);
      const bool bValidProfile = (bitDepthIdx > 3 || chromaFormatIdx>3) ? false : (validRExtProfileNames[intraIdx][bitDepthIdx][chromaFormatIdx] != NONE);
      xConfirmPara(!bValidProfile, "Invalid intra constraint flag, bit depth constraint flag and chroma format constraint flag combination for a RExt profile");
      const bool bUsingGeneralRExtTools  = m_transformSkipRotationEnabledFlag        ||
                                           m_transformSkipContextEnabledFlag         ||
                                           m_rdpcmEnabledFlag[RDPCM_SIGNAL_IMPLICIT] ||
                                           m_rdpcmEnabledFlag[RDPCM_SIGNAL_EXPLICIT] ||
                                           !m_enableIntraReferenceSmoothing          ||
                                           m_persistentRiceAdaptationEnabledFlag     ||
                                           m_log2MaxTransformSkipBlockSize!=2;
      const bool bUsingChromaQPTool      = m_cuChromaQpOffsetSubdiv >= 0;
      const bool bUsingExtendedPrecision = m_extendedPrecisionProcessingFlag;

      xConfirmPara((m_chromaFormatConstraint==CHROMA_420 || m_chromaFormatConstraint==CHROMA_400) && bUsingChromaQPTool, "CU Chroma QP adjustment cannot be used for 4:0:0 or 4:2:0 RExt profiles");
      xConfirmPara(m_bitDepthConstraint != 16 && bUsingExtendedPrecision, "Extended precision can only be used in 16-bit RExt profiles");
      if (!(m_chromaFormatConstraint == CHROMA_400 && m_bitDepthConstraint == 16) && m_chromaFormatConstraint!=CHROMA_444)
      {
        xConfirmPara(bUsingGeneralRExtTools, "Combination of tools and profiles are not possible in the specified RExt profile.");
      }
      xConfirmPara( m_onePictureOnlyConstraintFlag && m_chromaFormatConstraint!=CHROMA_444, "chroma format constraint must be 4:4:4 when one-picture-only constraint flag is 1");
      xConfirmPara( m_onePictureOnlyConstraintFlag && m_bitDepthConstraint != 8 && m_bitDepthConstraint != 16, "bit depth constraint must be 8 or 16 when one-picture-only constraint flag is 1");
      xConfirmPara( m_onePictureOnlyConstraintFlag && m_framesToBeEncoded > 1, "Number of frames to be encoded must be 1 when one-picture-only constraint flag is 1.");

      if (!m_intraConstraintFlag && m_bitDepthConstraint==16 && m_chromaFormatConstraint==CHROMA_444)
      {
        msg( WARNING, "********************************************************************************************************\n");
        msg( WARNING, "** WARNING: The RExt constraint flags describe a non standard combination (used for development only) **\n");
        msg( WARNING, "********************************************************************************************************\n");
      }
    }
    else
    {
      xConfirmPara( m_chromaFormatConstraint != CHROMA_444, "chroma format constraint must be 4:4:4 in the High Throughput 4:4:4 16-bit Intra profile.");
      xConfirmPara( m_bitDepthConstraint     != 16,         "bit depth constraint must be 4:4:4 in the High Throughput 4:4:4 16-bit Intra profile.");
      xConfirmPara( m_intraConstraintFlag    != 1,          "intra constraint flag must be 1 in the High Throughput 4:4:4 16-bit Intra profile.");
    }
  }
  else
  {
    xConfirmPara(m_bitDepthConstraint!=((m_profile==Profile::MAIN10 || m_profile==Profile::NEXT)?10:8), "BitDepthConstraint must be 8 for MAIN profile and 10 for MAIN10 profile.");
    xConfirmPara(m_chromaFormatConstraint!=CHROMA_420 && m_profile!=Profile::NEXT, "ChromaFormatConstraint must be 420 for non main-RExt and non-Next profiles.");
    xConfirmPara(m_intraConstraintFlag==true, "IntraConstraintFlag must be false for non main_RExt profiles.");
    xConfirmPara(m_lowerBitRateConstraintFlag==false, "LowerBitrateConstraintFlag must be true for non main-RExt profiles.");
    xConfirmPara(m_profile == Profile::MAINSTILLPICTURE && m_framesToBeEncoded > 1, "Number of frames to be encoded must be 1 when main still picture profile is used.");

    xConfirmPara(m_crossComponentPredictionEnabledFlag==true, "CrossComponentPrediction must not be used for non main-RExt profiles.");
    xConfirmPara(m_log2MaxTransformSkipBlockSize>=6, "Transform Skip Log2 Max Size must be less or equal to 5.");
    xConfirmPara(m_transformSkipRotationEnabledFlag==true, "UseResidualRotation must not be enabled for non main-RExt profiles.");
    xConfirmPara(m_transformSkipContextEnabledFlag==true, "UseSingleSignificanceMapContext must not be enabled for non main-RExt profiles.");
    xConfirmPara(m_rdpcmEnabledFlag[RDPCM_SIGNAL_IMPLICIT]==true, "ImplicitResidualDPCM must not be enabled for non main-RExt profiles.");
    xConfirmPara(m_rdpcmEnabledFlag[RDPCM_SIGNAL_EXPLICIT]==true, "ExplicitResidualDPCM must not be enabled for non main-RExt profiles.");
    xConfirmPara(m_persistentRiceAdaptationEnabledFlag==true, "GolombRiceParameterAdaption must not be enabled for non main-RExt profiles.");
    xConfirmPara(m_extendedPrecisionProcessingFlag==true, "UseExtendedPrecision must not be enabled for non main-RExt profiles.");
    xConfirmPara(m_highPrecisionOffsetsEnabledFlag==true, "UseHighPrecisionPredictionWeighting must not be enabled for non main-RExt profiles.");
    xConfirmPara(m_enableIntraReferenceSmoothing==false, "EnableIntraReferenceSmoothing must be enabled for non main-RExt profiles.");
    xConfirmPara(m_cabacBypassAlignmentEnabledFlag, "AlignCABACBeforeBypass cannot be enabled for non main-RExt profiles.");
  }


  // check range of parameters
  xConfirmPara( m_inputBitDepth[CHANNEL_TYPE_LUMA  ] < 8,                                   "InputBitDepth must be at least 8" );
  xConfirmPara( m_inputBitDepth[CHANNEL_TYPE_CHROMA] < 8,                                   "InputBitDepthC must be at least 8" );

#if !RExt__HIGH_BIT_DEPTH_SUPPORT
  if (m_extendedPrecisionProcessingFlag)
  {
    for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
    {
      xConfirmPara((m_internalBitDepth[channelType] > 8) , "Model is not configured to support high enough internal accuracies - enable RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc...");
    }
  }
  else
  {
    for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
    {
      xConfirmPara((m_internalBitDepth[channelType] > 12) , "Model is not configured to support high enough internal accuracies - enable RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc...");
    }
  }
#endif

  xConfirmPara( (m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA  ] < m_inputBitDepth[CHANNEL_TYPE_LUMA  ]), "MSB-extended bit depth for luma channel (--MSBExtendedBitDepth) must be greater than or equal to input bit depth for luma channel (--InputBitDepth)" );
  xConfirmPara( (m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] < m_inputBitDepth[CHANNEL_TYPE_CHROMA]), "MSB-extended bit depth for chroma channel (--MSBExtendedBitDepthC) must be greater than or equal to input bit depth for chroma channel (--InputBitDepthC)" );

  xConfirmPara( m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA]   > (m_internalBitDepth[CHANNEL_TYPE_LUMA  ]<10?0:(m_internalBitDepth[CHANNEL_TYPE_LUMA  ]-10)), "SaoLumaOffsetBitShift must be in the range of 0 to InternalBitDepth-10, inclusive");
  xConfirmPara( m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] > (m_internalBitDepth[CHANNEL_TYPE_CHROMA]<10?0:(m_internalBitDepth[CHANNEL_TYPE_CHROMA]-10)), "SaoChromaOffsetBitShift must be in the range of 0 to InternalBitDepthC-10, inclusive");

  xConfirmPara( m_chromaFormatIDC >= NUM_CHROMA_FORMAT,                                     "ChromaFormatIDC must be either 400, 420, 422 or 444" );
  std::string sTempIPCSC="InputColourSpaceConvert must be empty, "+getListOfColourSpaceConverts(true);
  xConfirmPara( m_inputColourSpaceConvert >= NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS,         sTempIPCSC.c_str() );
  xConfirmPara( m_InputChromaFormatIDC >= NUM_CHROMA_FORMAT,                                "InputChromaFormatIDC must be either 400, 420, 422 or 444" );
  xConfirmPara( m_iFrameRate <= 0,                                                          "Frame rate must be more than 1" );
  xConfirmPara( m_temporalSubsampleRatio < 1,                                               "Temporal subsample rate must be no less than 1" );
  xConfirmPara( m_framesToBeEncoded <= 0,                                                   "Total Number Of Frames encoded must be more than 0" );
  xConfirmPara( m_framesToBeEncoded < m_switchPOC,                                          "debug POC out of range" );

  xConfirmPara( m_iGOPSize < 1 ,                                                            "GOP Size must be greater or equal to 1" );
  xConfirmPara( m_iGOPSize > 1 &&  m_iGOPSize % 2,                                          "GOP Size must be a multiple of 2, if GOP Size is greater than 1" );
  xConfirmPara( (m_iIntraPeriod > 0 && m_iIntraPeriod < m_iGOPSize) || m_iIntraPeriod == 0, "Intra period must be more than GOP size, or -1 , not 0" );
  xConfirmPara( m_iDecodingRefreshType < 0 || m_iDecodingRefreshType > 3,                   "Decoding Refresh Type must be comprised between 0 and 3 included" );
  if(m_iDecodingRefreshType == 3)
  {
    xConfirmPara( !m_recoveryPointSEIEnabled,                                               "When using RecoveryPointSEI messages as RA points, recoveryPointSEI must be enabled" );
  }

  if (m_isField)
  {
    if (!m_pictureTimingSEIEnabled)
    {
      msg( WARNING, "****************************************************************************\n");
      msg( WARNING, "** WARNING: Picture Timing SEI should be enabled for field coding!        **\n");
      msg( WARNING, "****************************************************************************\n");
    }
  }

  if(m_crossComponentPredictionEnabledFlag && (m_chromaFormatIDC != CHROMA_444))
  {
    msg( WARNING, "****************************************************************************\n");
    msg( WARNING, "** WARNING: Cross-component prediction is specified for 4:4:4 format only **\n");
    msg( WARNING, "****************************************************************************\n");

    m_crossComponentPredictionEnabledFlag = false;
  }

  if ( m_CUTransquantBypassFlagForce && m_bUseHADME )
  {
    msg( WARNING, "****************************************************************************\n");
    msg( WARNING, "** WARNING: --HadamardME has been disabled due to the enabling of         **\n");
    msg( WARNING, "**          --CUTransquantBypassFlagForce                                 **\n");
    msg( WARNING, "****************************************************************************\n");

    m_bUseHADME = false; // this has been disabled so that the lambda is calculated slightly differently for lossless modes (as a result of JCTVC-R0104).
  }

  xConfirmPara (m_log2MaxTransformSkipBlockSize < 2, "Transform Skip Log2 Max Size must be at least 2 (4x4)");


  if( m_SubPuMvpMode == 3 && m_maxNumMergeCand < 7 )
  {
    msg( WARNING, "****************************************************************************\n" );
    msg( WARNING, "** WARNING: Allowing less than 7 merge candidates, although both          **\n" );
    msg( WARNING, "**          advanced sup-pu temporal merging modes are enabled.           **\n" );
    msg( WARNING, "****************************************************************************\n" );
  }
  else if( m_SubPuMvpMode != 0 && m_maxNumMergeCand < 6 )
  {
    msg( WARNING, "****************************************************************************\n" );
    msg( WARNING, "** WARNING: Allowing less than 6 merge candidates, although               **\n" );
    msg( WARNING, "**          an advanced sup-pu temporal merging mode is enabled.          **\n" );
    msg( WARNING, "****************************************************************************\n" );
  }
  xConfirmPara( m_iQP < -6 * (m_internalBitDepth[CHANNEL_TYPE_LUMA] - 8) || m_iQP > MAX_QP, "QP exceeds supported range (-QpBDOffsety to 63)" );
#if W0038_DB_OPT
  xConfirmPara( m_deblockingFilterMetric!=0 && (m_bLoopFilterDisable || m_loopFilterOffsetInPPS), "If DeblockingFilterMetric is non-zero then both LoopFilterDisable and LoopFilterOffsetInPPS must be 0");
#else
  xConfirmPara( m_DeblockingFilterMetric && (m_bLoopFilterDisable || m_loopFilterOffsetInPPS), "If DeblockingFilterMetric is true then both LoopFilterDisable and LoopFilterOffsetInPPS must be 0");
#endif
  xConfirmPara( m_loopFilterBetaOffsetDiv2 < -6 || m_loopFilterBetaOffsetDiv2 > 6,          "Loop Filter Beta Offset div. 2 exceeds supported range (-6 to 6)" );
  xConfirmPara( m_loopFilterTcOffsetDiv2 < -6 || m_loopFilterTcOffsetDiv2 > 6,              "Loop Filter Tc Offset div. 2 exceeds supported range (-6 to 6)" );
  xConfirmPara( m_iSearchRange < 0 ,                                                        "Search Range must be more than 0" );
  xConfirmPara( m_bipredSearchRange < 0 ,                                                   "Bi-prediction refinement search range must be more than 0" );
  xConfirmPara( m_minSearchWindow < 0,                                                      "Minimum motion search window size for the adaptive window ME must be greater than or equal to 0" );
  xConfirmPara( m_iMaxDeltaQP > MAX_DELTA_QP,                                               "Absolute Delta QP exceeds supported range (0 to 7)" );
#if ENABLE_QPA
  xConfirmPara( m_bUsePerceptQPA && m_uiDeltaQpRD > 0,                                      "Perceptual QPA cannot be used together with slice-level multiple-QP optimization" );
#endif
#if SHARP_LUMA_DELTA_QP
  xConfirmPara( m_lumaLevelToDeltaQPMapping.mode && m_uiDeltaQpRD > 0,                      "Luma-level-based Delta QP cannot be used together with slice level multiple-QP optimization\n" );
#endif
  if (m_lumaLevelToDeltaQPMapping.mode && m_lumaReshapeEnable)
  {
    msg(WARNING, "For HDR-PQ, reshaper should be used mutual-exclusively with Luma-level-based Delta QP. If use luma DQP, turn reshaper off.\n");
    m_lumaReshapeEnable = false;
  }
  if (!m_lumaReshapeEnable)
  {
    m_reshapeSignalType = RESHAPE_SIGNAL_NULL;
    m_intraCMD = 0;
  }
  if (m_lumaReshapeEnable && m_reshapeSignalType == RESHAPE_SIGNAL_PQ)
  {
    m_intraCMD = 1;
  }
#if JVET_O0432_LMCS_ENCODER
  else if (m_lumaReshapeEnable && (m_reshapeSignalType == RESHAPE_SIGNAL_SDR || m_reshapeSignalType == RESHAPE_SIGNAL_HLG))
#else
  else if (m_lumaReshapeEnable && m_reshapeSignalType == RESHAPE_SIGNAL_SDR)
#endif
  {
    m_intraCMD = 0;
  }
  else
  {
    m_lumaReshapeEnable = false;
  }
#if JVET_O0432_LMCS_ENCODER
  if (m_lumaReshapeEnable)
  {
    xConfirmPara(m_updateCtrl < 0, "Min. LMCS Update Control is 0");
    xConfirmPara(m_updateCtrl > 2, "Max. LMCS Update Control is 2");
    xConfirmPara(m_adpOption < 0, "Min. LMCS Adaptation Option is 0");
    xConfirmPara(m_adpOption > 4, "Max. LMCS Adaptation Option is 4");
    xConfirmPara(m_initialCW < 0, "Min. Initial Total Codeword is 0");
    xConfirmPara(m_initialCW > 1023, "Max. Initial Total Codeword is 1023");
    if (m_updateCtrl > 0 && m_adpOption > 2) { m_adpOption -= 2; }
  }
#endif

  xConfirmPara( m_cbQpOffset < -12,   "Min. Chroma Cb QP Offset is -12" );
  xConfirmPara( m_cbQpOffset >  12,   "Max. Chroma Cb QP Offset is  12" );
  xConfirmPara( m_crQpOffset < -12,   "Min. Chroma Cr QP Offset is -12" );
  xConfirmPara( m_crQpOffset >  12,   "Max. Chroma Cr QP Offset is  12" );
  xConfirmPara( m_cbQpOffsetDualTree < -12,   "Min. Chroma Cb QP Offset for dual tree is -12" );
  xConfirmPara( m_cbQpOffsetDualTree >  12,   "Max. Chroma Cb QP Offset for dual tree is  12" );
  xConfirmPara( m_crQpOffsetDualTree < -12,   "Min. Chroma Cr QP Offset for dual tree is -12" );
  xConfirmPara( m_crQpOffsetDualTree >  12,   "Max. Chroma Cr QP Offset for dual tree is  12" );
#if JVET_O0376_SPS_JOINTCBCR_FLAG
  if (m_JointCbCrMode && (m_chromaFormatIDC == CHROMA_400))
  {
    msg( WARNING, "****************************************************************************\n");
    msg( WARNING, "** WARNING: --JointCbCr has been disabled because the chromaFormat is 400 **\n");
    msg( WARNING, "****************************************************************************\n");
    m_JointCbCrMode = false;
  }
  if (m_JointCbCrMode)
  {
    xConfirmPara( m_cbCrQpOffset < -12, "Min. Joint Cb-Cr QP Offset is -12");
    xConfirmPara( m_cbCrQpOffset >  12, "Max. Joint Cb-Cr QP Offset is  12");
    xConfirmPara( m_cbCrQpOffsetDualTree < -12, "Min. Joint Cb-Cr QP Offset for dual tree is -12");
    xConfirmPara( m_cbCrQpOffsetDualTree >  12, "Max. Joint Cb-Cr QP Offset for dual tree is  12");
  }
#else
  xConfirmPara( m_cbCrQpOffset < -12, "Min. Joint Cb-Cr QP Offset is -12");
  xConfirmPara( m_cbCrQpOffset >  12, "Max. Joint Cb-Cr QP Offset is  12");
  xConfirmPara( m_cbCrQpOffsetDualTree < -12, "Min. Joint Cb-Cr QP Offset for dual tree is -12");
  xConfirmPara( m_cbCrQpOffsetDualTree >  12, "Max. Joint Cb-Cr QP Offset for dual tree is  12");
#endif
  xConfirmPara( m_iQPAdaptationRange <= 0,                                                  "QP Adaptation Range must be more than 0" );
  if (m_iDecodingRefreshType == 2)
  {
    xConfirmPara( m_iIntraPeriod > 0 && m_iIntraPeriod <= m_iGOPSize ,                      "Intra period must be larger than GOP size for periodic IDR pictures");
  }
  xConfirmPara( m_uiMaxCUDepth > MAX_CU_DEPTH,                                              "MaxPartitionDepth exceeds predefined MAX_CU_DEPTH limit");
  xConfirmPara( m_uiMaxCUWidth > MAX_CU_SIZE,                                               "MaxCUWith exceeds predefined MAX_CU_SIZE limit");

  xConfirmPara( m_uiMinQT[0] < 1<<MIN_CU_LOG2,                                              "Minimum QT size should be larger than or equal to 4");
  xConfirmPara( m_uiMinQT[1] < 1<<MIN_CU_LOG2,                                              "Minimum QT size should be larger than or equal to 4");
#if JVET_O0526_MIN_CTU_SIZE
  xConfirmPara( m_uiCTUSize < 32,                                                           "CTUSize must be greater than or equal to 32");
  xConfirmPara( m_uiCTUSize > 128,                                                          "CTUSize must be less than or equal to 128");
  xConfirmPara( m_uiCTUSize != 32 && m_uiCTUSize != 64 && m_uiCTUSize != 128,               "CTUSize must be a power of 2 (32, 64, or 128)");
#else
  xConfirmPara( m_uiCTUSize < 16,                                                           "Maximum partition width size should be larger than or equal to 16");
  xConfirmPara( m_uiCTUSize < 16,                                                           "Maximum partition height size should be larger than or equal to 16");
#endif
#if !JVET_O0640_PICTURE_SIZE_CONSTRAINT
  xConfirmPara( (m_iSourceWidth  % (1<<MIN_CU_LOG2))!=0,                                    "Resulting coded frame width must be a multiple of the minimum unit size");
  xConfirmPara( (m_iSourceHeight % (1<<MIN_CU_LOG2))!=0,                                    "Resulting coded frame height must be a multiple of the minimum unit size");
  xConfirmPara( (m_iSourceWidth  % (1<<MIN_CU_LOG2))!=0,                                    "Resulting coded frame width must be a multiple of the minimum unit size");
  xConfirmPara( (m_iSourceHeight % (1<<MIN_CU_LOG2))!=0,                                    "Resulting coded frame height must be a multiple of the minimum unit size");
  xConfirmPara( (m_iSourceWidth  % (1<<MIN_CU_LOG2))!=0,                                    "Resulting coded frame width must be a multiple of the minimum unit size");
  xConfirmPara( (m_iSourceHeight % (1<<MIN_CU_LOG2))!=0,                                    "Resulting coded frame height must be a multiple of the minimum unit size");
#endif
  xConfirmPara( m_uiMaxCUDepth < 1,                                                         "MaxPartitionDepth must be greater than zero");
  xConfirmPara( (m_uiMaxCUWidth  >> m_uiMaxCUDepth) < 4,                                    "Minimum partition width size should be larger than or equal to 8");
  xConfirmPara( (m_uiMaxCUHeight >> m_uiMaxCUDepth) < 4,                                    "Minimum partition height size should be larger than or equal to 8");
  xConfirmPara( m_uiMaxCUWidth < 16,                                                        "Maximum partition width size should be larger than or equal to 16");
  xConfirmPara( m_uiMaxCUHeight < 16,                                                       "Maximum partition height size should be larger than or equal to 16");
#if JVET_O0640_PICTURE_SIZE_CONSTRAINT
  xConfirmPara( (m_iSourceWidth  % (std::max(8, int(m_uiMaxCUWidth  >> (m_uiMaxCUDepth - 1))))) != 0, "Resulting coded frame width must be a multiple of Max(8, the minimum CU size)");
  xConfirmPara( (m_iSourceHeight % (std::max(8, int(m_uiMaxCUHeight >> (m_uiMaxCUDepth - 1))))) != 0, "Resulting coded frame height must be a multiple of Max(8, the minimum CU size)");
#else
  xConfirmPara( (m_iSourceWidth  % (m_uiMaxCUWidth  >> (m_uiMaxCUDepth-1)))!=0,             "Resulting coded frame width must be a multiple of the minimum CU size");
  xConfirmPara( (m_iSourceHeight % (m_uiMaxCUHeight >> (m_uiMaxCUDepth-1)))!=0,             "Resulting coded frame height must be a multiple of the minimum CU size");
#endif
#if MAX_TB_SIZE_SIGNALLING
  xConfirmPara( m_log2MaxTbSize > 6, "Log2MaxTbSize must be 6 or smaller." );
#endif
#if JVET_O0545_MAX_TB_SIGNALLING
  xConfirmPara( m_log2MaxTbSize < 5,  "Log2MaxTbSize must be 5 or greater." );
#endif
  xConfirmPara( m_maxNumMergeCand < 1,  "MaxNumMergeCand must be 1 or greater.");
  xConfirmPara( m_maxNumMergeCand > MRG_MAX_NUM_CANDS, "MaxNumMergeCand must be no more than MRG_MAX_NUM_CANDS." );
  xConfirmPara( m_maxNumTriangleCand > TRIANGLE_MAX_NUM_UNI_CANDS, "MaxNumTriangleCand must be no more than TRIANGLE_MAX_NUM_UNI_CANDS." );
  xConfirmPara( m_maxNumTriangleCand > m_maxNumMergeCand, "MaxNumTriangleCand must be no more than MaxNumMergeCand." );
  xConfirmPara( 0 < m_maxNumTriangleCand && m_maxNumTriangleCand < 2, "MaxNumTriangleCand must be no less than 2 unless MaxNumTriangleCand is 0." );
#if JVET_O0455_IBC_MAX_MERGE_NUM
  xConfirmPara( m_maxNumIBCMergeCand < 1, "MaxNumIBCMergeCand must be 1 or greater." );
  xConfirmPara( m_maxNumIBCMergeCand > IBC_MRG_MAX_NUM_CANDS, "MaxNumIBCMergeCand must be no more than IBC_MRG_MAX_NUM_CANDS." );
#endif
  xConfirmPara( m_maxNumAffineMergeCand < 1, "MaxNumAffineMergeCand must be 1 or greater." );
  xConfirmPara( m_maxNumAffineMergeCand > AFFINE_MRG_MAX_NUM_CANDS, "MaxNumAffineMergeCand must be no more than AFFINE_MRG_MAX_NUM_CANDS." );
  if ( m_Affine == 0 )
  {
    m_maxNumAffineMergeCand = m_SubPuMvpMode;
#if JVET_O0070_PROF
    if (m_PROF) msg(WARNING, "PROF is forcefully disabled when Affine is off \n");
    m_PROF = false;
#endif
  }

  xConfirmPara( m_MTS < 0 || m_MTS > 3, "MTS must be greater than 0 smaller than 4" );
  xConfirmPara( m_MTSIntraMaxCand < 0 || m_MTSIntraMaxCand > 5, "m_MTSIntraMaxCand must be greater than 0 and smaller than 6" );
  xConfirmPara( m_MTSInterMaxCand < 0 || m_MTSInterMaxCand > 5, "m_MTSInterMaxCand must be greater than 0 and smaller than 6" );
  xConfirmPara( m_MTS != 0 && m_MTSImplicit != 0, "Both explicit and implicit MTS cannot be enabled at the same time" );
#if !JVET_O0525_REMOVE_PCM
  if( m_usePCM)
  {
    for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
    {
      xConfirmPara(((m_MSBExtendedBitDepth[channelType] > m_internalBitDepth[channelType]) && m_bPCMInputBitDepthFlag), "PCM bit depth cannot be greater than internal bit depth (PCMInputBitDepthFlag cannot be used when InputBitDepth or MSBExtendedBitDepth > InternalBitDepth)");
    }
    xConfirmPara(  m_uiPCMLog2MinSize < 3,                                      "PCMLog2MinSize must be 3 or greater.");
    xConfirmPara(  m_uiPCMLog2MinSize > 5,                                      "PCMLog2MinSize must be 5 or smaller.");
    xConfirmPara(  m_pcmLog2MaxSize > 5,                                        "PCMLog2MaxSize must be 5 or smaller.");
    xConfirmPara(  m_pcmLog2MaxSize < m_uiPCMLog2MinSize,                       "PCMLog2MaxSize must be equal to or greater than m_uiPCMLog2MinSize.");
  }
#endif

  if (m_useBDPCM)
  {
    xConfirmPara(!m_useTransformSkip, "BDPCM cannot be used when transform skip is disabled.");
  }

  if (m_sliceMode!=NO_SLICES)
  {
    xConfirmPara( m_sliceArgument < 1 ,         "SliceArgument should be larger than or equal to 1" );
  }


  bool tileFlag = (m_numTileColumnsMinus1 > 0 || m_numTileRowsMinus1 > 0 );
  if (m_profile!=Profile::HIGHTHROUGHPUTREXT)
  {
    xConfirmPara( tileFlag && m_entropyCodingSyncEnabledFlag, "Tiles and entropy-coding-sync (Wavefronts) can not be applied together, except in the High Throughput Intra 4:4:4 16 profile");
  }

  xConfirmPara( m_iSourceWidth  % SPS::getWinUnitX(m_chromaFormatIDC) != 0, "Picture width must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_iSourceHeight % SPS::getWinUnitY(m_chromaFormatIDC) != 0, "Picture height must be an integer multiple of the specified chroma subsampling");

  xConfirmPara( m_aiPad[0] % SPS::getWinUnitX(m_chromaFormatIDC) != 0, "Horizontal padding must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_aiPad[1] % SPS::getWinUnitY(m_chromaFormatIDC) != 0, "Vertical padding must be an integer multiple of the specified chroma subsampling");

  xConfirmPara( m_confWinLeft   % SPS::getWinUnitX(m_chromaFormatIDC) != 0, "Left conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confWinRight  % SPS::getWinUnitX(m_chromaFormatIDC) != 0, "Right conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confWinTop    % SPS::getWinUnitY(m_chromaFormatIDC) != 0, "Top conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confWinBottom % SPS::getWinUnitY(m_chromaFormatIDC) != 0, "Bottom conformance window offset must be an integer multiple of the specified chroma subsampling");


  // max CU width and height should be power of 2
  uint32_t ui = m_uiMaxCUWidth;
  while(ui)
  {
    ui >>= 1;
    if( (ui & 1) == 1)
    {
      xConfirmPara( ui != 1 , "Width should be 2^n");
    }
  }
  ui = m_uiMaxCUHeight;
  while(ui)
  {
    ui >>= 1;
    if( (ui & 1) == 1)
    {
      xConfirmPara( ui != 1 , "Height should be 2^n");
    }
  }

  /* if this is an intra-only sequence, ie IntraPeriod=1, don't verify the GOP structure
   * This permits the ability to omit a GOP structure specification */
  if (m_iIntraPeriod == 1 && m_GOPList[0].m_POC == -1)
  {
    m_GOPList[0] = GOPEntry();
    m_GOPList[0].m_QPFactor = 1;
    m_GOPList[0].m_betaOffsetDiv2 = 0;
    m_GOPList[0].m_tcOffsetDiv2 = 0;
    m_GOPList[0].m_POC = 1;
    m_RPLList0[0] = RPLEntry();
    m_RPLList1[0] = RPLEntry();
    m_RPLList0[0].m_POC = m_RPLList1[0].m_POC = 1;
    m_RPLList0[0].m_numRefPicsActive = 4;
    m_GOPList[0].m_numRefPicsActive0 = 4;
  }
  else
  {
    xConfirmPara( m_intraConstraintFlag, "IntraConstraintFlag cannot be 1 for inter sequences");
  }

  int multipleFactor = m_compositeRefEnabled ? 2 : 1;
  bool verifiedGOP=false;
  bool errorGOP=false;
  int checkGOP=1;
  int numRefs = m_isField ? 2 : 1;
  int refList[MAX_NUM_REF_PICS+1] = {0};
  if(m_isField)
  {
    refList[1] = 1;
  }
  bool isOK[MAX_GOP];
  for(int i=0; i<MAX_GOP; i++)
  {
    isOK[i]=false;
  }
  int numOK=0;
  xConfirmPara( m_iIntraPeriod >=0&&(m_iIntraPeriod%m_iGOPSize!=0), "Intra period must be a multiple of GOPSize, or -1" );

  for(int i=0; i<m_iGOPSize; i++)
  {
    if (m_GOPList[i].m_POC == m_iGOPSize * multipleFactor)
    {
      xConfirmPara( m_GOPList[i].m_temporalId!=0 , "The last frame in each GOP must have temporal ID = 0 " );
    }
  }

  if ( (m_iIntraPeriod != 1) && !m_loopFilterOffsetInPPS && (!m_bLoopFilterDisable) )
  {
    for(int i=0; i<m_iGOPSize; i++)
    {
      xConfirmPara( (m_GOPList[i].m_betaOffsetDiv2 + m_loopFilterBetaOffsetDiv2) < -6 || (m_GOPList[i].m_betaOffsetDiv2 + m_loopFilterBetaOffsetDiv2) > 6, "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-6 to 6)" );
      xConfirmPara( (m_GOPList[i].m_tcOffsetDiv2 + m_loopFilterTcOffsetDiv2) < -6 || (m_GOPList[i].m_tcOffsetDiv2 + m_loopFilterTcOffsetDiv2) > 6, "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-6 to 6)" );
    }
  }

#if W0038_CQP_ADJ
  for(int i=0; i<m_iGOPSize; i++)
  {
    xConfirmPara( abs(m_GOPList[i].m_CbQPoffset               ) > 12, "Cb QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    xConfirmPara( abs(m_GOPList[i].m_CbQPoffset + m_cbQpOffset) > 12, "Cb QP Offset for one of the GOP entries, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
    xConfirmPara( abs(m_GOPList[i].m_CrQPoffset               ) > 12, "Cr QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    xConfirmPara( abs(m_GOPList[i].m_CrQPoffset + m_crQpOffset) > 12, "Cr QP Offset for one of the GOP entries, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );
  }
  xConfirmPara( abs(m_sliceChromaQpOffsetIntraOrPeriodic[0]                 ) > 12, "Intra/periodic Cb QP Offset exceeds supported range (-12 to 12)" );
  xConfirmPara( abs(m_sliceChromaQpOffsetIntraOrPeriodic[0]  + m_cbQpOffset ) > 12, "Intra/periodic Cb QP Offset, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
  xConfirmPara( abs(m_sliceChromaQpOffsetIntraOrPeriodic[1]                 ) > 12, "Intra/periodic Cr QP Offset exceeds supported range (-12 to 12)" );
  xConfirmPara( abs(m_sliceChromaQpOffsetIntraOrPeriodic[1]  + m_crQpOffset ) > 12, "Intra/periodic Cr QP Offset, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );
#endif

#if JVET_O0050_LOCAL_DUAL_TREE
  xConfirmPara( m_fastLocalDualTreeMode < 0 || m_fastLocalDualTreeMode > 2, "FastLocalDualTreeMode must be in range [0..2]" );
#endif

  int extraRPLs = 0;
  //start looping through frames in coding order until we can verify that the GOP structure is correct.
  while (!verifiedGOP && !errorGOP)
  {
    int curGOP = (checkGOP - 1) % m_iGOPSize;
    int curPOC = ((checkGOP - 1) / m_iGOPSize)*m_iGOPSize * multipleFactor + m_RPLList0[curGOP].m_POC;
    if (m_RPLList0[curGOP].m_POC < 0 || m_RPLList1[curGOP].m_POC < 0)
    {
      msg(WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n");
      errorGOP = true;
    }
    else
    {
      //check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next GOP.
      bool beforeI = false;
      for (int i = 0; i< m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
        if (absPOC < 0)
        {
          beforeI = true;
        }
        else
        {
          bool found = false;
          for (int j = 0; j<numRefs; j++)
          {
            if (refList[j] == absPOC)
            {
              found = true;
              for (int k = 0; k<m_iGOPSize; k++)
              {
                if (absPOC % (m_iGOPSize * multipleFactor) == m_RPLList0[k].m_POC % (m_iGOPSize * multipleFactor))
                {
                  if (m_RPLList0[k].m_temporalId == m_RPLList0[curGOP].m_temporalId)
                  {
                    m_RPLList0[k].m_refPic = true;
                  }
                }
              }
            }
          }
          if (!found)
          {
            msg(WARNING, "\nError: ref pic %d is not available for GOP frame %d\n", m_RPLList0[curGOP].m_deltaRefPics[i], curGOP + 1);
            errorGOP = true;
          }
        }
      }
      if (!beforeI && !errorGOP)
      {
        //all ref frames were present
        if (!isOK[curGOP])
        {
          numOK++;
          isOK[curGOP] = true;
          if (numOK == m_iGOPSize)
          {
            verifiedGOP = true;
          }
        }
      }
      else
      {
        //create a new RPLEntry for this frame containing all the reference pictures that were available (POC > 0)
        m_RPLList0[m_iGOPSize + extraRPLs] = m_RPLList0[curGOP];
        m_RPLList1[m_iGOPSize + extraRPLs] = m_RPLList1[curGOP];
        int newRefs0 = 0;
        for (int i = 0; i< m_RPLList0[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[newRefs0] = m_RPLList0[curGOP].m_deltaRefPics[i];
            newRefs0++;
          }
        }
        int numPrefRefs0 = m_RPLList0[curGOP].m_numRefPicsActive;

        int newRefs1 = 0;
        for (int i = 0; i< m_RPLList1[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - m_RPLList1[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[newRefs1] = m_RPLList1[curGOP].m_deltaRefPics[i];
            newRefs1++;
          }
        }
        int numPrefRefs1 = m_RPLList1[curGOP].m_numRefPicsActive;

        for (int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % m_iGOPSize;
          int offPOC = ((checkGOP - 1 + offset) / m_iGOPSize)*(m_iGOPSize * multipleFactor) + m_RPLList0[offGOP].m_POC;
          if (offPOC >= 0 && m_RPLList0[offGOP].m_temporalId <= m_RPLList0[curGOP].m_temporalId)
          {
            bool newRef = false;
            for (int i = 0; i<(newRefs0 + newRefs1); i++)
            {
              if (refList[i] == offPOC)
              {
                newRef = true;
              }
            }
            for (int i = 0; i<newRefs0; i++)
            {
              if (m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newRefs0;
              //this picture can be added, find appropriate place in list and insert it.
              if (m_RPLList0[offGOP].m_temporalId == m_RPLList0[curGOP].m_temporalId)
              {
                m_RPLList0[offGOP].m_refPic = true;
              }
              for (int j = 0; j<newRefs0; j++)
              {
                if (m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for (int j = insertPoint; j<newRefs0 + 1; j++)
              {
                int newPrev = m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[j];
                m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[j] = prev;
                prev = newPrev;
              }
              newRefs0++;
            }
          }
          if (newRefs0 >= numPrefRefs0)
          {
            break;
          }
        }

        for (int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % m_iGOPSize;
          int offPOC = ((checkGOP - 1 + offset) / m_iGOPSize)*(m_iGOPSize * multipleFactor) + m_RPLList1[offGOP].m_POC;
          if (offPOC >= 0 && m_RPLList1[offGOP].m_temporalId <= m_RPLList1[curGOP].m_temporalId)
          {
            bool newRef = false;
            for (int i = 0; i<(newRefs0 + newRefs1); i++)
            {
              if (refList[i] == offPOC)
              {
                newRef = true;
              }
            }
            for (int i = 0; i<newRefs1; i++)
            {
              if (m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newRefs1;
              //this picture can be added, find appropriate place in list and insert it.
              if (m_RPLList1[offGOP].m_temporalId == m_RPLList1[curGOP].m_temporalId)
              {
                m_RPLList1[offGOP].m_refPic = true;
              }
              for (int j = 0; j<newRefs1; j++)
              {
                if (m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for (int j = insertPoint; j<newRefs1 + 1; j++)
              {
                int newPrev = m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[j];
                m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[j] = prev;
                prev = newPrev;
              }
              newRefs1++;
            }
          }
          if (newRefs1 >= numPrefRefs1)
          {
            break;
          }
        }

        m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPics = newRefs0;
        m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPicsActive = min(m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPics, m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPicsActive);
        m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPics = newRefs1;
        m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPicsActive = min(m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPics, m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPicsActive);
        curGOP = m_iGOPSize + extraRPLs;
        extraRPLs++;
      }
      numRefs = 0;
      for (int i = 0; i< m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
        if (absPOC >= 0)
        {
          refList[numRefs] = absPOC;
          numRefs++;
        }
      }
      for (int i = 0; i< m_RPLList1[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - m_RPLList1[curGOP].m_deltaRefPics[i];
        if (absPOC >= 0)
        {
          bool alreadyExist = false;
          for (int j = 0; !alreadyExist && j < numRefs; j++)
          {
            if (refList[j] == absPOC)
            {
              alreadyExist = true;
            }
          }
          if (!alreadyExist)
          {
            refList[numRefs] = absPOC;
            numRefs++;
          }
        }
      }
      refList[numRefs] = curPOC;
      numRefs++;
    }
    checkGOP++;
  }
  xConfirmPara(errorGOP, "Invalid GOP structure given");

  m_maxTempLayer = 1;

  for(int i=0; i<m_iGOPSize; i++)
  {
    if(m_GOPList[i].m_temporalId >= m_maxTempLayer)
    {
      m_maxTempLayer = m_GOPList[i].m_temporalId+1;
    }
    xConfirmPara(m_GOPList[i].m_sliceType!='B' && m_GOPList[i].m_sliceType!='P' && m_GOPList[i].m_sliceType!='I', "Slice type must be equal to B or P or I");
  }
  for(int i=0; i<MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_maxDecPicBuffering[i] = 1;
  }
  for(int i=0; i<m_iGOPSize; i++)
  {
    int numRefPic = m_RPLList0[i].m_numRefPics;
    for (int tmp = 0; tmp < m_RPLList1[i].m_numRefPics; tmp++)
    {
      bool notSame = true;
      for (int jj = 0; notSame && jj < m_RPLList0[i].m_numRefPics; jj++)
      {
        if (m_RPLList1[i].m_deltaRefPics[tmp] == m_RPLList0[i].m_deltaRefPics[jj]) notSame = false;
      }
      if (notSame) numRefPic++;
    }
    if (numRefPic + 1 > m_maxDecPicBuffering[m_GOPList[i].m_temporalId])
    {
      m_maxDecPicBuffering[m_GOPList[i].m_temporalId] = numRefPic + 1;
    }
    int highestDecodingNumberWithLowerPOC = 0;
    for(int j=0; j<m_iGOPSize; j++)
    {
      if(m_GOPList[j].m_POC <= m_GOPList[i].m_POC)
      {
        highestDecodingNumberWithLowerPOC = j;
      }
    }
    int numReorder = 0;
    for(int j=0; j<highestDecodingNumberWithLowerPOC; j++)
    {
      if(m_GOPList[j].m_temporalId <= m_GOPList[i].m_temporalId &&
        m_GOPList[j].m_POC > m_GOPList[i].m_POC)
      {
        numReorder++;
      }
    }
    if(numReorder > m_numReorderPics[m_GOPList[i].m_temporalId])
    {
      m_numReorderPics[m_GOPList[i].m_temporalId] = numReorder;
    }
  }

  for(int i=0; i<MAX_TLAYER-1; i++)
  {
    // a lower layer can not have higher value of m_numReorderPics than a higher layer
    if(m_numReorderPics[i+1] < m_numReorderPics[i])
    {
      m_numReorderPics[i+1] = m_numReorderPics[i];
    }
    // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1, inclusive
    if(m_numReorderPics[i] > m_maxDecPicBuffering[i] - 1)
    {
      m_maxDecPicBuffering[i] = m_numReorderPics[i] + 1;
    }
    // a lower layer can not have higher value of m_uiMaxDecPicBuffering than a higher layer
    if(m_maxDecPicBuffering[i+1] < m_maxDecPicBuffering[i])
    {
      m_maxDecPicBuffering[i+1] = m_maxDecPicBuffering[i];
    }
  }

  // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] -  1, inclusive
  if(m_numReorderPics[MAX_TLAYER-1] > m_maxDecPicBuffering[MAX_TLAYER-1] - 1)
  {
    m_maxDecPicBuffering[MAX_TLAYER-1] = m_numReorderPics[MAX_TLAYER-1] + 1;
  }


  for (int i=0; i<MAX_TILES; i++)
  {
    if (m_brickSplits[i].m_tileIdx>=0)
    {
      m_brickSplitMap[m_brickSplits[i].m_tileIdx] = m_brickSplits[i];
      // ToDo: check that brick dimensions don't exceed tile dimensions
    }
  }

  if ((m_MCTSEncConstraint) && (m_bLFCrossTileBoundaryFlag))
  {
    printf("Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling filtering across tile boundaries!\n");
    m_bLFCrossTileBoundaryFlag = false;
  }
  if ((m_MCTSEncConstraint) && (m_TMVPModeId))
  {
    printf("Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling TMVP!\n");
    m_TMVPModeId = 0;
  }

  if ((m_MCTSEncConstraint) && ( m_alf ))
  {
    printf("Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling ALF!\n");
    m_alf = false;
  }
  if( ( m_MCTSEncConstraint ) && ( m_BIO ) )
  {
    printf( "Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling BIO!\n" );
    m_BIO = false;
  }

  if (m_toneMappingInfoSEIEnabled)
  {
    xConfirmPara( m_toneMapCodedDataBitDepth < 8 || m_toneMapCodedDataBitDepth > 14 , "SEIToneMapCodedDataBitDepth must be in rage 8 to 14");
    xConfirmPara( m_toneMapTargetBitDepth < 1 || (m_toneMapTargetBitDepth > 16 && m_toneMapTargetBitDepth < 255) , "SEIToneMapTargetBitDepth must be in rage 1 to 16 or equal to 255");
    xConfirmPara( m_toneMapModelId < 0 || m_toneMapModelId > 4 , "SEIToneMapModelId must be in rage 0 to 4");
    xConfirmPara( m_cameraIsoSpeedValue == 0, "SEIToneMapCameraIsoSpeedValue shall not be equal to 0");
    xConfirmPara( m_exposureIndexValue  == 0, "SEIToneMapExposureIndexValue shall not be equal to 0");
    xConfirmPara( m_extendedRangeWhiteLevel < 100, "SEIToneMapExtendedRangeWhiteLevel should be greater than or equal to 100");
    xConfirmPara( m_nominalBlackLevelLumaCodeValue >= m_nominalWhiteLevelLumaCodeValue, "SEIToneMapNominalWhiteLevelLumaCodeValue shall be greater than SEIToneMapNominalBlackLevelLumaCodeValue");
    xConfirmPara( m_extendedWhiteLevelLumaCodeValue < m_nominalWhiteLevelLumaCodeValue, "SEIToneMapExtendedWhiteLevelLumaCodeValue shall be greater than or equal to SEIToneMapNominalWhiteLevelLumaCodeValue");
  }

  if (m_kneeSEIEnabled && !m_kneeSEICancelFlag)
  {
    xConfirmPara( m_kneeSEINumKneePointsMinus1 < 0 || m_kneeSEINumKneePointsMinus1 > 998, "SEIKneeFunctionNumKneePointsMinus1 must be in the range of 0 to 998");
    for ( uint32_t i=0; i<=m_kneeSEINumKneePointsMinus1; i++ )
    {
      xConfirmPara( m_kneeSEIInputKneePoint[i] < 1 || m_kneeSEIInputKneePoint[i] > 999, "SEIKneeFunctionInputKneePointValue must be in the range of 1 to 999");
      xConfirmPara( m_kneeSEIOutputKneePoint[i] < 0 || m_kneeSEIOutputKneePoint[i] > 1000, "SEIKneeFunctionInputKneePointValue must be in the range of 0 to 1000");
      if ( i > 0 )
      {
        xConfirmPara( m_kneeSEIInputKneePoint[i-1] >= m_kneeSEIInputKneePoint[i],  "The i-th SEIKneeFunctionInputKneePointValue must be greater than the (i-1)-th value");
        xConfirmPara( m_kneeSEIOutputKneePoint[i-1] > m_kneeSEIOutputKneePoint[i],  "The i-th SEIKneeFunctionOutputKneePointValue must be greater than or equal to the (i-1)-th value");
      }
    }
  }

  if (m_chromaResamplingFilterSEIenabled)
  {
    xConfirmPara( (m_chromaFormatIDC == CHROMA_400 ), "chromaResamplingFilterSEI is not allowed to be present when ChromaFormatIDC is equal to zero (4:0:0)" );
    xConfirmPara(m_vuiParametersPresentFlag && m_chromaLocInfoPresentFlag && (m_chromaSampleLocTypeTopField != m_chromaSampleLocTypeBottomField ), "When chromaResamplingFilterSEI is enabled, ChromaSampleLocTypeTopField has to be equal to ChromaSampleLocTypeBottomField" );
  }

  if ( m_RCEnableRateControl )
  {
    if ( m_RCForceIntraQP )
    {
      if ( m_RCInitialQP == 0 )
      {
        msg( WARNING, "\nInitial QP for rate control is not specified. Reset not to use force intra QP!" );
        m_RCForceIntraQP = false;
      }
    }
    xConfirmPara( m_uiDeltaQpRD > 0, "Rate control cannot be used together with slice level multiple-QP optimization!\n" );
#if U0132_TARGET_BITS_SATURATION
    if ((m_RCCpbSaturationEnabled) && (m_level!=Level::NONE) && (m_profile!=Profile::NONE))
    {
      uint32_t uiLevelIdx = (m_level / 10) + (uint32_t)((m_level % 10) / 3);    // (m_level / 30)*3 + ((m_level % 10) / 3);
      xConfirmPara(m_RCCpbSize > g_uiMaxCpbSize[m_levelTier][uiLevelIdx], "RCCpbSize should be smaller than or equal to Max CPB size according to tier and level");
      xConfirmPara(m_RCInitialCpbFullness > 1, "RCInitialCpbFullness should be smaller than or equal to 1");
    }
#endif
  }
#if U0132_TARGET_BITS_SATURATION
  else
  {
    xConfirmPara( m_RCCpbSaturationEnabled != 0, "Target bits saturation cannot be processed without Rate control" );
  }
#endif

  xConfirmPara(!m_TransquantBypassEnabledFlag && m_CUTransquantBypassFlagForce, "CUTransquantBypassFlagForce cannot be 1 when TransquantBypassEnableFlag is 0");

  xConfirmPara(m_log2ParallelMergeLevel < 2, "Log2ParallelMergeLevel should be larger than or equal to 2");

  if (m_framePackingSEIEnabled)
  {
    xConfirmPara(m_framePackingSEIType < 3 || m_framePackingSEIType > 5 , "SEIFramePackingType must be in rage 3 to 5");
  }

  if (m_segmentedRectFramePackingSEIEnabled)
  {
    xConfirmPara(m_framePackingSEIEnabled , "SEISegmentedRectFramePacking must be 0 when SEIFramePacking is 1");
  }

  if((m_numTileColumnsMinus1 <= 0) && (m_numTileRowsMinus1 <= 0) && m_tmctsSEIEnabled)
  {
    msg( WARNING, "Warning: SEITempMotionConstrainedTileSets is set to false to disable temporal motion-constrained tile sets SEI message because there are no tiles enabled.\n");
    m_tmctsSEIEnabled = false;
  }

  if(m_timeCodeSEIEnabled)
  {
    xConfirmPara(m_timeCodeSEINumTs > MAX_TIMECODE_SEI_SETS, "Number of time sets cannot exceed 3");
  }

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  xConfirmPara(m_preferredTransferCharacteristics > 255, "transfer_characteristics_idc should not be greater than 255.");
#endif
  xConfirmPara( unsigned(m_ImvMode) > 1, "ImvMode exceeds range (0 to 1)" );
  xConfirmPara( m_decodeBitstreams[0] == m_bitstreamFileName, "Debug bitstream and the output bitstream cannot be equal.\n" );
  xConfirmPara( m_decodeBitstreams[1] == m_bitstreamFileName, "Decode2 bitstream and the output bitstream cannot be equal.\n" );
  xConfirmPara(unsigned(m_LMChroma) > 1, "LMMode exceeds range (0 to 1)");
#if EXTENSION_360_VIDEO
  check_failed |= m_ext360.verifyParameters();
#endif

#undef xConfirmPara
  return check_failed;
}

const char *profileToString(const Profile::Name profile)
{
  static const uint32_t numberOfProfiles = sizeof(strToProfile)/sizeof(*strToProfile);

  for (uint32_t profileIndex = 0; profileIndex < numberOfProfiles; profileIndex++)
  {
    if (strToProfile[profileIndex].value == profile)
    {
      return strToProfile[profileIndex].str;
    }
  }

  //if we get here, we didn't find this profile in the list - so there is an error
  EXIT( "ERROR: Unknown profile \"" << profile << "\" in profileToString" );
  return "";
}

void EncAppCfg::xPrintParameter()
{
  //msg( DETAILS, "\n" );
  msg( DETAILS, "Input          File                    : %s\n", m_inputFileName.c_str() );
  msg( DETAILS, "Bitstream      File                    : %s\n", m_bitstreamFileName.c_str() );
  msg( DETAILS, "Reconstruction File                    : %s\n", m_reconFileName.c_str() );
  msg( DETAILS, "Real     Format                        : %dx%d %gHz\n", m_iSourceWidth - m_confWinLeft - m_confWinRight, m_iSourceHeight - m_confWinTop - m_confWinBottom, (double)m_iFrameRate / m_temporalSubsampleRatio );
  msg( DETAILS, "Internal Format                        : %dx%d %gHz\n", m_iSourceWidth, m_iSourceHeight, (double)m_iFrameRate / m_temporalSubsampleRatio );
  msg( DETAILS, "Sequence PSNR output                   : %s\n", ( m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only" ) );
  msg( DETAILS, "Hexadecimal PSNR output                : %s\n", ( m_printHexPsnr ? "Enabled" : "Disabled" ) );
  msg( DETAILS, "Sequence MSE output                    : %s\n", ( m_printSequenceMSE ? "Enabled" : "Disabled" ) );
  msg( DETAILS, "Frame MSE output                       : %s\n", ( m_printFrameMSE ? "Enabled" : "Disabled" ) );
  msg( DETAILS, "Cabac-zero-word-padding                : %s\n", ( m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled" ) );
  if (m_isField)
  {
    msg( DETAILS, "Frame/Field                            : Field based coding\n" );
    msg( DETAILS, "Field index                            : %u - %d (%d fields)\n", m_FrameSkip, m_FrameSkip + m_framesToBeEncoded - 1, m_framesToBeEncoded );
    msg( DETAILS, "Field Order                            : %s field first\n", m_isTopFieldFirst ? "Top" : "Bottom" );

  }
  else
  {
    msg( DETAILS, "Frame/Field                            : Frame based coding\n" );
    msg( DETAILS, "Frame index                            : %u - %d (%d frames)\n", m_FrameSkip, m_FrameSkip + m_framesToBeEncoded - 1, m_framesToBeEncoded );
  }
  if (m_profile == Profile::MAINREXT)
  {
    ExtendedProfileName validProfileName;
    if (m_onePictureOnlyConstraintFlag)
    {
      validProfileName = m_bitDepthConstraint == 8 ? MAIN_444_STILL_PICTURE : (m_bitDepthConstraint == 16 ? MAIN_444_16_STILL_PICTURE : NONE);
    }
    else
    {
      const uint32_t intraIdx = m_intraConstraintFlag ? 1:0;
      const uint32_t bitDepthIdx = (m_bitDepthConstraint == 8 ? 0 : (m_bitDepthConstraint ==10 ? 1 : (m_bitDepthConstraint == 12 ? 2 : (m_bitDepthConstraint == 16 ? 3 : 4 ))));
      const uint32_t chromaFormatIdx = uint32_t(m_chromaFormatConstraint);
      validProfileName = (bitDepthIdx > 3 || chromaFormatIdx>3) ? NONE : validRExtProfileNames[intraIdx][bitDepthIdx][chromaFormatIdx];
    }
    std::string rextSubProfile;
    if (validProfileName!=NONE)
    {
      rextSubProfile=enumToString(strToExtendedProfile, sizeof(strToExtendedProfile)/sizeof(*strToExtendedProfile), validProfileName);
    }
    if (rextSubProfile == "main_444_16")
    {
      rextSubProfile="main_444_16 [NON STANDARD]";
    }
    msg( DETAILS, "Profile                                : %s (%s)\n", profileToString(m_profile), (rextSubProfile.empty())?"INVALID REXT PROFILE":rextSubProfile.c_str() );
  }
  else
  {
    msg( DETAILS, "Profile                                : %s\n", profileToString(m_profile) );
  }
  msg( DETAILS, "CU size / depth / total-depth          : %d / %d / %d\n", m_uiMaxCUWidth, m_uiMaxCUDepth, m_uiMaxCodingDepth );
#if MAX_TB_SIZE_SIGNALLING
  msg( DETAILS, "Max TB size                            : %d \n", 1 << m_log2MaxTbSize );
#endif
#if !JVET_O0525_REMOVE_PCM
  msg( DETAILS, "Min PCM size                           : %d\n", 1 << m_uiPCMLog2MinSize);
#endif
  msg( DETAILS, "Motion search range                    : %d\n", m_iSearchRange );
  msg( DETAILS, "Intra period                           : %d\n", m_iIntraPeriod );
  msg( DETAILS, "Decoding refresh type                  : %d\n", m_iDecodingRefreshType );
#if QP_SWITCHING_FOR_PARALLEL
  if (m_qpIncrementAtSourceFrame.bPresent)
  {
    msg( DETAILS, "QP                                     : %d (incrementing internal QP at source frame %d)\n", m_iQP, m_qpIncrementAtSourceFrame.value);
  }
  else
  {
    msg( DETAILS, "QP                                     : %d\n", m_iQP);
  }
#else
  msg( DETAILS, "QP                                     : %5.2f\n", m_fQP );
#endif
  msg( DETAILS, "Max dQP signaling subdiv               : %d\n", m_cuQpDeltaSubdiv);

  msg( DETAILS, "Cb QP Offset (dual tree)               : %d (%d)\n", m_cbQpOffset, m_cbQpOffsetDualTree);
  msg( DETAILS, "Cr QP Offset (dual tree)               : %d (%d)\n", m_crQpOffset, m_crQpOffsetDualTree);
  msg( DETAILS, "QP adaptation                          : %d (range=%d)\n", m_bUseAdaptiveQP, (m_bUseAdaptiveQP ? m_iQPAdaptationRange : 0) );
  msg( DETAILS, "GOP size                               : %d\n", m_iGOPSize );
  msg( DETAILS, "Input bit depth                        : (Y:%d, C:%d)\n", m_inputBitDepth[CHANNEL_TYPE_LUMA], m_inputBitDepth[CHANNEL_TYPE_CHROMA] );
  msg( DETAILS, "MSB-extended bit depth                 : (Y:%d, C:%d)\n", m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA], m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] );
  msg( DETAILS, "Internal bit depth                     : (Y:%d, C:%d)\n", m_internalBitDepth[CHANNEL_TYPE_LUMA], m_internalBitDepth[CHANNEL_TYPE_CHROMA] );
#if !JVET_O0525_REMOVE_PCM
  msg( DETAILS, "PCM sample bit depth                   : (Y:%d, C:%d)\n", m_bPCMInputBitDepthFlag ? m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA] : m_internalBitDepth[CHANNEL_TYPE_LUMA],
                                                                    m_bPCMInputBitDepthFlag ? m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] : m_internalBitDepth[CHANNEL_TYPE_CHROMA] );
#endif
  msg( DETAILS, "Intra reference smoothing              : %s\n", (m_enableIntraReferenceSmoothing           ? "Enabled" : "Disabled") );
  msg( DETAILS, "cu_chroma_qp_offset_subdiv             : %d\n", m_cuChromaQpOffsetSubdiv);
  msg( DETAILS, "extended_precision_processing_flag     : %s\n", (m_extendedPrecisionProcessingFlag         ? "Enabled" : "Disabled") );
  msg( DETAILS, "implicit_rdpcm_enabled_flag            : %s\n", (m_rdpcmEnabledFlag[RDPCM_SIGNAL_IMPLICIT] ? "Enabled" : "Disabled") );
  msg( DETAILS, "explicit_rdpcm_enabled_flag            : %s\n", (m_rdpcmEnabledFlag[RDPCM_SIGNAL_EXPLICIT] ? "Enabled" : "Disabled") );
  msg( DETAILS, "transform_skip_rotation_enabled_flag   : %s\n", (m_transformSkipRotationEnabledFlag        ? "Enabled" : "Disabled") );
  msg( DETAILS, "transform_skip_context_enabled_flag    : %s\n", (m_transformSkipContextEnabledFlag         ? "Enabled" : "Disabled") );
  msg( DETAILS, "cross_component_prediction_enabled_flag: %s\n", (m_crossComponentPredictionEnabledFlag     ? (m_reconBasedCrossCPredictionEstimate ? "Enabled (reconstructed-residual-based estimate)" : "Enabled (encoder-side-residual-based estimate)") : "Disabled") );
  msg( DETAILS, "high_precision_offsets_enabled_flag    : %s\n", (m_highPrecisionOffsetsEnabledFlag         ? "Enabled" : "Disabled") );
  msg( DETAILS, "persistent_rice_adaptation_enabled_flag: %s\n", (m_persistentRiceAdaptationEnabledFlag     ? "Enabled" : "Disabled") );
  msg( DETAILS, "cabac_bypass_alignment_enabled_flag    : %s\n", (m_cabacBypassAlignmentEnabledFlag         ? "Enabled" : "Disabled") );
  if (m_bUseSAO)
  {
    msg( DETAILS, "log2_sao_offset_scale_luma             : %d\n", m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA] );
    msg( DETAILS, "log2_sao_offset_scale_chroma           : %d\n", m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
  }

  switch (m_costMode)
  {
    case COST_STANDARD_LOSSY:               msg( DETAILS, "Cost function:                         : Lossy coding (default)\n"); break;
    case COST_SEQUENCE_LEVEL_LOSSLESS:      msg( DETAILS, "Cost function:                         : Sequence_level_lossless coding\n"); break;
    case COST_LOSSLESS_CODING:              msg( DETAILS, "Cost function:                         : Lossless coding with fixed QP of %d\n", LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP); break;
    case COST_MIXED_LOSSLESS_LOSSY_CODING:  msg( DETAILS, "Cost function:                         : Mixed_lossless_lossy coding with QP'=%d for lossless evaluation\n", LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME); break;
    default:                                msg( DETAILS, "Cost function:                         : Unknown\n"); break;
  }

  msg( DETAILS, "RateControl                            : %d\n", m_RCEnableRateControl );
  msg( DETAILS, "WPMethod                               : %d\n", int(m_weightedPredictionMethod));

  if(m_RCEnableRateControl)
  {
    msg( DETAILS, "TargetBitrate                          : %d\n", m_RCTargetBitrate );
    msg( DETAILS, "KeepHierarchicalBit                    : %d\n", m_RCKeepHierarchicalBit );
    msg( DETAILS, "LCULevelRC                             : %d\n", m_RCLCULevelRC );
    msg( DETAILS, "UseLCUSeparateModel                    : %d\n", m_RCUseLCUSeparateModel );
    msg( DETAILS, "InitialQP                              : %d\n", m_RCInitialQP );
    msg( DETAILS, "ForceIntraQP                           : %d\n", m_RCForceIntraQP );
#if U0132_TARGET_BITS_SATURATION
    msg( DETAILS, "CpbSaturation                          : %d\n", m_RCCpbSaturationEnabled );
    if (m_RCCpbSaturationEnabled)
    {
      msg( DETAILS, "CpbSize                                : %d\n", m_RCCpbSize);
      msg( DETAILS, "InitalCpbFullness                      : %.2f\n", m_RCInitialCpbFullness);
    }
#endif
  }

  msg( DETAILS, "Max Num Merge Candidates               : %d\n", m_maxNumMergeCand );
  msg( DETAILS, "Max Num Affine Merge Candidates        : %d\n", m_maxNumAffineMergeCand );
  msg( DETAILS, "Max Num Triangle Merge Candidates      : %d\n", m_maxNumTriangleCand );
#if JVET_O0455_IBC_MAX_MERGE_NUM
  msg( DETAILS, "Max Num IBC Merge Candidates           : %d\n", m_maxNumIBCMergeCand );
#endif
  msg( DETAILS, "\n");

  msg( VERBOSE, "TOOL CFG: ");
  msg( VERBOSE, "IBD:%d ", ((m_internalBitDepth[CHANNEL_TYPE_LUMA] > m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA]) || (m_internalBitDepth[CHANNEL_TYPE_CHROMA] > m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA])));
  msg( VERBOSE, "HAD:%d ", m_bUseHADME                          );
  msg( VERBOSE, "RDQ:%d ", m_useRDOQ                            );
  msg( VERBOSE, "RDQTS:%d ", m_useRDOQTS                        );
  msg( VERBOSE, "RDpenalty:%d ", m_rdPenalty                    );
#if SHARP_LUMA_DELTA_QP
  msg( VERBOSE, "LQP:%d ", m_lumaLevelToDeltaQPMapping.mode     );
#endif
  msg( VERBOSE, "SQP:%d ", m_uiDeltaQpRD                        );
  msg( VERBOSE, "ASR:%d ", m_bUseASR                            );
  msg( VERBOSE, "MinSearchWindow:%d ", m_minSearchWindow        );
  msg( VERBOSE, "RestrictMESampling:%d ", m_bRestrictMESampling );
  msg( VERBOSE, "FEN:%d ", int(m_fastInterSearchMode)           );
  msg( VERBOSE, "ECU:%d ", m_bUseEarlyCU                        );
  msg( VERBOSE, "FDM:%d ", m_useFastDecisionForMerge            );
  msg( VERBOSE, "CFM:%d ", m_bUseCbfFastMode                    );
  msg( VERBOSE, "ESD:%d ", m_useEarlySkipDetection              );
  msg( VERBOSE, "TransformSkip:%d ",     m_useTransformSkip     );
  msg( VERBOSE, "TransformSkipFast:%d ", m_useTransformSkipFast );
  msg( VERBOSE, "TransformSkipLog2MaxSize:%d ", m_log2MaxTransformSkipBlockSize);
#if JVET_O1136_TS_BDPCM_SIGNALLING
  msg( VERBOSE, "BDPCM:%d ", m_useBDPCM                         );
#endif
  msg( VERBOSE, "Slice: M=%d ", int(m_sliceMode));
  if (m_sliceMode!=NO_SLICES)
  {
    msg( VERBOSE, "A=%d ", m_sliceArgument);
  }
  msg( VERBOSE, "Tiles:%dx%d ", m_numTileColumnsMinus1 + 1, m_numTileRowsMinus1 + 1 );
  msg( VERBOSE, "MCTS:%d ", m_MCTSEncConstraint );
  msg( VERBOSE, "CIP:%d ", m_bUseConstrainedIntraPred);
  msg( VERBOSE, "SAO:%d ", (m_bUseSAO)?(1):(0));
  msg( VERBOSE, "ALF:%d ", m_alf ? 1 : 0 );
#if !JVET_O0525_REMOVE_PCM
  msg( VERBOSE, "PCM:%d ", (m_usePCM && (1<<m_uiPCMLog2MinSize) <= m_uiMaxCUWidth)? 1 : 0);
#endif

  if (m_TransquantBypassEnabledFlag && m_CUTransquantBypassFlagForce)
  {
    msg( VERBOSE, "TransQuantBypassEnabled: =1");
  }
  else
  {
    msg( VERBOSE, "TransQuantBypassEnabled:%d ", (m_TransquantBypassEnabledFlag)? 1:0 );
  }

  msg( VERBOSE, "WPP:%d ", (int)m_useWeightedPred);
  msg( VERBOSE, "WPB:%d ", (int)m_useWeightedBiPred);
  msg( VERBOSE, "PME:%d ", m_log2ParallelMergeLevel);
  const int iWaveFrontSubstreams = m_entropyCodingSyncEnabledFlag ? (m_iSourceHeight + m_uiMaxCUHeight - 1) / m_uiMaxCUHeight : 1;
  msg( VERBOSE, " WaveFrontSynchro:%d WaveFrontSubstreams:%d", m_entropyCodingSyncEnabledFlag?1:0, iWaveFrontSubstreams);
  msg( VERBOSE, " ScalingList:%d ", m_useScalingListId );
  msg( VERBOSE, "TMVPMode:%d ", m_TMVPModeId     );

  msg( VERBOSE, " DQ:%d ", m_depQuantEnabledFlag);
  msg( VERBOSE, " SignBitHidingFlag:%d ", m_signDataHidingEnabledFlag);
  msg( VERBOSE, "RecalQP:%d ", m_recalculateQPAccordingToLambda ? 1 : 0 );

  if( m_profile == Profile::NEXT )
  {
    msg( VERBOSE, "\nNEXT TOOL CFG: " );
    msg( VERBOSE, "LFNST:%d ", m_LFNST );
    msg( VERBOSE, "MMVD:%d ", m_MMVD);
    msg( VERBOSE, "Affine:%d ", m_Affine );
    if ( m_Affine )
    {
      msg( VERBOSE, "AffineType:%d ", m_AffineType );
    }
#if JVET_O0070_PROF
    msg(VERBOSE, "PROF:%d ", m_PROF);
#endif
    msg(VERBOSE, "SubPuMvp:%d+%d ", m_SubPuMvpMode & 1, (m_SubPuMvpMode & 2) == 2);
    msg( VERBOSE, "DualITree:%d ", m_dualTree );
    msg( VERBOSE, "IMV:%d ", m_ImvMode );
    msg( VERBOSE, "BIO:%d ", m_BIO );
    msg( VERBOSE, "LMChroma:%d ", m_LMChroma );
    if( m_LMChroma && m_chromaFormatIDC == CHROMA_420 )
    {
      msg( VERBOSE, "CclmCollocatedChroma:%d ", m_cclmCollocatedChromaFlag );
    }
    msg( VERBOSE, "MTS: %1d(intra) %1d(inter) ", m_MTS & 1, ( m_MTS >> 1 ) & 1 );
    msg( VERBOSE, "SBT:%d ", m_SBT );
    msg( VERBOSE, "ISP:%d ", m_ISP );
    msg( VERBOSE, "SMVD:%d ", m_SMVD );
    msg( VERBOSE, "CompositeLTReference:%d ", m_compositeRefEnabled);
    msg( VERBOSE, "GBi:%d ", m_GBi );
    msg( VERBOSE, "GBiFast:%d ", m_GBiFast );
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
    msg( VERBOSE, "LADF:%d ", m_LadfEnabed );
#endif
    msg(VERBOSE, "MHIntra:%d ", m_MHIntra);
    msg( VERBOSE, "Triangle:%d ", m_Triangle );
    m_allowDisFracMMVD = m_MMVD ? m_allowDisFracMMVD : false;
    if ( m_MMVD )
      msg(VERBOSE, "AllowDisFracMMVD:%d ", m_allowDisFracMMVD);
    msg( VERBOSE, "AffineAmvr:%d ", m_AffineAmvr );
    m_AffineAmvrEncOpt = m_AffineAmvr ? m_AffineAmvrEncOpt : false;
    msg( VERBOSE, "AffineAmvrEncOpt:%d ", m_AffineAmvrEncOpt );
    msg(VERBOSE, "DMVR:%d ", m_DMVR);
    msg(VERBOSE, "MmvdDisNum:%d ", m_MmvdDisNum);
#if !JVET_O1136_TS_BDPCM_SIGNALLING    
    msg(VERBOSE, "RDPCM:%d ", m_RdpcmMode );
#endif    
#if JVET_O0376_SPS_JOINTCBCR_FLAG
    msg(VERBOSE, "JointCbCr:%d ", m_JointCbCrMode);
#endif
  }
#if JVET_O0119_BASE_PALETTE_444
    m_PLTMode = ( m_chromaFormatIDC == CHROMA_444) ? m_PLTMode : 0u;
    msg(VERBOSE, "PLT:%d ", m_PLTMode);
#endif
    msg(VERBOSE, "IBC:%d ", m_IBCMode);
  msg( VERBOSE, "HashME:%d ", m_HashME );
  msg( VERBOSE, "WrapAround:%d ", m_wrapAround);
  if( m_wrapAround )
  {
    msg( VERBOSE, "WrapAroundOffset:%d ", m_wrapAroundOffset );
  }
  // ADD_NEW_TOOL (add some output indicating the usage of tools)
  msg(VERBOSE, "LoopFilterAcrossVirtualBoundaries:%d ", m_loopFilterAcrossVirtualBoundariesDisabledFlag);
  if ( m_loopFilterAcrossVirtualBoundariesDisabledFlag )
  {
    msg(VERBOSE, "vertical virtual boundaries:[");
    for (unsigned i = 0; i < m_numVerVirtualBoundaries; i++)
    {
      msg(VERBOSE, " %d", m_virtualBoundariesPosX[i]);
    }
    msg(VERBOSE, " ] horizontal virtual boundaries:[");
    for (unsigned i = 0; i < m_numHorVirtualBoundaries; i++)
    {
      msg(VERBOSE, " %d", m_virtualBoundariesPosY[i]);
    }
    msg(VERBOSE, " ] ");
  }
    msg(VERBOSE, "Reshape:%d ", m_lumaReshapeEnable);
    if (m_lumaReshapeEnable)
    {
#if JVET_O0432_LMCS_ENCODER
      msg(VERBOSE, "(Signal:%s ", m_reshapeSignalType == 0 ? "SDR" : (m_reshapeSignalType == 2 ? "HDR-HLG" : "HDR-PQ"));
      msg(VERBOSE, "Opt:%d", m_adpOption);
      if (m_adpOption > 0) { msg(VERBOSE, " CW:%d", m_initialCW); }
#else
      msg(VERBOSE, "(Sigal:%s ", m_reshapeSignalType==0? "SDR" : "HDR-PQ");
#endif
      msg(VERBOSE, ") ");
    }
    msg(VERBOSE, "MIP:%d ", m_MIP);
    msg(VERBOSE, "EncDbOpt:%d ", m_encDbOpt);
  msg( VERBOSE, "\nFAST TOOL CFG: " );
  msg( VERBOSE, "LCTUFast:%d ", m_useFastLCTU );
  msg( VERBOSE, "FastMrg:%d ", m_useFastMrg );
  msg( VERBOSE, "PBIntraFast:%d ", m_usePbIntraFast );
  if( m_ImvMode ) msg( VERBOSE, "IMV4PelFast:%d ", m_Imv4PelFast );
  if( m_MTS ) msg( VERBOSE, "MTSMaxCand: %1d(intra) %1d(inter) ", m_MTSIntraMaxCand, m_MTSInterMaxCand );
  if( m_ISP ) msg( VERBOSE, "ISPFast:%d ", m_useFastISP );
  if( m_LFNST ) msg( VERBOSE, "FastLFNST:%d ", m_useFastLFNST );
  msg( VERBOSE, "AMaxBT:%d ", m_useAMaxBT );
  msg( VERBOSE, "E0023FastEnc:%d ", m_e0023FastEnc );
  msg( VERBOSE, "ContentBasedFastQtbt:%d ", m_contentBasedFastQtbt );
  msg( VERBOSE, "UseNonLinearAlfLuma:%d ", m_useNonLinearAlfLuma );
  msg( VERBOSE, "UseNonLinearAlfChroma:%d ", m_useNonLinearAlfChroma );
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB
  msg( VERBOSE, "MaxNumAlfAlternativesChroma:%d ", m_maxNumAlfAlternativesChroma );
#endif
  if( m_MIP ) msg(VERBOSE, "FastMIP:%d ", m_useFastMIP);
#if JVET_O0050_LOCAL_DUAL_TREE
  msg( VERBOSE, "FastLocalDualTree:%d ", m_fastLocalDualTreeMode );
#endif

  msg( VERBOSE, "NumSplitThreads:%d ", m_numSplitThreads );
  if( m_numSplitThreads > 1 )
  {
    msg( VERBOSE, "ForceSingleSplitThread:%d ", m_forceSplitSequential );
  }
  msg( VERBOSE, "NumWppThreads:%d+%d ", m_numWppThreads, m_numWppExtraLines );
  msg( VERBOSE, "EnsureWppBitEqual:%d ", m_ensureWppBitEqual );

#if JVET_O1164_RPR
  if( m_rprEnabled )
  {
    msg( VERBOSE, "RPR:(%1.2lfx, %1.2lfx)|%d", m_scalingRatioHor, m_scalingRatioVer, m_switchPocPeriod );
  }
  else
  {
    msg( VERBOSE, "RPR:%d", 0 );
  }
#endif

#if EXTENSION_360_VIDEO
  m_ext360.outputConfigurationSummary();
#endif

  msg( VERBOSE, "\n\n");

  msg( NOTICE, "\n");

  fflush( stdout );
}

bool confirmPara(bool bflag, const char* message)
{
  if (!bflag)
  {
    return false;
  }

  msg( ERROR, "Error: %s\n",message);
  return true;
}

//! \}
