/**********************************************************************

  Audacity: A Digital Audio Editor

  ExportDSPADPCM.cpp

  Jack Andersen

**********************************************************************/

#include <string>
#include <wx/defs.h>
#include <wx/choice.h>
#include <wx/dynlib.h>
#include <wx/filename.h>
#include <wx/intl.h>
#include <wx/timer.h>
#include <wx/msgdlg.h>
#include <wx/progdlg.h>
#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/window.h>
#include <wx/arrstr.h>

#include "../Audacity.h"
#include "../FileFormats.h"
#include "../Internat.h"
#include "../LabelTrack.h"
#include "../Mix.h"
#include "../Prefs.h"
#include "../Project.h"
#include "../Tags.h"
#include "../Track.h"
#include "../WaveTrack.h"
#include "../ondemand/ODManager.h"

#include "Export.h"
#include "ExportDSPADPCM.h"

//----------------------------------------------------------------------------
// DSPADPCM Internals
//----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <string.h>

#ifndef __BIG_ENDIAN__

static inline uint32_t bswapu32(uint32_t val)
{
#if __GNUC__
    return __builtin_bswap32(val);
#elif _WIN32
    return _byteswap_ulong(val);
#else
    val = (val & 0x0000FFFF) << 16 | (val & 0xFFFF0000) >> 16;
    val = (val & 0x00FF00FF) << 8 | (val & 0xFF00FF00) >> 8;
    return val;
#endif
}

static inline int16_t bswap16(int16_t val)
{
#if __GNUC__
    return __builtin_bswap16(val);
#elif _WIN32
    return _byteswap_ushort(val);
#else
    return (val = (val << 8) | ((val >> 8) & 0xFF));
#endif
}

#else

#define bswapu32(val) (val)
#define bswap16(val) (val)

#endif

static inline unsigned sampleidx_to_nibbleidx(unsigned sampleidx)
{
    unsigned frame = sampleidx / 14;
    unsigned frame_samp = sampleidx % 14;
    return frame * 16 + frame_samp + 2;
}

static inline unsigned sampleidx_to_byteidx(unsigned sampleidx)
{
    unsigned frame = sampleidx / 14;
    unsigned frame_samp = sampleidx % 14;
    return frame * 8 + frame_samp / 2 + 1;
}

/* Standard DSPADPCM header */
struct dspadpcm_header
{
    uint32_t num_samples;
    uint32_t num_nibbles;
    uint32_t sample_rate;
    uint16_t loop_flag;
    uint16_t format; /* 0 for ADPCM */
    uint32_t loop_start_nibble;
    uint32_t loop_end_nibble;
    uint32_t zero;
    int16_t coef[16];
    int16_t gain;
    int16_t ps;
    int16_t hist1;
    int16_t hist2;
    int16_t loop_ps;
    int16_t loop_hist1;
    int16_t loop_hist2;
    uint16_t pad[11];
};

/* RS03 Header (following magic) */
struct rs03_header
{
    uint32_t chan_count;
    uint32_t num_samples;
    uint32_t sample_rate;
    uint32_t chan_byte_count;
    uint16_t loop_flag;
    uint16_t format;
    uint32_t loop_start;
    uint32_t loop_end;
    int16_t coefs[2][16];
};

/* FSB Header (following magic) */
struct fsb3_header
{
    uint32_t sampleCount;
    uint32_t headerSize;
    uint32_t dataSize;
    uint32_t version;
    uint32_t mode;
};

/* FSB3.1 Sample Header */
#define FSB_MAX_NAME_LENGTH 30
#define FSOUND_LOOP_NORMAL 0x00000002
#define FSOUND_GCADPCM 0x02000000
struct fsb31_sample_header
{
    uint16_t size;
    char     name[FSB_MAX_NAME_LENGTH];
    uint32_t sampleCount;
    uint32_t dataSize;
    uint32_t loopStart;
    uint32_t loopEnd;
    uint32_t mode;
    uint32_t defaultFreq;
    uint16_t defVol;
    int16_t  defPan;
    uint16_t defPri;
    uint16_t channelCount;
    float    minDistance;
    float    maxDistance;
    uint32_t varFreg;
    uint16_t varVol;
    int16_t  varPan;
};

struct fsb_dspadpcm_channel
{
    int16_t coef[16];
    char padding[14];
};

/* RAS header after magic (DKC Tropical Freeze) */
struct ras_header
{
    uint32_t chanCount1;
    uint32_t chanCount2;
    uint32_t numSamples;
    uint32_t metaDataFlag;
    uint32_t sampleRate;
    uint32_t dataOffset;
    uint32_t dataSize;
    uint32_t blockSize;
    uint32_t numBlocks;
    uint32_t startSample;
    uint32_t lastSampleOfLastBlock;
    uint32_t loopStartBlock;
    uint32_t loopStartSample;
    uint32_t loopEndBlock;
    uint32_t loopEndSample;
};

static void SwapRASHeader(ras_header* h)
{
    h->chanCount1 = bswapu32(h->chanCount1);
    h->chanCount2 = bswapu32(h->chanCount2);
    h->numSamples = bswapu32(h->numSamples);
    h->metaDataFlag = bswapu32(h->metaDataFlag);
    h->sampleRate = bswapu32(h->sampleRate);
    h->dataOffset = bswapu32(h->dataOffset);
    h->dataSize = bswapu32(h->dataSize);
    h->blockSize = bswapu32(h->blockSize);
    h->numBlocks = bswapu32(h->numBlocks);
    h->startSample = bswapu32(h->startSample);
    h->lastSampleOfLastBlock = bswapu32(h->lastSampleOfLastBlock);
    h->loopStartBlock = bswapu32(h->loopStartBlock);
    h->loopStartSample = bswapu32(h->loopStartSample);
    h->loopEndBlock = bswapu32(h->loopEndBlock);
    h->loopEndSample = bswapu32(h->loopEndSample);
}

struct ras_dspadpcm_channel
{
    int16_t coefs[16];
    uint32_t unknowns[4];
};

static void SwapRASChannel(ras_dspadpcm_channel* h)
{
    for (int i=0 ; i<16 ; ++i)
        h->coefs[i] = bswap16(h->coefs[i]);
    for (int i=0 ; i<4 ; ++i)
        h->unknowns[i] = bswapu32(h->unknowns[i]);
}

struct ras_track_meta
{
    uint32_t unknown1;
    uint32_t bpmFlag;
    float bpm;
    uint32_t unknowns2[5];
};

static void SwapRASTrackMeta(ras_track_meta* h)
{
    h->unknown1 = bswapu32(h->unknown1);
    h->bpmFlag = bswapu32(h->bpmFlag);
    *((uint32_t*)&h->bpm) = bswapu32(*((uint32_t*)&h->bpm));
    for (int i=0 ; i<5 ; ++i)
        h->unknowns2[i] = bswapu32(h->unknowns2[i]);
}

typedef unsigned char TADPCMFrame[8];
typedef unsigned char TADPCMStereoFrame[4][2][2];

/* Reference:
 * https://code.google.com/p/brawltools/source/browse/trunk/BrawlLib/Wii/Audio/AudioConverter.cs
 */

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/* Temporal Vector
 * A contiguous history of 3 samples starting with
 * 'current' and going 2 backwards
 */
typedef double tvec[3];

static inline void InnerProductMerge(tvec vecOut, short pcmBuf[14])
{
    for (int i=0 ; i<=2 ; i++)
    {
        vecOut[i] = 0.0f;
        for (int x=0 ; x<14 ; x++)
            vecOut[i] -= pcmBuf[x-i] * pcmBuf[x];
    }
}

static inline void OuterProductMerge(tvec mtxOut[3], short pcmBuf[14])
{
    for (int x=1 ; x<=2 ; x++)
        for (int y=1 ; y<=2 ; y++)
        {
            mtxOut[x][y] = 0.0;
            for (int z=0 ; z<14 ; z++)
                mtxOut[x][y] += pcmBuf[z-x] * pcmBuf[z-y];
        }
}

static bool AnalyzeRanges(tvec mtx[3], int* vecIdxsOut)
{
    double recips[3];
    double val, tmp, min, max;

    /* Get greatest distance from zero */
    for (int x=1 ; x<=2 ; x++)
    {
        val = MAX(fabs(mtx[x][1]), fabs(mtx[x][2]));
        if (val < DBL_EPSILON)
            return true;

        recips[x] = 1.0 / val;
    }

    int maxIndex = 0;
    for (int i=1 ; i<=2 ; i++)
    {
        for (int x=1 ; x<i ; x++)
        {
            tmp = mtx[x][i];
            for (int y=1 ; y<x ; y++)
                tmp -= mtx[x][y] * mtx[y][i];
            mtx[x][i] = tmp;
        }

        val = 0.0;
        for (int x=i ; x<=2 ; x++)
        {
            tmp = mtx[x][i];
            for (int y=1 ; y<i ; y++)
                tmp -= mtx[x][y] * mtx[y][i];

            mtx[x][i] = tmp;
            tmp = fabs(tmp) * recips[x];
            if (tmp >= val)
            {
                val = tmp;
                maxIndex = x;
            }
        }

        if (maxIndex == i)
        {
            for (int y=1 ; y<=2 ; y++)
            {
                tmp = mtx[maxIndex][y];
                mtx[maxIndex][y] = mtx[i][y];
                mtx[i][y] = tmp;
            }
            recips[maxIndex] = recips[i];
        }

        vecIdxsOut[i] = maxIndex;

        if (mtx[i][i] == 0.0)
            return true;

        if (i != 2)
        {
            tmp = 1.0 / mtx[i][i];
            for (int x=i+1 ; x<=2 ; x++)
                mtx[x][i] *= tmp;
        }
    }

    /* Get range */
    min = 1.0e10;
    max = 0.0;
    for (int i=1 ; i<=2 ; i++)
    {
        tmp = fabs(mtx[i][i]);
        if (tmp < min)
            min = tmp;
        if (tmp > max)
            max = tmp;
    }

    if (min / max < 1.0e-10)
        return true;

    return false;
}

static void BidirectionalFilter(tvec mtx[3], int* vecIdxs, tvec vecOut)
{
    double tmp;

    for (int i=1, x=0 ; i<=2 ; i++)
    {
        int index = vecIdxs[i];
        tmp = vecOut[index];
        vecOut[index] = vecOut[i];
        if (x != 0)
            for (int y=x ; y<=i-1 ; y++)
                tmp -= vecOut[y] * mtx[i][y];
        else if (tmp != 0.0)
            x = i;
        vecOut[i] = tmp;
    }

    for (int i=2 ; i>0 ; i--)
    {
        tmp = vecOut[i];
        for (int y=i+1 ; y<=2 ; y++)
            tmp -= vecOut[y] * mtx[i][y];
        vecOut[i] = tmp / mtx[i][i];
    }

    vecOut[0] = 1.0;
}

static bool QuadraticMerge(tvec inOutVec)
{
    double v0, v1, v2 = inOutVec[2];
    double tmp = 1.0 - (v2 * v2);

    if (tmp == 0.0)
        return true;

    v0 = (inOutVec[0] - (v2 * v2)) / tmp;
    v1 = (inOutVec[1] - (inOutVec[1] * v2)) / tmp;

    inOutVec[0] = v0;
    inOutVec[1] = v1;

    return fabs(v1) > 1.0;
}

static void FinishRecord(tvec in, tvec out)
{
    for (int z=1 ; z<=2 ; z++)
    {
        if (in[z] >= 1.0)
            in[z] = 0.9999999999;
        else if (in[z] <= -1.0)
            in[z] = -0.9999999999;
    }
    out[0] = 1.0;
    out[1] = (in[2] * in[1]) + in[1];
    out[2] = in[2];
}

static void MatrixFilter(tvec src, tvec dst)
{
    tvec mtx[3];

    mtx[2][0] = 1.0;
    for (int i=1 ; i<=2 ; i++)
        mtx[2][i] = -src[i];

    for (int i=2 ; i>0 ; i--)
    {
        double val = 1.0 - (mtx[i][i] * mtx[i][i]);
        for (int y = 1; y <= i; y++)
            mtx[i-1][y] = ((mtx[i][i] * mtx[i][y]) + mtx[i][y]) / val;
    }

    dst[0] = 1.0;
    for (int i=1 ; i<=2 ; i++)
    {
        dst[i] = 0.0;
        for (int y=1 ; y<=i ; y++)
            dst[i] += mtx[i][y] * dst[i-y];
    }
}

static void MergeFinishRecord(tvec src, tvec dst)
{
    tvec tmp;
    double val = src[0];

    dst[0] = 1.0;
    for (int i=1 ; i<=2 ; i++)
    {
        double v2 = 0.0;
        for (int y=1 ; y<i ; y++)
            v2 += dst[y] * src[i-y];

        if (val > 0.0)
            dst[i] = -(v2 + src[i]) / val;
        else
            dst[i] = 0.0;

        tmp[i] = dst[i];

        for (int y=1 ; y<i ; y++)
            dst[y] += dst[i] * dst[i - y];

        val *= 1.0 - (dst[i] * dst[i]);
    }

    FinishRecord(tmp, dst);
}

static double ContrastVectors(tvec source1, tvec source2)
{
    double val = (-source2[2] * -source2[1] + -source2[1]) / (1.0 - source2[2] * source2[2]);
    double val1 = (source1[0] * source1[0]) + (source1[1] * source1[1]) + (source1[2] * source1[2]);
    double val2 = (source1[0] * source1[1]) + (source1[1] * source1[2]);
    double val3 = source1[0] * source1[2];
    return val1 + (2.0 * val * val2) + (2.0 * (-source2[1] * val + -source2[2]) * val3);
}

static void FilterRecords(tvec vecBest[8], int exp, tvec records[], int recordCount)
{
    tvec bufferList[8];

    int buffer1[8];
    tvec buffer2;

    int index;
    double value, tempVal = 0;

    for (int x=0 ; x<2 ; x++)
    {
        for (int y=0 ; y<exp ; y++)
        {
            buffer1[y] = 0;
            for (int i=0 ; i<=2 ; i++)
                bufferList[y][i] = 0.0;
        }
        for (int z=0 ; z<recordCount ; z++)
        {
            index = 0;
            value= 1.0e30;
            for (int i=0 ; i<exp ; i++)
            {
                tempVal = ContrastVectors(vecBest[i], records[z]);
                if (tempVal < value)
                {
                    value = tempVal;
                    index = i;
                }
            }
            buffer1[index]++;
            MatrixFilter(records[z], buffer2);
            for (int i=0 ; i<=2 ; i++)
                bufferList[index][i] += buffer2[i];
        }

        for (int i=0 ; i<exp ; i++)
            if (buffer1[i] > 0)
                for (int y=0 ; y<=2 ; y++)
                    bufferList[i][y] /= buffer1[i];

        for (int i=0 ; i<exp ; i++)
            MergeFinishRecord(bufferList[i], vecBest[i]);
    }
}

static void DSPCorrelateCoefs(const short* source, int samples, short* coefsOut)
{
    int numFrames = (samples + 13) / 14;
    int frameSamples;

    short* blockBuffer = (short*)calloc(sizeof(short), 0x3800);
    short pcmHistBuffer[2][14] = {};

    tvec vec1;
    tvec vec2;

    tvec mtx[3];
    int vecIdxs[3];

    tvec* records = (tvec*)calloc(sizeof(tvec), numFrames * 2);
    int recordCount = 0;

    tvec vecBest[8];

    /* Iterate though 1024-block frames */
    for (int x=samples ; x>0 ;)
    {
        if (x > 0x3800) /* Full 1024-block frame */
        {
            frameSamples = 0x3800;
            x -= 0x3800;
        }
        else /* Partial frame */
        {
            /* Zero lingering block samples */
            frameSamples = x;
            for (int z=0 ; z<14 && z+frameSamples<0x3800 ; z++)
                blockBuffer[frameSamples+z] = 0;
            x = 0;
        }

        /* Copy (potentially non-frame-aligned PCM samples into aligned buffer) */
        memcpy(blockBuffer, source, frameSamples * sizeof(short));
        source += frameSamples;


        for (int i=0 ; i<frameSamples ;)
        {
            for (int z=0 ; z<14 ; z++)
                pcmHistBuffer[0][z] = pcmHistBuffer[1][z];
            for (int z=0 ; z<14 ; z++)
                pcmHistBuffer[1][z] = blockBuffer[i++];

            InnerProductMerge(vec1, pcmHistBuffer[1]);
            if (fabs(vec1[0]) > 10.0)
            {
                OuterProductMerge(mtx, pcmHistBuffer[1]);
                if (!AnalyzeRanges(mtx, vecIdxs))
                {
                    BidirectionalFilter(mtx, vecIdxs, vec1);
                    if (!QuadraticMerge(vec1))
                    {
                        FinishRecord(vec1, records[recordCount]);
                        recordCount++;
                    }
                }
            }
        }
    }

    vec1[0] = 1.0;
    vec1[1] = 0.0;
    vec1[2] = 0.0;

    for (int z=0 ; z<recordCount ; z++)
    {
        MatrixFilter(records[z], vecBest[0]);
        for (int y=1 ; y<=2 ; y++)
            vec1[y] += vecBest[0][y];
    }
    for (int y=1 ; y<=2 ; y++)
        vec1[y] /= recordCount;

    MergeFinishRecord(vec1, vecBest[0]);


    int exp = 1;
    for (int w=0 ; w<3 ;)
    {
        vec2[0] = 0.0;
        vec2[1] = -1.0;
        vec2[2] = 0.0;
        for (int i=0 ; i<exp ; i++)
            for (int y=0 ; y<=2 ; y++)
                vecBest[exp+i][y] = (0.01 * vec2[y]) + vecBest[i][y];
        ++w;
        exp = 1 << w;
        FilterRecords(vecBest, exp, records, recordCount);
    }

    /* Write output */
    for (int z=0 ; z<8 ; z++)
    {
        double d;
        d = -vecBest[z][1] * 2048.0;
        if (d > 0.0)
            coefsOut[z*2] = (d > 32767.0) ? (short)32767 : (short)lround(d);
        else
            coefsOut[z*2] = (d < -32768.0) ? (short)-32768 : (short)lround(d);

        d = -vecBest[z][2] * 2048.0;
        if (d > 0.0)
            coefsOut[z*2+1] = (d > 32767.0) ? (short)32767 : (short)lround(d);
        else
            coefsOut[z*2+1] = (d < -32768.0) ? (short)-32768 : (short)lround(d);
    }

    /* Free memory */
    free(records);
    free(blockBuffer);

}

//Make sure source includes the yn values (16 samples total)
static void DSPEncodeFrame(short* pcmInOut, int sampleCount, unsigned char* adpcmOut, const short* coefsIn)
{
    int buffer1[128];
    int buffer2[112];

    unsigned bestDistance = ~0;
    unsigned distAccum;
    int bestIndex = 0;
    int bestScale = 0;

    int distance, index, scale;

    int *p1, *p2, *t1, *t2;
    short* sPtr;
    int v1, v2, v3;

    /* Iterate through each coef set, finding the set with the smallest error */
    p1 = buffer1;
    p2 = buffer2;
    for (int i=0 ; i<8 ; i++, p1+=16, p2+=14, coefsIn+=2)
    {
        /* Set yn values */
        t1 = p1;
        *t1++ = pcmInOut[0];
        *t1++ = pcmInOut[1];

        /* Round and clamp samples for this coef set */
        distance = 0;
        sPtr = pcmInOut;
        for (int y=0 ; y<sampleCount ; y++)
        {
            /* Multiply previous samples by coefs */
            *t1++ = v1 = ((sPtr[0] * coefsIn[1]) + (sPtr[1] * coefsIn[0])) >> 11;
            /* Subtract from current sample */
            v2 = sPtr[2] - v1;
            /* Clamp */
            v3 = (v2 >= 32767) ? 32767 : (v2 <= -32768) ? -32768 : v2;
            /* Compare distance */
            if (abs(v3) > abs(distance))
                distance = v3;

            sPtr += 1;
        }

        /* Set initial scale */
        for (scale=0 ; (scale<=12) && ((distance>7) || (distance<-8)); scale++, distance>>=1) ;
        scale = (scale <= 1) ? -1 : scale - 2;

        do
        {
            scale++;
            distAccum = 0;
            index = 0;

            t1 = p1;
            t2 = p2;
            sPtr = pcmInOut + 2;
            for (int y=0 ; y<sampleCount ; y++)
            {
                /* Multiply previous */
                v1 = ((t1[0] * coefsIn[1]) + (t1[1] * coefsIn[0]));
                /* Evaluate from real sample */
                v2 = ((*sPtr << 11) - v1) / 2048;
                /* Round to nearest sample */
                v3 = (v2 > 0) ? (int)((double)v2 / (1 << scale) + 0.4999999f) : (int)((double)v2 / (1 << scale) - 0.4999999f);

                /* Clamp sample and set index */
                if (v3 < -8)
                {
                    if (index < (v3 = -8 - v3))
                        index = v3;
                    v3 = -8;
                }
                else if (v3 > 7)
                {
                    if (index < (v3 -= 7))
                        index = v3;
                    v3 = 7;
                }

                /* Store result */
                *t2++ = v3;

                /* Round and expand */
                v1 = (v1 + ((v3 * (1 << scale)) << 11) + 1024) >> 11;
                /* Clamp and store */
                t1[2] = v2 = (v1 >= 32767) ? 32767 : (v1 <= -32768) ? -32768 : v1;
                /* Accumulate distance */
                v3 = *sPtr++ - v2;
                distAccum += v3 * v3;

                t1 += 1;

                /* Break if we're higher than a previous search */
                if (distAccum >= bestDistance)
                    break;
            }

            for (int x=index+8 ; x>256 ; x>>=1)
                if (++scale >= 12)
                    scale = 11;

        } while ((scale < 12) && (index > 1));

        if (distAccum < bestDistance)
        {
            bestDistance = distAccum;
            bestIndex = i;
            bestScale = scale;
        }
    }

    p1 = buffer1 + (bestIndex * 16) + 2;
    p2 = buffer2 + (bestIndex * 14);

    /* Write converted samples */
    sPtr = pcmInOut + 2;
    for (int i=0 ; i<sampleCount ; i++)
        *sPtr++ = (short)*p1++;

    /* Write ps */
    *adpcmOut++ = (char)((bestIndex << 4) | (bestScale & 0xF));

    /* Zero remaining samples */
    for (int i=sampleCount ; i<14 ; i++)
        p2[i] = 0;

    /* Write output samples */
    for (int y=0 ; y++<7 ;)
    {
        *adpcmOut++ = (char)((p2[0] << 4) | (p2[1] & 0xF));
        p2 += 2;
    }
}

//----------------------------------------------------------------------------
// Begin Plugin
//----------------------------------------------------------------------------

struct SLayouts
{
    int fmt;
    wxChar* name;
    wxChar* desc;
};

enum EDspLayout
{
    DSPADPCM_STD = 0,
    DSPADPCM_RS03 = 1,
    DSPADPCM_LAYOUTMAX = 2
};

static const SLayouts kDspLayouts[] =
{
{
    DSPADPCM_STD,
    wxT("Standard Mono"),
    wxT("Standard mono .dsp layout from Nintendo's DSPADPCM.EXE\n\nStereo projects export a L/R pair")
},
{
    DSPADPCM_RS03,
    wxT("RS03 Stereo"),
    wxT("Block-interleaved RS03 layout used in Metroid Prime 2")
}
};

enum ECsmpLayout
{
    DSPADPCM_CSMP_MP3 = 0,
    DSPADPCM_CSMP_DKCTF = 1,
    DSPADPCM_CSMP_LAYOUTMAX = 2
};

static const SLayouts kCsmpLayouts[] =
{
{
    DSPADPCM_CSMP_MP3,
    wxT("MP3 / DKCR"),
    wxT("CSMP resource from Metroid Prime 3 or Donkey Kong Country Returns")
},
{
    DSPADPCM_CSMP_DKCTF,
    wxT("DKCTF"),
    wxT("CSMP resource from Donkey Kong Country Tropical Freeze")
}
};

//----------------------------------------------------------------------------
// User Prefs
//----------------------------------------------------------------------------

static int ReadExportDSPLayoutPref()
{
    return gPrefs->Read(wxT("/FileFormats/ExportFormat_DSPADPCM"),
                        (long int)0);
}

static void WriteExportDSPLayoutPref(int format)
{
    gPrefs->Write(wxT("/FileFormats/ExportFormat_DSPADPCM"), (long int)format);
    gPrefs->Flush();
}

static int ReadExportCSMPLayoutPref()
{
    return gPrefs->Read(wxT("/FileFormats/ExportFormat_DSPADPCM_CSMP"),
                        (long int)0);
}

static void WriteExportCSMPLayoutPref(int format)
{
    gPrefs->Write(wxT("/FileFormats/ExportFormat_DSPADPCM_CSMP"), (long int)format);
    gPrefs->Flush();
}

//----------------------------------------------------------------------------
// ExportDSPADPCMOptions Class
//----------------------------------------------------------------------------

#define DESC_WRAP 250
#define ID_LAYOUT_CHOICE         7501

class ExportDSPADPCMOptionsBase : public wxDialog
{
public:
    virtual int GetUserPref()=0;
    virtual void SetUserPref(int pref)=0;

    ExportDSPADPCMOptionsBase(const SLayouts* layouts, int layoutCount, int defaultLayout,
                              const wxString& title=wxString(_("DSPADPCM Options")))
    : wxDialog(NULL, wxID_ANY, title), mLayouts(layouts), mLayoutCount(layoutCount)
    {
        SetName(GetTitle());

        mOk = NULL;

        mLayoutFromChoice = defaultLayout;
        if (mLayoutFromChoice < 0 || mLayoutFromChoice >= layoutCount)
            mLayoutFromChoice = 0;

        for (int i=0 ; i<layoutCount; ++i)
            mLayoutNames.Add(layouts[i].name);

        ShuttleGui S(this, eIsCreatingFromPrefs);
        PopulateOrExchange(S);

        Layout();
        Fit();
        Center();
    }

    void PopulateOrExchange(ShuttleGui & S)
    {
        S.StartHorizontalLay(wxEXPAND, true);
        {
            S.StartStatic(_("DSPADPCM Layout Option"), true);
            {
                S.StartMultiColumn(2, wxEXPAND);
                {
                    S.SetStretchyCol(1);
                    mLayoutChoice = S.Id(ID_LAYOUT_CHOICE)
                                     .AddChoice(_("Layout:"),
                                                mLayoutNames[mLayoutFromChoice],
                                                &mLayoutNames);
                }
                S.EndMultiColumn();
                mDesc = S.AddVariableText(mLayouts[mLayoutFromChoice].desc);
                mDesc->Wrap(DESC_WRAP);
            }
            S.EndStatic();
        }
        S.EndHorizontalLay();

        S.AddStandardButtons();
        mOk = (wxButton *)wxWindow::FindWindowById(wxID_OK, this);
    }

    void OnLayoutChoice(wxCommandEvent & evt)
    {
        int choice = mLayoutChoice->GetSelection();
        if (choice >= 0 && choice < mLayoutCount)
        {
            mDesc->SetLabel(mLayouts[choice].desc);
            mDesc->Wrap(DESC_WRAP);
            SetSizerAndFit(GetSizer());
        }
    }

    void OnOK(wxCommandEvent& event)
    {
        SetUserPref(mLayoutChoice->GetSelection());
        EndModal(wxID_OK);
    }

private:
    const SLayouts* mLayouts;
    int mLayoutCount;
    wxArrayString mLayoutNames;
    wxChoice *mLayoutChoice;
    wxStaticText *mDesc;
    wxButton *mOk;
    int mLayoutFromChoice;

    DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(ExportDSPADPCMOptionsBase, wxDialog)
EVT_CHOICE(ID_LAYOUT_CHOICE,   ExportDSPADPCMOptionsBase::OnLayoutChoice)
EVT_BUTTON(wxID_OK,            ExportDSPADPCMOptionsBase::OnOK)
END_EVENT_TABLE()

class ExportDSPADPCMOptionsDsp : public ExportDSPADPCMOptionsBase
{
public:
    ExportDSPADPCMOptionsDsp()
    : ExportDSPADPCMOptionsBase(kDspLayouts, DSPADPCM_LAYOUTMAX, ReadExportDSPLayoutPref(), wxString(_("DSP Options"))) {}
    int GetUserPref() {return ReadExportDSPLayoutPref();}
    void SetUserPref(int pref) {WriteExportDSPLayoutPref(pref);}
};

class ExportDSPADPCMOptionsCsmp : public ExportDSPADPCMOptionsBase
{
public:
    ExportDSPADPCMOptionsCsmp()
    : ExportDSPADPCMOptionsBase(kCsmpLayouts, DSPADPCM_CSMP_LAYOUTMAX, ReadExportCSMPLayoutPref(), wxString(_("CSMP Options"))) {}
    int GetUserPref() {return ReadExportCSMPLayoutPref();}
    void SetUserPref(int pref) {WriteExportCSMPLayoutPref(pref);}
};

//----------------------------------------------------------------------------
// ExportDSPADPCM Class
//----------------------------------------------------------------------------

class ExportDSPADPCM : public ExportPlugin
{
    int mDspFormat;
    int mCsmpFormat;
    int mFsbFormat;
    int mRasFormat;
public:
    ExportDSPADPCM();
    void Destroy();
    bool DisplayOptions(wxWindow *parent, int subfmt = 0);
    int Export(AudacityProject *project,
               int channels,
               wxString fName,
               bool selectedOnly,
               double t0,
               double t1,
               MixerSpec *mixerSpec = NULL,
               Tags *metadata = NULL,
               int subformat = 0);

private:
    int ExportStandard(AudacityProject *project,
                       int numChannels,
                       const wxString& fName,
                       bool selectionOnly,
                       double t0,
                       double t1,
                       MixerSpec *mixerSpec,
                       bool csmp, ECsmpLayout csmpLayout);
    int ExportRS03(AudacityProject *project,
                   int numChannels,
                   const wxString& fName,
                   bool selectionOnly,
                   double t0,
                   double t1,
                   MixerSpec *mixerSpec);
    int ExportFSB31(AudacityProject *project,
                    int numChannels,
                    const wxString& fName,
                    bool selectionOnly,
                    double t0,
                    double t1,
                    MixerSpec *mixerSpec);
    int ExportRAS(AudacityProject *project,
                  int numChannels,
                  const wxString& fName,
                  bool selectionOnly,
                  double t0,
                  double t1,
                  MixerSpec *mixerSpec);
};

ExportDSPADPCM::ExportDSPADPCM()
    :  ExportPlugin()
{
    mDspFormat = AddFormat() - 1;
    SetFormat(wxT("DSP"), mDspFormat);
    SetCanMetaData(false, mDspFormat);
    SetDescription(wxT("Nintendo GameCube DSPADPCM (.dsp)"), mDspFormat);
    SetMaxChannels(2, mDspFormat);
    AddExtension(wxT("dsp"), mDspFormat);

    mCsmpFormat = AddFormat() - 1;
    SetFormat(wxT("CSMP"), mCsmpFormat);
    SetCanMetaData(false, mCsmpFormat);
    SetDescription(wxT("Nintendo GameCube DSPADPCM (.csmp)"), mCsmpFormat);
    SetMaxChannels(2, mCsmpFormat);
    AddExtension(wxT("csmp"), mCsmpFormat);

    mFsbFormat = AddFormat() - 1;
    SetFormat(wxT("FSB"), mFsbFormat);
    SetCanMetaData(false, mFsbFormat);
    SetDescription(wxT("Nintendo GameCube DSPADPCM (.fsb,.strm)"), mFsbFormat);
    SetMaxChannels(2, mFsbFormat);
    AddExtension(wxT("fsb"), mFsbFormat);
    AddExtension(wxT("strm"), mFsbFormat);

    mRasFormat = AddFormat() - 1;
    SetFormat(wxT("RAS"), mRasFormat);
    SetCanMetaData(false, mRasFormat);
    SetDescription(wxT("Nintendo GameCube DSPADPCM (.ras,.strm)"), mRasFormat);
    SetMaxChannels(2, mRasFormat);
    AddExtension(wxT("ras"), mRasFormat);
    AddExtension(wxT("strm"), mRasFormat);
}

void ExportDSPADPCM::Destroy()
{
    delete this;
}

bool ExportDSPADPCM::DisplayOptions(wxWindow *parent, int subfmt)
{
    if (subfmt == mDspFormat)
    {
        ExportDSPADPCMOptionsDsp od;
        od.ShowModal();
        return true;
    }
    else if (subfmt == mCsmpFormat)
    {
        ExportDSPADPCMOptionsCsmp od;
        od.ShowModal();
        return true;
    }
    return false;
}


int ExportDSPADPCM::ExportStandard(AudacityProject *project,
                                   int numChannels,
                                   const wxString& fName,
                                   bool selectionOnly,
                                   double t0,
                                   double t1,
                                   MixerSpec *mixerSpec,
                                   bool csmp, ECsmpLayout csmpLayout)
{
    double       rate = project->GetRate();
    TrackList   *tracks = project->GetTracks();

    unsigned int sampleRate = (unsigned int)(rate + 0.5);
    unsigned int sampleFrames = (unsigned int)((t1 - t0)*rate + 0.5);

    wxFile f[2];   // will be closed when it goes out of scope
    if (numChannels > 1)
    {
        wxString fileName;
        std::size_t dotPos;

        fileName = fName;
        dotPos = fileName.rfind('.');
        if (dotPos > 0)
            fileName.insert(dotPos, 1, 'L');
        else
            fileName += 'L';
        if (!f[0].Open(fileName, wxFile::write)) {
            wxMessageBox(wxString::Format(_("Cannot export audio to %s"),
                                          fileName.c_str()));
            return false;
        }

        fileName = fName;
        dotPos = fileName.rfind('.');
        if (dotPos > 0)
            fileName.insert(dotPos, 1, 'R');
        else
            fileName += 'R';
        if (!f[1].Open(fileName, wxFile::write)) {
            wxMessageBox(wxString::Format(_("Cannot export audio to %s"),
                                          fileName.c_str()));
            return false;
        }
    }
    else
    {
        if (!f[0].Open(fName, wxFile::write)) {
            wxMessageBox(wxString::Format(_("Cannot export audio to %s"),
                                          fName.c_str()));
            return false;
        }
    }

    int updateResult = eProgressSuccess;

    int numWaveTracks;
    WaveTrack **waveTracks;
    tracks->GetWaveTracks(selectionOnly, &numWaveTracks, &waveTracks);
    Mixer *mixer = CreateMixer(numWaveTracks, waveTracks,
                               tracks->GetTimeTrack(),
                               t0, t1,
                               numChannels, sampleFrames, false,
                               rate, int16Sample, true, mixerSpec);

    ProgressDialog *progress = new ProgressDialog(wxFileName(fName).GetName(),
                                                  selectionOnly ?
                                                      _("Exporting the selected audio as Standard DSPADPCM") :
                                                      _("Exporting the entire project as Standard DSPADPCM"));

    sampleCount numSamples = mixer->Process(sampleFrames);
    unsigned int packetCount = (numSamples+13) / 14;


    if (numSamples <= 2)
    {
        delete progress;
        delete mixer;
        delete[] waveTracks;
        return eProgressFailed;
    }

    /* See if project contains loop region */
    TrackListOfKindIterator labelIt(Track::Label);
    const LabelTrack* labelTrack;
    bool loops = false;
    uint32_t loopStartSample = 0;
    uint32_t loopStartNibble = 0;
    uint32_t loopEndSample = 0;
    uint32_t loopEndNibble = 0;
    for (labelTrack = (LabelTrack*)labelIt.First(tracks);
         labelTrack;
         labelTrack = (LabelTrack*)labelIt.Next())
    {
        for (int l=0 ; l<labelTrack->GetNumLabels() ; ++l)
        {
            const LabelStruct* label = labelTrack->GetLabel(l);
            if (!label->title.CmpNoCase(wxT("loop")))
            {
                loops = true;
                loopStartSample = label->getT0() * rate;
                loopStartNibble = sampleidx_to_nibbleidx(label->getT0() * rate);
                loopEndSample = label->getT1() * rate;
                loopEndNibble = sampleidx_to_nibbleidx(label->getT1() * rate);
                break;
            }
        }
        if (loops)
            break;
    }

    bool loopHistAdded[2][2] = {};
    int16_t loopHist[2][2] = {};
    wxFileOffset dspHeaderOff[2] = {};

    /* Encode samples */
    for (int c=0 ; c<numChannels ; ++c)
    {
        short* mixed = (short*)mixer->GetBuffer(c);
        short coefs[16];
        DSPCorrelateCoefs(mixed, numSamples, coefs);

        if (csmp)
        {
            if (csmpLayout == DSPADPCM_CSMP_MP3)
            {
                f[c].Write("CSMP\x00\x00\x00\x01", 8);
                f[c].Write("INFO\x00\x00\x00\x0C", 8);
                f[c].Write("\x01", 1);
                if (loops)
                    f[c].Write("\x01", 1);
                else
                    f[c].Write("\x00", 1);
                f[c].Write("\x00\x00\x00\x00\x00\x00", 6);
                uint32_t float100 = bswapu32(0x42C80000);
                f[c].Write(&float100, 4);
                f[c].Write("PAD \x00\x00\x00\x14", 8);
                f[c].Write("\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff", 20);
                f[c].Write("DATA", 4);
                uint32_t dataSz = bswapu32(sizeof(dspadpcm_header) + packetCount * 8 - 4);
                f[c].Write(&dataSz, 4);
            }
            else if (csmpLayout == DSPADPCM_CSMP_DKCTF)
            {
                uint32_t dspSz = sizeof(dspadpcm_header) + packetCount * 8;
                f[c].Write("RFRM\x00\x00\x00\x00", 8);
                uint32_t resourceSz = bswapu32(dspSz + 160);
                f[c].Write(&resourceSz, 4);
                f[c].Write("\x00\x00\x00\x00\x00\x00\x00\x00""CSMP""\x00\x00\x00\x0A\x00\x00\x00\x0A", 20);
                f[c].Write("LABL""\x00\x00\x00\x00\x00\x00\x00\x50\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01"
                           "\x42\xD8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x04\x41\xF8\xE3\x85\x00\x00\x00\x01"
                           "\x42\xD8\x00\x00\x00\x00\x00\x00\x41\xF8\xE3\x85\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x04\x43\x4A\x35\x81\x00\x00\x00\x00"
                           "\x00\x00\x00\x00\x00\x00\x00\x13""FMTA""\x00\x00\x00\x00\x00\x00\x00\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
                           "\x02\x00\x00\x00\x03""DATA""\x00\x00\x00\x00", 141);
                uint32_t dataSz = bswapu32(dspSz + 3);
                f[c].Write(&dataSz, 4);
                f[c].Write("\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 15);
            }
        }

        dspHeaderOff[c] = f[c].Tell();
        struct dspadpcm_header header = {};
        header.num_samples = bswapu32(numSamples-2);
        header.num_nibbles = bswapu32(packetCount*16);
        header.sample_rate = bswapu32(sampleRate);
        header.loop_flag = bswap16((int16_t)loops);
        if (csmp && csmpLayout == DSPADPCM_CSMP_MP3)
        {
            /* MP3/DKCR use sample indices for loop addressing */
            header.loop_start_nibble = bswapu32(loopStartSample);
            header.loop_end_nibble = bswapu32(loopEndSample);
        }
        else
        {
            /* Standard DSP and DKCTF use nibble indices for loop addressing */
            header.loop_start_nibble = bswapu32(loopStartNibble);
            header.loop_end_nibble = bswapu32(loopEndNibble);
        }
        for (int i=0 ; i<16 ; ++i)
            header.coef[i] = bswap16(coefs[i]);
        f[c].Write(&header, sizeof(header));

        short convSamps[16] = {};
        unsigned char block[8];
        unsigned int writtenSamples = 0;
        for (int p=0 ; p<packetCount ; ++p)
        {
            for (int s=0 ; s<14 ; ++s)
            {
                unsigned int sample = p*14+s;
                if (sample >= numSamples)
                    convSamps[s+2] = 0;
                else
                    convSamps[s+2] = mixed[sample];
            }

            DSPEncodeFrame(convSamps, MIN(14, packetCount * 14 - writtenSamples), block, coefs);

            /* Resolve loop sample */
            if (!loopHistAdded[c][0] &&
                writtenSamples <= loopStartSample - 1 &&
                writtenSamples + 14 > loopStartSample - 1)
            {
                loopHistAdded[c][0] = true;
                loopHist[c][0] = convSamps[loopStartSample-writtenSamples-1];
            }
            if (!loopHistAdded[c][1] &&
                writtenSamples <= loopStartSample - 2 &&
                writtenSamples + 14 > loopStartSample - 2)
            {
                loopHistAdded[c][1] = true;
                loopHist[c][1] = convSamps[loopStartSample-writtenSamples-2];
            }

            convSamps[0] = convSamps[14];
            convSamps[1] = convSamps[15];

            f[c].Write(block, 8);
            writtenSamples += 14;
            if (!(p%1024))
                if ((updateResult = progress->Update(writtenSamples, numSamples)) != eProgressSuccess)
                    break;
        }
        if (updateResult != eProgressSuccess)
            break;
    }

    if (loopHistAdded[0][0] && loopHistAdded[0][1])
    {
        loopHist[0][0] = bswap16(loopHist[0][0]);
        loopHist[0][1] = bswap16(loopHist[0][1]);
        f[0].Seek(dspHeaderOff[0] + offsetof(struct dspadpcm_header, loop_hist1));
        f[0].Write(loopHist[0], 4);
    }

    if (loopHistAdded[1][0] && loopHistAdded[1][1])
    {
        loopHist[1][0] = bswap16(loopHist[1][0]);
        loopHist[1][1] = bswap16(loopHist[1][1]);
        f[1].Seek(dspHeaderOff[1] + offsetof(struct dspadpcm_header, loop_hist1));
        f[1].Write(loopHist[1], 4);
    }

    delete progress;
    delete mixer;
    delete[] waveTracks;
    return updateResult;
}

int ExportDSPADPCM::ExportRS03(AudacityProject *project,
                               int numChannels,
                               const wxString& fName,
                               bool selectionOnly,
                               double t0,
                               double t1,
                               MixerSpec *mixerSpec)
{
    double       rate = project->GetRate();
    TrackList   *tracks = project->GetTracks();

    unsigned int sampleRate = (unsigned int)(rate + 0.5);
    unsigned int sampleFrames = (unsigned int)((t1 - t0)*rate + 0.5);

    wxFile fs;   // will be closed when it goes out of scope
    if (!fs.Open(fName, wxFile::write)) {
        wxMessageBox(wxString::Format(_("Cannot export audio to %s"),
                                      fName.c_str()));
        return false;
    }

    int updateResult = eProgressSuccess;

    int numWaveTracks;
    WaveTrack **waveTracks;
    tracks->GetWaveTracks(selectionOnly, &numWaveTracks, &waveTracks);
    Mixer *mixer = CreateMixer(numWaveTracks, waveTracks,
                               tracks->GetTimeTrack(),
                               t0, t1,
                               numChannels, sampleFrames, false,
                               rate, int16Sample, true, mixerSpec);

    ProgressDialog *progress = new ProgressDialog(wxFileName(fName).GetName(),
                                                  selectionOnly ?
                                                      _("Exporting the selected audio as RS03 DSPADPCM") :
                                                      _("Exporting the entire project as RS03 DSPADPCM"));

    sampleCount numSamples = mixer->Process(sampleFrames);
    if (numSamples <= 2)
    {
        delete progress;
        delete mixer;
        delete[] waveTracks;
        return eProgressFailed;
    }

    /* See if project contains loop region */
    TrackListOfKindIterator labelIt(Track::Label);
    const LabelTrack* labelTrack;
    bool loops = false;
    uint32_t loopStartSample = 0;
    uint32_t loopStartByte = 0;
    uint32_t loopEndByte = 0;
    for (labelTrack = (LabelTrack*)labelIt.First(tracks);
         labelTrack;
         labelTrack = (LabelTrack*)labelIt.Next())
    {
        for (int l=0 ; l<labelTrack->GetNumLabels() ; ++l)
        {
            const LabelStruct* label = labelTrack->GetLabel(l);
            if (!label->title.CmpNoCase(wxT("loop")))
            {
                loops = true;
                loopStartSample = label->getT0() * sampleRate;
                loopStartByte = sampleidx_to_byteidx(label->getT0() * rate);
                loopEndByte = sampleidx_to_byteidx(label->getT1() * rate);
                break;
            }
        }
        if (loops)
            break;
    }

    short* mixed[2];
    short coefs[2][16] = {};
    for (int c=0 ; c<numChannels ; ++c)
    {
        mixed[c] = (short*)mixer->GetBuffer(c);
        DSPCorrelateCoefs(mixed[c], numSamples, coefs[c]);
    }

    /* Compute sample/block/frame counts */
    unsigned chanFullFrames = numSamples / 14;
    unsigned chanFullBlocks = chanFullFrames / 4576;
    chanFullFrames = chanFullBlocks * 4576;
    unsigned chanFullSamples = chanFullFrames * 14;
    unsigned chanRemSamples = numSamples - chanFullSamples;
    unsigned chanRemBytes = ((chanRemSamples * 8 / 14) + 7) & ~7;
    unsigned chanRemFrames = chanRemBytes / 8;

    /* Write header */
    struct rs03_header header = {};
    header.chan_count = bswapu32(numChannels);
    header.num_samples = bswapu32(numSamples);
    header.sample_rate = bswapu32(sampleRate);
    header.chan_byte_count = bswapu32(chanFullFrames * 8 + chanRemBytes);
    header.loop_flag = bswap16((uint16_t)loops);
    header.loop_start = bswapu32(loopStartByte);
    header.loop_end = bswapu32(loopEndByte);
    for (int i=0 ; i<16 ; ++i)
        header.coefs[0][i] = bswap16(coefs[0][i]);
    for (int i=0 ; i<16 ; ++i)
        header.coefs[1][i] = bswap16(coefs[1][i]);
    fs.Write("RS\x00\x03", 4);
    fs.Write(&header, sizeof(header));

    TADPCMFrame* adpcmBlock = new TADPCMFrame[4576];
    short convSamps[2][16] = {};

    /* Write full-blocks */
    for (int b=0 ; b<chanFullBlocks ; ++b)
    {
        for (int c=0 ; c<numChannels ; ++c)
        {
            for (int f=0 ; f<4576 ; ++f)
            {
                for (int s=0 ; s<14 ; ++s)
                {
                    unsigned int sample = (b*4576+f)*14+s;
                    if (sample >= numSamples)
                        convSamps[c][s+2] = 0;
                    else
                        convSamps[c][s+2] = mixed[c][sample];
                }
                DSPEncodeFrame(convSamps[c], 14, adpcmBlock[f], coefs[c]);
                convSamps[c][0] = convSamps[c][14];
                convSamps[c][1] = convSamps[c][15];
            }
            fs.Write(adpcmBlock, 4576 * 8);
        }
        if ((updateResult = progress->Update((unsigned int)b*4576*14, numSamples)) != eProgressSuccess)
            break;
    }

    /* Write remaining block */
    if (updateResult == eProgressSuccess)
    {
        for (int c=0 ; c<numChannels ; ++c)
        {
            memset(adpcmBlock, 0, 4576 * 8);
            unsigned remSamples = chanRemSamples;
            for (int f=0 ; f<chanRemFrames ; ++f)
            {
                int s;
                for (s=0 ; s<14 && s<remSamples ; ++s)
                {
                    unsigned int sample = chanFullSamples + f*14 + s;
                    if (sample >= numSamples)
                        convSamps[c][s+2] = 0;
                    else
                        convSamps[c][s+2] = mixed[c][sample];
                }
                DSPEncodeFrame(convSamps[c], s, adpcmBlock[f], coefs[c]);
                convSamps[c][0] = convSamps[c][14];
                convSamps[c][1] = convSamps[c][15];
                remSamples -= s;
            }
            fs.Write(adpcmBlock, chanRemBytes);
        }
    }

    delete progress;
    delete mixer;
    delete[] waveTracks;
    delete[] adpcmBlock;
    return updateResult;
}

int ExportDSPADPCM::ExportFSB31(AudacityProject *project,
                                int numChannels,
                                const wxString& fName,
                                bool selectionOnly,
                                double t0,
                                double t1,
                                MixerSpec *mixerSpec)
{
    double       rate = project->GetRate();
    TrackList   *tracks = project->GetTracks();

    unsigned int sampleRate = (unsigned int)(rate + 0.5);
    unsigned int sampleFrames = (unsigned int)((t1 - t0)*rate + 0.5);

    wxFile fs;   // will be closed when it goes out of scope
    if (!fs.Open(fName, wxFile::write)) {
        wxMessageBox(wxString::Format(_("Cannot export audio to %s"),
                                      fName.c_str()));
        return false;
    }

    int updateResult = eProgressSuccess;

    int numWaveTracks;
    WaveTrack **waveTracks;
    tracks->GetWaveTracks(selectionOnly, &numWaveTracks, &waveTracks);
    Mixer *mixer = CreateMixer(numWaveTracks, waveTracks,
                               tracks->GetTimeTrack(),
                               t0, t1,
                               numChannels, sampleFrames, false,
                               rate, int16Sample, true, mixerSpec);

    ProgressDialog *progress = new ProgressDialog(wxFileName(fName).GetName(),
                                                  selectionOnly ?
                                                      _("Exporting the selected audio as FSB 3.1 DSPADPCM") :
                                                      _("Exporting the entire project as FSB 3.1 DSPADPCM"));

    sampleCount numSamples = mixer->Process(sampleFrames);
    if (numSamples <= 2)
    {
        delete progress;
        delete mixer;
        delete[] waveTracks;
        return eProgressFailed;
    }

    /* See if project contains loop region */
    TrackListOfKindIterator labelIt(Track::Label);
    const LabelTrack* labelTrack;
    bool loops = false;
    uint32_t loopStartSample = 0;
    uint32_t loopEndSample = 0;
    for (labelTrack = (LabelTrack*)labelIt.First(tracks);
         labelTrack;
         labelTrack = (LabelTrack*)labelIt.Next())
    {
        for (int l=0 ; l<labelTrack->GetNumLabels() ; ++l)
        {
            const LabelStruct* label = labelTrack->GetLabel(l);
            if (!label->title.CmpNoCase(wxT("loop")))
            {
                loops = true;
                loopStartSample = label->getT0() * sampleRate;
                loopEndSample = label->getT1() * sampleRate;
                break;
            }
        }
        if (loops)
            break;
    }

    short* mixed[2];
    short coefs[2][16] = {};
    for (int c=0 ; c<numChannels ; ++c)
    {
        mixed[c] = (short*)mixer->GetBuffer(c);
        DSPCorrelateCoefs(mixed[c], numSamples, coefs[c]);
    }

    /* Compute frame count */
    unsigned chanFrames = (numSamples + 13) / 14;

    /* Write headers */
    struct fsb3_header header = {};
    header.sampleCount = numSamples;
    header.headerSize = sizeof(fsb31_sample_header) + sizeof(fsb_dspadpcm_channel) * numChannels;
    header.dataSize = chanFrames * numChannels * 8;
    header.version = 0x00030001;
    fs.Write("FSB3", 4);
    fs.Write(&header, sizeof(header));

    struct fsb31_sample_header sheader = {};
    sheader.size = header.headerSize;
    sheader.sampleCount = numSamples;
    sheader.dataSize = header.dataSize;
    sheader.loopStart = loopStartSample;
    sheader.loopEnd = loopEndSample;
    sheader.mode = FSOUND_GCADPCM;
    if (loops)
        sheader.mode |= FSOUND_LOOP_NORMAL;
    sheader.defaultFreq = sampleRate;
    sheader.defVol = 255;
    sheader.defPan = 128;
    sheader.defPri = 255;
    sheader.channelCount = numChannels;
    sheader.minDistance = 1.0;
    sheader.maxDistance = 10000.0;
    fs.Write(&sheader, sizeof(sheader));

    for (int c=0 ; c<numChannels ; ++c)
    {
        fsb_dspadpcm_channel fsbChan = {};
        for (int i=0 ; i<16 ; ++i)
            fsbChan.coef[i] = bswap16(coefs[c][i]);
        fs.Write(&fsbChan, sizeof(fsbChan));
    }

    unsigned curSample = 0;
    unsigned remSamples = numSamples;
    unsigned remFrames = chanFrames;
    unsigned numIOBlocks = (chanFrames + 511) / 512;

    if (numChannels == 2)
    {
        TADPCMStereoFrame* adpcmBlock = new TADPCMStereoFrame[512];
        short convSamps[2][16] = {};
        for (int b=0 ; b<numIOBlocks ; ++b)
        {
            int f;
            for (f=0 ; f<512 && f<remFrames ; ++f)
            {
                for (int s=0 ; s<14 && s<remSamples ; ++s)
                {
                    for (int c=0 ; c<2 ; ++c)
                    {
                        if (curSample >= numSamples)
                            convSamps[c][s+2] = 0;
                        else
                            convSamps[c][s+2] = mixed[c][curSample];
                    }
                    ++curSample;
                    --remSamples;
                }
                for (int c=0 ; c<2 ; ++c)
                {
                    TADPCMFrame frame;
                    DSPEncodeFrame(convSamps[c], 14, frame, coefs[c]);
                    convSamps[c][0] = convSamps[c][14];
                    convSamps[c][1] = convSamps[c][15];
                    for (int i=0 ; i<8 ; ++i)
                        adpcmBlock[f][i/2][c][i%2] = frame[i];
                }
                --remFrames;
            }
            fs.Write(adpcmBlock, f*16);
            if ((updateResult = progress->Update(curSample, numSamples)) != eProgressSuccess)
                break;
        }
        delete[] adpcmBlock;
    }
    else
    {
        TADPCMFrame* adpcmBlock = new TADPCMFrame[512];
        short convSamps[16] = {};
        for (int b=0 ; b<numIOBlocks ; ++b)
        {
            int f;
            for (f=0 ; f<512 && f<remFrames ; ++f)
            {
                for (int s=0 ; s<14 && s<remSamples ; ++s)
                {
                    if (curSample >= numSamples)
                        convSamps[s+2] = 0;
                    else
                        convSamps[s+2] = mixed[0][curSample];
                    ++curSample;
                    --remSamples;
                }
                DSPEncodeFrame(convSamps, 14, adpcmBlock[f], coefs[0]);
                convSamps[0] = convSamps[14];
                convSamps[1] = convSamps[15];
                --remFrames;
            }
            fs.Write(adpcmBlock, f*8);
            if ((updateResult = progress->Update(curSample, numSamples)) != eProgressSuccess)
                break;
        }
        delete[] adpcmBlock;
    }

    delete progress;
    delete mixer;
    delete[] waveTracks;
    return updateResult;
}

int ExportDSPADPCM::ExportRAS(AudacityProject *project,
                              int numChannels,
                              const wxString& fName,
                              bool selectionOnly,
                              double t0,
                              double t1,
                              MixerSpec *mixerSpec)
{
    double       rate = project->GetRate();
    TrackList   *tracks = project->GetTracks();
    Tags        *tags = project->GetTags();

    unsigned int sampleRate = (unsigned int)(rate + 0.5);
    unsigned int sampleFrames = (unsigned int)((t1 - t0)*rate + 0.5);

    wxFile fs;   // will be closed when it goes out of scope
    if (!fs.Open(fName, wxFile::write)) {
        wxMessageBox(wxString::Format(_("Cannot export audio to %s"),
                                      fName.c_str()));
        return false;
    }

    int updateResult = eProgressSuccess;

    int numWaveTracks;
    WaveTrack **waveTracks;
    tracks->GetWaveTracks(selectionOnly, &numWaveTracks, &waveTracks);
    Mixer *mixer = CreateMixer(numWaveTracks, waveTracks,
                               tracks->GetTimeTrack(),
                               t0, t1,
                               numChannels, sampleFrames, false,
                               rate, int16Sample, true, mixerSpec);

    ProgressDialog *progress = new ProgressDialog(wxFileName(fName).GetName(),
                                                  selectionOnly ?
                                                      _("Exporting the selected audio as RAS DSPADPCM") :
                                                      _("Exporting the entire project as RAS DSPADPCM"));

    sampleCount numSamples = mixer->Process(sampleFrames);
    if (numSamples <= 2)
    {
        delete progress;
        delete mixer;
        delete[] waveTracks;
        return eProgressFailed;
    }

    /* See if project contains loop region */
    TrackListOfKindIterator labelIt(Track::Label);
    const LabelTrack* labelTrack;
    bool loops = false;
    uint32_t loopStartSample = 0;
    uint32_t loopEndSample = 0;
    for (labelTrack = (LabelTrack*)labelIt.First(tracks);
         labelTrack;
         labelTrack = (LabelTrack*)labelIt.Next())
    {
        for (int l=0 ; l<labelTrack->GetNumLabels() ; ++l)
        {
            const LabelStruct* label = labelTrack->GetLabel(l);
            if (!label->title.CmpNoCase(wxT("loop")))
            {
                loops = true;
                loopStartSample = label->getT0() * sampleRate;
                loopEndSample = label->getT1() * sampleRate;
                break;
            }
        }
        if (loops)
            break;
    }

    short* mixed[2];
    short coefs[2][16] = {};
    for (int c=0 ; c<numChannels ; ++c)
    {
        mixed[c] = (short*)mixer->GetBuffer(c);
        DSPCorrelateCoefs(mixed[c], numSamples, coefs[c]);
    }

    /* Compute frame and block count */
    unsigned chanFrames = (numSamples + 13) / 14;
    unsigned chanBlocks = (chanFrames + 4095) / 4096;

    /* Write headers */
    struct ras_header header = {};
    header.chanCount1 = numChannels;
    header.chanCount2 = numChannels;
    header.numSamples = numSamples;
    header.metaDataFlag = tags->HasTag(wxT("BPM")) ? 2 : 0;
    header.sampleRate = sampleRate;
    header.dataOffset = header.metaDataFlag ? 192 : 160;
    header.dataSize = chanBlocks * 32768 * numChannels;
    header.blockSize = 32768;
    header.numBlocks = chanBlocks;
    header.startSample = 0;
    header.lastSampleOfLastBlock = numSamples - (chanBlocks-1) * 4096 * 14;
    if (loops)
    {
        header.loopStartBlock = loopStartSample / 14 / 4096;
        header.loopStartSample = loopStartSample - header.loopStartBlock * 4096 * 14;
        header.loopEndBlock = loopEndSample / 14 / 4096;
        header.loopEndSample = loopEndSample - header.loopEndBlock * 4096 * 14;
    }
    SwapRASHeader(&header);
    fs.Write("RAS_", 4);
    fs.Write(&header, sizeof(header));

    for (int c=0 ; c<numChannels ; ++c)
    {
        ras_dspadpcm_channel rasChan = {};
        for (int i=0 ; i<16 ; ++i)
            rasChan.coefs[i] = coefs[c][i];
        SwapRASChannel(&rasChan);
        fs.Write(&rasChan, sizeof(rasChan));
    }

    struct ras_track_meta meta = {};
    if (header.metaDataFlag)
    {
        wxString bpmStr = tags->GetTag(wxT("BPM"));
        double bpmVal;
        if (bpmStr.ToDouble(&bpmVal))
        {
            meta.bpmFlag = 1;
            meta.bpm = bpmVal;
        }
        meta.unknowns2[2] = 2;
        meta.unknowns2[4] = 4;
        SwapRASTrackMeta(&meta);
        fs.Write(&meta, sizeof(meta));
    }

    unsigned curSample[2] = {};
    unsigned remSamples[2] = {numSamples, numSamples};

    TADPCMFrame* adpcmBlock = new TADPCMFrame[4096];
    short convSamps[2][16] = {};
    for (int b=0 ; b<chanBlocks ; ++b)
    {
        for (int c=0 ; c<numChannels ; ++c)
        {
            int f;
            for (f=0 ; f<4096 ; ++f)
            {
                int s;
                for (s=0 ; s<14 && s<remSamples[c] ; ++s)
                {
                    if (curSample[c] >= numSamples)
                        convSamps[c][s+2] = 0;
                    else
                        convSamps[c][s+2] = mixed[c][curSample[c]];
                    ++curSample[c];
                    --remSamples[c];
                }
                if (s)
                {
                    DSPEncodeFrame(convSamps[c], s, adpcmBlock[f], coefs[c]);
                    convSamps[c][0] = convSamps[c][14];
                    convSamps[c][1] = convSamps[c][15];
                }
                else
                    memset(adpcmBlock[f], 0, 8);
            }
            fs.Write(adpcmBlock, f*8);
        }
        if ((updateResult = progress->Update(curSample[0], numSamples)) != eProgressSuccess)
            break;
    }
    delete[] adpcmBlock;

    delete progress;
    delete mixer;
    delete[] waveTracks;
    return updateResult;
}

int ExportDSPADPCM::Export(AudacityProject *project,
                           int numChannels,
                           wxString fName,
                           bool selectionOnly,
                           double t0,
                           double t1,
                           MixerSpec *mixerSpec,
                           Tags*,
                           int subformat)
{
    if (numChannels > 2)
    {
        wxMessageBox(_("Project must have 1 or 2 channels"));
        return false;
    }

    if (subformat == mDspFormat)
    {
        int selLayout = ReadExportDSPLayoutPref();
        if (selLayout < 0 || selLayout >= DSPADPCM_LAYOUTMAX)
            selLayout = 0;

        if (selLayout == DSPADPCM_STD)
            return ExportStandard(project, numChannels, fName, selectionOnly, t0, t1,
                                  mixerSpec, false, DSPADPCM_CSMP_LAYOUTMAX);
        else if (selLayout == DSPADPCM_RS03)
            return ExportRS03(project, numChannels, fName, selectionOnly, t0, t1, mixerSpec);
    }
    else if (subformat == mCsmpFormat)
    {
        int selLayout = ReadExportCSMPLayoutPref();
        if (selLayout < 0 || selLayout >= DSPADPCM_CSMP_LAYOUTMAX)
            selLayout = 0;

        return ExportStandard(project, numChannels, fName, selectionOnly, t0, t1,
                              mixerSpec, true, (ECsmpLayout)selLayout);
    }
    else if (subformat == mFsbFormat)
    {
        return ExportFSB31(project, numChannels, fName, selectionOnly, t0, t1, mixerSpec);
    }
    else if (subformat == mRasFormat)
    {
        if (numChannels != 2)
        {
            wxMessageBox(_("Project must have 2 channels"));
            return false;
        }
        return ExportRAS(project, numChannels, fName, selectionOnly, t0, t1, mixerSpec);
    }

    return eProgressFailed;
}

ExportPlugin *New_ExportDSPADPCM()
{
    return new ExportDSPADPCM();
}
