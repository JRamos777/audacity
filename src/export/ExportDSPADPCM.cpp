/**********************************************************************

  Audacity: A Digital Audio Editor

  ExportDSPADPCM.cpp

  Jack Andersen

**********************************************************************/

#include <string>

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

static inline uint16_t bswapu16(uint16_t val)
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
#define bswapu16(val) (val)

#endif

/* Standard DSPADPCM header */
struct dspadpcm_header
{
    uint32_t num_samples;
    uint32_t num_nibbles;
    uint32_t sample_rate;
    uint16_t loop_flag;
    uint16_t format; /* 0 for ADPCM */
    uint32_t loop_start;
    uint32_t loop_end;
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
        else if(tmp != 0.0)
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
        if(in[z] >= 1.0)
            in[z] = 0.9999999999;
        if(in[z] <= -1.0)
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
    double val = (-source2[2] * -source2[1] + -source2[1]) / (1.0 - -source2[2] * -source2[2]);
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
    int numBlocks = (samples + 13) / 14;
    int blockSamples;

    short* frameBuffer = (short*)calloc(sizeof(short), 0x3800);
    short pcmHistBuffer[2][14] = {};

    tvec vec1;
    tvec vec2;

    tvec mtx[3];
    int vecIdxs[3];

    tvec* records = (tvec*)calloc(sizeof(tvec), numBlocks * 2);
    int recordCount = 0;

    tvec vecBest[8];

    /* Iterate though 1024-block frames */
    for (int x=samples ; x>0 ;)
    {
        if (x > 0x3800) /* Full 1024-block frame */
        {
            blockSamples = 0x3800;
            x -= 0x3800;
        }
        else /* Partial frame */
        {
            /* Zero lingering block samples */
            blockSamples = x;
            for(int z=0 ; z<14 && z+blockSamples<0x3800 ; z++)
                frameBuffer[blockSamples+z] = 0;
            x = 0;
        }

        /* Copy (potentially non-block-aligned PCM samples into aligned buffer) */
        memcpy(frameBuffer, source, blockSamples * sizeof(short));
        source += blockSamples;


        for (int i=0 ; i<blockSamples ;)
        {
            for (int z=0 ; z<14 ; z++)
                pcmHistBuffer[0][z] = pcmHistBuffer[1][z];
            for (int z=0 ; z<14 ; z++)
                pcmHistBuffer[1][z] = frameBuffer[i++];

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
    free(frameBuffer);

}

//Make sure source includes the yn values (16 samples total)
static void DSPEncodeChunk(short* pcmInOut, int sampleCount, unsigned char* adpcmOut, const short* coefsIn)
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

struct
{
    wxChar *name;
    wxChar *desc;
    wxChar *ext;
}
static const kFormats[] =
{
{ wxT("DSP"), XO("Nintendo GameCube DSPADPCM"), wxT("dsp") },
};

//----------------------------------------------------------------------------
// ExportDSPADPCM Class
//----------------------------------------------------------------------------

class ExportDSPADPCM : public ExportPlugin
{
public:

    ExportDSPADPCM();
    void Destroy();

    // Required

    int Export(AudacityProject *project,
               int channels,
               wxString fName,
               bool selectedOnly,
               double t0,
               double t1,
               MixerSpec *mixerSpec = NULL,
               Tags *metadata = NULL,
               int subformat = 0);

};

ExportDSPADPCM::ExportDSPADPCM()
    :  ExportPlugin()
{
    for (size_t i = 0; i < WXSIZEOF(kFormats); i++)
    {
        int format = AddFormat() - 1;

        SetFormat(kFormats[i].name, format);
        SetCanMetaData(false, format);
        SetDescription(wxGetTranslation(kFormats[i].desc), format);
        AddExtension(kFormats[i].ext, format);
        SetMaxChannels(2, format);
    }
}

void ExportDSPADPCM::Destroy()
{
    delete this;
}

/**
 *
 * @param subformat Control whether we are doing a "preset" export to a popular
 * file type, or giving the user full control over libsndfile.
 */
int ExportDSPADPCM::Export(AudacityProject *project,
                           int numChannels,
                           wxString fName,
                           bool selectionOnly,
                           double t0,
                           double t1,
                           MixerSpec *mixerSpec,
                           Tags *metadata,
                           int subformat)
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
                                                      wxString::Format(_("Exporting the selected audio as %s"),
                                                                       GetDescription(subformat).c_str()) :
                                                      wxString::Format(_("Exporting the entire project as %s"),
                                                                       GetDescription(subformat).c_str()));

    sampleCount numSamples = mixer->Process(sampleFrames);
    unsigned int packetCount = (numSamples+13) / 14;

    if (numSamples <= 2)
        goto done;

    for (int c=0 ; c<numChannels ; ++c)
    {
        short* mixed = (short*)mixer->GetBuffer(c);
        short coefs[16];
        DSPCorrelateCoefs(mixed, numSamples, coefs);

        struct dspadpcm_header header = {};
        header.num_samples = bswapu32(numSamples-2);
        header.num_nibbles = bswapu32(packetCount*16);
        header.sample_rate = bswapu32(sampleRate);
        for (int i=0 ; i<16 ; ++i)
            header.coef[i] = bswapu16(coefs[i]);
        header.hist1 = bswapu16(mixed[0]);
        header.hist2 = bswapu16(mixed[1]);
        f[c].Write(&header, sizeof(header));

        short convSamps[16] = {mixed[0], mixed[1]};
        mixed += 2;
        unsigned char block[8];
        unsigned int writtenSamples = 0;
        for (int p=0 ; p<packetCount ; ++p)
        {
            for (int s=0 ; s<14 ; ++s)
            {
                unsigned int sample = p*14+s+2;
                if (sample >= numSamples)
                    convSamps[s+2] = 0;
                else
                    convSamps[s+2] = mixed[sample];
            }

            DSPEncodeChunk(convSamps, MIN(14, packetCount * 14 - writtenSamples), block, coefs);

            convSamps[0] = convSamps[14];
            convSamps[1] = convSamps[15];

            f[c].Write(block, 8);
            writtenSamples += 14;
            if (!(p%48))
                progress->Update(writtenSamples, numSamples);
        }
    }

done:
    delete progress;
    delete mixer;
    delete[] waveTracks;
    return updateResult;
}

ExportPlugin *New_ExportDSPADPCM()
{
    return new ExportDSPADPCM();
}