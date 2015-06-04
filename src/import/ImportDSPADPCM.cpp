/**********************************************************************

  Audacity: A Digital Audio Editor

  ImportDSPADPCM.cpp

  Jack Andersen

*//****************************************************************//**

\class DSPADPCMImportFileHandle
\brief An ImportFileHandle for DSPADPCM data

*//****************************************************************//**

\class DSPADPCMImportPlugin
\brief An ImportPlugin for PCM data

*//*******************************************************************/

#include <stdint.h>
#include <string.h>

#include "../Audacity.h"
#include "../AudacityApp.h"
#include "../Internat.h"
#include "../Tags.h"
#include "ImportDSPADPCM.h"

#include "../FileFormats.h"
#include "../Prefs.h"
#include "../WaveTrack.h"
#include "../LabelTrack.h"
#include "ImportPlugin.h"

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

typedef unsigned char TADPCMFrame[8];
typedef unsigned char TADPCMStereoFrame[4][2][2];

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

static void SwapDSPHeader(struct dspadpcm_header* h)
{
    h->num_samples = bswapu32(h->num_samples);
    h->num_nibbles = bswapu32(h->num_nibbles);
    h->sample_rate = bswapu32(h->sample_rate);
    h->loop_flag = bswap16(h->loop_flag);
    h->format = bswap16(h->format);
    h->loop_start = bswapu32(h->loop_start);
    h->loop_end = bswapu32(h->loop_end);
    for (int c=0 ; c<16 ; ++c)
        h->coef[c] = bswap16(h->coef[c]);
    h->gain = bswap16(h->gain);
    h->ps = bswap16(h->ps);
    h->hist1 = bswap16(h->hist1);
    h->hist2 = bswap16(h->hist2);
    h->loop_ps = bswap16(h->loop_ps);
    h->loop_hist1 = bswap16(h->loop_hist1);
    h->loop_hist2 = bswap16(h->loop_hist2);
}

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

static void SwapRS03Header(struct rs03_header* h)
{
    h->chan_count = bswapu32(h->chan_count);
    h->num_samples = bswapu32(h->num_samples);
    h->sample_rate = bswapu32(h->sample_rate);
    h->chan_byte_count = bswapu32(h->chan_byte_count);
    h->loop_flag = bswap16(h->loop_flag);
    h->format = bswap16(h->format);
    h->loop_start = bswapu32(h->loop_start);
    h->loop_end = bswapu32(h->loop_end);
    for (int c=0 ; c<16 ; ++c)
        h->coefs[0][c] = bswap16(h->coefs[0][c]);
    for (int c=0 ; c<16 ; ++c)
        h->coefs[1][c] = bswap16(h->coefs[1][c]);
}

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

static void SwapFSBDSPADPCMChannels(struct fsb_dspadpcm_channel* h, int chanCount)
{
    for (int c=0 ; c<chanCount ; ++c)
        for (int co=0 ; co<16 ; ++co)
            h[c].coef[co] = bswap16(h[c].coef[co]);
}

static const int nibble_to_int[16] = {0,1,2,3,4,5,6,7,-8,-7,-6,-5,-4,-3,-2,-1};

static inline short samp_clamp(int val) {
    if (val < -32768) val = -32768;
    if (val > 32767) val = 32767;
    return val;
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

static inline unsigned nibbleidx_to_sampleidx(unsigned nibbleidx)
{
    unsigned frame = nibbleidx / 16;
    unsigned frame_samp = MAX(2, nibbleidx % 16) - 2;
    return frame * 14 + frame_samp;
}

static inline unsigned byteidx_to_sampleidx(unsigned byteidx)
{
    unsigned frame = byteidx / 8;
    unsigned frame_byte = MAX(1, byteidx % 8) - 1;
    return frame * 14 + frame_byte * 2;
}

static const wxChar *exts[] =
{
    wxT("dsp"),
    wxT("fsb"),
    wxT("strm"),
    wxT("csmp")
};

/**************************************************************************
 * Plugin Class
 **************************************************************************/

class DSPADPCMImportPlugin : public ImportPlugin
{
public:
    DSPADPCMImportPlugin()
    : ImportPlugin(wxArrayString(WXSIZEOF(exts), exts))
    {
    }

    ~DSPADPCMImportPlugin() { }

    wxString GetPluginStringID() { return wxT("gc-dspadpcm"); }
    wxString GetPluginFormatDescription() { return _("Nintendo GameCube DSPADPCM"); }
    ImportFileHandle *Open(wxString Filename);
};

/* Plugin factory */
void GetDSPADPCMImportPlugin(ImportPluginList * importPluginList,
                             UnusableImportPluginList * WXUNUSED(unusableImportPluginList))
{
    importPluginList->Append(new DSPADPCMImportPlugin);
}

/**************************************************************************
 * File Handle Classes
 **************************************************************************/

/* Base class (takes care of Audacity formalities) */
class DSPADPCMBaseImportFileHandle : public ImportFileHandle
{
public:
    DSPADPCMBaseImportFileHandle(const wxString& filename)
    : ImportFileHandle(filename) {}
    wxInt32 GetStreamCount(){ return 1; }
    wxArrayString *GetStreamInfo(){ return NULL; }
    void SetStreamUsage(wxInt32 WXUNUSED(StreamID), bool WXUNUSED(Use)){}
    bool valid;
};

/* Mono standard .dsp file */
class DSPADPCMStandardMonoImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMStandardMonoImportFileHandle(wxFile *file, const wxString& filename);
    ~DSPADPCMStandardMonoImportFileHandle();

    wxString GetFileDescription() { return _("Mono GameCube DSPADPCM"); }
    int GetFileUncompressedBytes() { return mHeader.num_samples * 2; }
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags);

protected:
    bool mSampleIndexedLoops;
private:
    wxFile *mFile;
    struct dspadpcm_header mHeader;
};

/* Stereo-pair of standard .dsp files */
class DSPADPCMStandardStereoImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMStandardStereoImportFileHandle(wxFile *lFile, wxFile *rFile, const wxString& filename);
    ~DSPADPCMStandardStereoImportFileHandle();

    wxString GetFileDescription() { return _("Stereo Pair GameCube DSPADPCM"); }
    int GetFileUncompressedBytes() { return MAX(mHeader[0].num_samples, mHeader[1].num_samples) * 2 * 2; }
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags);

private:
    wxFile *mFile[2];
    struct dspadpcm_header mHeader[2];
};

/* RS03 block-interleaved .dsp file (from Metroid Prime 2) */
class DSPADPCMRS03ImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMRS03ImportFileHandle(wxFile *file, const wxString& filename);
    ~DSPADPCMRS03ImportFileHandle();

    wxString GetFileDescription() { return _("RS03 Stereo GameCube DSPADPCM"); }
    int GetFileUncompressedBytes() { return mHeader.num_samples * 2 * mHeader.chan_count; }
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags);

private:
    wxFile *mFile;
    struct rs03_header mHeader;
};

/* Mono CSMP-headered file */
class DSPADPCMCSMPMonoImportFileHandle : public DSPADPCMStandardMonoImportFileHandle
{
    float mVolume;
public:
    DSPADPCMCSMPMonoImportFileHandle(wxFile *file, const wxString& filename, float volume)
    : DSPADPCMStandardMonoImportFileHandle(file, filename), mVolume(volume)
    {
        mSampleIndexedLoops = true;
    }

    wxString GetFileDescription() { return _("Mono CSMP DSPADPCM"); }
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags)
    {
        int result = DSPADPCMStandardMonoImportFileHandle::Import(trackFactory, outTracks, outNumTracks, tags);
        if (result == eProgressSuccess)
        {
            WaveTrack* wt = (WaveTrack*)(*outTracks)[0];
            wt->SetGain(mVolume / 100.0);
        }
        return result;
    }
};

/* FSB3.1 file */
class DSPADPCMFSB31ImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMFSB31ImportFileHandle(wxFile *file, const wxString& filename);
    ~DSPADPCMFSB31ImportFileHandle();

    wxString GetFileDescription() { return _("FSB 3.1 Stereo GameCube DSPADPCM"); }
    int GetFileUncompressedBytes() { return mHeader.sampleCount * 2 * mHeader.channelCount; }
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags);

private:
    wxFile *mFile;
    struct fsb31_sample_header mHeader;
    struct fsb_dspadpcm_channel mCoefs[2];
};

/**************************************************************************
 * Plugin Implementation
 **************************************************************************/

ImportFileHandle *DSPADPCMImportPlugin::Open(wxString filename)
{
    wxFile* lfile = NULL;
    wxFile* rfile = NULL;

    wxFile* file = new wxFile(filename);
    if (!file->IsOpened()) {
        delete file;
        return NULL;
    }

    char magic[4];

    /* Check for RS03 format */
    file->Read(magic, 4);
    if (!memcmp(magic, "RS\x00\x03", 4))
    {
        DSPADPCMRS03ImportFileHandle* fh = new DSPADPCMRS03ImportFileHandle(file, filename);
        if (!fh->valid)
        {
            delete fh;
            return NULL;
        }
        return fh;
    }
    else if (!memcmp(magic, "FSB3", 4))
    {
        DSPADPCMFSB31ImportFileHandle* fh = new DSPADPCMFSB31ImportFileHandle(file, filename);
        if (!fh->valid)
        {
            delete fh;
            return NULL;
        }
        return fh;
    }
    else if (!memcmp(magic, "CSMP", 4))
    {
        uint32_t csmpVersion = 0;
        file->Read(&csmpVersion, 4);
        csmpVersion = bswapu32(csmpVersion);
        if (csmpVersion != 1)
        {
            delete file;
            return NULL;
        }

        float csmpVolume = 100.0;
        while (true)
        {
            if (file->Read(magic, 4) != 4)
                break;

            uint32_t size = 0;
            if (file->Read(&size, 4) != 4)
                break;
            size = bswapu32(size);

            if (!memcmp(magic, "INFO", 4))
            {
                file->Seek(8, wxFromCurrent);
                uint32_t volumeInt;
                file->Read(&volumeInt, 4);
                *((uint32_t*)&csmpVolume) = bswapu32(volumeInt);
                file->Seek(size-12, wxFromCurrent);
            }
            else if (!memcmp(magic, "PAD ", 4) || !memcmp(magic, "NAME", 4))
            {
                file->Seek(size, wxFromCurrent);
                continue;
            }
            else if (!memcmp(magic, "DATA", 4))
            {
                DSPADPCMCSMPMonoImportFileHandle* fh = new DSPADPCMCSMPMonoImportFileHandle(file, filename, csmpVolume);
                if (!fh->valid)
                {
                    delete fh;
                    return NULL;
                }
                return fh;
            }
        }
        delete file;
        return NULL;
    }
    file->Seek(0);

    /* Assume standard mono .dsp (check for stereo pair) */
    std::size_t dotpos = filename.rfind('.');
    if (dotpos < 0 )
        dotpos = filename.size();

    if (filename[dotpos-1] == 'L' ||
        filename[dotpos-1] == 'l')
    {
        wxString rFilename = filename;
        rFilename[dotpos-1] = 'R';
        rfile = new wxFile(rFilename);
        if (!rfile->IsOpened()) {
            delete rfile;
            rFilename[dotpos-1] = 'r';
            rfile = new wxFile(rFilename);
            if (!rfile->IsOpened()) {
                delete rfile;
                rfile = NULL;
            }
        }
        lfile = file;
    }
    else if (filename[dotpos-1] == 'R' ||
             filename[dotpos-1] == 'r')
    {
        wxString lFilename = filename;
        lFilename[dotpos-1] = 'L';
        lfile = new wxFile(lFilename);
        if (!lfile->IsOpened()) {
            delete lfile;
            lFilename[dotpos-1] = 'l';
            lfile = new wxFile(lFilename);
            if (!lfile->IsOpened()) {
                delete lfile;
                lfile = NULL;
            }
        }
        if (!lfile)
            lfile = file;
        else
            rfile = file;
    }

    if (rfile && lfile)
    {
        /* Stereo pair import */
        DSPADPCMStandardStereoImportFileHandle* fh = new DSPADPCMStandardStereoImportFileHandle(lfile, rfile, filename);
        if (!fh->valid)
        {
            delete fh;
            return NULL;
        }
        return fh;
    }

    /* Regular mono import */
    DSPADPCMStandardMonoImportFileHandle* fh = new DSPADPCMStandardMonoImportFileHandle(file, filename);
    if (!fh->valid)
    {
        delete fh;
        return NULL;
    }
    return fh;

}

/**************************************************************************
 * Standard Mono
 **************************************************************************/

DSPADPCMStandardMonoImportFileHandle::DSPADPCMStandardMonoImportFileHandle(wxFile *file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename), mSampleIndexedLoops(false)
{
    wxASSERT(file);
    mFile = file;

    /* Read in DSP header(s) here */
    mFile->Read(&mHeader, sizeof(mHeader));
    SwapDSPHeader(&mHeader);

    valid = true;
    if (mHeader.format)
        valid = false;
}

int DSPADPCMStandardMonoImportFileHandle::Import(TrackFactory *trackFactory,
                                     Track ***outTracks,
                                     int *outNumTracks,
                                     Tags *tags)
{
    CreateProgress();

    WaveTrack* channel = trackFactory->NewWaveTrack(int16Sample, mHeader.sample_rate);

    sampleCount fileTotalFrames = mHeader.num_samples;
    int updateResult = false;

    unsigned long samplescompleted = 0;
    short hist[2] = {mHeader.hist1, mHeader.hist2};

    TADPCMFrame* adpcmBlock = new TADPCMFrame[4096];
    unsigned blockFrame = 0;
    short pcmBlock[14];
    while (samplescompleted < mHeader.num_samples)
    {
        unsigned long samplesremaining = mHeader.num_samples - samplescompleted;
        if (!(blockFrame % 4096))
        {
            mFile->Read(adpcmBlock, 4096 * 8);
            blockFrame = 0;
        }
        unsigned char cIdx = (adpcmBlock[blockFrame][0]>>4) & 0xf;
        short factor1 = mHeader.coef[cIdx*2];
        short factor2 = mHeader.coef[cIdx*2+1];
        unsigned char exp = adpcmBlock[blockFrame][0] & 0xf;
        int s;
        for (s=0 ; s<14 && s<samplesremaining ; ++s) {
            int sample_data = (s&1)?
            nibble_to_int[(adpcmBlock[blockFrame][s/2+1])&0xf]:
            nibble_to_int[(adpcmBlock[blockFrame][s/2+1]>>4)&0xf];
            sample_data <<= exp;
            sample_data <<= 11;
            sample_data += 1024;
            sample_data +=
            factor1 * hist[0] +
            factor2 * hist[1];
            sample_data >>= 11;
            sample_data = samp_clamp(sample_data);
            pcmBlock[s] = sample_data;
            hist[1] = hist[0];
            hist[0] = sample_data;
        }
        channel->Append((samplePtr)pcmBlock, int16Sample, (sampleCount)s);
        samplescompleted += s;
        ++blockFrame;

        if (!(samplescompleted % 1024))
        {
            updateResult = mProgress->Update((long long unsigned)samplescompleted,
                                             (long long unsigned)fileTotalFrames);
            if (updateResult != eProgressSuccess)
                break;
        }
    }

    if (updateResult == eProgressFailed || updateResult == eProgressCancelled)
    {
        delete channel;
        delete[] adpcmBlock;
        return updateResult;
    }

    *outNumTracks = ((mHeader.loop_flag) ? 2 : 1);
    *outTracks = new Track *[*outNumTracks];
    channel->Flush();
    (*outTracks)[0] = channel;

    /* Add loop label */
    if (mHeader.loop_flag)
    {
        LabelTrack* lt = trackFactory->NewLabelTrack();
        double sr = mHeader.sample_rate;
        if (mSampleIndexedLoops)
            lt->AddLabel(SelectedRegion(
                         mHeader.loop_start / sr,
                         mHeader.loop_end / sr), wxT("LOOP"));
        else
            lt->AddLabel(SelectedRegion(
                         nibbleidx_to_sampleidx(mHeader.loop_start) / sr,
                         nibbleidx_to_sampleidx(mHeader.loop_end) / sr), wxT("LOOP"));
        (*outTracks)[1] = lt;
    }

    delete[] adpcmBlock;
    return updateResult;
}

DSPADPCMStandardMonoImportFileHandle::~DSPADPCMStandardMonoImportFileHandle()
{
    if (mFile)
    {
        if (mFile->IsOpened())
            mFile->Close();
        delete mFile;
    }
}

/**************************************************************************
 * Standard Stereo Pair
 **************************************************************************/

DSPADPCMStandardStereoImportFileHandle::DSPADPCMStandardStereoImportFileHandle(wxFile *lFile, wxFile *rFile, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(lFile);
    wxASSERT(rFile);
    mFile[0] = lFile;
    mFile[1] = rFile;

    /* Read in DSP header(s) here */
    mFile[0]->Read(&mHeader[0], sizeof(mHeader[0]));
    SwapDSPHeader(&mHeader[0]);
    mFile[1]->Read(&mHeader[1], sizeof(mHeader[1]));
    SwapDSPHeader(&mHeader[1]);

    valid = true;
    if (mHeader[0].format || mHeader[1].format)
        valid = false;
}

int DSPADPCMStandardStereoImportFileHandle::Import(TrackFactory *trackFactory,
                                     Track ***outTracks,
                                     int *outNumTracks,
                                     Tags *tags)
{
    CreateProgress();

    WaveTrack* channels[2];

    for (int c=0 ; c<2 ; c++)
    {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, mHeader[c].sample_rate);
        switch (c)
        {
        case 0:
            channels[c]->SetChannel(Track::LeftChannel);
            break;
        case 1:
            channels[c]->SetChannel(Track::RightChannel);
            break;
        default:
            channels[c]->SetChannel(Track::MonoChannel);
        }
    }
    channels[0]->SetLinked(true);

    sampleCount fileTotalFrames = MAX(mHeader[0].num_samples, mHeader[1].num_samples);
    int updateResult = false;

    unsigned long samplescompleted[2] = {};
    short hist[2][2] = {{mHeader[0].hist1, mHeader[0].hist2},
                        {mHeader[1].hist1, mHeader[1].hist2}};

    TADPCMFrame* adpcmBlock[2];
    adpcmBlock[0] = new TADPCMFrame[4096];
    adpcmBlock[1] = new TADPCMFrame[4096];
    unsigned blockFrame[2] = {};
    short pcmBlock[14];
    while (MAX(samplescompleted[0], samplescompleted[1]) <
           MAX(mHeader[0].num_samples, mHeader[1].num_samples))
    {
        for (int c=0 ; c<2 ; c++)
        {
            if (samplescompleted[c] < mHeader[c].num_samples)
            {
                unsigned long samplesremaining = mHeader[c].num_samples - samplescompleted[c];
                if (!(blockFrame[c] % 4096))
                {
                    mFile[c]->Read(adpcmBlock[c], 4096 * 8);
                    blockFrame[c] = 0;
                }
                unsigned char cIdx = (adpcmBlock[c][blockFrame[c]][0]>>4) & 0xf;
                short factor1 = mHeader[c].coef[cIdx*2];
                short factor2 = mHeader[c].coef[cIdx*2+1];
                unsigned char exp = adpcmBlock[c][blockFrame[c]][0] & 0xf;
                int s;
                for (s=0 ; s<14 && s<samplesremaining ; ++s) {
                    int sample_data = (s&1)?
                    nibble_to_int[(adpcmBlock[c][blockFrame[c]][s/2+1])&0xf]:
                    nibble_to_int[(adpcmBlock[c][blockFrame[c]][s/2+1]>>4)&0xf];
                    sample_data <<= exp;
                    sample_data <<= 11;
                    sample_data += 1024;
                    sample_data +=
                    factor1 * hist[c][0] +
                    factor2 * hist[c][1];
                    sample_data >>= 11;
                    sample_data = samp_clamp(sample_data);
                    pcmBlock[s] = sample_data;
                    hist[c][1] = hist[c][0];
                    hist[c][0] = sample_data;
                }
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (sampleCount)s);
                samplescompleted[c] += s;
                ++blockFrame[c];
            }
        }

        long long unsigned fc = MAX(samplescompleted[0], samplescompleted[1]);
        if (!(fc % 1024))
        {
            updateResult = mProgress->Update((long long unsigned)fc,
                                             (long long unsigned)fileTotalFrames);
            if (updateResult != eProgressSuccess)
                break;
        }
    }

    if (updateResult == eProgressFailed || updateResult == eProgressCancelled)
    {
        for (int c=0 ; c<2 ; c++)
            delete channels[c];
        delete[] adpcmBlock[0];
        delete[] adpcmBlock[1];
        return updateResult;
    }

    *outNumTracks = ((mHeader[0].loop_flag) ? 3 : 2);
    *outTracks = new Track *[*outNumTracks];
    for(int c=0 ; c<2 ; c++) {
        channels[c]->Flush();
        (*outTracks)[c] = channels[c];
    }

    /* Add loop label */
    if (mHeader[0].loop_flag)
    {
        LabelTrack* lt = trackFactory->NewLabelTrack();
        double sr = mHeader[0].sample_rate;
        lt->AddLabel(SelectedRegion(
                     nibbleidx_to_sampleidx(mHeader[0].loop_start) / sr,
                     nibbleidx_to_sampleidx(mHeader[0].loop_end) / sr), wxT("LOOP"));
        (*outTracks)[2] = lt;
    }

    delete[] adpcmBlock[0];
    delete[] adpcmBlock[1];
    return updateResult;
}

DSPADPCMStandardStereoImportFileHandle::~DSPADPCMStandardStereoImportFileHandle()
{
    if (mFile[0]) {
        if (mFile[0]->IsOpened())
            mFile[0]->Close();
        delete mFile[0];
    }
    if (mFile[1])
    {
        if (mFile[1]->IsOpened())
            mFile[1]->Close();
        delete mFile[1];
    }
}

/**************************************************************************
 * RS03 Stereo
 **************************************************************************/

DSPADPCMRS03ImportFileHandle::DSPADPCMRS03ImportFileHandle(wxFile *file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = file;

    /* Read in DSP header(s) here */
    mFile->Read(&mHeader, sizeof(mHeader));
    SwapRS03Header(&mHeader);

    valid = true;
    if (mHeader.format || mHeader.chan_count > 2)
        valid = false;
}

int DSPADPCMRS03ImportFileHandle::Import(TrackFactory *trackFactory,
                                     Track ***outTracks,
                                     int *outNumTracks,
                                     Tags *tags)
{
    CreateProgress();

    WaveTrack* channels[2];
    for (int c=0 ; c<mHeader.chan_count ; ++c)
    {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, mHeader.sample_rate);
        if (mHeader.chan_count == 2)
            switch (c)
            {
            case 0:
                channels[c]->SetChannel(Track::LeftChannel);
                break;
            case 1:
                channels[c]->SetChannel(Track::RightChannel);
                break;
            default:
                channels[c]->SetChannel(Track::MonoChannel);
            }
    }
    if (mHeader.chan_count == 2)
        channels[0]->SetLinked(true);

    int updateResult = eProgressSuccess;

    unsigned long samplescompleted[2] = {};
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    /* Compute number of full-sized blocks */
    unsigned chanFullblocks = mHeader.chan_byte_count / 0x8f00;
    TADPCMFrame* adpcmBlock = new TADPCMFrame[4576];
    short pcmBlock[14];
    for (int b=0 ; b<chanFullblocks ; ++b)
    {
        for (int c=0 ; c<mHeader.chan_count ; ++c)
        {
            mFile->Read(adpcmBlock, 4576 * 8);
            for (int f=0 ; f<4576 ; ++f)
            {
                unsigned char cIdx = (adpcmBlock[f][0]>>4) & 0xf;
                short factor1 = mHeader.coefs[c][cIdx*2];
                short factor2 = mHeader.coefs[c][cIdx*2+1];
                unsigned char exp = adpcmBlock[f][0] & 0xf;
                int s;
                for (s=0 ; s<14 ; ++s) {
                    int sample_data = (s&1)?
                    nibble_to_int[(adpcmBlock[f][s/2+1])&0xf]:
                    nibble_to_int[(adpcmBlock[f][s/2+1]>>4)&0xf];
                    sample_data <<= exp;
                    sample_data <<= 11;
                    sample_data += 1024;
                    sample_data +=
                    factor1 * hist[c][0] +
                    factor2 * hist[c][1];
                    sample_data >>= 11;
                    sample_data = samp_clamp(sample_data);
                    pcmBlock[s] = sample_data;
                    hist[c][1] = hist[c][0];
                    hist[c][0] = sample_data;
                }
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (sampleCount)s);
                samplescompleted[c] += s;
            }
        }
        long long unsigned fc = MAX(samplescompleted[0], samplescompleted[1]);
        updateResult = mProgress->Update((long long unsigned)fc,
                                         (long long unsigned)mHeader.num_samples);
        if (updateResult != eProgressSuccess)
            break;
    }

    if (updateResult == eProgressSuccess)
    {
        /* Compute last block */
        unsigned chanRemSamples = mHeader.num_samples - chanFullblocks * 0x8f00 * 14 / 8;
        unsigned chanRemBytes = ((chanRemSamples * 8 / 14) + 7) & ~7;
        unsigned chanRemFrames = chanRemBytes / 8;

        for (int c=0 ; c<mHeader.chan_count ; ++c)
        {
            unsigned long samplesremaining = chanRemSamples;
            mFile->Read(adpcmBlock, chanRemBytes);
            for (int f=0 ; f<chanRemFrames ; ++f)
            {
                unsigned char cIdx = (adpcmBlock[f][0]>>4) & 0xf;
                short factor1 = mHeader.coefs[c][cIdx*2];
                short factor2 = mHeader.coefs[c][cIdx*2+1];
                unsigned char exp = adpcmBlock[f][0] & 0xf;
                int s;
                for (s=0 ; s<14 && s<samplesremaining ; ++s) {
                    int sample_data = (s&1)?
                    nibble_to_int[(adpcmBlock[f][s/2+1])&0xf]:
                    nibble_to_int[(adpcmBlock[f][s/2+1]>>4)&0xf];
                    sample_data <<= exp;
                    sample_data <<= 11;
                    sample_data += 1024;
                    sample_data +=
                    factor1 * hist[c][0] +
                    factor2 * hist[c][1];
                    sample_data >>= 11;
                    sample_data = samp_clamp(sample_data);
                    pcmBlock[s] = sample_data;
                    hist[c][1] = hist[c][0];
                    hist[c][0] = sample_data;
                }
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (sampleCount)s);
                samplescompleted[c] += s;
                samplesremaining -= s;
            }
        }
    }

    if (updateResult == eProgressFailed || updateResult == eProgressCancelled)
    {
        for (int c=0 ; c<2 ; c++)
            delete channels[c];
        delete[] adpcmBlock;
        return updateResult;
    }

    *outNumTracks = ((mHeader.loop_flag) ? mHeader.chan_count + 1 : mHeader.chan_count);
    *outTracks = new Track *[*outNumTracks];
    for (int c=0 ; c<mHeader.chan_count ; ++c)
    {
        channels[c]->Flush();
        (*outTracks)[c] = channels[c];
    }

    /* Add loop label */
    if (mHeader.loop_flag)
    {
        LabelTrack* lt = trackFactory->NewLabelTrack();
        double sr = mHeader.sample_rate;
        lt->AddLabel(SelectedRegion(
                     byteidx_to_sampleidx(mHeader.loop_start) / sr,
                     byteidx_to_sampleidx(mHeader.loop_end) / sr), wxT("LOOP"));
        (*outTracks)[mHeader.chan_count] = lt;
    }

    delete[] adpcmBlock;
    return updateResult;
}

DSPADPCMRS03ImportFileHandle::~DSPADPCMRS03ImportFileHandle()
{
    if (mFile)
    {
        if (mFile->IsOpened())
            mFile->Close();
        delete mFile;
    }
}

/**************************************************************************
 * FSB 3.1 Stereo
 **************************************************************************/

DSPADPCMFSB31ImportFileHandle::DSPADPCMFSB31ImportFileHandle(wxFile *file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = file;
    valid = false;

    fsb3_header mainHeader;
    mFile->Read(&mainHeader, sizeof(mainHeader));
    if (mainHeader.version != 0x00030001)
        return;
    if (mainHeader.sampleCount < 1)
        return;

    size_t remHeaderSz = mainHeader.headerSize;
    remHeaderSz -= mFile->Read(&mHeader, sizeof(mHeader));
    if (!(mHeader.mode & FSOUND_GCADPCM))
        return;

    for (int c=0 ; c<mHeader.channelCount ; ++c)
        remHeaderSz -= mFile->Read(&mCoefs[c], sizeof(fsb_dspadpcm_channel));
    SwapFSBDSPADPCMChannels(mCoefs, mHeader.channelCount);

    if (remHeaderSz)
        mFile->Seek(remHeaderSz, wxFromCurrent);

    if (mHeader.channelCount <= 2)
        valid = true;
}

int DSPADPCMFSB31ImportFileHandle::Import(TrackFactory *trackFactory,
                                     Track ***outTracks,
                                     int *outNumTracks,
                                     Tags *tags)
{
    CreateProgress();

    WaveTrack* channels[2];
    for (int c=0 ; c<mHeader.channelCount ; ++c)
    {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, mHeader.defaultFreq);
        if (mHeader.channelCount == 2)
            switch (c)
            {
            case 0:
                channels[c]->SetChannel(Track::LeftChannel);
                break;
            case 1:
                channels[c]->SetChannel(Track::RightChannel);
                break;
            default:
                channels[c]->SetChannel(Track::MonoChannel);
            }
    }
    if (mHeader.channelCount == 2)
        channels[0]->SetLinked(true);

    int updateResult = eProgressSuccess;

    unsigned long samplescompleted = 0;
    unsigned long samplesremaining = mHeader.sampleCount;
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    if (mHeader.channelCount == 2)
    {
        short pcmBlock[2][14];
        unsigned readBlocks = (mHeader.dataSize + 8191) / 8192;
        unsigned remFrames = (mHeader.dataSize + 15) / 16;
        TADPCMStereoFrame* adpcmBlock = new TADPCMStereoFrame[512];
        for (int b=0 ; b<readBlocks ; ++b)
        {
            mFile->Read(adpcmBlock, 8192);
            for (int f=0 ; f<512 && f<remFrames ; ++f)
            {
                int s;
                for (int c=0 ; c<2 ; ++c)
                {
                    unsigned char cIdx = (adpcmBlock[f][0][c][0]>>4) & 0xf;
                    short factor1 = mCoefs[c].coef[cIdx*2];
                    short factor2 = mCoefs[c].coef[cIdx*2+1];
                    unsigned char exp = adpcmBlock[f][0][c][0] & 0xf;
                    for (s=0 ; s<14 && s<samplesremaining ; ++s) {
                        int sample_byte_idx = s/2+1;
                        int sample_data = (s&1)?
                        nibble_to_int[(adpcmBlock[f][sample_byte_idx/2][c][sample_byte_idx%2])&0xf]:
                        nibble_to_int[(adpcmBlock[f][sample_byte_idx/2][c][sample_byte_idx%2]>>4)&0xf];
                        sample_data <<= exp;
                        sample_data <<= 11;
                        sample_data += 1024;
                        sample_data +=
                        factor1 * hist[c][0] +
                        factor2 * hist[c][1];
                        sample_data >>= 11;
                        sample_data = samp_clamp(sample_data);
                        pcmBlock[c][s] = sample_data;
                        hist[c][1] = hist[c][0];
                        hist[c][0] = sample_data;
                    }
                    channels[c]->Append((samplePtr)pcmBlock[c], int16Sample, (sampleCount)s);
                }
                samplescompleted += s;
                samplesremaining -= s;
            }
            updateResult = mProgress->Update((long long unsigned)samplescompleted,
                                             (long long unsigned)mHeader.sampleCount);
            if (updateResult != eProgressSuccess)
                break;
            remFrames -= 512;
        }
        delete[] adpcmBlock;
    }
    else if (mHeader.channelCount == 1)
    {
        short pcmBlock[14];
        unsigned readBlocks = (mHeader.dataSize + 4095) / 4096;
        unsigned remFrames = (mHeader.dataSize + 7) / 8;
        TADPCMFrame* adpcmBlock = new TADPCMFrame[512];
        for (int b=0 ; b<readBlocks ; ++b)
        {
            mFile->Read(adpcmBlock, 4096);
            for (int f=0 ; f<512 && f<remFrames ; ++f)
            {
                int s;
                unsigned char cIdx = (adpcmBlock[f][0]>>4) & 0xf;
                short factor1 = mCoefs[0].coef[cIdx*2];
                short factor2 = mCoefs[0].coef[cIdx*2+1];
                unsigned char exp = adpcmBlock[f][0] & 0xf;
                for (s=0 ; s<14 && s<samplesremaining ; ++s) {
                    int sample_byte_idx = s/2+1;
                    int sample_data = (s&1)?
                    nibble_to_int[(adpcmBlock[f][sample_byte_idx])&0xf]:
                    nibble_to_int[(adpcmBlock[f][sample_byte_idx]>>4)&0xf];
                    sample_data <<= exp;
                    sample_data <<= 11;
                    sample_data += 1024;
                    sample_data +=
                    factor1 * hist[0][0] +
                    factor2 * hist[0][1];
                    sample_data >>= 11;
                    sample_data = samp_clamp(sample_data);
                    pcmBlock[s] = sample_data;
                    hist[0][1] = hist[0][0];
                    hist[0][0] = sample_data;
                }
                channels[0]->Append((samplePtr)pcmBlock, int16Sample, (sampleCount)s);
                samplescompleted += s;
                samplesremaining -= s;
            }
            updateResult = mProgress->Update((long long unsigned)samplescompleted,
                                             (long long unsigned)mHeader.sampleCount);
            if (updateResult != eProgressSuccess)
                break;
            remFrames -= 512;
        }
        delete[] adpcmBlock;
    }

    if (updateResult == eProgressFailed || updateResult == eProgressCancelled)
    {
        for (int c=0 ; c<2 ; c++)
            delete channels[c];
        return updateResult;
    }

    *outNumTracks = ((mHeader.mode & FSOUND_LOOP_NORMAL) ? mHeader.channelCount + 1 : mHeader.channelCount);
    *outTracks = new Track *[*outNumTracks];
    for (int c=0 ; c<mHeader.channelCount ; ++c)
    {
        channels[c]->Flush();
        (*outTracks)[c] = channels[c];
    }

    /* Add loop label */
    if (mHeader.mode & FSOUND_LOOP_NORMAL)
    {
        LabelTrack* lt = trackFactory->NewLabelTrack();
        double sr = mHeader.defaultFreq;
        lt->AddLabel(SelectedRegion(
                     mHeader.loopStart / sr,
                     mHeader.loopEnd / sr), wxT("LOOP"));
        (*outTracks)[mHeader.channelCount] = lt;
    }

    return updateResult;
}

DSPADPCMFSB31ImportFileHandle::~DSPADPCMFSB31ImportFileHandle()
{
    if (mFile)
    {
        if (mFile->IsOpened())
            mFile->Close();
        delete mFile;
    }
}
