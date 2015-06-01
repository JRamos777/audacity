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

static inline unsigned sampleidx_to_nibbleidx(unsigned sampleidx)
{
    unsigned frame = sampleidx / 14;
    unsigned frame_samp = sampleidx % 14;
    return frame * 16 + frame_samp + 2;
}

static inline unsigned byteidx_to_sampleidx(unsigned byteidx)
{
    unsigned frame = byteidx / 8;
    unsigned frame_byte = MAX(1, byteidx % 8) - 1;
    return frame * 14 + frame_byte * 2;
}

static inline unsigned sampleidx_to_byteidx(unsigned sampleidx)
{
    unsigned frame = sampleidx / 14;
    unsigned frame_samp = sampleidx % 14;
    return frame * 8 + frame_samp / 2 + 1;
}

static const wxChar *exts[] =
{
    wxT("dsp")
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

private:
    friend class DSPADPCMImportPlugin;
    bool valid;
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
    int GetFileUncompressedBytes() { return MAX(mHeader[0].num_samples, mHeader[1].num_samples) * 2; }
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags);

private:
    friend class DSPADPCMImportPlugin;
    bool valid;
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
    int GetFileUncompressedBytes() { return mHeader.num_samples * 2; }
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags);

private:
    friend class DSPADPCMImportPlugin;
    bool valid;
    wxFile *mFile;
    struct rs03_header mHeader;
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
    :  DSPADPCMBaseImportFileHandle(filename)
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

    unsigned char adpcmBlock[4096][8];
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
        lt->AddLabel(SelectedRegion(
                     nibbleidx_to_sampleidx(mHeader.loop_start) / sr,
                     nibbleidx_to_sampleidx(mHeader.loop_end) / sr), wxT("LOOP"));
        (*outTracks)[1] = lt;
    }

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

    unsigned char adpcmBlock[2][4096][8];
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
    unsigned char adpcmBlock[4576][8];
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
        /* Compute last block (round up to nearest block size) */
        unsigned chanRemBytes = mHeader.chan_byte_count % 0x8f00;
        unsigned chanRemFrames = (chanRemBytes + 7) / 8;

        for (int c=0 ; c<mHeader.chan_count ; ++c)
        {
            mFile->Read(adpcmBlock, chanRemFrames * 8);
            for (int f=0 ; f<chanRemFrames ; ++f)
            {
                unsigned long samplesremaining = mHeader.num_samples - samplescompleted[c];
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
            }
        }
    }

    if (updateResult == eProgressFailed || updateResult == eProgressCancelled)
    {
        for (int c=0 ; c<2 ; c++)
            delete channels[c];
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
