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

static const int nibble_to_int[16] = {0,1,2,3,4,5,6,7,-8,-7,-6,-5,-4,-3,-2,-1};

static inline short samp_clamp(int val) {
    if (val < -32768) val = -32768;
    if (val > 32767) val = 32767;
    return val;
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define DESC _("Nintendo GameCube DSPADPCM")

static const wxChar *exts[] =
{
    wxT("dsp")
};

class DSPADPCMImportPlugin : public ImportPlugin
{
public:
    DSPADPCMImportPlugin()
        :  ImportPlugin(wxArrayString(WXSIZEOF(exts), exts))
    {
    }

    ~DSPADPCMImportPlugin() { }

    wxString GetPluginStringID() { return wxT("gc-dspadpcm"); }
    wxString GetPluginFormatDescription();
    ImportFileHandle *Open(wxString Filename);
};


class DSPADPCMImportFileHandle : public ImportFileHandle
{
public:
    DSPADPCMImportFileHandle(wxFile *lFile, wxFile *rFile, wxString filename);
    ~DSPADPCMImportFileHandle();

    wxString GetFileDescription();
    int GetFileUncompressedBytes();
    int Import(TrackFactory *trackFactory, Track ***outTracks,
               int *outNumTracks, Tags *tags);

    wxInt32 GetStreamCount(){ return 1; }

    wxArrayString *GetStreamInfo(){ return NULL; }

    void SetStreamUsage(wxInt32 WXUNUSED(StreamID), bool WXUNUSED(Use)){}

private:
    friend class DSPADPCMImportPlugin;
    wxFile *mFile[2];
    struct dspadpcm_header mHeader[2];
};

void GetDSPADPCMImportPlugin(ImportPluginList * importPluginList,
                             UnusableImportPluginList * WXUNUSED(unusableImportPluginList))
{
    importPluginList->Append(new DSPADPCMImportPlugin);
}

wxString DSPADPCMImportPlugin::GetPluginFormatDescription()
{
    return DESC;
}

ImportFileHandle *DSPADPCMImportPlugin::Open(wxString filename)
{
    wxFile* lfile = NULL;
    wxFile* rfile = NULL;

    wxFile* file = new wxFile(filename);
    if (!file->IsOpened()) {
        delete file;
        return NULL;
    }

    std::size_t dotpos = filename.rfind('.');
    if (dotpos < 0 )
        dotpos = filename.size();

    /* Stereo pair check */
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
    else
    {
        lfile = file;
    }

    DSPADPCMImportFileHandle* fh = new DSPADPCMImportFileHandle(lfile, rfile, filename);
    if (fh->mHeader[0].format || fh->mHeader[1].format)
    {
        delete fh;
        return NULL;
    }

    return fh;
}

DSPADPCMImportFileHandle::DSPADPCMImportFileHandle(wxFile *lFile, wxFile *rFile, wxString filename)
    :  ImportFileHandle(filename)
{
    wxASSERT(lFile);
    mFile[0] = lFile;
    mFile[1] = rFile;

    /* Read in DSP header(s) here */
    mFile[0]->Read(&mHeader[0], sizeof(mHeader[0]));
    SwapDSPHeader(&mHeader[0]);
    if (mFile[1])
    {
        mFile[1]->Read(&mHeader[1], sizeof(mHeader[1]));
        SwapDSPHeader(&mHeader[1]);
    }
    else
        memset(&mHeader[1], 0, sizeof(mHeader[1]));
}

wxString DSPADPCMImportFileHandle::GetFileDescription()
{
    return DESC;
}

int DSPADPCMImportFileHandle::GetFileUncompressedBytes()
{
    return (mFile[1] ? MAX(mHeader[0].num_samples, mHeader[1].num_samples) * 2 : mHeader[0].num_samples) * 2;
}

int DSPADPCMImportFileHandle::Import(TrackFactory *trackFactory,
                                     Track ***outTracks,
                                     int *outNumTracks,
                                     Tags *tags)
{
    CreateProgress();

    unsigned int channelCount = mFile[1] ? 2 : 1;
    WaveTrack* channels[2];

    int c;
    for (c=0 ; c<channelCount ; c++) {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, mHeader[c].sample_rate);

        if (channelCount > 1)
            switch (c) {
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

    if (channelCount == 2)
        channels[0]->SetLinked(true);

    sampleCount fileTotalFrames = MAX(mHeader[0].num_samples, mHeader[1].num_samples);
    int updateResult = false;

    // Otherwise, we're in the "copy" mode, where we read in the actual
    // samples from the file and store our own local copy of the
    // samples in the tracks.

    unsigned long framescompleted[2] = {};
    short hist[2][2] = {{mHeader[0].hist1, mHeader[0].hist2},
                        {mHeader[1].hist1, mHeader[1].hist2}};

    long block;
    do {

        block = 0;
        for (c=0 ; c<channelCount ; c++)
        {
            unsigned char adpcmBlock[8];
            short pcmBlock[14];
            if (framescompleted[c] < mHeader[c].num_samples)
            {
                unsigned long framesremaining = mHeader[c].num_samples - framescompleted[c];
                mFile[c]->Read(adpcmBlock, 8);
                unsigned char cIdx = (adpcmBlock[0]>>4) & 0xf;
                short factor1 = mHeader[c].coef[cIdx*2];
                short factor2 = mHeader[c].coef[cIdx*2+1];
                unsigned char exp = adpcmBlock[0] & 0xf;
                int s;
                for (s=0 ; s<14 && s<framesremaining ; ++s) {
                    int sample_data = (s&1)?
                    nibble_to_int[(adpcmBlock[s/2+1])&0xf]:
                    nibble_to_int[(adpcmBlock[s/2+1]>>4)&0xf];
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
                framescompleted[c] += s;
                if (s > block)
                    block = s;
            }
        }

        long long unsigned fc = MAX(framescompleted[0], framescompleted[1]);
        if (!(fc%1024))
        {
            updateResult = mProgress->Update((long long unsigned)fc,
                                             (long long unsigned)fileTotalFrames);
            if (updateResult != eProgressSuccess)
                break;
        }

    } while (block > 0);

    if (updateResult == eProgressFailed || updateResult == eProgressCancelled) {
        for (c=0 ; c<channelCount ; c++)
            delete channels[c];

        return updateResult;
    }

    *outNumTracks = ((mHeader[0].loop_flag) ? channelCount + 1 : channelCount);
    *outTracks = new Track *[*outNumTracks];
    for(c=0 ; c<channelCount ; c++) {
        channels[c]->Flush();
        (*outTracks)[c] = channels[c];
    }

    /* Add loop label */
    if (mHeader[0].loop_flag)
    {
        LabelTrack* lt = trackFactory->NewLabelTrack();
        double nibblesPerSec = mHeader[0].sample_rate * 16 / 14.0;
        lt->AddLabel(SelectedRegion(
                     mHeader[0].loop_start / nibblesPerSec,
                     mHeader[0].loop_end / nibblesPerSec), wxT("LOOP"));
        (*outTracks)[channelCount] = lt;
    }

    return updateResult;
}

DSPADPCMImportFileHandle::~DSPADPCMImportFileHandle()
{
    if(mFile[0]) {
        if (mFile[0]->IsOpened()) {
            mFile[0]->Close();
        }
        delete mFile[0];
    }
    if(mFile[1]) {
        if (mFile[1]->IsOpened()) {
            mFile[1]->Close();
        }
        delete mFile[1];
    }
}
