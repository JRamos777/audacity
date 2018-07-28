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

#define ROUND_UP_32(val) (((val) + 31) & ~31)

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

struct stm_header
{
    uint16_t field00;                // 0x00, always 512, possibly a version number of 2.00
    uint16_t sampleRate;             // 0x02, usually 32000
    uint32_t numChannels;            // 0x04, usually 2 (could also be initial offset)
    uint32_t adpcmData2Offset;       // 0x08, byte offset to the adpcm data of the second channel (relative to the end of the this header + all channel headers)
    uint32_t adpcmLoopStartOffset;   // 0x0C, byte loop start offset, equal to 0xFFFFFFFF if not used (relative to the start of the data itself, not the headers)
    uint32_t adpcmData2OffsetAux1;   // 0x10, same value as adpcmData2Offset
    uint32_t adpcmData2OffsetAux2;   // 0x14, same value as adpcmData2Offset
    uint32_t adpcmLoopOffsetAux1;    // 0x18, if adpcmLoopOffset != 0xFFFFFFFF then same value else value is 0
    uint32_t adpcmLoopOffsetAux2;    // 0x1C, same as adpcmData2OffsetAux1
    uint8_t  padding[32];            // 0x20, 32 bytes unused
};

static void SwapSTMHeader(stm_header* h)
{
    h->field00 = bswap16(h->field00);
    h->sampleRate = bswap16(h->sampleRate);
    h->numChannels = bswapu32(h->numChannels);
    h->adpcmData2Offset = bswapu32(h->adpcmData2Offset);
    h->adpcmLoopStartOffset = bswapu32(h->adpcmLoopStartOffset);
    h->adpcmData2OffsetAux1 = bswapu32(h->adpcmData2OffsetAux1);
    h->adpcmData2OffsetAux2 = bswapu32(h->adpcmData2OffsetAux2);
    h->adpcmLoopOffsetAux1 = bswapu32(h->adpcmLoopOffsetAux1);
    h->adpcmLoopOffsetAux2 = bswapu32(h->adpcmLoopOffsetAux2);
}

struct bcwav_reference
{
    uint16_t id;
    uint32_t offset;
};

struct bcwav_sized_reference
{
    bcwav_reference ref;
    uint32_t size;
};

struct bcwav_header
{
    uint32_t magic;
    uint16_t endianness;
    uint16_t headerSize;
    uint32_t version;
    uint32_t fileSize;
    uint16_t numChunks;
    bcwav_sized_reference infoRef;
    bcwav_sized_reference dataRef;
};

struct bcwav_chunk_header
{
    uint32_t magic;
    uint32_t size;
};

struct bcwav_info
{
    bcwav_chunk_header header;
    uint8_t enc;
    uint8_t loop;
    uint32_t sampleRate;
    uint32_t loopStart;
    uint32_t loopEnd;
    uint32_t pad;
    uint32_t chanCount;
};

struct bcwav_channel_info
{
    bcwav_reference samplesRef;
    bcwav_reference adpcmInfoRef;
    uint32_t pad;
};

struct bcwav_dspadpcm_info
{
    int16_t coef[16];
    int16_t ps;
    int16_t hist1;
    int16_t hist2;
    int16_t loop_ps;
    int16_t loop_hist1;
    int16_t loop_hist2;
};

struct gcub_header
{
    uint32_t magic;
    uint32_t chanCount;
    uint32_t sampleRate;
    uint32_t endOffset;
    int16_t coef[2][16];
};

static void SwapGCubHeader(gcub_header* h)
{
    h->magic = bswapu32(h->magic);
    h->chanCount = bswapu32(h->chanCount);
    h->sampleRate = bswapu32(h->sampleRate);
    h->endOffset = bswapu32(h->endOffset);
    for (int c=0 ; c<2 ; ++c)
        for (int i=0 ; i<16 ; ++i)
            h->coef[c][i] = bswap16(h->coef[c][i]);
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
    wxT("csmp"),
    wxT("ras"),
    wxT("rsf"),
    wxT("adp"),
    wxT("stm"),
    wxT("bcwav"),
    wxT("gcub")
};

/**************************************************************************
 * Plugin Class
 **************************************************************************/

class DSPADPCMImportPlugin : public ImportPlugin
{
public:
    DSPADPCMImportPlugin()
    : ImportPlugin(wxArrayString(WXSIZEOF(exts), exts))
    {}

    ~DSPADPCMImportPlugin() {}

    wxString GetPluginStringID() { return wxT("gc-dspadpcm"); }
    wxString GetPluginFormatDescription() { return _("Nintendo GameCube DSPADPCM"); }
    std::unique_ptr<ImportFileHandle> Open(const wxString& Filename);
};

/* Plugin factory */
void GetDSPADPCMImportPlugin(ImportPluginList & importPluginList,
                             UnusableImportPluginList & WXUNUSED(unusableImportPluginList))
{
    importPluginList.push_back(std::make_unique<DSPADPCMImportPlugin>());
}

/**************************************************************************
 * File Handle Classes
 **************************************************************************/

/* Base class (takes care of Audacity formalities) */
class DSPADPCMBaseImportFileHandle : public ImportFileHandle
{
    wxArrayString m_emptyInfo;
public:
    DSPADPCMBaseImportFileHandle(const wxString& filename)
    : ImportFileHandle(filename) {}
    wxInt32 GetStreamCount(){ return 1; }
    const wxArrayString &GetStreamInfo(){ return m_emptyInfo; }
    void SetStreamUsage(wxInt32 WXUNUSED(StreamID), bool WXUNUSED(Use)){}
    bool valid;
};

/* Mono standard .dsp file */
class DSPADPCMStandardMonoImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMStandardMonoImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("Mono GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes() { return mHeader.num_samples * 2; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

protected:
    bool mSampleIndexedLoops;
private:
    std::unique_ptr<wxFile> mFile;
    struct dspadpcm_header mHeader;
};

/* Stereo-pair of standard .dsp files */
class DSPADPCMStandardStereoImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMStandardStereoImportFileHandle(std::unique_ptr<wxFile>&& lFile, std::unique_ptr<wxFile>&& rFile, const wxString& filename);

    wxString GetFileDescription() { return _("Stereo Pair GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes() { return MAX(mHeader[0].num_samples, mHeader[1].num_samples) * 2 * 2; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile[2];
    struct dspadpcm_header mHeader[2];
};

/* RS03 block-interleaved .dsp file (from Metroid Prime 2) */
class DSPADPCMRS03ImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMRS03ImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("RS03 Stereo GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes() { return mHeader.num_samples * 2 * mHeader.chan_count; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    struct rs03_header mHeader;
};

/* Mono CSMP-headered file */
class DSPADPCMCSMPMonoImportFileHandle : public DSPADPCMStandardMonoImportFileHandle
{
    float mVolume;
public:
    DSPADPCMCSMPMonoImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename, float volume, bool sampleLoops)
        : DSPADPCMStandardMonoImportFileHandle(std::move(file), filename), mVolume(volume)
    {
        mSampleIndexedLoops = sampleLoops;
    }

    wxString GetFileDescription() { return _("Mono CSMP DSPADPCM"); }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders &outTracks, Tags *tags)
    {
        ProgressResult result = DSPADPCMStandardMonoImportFileHandle::Import(trackFactory, outTracks, tags);
        if (result == ProgressResult::Success)
        {
            static_cast<WaveTrack*>(outTracks[0].get())->SetGain(mVolume / 100.0);
        }
        return result;
    }
};

/* FSB3.1 file */
class DSPADPCMFSB31ImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMFSB31ImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("FSB 3.1 Stereo GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes() { return mHeader.sampleCount * 2 * mHeader.channelCount; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    struct fsb31_sample_header mHeader;
    struct fsb_dspadpcm_channel mCoefs[2];
};

/* RAS file (DKC Tropical Freeze) */
class DSPADPCMRASImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMRASImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("RAS Stereo GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes() { return mHeader.numSamples * 2 * mHeader.chanCount1; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    struct ras_header mHeader;
    struct ras_dspadpcm_channel mCoefs[2];
    struct ras_track_meta mMeta;
};

/* STM file */
class DSPADPCMSTMImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMSTMImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("STM Stereo GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes()
    {
        return mDSPHeaders[0].num_samples * 2 * mHeader.numChannels;
    }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    struct stm_header mHeader;
    std::vector<dspadpcm_header> mDSPHeaders;
};

/* BCWAV file */
class DSPADPCMBCWAVImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMBCWAVImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("BCWAV Stereo GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes() { return mInfo.loopEnd * mInfo.chanCount * 2; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    struct bcwav_info mInfo;
    std::vector<std::pair<uint32_t, bcwav_dspadpcm_info>> mChannels;
};

/* GCub file */
class DSPADPCMGCUBImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMGCUBImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("GCub Stereo GameCube DSPADPCM"); }
    ByteCount GetFileUncompressedBytes()
    { return mSampleCount * 2 * mHeader.chanCount; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    struct gcub_header mHeader;
    uint32_t mStartOffset;
    uint32_t mSampleCount;
};

/* RSF file */
class DSPADPCMRSFImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMRSFImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("RSF Stereo G.721 ADPCM"); }
    ByteCount GetFileUncompressedBytes() { return mNumSamples * 2 * 2; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    unsigned mNumSamples;
};

/* ADP file */
class DSPADPCMADPImportFileHandle : public DSPADPCMBaseImportFileHandle
{
public:
    DSPADPCMADPImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename);

    wxString GetFileDescription() { return _("ADP GameCube Streamed ADPCM"); }
    ByteCount GetFileUncompressedBytes() { return mNumBlocks * 28 * 2 * 2; }
    ProgressResult Import(TrackFactory *trackFactory, TrackHolders&, Tags *tags);

private:
    std::unique_ptr<wxFile> mFile;
    unsigned mNumBlocks;
};

/**************************************************************************
 * Plugin Implementation
 **************************************************************************/

std::unique_ptr<ImportFileHandle> DSPADPCMImportPlugin::Open(const wxString& filename)
{
    std::unique_ptr<wxFile> lfile;
    std::unique_ptr<wxFile> rfile;

    std::unique_ptr<wxFile> file = std::make_unique<wxFile>(filename);
    if (!file->IsOpened())
        return {};

    /* Check for RSF/ADP/STM file */
    if (filename.EndsWith(wxT(".rsf")))
    {
        auto fh =
        std::make_unique<DSPADPCMRSFImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    else if (filename.EndsWith(wxT(".adp")))
    {
        auto fh =
        std::make_unique<DSPADPCMADPImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    else if (filename.EndsWith(wxT(".stm")))
    {
        auto fh =
        std::make_unique<DSPADPCMSTMImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    else if (filename.EndsWith(wxT(".bcwav")))
    {
        auto fh =
        std::make_unique<DSPADPCMBCWAVImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    else if (filename.EndsWith(wxT(".gcub")))
    {
        auto fh =
        std::make_unique<DSPADPCMGCUBImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }

    char magic[4];

    /* Check for Retro-specific DSP formats */
    file->Read(magic, 4);
    if (!memcmp(magic, "RS\x00\x03", 4))
    {
        auto fh =
        std::make_unique<DSPADPCMRS03ImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    else if (!memcmp(magic, "FSB3", 4))
    {
        auto fh =
        std::make_unique<DSPADPCMFSB31ImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    else if (!memcmp(magic, "CSMP", 4))
    {
        uint32_t csmpVersion = 0;
        file->Read(&csmpVersion, 4);
        csmpVersion = bswapu32(csmpVersion);
        if (csmpVersion != 1)
            return {};

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
            else if (!memcmp(magic, "DATA", 4))
            {
                auto fh =
                std::make_unique<DSPADPCMCSMPMonoImportFileHandle>(std::move(file), filename, csmpVolume, true);
                if (!fh->valid)
                    return {};
                return fh;
            }
            else
            {
                file->Seek(size, wxFromCurrent);
            }
        }
        return {};
    }
    else if (!memcmp(magic, "RFRM", 4))
    {
        file->Seek(16, wxFromCurrent);
        file->Read(magic, 4);
        if (!memcmp(magic, "CSMP", 4))
        {
            file->Seek(8, wxFromCurrent);
            char chanCount = 0;
            uint32_t fmtAlign = 0;
            while (true)
            {
                if (file->Read(magic, 4) != 4)
                    break;
                file->Seek(4, wxFromCurrent);
                uint32_t size = 0;
                if (file->Read(&size, 4) != 4)
                    break;
                size = bswapu32(size);
                file->Seek(12, wxFromCurrent);

                if (!memcmp(magic, "FMTA", 4))
                {
                    file->Read(&chanCount, 1);
                    file->Read(&fmtAlign, 4);
                    fmtAlign = bswapu32(fmtAlign);
                    file->Seek(size-5, wxFromCurrent);
                }
                else if (!memcmp(magic, "DATA", 4))
                {
                    file->Seek(fmtAlign, wxFromCurrent);
                    if (chanCount == 1)
                    {
                        auto fh =
                        std::make_unique<DSPADPCMCSMPMonoImportFileHandle>(std::move(file), filename, 100.0, false);
                        if (!fh->valid)
                            return {};
                        return fh;
                    }
                    else if (chanCount == 2)
                    {
                        wxFileOffset roff = file->Tell() + (size - fmtAlign) / 2;
                        std::unique_ptr<wxFile> rfile = std::make_unique<wxFile>(filename);
                        if (!rfile->IsOpened())
                            break;
                        rfile->Seek(roff);
                        auto fh =
                        std::make_unique<DSPADPCMStandardStereoImportFileHandle>(std::move(file), std::move(rfile), filename);
                        if (!fh->valid)
                            return {};
                        return fh;
                    }
                    break;
                }
                else
                {
                    file->Seek(size, wxFromCurrent);
                }
            }
        }
        return {};
    }
    else if (!memcmp(magic, "RAS_", 4))
    {
        auto fh =
        std::make_unique<DSPADPCMRASImportFileHandle>(std::move(file), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    file->Seek(0);

    /* Assume standard mono .dsp (check for stereo pair) */
    std::size_t dotpos = filename.rfind('.');
    if (dotpos == wxString::npos)
        dotpos = filename.size();

    if (filename[dotpos-1] == 'L' ||
        filename[dotpos-1] == 'l')
    {
        wxString rFilename = filename;
        rFilename[dotpos-1] = 'R';
        rfile = std::make_unique<wxFile>(rFilename);
        if (!rfile->IsOpened()) {
            rFilename[dotpos-1] = 'r';
            rfile = std::make_unique<wxFile>(rFilename);
            if (!rfile->IsOpened()) {
                rfile.reset();
            }
        }
        lfile = std::move(file);
    }
    else if (filename[dotpos-1] == 'R' ||
             filename[dotpos-1] == 'r')
    {
        wxString lFilename = filename;
        lFilename[dotpos-1] = 'L';
        lfile = std::make_unique<wxFile>(lFilename);
        if (!lfile->IsOpened()) {
            lFilename[dotpos-1] = 'l';
            lfile = std::make_unique<wxFile>(lFilename);
            if (!lfile->IsOpened()) {
                lfile.reset();
            }
        }
        if (!lfile)
            lfile = std::move(file);
        else
            rfile = std::move(file);
    }

    if (rfile && lfile)
    {
        /* Stereo pair import */
        auto fh = std::make_unique<DSPADPCMStandardStereoImportFileHandle>(std::move(lfile), std::move(rfile), filename);
        if (!fh->valid)
            return {};
        return fh;
    }
    else if (rfile)
        file = std::move(rfile);
    else if (lfile)
        file = std::move(lfile);

    /* Regular mono import */
    auto fh = std::make_unique<DSPADPCMStandardMonoImportFileHandle>(std::move(file), filename);
    if (!fh->valid)
        return {};
    return fh;

}

/**************************************************************************
 * Standard Mono
 **************************************************************************/

DSPADPCMStandardMonoImportFileHandle::DSPADPCMStandardMonoImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename), mSampleIndexedLoops(false)
{
    wxASSERT(file);
    mFile = std::move(file);

    /* Read in DSP header(s) here */
    mFile->Read(&mHeader, sizeof(mHeader));
    SwapDSPHeader(&mHeader);

    valid = true;
    if (mHeader.format)
        valid = false;
}

ProgressResult DSPADPCMStandardMonoImportFileHandle::Import(TrackFactory *trackFactory,
                                     TrackHolders &outTracks,
                                     Tags *tags)
{
    outTracks.clear();
    CreateProgress();

    std::unique_ptr<WaveTrack> channel = trackFactory->NewWaveTrack(int16Sample, mHeader.sample_rate);

    size_t fileTotalFrames = mHeader.num_samples;
    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted = 0;
    short hist[2] = {mHeader.hist1, mHeader.hist2};

    auto adpcmBlock = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[4096]);
    unsigned blockFrame = 0;
    short pcmBlock[14];
    while (samplescompleted < mHeader.num_samples)
    {
        unsigned long samplesremaining = mHeader.num_samples - samplescompleted;
        if (!(blockFrame % 4096))
        {
            mFile->Read(adpcmBlock.get(), 4096 * 8);
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
        channel->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
        samplescompleted += s;
        ++blockFrame;

        if (!(samplescompleted % 1024))
        {
            updateResult = mProgress->Update((long long unsigned)samplescompleted,
                                             (long long unsigned)fileTotalFrames);
            if (updateResult != ProgressResult::Success)
                break;
        }
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    channel->Flush();
    outTracks.push_back(std::move(channel));

    /* Add loop label */
    if (mHeader.loop_flag)
    {
        std::unique_ptr<LabelTrack> lt = trackFactory->NewLabelTrack();
        double sr = mHeader.sample_rate;
        if (mSampleIndexedLoops)
            lt->AddLabel(SelectedRegion(
                         mHeader.loop_start / sr,
                         mHeader.loop_end / sr), wxT("LOOP"));
        else
            lt->AddLabel(SelectedRegion(
                         nibbleidx_to_sampleidx(mHeader.loop_start) / sr,
                         nibbleidx_to_sampleidx(mHeader.loop_end) / sr), wxT("LOOP"));
        outTracks.push_back(std::unique_ptr<WaveTrack>(static_cast<WaveTrack*>(static_cast<Track*>(lt.release()))));
    }

    return updateResult;
}

/**************************************************************************
 * Standard Stereo Pair
 **************************************************************************/

DSPADPCMStandardStereoImportFileHandle::DSPADPCMStandardStereoImportFileHandle(std::unique_ptr<wxFile>&& lFile, std::unique_ptr<wxFile>&& rFile, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(lFile);
    wxASSERT(rFile);
    mFile[0] = std::move(lFile);
    mFile[1] = std::move(rFile);

    /* Read in DSP header(s) here */
    mFile[0]->Read(&mHeader[0], sizeof(mHeader[0]));
    SwapDSPHeader(&mHeader[0]);
    mFile[1]->Read(&mHeader[1], sizeof(mHeader[1]));
    SwapDSPHeader(&mHeader[1]);

    valid = true;
    if (mHeader[0].format || mHeader[1].format)
        valid = false;
}

ProgressResult DSPADPCMStandardStereoImportFileHandle::Import(TrackFactory *trackFactory,
                                     TrackHolders &outTracks,
                                     Tags *tags)
{
    CreateProgress();

    std::unique_ptr<WaveTrack> channels[2];

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

    size_t fileTotalFrames = MAX(mHeader[0].num_samples, mHeader[1].num_samples);
    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted[2] = {};
    short hist[2][2] = {{mHeader[0].hist1, mHeader[0].hist2},
                        {mHeader[1].hist1, mHeader[1].hist2}};

    std::unique_ptr<TADPCMFrame[]> adpcmBlock[2];
    adpcmBlock[0] = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[4096]);
    adpcmBlock[1] = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[4096]);
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
                    mFile[c]->Read(adpcmBlock[c].get(), 4096 * 8);
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
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted[c] += s;
                ++blockFrame[c];
            }
        }

        long long unsigned fc = MAX(samplescompleted[0], samplescompleted[1]);
        if (!(fc % 1024))
        {
            updateResult = mProgress->Update((long long unsigned)fc,
                                             (long long unsigned)fileTotalFrames);
            if (updateResult != ProgressResult::Success)
                break;
        }
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for(int c=0 ; c<2 ; c++) {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    /* Add loop label */
    if (mHeader[0].loop_flag)
    {
        std::unique_ptr<LabelTrack> lt = trackFactory->NewLabelTrack();
        double sr = mHeader[0].sample_rate;
        lt->AddLabel(SelectedRegion(
                     nibbleidx_to_sampleidx(mHeader[0].loop_start) / sr,
                     nibbleidx_to_sampleidx(mHeader[0].loop_end) / sr), wxT("LOOP"));
        outTracks.push_back(std::unique_ptr<WaveTrack>(static_cast<WaveTrack*>(static_cast<Track*>(lt.release()))));

    }

    return updateResult;
}

/**************************************************************************
 * RS03 Stereo
 **************************************************************************/

DSPADPCMRS03ImportFileHandle::DSPADPCMRS03ImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);

    /* Read in DSP header(s) here */
    mFile->Read(&mHeader, sizeof(mHeader));
    SwapRS03Header(&mHeader);

    valid = true;
    if (mHeader.format || mHeader.chan_count > 2)
        valid = false;
}

ProgressResult DSPADPCMRS03ImportFileHandle::Import(TrackFactory *trackFactory,
                                     TrackHolders &outTracks,
                                     Tags *tags)
{
    CreateProgress();

    std::unique_ptr<WaveTrack> channels[2];
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

    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted[2] = {};
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    /* Compute number of full-sized blocks */
    unsigned chanFullblocks = mHeader.chan_byte_count / 0x8f00;
    auto adpcmBlock = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[4576]);
    short pcmBlock[14];
    for (int b=0 ; b<chanFullblocks ; ++b)
    {
        for (int c=0 ; c<mHeader.chan_count ; ++c)
        {
            mFile->Read(adpcmBlock.get(), 4576 * 8);
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
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted[c] += s;
            }
        }
        long long unsigned fc = MAX(samplescompleted[0], samplescompleted[1]);
        updateResult = mProgress->Update((long long unsigned)fc,
                                         (long long unsigned)mHeader.num_samples);
        if (updateResult != ProgressResult::Success)
            break;
    }

    if (updateResult == ProgressResult::Success)
    {
        /* Compute last block */
        unsigned chanRemSamples = mHeader.num_samples - chanFullblocks * 0x8f00 * 14 / 8;
        unsigned chanRemBytes = ((chanRemSamples * 8 / 14) + 7) & ~7;
        unsigned chanRemFrames = chanRemBytes / 8;

        for (int c=0 ; c<mHeader.chan_count ; ++c)
        {
            unsigned long samplesremaining = chanRemSamples;
            mFile->Read(adpcmBlock.get(), chanRemBytes);
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
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted[c] += s;
                samplesremaining -= s;
            }
        }
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<mHeader.chan_count ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    /* Add loop label */
    if (mHeader.loop_flag)
    {
        std::unique_ptr<LabelTrack> lt = trackFactory->NewLabelTrack();
        double sr = mHeader.sample_rate;
        lt->AddLabel(SelectedRegion(
                     byteidx_to_sampleidx(mHeader.loop_start) / sr,
                     byteidx_to_sampleidx(mHeader.loop_end) / sr), wxT("LOOP"));
        outTracks.push_back(std::unique_ptr<WaveTrack>(static_cast<WaveTrack*>(static_cast<Track*>(lt.release()))));
    }

    return updateResult;
}

/**************************************************************************
 * FSB 3.1 Stereo
 **************************************************************************/

DSPADPCMFSB31ImportFileHandle::DSPADPCMFSB31ImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);
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

ProgressResult DSPADPCMFSB31ImportFileHandle::Import(TrackFactory *trackFactory,
                                     TrackHolders &outTracks,
                                     Tags *tags)
{
    CreateProgress();

    std::unique_ptr<WaveTrack> channels[2];
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

    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted = 0;
    unsigned long samplesremaining = mHeader.sampleCount;
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    if (mHeader.channelCount == 2)
    {
        short pcmBlock[2][14];
        unsigned readBlocks = (mHeader.dataSize + 8191) / 8192;
        unsigned remFrames = (mHeader.dataSize + 15) / 16;
        auto adpcmBlock = std::unique_ptr<TADPCMStereoFrame[]>(new TADPCMStereoFrame[512]);
        for (int b=0 ; b<readBlocks ; ++b)
        {
            mFile->Read(adpcmBlock.get(), 8192);
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
                    channels[c]->Append((samplePtr)pcmBlock[c], int16Sample, (size_t)s);
                }
                samplescompleted += s;
                samplesremaining -= s;
            }
            updateResult = mProgress->Update((long long unsigned)samplescompleted,
                                             (long long unsigned)mHeader.sampleCount);
            if (updateResult != ProgressResult::Success)
                break;
            remFrames -= 512;
        }
    }
    else if (mHeader.channelCount == 1)
    {
        short pcmBlock[14];
        unsigned readBlocks = (mHeader.dataSize + 4095) / 4096;
        unsigned remFrames = (mHeader.dataSize + 7) / 8;
        auto adpcmBlock = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[512]);
        for (int b=0 ; b<readBlocks ; ++b)
        {
            mFile->Read(adpcmBlock.get(), 4096);
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
                channels[0]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted += s;
                samplesremaining -= s;
            }
            updateResult = mProgress->Update((long long unsigned)samplescompleted,
                                             (long long unsigned)mHeader.sampleCount);
            if (updateResult != ProgressResult::Success)
                break;
            remFrames -= 512;
        }
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<mHeader.channelCount ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    /* Add loop label */
    if (mHeader.mode & FSOUND_LOOP_NORMAL)
    {
        std::unique_ptr<LabelTrack> lt = trackFactory->NewLabelTrack();
        double sr = mHeader.defaultFreq;
        lt->AddLabel(SelectedRegion(
                     mHeader.loopStart / sr,
                     mHeader.loopEnd / sr), wxT("LOOP"));
        outTracks.push_back(std::unique_ptr<WaveTrack>(static_cast<WaveTrack*>(static_cast<Track*>(lt.release()))));
    }

    return updateResult;
}

/**************************************************************************
 * RAS Stereo
 **************************************************************************/

DSPADPCMRASImportFileHandle::DSPADPCMRASImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);
    valid = false;

    mFile->Read(&mHeader, sizeof(mHeader));
    SwapRASHeader(&mHeader);

    for (int c=0 ; c<mHeader.chanCount1 ; ++c)
    {
        mFile->Read(&mCoefs[c], sizeof(ras_dspadpcm_channel));
        SwapRASChannel(&mCoefs[c]);
    }

    memset(&mMeta, 0, sizeof(ras_track_meta));
    if (mHeader.metaDataFlag)
    {
        mFile->Read(&mMeta, sizeof(ras_track_meta));
        SwapRASTrackMeta(&mMeta);
    }

    mFile->Seek(mHeader.dataOffset);
    if (mHeader.chanCount1 == 2)
        valid = true;
}

ProgressResult DSPADPCMRASImportFileHandle::Import(TrackFactory *trackFactory,
                                     TrackHolders &outTracks,
                                     Tags *tags)
{
    CreateProgress();

    std::unique_ptr<WaveTrack> channels[2];
    for (int c=0 ; c<mHeader.chanCount1 ; ++c)
    {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, mHeader.sampleRate);
        if (mHeader.chanCount1 == 2)
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
    if (mHeader.chanCount1 == 2)
        channels[0]->SetLinked(true);

    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted[2] = {};
    unsigned long samplesremaining[2] = {mHeader.numSamples, mHeader.numSamples};
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    unsigned framesPerBlock = mHeader.blockSize / 8;
    auto adpcmBlock = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[framesPerBlock]);
    for (int b=0 ; b<mHeader.numBlocks ; ++b)
    {
        for (int c=0 ; c<mHeader.chanCount1 ; ++c)
        {
            mFile->Read(adpcmBlock.get(), mHeader.blockSize);
            for (int f=0 ; f<framesPerBlock ; ++f)
            {
                short pcmBlock[14];
                unsigned char cIdx = (adpcmBlock[f][0]>>4) & 0xf;
                short factor1 = mCoefs[c].coefs[cIdx*2];
                short factor2 = mCoefs[c].coefs[cIdx*2+1];
                unsigned char exp = adpcmBlock[f][0] & 0xf;
                int s;
                for (s=0 ; s<14 && s<samplesremaining[c] ; ++s) {
                    int sample_byte_idx = s/2+1;
                    int sample_data = (s&1)?
                    nibble_to_int[(adpcmBlock[f][sample_byte_idx])&0xf]:
                    nibble_to_int[(adpcmBlock[f][sample_byte_idx]>>4)&0xf];
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
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted[c] += s;
                samplesremaining[c] -= s;
            }
        }
        updateResult = mProgress->Update((long long unsigned)samplescompleted[0],
                                         (long long unsigned)mHeader.numSamples);
        if (updateResult != ProgressResult::Success)
            break;
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<mHeader.chanCount1 ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    /* Add loop label */
    if (mHeader.loopEndBlock || mHeader.loopEndSample)
    {
        std::unique_ptr<LabelTrack> lt = trackFactory->NewLabelTrack();
        double sr = mHeader.sampleRate;
        lt->AddLabel(SelectedRegion(
                     (mHeader.loopStartBlock * framesPerBlock * 14 + mHeader.loopStartSample) / sr,
                     (mHeader.loopEndBlock * framesPerBlock * 14 + mHeader.loopEndSample) / sr), wxT("LOOP"));
        outTracks.push_back(std::unique_ptr<WaveTrack>(static_cast<WaveTrack*>(static_cast<Track*>(lt.release()))));
    }

    /* Add BPM metadata */
    if (mMeta.bpmFlag)
        tags->SetTag(wxT("BPM"), wxString::Format(wxT("%g"), mMeta.bpm));

    return updateResult;
}

/**************************************************************************
 * STM Stereo
 **************************************************************************/

DSPADPCMSTMImportFileHandle::DSPADPCMSTMImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);
    valid = false;

    mFile->Read(&mHeader, sizeof(mHeader));
    SwapSTMHeader(&mHeader);
    if (mHeader.field00 != 512 || mHeader.numChannels == 0)
        return;
    valid = true;

    mDSPHeaders.reserve(mHeader.numChannels);
    for (int c=0 ; c<mHeader.numChannels ; ++c)
    {
        mDSPHeaders.emplace_back();
        mFile->Read(&mDSPHeaders.back(), sizeof(dspadpcm_header));
        SwapDSPHeader(&mDSPHeaders.back());
    }
}

ProgressResult DSPADPCMSTMImportFileHandle::Import(TrackFactory *trackFactory, TrackHolders& outTracks, Tags *tags)
{
    CreateProgress();

    std::vector<std::unique_ptr<WaveTrack>> channels;
    channels.reserve(mHeader.numChannels);
    for (int c=0 ; c<mHeader.numChannels ; ++c)
    {
        channels.push_back(trackFactory->NewWaveTrack(int16Sample, mHeader.sampleRate));
        if (mHeader.numChannels == 2)
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
    if (mHeader.numChannels == 2)
        channels[0]->SetLinked(true);

    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted[2] = {};
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    /* Compute number of full-sized blocks */
    auto adpcmBlock = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[4096]);
    short pcmBlock[14];
    for (int c=0 ; c<mHeader.numChannels ; ++c)
    {
        if (c == 1)
            mFile->Seek(64 + 96 * mHeader.numChannels + ROUND_UP_32(mHeader.adpcmData2Offset));
        while (samplescompleted[c] < mDSPHeaders[c].num_samples)
        {
            int remSamples = mDSPHeaders[c].num_samples - samplescompleted[c];
            int loadBlks = MIN(4096, (remSamples + 13) / 14);
            mFile->Read(adpcmBlock.get(), loadBlks * 8);
            for (int f=0 ; f<loadBlks ; ++f)
            {
                unsigned char cIdx = (adpcmBlock[f][0]>>4) & 0xf;
                short factor1 = mDSPHeaders[c].coef[cIdx*2];
                short factor2 = mDSPHeaders[c].coef[cIdx*2+1];
                unsigned char exp = adpcmBlock[f][0] & 0xf;
                int s;
                for (s=0 ; s<14 && s<remSamples ; ++s) {
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
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted[c] += s;
                remSamples -= s;
            }
        }
    }
    long long unsigned fc = MAX(samplescompleted[0], samplescompleted[1]);
    updateResult = mProgress->Update((long long unsigned)fc,
                                     (long long unsigned)mDSPHeaders[0].num_samples);

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<mHeader.numChannels ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    /* Add loop label */
    if (mHeader.adpcmLoopStartOffset != 0xffffffff)
    {
        std::unique_ptr<LabelTrack> lt = trackFactory->NewLabelTrack();
        double sr = mHeader.sampleRate;
        lt->AddLabel(SelectedRegion(
                     byteidx_to_sampleidx(mHeader.adpcmLoopStartOffset) / sr,
                     byteidx_to_sampleidx(mHeader.adpcmData2OffsetAux1) / sr), wxT("LOOP"));
        outTracks.push_back(std::unique_ptr<WaveTrack>(static_cast<WaveTrack*>(static_cast<Track*>(lt.release()))));
    }

    return updateResult;
}

/**************************************************************************
 * BCWAV Stereo
 **************************************************************************/

DSPADPCMBCWAVImportFileHandle::DSPADPCMBCWAVImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);
    valid = false;

    bcwav_header header;
    mFile->Read(&header, sizeof(header));

    if (header.magic != 0x56415743)
        return;

    mFile->Seek(header.infoRef.ref.offset);
    mFile->Read(&mInfo, sizeof(mInfo));
    if (mInfo.enc != 2)
        return;

    if (!mInfo.chanCount)
        return;

    valid = true;

    std::vector<uint32_t> chanOffs;
    chanOffs.reserve(mInfo.chanCount);
    mChannels.reserve(mInfo.chanCount);
    for (int c=0 ; c<mInfo.chanCount ; ++c)
    {
        bcwav_reference chanRef;
        mFile->Read(&chanRef, sizeof(chanRef));
        chanOffs.push_back(header.infoRef.ref.offset + 28 + chanRef.offset);
    }

    for (int c=0 ; c<mInfo.chanCount ; ++c)
    {
        bcwav_channel_info chanInfo;
        mFile->Seek(chanOffs[c]);
        mFile->Read(&chanInfo, sizeof(chanInfo));

        mChannels.emplace_back();
        mChannels[c].first = header.dataRef.ref.offset + 8 + chanInfo.samplesRef.offset;
        mFile->Seek(chanOffs[c] + chanInfo.adpcmInfoRef.offset);
        mFile->Read(&mChannels[c].second, sizeof(bcwav_dspadpcm_info));
    }
}

ProgressResult DSPADPCMBCWAVImportFileHandle::Import(TrackFactory *trackFactory, TrackHolders& outTracks, Tags *tags)
{
    CreateProgress();

    std::vector<std::unique_ptr<WaveTrack>> channels;
    channels.reserve(mInfo.chanCount);
    for (int c=0 ; c<mInfo.chanCount ; ++c)
    {
        channels.push_back(trackFactory->NewWaveTrack(int16Sample, mInfo.sampleRate));
        if (mInfo.chanCount == 2)
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
    if (mInfo.chanCount == 2)
        channels[0]->SetLinked(true);

    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted[2] = {};
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    /* Compute number of full-sized blocks */
    auto adpcmBlock = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[4096]);
    short pcmBlock[14];
    for (int c=0 ; c<mInfo.chanCount ; ++c)
    {
        mFile->Seek(mChannels[c].first);
        while (samplescompleted[c] < mInfo.loopEnd)
        {
            int remSamples = mInfo.loopEnd - samplescompleted[c];
            int loadBlks = MIN(4096, (remSamples + 13) / 14);
            mFile->Read(adpcmBlock.get(), loadBlks * 8);
            for (int f=0 ; f<loadBlks ; ++f)
            {
                unsigned char cIdx = (adpcmBlock[f][0]>>4) & 0xf;
                short factor1 = mChannels[c].second.coef[cIdx*2];
                short factor2 = mChannels[c].second.coef[cIdx*2+1];
                unsigned char exp = adpcmBlock[f][0] & 0xf;
                int s;
                for (s=0 ; s<14 && s<remSamples ; ++s) {
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
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted[c] += s;
                remSamples -= s;
            }
        }
    }
    long long unsigned fc = MAX(samplescompleted[0], samplescompleted[1]);
    updateResult = mProgress->Update((long long unsigned)fc,
                                     (long long unsigned)mInfo.loopEnd);

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<mInfo.chanCount ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    /* Add loop label */
    if (mInfo.loop)
    {
        std::unique_ptr<LabelTrack> lt = trackFactory->NewLabelTrack();
        double sr = mInfo.sampleRate;
        lt->AddLabel(SelectedRegion(
                     mInfo.loopStart / sr,
                     mInfo.loopEnd / sr), wxT("LOOP"));
        outTracks.push_back(std::unique_ptr<WaveTrack>(static_cast<WaveTrack*>(static_cast<Track*>(lt.release()))));
    }

    return updateResult;
}

/**************************************************************************
 * GCub Stereo
 **************************************************************************/

DSPADPCMGCUBImportFileHandle::DSPADPCMGCUBImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
: DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);

    /* Read in DSP header(s) here */
    mFile->Read(&mHeader, sizeof(mHeader));
    SwapGCubHeader(&mHeader);

    uint32_t subChunkCheck;
    mFile->Seek(0x60, wxFromStart);
    mFile->Read(&subChunkCheck, 4);
    if (subChunkCheck == 0x78784347)
        mStartOffset = 0x88;
    else
        mStartOffset = 0x60;

    mSampleCount = (mHeader.endOffset - mStartOffset) / 8 / mHeader.chanCount * 14;

    valid = true;
    if (mHeader.chanCount > 2)
        valid = false;
}

ProgressResult DSPADPCMGCUBImportFileHandle::Import(TrackFactory *trackFactory,
                                         TrackHolders &outTracks,
                                         Tags *tags)
{
    CreateProgress();

    std::unique_ptr<WaveTrack> channels[2];
    for (int c=0 ; c<mHeader.chanCount ; ++c)
    {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, mHeader.sampleRate);
        if (mHeader.chanCount == 2)
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
    if (mHeader.chanCount == 2)
        channels[0]->SetLinked(true);

    ProgressResult updateResult = ProgressResult::Success;

    unsigned long samplescompleted[2] = {};
    short hist[2][2] = {{0, 0},
                        {0, 0}};

    mFile->Seek(mStartOffset, wxFromStart);

    /* Compute number of full-sized blocks */
    unsigned chanFullblocks = (mSampleCount + (4096 * 14 - 1)) / (4096 * 14);
    auto adpcmBlock = std::unique_ptr<TADPCMFrame[]>(new TADPCMFrame[4096]);
    short pcmBlock[14];
    for (int b=0 ; b<chanFullblocks ; ++b)
    {
        for (int c=0 ; c<mHeader.chanCount ; ++c)
        {
            mFile->Read(adpcmBlock.get(), 4096 * 8);
            for (int f=0 ; f<4096 ; ++f)
            {
                unsigned long samplesrem = mSampleCount - samplescompleted[c];
                if (!samplesrem)
                    break;

                unsigned char cIdx = (adpcmBlock[f][0]>>4) & 0xf;
                short factor1 = mHeader.coef[c][cIdx*2];
                short factor2 = mHeader.coef[c][cIdx*2+1];
                unsigned char exp = adpcmBlock[f][0] & 0xf;
                int s;
                for (s=0 ; s<14 && s<samplesrem ; ++s) {
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
                channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
                samplescompleted[c] += s;
            }
        }
        long long unsigned fc = MAX(samplescompleted[0], samplescompleted[1]);
        updateResult = mProgress->Update((long long unsigned)fc,
                                         (long long unsigned)mSampleCount);
        if (updateResult != ProgressResult::Success)
            break;
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<mHeader.chanCount ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    return updateResult;
}

/**************************************************************************
 * RSF Stereo
 **************************************************************************/

struct g72x_state {
    long yl;    /* Locked or steady state step size multiplier. */
    short yu;   /* Unlocked or non-steady state step size multiplier. */
    short dms;  /* Short term energy estimate. */
    short dml;  /* Long term energy estimate. */
    short ap;   /* Linear weighting coefficient of 'yl' and 'yu'. */

    short a[2]; /* Coefficients of pole portion of prediction filter. */
    short b[6]; /* Coefficients of zero portion of prediction filter. */
    short pk[2];    /*
             * Signs of previous two samples of a partially
             * reconstructed signal.
             */
    short dq[6];    /*
             * Previous 6 samples of the quantized difference
             * signal represented in an internal floating point
             * format.
             */
    short sr[2];    /*
             * Previous 2 samples of the quantized difference
             * signal represented in an internal floating point
             * format.
             */
    char td;    /* delayed tone detect, new in 1988 version */
};

static short power2[15] = {1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80,
                0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000};

/*
 * quan()
 *
 * quantizes the input val against the table of size short integers.
 * It returns i if table[i - 1] <= val < table[i].
 *
 * Using linear search for simple coding.
 */
static int
quan(
    int     val,
    short   *table,
    int     size)
{
    int     i;

    for (i = 0; i < size; i++)
        if (val < *table++)
            break;
    return (i);
}

/*
 * fmult()
 *
 * returns the integer product of the 14-bit integer "an" and
 * "floating point" representation (4-bit exponent, 6-bit mantessa) "srn".
 */
static int
fmult(
    int		an,
    int		srn)
{
    short		anmag, anexp, anmant;
    short		wanexp, wanmant;
    short		retval;

    anmag = (an > 0) ? an : ((-an) & 0x1FFF);
    anexp = quan(anmag, power2, 15) - 6;
    anmant = (anmag == 0) ? 32 :
        (anexp >= 0) ? anmag >> anexp : anmag << -anexp;
    wanexp = anexp + ((srn >> 6) & 0xF) - 13;

    wanmant = (anmant * (srn & 077) + 0x30) >> 4;
    retval = (wanexp >= 0) ? ((wanmant << wanexp) & 0x7FFF) :
        (wanmant >> -wanexp);

    return (((an ^ srn) < 0) ? -retval : retval);
}

/*
 * g72x_init_state()
 *
 * This routine initializes and/or resets the g72x_state structure
 * pointed to by 'state_ptr'.
 * All the initial state values are specified in the CCITT G.721 document.
 */
static void
g72x_init_state(
    struct g72x_state *state_ptr)
{
    int		cnta;

    state_ptr->yl = 34816;
    state_ptr->yu = 544;
    state_ptr->dms = 0;
    state_ptr->dml = 0;
    state_ptr->ap = 0;
    for (cnta = 0; cnta < 2; cnta++) {
        state_ptr->a[cnta] = 0;
        state_ptr->pk[cnta] = 0;
        state_ptr->sr[cnta] = 32;
    }
    for (cnta = 0; cnta < 6; cnta++) {
        state_ptr->b[cnta] = 0;
        state_ptr->dq[cnta] = 32;
    }
    state_ptr->td = 0;
}

/*
 * predictor_zero()
 *
 * computes the estimated signal from 6-zero predictor.
 *
 */
static int
predictor_zero(
    struct g72x_state *state_ptr)
{
    int		i;
    int		sezi;

    sezi = fmult(state_ptr->b[0] >> 2, state_ptr->dq[0]);
    for (i = 1; i < 6; i++)			/* ACCUM */
        sezi += fmult(state_ptr->b[i] >> 2, state_ptr->dq[i]);
    return (sezi);
}
/*
 * predictor_pole()
 *
 * computes the estimated signal from 2-pole predictor.
 *
 */
static int
predictor_pole(
    struct g72x_state *state_ptr)
{
    return (fmult(state_ptr->a[1] >> 2, state_ptr->sr[1]) +
        fmult(state_ptr->a[0] >> 2, state_ptr->sr[0]));
}
/*
 * step_size()
 *
 * computes the quantization step size of the adaptive quantizer.
 *
 */
static int
step_size(
    struct g72x_state *state_ptr)
{
    int		y;
    int		dif;
    int		al;

    if (state_ptr->ap >= 256)
        return (state_ptr->yu);
    else {
        y = state_ptr->yl >> 6;
        dif = state_ptr->yu - y;
        al = state_ptr->ap >> 2;
        if (dif > 0)
            y += (dif * al) >> 6;
        else if (dif < 0)
            y += (dif * al + 0x3F) >> 6;
        return (y);
    }
}

/*
 * reconstruct()
 *
 * Returns reconstructed difference signal 'dq' obtained from
 * codeword 'i' and quantization step size scale factor 'y'.
 * Multiplication is performed in log base 2 domain as addition.
 */
static int
reconstruct(
    int		sign,	/* 0 for non-negative value */
    int		dqln,	/* G.72x codeword */
    int		y)	/* Step size multiplier */
{
    short		dql;	/* Log of 'dq' magnitude */
    short		dex;	/* Integer part of log */
    short		dqt;
    short		dq;	/* Reconstructed difference signal sample */

    dql = dqln + (y >> 2);	/* ADDA */

    if (dql < 0) {
        return ((sign) ? -0x8000 : 0);
    } else {		/* ANTILOG */
        dex = (dql >> 7) & 15;
        dqt = 128 + (dql & 127);
        dq = (dqt << 7) >> (14 - dex);
        return ((sign) ? (dq - 0x8000) : dq);
    }
}


/*
 * update()
 *
 * updates the state variables for each output code
 */
static void
update(
    /*int		code_size,*/	/* distinguish 723_40 with others */
    int		y,		/* quantizer step size */
    int		wi,		/* scale factor multiplier */
    int		fi,		/* for long/short term energies */
    int		dq,		/* quantized prediction difference */
    int		sr,		/* reconstructed signal */
    int		dqsez,		/* difference from 2-pole predictor */
    struct g72x_state *state_ptr)	/* coder state pointer */
{
    int		cnt;
    short		mag, exp;	/* Adaptive predictor, FLOAT A */
    short		a2p;		/* LIMC */
    short		a1ul;		/* UPA1 */
    short		pks1;	/* UPA2 */
    short		fa1;
    char		tr;		/* tone/transition detector */
    short		ylint, thr2, dqthr;
    short  		ylfrac, thr1;
    short		pk0;

    pk0 = (dqsez < 0) ? 1 : 0;	/* needed in updating predictor poles */

    mag = dq & 0x7FFF;		/* prediction difference magnitude */
    /* TRANS */
    ylint = state_ptr->yl >> 15;	/* exponent part of yl */
    ylfrac = (state_ptr->yl >> 10) & 0x1F;	/* fractional part of yl */
    thr1 = (32 + ylfrac) << ylint;		/* threshold */
    thr2 = (ylint > 9) ? 31 << 10 : thr1;	/* limit thr2 to 31 << 10 */
    dqthr = (thr2 + (thr2 >> 1)) >> 1;	/* dqthr = 0.75 * thr2 */
    if (state_ptr->td == 0)		/* signal supposed voice */
        tr = 0;
    else if (mag <= dqthr)		/* supposed data, but small mag */
        tr = 0;			/* treated as voice */
    else				/* signal is data (modem) */
        tr = 1;

    /*
     * Quantizer scale factor adaptation.
     */

    /* FUNCTW & FILTD & DELAY */
    /* update non-steady state step size multiplier */
    state_ptr->yu = y + ((wi - y) >> 5);

    /* LIMB */
    if (state_ptr->yu < 544)	/* 544 <= yu <= 5120 */
        state_ptr->yu = 544;
    else if (state_ptr->yu > 5120)
        state_ptr->yu = 5120;

    /* FILTE & DELAY */
    /* update steady state step size multiplier */
    state_ptr->yl += state_ptr->yu + ((-state_ptr->yl) >> 6);

    /*
     * Adaptive predictor coefficients.
     */
    if (tr == 1) {			/* reset a's and b's for modem signal */
        state_ptr->a[0] = 0;
        state_ptr->a[1] = 0;
        state_ptr->b[0] = 0;
        state_ptr->b[1] = 0;
        state_ptr->b[2] = 0;
        state_ptr->b[3] = 0;
        state_ptr->b[4] = 0;
        state_ptr->b[5] = 0;
        a2p=0;          /* won't be used, clear warning */
    } else {			/* update a's and b's */
        pks1 = pk0 ^ state_ptr->pk[0];		/* UPA2 */

        /* update predictor pole a[1] */
        a2p = state_ptr->a[1] - (state_ptr->a[1] >> 7);
        if (dqsez != 0) {
            fa1 = (pks1) ? state_ptr->a[0] : -state_ptr->a[0];
            if (fa1 < -8191)	/* a2p = function of fa1 */
                a2p -= 0x100;
            else if (fa1 > 8191)
                a2p += 0xFF;
            else
                a2p += fa1 >> 5;

            if (pk0 ^ state_ptr->pk[1])
                /* LIMC */
                if (a2p <= -12160)
                    a2p = -12288;
                else if (a2p >= 12416)
                    a2p = 12288;
                else
                    a2p -= 0x80;
            else if (a2p <= -12416)
                a2p = -12288;
            else if (a2p >= 12160)
                a2p = 12288;
            else
                a2p += 0x80;
        }

        /* TRIGB & DELAY */
        state_ptr->a[1] = a2p;

        /* UPA1 */
        /* update predictor pole a[0] */
        state_ptr->a[0] -= state_ptr->a[0] >> 8;
        if (dqsez != 0) {
            if (pks1 == 0)
                state_ptr->a[0] += 192;
            else
                state_ptr->a[0] -= 192;
        }

        /* LIMD */
        a1ul = 15360 - a2p;
        if (state_ptr->a[0] < -a1ul)
            state_ptr->a[0] = -a1ul;
        else if (state_ptr->a[0] > a1ul)
            state_ptr->a[0] = a1ul;

        /* UPB : update predictor zeros b[6] */
        for (cnt = 0; cnt < 6; cnt++) {
            /*if (code_size == 5)*/		/* for 40Kbps G.723 */
            /*	state_ptr->b[cnt] -= state_ptr->b[cnt] >> 9;*/
            /*else*/			/* for G.721 and 24Kbps G.723 */
                state_ptr->b[cnt] -= state_ptr->b[cnt] >> 8;
            if (dq & 0x7FFF) {			/* XOR */
                if ((dq ^ state_ptr->dq[cnt]) >= 0)
                    state_ptr->b[cnt] += 128;
                else
                    state_ptr->b[cnt] -= 128;
            }
        }
    }

    for (cnt = 5; cnt > 0; cnt--)
        state_ptr->dq[cnt] = state_ptr->dq[cnt-1];
    /* FLOAT A : convert dq[0] to 4-bit exp, 6-bit mantissa f.p. */
    if (mag == 0) {
        state_ptr->dq[0] = (dq >= 0) ? 0x20 : 0xFC20;
    } else {
        exp = quan(mag, power2, 15);
        state_ptr->dq[0] = (dq >= 0) ?
            (exp << 6) + ((mag << 6) >> exp) :
            (exp << 6) + ((mag << 6) >> exp) - 0x400;
    }

    state_ptr->sr[1] = state_ptr->sr[0];
    /* FLOAT B : convert sr to 4-bit exp., 6-bit mantissa f.p. */
    if (sr == 0) {
        state_ptr->sr[0] = 0x20;
    } else if (sr > 0) {
        exp = quan(sr, power2, 15);
        state_ptr->sr[0] = (exp << 6) + ((sr << 6) >> exp);
    } else if (sr > -32768) {
        mag = -sr;
        exp = quan(mag, power2, 15);
        state_ptr->sr[0] =  (exp << 6) + ((mag << 6) >> exp) - 0x400;
    } else
        state_ptr->sr[0] = 0xFC20;

    /* DELAY A */
    state_ptr->pk[1] = state_ptr->pk[0];
    state_ptr->pk[0] = pk0;

    /* TONE */
    if (tr == 1)		/* this sample has been treated as data */
        state_ptr->td = 0;	/* next one will be treated as voice */
    else if (a2p < -11776)	/* small sample-to-sample correlation */
        state_ptr->td = 1;	/* signal may be data */
    else				/* signal is voice */
        state_ptr->td = 0;

    /*
     * Adaptation speed control.
     */
    state_ptr->dms += (fi - state_ptr->dms) >> 5;		/* FILTA */
    state_ptr->dml += (((fi << 2) - state_ptr->dml) >> 7);	/* FILTB */

    if (tr == 1)
        state_ptr->ap = 256;
    else if (y < 1536)					/* SUBTC */
        state_ptr->ap += (0x200 - state_ptr->ap) >> 4;
    else if (state_ptr->td == 1)
        state_ptr->ap += (0x200 - state_ptr->ap) >> 4;
    else if (abs((state_ptr->dms << 2) - state_ptr->dml) >=
        (state_ptr->dml >> 3))
        state_ptr->ap += (0x200 - state_ptr->ap) >> 4;
    else
        state_ptr->ap += (-state_ptr->ap) >> 4;
}

/*
 * Maps G.721 code word to reconstructed scale factor normalized log
 * magnitude values.
 */
static short	_dqlntab[16] = {-2048, 4, 135, 213, 273, 323, 373, 425,
                425, 373, 323, 273, 213, 135, 4, -2048};

/* Maps G.721 code word to log of scale factor multiplier. */
static short	_witab[16] = {-12, 18, 41, 64, 112, 198, 355, 1122,
                1122, 355, 198, 112, 64, 41, 18, -12};
/*
 * Maps G.721 code words to a set of values whose long and short
 * term averages are computed and then compared to give an indication
 * how stationary (steady state) the signal is.
 */
static short	_fitab[16] = {0, 0, 0, 0x200, 0x200, 0x200, 0x600, 0xE00,
                0xE00, 0x600, 0x200, 0x200, 0x200, 0, 0, 0};

/*
 * g721_decoder()
 *
 * Description:
 *
 * Decodes a 4-bit code of G.721 encoded data of i and
 * returns the resulting linear PCM, A-law or u-law value.
 * return -1 for unknown out_coding value.
 */
static int
g721_decoder(
    int		i,
    struct g72x_state *state_ptr)
{
    short		sezi, sei, sez, se;	/* ACCUM */
    short		y;			/* MIX */
    short		sr;			/* ADDB */
    short		dq;
    short		dqsez;

    i &= 0x0f;			/* mask to get proper bits */
    sezi = predictor_zero(state_ptr);
    sez = sezi >> 1;
    sei = sezi + predictor_pole(state_ptr);
    se = sei >> 1;			/* se = estimated signal */

    y = step_size(state_ptr);	/* dynamic quantizer step size */

    dq = reconstruct(i & 0x08, _dqlntab[i], y); /* quantized diff. */

    sr = (dq < 0) ? (se - (dq & 0x3FFF)) : se + dq;	/* reconst. signal */

    dqsez = sr - se + sez;			/* pole prediction diff. */

    update(y, _witab[i] << 5, _fitab[i], dq, sr, dqsez, state_ptr);

    return (sr << 2);	/* sr was 14-bit dynamic range */
}

DSPADPCMRSFImportFileHandle::DSPADPCMRSFImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);
    valid = true;

    mFile->Seek(0, wxFromEnd);
    mNumSamples = mFile->Tell();
    mFile->Seek(0);
}

ProgressResult DSPADPCMRSFImportFileHandle::Import(TrackFactory *trackFactory,
                                     TrackHolders &outTracks,
                                     Tags *tags)
{
    CreateProgress();

    std::unique_ptr<WaveTrack> channels[2];
    for (int c=0 ; c<2 ; ++c)
    {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, 32000);
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

    ProgressResult updateResult = ProgressResult::Success;

    unsigned numBlocks = (mNumSamples / 2 + 4095) / 4096;
    unsigned completedSamples = 0;
    for (int c=0 ; c<2 ; ++c)
    {
        mFile->Seek((mNumSamples / 2) * c);

        struct g72x_state state;
        g72x_init_state(&state);

        char adpcmBlock[4096];
        short pcmBlock[8192];
        unsigned remSamples = mNumSamples;
        for (int b=0 ; b<numBlocks ; ++b)
        {
            mFile->Read(adpcmBlock, 4096);
            int s;
            for (s=0 ; s<8192 && s<remSamples ; ++s)
                pcmBlock[s] = g721_decoder((s&1) ? adpcmBlock[s/2]>>4 : adpcmBlock[s/2], &state);

            channels[c]->Append((samplePtr)pcmBlock, int16Sample, (size_t)s);
            completedSamples += s;
            remSamples -= s;
            updateResult = mProgress->Update((long long unsigned)completedSamples,
                                             (long long unsigned)mNumSamples * 2);
            if (updateResult != ProgressResult::Success)
                break;
        }

        if (updateResult != ProgressResult::Success)
            break;
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<2 ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    return updateResult;
}

/**************************************************************************
 * ADP Stereo
 **************************************************************************/

#define ADP_ONE_BLOCK_SIZE 32
#define ADP_SAMPLES_PER_BLOCK 28

static short ADPDecodeSample(int32_t bits, int32_t q, int32_t& hist1, int32_t& hist2)
{
    int32_t hist = 0;
    switch (q >> 4)
    {
    case 0:
        hist = 0;
        break;
    case 1:
        hist = (hist1 * 0x3c);
        break;
    case 2:
        hist = (hist1 * 0x73) - (hist2 * 0x34);
        break;
    case 3:
        hist = (hist1 * 0x62) - (hist2 * 0x37);
        break;
    }
    hist = (hist + 0x20) >> 6;
    if (hist < -0x200000)
        hist = -0x200000;
    else if (hist > 0x1fffff)
        hist = 0x1fffff;

    int32_t cur = (((int16_t)(bits << 12) >> (q & 0xf)) << 6) + hist;

    hist2 = hist1;
    hist1 = cur;

    cur >>= 6;
    if (cur < -0x8000)
        cur = -0x8000;
    else if (cur > 0x7fff)
        cur = 0x7fff;

    return (int16_t)cur;
}

DSPADPCMADPImportFileHandle::DSPADPCMADPImportFileHandle(std::unique_ptr<wxFile>&& file, const wxString& filename)
    :  DSPADPCMBaseImportFileHandle(filename)
{
    wxASSERT(file);
    mFile = std::move(file);
    valid = true;

    mFile->Seek(0, wxFromEnd);
    mNumBlocks = mFile->Tell() / ADP_ONE_BLOCK_SIZE;
    mFile->Seek(0);
}

ProgressResult DSPADPCMADPImportFileHandle::Import(TrackFactory *trackFactory,
                                     TrackHolders &outTracks,
                                     Tags *tags)
{
    CreateProgress();

    std::unique_ptr<WaveTrack> channels[2];
    for (int c=0 ; c<2 ; ++c)
    {
        channels[c] = trackFactory->NewWaveTrack(int16Sample, 48000);
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

    ProgressResult updateResult = ProgressResult::Success;

    unsigned completedSamples = 0;

    int32_t histl1 = 0;
    int32_t histl2 = 0;
    int32_t histr1 = 0;
    int32_t histr2 = 0;

    struct SADPBlock
    {
        unsigned char b[ADP_ONE_BLOCK_SIZE];
    };
    auto blocks = std::unique_ptr<SADPBlock[]>(new SADPBlock[mNumBlocks]);
    mFile->Read(blocks.get(), ADP_ONE_BLOCK_SIZE * mNumBlocks);

    short pcmBlock[2][ADP_SAMPLES_PER_BLOCK];
    for (int b=0 ; b<mNumBlocks ; ++b)
    {
        int s;
        for (s=0 ; s<ADP_SAMPLES_PER_BLOCK ; ++s)
        {
            pcmBlock[0][s] = ADPDecodeSample(blocks[b].b[4+s] & 0xf, blocks[b].b[0], histl1, histl2);
            pcmBlock[1][s] = ADPDecodeSample(blocks[b].b[4+s] >> 4, blocks[b].b[1], histr1, histr2);
        }
        channels[0]->Append((samplePtr)pcmBlock[0], int16Sample, (size_t)s);
        channels[1]->Append((samplePtr)pcmBlock[1], int16Sample, (size_t)s);
        completedSamples += s;

        if (!(b%64))
        {
            updateResult = mProgress->Update((long long unsigned)b,
                                             (long long unsigned)mNumBlocks);
            if (updateResult != ProgressResult::Success)
                break;
        }
    }

    if (updateResult == ProgressResult::Failed || updateResult == ProgressResult::Cancelled)
    {
        return updateResult;
    }

    for (int c=0 ; c<2 ; ++c)
    {
        channels[c]->Flush();
        outTracks.push_back(std::move(channels[c]));
    }

    return updateResult;
}

