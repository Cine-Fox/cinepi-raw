/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * Based on mjpeg_encoder.cpp, modifications by Csaba Nagy & Will Whang 
 *
 * dng_encoder.cpp - dng video encoder.
 */

#include <chrono>
#include <iostream>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <tiffio.h>
#include <tiffio.hxx>
#include <sstream>
#include <fstream>
#include <regex>

#include "core/still_options.hpp"
#include "core/stream_info.hpp"

#include "dng_encoder.hpp"
#include <arm_neon.h>
#include "utils.hpp"

// extern "C" {
// #include "lj92.h"
// }

#include "yuv2rgb.hpp"

#include <filesystem>
namespace fs = std::filesystem;

#define ONE_MB 1048576
#define BLOCK_SIZE 4096 

using namespace libcamera;


static char TIFF_RGGB[4] = { 0, 1, 1, 2 };
static char TIFF_GRBG[4] = { 1, 0, 2, 1 };
static char TIFF_BGGR[4] = { 2, 1, 1, 0 };
static char TIFF_GBRG[4] = { 1, 2, 0, 1 };

struct BayerFormat
{
	char const *name;
	int bits;
	char const *order;
	bool packed;
	bool compressed;
};

// CinemaDNG Tags
ttag_t TIFFTAG_FRAMERATE =  0xC764;
ttag_t TIFFTAG_TIMECODE = 0xC763;
ttag_t TIFFTAG_CAMERALABEL = 0xC7A1;
ttag_t TIFFTAG_REELNAME = 0xC789;
ttag_t TIFFTAG_TSTOP = 0xC772;
char frameRateStr[] = "FrameRate";
char timeCodeStr[] = "TimeCodes";
char cameraLabelStr[] = "CameraLabel";
char reelNameStr[] = "ReelName";
char tStopStr[] = "TStop";
static const TIFFFieldInfo xtiffFieldInfo[] = {
    { TIFFTAG_FRAMERATE, 1, 1, TIFF_SRATIONAL,   FIELD_CUSTOM,
      true, false,  frameRateStr },
    { TIFFTAG_TSTOP, 1, 1, TIFF_RATIONAL,   FIELD_CUSTOM,
      true, false,  tStopStr },
    { TIFFTAG_TIMECODE, 8, 8, TIFF_BYTE,    FIELD_CUSTOM,
      true, false,  timeCodeStr },
    { TIFFTAG_CAMERALABEL, TIFF_VARIABLE, TIFF_VARIABLE, TIFF_ASCII,      FIELD_CUSTOM, 
      true, false, cameraLabelStr },
    { TIFFTAG_REELNAME, TIFF_VARIABLE, TIFF_VARIABLE, TIFF_ASCII,      FIELD_CUSTOM, 
      true, false, reelNameStr }
};


static const std::map<PixelFormat, BayerFormat> bayer_formats =
{
	{ formats::SRGGB10_CSI2P, { "RGGB-10", 10, TIFF_RGGB, true, false } },
	{ formats::SGRBG10_CSI2P, { "GRBG-10", 10, TIFF_GRBG, true, false } },
	{ formats::SBGGR10_CSI2P, { "BGGR-10", 10, TIFF_BGGR, true, false } },
	{ formats::SGBRG10_CSI2P, { "GBRG-10", 10, TIFF_GBRG, true, false } },

	{ formats::SRGGB10, { "RGGB-10", 10, TIFF_RGGB, false, false } },
	{ formats::SGRBG10, { "GRBG-10", 10, TIFF_GRBG, false, false } },
	{ formats::SBGGR10, { "BGGR-10", 10, TIFF_BGGR, false, false } },
	{ formats::SGBRG10, { "GBRG-10", 10, TIFF_GBRG, false, false } },

	{ formats::SRGGB12_CSI2P, { "RGGB-12", 12, TIFF_RGGB, true, false } },
	{ formats::SGRBG12_CSI2P, { "GRBG-12", 12, TIFF_GRBG, true, false } },
	{ formats::SBGGR12_CSI2P, { "BGGR-12", 12, TIFF_BGGR, true, false } },
	{ formats::SGBRG12_CSI2P, { "GBRG-12", 12, TIFF_GBRG, true, false } },

	{ formats::SRGGB12, { "RGGB-12", 12, TIFF_RGGB, false, false } },
	{ formats::SGRBG12, { "GRBG-12", 12, TIFF_GRBG, false, false } },
	{ formats::SBGGR12, { "BGGR-12", 12, TIFF_BGGR, false, false } },
	{ formats::SGBRG12, { "GBRG-12", 12, TIFF_GBRG, false, false } },

	{ formats::SRGGB16, { "RGGB-16", 16, TIFF_RGGB, false, false } },
	{ formats::SGRBG16, { "GRBG-16", 16, TIFF_GRBG, false, false } },
	{ formats::SBGGR16, { "BGGR-16", 16, TIFF_BGGR, false, false } },
	{ formats::SGBRG16, { "GBRG-16", 16, TIFF_GBRG, false, false } },

	{ formats::R10_CSI2P, { "BGGR-10", 10, TIFF_BGGR, true, false } },
	{ formats::R10, { "BGGR-10", 10, TIFF_BGGR, false, false } },
	// Currently not in the main libcamera branch
	//{ formats::R12_CSI2P, { "BGGR-12", 12, TIFF_BGGR, true } },
	{ formats::R12, { "BGGR-12", 12, TIFF_BGGR, false, false } },

	/* PiSP compressed formats. */
	{ formats::RGGB16_PISP_COMP1, { "RGGB-16-PISP", 16, TIFF_RGGB, false, true } },
	{ formats::GRBG16_PISP_COMP1, { "GRBG-16-PISP", 16, TIFF_GRBG, false, true } },
	{ formats::GBRG16_PISP_COMP1, { "GBRG-16-PISP", 16, TIFF_GBRG, false, true } },
	{ formats::BGGR16_PISP_COMP1, { "BGGR-16-PISP", 16, TIFF_BGGR, false, true } },
};

void pack_8bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    for (size_t i = 0; i < num_pixels; i++) {
        dst[0] = src[i];
    }
}

void pack_10bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    // 5 bytes can hold 4 10-bit pixels
    // Every iteration of the loop processes 4 pixels (40 bits)
    for (size_t i = 0; i < num_pixels; i += 4) {
        dst[0] = src[i] >> 2;                               // Highest 8 bits of pixel 1
        dst[1] = (src[i] << 6) | (src[i + 1] >> 4);         // Lowest 2 bits of pixel 1 + highest 6 bits of pixel 2
        dst[2] = (src[i + 1] << 4) | (src[i + 2] >> 6);     // Lowest 4 bits of pixel 2 + highest 4 bits of pixel 3
        dst[3] = (src[i + 2] << 2) | (src[i + 3] >> 8);     // Lowest 6 bits of pixel 3 + highest 2 bits of pixel 4
        dst[4] = src[i + 3];                                // Lowest 8 bits of pixel 4

        dst += 5; // Move to the next 5 bytes
    }
}

void pack_12bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    // 3 bytes can hold 2 12-bit pixels
    // Every iteration of the loop processes 2 pixels (24 bits)
    for (size_t i = 0; i < num_pixels; i += 2) {
        dst[0] = src[i] >> 4;                               // Highest 8 bits of pixel 1
        dst[1] = (src[i] << 4) | (src[i + 1] >> 8);         // Lowest 4 bits of pixel 1 + highest 4 bits of pixel 2
        dst[2] = src[i + 1];                                // Lowest 8 bits of pixel 2

        dst += 3; // Move to the next 3 bytes
    }
}

void pack_14bit_data(const uint16_t* src, uint8_t* dst, size_t num_pixels) {
    // Ensure that we have enough pixels (must be a multiple of 4)
    if (num_pixels % 4 != 0) return;

    // Every iteration of the loop processes 4 pixels (56 bits)
    for (size_t i = 0; i < num_pixels; i += 4) {
        dst[0] = (src[i] >> 6);                                  // Highest 8 bits of pixel 1
        dst[1] = ((src[i] & 0x3F) << 2) | (src[i + 1] >> 12);    // Lowest 6 bits of pixel 1 and highest 2 bits of pixel 2
        dst[2] = (src[i + 1] >> 4) & 0xFF;                       // Middle 8 bits of pixel 2
        dst[3] = ((src[i + 1] & 0xF) << 4) | (src[i + 2] >> 10); // Lowest 4 bits of pixel 2 and highest 4 bits of pixel 3
        dst[4] = (src[i + 2] >> 2) & 0xFF;                       // Middle 8 bits of pixel 3
        dst[5] = ((src[i + 2] & 0x3) << 6) | (src[i + 3] >> 8);  // Lowest 2 bits of pixel 3 and highest 6 bits of pixel 4
        dst[6] = src[i + 3] & 0xFF;                              // Lowest 8 bits of pixel 4

        dst += 7; // Move to the next 7 bytes
    }
}


struct Matrix
{
Matrix(float m0, float m1, float m2,
       float m3, float m4, float m5,
       float m6, float m7, float m8)
    {
        m[0] = m0, m[1] = m1, m[2] = m2;
        m[3] = m3, m[4] = m4, m[5] = m5;
        m[6] = m6, m[7] = m7, m[8] = m8;
    }
    Matrix(float diag0, float diag1, float diag2) : Matrix(diag0, 0, 0, 0, diag1, 0, 0, 0, diag2) {}
    Matrix() {}
    float m[9];
    Matrix T() const
    {
        return Matrix(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
    }
    Matrix C() const
    {
        return Matrix(m[4] * m[8] - m[5] * m[7], -(m[3] * m[8] - m[5] * m[6]), m[3] * m[7] - m[4] * m[6],
                      -(m[1] * m[8] - m[2] * m[7]), m[0] * m[8] - m[2] * m[6], -(m[0] * m[7] - m[1] * m[6]),
                      m[1] * m[5] - m[2] * m[4], -(m[0] * m[5] - m[2] * m[3]), m[0] * m[4] - m[1] * m[3]);
    }
    Matrix Adj() const { return C().T(); }
    float Det() const
    {
        return (m[0] * (m[4] * m[8] - m[5] * m[7]) -
                m[1] * (m[3] * m[8] - m[5] * m[6]) +
                m[2] * (m[3] * m[7] - m[4] * m[6]));
    }
    Matrix Inv() const { return Adj() * (1.0 / Det()); }
    Matrix operator*(Matrix const &other) const
    {
        Matrix result;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                result.m[i * 3 + j] =
                    m[i * 3] * other.m[j] + m[i * 3 + 1] * other.m[3 + j] + m[i * 3 + 2] * other.m[6 + j];
        return result;
    }
    Matrix operator*(float const &f) const
    {
        Matrix result;
        for (int i = 0; i < 9; i++)
            result.m[i] = m[i] * f;
        return result;
    }
};

#include <pthread.h>

DngEncoder::DngEncoder(RawOptions const *options)
    : Encoder(options), // Assuming you're calling the base class constructor
      encoder_initialized_(false),
      encodeCheck_(false),
      abortEncode_(false), 
      abortOutput_(false), 
      resetCount_(false), 
      index_(0), 
      frames_(0), 
      options_(options)
{
    console = spdlog::stdout_color_mt("dng_encoder");

    for (int i = 0; i < NUM_ENC_THREADS; i++){
        encode_thread_[i] = std::thread(std::bind(&DngEncoder::encodeThread, this, i));
    }
    for (int i = 0; i < NUM_DISK_THREADS; i++){
        disk_thread_[i] = std::thread(std::bind(&DngEncoder::diskThread, this, i));
    }

    console->info("DngEncoder started!");
}

DngEncoder::~DngEncoder()
{
    abortEncode_ = true;
    for (int i = 0; i < NUM_ENC_THREADS; i++){
        encode_thread_[i].join();
    }
    for (int i = 0; i < NUM_DISK_THREADS; i++){
        disk_thread_[i].join();
    }

    abortOutput_ = true;
    console->info("DngEncoder stopped!");
}

void DngEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
{
    {
        std::lock_guard<std::mutex> lock(encode_mutex_);
    }
    {
        input_done_callback_(nullptr);
        output_ready_callback_(mem, size, timestamp_us, true);
    }       
}

void DngEncoder::EncodeBuffer2(int fd, size_t size, void *mem, StreamInfo const &info, size_t losize, void *lomem, StreamInfo const &loinfo, int64_t timestamp_us, CompletedRequest::ControlList const &metadata)
{
    {
        std::lock_guard<std::mutex> lock(encode_mutex_);
        EncodeItem item = { mem, size, info, lomem, losize, loinfo, metadata, timestamp_us, index_++ };
        encode_queue_.push(item);
        encode_cond_var_.notify_all();
    }
}

void DngEncoder::setup_encoder(libcamera::StreamConfiguration const &cfg, libcamera::StreamConfiguration const &lo_cfg, CompletedRequest::ControlList const &metadata)
{
    // Check the Bayer format
    auto it = bayer_formats.find(cfg.pixelFormat);
    if (it == bayer_formats.end())
        throw std::runtime_error("unsupported Bayer format");
    BayerFormat const &bayer_format = it->second;
    console->debug("Bayer format is {}", bayer_format.name);

    dng_info.bits = bayer_format.bits;
    dng_info.bits = 12;

    // white level
    dng_info.white = (1 << dng_info.bits) - 1;

    // black_level configuartion
    dng_info.black = 4096 * (1 << dng_info.bits) / 65536.0;
    std::fill(std::begin(dng_info.black_levels), std::end(dng_info.black_levels), dng_info.black);

    auto bl = metadata.get(controls::SensorBlackLevels);
    if (bl)
    {
        // levels is in the order R, Gr, Gb, B. Re-order it for the actual bayer order.
        for (int i = 0; i < 4; i++)
        {
            int j = bayer_format.order[i];
            j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!bayer_format.order[i ^ 1]);
            dng_info.black_levels[j] = (*bl)[i] * (1 << dng_info.bits) / 65536.0;
        }
    }
    else
        console->error("WARNING: no black level found, using default");

    // AnalogBalance -- Nuetral setup
    std::fill(std::begin(dng_info.NEUTRAL), std::end(dng_info.NEUTRAL), 1);
    std::fill(std::begin(dng_info.ANALOGBALANCE), std::end(dng_info.ANALOGBALANCE), 1);


    // CCM Configuration
    Matrix WB_GAINS(1, 1, 1);
    auto cg = metadata.get(controls::ColourGains);
    if (cg)
    {
        dng_info.NEUTRAL[0] = 1.0 / (*cg)[0];
        dng_info.NEUTRAL[2] = 1.0 / (*cg)[1];
        WB_GAINS = Matrix((*cg)[0], 1, (*cg)[1]);
    }

    // Use a slightly plausible default CCM in case the metadata doesn't have one (it should!).
    Matrix CCM(1.90255, -0.77478, -0.12777,
               -0.31338, 1.88197, -0.56858,
               -0.06001, -0.61785, 1.67786);
    auto ccm = metadata.get(controls::ColourCorrectionMatrix);
    if (ccm)
    {
        CCM = Matrix((*ccm)[0], (*ccm)[1], (*ccm)[2], (*ccm)[3], (*ccm)[4], (*ccm)[5], (*ccm)[6], (*ccm)[7], (*ccm)[8]);
    }
    else
        console->error("WARNING: no CCM metadata found");

    // This maxtrix from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
    Matrix RGB2XYZ(0.4124564, 0.3575761, 0.1804375,
                   0.2126729, 0.7151522, 0.0721750,
                   0.0193339, 0.1191920, 0.9503041);
    Matrix CAM_XYZ = (RGB2XYZ * CCM * WB_GAINS).Inv();
    std::copy(std::begin(CAM_XYZ.m), std::end(CAM_XYZ.m), std::begin(dng_info.CAM_XYZ));

    // cfa
    dng_info.cfa_repeat_pattern_dim[0] = 2;
    dng_info.cfa_repeat_pattern_dim[1] = 2;
    dng_info.black_level_repeat_dim[0] = 2;
    dng_info.black_level_repeat_dim[1] = 2;
    memcpy(dng_info.bayer_order,bayer_format.order,4);

    // offsets
    // dng_info.offset_y_start = options_->rawCrop[0];
    // dng_info.offset_y_end = options_->rawCrop[1];
    // dng_info.offset_x_start = options_->rawCrop[2];
    // dng_info.offset_x_end = options_->rawCrop[3];

    dng_info.offset_y_start = 0;
    dng_info.offset_y_end = 0;
    dng_info.offset_x_start = 0;
    dng_info.offset_x_end = 0;

    // const float bppf = (dng_bits/8);
    // const uint16_t byte_offset_x = (bppf * offset_x_start) / sizeof(uint64_t); 
    // // const uint16_t read_length_x = (1.5 * (info.width - (offset_x_start+offset_x_end))) / sizeof(uint64_t);
    dng_info.t_height = (cfg.size.height - (dng_info.offset_y_start+dng_info.offset_y_end));
    dng_info.t_width = (cfg.size.width - (dng_info.offset_x_start+dng_info.offset_x_end));

    // thumbnail config
    dng_info.thumbType = options_->thumbnail;
    dng_info.thumbWidth = 32;
    dng_info.thumbHeight = 32;
    dng_info.thumbPhotometric = PHOTOMETRIC_MINISBLACK;
    dng_info.thumbBitsPerSample = 8;
    dng_info.thumbSamplesPerPixel = 1;
    unsigned int thumbnail_size = dng_info.thumbWidth * dng_info.thumbHeight;

    switch(dng_info.thumbType){
        case 1:
            dng_info.thumbWidth = lo_cfg.stride;
            dng_info.thumbHeight = lo_cfg.size.height;
            thumbnail_size = dng_info.thumbWidth * dng_info.thumbHeight;
            break;
        case 2:
            dng_info.thumbWidth = lo_cfg.size.width;
            dng_info.thumbHeight = lo_cfg.size.height;
            dng_info.thumbSamplesPerPixel = 3;
            dng_info.thumbPhotometric = PHOTOMETRIC_RGB;
            thumbnail_size = lo_cfg.stride*3*lo_cfg.size.height;
            break;
    }

    // buffer_size calculation
    const unsigned int dng_wrapper_size = 20000; // ~20kb, much smaller in practice.  
    const unsigned int frame_size = (cfg.size.width * cfg.size.height * dng_info.bits) / 8;
    // const unsigned int frame_size = (cfg.stride * cfg.size.height);
    const unsigned long bytes = frame_size + dng_wrapper_size + thumbnail_size;
    dng_info.buffer_size = ((bytes + ONE_MB - 1) / ONE_MB) * ONE_MB;

    // compression and other
    dng_info.compression = COMPRESSION_NONE;

    // extra metadata
    dng_info.make = "Raspberry Pi";
    dng_info.model = options_->model;
    dng_info.serial = getHwId();
    dng_info.software = "Libcamera;cinepi-raw";
    dng_info.ucm = options_->ucm.value_or("CinePI");

    // adjust disk_buffer
    const double MAX_RAM_FRACTION = 2.0 / 3.0;
    std::ifstream meminfo("/proc/meminfo");
    std::string content((std::istreambuf_iterator<char>(meminfo)), std::istreambuf_iterator<char>());

    std::regex memAvailRegex(R"(MemAvailable:\s+(\d+)\s+kB)");
    std::smatch match;

    size_t totalRam = 0;
    if (std::regex_search(content, match, memAvailRegex)) {
        totalRam = std::stoull(match[1]) * 1024;  // Convert kilobytes to bytes
    }
    max_buffer_frames = (MAX_RAM_FRACTION * totalRam) / dng_info.buffer_size;

    console->debug("Max Frames in Buffer: {}", max_buffer_frames);

    encoder_initialized_ = true;
    console->info("DngEncoder is setup!");
}

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

typedef struct {
    unsigned char* buffer;  // In-memory buffer
    toff_t offset;          // Current offset
    toff_t usedSize;        // Track the maximum offset written to
    toff_t totalSize;       // Total size of the buffer
} MemoryBuffer;


tsize_t myTIFFReadProc(thandle_t fd, tdata_t buf, tsize_t size) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    if (memBuf->offset + size > memBuf->usedSize) {
        size = memBuf->usedSize - memBuf->offset;
    }
    memcpy(buf, memBuf->buffer + memBuf->offset, size);
    memBuf->offset += size;
    return size;
}

tsize_t myTIFFWriteProc(thandle_t fd, tdata_t buf, tsize_t size) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    if (memBuf->offset + size > memBuf->totalSize) {
        // If writing the data would overflow the buffer, adjust the size.
        size = memBuf->totalSize - memBuf->offset;
    }
    memcpy(memBuf->buffer + memBuf->offset, buf, size);
    memBuf->offset += size;
    if (memBuf->offset > memBuf->usedSize) {
        memBuf->usedSize = memBuf->offset;
    }
    return size;
}

toff_t myTIFFSeekProc(thandle_t fd, toff_t off, int whence) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    switch (whence) {
        case SEEK_SET:
            memBuf->offset = off;
            break;
        case SEEK_CUR:
            memBuf->offset += off;
            break;
        case SEEK_END:
            memBuf->offset = memBuf->usedSize + off;
            break;
    }
    return memBuf->offset;
}

toff_t myTIFFSizeProc(thandle_t fd) {
    MemoryBuffer* memBuf = (MemoryBuffer*) fd;
    return memBuf->usedSize;
}

int myTIFFCloseProc(thandle_t fd) {
    // Nothing special to do in the memory buffer context.
    // Memory will be freed after writing to disk, outside of the libTIFF context.
    return 0;
}


size_t DngEncoder::dng_save(int thread_num, uint8_t const *mem_tiff, uint8_t const *mem, StreamInfo const &info, uint8_t const *lomem, StreamInfo const &loinfo, size_t losize,
              ControlList const &metadata, uint64_t fn)
{
    const DngInfo& constDngInfo = dng_info;

    // get raw unique value as sensor timestamp
    std::array<uint8_t, 8> rawUniq = {};
    if (auto rU = metadata.get(libcamera::controls::SensorTimestamp)) {
        std::copy_n(reinterpret_cast<uint8_t*>(&*rU), rawUniq.size(), rawUniq.begin());
    }

    // get shutter speed
    auto exp = metadata.get(controls::ExposureTime);
    float exp_time = 10000;
    if (exp)
        exp_time = *exp;
    else
        console->error("WARNING: default to exposure time of {}us", exp_time);
    exp_time /= 1e6;

    // get iso
    auto ag = metadata.get(controls::AnalogueGain);
    uint16_t iso = 100;
    if (ag)
        iso = *ag * 100.0;
    else
        console->error("WARNING: default to ISO value of {}", iso);

    // begin writing TIFF/DNG
    TIFF *tif = nullptr;
    MemoryBuffer memBuf;
    memBuf.buffer = (unsigned char*)mem_tiff;
    if (!memBuf.buffer) {
        console->error("Failed to allocate memory\n");
        exit(1);
    }
    memBuf.offset = 0;
    memBuf.usedSize = 0;
    memBuf.totalSize = constDngInfo.buffer_size;

    console->trace("thrd: {} Writing DNG {}", thread_num, fn);
    try
    {
        toff_t offset_subifd = 0, offset_exififd = 0;

        tif = TIFFClientOpen("memory", "w", (thandle_t)&memBuf,
                                   myTIFFReadProc, myTIFFWriteProc,
                                   myTIFFSeekProc, myTIFFCloseProc,
                                   myTIFFSizeProc, NULL, NULL);

        if (!tif)
            throw std::runtime_error("could not open file " + fn);
        
        console->trace("thrd: {} Writing DNG thumbnail {}", thread_num, fn);
        TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 1);
        TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, constDngInfo.thumbWidth);
        TIFFSetField(tif, TIFFTAG_IMAGELENGTH, constDngInfo.thumbHeight);
        TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, constDngInfo.thumbBitsPerSample);
        TIFFSetField(tif, TIFFTAG_COMPRESSION, constDngInfo.compression);
        TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, constDngInfo.thumbPhotometric );
        TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, constDngInfo.thumbSamplesPerPixel);
        TIFFSetField(tif, TIFFTAG_MAKE, constDngInfo.make.c_str());
        TIFFSetField(tif, TIFFTAG_MODEL, constDngInfo.model.c_str());
        TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, constDngInfo.ucm.c_str());
        TIFFSetField(tif, TIFFTAG_CAMERASERIALNUMBER, constDngInfo.serial.c_str());
        TIFFSetField(tif, TIFFTAG_RAWDATAUNIQUEID, rawUniq.data());
        TIFFSetField(tif, TIFFTAG_SOFTWARE, constDngInfo.software.c_str());
        TIFFSetField(tif, TIFFTAG_DNGVERSION, "\001\004\000\000");
        TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, "\001\001\000\000");
        TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
        TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
        TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, constDngInfo.CAM_XYZ);
        TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, constDngInfo.NEUTRAL);
        TIFFSetField(tif, TIFFTAG_CALIBRATIONILLUMINANT1, 21);
        TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
        TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);

        switch (constDngInfo.thumbType) {
            case 0: {
                // a small 32x32 black thumbnail as a placeholder.
                size_t tSize = constDngInfo.thumbWidth * constDngInfo.thumbHeight;
                std::vector<uint8_t> thumb(tSize, 0);
                TIFFWriteRawStrip(tif, 0, thumb.data(), tSize);
                break;
            }
            case 1: {
                // a luma only thumbnail from YUV data.
                uint8_t* Y_data = const_cast<uint8_t*>(lomem);
                TIFFWriteRawStrip(tif, 0, Y_data, constDngInfo.thumbWidth * constDngInfo.thumbHeight);
                break;
            }
            case 2: {
                // a small RGB thumbnail. (
                size_t rowSize = loinfo.stride*3;
                size_t thumbSize = rowSize*loinfo.height;
                std::vector<uint8_t> thumb(thumbSize);
                uint8_t *read = thumb.data();
                if(nv21_to_rgb(read, lomem, loinfo.stride, loinfo.height) != 1){
                    throw std::runtime_error("error converting yuv2rgb image data");
                }
                for(unsigned int y = 0; y < loinfo.height; y++){
                    if (TIFFWriteScanline(tif, (read + y*rowSize), y, 0) != 1)
                        throw std::runtime_error("error writing DNG image data");   
                }
                break;
            }
        }

        TIFFCheckpointDirectory(tif);
        TIFFWriteDirectory(tif);

        console->trace("thrd: {} Writing DNG thumbnail end {}", thread_num, fn);
    
        // The main image (actually tends to show up as "sub-image 1").
        TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
        TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, info.stride/2);
        TIFFSetField(tif, TIFFTAG_IMAGELENGTH, constDngInfo.t_height);
        TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, constDngInfo.bits);
        TIFFSetField(tif, TIFFTAG_COMPRESSION, constDngInfo.compression);
        TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_CFA);
        TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
        TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
        TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, constDngInfo.cfa_repeat_pattern_dim);
#if TIFFLIB_VERSION >= 20201219 // version 4.2.0 or later
        TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, constDngInfo.bayer_order);
#else
        TIFFSetField(tif, TIFFTAG_CFAPATTERN, constDngInfo.bayer_order);
#endif
        TIFFSetField(tif, TIFFTAG_WHITELEVEL, 1, &constDngInfo.white);
        TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, &constDngInfo.black_level_repeat_dim);
        TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 4, &constDngInfo.black_levels);
        
        TIFFSetField(tif, TIFFTAG_ANALOGBALANCE, 3, constDngInfo.ANALOGBALANCE);
        TIFFSetField(tif, TIFFTAG_BASELINEEXPOSURE, 1.0);
        TIFFSetField(tif, TIFFTAG_BASELINENOISE, 1.0);
        TIFFSetField(tif, TIFFTAG_BASELINESHARPNESS, 1.0);
        TIFFSetField(tif, TIFFTAG_BAYERGREENSPLIT, 0);
        TIFFSetField(tif, TIFFTAG_LINEARRESPONSELIMIT, 1.0);
        
        time_t t;
        time(&t);
        struct tm *time_info = localtime(&t);
        TIFFMergeFieldInfo(tif, xtiffFieldInfo, 5);
        const double frameRate = (double)*options_->framerate;
        TIFFSetField(tif, TIFFTAG_FRAMERATE, &frameRate);

        const uint8_t frames = static_cast<uint8_t>(fn % static_cast<int>(frameRate));
        const uint8_t seconds = static_cast<uint8_t>(time_info->tm_sec);
        const uint8_t minutes = static_cast<uint8_t>(time_info->tm_min);
        const uint8_t hours = static_cast<uint8_t>(time_info->tm_hour);

        const char timecode[] = { 
            static_cast<char>((frames / 10) << 4 | (frames % 10)),     // Tens of Frames and Units of Frames
            static_cast<char>((seconds / 10) << 4 | (seconds % 10)),   // Tens of Seconds and Units of Seconds
            static_cast<char>((minutes / 10) << 4 | (minutes % 10)),   // Tens of Minutes and Units of Minutes
            static_cast<char>((hours / 10) << 4 | (hours % 10)),       // Tens of Hours and Units of Hours
            0,  // BG 2
            0,  // BG 4
            0,  // BG 6
            0   // BG 8
        };
        TIFFSetField(tif, TIFFTAG_TIMECODE, &timecode);

        // capture the originationTimeCode for the first frame, used by sound module for sync. 
        if(fn == 0){
            originationTimeCode = {hours, minutes, seconds, frames, timecode[3], timecode[2], timecode[1], timecode[0]};
            originationDate = {(uint16_t)(time_info->tm_year + 1900),  (uint16_t)(time_info->tm_mon + 1), (uint16_t)time_info->tm_mday};
        }
        
        TIFFSetField(tif, TIFFTAG_CAMERALABEL, "Camera A");
        TIFFSetField(tif, TIFFTAG_REELNAME, "A0000_001234");
        const double tstop = 2.0;
        TIFFSetField(tif, TIFFTAG_TSTOP, &tstop);

        console->trace("thrd: {} Writing DNG main image {}", thread_num, fn);

        auto main_image_start = std::chrono::high_resolution_clock::now();

        const unsigned int num_pixels = constDngInfo.t_height * (info.stride/2);
        const size_t data_size = (num_pixels * constDngInfo.bits ) / 8;
        std::vector<uint8_t> packed_data(data_size);

        auto packData = [&](int bits) -> uint8_t* {
            switch(bits) {
                case 8:  pack_8bit_data((uint16_t*)mem, (uint8_t*)packed_data.data(), num_pixels); return (uint8_t*)packed_data.data();
                case 10: pack_10bit_data((uint16_t*)mem, (uint8_t*)packed_data.data(), num_pixels); return (uint8_t*)packed_data.data();
                case 12: pack_12bit_data((uint16_t*)mem, (uint8_t*)packed_data.data(), num_pixels); return (uint8_t*)packed_data.data();
                case 14: pack_14bit_data((uint16_t*)mem, (uint8_t*)packed_data.data(), num_pixels); return (uint8_t*)packed_data.data();
                case 16:
                default: return (uint8_t*)mem;
            }
        };

        int size = (constDngInfo.bits == 8 || constDngInfo.bits == 16) ? info.stride * info.height : data_size;
        TIFFWriteRawStrip(tif, 0, packData(constDngInfo.bits), size);

        auto main_image_end = std::chrono::high_resolution_clock::now();
        auto main_image_duration = std::chrono::duration_cast<std::chrono::milliseconds>(main_image_end - main_image_start).count();

        console->debug("main image written to dng: {}ms", main_image_duration);

        TIFFCheckpointDirectory(tif);
        offset_subifd = TIFFCurrentDirOffset(tif);
        console->trace("thrd: {} Writing DNG main dict {}", thread_num, fn);
        TIFFWriteDirectory(tif);

        console->trace("thrd: {} Writing DNG EXIF {}", thread_num, fn);
        
        TIFFCreateEXIFDirectory(tif);

        console->trace("thrd: {} Writing DNG TIFFCreateEXIFDirectory {}", thread_num, fn);
        char time_str[32];
        strftime(time_str, 32, "%Y:%m:%d %H:%M:%S", time_info);
        TIFFSetField(tif, EXIFTAG_DATETIMEORIGINAL, time_str);

        TIFFSetField(tif, EXIFTAG_ISOSPEEDRATINGS, 1, &iso);
        TIFFSetField(tif, EXIFTAG_EXPOSURETIME, exp_time);

        TIFFCheckpointDirectory(tif);
        offset_exififd = TIFFCurrentDirOffset(tif);
        TIFFWriteDirectory(tif);
        TIFFSetDirectory(tif, 0);
        TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);

        TIFFWriteDirectory(tif);
        TIFFUnlinkDirectory(tif, 2);
        TIFFClose(tif);

        console->trace("thrd: {} Writing DNG TIFFClose {}", thread_num, fn);
        return memBuf.usedSize;
    }
    catch (std::exception const &e)
    {
        if (tif){
            TIFFClose(tif);
        }
        if (memBuf.buffer) {
            free(memBuf.buffer);
        }
        throw;
    }
}


void DngEncoder::encodeThread(int num)
{
    std::chrono::duration<double> encode_time(0);
    EncodeItem encode_item;

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(encode_mutex_);
            while (true)
            {   
                if (!encode_queue_.empty())
                {
                    encode_item = encode_queue_.front();
                    encode_queue_.pop();
                    break;
                }
                else{
                    encode_cond_var_.wait_for(lock, 500us);
                }
            }
        }

        frames_ = {encode_item.index};
        console->trace("Thread[{}] encode frame: {}", num, encode_item.index);

        {   
            auto start_time = std::chrono::high_resolution_clock::now();
            
            uint8_t *mem_tiff;
            if (posix_memalign((void **)&mem_tiff, BLOCK_SIZE, dng_info.buffer_size) != 0) {
                perror("Error allocating aligned memory");
                return;
            }
            size_t tiff_size = dng_save(num,(const uint8_t*) mem_tiff,(const uint8_t*)encode_item.mem, encode_item.info, (const uint8_t*)encode_item.lomem, encode_item.loinfo, encode_item.losize, encode_item.met, encode_item.index);
            DiskItem item = { mem_tiff, tiff_size, encode_item.info, encode_item.met, encode_item.timestamp_us, encode_item.index};

            std::lock_guard<std::mutex> lock(disk_mutex_);
            disk_buffer_.push(std::move(item));
            int amount = disk_buffer_.size();
            disk_cond_var_.notify_all();
            auto end_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            console->info("Thread[{}] {} Time taken for the encode: {} milliseconds, disk buffer count:{} Size:{}", num, encode_item.index, duration, amount, tiff_size);
        }

        {
            input_done_callback_(nullptr);
            output_ready_callback_(encode_item.mem, encode_item.size, encode_item.timestamp_us, true);
        }       

    }
}


//Flushing data to disk
void DngEncoder::diskThread(int num)
{
    DiskItem disk_item;

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(disk_mutex_);
            while (true)
            {
                if (!disk_buffer_.empty())
                {
                    disk_item = disk_buffer_.front();
                    disk_buffer_.pop();
                    break;
                }
                else{
                    disk_cond_var_.wait_for(lock, 1ms);
                }
            }
        }

        std::ostringstream oss;
        oss << options_->mediaDest << '/' 
            << options_->folder << '/' 
            << options_->folder << '_'
            << std::setw(9) << std::setfill('0') << disk_item.index 
            << ".dng";

        std::string filename = oss.str();
    
        console->trace("Thread[{}]  Save frame to disk: {}", num,  disk_item.index);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        // Now save the memory buffer to disk
        int fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_DIRECT, 0644);
        if (fd != -1) {

            // Provide sequential access hint
            posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
            posix_fadvise(fd, 0, 0, POSIX_FADV_NOREUSE);
            if(write(fd, disk_item.mem_tiff, dng_info.buffer_size) != dng_info.buffer_size) {
                perror("Error writing to file");
            }
            ftruncate(fd, disk_item.size);
            close(fd);

        } else {
            fprintf(stderr, "Failed to open file for writing\n");
        }
        // Clean up
        free(disk_item.mem_tiff);
        auto end_time = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        console->info("Thread[{}] {} Time taken for the disk io: {} milliseconds", num, disk_item.index, duration);
        
    }
}
