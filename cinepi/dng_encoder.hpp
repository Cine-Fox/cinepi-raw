#pragma once
#ifndef DNG_ENCODER_H
#define DNG_ENCODER_H

#include <condition_variable>
#include <mutex>
#include <queue>
#include <deque>
#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "encoder/encoder.hpp"
#include "raw_options.hpp"
#include "cinepi_frameinfo.hpp"

class DngEncoder : public Encoder
{
public:
	DngEncoder(RawOptions const *options);
	~DngEncoder();
	// Encode the given buffer.
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us) override;
	void EncodeBuffer2(int fd, size_t size, void *mem, StreamInfo const &info, size_t losize, void *lomem, StreamInfo const &loinfo, int64_t timestamp_us, CompletedRequest::ControlList const &metadata);
	void resetFrameCount(){
		timestamps.clear();
		originationTimeCode.fill(0);
		originationDate.fill(0);
		index_ = 0;
	}
	int bufferSize(){
		return disk_buffer_.size();
	}
	uint64_t getFrameCount(){
		return frames_;
	}

	void log_ts(int64_t ts){
		timestamps.push_back(ts);
	}

	void setup_encoder(libcamera::StreamConfiguration const &cfg, libcamera::StreamConfiguration const &lo_cfg, CompletedRequest::ControlList const &metadata);
	bool initialized(){
		return encoder_initialized_;
	}
	void reset_encoder(){
		encoder_initialized_ = false;
	}
	bool buffer_full(){
		return (disk_buffer_.size() > max_buffer_frames);
	}

	std::vector<int64_t> timestamps;
	std::array<uint8_t, 8> originationTimeCode;
	std::array<uint16_t, 3> originationDate;

private:
	std::shared_ptr<spdlog::logger> console;

	static const int NUM_ENC_THREADS = 2;
	static const int NUM_DISK_THREADS = 8;

	void encodeThread(int num);
	void diskThread(int num);

	void outputThread();

	bool encoder_initialized_;
	struct DngInfo
	{
		uint8_t bits;

		uint32_t white;
		float black;
		float black_levels[4];

		float NEUTRAL[3];
		float ANALOGBALANCE[3];

		short cfa_repeat_pattern_dim[2];
		uint16_t black_level_repeat_dim[2];
		char bayer_order[4];

		float CAM_XYZ[9];

		uint16_t offset_y_start;
		uint16_t offset_y_end;
		uint16_t offset_x_start;
		uint16_t offset_x_end;
		float bppf;
		uint16_t byte_offset_x;

		uint16_t t_height;
		uint16_t t_width;

		unsigned int compression;

		size_t buffer_size;

		uint8_t thumbType;
		uint16_t thumbWidth;
		uint16_t thumbHeight;
		uint16_t thumbPhotometric;
		uint16_t thumbBitsPerSample;
		uint16_t thumbSamplesPerPixel;

		std::string make;
		std::string model;
		std::string serial;
		std::string ucm;
		std::string software;
	};
	DngInfo dng_info;
	unsigned int max_buffer_frames;

	bool encodeCheck_;
	bool abortEncode_;
	bool abortOutput_;
	bool resetCount_;
	uint64_t index_;
	uint64_t frames_;

    RawOptions const *options_;

	size_t dng_save(int thread_num, uint8_t const *mem_tiff, uint8_t const *mem, StreamInfo const &info, uint8_t const *lomem, StreamInfo const &loinfo, size_t losize,
			libcamera::ControlList const &metadata, uint64_t fn);

	struct EncodeItem
	{

		void *mem;
        size_t size;
		StreamInfo info;
		void *lomem;
		size_t losize;
		StreamInfo loinfo;
		CompletedRequest::ControlList met;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<EncodeItem> encode_queue_;
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];

	struct DiskItem
	{
		void *mem_tiff;
        size_t size;
		StreamInfo info;
		CompletedRequest::ControlList met;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<DiskItem> disk_buffer_;
	std::mutex disk_mutex_;
	std::condition_variable disk_cond_var_;
	std::thread disk_thread_[NUM_DISK_THREADS];
};

#endif
