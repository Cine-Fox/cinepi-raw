#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdint.h>

#include "dng_encoder.hpp"
#include "preview/preview.hpp"
#include "core/logging.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

#include <mutex>
#include <queue>
#include <thread>

#include "utils.hpp"

#include "cinepi_recorder.hpp"
#include "cinepi_state.hpp"
#include "raw_options.hpp"
#include <sw/redis++/redis++.h>

#define CHANNEL_CONTROLS "cp_controls"
#define CHANNEL_STATS "cp_stats"

using namespace sw::redis;

class CinePIController : public CinePIState
{
    public:
        CinePIController(CinePIRecorder *app) : CinePIState(), app_(app), options_(app->GetOptions()), abortThread_(false) {};
        ~CinePIController() {
            abortThread_ = true;
            main_thread_.join();
        };

        void start(){
            redis_ = new Redis(options_->redis);
            std::cout << redis_->ping() << std::endl;
            // sync();
            main_thread_ = std::thread(std::bind(&CinePIController::mainThread, this));
            pub_thread_ = std::thread(std::bind(&CinePIController::pubThread, this));
        }

        void sync();

        void process();

    protected:

    private:
        void mainThread();
        void pubThread();

        CinePIRecorder *app_;
        RawOptions *options_;
        Redis *redis_;

        bool abortThread_;
        std::thread main_thread_;
        std::thread pub_thread_;
};