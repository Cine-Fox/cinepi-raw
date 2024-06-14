#include "cinepi_controller.hpp"

using namespace std;
using namespace std::chrono;

#define CP_DEF_WIDTH 1920
#define CP_DEF_HEIGHT 1080
#define CP_DEF_FRAMERATE 30
#define CP_DEF_ISO 400
#define CP_DEF_SHUTTER 50
#define CP_DEF_AWB 1
#define CP_DEF_COMPRESS 0
#define CP_DEF_THUMBNAIL 1
#define CP_DEF_THUMBNAIL_SIZE 3

void CinePIController::sync(){
    // getAllKeysAndValuesFromRedis();

    auto pipe = redis_->pipeline();
    auto pipe_replies = pipe.get(CONTROL_KEY_WIDTH)
                            .get(CONTROL_KEY_HEIGHT)
                            .get(CONTROL_KEY_FRAMERATE)
                            .get(CONTROL_KEY_ISO)
                            .get(CONTROL_KEY_SHUTTER_SPEED)
                            .get(CONTROL_KEY_WB)
                            .get(CONTROL_KEY_COLORGAINS)
                            .get(CONTROL_KEY_COMPRESSION)
                            .get(CONTROL_KEY_THUMBNAIL)
                            .get(CONTROL_KEY_THUMBNAIL_SIZE)
                            .get("log_level")
                            .get("ucm")
                            .get("mic_gain")
                            .exec();

    auto width = pipe_replies.get<OptionalString>(0);
    if(width){
        width_ = stoi(*width);
    }else{
        width_ = CP_DEF_WIDTH;
        redis_->set(CONTROL_KEY_WIDTH, to_string(width_));
    }

    console->critical(1);
        
    auto height = pipe_replies.get<OptionalString>(1);
    if(height){
        height_ = stoi(*height);
    }else{
        height_ = CP_DEF_HEIGHT;
        redis_->set(CONTROL_KEY_HEIGHT, to_string(height_)); 
    }

    console->critical(2);

    auto framerate = pipe_replies.get<OptionalString>(2);
    if(framerate){
        framerate_ = stoi(*framerate);
    }else{
        framerate_ = CP_DEF_FRAMERATE;
        redis_->set(CONTROL_KEY_FRAMERATE, to_string(framerate_)); 
    }

    console->critical(3);

    auto iso = pipe_replies.get<OptionalString>(3);
    if(iso){
        iso_ = stoi(*iso)/100;
    }else{
        iso_ = CP_DEF_ISO;
        redis_->set(CONTROL_KEY_ISO, to_string(iso_)); 
    }

    console->critical(4);

    auto shutter_speed = pipe_replies.get<OptionalString>(4);
    if(shutter_speed){
        shutter_speed_ = stoi(*shutter_speed);
    }else{
        shutter_speed_ = CP_DEF_SHUTTER;
        redis_->set(CONTROL_KEY_SHUTTER_SPEED, to_string(shutter_speed_)); 
    }

    console->critical(5);

    auto awb = pipe_replies.get<OptionalString>(5);
    if(awb){
        awb_ = stoi(*awb);
    }else{
        awb_ = CP_DEF_AWB;
        redis_->set(CONTROL_KEY_WB, to_string(awb_)); 
    }

    console->critical(6);
    
    auto compress = pipe_replies.get<OptionalString>(7);
    if(compress){
        compression_ = stoi(*compress);
    }else{
        compression_ = CP_DEF_COMPRESS;
        redis_->set(CONTROL_KEY_COMPRESSION, to_string(compression_));
    }

    console->critical(7);

    char *ptr = strtok(&(*pipe_replies.get<OptionalString>(6))[0], ",");
    uint8_t i = 0;
    while(ptr != NULL){
        cg_rb_[i] = (float)stof(ptr);
        i++;
        ptr = strtok(NULL, ",");  
    }

    console->critical(8);

    auto thumbnail = pipe_replies.get<OptionalString>(8);
    if(thumbnail){
        thumbnail_ = stoi(*thumbnail);
    }else{
        thumbnail_ = CP_DEF_THUMBNAIL;
        redis_->set(CONTROL_KEY_THUMBNAIL, to_string(thumbnail_));
    }

    console->critical(9);

    auto thumbnail_size = pipe_replies.get<OptionalString>(9);
    if(thumbnail_size){
        thumbnail_size_ = stoi(*thumbnail_size);
    }else{
        thumbnail_size_ = CP_DEF_THUMBNAIL_SIZE;
        redis_->set(CONTROL_KEY_THUMBNAIL, to_string(thumbnail_size_));
    }

    console->critical(10);

    auto log_level = pipe_replies.get<OptionalString>(10);
    if(log_level){
        spdlog::set_level(spdlog::level::from_str(*log_level));
    }

    console->critical(11);

    auto ucm = pipe_replies.get<OptionalString>(11);
    if(ucm){
        options_->ucm = *ucm;
    }

    console->critical(12);

    auto mic_gain = pipe_replies.get<OptionalString>(12);
    if(mic_gain){
        options_->mic_gain = stoi(*mic_gain);
        system(("amixer -c 1 sset 'Mic' " + *mic_gain + " > /dev/null 2>&1").c_str());
    }

    console->critical(13);

    // std::unordered_map<std::string, std::string> m;
    // redis_->hgetall("rawCrop", std::inserter(m, m.begin()));

    // options_->rawCrop[0] = std::stoi(m["offset_y_start"]);
    // options_->rawCrop[1] = std::stoi(m["offset_y_end"]);
    // options_->rawCrop[2] = std::stoi(m["offset_x_start"]);
    // options_->rawCrop[3] = std::stoi(m["offset_x_end"]);

    console->critical(14);

    libcamera::ControlList cl;
    cl.set(libcamera::controls::rpi::StatsOutputEnable, true);
    app_->SetControls(cl);

    options_->thumbnail = thumbnail_;
    options_->thumbnailSize = thumbnail_size_;
    
    options_->compression = compression_;
    // options_->width = width_;
    // options_->height = height_;
    options_->framerate = framerate_;
    options_->gain = iso_;

    options_->awbEn = awb_;
    if(awb_)
        options_->awb_index = 5; // daylight
    else{
        options_->awb_gain_r = cg_rb_[0];
        options_->awb_gain_b = cg_rb_[1];
    }
    
    options_->denoise = "off";
    // options_->lores_width = options_->width >> 3;
    // options_->lores_height = options_->height >> 3;
    options_->mode_string = "0:0:0:0";
}

void CinePIController::process(CompletedRequestPtr &completed_request){
    CinePIFrameInfo info(completed_request->metadata);

    Json::Value data;
    Json::Value histo;
    data["framerate"] = completed_request->framerate;
    data["colorTemp"] = info.colorTemp;
    data["focus"] = info.focus;
    data["frameCount"] = app_->GetEncoder()->getFrameCount();
    data["bufferSize"] = app_->GetEncoder()->bufferSize();
    redis_->publish(CHANNEL_STATS, data.toStyledString());
    
}

void CinePIController::mainThread(){

    console->info("CinePIController Started!");
    auto sub = redis_->subscriber();

    using MessageHandler = std::function<void(const std::optional<std::string>&)>;

    std::unordered_map<std::string, MessageHandler> handlers = {
        { "lv_zoom", [this](const std::optional<std::string>& r) {
            if(r) {
                double zoomFactor = stod(*r);
                // Retrieve the maximum sensor area
                libcamera::Rectangle sensor_area = app_->GetCameras()[0]->controls().at(&controls::ScalerCrop).max().get<libcamera::Rectangle>();

                // Calculate the dimensions of the zoomed-in area based on the zoom factor
                int w = static_cast<int>(sensor_area.width / zoomFactor);
                int h = static_cast<int>(sensor_area.height / zoomFactor);

                // Calculate the top-left corner of the new crop area to keep it centered
                int x = (sensor_area.width - w) / 2;
                int y = (sensor_area.height - h) / 2;

                // Define the crop rectangle
                libcamera::Rectangle crop(x, y, w, h);

                // Translate the crop rectangle by the sensor area's top-left point to align with the global coordinate system, if necessary
                // This step might be redundant if the sensor_area's top-left is already considered (0,0) in your coordinate system.
                // crop.translateBy(sensor_area.topLeft());

                // Log and apply the crop
                LOG(2, "Using crop " << crop.toString());
                libcamera::ControlList cl;
                cl.set(controls::ScalerCrop, crop);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_RAW_CROP, [this](const std::optional<std::string>& r) {
            std::unordered_map<std::string, std::string> m;
            redis_->hgetall("rawCrop", std::inserter(m, m.begin()));

            options_->rawCrop[0] = std::stoi(m["offset_y_start"]);
            options_->rawCrop[1] = std::stoi(m["offset_y_end"]);
            options_->rawCrop[2] = std::stoi(m["offset_x_start"]);
            options_->rawCrop[3] = std::stoi(m["offset_x_end"]);
        }},
        { CONTROL_KEY_RECORD, [this](const std::optional<std::string>& r) {
            if(r) {
                // int rec = stoi(*r);
                // switch(rec){
                //     case 1:
                //         trigger_ = rec;
                //         is_recording_ = (bool)rec;
                //     case 0:
                //         trigger_ = 0;
                //         is_recording_ = (bool)rec;
                //     case -1:
                //         trigger_ = rec;
                //         is_recording_ = false;
                // };
                trigger_ = !is_recording_ ? 1 : -1;
                is_recording_ = (bool)stoi(*r);
            }
        }},
        { CONTROL_KEY_ISO, [this](const std::optional<std::string>& r) {
            if(r) {
                iso_ = (unsigned int)(stoi(*r)/100.0);
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AnalogueGain, iso_);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_WB, [this](const std::optional<std::string>& r) {
            if(r) {
                awb_ = (unsigned int)(stoi(*r));
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AwbEnable, awb_);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_COLORGAINS, [this](const std::optional<std::string>& r) {
            if(r) {
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AwbEnable, false);
                app_->SetControls(cl);
                std::string cg_rb_s = *r;
                char *ptr = strtok(&cg_rb_s[0], ",");
                uint8_t i = 0;
                while(ptr != NULL){
                    cg_rb_[i] = (float)stof(ptr);
                    i++;
                    ptr = strtok(NULL, ",");
                }
                cl.set(libcamera::controls::ColourGains, libcamera::Span<const float, 2>({ cg_rb_[0], cg_rb_[1] }));
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_SHUTTER_ANGLE, [this](const std::optional<std::string>& r) {
            if(r) {
                shutter_angle_ = stof(*r);
                shutter_speed_ = 1.0 / ((framerate_ * 360.0) / shutter_angle_);
                uint64_t shutterTime = shutter_speed_ * 1e+6;
                libcamera::ControlList cl;
                cl.set(libcamera::controls::ExposureTime, shutterTime);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_SHUTTER_SPEED, [this](const std::optional<std::string>& r) {
            if(r) {
                shutter_speed_ = 1.0 / stof(*r);
                uint64_t shutterTime = shutter_speed_ * 1e+6;
                libcamera::ControlList cl;
                cl.set(libcamera::controls::ExposureTime, shutterTime);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_WIDTH, [this](const std::optional<std::string>& r) {
            if(r) {
                width_ = (uint16_t)(stoi(*r));
                options_->width = width_;
            }
        }},
        { CONTROL_KEY_HEIGHT, [this](const std::optional<std::string>& r) {
            if(r) {
                height_ = (uint16_t)(stoi(*r));
                options_->height = height_;
            }
        }},
        { CONTROL_KEY_COMPRESSION, [this](const std::optional<std::string>& r) {
            if(r) {
                compression_ = stoi(*r);
                options_->compression = compression_;
                cameraInit_ = true;
            }
        }},
        { CONTROL_KEY_FRAMERATE, [this](const std::optional<std::string>& r) {
            if(r) {
                framerate_ = stof(*r);
                options_->framerate = framerate_;

                long int durationValues[2] = { static_cast<long int>(1000000.0 / framerate_),
                                            static_cast<long int>(1000000.0 / framerate_) };

                libcamera::Span<const long int, 2> durationRange(durationValues, 2);
                libcamera::ControlList cl;
                cl.set(libcamera::controls::FrameDurationLimits, durationRange);
                app_->SetControls(cl);
            }
        }},
        { CONTROL_KEY_CAMERAINIT, [this](const std::optional<std::string>& r) {
            cameraInit_ = true;
        }},
        { CONTROL_KEY_THUMBNAIL, [this](const std::optional<std::string>& r) {
            if(r) {
                options_->thumbnail = stoi(*r);
            }
        }},
        { CONTROL_KEY_THUMBNAIL_SIZE, [this](const std::optional<std::string>& r) {
            if(r) {
                options_->thumbnailSize = stoi(*r);
                cameraInit_ = true;
            }
        }},
        { "log_level", [this](const std::optional<std::string>& r) {
            if(r) {
                spdlog::set_level(spdlog::level::from_str(*r));
            }
        }},
        { "mic_gain", [this](const std::optional<std::string>& r) {
            if(r) {
                options_->mic_gain = stoi(*r);
                system(("amixer -c 1 sset 'Mic' " + *r + " > /dev/null 2>&1").c_str());
            }
        }}
    };


    sub.on_message([this, &handlers](std::string channel, std::string msg) {
        console->trace("{} from: {}", msg, channel);

        auto r = redis_->get(msg);

        auto it = handlers.find(msg);
        if (it != handlers.end()) {
            it->second(r);
        }
        
        redis_->bgsave();
    });

    sub.subscribe(CHANNEL_CONTROLS);

    while (true) {
        try {
            if(abortThread_){
                return;
            }
            sub.consume();
        } catch (const Error &err) {
            // Handle exceptions.
        }
    }
}
