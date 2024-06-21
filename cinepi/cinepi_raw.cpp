/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Csaba Nagy.
 *
 * cinepi_raw.cpp - cinepi raw dng recording app.
 */

#include <chrono>
#include "cinepi_sound.hpp"
#include "cinepi_controller.hpp"

#include "dng_encoder.hpp"
#include "output/output.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

using namespace std::placeholders;

// The main even loop for the application.
static void event_loop(CinePIRecorder &app, CinePIController &controller, CinePISound &sound)
{
	controller.start();
	controller.sync();

//	sound.start();

	static auto console = spdlog::stdout_color_mt("event_loop"); 

	RawOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.StartEncoder();
	std::vector<std::shared_ptr<libcamera::Camera>> cameras = app.GetCameras();
	if (cameras.size() == 0)
    	throw std::runtime_error("no cameras available");
	app.GetOptions()->model = app.CameraModel();

	for (unsigned int count = 0; ; count++)
	{
		// if we change the sensor mode, restart the camera. 
		if(controller.configChanged()){
			if(controller.cameraRunning){
				app.StopCamera();
				app.Teardown();
			}
			app.ConfigureVideo(CinePIRecorder::FLAG_VIDEO_RAW, 0);
			app.StartCamera();
			controller.cameraRunning = true;

			libcamera::StreamConfiguration const &cfg = app.RawStream()->configuration();
			console->info("Raw stream: {}x{} : {} : {}", cfg.size.width, cfg.size.height, cfg.stride, cfg.pixelFormat.toString());
			app.GetEncoder()->reset_encoder();
			controller.process_stream_info(cfg);
		}

		CinePIRecorder::Msg msg = app.Wait();

//		controller.setShutterAngle(180.0);

		if (msg.type == RPiCamApp::MsgType::Quit)
			return;

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			console->error("Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type != CinePIRecorder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

		// parse the frame info metadata for the current frame, publish to redis stats channel
		controller.process(completed_request);

		// check for record trigger signal, open a new folder if rec_start or reset frame count if _rec_stop
		int trigger = controller.triggerRec();
		if(trigger > 0){
			controller.folderOpen = create_clip_folder(app.GetOptions(), controller.getClipNumber());
			app.GetEncoder()->resetFrameCount();
//			sound.record_start();
		} else if (trigger < 0){
			controller.folderOpen = false;
//			sound.record_stop();
		}

		// send frame to dng encoder and save to disk
		if(controller.isRecording() && controller.folderOpen){
			// check to make sure our buffer is not full, stop recording if so. 
			if(app.GetEncoder()->buffer_full()){
				controller.setRecording(false);
			}
			app.EncodeBuffer(completed_request, app.RawStream(), app.LoresStream());
		}

		// show frame on display
		app.ShowPreview(completed_request, app.GetMainStream());       

		console->info("Frame Number: {}", count);
	}
}

int main(int argc, char *argv[])
{
	try
	{
		CinePIRecorder app;
		CinePISound sound(&app);
		CinePIController controller(&app);

		// libcamera::logSetTarget(libcamera::LoggingTargetNone);

		RawOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			options->mediaDest = "/media/RAW";
			options->rawCrop[0] = 0;
			options->rawCrop[1] = 0;
			options->rawCrop[2] = 0;
			options->rawCrop[3] = 0;

			if (options->verbose >= 2)
				options->Print();

			event_loop(app, controller, sound);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
