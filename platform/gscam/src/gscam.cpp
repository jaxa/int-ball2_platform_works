#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>


#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_calibration_parsers/parse_ini.h>

#include <gscam/gscam.h>

# include "platform_msgs/CameraStatus.h"
# include "platform_msgs/PowerStatus.h"

namespace gscam {

  GSCam::GSCam(ros::NodeHandle nh_camera) :
    gsconfig_(""),
    pipeline_(NULL),
    sink_(NULL),
    nh_(nh_camera),
    image_transport_(nh_camera),
    camera_info_manager_(nh_camera),
    restart_once_(false),
    streaming_on_(false),
    image_height_(1024),
    image_width_(1280)
  {
    pipeline_format_streaming_on_ = "nvarguscamerasrc sensor-id=%d "
                                    "wbmode=%d "
                                    "ispdigitalgainrange=\"%f %f\" "
                                    "exposuretimerange=\"%d %d\" ! "
                                    "video/x-raw(memory:NVMM), "
                                    "width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
                                    "tee name=t "
                                    "t. ! queue ! nvvidconv top=%d bottom=%d left=%d right=%d ! "
                                    "omxh264enc control-rate=2 bitrate=%d profile=2 ! "
                                    "video/x-h264, width=(int)%d, height=(int)%d, stream-format=byte-stream ! "
                                    "h264parse config-interval=3 ! "
                                    "mpegtsmux alignment=7 ! "
                                    "rndbuffersize max=1316 min=1316 ! "
                                    "tsparse ! queue ! "
                                    "multiudpsink clients=%s:%d bind-port=%d force-ipv4=true sync=false async=false "
                                    "t. ! queue ! nvvidconv flip-method=0 top=%d bottom=%d left=%d right=%d ! "
                                    "video/x-raw, format=(string)RGBA ! videoconvert";

    pipeline_format_streaming_off_ = "nvarguscamerasrc sensor-id=%d "
                                      "wbmode=%d "
                                      "ispdigitalgainrange=\"%f %f\" "
                                      "exposuretimerange=\"%d %d\" ! "
                                      "video/x-raw(memory:NVMM), "
                                      "width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
                                      "queue ! nvvidconv flip-method=0 top=%d bottom=%d left=%d right=%d ! "
                                      "video/x-raw, format=(string)RGBA ! videoconvert";

    update_params_server_ = nh_.advertiseService("update_params", &GSCam::update_params, this);
    status_pub_ = nh_.advertise<platform_msgs::CameraStatus>("status", 1);
  }

  GSCam::~GSCam()
  {
  }

  bool GSCam::configure()
  {
    if(!load_modifiable_rosparams()) {
      return false;
    }
    nh_.param("status_publish_rate", status_publish_rate_, 1.0);

    // Get additional gscam configuration
    nh_.param("sync_sink", sync_sink_, true);
    nh_.param("preroll", preroll_, false);
    nh_.param("use_gst_timestamps", use_gst_timestamps_, false);

    nh_.param("reopen_on_eof", reopen_on_eof_, false);

    // Get the camera parameters filf
    nh_.getParam("camera_info_url", camera_info_url_);
    nh_.getParam("camera_name", camera_name_);

    // Get the image encoding
    nh_.param("image_encoding", image_encoding_, sensor_msgs::image_encodings::RGB8);
    if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
        image_encoding_ != sensor_msgs::image_encodings::MONO8 && 
        image_encoding_ != "jpeg") {
      ROS_FATAL_STREAM("Unsupported image encoding: " + image_encoding_);
    }

    camera_info_manager_.setCameraName(camera_name_);

    if(camera_info_manager_.validateURL(camera_info_url_)) {
      camera_info_manager_.loadCameraInfo(camera_info_url_);
      ROS_INFO_STREAM("Loaded camera calibration from "<<camera_info_url_);
    } else {
      ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
    }

    if(status_publish_timer_ == nullptr) {
      status_publish_timer_ = nh_.createTimer(ros::Rate(status_publish_rate_), &GSCam::publish_status, this);
    }

    return true;
  }

  bool GSCam::init_stream()
  {
    if(!gst_is_initialized()) {
      // Initialize gstreamer pipeline
      ROS_DEBUG_STREAM( "Initializing gstreamer..." );
      gst_init(0,0);
    }

    ROS_DEBUG_STREAM( "Gstreamer Version: " << gst_version_string() );

    GError *error = 0; // Assignment to zero is a gst requirement

    pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
    if (pipeline_ == NULL) {
      ROS_FATAL_STREAM( error->message );
      return false;
    }

    // Create RGB sink
    sink_ = gst_element_factory_make("appsink",NULL);
    GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

#if (GST_VERSION_MAJOR == 1)
    // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        caps = gst_caps_new_simple( "video/x-raw", 
            "format", G_TYPE_STRING, "RGB",
            NULL); 
    } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
        caps = gst_caps_new_simple( "video/x-raw", 
            "format", G_TYPE_STRING, "GRAY8",
            NULL); 
    } else if (image_encoding_ == "jpeg") {
        caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
    }
#else
    if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        caps = gst_caps_new_simple( "video/x-raw-rgb", NULL,NULL); 
    } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
        caps = gst_caps_new_simple("video/x-raw-gray", NULL, NULL);
    } else if (image_encoding_ == "jpeg") {
        caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
    }
#endif

    gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
    gst_caps_unref(caps);

    // Set whether the sink should sync
    // Sometimes setting this to true can cause a large number of frames to be
    // dropped
    gst_base_sink_set_sync(
        GST_BASE_SINK(sink_),
        (sync_sink_) ? TRUE : FALSE);

    if(GST_IS_PIPELINE(pipeline_)) {
      GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
      g_assert(outpad);

      GstElement *outelement = gst_pad_get_parent_element(outpad);
      g_assert(outelement);
      gst_object_unref(outpad);

      if(!gst_bin_add(GST_BIN(pipeline_), sink_)) {
        ROS_FATAL("gst_bin_add() failed");
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      if(!gst_element_link(outelement, sink_)) {
        ROS_FATAL("GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      gst_object_unref(outelement);
    } else {
      GstElement* launchpipe = pipeline_;
      pipeline_ = gst_pipeline_new(NULL);
      g_assert(pipeline_);

      gst_object_unparent(GST_OBJECT(launchpipe));

      gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

      if(!gst_element_link(launchpipe, sink_)) {
        ROS_FATAL("GStreamer: cannot link launchpipe -> sink");
        gst_object_unref(pipeline_);
        return false;
      }
    }

    // Calibration between ros::Time and gst timestamps
    GstClock * clock = gst_system_clock_obtain();
    ros::Time now = ros::Time::now();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct)/1e6;
    ROS_INFO("Time offset: %.3f",time_offset_);

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
      return false;
    } else {
      ROS_DEBUG_STREAM("Stream is PAUSED.");
    }

    // Create ROS camera interface
    if (image_encoding_ == "jpeg") {
        jpeg_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("image_raw/compressed",1);
        cinfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info",1);
    } else {
        camera_pub_ = image_transport_.advertiseCamera("image_raw", 1);
    }

    return true;
  }

  void GSCam::publish_stream()
  {
    ROS_INFO_STREAM("Publishing stream...");

    // Pre-roll camera if needed
    if (preroll_) {
      ROS_DEBUG("Performing preroll...");

      //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
      //I am told this is needed and am erring on the side of caution.
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PLAY during preroll.");
        return;
      } else {
        ROS_DEBUG("Stream is PLAYING in preroll.");
      }

      gst_element_set_state(pipeline_, GST_STATE_PAUSED);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PAUSE.");
        return;
      } else {
        ROS_INFO("Stream is PAUSED in preroll.");
      }
    }

    if(gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Could not start stream!");
      return;
    }
    ROS_INFO("Started stream.");

    while(ros::ok() && !restart_once_) 
    {
      // This should block until a new frame is awake, this way, we'll run at the
      // actual capture framerate of the device.
      // ROS_DEBUG("Getting data...");
#if (GST_VERSION_MAJOR == 1)
      GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
      if(!sample) {
        ROS_ERROR("Could not get gstreamer sample.");
        break;
      }
      GstBuffer* buf = gst_sample_get_buffer(sample);
      GstMemory *memory = gst_buffer_get_memory(buf, 0);
      GstMapInfo info;

      gst_memory_map(memory, &info, GST_MAP_READ);
      gsize &buf_size = info.size;
      guint8* &buf_data = info.data;
#else
      GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink_));
      guint &buf_size = buf->size;
      guint8* &buf_data = buf->data;
#endif
      GstClockTime bt = gst_element_get_base_time(pipeline_);
      // ROS_INFO("New buffer: timestamp %.6f %lu %lu %.3f",
      //         GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_, buf->timestamp, bt, time_offset_);


#if 0
      GstFormat fmt = GST_FORMAT_TIME;
      gint64 current = -1;

       Query the current position of the stream
      if (gst_element_query_position(pipeline_, &fmt, &current)) {
          ROS_INFO_STREAM("Position "<<current);
      }
#endif

      // Stop on end of stream
      if (!buf) {
        ROS_INFO("Stream ended.");
        break;
      }

      // ROS_DEBUG("Got data.");

      // Get the image width and height
      GstPad* pad = gst_element_get_static_pad(sink_, "sink");
#if (GST_VERSION_MAJOR == 1)
      const GstCaps *caps = gst_pad_get_current_caps(pad);
#else
      const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
#endif
      GstStructure *structure = gst_caps_get_structure(caps,0);
      gst_structure_get_int(structure,"width",&width_);
      gst_structure_get_int(structure,"height",&height_);

      // Update header information
      sensor_msgs::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
      sensor_msgs::CameraInfoPtr cinfo;
      cinfo.reset(new sensor_msgs::CameraInfo(cur_cinfo));
      if (use_gst_timestamps_) {
#if (GST_VERSION_MAJOR == 1)
          cinfo->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->pts+bt)/1e6+time_offset_);
#else
          cinfo->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_);
#endif
      } else {
          cinfo->header.stamp = ros::Time::now();
      }
      // ROS_INFO("Image time stamp: %.3f",cinfo->header.stamp.toSec());
      cinfo->header.frame_id = frame_id_;
      if (image_encoding_ == "jpeg") {
          sensor_msgs::CompressedImagePtr img(new sensor_msgs::CompressedImage());
          img->header = cinfo->header;
          img->format = "jpeg";
          img->data.resize(buf_size);
          std::copy(buf_data, (buf_data)+(buf_size),
                  img->data.begin());
          jpeg_pub_.publish(img);
          cinfo_pub_.publish(cinfo);
      } else {
          // Complain if the returned buffer is smaller than we expect
          const unsigned int expected_frame_size =
              image_encoding_ == sensor_msgs::image_encodings::RGB8
              ? width_ * height_ * 3
              : width_ * height_;

          if (buf_size < expected_frame_size) {
              ROS_WARN_STREAM( "GStreamer image buffer underflow: Expected frame to be "
                      << expected_frame_size << " bytes but got only "
                      << (buf_size) << " bytes. (make sure frames are correctly encoded)");
          }

          // Construct Image message
          sensor_msgs::ImagePtr img(new sensor_msgs::Image());

          img->header = cinfo->header;

          // Image data and metadata
          img->width = width_;
          img->height = height_;
          img->encoding = image_encoding_;
          img->is_bigendian = false;
          img->data.resize(expected_frame_size);

          // Copy only the data we received
          // Since we're publishing shared pointers, we need to copy the image so
          // we can free the buffer allocated by gstreamer
          if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
              img->step = width_ * 3;
          } else {
              img->step = width_;
          }
          std::copy(
                  buf_data,
                  (buf_data)+(buf_size),
                  img->data.begin());

          // Publish the image/info
          camera_pub_.publish(img, cinfo);
      }

      // Release the buffer
      if(buf) {
#if (GST_VERSION_MAJOR == 1)
        gst_memory_unmap(memory, &info);
        gst_memory_unref(memory);
#endif
        gst_buffer_unref(buf);
      }

      ros::spinOnce();
    }
  }

  void GSCam::cleanup_stream()
  {
    // Clean up
    ROS_INFO("Stopping gstreamer pipeline...");
    if(pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = NULL;
    }
  }

  bool GSCam::load_modifiable_rosparams()
  {
    if(!nh_.getParam("sensor_id", sensor_id_)) {
        ROS_FATAL( "Can't get rosparam \"sensor_id\"" );
        return false;
    }
    if(!nh_.getParam("white_balance_mode", white_balance_mode_)) {
        ROS_FATAL( "Can't get rosparam \"white_balance_mode\"" );
        return false;
    }
    if(!nh_.getParam("camera_gain_min", camera_gain_min_)) {
        ROS_FATAL( "Can't get rosparam \"camera_gain_min\"" );
        return false;
    }
    if(!nh_.getParam("camera_gain_max", camera_gain_max_)) {
        ROS_FATAL( "Can't get rosparam \"camera_gain_max\"" );
        return false;
    }
    if(!nh_.getParam("fps", frame_rate_)) {
        ROS_FATAL( "Can't get rosparam \"fps\"" );
        return false;
    }
    if(!nh_.getParam("resolution_type", resolution_type_)) {
        ROS_FATAL( "Can't get rosparam \"resolution_type\"" );
        return false;
    }
	  if(!nh_.getParam("resolution_settings", resolution_settings_)) {
        ROS_FATAL( "Can't get rosparam \"resolution_settings\"" );
        return false;
    }
    if(!nh_.getParam("camera_zoom_min", camera_zoom_min_)) {
        ROS_FATAL( "Can't get rosparam \"camera_zoom_min\"" );
        return false;
    }
    if(!nh_.getParam("camera_zoom_max", camera_zoom_max_)) {
        ROS_FATAL( "Can't get rosparam \"camera_zoom_max\"" );
        return false;
    }
    if(!nh_.getParam("camera_EV", camera_EV_)) {
        ROS_FATAL( "Can't get rosparam \"camera_EV\"" );
        return false;
    }
    if(!nh_.getParam("lens_f_number", lens_f_number_)) {
        ROS_FATAL( "Can't get rosparam \"lens_f_number\"" );
        return false;
    }
    if(!nh_.getParam("camera_exposure_time_threshold_min", camera_exposure_time_threshold_min_)) {
        ROS_FATAL( "Can't get rosparam \"camera_exposure_time_threshold_min\"" );
        return false;
    }
    if(!nh_.getParam("camera_exposure_time_threshold_max", camera_exposure_time_threshold_max_)) {
        ROS_FATAL( "Can't get rosparam \"camera_exposure_time_threshold_max\"" );
        return false;
    }
    if(!nh_.getParam("bit_rate", bit_rate_)) {
        ROS_FATAL( "Can't get rosparam \"bit_rate\"" );
        return false;
    }
    if(!nh_.getParam("target_host_ip", target_host_ip_)) {
        ROS_FATAL( "Can't get rosparam \"target_host_ip\"" );
        return false;
    }
    if(!nh_.getParam("target_host_port", target_host_port_)) {
        ROS_FATAL( "Can't get rosparam \"target_host_port\"" );
        return false;
    }
    if(!nh_.getParam("streaming_bind_port", streaming_bind_port_)) {
        ROS_FATAL( "Can't get rosparam \"streaming_bind_port\"" );
        return false;
    }
    if(!nh_.getParam("streaming_on", streaming_on_)) {
        ROS_FATAL( "Can't get rosparam \"streaming_on\"" );
        return false;
    }
    if(!nh_.getParam("frame_id", frame_id_)) {
        ROS_FATAL( "Can't get rosparam \"frame_id\"" );
        return false;
    }

    camera_gain_ = check_in_range_and_get_param("camera_gain", camera_gain_min_, camera_gain_max_);
    if(camera_gain_ < 0) {
        return false;
    }
    camera_zoom_ = check_in_range_and_get_param("camera_zoom", camera_zoom_min_, camera_zoom_max_);
    if(camera_zoom_ < 0) {
        return false;
    }

    set_image_width_and_height(resolution_type_);
    set_exposure_time_nsec(camera_EV_);
    set_cropped_image_area(camera_zoom_);

    return setup_pipeline(streaming_on_);
  }

  float GSCam::check_in_range_and_get_param(const std::string rosparam_name, const float value_min, const float value_max) {
    float value;

    nh_.getParam(rosparam_name, value);
    if (!(value_min <= value <= value_max)) {
      ROS_WARN("%s parameter is out of range [%f, %f].",
               rosparam_name.c_str(), value_min, value_max);
      ROS_DEBUG("GSCam::check_in_range_and_get_param out");

      //raise ValueError
      ROS_ERROR("%s parameter is out of range", rosparam_name.c_str());
	    return -1;
    }
      return value;
  }


  void GSCam::set_exposure_time_nsec(const float exposure_value) {
    ROS_DEBUG("GSCam::set_exposure_time_nsec in");
    exposure_time_ = lens_f_number_ * lens_f_number_ / std::pow(2, exposure_value);
    if (exposure_time_ <= camera_exposure_time_threshold_min_ or
        exposure_time_ >= camera_exposure_time_threshold_max_) {
      ROS_ERROR("Exposure time [%f] is invalid", exposure_time_);
    } else {
        camera_exposure_time_ = std::floor(exposure_time_ * 1000000000);
    }
    ROS_DEBUG("GSCam::set_exposure_time_nsec out");
  }

  void GSCam::set_image_width_and_height(const int resolution_type) {
    ROS_DEBUG("GSCam::set_image_width_and_height in");
    
    int icnt;

    if (!(resolution_type == reso_type_.HD or 
          resolution_type == reso_type_.FullHD or
          resolution_type == reso_type_.Four_K)) {
      ROS_WARN("requested resolution type [%d] is none of HD(%d), FullHD(%d) and Four_K(%d).",
               resolution_type, reso_type_.HD, reso_type_.FullHD, reso_type_.Four_K);
      ROS_DEBUG("GSCam::set_image_width_and_height out");
      return;
    }
    for (icnt = 0 ; icnt < resolution_settings_.size() ; icnt++) {
      if (static_cast<int>(resolution_settings_[icnt]["id"]) == resolution_type) {
        image_width_ = static_cast<int>(resolution_settings_[icnt]["image_width"]);
        image_height_ = static_cast<int>(resolution_settings_[icnt]["image_height"]);
        break;
      }
	  }

    ROS_DEBUG("GSCam::set_image_width_and_height out");
  }

  void GSCam::set_cropped_image_area(const float zoom_rate) {
    ROS_DEBUG("GSCam::set_cropped_image_area in");

    if (zoom_rate < camera_zoom_min_ or zoom_rate > camera_zoom_max_) {
      ROS_ERROR("zoom rate [%f] is invalid", zoom_rate);
    } else {
      // set cropping area
      int cropped_image_width = static_cast<int>(image_width_ / zoom_rate);
      int cropped_image_height = static_cast<int>(image_height_ / zoom_rate);
      cropped_image_top_ = static_cast<int>(image_height_ * (zoom_rate - 1.0) /
                                            (2.0 * (zoom_rate + 1.0)));
      cropped_image_bottom_ = cropped_image_top_ + cropped_image_height;
      cropped_image_left_ = static_cast<int>(image_width_ * (zoom_rate - 1.0) /
                                             (2.0 * (zoom_rate + 1.0)));
      cropped_image_right_ = cropped_image_left_ + cropped_image_width;
    }

    ROS_DEBUG("GSCam::set_cropped_image_area out");
  }

  bool GSCam::setup_pipeline(const bool streaming) {
    ROS_DEBUG("GSCam::setup_pipeline in");

    if(streaming) {
      std::string pipeline_string_format = pipeline_format_streaming_on_;

      int gs_pipeline_length = printf(pipeline_string_format.c_str(),
                                      sensor_id_,
                                      white_balance_mode_,
                                      camera_gain_, camera_gain_,
                                      camera_exposure_time_, camera_exposure_time_,
                                      image_width_, image_height_, frame_rate_,
                                      cropped_image_top_, cropped_image_bottom_, cropped_image_left_, cropped_image_right_,
                                      bit_rate_,
                                      image_width_, image_height_,
                                      target_host_ip_.c_str(), target_host_port_, streaming_bind_port_,
                                      cropped_image_top_, cropped_image_bottom_, cropped_image_left_, cropped_image_right_);
      if (gs_pipeline_length < 0) {
          ROS_FATAL( "Can't define GStreamer pipeline strings." );
          return false;
      }

      // add gs_pipeline_length to 1 (\0)
      gs_pipeline_length++;
      char gsconfig_buffer[gs_pipeline_length];

      int gsconfig_buffer_result = snprintf(gsconfig_buffer,
                                            gs_pipeline_length,
                                            pipeline_string_format.c_str(),
                                            sensor_id_,
                                            white_balance_mode_,
                                            camera_gain_, camera_gain_,
                                            camera_exposure_time_, camera_exposure_time_,
                                            image_width_, image_height_, frame_rate_,
                                            cropped_image_top_, cropped_image_bottom_, cropped_image_left_, cropped_image_right_,
                                            bit_rate_,
                                            image_width_, image_height_,
                                            target_host_ip_.c_str(), target_host_port_, streaming_bind_port_,
                                            cropped_image_top_, cropped_image_bottom_, cropped_image_left_, cropped_image_right_);
      if (gsconfig_buffer_result < 0) {
        ROS_FATAL( "Can't define GStreamer pipeline strings." );
        return false;
      }

      gsconfig_ = std::string(gsconfig_buffer);
      ROS_INFO_STREAM("Setup gstreamer pipeline: \"" << gsconfig_ << "\"");

    }else{
      std::string pipeline_string_format = pipeline_format_streaming_off_;

      int gs_pipeline_length = printf(pipeline_string_format.c_str(),
                                      sensor_id_,
                                      white_balance_mode_,
                                      camera_gain_, camera_gain_,
                                      camera_exposure_time_, camera_exposure_time_,
                                      image_width_, image_height_, frame_rate_,
                                      cropped_image_top_, cropped_image_bottom_, cropped_image_left_, cropped_image_right_);
      if (gs_pipeline_length < 0) {
          ROS_FATAL( "Can't define GStreamer pipeline strings." );
          return false;
      }

      // add gs_pipeline_length to 1 (\0)
      gs_pipeline_length++;
      char gsconfig_buffer[gs_pipeline_length];

      int gsconfig_buffer_result = snprintf(gsconfig_buffer,
                                            gs_pipeline_length,
                                            pipeline_string_format.c_str(),
                                            sensor_id_,
                                            white_balance_mode_,
                                            camera_gain_, camera_gain_,
                                            camera_exposure_time_, camera_exposure_time_,
                                            image_width_, image_height_, frame_rate_,
                                            cropped_image_top_, cropped_image_bottom_, cropped_image_left_, cropped_image_right_);
      if (gsconfig_buffer_result < 0) {
        ROS_FATAL( "Can't define GStreamer pipeline strings." );
        return false;
      }

      gsconfig_ = std::string(gsconfig_buffer);
      ROS_INFO_STREAM("Setup gstreamer pipeline: \"" << gsconfig_ << "\"");
    }

    ROS_DEBUG("GSCam::setup_pipeline out");
    return true;
  }

  bool GSCam::update_params(ib2_msgs::UpdateParameter::Request&, ib2_msgs::UpdateParameter::Response& res) {
    res.stamp = ros::Time::now();

    if(load_modifiable_rosparams()) {
      ROS_INFO("Update parameter succeeded.");
      res.status = ib2_msgs::UpdateParameter::Response::SUCCESS;
      restart_once_ = true;
    }else{
      ROS_ERROR("Update parameter failed.");
      res.status = ib2_msgs::UpdateParameter::Response::FAILURE_UPDATE;
      return false;
    }

    return true;
  }

  void GSCam::publish_status(const ros::TimerEvent& /*_event*/) {
    auto msg = platform_msgs::CameraStatus();
    msg.streaming_status.status = streaming_on_ ? 
      platform_msgs::PowerStatus::ON : platform_msgs::PowerStatus::OFF;
    status_pub_.publish(msg);
  }

  void GSCam::run() {
    while(ros::ok()) {
      if(!this->configure()) {
        ROS_FATAL("Failed to configure gscam!");
        break;
      }

      if(!this->init_stream()) {
        ROS_FATAL("Failed to initialize gscam stream!");
        break;
      }

      // Block while publishing
      this->publish_stream();

      this->cleanup_stream();

      ROS_INFO("GStreamer stream stopped!");

      if(reopen_on_eof_) {
        ROS_INFO("Reopening stream...");
        restart_once_ = false;
      } else if(restart_once_) {
        ROS_INFO("Reopening stream (restart_once_ = true)...");
        restart_once_ = false;
      } else {
        ROS_INFO("Cleaning up stream and exiting...");
        break;
      }
    }

  }

  // Example callbacks for appsink
  void gst_eos_cb(GstAppSink *appsink, gpointer user_data ) {
  }
  GstFlowReturn gst_new_preroll_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }
  GstFlowReturn gst_new_asample_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }

}
