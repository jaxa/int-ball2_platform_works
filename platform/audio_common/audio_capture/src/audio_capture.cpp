#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "audio_common_msgs/AudioData.h"
#include "audio_common_msgs/AudioInfo.h"

#include "ib2_msgs/UpdateParameter.h"

#include "platform_msgs/MicrophoneStatus.h"

namespace audio_transport
{
  class RosGstCapture
  {
    public:
      RosGstCapture()
      {
	_nh = ros::NodeHandle("~");
        _pipeline_format_streaming_on = "pulsesrc device=%s ! tee name=t "
                                        "t. ! queue ! audioparse rate=%d channels=%d use-sink-caps=true ! queue ! decodebin ! audioresample ! audioconvert ! "
                                        "audiorate ! voaacenc ! aacparse ! queue ! mpegtsmux alignment=7 ! queue ! rndbuffersize max=1316 min=1316 ! tsparse ! "
                                        "queue ! multiudpsink clients=%s:%d bind-port=%d force-ipv4=true sync=false async=false "
                                        "t. ! queue ! audio/x-raw, format=%s, channels=%d, width=16, depth=%d, rate=%d, signed=TRUE ";

        _pipeline_format_streaming_off = "pulsesrc device=%s ! "
                                          "audio/x-raw, format=%s, channels=%d, width=16, depth=%d, rate=%d, signed=TRUE ";

        _pipeline_format_mp3_suffix = " ! audioconvert ! lamemp3enc target=1 bitrate=%d";

        _pipeline_format_wave_suffix = " ! wavenc";

        std::string dst_type;

        // Need to encoding or publish raw wave data
        ros::param::param<std::string>("~sample_format", _sample_format, "S16LE");

        // only available for raw data
        ros::param::param<int>("~channels", _channels, 1);
        ros::param::param<int>("~depth", _depth, 16);
        ros::param::param<int>("~sample_rate", _sample_rate, 16000);

        // The destination of the audio
        ros::param::param<std::string>("~dst", dst_type, "appsink");

        if(!_nh.getParam("device", _device)) {
            ROS_FATAL( "Can't get rosparam \"device\"" );
            std::runtime_error("Can't get rosparam \"device\""); 
        }
        _nh.param("status_publish_rate", _status_publish_rate, 1.0);
        if(!loadReloadableRosparams()) {
            std::runtime_error("loadReloadableRosparams failed"); 
        }

        _pub = _nh.advertise<audio_common_msgs::AudioData>("audio", 10, true);
        _pub_info = _nh.advertise<audio_common_msgs::AudioInfo>("audio_info", 1, true);
        _updateParams_srv = _nh.advertiseService("update_params", &RosGstCapture::updateParams, this);
        _status_publish_timer = _nh.createTimer(ros::Duration(1.0 / _status_publish_rate), &RosGstCapture::publishStatus, this);
        _pub_status = _nh.advertise<platform_msgs::MicrophoneStatus>("status", 1, true);

        startGstThread();
      }

      ~RosGstCapture()
      {
        quitGstThread();
      }

      void startGstThread()
      {
        setupPipeline();
        _loop = g_main_loop_new(NULL, false);
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error", G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // Create sink
        _sink = gst_element_factory_make("appsink", NULL);
        g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
        g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
        g_signal_connect( G_OBJECT(_sink), "new-sample", G_CALLBACK(onNewBuffer), this);
							
        // Set whether the sink should sync
        // Sometimes setting this to true can cause a large number of frames to be
        // dropped

        GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(_pipeline), GST_PAD_SRC);
        g_assert(outpad);

        GstElement *outelement = gst_pad_get_parent_element(outpad);
        g_assert(outelement);
        gst_object_unref(outpad);

        if(!gst_bin_add(GST_BIN(_pipeline), _sink)) {
            ROS_FATAL("gst_bin_add() failed");
            gst_object_unref(outelement);
            gst_object_unref(_pipeline);
        }

        if(!gst_element_link(outelement, _sink)) {
            ROS_FATAL("GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
            gst_object_unref(outelement);
            gst_object_unref(_pipeline);
        }

        gst_object_unref(outelement);

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
        ROS_INFO("Started stream.");

        audio_common_msgs::AudioInfo info_msg;
        info_msg.channels = _channels;
        info_msg.sample_rate = _sample_rate;
        info_msg.sample_format = _sample_format;
        info_msg.bitrate = _bitrate;
        info_msg.coding_format = _format;
        _pub_info.publish(info_msg);
      }

      void quitGstThread()
      {
        if(_loop != NULL) {
          ROS_INFO("Stopping gstreamer pipeline...");
          g_main_loop_quit(_loop);

          gst_element_set_state(_pipeline, GST_STATE_NULL);
          gst_object_unref(_pipeline);
          _pipeline = NULL;

          g_main_loop_unref(_loop);
          _loop = NULL;

          _gst_thread.join();
        }
      }

      void exitOnMainThread(int code)
      {
        exit(code);
      }
      
      bool loadReloadableRosparams()
      {
        if(!_nh.getParam("bitrate", _bitrate)) {
            ROS_FATAL( "Can't get rosparam \"bitrate\"" );
            return false;
        }
        if(!_nh.getParam("mp3_encode", _mp3_encode)) {
            ROS_FATAL( "Can't get rosparam \"mp3_encode\"" );
            return false;
        }
        if(!_nh.getParam("streaming_on", _streaming_on)) {
            ROS_FATAL( "Can't get rosparam \"streaming_on\"" );
            return false;
        }
        if(!_nh.getParam("target_host_ip", _target_host_ip)) {
            ROS_FATAL( "Can't get rosparam \"target_host_ip\"" );
            return false;
        }
        if(!_nh.getParam("target_host_port", _target_host_port)) {
            ROS_FATAL( "Can't get rosparam \"target_host_port\"" );
            return false;
        }
        if(!_nh.getParam("streaming_bind_port", _streaming_bind_port)) {
            ROS_FATAL( "Can't get rosparam \"streaming_bind_port\"" );
            return false;
        }
        return true;
      }

      void publish( const audio_common_msgs::AudioData &msg )
      {
        _pub.publish(msg);
      }

      void setupPipeline()
      {
        std::vector<char> pipeline_buffer;

        if(_streaming_on) {
          // Enable streaming and publish topics

          int pipeline_length;
          if(_mp3_encode) {
            // publish MP3-encoded audio data.
            pipeline_length = printf((_pipeline_format_streaming_on + _pipeline_format_mp3_suffix).c_str(),
                                      _device.c_str(),
                                      _sample_rate, _channels,
                                      _target_host_ip.c_str(), _target_host_port, _streaming_bind_port,
                                      _sample_format.c_str(), _channels, _depth, _sample_rate, _bitrate);
          }else{
            pipeline_length = printf((_pipeline_format_streaming_on + _pipeline_format_wave_suffix).c_str(),
                                      _device.c_str(),
                                      _sample_rate, _channels,
                                      _target_host_ip.c_str(), _target_host_port, _streaming_bind_port,
                                      _sample_format.c_str(), _channels, _depth, _sample_rate);
          }

          // add gs_pipeline_length to 1 (\0)
          pipeline_length++;
          pipeline_buffer.resize(pipeline_length);

          if(_mp3_encode) {
            // publish MP3-encoded audio data.
            snprintf(pipeline_buffer.data(), pipeline_length,
                      (_pipeline_format_streaming_on + _pipeline_format_mp3_suffix).c_str(),
                      _device.c_str(),
                      _sample_rate, _channels,
                      _target_host_ip.c_str(), _target_host_port, _streaming_bind_port,
                      _sample_format.c_str(), _channels, _depth, _sample_rate, _bitrate);

          } else {
            snprintf(pipeline_buffer.data(), pipeline_length,
                      (_pipeline_format_streaming_on + _pipeline_format_wave_suffix).c_str(),
                      _device.c_str(),
                      _sample_rate, _channels,
                      _target_host_ip.c_str(), _target_host_port, _streaming_bind_port,
                      _sample_format.c_str(), _channels, _depth, _sample_rate);
          }

        } else {
          // only publish topics
          
          int pipeline_length;
          if(_mp3_encode) {
            // publish MP3-encoded audio data.
            pipeline_length = printf((_pipeline_format_streaming_off + _pipeline_format_mp3_suffix).c_str(),
                                    _device.c_str(),
                                    _sample_format.c_str(), _channels, _depth, _sample_rate, _bitrate);
          } else {
            pipeline_length = printf((_pipeline_format_streaming_off + _pipeline_format_wave_suffix).c_str(),
                                    _device.c_str(),
                                    _sample_format.c_str(), _channels, _depth, _sample_rate);
          }

          // add gs_pipeline_length to 1 (\0)
          pipeline_length++;
          pipeline_buffer.resize(pipeline_length);

          if(_mp3_encode) {
            // publish MP3-encoded audio data.
            snprintf(pipeline_buffer.data(), pipeline_length,
                    (_pipeline_format_streaming_off + _pipeline_format_mp3_suffix).c_str(),
                    _device.c_str(),
                    _sample_format.c_str(), _channels, _depth, _sample_rate, _bitrate);
          } else {
            snprintf(pipeline_buffer.data(), pipeline_length,
                    (_pipeline_format_streaming_off + _pipeline_format_wave_suffix).c_str(),
                    _device.c_str(),
                    _sample_format.c_str(), _channels, _depth, _sample_rate);
          }
        }

        GError *err = NULL;
        ROS_INFO("Build pipeline: %s", pipeline_buffer.data());
        _pipeline = gst_parse_launch(pipeline_buffer.data(), &err);
        if(err) {
          ROS_FATAL("gst_parse_launch failed.");
          throw std::runtime_error("gst_parse_launch failed.");
        }
      }

      bool updateParams(ib2_msgs::UpdateParameter::Request&, ib2_msgs::UpdateParameter::Response& res) {
        res.stamp = ros::Time::now();

        if(loadReloadableRosparams()) {
          quitGstThread();
          startGstThread();
          ROS_INFO("Update parameter succeeded.");
          res.status = ib2_msgs::UpdateParameter::Response::SUCCESS;
        }else{
          ROS_ERROR("Update parameter failed.");
          res.status = ib2_msgs::UpdateParameter::Response::FAILURE_UPDATE;
          return false;
        }

        return true;
      }

      void publishStatus(const ros::TimerEvent& /*_event*/) {
        auto msg = platform_msgs::MicrophoneStatus();
        msg.streaming_status.status = _streaming_on ? 
          platform_msgs::PowerStatus::ON : platform_msgs::PowerStatus::OFF;
        _pub_status.publish(msg);
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        audio_common_msgs::AudioData msg;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize( map.size );

        memcpy( &msg.data[0], map.data, map.size );

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        server->publish(msg);

        return GST_FLOW_OK;
      }

      static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        ROS_ERROR_STREAM("gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
      }

    private:
      ros::NodeHandle _nh;
      ros::Publisher _pub;
      ros::Publisher _pub_info;

      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_filter, *_sink, *_convert, *_encode;
      GstBus *_bus;
      int _bitrate, _channels, _depth, _sample_rate;
      GMainLoop *_loop;
      std::string _format, _sample_format;

      // Int-Ball2 specific configuration
      bool _mp3_encode;
      bool _streaming_on;
      double _status_publish_rate;
      int _streaming_bind_port;
      int _target_host_port;
      std::string _device;
      std::string _pipeline_format_mp3_suffix;
      std::string _pipeline_format_wave_suffix;
      std::string _pipeline_format_streaming_off;
      std::string _pipeline_format_streaming_on;
      std::string _target_host_ip;

      // Update Parameter Service
      ros::ServiceServer _updateParams_srv;

      // Microphone status publisher
      ros::Publisher _pub_status;

      // Timer for publish status
      ros::Timer _status_publish_timer;

  };
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "microphone");
  gst_init(&argc, &argv);

  audio_transport::RosGstCapture server;
  ros::spin();
}
