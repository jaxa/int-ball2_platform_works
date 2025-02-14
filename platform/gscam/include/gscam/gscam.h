#ifndef __GSCAM_GSCAM_H
#define __GSCAM_GSCAM_H

extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <stdexcept>

#include "ib2_msgs/MainCameraResolutionType.h"
#include "ib2_msgs/UpdateParameter.h"
#include <XmlRpcValue.h>

namespace gscam {

  class GSCam {
  public:
    GSCam(ros::NodeHandle nh_camera);
    ~GSCam();

    bool configure();
    bool init_stream();
    void publish_stream();
    void cleanup_stream();

    bool load_modifiable_rosparams();
    bool setup_pipeline(const bool streaming);
    bool update_params(ib2_msgs::UpdateParameter::Request&, ib2_msgs::UpdateParameter::Response& res);
    float check_in_range_and_get_param(const std::string rosparam_name, const float value_min, const float value_max);
    void publish_status(const ros::TimerEvent& event);
    void set_cropped_image_area(const float zoom_rate);
    void set_exposure_time_nsec(const float exposure_value);
    void set_image_width_and_height(const int resolution_type);

    void run();

  private:
    // General gstreamer configuration
    std::string gsconfig_;

    // Gstreamer structures
    GstElement *pipeline_;
    GstElement *sink_;

    // Appsink configuration
    bool sync_sink_;
    bool preroll_;
    bool reopen_on_eof_;
    bool use_gst_timestamps_;

    // Camera publisher configuration
    std::string frame_id_;
    int width_, height_;
    std::string image_encoding_;
    std::string camera_name_;
    std::string camera_info_url_;

    // Int-Ball2 specific configuration
    float       exposure_time_;
    bool        restart_once_;
    bool        streaming_on_;
    double      status_publish_rate_;
    float       camera_EV_;
    float       camera_gain_;
    float       camera_gain_max_;
    float       camera_gain_min_;
    float       camera_zoom_;
    float       camera_zoom_max_;
    float       camera_zoom_min_;
    float       lens_f_number_;
    int         bit_rate_;
    int         camera_exposure_time_;
    int         camera_exposure_time_threshold_max_;
    int         camera_exposure_time_threshold_min_;
    int         cropped_image_bottom_;
    int         cropped_image_left_;
    int         cropped_image_right_;
    int         cropped_image_top_;
    int         frame_rate_;
    int         image_height_;
    int         image_width_;
    int         resolution_type_;
    int         sensor_id_;
    int         streaming_bind_port_;
    int         target_host_port_;
    int         white_balance_mode_;
    std::string pipeline_format_streaming_off_;
    std::string pipeline_format_streaming_on_;
    std::string target_host_ip_;
    ib2_msgs::MainCameraResolutionType reso_type_;
    XmlRpc::XmlRpcValue resolution_settings_;

    // ROS Inteface
    // Calibration between ros::Time and gst timestamps
    double time_offset_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport_;
    camera_info_manager::CameraInfoManager camera_info_manager_;
    image_transport::CameraPublisher camera_pub_;
    // Case of a jpeg only publisher
    ros::Publisher jpeg_pub_;
    ros::Publisher cinfo_pub_;

    // Int-Ball2 Update Parameter Service
    ros::ServiceServer update_params_server_;

    // Camera status publisher
    ros::Publisher status_pub_;

    // Timer for publish status
    ros::Timer status_publish_timer_;
  };

}

#endif // ifndef __GSCAM_GSCAM_H
