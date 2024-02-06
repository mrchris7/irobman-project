#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include "ros/ros.h"
#include "irobman_project/SetPoints.h"
#include "irobman_project/GetPoses.h"
#include "irobman_project/helper.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/PoseArray.msg"
#include "geometry_msgs/Pose.h"
#include <memory>
#include <string>
#include <filesystem>

#include <icg/zed_camera.h>
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/normal_viewer.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/renderer_geometry.h>
#include <icg/static_detector.h>
#include <icg/tracker.h>
#include <Eigen/Geometry>

class PoseEstimationNode {
public:
    PoseEstimationNode();

    bool handleToggleTracker(std_srvs::SetBool::Request& req,
                             std_srvs::SetBool::Response& res);

    bool handlePrepareTracker(irobman_project::SendPoints::Request& req,
                              irobman_project::SendPoints::Response& res);

    bool handleRetrievePoses(irobman_project::RetrievePoses::Request& req,
                             irobman_project::RetrievePoses::Response& res);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer toggle_service_;
    ros::ServiceServer prepare_service_;
    ros::ServiceServer retrieve_service_;
    ros::Publisher poses_pub_;

    std::shared_ptr<icg::Tracker> tracker_ptr_;
    std::shared_ptr<icg::RendererGeometry> renderer_geometry_ptr_;
    std::shared_ptr<icg::ZEDColorCamera> color_camera_ptr_;
    std::shared_ptr<icg::ZEDDepthCamera> depth_camera_ptr_;
    std::shared_ptr<icg::FocusedBasicDepthRenderer> color_depth_renderer_ptr_;
    std::shared_ptr<icg::FocusedBasicDepthRenderer> depth_depth_renderer_ptr_;
};

#endif // POSE_ESTIMATION_H
