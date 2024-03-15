#include "pose_estimation.h"

PoseEstimationNode::PoseEstimationNode() {
    nh_ = ros::NodeHandle("~");

    poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("cube_poses", 10);
    toggle_service_ = nh_.advertiseService("ToggleTracker", &PoseEstimationNode::handleToggleTracker, this);
    prepare_service_ = nh_.advertiseService("PrepareTracker", &PoseEstimationNode::handlePrepareTracker, this);
    retrieve_service_ = nh_.advertiseService("RetrieveTrackedPoses", &PoseEstimationNode::handleRetrievePoses, this);

    // Set up tracker
    directory_ = "/opt/ros_ws/src/irobman-project/objs/cube";

    constexpr bool kUseDepthViewer = false;
    constexpr bool kUseColorViewer = true;
    kMeasureOcclusions_ = true;
    kModelOcclusions_ = false;
    kVisualizePoseResult_ = false;
    constexpr bool kSaveImages = false;
    std::filesystem::path save_directory{""};

    // Set up tracker and renderer geometry
    tracker_ptr_ = std::make_shared<icg::Tracker>("tracker");
    renderer_geometry_ptr_ = std::make_shared<icg::RendererGeometry>("renderer geometry");

    // Set up cameras
    color_camera_ptr_ = std::make_shared<icg::ZEDColorCamera>("zed_color");
    depth_camera_ptr_ = std::make_shared<icg::ZEDDepthCamera>("zed_depth");
    //color_camera_ptr_ = std::make_shared<icg::TestColorCamera>("test_color");
    //depth_camera_ptr_ = std::make_shared<icg::TestDepthCamera>("test_depth");

    // Set up viewers
    if (kUseColorViewer) {
        color_viewer_ptr_ = std::make_shared<icg::NormalColorViewer>("color_viewer", color_camera_ptr_, renderer_geometry_ptr_);
        if (kSaveImages) color_viewer_ptr_->StartSavingImages(save_directory, "png");
        tracker_ptr_->AddViewer(color_viewer_ptr_);
    }
    if (kUseDepthViewer) {
        depth_viewer_ptr_ = std::make_shared<icg::NormalDepthViewer>("depth_viewer", depth_camera_ptr_, renderer_geometry_ptr_, 0.3f, 1.0f);
        if (kSaveImages) depth_viewer_ptr_->StartSavingImages(save_directory, "png");
        tracker_ptr_->AddViewer(depth_viewer_ptr_);
    }

    // Set up depth renderer
    color_depth_renderer_ptr_ = std::make_shared<icg::FocusedBasicDepthRenderer>("color_depth_renderer", renderer_geometry_ptr_, color_camera_ptr_);
    depth_depth_renderer_ptr_ = std::make_shared<icg::FocusedBasicDepthRenderer>("depth_depth_renderer", renderer_geometry_ptr_, depth_camera_ptr_);

    if (!tracker_ptr_->SetUp()) {
        ROS_ERROR("Failed to set up tracker.");
        return;
    }
    if (!tracker_ptr_->RunTrackerProcess(true, false)) {
        ROS_ERROR("Failed to run tracker process.");
        return;
    }
}

bool PoseEstimationNode::handleToggleTracker(std_srvs::SetBool::Request& req,
                                             std_srvs::SetBool::Response& res) {
    if (req.data == true) {
        // optional start //
        tracker_ptr_->ExecuteDetection();
        // optional end //
        tracker_ptr_->StartTracking();
    }
    else {
        tracker_ptr_->StopTracking();
    }
    res.success = true;
    return true;
}

bool PoseEstimationNode::handlePrepareTracker(irobman_project::SetPoints::Request& req,
                                              irobman_project::SetPoints::Response& res) {
    
    tracker_ptr_->StopTracking();
    
    // Remove all bodies from the tracker (and all objs that have something to do with it)
    tracker_ptr_->ClearDetectors();
    tracker_ptr_->ClearPublishers();
    tracker_ptr_->ClearOptimizers();
    renderer_geometry_ptr_->ClearBodies();
    color_depth_renderer_ptr_->ClearReferencedBodies();
    depth_depth_renderer_ptr_->ClearReferencedBodies();
    body_ptrs_.clear();

    //std::vector<std::shared_ptr<icg::Body>> body_ptrs;  // needs to be a member, otherwise segmentation fault
    int num_points = req.points.size();

    // Create a new body (and dependent objects) for every detected cube
    for (int i = 0; i < num_points; ++i) {
        const std::string body_name = "cube_" + std::to_string(i);

        const auto& point = req.points[i];

        // Set up body
        std::filesystem::path geometry_path{directory_ / "cube.obj"};
        Transform3fA geometry2body_pose;
        geometry2body_pose.matrix() << 1., 0, 0, 0,
                                       0, 1., 0, 0,
                                       0, 0, 1., -0.006,
                                       0, 0, 0, 1.;
        auto body_ptr{std::make_shared<icg::Body>(
            body_name, geometry_path, 1.0f, true, true, geometry2body_pose)};
        
        renderer_geometry_ptr_->AddBody(body_ptr);
        color_depth_renderer_ptr_->AddReferencedBody(body_ptr);
        depth_depth_renderer_ptr_->AddReferencedBody(body_ptr);
        body_ptrs_.push_back(body_ptr);

        // Set up detector
        Transform3fA body2world_pose;
        body2world_pose.matrix() << 1., 0, 0, point.x,
                                    0, 1., 0, point.y,
                                    0, 0, 1., point.z,
                                    0, 0, 0, 1.;                               
        auto detector_ptr{std::make_shared<icg::StaticDetector>(
            body_name + "_detector", body_ptr, body2world_pose)};
        tracker_ptr_->AddDetector(detector_ptr);

        // Set up models
        auto region_model_ptr{std::make_shared<icg::RegionModel>(
            body_name + "_region_model", body_ptr,
            directory_ / (body_name + "_region_model.bin"))};
        auto depth_model_ptr{std::make_shared<icg::DepthModel>(
            body_name + "_depth_model", body_ptr,
            directory_ / (body_name + "_depth_model.bin"))};

        // Set up modalities
        auto region_modality_ptr{std::make_shared<icg::RegionModality>(
            body_name + "_region_modality", body_ptr, color_camera_ptr_,
            region_model_ptr)};
        auto depth_modality_ptr{std::make_shared<icg::DepthModality>(
            body_name + "_depth_modality", body_ptr, depth_camera_ptr_,
            depth_model_ptr)};
        if (kVisualizePoseResult_) {
            region_modality_ptr->set_visualize_pose_result(true);
        }
        if (kMeasureOcclusions_) {
            region_modality_ptr->MeasureOcclusions(depth_camera_ptr_);
            depth_modality_ptr->MeasureOcclusions();
        }
        if (kModelOcclusions_) {
            region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr_);
            depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr_);
        }

        // Set up optimizer
        auto body1_optimizer_ptr{
            std::make_shared<icg::Optimizer>(body_name + "_optimizer")};
        body1_optimizer_ptr->AddModality(region_modality_ptr);
        body1_optimizer_ptr->AddModality(depth_modality_ptr);
        tracker_ptr_->AddOptimizer(body1_optimizer_ptr);
    }

    // Add publisher
    auto publisher_ptr{std::make_shared<icg::ROSPublisher>("ros_publisher", poses_pub_, body_ptrs_)};
    tracker_ptr_->AddPublisher(publisher_ptr);

    // Set up tracker
    if (!tracker_ptr_->SetUp()) {
        res.success = false;
        res.message = "Error preparing the tracker.";
        return false;
    } 
        
    res.success = true;
    return true;
}

bool PoseEstimationNode::handleRetrievePoses(irobman_project::GetPoses::Request& req,
                                             irobman_project::GetPoses::Response& res) {
    const auto& bodies = tracker_ptr_->body_ptrs();  // or directly body_ptrs_ ?

    // Iterate over all bodies and access their pose
    for (const auto& body_ptr : bodies) {
        const Transform3fA &transform = body_ptr->body2world_pose();
        geometry_msgs::Pose pose = transformToPose(transform);
        res.poses.push_back(pose);
    }
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_estimation");
    PoseEstimationNode pose_estimation;
    ros::spin();
    return 0;
}