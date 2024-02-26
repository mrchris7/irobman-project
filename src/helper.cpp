#include <helper.h>


geometry_msgs::Pose transformToPose(const Transform3fA& transform) {

    Eigen::Vector3f translation = transform.translation();
    Eigen::Quaternionf rotation(transform.rotation());

    geometry_msgs::Pose pose;
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();

    return pose;
}


Transform3fA poseToTransform(const geometry_msgs::Pose& pose) {

    Eigen::Vector3f translation(pose.position.x,
                                pose.position.y,
                                pose.position.z);
                                
    Eigen::Quaternionf rotation(pose.orientation.w, 
                                pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z);

    Transform3fA transform;
    transform.translation() = translation;
    transform.linear() = rotation.toRotationMatrix();

    return transform;
}