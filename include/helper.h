#ifndef HELPER_H
#define HELPER_H

#include "geometry_msgs/Pose.h"
#include <Eigen/Geometry>


#include <ros/ros.h>
#include <icg/common.h>

typedef Eigen::Transform<float, 3, Eigen::Affine> Transform3fA;

geometry_msgs::Pose transformToPose(const Transform3fA& transform);

Transform3fA poseToTransform(const geometry_msgs::Pose& pose);

#endif // HELPER_H