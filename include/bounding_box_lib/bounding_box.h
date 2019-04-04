#ifndef __BOUNDING_BOX_LIB_BOUNDING_BOX_H
#define __BOUNDING_BOX_LIB_BOUNDING_BOX_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace bounding_box_lib
{
    class BoundingBox
    {
    public:
        BoundingBox(void);

        void set_rgb(int, int, int);// r, g, b
        void set_scale(double, double, double);// x, y, z
        void set_orientation(double, double, double);// roll, pitch, yaw
        void set_orientation(double, double, double, double);// x, y, z, w
        void set_centroid(double, double, double);// x, y, z
        void set_pose(geometry_msgs::Pose);
        void set_frame_id(std::string);
        void set_id(int);
        void calculate_vertices(void);
        visualization_msgs::Marker get_bounding_box(void);
        void reset(void);
        void make_cuboid(void);

    private:
        visualization_msgs::Marker bb;
        std::vector<geometry_msgs::Point> vertices;// ftl, ftr, fbl, fbr, btl, btr, bbl, bbr

    };

    Eigen::Quaterniond get_quaternion_eigen_from_msg(geometry_msgs::Quaternion);
    Eigen::Vector3d get_point_eigen_from_msg(geometry_msgs::Point);
    geometry_msgs::Point get_point_msg_from_eigen(Eigen::Vector3d);
}

#endif// __BOUNDING_BOX_LIB_BOUNDING_BOX_H
