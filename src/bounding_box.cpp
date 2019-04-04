#include "bounding_box_lib/bounding_box.h"

namespace bounding_box_lib
{
    BoundingBox::BoundingBox(void)
    {
        bb.type = visualization_msgs::Marker::LINE_LIST;
        bb.action = visualization_msgs::Marker::ADD;
        //bb.lifetime = ros::Duration();
        bb.lifetime = ros::Duration(0.1);
        bb.header.frame_id = "/map";
        set_rgb(0, 255, 255);
        bb.color.a = 0.7;
        reset();
    }

    void BoundingBox::set_rgb(int r, int g, int b)
    {
        bb.color.r = r / 255.0;
        bb.color.g = g / 255.0;
        bb.color.b = b / 255.0;
    }

    void BoundingBox::set_scale(double x, double y, double z)
    {
        bb.scale.x = x;
        bb.scale.y = y;
        bb.scale.z = z;
    }

    void BoundingBox::set_orientation(double roll, double pitch, double yaw)
    {
        bb.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    }

    void BoundingBox::set_orientation(double x, double y, double z, double w)
    {
        bb.pose.orientation.x = x;
        bb.pose.orientation.y = y;
        bb.pose.orientation.z = z;
        bb.pose.orientation.w = w;
    }

    void BoundingBox::set_centroid(double x, double y, double z)
    {
        bb.pose.position.x = x;
        bb.pose.position.y = y;
        bb.pose.position.z = z;
    }

    void BoundingBox::set_pose(geometry_msgs::Pose pose)
    {
        bb.pose = pose;
    }
    void BoundingBox::set_frame_id(std::string frame)
    {
        bb.header.frame_id = frame;
    }

    void BoundingBox::set_id(int id)
    {
        bb.id = id;
    }

    void BoundingBox::calculate_vertices(void)
    {
        for(int i=0;i<8;i++){
            vertices[i].x = bb.scale.x * ((i < 4) ? 0.5 : -0.5);
            vertices[i].y = bb.scale.y * ((i % 2 == 0) ? 0.5 : -0.5);
            vertices[i].z = bb.scale.z * ((i % 4 == 0) || (i % 4 == 1) ? 0.5 : -0.5);
        }

        for(auto& vertex : vertices){
             vertex = get_point_msg_from_eigen(get_quaternion_eigen_from_msg(bb.pose.orientation) * get_point_eigen_from_msg(vertex) + get_point_eigen_from_msg(bb.pose.position));
        }

        make_cuboid();
    }

    visualization_msgs::Marker BoundingBox::get_bounding_box(void)
    {
        bb.header.stamp = ros::Time::now();
        reset();
        return bb;
    }

    void BoundingBox::reset(void)
    {
        bb.scale.x = 0.05;// line width
        bb.scale.y = 0;
        bb.scale.z = 0;
        bb.pose.position.x = 0;
        bb.pose.position.y = 0;
        bb.pose.position.z = 0;
        bb.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        for(int i=0;i<8;i++){
            vertices.push_back(geometry_msgs::Point());
        }
    }

    void BoundingBox::make_cuboid(void)
    {
        bb.points.clear();
        bb.points.push_back(vertices[0]);
        bb.points.push_back(vertices[1]);

        bb.points.push_back(vertices[0]);
        bb.points.push_back(vertices[2]);

        bb.points.push_back(vertices[0]);
        bb.points.push_back(vertices[4]);

        bb.points.push_back(vertices[1]);
        bb.points.push_back(vertices[3]);

        bb.points.push_back(vertices[1]);
        bb.points.push_back(vertices[5]);

        bb.points.push_back(vertices[2]);
        bb.points.push_back(vertices[3]);

        bb.points.push_back(vertices[2]);
        bb.points.push_back(vertices[6]);

        bb.points.push_back(vertices[3]);
        bb.points.push_back(vertices[7]);

        bb.points.push_back(vertices[4]);
        bb.points.push_back(vertices[6]);

        bb.points.push_back(vertices[4]);
        bb.points.push_back(vertices[5]);

        bb.points.push_back(vertices[5]);
        bb.points.push_back(vertices[7]);

        bb.points.push_back(vertices[6]);
        bb.points.push_back(vertices[7]);
    }

    Eigen::Quaterniond get_quaternion_eigen_from_msg(geometry_msgs::Quaternion q)
    {
        Eigen::Quaterniond q_e{q.w, q.x, q.y, q.z};
        q_e.normalize();
        return q_e;
    }

    Eigen::Vector3d get_point_eigen_from_msg(geometry_msgs::Point p)
    {
        Eigen::Vector3d v{p.x, p.y, p.z};
        return v;
    }

    geometry_msgs::Point get_point_msg_from_eigen(Eigen::Vector3d v)
    {
        geometry_msgs::Point p;
        p.x = v[0];
        p.y = v[1];
        p.z = v[2];
        return p;
    }
}
