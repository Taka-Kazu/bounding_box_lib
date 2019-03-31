#include <ros/ros.h>

#include "bounding_box_lib/bounding_box.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_bb");

    ros::NodeHandle nh;

    ros::Publisher bb_pub = nh.advertise<visualization_msgs::Marker>("/bounding_box", 1);

    bounding_box_lib::BoundingBox bb;
    bb.set_rgb(255, 0, 255);
    bb.set_id(0);
    bb.set_scale(0.3, 0.5, 1.7);
    bb.set_orientation(0, 0, M_PI/4.0);
    bb.set_centroid(3, 0, 1.7 * 0.5);
    bb.set_frame_id("/map");
    bb.calculate_vertices();
    visualization_msgs::Marker marker = bb.get_bounding_box();

    ros::Rate loop_rate(10);

    while(ros::ok()){
        bb_pub.publish(marker);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
