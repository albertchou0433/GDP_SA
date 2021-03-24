#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/thread/thread.hpp>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <string>
#include <iostream>

#include <tf/transform_listener.h>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <math.h>

#include <fstream>

using namespace message_filters;
using namespace std;

#define class_num 1
#define PI 3.1415926
/*
    string const nomFichier("score.txt");
    ofstream monFlux(nomFichier.c_str(), ios::app);
*/
    void imageCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detect_msg, const sensor_msgs::PointCloud2::ConstPtr& pCloud)
    {
      geometry_msgs::PointStamped camera_pt;
      geometry_msgs::PointStamped inertial_pt;
      // get width and height of 2D point cloud data
      int width = pCloud->width;
      int height = pCloud->height;

      int u[class_num] = {};
      int v[class_num] = {};
 
      int num = detect_msg->bounding_boxes.size();
 
      for (int i = 0; i < num; i++)
      {
        u[i] = ((detect_msg->bounding_boxes[i].xmax - detect_msg->bounding_boxes[i].xmin)/2) + detect_msg->bounding_boxes[i].xmin; 
        v[i] = ((detect_msg->bounding_boxes[i].ymax - detect_msg->bounding_boxes[i].ymin)*2/7) + detect_msg->bounding_boxes[i].ymin;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = (v[i])*pCloud->row_step + u[i]*pCloud->point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + pCloud->fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + pCloud->fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + pCloud->fields[2].offset; // Z has an offset of 8

      float camera_x = 0.0;
      float camera_y = 0.0;
      float camera_z = 0.0;

      memcpy(&camera_x, &pCloud->data[arrayPosX], sizeof(float));
      memcpy(&camera_y, &pCloud->data[arrayPosY], sizeof(float));
      memcpy(&camera_z, &pCloud->data[arrayPosZ], sizeof(float));

      camera_pt.header = pCloud->header;
      camera_pt.point.x =  camera_z;
      camera_pt.point.y = -camera_x;
      camera_pt.point.z = -camera_y;

      tf2_ros::Buffer tf_buffer;
      tf2_ros::TransformListener tf2_listener(tf_buffer);
      geometry_msgs::TransformStamped transform;

      transform = tf_buffer.lookupTransform("map", "camera_link", ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(camera_pt, inertial_pt, transform);

      string F = inertial_pt.header.frame_id;

      cout << F <<"\n";

    ROS_INFO("pose_x: %f", inertial_pt.point.x);
    ROS_INFO("pose_y: %f", inertial_pt.point.y);
    ROS_INFO("pose_z: %f", inertial_pt.point.z);
   
/*
    if(monFlux)    
    {
        //darknet_ros_msgs::BoundingBox boundingBox;
        //sensor_msgs::PointCloud2 pCould
        float depth = sqrt( camera_z*camera_z + camera_x*camera_x + camera_y*camera_y);

        monFlux << "P " <<  detect_msg->bounding_boxes[i].probability << endl;

        monFlux << "D " << depth << endl;

    }
    else
    {
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }
      }
*/

    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagegrab");
    ros::NodeHandle n;
    ros::Rate rate(10.0);
    printf("READY to get image\n");

    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> object_sub(n, "/darknet_ros/bounding_boxes", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/camera/depth/points", 1); 


    typedef sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), object_sub, depth_sub); 

    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    ros::spin();
    return 0;
}
