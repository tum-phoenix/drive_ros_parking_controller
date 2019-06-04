#ifndef PARALLEL_PARKING_H
#define PARALLEL_PARKING_H

#include <ros/ros.h>
#include <drive_ros_msgs/EnvironmentModel.h>
#include <drive_ros_msgs/ObstacleEnvironment.h>
#include <drive_ros_msgs/TrafficMarkEnvironment.h>
#include <drive_ros_msgs/TrajectoryMetaInput.h>
#include <drive_ros_msgs/ParkingInProgress.h>
#include <drive_ros_msgs/Lane.h>
#include <bt_node/value_definitions.h>
#include <vector>
#include <limits>

class ParallelParking {
public:
    ParallelParking(ros::NodeHandle &nh);
    ~ParallelParking();


    struct ParkingSpot{
        float length_;         //length of spot
        float distance_back_;   //distance from car to back of spot
        float distance_front_;  //distance from car to front of spot

        ParkingSpot(){}
        ParkingSpot(float length, float distance_back, float distance_front){
            length_ = length;
            distance_back_ = distance_back;
            distance_front_ = distance_front;
        }
    };

    //parse obstacle and traffic marks to find a parking spot
    bool findSpot(ParkingSpot* choosen_spot_);

    bool isParking();

private:
    //state values
    #define FIND 0
    #define GO_IN 1
    #define SUCCESS 2
    #define GO_OUT 3
    
    #define MIN_DIST_TO_SPOT 20

    ros::NodeHandle nh_;

    ros::Subscriber env_model_sub;
    ros::ServiceServer parallel_service;

    bool parking_in_progress_ = false;

    //callback for env_model_sub
    void getEnvModel(const drive_ros_msgs::EnvironmentModel &envModelMsg);
    //callback for parallel_service
    bool sendParkingStatus(drive_ros_msgs::ParkingInProgress::Request &req, drive_ros_msgs::ParkingInProgress::Response &res);

    //store enviornment model
    std::vector<drive_ros_msgs::ObstacleEnvironment> obstacles_;
    std::vector<drive_ros_msgs::TrafficMarkEnvironment> traffic_marks_;
};

#endif