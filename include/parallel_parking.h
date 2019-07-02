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
        float start_;   //distance from car to back of spot
        float end_;  //distance from car to front of spot

        ParkingSpot(){}
        ParkingSpot(float length, float distance_back, float distance_front){
            length_ = length;
            start_ = distance_back;
            end_ = distance_front;
        }
    };

    //parse obstacle and traffic marks to find a parking spot
    bool findSpot(ParkingSpot* choosen_spot_);

    bool isParking();

private:
    //state values
    #define FIND        0
    #define GO_IN       1
    #define SUCCESS     2
    #define GO_OUT      3
    
    #define MIN_DIST_TO_SPOT    20      // min dist from car to satrt of spot to allow time to slow down/reposition
    #define MIN_SPOT_LENGTH     400     // min spot length to allow car to comfortably park
    #define AVG_OBST_LENGTH     30      // avg length of parking obst
    #define BLOCKED_SPOT_LENGTH 100     // lenght of area off-limits do to MARKING_PARKING_SPOT_BLOCKED

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

    bool isBlockedArea(std::vector<float> blocked_spots, float start, float end);
    void findPotentialSpots(std::vector<int> right_obst_ids, std::vector<ParkingSpot> potential_spots, std::vector<float> blocked_spots);
    bool findClosestSpot(std::vector<ParkingSpot> potential_spots, ParkingSpot* choosen_spot);
};

#endif