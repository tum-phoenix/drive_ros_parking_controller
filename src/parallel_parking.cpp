#include <parallel_parking.h>

ParallelParking::ParallelParking(ros::NodeHandle &nh){
    nh_ = nh;
    env_model_sub = nh_.subscribe("environment_model", 1, &ParallelParking::getEnvModel, this);
    parallel_service = nh_.advertiseService("parking_status", &ParallelParking::sendParkingStatus, this);
    //publishing func to publish trajectory meta data
}
ParallelParking::~ParallelParking(){}

void ParallelParking::getEnvModel(const drive_ros_msgs::EnvironmentModel &envModelMsg){
    ROS_INFO("parallel parking recieved environment model");

    obstacles_ = envModelMsg.obstacles;
    ROS_INFO_STREAM("obj_track_distance: " << obstacles_[0].obj_track_distance);

    traffic_marks_ = envModelMsg.traffic_marks;
    ROS_INFO_STREAM("track_distance: " << traffic_marks_[0].track_distance);
}

bool ParallelParking::sendParkingStatus(drive_ros_msgs::ParkingInProgress::Request &req, drive_ros_msgs::ParkingInProgress::Response &res){
    if(req.parking_expected == true){
        ROS_INFO_STREAM("need to begin parking");
        parking_in_progress_ = true;
        res.parking_in_progress = true;
    }
    else{
        parking_in_progress_ = false;
        res.parking_in_progress = false;
    }
    return true;
}

bool ParallelParking::isParking(){
    return parking_in_progress_;
}

bool ParallelParking::isBlockedArea(std::vector<float> blocked_spots, float start, float end){
    for(int i = 0; i<blocked_spots.size(); i++){
        if(start <= blocked_spots[i] && blocked_spots[i] <= end){
            return true;
        }
    }
    return false;
}

void ParallelParking::findPotentialSpots(std::vector<int> right_obst_ids, std::vector<ParallelParking::ParkingSpot> potential_spots, std::vector<float> blocked_spots){
    //find parking spots between obsts in right lane and push them to potential_spots_
    int num_right_obsts_ = right_obst_ids.size();
    for(int i = 0; i <num_right_obsts_; i++){
        float i_track_distance_ = obstacles_[right_obst_ids[i]].obj_track_distance;

        //only consider spot if starts at least MIN_DIST_TO_SPOT away
        if(i_track_distance_ >= MIN_DIST_TO_SPOT){
            float min_front_dist_ = std::numeric_limits<float>::max();
            int id_front_obst_ = -1;
            //for each obst, find closest obst in front
            for(int j = 0; j<num_right_obsts_; j++){
                if(i==j){
                    continue;
                }
                else{
                    float j_track_distance_ = obstacles_[right_obst_ids[j]].obj_track_distance;

                    //if its in front and closer than min_front_dist_, update closest obst
                    if(j_track_distance_ > i_track_distance_ && (j_track_distance_ - i_track_distance_) < min_front_dist_){
                        min_front_dist_ = j_track_distance_ - i_track_distance_;

                        //don't consider spot if too short or in blocked spot area
                        if(min_front_dist_ >= MIN_SPOT_LENGTH || isBlockedArea(blocked_spots, i_track_distance_, j_track_distance_)){
                            id_front_obst_ = -1;
                            break;
                        }
                        //otherwise update id of closest obstacle in front of it
                        else{
                            id_front_obst_ = j;

                        }
                    }
                }
            }

            //if front obst found, store parking spot
            if(id_front_obst_ != -1){
                ParkingSpot newSpot = ParkingSpot(min_front_dist_,
                                                  i_track_distance_ + AVG_OBST_LENGTH,
                                                  obstacles_[right_obst_ids[id_front_obst_]].obj_track_distance);
                potential_spots.push_back(newSpot);
            }
        }
    }
}

bool ParallelParking::findClosestSpot(std::vector<ParkingSpot> potential_spots, ParallelParking::ParkingSpot* choosen_spot){
    int num_spots_ = potential_spots.size();

    //find closest spot to car
    if(num_spots_ != 0){
        float min_dist_to_spot_ = std::numeric_limits<float>::max();
        int id_best_spot_;

        //determine which spot to park in
        for(int i = 0; i< num_spots_; i++){
            if(potential_spots[i].start_ > MIN_DIST_TO_SPOT){
                min_dist_to_spot_ = potential_spots[i].start_;
                id_best_spot_ = i;
            }
        }

        //update vales to reflect best spot that was found
        choosen_spot->length_ = potential_spots[id_best_spot_].length_;
        choosen_spot->start_ = potential_spots[id_best_spot_].start_;
        choosen_spot->end_ = potential_spots[id_best_spot_].end_;

        ROS_INFO_STREAM("found valid parking spot with:");
        ROS_INFO_STREAM("\tlength: " << choosen_spot->length_);
        ROS_INFO_STREAM("\tstart: " << choosen_spot->start_);
        ROS_INFO_STREAM("\tend: " << choosen_spot->end_);

        return true;
    }
    else{
        ROS_INFO_STREAM("no valid parking spot found");
        return false;
    }
}

//returns false if no spot found
bool ParallelParking::findSpot(ParallelParking::ParkingSpot* choosen_spot_){
    //parse traffic marks, note location of blocked spts
    std::vector<float> blocked_spots_;
    int num_marks_ = traffic_marks_.size();
    for(int i = 0; i<num_marks_; i++){
        if(traffic_marks_[i].id == MARKING_PARKING_SPOT_BLOCKED){
            blocked_spots_.push_back(traffic_marks_[i].track_distance);
        }
    }

    int num_obsts_ = obstacles_.size();
    //holds id of obsts in right lane
    std::vector<int> right_obst_ids;
    //parse obsts add obst to vect if in the right line (=3)
    for(int i = 0; i<num_obsts_; i++){
        if(obstacles_[i].obj_lane.obj_lane == 3){
            right_obst_ids.push_back(i);
        }
    }

    std::vector<ParkingSpot> potential_spots_;
    findPotentialSpots(right_obst_ids, potential_spots_, blocked_spots_);
    bool spotFound = findClosestSpot(potential_spots_, choosen_spot_);

    return spotFound;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "parking_controller");
    ros::NodeHandle nh;
    ParallelParking p(nh);
    // ros::spin();

    ros::Rate r(5);
    while(ros::ok()){
        //wait until parking requested
        while(!p.isParking()){
            ros::spinOnce();
            r.sleep();
        }

        //parking logic, exit  loop when parking finished
        
        //state of parking (FIND, GO_IN, SUCCESS, GO_OUT)
        int state = FIND;
        ParallelParking::ParkingSpot current_spot_ = ParallelParking::ParkingSpot();
        while(p.isParking()){
            if(state == FIND){
                if(p.findSpot(&current_spot_) == true){
                    state = GO_IN;
                }
            }
            else if(state == GO_IN){

            }
            else if (state == SUCCESS){

            }
            else if (state == GO_OUT){

            }
            ros::spinOnce();
        }
    }
}