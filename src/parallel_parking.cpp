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
    // ROS_INFO_STREAM("obj_track_distance: " << obstacles_[0].obj_track_distance);

    traffic_marks_ = envModelMsg.traffic_marks;
    // ROS_INFO_STREAM("track_distance: " << traffic_marks_[0].track_distance);
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

//returns -1 if no spot found, otherwise id of spot it will park in
bool ParallelParking::findSpot(ParallelParking::ParkingSpot* choosen_spot_){
    //parse traffic marks
    int num_marks_ = traffic_marks_.size();
    for(int i = 0; i<num_marks_; i++){
        if(traffic_marks_[i].id == MARKING_BARRED_AREA_RIGHT){
            return 0;
        }
        else if(traffic_marks_[i].id == MARKING_PARKING_SPOT_BLOCKED){
            //TODO: note track_distance to avoid (how big is blocked spot?)
        }
    }

    int num_obsts_ = obstacles_.size();
    //holds id of obsts in right lane
    std::vector<int> right_obst_ids_;

    //parse obsts
    for(int i = 0; i<num_obsts_; i++){
        //add obst to vecotr if in the right line (=3)
        if(obstacles_[i].obj_lane.obj_lane == 3){
            right_obst_ids_.push_back(i);
        }
    }

    //find parking spots between obsts in right lane and push them to potential_spots_
    int num_right_obsts_ = right_obst_ids_.size();
    std::vector<ParkingSpot> potential_spots_;
    for(int i = 0; i <num_right_obsts_; i++){
        float i_track_distance_ = obstacles_[right_obst_ids_[i]].obj_track_distance;
        float min_front_dist_ = std::numeric_limits<float>::max();
        int id_front_obst_ = -1;
        //for each obst, find closest obst in front
        for(int j = 0; j<num_right_obsts_; j++){
            if(i==j){
                continue;
            }
            else{
                float j_track_distance_ = obstacles_[right_obst_ids_[j]].obj_track_distance;

                //if its in front and closer than min_front_dist_, update closest obst
                if(j_track_distance_ > i_track_distance_ && (j_track_distance_ - i_track_distance_) < min_front_dist_){
                    min_front_dist_ = j_track_distance_ - i_track_distance_;
                    id_front_obst_ = j;
                }
            }
        }
        //if front obst found, store parking spot
        if(id_front_obst_ != -1){
            ParkingSpot newSpot = ParkingSpot(min_front_dist_, i_track_distance_, obstacles_[right_obst_ids_[id_front_obst_]].obj_track_distance);
            potential_spots_.push_back(newSpot);
        }
    }

    int num_spots_ = potential_spots_.size();
    
    if(num_spots_ != 0){
        float min_dist_to_spot_ = std::numeric_limits<float>::max();
        int id_best_spot_;

        //determine which spot to park in
        for(int i = 0; i< num_spots_; i++){
            if(potential_spots_[i].distance_back_ > MIN_DIST_TO_SPOT){
                min_dist_to_spot_ = potential_spots_[i].distance_back_;
                id_best_spot_ = i;
            }
        }

        //update vales to reflect best spot that was found
        choosen_spot_->length_ = potential_spots_[id_best_spot_].length_;
        choosen_spot_->distance_back_ = potential_spots_[id_best_spot_].distance_back_;
        choosen_spot_->distance_front_ = potential_spots_[id_best_spot_].distance_front_;

        return true;
    }
    else{
        return false;
    }
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