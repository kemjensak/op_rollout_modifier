#include <op_rollout_modifier.hpp>
namespace op_rollout_modifier
{
    RolloutModifierClass::RolloutModifierClass()
    :_nh("~"), x_min_(0.0), x_max_(200.0), y_min_(-2.0), y_max_(2.0), z_min_(-2.0), z_max_(1.0)
    {
        _nh.param<std::string>("radar_topic", radar_topic_, "/carla/ego_vehicle/radar_front");
        _nh.param<float>("thres_radius", thres_radius_, 2);

        PlannerHNS::ROSHelpers::getTransformFromTF("ego_vehicle/radar_front", "ego_vehicle/lidar", tf_listener, radar2lidar_transform_);
        radar2lidar_pose_.position.x = radar2lidar_transform_.getOrigin().x();
        radar2lidar_pose_.position.y = radar2lidar_transform_.getOrigin().y();
        radar2lidar_pose_.position.z = radar2lidar_transform_.getOrigin().z();

        pub_rollouts_number = nh.advertise<std_msgs::Int32>("op_update_rollouts_number", 1);
        pub_RadarPointRviz = nh.advertise<visualization_msgs::MarkerArray>("radar_point_rviz", 1);

        sub_LocalPlannerPaths = nh.subscribe("/local_weighted_trajectories", 1, &RolloutModifierClass::callbackGetLocalPlannerPath, this);
        sub_RadarData = nh.subscribe(radar_topic_, 1, &RolloutModifierClass::callbackGetRadarData, this);
        sub_DetectedObjectsArray = nh.subscribe("/detection/contour_tracker/objects", 1, &RolloutModifierClass::callbackGetDetectedObjectsArray, this);
        

        // sub_radar_msg_ = nh_.subscribe(radar_topic_, 100)
    }

    RolloutModifierClass::~RolloutModifierClass()
    {

    }
    // void RolloutModifierClass::setFOV
    
    void RolloutModifierClass::callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
    {
        // std::cout << msg->lanes.size();
        // ROS_INFO("received rollout number: %d", msg->lanes.size());
        if(msg->lanes.size() > 0)
        {
            //m_RollOuts.clear();
            std::cout << msg->lanes.size();
            for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
            {
                
            }

            
        }
    }

    void RolloutModifierClass::callbackGetRadarData(const PointCloudRadar::ConstPtr &msg)
    { 
        // x, y, z, Range, Velocity, AzimuthAngle, ElevationAngle
        // ROS_INFO("Received Radar points number: %d", msg->points.size());
        radar_points_transformed_.clear();
        
        BOOST_FOREACH (const RadarXYZ& point, msg->points)
        {
            RadarXYZ point_transformed = point;
            point_transformed.x += radar2lidar_pose_.position.x;
            point_transformed.y += radar2lidar_pose_.position.y;
            point_transformed.z += radar2lidar_pose_.position.z;

            radar_points_transformed_.push_back(point_transformed);
        }
        
        // std::cout << "x" << msg->x << 
    }

    void RolloutModifierClass::callbackGetDetectedObjectsArray(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
    {
        ROS_INFO("received objects number: %d", msg->objects.size());
        lidar_objects_filtered_.clear();

        for (unsigned int i = 0; i < msg->objects.size(); i ++)
        {
            if(msg->objects[i].pose.position.x > x_min_ && msg->objects[i].pose.position.x < x_max_ 
            && msg->objects[i].pose.position.y > y_min_ && msg->objects[i].pose.position.y < y_max_ 
            && msg->objects[i].pose.position.z > z_min_ && msg->objects[i].pose.position.z < z_max_)
            {
                lidar_objects_filtered_.push_back(msg->objects[i]);
            }
        }
    }

    void RolloutModifierClass::GetObjectSpeedFromRadarPoints()
    {
        std::vector<float> object_velocity_from_radarpoints;
        for (unsigned int obj_num = 0; obj_num < lidar_objects_filtered_.size(); obj_num++)
        {
            for (unsigned int pt_num = 0; pt_num < radar_points_transformed_.size(); pt_num++)
            {
                if (thres_radius_ < hypot(lidar_objects_filtered_[obj_num].x - radar_points_transformed_[pt_num].x, 
                                          lidar_objects_filtered_[obj_num].y - radar_points_transformed_[pt_num].y))
                    {
                        // final_radar_points_.push_back(radar_points_transformed_[pt_num]);
                        object_velocity_from_radarpoints.push_back(radar_points_transformed_[pt_num].Velocity);
                    }
                      
            }
            float sum = std::accumulate(object_velocity_from_radarpoints.begin(), object_velocity_from_radarpoints.end(), 0.0);
            float mean = sum / object_velocity_from_radarpoints.size();
        
            ROS_INFO("obj_num: %d mean: %f", obj_num, mean);
        }
      

    }
    

    // void RolloutModifierClass::vizRadarPoint()
    // {

    // }

  



}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "op_rollout_modifier");

    op_rollout_modifier::RolloutModifierClass OPM;

    ros::Rate loop_rate(50);

    while (ros::ok())
	{
        OPM.GetObjectSpeedFromRadarPoints();
		ros::spinOnce();
        loop_rate.sleep();
    }
}       
