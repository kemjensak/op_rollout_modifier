#include <op_rollout_modifier.hpp>
namespace op_rollout_modifier
{
    RolloutModifierClass::RolloutModifierClass()
    :_nh("~"), x_min_(0.0), x_max_(200.0), y_min_(-1.0), y_max_(1.0), z_min_(-2.0), z_max_(1.0), aes_flag_(false)
    {
        emr_rollout_num_.data = 1;
        emr_gain_ = 0.6;
        _nh.param<std::string>("radar_topic", radar_topic_, "/carla/ego_vehicle/radar_front");
        _nh.param<float>("thres_radius", thres_radius_, 2);
        
        PlannerHNS::ROSHelpers::getTransformFromTF("ego_vehicle/radar_front", "ego_vehicle/lidar", tf_listener, radar2lidar_transform_);
        radar2lidar_pose_.position.x = radar2lidar_transform_.getOrigin().x();
        radar2lidar_pose_.position.y = radar2lidar_transform_.getOrigin().y();
        radar2lidar_pose_.position.z = radar2lidar_transform_.getOrigin().z();

        pub_rollouts_number = nh.advertise<std_msgs::Int32>("op_update_rollouts_number", 1);
        // pub_RadarPointRviz = nh.advertise<visualization_msgs::MarkerArray>("radar_point_rviz", 1);
        pub_FilteredPolygonsRviz = nh.advertise<visualization_msgs::MarkerArray>("filtered_polygons", 1);

        sub_LocalPlannerPaths = nh.subscribe("/local_weighted_trajectories", 1, &RolloutModifierClass::callbackGetLocalPlannerPath, this);
        sub_RadarData = nh.subscribe(radar_topic_, 1, &RolloutModifierClass::callbackGetRadarData, this);
        sub_DetectedObjectsArray = nh.subscribe("/detection/contour_tracker/objects", 1, &RolloutModifierClass::callbackGetDetectedObjectsArray, this);
        

        // sub_radar_msg_ = nh_.subscribe(radar_topic_, 100)
        // m_nDummyObjPerRep = 150;
        // m_nDetectedObjRepresentations = 5;
        // m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
        // m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
        // m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
        // m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
        // m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
        // m_DetectedPolygonsActual = m_DetectedPolygonsDummy;
        // PlannerHNS::ROSHelpers::InitMarkers(m_nDummyObjPerRep, m_DetectedPolygonsDummy.at(0), m_DetectedPolygonsDummy.at(1), m_DetectedPolygonsDummy.at(2), m_DetectedPolygonsDummy.at(3), m_DetectedPolygonsDummy.at(4));

        // m_MatchingInfoDummy.push_back(visualization_msgs::MarkerArray());
        // m_MatchingInfoActual = m_MatchingInfoDummy;
        // PlannerHNS::ROSHelpers::InitMatchingMarkers(m_nDummyObjPerRep, m_MatchingInfoDummy.at(0));
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
            // std::cout << msg->lanes.size();
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
        // ROS_INFO("received objects number: %d", msg->objects.size());
        // ROS_INFO("%f", radar2lidar_pose_.position.z);
        lidar_objects_filtered_.clear();

        for (unsigned int i = 0; i < msg->objects.size(); i ++)
        {
            if(msg->objects.at(i).pose.position.x > x_min_ && msg->objects.at(i).pose.position.x < x_max_ 
            && msg->objects.at(i).pose.position.y > y_min_ && msg->objects.at(i).pose.position.y < y_max_ 
            && msg->objects.at(i).pose.position.z > z_min_ && msg->objects.at(i).pose.position.z < z_max_)
            {
                lidar_objects_filtered_.push_back(msg->objects.at(i));
            }
        }
    }

    void RolloutModifierClass::getObjectdataFromRadarPoints(const vector<autoware_msgs::DetectedObject>& lidar_objects_filtered, const vector<RadarXYZ>& radar_points_transformed)
    {
        std::vector<float> object_velocity_from_radarpoints, object_range_from_radarpoints;

        for (unsigned int obj_num = 0; obj_num < lidar_objects_filtered.size(); obj_num++)
        {
            object_velocity_from_radarpoints.clear();
            object_range_from_radarpoints.clear();
            sum_radar_velocity_ = mean_velocity_ = min_range_ = 0.0;
            for (unsigned int pt_num = 0; pt_num < radar_points_transformed.size(); pt_num++)
            {
                if (thres_radius_ < hypot(lidar_objects_filtered.at(obj_num).x - radar_points_transformed.at(pt_num).x,
                                          lidar_objects_filtered.at(obj_num).y - radar_points_transformed.at(pt_num).y))
                    {
                        object_range_from_radarpoints.push_back(radar_points_transformed.at(pt_num).Range);
                        object_velocity_from_radarpoints.push_back(radar_points_transformed.at(pt_num).Velocity);
                    }
                      
            }

            if (object_velocity_from_radarpoints.size() > 1)
            {
                sum_radar_velocity_ = std::accumulate(object_velocity_from_radarpoints.begin(), object_velocity_from_radarpoints.end(), 0.0);
                mean_velocity_ = sum_radar_velocity_ / object_velocity_from_radarpoints.size();
                min_range_ = *min_element(object_range_from_radarpoints.begin(), object_range_from_radarpoints.end());
            
                ROS_INFO("min: %f",min_range_);
                
                if (mean_velocity_<0) emergencyDetector(min_range_, -mean_velocity_);

                
            }
        }
    }

    void RolloutModifierClass::emergencyDetector(const float& range, const float& velocity)
    {
        if ( emr_gain_ * (pow(velocity, 2) * 0.0648 + (0.72 + velocity)) > range  && aes_flag_ == false)
        {
            pub_rollouts_number.publish(emr_rollout_num_);
            ROS_WARN("AES activate INFO /n mean: %f min: %f",  mean_velocity_, last_min_range_-radar2lidar_pose_.position.x);

            // ROS_INFO("activated");
            aes_flag_ = true;
        }
        last_min_range_ = min_range_;

    }

    void RolloutModifierClass::mainLoop()
    {
        getObjectdataFromRadarPoints(lidar_objects_filtered_, radar_points_transformed_);
        
    }
    

    // void RolloutModifierClass::VisualizeLocalTracking()
    // {
    //     PlannerHNS::ROSHelpers::ConvertTrackedObjectsMarkers(m_CurrentPos, lidar_objects_filtered_,
    //                 m_DetectedPolygonsDummy.at(0), //centers_d
    //                 m_DetectedPolygonsDummy.at(1), //dirs_d
    //                 m_DetectedPolygonsDummy.at(2), //text_info_d
    //                 m_DetectedPolygonsDummy.at(3), //polygons_d
    //                 m_DetectedPolygonsDummy.at(4), //tracked_traj_d
    //                 m_DetectedPolygonsActual.at(0), //centers
    //                 m_DetectedPolygonsActual.at(1), //dirs
    //                 m_DetectedPolygonsActual.at(2), //text_info
    //                 m_DetectedPolygonsActual.at(3), //polygons
    //                 m_DetectedPolygonsActual.at(4)); //tracked_traj

    //             visualization_msgs::MarkerArray &centers, visualization_msgs::MarkerArray &dirs, visualization_msgs::MarkerArray &text_info, visualization_msgs::MarkerArray &polygons, visualization_msgs::MarkerArray &tracked_traj)

    //     m_DetectedPolygonsAllMarkers.markers.clear();
    //     m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(0).markers.begin(), m_DetectedPolygonsActual.at(0).markers.end());
    //     m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(1).markers.begin(), m_DetectedPolygonsActual.at(1).markers.end());
    //     m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(2).markers.begin(), m_DetectedPolygonsActual.at(2).markers.end());
    //     m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(3).markers.begin(), m_DetectedPolygonsActual.at(3).markers.end());
    //     m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(4).markers.begin(), m_DetectedPolygonsActual.at(4).markers.end());

    //     pub_DetectedPolygonsRviz.publish(m_DetectedPolygonsAllMarkers);
    // }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "op_rollout_modifier");

    op_rollout_modifier::RolloutModifierClass OPM;

    ros::Rate loop_rate(50);

    while (ros::ok())
	{
        
        
        OPM.mainLoop();
		ros::spinOnce();
        loop_rate.sleep();
    }
}       
