#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <autoware_msgs/LaneArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <boost/foreach.hpp>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ControlCommand.h>
#include <autoware_msgs/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/DecisionMaker.h"
#include "op_utility/DataRW.h"
#include "op_ros_helpers/ROSMapHandler.h"
#include "op_ros_helpers/ROSVelocityHandler.h"
#include "op_ros_helpers/op_ParamsHandler.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/KmlMapLoader.h"
#include "op_planner/OpenDriveMapLoader.h"
#include "op_planner/Lanelet2MapLoader.h"
#include "op_planner/VectorMapLoader.h"

#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
struct RadarXYZ
{
    PCL_ADD_POINT4D;
    float Range, Velocity, AzimuthAngle, ElevationAngle;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (RadarXYZ,           
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, Range, Range)
                                    (float, Velocity, Velocity)
                                    (float, AzimuthAngle, AzimuthAngle)
                                    (float, ElevationAngle, ElevationAngle)

)


typedef pcl::PointCloud<RadarXYZ> PointCloudRadar;

namespace op_rollout_modifier
{
    class RolloutModifierClass
    {
        public:
            RolloutModifierClass();
            ~RolloutModifierClass();            

            void run();
            void GetObjectSpeedFromRadarPoints();
        
        protected:
            ros::NodeHandle nh;
            ros::NodeHandle _nh;
            
            std::string radar_topic_;
            std::vector<RadarXYZ> radar_points_transformed_, final_radar_points_;
            std::vector<autoware_msgs::DetectedObject> lidar_objects_filtered_;
            float x_min_, x_max_, y_min_, y_max_, z_min_, z_max_, thres_radius_;

            tf::StampedTransform radar2lidar_transform_;
	        tf::TransformListener tf_listener;

            geometry_msgs::Pose radar2lidar_pose_;

            void callbackGetRadarData(const PointCloudRadar::ConstPtr &msg);
            void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
            void callbackGetDetectedObjectsArray(const autoware_msgs::DetectedObjectArrayConstPtr& msg);

            ros::Publisher pub_rollouts_number;
            ros::Publisher pub_RadarPointRviz;

            ros::Subscriber sub_LocalPlannerPaths;
            ros::Subscriber sub_RadarData;
            ros::Subscriber sub_DetectedObjectsArray;
            // ros::Subscriber
    };

   

}