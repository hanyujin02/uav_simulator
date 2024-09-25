//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <gazebo/plugins/RayPlugin.hh>
#include <uav_simulator/livox_lidar/livox_ode_multiray_shape.h>

namespace gazebo {
struct AviaRotateInfo {
    double time;
    double azimuth;
    double zenith;
};

class LivoxPointsPlugin : public RayPlugin {
 public:
    LivoxPointsPlugin();

    virtual ~LivoxPointsPlugin();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

 private:
    ignition::math::Angle AngleMin() const;

    ignition::math::Angle AngleMax() const;

    double GetAngleResolution() const GAZEBO_DEPRECATED(7.0);

    double AngleResolution() const;

    double GetRangeMin() const GAZEBO_DEPRECATED(7.0);

    double RangeMin() const;

    double GetRangeMax() const GAZEBO_DEPRECATED(7.0);

    double RangeMax() const;

    double GetRangeResolution() const GAZEBO_DEPRECATED(7.0);

    double RangeResolution() const;

    int GetRayCount() const GAZEBO_DEPRECATED(7.0);

    int RayCount() const;

    int GetRangeCount() const GAZEBO_DEPRECATED(7.0);

    int RangeCount() const;

    int GetVerticalRayCount() const GAZEBO_DEPRECATED(7.0);

    int VerticalRayCount() const;

    int GetVerticalRangeCount() const GAZEBO_DEPRECATED(7.0);

    int VerticalRangeCount() const;

    ignition::math::Angle VerticalAngleMin() const;

    ignition::math::Angle VerticalAngleMax() const;

    double GetVerticalAngleResolution() const GAZEBO_DEPRECATED(7.0);

    double VerticalAngleResolution() const;

 protected:
    virtual void OnNewLaserScans();

 private:
    void InitializeRays(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                        boost::shared_ptr<physics::LivoxOdeMultiRayShape>& ray_shape);

    void InitializeScan(msgs::LaserScan*& scan);

    boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
    gazebo::physics::CollisionPtr laserCollision;
    physics::EntityPtr parentEntity;
    transport::PublisherPtr scanPub;
    sdf::ElementPtr sdfPtr;
    msgs::LaserScanStamped laserMsg;
    transport::NodePtr node;
    sensors::RaySensorPtr raySensor;
    std::vector<AviaRotateInfo> aviaInfos;

    std::shared_ptr<ros::NodeHandle> rosNode;
    ros::Publisher rosPointPub;
    std::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;

    int64_t samplesStep = 0;
    int64_t currStartIndex = 0;
    int64_t maxPointSize = 1000;
    int64_t downSample = 1;
};

}  // namespace gazebo

#endif  // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H