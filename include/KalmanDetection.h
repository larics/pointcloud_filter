#ifndef KALMAN_DETECTION_H
#define KALMAN_DETECTION_H

#include "KalmanFilter.h"
#include <pointcloud_filter/BrickDistanceParametersConfig.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>

#define NO_DISTANCE_DETECTED -1
#define MAX_INVALID_TIME 2

typedef pointcloud_filter::BrickDistanceParametersConfig bdist_cfg;
class KalmanDetection
{

public:

KalmanDetection(std::string ns):
    _bdistServer {_bdistMutex, ros::NodeHandle("kalman_ns/" + ns + "_config")},
    _kalmanFilter {new KalmanFilter},
    _kalmanInitialized {false},
    _filteredDistance (NO_DISTANCE_DETECTED),
    _filteredDistanceVel (0),
    _timeInvalid (0),
    _ns (ns)
{
}

~KalmanDetection()
{   
}

void initializeParameters(ros::NodeHandle& nh)
{
    ROS_WARN("KalmanDetection::initializeParameters()");
    // Setup dynamic reconfigure server

    double kalmanNoiseMv;
    double kalmanNoisePos;
    double kalmanNoiseVel;
    bool initialized = 
        nh.getParam(_ns + "/kalman/noise_mv", kalmanNoiseMv) &&
        nh.getParam(_ns + "/kalman/noise_pos", kalmanNoisePos) &&
        nh.getParam(_ns + "/kalman/noise_vel", kalmanNoiseVel);

    _kalmanFilter->setMeasureNoise(kalmanNoiseMv);
    _kalmanFilter->setPositionNoise(kalmanNoisePos);
    _kalmanFilter->setVelocityNoise(kalmanNoiseVel);
    ROS_INFO_STREAM(*_kalmanFilter);

    if (!initialized)
    {
        ROS_FATAL("KalmanDetection::initializeParameters() - parameter initialization failed.");
        throw std::invalid_argument("KalmanDetection parameters not properly set.");
    }

    bdist_cfg cfg;
    cfg.noise_mv = kalmanNoiseMv;
    cfg.noise_pos = kalmanNoisePos;
    cfg.noise_vel = kalmanNoiseVel;
    
	_bdistServer.updateConfig(cfg);
	_bdistParamCb = boost::bind(
		&KalmanDetection::parametersCallback, this, _1, _2);
	_bdistServer.setCallback(_bdistParamCb);

    // Initialize publisher
    _filtDistPub = nh.advertise<std_msgs::Float32>("kalman/" + _ns + "/filtered", 1);
    _filtDistVelPub = nh.advertise<std_msgs::Float32>("kalman/" + _ns + "/filtered_vel", 1);
}

void parametersCallback(
    bdist_cfg& configMsg,
    uint32_t level)
{
    ROS_DEBUG("KalmanDetection::parametersCallback");
    _kalmanFilter->setMeasureNoise(configMsg.noise_mv);
    _kalmanFilter->setPositionNoise(configMsg.noise_pos);
    _kalmanFilter->setVelocityNoise(configMsg.noise_vel);
    ROS_INFO_STREAM(*_kalmanFilter);
}

void filterCurrentDistance(double dt, double currDistance, bool newMeasurementFlag)
{   
    // Reset filtered distance if filter is not initialized
    if (!_kalmanInitialized)
    {
        resetState();
    }

    // Check if initialization failed
    if (!_kalmanInitialized && currDistance < 0)
    {
        ROS_WARN("KalmanFilter - Failed to initialize");
        return;
    }

    // Check if initialization should take place
    if (!_kalmanInitialized && currDistance >= 0)
    {
        _kalmanInitialized = true;
        _kalmanFilter->initializePosition(currDistance);
        ROS_WARN("KalmanFilter - Initialized.");
    }

    // Do model update
    _kalmanFilter->modelUpdate(dt);

    // Do measure update if everything is valid
    if (newMeasurementFlag && currDistance > 0)
    {
        // ROS_INFO("KalmanFilter - New measurement! update called");
        _kalmanFilter->measureUpdate(currDistance);
        _timeInvalid = 0;
    }
    else
    {
        // Increase time invalid
        // ROS_WARN("KalmanFilter - doing only model update");
        _timeInvalid += dt;
    }

    // Check if invalid time reached maximum
    if (_timeInvalid > MAX_INVALID_TIME)
    {
        resetState();
        ROS_FATAL("KalmanFilter - Max invalid time reached.");
        return;
    }

    // Get kalman filter position and velocity
    _filteredDistance = _kalmanFilter->getPosition();
    _filteredDistanceVel = _kalmanFilter->getVelocity();
}

void publish()
{
    std_msgs::Float32 distMsg;
    distMsg.data = _filteredDistance;
    _filtDistPub.publish(distMsg);

    std_msgs::Float32 velMsg;
    velMsg.data = _filteredDistanceVel;
    _filtDistVelPub.publish(velMsg);    
}

void resetState()
{
    _kalmanInitialized = false;
    _timeInvalid = 0;
    _filteredDistance = NO_DISTANCE_DETECTED;
    _filteredDistanceVel = 0;
}

double getState()
{
    return _filteredDistance;
}

double getStateVel()
{
    return _filteredDistanceVel;
}

private:

    /** Define Dynamic Reconfigure parameters **/
    boost::recursive_mutex _bdistMutex;
    dynamic_reconfigure::Server<bdist_cfg> _bdistServer;
    dynamic_reconfigure::Server<bdist_cfg>::CallbackType _bdistParamCb;
        
    /** Kalman filter object */
    std::unique_ptr<KalmanFilter> _kalmanFilter;

    ros::Publisher _filtDistPub;
    ros::Publisher _filtDistVelPub;

	/** Flag signaling that kalman filter is initialized. */
	bool _kalmanInitialized;

	/** Filtered distance - Kalman Filter output */
	double _filteredDistance;

	/** Filtered distance velocity - Kalman Filter output */
	double _filteredDistanceVel;

	/** Time passed while measurements are invalid. */
	double _timeInvalid;

    const std::string _ns;
};

#endif /* KALMAN_DETECTION_H */