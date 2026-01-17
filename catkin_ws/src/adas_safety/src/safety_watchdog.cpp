/**
 * @file safety_watchdog.cpp
 * @brief Deterministic Safety Watchdog (C++ for real-time performance)
 * 确定性安全监控节点 (Phase 3 - 实车部署)
 * 
 * This C++ version provides deterministic timing for real vehicle deployment.
 * Use this instead of Python safety_monitor.py for ASIL-D compliance.
 */

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/AccelStamped.h>
#include <mutex>
#include <atomic>

class SafetyWatchdog
{
public:
    SafetyWatchdog() : nh_("~"), primary_alive_(true), override_active_(false)
    {
        // Parameters
        nh_.param("watchdog_timeout_ms", watchdog_timeout_ms_, 50.0);
        nh_.param("safe_deceleration", safe_deceleration_, 5.0);
        
        // Publishers
        safe_cmd_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/adas/safe_command", 1);
        
        // Subscribers
        heartbeat_sub_ = nh_.subscribe("/adas/heartbeat", 1, &SafetyWatchdog::heartbeatCallback, this);
        command_sub_ = nh_.subscribe("/adas/aeb_command", 1, &SafetyWatchdog::commandCallback, this);
        
        // Watchdog timer - high frequency
        watchdog_timer_ = nh_.createTimer(ros::Duration(0.005), &SafetyWatchdog::watchdogCallback, this);
        
        last_heartbeat_time_ = ros::Time::now();
        
        ROS_INFO("Safety Watchdog (C++) initialized - timeout: %.0fms", watchdog_timeout_ms_);
    }
    
    void heartbeatCallback(const std_msgs::Header::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_heartbeat_time_ = msg->stamp;
        
        if (!primary_alive_)
        {
            ROS_INFO("Safety Watchdog: Primary node recovered");
        }
        primary_alive_ = true;
    }
    
    void commandCallback(const geometry_msgs::AccelStamped::ConstPtr& msg)
    {
        if (override_active_)
        {
            return; // Ignore primary commands during override
        }
        
        // Forward command (could add plausibility check here)
        safe_cmd_pub_.publish(*msg);
    }
    
    void watchdogCallback(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        double age_ms = (ros::Time::now() - last_heartbeat_time_).toSec() * 1000;
        
        if (age_ms > watchdog_timeout_ms_)
        {
            if (primary_alive_)
            {
                ROS_WARN("Safety Watchdog: TIMEOUT (%.0fms) - activating override", age_ms);
                primary_alive_ = false;
                override_active_ = true;
            }
            
            // Execute safe state
            executeSafeState();
        }
        else
        {
            if (override_active_ && primary_alive_)
            {
                ROS_INFO("Safety Watchdog: Releasing override");
                override_active_ = false;
            }
        }
    }
    
private:
    void executeSafeState()
    {
        geometry_msgs::AccelStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.accel.linear.x = -safe_deceleration_;
        cmd.accel.linear.y = 0.0;
        cmd.accel.linear.z = 0.0;
        
        safe_cmd_pub_.publish(cmd);
    }
    
    ros::NodeHandle nh_;
    ros::Publisher safe_cmd_pub_;
    ros::Subscriber heartbeat_sub_;
    ros::Subscriber command_sub_;
    ros::Timer watchdog_timer_;
    
    ros::Time last_heartbeat_time_;
    std::mutex mutex_;
    
    std::atomic<bool> primary_alive_;
    std::atomic<bool> override_active_;
    
    double watchdog_timeout_ms_;
    double safe_deceleration_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_watchdog_node");
    SafetyWatchdog watchdog;
    ros::spin();
    return 0;
}
