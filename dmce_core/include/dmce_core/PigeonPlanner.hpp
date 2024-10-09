#pragma once

#include "Planner.hpp"
#include "dmce_core/utils.hpp"
#include "dmce_msgs/RobotConnectivity.h"

#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"

namespace dmce {

	class PigeonPlanner : public Planner {
	public:
    PigeonPlanner(double robotDiameter)
      : Planner(robotDiameter)
    {
      nRobots_ = utils::getRobotCount();

      ros::NodeHandle nh;
      connectivitySubscriber_ = nh.subscribe("/groundStation/RobotConnectivity", 5, &PigeonPlanner::connectivityCallback_, this);
      stopPublisher_ = nh.advertise<std_msgs::Empty>("stop", 1);

			/*
      plan_t plan;
      Eigen::Vector2d target(8, 5);
			plan.push_back(posToPose(target));
			latestPlan_ = plan;
      */
    }

	protected:
		std::pair<bool, plan_t> getLatestPlan_() override
    {
      if (!navFailure_ && !latestPlan_.empty())
      {
        latestPlan_.erase(latestPlan_.begin()); // remove first element
        navFailure_ = false;
      }
			auto plan = latestPlan_;
			return std::make_pair((plan.size() > 0), plan);
    }
		
    void updatePlan_() override
    {
      static unsigned int robotIdToFind = 0;
      static bool stopRequested = false;
      static ros::Time time;

      switch (state_)
      {
        case State::Stationary:
          // Check if connection lost
          for (unsigned int i = 1; i <= nRobots_; i++)
          {
            ros::Time lastConnect = ros::Time::now();
            if (!connectivity_[i])
            {
              if (lastConnect_[i] <= lastConnect)
              {
                robotIdToFind = i;
                lastConnect = lastConnect_[i];
              }
            }
          }
          if (robotIdToFind != 0)
          {
            plan_t plan;
            for (pose_t pose : plans_[robotIdToFind].poses)
            {
              std::cout << pose << std::endl;
              plan.push_back(pose);
            }
            latestPlan_ = plan;
            state_ = State::Moving;
          }
          break;
        case State::Moving:
          // Check if stop already requested
          if (stopRequested)
          {
            // Check if sufficient time waited
            if (ros::Time::now() - time >= ros::Duration(1))
            {
              stopRequested = false;
              state_ = State::Stationary;
            }
          }
          // Check if connection restored
          else if (connectivity_[robotIdToFind])
          {
            latestPlan_.clear(); // remove all points
            stopPublisher_.publish(std_msgs::Empty());
            stopRequested = true;
            time = ros::Time::now(); // set time
          }
          break;
      }
      //std::cout << state_ << std::endl;
		}

    void peerPlanCallback(const dmce_msgs::RobotPlan& msg)
    {
      //size_t i = msg.robotId;
      plans_[msg.robotId] = msg.path;
    }

    void signalNavigationFailure()
    {
      navFailure_ = true;
    }

  private:
    enum State {Stationary, Moving};
		
    plan_t latestPlan_;
    ros::Subscriber connectivitySubscriber_;
    ros::Publisher stopPublisher_;
    ros::ServiceClient stopClient_;
    unsigned int nRobots_;
    bool connectivity_[5];
    ros::Time lastConnect_[5];
    nav_msgs::Path plans_[5];
    State state_ = State::Stationary;
    bool navFailure_ = false;

    void connectivityCallback_(const dmce_msgs::RobotConnectivity& msg)
    {
      for (unsigned int i = 0; i <= nRobots_; i++)
      {
        if (msg.data[i] == true && msg.data[i * (nRobots_ + 1)] == true)
        {
          connectivity_[i] = true;
          lastConnect_[i] = ros::Time::now();
        }
        else
        {
          connectivity_[i] = false;
        }
      }
    }
	};
}
