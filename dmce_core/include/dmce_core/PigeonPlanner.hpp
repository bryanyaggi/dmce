#pragma once

#include "Planner.hpp"
#include "dmce_core/utils.hpp"
#include "dmce_msgs/RobotConnectivity.h"

#include "std_srvs/Empty.h"

namespace dmce {

	/**
	 * This action planner simply returns a random point on the map.
	 */
	class PigeonPlanner : public Planner {
	public:
		//using Planner::Planner;
    PigeonPlanner(double robotDiameter)
      : Planner(robotDiameter)
    {
      nRobots_ = utils::getRobotCount();

      ros::NodeHandle nh;
      connectivitySubscriber_ = nh.subscribe("/groundStation/RobotConnectivity", 5, &PigeonPlanner::connectivityCallback_, this);
      stopClient_ = nh.serviceClient<std_srvs::Empty>("stop");

      if (!stopClient_.waitForExistence(ros::Duration(5)))
      {
        throw std::runtime_error(
            "[PigeonPlanner] Stop service not found!");
      }

			plan_t plan;
      Eigen::Vector2d target(8, 5);
			plan.push_back(posToPose(target));
			latestPlan_ = plan;
      switch_ = false;
    }

	protected:
		std::pair<bool, plan_t> getLatestPlan_() override
    {
			auto plan = latestPlan_;
			latestPlan_.clear();
			return std::make_pair((plan.size() > 0), plan);
			//return std::make_pair(false, plan_t{});
    }
		
    void updatePlan_() override
    {
      static unsigned int robotIdToFind = 0;
      static bool serviceFailed = false;

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
              plan.push_back(pose);
            }
            latestPlan_ = plan;
            state_ = State::Moving;
          }
          break;
        case State::Moving:
          // Check if connection restored
          if (serviceFailed || connectivity_[robotIdToFind])
          {
            latestPlan_.clear();
            std_srvs::Empty srv;
            if (stopClient_.call(srv))
            {
              serviceFailed = false;
              state_ = State::Stationary;
            }
            else
            {
              serviceFailed = true;
            }
            //state_ = State::Stationary;
          }
          break;
      }
      std::cout << state_ << std::endl;
      /*
			auto map = getMap();
			Eigen::Vector2d mapSize = map.getLength();
			Eigen::Vector2d mapPos = map.getPosition();
			Eigen::Vector2d rand = Eigen::Vector2d::Random();
			pos_t target = mapPos - 0.5*mapSize.cwiseProduct(rand);
			plan_t plan;
			plan.push_back(posToPose(target));
			latestPlan_ = plan;
      */
		}

    void peerPlanCallback(const dmce_msgs::RobotPlan& msg)
    {
      //size_t i = msg.robotId;
      plans_[msg.robotId] = msg.path;
    }

  private:
    enum State {Stationary, Moving};
		
    plan_t latestPlan_;
    ros::Subscriber connectivitySubscriber_;
    ros::ServiceClient stopClient_;
    unsigned int nRobots_;
    bool connectivity_[5];
    ros::Time lastConnect_[5];
    nav_msgs::Path plans_[5];
    bool switch_;
    State state_ = State::Stationary;

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
      //std::cout << lastConnect_[1] << lastConnect_[2] << std::endl;
    }
	};
}
