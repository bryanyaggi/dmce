#pragma once

#include <deque>

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
    }

	protected:
		std::pair<bool, plan_t> getLatestPlan_() override
    {
      // remove next goal (achieved or nav failure)
      if (!latestPlan_.empty() && (pursuingState_ != PursuingState::Searching))
      {
        latestPlan_.pop_front();
      }
      // clear nav failure
      if (navFailure_)
      {
        navFailure_ = false;
      }
			plan_t plan = plan_t(latestPlan_.begin(), latestPlan_.end()); // convert to vector
			return std::make_pair((plan.size() > 0), plan);
    }
		
    void updatePlan_() override
    {
      static unsigned int robotIdToFind = 0;
      static bool stopRequested = false;
      static ros::Time time;
      static ros::Time lastConnect;

      switch (state_)
      {
        case State::Stationary:
          // Check if connection lost
          lastConnect = ros::Time::now();
          for (unsigned int i = 1; i <= nRobots_; i++)
          {
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
            latestPlan_ = pland_t(plan.begin(), plan.end());
            std::cout << nRobots_ << " " << robotIdToFind << std::endl;
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
              robotIdToFind = 0;
              state_ = State::Stationary;
              pursuingState_ = PursuingState::Following;
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
          // Run sub state machine
          else
          {
            switch (pursuingState_)
            {
              case PursuingState::Following:
                // Check if plan complete or navigation failure
                if (latestPlan_.empty())
                {
                  pursuingState_ = PursuingState::Searching;
                }
                break;
              case PursuingState::Searching:
                //std::cout << "Searching" << std::endl;
                // Random
                auto map = getMap();
                Eigen::Vector2d mapSize = map.getLength();
                Eigen::Vector2d mapPos = map.getPosition();
                Eigen::Vector2d rand = Eigen::Vector2d::Random();
                pos_t target = mapPos - 0.5 * mapSize.cwiseProduct(rand);
                plan_t plan;
                plan.push_back(posToPose(target));
                latestPlan_ = pland_t(plan.begin(), plan.end());
                break;
            }
          }
      }
      //std::cout << state_  << " " << pursuingState_ << " " << latestPlan_.size() << std::endl;
		}

    void peerPlanCallback(const dmce_msgs::RobotPlan& msg)
    {
      plans_[msg.robotId] = msg.path;
    }

    void signalNavigationFailure()
    {
      navFailure_ = true;
    }

  private:
    enum State {Stationary, Moving};
    enum PursuingState {Following, Searching};
		
    using pland_t = std::deque<pose_t>;

    //plan_t latestPlan_;
    pland_t latestPlan_;
    ros::Subscriber connectivitySubscriber_;
    ros::Publisher stopPublisher_;
    ros::ServiceClient stopClient_;
    unsigned int nRobots_;
    bool connectivity_[5];
    ros::Time lastConnect_[5];
    nav_msgs::Path plans_[5];
    State state_ = State::Stationary;
    PursuingState pursuingState_ = PursuingState::Following;
    bool navFailure_ = false;
    int failCounter_ = 0;

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
