#pragma once

#include "Planner.hpp"

namespace dmce {

	/**
	 * This action planner simply returns the closest frontier cell.
	 */
	class FrontierPlanner : public Planner {
		plan_t latestPlan_;
    plan_t closestFrontiers_;

	public:
		using Planner::Planner;

	protected:
		std::pair<bool, plan_t> getLatestPlan_() override {
			auto plan = latestPlan_;
			latestPlan_.clear();
			return std::make_pair((plan.size() > 0), plan);
		}
	
    std::pair<bool, plan_t> getPlanToShare_() override {
      plan_t plan;
      plan.push_back(posToPose(getPosition())); // add current position
			auto latestPlan = latestPlan_;
      //auto latestPlan = closestFrontiers_;
      for (auto pose : latestPlan)
      {
        plan.push_back(pose);
      }
			return std::make_pair((plan.size() > 0), plan);
    }

		void updatePlan_() override {
			auto map = getMap();
			auto res = map.getResolution();
			auto frontier = map.getFrontier();
			pos_t robotPos = getPosition();
			plan_t plan;
      std::vector<pos_t> closestFrontiersPos;
      plan_t closestFrontiers;

			if (frontier.size() > 0)
      {
				pos_t candidatePos, closestFrontier;
				map.getPosition(frontier[0], closestFrontier);
        closestFrontiersPos.push_back(closestFrontier);
				double minDist = (closestFrontier - robotPos).squaredNorm();
				for (unsigned i = 1; i < frontier.size(); i++)
        {
					map.getPosition(frontier[i], candidatePos);
          closestFrontiersPos.push_back(candidatePos);
					double dist = (candidatePos - robotPos).squaredNorm();
					if (dist < minDist)
          {
						minDist = dist;
						closestFrontier = candidatePos;
					}
				}
				plan.push_back(posToPose(closestFrontier));
        std::sort(closestFrontiersPos.begin(), closestFrontiersPos.end(),
            [closestFrontier](pos_t p1, pos_t p2)
            {return (p1 - closestFrontier).squaredNorm() < (p2 - closestFrontier).squaredNorm();});
			}

			latestPlan_ = plan;
      for (pos_t pos : closestFrontiersPos)
      {
        closestFrontiers.push_back(posToPose(pos));
      }
      closestFrontiers_ = closestFrontiers;
		}
	};
}
