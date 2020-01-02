#include <iostream>
#include <map>

#include "dstar_lite.h"
#include "baseline_astar.h"
#include "utility.h"

// Construct node during searching is successful, but I'm not sure if the g_cost and rhs is correct, need some thinking
namespace sub_routine_dstar_lite {
	void DStarLite::UnitTest() {
		bool res_flag = MakePlan();
		if (res_flag) {
			std::cout << "Reached the goal successfully" << std::endl;
		}
		else {
			std::cout << "Can't find a valid route" << std::endl;
		}
	}
}

namespace astar_planner {
	void AStarPlanner::UnitTest() {
		bool res_flag = MakePlan();
	}
}

// TODO: Providing initial values to the A* planner; 
//       Debug for the A* planner;
//       Compare the results with the D* lite planner.

int main(int argc, wchar_t** argv) {
	sub_routine_dstar_lite::Node2D* start_node = new sub_routine_dstar_lite::Node2D;
	sub_routine_dstar_lite::Node2D* goal_node = new sub_routine_dstar_lite::Node2D;
	start_node->x = 1;
	start_node->y = 1;
	goal_node->x = 8;
	goal_node->y = 1;
	goal_node->h_cost = sub_routine_dstar_lite::CalculateHeuristics(goal_node, start_node);

	std::vector<std::vector<unsigned char>> map_info;
	for (int i = 0; i < 10; i++) {
		std::vector<unsigned char> row;
		for (int j = 0; j < 5; j++) {
			row.push_back(0);
		}
		map_info.push_back(row);
	}

	// obstacles
	// for (int i = 0; i < 20; ++i) {
	// 	map_info[12][i] = 1;
	// 	map_info[14][29 - i] = 1;
	// }
	// for (int i = 14; i < 30; ++i) {
	// 	map_info[i][15] = 1;
	// }
	//map_info[1][1] = 0;
	sub_routine_dstar_lite::DStarLite dsl(start_node, goal_node, &map_info);
	dsl.UnitTest();
	
}