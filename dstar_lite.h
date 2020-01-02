#pragma once

#include <vector>
#include <map>

#include "utility.h"

namespace sub_routine_dstar_lite {

	
	class DStarLite {
		// map of ROI, 0 is free, 1 is occupied 
		// BE CAREFUL!! The innner vector is a column and the map should be a rectangle!
		std::vector<std::vector<unsigned char>>* envmap;
		int height_map = 0;
		int width_map = 0;

		// search graph, constructed during operation
		std::vector<std::vector<Node2D*>> search_graph;

		// initial node
		Node2D* s_start;
		// goal node
		Node2D* s_goal;

		// open list, NOTE: Every element in a std::map is unique so the same key could only points to the same value
		// It is assumed all nodes have differenet key values
		std::map<PriorityKey, Node2D*, PKeyCompare> open_list;

		// accumulated heuristic cost
		double km = 0;

		// count how many times the search runs
		int search_counter = 0;

		// is map changed?
		bool map_changed = false;

		// which nodes are changed?
		std::vector<std::vector<int>> changed_id;
		std::vector<std::vector<double>> changed_xy;
		std::vector<Node2D*> changed_node;

		inline bool OutOfRange(const int ix, const int iy) {
			return (ix < 0 || ix > width_map - 1 || iy < 0 || iy > height_map - 1);
		}
		inline bool IsOccupied(const int ix, const int iy) {
			if (OutOfRange(ix, iy)) {
				return true;
			}
			return ((*envmap)[ix][iy] == 1);
		}

		// if fillin_nullptr is true, this function will create a new node when the node at (x, y) doesn't exist
		Node2D* LocateNode(const double x, const double y, bool fillin_nullptr = true);
		void LocateIndex(const double x, const double y, int& ix, int& iy);
		double RightHandside(Node2D* current_state);
		void UpdateKey(Node2D* current_state);
		void UpdateStates(Node2D* current_state);
		bool ComputeShortestPath();
		void InitNode(Node2D* s, const double x, const double y);
		void InitPlanner();
		void MatchSearchGraph();
		// Calculate Edge Cost: inf if one of the node is a part of obstacles; kActionCost2D[i] otherwise
		double EdgeCost(const Node2D* s1, const Node2D* s2, const int action_id);

		
		// Set the graph to nullptr, initialize a node when it is covered.
		inline void InitSearchGraph(const int width, const int height, const int heading_grid = 0) {
			search_graph.resize(width);
			for (auto& i : search_graph) {
				i.resize(height);
				for (auto& j : i) {
					j = nullptr;
				}
			}
			if (s_start != nullptr) {
				search_graph[s_start->x][s_start->y] = s_start;
			}
		}

	public:
		void UnitTest();
		bool MakePlan();
		DStarLite(Node2D* start_node, Node2D* goal_node, std::vector<std::vector<unsigned char>>* map_info);
		DStarLite() = delete;
		void UpdateMap(std::vector<std::vector<unsigned char>>* map_new);
		// void SetStartNode(const double x, const double y);
		// void SetStartNode(Node2D* start_node);
		// void SetGoalNode(const double x, const double y);
		// void SetGoalNode(Node2D* goal_node);

		// This path is represented as a linked table
		inline const Node2D* GetOptimalPath() {
			return s_start;
		}
	};

}