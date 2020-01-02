#include <algorithm>
#include <cmath>
#include <iostream>

#include "dstar_lite.h"
#include "utility.h"

namespace sub_routine_dstar_lite {

	// open_list = empty()
	// km = 0
	// rhs(s_goal) = 0
	// UpdateKey(s_goal)
	// open_list[key(s_goal)] = s_goal
	void DStarLite::InitPlanner() {
		// This is only for testing, the initialization should be careful
		open_list.clear();
		km = 0;
		if (s_start == nullptr) {
			s_start = new Node2D;
			InitNode(s_start, 0, 0);
		}
		s_start->h_cost = 0;
		s_start->g_cost = kMaximumVal;
		s_start->rhs = kMaximumVal;
		UpdateKey(s_start);
		if (s_goal == nullptr) {
			s_goal = new Node2D;
			InitNode(s_goal, 0, 0);
		}
		s_goal->h_cost = CalculateHeuristics(s_goal, s_start);
		s_goal->g_cost = kMaximumVal;
		s_goal->rhs = 0;
		UpdateKey(s_goal);
		open_list[s_goal->pkey] = s_goal;
	}

	void DStarLite::InitNode(Node2D* s, const double x, const double y) {
		// g_cost and rhs are default initialized as kMaximum
		s->x = x;
		s->y = y;
		if (s_start != nullptr) {
			s->h_cost = CalculateHeuristics(s_start, s);
		}
		else {
			s->h_cost = 0;
		}
		RightHandside(s);
		UpdateKey(s);
	}
	// Return the pointer that points to the node at given x and y
	// In simple 2D map, it is assumed that the index of a node in the graph 
	// is the same with its position in (x, y).
	Node2D* DStarLite::LocateNode(const double x, const double y, bool fillin_nullptr) {
		int ix = static_cast<int>(x);
		int iy = static_cast<int>(y);
		// out of boundary
		if (OutOfRange(ix, iy)) {
			return nullptr;
		}
		// obstacles
		if (IsOccupied(ix, iy)) {
			return nullptr;
		}
		if (search_graph[ix][iy] == nullptr && fillin_nullptr) {
			search_graph[ix][iy] = new Node2D;
			InitNode(search_graph[ix][iy], x, y);
		}
		return search_graph[ix][iy];
	}

	// to be changed for refined grid.
	void DStarLite::LocateIndex(const double x, const double y, int& ix, int& iy) {
		ix = static_cast<int>(x);
		iy = static_cast<int>(y);
	}

	double DStarLite::EdgeCost(const Node2D* s1, const Node2D* s2, const int action_id) {
		if (s1 == nullptr || s2 == nullptr) {
			return kMaximumVal;
		}
		int ix1, iy1, ix2, iy2;
		LocateIndex(s1->x, s1->y, ix1, iy1);
		LocateIndex(s2->x, s2->y, ix2, iy2);
		if (IsOccupied(ix1, iy1) || IsOccupied(ix2, iy2)) {
			return kMaximumVal;
		}
		return kActionCost2D[action_id];
	}

	void DStarLite::UpdateMap(std::vector<std::vector<unsigned char>>* map_new) {
		if (map_new->size() != width_map || map_new->back().size() != height_map) {
			// The memory of this map object is allocatied outside the planner, no delete here
			// a shared pointer might be better?
			envmap = map_new;
			map_changed = true;
		}
		bool map_changed_flag = false;
		for (int i = 0; i < width_map; ++i) {
			for (int j = 0; j < height_map; ++j) {
				if ((*envmap)[i][j] != (*map_new)[i][j]) {
					Node2D* n = LocateNode(static_cast<double>(i), static_cast<double>(j), false);
					if (n != nullptr) {
						changed_node.push_back(n);
					}
					(*envmap)[i][j] = (*map_new)[i][j];
					map_changed_flag = true;
				}
			}
		}
		map_changed = map_changed_flag;
	}

	// Compute and update the right hand side value of current_state, which is the 
	// minimum of c(s, s') + g(s'), s = current_state, s': all possible previous states
	// rhs is the optimal cost-to-go when env info is updated, while g(s) is the old cost.
	double DStarLite::RightHandside(Node2D* current_state) {
		// rhs(s_goal) == 0, we are searching backward
		if (current_state->x == s_goal->x && current_state->y == s_goal->y) {
			return 0;
		}
		double rhs = kMaximumVal;
		// in this simple scenarios, the StateTransition could be used for both forward and backward movement
		int num_actions = 4;
		// search all the successors of the current_state, successors because we're in backward searching
		// The closer the node to the goal state, the smaller the g_cost! And s_goal->g_cost = 0;
		// Note that the pointer called successor in the node is only meant to be used when recovering the optimal path.
		for (int i = 0; i < num_actions; ++i) {
			double next_x, next_y;
			StateTransition(current_state, next_x, next_y, i);
			// If the node doesn't exist, this function won't create it. It will only be created at UpdateStates.
			Node2D* next_node = LocateNode(next_x, next_y, false);
			double edge_cost = EdgeCost(current_state, next_node, i);
			if (next_node != nullptr && edge_cost + next_node->g_cost < rhs) {
				rhs = std::min(edge_cost + next_node->g_cost, kMaximumVal);
				// store path
				current_state->successor = next_node;
				next_node->predecessor = current_state;
			}
		}
		// update the node
		current_state->rhs = rhs;

		return rhs;
	}

	// calculate and store the key of a node
	void DStarLite::UpdateKey(Node2D* current_state) {
		// Note rhs of the current_node is not updated here. RightHandSide must be called explicitly to update rhs.
		double smaller_cost = std::min<double>(current_state->rhs, current_state->g_cost);
		current_state->pkey.k1 = smaller_cost + current_state->h_cost + km;
		current_state->pkey.k2 = smaller_cost;
		// It is possible to have the same k1 and k2 in different nodes, so (x, y) is also considered to avoid colision in the open list
		current_state->pkey.x = current_state->x;
		current_state->pkey.y = current_state->y;
	}

	// 1 if s was not visited before
	// 2	   g(s) = inf
	// 3 if (s != s_goal) rhs(s) = RightHandSide(s)
	// 4 if (s in open_list) remove s from open_list
	// 5 if (g(s) != rhs(s)) insert s into open_list with key(s)
	void DStarLite::UpdateStates(Node2D* current_state) {
		// 1, 2: g(s) is initialized as inf, so skip this part
		// 3
		double rhs_s = current_state->rhs;
		if (current_state->x != s_goal->x || current_state->y != s_goal->y) {
			// the rhs inside the node is updated
			rhs_s = RightHandside(current_state);
		}
		// 4
		if (open_list.count(current_state->pkey) > 0) {
			open_list.erase(current_state->pkey);
		}
		// 5
		if (current_state->g_cost != rhs_s) {
			UpdateKey(current_state);
			open_list[current_state->pkey] = current_state;
		}
		// 4,5 could be merged to save some operations, but let's focus on this version.
		
		// ++search_counter;
	}

	// This function removes nodes in the search_graph that have been identified as obstacles in map updates.
	void DStarLite::MatchSearchGraph() {
		for (auto& node_id : changed_id) {
			// don't have to do anything if the obstacle hasn't been explored 
			if (search_graph[node_id[0]][node_id[1]] == nullptr) {
				continue;
			}
			Node2D* temp_handle = search_graph[node_id[0]][node_id[1]];
			// remove obstacles from the open list
			if (open_list.count(temp_handle->pkey) > 0) {
				open_list.erase(temp_handle->pkey);
			}
			// remove obstacles from the search graph
			search_graph[node_id[0]][node_id[1]] = nullptr;
			// release the memory
			delete temp_handle;
		}
	}

	// 1. s = open_list.min_key()
	// 2. while (key(s) < key(s_start) || rhs(s_start) != g(s_start))
	// 3.     k_old = key(s)
	// 4.     pop s from open_list
	// 5.     update key(s)
	// 6.     if (k_old < key(s))
	// 7.         open_list.[key(s)] = s
	// 8.     else if (g(u) > rhs(u))
	// 9.         g(u) = rhs(u)
	// 10.        for all s in Pred(u) UpdateStates(s)
	// 10.    else 
	// 11.        g(u) = inf 
	// 12.        for all s' in Pred(u)and{u} update_states(s')
	bool DStarLite::ComputeShortestPath() {
		// search success?
		bool res_flag = false; 
		// helper function to compare keys
		PKeyCompare pkc;
		// 1.
		Node2D* current_node = open_list.begin()->second;
		// 2.
		UpdateKey(s_start);
		while (pkc(current_node->pkey, s_start->pkey) || s_start->rhs != s_start->g_cost) {
			PriorityKey k_old = current_node->pkey;
			open_list.erase(current_node->pkey);
			// RightHandside(current_node);
			UpdateKey(current_node);
			if (pkc(k_old, current_node->pkey)) {
				open_list[current_node->pkey] = current_node;
			}
			else if (current_node->g_cost > current_node->rhs) {
				++search_counter;
				current_node->g_cost = current_node->rhs;
				for (int i = 0; i < 4; ++i) {
					double pre_x, pre_y;
					StateTransition(current_node, pre_x, pre_y, i);
					// below, new nodes will be created
					Node2D* pre_node = LocateNode(pre_x, pre_y);
					if (pre_node != nullptr) {
						UpdateStates(pre_node);
					}
				}
			}
			else {
				++search_counter;
				current_node->g_cost = kMaximumVal;
				for (int i = 0; i < 4; ++i) {
					double pre_x, pre_y;
					StateTransition(current_node, pre_x, pre_y, i);
					Node2D* pre_node = LocateNode(pre_x, pre_y);
					if (pre_node != nullptr) {
						UpdateStates(pre_node);
					}
				}
				UpdateStates(current_node);
			}

			UpdateKey(s_start);

			// Has to do this check before changing current_node to avoid segmentation fault
			if (!(pkc(current_node->pkey, s_start->pkey) || s_start->rhs != s_start->g_cost)) {
				res_flag = true;
				break;
			}
			else if (open_list.empty()) {
				res_flag = false;
				break;
			}
			current_node = open_list.begin()->second;
			
			// The search process is not along the optimal path (and it shouldn't!) because the heaustic cost is 
			// kind of small. And it can't focus the search exactly on the optimal direction
		}
		
		// let's try to clean the open_list
		// open_list.clear();
		// while, no good...

		return res_flag;
	} // end ComputeShortestPath()

	// s_last = s_start
	// Initialize()
	// ComputeShortestPath()
	// while(s_start != s_goal){
	//     s_start = argmin{ g(s') + c(s_start, s') } s' in Succ(s_start)
	//     Move to Start (Take an action)
	//	   UpdateMap
	//     if any edge cost changed in the map
	//         km = km + h(s_last, s_start) heuristic cost
	//         s_last = s_start
	//         for all edge (u, v) that changed costs
	//              Update edge cost c(u, v)
	//              UpdateStates(v)
	//         ComputeShortestPath();
	bool DStarLite::MakePlan() {
		bool res_flag = false;
		Node2D* s_last = s_start;
		InitPlanner();
		res_flag = ComputeShortestPath();
		if (!res_flag) {
			return res_flag;
		}

		// hack
		int num_nullptr = NullptrCounter(&search_graph);
		PrintPath(s_start);
		// PrintSearchGraph(search_graph);

		while (s_start != s_goal) {
			// successor is used as the storage of the path
			s_start = s_start->successor;
			
			// no real hardware, no real action
			
			// In real application, map should be updated externally
			// This could be done, for example, by ROS callback
			// We're just demonstrating the algorithm here.
			std::vector<std::vector<unsigned char>> map_new;
			MakeNewMap(map_new, 7, 0, 1);
			MakeNewMap(map_new, 7, 1, 1);
			MakeNewMap(map_new, 7, 2, 1);
			MakeNewMap(map_new, 7, 3, 1);
			// MakeNewMap(map_new, 4, 4, 1);
			UpdateMap(&map_new);

			// If the map has been changed, update corresponding nodes
			if (map_changed) {
				km += CalculateHeuristics(s_last, s_start);
				s_last = s_start;
				s_start->g_cost = kMaximumVal;
				s_start->rhs = kMaximumVal;

				// MatchSearchGraph();
				std::map<PriorityKey, Node2D*, PKeyCompare> surroundings;

				for (auto& current_node : changed_node) {
					if (current_node == nullptr) {
						// shouldn't happen
						continue;
					}
					RightHandside(current_node); // rhs of the current node should be inf
					// update states of the surrounding nodes
					for (int i = 0; i < 4; ++i) {
						double sur_x, sur_y;
						StateTransition(current_node, sur_x, sur_y, i);
						Node2D* sur_node = LocateNode(sur_x, sur_y, false);
						if (sur_node != nullptr) {
							UpdateStates(sur_node);
						}
					}
					UpdateStates(current_node);
				}

				// hack
				// PrintSearchGraph(search_graph);
				
				// reset
				changed_xy.clear();
				changed_id.clear();
				changed_node.clear();
				map_changed = false;
				
				if (!open_list.empty()) {
					res_flag = ComputeShortestPath();
					if (!res_flag) {
						return res_flag;
					}
				}
				
				// hack
				PrintPath(s_start);

			}
		}
		if (s_goal == s_start) {
			res_flag = true;
		}
		
		return res_flag;

	}

	DStarLite::DStarLite(Node2D* start_node, Node2D* goal_node, std::vector<std::vector<unsigned char>>* map_info) {
		s_start = start_node;
		s_goal = goal_node;
		envmap = map_info;
		width_map = map_info->size();
		height_map = map_info->back().size();

		InitSearchGraph(width_map, height_map);

		int x_s, y_s, x_g, y_g;
		LocateIndex(start_node->x, start_node->y, x_s, y_s);
		LocateIndex(goal_node->x, goal_node->y, x_g, y_g);
		search_graph[x_s][y_s] = start_node;
		search_graph[x_g][y_g] = goal_node;
		
	}

}