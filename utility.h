#pragma once

#include <cmath>
#include <chrono>
#include <vector>
#include <limits>

namespace sub_routine_dstar_lite {

	// is in the open list, has been expended, is not touched, is being expended
	// This is used for the A* planner
	enum STATUS { OPEN, CLOSED, UNTOCHED, EXPENDING };

	// The priority key that is used to sort the OPEN list
	struct PriorityKey {
		double k1 = 0;
		double k2 = 0;
		double x = 0;
		double y = 0;

		// The copy constructor, I think the complier will probably do this for me. Just in case...
		PriorityKey& operator= (const PriorityKey& rhs) {
			this->k1 = rhs.k1;
			this->k2 = rhs.k2;
			this->x = rhs.x;
			this->y = rhs.y;
			return *this;
		}
	};

	// Key compare rule: key1 < key2 iff (key1.k1 < key2.k1) or (key1.k1 == key2.k1 && key1.k2 < key2.k2)
	// It's possible that 2 different nodes have the same key, so there coordinates are also compared
	struct PKeyCompare {
		bool operator()(const PriorityKey& lhs, const PriorityKey& rhs) const {
			if (lhs.k1 < rhs.k1) {
				return true;
			}
			else if (lhs.k1 == rhs.k1) {
				if (lhs.k2 == rhs.k2) {
					if (lhs.x == rhs.x) {
						return (lhs.y < rhs.y);
					}
					return (lhs.x < rhs.x);
				}
				return (lhs.k2 < rhs.k2);
			}
			else {
				return false;
			}
		}
	};

	const double kMaximumVal = std::numeric_limits<double>::max() / 4;

	struct Node2D {
		// label used in A* search
		STATUS node_status = UNTOCHED;

		// position
		double x = 0;
		double y = 0;

		// h cost
		double h_cost = 0;
		// g cost 
		double g_cost = kMaximumVal;
		// right hand side
		double rhs = kMaximumVal;

		// The following pointers are used to restore the path
		Node2D* predecessor = nullptr;
		Node2D* successor = nullptr;

		PriorityKey pkey;

		Node2D& operator= (const Node2D& rhs_node) {
			this->x = rhs_node.x;
			this->y = rhs_node.y;
			this->h_cost = rhs_node.h_cost;
			this->g_cost = rhs_node.g_cost;
			this->rhs = rhs_node.rhs;
			// yes, they should point to the same address
			this->predecessor = rhs_node.predecessor;
			this->successor = rhs_node.successor;

			return *this;
		}
	};

	// The only difference is the orientation
	struct Node3D : Node2D {
		// orientation
		double theta = 0;

		Node3D& operator= (const Node3D& rhs_node) {
			this->theta = rhs_node.theta;

			return *this;
		}
	};

	// motion primitives
	// up, right, down, left
	const std::vector<std::vector<int>> kAction2D{ {1,0}, {0,1}, {-1,0}, {0,-1} };
	const std::vector<double> kActionCost2D{ 1, 1, 1, 1 };

	// This function could be used as both the forward step and the backward step, if the action cost are the same
	inline void StateTransition(const Node2D* current_state, double& next_x, double& next_y, const int action_id) {
		next_x = current_state->x + static_cast<double>(kAction2D[action_id][0]);
		next_y = current_state->y + static_cast<double>(kAction2D[action_id][1]);
	}

	inline void StateTransition(const double current_x, const double current_y, double& next_x, double& next_y, const int action_id) {
		next_x = current_x + static_cast<double>(kAction2D[action_id][0]);
		next_y = current_y + static_cast<double>(kAction2D[action_id][1]);
	}

	// current heurisitcs: The Euclidean norm
	double CalculateHeuristics(Node2D* s1, Node2D* s2);

	// This function takes a map and change the (i, j) element into cost
	// If map_new is empty, a new map will be created
	void MakeNewMap(std::vector<std::vector<unsigned char>>& map_new, int i, int j, unsigned char cost);

	int NullptrCounter(const std::vector<std::vector<Node2D*>>* graph);

	// This function only prints the indices of nodes in the path. No visualization...
	void PrintPath(Node2D* statr_node);

	void PrintSearchGraph(const std::vector<std::vector<Node2D*>>& s_g);
}