#include <iostream>

#include "utility.h"

namespace sub_routine_dstar_lite{
	double CalculateHeuristics(Node2D* s1, Node2D* s2) {
		double hc = 0;
		
		double dx2 = std::pow(s1->x - s2->x, 2);
		double dy2 = std::pow(s1->y - s2->y, 2);
		hc = std::sqrt(dx2 + dy2);
		// Sub optimal factor
		hc *= 1;

		// hc = std::fabs(s1->x - s2->x) + std::fabs(s1->y - s2->y);

		return hc;
	}

	void MakeNewMap(std::vector<std::vector<unsigned char>>& map_new, int i_cost, int j_cost, unsigned char cost) {
		if (map_new.empty()) {
			for (int i = 0; i < 10; i++) {
				std::vector<unsigned char> row;
				for (int j = 0; j < 5; j++) {
					if (i == i_cost && j == j_cost) {
						row.push_back(cost);
					}
					else {
						row.push_back(0);
					}
				}
				map_new.push_back(row);
			}
		}
		else {
			map_new[i_cost][j_cost] = cost;
		}
	}

	int NullptrCounter(const std::vector<std::vector<Node2D*>>* graph) {
		int num_nullptr = 0;
		for (auto& r : *graph){
			for (auto& i : r) {
				if (i == nullptr) {
					++num_nullptr;
				}
			}
		}
		return num_nullptr;
	}

	void PrintPath(Node2D* start_node) {
		Node2D* current_node = start_node;
		int limitor = 0;
		while (limitor < 100 && current_node != nullptr && current_node->successor != nullptr) {
			++limitor;
			std::cout << "(" << current_node->x << "," << current_node->y << ")" << "-->";
			current_node = current_node->successor;
		}
		if (current_node != nullptr) {
			std::cout << "(" << current_node->x << "," << current_node->y << ")" << std::endl;
		}
	}

	void PrintSearchGraph(const std::vector<std::vector<Node2D*>>& s_g) {
		for (auto& xi : s_g) {
			for (auto& yi : xi) {
				if (yi == nullptr) {
					std::cout << "(N,N,N)";
				}
				else {
					std::cout << "(" << yi->x << "," << yi->y << "," << yi->g_cost << "," << yi->rhs << ")";
				}
			}
			std::cout << std::endl;
		}
	}

}