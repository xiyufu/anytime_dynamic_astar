#pragma once

#include <vector>
#include <limits>
#include <cmath>
#include <map>

#include "utility.h"

namespace astar_planner {
    using namespace sub_routine_dstar_lite;

    // The A* planner
    class AStarPlanner {
        // vertices holds the object of nodes, other handles like start_pose_ will point to an address here
        std::vector<std::vector<Node2D*>> search_graph;
        // map of the environment, 0 is free, 100 is occupied
        std::vector<std::vector<unsigned char>> *envmap_;
        int height_map = 0;
        int width_map = 0;

        Node2D* start_node_ = nullptr;
        Node2D* goal_node_ = nullptr;

        std::map<PriorityKey, Node2D*, PKeyCompare> open_list_;

        int search_counter = 0;

        // 4-connected gird, start from upper left corner, clockwise.
        const int NUM_CONNECT = 4;
        const int actions_x[4] = { 0, 1, 0, -1 };
        const int actions_y[4] = { 1, 0, -1, 0 };
        const double actions_cost[4]{ 1, 1, 1, 1 };

        bool is_verbose;

        inline bool OutOfRange(const int ix, const int iy) {
            return (ix < 0 || ix > width_map - 1 || iy < 0 || iy > height_map - 1);
        }
        inline bool IsOccupied(const int ix, const int iy) {
            if (OutOfRange(ix, iy)) {
                return true;
            }
            return ((*envmap_)[ix][iy] == 1);
        }

        // Set the graph to nullptr, initialize a node when it is covered.
        inline void InitSearchGraph(const int width, const int height, const int heading_grid = 0) {
            search_graph.resize(width);
            for (auto& i : search_graph) {
                i.resize(height);
                for (auto& j : i) {
                    j = nullptr;
                }
            }
            if (start_node_ != nullptr) {
                search_graph[start_node_->x][start_node_->y] = start_node_;
            }
        }

        // to be changed for refined grid.
        void LocateIndex(const double x, const double y, int& ix, int& iy) {
            ix = static_cast<int>(x);
            iy = static_cast<int>(y);
        }

        void UpdateKey(Node2D* current_node) {
            current_node->pkey.k1 = current_node->g_cost + current_node->h_cost;
            current_node->pkey.k2 = current_node->g_cost;
            current_node->pkey.x = current_node->x;
            current_node->pkey.y = current_node->y;
        }

        Node2D* LocateNode(const double x, const double y, bool fillin_nullptr = true);

        void InitNodes(Node2D* s, const double x, const double y);
        double EdgeCost(const Node2D* s1, const Node2D* s2, const int action_id);

    public:
        int max_iter = 1e5;
        AStarPlanner() = delete;
        AStarPlanner(Node2D* start_node, Node2D* goal_node, std::vector<std::vector<unsigned char>>* map_info);
        bool MakePlan();
        void UnitTest();

        inline void SetMaxIter(const int max_i) {
            max_iter = max_i;
        }

        // This path is represented as a linked table
        inline const Node2D* GetOptimalPath() {
            return start_node_;
        }

    };

}
