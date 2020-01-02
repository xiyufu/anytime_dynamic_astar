#include <iostream>

#include "baseline_astar.h"
#include "utility.h"

namespace astar_planner {

    Node2D* AStarPlanner::LocateNode(const double x, const double y, bool fillin_nullptr) {
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
            InitNodes(search_graph[ix][iy], x, y);
        }
        return search_graph[ix][iy];
    }

    void AStarPlanner::InitNodes(Node2D* s, const double x, const double y) {
        // g_cost is default initialized as kMaximum
        s->x = x;
        s->y = y;
        if (goal_node_ != nullptr) {
            s->h_cost = CalculateHeuristics(goal_node_, s);
        }
        else {
            s->h_cost = 0;
        }
        UpdateKey(s);
    }

    AStarPlanner::AStarPlanner(Node2D* start_node, Node2D* goal_node, std::vector<std::vector<unsigned char>>* map_info) {
        start_node_ = start_node;
        goal_node_ = goal_node;
        envmap_ = map_info;
        width_map = map_info->size();
        height_map = map_info->back().size();

        InitSearchGraph(width_map, height_map);

        int x_s, y_s, x_g, y_g;
        LocateIndex(start_node->x, start_node->y, x_s, y_s);
        LocateIndex(goal_node->x, goal_node->y, x_g, y_g);
        search_graph[x_s][y_s] = start_node;
        search_graph[x_g][y_g] = goal_node;
    }

    double AStarPlanner::EdgeCost(const Node2D* s1, const Node2D* s2, const int action_id) {
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

    // peform A* search, if successed, return true, store the results into plan_ 
    bool AStarPlanner::MakePlan() {
        open_list_.clear();
        start_node_->successor = nullptr;
        start_node_->g_cost = 0;
        start_node_->h_cost = CalculateHeuristics(start_node_, goal_node_);
        start_node_->node_status = OPEN;
        UpdateKey(start_node_);
        open_list_[start_node_->pkey] = start_node_;
        int iter_counter = 0;

        while (!open_list_.empty() && iter_counter < max_iter) {
            ++iter_counter;

            Node2D* current_node = open_list_.begin()->second;
            current_node->node_status = EXPENDING;
            open_list_.erase(current_node->pkey);

            // reached goal?
            if (goal_node_->node_status == EXPENDING) {
                return true;
            }

            // expending current node
            for (int i = 0; i < 4; ++i) {
                double pre_x, pre_y;
                StateTransition(current_node, pre_x, pre_y, i);
                // below, new nodes will be created
                Node2D* pre_node = LocateNode(pre_x, pre_y);
                if (pre_node != nullptr && pre_node->node_status != CLOSED) {
                    double delta_g = EdgeCost(current_node, pre_node, i);
                    if (delta_g + current_node->g_cost < pre_node->g_cost) {
                        pre_node->g_cost = delta_g + current_node->g_cost;
                        pre_node->predecessor = current_node;
                    }
                    if (pre_node->node_status != OPEN) {
                        UpdateKey(pre_node);
                        pre_node->node_status = OPEN;
                        open_list_[pre_node->pkey] = pre_node;
                    }
                    else {
                        open_list_.erase(pre_node->pkey);
                        UpdateKey(pre_node);
                        open_list_[pre_node->pkey] = pre_node;
                    }
                }
            }

            current_node->node_status = CLOSED;

        }

        // search failed
        if (open_list_.empty()) {
            std::cout << "Couldn't find a plan" << std::endl;
        }

        return false;
    }


}