#pragma once

#include <chrono>
#include "structs.h"
#include "map.h"
#include "task.h"
#include "heuristic.h"
#include "config.h"
#include "sipp.h"

class CBS{
    public:
        CBS(){};
        bool init_root(const Task& task, const Map& map);
        double get_cost(CBS_Node node, int agent_id);
        bool check_conflict(Move move1, Move move2);
        // bool validate_constraints(std::list<Constraint> constraints, int agent_id);
        void find_new_conflicts(const Map& map, const Task& task, CBS_Node& node, std::vector<simplePath>& paths, const simplePath path,
                                const std::list<Conflict>& conflicts, int& low_level_searches, int& low_level_expansions);
        Solution solve(const Map& map, const Task& task, const Config& config);
        Conflict get_conflict(std::list<Conflict>& conflicts);
        Conflict check_paths(const simplePath& pathA, const simplePath& pathB);
        Constraint get_wait_constraint(int agent, Move move1, Move move2);
        Constraint get_constraint(int agent, Move move1, Move move2);
        std::list<Constraint> get_constraints(CBS_Node* node, int agent_id);
        std::vector<Conflict> get_all_conflicts(const std::vector<simplePath>& paths, int id);
        std::vector<simplePath> get_paths(CBS_Node* node, unsigned int agents_size);
        
        Solution solution;
        CBS_Tree tree;
        SIPP planner;
        Heuristic heuristic;
        Config config;
        const Map* map;        
};