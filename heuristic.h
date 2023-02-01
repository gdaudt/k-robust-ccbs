#pragma once

#include "map.h"
#include "structs.h"
#include <unordered_map>
#include <vector>
#include "const.h"

class Heuristic{
    private:
        Open_List open;
        std::vector<std::vector<double>> h_values;
        Node find_min();
        double dist(const Node& a, const Node& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
    public:
        Heuristic(){};
        void init(unsigned int size, unsigned int agents);
        //calculate h values for all nodes in the map for a given agent
        void calculate_h_values(const Map& map, const Agent agent);
        unsigned int get_size() const { return h_values.size(); };
        double get_value(int id_node, int id_agent) const { return h_values[id_node][id_agent]; };
        void print(int agentid){
            std::cout << "h_values for agent " << agentid << ": " << std::endl;
            for(unsigned int i = 0; i < h_values.size(); i++)
                std::cout << h_values[i][agentid] << " ";
            std::cout << std::endl;
        }
};