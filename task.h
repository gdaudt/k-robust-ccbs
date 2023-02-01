#pragma once
#include "tinyxml2.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "tinyxml.h"
#include "const.h"
#include "structs.h"
#include "map.h"

class Task{
    private:
        std::vector<Agent> agents;
    public:
        bool get_task(const char* FileName, int k=-1);
        void make_ids(int width);
        int get_agents_size() const { return agents.size(); };
        void print_task()
        {
            //for(int i=0; i<agents.size(); i++)
            //    std::cout<<i<<","<<agents[i].start_i<<","<<agents[i].start_j<<","<<agents[i].goal_i<<","<<agents[i].goal_j<<"\n";
            for(auto agent:agents)
                std::cout<<"<agent start_i=\""<<agent.start_i<<"\" start_j=\""<<agent.start_j<<"\" goal_i=\""<<agent.goal_i<<"\" goal_j=\""<<agent.goal_j<<"\"/>\n";
        }
        std::vector<Agent> get_agents() const { return agents; };
        Agent get_agent (int id) const;
        Task();
};