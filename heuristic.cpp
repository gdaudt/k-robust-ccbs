#include "heuristic.h"

void Heuristic::init(unsigned int size, unsigned int agents){
    h_values.clear();
    h_values.resize(size);
    for(unsigned int i = 0; i < size; i++)
        h_values[i].resize(agents, -1);
}

//calcultes all the h_values for a given agent
//the h_values are stored in the h_values vector
//gets the goal node for the agent and reverse searches the path to all the nodes in the map
void Heuristic::calculate_h_values(const Map& map, Agent agent){
    //starts at the goal node and computes the heuristic value for every node in the map
    Node curNode(agent.goal_id, 0, 0, agent.goal_i, agent.goal_j), newNode;
    open.clear();
    open.insert(curNode);
    while(!open.empty())
    {
        curNode = find_min();
        h_values[curNode.id][agent.id] = curNode.g;
        std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
        //valid moves are precomputed when the map is loaded
        //iterates through every valid move and computes the heuristic value using the dist function
        for(auto move: valid_moves)
        {
            // move.print();
            newNode.i = move.i;
            newNode.j = move.j;
            newNode.id = move.id;
            newNode.g = curNode.g + dist(curNode, newNode);
            if(h_values[newNode.id][agent.id] < 0)
            {
                auto it = open.get<1>().find(newNode.id);
                if(it != open.get<1>().end())
                {                    
                    if(it->g > newNode.g)
                        open.get<1>().erase(it);
                    else
                        continue;
                }
                open.insert(newNode);
            }
        }
    }
}

Node Heuristic::find_min(){
    Node min = *open.begin();
    open.erase(open.begin());
    return min;
}

