#include "sipp.h"

void SIPP::clear()
{
    clear_pq(open);
    closed.clear();
    collision_intervals.clear();
    constraints.clear();
    visited.clear();
    path.cost = -1;
}

double SIPP::distance(const Node& a, const Node& b)
{
    return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

void SIPP::find_successors(Node current, const Map& map, std::vector<Node>& successors, Heuristic& h_values, Node goal){
    
    Node successor;
    std::vector<Node> valid_moves = map.get_valid_moves(current.id);
    for(auto move:valid_moves){
        successor.id = move.id;
        successor.i = move.i;
        successor.j = move.j;
        double cost = distance(current, successor);
        successor.g = current.g + cost;
        // std::vector<std::pair<double, double>> intervals(0);
        // auto collision_it = collision_intervals.find(successor.id);
        //need to implement the methods to check if successors are valid
        //through the collision intervals of the nodes and through the constraints that exist between current and successor
        //constraints.find({current.id, successor.id})
        successor.f = successor.g + h_values.get_value(successor.id, agent.id);
        // std::cout << "successor: ";
        // successor.print();
        successors.push_back(successor);
    }
}

Path SIPP::findPath(Agent agent, const Map& map, std::list<Constraint> constraints, Heuristic& h_values){
    this->clear();
    this->agent = agent;
    Node start = Node(agent.start_id, 0, 0, agent.start_i, agent.start_j, nullptr, 0, 0); 
    Node goal = Node(agent.goal_id, 0, 0, agent.goal_i, agent.goal_j, nullptr, 0, 0);
    std::cout << "start: ";
    start.print();
    std::cout << "goal: ";
    goal.print();
    start.f = start.g + h_values.get_value(start.id, agent.id);
    open.push(start);
    Node current;
    std::vector<Node> successors;
    Path result;
    int expanded(0);
    while(!open.empty()){
        current = open.top();
        open.pop();
        // std::cout << "current: ";
        // current.print();
        auto parent = &closed.insert({current.id + current.interval_id * map.get_size(), current}).first->second;
        if(current.id == goal.id){
            result.nodes = construct_path(current);
            result.cost = current.g;
            result.expanded = int(closed.size());
            result.nodes.shrink_to_fit();
            result.agentID = agent.id;
            std::cout << "Goal found: " << std::endl;
            result.print();
            return result;
        }
        successors.clear();
        find_successors(current, map, successors, h_values, goal);
        // std::cout << "successors: " << successors.size() << std::endl;
        for(auto successor:successors){
            successor.parent = parent;
            if(closed.find(successor.id + successor.interval_id * map.get_size()) == closed.end()){
                open.push(successor);
            }
        }
    }
    return result;
}

std::vector<Node> SIPP::construct_path(Node current){
    path.nodes.clear();
    do{
        path.nodes.insert(path.nodes.begin(), current);
        current = *current.parent;
    }
    while(current.parent != nullptr);
    path.nodes.insert(path.nodes.begin(), current);

    return path.nodes;
}