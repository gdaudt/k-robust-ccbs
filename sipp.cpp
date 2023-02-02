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
        successor.f = successor.g + h_values.get_value(successor.id, agent.id);
        std::vector<std::pair<double, double>> intervals(0);
        auto colls_it = collision_intervals.find(successor.id);
        std::cout << "collision intervals for successor " << successor.id << std::endl;
        if(colls_it != collision_intervals.end()){
            std::pair<double, double> interval = {0, CN_INFINITY};
            //iterates through all the collision intervals for the successor node
            for(unsigned int i = 0; i < colls_it->second.size(); i++){
                std::cout << "interval: " << interval.first << ", " << interval.second << std::endl;
                //add each of the intervals to the colls_it vector
                interval.second = colls_it->second[i].first;
                intervals.push_back(interval);
                std::cout << "pushing back interval: " << interval.first << ", " << interval.second << std::endl;
                interval.first = colls_it->second[i].second;
                std::cout << "interval with first = colls_it[i].second: " << interval.first << ", " << interval.second << std::endl;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
            std::cout << "last interval: " << interval.first << ", " << interval.second << std::endl;
        }
        else{
            intervals.push_back({0, CN_INFINITY});
            std::cout << "none, adding whole time" << std::endl;
        }
        //iterates through all the intervals
        //gets the constraints for the current node
        auto cons_it = constraints.find({current.id, successor.id});
        int id(0);
        for(auto interval:intervals){
            successor.interval_id = id;
            std::cout << "interval id: " << id << std::endl;
            id++;
            //checks if the node has been visited 
            std::cout << "key value: " << successor.id + successor.interval_id * map.get_size() << std::endl;
            auto visited_it = visited.find(successor.id + successor.interval_id * map.get_size());
            if(visited_it != visited.end())
                if(visited_it->second.second){
                    std::cout << "node has been visited" << std::endl;
                    continue;
                }
            //checks if the interval end time is less than the successors.g
            if(interval.second < successor.g){
                std::cout << "interval end time is less than successor.g" << std::endl;
                continue;
            }
            //if the new g value is better, updates the g value, because it means that the successor arrives earlier to the node than the previous one
            if(interval.second < successor.g){
                std::cout << "successor.g = " << successor.g << "interval.first = " << interval.first << std::endl;
                successor.g = interval.first;
                std::cout << "successor.g = interval.first" << std::endl;
            }
            // if the constraints already exist between the current node and the new node, checks if the new g value is in the interval
            if(cons_it != constraints.end()){
                for(unsigned int i = 0; i < cons_it->second.size(); i++){
                    //if it is, updates the g value to the end of the interval
                    //newNode.g - cost = curNode.g, so if the cost to reach the current node is bigger than the t1 on the second node
                    // and if the cost to reach the current node is lower than the end time on the second node
                    // means that you can only reach the node after the t2 + the cost to travel the node
                    if(successor.g - cost + CN_EPSILON > cons_it->second[i].t1 && successor.g - cost < cons_it->second[i].t2){                       
                        successor.g = cons_it->second[i].t2 + cost;                        
                    }
                }
            }
            successor.interval = interval;
            // if the new node cost is larger than the time horizon or the new g value is larger than the time horizon, keep going
            if(successor.g - cost > current.interval.second || successor.g > successor.interval.second)
                continue;
            // if the node has been visited, checks if the new g value is better, if it is, updates the g value
            if(visited_it != visited.end()){
                if(visited_it->second.first - CN_EPSILON < successor.g){
                    continue;
                }
                else{
                    visited_it->second.first = successor.g;
                }
            }
            else{
                //if the node has not been visited, adds it to the visited map
                visited.insert({successor.id + successor.interval_id * map.get_size(), {successor.g, false}});
            }
            //since not working with landmarks, just get the h value previously computed
            current.f = successor.g + h_values.get_value(successor.id, agent.id);
            std::cout << "pushing successor: ";
            successor.print();
            successors.push_back(successor);            
        }

    }
}

void SIPP::make_constraints(std::list<Constraint>& constraints){
    for(auto con:constraints){
        if(con.id1 == con.id2){
            add_collision_interval(con.id1, con.t1, con.t2);
        }
        else{
            add_move_constraint(Move(con));
        }
    }
}

void SIPP::add_collision_interval(int id, double begin, double end){
    std::vector<std::pair<double, double>> intervals(0);
    auto interval = std::make_pair(begin, end);
    if(collision_intervals.count(id) == 0){
        collision_intervals.insert({id, {interval}});
    }
    else{
        collision_intervals[id].push_back(interval);
    }
    std::sort(collision_intervals[id].begin(), collision_intervals[id].end());
    for(unsigned int i = 0; i < collision_intervals[id].size() - 1; i++){
        //if the end of the current interval is greater than the start of the next interval
        if(collision_intervals[id][i].second + CN_EPSILON > collision_intervals[id][i + 1].first){
            //blend the two intervals and erase the next one
            collision_intervals[id][i].second = collision_intervals[id][i + 1].second;
            collision_intervals[id].erase(collision_intervals[id].begin() + i + 1);
            i--;
        }
    }
}

void SIPP::add_move_constraint(Move move){
    std::vector<Move> move_cons(0);
    if(constraints.count({move.id1, move.id2}) == 0){
        constraints.insert({{move.id1, move.id2}, {move}});
    }
    else{
       move_cons = constraints.at({move.id1, move.id2});
       bool inserted(false);
       for(unsigned int i = 0; i < move_cons.size(); i++){
           if(inserted){
               break;
           }
           //if the new move starts before the current move
           if(move_cons[i].t1 > move.t1){
                //if the new move ends after the current move starts
               if(move_cons[i].t1 < move.t2 + CN_EPSILON){
                    //update the current move to start at the new move's start
                    move_cons[i].t1 = move.t1;
                    //if the new move ends after the current move ends
                    if(move_cons[i].t2 < move.t2 + CN_EPSILON){
                        //update the current move to end at the new move's end
                        move_cons[i].t2 = move.t2;
                    }
                    inserted = true;
                    //if there are moves before the current one
                    if(i != 0){
                        //if the new move starts before the previous move ends and new move ends after the previous move ends
                        if(move_cons[i-1].t2 + CN_EPSILON > move.t1 && move_cons[i-1].t2 < move.t2 + CN_EPSILON){
                            //update the previous move to end at the new move's end
                            move_cons[i-1].t2 = move.t2;
                            //erase the current move
                            move_cons.erase(move_cons.begin() + i);
                        }
                        inserted = true;
                    }
                }
                else{
                    //if there are moves before the current one
                    if(i != 0){
                        //if the current move's end is between new_move's start and end
                        if(move_cons[i-1].t2 + CN_EPSILON > move.t1 && move_cons[i-1].t2 < move.t2 + CN_EPSILON){
                            //update the previous move to end at the new move's end
                            move_cons[i-1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    move_cons.insert(move_cons.begin() + i, move);
                    inserted = true;
                    }
                }
           }
       }
       //if the last move's end is between new_move's start and end
       if(move_cons.back().t2 + CN_EPSILON > move.t1 && move_cons.back().t2 < move.t2 + CN_EPSILON){
           //update the last move to end at the new move's end
           move_cons.back().t2 = move.t2;
        }
        else if(!inserted){
            move_cons.push_back(move);
        }
        constraints.at({move.id1, move.id2}) = move_cons;
    }
}

Path SIPP::findPath(Agent agent, const Map& map, std::list<Constraint> constraints, Heuristic& h_values){
    this->clear();
    this->agent = agent;
    make_constraints(constraints);
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
    if(constraints.size() > 0)
        this->print_constraints();
    else
        std::cout << "No constraints" << std::endl;
    while(!open.empty()){
        current = open.top();
        open.pop();
        std::cout << "min node:" << std::endl;
        current.print();        
        auto v = visited.find(current.id + current.interval_id * map.get_size());
        if(v->second.second)
            continue;
        v->second.second = true;
        auto parent = &closed.insert({current.id + current.interval_id * map.get_size(), current}).first->second;
        std::cout << "parent: " << std::endl;
        parent->print();
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

