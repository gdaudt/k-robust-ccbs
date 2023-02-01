#include "cbs.h"

bool CBS::init_root(const Task& task, const Map& map)
{
    CBS_Node root;
    std::cout << "Initializing root node..." << std::endl;
    simplePath path;
    for(int i = 0; i < task.get_agents().size(); i++)
    {
        Agent agent = task.get_agent(i);
        std::cout << "finding path for agent " << i << std::endl;      
        path = planner.findPath(agent, map, {}, heuristic);
        if(path.cost < 0)
            return false;
        root.paths.push_back(path);
        root.cost += path.cost;
    }
    root.low_level_expanded = 0;
    root.parent = nullptr;
    root.id = 1;
    root.id_str = "1";
    auto conflicts = get_all_conflicts(root.paths, -1);
    root.conflicts_num = conflicts.size();

    for (auto conflict:conflicts){
        root.conflicts.push_back(conflict);
        conflict.print();
    }
    solution.init_cost = root.cost;
    tree.add_node(root);
    tree.print();
    return true;
}

Conflict CBS::get_conflict(std::list<Conflict>& conflicts)
{
    auto best_it = conflicts.begin();
    for(auto it = conflicts.begin(); it != conflicts.end(); it++)
    {
        if(best_it->time < it->time)
            best_it = it;
    }
    Conflict conflict = *best_it;
    conflicts.erase(best_it);   
    return conflict;
}

std::vector<Conflict> CBS::get_all_conflicts(const std::vector<simplePath>& paths, int id)
{
    std::vector<Conflict> conflicts;
    //check all conflicts
    if(id < 0){
        for(int i = 0; i < paths.size(); i++)
        {
            for(int j = i+1; j < paths.size(); j++)
            {
                auto conflict = check_paths(paths[i], paths[j]);
                if(conflict.agent1 >= 0)
                    conflicts.push_back(conflict);
            }
        }
    }
    else //check conflicts with agent id
    {
        for(int i = 0; i < paths.size(); i++)
        {
            if(int (i) == id)
                continue;
            auto conflict = check_paths(paths[i], paths[id]);
            if(conflict.agent1 >= 0)
                conflicts.push_back(conflict);            
        }
    }
    return conflicts;
}

Conflict CBS::check_paths(const simplePath& pathA, const simplePath& pathB)
{
    unsigned int a(0), b(0);
    auto nodesA = pathA.nodes;
    auto nodesB = pathB.nodes;
    while(a < nodesA.size() - 1 || b < nodesB.size() - 1){
        double dist = sqrt(pow(map->get_i(nodesA[a].id) - map->get_i(nodesB[b].id), 2) + pow(map->get_j(nodesA[a].id) - map->get_j(nodesB[b].id), 2)) - CN_EPSILON;
        if(a < nodesA.size() - 1 && b < nodesB.size() - 1) // if both agents have not reached their goals yet
        {
            if(dist < (nodesA[a+1].g - nodesA[a].g) + (nodesB[b+1].g - nodesB[b].g) + CN_AGENT_SIZE*2)
                if(check_conflict(Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[a].g, nodesB[b].g), Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1]));
        }
        else if(a == nodesA.size() - 1) // if agent A has already reached the goal
        {
            if(dist < (nodesB[b+1].g - nodesB[b].g) + CN_AGENT_SIZE*2)
                if(check_conflict(Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[a].g, nodesB[b].g), Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id), Move(nodesB[b], nodesB[b+1]));
        }
        else if(b == nodesB.size() - 1) // if agent B has already reached the goal
        {
            if(dist < (nodesA[a+1].g - nodesA[a].g) + CN_AGENT_SIZE*2)
                if(check_conflict(Move(nodesA[a], nodesA[a+1]), Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id)))
                    return Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[a].g, nodesB[b].g), Move(nodesA[a], nodesA[a+1]), Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id));
        }
        //if a has reached its goal, increment b
        if(a == nodesA.size() - 1)
            b++;
        //if b has reached its goal, increment a
        else if(b == nodesB.size() - 1)
            a++;
        //if both future agents cost is the same, increment both
        else if(fabs(nodesA[a+1].g - nodesB[b+1].g) < CN_EPSILON)
        {
            a++;
            b++;
        }
        //if a's future cost is smaller, increment a
        else if(nodesA[a+1].g < nodesB[b+1].g)
            a++;
        //if b's future cost is smaller, increment b
        else if(nodesB[b+1].g - CN_EPSILON < nodesA[a+1].g)
            b++;
    }    
    return Conflict();
}

std::list<Constraint> CBS::get_constraints(CBS_Node *node, int agent_id){
    CBS_Node* current = node;
    std::list<Constraint> constraints(0);
    while(current->parent != nullptr){
        if(agent_id < 0 || current->constraint.agent == agent_id)
            constraints.push_back(current->constraint);
        current = current->parent;
    }
    return constraints;
}

bool CBS::check_conflict(Move move1, Move move2){      
    double startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1), endTimeB(move2.t2);
    double m1i1(map->get_i(move1.id1)), m1i2(map->get_i(move1.id2)), m1j1(map->get_j(move1.id1)), m1j2(map->get_j(move1.id2));
    double m2i1(map->get_i(move2.id1)), m2i2(map->get_i(move2.id2)), m2j1(map->get_j(move2.id1)), m2j2(map->get_j(move2.id2));
    Vector2D A(m1i1, m1j1);
    Vector2D B(m2i1, m2j1);
    Vector2D VA((m1i2 - m1i1)/(move1.t2 - move1.t1), (m1j2 - m1j1)/(move1.t2 - move1.t1));
    Vector2D VB((m2i2 - m2i1)/(move2.t2 - move2.t1), (m2j2 - m2j1)/(move2.t2 - move2.t1));
    if(startTimeB > startTimeA)
    {
        A += VA*(startTimeB-startTimeA);
        startTimeA = startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
        B += VB*(startTimeA - startTimeB);
        startTimeB = startTimeA;
    }
    double r(2*CN_AGENT_SIZE);
    Vector2D w(B - A);
    double c(w*w - r*r);
    if(c < 0){
        std::cout << "c < 0, agents will collide<< " << std::endl; 
        return true;
    }
    Vector2D v(VA - VB);
    double a(v*v);
    double b(w*v);
    double dscr(b*b - a*c);
    if(dscr - CN_EPSILON < 0){
        std::cout << "dscr < 0, agents will not collide" << std::endl;
        return false;
    }
    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON){
        std::cout << "ctime > 0, agents will collide" << std::endl;
        return true;
    }
    return false;
}

Constraint CBS::get_wait_constraint(int agent, Move move1, Move move2){
    
}

//this is probably the main place where to put the k-robust modification 
Constraint CBS::get_constraint(int agent, Move move1, Move move2){
    if(move1.id1 == move1.id2)
        return get_wait_constraint(agent, move1, move2);
    if(move2.t2 == CN_INFINITY)
        return Constraint(agent, move1.t1, CN_INFINITY, move1.id1, move1.id2);
    double startTimeA(move1.t1), endTimeA(move1.t2);
    double delta = move2.t2 - move2.t1;
    while (delta > config.precision/2.0){
        if(check_conflict(move1, move2)){
            move1.t1 += delta;
            move1.t2 += delta;
        }
        else{
            move1.t1 -= delta;
            move1.t2 -= delta;
        }
        if(move1.t1 > move1.t2 + CN_EPSILON){
            move1.t1 = move1.t2;
            move1.t2 = move1.t1 + endTimeA - startTimeA;
            break;
        }
        delta /= 2.0;
    }
    if(delta < config.precision/2.0 + CN_EPSILON && check_conflict(move1, move2)){
        move1.t1 = fmin(move1.t1 + delta*2, move2.t2);
        move1.t2 = move1.t1 + endTimeA - startTimeA;
    }
    return Constraint(agent, startTimeA, move1.t1, move1.id1, move1.id2);
}

void CBS::find_new_conflicts(const Map& map, const Task& task, CBS_Node& node, 
    std::vector<simplePath>& paths, const simplePath path, const std::list<Conflict>& conflicts, int& low_level_searches, int& low_level_expansions){
    

}


Solution CBS::solve(const Map& map, const Task& task, const Config& cfg){
    std::cout << "solving" << std::endl;
    config = cfg;
    this->map = &map;
    heuristic.init(map.get_size(), task.get_agents_size());
    for(int i = 0; i < int(task.get_agents_size()); i++){
        Agent agent = task.get_agent(i);
        heuristic.calculate_h_values(map, agent);
    }
    auto t = std::chrono::high_resolution_clock::now();
    if(!this->init_root(task, map))
        return solution;
    solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.found = true;
    CBS_Node node;
    std::chrono::duration<double> time_spent;
    int expanded(1);
    double time(0);
    std::list<Conflict> conflicts;
    Conflict conflict;
    std::vector<int> conflicting_agents;
    std::vector<std::pair<int, int>> conflicting_pairs;
    int low_level_searches(0);
    int low_level_expanded(0);
    int id = 2;
    do{
        auto parent = tree.get_best_node();
        node = *parent;
        node.cost -= node.h;
        node.print();
        parent->conflicts.clear();
        auto paths = get_paths(&node, task.get_agents_size());
        auto time_now = std::chrono::high_resolution_clock::now();
        conflicts = node.conflicts;
        if(conflicts.empty()){
            //no conflicts, solution found
            std::cout << "solution found" << std::endl;
            break;
        }
        else{
            conflict = get_conflict(conflicts);
        }
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
        time += time_spent.count();
        expanded++;

        std::list<Constraint> constraintsA = get_constraints(&node, conflict.agent1);
        Constraint constraintA(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
        constraintsA.push_back(constraintA);
        simplePath pathA;
        pathA = planner.findPath(task.get_agent(conflict.agent1), map, constraintsA, heuristic);
        low_level_searches++;
        low_level_expanded += pathA.expanded;

        std::list<Constraint> constraintsB = get_constraints(&node, conflict.agent2);
        Constraint constraintB(get_constraint(conflict.agent2, conflict.move2, conflict.move1));
        constraintsB.push_back(constraintB);
        simplePath pathB;
        pathB = planner.findPath(task.get_agent(conflict.agent2), map, constraintsB, heuristic);
        low_level_searches++;
        low_level_expanded += pathB.expanded;

        CBS_Node right_node({pathA}, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), 0, node.total_constraints+1);
        CBS_Node left_node({pathB}, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), 0, node.total_constraints+1);

        right_node.id_str = node.id_str + "0";
        left_node.id_str = node.id_str + "1";
        right_node.id = id++;
        left_node.id = id++;

        if(pathA.cost > 0){
            time_now = std::chrono::high_resolution_clock::now();
            find_new_conflicts(map, task, right_node, paths, pathA, conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if(right_node.cost > 0){
                tree.add_node(right_node);
            }
        }

        if(pathB.cost > 0){
            time_now = std::chrono::high_resolution_clock::now();
            find_new_conflicts(map, task, left_node, paths, pathB, conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if(left_node.cost > 0){
                tree.add_node(left_node);
            }
        }

        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if(time_spent.count() > config.timelimit)
        {
            solution.found = false;
            break;
        }
        tree.print();

    } while (tree.get_open_size() > 0);



}

double CBS::get_cost(CBS_Node node, int agent_id){
    while(node.parent != nullptr){
        if(node.paths.begin()->agentID == agent_id)
            return node.paths.begin()->cost;
        node = *node.parent;
    }
    return node.paths.at(agent_id).cost;
}

std::vector<simplePath> CBS::get_paths(CBS_Node* node, unsigned int agents_size){
    CBS_Node* current = node;
    std::vector<simplePath> paths(agents_size);
    while(current->parent != nullptr){
        if(paths.at(current->paths.begin()->agentID).cost < 0)
            paths.at(current->paths.begin()->agentID) = *current->paths.begin();
        current = current->parent;
    }
    for(unsigned int i = 0; i < agents_size; i++){
        if(paths.at(i).cost < 0)
            paths.at(i) = current->paths.at(i);
    }
    return paths;
}