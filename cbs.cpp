#include "cbs.h"

bool CBS::init_root(const Task& task, const Map& map)
{
    CBS_Node root;
    // std::cout << "Initializing root node..." << std::endl;
    simplePath path;
    for(int i = 0; i < task.get_agents().size(); i++)
    {
        Agent agent = task.get_agent(i);
        // std::cout << "finding path for agent " << i << std::endl;
        rt.build({}, {}, agent.id, task.get_agent_start_id(agent.id));       
        path = planner.findPath(agent, map, {}, heuristic, rt);
        rt.clear();        
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
        // conflict.print();
    }
    solution.init_cost = root.cost;
    tree.add_node(root);
    // tree.print();
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
                // std::cout << "checking conflicts between agent " << i << " and agent " << j << std::endl;
                if(config.k_robustness > 0){
                    std::chrono::steady_clock::time_point kbegin = std::chrono::steady_clock::now();
                    auto k_conflict = check_paths_robust(paths[i], paths[j]);
                    if(k_conflict.agent1 >= 0)
                        conflicts.push_back(k_conflict);
                    std::chrono::steady_clock::time_point kend = std::chrono::steady_clock::now();
                    k_conflict_time += std::chrono::duration_cast<std::chrono::duration<double>>(kend - kbegin);
                }
                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                auto conflict = check_paths(paths[i], paths[j]);
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                conflict_time += std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
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
            if(config.k_robustness > 0){
                auto kbegin = std::chrono::steady_clock::now();
                auto k_conflict = check_paths_robust(paths[i], paths[id]);
                if(k_conflict.agent1 >= 0){
                    conflicts.push_back(k_conflict);            
                    // std::cout << "check_paths_robust conflict with agent " << i << " and agent " << id << " is " << k_conflict.agent1 << " and " << k_conflict.agent2 << std::endl;
                    // k_conflict.print();
                }
                auto kend = std::chrono::steady_clock::now();
                k_conflict_time += std::chrono::duration_cast<std::chrono::duration<double>>(kend - kbegin);
            }
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            auto conflict = check_paths(paths[i], paths[id]);
            if(conflict.agent1 >= 0){
                conflicts.push_back(conflict);            
                // std::cout << "check_paths conflict with agent " << i << " and agent " << id << " is " << conflict.agent1 << " and " << conflict.agent2 << std::endl;
                // conflict.print();
            }
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            conflict_time += std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
        }
    }
    return conflicts;
}

Conflict CBS::check_paths_robust(const simplePath& pathA, const simplePath& pathB){
    auto nodesA = pathA.nodes;
    auto nodesB = pathB.nodes;    
    unsigned int a(0), b(0);    
    Conflict c;    
    while(a < nodesA.size() -1 || b < nodesB.size() -1){        
        // std::cout << "testing at id " << nodesA.at(a).id << " and " << nodesB.at(b).id << std::endl;
        // std::cout << "g values are " << nodesA.at(a).g << " and " << nodesB.at(b).g << std::endl;
        if(nodesA.at(a).id == nodesB.at(b).id){
            // std::cout << "agent " << pathA.agentID << " and agent " << pathB.agentID << " are in the same position at a = " << a << " and b = " << b << std::endl;
            // std::cout << "nodesA.at(a).id= " << nodesA.at(a).id << " and nodesB.at(b).id = " << nodesB.at(b).id << std::endl;            
            // std::cout << "condition value = " << nodesA.at(a).g << " >= " << std::max(0.0, nodesB.at(b).g - config.k_robustness) << " and " << nodesA.at(a).g << " <= " << nodesA.at(b).g + config.k_robustness << std::endl;
            if(a == nodesA.size() -1){                
                c = Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[a].g, nodesB[b].g), Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id), Move(nodesB[b-1], nodesB[b]), true);
                c.k_robust = true;
                return c;
            }
            if(b == nodesB.size() -1){                
                c = Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[a].g, nodesB[b].g), Move(nodesA[a-1], nodesA[a]), Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id), true);
                c.k_robust = true;
                return c;
            }
            if(nodesA.at(a).g >= std::max(0.0, nodesB.at(b).g - config.k_robustness) && nodesA.at(a).g <= nodesB.at(b).g + config.k_robustness){
                if(nodesA.at(a).g == 0){
                    c = Conflict(pathA.agentID, pathB.agentID, nodesB.at(b).g, Move(0.0, 0.0, nodesA.at(a).id, nodesA.at(a).id), Move(nodesB.at(b-1), nodesB.at(b)), true);
                    //old
                    // c = Conflict(pathA.agentID, pathB.agentID, nodesB.at(b).g, Move(nodesA.at(a), nodesA.at(a+1)), Move(nodesB.at(b-1), nodesB.at(b)), true);
                    c.k_robust = true;
                    return c;
                }
                if(nodesB.at(b).g == 0){
                    // old
                    // c = Conflict(pathA.agentID, pathB.agentID, nodesA.at(a).g, Move(nodesA.at(a-1), nodesA.at(a)), Move(nodesB.at(b), nodesB.at(b+1)), true);
                    c = Conflict(pathA.agentID, pathB.agentID, nodesA.at(a).g, Move(nodesA.at(a-1), nodesA.at(a)), Move(0.0, 0.0, nodesB.at(b).id, nodesB.at(b).id), true);
                    c.k_robust = true;
                    return c;
                }
                else{
                    c = Conflict(pathA.agentID, pathB.agentID, nodesA.at(a).g, Move(nodesA.at(a-1), nodesA.at(a)), Move(nodesB.at(b-1), nodesB.at(b)), true);
                    c.k_robust = true;
                    return c;
                }                
            }
        }
        if(a == nodesA.size() -1){
            b++;
        }
        else if(b == nodesB.size() -1){
            a++;
        }
        else if(nodesA[a+1].g < nodesB[b+1].g - config.k_robustness)
            a++;
        //if b's future cost is smaller, increment b
        else if(nodesB[b+1].g - CN_EPSILON < nodesA[a+1].g - config.k_robustness)
            b++;
        else{
            a++;
        }

    }
    // for(int i = 0; i < nodesA.size(); i++){
    //     for(int j = 0; j < nodesB.size(); j++){
    //         if(nodesB.at(j).id == nodesA.at(i).id){
    //             // std::cout << "nodesB.at(i).g: " << nodesB.at(i).g << std::endl;
    //             // std::cout << "nodesA.at(j).g: " << nodesA.at(j).g << std::endl;
    //             // std::cout << "std::max(0.0, nodesB.at(i).g - config.k_robustness): " << std::max(0.0, nodesB.at(i).g - config.k_robustness) << std::endl;
    //             // std::cout << "nodesB.at(i).g + config.k_robustness: " << nodesB.at(i).g + config.k_robustness << std::endl;
    //             if(nodesA.at(i).g > std::max(0.0, nodesB.at(j).g - config.k_robustness) && nodesA.at(i).g < nodesB.at(j).g + config.k_robustness){
    //                 // std::cout << "generating k-robust conflict: ";
    //                 // std::cout << "agent " << pathA.agentID << " at " << nodesA.at(j).g << " and agent " << pathB.agentID << " at " << nodesB.at(i).g << std::endl;
    //                 // std::cout << "positions: " << nodesA.at(j).id << " and " << nodesB.at(i).id << std::endl;
    //                 if(nodesA.at(i).g == 0)
    //                     c = Conflict(pathA.agentID, pathB.agentID, nodesB.at(j).g, Move(nodesA.at(i), nodesA.at(i+1)), Move(nodesB.at(j-1), nodesB.at(j)), true);
    //                 else if(nodesB.at(j).g == 0)
    //                     c = Conflict(pathA.agentID, pathB.agentID, nodesA.at(i).g, Move(nodesA.at(i-1), nodesA.at(i)), Move(nodesB.at(j), nodesB.at(j+1)), true);
    //                 else if(i == nodesA.size()-1){
    //                     std::cout << "AGENT " << pathA.agentID << " at the end of the path" << std::endl;
    //                     c = Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[i].g, nodesB[j].g), Move(nodesA[i].g, CN_INFINITY, nodesA[i].id, nodesA[i].id), Move(nodesB[j-1], nodesB[j]), true);
    //                 }
    //                 else if(j == nodesB.size()-1){
    //                     std::cout << "AGENT " << pathB.agentID << " at the end of the path" << std::endl;
    //                     c = Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[i].g, nodesB[j].g), Move(nodesA[i-1], nodesA[i]), Move(nodesB[j].g, CN_INFINITY, nodesB[j].id, nodesB[j].id), true);
    //                 }
    //                 else
    //                     c = Conflict(pathB.agentID, pathA.agentID, std::max(nodesA.at(i).g, nodesB.at(j).g), Move(nodesB.at(j-1), nodesB.at(j)), Move(nodesA.at(i-1), nodesA.at(i)), true);
    //                 c.k_robust = true;
    //                 return c;
    //             }                    
    //         }
    //     }
    // }
    return Conflict();
}

Conflict CBS::check_paths(const simplePath& pathA, const simplePath& pathB)
{
    unsigned int a(0), b(0);
    auto nodesA = pathA.nodes;
    auto nodesB = pathB.nodes;
    Conflict c;    
    while(a < nodesA.size() - 1 || b < nodesB.size() - 1){
        double dist = sqrt(pow(map->get_i(nodesA[a].id) - map->get_i(nodesB[b].id), 2) + pow(map->get_j(nodesA[a].id) - map->get_j(nodesB[b].id), 2)) - CN_EPSILON;
        if(a < nodesA.size() - 1 && b < nodesB.size() - 1) // if both agents have not reached their goals yet
        {
            //just to remind, g + cn_agent_size*2 is the velocity of the agent
            if(dist < (nodesA[a+1].g - nodesA[a].g) + (nodesB[b+1].g - nodesB[b].g) + config.agent_size*2)
                if(check_conflict(Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[a].g, nodesB[b].g), Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1]));
        }
        else if(a == nodesA.size() - 1) // if agent A has already reached the goal
        {
            if(dist < (nodesB[b+1].g - nodesB[b].g) + config.agent_size*2)
                if(check_conflict(Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, std::min(nodesA[a].g, nodesB[b].g), Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id), Move(nodesB[b], nodesB[b+1]));
        }
        else if(b == nodesB.size() - 1) // if agent B has already reached the goal
        {
            if(dist < (nodesA[a+1].g - nodesA[a].g) + config.agent_size*2)
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


//is it possible that checking for a k-robust conflict can be the same as extending the time of one of the moves according to k_robust parameters?
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
    double r(2*config.agent_size);
    Vector2D w(B - A);
    double c(w*w - r*r);
    if(c < 0){
        // std::cout << "c < 0, agents will collide<< " << std::endl; 
        return true;
    }
    Vector2D v(VA - VB);
    double a(v*v);
    double b(w*v);
    double dscr(b*b - a*c);
    if(dscr - CN_EPSILON < 0){
        // std::cout << "dscr < 0, agents will not collide" << std::endl;
        return false;
    }
    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON){
        // std::cout << "ctime > 0, agents will collide" << std::endl;
        return true;
    }
    return false;
}

Constraint CBS::get_wait_constraint(int agent, Move move1, Move move2){
    // std::cout << "get_wait_constraint" << std::endl;
    double radius = 2*config.agent_size;
    // i0,j0 = move2id1 i1,j1 = move2id2 i2,j2 = move1id1
    double i0(map->get_i(move2.id1)), j0(map->get_j(move2.id1)), i1(map->get_i(move2.id2)), j1(map->get_j(move2.id2)), i2(map->get_i(move1.id1)), j2(map->get_j(move1.id1));
    std::pair<double,double> interval;
    Point point(i2,j2), p0(i0,j0), p1(i1,j1);
    int cls = point.classify(p0, p1);
    double dist = fabs((i0 - i1)*j2 + (j1 - j0)*i2 + (j0*i1 - i0*j1))/sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
    double da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
    double db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
    double ha = sqrt(da - dist*dist);
    double size = sqrt(radius*radius - dist*dist);
    if(cls == 3)
    {
        interval.first = move2.t1;
        interval.second = move2.t1 + (sqrt(radius*radius - dist*dist) - ha);
    }
    else if(cls == 4)
    {
        interval.first = move2.t2 - sqrt(radius*radius - dist*dist) + sqrt(db - dist*dist);
        interval.second = move2.t2;
    }
    else if(da < radius*radius)
    {
        if(db < radius*radius)
        {
            interval.first = move2.t1;
            interval.second = move2.t2;
        }
        else
        {
            double hb = sqrt(db - dist*dist);
            interval.first = move2.t1;
            interval.second = move2.t2 - hb + size;
        }
    }
    else
    {
        if(db < radius*radius)
        {
            interval.first = move2.t1 + ha - size;
            interval.second = move2.t2;
        }
        else
        {
            interval.first = move2.t1 + ha - size;
            interval.second = move2.t1 + ha + size;
        }
    }    
    // Constraint c(agent, interval.first, interval.second, move1.id1, move1.id2);
    // std::cout << "wait constraint: " << std::endl;
    // c.print();
    return Constraint(agent, interval.first, interval.second, move1.id1, move1.id2);
}

Constraint CBS::get_constraint(int agent, Move move1, Move move2, bool k_robust){
    Constraint c;
    std::pair<double, double> interval;    
    interval = std::make_pair(move1.t1, move1.t2);
    if(move1.id1 == move1.id2)
        // c = get_wait_constraint(agent, move1, move2);
        c = Constraint(agent, interval.first, interval.second, move1.id1, move1.id2);
    else{
        // std::cout << "RETURNING K-ROBUST CONSTRAINT" << std::endl;
        c = Constraint(agent, interval.first, interval.second, move1.id1, move1.id2);            
    }
    c.k_robust = k_robust;    
    return c;
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
    // double diff = startTimeA - config.k_robustness;
    // startTimeA = std::max(0.0, diff);
    // move1.t1 += config.k_robustness;
    // std::cout << "k = " << config.k_robustness << std::endl;
    return Constraint(agent, startTimeA, move1.t1, move1.id1, move1.id2);
}

//this function updates the conflicts of the tree node spawned from the current popped tree node
void CBS::find_new_conflicts(const Map& map, const Task& task, CBS_Node& node, 
                            std::vector<simplePath>& paths, const simplePath path, const std::list<Conflict>& conflicts, 
                            int& low_level_searches, int& low_level_expansions)
{
    paths[path.agentID] = path;
    auto new_conflicts = get_all_conflicts(paths, path.agentID);    

    // std::cout << "FINDING NEW CONFLICTS FOR AGENT " << path.agentID << std::endl;
    std::list<Conflict> conflictsA({});
    for(auto c: conflicts)
        if(c.agent1 != path.agentID && c.agent2 != path.agentID)
            conflictsA.push_back(c);
    node.conflicts = conflictsA;
    for(auto n:new_conflicts){
        node.conflicts.push_back(n);
        // n.print();
    }
    node.conflicts_num = node.conflicts.size();
    // std::cout << "new conflicts on node: " << node.conflicts.size() << std::endl;
    return;
}


Solution CBS::solve(const Map& map, const Task& task, const Config& cfg){
    std::cout << "solving" << std::endl;
    config = cfg;
    this->map = &map;
    std::cout << "rt.k_robustness = " << rt.k_robust << std::endl;
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
    bool prune_right = false;
    bool prune_left = false;
    do{
        auto parent = tree.get_best_node();
        // std::cout << "Popping node: ";
        // parent->print();
        // for(auto c: parent->conflicts)
        // {            
        //     c.print();
        // }
        // parent->constraint.print();
        node = *parent;
        node.cost -= node.h;
        parent->conflicts.clear();
        auto paths = get_paths(&node, task.get_agents_size());
        auto time_now = std::chrono::high_resolution_clock::now();
        conflicts = node.conflicts;
        if(conflicts.empty()){
            //no conflicts, solution found
            std::cout << "solution found" << std::endl;
            // tree.print();
            break;
        }
        else{
            conflict = get_conflict(conflicts);
        }
        // std::cout << "Conflicts in this iteration: ";
        // conflict.print();
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
        time += time_spent.count();
        expanded++;

        auto ctbegin = std::chrono::steady_clock::now();
        auto ctend = std::chrono::steady_clock::now();
        ct_time += std::chrono::duration_cast<std::chrono::duration<double>>(ctend - ctbegin);
        std::list<Constraint> constraintsA = get_constraints(&node, conflict.agent1);
        Constraint constraintA;
        if(conflict.k_robust)
            constraintA = get_constraint(conflict.agent1, conflict.move1, conflict.move2, conflict.k_robust);
        else
            constraintA = get_constraint(conflict.agent1, conflict.move1, conflict.move2);
        auto it = std::find(constraintsA.begin(), constraintsA.end(), constraintA);
        if(it == constraintsA.end())
            constraintsA.push_back(constraintA);
        else{
            prune_right = true;
        }
           
        // std::cout << "CONSTRAINT A GENERATED: " << std::endl;
        // constraintA.print();
        simplePath pathA;
        // std::cout << "REPLANNED AGENT " << conflict.agent1 << std::endl;
        auto rtbegin = std::chrono::steady_clock::now();
        rt.build(paths, constraintsA, conflict.agent1, task.get_agent_start_id(conflict.agent1));
        // std::cout << "Agent " << conflict.agent1 << " reservation table: " << std::endl;
        // rt.print();
        auto rtend = std::chrono::steady_clock::now();
        rt_time += std::chrono::duration_cast<std::chrono::duration<double>>(rtend - rtbegin);
        auto pathbegin = std::chrono::steady_clock::now();
        pathA = planner.findPath(task.get_agent(conflict.agent1), map, constraintsA, heuristic, rt);
        auto pathend = std::chrono::steady_clock::now();
        low_level_time += std::chrono::duration_cast<std::chrono::duration<double>>(pathend - pathbegin);
        rt.clear();
        low_level_searches++;
        low_level_expanded += pathA.expanded;

        simplePath pathB;
        std::list<Constraint> constraintsB = get_constraints(&node, conflict.agent2);
        Constraint constraintB;
        if(conflict.k_robust)
            constraintB = get_constraint(conflict.agent2, conflict.move2, conflict.move1, conflict.k_robust);
        else
            constraintB = get_constraint(conflict.agent2, conflict.move2, conflict.move1);
        it = std::find(constraintsB.begin(), constraintsB.end(), constraintB);
        if(it == constraintsB.end())
            constraintsB.push_back(constraintB);
        else{
            prune_left = true;
        }

        // std::cout << "CONSTRAINT B GENERATED: " << std::endl;
        // constraintB.print();
        // std::cout << "REPLANNED AGENT " << conflict.agent2 << std::endl;
        rtbegin = std::chrono::steady_clock::now();
        rt.build(paths, constraintsB, conflict.agent2, task.get_agent_start_id(conflict.agent2));
        // std::cout << "Agent " << conflict.agent2 << " reservation table: " << std::endl;
        // rt.print();
        rtend = std::chrono::steady_clock::now();
        rt_time += std::chrono::duration_cast<std::chrono::duration<double>>(rtend - rtbegin);
        pathbegin = std::chrono::steady_clock::now();
        pathB = planner.findPath(task.get_agent(conflict.agent2), map, constraintsB, heuristic, rt);
        pathend = std::chrono::steady_clock::now();
        low_level_time += std::chrono::duration_cast<std::chrono::duration<double>>(pathend - pathbegin);
        rt.clear();
        low_level_searches++;
        low_level_expanded += pathB.expanded;
        
        auto pathsA = replace_path(paths, conflict.agent1, pathA);
        auto pathsB = replace_path(paths, conflict.agent2, pathB);
        CBS_Node right_node(pathsA, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), 0, node.total_constraints+1);
        CBS_Node left_node(pathsB, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), 0, node.total_constraints+1);

        right_node.id_str = node.id_str + "0";
        left_node.id_str = node.id_str + "1";
        right_node.id = id++;
        left_node.id = id++;

        if(pathA.cost > 0){
            time_now = std::chrono::high_resolution_clock::now();
            find_new_conflicts(map, task, right_node, pathsA, pathA, conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if(right_node.cost > 0){
                // std::cout << "----ADDING RIGHT NODE----" << std::endl;
                // std::cout << "AGENT REPLANNED: " << conflict.agent1 << std::endl;
                // right_node.print();
                if(!prune_right){
                    tree.add_node(right_node); 
                    prune_right = false;
                }
            }
        }

        if(pathB.cost > 0){
            time_now = std::chrono::high_resolution_clock::now();
            find_new_conflicts(map, task, left_node, pathsB, pathB, conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if(left_node.cost > 0){
                // std::cout << "----ADDING LEFT NODE----" << std::endl;
                // std::cout << "AGENT REPLANNED: " << conflict.agent2 << std::endl;
                // left_node.print();
                if(!prune_left){
                    tree.add_node(left_node);
                    prune_left = false;
                }
            }
        }

        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if(time_spent.count() > config.timelimit)
        {
            std::cout << "TIME LIMIT REACHED, solution not found" << std::endl;
            solution.found = false;
            tree.print();
            break;
        }
        // tree.print();

    } while (tree.get_open_size() > 0);
    solution.paths = get_paths(&node, task.get_agents_size());
    solution.flowtime = node.cost;
    solution.low_level_expanded = low_level_expanded;
    solution.low_level_expansions = low_level_searches;
    solution.high_level_expanded = expanded;
    solution.high_level_generated = int(tree.get_open_size());
    for (auto path : solution.paths)
        solution.makespan = (path.cost > solution.makespan) ? path.cost : solution.makespan;
    solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.check_time = time;
    solution.conflict_time = conflict_time;
    solution.k_conflict_time = k_conflict_time;
    solution.low_level_time = low_level_time;
    solution.rt_time = rt_time;
    solution.ct_time = ct_time;
    for (auto path : solution.paths)
        path.print();
    return solution;
}

std::vector<simplePath> CBS::replace_path(std::vector<simplePath> paths, int agent_id, simplePath path){
    auto replaced_paths = paths;
    replaced_paths.at(agent_id) = path;
    return replaced_paths;
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
    // CBS_Node* current = node;
    // std::vector<simplePath> paths(agents_size);
    // while(current->parent != nullptr){
    //     if(paths.at(current->paths.begin()->agentID).cost < 0)
    //         paths.at(current->paths.begin()->agentID) = *current->paths.begin();
    //     current = current->parent;
    // }
    // for(unsigned int i = 0; i < agents_size; i++){
    //     if(paths.at(i).cost < 0)
    //         paths.at(i) = current->paths.at(i);
    // }
    auto paths = node->paths;
    // std::cout << "paths returned: " << std::endl;
    // for(auto p : paths){
    //     p.print();
    // }
    return paths;
}