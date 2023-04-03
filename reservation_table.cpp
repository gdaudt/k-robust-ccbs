#include "reservation_table.h"

void ReservationTable::clear(){
    ct.clear();
    sit.clear();
    cat.clear();
}


void ReservationTable::insert_path_to_ct(const simplePath& path)
{
    // std::cout << "inside rt k_robust = " << k_robust << std::endl;
    if(path.nodes.empty())
        return;
    auto prev = path.nodes.begin();
    auto curr = path.nodes.begin();
    curr++;
    while(curr != path.nodes.end()){
        if(prev->id != curr->id){
            ct[prev->id].emplace_back(std::max(0.0, prev->g - k_robust), curr->g + k_robust);
            prev = curr;
        }
        curr++;
    }    
    // ct[path.nodes.back().id].emplace_back(path.nodes.back().g, CN_INFINITY);
}

void ReservationTable::print() const{
    std::cout << "Reservation Table: " << std::endl;
    for(auto c:ct){
        std::cout << "Key: " << c.first << std::endl;
        for(auto m:c.second){
            std::cout << "Interval: " << m.first << ", " << m.second << std::endl;
        }
    }
    std::cout << "Safe Intervals Table: " << std::endl;
    for(auto c:sit){
        std::cout << "Key: " << c.first << std::endl;
        for(auto m:c.second){
            std::cout << "Interval: " << std::get<0>(m) << ", " << std::get<1>(m) << " conflict: " << std::get<2>(m) << std::endl;
        }
    }
    std::cout << std::endl;
}

void ReservationTable::print_sit() const{
    std::cout << "Safe Intervals Table: " << std::endl;
    for(auto c:sit){
        std::cout << "Key: " << c.first << std::endl;
        for(auto m:c.second){
            std::cout << "Interval: " << std::get<0>(m) << ", " << std::get<1>(m) << " conflict: " << std::get<2>(m) << std::endl;
        }
    }
}

void ReservationTable::build(const std::vector<simplePath>& paths, const std::list<Constraint>& constraints, int current_agent, int start_location)
{
    //test later with the high-priority agents being the index of the agent (this makes it PBS)
    for(int i = 0; i < (int)paths.size(); i++){
        if(i == current_agent)
            continue;
        insert_path_to_ct(paths[i]);
    }

    insert_constraints_for_starts(paths, current_agent, start_location);
    add_initial_constraints(constraints, current_agent);    
}

void ReservationTable::add_initial_constraints(const std::list<Constraint>& constraints, int current_agent)
{    
    for(auto c:constraints){
        if(c.agent == current_agent){
            ct[c.id1].emplace_back(c.t1-k_robust, c.t1+k_robust);
            ct[c.id2].emplace_back(c.t2-k_robust, c.t2+k_robust);            
        }
    }
}

void ReservationTable::insert_constraint_to_sit(int location, double t_min, double t_max)
{
    if (sit.find(location) == sit.end())
    {
        if (t_min > 0)
        {
			sit[location].emplace_back(0, t_min, 0);
        }
		sit[location].emplace_back(t_max, CN_INFINITY, 0);
        return;
    }
    for (auto it = sit[location].begin(); it != sit[location].end();)
    {
        if (t_min >= std::get<1>(*it))
			++it; 
        else if (t_max <= std::get<0>(*it))
            break;
       else  if (std::get<0>(*it) < t_min && std::get<1>(*it) <= t_max)
        {
            (*it) = std::make_tuple(std::get<0>(*it), t_min, 0);
			++it;
        }
        else if (t_min <= std::get<0>(*it) && t_max < std::get<1>(*it))
        {
            (*it) = std::make_tuple(t_max, std::get<1>(*it), 0);
            break;
        }
        else if (std::get<0>(*it) < t_min && t_max < std::get<1>(*it))
        {
			sit[location].insert(it, std::make_tuple(std::get<0>(*it), t_min, 0));
            (*it) = std::make_tuple(t_max, std::get<1>(*it), 0);
            break;
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            it = sit[location].erase(it);
        }
    }    
}

void ReservationTable::insert_constraints_for_starts(const std::vector<simplePath>& paths, int current_agent, int start_location)
{
    for(int i = 0; i < (int)paths.size(); i++){
        if(paths[i].cost <= 0){
            continue;
        }
        else if(i != current_agent){
            int start = paths[i].nodes.front().id;
            if(start < 0)
                continue;
            for(auto node:paths[i].nodes){
                if(node.id != start_location){
                    ct[node.id].emplace_back(0, node.g + k_robust);
                    break;
                }
            }
        }
    }
}

void ReservationTable::insert_soft_constraints(int location, double t1, double t2){
    
    if(sit.find(location) == sit.end()){
        if(t1 > 0)
            sit[location].emplace_back(0.0, t1, 0);
        sit[location].emplace_back(t2, CN_INFINITY, 0);
        return;
    }
    for(auto it = sit[location].begin(); it != sit[location].end(); it++){
        if (t1 >= std::get<1>(*it))
            continue;
        else if (t2 <= std::get<0>(*it))
            break;
        else if (std::get<2>(*it)) // the interval already has conflicts. No need to update
			continue;
        
        if(std::get<0>(*it) < t1 && std::get<1>(*it) <= t2){
            sit[location].insert(it, std::make_tuple(std::get<0>(*it), t1, 0));            
            (*it) = std::make_tuple(t1, std::get<1>(*it), true);
        }        
        else if(t1 <= std::get<0>(*it) && t2 < std::get<1>(*it)){
            sit[location].insert(it, std::make_tuple(std::get<0>(*it), t2, 1));
            (*it) = std::make_tuple(t2, std::get<1>(*it), 0);
        }
        else if(std::get<0>(*it) < t1 && t2 < std::get<1>(*it)){
            sit[location].insert(it, std::make_tuple(std::get<0>(*it), t1, 0));
            sit[location].insert(it, std::make_tuple(t1, t2, 1));
            (*it) = std::make_tuple(t2, std::get<1>(*it), 0);
        }
        else{
            (*it) = std::make_tuple(std::get<0>(*it), std::get<1>(*it), 1);
        }
    }
    
}

void ReservationTable::insert_path_to_cat(const simplePath& path)
{
    if(path.cost <= 0)
        return;      
}

void ReservationTable::update_sit(int location){
    if(sit.find(location) == sit.end()){
        const auto& it = ct.find(location);
        if(it != ct.end()){
            for(auto c:it->second){
                insert_constraint_to_sit(location, c.first, c.second);
            }    
            ct.erase(it);
        }        
    }
}

std::list<std::tuple<double, double, bool> > ReservationTable::get_safe_intervals(int location, double lower_bound, double upper_bound)
{
    std::list<std::tuple<double, double, bool> > safe_intervals;
    if(lower_bound >= upper_bound)
        return safe_intervals;    
    update_sit(location);
    auto it = sit.find(location);
    if(it == sit.end()){
        safe_intervals.emplace_back(0, CN_INFINITY, false);
        return safe_intervals;
    }

    for(auto interval : it->second){
        if(lower_bound >= std::get<1>(interval))
            continue;        
        else if(upper_bound <= std::get<0>(interval))
            break;        
        else
        {
            safe_intervals.emplace_back(interval);
        }
    }
    return safe_intervals;
}

double ReservationTable::get_holding_time(int location){
    update_sit(location);
    if(sit.find(location) == sit.end())
        return 0;
    double t = std::get<1>(sit[location].back());
    if(t < CN_INFINITY)
        return CN_INFINITY;
    for(auto p = sit[location].rbegin(); p != sit[location].rend(); ++p){
        if(std::get<1>(*p) == t)
            t = std::get<0>(*p);
        else
            break;
    }
    return t;    
}

std::tuple<double, double, bool> ReservationTable::get_first_safe_interval(int location)
{
    update_sit(location);
    auto it = sit.find(location);
    if(it == sit.end())
        return std::make_tuple(0, CN_INFINITY, false);
    return it->second.front();
}

bool ReservationTable::find_safe_interval(std::pair<double, double>& interval, int location, double t_min)
{
    update_sit(location);

    auto it = sit.find(location);
    if(it == sit.end()){
        return t_min == 0;
    }
    for(auto i : it->second){
        if(t_min == std::get<0>(i)){
            interval = std::make_pair(std::get<0>(i), std::get<1>(i));
            return true;
        }
        else if(t_min < std::get<0>(i))
            break;
    }
    return false;
}





