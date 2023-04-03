#pragma once
#include "structs.h"
#include "map.h"
#include <iostream>
#include <unordered_map>
#include <tuple>

class ReservationTable
{
    public:
        double k_robust;
        int num_agents;
        double runtime;

        void clear();
        void copy(const ReservationTable& other) { ct = other.ct; }
        void build(const std::vector<simplePath>& paths, const std::list<Constraint>& constraints, int current_agent, int start_location);
        void print() const;
        void print_sit() const;
        std::unordered_map<size_t, std::list<std::pair<double, double> > > get_ct() const { return ct; }
        //default constructor
        ReservationTable() : map(Map(0, 0)) { };
        ReservationTable(const Map& map) : map(map) { };
        ReservationTable(const Map& map, double k_robust) : map(map), k_robust(k_robust) { };

        std::list<std::tuple<double, double, bool> > get_safe_intervals(int location, double t1, double t2);
        std::tuple<double, double, bool> get_first_safe_interval(int location);        
        bool find_safe_interval(std::pair<double, double>& interval, int location, double t_min);
        double get_holding_time(int location);


    private:
        const Map& map;
        std::unordered_map<size_t, std::list<std::pair<double, double> > > ct;
        std::unordered_map<size_t, std::list<std::tuple<double, double, bool> > > sit;
        std::vector<std::vector<bool> > cat; // (timestep, location) -> has a conflict or not
        void add_initial_constraints(const std::list<Constraint>& constraints, int current_agent);
        void insert_constraints_for_starts(const std::vector<simplePath>& paths, int current_agent, int start_position);
        void insert_soft_constraints(int location, double t1, double t2);
        void update_sit(int location);
        void insert_constraint_to_sit(int location, double t1, double t2);
        void insert_path_to_ct(const simplePath& path);
        void insert_path_to_cat(const simplePath& path);

};