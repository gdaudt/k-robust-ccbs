#pragma once

#include "structs.h"
#include "map.h"
#include "heuristic.h"
#include "reservation_table.h"
#include <unordered_map>
#include <map>
#include <set>
#include <queue>

struct NodeComparator{
    bool operator()(const Node& lhs, const Node& rhs) const
    {
        if(lhs.f == rhs.f)
            return lhs.g > rhs.g;
        return lhs.f > rhs.f;
    }
};

template <class T, class S, class C>
void clear_pq(std::priority_queue<T, S, C>& q){
    q=std::priority_queue<T, S, C>();
}

class SIPP{

    private:
    
    public:
        ~SIPP(){};
        SIPP(){};
        Path findPath(Agent agent, const Map& map, std::list<Constraint> constraints, Heuristic& h_values, ReservationTable& rt);
        Agent agent;
        void find_successors(Node current, const Map& map, std::vector<Node>& successors, Heuristic& h_values, Node goal, ReservationTable& rt);
        void find_successors_sit(Node current, const Map& map, std::vector<Node>& successors, Heuristic& h_values, Node goal, ReservationTable& rt);
        double distance(const Node& a, const Node& b);
        std::vector<Node> construct_path(Node current);
        void make_constraints(std::list<Constraint>& constraints);
        void make_constraints(std::list<Constraint>& constraints, ReservationTable& rt);
        void clear();
        void add_collision_interval(int id, double begin, double end);
        void add_move_constraint(Move move);   
        void add_soft_constraint(int id, double begin, double end);
             
        //priority queue for open list, using a node as key and the node's f value as priority, tiebreaking with g value
        std::priority_queue<Node, std::vector<Node>, NodeComparator> open;
        std::unordered_map<int, Node> closed;
        std::unordered_map<int, std::pair<double, bool>> visited;
        //stores sets of collision_intervals associated with cells
        //key = node.id, pair.first = begin, pair.second = end
        std::unordered_map<int, std::vector<std::pair<double, double>>> collision_intervals;
        //stores vectors of constraints associated with moves
        std::map<std::pair<int, int>, std::vector<Move>> constraints;
        std::unordered_map<int, std::vector<std::pair<double, double> > > soft_constraints;
        double k_robust;
        Path path;       
        
        void print_constraints(){
            if(constraints.size() == 0){
                std::cout << "No constraints" << std::endl;
                return;
            }
            std::cout << "Constraints: " << std::endl;
            for(auto c:constraints){
                std::cout << "Key: nodeid1: " << c.first.first << ", nodeid2: " << c.first.second << std::endl;
                for(auto m:c.second){
                    m.print();
                }
            }
        }

        void print_soft_constraints(){
            if(soft_constraints.size() == 0){
                std::cout << "No soft constraints" << std::endl;
                return;
            }
            std::cout << "Soft Constraints: " << std::endl;
            for(auto c:soft_constraints){
                std::cout << "Key: " << c.first << std::endl;
                for(auto m:c.second){
                    std::cout << "Interval: " << m.first << ", " << m.second << std::endl;
                }
            }
        }

        void print_collision_intervals(){
            if(collision_intervals.size() == 0){
                std::cout << "No collision intervals" << std::endl;
                return;
            }
            std::cout << "Collision Intervals: " << std::endl;
            for(auto c:collision_intervals){
                std::cout << "Key: " << c.first << std::endl;
                for(auto m:c.second){
                    std::cout << "Interval: " << m.first << ", " << m.second << std::endl;
                }
            }
        }

        //copy the open queue to print it
        void print_open(){
            if(open.empty()){
                std::cout << "Open is empty" << std::endl;
                return;
            }
            auto temp = open;
            while(!temp.empty()){
                Node n = temp.top();
                temp.pop();
                n.print();
            }
        }

        //print the closed map
        void print_closed(){
            for(auto c:closed){
                std::cout << "Key: " << c.first << std::endl;
                c.second.print();
            }
        }

        //print the visited map
        void print_visited(){
            for(auto c:visited){
                std::cout << "Key: " << c.first << std::endl;
                std::cout << "Value: " << c.second.first << ", " << c.second.second << std::endl;
            }
        }
};

