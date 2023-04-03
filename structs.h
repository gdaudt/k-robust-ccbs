#pragma once

#include <vector>
#include <math.h>
#include <iostream>
#include <iterator>
#include <chrono>
#include "const.h"
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
using boost::multi_index_container;
using namespace boost::multi_index;

struct Agent{
    double start_i, start_j, goal_i, goal_j;
    int start_id, goal_id;
    int id;
    double size;
    Agent(int s_id = -1, int g_id = -1, int _id = -1)
        :start_id(s_id), goal_id(g_id), id(_id) {}
    void print()
    {
        //ids are coordinates in the map but in a 1D array (i*map_width + j)
        std::cout << "Agent id: " << id << " start_id: " << start_id << " goal_id: " << goal_id << std::endl;
        
    }
};

struct Node{
    //node id is the index represented by i*width+j
    int id;
    double f, g, i, j;
    Node* parent;
    std::pair<double, double> interval;
    int interval_id;
    //constructor receiving all parameters and setting numbers as -1 and nullptr for parent
    Node(int _id = -1, double _f = -1, double _g = -1, double _i = -1, double _j = -1, Node* _parent = nullptr, double begin = -1, double end = -1)
        :id(_id), f(_f), g(_g), i(_i), j(_j), parent(_parent), interval(std::make_pair(begin, end)) {interval_id = 0;}
    // < operator that compares g values
    bool operator<(const Node& rhs) const { return g < rhs.g; }
    void print(){
        std::cout << "Node: " << id << " f: " << f << " g: " << g << " i: " << i << " j: " << j << 
        " interval: " << interval.first << "," << interval.second << std::endl;      
    }
};

struct Path
{
    std::vector<Node> nodes;
    double cost;
    int agentID;
    int expanded;
    Path(std::vector<Node> _nodes = std::vector<Node>(0), double _cost = -1, int _agentID = -1)
        : nodes(_nodes), cost(_cost), agentID(_agentID) {expanded = 0;}
    void print()
    {
        std::cout << "Path cost: " << cost << " agentID: " << agentID << std::endl;
        for (auto node : nodes)
            node.print();
    }
};

struct Constraint{
    int agent;
    double t1, t2; //prohibited to start moving from (i1,j1) to (i2,j2) between t1 and t2
    int id1, id2;
    bool k_robust;
    Constraint(int _agent = -1, double _t1 = -1, double _t2 = -1, int _id1 = -1, int _id2 = -1, bool _k_robust = false)
        :agent(_agent), t1(_t1), t2(_t2), id1(_id1), id2(_id2), k_robust(_k_robust) {};
    friend std::ostream& operator<<(std::ostream& os, const Constraint& c)
    {
        os << "Constraint: agent: " << c.agent << " t1: " << c.t1 << " t2: " << c.t2 << " id1: " << c.id1 << " id2: " << c.id2 << std::endl;
        return os;
    }

    bool operator<(const Constraint& rhs) const { return t1 < rhs.t1; }
    void print(){
        std::cout << "Constraint: agent: " << agent << " t1: " << t1 << " t2: " << t2 << " id1: " << id1 << " id2: " << id2 << std::endl;
    }

    bool operator==(const Constraint& rhs) const { return (agent == rhs.agent && t1 == rhs.t1 && t2 == rhs.t2 && id1 == rhs.id1 && id2 == rhs.id2); }
};

struct simpleNode{
    int id;
    double g;
    simpleNode(int id_=-1, double g_=-1):id(id_),g(g_){}
    simpleNode(const Node &n)
    {
        id = n.id;
        g = n.g;
    }
    void print(){
        std::cout << "simpleNode id: " << id << " g: " << g << std::endl;
    }
};

//Move is an action that an agent can take
//it has a begin time, an end time, and a start and end node
//a Move can be constructed from a Constraint, a Move, or two Nodes
struct Move
{
    double t1, t2; // t2 is required for wait action
    //id is the id of the node that the agent is moving to
    int id1, id2;
    Move(double _t1 = -1, double _t2 = -1, int _id1 = -1, int _id2 = -1)
        : t1(_t1), t2(_t2), id1(_id1), id2(_id2) {}
    Move(const Move& move) : t1(move.t1), t2(move.t2), id1(move.id1), id2(move.id2) {}
    Move(const Constraint& con) : t1(con.t1), t2(con.t2), id1(con.id1), id2(con.id2) {}
    Move(Node a, Node b) : t1(a.g), t2(b.g), id1(a.id), id2(b.id) {}
    Move(simpleNode a, simpleNode b) : t1(a.g), t2(b.g), id1(a.id), id2(b.id) {}

    bool operator <(const Move& other) const
    {
        if     (id1 < other.id1) return true;
        else if(id1 > other.id1) return false;
        else if(id2 < other.id2) return true;
        else return false;
    }
    void print()
    {
        std::cout<<"Move: t1= "<<t1<<" t2= "<<t2<<" id1= "<<id1<<" id2= "<<id2<<std::endl;
    }
};

struct simplePath{
    std::vector<simpleNode> nodes;
    double cost;
    int agentID;
    int expanded;
    simplePath(std::vector<simpleNode> _nodes = std::vector<simpleNode>(0), double _cost = -1, int _agentID = -1)
        : nodes(_nodes), cost(_cost), agentID(_agentID) {expanded = 0;}
    simplePath operator=(const Path &p)
    {
        nodes.clear();
        for (auto n : p.nodes)
            nodes.push_back(simpleNode(n));
        cost = p.cost;
        agentID = p.agentID;
        expanded = p.expanded;
        return *this;
    }
    void print()
    {
        std::cout << "simplePath cost: " << cost << " agentID: " << agentID << std::endl;
        for (auto node : nodes)
            node.print();
    } 
};

class Vector2D {
  public:
    Vector2D(double _i = 0.0, double _j = 0.0):i(_i),j(_j){}
    double i, j;

    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator -() { return Vector2D(-i,-j); }
    inline Vector2D operator /(const double &num) { return Vector2D(i/num, j/num); }
    inline Vector2D operator *(const double &num) { return Vector2D(i*num, j*num); }
    inline double operator *(const Vector2D &vec){ return i*vec.i + j*vec.j; }
    inline void operator +=(const Vector2D &vec) { i += vec.i; j += vec.j; }
    inline void operator -=(const Vector2D &vec) { i -= vec.i; j -= vec.j; }
};

class Point {
public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
};

struct Conflict{
    int agent1, agent2;
    double time; // when building, time is the min(path1.g, path2.g)
    Move move1, move2; //conflict ocurring from move1 going to move2
    double overcost; // overcost is for the h1 heuristic, which we're not currently using
    bool k_robust; // true if the conflict is k_robust
    Conflict(int _agent1 = -1, int _agent2 = -1, double _time = CN_INFINITY, Move _move1 = Move(), Move _move2 = Move(), double _overcost = -1, bool _k_robust = false)
        :agent1(_agent1), agent2(_agent2), time(_time), move1(_move1), move2(_move2), overcost(_overcost), k_robust(_k_robust) {};

    bool operator < (const Conflict& rhs) const { return this->overcost < rhs.overcost; }

    void print(){
        std::cout << "Conflict: agent1: " << agent1 << " agent2: " << agent2 << " time: " << time << " k_robust: " << k_robust << std::endl;
        move1.print();
        move2.print();
    }
};

struct Step
{
    int i;
    int j;
    int id;
    double cost;
    //Step(const Node& node): i(node.i), j(node.j), id(node.id), cost(node.g) {}
    Step(int _i = 0, int _j = 0, int _id = 0, double _cost = -1.0): i(_i), j(_j), id(_id), cost(_cost) {}
    void print()
    {
        std::cout<<"Step: i= "<<i<<" j= "<<j<<" id= "<<id<<" cost= "<<cost<<std::endl;
    }
};

struct CBS_Node{    
    int id;
    double cost;
    double h;
    unsigned int conflicts_num;
    unsigned int total_constraints;
    unsigned int low_level_expanded;
    std::string id_str;
    CBS_Node* parent;
    Constraint constraint;
    std::vector<simplePath> paths;
    std::list<Conflict> conflicts;

    CBS_Node(std::vector<simplePath> _paths = {}, CBS_Node* _parent = nullptr, Constraint _constraint = Constraint(), double _cost = 0, int _conflicts_num = 0, int total_constraints = 0, int low_level_expanded = 0)
        : paths(_paths), parent(_parent), constraint(_constraint), cost(_cost), conflicts_num(_conflicts_num), total_constraints(total_constraints), low_level_expanded(low_level_expanded) {
            low_level_expanded = 0;
            h = 0;
            conflicts.clear();            
        }
    
    ~CBS_Node(){
        parent = nullptr;
        conflicts.clear();
        paths.clear();        
    }

    void print(){
        std::cout<<"CBS_Node: id= "<<id<<" cost= "<<cost<<" h= "<<h<<" conflicts_num= "<<conflicts_num<<" total_constraints= "<<total_constraints<<" low_level_expanded= "<<low_level_expanded;
        if(parent != nullptr)
            std::cout<<" parent_id= "<<parent->id<<std::endl;
        else
            std::cout<<" parent_id= "<<-1<<std::endl;
        std::cout<<"paths: "<<std::endl;
        for(auto path : paths){
            path.print();
        }
        std::cout<<"conflicts: "<<std::endl;
        for(auto conflict : conflicts){
            conflict.print();
        }
        std::cout << "constraints: " << std::endl;
        constraint.print();
    }
};

// multi-index container that stores Node, indexed by ordered_non_unique using the g value
// and hashed unique using the Node.id
typedef multi_index_container<
    Node,
    indexed_by<
        ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, g)>,
        boost::multi_index::hashed_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, id)>
        >
    >
Open_List;

struct cost{};
struct id{};

struct Open_Element{
    CBS_Node* tree_ptr;
    int id;
    double cost;
    unsigned int conflicts_num;
    unsigned int constraints_num;
    Open_Element(CBS_Node* _tree_ptr = nullptr, int _id = -1, double _cost = -1, unsigned int _conflicts_num = 0, unsigned int _constraints_num = 0)
        : tree_ptr(_tree_ptr), id(_id), cost(_cost), conflicts_num(_conflicts_num), constraints_num(_constraints_num) {}
    ~Open_Element(){
        tree_ptr = nullptr;
    }
};

typedef multi_index_container<
    Open_Element,
    indexed_by<
        ordered_non_unique<composite_key<Open_Element, BOOST_MULTI_INDEX_MEMBER(Open_Element, double, cost), BOOST_MULTI_INDEX_MEMBER(Open_Element, unsigned int, conflicts_num), BOOST_MULTI_INDEX_MEMBER(Open_Element, unsigned int, constraints_num)>,
        composite_key_compare<std::less<double>, std::less<int>, std::greater<int>>>,
        hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Open_Element, int, id)>
    >
> CT_container;

class CBS_Tree{
    //list to store nodes unordered
    std::list<CBS_Node> tree;
    //container stores cbs_nodes ordered by <cost, <conflicts_num, >constraints_number
    CT_container open_list;
    int open_size;
    std::set<int> closed;
    public:
        CBS_Tree(){
            open_size = 0;
        }
        ~CBS_Tree(){
            tree.clear();
            open_list.clear();
            closed.clear();
        }
        unsigned int size(){
            return tree.size();
        }
        int get_open_size(){
            return open_size;
        }

        int get_size(){
            return tree.size();
        }

        void add_node(CBS_Node node){
            tree.push_back(node);
            open_list.insert(Open_Element(&tree.back(), node.id, node.cost, node.conflicts_num, node.total_constraints));
            open_size++;
        }
        void print(){
            std::cout<<"CBS_Tree: "<<std::endl;
            for(auto node : tree){
                node.print();
            }
            std::cout<< "closed: " << std::endl;
            for(auto node : closed){
                std::cout<<node<<" ";
            }
        }
        CBS_Node* get_best_node(){
            if(open_size == 0){
                return nullptr;
            }
            auto it = open_list.begin();
            CBS_Node* best_node = it->tree_ptr;
            open_list.get<0>().erase(it);
            open_size--;
            closed.insert(best_node->id);
            return best_node;
        }
        std::vector<simplePath> get_paths(CBS_Node node, int size){
            std::vector<simplePath> paths(size);
            while(node.parent != nullptr){
                if(paths.at(node.paths.begin()->agentID).nodes.empty())
                    paths.at(node.paths.begin()->agentID) = *node.paths.begin();
                node = *node.parent;
            }
            for(unsigned int i = 0; i < paths.size(); i++){
                if(paths.at(i).nodes.empty()){
                    paths.at(i) = node.paths.at(i);
                }
            }
            return paths;            
        }
        CBS_Node* get_front(){
            open_size--;        
            auto pointer = open_list.get<0>().begin()->tree_ptr;
            open_list.get<0>().erase(open_list.get<0>().begin());
            return pointer;        
        }
};

struct Solution{
    bool found;
    double flowtime;
    double makespan;
    double check_time;
    double init_cost;
    int constraints_number;
    int max_constraints;
    int high_level_generated;
    int high_level_expanded;
    int low_level_expansions;
    double low_level_expanded;
    std::chrono::duration<double> time;
    std::chrono::duration<double> init_time;
    std::chrono::duration<double> conflict_time;
    std::chrono::duration<double> k_conflict_time;
    std::chrono::duration<double> rt_time;
    std::chrono::duration<double> low_level_time;
    std::chrono::duration<double> ct_time;
    std::vector<simplePath> paths;
    Solution(double _flowtime = -1, double _makespan = -1, std::vector<simplePath> _paths = {})
        : flowtime(_flowtime), makespan(_makespan), paths(_paths) { init_cost = -1; constraints_number = 0; low_level_expanded = 0; low_level_expansions = 0; max_constraints = 0;}
    ~Solution() { paths.clear(); found = false;}
    void print(){
        std::cout << "Flowtime: " << flowtime << std::endl;
        std::cout << "Makespan: " << makespan << std::endl;
        std::cout << "Constraints: " << constraints_number << std::endl;
        std::cout << "Max Constraints: " << max_constraints << std::endl;
        std::cout << "Init Cost: " << init_cost << std::endl;
        std::cout << "High Level Expanded: " << high_level_expanded << std::endl;
        std::cout << "High Level Generated: " << high_level_generated << std::endl;
        std::cout << "Low Level Expanded: " << low_level_expanded << std::endl;
        std::cout << "Low Level Expansions: " << low_level_expansions << std::endl;
        std::cout << "Check Time: " << check_time << std::endl;
        std::cout << "Init Time: " << init_time.count() << std::endl;
        std::cout << "Total Time: " << time.count() << std::endl;
        std::cout << "Conflict Time: " << conflict_time.count() << std::endl;
        std::cout << "K Conflict Time: " << k_conflict_time.count() << std::endl;
        std::cout << "RT Time: " << rt_time.count() << std::endl;
        std::cout << "Low Level Time: " << low_level_time.count() << std::endl;
        std::cout << "Constraint Time: " << ct_time.count() << std::endl;
    }
};

