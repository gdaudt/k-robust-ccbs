#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <sstream>
#include "tinyxml2.h"
#include "const.h"
#include "structs.h"


class Map{

private:

    std::vector<std::vector<int>> grid;
    std::vector<std::vector<Node>> valid_moves; 
    int height, width, size;
    int  connectedness = 3;
    double agent_size;    
    bool cell_is_obstacle(int i, int j) const;

public:
    Map(double size){ agent_size = size;}
    bool get_map(const char* FileName);
    bool get_grid(const char* FileName);
    int get_size() const { return size; };
    int get_height() const { return height; };
    int get_width() const { return width; };
    int get_i(int id) const { return int(id / width); };
    int get_j(int id) const { return id % width; };
    std::vector<std::vector<Node>> get_valid_moves() const { return valid_moves; };
    std::vector<Node> get_valid_moves(int id) const { return valid_moves[id]; };
    void print() const;
};