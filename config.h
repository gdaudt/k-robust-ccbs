#pragma once
#include "tinyxml2.h"
#include "const.h"
#include <string>
#include <iostream>
#include <sstream>
#include <math.h>

class Config
{
public:
    Config();
    void get_config(const char* fileName);
    double  precision;
    double  agent_size;
    double  timelimit;
};

