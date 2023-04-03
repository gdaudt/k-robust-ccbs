#include <iostream>
#include <fstream>
#include "map.h"
#include "task.h"
#include "heuristic.h"
#include "config.h"
#include "sipp.h"
#include "cbs.h"
#include "xml_logger.h"

int main(int argc, const char *argv[])
{        
    Config config = Config();
    if(argc > 2){        
        config.get_config(argv[3]);
    }    
    Map map = Map(config.agent_size, config.connectedness);
    map.get_map(argv[1]);
    map.print();
    Task task;
    task.get_task(argv[2]);
    task.make_ids(map.get_width());
    task.print_task();
    config.print();
    // Heuristic h;
    // auto agents = task.get_agents();    
    // h.init(map.get_size(), agents.size());
    // for(auto agent:agents)
    //     h.calculate_h_values(map, agent);
    // for(auto a:agents){
    //     a.print();
    //     h.print(a.id);
    // }
    // SIPP sipp;
    // std::list<Constraint> constraints;
    // for (auto agent:agents){
    //     Path path = sipp.findPath(agent, map, constraints, h);
    //  //   path.print();
    // }
    CBS cbs(map, config);
    auto solution = cbs.solve(map, task, config);
    solution.print();
    XML_logger logger;
    logger.get_log(argv[2]);
    logger.write_to_log_summary(solution);
    logger.write_to_log_path(solution, map);
    logger.save_log();
    return 0;
}