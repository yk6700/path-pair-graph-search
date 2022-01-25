#include <iostream>
#include <memory>

#include "ShortestPathHeuristic.h"
#include "../Utils/Definitions.h"
#include "../Utils/IOUtils.h"
#include "../Utils/Logger.h"
#include "../BiCriteria/BOAStar.h"
#include "../BiCriteria/PPA.h"

const std::string resource_path = "src/Example/Resources/";

// Simple example to demonstarte the usage of the algorithm
void single_run_ny_map(size_t source, size_t target, double eps, LoggerPtr logger) {
//    size_t a = 10;
//    size_t c = 4;
//    double d = ((double)a) / ((double)c);
//    std::cout << d << std::endl;
    std::cout << "-----Start NY Map Single Example: SRC=" << source << " DEST=" << target << " EPS=" << eps << "-----" << std::endl;

    // Load files
    size_t graph_size;
    std::vector<Edge> edges;
    if (load_gr_files(resource_path+"USA-road-d.NE.gr", resource_path+"USA-road-t.NE.gr", edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return;
    }

    std::cout << "Graph Size: " << graph_size << std::endl;

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    // Compute heuristic
    std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    std::cout << "Finish Computing Heuristic\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    // Compute BOAStar
    Pair<size_t> bound = Pair<size_t>({3350000, 3350000});
    std::cout << "Start Computing BOAStar" << std::endl;
    SolutionSet boa_solutions;
    BOAStar boa_star(graph, {eps,eps},bound, logger);
    boa_star(source, target, heuristic, boa_solutions, bound, 4);
    std::cout << "Finish Computing BOAStar" << std::endl;

    std::cout << "BOAStar Solutions:" << std::endl;
    size_t solutions_count = 0;
    for (auto solution = boa_solutions.begin(); solution != boa_solutions.end(); solution++) {
        std::cout << ++solutions_count << ". " << *(*solution) << std::endl;
    }
    std::cout << std::endl;

    // Compute PPA
//    std::cout << "Start Computing PPA" << std::endl;
//    SolutionSet ppa_solutions;
//    PPA ppa(graph, {eps,eps}, logger);
//    ppa(source, target, heuristic, ppa_solutions);
//    std::cout << "Finish Computing PPA" << std::endl;
//
//    solutions_count = 0;
//    for (auto solution = ppa_solutions.begin(); solution != ppa_solutions.end(); solution++) {
//        std::cout << ++solutions_count << ". " << *(*solution) << std::endl;
//    }
//    std::cout << std::endl;

    std::cout << "-----End NY Map Single Example-----" << std::endl;
}


void run_queries(std::string map, double eps, LoggerPtr logger, Pair<size_t> bound, int decider = 1) {
    std::cout << "-----Start " << map << " Map Queries Example: BOUND=" << bound << "-----" << std::endl;

    // Load files
    size_t graph_size;
    std::vector<Edge> edges;
    if (load_gr_files(resource_path+"USA-road-d."+map+".gr", resource_path+"USA-road-t."+map+".gr", edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return;
    }

    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(resource_path+"USA-road-"+map+"-queries", queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {
        std::cout << "Started Query: " << ++query_count << "/" << queries.size() << std::endl;
//        if(query_count == 13 && map.compare("NE") == 0 && decider == 4){
//            continue;
//        }
        size_t source = iter->first;
        size_t target = iter->second;

        ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);

        using std::placeholders::_1;
        Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

        SolutionSet boa_solutions;
        BOAStar boa_star(graph, {eps,eps},bound, logger);
        boa_star(source, target, heuristic, boa_solutions, bound, decider);
//        SolutionSet ppa_solutions;
//        PPA ppa(graph, {eps,eps}, logger);
//        ppa(source, target, heuristic, ppa_solutions);
    }

    std::cout << "-----End " << map << " Map Queries Example-----" << std::endl;
}


// Run all queries on all availible maps. The logs outputed from this function are
// used for running the tests
void run_all_queries(void) {
    //decider = {0: more_than_full_cost, 1: cost_min, 2: cost_max, 3: cost_avg, 4: hur_min, 5: hur_max, 6: hur_avg}
    std::string bay_loggers[7] = {"queries_BAY_regular_log.json", "queries_BAY_ps_min_log.json", "queries_BAY_ps_max_log.json", "queries_BAY_ps_avg_log.json", "queries_BAY_phs_min_log.json", "queries_BAY_phs_max_log.json", "queries_BAY_phs_avg_log.json"};
    std::string col_loggers[7] = {"queries_COL_regular_log.json", "queries_COL_ps_min_log.json", "queries_COL_ps_max_log.json", "queries_COL_ps_avg_log.json", "queries_COL_phs_min_log.json", "queries_COL_phs_max_log.json", "queries_COL_phs_avg_log.json"};
    std::string ne_loggers[7] = {"queries_NE_regular_log.json", "queries_NE_ps_min_log.json", "queries_NE_ps_max_log.json", "queries_NE_ps_avg_log.json", "queries_NE_phs_min_log.json", "queries_NE_phs_max_log.json", "queries_NE_phs_avg_log.json"};
    std::string ny_loggers[7] = {"queries_NY_regular_log.json", "queries_NY_ps_min_log.json", "queries_NY_ps_max_log.json", "queries_NY_ps_avg_log.json", "queries_NY_phs_min_log.json", "queries_NY_phs_max_log.json", "queries_NY_phs_avg_log.json"};

    for(int i = 1; i <= 6; i++){
        std::cout << bay_loggers[i] << std::endl;
        LoggerPtr logger_bay = new Logger(bay_loggers[i]);
        run_queries("BAY", 0, logger_bay, Pair<size_t>({1450000,1450000}), i);
        run_queries("BAY", 0, logger_bay, Pair<size_t>({1500000,1500000}), i);
        run_queries("BAY", 0, logger_bay, Pair<size_t>({1550000,1550000}), i);
        run_queries("BAY", 0, logger_bay, Pair<size_t>({2000000,2000000}), i);
        run_queries("BAY", 0, logger_bay, Pair<size_t>({3000000,3000000}), i);
        run_queries("BAY", 0, logger_bay, Pair<size_t>({5000000,5000000}), i);
        delete logger_bay;

        LoggerPtr logger_col = new Logger(col_loggers[i]);
        run_queries("COL", 0, logger_col, Pair<size_t>({4800000,4800000}), i);
        run_queries("COL", 0, logger_col, Pair<size_t>({4900000,4900000}), i);
        run_queries("COL", 0, logger_col, Pair<size_t>({5000000,5000000}), i);
        run_queries("COL", 0, logger_col, Pair<size_t>({5100000,5100000}), i);
        run_queries("COL", 0, logger_col, Pair<size_t>({8000000,8000000}), i);
        run_queries("COL", 0, logger_col, Pair<size_t>({10000000,10000000}), i);
        run_queries("COL", 0, logger_col, Pair<size_t>({12000000,12000000}), i);
        run_queries("COL", 0, logger_col, Pair<size_t>({15000000,15000000}), i);
        delete logger_col;

        LoggerPtr logger_ne = new Logger(ne_loggers[i]);
        run_queries("NE", 0, logger_ne, Pair<size_t>({3350000,3350000}), i);
        run_queries("NE", 0, logger_ne, Pair<size_t>({3400000,3400000}), i);
        run_queries("NE", 0, logger_ne, Pair<size_t>({3500000,3500000}), i);
        run_queries("NE", 0, logger_ne, Pair<size_t>({3600000,3600000}), i);
        run_queries("NE", 0, logger_ne, Pair<size_t>({5000000,5000000}), i);
        run_queries("NE", 0, logger_ne, Pair<size_t>({8000000,8000000}), i);
        run_queries("NE", 0, logger_ne, Pair<size_t>({10000000,10000000}), i);
        run_queries("NE", 0, logger_ne, Pair<size_t>({15000000,15000000}), i);
        delete logger_ne;

        LoggerPtr logger_ny = new Logger(ny_loggers[i]);
        run_queries("NY", 0, logger_ny, Pair<size_t>({1000000,1000000}),i);
        run_queries("NY", 0, logger_ny, Pair<size_t>({1050000,1050000}), i);
        run_queries("NY", 0, logger_ny, Pair<size_t>({1100000,1100000}), i);
        run_queries("NY", 0, logger_ny, Pair<size_t>({1500000,1500000}), i);
        run_queries("NY", 0, logger_ny, Pair<size_t>({2000000,2000000}), i);
        run_queries("NY", 0, logger_ny, Pair<size_t>({3000000,3000000}), i);
        delete logger_ny;
    }



}


int main(void) {

    //    LoggerPtr logger = new Logger("example_log.json");
//    // Easy - Benchmark C_BOA code gets around 20ms
//    size_t easy_source = 1272432;
//    size_t easy_target = 53503;
//    single_run_ny_map(easy_source, easy_target, 0, logger);
//
//    // Hard - Benchmark C_BOA code gets around 2k ms
//    size_t hard_source = 180834;
//    size_t hard_target = 83150;
//    single_run_ny_map(hard_source, hard_target, 0, logger);
//    delete logger;

     try {
         run_all_queries();
     } catch (const std::exception &e) {
         std::cout << "Exception: " << e.what() << std::endl;
     }
    return 0;
}

