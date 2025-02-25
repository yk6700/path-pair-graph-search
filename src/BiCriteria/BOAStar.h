#ifndef BI_CRITERIA_BOA_STAR_H
#define BI_CRITERIA_BOA_STAR_H

#include <vector>
#include "../Utils/Definitions.h"
#include "../Utils/Logger.h"

class BOAStar {
private:
    const AdjacencyMatrix   &adj_matrix;
    Pair<double>            eps;
    const LoggerPtr         logger;
    Pair<size_t>            bounds;

    void start_logging(size_t source, size_t target);
    void end_logging(SolutionSet &solutions, int expended, int generated);

public:
    BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, Pair<size_t> bound, const LoggerPtr logger=nullptr);
    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, Pair<size_t> Bound, int decider=1);
};

#endif //BI_CRITERIA_BOA_STAR_H