#include <memory>
#include <algorithm>

#include "BOAStar.h"

BOAStar::BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, Pair<size_t> bound, const LoggerPtr logger) :
	adj_matrix(adj_matrix), eps(eps), logger(logger), bounds(bound) {}

void BOAStar::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, Pair<size_t> Bound) {
    this->start_logging(source, target);
    //Bound = this->bounds;

    NodePtr node;
    NodePtr next;
    int expended = 0;
    int generated = 0;

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    std::vector<NodePtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<size_t> min_g2(this->adj_matrix.size()+1, MAX_COST);

    // Init open heap
    Node::more_than_full_cost_max more_than; //TODO change queue deciders
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, Pair<size_t>({0,0}), heuristic(source), Bound);
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);
    generated++;

    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();

        // Dominance check
        if ((((1+this->eps[1])*(node->g[1]+node->h[1])) >= min_g2[target]) ||
            (node->g[1] >= min_g2[node->id])) {
            closed.push_back(node);
            //std::cout << "f: " << node->g[1]+node->h[1] << ", min_g2_targer: " << min_g2[target] << ", g1:" << node->g[1] << ", min_g2_next: " << min_g2[node->id] << std::endl;
            continue;
        }

        min_g2[node->id] = node->g[1];

        if (node->id == target) {
            solutions.push_back(node);
            this->end_logging(solutions, expended, generated);
            return;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        //TODO add expand
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            Pair<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            Pair<size_t> next_h = heuristic(next_id);
            //std::cout << "next_g: " << next_g << std::endl;
            //std::cout << "next_h: " << next_h << std::endl;
            //TODO add bound check
            if(next_g[0]+next_h[0] > Bound[0] || next_g[1]+next_h[1] > Bound[1]){
                //std::cout << "f1: " << next_g[0]+next_h[0] << ", f2: " << next_g[1]+next_h[1] << std::endl;
                continue;
            }
            // Dominance check
            if ((((1+this->eps[1])*(next_g[1]+next_h[1])) >= min_g2[target]) ||
                (next_g[1] >= min_g2[next_id])) {
                //std::cout << "f: " << next_g[1]+next_h[1] << ", min_g2_targer: " << min_g2[target] << ", g1:" << next_g[1] << ", min_g2_next: " << min_g2[next_id] << std::endl;
                continue;
            }
            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            next = std::make_shared<Node>(next_id, next_g, next_h, Bound,node);
            //std::cout << "F: " << next->f << std::endl;
            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
            generated++; //TODO add generate
            closed.push_back(node);
        }
        expended++;
    }

    //TODO add expanded and generated nodes nodes
    this->end_logging(solutions, expended, generated);
}


void BOAStar::start_logging(size_t source, size_t target) {
    // All logging is done in JSON format
    std::stringstream start_info_json;
    start_info_json
        << "{\n"
        <<      "\t\"name\": \"BOAStar\",\n"
        <<      "\t\"eps\": " << this->eps << ",\n"
        <<      "\t\"bounds\": " << this->bounds << "\n"
        << "}";

    if (this->logger != nullptr) {
        LOG_START_SEARCH(*this->logger, source, target, start_info_json.str());
    }
}


void BOAStar::end_logging(SolutionSet &solutions, int expended, int generated) {
    // All logging is done in JSON format
    std::stringstream finish_info_json;
    //TODO add expanded and generated nodes nodes
    finish_info_json
            << "{\n"
            <<      "\t\"Expended\": " << expended << ",";
    finish_info_json
            << "\n"
            <<      "\t\"Generated\": " << generated << ",";

    finish_info_json
        << "\n"
        <<      "\t\"solutions\": [";

    size_t solutions_count = 0;
    for (auto solution = solutions.begin(); solution != solutions.end(); ++solution) {
        if (solution != solutions.begin()) {
            finish_info_json << ",";
        }
        finish_info_json << "\n\t\t" << **solution;
        solutions_count++;
    }

    finish_info_json
        <<      "\n\t],\n"
        <<      "\t\"amount_of_solutions\": " << solutions_count << "\n"
        << "}" <<std::endl;

    if (this->logger != nullptr) {
        LOG_FINISH_SEARCH(*(this->logger), finish_info_json.str());
    }
}