#include <limits>
#include <memory>
#include <algorithm>

#include "ShortestPathHeuristic.h"

#include <errno.h>
#include <stdint.h>
// Return best (rounded to nearest - ties to even) float to size_t conversion possible.
// Set ERANGE when `f` was too small, too great.  Return 0, SIZE_MAX.
// Set EDOM when `f` is NAN. Return 0.
/*
 * Converting `float f` to `size_t` with a cast is well defined when
 * mathematically f > -1.0 and f < SIZE_MAX + 1.
 *
 * SIZE_MAX_P1_FLOAT: SIZE_MAX + 1 can overflow integer math, yet since
 * SIZE_MAX is a Mersenne number
 * (http://mathworld.wolfram.com/MersenneNumber.html),
 * (SIZE_MAX/2 + 1) is computable and is exactly half of SIZE_MAX + 1.
 *
 * To return a rounded to nearest size_t,
 * SIZE_MAX + 0.5 <= f also leads to out-of-range. Instead of
 * `f < SIZE_MAX_P1_FLOAT - 0.5f` for upper limit test, use
 * `f - SIZE_MAX_P1_FLOAT < -0.5f`
 * to prevent precision loss near the upper bound.
 *
 * On rare platforms, FLT_MAX < SIZE_MAX+1 and an upper bound check
 * on finite `f` is not needed.
 * Below code does not yet account for that.
 */

// `float` value 1 past SIZE_MAX:
#define SIZE_MAX_P1_FLOAT  ((SIZE_MAX/2 + 1)*2.0f)

size_t float_rounded_to_size_t(float f) {
    // In range?
    if (f >= -0.5f && f - SIZE_MAX_P1_FLOAT < -0.5f) {
        size_t sz = (size_t) f;
        float frac = f - (float) sz;
        if (frac > 0.5f || (frac >= 0.5f && sz % 2)) {
            sz++;
        }
        return sz;
    }
    if (f >= 0.0f) {
        errno = ERANGE;
        return SIZE_MAX;  // f is too great
    }
    if (f < 0.0f) {
        errno = ERANGE;
        return 0;  // f is too negative
    }
    errno = EDOM;
    return 0;  // f is not-a-number
}


ShortestPathHeuristic::ShortestPathHeuristic(size_t source, size_t graph_size, const AdjacencyMatrix &adj_matrix)
    : source(source), all_nodes(graph_size+1, nullptr) {
    size_t i = 0;
    for (auto node_iter = this->all_nodes.begin(); node_iter != this->all_nodes.end(); node_iter++) {
        *node_iter = std::make_shared<Node>(i++, Pair<size_t>({0,0}), Pair<size_t>({MAX_COST,MAX_COST}), Pair<size_t>({1,1}));
    }

    compute(0, adj_matrix);
    compute(1, adj_matrix);
}

//TODO change for different heuristic
Pair<size_t> ShortestPathHeuristic::operator()(size_t node_id) {
    size_t h1 = 0.9 * this->all_nodes[node_id]->h[0];
    size_t h2 = 0.9 * this->all_nodes[node_id]->h[1];
//    std::cout << "h1: " << this->all_nodes[node_id]->h[1] << ", h2: " << this->all_nodes[node_id]->h[2] << std::endl;
//    std::cout << "h1_double: " << h1 << ", h2_double: " << h2 << std::endl;
//    std::cout << "h1_t: " << (size_t)h1 << ", h2_t: " << (size_t)h2 << std::endl;

    return Pair<size_t>{h1, h2};
    //return this->all_nodes[node_id]->h;
}


// Implements Dijkstra shortest path algorithm per cost_idx cost function
void ShortestPathHeuristic::compute(size_t cost_idx, const AdjacencyMatrix &adj_matrix) {
    // Init all heuristics to MAX_COST
    for (auto node_iter = this->all_nodes.begin(); node_iter != this->all_nodes.end(); node_iter++) {
        (*node_iter)->h[cost_idx] = MAX_COST;
    }

    NodePtr node;
    NodePtr next;

    // Init open heap
    Node::more_than_specific_heurisitic_cost more_than(cost_idx);
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    this->all_nodes[this->source]->h[cost_idx] = 0;
    open.push_back(this->all_nodes[this->source]);
    std::push_heap(open.begin(), open.end(), more_than);


    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            next = this->all_nodes[p_edge->target];

            // Dominance check
            if (next->h[cost_idx] <= (node->h[cost_idx]+p_edge->cost[cost_idx])) {
                continue;
            }

            // If not dominated push to queue
            next->h[cost_idx] = node->h[cost_idx] + p_edge->cost[cost_idx];
            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }
}
