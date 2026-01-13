//
// Created by cuong on 5/9/24.
//

#ifndef ALNS_OPS_H
#define ALNS_OPS_H
#include <random>

#include "../parameters.h"
#include "solution.h"
#include <memory>
#include "../instance.h"

class ALNS {
public:
    std::mt19937 mt{std::random_device()()};
    std::shared_ptr<Instance> instance;
    std::shared_ptr<Parameter> param;
    unsigned long this_seed;
    // std::shared_ptr<Utilities> utils;
    // std::uniform_real_distribution<double> noise_dist;
    std::vector<int> count_used;
    std::vector<int> count_improved;
    std::vector<int> count_best;
    double t_insert = 0;
    double t_random_remove = 0;
    double t_random_drone_remove = 0;
    double t_wn_remove = 0;
    double t_r_truck_remove = 0;
    double t_string_remove = 0;
    int local_search_swap_improve = {0};
    double initial_objective;
    int lazy_insert_iter = 0;
    int iter_best;


    ALNS(const uint_fast32_t seed,
               const std::shared_ptr<Instance> &instance,
               const std::shared_ptr<Parameter> &param) : instance(instance), param(param) {
        this_seed = seed;
        mt = std::mt19937(seed);

    }

    std::shared_ptr<Solution> Solve();


    /*
     * Operators
    */

    // Operator selector
    int Select(const std::vector<double> &weights);
    // Removal operators
    void RandomRemove(std::shared_ptr<Solution> &solution, int remove_size);
    void WorstTruckNodeRemove(std::shared_ptr<Solution> &solution, int remove_size);
    void WorstSortieRemove(std::shared_ptr<Solution> &solution, int remove_size);
    void RandomDroneRemove(std::shared_ptr<Solution> &solution);
    void RandomSortieRemove(std::shared_ptr<Solution> &solution);
    void RandomTruckRemove(std::shared_ptr<Solution> &solution, int remove_size);
    void RandomStringRemove(std::shared_ptr<Solution> &solution, int remove_size);


    // Insertion operators
    void BestCostInsert(std::shared_ptr<Solution> &solution);
    void RandomSequenceBestCostInsert(std::shared_ptr<Solution> &solution);
    void TruckFirstSortieSecond(std::shared_ptr<Solution> &solution);
    void TFSCGPT(std::shared_ptr<Solution> &solution);
    void SortieFirstTruckSecond(std::shared_ptr<Solution> &solution);

    // Local Search
    void TruckLocalSearch(std::shared_ptr<Solution> &solution) ;
    void SortieLocalSearch(std::shared_ptr<Solution> &solution);
    // Revisit operators
    static void Revisit(std::shared_ptr<Solution> &solution);




};
#endif //ALNS_OPS_H
