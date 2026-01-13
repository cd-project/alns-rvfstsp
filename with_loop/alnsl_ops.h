#ifndef ALNS_OPSL_H
#define ALNS_OPSL_H
#include <random>

#include "../parameters.h"
#include "../with_loop/solutionl.h"
#include <memory>
#include "../instance.h"

class ALNSL {
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


    ALNSL(const uint_fast32_t seed,
               const std::shared_ptr<Instance> &instance,
               const std::shared_ptr<Parameter> &param) : instance(instance), param(param) {
        this_seed = seed;
        mt = std::mt19937(seed);

    }

    std::shared_ptr<SolutionL> Solve();


    /*
     * Operators
    */

    // Operator selector
    int Select(const std::vector<double> &weights);
    // Removal operators
    void RandomRemove(std::shared_ptr<SolutionL>& solution);
    void WorstTruckNodeRemove(std::shared_ptr<SolutionL>& solution);
    void WorstSortieRemove(std::shared_ptr<SolutionL>& solution);
    void RandomDroneRemove(std::shared_ptr<SolutionL> &solution);
    void RandomSortieRemove(std::shared_ptr<SolutionL> &solution);
    void RandomTruckRemove(std::shared_ptr<SolutionL>& solution);
    void RandomStringRemove(std::shared_ptr<SolutionL>& solution);


    // Insertion operators
    void BestCostInsert(std::shared_ptr<SolutionL> &solution);
    void RandomSequenceBestCostInsert(std::shared_ptr<SolutionL> &solution);
    void RandomMixRandomAndBest(std::shared_ptr<SolutionL>& solution);

    // Local Search
    void TruckLocalSearch(std::shared_ptr<SolutionL> &solution) ;
    void SortieLocalSearch(std::shared_ptr<SolutionL> &solution);
    // Revisit operators
    void Revisit(std::shared_ptr<SolutionL> &solution);
    void LoopSearch(std::shared_ptr<SolutionL> &solution);




};
#endif //ALNS_OPSL_H
