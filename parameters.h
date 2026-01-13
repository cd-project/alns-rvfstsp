//
// Created by cuong on 5/9/24.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H
class Parameter {
public:
    // CPU parameters
    double n_thread = 16;

    // ALNS parameters
    int number_of_run = 10;
    int max_iteration = 250000;
    int max_iteration_without_improvements = 100000;
    double maximum_remove_rate = 0.77;
    double minimum_remove_rate = 0.2;
    int max_remove = 40;
    double cost_power = 0.00;
    double cooling_rate = 0.99948;
    double temperature_control = 0.328;
    double insert_operator_1 = 1;
    double insert_operator_2 = 1;
    double insert_operator_3 = 1;
    double insert_operator_4 = 1;
    double insert_operator_5 = 1;
    double regret_insert_2 = 1;
    double profit_regret_2 = 4;
    double regret_insert_3 = 3;
    double profit_regret_3 = 3;
    double regret_insert_4 = 1;
    double profit_regret_4 = 1;
    double remove_operator_1 = 1;
    double remove_operator_2 = 1;
    double remove_operator_3 = 1;
    double remove_operator_4 = 1;
    double remove_operator_5 = 1;
    double remove_operator_6 = 1;
    double remove_operator_7 = 1;
    double best_score = 5;
    double improved_score = 2;
    double accepted_score = 1;
    double rejected_score = 0.5;
    double decay_insert = 0.8;
    double decay_remove = 0.8;
    int profit_insert_random_power = 5;
    int cost_remove_random_power = 3;
    int profit_over_cost_remove_random_power = 6;
    int profit_remove_random_power = 11;
    int proximity_insert_random_power = 4;
    int relate_remove_random_power = 10;
    double noise_factor = 0.011;
    double alpha = 18;
    double beta = 1;
};
#endif //PARAMETERS_H
