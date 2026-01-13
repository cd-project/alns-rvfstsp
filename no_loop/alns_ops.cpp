#include "../no_loop/alns_ops.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include "../no_loop/solution.h"

// inline bool exist(const std::vector<int>& vec, int element) {
//     // Use std::find to search for the element in the vector
//     return std::ranges::find(vec, element) != vec.end();
// }

std::shared_ptr<Solution> ALNS::Solve() {
    int iter_with_best_solution = 0;
    int stopped_at_iter = 0;
    bool non_im_trigger = false;
    std::vector<double> insert_weights = {
        param->insert_operator_1,
        param->insert_operator_2,
        // param->insert_operator_3,
    };

    std::vector<double> remove_weights = {
        param->remove_operator_1,
        param->remove_operator_2,
        param->remove_operator_3,
        param->remove_operator_4,
        param->remove_operator_5,
        param->remove_operator_6,
        param->remove_operator_7,
    };

    count_used.resize(insert_weights.size() + remove_weights.size());
    count_improved.resize(insert_weights.size() + remove_weights.size());
    count_best.resize(insert_weights.size() + remove_weights.size());

    auto initial_solution = std::make_shared<Solution>(instance);
    RandomSequenceBestCostInsert(initial_solution);
    initial_solution->calculate_objective();
    initial_objective = initial_solution->objective;

    auto best_solution = initial_solution;
    const double initial_temp = -param->temperature_control * best_solution->objective / log(0.5);
    double temperature = initial_temp;
    std::uniform_real_distribution<double> double_dist(0, 1);
    int no_improve_iter_count = 0;
    int count_accept_worse = 0;
    std::vector<int> count_lazy_remove(remove_weights.size());
    for (int it = 0; it < param->max_iteration; ++it) {
        auto new_solution = initial_solution->copy();
        std::uniform_int_distribution int_dist_truck_remove(
        static_cast<int>(floor(new_solution->truck_route.size() / 5)), static_cast<int>(floor(new_solution->truck_route.size() / 2.5)));
        const int truck_remove_size = int_dist_truck_remove(mt);
        const int select_insert = Select(insert_weights);
        const int select_remove = Select(remove_weights);

        count_used[select_insert]++;
        count_used[select_remove + insert_weights.size()]++;

        switch (select_remove) {
        case 0: {
            RandomStringRemove(new_solution, truck_remove_size);
            break;
        }
        case 1: {
            RandomSortieRemove(new_solution);
            break;
        }
        case 2: {
            RandomRemove(new_solution, truck_remove_size);
            break;
        }
        case 3: {
            RandomDroneRemove(new_solution);
            break;
        }
        case 4: {
            WorstTruckNodeRemove(new_solution, truck_remove_size);
            break;
        }
        case 5: {
            RandomTruckRemove(new_solution, truck_remove_size);
            break;
        }
        case 6: {
            WorstSortieRemove(new_solution, truck_remove_size);
            break;
        }
        default: {
            break;
        }
        }
        std::vector<int> to_insert;
        to_insert.reserve(instance->C.size()); // Reserve space to avoid multiple allocations
        for (int i : instance->C) {
            if (!new_solution->visited[i]) {
                to_insert.push_back(i);
            }
        }
        if (to_insert.size() <= 2) {
            count_lazy_remove[select_remove]++;
        }
        switch (select_insert) {
        case 0: {
            // auto start_insert = std::chrono::high_resolution_clock::now();
            RandomSequenceBestCostInsert(new_solution);
            // auto end_insert = std::chrono::high_resolution_clock::now(); // End timing
            // std::chrono::duration<double> elapsed_insert = end_insert - start_insert;
            // t_insert += elapsed_insert.count();
            break;
        }
        case 1: {
            BestCostInsert(new_solution);
            break;
        } case 2: {
            SortieFirstTruckSecond(new_solution);
        }
        default: {
            break;
        }
        }

        // new_//solution->invalid_occurrence();
        Revisit(new_solution);
        new_solution->calculate_objective();
        if (new_solution->objective >= best_solution->objective) {
            no_improve_iter_count++;
        }
        if (no_improve_iter_count >= param->max_iteration_without_improvements) {
            non_im_trigger = true;
            break;
        }
        int update_case = -1;

        if (const double prob = exp((initial_solution->objective - new_solution->objective) / temperature);
           double_dist(mt) < prob || new_solution->objective <= initial_solution->objective) {
            if (new_solution->objective < initial_solution->objective) {
                count_improved[select_insert]++;
                count_improved[select_remove + insert_weights.size()]++;
                update_case = 1;
            }
            else {
                update_case = 2;
            } // TruckLocalSearch(new_solution);
            initial_solution = new_solution;

            if (initial_solution->objective < best_solution->objective) {
                update_case = 3;
                best_solution = initial_solution;
                count_best[select_insert]++;
                count_best[select_remove + insert_weights.size()]++;
                iter_with_best_solution = it;
                no_improve_iter_count = 0;
            }
        }
        else {
            update_case = 4;
        }
        switch (update_case) {
        case 1: {
            insert_weights[select_insert] = param->decay_insert * insert_weights[select_insert] + (1 - param->
                    decay_insert) * param->
                improved_score;
            remove_weights[select_remove] = param->decay_remove * remove_weights[select_remove] + (1 - param->
                    decay_remove) * param->
                improved_score;
            break;
        }

        case 2: {
            insert_weights[select_insert] = param->decay_insert * insert_weights[select_insert] + (1 - param->
                    decay_insert) * param->
                accepted_score;
            remove_weights[select_remove] = param->decay_remove * remove_weights[select_remove] + (1 - param->
                    decay_remove) * param->
                accepted_score;
            break;
        }

        case 3: {
            insert_weights[select_insert] = param->decay_insert * insert_weights[select_insert] + (1 - param->
                    decay_insert) * param->
                best_score;
            remove_weights[select_remove] = param->decay_remove * remove_weights[select_remove] + (1 - param->
                    decay_remove) * param->
                best_score;
            break;
        }
        case 4: {
            insert_weights[select_insert] = param->decay_insert * insert_weights[select_insert] + (1 - param->
                    decay_insert) * param->
                rejected_score;
            remove_weights[select_remove] = param->decay_remove * remove_weights[select_remove] + (1 - param->
                    decay_remove) * param->
                rejected_score;
            break;
        }
        default: break;
        }
        temperature *= param->cooling_rate;
    }
    iter_best = iter_with_best_solution;
    std::cout << "------------------------------------------------------------------------" << std::endl;
    std::cout << "Best solution found at iteration " << iter_with_best_solution << ": " << best_solution->objective <<
        std::endl;
    // // std::cout << "Number of iterations with lazy insert: " << lazy_insert_iter << std::endl;
    // // std::cout << "Lazy remove summary: " << std::endl;
    // // for (int i = 0; i < count_lazy_remove.size(); ++i) {
    // //     std::cout << "Remove " << i << " lazy count: " << count_lazy_remove[i] << std::endl;
    // // }
    // std::cout << "weight summary: " << std::endl;
    // std::cout << "insert: " << std::endl;
    // for (int i = 0; i < insert_weights.size(); ++i) {
    //     std::cout << "Insert " << i << ": " << insert_weights[i] << std::endl;
    // }
    // std::cout << "remove: " << std::endl;
    // for (int i = 0; i < remove_weights.size(); ++i) {
    //     std::cout << "remove " << i << ": " << remove_weights[i] << std::endl;
    // }
    // std::cout << "------------------------------------------------------------------------" << std::endl;

    // double rate = 0;
    // for (const int d : best_solution->served_by_drone) {
    //     const double this_rate = best_solution->sortie_stages[d].truck_trip >= best_solution->sortie_stages[d].
    //                              drone_trip
    //                                  ? best_solution->sortie_stages[d].truck_trip / best_solution->sortie_stages[d].
    //                                  drone_trip
    //                                  : best_solution->sortie_stages[d].drone_trip / best_solution->sortie_stages[d].
    //                                  truck_trip;
    //     std::cout << "Rate of sortie " << d << ": " << this_rate << ", length: " << std::max(
    //         best_solution->sortie_stages[d].drone_trip, best_solution->sortie_stages[d].
    //         truck_trip) << std::endl;
    //     rate += this_rate;
    // }
    // const double num_sbd = best_solution->served_by_drone.size() * 1.00;
    // std::cout << "Rate diff: " << fabs(rate /= num_sbd - 1) << std::endl;
    // std::cout << "Operators performance: " << std::endl;
    // std::cout << "------------------------------------------------------------------------" << std::endl;
    // std::cout << "BEST OPS:" << std::endl;
    // std::cout << "Insert: " << std::endl;
    // for (int i = 0; i < insert_weights.size(); ++i) {
    //     std::cout << "Insert " << i << ": " << count_best[i] << std::endl;
    // }
    // std::cout << "Remove: " << std::endl;
    // for (int i = 0; i < remove_weights.size(); ++i) {
    //     std::cout << "Remove " << i << ": " << count_best[i + insert_weights.size()] << std::endl;
    // }
    // std::cout << "------------------------------------------------------------------------" << std::endl;
    // std::cout << "Improved OPS:" << std::endl;
    // std::cout << "Insert: " << std::endl;
    // for (int i = 0; i < insert_weights.size(); ++i) {
    //     std::cout << "Insert " << i << ": " << count_improved[i] << std::endl;
    // }
    // std::cout << "Remove: " << std::endl;
    // for (int i = 0; i < remove_weights.size(); ++i) {
    //     std::cout << "Remove " << i << ": " << count_improved[i + insert_weights.size()] << std::endl;
    // }

    return best_solution;
}

int ALNS::Select(const std::vector<double>& weights) {
    auto zero_one_double = std::uniform_real_distribution<double>(0, 1);
    const double r = zero_one_double(mt);

    std::vector<double> sum(weights.size());
    sum[0] = weights[0];
    for (int i = 1; i < weights.size(); ++i) {
        sum[i] = sum[i - 1] + weights[i];
    }
    for (int i = weights.size() - 2; i >= 0; --i) {
        if (r > sum[i] / sum[weights.size() - 1]) return i + 1;
    }

    return 0;
}

void ALNS::RandomRemove(std::shared_ptr<Solution>& solution, const int remove_size) {
    std::vector<int> cus;
    for (int i = 1; i < solution->visited.size(); i++) {
        if (solution->visited[i]) {
            cus.push_back(i);
        }
    }
    // const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine rng(seed);

    // Shuffle the vector
    std::ranges::shuffle(cus.begin(), cus.end(), mt);

    for (int i = 0; i < remove_size; i++) {
        solution->remove_customer(cus[i]);
    }
}

void ALNS::WorstTruckNodeRemove(std::shared_ptr<Solution>& solution, const int remove_size) {
    std::vector<int> ignored;
    for (int i = 0; i < remove_size; i++) {
        // find the costliest index for this iteration
        double max_cost = -1e9;
        int max_cost_index = -1;
        for (int index = 1; index < solution->truck_route.size() - 1; index++) {
            if (const double this_index_cost = instance->tau[solution->truck_route[index - 1]][solution->truck_route[
                        index]]
                    + instance->tau[solution->truck_route[index]][solution->truck_route[index + 1]]
                    - instance->tau[solution->truck_route[index - 1]][solution->truck_route[index + 1]];
                this_index_cost > max_cost
            ) {
                max_cost = this_index_cost;
                max_cost_index = index;
            }
        }
        // if (const bool removed = solution->remove_truck_index(max_cost_index); !removed) {
        //     return;
        // }
        // remove this index.
        if (const bool removed = solution->remove_truck_index(max_cost_index); !removed) {
            ignored.push_back(max_cost_index);
        }
        else {
            ignored.clear();
        }
    }
}

void ALNS::RandomTruckRemove(std::shared_ptr<Solution>& solution, const int remove_size) {
    // solution->sortie_invalid_occurrence();
    std::vector<int> ignored_index;
    for (int i = 0; i < remove_size; i++) {
        // randomly select a index.
        std::vector<int> could_be_removed;
        for (int index = 1; index < solution->truck_route.size() - 1; index++) {
            if (!exist(ignored_index, index)) {
                could_be_removed.push_back(index);
            }
        }

        std::ranges::shuffle(could_be_removed.begin(), could_be_removed.end(), mt);
        for (const int index : could_be_removed) {
            if (const bool removed = solution->remove_truck_index(index); !removed) {
                continue;
            }
            // xoa duoc roi. go to end of loop.
            // solution->sortie_invalid_occurrence();
            goto eol;
        }

    eol:;
    }
}

void ALNS::RandomDroneRemove(std::shared_ptr<Solution>& solution) {
    if (!solution->served_by_drone.empty()) {
        std::uniform_int_distribution<int> to_remove_dist(1, solution->served_by_drone.size());

        // const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        // std::default_random_engine rng(seed);
        const int to_remove = to_remove_dist(mt);
        // Shuffle the vector

        // shuffle.
        std::ranges::shuffle(solution->served_by_drone, mt);
        std::vector<int> to_be_removed(to_remove);
        std::copy_n(solution->served_by_drone.begin(), to_remove, to_be_removed.begin());

        for (const int c : to_be_removed) {
            solution->remove_drone_customer(c);
        }
    }
}

void ALNS::RandomStringRemove(std::shared_ptr<Solution>& solution, const int remove_size) {
    std::uniform_int_distribution<int> start_index_distribution(1, solution->truck_route.size() - 2);
    // const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine rng(seed);

    const int index = start_index_distribution(mt);
    // got the start index
    // perform the string removal
    solution->remove_string(index, remove_size);
}

void ALNS::WorstSortieRemove(std::shared_ptr<Solution>& solution, const int remove_size) {
    int num_node_removed = 0;
    std::vector<int> ignored;
    for (int count = 0; count < solution->served_by_drone.size(); count++) {
        // solution->sortie_feasibility_check();
        // in the current solution, what is the most rate-bugged sortie?
        int highest_rate_sortie = -1;
        double highest_rate = 0;
        for (const int d : solution->served_by_drone) {
            if (exist(ignored, d)) {
                continue;
            }
            const double rate = solution->sortie_stages[d].truck_trip >= solution->sortie_stages[d].drone_trip
                                    ? solution->sortie_stages[d].truck_trip / solution->sortie_stages[d].drone_trip
                                    : solution->sortie_stages[d].drone_trip / solution->sortie_stages[d].truck_trip;

            if (rate > highest_rate) {
                highest_rate = rate;
                highest_rate_sortie = d;
            }
        }
        if (highest_rate - 1 < 1e-2 || highest_rate_sortie == -1) {
            return;
        }

        if (const int removed = solution->remove_highest_rate_sortie(highest_rate_sortie); removed > 0) {
            num_node_removed += removed;
            if (num_node_removed >= remove_size) {
                return;
            }
            ignored.clear();
        }
        else {
            ignored.push_back(highest_rate_sortie);
        }
    }
}

void ALNS::TruckFirstSortieSecond(std::shared_ptr<Solution>& solution) {
    std::cout << "Solution before:" << std::endl;
    std::vector<int> to_insert;
    for (int i : instance->C) {
        if (!solution->visited[i]) {
            to_insert.push_back(i);
        }
    }
    // const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine rng(seed);

    // Shuffle the vector
    std::ranges::shuffle(to_insert.begin(), to_insert.end(), mt);
    for (int& c : to_insert) {
        // insert all customer into truck route.
        // even flat will be inserted into truck route.
        if (solution->truck_route.size() == 2) {
            solution->truck_route.insert(solution->truck_route.begin() + 1, c);
            solution->visited[c] = true;
            continue;
        }

        // tính đường thêm vào truck
        int best_truck_index = -1;
        double best_truck_cost = 1e9;
        int insert_into_truck_route_of_sortie = -1;
        double new_truck_cost_for_sortie = -1;

        for (int index = 1; index < solution->truck_route.size(); index++) {
            // check if this index is in a sortie?
            bool index_in_sortie = false;

            for (const int d : solution->served_by_drone) {
                if (auto& ss = solution->sortie_stages[d]; ss.start_index < index && index <= ss.end_index) {
                    index_in_sortie = true;

                    double new_truck_cost = ss.truck_trip - instance->tau[solution->truck_route[index - 1]][solution->
                            truck_route[index]] +
                        instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->truck_route[
                            index]];

                    if (new_truck_cost > instance->e - instance->sr) {
                        break;
                    }

                    double increased_cost = 0;
                    if (new_truck_cost > ss.drone_trip) {
                        increased_cost = new_truck_cost - std::min(ss.truck_trip,
                                                                   ss.drone_trip);
                    }
                    if (increased_cost < best_truck_cost) {
                        best_truck_index = index;
                        best_truck_cost = increased_cost;
                        insert_into_truck_route_of_sortie = d;
                        new_truck_cost_for_sortie = new_truck_cost;
                    }
                }
            }

            if (!index_in_sortie) {
                // tinh increased cost
                double increased_cost = instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->
                        truck_route[index]] -
                    instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];

                if (increased_cost < best_truck_cost) {
                    best_truck_index = index;
                    best_truck_cost = increased_cost;
                    insert_into_truck_route_of_sortie = -1;
                }
            }
        }
        if (best_truck_index > 0) {
            // do the insertion.
            // have to calculate new sortie length too if insert into sortie.
            solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
            solution->visited[c] = true;
            // update sorties với index phía sau best_truck_index
            for (const int d : solution->served_by_drone) {
                if (solution->sortie_stages[d].start_index >= best_truck_index) {
                    solution->sortie_stages[d].start_index++;
                }
                if (solution->sortie_stages[d].end_index >= best_truck_index) {
                    solution->sortie_stages[d].end_index++;
                }
            }

            // done. we do continue this loop.
            if (insert_into_truck_route_of_sortie > -1) {
                solution->sortie_stages[insert_into_truck_route_of_sortie].truck_trip = new_truck_cost_for_sortie;
            }
        }
        if (best_truck_index < 0) {
            // // đè thằng nào min cost xuống.
            // int which_sortie_to_flat = -1;
            // int best_index_for_flat = -1;
            // double best_overall = 1e9;
            // for (const int d : solution->served_by_drone) {
            //     // nếu đè d xuống và đè vào đâu đó min cost trong khoảng của nó.
            //     int best_flat_index = -1;
            //     int best_truck_index_this_sortie = -1;
            //     double this_best_flat_cost = 1e9;
            //
            //     for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].
            //          end_index; index++) {
            //         // thêm vào index, cost sẽ là? phần thêm = index - 1 -> flat + flat-> index
            //         if (const double increased_cost = instance->tau[solution->truck_route[index - 1]][d] + instance->tau
            //                 [d][
            //                     solution->truck_route[index]]
            //                 - instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];
            //             increased_cost
            //             < this_best_flat_cost) {
            //             this_best_flat_cost = increased_cost;
            //             best_flat_index = index;
            //         }
            //     }
            //     std::vector<int> copy_route;
            //     for (int i = solution->sortie_stages[d].start_index; i <= solution->sortie_stages[d].end_index; i++) {
            //         copy_route.push_back(solution->truck_route[i]);
            //     }
            //     copy_route.insert(copy_route.begin() + best_flat_index - solution->sortie_stages[d].start_index, d);
            //
            //     // thêm khách hàng c vào truck hay drone vào cái copy route này?
            //     for (int i = 1; i < copy_route.size(); i++) {
            //         // add cost = bonus cost cho việc flat +
            //         // cost cho việc thêm c vào index i (thêm 2 cạnh ở copy route) -
            //         // cạnh bị phá ở copy route
            //         if (const double add_cost = this_best_flat_cost - instance->tau[copy_route[i - 1]][copy_route[i]] +
            //                 instance->tau[copy_route[i - 1]][c] + instance->tau[c][copy_route[i]]; add_cost <
            //             best_truck_cost) {
            //             best_truck_cost = add_cost;
            //             best_truck_index_this_sortie = solution->sortie_stages[d].start_index + i;
            //         }
            //     }
            //     if (best_truck_cost < best_overall) {
            //         best_overall = best_truck_cost;
            //         which_sortie_to_flat = d;
            //         best_index_for_flat = best_flat_index;
            //         best_truck_index = best_truck_index_this_sortie;
            //     }
            // }
            //
            //
            // // Xoa khoi drone
            // std::erase(solution->served_by_drone, which_sortie_to_flat);
            // for (const int d : solution->served_by_drone) {
            //     if (solution->sortie_stages[d].start_index >= solution->sortie_stages[which_sortie_to_flat].end_index) {
            //         solution->sortie_stages[d].start_index++;
            //     }
            //     if (solution->sortie_stages[d].end_index >= solution->sortie_stages[which_sortie_to_flat].end_index) {
            //         solution->sortie_stages[d].end_index++;
            //     }
            // }
            // solution->sortie_stages[which_sortie_to_flat].start_index = -1;
            // solution->sortie_stages[which_sortie_to_flat].end_index = -1;
            // solution->sortie_stages[which_sortie_to_flat].truck_trip = -1;
            // solution->sortie_stages[which_sortie_to_flat].drone_trip = -1;
            // // them vao truck
            // solution->truck_route.insert(solution->truck_route.begin() + best_index_for_flat, which_sortie_to_flat);
            //
            //
            // solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
            // solution->visited[c] = true;
            // for (auto& st : solution->sortie_stages) {
            //     if (st.start_index >= best_truck_index) {
            //         st.start_index++;
            //     }
            //     if (st.end_index >= best_truck_index) {
            //         st.end_index++;
            //     }
            // }
            // Initialize variables to track the best sortie, indices, and costs
            int which_sortie_to_flat = -1;
            int best_index_for_flat = -1;
            bool truck_after_flat = true;
            double best_overall = 1e9;
            double best_overall_drone_cost, best_overall_truck_cost;

            for (const int d : solution->served_by_drone) {
                // Initialize variables for tracking the best costs and indices within the current sortie
                int best_flat_index = -1;
                int best_truck_index_this_sortie = -1;
                int best_start_index_this_sortie = -1;
                int best_end_index_this_sortie = -1;

                double this_best_flat_cost = 1e9;
                double best_truck_trip_this_sortie, best_drone_trip_this_sortie;

                // Calculate the best flat cost for the current sortie
                for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].
                     end_index; index++) {
                    double increased_cost = instance->tau[solution->truck_route[index - 1]][d] +
                        instance->tau[d][solution->truck_route[index]] -
                        instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];

                    if (increased_cost < this_best_flat_cost) {
                        this_best_flat_cost = increased_cost;
                        best_flat_index = index;
                    }
                }

                // Copy the route and insert the current sortie
                std::vector<int> copy_route(solution->truck_route.begin() + solution->sortie_stages[d].start_index,
                                            solution->truck_route.begin() + solution->sortie_stages[d].end_index + 1);
                copy_route.insert(copy_route.begin() + best_flat_index - solution->sortie_stages[d].start_index, d);

                // Evaluate costs for adding customer 'c' to the truck route
                for (int i = 1; i < copy_route.size(); i++) {
                    double add_cost = this_best_flat_cost - instance->tau[copy_route[i - 1]][copy_route[i]] +
                        instance->tau[copy_route[i - 1]][c] + instance->tau[c][copy_route[i]];

                    if (add_cost < best_truck_cost) {
                        best_truck_cost = add_cost;
                        best_truck_index_this_sortie = solution->sortie_stages[d].start_index + i;
                    }
                }

                // Compare and update the best overall cost
                if (best_truck_cost < 1e9) {
                    if (best_truck_cost < best_overall) {
                        best_overall = best_truck_cost;
                        which_sortie_to_flat = d;
                        best_index_for_flat = best_flat_index;
                        best_truck_index = best_truck_index_this_sortie;
                        truck_after_flat = true;
                    }
                }
            }

            // Update the solution after finding the best sortie to flatten
            std::erase(solution->served_by_drone, which_sortie_to_flat);
            for (const int d : solution->served_by_drone) {
                auto& sortie_stages = solution->sortie_stages[d];
                if (sortie_stages.start_index >= solution->sortie_stages[which_sortie_to_flat].end_index) {
                    sortie_stages.start_index++;
                    sortie_stages.end_index++;
                    continue;
                }

                if (sortie_stages.end_index >= solution->sortie_stages[which_sortie_to_flat].end_index) {
                    sortie_stages.end_index++;
                }
            }

            solution->truck_route.insert(solution->truck_route.begin() + best_index_for_flat, which_sortie_to_flat);

            // Update the route and stages based on the best overall cost
            if (truck_after_flat || instance->heavy_bool[c]) {
                solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
                solution->visited[c] = true;
                for (const int d : solution->served_by_drone) {
                    auto& sortie_stage = solution->sortie_stages[d];
                    if (sortie_stage.start_index >= best_truck_index) {
                        sortie_stage.start_index++;
                        sortie_stage.end_index++;
                        continue;
                    }
                    if (sortie_stage.end_index >= best_truck_index) {
                        sortie_stage.end_index++;
                    }
                }
            }
        }
    }
    // bool improvable = true;
    // while (improvable) {
    //     improvable = false;
    //     std::unordered_map<int, int> countMap;
    //     std::vector<int> duplicateIndices;
    //
    //     // First pass: Count occurrences of each element
    //     for (const auto& element : solution->truck_route) {
    //         countMap[element]++;
    //     }
    //
    //     // Second pass: Collect indices of duplicates
    //     for (int i = 0; i < solution->truck_route.size(); ++i) {
    //         if (countMap[solution->truck_route[i]] > 1) {
    //             duplicateIndices.push_back(i);
    //         }
    //     }
    //
    //     auto spaces = solution->find_sortie_spaces();
    //     std::vector<int> possible_candidate_indexes;
    //     std::vector<int> candidate_belongs_to_sortie(instance->num_node, 0);
    //     std::vector new_truck_trip_without_candidate(
    //         instance->num_node, std::vector<double>(instance->num_node));
    //     for (const int d : solution->served_by_drone) {
    //         for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].end_index -
    //              1; index++) {
    //             if (!exist(duplicateIndices, index) && !
    //                 exist(solution->served_by_drone, solution->truck_route[index])) {
    //                 if (!instance->heavy_bool[solution->truck_route[index]]) {
    //                     if (const double new_truck_trip = solution->sortie_stages[d].truck_trip
    //                             - instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]]
    //                             - instance->tau[solution->truck_route[index]][solution->truck_route[index + 1]]
    //                             + instance->tau[solution->truck_route[index - 1]][solution->truck_route[index + 1]];
    //                         new_truck_trip <= instance->e - instance->sr) {
    //                         possible_candidate_indexes.emplace_back(index);
    //                         candidate_belongs_to_sortie[index] = d;
    //                         new_truck_trip_without_candidate[d][index] = new_truck_trip;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //
    //
    //     for (const auto& [fst, snd] : spaces) {
    //         for (int index = fst + 1; index <= snd - 1; index++) {
    //             if (!exist(duplicateIndices, index)) {
    //                 if (!instance->heavy_bool[solution->truck_route[index]] && !exist(
    //                     solution->served_by_drone, solution->truck_route[index])) {
    //                     possible_candidate_indexes.emplace_back(index);
    //                 }
    //             }
    //         }
    //     }
    //
    //     // got all possible candidate?
    //     int best_index_candidate = -1, best_start_index = -1, best_end_index = -1;
    //     double best_truck_trip = -1, best_drone_trip = -1, best_increased_cost = 0;
    //
    //     for (const int candidate : possible_candidate_indexes) {
    //         // tinh increased cost cho sortie/quang duong bi anh huong (neu co)
    //         double increased_cost = 0;
    //         if (candidate_belongs_to_sortie[candidate] != 0) {
    //             // = new_cost - current_cost
    //             increased_cost += (std::max(
    //                     new_truck_trip_without_candidate[candidate_belongs_to_sortie[candidate]][candidate],
    //                     solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip))
    //                 - std::max(solution->sortie_stages[candidate_belongs_to_sortie[candidate]].truck_trip,
    //                            solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip);
    //         }
    //         else {
    //             increased_cost += -instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[candidate]]
    //                 - instance->tau[solution->truck_route[candidate]][solution->truck_route[candidate + 1]]
    //                 + instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[candidate + 1]];
    //         }
    //         // tinh increased cost cho viec tao sortie moi
    //         for (const auto& [fst, snd] : spaces) {
    //             for (int start = fst; start <= snd - 1; start++) {
    //                 double truck_trip = 0;
    //                 for (int end = start + 1; end <= snd; end++) {
    //                     if (solution->truck_route[start] == solution->truck_route[end]) {
    //                         break;
    //                     }
    //                     truck_trip += instance->tau[solution->truck_route[end - 1]][solution->truck_route[end]];
    //                     if (truck_trip >= instance->e - instance->sr) {
    //                         break;
    //                     }
    //                     // dua candidate nay vao sortie.
    //                     if (const double drone_trip = instance->tau_prime[solution->truck_route[start]][solution->
    //                                 truck_route[candidate]]
    //                             + instance->tau_prime[solution->truck_route[candidate]][solution->truck_route[end]];
    //                         drone_trip <= instance->e - instance->sr) {
    //                         // increased_cost when creating this sortie?
    //                         if (drone_trip > truck_trip) {
    //                             if (const double inc = increased_cost + (drone_trip - truck_trip); inc <
    //                                 best_increased_cost) {
    //                                 best_increased_cost = inc;
    //                                 improvable = true;
    //                                 best_index_candidate = candidate;
    //                                 best_start_index = start;
    //                                 best_end_index = end;
    //                                 best_truck_trip = truck_trip;
    //                                 best_drone_trip = drone_trip;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     // found the best.
    //     if (improvable) {
    //         solution->served_by_drone.push_back(solution->truck_route[best_index_candidate]);
    //         solution->sortie_stages[solution->truck_route[best_index_candidate]] = {
    //             best_start_index, best_end_index, best_drone_trip, best_truck_trip
    //         };
    //         // update sortie phia sau candidate index nay
    //         for (const int d : solution->served_by_drone) {
    //             if (solution->sortie_stages[d].start_index > best_index_candidate) {
    //                 solution->sortie_stages[d].start_index--;
    //             }
    //             if (solution->sortie_stages[d].end_index > best_index_candidate) {
    //                 solution->sortie_stages[d].end_index--;
    //             }
    //         }
    //         solution->sortie_stages[candidate_belongs_to_sortie[best_index_candidate]].truck_trip =
    //             new_truck_trip_without_candidate[candidate_belongs_to_sortie[best_index_candidate]][
    //                 best_index_candidate];
    //         solution->truck_route.erase(solution->truck_route.begin() + best_index_candidate);
    //     }
    // }
}

void ALNS::SortieFirstTruckSecond(std::shared_ptr<Solution>& solution) {
    std::vector<int> to_insert;
    for (int i : instance->C) {
        if (!solution->visited[i]) {
            to_insert.push_back(i);
        }
    }
    const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);

    // Shuffle the vector
    std::ranges::shuffle(to_insert.begin(), to_insert.end(), mt);
    for (const int& c : to_insert) {
        if (instance->heavy_bool[c]) {
            int best_truck_index = -1;
            double best_truck_cost = 1e9;
            int insert_into_truck_route_of_sortie = -1;
            double new_truck_cost_for_sortie = -1;

            for (int index = 1; index < solution->truck_route.size(); index++) {
                // check if this index is in a sortie?
                bool index_in_sortie = false;

                for (const int d : solution->served_by_drone) {
                    if (solution->sortie_stages[d].start_index < index && index <= solution->sortie_stages[d].
                        end_index) {
                        index_in_sortie = true;
                        // gia su them vao sortie nay tai index nay
                        const double this_truck_cost = solution->sortie_stages[d].truck_trip;
                        // nếu thêm khách hàng c vào index
                        // mất cạnh: index-1 -> index
                        // thêm cạnh: index-1 -> c và c->index
                        const double new_truck_cost = this_truck_cost - instance->tau[solution->truck_route[index -
                                1]][
                                solution->
                                truck_route[index]] +
                            instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->
                                truck_route[
                                    index]];
                        if (new_truck_cost > instance->e - instance->sr) {
                            break;
                        }
                        double increased_cost = 0;
                        if (new_truck_cost > solution->sortie_stages[d].drone_trip) {
                            if (this_truck_cost < solution->sortie_stages[d].drone_trip) {
                                increased_cost = new_truck_cost - solution->sortie_stages[d].drone_trip;
                            }
                            else {
                                increased_cost = new_truck_cost - this_truck_cost;
                            }
                        }
                        if (increased_cost < best_truck_cost) {
                            best_truck_index = index;
                            best_truck_cost = increased_cost;
                            insert_into_truck_route_of_sortie = d;
                            new_truck_cost_for_sortie = new_truck_cost;
                        }
                    }
                }

                if (!index_in_sortie) {
                    // tinh increased cost
                    if (const double increased_cost = instance->tau[solution->truck_route[index - 1]][c] + instance
                            ->tau[c][
                                solution->truck_route[index]] -
                            instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];
                        increased_cost <
                        best_truck_cost) {
                        best_truck_index = index;
                        best_truck_cost = increased_cost;
                        insert_into_truck_route_of_sortie = -1;
                    }
                }
            }
            if (best_truck_index > 0) {
                // do the insertion.
                // have to calculate new sortie length too if insert into sortie.
                solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
                solution->visited[c] = true;
                // update sorties với index phía sau best_truck_index
                for (const int d : solution->served_by_drone) {
                    if (solution->sortie_stages[d].start_index >= best_truck_index) {
                        solution->sortie_stages[d].start_index++;
                    }
                    if (solution->sortie_stages[d].end_index >= best_truck_index) {
                        solution->sortie_stages[d].end_index++;
                    }
                }

                // done. we do continue this loop.
                if (insert_into_truck_route_of_sortie > -1) {
                    solution->sortie_stages[insert_into_truck_route_of_sortie].truck_trip =
                        new_truck_cost_for_sortie;
                }
            }
            if (best_truck_index < 0) {
                // đè thằng nào min cost xuống.
                int which_sortie_to_flat = -1;
                int best_index_for_flat = -1;
                double best_overall = 1e9;
                for (const int d : solution->served_by_drone) {
                    // nếu đè d xuống và đè vào đâu đó min cost trong khoảng của nó.
                    int best_flat_index = -1;
                    int best_truck_index_this_sortie = -1;
                    double this_best_flat_cost = 1e9;

                    for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d]
                         .
                         end_index; index++) {
                        // thêm vào index, cost sẽ là? phần thêm = index - 1 -> flat + flat-> index
                        if (const double increased_cost = instance->tau[solution->truck_route[index - 1]][d] +
                                instance->tau
                                [d][
                                    solution->truck_route[index]]
                                - instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];
                            increased_cost
                            < this_best_flat_cost) {
                            this_best_flat_cost = increased_cost;
                            best_flat_index = index;
                        }
                    }
                    std::vector<int> copy_route;
                    for (int i = solution->sortie_stages[d].start_index; i <= solution->sortie_stages[d].end_index;
                         i++) {
                        copy_route.push_back(solution->truck_route[i]);
                    }
                    copy_route.insert(copy_route.begin() + best_flat_index - solution->sortie_stages[d].start_index,
                                      d);

                    // thêm khách hàng c vào truck hay drone vào cái copy route này?
                    for (int i = 1; i < copy_route.size(); i++) {
                        // add cost = bonus cost cho việc flat +
                        // cost cho việc thêm c vào index i (thêm 2 cạnh ở copy route) -
                        // cạnh bị phá ở copy route
                        if (const double add_cost = this_best_flat_cost - instance->tau[copy_route[i - 1]][
                                    copy_route[i]] +
                                instance->tau[copy_route[i - 1]][c] + instance->tau[c][copy_route[i]]; add_cost <
                            best_truck_cost) {
                            best_truck_cost = add_cost;
                            best_truck_index_this_sortie = solution->sortie_stages[d].start_index + i;
                        }
                    }
                    if (best_truck_cost < best_overall) {
                        best_overall = best_truck_cost;
                        which_sortie_to_flat = d;
                        best_index_for_flat = best_flat_index;
                        best_truck_index = best_truck_index_this_sortie;
                    }
                }


                // Xoa khoi drone
                std::erase(solution->served_by_drone, which_sortie_to_flat);
                for (const int d : solution->served_by_drone) {
                    if (solution->sortie_stages[d].start_index >= solution->sortie_stages[which_sortie_to_flat].
                        end_index) {
                        solution->sortie_stages[d].start_index++;
                    }
                    if (solution->sortie_stages[d].end_index >= solution->sortie_stages[which_sortie_to_flat].
                        end_index) {
                        solution->sortie_stages[d].end_index++;
                    }
                }
                solution->sortie_stages[which_sortie_to_flat].start_index = -1;
                solution->sortie_stages[which_sortie_to_flat].end_index = -1;
                solution->sortie_stages[which_sortie_to_flat].truck_trip = -1;
                solution->sortie_stages[which_sortie_to_flat].drone_trip = -1;
                // them vao truck
                solution->truck_route.insert(solution->truck_route.begin() + best_index_for_flat,
                                             which_sortie_to_flat);


                solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
                solution->visited[c] = true;
                for (auto& st : solution->sortie_stages) {
                    if (st.start_index >= best_truck_index) {
                        st.start_index++;
                    }
                    if (st.end_index >= best_truck_index) {
                        st.end_index++;
                    }
                }
            }
        }
        else {
            // which spaces to insert.
            double best_drone_trip = -1;
            double best_truck_trip = -1;
            int best_start = -1, best_end = -1;
            double best_add_cost = 1e9;
            for (auto spaces = solution->find_sortie_spaces(); auto& [fst, snd] : spaces) {
                for (int start = fst; start <= snd - 1; start++) {
                    for (int end = start + 1; end <= snd; end++) {
                        const double drone_trip = instance->tau_prime[solution->truck_route[start]][c]
                            + instance->tau_prime[c][solution->truck_route[end]];
                        if (drone_trip <= instance->e - instance->sr) {
                            // if drone trip is acceptable
                            // check the truck trip.
                            double truck_trip = 0;
                            for (int l = start; l <= end - 1; l++) {
                                truck_trip += instance->tau[solution->truck_route[l]][solution->truck_route[l + 1]];
                            }
                            if (truck_trip <= instance->e - instance->sr) {
                                // drone trip and truck trip is okay.
                                // add cost?
                                if (const double add_cost = std::max(0.0, drone_trip - truck_trip); add_cost == 0) {
                                    // update luon. du sao cung khong the < 0.
                                    solution->visited[c] = true;
                                    solution->served_by_drone.push_back(c);
                                    solution->sortie_stages[c].start_index = start;
                                    solution->sortie_stages[c].end_index = end;
                                    solution->sortie_stages[c].truck_trip = truck_trip;
                                    solution->sortie_stages[c].drone_trip = drone_trip;
                                    goto done;
                                }
                                else {
                                    // update add_cost.
                                    if (add_cost < best_add_cost) {
                                        best_add_cost = add_cost;
                                        best_drone_trip = drone_trip;
                                        best_truck_trip = truck_trip;
                                        best_start = start;
                                        best_end = end;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (best_start != -1) {
                // got. add to drone trip.
                solution->visited[c] = true;
                solution->served_by_drone.push_back(c);
                solution->sortie_stages[c].start_index = best_start;
                solution->sortie_stages[c].end_index = best_end;
                solution->sortie_stages[c].truck_trip = best_truck_trip;
                solution->sortie_stages[c].drone_trip = best_drone_trip;
            }
            else {
                int best_truck_index = -1;
                double best_truck_cost = 1e9;
                int insert_into_truck_route_of_sortie = -1;
                double new_truck_cost_for_sortie = -1;

                for (int index = 1; index < solution->truck_route.size(); index++) {
                    // check if this index is in a sortie?
                    bool index_in_sortie = false;

                    for (const int d : solution->served_by_drone) {
                        if (solution->sortie_stages[d].start_index < index && index <= solution->sortie_stages[d].
                            end_index) {
                            index_in_sortie = true;
                            // gia su them vao sortie nay tai index nay
                            const double this_truck_cost = solution->sortie_stages[d].truck_trip;
                            // nếu thêm khách hàng c vào index
                            // mất cạnh: index-1 -> index
                            // thêm cạnh: index-1 -> c và c->index
                            const double new_truck_cost = this_truck_cost - instance->tau[solution->truck_route[index -
                                    1]][
                                    solution->
                                    truck_route[index]] +
                                instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->
                                    truck_route[
                                        index]];
                            if (new_truck_cost > instance->e - instance->sr) {
                                break;
                            }
                            double increased_cost = 0;
                            if (new_truck_cost > solution->sortie_stages[d].drone_trip) {
                                if (this_truck_cost < solution->sortie_stages[d].drone_trip) {
                                    increased_cost = new_truck_cost - solution->sortie_stages[d].drone_trip;
                                }
                                else {
                                    increased_cost = new_truck_cost - this_truck_cost;
                                }
                            }
                            if (increased_cost < best_truck_cost) {
                                best_truck_index = index;
                                best_truck_cost = increased_cost;
                                insert_into_truck_route_of_sortie = d;
                                new_truck_cost_for_sortie = new_truck_cost;
                            }
                        }
                    }

                    if (!index_in_sortie) {
                        // tinh increased cost
                        if (const double increased_cost = instance->tau[solution->truck_route[index - 1]][c] + instance
                                ->tau[c][
                                    solution->truck_route[index]] -
                                instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];
                            increased_cost <
                            best_truck_cost) {
                            best_truck_index = index;
                            best_truck_cost = increased_cost;
                            insert_into_truck_route_of_sortie = -1;
                        }
                    }
                }
                if (best_truck_index > 0) {
                    // do the insertion.
                    // have to calculate new sortie length too if insert into sortie.
                    solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
                    solution->visited[c] = true;
                    // update sorties với index phía sau best_truck_index
                    for (const int d : solution->served_by_drone) {
                        if (solution->sortie_stages[d].start_index >= best_truck_index) {
                            solution->sortie_stages[d].start_index++;
                        }
                        if (solution->sortie_stages[d].end_index >= best_truck_index) {
                            solution->sortie_stages[d].end_index++;
                        }
                    }

                    // done. we do continue this loop.
                    if (insert_into_truck_route_of_sortie > -1) {
                        solution->sortie_stages[insert_into_truck_route_of_sortie].truck_trip =
                            new_truck_cost_for_sortie;
                    }
                }
                if (best_truck_index < 0) {
                    // đè thằng nào min cost xuống.
                    int which_sortie_to_flat = -1;
                    int best_index_for_flat = -1;
                    double best_overall = 1e9;
                    for (const int d : solution->served_by_drone) {
                        // nếu đè d xuống và đè vào đâu đó min cost trong khoảng của nó.
                        int best_flat_index = -1;
                        int best_truck_index_this_sortie = -1;
                        double this_best_flat_cost = 1e9;

                        for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d]
                             .
                             end_index; index++) {
                            // thêm vào index, cost sẽ là? phần thêm = index - 1 -> flat + flat-> index
                            if (const double increased_cost = instance->tau[solution->truck_route[index - 1]][d] +
                                    instance->tau
                                    [d][
                                        solution->truck_route[index]]
                                    - instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];
                                increased_cost
                                < this_best_flat_cost) {
                                this_best_flat_cost = increased_cost;
                                best_flat_index = index;
                            }
                        }
                        std::vector<int> copy_route;
                        for (int i = solution->sortie_stages[d].start_index; i <= solution->sortie_stages[d].end_index;
                             i++) {
                            copy_route.push_back(solution->truck_route[i]);
                        }
                        copy_route.insert(copy_route.begin() + best_flat_index - solution->sortie_stages[d].start_index,
                                          d);

                        // thêm khách hàng c vào truck hay drone vào cái copy route này?
                        for (int i = 1; i < copy_route.size(); i++) {
                            // add cost = bonus cost cho việc flat +
                            // cost cho việc thêm c vào index i (thêm 2 cạnh ở copy route) -
                            // cạnh bị phá ở copy route
                            if (const double add_cost = this_best_flat_cost - instance->tau[copy_route[i - 1]][
                                        copy_route[i]] +
                                    instance->tau[copy_route[i - 1]][c] + instance->tau[c][copy_route[i]]; add_cost <
                                best_truck_cost) {
                                best_truck_cost = add_cost;
                                best_truck_index_this_sortie = solution->sortie_stages[d].start_index + i;
                            }
                        }
                        if (best_truck_cost < best_overall) {
                            best_overall = best_truck_cost;
                            which_sortie_to_flat = d;
                            best_index_for_flat = best_flat_index;
                            best_truck_index = best_truck_index_this_sortie;
                        }
                    }


                    // Xoa khoi drone
                    std::erase(solution->served_by_drone, which_sortie_to_flat);
                    for (const int d : solution->served_by_drone) {
                        if (solution->sortie_stages[d].start_index >= solution->sortie_stages[which_sortie_to_flat].
                            end_index) {
                            solution->sortie_stages[d].start_index++;
                        }
                        if (solution->sortie_stages[d].end_index >= solution->sortie_stages[which_sortie_to_flat].
                            end_index) {
                            solution->sortie_stages[d].end_index++;
                        }
                    }
                    solution->sortie_stages[which_sortie_to_flat].start_index = -1;
                    solution->sortie_stages[which_sortie_to_flat].end_index = -1;
                    solution->sortie_stages[which_sortie_to_flat].truck_trip = -1;
                    solution->sortie_stages[which_sortie_to_flat].drone_trip = -1;
                    // them vao truck
                    solution->truck_route.insert(solution->truck_route.begin() + best_index_for_flat,
                                                 which_sortie_to_flat);


                    solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
                    solution->visited[c] = true;
                    for (auto& st : solution->sortie_stages) {
                        if (st.start_index >= best_truck_index) {
                            st.start_index++;
                        }
                        if (st.end_index >= best_truck_index) {
                            st.end_index++;
                        }
                    }
                }
            }
        done:;



        }
    }
}

void ALNS::TruckLocalSearch(std::shared_ptr<Solution>& solution) {
    const auto imp = solution->swap_mid_only();
    local_search_swap_improve += imp;
}

void ALNS::Revisit(std::shared_ptr<Solution>& solution) {
    solution->revisit();
}

void ALNS::RandomSequenceBestCostInsert(std::shared_ptr<Solution>& solution) {
    std::vector<int> to_insert;
    to_insert.reserve(instance->C.size()); // Reserve space to avoid multiple allocations
    for (int i : instance->C) {
        if (!solution->visited[i]) {
            to_insert.push_back(i);
        }
    }
    if (to_insert.size() <= 2) {
        lazy_insert_iter++;
    }

    const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);

    // Shuffle the vector
    std::ranges::shuffle(to_insert, mt);

    for (int c : to_insert) {
        if (solution->truck_route.size() == 2) {
            solution->truck_route.insert(solution->truck_route.begin() + 1, c);
            solution->visited[c] = true;
            continue;
        }

        double best_drone_cost = 1e9;
        int best_start_index = -1;
        int best_end_index = -1;
        int best_truck_index = -1;
        double best_truck_cost = 1e9;
        int insert_into_truck_route_of_sortie = -1;
        double new_truck_cost_for_sortie = -1;

        for (int index = 1; index < solution->truck_route.size(); index++) {
            bool index_in_sortie = false;

            for (const int d : solution->served_by_drone) {
                if (auto& ss = solution->sortie_stages[d]; ss.start_index < index && index <= ss.end_index) {
                    index_in_sortie = true;

                    double new_truck_cost = ss.truck_trip - instance->tau[solution->truck_route[index - 1]][solution->
                            truck_route[index]] +
                        instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->truck_route[
                            index]];

                    if (new_truck_cost > instance->e - instance->sr) {
                        break;
                    }

                    double increased_cost = 0;
                    if (new_truck_cost > ss.drone_trip) {
                        increased_cost = new_truck_cost - std::min(ss.truck_trip,
                                                                   ss.drone_trip);
                    }
                    if (increased_cost < best_truck_cost) {
                        best_truck_index = index;
                        best_truck_cost = increased_cost;
                        insert_into_truck_route_of_sortie = d;
                        new_truck_cost_for_sortie = new_truck_cost;
                    }
                }
            }

            if (!index_in_sortie) {
                double increased_cost = instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->
                        truck_route[index]] -
                    instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];

                if (increased_cost < best_truck_cost) {
                    best_truck_index = index;
                    best_truck_cost = increased_cost;
                    insert_into_truck_route_of_sortie = -1;
                }
            }
        }

        if (best_truck_index < 0) {
            // Initialize variables to track the best sortie, indices, and costs
            int which_sortie_to_flat = -1;
            int best_index_for_flat = -1;
            bool truck_after_flat = true;
            double best_overall = 1e9;
            double best_overall_drone_cost, best_overall_truck_cost;

            for (const int d : solution->served_by_drone) {
                // Initialize variables for tracking the best costs and indices within the current sortie
                int best_flat_index = -1;
                int best_truck_index_this_sortie = -1;
                int best_start_index_this_sortie = -1;
                int best_end_index_this_sortie = -1;

                double this_best_flat_cost = 1e9;
                double best_truck_trip_this_sortie, best_drone_trip_this_sortie;

                // Calculate the best flat cost for the current sortie
                for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].
                     end_index; index++) {
                    double increased_cost = instance->tau[solution->truck_route[index - 1]][d] +
                        instance->tau[d][solution->truck_route[index]] -
                        instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];

                    if (increased_cost < this_best_flat_cost) {
                        this_best_flat_cost = increased_cost;
                        best_flat_index = index;
                    }
                }

                // Copy the route and insert the current sortie
                std::vector<int> copy_route(solution->truck_route.begin() + solution->sortie_stages[d].start_index,
                                            solution->truck_route.begin() + solution->sortie_stages[d].end_index + 1);
                copy_route.insert(copy_route.begin() + best_flat_index - solution->sortie_stages[d].start_index, d);

                // Evaluate costs for adding customer 'c' to the truck route
                for (int i = 1; i < copy_route.size(); i++) {
                    double add_cost = this_best_flat_cost - instance->tau[copy_route[i - 1]][copy_route[i]] +
                        instance->tau[copy_route[i - 1]][c] + instance->tau[c][copy_route[i]];

                    if (add_cost < best_truck_cost) {
                        best_truck_cost = add_cost;
                        best_truck_index_this_sortie = solution->sortie_stages[d].start_index + i;
                    }
                }

                // Evaluate costs for adding customer 'c' to the drone route
                if (!instance->heavy_bool[c]) {
                    for (int l = 0; l <= copy_route.size() - 2; l++) {
                        double truck_cost = 0;
                        for (int r = l + 1; r <= copy_route.size() - 1; r++) {
                            if (copy_route[l] == copy_route[r]) {
                                break;
                            }
                            double drone_cost = instance->tau_prime[copy_route[l]][c] + instance->tau_prime[c][
                                copy_route[r]];
                            if (drone_cost > instance->e - instance->sr) {
                                continue;
                            }

                            truck_cost += instance->tau[copy_route[r - 1]][copy_route[r]];
                            if (truck_cost > instance->e - instance->sr) {
                                break;
                            }

                            double wait = std::max(0.0, drone_cost - truck_cost);
                            if (double add_cost = this_best_flat_cost + wait; add_cost < best_drone_cost) {
                                best_drone_cost = add_cost;
                                best_start_index_this_sortie = solution->sortie_stages[d].start_index + l;
                                best_end_index_this_sortie = solution->sortie_stages[d].start_index + r;
                                best_truck_trip_this_sortie = truck_cost;
                                best_drone_trip_this_sortie = drone_cost;
                            }
                        }
                    }
                }

                // Compare and update the best overall cost
                if (best_truck_cost < best_drone_cost) {
                    if (best_truck_cost < best_overall) {
                        best_overall = best_truck_cost;
                        which_sortie_to_flat = d;
                        best_index_for_flat = best_flat_index;
                        best_truck_index = best_truck_index_this_sortie;
                        truck_after_flat = true;
                    }
                }
                else {
                    if (best_drone_cost < best_overall) {
                        best_overall = best_drone_cost;
                        which_sortie_to_flat = d;
                        best_index_for_flat = best_flat_index;
                        best_start_index = best_start_index_this_sortie;
                        best_end_index = best_end_index_this_sortie;
                        truck_after_flat = false;
                        best_overall_drone_cost = best_drone_trip_this_sortie;
                        best_overall_truck_cost = best_truck_trip_this_sortie;
                    }
                }
            }

            // Update the solution after finding the best sortie to flatten
            std::erase(solution->served_by_drone, which_sortie_to_flat);
            for (const int d : solution->served_by_drone) {
                auto& sortie_stages = solution->sortie_stages[d];
                if (sortie_stages.start_index >= solution->sortie_stages[which_sortie_to_flat].end_index) {
                    sortie_stages.start_index++;
                    sortie_stages.end_index++;
                    continue;
                }

                if (sortie_stages.end_index >= solution->sortie_stages[which_sortie_to_flat].end_index) {
                    sortie_stages.end_index++;
                }
            }
            solution->truck_route.insert(solution->truck_route.begin() + best_index_for_flat, which_sortie_to_flat);
            // Update the route and stages based on the best overall cost
            if (truck_after_flat || instance->heavy_bool[c]) {
                solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
                solution->visited[c] = true;
                for (const int d : solution->served_by_drone) {
                    auto& sortie_stage = solution->sortie_stages[d];
                    if (sortie_stage.start_index >= best_truck_index) {
                        sortie_stage.start_index++;
                        sortie_stage.end_index++;
                        continue;
                    }
                    if (sortie_stage.end_index >= best_truck_index) {
                        sortie_stage.end_index++;
                    }
                }
                // solution->sortie_invalid_occurrence();
            }
            else {
                solution->served_by_drone.push_back(c);
                solution->sortie_stages[c] = {
                    best_start_index, best_end_index, best_overall_drone_cost, best_overall_truck_cost
                };
                solution->visited[c] = true;
                // solution->sortie_invalid_occurrence();
            }
        }
        else {
            double best_truck_trip = 1e9;
            double best_drone_trip = 1e9;
            auto spaces = solution->find_sortie_spaces();

            if (!instance->heavy_bool[c]) {
                for (auto [fst, snd] : spaces) {
                    for (int start = fst; start <= snd - 1; start++) {
                        double truck_travel_cost = 0;
                        for (int end = start + 1; end <= snd; end++) {
                            if (solution->truck_route[start] == solution->truck_route[end]) {
                                break;
                            }
                            truck_travel_cost += instance->tau[solution->truck_route[end - 1]][solution->truck_route[
                                end]];

                            if (truck_travel_cost > instance->e - instance->sr) {
                                break;
                            }

                            double drone_travel_cost = instance->tau_prime[solution->truck_route[start]][c] + instance->
                                tau_prime[c][solution->truck_route[end]];

                            if (drone_travel_cost > instance->e - instance->sr) {
                                continue;
                            }

                            if (double waiting_time = std::max(0.0, drone_travel_cost - truck_travel_cost); waiting_time
                                <= best_drone_cost) {
                                best_drone_cost = waiting_time;
                                best_start_index = start;
                                best_end_index = end;
                                best_drone_trip = drone_travel_cost;
                                best_truck_trip = truck_travel_cost;
                            }
                        }
                    }
                }
            }

            if (best_truck_cost < best_drone_cost) {
                // std::cout << "solution before the bug:" << std::endl;
                // solution->print_solution();
                solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
                solution->visited[c] = true;

                for (const int d : solution->served_by_drone) {
                    auto& sortie_stage = solution->sortie_stages[d];
                    if (sortie_stage.start_index >= best_truck_index) {
                        sortie_stage.start_index++;
                        sortie_stage.end_index++;
                        continue;
                    }
                    if (sortie_stage.end_index >= best_truck_index) {
                        sortie_stage.end_index++;
                    }
                }

                if (insert_into_truck_route_of_sortie > -1) {
                    solution->sortie_stages[insert_into_truck_route_of_sortie].truck_trip = new_truck_cost_for_sortie;
                }
                // solution->sortie_invalid_occurrence();
            }
            else if (best_start_index > -1 && best_end_index > -1) {
                solution->served_by_drone.push_back(c);
                solution->visited[c] = true;
                solution->sortie_stages[c] = {best_start_index, best_end_index, best_drone_trip, best_truck_trip};
                // solution->sortie_invalid_occurrence();
            }
        }
    }
    bool improvable = true;
    while (improvable) {
        improvable = false;
        std::unordered_map<int, int> countMap;
        std::vector<int> duplicateIndices;

        // First pass: Count occurrences of each element
        for (const auto& element : solution->truck_route) {
            countMap[element]++;
        }

        // Second pass: Collect indices of duplicates
        for (int i = 0; i < solution->truck_route.size(); ++i) {
            if (countMap[solution->truck_route[i]] > 1) {
                duplicateIndices.push_back(i);
            }
        }

        auto spaces = solution->find_sortie_spaces();

        std::vector<int> possible_candidate_indexes;
        std::vector<int> candidate_belongs_to_sortie(instance->num_node, 0);
        std::vector new_truck_trip_without_candidate(
            instance->num_node, std::vector<double>(instance->num_node));
        for (const int d : solution->served_by_drone) {
            for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].end_index -
                 1; index++) {
                if (!exist(duplicateIndices, index) && !instance->heavy_bool[solution->truck_route[index]]) {
                    if (const double new_truck_trip = solution->sortie_stages[d].truck_trip
                            - instance->tau[solution->truck_route[index - 1]]
                            [solution->truck_route[index]]
                            - instance->tau[solution->truck_route[index]]
                            [solution->truck_route[index + 1]]
                            + instance->tau[solution->truck_route[index - 1]]
                            [solution->truck_route[index + 1]];
                        new_truck_trip <= instance->e - instance->sr) {
                        possible_candidate_indexes.emplace_back(index);
                        candidate_belongs_to_sortie[index] = d;
                        new_truck_trip_without_candidate[d][index] = new_truck_trip;
                    }
                }
            }
        }


        for (const auto& [fst, snd] : spaces) {
            for (int index = fst + 1; index <= snd - 1; index++) {
                if (!exist(duplicateIndices, index)) {
                    if (!exist(duplicateIndices, index) && !instance->heavy_bool[solution->truck_route[index]]) {
                        possible_candidate_indexes.emplace_back(index);
                    }
                }
            }
        }

        // got all possible candidate?
        int best_index_candidate = -1, best_start_index = -1, best_end_index = -1;
        double best_truck_trip = -1, best_drone_trip = -1, best_increased_cost = 0;

        for (const int candidate : possible_candidate_indexes) {
            // tinh increased cost cho sortie/quang duong bi anh huong (neu co)
            double increased_cost = candidate_belongs_to_sortie[candidate] != 0
                                        ? (std::max(
                                            new_truck_trip_without_candidate[candidate_belongs_to_sortie[candidate]][
                                                candidate],
                                            solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip))
                                        - std::max(
                                            solution->sortie_stages[candidate_belongs_to_sortie[candidate]].truck_trip,
                                            solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip)
                                        : -instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[
                                            candidate]]
                                        - instance->tau[solution->truck_route[candidate]][solution->truck_route[
                                            candidate + 1]]
                                        + instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[
                                            candidate + 1]];

            // tinh increased cost cho viec tao sortie moi
            for (const auto& [fst, snd] : spaces) {
                for (int start = fst; start <= snd - 1; start++) {
                    double truck_trip = 0;
                    for (int end = start + 1; end <= snd; end++) {
                        if (solution->truck_route[start] == solution->truck_route[end]) {
                            break;
                        }
                        truck_trip += instance->tau[solution->truck_route[end - 1]][solution->truck_route[end]];
                        if (truck_trip >= instance->e - instance->sr) {
                            break;
                        }
                        // dua candidate nay vao sortie.
                        if (const double drone_trip = instance->tau_prime[solution->truck_route[start]][solution->
                                    truck_route[candidate]]
                                + instance->tau_prime[solution->truck_route[candidate]][solution->truck_route[end]];
                            drone_trip <= instance->e - instance->sr) {
                            // increased_cost when creating this sortie?
                            if (drone_trip > truck_trip) {
                                if (const double inc = increased_cost + (drone_trip - truck_trip); inc <
                                    best_increased_cost) {
                                    best_increased_cost = inc;
                                    improvable = true;
                                    best_index_candidate = candidate;
                                    best_start_index = start;
                                    best_end_index = end;
                                    best_truck_trip = truck_trip;
                                    best_drone_trip = drone_trip;
                                }
                            }
                        }
                    }
                }
            }
        }
        // found the best.
        if (improvable) {
            solution->served_by_drone.push_back(solution->truck_route[best_index_candidate]);
            solution->sortie_stages[solution->truck_route[best_index_candidate]] = {
                best_start_index, best_end_index, best_drone_trip, best_truck_trip
            };
            // update sortie phia sau candidate index nay
            for (const int d : solution->served_by_drone) {
                if (solution->sortie_stages[d].start_index > best_index_candidate) {
                    solution->sortie_stages[d].start_index--;
                    solution->sortie_stages[d].end_index--;
                    continue;
                }
                if (solution->sortie_stages[d].end_index > best_index_candidate) {
                    solution->sortie_stages[d].end_index--;
                }
            }
            solution->sortie_stages[candidate_belongs_to_sortie[best_index_candidate]].truck_trip =
                new_truck_trip_without_candidate[candidate_belongs_to_sortie[best_index_candidate]][
                    best_index_candidate];
            solution->truck_route.erase(solution->truck_route.begin() + best_index_candidate);
        }
    }
}

void ALNS::TFSCGPT(std::shared_ptr<Solution>& solution) {
    std::vector<int> to_insert;
    for (int i : instance->C) {
        if (!solution->visited[i]) {
            to_insert.push_back(i);
        }
    }

    const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);
    std::ranges::shuffle(to_insert, rng);

    for (int c : to_insert) {
        if (solution->truck_route.size() == 2) {
            solution->truck_route.insert(solution->truck_route.begin() + 1, c);
            solution->visited[c] = true;
            continue;
        }

        int best_truck_index = -1;
        double best_truck_cost = std::numeric_limits<double>::max();
        int insert_into_truck_route_of_sortie = -1;
        double new_truck_cost_for_sortie = -1;

        for (int index = 1; index < solution->truck_route.size(); ++index) {
            bool index_in_sortie = false;

            for (const int d : solution->served_by_drone) {
                if (solution->sortie_stages[d].start_index < index && index <= solution->sortie_stages[d].end_index) {
                    index_in_sortie = true;
                    double this_truck_cost = solution->sortie_stages[d].truck_trip;
                    double new_truck_cost = this_truck_cost - instance->tau[solution->truck_route[index - 1]][solution->
                            truck_route[index]] +
                        instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->truck_route[
                            index]];
                    if (new_truck_cost > instance->e - instance->sr) break;

                    double increased_cost = std::max(0.0, new_truck_cost - solution->sortie_stages[d].drone_trip);
                    if (increased_cost < best_truck_cost) {
                        best_truck_index = index;
                        best_truck_cost = increased_cost;
                        insert_into_truck_route_of_sortie = d;
                        new_truck_cost_for_sortie = new_truck_cost;
                    }
                }
            }

            if (!index_in_sortie) {
                double increased_cost = instance->tau[solution->truck_route[index - 1]][c] + instance->tau[c][solution->
                        truck_route[index]] -
                    instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];
                if (increased_cost < best_truck_cost) {
                    best_truck_index = index;
                    best_truck_cost = increased_cost;
                    insert_into_truck_route_of_sortie = -1;
                }
            }
        }

        if (best_truck_index > 0) {
            solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
            solution->visited[c] = true;

            for (int d : solution->served_by_drone) {
                if (solution->sortie_stages[d].start_index >= best_truck_index)
                    solution->sortie_stages[d].start_index
                        ++;
                if (solution->sortie_stages[d].end_index >= best_truck_index) solution->sortie_stages[d].end_index++;
            }

            if (insert_into_truck_route_of_sortie > -1) {
                solution->sortie_stages[insert_into_truck_route_of_sortie].truck_trip = new_truck_cost_for_sortie;
            }
        }
        else {
            int which_sortie_to_flat = -1;
            int best_index_for_flat = -1;
            double best_overall = std::numeric_limits<double>::max();

            for (int d : solution->served_by_drone) {
                int best_flat_index = -1;
                double this_best_flat_cost = std::numeric_limits<double>::max();

                for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].
                     end_index; ++index) {
                    double increased_cost = instance->tau[solution->truck_route[index - 1]][d] + instance->tau[d][
                            solution->truck_route[index]] -
                        instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];
                    if (increased_cost < this_best_flat_cost) {
                        this_best_flat_cost = increased_cost;
                        best_flat_index = index;
                    }
                }

                std::vector<int> copy_route(solution->truck_route.begin() + solution->sortie_stages[d].start_index,
                                            solution->truck_route.begin() + solution->sortie_stages[d].end_index + 1);
                copy_route.insert(copy_route.begin() + best_flat_index - solution->sortie_stages[d].start_index, d);

                for (int i = 1; i < copy_route.size(); ++i) {
                    double add_cost = this_best_flat_cost - instance->tau[copy_route[i - 1]][copy_route[i]] +
                        instance->tau[copy_route[i - 1]][c] + instance->tau[c][copy_route[i]];
                    if (add_cost < best_truck_cost) {
                        best_truck_cost = add_cost;
                        best_truck_index = solution->sortie_stages[d].start_index + i;
                    }
                }

                if (best_truck_cost < best_overall) {
                    best_overall = best_truck_cost;
                    which_sortie_to_flat = d;
                    best_index_for_flat = best_flat_index;
                }
            }

            std::erase(solution->served_by_drone, which_sortie_to_flat);
            for (int d : solution->served_by_drone) {
                if (solution->sortie_stages[d].start_index >= solution->sortie_stages[which_sortie_to_flat].end_index)
                    solution->sortie_stages[d].start_index++;
                if (solution->sortie_stages[d].end_index >= solution->sortie_stages[which_sortie_to_flat].end_index)
                    solution->sortie_stages[d].end_index++;
            }

            solution->sortie_stages[which_sortie_to_flat].start_index = -1;
            solution->sortie_stages[which_sortie_to_flat].end_index = -1;
            solution->sortie_stages[which_sortie_to_flat].truck_trip = -1;
            solution->sortie_stages[which_sortie_to_flat].drone_trip = -1;

            solution->truck_route.insert(solution->truck_route.begin() + best_index_for_flat, which_sortie_to_flat);
            solution->truck_route.insert(solution->truck_route.begin() + best_truck_index, c);
            solution->visited[c] = true;

            for (auto& st : solution->sortie_stages) {
                if (st.start_index >= best_truck_index) st.start_index++;
                if (st.end_index >= best_truck_index) st.end_index++;
            }
        }
    }
    bool improvable = true;
    while (improvable) {
        improvable = false;
        auto spaces = solution->find_sortie_spaces();

        int best_index = -1;
        int best_start = -1;
        int best_end = -1;
        double best_drone_trip = -1;
        double best_truck_trip = -1;
        double best_reduced_cost = -std::numeric_limits<double>::max();
        bool customer_in_sortie_truck_route = false;
        int best_sortie_affected = -1;
        double best_new_truck_trip_sortie_affected = -1;

        for (const auto& [fst, snd] : spaces) {
            for (int start = fst; start < snd; ++start) {
                for (int end = start + 1; end <= snd; ++end) {
                    for (int i = 1; i < solution->truck_route.size() - 1; ++i) {
                        if (instance->heavy_bool[solution->truck_route[i]]) continue;

                        bool index_l_r = false;
                        bool is_mid = false;
                        int mid_of = -1;

                        for (int d : solution->served_by_drone) {
                            if (i == solution->sortie_stages[d].start_index || i == solution->sortie_stages[d].
                                end_index) {
                                index_l_r = true;
                                break;
                            }
                            if (solution->sortie_stages[d].start_index < i && i < solution->sortie_stages[d].
                                end_index) {
                                mid_of = d;
                                is_mid = true;
                            }
                        }

                        if (index_l_r) continue;

                        double drone_trip = instance->tau_prime[solution->truck_route[start]][solution->truck_route[i]]
                            +
                            instance->tau_prime[solution->truck_route[i]][solution->truck_route[end]];
                        if (drone_trip > instance->e - instance->sr) continue;

                        if (start <= i && i <= end) {
                            if (start != i && end != i) {
                                double old_truck_trip = 0;
                                for (int l = start; l <= end - 1; ++l) {
                                    old_truck_trip += instance->tau[solution->truck_route[l]][solution->truck_route[l +
                                        1]];
                                }
                                double new_truck_trip = instance->tau[solution->truck_route[start]][solution->
                                    truck_route[end]];

                                double reduced_cost = old_truck_trip - new_truck_trip;
                                if (reduced_cost > best_reduced_cost) {
                                    best_reduced_cost = reduced_cost;
                                    best_index = i;
                                    best_start = start;
                                    best_end = end;
                                    best_truck_trip = new_truck_trip;
                                    best_drone_trip = drone_trip;
                                    customer_in_sortie_truck_route = false;
                                }
                            }
                        }
                        else {
                            double increased_cost = instance->tau[solution->truck_route[i - 1]][solution->truck_route[i
                                    + 1]] - instance->tau[solution->truck_route[i - 1]][solution->truck_route[i]] -
                                instance
                                ->tau[solution->truck_route[i]][solution->truck_route[i + 1]];

                            double new_truck_trip_for_sortie_affected = 0;
                            if (is_mid) {
                                new_truck_trip_for_sortie_affected = solution->sortie_stages[mid_of].truck_trip +
                                    increased_cost;
                                if (new_truck_trip_for_sortie_affected > instance->e - instance->sr) continue;
                            }

                            double old_truck_trip = 0;
                            for (int l = start; l <= end - 1; ++l) {
                                old_truck_trip += instance->tau[solution->truck_route[l]][solution->truck_route[l + 1]];
                            }
                            double new_truck_trip = instance->tau[solution->truck_route[start]][solution->truck_route[
                                end]];

                            double reduced_cost = old_truck_trip - new_truck_trip;
                            if (is_mid) reduced_cost += increased_cost;

                            if (reduced_cost > best_reduced_cost) {
                                best_reduced_cost = reduced_cost;
                                best_index = i;
                                best_start = start;
                                best_end = end;
                                best_truck_trip = new_truck_trip;
                                best_drone_trip = drone_trip;
                                customer_in_sortie_truck_route = true;
                                best_sortie_affected = mid_of;
                                best_new_truck_trip_sortie_affected = new_truck_trip_for_sortie_affected;
                            }
                        }
                    }
                }
            }
        }

        if (best_reduced_cost > -std::numeric_limits<double>::max()) {
            improvable = true;

            if (customer_in_sortie_truck_route) {
                solution->truck_route.erase(solution->truck_route.begin() + best_index);
                for (int d : solution->served_by_drone) {
                    if (solution->sortie_stages[d].start_index >= best_index) solution->sortie_stages[d].start_index--;
                    if (solution->sortie_stages[d].end_index >= best_index) solution->sortie_stages[d].end_index--;
                }

                solution->sortie_stages[best_sortie_affected].truck_trip = best_new_truck_trip_sortie_affected;
            }

            solution->truck_route.insert(solution->truck_route.begin() + best_end, solution->truck_route[best_index]);
            solution->served_by_drone.push_back(solution->truck_route[best_end]);
            solution->sortie_stages[solution->truck_route[best_end]].start_index = best_start;
            solution->sortie_stages[solution->truck_route[best_end]].end_index = best_end;
            solution->sortie_stages[solution->truck_route[best_end]].truck_trip = best_truck_trip;
            solution->sortie_stages[solution->truck_route[best_end]].drone_trip = best_drone_trip;

            for (auto& st : solution->sortie_stages) {
                if (st.start_index >= best_index) st.start_index++;
                if (st.end_index >= best_index) st.end_index++;
            }
        }
    }
}

void ALNS::BestCostInsert(std::shared_ptr<Solution>& solution) {
    std::vector<int> to_insert;
    to_insert.reserve(instance->C.size()); // Reserve space to avoid multiple allocations
    for (int i : instance->C) {
        if (!solution->visited[i]) {
            to_insert.push_back(i);
        }
    }
    if (to_insert.size() < 2) {
        lazy_insert_iter++;
    }

    const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);

    // Shuffle the vector
    std::ranges::shuffle(to_insert, mt);
    while (!to_insert.empty()) {
        // //solution->invalid_occurrence();
        // iterate through all, add best.
        int best_customer = -1, best_overall_truck_index = -1, best_overall_start_index = -1, best_overall_end_index = -
                1, best_overall_insert_into_sortie = -1;
        double best_overall_increased_cost = 1e9, best_overall_truck_trip = -1,
               best_overall_drone_trip = -1, best_overall_new_truck_cost_in_sortie = -1;
        bool insert_into_truck = true;
        for (int c : to_insert) {
            double best_drone_cost = 1e9, best_truck_cost = 1e9, new_truck_cost_for_sortie = -1;
            int best_start_index = -1, best_end_index = -1, best_truck_index = -1, insert_into_truck_route_of_sortie = -
                    1;

            for (int index = 1; index < solution->truck_route.size(); index++) {
                const int prev_customer = solution->truck_route[index - 1];
                const int next_customer = solution->truck_route[index];
                bool index_in_sortie = false;

                // Precompute tau values outside the drone loop
                const double current_trip_cost = instance->tau[prev_customer][next_customer];
                const double cost_to_c = instance->tau[prev_customer][c];
                const double cost_from_c_to_next = instance->tau[c][next_customer];

                for (const int d : solution->served_by_drone) {
                    // Check if the index is within this sortie stage
                    if (const auto& ss = solution->sortie_stages[d]; ss.start_index < index && index <= ss.end_index) {
                        index_in_sortie = true;

                        // Calculate the new truck cost if customer c is inserted
                        double new_truck_cost = ss.truck_trip - current_trip_cost + cost_to_c + cost_from_c_to_next;

                        // Break early if the new cost exceeds constraints
                        if (new_truck_cost > instance->e - instance->sr) {
                            break;
                        }

                        // Calculate the increased cost only if new_truck_cost is greater than the drone's cost
                        double increased_cost = (new_truck_cost > ss.drone_trip)
                                                    ? new_truck_cost - std::min(ss.truck_trip, ss.drone_trip)
                                                    : 0;

                        // Update the best cost and index if this is a better insertion
                        if (increased_cost < best_truck_cost) {
                            best_truck_index = index;
                            best_truck_cost = increased_cost;
                            insert_into_truck_route_of_sortie = d;
                            new_truck_cost_for_sortie = new_truck_cost;
                        }
                    }
                }

                // If the index is not part of any sortie, calculate the cost for direct insertion into truck route
                if (!index_in_sortie) {
                    double increased_cost = cost_to_c + cost_from_c_to_next - current_trip_cost;

                    if (increased_cost < best_truck_cost) {
                        best_truck_index = index;
                        best_truck_cost = increased_cost;
                        insert_into_truck_route_of_sortie = -1; // No sortie insertion
                    }
                }
            }

            double best_truck_trip = 1e9;
            double best_drone_trip = 1e9;

            auto spaces = solution->find_sortie_spaces();

            if (!instance->heavy_bool[c]) {
                for (const auto& [fst, snd] : spaces) {
                    // Iterate through valid starting points in sortie space
                    for (int start = fst; start <= snd - 1; ++start) {
                        double truck_travel_cost = 0.0;

                        // Precompute to avoid recalculations
                        const int start_customer = solution->truck_route[start];

                        for (int end = start + 1; end <= snd; ++end) {
                            const int prev_customer = solution->truck_route[end - 1];
                            const int next_customer = solution->truck_route[end];

                            // If consecutive customers in the truck route are the same, break early
                            if (start_customer == next_customer) {
                                break;
                            }

                            // Accumulate truck travel cost
                            truck_travel_cost += instance->tau[prev_customer][next_customer];

                            // Break early if the truck cost exceeds energy limit
                            if (truck_travel_cost > instance->e - instance->sr) {
                                break;
                            }

                            // Calculate drone travel cost
                            double drone_travel_cost = instance->tau_prime[start_customer][c] +
                                instance->tau_prime[c][next_customer];

                            // Skip if drone travel cost exceeds energy limit
                            if (drone_travel_cost > instance->e - instance->sr) {
                                continue;
                            }

                            // Calculate waiting time (only calculate max if needed)

                            // Update best costs if a better solution is found
                            if (double waiting_time = std::max(0.0, drone_travel_cost - truck_travel_cost); waiting_time
                                <= best_drone_cost) {
                                best_drone_cost = waiting_time;
                                best_start_index = start;
                                best_end_index = end;
                                best_drone_trip = drone_travel_cost;
                                best_truck_trip = truck_travel_cost;
                            }
                        }
                    }
                }
            }
            if (best_truck_cost <= best_drone_cost && best_truck_cost < best_overall_increased_cost) {
                best_overall_increased_cost = best_truck_cost;
                best_customer = c;
                best_overall_truck_index = best_truck_index;
                best_overall_insert_into_sortie = insert_into_truck_route_of_sortie;
                best_overall_new_truck_cost_in_sortie = new_truck_cost_for_sortie;
                insert_into_truck = true;
            }
            else if (best_drone_cost < best_truck_cost && best_drone_cost < best_overall_increased_cost) {
                best_overall_increased_cost = best_drone_cost;
                best_customer = c;
                best_overall_start_index = best_start_index;
                best_overall_end_index = best_end_index;
                best_overall_drone_trip = best_drone_trip;
                best_overall_truck_trip = best_truck_trip;
                insert_into_truck = false;
            }
        }
        if (best_customer != -1) {
            // got.
            if (insert_into_truck) {
                solution->truck_route.insert(solution->truck_route.begin() + best_overall_truck_index, best_customer);
                solution->visited[best_customer] = true;
                for (const int d : solution->served_by_drone) {
                    auto& sortie_stage = solution->sortie_stages[d];
                    if (sortie_stage.start_index >= best_overall_truck_index) {
                        sortie_stage.start_index++;
                        sortie_stage.end_index++;
                        continue;
                    }
                    if (sortie_stage.end_index >= best_overall_truck_index) {
                        sortie_stage.end_index++;
                    }
                }

                if (best_overall_insert_into_sortie > -1) {
                    solution->sortie_stages[best_overall_insert_into_sortie].truck_trip =
                        best_overall_new_truck_cost_in_sortie;
                }
                //solution->invalid_occurrence();
            }
            else {
                solution->served_by_drone.push_back(best_customer);
                solution->visited[best_customer] = true;
                solution->sortie_stages[best_customer] = {
                    best_overall_start_index, best_overall_end_index, best_overall_drone_trip, best_overall_truck_trip
                };
                //solution->invalid_occurrence();
            }
        }
        else {
            // have to flat. which one to flat?
            // Initialize variables to track the best sortie, indices, and costs
            int best_overall_which_sortie_to_flat = -1;
            int best_overall_index_for_flat = -1;
            bool best_overall_truck_after_flat = true;

            for (int c : to_insert) {
                double this_customer_best_truck_cost = 1e9, this_customer_best_drone_cost = 1e9;

                for (const int d : solution->served_by_drone) {
                    // Precompute sortie stage indices to avoid repeated access
                    const auto& sortie_stage = solution->sortie_stages[d];
                    int sortie_start = sortie_stage.start_index;
                    int sortie_end = sortie_stage.end_index;

                    // Initialize variables for tracking the best costs and indices within the current sortie
                    int best_flat_index = -1;
                    int best_truck_index_this_sortie = -1;
                    int best_start_index_this_sortie = -1;
                    int best_end_index_this_sortie = -1;

                    double this_best_flat_cost = 1e9;
                    double best_truck_trip_this_sortie = 0, best_drone_trip_this_sortie = 0;

                    // Calculate the best flat cost for the current sortie
                    for (int index = sortie_start + 1; index <= sortie_end; ++index) {
                        const double increased_cost = instance->tau[solution->truck_route[index - 1]][d] +
                            instance->tau[d][solution->truck_route[index]] -
                            instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]];

                        if (increased_cost < this_best_flat_cost) {
                            this_best_flat_cost = increased_cost;
                            best_flat_index = index;
                        }
                    }

                    // Copy the relevant part of the route once instead of every time in inner loops
                    std::vector<int> copy_route(solution->truck_route.begin() + sortie_start,
                                                solution->truck_route.begin() + sortie_end + 1);

                    // Insert the current sortie 'd' at the best flat index found
                    copy_route.insert(copy_route.begin() + best_flat_index - sortie_start, d);

                    // Evaluate costs for adding customer 'c' to the truck route
                    for (int i = 1; i < copy_route.size(); ++i) {
                        const double add_cost = this_best_flat_cost - instance->tau[copy_route[i - 1]][copy_route[i]] +
                            instance->tau[copy_route[i - 1]][c] + instance->tau[c][copy_route[i]];

                        if (add_cost < this_customer_best_truck_cost) {
                            this_customer_best_truck_cost = add_cost;
                            best_truck_index_this_sortie = sortie_start + i;
                        }
                    }

                    // Evaluate costs for adding customer 'c' to the drone route if not heavy
                    if (!instance->heavy_bool[c]) {
                        for (int l = 0; l < copy_route.size() - 1; ++l) {
                            double truck_cost = 0;

                            for (int r = l + 1; r < copy_route.size(); ++r) {
                                if (copy_route[l] == copy_route[r]) {
                                    break;
                                }

                                const double drone_cost = instance->tau_prime[copy_route[l]][c] +
                                    instance->tau_prime[c][copy_route[r]];

                                if (drone_cost > instance->e - instance->sr) {
                                    continue;
                                }

                                truck_cost += instance->tau[copy_route[r - 1]][copy_route[r]];
                                if (truck_cost > instance->e - instance->sr) {
                                    break;
                                }

                                const double wait = std::max(0.0, drone_cost - truck_cost);

                                if (const double add_cost = this_best_flat_cost + wait; add_cost <
                                    this_customer_best_drone_cost) {
                                    this_customer_best_drone_cost = add_cost;
                                    best_start_index_this_sortie = sortie_start + l;
                                    best_end_index_this_sortie = sortie_start + r;
                                    best_truck_trip_this_sortie = truck_cost;
                                    best_drone_trip_this_sortie = drone_cost;
                                }
                            }
                        }
                    }

                    // Compare and update the best overall cost between truck and drone routes
                    if (this_customer_best_truck_cost < this_customer_best_drone_cost) {
                        if (this_customer_best_truck_cost < best_overall_increased_cost) {
                            best_customer = c;
                            best_overall_increased_cost = this_customer_best_truck_cost;
                            best_overall_which_sortie_to_flat = d;
                            best_overall_index_for_flat = best_flat_index;
                            best_overall_truck_index = best_truck_index_this_sortie;
                            best_overall_truck_after_flat = true;
                        }
                    }
                    else {
                        if (this_customer_best_drone_cost < best_overall_increased_cost) {
                            best_customer = c;
                            best_overall_increased_cost = this_customer_best_drone_cost;
                            best_overall_which_sortie_to_flat = d;
                            best_overall_index_for_flat = best_flat_index;
                            best_overall_start_index = best_start_index_this_sortie;
                            best_overall_end_index = best_end_index_this_sortie;
                            best_overall_truck_after_flat = false;
                            best_overall_drone_trip = best_drone_trip_this_sortie;
                            best_overall_truck_trip = best_truck_trip_this_sortie;
                        }
                    }
                }
            }

            // update the solution.
            // std::erase(solution->served_by_drone, best_overall_which_sortie_to_flat);
            // for (const int d : solution->served_by_drone) {
            //     auto& sortie_stages = solution->sortie_stages[d];
            //     if (sortie_stages.start_index >= solution->sortie_stages[best_overall_which_sortie_to_flat].end_index) {
            //         sortie_stages.start_index++;
            //         sortie_stages.end_index++;
            //         continue;
            //     }
            //
            //     if (sortie_stages.end_index >= solution->sortie_stages[best_overall_which_sortie_to_flat].end_index) {
            //         sortie_stages.end_index++;
            //     }
            // }
            //
            // solution->truck_route.insert(solution->truck_route.begin() + best_overall_index_for_flat,
            //                              best_overall_which_sortie_to_flat);
            //
            // // Update the route and stages based on the best overall cost
            // if (best_overall_truck_after_flat || instance->heavy_bool[best_customer]) {
            //     solution->truck_route.insert(solution->truck_route.begin() + best_overall_truck_index, best_customer);
            //     solution->visited[best_customer] = true;
            //     for (const int d : solution->served_by_drone) {
            //         auto& sortie_stage = solution->sortie_stages[d];
            //         if (sortie_stage.start_index >= best_overall_truck_index) {
            //             sortie_stage.start_index++;
            //             sortie_stage.end_index++;
            //             continue;
            //         }
            //         if (sortie_stage.end_index >= best_overall_truck_index) {
            //             sortie_stage.end_index++;
            //         }
            //     }
            // }
            // else {
            //     solution->served_by_drone.push_back(best_customer);
            //     solution->sortie_stages[best_customer] = {
            //         best_overall_start_index, best_overall_end_index, best_overall_drone_trip, best_overall_truck_trip
            //     };
            //     solution->visited[best_customer] = true;
            // }
            // Remove the best sortie to flatten from the served_by_drone list
            std::erase(solution->served_by_drone, best_overall_which_sortie_to_flat);

            // Precompute the start and end indices of the flattened sortie
            const int flat_start_index = solution->sortie_stages[best_overall_which_sortie_to_flat].start_index;
            const int flat_end_index = solution->sortie_stages[best_overall_which_sortie_to_flat].end_index;

            // Update sortie stages indices after flattening
            for (const int d : solution->served_by_drone) {
                if (auto& sortie_stage = solution->sortie_stages[d]; sortie_stage.start_index >= flat_end_index) {
                    // Increment both start and end if the entire sortie is after the flattened one
                    sortie_stage.start_index++;
                    sortie_stage.end_index++;
                }
                else if (sortie_stage.end_index >= flat_end_index) {
                    // Increment only the end if it overlaps
                    sortie_stage.end_index++;
                }
            }

            // Insert the flattened sortie into the truck route
            solution->truck_route.insert(solution->truck_route.begin() + best_overall_index_for_flat,
                                         best_overall_which_sortie_to_flat);

            // If truck should take the customer or the customer is heavy, update the truck route
            if (best_overall_truck_after_flat || instance->heavy_bool[best_customer]) {
                solution->truck_route.insert(solution->truck_route.begin() + best_overall_truck_index, best_customer);
                solution->visited[best_customer] = true;

                // Update the sortie stages for other drones after inserting the customer into the truck route
                for (const int d : solution->served_by_drone) {
                    if (auto& sortie_stage = solution->sortie_stages[d]; sortie_stage.start_index >=
                        best_overall_truck_index) {
                        sortie_stage.start_index++;
                        sortie_stage.end_index++;
                    }
                    else if (sortie_stage.end_index >= best_overall_truck_index) {
                        sortie_stage.end_index++;
                    }
                }
            }
            else {
                // If the customer will be served by a drone, add them to the drone route
                solution->served_by_drone.push_back(best_customer);
                solution->sortie_stages[best_customer] = {
                    best_overall_start_index, best_overall_end_index, best_overall_drone_trip, best_overall_truck_trip
                };
                solution->visited[best_customer] = true;
            }
            //solution->invalid_occurrence();
        }
        if (auto it = std::ranges::find(to_insert, best_customer); it != to_insert.end()) {
            to_insert.erase(it);
        }
    }
    bool improvable = true;
    while (improvable) {
        improvable = false;
        std::unordered_map<int, int> countMap;
        std::vector<int> duplicateIndices;

        // First pass: Count occurrences of each element
        for (const auto& element : solution->truck_route) {
            countMap[element]++;
        }

        // Second pass: Collect indices of duplicates
        for (int i = 0; i < solution->truck_route.size(); ++i) {
            if (countMap[solution->truck_route[i]] > 1) {
                duplicateIndices.push_back(i);
            }
        }

        auto spaces = solution->find_sortie_spaces();

        std::vector<int> possible_candidate_indexes;
        std::vector<int> candidate_belongs_to_sortie(instance->num_node, 0);
        std::vector new_truck_trip_without_candidate(
            instance->num_node, std::vector<double>(instance->num_node));
        for (const int d : solution->served_by_drone) {
            for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].end_index -
                 1; index++) {
                if (!exist(duplicateIndices, index) && !instance->heavy_bool[solution->truck_route[index]]) {
                    if (const double new_truck_trip = solution->sortie_stages[d].truck_trip
                            - instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]]
                            - instance->tau[solution->truck_route[index]][solution->truck_route[index + 1]]
                            + instance->tau[solution->truck_route[index - 1]][solution->truck_route[index + 1]];
                        new_truck_trip <= instance->e - instance->sr) {
                        possible_candidate_indexes.emplace_back(index);
                        candidate_belongs_to_sortie[index] = d;
                        new_truck_trip_without_candidate[d][index] = new_truck_trip;
                    }
                }
            }
        }


        for (const auto& [fst, snd] : spaces) {
            for (int index = fst + 1; index <= snd - 1; index++) {
                if (!exist(duplicateIndices, index)) {
                    if (!exist(duplicateIndices, index) && !instance->heavy_bool[solution->truck_route[index]]) {
                        possible_candidate_indexes.emplace_back(index);
                    }
                }
            }
        }

        // got all possible candidate?
        int best_index_candidate = -1, best_start_index = -1, best_end_index = -1;
        double best_truck_trip = -1, best_drone_trip = -1, best_increased_cost = 0;

        for (const int candidate : possible_candidate_indexes) {
            // tinh increased cost cho sortie/quang duong bi anh huong (neu co)
            double increased_cost = candidate_belongs_to_sortie[candidate] != 0
                                        ? (std::max(
                                            new_truck_trip_without_candidate[candidate_belongs_to_sortie[candidate]][
                                                candidate],
                                            solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip))
                                        - std::max(
                                            solution->sortie_stages[candidate_belongs_to_sortie[candidate]].truck_trip,
                                            solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip)
                                        : -instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[
                                            candidate]]
                                        - instance->tau[solution->truck_route[candidate]][solution->truck_route[
                                            candidate + 1]]
                                        + instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[
                                            candidate + 1]];

            // tinh increased cost cho viec tao sortie moi
            for (const auto& [fst, snd] : spaces) {
                for (int start = fst; start <= snd - 1; start++) {
                    double truck_trip = 0;
                    for (int end = start + 1; end <= snd; end++) {
                        if (solution->truck_route[start] == solution->truck_route[end]) {
                            break;
                        }
                        truck_trip += instance->tau[solution->truck_route[end - 1]][solution->truck_route[end]];
                        if (truck_trip >= instance->e - instance->sr) {
                            break;
                        }
                        // dua candidate nay vao sortie.
                        if (const double drone_trip = instance->tau_prime[solution->truck_route[start]][solution->
                                    truck_route[candidate]]
                                + instance->tau_prime[solution->truck_route[candidate]][solution->truck_route[end]];
                            drone_trip <= instance->e - instance->sr) {
                            // increased_cost when creating this sortie?
                            if (drone_trip > truck_trip) {
                                if (const double inc = increased_cost + (drone_trip - truck_trip); inc <
                                    best_increased_cost) {
                                    best_increased_cost = inc;
                                    improvable = true;
                                    best_index_candidate = candidate;
                                    best_start_index = start;
                                    best_end_index = end;
                                    best_truck_trip = truck_trip;
                                    best_drone_trip = drone_trip;
                                }
                            }
                        }
                    }
                }
            }
        }
        // found the best.
        if (improvable) {
            //solution->invalid_occurrence();
            solution->served_by_drone.push_back(solution->truck_route[best_index_candidate]);
            solution->sortie_stages[solution->truck_route[best_index_candidate]] = {
                best_start_index, best_end_index, best_drone_trip, best_truck_trip
            };
            // update sortie phia sau candidate index nay
            for (const int d : solution->served_by_drone) {
                if (solution->sortie_stages[d].start_index > best_index_candidate) {
                    solution->sortie_stages[d].start_index--;
                    solution->sortie_stages[d].end_index--;
                    continue;
                }
                if (solution->sortie_stages[d].end_index > best_index_candidate) {
                    solution->sortie_stages[d].end_index--;
                }
            }
            solution->sortie_stages[candidate_belongs_to_sortie[best_index_candidate]].truck_trip =
                new_truck_trip_without_candidate[candidate_belongs_to_sortie[best_index_candidate]][
                    best_index_candidate];
            solution->truck_route.erase(solution->truck_route.begin() + best_index_candidate);
            //solution->invalid_occurrence();
        }
    }
}


void ALNS::RandomSortieRemove(std::shared_ptr<Solution>& solution) {
    if (!solution->served_by_drone.empty()) {
        std::uniform_int_distribution<int> to_remove_dist(1, solution->served_by_drone.size());

        const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine rng(seed);
        const int to_remove = to_remove_dist(mt);
        // Shuffle the vector

        // shuffle.
        std::ranges::shuffle(solution->served_by_drone, mt);
        std::vector<int> to_be_removed(to_remove);
        std::copy_n(solution->served_by_drone.begin(), to_remove, to_be_removed.begin());

        for (const int c : to_be_removed) {
            solution->remove_sortie(c);
        }
    }
}

void ALNS::SortieLocalSearch(std::shared_ptr<Solution>& solution) {
    bool improvable = true;
    while (improvable) {
        improvable = false;
        std::unordered_map<int, int> countMap;
        std::vector<int> duplicateIndices;

        // First pass: Count occurrences of each element
        for (const auto& element : solution->truck_route) {
            countMap[element]++;
        }

        // Second pass: Collect indices of duplicates
        for (int i = 0; i < solution->truck_route.size(); ++i) {
            if (countMap[solution->truck_route[i]] > 1) {
                duplicateIndices.push_back(i);
            }
        }

        auto spaces = solution->find_sortie_spaces();
        std::vector<int> possible_candidate_indexes;
        std::vector<int> candidate_belongs_to_sortie(instance->num_node, 0);
        std::vector new_truck_trip_without_candidate(
            200, std::vector<double>(200));
        for (const int d : solution->served_by_drone) {
            for (int index = solution->sortie_stages[d].start_index + 1; index <= solution->sortie_stages[d].end_index -
                 1; index++) {
                if (!exist(duplicateIndices, index) && !
                    exist(solution->served_by_drone, solution->truck_route[index])) {
                    if (!instance->heavy_bool[solution->truck_route[index]]) {
                        if (const double new_truck_trip = solution->sortie_stages[d].truck_trip
                                - instance->tau[solution->truck_route[index - 1]][solution->truck_route[index]]
                                - instance->tau[solution->truck_route[index]][solution->truck_route[index + 1]]
                                + instance->tau[solution->truck_route[index - 1]][solution->truck_route[index + 1]];
                            new_truck_trip <= instance->e - instance->sr) {
                            possible_candidate_indexes.emplace_back(index);
                            candidate_belongs_to_sortie[index] = d;
                            new_truck_trip_without_candidate[d][index] = new_truck_trip;
                        }
                    }
                }
            }
        }


        for (const auto& [fst, snd] : spaces) {
            for (int index = fst + 1; index <= snd - 1; index++) {
                if (!exist(duplicateIndices, index)) {
                    if (!instance->heavy_bool[solution->truck_route[index]] && !exist(
                        solution->served_by_drone, solution->truck_route[index])) {
                        possible_candidate_indexes.emplace_back(index);
                    }
                }
            }
        }

        // got all possible candidate?
        int best_index_candidate = -1, best_start_index = -1, best_end_index = -1;
        double best_truck_trip = -1, best_drone_trip = -1, best_increased_cost = 0;

        for (const int candidate : possible_candidate_indexes) {
            // tinh increased cost cho sortie/quang duong bi anh huong (neu co)
            double increased_cost = 0;
            if (candidate_belongs_to_sortie[candidate] != 0) {
                // = new_cost - current_cost
                increased_cost += (std::max(
                        new_truck_trip_without_candidate[candidate_belongs_to_sortie[candidate]][candidate],
                        solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip))
                    - std::max(solution->sortie_stages[candidate_belongs_to_sortie[candidate]].truck_trip,
                               solution->sortie_stages[candidate_belongs_to_sortie[candidate]].drone_trip);
            }
            else {
                increased_cost += -instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[candidate]]
                    - instance->tau[solution->truck_route[candidate]][solution->truck_route[candidate + 1]]
                    + instance->tau[solution->truck_route[candidate - 1]][solution->truck_route[candidate + 1]];
            }
            // tinh increased cost cho viec tao sortie moi
            for (const auto& [fst, snd] : spaces) {
                for (int start = fst; start <= snd - 1; start++) {
                    double truck_trip = 0;
                    for (int end = start + 1; end <= snd; end++) {
                        if (solution->truck_route[start] == solution->truck_route[end]) {
                            break;
                        }
                        truck_trip += instance->tau[solution->truck_route[end - 1]][solution->truck_route[end]];
                        if (truck_trip >= instance->e - instance->sr) {
                            break;
                        }
                        // dua candidate nay vao sortie.
                        if (const double drone_trip = instance->tau_prime[solution->truck_route[start]][solution->
                                    truck_route[candidate]]
                                + instance->tau_prime[solution->truck_route[candidate]][solution->truck_route[end]];
                            drone_trip <= instance->e - instance->sr) {
                            // increased_cost when creating this sortie?
                            if (drone_trip > truck_trip) {
                                if (const double inc = increased_cost + (drone_trip - truck_trip); inc <
                                    best_increased_cost) {
                                    best_increased_cost = inc;
                                    improvable = true;
                                    best_index_candidate = candidate;
                                    best_start_index = start;
                                    best_end_index = end;
                                    best_truck_trip = truck_trip;
                                    best_drone_trip = drone_trip;
                                }
                            }
                        }
                    }
                }
            }
        }
        // found the best.
        if (improvable) {
            solution->served_by_drone.push_back(solution->truck_route[best_index_candidate]);
            solution->sortie_stages[solution->truck_route[best_index_candidate]] = {
                best_start_index, best_end_index, best_drone_trip, best_truck_trip
            };
            // update sortie phia sau candidate index nay
            for (const int d : solution->served_by_drone) {
                if (solution->sortie_stages[d].start_index > best_index_candidate) {
                    solution->sortie_stages[d].start_index--;
                }
                if (solution->sortie_stages[d].end_index > best_index_candidate) {
                    solution->sortie_stages[d].end_index--;
                }
            }
            solution->sortie_stages[candidate_belongs_to_sortie[best_index_candidate]].truck_trip =
                new_truck_trip_without_candidate[candidate_belongs_to_sortie[best_index_candidate]][
                    best_index_candidate];
            solution->truck_route.erase(solution->truck_route.begin() + best_index_candidate);
        }
    }
}
