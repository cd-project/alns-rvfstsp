//
// Created by cuong on 5/9/24.
//

#ifndef SOLUTION_H
#define SOLUTION_H

#include <algorithm>
#include <memory>
#include <unordered_set>

#include "../instance.h"

inline bool exist(const std::vector<int>& vec, int element) {
    // Use std::find to search for the element in the vector
    return std::ranges::find(vec, element) != vec.end();
}

struct Sortie {
    int start_truck_index;
    // int launch;
    int customer;
    // int rendezvous;
    int end_truck_index;

    Sortie(const int c, const int s, const int e) {
        // launch = l;
        customer = c;
        // rendezvous = r;
        start_truck_index = s;
        end_truck_index = e;
    }
};

struct SortieInfo {
    int start_index;
    int end_index;
    double drone_trip;
    double truck_trip;

    SortieInfo(const int s, const int e, const double dt, const double tt) {
        start_index = s;
        end_index = e;
        drone_trip = dt;
        truck_trip = tt;
    }
};


class Solution {
public:
    std::shared_ptr<Instance> instance;
    std::vector<int> truck_route;
    std::vector<bool> visited;
    std::vector<int> served_by_drone;
    // std::vector<Sortie> sorties;
    double objective;
    explicit Solution(const std::shared_ptr<Instance>& instance);
    std::shared_ptr<Solution> copy();
    void calculate_objective();
    void print_solution_light();
    std::vector<SortieInfo> sortie_stages;
    // feasibility checkers
    bool feasibility_check();
    void sortie_feasibility_check();
    void invalid_occurrence();
    void sortie_invalid_occurrence();


    // remover
    void remove_customer(int customer_to_remove);
    void remove_truck_index_loop_no_battery(int index);
    void remove_customer_loop_no_battery(int customer_to_remove);
    bool remove_truck_index(int index);
    void remove_drone_customer(int c);
    void remove_sortie(int c);
    int remove_highest_rate_sortie(int c);
    void remove_string(int start_index, int remove_size);
    void duplicate_node_cleaner();


    // truck local search algorithms
    void two_opt();
    void swap();
    int swap_mid_only();
    void relocate();

    // Revisit
    void revisit();

    // Loop


    // utils
    void swap_op(int i, int j);
    void revisit_loop_no_battery();
    int nearest_sortie_start_index(int index);
    [[nodiscard]] double get_truck_cost(const int& index1, const int& index2) const;
    void print_solution();
    void check_duplicates(const std::vector<int>& vec);
    void print_truck_tour();
    void easy_sortie_update();
    void loop_check();

    std::vector<std::pair<int, int>> spaces_separator(const int space_start_index, const int space_end_index) {
        std::vector<std::pair<int, int>> return_spaces;
        int start_index = space_start_index;
        bool end = false;
        while (!end) {
            std::vector<int> current_nodes;
            current_nodes.push_back(truck_route[start_index]);
            for (int current_index = start_index + 1; current_index <= space_end_index; current_index++) {
                if (current_index == space_end_index) {
                    if (!exist(current_nodes, truck_route[current_index])) {
                        return_spaces.emplace_back(start_index, current_index);
                    }
                    else {
                        return_spaces.insert(return_spaces.end(), {
                                                 {start_index, current_index - 1}, {current_index - 1, current_index}
                                             });
                    }
                    end = true;
                    break;
                }
                if (!exist(current_nodes, truck_route[current_index])) {
                    current_nodes.push_back(truck_route[current_index]);
                }
                else {
                    // meet.

                    return_spaces.emplace_back(start_index, current_index - 1);
                    start_index = current_index;
                    break;
                }
            }
        }
        return return_spaces;
    }

    std::vector<std::pair<int, int>> find_sortie_spaces() {
        // Reserve space for vectors to avoid multiple reallocations
        std::vector<std::pair<int, int>> spaces;
        spaces.reserve(served_by_drone.size() + 1); // Max possible spaces

        // Early exit if no sorties are served by drones
        if (served_by_drone.empty()) {
            return spaces_separator(0, truck_route.size() - 1);
            return spaces;
        }

        // Create a vector of indices instead of copying SortieInfo objects
        std::vector<int> sortie_indices;
        sortie_indices.reserve(served_by_drone.size());
        for (const int d : served_by_drone) {
            sortie_indices.push_back(d);
        }

        // Sort the sortie indices based on their start_index
        std::ranges::sort(sortie_indices, [&](const int a, const int b) {
            return sortie_stages[a].start_index < sortie_stages[b].start_index;
        });

        // Check space before the first sortie
        if (const SortieInfo& first_sortie = sortie_stages[sortie_indices[0]]; first_sortie.start_index > 0) {
            const auto& temp = spaces_separator(0, first_sortie.start_index);

            spaces.insert(spaces.end(), std::make_move_iterator(temp.begin()), std::make_move_iterator(temp.end()));
        }

        // Check spaces between sorties
        for (size_t i = 0; i < sortie_indices.size() - 1; ++i) {
            const SortieInfo& current_sortie = sortie_stages[sortie_indices[i]];

            if (const SortieInfo& next_sortie = sortie_stages[sortie_indices[i + 1]]; current_sortie.end_index <
                next_sortie.start_index) {
                const auto& temp = spaces_separator(current_sortie.end_index, next_sortie.start_index);
                spaces.insert(spaces.end(), std::make_move_iterator(temp.begin()), std::make_move_iterator(temp.end()));
            }
        }

        // Check space after the last sortie
        if (const SortieInfo& last_sortie = sortie_stages[sortie_indices.back()]; last_sortie.end_index < truck_route.
            size() - 1) {
            const auto& temp = spaces_separator(last_sortie.end_index, truck_route.size() - 1);
            spaces.insert(spaces.end(), std::make_move_iterator(temp.begin()), std::make_move_iterator(temp.end()));
        }

        return spaces;
    }
};

#endif //SOLUTION_H
