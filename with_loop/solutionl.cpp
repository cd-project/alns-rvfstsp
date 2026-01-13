#include "../with_loop/solutionl.h"

#include <cassert>
#include <cmath>
#include <strings.h>
#include <unordered_map>
#include <unordered_set>
//
// Created by cuong on 5/9/24.
//
// inline bool exist(const std::vector<int>& vec, int element) {
//     // Use std::find to search for the element in the vector
//     return std::find(vec.begin(), vec.end(), element) != vec.end();
// }

SolutionL::SolutionL(const std::shared_ptr<Instance>& instance) {
    this->instance = instance;
    visited.resize(instance->num_node);
    visited[0] = true;
    visited[instance->num_node] = true;
    truck_route.push_back(0);
    truck_route.push_back(instance->num_node);
    sortie_stages.resize(instance->num_node, SortieInfoL{-1, -1, -1, -1});
}

std::shared_ptr<SolutionL> SolutionL::copy() {
    auto new_solution = std::make_shared<SolutionL>(instance);
    new_solution->truck_route = truck_route;
    new_solution->served_by_drone = served_by_drone;
    new_solution->sortie_stages = sortie_stages;
    new_solution->visited = visited;
    new_solution->objective = objective;
    return new_solution;
}

double SolutionL::get_truck_cost(const int& index1, const int& index2) const {
    double cost = 0;
    for (int i = index1; i < index2; i++) {
        cost += instance->tau[truck_route[i]][truck_route[i + 1]];
    }
    return cost;
}

// Calculate current solution's objective value.
void SolutionL::calculate_objective() {
    double obj = 0;
    // truck tour length
    for (int i = 0; i < truck_route.size() - 1; i++) {
        obj += instance->tau[truck_route[i]][truck_route[i + 1]];
    }

    for (const int& d : served_by_drone) {
        if (sortie_stages[d].start_index != 0) {
            obj += instance->sl;
        }
        const double w = std::max(0.0, sortie_stages[d].drone_trip - sortie_stages[d].truck_trip);
        obj += w;
        obj += instance->sr;
    }
    // sortie time calculation

    objective = obj;
}

void SolutionL::print_solution_light() {
    std::cout << "----------------------------PS-------------------------------" << std::endl;
    std::cout << "Truck routes: ";
    for (int x : truck_route) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
    std::cout << "Served by drone: ";
    for (int& d : served_by_drone) {
        std::cout << d << " ";
    }
    std::cout << std::endl;

    std::cout << "Number of sorties: " << served_by_drone.size() << std::endl;

    for (const int& d : served_by_drone) {
        std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
        std::cout << "Start index: " << sortie_stages[d].start_index << " at node " << truck_route[sortie_stages[d].
                start_index] <<
            std::endl;
        std::cout << "End index: " << sortie_stages[d].end_index << " at node " << truck_route[sortie_stages[d].
            end_index] << std::endl;
        std::cout << "Customer: " << d << std::endl;
        std::cout << "Truck-trip: " << sortie_stages[d].truck_trip << std::endl;
        std::cout << "Drone-trip: " << sortie_stages[d].drone_trip << std::endl;


        // double re_drone =
        std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
    }

    std::cout << "Not visited customer: ";
    for (int i = 1; i < visited.size(); i++) {
        if (!visited[i]) {
            std::cout << i << " ";
        }
    }
    std::cout << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
}

void SolutionL::print_solution() {
    std::cout << "----------------------------PS-------------------------------" << std::endl;
    std::cout << "Truck routes: ";
    for (int x : truck_route) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
    std::cout << "Served by drone: ";
    for (int& d : served_by_drone) {
        std::cout << d << " ";
    }
    std::cout << std::endl;
    double truck_length = 0;
    for (int i = 0; i < truck_route.size() - 1; i++) {
        std::cout << "from " << truck_route[i] << "->" << truck_route[i + 1] << ": " << instance->tau[truck_route[i]][
            truck_route[i + 1]] << std::endl;
        truck_length += instance->tau[truck_route[i]][truck_route[i + 1]];
    }
    std::cout << "Truck route length: " << truck_length << std::endl;
    std::cout << "Number of sorties: " << served_by_drone.size() << std::endl;

    for (const int& d : served_by_drone) {
        std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
        std::cout << "Start index: " << sortie_stages[d].start_index << " at node " << truck_route[sortie_stages[d].
                start_index] <<
            std::endl;
        std::cout << "End index: " << sortie_stages[d].end_index << " at node " << truck_route[sortie_stages[d].
            end_index] << std::endl;
        std::cout << "Customer: " << d << std::endl;
        std::cout << "Truck-trip: " << sortie_stages[d].truck_trip << std::endl;
        std::cout << "Drone-trip: " << sortie_stages[d].drone_trip << std::endl;


        // double re_drone =
        std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
    }

    std::cout << "Not visited customer: ";
    for (int i = 1; i < visited.size(); i++) {
        if (!visited[i]) {
            std::cout << i << " ";
        }
    }
    std::cout << std::endl;
    calculate_objective();
    std::cout << "Objective value: " << objective << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
}

void SolutionL::check_duplicates(const std::vector<int>& vec) {
    if (!vec.empty()) {
        const int maxValue = *std::ranges::max_element(vec);
        const int minValue = *std::ranges::min_element(vec);

        // Create a boolean lookup table for all possible values in the vector
        std::vector<bool> seen(maxValue - minValue + 1, false);

        for (const int num : vec) {
            if (seen[num - minValue]) {
                throw std::runtime_error("Error: Duplicate element found in the vector.");
            }
            seen[num - minValue] = true;
        }
    }
}

bool SolutionL::feasibility_check() {
    std::cout << "Starting solution feasibility check..." << std::endl;
    std::cout << "current solution:" << std::endl;
    print_solution();
    if (truck_route.size() > instance->num_node + 1) {
        throw std::runtime_error("bugged truck route: too many nodes");
    }
    if (truck_route[0] != 0 || truck_route[truck_route.size() - 1] != instance->num_node) {
        throw std::runtime_error("Truck route start/end bugged");
    }
    // std::cout << "Checking for unvisited node..." << std::endl;
    for (int i = 0; i < instance->num_node; i++) {
        bool v = false;
        for (auto& x : truck_route) {
            if (x == i) {
                v = true;
            }
        }
        for (auto& x : served_by_drone) {
            if (x == i) {
                v = true;
            }
        }
        if (!v) {
            print_solution();
            throw std::runtime_error("Node " + std::to_string(i) + " was not visited!");
        }
    }

    // check sortie only?
    // check if customer in sortie in heavy.
    // std::cout << "Checking duplicate drone nodes..." << std::endl;
    check_duplicates(served_by_drone);
    // std::cout << "Check customer served by both vehicles..." << std::endl;
    for (int cus : truck_route) {
        if (existL(served_by_drone, cus)) {
            throw std::runtime_error(std::to_string(cus) + " served by both!");
        }
    }
    std::vector<int> to_insert;
    to_insert.reserve(instance->C.size()); // Reserve space to avoid multiple allocations
    for (int i : instance->C) {
        if (!visited[i]) {
            to_insert.push_back(i);
        }
    }
    for (const int i : to_insert) {
        if (existL(truck_route, i)) {
            throw std::runtime_error("Insert " + std::to_string(i) + " is already in the truck route.");
        }
        if (existL(served_by_drone, i)) {
            throw std::runtime_error("Insert " + std::to_string(i) + " is already in the drone route.");
        }
    }
    for (const int& d : served_by_drone) {
        // std::cout << "Checking sortie " << d << std::endl;
        // std::unordered_map<int, int> countMap;
        // std::vector<int> duplicateIndices;
        //
        // // First pass: Count occurrences of each element
        // for (int i = sortie_stages[d].start_index; i <= sortie_stages[d].end_index; i++) {
        //     countMap[truck_route[i]]++;
        // }
        //
        // // Second pass: Collect indices of duplicates
        // for (int i = sortie_stages[d].start_index; i <= sortie_stages[d].end_index; i++) {
        //     if (countMap[truck_route[i]] > 1) {
        //         duplicateIndices.push_back(i);
        //     }
        // }
        if (sortie_stages[d].end_index - sortie_stages[d].start_index >= 2) {
            for (int i = sortie_stages[d].start_index + 1; i < sortie_stages[d].end_index; i++) {
                if (truck_route[i] == truck_route[sortie_stages[d].start_index] || truck_route[i] == truck_route[
                    sortie_stages[d].end_index]) {
                    throw std::runtime_error("invalid duplicate.");
                }
            }
        }
        // if (!duplicateIndices.empty()) {
        //     throw std::runtime_error("Error: Duplicate element found in the sortie.");
        // }
        assert(sortie_stages[d].start_index < sortie_stages[d].end_index);
        // assert(truck_route[sortie_stages[d].start_index] != truck_route[sortie_stages[d].end_index]);
        assert(sortie_stages[d].drone_trip <= instance->e - instance->sr);
        assert(sortie_stages[d].truck_trip <= instance->e - instance->sr);
        assert(sortie_stages[d].truck_trip >= 0);
        assert(sortie_stages[d].drone_trip > 0);
        assert(instance->heavy_bool[d] == false);
        double re_truck = 0;
        for (int start = sortie_stages[d].start_index; start <= sortie_stages[d].end_index - 1; start++) {
            re_truck += instance->tau[truck_route[start]][truck_route[start + 1]];
        }
        const double re_drone = instance->tau_prime[truck_route[sortie_stages[d].start_index]][d]
            + instance->tau_prime[d][truck_route[sortie_stages[d].end_index]];
        if (fabs(re_truck - sortie_stages[d].truck_trip) > 1e-4) {
            throw std::runtime_error(
                "re_truck = " + std::to_string(re_truck) + " while truck_trip = " + std::to_string(
                    sortie_stages[d].truck_trip));
        }
        if (fabs(re_drone - sortie_stages[d].drone_trip) > 1e-4) {
            throw std::runtime_error(
                "re_drone = " + std::to_string(re_truck) + " while drone_trip = " + std::to_string(
                    sortie_stages[d].drone_trip));
        }
    }

    // std::cout << "Checking for crossing sorties: " << std::endl;
    for (const int d : served_by_drone) {
        for (const int d1 : served_by_drone) {
            if (d != d1) {
                if (sortie_stages[d1].start_index > sortie_stages[d].start_index && sortie_stages[d1].start_index <
                    sortie_stages[d].end_index) {
                    throw std::runtime_error("Crossing! Printing solution before exiting...");
                }
                if (sortie_stages[d1].end_index > sortie_stages[d].start_index && sortie_stages[d1].end_index <
                    sortie_stages[d].end_index) {
                    throw std::runtime_error("Crossing! Printing solution before exiting...");
                }
            }
        }
    }
    std::cout << "Solution is feasible." << std::endl;
    return true;
}

void SolutionL::print_truck_tour() {
    std::cout << "Printing truck tour: ";
    for (int i = 0; i < truck_route.size(); i++) {
        std::cout << truck_route[i] << " ";
    }
    std::cout << std::endl;
}


void SolutionL::sortie_feasibility_check() {
    std::cout << "Starting sortie-only feasibility check: " << std::endl;
    print_solution();
    if (truck_route.size() > instance->num_node + 1) {
        throw std::runtime_error("bugged truck route: too many nodes");
    }
    if (truck_route[0] != 0 || truck_route[truck_route.size() - 1] != instance->num_node) {
        throw std::runtime_error("Truck route start/end bugged");
    }
    check_duplicates(served_by_drone);
    std::vector<int> to_insert;
    to_insert.reserve(instance->C.size()); // Reserve space to avoid multiple allocations
    for (int i : instance->C) {
        if (!visited[i]) {
            to_insert.push_back(i);
        }
    }
    for (const int i : to_insert) {
        if (existL(truck_route, i)) {
            throw std::runtime_error("Insert " + std::to_string(i) + " is already in the truck route.");
        }
        if (existL(served_by_drone, i)) {
            throw std::runtime_error("Insert " + std::to_string(i) + " is already in the drone route.");
        }
    }
    for (int i = 0; i <= instance->num_node; i++) {
        // check for missing node.
        if (!existL(truck_route, i) && !existL(served_by_drone, i) && visited[i]) {
            throw std::runtime_error("Node " + std::to_string(i) + " is not in truck route or served by drone");
        }
        if (existL(truck_route, i) && existL(served_by_drone, i)) {
            throw std::runtime_error("Node " + std::to_string(i) + " is in both truck and drone route");
        }
    }
    for (const int& d : served_by_drone) {
        if (sortie_stages[d].start_index < 0 || sortie_stages[d].start_index >= truck_route.size() - 1) {
            throw std::runtime_error("Start index is bugged!");
        }
        if (sortie_stages[d].end_index <= 0 || sortie_stages[d].end_index > truck_route.size() - 1) {
            throw std::runtime_error("end index is bugged!");
        }
        // if (sortie_stages[d].end_index - sortie_stages[d].start_index >= 2) {
        //     for (int i = sortie_stages[d].start_index + 1; i < sortie_stages[d].end_index; i++) {
        //         if (truck_route[i] == truck_route[sortie_stages[d].start_index] || truck_route[i] == truck_route[
        //             sortie_stages[d].end_index]) {
        //             throw std::runtime_error("invalid duplicate.");
        //         }
        //     }
        // }

        if (sortie_stages[d].start_index >= sortie_stages[d].end_index) {
            throw std::runtime_error("Sortie " + std::to_string(d) + " has start index " + std::to_string(
                sortie_stages[d].start_index) + " geq than end index " + std::to_string(sortie_stages[d].end_index));
        }
        // assert(sortie_stages[d].start_index < sortie_stages[d].end_index);
        // assert(truck_route[sortie_stages[d].start_index] != truck_route[sortie_stages[d].end_index]);
        assert(sortie_stages[d].drone_trip <= instance->e - instance->sr);
        assert(sortie_stages[d].truck_trip <= instance->e - instance->sr);
        assert(sortie_stages[d].truck_trip >= -0.00001);
        assert(sortie_stages[d].drone_trip > 0);
        assert(instance->heavy_bool[d] == false);
        double re_truck = 0;
        for (int start = sortie_stages[d].start_index; start <= sortie_stages[d].end_index - 1; start++) {
            re_truck += instance->tau[truck_route[start]][truck_route[start + 1]];
        }
        const double re_drone = instance->tau_prime[truck_route[sortie_stages[d].start_index]][d]
            + instance->tau_prime[d][truck_route[sortie_stages[d].end_index]];
        if (fabs(re_truck - sortie_stages[d].truck_trip) > 1e-4) {
            throw std::runtime_error(
                "re_truck = " + std::to_string(re_truck) + " while truck_trip = " + std::to_string(
                    sortie_stages[d].truck_trip));
        }
        if (fabs(re_drone - sortie_stages[d].drone_trip) > 1e-4) {
            throw std::runtime_error(
                "re_drone = " + std::to_string(re_truck) + " while drone_trip = " + std::to_string(
                    sortie_stages[d].drone_trip));
        }
    }

    for (const int d : served_by_drone) {
        for (const int d1 : served_by_drone) {
            if (d != d1) {
                if (sortie_stages[d1].start_index > sortie_stages[d].start_index && sortie_stages[d1].start_index <
                    sortie_stages[d].end_index) {
                    throw std::runtime_error("Crossing! Printing solution before exiting...");
                }
                if (sortie_stages[d1].end_index > sortie_stages[d].start_index && sortie_stages[d1].end_index <
                    sortie_stages[d].end_index) {
                    throw std::runtime_error("Crossing! Printing solution before exiting...");
                }
            }
        }
    }
    std::cout << "Check completed. Feasible sorties." << std::endl;
}


void SolutionL::remove_drone_customer(const int c) {
    const int start_node = truck_route[sortie_stages[c].start_index];
    const int end_node = truck_route[sortie_stages[c].end_index];
    int count_start = 0, count_end = 0;
    for (int i = 1; i < truck_route.size() - 1; i++) {
        if (truck_route[i] == start_node) {
            count_start++;
        }
        if (truck_route[i] == end_node) {
            count_end++;
        }
    }
    std::erase(served_by_drone, c);
    visited[c] = false;
    if (count_end > 1) {
        remove_truck_index(sortie_stages[c].end_index);
    }
    if (count_start > 1) {
        remove_truck_index(sortie_stages[c].start_index);
    }
}

inline bool isElementInArray(const std::vector<int>& arr, int c) {
    // Chuyển đổi mảng thành std::unordered_set
    std::unordered_set<int> set(arr.begin(), arr.end());
    // Kiểm tra xem phần tử c có tồn tại trong tập hợp không
    return set.find(c) != set.end();
}


void SolutionL::remove_customer(const int customer_to_remove) {
    if (isElementInArray(served_by_drone, customer_to_remove)) {
        remove_drone_customer(customer_to_remove);
    }
    else {
        std::vector<int> customer_indexes;
        for (int i = truck_route.size() - 1; i > 0; --i) {
            if (truck_route[i] == customer_to_remove) {
                customer_indexes.push_back(i);
            }
        }
        for (const int index : customer_indexes) {
            if (truck_route[index - 1] == truck_route[index + 1]) {
                continue;
            }
            bool is_middle = false, is_l = false, is_r = false;
            int l_of = -1, r_of = -1, mid_of = -1;

            for (const int& d : served_by_drone) {
                const auto& ss = sortie_stages[d];
                if (index > ss.start_index && index < ss.end_index) {
                    is_middle = true;
                    mid_of = d;
                }
                if (index == ss.start_index) {
                    is_l = true;
                    l_of = d;
                }
                if (index == ss.end_index) {
                    is_r = true;
                    r_of = d;
                }
            }

            if (!is_middle && !is_l && !is_r) {
                for (const int& d : served_by_drone) {
                    if (sortie_stages[d].start_index > index) {
                        sortie_stages[d].start_index--;
                    }
                    if (sortie_stages[d].end_index > index) {
                        sortie_stages[d].end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + index);
                continue;
            }

            if (is_l && is_r) {
                const int r_of_l_customer = sortie_stages[l_of].end_index;
                const int l_of_r_customer = sortie_stages[r_of].start_index;

                if (truck_route[index + 1] == truck_route[l_of_r_customer] || truck_route[index + 1] == truck_route[
                    r_of_l_customer]) {
                    continue;
                }
                if (sortie_stages[l_of].end_index - sortie_stages[l_of].start_index == 1 ||
                    sortie_stages[r_of].end_index - sortie_stages[r_of].start_index == 1) {
                    continue;
                }
                // if (const auto it_r_of= std::find(truck_route.begin() + sortie_stages[r_of].start_index, truck_route.begin() + sortie_stages[r_of].end_index, truck_route[index+1]); it_r_of != truck_route.begin() + sortie_stages[r_of].end_index) {
                //     continue;
                // }

                const double new_l_of_drone_cost = instance->tau_prime[truck_route[index + 1]][l_of] +
                    instance->tau_prime[l_of][truck_route[r_of_l_customer]];
                const double new_r_of_drone_cost = instance->tau_prime[truck_route[l_of_r_customer]][r_of] +
                    instance->tau_prime[r_of][truck_route[index + 1]];

                if (new_l_of_drone_cost > instance->e - instance->sr ||
                    new_r_of_drone_cost > instance->e - instance->sr) {
                    continue;
                }

                const double new_r_of_truck_trip = sortie_stages[r_of].truck_trip -
                    instance->tau[truck_route[index - 1]][truck_route[index]] +
                    instance->tau[truck_route[index - 1]][truck_route[index + 1]];

                if (new_r_of_truck_trip > instance->e - instance->sr) {
                    continue;
                }

                sortie_stages[l_of].drone_trip = new_l_of_drone_cost;
                sortie_stages[r_of].drone_trip = new_r_of_drone_cost;
                sortie_stages[r_of].truck_trip = new_r_of_truck_trip;
                sortie_stages[l_of].truck_trip -= instance->tau[truck_route[index]][truck_route[index + 1]];

                for (const int& d : served_by_drone) {
                    if (sortie_stages[d].start_index > index) {
                        sortie_stages[d].start_index--;
                    }
                    if (sortie_stages[d].end_index > index) {
                        sortie_stages[d].end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + index);
            }
            else if (is_l && !is_r) {
                const int r_of_l_customer = sortie_stages[l_of].end_index;
                if (truck_route[index - 1] == truck_route[r_of_l_customer]) {
                    continue;
                }
                // if (const auto it = std::find(truck_route.begin() + sortie_stages[l_of].start_index+1, truck_route.begin() + sortie_stages[l_of].end_index, truck_route[index-1]); it != truck_route.begin() + sortie_stages[l_of].end_index) {
                //     continue;
                // }
                const double new_l_of_drone_trip = instance->tau_prime[truck_route[index - 1]][l_of] +
                    instance->tau_prime[l_of][truck_route[r_of_l_customer]];

                if (new_l_of_drone_trip > instance->e - instance->sr) {
                    continue;
                }

                const double new_l_of_truck_trip = sortie_stages[l_of].truck_trip -
                    instance->tau[truck_route[index]][truck_route[index + 1]] +
                    instance->tau[truck_route[index - 1]][truck_route[index + 1]];

                if (new_l_of_truck_trip > instance->e - instance->sr) {
                    continue;
                }

                sortie_stages[l_of].truck_trip = new_l_of_truck_trip;
                sortie_stages[l_of].drone_trip = new_l_of_drone_trip;

                for (const int& d : served_by_drone) {
                    if (sortie_stages[d].start_index >= index) {
                        sortie_stages[d].start_index--;
                    }
                    if (sortie_stages[d].end_index > index) {
                        sortie_stages[d].end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + index);
            }
            else if (!is_l && is_r) {
                if (truck_route[index + 1] == truck_route[sortie_stages[r_of].start_index]) {
                    continue;
                }
                // if (const auto it = std::find(truck_route.begin() + sortie_stages[r_of].start_index+1, truck_route.begin() + sortie_stages[r_of].end_index, truck_route[index+1]); it != truck_route.begin() + sortie_stages[r_of].end_index) {
                //     continue;
                // }
                const double new_r_of_drone_trip = instance->tau_prime[truck_route[sortie_stages[r_of].start_index]][
                        r_of] +
                    instance->tau_prime[r_of][truck_route[index + 1]];

                if (new_r_of_drone_trip > instance->e - instance->sr) {
                    continue;
                }

                const double new_r_of_truck_trip = sortie_stages[r_of].truck_trip -
                    instance->tau[truck_route[index - 1]][truck_route[index]] +
                    instance->tau[truck_route[index - 1]][truck_route[index + 1]];

                if (new_r_of_truck_trip > instance->e - instance->sr) {
                    continue;
                }

                sortie_stages[r_of].drone_trip = new_r_of_drone_trip;
                sortie_stages[r_of].truck_trip = new_r_of_truck_trip;

                for (const int& d : served_by_drone) {
                    if (sortie_stages[d].start_index > index) {
                        sortie_stages[d].start_index--;
                    }
                    if (sortie_stages[d].end_index > index) {
                        sortie_stages[d].end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + index);
            }
            else if (is_middle) {
                sortie_stages[mid_of].truck_trip = sortie_stages[mid_of].truck_trip -
                    instance->tau[truck_route[index - 1]][truck_route[index]] -
                    instance->tau[truck_route[index]][truck_route[index + 1]] +
                    instance->tau[truck_route[index - 1]][truck_route[index + 1]];

                for (const int& d : served_by_drone) {
                    if (sortie_stages[d].start_index > index) {
                        sortie_stages[d].start_index--;
                    }
                    if (sortie_stages[d].end_index > index) {
                        sortie_stages[d].end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + index);
            }
        }

        bool removed_all = true;
        for (int i = 0; i < truck_route.size(); ++i) {
            if (truck_route[i] == customer_to_remove) {
                removed_all = false;
                break;
            }
        }
        if (isElementInArray(served_by_drone, customer_to_remove)) {
            removed_all = false;
        }
        if (removed_all) {
            visited[customer_to_remove] = false;
        }
    }
}

void SolutionL::remove_truck_index_loop_no_battery(const int index) {
    if (truck_route[index - 1] == truck_route[index + 1]) {
        return;
    }
    const int customer = truck_route[index];
    bool removable = true;
    bool is_middle = false, is_l = false, is_r = false;
    int l_of = -1, r_of = -1, mid_of = -1;
    for (const int& d : served_by_drone) {
        const auto& ss = sortie_stages[d];
        if (index > ss.start_index && index < ss.end_index) {
            is_middle = true;
            mid_of = d;
        }
        if (index == ss.start_index) {
            is_l = true;
            l_of = d;
        }
        if (index == ss.end_index) {
            is_r = true;
            r_of = d;
        }
    }

    // got.
    if (!is_middle && !is_l && !is_r) {
        // removal will affect all sortie after it.
        for (const int& d : served_by_drone) {
            auto& ss = sortie_stages[d];
            if (ss.start_index > index) {
                ss.start_index--;
                ss.end_index--;
                continue;
            }
            if (ss.end_index > index) {
                ss.end_index--;
            }
        }
        truck_route.erase(truck_route.begin() + index);
    }
    if (is_l && is_r) {
        auto& ss_l_of = sortie_stages[l_of];
        auto& ss_r_of = sortie_stages[r_of];

        const int r_of_l_customer = ss_l_of.end_index;
        const int l_of_r_customer = ss_r_of.start_index;

        // Check if the next truck route customer is the same as any critical customer in the current routes
        // if (truck_route[index + 1] == truck_route[l_of_r_customer] ||
        //     truck_route[index + 1] == truck_route[r_of_l_customer]) {
        //     return;
        // }

        // Check if any route segment is too short
        if ((ss_l_of.end_index - ss_l_of.start_index == 1) ||
            (ss_r_of.end_index - ss_r_of.start_index == 1)) {
            removable = false;
        }

        // Calculate new drone costs and check feasibility
        const double new_l_of_drone_cost = instance->tau_prime[truck_route[index + 1]][l_of] +
            instance->tau_prime[l_of][truck_route[r_of_l_customer]];
        // if (new_l_of_drone_cost > instance->e - instance->sr) {
        //     removable = false;
        // }

        const double new_r_of_drone_cost = instance->tau_prime[truck_route[l_of_r_customer]][r_of] +
            instance->tau_prime[r_of][truck_route[index + 1]];
        // if (new_r_of_drone_cost > instance->e - instance->sr) {
        //     removable = false;
        // }

        // Calculate and check new truck trip feasibility
        const double new_r_of_truck_trip = ss_r_of.truck_trip -
            instance->tau[truck_route[index - 1]][truck_route[index]] +
            instance->tau[truck_route[index - 1]][truck_route[index + 1]];
        // if (new_r_of_truck_trip > instance->e - instance->sr) {
        //     removable = false;
        // }

        // If all conditions are met, update the sortie stages and truck route
        if (removable) {
            ss_l_of.drone_trip = new_l_of_drone_cost;
            ss_r_of.drone_trip = new_r_of_drone_cost;
            ss_r_of.truck_trip = new_r_of_truck_trip;
            ss_l_of.truck_trip -= instance->tau[truck_route[index]][truck_route[index + 1]];

            // Adjust indices for served_by_drone
            for (int d : served_by_drone) {
                auto& ss = sortie_stages[d];
                if (ss.start_index > index) {
                    ss.start_index--;
                }
                if (ss.end_index > index) {
                    ss.end_index--;
                }
            }

            // Remove the truck route element at the given index
            truck_route.erase(truck_route.begin() + index);
        }
    }
    else if (is_l && !is_r) {
        auto& ss_l_of = sortie_stages[l_of];
        const int r_of_l_customer = ss_l_of.end_index;
        // if (truck_route[index - 1] == truck_route[r_of_l_customer]) {
        //     return;
        // }
        const double new_l_of_drone_trip = instance->tau_prime[truck_route[index - 1]][l_of]
            + instance->tau_prime[l_of][truck_route[r_of_l_customer]];

        const double new_l_of_truck_trip = ss_l_of.truck_trip
            - instance->tau[truck_route[index]][truck_route[index + 1]]
            + instance->tau[truck_route[index - 1]][truck_route[index + 1]];
        ss_l_of.truck_trip = new_l_of_truck_trip;
        ss_l_of.drone_trip = new_l_of_drone_trip;
        for (const int& d : served_by_drone) {
            auto& ss = sortie_stages[d];
            if (ss.start_index >= index) {
                ss.start_index--;
                ss.end_index--;
                continue;
            }
            if (ss.end_index > index) {
                ss.end_index--;
            }
        }
        truck_route.erase(truck_route.begin() + index);
    }
    else if (!is_l && is_r) {
        auto& ss_r_of = sortie_stages[r_of];
        // if (truck_route[index + 1] == truck_route[ss_r_of.start_index]) {
        //     return;
        // }
        // new drone tour feasibility check
        const double new_r_of_drone_trip = instance->tau_prime[truck_route[ss_r_of.start_index]][r_of]
            + instance->tau_prime[r_of][truck_route[index + 1]];
        const double new_r_of_truck_trip = ss_r_of.truck_trip
            - instance->tau[truck_route[index - 1]][truck_route[index]]
            + instance->tau[truck_route[index - 1]][truck_route[index + 1]];
        ss_r_of.drone_trip = new_r_of_drone_trip;
        ss_r_of.truck_trip = new_r_of_truck_trip;
        for (const int d : served_by_drone) {
            auto& ss = sortie_stages[d];
            if (ss.start_index > index) {
                ss.start_index--;
                ss.end_index--;
                continue;
            }
            if (ss.end_index > index) {
                ss.end_index--;
            }
        }
        truck_route.erase(truck_route.begin() + index);
    }
    else if (is_middle) {
        // vẫn phải kiểm tra truck tour mới có feasible hay không.


        sortie_stages[mid_of].truck_trip = sortie_stages[mid_of].truck_trip
            - instance->tau[truck_route[index - 1]][truck_route[index]]
            - instance->tau[truck_route[index]][truck_route[index + 1]]
            + instance->tau[truck_route[index - 1]][truck_route[index + 1]];;

        for (const int d : served_by_drone) {
            auto& ss = sortie_stages[d];
            if (ss.start_index > index) {
                ss.start_index--;
                ss.end_index--;
                continue;
            }
            if (ss.end_index > index) {
                ss.end_index--;
            }
        }
        truck_route.erase(truck_route.begin() + index);
    }


    if (removable) {
        bool removed_all = true;
        for (const int i : truck_route) {
            if (i == customer) {
                removed_all = false;
            }
        }
        if (isElementInArray(served_by_drone, customer)) {
            removed_all = false;
        }
        if (removed_all) {
            visited[customer] = false;
        }
    }
}

bool SolutionL::remove_truck_index(const int index) {
    if (truck_route[index - 1] == truck_route[index + 1]) {
        return false;
    }
    const int customer = truck_route[index];
    bool removable = true;
    bool is_middle = false, is_l = false, is_r = false;
    int l_of = -1, r_of = -1, mid_of = -1;
    for (const int& d : served_by_drone) {
        const auto& ss = sortie_stages[d];
        if (index > ss.start_index && index < ss.end_index) {
            is_middle = true;
            mid_of = d;
        }
        if (index == ss.start_index) {
            is_l = true;
            l_of = d;
        }
        if (index == ss.end_index) {
            is_r = true;
            r_of = d;
        }
    }

    // got.
    if (!is_middle && !is_l && !is_r) {
        // removal will affect all sortie after it.
        for (const int& d : served_by_drone) {
            auto& ss = sortie_stages[d];
            if (ss.start_index > index) {
                ss.start_index--;
                ss.end_index--;
                continue;
            }
            if (ss.end_index > index) {
                ss.end_index--;
            }
        }
        truck_route.erase(truck_route.begin() + index);
        // sortie_invalid_occurrence();
    }
    if (is_l && is_r) {
        auto& ss_l_of = sortie_stages[l_of];
        auto& ss_r_of = sortie_stages[r_of];

        const int r_of_l_customer = ss_l_of.end_index;
        const int l_of_r_customer = ss_r_of.start_index;

        // if (const auto it_r_of= std::find(truck_route.begin() + ss_r_of.start_index, truck_route.begin() + ss_r_of.end_index, truck_route[index+1]); it_r_of != truck_route.begin() + ss_r_of.end_index) {
        //     return false;
        // }
        // Check if the next truck route customer is the same as any critical customer in the current routes
        if (truck_route[index + 1] == truck_route[l_of_r_customer] ||
            truck_route[index + 1] == truck_route[r_of_l_customer]) {
            return false;
        }

        // Check if any route segment is too short
        if ((ss_l_of.end_index - ss_l_of.start_index == 1) ||
            (ss_r_of.end_index - ss_r_of.start_index == 1)) {
            return false;
            removable = false;
        }

        // Calculate new drone costs and check feasibility
        const double new_l_of_drone_cost = instance->tau_prime[truck_route[index + 1]][l_of] +
            instance->tau_prime[l_of][truck_route[r_of_l_customer]];
        if (new_l_of_drone_cost > instance->e - instance->sr) {
            return false;
            removable = false;
        }

        const double new_r_of_drone_cost = instance->tau_prime[truck_route[l_of_r_customer]][r_of] +
            instance->tau_prime[r_of][truck_route[index + 1]];
        if (new_r_of_drone_cost > instance->e - instance->sr) {
            return false;
            removable = false;
        }

        // Calculate and check new truck trip feasibility
        const double new_r_of_truck_trip = ss_r_of.truck_trip -
            instance->tau[truck_route[index - 1]][truck_route[index]] +
            instance->tau[truck_route[index - 1]][truck_route[index + 1]];
        if (new_r_of_truck_trip > instance->e - instance->sr) {
            return false;
            removable = false;
        }

        // If all conditions are met, update the sortie stages and truck route
        if (removable) {
            ss_l_of.drone_trip = new_l_of_drone_cost;
            ss_r_of.drone_trip = new_r_of_drone_cost;
            ss_r_of.truck_trip = new_r_of_truck_trip;
            ss_l_of.truck_trip -= instance->tau[truck_route[index]][truck_route[index + 1]];

            // Adjust indices for served_by_drone
            for (const int d : served_by_drone) {
                auto& ss = sortie_stages[d];
                if (ss.start_index > index) {
                    ss.start_index--;
                }
                if (ss.end_index > index) {
                    ss.end_index--;
                }
            }

            // Remove the truck route element at the given index
            truck_route.erase(truck_route.begin() + index);
            // sortie_invalid_occurrence();
        }
    }
    else if (is_l && !is_r) {
        auto& ss_l_of = sortie_stages[l_of];
        const int r_of_l_customer = ss_l_of.end_index;
        if (truck_route[index - 1] == truck_route[r_of_l_customer]) {
            return false;
        }
        // if (const auto it = std::find(truck_route.begin() + ss_l_of.start_index+1, truck_route.begin() + ss_l_of.end_index, truck_route[index-1]); it != truck_route.begin() + ss_l_of.end_index) {
        //     return false;
        // }
        if (const double new_l_of_drone_trip = instance->tau_prime[truck_route[index - 1]][l_of]
                + instance->tau_prime[l_of][truck_route[r_of_l_customer]];
            new_l_of_drone_trip <= instance->e - instance->sr) {
            if (const double new_l_of_truck_trip = ss_l_of.truck_trip
                    - instance->tau[truck_route[index]][truck_route[index + 1]]
                    + instance->tau[truck_route[index - 1]][truck_route[index + 1]]; new_l_of_truck_trip <= instance->e
                -
                instance->sr) {
                ss_l_of.truck_trip = new_l_of_truck_trip;
                ss_l_of.drone_trip = new_l_of_drone_trip;
                for (const int& d : served_by_drone) {
                    auto& ss = sortie_stages[d];
                    if (ss.start_index >= index) {
                        ss.start_index--;
                        ss.end_index--;
                        continue;
                    }
                    if (ss.end_index > index) {
                        ss.end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + index);
                // sortie_invalid_occurrence();
            }
            else {
                removable = false;
            }
        }
        else {
            removable = false;
        }
    }
    else if (!is_l && is_r) {
        auto& ss_r_of = sortie_stages[r_of];
        if (truck_route[index + 1] == truck_route[ss_r_of.start_index]) {
            return false;
        }
        // if (const auto it = std::find(truck_route.begin() + ss_r_of.start_index+1, truck_route.begin() + ss_r_of.end_index, truck_route[index+1]); it != truck_route.begin() + ss_r_of.end_index) {
        //     return false;
        // }
        // new drone tour feasibility check
        if (const double new_r_of_drone_trip = instance->tau_prime[truck_route[ss_r_of.start_index]][r_of]
            + instance->tau_prime[r_of][truck_route[index + 1]]; new_r_of_drone_trip <= instance->e - instance->sr) {
            if (const double new_r_of_truck_trip = ss_r_of.truck_trip
                    - instance->tau[truck_route[index - 1]][truck_route[index]]
                    + instance->tau[truck_route[index - 1]][truck_route[index + 1]]; new_r_of_truck_trip <= instance->e
                -
                instance->sr) {
                ss_r_of.drone_trip = new_r_of_drone_trip;
                ss_r_of.truck_trip = new_r_of_truck_trip;
                for (const int d : served_by_drone) {
                    auto& ss = sortie_stages[d];
                    if (ss.start_index > index) {
                        ss.start_index--;
                        ss.end_index--;
                        continue;
                    }
                    if (ss.end_index > index) {
                        ss.end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + index);
                // sortie_invalid_occurrence();
            }
            else {
                removable = false;
            }
        }
        else {
            removable = false;
        }
    }
    else if (is_middle) {
        // vẫn phải kiểm tra truck tour mới có feasible hay không.
        sortie_stages[mid_of].truck_trip = sortie_stages[mid_of].truck_trip
            - instance->tau[truck_route[index - 1]][truck_route[index]]
            - instance->tau[truck_route[index]][truck_route[index + 1]]
            + instance->tau[truck_route[index - 1]][truck_route[index + 1]];;

        for (const int d : served_by_drone) {
            auto& ss = sortie_stages[d];
            if (ss.start_index > index) {
                ss.start_index--;
                ss.end_index--;
                continue;
            }
            if (ss.end_index > index) {
                ss.end_index--;
            }
        }
        truck_route.erase(truck_route.begin() + index);
    }


    if (removable) {
        bool removed_all = true;
        for (const int i : truck_route) {
            if (i == customer) {
                removed_all = false;
            }
        }
        if (isElementInArray(served_by_drone, customer)) {
            removed_all = false;
        }
        if (removed_all) {
            visited[customer] = false;
        }
        return true;
    }
    else {
        return false;
    }
}

void SolutionL::remove_string(const int start_index, const int remove_size) {
    // just remove the string start from start_index, kill all sortie associated with the remove.
    std::vector<std::pair<int, int>> index_pairs;
    // for later check.
    if (start_index + remove_size - 1 > truck_route.size() - 2) {
        index_pairs.emplace_back(start_index, truck_route.size() - 2);
        if (const int last_index_sec = 1 + remove_size - truck_route.size() + start_index; last_index_sec >=
            start_index) {
            index_pairs.emplace_back(1, start_index - 1);
        }
        else {
            index_pairs.emplace_back(1, 1 + remove_size - truck_route.size() + start_index);
        }
    }
    else {
        index_pairs.emplace_back(start_index, start_index + remove_size - 1);
    }
    // case check.
    for (const auto& [fst, snd] : index_pairs) {
        // remove drone customer correlated to this these index
        std::vector<int> drone_removal;
        for (const int d : served_by_drone) {
            if ((fst <= sortie_stages[d].start_index && sortie_stages[d].start_index <= snd) || (fst <= sortie_stages[d].
                end_index && sortie_stages[d].end_index <= snd) || (fst >= sortie_stages[d].start_index && snd <=
                    sortie_stages[d].end_index)) {
                drone_removal.push_back(d);
            }
        }
        for (const int d : drone_removal) {
            std::erase(served_by_drone, d);
            visited[d] = false;
        }

        std::vector<int> count(instance->num_node, 0);
        for (int i = 1; i < fst; i++) {
            count[truck_route[i]]++;
        }
        for (int i = snd + 1; i < truck_route.size(); i++) {
            count[truck_route[i]]++;
        }
        for (int i = fst; i <= snd; i++) {
            if (count[truck_route[i]] == 0) {
                visited[truck_route[i]] = false;
            }
        }
        truck_route.erase(truck_route.begin() + fst, truck_route.begin() + snd + 1);
        for (const int d : served_by_drone) {
            if (sortie_stages[d].start_index >= snd) {
                sortie_stages[d].start_index -= (snd - fst + 1);
                sortie_stages[d].end_index -= (snd - fst + 1);
            }
        }
    }
}

int SolutionL::swap_mid_only() {
    int imp = 0;
start:
    for (int i = 1; i < truck_route.size() - 2; ++i) {
        for (int j = i + 1; j < truck_route.size() - 1; ++j) {
            // check if they are both mid of something.
            bool swappable = true;

            int i_mid_of = -1;
            int j_mid_of = -1;
            for (const int& d : served_by_drone) {
                if (i == sortie_stages[d].start_index || i == sortie_stages[d].end_index || j == sortie_stages[d].
                    start_index || j == sortie_stages[d].end_index) {
                    swappable = false;
                    break;
                }
                if (sortie_stages[d].start_index < i && i < sortie_stages[d].end_index) {
                    i_mid_of = d;
                }
                if (sortie_stages[d].start_index < j && j < sortie_stages[d].end_index) {
                    j_mid_of = d;
                }
            }
            if (!swappable) {
                continue;
            }

            const double sum_adjacent_of_i = instance->tau[truck_route[i - 1]][truck_route[i]]
                + instance->tau[truck_route[i]][truck_route[i + 1]];
            const double sum_adjacent_of_j = instance->tau[truck_route[j - 1]][truck_route[j]]
                + instance->tau[truck_route[j]][truck_route[j + 1]];
            // qua duoc ai tren
            if (i_mid_of == -1 && j_mid_of == -1) {
                // bool better = false;
                // cha la mid cua cai gi ca.
                // check cost if perform swap.
                // if oke thi swap luon.

                if (j == i + 1) {
                    // easier to compute.
                    // if gain < loss => apply;

                    if (instance->tau[truck_route[i - 1]][truck_route[j]]
                        + instance->tau[truck_route[i]][truck_route[j + 1]]
                        < instance->tau[truck_route[i - 1]][truck_route[i]]
                        + instance->tau[truck_route[j]][truck_route[j + 1]]) {
                        // imp++;
                        const int temp = truck_route[i];
                        truck_route[i] = truck_route[j];
                        truck_route[j] = temp;
                        calculate_objective();
                        goto start;
                    }
                }
                else {
                    // gain < loss
                    const double increased_cost = instance->tau[truck_route[i - 1]][truck_route[j]]
                        + instance->tau[truck_route[j]][truck_route[i + 1]]
                        + instance->tau[truck_route[j - 1]][truck_route[i]]
                        + instance->tau[truck_route[i]][truck_route[j + 1]] - (sum_adjacent_of_i + sum_adjacent_of_j);

                    if (increased_cost
                        <
                        0 && increased_cost < -1e-3) {
                        imp++;
                        const int temp = truck_route[i];
                        truck_route[i] = truck_route[j];
                        truck_route[j] = temp;
                        calculate_objective();
                        goto start;
                    }
                }
            }
            else if (i_mid_of != -1 && j_mid_of == -1) {
                // kiem tra sortie thuoc i, neu swap j vao thi co tot hon khong?
                const double current_i_mid_cost = std::max(sortie_stages[i_mid_of].truck_trip,
                                                           sortie_stages[i_mid_of].drone_trip);
                double new_truck_cost_of_i_mid = sortie_stages[i_mid_of].truck_trip - sum_adjacent_of_i
                    + instance->tau[truck_route[i - 1]][truck_route[j]] + instance->tau[truck_route[j]][truck_route[i +
                        1]];
                if (new_truck_cost_of_i_mid <= instance->e - instance->sr) {
                    double new_i_cost = std::max(new_truck_cost_of_i_mid, sortie_stages[i_mid_of].drone_trip);
                    double sortie_increased_cost = new_i_cost - current_i_mid_cost;
                    double increased_cost_at_j = instance->tau[truck_route[j - 1]][truck_route[i]] + instance->tau[
                            truck_route[i]][truck_route[j + 1]]
                        - sum_adjacent_of_j;
                    if (sortie_increased_cost + increased_cost_at_j < -1e-3) {
                        calculate_objective();
                        const double current_objective = objective;

                        const int temp = truck_route[i];
                        truck_route[i] = truck_route[j];
                        truck_route[j] = temp;
                        // update
                        sortie_stages[i_mid_of].truck_trip = new_truck_cost_of_i_mid;
                        calculate_objective();
                        std::cout << "SolutionL after the swap: " << std::endl;
                        if (fabs(current_objective - objective - (sortie_increased_cost + increased_cost_at_j) * -1.0) >
                            1e-3) {
                            std::cout << "chenh lech: " << std::endl;
                            std::cout << "dang ra: " << (sortie_increased_cost + increased_cost_at_j) * -1.0 <<
                                std::endl;
                            std::cout << current_objective << std::endl;
                            std::cout << objective << std::endl;
                            std::cout << "thuc te: " << current_objective - objective << std::endl;
                        }
                        // do the swap.


                        goto start;
                    }
                }
            }
            else if (i_mid_of == -1 && j_mid_of != -1) {
                // kiem tra sortie thuoc i, neu swap j vao thi co tot hon khong?
                const double current_j_mid_cost = std::max(sortie_stages[j_mid_of].truck_trip,
                                                           sortie_stages[j_mid_of].drone_trip);
                double new_truck_cost_of_j_mid = sortie_stages[j_mid_of].truck_trip - sum_adjacent_of_j
                    + instance->tau[truck_route[j - 1]][truck_route[i]] + instance->tau[truck_route[i]][truck_route[j +
                        1]];
                if (new_truck_cost_of_j_mid <= instance->e - instance->sr) {
                    const double new_j_cost = std::max(new_truck_cost_of_j_mid, sortie_stages[j_mid_of].drone_trip);
                    const double sortie_increased_cost = new_j_cost - current_j_mid_cost;
                    const double increased_cost_at_i = instance->tau[truck_route[i - 1]][truck_route[j]] + instance->tau
                        [
                            truck_route[j]][truck_route[i + 1]]
                        - sum_adjacent_of_i;
                    if (sortie_increased_cost + increased_cost_at_i < -1e-3) {
                        const int temp = truck_route[i];
                        truck_route[i] = truck_route[j];
                        truck_route[j] = temp;
                        sortie_stages[j_mid_of].truck_trip = new_truck_cost_of_j_mid;
                        calculate_objective();
                        goto start;
                    }
                }
            }
            else if (i_mid_of != -1 && j_mid_of != -1 && i_mid_of == j_mid_of) {
                // trong cung 1 sortie.
            }
            else if (i_mid_of != -1 && j_mid_of != -1 && i_mid_of != j_mid_of) {}
        }
    }
    return imp;
}


void SolutionL::swap() {
start:
    for (int i = 1; i < truck_route.size() - 2; ++i) {
        for (int j = i + 1; j < truck_route.size() - 1; ++j) {
            bool better = false;
            const double sum_adjacent_of_i = instance->tau[truck_route[i - 1]][truck_route[i]]
                + instance->tau[truck_route[i]][truck_route[i + 1]];
            const double sum_adjacent_of_j = instance->tau[truck_route[j - 1]][truck_route[j]]
                + +instance->tau[truck_route[j]][truck_route[j + 1]];

            if (j == i + 1) {
                // easier to compute.
                // if gain < loss => apply;

                if (instance->tau[truck_route[i - 1]][truck_route[j]]
                    + instance->tau[truck_route[i]][truck_route[j + 1]]
                    < instance->tau[truck_route[i - 1]][truck_route[i]]
                    + instance->tau[truck_route[j]][truck_route[j + 1]]) {
                    better = true;
                }
            }
            else {
                // gain < loss
                if (instance->tau[truck_route[i - 1]][truck_route[j]]
                    + instance->tau[truck_route[j]][truck_route[i + 1]]
                    + instance->tau[truck_route[j - 1]][truck_route[i]]
                    + instance->tau[truck_route[i]][truck_route[j + 1]]
                    <
                    sum_adjacent_of_i + sum_adjacent_of_j) {
                    better = true;
                }
            }
            if (better) {
                bool i_l = false, i_r = false, i_mid = false;
                bool j_l = false, j_r = false, j_mid = false;
                int i_l_of = -1, i_r_of = -1, i_mid_of = -1;
                int j_l_of = -1, j_r_of = -1, j_mid_of = -1;
                for (const int d : served_by_drone) {
                    if (i == sortie_stages[d].start_index) {
                        i_l = true;
                        i_l_of = d;
                    }
                    if (i == sortie_stages[d].end_index) {
                        i_r = true;
                        i_r_of = d;
                    }
                    if (i > sortie_stages[d].start_index && i < sortie_stages[d].end_index) {
                        i_mid = true;
                        i_mid_of = d;
                    }
                    if (j == sortie_stages[d].start_index) {
                        j_l = true;
                        j_l_of = d;
                    }
                    if (j == sortie_stages[d].end_index) {
                        j_r = true;
                        j_r_of = d;
                    }
                    if (j > sortie_stages[d].start_index && j < sortie_stages[d].end_index) {
                        j_mid = true;
                        j_mid_of = d;
                    }
                }
                // enough data.
                // iterate through scenes.
                // if not both thoi da nhi?
                if ((i_l && !i_r) || (j_l && !j_r)) {
                    if (i_mid && j_mid) {
                        // check new truck route feasibility.
                        if (i_mid_of == j_mid_of) {
                            // swap immediately.
                            const int temp = truck_route[i];
                            truck_route[i] = truck_route[j];
                            truck_route[j] = temp;
                            goto start;
                        }
                        // must check both trip.
                        // check trip of i_mid_of
                        if (sortie_stages[i_mid_of].truck_trip
                            - sum_adjacent_of_i
                            + instance->tau[truck_route[i - 1]][truck_route[j]]
                            + instance->tau[truck_route[j]][truck_route[i + 1]]
                            <= instance->e - instance->sr
                            &&
                            sortie_stages[j_mid_of].truck_trip
                            - sum_adjacent_of_j
                            + instance->tau[truck_route[j - 1]][truck_route[i]]
                            + instance->tau[truck_route[i]][truck_route[j + 1]]
                            <= instance->e - instance->sr) {
                            const int temp = truck_route[i];
                            truck_route[i] = truck_route[j];
                            truck_route[j] = temp;
                            goto start;
                        }
                    }
                    else if (i_mid && j_l) {
                        if (i_mid_of == j_l_of) {
                            if (sortie_stages[i_mid_of].drone_trip
                                - instance->tau_prime[truck_route[j]][i_mid_of]
                                + instance->tau_prime[truck_route[i]][i_mid_of]
                                <= instance->e - instance->sr
                                &&
                                sortie_stages[i_mid_of].truck_trip
                                - sum_adjacent_of_i
                                - instance->tau[truck_route[j]][truck_route[j + 1]]
                                + instance->tau[truck_route[i]][truck_route[j + 1]]
                                + instance->tau[truck_route[i - 1]][truck_route[j]]
                                + instance->tau[truck_route[j]][truck_route[i + 1]]
                                <= instance->e - instance->sr) {
                                const int temp = truck_route[i];
                                truck_route[i] = truck_route[j];
                                truck_route[j] = temp;
                                goto start;
                            }
                        }
                        // i_mid_of != j_l_of
                        // check new drone trip cho j_l_of
                        if ( // new drone trip cho j_l_of
                            sortie_stages[j_l_of].drone_trip
                            - instance->tau_prime[truck_route[j]][j_l_of]
                            + instance->tau_prime[truck_route[i]][j_l_of]
                            <= instance->e - instance->sr
                            &&
                            // new truck trip cho j_l_of
                            sortie_stages[j_l_of].truck_trip
                            - instance->tau[truck_route[j]][truck_route[j + 1]]
                            + instance->tau[truck_route[i]][truck_route[j + 1]]
                            <= instance->e - instance->sr
                            &&
                            // new truck trip cho i_mid_of
                            sortie_stages[i_mid_of].truck_trip
                            - sum_adjacent_of_i
                            + instance->tau[truck_route[i - 1]][truck_route[j]]
                            + instance->tau[truck_route[j]][truck_route[i + 1]]
                            <= instance->e - instance->sr) {
                            const int temp = truck_route[i];
                            truck_route[i] = truck_route[j];
                            truck_route[j] = temp;
                            goto start;
                        }
                        // check new truck trip cho j_l_of
                        // check new truck trip cho i_mid_of
                    }
                    else if (i_mid && j_r) {
                        if (i_mid_of == j_r_of) {
                            // only need to check the new drone trip and truck trip
                            if (sortie_stages[i_mid_of].drone_trip
                                - instance->tau_prime[i_mid_of][truck_route[j]]
                                + instance->tau_prime[i_mid_of][truck_route[i]]
                                <= instance->e - instance->sr
                                &&
                                sortie_stages[i_mid_of].truck_trip
                                - sum_adjacent_of_i
                                - instance->tau[truck_route[j - 1]][truck_route[j]]
                                + instance->tau[truck_route[i - 1]][truck_route[j]]
                                + instance->tau[truck_route[j]][truck_route[i + 1]]
                                + instance->tau[truck_route[j - 1]][truck_route[i]]
                                <= instance->e - instance->sr) {
                                const int temp = truck_route[i];
                                truck_route[i] = truck_route[j];
                                truck_route[j] = temp;
                                goto start;
                            }
                        }
                        if ( // new truck trip of i_mid_of
                            sortie_stages[i_mid_of].truck_trip
                            - sum_adjacent_of_i
                            + instance->tau[truck_route[i - 1]][truck_route[j]]
                            + instance->tau[truck_route[j]][truck_route[i + 1]]
                            <= instance->e - instance->sr
                            &&
                            // new drone trip of j_r_of
                            sortie_stages[j_r_of].drone_trip
                            - instance->tau_prime[j_r_of][truck_route[j]]
                            + instance->tau_prime[j_r_of][truck_route[i]]
                            <= instance->e - instance->sr
                            &&
                            // new truck trip of j_r_of
                            sortie_stages[j_r_of].truck_trip
                            - instance->tau[truck_route[j - 1]][truck_route[j]]
                            + instance->tau[truck_route[j - 1]][truck_route[i]]
                            <= instance->e - instance->sr) {
                            const int temp = truck_route[i];
                            truck_route[i] = truck_route[j];
                            truck_route[j] = temp;
                            goto start;
                        }
                    }
                    else if (i_l && j_mid) {
                        if (i_l_of == j_mid_of) {
                            if (sortie_stages[i_l_of].drone_trip
                                - instance->tau_prime[truck_route[i]][i_l_of]
                                + instance->tau_prime[truck_route[j]][i_l_of]
                                <= instance->e - instance->sr
                                &&
                                sortie_stages[i_l_of].truck_trip
                                - sum_adjacent_of_j
                                - instance->tau[truck_route[i]][truck_route[i + 1]]
                                + instance->tau[truck_route[j]][truck_route[i + 1]]
                                + instance->tau[truck_route[j - 1]][truck_route[i]]
                                + instance->tau[truck_route[i]][truck_route[j + 1]]
                                <= instance->e - instance->sr) {
                                const int temp = truck_route[i];
                                truck_route[i] = truck_route[j];
                                truck_route[j] = temp;
                                goto start;
                            }
                        }
                        if ( // new i_l_of drone trip
                            sortie_stages[i_l_of].drone_trip
                            - instance->tau_prime[truck_route[i]][i_l_of]
                            + instance->tau_prime[truck_route[j]][i_l_of]
                            <= instance->e - instance->sr
                            &&
                            // new i_l_of truck trip
                            sortie_stages[i_l_of].truck_trip
                            - instance->tau[truck_route[i]][truck_route[i + 1]]
                            + instance->tau[truck_route[j]][truck_route[i + 1]]
                            <= instance->e - instance->sr
                            &&
                            // new j_mid_of truck trip
                            sortie_stages[j_mid_of].truck_trip
                            - sum_adjacent_of_j
                            + instance->tau[truck_route[j - 1]][truck_route[i]]
                            + instance->tau[truck_route[i]][truck_route[j + 1]]
                            <= instance->e - instance->sr
                        ) {
                            const int temp = truck_route[i];
                            truck_route[i] = truck_route[j];
                            truck_route[j] = temp;
                            goto start;
                        }
                    }
                }


                // check if swappable?
            }
        }
    }
}

void SolutionL::loop_check() {
    for (int index = 1; index < truck_route.size() - 1; index++) {
        if (truck_route[index] == truck_route[index + 1]) {
            print_solution();
            exit(0);
        }
    }
}

void SolutionL::easy_sortie_update() {
    for (const int& d : served_by_drone) {
        sortie_stages[d].drone_trip = instance->tau_prime[truck_route[sortie_stages[d].start_index]][d]
            + instance->tau_prime[d][truck_route[sortie_stages[d].end_index]];
        double truck_trip = 0;
        for (int start = sortie_stages[d].start_index; start < sortie_stages[d].end_index; start++) {
            truck_trip += instance->tau[truck_route[start]][truck_route[start + 1]];
        }
        sortie_stages[d].truck_trip = truck_trip;
    }
}

void SolutionL::swap_op(const int i, const int j) {
    const int temp = truck_route[i];
    truck_route[i] = truck_route[j];
    truck_route[j] = temp;
}


void SolutionL::revisit() {
    bool revisitable = true;
tag:
    while (revisitable) {
        revisitable = false;
        std::unordered_map<int, int> countMap;
        std::vector<int> duplicateIndices;

        // First pass: Count occurrences of each element
        for (const auto& element : truck_route) {
            countMap[element]++;
        }

        // Second pass: Collect indices of duplicates
        for (int i = 0; i < truck_route.size(); ++i) {
            if (countMap[truck_route[i]] > 1) {
                duplicateIndices.push_back(i);
            }
        }
        std::vector<int> index_belongs_to_sortie(instance->num_node + 1, -1);
        for (int i = 1; i < truck_route.size() - 1; i++) {
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index <= i && i <= sortie_stages[d].end_index) {
                    index_belongs_to_sortie[i] = d;
                    break;
                }
            }
        }
        for (int index = 1; index < truck_route.size() - 3; index++) {
            int end_index_of_sortie_with_index = -1;
            if (index_belongs_to_sortie[index] != -1) {
                end_index_of_sortie_with_index = sortie_stages[index_belongs_to_sortie[index]].end_index;
            }
            for (int rep_index = index + 2; rep_index < truck_route.size() - 1; rep_index++) {
                if (rep_index == end_index_of_sortie_with_index) {
                    continue;
                }
                // do:
                if (existL(duplicateIndices, rep_index)) {
                    continue;
                }
                if (truck_route[rep_index - 1] == truck_route[index] || truck_route[rep_index + 1] == truck_route[
                    index] || truck_route[index] == truck_route[rep_index]) {
                    continue;
                }
                if (!instance->heavy_bool[truck_route[rep_index]]) {
                    // proceed to check if the node is sortie related.

                    bool is_l = false, is_r = false, is_mid = false;
                    int l_of = -1, r_of = -1;
                    for (const int d : served_by_drone) {
                        if (sortie_stages[d].start_index < rep_index && rep_index < sortie_stages[d].end_index) {
                            is_mid = true;
                            break;
                        }
                        if (sortie_stages[d].start_index == rep_index) {
                            is_l = true;
                            l_of = d;
                            continue;
                        }
                        if (sortie_stages[d].end_index == rep_index) {
                            is_r = true;
                            r_of = d;
                        }
                    }
                    if (is_mid) {
                        continue;
                    }
                    auto spaces = find_sortie_spaces();

                    // is not in sortie.
                    // tryna check.
                    if (!is_l && !is_r) {
                        int earliest_launch_index = -1;
                        int earliest_rendezvous_index = -1;

                        for (const auto& [fst, snd] : spaces) {
                            if (fst <= rep_index && rep_index <= snd) {
                                earliest_launch_index = std::max(fst, index + 1); // Combine and simplify
                                earliest_rendezvous_index = snd;
                                break;
                            }
                        }
                        double best_left_saved_cost = -1e9;
                        double best_right_saved_cost = -1e9;
                        int best_launch_index = -1;
                        double best_left_truck_trip = -1;
                        double best_left_drone_trip = -1;

                        int best_rendezvous_index = -1;
                        double best_right_truck_trip = -1;
                        double best_right_drone_trip = -1;
                        // span left.
                        double current_left_truck_trip = 0;
                        for (int launch_index = rep_index - 1; launch_index >= earliest_launch_index;
                             launch_index--) {
                            if (truck_route[launch_index] == truck_route[index]) {
                                break; // we do not need to go further.
                            }
                            current_left_truck_trip += (launch_index == rep_index - 1)
                                                           ? instance->tau[truck_route[launch_index]][truck_route[
                                                               index]]
                                                           : instance->tau[truck_route[launch_index]][truck_route[
                                                               launch_index + 1]];

                            if (current_left_truck_trip > instance->e - instance->sr) break;
                            // Early exit if constraint violated


                            if (const double left_drone_trip = instance->tau_prime[truck_route[launch_index]][
                                        truck_route[rep_index]]
                                    + instance->tau_prime[truck_route[rep_index]][truck_route[index]];
                                left_drone_trip
                                <= instance->e - instance->sr) {
                                const double old_cost = current_left_truck_trip
                                    - instance->tau[truck_route[rep_index - 1]][truck_route[index]]
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                    + instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]];
                                const double new_cost = std::max(left_drone_trip, current_left_truck_trip)
                                    + instance->tau[truck_route[index]][truck_route[rep_index + 1]];
                                if (const double saved = old_cost - new_cost; saved > 1e-3 && saved >
                                    best_left_saved_cost) {
                                    revisitable = true;
                                    best_left_saved_cost = saved;
                                    best_launch_index = launch_index;
                                    best_left_truck_trip = current_left_truck_trip;
                                    best_left_drone_trip = left_drone_trip;
                                }
                            }
                        }
                        // span right
                        double current_right_truck_trip = 0;
                        for (int rendezvous_index = rep_index + 1; rendezvous_index <= earliest_rendezvous_index
                             ; rendezvous_index++) {
                            if (truck_route[index] == truck_route[rendezvous_index]) {
                                break;
                            }
                            current_right_truck_trip += (rendezvous_index == rep_index + 1)
                                                            ? instance->tau[truck_route[index]][truck_route[
                                                                rendezvous_index]]
                                                            : instance->tau[truck_route[rendezvous_index - 1]][
                                                                truck_route[rendezvous_index]];

                            if (current_right_truck_trip > instance->e - instance->sr) break;
                            // Early exit if constraint violated


                            if (const double right_drone_trip = instance->tau_prime[truck_route[index]][
                                        truck_route[
                                            rep_index]]
                                    + instance->tau_prime[truck_route[rep_index]][truck_route[rendezvous_index]]
                                ;
                                right_drone_trip <= instance->e - instance->sr) {
                                const double old_cost = current_right_truck_trip
                                    - instance->tau[truck_route[index]][truck_route[rep_index + 1]]
                                    + instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]]
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]];
                                const double new_cost = std::max(right_drone_trip, current_right_truck_trip)
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[index]];
                                if (const double saved = old_cost - new_cost; saved > 1e-3 && saved >
                                    best_left_saved_cost) {
                                    revisitable = true;
                                    best_right_saved_cost = saved;
                                    best_rendezvous_index = rendezvous_index;
                                    best_right_truck_trip = current_right_truck_trip;
                                    best_right_drone_trip = right_drone_trip;
                                }
                            }
                        }
                        // so sanh
                        if (revisitable) {
                            if (best_left_saved_cost > best_right_saved_cost) {
                                // Execute left
                                if (std::ranges::find(served_by_drone, truck_route[rep_index]) == served_by_drone.
                                    end()) {
                                    served_by_drone.push_back(truck_route[rep_index]);
                                }
                                sortie_stages[truck_route[rep_index]] = {
                                    best_launch_index, rep_index, best_left_drone_trip, best_left_truck_trip
                                };
                            }
                            else if (best_right_saved_cost > best_left_saved_cost) {
                                // Execute right
                                if (std::ranges::find(served_by_drone, truck_route[rep_index]) == served_by_drone.
                                    end()) {
                                    served_by_drone.push_back(truck_route[rep_index]);
                                }
                                sortie_stages[truck_route[rep_index]] = {
                                    rep_index, best_rendezvous_index, best_right_drone_trip, best_right_truck_trip
                                };
                            }
                            truck_route[rep_index] = truck_route[index];
                            goto tag;
                        }
                    }
                    else if (is_l && !is_r) {
                        if (truck_route[index] == truck_route[sortie_stages[l_of].end_index]) {
                            continue;
                        }
                        int earliest_launch_index_free_set = -1;
                        for (const auto& [fst, snd] : spaces) {
                            if (snd == rep_index) {
                                earliest_launch_index_free_set = fst;
                                if (earliest_launch_index_free_set <= index) {
                                    earliest_launch_index_free_set = index + 1;
                                }
                                break;
                            }
                        }
                        // first case check: (free set) -  rep index - index, index - l_of _ end.
                        double best_first_case_saved_cost = -1e9;
                        int best_launch_index = -1;
                        double best_rep_truck_trip = -1;
                        double best_rep_drone_trip = -1;
                        double best_l_of_truck_trip = -1;
                        double best_l_of_drone_trip = -1;
                        const double l_of_trip = std::max(sortie_stages[l_of].truck_trip,
                                                          sortie_stages[l_of].drone_trip);
                        if (const double l_of_new_drone_trip = instance->tau_prime[truck_route[index]][l_of]
                                + instance->tau_prime[l_of][truck_route[sortie_stages[l_of].end_index]];
                            l_of_new_drone_trip
                            <= instance->e - instance->sr) {
                            if (const double l_of_new_truck_trip = sortie_stages[l_of].truck_trip
                                    - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]]
                                    + instance->tau[truck_route[index]][truck_route[rep_index + 1]];
                                l_of_new_truck_trip <= instance->e - instance->sr) {
                                // will be able to check the rep trip.
                                // the rep trip will be checked backward to ensure truck trip feasibility.
                                double current_rep_truck_trip = 0;

                                for (int launch_index = rep_index - 1; launch_index >= earliest_launch_index_free_set;
                                     launch_index--) {
                                    if (truck_route[launch_index] == truck_route[index]) {
                                        break;
                                    }

                                    current_rep_truck_trip += (launch_index == rep_index - 1)
                                                                  ? instance->tau[truck_route[launch_index]][truck_route
                                                                      [index]]
                                                                  : instance->tau[truck_route[launch_index]][truck_route
                                                                      [launch_index + 1]];
                                    if (current_rep_truck_trip > instance->e - instance->sr) {
                                        break;
                                    }

                                    // truck trip is good.
                                    // if the associated drone trip is good.
                                    if (const double rep_drone_trip = instance->tau_prime[truck_route[
                                                launch_index]]
                                            [truck_route[rep_index]]
                                            + instance->tau_prime[truck_route[rep_index]][truck_route[index]];
                                        rep_drone_trip <= instance->e - instance->sr) {
                                        const double old_cost = l_of_trip + current_rep_truck_trip
                                            - instance->tau[truck_route[rep_index - 1]][truck_route[index]]
                                            + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]];
                                        const double new_cost = std::max(rep_drone_trip, current_rep_truck_trip)
                                            + std::max(l_of_new_truck_trip, l_of_new_drone_trip);
                                        if (const double saved = old_cost - new_cost; saved > 1e-3 && saved >
                                            best_first_case_saved_cost) {
                                            revisitable = true;
                                            best_first_case_saved_cost = saved;
                                            best_launch_index = launch_index;
                                            best_rep_truck_trip = current_rep_truck_trip;
                                            best_rep_drone_trip = rep_drone_trip;
                                            best_l_of_truck_trip = l_of_new_truck_trip;
                                            best_l_of_drone_trip = l_of_new_drone_trip;
                                        }
                                    }
                                }
                            }
                        }
                        if (revisitable) {
                            sortie_stages[l_of].truck_trip = best_l_of_truck_trip;
                            sortie_stages[l_of].drone_trip = best_l_of_drone_trip;

                            if (bool exist = std::ranges::find(served_by_drone, truck_route[rep_index]) !=
                                served_by_drone.end(); !exist) {
                                served_by_drone.push_back(truck_route[rep_index]);
                            }

                            sortie_stages[truck_route[rep_index]] = {
                                best_launch_index, rep_index, best_rep_drone_trip, best_rep_truck_trip
                            };
                            truck_route[rep_index] = truck_route[index];
                            goto tag;
                        }
                        double best_second_case_saved_cost = -1e9;
                        // we check for case 2
                        // (free set) - l_of - index, index - rep - end.
                        if (const double rep_new_truck_trip = sortie_stages[l_of].truck_trip
                                - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]]
                                + instance->tau[truck_route[index]][truck_route[rep_index + 1]];
                            rep_new_truck_trip <= instance->e - instance->sr) {
                            if (const double rep_new_drone_trip = instance->tau_prime[truck_route[index]][
                                        truck_route[rep_index]]
                                    + instance->tau_prime[truck_route[rep_index]][truck_route[sortie_stages[l_of].
                                        end_index]];
                                rep_new_drone_trip <= instance->e - instance->sr) {
                                double current_l_of_truck_trip = 0;
                                for (int launch_index = rep_index - 1; launch_index >=
                                     earliest_launch_index_free_set;
                                     launch_index--) {
                                    if (truck_route[launch_index] == truck_route[index]) {
                                        break;
                                    }
                                    current_l_of_truck_trip += (launch_index == rep_index - 1)
                                                                   ? instance->tau[truck_route[launch_index]][
                                                                       truck_route[index]]
                                                                   : instance->tau[truck_route[launch_index]][
                                                                       truck_route[
                                                                           launch_index + 1]];


                                    if (const double l_of_drone_trip = instance->tau_prime[truck_route[
                                                launch_index]][l_of]
                                            + instance->tau_prime[l_of][truck_route[index]]; l_of_drone_trip <=
                                        instance->e - instance->sr) {
                                        // the current_l_of_truck_trip contains route with "index"
                                        // while the "old_cost" doesn't.

                                        const double old_cost = l_of_trip + current_l_of_truck_trip
                                            - instance->tau[truck_route[rep_index - 1]][truck_route[index]]
                                            + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]];
                                        const double new_cost = std::max(
                                                current_l_of_truck_trip, l_of_drone_trip)
                                            + std::max(rep_new_truck_trip, rep_new_drone_trip);
                                        if (const double saved = old_cost - new_cost; saved > 1e-3 && saved >
                                            best_second_case_saved_cost) {
                                            revisitable = true;
                                            best_second_case_saved_cost = saved;
                                            best_launch_index = launch_index;
                                            best_l_of_truck_trip = current_l_of_truck_trip;
                                            best_l_of_drone_trip = l_of_drone_trip;
                                            best_rep_truck_trip = rep_new_truck_trip;
                                            best_rep_drone_trip = rep_new_drone_trip;
                                        }
                                    }
                                }
                            }
                        }
                        if (revisitable) {
                            // rep update.
                            if (bool exist = std::ranges::find(served_by_drone, truck_route[rep_index]) !=
                                served_by_drone.end(); !exist) {
                                served_by_drone.push_back(truck_route[rep_index]);
                            }
                            sortie_stages[truck_route[rep_index]] = {
                                rep_index, sortie_stages[l_of].end_index, best_rep_drone_trip, best_rep_truck_trip
                            };
                            sortie_stages[l_of] = {
                                best_launch_index, rep_index, best_l_of_drone_trip, best_l_of_truck_trip
                            };
                            truck_route[rep_index] = truck_route[index];
                            goto tag;
                        }
                    }
                    if (!is_l && is_r) {
                        if (truck_route[index] == truck_route[sortie_stages[r_of].start_index]) {
                            continue;
                        }
                        int earliest_rendezvous_index = -1;
                        for (const auto& [fst, snd] : spaces) {
                            if (fst == rep_index) {
                                earliest_rendezvous_index = snd;
                                break;
                            }
                        }
                        double best_first_case_saved_cost = -1e9;
                        int best_rendezvous_index = -1; // in the free set. {}
                        double best_rep_truck_trip = -1;
                        double best_rep_drone_trip = -1;
                        double best_r_of_truck_trip = -1;
                        double best_r_of_drone_trip = -1;
                        const double r_of_trip = std::max(sortie_stages[r_of].truck_trip,
                                                          sortie_stages[r_of].drone_trip);
                        // we do the first case:
                        // (start - r_of - index, index - rep_index - {free set of nodes})
                        if (const double new_r_of_truck_trip = sortie_stages[r_of].truck_trip -
                                instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                + instance->tau[truck_route[rep_index - 1]][truck_route[index]];
                            new_r_of_truck_trip <= instance->e - instance->sr) {
                            // drone trip of r_of
                            if (const double new_r_of_drone_trip = instance->tau_prime[truck_route[sortie_stages[r_of].
                                        start_index]][r_of]
                                    + instance->tau_prime[r_of][truck_route[index]]; new_r_of_drone_trip <= instance->e
                                -
                                instance->sr) {
                                // okay.
                                // check the rep_index stats.
                                double current_rep_index_truck_trip = 0;
                                for (int rendezvous_index = rep_index + 1; rendezvous_index <= earliest_rendezvous_index
                                     ; rendezvous_index++) {
                                    if (truck_route[index] == truck_route[rendezvous_index]) {
                                        break;
                                    }
                                    current_rep_index_truck_trip += (rendezvous_index == rep_index + 1)
                                                                        ? instance->tau[truck_route[index]][
                                                                            truck_route[rendezvous_index]]
                                                                        : instance->tau[truck_route[rendezvous_index -
                                                                            1]]
                                                                        [truck_route[rendezvous_index]];

                                    if (current_rep_index_truck_trip > instance->e - instance->sr) {
                                        break;
                                    }


                                    if (const double rep_drone_trip = instance->tau_prime[truck_route[index]][
                                                truck_route[rep_index]]
                                            + instance->tau_prime[truck_route[rep_index]][truck_route[
                                                rendezvous_index]]
                                        ; rep_drone_trip <= instance->e - instance->sr) {
                                        const double old_cost = r_of_trip + current_rep_index_truck_trip
                                            - instance->tau[truck_route[index]][truck_route[rep_index + 1]]
                                            + instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]];
                                        const double new_cost = std::max(
                                                new_r_of_drone_trip, new_r_of_truck_trip)
                                            + std::max(current_rep_index_truck_trip, rep_drone_trip);
                                        if (const double saved = old_cost - new_cost; saved > 1e-3 && saved >
                                            best_first_case_saved_cost) {
                                            revisitable = true;
                                            best_first_case_saved_cost = saved;
                                            best_rendezvous_index = rendezvous_index;
                                            best_r_of_truck_trip = new_r_of_truck_trip;
                                            best_r_of_drone_trip = new_r_of_drone_trip;
                                            best_rep_truck_trip = current_rep_index_truck_trip;
                                            best_rep_drone_trip = rep_drone_trip;
                                        }
                                    }
                                }
                            }
                        }
                        if (revisitable) {
                            sortie_stages[r_of].truck_trip = best_r_of_truck_trip;
                            sortie_stages[r_of].drone_trip = best_r_of_drone_trip;
                            if (bool exist = std::ranges::find(served_by_drone, truck_route[rep_index]) !=
                                served_by_drone.end(); !exist) {
                                served_by_drone.push_back(truck_route[rep_index]);
                            }
                            sortie_stages[truck_route[rep_index]] = {
                                rep_index, best_rendezvous_index, best_rep_drone_trip, best_rep_truck_trip
                            };
                            truck_route[rep_index] = truck_route[index];

                            goto tag;
                        }
                        // else?
                        // we check the second case.
                        // (start_r_of - rep - index), (index - r_of - free set)
                        double best_second_case_saved_cost = -1e9;
                        if (const double rep_new_truck_trip = sortie_stages[r_of].truck_trip
                                - instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                + instance->tau[truck_route[rep_index - 1]][truck_route[index]];
                            rep_new_truck_trip <= instance->e - instance->sr) {
                            if (const double rep_new_drone_trip = instance->tau_prime[truck_route[sortie_stages[r_of].
                                        start_index]][truck_route[rep_index]]
                                    + instance->tau_prime[truck_route[rep_index]][truck_route[index]];
                                rep_new_drone_trip <=
                                instance->e - instance->sr) {
                                double current_r_of_truck_trip = 0;
                                for (int rendezvous_index = rep_index + 1; rendezvous_index <= earliest_rendezvous_index
                                     ; rendezvous_index++) {
                                    if (truck_route[index] == truck_route[rendezvous_index]) {
                                        break;
                                    }
                                    current_r_of_truck_trip += (rendezvous_index == rep_index + 1)
                                                                   ? instance->tau[truck_route[index]][truck_route[
                                                                       rendezvous_index]]
                                                                   : instance->tau[truck_route[rendezvous_index - 1]]
                                                                   [truck_route[rendezvous_index]];

                                    if (current_r_of_truck_trip > instance->e - instance->sr) {
                                        break;
                                    }
                                    if (const double r_of_drone_trip = instance->tau_prime[truck_route[index]][
                                                r_of]
                                            + instance->tau_prime[r_of][truck_route[rendezvous_index]];
                                        r_of_drone_trip
                                        <= instance->e - instance->sr) {
                                        const double old_cost = r_of_trip + current_r_of_truck_trip
                                            - instance->tau[truck_route[index]][truck_route[rep_index + 1]]
                                            + instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]];
                                        const double new_cost = std::max(
                                                current_r_of_truck_trip, r_of_drone_trip)
                                            + std::max(rep_new_truck_trip, rep_new_drone_trip);
                                        if (const double saved = old_cost - new_cost; saved > 1e-3 && saved >
                                            best_second_case_saved_cost) {
                                            revisitable = true;
                                            best_second_case_saved_cost = saved;
                                            best_rendezvous_index = rendezvous_index;
                                            best_r_of_truck_trip = current_r_of_truck_trip;
                                            best_r_of_drone_trip = r_of_drone_trip;
                                            best_rep_truck_trip = rep_new_truck_trip;
                                            best_rep_drone_trip = rep_new_drone_trip;
                                        }
                                    }
                                }
                            }
                        }
                        if (revisitable) {
                            if (bool exist = std::ranges::find(served_by_drone, truck_route[rep_index]) !=
                                served_by_drone.end(); !exist) {
                                served_by_drone.push_back(truck_route[rep_index]);
                            }
                            sortie_stages[truck_route[rep_index]] = {
                                sortie_stages[r_of].start_index, rep_index, best_rep_drone_trip, best_rep_truck_trip
                            };
                            sortie_stages[r_of] = {
                                rep_index, best_rendezvous_index, best_r_of_drone_trip, best_r_of_truck_trip
                            };
                            truck_route[rep_index] = truck_route[index];
                            goto tag;
                        }
                    }
                }
            }
        }
    }
}


// RemoveSortie removes the start, end index and the drone-served customer c.
void SolutionL::remove_sortie(const int c) {
    const int start_index = sortie_stages[c].start_index;
    const int end_index = sortie_stages[c].end_index;
    const int start_node = truck_route[start_index];
    const int end_node = truck_route[end_index];
    int end_of = -1, start_of = -1, end_of_start_of = -1, start_of_end_of = -1;
    int removed = 2; // 2 is max.
    for (const int d : served_by_drone) {
        if (sortie_stages[d].start_index == end_index) {
            start_of = d;
            end_of_start_of = sortie_stages[d].end_index;
        }
        if (sortie_stages[d].end_index == start_index) {
            end_of = d;
            start_of_end_of = sortie_stages[d].start_index;
        }
    }

    if (start_of == -1 && end_of == -1) {
        // PROCEDURE:
        // 1. delete sortie from served by drone
        // 2. update (decrease) all subsequence sortie indexes
        // 3. perform deletion in truck route
        std::erase(served_by_drone, c);
        if (start_index == 0) {
            removed -= 1;
        }
        if (end_index == truck_route.size() - 1) {
            removed -= 1;
        }
        // all sortie after end index would need to be updated
        for (const int d : served_by_drone) {
            if (sortie_stages[d].start_index >= end_index) {
                sortie_stages[d].start_index -= removed;
                sortie_stages[d].end_index -= removed;
                continue;
            }
            if (sortie_stages[d].end_index >= end_index) {
                sortie_stages[d].end_index -= removed;
            }
        }
        if (start_index == 0 && end_index == truck_route.size() - 1) {}
        else if (start_index == 0 && end_index != truck_route.size() - 1) {
            truck_route.erase(truck_route.begin() + end_index);
            if (const bool ext = existL(truck_route, end_node); !ext) {
                visited[end_node] = false;
            }
        }
        else if (start_index != 0 && end_index == truck_route.size() - 1) {
            truck_route.erase(truck_route.begin() + start_index);
            if (const bool ext = existL(truck_route, start_node); !ext) {
                visited[start_node] = false;
            }
        }
        else {
            truck_route.erase(truck_route.begin() + end_index);
            truck_route.erase(truck_route.begin() + start_index);
            if (const bool ext = existL(truck_route, start_node); !ext) {
                visited[start_node] = false;
            }
            if (const bool ext = existL(truck_route, end_node); !ext) {
                visited[end_node] = false;
            }
        }
        bool removed_all = true;
        for (const int i : truck_route) {
            if (i == c) {
                removed_all = false;
            }
        }
        if (removed_all) {
            visited[c] = false;
        }
        //
    }
    else if (start_of != -1 && end_of == -1) {
        if (end_index - start_index > 1) {
            if (truck_route[end_index - 1] == truck_route[end_of_start_of]) {
                return;
            }
        }
        else {
            if (end_index >= 2) {
                if (truck_route[end_index - 2] == truck_route[end_of_start_of]) {
                    return;
                }
            }
        }
        double new_drone_trip = 0;
        if (end_index - start_index > 1) {
            new_drone_trip = instance->tau_prime[truck_route[end_index - 1]][start_of]
                + instance->tau_prime[start_of][truck_route[end_of_start_of]];
        }
        else {
            if (start_index != 0) {
                new_drone_trip = instance->tau_prime[truck_route[end_index - 2]][start_of]
                    + instance->tau_prime[start_of][truck_route[end_of_start_of]];
            }
            else {
                new_drone_trip = instance->tau_prime[truck_route[end_index - 1]][start_of]
                    + instance->tau_prime[start_of][truck_route[end_of_start_of]];
            }
        }

        if (new_drone_trip > instance->e - instance->sr) {
            return;
        }
        double new_truck_trip = 0;
        if (end_index - start_index > 1) {
            new_truck_trip = sortie_stages[start_of].truck_trip
                - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
                + instance->tau[truck_route[end_index - 1]][truck_route[end_index + 1]];
        }
        else {
            if (start_index != 0) {
                new_truck_trip = sortie_stages[start_of].truck_trip
                    - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
                    + instance->tau[truck_route[end_index - 2]][truck_route[end_index + 1]];
            }
            else {
                new_truck_trip = sortie_stages[start_of].truck_trip
                    - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
                    + instance->tau[truck_route[end_index - 1]][truck_route[end_index + 1]];
            }
        }

        if (new_truck_trip > instance->e - instance->sr) {
            return;
        }
        if (start_index == 0) {
            removed -= 1;
        }
        if (end_index == truck_route.size() - 1) {
            removed -= 1;
        }
        // ok, proceed.
        sortie_stages[start_of].truck_trip = new_truck_trip;
        sortie_stages[start_of].drone_trip = new_drone_trip;
        std::erase(served_by_drone, c);

        // all sortie after end index would need to be updated
        for (const int d : served_by_drone) {
            if (sortie_stages[d].start_index >= end_index) {
                sortie_stages[d].start_index -= removed;
                sortie_stages[d].end_index -= removed;
                continue;
            }
            if (sortie_stages[d].end_index >= end_index) {
                sortie_stages[d].end_index -= removed;
            }
        }
        if (start_index == 0 && end_index == truck_route.size() - 1) {
            return;
        }
        else if (start_index == 0 && end_index != truck_route.size() - 1) {
            truck_route.erase(truck_route.begin() + end_index);
            if (const bool ext = existL(truck_route, end_node); !ext) {
                visited[end_node] = false;
            }
        }
        else if (start_index != 0 && end_index == truck_route.size() - 1) {
            truck_route.erase(truck_route.begin() + start_index);
            if (const bool ext = existL(truck_route, start_node); !ext) {
                visited[start_node] = false;
            }
        }
        else {
            truck_route.erase(truck_route.begin() + end_index);
            truck_route.erase(truck_route.begin() + start_index);
            if (const bool ext = existL(truck_route, start_node); !ext) {
                visited[start_node] = false;
            }
            if (const bool ext = existL(truck_route, end_node); !ext) {
                visited[end_node] = false;
            }
        }
        bool removed_all = true;
        for (const int i : truck_route) {
            if (i == c) {
                removed_all = false;
            }
        }
        if (removed_all) {
            visited[c] = false;
        }
    }
    else if (start_of == -1 && end_of != -1) {
        double new_drone_trip = 0, new_truck_trip = 0;
        if (end_index - start_index > 1) {
            if (truck_route[start_of_end_of] == truck_route[start_index + 1]) {
                return;
            }
            new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
                + instance->tau_prime[end_of][truck_route[start_index + 1]];
        }
        else {
            if (end_index != truck_route.size() - 1) {
                if (truck_route[start_of_end_of] == truck_route[end_index + 1]) {
                    return;
                }
                new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
                    + instance->tau_prime[end_of][truck_route[end_index + 1]];
            }
            else {
                new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
                    + instance->tau_prime[end_of][truck_route[end_index]];
            }
        }

        if (new_drone_trip > instance->e - instance->sr) {
            return;
        }
        if (end_index - start_index > 1) {
            new_truck_trip = sortie_stages[end_of].truck_trip
                - instance->tau[truck_route[start_index - 1]][truck_route[start_index]]
                + instance->tau[truck_route[start_index - 1]][truck_route[start_index + 1]];
        }
        else {
            if (end_index != truck_route.size() - 1) {
                new_truck_trip = sortie_stages[end_of].truck_trip
                    - instance->tau[truck_route[start_index - 1]][truck_route[start_index]]
                    + instance->tau[truck_route[start_index - 1]][truck_route[end_index + 1]];
            }
            else {
                new_truck_trip = sortie_stages[end_of].truck_trip
                    - instance->tau[truck_route[start_index - 1]][truck_route[start_index]]
                    + instance->tau[truck_route[start_index - 1]][truck_route[end_index]];
            }
        }

        if (new_truck_trip > instance->e - instance->sr) {
            return;
        }
        if (start_index == 0) {
            removed -= 1;
        }
        if (end_index == truck_route.size() - 1) {
            removed -= 1;
        }
        // ok. proceed to delete.
        sortie_stages[end_of].truck_trip = new_truck_trip;
        sortie_stages[end_of].drone_trip = new_drone_trip;
        std::erase(served_by_drone, c);

        // all sortie after end index would need to be updated
        for (const int d : served_by_drone) {
            if (sortie_stages[d].start_index >= end_index) {
                sortie_stages[d].start_index -= removed;
                sortie_stages[d].end_index -= removed;
                continue;
            }
            if (sortie_stages[d].end_index >= end_index) {
                sortie_stages[d].end_index -= removed;
            }
        }

        if (start_index == 0 && end_index != truck_route.size() - 1) {
            truck_route.erase(truck_route.begin() + end_index);
            if (const bool ext = existL(truck_route, end_node); !ext) {
                visited[end_node] = false;
            }
        }
        else if (start_index != 0 && end_index == truck_route.size() - 1) {
            truck_route.erase(truck_route.begin() + start_index);
            if (const bool ext = existL(truck_route, start_node); !ext) {
                visited[start_node] = false;
            }
        }
        else {
            truck_route.erase(truck_route.begin() + end_index);
            truck_route.erase(truck_route.begin() + start_index);
            if (const bool ext = existL(truck_route, start_node); !ext) {
                visited[start_node] = false;
            }
            if (const bool ext = existL(truck_route, end_node); !ext) {
                visited[end_node] = false;
            }
        }
        bool removed_all = true;
        for (const int i : truck_route) {
            if (i == c) {
                removed_all = false;
            }
        }
        if (removed_all) {
            visited[c] = false;
        }
    }
    else if (start_of != -1 && end_of != -1) {
        // both
        // chi delete sortie.
        std::erase(served_by_drone, c);
        //
        bool removed_all = true;
        for (const int i : truck_route) {
            if (i == c) {
                removed_all = false;
            }
        }
        if (removed_all) {
            visited[c] = false;
        }
    }
    //
}

int SolutionL::remove_highest_rate_sortie(const int c) {
    const int start_index = sortie_stages[c].start_index;
    const int end_index = sortie_stages[c].end_index;
    int end_of = -1, start_of = -1, end_of_start_of = -1, start_of_end_of = -1;
    int removed = end_index - start_index + 1;
    for (const int d : served_by_drone) {
        if (sortie_stages[d].start_index == end_index) {
            start_of = d;
            end_of_start_of = sortie_stages[d].end_index;
        }
        if (sortie_stages[d].end_index == start_index) {
            end_of = d;
            start_of_end_of = sortie_stages[d].start_index;
        }
    }

    if (start_index == 0) {
        removed -= 1;
    }
    if (end_index == truck_route.size() - 1) {
        removed -= 1;
    }
    if (start_of == -1 && end_of == -1) {
        // PROCEDURE:
        // 1. delete sortie from served by drone
        // 2. update (decrease) all subsequence sortie indexes
        // 3. perform deletion in truck route
        // 4. if deletion is feasible: return removed + 1.

        if (start_index == 0 && end_index == truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;
            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }
            for (int idx = 1; idx < truck_route.size() - 1; idx++) {
                visited[truck_route[idx]] = false;
            }
            truck_route.erase(truck_route.begin() + 1, truck_route.begin() + end_index);

            return removed + 1;
        }
        else if (start_index == 0 && end_index != truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            // only count the outer.
            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }

            for (int idx = start_index + 1; idx <= end_index; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index + 1, truck_route.begin() + end_index + 1);

            return removed + 1;
        }
        else if (start_index != 0 && end_index == truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index; idx <= end_index - 1; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index, truck_route.begin() + end_index);

            return removed + 1;
        }
        else {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index; idx <= end_index; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index, truck_route.begin() + end_index + 1);

            return removed + 1;
        }
    }
    else if (start_of != -1 && end_of == -1) {
        if (start_index > 1) {
            if (truck_route[start_index - 1] == truck_route[end_of_start_of]) {
                return 0;
            }
        }

        double new_drone_trip = 0, new_truck_trip = 0;
        if (start_index <= 1) {
            new_drone_trip = instance->tau_prime[0][start_of]
                + instance->tau_prime[start_of][truck_route[end_of_start_of]];
            if (new_drone_trip > instance->e - instance->sr) {
                return 0;
            }
            new_truck_trip = sortie_stages[start_of].truck_trip
                - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
                + instance->tau[0][truck_route[end_index + 1]];
            if (new_truck_trip > instance->e - instance->sr) {
                return 0;
            }
        }
        else {
            new_drone_trip = instance->tau_prime[truck_route[start_index - 1]][start_of]
                + instance->tau_prime[start_of][truck_route[end_of_start_of]];
            if (new_drone_trip > instance->e - instance->sr) {
                return 0;
            }
            new_truck_trip = sortie_stages[start_of].truck_trip
                - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
                + instance->tau[truck_route[start_index - 1]][truck_route[end_index + 1]];
            if (new_truck_trip > instance->e - instance->sr) {
                return 0;
            }
        }

        // ok. proceed
        sortie_stages[start_of].truck_trip = new_truck_trip;
        sortie_stages[start_of].drone_trip = new_drone_trip;
        if (start_index == 0 && end_index == truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;
            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }
            for (int idx = 1; idx < truck_route.size() - 1; idx++) {
                visited[truck_route[idx]] = false;
            }
            truck_route.erase(truck_route.begin() + 1, truck_route.begin() + end_index);

            return removed + 1;
        }
        else if (start_index == 0 && end_index != truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index + 1; idx <= end_index; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index + 1, truck_route.begin() + end_index + 1);


            return removed + 1;
        }
        else if (start_index != 0 && end_index == truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index; idx <= end_index - 1; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index, truck_route.begin() + end_index);


            return removed + 1;
        }
        else {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index; idx <= end_index; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index, truck_route.begin() + end_index + 1);

            return removed + 1;
        }
    }
    else if (start_of == -1 && end_of != -1) {
        if (end_index < truck_route.size() - 1) {
            if (truck_route[start_of_end_of] == truck_route[end_index + 1]) {
                return 0;
            }
        }

        double new_drone_trip = 0, new_truck_trip = 0;
        if (end_index == truck_route.size() - 1) {
            new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
                + instance->tau_prime[end_of][0];
            if (new_drone_trip > instance->e - instance->sr) {
                return 0;
            }
            new_truck_trip = sortie_stages[end_of].truck_trip
                - instance->tau[truck_route[start_index - 1]][truck_route[start_index]]
                + instance->tau[truck_route[start_index - 1]][truck_route[0]];
            if (new_truck_trip > instance->e - instance->sr) {
                return 0;
            }
        }
        else {
            new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
                + instance->tau_prime[end_of][truck_route[end_index + 1]];
            if (new_drone_trip > instance->e - instance->sr) {
                return 0;
            }
            new_truck_trip = sortie_stages[end_of].truck_trip
                - instance->tau[truck_route[start_index - 1]][truck_route[start_index]]
                + instance->tau[truck_route[start_index - 1]][truck_route[end_index + 1]];
            if (new_truck_trip > instance->e - instance->sr) {
                return 0;
            }
        }
        // ok. proceed
        sortie_stages[end_of].truck_trip = new_truck_trip;
        sortie_stages[end_of].drone_trip = new_drone_trip;
        if (start_index == 0 && end_index == truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;
            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }
            for (int idx = 1; idx < truck_route.size() - 1; idx++) {
                visited[truck_route[idx]] = false;
            }
            truck_route.erase(truck_route.begin() + 1, truck_route.begin() + end_index);


            return removed + 1;
        }
        else if (start_index == 0 && end_index != truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index + 1; idx <= end_index; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index + 1, truck_route.begin() + end_index + 1);


            return removed + 1;
        }
        else if (start_index != 0 && end_index == truck_route.size() - 1) {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index; idx <= end_index - 1; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index, truck_route.begin() + end_index);


            return removed + 1;
        }
        else {
            std::erase(served_by_drone, c);
            visited[c] = false;

            // all sortie after end index would need to be updated
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index >= end_index) {
                    sortie_stages[d].start_index -= removed;
                    sortie_stages[d].end_index -= removed;
                    continue;
                }
                if (sortie_stages[d].end_index >= end_index) {
                    sortie_stages[d].end_index -= removed;
                }
            }

            std::vector<int> count(instance->num_node, 0);

            for (int idx = 1; idx < start_index; idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = end_index + 1; idx < truck_route.size(); idx++) {
                count[truck_route[idx]]++;
            }
            for (int idx = start_index; idx <= end_index; idx++) {
                if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
                // is not sure
            }
            truck_route.erase(truck_route.begin() + start_index, truck_route.begin() + end_index + 1);


            return removed + 1;
        }
    }
    else if (start_of != -1 && end_of != -1) {
        removed = end_index - start_index - 1;
        std::erase(served_by_drone, c);
        visited[c] = false;

        // all sortie after end index would need to be updated
        for (const int d : served_by_drone) {
            if (sortie_stages[d].start_index >= end_index) {
                sortie_stages[d].start_index -= removed;
                sortie_stages[d].end_index -= removed;
                continue;
            }
            if (sortie_stages[d].end_index >= end_index) {
                sortie_stages[d].end_index -= removed;
            }
        }

        std::vector<int> count(instance->num_node, 0);

        for (int idx = 1; idx <= start_index; idx++) {
            count[truck_route[idx]]++;
        }
        for (int idx = end_index; idx < truck_route.size(); idx++) {
            count[truck_route[idx]]++;
        }
        for (int idx = start_index + 1; idx <= end_index - 1; idx++) {
            if (count[truck_route[idx]] == 0) { visited[truck_route[idx]] = false; }
            // is not sure
        }
        truck_route.erase(truck_route.begin() + start_index + 1, truck_route.begin() + end_index);

        return removed + 1;
    }
    return 0;


    //     double new_drone_trip = 0;
    //     if (end_index - start_index > 1) {
    //         new_drone_trip = instance->tau_prime[truck_route[end_index - 1]][start_of]
    //             + instance->tau_prime[start_of][truck_route[end_of_start_of]];
    //     }
    //     else {
    //         if (start_index != 0) {
    //             new_drone_trip = instance->tau_prime[truck_route[end_index - 2]][start_of]
    //                 + instance->tau_prime[start_of][truck_route[end_of_start_of]];
    //         }
    //         else {
    //             new_drone_trip = instance->tau_prime[truck_route[end_index - 1]][start_of]
    //                 + instance->tau_prime[start_of][truck_route[end_of_start_of]];
    //         }
    //     }
    //
    //     if (new_drone_trip > instance->e - instance->sr) {
    //         return;
    //     }
    //     double new_truck_trip = 0;
    //     if (end_index - start_index > 1) {
    //         new_truck_trip = sortie_stages[start_of].truck_trip
    //             - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
    //             + instance->tau[truck_route[end_index - 1]][truck_route[end_index + 1]];
    //     }
    //     else {
    //         if (start_index != 0) {
    //             new_truck_trip = sortie_stages[start_of].truck_trip
    //                 - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
    //                 + instance->tau[truck_route[end_index - 2]][truck_route[end_index + 1]];
    //         }
    //         else {
    //             new_truck_trip = sortie_stages[start_of].truck_trip
    //                 - instance->tau[truck_route[end_index]][truck_route[end_index + 1]]
    //                 + instance->tau[truck_route[end_index - 1]][truck_route[end_index + 1]];
    //         }
    //     }
    //
    //     if (new_truck_trip > instance->e - instance->sr) {
    //         return;
    //     }
    //     if (start_index == 0) {
    //         removed -= 1;
    //     }
    //     if (end_index == truck_route.size() - 1) {
    //         removed -= 1;
    //     }
    //     // ok, proceed.
    //     sortie_stages[start_of].truck_trip = new_truck_trip;
    //     sortie_stages[start_of].drone_trip = new_drone_trip;
    //     std::erase(served_by_drone, c);
    //
    //     // all sortie after end index would need to be updated
    //     for (const int d : served_by_drone) {
    //         if (sortie_stages[d].start_index >= end_index) {
    //             sortie_stages[d].start_index -= removed;
    //             sortie_stages[d].end_index -= removed;
    //             continue;
    //         }
    //         if (sortie_stages[d].end_index >= end_index) {
    //             sortie_stages[d].end_index -= removed;
    //         }
    //     }
    //     if (start_index == 0 && end_index == truck_route.size() - 1) {
    //         return;
    //     }
    //     else if (start_index == 0 && end_index != truck_route.size() - 1) {
    //         truck_route.erase(truck_route.begin() + end_index);
    //         if (const bool ext = exist(truck_route, end_node); !ext) {
    //             visited[end_node] = false;
    //         }
    //     }
    //     else if (start_index != 0 && end_index == truck_route.size() - 1) {
    //         truck_route.erase(truck_route.begin() + start_index);
    //         if (const bool ext = exist(truck_route, start_node); !ext) {
    //             visited[start_node] = false;
    //         }
    //     }
    //     else {
    //         truck_route.erase(truck_route.begin() + end_index);
    //         truck_route.erase(truck_route.begin() + start_index);
    //         if (const bool ext = exist(truck_route, start_node); !ext) {
    //             visited[start_node] = false;
    //         }
    //         if (const bool ext = exist(truck_route, end_node); !ext) {
    //             visited[end_node] = false;
    //         }
    //     }
    //     bool removed_all = true;
    //     for (const int i : truck_route) {
    //         if (i == c) {
    //             removed_all = false;
    //         }
    //     }
    //     if (removed_all) {
    //         visited[c] = false;
    //     }
    //
    // }
    // else if (start_of == -1 && end_of != -1) {
    //     if (truck_route[start_of_end_of] == truck_route[start_index + 1]) {
    //         return;
    //     }
    //     double new_drone_trip = 0, new_truck_trip = 0;
    //     if (end_index - start_index > 1) {
    //         new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
    //         + instance->tau_prime[end_of][truck_route[start_index+1]];
    //     } else {
    //         if (end_index != truck_route.size() - 1) {
    //             new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
    //         + instance->tau_prime[end_of][truck_route[end_index+1]];
    //         } else {
    //             new_drone_trip = instance->tau_prime[truck_route[start_of_end_of]][end_of]
    //             + instance->tau_prime[end_of][truck_route[end_index]];
    //         }
    //     }
    //
    //     if (new_drone_trip > instance->e - instance->sr) {
    //         return;
    //     }
    //     if (end_index - start_index > 1) {
    //         new_truck_trip = sortie_stages[end_of].truck_trip
    //         - instance->tau[truck_route[start_index-1]][truck_route[start_index]]
    //         + instance->tau[truck_route[start_index-1]][truck_route[start_index+1]];
    //     } else {
    //         if (end_index != truck_route.size()-1) {
    //
    //             new_truck_trip = sortie_stages[end_of].truck_trip
    //             - instance->tau[truck_route[start_index-1]][truck_route[start_index]]
    //             + instance->tau[truck_route[start_index-1]][truck_route[end_index+1]];
    //         } else {
    //             new_truck_trip = sortie_stages[end_of].truck_trip
    //             - instance->tau[truck_route[start_index-1]][truck_route[start_index]]
    //             + instance->tau[truck_route[start_index-1]][truck_route[end_index]];
    //         }
    //     }
    //
    //     if (new_truck_trip > instance->e - instance->sr) {
    //         return;
    //     }
    //     if (start_index == 0) {
    //         removed -= 1;
    //     }
    //     if (end_index == truck_route.size() - 1) {
    //         removed -= 1;
    //     }
    //     // ok. proceed to delete.
    //     sortie_stages[end_of].truck_trip = new_truck_trip;
    //     sortie_stages[end_of].drone_trip = new_drone_trip;
    //     std::erase(served_by_drone, c);
    //
    //     // all sortie after end index would need to be updated
    //     for (const int d : served_by_drone) {
    //         if (sortie_stages[d].start_index >= end_index) {
    //             sortie_stages[d].start_index -= removed;
    //             sortie_stages[d].end_index -= removed;
    //             continue;
    //         }
    //         if (sortie_stages[d].end_index >= end_index) {
    //             sortie_stages[d].end_index -= removed;
    //         }
    //     }
    //
    //     if (start_index == 0 && end_index != truck_route.size() - 1) {
    //         truck_route.erase(truck_route.begin() + end_index);
    //         if (const bool ext = exist(truck_route, end_node); !ext) {
    //             visited[end_node] = false;
    //         }
    //     }
    //     else if (start_index != 0 && end_index == truck_route.size() - 1) {
    //         truck_route.erase(truck_route.begin() + start_index);
    //         if (const bool ext = exist(truck_route, start_node); !ext) {
    //             visited[start_node] = false;
    //         }
    //     }
    //     else {
    //         truck_route.erase(truck_route.begin() + end_index);
    //         truck_route.erase(truck_route.begin() + start_index);
    //         if (const bool ext = exist(truck_route, start_node); !ext) {
    //             visited[start_node] = false;
    //         }
    //         if (const bool ext = exist(truck_route, end_node); !ext) {
    //             visited[end_node] = false;
    //         }
    //     }
    //     bool removed_all = true;
    //     for (const int i : truck_route) {
    //         if (i == c) {
    //             removed_all = false;
    //         }
    //     }
    //     if (removed_all) {
    //         visited[c] = false;
    //     }
    //
    // }
    // else if (start_of != -1 && end_of != -1) {
    //     // both
    //     // chi delete sortie.
    //     std::erase(served_by_drone, c);
    //     bool removed_all = true;
    //     for (const int i : truck_route) {
    //         if (i == c) {
    //             removed_all = false;
    //         }
    //     }
    //     if (removed_all) {
    //         visited[c] = false;
    //     }
    // }
}

void SolutionL::duplicate_node_cleaner() {
    std::unordered_map<int, int> countMap;
    std::vector<int> duplicateIndices;

    // Combine the counting and index collection in a single pass
    for (int i = 0; i < truck_route.size(); ++i) {
        if (int element = truck_route[i]; ++countMap[element] > 1) {
            duplicateIndices.push_back(i); // Collect index if element is a duplicate
        }
    }
    if (duplicateIndices.empty()) {
        return;
    }

    // Sort indices in descending order if needed
    std::ranges::sort(duplicateIndices, std::greater<int>());

    for (const int index : duplicateIndices) {
        const int node = truck_route[index];
        bool in_sortie = false;

        // Efficiently check if the current index is within any sortie
        for (const int d : served_by_drone) {
            if (sortie_stages[d].start_index <= index && index <= sortie_stages[d].end_index) {
                in_sortie = true;
                break; // Early exit if found in sortie
            }
        }

        if (!in_sortie) {
            // Update sortie ranges if needed
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index > index) {
                    sortie_stages[d].start_index--;
                    sortie_stages[d].end_index--;
                }
                else if (sortie_stages[d].end_index > index) {
                    sortie_stages[d].end_index--;
                }
            }

            // Remove the duplicate node from truck_route
            truck_route.erase(truck_route.begin() + index);

            // Check if the node no longer exists in truck_route
            if (std::ranges::find(truck_route, node) == truck_route.end()) {
                visited[node] = false; // Mark as not visited if all occurrences are removed
            }


        }
    }

    for (const int d : served_by_drone) {
        if (sortie_stages[d].end_index - sortie_stages[d].start_index >= 2) {
            for (int i = sortie_stages[d].end_index-1; i >= sortie_stages[d].start_index+1; i--) {
                if (truck_route[i] == truck_route[sortie_stages[d].start_index] || truck_route[i] == truck_route[
                    sortie_stages[d].end_index]) {
                    // remove that i.
                    sortie_stages[d].truck_trip = sortie_stages[d].truck_trip
                        - instance->tau[truck_route[i - 1]][truck_route[i]]
                        - instance->tau[truck_route[i]][truck_route[i + 1]]
                        + instance->tau[truck_route[i - 1]][truck_route[i + 1]];
                    for (const int dp : served_by_drone) {
                        if (sortie_stages[dp].start_index > i) {
                            sortie_stages[dp].start_index--;
                            sortie_stages[dp].end_index--;
                            continue;
                        }
                        if (sortie_stages[dp].end_index > i) {
                            sortie_stages[dp].end_index--;
                        }
                    }
                    truck_route.erase(truck_route.begin() + i);

                }
            }
        }
    }

    for (int i = truck_route.size() - 1; i > 1; i--) {
        if (truck_route[i] == truck_route[i - 1]) {
            // check if no sortie between.
            bool del = true;
            for (const int d : served_by_drone) {
                if (sortie_stages[d].start_index == i - 1 && sortie_stages[d].end_index == i) {
                    del = false;
                    break;
                }
            }
            if (del) {

                for (const int d : served_by_drone) {
                    if (sortie_stages[d].start_index >= i) {
                        sortie_stages[d].start_index--;
                        sortie_stages[d].end_index--;
                    }
                }
                truck_route.erase(truck_route.begin() + i);


            }
        }
    }
}

void SolutionL::loop_search() {
    bool loopable = true;
tag:
    while (loopable) {
        loopable = false;
        std::unordered_map<int, int> countMap;
        std::vector<int> duplicateIndices;

        // First pass: Count occurrences of each element
        for (const auto& element : truck_route) {
            countMap[element]++;
        }
        std::vector<int> index_is_start(truck_route.size(), -1);
        std::vector<int> index_is_mid(truck_route.size(), -1);
        std::vector<int> index_is_end(truck_route.size(), -1);

        for (const int d : served_by_drone) {
            for (int index = sortie_stages[d].start_index; index <= sortie_stages[d].end_index; index++) {
                if (index == sortie_stages[d].start_index) {
                    index_is_start[index] = d;
                    continue;
                }
                if (index == sortie_stages[d].end_index) {
                    index_is_end[index] = d;
                    continue;
                }
                index_is_mid[index] = d;
            }
        }

        auto loop_spaces = find_loop_spaces();

        // sort the loop space ascending order.
        std::ranges::sort(loop_spaces, [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
            return a.first < b.first;
        });

        for (const auto& [fst, snd] : loop_spaces) {
            for (int index = fst; index <= snd; index++) {
                if (index == 0 || index == truck_route.size() - 1) {
                    continue;
                }
                // index in this space: can replace next indices.
                // try to replace the next: any good move will be used.
                for (int rep_index = index + 1; rep_index < truck_route.size() - 1; rep_index++) {
                    // see if the rep_index is something?
                    if (truck_route[rep_index] == index || countMap[truck_route[rep_index]] > 1 || instance->heavy_bool[
                        truck_route[rep_index]]) {
                        continue;
                    }
                    // xet xem rep_index thuoc type gi?
                    if (index_is_start[rep_index] > -1 && index_is_end[rep_index] > -1) {
                        if (sortie_stages[index_is_start[rep_index]].end_index - sortie_stages[index_is_start[
                            rep_index]].start_index == 1) {
                            continue;
                        }
                        // vua la start vua la launch? can't delete?
                        // co the xet sau
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][truck_route[
                                rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            // left sortie (r_of).
                            if (const double new_r_of_drone_trip = instance->tau_prime[truck_route[sortie_stages[
                                        index_is_end[rep_index]].start_index]][index_is_end[rep_index]]
                                    + instance->tau_prime[index_is_end[rep_index]][truck_route[rep_index + 1]];
                                new_r_of_drone_trip <= instance->e - instance->sr) {
                                if (const double new_r_of_truck_trip = sortie_stages[index_is_end[rep_index]].truck_trip
                                        - instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                        + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                                    new_r_of_drone_trip <= instance->e - instance->sr) {
                                    // check the right sortie (l_of)
                                    if (const double new_l_of_drone_trip = instance->tau_prime[truck_route[rep_index +
                                                1]][index_is_start[rep_index]]
                                            + instance->tau_prime[index_is_start[rep_index]][truck_route[sortie_stages[
                                                index_is_start[rep_index]].end_index]];
                                        new_l_of_drone_trip <= instance->e - instance->sr) {
                                        const double new_l_of_truck_trip = sortie_stages[index_is_start[rep_index]].
                                            truck_trip
                                            - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]];
                                        const double old_cost = std::max(
                                                sortie_stages[index_is_start[rep_index]].truck_trip,
                                                sortie_stages[index_is_start[rep_index]].drone_trip)
                                            + std::max(sortie_stages[index_is_end[rep_index]].truck_trip,
                                                       sortie_stages[index_is_end[rep_index]].truck_trip);
                                        const double new_cost = new_drone_trip + instance->sl + instance->sr
                                            + std::max(new_l_of_truck_trip, new_l_of_drone_trip)
                                            + std::max(new_r_of_truck_trip, new_r_of_drone_trip);
                                        if (new_cost < old_cost) {
                                            loopable = true;
                                            for (const int d : served_by_drone) {
                                                if (sortie_stages[d].start_index >= index && sortie_stages[d].end_index
                                                    <= rep_index) {
                                                    sortie_stages[d].start_index++;
                                                    sortie_stages[d].end_index++;
                                                }
                                            }
                                            sortie_stages[index_is_start[rep_index]].start_index++;
                                            served_by_drone.push_back(truck_route[rep_index]);
                                            sortie_stages[truck_route[rep_index]] = {
                                                index, index + 1, new_drone_trip, 0.0
                                            };
                                            sortie_stages[index_is_start[rep_index]].truck_trip = new_l_of_truck_trip;
                                            sortie_stages[index_is_start[rep_index]].drone_trip = new_l_of_drone_trip;
                                            sortie_stages[index_is_end[rep_index]].truck_trip = new_r_of_truck_trip;
                                            sortie_stages[index_is_end[rep_index]].drone_trip = new_r_of_drone_trip;

                                            truck_route.erase(truck_route.begin() + rep_index);
                                            truck_route.insert(truck_route.begin() + index + 1, truck_route[index]);
                                            goto tag;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else if (index_is_start[rep_index] > -1 && index_is_end[rep_index] == -1) {
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][truck_route[
                                rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            if (const double affected_new_truck_trip = sortie_stages[index_is_start[rep_index]].
                                    truck_trip
                                    - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]]
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                                affected_new_truck_trip <= instance->e - instance->sr) {
                                if (const double affected_new_drone_trip = instance->tau_prime[truck_route[rep_index -
                                            1]][index_is_start[rep_index]]
                                        + instance->tau_prime[index_is_start[rep_index]][truck_route[sortie_stages[
                                            index_is_start[rep_index]].end_index]];
                                    affected_new_drone_trip <= instance->e - instance->sr) {
                                    const double old_cost = std::max(
                                        sortie_stages[index_is_start[rep_index]].truck_trip,
                                        sortie_stages[index_is_start[rep_index]].drone_trip);
                                    const double new_cost = std::max(affected_new_truck_trip, affected_new_drone_trip) +
                                        new_drone_trip + instance->sl + instance->sr;
                                    if (new_cost < old_cost) {
                                        loopable = true;
                                        for (const int d : served_by_drone) {
                                            if (sortie_stages[d].start_index >= index && sortie_stages[d].end_index <
                                                rep_index) {
                                                sortie_stages[d].start_index++;
                                                sortie_stages[d].end_index++;
                                            }
                                        }
                                        served_by_drone.push_back(truck_route[rep_index]);
                                        sortie_stages[truck_route[rep_index]] = {index, index + 1, new_drone_trip, 0};
                                        sortie_stages[index_is_start[rep_index]].truck_trip = affected_new_truck_trip;
                                        sortie_stages[index_is_start[rep_index]].drone_trip = affected_new_drone_trip;

                                        truck_route.erase(truck_route.begin() + rep_index);
                                        truck_route.insert(truck_route.begin() + index, truck_route[index]);
                                        goto tag;
                                    }
                                }
                            }
                        }
                    }
                    else if (index_is_end[rep_index] > -1 && index_is_start[rep_index] == -1) {
                        // chi la end thoi
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][truck_route[
                                rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            if (const double affected_new_truck_trip = sortie_stages[index_is_end[rep_index]].truck_trip
                                    - instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                                affected_new_truck_trip <= instance->e - instance->sr) {
                                if (const double affected_new_drone_trip = instance->tau_prime[truck_route[sortie_stages
                                            [index_is_end[rep_index]].start_index]][index_is_end[rep_index]]
                                        + instance->tau_prime[index_is_end[rep_index]][truck_route[rep_index + 1]];
                                    affected_new_drone_trip <= instance->e - instance->sr) {
                                    const double old_cost = std::max(sortie_stages[index_is_end[rep_index]].truck_trip,
                                                                     sortie_stages[index_is_end[rep_index]].drone_trip);
                                    const double new_cost = std::max(affected_new_truck_trip, affected_new_drone_trip)
                                        + new_drone_trip + instance->sl + instance->sr;
                                    if (new_cost < old_cost) {
                                        loopable = true;
                                        for (const int d : served_by_drone) {
                                            if (sortie_stages[d].start_index >= index && sortie_stages[d].end_index <=
                                                rep_index) {
                                                sortie_stages[d].start_index++;
                                                sortie_stages[d].end_index++;
                                            }
                                        }
                                        served_by_drone.push_back(truck_route[rep_index]);
                                        sortie_stages[truck_route[rep_index]] = {index, index + 1, new_drone_trip, 0};
                                        sortie_stages[index_is_end[rep_index]].truck_trip = affected_new_truck_trip;
                                        sortie_stages[index_is_end[rep_index]].drone_trip = affected_new_drone_trip;

                                        truck_route.erase(truck_route.begin() + rep_index);
                                        truck_route.insert(truck_route.begin() + index, truck_route[index]);
                                        goto tag;
                                    }
                                }
                            }
                        }
                    }
                    else if (index_is_mid[rep_index] > -1) {
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][truck_route[
                                rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            const double affected_new_truck_route = sortie_stages[index_is_mid[rep_index]].truck_trip
                                - instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]]
                                + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                            if (affected_new_truck_route <= instance->e - instance->sr) {
                                const double old_cost = std::max(sortie_stages[index_is_mid[rep_index]].truck_trip,
                                                                 sortie_stages[index_is_mid[rep_index]].drone_trip);
                                const double new_cost = new_drone_trip + instance->sl + instance->sr
                                    + std::max(affected_new_truck_route,
                                               sortie_stages[index_is_mid[rep_index]].drone_trip);
                                if (new_cost < old_cost) {
                                    loopable = true;
                                    // execute this move immediately
                                    for (const int d : served_by_drone) {
                                        if (sortie_stages[d].start_index >= index && sortie_stages[d].end_index <
                                            rep_index) {
                                            sortie_stages[d].start_index++;
                                            sortie_stages[d].end_index++;
                                        }
                                    }
                                    sortie_stages[index_is_mid[rep_index]].start_index++;
                                    served_by_drone.push_back(truck_route[rep_index]);
                                    sortie_stages[index_is_mid[rep_index]].truck_trip = affected_new_truck_route;
                                    sortie_stages[truck_route[rep_index]] = {index, index + 1, new_drone_trip, 0.0};
                                    truck_route.erase(truck_route.begin() + rep_index);
                                    truck_route.insert(truck_route.begin() + index + 1, truck_route[index]);
                                    goto tag;
                                }
                            }
                        }
                    }
                    else {
                        // khong lien quan den ai.
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][truck_route[
                                rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            const double old_cost = instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                + instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]];
                            const double new_cost = new_drone_trip + instance->sl + instance->sr
                                + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                            if (new_cost < old_cost) {
                                loopable = true;
                                for (const int d : served_by_drone) {
                                    if (sortie_stages[d].start_index >= index && sortie_stages[d].end_index <
                                        rep_index) {
                                        sortie_stages[d].start_index++;
                                        sortie_stages[d].end_index++;
                                    }
                                }

                                served_by_drone.push_back(truck_route[rep_index]);
                                sortie_stages[truck_route[rep_index]] = {
                                    index, index + 1, new_drone_trip, 0
                                };
                                truck_route.insert(truck_route.begin() + index + 1, truck_route[index]);
                                truck_route.erase(truck_route.begin() + rep_index + 1);
                                goto tag;
                            }
                        }
                    }
                }
                for (int rep_index = index - 1; rep_index >= 1; rep_index--) {
                    if (truck_route[rep_index] == index || countMap[truck_route[rep_index]] > 1 || instance->heavy_bool[
                        truck_route[rep_index]]) {
                        continue;
                    }
                    if (index_is_start[rep_index] > -1 && index_is_end[rep_index] > -1) {
                        if (sortie_stages[index_is_start[rep_index]].end_index - sortie_stages[index_is_start[
                            rep_index]].start_index == 1) {
                            continue;
                        }
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][truck_route[
                                rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            if (const double new_r_of_drone_trip = instance->tau_prime[truck_route[sortie_stages[
                                        index_is_end[rep_index]].start_index]][index_is_end[rep_index]]
                                    + instance->tau_prime[index_is_end[rep_index]][truck_route[rep_index + 1]];
                                new_r_of_drone_trip <= instance->e - instance->sr) {
                                if (const double new_r_of_truck_trip = sortie_stages[index_is_end[rep_index]].truck_trip
                                        - instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                        + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                                    new_r_of_drone_trip <= instance->e - instance->sr) {
                                    // check the right sortie (l_of)
                                    if (const double new_l_of_drone_trip = instance->tau_prime[truck_route[rep_index +
                                                1]][index_is_start[rep_index]]
                                            + instance->tau_prime[index_is_start[rep_index]][truck_route[sortie_stages[
                                                index_is_start[rep_index]].end_index]];
                                        new_l_of_drone_trip <= instance->e - instance->sr) {
                                        const double new_l_of_truck_trip = sortie_stages[index_is_start[rep_index]].
                                            truck_trip
                                            - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]];
                                        const double old_cost = std::max(
                                                sortie_stages[index_is_start[rep_index]].truck_trip,
                                                sortie_stages[index_is_start[rep_index]].drone_trip)
                                            + std::max(sortie_stages[index_is_end[rep_index]].truck_trip,
                                                       sortie_stages[index_is_end[rep_index]].truck_trip);
                                        const double new_cost = new_drone_trip + instance->sl + instance->sr
                                            + std::max(new_l_of_truck_trip, new_l_of_drone_trip)
                                            + std::max(new_r_of_truck_trip, new_r_of_drone_trip);
                                        if (new_cost < old_cost) {
                                            loopable = true;
                                            for (const int d : served_by_drone) {
                                                if (sortie_stages[d].start_index > rep_index && sortie_stages[d].
                                                    end_index <= index) {
                                                    sortie_stages[d].start_index--;
                                                    sortie_stages[d].end_index--;
                                                }
                                            }
                                            sortie_stages[index_is_start[rep_index]].end_index--;
                                            served_by_drone.push_back(truck_route[rep_index]);
                                            sortie_stages[truck_route[rep_index]] = {
                                                index - 1, index, new_drone_trip, 0.0
                                            };
                                            sortie_stages[index_is_start[rep_index]].truck_trip = new_l_of_truck_trip;
                                            sortie_stages[index_is_start[rep_index]].drone_trip = new_l_of_drone_trip;
                                            sortie_stages[index_is_end[rep_index]].truck_trip = new_r_of_truck_trip;
                                            sortie_stages[index_is_end[rep_index]].drone_trip = new_r_of_drone_trip;

                                            truck_route.insert(truck_route.begin() + index, truck_route[index]);
                                            truck_route.erase(truck_route.begin() + rep_index);
                                            goto tag;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else if (index_is_start[rep_index] > -1 && index_is_end[rep_index] == -1) {
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][
                                truck_route[
                                    rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            if (const double affected_new_truck_trip = sortie_stages[index_is_start[
                                        rep_index]].
                                    truck_trip
                                    - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]]
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                                affected_new_truck_trip <= instance->e - instance->sr) {
                                if (const double affected_new_drone_trip = instance->tau_prime[truck_route[
                                            rep_index -
                                            1]][index_is_start[rep_index]]
                                        + instance->tau_prime[index_is_start[rep_index]][truck_route[
                                            sortie_stages[
                                                index_is_start[rep_index]].end_index]];
                                    affected_new_drone_trip <= instance->e - instance->sr) {
                                    const double old_cost = std::max(
                                        sortie_stages[index_is_start[rep_index]].truck_trip,
                                        sortie_stages[index_is_start[rep_index]].drone_trip);
                                    const double new_cost = std::max(
                                            affected_new_truck_trip, affected_new_drone_trip) +
                                        new_drone_trip + instance->sl + instance->sr;
                                    if (new_cost < old_cost) {
                                        loopable = true;
                                        for (const int d : served_by_drone) {
                                            if (sortie_stages[d].start_index >= rep_index && sortie_stages[
                                                    d].end_index
                                                <= index) {
                                                sortie_stages[d].start_index--;
                                                sortie_stages[d].end_index--;
                                            }
                                        }

                                        served_by_drone.push_back(truck_route[rep_index]);
                                        sortie_stages[truck_route[rep_index]] = {
                                            index - 1, index, new_drone_trip, 0.0
                                        };
                                        sortie_stages[index_is_start[rep_index]].truck_trip =
                                            affected_new_truck_trip;
                                        sortie_stages[index_is_start[rep_index]].drone_trip =
                                            affected_new_drone_trip;
                                        truck_route.insert(truck_route.begin() + index, truck_route[index]);
                                        truck_route.erase(truck_route.begin() + rep_index);
                                        goto tag;
                                    }
                                }
                            }
                        }
                    }
                    else if (index_is_end[rep_index] > -1 && index_is_start[rep_index] == -1) {
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][
                                truck_route[
                                    rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            if (const double affected_new_truck_trip = sortie_stages[index_is_end[
                                        rep_index]].truck_trip
                                    - instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                                affected_new_truck_trip <= instance->e - instance->sr) {
                                if (const double affected_new_drone_trip = instance->tau_prime[truck_route[
                                            sortie_stages
                                            [index_is_end[rep_index]].start_index]][index_is_end[rep_index]]
                                        + instance->tau_prime[index_is_end[rep_index]][truck_route[rep_index
                                            + 1]];
                                    affected_new_drone_trip <= instance->e - instance->sr) {
                                    const double old_cost = std::max(
                                        sortie_stages[index_is_end[rep_index]].truck_trip,
                                        sortie_stages[index_is_end[rep_index]].drone_trip);
                                    const double new_cost = std::max(
                                            affected_new_truck_trip, affected_new_drone_trip)
                                        + new_drone_trip + instance->sl + instance->sr;
                                    if (new_cost < old_cost) {
                                        loopable = true;
                                        for (const int d : served_by_drone) {
                                            if (sortie_stages[d].start_index > rep_index && sortie_stages[d]
                                                .end_index
                                                <= index) {
                                                sortie_stages[d].start_index--;
                                                sortie_stages[d].end_index--;
                                            }
                                        }
                                        served_by_drone.push_back(truck_route[rep_index]);
                                        sortie_stages[truck_route[rep_index]] = {
                                            index - 1, index, new_drone_trip, 0.0
                                        };
                                        sortie_stages[index_is_end[rep_index]].truck_trip =
                                            affected_new_truck_trip;
                                        sortie_stages[index_is_end[rep_index]].drone_trip =
                                            affected_new_drone_trip;
                                        truck_route.insert(truck_route.begin() + index, truck_route[index]);
                                        truck_route.erase(truck_route.begin() + rep_index);
                                        goto tag;
                                    }
                                }
                            }
                        }
                    }
                    else if (index_is_mid[rep_index] > -1) {
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[index]][
                                truck_route[
                                    rep_index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            if (const double affected_new_truck_route = sortie_stages[index_is_mid[
                                        rep_index]].
                                    truck_trip
                                    - instance->tau[truck_route[rep_index - 1]][truck_route[rep_index]]
                                    - instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]]
                                    + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                                affected_new_truck_route <= instance->e - instance->sr) {
                                const double old_cost = std::max(
                                    sortie_stages[index_is_mid[rep_index]].truck_trip,
                                    sortie_stages[index_is_mid[rep_index]].drone_trip);
                                const double new_cost = new_drone_trip + instance->sl + instance->sr
                                    + std::max(affected_new_truck_route,
                                               sortie_stages[index_is_mid[rep_index]].drone_trip);
                                if (new_cost < old_cost) {
                                    loopable = true;
                                    for (const int d : served_by_drone) {
                                        if (sortie_stages[d].start_index > rep_index && sortie_stages[d].
                                            end_index <=
                                            index) {
                                            sortie_stages[d].start_index--;
                                            sortie_stages[d].end_index--;
                                        }
                                    }
                                    sortie_stages[index_is_mid[rep_index]].end_index--;
                                    served_by_drone.push_back(truck_route[rep_index]);
                                    sortie_stages[truck_route[rep_index]] = {
                                        index - 1, index, new_drone_trip, 0.0
                                    };
                                    sortie_stages[index_is_mid[rep_index]].truck_trip =
                                        affected_new_truck_route;
                                    truck_route.insert(truck_route.begin() + index, truck_route[index]);
                                    truck_route.erase(truck_route.begin() + rep_index);
                                    goto tag;
                                }
                            }
                        }
                    }
                    else {
                        if (const double new_drone_trip = 2.0 * instance->tau_prime[truck_route[rep_index]][
                                truck_route[
                                    index]];
                            new_drone_trip <= instance->e - instance->sr) {
                            const double old_cost = instance->tau[truck_route[rep_index - 1]][truck_route[
                                    rep_index]]
                                + instance->tau[truck_route[rep_index]][truck_route[rep_index + 1]];
                            const double new_cost = new_drone_trip + instance->sl + instance->sr
                                + instance->tau[truck_route[rep_index - 1]][truck_route[rep_index + 1]];
                            if (new_cost < old_cost) {
                                loopable = true;
                                for (const int d : served_by_drone) {
                                    if (sortie_stages[d].start_index > rep_index && sortie_stages[d].
                                        end_index <=
                                        index) {
                                        sortie_stages[d].start_index--;
                                        sortie_stages[d].end_index--;
                                    }
                                }
                                served_by_drone.push_back(truck_route[rep_index]);
                                sortie_stages[truck_route[rep_index]] = {
                                    index - 1, index, new_drone_trip, 0.0
                                };
                                truck_route.insert(truck_route.begin() + index, truck_route[index]);
                                truck_route.erase(truck_route.begin() + rep_index);
                                goto tag;
                            }
                        }
                    }
                }
            }
        }
    }
}


void SolutionL::invalid_occurrence() {
    std::vector<std::vector<int>> occurrences(instance->num_node + 1);
    for (int index = 1; index < truck_route.size() - 1; ++index) {
        occurrences[truck_route[index]].push_back(index);
    }
    for (int i = 1; i < instance->num_node; ++i) {
        if (occurrences[i].size() > 1) {
            for (int index = 1; index < occurrences[i].size(); index++) {
                bool invalid = true;
                // if this index is in middle of sortie or not in any sortie. => invalid
                for (const int d : served_by_drone) {
                    if (sortie_stages[d].start_index == occurrences[i][index] || occurrences[i][index] ==
                        sortie_stages[
                            d].end_index) {
                        // in middle.
                        invalid = false;
                        break;
                    }
                }
                if (invalid) {
                    std::cout << "SolutionL before throw: " << std::endl;
                    print_solution();
                    throw std::runtime_error(
                        "Invalid occurrence of node " + std::to_string(i) + " at index " + std::to_string(index));
                }
            }
        }
    }
}

void SolutionL::sortie_invalid_occurrence() {
    std::cout << "Solution checking: Sortie invalid occurrence" << std::endl;
    print_solution();
    for (const int d : served_by_drone) {
        std::vector<int> count(instance->num_node, 0);
        for (int i = sortie_stages[d].start_index; i <= sortie_stages[d].end_index; ++i) {
            count[truck_route[i]]++;
        }
        for (int i = 0; i < instance->num_node; ++i) {
            if (count[i] > 1) {
                std::cout << "Solution before throw: " << std::endl;
                print_solution();
                throw std::runtime_error(
                    "Invalid occurrence of node " + std::to_string(i) + " in sortie " + std::to_string(d));
            }
        }
    }
    std::cout << "Passed invalid sortie." << std::endl;
}

void SolutionL::energy_constraint_check() {
    std::cout << "Starting non-reoccurrence feasibility check: " << std::endl;
    print_solution();
    if (truck_route.size() > instance->num_node + 1) {
        throw std::runtime_error("bugged truck route: too many nodes");
    }
    if (truck_route[0] != 0 || truck_route[truck_route.size() - 1] != instance->num_node) {
        throw std::runtime_error("Truck route start/end bugged");
    }
    check_duplicates(served_by_drone);
    std::vector<int> to_insert;
    to_insert.reserve(instance->C.size()); // Reserve space to avoid multiple allocations
    for (int i : instance->C) {
        if (!visited[i]) {
            to_insert.push_back(i);
        }
    }
    for (const int i : to_insert) {
        if (existL(truck_route, i)) {
            throw std::runtime_error("Insert " + std::to_string(i) + " is already in the truck route.");
        }
        if (existL(served_by_drone, i)) {
            throw std::runtime_error("Insert " + std::to_string(i) + " is already in the drone route.");
        }
    }
    for (int i = 0; i <= instance->num_node; i++) {
        // check for missing node.
        if (!existL(truck_route, i) && !existL(served_by_drone, i) && visited[i]) {
            throw std::runtime_error("Node " + std::to_string(i) + " is not in truck route or served by drone");
        }
        if (existL(truck_route, i) && existL(served_by_drone, i)) {
            throw std::runtime_error("Node " + std::to_string(i) + " is in both truck and drone route");
        }
    }
    for (const int& d : served_by_drone) {
        if (sortie_stages[d].start_index >= sortie_stages[d].end_index) {
            throw std::runtime_error("Sortie " + std::to_string(d) + " has start index " + std::to_string(
                sortie_stages[d].start_index) + " geq than end index " + std::to_string(sortie_stages[d].end_index));
        }
        // assert(sortie_stages[d].start_index < sortie_stages[d].end_index);
        // assert(truck_route[sortie_stages[d].start_index] != truck_route[sortie_stages[d].end_index]);
        assert(sortie_stages[d].drone_trip <= instance->e - instance->sr);
        assert(sortie_stages[d].truck_trip <= instance->e - instance->sr);
        assert(sortie_stages[d].truck_trip >= -0.00001);
        assert(sortie_stages[d].drone_trip > 0);
        assert(instance->heavy_bool[d] == false);
        double re_truck = 0;
        for (int start = sortie_stages[d].start_index; start <= sortie_stages[d].end_index - 1; start++) {
            re_truck += instance->tau[truck_route[start]][truck_route[start + 1]];
        }
        const double re_drone = instance->tau_prime[truck_route[sortie_stages[d].start_index]][d]
            + instance->tau_prime[d][truck_route[sortie_stages[d].end_index]];
        if (fabs(re_truck - sortie_stages[d].truck_trip) > 1e-4) {
            throw std::runtime_error(
                "re_truck = " + std::to_string(re_truck) + " while truck_trip = " + std::to_string(
                    sortie_stages[d].truck_trip));
        }
        if (fabs(re_drone - sortie_stages[d].drone_trip) > 1e-4) {
            throw std::runtime_error(
                "re_drone = " + std::to_string(re_truck) + " while drone_trip = " + std::to_string(
                    sortie_stages[d].drone_trip));
        }
    }

    for (const int d : served_by_drone) {
        for (const int d1 : served_by_drone) {
            if (d != d1) {
                if (sortie_stages[d1].start_index > sortie_stages[d].start_index && sortie_stages[d1].start_index <
                    sortie_stages[d].end_index) {
                    throw std::runtime_error("Crossing! Printing solution before exiting...");
                }
                if (sortie_stages[d1].end_index > sortie_stages[d].start_index && sortie_stages[d1].end_index <
                    sortie_stages[d].end_index) {
                    throw std::runtime_error("Crossing! Printing solution before exiting...");
                }
            }
        }
    }
    std::cout << "Check completed. Feasible sorties. (without node reoccurrence check)" << std::endl;
}

// void SolutionL::compare_results(const std::string &our_result_path, const std::string &agatz_result_path, const std::string &final_file) {
//     // Read rival solutions into a map
//     std::unordered_map<std::string, std::string> rivalSolutions;
//     std::ifstream rivalFileStream(agatz_result_path);
//     std::string rivalLine, instance, solution;
//
//     // Parse the rival file
//     while (getline(rivalFileStream, rivalLine)) {
//         std::istringstream rivalStream(rivalLine);
//         getline(rivalStream, instance, ','); // Get instance name
//         solution = rivalLine.substr(instance.length() + 1); // Get solution part
//         rivalSolutions[instance] = solution;
//     }
//     rivalFileStream.close();
//
//     // Read my solution file, append rival solutions, and overwrite the file
//     std::ifstream myFileStream(our_result_path);
//     std::string myLine;
//     std::vector<std::string> updatedLines;
//
//     while (getline(myFileStream, myLine)) {
//         std::istringstream myStream(myLine);
//         getline(myStream, instance, ','); // Get instance name from my solution file
//         getline(myStream, instance, ','); // Get instance name from my solution file
//
//         if (rivalSolutions.find(instance) != rivalSolutions.end()) {
//             myLine += "," + rivalSolutions[instance]; // Append rival solution if instance matches
//         }
//         updatedLines.push_back(myLine); // Store updated line
//     }
//     myFileStream.close();
//
//     // Write updated content back to the file
//     std::ofstream myOutputFile(final_file);
//     for (const auto& updatedLine : updatedLines) {
//         myOutputFile << updatedLine << std::endl;
//     }
//     myOutputFile.close();
// }

void SolutionL::loop_search_2() {
    bool loopable = true;
tag:
    while (loopable) {
        loopable = false;
        std::unordered_map<int, int> countMap;
        std::vector<int> duplicateIndices;

        // First pass: Count occurrences of each element
        for (const auto& element : truck_route) {
            countMap[element]++;
        }
        for (int index = 0; index < truck_route.size(); index++) {
            if (countMap[truck_route[index]] > 1) {
                duplicateIndices.push_back(index);
            }
        }
        std::vector<int> index_is_start(truck_route.size(), -1);
        std::vector<int> index_is_mid(truck_route.size(), -1);
        std::cout << truck_route.size() << std::endl;
        std::vector<int> index_is_end(truck_route.size(), -1);

        for (const int d : served_by_drone) {
            for (int index = sortie_stages[d].start_index; index <= sortie_stages[d].end_index; index++) {
                if (index == sortie_stages[d].start_index) {
                    index_is_start[index] = d;
                    continue;
                }
                if (index == sortie_stages[d].end_index) {
                    index_is_end[index] = d;
                    continue;
                }
                index_is_mid[index] = d;
            }
        }

        auto loop_spaces = find_loop_spaces();

        // sort the loop space ascending order.
        std::ranges::sort(loop_spaces, [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
            return a.first < b.first;
        });
        for (int i = 0; i < loop_spaces.size(); i++) {
            // in each loop_spaces.
            // span right: to the next sorties.
            int max_rendezvous_index = truck_route.size() - 1;
            int st_start = -1;
            int next_right_st = -1;
            for (const int d : served_by_drone) {
                if (sortie_stages[d].end_index > loop_spaces[i].second && sortie_stages[d].end_index <=
                    max_rendezvous_index) {
                    max_rendezvous_index = sortie_stages[d].end_index - 1;
                    st_start = sortie_stages[d].start_index;
                    next_right_st = d;
                }
            }
            if (max_rendezvous_index == truck_route.size() - 1) {
                max_rendezvous_index -=1;
            }
            if (st_start == -1) {
                st_start = max_rendezvous_index - 1;
            }
            // in this.
            // tu start of loop_space -> max_rendezvous_index
            const int earliest_start = std::max(1, loop_spaces[i].first);
            for (int start_index = earliest_start; start_index <= st_start; start_index++) {
                double original_truck_trip = 0;
                for (int rendezvous_index = start_index + 1; rendezvous_index <= max_rendezvous_index; rendezvous_index
                     ++) {
                    original_truck_trip += instance->tau[truck_route[rendezvous_index - 1]][truck_route[
                        rendezvous_index]];
                    if (existL(duplicateIndices, rendezvous_index)) {
                        continue;
                    }
                    if (const double truck_trip = original_truck_trip - instance->tau[truck_route[rendezvous_index - 1]]
                            [truck_route[rendezvous_index]]
                            + instance->tau[truck_route[rendezvous_index - 1]][truck_route[start_index]]; truck_trip <=
                        instance->e - instance->sr) {
                        // calculate new drone?
                        if (const double drone_trip = 2.0 * instance->tau_prime[truck_route[start_index]][truck_route[
                            rendezvous_index]]; drone_trip <= instance->e - instance->sr) {
                            // oke.
                            // if rendezvous hasn't touched
                            if (rendezvous_index < st_start) {
                                // then
                                if (const double new_cost = std::max(truck_trip, drone_trip) + instance->tau[truck_route
                                        [rendezvous_index - 1]][truck_route[start_index]];
                                    new_cost < original_truck_trip && !instance->heavy_bool[truck_route[rendezvous_index]]) {
                                    // execute immediately.
                                    // std::cout << "pre-sol: " << std::endl;
                                    // sortie_feasibility_check();
                                    // std::cout << "index/"
                                    loopable = true;
                                    served_by_drone.push_back(truck_route[rendezvous_index]);
                                    sortie_stages[truck_route[rendezvous_index]] = {
                                        start_index, rendezvous_index, drone_trip, truck_trip
                                    };

                                    truck_route.erase(truck_route.begin() + rendezvous_index);
                                    truck_route.insert(truck_route.begin() + rendezvous_index,
                                                       truck_route[start_index]);
                                    goto tag;
                                }
                            }
                            else {
                                // // span into the next sortie.
                                // // then we have to calculate the next sortie's feasibility.
                                // if (const double new_truck_trip = sortie_stages[next_right_st].truck_trip
                                //     - instance->tau[truck_route[st_start]][truck_route[st_start+1]]
                                //     + instance->tau[truck_route[start_index]][truck_route[st_start+1]];
                                //     new_truck_trip <= instance->e - instance->sr) {
                                //     if (const double new_drone_trip = instance->tau_prime[truck_route[start_index]][next_right_st]
                                //         + instance->tau_prime[next_right_st][truck_route[max_rendezvous_index+1]];
                                //         new_drone_trip <= instance->e - instance->sr) {
                                //         const double old_cost = original_truck_trip + std::max(sortie_stages[next_right_st].truck_trip, sortie_stages[next_right_st].drone_trip);
                                //         const double new_cost = std::max(truck_trip, drone_trip) + instance->sl + instance->sr
                                //         + std::max(new_truck_trip, new_drone_trip);
                                //         if (new_cost < old_cost) {
                                //             // std::cout << "pre-sol: " << std::endl;
                                //             // sortie_feasibility_check();
                                //             loopable = true;
                                //             sortie_stages[next_right_st].start_index = rendezvous_index;
                                //             sortie_stages[next_right_st].truck_trip = new_truck_trip;
                                //             sortie_stages[next_right_st].drone_trip = new_drone_trip;
                                //             served_by_drone.push_back(truck_route[rendezvous_index]);
                                //             sortie_stages[truck_route[rendezvous_index]] = {start_index, rendezvous_index, drone_trip, truck_trip};
                                //             truck_route.erase(truck_route.begin() + rendezvous_index);
                                //             truck_route.insert(truck_route.begin() + rendezvous_index,
                                //                                truck_route[start_index]);
                                //             sortie_feasibility_check();
                                //             goto tag;
                                //         }
                                //     }
                                // }
                            }
                        }
                    }
                    // truck_trip still viable
                }
            }
        }

    }
}