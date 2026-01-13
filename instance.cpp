//
// Created by cuong on 18/01/2024.
//
#include "instance.h"

#include <cmath>
#include <utility>

#include "instance.h"
#include <string>
#include <vector>
#include <algorithm>
#include <random>

inline bool exist(const std::vector<int>& vec, int element) {
    // Use std::find to search for the element in the vector
    return std::find(vec.begin(), vec.end(), element) != vec.end();
}

static std::vector<std::string> SplitStringWithDelimiter(const std::string& s, const std::string& delimiter) {
    std::vector<std::string> returnValue;
    std::string::size_type start = 0;
    std::string::size_type end = s.find(delimiter);

    while (end != std::string::npos) {
        returnValue.push_back(s.substr(start, end - start));
        start = end + 1;
        end = s.find(delimiter, start);
    }

    returnValue.push_back(s.substr(start));
    return returnValue;
}

inline double manhattanDistance(double x1, double y1, double x2, double y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

inline double euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

Instance::Instance(std::string& fp) {
    folder_path = fp;
    if (folder_path.find("maxradius") != std::string::npos) {
        agatz_type = "maxradius";
    }
    else if (folder_path.find("novisit") != std::string::npos) {
        agatz_type = "novisit";
    }
    else {
        agatz_type = "base";
    }
}

void MurrayInstance::read() {
    std::cout << "Input is specified as type M: Murray instance.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-" << std::endl;

    std::string c_prime_path = folder_path + "/Cprime.csv";
    std::string nodes_path = folder_path + "/nodes.csv";
    std::string tau_path = folder_path + "/tau.csv";
    std::string tau_prime_path = folder_path + "/tauprime.csv";

    std::ifstream c_ifs(c_prime_path);
    std::ifstream n_ifs(nodes_path);
    std::ifstream t_ifs(tau_path);
    std::ifstream t_prime_ifs(tau_prime_path);

    std::string str;
    getline(c_ifs, str);
    auto c_split_str = SplitStringWithDelimiter(str, ",");
    for (auto& s : c_split_str) {
        c_prime.push_back(stoi(s));
    }
    std::vector<double> X_coord;
    std::vector<double> Y_coord;
    std::cout << std::endl;
    int n = 0;
    while (getline(n_ifs, str)) {
        if (n == 0) {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            drone_speed = stod(n_split_str[n_split_str.size() - 1]);
            std::cout << "drone speed: " << drone_speed << " miles/minute" << std::endl;
            X_coord.push_back(stod(n_split_str[1]));
            Y_coord.push_back(stod(n_split_str[2]));
        }
        else {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            int x = stoi(n_split_str[n_split_str.size() - 1]);
            if (x == 1) {
                heavy.push_back(stoi(n_split_str[0]));
            }
            X_coord.push_back(stod(n_split_str[1]));
            Y_coord.push_back(stod(n_split_str[2]));
        }
        n++;
    }
    std::cout << "uav non-eligible customer: ";
    for (auto x : heavy) {
        std::cout << x << " ";
    }
    heavy_bool.resize(n);
    std::cout << std::endl;
    getline(n_ifs, str);
    n -= 1;
    //n = 20;

    num_node = n;
    tau.resize(num_node + 1);
    tau_prime.resize(num_node + 1);
    for (int i = 0; i < tau.size(); i++) {
        tau[i].resize(num_node + 1);
        tau_prime[i].resize(num_node + 1);
    }

    double d;
    char c;

    for (int i = 0; i < n + 1; i++) {
        getline(t_prime_ifs, str);
        std::istringstream iss(str);
        for (int j = 0; j < n + 1; j++) {
            iss >> d >> c;
            tau_prime[i][j] = d;
        }
    }


    for (int i = 0; i < n + 1; i++) {
        getline(t_ifs, str);
        std::istringstream iss(str);
        for (int j = 0; j < n + 1; j++) {
            iss >> d >> c;

            tau[i][j] = d;
        }
    }
    std::cout << "Printing tau:" << std::endl;
    for (int i = 0; i < n + 1; i++) {
        for (int j = 0; j < n + 1; j++) {
            std::cout << tau[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "Printing tau_prime:" << std::endl;
    for (int i = 0; i < n + 1; i++) {
        for (int j = 0; j < n + 1; j++) {
            std::cout << tau_prime[i][j] << " ";
        }
        std::cout << std::endl;
    }
    c_prime_bool.resize(n, false);
    heavy_bool.resize(n, false);

    for (int i : c_prime) {
        c_prime_bool[i] = true;
    }
    for (int i : heavy) {
        heavy_bool[i] = true;
    }
}

void NielsInstance::read() {
    if (std::ifstream fin(folder_path); !fin.is_open()) {
        throw std::runtime_error("Could not open file");
    }
    else {
        // agatz_type = "novisit";
        std::string line;
        if (agatz_type == "maxradius") {
            std::getline(fin, line);
            auto split_radius = SplitStringWithDelimiter(line, " ");
            if (split_radius[1] == "Infinity") {
                dtl = 1e9;
            }
            else {
                dtl = stod(split_radius[1]);
                std::getline(fin, line);
                restricted_dtl = true;
            }
        }
        else if (agatz_type == "novisit") {
            std::getline(fin, line);
            auto split_radius = SplitStringWithDelimiter(line, " ");
            if (split_radius[1] == "Infinity") {
                dtl = 1e9;
            }
            else {
                dtl = stod(split_radius[1]);
                restricted_dtl = true;
            }
            while (std::getline(fin, line)) {
                if (line.empty()) {
                    break;
                }
                // else.
                auto split_novisit = SplitStringWithDelimiter(line, " ");
                heavy.push_back(std::stoi(split_novisit[1]));
            }
        }
        std::getline(fin, line);
        std::getline(fin, line);
        truck_speed_factor = std::stod(line);
        std::getline(fin, line);
        std::getline(fin, line);
        drone_speed_factor = std::stod(line);
        std::getline(fin, line);
        std::getline(fin, line);
        num_node = std::stoi(line);
        if (num_node < 5 || num_node > 17) {
            // throw std::runtime_error("qua it/nhieu nodes");
            // exit(0);
        }

        std::vector<std::pair<double, double>> coord(num_node + 1);
        std::getline(fin, line);
        std::getline(fin, line);
        auto s = SplitStringWithDelimiter(line, " ");
        coord[0] = std::make_pair(std::stod(s[0]), std::stod(s[1]));
        std::getline(fin, line);
        heavy_bool.resize(num_node);
        for (int i = 1; i < num_node; i++) {
            std::getline(fin, line);
            s = SplitStringWithDelimiter(line, " ");
            coord[i] = std::make_pair(std::stod(s[0]), std::stod(s[1]));
            if (!exist(heavy, i)) {
                c_prime.push_back(i);
            }
            else {
                heavy_bool[i] = true;
            }
            C.push_back(i);
        }
        // std::cout << "c_prime:";
        // for (auto x : c_prime) {
        //     std::cout << x << " ";
        // }
        // std::cout << std::endl;
        coord[num_node] = coord[0];
        // std::cout << "Printing coordinations: " << std::endl;
        // for (auto c : coord) {
        //     std::cout << c.first << " " << c.second << std::endl;
        // }
        // std::cout << "End of printing coord." << std::endl;
        tau.resize(num_node + 1);
        tau_prime.resize(num_node + 1);
        for (int i = 0; i < tau.size(); i++) {
            tau[i].resize(num_node + 1);
            tau_prime[i].resize(num_node + 1);
        }

        for (int i = 0; i < tau.size(); i++) {
            for (int j = 0; j < tau[i].size(); j++) {
                if (i == j) {
                    tau[i][j] = 0;
                    tau_prime[i][j] = 0;
                }
                else {
                    // tau[i][j] = euclideanDistance(coord[i].first, coord[i].second, coord[j].first, coord[j].second) *
                    //     truck_speed_factor;
                    tau[i][j] = euclideanDistance(coord[i].first, coord[i].second, coord[j].first, coord[j].second) *
                        truck_speed_factor;
                    tau_prime[i][j] = euclideanDistance(coord[i].first, coord[i].second, coord[j].first,
                                                        coord[j].second) *
                        drone_speed_factor;
                }
            }
        }
        for (int j = 0; j < tau.size(); j++) {
            tau[num_node][j] = 0;
            tau_prime[num_node][j] = 0;
        }
        for (int i = 0; i < tau.size(); i++) {
            tau[i][num_node] = tau[i][0];
        }

        // perturbation
        // if (folder_path.find("irace") != std::string::npos && folder_path.find("train") != std::string::npos) {
        //     std::random_device rd;  // Seed for the random number generator
        //     std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        //
        //     // Define the range [0.9, 1.1]
        //     std::uniform_real_distribution dis(0.9, 1.1);
        //
        //     // Generate a random number in the range
        //     double random_number = dis(gen);
        //     for (int i = 0; i < tau.size(); i++) {
        //         for (int j = 0; j < tau[i].size(); j++) {
        //             if (i != j) {
        //                 tau[i][j] = random_number * tau[i][j];
        //                 tau_prime[i][j] = random_number * tau_prime[i][j];
        //             }
        //         }
        //     }
        // }
    }
}

void PoikonenInstance::read() {
    std::cout << "Input is specified as type P: Poikonen instance.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-" << std::endl;

    std::string tau_path = folder_path + "/tauT.csv";
    std::string tau_prime_path = folder_path + "/tauD.csv";

    std::ifstream t_fin(tau_path);
    std::ifstream tp_fin(tau_prime_path);


    std::string line;
    while (std::getline(t_fin, line)) {
        // Read each line of the CSV file
        std::vector<double> row;
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            // Split each line into individual values
            try {
                double value = std::stod(token); // Convert string to integer
                row.push_back(value); // Store the value in the row vector
            }
            catch (const std::exception& e) {
                std::cerr << "Error converting string to double: " << e.what() << std::endl;
            }
        }
        tau.push_back(row); // Add the row vector to the 2D vector representing the matrix
    }
    t_fin.close(); // Close the file

    while (std::getline(tp_fin, line)) {
        // Read each line of the CSV file
        std::vector<double> row;
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            // Split each line into individual values
            try {
                double value = std::stod(token); // Convert string to integer
                row.push_back(value); // Store the value in the row vector
            }
            catch (const std::exception& e) {
                std::cerr << "Error converting string to double: " << e.what() << std::endl;
            }
        }
        tau_prime.push_back(row); // Add the row vector to the 2D vector representing the matrix
    }
    num_node = tau.size() - 1;
    for (int i = 1; i < tau.size() - 1; i++) {
        c_prime.push_back(i);
    }
    std::cout << "Printing tau: " << std::endl;
    for (int i = 0; i < tau.size(); i++) {
        for (int j = 0; j < tau[i].size(); j++) {
            std::cout << tau[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Printing tau_prime: " << std::endl;
    for (int i = 0; i < tau_prime.size(); i++) {
        for (int j = 0; j < tau_prime[i].size(); j++) {
            std::cout << tau_prime[i][j] << " ";
        }
        std::cout << std::endl;
    }
}





