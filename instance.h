//
// Created by cuong on 18/01/2024.
//
#pragma once
#ifndef INSTANCE_H
#define INSTANCE_H
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>

class Instance {
public:
    std::string folder_path;
    int num_node;
    std::vector<std::vector<double> > tau;
    std::vector<std::vector<double> > tau_prime;
    double drone_speed;
    std::vector<int> C;
    std::vector<int> c_prime;
    std::vector<int> heavy;
    std::vector<bool> c_prime_bool;
    std::vector<bool> heavy_bool;
    std::string agatz_type;

    std::vector<int> revisit_rank;
    double e = 99999;
    double dtl;
    double sl = 0;
    double sr = 0;
    bool restricted_dtl = false;


    explicit Instance(std::string &folder_path) ;

    virtual void read() = 0;

    virtual ~Instance() = default;
};

class MurrayInstance final: public Instance {
public:
    explicit MurrayInstance(std::string &folder_path) : Instance(folder_path) {}

    void read() override;
};

//
class NielsInstance final: public Instance {
public:
    explicit NielsInstance(std::string &folder_path) : Instance(folder_path) {}
    double truck_speed_factor;
    double drone_speed_factor;
    void read() override;
};

class PoikonenInstance final: public Instance {
public:
    explicit PoikonenInstance(std::string &folder_path) : Instance(folder_path) {}

    void read() override;
};

class InstanceFactory {
public:
    static std::shared_ptr<Instance> createInstance(const std::string &type, std::string &folder_path) {
        if (type == "M") {
            return std::make_shared<MurrayInstance>(folder_path);
        } else if (type == "N") {
            return std::make_shared<NielsInstance>(folder_path);
        } else if (type == "P") {
            return std::make_shared<PoikonenInstance>(folder_path);
        } else {
            std::cout << "Unsupported type of instance.";
            return nullptr; // Handle unsupported type
        }
    }
};


#endif //INSTANCE_H
