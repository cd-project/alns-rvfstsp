#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <cmath>
#include <unordered_map>

#include "instance.h"
#include "no_loop/alns_ops.h"
#include "parameters.h"
#include "with_loop/alnsl_ops.h"

namespace fs = std::filesystem;
struct OurResult {
  double Best;
  double Gap;
  double Time;
  double Im;
};
static std::vector<std::string> SplitStringWithDelimiter(
  const std::string& s, const std::string& delimiter) {
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

inline double round_to_dec(double& x, const int dpoint) {
  // Modify the value in place
  const double factor = std::pow(10.0, dpoint);
  x = std::round(x * factor) / factor;
  return x;
}

// server: 392       99996   10   40          90          87        92           39          48          35  72  80  32  74  13  92  20  51  86      576
// self 1: 482       99998   11   32          79          77        97           57          23          85  89  90  19  47  38  36  51  68  7
inline void get_summary_agatz() {
  std::ifstream agatz_file("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/large_agatz_ep_all.csv");
  std::ofstream large_agatz_summary("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/large_agatz_summary.csv", std::ios::app);
  int cnt = 0;
  std::string instance_name;
  double best = 1e9;
  double avg = 0;
  double avg_time = 0;
  double gap;
  std::string line;
  while (!agatz_file.eof()) {
    std::getline(agatz_file, line);
    auto split = SplitStringWithDelimiter(line, ",");
    if (cnt == 0) {
      instance_name = split[0];
    }
    if (split[0] == "uniform-79-n50") {
      std::cout << line << std::endl;
    }


    auto this_obj = std::stod(split[1]);
    if (this_obj < best) {
      best = this_obj;
    }
    avg += this_obj;
    avg_time += std::stod(split[2]);
    if (split[0] == "uniform-79-n50") {
      std::cout << cnt << " " << std::stod(split[2]) << std::endl;
      std::cout << "current avg sum: " << avg_time << std::endl;
    }
    cnt++;
    if (cnt != 0 && cnt != 10) {
      if (split[0] != instance_name) {
        throw std::runtime_error("Something wrong; check: " + split[0] + "/" + instance_name);
      }
    }
    if (cnt == 10) {
      if (split[0] == "uniform-79-n50") {
        std::cout << avg_time << std::endl;
      }
      avg /= 10;
      avg_time /= 10;
      gap = 100*((avg/best) - 1);
      large_agatz_summary << instance_name << "," << round_to_dec(best,2) << "," << round_to_dec(avg,2) << "," << round_to_dec(avg_time,0) << "," << round_to_dec(gap,2) << "\n";
      cnt = 0;
      best = 1e9;
      avg = 0;
      avg_time = 0;
      gap = 0;
    }
  }
}
inline void get_tex_large_agatz() {
  std::map<std::string, OurResult> our_result_250k;
  std::map<std::string, OurResult> our_result_unlimited;
  std::map<std::string, OurResult> agatz_result;
  std::ifstream agatz_file("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/large_agatz_summary.csv");
  std::string line;
  while (!agatz_file.eof()) {
    std::getline(agatz_file, line);
    auto split = SplitStringWithDelimiter(line, ",");
    auto instance_name = split[0];
    auto best = std::stod(split[1]);
    auto time = std::stod(split[3]);
    auto gap = std::stod(split[4]);
    // do it with the map.
    agatz_result[instance_name] = OurResult{best, gap, time, 0};
  }
  std::ifstream our_lim("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/agatz_large_uniform_new_param_250k_100k_setting78.csv");
  while (!our_lim.eof()) {
    std::getline(our_lim, line);
    auto split = SplitStringWithDelimiter(line, ",");
    auto instance_name = split[1];
    if (instance_name.find("n250") != std::string::npos) {
      continue;
    }
    auto best = std::stod(split[2]);
    auto gap = std::stod(split[6]);
    auto time = std::stod(split[4]) / 10.00;
    auto im = (1.00 - best / agatz_result[instance_name].Best) * 100.00;
    our_result_250k[instance_name] = OurResult{
      round_to_dec(best, 2), round_to_dec(gap, 2), round_to_dec(time, 0), round_to_dec(im, 2)
    };
  }

  std::ifstream our_unlimited(
    "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/agatz_large_uniform_new_param_unlimited_100k_setting78.csv");
  while (!our_unlimited.eof()) {
    std::getline(our_unlimited, line);
    auto split = SplitStringWithDelimiter(line, ",");
    auto instance_name = split[1];
    if (instance_name.find("n250") != std::string::npos) {
      continue;
    }
    auto best = std::stod(split[2]);
    auto gap = std::stod(split[6]);
    auto time = std::stod(split[4]) / 10.00;
    auto im = (1.00 - best / agatz_result[instance_name].Best) * 100.00;
    our_result_unlimited[instance_name] = OurResult{
      round_to_dec(best, 2), round_to_dec(gap, 2), round_to_dec(time, 0), round_to_dec(im, 2)
    };
  }
  std::map<int, double> agatz_time;
  std::map<int, double> agatz_gap;
  std::map<int, int> agatz_beaten_base;
  std::map<int, int> agatz_beaten_unlim;
  std::map<int, double> improve_obj_lim;
  std::map<int, double> improve_obj_unlim;
  std::map<int, double> max_loss_lim;
  std::map<int, double> max_loss_unlim;
  std::map<int, double> max_win_lim;
  std::map<int, double> max_win_unlim;
  std::map<int, double> min_loss_lim;
  std::map<int, double> min_loss_unlim;
  std::map<int, double> min_win_lim;
  std::map<int, double> min_win_unlim;

  std::map<int, double> lim_time;
  std::map<int, double> unlim_time;
  std::map<int, double> lim_gap;
  std::map<int, double> unlim_gap;
  for (auto x: {50, 75, 100, 175}) {
    min_loss_lim[x] = 1e9;
    min_loss_unlim[x] = 1e9;
    min_win_lim[x] = 1e9;
    min_win_unlim[x] = 1e9;
  }

  for (const auto &r:agatz_result) {
    auto instance_name = r.first;
    int n_node;
    if (instance_name.find("n50") != std::string::npos) {
      n_node = 50;
    } else if (instance_name.find("n75") != std::string::npos) {
      n_node = 75;
    } else if (instance_name.find("n100") != std::string::npos) {
      n_node = 100;
    } else if (instance_name.find("n175") != std::string::npos) {
      n_node = 175;
    }
    if (r.second.Best > our_result_250k[instance_name].Best) {
      agatz_beaten_base[n_node]++;
    }
    if (r.second.Best > our_result_unlimited[instance_name].Best) {
      agatz_beaten_unlim[n_node]++;
    }
    agatz_time[n_node] += r.second.Time;
    agatz_gap[n_node] += r.second.Gap;
    auto imp_lim = (1 - our_result_250k[instance_name].Best / r.second.Best) * 100;
    std::cout << instance_name << std::endl;
    std::cout << imp_lim << std::endl;
    improve_obj_lim[n_node] += imp_lim;
    if (imp_lim < 0) {
      if (abs(imp_lim) > max_loss_lim[n_node]) {
        max_loss_lim[n_node] = abs(imp_lim);
      }
      if (abs(imp_lim) < abs(min_loss_lim[n_node])) {
        min_loss_lim[n_node] = imp_lim;
      }
    }
    if (imp_lim > 0) {
      if (imp_lim > max_win_lim[n_node]) {
        max_win_lim[n_node] = imp_lim;
      }
      if (imp_lim < min_win_lim[n_node]) {
        min_win_lim[n_node] = imp_lim;
      }
    }
    auto imp_unlim = ((1 - our_result_unlimited[instance_name].Best / r.second.Best) * 100);
    improve_obj_unlim[n_node] += imp_unlim;
    if (imp_unlim < 0) {
      if (abs(imp_unlim) > max_loss_unlim[n_node]) {
        max_loss_unlim[n_node] = abs(imp_unlim);
      }
      if (abs(imp_unlim) < abs(min_loss_unlim[n_node])) {
        min_loss_unlim[n_node] = imp_unlim;
      }
    }
    if (imp_unlim > 0) {
      if (imp_unlim > max_win_unlim[n_node]) {
        max_win_unlim[n_node] = imp_unlim;
      }
      if (imp_unlim < min_win_unlim[n_node]) {
        min_win_unlim[n_node] = imp_unlim;
      }
    }
    lim_time[n_node] += our_result_250k[instance_name].Time;
    lim_gap[n_node] += our_result_250k[instance_name].Gap;
    unlim_time[n_node] += our_result_unlimited[instance_name].Time;
    unlim_gap[n_node] += our_result_unlimited[instance_name].Gap;
  }

  for (auto x:agatz_time) {
    agatz_time[x.first] /= 30;
    agatz_gap[x.first] /= 30;
    improve_obj_lim[x.first] /= 30;
    improve_obj_unlim[x.first] /= 30;
    lim_time[x.first] /= 30;
    lim_gap[x.first] /= 30;
    unlim_time[x.first] /= 30;
    unlim_gap[x.first] /= 30;
  }
  std::ofstream out("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/tex.csv", std::ios::app);
  // for (auto x: {50, 75, 100, 175}) {
  //   out << x << "," << agatz_time[x] << "," << agatz_gap[x] << "," << agatz_beaten_base[x] << "," << improve_obj_lim[x] << "," << min_loss_lim[x] << "," << max_loss_lim[x] <<
  //     ","  << min_win_lim[x] << "," << max_win_lim[x] << "," << lim_time[x] << "," << lim_gap[x] << "," << agatz_beaten_unlim[x] << "," << improve_obj_unlim[x] << "," << min_loss_unlim[x] << ","
  //   << max_loss_unlim[x] << "," << min_win_unlim[x] << "," << max_win_unlim[x] << "," << unlim_time[x] << "," << unlim_gap[x] << std::endl;
  // }
  for (auto x: {50, 75, 100, 175}) {
    out << x << " & " << 30  << " & " << round_to_dec(agatz_gap[x],2) << " & " << round_to_dec(agatz_time[x], 0) << " & " << agatz_beaten_unlim[x] << " & " << round_to_dec(improve_obj_unlim[x],2) << " & " << round_to_dec(max_win_unlim[x],2) << " & " << round_to_dec(max_loss_unlim[x],2) << " & " << round_to_dec(unlim_gap[x],2) << " & " << round_to_dec(unlim_time[x],0) << "\n";
  }




}
inline void cal_res() {

  std::map<std::string, OurResult> our_result_250k;
  std::map<std::string, OurResult> our_result_unlimited;
  std::map<std::string, OurResult> agatz_result;
  std::ifstream agatz_file("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/large_agatz_summary.csv");
  std::string line;
  while (!agatz_file.eof()) {
    std::getline(agatz_file, line);
    auto split = SplitStringWithDelimiter(line, ",");
    auto instance_name = split[0];
    auto best = std::stod(split[1]);
    auto time = std::stod(split[3]);
    auto gap = std::stod(split[4]);
    // do it with the map.
    agatz_result[instance_name] = OurResult{best, gap, time, 0};
  }
  std::ifstream our_lim("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/large_uniform_new_param_250k_100k_setting78.csv");
  while (!our_lim.eof()) {
    std::getline(our_lim, line);
    auto split = SplitStringWithDelimiter(line, ",");
    auto instance_name = split[1];
    auto best = std::stod(split[2]);
    auto gap = std::stod(split[6]);
    auto time = std::stod(split[4]) / 10.00;
    auto im = (1.00 - best / agatz_result[instance_name].Best) * 100.00;
    our_result_250k[instance_name] = OurResult{
      round_to_dec(best, 2), round_to_dec(gap, 2), round_to_dec(time, 0), round_to_dec(im, 2)
    };
  }

  std::ifstream our_unlimited(
    "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/large_uniform_new_param_unlimited_100k_setting78.csv");
  while (!our_unlimited.eof()) {
    std::getline(our_unlimited, line);
    auto split = SplitStringWithDelimiter(line, ",");
    auto instance_name = split[1];
    auto best = std::stod(split[2]);
    auto gap = std::stod(split[6]);
    auto time = std::stod(split[4]) / 10.00;
    auto im = (1.00 - best / agatz_result[instance_name].Best) * 100.00;
    our_result_unlimited[instance_name] = OurResult{
      round_to_dec(best, 2), round_to_dec(gap, 2), round_to_dec(time, 0), round_to_dec(im, 2)
    };
  }
  std::ofstream outf("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/tex.csv", std::ios::app);

  for (const auto& i : agatz_result) {
    auto instance_name = i.first;
    auto a_best = i.second.Best;
    auto a_gap = i.second.Gap;
    auto a_time = i.second.Time;

    auto lim_best = our_result_250k[instance_name].Best;
    auto lim_gap = our_result_250k[instance_name].Gap;
    auto lim_time = our_result_250k[instance_name].Time;
    auto lim_im = our_result_250k[instance_name].Im;

    auto unlim_best = our_result_unlimited[instance_name].Best;
    auto unlim_gap = our_result_unlimited[instance_name].Gap;
    auto unlim_time = our_result_unlimited[instance_name].Time;
    auto unlim_im = our_result_unlimited[instance_name].Im;

    outf << instance_name << " & " << a_best << " & " << a_gap << " & " << a_time << " & " << lim_best
      << " & " << lim_im << " & " << lim_gap << " & " << lim_time << " & " << "\\textbf{" << unlim_best
      << "} & " << unlim_im << " & " << unlim_gap << " & " << unlim_time << " \\" << "\\" << std::endl;
  }
}
inline void get_analysis() {
  const std::string baseline = "fstsp", rv = "revisit", rvl = "revisitloop";;
  const std::vector<std::string> distribution = {
    "c-c", "c-r", "c-rc",
    "e-c", "e-r", "e-rc",
    "r-c", "r-r", "r-rc",
  };
  const std::vector<std::string> speed_settings = {"1", "2", "3", "4"};
  const std::vector<std::string> endurance_settings = {"unlim", "20", "40"};

  std::map<std::string, double> objective;
  std::map<std::string, double> output_result;
  for (const auto& dist : distribution) {
    for (const auto &endurance: endurance_settings) {
      for (const auto& speed : speed_settings) {
        // for each of this, we have 10 instance. cal.
        // where to look for data?
        std::string baseline_name = "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/analysis_results/ana_" + baseline + "_dtl_" + endurance + "/" + "analysis_" + baseline + "_dronespeed_" + speed + "_dtl_" + endurance + ".csv";
        std::string rv_name = "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/analysis_results/ana_" + rv + "_dtl_" + endurance + "/" + "analysis_" + rv + "_dronespeed_" + speed + "_dtl_" + endurance + ".csv";
        std::string rvl_name = "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/analysis_results/ana_" + rvl + "_dtl_" + endurance + "/" + "analysis_" + rvl + "_dronespeed_" + speed + "_dtl_" + endurance + ".csv";
        // get all data of baseline into objective
        std::ifstream baseline_file(baseline_name);
        if (!baseline_file.is_open()) {
          std::cerr << "Error opening file " << baseline_name << std::endl;
          exit(0);
        }
        std::string line;
        while (std::getline(baseline_file, line)) {
          std::cout << line << std::endl;
          auto split = SplitStringWithDelimiter(line, ",");
          auto i_name = split[0];
          i_name += "_" + baseline + "_dronespeed_" + speed + "_dtl_" + endurance;
          objective[i_name] = std::stod(split[1]);
        }
        std::ifstream rv_file(rv_name);
        while (std::getline(rv_file, line)) {
          auto split = SplitStringWithDelimiter(line, ",");
          auto i_name = split[0];
          i_name += "_" + rv + "_dronespeed_" + speed + "_dtl_" + endurance;
          objective[i_name] = std::stod(split[1]);
        }
        std::ifstream rvl_file(rvl_name);
        while (std::getline(rvl_file, line)) {
          auto split = SplitStringWithDelimiter(line, ",");
          auto i_name = split[0];
          i_name += "_" + rvl + "_dronespeed_" + speed + "_dtl_" + endurance;
          objective[i_name] = std::stod(split[1]);
        }
      }
    }
  }
  std::vector<std::string> list_weird_files;
  std::ofstream out("/Users/cuong/CLionProjects/ALNS_RV_FSTSP/out_ana_latex.txt", std::ios::app);
  for (const auto &dist : distribution) {
    for (const auto &endurance: endurance_settings) {
      for (const auto &speed : speed_settings) {
      double sum_obj_baseline = 0, sum_obj_rv = 0, sum_obj_rvl = 0;
        for (int v = 1; v <= 10; v++) {
          std::string i_name = dist + "-" + std::to_string(v) + ".txt_";

          std::string base_i = i_name + baseline + "_dronespeed_" + speed + "_dtl_" + endurance;
          std::string rv_i = i_name + rv + "_dronespeed_" + speed + "_dtl_" + endurance;
          std::string rvl_i = i_name + rvl + "_dronespeed_" + speed + "_dtl_" + endurance;

          std::cout << objective[base_i] << "\n" << objective[rv_i] << "\n" << objective[rvl_i] << std::endl << "--------------";
          sum_obj_baseline += objective[base_i];
          sum_obj_rv += objective[rv_i];
          sum_obj_rvl += objective[rvl_i];
          if (objective[rv_i] < objective[base_i] && speed == "1") {
            std::cout << i_name << "\n";
            std::cout << dist << " " << endurance << " " << speed << std::endl;
            list_weird_files.push_back(i_name+"_"+endurance+"_"+speed+"_" + std::to_string(objective[rv_i]) + "_" + std::to_string(objective[base_i]) );
          }
        }
        double improve_rv_against_baseline = (sum_obj_baseline - sum_obj_rv)/sum_obj_baseline*100;
        double improve_rvl_against_baseline = (sum_obj_baseline - sum_obj_rvl)/sum_obj_baseline*100;

        output_result[dist + "_" + endurance + "_" + speed + "_rv"] = round_to_dec(improve_rv_against_baseline,2);
        output_result[dist + "_" + endurance + "_" + speed + "_rvl"] = round_to_dec(improve_rvl_against_baseline,2);
      }
    }
  }
  std::cout << "Number of weird output files: " << list_weird_files.size() << std::endl;
  for (const auto &file : list_weird_files) {
    std::cout << file << std::endl;
  }
  for (const auto &dist : distribution) {
    std::string to_write = "\\multirow{3}{*}{\\hspace*{1em}" + dist +
    "}\n";
    for (const auto &endurance: endurance_settings) {
       to_write += "$E = \\" + endurance + " $ &";
      for (int i = 0; i < speed_settings.size(); i++) {
          to_write += std::to_string(round_to_dec(output_result[dist + "_" + endurance + "_" + speed_settings[i] + "_rv"], 2)) + " & ";
      }
      for (int i = 0; i < speed_settings.size(); i++) {
        if (i != speed_settings.size() - 1) {
          to_write += std::to_string(round_to_dec(output_result[dist + "_" + endurance + "_" + speed_settings[i] + "_rvl"] ,2)) + " & ";
        } else {
          to_write += std::to_string(round_to_dec(output_result[dist + "_" + endurance + "_" + speed_settings[i] + "_rvl"], 2)) + " \'\\\'" + "\n";
        }
      }
    }
    to_write += "\n \\midrule \n";
    std::cout << to_write << std::endl;
    out << to_write;
  }
}

int main(int argc, char** argv) {
  // get_analysis();

  // get_tex_large_agatz();
  // return 0;
  std::string folder_path;
  std::string instance_name;
  const auto param = std::make_shared<Parameter>();

  if (argc > 2) {
    for (int i = 1; i < argc; ++i) {
      // Check if the current argument is "-i"
      if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
        instance_name = std::string(
          argv[i + 1]); // Store the instance name from the next argument
        i++; // Skip the next argument as it has been processed
      }
      if (strcmp(argv[i], "-temp") == 0 && i + 1 < argc) {
        param->temperature_control =
          std::stod(std::string(argv[i + 1])) / 1000.0;
        // Store the instance name from the next argument
        i++; // Skip the next argument as it has been processed
      }
      if (strcmp(argv[i], "-cr") == 0 && i + 1 < argc) {
        param->cooling_rate = std::stod(std::string(argv[i + 1])) / 100000.0;
        // Store the instance name from the next argument
        i++; // Skip the next argument as it has been processed
      }
      if (strcmp(argv[i], "-minR") == 0 && i + 1 < argc) {
        param->minimum_remove_rate = std::stod(std::string(argv[i + 1])) / 100.0;
        i++; // Skip the next argument as it has been processed
      }
      if (strcmp(argv[i], "-maxR") == 0 && i + 1 < argc) {
        param->maximum_remove_rate = std::stod(std::string(argv[i + 1])) / 100.0;

        i++; // Skip the next argument as it has been processed
      }
      if (strcmp(argv[i], "-dci") == 0 && i + 1 < argc) {
        param->decay_insert = std::stod(std::string(argv[i + 1])) / 100.0;
        i++;
      }
      if (strcmp(argv[i], "-dcr") == 0 && i + 1 < argc) {
        param->decay_remove = std::stod(std::string(argv[i + 1])) / 100.0;
        i++;
      }
      if (strcmp(argv[i], "-bestscore") == 0 && i + 1 < argc) {
        param->best_score = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-impscore") == 0 && i + 1 < argc) {
        param->improved_score = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-accscore") == 0 && i + 1 < argc) {
        param->accepted_score = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-rejscore") == 0 && i + 1 < argc) {
        param->rejected_score = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-i1") == 0 && i + 1 < argc) {
        param->insert_operator_1 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-i2") == 0 && i + 1 < argc) {
        param->insert_operator_2 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-r1") == 0 && i + 1 < argc) {
        param->remove_operator_1 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-r2") == 0 && i + 1 < argc) {
        param->remove_operator_2 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-r3") == 0 && i + 1 < argc) {
        param->remove_operator_3 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-r4") == 0 && i + 1 < argc) {
        param->remove_operator_4 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-r5") == 0 && i + 1 < argc) {
        param->remove_operator_5 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-r6") == 0 && i + 1 < argc) {
        param->remove_operator_6 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
      if (strcmp(argv[i], "-r7") == 0 && i + 1 < argc) {
        param->remove_operator_7 = std::stod(std::string(argv[i + 1])) / 10.0;
        i++;
      }
    }
    folder_path = instance_name;
    auto path_split = SplitStringWithDelimiter(folder_path, "/");
    // folder_path = "/home/cuong/Downloads/irace_tune/test/uniform-4-n19.txt";
    instance_name = path_split[path_split.size() - 1];
    instance_name.erase(instance_name.size() - 4, 4);
  }
  else {
    // "/home/cuong/CLionProjects/RV-FSTSP-heuristic/generated_instances_clone_10/r_rc/r_rc_6"
    folder_path = "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/Niels_instances/uniform/uniform-3-n16.txt";
    auto path_split = SplitStringWithDelimiter(folder_path, "/");
    instance_name = path_split[path_split.size() - 1];
    instance_name.erase(instance_name.size() - 4, 4);
    param->temperature_control = 0.134;
    param->cooling_rate = 0.99997;
    param->minimum_remove_rate = 0.16;
    param->maximum_remove_rate = 0.40;
    param->decay_insert = 0.81;
    param->decay_remove = 0.65;
    param->best_score = 4.6;
    param->improved_score = 5.1;
    param->accepted_score = 2.0;
    param->rejected_score = 3.4;
    param->insert_operator_1 = 4.3;
    param->insert_operator_2 = 3.2;
    param->remove_operator_1 = 1.4;
    param->remove_operator_2 = 4.7;
    param->remove_operator_3 = 7.9;
    param->remove_operator_4 = 7.7;
    param->remove_operator_5 = 4.3;
    param->remove_operator_6 = 2.2;
    param->remove_operator_7 = 6.1;
  }

  std::shared_ptr<Instance> instance =
    InstanceFactory::createInstance("N", folder_path);
  instance->read();

  u_int seed = 42;
  std::mt19937 gen(seed);
  double avg = 0;
  double avg_ini = 0;
  double best_objective = 1e9;
  bool loop = true;
  if (!loop) {
    std::cout << "NO LOOP VARIANT. STARTING..." << std::endl;
    std::vector<std::shared_ptr<SolutionL>> results(param->number_of_run);
    instance->restricted_dtl = true;

    std::shared_ptr<SolutionL> best_solution;

    std::string sol_file_name =
      "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/Niels_solutions/uniform/solutions/" +
      instance_name + "-DP.txt";
    std::string opt;
    bool opt_loop = false;
    if (auto exist = fs::exists(sol_file_name); exist) {
      std::cout << "Solution file found." << std::endl;
      if (std::ifstream ifs(sol_file_name); ifs.is_open()) {
        std::string line;
        int n1, n2;
        while (std::getline(ifs, line)) {
          if (line.find("Total cost") != std::string::npos) {
            // found
            std::string numberStr;

            // Find the position of the colon and the first digit
            size_t colonPos = line.find(':');
            size_t startPos = line.find_first_of("0123456789", colonPos);

            // Extract the number using substr
            if (startPos != std::string::npos) {
              numberStr = line.substr(startPos);
            }
            numberStr.erase(numberStr.size() - 3, 3);
            opt = numberStr;
            std::cout << "Optimal objective: " << std::stod(opt) << std::endl;
          }
          else {
            std::istringstream iss(line);
            iss >> n1 >> n2;
            if (n1 == n2 && n1 != 0) {
              std::cout << "loop: " << n1 << std::endl;
              opt_loop = true;
            }
          }
        }
      }
      else {
        throw std::runtime_error("Could not open solution file " +
          sol_file_name);
      }
    }
    else {
      throw std::runtime_error("Solution file not found: " + sol_file_name);
    }
    if (opt_loop) {
      return 0;
    }
    int iter_best = 1e9;
    const auto start =
      std::chrono::high_resolution_clock::now(); // Start timing
    for (int i = 0; i < param->number_of_run; ++i) {
      auto internal_seed = gen();
      auto alns = std::make_shared<ALNSL>(internal_seed, instance, param);
      results[i] = alns->Solve();
      if (fabs(best_objective - results[i]->objective) < 0.0001) {
        if (alns->iter_best < iter_best) {
          iter_best = alns->iter_best;
        }
      }
      else {
        if (results[i]->objective < best_objective) {
          best_objective = results[i]->objective;
          best_solution = results[i];
          iter_best = alns->iter_best;
        }
      }
      avg += results[i]->objective;
      avg_ini += alns->initial_objective;
    }

    const auto end = std::chrono::high_resolution_clock::now(); // End timing
    const std::chrono::duration<double> elapsed = end - start;
    avg /= param->number_of_run;
    avg_ini /= param->number_of_run;
    // get the optimal value of this instance.

    // construct the solution file name.

    auto optimal = std::stod(opt);
    double avg_ini_opt, avg_ini_best_heuristic;
    double avg_heuristic_opt, avg_heuristic_best_heuristic;
    double opt_gap;
    if (!opt.empty()) {
      // calculate average gap to the optimal
      avg_ini_opt = (avg_ini / optimal - 1) * 100;
      avg_heuristic_opt = (avg / optimal - 1) * 100;
      opt_gap = (best_objective / optimal - 1) * 100;
    }

    // to best heuristic
    avg_heuristic_best_heuristic = (avg / best_objective - 1) * 100;
    avg_ini_best_heuristic = (avg_ini / best_objective - 1) * 100;

    // print all?
    std::string out_file =
      "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/small_alns_no_loop.csv";
    std::fstream out_stream(out_file, std::ios::app);

    // Number of nodes,Instance name,Optimal(DP),Heuristic best (no
    // loops),Time(s),Gap to
    // optimal(%),Avg_ALNS_to_opt,Avg_ALNS_best,Avg_ini_to_opt,Avg_ini_to_best

    out_stream << instance->num_node << "," << opt_loop << "," << instance_name
      << "," << opt << "," << best_objective << "," << elapsed.count()
      << "," << opt_gap << "," << iter_best << "," << avg_heuristic_opt
      << "," << avg_heuristic_best_heuristic << "," << avg_ini_opt
      << "," << avg_ini_best_heuristic << "\n";
    std::cout << "Best solution:" << std::endl;
    best_solution->feasibility_check();
    std::cout << best_objective << std::endl;
    std::cout << "Elapsed time: " << elapsed.count() << std::endl;
  }
  else {
    bool loop_at_first_node = false;
    std::vector<std::shared_ptr<SolutionL>> results(param->number_of_run);
    instance->restricted_dtl = true;

    std::shared_ptr<SolutionL> best_solution;
    // best_solution->compare_results("/home/cuong/CLionProjects/RV-FSTSP-heuristic/big_instances.csv", "/home/cuong/CLionProjects/RV-FSTSP-heuristic/agatz_ep_all.csv", "/home/cuong/CLionProjects/RV-FSTSP-heuristic/compare_result.csv");
    // return 0;
    //
    std::string sol_file_name =
      "/home/cuong/CLionProjects/RV-FSTSP/Niels_solutions/uniform/solutions/" +
      instance_name + "-DP.txt";
    std::cout << sol_file_name << std::endl;
    std::string opt;
    bool opt_loop = false;
    if (auto exist = fs::exists(sol_file_name); exist) {
      std::cout << "Solution file found." << std::endl;
      if (std::ifstream ifs(sol_file_name); ifs.is_open()) {
        std::string line;
        int n1 = -1, n2 = -1;
        while (std::getline(ifs, line)) {
          if (line.find("Total cost") != std::string::npos) {
            // found
            std::string numberStr;

            // Find the position of the colon and the first digit
            size_t colonPos = line.find(':');
            size_t startPos = line.find_first_of("0123456789",
                                                 colonPos);

            // Extract the number using substr
            if (startPos != std::string::npos) {
              numberStr = line.substr(startPos);
            }
            numberStr.erase(numberStr.size() - 3, 3);
            opt = numberStr;
            std::cout << "Optimal objective: " << std::stod(opt) <<
              std::endl;
          }
          else {
            std::istringstream iss(line);
            iss >> n1 >> n2;
            if (n1 == n2 && n1 != 0) {
              opt_loop = true;
            }
            if (n1 == 0 && n2 == 0) {
              int n3;
              iss >> n3;
              if (n3 != -1) {
                std::cout << line << std::endl;
                std::cout << "Loop at the first node!" << std::endl;
                loop_at_first_node = true;
              }
            }
          }
        }
      }
      else {
        // return 0;
      }
    }
    else {
      // return 0;
    }
    // if (opt_loop == true) {
    //   return 0;
    // }
    std::cout << "-------------Parameter settings-------------------------" << std::endl;
    std::cout << "Iteration budget: " << param->max_iteration << std::endl;
    std::cout << "Maximum iteration without improvement: " << param->max_iteration_without_improvements << std::endl;
    std::cout << "Minimum removal rate: " << param->minimum_remove_rate << std::endl;
    std::cout << "Maximum removal rate: " << param->maximum_remove_rate << std::endl;
    std::cout << "Insert decay rate: " << param->decay_insert << std::endl;
    std::cout << "Remove decay rate: " << param->decay_remove << std::endl;
    std::cout << "Best score: " << param->best_score << std::endl;
    std::cout << "Improve score: " << param->improved_score << std::endl;
    std::cout << "Accept score: " << param->accepted_score << std::endl;
    std::cout << "Reject score: " << param->rejected_score << std::endl;
    std::cout << "Insert 1 weight: " << param->insert_operator_1 << std::endl;
    std::cout << "Insert 2 weight: " << param->insert_operator_2 << std::endl;
    std::cout << "Remove 1 weight: " << param->remove_operator_1 << std::endl;
    std::cout << "Remove 2 weight: " << param->remove_operator_2 << std::endl;
    std::cout << "Remove 3 weight: " << param->remove_operator_3 << std::endl;
    std::cout << "Remove 4 weight: " << param->remove_operator_4 << std::endl;
    std::cout << "Remove 5 weight: " << param->remove_operator_5 << std::endl;
    std::cout << "Remove 6 weight: " << param->remove_operator_6 << std::endl;
    std::cout << "Remove 7 weight: " << param->remove_operator_7 << std::endl;
    std::cout << "-------------End of param settings----------------" << std::endl;
    std::string out_file =
      "/Users/cuong/CLionProjects/ALNS_RV_FSTSP/uniform_extra_slsr_0_dtl_1e9.csv";
    std::fstream out_stream(out_file, std::ios::app);
    if (!out_stream.is_open()) {
      std::cout << "Could not open output file " << out_file << std::endl;
      exit(1);
    }
    // out_stream << instance_name << std::endl;
    // return 0;
    // out_stream << "\n" << loop_at_first_node << "," << instance->num_node << "," << opt_loop << "," <<
    // instance_name << ",";
    int iter_best = 1e9;
    const auto start = std::chrono::high_resolution_clock::now(); // Start timing
    for (int i = 0; i < param->number_of_run; ++i) {
      auto internal_seed = gen();
      auto alns = std::make_shared<ALNSL>(internal_seed, instance, param);
      results[i] = alns->Solve();
      if (fabs(best_objective - results[i]->objective) < 0.0001) {
        if (alns->iter_best < iter_best) {
          iter_best = alns->iter_best;
        }
      }
      else {
        if (results[i]->objective < best_objective) {
          best_objective = results[i]->objective;
          best_solution = results[i];
          iter_best = alns->iter_best;
        }
      }
      avg += results[i]->objective;
      avg_ini += alns->initial_objective;
    }

    const auto end = std::chrono::high_resolution_clock::now(); // End timing
    const std::chrono::duration<double> elapsed = end - start;
    avg /= param->number_of_run;
    avg_ini /= param->number_of_run;
    //
    //
    //
    // auto optimal = std::stod(opt);
    double avg_ini_opt, avg_ini_best_heuristic;
    double avg_heuristic_opt, avg_heuristic_best_heuristic;
    double opt_gap;
    // if (!opt.empty()) {
    //     // calculate average gap to the optimal
    //     avg_ini_opt = (avg_ini / optimal - 1) * 100;
    //     avg_heuristic_opt = (avg / optimal - 1) * 100;
    //     opt_gap = (best_objective / optimal - 1) * 100;
    // }
    // //
    // // to best heuristic
    avg_heuristic_best_heuristic = (avg / best_objective - 1) * 100;
    avg_ini_best_heuristic = (avg_ini / best_objective - 1) * 100;
    // //
    // //
    std::cout << "Best solution:" << std::endl;
    best_solution->feasibility_check();
    std::cout << best_objective << std::endl;
    std::cout << "elapsed: " << elapsed.count() << std::endl;
    // int n_loop = 0;
    // for (const int d: best_solution->served_by_drone) {
    //   if (best_solution->truck_route[best_solution->sortie_stages[d].start_index] == best_solution->truck_route[best_solution->sortie_stages[d].end_index]) {
    //     n_loop++;
    //   }
    // }
    // std::unordered_map<int, int> countMap;
    // std::vector<int> duplicateIndices;
    // std::vector<int> duplicateNodes;
    // // First pass: Count occurrences of each element
    // for (const auto& element : best_solution->truck_route) {
    //   countMap[element]++;
    // }
    //
    // // Second pass: Collect indices of duplicates
    // for (int i = 0; i < best_solution->truck_route.size(); ++i) {
    //   if (countMap[best_solution->truck_route[i]] > 1) {
    //     duplicateIndices.push_back(i);
    //   }
    // }


    //
    // // // Number of nodes,Instance name,Optimal(DP),Heuristic best (no
    // //
    // // out_stream << instance->num_node << "," <<
    // //   instance_name << "," << best_objective << "," <<
    // //       elapsed.count() << ","
    // //        << iter_best << "," << n_loop << "," << duplicateIndices.size() << "\n";
    //
    //
    // large outstream
    out_stream << instance->num_node << "," << instance_name << "," << best_objective << "," << avg << "," << elapsed.
      count() << ","
      << iter_best << "," << avg_heuristic_best_heuristic << "," << avg_ini_best_heuristic << "\n";
    // small outstream
    // out_stream << opt << "," << instance->num_node << "," << instance_name << "," << best_objective << "," <<
    //     elapsed.count() << ","
    //     << opt_gap << "," << iter_best << "," << avg_heuristic_opt << "," <<
    //     avg_heuristic_best_heuristic << "," << avg_ini_opt << ","
    //     << avg_ini_best_heuristic << "\n";
    out_stream.flush();
    out_stream.close();

    return 0;
  }
}
