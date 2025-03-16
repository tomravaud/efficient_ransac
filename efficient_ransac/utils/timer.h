#pragma once

#include <chrono>
#include <iostream>
#include <fstream>

namespace efficient_ransac {

struct Measurement {
  double value;
  std::string id;
};


class Timer {
 public:
  Timer(const std::string& name, const std::filesystem::path &output_path)
      : name_(name), output_path_(output_path), start_time_(std::chrono::high_resolution_clock::now()) {}

  ~Timer() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration<double>(end_time - start_time_).count();
    std::clog << "[INFO] " << name_ << " executed in " << duration << " s\n";
    std::ofstream file(output_path_.parent_path() /("timing.txt"), std::ios::app); 
    for (auto m : saved_times_) {
      file << m.id << " " << m.value << std::endl;
    }
    file.close(); 
  }

  void setTime(std::string id){
    Measurement m;
    m.value = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time_).count();
    m.id = id;
    saved_times_.push_back(m);
  }

 private:
  std::string name_;
  std::chrono::high_resolution_clock::time_point start_time_;
  std::vector<Measurement> saved_times_;
  std::filesystem::path output_path_;

};

}  // namespace efficient_ransac
