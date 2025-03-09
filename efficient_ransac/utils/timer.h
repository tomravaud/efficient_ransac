#pragma once

#include <chrono>
#include <iostream>

namespace efficient_ransac {

class Timer {
 public:
  Timer(const std::string& name)
      : name_(name), start_time_(std::chrono::high_resolution_clock::now()) {}

  ~Timer() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration<double>(end_time - start_time_).count();
    std::clog << "[INFO] " << name_ << " executed in " << duration << " s\n";
  }

 private:
  std::string name_;
  std::chrono::high_resolution_clock::time_point start_time_;
};

}  // namespace efficient_ransac
