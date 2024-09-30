#ifndef STEPPING_DEBUG_HPP
#define STEPPING_DEBUG_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <mutex>
#include <condition_variable>

namespace fast_planner {

class SteppingDebug {
private:
  std::mutex mtx_;
  std::condition_variable cv_;
  bool proceed_;
  std::map<std::string, int> reason_count_;

public:
  void init();
  void waitForInput(const std::string& reason);
  void keyboardInput();
  void printReasonCounts();
};

}  // namespace fast_planner

#endif  // STEPPING_DEBUG_HPP
