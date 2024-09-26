#ifndef TIME_DEBUG_HPP
#define TIME_DEBUG_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <unordered_map>
namespace fast_planner {
class DebugTimer {
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> overall_start_time;  // 记录总开始时间
  std::unordered_map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>>
      function_start_times;                                    // 记录每个函数的开始时间
  std::vector<std::pair<std::string, double>> function_times;  // 记录函数名字和运行时间
  bool print_time_;

public:
  // 设置调试的起始时间，清空之前的数据
  void setstart_time(const std::string& function_name, bool print_time);
  void setstart_time(const std::string& function_name, int output_thr);  // 隔多少次输出一次
  // 记录某个函数的开始时间
  void function_start(const std::string& function_name);
  // 记录某个函数的结束时间，并可选择性输出该函数的运行时间
  void function_end(const std::string& function_name);
  // 输出所有记录的函数运行时间，以及总运行时间
  void output_time();
};
}  // namespace fast_planner
#endif  // TIME_DEBUG_HPP
