#include "time_debug.hpp"
#include <iomanip>  // for std::setw and std::left
namespace fast_planner {

void DebugTimer::setstart_time(const std::string& function_name, bool print_time) {
  overall_start_time = std::chrono::high_resolution_clock::now();
  function_start_times.clear();  // 清除之前的函数开始时间
  function_times.clear();        // 清除之前的函数时间数据
  print_time_ = print_time;
  if (print_time_) {
    std::cout << "\n==================== Function Execution Times "
                 "===================="
              << std::endl;
    std::cout << "[DebugTimer] Start Record: " << function_name << " ------------ " << std::endl;
  }
}
void DebugTimer::setstart_time(const std::string& function_name, int output_thr) {
  overall_start_time = std::chrono::high_resolution_clock::now();
  function_start_times.clear();  // 清除之前的函数开始时间
  function_times.clear();        // 清除之前的函数时间数据
  static int enter_count = 0;
  if (++enter_count >= output_thr) {
    print_time_ = true;
    enter_count = 0;
  } else
    print_time_ = false;
  if (print_time_) {
    std::cout << "\n==================== Function Execution Times "
                 "===================="
              << std::endl;
    std::cout << "[DebugTimer] Start Record: " << function_name << " ------------ " << std::endl;
  }
}

void DebugTimer::function_start(const std::string& function_name) {
  if (print_time_) {
    function_start_times[function_name] = std::chrono::high_resolution_clock::now();
  }
}

void DebugTimer::function_end(const std::string& function_name) {
  if (print_time_) {
    auto it = function_start_times.find(function_name);
    if (it == function_start_times.end()) {
      std::cerr << "[DebugTimer] Error: function '" << function_name << "' was not started!" << std::endl;
      return;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(end_time - it->second).count();
    function_times.push_back({ function_name, elapsed_time });
    function_start_times.erase(it);
    std::cout << std::left << " -  " << std::setw(30) << function_name << ":    " << std::fixed << std::setprecision(6)
              << elapsed_time << " (sec)" << std::endl;
  }
}

void DebugTimer::output_time() {
  if (print_time_) {
    //   std::cout << "==================== Function Execution Times
    //   ====================" << std::endl; for (const auto& entry :
    //   function_times) {
    //     std::cout << "Time of " << std::left << std::setw(24) << entry.first
    //     << ": " << std::fixed << std::setprecision(6)
    //               << entry.second << " (sec)" << std::endl;
    //   }
    auto total_end_time = std::chrono::high_resolution_clock::now();
    double total_elapsed_time = std::chrono::duration<double>(total_end_time - overall_start_time).count();
    function_start_times.clear();  // 清除之前的函数开始时间
    function_times.clear();        // 清除之前的函数时间数据
    std::cout << "\033[33m"        // 开始黄色输出
              << std::left << "----" << std::setw(30) << "Total elapsed time" << std::fixed << ":    " << std::setprecision(6)
              << total_elapsed_time << " (sec)"
              << "\033[0m"  // 重置颜色
              << std::endl;
    std::cout << "=================================================================\n" << std::endl;
  }
}

}  // namespace fast_planner
