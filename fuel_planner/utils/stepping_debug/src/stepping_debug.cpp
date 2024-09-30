#include "stepping_debug.hpp"
#include <thread>
namespace fast_planner {

void SteppingDebug::init() {
  proceed_ = false;
}

void SteppingDebug::waitForInput(const std::string& reason) {
  std::unique_lock<std::mutex> lock(mtx_);

  std::cout << "Paused due to reason: \033[36m" << reason << "\033[0m" << std::endl;
  reason_count_[reason]++;
  std::thread input_thread(&SteppingDebug::keyboardInput, this);
  cv_.wait(lock, [this]() { return proceed_; });
  std::cout << "Input received. Continuing..." << std::endl;
  std::cout << "Times reason '" << reason << "' occurred: " << reason_count_[reason] << std::endl;
  proceed_ = false;
  input_thread.join();
}

// 等待键盘输入的辅助函数
void SteppingDebug::keyboardInput() {
  std::string user_input;
  std::cout << "Please press Enter to continue..." << std::endl;

  std::getline(std::cin, user_input);
  {
    std::lock_guard<std::mutex> lock(mtx_);
    proceed_ = true;
  }
  cv_.notify_one();
}

// 输出字符调用次数的统计
void SteppingDebug::printReasonCounts() {
  std::cout << "Reason call counts:" << std::endl;
  for (const auto& entry : reason_count_) {
    std::cout << "Reason: " << entry.first << ", Count: " << entry.second << std::endl;
  }
}

}  // namespace fast_planner
