#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

class RealTimePlotter {
private:
  std::map<std::string, std::vector<double>> data_x;  // 存储自变量（横轴）
  std::map<std::string, std::vector<double>> data_y;  // 存储因变量（纵轴）

public:
  // 1. 初始化图表，清除现有数据
  void initPlot(const std::string& x_label, const std::string& y_label) {
    // 如果之前有相同的变量，清除数据
    if (data_x.find(x_label) != data_x.end()) {
      data_x[x_label].clear();
    }
    if (data_y.find(y_label) != data_y.end()) {
      data_y[y_label].clear();
    }

    // 设置图表标题和轴标签
    plt::figure();
    plt::xlabel(x_label);
    plt::ylabel(y_label);
    plt::grid(true);
  }

  // 2. 存储自变量和因变量的值
  void addData(const std::string& x_label, const std::string& y_label, double x_value, double y_value) {
    data_x[x_label].push_back(x_value);
    data_y[y_label].push_back(y_value);
  }

  // 3. 实时绘图，更新一次
  void updatePlot(const std::string& x_label, const std::string& y_label) {
    if (data_x.find(x_label) != data_x.end() && data_y.find(y_label) != data_y.end()) {
      plt::clf();                                         // 清除之前的绘图
      plt::plot(data_x[x_label], data_y[y_label], "-r");  // 绘制红色线条
      plt::draw();
      plt::pause(0.01);  // 暂停，确保显示更新
    } else {
      std::cerr << "Error: Variables not found!" << std::endl;
    }
  }
};
