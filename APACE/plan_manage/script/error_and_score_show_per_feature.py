import csv
import sys
import rospkg
import os
import matplotlib.pyplot as plt
# 文件路径
rospack = rospkg.RosPack()
package_name = 'vins'  # 你的功能包名称
package_path = rospack.get_path(package_name)

# 构建相对路径
relative_path = 'result/features_data.csv'
parent_directory = os.path.dirname(package_path)
full_path = os.path.join(parent_directory, relative_path)
full_path = os.path.normpath(full_path)

features = []
# 读取 CSV 文件
with open(full_path, mode='r') as file:
    reader = csv.reader(file)
    headers = next(reader)
    
    for row in reader:
        feature_id = int(row[0])
        start_frame = int(row[1])
        used_num = int(row[2])
        score = float(row[3])
        stereo_error = [list(map(float, error.split())) for error in row[4].split(';') if error]
        track_errors = [list(map(float, error.split())) for error in row[5].split(';') if error]
        truth_errors = [list(map(float, error.split())) for error in row[6].split(';') if error]
        feature = {
            'feature_id': feature_id,
            'start_frame': start_frame,
            'used_num': used_num,
            'score': score,
            'stereo_error': stereo_error,
            'track_error_per_frame': track_errors,
            'truth_error_per_frame': truth_errors
        }
        features.append(feature)

# 输出数据
print(f"Number of features: {len(features)}")

# 提取数据
truth_errors_first_element = []
scores = []
stereo_errors = []
track_errors_first_element = []

for feature in features:
    if feature['truth_error_per_frame']:
        truth_errors_first_element.append(feature['truth_error_per_frame'][0][3])
        scores.append(feature['score'])
        stereo_error_ = abs(feature['stereo_error'][0][2])*500
        stereo_errors.append(stereo_error_)
        track_errors_first_element.append(feature['track_error_per_frame'][0][2])

# 对 truth_errors_first_element 进行排序
sorted_indices = sorted(range(len(truth_errors_first_element)), key=lambda i: truth_errors_first_element[i])
sorted_truth_errors_first_element = [truth_errors_first_element[i] for i in sorted_indices]
sorted_scores = [scores[i] for i in sorted_indices]
sorted_stereo_errors = [stereo_errors[i] for i in sorted_indices]
sorted_track_errors_first_element = [track_errors_first_element[i] for i in sorted_indices]

# sorted_indices = sorted(range(len(track_errors_first_element)), key=lambda i: track_errors_first_element[i])
# sorted_truth_errors_first_element = [truth_errors_first_element[i] for i in sorted_indices]
# sorted_scores = [scores[i] for i in sorted_indices]
# sorted_stereo_errors = [stereo_errors[i] for i in sorted_indices]
# sorted_track_errors_first_element = [track_errors_first_element[i] for i in sorted_indices]

# 绘制图表
plt.figure(figsize=(10, 6))

plt.plot(sorted_truth_errors_first_element, sorted_scores, 'o-', label='Score')
# plt.plot(sorted_truth_errors_first_element, sorted_stereo_errors, 's-', label='Stereo Error')
# plt.plot(sorted_truth_errors_first_element, sorted_track_errors_first_element, '^-', label='Track Error (first element)')
# plt.plot(sorted_stereo_errors, sorted_scores, '^-', label='Track Error (first element)')
# plt.plot(sorted_track_errors_first_element, sorted_stereo_errors, '^-', label='Track Error (first element)')

plt.xlabel('Truth Error (first element)')
plt.ylabel('Values')
plt.title('Truth Error vs. Score, Stereo Error, and Track Error (first element)')
plt.legend()
plt.grid(True)
plt.show()