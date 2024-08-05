import csv
import sys
import rospkg
import os
import matplotlib.pyplot as plt
import cv2
import numpy as np
pt_mouse = [0,0]
def mouse_callback(event, x, y, flags, param):
    global pt_mouse
    pt_mouse = x, y
    # print("mouth callback",pt_mouse)

def score_for_one_feature(score_point, imLeft):
    winSize = (21, 21)
    halfWin = (winSize[0] - 1) * 0.5, (winSize[1] - 1) * 0.5

    I = imLeft
    derivI = cv2.Sobel(I, cv2.CV_16S, 1, 1)  # 计算导数

    iprevPt = (int(score_point[0] - halfWin[0]), int(score_point[1] - halfWin[1]))

    # 检查是否在边界内
    if iprevPt[0] < 0 or iprevPt[0] + winSize[0] >= I.shape[1] or \
       iprevPt[1] < 0 or iprevPt[1] + winSize[1] >= I.shape[0]:
        return 0  # 超出边界，无法计算

    a = score_point[0] - iprevPt[0]
    b = score_point[1] - iprevPt[1]
    W_BITS = 14
    FLT_SCALE = 1.0 / (1 << 20)
    iw00 = int((1.0 - a) * (1.0 - b) * (1 << W_BITS))
    iw01 = int(a * (1.0 - b) * (1 << W_BITS))
    iw10 = int((1.0 - a) * b * (1 << W_BITS))
    iw11 = (1 << W_BITS) - iw00 - iw01 - iw10

    iA11 = iA12 = iA22 = 0.0

    # 遍历窗口内的每个像素
    for y in range(winSize[1]):
        for x in range(winSize[0]):
            src_x = iprevPt[0] + x
            src_y = iprevPt[1] + y

            if src_x >= 0 and src_x < I.shape[1] - 1 and src_y >= 0 and src_y < I.shape[0] - 1:
                dsrc_x = derivI[src_y, src_x, 0] if I.ndim > 2 else derivI[src_y, src_x]  # x 方向的导数
                dsrc_y = derivI[src_y, src_x + 1, 0] if I.ndim > 2 else derivI[src_y, src_x + 1]  # x 方向的导数（右）
                ixval = int(dsrc_x * iw00 + dsrc_x * iw01 + dsrc_y * iw10 + dsrc_y * iw11)
                
                dsrc_x = derivI[src_y + 1, src_x, 0] if I.ndim > 2 else derivI[src_y + 1, src_x]  # y 方向的导数
                dsrc_y = derivI[src_y + 1, src_x + 1, 0] if I.ndim > 2 else derivI[src_y + 1, src_x + 1]  # y 方向的导数（右）
                iyval = int(dsrc_x * iw00 + dsrc_x * iw01 + dsrc_y * iw10 + dsrc_y * iw11)

                iA11 += ixval * ixval
                iA12 += ixval * iyval
                iA22 += iyval * iyval

    A11 = iA11 * FLT_SCALE
    A12 = iA12 * FLT_SCALE
    A22 = iA22 * FLT_SCALE

    minEig = (A22 + A11 - np.sqrt((A11 - A22) ** 2 + 4.0 * A12 ** 2)) / (2 * winSize[0] * winSize[1])

    return minEig # 调整系数，确保返回值范围合理

def process_features(features):
    id = 0
    num_features = len(features)
    id_last = -1
    while True:
        if id != id_last:
            feature = features[id]
            image = feature['feature_picture']
            feature_position = feature['feature_position']
            
            if len(image.shape) == 2:  # 如果是灰度图像
                image_score = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
                image_origin = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            else:
                image_score = image.copy()
                image_origin = image.copy()

            features_with_score = []
            for pt in feature_position:
                score = score_for_one_feature(pt, image_origin)
                features_with_score.append((pt, score))

            features_with_score.sort(key=lambda x: x[1], reverse=False)  # 按分数从低到高排序

            if features_with_score:
                min_score = features_with_score[0][1]
                max_score = features_with_score[-1][1]
                avg_score = np.mean([score for _, score in features_with_score])
            else:
                min_score = max_score = avg_score = 0

            for i, (pt, score) in enumerate(features_with_score):
                brightness = int((i + 1.0) / len(features_with_score) * 255)
                brightness = max(0, min(255, brightness))
                cv2.circle(image_score, (int(pt[0]), int(pt[1])), 3, (0, 0, brightness), 2)
                top_left = (int(pt[0] - 10.5), int(pt[1] - 10.5))  # 21/2 = 10.5，矩形中心在特征点
                bottom_right = (int(pt[0] + 10.5), int(pt[1] + 10.5))
                cv2.rectangle(image_score, top_left, bottom_right, (0, 255, 0), 1)  # 绿色矩形，线宽为1
            print("This is picture:", id)
            cv2.setMouseCallback('Highlighted Features', mouse_callback)
        id_last = id
        global pt_mouse    
        score_now = score_for_one_feature(pt_mouse, image_origin)
        distance_near = 999
        for i, (pt, score) in enumerate(features_with_score):
            pt_array = np.array(pt)
            pt_mouse_array = np.array(pt_mouse)
            distance = np.linalg.norm(pt_array - pt_mouse_array)
            if(distance < distance_near):
                distance_near = distance
                closest_score = score
        feature_score_near = closest_score

        # 创建黑色背景的矩形区域
        height, width, _ = image_score.shape
        text_bg_height = 40
        text_bg = np.zeros((text_bg_height, width, 3), dtype=np.uint8)
        text_bg[:] = [0, 0, 0]  # 黑色背景

        # 在黑色背景上写入文本
        text = (f"ID: {id:2d} AVG: {avg_score:.4f} SCORE: {score_now:.4f} fe_SCORE: {feature_score_near:.4f}")
        
        cv2.putText(text_bg, text, (10, text_bg_height // 2 + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)  # 红色文本
        
        # 将黑色背景区域添加到原始图像底部
        image_score_with_text = np.vstack((image_score, text_bg))

        cv2.imshow("Highlighted Features", image_score_with_text)
        
        key = cv2.waitKey(1)

        if key == 27:  # ESC键退出
            break
        elif key == 81 or key == 82:  # 左箭头键查看上一个图像
            id = (id - 1) % num_features
        elif key == 83 or key == 84 or key == 32:  # 右箭头键或空格查看下一个图像
            id = (id + 1) % num_features
    
    cv2.destroyAllWindows()  # 关闭所有OpenCV窗口




# 文件路径
rospack = rospkg.RosPack()
package_name = 'vins'  # 你的功能包名称
package_path = rospack.get_path(package_name)

# 构建相对路径
relative_path = 'result/score_compare/feature_num.csv'
image_path = 'result/score_compare'
parent_directory = os.path.dirname(package_path)
csv_path = os.path.join(parent_directory, relative_path)
image_directory = os.path.join(parent_directory, image_path)

features = []
# 读取 CSV 文件
with open(csv_path, mode='r') as file:
    reader = csv.reader(file)
    headers = next(reader)
    
    for idx, row in enumerate(reader):
        feature_positions = row[0]  # 假设特征点数据在第一列
        # 处理特征点数据
        points = feature_positions.split(';')
        feature_position = [tuple(map(float, point.split())) for point in points if point.strip()]
        # 生成图像文件路径
        image_file = f"output{idx}.png"
        image_path = os.path.join(image_directory, image_file)
        image = cv2.imread(image_path)

        feature = {
            'feature_position': feature_position,
            'feature_picture': image,
        }
        features.append(feature)

def main():
    try:
        cv2.namedWindow('Highlighted Features')
        process_features(features)
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        cv2.destroyAllWindows()  # 确保关闭所有OpenCV窗口

if __name__ == "__main__":
    main()