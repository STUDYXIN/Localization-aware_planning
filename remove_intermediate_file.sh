#!/bin/zsh

source ~/.zshrc

# 获取 vins 包的路径
VINS_PACKAGE_PATH=$(rospack find vins)

# 设置路径
REMOVE_PATH1="${VINS_PACKAGE_PATH}/../result"
REMOVE_PATH2="${VINS_PACKAGE_PATH}/../result/score_compare"
echo "开始删除:"

# 清空 REMOVE_PATH1 目录下的所有文件内容，保留文件本身
for file in "${REMOVE_PATH1}"/*; do
    if [ -f "$file" ]; then
       : > "$file"  # 清空文件内容
    fi
done
echo "1. 清空了 ${REMOVE_PATH1} 目录下所有文件的内容，保留文件本身。"
# 删除 REMOVE_PATH2 目录中的所有文件
rm -f "${REMOVE_PATH2}"/*

# 重新创建 feature_num.csv 文件
touch "${REMOVE_PATH2}/feature_num.csv"
echo "2. 删除了 ${REMOVE_PATH2} 目录中的所有文件，并重新创建了 feature_num.csv 文件。"
echo "操作完成:"