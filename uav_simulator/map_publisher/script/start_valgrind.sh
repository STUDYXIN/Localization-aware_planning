#!/bin/bash

# 检测并获取 exploration_node 的 PID
PID=$(pgrep -f exploration_node)

# 检查是否找到了 PID
if [ -z "$PID" ]; then
    echo "exploration_node is not running."
    exit 1
fi

echo "Found exploration_node with PID: $PID"

# 创建一个 GDB 命令文件，用于在目标进程中启动 valgrind
GDB_CMD_FILE=$(mktemp)

cat << EOF > $GDB_CMD_FILE
call (int)system("valgrind --tool=massif --massif-out-file=/home/star/ActiveSlam/DEUBG_FILE/massif.out.$PID --pid=$PID &")
detach
quit
EOF

# 使用 gdb 将 valgrind 注入到指定的 PID
gdb -p $PID -batch -x $GDB_CMD_FILE

# 删除临时的 GDB 命令文件
rm $GDB_CMD_FILE

echo "Valgrind has been injected into exploration_node with PID: $PID"
