#!/bin/bash

# 读取 CSV 文件的路径

csv_file="/home/lightwheel/Downloads/Floorplan.csv"

# 逐行读取 CSV 文件
while IFS=, read -r file git_version; do
    # 跳过 CSV 文件的标题行
    if [[ "$file" == "file" && "$git_version" == "git_version" ]]; then
        continue
    fi

    # 检查 file 和 git_version 是否为空
    if [[ -n "$file" && -n "$git_version" ]]; then
        # 构建 layout_{file} 的路径
        layout_file="layout${file}.yaml"

        # 执行 git checkout 命令
        echo "Checking out $layout_file to version $git_version"
        git checkout "$git_version" "$layout_file"
    fi
done < "$csv_file"
