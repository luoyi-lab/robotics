#!/bin/bash
# PyBullet 环境搭建脚本（在 WSL Ubuntu 里运行）
set -e

export PATH="$HOME/.local/bin:$PATH"
export https_proxy=http://127.0.0.1:7890
export http_proxy=http://127.0.0.1:7890

echo "=== 1) 下载并安装 Python 3.11 ==="
uv python install 3.11

echo
echo "=== 2) 当前已安装的 Python ==="
uv python list --only-installed

echo
echo "=== 3) 创建虚拟环境 ~/pybullet-env ==="
if [ -d "$HOME/pybullet-env" ]; then
    echo "(已存在，跳过)"
else
    uv venv --python 3.11 "$HOME/pybullet-env"
fi

echo
echo "=== 4) 安装 pybullet + numpy + matplotlib ==="
uv pip install --python "$HOME/pybullet-env/bin/python" pybullet numpy matplotlib

echo
echo "=== 5) 版本确认 ==="
"$HOME/pybullet-env/bin/python" -c "import sys, pybullet, numpy, matplotlib; print('Python', sys.version.split()[0]); print('pybullet', pybullet.getAPIVersion()); print('numpy', numpy.__version__); print('matplotlib', matplotlib.__version__)"

echo
echo "=== 完成 ==="
