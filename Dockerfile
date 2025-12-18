# 使用 x86 Python 基础镜像（OpenSeesPy 需要）
FROM --platform=linux/amd64 python:3.9-slim

# 安装系统依赖 - OpenSeesPy需要的库
RUN apt-get update && apt-get install -y \
    libgomp1 \
    libgfortran5 \
    libblas3 \
    liblapack3 \
    libgcc-s1 \
    libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

# 设置工作目录
WORKDIR /app

# 安装 Python 依赖
RUN pip install --no-cache-dir flask flask-cors openseespy

# 测试OpenSeesPy是否能导入
RUN python -c "import openseespy.opensees as ops; print('OpenSeesPy imported successfully')"

# 复制服务器代码
COPY opensees_server.py .

# 复制前端文件
COPY experiments/ ./experiments/
COPY index.html .

# 设置默认端口环境变量（Cloud Run会覆盖）
ENV PORT=8080

# 启动服务器
CMD ["python", "opensees_server.py"]
