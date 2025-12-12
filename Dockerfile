# 使用官方 Nginx 镜像作为基础
FROM nginx:alpine

# 将当前目录下的所有文件复制到 Nginx 的默认静态文件目录
COPY . /usr/share/nginx/html

# 暴露 80 端口
EXPOSE 80

# 启动 Nginx
CMD ["nginx", "-g", "daemon off;"]
