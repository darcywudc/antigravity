# 使用官方 Nginx 镜像作为基础
FROM nginx:alpine

# 将当前目录下的所有文件复制到 Nginx 的默认静态文件目录
COPY . /usr/share/nginx/html

# Copy custom nginx configuration
COPY nginx.conf /etc/nginx/conf.d/default.conf

# Expose port 8080 (Cloud Run expectation)
EXPOSE 8080

# Start Nginx
CMD ["nginx", "-g", "daemon off;"]
