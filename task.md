# 线性代数的本质 (Essence of Linear Algebra) - 3Blue1Brown 结构版

## 项目目标
基于 3Blue1Brown "Essence of Linear Algebra" 系列视频重构内容组织。
侧重于几何直觉和交互式可视化探索。

## 章节开发计划 (基于 Essence of Linear Algebra 播放列表)

- [x] **1. 向量 (Vectors)**
  - 物理/CS/数学视角，加法，数乘。
  - [x] 从旧版 Ch1 迁移并精简
- [ ] **2. 线性组合、张成空间与基 (Linear combinations, span, and basis vectors)**
  - [ ] 线性组合可视化 (复用旧版Ch1组件)
  - [ ] 什么是空间 (Span)
  - [ ] 线性相关性与基的定义
- [ ] **3. 线性变换与矩阵 (Linear transformations and matrices)**
  - [ ] 变换的概念：输入向量->输出向量
  - [ ] 矩阵作为变换的数值描述
  - [ ] 可视化：网格变换 (复用旧版Ch2)
- [ ] **4. 矩阵乘法与复合变换 (Matrix multiplication as composition)**
  - [ ] 复合变换的几何意义：先旋转再剪切 vs 先剪切再旋转
  - [ ] 矩阵乘法规则的几何推导
- [ ] **5. 三维线性变换 (Three-dimensional linear transformations)**
  - [ ] 3D 坐标系与变换可视化 (需要 3D 引擎支持)
- [ ] **6. 行列式 (The determinant)**
  - [ ] 面积缩放因子 (2D)
  - [ ] 体积缩放因子 (3D)
  - [ ] 负行列式的含义 (手性/翻转)
- [ ] **7. 逆矩阵、列空间与零空间 (Inverse matrices, column space and null space)**
  - [ ] 线性方程组的几何意义 (变换后寻找解)
  - [ ] 秩 (Rank) 的直观理解
- [ ] **8. 非方阵 (Nonsquare matrices)**
  - [ ] 维度变换 (2D->3D, 3D->2D)
- [ ] **9. 点积与对偶性 (Dot products and duality)**
  - [ ] 投影的几何解释
  - [ ] 为什么点积与投影有关？(对偶性)
- [ ] **10. 叉积 (Cross products)**
  - [ ] 标准几何介绍 (右手定则，面积)
- [ ] **11. 线性变换视角下的叉积 (Cross products visualised)**
  - [ ] 真正的数学定义
- [ ] **12. 克莱姆法则 (Cramer's rule)**
  - [ ] 几何解释
- [ ] **13. 基变换 (Change of basis)**
  - [ ] "Jennifer"的坐标系 vs "我们"的坐标系
  - [ ] $A^{-1} M A$ 的含义
- [ ] **14. 特征值与特征向量 (Eigenvectors and eigenvalues)**
  - [ ] 特征基
- [ ] **15. 抽象向量空间 (Abstract vector spaces)**
  - [ ] 函数作为向量，导数作为变换

## 技术任务
- [x] 创建分支 `essence-of-la`
- [ ] 更新侧边栏导航
- [ ] 拆分现有脚本和 HTML 以适配新结构
- [ ] 升级可视化引擎支持 3D (可选/待定)
