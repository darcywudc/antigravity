/**
 * 矩阵运算库
 * 提供矩阵的基本运算和分解
 */

class Matrix {
    constructor(rows, cols, data = null) {
        this.rows = rows;
        this.cols = cols;

        if (data) {
            this.data = data;
        } else {
            // 初始化为零矩阵
            this.data = [];
            for (let i = 0; i < rows; i++) {
                this.data[i] = new Array(cols).fill(0);
            }
        }
    }

    // 获取元素
    get(i, j) {
        return this.data[i][j];
    }

    // 设置元素
    set(i, j, value) {
        this.data[i][j] = value;
        return this;
    }

    // 克隆矩阵
    clone() {
        const newData = this.data.map(row => [...row]);
        return new Matrix(this.rows, this.cols, newData);
    }

    // 矩阵加法
    add(m) {
        if (this.rows !== m.rows || this.cols !== m.cols) {
            throw new Error('矩阵维度不匹配');
        }
        const result = new Matrix(this.rows, this.cols);
        for (let i = 0; i < this.rows; i++) {
            for (let j = 0; j < this.cols; j++) {
                result.data[i][j] = this.data[i][j] + m.data[i][j];
            }
        }
        return result;
    }

    // 矩阵减法
    subtract(m) {
        if (this.rows !== m.rows || this.cols !== m.cols) {
            throw new Error('矩阵维度不匹配');
        }
        const result = new Matrix(this.rows, this.cols);
        for (let i = 0; i < this.rows; i++) {
            for (let j = 0; j < this.cols; j++) {
                result.data[i][j] = this.data[i][j] - m.data[i][j];
            }
        }
        return result;
    }

    // 数乘
    scale(scalar) {
        const result = new Matrix(this.rows, this.cols);
        for (let i = 0; i < this.rows; i++) {
            for (let j = 0; j < this.cols; j++) {
                result.data[i][j] = this.data[i][j] * scalar;
            }
        }
        return result;
    }

    // 矩阵乘法
    multiply(m) {
        if (this.cols !== m.rows) {
            throw new Error('矩阵维度不兼容');
        }
        const result = new Matrix(this.rows, m.cols);
        for (let i = 0; i < this.rows; i++) {
            for (let j = 0; j < m.cols; j++) {
                let sum = 0;
                for (let k = 0; k < this.cols; k++) {
                    sum += this.data[i][k] * m.data[k][j];
                }
                result.data[i][j] = sum;
            }
        }
        return result;
    }

    // 矩阵与向量相乘
    multiplyVector(v) {
        if (this.cols !== v.length) {
            throw new Error('维度不匹配');
        }
        const result = [];
        for (let i = 0; i < this.rows; i++) {
            let sum = 0;
            for (let j = 0; j < this.cols; j++) {
                sum += this.data[i][j] * v[j];
            }
            result.push(sum);
        }
        return result;
    }

    // 转置
    transpose() {
        const result = new Matrix(this.cols, this.rows);
        for (let i = 0; i < this.rows; i++) {
            for (let j = 0; j < this.cols; j++) {
                result.data[j][i] = this.data[i][j];
            }
        }
        return result;
    }

    // 行列式（仅适用于方阵）
    determinant() {
        if (this.rows !== this.cols) {
            throw new Error('只有方阵才有行列式');
        }

        const n = this.rows;

        if (n === 1) {
            return this.data[0][0];
        }

        if (n === 2) {
            return this.data[0][0] * this.data[1][1] - this.data[0][1] * this.data[1][0];
        }

        // 使用LU分解计算行列式
        const { L, U, permutations } = this.luDecomposition();
        let det = Math.pow(-1, permutations);
        for (let i = 0; i < n; i++) {
            det *= U.data[i][i];
        }
        return det;
    }

    // LU分解（带部分主元选取）
    luDecomposition() {
        const n = this.rows;
        const L = Matrix.identity(n);
        const U = this.clone();
        let permutations = 0;

        for (let k = 0; k < n; k++) {
            // 找主元
            let maxRow = k;
            let maxVal = Math.abs(U.data[k][k]);
            for (let i = k + 1; i < n; i++) {
                if (Math.abs(U.data[i][k]) > maxVal) {
                    maxVal = Math.abs(U.data[i][k]);
                    maxRow = i;
                }
            }

            // 交换行
            if (maxRow !== k) {
                [U.data[k], U.data[maxRow]] = [U.data[maxRow], U.data[k]];
                permutations++;
                // 交换L的已计算部分
                for (let j = 0; j < k; j++) {
                    [L.data[k][j], L.data[maxRow][j]] = [L.data[maxRow][j], L.data[k][j]];
                }
            }

            // 消元
            for (let i = k + 1; i < n; i++) {
                if (Math.abs(U.data[k][k]) < 1e-10) continue;
                const factor = U.data[i][k] / U.data[k][k];
                L.data[i][k] = factor;
                for (let j = k; j < n; j++) {
                    U.data[i][j] -= factor * U.data[k][j];
                }
            }
        }

        return { L, U, permutations };
    }

    // 逆矩阵
    inverse() {
        if (this.rows !== this.cols) {
            throw new Error('只有方阵才有逆矩阵');
        }

        const n = this.rows;
        const augmented = new Matrix(n, 2 * n);

        // 构建增广矩阵 [A | I]
        for (let i = 0; i < n; i++) {
            for (let j = 0; j < n; j++) {
                augmented.data[i][j] = this.data[i][j];
                augmented.data[i][j + n] = (i === j) ? 1 : 0;
            }
        }

        // 高斯-乔丹消元
        for (let col = 0; col < n; col++) {
            // 找主元
            let maxRow = col;
            for (let row = col + 1; row < n; row++) {
                if (Math.abs(augmented.data[row][col]) > Math.abs(augmented.data[maxRow][col])) {
                    maxRow = row;
                }
            }
            [augmented.data[col], augmented.data[maxRow]] = [augmented.data[maxRow], augmented.data[col]];

            // 检查是否奇异
            if (Math.abs(augmented.data[col][col]) < 1e-10) {
                throw new Error('矩阵是奇异的，无法求逆');
            }

            // 归一化主元行
            const pivot = augmented.data[col][col];
            for (let j = 0; j < 2 * n; j++) {
                augmented.data[col][j] /= pivot;
            }

            // 消去其他行
            for (let row = 0; row < n; row++) {
                if (row !== col) {
                    const factor = augmented.data[row][col];
                    for (let j = 0; j < 2 * n; j++) {
                        augmented.data[row][j] -= factor * augmented.data[col][j];
                    }
                }
            }
        }

        // 提取逆矩阵
        const inv = new Matrix(n, n);
        for (let i = 0; i < n; i++) {
            for (let j = 0; j < n; j++) {
                inv.data[i][j] = augmented.data[i][j + n];
            }
        }
        return inv;
    }

    // 特征值和特征向量（2x2矩阵的解析解）
    eigenvalues2x2() {
        if (this.rows !== 2 || this.cols !== 2) {
            throw new Error('此方法仅适用于2x2矩阵');
        }

        const a = this.data[0][0];
        const b = this.data[0][1];
        const c = this.data[1][0];
        const d = this.data[1][1];

        const trace = a + d;
        const det = a * d - b * c;
        const discriminant = trace * trace - 4 * det;

        if (discriminant >= 0) {
            const sqrtDisc = Math.sqrt(discriminant);
            return [
                (trace + sqrtDisc) / 2,
                (trace - sqrtDisc) / 2
            ];
        } else {
            // 复数特征值
            const real = trace / 2;
            const imag = Math.sqrt(-discriminant) / 2;
            return [
                { real, imag },
                { real, imag: -imag }
            ];
        }
    }

    // 特征向量（2x2矩阵）
    eigenvectors2x2() {
        const eigenvalues = this.eigenvalues2x2();
        const vectors = [];

        for (const lambda of eigenvalues) {
            if (typeof lambda === 'object') {
                // 复数特征值，跳过
                vectors.push(null);
                continue;
            }

            // (A - λI)v = 0
            const a = this.data[0][0] - lambda;
            const b = this.data[0][1];

            let v;
            if (Math.abs(b) > 1e-10) {
                v = [-b, a];
            } else if (Math.abs(a) > 1e-10) {
                v = [1, 0];
            } else {
                v = [0, 1];
            }

            // 归一化
            const mag = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
            vectors.push([v[0] / mag, v[1] / mag]);
        }

        return { eigenvalues, eigenvectors: vectors };
    }

    // 行阶梯形
    rowEchelonForm() {
        const result = this.clone();
        const m = result.rows;
        const n = result.cols;

        let lead = 0;
        for (let r = 0; r < m; r++) {
            if (lead >= n) break;

            let i = r;
            while (Math.abs(result.data[i][lead]) < 1e-10) {
                i++;
                if (i === m) {
                    i = r;
                    lead++;
                    if (lead === n) return result;
                }
            }

            [result.data[i], result.data[r]] = [result.data[r], result.data[i]];

            const lv = result.data[r][lead];
            for (let j = 0; j < n; j++) {
                result.data[r][j] /= lv;
            }

            for (let i = 0; i < m; i++) {
                if (i !== r) {
                    const lv = result.data[i][lead];
                    for (let j = 0; j < n; j++) {
                        result.data[i][j] -= lv * result.data[r][j];
                    }
                }
            }

            lead++;
        }

        return result;
    }

    // 转换为2D数组
    toArray() {
        return this.data.map(row => [...row]);
    }

    // 打印矩阵
    toString(precision = 2) {
        return this.data.map(row =>
            row.map(val => val.toFixed(precision).padStart(8)).join(' ')
        ).join('\n');
    }

    // 静态方法：单位矩阵
    static identity(n) {
        const m = new Matrix(n, n);
        for (let i = 0; i < n; i++) {
            m.data[i][i] = 1;
        }
        return m;
    }

    // 静态方法：零矩阵
    static zeros(rows, cols) {
        return new Matrix(rows, cols);
    }

    // 静态方法：从2D数组创建
    static fromArray(arr) {
        const rows = arr.length;
        const cols = arr[0].length;
        return new Matrix(rows, cols, arr.map(row => [...row]));
    }

    // 2D变换矩阵
    static rotation2D(angle) {
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);
        return Matrix.fromArray([
            [cos, -sin],
            [sin, cos]
        ]);
    }

    static scale2D(sx, sy) {
        return Matrix.fromArray([
            [sx, 0],
            [0, sy]
        ]);
    }

    static shear2D(kx, ky) {
        return Matrix.fromArray([
            [1, kx],
            [ky, 1]
        ]);
    }

    static reflection2D(axis) {
        if (axis === 'x') {
            return Matrix.fromArray([[1, 0], [0, -1]]);
        } else if (axis === 'y') {
            return Matrix.fromArray([[-1, 0], [0, 1]]);
        } else if (axis === 'origin') {
            return Matrix.fromArray([[-1, 0], [0, -1]]);
        }
        throw new Error('未知的反射轴');
    }
}

// 导出
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { Matrix };
}
