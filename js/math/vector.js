/**
 * 向量运算库
 * 提供2D和3D向量的基本运算
 */

class Vector2D {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  // 克隆向量
  clone() {
    return new Vector2D(this.x, this.y);
  }

  // 向量加法
  add(v) {
    return new Vector2D(this.x + v.x, this.y + v.y);
  }

  // 向量减法
  subtract(v) {
    return new Vector2D(this.x - v.x, this.y - v.y);
  }

  // 数乘
  scale(scalar) {
    return new Vector2D(this.x * scalar, this.y * scalar);
  }

  // 点积
  dot(v) {
    return this.x * v.x + this.y * v.y;
  }

  // 叉积（返回标量，表示z分量）
  cross(v) {
    return this.x * v.y - this.y * v.x;
  }

  // 模长
  magnitude() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }

  // 归一化
  normalize() {
    const mag = this.magnitude();
    if (mag === 0) return new Vector2D(0, 0);
    return new Vector2D(this.x / mag, this.y / mag);
  }

  // 旋转（弧度）
  rotate(angle) {
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    return new Vector2D(
      this.x * cos - this.y * sin,
      this.x * sin + this.y * cos
    );
  }

  // 向量夹角
  angleTo(v) {
    const dot = this.dot(v);
    const mags = this.magnitude() * v.magnitude();
    if (mags === 0) return 0;
    return Math.acos(Math.max(-1, Math.min(1, dot / mags)));
  }

  // 投影到另一个向量
  projectOnto(v) {
    const vMagSq = v.dot(v);
    if (vMagSq === 0) return new Vector2D(0, 0);
    const scalar = this.dot(v) / vMagSq;
    return v.scale(scalar);
  }

  // 转换为数组
  toArray() {
    return [this.x, this.y];
  }

  // 从数组创建
  static fromArray(arr) {
    return new Vector2D(arr[0] || 0, arr[1] || 0);
  }

  // 零向量
  static zero() {
    return new Vector2D(0, 0);
  }

  // 单位向量
  static unit(angle) {
    return new Vector2D(Math.cos(angle), Math.sin(angle));
  }

  // 线性插值
  static lerp(v1, v2, t) {
    return new Vector2D(
      v1.x + (v2.x - v1.x) * t,
      v1.y + (v2.y - v1.y) * t
    );
  }
}

class Vector3D {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  clone() {
    return new Vector3D(this.x, this.y, this.z);
  }

  add(v) {
    return new Vector3D(this.x + v.x, this.y + v.y, this.z + v.z);
  }

  subtract(v) {
    return new Vector3D(this.x - v.x, this.y - v.y, this.z - v.z);
  }

  scale(scalar) {
    return new Vector3D(this.x * scalar, this.y * scalar, this.z * scalar);
  }

  dot(v) {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }

  cross(v) {
    return new Vector3D(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x
    );
  }

  magnitude() {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }

  normalize() {
    const mag = this.magnitude();
    if (mag === 0) return new Vector3D(0, 0, 0);
    return new Vector3D(this.x / mag, this.y / mag, this.z / mag);
  }

  toArray() {
    return [this.x, this.y, this.z];
  }

  static fromArray(arr) {
    return new Vector3D(arr[0] || 0, arr[1] || 0, arr[2] || 0);
  }

  static zero() {
    return new Vector3D(0, 0, 0);
  }

  static lerp(v1, v2, t) {
    return new Vector3D(
      v1.x + (v2.x - v1.x) * t,
      v1.y + (v2.y - v1.y) * t,
      v1.z + (v2.z - v1.z) * t
    );
  }
}

// 导出
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { Vector2D, Vector3D };
}
