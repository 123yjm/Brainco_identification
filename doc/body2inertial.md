# Body → Inertial 参数转换规则

## 概述

`RigidBody` 存储的是物理直观参数（质量、质心位置、绕质心的惯量张量），而辨识所用的回归矩阵 $Y$ 要求参数向量 $\boldsymbol{\beta}$ 为**标准惯性参数**。`InertialParams::fromBody()` 完成这个转换。

## 输入：RigidBody

| 字段 | 符号 | 含义 |
|------|------|------|
| `mass` | $m$ | 刚体质量 |
| `com` | $\mathbf{c} = (c_x, c_y, c_z)$ | 质心在 Body 坐标系中的位置 |
| `Ixx, Iyy, Izz` | $\bar{I}_{xx}, \bar{I}_{yy}, \bar{I}_{zz}$ | 绕**质心**的主惯量 |
| `Ixy, Ixz, Iyz` | $\bar{I}_{xy}, \bar{I}_{xz}, \bar{I}_{yz}$ | 绕**质心**的惯量积 |

## 输出：InertialParams（10 个标准惯性参数）

| 参数 | 公式 | 含义 |
|------|------|------|
| $m$ | $m$ | 质量（不变） |
| $m_x$ | $m \cdot c_x$ | 一阶矩 x |
| $m_y$ | $m \cdot c_y$ | 一阶矩 y |
| $m_z$ | $m \cdot c_z$ | 一阶矩 z |
| $I_{xx}$ | $\bar{I}_{xx} + m(c_y^2 + c_z^2)$ | 绕 Body 原点的惯量 xx |
| $I_{xy}$ | $\bar{I}_{xy} - m \cdot c_x c_y$ | 绕 Body 原点的惯量积 xy |
| $I_{xz}$ | $\bar{I}_{xz} - m \cdot c_x c_z$ | 绕 Body 原点的惯量积 xz |
| $I_{yy}$ | $\bar{I}_{yy} + m(c_x^2 + c_z^2)$ | 绕 Body 原点的惯量 yy |
| $I_{yz}$ | $\bar{I}_{yz} - m \cdot c_y c_z$ | 绕 Body 原点的惯量积 yz |
| $I_{zz}$ | $\bar{I}_{zz} + m(c_x^2 + c_y^2)$ | 绕 Body 原点的惯量 zz |

## 核心：平行轴定理 (Parallel Axis Theorem / Steiner's Theorem)

惯量从**质心 C** 平移到 **Body 原点 O**，位移向量 $\mathbf{c} = (c_x, c_y, c_z)$：

$$\mathbf{I}_O = \mathbf{I}_C + m \big( \mathbf{c}^T \mathbf{c} \, \mathbf{1}_{3\times3} - \mathbf{c} \mathbf{c}^T \big)$$

展开为矩阵形式：

$$\begin{bmatrix}
c_y^2+c_z^2 & -c_x c_y & -c_x c_z \\
-c_x c_y & c_x^2+c_z^2 & -c_y c_z \\
-c_x c_z & -c_y c_z & c_x^2+c_y^2
\end{bmatrix}$$

## 为何需要此转换？

辨识方程为线性形式 $Y \boldsymbol{\beta} = \boldsymbol{\tau}$，其中回归矩阵 $Y$ 的列对应的正是上述 10 个标准惯性参数。直接使用质心惯量 $\bar{I}$ 会导致矩阵列不对齐，必须统一到 Body 原点坐标系。

## 辨识后的逆变换

辨识得到的 $\boldsymbol{\beta}$ 是标准惯性参数，如需还原物理参数：

| 物理量 | 逆推公式 |
|--------|----------|
| $m$ | $\beta_0$ |
| $c_x$ | $\beta_1 / \beta_0$ |
| $c_y$ | $\beta_2 / \beta_0$ |
| $c_z$ | $\beta_3 / \beta_0$ |
| $\bar{I}_{xx}$ | $\beta_4 - m(c_y^2 + c_z^2)$ |
| $\bar{I}_{xy}$ | $\beta_5 + m \cdot c_x c_y$ |
| … | …（同理逆推其余惯量分量） |

## 代码位置

- 头文件：[include/body2inertial.hpp](../include/body2inertial.hpp) — `InertialParams` 结构体定义
- 实现：[src/body2inertial.cpp](../src/body2inertial.cpp) — `InertialParams::fromBody()`
