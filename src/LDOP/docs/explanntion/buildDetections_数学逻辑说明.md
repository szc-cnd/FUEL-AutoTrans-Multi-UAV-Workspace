# buildDetections 数学逻辑说明

本文把 `DynamicObjectClusterer::buildDetections` 的实现过程转写为数学步骤，便于从“代码细节”切换到“算法视角”。

## 0. 代码执行路线

读代码时可以先把 `DynamicObjectClusterer` 拆成两层：

1. `processDynamicObjects()` 是模块级入口，只负责计时、调用 `buildDetections()`、再调用 `buildObjectMarkers()`。
2. `buildDetections()` 才是检测生成主流程；下面各数学章节基本按它的代码顺序展开。

`buildDetections()` 的执行顺序是：

```text
dynamic_points
  -> 按 UFOMap voxel key 分桶 buckets / bucket_by_key
  -> 按 key 稳定排序 sorted_bucket_indices
  -> 生成 6/18/26 连通偏移 offsets
  -> 对未访问桶做 BFS，得到一个当前帧连通块 cluster_points
  -> 点数过滤 min_points
  -> 统计 Fragment(min/max/sum/count)
  -> 用 AABB 对角线过滤 max_extent
  -> mergeFragmentedDetections() 合并上下断裂 fragment
  -> fragmentToDetection() 生成 DynamicObjectDetection
```

几个局部变量的角色也可以先记住：

1. `buckets` 保存“非空体素桶”，每个桶只存同一个 voxel key 下的点索引。
2. `bucket_by_key` 是从 voxel key 到桶下标的查找表，BFS 找邻居时靠它快速判断邻居桶是否存在。
3. `cluster_points` 是一次 BFS 收集到的原始点下标集合。
4. `Fragment` 是 detection 的半成品；合并前后都保留 AABB 边界和点云质心所需的 `sum/count`。

## 1. 输入与符号

设当前帧动态点集合为：

$$
\mathcal{P}=\{p_i\}_{i=1}^{N}
$$

每个点包含两类信息：

1. 空间坐标：$p_i=(x_i,y_i,z_i)$
2. 体素键：$k_i=(k_i^x,k_i^y,k_i^z,d_i)$

其中 $d_i$ 是八叉树 depth（层级）。在本实现中，键相等定义为四元组完全相等：

$$
k_a = k_b \iff (k_a^x,k_a^y,k_a^z,d_a)=(k_b^x,k_b^y,k_b^z,d_b)
$$

## 2. 按键分桶

先把点按体素键聚合，构造映射：

$$
\phi: k \mapsto B_k
$$

其中 $B_k$ 是属于键 $k$ 的点索引集合：

$$
B_k = \{i\mid k_i = k\}
$$

这一步等价于把原始点集压缩成“非空体素桶集合”。

## 3. 稳定排序

对桶索引按键做字典序排序：

$$
(d, k^x, k^y, k^z)
$$

即先比 depth，再比 $x/y/z$。数学上不改变聚类结果，只改变遍历顺序。工程目的：

1. 提高结果可复现性
2. 减少检测 ID 的抖动

## 4. 构建 key 空间图与邻接规则

把每个非空桶看作图节点 $v_k$。若两个键在连通规则下是邻居，则在它们之间连边。

给定当前键 $k=(k^x,k^y,k^z,d)$，同层步长为：

$$
s = 2^d
$$

邻居候选通过偏移 $\Delta=(\Delta_x,\Delta_y,\Delta_z)$ 生成：

$$
k'=(k^x+\Delta_x s,\;k^y+\Delta_y s,\;k^z+\Delta_z s,\;d)
$$

其中偏移集合由 connectivity 决定：

1. 6 连通：只允许单轴变化
2. 18 连通：允许最多两轴同时变化
3. 26 连通：允许三轴变化

注意：负向平移会做下溢保护（无符号 key 不能减到负数）。

## 5. BFS 连通域聚类

在上述图上执行 BFS，得到若干连通分量：

$$
\mathcal{C}_1,\mathcal{C}_2,\dots,\mathcal{C}_m
$$

每个连通分量对应一个候选目标，其点索引集合为桶索引并集：

$$
I(\mathcal{C}) = \bigcup_{k\in\mathcal{C}} B_k
$$

## 6. 点数阈值过滤

若某连通分量点数不足最小阈值 $n_{\min}$，则视为噪声并丢弃：

$$
|I(\mathcal{C})| < n_{\min} \Rightarrow \text{discard}
$$

## 7. Fragment 统计量

对每个保留分量，先不直接生成 `DynamicObjectDetection`，而是生成内部 `Fragment` 统计量。`Fragment` 同时保存真实 AABB 边界、点坐标和与点数：

$$
x_{\min}=\min_{i\in I(\mathcal{C})} x_i,\quad x_{\max}=\max_{i\in I(\mathcal{C})} x_i
$$
$$
y_{\min}=\min_{i\in I(\mathcal{C})} y_i,\quad y_{\max}=\max_{i\in I(\mathcal{C})} y_i
$$
$$
z_{\min}=\min_{i\in I(\mathcal{C})} z_i,\quad z_{\max}=\max_{i\in I(\mathcal{C})} z_i
$$

AABB 尺寸：

$$
\text{size}_x=x_{\max}-x_{\min},\;\text{size}_y=y_{\max}-y_{\min},\;\text{size}_z=z_{\max}-z_{\min}
$$

点坐标和与点数：

$$
S_x=\sum_{i\in I(\mathcal{C})}x_i,\quad
S_y=\sum_{i\in I(\mathcal{C})}y_i,\quad
S_z=\sum_{i\in I(\mathcal{C})}z_i,\quad
n=|I(\mathcal{C})|
$$

这里要特别区分两个概念：

1. `Fragment.min/max` 表示真实 AABB 边界，用于尺度过滤和后续上下碎片合并判断。
2. 最终 `bbox.center` 表示点云质心，不是 AABB 几何中心；后续 tracker 把它作为位置观测。

## 8. 尺寸上限过滤

用 AABB 对角线长度做尺度约束：

$$
L = \sqrt{\text{size}_x^2 + \text{size}_y^2 + \text{size}_z^2}
$$

若启用最大尺度阈值 $E_{\max}$，则：

$$
L > E_{\max} \Rightarrow \text{discard}
$$

通过过滤的连通分量才进入 fragment 列表。这样可以先剔除异常大的初始 cluster，再做后续碎片合并。

## 9. 垂直碎片合并

初始 fragment 之间会做一轮当前帧内的后处理，用于修复远处目标被上下切成多个 cluster 的情况。合并只针对垂直方向断裂，不跨水平空隙连接。

给定两个 fragment $a,b$，先计算它们的 z 向整体跨度：

$$
H_z=\max(z^a_{\max},z^b_{\max})-\min(z^a_{\min},z^b_{\min})
$$

若：

$$
H_z \le \max(\text{size}^a_z,\text{size}^b_z)
$$

则说明两个 fragment 的高度范围没有形成明确的上下堆叠关系，更像同高度并排目标，不合并。

再计算 z 向空隙：

$$
g_z=\max(z^a_{\min}-z^b_{\max},\;z^b_{\min}-z^a_{\max},\;0)
$$

只有满足：

$$
g_z \le g_{z,\max}
$$

才认为两个 fragment 在竖直方向足够接近。这里 $g_{z,\max}$ 对应参数：

```yaml
dynamic_cluster_vertical_merge_max_z_gap
```

水平投影必须有足够面积重叠。xy 交叠面积为：

$$
w_x=\max(0,\min(x^a_{\max},x^b_{\max})-\max(x^a_{\min},x^b_{\min}))
$$

$$
w_y=\max(0,\min(y^a_{\max},y^b_{\max})-\max(y^a_{\min},y^b_{\min}))
$$

$$
A_{\cap}=w_xw_y
$$

较小水平投影面积为：

$$
A_{\min}=\min(\text{size}^a_x\text{size}^a_y,\;\text{size}^b_x\text{size}^b_y)
$$

合并要求：

$$
A_{\cap}>0,\quad A_{\min}>0,\quad \frac{A_{\cap}}{A_{\min}}\ge \rho_{xy}
$$

其中 $\rho_{xy}$ 对应参数：

```yaml
dynamic_cluster_vertical_merge_min_xy_overlap_ratio
```

当前实现已经去掉 xy gap 合并：如果两个 fragment 的水平投影没有面积重叠，即使它们在 xy 平面距离很近，也不会合并。

最后还会检查合并后的 AABB 对角线是否超过 $E_{\max}$。若超过，说明这次合并会产生过大的目标框，也不合并。

合并本身是对统计量做累加：

$$
x_{\min}^{new}=\min(x_{\min}^a,x_{\min}^b),\quad
x_{\max}^{new}=\max(x_{\max}^a,x_{\max}^b)
$$

y/z 方向同理，并且：

$$
S^{new}=S^a+S^b,\quad n^{new}=n^a+n^b
$$

代码会反复扫描 fragment 列表，合并第一对满足条件的上下碎片，然后重新扫描。这样上/中/下多段断裂也可以逐步合成一个 fragment。

## 10. 输出检测对象

对合并后的每个 fragment，统一生成一个检测：

1. `detection_id`：顺序分配
2. `bbox.center`：由点云质心给出，作为跟踪阶段的位置观测
3. `bbox.size`：由 fragment 的真实 AABB 尺寸给出，用于尺度表达和可视化
4. `bbox.yaw = 0`
5. `stamp`：当前帧时间

质心计算为：

$$
\text{center}=\frac{1}{n}(S_x,S_y,S_z)^T
$$

尺寸计算为：

$$
\text{size}_x=x_{\max}-x_{\min},\quad
\text{size}_y=y_{\max}-y_{\min},\quad
\text{size}_z=z_{\max}-z_{\min}
$$

最终输出：

$$
\mathcal{D}=\{d_j\}_{j=1}^{M}
$$

其中 $M$ 为本帧有效目标数。

## 11. 一句话总结

该函数可以概括为：

1. 点云在体素键空间离散聚合
2. 在键图上做连通域分割
3. 对每个连通域生成 `Fragment(min/max/sum/count)`
4. 通过点数和尺度阈值过滤初始 fragment
5. 合并满足竖向间隙和水平投影重叠条件的上下碎片
6. 最后把 fragment 转成检测框，`bbox.center` 保持点云质心语义
