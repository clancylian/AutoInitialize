老大，你这个 BTC（**Binary Triangle Context**）代码我看了一遍，这个实现实际上就是论文里的 **BTC + STDesc** 初始化定位方案。它的流程基本是：

> 点云 → 体素 → 平面提取 → Binary Corner → Triangle(STD) → 数据库检索 → 姿态估计 → ICP验证

真正影响效果的参数几乎全部集中在 `read_parameters()` 中。

下面我按照整个算法流程，把每个参数为什么存在、调大会怎样、调小会怎样、适合什么场景，都详细讲一遍。

---

# 一、整体流程对应参数

```
输入点云
      │
      ▼
Voxel划分
      │
      ├──────── voxel_size
      ├──────── voxel_init_num
      └──────── plane_detection_thre
      ▼
平面检测
      │
      ├──────── plane_merge_normal_thre
      ├──────── plane_merge_dis_thre
      └──────── proj_plane_num
      ▼
Binary Descriptor提取
      │
      ├──────── proj_image_resolution
      ├──────── proj_image_high_inc
      ├──────── proj_dis_min/max
      ├──────── summary_min_thre
      ├──────── useful_corner_num
      ├──────── line_filter_enable
      ├──────── touch_filter_enable
      └──────── non_max_suppression_radius
      ▼
STD生成
      │
      ├──────── descriptor_near_num
      ├──────── descriptor_min_len
      ├──────── descriptor_max_len
      └──────── std_side_resolution
      ▼
数据库匹配
      │
      ├──────── skip_near_num
      ├──────── rough_dis_threshold
      ├──────── similarity_threshold
      ├──────── candidate_num
      ▼
姿态验证
      │
      ├──────── icp_threshold
      ├──────── normal_threshold
      └──────── dis_threshold
```

---

# 二、Voxel相关参数

---

## 1. voxel_size

默认

```
0.5m
```

作用：

把点云划成Voxel。

代码：

```cpp
loc_xyz[j] = p_c[j] / voxel_size_;
```

就是说

```
0.5m
```

代表

```
50cm 一个Voxel
```

影响：

### 调大

例如

```
1m
```

优点：

* 更快
* Plane更稳定
* 噪声减少

缺点：

* 小结构消失
* Corner数量下降

适合

```
户外
高速机器人
```

---

### 调小

例如

```
0.2m
```

优点：

保留细节

缺点：

Plane碎片很多

所以

```
0.3~0.5
```

一般最好。

---

## 2. voxel_init_num

默认

```
10
```

意思：

一个Voxel至少

```
10个点
```

才开始拟合Plane。

代码：

```cpp
if(voxel_points_.size()>voxel_init_num_)
```

---

调大

例如20

Plane更可靠

但是Plane数量下降。

---

调小

例如5

Plane很多

但是误检很多。

---

一般：

16线雷达：

```
5~8
```

32线：

```
8~12
```

64线：

```
10~20
```

---

# 三、Plane检测参数

---

## plane_detection_thre

默认

```
0.01
```

这是最重要参数之一。

代码：

```cpp
if(evalsReal(evalsMin)<plane_detection_thre_)
```

即

最小特征值

必须足够小。

数学意义：

```
λ3 << λ2 << λ1
```

说明

像平面。

---

调小

```
0.005
```

Plane更严格。

优点：

误检少。

缺点：

Plane数量减少。

---

调大

```
0.03
```

Plane增多。

但是：

很多墙角、

树叶、

杂乱点

都会变Plane。

---

推荐

室内：

```
0.005~0.01
```

室外：

```
0.02
```

---

## plane_merge_normal_thre

默认

```
0.1
```

控制

法向夹角。

代码

```cpp
normal_diff.norm()
```

实际上

约等于

```
cosθ
```

---

越小

只允许

方向几乎一致。

---

越大

很多Plane合并。

推荐：

```
0.1
```

不要超过

```
0.3
```

---

## plane_merge_dis_thre

默认

```
0.3m
```

两个Plane距离。

超过

```
30cm
```

不合并。

---

太大

不同墙会合并。

太小

同一墙碎裂。

推荐：

```
0.3~0.5
```

---

## proj_plane_num

默认

```
2
```

Binary只投影几个Plane。

Plane越多：

Corner越多。

速度越慢。

一般：

```
1~3
```

---

# 四、Binary Descriptor参数

---

## proj_image_resolution

默认

```
0.5m
```

Binary图像分辨率。

越小

Binary更细。

速度下降。

推荐：

```
0.2~0.5
```

---

## proj_image_high_inc

默认

```
0.1m
```

这是BTC最大的特色。

它不是灰度图。

而是

```
距离Plane的高度Histogram
```

例如：

```
0~5m
```

每

```
0.1m
```

一层。

Binary就是：

```
00011101010...
```

---

调小

```
0.05m
```

Binary长度翻倍。

更精细。

噪声增加。

---

调大

```
0.2m
```

Binary变短。

鲁棒。

但区分能力下降。

---

## proj_dis_min

默认

```
0
```

---

## proj_dis_max

默认

```
5m
```

就是说：

距离Plane超过5米

不用。

一般

```
3~8m
```

即可。

---

## summary_min_thre

默认

```
10
```

这是Corner强度。

代码：

```cpp
summary_=segmnt_dis
```

意思：

有多少高度层

被占据。

越大

说明结构越复杂。

---

调大

Corner减少。

更稳定。

---

调小

Corner增多。

误检增加。

推荐：

```
8~15
```

---

## useful_corner_num

默认

```
100
```

最后保留多少Corner。

这是速度和精度的重要参数。

100

已经比较合理。

如果地图特别大：

可以

```
200
```

---

# 五、Corner过滤

---

## line_filter_enable

默认

```
1
```

是否删除

线状Corner。

例如：

```
████████
```

走廊。

很多Corner都在线上。

容易误匹配。

建议：

一直开。

---

## touch_filter_enable

默认

```
0
```

要求Corner必须接触Plane。

一般不开。

---

## non_max_suppression_radius

默认

```
2m
```

NMS。

半径内

只保留最大Corner。

---

太小

重复Corner。

太大

Corner过少。

推荐：

```
1~2m
```

---

# 六、STD参数

---

## descriptor_near_num

默认

```
20
```

每个Corner

找20邻居。

组合Triangle。

Triangle数量：

约

```
20²
```

增长很快。

所以：

20已经很多。

---

## descriptor_min_len

默认

```
2m
```

边长太小

不要。

否则

容易受噪声影响。

---

## descriptor_max_len

默认

```
50m
```

太长

不要。

否则

跨建筑。

意义不大。

---

## std_side_resolution

默认

```
0.2m
```

Triangle边长量化。

例如

```
3.25m

↓

16
```

数据库检索靠它。

越小

区分能力越强。

数据库越大。

推荐：

```
0.2
```

---

# 七、数据库匹配

---

## skip_near_num

默认

```
30帧
```

避免：

刚刚经过的位置

立即闭环。

初始化定位一般影响不大。

---

## candidate_num

默认

```
20
```

候选地图数量。

越大：

召回增加。

速度下降。

一般：

```
20~50
```

---

## rough_dis_threshold

默认

```
0.01
```

Triangle边长误差：

允许

```
1%
```

例如

```
10m

允许

10cm
```

---

## similarity_threshold

默认

```
0.85
```

Binary相似度。

这是影响召回率的核心参数。

Binary计算：

```
2×交集
-----------
A+B
```

属于 Dice Similarity。

---

调大

```
0.9
```

误匹配少。

漏检增加。

---

调小

```
0.7
```

召回增加。

误匹配增加。

推荐：

```
0.8~0.9
```

---

# 八、最终验证

---

## icp_threshold

默认

```
0.15
```

不是ICP。

实际上是：

```
Plane几何一致率
```

如果

```
15%
```

Plane匹配成功。

认为定位成功。

---

## normal_threshold

默认

```
0.2
```

Plane法向差。

越小

要求越严格。

推荐：

```
0.15~0.25
```

---

## dis_threshold

默认

```
0.5m
```

Plane点到Plane距离。

越小

定位更准。

越容易失败。

---

# 九、如果是你的场景（轮式机器狗 + SuperLIO 初始化定位）

结合你之前介绍的场景（二维地图、激光雷达初始化定位，希望将 BTC 作为初始化模块），我会优先调整以下几个参数，而不是全部一起改：

| 优先级   | 参数                           | 建议调整            | 原因                          |
| ----- | ---------------------------- | --------------- | --------------------------- |
| ★★★★★ | `similarity_threshold`       | 0.85 → 0.75~0.8 | 提高首次定位召回率，尤其在环境有变化时。        |
| ★★★★★ | `summary_min_thre`           | 10 → 8          | 保留更多 Binary Corner，增加可匹配特征。 |
| ★★★★☆ | `useful_corner_num`          | 100 → 150~200   | 增加描述子数量，提高大场景定位成功率。         |
| ★★★★☆ | `descriptor_near_num`        | 20 → 25         | 生成更多稳定三角形，但注意计算量会上升。        |
| ★★★☆☆ | `plane_detection_thre`       | 0.01 → 0.008    | 更严格筛选平面，减少动态物体和噪声带来的误检。     |
| ★★★☆☆ | `candidate_num`              | 20 → 50         | 扩大候选检索范围，提升复杂地图中的召回概率。      |
| ★★☆☆☆ | `non_max_suppression_radius` | 2 → 1.5         | 在狭窄走廊等结构重复区域保留更多角点。         |

从你的代码实现来看，这套 BTC 参数更偏向于**普通闭环检测**的默认配置。如果你的目标是**机器人开机后的全局初始化定位**，我还建议进一步修改两部分核心逻辑：一是让三角描述子对旋转和局部地图变化更鲁棒；二是在候选筛选阶段加入类似 Scan Context 的全局粗检索，再由 BTC 精匹配，这样定位成功率通常会明显高于单独使用 BTC。这也是当前许多最新初始化定位系统常采用的组合方案。
