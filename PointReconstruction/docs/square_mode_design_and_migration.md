# Square Mode 设计与迁移说明

## 1) 为什么历史上有 4 种模式

历史代码按“中间几何构造方式”拆成 4 类内部算法（`eRoomSquare_type`）：

- `SQUARE_BY_ROOMCONTOUR`：直接从房间轮廓找方。
- `SQUARE_BY_CUBOID`：先从墙体顶点构造一个更规则的 cuboid-like 轮廓，再找方。
- `SQUARE_BY_CONVEXITY`：在 cuboid 基础上继续做凸点/缺陷相关优化。
- `SQUARE_BY_MIN_LOSS`：尝试多个起始参考并选最小损失结果。

当初这样设计是为了把“规则性”“凸点优化”“损失最小化”分开试验，便于迭代参数和效果对比。

## 2) 现在为什么建议只有 2 种模式

从产品使用角度，用户通常只关心两类目标：

1. **正则化找方（Regularized）**  
   有轴线就以轴线为基准；无轴线则回退到参考墙，结果更规整。

2. **贴合优先/面积优先（Fit-Max-Area）**  
   更强调贴合原始墙面与损失最小。

因此外部配置建议只暴露 `square_mode` 的两个值：

- `0`: `SQUARE_STRATEGY_REGULARIZED`
- `1`: `SQUARE_STRATEGY_FIT_MAX_AREA`

## 3) 向后兼容策略

为了兼容旧任务参数，保留 legacy 映射：

- `2 -> SQUARE_STRATEGY_REGULARIZED`
- `3 -> SQUARE_STRATEGY_FIT_MAX_AREA`

并在日志中打印 raw mode 与 mapped strategy，方便排查旧数据。

## 4) 现网建议

- 新项目只使用 `square_mode = 0/1`。
- 旧项目传 `2/3` 不会中断，但建议逐步清理为 `0/1`。
- 问题排查时优先看：
  - raw square mode
  - mapped strategy mode
  - 是否启用 axis (`square_by_axis`)
  - axis 配置是否有效拟合
