# 结构设计交互需求包

本目录用于与 UI/CAD 插件开发团队对接，当前包含：

1. 业务概览和系统边界。
2. 用户交互流程和关键页面线框。
3. 真实案例演示模板。
4. 接口格式示例。
5. 样例图纸和结果图。

业务接口文档见 [./API.md](./API.md)。

概念图素材见 `assets/`：

- `shear-wall-plugin-ui-workflow.svg`：剪力墙智能设计选项卡和完整流程参考图。
- `shear-wall-plugin-sequence.svg`：CAD 插件和业务后端之间的交互时序图。


## 整体流程

当前系统已经分成两层：

- CAD 插件/前端层：从图纸读取图层、显示结果、接受人工编辑、写回 CAD。
- 业务算法/API 层：只接收 JSON 几何数据，返回骨架或结构构件 JSON，不直接操作 CAD。

整体流程如下：

```text
CAD 图层原始图元
    -> 骨架提取
    -> 工程师检查、修改骨架
    -> 骨架校准
    -> 结构布置
    -> 工程师检查、修改剪力墙和梁
    -> 结构校准
    -> CAD 插件展示/写回
```

**1. 原始输入**

插件从三类图层读取“显示出来的原始图元”，并传入 API：

```json
{
  "wall_geometries": [],
  "opening_geometries": [],
  "axis_geometries": []
}
```

实际通常对应：

- `wall_geometries`：`WALL` 图层，墙体线、折线等。
- `opening_geometries`：`WINDOW` 图层，门窗原始线、圆弧、块内部图元等。
- `axis_geometries`：`DOTE` 图层，建筑轴网线。

每个图元的基本形式：

```json
{
  "layer": "WALL",
  "geom_type": "LINE",
  "params": {
    "start": [0, 0],
    "end": [3000, 0]
  },
  "block_path": []
}
```

支持的主要类型有 `LINE`、`ARC`、`CIRCLE`、`ELLIPSE`、多段线。`ARC` 保留圆心、半径、起终点，因此门扇圆弧无需在输入阶段退化成线段。

**2. 骨架提取**

接口：

```text
POST /api/v1/skeleton/extract
```


输出：

```json
{
  "wall_axes": [
    {
      "id": "wall_axis_0",
      "line": [[x1, y1], [x2, y2]],
      "thickness": 200
    }
  ],
  "opening_embedments": [
    {
      "id": "opening_embedment_0",
      "line": [[x1, y1], [x2, y2]],
      "opening_type": "door",
      "cluster_index": 0,
      "confidence": 1.0,
      "reason": "..."
    }
  ],
  "axis_lines": [
    {
      "id": "axis_line_0",
      "line": [[x1, y1], [x2, y2]]
    }
  ],
  "slab_regions": [],
  "isolated_points": [[x, y]],
  "diagnostics": {}
}
```

其中：

- `wall_axes`：最终墙体单线骨架，保留 `thickness`。
- `opening_embedments`：门、窗等洞口所在的嵌入线。
- `axis_lines`：用于后续校准与规整的参考轴网。
- `slab_regions`：骨架围合产生的楼板区域，可用于检查空间是否闭合。
- `isolated_points`：骨架线网中度数为 1 的悬垂端点坐标；前端应绘制为待修正标记，不应作为骨架对象回传。
- `diagnostics`：数量、墙厚候选值、聚类数量等排查数据。

**3. 人工修改后的骨架校准**

接口：

```text
POST /api/v1/skeleton/normalize
```

前端传回工程师编辑后的：

```json
{
  "wall_axes": [],
  "opening_embedments": [],
  "axis_lines": []
}
```


输出格式与骨架提取相同。此阶段的输出应作为结构设计的正式输入。

**4. 结构设计**

接口：

```text
POST /api/v1/structure/design
```

输入仍是确认后的骨架：

```json
{
  "wall_axes": [],
  "opening_embedments": [],
  "axis_lines": []
}
```


输出：

```json
{
  "shear_walls": [
    {
      "id": "shear_wall_0",
      "line": [[x1, y1], [x2, y2]],
      "thickness": 200,
      "source": "dominant_wall_thickness|..."
    }
  ],
  "beams": [
    {
      "id": "beam_0",
      "line": [[x1, y1], [x2, y2]],
      "kind": "perimeter",
      "reason": "...",
      "related_ids": []
    }
  ],
  "slab_regions": [],
  "diagnostics": {}
}
```

梁的 `kind` 包括：

- `perimeter`：外围边梁。
- `balcony`：阳台过梁。
- `coupling`：连梁。
- `slab_divider`：大楼板的划分梁。

**5. 人工修改后的结构校准**

接口：

```text
POST /api/v1/structure/normalize
```

前端提交工程师调整后的：

```json
{
  "shear_walls": [],
  "beams": []
}
```
