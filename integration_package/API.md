# 结构设计业务接口

该目录只封装业务逻辑接口，不读取 CAD/DXF 文件，不写 CAD 图层，也不绑定 HTTP 框架。前端或 CAD 插件负责读取图层元素，将结构化几何传入这些接口；后端返回结构化结果，前端负责展示、编辑和写回 CAD。

当前对外函数：

- `design_api.extract_skeleton(payload)`
- `design_api.normalize_skeleton(payload)`
- `design_api.design_structure(payload)`
- `design_api.normalize_structure(payload)`

所有接口接收普通 `dict`，返回普通 `dict`，可直接被 FastAPI、桌面程序、CAD 插件桥接层包装。

## 参数策略

当前对接阶段，UI/CAD 插件只需要传递几何数据和人工编辑后的构件数据，不需要传算法 `options`。

- 业务后端使用默认参数运行提取、生成和校准流程。
- 本文档中的请求示例以 UI 当前必须传递的字段为主。
- `options` 仍由业务接口内部保留，后续如果需要开放少量工程参数，再设计明确的设置项和可公开参数列表。

## 交互流程

1. CAD 插件读取墙体、门窗、轴网图层基础图元。
2. 调用 `extract_skeleton` 提取建筑骨架。
3. 前端展示墙轴线、门窗嵌入线、楼板分区和孤立点，工程师人工修正。
4. 调用 `normalize_skeleton` 对人工修改后的骨架做对齐、拓扑修复、剪枝，并重新返回楼板分区和孤立点。
5. 调用 `design_structure` 基于确认后的骨架生成剪力墙和梁。
6. 前端展示结构设计结果，工程师人工修正。
7. 调用 `normalize_structure` 对修改后的剪力墙和梁做剪枝、延伸搭接，并重新返回楼板分区。

## 通用数据结构

### CAD 基础图元

```json
{
  "id": "optional_frontend_id",
  "layer": "WALL",
  "geom_type": "LINE",
  "params": {
    "start": [0, 0],
    "end": [3000, 0]
  },
  "block_path": []
}
```

常用 `geom_type`：

- `LINE`: `{"start": [x, y], "end": [x, y]}`
- `ARC`: `{"center": [x, y], "radius": r, "start_angle": deg, "end_angle": deg, "start": [x, y], "end": [x, y]}`
- `CIRCLE`: `{"center": [x, y], "radius": r}`
- `ELLIPSE`: `{"points": [[x, y], ...]}`

### 线

```json
[[0, 0], [3000, 0]]
```

目前核心算法主要处理水平/竖直线段。折线建议由前端拆成多条线段。

### 墙轴线

```json
{
  "id": "wall_axis_0",
  "line": [[0, 0], [3000, 0]],
  "thickness": 200
}
```

### 门窗嵌入线

```json
{
  "id": "opening_embedment_0",
  "line": [[1000, 0], [1900, 0]],
  "opening_type": "door",
  "cluster_index": 3,
  "confidence": 0.9,
  "reason": "arc_hinge"
}
```

`opening_type` 可为 `door / window / balcony / opening`。

### 楼板分区

骨架提取和结构设计都会返回 `slab_regions`，用于前端判断骨架或结构线网是否闭合。

```json
{
  "id": "slab_region_0",
  "polygon": {
    "exterior": [[0, 0], [3000, 0], [3000, 4000], [0, 4000], [0, 0]],
    "interiors": []
  },
  "inner_boundary": [[...]],
  "source_polygon_index": 0,
  "hole_index": 0,
  "area": 12000000
}
```

### 孤立点

`isolated_points` 是骨架线网中度数为 1 的端点坐标，即未连接成闭合区域的悬垂端点。骨架提取和骨架校正都会返回该字段，CAD 插件应在图中标识这些位置，供工程师补线或调整连接关系。

```json
[[0, 0], [3000, 4000]]
```

### 剪力墙

```json
{
  "id": "shear_wall_0",
  "line": [[0, 0], [3000, 0]],
  "thickness": 200,
  "source": "dominant_wall_thickness|layout_adjusted"
}
```

### 梁

```json
{
  "id": "beam_0",
  "line": [[0, 0], [3000, 0]],
  "kind": "perimeter",
  "reason": "slab_footprint_outer_contour_minus_shear_wall",
  "related_ids": []
}
```

`kind` 可为：

- `perimeter`: 外轮廓边梁
- `balcony`: 阳台过梁
- `coupling`: 连梁
- `slab_divider`: 板划分梁

## API

### `extract_skeleton(payload)`

从前端传入的原始图元提取建筑骨架。

请求：

```json
{
  "wall_geometries": [],
  "opening_geometries": [],
  "axis_geometries": []
}
```

响应：

```json
{
  "wall_axes": [],
  "opening_embedments": [],
  "axis_lines": [],
  "slab_regions": [],
  "isolated_points": [],
  "diagnostics": {
    "wall_axis_count": 0,
    "opening_embedment_count": 0,
    "axis_line_count": 0,
    "slab_region_count": 0,
    "isolated_point_count": 0,
    "wall_geometry_count": 0,
    "opening_geometry_count": 0,
    "axis_geometry_count": 0,
    "opening_cluster_count": 0,
    "wall_thicknesses": []
  }
}
```

### `normalize_skeleton(payload)`

接收人工修改后的骨架，重新对齐、拓扑修复、剪枝，并返回楼板分区。

请求：

```json
{
  "wall_axes": [],
  "opening_embedments": [],
  "axis_lines": []
}
```

响应同 `extract_skeleton`。

### `design_structure(payload)`

基于确认后的建筑骨架生成剪力墙和梁。

请求：

```json
{
  "wall_axes": [],
  "opening_embedments": [],
  "axis_lines": []
}
```

响应：

```json
{
  "shear_walls": [],
  "beams": [],
  "slab_regions": [],
  "diagnostics": {
    "source": "generated",
    "dominant_wall_thickness": 200,
    "shear_wall_count": 0,
    "beam_count": 0,
    "beam_counts": {},
    "slab_region_count": 0
  }
}
```

### `normalize_structure(payload)`

接收人工修改后的剪力墙和梁，做剪枝、悬垂梁延伸搭接，并重新生成楼板分区。

请求：

```json
{
  "shear_walls": [],
  "beams": []
}
```

响应同 `design_structure`，其中 `diagnostics.source` 为 `normalized`。

## 设计原则

- 前端负责 CAD 读取、图层管理、人工编辑、写回 CAD。
- 后端接口只接收结构化几何数据，只返回结构化业务结果。
- 骨架提取和结构设计都提供 normalize 接口，用于人工修改后的再校正。
- `slab_regions` 是闭合性检查的重要反馈，骨架阶段和结构阶段都返回。
