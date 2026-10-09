# 接口格式示例

本文件只说明 UI/CAD 插件当前需要传递的核心字段和响应格式，不绑定某一个真实案例。完整字段含义见 [API.md](API.md)。

## 当前参数策略

- 当前阶段 UI 不需要向业务接口传算法 `options`。
- 业务后端使用默认参数完成骨架提取、结构生成和校准。
- 后续如果工程师确实需要控制核心参数，再单独设计设置项和可公开参数白名单。
- 因此下面示例不包含 `options`。

## 调用顺序

```text
CAD 图层图元
    -> extract_skeleton
    -> 骨架编辑
    -> normalize_skeleton
    -> design_structure
    -> 结构编辑
    -> normalize_structure
    -> 写回 CAD
```

## 1. 提取建筑骨架

### 请求

`extract_skeleton`

```json
{
  "wall_geometries": [
    {
      "id": "cad_wall_001",
      "layer": "WALL",
      "geom_type": "LINE",
      "params": {
        "start": [1000, 2000],
        "end": [5800, 2000]
      }
    }
  ],
  "opening_geometries": [
    {
      "id": "cad_opening_001",
      "layer": "WINDOW",
      "geom_type": "ARC",
      "params": {
        "center": [3200, 2000],
        "radius": 900,
        "start_angle": 0,
        "end_angle": 90,
        "start": [4100, 2000],
        "end": [3200, 2900]
      }
    }
  ],
  "axis_geometries": [
    {
      "id": "cad_axis_001",
      "layer": "DOTE",
      "geom_type": "LINE",
      "params": {
        "start": [1000, 0],
        "end": [1000, 8000]
      }
    }
  ]
}
```

### 响应

```json
{
  "wall_axes": [
    {
      "id": "wall_axis_0",
      "line": [[1000, 2100], [5800, 2100]],
      "thickness": 200
    }
  ],
  "opening_embedments": [
    {
      "id": "opening_embedment_0",
      "line": [[3200, 2100], [4100, 2100]],
      "opening_type": "door",
      "cluster_index": 0,
      "confidence": 0.9,
      "reason": "algorithm_reason"
    }
  ],
  "axis_lines": [
    {
      "id": "axis_line_0",
      "line": [[1000, 0], [1000, 8000]]
    }
  ],
  "slab_regions": [
    {
      "id": "slab_region_0",
      "polygon": {
        "exterior": [[1000, 2100], [5800, 2100], [5800, 6200], [1000, 6200], [1000, 2100]],
        "interiors": []
      },
      "inner_boundary": [[1150, 2250], [5650, 2250], [5650, 6050], [1150, 6050], [1150, 2250]],
      "source_polygon_index": 0,
      "hole_index": 0,
      "area": 19680000
    }
  ],
  "isolated_points": [],
  "diagnostics": {
    "wall_axis_count": 1,
    "opening_embedment_count": 1,
    "axis_line_count": 1,
    "slab_region_count": 1,
    "isolated_point_count": 0,
    "wall_geometry_count": 1,
    "opening_geometry_count": 1,
    "axis_geometry_count": 1,
    "opening_cluster_count": 1,
    "wall_thicknesses": [200]
  }
}
```

UI 应保存并显示骨架对象；`slab_regions` 主要用于闭合性检查和填色展示，`isolated_points` 应作为待修正端点标识。

## 2. 校准人工修改后的骨架

### 请求

`normalize_skeleton`

```json
{
  "wall_axes": [
    {
      "id": "wall_axis_manual_001",
      "line": [[1000, 2100], [5800, 2100]],
      "thickness": 200
    }
  ],
  "opening_embedments": [
    {
      "id": "opening_manual_001",
      "line": [[3200, 2100], [4100, 2100]],
      "opening_type": "door"
    }
  ],
  "axis_lines": [
    {
      "id": "axis_line_0",
      "line": [[1000, 0], [1000, 8000]]
    }
  ]
}
```

### 响应

响应结构与 `extract_skeleton` 相同：

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
    "isolated_point_count": 0
  }
}
```

## 3. 生成结构布置

### 请求

`design_structure`

```json
{
  "wall_axes": [
    {
      "id": "wall_axis_0",
      "line": [[1000, 2100], [5800, 2100]],
      "thickness": 200
    }
  ],
  "opening_embedments": [
    {
      "id": "opening_embedment_0",
      "line": [[3200, 2100], [4100, 2100]],
      "opening_type": "door"
    }
  ],
  "axis_lines": [
    {
      "id": "axis_line_0",
      "line": [[1000, 0], [1000, 8000]]
    }
  ]
}
```

### 响应

```json
{
  "shear_walls": [
    {
      "id": "shear_wall_0",
      "line": [[1000, 2100], [3600, 2100]],
      "thickness": 200,
      "source": "dominant_wall_thickness"
    }
  ],
  "beams": [
    {
      "id": "beam_0",
      "line": [[3600, 2100], [5800, 2100]],
      "kind": "coupling",
      "reason": "algorithm_reason",
      "related_ids": []
    }
  ],
  "slab_regions": [],
  "diagnostics": {
    "source": "generated",
    "dominant_wall_thickness": 200,
    "shear_wall_count": 1,
    "beam_count": 1,
    "beam_counts": {
      "coupling": 1
    },
    "slab_region_count": 0
  }
}
```

## 4. 校准人工修改后的结构

### 请求

`normalize_structure`

```json
{
  "shear_walls": [
    {
      "id": "shear_wall_manual_001",
      "line": [[1000, 2100], [3600, 2100]],
      "thickness": 200
    }
  ],
  "beams": [
    {
      "id": "beam_manual_001",
      "line": [[3600, 2100], [5800, 2100]],
      "kind": "coupling"
    }
  ]
}
```

### 响应

响应结构与 `design_structure` 相同：

```json
{
  "shear_walls": [],
  "beams": [],
  "slab_regions": [],
  "diagnostics": {
    "source": "normalized",
    "shear_wall_count": 0,
    "beam_count": 0,
    "beam_counts": {},
    "slab_region_count": 0
  }
}
```

## UI 当前必须保留的字段

为支持“自动生成 -> 人工编辑 -> 回传校准”，UI 至少应保留：

- 线坐标 `line`。
- 墙厚 `thickness`。
- 门窗嵌入线类型 `opening_type`。
- 梁类型 `kind`。
- 各对象 `id`，便于 UI 自己跟踪编辑对象。

算法输出中的 `reason`、`confidence`、`diagnostics` 可用于提示和排查，不应成为 UI 编辑的硬性前置条件。
