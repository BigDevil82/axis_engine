从住宅建筑平面图的 DXF 文件读取墙体、门窗图层元素：提取墙轴线、推断门窗嵌入线，完成骨架化表示与房间识别/分割。

## 当前实现边界

- **仅支持 DXF 输入**（已移除 JSON 读取与 JSON 绘图入口）。
- 主编排入口：`axis_engine\cad_processor.py` 中 `CADLayoutProcessor`。

## 核心流程

1. `build_geometry()`：读取 DXF，提取墙线、墙轴线、门窗聚类与嵌入线。
2. `collect_network_segments()`：将墙/门/窗语义线统一为校准输入。
3. `generate_rooms()`：线网校准后 polygonize + 矩形分解，得到房间结果。

> 调用顺序要求：必须先执行 `build_geometry()`，再调用后续步骤。若墙线/轴线/房间为空，处理器会抛出明确异常而不是静默继续。

## 最小验证脚本（cad_tests）

- `plot_wall_axes_from_dxf.py`：墙轴线提取验证。
- `plot_windows_from_dxf.py`：门窗聚类与嵌入线验证。
- `plot_constraint_calibration.py`：约束校准验证。

## 结构说明（重构后）

- `cad_processor.py`：流程编排与状态管理。
- `room_generation.py`：房间生成服务（校准 + polygonize + 矩形分解）。
