# 结构设计交互需求包

本目录用于与 UI/CAD 插件开发团队对接，当前包含：

1. 业务概览和系统边界。
2. 用户交互流程和关键页面线框。
3. 真实案例演示模板。
4. 接口格式示例。
5. 样例图纸和结果图。

建议阅读顺序：

- [01_business_overview.md](01_business_overview.md)
- [02_user_workflow.md](02_user_workflow.md)
- [03_ui_wireframes.md](03_ui_wireframes.md)
- [04_sample_case_storyboard.md](04_sample_case_storyboard.md)
- [05_api_examples.md](05_api_examples.md)
- [06_sample_cases](06_sample_cases)
- [07_shear_wall_plugin_ui.md](07_shear_wall_plugin_ui.md)

业务接口文档见 [./API.md](./API.md)。

概念图素材见 `assets/`：

- `cad-plugin-two-stage-ui-concept.png`：骨架检查和结构检查双阶段界面概念。
- `cad-plugin-workflow-concept.png`：从图层输入到写回 CAD 的流程概念。
- `shear-wall-plugin-ui-workflow.svg`：剪力墙智能设计选项卡和完整流程参考图。
