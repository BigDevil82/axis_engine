# HTTP API

`design_api` exposes the existing four business actions through FastAPI. CAD plugins remain responsible for reading the source drawing, rendering results, and writing engineer edits back to CAD.

## Start the service

```powershell
uv run python -m design_api.web.server --host 0.0.0.0 --port 10187
```

For development, append `--reload`. The interactive OpenAPI document is available at `http://<server>:10187/docs`, and the machine-readable schema is at `/openapi.json`.

## Endpoints

| Method | Path | Business action |
| --- | --- | --- |
| `GET` | `/health` | Service health check |
| `POST` | `/api/v1/skeleton/extract` | Extract wall axes, opening embedments, and slab regions from raw CAD primitives |
| `POST` | `/api/v1/skeleton/normalize` | Calibrate an engineer-edited skeleton and regenerate slab regions |
| `POST` | `/api/v1/structure/design` | Generate shear walls, beams, and structural slab regions from a confirmed skeleton |
| `POST` | `/api/v1/structure/normalize` | Calibrate an engineer-edited structural layout and regenerate slab regions |

Request and response fields are defined in the OpenAPI document. The business-level field definitions and examples remain in [integration_package/API.md](../integration_package/API.md) and [integration_package/05_api_examples.md](../integration_package/05_api_examples.md).

## Example

```powershell
$body = @{
  shear_walls = @()
  beams = @()
} | ConvertTo-Json

Invoke-RestMethod `
  -Method Post `
  -Uri http://127.0.0.1:10187/api/v1/structure/normalize `
  -ContentType application/json `
  -Body $body
```

Invalid JSON fields are returned as HTTP `422`. Valid requests that cannot be processed by the geometric engine are returned as HTTP `400` with the diagnostic message. The service does not read, write, or control CAD.

## 宝塔 Linux 部署

在宝塔的 Python 项目中选择 **命令行启动**，以便明确使用 ASGI worker：

| 字段 | 设置 |
| --- | --- |
| 项目名称 | `axis-engine-api` |
| Python 环境 | Python `3.12` 或更高版本 |
| 项目路径 | 项目根目录，例如 `/www/wwwroot/axis-engine` |
| 启动用户 | `www` |
| 安装依赖包路径 | `<项目路径>/requirements.txt` |
| 启动命令 | `python -m gunicorn --workers 2 --worker-class uvicorn.workers.UvicornWorker --bind 127.0.0.1:10187 --timeout 300 design_api.web.app:app` |

宝塔 Python 环境应先安装 `requirements.txt` 中的依赖。不要使用 Python 3.10，因为项目要求 Python 3.12 以上。Gunicorn 仅适用于 Linux；本地 Windows 调试继续使用 `uv run python -m design_api.web.server`。

推荐通过宝塔的网站反向代理对外提供服务：上游地址填写 `http://127.0.0.1:10187`，域名通过 HTTPS 访问 `<域名>/docs` 和 `<域名>/api/v1/...`。服务当前没有身份认证，不应直接把 `10187` 暴露到公网；如 CAD 插件必须直连，则仅在受控内网开放该端口。
