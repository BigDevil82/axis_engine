# HTTP API

`design_api` exposes the existing four business actions through FastAPI. CAD plugins remain responsible for reading the source drawing, rendering results, and writing engineer edits back to CAD.

## Start the service

```powershell
uv run python -m design_api.web.server --host 0.0.0.0 --port 8000
```

For development, append `--reload`. The interactive OpenAPI document is available at `http://<server>:8000/docs`, and the machine-readable schema is at `/openapi.json`.

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
  -Uri http://127.0.0.1:8000/api/v1/structure/normalize `
  -ContentType application/json `
  -Body $body
```

Invalid JSON fields are returned as HTTP `422`. Valid requests that cannot be processed by the geometric engine are returned as HTTP `400` with the diagnostic message. The service does not read, write, or control CAD.
