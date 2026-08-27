# Demo API Test

Export API input from the real drawing:

```powershell
uv run python tmp/demo_api/export_input.py
```

Run all public API stages against the deployed service and save JSON responses and PNG renderings:

```powershell
uv run python tmp/demo_api/run_pipeline.py
```

The default source layers are `WALL`, `WINDOW`, and `DOTE`; pass `--wall-layer`, `--opening-layer`, or `--axis-layer` repeatedly to override them during export. API responses and renderings are written to `tmp/demo_api/output`.
