"""FastAPI application factory and deployable ASGI application."""

from __future__ import annotations

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from design_api.web.routes import router


def create_app() -> FastAPI:
    app = FastAPI(
        title="Axis Engine Design API",
        version="0.1.0",
        description="Building skeleton extraction and structural layout services.",
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.include_router(router)

    @app.get("/health", tags=["health"])
    def health() -> dict[str, str]:
        return {"status": "ok"}

    return app


app = create_app()
