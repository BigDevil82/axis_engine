"""Versioned HTTP routes that delegate to the pure business interfaces."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException

from design_api import design_structure, extract_skeleton, normalize_skeleton, normalize_structure
from design_api.web.schemas import (
    DesignStructureRequest,
    ExtractSkeletonRequest,
    NormalizeSkeletonRequest,
    NormalizeStructureRequest,
    SkeletonResponse,
    StructureResponse,
)

router = APIRouter(prefix="/api/v1", tags=["design"])


@router.post("/skeleton/extract", response_model=SkeletonResponse)
def extract_skeleton_route(request: ExtractSkeletonRequest) -> dict:
    return _run(extract_skeleton, request.model_dump())


@router.post("/skeleton/normalize", response_model=SkeletonResponse)
def normalize_skeleton_route(request: NormalizeSkeletonRequest) -> dict:
    return _run(normalize_skeleton, request.model_dump())


@router.post("/structure/design", response_model=StructureResponse)
def design_structure_route(request: DesignStructureRequest) -> dict:
    return _run(design_structure, request.model_dump())


@router.post("/structure/normalize", response_model=StructureResponse)
def normalize_structure_route(request: NormalizeStructureRequest) -> dict:
    return _run(normalize_structure, request.model_dump())


def _run(action, payload: dict) -> dict:
    try:
        return action(payload)
    except (RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
