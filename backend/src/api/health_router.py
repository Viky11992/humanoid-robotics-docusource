from fastapi import APIRouter, Depends
from typing import Dict, Any
from src.middleware.auth import api_key_auth
import time
import asyncio

router = APIRouter()

@router.get("/health")
async def health_check():
    """
    Health check endpoint to verify service availability
    """
    # Check if external services are accessible
    services_status = {
        "cohere": False,
        "qdrant": False,
        "postgres": False
    }

    # Test Cohere connectivity
    try:
        from src.services.cohere_service import cohere_service
        # Just check if the service is initialized properly
        if hasattr(cohere_service, 'client'):
            services_status["cohere"] = True
    except Exception:
        services_status["cohere"] = False

    # Test Qdrant connectivity
    try:
        from src.services.qdrant_service import qdrant_service
        # Test collection exists
        if hasattr(qdrant_service, 'client'):
            collections = qdrant_service.client.get_collections()
            services_status["qdrant"] = True
    except Exception:
        services_status["qdrant"] = False

    # Test Postgres connectivity
    try:
        from src.services.postgres_service import postgres_service
        if postgres_service.pool:
            async with postgres_service.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
                services_status["postgres"] = True
    except Exception:
        services_status["postgres"] = False

    overall_status = "healthy" if all(services_status.values()) else "degraded"

    return {
        "status": overall_status,
        "services": services_status,
        "timestamp": time.time()
    }

@router.get("/ready")
async def readiness_check():
    """
    Readiness check endpoint to verify service is ready to accept traffic
    """
    # For now, just check the same services as health
    services_status = {
        "cohere": False,
        "qdrant": False,
        "postgres": False
    }

    # Test Cohere connectivity
    try:
        from src.services.cohere_service import cohere_service
        if hasattr(cohere_service, 'client'):
            services_status["cohere"] = True
    except Exception:
        services_status["cohere"] = False

    # Test Qdrant connectivity
    try:
        from src.services.qdrant_service import qdrant_service
        if hasattr(qdrant_service, 'client'):
            collections = qdrant_service.client.get_collections()
            services_status["qdrant"] = True
    except Exception:
        services_status["qdrant"] = False

    # Test Postgres connectivity
    try:
        from src.services.postgres_service import postgres_service
        if postgres_service.pool:
            async with postgres_service.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
                services_status["postgres"] = True
    except Exception:
        services_status["postgres"] = False

    overall_status = "ready" if all(services_status.values()) else "not ready"

    return {
        "status": overall_status,
        "services": services_status,
        "timestamp": time.time()
    }