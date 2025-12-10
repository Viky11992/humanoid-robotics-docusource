from fastapi import HTTPException, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from starlette.status import HTTP_403_FORBIDDEN
import os
from dotenv import load_dotenv

load_dotenv()

class APIKeyAuth:
    def __init__(self):
        self.api_key = os.getenv("API_KEY")
        self.security = HTTPBearer(auto_error=False)

    async def __call__(self, request: Request):
        # Extract the API key from the Authorization header
        credentials: HTTPAuthorizationCredentials = await self.security(request)

        if not credentials or credentials.credentials != self.api_key:
            raise HTTPException(
                status_code=HTTP_403_FORBIDDEN,
                detail="Could not validate credentials"
            )

        return credentials.credentials


# Alternative implementation as a middleware class
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
import json

class AuthMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        # Skip auth for certain endpoints (like health checks)
        if request.url.path in ["/", "/health", "/docs", "/redoc"]:
            return await call_next(request)

        # Extract the API key from the x-api-key header
        api_key = request.headers.get("x-api-key")
        expected_api_key = os.getenv("API_KEY")

        if not api_key or api_key != expected_api_key:
            return Response(
                status_code=403,
                content=json.dumps({"detail": "Could not validate credentials"}),
                media_type="application/json"
            )

        response = await call_next(request)
        return response