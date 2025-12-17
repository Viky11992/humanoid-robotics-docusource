from fastapi import HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class APIKeyAuth:
    def __init__(self):
        self.required_api_key = os.getenv("GENERAL_API_KEY")
        if not self.required_api_key:
            raise ValueError("GENERAL_API_KEY environment variable is required")
        self.security = HTTPBearer()

    async def __call__(self, request: Request) -> bool:
        credentials: HTTPAuthorizationCredentials = await self.security(request)
        if credentials.credentials != self.required_api_key:
            logger.warning(f"Unauthorized access attempt with key: {credentials.credentials[:8]}...")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid API key"
            )
        return True

# Initialize the auth middleware
api_key_auth = APIKeyAuth()

# Rate limiting implementation
from functools import wraps
from collections import defaultdict
import time

class RateLimiter:
    def __init__(self, requests: int = 100, window: int = 60):
        self.requests = requests
        self.window = window  # in seconds
        self.requests_log = defaultdict(list)

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if a request from the given identifier is allowed based on rate limits

        Args:
            identifier: Unique identifier for the requester (e.g., IP address or API key)

        Returns:
            True if request is allowed, False otherwise
        """
        now = time.time()
        # Clean old requests outside the window
        self.requests_log[identifier] = [
            req_time for req_time in self.requests_log[identifier]
            if now - req_time < self.window
        ]

        # Check if we've exceeded the limit
        if len(self.requests_log[identifier]) >= self.requests:
            return False

        # Add current request to log
        self.requests_log[identifier].append(now)
        return True

# Initialize rate limiter
rate_limiter = RateLimiter(requests=100, window=60)  # 100 requests per minute