from fastapi import APIRouter, Depends, HTTPException, status, Request
from typing import Dict, Any, Optional
import time
import logging
from src.models.chat import ChatRequest, ChatResponse
from src.middleware.auth import api_key_auth, rate_limiter
from src.services.rag_service import rag_service

logger = logging.getLogger(__name__)

router = APIRouter()

@router.post("/agent/ask")
async def agent_ask_endpoint(
    request: Request
) -> Dict[str, Any]:
    """
    Agent endpoint that processes user queries using RAG methodology
    This endpoint is designed to match the expected interface for Docusaurus integration
    """
    # Extract API key from Authorization header (Bearer token format)
    auth_header = request.headers.get("authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header with Bearer token required"
        )

    api_key = auth_header.split(" ")[1]

    # Verify the API key using the auth middleware function
    if not api_key_auth(api_key):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key"
        )

    # Rate limiting check
    client_id = "default_client"  # In a real app, this would be the actual client identifier
    if not rate_limiter.is_allowed(client_id):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded"
        )

    try:
        # Get JSON body from request
        body = await request.json()

        # Extract question and optional selected text from request
        question = body.get("question")
        selected_text = body.get("selected_text", None)
        context_restrict = body.get("context_restrict", bool(selected_text))

        if not question:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Question is required"
            )

        # Process the query using RAG service
        result = await rag_service.process_query(
            query=question,
            session_id=None,  # Generate new session for each agent request
            selected_text=selected_text,
            context_restrict=context_restrict,
            max_results=5,
            similarity_threshold=0.7
        )

        # Format response to match expected agent interface
        response = {
            "answer": result["response"],
            "sources": [source.dict() for source in result["sources"]],
            "session_id": result["session_id"],
            "query_time": result["query_time"]
        }

        return response

    except Exception as e:
        logger.error(f"Error processing agent request: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )