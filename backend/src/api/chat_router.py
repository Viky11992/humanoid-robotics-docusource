from fastapi import APIRouter, Depends, HTTPException, status
from typing import Dict, Any
import time
import logging
from src.models.chat import ChatRequest, ChatResponse
from src.middleware.auth import api_key_auth, rate_limiter
from src.services.rag_service import rag_service

logger = logging.getLogger(__name__)

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    auth: bool = Depends(api_key_auth)
) -> ChatResponse:
    """
    Chat endpoint that processes user queries using RAG methodology
    """
    # Rate limiting check
    client_id = "default_client"  # In a real app, this would be the actual client identifier
    if not rate_limiter.is_allowed(client_id):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded"
        )

    try:
        # Process the query using RAG service
        result = await rag_service.process_query(
            query=request.query,
            session_id=request.session_id,
            selected_text=request.selected_text,
            context_restrict=request.context_restrict,
            max_results=request.max_results,
            similarity_threshold=request.similarity_threshold
        )

        # Format the response
        response = ChatResponse(
            response=result["response"],
            sources=result["sources"],
            session_id=result["session_id"],
            query_time=result["query_time"]
        )

        return response

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )

@router.post("/embeddings")
async def embeddings_endpoint(
    request: Dict[str, Any],
    auth: bool = Depends(api_key_auth)
):
    """
    Generate embeddings for text content used in the RAG system
    """
    # Rate limiting check
    client_id = "default_client"
    if not rate_limiter.is_allowed(client_id):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded"
        )

    try:
        text = request.get("text")
        input_type = request.get("input_type", "document")

        if not text:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Text is required"
            )

        # Generate embeddings using Cohere service
        from src.services.cohere_service import cohere_service
        embeddings = cohere_service.generate_embeddings([text], input_type=input_type)

        return {
            "embeddings": embeddings[0] if embeddings else [],
            "text": text,
            "model": cohere_service.embed_model
        }

    except Exception as e:
        logger.error(f"Error generating embeddings: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )