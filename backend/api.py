from fastapi import APIRouter, Depends
from typing import List, Optional
import os
import sys
import importlib

# Add the current directory to the path
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.append(current_dir)

from models import AgentAskRequest, AgentAskResponse
from middleware import APIKeyAuth
from rag_tool import rag_tool
from logging_service import log_query

# Create API router
agent_router = APIRouter()

# Initialize API key auth
api_key_auth = APIKeyAuth()

from fastapi import HTTPException

@agent_router.post("/agent/ask", response_model=AgentAskResponse)
async def agent_ask(
    request: AgentAskRequest,
    api_key: str = Depends(api_key_auth)
):
    """
    Handle user questions with optional selected text as context.

    Args:
        request: AgentAskRequest containing question and optional selected_text
        api_key: Authenticated API key (automatically validated)

    Returns:
        AgentAskResponse with answer and sources
    """
    question = request.question

    # Validate input parameters
    if not question or not question.strip():
        raise HTTPException(
            status_code=400,
            detail="Question is required and cannot be empty"
        )

    # Limit question length to prevent abuse
    if len(question) > 2000:
        raise HTTPException(
            status_code=400,
            detail="Question is too long (max 2000 characters)"
        )

    if request.selected_text and len(request.selected_text) > 10000:
        raise HTTPException(
            status_code=400,
            detail="Selected text is too long (max 10000 characters)"
        )

    selected_text = request.selected_text

    try:
        # Use the RAG tool to synthesize the answer
        result = rag_tool.synthesize_answer(
            question=question,
            selected_text=selected_text
        )

        # Log the query and response to Neon Postgres
        try:
            log_query(
                question=question,
                answer=result["answer"],
                sources=result["sources"]
            )
        except Exception as e:
            # Log the error but don't fail the request
            print(f"Error logging query: {e}")

        return AgentAskResponse(
            answer=result["answer"],
            sources=result["sources"]
        )
    except Exception as e:
        # Log the error
        print(f"Error processing agent ask request: {e}")

        # Log failed query attempt
        try:
            log_query(
                question=question,
                answer=f"Error processing request: {str(e)}",
                sources=[],
                user_id="system"
            )
        except:
            pass  # Don't fail the response if logging fails

        raise HTTPException(
            status_code=500,
            detail="Error processing your request"
        )

# Additional endpoints will be added in later phases