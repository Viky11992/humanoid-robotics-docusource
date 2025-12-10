from fastapi import APIRouter, Depends, HTTPException
from typing import List
from pydantic import BaseModel
import sys
import os

# Add the current directory to the path
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.append(current_dir)

from middleware import APIKeyAuth
from qdrant_operations import QdrantOperations
from qdrant_config import get_qdrant_client
from embedding_generator import embedding_generator
from text_chunker import text_chunker
from database import get_db_connection

# Create API router
rag_router = APIRouter()

# Initialize API key auth
api_key_auth = APIKeyAuth()

# Initialize Qdrant operations
qdrant_client = get_qdrant_client()
qdrant_ops = QdrantOperations(qdrant_client)


class RagIngestRequestModel(BaseModel):
    text: str

class RagIngestResponseModel(BaseModel):
    status: str

class RagSearchRequestModel(BaseModel):
    query: str

class SearchResultModel(BaseModel):
    id: str
    text: str
    score: float

class RagSearchResponseModel(BaseModel):
    results: List[SearchResultModel]


@rag_router.post("/rag/ingest", response_model=RagIngestResponseModel)
async def rag_ingest(
    request: RagIngestRequestModel,
    api_key: str = Depends(api_key_auth)
):
    """
    Ingest book text, generate embeddings, and store in Qdrant and Neon Postgres.

    Args:
        request: Request containing the full book text
        api_key: Authenticated API key (automatically validated)

    Returns:
        Status of the ingestion process
    """
    text = request.text

    if not text or not text.strip():
        raise HTTPException(
            status_code=400,
            detail="Text is required and cannot be empty"
        )

    try:
        # Chunk the text
        chunks = text_chunker.chunk_text(text)

        if not chunks:
            raise HTTPException(
                status_code=400,
                detail="Could not chunk the provided text"
            )

        # Extract text from chunks for embedding
        chunk_texts = [chunk["text"] for chunk in chunks]

        # Generate embeddings
        embeddings = embedding_generator.generate_embeddings(chunk_texts)

        # Prepare data for Qdrant
        metadata_list = []
        for i, chunk in enumerate(chunks):
            metadata_list.append({
                "text": chunk["text"],
                "start_idx": chunk.get("start_idx", 0),
                "end_idx": chunk.get("end_idx", 0),
                "chunk_index": i
            })

        # Add embeddings to Qdrant
        ids = qdrant_ops.add_embeddings(chunk_texts, metadata_list)

        # Store metadata in Neon Postgres
        conn = get_db_connection()
        try:
            with conn.cursor() as cursor:
                # Insert chunks into the database
                for i, (chunk_id, chunk_data) in enumerate(zip(ids, chunks)):
                    query = """
                        INSERT INTO text_chunks (chunk_id, chunk_text, page_number, section_title)
                        VALUES (%s, %s, %s, %s)
                        RETURNING id;
                    """
                    # For now, setting page_number and section_title to None
                    # In a real implementation, these would come from the book metadata
                    cursor.execute(query, (
                        chunk_id,
                        chunk_data["text"],
                        chunk_data.get("page_number"),
                        chunk_data.get("section_title")
                    ))
                conn.commit()
        except Exception as e:
            conn.rollback()
            raise e
        finally:
            conn.close()

        return RagIngestResponseModel(status="success")
    except Exception as e:
        print(f"Error during ingestion: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error during ingestion: {str(e)}"
        )


@rag_router.post("/rag/search", response_model=RagSearchResponseModel)
async def rag_search(
    request: RagSearchRequestModel,
    api_key: str = Depends(api_key_auth)
):
    """
    Search in Qdrant for relevant chunks based on the query.

    Args:
        request: Request containing the search query
        api_key: Authenticated API key (automatically validated)

    Returns:
        Search results with id, text, and score
    """
    query = request.query

    if not query or not query.strip():
        raise HTTPException(
            status_code=400,
            detail="Query is required and cannot be empty"
        )

    if len(query) > 2000:
        raise HTTPException(
            status_code=400,
            detail="Query is too long (max 2000 characters)"
        )

    try:
        # Import vector search tool
        from .vector_search_tool import vector_search_tool

        # Perform vector search
        results = vector_search_tool.search(query, limit=10)

        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append(SearchResultModel(
                id=result["id"],
                text=result["text"],
                score=result["score"]
            ))

        return RagSearchResponseModel(results=formatted_results)
    except Exception as e:
        print(f"Error during search: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error during search: {str(e)}"
        )