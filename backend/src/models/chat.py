from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid

class Source(BaseModel):
    content: str
    score: float
    source: str

class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None
    context_restrict: bool = False
    max_results: int = 5
    similarity_threshold: float = 0.7

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    session_id: str
    query_time: float

class EmbeddingsRequest(BaseModel):
    text: str
    input_type: str = Field(default="document", pattern="^(query|document)$")

class EmbeddingsResponse(BaseModel):
    embeddings: List[float]
    text: str
    model: str