from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

class EmbeddingChunk(BaseModel):
    text: str
    embedding: List[float]
    source: str
    chunk_id: str
    section: Optional[str] = None
    page_number: Optional[int] = None
    metadata: Optional[dict] = None
    created_at: datetime = None

    def __init__(self, **data):
        super().__init__(**data)
        if self.created_at is None:
            self.created_at = datetime.utcnow()