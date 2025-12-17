from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid

class Session(BaseModel):
    id: str
    user_id: Optional[str] = None
    created_at: datetime = None
    updated_at: datetime = None
    metadata: Optional[Dict[str, Any]] = None

    def __init__(self, **data):
        super().__init__(**data)
        if self.created_at is None:
            self.created_at = datetime.utcnow()
        if self.updated_at is None:
            self.updated_at = datetime.utcnow()
        if self.id is None:
            self.id = str(uuid.uuid4())

class Conversation(BaseModel):
    id: str
    session_id: str
    query: str
    response: str
    selected_text: Optional[str] = None
    context_used: Optional[List[str]] = None
    sources: Optional[List[Dict[str, Any]]] = None
    created_at: datetime = None
    metadata: Optional[Dict[str, Any]] = None

    def __init__(self, **data):
        super().__init__(**data)
        if self.created_at is None:
            self.created_at = datetime.utcnow()
        if self.id is None:
            self.id = str(uuid.uuid4())