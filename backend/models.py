from pydantic import BaseModel
from typing import List, Optional

class AgentAskRequest(BaseModel):
    """
    Request model for the /agent/ask endpoint.
    """
    question: str
    selected_text: Optional[str] = None

class AgentAskResponse(BaseModel):
    """
    Response model for the /agent/ask endpoint.
    """
    answer: str
    sources: List[str]