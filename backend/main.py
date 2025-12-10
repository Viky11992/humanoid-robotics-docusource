from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Import local modules
import sys
import os
sys.path.append(os.path.dirname(__file__))

from qdrant_config import get_qdrant_client
from qdrant_operations import QdrantOperations
from middleware import AuthMiddleware

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot API for book content",
    version="0.1.0"
)

# Add middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add authentication middleware
app.add_middleware(AuthMiddleware)

# Initialize Qdrant client and operations
qdrant_client = get_qdrant_client()
qdrant_ops = QdrantOperations(qdrant_client)

@app.on_event("startup")
async def startup_event():
    """Initialize Qdrant collection on startup"""
    qdrant_ops.create_collection()
    print("Qdrant collection initialized")

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Include API routes
import api
import rag_api
app.include_router(api.agent_router)
app.include_router(rag_api.rag_router)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)