from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os
import logging
from src.api.chat_router import router as chat_router
from src.api.health_router import router as health_router
from src.api.agent_router import router as agent_router
from src.services.postgres_service import postgres_service

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize the app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation Chatbot for book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "https://viky11992.github.io",  # GitHub Pages deployment
        "https://humanoid-robotics.vercel.app",  # Vercel deployment
        "https://*.vercel.app"  # Vercel deployment with any subdomain
    ],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
app.include_router(health_router, prefix="/api/v1", tags=["health"])
app.include_router(agent_router, prefix="/api/v1", tags=["agent"])

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    logger.info("Initializing PostgreSQL service...")
    await postgres_service.initialize()
    logger.info("PostgreSQL service initialized successfully")

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up services on shutdown"""
    logger.info("Closing PostgreSQL connection pool...")
    await postgres_service.close()
    logger.info("PostgreSQL connection pool closed")

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)