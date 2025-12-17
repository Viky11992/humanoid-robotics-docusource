import asyncpg
import os
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
import logging
import json
from datetime import datetime
import uuid

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class PostgresService:
    def __init__(self):
        self.database_url = os.getenv("NEON_DATABASE_URL")
        if not self.database_url:
            raise ValueError("NEON_DATABASE_URL environment variable is required")

        self.pool = None

    async def initialize(self):
        """Initialize the connection pool"""
        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.database_url,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            await self._create_tables()
            logger.info("PostgreSQL service initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing PostgreSQL service: {str(e)}")
            raise e

    async def _create_tables(self):
        """Create required tables if they don't exist"""
        async with self.pool.acquire() as conn:
            # Create sessions table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS sessions (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    user_id VARCHAR(255),
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    metadata JSONB
                )
            """)

            # Create conversations table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS conversations (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    session_id UUID REFERENCES sessions(id),
                    query TEXT NOT NULL,
                    response TEXT NOT NULL,
                    selected_text TEXT,
                    context_used TEXT[],
                    sources JSONB,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    metadata JSONB
                )
            """)

            # Create documents table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS documents (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    title VARCHAR(500) NOT NULL,
                    source_url VARCHAR(1000),
                    content_hash VARCHAR(64),
                    chunk_count INTEGER,
                    word_count INTEGER,
                    metadata JSONB,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            # Create users table (if needed for advanced features)
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS users (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    email VARCHAR(255) UNIQUE,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    preferences JSONB
                )
            """)

            # Create indexes
            await conn.execute("CREATE INDEX IF NOT EXISTS idx_sessions_created_at ON sessions(created_at)")
            await conn.execute("CREATE INDEX IF NOT EXISTS idx_conversations_session_id ON conversations(session_id)")
            await conn.execute("CREATE INDEX IF NOT EXISTS idx_conversations_created_at ON conversations(created_at)")
            await conn.execute("CREATE INDEX IF NOT EXISTS idx_documents_content_hash ON documents(content_hash)")

            logger.info("Tables created successfully")

    async def create_session(self, user_id: Optional[str] = None, metadata: Optional[Dict] = None) -> str:
        """Create a new session"""
        async with self.pool.acquire() as conn:
            session_id = str(uuid.uuid4())
            await conn.execute("""
                INSERT INTO sessions (id, user_id, metadata)
                VALUES ($1, $2, $3)
            """, session_id, user_id, json.dumps(metadata) if metadata else None)
            return session_id

    async def get_session(self, session_id: str) -> Optional[Dict]:
        """Get session by ID"""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow("""
                SELECT id, user_id, created_at, updated_at, metadata
                FROM sessions
                WHERE id = $1
            """, session_id)

            if row:
                return {
                    "id": str(row["id"]),
                    "user_id": row["user_id"],
                    "created_at": row["created_at"],
                    "updated_at": row["updated_at"],
                    "metadata": row["metadata"]
                }
            return None

    async def create_conversation(self, session_id: str, query: str, response: str,
                                selected_text: Optional[str] = None,
                                context_used: Optional[List[str]] = None,
                                sources: Optional[List[Dict]] = None,
                                metadata: Optional[Dict] = None) -> str:
        """Create a new conversation record"""
        async with self.pool.acquire() as conn:
            conversation_id = str(uuid.uuid4())
            await conn.execute("""
                INSERT INTO conversations (id, session_id, query, response, selected_text, context_used, sources, metadata)
                VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            """, conversation_id, session_id, query, response,
                selected_text, context_used,
                json.dumps(sources) if sources else None,
                json.dumps(metadata) if metadata else None)
            return conversation_id

    async def get_conversations_by_session(self, session_id: str, limit: int = 10) -> List[Dict]:
        """Get conversations for a session"""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch("""
                SELECT id, session_id, query, response, selected_text, context_used, sources, created_at
                FROM conversations
                WHERE session_id = $1
                ORDER BY created_at DESC
                LIMIT $2
            """, session_id, limit)

            conversations = []
            for row in rows:
                conversations.append({
                    "id": str(row["id"]),
                    "session_id": str(row["session_id"]),
                    "query": row["query"],
                    "response": row["response"],
                    "selected_text": row["selected_text"],
                    "context_used": row["context_used"],
                    "sources": row["sources"],
                    "created_at": row["created_at"]
                })
            return conversations

    async def create_document(self, title: str, source_url: Optional[str] = None,
                            content_hash: Optional[str] = None,
                            chunk_count: Optional[int] = None,
                            word_count: Optional[int] = None,
                            metadata: Optional[Dict] = None) -> str:
        """Create a new document record"""
        async with self.pool.acquire() as conn:
            document_id = str(uuid.uuid4())
            await conn.execute("""
                INSERT INTO documents (id, title, source_url, content_hash, chunk_count, word_count, metadata)
                VALUES ($1, $2, $3, $4, $5, $6, $7)
            """, document_id, title, source_url, content_hash, chunk_count, word_count,
                json.dumps(metadata) if metadata else None)
            return document_id

    async def get_document_by_hash(self, content_hash: str) -> Optional[Dict]:
        """Get document by content hash"""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow("""
                SELECT id, title, source_url, chunk_count, word_count, metadata, created_at
                FROM documents
                WHERE content_hash = $1
            """, content_hash)

            if row:
                return {
                    "id": str(row["id"]),
                    "title": row["title"],
                    "source_url": row["source_url"],
                    "chunk_count": row["chunk_count"],
                    "word_count": row["word_count"],
                    "metadata": row["metadata"],
                    "created_at": row["created_at"]
                }
            return None

    async def close(self):
        """Close the connection pool"""
        if self.pool:
            await self.pool.close()
            logger.info("PostgreSQL connection pool closed")

# Global instance
postgres_service = PostgresService()