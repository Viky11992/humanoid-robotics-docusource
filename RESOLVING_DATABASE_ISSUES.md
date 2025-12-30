# Resolving Database Connection Issues for RAG Chatbot

## Problem
The RAG chatbot is returning "500 Internal Server Error" when trying to process queries. This is happening because:

1. The PostgreSQL service requires a database connection (NEON_DATABASE_URL)
2. The provided database URL may not be accessible or credentials may have expired
3. The RAG service tries to create sessions and store conversations in the database

## Solutions

### Solution 1: Set up a local PostgreSQL database (Recommended)

1. Install PostgreSQL locally on your system
2. Create a database for the application
3. Update the backend/.env file with your local database URL:

```
NEON_DATABASE_URL=postgresql://username:password@localhost:5432/rag_chatbot_db
```

### Solution 2: Use a free PostgreSQL service

Sign up for a free PostgreSQL service like:
- Supabase (https://supabase.com/)
- Aiven PostgreSQL (free tier)
- Render PostgreSQL (free tier)

Then update the NEON_DATABASE_URL in the .env file with the provided connection string.

### Solution 3: Update the RAG Service (Advanced)

The RAG service could be modified to handle database failures gracefully by:
- Creating a mock database service for development
- Implementing fallback behavior when database is unavailable
- Making database storage optional for basic functionality

### Solution 4: Quick Test with Context Restriction

For immediate testing without database issues, you can modify the RAG service temporarily to skip database operations by using context restriction mode which bypasses the database calls:

1. In the agent router, provide a session ID to avoid session creation
2. Use selected_text with context_restrict=True to bypass vector search

## Verification Steps

After setting up a database:

1. Start the backend:
```bash
cd backend
uvicorn src.main:app --host 127.0.0.1 --port 8000
```

2. Test the health endpoint:
```bash
curl http://127.0.0.1:8000/api/v1/health
```

3. Test the agent endpoint:
```bash
curl -X POST http://127.0.0.1:8000/api/v1/agent/ask \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU" \
  -d '{"question": "What is Physical AI?", "selected_text": null, "context_restrict": false}'
```

## Troubleshooting

If you still get errors:

1. Check that PostgreSQL is running locally
2. Verify the database URL in .env is correct
3. Ensure the database user has necessary permissions
4. Check that required tables are created (the service should create them automatically on startup)

## Environment Setup

Make sure your backend/.env file has all required variables:
- COHERE_API_KEY (required for embeddings and responses)
- QDRANT_URL (for vector storage, with fallback)
- QDRANT_API_KEY (for vector storage)
- NEON_DATABASE_URL (for conversation storage)
- GENERAL_API_KEY (for authentication)