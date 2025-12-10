# Implementation Plan for RAG Chatbot (OpenAI Agent SDK + FastAPI + Qdrant + Neon)

## Phase 1 — Project Setup
- Initialize project folder.
- Install dependencies:
  - openai
  - openai-agent-sdk
  - fastapi
  - qdrant-client
  - psycopg
  - uvicorn
  - python-dotenv
  - pytest (for unit and integration testing)
- Create project folders:
  - /specs
  - /backend
  - /agent
  - /rag
  - /frontend

## Phase 2 — OpenAI Agent SDK Setup
- Create RAG Agent structure in Python.
- Implement and register core tools:
  - Text embedding tool (using `text-embedding-3-large`)
  - Vector search tool (for Qdrant)
  - Metadata lookup tool (for Neon)
  - RAG answer synthesis tool (grounded in provided context)
- Configure agent:
  - model: "gpt-4.1"
  - system prompt: "You are an AI assistant. Answer questions strictly based on the provided book content. If the answer is not found in the context, state that clearly. Do not use external knowledge."

## Phase 3 — FastAPI Backend
- Develop API endpoints:
  - POST `/agent/ask`:
    - Accepts `question` (string) and `selected_text` (string, optional).
    - Orchestrates call to OpenAI Agent.
    - Ensures `x-api-key` authentication is enforced.
    - Returns `answer` (string) and `sources` (list of strings).
  - POST `/rag/ingest`:
    - Accepts `text` (string) - the full book content.
    - Processes text for ingestion (chunking, embedding).
    - Stores embeddings in Qdrant and metadata in Neon.
    - Returns `status` (string).
  - POST `/rag/search`:
    - Accepts `query` (string).
    - Performs search in Qdrant.
    - Returns search `results` (id, text, score).
- Implement security middleware for `x-api-key` enforcement on all endpoints.

## Phase 4 — Qdrant Vector Store Setup
- Set up Qdrant collection with appropriate vector configuration.
- Implement text chunking and embedding script using `text-embedding-3-large`.
- Ingest book text, store chunks and embeddings in Qdrant.
- Store associated metadata (e.g., chunk ID, source file/page) in Neon Postgres.

## Phase 5 — RAG Logic (Agent Tools)
- **Embedding Tool**: Handles text chunking and embedding generation.
- **Vector Search Tool**: Queries Qdrant for relevant chunks based on input query or selected text.
- **Metadata Lookup Tool**: Retrieves metadata from Neon based on Qdrant search results.
- **RAG Answer Tool**:
    - Combines retrieved chunks from Qdrant and any provided `selected_text`.
    - Sends the combined context and user question to the OpenAI Agent for a grounded answer.
    - Formats the final answer and identified sources.

## Phase 6 — ChatKit UI Frontend
- Integrate ChatKit SDK.
- Build the following UI components:
  - Chat window for user interaction.
  - Feature to allow users to select text from the book content.
  - Viewer to display source chunks used for the answer.
- Connect frontend to the `/agent/ask` API endpoint.

## Phase 7 — Deployment
- Deploy FastAPI backend → Cloud Run / Render (serverless functions).
- Host Qdrant → Qdrant Cloud (Free Tier).
- Use Neon Postgres → Serverless Postgres instance.
- Agent logic → Managed by OpenAI Agent SDK.
- Deploy Frontend → Vercel / Netlify.

## Phase 8 — Testing and Validation
- **Unit Tests**:
  - Framework: `pytest`
  - Scope: Embeddings generation, Qdrant vector search functions, agent answer tool logic.
  - Target Coverage: 80%
- **Integration Tests**:
  - Framework: `pytest`
  - Scope: Full RAG pipeline (API endpoints, Qdrant interactions, Neon DB logging, Agent orchestration).
  - Target Coverage: 75%
- **End-to-End Tests**:
  - Framework: Playwright / Selenium (if ChatKit doesn't provide E2E testing tools)
  - Scope: Simulating user interaction via ChatKit UI, validating responses and sources.
  - Target Coverage: 70%
- **Performance Testing**:
  - Use tools like `locust` or `k6` to test the `/agent/ask` endpoint under load.
  - Measure p95 response times and ensure they are under 2 seconds.
- **Security Testing**:
  - Validate `x-api-key` enforcement on all API endpoints.
  - Perform basic checks for common web vulnerabilities (e.g., OWASP Top 10).
