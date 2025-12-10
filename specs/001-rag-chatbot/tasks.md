# Tasks for RAG Chatbot Integration

## Feature Overview
Retrieval-Augmented Generation chatbot integrated with OpenAI Agent SDK, FastAPI backend, Neon Postgres, and Qdrant vector database. The agent answers questions about the book, including questions based on user-selected text.

## Implementation Strategy
This implementation follows a phased approach, starting with project setup and foundational components, then implementing user stories in priority order. Each user story is designed to be independently testable and deliverable.

## Dependencies
User stories should be completed in priority order (P1 first). Some foundational components may be required before specific user stories can be implemented.

## Parallel Execution Opportunities
Tasks marked with [P] can be executed in parallel with other tasks from different files or non-dependent tasks.

---

## Phase 1: Setup
Setup tasks for project initialization and environment configuration.

### Goal
Initialize the project structure and install required dependencies.

### Independent Test Criteria
Project can be built and basic imports work without errors.

### Tasks
- [X] T001 Initialize project folder structure with backend, agent, rag, frontend directories
- [X] T002 Install dependencies: openai, openai-agent-sdk, fastapi, qdrant-client, psycopg, uvicorn, python-dotenv, pytest
- [X] T003 Create .env file with placeholder environment variables for API keys and connection strings
- [X] T004 Set up basic configuration files (pyproject.toml or requirements.txt)

---

## Phase 2: Foundational Components
Foundational tasks that block all user stories - must be completed before user story phases begin.

### Goal
Set up core infrastructure components required by all user stories.

### Independent Test Criteria
Core services can be initialized and basic operations performed.

### Tasks
- [X] T005 Set up Qdrant client connection with configuration from environment variables
- [X] T006 Set up Neon Postgres connection with configuration from environment variables
- [X] T007 Create Qdrant collection for storing book text embeddings with appropriate vector configuration
- [X] T008 Create database schema for storing book metadata, ingestion status, and query logs in Neon Postgres
- [X] T009 Implement security middleware for x-api-key authentication enforcement
- [X] T010 Create basic FastAPI application with proper configuration

---

## Phase 3: User Story 1 - Text Selection Context
**User Story**: User can select text from the book content and the chatbot uses that as context. (P1)

### Goal
Implement functionality to accept user-selected text and use it as context for the chatbot.

### Independent Test Criteria
Given selected text and a question, the system returns an answer based on the provided context.

### Tasks
- [X] T011 [US1] Create data models for the /agent/ask endpoint (question: string, selected_text: string optional)
- [X] T012 [US1] Create data models for the response (answer: string, sources: list of strings)
- [X] T013 [US1] Implement POST /agent/ask endpoint with authentication
- [X] T014 [US1] Create RAG answer synthesis tool that combines selected_text with question for OpenAI Agent
- [X] T015 [US1] Configure OpenAI Agent with system prompt: "Answer questions strictly based on the provided book content. If the answer is not found in the context, state that clearly. Do not use external knowledge."
- [X] T016 [US1] Implement logic to pass selected_text as context to the OpenAI Agent
- [X] T017 [US1] Add logging of user queries and agent responses to Neon Postgres
- [X] T018 [US1] Write unit tests for the /agent/ask endpoint with selected_text
- [X] T019 [US1] Write integration tests for the full flow with selected_text context

---

## Phase 4: User Story 2 - Vector Search Context
**User Story**: If no selected text is provided, chatbot uses vector search in Qdrant for context. (P1)

### Goal
Implement vector search functionality to find relevant book content when no selected text is provided.

### Independent Test Criteria
Given a question without selected text, the system performs vector search in Qdrant and returns an answer based on the retrieved context.

### Tasks
- [X] T020 [US2] Implement text chunking utility for processing book content
- [X] T021 [US2] Implement embedding generation using text-embedding-3-large
- [X] T022 [US2] Create vector search tool for querying Qdrant based on input query
- [X] T023 [US2] Implement metadata lookup tool to retrieve metadata from Neon based on Qdrant search results
- [X] T024 [US2] Update RAG answer synthesis tool to use vector search results when selected_text is not provided
- [X] T025 [US2] Modify /agent/ask endpoint to conditionally use vector search when no selected_text is provided
- [X] T026 [US2] Write unit tests for vector search functionality
- [X] T027 [US2] Write integration tests for the full flow without selected_text

---

## Phase 5: User Story 3 - Agent SDK Integration
**User Story**: Use OpenAI Agent SDK and ChatKit for question answering, ensuring answers are context-grounded. (P1)

### Goal
Integrate OpenAI Agent SDK to ensure answers are strictly grounded in provided context.

### Independent Test Criteria
The agent returns answers that are strictly based on provided context without using external knowledge.

### Tasks
- [X] T028 [US3] Create OpenAI Agent structure with proper configuration
- [X] T029 [US3] Register embedding tool in the OpenAI Agent
- [X] T030 [US3] Register vector search tool in the OpenAI Agent
- [X] T031 [US3] Register metadata lookup tool in the OpenAI Agent
- [X] T032 [US3] Register RAG answer synthesis tool in the OpenAI Agent
- [X] T033 [US3] Configure agent model to use "gpt-4.1"
- [X] T034 [US3] Ensure agent strictly follows system prompt to avoid external knowledge
- [X] T035 [US3] Write unit tests for agent tools
- [X] T036 [US3] Write integration tests for agent response quality

---

## Phase 6: User Story 4 - API Endpoints
**User Story**: Expose a POST /agent/ask endpoint for receiving user questions and optional selected_text. (P1)

### Goal
Complete the API endpoint implementation with proper request/response handling and authentication.

### Independent Test Criteria
The /agent/ask endpoint accepts questions and optional selected text, enforces authentication, and returns properly formatted responses.

### Tasks
- [X] T037 [US4] Complete implementation of POST /agent/ask endpoint with full error handling
- [X] T038 [US4] Ensure x-api-key authentication is enforced on the endpoint
- [X] T039 [US4] Validate input parameters (question required, selected_text optional)
- [X] T040 [US4] Format response with answer and sources as specified
- [X] T041 [US4] Add proper error responses for invalid requests
- [X] T042 [US4] Write comprehensive API tests for the /agent/ask endpoint
- [X] T043 [US4] Document API endpoint with OpenAPI/Swagger

---

## Phase 7: User Story 5 - Book Ingestion
**User Story**: Ingest book text, generate embeddings using text-embedding-3-large, and store chunks with embeddings in Qdrant and metadata in Neon Postgres. (P1)

### Goal
Implement the ingestion pipeline to process book content and store it in vector and metadata stores.

### Independent Test Criteria
Book text can be ingested, chunked, embedded, and stored in both Qdrant and Neon Postgres.

### Tasks
- [X] T044 [US5] Create POST /rag/ingest endpoint that accepts text input
- [X] T045 [US5] Implement text processing and chunking logic for book ingestion
- [X] T046 [US5] Generate embeddings for text chunks using text-embedding-3-large
- [X] T047 [US5] Store embeddings in Qdrant with appropriate metadata
- [X] T048 [US5] Store metadata in Neon Postgres with references to Qdrant vectors
- [X] T049 [US5] Return appropriate status response from /rag/ingest endpoint
- [X] T050 [US5] Implement ingestion status tracking in database
- [X] T051 [US5] Write unit tests for ingestion components
- [X] T052 [US5] Write integration tests for the full ingestion pipeline

---

## Phase 8: User Story 6 - Context-Grounded Answers
**User Story**: Agent must return strictly context-based answers, avoiding external knowledge. (P1)

### Goal
Ensure the agent strictly follows the system prompt to avoid using external knowledge.

### Independent Test Criteria
The agent refuses to answer questions that cannot be answered based on the provided context.

### Tasks
- [X] T053 [US6] Implement response validation to ensure answers are context-grounded
- [X] T054 [US6] Add logic to detect when answer is not found in context
- [X] T055 [US6] Ensure agent clearly states when answer is not available in context
- [X] T056 [US6] Test with questions that require external knowledge to ensure proper rejection
- [X] T057 [US6] Write tests to validate the agent's adherence to context-only responses
- [X] T058 [US6] Add response quality metrics to monitor grounding effectiveness

---

## Phase 9: User Story 7 - Query Logging
**User Story**: Log all user queries and agent responses in Neon Postgres. (P2)

### Goal
Implement comprehensive logging of user interactions for analytics and debugging.

### Independent Test Criteria
All queries and responses are logged in Neon Postgres with appropriate metadata.

### Tasks
- [X] T059 [US7] Define database schema for query logs in Neon Postgres
- [X] T060 [US7] Implement logging function to store queries and responses
- [X] T061 [US7] Add timestamp and metadata to query logs
- [X] T062 [US7] Ensure logging doesn't impact response time significantly
- [X] T063 [US7] Add error handling for logging failures
- [X] T064 [US7] Write tests for query logging functionality
- [X] T065 [US7] Implement log retrieval API for analytics (if needed)

---

## Phase 10: User Story 8 - Search Endpoint
**User Story**: Implement search functionality to allow direct access to vector search capabilities.

### Goal
Provide a direct search endpoint that allows clients to query the vector database.

### Independent Test Criteria
The /rag/search endpoint accepts queries and returns relevant results from Qdrant.

### Tasks
- [X] T066 [US8] Create POST /rag/search endpoint that accepts query input
- [X] T067 [US8] Implement vector search logic to query Qdrant
- [X] T068 [US8] Format search results with id, text, and score
- [X] T069 [US8] Add authentication to the search endpoint
- [X] T070 [US8] Write unit tests for the search functionality
- [X] T071 [US8] Write integration tests for the /rag/search endpoint

---

## Phase 11: Frontend Integration
Integrate ChatKit frontend for user interaction.

### Goal
Provide a user-friendly interface for interacting with the RAG chatbot.

### Independent Test Criteria
Users can interact with the chatbot through a web interface that connects to the backend.

### Tasks
- [X] T072 Integrate ChatKit SDK into frontend
- [X] T073 Create chat window component for user interaction
- [X] T074 Implement text selection feature from book content
- [X] T075 Create source viewer to display chunks used for answers
- [X] T076 Connect frontend to /agent/ask API endpoint
- [X] T077 Implement API key handling in frontend
- [X] T078 Write frontend tests for user interactions

---

## Phase 12: Polish & Cross-Cutting Concerns
Final touches, optimization, and cross-cutting concerns.

### Goal
Complete the implementation with proper error handling, performance optimization, and deployment readiness.

### Independent Test Criteria
System is production-ready with appropriate performance, security, and reliability measures.

### Tasks
- [X] T079 Optimize response times to meet p95 < 2s requirement
- [X] T080 Implement comprehensive error handling throughout the application
- [X] T081 Add performance monitoring and metrics collection
- [X] T082 Implement proper logging for debugging and monitoring
- [X] T083 Add input validation and sanitization to prevent injection attacks
- [X] T084 Conduct security review of API endpoints and authentication
- [X] T085 Write end-to-end tests simulating complete user interactions
- [X] T086 Prepare deployment configuration for serverless functions
- [X] T087 Document the API and deployment process
- [X] T088 Conduct final integration testing