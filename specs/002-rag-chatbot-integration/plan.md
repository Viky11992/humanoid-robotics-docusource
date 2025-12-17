# Implementation Plan: Integrated RAG Chatbot Development

**Branch**: `002-rag-chatbot-integration` | **Date**: 2025-12-16 | **Spec**: [link to spec.md]
**Input**: Feature specification from `/specs/002-rag-chatbot-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Development of a complete RAG (Retrieval-Augmented Generation) chatbot system that integrates with a Docusaurus-published book, using Cohere for embeddings and generation, FastAPI for the backend, Qdrant for vector storage, and Neon Postgres for metadata. The system will support both full-book queries and context-restricted selected-text queries while staying within free tier limits of all services.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend components
**Primary Dependencies**: FastAPI, Cohere SDK, Qdrant client, Neon Postgres driver, Docusaurus
**Storage**: Qdrant Cloud (vector store), Neon Serverless Postgres (metadata)
**Testing**: pytest for backend, Jest for frontend components
**Target Platform**: Web application (Docusaurus site with embedded chatbot)
**Project Type**: Web (backend API + frontend integration)
**Performance Goals**: < 5 seconds response time for 95% of queries, support 100 concurrent users
**Constraints**: Stay within free tier limits (1GB Qdrant, Cohere rate limits, Neon compute/storage bounds)
**Scale/Scope**: Handle 20,000-40,000 words of book content, support 1000 daily active users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- Specification-first development: All components defined in specifications before implementation ✓
- Accuracy and context faithfulness: Responses grounded in book content with no hallucinations ✓
- Reproducibility: Full documentation of data ingestion and embedding process ✓
- Security and privacy: API keys in environment variables, not exposed in client-side code ✓
- Zero additional cost: Using free tiers of Cohere, Qdrant Cloud, Neon Serverless Postgres ✓
- Accessibility: Chatbot integrated into Docusaurus site, responsive design ✓
- RAG pipeline: Using Cohere models for embeddings and generation ✓
- Backend: FastAPI with Cohere SDK integration ✓
- Vector store: Qdrant Cloud Free Tier for embeddings ✓
- Metadata storage: Neon Serverless Postgres for session management ✓
- Frontend embedding: React component for Docusaurus integration ✓
- Selected text feature: Client-side capture with backend processing ✓

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── chat.py
│   │   ├── embedding.py
│   │   └── session.py
│   ├── services/
│   │   ├── cohere_service.py
│   │   ├── qdrant_service.py
│   │   ├── postgres_service.py
│   │   └── rag_service.py
│   ├── api/
│   │   ├── chat_router.py
│   │   ├── embeddings_router.py
│   │   └── health_router.py
│   └── main.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
└── requirements.txt

frontend/
├── src/
│   ├── components/
│   │   ├── RagChatbot.jsx
│   │   └── SelectedTextHandler.jsx
│   ├── services/
│   │   └── api.js
│   └── hooks/
│       └── useChat.js
└── static/
    └── js/
        └── rag-chatbot.js
```

**Structure Decision**: Web application structure selected with separate backend (FastAPI) and frontend (React components for Docusaurus integration) to maintain clean separation of concerns while enabling seamless integration with the Docusaurus site.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple services | Cohere, Qdrant, and Neon integration required | Single service approach insufficient for RAG architecture |
| Complex data flow | Vector retrieval and generation pipeline | Simple keyword search insufficient for accurate responses |