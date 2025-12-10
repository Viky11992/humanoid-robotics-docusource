title: "RAG Chatbot Integration"
version: 1.0.0
description: >
  Retrieval-Augmented Generation chatbot integrated with OpenAI Agent SDK,
  FastAPI backend, Neon Postgres, and Qdrant vector database. The agent answers
  questions about the book, including questions based on user-selected text.

functional_requirements:
  - id: RAG-01
    description: "User can select text from the book content and the chatbot uses that as context."
    priority: P1
  - id: RAG-02
    description: "If no selected text is provided, chatbot uses vector search in Qdrant for context."
    priority: P1
  - id: RAG-03
    description: "Use OpenAI Agent SDK and ChatKit for question answering, ensuring answers are context-grounded."
    priority: P1
  - id: RAG-04
    description: "Expose a POST /agent/ask endpoint for receiving user questions and optional selected_text."
    priority: P1
  - id: RAG-05
    description: "Ingest book text, generate embeddings using text-embedding-3-large, and store chunks with embeddings in Qdrant and metadata in Neon Postgres."
    priority: P1
  - id: RAG-06
    description: "Agent must return strictly context-based answers, avoiding external knowledge."
    priority: P1
  - id: RAG-07
    description: "Log all user queries and agent responses in Neon Postgres."
    priority: P2

non_functional_requirements:
  - requirement: "Response time should be under 2 seconds (p95 for agent response)."
    metric: "p95 response time"
    target: "< 2s"
  - requirement: "API secured with x-api-key authentication."
    metric: "API Key Authentication"
    target: "Implemented and enforced"
  - requirement: "Scalable to handle concurrent requests within Qdrant Cloud Free Tier limitations."
    metric: "Concurrency"
    target: "Cloud Free Tier limits"
  - requirement: "Deployable via serverless functions and Cloud Code Router."
    metric: "Deployment method"
    target: "Serverless, Cloud Code Router"

api_endpoints:
  - path: /agent/ask
    method: POST
    input:
      question: string
      selected_text: string (optional)
    output:
      answer: string
      sources:
        - string
  - path: /rag/ingest
    method: POST
    input:
      text: string # Full text of the book to be ingested
    output:
      status: string # e.g., "success", "failed"
  - path: /rag/search
    method: POST
    input:
      query: string # The search query (can be user question or chunk)
    output:
      results:
        - id: string
          text: string
          score: float

components:
  - name: OpenAI Agent SDK
    role: "Core RAG intelligence for question answering and tool orchestration."
  - name: ChatKit
    role: "Frontend chat interface for user interaction."
  - name: FastAPI
    role: "Host API endpoints, manage request routing, and integrate agent tools."
  - name: Qdrant Cloud
    role: "Vector store for efficient similarity search of book chunks."
  - name: Neon Postgres
    role: "Store book metadata, ingestion status, and query logs."

testing_strategy:
  - description: "Unit tests for core logic (embeddings, vector search, answer generation tools)."
    coverage: "80%"
  - description: "Integration tests for the full RAG pipeline, including API endpoints and database interactions."
    coverage: "75%"
  - description: "End-to-end tests simulating user interaction via ChatKit."
    coverage: "70%"
