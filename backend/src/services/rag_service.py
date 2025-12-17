from typing import List, Dict, Any, Optional
import time
from src.services.cohere_service import cohere_service
from src.services.qdrant_service import qdrant_service
from src.services.postgres_service import postgres_service
from src.models.chat import Source
from src.utils.text_processor import chunk_text, validate_chunk, generate_content_hash
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.cohere_service = cohere_service
        self.qdrant_service = qdrant_service
        self.postgres_service = postgres_service

    async def process_query(self,
                           query: str,
                           session_id: Optional[str] = None,
                           selected_text: Optional[str] = None,
                           context_restrict: bool = False,
                           max_results: int = 5,
                           similarity_threshold: float = 0.7) -> Dict[str, Any]:
        """
        Process a user query using RAG methodology

        Args:
            query: The user's query
            session_id: Session identifier for conversation history
            selected_text: Selected text to restrict context (if any)
            context_restrict: Whether to restrict to selected text only
            max_results: Maximum number of context results to retrieve
            similarity_threshold: Minimum similarity score for retrieved context

        Returns:
            Dictionary with response, sources, and metadata
        """
        start_time = time.time()

        # If context restriction is enabled and selected text is provided,
        # use only the selected text as context
        if context_restrict and selected_text:
            # Generate response using only selected text as context
            response = self.cohere_service.generate_response(query, selected_text)
            sources = [Source(content=selected_text[:200] + "...", score=1.0, source="selected_text")]
        else:
            # Standard RAG flow: retrieve context from vector store
            sources = []

            # Generate embedding for the query
            query_embeddings = self.cohere_service.generate_embeddings([query], input_type="query")
            query_embedding = query_embeddings[0]

            # Search for similar content in Qdrant
            similar_docs = self.qdrant_service.search_similar(
                query_embedding=query_embedding,
                limit=max_results,
                threshold=similarity_threshold
            )

            # Extract content for context
            context_parts = []
            for doc in similar_docs:
                context_parts.append(doc["content"])
                sources.append(Source(
                    content=doc["content"][:200] + "...",  # Truncate for display
                    score=doc["score"],
                    source=doc["source"]
                ))

            # Combine context
            context = "\n\n".join(context_parts)

            # Generate response using retrieved context
            response = self.cohere_service.generate_response(query, context)

        query_time = time.time() - start_time

        # Create or update session if needed
        if not session_id:
            session_id = await self.postgres_service.create_session()

        # Store the conversation
        await self.postgres_service.create_conversation(
            session_id=session_id,
            query=query,
            response=response,
            selected_text=selected_text,
            sources=[source.dict() for source in sources] if sources else None
        )

        return {
            "response": response,
            "sources": sources,
            "session_id": session_id,
            "query_time": query_time
        }

    async def ingest_document(self,
                             title: str,
                             content: str,
                             source_url: Optional[str] = None,
                             section: Optional[str] = None) -> bool:
        """
        Ingest a document into the RAG system

        Args:
            title: Title of the document
            content: Content of the document
            source_url: URL where the document came from
            section: Section or chapter identifier

        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if document already exists using content hash
            content_hash = generate_content_hash(content)
            existing_doc = await self.postgres_service.get_document_by_hash(content_hash)

            if existing_doc:
                logger.info(f"Document already exists with hash: {content_hash[:8]}...")
                return True

            # Chunk the content
            chunks = chunk_text(content, chunk_size=512, overlap=50)

            # Filter valid chunks
            valid_chunks = [chunk for chunk in chunks if validate_chunk(chunk)]

            if not valid_chunks:
                logger.warning("No valid chunks found for ingestion")
                return False

            # Generate embeddings for all chunks
            embeddings = self.cohere_service.generate_embeddings(valid_chunks, input_type="document")

            # Prepare metadata for each chunk
            sources = [source_url or title for _ in valid_chunks]
            chunk_ids = [f"{content_hash[:8]}_{i}" for i in range(len(valid_chunks))]
            sections = [section for _ in valid_chunks] if section else ["" for _ in valid_chunks]

            # Store embeddings in Qdrant
            success = self.qdrant_service.store_text_chunks_with_embeddings(
                texts=valid_chunks,
                embeddings=embeddings,
                sources=sources,
                chunk_ids=chunk_ids,
                sections=sections
            )

            if success:
                # Store document metadata in Postgres
                await self.postgres_service.create_document(
                    title=title,
                    source_url=source_url,
                    content_hash=content_hash,
                    chunk_count=len(valid_chunks),
                    word_count=len(content.split()),
                    metadata={"section": section} if section else {}
                )
                logger.info(f"Successfully ingested document: {title} ({len(valid_chunks)} chunks)")
                return True
            else:
                logger.error(f"Failed to store embeddings for document: {title}")
                return False

        except Exception as e:
            logger.error(f"Error ingesting document {title}: {str(e)}")
            return False

    async def get_conversation_history(self, session_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get conversation history for a session

        Args:
            session_id: Session identifier
            limit: Maximum number of conversations to return

        Returns:
            List of conversation records
        """
        return await self.postgres_service.get_conversations_by_session(session_id, limit)

# Singleton instance
rag_service = RAGService()