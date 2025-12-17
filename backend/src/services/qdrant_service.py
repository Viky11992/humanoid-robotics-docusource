from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self):
        url = os.getenv("QDRANT_URL")
        api_key = os.getenv("QDRANT_API_KEY")
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

        if not url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")
        if not collection_name:
            raise ValueError("QDRANT_COLLECTION_NAME environment variable is required")

        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
        self.vector_size = 1024  # Cohere multilingual v3 embedding dimensions
        self.distance = models.Distance.COSINE

        # Ensure collection exists
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the collection exists with the correct configuration"""
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [col.name for col in collections]

            if self.collection_name not in collection_names:
                # Create collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size,
                        distance=self.distance
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {str(e)}")
            raise e

    def store_embeddings(self,
                        texts: List[str],
                        sources: List[str],
                        chunk_ids: List[str],
                        sections: Optional[List[str]] = None,
                        metadata_list: Optional[List[Dict[str, Any]]] = None) -> bool:
        """
        Store text embeddings in Qdrant

        Args:
            texts: List of text chunks to store
            sources: List of source identifiers for each text
            chunk_ids: List of unique chunk identifiers
            sections: Optional list of section identifiers
            metadata_list: Optional list of additional metadata for each text

        Returns:
            True if successful, False otherwise
        """
        try:
            # Generate IDs for the points
            ids = list(range(len(texts)))

            # Prepare payloads
            payloads = []
            for i in range(len(texts)):
                payload = {
                    "content": texts[i],
                    "source": sources[i],
                    "chunk_id": chunk_ids[i],
                    "section": sections[i] if sections and i < len(sections) else "",
                    "metadata": metadata_list[i] if metadata_list and i < len(metadata_list) else {}
                }
                payloads.append(payload)

            # Store in Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=models.Batch(
                    ids=ids,
                    vectors=texts,  # Will be replaced by embeddings in actual usage
                    payloads=payloads
                )
            )

            logger.info(f"Stored {len(texts)} embeddings in Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error storing embeddings: {str(e)}")
            return False

    def search_similar(self,
                      query_embedding: List[float],
                      limit: int = 5,
                      threshold: float = 0.7) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in Qdrant

        Args:
            query_embedding: The embedding to search for
            limit: Maximum number of results to return
            threshold: Minimum similarity score threshold

        Returns:
            List of similar documents with content, source, and similarity score
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                score_threshold=threshold
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "content": result.payload.get("content", ""),
                    "source": result.payload.get("source", ""),
                    "chunk_id": result.payload.get("chunk_id", ""),
                    "section": result.payload.get("section", ""),
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {})
                })

            logger.info(f"Found {len(formatted_results)} similar documents")
            return formatted_results
        except Exception as e:
            logger.error(f"Error searching for similar embeddings: {str(e)}")
            return []

    def store_text_chunks_with_embeddings(self,
                                        texts: List[str],
                                        embeddings: List[List[float]],
                                        sources: List[str],
                                        chunk_ids: List[str],
                                        sections: Optional[List[str]] = None,
                                        metadata_list: Optional[List[Dict[str, Any]]] = None) -> bool:
        """
        Store text chunks with pre-generated embeddings in Qdrant

        Args:
            texts: List of text chunks to store
            embeddings: List of pre-generated embeddings
            sources: List of source identifiers for each text
            chunk_ids: List of unique chunk identifiers
            sections: Optional list of section identifiers
            metadata_list: Optional list of additional metadata for each text

        Returns:
            True if successful, False otherwise
        """
        try:
            # Generate IDs for the points
            ids = list(range(len(texts)))

            # Prepare payloads
            payloads = []
            for i in range(len(texts)):
                payload = {
                    "content": texts[i],
                    "source": sources[i],
                    "chunk_id": chunk_ids[i],
                    "section": sections[i] if sections and i < len(sections) else "",
                    "metadata": metadata_list[i] if metadata_list and i < len(metadata_list) else {}
                }
                payloads.append(payload)

            # Store in Qdrant with embeddings
            self.client.upsert(
                collection_name=self.collection_name,
                points=models.Batch(
                    ids=ids,
                    vectors=embeddings,
                    payloads=payloads
                )
            )

            logger.info(f"Stored {len(texts)} text chunks with embeddings in Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error storing text chunks with embeddings: {str(e)}")
            return False

    def delete_collection(self):
        """Delete the entire collection (use with caution)"""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Error deleting collection: {str(e)}")
            return False

# Singleton instance
qdrant_service = QdrantService()