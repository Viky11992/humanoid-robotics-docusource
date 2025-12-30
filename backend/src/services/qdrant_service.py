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

        # Check if we're in local development mode (for testing without Qdrant)
        is_local_dev = os.getenv("DEBUG", "false").lower() == "true"

        if not collection_name:
            raise ValueError("QDRANT_COLLECTION_NAME environment variable is required")

        # Initialize client based on environment
        if url and not (is_local_dev and "localhost" in url and not self._check_qdrant_connection(url, api_key)):
            try:
                self.client = QdrantClient(url=url, api_key=api_key)
                self.collection_name = collection_name
                self.vector_size = 1024  # Cohere multilingual v3 embedding dimensions
                self.distance = models.Distance.COSINE

                # Ensure collection exists
                self._ensure_collection_exists()
            except Exception as e:
                logger.warning(f"Could not connect to Qdrant at {url}: {str(e)}")
                logger.info("Initializing with mock client for local development...")
                self._init_mock_client(collection_name)
        else:
            self._init_mock_client(collection_name)

    def _check_qdrant_connection(self, url, api_key):
        """Check if Qdrant is accessible"""
        try:
            from qdrant_client.http.exceptions import ResponseHandlingException
            test_client = QdrantClient(url=url, api_key=api_key, timeout=2.0)
            test_client.get_collections()
            return True
        except:
            return False

    def _init_mock_client(self, collection_name):
        """Initialize a mock client for local development"""
        logger.info("Using mock Qdrant client for local development")
        self.client = None  # Will be initialized when needed
        self.collection_name = collection_name
        self.vector_size = 1024
        self.distance = models.Distance.COSINE
        self._mock_storage = []

    def _get_client(self):
        """Get actual Qdrant client, initializing if needed"""
        if self.client is None:
            url = os.getenv("QDRANT_URL")
            api_key = os.getenv("QDRANT_API_KEY")
            if url and api_key:
                try:
                    self.client = QdrantClient(url=url, api_key=api_key)
                    self._ensure_collection_exists()
                except Exception as e:
                    logger.error(f"Failed to initialize Qdrant client: {e}")
                    raise
        return self.client

    def _ensure_collection_exists(self):
        """Ensure the collection exists with the correct configuration"""
        if self.client is not None:
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
        else:
            logger.info(f"Mock client: Collection {self.collection_name} assumed to exist")

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
        if self.client is not None:
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
                self._get_client().upsert(
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
        else:
            # Mock implementation - store in memory
            for i, text in enumerate(texts):
                mock_point = {
                    "id": i,
                    "vector": text,  # In mock, we'll just store the text
                    "payload": {
                        "content": text,
                        "source": sources[i] if i < len(sources) else "",
                        "chunk_id": chunk_ids[i] if i < len(chunk_ids) else "",
                        "section": sections[i] if sections and i < len(sections) else "",
                        "metadata": metadata_list[i] if metadata_list and i < len(metadata_list) else {}
                    }
                }
                self._mock_storage.append(mock_point)
            logger.info(f"Stored {len(texts)} embeddings in mock storage")
            return True

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
        if self.client is not None:
            try:
                results = self._get_client().query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=limit,
                    score_threshold=threshold
                )

                # Format results - query_points returns QueryResponse with results attribute
                formatted_results = []

                # Handle different return types from query_points
                if hasattr(results, 'results'):
                    # Newer API: query_points returns QueryResponse with 'results' attribute
                    results_list = results.results
                elif hasattr(results, 'points'):
                    # Older format
                    results_list = results.points
                else:
                    # Direct list (fallback)
                    results_list = results

                for result in results_list:
                    # Handle different result structures
                    payload = getattr(result, 'payload', getattr(result, 'document', {}))
                    score = getattr(result, 'score', getattr(result, 'score', 0.0))

                    formatted_results.append({
                        "content": payload.get("content", ""),
                        "source": payload.get("source", ""),
                        "chunk_id": payload.get("chunk_id", ""),
                        "section": payload.get("section", ""),
                        "score": score if score is not None else 0.0,
                        "metadata": payload.get("metadata", {})
                    })

                logger.info(f"Found {len(formatted_results)} similar documents")
                return formatted_results
            except Exception as e:
                logger.error(f"Error searching for similar embeddings: {str(e)}")
                return []
        else:
            # Mock implementation - return stored items if we can't do proper similarity
            # For simplicity in mock mode, return all stored items with score 1.0 if there are few
            formatted_results = []
            for item in self._mock_storage[:limit]:
                formatted_results.append({
                    "content": item["payload"]["content"],
                    "source": item["payload"]["source"],
                    "chunk_id": item["payload"]["chunk_id"],
                    "section": item["payload"]["section"],
                    "score": 1.0,  # Mock score
                    "metadata": item["payload"]["metadata"]
                })
            logger.info(f"Found {len(formatted_results)} similar documents in mock storage")
            return formatted_results

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
        if self.client is not None:
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
                self._get_client().upsert(
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
        else:
            # Mock implementation
            for i, text in enumerate(texts):
                mock_point = {
                    "id": i,
                    "vector": embeddings[i] if i < len(embeddings) else [0.0] * self.vector_size,
                    "payload": {
                        "content": text,
                        "source": sources[i] if i < len(sources) else "",
                        "chunk_id": chunk_ids[i] if i < len(chunk_ids) else "",
                        "section": sections[i] if sections and i < len(sections) else "",
                        "metadata": metadata_list[i] if metadata_list and i < len(metadata_list) else {}
                    }
                }
                self._mock_storage.append(mock_point)
            logger.info(f"Stored {len(texts)} text chunks with embeddings in mock storage")
            return True

    def delete_collection(self):
        """Delete the entire collection (use with caution)"""
        if self.client is not None:
            try:
                self._get_client().delete_collection(collection_name=self.collection_name)
                logger.info(f"Deleted collection: {self.collection_name}")
                return True
            except Exception as e:
                logger.error(f"Error deleting collection: {str(e)}")
                return False
        else:
            # Mock implementation
            self._mock_storage = []
            logger.info(f"Cleared mock collection: {self.collection_name}")
            return True

# Singleton instance
qdrant_service = QdrantService()