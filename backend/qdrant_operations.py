from qdrant_client import QdrantClient  # This is the external library, not our file
from qdrant_client.http import models
from typing import List, Dict, Any
import uuid

class QdrantOperations:
    def __init__(self, client: QdrantClient):
        self.client = client
        self.collection_name = "book_embeddings"

    def create_collection(self):
        """
        Create Qdrant collection for storing book text embeddings with appropriate vector configuration.
        Uses text-embedding-3-large which produces 3072-dimensional vectors.
        """
        # Check if collection already exists
        try:
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists.")
            return
        except:
            pass  # Collection doesn't exist, proceed to create

        # Create collection with 3072 dimensions (for text-embedding-3-large)
        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=3072,  # text-embedding-3-large produces 3072-dimensional vectors
                distance=models.Distance.COSINE
            )
        )
        print(f"Collection '{self.collection_name}' created successfully.")

    def add_embeddings(self, texts: List[str], metadata: List[Dict[str, Any]] = None):
        """
        Add text embeddings to the collection.

        Args:
            texts: List of text chunks to embed and store
            metadata: List of metadata dictionaries corresponding to each text
        """
        if metadata is None:
            metadata = [{}] * len(texts)

        # Generate IDs for the points
        ids = [str(uuid.uuid4()) for _ in texts]

        # In a real implementation, we would generate actual embeddings here
        # For now, we'll create placeholder vectors (in real implementation,
        # these would come from the embedding model)
        # For demonstration, we'll use dummy vectors - in real implementation,
        # these would be actual embeddings from text-embedding-3-large
        vectors = [[0.0] * 3072 for _ in texts]  # Placeholder vectors

        # Create points to upsert
        points = [
            models.PointStruct(
                id=ids[i],
                vector=vectors[i],
                payload={
                    "text": texts[i],
                    "metadata": metadata[i]
                }
            )
            for i in range(len(texts))
        ]

        # Upsert the points to the collection
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return ids

    def search_similar(self, query_vector: List[float], limit: int = 5):
        """
        Search for similar text chunks based on the query vector.

        Args:
            query_vector: Vector to search for similarity
            limit: Number of results to return

        Returns:
            List of similar points with their payloads
        """
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit
        )

        return results

    def get_collection_info(self):
        """
        Get information about the collection.
        """
        return self.client.get_collection(self.collection_name)