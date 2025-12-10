from typing import List, Dict, Any
import sys
import os

# Add the current directory to the path
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.append(current_dir)

from qdrant_config import get_qdrant_client
from qdrant_operations import QdrantOperations
from embedding_generator import embedding_generator

class VectorSearchTool:
    """
    Tool for querying Qdrant based on input query to find relevant chunks.
    """

    def __init__(self):
        self.qdrant_client = get_qdrant_client()
        self.qdrant_ops = QdrantOperations(self.qdrant_client)

    def search(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for relevant chunks in Qdrant based on the input query.

        Args:
            query: The search query (can be user question or chunk)
            limit: Number of results to return

        Returns:
            List of search results with id, text, and score
        """
        # Generate embedding for the query
        query_embedding = embedding_generator.generate_embedding(query)

        # Search in Qdrant
        results = self.qdrant_ops.search_similar(query_embedding, limit=limit)

        # Format results to match the required structure
        formatted_results = []
        for result in results:
            formatted_results.append({
                "id": result.id,
                "text": result.payload.get("text", ""),
                "score": result.score
            })

        return formatted_results


# Global instance
vector_search_tool = VectorSearchTool()