from typing import List
import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

class EmbeddingGenerator:
    """
    Generate embeddings using Google's embedding model.
    """

    def __init__(self):
        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        self.model = 'embedding-001'  # Google's embedding model

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Google's embedding model.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        if not texts:
            return []

        embeddings = []
        for text in texts:
            # Use Google's embedding generation
            result = genai.embed_content(
                model=self.model,
                content=text,
                task_type="retrieval_document"  # or "retrieval_query" for queries
            )
            embeddings.append(result['embedding'])

        return embeddings

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text string to embed

        Returns:
            Embedding vector (list of floats)
        """
        embeddings = self.generate_embeddings([text])
        return embeddings[0] if embeddings else []


# Global instance
embedding_generator = EmbeddingGenerator()