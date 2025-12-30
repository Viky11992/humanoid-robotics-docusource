import cohere
from typing import List, Dict, Any
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class CohereService:
    def __init__(self):
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = cohere.Client(api_key)
        self.embed_model = "embed-multilingual-v3.0"  # Recommended for multilingual content
        self.generate_model = "command-r-plus-08-2024"  # Updated model for RAG applications

    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for the given texts using Cohere

        Args:
            texts: List of texts to embed
            input_type: "search_document" for embedding content, "search_query" for embedding search queries

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.embed_model,
                input_type=input_type
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            print(f"Error generating embeddings: {str(e)}")
            raise e

    def generate_response(self, prompt: str, context: str = None) -> str:
        """
        Generate a response using Cohere's language model

        Args:
            prompt: The user's query or prompt
            context: Retrieved context to ground the response

        Returns:
            Generated response text
        """
        try:
            # If context is provided, include it in the message
            if context:
                message = f"Based on the following context: {context}\n\nAnswer the following question: {prompt}\n\nIf the answer is not in the provided context, please state that you don't have enough information from the context to answer the question accurately."
            else:
                message = f"{prompt}\n\nProvide a helpful response."

            response = self.client.chat(
                model=self.generate_model,
                message=message,
                max_tokens=4000,  # Max output for command-r-plus
                temperature=0.3,  # Lower temperature for more factual responses
            )

            generated_text = response.text if response else ""

            # Validate response to prevent hallucinations
            validated_response = self._validate_response(generated_text, context)

            return validated_response
        except Exception as e:
            print(f"Error generating response: {str(e)}")
            raise e

    def _validate_response(self, response: str, context: str = None) -> str:
        """
        Validate the generated response to prevent hallucinations

        Args:
            response: The generated response
            context: The context that was provided to the model

        Returns:
            Validated response
        """
        # Check if the response contains phrases indicating lack of information
        if "I don't have enough information" in response or "I cannot find" in response:
            # This is a valid response when context doesn't contain the answer
            return response

        # Additional validation logic can be added here
        # For now, return the response as is
        return response

    def rerank(self, query: str, documents: List[str], top_n: int = 5) -> List[Dict[str, Any]]:
        """
        Rerank documents based on relevance to the query

        Args:
            query: The search query
            documents: List of documents to rerank
            top_n: Number of top results to return

        Returns:
            List of reranked documents with indices and relevance scores
        """
        try:
            response = self.client.rerank(
                model="rerank-multilingual-v2.0",  # Use dedicated rerank model
                query=query,
                documents=documents,
                top_n=top_n
            )
            return [
                {
                    "index": result.index,
                    "relevance_score": result.relevance_score,
                    "document": result.document
                }
                for result in response.results
            ]
        except Exception as e:
            print(f"Error reranking documents: {str(e)}")
            raise e

# Singleton instance
cohere_service = CohereService()