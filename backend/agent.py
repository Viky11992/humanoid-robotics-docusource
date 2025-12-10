from typing import Dict, Any, List
import os
import google.generativeai as genai
from dotenv import load_dotenv

from .rag_tool import rag_tool
from .vector_search_tool import vector_search_tool
from .embedding_generator import embedding_generator
from .metadata_lookup_tool import metadata_lookup_tool
from .text_chunker import text_chunker

load_dotenv()

class GoogleGeminiAgent:
    """
    Google Gemini Agent structure with registered tools for RAG functionality.
    """

    def __init__(self):
        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-2.5-flash')
        self.system_prompt = (
            "Answer questions strictly based on the provided book content. "
            "If the answer is not found in the context, state that clearly. Do not use external knowledge."
        )

        # Register tools
        self.tools = {
            "rag_answer_synthesis": rag_tool,
            "vector_search": vector_search_tool,
            "embedding_generator": embedding_generator,
            "metadata_lookup": metadata_lookup_tool,
            "text_chunker": text_chunker
        }

        # Tool mappings
        self.tool_functions = {
            "vector_search": self._execute_vector_search,
            "embedding_generator": self._execute_embedding_generation,
            "metadata_lookup": self._execute_metadata_lookup,
            "text_chunking": self._execute_text_chunking
        }

    def _execute_vector_search(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Execute vector search using the vector search tool."""
        return vector_search_tool.search(query, limit)

    def _execute_embedding_generation(self, texts: List[str]) -> List[List[float]]:
        """Execute embedding generation using the embedding generator."""
        return embedding_generator.generate_embeddings(texts)

    def _execute_metadata_lookup(self, qdrant_result_ids: List[str]) -> List[Dict[str, Any]]:
        """Execute metadata lookup using the metadata lookup tool."""
        return metadata_lookup_tool.lookup_metadata(qdrant_result_ids)

    def _execute_text_chunking(self, text: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict[str, Any]]:
        """Execute text chunking using the text chunker."""
        chunker = text_chunker.__class__(chunk_size=chunk_size, overlap=overlap)
        return chunker.chunk_text(text)

    def ask(self, question: str, selected_text: str = None) -> Dict[str, Any]:
        """
        Process a question using the agent with RAG capabilities.

        Args:
            question: The question to answer
            selected_text: Optional context provided by the user

        Returns:
            Dictionary with answer and sources
        """
        # Use the RAG tool to synthesize the answer
        result = rag_tool.synthesize_answer(
            question=question,
            selected_text=selected_text
        )

        return result

    def ensure_context_only_response(self, question: str, context: str = None) -> str:
        """
        Ensure the agent strictly follows the system prompt to avoid external knowledge.

        Args:
            question: The question to answer
            context: The context to use for answering

        Returns:
            The answer from the agent
        """
        if context:
            prompt = f"{self.system_prompt}\n\nContext: {context}\n\nQuestion: {question}"
        else:
            prompt = f"{self.system_prompt}\n\nQuestion: {question}"

        response = self.model.generate_content(
            prompt,
            generation_config=genai.types.GenerationConfig(
                temperature=0.3,
                max_output_tokens=1000
            )
        )

        return response.text


# Global instance
google_agent = GoogleGeminiAgent()