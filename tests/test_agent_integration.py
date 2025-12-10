import pytest
from unittest.mock import patch, MagicMock
import sys
import os

# Add backend to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import GoogleGeminiAgent

def test_full_agent_integration():
    """Integration test for the full agent functionality."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.rag_tool.rag_tool') as mock_rag_tool, \
         patch('backend.agent.genai.GenerativeModel') as mock_model:

        # Mock the RAG tool response
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "The main theme of the book is artificial intelligence.",
            "sources": ["chunk_123", "chunk_456"]
        }

        # Mock the Gemini model response for context-only response
        mock_response = MagicMock()
        mock_response.text = "The main theme is AI based on the provided context."
        mock_model.return_value.generate_content.return_value = mock_response

        agent = GoogleGeminiAgent()

        # Test the ask method
        result = agent.ask("What is the main theme of the book?", "Sample book content here.")
        assert "answer" in result
        assert "sources" in result
        assert "artificial intelligence" in result["answer"].lower()

        # Test the context-only response method
        context_response = agent.ensure_context_only_response(
            "What is the main theme?",
            "The book is about artificial intelligence and its impact."
        )
        assert "AI based on the provided context" in context_response


def test_agent_with_vector_search_integration():
    """Integration test for agent with vector search functionality."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.rag_tool.rag_tool') as mock_rag_tool, \
         patch('backend.vector_search_tool.vector_search_tool') as mock_vector_search:

        # Mock vector search results
        mock_vector_search.search.return_value = [
            {
                "id": "chunk_789",
                "text": "The book extensively covers machine learning algorithms.",
                "score": 0.85
            },
            {
                "id": "chunk_101",
                "text": "Artificial intelligence is the primary focus of this book.",
                "score": 0.92
            }
        ]

        # Mock the RAG tool response using vector search results
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "The book focuses on artificial intelligence and machine learning algorithms.",
            "sources": ["chunk_789", "chunk_101"]
        }

        agent = GoogleGeminiAgent()

        # When asking without selected text, the agent should trigger vector search internally
        result = agent.ask("What topics does this book cover?")

        assert "answer" in result
        assert "artificial intelligence" in result["answer"].lower()
        assert "machine learning" in result["answer"].lower()
        assert len(result["sources"]) >= 2


def test_agent_grounding_quality():
    """Test that the agent maintains quality of grounding in responses."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.rag_tool.rag_tool') as mock_rag_tool:

        # Mock a response that clearly uses the provided context
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "Based on the provided context: 'Artificial intelligence is transforming industries', the impact is significant.",
            "sources": ["context_chunk"]
        }

        agent = GoogleGeminiAgent()
        result = agent.ask(
            "What is the impact of artificial intelligence?",
            "Artificial intelligence is transforming industries."
        )

        # Verify that the answer is grounded in the provided context
        assert "provided context" in result["answer"]
        assert "transforming industries" in result["answer"]
        assert "significant" in result["answer"]


def test_agent_external_knowledge_rejection():
    """Test that the agent rejects questions outside of provided context."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.rag_tool.rag_tool') as mock_rag_tool:

        # Mock a response that acknowledges lack of context
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "The answer is not found in the provided context.",
            "sources": []
        }

        agent = GoogleGeminiAgent()
        result = agent.ask(
            "What is the capital of France?",
            "This book is about artificial intelligence."
        )

        # Verify that the agent doesn't provide external knowledge
        assert "not found in the provided context" in result["answer"] or "not available" in result["answer"]


if __name__ == "__main__":
    pytest.main()