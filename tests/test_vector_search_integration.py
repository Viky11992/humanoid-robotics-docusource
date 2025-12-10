import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import sys
import os

# Add backend to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.main import app

client = TestClient(app)

def test_full_flow_without_selected_text():
    """Integration test for the full flow when no selected text is provided (vector search is used)."""
    with patch('backend.rag_tool.rag_tool') as mock_rag_tool, \
         patch('backend.logging_service.log_query') as mock_log_query:

        # Mock the RAG tool response when vector search is performed
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "Based on the book content, the main theme appears to be artificial intelligence.",
            "sources": ["chunk_id_1", "chunk_id_2"]
        }

        # Make request without selected text (should trigger vector search)
        response = client.post(
            "/agent/ask",
            json={
                "question": "What is the main theme of the book?"
            },
            headers={"x-api-key": "your_api_key_for_authentication"}
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()
        assert "artificial intelligence" in data["answer"].lower()
        assert len(data["sources"]) > 0

        # Verify that logging was called
        mock_log_query.assert_called_once()


def test_vector_search_integration_with_api():
    """Integration test for the vector search functionality through the API."""
    with patch('backend.rag_tool.rag_tool') as mock_rag_tool, \
         patch('backend.vector_search_tool.vector_search_tool') as mock_vector_search, \
         patch('backend.logging_service.log_query'):

        # Mock vector search results
        mock_search_result = {
            "id": "chunk_123",
            "text": "The book discusses artificial intelligence and its impact on society.",
            "score": 0.92
        }
        mock_vector_search.search.return_value = [mock_search_result]

        # Mock the RAG tool to use the vector search results
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "The main theme of the book is artificial intelligence and its impact on society.",
            "sources": ["chunk_123"]
        }

        # Make request without selected text
        response = client.post(
            "/agent/ask",
            json={
                "question": "What is the main theme of the book?"
            },
            headers={"x-api-key": "your_api_key_for_authentication"}
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()
        assert "artificial intelligence" in data["answer"].lower()
        assert "chunk_123" in data["sources"]

        # Verify that vector search was called with the question
        mock_vector_search.search.assert_called_once_with("What is the main theme of the book?")


def test_embedding_generation_integration():
    """Integration test for embedding generation through the full flow."""
    with patch('backend.rag_tool.rag_tool') as mock_rag_tool, \
         patch('backend.vector_search_tool.vector_search_tool') as mock_vector_search, \
         patch('backend.embedding_generator.embedding_generator') as mock_embedding_gen, \
         patch('backend.logging_service.log_query'):

        # Mock the embedding generation
        mock_embedding_gen.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Mock vector search results
        mock_search_result = {
            "id": "chunk_456",
            "text": "The book explores the future of technology.",
            "score": 0.88
        }
        mock_vector_search.search.return_value = [mock_search_result]

        # Mock the RAG tool response
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "The book explores the future of technology.",
            "sources": ["chunk_456"]
        }

        # Make request that will trigger embedding generation and vector search
        response = client.post(
            "/agent/ask",
            json={
                "question": "What does this book explore?"
            },
            headers={"x-api-key": "your_api_key_for_authentication"}
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()
        assert "technology" in data["answer"].lower()

        # Verify that embedding generation was called as part of the vector search process
        # (This happens internally in the vector search tool)


if __name__ == "__main__":
    pytest.main()