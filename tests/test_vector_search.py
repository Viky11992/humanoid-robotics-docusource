import pytest
from unittest.mock import patch, MagicMock
import sys
import os

# Add backend to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.vector_search_tool import VectorSearchTool
from backend.embedding_generator import EmbeddingGenerator

def test_vector_search_tool_initialization():
    """Test that the VectorSearchTool initializes properly."""
    with patch('backend.qdrant_client.get_qdrant_client'), \
         patch('backend.qdrant_operations.QdrantOperations'):
        tool = VectorSearchTool()
        assert tool is not None


def test_vector_search_functionality():
    """Test the vector search functionality."""
    with patch('backend.qdrant_client.get_qdrant_client'), \
         patch('backend.qdrant_operations.QdrantOperations') as mock_qdrant_ops, \
         patch('backend.embedding_generator.embedding_generator') as mock_embedding_gen:

        # Mock the embedding generation
        mock_embedding_gen.generate_embedding.return_value = [0.1, 0.2, 0.3, 0.4]

        # Mock the search results
        mock_result = MagicMock()
        mock_result.id = "test_id_1"
        mock_result.payload = {"text": "This is a test chunk"}
        mock_result.score = 0.85
        mock_qdrant_ops.search_similar.return_value = [mock_result]

        # Create the tool
        tool = VectorSearchTool()

        # Perform search
        results = tool.search("test query", limit=5)

        # Verify the results
        assert len(results) == 1
        assert results[0]["id"] == "test_id_1"
        assert results[0]["text"] == "This is a test chunk"
        assert results[0]["score"] == 0.85

        # Verify that embedding generation and search were called
        mock_embedding_gen.generate_embedding.assert_called_once_with("test query")
        mock_qdrant_ops.search_similar.assert_called_once_with([0.1, 0.2, 0.3, 0.4], limit=5)


def test_embedding_generator_initialization():
    """Test that the EmbeddingGenerator initializes properly."""
    with patch.dict(os.environ, {"OPENAI_API_KEY": "test_key"}):
        generator = EmbeddingGenerator()
        assert generator is not None
        assert generator.model == "text-embedding-3-large"


def test_embedding_generation():
    """Test the embedding generation functionality."""
    with patch.dict(os.environ, {"OPENAI_API_KEY": "test_key"}), \
         patch('backend.embedding_generator.OpenAI') as mock_openai:

        # Mock the API response
        mock_response = MagicMock()
        mock_response.data = [
            MagicMock(embedding=[0.1, 0.2, 0.3])
        ]
        mock_openai.return_value.embeddings.create.return_value = mock_response

        generator = EmbeddingGenerator()
        embeddings = generator.generate_embeddings(["test text"])

        assert len(embeddings) == 1
        assert embeddings[0] == [0.1, 0.2, 0.3]


def test_text_chunker_initialization():
    """Test that the TextChunker initializes properly."""
    from backend.text_chunker import TextChunker

    chunker = TextChunker(chunk_size=500, overlap=50)
    assert chunker.chunk_size == 500
    assert chunker.overlap == 50


def test_text_chunking():
    """Test the text chunking functionality."""
    from backend.text_chunker import TextChunker

    chunker = TextChunker(chunk_size=20, overlap=5)
    text = "This is a sample text for chunking. It should be broken into smaller pieces."
    chunks = chunker.chunk_text(text)

    assert len(chunks) > 0
    assert "text" in chunks[0]
    assert "metadata" in chunks[0]


if __name__ == "__main__":
    pytest.main()