#!/usr/bin/env python3
"""
Test script to verify that the Cohere service is properly connected and working.
"""

import asyncio
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.cohere_service import cohere_service
from src.services.rag_service import rag_service
from dotenv import load_dotenv

async def test_cohere_connection():
    """Test basic Cohere API connection."""
    print("Testing Cohere API connection...")

    try:
        # Test generating a simple response without context
        response = cohere_service.generate_response("Hello, how are you?")
        print(f"✓ Simple response generated: {response[:100]}...")

        # Test generating embeddings
        embeddings = cohere_service.generate_embeddings(["Hello world", "Test document"])
        print(f"✓ Embeddings generated successfully. Shape: {len(embeddings)} vectors")

        print("✓ Cohere service is properly connected and functional!")
        return True
    except Exception as e:
        print(f"ERROR: Error testing Cohere connection: {str(e)}")
        return False

async def test_rag_functionality():
    """Test the full RAG functionality."""
    print("\nTesting RAG functionality...")

    try:
        # Initialize PostgreSQL service first
        from src.services.postgres_service import postgres_service
        await postgres_service.initialize()
        print("✓ PostgreSQL service initialized")

        # Test processing a simple query
        result = await rag_service.process_query(
            query="What is Physical AI?",
            max_results=2,
            similarity_threshold=0.5
        )

        print(f"✓ RAG query processed successfully")
        print(f"  Response: {result['response'][:150]}...")
        print(f"  Sources: {len(result['sources'])} sources found")
        print(f"  Query time: {result['query_time']:.2f} seconds")

        # Close PostgreSQL connection
        await postgres_service.close()

        print("✓ RAG system is fully functional!")
        return True
    except Exception as e:
        print(f"ERROR: Error testing RAG functionality: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    # Load environment variables
    load_dotenv()

    print("Starting LLM connectivity tests...\n")

    # Test Cohere connection
    cohere_ok = await test_cohere_connection()

    # Test RAG functionality
    rag_ok = await test_rag_functionality()

    print(f"\nTest Results:")
    print(f"Cohere Service: {'✓ PASS' if cohere_ok else '✗ FAIL'}")
    print(f"RAG System: {'✓ PASS' if rag_ok else '✗ FAIL'}")

    if cohere_ok and rag_ok:
        print("\n✓ All systems are connected and working properly!")
        return True
    else:
        print("\n✗ Some tests failed.")
        return False

if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)