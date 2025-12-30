import sys
import os
import asyncio

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

async def test_direct_rag():
    """Test the RAG service directly without API layers"""
    from dotenv import load_dotenv
    load_dotenv()

    # Initialize services
    from src.services.postgres_service import postgres_service
    from src.services.rag_service import rag_service

    print("Initializing PostgreSQL service...")
    await postgres_service.initialize()
    print("PostgreSQL service initialized successfully!")

    print("\nTesting RAG service directly...")
    try:
        import uuid
        # Generate a proper UUID for the session
        proper_session_id = str(uuid.uuid4())

        result = await rag_service.process_query(
            query="What is Physical AI?",
            session_id=proper_session_id,
            selected_text=None,
            context_restrict=False,
            max_results=5,
            similarity_threshold=0.5
        )
        print("SUCCESS! RAG service returned:")
        print(f"Response: {result['response'][:200]}...")
        print(f"Sources: {len(result['sources'])} sources found")
        print(f"Query time: {result['query_time']:.2f}s")
        return True
    except Exception as e:
        print(f"ERROR in RAG service: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_direct_rag())
    if success:
        print("\n✅ RAG service is working correctly!")
    else:
        print("\n❌ RAG service has issues")