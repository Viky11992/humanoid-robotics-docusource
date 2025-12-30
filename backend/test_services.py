import asyncio
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

def test_services():
    """Test if all required services can be imported and initialized"""
    print("Testing service initialization...")

    try:
        # Test importing the main components
        from src.services.cohere_service import cohere_service
        print("[SUCCESS] Cohere service imported successfully")

        from src.services.qdrant_service import qdrant_service
        print("[SUCCESS] Qdrant service imported successfully")

        from src.services.postgres_service import postgres_service
        print("[SUCCESS] Postgres service imported successfully")

        from src.services.rag_service import rag_service
        print("[SUCCESS] RAG service imported successfully")

        # Test if services have the required methods
        print("\nTesting service methods...")

        # Test cohere service
        if hasattr(cohere_service, 'generate_embeddings'):
            print("[SUCCESS] Cohere service has generate_embeddings method")
        else:
            print("[ERROR] Cohere service missing generate_embeddings method")

        if hasattr(cohere_service, 'generate_response'):
            print("[SUCCESS] Cohere service has generate_response method")
        else:
            print("[ERROR] Cohere service missing generate_response method")

        # Test qdrant service
        if hasattr(qdrant_service, 'search_similar'):
            print("[SUCCESS] Qdrant service has search_similar method")
        else:
            print("[ERROR] Qdrant service missing search_similar method")

        if hasattr(qdrant_service, 'store_text_chunks_with_embeddings'):
            print("[SUCCESS] Qdrant service has store_text_chunks_with_embeddings method")
        else:
            print("[ERROR] Qdrant service missing store_text_chunks_with_embeddings method")

        # Test postgres service
        if hasattr(postgres_service, 'create_session'):
            print("[SUCCESS] Postgres service has create_session method")
        else:
            print("[ERROR] Postgres service missing create_session method")

        print("\nAll services are properly initialized!")
        return True

    except Exception as e:
        print(f"[ERROR] Error initializing services: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def test_rag_process():
    """Test the RAG process method directly"""
    print("\nTesting RAG process method...")

    try:
        from src.services.rag_service import rag_service

        # Test with minimal parameters
        async def test_async():
            try:
                result = await rag_service.process_query(
                    query="test",
                    session_id=None,
                    selected_text=None,
                    context_restrict=False,
                    max_results=1,
                    similarity_threshold=0.5
                )
                print(f"[SUCCESS] RAG process completed: {type(result)}")
                return result
            except Exception as e:
                print(f"[ERROR] RAG process failed: {str(e)}")
                import traceback
                traceback.print_exc()
                return None

        result = asyncio.run(test_async())
        return result is not None

    except Exception as e:
        print(f"[ERROR] Error in RAG test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Running service tests...")
    services_ok = test_services()

    if services_ok:
        rag_ok = test_rag_process()
        if rag_ok:
            print("\n[SUCCESS] All tests passed!")
        else:
            print("\n[ERROR] RAG process test failed")
    else:
        print("\n[ERROR] Service initialization failed")