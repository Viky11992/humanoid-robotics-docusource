import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def check_qdrant_api():
    """Check what methods are available on the Qdrant client"""
    from dotenv import load_dotenv
    load_dotenv()

    from src.services.qdrant_service import qdrant_service

    print("Qdrant client type:", type(qdrant_service.client))
    print("Qdrant client attributes:", [attr for attr in dir(qdrant_service.client) if not attr.startswith('_')])

    # Check if search method exists
    has_search = hasattr(qdrant_service.client, 'search')
    print(f"Has 'search' method: {has_search}")

    # Check for similar method names
    potential_methods = [attr for attr in dir(qdrant_service.client) if 'search' in attr.lower() or 'find' in attr.lower() or 'query' in attr.lower()]
    print(f"Potential search-related methods: {potential_methods}")

    # Check for collection methods
    collection_methods = [attr for attr in dir(qdrant_service.client) if 'collection' in attr.lower()]
    print(f"Collection-related methods: {collection_methods}")

if __name__ == "__main__":
    check_qdrant_api()