import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

def get_qdrant_client():
    """
    Creates and returns a Qdrant client instance based on environment configuration.
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if qdrant_url and qdrant_api_key:
        # Connect to Qdrant Cloud
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )
    elif os.getenv("QDRANT_HOST"):
        # Connect to local Qdrant instance
        client = QdrantClient(
            host=os.getenv("QDRANT_HOST", "localhost"),
            port=int(os.getenv("QDRANT_PORT", 6333)),
        )
    else:
        # Connect to local Qdrant instance (default)
        client = QdrantClient(
            host="localhost",
            port=6333,
        )

    return client