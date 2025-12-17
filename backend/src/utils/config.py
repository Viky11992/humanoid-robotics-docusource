# Configuration constants for text processing

# Chunk size configuration (in words/approximate tokens)
CHUNK_SIZE_MIN = 50
CHUNK_SIZE_MAX = 1024
DEFAULT_CHUNK_SIZE = 512

# Overlap configuration
OVERLAP_SIZE = 50

# Text validation
MIN_CONTENT_LENGTH = 10
MAX_CONTENT_LENGTH = 10000  # Maximum content length in characters

# API configuration
COHERE_MAX_INPUT_LENGTH = 5000  # Maximum text length for Cohere API