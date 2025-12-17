import re
from typing import List, Tuple
import hashlib
from .config import CHUNK_SIZE_MIN, CHUNK_SIZE_MAX, OVERLAP_SIZE

def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """
    Split text into chunks of specified size with overlap

    Args:
        text: The text to chunk
        chunk_size: Size of each chunk in tokens/approximate words
        overlap: Number of tokens to overlap between chunks

    Returns:
        List of text chunks
    """
    # Simple approximation: assume 1 word = 1.3 tokens on average
    # For a more accurate approach, we'd use a tokenizer, but for now:
    # Split by sentences and group them into chunks
    sentences = re.split(r'[.!?]+', text)
    sentences = [s.strip() for s in sentences if s.strip()]

    chunks = []
    current_chunk = ""

    for sentence in sentences:
        # Check if adding this sentence would exceed chunk size
        test_chunk = current_chunk + " " + sentence if current_chunk else sentence

        # Simple word count as approximation
        word_count = len(test_chunk.split())

        if word_count <= chunk_size:
            current_chunk = test_chunk
        else:
            if current_chunk:  # If there's content in the current chunk
                chunks.append(current_chunk.strip())

            # Start a new chunk with the current sentence
            # If the sentence itself is too long, we'll split it
            if len(sentence.split()) > chunk_size:
                # Split long sentence into smaller pieces
                sentence_parts = split_long_sentence(sentence, chunk_size)
                for part in sentence_parts[:-1]:
                    chunks.append(part)
                current_chunk = sentence_parts[-1]
            else:
                current_chunk = sentence

    # Add the last chunk if it exists
    if current_chunk:
        chunks.append(current_chunk.strip())

    return chunks

def split_long_sentence(sentence: str, max_size: int) -> List[str]:
    """
    Split a long sentence into smaller parts

    Args:
        sentence: The sentence to split
        max_size: Maximum size for each part

    Returns:
        List of sentence parts
    """
    words = sentence.split()
    if len(words) <= max_size:
        return [sentence]

    parts = []
    current_part = []

    for word in words:
        if len(current_part) < max_size:
            current_part.append(word)
        else:
            parts.append(" ".join(current_part))
            current_part = [word]

    if current_part:
        parts.append(" ".join(current_part))

    return parts

def validate_chunk(chunk: str, min_size: int = 50, max_size: int = 1024) -> bool:
    """
    Validate that a text chunk meets size requirements

    Args:
        chunk: The text chunk to validate
        min_size: Minimum number of words/characters
        max_size: Maximum number of words/characters

    Returns:
        True if valid, False otherwise
    """
    word_count = len(chunk.split())
    return min_size <= word_count <= max_size

def generate_content_hash(content: str) -> str:
    """
    Generate a SHA256 hash for content deduplication

    Args:
        content: The content to hash

    Returns:
        SHA256 hash as hex string
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()

def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and normalizing

    Args:
        text: The text to clean

    Returns:
        Cleaned text
    """
    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)
    # Remove special characters that might cause issues
    text = re.sub(r'[^\w\s\.\,\!\?\;\:\-\n\r]', ' ', text)
    return text.strip()

def extract_metadata(text: str, source: str = None) -> dict:
    """
    Extract basic metadata from text

    Args:
        text: The text to analyze
        source: Source identifier

    Returns:
        Dictionary with metadata
    """
    word_count = len(text.split())
    char_count = len(text)

    return {
        "word_count": word_count,
        "char_count": char_count,
        "source": source,
        "created_at": None  # Will be set by the caller
    }