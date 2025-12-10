from typing import List, Dict, Any
import re

class TextChunker:
    """
    Utility for processing book content and breaking it into chunks for embedding.
    """

    def __init__(self, chunk_size: int = 1000, overlap: int = 100):
        """
        Initialize the text chunker.

        Args:
            chunk_size: Maximum size of each chunk in characters
            overlap: Number of overlapping characters between chunks
        """
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk_text(self, text: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Break text into overlapping chunks.

        Args:
            text: The text to be chunked
            metadata: Additional metadata to include with each chunk

        Returns:
            List of dictionaries containing the chunk text and associated metadata
        """
        if not text:
            return []

        if metadata is None:
            metadata = {}

        chunks = []
        start_idx = 0

        while start_idx < len(text):
            # Determine the end index for this chunk
            end_idx = start_idx + self.chunk_size

            # If this is the last chunk, include the remainder
            if end_idx >= len(text):
                end_idx = len(text)
            else:
                # Try to break at sentence boundary
                temp_end = end_idx
                while temp_end < len(text) and text[temp_end] not in '.!?':
                    temp_end += 1

                # If no sentence boundary found, break at the original end point
                if temp_end < len(text) and text[temp_end] in '.!?':
                    end_idx = temp_end + 1  # Include the punctuation
                else:
                    end_idx = temp_end

            # Extract the chunk
            chunk_text = text[start_idx:end_idx]

            # Create chunk with metadata
            chunk = {
                "text": chunk_text,
                "start_idx": start_idx,
                "end_idx": end_idx,
                "metadata": metadata.copy()
            }

            chunks.append(chunk)

            # Move to the next chunk with overlap
            start_idx = end_idx - self.overlap

            # If we're at the end, break
            if end_idx >= len(text):
                break

        return chunks

    def chunk_by_paragraphs(self, text: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Break text into chunks by paragraphs, falling back to character-based chunking if paragraphs are too large.

        Args:
            text: The text to be chunked
            metadata: Additional metadata to include with each chunk

        Returns:
            List of dictionaries containing the chunk text and associated metadata
        """
        if not text:
            return []

        if metadata is None:
            metadata = {}

        # Split by paragraphs
        paragraphs = re.split(r'\n\s*\n', text)
        chunks = []

        for paragraph in paragraphs:
            if len(paragraph) <= self.chunk_size:
                # Paragraph fits in a single chunk
                chunks.append({
                    "text": paragraph,
                    "metadata": metadata.copy()
                })
            else:
                # Paragraph is too large, chunk it by characters
                sub_chunks = self.chunk_text(paragraph, metadata)
                chunks.extend(sub_chunks)

        return chunks


# Global instance
text_chunker = TextChunker()