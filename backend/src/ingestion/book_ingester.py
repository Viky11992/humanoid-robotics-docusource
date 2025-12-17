import os
import asyncio
from typing import List, Dict, Any
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import logging
from src.services.rag_service import rag_service

logger = logging.getLogger(__name__)

class BookIngester:
    def __init__(self):
        self.supported_formats = {'.md', '.mdx', '.txt', '.html', '.htm'}
        self.rag_service = rag_service

    async def ingest_book_from_directory(self, directory_path: str, base_url: str = None) -> Dict[str, Any]:
        """
        Ingest an entire book from a directory of markdown files

        Args:
            directory_path: Path to directory containing book content
            base_url: Base URL for source references

        Returns:
            Dictionary with ingestion statistics
        """
        directory = Path(directory_path)
        if not directory.exists():
            raise ValueError(f"Directory does not exist: {directory_path}")

        files_processed = 0
        chunks_ingested = 0
        errors = []

        # Get all supported files
        files = []
        for ext in self.supported_formats:
            files.extend(directory.rglob(f"*{ext}"))

        for file_path in files:
            try:
                # Extract section information from file path
                relative_path = file_path.relative_to(directory)
                section = str(relative_path.parent) if relative_path.parent != Path('.') else 'root'

                # Extract title from filename
                title = file_path.stem.replace('_', ' ').replace('-', ' ').title()

                # Read and process the file
                content = await self._read_file(file_path)

                # Generate source URL if base_url is provided
                source_url = f"{base_url}/{relative_path}" if base_url else str(file_path)

                # Ingest the document
                success = await self.rag_service.ingest_document(
                    title=title,
                    content=content,
                    source_url=source_url,
                    section=section
                )

                if success:
                    files_processed += 1
                    # Count the chunks that were created
                    # (in a real implementation, we'd track this more precisely)
                else:
                    errors.append(f"Failed to ingest: {file_path}")

            except Exception as e:
                logger.error(f"Error ingesting file {file_path}: {str(e)}")
                errors.append(f"Error ingesting {file_path}: {str(e)}")

        return {
            "files_processed": files_processed,
            "chunks_ingested": chunks_ingested,
            "errors": errors,
            "status": "completed_with_errors" if errors else "completed"
        }

    async def ingest_single_file(self, file_path: str, title: str = None, source_url: str = None, section: str = None) -> bool:
        """
        Ingest a single file into the RAG system

        Args:
            file_path: Path to the file to ingest
            title: Title for the document (if not provided, derived from filename)
            source_url: URL for the source reference
            section: Section identifier

        Returns:
            True if successful, False otherwise
        """
        try:
            path = Path(file_path)
            if not path.exists():
                raise ValueError(f"File does not exist: {file_path}")

            # Use filename as title if not provided
            if not title:
                title = path.stem.replace('_', ' ').replace('-', ' ').title()

            # Read and process the file
            content = await self._read_file(path)

            # Ingest the document
            success = await self.rag_service.ingest_document(
                title=title,
                content=content,
                source_url=source_url,
                section=section
            )

            return success

        except Exception as e:
            logger.error(f"Error ingesting single file {file_path}: {str(e)}")
            return False

    async def _read_file(self, file_path: Path) -> str:
        """
        Read and extract text content from a file based on its extension

        Args:
            file_path: Path to the file to read

        Returns:
            Extracted text content
        """
        ext = file_path.suffix.lower()

        with open(file_path, 'r', encoding='utf-8') as f:
            raw_content = f.read()

        if ext in {'.md', '.mdx'}:
            # Convert markdown to plain text
            html = markdown.markdown(raw_content)
            soup = BeautifulSoup(html, 'html.parser')
            return soup.get_text()
        elif ext in {'.html', '.htm'}:
            # Extract text from HTML
            soup = BeautifulSoup(raw_content, 'html.parser')
            return soup.get_text()
        elif ext == '.txt':
            # Plain text file
            return raw_content
        else:
            # Default to plain text
            return raw_content

    async def ingest_from_text(self, title: str, content: str, source_url: str = None, section: str = None) -> bool:
        """
        Ingest content directly from text

        Args:
            title: Title for the document
            content: The content to ingest
            source_url: URL for the source reference
            section: Section identifier

        Returns:
            True if successful, False otherwise
        """
        try:
            success = await self.rag_service.ingest_document(
                title=title,
                content=content,
                source_url=source_url,
                section=section
            )
            return success
        except Exception as e:
            logger.error(f"Error ingesting text content: {str(e)}")
            return False

# Global instance
book_ingester = BookIngester()