#!/usr/bin/env python3
"""
Script to ingest documentation files from the my-website/docs directory into the RAG system.
This will allow the chatbot to answer questions based on the book content.
"""

import asyncio
import os
import sys
from pathlib import Path
import re
from typing import List, Tuple

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.rag_service import rag_service
from src.utils.text_processor import chunk_text


def extract_title_from_md(content: str) -> str:
    """Extract title from markdown content, either from frontmatter or first heading."""
    # Try to extract title from frontmatter
    frontmatter_match = re.search(r'^---\s*\n(.*?)\n---', content, re.DOTALL)
    if frontmatter_match:
        frontmatter = frontmatter_match.group(1)
        title_match = re.search(r'title:\s*["\']?(.*?)(?:["\']?\s*$|["\']?\s+)', frontmatter)
        if title_match:
            return title_match.group(1).strip()

    # If no title in frontmatter, try first ## heading
    lines = content.split('\n')
    for line in lines:
        if line.startswith('# '):
            return line[2:].strip()
        elif line.startswith('## '):
            return line[3:].strip()

    return "Untitled Document"


def read_markdown_files(docs_dir: str) -> List[Tuple[str, str, str]]:
    """Read all markdown files from the docs directory and its subdirectories."""
    files_content = []
    docs_path = Path(docs_dir)

    for md_file in docs_path.rglob("*.md"):
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()
            title = extract_title_from_md(content)
            relative_path = md_file.relative_to(docs_path.parent)  # Include parent directory in path
            files_content.append((title, content, str(relative_path)))

    return files_content


async def ingest_documents():
    """Ingest all documentation files into the RAG system."""
    print("Starting document ingestion...")

    # Path to the docs directory (relative to the project root)
    docs_dir = os.path.join(os.path.dirname(__file__), '..', 'my-website', 'docs')

    if not os.path.exists(docs_dir):
        print(f"Error: Docs directory not found at {docs_dir}")
        return

    print(f"Reading markdown files from {docs_dir}...")
    files_content = read_markdown_files(docs_dir)

    print(f"Found {len(files_content)} markdown files to ingest")

    success_count = 0
    for i, (title, content, source_path) in enumerate(files_content, 1):
        print(f"Ingesting ({i}/{len(files_content)}): {title}")

        try:
            # Use the RAG service to ingest the document
            success = await rag_service.ingest_document(
                title=title,
                content=content,
                source_url=source_path,  # Using file path as source
                section=os.path.dirname(source_path)  # Using directory as section
            )

            if success:
                success_count += 1
                print(f"  SUCCESS: Successfully ingested: {title}")
            else:
                print(f"  FAILED: Failed to ingest: {title}")

        except Exception as e:
            print(f"  ERROR: Error ingesting {title}: {str(e)}")

    print(f"\nIngestion completed! Successfully ingested {success_count}/{len(files_content)} documents.")


async def main():
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    # Initialize the PostgreSQL service first
    from src.services.postgres_service import postgres_service
    try:
        await postgres_service.initialize()
        print("PostgreSQL service initialized successfully")
    except Exception as e:
        print(f"Error initializing PostgreSQL service: {str(e)}")
        return

    # Run the ingestion
    await ingest_documents()

    # Close the PostgreSQL connection pool
    await postgres_service.close()


if __name__ == "__main__":
    asyncio.run(main())