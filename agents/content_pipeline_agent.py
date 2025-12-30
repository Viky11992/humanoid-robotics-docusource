"""
Content Pipeline Agent
Automates the entire content processing pipeline from new documentation to RAG system integration
"""

import os
import asyncio
import logging
from pathlib import Path
from typing import List, Dict, Optional
import markdown
from bs4 import BeautifulSoup
import re
from dataclasses import dataclass
from datetime import datetime

# Import existing backend services
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))
from backend.src.services.qdrant_service import QdrantService
from backend.src.services.cohere_service import CohereService
from backend.src.models.document import Document


@dataclass
class ContentChunk:
    id: str
    content: str
    metadata: Dict
    embedding: Optional[List[float]] = None


class ContentPipelineAgent:
    def __init__(self):
        self.qdrant_service = QdrantService()
        self.cohere_service = CohereService()
        self.logger = logging.getLogger(__name__)
        self.content_dir = Path("my-website/docs")

    async def monitor_content_changes(self):
        """Monitor the docs directory for new or updated content"""
        self.logger.info("Starting content monitoring...")

        # Process all existing content first
        await self.process_all_content()

        # In a real implementation, this would use file system watchers
        # For now, we'll just process all content
        self.logger.info("Content monitoring started")

    async def process_all_content(self):
        """Process all content in the documentation directory"""
        markdown_files = list(self.content_dir.rglob("*.md"))
        self.logger.info(f"Found {len(markdown_files)} markdown files to process")

        for file_path in markdown_files:
            await self.process_file(file_path)

    async def process_file(self, file_path: Path):
        """Process a single markdown file"""
        try:
            self.logger.info(f"Processing file: {file_path}")

            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Parse frontmatter and content
            frontmatter, body = self.parse_frontmatter(content)

            # Extract content sections
            sections = self.extract_content_sections(body, file_path)

            # Process each section as a chunk
            for i, section in enumerate(sections):
                chunk_id = f"{file_path.stem}_section_{i}"

                # Create metadata
                metadata = {
                    "source_file": str(file_path),
                    "section_title": section.get("title", ""),
                    "module": self.extract_module_from_path(file_path),
                    "created_at": datetime.now().isoformat(),
                    "updated_at": datetime.now().isoformat(),
                    **frontmatter  # Include frontmatter in metadata
                }

                # Create content chunk
                chunk = ContentChunk(
                    id=chunk_id,
                    content=section["content"],
                    metadata=metadata
                )

                # Generate embedding
                embedding = await self.cohere_service.embed_text(section["content"])
                chunk.embedding = embedding

                # Store in vector database
                await self.qdrant_service.upsert_document(chunk)

                self.logger.info(f"Stored chunk: {chunk_id}")

        except Exception as e:
            self.logger.error(f"Error processing file {file_path}: {str(e)}")

    def parse_frontmatter(self, content: str) -> tuple[Dict, str]:
        """Parse YAML frontmatter from markdown content"""
        frontmatter = {}
        body = content

        # Look for YAML frontmatter between --- delimiters
        pattern = r'^---\n(.*?)\n---\n(.*)'
        match = re.match(pattern, content, re.DOTALL)

        if match:
            frontmatter_text = match.group(1)
            body = match.group(2)

            # Simple YAML parsing (in a real implementation, use pyyaml)
            for line in frontmatter_text.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip()

        return frontmatter, body

    def extract_content_sections(self, content: str, file_path: Path) -> List[Dict[str, str]]:
        """Extract content sections from markdown, with context-aware chunking"""
        # Convert markdown to HTML to better understand structure
        html = markdown.markdown(content)
        soup = BeautifulSoup(html, 'html.parser')

        sections = []
        current_title = "Overview"

        # Split content by headings to maintain context
        elements = soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'p', 'li', 'pre'])

        current_section_content = []

        for element in elements:
            if element.name and element.name.startswith('h'):
                # If we have accumulated content, save the previous section
                if current_section_content:
                    section_text = ' '.join(current_section_content).strip()
                    if len(section_text) > 50:  # Only save substantial sections
                        sections.append({
                            "title": current_title,
                            "content": section_text
                        })
                        current_section_content = []

                # Set the new title
                current_title = element.get_text().strip()
            else:
                # Add content to current section
                text = element.get_text().strip()
                if text:
                    current_section_content.append(text)

        # Add the final section if there's content
        if current_section_content:
            section_text = ' '.join(current_section_content).strip()
            if len(section_text) > 50:
                sections.append({
                    "title": current_title,
                    "content": section_text
                })

        # If no sections were created, use the whole content as one section
        if not sections:
            sections.append({
                "title": file_path.stem,
                "content": content
            })

        return sections

    def extract_module_from_path(self, file_path: Path) -> str:
        """Extract module name from file path"""
        parts = file_path.parts
        for part in parts:
            if 'module' in part.lower():
                return part
        return 'general'

    async def validate_content(self, content: str) -> Dict[str, bool]:
        """Validate content formatting and structure"""
        validation_results = {
            'has_frontmatter': '---' in content[:500],  # Check first 500 chars for frontmatter
            'valid_markdown': True,  # Simplified check
            'has_headings': bool(re.search(r'#{1,6}\s', content)),
            'no_broken_links': True,  # Would implement proper link checking
        }

        # Additional validation could be added here
        return validation_results

    async def rebuild_index(self):
        """Rebuild the entire content index"""
        self.logger.info("Rebuilding content index...")

        # Clear existing index
        await self.qdrant_service.clear_collection()

        # Process all content
        await self.process_all_content()

        self.logger.info("Content index rebuilt successfully")


# Example usage
async def main():
    agent = ContentPipelineAgent()
    await agent.process_all_content()
    print("Content pipeline completed!")


if __name__ == "__main__":
    asyncio.run(main())