---
description: Complete content ingestion and processing workflow for book documentation
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

Given the user arguments, implement a complete content processing pipeline for the Physical AI & Humanoid Robotics book project:

1. **Parse arguments** to determine operation mode:
   - `--process <path>`: Process specific file or directory
   - `--file <file>`: Process specific file
   - `--validate`: Validate content before processing
   - `--rebuild`: Rebuild entire content index
   - Default: Show usage information

2. **Content Processing Flow**:
   - For files: Read Markdown content and validate structure
   - For directories: Recursively find all .md files
   - Parse frontmatter and content sections
   - Chunk content with context-aware boundaries
   - Generate embeddings using Cohere API
   - Store in Qdrant vector database with metadata

3. **Validation Steps**:
   - Check file format and structure
   - Verify links and references
   - Validate Markdown syntax
   - Ensure content fits project guidelines

4. **Integration with Existing System**:
   - Use existing backend services from `backend/src/`
   - Leverage Cohere integration (`backend/src/services/cohere_service.py`)
   - Use Qdrant vector database (`backend/src/services/qdrant_service.py`)
   - Maintain compatibility with Docusaurus structure

5. **Error Handling**:
   - Log processing errors with file paths
   - Continue processing other files if one fails
   - Report summary of successes/failures
   - Provide detailed error information for debugging

6. **Output**:
   - Show processing progress
   - Report number of files processed
   - List any errors encountered
   - Confirm successful storage in vector database

## Implementation Notes

- Use the existing ingestion system from `backend/src/api/ingestion_router.py`
- Follow the same chunking strategy as the existing RAG system
- Maintain consistency with existing metadata schema
- Preserve content hierarchy and relationships
- Ensure content is properly indexed for search

---