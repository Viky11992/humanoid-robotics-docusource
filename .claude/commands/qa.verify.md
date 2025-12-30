---
description: Validate AI response quality and accuracy for the book project
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

Given the user arguments, implement response quality validation for the Physical AI & Humanoid Robotics book project:

1. **Parse arguments** to determine validation mode:
   - `--response "<text>"`: Validate specific response text
   - `--conversation <id>`: Validate entire conversation by ID
   - `--audit [--days N]`: Audit recent responses (default: last 7 days)
   - Default: Show usage information

2. **Validation Process**:
   - Cross-reference response content with source documentation
   - Check citation accuracy and relevance
   - Detect potential hallucinations or fabrications
   - Verify technical accuracy against book content
   - Assess educational value and clarity

3. **Quality Metrics**:
   - Accuracy score (0-100%)
   - Citation validity (proper source references)
   - Hallucination detection (factual accuracy)
   - Technical correctness (domain-specific accuracy)
   - Educational value (helpfulness for learning)

4. **Integration with Existing System**:
   - Access conversation history from PostgreSQL database
   - Cross-reference with documentation in Qdrant vector store
   - Use existing Cohere integration for semantic validation
   - Leverage existing backend services

5. **Validation Steps**:
   - Extract key claims from the response
   - Search documentation for supporting evidence
   - Flag inconsistencies or unsupported claims
   - Verify technical terms and concepts match book content
   - Assess response completeness and relevance

6. **Output**:
   - Quality score and detailed breakdown
   - List of potential issues or concerns
   - Suggested improvements if needed
   - Summary of validation results

## Implementation Notes

- Use the existing chat and RAG functionality from `backend/src/api/agent_router.py`
- Access conversation data through existing database models
- Leverage the same retrieval mechanism as the RAG system
- Follow existing error handling and logging patterns
- Maintain compatibility with current response format

---