---
description: Optimize RAG system performance and response quality
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

Given the user arguments, implement RAG system optimization for the Physical AI & Humanoid Robotics book project:

1. **Parse arguments** to determine optimization mode:
   - `--auto`: Automatic parameter optimization
   - `--retrieval`: Optimize similarity search parameters
   - `--context`: Optimize context window usage
   - `--report`: Generate performance optimization report
   - `--benchmark`: Run performance benchmarks
   - Default: Show usage information

2. **Optimization Components**:
   - Vector search parameters (similarity thresholds, result counts)
   - Context window management (chunk size, overlap, selection)
   - Embedding parameters (model, dimension, normalization)
   - Response generation parameters (temperature, context usage)
   - Performance metrics (latency, accuracy, resource usage)

3. **Optimization Process**:
   - Analyze current system performance metrics
   - Test different parameter combinations
   - Measure response quality and accuracy
   - Evaluate performance trade-offs
   - Identify optimal configurations

4. **Integration with Existing System**:
   - Use existing RAG implementation from `backend/src/services/rag_service.py`
   - Access Cohere integration for embedding optimization
   - Use Qdrant service for search parameter tuning
   - Follow existing response generation patterns

5. **Tuning Steps**:
   - Evaluate current retrieval effectiveness
   - Test different similarity thresholds
   - Optimize chunk size and overlap for content
   - Adjust context selection algorithms
   - Fine-tune response generation parameters

6. **Output**:
   - Current performance metrics and baseline
   - Recommended parameter adjustments
   - Expected performance improvements
   - Implementation instructions for changes
   - Performance benchmark comparison

## Implementation Notes

- Use the existing RAG system as the foundation for optimization
- Follow the same retrieval and generation patterns as current system
- Maintain compatibility with existing API endpoints
- Ensure optimizations don't compromise response quality
- Provide measurable improvements in performance metrics

---