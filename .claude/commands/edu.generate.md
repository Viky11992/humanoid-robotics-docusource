---
description: Generate educational materials and exercises for book chapters
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

Given the user arguments, implement educational content generation for the Physical AI & Humanoid Robotics book project:

1. **Parse arguments** to determine generation mode:
   - `--type <type> --target <path>`: Generate specific type for target content
     - Types: `questions`, `exercises`, `summary`, `objectives`, `assessments`
   - `--difficulty <level>`: Set difficulty level (beginner, intermediate, advanced)
   - `--topic <topic>`: Generate content focused on specific topic
   - Default: Show usage information

2. **Content Generation Process**:
   - For questions: Generate multiple-choice, short-answer, and practical questions
   - For exercises: Create hands-on robotics exercises and practical tasks
   - For summaries: Create chapter/module summaries and key points
   - For objectives: Define learning objectives and outcomes
   - For assessments: Generate quizzes and evaluation materials

3. **Educational Standards**:
   - Align with Bloom's taxonomy levels
   - Match difficulty to target audience (developers, engineers, students)
   - Follow educational best practices for technical content
   - Ensure content is relevant to robotics and AI concepts
   - Maintain consistency with book's educational approach

4. **Integration with Existing System**:
   - Access content from documentation structure in `my-website/docs/`
   - Use existing content as context for generation
   - Follow Docusaurus markdown format for output
   - Maintain compatibility with existing educational content

5. **Generation Steps**:
   - Analyze source content structure and key concepts
   - Identify important topics and learning points
   - Generate content appropriate for specified difficulty level
   - Ensure technical accuracy based on book content
   - Format output in proper markdown for Docusaurus

6. **Output**:
   - Generated educational content in proper format
   - Suggested placement within documentation structure
   - Quality assessment of generated content
   - Recommendations for refinement if needed

## Implementation Notes

- Use the existing documentation structure as reference material
- Follow the same technical accuracy standards as the book content
- Generate content that enhances learning experience
- Maintain consistency with the book's educational philosophy
- Ensure generated content is actionable and practical

---