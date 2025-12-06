---
id: 1
title: Update project constitution with details
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: N/A
command: /sp.constitution
labels: [constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 -  .specify/memory/constitution.md
tests: []
---

## Prompt

Project: Create a Textbook for Teaching Physical AI & Humanoid Robotics Course
Core principles:

Accuracy through expert source verification and integration of current research in robotics and AI
Clarity for educational audience (university-level students with engineering or computer science background)
Comprehensiveness (covering theoretical foundations, practical applications, and ethical considerations)
Practicality (include code examples, simulations, and deployable resources for hands-on learning)
Innovation (emphasize emerging trends in physical AI and humanoid robotics, such as human-robot collaboration)

Key standards:

All technical claims must be traceable to sources, with explanations of methodologies and limitations
Citation format: APA style for references, with in-text citations for all factual assertions
Source types: minimum 50% peer-reviewed articles or conference papers (e.g., from IEEE, ICRA, or NeurIPS); include reputable textbooks, technical reports, and open-source repositories
Plagiarism check: 0% tolerance before final deployment; use tools like Turnitin or Grammarly for verification
Writing clarity: Flesch-Kincaid grade 12-14, with technical terms defined on first use and accompanied by diagrams or examples
Code and tools integration: Use Spec-Kit Plus for specification-driven development and Claude Code for AI-assisted coding; ensure all code snippets are executable and version-controlled
Structure: Organize as a Docusaurus site with markdown chapters, navigation, search functionality, and multimedia embeds (e.g., videos of robot demos)

Constraints:

Chapter count: 10-15 chapters, covering topics from basics (e.g., kinematics, sensors) to advanced (e.g., AI integration, humanoid ethics)
Total content length: Equivalent to 200-300 pages if printed, with each chapter 5,000-10,000 words including code and figures
Minimum 30 sources, diversified across academic, industry, and open-source domains
Format: Docusaurus markdown files, deployed to GitHub Pages as a static site; include responsive design for mobile access
Tools restriction: Exclusively use Spec-Kit Plus for project specs and Claude Code for generation; no external unpaid APIs or proprietary software
Timeline: Complete draft in 4-6 weeks, with iterative reviews

Success criteria:

All content verified against sources for technical accuracy and relevance
Zero plagiarism detected in final checks
Successful deployment to GitHub Pages with functional navigation, search, and no broken links
Passes peer review simulation (e.g., feedback from 3-5 experts in robotics/AI)
Educational impact: Includes at least 5 interactive elements (e.g., code playgrounds via embedded Jupyter or simulations) and receives positive usability feedback
Completeness: Covers all required topics, with glossary, index, and supplementary resources (e.g., datasets, models) linked or hosted

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md. The version remains 1.0.0. Templates requiring updates: .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md, .specify/templates/commands/*.md, README.md, docs/quickstart.md. Suggested commit message: `docs: update constitution`

## Outcome

- ✅ Impact: Minor
- 🧪 Tests: N/A
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: N/A
- 🧠 Reflection: N/A

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A