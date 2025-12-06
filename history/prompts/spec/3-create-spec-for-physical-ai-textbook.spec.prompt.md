---
id: 3
title: Create spec for physical ai textbook
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-textbook
branch: 1-physical-ai-textbook
user: N/A
command: /sp.specify
labels: [spec]
links:
  spec: specs/1-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-physical-ai-textbook/spec.md
 - specs/1-physical-ai-textbook/checklists/requirements.md
tests: []
---

## Prompt

Textbook for Teaching Physical AI & Humanoid Robotics Course
Target audience: University-level students and educators with engineering, computer science, or related backgrounds; professionals seeking upskilling in robotics and AI integration
Focus: Building foundational knowledge in physical AI (e.g., embodiment, sensing, actuation) and humanoid robotics (e.g., design, control, human-robot interaction), with emphasis on practical skills, current research, and future trends like collaborative workspaces
Success criteria:

Identifies 5+ key applications of physical AI in humanoid robotics with supporting evidence and examples
Cites 30+ diverse sources, including at least 50% peer-reviewed articles or conference papers
Readers can design a basic humanoid robot simulation or explain integration of AI agents after completing the course
All technical claims supported by traceable evidence, with methodologies and limitations discussed
Successful deployment as a functional Docusaurus site with navigation, search, and interactive elements

Constraints:

Chapter count: 10-15 chapters, each 5,000-10,000 words including code, figures, and references
Total content equivalence: 200-300 printed pages
Format: Docusaurus markdown files with APA citations, deployed to GitHub Pages
Sources: Mix of peer-reviewed journals/conferences (e.g., IEEE, ICRA), textbooks, technical reports, and open-source repos; prioritize content from past 10 years where applicable
Tools restriction: Use only Spec-Kit Plus for specs and Claude Code for AI-assisted generation
Timeline: Complete draft in 4-6 weeks, with iterative reviews

Not building:

Comprehensive review of general AI software agents (covered in separate book)
Vendor-specific hardware or software comparisons
Discussion of non-robotics ethical issues (e.g., broad AI bias; focus only on robotics-specific ethics)
Physical prototypes, hardware builds, or deployment beyond static web site

## Response snapshot

Created specification for Physical AI & Humanoid Robotics Textbook. The spec includes user scenarios, requirements, edge cases, and success criteria. The spec is ready for planning. A Spec Quality Checklist has been created at specs/1-physical-ai-textbook/checklists/requirements.md. The branch 1-physical-ai-textbook has been created and checked out.

## Outcome

- ✅ Impact: Major
- 🧪 Tests: N/A
- 📁 Files: specs/1-physical-ai-textbook/spec.md, specs/1-physical-ai-textbook/checklists/requirements.md
- 🔁 Next prompts: /sp.plan or /sp.clarify
- 🧠 Reflection: N/A

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A