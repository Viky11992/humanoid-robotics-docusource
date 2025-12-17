---
id: constitution-20251216-001
title: Amend Constitution Spec Driven Book Creation
stage: constitution
date: 2025-12-16
surface: Project constitution update for AI/Spec-Driven Book Creation using Docusaurus and GitHub Pages
model: Claude Sonnet 4.5
feature: constitution
branch: 1-physical-ai-textbook
user: M.Muneeb
command: /sp.constitution
labels: [constitution, specification, docusaurus, github-pages, ai-assisted]
links:
  spec:
  ticket:
  adr:
  pr:
files:
  - path: .specify/memory/constitution.md
    action: modified
    summary: Updated constitution to reflect new project goals and principles
tests:
  - type: manual
    status: pending
    description: Verify constitution aligns with project requirements
---

## Prompt

Project: AI/Spec-Driven Book Creation using Docusaurus and GitHub Pages
Core principles:

Specification-first development: Every element of the book (structure, content, code, features) must be explicitly defined in specifications before implementation
AI-assisted but human-guided: Use Claude Code extensively for drafting, refactoring, and code generation, but all output must be reviewed, verified, and approved by the human author
Transparency and reproducibility: The entire creation process, including prompts used with Claude Code and specification evolution, must be documented so readers can replicate the workflow
Clarity and accessibility: Content written for developers with intermediate knowledge of JavaScript/React and Markdown, using clear explanations, examples, and progressive complexity
Open-source ethos: The complete book source is public, free to read, and encourages contributions

Key standards:

Built exclusively with Docusaurus (latest stable version) and standard plugins
Content written in Markdown/MDX with proper syntax highlighting for code blocks
All code examples must be functional, tested, and include setup instructions
Site must be fully responsive and accessible (WCAG AA compliant where practical)
Version control: All commits follow conventional commit messages; main branch always deployable
Documentation of AI usage: Include actual prompts and iterations where Claude Code contributed significantly

Constraints:

Tools restricted to: Spec-Kit Plus (for specification management), Claude Code (for AI assistance), Docusaurus, Git/GitHub, and standard free plugins
No external paid services or premium AI features beyond Claude Code
Book length: Minimum 10 chapters/sections with substantive content (approximately 20,000–40,000 words total)
Must include practical end-to-end tutorial: from initial spec to final deployed GitHub Pages site
Deployment target: GitHub Pages (using github.io domain or custom domain if configured)

Success criteria:

Live, publicly accessible book deployed on GitHub Page

## Response snapshot

Updated the constitution file at .specify/memory/constitution.md with the new project details. The changes included:

1. Updated the project title to "AI/Spec-Driven Book Creation using Docusaurus and GitHub Pages Constitution"
2. Replaced the old core principles with the new ones:
   - Specification-first development
   - AI-assisted but human-guided
   - Transparency and reproducibility
   - Clarity and accessibility
   - Open-source ethos
3. Updated the Key Standards section with the new requirements
4. Updated the Constraints section with the specified limitations
5. Updated the Success Criteria to match the requirement
6. Updated the version to 1.1.0 and the last amended date to today (2025-12-16)

The Sync Impact Report indicates this is a minor version update (1.0.0 → 1.1.0) due to the addition of new principles and updates to existing sections.

## Outcome

- ✅ Impact: Updated constitution to align with AI/Spec-Driven Book Creation project requirements
- 🧪 Tests: Manual verification needed
- 📁 Files: Modified .specify/memory/constitution.md
- 🔁 Next prompts: Plan implementation of the book creation process
- 🧠 Reflection: The constitution now properly reflects the project's core principles and requirements for specification-driven development with AI assistance

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable for this change
- Prompt variant (if applicable): Standard constitution update
- Next experiment (smallest change to try): Begin implementing the specification-driven book creation process