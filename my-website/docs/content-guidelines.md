# Docusaurus Markdown Content Guidelines

This document outlines the guidelines for structuring and formatting Markdown content within the "Physical AI & Humanoid Robotics" textbook to ensure consistency, clarity, and maintainability.

## 1. File Naming and Location

*   All chapter files should reside in the `my-website/docs/` directory.
*   Organize chapters into subdirectories (e.g., `my-website/docs/chapter1/`).
*   The main content file for each chapter subdirectory should be `index.md` (or `index.mdx` if using MDX).
*   Use `kebab-case` for file and directory names (e.g., `introduction-to-ai.md`).

## 2. Frontmatter (Required for each chapter/document)

Every Markdown file representing a chapter or major section **must** include frontmatter at the top.

*   **`title` (Required)**: The main title of the chapter/document. This will be displayed in the navigation and page header.
*   **`description` (Required)**: A brief summary of the chapter's content, used for SEO and page previews.
*   **`sidebar_label` (Optional)**: If different from `title`, this text will appear in the sidebar navigation.
*   **`slug` (Optional)**: Custom URL slug for the page. If not provided, Docusaurus will generate one from the title.
*   **`position` (Optional)**: Numeric value to control the order of items in the sidebar. Lower numbers appear higher.
*   **`id` (Required for unique identification in sidebar)**: A unique identifier for the document, typically the filename without the extension.

**Example Frontmatter:**

```markdown
---
title: "Introduction to Physical AI and Humanoid Robotics"
description: "An overview of foundational concepts in physical AI, humanoid robotics, and their historical context."
sidebar_label: "Introduction"
slug: /intro
position: 1
id: intro
---
```

## 3. Headings

Maintain a consistent heading hierarchy to ensure readability and accessibility.

*   **`#` (H1)**: Reserved for the `title` defined in the frontmatter. Do not use H1 within the document body.
*   **`##` (H2)**: Main sections within a chapter (e.g., "What is Physical AI?", "Components of a Humanoid Robot").
*   **`###` (H3)**: Subsections within an H2 section.
*   **`####` (H4)**: Further subdivisions (use sparingly).

**Example Heading Structure:**

```markdown
---
title: "Chapter Title"
---

## Main Section 1
### Subsection 1.1
#### Sub-subsection 1.1.1

## Main Section 2
```

## 4. Paragraphs and Text Formatting

*   **Line Breaks**: Use single line breaks between paragraphs for better readability.
*   **Bold**: `**bold text**` or `__bold text__` for emphasis.
*   **Italic**: `*italic text*` or `_italic text_` for emphasis.
*   **Code Snippets (inline)**: Use backticks (`` `code` ``) for inline code.
*   **Blockquotes**: Use `>` for quoting text.

## 5. Code Blocks

Use fenced code blocks with language identifiers for syntax highlighting.

*   **Standard Code Block**:
    ````markdown
    ```python
    def hello_world():
        print("Hello, Physical AI!")
    ```
    ````

*   **Line Highlighting**: Highlight specific lines using `{}` after the language.
    ````markdown
    ```python {2,4}
    def calculate_force(mass, acceleration):
        force = mass * acceleration # Highlight this line
        return force
    ```
    ````

*   **Interactive Code Block (for JS/React)**: Use `live` after the language for interactive playgrounds (requires Docusaurus live codeblock plugin).
    ````markdown
    ```jsx live
    import React from 'react';
    function Counter() {
      const [count, setCount] = React.useState(0);
      return (
        <div>
          <p>You clicked {count} times</p>
          <button onClick={() => setCount(count + 1)}>
            Click me
          </button>
        </div>
      );
    }
    <Counter />
    ```
    ````

## 6. Images and Multimedia

*   Store images in `my-website/static/img/`.
*   Use relative paths from the root of the Docusaurus site.
*   Provide descriptive alt text for accessibility.

**Example Image:**

````markdown
![Diagram of a robotic arm's kinematic chain](img/robot-arm-kinematics.png)
````

## 7. Links

*   **Internal Links**: Use relative paths to other Docusaurus documents.
    ````markdown
    [Introduction Chapter](/docs/intro)
    ````
*   **External Links**: Use full URLs.
    ````markdown
    [Official Docusaurus Website](https://docusaurus.io/)
    ````
*   **Reference Links**: Link to APA citations as defined in `docs/references.md`.

## 8. Lists

*   **Unordered Lists**: Use `*`, `-`, or `+`.
    ```markdown
    * Item 1
    * Item 2
    ```
*   **Ordered Lists**: Use numbers followed by a period.
    ```markdown
    1. First item
    2. Second item
    ```

## 9. Admonitions (Info, Warning, Danger, etc.)

Use Docusaurus admonitions for important notes, tips, warnings, etc.

````markdown
:::tip
This is a helpful tip for understanding kinematics.
:::

:::warning
Be cautious when implementing control systems without proper safety checks.
:::
````

## 10. Tables

Use Markdown table syntax for tabular data.

````markdown
| Concept       | Description                            | Example        |
|---------------|----------------------------------------|----------------|
| Kinematics    | Study of motion without forces         | Joint angles   |
| Dynamics      | Study of motion with forces            | Torque control |
````

## 12. Defining Technical Terms

All technical terms and jargon **must be defined clearly on their first use** within each chapter. This ensures that readers with varying backgrounds can understand the content without needing to consult external resources.

*   **Provide a concise definition**: Explain the term in simple language.
*   **Use inline code**: For programming-related terms, use inline code formatting (e.g., `` `kinematics` ``).
*   **Refer to the Glossary**: For a comprehensive list of all defined terms, refer readers to the [Glossary](/docs/glossary).

## 11. Citations

*   Refer to `docs/references.md` for guidelines on APA citation style and integration.
*   Use a consistent in-text citation format (e.g., `(Author, Year)`).

By adhering to these guidelines, we ensure a high-quality, consistent, and user-friendly reading experience for the "Physical AI & Humanoid Robotics" textbook.