# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

This guide provides instructions for setting up your local development environment, contributing content, and deploying the "Physical AI & Humanoid Robotics" Docusaurus textbook.

## 1. Prerequisites

Before you begin, ensure you have the following software installed:

*   **Node.js (LTS version)**: Docusaurus requires Node.js. Download from [nodejs.org](https://nodejs.org/).
*   **npm or Yarn**: Package managers for Node.js. npm is installed with Node.js; Yarn can be installed via `npm install -g yarn`.
*   **Git**: Version control system. Download from [git-scm.com](https://git-scm.com/).
*   **Python 3.9+**: For running code examples and simulations. Download from [python.org](https://www.python.org/downloads/).

## 2. Local Development Setup

Follow these steps to get the Docusaurus project running on your local machine:

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/Viky11992/Heckathone-001.git
    cd Heckathone-001/my-website
    ```

2.  **Install Dependencies**:
    ```bash
    npm install
    # or
    yarn install
    ```

3.  **Start the Development Server**:
    ```bash
    npm run start
    # or
    yarn start
    ```
    This will open the textbook in your browser at `http://localhost:3000`. The development server features live reloading, so changes to your Markdown or React files will be reflected automatically.

## 3. Content Contribution Guide

This section outlines how to contribute content to the textbook.

### Creating New Chapters/Documents

1.  **Create a New Markdown File**:
    *   Navigate to the `my-website/docs/` directory.
    *   Create a new subdirectory for your chapter (e.g., `my-website/docs/chapter_new/`).
    *   Inside the subdirectory, create an `index.md` (or `index.mdx`) file.
    *   Use `kebab-case` for file and directory names.

2.  **Add Frontmatter**:
    *   Every new document **must** include frontmatter at the top. Refer to `my-website/docs/content-guidelines.md` for required fields and examples.

### Adding Code Examples

*   **Inline Code**: Use backticks (`` `code` ``) for short code snippets within paragraphs.
*   **Code Blocks**: Use fenced code blocks with language identifiers.
    ````markdown
    ```python
    print("Hello, Robotics!")
    ```
    ````
*   **Interactive JavaScript/React Code**: Use the `live` attribute for interactive code blocks.
    ````markdown
    ```jsx live
    import React from 'react';
    function MyComponent() {
      // ...
    }
    <MyComponent />
    ```
    ````
*   **External Python Examples**: For larger Python examples or simulations, store them in the `my-website/examples/` directory or within a chapter's dedicated `code-examples` subdirectory (e.g., `my-website/docs/chapter1/code-examples/`). Link to them from the Markdown content.

### Embedding Images and Multimedia

1.  **Store Assets**: Place images and other static assets in `my-website/static/img/`.
2.  **Reference in Markdown**: Use Markdown image syntax with relative paths from the Docusaurus site root.
    ````markdown
    ![Alt text for image](img/my-image.png)
    ````

### Adding Citations

1.  **APA Guidelines**: Refer to `my-website/docs/references.md` for detailed APA (7th Edition) citation guidelines.
2.  **In-Text Citations**: Use parenthetical or narrative citations (e.g., `(Author, Year)`).
3.  **Reference List**: All cited sources must be listed in `my-website/docs/references.md` following APA format. You can create anchor IDs for direct linking from content.

## 4. Deployment Instructions (GitHub Pages)

The textbook is configured to be deployed to GitHub Pages.

1.  **Build the Static Site**:
    ```bash
    npm run build
    # or
    yarn build
    ```
    This command generates static HTML, CSS, and JavaScript files in the `my-website/build/` directory.

2.  **Deploy to GitHub Pages**:
    The deployment process is typically automated via GitHub Actions when changes are pushed to the `main` branch. Ensure your repository is configured for GitHub Pages to serve from the `gh-pages` branch.
    The `docusaurus.config.ts` is configured with:
    *   `url: 'https://Viky11992.github.io'`
    *   `baseUrl: '/Heckathone-001/'`
    *   `organizationName: 'Viky11992'`
    *   `projectName: 'Heckathone-001'`

    For manual deployment (not recommended if GitHub Actions is set up):
    ```bash
    GIT_USER=Viky11992 USE_SSH=true yarn deploy
    # or
    GIT_USER=Viky11992 USE_SSH=true npm run deploy
    ```
    This command builds the site and pushes the `build` output to the `gh-pages` branch of your GitHub repository.

By following these guidelines, you can effectively contribute to and manage the "Physical AI & Humanoid Robotics" textbook.
