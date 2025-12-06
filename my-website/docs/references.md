# APA Citation Guidelines and References

This document outlines the APA (7th Edition) citation guidelines for the "Physical AI & Humanoid Robotics" textbook. All content contributors must adhere to these standards for in-text citations and the reference list.

## 1. In-Text Citations

In-text citations are crucial for acknowledging sources and avoiding plagiarism.

*   **Parenthetical Citation**: Include the author's last name and year of publication in parentheses.
    *   Example: `(Smith, 2023)`
    *   For direct quotes, include the page number: `(Smith, 2023, p. 45)`
*   **Narrative Citation**: Integrate the author's name into the sentence, followed by the year in parentheses.
    *   Example: `Smith (2023) argued that...`
    *   For direct quotes, include the page number at the end of the quote: `Smith (2023) stated, "Robotics is evolving rapidly" (p. 45).`
*   **Multiple Authors**:
    *   Two authors: `(Smith & Jones, 2022)` or `Smith and Jones (2022) found...`
    *   Three or more authors: `(Garcia et al., 2021)` or `Garcia et al. (2021) concluded...`
*   **No Author**: Use the title of the work and the year.
    *   Example: `(Title of Work, 2020)`
*   **No Date**: Use "n.d." for no date.
    *   Example: `(Brown, n.d.)`

## 2. Reference List (at the end of `references.md` or each chapter for specific references)

All sources cited in the text must appear in a comprehensive reference list, ordered alphabetically by the author's last name.

### General Format:

*   **Author, A. A. (Year). *Title of work*. Publisher.**

### Examples by Source Type:

*   **Book:**
    *   Author, A. A. (Year of publication). *Title of work*. Publisher.
    *   Example: `Russell, S. J., & Norvig, P. (2010). *Artificial intelligence: A modern approach* (3rd ed.). Prentice Hall.`

*   **Journal Article:**
    *   Author, A. A., Author, B. B., & Author, C. C. (Year). Title of article. *Title of Periodical, volume*(issue), pages. DOI
    *   Example: `Fei-Fei, L., & Perona, P. (2005). A Bayesian approach to unsupervised learning of object categories. *Proceedings of the IEEE International Conference on Computer Vision*, *1*(2), 1-8. https://doi.org/10.1109/ICCV.2005.105`

*   **Chapter in an Edited Book:**
    *   Author, A. A. (Year). Title of chapter. In E. E. Editor & F. F. Editor (Eds.), *Title of book* (pp. pages). Publisher.
    *   Example: `Brooks, R. A. (2002). The relationship between matter and mind. In M. A. Arbib (Ed.), *The handbook of brain theory and neural networks* (pp. 953-956). MIT Press.`

*   **Website/Webpage:**
    *   Author, A. A. (Year, Month Day). *Title of page*. Site name. URL
    *   Example: `Docusaurus. (n.d.). *Markdown features*. Retrieved December 6, 2025, from https://docusaurus.io/docs/markdown-features`

*   **Conference Paper (Published in Proceedings):**
    *   Author, A. A. (Year, Month Day). *Title of paper* [Paper presentation]. Conference Name, Location. DOI (if available)
    *   Example: `Mnih, V., Kavukcuoglu, K., Silver, D., Rusu, A. A., Veness, J., Bellemare, M. G., ... & Hassabis, D. (2013, July). *Playing Atari with deep reinforcement learning* [Paper presentation]. NIPS Deep Learning Workshop, Lake Tahoe, NV.`

*   **Technical Report:**
    *   Author, A. A. (Year). *Title of report* (Report No. XXX). Publisher. URL
    *   Example: `OpenAI. (2023). *GPT-4 Technical Report*. https://openai.com/research/gpt-4`

## 3. Integrating with Docusaurus Markdown

To link to references from your content:

1.  **Create an entry in `my-website/docs/references.md`**: Follow the APA format for the source type.
2.  **Assign a unique Markdown anchor ID**: For each reference in `references.md`, you can create a hidden HTML anchor or use a heading ID.
    *   Example in `references.md`:
        ```markdown
        ### <span id="russell2010">Russell, S. J., & Norvig, P. (2010). *Artificial intelligence: A modern approach* (3rd ed.). Prentice Hall.</span>
        ```
3.  **Link from content**: Use a Markdown link to the anchor ID.
    *   Example in `my-website/docs/chapter1/index.md`:
        ```markdown
        ...as discussed by Russell and Norvig ([2010](#russell2010)).
        ```
    *   Alternatively, you can create a specific Docusaurus component for citations that can automatically link to the `references.md` page. This would require designing and implementing `src/components/Citation.js`.

By consistently applying these guidelines, we will ensure academic rigor and a professional presentation for the textbook.