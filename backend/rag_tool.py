from typing import List, Dict, Any, Optional
import os
import google.generativeai as genai
from dotenv import load_dotenv
import sys

# Add the current directory to the path
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Import the vector search and metadata lookup tools
from vector_search_tool import vector_search_tool
from metadata_lookup_tool import metadata_lookup_tool

load_dotenv()

class RAGAnswerSynthesisTool:
    """
    Tool for synthesizing answers using RAG (Retrieval-Augmented Generation)
    with provided context and Google Gemini model.
    """

    def __init__(self):
        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-2.5-flash')

        self.system_prompt = (
            "Answer questions strictly based on the provided book content. "
            "If the answer is not found in the context, state that clearly. Do not use external knowledge."
        )

    def synthesize_answer(
        self,
        question: str,
        selected_text: Optional[str] = None,
        vector_search_results: Optional[List[Dict[str, Any]]] = None
    ) -> Dict[str, Any]:
        """
        Synthesize an answer based on the question and provided context.

        Args:
            question: The user's question
            selected_text: Text selected by the user as context (optional)
            vector_search_results: Results from vector search in Qdrant (optional)

        Returns:
            Dictionary containing the answer and sources
        """
        # If no selected_text is provided, perform vector search
        if not selected_text and not vector_search_results:
            vector_search_results = vector_search_tool.search(question)

        # Combine all available context
        context_parts = []
        sources = []

        if selected_text:
            context_parts.append(f"User-provided context: {selected_text}")
            sources.append("user_selected_text")

        if vector_search_results:
            for result in vector_search_results:
                if 'text' in result:
                    context_parts.append(result['text'])
                    if 'id' in result:
                        sources.append(result['id'])

        # Create the full context
        full_context = "\n\n".join(context_parts)

        # Prepare the prompt for the Gemini model
        if full_context:
            prompt = f"{self.system_prompt}\n\nContext: {full_context}\n\nQuestion: {question}"
        else:
            prompt = f"{self.system_prompt}\n\nQuestion: {question}"

        # Generate content using Gemini
        response = self.model.generate_content(
            prompt,
            generation_config=genai.types.GenerationConfig(
                temperature=0.3,
                max_output_tokens=1000
            )
        )

        answer = response.text

        return {
            "answer": answer,
            "sources": sources
        }

# Create a global instance
rag_tool = RAGAnswerSynthesisTool()