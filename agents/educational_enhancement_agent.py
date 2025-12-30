"""
Educational Enhancement Agent
Creates educational materials and personalizes learning experiences
"""

import asyncio
import logging
from typing import List, Dict, Optional
from dataclasses import dataclass
from datetime import datetime
import re
import random

# Import existing backend services
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))
from backend.src.services.cohere_service import CohereService
from backend.src.services.qdrant_service import QdrantService


@dataclass
class EducationalContent:
    title: str
    content: str
    difficulty: str
    content_type: str  # question, exercise, summary, objective
    metadata: Dict


@dataclass
class LearningPath:
    user_id: str
    current_module: str
    progress: float
    strengths: List[str]
    weaknesses: List[str]
    recommendations: List[str]


class EducationalEnhancementAgent:
    def __init__(self):
        self.cohere_service = CohereService()
        self.qdrant_service = QdrantService()
        self.logger = logging.getLogger(__name__)

    async def generate_questions(self, content_path: str, difficulty: str = "intermediate", count: int = 5) -> List[EducationalContent]:
        """Generate practice questions based on chapter content"""
        self.logger.info(f"Generating {count} questions for {content_path} at {difficulty} level")

        # Get content from documentation
        content = await self.get_content_from_path(content_path)

        # Generate questions based on content
        questions = []
        for i in range(count):
            question_prompt = f"""
            Based on the following content from the Physical AI & Humanoid Robotics book, generate a practice question appropriate for {difficulty} level students:

            Content: {content}

            Generate a question that tests understanding of key concepts, with appropriate complexity for {difficulty} level learners.
            """

            question_text = await self.cohere_service.generate_text(question_prompt)
            question = EducationalContent(
                title=f"Question {i+1}",
                content=question_text,
                difficulty=difficulty,
                content_type="question",
                metadata={
                    "source": content_path,
                    "generated_at": datetime.now().isoformat()
                }
            )
            questions.append(question)

        return questions

    async def generate_exercises(self, module: str, difficulty: str = "intermediate") -> List[EducationalContent]:
        """Generate hands-on exercises for a specific module"""
        self.logger.info(f"Generating exercises for module {module}")

        # Get content related to the module
        module_content = await self.qdrant_service.search_documents(module, limit=5)

        exercise_prompt = f"""
        Based on the Physical AI & Humanoid Robotics content related to {module}, generate a hands-on exercise appropriate for {difficulty} level students.

        The exercise should be practical and engaging, allowing students to apply concepts from the module.
        Include clear instructions and expected outcomes.
        """

        exercise_text = await self.cohere_service.generate_text(exercise_prompt)

        exercise = EducationalContent(
            title=f"Hands-on Exercise: {module}",
            content=exercise_text,
            difficulty=difficulty,
            content_type="exercise",
            metadata={
                "module": module,
                "generated_at": datetime.now().isoformat()
            }
        )

        return [exercise]

    async def generate_summary(self, content_path: str) -> EducationalContent:
        """Generate a summary of key points from content"""
        self.logger.info(f"Generating summary for {content_path}")

        content = await self.get_content_from_path(content_path)

        summary_prompt = f"""
        Create a concise summary of the key points from the following Physical AI & Humanoid Robotics content:

        {content}

        The summary should highlight the most important concepts, principles, and takeaways for students.
        """

        summary_text = await self.cohere_service.generate_text(summary_prompt)

        summary = EducationalContent(
            title=f"Summary: {content_path}",
            content=summary_text,
            difficulty="all",
            content_type="summary",
            metadata={
                "source": content_path,
                "generated_at": datetime.now().isoformat()
            }
        )

        return summary

    async def generate_learning_objectives(self, content_path: str) -> EducationalContent:
        """Generate learning objectives for content"""
        self.logger.info(f"Generating learning objectives for {content_path}")

        content = await self.get_content_from_path(content_path)

        objectives_prompt = f"""
        Based on the following Physical AI & Humanoid Robotics content, generate clear learning objectives:

        {content}

        Learning objectives should be specific, measurable, achievable, relevant, and time-bound (SMART).
        Focus on what students should be able to understand or do after studying this content.
        """

        objectives_text = await self.cohere_service.generate_text(objectives_prompt)

        objectives = EducationalContent(
            title=f"Learning Objectives: {content_path}",
            content=objectives_text,
            difficulty="all",
            content_type="objective",
            metadata={
                "source": content_path,
                "generated_at": datetime.now().isoformat()
            }
        )

        return objectives

    async def generate_assessment(self, module: str, difficulty: str = "intermediate", question_count: int = 10) -> EducationalContent:
        """Generate an assessment for a module"""
        self.logger.info(f"Generating assessment for {module}")

        # Get content for the module
        module_content = await self.qdrant_service.search_documents(module, limit=3)

        assessment_prompt = f"""
        Create a comprehensive assessment for the {module} module of the Physical AI & Humanoid Robotics book.
        The assessment should be appropriate for {difficulty} level students and include {question_count} questions.

        Include a mix of question types (multiple choice, short answer, practical application).
        Focus on key concepts from the module.
        """

        assessment_text = await self.cohere_service.generate_text(assessment_prompt)

        assessment = EducationalContent(
            title=f"Assessment: {module}",
            content=assessment_text,
            difficulty=difficulty,
            content_type="assessment",
            metadata={
                "module": module,
                "question_count": question_count,
                "generated_at": datetime.now().isoformat()
            }
        )

        return assessment

    async def adapt_content_difficulty(self, content: str, target_difficulty: str) -> str:
        """Adapt content difficulty based on target level"""
        adaptation_prompt = f"""
        Adapt the following Physical AI & Humanoid Robotics content to match {target_difficulty} level:

        Original content: {content}

        Adjust the complexity, examples, and explanations to be appropriate for {target_difficulty} level learners.
        """

        adapted_content = await self.cohere_service.generate_text(adaptation_prompt)
        return adapted_content

    async def create_learning_path(self, user_id: str, initial_module: str) -> LearningPath:
        """Create a personalized learning path for a user"""
        self.logger.info(f"Creating learning path for user {user_id}")

        # In a real implementation, this would analyze user data and performance
        # For now, create a basic learning path
        path = LearningPath(
            user_id=user_id,
            current_module=initial_module,
            progress=0.0,
            strengths=[],
            weaknesses=[],
            recommendations=[f"Start with the {initial_module} module", "Complete all exercises", "Take module assessments"]
        )

        return path

    async def suggest_additional_resources(self, topic: str) -> List[str]:
        """Suggest additional learning resources for a topic"""
        # Search for related content in documentation
        related_content = await self.qdrant_service.search_documents(topic, limit=3)

        resources = []
        for item in related_content:
            resources.append(item.payload.get("source_file", "Unknown"))

        # Add some general suggestions
        resources.extend([
            f"Review chapter on {topic} in the book",
            "Practice with the provided exercises",
            "Join the discussion forum for this topic"
        ])

        return resources

    async def get_content_from_path(self, content_path: str) -> str:
        """Retrieve content from documentation path"""
        # This would normally read from the actual documentation files
        # For now, search in the vector database
        search_results = await self.qdrant_service.search_documents(content_path, limit=1)

        if search_results:
            return search_results[0].payload.get("content", "")
        else:
            # If not found in vector DB, return a placeholder
            return f"Content for {content_path} would be retrieved from documentation here."

    async def generate_by_topic(self, topic: str, content_type: str, difficulty: str = "intermediate") -> List[EducationalContent]:
        """Generate educational content based on topic and type"""
        if content_type == "questions":
            # Find content related to the topic
            topic_content = await self.qdrant_service.search_documents(topic, limit=1)
            if topic_content:
                content_path = topic_content[0].payload.get("source_file", topic)
                return await self.generate_questions(content_path, difficulty, 3)
        elif content_type == "exercises":
            return await self.generate_exercises(topic, difficulty)
        elif content_type == "summary":
            topic_content = await self.qdrant_service.search_documents(topic, limit=1)
            if topic_content:
                content_path = topic_content[0].payload.get("source_file", topic)
                return [await self.generate_summary(content_path)]
        elif content_type == "objectives":
            topic_content = await self.qdrant_service.search_documents(topic, limit=1)
            if topic_content:
                content_path = topic_content[0].payload.get("source_file", topic)
                return [await self.generate_learning_objectives(content_path)]
        elif content_type == "assessments":
            return [await self.generate_assessment(topic, difficulty)]

        return []

    async def track_learning_progress(self, user_id: str, module: str, score: float) -> Dict[str, any]:
        """Track and analyze user learning progress"""
        # This would normally update user data in a database
        # For now, return analysis based on score
        analysis = {
            "user_id": user_id,
            "module": module,
            "score": score,
            "performance_level": self.get_performance_level(score),
            "recommendations": self.get_recommendations_for_score(score, module),
            "next_steps": await self.get_next_steps(module, score)
        }

        return analysis

    def get_performance_level(self, score: float) -> str:
        """Determine performance level based on score"""
        if score >= 0.9:
            return "Excellent"
        elif score >= 0.8:
            return "Good"
        elif score >= 0.7:
            return "Satisfactory"
        elif score >= 0.6:
            return "Needs Improvement"
        else:
            return "Requires Attention"

    def get_recommendations_for_score(self, score: float, module: str) -> List[str]:
        """Get recommendations based on performance score"""
        recommendations = []

        if score < 0.7:
            recommendations.append(f"Review the {module} content more thoroughly")
            recommendations.append("Complete additional practice exercises")
            recommendations.append("Seek additional resources on challenging topics")

        if score >= 0.9:
            recommendations.append(f"Consider advanced topics in {module}")
            recommendations.append("Help other students with this content")

        return recommendations

    async def get_next_steps(self, current_module: str, score: float) -> List[str]:
        """Suggest next steps based on current performance"""
        if score >= 0.7:
            # Ready to move forward
            next_modules = await self.get_next_modules(current_module)
            return [f"Continue to next module: {next_modules[0] if next_modules else 'Advanced topics'}"]
        else:
            # Need more practice
            return [
                f"Review {current_module} content",
                "Complete additional exercises",
                "Retake the assessment"
            ]

    async def get_next_modules(self, current_module: str) -> List[str]:
        """Get the sequence of modules after the current one"""
        # Define module sequence
        module_sequence = [
            "intro",
            "module-1-ros2",
            "module-2-simulation",
            "module-3-isaac",
            "module-4-vla",
            "hardware-setup",
            "appendices"
        ]

        try:
            current_index = module_sequence.index(current_module.split('/')[-1] if '/' in current_module else current_module)
            if current_index + 1 < len(module_sequence):
                return [module_sequence[current_index + 1]]
        except ValueError:
            # Module not in sequence
            pass

        return []


# Example usage
async def main():
    agent = EducationalEnhancementAgent()

    # Generate some educational content
    questions = await agent.generate_questions("docs/module-1-ros2/ros2-intro.md", "beginner", 2)
    for q in questions:
        print(f"Question: {q.content}\n")

    summary = await agent.generate_summary("docs/module-2-simulation/gazebo-intro.md")
    print(f"Summary: {summary.content}\n")


if __name__ == "__main__":
    asyncio.run(main())