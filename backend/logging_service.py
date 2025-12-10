import sys
import os

# Add the current directory to the path
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.append(current_dir)

from database import get_db_connection
from typing import List
import json

def log_query(question: str, answer: str, sources: List[str], user_id: str = None):
    """
    Log user queries and agent responses to Neon Postgres.

    Args:
        question: The user's question
        answer: The agent's answer
        sources: List of sources used to generate the answer
        user_id: Optional user identifier
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            # Convert sources list to a format that can be stored in the database
            sources_str = json.dumps(sources)  # Using JSON format for the array

            query = """
                INSERT INTO query_logs (question, answer, sources, user_id)
                VALUES (%s, %s, %s, %s)
                RETURNING id;
            """
            cursor.execute(query, (question, answer, sources_str, user_id))
            log_id = cursor.fetchone()[0]
            conn.commit()

            return log_id
    except Exception as e:
        print(f"Error logging query: {e}")
        conn.rollback()
        raise
    finally:
        conn.close()