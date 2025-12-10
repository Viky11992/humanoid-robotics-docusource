import os
from psycopg import connect
from contextlib import contextmanager
from dotenv import load_dotenv

load_dotenv()

def get_db_connection():
    """
    Creates and returns a connection to the Neon Postgres database.
    """
    database_url = os.getenv("NEON_DATABASE_URL")

    if not database_url:
        raise ValueError("NEON_DATABASE_URL environment variable is required")

    return connect(database_url)

@contextmanager
def get_db_cursor():
    """
    Context manager for database cursor operations.
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            yield cursor
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()