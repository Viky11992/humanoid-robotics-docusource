from typing import List, Dict, Any
import sys
import os

# Add the current directory to the path
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.append(current_dir)

from database import get_db_connection
import json

class MetadataLookupTool:
    """
    Tool for retrieving metadata from Neon based on Qdrant search results.
    """

    def lookup_metadata(self, qdrant_result_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve metadata from Neon based on Qdrant search result IDs.

        Args:
            qdrant_result_ids: List of IDs from Qdrant search results

        Returns:
            List of metadata dictionaries corresponding to the IDs
        """
        if not qdrant_result_ids:
            return []

        conn = get_db_connection()
        try:
            with conn.cursor() as cursor:
                # Create placeholders for the query
                placeholders = ','.join(['%s'] * len(qdrant_result_ids))

                # Query to find metadata based on chunk_id (which corresponds to Qdrant point ID)
                query = f"""
                    SELECT chunk_id, book_id, page_number, section_title, chunk_text
                    FROM text_chunks
                    WHERE chunk_id IN ({placeholders})
                """

                cursor.execute(query, qdrant_result_ids)
                rows = cursor.fetchall()

                # Convert rows to list of dictionaries
                metadata_list = []
                for row in rows:
                    chunk_id, book_id, page_number, section_title, chunk_text = row
                    metadata_list.append({
                        "chunk_id": chunk_id,
                        "book_id": book_id,
                        "page_number": page_number,
                        "section_title": section_title,
                        "chunk_text": chunk_text
                    })

                return metadata_list
        except Exception as e:
            print(f"Error looking up metadata: {e}")
            raise
        finally:
            conn.close()


# Global instance
metadata_lookup_tool = MetadataLookupTool()