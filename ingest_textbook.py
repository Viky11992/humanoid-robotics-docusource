import requests
import json

# Configuration
API_BASE_URL = 'http://localhost:8000'
API_KEY = 'OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU'  # Your API key

def ingest_textbook_content(text_content):
    """
    Ingest textbook content into the RAG system
    """
    url = f"{API_BASE_URL}/rag/ingest"

    headers = {
        'Content-Type': 'application/json',
        'x-api-key': API_KEY
    }

    payload = {
        'text': text_content
    }

    try:
        response = requests.post(url, headers=headers, json=payload)

        if response.status_code == 200:
            print("✅ Textbook content successfully ingested into the RAG system!")
            print(f"Response: {response.json()}")
            return True
        else:
            print(f"❌ Error: {response.status_code}")
            print(f"Response: {response.text}")
            return False

    except Exception as e:
        print(f"❌ Exception occurred: {str(e)}")
        return False

def load_textbook_from_file(file_path):
    """
    Load textbook content from a text file
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
            return content
    except Exception as e:
        print(f"❌ Error reading file: {str(e)}")
        return None

def main():
    print("📚 RAG Chatbot - Textbook Content Ingestion Tool")
    print("=" * 50)

    print("\nChoose an option:")
    print("1. Load content from a text file")
    print("2. Paste content directly")

    choice = input("\nEnter your choice (1 or 2): ").strip()

    if choice == "1":
        file_path = input("Enter the path to your textbook file: ").strip()
        content = load_textbook_from_file(file_path)

        if content is None:
            print("❌ Could not load content from file. Exiting.")
            return

    elif choice == "2":
        print("\nEnter your textbook content (press Enter twice when done):")
        lines = []
        while True:
            line = input()
            if line == "":
                break
            lines.append(line)
        content = "\n".join(lines)

        if not content.strip():
            print("❌ No content entered. Exiting.")
            return
    else:
        print("❌ Invalid choice. Exiting.")
        return

    print(f"\n📝 Content length: {len(content)} characters")
    confirm = input("Proceed with ingestion? (y/n): ").strip().lower()

    if confirm == 'y':
        success = ingest_textbook_content(content)
        if success:
            print("\n🎉 Your textbook content is now available for the RAG chatbot!")
            print("You can now ask questions about your textbook content using the chat interface.")
    else:
        print("❌ Ingestion cancelled.")

if __name__ == "__main__":
    main()