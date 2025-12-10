-- Database schema for storing book metadata, ingestion status, and query logs in Neon Postgres

-- Table for book metadata
CREATE TABLE IF NOT EXISTS books (
    id SERIAL PRIMARY KEY,
    title VARCHAR(255) NOT NULL,
    author VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Table for text chunks and their metadata
CREATE TABLE IF NOT EXISTS text_chunks (
    id SERIAL PRIMARY KEY,
    book_id INTEGER REFERENCES books(id),
    chunk_text TEXT NOT NULL,
    chunk_id VARCHAR(255) UNIQUE NOT NULL, -- ID that corresponds to Qdrant point ID
    page_number INTEGER,
    section_title VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Table for query logs
CREATE TABLE IF NOT EXISTS query_logs (
    id SERIAL PRIMARY KEY,
    question TEXT NOT NULL,
    answer TEXT NOT NULL,
    sources TEXT[], -- Array of source chunk IDs
    user_id VARCHAR(255), -- Optional user identifier
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Table for ingestion status tracking
CREATE TABLE IF NOT EXISTS ingestion_status (
    id SERIAL PRIMARY KEY,
    book_id INTEGER REFERENCES books(id),
    status VARCHAR(50) NOT NULL DEFAULT 'processing', -- processing, completed, failed
    total_chunks INTEGER DEFAULT 0,
    processed_chunks INTEGER DEFAULT 0,
    error_message TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    completed_at TIMESTAMP
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_text_chunks_book_id ON text_chunks(book_id);
CREATE INDEX IF NOT EXISTS idx_text_chunks_chunk_id ON text_chunks(chunk_id);
CREATE INDEX IF NOT EXISTS idx_query_logs_created_at ON query_logs(created_at);
CREATE INDEX IF NOT EXISTS idx_ingestion_status_book_id ON ingestion_status(book_id);