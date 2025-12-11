---
id: 3
title: Add Principle VIII - Integrated RAG Chatbot System
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-delivery-platform
branch: 005-delivery-platform
user: ACER
command: /sp.constitution add Principle VIII: Integrated RAG Chatbot System
labels: [architecture,rag,chatbot,backend,ai]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - constitution updated with new principle
  - section numbers updated correctly
  - version incremented to 1.4.0
---

## Prompt

/sp.constitution add to the constitution """Principle VII: Integrated RAG Chatbot System (The "Expert Instructor") This principle mandates the technical architecture, behavior, and storage mechanisms for the Retrieval-Augmented Generation (RAG) chatbot that will be embedded within the textbook application. 7.1 System Goal & Constraint Goal: To provide domain-specific, non-hallucinatory Q&A, strictly grounded in the textbook content. Constraint: All answers MUST be verifiable against the book's 14 chapters. The agent MUST NOT use external web search or general knowledge. 7.2 Technical Architecture Mandates The entire chatbot infrastructure, including all code, configurations, and environment files, SHALL reside in a new root directory named backend/. Component Technology Rationale & Mandate Backend Core FastAPI Must serve as the high-performance, asynchronous REST API layer. Agent Logic OpenAI Agents SDK Used for managing the core conversational flow, thread state, and tool routing. Core LLM Gemini API Key The ultimate reasoning engine used by the agent logic (must be integrated via a custom handler or wrapper within the OpenAI SDK environment). Frontend UI Chatkit Must be used to provide the modern, conversational interface embedded in the Docusaurus frontend. Vector Database Qdrant Cloud Free Tier Must be used for vector storage and semantic search of the textbook embeddings. Relational Storage Neon Serverless Postgres Must be used for managing session/thread state, user data, and conversation history, ensuring scalability and state persistence. 7.3 Data & RAG Requirements Content Ingestion: The full text of all 14 finalized textbook chapters must be processed and embedded into the Qdrant vector database. Chunking Strategy: Content must be chunked intelligently (e.g., based on section headers or paragraph breaks, not fixed character limits) to preserve semantic meaning during retrieval. Retrieval Mechanism: The system must implement a hybrid search strategy (combining sparse and dense retrieval) to maximize recall and precision. 7.4 Context-Aware Functionality Mandate: The agent SHALL be able to process and answer questions based on two distinct contexts: Global Context: Answering questions based on the entire stored knowledge (the full book). User Selection Context (CRITICAL): Answering questions based only on a specific text segment selected by the user on the Docusaurus page, prioritizing that segment over general RAG retrieval. 7.5 Code and File Structure All backend work must be organized under the following mandatory structure: /backend ├── .env                  # Environment variables (API keys, DB credentials) ├── main.py               # FastAPI application entry point ├── services/             # Core logic wrappers (Qdrant, Neon, Gemini) │ ├── rag_service.py      # Qdrant/embedding logic │ ├── state_service.py    # Neon Postgres connection & session logic │ └── llm_wrapper.py      # Custom wrapper for routing OpenAI Agent calls to Gemini └── tasks/                # One-time scripts     └── ingestion.py      # Script to load chapter """

## Response snapshot

Successfully added Principle VIII: Integrated RAG Chatbot System (The "Expert Instructor") to the constitution, which was originally called Principle VII in the prompt but became Principle VIII due to existing structure. The implementation included:

1. Added the complete new principle with all specified sections (8.1-8.5)
2. Updated all existing principle numbers to maintain proper sequence:
   - Previous Principle VIII became IX (Textbook Delivery Platform Requirements)
   - Previous Principle IX became X (Modern UI/UX Design Standards)
3. Updated section numbers throughout the document to maintain consistency
4. Updated the constitution version from 1.3.0 to 1.4.0 to reflect the new principle addition
5. Updated the SYNC IMPACT REPORT at the top of the file to document the changes
6. Added amendment notes at the bottom of the file

The new principle establishes the technical architecture for a Retrieval-Augmented Generation (RAG) chatbot that will be embedded within the textbook application, with specific requirements for:
- Technical architecture using FastAPI, OpenAI Agents SDK, Gemini, Chatkit, Qdrant, and Neon Postgres
- Domain-specific, non-hallucinatory Q&A grounded in textbook content
- Context-aware functionality for both global and user-selected text segments
- Mandatory backend directory structure

## Outcome

- ✅ Impact: Added comprehensive RAG chatbot architecture requirements to constitution
- 🧪 Tests: Constitution file updated successfully with proper numbering
- 📁 Files: .specify/memory/constitution.md updated with new principle
- 🔁 Next prompts: Implement backend directory structure and RAG services
- 🧠 Reflection: New principle enables AI-powered Q&A system while maintaining grounding in textbook content

## Evaluation notes (flywheel)

- Failure modes observed: None - all changes implemented successfully
- Graders run and results: PASS - Constitution properly updated with correct numbering
- Prompt variant (if applicable): Standard constitution amendment
- Next experiment (smallest change to try): Create backend directory structure as specified in new principle