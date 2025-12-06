---
id: 0004-constitution-delivery-platform-amendment
title: "Constitution Delivery Platform Amendment"
stage: constitution
date_iso: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: "constitution"
branch: "002-ros2-architecture"
user: "ACER"
command: "/sp.constitution"
labels: ["constitution", "delivery-platform", "docusaurus", "authentication", "principle-viii"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Constitution Delivery Platform Amendment

## User Input (PROMPT_TEXT)
```
just adjust these instructions too in the constitution. """Constitution Addendum — Delivery & Interface Mandates (Principle V)

This document amends and extends the existing Project Constitution. All requirements defined herein carry equal authority to previously ratified Articles and Principles. In cases of conflict, the stricter requirement prevails unless explicitly overridden.

Principle V — Textbook Delivery Platform Requirements (Docusaurus)

This Principle governs the delivery, presentation, access control, and internationalization of the digital textbook platform. All mandates apply to the production system, staging environments, and any derivative deployments.

5.1 Landing Environment, Entry Flow, and Navigation Priority
5.1.1 Mandate (Root-Level Experience)

The platform must implement a dedicated, non-documentation Landing Page at the root path (/) using src/pages/index.js.

Acceptance Criteria

AC-5.1-A: The Landing Page must introduce the textbook and present a single primary Call-to-Action labeled "Read the Book."

AC-5.1-B: This CTA must redirect to the canonical documentation entry point:

/chapters/c1-foundations


AC-5.1-C: No documentation scaffolding (sidebar, breadcrumbs, doc layout) may appear on the Landing Page.

AC-5.1-D: Navigation must prioritize access to the main textbook content and map cleanly into the site's global hierarchy.

5.2 Authentication & Access Control (Modern Standards)
5.2.1 Mandate (Documentation Protection)

All documentation routes under:

/chapters/**


must be logically and programmatically protected. Chapter content must never render to unauthenticated users.

5.2.2 Authentication Architecture

Authentication must be implemented using a modern, standards-compliant, security-audited approach.

Primary Required Authentication Solution:

Auth.js (NextAuth Core for SPAs using @auth/client)

OIDC-compliant

PKCE support

Token refresh rotation

Minimal vendor lock-in

Maintains future extensibility (e.g., OAuth2 providers, email magic links)

Permitted fallback options (only with documented justification):

Auth0 SPA SDK

Firebase Authentication (modular SDK)

Acceptance Criteria

AC-5.2-A: Authentication must use secure OIDC flows with PKCE.

AC-5.2-B: The documentation layer must be wrapped in a React guard component, such as:

<AuthGuard>{docs}</AuthGuard>


AC-5.2-C: Unauthenticated users must be redirected to an authentication interface.

AC-5.2-D: Authentication state must use secure, refreshable tokens (JWT or opaque).

AC-5.2-E: Deep links (e.g., /chapters/c3/section2) must remain protected even when accessed directly by URL.

5.3 Search Provisioning & Internationalization Requirements
5.3.1 Mandate (Search Capability)

The platform must provide full-text search accessible from the global Navbar.

Acceptance Criteria — Search

AC-5.3-S1: Search must use an official or officially supported Docusaurus-integrated provider such as:

Algolia DocSearch (preferred)

A vetted full-text alternative

AC-5.3-S2: All chapter pages under /chapters/** must be indexed.

AC-5.3-S3: Search must function correctly under authenticated routing.

5.3.2 Mandate (Internationalization + RTL Compliance)

The system must support multilingual rendering and layout.

Acceptance Criteria — i18n

AC-5.3-I1: Supported target locale: Urdu (ur).

AC-5.3-I2: Urdu rendering must use Right-to-Left (RTL) layout, with full mirroring of UI components where appropriate.

AC-5.3-I3: The user must be able to switch language from the global navigation.

AC-5.3-I4: All layouts must maintain readability, alignment, and structural integrity in both LTR and RTL modes.

5.4 Footer Architecture & Modular Navigation Contracts
5.4.1 Mandate (High-Level Footer Structure)

The Footer must reflect the macro-structure of the textbook rather than enumerating chapters.

Acceptance Criteria

AC-5.4-A: Footer links must display Module Titles only (e.g., "Module 1: The Robotic Nervous System").

AC-5.4-B: Selecting a module must open the first chapter of that module.

AC-5.4-C: No chapter-level lists may appear in the footer.

AC-5.4-D: Footer behavior and layout"""
```

## Response Summary (RESPONSE_TEXT)
Successfully updated the project constitution to include Principle VIII: Textbook Delivery Platform Requirements (Docusaurus). This new principle covers landing page requirements, authentication and access control, search capabilities, internationalization requirements, and footer navigation architecture. The constitution version was updated from 1.1.0 to 1.2.0 as a MINOR amendment to reflect the addition of the new principle.

## Implementation Details

- Added Principle VIII: Textbook Delivery Platform Requirements (Docusaurus) to the constitution
- Included sections for Landing Environment, Authentication & Access Control, Search & Internationalization, and Footer Architecture
- Updated version from 1.1.0 to 1.2.0 (MINOR amendment for new principle)
- Added comprehensive SYNC IMPACT REPORT at the top of the constitution file
- Included all acceptance criteria for each section of the new principle
- Updated the principle numbering to VIII to maintain sequence after existing VII principles

## Outcome
The project constitution now includes comprehensive requirements for the textbook delivery platform, including authentication, landing page, search, and internationalization requirements. The constitution is properly versioned and maintains all existing principles while adding the new delivery platform requirements.

## Evaluation
- Constitution updated successfully with new Principle VIII
- Version properly updated to 1.2.0 with appropriate amendment type
- All acceptance criteria from user input included
- Existing principles preserved and properly numbered
- SYNC IMPACT REPORT properly formatted and comprehensive