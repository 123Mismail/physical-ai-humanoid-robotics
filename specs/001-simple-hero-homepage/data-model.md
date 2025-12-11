# Data Model: Simple Hero Homepage

## Overview
The simple hero homepage is primarily a presentation-focused component with minimal data requirements. The page displays static content from the site configuration and requires no persistent storage or complex data structures.

## Entities

### 1. Homepage Content
- **Purpose**: Represents the static content displayed on the hero page
- **Fields**:
  - `title`: String - The main book title displayed as H1
  - `subtitle`: String (optional) - Subtitle or tagline supporting the main title
  - `startReadingUrl`: String - URL for the "Start Reading" button (typically `/chapters/c1-foundations`)
  - `githubUrl`: String - URL for the GitHub button (repository link)
  - `ctaText`: String - Call-to-action text for primary button
  - `secondaryCtaText`: String - Call-to-action text for secondary button

### 2. Site Configuration (Inherited)
- **Purpose**: Global site metadata inherited from Docusaurus configuration
- **Fields**:
  - `siteTitle`: String - Global site title from docusaurus.config.js
  - `siteTagline`: String - Global site tagline from docusaurus.config.js
  - `favicon`: String - Site favicon path
  - `metadata`: Object - Additional SEO and meta information

## Validation Rules

### Homepage Content Validation
- `title` must not be empty (required field)
- `startReadingUrl` must be a valid relative or absolute URL
- `githubUrl` must be a valid absolute URL (external link)
- `ctaText` must not exceed 50 characters for proper button display
- All text fields must support internationalization (i18n) for Urdu locale

## Relationships
- The Homepage Content entity uses Site Configuration values as defaults
- Site Configuration provides fallback values if Homepage Content fields are not explicitly defined

## State Transitions
- The page has no state transitions - it's a static presentation component
- The only dynamic aspect is the responsive layout that adapts to screen size

## Constraints
- All content must be loaded within 3 seconds (performance requirement)
- Content must be accessible to screen readers and keyboard navigation users
- Text content must maintain readability with a minimum 4.5:1 contrast ratio against backgrounds
- Content must be responsive and adapt to mobile, tablet, and desktop screen sizes