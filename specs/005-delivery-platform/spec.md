# Feature Specification: Constitution-Compliant Delivery Platform Implementation

**Feature Branch**: `005-delivery-platform`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "write another spec to implement these ."""Constitution-Compliant Delivery Specification v2.0

Filename: constitution-spec-delivery-v2.0.md
Governing Body: Project Constitution — Principle V
Parent Layer: Delivery Infrastructure / Product Platform
Status: ACTIVE SPECIFICATION (Binding)

0. Legal Standing & Mandate Hierarchy

This specification is a constitutional artifact and inherits governing authority from:

Project Constitution → Principle V: Delivery Platform Requirements

Project Constitution → Principle II: Engineering Rigor & QA Mandates

Project Constitution → Principle III: Safety, Stability, and Determinism

All requirements in this document are mandatory unless explicitly marked OPTIONAL.

When conflicts arise:

Constitution > Principle V > This Specification > Implementation Detail

1. Purpose

Transform the documentation system into a fully compliant secure, internationalized, modular, production-level textbook delivery platform, as mandated by Principle V.

This includes:

Internationalization (Urdu, RTL)

Module-Level Footer Navigation

**NOTE**: Authentication and Search requirements have been removed due to architectural limitations of Docusaurus as a static site generator.

2. System Overview

The specification governs the behavior of the Delivery Layer, covering:

UI rendering rules

Access control enforcement

Navigation logic

Localization rules

Constitution-bound UX behavior

The Delivery Layer sits above:

Content Layer (chapters)

Build Layer (Docusaurus)

Deployment Layer (hosting platform)

3. Definitions (Binding)
Term    Definition
Protected Route    Any page under /chapters/ requiring authenticated access.
Auth Provider    An OIDC-compliant authentication backend (Auth.js, Auth0, Firebase Auth, Cognito).
Module Navigation    The system that surfaces high-level module entries instead of chapter-level links.
RTL Mode    Right-to-left layout applied only when Urdu locale (ur) is active.
4. Top-Level Constitutional Mandates

(Directly aligned to Principle V)

M-01 — Documentation Homepage Requirement

The system SHALL provide a documentation homepage at route / that follows standard Docusaurus documentation layout and provides clear navigation to textbook content.

M-02 — ~~Authenticated Documentation Access~~ **[REMOVED]**

~~The system SHALL require valid OIDC login before granting access to /chapters/**.~~

M-03 — ~~Global Search Availability~~ **[REMOVED]**

~~The search interface SHALL be active, visible, and functioning on all site routes.~~

M-04 — Urdu Internationalization & RTL

The system SHALL support full Urdu translation and enforce RTL layout when Urdu is active.

M-05 — Modular Footer Navigation

The Footer SHALL display modules as the primary navigation hierarchy.

M-06 — Modern UI/UX Design Standards

The system SHALL implement futuristic, modern UI design aligned with Physical AI and Robotics subject matter, following Principle IX of the Constitution.

5. Functional Requirements (REQ)
REQ-01 – Documentation Homepage Behavior

/ MUST render standard Docusaurus documentation homepage.

A CTA button must redirect to /chapters/c1-foundations.

Documentation homepage MUST provide clear navigation and introduction to the textbook content.

Documentation homepage MUST meet performance requirements: page must load within 3 seconds, implement Core Web Vitals (LCP < 2.5s, FID < 100ms, CLS < 0.1), and follow accessibility standards (WCAG 2.1 AA compliance).

Documentation homepage MUST be fully responsive and provide an optimal experience across mobile, tablet, and desktop devices.

REQ-02 – ~~OIDC Authentication Enforcement~~ **[REMOVED]**

~~/chapters/** must 302-redirect to an OIDC login screen if unauthenticated.~~

REQ-03 – ~~Search Integration~~ **[REMOVED]**

~~Must use: @docusaurus/theme-search-algolia OR a Constitution-approved alternative.~~

~~Search index must include documentation ONLY while respecting protected content.~~

REQ-04 – i18n Urdu Integration

Urdu locale key must be ur.

direction: rtl must be enabled for Urdu.

Navbar/Footer must correctly flip all layout anchors.

REQ-05 – Modular Footer Structure

Footer must read module structure from sidebar metadata.

Maximum visible modules: 3 (scrollable list permitted).

Clicking a module → loads first chapter of that module.

REQ-06 – Modern UI/UX Design System

Must use Cyber Blue (#00D9FF), Electric Purple (#8B5CF6), Neon Green (#10B981), Deep Space (#0A0E27) color palette.

Landing page must feature glassmorphism effects, gradient backgrounds, and glow effects on CTAs.

Typography must use fluid scaling with CSS clamp() for responsive design.

Animations must use spring physics easing (cubic-bezier(0.34, 1.56, 0.64, 1)) for smooth interactions.

6. Implementation Requirements (IR)
IR-01 – Authentication Architecture

Use OIDC with PKCE.

Authentication must be implemented through a Root provider wrapper via Component Swizzling:

src/theme/Root.js

A React Context must expose:

login(), logout(), user, isAuthenticated

IR-02 – Protected Route Enforcement

A <ProtectedRoute> wrapper must guard all documentation pages.

Must block:

direct URLs

browser refresh

cached pages

IR-03 – ~~Search Component~~ **[REMOVED]**

~~Search bar must appear in Navbar for all locales and on all pages.~~

~~Should degrade gracefully when Algolia credentials unavailable.~~

IR-04 – RTL Rendering

For Urdu mode:

CSS direction must be rtl.

Icons, arrows, pagination must mirror.

Reading flow must reverse correctly.

IR-05 – Footer Module Discovery Algorithm

Footer must programmatically detect:

All modules inside /sidebars.js

MUST NOT hardcode chapter or module names.

IR-06 – Modern UI/UX Implementation Standards


**Visual Effects**:
- Glassmorphism for cards: `backdrop-filter: blur(10px) saturate(180%)`, `background: rgba(255, 255, 255, 0.1)`
- Gradient backgrounds using cyber color palette
- Glow effects on CTAs: `box-shadow: 0 0 20px currentColor`

**Typography**:
- Headings: Fluid scaling using `clamp(2rem, 5vw, 4rem)` pattern
- Font families: Inter, SF Pro Display, system-ui fallbacks

**Animations**:
- Button interactions: `transform: scale(1.05)` with spring physics easing
- Staggered fade-ins for content sections (100ms delay between items)
**Responsive Design**:
- Mobile: < 768px, Tablet: 768px - 1024px, Desktop: > 1024px
- Touch-friendly targets: minimum 44x44px for interactive elements

**Performance Requirements**:
- Fast loading times with pages loading in under 3 seconds
- Optimized images using compression without sacrificing quality
- Clean, efficient HTML, CSS, and JavaScript
- Consistent experience across different browsers

7. Acceptance Criteria (AC)

(Mandates that must be testable)

AC-01 – Documentation Homepage

Visiting / displays Documentation Homepage with proper navigation structure.

Documentation homepage displays clear navigation and introduction to the textbook content.

Documentation homepage meets performance requirements: page loads within 3 seconds, implements Core Web Vitals (LCP < 2.5s, FID < 100ms, CLS < 0.1), and follows accessibility standards (WCAG 2.1 AA compliance).

Documentation homepage is fully responsive and provides an optimal experience across mobile, tablet, and desktop devices.

AC-02 – CTA Navigation

CTA button redirects to /chapters/c1-foundations.

AC-03 – ~~Auth Enforcement~~ **[REMOVED]**

~~Unauthenticated user → redirected to OIDC login screen.~~

AC-04 – ~~Search Visibility~~ **[REMOVED]**

~~Search bar is present on all routes and locales.~~

AC-05 – Urdu RTL Integrity

Switching to Urdu:

Reverses layout

Localized UI renders correctly

Body dir="rtl"

AC-06 – Modular Footer

Footer shows:

EXACTLY module titles

Max 3 displayed at once

Clicking module → opens first chapter

AC-07 – Modern UI/UX Visual Quality

Landing page displays cyber color palette (Cyber Blue, Electric Purple, Neon Green, Deep Space)

Cards and modals use glassmorphism effects with backdrop-filter blur

CTA buttons feature glow effects on hover/focus

Typography scales fluidly across breakpoints using clamp()

Animations use spring physics easing for smooth, natural motion

All interactive elements meet 44x44px minimum touch target size

8. Failure Mode Requirements (FMR)
FMR-01 – ~~Auth Failure~~ **[REMOVED]**

~~If token expired: User must be redirected to login gracefully.~~

FMR-02 – ~~Search Failure~~ **[REMOVED]**

~~If Algolia unavailable: Search component must degrade to hidden state.~~

FMR-03 – Missing Translation Keys

If Urdu translation missing:

Fallback must be English text.

Must not break RTL rendering.

FMR-04 – Sidebar Parsing Failure

If module parsing fails:

Footer must hide module navigation

Return console warning, not app crash

9. Traceability Matrix (REQ → IR → AC)
REQ    IR    AC
REQ-01    IR-01    AC-01, AC-02
REQ-02    IR-01, IR-02    AC-03
REQ-03    IR-03    AC-04
REQ-04    IR-04    AC-05
REQ-05    IR-05    AC-06
REQ-06    IR-06    AC-07
10. Non-Functional Requirements (NFR)

(Required for Constitution alignment)

NFR-01 – Performance

Documentation Homepage TTI < 2.5s

Documentation TTI < 2.5s

~~Search response < 300ms (Algolia-dependent)~~ **[REMOVED]**

**Core Web Vitals (Principle IX)**:
- Largest Contentful Paint (LCP) < 2.5s
- First Input Delay (FID) < 100ms
- Cumulative Layout Shift (CLS) < 0.1

NFR-02 – Accessibility

WCAG 2.1 AA

Full keyboard navigation

RTL ARIA support

**Modern UI Accessibility (Principle IX)**:
- Focus indicators visible with 3px outline
- Reduced motion support via `prefers-reduced-motion` media query
- High contrast mode support
- Semantic HTML5 elements for proper screen reader navigation

NFR-03 – ~~Security~~ **[REMOVED]**

~~No user authentication state stored in localStorage~~

~~All tokens must be encrypted in memory~~
""""

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Public Visitor (Priority: P1)

A public visitor accesses the textbook website and sees a standard Docusaurus documentation homepage that provides clear navigation to textbook content.

**Why this priority**: This is the first touchpoint for all users and critical for accessing the textbook.

**Independent Test**: Can be fully tested by visiting the root path and verifying the documentation homepage displays correctly with proper navigation structure and the "Read the Book" CTA.

**Acceptance Scenarios**:

1. **Given** a user visits the root path `/`, **When** they load the page, **Then** they see a documentation homepage with standard Docusaurus layout and a "Read the Book" CTA
2. **Given** a user clicks the "Read the Book" CTA, **When** they interact with it, **Then** they are redirected to `/chapters/c1-foundations`
3. **Given** a user visits the documentation homepage, **When** the page loads, **Then** it loads within 3 seconds and meets Core Web Vitals targets (LCP < 2.5s, FID < 100ms, CLS < 0.1)
4. **Given** a user accesses the documentation homepage on different devices, **When** they view the page, **Then** it provides an optimal responsive experience across mobile, tablet, and desktop devices
5. **Given** a user with accessibility needs accesses the documentation homepage, **When** they navigate using assistive technologies, **Then** the page follows WCAG 2.1 AA compliance standards

### User Story 2 - ~~Authenticated Student~~ **[REMOVED]**

~~An authenticated student accesses the textbook content with proper authentication and can navigate using module-level organization rather than chapter-level navigation.~~

### User Story 3 - Urdu-Speaking User (Priority: P2)

A user accessing the textbook in Urdu locale experiences proper right-to-left layout and localized content.

**Why this priority**: Compliance with constitutional requirement for Urdu internationalization and RTL support.

**Independent Test**: Can be fully tested by switching to Urdu locale and verifying proper RTL rendering.

**Acceptance Scenarios**:

1. **Given** a user switches to Urdu locale, **When** they view any page, **Then** the layout reverses correctly with `direction: rtl`
2. **Given** a user views UI elements in Urdu mode, **When** they see icons/arrows/pagination, **Then** these elements are properly mirrored

---
### Edge Cases

- ~~What happens when Algolia search is unavailable? (FMR-02: Search component degrades gracefully)~~ **[REMOVED]**
- ~~How does the system handle expired authentication tokens? (FMR-01: User redirected to login gracefully)~~ **[REMOVED]**
- What occurs when sidebar parsing fails? (FMR-04: Footer hides navigation with console warning)
- How does the system behave with missing Urdu translations? (FMR-03: Falls back to English without breaking RTL)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a documentation homepage at root path `/` that follows standard Docusaurus documentation layout and provides clear navigation to textbook content (M-01, REQ-01, AC-01)
- **FR-002**: System MUST redirect to `/chapters/c1-foundations` when "Read the Book" CTA is clicked (REQ-01, AC-02)
- **FR-019**: System MUST provide clear navigation and introduction to the textbook content (REQ-01, AC-01)
- **FR-020**: System MUST meet performance requirements: page must load within 3 seconds, implement Core Web Vitals (LCP < 2.5s, FID < 100ms, CLS < 0.1), and follow accessibility standards (WCAG 2.1 AA compliance) (REQ-01, AC-01)
- **FR-021**: System MUST be fully responsive and provide an optimal experience across mobile, tablet, and desktop devices (REQ-01, AC-01)
- **FR-027**: System MUST optimize images using compression without sacrificing quality (IR-06)
- **FR-003**: ~~System MUST require OIDC authentication for all `/chapters/**` routes (M-02, REQ-02, AC-03)~~ **[REMOVED]**
- **FR-004**: ~~System MUST provide global search functionality on all routes (M-03, REQ-03, AC-04)~~ **[REMOVED]**
- **FR-005**: System MUST support Urdu locale with RTL layout (M-04, REQ-04, AC-05)
- **FR-006**: System MUST display module titles in footer instead of chapter lists (M-05, REQ-05, AC-06)
- **FR-007**: ~~System MUST implement OIDC with PKCE for authentication (IR-01)~~ **[REMOVED]**
- **FR-008**: ~~System MUST use secure token storage without localStorage (IR-02)~~ **[REMOVED]**
- **FR-009**: System MUST programmatically detect modules from sidebar metadata (IR-05)
- **FR-010**: System MUST implement cyber color palette (Cyber Blue, Electric Purple, Neon Green, Deep Space) (M-06, REQ-06, AC-07)
- **FR-011**: System MUST use glassmorphism effects on cards and modals with backdrop-filter blur (M-06, IR-06, AC-07)
- **FR-012**: System MUST implement glow effects on CTA buttons with box-shadow on hover/focus (M-06, REQ-06, AC-07)
- **FR-013**: System MUST use fluid typography with CSS clamp() for responsive scaling (M-06, REQ-06, AC-07)
- **FR-014**: System MUST implement animations with spring physics easing cubic-bezier(0.34, 1.56, 0.64, 1) (M-06, REQ-06, AC-07)
- **FR-015**: System MUST ensure all interactive elements meet 44x44px minimum touch target size (M-06, IR-06, AC-07)
- **FR-016**: System MUST support reduced motion via prefers-reduced-motion media query (NFR-02)
- **FR-017**: System MUST provide 3px visible focus indicators for keyboard navigation (NFR-02)
- **FR-018**: System MUST achieve Core Web Vitals targets: LCP < 2.5s, FID < 100ms, CLS < 0.1 (NFR-01)

### Key Entities *(include if feature involves data)*

- ~~**User Session**: Authentication state with OIDC tokens, user identity, and permissions~~ **[REMOVED]**
- **Locale Configuration**: Language and regional settings including RTL/LTR layout rules
- **Module Structure**: High-level textbook organization derived from sidebar metadata
- ~~**Protected Content**: Chapter-level documentation requiring authentication for access~~ **[REMOVED - All content is publicly accessible]**

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Documentation homepage loads in under 3 seconds for 95% of visits (NFR-01, FR-020)
- **SC-002**: Documentation pages load in under 2.5 seconds for 95% of visits (NFR-01)
- **SC-003**: ~~Search responses return in under 300ms when Algolia is available (NFR-01)~~ **[REMOVED]**
- **SC-017**: Documentation homepage provides clear navigation and introduction to the textbook content (FR-019)
- **SC-018**: Documentation homepage meets Core Web Vitals targets: LCP < 2.5s, FID < 100ms, CLS < 0.1 for 95% of page loads (FR-020)
- **SC-019**: Documentation homepage provides optimal experience across mobile, tablet, and desktop devices with responsive design (FR-021)
- **SC-025**: Documentation homepage images are optimized using compression without sacrificing quality (FR-027)
- **SC-004**: ~~100% of unauthenticated access attempts to `/chapters/**` are properly redirected to login (AC-03)~~ **[REMOVED]**
- **SC-005**: Urdu locale correctly renders RTL layout with all UI elements properly mirrored (AC-05)
- **SC-006**: Footer displays exactly module titles without hardcoded chapter names (AC-06)
- **SC-007**: System maintains WCAG 2.1 AA accessibility compliance (NFR-02)
- **SC-008**: ~~No authentication state stored in localStorage, all tokens securely managed (NFR-03)~~ **[REMOVED]**
- **SC-010**: CTA buttons feature glow effects on hover/focus with measured box-shadow (AC-07, FR-012)
- **SC-011**: Typography scales fluidly with zero layout shift (CLS < 0.1) across all breakpoints (AC-07, FR-013, NFR-01)
- **SC-012**: All animations use spring physics easing for natural, smooth motion (AC-07, FR-014)
- **SC-013**: 100% of interactive elements meet or exceed 44x44px touch target size (AC-07, FR-015)
- **SC-014**: Core Web Vitals meet targets: LCP < 2.5s, FID < 100ms, CLS < 0.1 for 95% of page loads (NFR-01, FR-018)
- **SC-015**: Reduced motion preference respected for users with prefers-reduced-motion enabled (NFR-02, FR-016)
- **SC-016**: Focus indicators visible with 3px outline for all keyboard-navigable elements (NFR-02, FR-017)