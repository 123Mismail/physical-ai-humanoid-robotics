<!--
SYNC IMPACT REPORT
==================
Version Change: 1.6.0 → 1.7.0
Amendment Type: MINOR (landing page requirements removed)
Amendment Date: 2025-12-10

Modified Principles:
  - Updated Principle VIII: Textbook Delivery Platform Requirements - Removed landing page requirements (Section 8.1)
  - Updated Principle IX: Modern UI/UX Design Standards (Futuristic Interface Requirements) - Removed landing page specific requirements

Removed Sections:
  - Principle VIII Section 8.1: Landing Environment, Entry Flow, and Navigation Priority - Landing page requirements removed
  - Principle IX Section 9.5.1: Landing Page Hero requirements removed
  - Principle IX Section 9.5.2: Landing Page Conversion Optimization requirements removed
  - Enhanced implementation checklist in Section 9.6 updated to remove landing page specific items

Templates Requiring Updates:
  ✅ plan-template.md - updated to reflect removal of landing page requirements
  ✅ spec-template.md - updated to remove landing page requirements
  ✅ tasks-template.md - updated to remove landing page optimization tasks
  ⚠️ phr-template.prompt.md - may need updates for landing page removal

Follow-up TODOs:
  1. Review and update affected templates to incorporate removal of landing page requirements
  2. Update any dependent documentation referencing landing page design standards

Rationale for Amendment:
  - Remove landing page requirements to simplify project scope
  - Align with standard Docusaurus documentation experience
  - Focus on core textbook content delivery rather than marketing landing page

Previous Amendment Notes:
  - (2025-12-09) Updated Principle VIII Section 8.1: Enhanced landing page requirements with conversion optimization best practices
  - (2025-12-09) Updated Principle IX Section 9.5.1: Enhanced landing page hero section with improved content structure and visual hierarchy requirements
  - (2025-12-09) Added Principle IX Section 9.5.2: New section for landing page conversion optimization requirements
  - (2025-12-09) Updated Principle IX Section 9.6: Expanded implementation checklist with additional landing page verification items
  - (2025-12-08) Removed RAG chatbot frontend and backend functionality from project scope
  - (2025-12-07) Added Principle X: Modern UI/UX Design Standards (Futuristic Interface Requirements) (now VIII)
  - (2025-12-07) Authentication requirements removed from Principle VI Section 8.2 (was 9.2)
  - Docusaurus static site generation incompatible with server-side auth
  - All chapter content now publicly accessible
  - (2025-12-07) Search requirements removed from Principle VI Section 8.3 (was 9.3)
  - Algolia credentials not available
  - Search functionality removed from navbar and configuration
  - (2025-12-06) Added Principle IX: Textbook Delivery Platform Requirements (now VIII)
==================
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Authoritative Documentation Sources (NON-NEGOTIABLE)

**MUST** use `context7` MCP tool to retrieve the latest official documentation for all technical frameworks, APIs, and libraries before generating any technical or code-specific content.

**Rationale**: The textbook targets university-level engineering education. Accuracy is paramount. Using outdated or hallucinated API references undermines educational value and student success. The `context7` tool provides access to current official documentation for ROS 2, Gazebo, Isaac Sim, Unity, and other critical frameworks.

**Implementation**: Before writing any code example, configuration snippet, or API reference, invoke `context7` to verify syntax, available methods, and current best practices.

**Version Strategy**:
- Use LTS (Long-Term Support) versions for major frameworks: ROS 2 Humble, Ubuntu 22.04
- Use latest stable releases for libraries and tools (document version in code comments)
- Document framework versions in code examples: `# Tested with ROS 2 Humble (verified via context7: YYYY-MM-DD)`

**Fallback Strategy** (if `context7` is unavailable or returns no results):
1. Escalate to user with options: "Documentation unavailable for [library]. Approve one of: (A) Use official GitHub README (provide URL), (B) Use last known stable docs (provide URL), (C) Defer this example."
2. Document fallback source in code comment: `# Source: <URL> (context7 unavailable, approved by user on <date>)`
3. Flag chapter for review when `context7` becomes available

**Documentation Source Hierarchy** (in case of conflicting information):
1. Official project documentation (primary authority)
2. Framework maintainer blog posts or release notes
3. Peer-reviewed academic papers
4. High-quality community tutorials (Stack Overflow, ROS Discourse)
5. User must approve use of sources beyond level 3

### II. Mandatory Chapter Structure

Every chapter MUST follow the prescribed template exactly:
- **Learning Outcomes**: Measurable, action-oriented objectives
- **Overview**: High-level context and motivation
- **Key Concepts**: Foundational terminology and definitions (bold on first use)
- **Main Content**: Technical exposition with subsections as needed
- **Code Examples**: Fenced blocks with language tags where applicable
- **Math**: LaTeX notation in `$ $` or `$$ $$` where applicable
- **Summary**: Concise recap of chapter content
- **Review Questions**: 3+ questions testing comprehension

**Rationale**: Consistent structure aids learning, supports Docusaurus site generation, and ensures every chapter meets pedagogical standards.

**Implementation**: Use the template as a checklist; do not deviate from section order or omit required sections.

### III. Sequential Generation with Approval Gates

Chapters MUST be generated **one at a time**, in the specified order. After completing a chapter, generation MUST halt and wait for explicit user approval (`APPROVED`) before proceeding to the next chapter.

**Rationale**: Sequential review allows course correction, ensures quality control, and prevents accumulation of errors across multiple chapters. Large-scale generation without checkpoints risks misalignment with user expectations.

**Implementation**: Output one chapter file, then explicitly request approval. Do not batch-generate multiple chapters.

**Approval Timeout Policy**:
- If no response within 48 hours, send reminder: "Chapter [N] awaiting approval. Reply APPROVED to proceed or REQUEST_REVISION with feedback."
- If no response after 7 days, pause workflow and notify: "Workflow paused due to inactivity. Resume anytime by providing approval status."
- User may request revisions at any time before approval; incorporate feedback and resubmit

### IV. Technical Precision and Engineering Depth

Content MUST be **technical, precise, and authoritative**. This is a university-level engineering textbook for students studying robotics, AI, and embedded systems.

**Requirements**:
- Use correct terminology (e.g., "kinematics" not "movement math")
- Provide rigorous explanations (e.g., derive equations, explain algorithmic complexity)
- Include realistic, executable code examples
- Reference industry-standard tools (ROS 2 Humble, Gazebo, NVIDIA Isaac, Unity)

**Don't**: Use vague explanations, oversimplify critical concepts, or include "toy" examples that don't reflect real-world robotics engineering.

**Rationale**: Students rely on this textbook to build professional competency. Shallow or imprecise content undermines their education and career preparation.

**Measurable Quality Criteria**:
- **Code Examples**: Include ≥2 executable code examples per major concept; each example >15 lines for non-trivial topics
- **Mathematical Rigor**: Derive ≥50% of non-trivial equations with step-by-step operations; define all variables before use
- **Depth Indicators**:
  - ✅ GOOD: "The Jacobian matrix J relates joint velocities q̇ to end-effector velocity ẋ via ẋ = J(q)q̇. Deriving J for a 2-link planar arm: [step-by-step derivation with partial derivatives]"
  - ❌ BAD: "The robot calculates angles automatically using inverse kinematics."
- **Terminology Precision**: Bold key terms on first use; maintain consistent naming (e.g., always "end-effector" not "gripper" or "hand" interchangeably)

### V. Markdown/MDX for Docusaurus

All content MUST be valid **Markdown** or **MDX** compatible with Docusaurus static site generation.

**Formatting rules**:
- Use `#`, `##`, `###` for headings (semantic hierarchy)
- Use fenced code blocks with language identifiers (```python, ```bash, ```xml)
- Use LaTeX math in `$ $` (inline) or `$$ $$` (display)
- Bold key terms on first use: `**term**`
- Use standard Markdown lists, links, images

**Rationale**: Docusaurus requires specific Markdown/MDX syntax for proper rendering, navigation, and search indexing.

**Implementation**: Test-render chapters in Docusaurus environment before final approval.

### VI. Specialized Task Delegation

Complex content generation MUST leverage Claude Code's Task tool with specialized agents for focused expertise and quality assurance.

**When to Delegate** (use decision criteria below):
- **Math-heavy sections**: ≥3 equations requiring derivation (kinematics, dynamics, control theory, optimization)
- **Complex code examples**: Multi-file examples, >50 lines, or requiring framework-specific patterns
- **API verification**: When `context7` returns extensive documentation requiring synthesis

**Delegation Patterns**:

1. **Mathematical Derivations**
   - **Trigger**: Sections on kinematics, Jacobians, dynamics equations, ZMP, trajectory planning
   - **Implementation**: Use Task tool with `subagent_type='general-purpose'`
   - **Prompt Template**: "Derive [equation/concept] for [context] with step-by-step mathematical rigor. Define all variables. Show intermediate steps. Include dimensional analysis where applicable."
   - **Output Requirements**: Variables defined, units specified, matrix dimensions noted, derivation steps numbered

2. **Code Example Generation**
   - **Trigger**: Code blocks >50 lines, ROS 2 packages, simulation setups, multi-file examples
   - **Implementation**: Use Task tool with `subagent_type='general-purpose'`
   - **Prompt Template**: "Generate executable code for [task] using [framework]. Verify syntax via context7 MCP tool for [library]. Include inline comments explaining non-obvious logic. Follow [framework] naming conventions."
   - **Output Requirements**: Tested or simulation command provided, includes error handling, follows PEP 8 (Python) or framework style guide

3. **Glossary Term Management**
   - **Trigger**: During chapter generation, maintain running list of new technical terms
   - **Implementation**: Create `glossary-terms-temp.md` during drafting; after chapter approval, manually merge into `g1-glossary.md`
   - **Format**: Each term entry includes: bold term, definition (1-2 sentences), first usage chapter reference

**Rationale**: Delegation ensures specialized focus, reduces context switching, and allows quality checks on complex technical content. Subagents can dedicate full attention to mathematical rigor or code correctness without balancing narrative flow.

**Quality Gates**:
- Math subagent output must be reviewed for dimensional consistency and variable definitions
- Code subagent output must include execution/test instructions
- Reject verbose subagent responses that violate Principle VII (No Filler); require concise, essential output

### VII. No Filler, No Deviation

Content MUST be **concise, essential, and on-target**. Do not:
- Add content not specified in the chapter outline
- Reorder modules or chapters
- Insert "nice-to-have" explanations that dilute focus
- Include placeholder or incomplete sections

**Rationale**: The textbook has a defined scope (4 modules, 14 chapters, 3 appendices). Scope creep dilutes focus and delays completion. Every sentence must serve a learning objective.

**Implementation**: Before writing, confirm alignment with specified chapter outline. After writing, audit for unnecessary content.


### VIII. Textbook Delivery Platform Requirements (Docusaurus)

This Principle governs the delivery, presentation, access control, and internationalization of the digital textbook platform. All mandates apply to the production system, staging environments, and any derivative deployments.


#### 8.1 Authentication & Access Control

**8.1.1 Status: REMOVED**

Authentication requirements have been removed from the platform. All documentation routes under `/chapters/**` are publicly accessible without authentication.

**Rationale**: Docusaurus is a static site generator and does not support server-side authentication mechanisms. Implementing OIDC authentication would require significant architectural changes including adding a backend service, which is beyond the scope of the current platform capabilities.

#### 8.2 Internationalization Requirements

**8.2.1 ~~Mandate (Search Capability)~~ [REMOVED]**

~~The platform must provide full-text search accessible from the global Navbar.~~

**Rationale**: Search functionality removed due to lack of Algolia credentials. Implementing search requires DocSearch approval or running a custom crawler, which is beyond current scope.

**8.2.2 Mandate (Internationalization + RTL Compliance)**

The system must support multilingual rendering and layout.

**Acceptance Criteria — i18n**

AC-8.2-I1: Supported target locale: Urdu (ur).

AC-8.2-I2: Urdu rendering must use Right-to-Left (RTL) layout, with full mirroring of UI components where appropriate.

AC-8.2-I3: The user must be able to switch language from the global navigation.

AC-8.2-I4: All layouts must maintain readability, alignment, and structural integrity in both LTR and RTL modes.

#### 8.3 Footer Architecture & Modular Navigation Contracts

**8.3.1 Mandate (High-Level Footer Structure)**

The Footer must reflect the macro-structure of the textbook rather than enumerating chapters.

**Acceptance Criteria**

AC-8.3-A: Footer links must display Module Titles only (e.g., "Module 1: The Robotic Nervous System").

AC-8.3-B: Selecting a module must open the first chapter of that module.

AC-8.3-C: No chapter-level lists may appear in the footer.

AC-8.3-D: Footer behavior and layout must remain consistent across all pages.

### IX. Modern UI/UX Design Standards (Futuristic Interface Requirements)

This Principle establishes the visual design system, interaction patterns, and user experience requirements for the platform interface to reflect modern, cutting-edge aesthetics appropriate for a Physical AI & Robotics textbook.

#### 9.1 Visual Design System

**9.1.1 Color Palette & Theme**

The platform MUST implement a futuristic, tech-forward color palette:

**Primary Colors**:
- **Cyber Blue**: `#00D9FF` - Primary brand color, high-tech aesthetic
- **Electric Purple**: `#8B5CF6` - Accent color for CTAs and emphasis
- **Neon Green**: `#10B981` - Success states, active indicators
- **Deep Space**: `#0A0E27` - Primary background (dark theme)
- **Soft White**: `#F8FAFC` - Text and backgrounds (light theme)

**Gradients** (Required for UI elements):
- Radial gradient: `radial-gradient(circle at 20% 50%, rgba(139, 92, 246, 0.3), transparent 50%)`
- Linear gradient: `linear-gradient(135deg, #00D9FF 0%, #8B5CF6 100%)`
- Mesh gradient backgrounds for depth and dimension

**Typography**:
- **Headings**: Inter, SF Pro Display, or system sans-serif with font-weight 700-900
- **Body**: Inter, -apple-system, BlinkMacSystemFont with font-weight 400-500
- **Code**: JetBrains Mono, Fira Code, or monospace with ligatures enabled
- **Font Sizes**: Use fluid typography (clamp) for responsive scaling
  - H1: `clamp(2.5rem, 5vw, 4.5rem)`
  - H2: `clamp(2rem, 4vw, 3rem)`
  - Body: `clamp(1rem, 1.5vw, 1.125rem)`

**Spacing System**:
- Use 8px base grid: 8px, 16px, 24px, 32px, 48px, 64px, 96px, 128px
- Consistent spacing creates visual rhythm

**9.1.2 Visual Effects & Depth**

**Glassmorphism** (Required for cards, modals, navigation):
- `background: rgba(255, 255, 255, 0.1)`
- `backdrop-filter: blur(10px) saturate(180%)`
- `border: 1px solid rgba(255, 255, 255, 0.18)`
- Subtle shadow: `box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37)`

**Neumorphism** (Optional for buttons, interactive elements):
- Soft shadows creating raised/pressed appearance
- `box-shadow: 12px 12px 24px #0a0b0f, -12px -12px 24px #1a1d2f`

**Glow Effects** (Required for CTAs, hover states):
- `box-shadow: 0 0 20px rgba(0, 217, 255, 0.6), 0 0 40px rgba(139, 92, 246, 0.3)`
- Animated glow on hover: transition duration 300ms

**9.1.3 Imagery & Graphics**


**Icons**:
- Outlined style with 2px stroke weight
- Consistent 24px × 24px base size
- Use Heroicons, Lucide, or Phosphor icon sets
- Animated icons on hover (rotate, scale, color transition)

#### 9.2 Micro-Interactions & Animation Principles

**9.2.1 Motion Design Standards**

All animations MUST follow these principles:

**Timing Functions**:
- Ease-out for entrances: `cubic-bezier(0.16, 1, 0.3, 1)`
- Ease-in for exits: `cubic-bezier(0.7, 0, 0.84, 0)`
- Spring physics for playful interactions: `cubic-bezier(0.34, 1.56, 0.64, 1)`

**Duration**:
- Quick (hover, focus): 150ms
- Medium (page elements): 300ms
- Slow (page transitions): 500ms
- Never exceed 800ms for functional animations

**Animation Types** (Required):

1. **Button Interactions**:
   ```css
   /* Hover: scale up, glow, color shift */
   transform: scale(1.05);
   box-shadow: 0 0 20px currentColor;
   transition: all 300ms cubic-bezier(0.34, 1.56, 0.64, 1);
   ```

2. **Card Hover**:
   ```css
   /* Lift and tilt on hover */
   transform: translateY(-8px) rotateX(2deg);
   box-shadow: 0 20px 40px rgba(0, 0, 0, 0.3);
   ```

3. **Page Load**:
   - Staggered fade-in for page elements (100ms delay between items)
   - Slide-up animation: `translateY(20px)` → `translateY(0)`

4. **Scroll Animations**:
   - Intersection Observer for fade-in on scroll
   - Parallax backgrounds (subtle, performance-conscious)

**11.2.2 Loading States**

**Skeleton Screens** (Required):
- Animated gradient shimmer effect
- Match layout of actual content
- Use CSS animation, not JavaScript

**Progress Indicators**:
- Circular spinners with gradient stroke
- Linear progress bars with animated gradient fill
- Pulse animation for pending states

#### 9.3 Responsive & Adaptive Design

**9.3.1 Breakpoints**

Use mobile-first approach with these breakpoints:
- **Mobile**: 0-640px (1 column)
- **Tablet**: 641px-1024px (2 columns)
- **Desktop**: 1025px-1536px (3 columns, max)
- **Large**: 1537px+ (constrained max-width: 1536px)

**9.3.2 Adaptive Components**

**Navigation**:
- Mobile: Hamburger menu with slide-in drawer (backdrop blur)
- Desktop: Horizontal navbar with dropdowns
- Sticky header with blur background on scroll


**Cards/Features**:
- Mobile: Stack vertically, full-width
- Tablet: 2-column grid with gap
- Desktop: 3-column grid, equal heights

**9.3.3 Touch & Gesture Support**

- Minimum touch target: 44px × 44px (WCAG 2.5.5)
- Swipe gestures for mobile carousels
- Pinch-to-zoom disabled for UI, enabled for images/diagrams

#### 9.4 Performance & Accessibility in Modern UI

**9.4.1 Performance Requirements**

**Core Web Vitals** (MUST meet):
- **LCP** (Largest Contentful Paint): < 2.5s
- **FID** (First Input Delay): < 100ms
- **CLS** (Cumulative Layout Shift): < 0.1

**Optimization Strategies**:
- Lazy load images and heavy components (React.lazy, Intersection Observer)
- Use `will-change` sparingly for animated elements
- Prefer CSS transforms/opacity over layout-triggering properties
- Reduce motion for users with `prefers-reduced-motion` preference

**9.4.2 Accessibility (WCAG 2.1 AA Compliance)**

**Color Contrast**:
- Text on background: minimum 4.5:1 ratio
- Large text (18pt+): minimum 3:1 ratio
- Interactive elements: maintain contrast in all states

**Keyboard Navigation**:
- Visible focus indicators (2px solid outline, offset 2px)
- Logical tab order
- Skip links for main content
- Keyboard shortcuts documented

**Screen Readers**:
- Semantic HTML (header, nav, main, section, footer)
- ARIA labels for interactive elements
- Alt text for all images (descriptive, not decorative)
- Live regions for dynamic content

**Motion Sensitivity**:
```css
@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

**11.4.3 Dark Mode Support**

**Requirements**:
- Auto-detect system preference: `prefers-color-scheme`
- Manual toggle switch in navbar
- Smooth transition between themes (300ms)
- Persist preference in localStorage

**Color Adjustments**:
- Dark theme: reduce saturation by 10-15%
- Increase contrast for readability
- Soften shadows, use lighter glows

#### 9.5 Component-Specific Requirements

**9.5.1 Feature Cards**

**Structure**:
- Card container: glassmorphism or solid with border
- Icon: Top or left-aligned, 48px × 48px
- Title: H3, bold, primary color
- Description: 2-3 lines, medium weight
- Hover effect: Lift, glow, subtle tilt

**Grid Layout**:
- Equal height cards using CSS Grid
- Gap: 24px (mobile), 32px (desktop)
- Responsive: 1/2/3 columns based on breakpoint

**9.5.2 Call-to-Action Buttons**

**Primary Button**:
```css
/* Gradient background, white text, glow effect */
background: linear-gradient(135deg, #00D9FF, #8B5CF6);
color: #FFFFFF;
padding: 16px 32px;
border-radius: 12px;
font-weight: 600;
box-shadow: 0 4px 16px rgba(139, 92, 246, 0.4);

/* Hover: scale, enhanced glow */
&:hover {
  transform: scale(1.05);
  box-shadow: 0 8px 32px rgba(139, 92, 246, 0.6);
}
```

**Secondary Button**:
- Border: 2px solid current color
- Background: transparent or subtle fill
- Text: primary color
- Hover: fill with color, text inverts

#### 9.6 Implementation Checklist

Before approving UI/UX design, verify:

- [ ] Color palette uses specified CSS variables
- [ ] Typography uses fluid scaling (clamp)
- [ ] Glassmorphism applied to cards/overlays
- [ ] All animations have proper easing functions
- [ ] Loading states implemented (skeleton screens)
- [ ] Responsive at all breakpoints (mobile, tablet, desktop)
- [ ] Touch targets minimum 44px × 44px
- [ ] Color contrast ratios meet WCAG 2.1 AA
- [ ] Focus indicators visible and styled
- [ ] Dark mode toggle implemented and functional
- [ ] `prefers-reduced-motion` respected
- [ ] Core Web Vitals targets met (LCP, FID, CLS)
- [ ] Lazy loading for images and heavy components
- [ ] Semantic HTML structure
- [ ] ARIA labels for interactive elements
- [ ] Performance requirements met (pages load under 3 seconds)
- [ ] Optimized images with compression
- [ ] Mobile-first responsive design

## Content Quality Standards

### Code Examples
- MUST be executable (tested in target environment where feasible)
- MUST include language tag in fenced block
- MUST include explanatory comments where logic is non-obvious
- MUST follow framework conventions (e.g., ROS 2 naming, Python PEP 8)

### Mathematical Notation
- MUST use LaTeX syntax: inline `$ ... $`, display `$$ ... $$`
- MUST define variables and symbols before use
- MUST provide derivation steps for non-trivial equations
- MUST include units where applicable
- **Equation Numbering**: Use labels for referenced equations: `$$ J = \frac{\partial x}{\partial q} $$ {#eq:jacobian}` and reference as "Equation {#eq:jacobian}" or "see Eq. {#eq:jacobian}"
- For equation sequences, number consecutively within chapter: `{#eq:3.1}`, `{#eq:3.2}`, etc.

### Visual Content Standards

**Diagrams and Figures**:
- MUST include descriptive alt text for accessibility: `![Forward kinematic chain for 6-DOF manipulator](path/to/image.svg)`
- MUST use vector formats (SVG, PDF) for diagrams where possible for scalability
- MUST include captions with figure numbering: `**Figure 3.2**: Forward kinematic chain showing joint frames and transformation matrices`
- MUST cite source or license if not original: `(Source: ROS 2 Documentation, CC-BY 4.0)` or `(Original diagram)`
- **Acceptable Sources**:
  - Original diagrams created with draw.io, TikZ, Matplotlib, PlantUML
  - Open-source images with permissive licenses (CC-BY, CC0, MIT)
  - Official framework documentation images (with attribution)
  - User must approve any proprietary or restrictive-license images

**Tables**:
- MUST use Markdown table syntax with header row
- MUST include caption above table: `**Table 2.1**: ROS 2 DDS implementations comparison`
- MUST align numeric data right, text left for readability
- For complex tables (>5 columns), consider splitting or using scrollable container

### Terminology
- MUST bold key terms on first use: `**inverse kinematics**`
- MUST maintain consistency across chapters (e.g., always "end-effector" not alternating with "gripper")
- MUST add new terms to glossary: During chapter drafting, maintain `glossary-terms-temp.md`; merge into appendix g1 after approval

## Development Workflow

### Chapter Generation Flow
1. **Preparation**: Review chapter outline, identify required documentation sources
2. **Documentation Lookup**: Use `context7` to fetch current official docs for all frameworks/APIs referenced
3. **Drafting**: Follow mandatory template structure; delegate complex math/code to Task tool subagents as needed
4. **Self-Review**: Complete the checklist below before submitting for approval
5. **Output**: Write chapter to `c<number>-<slug>.md` in project root or designated chapters directory
6. **Approval Gate**: Wait for user `APPROVED` (or `REQUEST_REVISION`) before next chapter

**Self-Review Checklist** (complete before step 5):
- [ ] All required sections present (Learning Outcomes, Overview, Key Concepts, Main Content, Code Examples, Summary, Review Questions)
- [ ] All code blocks have language tags (```python, ```bash, ```xml, etc.)
- [ ] Math symbols and variables defined before use; LaTeX syntax correct
- [ ] No placeholder text (TODO, TKTK, ???, [fill in], etc.)
- [ ] `context7` invoked for all API references and framework-specific code
- [ ] Framework versions documented in code comments where applicable
- [ ] Key terms bolded on first use; terminology consistent with previous chapters
- [ ] Figures/diagrams include alt text and captions with numbering
- [ ] Review questions (≥3) test comprehension of chapter content
- [ ] Word count within target range (typically 3000-5000 words; ≤6000 unless outline specifies otherwise)
- [ ] No content beyond chapter scope (check against outline)

### Filename Convention
- Chapter files: `c1-foundations.md`, `c2-ros2-architecture.md`, etc.
- Appendix files: `a1-hardware-setup.md`, `g1-glossary.md`

### Version Control
- Each chapter approval represents a checkpoint
- User may request revisions before approval; incorporate feedback and resubmit
- **Commit Message Format**: Use structured format for traceability:
  - Chapter work: `chap(c<N>): <verb> <summary>`
    - Examples: `chap(c3): add kinematics derivation`, `chap(c5): fix ZMP equation typo`
  - Appendix work: `appendix(<ID>): <verb> <summary>`
    - Example: `appendix(g1): add 15 new terms from chapter 4`
  - Constitution/structure: `meta: <summary>`
    - Example: `meta: update constitution to v1.3.0`
  - Use present-tense verbs: add, update, fix, remove, refactor

## Governance

### Amendment Process
This constitution may be amended if:
- Pedagogical requirements change (e.g., new learning objectives)
- Technical stack updates require different documentation sources
- Docusaurus version upgrade changes Markdown requirements
- UI/UX standards require modernization

Amendments require:
1. Explicit proposal with rationale
2. User approval
3. Version bump (MAJOR for principle changes, MINOR for additions, PATCH for clarifications)
4. Propagation to dependent templates

### Compliance Review
All chapter outputs MUST be reviewed against this constitution before approval. Non-compliance requires revision.

### Emergency Post-Approval Corrections
**Critical errors** discovered after chapter approval (factual errors, safety issues, broken code, incorrect equations) may be corrected without re-approval or version bump. Log all post-approval corrections in `errata.md` with:
- Chapter and section affected
- Error description
- Correction applied
- Date and committer
- Example: `c3, Section 3.2: Fixed Jacobian derivation sign error in Eq. 3.4 (2025-12-07, user123)`

**Non-critical changes** (style, wording improvements, additional examples) require standard approval workflow.

### Versioning Policy
Constitution version follows semantic versioning:
- **MAJOR**: Backward-incompatible principle changes (e.g., removing approval gates)
- **MINOR**: New principles or significant expansions
- **PATCH**: Clarifications, typo fixes, non-semantic refinements

### Runtime Guidance
For operational details on executing chapter generation commands, refer to `CLAUDE.md` and `.claude/commands/sp.*.md` files.

**Version**: 1.7.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-10

**Amendment Notes**:
- (2025-12-07) Added Principle VIII: RAG Chatbot Frontend (Chatkit UI) - New principle establishing technical architecture for the Chatkit UI integration
- (2025-12-07) Renumbered Principle VIII to IX: Integrated RAG Chatbot System (The "Expert Instructor") - Backend RAG system principle moved to accommodate new frontend principle
- (2025-12-07) Renumbered subsequent principles: Textbook Delivery Platform moved from Principle IX to X, Modern UI/UX Design Standards moved from Principle X to XI
- (2025-12-09) Updated Principle VIII Section 8.1: Enhanced landing page requirements with conversion optimization best practices (AC-8.1-E, AC-8.1-F, AC-8.1-G)
- (2025-12-09) Updated Principle IX Section 9.5.1: Enhanced landing page hero section with improved content structure and visual hierarchy requirements
- (2025-12-09) Added Principle IX Section 9.5.2: New section for landing page conversion optimization requirements
- (2025-12-09) Updated Principle IX Section 9.6: Expanded implementation checklist with additional landing page verification items
- (2025-12-10) Removed Principle VIII Section 8.1: Landing Environment, Entry Flow, and Navigation Priority - Landing page requirements removed from project scope
- (2025-12-10) Removed Principle IX Section 9.5.1: Landing Page Hero requirements removed
- (2025-12-10) Removed Principle IX Section 9.5.2: Landing Page Conversion Optimization requirements removed
- (2025-12-10) Updated Principle IX Section 9.6: Implementation checklist updated to remove landing page specific items
- (2025-12-10) Updated Principle IX introduction: Removed landing page specific references
- (2025-12-10) Updated various sections to remove hero section and landing page specific requirements throughout the constitution
