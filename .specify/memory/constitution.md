<!--
SYNC IMPACT REPORT
==================
Version Change: 1.0.0 → 1.1.0
Amendment Type: MINOR (new sections, expanded principles, non-breaking)
Amendment Date: 2025-12-06

Modified Principles:
  - Principle I: Added version strategy, fallback strategy, documentation source hierarchy
  - Principle III: Added approval timeout policy (48hr reminder, 7-day pause)
  - Principle IV: Added measurable quality criteria with examples
  - Principle VI: Complete rewrite - replaced undefined <<call:...>> syntax with concrete Claude Code Task tool invocations

Added Sections:
  - Visual Content Standards (diagrams, figures, tables)
  - Self-Review Checklist (11-item pre-approval checklist)
  - Commit Message Format (structured conventions)
  - Emergency Post-Approval Corrections (errata workflow)
  - Equation Numbering (LaTeX label/reference syntax)

Enhanced Sections:
  - Mathematical Notation: Added equation numbering guidance
  - Terminology: Added glossary update process (glossary-terms-temp.md workflow)
  - Chapter Generation Flow: Integrated self-review checklist, clarified subagent delegation

Templates Requiring Updates:
  ⚠️ spec-template.md - may need visual content placeholders (diagrams, tables)
  ⚠️ plan-template.md - should reference documentation source hierarchy
  ⚠️ tasks-template.md - should include self-review checklist task
  ✅ phr-template.prompt.md - no changes required

Follow-up TODOs:
  1. Review and update affected templates if visual content sections needed
  2. Create errata.md file in project root for post-approval corrections tracking
  3. Validate self-review checklist during first chapter generation (c1)

Rationale for Amendment:
  - Address operational gaps (context7 fallback, subagent invocation model)
  - Improve enforceability (measurable criteria, self-review checklist)
  - Add missing standards (visual content essential for robotics textbook)
  - Enhance workflow clarity (commit format, timeout policy, glossary process)
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
    - Example: `meta: update constitution to v1.1.0`
  - Use present-tense verbs: add, update, fix, remove, refactor

## Governance

### Amendment Process
This constitution may be amended if:
- Pedagogical requirements change (e.g., new learning objectives)
- Technical stack updates require different documentation sources
- Docusaurus version upgrade changes Markdown requirements

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

**Version**: 1.1.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-06
