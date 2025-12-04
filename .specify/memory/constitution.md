<!--
SYNC IMPACT REPORT
==================
Version Change: INITIAL → 1.0.0
Modified Principles: None (initial version)
Added Sections:
  - Core Principles (7 principles)
  - Content Quality Standards
  - Development Workflow
  - Governance
Templates Requiring Updates:
  ✅ spec-template.md - aligned with chapter structure requirements
  ✅ plan-template.md - aligned with sequential generation workflow
  ✅ tasks-template.md - aligned with documentation verification tasks
Follow-up TODOs: None
==================
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Authoritative Documentation Sources (NON-NEGOTIABLE)

**MUST** use `context7` MCP tool to retrieve the latest official documentation for all technical frameworks, APIs, and libraries before generating any technical or code-specific content.

**Rationale**: The textbook targets university-level engineering education. Accuracy is paramount. Using outdated or hallucinated API references undermines educational value and student success. The `context7` tool provides access to current official documentation for ROS 2, Gazebo, Isaac Sim, Unity, and other critical frameworks.

**Implementation**: Before writing any code example, configuration snippet, or API reference, invoke `context7` to verify syntax, available methods, and current best practices.

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

### IV. Technical Precision and Engineering Depth

Content MUST be **technical, precise, and authoritative**. This is a university-level engineering textbook for students studying robotics, AI, and embedded systems.

**Requirements**:
- Use correct terminology (e.g., "kinematics" not "movement math")
- Provide rigorous explanations (e.g., derive equations, explain algorithmic complexity)
- Include realistic, executable code examples
- Reference industry-standard tools (ROS 2 Humble, Gazebo, NVIDIA Isaac, Unity)

**Don't**: Use vague explanations, oversimplify critical concepts, or include "toy" examples that don't reflect real-world robotics engineering.

**Rationale**: Students rely on this textbook to build professional competency. Shallow or imprecise content undermines their education and career preparation.

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

### VI. Subagent and Skill Invocation

Complex content generation MUST leverage specialized subagents and skills via tagged invocation:

- **Math Subagent**: For deriving equations, explaining kinematics, control theory
  ```
  <<call:math_subagent>>
  [math content]
  <</call:math_subagent>>
  ```

- **Code Generation Skill**: For generating and validating executable code examples
  ```
  <<call:code_generation_skill>>
  [code]
  <</call:code_generation_skill>>
  ```

- **Terminology Skill**: For maintaining consistent glossary entries
  ```
  <<call:terminology_skill>>
  [terms]
  <</call:terminology_skill>>
  ```

**Rationale**: Subagents provide specialized expertise and consistency. Math subagent ensures equation accuracy; code skill ensures executable examples; terminology skill maintains glossary coherence.

**Implementation**: When encountering math-heavy sections (e.g., kinematics, ZMP), invoke math subagent. For code examples, invoke code generation skill to validate syntax and execution.

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
- MUST use LaTeX syntax
- MUST define variables and symbols before use
- MUST provide derivation steps for non-trivial equations
- MUST include units where applicable

### Terminology
- MUST bold key terms on first use
- MUST maintain consistency across chapters
- MUST add new terms to glossary (Appendix G1)

## Development Workflow

### Chapter Generation Flow
1. **Preparation**: Review chapter outline, identify required documentation sources
2. **Documentation Lookup**: Use `context7` to fetch current official docs
3. **Drafting**: Follow mandatory template, invoke subagents as needed
4. **Self-Review**: Validate against all principles and quality standards
5. **Output**: Write chapter to `c<number>-<slug>.md`
6. **Approval Gate**: Wait for user `APPROVED` before next chapter

### Filename Convention
- Chapter files: `c1-foundations.md`, `c2-ros2-architecture.md`, etc.
- Appendix files: `a1-hardware-setup.md`, `g1-glossary.md`

### Version Control
- Each chapter approval represents a checkpoint
- User may request revisions before approval
- Track major content changes in commit messages

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

### Versioning Policy
Constitution version follows semantic versioning:
- **MAJOR**: Backward-incompatible principle changes (e.g., removing approval gates)
- **MINOR**: New principles or significant expansions
- **PATCH**: Clarifications, typo fixes, non-semantic refinements

### Runtime Guidance
For operational details on executing chapter generation commands, refer to `CLAUDE.md` and `.claude/commands/sp.*.md` files.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
