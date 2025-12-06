# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-06 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

## Summary

Generate a university-level textbook (14 chapters, 3 appendices) on Physical AI & Humanoid Robotics using an iterative, approval-gated workflow. Each chapter follows a mandatory template structure, uses context7 MCP tool for technical accuracy (ROS 2 Humble, Gazebo, Isaac Sim, Unity, VLA frameworks), and undergoes an 11-item self-review checklist before submission. Content is formatted as Markdown/MDX for Docusaurus deployment with LaTeX math, fenced code blocks, and visual content standards (SVG diagrams, numbered captions, alt text).

**Technical Approach**: Sequential chapter generation with approval gates. For each chapter (C1-C14) and appendix (A1-A2, G1):
1. Invoke context7 to retrieve official API documentation
2. Draft content following mandatory template (Learning Outcomes, Overview, Key Concepts, Main Content, Code Examples, Math, Summary, Review Questions)
3. Delegate complex tasks (≥3 equations, >50 line code examples) to Task tool subagents
4. Apply 11-item self-review checklist
5. Commit chapter to feature branch `feature/chapter-XX-slug`
6. Halt and wait for explicit "APPROVED" user confirmation
7. Merge terms into glossary-terms-temp.md, finalize glossary in G1

## Technical Context

**Language/Version**: Markdown/MDX (Docusaurus 2.x/3.x compatible); Python 3.10+ (for code examples using rclpy); XML (URDF); Bash (setup scripts); YAML (configuration)

**Primary Dependencies**:
- **context7 MCP Tool**: Retrieve official documentation for ROS 2 Humble, Gazebo, NVIDIA Isaac Sim, Unity, VLA frameworks (fallback strategy: escalate to user for manual documentation source approval)
- **Claude Code Task Tool**: Delegate math derivations (≥3 equations), code generation (>50 lines), documentation synthesis
- **Docusaurus**: Static site generator for final deployment (KaTeX/MathJax for LaTeX rendering)
- **Git + GitHub MCP**: Version control with structured commit messages (`chap(c<N>):`, `appendix(<ID>):`, `meta:`)

**Storage**: File-based Markdown/MDX chapters (c1-foundations.md, c2-ros2-architecture.md, etc.) in project root or designated `chapters/` directory. Glossary terms tracked in `glossary-terms-temp.md` during generation, consolidated into `g1-glossary.md` post-approval. Errata logging in `errata.md`.

**Testing**: Manual validation via 11-item self-review checklist per chapter. No automated test harness. Code examples validated via context7 syntax verification + simulation/execution commands documented in comments.

**Target Platform**: Docusaurus-based static website (Markdown/MDX source files). Educational content for university students with local ROS 2 Humble environment (Ubuntu 22.04, simulation tools).

**Project Type**: Educational content generation (textbook) - documentation-focused, not traditional software application. Source structure is chapter files + appendices + glossary.

**Performance Goals**:
- **Generation Speed**: ≤90 minutes per chapter draft (including context7 lookups, Task tool delegation, self-review)
- **Approval Latency**: ≥80% of chapters receive feedback within 72 hours
- **Context7 Success Rate**: ≥95% of API lookups succeed; ≤5% fallback to manual documentation
- **Student Comprehension Time**: <90 minutes average per chapter (measured via review questions)

**Constraints**:
- **Sequential Approval Gate**: MUST halt after each chapter until explicit "APPROVED" received (48hr reminder, 7-day pause)
- **Word Count**: 3000-5000 words per chapter target, ≤6000 maximum (exceptions require user approval)
- **Technical Accuracy**: 100% of code examples/APIs verified via context7 before inclusion (fallback documented)
- **Template Compliance**: 100% adherence to 11-item self-review checklist (no missing sections, no placeholders, all code tagged)

**Scale/Scope**:
- **14 Chapters** organized into 4 modules (Module 1: ROS 2 [C1-C4], Module 2: Simulation [C5-C8], Module 3: Edge Computing [C9-C11], Module 4: Humanoid Integration [C12-C14])
- **3 Appendices**: Hardware Setup (A1), Reference Tables (A2), Glossary (G1)
- **Estimated Content**: ~56,000 words total (14 chapters × 4000 avg), ~280 code examples (14 × 20 avg), ~140 equations (14 × 10 avg), ~42 review questions (14 × 3 min)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Authoritative Documentation Sources ✅ PASS
- **Requirement**: MUST use context7 MCP tool for all technical framework references
- **Compliance**: FR-004 mandates context7 invocation before generating code/API references for ROS 2 Humble, Gazebo, Isaac Sim, Unity, VLA
- **Fallback Strategy**: FR-004 includes escalation to user with options (GitHub README, last known docs, defer example) + documentation in code comments
- **Version Strategy**: FR-005 specifies LTS versions (ROS 2 Humble, Ubuntu 22.04) with version documentation in comments

### Principle II: Mandatory Chapter Structure ✅ PASS
- **Requirement**: Every chapter MUST follow prescribed template (Learning Outcomes, Overview, Key Concepts, Main Content, Code Examples, Math, Summary, Review Questions)
- **Compliance**: FR-003 enforces exact template structure; FR-009 includes "required sections present" in 11-item self-review checklist
- **Enforcement**: Self-review checklist validation before each chapter submission (blocking gate)

### Principle III: Sequential Generation with Approval Gates ✅ PASS
- **Requirement**: Chapters MUST be generated one at a time with explicit user approval before proceeding
- **Compliance**: FR-006 (halt after each chapter), FR-007 (48hr reminder), FR-008 (7-day pause notification)
- **Workflow**: User master execution plan aligns with constitution - create feature branch per chapter, wait for "APPROVED" keyword

### Principle IV: Technical Precision and Engineering Depth ✅ PASS
- **Requirement**: Content MUST be technical, precise, authoritative (≥2 code examples per concept, ≥50% equation derivations, rigorous explanations)
- **Compliance**: FR-013 (≥2 executable code examples >15 lines), FR-014 (≥50% equation derivations with step-by-step operations), measurable quality criteria in SC-005/SC-006
- **Enforcement**: Self-review checklist includes "≥2 code examples per major concept" and "≥50% equation derivations"

### Principle V: Markdown/MDX for Docusaurus ✅ PASS
- **Requirement**: All content MUST be valid Markdown/MDX with fenced code blocks (language tags), LaTeX math, semantic headings
- **Compliance**: FR-015 (Docusaurus-compatible Markdown/MDX), FR-012 (LaTeX syntax for equations), self-review checklist validates "all code blocks have language tags"
- **Validation**: SC-011 (100% Docusaurus rendering correctness)

### Principle VI: Specialized Task Delegation ✅ PASS
- **Requirement**: MUST use Task tool for math-heavy sections (≥3 equations), complex code (>50 lines), extensive documentation synthesis
- **Compliance**: FR-018 specifies Task tool delegation triggers; user master plan includes Task tool invocations for math/code validation
- **Quality Gates**: Constitution requires rejecting verbose subagent output (FR-018 quality gates implicit)

### Principle VII: No Filler, No Deviation ✅ PASS
- **Requirement**: Content MUST be concise, essential, on-target (no content beyond outline, no placeholders, no scope creep)
- **Compliance**: Self-review checklist includes "No content beyond chapter scope", "No placeholder text", "Word count ≤6000"
- **Enforcement**: Out of Scope section in spec.md explicitly excludes batch generation, videos, translation, automated testing

### Summary: ALL GATES PASS ✅
No constitution violations. Plan fully complies with all 7 principles. No complexity justification required.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── spec.md                          # Feature specification (completed)
├── plan.md                          # This file (/sp.plan output)
├── research.md                      # Phase 0: Documentation sources, tool integration research
├── data-model.md                    # Phase 1: Chapter/Module/Term/Approval Gate entity models
├── quickstart.md                    # Phase 1: How to generate first chapter (C1)
├── contracts/                       # Phase 1: Chapter generation workflow schemas
│   ├── chapter-schema.json          # JSON schema for chapter metadata
│   ├── approval-gate-schema.json    # Approval workflow state machine
│   └── glossary-entry-schema.json   # Glossary term format
├── checklists/
│   └── requirements.md              # Spec quality validation (completed - PASS)
└── tasks.md                         # Phase 2 output (/sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Textbook Content Structure (Markdown/MDX files)
chapters/
├── c1-foundations-physical-ai.md
├── c2-ros2-architecture.md
├── c3-ros2-actions.md
├── c4-urdf-robot-description.md
├── c5-gazebo-simulation.md
├── c6-isaac-sim-integration.md
├── c7-unity-robotics.md
├── c8-vla-models.md
├── c9-edge-computing-jetson.md
├── c10-real-time-control.md
├── c11-sensor-fusion.md
├── c12-whole-body-control.md
├── c13-zmp-walking.md
└── c14-humanoid-integration.md

appendices/
├── a1-hardware-setup.md
├── a2-reference-tables.md
└── g1-glossary.md

# Working Files (temp)
glossary-terms-temp.md               # Running list of terms during generation (deleted post-G1)
errata.md                            # Post-approval critical error corrections

# Docusaurus Configuration (deployment)
docusaurus.config.js                 # Site configuration
sidebars.js                          # Navigation structure (auto-generated from chapters/)
docs/                                # Symlink to chapters/ for Docusaurus
static/                              # Images, diagrams (SVG/PDF)
  ├── images/
  │   ├── c1/                        # Chapter-specific diagrams
  │   ├── c2/
  │   └── ...
  └── equations/                     # Standalone equation images (if needed)

# Version Control
.gitignore                           # Exclude glossary-terms-temp.md, node_modules
package.json                         # npm dependencies (Docusaurus, KaTeX/MathJax)
package-lock.json
```

**Structure Decision**: Educational content project (not traditional web/mobile app). Primary artifacts are Markdown/MDX chapter files organized by module. Docusaurus handles static site generation from `chapters/` directory. No backend/frontend split - purely static documentation. npm package manager used for Docusaurus tooling (KaTeX for LaTeX rendering, markdown plugins).

## Workflow Architecture

### Phase 0: Pre-Generation Research
**Objective**: Resolve all documentation sources, validate context7 access, establish tooling baseline.

**Research Tasks**:
1. **context7 Library ID Resolution**: For each framework (ROS 2 Humble, Gazebo, Isaac Sim, Unity, VLA), resolve Context7-compatible library IDs using `mcp__context7__resolve-library-id` tool
2. **Documentation Availability Check**: Test context7 retrieval for high-priority topics (rclpy API, URDF syntax, Isaac Sim sensors) to validate ≥95% success rate assumption
3. **Fallback Documentation Sources**: Identify backup URLs for each framework (official GitHub repos, ReadTheDocs, framework maintainer blogs) per constitution's 5-level documentation hierarchy
4. **LaTeX Math Rendering**: Validate Docusaurus KaTeX/MathJax plugin configuration supports equation labeling (`{#eq:N.M}`) and cross-referencing
5. **Visual Content Tooling**: Confirm draw.io, TikZ, Matplotlib, PlantUML availability for diagram generation; establish SVG export workflows
6. **Task Tool Delegation Patterns**: Define prompt templates for math subagent (equation derivation), code subagent (>50 line examples), documentation synthesis subagent

**Output**: `research.md` with resolved Context7 library IDs, validated context7 success rates, fallback documentation URLs, tooling baseline.

### Phase 1: Chapter Generation Workflow Design

#### 1A: Data Model (`data-model.md`)
Define entity models for chapter metadata tracking, approval state, glossary terms.

**Entities**:
- **Chapter**: `{id, number, title, slug, module, sections[], wordCount, approvalStatus, generatedPath, contextUsedSources[], reviewChecklistPassed}`
- **Module**: `{number, focusArea, chapterRange[], hardwareContext, learningObjectives[]}`
- **TechnicalTerm**: `{term, definition, firstUsageChapter, glossaryIncluded, relatedTerms[]}`
- **CodeExample**: `{chapterRef, language, lineCount, context7Verified, frameworkVersion, executionCommand, syntaxHighlighting}`
- **Equation**: `{chapterRef, latexSource, label, variableDefinitions[], derivationSteps[], dimensionalUnits, crossReferences[]}`
- **VisualContent**: `{chapterRef, type, fileFormat, filePath, altText, caption, sourceAttribution, chapterPlacement}`
- **ApprovalGate**: `{chapterNumber, submissionTimestamp, approvalStatus, reviewerFeedback, reminderTimestamps[], resolutionTimestamp}`

#### 1B: Workflow Contracts (`contracts/`)
Define JSON schemas and state machines for chapter generation lifecycle.

**Schemas**:
1. **chapter-schema.json**: Chapter metadata structure (maps to Chapter entity)
2. **approval-gate-schema.json**: Approval workflow state transitions (`Draft → UnderReview → [Approved | RevisionRequested] → Approved`)
3. **glossary-entry-schema.json**: Glossary term format (`{term, definition, chapterRef, synonyms[]}`)
4. **self-review-checklist-schema.json**: 11-item checklist structure with pass/fail per item

**State Machine: Approval Gate**
```
Draft → (submit) → UnderReview → (approve) → Approved
                 ↓
           (request_revision) → RevisionRequested → (resubmit) → UnderReview

Reminders:
  - 48 hours after UnderReview: send reminder
  - 7 days after UnderReview: pause workflow
```

#### 1C: Quickstart Guide (`quickstart.md`)
Step-by-step instructions for generating the first chapter (C1: Foundations of Physical AI).

**Quickstart Steps**:
1. Verify constitution loaded and context7 MCP connected
2. Create feature branch: `git checkout -b feature/chapter-01-foundations`
3. Invoke context7 for C1 dependencies:
   - `mcp__context7__get-library-docs` for ROS 2 sensor message types (sensor_msgs)
   - Retrieve Physical AI definitions from academic sources (fallback if context7 unavailable)
4. Draft C1 following mandatory template:
   - Learning Outcomes (3-5 action-oriented objectives)
   - Overview (Physical AI vs Digital AI distinction)
   - Key Concepts (sensor types: LiDAR, IMU, Camera, Force/Torque)
   - Main Content (sensor characteristics, ROS 2 message formats)
   - Code Examples (≥2 examples: sensor message publisher, sensor data subscriber)
   - Math (sensor calibration equations, noise models)
   - Summary (concise recap)
   - Review Questions (≥3 comprehension questions)
5. Delegate complex tasks:
   - If ≥3 equations: invoke Task tool with math subagent prompt
   - If >50 line code: invoke Task tool with code generation subagent
6. Apply 11-item self-review checklist:
   - [ ] All required sections present
   - [ ] All code blocks have language tags
   - [ ] Math symbols/variables defined, LaTeX syntax valid
   - [ ] No placeholder text (TODO, TKTK, ???)
   - [ ] context7 invoked for all API references
   - [ ] Framework versions documented in code comments
   - [ ] Key terms bolded on first use, terminology consistent
   - [ ] Figures/diagrams include alt text and captions
   - [ ] Review questions (≥3) test comprehension
   - [ ] Word count 3000-6000
   - [ ] No content beyond chapter scope
7. Save chapter: `chapters/c1-foundations-physical-ai.md`
8. Update glossary: Add new terms to `glossary-terms-temp.md`
9. Commit: `git commit -m "chap(c1): add foundations of physical AI chapter"`
10. Submit for approval: Halt and wait for user "APPROVED" response
11. On approval: Merge branch, proceed to C2

#### 1D: Agent Context Update
Run `.specify/scripts/bash/update-agent-context.sh claude` to add new technology from this plan to CLAUDE.md.

**Technologies to Add**:
- context7 MCP tool (ROS 2 Humble, Gazebo, Isaac Sim, Unity, VLA library IDs)
- Docusaurus Markdown/MDX formatting requirements
- LaTeX equation labeling conventions (`{#eq:N.M}`)
- Task tool delegation triggers (≥3 equations, >50 lines code)
- Git workflow (feature branch per chapter, approval gates)

### Phase 2: Task Generation (Deferred to `/sp.tasks`)
NOT created by `/sp.plan`. User will run `/sp.tasks` after plan approval to generate testable task breakdown for chapters C1-C14 and appendices A1-A2, G1.

## Chapter Generation Sequence (Post-Planning)

### Module 1: The Robotic Nervous System (ROS 2)

**C1: Foundations of Physical AI**
- **Focus**: Physical AI vs Digital AI distinction; sensor systems (LiDAR, IMU, Camera, Force/Torque)
- **context7 Calls**: ROS 2 sensor message types (sensor_msgs/LaserScan, sensor_msgs/Imu, sensor_msgs/Image, geometry_msgs/WrenchStamped)
- **Code Examples**: Sensor message publisher (Python rclpy), sensor data subscriber with callback
- **Math**: Sensor noise models (Gaussian noise, systematic bias), IMU orientation representation (quaternions)
- **Approval Gate**: Wait for "APPROVED" before C2

**C2: ROS 2 Humble Architecture**
- **Focus**: ROS 2 computational graph (nodes, topics, messages); rclpy fundamentals
- **context7 Calls**: rclpy API (rclpy.init, rclpy.create_node, rclpy.spin), topic communication patterns
- **Code Examples**: Minimal publisher node, minimal subscriber node, talker/listener example
- **Math**: Message latency analysis, publish/subscribe rate calculations
- **Approval Gate**: Wait for "APPROVED" before C3

**C3: ROS 2 Actions and Services**
- **Focus**: Services vs Actions; bridging high-level plans to motor commands
- **context7 Calls**: rclpy Action Server/Client API, action definition format (.action files)
- **Code Examples**: Action Server (Fibonacci example), Action Client with feedback, Service Server/Client
- **Math**: Goal-based control formulation, feedback loop stability
- **Approval Gate**: Wait for "APPROVED" before C4

**C4: URDF and Robot Description**
- **Focus**: Multi-link URDF structure (<link>, <joint>); Rviz visualization
- **context7 Calls**: URDF XML syntax, robot_state_publisher, joint_state_publisher
- **Code Examples**: 6-DOF manipulator URDF, humanoid URDF (simplified), Rviz configuration
- **Math**: Forward kinematics (Denavit-Hartenberg parameters), joint limits
- **Task Tool**: Delegate >50 line URDF file generation to code subagent
- **Approval Gate**: Wait for "APPROVED" before C5

### Module 2: Simulation Environments (Assumed)

**C5-C8**: Gazebo integration, NVIDIA Isaac Sim, Unity Robotics, VLA models
- **context7 Calls**: Gazebo SDF format, Isaac Sim sensor APIs, Unity Robotics Hub, VLA model interfaces
- **Code Examples**: Gazebo world files, Isaac Sim Python API, Unity C# robot controller
- **Math**: Physics simulation (rigid body dynamics, collision detection), sensor simulation (raytracing for LiDAR)
- **Visual Content**: Simulation environment screenshots (SVG diagrams of sensor placement, control loops)
- **Approval Gates**: Sequential after C5, C6, C7, C8

### Module 3: Edge Computing (Assumed)

**C9-C11**: Jetson deployment, real-time control, sensor fusion
- **context7 Calls**: NVIDIA Jetson SDK, real-time Linux (PREEMPT_RT), sensor fusion libraries (Robot Localization)
- **Code Examples**: ROS 2 on Jetson setup, real-time node configuration, Extended Kalman Filter (EKF) for sensor fusion
- **Math**: EKF equations (prediction, update), Kalman gain derivation, state covariance propagation
- **Task Tool**: Delegate EKF derivation to math subagent (≥3 equations)
- **Approval Gates**: Sequential after C9, C10, C11

### Module 4: Humanoid Integration (Assumed)

**C12-C14**: Whole-body control, ZMP walking, VLA integration
- **context7 Calls**: Whole-body control libraries (Pinocchio, Drake), ZMP calculation, VLA APIs
- **Code Examples**: Inverse kinematics solver, ZMP trajectory planner, VLA action execution
- **Math**: Jacobian derivation (6-DOF manipulator), ZMP stability criterion, trajectory optimization (quadratic programming)
- **Task Tool**: Delegate Jacobian/ZMP derivations to math subagent
- **Approval Gates**: Sequential after C12, C13, C14

### Appendices

**A1: Hardware Setup**
- **Focus**: RTX GPU, Jetson Orin, Ubuntu 22.04 requirements, ROS 2 Humble installation
- **Code Examples**: Bash installation scripts, environment setup
- **Approval Gate**: Wait for "APPROVED" before A2

**A2: Reference Tables**
- **Focus**: Kinematics and ZMP derivations (detailed math)
- **Math**: Full Jacobian derivation (Denavit-Hartenberg), ZMP equations, inverse kinematics closed-form solutions
- **Task Tool**: MANDATORY math subagent for all derivations
- **Approval Gate**: Wait for "APPROVED" before G1

**G1: Glossary**
- **Focus**: Consolidate all terms from glossary-terms-temp.md (alphabetical sorting)
- **Format**: Term → Definition (1-2 sentences) → First usage chapter reference
- **Approval Gate**: Final approval completes project

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. All constitution gates pass. No complexity justification required.

**Note**: Sequential approval workflow (Principle III) is not a violation but a core requirement. Each chapter's feature branch (`feature/chapter-XX-slug`) + approval gate is the intended architecture, not added complexity.

## Risk Analysis

### R1: context7 Unavailability Risk
- **Impact**: HIGH - Cannot verify technical accuracy without context7 (blocks FR-004)
- **Mitigation**: Fallback strategy (FR-004) escalates to user for manual documentation source approval. Pre-validate context7 success rate in Phase 0 research (target ≥95%). Maintain backup URLs for all frameworks in research.md.
- **Contingency**: If context7 fails mid-generation, document fallback source in code comment and flag chapter for review when context7 becomes available.

### R2: Approval Latency Risk
- **Impact**: MEDIUM - Delayed approvals block chapter progression (SC-009 target: ≥80% within 72 hours)
- **Mitigation**: Automated reminders (48hr, 7-day per FR-007/FR-008). User master plan specifies GitHub MCP for PR workflow, enabling async review.
- **Contingency**: 7-day pause notification allows indefinite wait without blocking. User can resume anytime.

### R3: Word Count Overrun Risk
- **Impact**: LOW - Chapters exceeding 6000 words require user approval exception
- **Mitigation**: Self-review checklist enforces word count check before submission. Principle VII (No Filler) encourages conciseness.
- **Contingency**: User approves exception or requests condensing. No automatic truncation.

### R4: Glossary Term Inconsistency Risk
- **Impact**: MEDIUM - Terminology drift across 14 chapters undermines SC-004 (≤2% variance)
- **Mitigation**: glossary-terms-temp.md running list tracks all terms. Self-review checklist enforces "terminology consistent with previous chapters" check.
- **Contingency**: Final G1 consolidation allows deduplication and consistency review before project completion.

### R5: Task Tool Delegation Overhead Risk
- **Impact**: LOW - Excessive subagent invocations may increase generation time beyond ≤90min target
- **Mitigation**: Clear delegation triggers (≥3 equations, >50 lines code) prevent overuse. Quality gates reject verbose subagent output (Constitution Principle VI).
- **Contingency**: If generation time exceeds 90min, adjust delegation thresholds (e.g., ≥5 equations instead of ≥3).

## Next Steps

1. **User Approval**: Review this plan.md and approve to proceed
2. **Phase 0 Execution**: Run research tasks to resolve context7 library IDs and validate tooling
3. **Phase 1 Execution**: Generate data-model.md, contracts/, quickstart.md
4. **Agent Context Update**: Run update-agent-context.sh to add new technology to CLAUDE.md
5. **Phase 2 Transition**: User runs `/sp.tasks` to generate task breakdown for C1-C14 and appendices
6. **Implementation**: Begin chapter generation starting with C1 per quickstart.md workflow

**Dependencies**:
- context7 MCP tool must be connected
- GitHub MCP for PR workflow (optional but recommended per user master plan)
- Docusaurus environment setup (npm package manager, KaTeX plugin)
- Git repository initialized with structured commit message conventions

**Estimated Timeline** (excluding approval wait times):
- Phase 0 (Research): ~2 hours
- Phase 1 (Data Model + Contracts + Quickstart): ~3 hours
- Phase 2 (Task Generation via /sp.tasks): ~1 hour
- **Per-Chapter Generation**: ≤90 minutes × 14 chapters = ~21 hours
- **Appendices**: ≤90 minutes × 3 = ~4.5 hours
- **Total Active Work**: ~31.5 hours (spread across weeks/months due to approval gates)
