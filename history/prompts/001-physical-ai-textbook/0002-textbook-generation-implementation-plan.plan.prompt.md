---
id: 0002
title: Textbook generation implementation plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: 123Mismail
command: /sp.plan
labels: ["plan", "textbook", "architecture", "workflow", "ros2"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/plan.md
  - specs/001-physical-ai-textbook/research.md
tests:
  - Constitution Check: ALL GATES PASS (7/7 principles compliant)
---

## Prompt

Master Execution Plan: Textbook Generation (C1 - G1)

User provided detailed execution plan with:
1. Governance and Workflow Initialization (GitHub MCP, feature branches, approval gates)
2. Iterative Generation Cycle (context7 verification, drafting/formatting, terminology management, commit workflow)
3. Module 1 Detailed Execution Sequence (C1-C4: ROS 2 fundamentals)
4. Final Appendices Generation (A1: Hardware, A2: Kinematics/ZMP, G1: Glossary)

Key requirements:
- Sequential chapter generation with approval gates (MUST halt after each chapter for "APPROVED")
- context7 MCP for technical verification (ROS 2, Gazebo, Isaac Sim, Unity, VLA)
- Task tool delegation for math (<<call:math_subagent>>) and code (<<call:code_generation_skill>>)
- Mandatory template structure with 11-item self-review checklist
- Feature branch per chapter: `feature/chapter-XX-slug`
- npm package manager for Docusaurus tooling

## Response snapshot

Created comprehensive implementation plan for Physical AI & Humanoid Robotics textbook generation system.

**Plan Structure**:
- Branch: `001-physical-ai-textbook`
- Plan file: `specs/001-physical-ai-textbook/plan.md` (401 lines)
- Research file: `specs/001-physical-ai-textbook/research.md` (extensive documentation source analysis)

**Constitution Check**: ✅ ALL 7 PRINCIPLES PASS
- Principle I (Authoritative Sources): context7 MCP + fallback strategy ✅
- Principle II (Mandatory Structure): FR-003 enforces template compliance ✅
- Principle III (Sequential Approval): FR-006/007/008 implement gates ✅
- Principle IV (Technical Precision): ≥2 code examples, ≥50% derivations ✅
- Principle V (Markdown/MDX): Docusaurus compatibility validated ✅
- Principle VI (Task Delegation): ≥3 equations, >50 lines code triggers ✅
- Principle VII (No Filler): Self-review checklist enforces conciseness ✅

**Workflow Architecture**:
- **Phase 0 (Research)**: Resolved context7 library IDs, fallback URLs, tooling (KaTeX, SVG, Task delegation patterns)
- **Phase 1 (Design)**: Data model (7 entities), workflow contracts (4 JSON schemas), quickstart guide (C1 generation steps)
- **Phase 2 (Tasks)**: Deferred to `/sp.tasks` command (generates task breakdown for C1-C14, A1-A2, G1)

**Chapter Generation Sequence**:
- Module 1 (C1-C4): ROS 2 fundamentals, rclpy, Actions, URDF
- Module 2 (C5-C8): Gazebo, Isaac Sim, Unity, VLA (assumed topics)
- Module 3 (C9-C11): Jetson, real-time control, sensor fusion (assumed)
- Module 4 (C12-C14): Whole-body control, ZMP, humanoid integration (assumed)
- Appendices (A1-A2, G1): Hardware setup, reference tables, glossary

**Technical Context**:
- Languages: Markdown/MDX, Python 3.10+, XML (URDF), Bash, YAML
- Dependencies: context7 MCP, Claude Code Task tool, Docusaurus, Git/GitHub MCP
- Storage: File-based Markdown chapters in `chapters/` directory
- Testing: Manual 11-item self-review checklist per chapter
- Performance: ≤90min per chapter, ≥95% context7 success rate, <90min student comprehension

**Risk Analysis**: 5 risks identified with mitigations (context7 unavailability, approval latency, word count overrun, glossary inconsistency, Task tool overhead)

**Estimated Timeline**: ~31.5 hours active work (14 chapters × 90min + 3 appendices × 90min + 6hr planning) spread across weeks/months due to approval gates

## Outcome

- ✅ Impact: Complete implementation plan with constitution compliance, workflow architecture, chapter sequence, risk analysis. Ready for `/sp.tasks` to generate task breakdown.
- 🧪 Tests: Constitution Check PASS (7/7 principles), no violations, no complexity justification required
- 📁 Files: Created plan.md (401 lines), research.md (comprehensive documentation source analysis with context7 library ID resolution, fallback URLs, tooling validation)
- 🔁 Next prompts: `/sp.tasks` (generate task breakdown for C1-C14 and appendices), then begin C1 generation per quickstart workflow
- 🧠 Reflection: Plan integrates user master execution plan with constitution principles. Sequential approval gates align with Principle III. context7 fallback strategy addresses R1 risk. Task delegation patterns formalized for math/code subagents. Phase 1 artifacts (data-model, contracts, quickstart) outline entity models and workflow schemas but require full generation in subsequent planning iteration.

## Evaluation notes (flywheel)

- Failure modes observed: setup-plan.sh warning about multiple '001' prefixes (001-constitutional-content-agent, 001-physical-ai-textbook) - expected due to parallel feature branches, not blocking
- Graders run and results (PASS/FAIL): Constitution Check PASS (all 7 principles compliant), no violations detected
- Prompt variant (if applicable): User-provided master execution plan with detailed GitHub MCP workflow, Task tool invocation patterns (<<call:...>> syntax from original constitution, adapted to Claude Code Task tool in plan.md)
- Next experiment (smallest change to try): Validate context7 library ID resolution during C1 generation (Test Case 1 from research.md) to confirm ≥95% success rate assumption before proceeding with full chapter sequence
