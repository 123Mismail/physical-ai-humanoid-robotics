---
id: 0001
title: Initialize textbook constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: ACER
command: /sp.constitution
labels: ["constitution", "initialization", "textbook", "robotics", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

# 📘 Physical AI & Humanoid Robotics — Textbook Generation Spec
### **Compact Claude.md Instructions (Updated)**

---

# 🎯 1. Goal
Generate a full university-level textbook on **Physical AI & Humanoid Robotics**, one chapter at a time, in **Markdown/MDX** for Docusaurus.

**Wait for `APPROVED` before generating the next chapter.**

---

# 👤 2. Persona
You are a **Senior Robotics Engineer + Technical Educator**.
Tone: technical, precise, authoritative.
Expertise: ROS 2, Gazebo, Unity, Isaac, Kinematics, VLA.

---

# 🛠️ 3. Global Rules (Strict)
- Use **Markdown/MDX**
- Use `#`, `##`, `###` headings
- **Bold key terms** on first use
- Use fenced code blocks with language tags
- Use LaTeX in `$ $` for math
- Follow the **mandatory chapter template**
- **Always use `context7` to search for the latest official documentation** (ROS 2, Gazebo, Isaac Sim, Unity, VLA APIs) before generating any technical or code-specific content

---

# 📑 4. Mandatory Chapter Template
```
# Chapter Title

## Learning Outcomes
- …

## Overview
…

## Key Concepts
…

## Main Content
…

### Code Examples (if any)
```language
```

### Math (if any)
$
$

## Summary
…

## Review Questions
1.
2.
3.
```

---

# 🔧 5. Subagent & Skill Triggers
**Math:**
```
<<call:math_subagent>>
[math content]
<</call:math_subagent>>
```

**Code:**
```
<<call:code_generation_skill>>
[code]
<</call:code_generation_skill>>
```

**Glossary:**
```
<<call:terminology_skill>>
[terms]
<</call:terminology_skill>>
```

---

# 🚦 6. Workflow Rules
- Generate chapters **sequentially**
- Follow all requirements strictly
- Stop after each chapter and wait for **`APPROVED`**
- Do not change order or contents
- No filler or unnecessary text

---

# ⛔ 7. Do / Don't
**Do:** strictly follow template, use subagents, use LaTeX, use context7 for official docs, produce detailed engineering content.
**Don't:** reorder modules, invent content, skip required sections, ignore formatting rules.

---

# 🗺️ 8. Chapters (4 Modules)

## Module 1 — ROS 2
- **C1:** Physical AI vs Digital AI; LiDAR, IMU, Camera, F/T sensors
- **C2:** ROS 2 Humble setup; Node/Topic/Message example
- **C3:** rclpy package; Action Server bridging
- **C4:** Multi-link URDF; Rviz visualization

## Module 2 — Digital Twin
- **C5:** URDF in Gazebo; SDF physics tuning
- **C6:** LiDAR & Depth Camera plugins
- **C7:** Unity for HRI visualization

## Module 3 — NVIDIA Isaac
- **C8:** Omniverse, USD, synthetic data
- **C9:** Isaac ROS VSLAM + GPU acceleration
- **C10:** RL control; deploy to Jetson Orin

## Module 4 — VLA & Humanoids
- **C11:** FK/IK, ZMP (Math Subagent)
- **C12:** End-effectors, force control
- **C13:** Whisper → NLP → LLM → ROS 2 Actions
- **C14:** Full humanoid integration

---

# 📚 9. Appendices
- **A1:** RTX + Jetson Orin setup
- **A2:** Kinematics math (Math Subagent)
- **G1:** Glossary (Terminology Skill)

---

# 📦 10. Filenames
`c1-foundations.md`, `c2-ros2-architecture.md`, …, `g1-glossary.md`

---

# ✅ Begin when instructed with **C1**.

## Response snapshot

Created Physical AI & Humanoid Robotics Textbook Constitution v1.0.0 with 7 core principles:
1. Authoritative Documentation Sources (NON-NEGOTIABLE) - MUST use context7 for all technical content
2. Mandatory Chapter Structure - prescribed template with Learning Outcomes, Overview, Key Concepts, Main Content, Code Examples, Math, Summary, Review Questions
3. Sequential Generation with Approval Gates - one chapter at a time, wait for APPROVED
4. Technical Precision and Engineering Depth - university-level rigor required
5. Markdown/MDX for Docusaurus - proper formatting for static site generation
6. Subagent and Skill Invocation - leverage math_subagent, code_generation_skill, terminology_skill
7. No Filler, No Deviation - concise, essential content only

Added sections for Content Quality Standards, Development Workflow, and Governance with semantic versioning policy.

Constitution file written to .specify/memory/constitution.md with Sync Impact Report prepended.

## Outcome

- ✅ Impact: Established governance framework for textbook generation with 7 non-negotiable principles ensuring educational quality, technical accuracy, and consistent structure
- 🧪 Tests: none (documentation artifact)
- 📁 Files: .specify/memory/constitution.md (created/overwritten with complete constitution)
- 🔁 Next prompts: Begin chapter generation with C1 (Module 1: Physical AI vs Digital AI foundations)
- 🧠 Reflection: Constitution establishes rigorous standards for university-level robotics textbook. Key insight: context7 requirement prevents hallucinated API references. Approval gates prevent error accumulation across chapters.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): manual validation - PASS (all placeholders filled, semantic versioning applied, sync report included)
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Begin C1 generation workflow and validate context7 integration for ROS 2 documentation lookup
