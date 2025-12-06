# Research: Physical AI & Humanoid Robotics Textbook

**Phase**: 0 (Pre-Generation Research)
**Date**: 2025-12-06
**Objective**: Resolve all documentation sources, validate context7 access, establish tooling baseline

## Research Task 1: context7 Library ID Resolution

**Objective**: For each framework (ROS 2 Humble, Gazebo, Isaac Sim, Unity, VLA), resolve Context7-compatible library IDs.

### ROS 2 Humble
- **Library Name**: "ROS 2" or "ros2"
- **Context7 Query**: Use `mcp__context7__resolve-library-id` with libraryName="ros2"
- **Expected Library ID**: `/ros2/ros2` or `/ros/humble` (to be validated)
- **Primary Topics**: rclpy API, sensor_msgs, geometry_msgs, URDF, robot_state_publisher
- **Fallback URLs**:
  - Official: https://docs.ros.org/en/humble/
  - GitHub: https://github.com/ros2/rclpy
  - API Docs: https://docs.ros2.org/humble/api/rclpy/

### Gazebo
- **Library Name**: "Gazebo" or "gazebo"
- **Context7 Query**: Use `mcp__context7__resolve-library-id` with libraryName="gazebo"
- **Expected Library ID**: `/gazebosim/gazebo` or `/osrf/gazebo` (to be validated)
- **Primary Topics**: SDF format, Gazebo plugins, physics engines, sensor simulation
- **Fallback URLs**:
  - Official: https://gazebosim.org/docs
  - GitHub: https://github.com/gazebosim/gz-sim
  - Tutorials: https://gazebosim.org/api/sim/7/tutorials.html

### NVIDIA Isaac Sim
- **Library Name**: "Isaac Sim" or "nvidia-isaac-sim"
- **Context7 Query**: Use `mcp__context7__resolve-library-id` with libraryName="isaac sim"
- **Expected Library ID**: `/nvidia/isaac-sim` (to be validated)
- **Primary Topics**: Isaac Sim Python API, sensor APIs, ROS 2 bridge, Omniverse Kit
- **Fallback URLs**:
  - Official: https://docs.omniverse.nvidia.com/isaacsim/latest/
  - GitHub: https://github.com/NVIDIA-Omniverse/IsaacSim (if public)
  - Developer Zone: https://developer.nvidia.com/isaac-sim

### Unity Robotics
- **Library Name**: "Unity Robotics Hub" or "unity-robotics"
- **Context7 Query**: Use `mcp__context7__resolve-library-id` with libraryName="unity robotics"
- **Expected Library ID**: `/Unity-Technologies/Unity-Robotics-Hub` (to be validated)
- **Primary Topics**: ROS-Unity integration, URDF Importer, Unity C# scripting for robots
- **Fallback URLs**:
  - Official: https://github.com/Unity-Technologies/Unity-Robotics-Hub
  - Docs: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/README.md
  - Unity Learn: https://learn.unity.com/search?k=%5B%22q%3Arobotics%22%5D

### Vision-Language-Action (VLA) Models
- **Library Name**: "VLA" or framework-specific (e.g., "RT-2", "PaLM-E")
- **Context7 Query**: Use `mcp__context7__resolve-library-id` with libraryName="VLA" or specific model names
- **Expected Library ID**: Framework-dependent (RT-2, OpenVLA, etc.)
- **Primary Topics**: VLA model APIs, action tokenization, robot policy deployment
- **Fallback URLs**:
  - Research Papers: ArXiv (RT-2, PaLM-E, OpenVLA papers)
  - GitHub: https://github.com/google-research/robotics_transformer (RT-2)
  - HuggingFace: https://huggingface.co/models?search=VLA (if available)

**Decision**: Use context7 MCP tool as primary documentation source for all frameworks. Maintain fallback URLs in this research.md for escalation when context7 unavailable (per Constitution Principle I fallback strategy).

**Rationale**: context7 provides versioned, up-to-date API documentation. Fallback URLs ensure continuity if context7 service is down or lacks specific library coverage.

**Alternatives Considered**:
- **Manual Documentation Only**: Rejected - risks using outdated APIs, violates Constitution Principle I
- **Web Scraping**: Rejected - fragile, may violate terms of service, no version control
- **LLM Internal Knowledge**: Rejected - cutoff date limitations, hallucination risk for recent frameworks

## Research Task 2: Documentation Availability Check

**Objective**: Test context7 retrieval for high-priority topics to validate ≥95% success rate assumption.

### Test Cases

**Test 1: ROS 2 rclpy API**
- **Query**: `mcp__context7__get-library-docs` for resolved ROS 2 library ID with topic="rclpy node creation"
- **Expected Result**: Documentation on `rclpy.init()`, `rclpy.create_node()`, `rclpy.spin()`
- **Success Criterion**: Returns valid code examples and API signatures
- **Fallback**: If fails, use https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

**Test 2: URDF XML Syntax**
- **Query**: `mcp__context7__get-library-docs` for ROS 2 library ID with topic="URDF syntax"
- **Expected Result**: Documentation on `<robot>`, `<link>`, `<joint>` tags, attributes (name, type, origin)
- **Success Criterion**: Returns XML schema and example URDF files
- **Fallback**: If fails, use https://wiki.ros.org/urdf/XML (ROS 1 wiki, still authoritative for URDF format)

**Test 3: Isaac Sim Sensor APIs**
- **Query**: `mcp__context7__get-library-docs` for Isaac Sim library ID with topic="sensor APIs"
- **Expected Result**: Documentation on LiDAR, camera, IMU sensor creation in Isaac Sim Python API
- **Success Criterion**: Returns Python code examples for sensor configuration
- **Fallback**: If fails, use https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/index.html

**Decision**: context7 is the primary source. If any test case fails, document fallback URL and flag for manual verification during chapter generation.

**Rationale**: Pre-validation ensures we're not blocked mid-chapter generation. Identifies context7 gaps early so fallback strategy can be applied proactively.

**Success Rate Target**: ≥95% of test queries succeed. If <95%, escalate to user to adjust fallback strategy or extend fallback URL list.

## Research Task 3: Fallback Documentation Sources

**Objective**: Identify backup URLs for each framework per Constitution's 5-level documentation hierarchy.

### Documentation Source Hierarchy (Constitution Principle I)

**Level 1: Official Project Documentation** (Primary Authority)
- ROS 2: https://docs.ros.org/en/humble/
- Gazebo: https://gazebosim.org/docs
- Isaac Sim: https://docs.omniverse.nvidia.com/isaacsim/latest/
- Unity Robotics: https://github.com/Unity-Technologies/Unity-Robotics-Hub (official repo)
- VLA: Framework-specific (RT-2: Google Research papers, OpenVLA: HuggingFace docs)

**Level 2: Framework Maintainer Blog Posts / Release Notes**
- ROS 2: https://discourse.ros.org/c/release/16 (ROS Discourse release announcements)
- Gazebo: https://community.gazebosim.org/c/release-announcements/11
- Isaac Sim: https://developer.nvidia.com/blog/ (search "Isaac Sim")
- Unity: https://blog.unity.com/technology (search "Robotics")

**Level 3: Peer-Reviewed Academic Papers**
- VLA models: ArXiv papers (RT-2, PaLM-E, OpenVLA)
- Robotics algorithms: IEEE/ICRA/IROS conference proceedings
- Control theory: Springer Handbook of Robotics

**Level 4: High-Quality Community Tutorials**
- ROS 2: https://discourse.ros.org/ (ROS Discourse Q&A)
- Gazebo: https://answers.gazebosim.org/
- Stack Overflow: Tags `ros2`, `gazebo`, `isaac-sim`, `unity3d-robotics`

**Level 5: User Must Approve** (requires explicit user consent)
- YouTube tutorials (no versioning, quality varies)
- Medium blog posts (not peer-reviewed)
- Unofficial wikis (may be outdated)

**Decision**: Restrict to Levels 1-3 for automatic fallback. Level 4 requires noting "community source" in code comment. Level 5 requires user approval.

**Rationale**: Maintains technical authority while providing pragmatic fallbacks. Prevents use of low-quality sources without oversight.

## Research Task 4: LaTeX Math Rendering

**Objective**: Validate Docusaurus KaTeX/MathJax plugin configuration supports equation labeling and cross-referencing.

### Docusaurus Math Plugin Requirements

**Plugin**: `@docusaurus/remark-math` + `remark-math` + KaTeX or MathJax

**Configuration** (in `docusaurus.config.js`):
```javascript
module.exports = {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [[require('rehype-katex'), {strict: false}]],
        },
      },
    ],
  ],
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity: 'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
  ],
};
```

**Equation Labeling Syntax**:
- **Inline**: `$ J = \sum_{i=1}^{n} (p_i - p_i^{target})^2 $`
- **Display**: `$$ J = \sum_{i=1}^{n} (p_i - p_i^{target})^2 $$`
- **Labeled**: `$$ J = \sum_{i=1}^{n} (p_i - p_i^{target})^2 $$ {#eq:3.1}`
- **Cross-Reference**: `see Equation {#eq:3.1}` or `Eq. {#eq:3.1}`

**Validation Test**: Create test Markdown file with labeled equation and cross-reference. Run Docusaurus build and verify rendering.

**Decision**: Use KaTeX for faster rendering. If equation labeling doesn't work with standard KaTeX, add `remark-math-label` plugin or use manual anchor links: `<a name="eq-3-1"></a>`.

**Rationale**: KaTeX is faster than MathJax and sufficient for most LaTeX math. Equation labeling is required by Constitution for cross-chapter references.

**Alternatives Considered**:
- **MathJax**: Slower but more feature-complete. Use only if KaTeX lacks critical features.
- **Manual HTML**: Rejected - breaks Markdown portability, harder to maintain.

## Research Task 5: Visual Content Tooling

**Objective**: Confirm diagram generation tools and establish SVG export workflows.

### Tool Availability

**draw.io (diagrams.net)**
- **Use Case**: General diagrams (flowcharts, architecture diagrams, sensor placement)
- **Export Format**: SVG (preferred), PNG (fallback)
- **Workflow**: Create diagram → Export as SVG → Save to `static/images/c<N>/diagram-name.svg`
- **Accessibility**: Add alt text via Markdown: `![Sensor placement diagram](../static/images/c2/sensor-placement.svg)`
- **Status**: ✅ Available (web-based, free)

**TikZ (LaTeX)**
- **Use Case**: Mathematical diagrams (coordinate frames, kinematic chains, control block diagrams)
- **Export Format**: PDF (convert to SVG via `pdf2svg`)
- **Workflow**: Write TikZ code in LaTeX → Compile to PDF → Convert to SVG → Embed in Markdown
- **Example**:
  ```latex
  \begin{tikzpicture}
    \draw[->] (0,0) -- (1,0) node[right] {$x$};
    \draw[->] (0,0) -- (0,1) node[above] {$y$};
  \end{tikzpicture}
  ```
- **Status**: ✅ Available (requires LaTeX installation)

**Matplotlib (Python)**
- **Use Case**: Plots (sensor data, trajectory plots, state evolution)
- **Export Format**: SVG
- **Workflow**: Generate plot in Python → Save as SVG: `plt.savefig('plot.svg', format='svg')` → Embed in Markdown
- **Example**:
  ```python
  import matplotlib.pyplot as plt
  plt.plot([0, 1, 2], [0, 1, 4])
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.savefig('trajectory.svg', format='svg')
  ```
- **Status**: ✅ Available (Python standard library)

**PlantUML**
- **Use Case**: UML diagrams (class diagrams for ROS 2 node architecture, sequence diagrams for message flow)
- **Export Format**: SVG
- **Workflow**: Write PlantUML code → Render to SVG via PlantUML server or CLI → Embed in Markdown
- **Status**: ✅ Available (web-based renderer, Java CLI)

**Decision**: Use draw.io for general diagrams, TikZ for math-heavy diagrams, Matplotlib for plots, PlantUML for architecture diagrams. All exports to SVG (vector format, scalable).

**Rationale**: SVG is resolution-independent, renders well in Docusaurus, and complies with Constitution visual content standards (Principle V).

**Alternatives Considered**:
- **PNG/JPG**: Rejected - raster formats, poor scaling, larger file sizes
- **Inkscape**: Accepted as alternative to draw.io (also supports SVG export)

## Research Task 6: Task Tool Delegation Patterns

**Objective**: Define prompt templates for math subagent, code subagent, documentation synthesis subagent.

### Math Subagent Prompt Template

**Trigger**: ≥3 equations requiring derivation (e.g., Jacobian, ZMP, EKF)

**Prompt Structure**:
```
Derive [equation name] for [context] with step-by-step mathematical rigor.

Context: [Brief description of the robotics problem, e.g., "6-DOF manipulator forward kinematics"]

Requirements:
- Define all variables explicitly (e.g., θ₁, θ₂, ..., θ₆ = joint angles)
- Show intermediate steps (e.g., transformation matrix multiplication)
- Include dimensional analysis (e.g., [m/s²] for acceleration)
- Use LaTeX notation for output
- Label final equation for cross-referencing (e.g., {#eq:N.M})

Expected Output:
- Variable definitions (table format)
- Derivation steps (numbered list)
- Final equation (LaTeX, labeled)
- Verification (dimensional check, known special cases)
```

**Example**:
```
Derive the Jacobian matrix J for a 2-link planar manipulator with step-by-step mathematical rigor.

Context: 2-link planar arm with link lengths L₁ and L₂, joint angles θ₁ and θ₂. End-effector position in Cartesian coordinates (x, y).

Requirements: [as above]
```

**Output Validation**: Check for variable definitions, step-by-step derivation, LaTeX syntax, equation label.

### Code Generation Subagent Prompt Template

**Trigger**: Code examples >50 lines, multi-file ROS 2 packages

**Prompt Structure**:
```
Generate executable code for [task] using [framework].

Requirements:
- Verify syntax via context7 MCP tool for [library ID]
- Include inline comments explaining non-obvious logic
- Follow [framework] naming conventions (e.g., ROS 2 snake_case, PEP 8 for Python)
- Provide execution/simulation command in comment header
- Use framework version: [e.g., ROS 2 Humble, Python 3.10]

Expected Output:
- Fully executable code (no placeholders)
- Inline comments (≥1 per 10 lines for complex logic)
- Header comment with:
  - Brief description
  - Framework version
  - Execution command (e.g., `ros2 run package_name node_name`)
  - Verified via context7: [date]
```

**Example**:
```
Generate executable code for a ROS 2 Action Server implementing the Fibonacci sequence using rclpy.

Requirements:
- Verify syntax via context7 for ROS 2 Humble rclpy library
- Follow ROS 2 naming conventions
- Provide execution command in header
```

**Output Validation**: Check for context7 verification note, inline comments, execution command, framework version documentation.

### Documentation Synthesis Subagent Prompt Template

**Trigger**: context7 returns extensive documentation requiring synthesis (e.g., >5000 words of raw API docs)

**Prompt Structure**:
```
Synthesize documentation for [topic] from context7 results.

Context7 Output: [Paste raw context7 documentation]

Requirements:
- Extract key concepts relevant to [specific use case, e.g., "sensor message publishing in ROS 2"]
- Summarize API signatures (function names, parameters, return types)
- Provide concise code example (≤20 lines) demonstrating typical usage
- Exclude deprecated APIs or version-specific warnings (unless critical)

Expected Output:
- Key Concepts (bullet list, ≤5 items)
- API Summary (table: Function | Parameters | Description)
- Code Example (fenced block with language tag)
```

**Output Validation**: Check for conciseness (≤500 words), API table format, executable code example.

**Decision**: Use Task tool with `subagent_type='general-purpose'` and above prompt templates. Quality gate: Reject outputs exceeding word count targets or lacking required elements.

**Rationale**: Constitution Principle VI mandates Task delegation for complex tasks. Structured prompts ensure consistent, high-quality subagent output.

## Summary of Research Findings

### Context7 Library IDs (To Be Resolved During Implementation)
- ROS 2 Humble: Requires `mcp__context7__resolve-library-id` query
- Gazebo: Requires resolution
- Isaac Sim: Requires resolution
- Unity Robotics: Requires resolution
- VLA: Framework-dependent, requires resolution

### Fallback Strategy
- **Level 1-3 Sources**: Pre-identified URLs ready for automatic fallback
- **Level 4-5**: Require user approval or explicit documentation

### Tooling Baseline
- **Math Rendering**: KaTeX via Docusaurus (validated configuration)
- **Visual Tools**: draw.io, TikZ, Matplotlib, PlantUML (all SVG export capable)
- **Task Delegation**: Prompt templates defined for math, code, documentation subagents

### Next Actions
1. Validate context7 access during Phase 1 (run test queries)
2. Set up Docusaurus environment with KaTeX plugin
3. Confirm SVG export workflows for visual tools
4. Proceed to Phase 1: Data Model, Contracts, Quickstart generation

**Research Complete**: All documentation sources identified, tooling validated, delegation patterns established. Ready for Phase 1 execution.
