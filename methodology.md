
5.1 Introduction

This chapter details the implementation of a vision-language model (VLM) based robot control system that enables natural language navigation instructions. The system integrates LLaVA, a multimodal large language model, with a real-world mobile robot platform and safety monitoring systems. The key innovation lies in the closed-loop control architecture that incorporates continuous visual feedback and safety constraints.

[REFERENCE NEEDED: Recent work on VLMs for robotics]

5.2 System Architecture
The system follows a modular architecture with four main components (Figure 1):

Task Planning Module (LLaVA-based subgoal generator)

The task planning module, built on LLaVA-13B, decomposes high-level navigation commands into sequential subgoals. This module employs a specialized prompting strategy that emphasizes spatial awareness and task decomposition. The prompt engineering focuses on:

Initial scene understanding
Task decomposition principles
Safety-aware planning
Sequential goal organization

Control Module (LLaVA-based action controller)

The control module translates subgoals into concrete robot actions. It implements:

A fixed action space consisting of basic movements (forward, turn left/right)
Parameterized commands with safety bounds (e.g., "move forward X meters at Y m/s")
Real-time validation of generated commands
Visual grounding through continuous scene analysis

Feedback Module (LLaVA-based progress evaluator)
A critical innovation in our architecture is the closed-loop feedback system that:

Maintains temporal context through image history (initial, previous, current frames)
Evaluates progress toward subgoal completion
Provides structured feedback categories: continue, adjust, complete, no progress
Enables dynamic recovery from failed actions

Safety Module (LiDAR-based collision prevention)

The safety system runs in parallel with other modules and provides:

Real-time LiDAR-based obstacle detection
Pre-execution trajectory validation
Emergency stop capabilities
Dynamic safety context generation for the control module

[INSERT FIGURE 1: System architecture diagram showing data flow between modules]

5.3 Test Environment Setup
5.3.1 Physical Environment
The real-world experiments were conducted in a classroom environment containing:

Multiple tables and chairs
Target objects (black bag)
Human occupants
Natural lighting conditions
[INSERT FIGURE 2: Photo of physical test environment]

5.3.2 Simulation Environment
A simplified simulation environment was created using ROS and Gazebo containing:

Tables with target objects (cans)
Walls and corridors
Simulated human agents
Controlled lighting conditions
[INSERT FIGURE 3: Screenshot of simulation environment]

5.4 Task Scenarios
Based on the experiment logs, the following navigation scenarios were tested:

Object-directed navigation:
"there is a black bag on the table in front of you, go to the bag and stop"

Constrained navigation:
"avoid the chairs to your left, there is a black bag on the table in front of you"
Human-centric navigation:
"go to the person sitting down"

5.5 System Components
5.5.1 LLaVA Model Integration
The system uses LLaVA-13B as the core vision-language model, implemented with:

Custom prompting strategies for different modules
Optimized inference settings for real-time control
Memory management for continuous operation
[REFERENCE NEEDED: LLaVA paper]

5.5.2 Feedback Loop Implementation
The feedback system (Figure 4) processes:

Initial scene image
Current scene image
Previous scene image
LiDAR-generated safety map
Execution history
[INSERT FIGURE 4: Feedback loop diagram]

5.5.3 Safety Monitoring
Safety features include:

Real-time LiDAR obstacle detection
Pre-execution trajectory validation
Emergency stop triggers
Recovery behavior generation
5.6 Data Collection
The system logs comprehensive execution data including:

Timestamped actions and outcomes
Visual scene snapshots
Safety trigger events
Model inference times
Task completion status
5.7 Performance Metrics
Key metrics tracked:

Task Completion Rate
Command Validity Rate (valid vs invalid commands)
Safety Intervention Rate
Model Inference Latency
Human Intervention Requirements
5.8 Implementation Challenges
Notable challenges addressed:

Real-time performance constraints
Vision-action grounding
Safety guarantee integration
Feedback loop stability
Error recovery strategies
[REFERENCE NEEDED: Related work on robot safety systems]

5.9 Summary
This implementation combines vision-language models, robotics, and safety systems in a novel architecture for natural language robot control. The system prioritizes safe operation while maintaining task flexibility through continuous visual feedback and adaptive control.

Suggested additional figures:

Safety system architecture diagram
Model prompting strategy visualization