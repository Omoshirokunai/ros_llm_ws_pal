Abstract
This thesis investigates the application of Vision-Language Models (VLMs) for direct robot control through natural language instructions. We present a novel architecture that integrates LLaVA-13B with a mobile robot platform, implementing a closed-loop control system with continuous visual feedback and safety monitoring. While recent advances in VLMs have shown promising results in vision-language tasks, their application to physical robot control presents unique challenges. Through extensive experimental evaluation, we demonstrate that current VLM architectures, despite their sophisticated language understanding capabilities, face significant limitations in real-world robotics applications. Our results show a 0% task completion rate, 34.19% hallucination rate, and an average inference time of 13.66 seconds, highlighting fundamental challenges in grounding language commands to physical actions. This work provides critical insights into the limitations of direct VLM-based robot control and suggests necessary architectural modifications for practical robotics applications.

Chapter 4: System Design
4.1 Research Question

"How can Vision-Language Models be effectively integrated with a TIAGo robot platform to enable safe and reliable natural language navigation control with a success rate above 80% in structured indoor environments within a 6-month development period?"

4.1.1 Sub-questions
How can visual feedback be effectively incorporated into VLM-based control to ensure reliable task completion assessment?
What safety mechanisms are necessary to prevent hallucination-induced unsafe actions in real-world deployments?
How can prompt engineering strategies improve the grounding between natural language commands and robot actions?

4.2 System Requirements
ID	Requirement	Priority	Validation Method
R1	Integration with existing PAL Robotics ROS control stack	High	System testing
R2	Real-time visual feedback processing (<2s)	High	Performance testing
R3	Safety constraint validation for all commands	High	Safety testing
R4	Natural language instruction parsing	Medium	User testing
R5	TIAGo robot platform compatibility	High	Integration testing
R6	Command validity rate >80%	Medium	Metrics analysis
R7	Modular architecture design	Medium	Code review


4.3 System Architecture
4.3.1 Overview
The system follows a modular architecture with four primary components:

1 Task Planning Module
LLaVA-based subgoal generator
Natural language parsing
Task decomposition logic

2 Control Module
Action space mapping
Command validation
Parameter bounds checking
Robot control interface

3 Feedback Module
Visual state tracking
Progress evaluation
Recovery generation
Temporal context management
4 Safety Module
LiDAR-based obstacle detection
Trajectory validation
Emergency stop handling
Dynamic safety context

4.3.2 Data Flow
The system implements a closed-loop architecture where:

Natural language commands are decomposed into sequential subgoals
Each subgoal generates concrete robot actions through the control module
Continuous visual feedback evaluates progress
Safety constraints are validated in parallel
Recovery behaviors are triggered when needed

4.3.3 Integration Points
Key integration requirements with the TIAGo platform include:

ROS action server interface
Camera and LiDAR sensor access
Motion control primitives
Safety system hooks
State feedback channels

The implementation uses:

LLaVA-13B as the core vision-language model
ROS for robot interface
Flask for system coordination
OpenCV for image processing
Thread-safe command execution
This architecture prioritizes:

Modularity for testing
Safety by design
Real-time performance
Graceful degradation
Integration flexibility