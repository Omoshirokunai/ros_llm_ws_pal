# VLMs for Robots *(Tiago Pal)*

[![YouTube Video](https://img.youtube.com/vi/oC8lpK0tnGA/0.jpg)](https://www.youtube.com/watch?v=oC8lpK0tnGA "Gemini, LLMs for Robots")

## Table of Contents

- [VLMs for Robots *(Tiago Pal)*](#vlms-for-robots-tiago-pal)
  - [Table of Contents](#table-of-contents)
  - [Problem Statement](#problem-statement)
  - [Rationale](#rationale)
  - [Methodology](#methodology)
    - [Current Progress](#current-progress)
  - [Evaluation](#evaluation)
    - [Evaluation Tasks](#evaluation-tasks)
      - [Object Location](#object-location)
      - [Navigation to Point](#navigation-to-point)
      - [Obstacle Avoidance](#obstacle-avoidance)
  - [Limitations \& Future Work](#limitations--future-work)
  - [Requirements](#requirements)
  - [Architecture](#architecture)
  - [Installation](#installation)
    - [ROS](#ros)
    - [Tiago and Simulation](#tiago-and-simulation)
    - [Ollama and LLaVA](#ollama-and-llava)
  - [Usage](#usage)
    - [Interacting with the Robot](#interacting-with-the-robot)

## Problem Statement

Given a high level textutal instruction to complete a task in the realworld, how can we get a robot to follow it us

## Rationale

This research explores the feasibility of using existing Vision-Language Models (VLMs) without fine-tuning for robot control in a zero-shot setting. The approach uses a hierarchical system of VLMs to break down high-level instructions into executable robot actions.

The key hypothesis was that pre-trained VLMs could:

1. Decompose complex tasks into logical subtasks
2. Generate appropriate control actions based on visual feedback
3. Evaluate task progress through visual comparison

This zero-shot approach aims to reduce the need for task-specific training while leveraging the general capabilities of foundation models.

## Methodology

The system employs three main components:

1. Subgoal Generator
   1. Takes high-level task description and initial scene image
   2. Outputs ordered sequence of subtasks
   3. Uses LLaVA model with task decomposition prompt
2. Control Model
   1. Receives current subtask and scene images
   2. Generates discrete robot actions (move forward, turn left/right)
   3. Considers:
      1. Initial vs current state comparison
      2. Previous actions and feedback
      3. Safety constraints
3. Feedback Model
   1. Compares initial, previous and current states
   2. Evaluates subtask completion
   3. Provides corrective suggestions
   4. Uses fixed set of feedback options

### Current Progress

The system can:

1. Generate subtasks from natural language commands
2. Execute basic navigation actions
3. Provide simple visual feedback
4. Maintain safety constraints

## Evaluation

the project will be evaluated on the robot's abaility to complete the following tasks:

### Evaluation Tasks

#### Object Location

prompt: find {object} in the center of a room

**Metrics**

- Success rate over 10 trials : determined by the number of successful completions (*sucessful completion is dertermined by the robot being able to locate the object in the center of the room*)
- Number of action reversals
- Distance traveled/optimal path ratio

#### Navigation to Point

prompt: move to the designated location where {object} is located

**Metrics**

- Final position error: determeined by the distance between the robot's final position and the target location
- Path smoothness (direction changes)
- Collision avoidance success rate : determined using number of robot actions that trigger the lidar collision avoidance system

#### Obstacle Avoidance

prompt: move to a {location} where you see a {object} while avoiding the chairs

Metrics:
Number of safety stops
Success rate in different configurations

## Limitations & Future Work

Key challenges identified:

1. Task Completion Detection
   - System struggles to reliably detect when subtasks are complete.
   - Adding scene description VLM to improve state understanding
   - Exploring whole-task context instead of sequential subtasks

2. Action Selection
   - Limited to 4 basic movements
   - No path planning capabilities
   - Feedback Quality

3. Binary completion assessment
   - Limited spatial reasoning
   - No quantitative progress metrics

## Requirements

1. Python 3.8
2. ROS Noetic
3. Tiago Pal robot
4. Simulation environment (gazebo)
5. Flask
6. Google Cloud credentials for Vertex AI
7. Ollama and LLaVA model

## Architecture

- User Interface: A web-based interface built with Flask to input commands.

- VLM Models:

- Robot Control: ROS-based control of the Tiago Pal robot, including movement, arm manipulation, and sensory feedback.

## Installation

### ROS

Install ROS Noetic on your system following the instructions from the official ROS website.

<https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS>

### Tiago and Simulation

Install the Tiago Pal robot simulation packages by following the instructions from the official ROS website:

<https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/Testing_simulation>

Launch the Tiago Pal simulation:

```sh
    roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true
```

<!-- ### Gemini

Install the Vertex AI Python SDK:

```sh
pip install google-cloud-aiplatform
```

Set up your Google Cloud credentials:

```sh
export GOOGLE_APPLICATION_CREDENTIALS=<path_to_your_credentials_file.json>
```

Initialize Vertex AI:

```python
import vertexai
vertexai.init(project="YOUR_PROJECT_ID", location="YOUR_REGION")
``` -->

### Ollama and LLaVA

To install ollama

```sh
curl -fsSL https://ollama.com/install.sh | sh
```

Install the necessary Python packages:

```sh

pip install ollama
```

This project uses only llava-llama3 *(but in theory you can swap in another capable VLM)* to install that version ~code for using gemini 1.5 as a vlm has been removed temporarily~:

```sh
ollama run llava-llama3
```

## Usage

Running the Simulation and Flask App
    Launch the ROS simulation:

```sh
cd ~/tiago_public_ws
source devel/setup.bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true
```

Start the Flask app:

```sh
export FLASK_APP=app.py
flask run
```

Open your web browser and navigate to <http://127.0.0.1:5000> to access the control interface.

### Interacting with the Robot

Use the web interface to input commands.
The Flask app will process these commands using Gemini, Ollama, and LLaVA.

The robot will execute the commands, providing feedback on each action.
