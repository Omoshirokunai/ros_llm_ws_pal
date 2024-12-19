import base64
import os
import threading
from typing import Optional

import ollama
import rich

# from dotenv import load_dotenv
from gemini_config import (
    generation_config,
    goal_setter_system_prompt,
    safety_settings,
    system_prompt,
    verification_system_prompt,
)
from sensor_data import LOCAL_PATHS, fetch_images

# load_dotenv()

class LLMController:
    def __init__(self, simulation: bool = True):
        # self.model_name = 'llava:13b'
        self.sim = simulation
        # Select model based on simulation flag
        if simulation:
            self.model_name = "llava:7b"  # Simulation model
            rich.print("[blue]Using simulation model: llava:8b[/blue]")
        else:
            self.model_name = "llava:13b"  # Real robot model
            rich.print("[blue]Using real robot model: llava:13b[/blue]")


        self.scene_cache = {}
        # self.inital_scene_description = None
        self.initial_scene_description = None
        self.last_scene_description = None
        self.command_lock = threading.Lock()
        self.debug = True  # Enable debug logging
        self.current_goal = None
        self.current_subtask = None
        self.feedback_history = [] #track feedback


    def load_images(self):
        """Fetch and load all required images"""
        if not fetch_images():
            raise Exception("Failed to fetch required images")

        images = {}
        for img_type in ['current', 'previous', 'map']:
            try:
                with open(LOCAL_PATHS[img_type], 'rb') as f:
                    images[img_type] = f.read()
            except Exception as e:
                raise Exception(f"Failed to load {img_type} image: {e}")

        return images



    def cache_scene_description(self, image_path: str) -> str:
        """Get scene description with caching"""
        if image_path in self.scene_cache:
            return self.scene_cache[image_path]

        description = self.get_scene_description(image_path)
        self.scene_cache[image_path] = description
        return description

    def get_scene_description(self, image_path: str) -> str:
        """Generate semantic description of the current scene"""
        try:
            system_prompt = """ You are a robot's visual system analyzing scenes. Describe:

    1. Distances and Measurements:
    - Estimate distances to objects (in meters)
    - Identify clear paths and their approximate widths
    - Note spatial gaps between objects

    2. Spatial Layout:
    - Forward path analysis (clear/blocked/narrow)
    - Clear paths and their directions
    - Side clearances (left/right spaces)
    - Distances (near, medium, far) to visible objects
    - Obstacles and their positions (coordinates relative to robot)

    3. Navigation Details:
    - Available turning radius
    - Potential collision risks
    - Safe passage widths

    4. Critical Safety Information:
    - Minimum clearance to nearest obstacle
    - Blocked directions
    - Dynamic elements (if any)

    Format your response as:
    FORWARD: {status} ({distance}m clear path)
    LEFT: {clearance}m space
    RIGHT: {clearance}m space
    NEAREST_OBSTACLE: {direction} at {distance}m
    SAFE_ACTIONS: {list of possible safe movements}
            Be brief and precise.
            """
            system_prompt = """ You are a robot's visual system analyzing scenes. Describe:

    1. Distances and Measurements:
    - Estimate distances to objects (in meters)
    - Identify clear paths and their approximate widths
    - Note spatial gaps between objects

    2. Spatial Layout:
    - Forward path analysis (clear/blocked/narrow)
    - Clear paths and their directions
    - Side clearances (left/right spaces)
    - Distances (near, medium, far) to visible objects
    - Obstacles and their positions (coordinates relative to robot)

    3. Navigation Details:
    - Available turning radius
    - Potential collision risks
    - Safe passage widths

    4. Critical Safety Information:
    - Minimum clearance to nearest obstacle
    - Blocked directions
    - Dynamic elements (if any)

    Format your response as:
    FORWARD: {Yes/No} there is {an/no} obstacle
    LEFT: {Yes/No} there is {an/no} obstacle
    RIGHT: {Yes/No} there is {an/no} obstacle
    NEAREST_OBSTACLE: {direction}
    SAFE_ACTIONS: {suggested best actions to take to avoid obstacles}
            Be brief and precise.
            """
            message = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": "Analyze the spatial layout and safety considerations for robot navigation", 'images': [image_path]}
                # {"role": "user", "content": "What does the robot see in this image?", 'images': [image_path]},
                # {"role": "user", "content": base64.b64encode(image).decode('utf-8'), "is_image": True}
            ]

            response = ollama.chat(
                model= self.model_name,
                messages=message,
                stream=False,
                options={
                    "top_p": 0.2,
                    "top_k": 10,
                    'num_predict': 100,
                    "top_p": 0.2,
                    "top_k": 10,
                    'num_predict': 100,
                    "max_tokens": 40,
                    "temperature": 0.3,
                }
            )
            print(f"scene descriptor: {response['message']['content'].strip()}")
            return response['message']['content'].strip()
        except Exception as e:
            rich.print(f"[red]Scene description failed:[/red] {str(e)}")
            return "Scene description unavailable"

    def generate_subgoals(self, prompt: str, initial_image:str) -> Optional[list]:
        scene_description = self.get_scene_description(initial_image)

        self.current_goal = prompt
        with self.command_lock:

            try:
                message = [
                    {"role": "system", "content": goal_setter_system_prompt, 'images': [initial_image]},
                    {"role": "user", "content": prompt + f"\nBased on the image describe to be: {scene_description}"},
                ]
                response = ollama.chat(
                    model=self.model_name,
                    messages=message,
                    stream=False,
                    options={
                    "top_p": 0.2,
                    "top_k": 10,
                    "max_output_tokens": 10,  # Restrict to short responses
                    "max_tokens" : 10,
                    "temperature": 0.3,       # More deterministic
                }
                )
                if response and response['message']['content']:
                    subgoals = response['message']['content'].split('\n')
                    # return response['message']['content'].split('\n')
                    subgoals = [s.strip() for s in subgoals if s.strip()]
                if subgoals:
                    return subgoals
            except Exception as e:
                rich.print(f"[red]Error in generate_subgoals[red]: {e}")
            return None

    def control_robot(self, subgoal: str, initial_image: str, current_image: str, previous_image: str, map_image:str, executed_actions: list = None, last_feedback:str = None, all_subgoals:list = [], safety_warning:str = None, safety_context:dict = None) -> str:
        self.current_subtask = subgoal
        scene_description = self.get_scene_description(current_image)

        with self.command_lock:
            try:

                system_prompt_ = f"""
    You are a robot controller that makes valid function calls to a robot.
    You can only respond with exactly one of these command formats:

    Basic Commands:
    - move forward
    - turn left
    - turn right
    - Complete

    Parameterized Commands:
    - turn right X degrees at Y rad/s (X: 1-140, Y: 0.1-0.5)
    - move forward X meters at Y m/s (X: 0.1-2.0, Y: 0.1-0.5)
    - turn left X degrees at Y rad/s (X: 1-140, Y: 0.1-0.5)

    Example valid outputs:
    - turn left X degrees at Y rad/s (X: between 1 and 140 degrees, Y: between 0.1 and 0.5)
    - move forward X meters at Y m/s (X: between 0.1 and 2.0 meters, Y: between 0.1 and 0.5)
    - turn right [angle] degrees at [turn rate] rad/s

    Example invalid outputs:
    - based on the image provided, ...

    Current goal: {subgoal}
    Environment: {scene_description}
    Last feedback: {last_feedback if last_feedback else 'No feedback yet'}
    Safety Status: {safety_warning if safety_warning else 'No safety warnings'}

    Rules:
    1. Response must match exactly one valid action format
    2. Parameters must be within safety limits
    3. Avoid obstacles shown in current environment
    4. Use parameterized commands for precise movements
    5. Use basic commands for simple adjustments
    6. Avoid high risk directions
    7. Never explain or justify your decisions
    8. Output exactly ONE command with no additional text

            """
                message = [
                    {"role": "system", "content": system_prompt_},
                {"role": "user", "content": "Initial state when task started:", 'images': [initial_image]},
                {"role": "user", "content": "Current environment shows this image:", 'images': [current_image]},
                # {"role": "user", "content": "lidar map of the environment white circles showing obstacles:", 'images': [map_image]},
                {"role": "user", "content": f"Command: "},
            ]
                response = ollama.chat(
                    # model=self.model_name,
                    model= self.model_name,
                    messages=message,
                    stream=False,
                    options={
                    "top_p": 0.3,
                    "min_p": 0.1,
                    "top_k": 10,
                    "num_predict": 16,
                    "temperature": 0.3,

                }
                )

                if response and response['message']['content']:
                    response = response['message']['content'].strip().lower()
                    print(f"debug control response: {response}")
                    return response
                    # return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in control_robot[red]: {e}")
            return "failed to understand"

    def get_feedback(self, initial_image: str, current_image: str, previous_image: str, map_image: str, current_subgoal: str, executed_actions: list, last_feedback: str = None, subgoals: list = None,) -> str:
         # Get fresh images from remote

        print("getting feedback")
        # with self.command_lock:
        try:

            # if self.initial_scene_description is None:
            #     self.initial_scene_description = self.cache_scene_description(initial_image)

            formatted_prompt = f"""
    I am a robot progress evaluator analyzing task completion based on the changes in the images and the lidar map.

    TASK CONTEXT:
    Main Goal: {self.current_goal}
    Subgoal Sequence: {' → '.join(subgoals)}
    Current Subtask: {current_subgoal}

    SPATIAL ANALYSIS:
    Current Scene: {self.get_scene_description(current_image)}

    EXECUTION HISTORY:
    Actions: [{', '.join(executed_actions if executed_actions else 'None')}]
    Last Feedback: {last_feedback if last_feedback else 'No feedback'}


    EVALUATION CRITERIA:
    1. Spatial Progress:
       - Distance to goal change
       - Orientation alignment
       - Obstacle clearance

    2. Task Completion:
       - Subtask requirements met
       - Goal state alignment
       - Safety constraints maintained

    3. Action Efficiency:
       - Movement optimality
       - Command precision
       - Resource usage


    RESPONSE OPTIONS (select exactly one):
    - continue
    - subtask complete
    - main goal complete
    - no progress
    - adjust:<specific_suggestion>

    Where:
    - continue (progress detected, continue current approach)
    - subtask complete (current subtask requirements met)
    - main goal complete (overall goal achieved)
    - no progress (no improvement toward goal)
    - adjust:<specific_suggestion> (e.g. "adjust:try turning [suggested direction]" or "adjust:try going forward")


    Rules:
    - Output exactly ONE command with no additional text
    - Respond with single option. For option 5, include brief guidance after colon.
                    """

            message = [
            {"role": "system", "content": formatted_prompt},
            {"role": "user", "content": "Initial image before executing any action shows:", 'images': [initial_image]},
            {"role": "user", "content": "Previous scene image before the action was executed:", 'images': [previous_image]},
            {"role": "user", "content": "Current scene image after executing the action:", 'images': [current_image]},
            {"role": "user", "content": "Lidar map of the environment white circles showing obstacles:", 'images': [map_image]},
            {"role": "user", "content": "The valid feedback option is:"}
        ]

            if self.debug:
                rich.print("[yellow]Sending feedback request:[/yellow]")
                rich.print(f"Current goal: {self.current_goal}")
                rich.print(f"Subtask: {self.current_subtask}")
                rich.print("Images included: Previous and Current")
                # model=self.model_name,
            response = ollama.chat(
                messages=message,
                model= self.model_name,
                stream=False,
                options={
                "top_p": 0.3,
                    "min_p": 0.1,
                    "top_k": 10,
                    "num_predict": 16,
                    "temperature": 0.3,
            }
            )

            if response and response['message']['content']:
                    response = response['message']['content'].strip().lower()
                    print(f"debug feedback response: {response}")
                    return response
                    # return response['message']['content'].strip().lower()
            return response['message']['content'].strip().lower()
        except Exception as e:
            rich.print(f"[red]Error in get_feedback[red]: {e}")
            return "failed to understand"

    def clear_cache(self):
        """Clear scene description cache"""
        self.scene_cache = {}
        self.initial_scene_description = None
        self.last_scene_description = None



