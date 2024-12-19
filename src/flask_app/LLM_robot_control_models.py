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
        self.inital_scene_description = None
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

        description = self.get_scene_description_from_llm(image_path)
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

    def control_robot(self, subgoal: str, initial_image: str, current_image: str, previous_image: str, map_image:str, executed_actions: list = None, last_feedback:str = None, all_subgoals:list = []) -> str:
        self.current_subtask = subgoal
        scene_description = self.get_scene_description(current_image)

        with self.command_lock:
            try:

                system_prompt_ = f"""
    You are a robot controller that makes valid function calls to a robot.
    You will respond with exactly one of these command formats:

    Basic Commands:
    - move forward
    - turn left
    - turn right
    - completed

    Parameterized Commands:
    - move forward X meters at Y m/s (X: 0.1-2.0, Y: 0.1-0.5)
    - turn left X degrees at Y rad/s (X: 1-180, Y: 0.1-0.5)
    - turn right X degrees at Y rad/s (X: 1-180, Y: 0.1-0.5)


    Current task: {', '.join(all_subgoals)}.
    Environment: {scene_description}
    Previous actions: {executed_actions if executed_actions else 'No action'}
    Last feedback: {last_feedback if last_feedback else 'No feedback yet'}

    Rules:
    1. Response must match exactly one valid action format
    2. Parameters must be within safety limits
    3. Avoid obstacles shown in current environment
    4. Use parameterized commands for precise movements
    5. Use basic commands for simple adjustments
            """
                message = [
                    {"role": "system", "content": system_prompt_},
                {"role": "user", "content": "Initial state when task started:", 'images': [initial_image]},
                {"role": "user", "content": "Current environment shows this image:", 'images': [current_image]},
                {"role": "user", "content": f"my next control function = "},

                # {"role": "user", "content": "Initial state when task started:", 'images': [initial_image]},
                # {"role": "user", "content": "the intitial environment state vs Current environment shows this image:", 'images': [initial_image, current_image]},
                # # {"role": "user", "content": "image showing lidar map showing obstacles in the environment", 'images': [map_image]},

                # {"role": "user", "content": f"Based on all image and the feedback on my last action being to: {last_feedback if last_feedback else 'no feedback'},\n my next action = "},
                # {"role": "system", "content": system_prompt_},
            ]
                response = ollama.chat(
                    # model=self.model_name,
                    model= self.model_name,
                    messages=message,
                    stream=False,
                    options={
                    "top_p": 0.6,
                    "min_p": 0.3,
                    "top_k": 30,
                    "num_predict": 9,
                    "temperature": 0.5,

                }
                )
                if response and response['message']['content']:
                    response = response['message']['content'].strip().lower()
                    return response
                    # return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in control_robot[red]: {e}")
            return "failed to understand"

    # def validate_control_response(response):
    #     """Extended validation for parameterized commands"""
    #     try:
    #         # Parse more complex commands like:
    #         # "move forward 0.5 meters at 0.3 m/s"
    #         # "turn left 45 degrees at 0.2 rad/s"



    #         command_parts = response.lower().split()
    #         if len(command_parts) >= 4:
    #             action = " ".join(command_parts[0:2])
    #             value = float(command_parts[2])
    #             units = command_parts[3]

    #             if action == "move forward" and units == "meters":
    #                 return True, ("move_forward", value)
    #             elif (action == "turn left" or action == "turn right") and units == "degrees":
    #                 return True, ("turn", value if action == "turn left" else -value)

    #     except:
    #         pass

    #     return False, None
    # def get_feedback(self, current_image: bytes, previous_image: bytes) -> str:
    # def get_feedback(self, current_image: bytes, previous_image: bytes, current_subgoal: str, executed_actions: list, last_feedback: str = None) -> str:

    def get_feedback(self, initial_image: str, current_image: str, previous_image: str, map_image: str, current_subgoal: str, executed_actions: list, last_feedback: str = None, subgoals: list = None) -> str:
         # Get fresh images from remote

        print("getting feedback")
        # with self.command_lock:
        try:

            if self.initial_scene_description is None:
                self.initial_scene_description = self.cache_scene_description(initial_image)

            formatted_prompt = f"""
    I am a robot progress evaluator analyzing task completion through spatial and behavioral metrics.

    TASK CONTEXT:
    Main Goal: {self.current_goal}
    Subgoal Sequence: {' → '.join(subgoals)}
    Current Subtask: {current_subgoal}

    SPATIAL ANALYSIS:
    Initial Scene: {self.initial_scene_description}
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
    - continue (measurable progress detected but subtask incomplete)
    - subtask complete (current subtask success criteria met)
    - main goal complete (overall goal state achieved)
    - no progress (no measurable improvement toward goal)

    STRICTLY RESPOND WITH ONE OPTION ONLY
                    """
            message = [
            {"role": "system", "content": formatted_prompt},
            {"role": "user", "content": "Initial image before executing any action shows:", 'images': [initial_image]},
            {"role": "user", "content": "Previous scene image before the actionwas executed:", 'images': [previous_image]},
            {"role": "user", "content": "Current scene image after executing the action:", 'images': [current_image]},
            # {"role": "user", "content": f"Progress after completing: {executed_actions if executed_actions else 'No action'} "}

            # {"role": "user", "content": base64.b64encode(map_image).decode('utf-8'), "is_image": True},
            # {"role": "user", "content": "Map:"},
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
                "top_k": 10,
                "num_predict": 5,
                "max_tokens" : 10,
                "temperature": 0.4,
            }
            )

            #check if repsonse if valid else reprompt the model and ask for feedback again
            valid_responses = ["continue", "subtask complete", "main goal complete", "no progress"]
            if response and response['message']['content']:
                response = response['message']['content'].strip().lower()
                if response in valid_responses:
                    return response
                else:
                    response = ollama.chat(
                        messages=message + [{"role": "system", "content": f"{response} is not a valid response, please respond with one of the following: continue, subtask complete, main goal complete, no progress"}],
                            model= self.model_name,
                        stream=False,
                        options={
                        "top_p": 0.2,
                        "top_k": 10,
                        "max_output_tokens": 20,  # Restrict to short responses
                        "max_tokens" : 10,
                        "temperature": 0.3,
                    }
                    )

                    if response == 'Main goal complete':
                        self.clear_cache()
                        return response['message']['content'].strip().lower()
                return response['message']['content'].strip().lower()
        except Exception as e:
            rich.print(f"[red]Error in get_feedback[red]: {e}")
        return "failed to understand"

    def clear_cache(self):
        """Clear scene description cache"""
        self.scene_cache = {}
        self.initial_scene_description = None
        self.last_scene_description = None


