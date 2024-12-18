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


        self.command_lock = threading.Lock()
        self.debug = True  # Enable debug logging
        self.current_goal = None
        self.current_subtask = None
        self.feedback_history = [] #track feedback


    # def get_map_context(self):
    #     print("Getting map context")
    #     """Get the current map as base64 string for LLM context"""
    #     # map_path = os.path.join(os.path.dirname(__file__), 'maps/map.jpg')
    #     map_path = os.getenv("LOCAL_MAP_PATH")
    #     if os.path.exists(map_path):
    #         with open(map_path, 'rb') as f:
    #             map_bytes = f.read()
    #             return base64.b64encode(map_bytes).decode('utf-8')
    #     return None

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


    def get_scene_description(self, image_path: str) -> str:
        """Generate semantic description of the current scene"""
        try:
            # system_prompt = """ You are a scene descriptor that provides a short useful description of a scene from a robot's perspective.
            # list objects seen in the image and their relative positions.
            # 1. Notable objects and their relative positions
            # 2. Open spaces and pathways
            # 4. Spatial relationships (left, right, front, behind)
            # give list of at least 4 objects that are visible in the image
            # Examples
            # - The scene shows a {object} on {object_location}
            # - The scene shows a {object} in front
            # - The scene partially shows a {object} on the {direction}
            # Be brief and precise.
            # """
            message = [
                {"role": "user", "content": "Give a short 40 word description of what is in this image", 'images': [image_path]},

                # {"role": "system", "content": system_prompt},
                # {"role": "user", "content": "What does the robot see in this image?", 'images': [image_path]},
                # {"role": "user", "content": base64.b64encode(image).decode('utf-8'), "is_image": True}
            ]

            response = ollama.chat(
                model= self.model_name,
                messages=message,
                stream=False,
                options={
                    **generation_config,
                    'num_predict': 40,
                    "max_output_tokens": 40,  # Restrict to short responses
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
        I am a robot controller that makes valid function calls to a robot.
        I will respond exclusively with one of the following control functions:
            - turn left
            - move forward
            - move backward
            - turn right
            - completed

        my goal is to achieve the following subgoals: {', '.join(all_subgoals)}.
        I am Currently trying to {self.current_goal}, my environment shows {scene_description}.

        My Previous actions: {', '.join(executed_actions) if executed_actions else 'None'}
        have resulted In Last feedback being: {last_feedback if last_feedback else 'No feedback yet'}

        The Rules are:
        1. Response must match exactly one valid action
        2. I will not provide any additional information or explanation
        3. I will avoid obstacles shown in the current environment image
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
                    "num_predict": 3,
                    "temperature": 0.5,
                    "mirostat": 1,
                    "mirostat_eta": 0.3,
                }
                )
                if response and response['message']['content']:
                    response = response['message']['content'].strip().lower()
                    return response
                    # return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in control_robot[red]: {e}")
            return "failed to understand"

    # def get_feedback(self, current_image: bytes, previous_image: bytes) -> str:
    # def get_feedback(self, current_image: bytes, previous_image: bytes, current_subgoal: str, executed_actions: list, last_feedback: str = None) -> str:
    def get_feedback(self, initial_image: str, current_image: str, previous_image: str, map_image: str, current_subgoal: str, executed_actions: list, last_feedback: str = None, subgoals: list = None) -> str:
         # Get fresh images from remote

        print("getting feedback")
        # with self.command_lock:
        try:
                # images = self.load_images()
                # Load map for context
                # with open(LOCAL_PATHS['map'], 'rb') as f:
                #     map_context = base64.b64encode(f.read()).decode('utf-8')

                formatted_prompt = f"""
                I am a robot progress evaluator that provides feedback on the robot's progress.
                Given the task of {self.current_goal} which will be achived by completing the following subgoals: {' → '.join(subgoals)}
                I will evaluate the robot's progress based on the images provided.
                The Current scene shows: {self.get_scene_description(current_image)}
                The Current Task is to {current_subgoal}.
                So far the robot has made the following moves: [{', '.join(executed_actions if executed_actions else 'None')}].

                I will Compare initial state and current state to determine and respond with only one of the following:
                    - continue (if some progress has been made but not complete)
                    - subtask complete (if the current subtask is complete)
                    - main goal complete (if the overall goal is achieved)
                    - no progress (if real progress has been made towards completing the goal)

                    I will not give any aditional information or explanation
                    and will RESPOND WITH EXACTLY ONE OPTION
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
                if response and response['message']['content']:\

                    response = response['message']['content'].strip().lower()
                    if response in valid_responses:
                        return response
                    else:
                        response = ollama.chat(
                            messages=message + [{"role": "system", "content": f"{response} is not a valid response, please respond with one of the following: continue, subtask complete, main goal complete, no progress"}],
                                model= self.model_name,
                            stream=False,
                            options={
                            **generation_config,
                            "num_predict": 5,
                            "max_output_tokens": 20,  # Restrict to short responses
                            "max_tokens" : 10,
                            "temperature": 0.2,
                        }
                        )
                    return response['message']['content'].strip().lower()
        except Exception as e:
            rich.print(f"[red]Error in get_feedback[red]: {e}")
        return "failed to understand"


