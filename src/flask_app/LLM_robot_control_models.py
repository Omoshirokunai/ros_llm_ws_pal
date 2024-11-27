import base64
import os
import threading
from typing import Optional

import ollama
import rich
from dotenv import load_dotenv
from gemini_config import (
    generation_config,
    goal_setter_system_prompt,
    safety_settings,
    system_prompt,
    verification_system_prompt,
)
from sensor_data import LOCAL_PATHS, fetch_images

load_dotenv()

class LLMController:
    def __init__(self):
        self.model_name = 'llava-llama3'
        self.command_lock = threading.Lock()
        self.debug = True  # Enable debug logging
        self.current_goal = None
        self.current_subtask = None


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

    def generate_subgoals(self, prompt: str) -> Optional[list]:
        self.current_goal = prompt
        with self.command_lock:
            try:
                message = [
                    {"role": "system", "content": goal_setter_system_prompt},
                    {"role": "user", "content": prompt},
                ]
                response = ollama.chat(
                    model=self.model_name,
                    messages=message,
                    stream=False,
                    options=generation_config
                )
                if response and response['message']['content']:
                    return response['message']['content'].split('\n')
            except Exception as e:
                rich.print(f"[red]Error in generate_subgoals[red]: {e}")
            return None

    def control_robot(self, subgoal: str, current_image: bytes, previous_image: bytes) -> str:
        self.current_subtask = subgoal

        with self.command_lock:
            try:
                system_prompt_ = f"""
                Your Goal: {self.current_goal}
                current Task: {self.current_subtask}
                in order to achive your goal and current task, would you like to:
                - move forward
                - move backward
                - move left
                - move right

                RESPOND WITH EXACTLY ONE OF THESE OPTIONS..
                """
                # Load map for context
                with open(LOCAL_PATHS['map'], 'rb') as f:
                    map_context = base64.b64encode(f.read()).decode('utf-8')

                message = [
                    {"role": "system", "content": system_prompt_},
                    {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                    {"role": "user", "content": "Above is what your camera can see. Here is a Lidar map of the Environment:"},
                    {"role": "user", "content": map_context, "is_image": True},
                    {"role": "user", "content": subgoal},
                ]
                response = ollama.chat(
                    # model=self.model_name,
                    model='llava:13b',

                    messages=message,
                    stream=False,
                    options=generation_config
                )
                valid_responses = [
                    "move forward",
                    "move backward",
                    "move left",
                    "move right",
                    "move head up",
                    "move head down",
                    "move head left",
                    "move head right",
                ]
                if response and response['message']['content']:
                    response = response['message']['content'].strip().lower()
                    return response
                    # return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in control_robot[red]: {e}")
            return "failed to understand"

    def get_feedback(self, current_image: bytes, previous_image: bytes) -> str:
         # Get fresh images from remote


        # with self.command_lock:
            try:
                images = self.load_images()
                # Load map for context
                with open(LOCAL_PATHS['map'], 'rb') as f:
                    map_context = base64.b64encode(f.read()).decode('utf-8')

                # map_context = self.get_map_context()

                 # Format system prompt with current subtask context
                formatted_prompt = f"""
                Task: {self.current_subtask}
                Compare the two images and tell me if the task is:
                - 'continue' (made progress but not complete)
                - 'subtask complete' (current task done)
                - 'main goal complete' (entire goal achieved)
                - 'no progress' (no changes detected)
                RESPOND WITH EXACTLY ONE OF THESE OPTIONS."""
                 # Correct message format for ollama
                message = [
                {"role": "system", "content": formatted_prompt},
                {"role": "user", "content": map_context, "is_image": True},
                {"role": "user", "content": "Environment map shown above. Here's previous image:"},
                {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
                {"role": "user", "content": "Previous image shown above. Here's current image:"},
                {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                {"role": "user", "content": "Based on these images, what's the task status?"}
            ]

                if self.debug:
                    rich.print("[yellow]Sending feedback request:[/yellow]")
                    rich.print(f"Current goal: {self.current_goal}")
                    rich.print(f"Subtask: {self.current_subtask}")
                    rich.print("Images included: Previous and Current")
                    # model=self.model_name,
                response = ollama.chat(
                    messages=message,
                    model='llava:13b',
                    stream=False,
                    options={
                    **generation_config,
                    "max_output_tokens": 20,  # Restrict to short responses
                    "max_tokens" : 10,
                    "temperature": 0.2,       # More deterministic
                }
                )
                if response and response['message']['content']:
                    return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in get_feedback[red]: {e}")
            return "failed to understand"