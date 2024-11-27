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

    def control_robot(self, subgoal: str, current_image: bytes, previous_image: bytes, executed_actions: list = None, last_feedback:str = None ) -> str:
        self.current_subtask = subgoal

        with self.command_lock:
            try:
                 # Enhanced system prompt with feedback context
                system_prompt_ = f"""
                You are a robot controller tasked with making valid function calls to a robot. You are to guide the robot to complete a series of subtasks that will lead to the completion of a main goal.
                Your Main Goal: {self.current_goal}
                Current Task: {subgoal}
                Previous Actions: {', '.join(executed_actions) if executed_actions else 'None'}
                Last Feedback: {last_feedback if last_feedback else 'No feedback yet'}

                Based on the previous actions and feedback, choose ONE action:
                - move forward
                - move backward
                - move left
                - move right

                Consider:
                1. Previous actions taken: {executed_actions}
                2. Last feedback received: {last_feedback}
                3. Current camera view
                4. Lidar environment map where the red dot represents the robot and white lines represent obstacles
                5. Progress needed for current subtask

                RESPOND WITH EXACTLY ONE OF THESE OPTIONS.
                """
                # Load map for context
                with open(LOCAL_PATHS['map'], 'rb') as f:
                    map_context = base64.b64encode(f.read()).decode('utf-8')

                message = [
                    {"role": "system", "content": system_prompt_},
                    {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                    {"role": "user", "content": "Current camera view above. Here is a Lidar map of the Environment:"},
                    {"role": "user", "content": map_context, "is_image": True},
                    {"role": "user", "content": f"Based on the previous actions ({', '.join(executed_actions) if executed_actions else 'None'}) and feedback ({last_feedback if last_feedback else 'None'}), what action should be taken next to achieve: {subgoal}?"}
                    # {"role": "user", "content": subgoal},
                ]
                response = ollama.chat(
                    # model=self.model_name,
                    model='llava:13b',

                    messages=message,
                    stream=False,
                    options=generation_config
                )
                # valid_responses = [
                #     "move forward",
                #     "move backward",
                #     "move left",
                #     "move right",
                #     "move head up",
                #     "move head down",
                #     "move head left",
                #     "move head right",
                # ]
                if response and response['message']['content']:
                    response = response['message']['content'].strip().lower()
                    return response
                    # return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in control_robot[red]: {e}")
            return "failed to understand"

    # def get_feedback(self, current_image: bytes, previous_image: bytes) -> str:
    def get_feedback(self, current_image: bytes, previous_image: bytes, current_subgoal: str, executed_actions: list) -> str:
         # Get fresh images from remote


        # with self.command_lock:
            try:
                images = self.load_images()
                # Load map for context
                with open(LOCAL_PATHS['map'], 'rb') as f:
                    map_context = base64.b64encode(f.read()).decode('utf-8')

                # map_context = self.get_map_context()

                 # Format system prompt with current subtask context
                # new feedback system prompt with action history
                formatted_prompt = f"""
                    Goal: {self.current_goal}
                    Current Task: {current_subgoal}
                    Actions executed so far: {', '.join(executed_actions)}

                    Compare the two images and determine the task status:
                    - 'continue' (made progress but not complete)
                    - 'subtask complete' (current task done)
                    - 'main goal complete' (entire goal achieved)
                    - 'no progress' (no changes detected)

                    Consider:
                    1. The sequence of actions taken so far: {executed_actions}
                    2. Visual changes between images
                    3. Progress toward the current subtask based on the images
                    4. Overall goal completion

                    RESPOND WITH EXACTLY ONE OF THESE OPTIONS.
                    """
                 # Correct message format for ollama
                message = [
                {"role": "system", "content": formatted_prompt},
                {"role": "user", "content": map_context, "is_image": True},
                {"role": "user", "content": "Environment lidar map shown above. Here's  the camera image before the last action was taken:"},
                {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
                {"role": "user", "content": "Here's the current image after the action was taken:"},
                {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                {"role": "user", "content": f"What is the task status after the action: {executed_actions[-1] if executed_actions else 'None'}?"}
                # {"role": "user", "content": "Based on these images and given , what's the task status?"}
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