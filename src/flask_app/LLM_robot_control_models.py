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


    def get_scene_description(self, image: bytes) -> str:
        """Generate semantic description of the current scene"""
        try:
            system_prompt = """Describe the scene from a robot's perspective. Focus on:
            1. Notable objects and their relative positions
            2. Open spaces and pathways
            4. Spatial relationships (left, right, front, behind)
            Be brief and precise.
            Examples
            - The scene shows a {object} on {object_location}
            - The scene shows a {object} in front
            - The scene partially shows a {object} on the {direction}
            """
            message = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": base64.b64encode(image).decode('utf-8'), "is_image": True}
            ]

            response = ollama.chat(
                model='llava:13b',
                messages=message,
                stream=False,
                options={
                    "max_tokens": 100,
                    "temperature": 0.2
                }
            )
            return response['message']['content'].strip()
        except Exception as e:
            rich.print(f"[red]Scene description failed:[/red] {str(e)}")
            return "Scene description unavailable"

    def generate_subgoals(self, prompt: str, initial_image:bytes) -> Optional[list]:
        scene_description = self.get_scene_description(initial_image)

        self.current_goal = prompt
        with self.command_lock:
            try:
                message = [
                    {"role": "system", "content": goal_setter_system_prompt},
                    {"role": "system", "content": "if task is already complete respond with 'complete'"},

                ]

                if initial_image:
                    message.extend([
                        {"role": "user", "content": base64.b64encode(initial_image).decode('utf-8'), "is_image": True},
                        #scene_description
                        {"role": "user", "content": f"Currently the robot sees: {scene_description}"},
                        {"role": "user", "content": f"Generate subgoals based on this initial state and the following goal:"}
                    ])

                message.append({"role": "user", "content": prompt})
                response = ollama.chat(
                    model=self.model_name,
                    messages=message,
                    stream=False,
                    options={
                    **generation_config,
                    "max_output_tokens": 20,  # Restrict to short responses
                    "max_tokens" : 10,
                    "temperature": 0.6,       # More deterministic
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

    def control_robot(self, subgoal: str, initial_image: bytes, current_image: bytes, previous_image: bytes, map_image:bytes, executed_actions: list = None, last_feedback:str = None, all_subgoals:list = []) -> str:
        self.current_subtask = subgoal
        scene_description = self.get_scene_description(current_image)

        with self.command_lock:
            try:
                system_prompt_ = f"""
            You are a robot controller tasked with completing: {self.current_goal}
            To achieve this, you must complete the follwoing: {', '.join(all_subgoals)}
            current scene shows: {scene_description}
            Current Subtask: {subgoal}
            Previous Actions: {', '.join(executed_actions[-5:]) if executed_actions else 'None'}
            Last Feedback: {last_feedback if last_feedback else 'No feedback yet'}

            Compare the initial state to current state and choose any ONE of these actions to achieve the subtask:
            - turn left
            - move forward
            - move backward
            - turn right

            Success Criteria:
            1. Progress toward goal location/object
            2. Maintaining safe distance from obstacles
            3. Efficient path selection
            4. Response to feedback suggestions

            RESPOND WITH EXACTLY ONE ACTION.
            """
                message = [
                {"role": "system", "content": system_prompt_},
                {"role": "user", "content": "Initial state when task started:"},
                {"role": "user", "content": base64.b64encode(initial_image).decode('utf-8'), "is_image": True},
                {"role": "user", "content": "Current state:"},
                {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                # {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
                # {"role": "user", "content": "Previous state:"},
                # {"role": "user", "content": base64.b64encode(map_image).decode('utf-8'), "is_image": True},
                # {"role": "user", "content": "Environment map:"},
                {"role": "user", "content": f"Based on all images and {last_feedback if last_feedback else 'no'} feedback, what action achieves {subgoal}?"}
            ]
                response = ollama.chat(
                    # model=self.model_name,
                    model='llava:13b',

                    messages=message,
                    stream=False,
                    options={
                    **generation_config,
                    "max_output_tokens": 20,  # Restrict to short responses
                    "max_tokens" : 10,
                    "temperature": 0.3,       # More deterministic
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
    def get_feedback(self, initial_image: bytes, current_image: bytes, previous_image: bytes, map_image: bytes, current_subgoal: str, executed_actions: list, last_feedback: str = None, subgoals: list = None) -> str:
         # Get fresh images from remote

        print("getting feedback")
        # with self.command_lock:
        try:
                # images = self.load_images()
                # Load map for context
                with open(LOCAL_PATHS['map'], 'rb') as f:
                    map_context = base64.b64encode(f.read()).decode('utf-8')

                formatted_prompt = f"""
                Give feedback on the robot's progress toward the main goal: {self.current_goal}
                    Goal: {self.current_goal}
                    Current Task: {current_subgoal}
                    Current scene shows: {self.get_scene_description(current_image)}
                    All Subtasks: {' → '.join(subgoals)}
                    Last Actions: {', '.join(executed_actions[-5:] if executed_actions else 'None')}
                    Previous Feedback: {last_feedback if last_feedback else 'None'}

                    Compare initial, previous and current states to determine:
                    - 'continue' - progress made but not subtask not complete
                    - 'subtask complete' - move to the next subtask
                    - 'main goal complete' - Overall objective achieved
                    - 'no progress' - No meaningful change
                    - 'do [specific action]' - Suggest better approach
                    - 'complete' - if all tasks are done

                    RESPOND WITH EXACTLY ONE OPTION
                    """
                message = [
            {"role": "system", "content": formatted_prompt},
            {"role": "user", "content": base64.b64encode(initial_image).decode('utf-8'), "is_image": True},
            {"role": "user", "content": "Initial state:"},
            {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
            {"role": "user", "content": "Current state:"},
            {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
            {"role": "user", "content": "Previous state:"},
            # {"role": "user", "content": base64.b64encode(map_image).decode('utf-8'), "is_image": True},
            # {"role": "user", "content": "Map:"},
            {"role": "user", "content": f"Evaluate progress after: {executed_actions[-1] if executed_actions else 'No action'}"}
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
                    "temperature": 0.3,       # More deterministic
                }
                )
                if response and response['message']['content']:
                    return response['message']['content'].strip().lower()
        except Exception as e:
            rich.print(f"[red]Error in get_feedback[red]: {e}")
        return "failed to understand"


