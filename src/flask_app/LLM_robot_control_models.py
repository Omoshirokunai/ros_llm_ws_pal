import base64
import os
import threading
from typing import Optional

import ollama
import rich
from gemini_config import (
    generation_config,
    goal_setter_system_prompt,
    safety_settings,
    system_prompt,
    verification_system_prompt,
)


class LLMController:
    def __init__(self):
        self.model_name = 'llava-llama3'
        self.command_lock = threading.Lock()
        self.debug = True  # Enable debug logging
        self.current_goal = None
        self.current_subtask = None


    def get_map_context(self):
        """Get the current map as base64 string for LLM context"""
        map_path = os.path.join(os.path.dirname(__file__), 'maps/current_map.png')
        if os.path.exists(map_path):
            with open(map_path, 'rb') as f:
                map_bytes = f.read()
                return base64.b64encode(map_bytes).decode('utf-8')
        return None

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
                message = [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
                    {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                    {"role": "user", "content": subgoal},
                ]
                response = ollama.chat(
                    # model=self.model_name,
                    model='llava:13b',

                    messages=message,
                    stream=False,
                    options=generation_config
                )
                if response and response['message']['content']:
                    return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in control_robot[red]: {e}")
            return "failed to understand"

    def get_feedback(self, current_image: bytes, previous_image: bytes) -> str:
        with self.command_lock:
            try:

                map_context = self.get_map_context()

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