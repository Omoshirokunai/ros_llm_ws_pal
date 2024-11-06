import base64
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


    def generate_subgoals(self, prompt: str) -> Optional[list]:
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
        with self.command_lock:
            try:
                message = [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
                    {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                    {"role": "user", "content": subgoal},
                ]
                response = ollama.chat(
                    model=self.model_name,
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
                 # Format system prompt with current subtask context
                formatted_prompt = f"""
                Task: {self.current_subtask}
                Compare the two images and tell me if the task is:
                - 'continue' (made progress but not complete)
                - 'subtask complete' (current task done)
                - 'main goal complete' (entire goal achieved)
                - 'no progress' (no changes detected)
                RESPOND WITH EXACTLY ONE OF THESE OPTIONS."""

                message = [
                    {
                        "role": "system",
                        "content": formatted_prompt
                    },
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image",
                                "data": base64.b64encode(previous_image).decode('utf-8')
                            },
                            {
                                "type": "text",
                                "data": "Previous image"
                            },
                            {
                                "type": "image",
                                "data": base64.b64encode(current_image).decode('utf-8')
                            },
                            {
                                "type": "text",
                                "data": "Current image - Has the robot completed the task?"
                            }
                        ]
                    }
                ]
                # Construct message with both images
                # message = [
                #     {"role": "system", "content": formatted_prompt},

                #     # {"role": "system", "content": verification_system_prompt},
                #     {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
                #     # {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                # ]
                if self.debug:
                    rich.print("[yellow]Sending feedback request:[/yellow]")
                    rich.print(f"Current goal: {self.current_goal}")
                    rich.print(f"Subtask: {self.current_subtask}")
                    rich.print("Images included: Previous and Current")
                response = ollama.chat(
                    model=self.model_name,
                    messages=message,
                    stream=False,
                    options={
                    **generation_config,
                    "max_output_tokens": 10,  # Restrict to short responses
                    "temperature": 0.1,       # More deterministic
                    'safety_settings': {
                    'use_safety_model': True,
                    'safety_model': 'openai/safetensors',
                    'safety_threshold': 0.5,
                    'safety_top_p': 0.95,
                    'safety_temperature': 0.7}
                }
                )
                if response and response['message']['content']:
                    return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in get_feedback[red]: {e}")
            return "failed to understand"