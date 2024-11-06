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

# # Initialize global variables
# goal_setter = ollama.Ollama(model='llava-llama3')
# robot_control_model = ollama.Ollama(model='llava-llama3')
# feedback_model = ollama.Ollama(model='llava-llama3')

# # Define the goal setter function
# def generate_subgoals(prompt: str) -> list:
#     message = [
#         {"role": "system", "content": goal_setter_system_prompt},
#         {"role": "user", "content": prompt},
#     ]
#     try:
#         response = goal_setter.chat(messages=message, stream=False, options=generation_config)
#         if response and response['message']['content']:
#             subgoals = response['message']['content'].split('\n')
#             rich.print(f"[green]Goal Setter[green]: {subgoals}")
#             return subgoals
#     except Exception as e:
#         rich.print(f"[red]Error in generate_subgoals[red]: {e}")
#     return []

# # Define the robot control function
# def control_robot(subgoal: str, current_image: bytes, previous_image: bytes) -> str:
#     current_image_base64 = base64.b64encode(current_image).decode('utf-8')
#     previous_image_base64 = base64.b64encode(previous_image).decode('utf-8')
#     message = [
#         {"role": "system", "content": system_prompt},
#         {"role": "user", "content": previous_image_base64, "is_image": True},
#         {"role": "user", "content": current_image_base64, "is_image": True},
#         {"role": "user", "content": subgoal},
#     ]
#     try:
#         response = robot_control_model.chat(messages=message, stream=False, options=generation_config)
#         if response and response['message']['content']:
#             action = response['message']['content'].strip().lower()
#             rich.print(f"[blue]Robot Control[blue]: {action}")
#             return action
#     except Exception as e:
#         rich.print(f"[red]Error in control_robot[red]: {e}")
#     return "failed to understand"

# # Define the feedback function
# def get_feedback(current_image: bytes, previous_image: bytes) -> str:
#     current_image_base64 = base64.b64encode(current_image).decode('utf-8')
#     previous_image_base64 = base64.b64encode(previous_image).decode('utf-8')
#     message = [
#         {"role": "system", "content": verification_system_prompt},
#         {"role": "user", "content": previous_image_base64, "is_image": True},
#         {"role": "user", "content": current_image_base64, "is_image": True},
#     ]
#     try:
#         response = feedback_model.chat(messages=message, stream=False, options=generation_config)
#         if response and response['message']['content']:
#             feedback = response['message']['content'].strip().lower()
#             rich.print(f"[purple]Feedback[purple]: {feedback}")
#             return feedback
#     except Exception as e:
#         rich.print(f"[red]Error in get_feedback[red]: {e}")
#     return "failed to understand"


class LLMController:
    def __init__(self):
        self.model_name = 'llava-llama3'
        # self.goal_setter = ollama.Ollama(model='llava-llama3')
        # self.robot_control = ollama.Ollama(model='llava-llama3')
        # self.feedback = ollama.Ollama(model='llava-llama3')
        self.command_lock = threading.Lock()

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
                message = [
                    {"role": "system", "content": verification_system_prompt},
                    {"role": "user", "content": base64.b64encode(previous_image).decode('utf-8'), "is_image": True},
                    {"role": "user", "content": base64.b64encode(current_image).decode('utf-8'), "is_image": True},
                ]
                response = ollama.chat(
                    model=self.model_name,
                    messages=message,
                    stream=False,
                    options={
                    **generation_config,
                    "max_output_tokens": 10,  # Restrict to short responses
                    "temperature": 0.1,       # More deterministic
                }
                )
                if response and response['message']['content']:
                    return response['message']['content'].strip().lower()
            except Exception as e:
                rich.print(f"[red]Error in get_feedback[red]: {e}")
            return "failed to understand"