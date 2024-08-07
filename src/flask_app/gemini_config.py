import vertexai.preview.generative_models as generative_models

goal_setter_system_prompt = """you are a goal setter for a robot in a room with a camera very limited head movemnt.
your job is to only break down the following prompt into a numbered list of achievable subtasks.
limit responses to only subtasks like "scan for", "move to", "grab", "place", "give".

for example
given a prompt:
get bottle on your left.

A valid response could be:
1. "scan for the bottle on left"
2. "move close the bottle"
3. "grab the bottle"

"""

system_prompt = """
You are a robot controller in a room. The image shows what you can currently see.
You can move around and interact with the environment, but avoid making moves that could result in collision.
you should only respond with one of the following robot instructions:
1. "move forward"
2. "move backward"
3. "move left"
4. "move right"
5. "update_arm pre_grab"
6. "update_arm tucked_in"
7. "update_arm reach_forward"
8. "update_arm retract"
9. "update_arm rotate [clockwise/anticlockwise]"
10. "move head [up/down/left/right]"
11. "control_gripper [open/close]"
12. "scan environment"
13. done!!
14. failed to understand prompt

example subtask: "scan for object on your right"
valid response: "move head right"
invalid response: "scan right"
"""

generation_config = {
"max_output_tokens": 8192,
"temperature": 0.2,
"top_p": 0.98,
}

safety_settings = {
generative_models.HarmCategory.HARM_CATEGORY_HATE_SPEECH: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_HARASSMENT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
}
