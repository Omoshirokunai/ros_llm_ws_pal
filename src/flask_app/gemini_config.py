import vertexai.preview.generative_models as generative_models

goal_setter_system_prompt = """you are a goal setter for a robot in a room with a camera very limited head movemnt.
your job is to only break down the following prompt into a numbered list of achievable subtasks.
limit responses to only subtasks like "look for", "move to", "grab", "place", "give".

for example
given a prompt:
get bottle on your left.

A valid response could be:
1. "look for the bottle on your left"
2. "move close the bottle"
3. "grab the bottle"

"""

system_prompt = """
You are a robot controller in a room, with the camera in your head, the image shows what you can see .
You can move around and interact with the environment, but avoid making moves that could result in collision.
based on the image, respond exclusivelywith one of the instructions in this list:
1. move forward
2. move backward
3. move left
4. move right
5. arm pre_grab
6. arm tucked_in
7. arm reach_forward
8. arm retract
9. arm rotate [clockwise/anticlockwise]
10. head [up/down/left/right]
11. control_gripper [open/close]
12. done!!
13. failed to understand prompt

do not use any words outside the 13 options in your vocabulary.
example subgoal:"turn your head to the left"
valid response: "head left"
invalid response: "turn head left"
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
