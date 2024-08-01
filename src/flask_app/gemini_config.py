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
you are a robot controller in a room the image is what you can see currently.
You can move around and interact with the environment.
you can only respond with valid robot instructions
the only available commands you can respond with are:
\"move forward\",
\"move left\",
\"move right\",
\"move backward\",
\"update_arm pre_grab\",
\"update_arm arm_left_point_up\",
\"update_arm reach_forward\",
\"failed to understand prompt\"

end with "done!!" if task is complete
respond with only one command at a time until done and make sure to avoid making moves that will result in collision
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
