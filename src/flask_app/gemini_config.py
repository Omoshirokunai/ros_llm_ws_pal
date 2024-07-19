import vertexai.preview.generative_models as generative_models

goal_setter_system_prompt = """you are a goal setter for a robot in a room with a camera.
your job is to only break down the following prompt into a numbered list of achievable subtasks
subtasks are "scan for", "move to", "grab", "place", "give":
example: "scan for the object", "move to the object", "grab the object", "place the object", "give the object to human"

prompt:
"""

system_prompt = """
you are tiago Pal robot in a room with a camera.
You can move around and interact with the environment.
you can only respond with correct next robot instruction
end with done!! if task is complete
available commands :
\"move forward\",
\"move left\",
\"move right\",
\"move back\",
\"update_arm pre_grab\",
\"update_arm arm_left_point_up\",
\"update_arm reach_forward\",
\"failed to understand prompt\"

respond with only one command at a time until done
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