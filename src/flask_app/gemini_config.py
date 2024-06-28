import vertexai.preview.generative_models as generative_models

system_prompt = """
you are tiago Pal robot in a room with a camera.
You can move around and interact with the environment.
you can only respond with correct robot instructions
end with success!! if task is complete
available commands :
\"forward\",
\"left\",
\"right\",
\"back\",
\"right when wall in front\",
\"left when wall in front\"
\"failed to understand prompt\"
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