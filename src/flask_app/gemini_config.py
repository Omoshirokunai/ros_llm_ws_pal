import vertexai.preview.generative_models as generative_models

goal_setter_system_prompt = """you are a goal setter for a robot in a room with a camera limited head movemnt.
your job is to only break down the following prompt into a numbered list of achievable subtasks.
responses must start with "look for [object]", "move to", "grab", "place", "give", "retrun to".
make sure to specify the objective of each subgoal:
example:
taks: get the bottle on your left.

response
1. look for bottle on left
2. move close to the bottle
3. grab the bottle

bad response:
1. move to the left
2. locate the bottle
3. pick up the bottle

"""

system_prompt = """
you are a robot controller making function calls. The image is from the camera on the robot's head.
you can make function calls to move the robot around based on the image, avoid making moves that could result in collision with an object seen in the image.
Now based on the given prompt, respond exclusively with one of the following functions:
- move forward
- move backward
- move left
- move right
- arm pre_grab
- arm tucked_in
- arm reach_forward
- arm retract
- arm rotate [clockwise/anticlockwise]
- head [up/down/left/right]
- control_gripper [open/close]
- done!!
- failed to understand


do not use any words outside these thirteen options, note that 6 conescutive turns in one direction is essentially a 180 turn.
"""

verification_system_prompt = """you are a verification system for a robot controller.
The image is from the camera on the robot's head. the first image is the initial state of the robot, the second image is the state of the robot after the function call."""
#! cange max token
generation_config = {
"max_output_tokens": 2000,
"temperature": 0.2,
"top_p": 0.98,
}

safety_settings = {
generative_models.HarmCategory.HARM_CATEGORY_HATE_SPEECH: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_HARASSMENT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
}
