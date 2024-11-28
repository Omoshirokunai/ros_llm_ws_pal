import vertexai.preview.generative_models as generative_models

goal_setter_system_prompt = """you are a goal setter for a robot in a room with a camera limited head movemnt.
your job is to only break down the following prompt into a numbered list of achievable subtasks.
responses must start with "look for [object] [object direction]", "move to", "retrun to".
make sure to specify the objective of each subgoal:
example:
taks: get the bottle on your left.

response
1. look for bottle on left
2. move close to the bottle

bad response:
1. move to the left
2. locate the bottle
3. pick up the bottle

Ensure subtasks are:
- Sequential and dependent
- Specific and actionable
- Within robot's capabilities
"""

system_prompt = """
you are a robot controller making function calls. The image is from the camera on the robot's head.
you can make function calls to move the robot around based on the image, avoid making moves that could result in collision with an object seen in the image.
Now based on the given prompt, respond exclusively with one of the following functions:
- move forward
- move backward
- move left
- move right
- head [up/down/left/right]
- control_gripper [open/close]
- done!!
- failed to understand

Rules:
- Use EXACTLY one of these commands
- Check image for obstacles before movement
- Ensure arm is tucked before base movement
- Consider camera view limitations
- Moving head will adjust the camera view
"""
# do not use any words outside these thirteen options, note that 6 conescutive turns in one direction is essentially a 180 turn.

verification_system_prompt = """You are analyzing two consecutive robot camera images to track task progress.

Current subtask: {subtask}

Compare the images and determine if the robot has:
1. Made progress but needs to continue
2. Completed the current subtask
3. Completed the main goal

Rules:
- Previous image shows the starting state
- Current image shows the result after an action
- Look for specific changes related to the subtask
- Consider object positions, robot position, and arm state

YOU MUST RESPOND WITH EXACTLY ONE OF THESE OPTIONS:
"continue" - if you see progress but subtask isn't complete
"subtask complete" - if current subtask is achieved
"main goal complete" - if overall goal is achieved
"no progress" - if no relevant changes detected

NO OTHER RESPONSES ARE ALLOWED."""

#! cange max token
generation_config = {
"max_output_tokens": 10,
"temperature": 0.1,
"top_p": 0.96,
}

safety_settings = {
generative_models.HarmCategory.HARM_CATEGORY_HATE_SPEECH: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
generative_models.HarmCategory.HARM_CATEGORY_HARASSMENT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
}
