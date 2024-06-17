import vertexai
import base64
from vertexai.generative_models import GenerativeModel, Part, Image
import vertexai.preview.generative_models as generative_models
from safe import PROJECT_ID, REGIONEU, CREDENTIALS
from IPython.display import display, Markdown 
import os
# import joblib
from datetime import datetime

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS

vertexai.init(project=PROJECT_ID, location=REGIONEU)
image = Image.load_from_file("/home/hameed/ros_llm_ws_pal/src/images/image.png")

# load past chat history
# try:
#     history = joblib.load("history.txt")
# except:
#     history = []
    # pass
FILENAME = "history.txt"
MAX_FILE_SIZE_MB = 5

def load_history():
    if os.path.exists(FILENAME):
        with open(FILENAME, "r", encoding="utf-8") as file:
            history = [line.strip() for line in file.readlines()]
    else:
        history = []
    return history

def rotate_file_if_needed():
    if os.path.exists(FILENAME) and os.path.getsize(FILENAME) > MAX_FILE_SIZE_MB * 1024 * 1024:
        os.rename(FILENAME, FILENAME + ".backup")

def save_to_file(history):
    rotate_file_if_needed()
    with open(FILENAME, "w", encoding="utf-8") as file:
        for message in history:
            file.write(f"{message}\n")
def add_message(history, role, text):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    history.append(f"{timestamp} {role}: {text}")

def format_history_for_model(history):
    formatted_history = []
    for message in history:
        timestamp, role_text = message.split(" ", 1)
        role, text = role_text.split(": ", 1)
        formatted_history.append(generative_models.Content(role=role, parts=text))
    return formatted_history

def multiturn_generate_content(chat_history, system_prompt, message="", image=None, generation_config=None, safety_settings=None):
    model = GenerativeModel(
    "gemini-1.5-flash-001",
    system_instruction=[system_prompt]
    )
    formatted_history = format_history_for_model(chat_history)
    chat = model.start_chat(history=formatted_history)
    api_repsonse = chat.send_message([image,message], generation_config=generation_config, safety_settings=safety_settings)
    
    
    #update history.txt
    return api_repsonse, chat.history
    

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
system_prompt = """
you are a two wheeled robot the image is a picture of the maze you are in; your goal will be provided by the user.
you can only respond with an ordered list of robot commands X number of times to execute command you are not allowed to execute the same command in a row end with success!! if task is complete
available commands :
\"forward\",
\"left\",
\"right\",
\"back\",
\"right when wall in front\",
\"left when wall in front\"

example:
1.forward x 10
2.right when wall in front
3.forward x 13
end. sucess"""

import google.api_core.exceptions

prompt = "what are the updated command to get to the cables/switches?"
history = load_history()
try:
    # response = multiturn_generate_content(history, system_prompt,prompt, image, generation_config=generation_config, safety_settings=safety_settings)
    # print(response.candidates[0].text)
    response, updated_history = multiturn_generate_content(history, system_prompt, prompt, image, generation_config=generation_config, safety_settings=safety_settings)
    response_text = response.candidates[0].text
    add_message(history, "user", prompt)
    add_message(history, "gemini", response_text)
    save_to_file(history)
    print(response_text)
except google.api_core.exceptions.ResourceExhausted:
    print("ResourceExhausted")
except vertexai.generative_models._generative_models.ResponseValidationError:
    print("ResponseValidationError")

#print history
print(history)
#print history
print("\n".join(history))
