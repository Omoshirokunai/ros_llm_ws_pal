import vertexai
import base64
from vertexai.generative_models import GenerativeModel, Part, Image
import vertexai.preview.generative_models as generative_models
from safe import PROJECT_ID, REGIONEU, CREDENTIALS
from IPython.display import display, Markdown 
import os
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS

vertexai.init(project=PROJECT_ID, location=REGIONEU)
image = Image.from_file("image.jpg")
def multiturn_generate_content(message="", image=None, generation_config=None, safety_settings=None):
  model = GenerativeModel(
    "gemini-1.5-flash-001",
    system_instruction=["""empty system prompt"""]
  )
  chat = model.start_chat()
  return chat.send_message(message, generation_config=generation_config, safety_settings=safety_settings)

generation_config = {
    "max_output_tokens": 8192,
    "temperature": 1,
    "top_p": 0.95,
}

safety_settings = {
    generative_models.HarmCategory.HARM_CATEGORY_HATE_SPEECH: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
    generative_models.HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: generative_models.HarmBlockThreshold.BLOCK_LOW_AND_ABOVE,
    generative_models.HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: generative_models.HarmBlockThreshold.BLOCK_LOW_AND_ABOVE,
    generative_models.HarmCategory.HARM_CATEGORY_HARASSMENT: generative_models.HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
}
prompt = "what is PI and why is it useful"
# generative_multimodal_model = GenerativeModel("gemini-1.5-pro")
# response = generative_multimodal_model.generate_content(prompt)
response = multiturn_generate_content(prompt, generation_config=generation_config, safety_settings=safety_settings)
print(response.candidates[0].text)