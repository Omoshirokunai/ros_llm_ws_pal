import google.generativeai as genai
from safe import GEMINI_API_KEY
import os

genai.configure(api_key=GEMINI_API_KEY)
# The Gemini 1.5 models are versatile and work with both text-only and multimodal prompts
model = genai.GenerativeModel('gemini-1.5-flash')

response = model.generate_content("Write a story about a magic backpack.")
print(response.text)
