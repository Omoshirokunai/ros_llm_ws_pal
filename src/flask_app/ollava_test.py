import ollama

def llava_generate(prompt, system_prompt, image_path):
    try:
        with open(image_path, 'rb') as image_file:
            image_data = image_file.read()

        response = ollama.generate(
            model='llava',
            prompt=prompt,
            system=system_prompt,
            images=[image_data],
            stream=False
        )
        return response['response']
    except Exception as e:
        print(f"Error in llava_generate: {e}")
        return None

# Specify your inputs here
image_path = "/home/hameed/ros_llm_ws_pal/src/images/image.png"
prompt = "What do you see in this image?"
system_prompt = "You are a helpful AI assistant that can see images."

# Generate response
response = llava_generate(prompt, system_prompt, image_path)

# Print the response
if response:
    print("LLaVA Response:")
    print(response)
else:
    print("Failed to get a response from LLaVA.")

# from ollama import Client
# client = Client(host='http://localhost:11434')
# response = client.chat(model='llava', messages=[
#   {
#     'role': 'user',
#     'content': 'Why is the sky blue?',
#   },
# ])

# print(response)