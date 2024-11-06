# Add to imports
import io

import torch
from PIL import Image
from transformers import DetrForObjectDetection, DetrImageProcessor


# Add to LLMController
def detect_objects(self, image_bytes):
    processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
    model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50")

    image = Image.open(io.BytesIO(image_bytes))
    inputs = processor(images=image, return_tensors="pt")
    outputs = model(**inputs)

    # Post-process object detection results
    target_sizes = torch.tensor([image.size[::-1]])
    results = processor.post_process_object_detection(outputs, target_sizes=target_sizes)[0]

    return [(model.config.id2label[label.item()], score.item())
            for label, score in zip(results["labels"], results["scores"])
            if score > 0.9]