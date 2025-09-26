import os
from huggingface_hub import hf_hub_download
from ultralytics import YOLO
from supervision import Detections
from PIL import Image
import numpy as np
from deepface import DeepFace
from scipy.spatial.distance import cosine


def download_model(repo_id: str, filename: str) -> str:
    return hf_hub_download(repo_id=repo_id, filename=filename)


def detect_face(model, image: Image) -> Detections:
    output = model(image)
    return Detections.from_ultralytics(output[0])


def scale_bbox(x1: float, y1: float, x2: float, y2: float, scale_factor: float, image_width: int, image_height: int) -> tuple:
    dx = (x2 - x1) * scale_factor
    dy = (y2 - y1) * scale_factor

    x1_scaled = max(x1 - dx, 0)
    y1_scaled = max(y1 - dy, 0)
    x2_scaled = min(x2 + dx, image_width)
    y2_scaled = min(y2 + dy, image_height)

    return x1_scaled, y1_scaled, x2_scaled, y2_scaled


def process_detection(detection, image: Image, scale_factor: float = 0.5) -> Image:
    bbox = detection  
    x1, y1, x2, y2 = bbox
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

    x1_scaled, y1_scaled, x2_scaled, y2_scaled = scale_bbox(x1, y1, x2, y2, scale_factor, image.width, image.height)
    return image.crop((x1_scaled, y1_scaled, x2_scaled, y2_scaled))

def dump_image(cropped_face: Image, output_path: str) -> None:
    cropped_face.save(output_path)
    print(f"Image saved at {output_path}")


def get_face_embedding(image: Image) -> np.ndarray:
    temp_filename = "temp_face.jpg"
    image.save(temp_filename)
    embedding = DeepFace.represent(temp_filename, model_name="Facenet", enforce_detection=False)
    os.remove(temp_filename)
    return np.array(embedding[0]['embedding'])


def compare_embeddings(embedding1: np.ndarray, embedding2: np.ndarray) -> float:
    return 1-cosine(embedding1, embedding2)


def is_face_known(new_embedding: np.ndarray, database: list, threshold: float = 0.7) -> bool:
    for known_embedding in database:
        similarity = compare_embeddings(new_embedding, known_embedding)
        if similarity > threshold:
            return True
    return False


def process_image(image_path: str, model, output_folder: str, database: list) -> None:
    image = Image.open(image_path)
    base_filename = os.path.splitext(os.path.basename(image_path))[0]  

    # Detect faces in the image
    results = detect_face(model, image)

    embeddings = []
    output_paths = []

    for i, bbox in enumerate(results.xyxy):
        cropped_face = process_detection(bbox, image, scale_factor=0.5)
        output_face_path = os.path.join(output_folder, f"{base_filename}_face_{i+1}_scaled.jpg")
        dump_image(cropped_face, output_face_path)

        # Get the face embedding
        embedding = get_face_embedding(cropped_face)
        embeddings.append(embedding)
        output_paths.append(output_face_path)

        # Check if the face is already known
        if is_face_known(embedding, database):
            print(f"Face {i+1} in {image_path} is already in the database!")
        else:
            print(f"Face {i+1} in {image_path} is a new face. Adding to the database.")
            database.append(embedding)  # Add to the database

    # Optionally, compare embeddings within the current image
    if len(embeddings) > 1:
        for i in range(len(embeddings) - 1):
            for j in range(i + 1, len(embeddings)):
                similarity = compare_embeddings(embeddings[i], embeddings[j])
                print(f"Similarity between face {i+1} and face {j+1}: {similarity:.4f}")


def process_all_images(input_folder: str, output_folder: str, model, database: list) -> None:
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in os.listdir(input_folder):
        if filename.lower().endswith(".jpg"):
            image_path = os.path.join(input_folder, filename)
            print(f"Processing image: {image_path}")
            process_image(image_path, model, output_folder, database)


def main():
    input_folder = "src/img_input_example"
    output_folder = "src/out_folder"
    model_path = download_model("arnabdhar/YOLOv8-Face-Detection", "model.pt")

    model = YOLO(model_path)

    face_database = []

    process_all_images(input_folder, output_folder, model, face_database)


if __name__ == "__main__":
    main()
