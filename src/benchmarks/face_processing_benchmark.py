"""
TESTS SCRIPTS FROM FACE_PROCESSING BY USING A SMALL SAMPLE OF IMAGES
"""

import os
import time
import csv
import numpy as np
from tqdm import tqdm
from PIL import Image
from ultralytics import YOLO
from deepface import DeepFace

from brain_test_scripts.test_facerecognition import download_model, detect_face, process_detection, \
                                                    compare_embeddings, is_face_known

EMBEDDING_MODELS = [
    "Facenet",
    "Facenet512",
    "ArcFace",
    "VGG-Face",
    "SFace",
    "GhostFaceNet",
]

# "overrride" of get_face_embedding for this benchmark
def get_embedding(image: Image.Image, model_name: str) -> np.ndarray:
    tmp = "temp_bench_face.jpg"
    image.save(tmp)
    result = DeepFace.represent(
        tmp, 
        model_name=model_name, 
        enforce_detection=False,
        detector_backend="skip"
    )
    os.remove(tmp)
    return np.array(result[0]["embedding"])


def run_benchmark(input_folder: str, output_tsv: str):
    """main logic of benchmark"""
    model_path = download_model("arnabdhar/YOLOv8-Face-Detection", "model.pt")
    model = YOLO(model_path)
    face_database = []

    rows = []

    tqdm_bar = tqdm(EMBEDDING_MODELS)
    for emb_name in tqdm_bar:   # for each model..
        tqdm_bar.set_description_str(f"Model: {emb_name}")
        print(f"\n embedder={emb_name}")
        face_database = [] 

        # full procedute for a single embedder model
        for filename in sorted(os.listdir(input_folder)):
            if not filename.lower().endswith((".jpg", ".jpeg", ".png")):
                continue

            image_path = os.path.join(input_folder, filename)
            image = Image.open(image_path)

            # detection
            t0 = time.time()
            results = detect_face(model, image)
            detection_time = time.time() - t0

            num_faces = len(results.xyxy)

            # embedding + matching stuff
            embeddings = []
            new_faces = 0
            known_faces = 0

            t1 = time.time()
            for bbox in results.xyxy:
                cropped = process_detection(bbox, image, scale_factor=0.5)
                emb = get_embedding(cropped, emb_name)
                embeddings.append(emb)

                if is_face_known(emb, face_database):
                    known_faces += 1
                else:
                    new_faces += 1
                    face_database.append(emb)
            embedding_time = time.time() - t1

            # avg intra-image similarity
            avg_similarity = 0.0
            pair_count = 0
            for i in range(len(embeddings) - 1):
                for j in range(i + 1, len(embeddings)):
                    avg_similarity += compare_embeddings(embeddings[i], embeddings[j])
                    pair_count += 1
            if pair_count > 0:
                avg_similarity /= pair_count

            rows.append({
                "embedder": emb_name,
                "image": filename,
                "width": image.width,
                "height": image.height,
                "num_faces": num_faces,
                "new_faces": new_faces,
                "known_faces": known_faces,
                "embedding_dim": len(embeddings[0]) if embeddings else 0,
                "avg_intra_similarity": round(avg_similarity, 4),
                "detection_time_s": round(detection_time, 4),
                "embedding_time_s": round(embedding_time, 4),
                "total_time_s": round(detection_time + embedding_time, 4),
                "db_size_after": len(face_database),
            })

            print(f"{filename}: {num_faces} faces, {new_faces} new, {known_faces} known")

    # write tsv out file to log resutls
    fieldnames = list(rows[0].keys()) if rows else []
    with open(output_tsv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter="\t")
        writer.writeheader()
        writer.writerows(rows)

    print(f"Benchmark results saved to {output_tsv}")


if __name__ == "__main__":
    FP_BENCHMARK_OUTPUT_FILE = "src/benchmarks/face_processing_benchmark.tsv"
    run_benchmark(
        input_folder="src/benchmarks/faces_images",
        output_tsv=FP_BENCHMARK_OUTPUT_FILE
    )