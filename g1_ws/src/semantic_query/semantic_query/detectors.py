"""Detector backends for semantic_query.

`Detector.detect(image_bgr, query_text)` returns the best `Detection` for the query, or None.

Two backends:
  * MockDetector — returns a centred box, no model. For offline pipeline testing.
  * GroundingDinoSam2Detector — Grounding DINO (open-vocab 2D box, via HF transformers) + optional
    SAM2 mask. Heavy imports are done lazily inside load(), so this module imports fine on a
    machine with no torch; the node can then fall back to the mock.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Detection:
    label: str
    score: float
    box_xyxy: tuple           # (x0, y0, x1, y1) in pixels
    mask: Optional[np.ndarray] = None   # HxW bool, or None


class Detector:
    def load(self):
        return self

    def detect(self, image_bgr, query_text) -> Optional[Detection]:
        raise NotImplementedError


class MockDetector(Detector):
    """No model: return a box in the central 40% of the frame, labelled with the query."""

    def detect(self, image_bgr, query_text) -> Optional[Detection]:
        h, w = image_bgr.shape[:2]
        x0, y0 = int(0.30 * w), int(0.30 * h)
        x1, y1 = int(0.70 * w), int(0.70 * h)
        return Detection(label=query_text or "object", score=0.50, box_xyxy=(x0, y0, x1, y1))


class GroundingDinoSam2Detector(Detector):
    def __init__(self, device="cuda", box_threshold=0.35, text_threshold=0.25,
                 use_sam2=True, gdino_weights="", sam2_weights=""):
        self.device = device
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.use_sam2 = use_sam2
        self.gdino_id = gdino_weights or "IDEA-Research/grounding-dino-tiny"
        self.sam2_id = sam2_weights or "facebook/sam2-hiera-small"
        self._torch = None
        self._proc = None
        self._gdino = None
        self._sam2 = None

    def load(self):
        import torch  # noqa: F401  (raises if unavailable -> node falls back to mock)
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        self._torch = torch
        self._proc = AutoProcessor.from_pretrained(self.gdino_id)
        self._gdino = AutoModelForZeroShotObjectDetection.from_pretrained(
            self.gdino_id).to(self.device).eval()
        if self.use_sam2:
            try:
                from sam2.sam2_image_predictor import SAM2ImagePredictor
                self._sam2 = SAM2ImagePredictor.from_pretrained(self.sam2_id, device=self.device)
            except Exception:
                self._sam2 = None   # box-only fallback
        return self

    def detect(self, image_bgr, query_text) -> Optional[Detection]:
        import cv2
        from PIL import Image

        torch = self._torch
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(rgb)
        # Grounding DINO wants lowercase, period-terminated phrases.
        text = query_text.strip().lower()
        if not text.endswith("."):
            text += "."

        inputs = self._proc(images=pil, text=text, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self._gdino(**inputs)
        results = self._proc.post_process_grounded_object_detection(
            outputs, inputs.input_ids,
            box_threshold=self.box_threshold, text_threshold=self.text_threshold,
            target_sizes=[pil.size[::-1]],
        )[0]

        scores = results["scores"]
        if len(scores) == 0:
            return None
        best = int(scores.argmax())
        box = [float(v) for v in results["boxes"][best].tolist()]  # xyxy pixels
        score = float(scores[best])
        label = str(results.get("labels", results.get("text_labels", [query_text]))[best]) \
            if results.get("labels") or results.get("text_labels") else query_text

        mask = None
        if self._sam2 is not None:
            try:
                self._sam2.set_image(rgb)
                masks, _, _ = self._sam2.predict(box=np.array(box)[None, :], multimask_output=False)
                mask = np.asarray(masks[0]).astype(bool)
            except Exception:
                mask = None
        return Detection(label=label or query_text, score=score,
                         box_xyxy=tuple(box), mask=mask)


def make_detector(backend, **kwargs) -> Detector:
    if backend == "mock":
        return MockDetector()
    if backend == "grounding_dino_sam2":
        return GroundingDinoSam2Detector(**kwargs)
    raise ValueError(f"unknown backend '{backend}'")
