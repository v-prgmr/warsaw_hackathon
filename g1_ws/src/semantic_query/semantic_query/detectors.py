"""Detector backends for semantic_query.

`Detector.detect(image_bgr, query_text)` returns the best `Detection` for the query, or None.

Two backends:
  * MockDetector — returns a centred box, no model. For offline pipeline testing.
  * GroundingDinoSam2Detector — Grounding DINO (open-vocabulary 2D box) + SAM2 (mask), both via
    Hugging Face ``transformers`` (SAM2: ``Sam2Model``, transformers >= 4.56; the
    ``facebookresearch/sam2`` package is used as a fallback if installed). Heavy imports are done
    lazily inside load(), so this module imports fine on a machine without torch; the node can
    then fall back to the mock. Runs on the GPU when available (``device: auto``).
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
    name = "detector"

    def load(self):
        return self

    def detect(self, image_bgr, query_text) -> Optional[Detection]:
        raise NotImplementedError


class MockDetector(Detector):
    """No model: return a box in the central 40% of the frame, labelled with the query."""
    name = "mock"

    def detect(self, image_bgr, query_text) -> Optional[Detection]:
        h, w = image_bgr.shape[:2]
        x0, y0 = int(0.30 * w), int(0.30 * h)
        x1, y1 = int(0.70 * w), int(0.70 * h)
        return Detection(label=query_text or "object", score=0.50, box_xyxy=(x0, y0, x1, y1))


class GroundingDinoSam2Detector(Detector):
    name = "grounding_dino_sam2"

    def __init__(self, device="auto", box_threshold=0.35, text_threshold=0.25,
                 use_sam2=True, gdino_weights="", sam2_weights=""):
        self.device = device
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.use_sam2 = use_sam2
        self.gdino_id = gdino_weights or "IDEA-Research/grounding-dino-tiny"
        self.sam2_id = sam2_weights or "facebook/sam2.1-hiera-small"
        self._torch = None
        self._proc = None
        self._gdino = None
        self._sam2 = None          # ("hf", processor, model) | ("sam2", predictor)
        self.sam2_error = None

    def load(self):
        import torch  # noqa: F401  (raises if unavailable -> node falls back to mock)
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        self._torch = torch
        if self.device in ("", "auto"):
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self._proc = AutoProcessor.from_pretrained(self.gdino_id)
        self._gdino = AutoModelForZeroShotObjectDetection.from_pretrained(
            self.gdino_id).to(self.device).eval()
        if self.use_sam2:
            try:
                from transformers import Sam2Model, Sam2Processor
                self._sam2 = ("hf", Sam2Processor.from_pretrained(self.sam2_id),
                              Sam2Model.from_pretrained(self.sam2_id).to(self.device).eval())
            except Exception as exc:  # noqa: BLE001
                try:
                    from sam2.sam2_image_predictor import SAM2ImagePredictor
                    self._sam2 = ("sam2", SAM2ImagePredictor.from_pretrained(
                        self.sam2_id, device=self.device))
                except Exception:  # noqa: BLE001
                    self._sam2, self.sam2_error = None, str(exc)   # box-only fallback
        return self

    def _postprocess(self, outputs, input_ids, size):
        post = self._proc.post_process_grounded_object_detection
        for kwargs in ({"threshold": self.box_threshold},        # transformers >= 4.51
                       {"box_threshold": self.box_threshold}):   # older
            try:
                return post(outputs, input_ids, text_threshold=self.text_threshold,
                            target_sizes=[size], **kwargs)[0]
            except TypeError:
                continue
        raise RuntimeError("unsupported transformers post_process_grounded_object_detection")

    def _mask(self, rgb, box):
        if self._sam2 is None:
            return None
        torch = self._torch
        try:
            if self._sam2[0] == "hf":
                _, proc, model = self._sam2
                inputs = proc(images=rgb, input_boxes=[[box]], return_tensors="pt").to(self.device)
                with torch.no_grad():
                    out = model(**inputs, multimask_output=False)
                masks = proc.post_process_masks(out.pred_masks.cpu(), inputs["original_sizes"])[0]
                return np.asarray(masks).reshape(-1, *rgb.shape[:2])[0].astype(bool)
            predictor = self._sam2[1]
            predictor.set_image(rgb)
            masks, _, _ = predictor.predict(box=np.array(box)[None, :], multimask_output=False)
            return np.asarray(masks[0]).astype(bool)
        except Exception as exc:  # noqa: BLE001
            self.sam2_error = str(exc)
            return None

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
        results = self._postprocess(outputs, inputs.input_ids, pil.size[::-1])

        scores = results["scores"]
        if len(scores) == 0:
            return None
        best = int(scores.argmax())
        box = [float(v) for v in results["boxes"][best].tolist()]  # xyxy pixels
        score = float(scores[best])
        labels = results.get("text_labels") or results.get("labels") or []
        label = str(labels[best]) if len(labels) > best and isinstance(labels[best], str) \
            else query_text
        return Detection(label=label or query_text, score=score, box_xyxy=tuple(box),
                         mask=self._mask(rgb, box))


def make_detector(backend, **kwargs) -> Detector:
    if backend == "mock":
        return MockDetector()
    if backend == "grounding_dino_sam2":
        return GroundingDinoSam2Detector(**kwargs)
    raise ValueError(f"unknown backend '{backend}'")
