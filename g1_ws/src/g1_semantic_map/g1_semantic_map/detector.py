"""Optional COCO instance-mask inference; imported only for the live node."""

import numpy as np


class SyntheticPaletteDetector:
    """Fixture-only oracle masks; never use for real perception."""

    categories = {0: "__background__", 62: "chair", 67: "dining table"}

    def predict(self, rgb):
        labels = np.zeros(rgb.shape[:2], dtype=np.uint16)
        scores = np.zeros(rgb.shape[:2], dtype=np.float32)
        red = (rgb[:, :, 0] > 180) & (rgb[:, :, 1] < 70)
        cyan = (rgb[:, :, 2] > 180) & (rgb[:, :, 1] > 110)
        labels[red] = 62
        labels[cyan] = 67
        scores[red | cyan] = 0.99
        return labels, scores


class CocoMaskRCNN:
    def __init__(self, score_threshold=0.65, mask_threshold=0.5, device="cpu"):
        import torch
        from torchvision.models.detection import (MaskRCNN_ResNet50_FPN_V2_Weights,
                                                   maskrcnn_resnet50_fpn_v2)

        self.torch = torch
        self.device = torch.device(device)
        self.weights = MaskRCNN_ResNet50_FPN_V2_Weights.DEFAULT
        self.categories = self.weights.meta["categories"]
        self.model = maskrcnn_resnet50_fpn_v2(weights=self.weights).to(self.device).eval()
        self.score_threshold = score_threshold
        self.mask_threshold = mask_threshold

    def predict(self, rgb):
        """Return per-pixel COCO class IDs and instance confidences."""
        torch = self.torch
        tensor = self.weights.transforms()(torch.from_numpy(rgb.copy()).permute(2, 0, 1))
        with torch.inference_mode():
            result = self.model([tensor.to(self.device)])[0]
        labels = np.zeros(rgb.shape[:2], dtype=np.uint16)
        confidence = np.zeros(rgb.shape[:2], dtype=np.float32)
        for score, label, mask in zip(result["scores"].cpu().numpy(),
                                      result["labels"].cpu().numpy(),
                                      result["masks"].cpu().numpy()):
            if score < self.score_threshold:
                break  # outputs are sorted by confidence
            selected = (mask[0] >= self.mask_threshold) & (confidence < score)
            labels[selected] = int(label)
            confidence[selected] = float(score)
        return labels, confidence
