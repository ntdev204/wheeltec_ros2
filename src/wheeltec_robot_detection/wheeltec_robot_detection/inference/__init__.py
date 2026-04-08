"""Base detector interface for object detection."""

from abc import ABC, abstractmethod
from typing import List, Tuple, Dict
import numpy as np


class BaseDetector(ABC):
    """Abstract base class for object detectors."""

    def __init__(self, model_path: str, input_size: Tuple[int, int],
                 confidence_threshold: float = 0.5, nms_threshold: float = 0.45):
        """
        Initialize detector.

        Args:
            model_path: Path to model file
            input_size: Input image size (width, height)
            confidence_threshold: Confidence threshold for detections
            nms_threshold: NMS IOU threshold
        """
        self.model_path = model_path
        self.input_size = input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold

    @abstractmethod
    def load_model(self) -> None:
        """Load the detection model."""
        pass

    @abstractmethod
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for inference.

        Args:
            image: Input image (BGR format)

        Returns:
            Preprocessed image tensor
        """
        pass

    @abstractmethod
    def inference(self, preprocessed_image: np.ndarray) -> np.ndarray:
        """
        Run inference on preprocessed image.

        Args:
            preprocessed_image: Preprocessed image tensor

        Returns:
            Raw model output
        """
        pass

    @abstractmethod
    def postprocess(self, output: np.ndarray, original_shape: Tuple[int, int]) -> List[Dict]:
        """
        Postprocess model output to get detections.

        Args:
            output: Raw model output
            original_shape: Original image shape (height, width)

        Returns:
            List of detections, each containing:
                - class_id: int
                - class_name: str
                - confidence: float
                - bbox: [x_min, y_min, x_max, y_max] in original image coordinates
        """
        pass

    def detect(self, image: np.ndarray) -> List[Dict]:
        """
        Run full detection pipeline.

        Args:
            image: Input image (BGR format)

        Returns:
            List of detections
        """
        original_shape = image.shape[:2]
        preprocessed = self.preprocess(image)
        output = self.inference(preprocessed)
        detections = self.postprocess(output, original_shape)
        return detections
