"""ByteTrack implementation for multi-object tracking."""

import numpy as np
from typing import List, Dict, Tuple
from scipy.optimize import linear_sum_assignment


class KalmanFilter:
    """Simple Kalman filter for bbox tracking."""

    def __init__(self):
        """Initialize Kalman filter with constant velocity model."""
        # State: [x, y, w, h, vx, vy, vw, vh]
        self.dt = 1.0
        self.state = np.zeros(8)
        self.covariance = np.eye(8) * 1000

        # State transition matrix
        self.F = np.eye(8)
        self.F[0, 4] = self.dt
        self.F[1, 5] = self.dt
        self.F[2, 6] = self.dt
        self.F[3, 7] = self.dt

        # Measurement matrix
        self.H = np.eye(4, 8)

        # Process noise
        self.Q = np.eye(8) * 0.01

        # Measurement noise
        self.R = np.eye(4) * 10

    def predict(self):
        """Predict next state."""
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        return self.state[:4]

    def update(self, measurement: np.ndarray):
        """Update state with measurement."""
        y = measurement - self.H @ self.state
        S = self.H @ self.covariance @ self.H.T + self.R
        K = np.linalg.solve(S, self.H @ self.covariance).T

        self.state = self.state + K @ y
        self.covariance = (np.eye(8) - K @ self.H) @ self.covariance

    def get_state(self) -> np.ndarray:
        """Get current bbox state [x, y, w, h]."""
        return self.state[:4]

    def get_velocity(self) -> np.ndarray:
        """Get current velocity [vx, vy]."""
        return self.state[4:6]


class Track:
    """Single object track."""

    _id_counter = 0

    def __init__(self, bbox: np.ndarray, class_id: int, class_name: str, confidence: float):
        """
        Initialize track.

        Args:
            bbox: Bounding box [x_min, y_min, x_max, y_max]
            class_id: Class ID
            class_name: Class name
            confidence: Detection confidence
        """
        self.track_id = Track._id_counter
        Track._id_counter += 1

        self.class_id = class_id
        self.class_name = class_name
        self.confidence = confidence

        # Convert to center format [x, y, w, h]
        x_min, y_min, x_max, y_max = bbox
        x = (x_min + x_max) / 2
        y = (y_min + y_max) / 2
        w = x_max - x_min
        h = y_max - y_min

        self.kalman = KalmanFilter()
        self.kalman.state[:4] = [x, y, w, h]

        self.age = 0
        self.hits = 1
        self.time_since_update = 0
        self.is_confirmed = False

    def predict(self):
        """Predict next position."""
        self.age += 1
        self.time_since_update += 1
        return self.kalman.predict()

    def update(self, bbox: np.ndarray, confidence: float):
        """Update track with new detection."""
        # Convert to center format
        x_min, y_min, x_max, y_max = bbox
        x = (x_min + x_max) / 2
        y = (y_min + y_max) / 2
        w = x_max - x_min
        h = y_max - y_min

        self.kalman.update(np.array([x, y, w, h]))
        self.confidence = confidence
        self.hits += 1
        self.time_since_update = 0

        # Confirm track after 3 consecutive hits
        if self.hits >= 3:
            self.is_confirmed = True

    def get_bbox(self) -> np.ndarray:
        """Get current bbox in corner format [x_min, y_min, x_max, y_max]."""
        x, y, w, h = self.kalman.get_state()
        x_min = x - w / 2
        y_min = y - h / 2
        x_max = x + w / 2
        y_max = y + h / 2
        return np.array([x_min, y_min, x_max, y_max])

    def get_velocity(self) -> Tuple[float, float]:
        """Get velocity in pixels/frame."""
        vx, vy = self.kalman.get_velocity()
        return float(vx), float(vy)


class ByteTracker:
    """ByteTrack multi-object tracker."""

    def __init__(self, track_thresh: float = 0.5, track_buffer: int = 30,
                 match_thresh: float = 0.8, min_box_area: int = 100):
        """
        Initialize ByteTracker.

        Args:
            track_thresh: Detection confidence threshold for track initialization
            track_buffer: Number of frames to keep lost tracks
            match_thresh: IOU threshold for matching
            min_box_area: Minimum bounding box area
        """
        self.track_thresh = track_thresh
        self.track_buffer = track_buffer
        self.match_thresh = match_thresh
        self.min_box_area = min_box_area

        self.tracked_tracks: List[Track] = []
        self.lost_tracks: List[Track] = []
        self.removed_tracks: List[Track] = []

    def update(self, detections: List[Dict]) -> List[Track]:
        """
        Update tracker with new detections.

        Args:
            detections: List of detections from detector

        Returns:
            List of active tracks
        """
        # Separate high and low confidence detections
        high_dets = []
        low_dets = []

        for det in detections:
            bbox = np.array(det['bbox'])
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])

            if area < self.min_box_area:
                continue

            if det['confidence'] >= self.track_thresh:
                high_dets.append(det)
            else:
                low_dets.append(det)

        # Predict all tracks
        for track in self.tracked_tracks:
            track.predict()

        # First association with high confidence detections
        unmatched_tracks, unmatched_dets = self._associate(
            self.tracked_tracks, high_dets, self.match_thresh
        )

        # Second association with low confidence detections
        if len(low_dets) > 0 and len(unmatched_tracks) > 0:
            unmatched_tracks_low = [self.tracked_tracks[i] for i in unmatched_tracks]
            _, unmatched_dets_low = self._associate(
                unmatched_tracks_low, low_dets, 0.5
            )
        else:
            unmatched_dets_low = list(range(len(low_dets)))

        # Initialize new tracks from unmatched high confidence detections
        for i in unmatched_dets:
            det = high_dets[i]
            track = Track(
                bbox=np.array(det['bbox']),
                class_id=det['class_id'],
                class_name=det['class_name'],
                confidence=det['confidence']
            )
            self.tracked_tracks.append(track)

        # Move lost tracks
        lost_tracks_idx = []
        for i, track in enumerate(self.tracked_tracks):
            if track.time_since_update > 0:
                lost_tracks_idx.append(i)

        for i in reversed(lost_tracks_idx):
            track = self.tracked_tracks.pop(i)
            self.lost_tracks.append(track)

        # Remove old lost tracks
        removed_idx = []
        for i, track in enumerate(self.lost_tracks):
            if track.time_since_update > self.track_buffer:
                removed_idx.append(i)

        for i in reversed(removed_idx):
            track = self.lost_tracks.pop(i)
            self.removed_tracks.append(track)

        # Return confirmed tracks
        return [t for t in self.tracked_tracks if t.is_confirmed]

    def _associate(self, tracks: List[Track], detections: List[Dict],
                   iou_threshold: float) -> Tuple[List[int], List[int]]:
        """
        Associate tracks with detections using Hungarian algorithm.

        Args:
            tracks: List of tracks
            detections: List of detections
            iou_threshold: IOU threshold for matching

        Returns:
            Tuple of (unmatched_track_indices, unmatched_detection_indices)
        """
        if len(tracks) == 0 or len(detections) == 0:
            return list(range(len(tracks))), list(range(len(detections)))

        # Compute IOU matrix
        iou_matrix = np.zeros((len(tracks), len(detections)))

        for i, track in enumerate(tracks):
            track_bbox = track.get_bbox()
            for j, det in enumerate(detections):
                det_bbox = np.array(det['bbox'])
                iou_matrix[i, j] = self._iou(track_bbox, det_bbox)

        # Hungarian algorithm
        cost_matrix = 1 - iou_matrix
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Filter matches by IOU threshold
        unmatched_tracks = list(range(len(tracks)))
        unmatched_dets = list(range(len(detections)))

        for i, j in zip(row_ind, col_ind):
            if iou_matrix[i, j] >= iou_threshold:
                # Match found
                tracks[i].update(
                    bbox=np.array(detections[j]['bbox']),
                    confidence=detections[j]['confidence']
                )
                unmatched_tracks.remove(i)
                unmatched_dets.remove(j)

        return unmatched_tracks, unmatched_dets

    @staticmethod
    def _iou(bbox1: np.ndarray, bbox2: np.ndarray) -> float:
        """
        Calculate IOU between two bboxes.

        Args:
            bbox1: [x_min, y_min, x_max, y_max]
            bbox2: [x_min, y_min, x_max, y_max]

        Returns:
            IOU value
        """
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])

        inter_area = max(0, x2 - x1) * max(0, y2 - y1)

        bbox1_area = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        bbox2_area = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])

        union_area = bbox1_area + bbox2_area - inter_area

        if union_area == 0:
            return 0.0

        return inter_area / union_area
