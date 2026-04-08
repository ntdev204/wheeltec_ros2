"""Export YOLOv8m PyTorch model to ONNX format."""

from ultralytics import YOLO
import argparse


def export_to_onnx(model_path: str, output_path: str, img_size: int = 640):
    """
    Export YOLOv8 model to ONNX format.

    Args:
        model_path: Path to YOLOv8 .pt file
        output_path: Output path for ONNX file
        img_size: Input image size
    """
    print(f"Loading model from {model_path}")
    model = YOLO(model_path)

    print(f"Exporting to ONNX (input size: {img_size}x{img_size})")
    model.export(
        format="onnx",
        imgsz=img_size,
        opset=17,          # TensorRT compatibility
        simplify=True,     # ONNX simplifier
        dynamic=False,     # Fixed input size for TensorRT
        half=False,        # FP32 ONNX (TRT will handle INT8)
    )

    print(f"ONNX model exported successfully")
    print(f"Output: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export YOLOv8 to ONNX")
    parser.add_argument("--model", type=str, default="yolov8m.pt", help="Path to YOLOv8 .pt file")
    parser.add_argument("--output", type=str, default="yolov8m.onnx", help="Output ONNX file path")
    parser.add_argument("--img-size", type=int, default=640, help="Input image size")

    args = parser.parse_args()

    export_to_onnx(args.model, args.output, args.img_size)
