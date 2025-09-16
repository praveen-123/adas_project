import torch
from ultralytics import YOLO

print("Testing GPU setup...")
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")

if torch.cuda.is_available():
    print(f"GPU device: {torch.cuda.get_device_name(0)}")
    print(f"GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.2f} GB")
    
    # Test with a small model
    model = YOLO('yolov8n.pt').to('cuda:0')
    results = model('https://ultralytics.com/images/bus.jpg', device=0)
    print("GPU test successful!")
else:
    print("GPU not available, using CPU")
