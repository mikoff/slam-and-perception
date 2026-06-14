import torch
import sys

def main():
    print("=" * 50)
    print("System & PyTorch Verification Script")
    print("=" * 50)
    print(f"Python Version: {sys.version}")
    print(f"PyTorch Version: {torch.__version__}")
    
    cuda_available = torch.cuda.is_available()
    print(f"CUDA Available: {cuda_available}")
    
    if cuda_available:
        print(f"CUDA Device Count: {torch.cuda.device_count()}")
        print(f"Current Device Index: {torch.cuda.current_device()}")
        print(f"Device Name: {torch.cuda.get_device_name(0)}")
        print(f"CUDA Capability: {torch.cuda.get_device_capability(0)}")
        
        # Test tensor operations on GPU
        try:
            device = torch.device("cuda")
            x = torch.randn(1000, 1000, device=device)
            y = torch.randn(1000, 1000, device=device)
            z = torch.matmul(x, y)
            print("Successfully performed matrix multiplication on GPU.")
        except Exception as e:
            print(f"Error executing operations on GPU: {e}")
    else:
        print("CUDA is NOT available. PyTorch will run on CPU.")
        # Test tensor operations on CPU
        x = torch.randn(1000, 1000)
        y = torch.randn(1000, 1000)
        z = torch.matmul(x, y)
        print("Successfully performed matrix multiplication on CPU.")
    
    print("=" * 50)

if __name__ == "__main__":
    main()
