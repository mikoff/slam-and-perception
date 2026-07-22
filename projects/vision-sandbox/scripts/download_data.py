import torchvision
from pathlib import Path

def main():
    # Path to repo root: projects/vision-sandbox/scripts/download_data.py -> projects -> skillup (repo root)
    repo_root = Path(__file__).resolve().parent.parent.parent.parent
    data_dir = repo_root / "data"
    print(f"Downloading CIFAR-10 to {data_dir}...")
    
    # Download train and test sets
    torchvision.datasets.CIFAR10(root=str(data_dir), train=True, download=True)
    torchvision.datasets.CIFAR10(root=str(data_dir), train=False, download=True)
    print("CIFAR-10 downloaded successfully.")

if __name__ == "__main__":
    main()
