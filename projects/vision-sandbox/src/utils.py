import os
from pathlib import Path
import torch
from torch.utils.data import DataLoader, Subset
import torchvision
import torchvision.transforms as transforms

def get_data_dir():
    curr = Path(os.getcwd()).resolve()
    # Try finding 'data' in parent folders (useful for notebooks and scripts)
    for _ in range(6):
        if (curr / "data").exists():
            return curr / "data"
        curr = curr.parent
    
    # Fallback to direct absolute path
    absolute_fallback = Path("/home/sashamikoff/git/skillup/data")
    if absolute_fallback.exists():
        return absolute_fallback
        
    raise FileNotFoundError("Could not locate global 'data' directory.")

def get_cifar_dataloaders(batch_size=128, num_workers=2, pin_memory=True, train_subset_size=10000, test_subset_size=1000):
    data_dir = get_data_dir()
    
    transform_train = transforms.Compose([
        transforms.RandomCrop(32, padding=4),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize((0.4914, 0.4822, 0.4465), (0.2023, 0.1994, 0.2010)),
    ])
    
    transform_test = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.4914, 0.4822, 0.4465), (0.2023, 0.1994, 0.2010)),
    ])
    
    trainset = torchvision.datasets.CIFAR10(root=str(data_dir), train=True, download=False, transform=transform_train)
    testset = torchvision.datasets.CIFAR10(root=str(data_dir), train=False, download=False, transform=transform_test)
    
    # Create subsets
    train_subset = Subset(trainset, list(range(train_subset_size)))
    test_subset = Subset(testset, list(range(test_subset_size)))
    
    train_loader = DataLoader(
        train_subset, 
        batch_size=batch_size, 
        shuffle=True, 
        num_workers=num_workers, 
        pin_memory=pin_memory
    )
    
    test_loader = DataLoader(
        test_subset, 
        batch_size=batch_size, 
        shuffle=False, 
        num_workers=num_workers, 
        pin_memory=pin_memory
    )
    
    return train_loader, test_loader, testset
