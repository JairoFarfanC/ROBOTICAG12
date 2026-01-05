import torch
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
from model import MNISTNet

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize((0.1307,), (0.3081,))
])

test_dataset = datasets.MNIST(
    root="data",
    train=False,
    download=True,
    transform=transform
)

test_loader = DataLoader(test_dataset, batch_size=1000, shuffle=False)

model = MNISTNet().to(device)
model.load_state_dict(torch.load("../my_network.pt", map_location=device))
model.eval()

correct = 0
total = 0

with torch.no_grad():
    for x, y in test_loader:
        x, y = x.to(device), y.to(device)
        out = model(x)
        pred = out.argmax(dim=1)
        correct += (pred == y).sum().item()
        total += y.size(0)

accuracy = correct / total
print(f"Accuracy: {accuracy*100:.2f}%")
