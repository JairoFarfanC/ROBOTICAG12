import os
import numpy as np
import torch

from model import MNISTNet

# RoboComp imports (existirán en Linux)
try:
    import RoboCompMNIST
except:
    pass


class SpecificWorker:
    def __init__(self, proxy_map=None):
        # --- Load DNN model ONCE ---
        self.device = torch.device("cpu")
        self.model = MNISTNet().to(self.device)

        model_path = os.path.join(
            os.path.dirname(__file__), "..", "my_network.pt"
        )
        state = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(state)
        self.model.eval()

        # Last detection
        self.last_digit = -1
        self.last_confidence = 0.0

        print("[DNN] MNIST model loaded")

    # --------------------------------------------------
    # Internal method: classify a 28x28 grayscale image
    # --------------------------------------------------
    def classify_28x28(self, img_28):
        """
        img_28: numpy array (28x28), grayscale [0..255]
        """
        x = torch.tensor(img_28, dtype=torch.float32)
        x = x.unsqueeze(0).unsqueeze(0) / 255.0

        with torch.no_grad():
            out = self.model(x)
            probs = torch.softmax(out, dim=1)
            conf, digit = torch.max(probs, dim=1)

        return int(digit.item()), float(conf.item())

    # --------------------------------------------------
    # Ice interface method (called from localiser)
    # --------------------------------------------------
    def get_number(self):
        """
        Returns the last detected digit or -1
        """
        if self.last_confidence < 0.8:
            return RoboCompMNIST.MNISTResult(-1, 0.0)

        return RoboCompMNIST.MNISTResult(
            self.last_digit,
            self.last_confidence
        )

    # --------------------------------------------------
    # Periodic compute (called by RoboComp)
    # --------------------------------------------------
    def compute(self):
        """
        This method should:
        1. Read image from Camera360RGB
        2. Detect a black square region
        3. Resize to 28x28
        4. Call classify_28x28()
        """

        # TODO: implemented by teammates (image format from Webots)
        self.last_digit = -1
        self.last_confidence = 0.0
