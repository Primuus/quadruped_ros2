from __future__ import annotations

from pathlib import Path

import numpy as np
import torch


class TorchScriptPolicy:
    def __init__(self, model_path: str | Path, device: str = 'cpu') -> None:
        self.model_path = Path(model_path).expanduser().resolve()
        if not self.model_path.exists():
            raise FileNotFoundError(f'Policy file not found: {self.model_path}')

        self.device = torch.device(device)
        self.model = torch.jit.load(str(self.model_path), map_location=self.device)
        self.model.eval()

    def infer(self, observation: np.ndarray) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32)
        obs_tensor = torch.from_numpy(obs).to(self.device).unsqueeze(0)

        with torch.no_grad():
            output = self.model(obs_tensor)

        if isinstance(output, (tuple, list)):
            output = output[0]

        action = output.squeeze(0).detach().to('cpu').numpy().astype(np.float32)
        return action

