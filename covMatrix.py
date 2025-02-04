import numpy as np
from typing import List

def covar(xs: List[float], ys: List[float]) -> np.ndarray:
    return np.cov(np.array([xs, ys])) # covariance matrix

# Test
xs = [0.4, 0.1, 0.2, 0.05, 0.6, 0.3, 0.05, 0.3, 0.5, 0.05]
ys = [0.2, 0.1, 0.1, 0.8, 0.2, 0.4, 0.05, 0.2, 0.3, 0]
print(covar(xs, ys))