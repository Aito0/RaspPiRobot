import numpy as np
from typing import List

def covar(xs: List[float], ys: List[float]) -> np.ndarray:

    return np.cov(np.array([xs, ys])) # covariance matrix

# Test
xs = [1, 2, 3, 4, 20]
ys = [2, 3, 15, 5, 6]
print(covar(xs, ys)) # [[62.5, 1.75], [1.75, 26.7]]