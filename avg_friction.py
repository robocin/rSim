import numpy as np

heights = np.array([
    93.0, 79.5, 101.0, 95.0,
    92.0, 93.5, 98.0, 94.0,
    108.0, 104.0, 101.0,
    103.0, 96.0, 94.0
])

hip = 150.0

sine = heights.mean()/hip

cosine = np.sqrt(1 - sine**2)

tan = sine/cosine

print(tan)

