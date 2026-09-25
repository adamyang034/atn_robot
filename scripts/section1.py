#!/usr/bin/env python3

import numpy as np

# add import and helper functions here

if __name__ == "__main__":
    np.random.seed(42)
    A = np.random.normal(size=(4, 4))
    B = np.random.normal(size=(4, 2))
    C = A @ B
    print(C)