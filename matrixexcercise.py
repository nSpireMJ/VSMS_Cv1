# libraries import

import numpy as np

# functions

def matrix_addition(A,B):
    return np.add(A,B)

def matrix_multiplication(C,D):
    return C @ D

def matrix_transpose(E):
    return E.T

def matrix_determinant(F):
    return np.linalg.det(F)

def matrix_inverse(G):
    return np.linalg.inv(G)

if __name__ == '__main__':
    # matrix definition
    A = np.array([[1,2],[3,4]])
    B = np.array([[5,6],[7,8]])

    E = np.array([[1,2],[4,5]])

    # function call
    print(matrix_addition(A,B))
    print(matrix_multiplication(A,B))
    print(matrix_transpose(E))
    print(matrix_determinant(E))
    print(matrix_inverse(E))