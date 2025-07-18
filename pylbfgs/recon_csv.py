#!/usr/bin/env python

import os
import numpy as np
import scipy.fftpack as spfft
import scipy.ndimage as spimg
#import matplotlib
#matplotlib.use('Agg') # no UI backend
import matplotlib.pyplot as plt

import imageio.v2 as imageio
from PIL import Image

from pylbfgs import owlqn

import csv


# Coeefficient for the L1 norm of variables (see OWL-QN algorithm)
ORTHANTWISE_C = 4

# File paths
ORIG_CAV_PATH = './continuous_output.csv'


#Dims
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 800

def dct2(x):
    """Return 2D discrete cosine transform.
    """
    return spfft.dct(
        spfft.dct(x.T, norm='ortho', axis=0).T, norm='ortho', axis=0)

def idct2(x):
    """Return inverse 2D discrete cosine transform.
    """
    return spfft.idct(
        spfft.idct(x.T, norm='ortho', axis=0).T, norm='ortho', axis=0)

def progress(x, g, fx, xnorm, gnorm, step, k, ls):
    """Just display the current iteration.
    """
    print('Iteration {}'.format(k))
    return 0


_image_dims = None  # track target image dimensions here
_ri_vector = None  # reference the random sampling indices here
_b_vector = None  # reference the sampled vector b here
_A_matrix = None  # reference the dct matrix operator A here


def evaluate(x, g, step):
    """An in-memory evaluation callback.
    """

    # we want to return two things:
    # (1) the norm squared of the residuals, sum((Ax-b).^2), and
    # (2) the gradient 2*A'(Ax-b)

    # expand x columns-first
    x2 = x.reshape((_image_dims[1], _image_dims[0])).T

    # Ax is just the inverse 2D dct of x2
    Ax2 = idct2(x2)

    # stack columns and extract samples
    Ax = Ax2.T.flat[_ri_vector].reshape(_b_vector.shape)

    # calculate the residual Ax-b and its 2-norm squared
    Axb = Ax - _b_vector
    fx = np.sum(np.power(Axb, 2))

    # project residual vector (k x 1) onto blank image (ny x nx)
    Axb2 = np.zeros(x2.shape)
    Axb2.T.flat[_ri_vector] = Axb  # fill columns-first

    # A'(Ax-b) is just the 2D dct of Axb2
    AtAxb2 = 2 * dct2(Axb2)
    AtAxb = AtAxb2.T.reshape(x.shape)  # stack columns

    # copy over the gradient vector
    np.copyto(g, AtAxb)

    return fx


def parse_csv(file_path: str):
    """
    Opens a CSV file, reads three comma-separated integer values from each line,
    and fills two 1D arrays based on these values.

    The first array is filled with (first_integer + second_integer * 600).
    The second array is filled with the third integer value.

    If the combination of the first and second integers is a duplicate of a previously
    processed line, that line's data will be discarded.

    Args:
        file_path (str): The path to the CSV file.

    Returns:
        tuple: A tuple containing two lists (arrays):
               - list_calculated_values: Stores (first_int + second_int * 600) for unique pairs
               - list_third_values: Stores the third integer value for unique pairs
               Returns (None, None) if the file cannot be opened or an error occurs during parsing.
    """
    list_calculated_values = []
    list_third_values = []
    seen_pairs = set() # To keep track of unique (first_int, second_int) combinations

    try:
        with open(file_path, mode='r', newline='') as csvfile:
            csv_reader = csv.reader(csvfile)
            line_num = 0
            for row in csv_reader:
                line_num += 1
                if not row:  # Skip empty lines
                    continue

                if len(row) != 3:
                    print(f"Warning: Skipping line {line_num} due to incorrect number of values. Expected 3, got {len(row)}: {row}")
                    continue

                try:
                    # Attempt to convert values to integers
                    val1 = int(row[0].strip())
                    val2 = int(row[1].strip())
                    val3 = int(row[2].strip())

                    current_pair = (val1, val2)

                    # Check for duplicates of the (val1, val2) pair
                    if current_pair not in seen_pairs:
                        seen_pairs.add(current_pair) # Add the new unique pair

                        # Calculate the first array value
                        calculated_value = val1 + (val2 * IMAGE_WIDTH)
                        list_calculated_values.append(calculated_value)

                        # Store the third value in the second array
                        list_third_values.append(val3)
                    else:
                        print(f"Info: Skipping line {line_num} due to duplicate (first, second) integer values: {current_pair}")

                except ValueError:
                    print(f"Warning: Skipping line {line_num} due to non-integer values: {row}")
                except Exception as e:
                    print(f"An unexpected error occurred on line {line_num}: {e} - {row}")

    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
        return None, None
    except Exception as e:
        print(f"An error occurred while opening or reading the file: {e}")
        return None, None

    return list_calculated_values, list_third_values



def main():

    global _b_vector, _A_matrix, _image_dims, _ri_vector

    # read image in grayscale, then downscale it
    #Xorig = spimg.imread(ORIG_IMAGE_PATH, flatten=True, mode='L')
    #Xorig = imageio.imread(ORIG_IMAGE_PATH, mode='L')
    #X = spimg.zoom(Xorig, SCALE)
    
    #ny, nx = X.shape
    ny = IMAGE_HEIGHT
    nx = IMAGE_WIDTH 
    X = np.zeros((nx, ny))

    # take random samples of image, store them in a vector b
    #k = round(nx * ny * SAMPLE)
    #ri = np.random.choice(nx*ny, k, replace=False)  # random sample of indices
    #b = X.T.flat[ri].astype(float)  # important: cast to 64 bit

    ri, b = parse_csv(ORIG_CAV_PATH)


    # This method evaluates the objective function sum((Ax-b).^2) and its
    # gradient without ever actually generating A (which can be massive).
    # Our ability to do this stems from our knowledge that Ax is just the
    # sampled idct2 of the spectral image (x in matrix form).

    # save image dims, sampling vector, and b vector and to global vars
    _image_dims = (ny, nx)
    _ri_vector = ri
    _b_vector = np.expand_dims(b, axis=1)

    # perform the L1 minimization in memory
    Xat2 = owlqn(nx*ny, evaluate, progress, ORTHANTWISE_C)

    

    # transform the output back into the spatial domain
    Xat = Xat2.reshape(nx, ny).T  # stack columns
    Xa = idct2(Xat)

    # create images of mask (for visualization)
    mask = np.zeros(X.shape)
    mask.T.flat[ri] = 255
    Xm = 255 * np.ones(X.shape)
    Xm.T.flat[ri] = X.T.flat[ri]

    # display the result
    f, ax = plt.subplots(1, 3, figsize=(14, 4))
    ax[0].imshow(X, cmap='gray', interpolation='none')
    ax[1].imshow(Xm, cmap='gray', interpolation='none')
    ax[2].imshow(Xa, cmap='gray', interpolation='none')
    plt.show()
    
    #imageio.imwrite('./input_image.png', X)
    imageio.imwrite('./sampled_image.png', Image.fromarray(Xm).convert('L'))
    imageio.imwrite('./reconstructed_image.png', Image.fromarray(Xa).convert('L'))

if __name__ == '__main__':
    main()
