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


# Fraction to scale the original image
SCALE = 1

# Fraction of the scaled image to randomly sample
SAMPLE = .25

# Coeefficient for the L1 norm of variables (see OWL-QN algorithm)
ORTHANTWISE_C = 4

# File paths
ORIG_IMAGE_PATH = 'test/cat_drawing_800.png'



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

def main():

    global _b_vector, _A_matrix, _image_dims, _ri_vector

    # read image in grayscale, then downscale it
    #Xorig = spimg.imread(ORIG_IMAGE_PATH, flatten=True, mode='L')
    Xorig = imageio.imread(ORIG_IMAGE_PATH, mode='L')
    X = spimg.zoom(Xorig, SCALE)
    ny, nx = X.shape

    # take random samples of image, store them in a vector b
    k = round(nx * ny * SAMPLE)
    ri = np.random.choice(nx*ny, k, replace=False)  # random sample of indices
    b = X.T.flat[ri].astype(float)  # important: cast to 64 bit


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
    
    imageio.imwrite('./input_image.png', X)
    imageio.imwrite('./sampled_image.png', Image.fromarray(Xm).convert('L'))
    imageio.imwrite('./reconstructed_image.png', Image.fromarray(Xa).convert('L'))

if __name__ == '__main__':
    main()
