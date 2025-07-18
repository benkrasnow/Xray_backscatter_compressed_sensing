PyLBFGS
=======

PyLBFGS is a Python 3 wrapper of the [libLBFGS][libLBFGS] library -- a C port (written by Naoaki Okazaki) of the Limited-memory Broyden-Fletcher-Goldfarb-Shanno (L-BFGS) algorithm (written in Fortran by Jorge Nocedal).

At this time, only the Orthant-Wise Limited-memory Quasi-Newton (OWL-QN) algorithm is exposed (although very little work would be required to expose the full [libLBFGS][libLBFGS] implementation).

Also note that PyLBFGS has only been compiled and tested with Python 3 on a 64-bit Ubuntu machine. It should work in other environments... but tweaking the installation commands is a task left to the reader.



Install
-------

1. Download [libLBFGS][libLBFGS] source code and install (see its README for extra info):

        sudo apt install libtool automake virtualenv
        ./autogen.sh
        ./configure --enable-sse2
        make
        sudo make install

2. Download or clone the PyLBFGS project:

        git clone https://rtaylor@bitbucket.org/rtaylor/pylbfgs.git
        cd pylbfgs

3. Setup and activate a virtual environment (modify the Python version if necessary):

        virtualenv -p python3.6 --prompt="(pylbfgs) " .venv
        . .venv/bin/activate

4. Install the project:

        python setup.py install

5. Alternatively, build in-place:

        python setup.py build_ext -i



Basic Use
---------

Import the OWL-QN algorithm:

    from pylbfgs import owlqn


Define an evaluation callback to provide the objective function and gradient evaluations:

    def evaluate(x, g, step):
        # The algorithm calls this callback to obtain the values of the
        # objective function and its gradients when needed. Store the
        # gradients into the numpy array g. Return the objective function
        # evaluated at x.
        return 0.0


Optionally, define a callback to receive the progress of the optimization process:

    def progress(x, g, fx, xnorm, gnorm, step, k, ls):
        # Print variables to screen or file or whatever. Return zero to 
        # continue algorithm; non-zero will halt execution.
        return 0


Run the OWL-QN algorithm. The result will be the minimum in vector form.

    orthantwise_c = 5
    x = owlqn(size_of_x, evaluate, progress, orthantwise_c)


If you want to use the built-in progress callback, just pass `None`.

Refer to the [libLBFGS API][libLBFGS_API] for more info about what each of the callback arguments mean.



Examples
--------

See the *example.py* script for an advanced example in which we use compressed sensing to reconstruct a sparsely sampled image. A more thorough example of compressed sensing (using PyLBFGS) can be found on my weblog [here][blog_post].

    pip install Pillow==5.0.0 scipy==1.0.0 matplotlib==2.1.2
    python example.py



Author & License
----------------

PyLBFGS was written by Robert Taylor and is licensed under the MIT license.



[liblbfgs]: http://chokkan.org/software/liblbfgs/
[libLBFGS_API]: http://www.chokkan.org/software/liblbfgs/index.html
[blog_post]: http://www.pyrunner.com/weblog/B/
