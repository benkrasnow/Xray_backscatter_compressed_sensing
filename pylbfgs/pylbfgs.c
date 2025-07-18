#include <Python.h>
#include <numpy/arrayobject.h>
#include <stdio.h>
#include <math.h>
#include <lbfgs.h>

typedef int bool;
#define true 1
#define false 0

static char module_docstring[] =
    "This module provides an interface to the Limited-memory Broyden-Fletcher-Goldfarb-Shanno (L-BFGS) algorithm written in C.";
static char owlqs_docstring[] =
    "Calculate the minimum of a user-defined objective function plus the L1-norm of the parameters.";

static PyObject *pylbfgs_owlqn(PyObject *self, PyObject *args);

static PyMethodDef module_methods[] = {
    {"owlqn", pylbfgs_owlqn, METH_VARARGS, owlqs_docstring},
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC PyInit_pylbfgs(void)
{
    
    PyObject *module;
    static struct PyModuleDef moduledef = {
        PyModuleDef_HEAD_INIT,
        "pylbfgs",
        module_docstring,
        -1,
        module_methods,
        NULL,
        NULL,
        NULL,
        NULL
    };
    module = PyModule_Create(&moduledef);
    if (!module) return NULL;

    /* Load `numpy` functionality. */
    import_array();

    return module;
}

static void set_error_string(const int value)
{
    switch (value) {

        case LBFGS_SUCCESS: // same as LBFGS_CONVERGENCE
            // do nothing
            break;
        case LBFGS_STOP:
            PyErr_WarnEx(NULL, "Optimization stopped.", 1);
            break;
        case LBFGS_ALREADY_MINIMIZED:
            PyErr_WarnEx(NULL, "The initial variables already minimize the objective function.", 1);
            break;
        case LBFGSERR_LOGICERROR:
            PyErr_SetString(PyExc_RuntimeError, "Logic error.");
            break;
        case LBFGSERR_OUTOFMEMORY:
            PyErr_SetString(PyExc_RuntimeError, "Insufficient memory.");
            break;
        case LBFGSERR_CANCELED:
            PyErr_SetString(PyExc_RuntimeError, "The minimization process has been canceled.");
            break;
        case LBFGSERR_INVALID_N:
            PyErr_SetString(PyExc_RuntimeError, "Invalid number of variables specified.");
            break;
        case LBFGSERR_INVALID_N_SSE:
            PyErr_SetString(PyExc_RuntimeError, "Invalid number of variables (for SSE) specified.");
            break;
        case LBFGSERR_INVALID_X_SSE:
            PyErr_SetString(PyExc_RuntimeError, "The array x must be aligned to 16 (for SSE).");
            break;
        case LBFGSERR_INVALID_EPSILON:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::epsilon specified.");
            break;
        case LBFGSERR_INVALID_TESTPERIOD:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::past specified.");
            break;
        case LBFGSERR_INVALID_DELTA:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::delta specified.");
            break;
        case LBFGSERR_INVALID_LINESEARCH:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::linesearch specified.");
            break;
        case LBFGSERR_INVALID_MINSTEP:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::max_step specified.");
            break;
        case LBFGSERR_INVALID_MAXSTEP:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::max_step specified.");
            break;
        case LBFGSERR_INVALID_FTOL:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::ftol specified.");
            break;
        case LBFGSERR_INVALID_WOLFE:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::wolfe specified.");
            break;
        case LBFGSERR_INVALID_GTOL:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::gtol specified.");
            break;
        case LBFGSERR_INVALID_XTOL:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::xtol specified.");
            break;
        case LBFGSERR_INVALID_MAXLINESEARCH:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::max_linesearch specified.");
            break;
        case LBFGSERR_INVALID_ORTHANTWISE:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::orthantwise_c specified.");
            break;
        case LBFGSERR_INVALID_ORTHANTWISE_START:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::orthantwise_start specified.");
            break;
        case LBFGSERR_INVALID_ORTHANTWISE_END:
            PyErr_SetString(PyExc_RuntimeError, "Invalid parameter lbfgs_parameter_t::orthantwise_end specified.");
            break;
        case LBFGSERR_OUTOFINTERVAL:
            PyErr_SetString(PyExc_RuntimeError, "The line-search step went out of the interval of uncertainty.");
            break;
        case LBFGSERR_INCORRECT_TMINMAX:
            PyErr_SetString(PyExc_RuntimeError, "A logic error occurred; alternatively, the interval of uncertainty became too small.");
            break;
        case LBFGSERR_ROUNDING_ERROR:
            PyErr_SetString(PyExc_RuntimeError, "A rounding error occurred; alternatively, no line-search step satisfies the sufficient decrease and curvature conditions.");
            break;
        case LBFGSERR_MINIMUMSTEP:
            PyErr_SetString(PyExc_RuntimeError, "The line-search step became smaller than lbfgs_parameter_t::min_step.");
            break;
        case LBFGSERR_MAXIMUMSTEP:
            PyErr_SetString(PyExc_RuntimeError, "The line-search step became larger than lbfgs_parameter_t::max_step.");
            break;
        case LBFGSERR_MAXIMUMLINESEARCH:
            PyErr_SetString(PyExc_RuntimeError, "The line-search routine reaches the maximum number of evaluations.");
            break;
        case LBFGSERR_MAXIMUMITERATION:
            PyErr_SetString(PyExc_RuntimeError, "The algorithm routine reaches the maximum number of iterations.");
            break;
        case LBFGSERR_WIDTHTOOSMALL:
            PyErr_SetString(PyExc_RuntimeError, "Relative width of the interval of uncertainty is at most lbfgs_parameter_t::xtol.");
            break;
        case LBFGSERR_INVALIDPARAMETERS:
            PyErr_SetString(PyExc_RuntimeError, "A logic error (negative line-search step) occurred.");
            break;
        case LBFGSERR_INCREASEGRADIENT:
            PyErr_SetString(PyExc_RuntimeError, "The current search direction increases the objective function value.");
            break;
        case LBFGSERR_UNKNOWNERROR:
        default:            
            PyErr_SetString(PyExc_RuntimeError, "An unknown error occurred.");
            break;
    }
}

static PyObject *py_callback_eval = NULL;
static PyObject *py_callback_prog = NULL;
static bool callback_error_occurred = false;

static lbfgsfloatval_t evaluate(
    void *instance,
    const lbfgsfloatval_t *x,
    lbfgsfloatval_t *g,
    const int n,
    const lbfgsfloatval_t step
    )
{
    /* Create args */
    npy_intp dims[2] = {n, 1};
    PyObject *x_vector = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, (void*)x);
    PyObject *g_vector = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, (void*)g);
    PyObject *arglist = Py_BuildValue("OOd", x_vector, g_vector, (double)step);
    if (x_vector == NULL || g_vector == NULL || arglist == NULL) {
        PyErr_SetString(PyExc_RuntimeError, "An unknown error occurred in evaluation method.");
        callback_error_occurred = true;
        return 0.0;
    }

    /* Execute callback */
    PyObject *result = PyObject_CallObject(py_callback_eval, arglist);

    /* Cleanup args */
    Py_DECREF(arglist);
    Py_DECREF(g_vector);
    Py_DECREF(x_vector);

    /* Check callback results */
    if (result == NULL) {
        // pass error back
        callback_error_occurred = true;
        return 0.0;
    }
    lbfgsfloatval_t fx = (lbfgsfloatval_t)PyFloat_AsDouble(result);
    Py_DECREF(result);
    if (PyErr_Occurred() != NULL) {
        // pass error back
        callback_error_occurred = true;
        return 0.0;
    }

    return fx;
}

static int progress(
    void *instance,
    const lbfgsfloatval_t *x,
    const lbfgsfloatval_t *g,
    const lbfgsfloatval_t fx,
    const lbfgsfloatval_t xnorm,
    const lbfgsfloatval_t gnorm,
    const lbfgsfloatval_t step,
    int n,
    int k,
    int ls
    )
{
    if (callback_error_occurred) {
        /* An error occurred, stop execution */
        return 1;
    }

    if (py_callback_prog == NULL) {
        /* Default progress report */
        PySys_WriteStdout("Iteration %d:\n", k);
        PySys_WriteStdout("  fx = %f, xnorm = %f, gnorm = %f, step = %f, k = %d, ls = %d\n", fx, xnorm, gnorm, step, k, ls);
        PySys_WriteStdout("\n");
    }
    else {
        /* Create args */
        npy_intp dims[2] = {n, 1};
        PyObject *x_vector = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, (void*)x);
        PyObject *g_vector = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, (void*)g);
        PyObject *arglist = Py_BuildValue("OOddddii",
            x_vector, g_vector, (double)fx, (double)xnorm, (double)gnorm, (double)step, k, ls);
        if (x_vector == NULL || g_vector == NULL || arglist == NULL) {
            PyErr_SetString(PyExc_RuntimeError, "An unknown error occurred in progress method.");
            return 0.0;
        }

        /* Execute callback */
        PyObject *result = PyObject_CallObject(py_callback_prog, arglist);

        /* Cleanup args */
        Py_DECREF(arglist);
        Py_DECREF(g_vector);
        Py_DECREF(x_vector);

        /* Check callback results */
        if (result == NULL) {
            // pass error back
            return 1;
        }
        int ret = PyLong_AsLong(result);
        Py_DECREF(result);
        if (PyErr_Occurred() != NULL) {
            // pass error back
            return 1;
        }

        return ret;
    }
    return 0;
}

static PyObject *pylbfgs_owlqn(PyObject *self, PyObject *args)
{
    /* Reset globals */
    py_callback_eval = NULL;
    py_callback_prog = NULL;

    /* Parse the input tuple */
    int n;
    double param_c;
    PyObject *eval_obj, *prog_obj;
    if (!PyArg_ParseTuple(args, "iOOd", &n, &eval_obj, &prog_obj, &param_c))
        return NULL;
    if (PyCallable_Check(eval_obj)) {
        Py_INCREF(eval_obj);
        py_callback_eval = eval_obj;
    }
    else {
        PyErr_SetString(PyExc_TypeError, "Evaluation method is not callable");
        return NULL;
    }
    if (PyCallable_Check(prog_obj)) {
        Py_INCREF(prog_obj);
        py_callback_prog = prog_obj;
    }
    
    /* Initialize solution vector */
    int i;
    lbfgsfloatval_t fx;
    lbfgsfloatval_t *x = lbfgs_malloc(n);
    if (x == NULL) {
        PyErr_SetString(PyExc_MemoryError, "Failed to allocate a memory block for variables");
        return NULL;
    }
    for (i = 0; i < n; i++) {
        x[i] = 1;
    }

    /* Initialize the parameters for the optimization. */
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = (lbfgsfloatval_t)param_c; // this tells lbfgs to do OWL-QN
    param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    // printf("params.c = %f\n", param.orthantwise_c);
    // printf("%d\n", LBFGSERR_INVALID_LINESEARCH);
    int lbfgs_ret = lbfgs(n, x, &fx, evaluate, progress, NULL, &param);
    // PySys_WriteStdout("OWL-QN optimization terminated with status code = %d\n", lbfgs_ret);
    set_error_string(lbfgs_ret);

    /* Copy solution to numpy array and free x */
    npy_intp dims[2] = {n, 1};
    PyObject *x_vector = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
    double *xptr;
    for (i = 0; i < n; i++) {
        xptr = PyArray_GETPTR2(x_vector, i, 0);
        *xptr = (double)x[i];
    }
    lbfgs_free(x);

    /* Cleanup */
    Py_DECREF(py_callback_eval);
    Py_XDECREF(py_callback_prog);

    /* Build and return the output tuple */
    PyObject *ret = Py_BuildValue("N", x_vector);
    return ret;
}
