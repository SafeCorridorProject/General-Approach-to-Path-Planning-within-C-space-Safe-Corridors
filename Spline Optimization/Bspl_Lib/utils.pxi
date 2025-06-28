from libc.math cimport M_PI, fabs, ceil, sqrt, log

cdef double invphi = 0.5*(sqrt(5.) - 1.)  # 1 / phi
cdef double invphi2 = 0.5*(3. - sqrt(5.))  # 1 / phi^2

cdef str DEF_EXT = ".pyiptp" # default extension when saving to file

cdef extern from "ubsplclib.h" nogil:
    double ATOL
    double RTOL
    int MTX2D(int r, int c, int cols)
    bint CLOSE(double a, double b, double rtol, double atol)
    bint GREQ(double a, double b, double rtol, double atol)
    bint GROF(double a, double b, double rtol, double atol)
    bint MIOF(double a, double b, double rtol, double atol)
    bint MIEQ(double a, double b, double rtol, double atol)
    int SUCCESS
    int MEM_ERR
    int DIV_ZERO_ERR
    int NOT_IMPLEMENTED
    int MAX_ITER
    int OUT_OF_RANGE
    int GENERIC_FAILURE
    void CurvePoint(int *n, int *p, double *U, double *P, double *u, double *C,
                    int *info)

cdef double degToRad(double angleInDegrees) except *:
    return angleInDegrees * M_PI / 180.0

cdef double radToDeg(double angleInRadians) except *:
    return angleInRadians * 180.0 / M_PI

cdef void _check_info(int info):
    """
    C function for error handling of cython libraries.
    
    """
    if info == SUCCESS: # successful termination
        return
    if info < 0:
        raise ValueError(f"the algorithm failed with 'info' value: {info}")
    elif info == MEM_ERR:
        raise MemoryError("memory allocation failed")
    elif info == DIV_ZERO_ERR:
        raise ZeroDivisionError("float division by zero")
    elif info == NOT_IMPLEMENTED:
        raise NotImplementedError
    elif info == MAX_ITER:
        raise Exception("exceeded max iterations")
    elif info == OUT_OF_RANGE:
        raise IndexError("out of range")
    elif info == GENERIC_FAILURE:
        raise Exception("error")

cdef bint isclose(double[::1] arr1, double[::1] arr2, double rtol = RTOL,
                  double atol = ATOL) except? False:
    """
    True if two arrays have the same shape and elements, False otherwise.
    
    """
    cdef Py_ssize_t i
    
    if arr1.shape[0] != arr2.shape[0]:
        return False
    
    for i in range(arr1.shape[0]):
        if not CLOSE(arr1[i], arr2[i], rtol, atol):
            return False
    
    return True

cdef bint issorted(double[::1] arr, int n)  except? False:
    """
    Checks if the array is sorted in strictly ascending order.
    
    Parameters
    ----------
    arr: double C-contiguous array
        Input array.
    n: int
        Array size.
    
    """
    cdef int i
     
    if (n == 0 or n == 1):
        return True
    
    for i in range(1, n): 
        if (arr[i - 1] > arr[i]):
            return False
    
    return True