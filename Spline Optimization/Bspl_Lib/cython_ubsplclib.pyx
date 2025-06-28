# cython: boundscheck=False
# cython: wraparound=False
# cython: nonecheck=False
# cython: infertypes=True
# cython: initializedcheck=False
# cython: cdivision=True
# cython: language_level=3
# distutils: define_macros=NPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION
# coding: utf8

"""
In this module there are some wrapper for bsplclib C-library functions.

All functions           Wrapper for
-------------           -------------------------
validknots              ValidKnotVector
eqspaced                EquallySpaced
averaging               AveragingAlg1
averaging2              AveragingAlg2
averaging3              AveragingAlg3
averagcpts              AveragingCpts
findspan                FindSpan
findspanmult            FindSpanMult
bfunpev                 OneBasisFun
bfuncev                 -
bfunspev                BasisFuns
dbfunspev               DersBasisFuns
splpev                  CurvePoint
splcev                  CurveEval
dsplpev                 CurveDerivsAlg1
dsplcev                 CurveDerivsEval
dsplcpts                CurveDerivCptsAlg1
dsplcpts2               CurveDerivCptsAlg2
asplcpts                CurveAntiDerivCpts
splkins                 CurveKnotIns
splkref                 RefineKnotVectCurve
splkrem                 RemoveCurveKnot
splksrem                RemoveCurveKnots
splkrem_notol           RemoveCurveKnotWithoutTol
getbndrem               GetRemovalBndCurve
degelevc                DegreeElevateCurve
gcsplint                -
gdsplint                -
gadsplint               -
cdsplint                SolveTridiagonal
splcjoin                -
splcsli                 -
knotuniondim            -
splcsum                 -
kevalappr               KnotEvalL2Approx
gcsplapp                -
gdsplapp                -
knotsdegelev            IncreaseMultByOne
bezpev                  PointOnBezierCurve
bezcev                  BezierCurveEval
bezpev2                 deCasteljau1
bezcev2                 BezierCurveEval2
getbmtxelement          GetBezierMatrixElement
getbezmat               GetBezierMatrix
pSparseCSCEval          _PSparseCSCEval
betaki                  _Betaki
aSparseCSCEval          _ASparseCSCEval
linInterpEval           LinInterpEval
concavityCheck          ConcavityCheck
convexityCheck          ConvexityCheck
maxConstraintsOnAxes    _maxConstraintsOnAxes

"""
import numpy as NP
cimport numpy as NP
NP.import_array()

from libc.math cimport fabs
from libc.string cimport memcpy as c_memcpy
from libc.string cimport memmove as c_memmove
from scipy.linalg.cython_lapack cimport dgbsv, dgesv
from scipy.linalg.cython_blas cimport dgemm

__all__=[
    "validknots",
    "eqspaced",
    "averaging",
    "averaging2",
    "averaging3",
    "averagcpts",
    "findspan",
    "findspanmult",
    "bfunpev",
    "bfuncev",
    "bfunspev",
    "dbfunspev",
    "splpev",
    "splcev",
    "dsplpev",
    "dsplcev",
    "dsplcpts",
    "dsplcpts2",
    "asplcpts",
    "splkins",
    "splkref",
    "splkrem",
    "splksrem",
    "splkrem_notol",
    "degelevc",
    "gcsplint",
    "gdsplint",
    "gadsplint",
    "cdsplint",
    "splcjoin",
    "splcsli",
    "knotuniondim",
    "splcsum",
    "kevalappr",
    "gcsplapp",
    "gdsplapp",
    "knotsdegelev",
    "pSparseCSCEval",
    "betaki",
    "aSparseCSCEval",
    "linInterpEval",
    "concavityCheck",
    "convexityCheck",
    ]

cdef extern from "ubsplclib.h" nogil:
    int ValidKnotVector(int *n, int *p, double *U)
    int FindSpan(int *n, int *p, double *u, double *U)
    void BasisFuns(int *i, double *u, int *p, double *U, double *left,
                   double *right, double *N)
    void OneBasisFun(int *p, int *m, double *U, int *i, double *u, double *N,
                     double *Nip)
    void EquallySpaced(int *n, double *x, int *p, double *U)
    void AveragingAlg1(int *n, double *x, int *p, double *U)
    void AveragingAlg2(int *n, double *x, int *p, double *U)
    void AveragingAlg3(int *n, double *x, int *p, int *k, int *l, double *U)
    void KnotsEvalL2Approx(int *nx, double *x, int *p, int *n, double *U)
    void AveragingCpts(int *n, int *p, double *U, double *C, int *info)
    void CurvePoint(int *n, int *p, double *U, double *P, double *u, double *C,
                    int *info)
    void CurveEval(int *n, int *p, double *U, double *P, double *C, int *np,
                   int *info)
    void DersBasisFuns(int *i, double *u, int *p, int *n, double *U,
                       double *ndu, double *a, double *left, double *right,
                       double *ders)
    void CurveDerivsAlg1(int *n, int *p, double *U, double *P, double *u,
                         int *d, double *CK, int *info)
    void CurveDerivsEval(int *n, int *p, double *U, double *P, int *r1,
                         int *r2, int *np, double *C, int *cp, double *CK,
                         int *info)
    void CurveDerivCptsAlg1(int *n, int *p, double *U, double *P, int *d,
                            int *r1, int *r2, double *PK, int *info)
    void FindSpanMult(int *n, int *p, double *u, double *U, int *k, int *s)
    void CurveKnotIns(int *np, int *p, double *UP, double *P, double *u,
                      int *k, int *s, int *r, int *nq, double *UQ, double *Q,
                      int *info)
    void RemoveCurveKnot(int *n, int *p, double *U, double *P, double *u,
                     int *r, int *s, int *num, double *TOL, int *t, int *info)
    void RemoveCurveKnots(int *n, int *p, double *U, double *P, double *TOL,
                          int *gap, int *info)
    void RemoveCurveKnotWithoutTol(int *n, int *p, double *U, double *P,
                       double *u, int *r, int *s, int *num, int *t, int *info)
    void DegreeElevateCurve(int *n, int *p, double *U, double *P, int *t,
                            double *Uh, int *nh, double *Q, int *info)
    void SolveTridiagonal(int *n, double *Q, double *U, double *P, int *info)
    void CurveAntiDerivCpts(int *n, int *p, double *U, double *P, int *d,
                            int *nk, double *UK, double *PK, int *info)
    void CurveDerivCptsAlg2(int *n, int *p, double *U, double *P, int *d,
                            int *nk, double *UK, double *PK, int *info)
    void RefineKnotVectCurve(int *n, int *p, double *U, double *P, double *X,
                             int *r, double *Ubar, double *Q, int *info)
    void IncreaseMultByOne(int *m, int *p, double *U, int *mh, double *Uh)
    
    void _PSparseCSCEval(double *dt, double *K, int *N, double *data,
                         int *row_ind, int *col_ind, int *info)
    void _Betaki(double *t, int *k, int *i, int *p, int *l, double *G, int *m,
                 double *U, double *Bki, int *info)
    void LinInterpEval(int *Nx, double *x, int *Nt, double *t, double *a,
                       int *info)
    int ConcavityCheck(double *c, double *x, int *N, int *conc)
    int ConvexityCheck(double *c, double *x, int *N, int *conv)
    void _ASparseCSCEval(double *G, double *U, int *p, double *dt, double *K,
        int *N, int *d, int *ccc, int *cvc, int *concav_span, int *convex_span,
        double *xconc, double *xconv, double *data, int *row_ind,
        int *col_ind, int *info)

cpdef int concavityCheck(double[::1] c, double[::1] x, int[::1] conc):
    cdef int N = conc.shape[0] - 1
    
    return ConcavityCheck(&c[0], &x[0], &N, &conc[0])

cpdef int convexityCheck(double[::1] c, double[::1] x, int[::1] conv):
    cdef int N = conv.shape[0] - 1
    
    return ConvexityCheck(&c[0], &x[0], &N, &conv[0])

cpdef void linInterpEval(double[::1] x, double[::1] t, double[::1] a):
    cdef int Nx = x.shape[0] - 1
    cdef int Nt = t.shape[0] - 1
    cdef int info
    
    assert a.shape[0] == Nt + 1
    
    LinInterpEval(&Nx, &x[0], &Nt, &t[0], &a[0], &info)
    _check_info(info)

cpdef void aSparseCSCEval(double[::1] G, double[::1] U, int p, double[::1] dt,
          double[::1] K, int[::1] concav_span, int[::1] convex_span,
          double[::1] xconc, double[::1] xconv, double[::1] data,
          int[::1] row_ind, int[::1] col_ind):
    cdef int d = 3
    cdef int ccc = concav_span.shape[0]
    cdef int cvc = convex_span.shape[0]
    cdef int info, N = dt.shape[0] - 1
    
    _ASparseCSCEval(&G[0], &U[0], &p, &dt[0], &K[0], &N, &d, &ccc, &cvc,
                    &concav_span[0], &convex_span[0], &xconc[0], &xconv[0],
                    &data[0], &row_ind[0], &col_ind[0], &info)
    _check_info(info)

cpdef double betaki(double t, int k, int i, int p, int l, double[::1] G,
                    double[::1] U):
    cdef double Bki
    cdef int info, m = U.shape[0] - 1
    
    _Betaki(&t, &k, &i, &p, &l, &G[0], &m, &U[0], &Bki, &info)
    _check_info(info)
    
    return Bki

cpdef void pSparseCSCEval(double[::1] dt, double[::1] K, double[::1] data,
                          int[::1] row_ind, int[::1] col_ind):
    
    cdef int info, N = dt.shape[0] - 1
    
    _PSparseCSCEval(&dt[0], &K[0], &N, &data[0], &row_ind[0], &col_ind[0],
                    &info)
    _check_info(info)

cpdef bint validknots(int p, double[::1] U):
    """
    Wrapper for 'ValidKnotVector' function of bsplclib C-library.
    
    Parameters
    ----------
    p: int
        Degree of the B-Spline.
    U: double C-contiguous array
        Knot vector.
    
    Returns
    -------
    out: bint
        0 if check passed, 1 otherwise.
    
    """
    cdef int n = U.shape[0] - p - 2
    cdef bint out
    
    out = <bint>ValidKnotVector(&n, &p, &U[0])
    return out

cpdef void eqspaced(double[::1] xp, int p, double[::1] U):
    """
    Wrapper for 'EquallySpaced' function of bsplclib C-library.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    p: int
        Degree of the B-Spline.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector.
    
    """
    cdef int n = xp.shape[0] - 1
    
    EquallySpaced(&n, &xp[0], &p, &U[0])

cpdef void averaging(double[::1] xp, int p, double[::1] U):
    """
    Wrapper for 'AveragingAlg1' function of bsplclib C-library.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    p: int
        Degree of the B-Spline.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector (size m, with m = n + p + 1 and n the highest index
        of 'xp' array).
    
    """
    cdef int n = xp.shape[0] - 1
    
    AveragingAlg1(&n, &xp[0], &p, &U[0])

cpdef void averaging2(double[::1] xp, int p, double[::1] U):
    """
    Wrapper for 'AveragingAlg2' function of bsplclib C-library.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    p: int
        Degree of the B-Spline.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector (size m, with m = n + p + 3 and n the highest index
        of 'xp' array).
    
    """
    cdef int n = xp.shape[0] - 1
    
    AveragingAlg2(&n, &xp[0], &p, &U[0])

cpdef void averaging3(double[::1] xp, int p, int k, int l, double[::1] U):
    """
    Wrapper for 'AveragingAlg3' function of bsplclib C-library.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    p: int
        Degree of the B-Spline.
    k: int
        Number of derivative constraints at left end.
    l: int
        Number of derivative constraints at right end.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector (size len(xp) + k + l + p + 1,  with n the highest index
        of 'xp' array).
    
    """
    cdef int n = xp.shape[0] - 1
    
    AveragingAlg3(&n, &xp[0], &p, &k, &l, &U[0])

cdef int distintknots(double[::1] U, int p):
    """
    Returns the number of distinct internal knots.
    """
    cdef Py_ssize_t i
    cdef int s
    
    s = 0
    for i in range(p + 1, U.shape[0] - p - 1):
        if U[i] != U[i - 1]:
            s += 1
    return s

cpdef void kevalappr(double[::1] xp, int p, double[::1] U):
    """
    Wrapper for 'KnotsEvalL2Approx' function of bsplclib C-library.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    p: int
        Degree of the B-Spline.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector.
    
    """
    cdef int m = xp.shape[0] - 1
    cdef int n = U.shape[0] - p - 2
    
    KnotsEvalL2Approx(&m, &xp[0], &p, &n, &U[0])

cpdef void gcsplint(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P):
    """
    Global Curve Spline Interpolation to point data.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    fp: double C-contiguous array
        The y-coordinates of the data points, same length as xp.
    p: int
        Degree of the B-Spline.
    U: double C-contiguous array
        Knot vector.
    
    Returns
    -------
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    
    """
    cdef:
        Py_ssize_t i, j
        int span
        int n = xp.shape[0] - 1
        double[::1] left = NP.empty(p + 1, dtype = NP.float64)
        double[::1] right = NP.empty(p + 1, dtype = NP.float64)
        double[::1] M = NP.empty(p + 1, dtype = NP.float64)
        int info, N = n + 1, kl = p - 1, ku = p - 1, nrhs = 1
        int ldab = 2*kl + ku + 1, ldb = N
        int[::1] ipiv = NP.empty(N, dtype = NP.intc)
        double[::1,:] AB = NP.zeros((ldab, N), dtype=NP.float64, order='F')
    
    assert n == fp.shape[0] - 1, "xp and fp must have same dimension"
    assert n == P.shape[0] - 1, "P dimension must be: len(xp)"
    assert p > 1, "p must be greater than 1"
    
    # LAPACK band storage scheme -> AB(KL + KU + 1 + i - j, j) = A(i, j)
    #
    # NOTE: This formula (reported in www.netlib.org/lapack/lug/node124.html)
    # refers to an indexing starting from 1 (default in Fortran), but in our
    # case the indexing starts from 0. Therefore the right formula is:
    #
    #                   AB(KL + KU + i - j, j) = A(i, j)
    #
    with nogil:
        AB[kl + ku][0] = 1.; AB[kl + ku][n] = 1.
        for i in range(1, n):
            span = FindSpan(&n, &p, &xp[i], &U[0])
            BasisFuns(&span, &xp[i], &p, &U[0], &left[0], &right[0], &M[0])
            
            for j in range(p + 1):
                # row: i;   col: span - p + j
                AB[kl + ku + i - span + p - j][span - p + j] = M[j]
        
        c_memcpy(&P[0], &fp[0], (n + 1)*sizeof(double))
        
        dgbsv(&N, &kl, &ku, &nrhs, &AB[0][0], &ldab, &ipiv[0], &P[0], &ldb,
              &info)
    
    if info:
        raise Exception("'dgesv' LAPACK function failed with 'info' "
                        "value: {}".format(info))

cpdef void gdsplint(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P, (double, double) dy = (0., 0.)):
    """
    Global curve Interpolation with end Derivatives specified.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    fp: double C-contiguous array
        The y-coordinates of the data points, same length as xp.
    p: int
        Degree of the B-Spline.
    dy: two-elements ctuple
        The slope of the first and end point.
    U: double C-contiguous array
        Knot vector.
    
    Returns
    -------
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    
    """
    cdef:
        Py_ssize_t i, j
        int span
        int n = xp.shape[0] - 1
        int m = U.shape[0] - 1
        int u = n + 2 # the highest index of control points
        double[::1] left = NP.empty(p + 1, dtype = NP.float64)
        double[::1] right = NP.empty(p + 1, dtype = NP.float64)
        double[::1] M = NP.empty(p + 1, dtype = NP.float64)
        int info, N = n + 3, kl = p - 1, ku = p - 1, nrhs = 1
        int ldab = 2*kl + ku + 1, ldb = N
        int[::1] ipiv = NP.empty(N, dtype = NP.intc)
        double[::1,:] AB = NP.zeros((ldab, N), dtype = NP.float64, order='F')
    
    assert n == fp.shape[0] - 1, "xp and fp must have same dimension"
    assert u == P.shape[0] - 1, "P dimension must be: len(xp) + 2"
    assert p > 1, "p must be greater than 1"
    
    # LAPACK band storage scheme -> AB(KL + KU + 1 + i - j, j) = A(i, j)
    #
    # NOTE: This formula (reported in www.netlib.org/lapack/lug/node124.html)
    # refers to an indexing starting from 1 (default in Fortran), but in our
    # case the indexing starts from 0. Therefore the right formula is:
    #
    #                   AB(KL + KU + i - j, j) = A(i, j)
    #
    with nogil:
        AB[kl + ku][0] = 1.; AB[kl + ku][n + 2] = 1.
        AB[kl + ku + 1][0] = -1.; AB[kl + ku][1] = 1.
        AB[kl + ku][n + 1] = -1.; AB[kl + ku - 1][n + 2] = 1.
        
        for i in range(1, n):
            span = FindSpan(&u, &p, &xp[i], &U[0])
            BasisFuns(&span, &xp[i], &p, &U[0], &left[0], &right[0], &M[0])
            
            for j in range(p + 1):
                # row: i + 1;   col: span - p + j
                AB[kl + ku + 1 + i - span + p - j][span - p + j] = M[j]
        
        P[0] = fp[0]; P[n + 2] = fp[n]
        P[1] = (U[p + 1] - xp[0])*dy[0]/p
        P[n + 1] = (xp[n] - U[m - p - 1])*dy[1]/p
        c_memcpy(&P[2], &fp[1], (n - 1)*sizeof(double))
        
        dgbsv(&N, &kl, &ku, &nrhs, &AB[0][0], &ldab, &ipiv[0], &P[0], &ldb,
              &info)
    
    if info:
        raise Exception("'dgesv' LAPACK function failed with 'info' "
                        "value: {}".format(info))

cpdef void gadsplint(double[::1] xp, double[::1] fp, int p, double[::1] Ds,
                     double[::1] De, double[::1] U, double[::1] P):
    """
    Global cuve Iterpolation with Arbitrary End Derivatives.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    fp: double C-contiguous array
        The y-coordinates of the data points, same length as xp.
    p: int
        Degree of the B-Spline.
    Ds: double C-contiguous array
        Derivates at the start point.
    De: double C-contiguous array
        Derivates at the end point.
    U: double C-contiguous array
        Knot vector.
    
    Returns
    -------
    P: double C-contiguous array
        Control Points (or B-splines coefficients, size len(xp) + len(Ds) +
        len(De))
    
    References
    ----------
    [1] Piegl, L. & Tiller, W.. (2000). Curve Interpolation with Arbitrary
        End Derivatives. Engineering with Computers. 16. 73-79.
    
    """
    cdef:
        Py_ssize_t i, j
        int n = xp.shape[0] - 1
        int k = Ds.shape[0] # the maximum constrained derivative on left
        int l = De.shape[0] # the maximum constrained derivative on right
        int d = max(k, l) # the maximum derivative order requested
        int m = U.shape[0] - 1
        int u = n + k + l # the highest index of control points
        int span
        double[:,::1] ndu = NP.empty((p + 1, p + 1), dtype = NP.float64)
        double[:,::1] a = NP.empty((2, p + 1), dtype = NP.float64)
        double[:,::1] nders = NP.empty((d + 1, p + 1), dtype = NP.float64)
        double[::1] left = NP.empty(p + 1, dtype = NP.float64)
        double[::1] right = NP.empty(p + 1, dtype = NP.float64)
        double[::1] M = NP.empty(p + 1, dtype = NP.float64)
        int info, N = u + 1, kl = p - 1, ku = p - 1, nrhs = 1
        int ldab = 2*kl + ku + 1, ldb = N
        int[::1] ipiv = NP.empty(N, dtype = NP.intc)
        double[::1,:] AB = NP.zeros((ldab, N), dtype = NP.float64, order='F')
    
    assert n == fp.shape[0] - 1, "xp and fp must have same dimension"
    assert u == P.shape[0] - 1, (
        "P dimension must be: len(xp) + len(Ds) + len(De)")
    assert p > 1, "p must be greater than 1"
    assert d < p, (
        "the maximum constrained derivative must be less than 'p'")
    
    # LAPACK band storage scheme -> AB(KL + KU + 1 + i - j, j) = A(i, j)
    #
    # NOTE: This formula (reported in www.netlib.org/lapack/lug/node124.html)
    # refers to an indexing starting from 1 (default in Fortran), but in our
    # case the indexing starts from 0. Therefore the right formula is:
    #
    #                   AB(KL + KU + i - j, j) = A(i, j)
    #
    with nogil:
        AB[kl + ku][0] = 1.; AB[kl + ku][u] = 1.
        span = p
        DersBasisFuns(&span, &xp[0], &p, &k, &U[0], &ndu[0][0], &a[0][0],
                      &left[0], &right[0], &nders[0][0])
        
        for i in range(k):
            for j in range(i + 2):
                # rows -> from 1 to k;  cols -> from 0 to k
                AB[kl + ku + i + 1 - j][j] = nders[i + 1][j]
        
        for i in range(1, n):
            span = FindSpan(&u, &p, &xp[i], &U[0])
            BasisFuns(&span, &xp[i], &p, &U[0], &left[0], &right[0], &M[0])
            
            for j in range(p + 1):
                # row: i + k;   col: span - p + j
                AB[kl + ku + k + i - span + p - j][span - p + j] = M[j]
        
        span = u
        DersBasisFuns(&span, &xp[n], &p, &l, &U[0], &ndu[0][0], &a[0][0],
                      &left[0], &right[0], &nders[0][0])
        
        for i in range(l):
            for j in range(i + 2):
                # rows -> from (u - 1) to (u - l); cols -> from u to (u - l)
                AB[kl + ku - i - 1 + j][u - j] = nders[i + 1][p - j]
        
        P[0] = fp[0]; P[u] = fp[n]
        c_memcpy(&P[1], &Ds[0], k*sizeof(double))
        c_memcpy(&P[1 + k], &fp[1], (n - 1)*sizeof(double))
        
        for i in range(l):
            P[n + k + i] = De[l - 1 - i]
        
        dgbsv(&N, &kl, &ku, &nrhs, &AB[0][0], &ldab, &ipiv[0], &P[0], &ldb,
              &info)
    
    if info:
        raise Exception("'dgesv' LAPACK function failed with 'info' "
                        "value: {}".format(info))

cpdef void cdsplint(double[::1] xp, double[::1] fp, double[::1] U,
                    double[::1] P, (double, double) dy = (0., 0.)):
    """
    Cubic Spline Interpolation with end Derivatives specified. Wrapper for
    'SolveTridiagonal' function of bsplclib C-library.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    fp: double C-contiguous array
        The y-coordinates of the data points, same length as xp.
    dy: two-elements ctuple
        The slope of the first and end point.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    
    """
    cdef:
        Py_ssize_t i
        int info, n = xp.shape[0] - 1
        double xp0, xpn
    
    assert n == fp.shape[0] - 1, "xp and fp must have same dimension"
    assert n == P.shape[0] - 3, "P dimension must be: len(xp) + 2"
    
    with nogil:
        xp0, xpn = xp[0], xp[n]
        for i in range(4):
            U[i] = xp0; U[n + 3 + i] = xpn
        
        c_memcpy(&U[4], &xp[1], n*sizeof(double))
        
        P[0] = fp[0]; P[n + 2] = fp[n]
        P[1] = P[0] + (U[4] - xp[0])*dy[0]/3.
        P[n + 1] = P[n + 2] - (xp[n] - U[n + 2])*dy[1]/3.
        
        SolveTridiagonal(&n, &fp[0], &U[0], &P[0], &info)
    _check_info(info)

cpdef void gcsplapp(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P):
    """
    Least squares curve approximation.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    fp: double C-contiguous array
        The y-coordinates of the data points, same length as xp.
    p: int
        Degree of the B-Spline.
    U: double C-contiguous array
        Knot vector.
    
    Returns
    -------
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    
    References
    ----------
    [1] Piegl, L. A., & Tiller, W. (1997). The NURBS book. Berlin: Springer.
        pp. 410-413.
    
    """
    cdef:
        Py_ssize_t i, j
        int span
        int u = U.shape[0] - 1, n = P.shape[0] - 1, m = xp.shape[0] - 1
        double fp0, fpm
        double N0p, Nnp, Njp
        char* transa = 't'
        char* transb = 'n'
        int M = n - 1, N = n - 1, K = m - 1
        int lda = K, ldb = K, ldc = M
        int info, nrhs = 1
        double alpha = 1., beta = 0.
        double[::1] _N = NP.empty(p + 1, dtype = NP.float64)
        double[::1] R = NP.empty(K, dtype = NP.float64)
        double[::1,:] A = NP.empty((K, N), dtype = NP.float64, order='F')
        double[::1,:] ATA = NP.empty((N, N),dtype = NP.float64, order='F')
        int[::1] ipiv = NP.empty(N, dtype=NP.intc)
    
    assert m == fp.shape[0] - 1, "xp and fp must have same dimension"
    assert p >= 1, "p must be greater than or equal to 1"
    assert n >= p, "n must be greater than or equal to p"
    assert m > n, ("the number of points to be approximated must be greater "
                   "than the number of control points")
    with nogil:
        span = 0
        fp0, fpm = fp[0], fp[m]
        for i in range(1, m):
            OneBasisFun(&p, &u, &U[0], &span, &xp[i], &_N[0], &N0p)
            OneBasisFun(&p, &u, &U[0], &n, &xp[i], &_N[0], &Nnp)
            R[i - 1] = fp[i] - N0p*fp0 - Nnp*fpm
        
        for j in range(1, n):
            P[j] = 0.
            span = <int>j
            for i in range(1, m):
                OneBasisFun(&p, &u, &U[0], &span, &xp[i], &_N[0], &Njp)
                P[j] += Njp * R[i - 1]
        
        for i in range(1, m): # iterate through rows
            for j in range(1, n): # iterate through columns
                span = <int>j
                OneBasisFun(
                    &p, &u, &U[0], &span, &xp[i], &_N[0],&A[i - 1][j - 1])
        
        dgemm(transa, transb, &M, &N, &K, &alpha, &A[0][0], &lda, &A[0][0],
              &ldb, &beta, &ATA[0][0], &ldc)
        
        P[0] = fp0; P[n] = fpm
        dgesv(&N, &nrhs, &ATA[0][0], &N, &ipiv[0], &P[1], &N, &info)
        
    if info:
        raise Exception("'dgesv' LAPACK function failed with 'info' "
                        "value: {}".format(info))

cpdef void gdsplapp(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P, (double, double) dy = (0., 0.)):
    """
    Least squares curve approximation with fixed endpoint derivatives.
    
    Parameters
    ----------
    xp: double C-contiguous array
        The x-coordinates of the data points, must be increasing.
    fp: double C-contiguous array
        The y-coordinates of the data points, same length as xp.
    p: int
        Degree of the B-Spline.
    U: double C-contiguous array
        Knot vector.
    dy: two-elements ctuple
        The slope of the first and end point.
    
    Returns
    -------
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    
    """
    cdef:
        Py_ssize_t i, j
        int span
        int u = U.shape[0] - 1, n = P.shape[0] - 1, m = xp.shape[0] - 1
        double fp0, fp1, fpm1, fpm
        double N0p, N1p, Nn1p, Nnp, Njp
        char* transa = 't'
        char* transb = 'n'
        int M = n - 3, N = n - 3, K = m - 1
        int lda = K, ldb = K, ldc = M
        int info, nrhs = 1
        double alpha = 1., beta = 0.
        double[::1] _N = NP.empty(p + 1, dtype = NP.float64)
        double[::1] R = NP.empty(K, dtype = NP.float64)
        double[::1,:] A = NP.empty((K, N), dtype = NP.float64, order='F')
        double[::1,:] ATA = NP.empty((N, N),dtype = NP.float64,order='F')
        int[::1] ipiv = NP.empty(N, dtype = NP.intc)
    
    assert m == fp.shape[0] - 1, "xp and fp must have same dimension"
    assert p >= 1, "p must be greater than or equal to 1"
    assert n >= p, "n must be greater than or equal to p"
    assert m > n, ("the number of points to be approximated must be greater "
                   "than the number of control points")
    with nogil:
        span = 0
        fp0, fp1, fpm1, fpm = fp[0], fp[1], fp[m - 1], fp[m]
        for i in range(1, m):
            OneBasisFun(&p, &u, &U[0], &span, &xp[i], &_N[0], &N0p)
            span += 1
            OneBasisFun(&p, &u, &U[0], &span, &xp[i], &_N[0], &N1p)
            span = n - 1
            OneBasisFun(&p, &u, &U[0], &span, &xp[i], &_N[0], &Nn1p)
            span += 1
            OneBasisFun(&p, &u, &U[0], &span, &xp[i], &_N[0], &Nnp)
            
            R[i - 1] = (fp[i] - N0p*fp0 - N1p*fp1 - Nn1p*fpm1 - Nnp*fpm)
        
        for j in range(2, n - 1):
            P[j] = 0.
            span = <int>j
            for i in range(1, m):
                OneBasisFun(&p, &u, &U[0], &span, &xp[i], &_N[0], &Njp)
                P[j] += Njp * R[i - 1]
        
        for i in range(1, m): # iterate through rows
            for j in range(2, n - 1): # iterate through columns
                span = <int>j
                OneBasisFun(
                    &p, &u, &U[0], &span, &xp[i], &_N[0],&A[i - 1][j - 2])
        
        dgemm(transa, transb, &M, &N, &K, &alpha, &A[0][0], &lda, &A[0][0],
              &ldb, &beta, &ATA[0][0], &ldc)
        
        P[0] = fp0; P[n] = fpm
        P[1] = fp0 + (U[p + 1] - xp[0])*dy[0]/p
        P[n - 1] = fpm - (xp[m] - U[n])*dy[1]/p
        dgesv(&N, &nrhs, &ATA[0][0], &N, &ipiv[0], &P[2], &N, &info)
    
    if info:
        raise Exception("'dgesv' LAPACK function failed with 'info' "
                        "value: {}".format(info))

cpdef void averagcpts(double[::1] U, int p, double [::1] C):
    """
    Wrapper for 'AveragingCpts' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector.
    p: int
        Degree of the basis functions.
    
    Returns
    -------
    C: double C-contiguous array
        Result of the 'averaging' formula.
    
    """
    cdef int info, n = C.shape[0] - 1
    
    AveragingCpts(&n, &p, &U[0], &C[0], &info)
    _check_info(info)

cpdef int findspan(int p, double u, double[::1] U):
    """
    Wrapper for 'FindSpan' function of bsplclib C-library.
    
    Parameters
    ----------
    p: int
        Degree of the basis functions.
    u: double
        Indipendent variable (x-value).
    U: double C-contiguous array
        Knot vector.
    
    Returns
    -------
    span: int
        Span index.
    
    """
    cdef int info = 0, n = U.shape[0] - p - 2
    
    if p < 0:
        info = -1
    elif validknots(p, U):
        info = -3
    
    _check_info(info)
    
    span = FindSpan(&n, &p, &u, &U[0])
    return span

cpdef (int, int) findspanmult(int p, double u, double[::1] U):
    """
    Wrapper for 'FindSpanMult' function of bsplclib C-library.
    
    Parameters
    ----------
    p: int
        Degree of the basis functions.
    u: double
        Indipendent variable (x-value).
    U: double C-contiguous array
        Knot vector.
    
    Returns
    -------
    span: int
        Span index.
    mult: int
        Multiplicity.
    
    """
    cdef int info = 0, n = U.shape[0] - p - 2
    cdef int span, mult
    
    if p < 0:
        info = -1
    elif validknots(p, U):
        info = -3
    
    _check_info(info)
    
    FindSpanMult(&n, &p, &u, &U[0], &span, &mult)
    return span, mult

cpdef double bfunpev(double u, int span, double[::1] U, int p):
    """
    Basis function point evaluation.
    
    Parameters
    ----------
    u: double
        Indipendent variable (x-value).
    span: int
        Span index.
    U: double C-contiguous array
        Knot vector.
    p: int
        Degree of the basis functions.
    
    Returns
    -------
    Nip: double
        Basis function evaluation of 'u'.
    
    """
    cdef int m = U.shape[0] - 1
    cdef double[::1] N = NP.empty(p + 1, dtype = NP.float64)
    cdef double Nip
    
    OneBasisFun(&p, &m, &U[0], &span, &u, &N[0], &Nip)
    return Nip

cpdef void bfuncev(double[::1] C, int span, double[::1] U, int p):
    """
    Basis function curve evaluation.
    
    Parameters
    ----------
    C: double C-contiguous array
        x-coordinates.
    span: int
        Span index.
    U: double C-contiguous array
        Knot vector.
    p: int
        Degree of the basis functions.
    
    Returns
    -------
    C: double C-contiguous array
        Basis function evaluation in the values of the input array 'C'.
    
    Notes
    -----
    The C array is overwritten with the output values, you need to make a copy.
    
    """
    cdef Py_ssize_t i
    cdef int m = U.shape[0] - 1, n = m - p - 1
    cdef double[::1] N = NP.empty(p + 1, dtype = NP.float64)
    
    with nogil:
        for i in range(C.shape[0]):
            OneBasisFun(&p, &m, &U[0], &span, &C[i], &N[0], &C[i])

cpdef void bfunspev(double u, int span, double[::1] U, int p, double[::1] N):
    """
    Wrapper for 'BasisFuns' function of bsplclib C-library.
    
    Parameters
    ----------
    u: double
        Indipendent variable (x-value).
    span: int
        Span index.
    U: double C-contiguous array
        Knot vector.
    p: int
        Degree of the basis functions.
    
    Returns
    -------
    N: double C-contiguous array.
        Must have (p + 1) size. N[i] is the b-fun N_(span - p + i, p)(u) with
        0 <= i <= p.
    
    """
    cdef:
        double[::1] left = NP.empty(p + 1, dtype = NP.float64)
        double[::1] right = NP.empty(p + 1, dtype = NP.float64)
    
    BasisFuns(&span, &u, &p, &U[0],&left[0], &right[0], &N[0])

cpdef void dbfunspev(double u, int span, double[::1] U, int p, int k,
                     double[:,::1] nders):
    """
    Wrapper for 'DersBasisFuns' function of bsplclib C-library.
    
    Parameters
    ----------
    u: double
        Indipendent variable (x-value).
    span: int
        Span index.
    U: double C-contiguous array
        Knot vector.
    p: int
        Degree of the basis functions.
    k: int
        The highest derivative order calculated.
    
    Returns
    -------
    nders: double C-contiguous 2D array
        Must have (k + 1, p + 1) size. nders[i][j] is the i-th derivative of
        the function N(span - p + j, p), where 0 <= i <= k and 0 <= j <= p.
    
    """
    cdef:
        double[::1] left = NP.empty(p + 1, dtype = NP.float64)
        double[::1] right = NP.empty(p + 1, dtype = NP.float64)
        double[:,::1] ndu = NP.empty((p + 1, p + 1), dtype = NP.float64)
        double[:,::1] a = NP.empty((2, p + 1), dtype = NP.float64)
    
    DersBasisFuns(&span, &u, &p, &k, &U[0], &ndu[0][0], &a[0][0], &left[0],
                  &right[0], &nders[0][0])

cpdef double splpev(double u, double[::1] U, double[::1] P, int p):
    """
    Wrapper for 'CurvePoint' function of bsplclib C-library.
    
    Parameters
    ----------
    u: double
        Indipendent variable (x-value).
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    
    Returns
    -------
    C: double
        B-spline evaluation of 'u'.
    
    """
    cdef double C
    cdef int info, n = P.shape[0] - 1
    
    CurvePoint(&n, &p, &U[0], &P[0], &u, &C, &info)
    _check_info(info)
    return C

cpdef void splcev(double[::1] C, double[::1] U, double[::1] P, int p):
    """
    Wrapper for 'CurveEval' function of bsplclib C-library.
    
    Parameters
    ----------
    C: double C-contiguous array
        x-coordinates.
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    
    Returns
    -------
    C: C-contiguous array
        B-spline evaluation in the values of the input array 'C'.
    
    Notes
    -----
    The C array is overwritten with the output values, you need to make a copy.
    
    """
    cdef int info, n = P.shape[0] - 1, np = C.shape[0] - 1
    
    CurveEval(&n, &p, &U[0], &P[0], &C[0], &np, &info)
    _check_info(info)

cpdef void dsplpev(double u, double[::1] U, double[::1] P, int p, int d,
                   double[::1] CK):
    """
    Wrapper for 'CurveDerivsAlg1' function of bsplclib C-library. The result is
    stored in the array CK[0], ..., CK[d].
    
    Parameters
    ----------
    u: double
        Indipendent variable (x-value).
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    d: int
        Max derivative degree (CK lenght is d + 1).
    
    Returns
    -------
    CK: double C-contiguous array
        The result is stored in the CK array so that CK[k] is the k-th
        derivative computed in 'u'.
    
    """
    cdef int info, n = P.shape[0] - 1
    
    CurveDerivsAlg1(&n, &p, &U[0], &P[0], &u, &d, &CK[0], &info)
    _check_info(info)

cpdef void dsplcev(double [::1] C, double[::1] U, double[::1] P, int p,
                   int r1, int r2, double [:, ::1] CK):
    """
    Wrapper for 'CurveDerivsEval' function of bsplclib C-library. The result is
    stored in the 2D array CK.
    
    Parameters
    ----------
    C: double C-contiguous array
        x-coordinates.
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    r1: int
        Minimum derivative order calculated.
    r2: int
        Maximum derivative order calculated.
    
    Returns
    -------
    CK: double C-contiguous 2D array
        The result is stored in the CK array so that CK[k][i] is the j-th
        derivative of the i-th C value, where j = k + r1.
    
    """
    cdef:
        int info
        int n = P.shape[0] - 1
        int np = C.shape[0] - 1
        int cp = CK.shape[0] - 1
    
    CurveDerivsEval(&n, &p, &U[0], &P[0], &r1, &r2, &np, &C[0], &cp,
                    &CK[0][0], &info)
    _check_info(info)

cpdef void dsplcpts(double[::1] U, double[::1] P, int p, int d, int r1, int r2,
                    double [:, ::1] PK):
    """
    Wrapper for 'CurveDerivCptsAlg1' function of bsplclib C-library. The result
    is stored in the PK array so that PK[k][i] is the i-th control point of the
    kth derivative curve, where 0 <= k <= d and r1 <= i <= (r2 - k). If r1 = 0
    and r2 = n, all control points are computed.
    
    For example, let   d = 2, p = 3, n = 6, r1 = 2, r2 = 6
    
    the PK array looks like:
    
        PK = [ P(0)(2)   P(0)(3)   P(0)(4)   P(0)(5)   P(0)(6)
               P(1)(2)   P(1)(3)   P(1)(4)   P(1)(5)      -
               P(2)(2)   P(2)(3)   P(2)(4)      -         -   ]
    
    where the elements merked - in the lower right corner are not referenced by
    the routine.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    d: int
        The highest derivative order calculated.
    r1: int
        Extreme left index of control points computed by the algorithm.
    r2: int
        As 'r1', but right index (see above).
    
    Returns
    -------
    PK: double C-contiguous 2D array
        The result is stored in the PK array so that PK[k][i] is the i-th
        control point of the kth derivative curve.
    
    """
    cdef int info, n = P.shape[0] - 1
    
    CurveDerivCptsAlg1(&n, &p, &U[0], &P[0], &d, &r1, &r2, &PK[0][0], &info)
    _check_info(info)

cpdef void splkins(double[::1] UP, double[::1] P, int p, double u, int r,
                   double[::1] UQ, double[::1] Q):
    """
    Wrapper for 'CurveKnotIns' function of bsplclib C-library. The knot vector
    and control points after knot insertion are stored in UQ and Q arrays.
    
    Parameters
    ----------
    UP: double C-contiguous array
        Knot vector before knot insertion.
    P: double C-contiguous array
        Control Points before knot insertion (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    u: double
        Knot to insert.
    r: int
        Number of times the knot must be inserted.
    
    Returns
    -------
    UQ: double C-contiguous array
        Knot vector after knot insertion.
    Q: double C-contiguous array
        Control Points after knot insertion (or B-splines coefficients).
    
    """
    cdef:
        int info
        int k, s
        int np = P.shape[0] - 1
        int nq = Q.shape[0] - 1
    
    FindSpanMult(&np, &p, &u, &UP[0], &k, &s)
    CurveKnotIns(&np, &p, &UP[0], &P[0], &u, &k, &s, &r, &nq, &UQ[0], &Q[0],
                 &info)
    _check_info(info)

cpdef int splkrem(double[::1] U, double[::1] P, int p, double u, int num,
                  double tol = 1e-11):
    """
    Wrapper for 'RemoveCurveKnot' function of bsplclib C-library. It returns
    't', the actual number of times the knot is removed. One knot removal
    results in a curve whose deviation from the original curve is less than
    'tol' on the entire parameter domain.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector before knot remove.
    P: double C-contiguous array
        Control Points before knot remove (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    u: double
        Knot to remove.
    num: int
        Number of times the knot must be removed.
    tol: double
        Tolerance. Default 1e-11.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector after knot remove.
    P: double C-contiguous array
        Control Points after knot remove (or B-splines coefficients).
    t: int
        The actual number of times the knot is removed.
    
    Notes
    -----
    The U and P arrays are overwritten with the output values, you need to
    make a copy.
    
    """
    cdef:
        int info
        int r, s, t
        int n = P.shape[0] - 1
    
    FindSpanMult(&n, &p, &u, &U[0], &r, &s)
    RemoveCurveKnot(&n, &p, &U[0], &P[0], &u, &r, &s, &num, &tol, &t, &info)
    _check_info(info)
    return t

cpdef int splksrem(double[::1] U, double[::1] P, int p, double tol = 1e-11):
    """
    Wrapper for 'RemoveCurveKnots' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector before knot remove.
    P: double C-contiguous array
        Control Points before knot remove (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    tol: double
        Tolerance. Default 1e-11.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector after knot remove.
    P: double C-contiguous array
        Control Points after knot remove (or B-splines coefficients).
    num: int
        Number of knots removed.
    
    Notes
    -----
    The U and P arrays are overwritten with the output values, you need to
    make a copy.
    
    """
    cdef:
        int info, num
        int n = P.shape[0] - 1
    
    RemoveCurveKnots(&n, &p, &U[0], &P[0], &tol, &num, &info)
    _check_info(info)
    return num

cpdef int splkrem_notol(double[::1] U, double[::1] P, int p, double u,int num):
    """
    Wrapper for 'RemoveCurveKnotWithoutTol' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector before knot remove.
    P: double C-contiguous array
        Control Points before knot remove (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    u: double
        Knot to remove.
    num: int
        Number of times the knot must be removed.
    
    Returns
    -------
    U: double C-contiguous array
        Knot vector after knot remove.
    P: double C-contiguous array
        Control Points after knot remove (or B-splines coefficients).
    t: int
        The actual number of times the knot is removed.
    
    Notes
    -----
    The U and P arrays are overwritten with the output values, you need to
    make a copy.
    
    """
    cdef:
        int info, r, s, t
        int n = P.shape[0] - 1
    
    FindSpanMult(&n, &p, &u, &U[0], &r, &s)
    RemoveCurveKnotWithoutTol(
        &n, &p, &U[0], &P[0], &u, &r, &s, &num, &t, &info)
    _check_info(info)
    return t

cpdef void degelevc(double[::1] U, double[::1] P, int p, int t, double[::1] Uh,
                    double[::1] Q):
    """
    Wrapper for 'DegreeElevateCurve' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector before degree elevation.
    P: double C-contiguous array
        Control Points before degree elevation (or B-splines coefficients).
    p: int
        Degree of the basis functions.
    t: int
        Number of degree elevation times.
    
    Returns
    -------
    Uh: double C-contiguous array
        Knot vector after degree elevation. Must be lenght
        U.shape[0] + t*(s + 2), where s = U.shape[0] - 2*(p + 1) numbers of
        internal knots.
    
    Q: double C-contiguous array
        Control Points after degree elevation (or B-splines coefficients). Must
        be lenght P.shape[0] + t*(s + 1), where s = U.shape[0] - 2*(p + 1).
    
    """
    cdef int info, nh, n = P.shape[0] - 1
    
    DegreeElevateCurve(&n, &p, &U[0], &P[0], &t, &Uh[0], &nh, &Q[0], &info)
    
    assert nh == Q.shape[0] - 1
    assert Uh.shape[0] - 1 == nh + (p + t) + 1
    
    _check_info(info)

cpdef void asplcpts(double[::1] U, double[::1] P, int p, int d, double[::1] UK,
                    double[::1] PK):
    """
    Wrapper for 'CurveAntiDerivCpts' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the curve to be integrated (initial degree).
    d: int
        Antiderivative order.
    
    Returns
    -------
    UK: double C-contiguous array
        New knot vector.
    PK: double C-contiguous array
        Control points after the integration.
    p: int
        New degree curve (p + d).
    
    """
    cdef int info, n = P.shape[0] - 1, nk = PK.shape[0] - 1
    
    CurveAntiDerivCpts(&n, &p, &U[0], &P[0], &d, &nk, &UK[0], &PK[0], &info)
    _check_info(info)

cpdef int dsplcpts2(double[::1] U, double[::1] P, int p, int d, double[::1] UK,
                    double[::1] PK):
    """
    Wrapper for 'CurveDerivCptsAlg2' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the curve to be derived (initial degree).
    d: int
        Derivative order.
    
    Returns
    -------
    UK: double C-contiguous array
        New knot vector.
    PK: double C-contiguous array
        The control points after the derivation are stored in PK[:nk + 1].
    nk: int
        The highest index of control points PK array.
    p: int
        New degree curve (p - d).
    
    """
    cdef int info, nk, n = P.shape[0] - 1
    
    CurveDerivCptsAlg2(&n, &p, &U[0], &P[0], &d, &nk, &UK[0], &PK[0], &info)
    _check_info(info)
    
    return nk

cpdef void splkref(double[::1] U, double[::1] P, int p, double[::1] X,
                   double[::1] Ubar, double[::1] Q):
    """
    Wrapper for 'RefineKnotVectCurve' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the curve to be derived (initial degree).
    X: double C-contiguous array
        New knots array. Must be in ascending order. New knots x(i) should be
        repeated in X with their multiplicities.
    
    Returns
    -------
    Ubar: double C-contiguous array
        Knot vector after knot refinement.
    Q: double C-contiguous array
        Control Points after knot refinement (or B-splines coefficients).
    
    """
    cdef int info, n = P.shape[0] - 1, r = X.shape[0] - 1
    RefineKnotVectCurve(&n, &p, &U[0], &P[0], &X[0], &r, &Ubar[0], &Q[0],
                        &info)
    _check_info(info)

cpdef void splcjoin(double[::1] Ul, double[::1] Pl, double[::1] Ur,
    double[::1] Pr, int p, double[::1] UK, double[::1] PK):
    """
    Joins two B-Spline of same order with C-1 joint.
    
    Parameters
    ----------
    Ul: double C-contiguous array
        Knot vector of the left B-Spline. Size ml + 1 with ml = nl + p + 1.
    Pl: double C-contiguous array
        Control Points of the left B-Spline. Size nl + 1.
    Ur: double C-contiguous array
        Knot vector of the right B-Spline. Size mr + 1 with mr = nr + p + 1.
    Pr: double C-contiguous array
        Control Points of the right B-Spline. Size nr + 1.
    p: int
        Degree of the curves.
    
    Returns
    -------
    UK: double C-contiguous array
        Knot vector of the resulting curve. Must have size mk + 1 with
        mk = nk + p + 1.
    PK: double C-contiguous array
        Control Points of the resulting curve. Must have size nk + 1 with
        nk = nl + nr + 1.
    
    """
    cdef:
        Py_ssize_t i
        int nl = Pl.shape[0] - 1, ml = Ul.shape[0] - 1
        int nr = Pr.shape[0] - 1, mr = Ur.shape[0] - 1
        int nk = PK.shape[0] - 1, mk = UK.shape[0] - 1
        double delta = 0.
    
    assert ml == nl + p + 1
    assert mr == nr + p + 1
    assert nk == nl + nr + 1
    assert mk == nk + p + 1
    
    # Join(&nl, &Ul[0], &Pl[0], &nr, &Ur[0], &Pr[0], &p, &nk, &UK[0], &PK[0]);
    
    if Ul[ml] != Ur[0]:
        delta = Ul[ml] - Ur[0]
    
    # fill the knot vector
    c_memcpy(&UK[0], &Ul[0], (ml + 1)*sizeof(double))
    with nogil:
        for i in range(nr + 1):
            UK[ml + 1 + i] = Ur[p + 1 + i] + delta
    
    # copy control points
    c_memcpy(&PK[0], &Pl[0], (nl + 1)*sizeof(double))
    c_memcpy(&PK[nl + 1], &Pr[0], (nr + 1)*sizeof(double))

cpdef tuple splcsli(double[::1] U, double[::1] P, int p, double u,
                    double[::1] UK, double[::1] PK):
    """
    Separates the curve of same order at a desired point of the knots domain.
    The outputs are stored as follows:
        
        U_(left) = UK[:nl + p + 2]  ;  U_(right) = UK[nl + 1:nl + nr + p + 3]
        
        P_(left) = PK[:nl + 1]  ;  P_(right) = PK[nl + 1:nl + nr + 2]
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector.
    P: double C-contiguous array
        Control Points (or B-splines coefficients).
    p: int
        Degree of the curve. Must be greater than 0.
    u: double
        Value of the knots domain where the curve must be separated.
    
    Returns
    -------
    UK: double C-contiguous array
        Knot vectors of the resulting curves (see above for storage schemes).
        Must have len(U) + p + 1 size.
    PK: double C-contiguous array
        Control Points of the resulting curves (see above for storage schemes).
        Must have len(P) + p + 1 size.
    nl: int
        The highest index of control points vector of the left resulting
        B-Spline.
    nr: int
        The highest index of control points vector of the right resulting
        B-Spline.
    
    """
    cdef:
        int k, s, r, nl, nk
        int n = P.shape[0] - 1
        int m = U.shape[0] - 1
        int info
    
    assert m == n + p + 1
    assert U[0] < u < U[m]
    
    FindSpanMult(&n, &p, &u, &U[0], &k, &s)
    
    if p - s == -1: # no-continuity
        # copy knots & cpts
        c_memcpy(&UK[0], &U[0], (m + 1)*sizeof(double))
        c_memcpy(&PK[0], &P[0], (n + 1)*sizeof(double))
        nl, nr = k - s, n - k + s - 1
        return nl, nr
    elif p - s < 0:
        assert False
    else:
        r = p - s # number of times the knot must be inserted
    
    nk = n + r
    if r > 0: # insert knot 'u' until multiplicity s = p
        CurveKnotIns(&n, &p, &U[0], &P[0], &u, &k, &s, &r,
                     &nk, &UK[0], &PK[0], &info)
        _check_info(info)
    else: # s = p -> only copy knots & cpts
        c_memcpy(&UK[0], &U[0], (m + 1)*sizeof(double))
        c_memcpy(&PK[0], &P[0], (n + 1)*sizeof(double))
    
    nl = k - s
    nr = n + r - nl
    # add an extra-node and an extra-cpt
    # c_memmove(&UK[nl + p + 1], &UK[nl + p], (nr + p + 1)*sizeof(double))
    c_memmove(&UK[nl + p + 1], &UK[nl + p], (nr + 2)*sizeof(double))
    c_memmove(&PK[nl + 1], &PK[nl], (nr + 1)*sizeof(double))
    return nl, nr

cpdef tuple knotuniondim(double[::1] U1, double[::1] U2, int p):
    """
    Auxiliary function that calculates the dimensions of the new knot vector
    given by the union of the input knot vectors.
    
    Parameters
    ----------
    U1: double C-contiguous array
        Knot vector of the first curve.
    U2: double C-contiguous array
        Knot vector of the second curve.
    p: int
        Degree of the curve.
    
    Returns
    -------
    mk: int
        The highest index of knot vector given by the union of the knot vectors
        U1 and U2.
    nk: int
        The highest index of control points related to the new knot vector.
    
    """
    cdef:
        Py_ssize_t i, j
        int mk, nk
        int m1 = U1.shape[0] - 1, m2 = U2.shape[0] - 1
        int n2 = m2 - p - 1
    
    assert U1[0] == U2[0], "same extremes of the knot vector is required"
    assert U1[m1] == U2[m2], "same extremes of the knot vector is required"
    
    mk = U1.shape[0] - 1
    
    with nogil:
        i = p + 1; j = i
        
        while (i <= n2):
            if MIOF(U2[i], U1[j], RTOL, ATOL):
            # if U2[i] < U1[j]:
                mk += 1; i += 1
            elif CLOSE(U2[i], U1[j], RTOL, ATOL):
            # elif U2[i] == U1[j]:
                i += 1; j += 1
            else:
                j += 1
        
        nk = mk - p - 1
    
    return mk, nk

cpdef void splcsum(double[::1] U1, double[::1] P1, double[::1] U2,
       double[::1] P2, int p, int mk, double[::1] UK, int nk, double[::1] PK):
    """
    Calculates the B-Spline sum of the input curves. The algorithm reshapes the
    curves on a common knot vector (as a union of the knot vectors of the two
    input curves) and then adds the vectors of the control points.
    
    Parameters
    ----------
    U1: double C-contiguous array
        Knot vector of the first curve.
    P1: double C-contiguous array
        Control Points (or B-splines coefficients) of the first curve.
    U2: double C-contiguous array
        Knot vector of the second curve.
    P2: double C-contiguous array
        Control Points (or B-splines coefficients) of the second curve.
    p: int
        Degree of the curve.
    mk: int
        The highest index of output knot vector UK.
    nk: int
        The highest index of output control points PK.
    
    Returns
    -------
    UK: double C-contiguous array
        Knot vector of the B-SPline sum.
    PK: double C-contiguous array
        Control points of the B-SPline sum.
    
    Notes
    -----
    'mk' and 'nk' should be calculated by the function 'knotuniondim'.
    
    """
    cdef Py_ssize_t i
    cdef int m1 = U1.shape[0] - 1, m2 = U2.shape[0] - 1
    
    assert U1[0] == U2[0], "same extremes of the knot vector is required"
    assert U1[m1] == U2[m2], "same extremes of the knot vector is required"
    
    if isclose(U1, U2):
        # special case: if the trajectories are defined on the same knot
        # vector, no nodes need to be added
        c_memcpy(&UK[0], &U1[0], U1.shape[0]*sizeof(double))
        
        for i in range(nk + 1):
            PK[i] = P1[i] + P2[i]
        
        return
    
    cdef Py_ssize_t j, k, ii, jj
    cdef double[::1] PK2, X1, X2
    
    # UK[:p + 1] = U1[0]
    # UK[- p - 1:] = U1[m1]
    
    PK2 = NP.empty_like(PK)
    X1 = NP.empty(mk - m1, dtype = NP.float64)
    X2 = NP.empty(mk - m2, dtype = NP.float64)
    
    with nogil:
        i = j = p + 1
        ii = jj = 0
        for k in range(p + 1, nk + 1):
            
            if MIOF(U1[i], U2[j], RTOL, ATOL):
            # if U1[i] < U2[j]:
                X2[jj] = U1[i]
                i += 1; jj += 1
                
            elif GROF(U1[i], U2[j], RTOL, ATOL):
            # elif U1[i] > U2[j]:
                X1[ii] = U2[j]
                j += 1; ii += 1
                
            else:
                i += 1; j += 1
    
    if mk > m1: splkref(U1, P1, p, X1, UK, PK)
    else: PK[:] = P1[:]
    
    if mk > m2: splkref(U2, P2, p, X2, UK, PK2)
    else: PK2[:] = P2[:]
    
    for i in range(nk + 1):
        PK[i] += PK2[i]

cpdef void knotsdegelev(double[::1] U, int p, double[::1] Uh):
    """
    Wrapper for 'IncreaseMultByOne' function of bsplclib C-library.
    
    Parameters
    ----------
    U: double C-contiguous array
        Knot vector.
    p: int
        Degree of the B-Spline.
    
    Returns
    -------
    Uh: double C-contiguous array
        New knot vector (with all multiplicities increased by 1).
    
    Notes
    -----
    The user must respect the relation: mh = m + s + 2, whit s the number of
    all distinct interior knots.
    
    """
    cdef int m = U.shape[0] - 1, mh = Uh.shape[0] - 1
    
    IncreaseMultByOne(&m, &p, &U[0], &mh, &Uh[0])

include "utils.pxi"