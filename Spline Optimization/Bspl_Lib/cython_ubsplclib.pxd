# -*- coding: utf-8 -*-
cpdef int concavityCheck(double[::1] c, double[::1] x, int[::1] conc)
cpdef int convexityCheck(double[::1] c, double[::1] x, int[::1] conv)
cpdef void linInterpEval(double[::1] x, double[::1] t, double[::1] a)
cpdef void aSparseCSCEval(double[::1] G, double[::1] U, int p, double[::1] dt,
          double[::1] K, int[::1] concav_span, int[::1] convex_span,
          double[::1] xconc, double[::1] xconv, double[::1] data,
          int[::1] row_ind, int[::1] col_ind)
cpdef double betaki(double t, int k, int i, int p, int l, double[::1] G,
                    double[::1] U)
cpdef void pSparseCSCEval(double[::1] dt, double[::1] K, double[::1] data,
                          int[::1] row_ind, int[::1] col_ind)
cpdef bint validknots(int p, double[::1] U)
cpdef void eqspaced(double[::1] xp, int p, double[::1] U)
cpdef void averaging(double[::1] xp, int p, double[::1] U)
cpdef void averaging2(double[::1] xp, int p, double[::1] U)
cpdef void averaging3(double[::1] xp, int p, int k, int l, double[::1] U)
cdef int distintknots(double[::1] U, int p)
cpdef void kevalappr(double[::1] xp, int p, double[::1] U)
cpdef void gcsplint(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P)
cpdef void gdsplint(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P, (double, double) dy = *)
cpdef void cdsplint(double[::1] xp, double[::1] fp, double[::1] U,
                    double[::1] P, (double, double) dy = *)
cpdef void gcsplapp(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P)
cpdef void gdsplapp(double[::1] xp, double[::1] fp, int p, double[::1] U,
                    double[::1] P, (double, double) dy = *)
cpdef void gadsplint(double[::1] xp, double[::1] fp, int p, double[::1] Ds,
                     double[::1] De, double[::1] U, double[::1] P)
cpdef void averagcpts(double[::1] U, int p, double [::1] C)
cpdef int findspan(int p, double u, double[::1] U)
cpdef (int, int) findspanmult(int p, double u, double[::1] U)
cpdef double bfunpev(double u, int span, double[::1] U, int p)
cpdef void bfuncev(double[::1] C, int span, double[::1] U, int p)
cpdef void bfunspev(double u, int span, double[::1] U, int p, double[::1] N)
cpdef void dbfunspev(double u, int span, double[::1] U, int p, int k,
                     double[:,::1] nders)
cpdef double splpev(double u, double[::1] U, double[::1] P, int p)
cpdef void splcev(double [::1] C, double[::1] U, double[::1] P, int p)
cpdef void dsplpev(double u, double[::1] U, double[::1] P, int p, int d,
                   double[::1] CK)
cpdef void dsplcev(double [::1] C, double[::1] U, double[::1] P, int p,
                   int r1, int r2, double [:, ::1] CK)
cpdef void dsplcpts(double[::1] U, double[::1] P, int p, int d, int r1, int r2,
                    double [:, ::1] PK)
cpdef void splkins(double[::1] UP, double[::1] P, int p, double u, int r,
                   double[::1] UQ, double[::1] Q)
cpdef int splkrem(double[::1] U, double[::1] P, int p, double u, int num,
                  double tol = *)
cpdef int splksrem(double[::1] U, double[::1] P, int p, double tol = *)
cpdef int splkrem_notol(double[::1] U, double[::1] P, int p, double u, int num)
cpdef void degelevc(double[::1] U, double[::1] P, int p, int t, double[::1] Uh,
                    double[::1] Q)
cpdef void asplcpts(double[::1] U, double[::1] P, int p, int d, double[::1] UK,
                    double[::1] PK)
cpdef int dsplcpts2(double[::1] U, double[::1] P, int p, int d, double[::1] UK,
                    double[::1] PK)
cpdef void splkref(double[::1] U, double[::1] P, int p, double[::1] X,
                   double[::1] Ubar, double[::1] Q)
cpdef void splcjoin(double[::1] Ul, double[::1] Pl, double[::1] Ur,
                    double[::1] Pr, int p, double[::1] UK, double[::1] PK)
cpdef tuple splcsli(double[::1] U, double[::1] P, int p, double u,
                    double[::1] UK, double[::1] PK)
cpdef tuple knotuniondim(double[::1] U1, double[::1] U2, int p)
cpdef void splcsum(double[::1] U1, double[::1] P1, double[::1] U2,
                   double[::1] P2, int p, int mk, double[::1] UK, int nk,
                   double[::1] PK)
cpdef void knotsdegelev(double[::1] U, int p, double[::1] Uh)