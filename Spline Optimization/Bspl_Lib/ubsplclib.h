#define INF 1.0/0.0
#define ATOL 1E-13
#define RTOL 1E-9

#define MTX2D(r, c, cols)  ((c) + (r) * (cols))
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)
#define DTOL(a, b, rtol, atol) (MAX(rtol * MAX(fabs(a), fabs(b)), atol))
#define CLOSE(a, b, rtol, atol) (fabs(a - b) <= DTOL(a, b, rtol, atol) ? 1 : 0)
#define GREQ(a, b, rtol, atol) (a > b - DTOL(a, b, rtol, atol) ? 1 : 0)
#define GROF(a, b, rtol, atol) (a > b + DTOL(a, b, rtol, atol) ? 1 : 0)
#define MIOF(a, b, rtol, atol) (a < b - DTOL(a, b, rtol, atol) ? 1 : 0)
#define MIEQ(a, b, rtol, atol) (a < b + DTOL(a, b, rtol, atol) ? 1 : 0)
#define LININT(t,tk,ak,tk1,ak1) (ak*(tk1-t)/(tk1-tk) + ak1*(t-tk)/(tk1-tk))

#define SUCCESS 0
#define MEM_ERR 1
#define DIV_ZERO_ERR 2
#define NOT_IMPLEMENTED 3
#define MAX_ITER 4
#define OUT_OF_RANGE 5
#define GENERIC_FAILURE 6

int ValidKnotVector(int *n, int *p, double *U);
int FindSpan(int *n, int *p, double *u, double *U);
void FindSpanMult(int *n, int *p, double *u, double *U, int *k, int *s);
void BasisFuns(int *i, double *u, int *p, double *U, double *left,
               double *right, double *N);
void OneBasisFun(int *p, int *m, double *U, int *i, double *u, double *N,
                 double *Nip);
void CurvePoint(int *n, int *p, double *U, double *P, double *u, double *C,
                int *info);
void CurveEval(int *n, int *p, double *U, double *P, double *C, int *np,
               int *info);
void EquallySpaced(int *n, double *x, int *p, double *U);
void AveragingAlg1(int *n, double *x, int *p, double *U);
void AveragingAlg2(int *n, double *x, int *p, double *U);
void AveragingAlg3(int *n, double *x, int *p, int *k, int *l, double *U);
void KnotsEvalAlg2(int *n, double *x, int *p, double *U);
void KnotsEvalL2Approx(int *nx, double *x, int *p, int *n, double *U);
void AveragingCpts(int *n, int *p, double *U, double *C, int *info);
void DersBasisFuns(int *i, double *u, int *p, int *n, double *U, double *ndu,
                   double *a, double *left, double *right, double *ders);
void CurveDerivsAlg1(int *n, int *p, double *U, double *P, double *u, int *d,
                     double *CK, int *info);
void CurveDerivsEval(int *n, int *p, double *U, double *P, int *r1, int *r2,
                     int *np, double *C, int *cp, double *CK, int *info);
void CurveDerivCptsAlg1(int *n, int *p, double *U, double *P, int *d, int *r1,
                        int *r2, double *PK, int *info);
void CurveKnotIns(int *np, int *p, double *UP, double *P, double *u, int *k,
                  int *s, int *r, int *nq, double *UQ, double *Q, int *info);
void RemoveCurveKnot(int *n, int *p, double *U, double *P, double *u, int *r,
                     int *s, int *num, double *TOL, int *t, int *info);
void RemoveCurveKnots(int *n, int *p, double *U, double *P, double *TOL,
                      int *gap, int *info);
void RemoveCurveKnotWithoutTol(int *n, int *p, double *U, double *P, double *u,
                               int *r, int *s, int *num, int *t, int *info);
void DegreeElevateCurve(int *n, int *p, double *U, double *P, int *t,
                        double *Uh, int *nh, double *Q, int *info);
void SolveTridiagonal(int *n, double *Q, double *U, double *P, int *info);
void CurveDerivCptsAlg2(int *n, int *p, double *U, double *P, int *d,
                        int *nk, double *UK, double *PK, int *info);
void CurveAntiDerivCpts(int *n, int *p, double *U, double *P, int *d,
                        int *nk, double *UK, double *PK, int *info);
void RefineKnotVectCurve(int *n, int *p, double *U, double *P, double *X,
                         int *r, double *Ubar, double *Q, int *info);
void IncreaseMultByOne(int *m, int *p, double *U, int *mh, double *Uh);
void PSparseCSCEval(double *dt, double *K, int *N, double *data, int *row_ind,
                     int *col_ind, int *info);
void _Betaki(double *t, int *k, int *i, int *p, int *l, double *G, int *m,
             double *U, double *Bki, int *info);
void LinInterpEval(int *Nx, double *x, int *Nt, double *t, double *a,
                   int *info);
int ConcavityCheck(double *c, double *x, int *N, int *conc);
int ConvexityCheck(double *c, double *x, int *N, int *conv);
void _ASparseCSCEval(double *G, double *U, int *p, double *dt, double *K,
    int *N, int *d, int *ccc, int *cvc, int *concav_span, int *convex_span,
    double *xconc, double *xconv, double *data, int *row_ind, int *col_ind,
    int *info);