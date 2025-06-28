import numpy as np
from Bspl_Lib.cython_ubsplclib import betaki, averagcpts, findspan, splcev, validknots, dbfunspev, dsplcev
import matplotlib.pyplot as plt
from scipy.sparse import csc_matrix
import osqp
import sys

np.set_printoptions(threshold=sys.maxsize)

from mpl_toolkits.mplot3d import Axes3D

###############################################################################
#### utils func.
###############################################################################
def get_bbox_corners(lower, upper):
    """
    Given lower = (x_min, y_min, z_min) and
          upper = (x_max, y_max, z_max),
    return an array of shape (8, 3) with all corner points of the bounding box.
    """
    xs = [lower[0], upper[0]]
    ys = [lower[1], upper[1]]
    zs = [lower[2], upper[2]]
    corners = []
    for x in xs:
        for y in ys:
            for z in zs:
                corners.append([x, y, z])
    return np.array(corners, dtype=np.float64)

def plot_bbox_3d(ax, lower, upper):
    """
    Plot a 3D wireframe for a single bounding box on the given axes (ax).
    """
    corners = get_bbox_corners(lower, upper)
    # Indices of the corners in a typical 3D bounding box (8 corners).
    # We'll connect them to form 12 edges.
    edges = [
        (0,1), (0,2), (0,4), (1,3), (1,5), (2,3), 
        (2,6), (3,7), (4,5), (4,6), (5,7), (6,7)
    ]
    for i1, i2 in edges:
        x_vals = [corners[i1, 0], corners[i2, 0]]
        y_vals = [corners[i1, 1], corners[i2, 1]]
        z_vals = [corners[i1, 2], corners[i2, 2]]
        ax.plot3D(x_vals, y_vals, z_vals, 'b-')

###############################################################################
###############################################################################

def compute_P_matrix(g: np.array):
    N = g.shape[0]
    P = np.zeros((N,N), dtype = np.float64, order='C')
    dg = np.empty(N, dtype = np.float64, order = 'C')
    K = np.empty(N, dtype = np.float64, order = 'C')

    for i in range(1,N):
        dg[i] = g[i] - g[i-1]
    
    for i in range(1,N-1):
        K[i] = dg[i + 1]*dg[i]
    
    for i in range(2, N - 2):
         P[i][i - 2] = (dg[i - 1]*dg[i])/K[i - 1]**2.0
         
         P[i][i - 1] = - ((dg[i - 1]*(dg[i - 1] + dg[i]))/K[i - 1]**2.0 +
            ((dg[i] + dg[i + 1])*dg[i + 1])/K[i]**2.0)
         
         P[i][i] = (dg[i - 1]**2.0/K[i - 1]**2.0 +
                    (dg[i] + dg[i + 1])**2.0/K[i]**2.0 +
                    (dg[i +2]**2.0)/K[i + 1]**2.0)
         
         P[i][i + 1] = - ((dg[i]*(dg[i] + dg[i + 1]))/K[i]**2.0 +
            ((dg[i + 1] + dg[i + 2])*dg[i + 2])/K[i + 1]**2.0)
         
         P[i][i + 2] = (dg[i + 1]*dg[i + 2])/K[i + 1]**2.0
    
    P[1][0] = -(dg[1] + dg[2])*dg[2]/K[1]**2.0
    P[1][1] = ((dg[1] + dg[2])**2.0/K[1]**2.0 +
        dg[3]**2.0/K[2]**2.0)
    P[1][2] = - dg[1]*(dg[1] + dg[2])/K[1]**2.0 - (
        dg[2] + dg[3])*dg[3]/K[2]**2.0
    P[1][3] = (dg[2]*dg[3])/K[2]**2.0
    
    P[0][1] = P[1][0]
    P[0][2] = P[2][0]
    P[0][0] = dg[2]**2.0/K[1]**2.0
    
    P[N-2][N-4] = (dg[N-3]*dg[N-2])/K[N-3]**2.0
    P[N-2][N-3] = - dg[N-3]*(dg[N-3] + dg[N-2])/K[N-3]**2.0 - (
        dg[N-2] + dg[N-1])*dg[N-1]/K[N-2]**2.0
    P[N-2][N-2] = dg[N-3]**2.0/K[N-3]**2.0 + (
        dg[N-2] + dg[N-1])**2.0/K[N-2]**2.0
    P[N-2][N-1] = - dg[N-2]*(dg[N-2] + dg[N-1])/K[N-2]**2.0
    
    P[N-1][N-3] = P[N-3][N-1]
    P[N-1][N-2] = P[N-2][N-1]
    P[N-1][N-1] = dg[N-2]**2.0/K[N-2]**2.0
    
    return P


if __name__ == "__main__":
    start = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    target = np.array([2.36951, -2.06501, -2.97366], dtype=np.float64)
    
    lower_bounds = np.array([
        [-1.55,    -1.55,    -1.55],
        [-0.452453,  -1.16847, 	-2.06161],
        [-0.673659,  -1.41302, -2.67907],
        [-0.607161,  -1.45352, -3.37473],
        [-0.116805,  -1.93423, -3.51065],
        [0.532035,  -1.94474, -3.52767],
        [0.884416,  -2.05578, -3.54931],
        [1.13383,  -2.28662, -3.58721],
        [1.30919,  -2.56098, -3.57697],
        [1.07235,   -2.62506, -3.29721],
        [1.76031,   -2.31685, -3.11083],
        [1.9822, -2.59845, -3.47735]

        ], dtype=np.float64, order='C')
    
    upper_bounds = np.array([
        [1.55,     1.55,     1.55],
        [0.247547,  -0.468475,  -1.36161],
        [0.0263407,  -0.713025,  -1.97907],
        [0.0928385,  -0.753524, -2.67473],
        [0.583195,  -1.23423, -2.81065],
        [0.932035,  -1.54474, -3.12767],
        [1.18442,  -1.75578, -3.24931],
        [1.53383,  -1.88662, -3.18721],
        [1.60919,  	-2.26098, -3.27697],
        [1.77235,   -1.92506, -2.59721],
        [2.06031,   -2.01685, -2.81083],
        [2.5822, -1.99845, 	-2.87735]

    ], dtype=np.float64, order='C')
    
    
    # start = (0., 1., 0.)
    # target = (4., 13., 0.)
    
    # lower_bounds = np.array([
    #     [0.,  0.,  0.],
    #     [4.,  2., -1.],
    #     [5.,  5.,  0.],
    #     [4.,  7.,  1.],
    #     [3.,  8.,  0.]
    # ], dtype=np.float64, order='C')
    
    # upper_bounds = np.array([
    #     [4.,  4.,  2.],
    #     [6.,  5.,  1.],
    #     [10., 11., 2.],
    #     [5.,  8.,  3.],
    #     [4., 14.,  4.]
    # ], dtype=np.float64, order='C')
    
    F = upper_bounds.shape[0]
    dim = upper_bounds.shape[1]
    d = 5
    Ns = F*d + 1
    nvars = dim*Ns
    n_continuity = 3
    
    knots_initial = np.arange(F + 1, dtype=np.float64).repeat(d)
    knots_initial = np.concatenate(
        [[knots_initial[0]], knots_initial, [knots_initial[-1]]])

    g = np.empty(Ns, dtype=np.float64, order='C')
    averagcpts(knots_initial, d, g)

    P = compute_P_matrix(g)
    Pbar = np.zeros((nvars, nvars), dtype=np.float64, order='C')
    
    for kd in range(dim):
        Pbar[kd*Ns:(kd+1)*Ns, kd*Ns:(kd+1)*Ns] = P

    n_box_constraints = dim*F*(d + 1)
    n_Cn_constraints = (F - 1)*n_continuity*dim
    n_ST_constraints = 2*dim

    n_constraints = n_box_constraints + n_ST_constraints + n_Cn_constraints
    
    
    A = np.zeros((n_constraints, nvars), dtype=np.float64, order='C')
    l_vec = -np.inf*np.ones(n_constraints, dtype = np.float64, order = 'C')
    u_vec =  np.inf*np.ones(n_constraints, dtype = np.float64, order = 'C')

    row = 0
    
    for i in range(F):
        for j in range(i*d, i*d + d + 1):
            if j == 0 or j == F*d:
                continue
            
            for kdim in range(dim):
                A[row, kdim*Ns + j] = 1.0
                l_vec[row]         = lower_bounds[i, kdim]
                u_vec[row]         = upper_bounds[i, kdim]
                row += 1

    for i in range(1, F):
        param = float(i)
        
        span_right = findspan(d, param, knots_initial)
        span_left = span_right - d

        nders_left = np.zeros((n_continuity + 1, d + 1),
                              dtype = np.float64, order = 'C')
        nders_right = np.zeros((n_continuity + 1, d + 1),
                                dtype = np.float64, order = 'C')
        
        dbfunspev(
            param, span_left, knots_initial, d, n_continuity, nders_left)
        dbfunspev(
            param, span_right, knots_initial, d, n_continuity, nders_right)
        
        for r in range(1, n_continuity + 1):
            for kdim in range(dim):
                left_block  = kdim*Ns + (i - 1)*d
                right_block = kdim*Ns + i*d
                
                for jcol in range(d + 1):
                    A[row, left_block  + jcol] += nders_left[r, jcol]
                    A[row, right_block + jcol] -= nders_right[r, jcol]
                l_vec[row] = 0.0
                u_vec[row] = 0.0
                row += 1
    
    for kdim in range(dim):
        A[row, kdim*Ns + 0] = 1.0
        l_vec[row]          = start[kdim]
        u_vec[row]          = start[kdim]
        row += 1
        
        A[row, kdim*Ns + (F*d)] = 1.0
        l_vec[row]              = target[kdim]
        u_vec[row]              = target[kdim]
        row += 1


    P_csc = csc_matrix(Pbar)
    A_csc = csc_matrix(A)
    q = np.zeros(nvars, dtype=np.float64, order='C')

    solver = osqp.OSQP()
    solver.setup(P=P_csc, q=q, A=A_csc, l=l_vec, u=u_vec, verbose=True)
    results = solver.solve()
    
    cpts_res = []
    for kd in range(dim):
        cpts_res.append(results.x[kd*Ns : (kd+1)*Ns])




    # --------------------------------------------------------------
    # Plot the bounding boxes in 3D
    # --------------------------------------------------------------
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    for lower, upper in zip(lower_bounds, upper_bounds):
        plot_bbox_3d(ax, lower, upper)

    ax.scatter(*start, s=50, label="start")
    ax.scatter(*target, s=50, label="target")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    max_range = max(
        x_limits[1] - x_limits[0], 
        y_limits[1] - y_limits[0], 
        z_limits[1] - z_limits[0]
    )
    mid_x = np.mean(x_limits)
    mid_y = np.mean(y_limits)
    mid_z = np.mean(z_limits)
    ax.set_xlim3d(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim3d(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim3d(mid_z - max_range/2, mid_z + max_range/2)
    
    
    Cx = np.linspace(knots_initial[0], knots_initial[-1], 101)
    Cy = np.linspace(knots_initial[0], knots_initial[-1], 101)
    Cz = np.linspace(knots_initial[0], knots_initial[-1], 101)

    splcev(Cx, knots_initial, cpts_res[0], d)
    splcev(Cy, knots_initial, cpts_res[1], d)
    splcev(Cz, knots_initial, cpts_res[2], d)
    
    ax.plot3D(Cx, Cy, Cz, 'k-', label="spline")
    ax.set_axis_off()
    plt.show()
    
    
    for nder in range(1,4):
        plt.figure()
        t = np.linspace(knots_initial[0], knots_initial[-1], 101)
        
        CKx = np.zeros((1, len(t)), dtype=np.float64, order='C')
        dsplcev(t, knots_initial, cpts_res[0], d, nder, nder, CKx)
        
        
        CKy = np.zeros((1, len(t)), dtype=np.float64, order='C')
        dsplcev(t, knots_initial, cpts_res[1], d, nder, nder, CKy)
        
        CKz = np.zeros((1, len(t)), dtype=np.float64, order='C')
        dsplcev(t, knots_initial, cpts_res[2], d, nder, nder, CKz)
        
        plt.plot(t, CKx[0], label=f"d{nder}x/du{nder}")
        plt.plot(t, CKy[0], label=f"d{nder}y/du{nder}")
        plt.plot(t, CKz[0], label=f"d{nder}z/du{nder}")

        plt.ylabel(f"derivata {nder}° ordine")
    
    plt.show()

    print('knots: ', repr(knots_initial))
    print('cpts_1: ', repr(cpts_res[0]))
    print('cpts_2: ', repr(cpts_res[1]))
    print('cpts_3: ', repr(cpts_res[2]))
    print('deg: ', d)

    t = np.linspace(knots_initial[0], knots_initial[-1], 101)

 
    spline_x = t.copy()
    spline_y = t.copy()
    spline_z = t.copy()


    splcev(spline_x, knots_initial, cpts_res[0], d)
    splcev(spline_y, knots_initial, cpts_res[1], d)
    splcev(spline_z, knots_initial, cpts_res[2], d)


    spline_points = np.vstack((spline_x, spline_y, spline_z)).T
    print(spline_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(spline_points[:,0], spline_points[:,1], spline_points[:,2], 'k-', label="spline")
    ax.legend()
    plt.show()