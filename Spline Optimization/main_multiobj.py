import numpy as np
from Bspl_Lib.cython_ubsplclib import betaki, averagcpts, findspan, splcev, validknots, dbfunspev, dsplcev
import matplotlib.pyplot as plt
from scipy.sparse import csc_matrix
import osqp
import sys
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import json
np.set_printoptions(threshold=sys.maxsize)

from mpl_toolkits.mplot3d import Axes3D


def plot_bbox_3d(ax, lower, upper, facecolor='gray', edgecolor='darkblue', alpha=0.05):
    """
    Disegna un bounding box 3D specificato da:
    - lower = (x_min, y_min, z_min)
    - upper = (x_max, y_max, z_max)
    
    facecolor: colore di riempimento
    edgecolor: colore dei bordi
    alpha: trasparenza (0.0 = trasparente, 1.0 = opaco)
    """
    x = [lower[0], upper[0]]
    y = [lower[1], upper[1]]
    z = [lower[2], upper[2]]
    
 
    vertices = [
        [(x[0], y[0], z[0]), (x[0], y[0], z[1]), (x[0], y[1], z[1]), (x[0], y[1], z[0])],  # faccia x = x[0]
        [(x[1], y[0], z[0]), (x[1], y[0], z[1]), (x[1], y[1], z[1]), (x[1], y[1], z[0])],  # faccia x = x[1]
        [(x[0], y[0], z[0]), (x[0], y[0], z[1]), (x[1], y[0], z[1]), (x[1], y[0], z[0])],  # faccia y = y[0]
        [(x[0], y[1], z[0]), (x[0], y[1], z[1]), (x[1], y[1], z[1]), (x[1], y[1], z[0])],  # faccia y = y[1]
        [(x[0], y[0], z[0]), (x[0], y[1], z[0]), (x[1], y[1], z[0]), (x[1], y[0], z[0])],  # faccia z = z[0]
        [(x[0], y[0], z[1]), (x[0], y[1], z[1]), (x[1], y[1], z[1]), (x[1], y[0], z[1])]   # faccia z = z[1]
    ]
    

    poly = Poly3DCollection(vertices,
                            facecolors=facecolor,
                            edgecolors=edgecolor,
                            alpha=alpha,
                            lightsource=True,
                            linewidth=0.15)
    ax.add_collection3d(poly)

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

def build_pattern(d, F):
    pattern = [0] * (d + 1) 
    for val in range(1, F):
        pattern += [val] * d  
    return pattern

if __name__ == "__main__":
    # start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    # target = np.array([0.247047, -0.650365, 0.663831, 1.3142, 2.89455, 1.66958], dtype=np.float64)

    # start = np.array([0.24, -0.64, 0.66, 1.3142, 2.89455, 1.66958], dtype=np.float64)
    # target = np.array([ -2.35, -0.526721, 0.7, -2.02, 0.738725, -1.48671], dtype=np.float64)

    start = np.array([1.60, -0.35, 1.15, 3.2, -1.45, 0.0], dtype=np.float64)
    target = np.array([ -1.4, -1.9, 0.52, 7.0, -1.55, 3.8], dtype=np.float64)
    #target = np.array([ -0.6, -1.64, 1.70, 8.5, -1.05, 4.53593619], dtype=np.float64)


    # start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    # target = np.array([ -2.92067, 1.21734, 1.11218, 3.03643, 0.220921, -1.47201], dtype=np.float64)
    
    lower_bounds = np.array([
                    [1.4, -0.55, 0.95, 3, -1.65, -0.2],
                    [1.21267, -0.688946, 0.941763, 3.44747, -1.67085, -0.0732632],
                    [1.02533, -0.827891, 0.933526, 3.89493, -1.6917, 0.0534735],
                    [0.837999, -0.966837, 0.925289, 4.3424, -1.71255, 0.18021],
                    [0.650665, -1.10578, 0.917052, 4.78986, -1.7334, 0.306947],
                    [0.463331, -1.24473, 0.908814, 5.23733, -1.75425, 0.433684],
                    [0.275998, -1.38367, 0.900577, 5.6848, -1.7751, 0.560336],
                    [0.0886638, -1.52262, 0.89234, 6.13226, -1.79595, 0.686989],
                    [-0.09867, -1.66156, 0.884103, 6.57973, -1.8168, 0.813642],
                    [-0.286004, -1.80051, 0.875866, 7.02719, -1.83765, 0.940294],
                    [-0.473337, -1.93946, 0.867629, 7.47466, -1.8585, 1.06695],
                    [-0.848005, -2.08539, 0.851155, 7.43997, -1.9002, 1.32025],
                    [-1.22267, -2.23133, 0.94093, 7.56778, -1.9419, 1.57356],
                    [-1.42267, -2.29068, 0.818206, 7.34874, -1.86732, 1.82686],
                    [-1.46067, -2.24982, 0.793495, 7.23115, -1.84352, 2.20682],
                    [-1.461, -2.22258, 0.763732, 7.15276, -1.85265, 2.46013],
                    [-1.46134, -2.19534, 0.79568, 7.07437, -1.84928, 2.71343],
                    [-1.46167, -2.1681, 0.596379, 6.99598, -1.72092, 2.96674],
                    [-1.462, -2.14086, 0.459577, 6.91759, -1.79255, 3.22004],
                    [-1.47483, -2.11362, 0.366526, 6.9142, -1.76418, 3.47335]
                ], dtype=np.float64, order='C')

    upper_bounds = np.array([
                        [1.8, -0.15, 1.35, 3.4, -1.25, 0.2],
                        [1.61267, -0.288946, 1.34176, 3.84747, -1.27085, 0.326737],
                        [1.42533, -0.427891, 1.33353, 4.29493, -1.2917, 0.453474],
                        [1.238, -0.566837, 1.32529, 4.7424, -1.31255, 0.58021],
                        [1.05067, -0.705782, 1.31705, 5.18986, -1.3334, 0.706947],
                        [0.863331, -0.844728, 1.30881, 5.63733, -1.35425, 0.833684],
                        [0.675998, -0.983673, 1.30058, 6.0848, -1.3751, 0.960336],
                        [0.488664, -1.12262, 1.29234, 6.53226, -1.39595, 1.08699],
                        [0.30133, -1.26156, 1.2841, 6.97973, -1.4168, 1.21364],
                        [0.113996, -1.40051, 1.27587, 7.42719, -1.43765, 1.34029],
                        [-0.0733375, -1.53946, 1.26763, 7.80591, -1.4585, 1.46695],
                        [-0.448005, -1.68539, 1.25115, 7.80872, -1.5002, 1.72025],
                        [-0.822672, -1.97508, 1.23468, 7.80528, -1.5419, 1.97356],
                        [-1.02267, -1.96568, 1.21821, 7.74874, -1.54857, 2.22686],
                        [-1.17942, -1.94982, 1.19349, 7.63115, -1.50602, 2.60682],
                        [-1.186, -1.95383, 1.13873, 7.55276, -1.47765, 2.86013],
                        [-1.19259, -1.97659, 1.04568, 7.38062, -1.44928, 3.11343],
                        [-1.24292, -1.7681, 0.952629, 7.22098, -1.42092, 3.36674],
                        [-1.31825, -1.74086, 0.859577, 7.21759, -1.39255, 3.62004],
                        [-1.33733, -1.71362, 0.766526, 7.20795, -1.36418, 3.87335]
    ], dtype=np.float64, order='C')
        
    # start = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    # target = np.array([2.36951, -2.06501, -2.97366], dtype=np.float64)
    
    # lower_bounds = np.array([
    #     [-1.55,    -1.55,    -1.55],
    #     [0.46526,  -2.06616, -0.361836],
    #     [1.02456,  -2.38752, -0.470596],
    #     [1.10525,  -2.58324, -1.14042],
    #     [1.79752,  -2.57498, -1.0688],
    #     [2.22458,  -2.47203, -1.25863],
    #     [2.28409,  -2.25043, -1.58627],
    #     [2.29112,  -2.26537, -1.98593],
    #     [2.25521,  -2.13937, -2.36387],
    #     [2.1584,   -2.25799, -2.85614],
    #     [2.1064,   -2.33423, -3.34755]
    #     ], dtype=np.float64, order='C')
    
    # upper_bounds = np.array([
    #     [1.55,     1.55,     1.55],
    #     [1.16526,  -1.36616,  0.338164],
    #     [1.62456,  -1.78752,  0.129404],
    #     [1.80525,  -1.88324, -0.440422],
    #     [2.29752,  -2.07498, -0.568802],
    #     [2.62458,  -2.07203, -0.858632],
    #     [2.68409,  -1.85043, -1.18627],
    #     [2.69112,  -1.86537, -1.58593],
    #     [2.65521,  -1.73937, -1.96387],
    #     [2.6584,   -1.75799, -2.35614],
    #     [2.6064,   -1.83423, -2.84755]
    # ], dtype=np.float64, order='C')
    
    
    # start = (0., 2., 0.)
    # target = (4.5, 8., 5.)
    
    # lower_bounds = np.array([
    #     [0.,  1.,  0.],
    #     [3.,  2., -1.],
    #     [5.,  5.,  0.],
    #     [4.,  8.,  2.]
    # ], dtype=np.float64, order='C')
    
    # upper_bounds = np.array([
    #     [4.,  4.,  2.],
    #     [6.,  5.,  1.5],
    #     [8.,  8.,  2.],
    #     [5.,  10., 6.]
    # ], dtype=np.float64, order='C')
    
    F = upper_bounds.shape[0]
    dim = upper_bounds.shape[1]
    d = 4
    Ns = F*d + 1
    nvars = dim*Ns
    n_continuity = 3
    
    pat = build_pattern(d, F)
    p_len = len(pat)
    
    knots_initial = np.arange(F + 1, dtype=np.float64).repeat(d)
    knots_initial = np.concatenate(
        [[knots_initial[0]], knots_initial, [knots_initial[-1]]])

    g = np.empty(Ns, dtype=np.float64, order='C')
    averagcpts(knots_initial, d, g)


    fig = plt.figure()
    ax = plt.axes(projection='3d')
    c = '#ff0080ff', '#0080ffff'
    alpha1 = 1.0
    alpha2 = 0.0

    #for l in range(len(alpha1)):
    P = compute_P_matrix(g)
    Pbar = np.zeros((2*nvars, 2*nvars), dtype=np.float64, order='C')
    
    for kd in range(dim):
        Pbar[kd*Ns:(kd+1)*Ns, kd*Ns:(kd+1)*Ns] = P

    Pbar *= alpha1

    n_box_constraints = dim*F*(d + 1)
    n_Cn_constraints = (F - 1)*n_continuity*dim
    n_ST_constraints = 2*dim
    n_F2_constraints = 2*nvars + 2*dim*(F - 1)
    

    n_constraints = n_box_constraints + n_ST_constraints + n_Cn_constraints + n_F2_constraints
    
    A = np.zeros((2*n_constraints, 2*nvars), dtype=np.float64, order='C')
    l_vec = -np.inf*np.ones(2*n_constraints, dtype = np.float64, order = 'C')
    u_vec =  np.inf*np.ones(2*n_constraints, dtype = np.float64, order = 'C')

    row = 0
    
    for i in range(F):
        for j in range(i*d, i*d + d + 1):
            # if j == 0 or j == F*d:
            #     continue
            
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

    

    for i in range(nvars):
        A[row, i] = 2.0
        A[row, i + nvars] = 1.0
        idx = pat[i% p_len]
        
        l_vec[row] = lower_bounds[:,i//Ns][idx] + upper_bounds[:,i//Ns][idx]
            
        row += 1
        
        if i%d == 0 and i%((F-1)*(d+1))!=0 and i//(Ns - 1) == 0:
            A[row, i] = 2.0
            A[row, i + nvars] = 1.0
            l_vec[row] = lower_bounds[:,i//Ns][idx + 1] + upper_bounds[:,i//Ns][idx + 1]
            
            row += 1
            
    
    for i in range(nvars):
        A[row, i] = -2.0
        A[row, i + nvars] = 1.0
        idx = pat[i% p_len]
        
        l_vec[row] = - (lower_bounds[:,i//Ns][idx] + upper_bounds[:,i//Ns][idx])
        
        row += 1
        
        if i%d == 0 and i%((F-1)*(d+1))!=0 and i//(Ns - 1) == 0:
            A[row, i] = -2.0
            A[row, i + nvars] = 1.0
            l_vec[row] = - (lower_bounds[:,i//Ns][idx + 1] + upper_bounds[:,i//Ns][idx + 1])
            
            row += 1
            
    
    P_csc = csc_matrix(Pbar)
    A_csc = csc_matrix(A)
    q = np.zeros(2*nvars, dtype=np.float64, order='C')
    q[nvars:] = alpha2
    
    solver = osqp.OSQP()
    solver.setup(P=P_csc, q=q, A=A_csc, l=l_vec, u=u_vec, verbose=True, max_iter = 1000000)
    results = solver.solve()
    
    cpts_res = []
    for kd in range(dim):
        cpts_res.append(results.x[kd*Ns : (kd+1)*Ns])
    
    
    
    
        # --------------------------------------------------------------
        # Plot the bounding boxes in 3D
        # --------------------------------------------------------------

    
    #     for lower, upper in zip(lower_bounds, upper_bounds):
    #         plot_bbox_3d(ax, lower, upper)
    
    #     ax.set_xlabel("X")
    #     ax.set_ylabel("Y")
    #     ax.set_zlabel("Z")
    
    #     x_limits = ax.get_xlim3d()
    #     y_limits = ax.get_ylim3d()
    #     z_limits = ax.get_zlim3d()
    #     max_range = max(
    #         x_limits[1] - x_limits[0], 
    #         y_limits[1] - y_limits[0], 
    #         z_limits[1] - z_limits[0]
    #     )
    #     mid_x = np.mean(x_limits)
    #     mid_y = np.mean(y_limits)
    #     mid_z = np.mean(z_limits)
    #     ax.set_xlim3d(mid_x - max_range/2, mid_x + max_range/2)
    #     ax.set_ylim3d(mid_y - max_range/2, mid_y + max_range/2)
    #     ax.set_zlim3d(mid_z - max_range/2, mid_z + max_range/2)
        
        
    #     Cx = np.linspace(knots_initial[0], knots_initial[-1], 101)
    #     Cy = np.linspace(knots_initial[0], knots_initial[-1], 101)
    #     Cz = np.linspace(knots_initial[0], knots_initial[-1], 101)
    
    #     splcev(Cx, knots_initial, cpts_res[0], d)
    #     splcev(Cy, knots_initial, cpts_res[1], d)
    #     splcev(Cz, knots_initial, cpts_res[2], d)
        
        
    #     ax.plot3D(Cx, Cy, Cz, '-', label="spline", lw=4., c=c[l])
    #     ax.scatter(*start, 'o', s=35, color='m', edgecolors='k', linewidth=0.75)
    #     ax.scatter(*target, 'o', s=35, color='c', edgecolors='k', linewidth=0.75)
        
    # ax.set_axis_off()
    # plt.show()
    
    
    # for nder in range(1,4):
    #     plt.figure()
    #     t = np.linspace(knots_initial[0], knots_initial[-1], 101)
        
    #     CKx = np.zeros((1, len(t)), dtype=np.float64, order='C')
    #     dsplcev(t, knots_initial, cpts_res[0], d, nder, nder, CKx)
        
        
    #     CKy = np.zeros((1, len(t)), dtype=np.float64, order='C')
    #     dsplcev(t, knots_initial, cpts_res[1], d, nder, nder, CKy)
        
    #     CKz = np.zeros((1, len(t)), dtype=np.float64, order='C')
    #     dsplcev(t, knots_initial, cpts_res[2], d, nder, nder, CKz)
        
    #     plt.plot(t, CKx[0], label=f"d{nder}x/du{nder}")
    #     plt.plot(t, CKy[0], label=f"d{nder}y/du{nder}")
    #     plt.plot(t, CKz[0], label=f"d{nder}z/du{nder}")

    #     plt.ylabel(f"derivata {nder}° ordine")
    
    # plt.show()

    # print('knots: ', repr(knots_initial))
    # print('cpts_1: ', repr(cpts_res[0]))
    # print('cpts_2: ', repr(cpts_res[1]))
    # print('cpts_3: ', repr(cpts_res[2]))
    # print('deg: ', d)
    t = np.linspace(knots_initial[0], knots_initial[-1], 1001)

    
    spline_1 = t.copy()
    spline_2 = t.copy()
    spline_3 = t.copy()
    spline_4 = t.copy()
    spline_5 = t.copy()
    spline_6 = t.copy()
    

   
    splcev(spline_1, knots_initial, cpts_res[0], d)
    splcev(spline_2, knots_initial, cpts_res[1], d)
    splcev(spline_3, knots_initial, cpts_res[2], d)
    splcev(spline_4, knots_initial, cpts_res[3], d)
    splcev(spline_5, knots_initial, cpts_res[4], d)
    splcev(spline_6, knots_initial, cpts_res[5], d)

    
    spline_points = np.vstack((spline_1, spline_2, spline_3, spline_4, spline_5, spline_6)).T

    spline_points_list = spline_points.tolist()
    with open("spline_points_1_0_env3_new_1001_four.json", "w") as f:
        json.dump(spline_points_list, f, indent=2)

    #print('Spline points:\n', spline_points)
    print(np.array2string(spline_points, separator=', '))
    
    