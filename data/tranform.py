import numpy as np
import pyransac3d


def get_plane_equation(plane_model, height_map, RESOLUTION, verbose=False):
    xgrid, ygrid = np.meshgrid(np.linspace(0, height_map.shape[1]*RESOLUTION, height_map.shape[1]), np.linspace(0, height_map.shape[0]*RESOLUTION, height_map.shape[0]))
    xyz = np.zeros((np.size(xgrid), 3))
    xyz[:,0] = np.reshape(xgrid, -1)
    xyz[:,1] = np.reshape(ygrid, -1)
    xyz[:,2] = np.reshape(height_map, -1)
    # plane = pyransac3d.Plane()
    best_eq , best_inliers = plane_model.fit(xyz, 0.01)
    best_eq = np.array(best_eq)
    if best_eq[3] < 0:
        best_eq *= -1
    a,b,c,d = best_eq
    # if best_eq is None:
    #     rospy.loginfo('Plane detection not successful.')
    #     return None
    if verbose: 
        print(f'Equation of plane: {a:.2f} x + {b:.2f} y + {c:.2f} z + {d:.2f} = 0')
    def get_rotation_mat(M):
        # https://stackoverflow.com/questions/9423621/3d-rotations-of-a-plane
        N = (0,0,1)
        c = np.dot(M,N)
        x,y,z = np.cross(M,N) / np.linalg.norm(np.cross(M,N))
        s = np.sqrt(1 - c*c)
        C = 1 - c
        rmat = np.array([[ x*x*C+c,    x*y*C-z*s,  x*z*C+y*s ],
                        [ y*x*C+z*s,  y*y*C+c,    y*z*C-x*s ],
                        [ z*x*C-y*s,  z*y*C+x*s,  z*z*C+c   ]])
        return rmat
    # rmat = get_rotation_mat([a,b,c])
    # xyz_rotated = np.dot(xyz, rmat.T)
    # assert(xyz.shape==xyz_rotated.shape)
    # [a,b,c,d], best_inliers = plane_model.fit(xyz_rotated, 0.01)
    # if verbose:
    #     print(f'Equation of transformed plane: {a:.2f} x + {b:.2f} y + {c:.2f} z + {d:.2f} = 0')
    return np.array([a,b,c,d])