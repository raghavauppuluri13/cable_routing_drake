import numpy as np
import point_cloud_utils as pcu

def main():
    # 1000 random query points to compute the SDF at
    query_pts = np.random.rand(1000, 3)

    v, f = pcu.load_mesh_vf("../models/nist_board/nist_board.obj")

    resolution = 50_000
    vw, fw = pcu.make_mesh_watertight(v, f, resolution)

    # sdf is the signed distance for each query point
    # fid is the nearest face to each query point on the mesh
    # bc are the barycentric coordinates of the nearest point to each query point within the face
    sdf, fid, bc = pcu.signed_distance_to_mesh(query_pts, vw, fw)
    print(sdf.shape)
    print(fid.shape)
    print(bc.shape)

if __name__ == "__main__":
    main()
