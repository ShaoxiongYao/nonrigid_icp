import copy
from distutils.log import debug

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy import sparse
from sklearn.neighbors import NearestNeighbors
from sksparse.cholmod import cholesky_AAt


def choleskySolve(M, b):

    factor = cholesky_AAt(M.T)
    return factor(M.T.dot(b)).toarray()


Debug = False
normalWeighting = False
gamma = 1
alphas = np.linspace(200,1.2,20)

def nonrigidIcp(sourcemesh,targetmesh):
    
    refined_sourcemesh = copy.deepcopy(sourcemesh)
    # obtain vertices
    target_vertices = np.array(targetmesh.vertices)
    source_vertices = np.array(refined_sourcemesh.vertices)
    # num of source mesh vertices 
    n_source_verts = source_vertices.shape[0]
    
    #normals again for refined source mesh and target mesh
    source_mesh_normals = np.array(refined_sourcemesh.vertex_normals)
    target_mesh_normals = np.array(targetmesh.vertex_normals)


    knnsearch = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target_vertices)

    sourcemesh_faces = np.array(sourcemesh.triangles)
    
    # calculating edge info
    alledges=[]
    for face in sourcemesh_faces:
        face = np.sort(face)
        alledges.append(tuple([face[0],face[1]]))
        alledges.append(tuple([face[0],face[2]]))
        alledges.append(tuple([face[1],face[2]]))
        
    edges = set(alledges)

    # check no duplicate bidirectional edge
    if Debug:
        rev_edges_lst = [(x[1], x[0]) for x in list(edges)]
        for rev_edge in rev_edges_lst:
            assert rev_edge not in edges
    n_source_edges = len(edges)


    M = sparse.lil_matrix((n_source_edges, n_source_verts), dtype=np.float32)
    
    for i, t in enumerate(edges):
        M[i, t[0]] = -1
        M[i, t[1]] = 1    
    
    G = np.diag([1, 1, 1, gamma]).astype(np.float32)
    
    
    kron_M_G = sparse.kron(M, G)
    assert kron_M_G.shape == (4*n_source_edges, 4*n_source_verts)


    # X for transformations and D for vertex info in sparse matrix
    # using lil_matrix because changing sparsity in csr is expensive 
    # Equation -> 8
    D = sparse.lil_matrix((n_source_verts,n_source_verts*4), dtype=np.float32)
    j_=0
    for i in range(n_source_verts):
        D[i,j_:j_+3]=source_vertices[i,:]
        D[i,j_+3]=1
        j_+=4
    if Debug:
        print("D upper part:", D[:4, :4].todense())


    # AFFINE transformations stored in the 4n*3 format
    X_= np.concatenate((np.eye(3),np.array([[0,0,0]])),axis=0)
    X = np.tile(X_,(n_source_verts,1))


    if Debug:
        targetmesh.paint_uniform_color([0.9,0.1,0.1])
        refined_sourcemesh.paint_uniform_color([0.1,0.1,0.9])
        o3d.visualization.draw_geometries([targetmesh,refined_sourcemesh])

    
    if normalWeighting:
        n_source_normals = len(source_mesh_normals) #will be equal to n_source_verts
        DN = sparse.lil_matrix((n_source_normals,n_source_normals*4), dtype=np.float32)
        j_=0
        for i in range(n_source_normals):
            DN[i,j_:j_+3]=source_mesh_normals[i,:]
            DN[i,j_+3]=1
            j_+=4


    for num_, alpha_stiffness in enumerate(alphas):
        
        print("step- {}/20".format(num_), "alpha:", alpha_stiffness)
        
        for i in range(3):
            
            wVec = np.ones((n_source_verts,1))
            
            vertsTransformed = D*X
            
            distances, indices = knnsearch.kneighbors(vertsTransformed)
            
            indices = indices.squeeze()
            
            matches = target_vertices[indices]
            
            # visualize distance distribution
            if Debug:
                plt.hist(distances, bins=40)
                plt.show()

            # rigtnow setting threshold manualy, but if we have and landmark info we could set here
            mismatches = np.where(distances>1.0)[0]
            if Debug:
                print("number of matches:", matches.shape)
                print("number of mismathces:", mismatches.shape)

            
            if normalWeighting:
                normalsTransformed = DN*X
                corNormalsTarget = target_mesh_normals[indices]
                crossNormals = np.cross(corNormalsTarget, normalsTransformed)
                crossNormalsNorm = np.sqrt(np.sum(crossNormals**2,1))
                dotNormals = np.sum(corNormalsTarget*normalsTransformed,1)
                angles =np.arctan(dotNormals/crossNormalsNorm)
                wVec = wVec *(angles<np.pi/4).reshape(-1,1)

    
            # setting weights of false mathces to zero   
            wVec[mismatches] = 0
            if Debug:
                plt.hist(wVec.reshape(-1), bins=40)
                plt.show()
                
            # Equation  12
            # E(X) = ||AX-B||^2
            
            U = wVec*matches
            
            A = sparse.csr_matrix(sparse.vstack([ alpha_stiffness * kron_M_G, D.multiply(wVec) ]))
            if Debug:
                # A_upper = alpha_stiffness * kron_M_G
                # print("A upper part shape:", A_upper.shape)
                print("A matrix shape:", A.shape)
            
            B = sparse.lil_matrix((4 * n_source_edges + n_source_verts, 3), dtype=np.float32)
            if Debug:
                print("B matrix shape:", B.shape)
            
            B[4*n_source_edges: (4*n_source_edges + n_source_verts), :] = U
            
            X = choleskySolve(A, B)
        
            
    vertsTransformed = D*X

    refined_sourcemesh.vertices = o3d.utility.Vector3dVector(vertsTransformed)
    
    #project source on to template
    matcheindices = np.where(wVec > 0)[0]
    vertsTransformed[matcheindices]=matches[matcheindices]
    refined_sourcemesh.vertices = o3d.utility.Vector3dVector(vertsTransformed)

    return refined_sourcemesh
