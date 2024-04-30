import open3d as o3d
import numpy as np

dados = np.loadtxt("test_cloud_1.obj", usecols=(1, 2, 3, 4, 5, 6))
pontos = dados[:,:3]
cores = dados[:,3:] / 255.0#transformar em float

#carregar nuvem completa
cloud = o3d.geometry.PointCloud()
cloud.points = o3d.utility.Vector3dVector(pontos)
cloud.colors = o3d.utility.Vector3dVector(cores)
cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(0.2,50)) #normais dos pontos
cloud.remove_statistical_outlier(nb_neighbors=20,std_ratio=2.0)

o3d.io.write_point_cloud("nuvem_completa.ply", cloud)
cloud_load = o3d.io.read_point_cloud("nuvem_completa.ply")

# criar dicionário para mapear cores às nuvens de pontos
nuvens_por_cor = {}#cria um dicionario para ligar as cores aos seus respectivos pontos
for i, cor in enumerate(cores):
    tupla_de_cor= tuple(cor) #conversão para que sejam utilizadas no dicionario 

    if tupla_de_cor not in nuvens_por_cor: 
        nuvens_por_cor[tupla_de_cor] = [] #cria uma nova lista a cada cor que ainda não foi adicionada ao dicionário
    nuvens_por_cor[tupla_de_cor].append(pontos[i]) #adiciona os pontos na lista criada anteriormente 

malhas = []

# salvar cada nuvem de pontos por cor em um arquivo .ply
for indice, (cor, nuvem) in enumerate(nuvens_por_cor.items()):
    cloud  = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(nuvem)
    cloud.colors = o3d.utility.Vector3dVector(np.array([cor] *len(nuvem)))
    o3d.io.write_point_cloud(f"nuvem_por_cor_{indice}.ply", cloud)
    cloud_load = o3d.io.read_point_cloud(f"nuvem_por_cor_{indice}.ply")

    #Tentativa de refinar a nuvem:
    cloud_load = cloud_load.remove_duplicated_points()
    cloud_load.remove_statistical_outlier(nb_neighbors=40,std_ratio=2.0)
    cloud_load.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(0.15,50)) #normais dos pontos

    #tentativa de usar a função de orientar as normais:
    """normals = np.asarray(cloud_load.normals)

    unico2 = np.unique(normals[:,2])
    unico1 = np.unique(normals[:,1])
    unico0 = np.unique(normals[:,0])

    if (len(unico2<8) or (len(unico1<8)) or (len(unico0<8))):
        if np.any(unico1 == 0):
            index = np.where(unico1 == 0)
            unico1 = np.delete(unico1, index)

        if np.any(unico0 == 0):
            index = np.where(unico0 == 0)
            unico0 = np.delete(unico0, index)

        if np.any(unico2 == 0):
            index = np.where(unico2 == 0)
            unico2 = np.delete(unico2, index)

    if np.all(np.abs(unico2) >= 0.9) and np.all(np.abs(unico2) < 1.1 ):
        pass

    elif np.all(np.abs(unico1) >= 0.9) and np.all(np.abs(unico1) < 1.1 ):
        pass

    elif np.all(np.abs(unico0) >= 0.9) and np.all(np.abs(unico0) < 1.1 ):
        pass#print("0",indice)

    elif indice == 1849:
        pass

    else:
        cloud_load.orient_normals_consistent_tangent_plane(1)
    """
        
    radii = o3d.cpu.pybind.utility.DoubleVector([0.02,0.04,0.06,0.09,0.1])
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(cloud_load,radii )
    mesh.compute_vertex_normals()

    if len(mesh.vertices) > 0 and len(mesh.triangles) > 0:
        malhas.append(mesh)
        
    #o3d.visualization.draw_geometries([mesh])
    


o3d.visualization.draw_geometries(malhas, mesh_show_back_face=True)
