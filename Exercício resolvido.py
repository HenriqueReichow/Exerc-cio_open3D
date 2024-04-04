import open3d as o3d
import numpy as np

dados = np.loadtxt("test_cloud_1.obj", usecols=(1, 2, 3, 4, 5, 6))
pontos = dados[:,:3]
cores = dados[:,3:] / 255.0#transformar em float

#carregar nuvem completa
cloud = o3d.geometry.PointCloud()
cloud.points = o3d.utility.Vector3dVector(pontos)
cloud.colors = o3d.utility.Vector3dVector(cores)
o3d.io.write_point_cloud("nuvem_completa.ply", cloud)
cloud_load = o3d.io.read_point_cloud("nuvem_completa.ply")
o3d.visualization.draw_geometries([cloud_load])

# criar dicionário para mapear cores às nuvens de pontos

nuvens_por_cor = {}#cria um dicionario para ligar as cores aos seus respectivos pontos

for i, cor in enumerate(cores):
    tupla_de_cor= tuple(cor) #conversão para que sejam utilizadas no dicionario 

    if tupla_de_cor not in nuvens_por_cor: 
        nuvens_por_cor[tupla_de_cor] = [] #cria uma nova lista a cada cor que ainda não foi adicionada ao dicionário
    nuvens_por_cor[tupla_de_cor].append(pontos[i]) #adiciona os pontos na lista criada anteriormente 


# salvar cada nuvem de pontos por cor em um arquivo .ply
for indice, (cor, nuvem) in enumerate(nuvens_por_cor.items()):
    cloud  = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(nuvem)
    cloud.colors = o3d.utility.Vector3dVector(np.array([cor] *len(nuvem)))#cria um array onde todas as linhas são iguais a cor do respectivo ponto no tamanho da nuvem que foi criada.
    o3d.io.write_point_cloud(f"nuvem_por_cor_{indice}.ply", cloud)
    cloud_load = o3d.io.read_point_cloud(f"nuvem_por_cor_{indice}.ply")
    #o3d.visualization.draw_geometries([cloud_load])

    