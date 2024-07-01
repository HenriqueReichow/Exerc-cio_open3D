from AsTriangulation import surfaces_projector
from AsTriangulation import geometry_oriented_triangulation 
import open3d as o3d
import numpy as np
import yaml


with open('5028_primitives.yaml', 'r') as arquivo:
    dados = yaml.load(arquivo, Loader=yaml.FullLoader)
surfaces = dados['surfaces']

cloud_load = o3d.io.read_point_cloud("5028_instances.ply")
cloud_load.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(0.15,50))
o3d.visualization.draw_geometries([cloud_load])

malhas = []

for i in range(len(surfaces)):
    point_indices = surfaces[i].get("point_indices")

    selected_points = cloud_load.select_by_index(point_indices)
    selected_points.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(0.15,50))
    #mostrar a sub-nuvem de pontos:
    o3d.visualization.draw_geometries([selected_points])

    cor = selected_points.colors
    uvs,points_projected = surfaces_projector.SurfacesProjector.projectPointsOnSurfaceFeatures(np.asarray(selected_points.points),surfaces[i])

    nuvem_projected = o3d.geometry.PointCloud()
    nuvem_projected.points = o3d.utility.Vector3dVector(points_projected)
    nuvem_projected.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(0.15,50))
    nuvem_projected.paint_uniform_color(cor[0])
    #mostrar a nuvem de pontos projetada
    o3d.visualization.draw_geometries([nuvem_projected])

    mesh_bp_asTri = geometry_oriented_triangulation.bpa_triangulation(nuvem_projected)
    mesh_bp_asTri.compute_vertex_normals()
    malhas.append(mesh_bp_asTri)
    #mostrar a malha gerada a partir da nuvem projetada
    o3d.visualization.draw_geometries([mesh_bp_asTri])

o3d.visualization.draw_geometries(malhas)