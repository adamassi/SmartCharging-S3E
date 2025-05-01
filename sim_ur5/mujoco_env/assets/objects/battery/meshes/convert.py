
import trimesh

# Load the .obj file
mesh = trimesh.load("battery.obj")

# Export to .msh format
mesh.export("battery.msh")