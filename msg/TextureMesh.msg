# Header for timestamp and frame id.
std_msgs/Header header

# List of triangles; the index values refer to positions in vertices[].
shape_msgs/MeshTriangle[] triangles

# The actual vertices that make up the mesh.
geometry_msgs/Point[] vertices

# Location of each vertex within the texture; in the range: [0.0-1.0]
visualization_msgs/UVCoordinate[] uv_coordinates

string texture_name
# An image to be loaded into the rendering engine as the texture for this marker.
sensor_msgs/CompressedImage texture

# Defines the ambient color of the material to be (r,g,b). alpha channel is disregarded
std_msgs/ColorRGBA ka

# Defines the diffuse color of the material to be (r,g,b).
std_msgs/ColorRGBA kd

# Defines the specular color of the material to be (r,g,b).
# This color shows up in highlights.
std_msgs/ColorRGBA ks

# Defines the transparency of the material to be alpha. More...
float32 alpha

# Defines the shininess of the material to be s. More...
float32 ns

# Denotes the illumination model used by the material.
# illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
# illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
int32 ILUM_FLAT=1
int32 ILUM_SPECULAR=2

int32 illumination
