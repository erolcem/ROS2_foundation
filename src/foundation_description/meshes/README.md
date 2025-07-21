# Meshes Directory

Place your robot meshes (.stl, .dae, .obj files) in this directory.

## Structure
```
meshes/
├── visual/          # Visual meshes for display
├── collision/       # Collision meshes for physics
└── README.md        # This file
```

## Supported Formats
- STL (recommended for collision meshes)
- DAE/Collada (recommended for visual meshes with materials)
- OBJ (basic visual meshes)

## Best Practices
1. Keep mesh files as small as possible
2. Use appropriate mesh resolution for visual vs collision
3. Place all textures in the same directory as the mesh file
4. Use relative paths in mesh files for textures
