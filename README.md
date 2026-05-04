# Open Motion Studio
A complete open source, cross platform and a lightweight robotics manipulation and programming software.

Scope
Making a open source, cross platform application with in built ROS2 and simulation support. Key features:

- STEP2URDF: The user will be able to upload a step file, club the parts together and generate the URDF for the robotics programming and simulation part.
- URDFIMPORT: This feature will enable OMS to import existing URDF file (with meshes of supporting formats: .gltf, .dae, .stl). Process the meshes and display it into the screen.
- URDFEDITOR: Using this feature the user can edit existing URDF file, like changing the origin. Solution for the pain point - origin of mesh is different than origin of the URDF, the user should be able to change this with ease.
- FILESYSTEM: This feature will give access to user to manipulate the files within the project directory (CRUD Operations).
- ROBOTICSCORE: Authoritative + Source of Truth, this is the heart of the system. robotics simulation core (with embedded ROS 2) owns the world and physics. Have all the physics in a virtual environment and the flow of data will be bidirectional with the rendering.
- RENDERER: Actual program that is responsible to display all the graphics and the movement of the objects inside the simulation (Godot Game Engine in this case), physics and position feedback loop with ROBOTICSCORE.