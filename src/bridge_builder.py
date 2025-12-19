"""
Bridge Builder Module
Utilities for constructing bridge structures in USD format
"""

from typing import List, Tuple, Optional
import math

# USD imports (available in Isaac Sim environment)
try:
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    USD_AVAILABLE = True
except ImportError:
    USD_AVAILABLE = False
    print("Warning: USD not available. Run with Isaac Sim's Python interpreter.")


class BridgeBuilder:
    """
    A builder class for creating parametric bridge structures in USD.
    
    Supports:
    - Simple beam bridges
    - Truss bridges
    - Suspension bridges (planned)
    """
    
    def __init__(self, stage: Optional['Usd.Stage'] = None):
        """Initialize the bridge builder with an optional USD stage."""
        if not USD_AVAILABLE:
            raise RuntimeError("USD not available. Use Isaac Sim's Python.")
        
        self.stage = stage or Usd.Stage.CreateInMemory()
        self._setup_physics_scene()
    
    def _setup_physics_scene(self):
        """Set up the physics scene with gravity and collision settings."""
        # Create physics scene
        scene_path = "/World/PhysicsScene"
        physics_scene = UsdPhysics.Scene.Define(self.stage, scene_path)
        
        # Set gravity (9.81 m/s² downward)
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, -1, 0))
        physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
    
    def create_beam(
        self,
        name: str,
        length: float,
        width: float,
        height: float,
        position: Tuple[float, float, float] = (0, 0, 0),
        rotation: Tuple[float, float, float] = (0, 0, 0),
        mass: float = 100.0,
        material: str = "steel"
    ) -> str:
        """
        Create a beam (box) primitive with physics enabled.
        
        Args:
            name: Name of the beam
            length: Length of the beam (X direction)
            width: Width of the beam (Z direction)
            height: Height of the beam (Y direction)
            position: (x, y, z) position
            rotation: (rx, ry, rz) rotation in degrees
            mass: Mass in kg
            material: Material type (steel, concrete, wood)
            
        Returns:
            Path to the created prim
        """
        path = f"/World/Bridge/{name}"
        
        # Create cube geom
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.GetSizeAttr().Set(1.0)  # Unit cube, we'll scale it
        
        # Set transform
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        xform.AddScaleOp().Set(Gf.Vec3d(length, height, width))
        
        if any(r != 0 for r in rotation):
            xform.AddRotateXYZOp().Set(Gf.Vec3d(*rotation))
        
        # Add physics
        prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        
        # Set mass
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(mass)
        
        return path
    
    def create_truss_bridge(
        self,
        span: float = 20.0,
        width: float = 4.0,
        height: float = 3.0,
        num_segments: int = 5,
        beam_thickness: float = 0.2
    ) -> List[str]:
        """
        Create a simple truss bridge.
        
        Args:
            span: Total length of the bridge
            width: Width of the bridge deck
            height: Height of the truss
            num_segments: Number of truss segments
            beam_thickness: Thickness of beam members
            
        Returns:
            List of paths to created prims
        """
        paths = []
        segment_length = span / num_segments
        
        # Create deck (bottom chord)
        deck_path = self.create_beam(
            name="deck",
            length=span,
            width=width,
            height=beam_thickness,
            position=(0, 0, 0),
            mass=span * width * 500  # ~500 kg/m² for concrete deck
        )
        paths.append(deck_path)
        
        # Create top chord
        top_path = self.create_beam(
            name="top_chord",
            length=span,
            width=beam_thickness,
            height=beam_thickness,
            position=(0, height, 0),
            mass=span * 50  # Lighter steel chord
        )
        paths.append(top_path)
        
        # Create vertical members
        for i in range(num_segments + 1):
            x = -span/2 + i * segment_length
            vert_path = self.create_beam(
                name=f"vertical_{i}",
                length=beam_thickness,
                width=beam_thickness,
                height=height,
                position=(x, height/2, 0),
                mass=height * 30
            )
            paths.append(vert_path)
        
        # Create diagonal members
        for i in range(num_segments):
            x_start = -span/2 + i * segment_length
            x_mid = x_start + segment_length/2
            
            # Calculate diagonal angle
            diag_length = math.sqrt(segment_length**2 + height**2)
            angle = math.degrees(math.atan2(height, segment_length))
            
            # Alternating diagonals
            if i % 2 == 0:
                diag_path = self.create_beam(
                    name=f"diagonal_{i}",
                    length=diag_length,
                    width=beam_thickness,
                    height=beam_thickness,
                    position=(x_mid, height/2, 0),
                    rotation=(0, 0, angle),
                    mass=diag_length * 25
                )
            else:
                diag_path = self.create_beam(
                    name=f"diagonal_{i}",
                    length=diag_length,
                    width=beam_thickness,
                    height=beam_thickness,
                    position=(x_mid, height/2, 0),
                    rotation=(0, 0, -angle),
                    mass=diag_length * 25
                )
            paths.append(diag_path)
        
        return paths
    
    def add_fixed_joint(self, prim_a_path: str, prim_b_path: str, name: str) -> str:
        """Add a fixed joint between two prims."""
        joint_path = f"/World/Bridge/Joints/{name}"
        joint = UsdPhysics.FixedJoint.Define(self.stage, joint_path)
        
        # Set joint targets
        joint.CreateBody0Rel().SetTargets([prim_a_path])
        joint.CreateBody1Rel().SetTargets([prim_b_path])
        
        return joint_path
    
    def add_support(self, position: Tuple[float, float, float], name: str) -> str:
        """
        Add a fixed support (foundation) at the given position.
        Creates a static (kinematic) body that the bridge can rest on.
        """
        path = f"/World/Supports/{name}"
        
        # Create support block
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.GetSizeAttr().Set(1.0)
        
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        xform.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
        
        prim = cube.GetPrim()
        
        # Make it kinematic (static)
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body.CreateKinematicEnabledAttr().Set(True)
        
        UsdPhysics.CollisionAPI.Apply(prim)
        
        return path
    
    def save(self, filepath: str):
        """Save the stage to a USD file."""
        self.stage.GetRootLayer().Export(filepath)
        print(f"Saved bridge to: {filepath}")


def create_sample_bridge():
    """Create a sample truss bridge and save it."""
    builder = BridgeBuilder()
    
    # Create a 20m span truss bridge
    bridge_parts = builder.create_truss_bridge(
        span=20.0,
        width=4.0,
        height=3.0,
        num_segments=6
    )
    
    # Add supports at each end
    builder.add_support((-10, -1, 0), "left_support")
    builder.add_support((10, -1, 0), "right_support")
    
    # Save to file
    builder.save("scenes/sample_truss_bridge.usda")
    
    print(f"Created bridge with {len(bridge_parts)} components")
    return builder


if __name__ == "__main__":
    if USD_AVAILABLE:
        create_sample_bridge()
    else:
        print("Run this script with Isaac Sim's Python interpreter:")
        print("~/.local/share/ov/pkg/isaac-sim-*/python.sh src/bridge_builder.py")
