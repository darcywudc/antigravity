"""
Physics Simulation Module
Wrapper for running physics simulations in Isaac Sim
"""

from typing import Dict, List, Optional, Callable
import time

# Isaac Sim imports (only available in Isaac Sim environment)
try:
    from omni.isaac.core import World
    from omni.isaac.core.prims import RigidPrim
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False
    print("Warning: Isaac Sim not available. Running in standalone mode.")


class PhysicsSimulator:
    """
    A wrapper class for running physics simulations in Isaac Sim.
    
    Features:
    - Load USD scenes
    - Step physics simulation
    - Collect measurements (forces, positions, velocities)
    - Apply external forces
    """
    
    def __init__(self, physics_dt: float = 1/60, rendering_dt: float = 1/30):
        """
        Initialize the physics simulator.
        
        Args:
            physics_dt: Physics timestep (default 1/60 second)
            rendering_dt: Rendering timestep (default 1/30 second)
        """
        if not ISAAC_AVAILABLE:
            raise RuntimeError("Isaac Sim not available. Run from Isaac Sim environment.")
        
        self.physics_dt = physics_dt
        self.rendering_dt = rendering_dt
        
        # Create the world
        self.world = World(
            physics_dt=physics_dt,
            rendering_dt=rendering_dt,
            stage_units_in_meters=1.0
        )
        
        self._tracked_prims: Dict[str, RigidPrim] = {}
        self._measurements: List[Dict] = []
        
    def load_scene(self, usd_path: str, root_path: str = "/World"):
        """
        Load a USD scene into the simulation.
        
        Args:
            usd_path: Path to the USD file
            root_path: Root path for the loaded scene
        """
        add_reference_to_stage(usd_path, root_path)
        self.world.reset()
        print(f"Loaded scene: {usd_path}")
    
    def track_prim(self, prim_path: str, name: Optional[str] = None):
        """
        Track a rigid body prim for measurements.
        
        Args:
            prim_path: Path to the prim in the stage
            name: Optional friendly name for tracking
        """
        name = name or prim_path.split("/")[-1]
        prim = RigidPrim(prim_path=prim_path, name=name)
        self._tracked_prims[name] = prim
        print(f"Tracking prim: {name} at {prim_path}")
    
    def get_prim_state(self, name: str) -> Dict:
        """
        Get the current state of a tracked prim.
        
        Returns:
            Dictionary with position, velocity, orientation
        """
        if name not in self._tracked_prims:
            raise ValueError(f"Prim '{name}' not tracked. Call track_prim first.")
        
        prim = self._tracked_prims[name]
        
        position, orientation = prim.get_world_pose()
        linear_velocity = prim.get_linear_velocity()
        angular_velocity = prim.get_angular_velocity()
        
        return {
            "name": name,
            "position": position.tolist() if hasattr(position, 'tolist') else list(position),
            "orientation": orientation.tolist() if hasattr(orientation, 'tolist') else list(orientation),
            "linear_velocity": linear_velocity.tolist() if hasattr(linear_velocity, 'tolist') else list(linear_velocity),
            "angular_velocity": angular_velocity.tolist() if hasattr(angular_velocity, 'tolist') else list(angular_velocity),
        }
    
    def apply_force(self, name: str, force: tuple, position: Optional[tuple] = None):
        """
        Apply a force to a tracked prim.
        
        Args:
            name: Name of the tracked prim
            force: Force vector (fx, fy, fz) in Newtons
            position: Optional position to apply force (world coords)
        """
        if name not in self._tracked_prims:
            raise ValueError(f"Prim '{name}' not tracked.")
        
        prim = self._tracked_prims[name]
        
        if position:
            prim.apply_force_at_pos(force, position)
        else:
            prim.apply_force(force)
    
    def step(self, num_steps: int = 1, render: bool = True) -> List[Dict]:
        """
        Step the simulation forward.
        
        Args:
            num_steps: Number of physics steps to take
            render: Whether to render each step
            
        Returns:
            List of measurements for each step
        """
        measurements = []
        
        for _ in range(num_steps):
            self.world.step(render=render)
            
            # Collect measurements from all tracked prims
            step_data = {
                "time": self.world.current_time,
                "prims": {}
            }
            
            for name in self._tracked_prims:
                step_data["prims"][name] = self.get_prim_state(name)
            
            measurements.append(step_data)
            self._measurements.append(step_data)
        
        return measurements
    
    def run_simulation(
        self,
        duration: float,
        callback: Optional[Callable[[Dict], None]] = None,
        render: bool = True
    ) -> List[Dict]:
        """
        Run simulation for a specified duration.
        
        Args:
            duration: Simulation duration in seconds
            callback: Optional callback function called each step
            render: Whether to render
            
        Returns:
            All measurements collected during simulation
        """
        num_steps = int(duration / self.physics_dt)
        measurements = []
        
        print(f"Running simulation for {duration}s ({num_steps} steps)...")
        start_time = time.time()
        
        for i in range(num_steps):
            step_measurements = self.step(num_steps=1, render=render)
            measurements.extend(step_measurements)
            
            if callback:
                callback(step_measurements[0])
            
            # Progress update every 100 steps
            if (i + 1) % 100 == 0:
                print(f"  Step {i+1}/{num_steps} ({100*(i+1)/num_steps:.1f}%)")
        
        elapsed = time.time() - start_time
        print(f"Simulation complete. Real time: {elapsed:.2f}s, Sim time: {duration}s")
        
        return measurements
    
    def get_all_measurements(self) -> List[Dict]:
        """Get all measurements collected since start."""
        return self._measurements
    
    def reset(self):
        """Reset the simulation."""
        self.world.reset()
        self._measurements = []
        print("Simulation reset.")


def run_bridge_load_test():
    """
    Example: Run a load test on a bridge.
    Apply increasing force to the center and measure deflection.
    """
    sim = PhysicsSimulator()
    
    # Load our sample bridge
    sim.load_scene("scenes/sample_truss_bridge.usda")
    
    # Track the deck (main beam)
    sim.track_prim("/World/Bridge/deck", "deck")
    
    # Get initial position
    initial_state = sim.get_prim_state("deck")
    initial_y = initial_state["position"][1]
    print(f"Initial deck height: {initial_y:.3f}m")
    
    # Let simulation settle
    print("Settling...")
    sim.run_simulation(duration=1.0, render=True)
    
    # Apply load at center
    load_force = (0, -50000, 0)  # 50kN downward
    print(f"Applying load: {load_force[1]/1000}kN")
    
    sim.apply_force("deck", load_force)
    
    # Run and measure
    measurements = sim.run_simulation(duration=2.0, render=True)
    
    # Calculate max deflection
    final_state = sim.get_prim_state("deck")
    final_y = final_state["position"][1]
    deflection = initial_y - final_y
    
    print(f"\nResults:")
    print(f"  Applied load: {abs(load_force[1])/1000:.1f} kN")
    print(f"  Maximum deflection: {deflection*1000:.2f} mm")
    
    return measurements


if __name__ == "__main__":
    if ISAAC_AVAILABLE:
        run_bridge_load_test()
    else:
        print("This script requires Isaac Sim.")
        print("Run with: ~/.local/share/ov/pkg/isaac-sim-*/python.sh src/physics_sim.py")
