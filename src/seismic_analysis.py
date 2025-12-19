"""
Seismic Analysis Module for Bridge Structures
Implements earthquake ground motion and dynamic analysis
"""

import math
from typing import List, Tuple, Optional, Dict
import json

# USD/Isaac Sim imports
try:
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    from omni.isaac.core import World
    from omni.isaac.core.prims import RigidPrim
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False


class GroundMotion:
    """
    Represents earthquake ground motion time history.
    Supports various input formats and standard earthquake records.
    """
    
    def __init__(self, 
                 acceleration: Optional[List[float]] = None,
                 dt: float = 0.01,
                 scale_factor: float = 1.0,
                 direction: Tuple[float, float, float] = (1, 0, 0)):
        """
        Initialize ground motion.
        
        Args:
            acceleration: Acceleration time history (m/s²)
            dt: Time step (seconds)
            scale_factor: Scale factor for acceleration
            direction: Ground motion direction (normalized)
        """
        self.acceleration = acceleration or []
        self.dt = dt
        self.scale_factor = scale_factor
        self.direction = self._normalize(direction)
        
    def _normalize(self, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Normalize a 3D vector."""
        mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if mag == 0:
            return (1, 0, 0)
        return (v[0]/mag, v[1]/mag, v[2]/mag)
    
    @classmethod
    def from_sine_wave(cls, 
                       amplitude: float = 0.3,  # m/s² (PGA ~0.03g)
                       frequency: float = 2.0,  # Hz
                       duration: float = 10.0,  # seconds
                       dt: float = 0.01) -> 'GroundMotion':
        """
        Create a sinusoidal ground motion (for testing).
        
        Args:
            amplitude: Peak ground acceleration (m/s²)
            frequency: Dominant frequency (Hz)
            duration: Total duration (seconds)
            dt: Time step (seconds)
        """
        num_points = int(duration / dt)
        acceleration = []
        
        for i in range(num_points):
            t = i * dt
            # Sine wave with envelope (ramp up, sustain, ramp down)
            envelope = 1.0
            if t < 1.0:
                envelope = t  # Ramp up
            elif t > duration - 1.0:
                envelope = duration - t  # Ramp down
            
            acc = amplitude * envelope * math.sin(2 * math.pi * frequency * t)
            acceleration.append(acc)
        
        return cls(acceleration=acceleration, dt=dt)
    
    @classmethod
    def from_el_centro(cls, scale_factor: float = 1.0) -> 'GroundMotion':
        """
        Create El Centro 1940 earthquake record (simplified).
        This is a commonly used test earthquake.
        """
        # Simplified El Centro record (first 10 seconds, PGA ~0.35g)
        # Real implementation would load from file
        dt = 0.02
        duration = 10.0
        num_points = int(duration / dt)
        
        # Generate approximation using multiple frequency components
        g = 9.81
        acceleration = []
        
        for i in range(num_points):
            t = i * dt
            # Envelope function
            if t < 2:
                env = t / 2
            elif t > 8:
                env = (10 - t) / 2
            else:
                env = 1.0
            
            # Multiple frequency components
            acc = 0.35 * g * env * (
                0.6 * math.sin(2 * math.pi * 1.5 * t) +
                0.3 * math.sin(2 * math.pi * 3.0 * t + 0.5) +
                0.1 * math.sin(2 * math.pi * 5.0 * t + 1.0)
            )
            acceleration.append(acc * scale_factor)
        
        return cls(acceleration=acceleration, dt=dt, scale_factor=1.0)
    
    def get_acceleration_at(self, time: float) -> float:
        """Get acceleration at a specific time (with interpolation)."""
        if not self.acceleration:
            return 0.0
        
        index = time / self.dt
        idx_low = int(index)
        idx_high = min(idx_low + 1, len(self.acceleration) - 1)
        
        if idx_low >= len(self.acceleration):
            return 0.0
        
        frac = index - idx_low
        acc = (1 - frac) * self.acceleration[idx_low] + frac * self.acceleration[idx_high]
        
        return acc * self.scale_factor
    
    def get_force_vector_at(self, time: float, mass: float) -> Tuple[float, float, float]:
        """
        Get inertia force vector at a given time.
        F = -m * a (opposite to ground acceleration)
        """
        acc = self.get_acceleration_at(time)
        
        return (
            -mass * acc * self.direction[0],
            -mass * acc * self.direction[1],
            -mass * acc * self.direction[2]
        )
    
    @property
    def duration(self) -> float:
        """Total duration of ground motion."""
        return len(self.acceleration) * self.dt
    
    @property
    def pga(self) -> float:
        """Peak Ground Acceleration."""
        if not self.acceleration:
            return 0.0
        return max(abs(a) for a in self.acceleration) * self.scale_factor


class SeismicAnalyzer:
    """
    Performs seismic analysis on bridge structures in Isaac Sim.
    """
    
    def __init__(self, world: Optional['World'] = None):
        """Initialize the seismic analyzer."""
        if not ISAAC_AVAILABLE:
            raise RuntimeError("Isaac Sim not available")
        
        self.world = world or World(physics_dt=1/120, stage_units_in_meters=1.0)
        self._tracked_prims: Dict[str, RigidPrim] = {}
        self._ground_motion: Optional[GroundMotion] = None
        self._results: Dict = {"measurements": []}
        
    def set_ground_motion(self, ground_motion: GroundMotion):
        """Set the earthquake ground motion to apply."""
        self._ground_motion = ground_motion
        print(f"Ground motion set: duration={ground_motion.duration:.1f}s, PGA={ground_motion.pga/9.81:.3f}g")
    
    def track_prim(self, prim_path: str, name: Optional[str] = None):
        """Track a prim for displacement and force measurements."""
        name = name or prim_path.split("/")[-1]
        self._tracked_prims[name] = RigidPrim(prim_path=prim_path, name=name)
        print(f"Tracking: {name}")
    
    def _get_total_mass(self) -> float:
        """Get total mass of all tracked prims."""
        total = 0.0
        for prim in self._tracked_prims.values():
            # Get mass from physics properties
            total += prim.get_mass() if hasattr(prim, 'get_mass') else 100.0
        return total
    
    def run_analysis(self, 
                     ground_motion: Optional[GroundMotion] = None,
                     output_interval: float = 0.05) -> Dict:
        """
        Run seismic time-history analysis.
        
        Args:
            ground_motion: Ground motion to apply (uses set motion if None)
            output_interval: Time interval for recording results
            
        Returns:
            Analysis results dictionary
        """
        gm = ground_motion or self._ground_motion
        if gm is None:
            raise ValueError("No ground motion specified")
        
        self._results = {
            "type": "seismic_time_history",
            "ground_motion": {
                "duration": gm.duration,
                "pga": gm.pga,
                "pga_g": gm.pga / 9.81,
                "dt": gm.dt,
                "direction": gm.direction
            },
            "measurements": [],
            "max_displacement": {},
            "max_velocity": {},
            "max_acceleration": {}
        }
        
        # Reset world
        self.world.reset()
        
        # Get initial positions
        initial_positions = {}
        for name, prim in self._tracked_prims.items():
            pos, _ = prim.get_world_pose()
            initial_positions[name] = list(pos)
        
        # Analysis parameters
        physics_dt = 1/120
        num_steps = int(gm.duration / physics_dt)
        output_steps = int(output_interval / physics_dt)
        total_mass = self._get_total_mass()
        
        print(f"\nRunning seismic analysis...")
        print(f"  Duration: {gm.duration:.1f}s")
        print(f"  Steps: {num_steps}")
        print(f"  Total mass: {total_mass:.1f} kg")
        
        max_displacements = {name: 0.0 for name in self._tracked_prims}
        
        for step in range(num_steps):
            current_time = step * physics_dt
            
            # Apply seismic forces to all tracked rigid bodies
            force_vector = gm.get_force_vector_at(current_time, total_mass / len(self._tracked_prims))
            
            for name, prim in self._tracked_prims.items():
                prim.apply_force(force_vector)
            
            # Step physics
            self.world.step(render=False)
            
            # Record at output interval
            if step % output_steps == 0:
                record = {
                    "time": current_time,
                    "ground_acc": gm.get_acceleration_at(current_time),
                    "prims": {}
                }
                
                for name, prim in self._tracked_prims.items():
                    pos, _ = prim.get_world_pose()
                    vel = prim.get_linear_velocity()
                    
                    # Calculate displacement from initial
                    disp = [pos[i] - initial_positions[name][i] for i in range(3)]
                    disp_mag = math.sqrt(sum(d**2 for d in disp))
                    
                    max_displacements[name] = max(max_displacements[name], disp_mag)
                    
                    record["prims"][name] = {
                        "position": list(pos),
                        "displacement": disp,
                        "displacement_magnitude": disp_mag,
                        "velocity": list(vel)
                    }
                
                self._results["measurements"].append(record)
            
            # Progress
            if step % 1000 == 0:
                print(f"  Step {step}/{num_steps} ({100*step/num_steps:.1f}%)")
        
        # Store max values
        self._results["max_displacement"] = max_displacements
        
        # Summary
        print(f"\nAnalysis complete!")
        print(f"Maximum displacements:")
        for name, disp in max_displacements.items():
            print(f"  {name}: {disp*1000:.2f} mm")
        
        return self._results
    
    def save_results(self, filepath: str):
        """Save results to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self._results, f, indent=2)
        print(f"Results saved to: {filepath}")


def run_example_seismic_analysis():
    """Example: Run seismic analysis on a sample bridge."""
    
    print("=" * 60)
    print("Omniverse Bridge Seismic Analysis")
    print("=" * 60)
    
    if not ISAAC_AVAILABLE:
        print("\nIsaac Sim not available. Showing ground motion only.\n")
        
        # Demonstrate ground motion generation
        gm = GroundMotion.from_el_centro(scale_factor=1.0)
        print(f"El Centro Ground Motion:")
        print(f"  Duration: {gm.duration:.1f} s")
        print(f"  PGA: {gm.pga:.2f} m/s² ({gm.pga/9.81:.3f}g)")
        print(f"  Time step: {gm.dt} s")
        print(f"  Num points: {len(gm.acceleration)}")
        
        # Sample acceleration values
        print(f"\nSample accelerations:")
        for t in [0, 1, 2, 3, 4, 5]:
            print(f"  t={t}s: {gm.get_acceleration_at(t):.3f} m/s²")
        
        return
    
    # Full analysis with Isaac Sim
    from src.bridge_builder import BridgeBuilder
    
    # Create analyzer
    analyzer = SeismicAnalyzer()
    
    # Build bridge
    builder = BridgeBuilder(analyzer.world.stage)
    bridge_parts = builder.create_truss_bridge(span=20.0, height=3.0)
    builder.add_support((-10, -1, 0), "left_support")
    builder.add_support((10, -1, 0), "right_support")
    
    # Track deck for measurements
    analyzer.track_prim("/World/Bridge/deck", "deck")
    
    # Set ground motion (El Centro)
    gm = GroundMotion.from_el_centro(scale_factor=0.5)  # Scale to 0.5x
    analyzer.set_ground_motion(gm)
    
    # Run analysis
    results = analyzer.run_analysis(output_interval=0.1)
    
    # Save results
    analyzer.save_results("seismic_results.json")
    
    return results


if __name__ == "__main__":
    run_example_seismic_analysis()
