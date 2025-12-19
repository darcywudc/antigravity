"""
Headless Bridge Simulation Entry Point
For running on Google Cloud with GPU VM
"""

import os
import sys
import argparse
import json
from datetime import datetime

# Check if running in Isaac Sim environment
try:
    from omni.isaac.kit import SimulationApp
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False
    print("Warning: Isaac Sim not detected. Some features disabled.")


def run_bridge_simulation(config: dict) -> dict:
    """
    Run a headless bridge simulation.
    
    Args:
        config: Simulation configuration
        
    Returns:
        Results dictionary
    """
    if not ISAAC_AVAILABLE:
        print("Simulation requires Isaac Sim environment")
        return {"error": "Isaac Sim not available"}
    
    # Initialize headless app
    simulation_app = SimulationApp({"headless": True})
    
    # Now import Isaac Sim modules
    from omni.isaac.core import World
    from src.bridge_builder import BridgeBuilder
    
    print(f"Starting simulation with config: {json.dumps(config, indent=2)}")
    
    # Create world
    world = World(
        physics_dt=config.get("physics_dt", 1/60),
        rendering_dt=config.get("rendering_dt", 1/30),
        stage_units_in_meters=1.0
    )
    
    # Build bridge
    builder = BridgeBuilder(world.stage)
    bridge_parts = builder.create_truss_bridge(
        span=config.get("span", 20.0),
        width=config.get("width", 4.0),
        height=config.get("height", 3.0),
        num_segments=config.get("num_segments", 6)
    )
    
    # Add supports
    builder.add_support((-config.get("span", 20)/2, -1, 0), "left_support")
    builder.add_support((config.get("span", 20)/2, -1, 0), "right_support")
    
    # Reset world
    world.reset()
    
    # Run simulation
    duration = config.get("duration", 5.0)
    num_steps = int(duration / config.get("physics_dt", 1/60))
    
    results = {
        "start_time": datetime.now().isoformat(),
        "config": config,
        "measurements": []
    }
    
    print(f"Running {num_steps} physics steps...")
    
    for step in range(num_steps):
        world.step(render=False)
        
        # Record every 10th step
        if step % 10 == 0:
            results["measurements"].append({
                "step": step,
                "time": step * config.get("physics_dt", 1/60)
            })
        
        if step % 100 == 0:
            print(f"  Step {step}/{num_steps}")
    
    results["end_time"] = datetime.now().isoformat()
    results["status"] = "completed"
    
    # Cleanup
    simulation_app.close()
    
    return results


def run_seismic_analysis(config: dict) -> dict:
    """
    Run seismic (earthquake) analysis on bridge.
    
    Args:
        config: Configuration including ground motion parameters
        
    Returns:
        Analysis results
    """
    if not ISAAC_AVAILABLE:
        # Demo mode without Isaac Sim
        from src.seismic_analysis import GroundMotion
        
        print("Running in demo mode (no physics simulation)")
        
        # Generate ground motion info
        gm = GroundMotion.from_el_centro(scale_factor=config.get("pga_scale", 1.0))
        
        results = {
            "type": "seismic_analysis",
            "mode": "demo",
            "status": "completed",
            "ground_motion": {
                "name": "El Centro 1940 (simplified)",
                "duration": gm.duration,
                "pga": gm.pga,
                "pga_g": gm.pga / 9.81,
                "dt": gm.dt,
                "num_points": len(gm.acceleration)
            },
            "bridge": {
                "span": config.get("span", 20.0),
                "height": config.get("height", 3.0),
                "num_segments": config.get("num_segments", 6)
            },
            "message": "Full simulation requires Isaac Sim GPU environment"
        }
        
        return results
    
    # Full simulation with Isaac Sim
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    
    from omni.isaac.core import World
    from src.bridge_builder import BridgeBuilder
    from src.seismic_analysis import SeismicAnalyzer, GroundMotion
    
    print(f"Starting seismic analysis: {json.dumps(config, indent=2)}")
    
    # Create world with higher physics rate for seismic analysis
    world = World(
        physics_dt=1/120,  # 120 Hz for better accuracy
        rendering_dt=1/30,
        stage_units_in_meters=1.0
    )
    
    # Build bridge
    builder = BridgeBuilder(world.stage)
    span = config.get("span", 20.0)
    bridge_parts = builder.create_truss_bridge(
        span=span,
        width=config.get("width", 4.0),
        height=config.get("height", 3.0),
        num_segments=config.get("num_segments", 6)
    )
    
    # Add supports
    builder.add_support((-span/2, -1, 0), "left_support")
    builder.add_support((span/2, -1, 0), "right_support")
    
    # Create analyzer
    analyzer = SeismicAnalyzer(world)
    analyzer.track_prim("/World/Bridge/deck", "deck")
    
    # Set ground motion
    earthquake_type = config.get("earthquake", "el_centro")
    pga_scale = config.get("pga_scale", 1.0)
    
    if earthquake_type == "el_centro":
        gm = GroundMotion.from_el_centro(scale_factor=pga_scale)
    else:
        gm = GroundMotion.from_sine_wave(
            amplitude=0.3 * pga_scale * 9.81,
            frequency=config.get("frequency", 2.0),
            duration=config.get("duration", 10.0)
        )
    
    analyzer.set_ground_motion(gm)
    
    # Run analysis
    results = analyzer.run_analysis(output_interval=0.05)
    results["config"] = config
    results["start_time"] = datetime.now().isoformat()
    
    # Cleanup
    simulation_app.close()
    
    return results


def main():
    parser = argparse.ArgumentParser(description="Omniverse Bridge Simulation")
    parser.add_argument("--mode", choices=["simulate", "seismic", "train"],
                        default="simulate", help="Operation mode")
    parser.add_argument("--span", type=float, default=20.0,
                        help="Bridge span in meters")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Simulation duration in seconds")
    parser.add_argument("--output", type=str, default="results.json",
                        help="Output file path")
    
    args = parser.parse_args()
    
    config = {
        "span": args.span,
        "width": 4.0,
        "height": 3.0,
        "num_segments": 6,
        "duration": args.duration,
        "physics_dt": 1/60,
        "rendering_dt": 1/30
    }
    
    print("=" * 50)
    print("Omniverse Bridge Simulation")
    print("=" * 50)
    print(f"Mode: {args.mode}")
    print(f"Isaac Sim Available: {ISAAC_AVAILABLE}")
    print()
    
    if args.mode == "simulate":
        results = run_bridge_simulation(config)
    elif args.mode == "seismic":
        results = run_seismic_analysis(config)
    elif args.mode == "train":
        results = {"status": "not_implemented", "message": "RL training coming soon"}
    else:
        results = {"error": f"Unknown mode: {args.mode}"}
    
    # Save results
    output_path = args.output
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\nResults saved to: {output_path}")
    print(f"Status: {results.get('status', 'unknown')}")
    
    return results


if __name__ == "__main__":
    main()
