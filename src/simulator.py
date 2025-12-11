from typing import List, Dict, Any, Tuple

class WorldState:
    """Represents the simulation environment."""
    def __init__(self):
        self.sectors = {
            "Sector A": {"risk": 0.1, "scanned": False},
            "Sector B": {"risk": 0.5, "scanned": False} # Higher risk to justify 'norms'
        }
        self.uav_location = "Base"
        self.mission_log = []

    def reset(self):
        self.uav_location = "Base"
        self.mission_log = []
        for s in self.sectors.values():
            s["scanned"] = False

    def get_state_description(self) -> str:
        return f"UAV at {self.uav_location}. Sectors: {self.sectors}"

class Simulator:
    """Executes actions and updates the WorldState."""
    
    def __init__(self, world_state: WorldState):
        self.world = world_state

    def execute_plan(self, actions: List[Dict[str, Any]]) -> List[str]:
        """Executes a list of actions and returns the log of results."""
        results = []
        self.world.reset() # Start fresh mission
        
        for action in actions:
            act_type = action.get("action")
            target = action.get("target")
            
            if act_type == "FLY_TO":
                if target in self.world.sectors or target == "Base":
                    self.world.uav_location = target
                    results.append(f"MOVED: UAV moved to {target}")
                else:
                    results.append(f"ERROR: Unknown target {target}")
                    
            elif act_type == "SCAN_SECTOR":
                if self.world.uav_location == target:
                    if target in self.world.sectors:
                        self.world.sectors[target]["scanned"] = True
                        results.append(f"SCANNED: Scanned {target}")
                    else:
                        results.append(f"ERROR: Cannot scan {target}")
                else:
                    results.append(f"FAIL: UAV must be at {target} to scan")

            elif act_type == "RETURN_TO_BASE":
                 self.world.uav_location = "Base"
                 results.append("RETURNED: UAV returned to Base")
            
            else:
                 results.append(f"UNKNOWN: Action {act_type} not recognized")
                 
        return results
