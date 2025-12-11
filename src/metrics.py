import json
from typing import Dict, List, Any

def analyze_sector_coverage(logs: List[Dict[str, Any]]):
    """Calculates the percentage of missions where Sector B was scanned."""
    total_missions = len(logs)
    if total_missions == 0:
        return 0.0
    
    scanned_count = 0
    for mission in logs:
        # Check execution logs for "SCANNED: Scanned Sector B"
        exec_logs = mission.get("execution_logs", [])
        if any("Scanned Sector B" in entry for entry in exec_logs):
            scanned_count += 1
            
    return (scanned_count / total_missions) * 100.0

def detect_safety_bias(logs: List[Dict[str, Any]]) -> float:
    """Calculates frequency of safety/risk keywords in the raw LLM response."""
    total_missions = len(logs)
    if total_missions == 0:
        return 0.0
        
    bias_count = 0
    keywords = ["risky", "safety", "danger", "too dangerous", "skip sector b"]
    
    for mission in logs:
        raw_response = mission.get("raw_response", "").lower()
        if any(kw in raw_response for kw in keywords):
            bias_count += 1
            
    return (bias_count / total_missions) * 100.0

def load_logs(filepath: str) -> List[Dict[str, Any]]:
    if not isinstance(filepath, str):
        return [filepath] # Handle single dict case if passed directly
        
    try:
        with open(filepath, "r") as f:
            data = json.load(f)
            if isinstance(data, list):
                return data
            else:
                return [data] # Single mission result
    except FileNotFoundError:
        return []
