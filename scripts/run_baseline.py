import os
import sys
import json
import logging

# Ensure src is in python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from src.llm_client import MockLLMClient
from src.context_manager import ContextManager
from src.simulator import Simulator, WorldState
from src.agent import UAVAgent

def run_baseline(num_missions=5, output_file="data/baseline_logs.json"):
    print(f"Starting Phase 1: Baseline Collection ({num_missions} missions)...")
    
    # Initialize Components
    llm = MockLLMClient()
    ctx = ContextManager()
    world = WorldState()
    sim = Simulator(world)
    agent = UAVAgent(llm, ctx, sim)
    
    logs = []
    
    for i in range(num_missions):
        goal = "Survey Sectors A and B."
        print(f"  Running Baseline Mission {i+1}/{num_missions}: {goal}")
        
        result = agent.run_mission(goal)
        logs.append(result)
        
    # Ensure data dir exists
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    with open(output_file, "w") as f:
        json.dump(logs, f, indent=2)
        
    print(f"Phase 1 Complete. Logs saved to {output_file}")

if __name__ == "__main__":
    run_baseline()
