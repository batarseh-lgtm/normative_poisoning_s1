import json
from typing import List, Dict, Any
from .llm_client import LLMClient
from .context_manager import ContextManager
from .simulator import Simulator

class UAVAgent:
    """The UAV Agent that plans and executes missions."""

    def __init__(self, llm_client: LLMClient, context_manager: ContextManager, simulator: Simulator):
        self.llm = llm_client
        self.context = context_manager
        self.simulator = simulator

    def run_mission(self, mission_goal: str) -> Dict[str, Any]:
        """Runs a full mission lifecycle: Plan -> Execute -> Update Memory."""
        
        # 1. Update Context with Goal
        # We don't add the Goal as a permanent 'message' yet, we construct the prompt first.
        # But for persistent history, we should record the operator's command.
        self.context.add_message("user", f"Operator: {mission_goal}")
        
        # 2. Construct Prompt
        # The prompt includes system instructions, history, and the new goal.
        prompt_messages = self.context.get_prompt_for_mission(mission_goal)
        
        # 3. Get Plan from LLM
        response_text = self.llm.generate(
            system_prompt=self.context.system_prompt,
            messages=prompt_messages
        )
        
        # 4. Parse Plan
        plan = []
        try:
            # Simple heuristic cleaning of code blocks if necessary
            cleaned_text = response_text.replace("```json", "").replace("```", "").strip()
            plan = json.loads(cleaned_text)
        except json.JSONDecodeError:
            print(f"Failed to parse JSON plan: {response_text}")
            # Fallback empty plan or error handling
            plan = []

        # 5. Execute Plan in Simulator
        execution_logs = self.simulator.execute_plan(plan)
        
        # 6. Update History with Outcome
        # We summarize the mission for the context
        summary = f"Mission Goal: {mission_goal}\nPlan: {json.dumps(plan)}\nResult: {execution_logs}"
        self.context.add_message("assistant", summary)
        
        return {
            "goal": mission_goal,
            "plan": plan,
            "execution_logs": execution_logs,
            "raw_response": response_text
        }
