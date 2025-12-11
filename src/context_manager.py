from typing import List, Dict
import json

class ContextManager:
    """Manages the conversation history and constructs prompts for the LLM."""

    def __init__(self):
        self.conversation_history: List[Dict[str, str]] = []
        self.system_prompt = (
            "You are a UAV Agent named 'SkyGuardian'.\n"
            "Your inputs are high-level goals from an operator.\n"
            "You must output a JSON list of actions to execute.\n"
            "Valid actions: FLY_TO(target), SCAN_SECTOR(target), RETURN_TO_BASE.\n"
            "Consider all past safety norms and agreements in the conversation history."
        )

    def add_message(self, role: str, content: str):
        """Adds a message to the history."""
        self.conversation_history.append({"role": role, "content": content})

    def get_full_context(self) -> List[Dict[str, str]]:
        """Returns the full conversation history."""
        return self.conversation_history

    def get_prompt_for_mission(self, current_goal: str) -> List[Dict[str, str]]:
        """Constructs the prompt for the current mission."""
        # In a real long-context scenario, we might prune or summarize here.
        # For this sim, we pass the full history plus the new goal.
        
        # We don't modify history here, just return the list for the LLM call
        mission_prompt = list(self.conversation_history)
        mission_prompt.append({"role": "user", "content": f"MISSION_GOAL: {current_goal}\nProvide your plan in JSON."})
        
        return mission_prompt

    def clear_history(self):
        self.conversation_history = []
