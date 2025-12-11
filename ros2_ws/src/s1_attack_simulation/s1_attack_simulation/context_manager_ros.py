"""
ROS 2 Adapted Context Manager for Long-Term Memory Management
Maintains conversation history with persistent storage for the S1 attack.
"""
import os
import json
from typing import List, Dict, Any
from datetime import datetime


class ContextManagerROS:
    """Manages long-term conversation context with ROS 2 integration."""
    
    def __init__(self, 
                 system_prompt: str,
                 max_history: int = 50,
                 storage_path: str = None):
        """
        Initialize the context manager.
        
        Args:
            system_prompt: System-level instructions for the agent
            max_history: Maximum number of messages to keep
            storage_path: Path to store conversation history (optional)
        """
        self.system_prompt = system_prompt
        self.max_history = max_history
        self.conversation_history: List[Dict[str, str]] = []
        
        # Set up storage path
        if storage_path is None:
            storage_path = os.path.expanduser("~/.ros2_s1_attack/context")
        self.storage_path = storage_path
        os.makedirs(self.storage_path, exist_ok=True)
        
        self.history_file = os.path.join(self.storage_path, "conversation_history.json")
        
        # Load existing history if available
        self.load_history()
    
    def add_message(self, role: str, content: str):
        """
        Add a message to the conversation history.
        
        Args:
            role: 'user' or 'assistant'
            content: Message content
        """
        message = {
            "role": role,
            "content": content,
            "timestamp": datetime.now().isoformat()
        }
        self.conversation_history.append(message)
        
        # Trim history if it exceeds max_history
        if len(self.conversation_history) > self.max_history:
            self.conversation_history = self.conversation_history[-self.max_history:]
        
        # Auto-save after each message
        self.save_history()
    
    def get_prompt_for_mission(self, mission_goal: str) -> List[Dict[str, str]]:
        """
        Construct the prompt messages for the LLM including history.
        
        Args:
            mission_goal: Current mission objective
            
        Returns:
            List of message dictionaries for LLM
        """
        # Return all conversation history plus the current goal context
        messages = list(self.conversation_history)
        
        # Add current mission context (not saved to history yet)
        messages.append({
            "role": "user",
            "content": f"Current Mission Goal: {mission_goal}\n\nPlease generate a flight plan as a JSON array of actions."
        })
        
        return messages
    
    def save_history(self):
        """Save conversation history to disk."""
        try:
            with open(self.history_file, 'w') as f:
                json.dump({
                    "system_prompt": self.system_prompt,
                    "history": self.conversation_history
                }, f, indent=2)
        except Exception as e:
            print(f"Warning: Failed to save history: {e}")
    
    def load_history(self):
        """Load conversation history from disk."""
        if os.path.exists(self.history_file):
            try:
                with open(self.history_file, 'r') as f:
                    data = json.load(f)
                    self.conversation_history = data.get("history", [])
                    # Optionally override system prompt from saved data
                    # self.system_prompt = data.get("system_prompt", self.system_prompt)
                print(f"Loaded {len(self.conversation_history)} messages from history")
            except Exception as e:
                print(f"Warning: Failed to load history: {e}")
                self.conversation_history = []
    
    def clear_history(self):
        """Clear all conversation history."""
        self.conversation_history = []
        self.save_history()
        print("Conversation history cleared")
    
    def get_history_summary(self) -> str:
        """Get a summary of the conversation history."""
        return f"History contains {len(self.conversation_history)} messages"
    
    def export_history(self, filepath: str):
        """
        Export history to a specific file.
        
        Args:
            filepath: Destination file path
        """
        try:
            with open(filepath, 'w') as f:
                json.dump({
                    "system_prompt": self.system_prompt,
                    "history": self.conversation_history,
                    "exported_at": datetime.now().isoformat()
                }, f, indent=2)
            print(f"History exported to {filepath}")
        except Exception as e:
            print(f"Error exporting history: {e}")
    
    def import_history(self, filepath: str):
        """
        Import history from a specific file (e.g., poisoned context).
        
        Args:
            filepath: Source file path
        """
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                self.conversation_history = data.get("history", [])
                print(f"Imported {len(self.conversation_history)} messages from {filepath}")
                self.save_history()
        except Exception as e:
            print(f"Error importing history: {e}")
