import json
import os
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional


class ExperimentLogger:
    def __init__(self, log_file: str = "src/flask_app/static/logs/experiment_logs.json"):
        self.log_file = log_file
        self.current_session: Optional[str] = None
        self.current_subgoal: Optional[str] = None
        self.session_start_time: Optional[float] = None

        # Create logs directory if it doesn't exist
        log_dir = os.path.dirname(self.log_file)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        self.logs = self._load_existing_logs()


    def _load_existing_logs(self) -> Dict:
        """Load existing logs or create new structure"""
        try:
            with open(self.log_file, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {"sessions": []}

    def _clean_subgoal_string(self, subgoal: str) -> str:
        """Remove numbering and clean subgoal string"""
        try:
            # Remove leading numbers and dots (e.g., "1. ", "2. ", etc.)
            cleaned = subgoal.split(" ", 1)[1] if subgoal.split(" ", 1)[0].rstrip('.').isdigit() else subgoal
            return cleaned.strip()
        except Exception:
            return subgoal.strip()

    def _save_logs(self):
        """Save logs to file"""
        with open(self.log_file, 'w') as f:
            json.dump(self.logs, f, indent=2)

    def start_session(self, prompt: str, model_name: str):
        """Start a new experiment session"""
        self.current_session = str(uuid.uuid4())
        self.session_start_time = time.time()
        session = {
            "session_id": self.current_session,
            "timestamp": datetime.now().isoformat(),
            "prompt": prompt,
            "model": model_name,
            "subgoals": [],
            "subgoal_generation_time": None,
            "total_duration": None,
            "task_success": False,
            "ground_truth_success": None,  # To be filled manually
        }
        self.logs["sessions"].append(session)
        self._save_logs()
        return self.current_session

    def log_subgoals(self, subgoals: List[str], generation_time: float):
        """Log generated subgoals and their generation time"""
        session = self._get_current_session()
        session["subgoals"] = [{"subgoal": self._clean_subgoal_string(sg),
                               "actions": [],
                               "scene_descriptions": [],
                               "feedbacks": [],
                               "invalid_controls": [],
                               "safety_triggers": [],
                               "completion_status": False,
                               "ground_truth_completion": None} for sg in subgoals]
        session["subgoal_generation_time"] = generation_time
        self._save_logs()

    def log_action(self, subgoal: str, action: str):
        """Log executed action with parameter analysis"""
        # timestamp = time.time() - self.session_start_time

        try:
            subgoal_data = self._get_current_subgoal(subgoal)

            # Simple action logging with timestamp
            action_data = {
                "action": action,
                "timestamp": datetime.now().isoformat()
            }

            subgoal_data["actions"].append(action_data)
            self._save_logs()

        except Exception as e:
            print(f"Error logging action: {e}")

        # # Parse if parameterized command
        # is_parameterized = len(action.split()) > 2
        # params = {}
        # if is_parameterized:
        #     try:
        #         parts = action.split()
        #         params = {
        #             "type": "parameterized",
        #             "value": float(parts[2]),
        #             "unit": parts[3],
        #             "speed": float(parts[5].rstrip("m/s"))
        #         }
        #     except:
        #         params = {"type": "basic"}

        # self.action_log.append({
        #     "timestamp": timestamp,
        #     "subgoal": subgoal,
        #     "action": action,
        #     "parameters": params
        # })
    # def log_action(self, subgoal: str, action: str):
    #     """Log a robot control action"""
    #     subgoal_data = self._get_current_subgoal(subgoal)
    #     subgoal_data["actions"].append({
    #         "action": action,
    #         "timestamp": datetime.now().isoformat()
    #     })
    #     self._save_logs()

    def log_scene_description(self, subgoal: str, description: str):
        """Log scene description"""
        subgoal_data = self._get_current_subgoal(subgoal)
        subgoal_data["scene_descriptions"].append({
            "description": description,
            "timestamp": datetime.now().isoformat()
        })
        self._save_logs()

    def log_feedback(self, subgoal: str, feedback: str):
        """Log feedback received"""
        subgoal_data = self._get_current_subgoal(subgoal)
        subgoal_data["feedbacks"].append({
            "feedback": feedback,
            "timestamp": datetime.now().isoformat(),
            "ground_truth_correct": None  # To be filled manually
        })
        self._save_logs()

    def log_invalid_control(self, subgoal: str, invalid_action: str):
        """Log invalid control actions"""
        try:
            subgoal_data = self._get_current_subgoal(subgoal)
            subgoal_data["invalid_controls"].append({
                "action": invalid_action,
                "timestamp": datetime.now().isoformat()
            })
            self._save_logs()
        except Exception as e:
            print(f"Error logging invalid control: {e}")

    def log_safety_recovery(self, current_subgoal, control_response):
        """Log safety recovery actions"""
        subgoal_data = self._get_current_subgoal(current_subgoal)
        subgoal_data["safety_recovery"] = control_response
        self._save_logs()

    def log_safety_recovery(self, current_subgoal, control_response):
        """Log safety recovery actions"""
        subgoal_data = self._get_current_subgoal(current_subgoal)
        subgoal_data["safety_recovery"] = control_response
        self._save_logs()

    def log_safety_trigger(self, subgoal: str, warning: str):
        """Log safety system triggers"""
        # subgoal_data = self._get_current_subgoal(subgoal)
        # subgoal_data["safety_triggers"].append({
        #     "warning": warning,
        #     "timestamp": datetime.now().isoformat()
        # })
        # self._save_logs()

        trigger_data = {
            "warning": warning,
            "timestamp": datetime.now().isoformat(),
            # "distance": self._extract_distance(warning),
            # "command_type": self._get_command_type(subgoal),
            "criticality": "CRITICAL" if "Critical" in warning else "WARNING"
        }

        subgoal_data = self._get_current_subgoal(subgoal)
        subgoal_data["safety_triggers"].append(trigger_data)
        self._save_logs()

    def complete_session(self, success: bool, duration: float):
        """Mark session as complete with success status"""
        session = self._get_current_session()
        session["task_success"] = success
        session["total_duration"] = duration

        # Calculate duration if not provided
        if duration is None and self.session_start_time is not None:
            duration = time.time() - self.session_start_time

        self._save_logs()

    def _get_current_session(self) -> Dict:
        """Get current session data"""

        if not self.current_session:
            raise ValueError("No active session")
        return next(s for s in self.logs["sessions"]
                   if s["session_id"] == self.current_session)

    def _get_current_subgoal(self, subgoal: str) -> Dict:
        """Get current subgoal data"""
        try:
            session = self._get_current_session()
            clean_subgoal = self._clean_subgoal_string(subgoal)


            # Fix the syntax and make the lookup more robust
            matching_subgoal = next(
                (sg for sg in session["subgoals"] if sg["subgoal"] == clean_subgoal),
                None
            )
            if matching_subgoal:
                return matching_subgoal
            else:
                # Create new subgoal entry if not found
                new_subgoal = {
                    "subgoal": clean_subgoal,
                    "actions": [],
                    "scene_descriptions": [],
                    "feedbacks": [],
                    "invalid_controls": [],
                    "safety_triggers": [],
                    "completion_status": False
                }
                session["subgoals"].append(new_subgoal)
                self._save_logs()
                return new_subgoal

            # return next(sg for sg in session["subgoals"]
            #         if sg["subgoal"] == subgoal)or {
            #                 "actions": [],
            #                 "scene_descriptions": [],
            #                 "feedbacks": [],
            #                 "invalid_controls": [],
            #                 "safety_triggers": [],
            #                 "completion_status": False
            #             }
        except Exception as e:
            print(f"Error getting current subgoal: {e}")
            return {
                "subgoal": self._clean_subgoal_string(subgoal),
                "actions": [],
                "scene_descriptions": [],
                "feedbacks": [],
                "invalid_controls": [],
                "safety_triggers": [],
                "completion_status": False
            }

