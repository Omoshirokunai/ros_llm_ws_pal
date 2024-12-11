# evaluation_metrics.py

import json
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns


@dataclass
class TaskMetrics:
    task_id: str
    prompt: str
    start_time: float
    completion_time: Optional[float] = None
    subgoals: List[str] = field(default_factory=list)
    actions: List[str] = field(default_factory=list)
    safety_violations: int = 0
    collision_checks: int = 0
    completed: bool = False
    scene_descriptions: List[str] = field(default_factory=list)

    @property
    def duration(self) -> float:
        if self.completion_time:
            return self.completion_time - self.start_time
        return 0.0

    @property
    def action_count(self) -> int:
        return len(self.actions)

    @property
    def reversal_count(self) -> int:
        reversals = 0
        for i in range(1, len(self.actions)):
            if (self.actions[i] == "move forward" and self.actions[i-1] == "move backward") or \
               (self.actions[i] == "move backward" and self.actions[i-1] == "move forward"):
                reversals += 1
        return reversals

    @property
    def path_smoothness(self) -> float:
        """Calculate path smoothness based on direction changes"""
        if len(self.actions) < 2:
            return 1.0

        direction_changes = 0
        for i in range(1, len(self.actions)):
            if self.actions[i] != self.actions[i-1]:
                direction_changes += 1

        return 1 - (direction_changes / len(self.actions))

class TaskEvaluator:
    def __init__(self):
        self.tasks: List[TaskMetrics] = []
        self.current_task: Optional[TaskMetrics] = None

    def start_task(self, prompt: str, subgoals: List[str]):
        """Initialize a new task evaluation"""
        task_id = f"task_{len(self.tasks)+1}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.current_task = TaskMetrics(
            task_id=task_id,
            prompt=prompt,
            start_time=time.time(),
            subgoals=subgoals
        )

    def log_action(self, action: str, scene_description: str = None):
        """Log robot action with scene context"""
        if self.current_task:
            self.current_task.actions.append(action)
            if scene_description:
                self.current_task.scene_descriptions.append(scene_description)

    def log_safety_event(self, collision_check: bool, violation: bool = False):
        """Log safety-related events"""
        if self.current_task:
            self.current_task.collision_checks += 1
            if violation:
                self.current_task.safety_violations += 1

    def complete_task(self, success: bool):
        """Mark current task as complete and store metrics"""
        if self.current_task:
            self.current_task.completed = success
            self.current_task.completion_time = time.time()
            self.tasks.append(self.current_task)
            self.current_task = None

    def generate_report(self, output_dir: str):
        """Generate evaluation visualizations and metrics"""
        os.makedirs(output_dir, exist_ok=True)

        # Aggregate metrics by prompt type
        prompt_metrics = {}
        for task in self.tasks:
            prompt_type = task.prompt.split()[0]  # e.g., "find", "move", etc.
            if prompt_type not in prompt_metrics:
                prompt_metrics[prompt_type] = []
            prompt_metrics[prompt_type].append({
                "success": task.completed,
                "duration": task.duration,
                "action_count": task.action_count,
                "reversals": task.reversal_count,
                "path_smoothness": task.path_smoothness,
                "safety_violations": task.safety_violations
            })

        # Plot metrics
        for metric in ["action_count", "reversals", "path_smoothness", "safety_violations"]:
            plt.figure(figsize=(10, 6))
            data = []
            labels = []
            for prompt_type, metrics in prompt_metrics.items():
                data.append([m[metric] for m in metrics])
                labels.extend([prompt_type] * len(metrics))

            sns.boxplot(x=labels, y=data)
            plt.title(f"{metric.replace('_', ' ').title()} by Task Type")
            plt.savefig(os.path.join(output_dir, f"{metric}_comparison.png"))
            plt.close()

        # Generate summary report
        report = {
            "overall_success_rate": sum(t.completed for t in self.tasks) / len(self.tasks),
            "metrics_by_task_type": {
                prompt_type: {
                    "success_rate": sum(m["success"] for m in metrics) / len(metrics),
                    "avg_duration": np.mean([m["duration"] for m in metrics]),
                    "avg_actions": np.mean([m["action_count"] for m in metrics]),
                    "avg_smoothness": np.mean([m["path_smoothness"] for m in metrics])
                }
                for prompt_type, metrics in prompt_metrics.items()
            }
        }

        with open(os.path.join(output_dir, "evaluation_report.json"), "w") as f:
            json.dump(report, f, indent=2)