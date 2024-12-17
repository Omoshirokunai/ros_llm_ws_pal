# evaluation_metrics.py

import json
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np
import rich
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

    # New tracking fields
    run_id: str = field(default_factory=lambda: f"run_{int(time.time())}")
    completed_subgoals: List[str] = field(default_factory=list)
    failed_subgoals: List[str] = field(default_factory=list)
    subgoal_completion_times: Dict[str, float] = field(default_factory=dict)
    current_subgoal: Optional[str] = None
    subgoal_attempts: Dict[str, int] = field(default_factory=dict)
    feedback_history: List[Dict[str, str]] = field(default_factory=list)

    distance_traveled: float = 0.0  # Total distance moved
    subtasks_completed: int = 0  # Number of completed subtasks
    failed_attempts: int = 0  # Number of failed action attempts
    stuck_count: int = 0  # Times "no progress" was reported
    total_rotations: float = 0.0  # Total rotation angle
    safety_stops: int = 0  # Number of safety-triggered stops


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

    @property
    def subtask_completion_rate(self) -> float:
        """Calculate ratio of completed subtasks"""
        if not self.subgoals:
            return 0.0
        return self.subtasks_completed / len(self.subgoals)

    @property
    def safety_score(self) -> float:
        """Calculate safety performance (0-1)"""
        if self.collision_checks == 0:
            return 1.0
        return 1 - (self.safety_violations / self.collision_checks)

    @property
    def efficiency_score(self) -> float:
        """Calculate task efficiency based on actions and time"""
        if self.duration == 0:
            return 0.0
        return 1 / (1 + (self.action_count / self.duration))



class TaskEvaluator:
    def __init__(self):
        self.tasks: List[TaskMetrics] = []
        self.current_task: Optional[TaskMetrics] = None
        plt.switch_backend('Agg')

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

    def log_subgoal_completion(self, subgoal: str, success: bool):
        """Log subgoal completion status"""
        if self.current_task:
            if success:
                self.current_task.completed_subgoals.append(subgoal)
                self.current_task.subgoal_completion_times[subgoal] = time.time()
            else:
                self.current_task.failed_subgoals.append(subgoal)
            self.current_task.subgoal_attempts[subgoal] = \
                self.current_task.subgoal_attempts.get(subgoal, 0) + 1

    def log_feedback(self, subgoal: str, feedback: str):
        """Log LLM feedback for current subgoal"""
        if self.current_task:
            self.current_task.feedback_history.append({
                "subgoal": subgoal,
                "feedback": feedback,
                "timestamp": time.time()
            })

    def complete_task(self, success: bool):
        """Mark current task as complete and store metrics"""
        if self.current_task:
            self.current_task.completed = success
            self.current_task.completion_time = time.time()
            self.tasks.append(self.current_task)
            self.current_task = None

    def generate_report(self, output_dir: str):
        """Generate evaluation visualizations and metrics"""

        try:
            os.makedirs(output_dir, exist_ok=True)
            # Guard against empty tasks list
            if not self.tasks:
                rich.print("[yellow]No tasks to generate report from[/yellow]")
                return

            # Aggregate metrics by prompt type
            prompt_metrics = {}
            for task in self.tasks:
                try:
                    # prompt_type = task.prompt.split()[0]  # e.g., "find", "move", etc.
                    # if prompt_type not in prompt_metrics:
                    #     prompt_metrics[prompt_type] = []

                    prompt_type = task.prompt.split()[0]  # e.g., "find", "move", etc.
                    if prompt_type not in prompt_metrics:
                        prompt_metrics[prompt_type] = []

                    # Convert metrics to basic types to avoid unhashable lists
                    metrics_dict = {
                        "success": bool(task.completed),
                        "duration": float(task.duration),
                        "action_count": int(task.action_count),
                        "reversals": int(task.reversal_count),
                        "path_smoothness": float(task.path_smoothness),
                        "safety_violations": int(task.safety_violations),
                        "run_id": task.run_id,
                "completed_subgoals": len(task.completed_subgoals),
                "failed_subgoals": len(task.failed_subgoals),
                "subgoal_attempts": task.subgoal_attempts,
                "subtask_completion_rate": float(task.subtask_completion_rate),
                "avg_attempts_per_subgoal": sum(task.subgoal_attempts.values()) / len(task.subgoals) if task.subgoals else 0

                    }

                    prompt_metrics[prompt_type].append(metrics_dict)

                except Exception as e:
                    rich.print(f"[red]Error processing task metrics:[/red] {e}")
                    continue

                 # Plot metrics with error handling
            metrics_to_plot = ["action_count", "reversals", "path_smoothness", "safety_violations"]
            for metric in metrics_to_plot:
                try:
                    plt.figure(figsize=(10, 6))
                    data = []
                    labels = []
                    for prompt_type, metrics in prompt_metrics.items():
                        metric_values = [m[metric] for m in metrics]
                        data.extend(metric_values)
                        labels.extend([prompt_type] * len(metric_values))

                    if data and labels:
                        sns.boxplot(x=labels, y=data)
                        plt.title(f"{metric.replace('_', ' ').title()} by Task Type")
                        plt.savefig(os.path.join(output_dir, f"{metric}_comparison.png"))
                    plt.close()
                except Exception as e:
                    rich.print(f"[red]Error generating plot for {metric}:[/red] {e}")
                    continue
            # Generate summary report with error handling
            try:
                    report = {
                        "overall_success_rate": sum(1 for t in self.tasks if t.completed) / len(self.tasks),
                        "metrics_by_task_type": {}
                    }

                    for prompt_type, metrics in prompt_metrics.items():
                        report["metrics_by_task_type"][prompt_type] = {
                            "success_rate": sum(m["success"] for m in metrics) / len(metrics),
                            "avg_duration": float(np.mean([m["duration"] for m in metrics])),
                            "avg_actions": float(np.mean([m["action_count"] for m in metrics])),
                            "avg_smoothness": float(np.mean([m["path_smoothness"] for m in metrics]))
                        }

                    with open(os.path.join(output_dir, "evaluation_report.json"), "w") as f:
                        json.dump(report, f, indent=2)

            except Exception as e:
                    rich.print(f"[red]Error generating summary report:[/red] {e}")

        except Exception as e:
            rich.print(f"[red]Error in generate_report:[/red] {e}")
        # # Plot metrics
        # for metric in ["action_count", "reversals", "path_smoothness", "safety_violations"]:
        #     plt.figure(figsize=(10, 6))
        #     data = []
        #     labels = []
        #     for prompt_type, metrics in prompt_metrics.items():
        #         data.append([m[metric] for m in metrics])
        #         labels.extend([prompt_type] * len(metrics))

        #     sns.boxplot(x=labels, y=data)
        #     plt.title(f"{metric.replace('_', ' ').title()} by Task Type")
        #     plt.savefig(os.path.join(output_dir, f"{metric}_comparison.png"))
        #     plt.close()

        # # Generate summary report
        # report = {
        #     "overall_success_rate": sum(t.completed for t in self.tasks) / len(self.tasks),
        #     "metrics_by_task_type": {
        #         prompt_type: {
        #             "success_rate": sum(m["success"] for m in metrics) / len(metrics),
        #             "avg_duration": np.mean([m["duration"] for m in metrics]),
        #             "avg_actions": np.mean([m["action_count"] for m in metrics]),
        #             "avg_smoothness": np.mean([m["path_smoothness"] for m in metrics])
        #         }
        #         for prompt_type, metrics in prompt_metrics.items()
        #     }
        # }

        # with open(os.path.join(output_dir, "evaluation_report.json"), "w") as f:
        #     json.dump(report, f, indent=2)