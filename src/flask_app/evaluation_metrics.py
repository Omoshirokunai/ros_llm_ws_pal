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

      # New success tracking fields
    ground_truth_success: bool = False  # User input for actual task success
    model_predicted_successes: Dict[str, bool] = field(default_factory=dict)  # Track model success claims per subgoal
    ground_truth_subgoal_successes: Dict[str, bool] = field(default_factory=dict)  # Track actual success per subgoal

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

    def log_subgoal_completion(self, subgoal: str, success: bool):
        """Log subgoal completion status"""
        if self.current_task:
            if success:
                self.current_task.completed_subgoals.append(subgoal)
                self.current_task.subgoal_completion_times[subgoal] = time.time()
                # Track model's prediction
                self.current_task.model_predicted_successes[subgoal] = True
            else:
                self.current_task.failed_subgoals.append(subgoal)
                self.current_task.model_predicted_successes[subgoal] = False

    def complete_task(self, success: bool):
        """Mark current task as complete and store metrics"""
        if self.current_task:
            self.current_task.completed = success
            self.current_task.completion_time = time.time()
            self.tasks.append(self.current_task)
            self.current_task = None

        # Check success rates in reporting
    def calculate_accuracy_metrics(self):
        """Calculate model prediction accuracy"""
        if not self.tasks:
            return {}

        correct_predictions = 0
        total_predictions = 0

        for task in self.tasks:
            if task.ground_truth_success is not None:
                total_predictions += 1
                if task.completed == task.ground_truth_success:
                    correct_predictions += 1

        return {
            "accuracy": correct_predictions / total_predictions if total_predictions > 0 else 0,
            "total_tasks": len(self.tasks)
        }

    def update_ground_truth(self, task_id: str, subgoal: str = None, success: bool = False):
        """Update ground truth and recalculate metrics"""
        for task in self.tasks:
            if task.task_id == task_id:
                if subgoal:
                    task.ground_truth_subgoal_successes[subgoal] = success
                else:
                    task.ground_truth_success = success
                # Regenerate report after update
                self.generate_report("src/flask_app/static/evaluation_results")
                return True
        return False
    def generate_report(self, output_dir: str):
        """Generate evaluation visualizations and metrics"""
        try:
            os.makedirs(output_dir, exist_ok=True)
            if not self.tasks:
                rich.print("[yellow]No tasks to generate report from[/yellow]")
                return

            # Aggregate metrics by prompt type
            prompt_metrics = {}
            for task in self.tasks:
                try:
                    prompt_type = task.prompt.split()[0]
                    if prompt_type not in prompt_metrics:
                        prompt_metrics[prompt_type] = []

                    # Calculate metrics
                    metrics_dict = {
                        "success": bool(task.completed),
                        "ground_truth_success": bool(task.ground_truth_success),
                        "duration": float(task.duration),
                        "action_count": int(task.action_count),
                        "path_smoothness": float(task.path_smoothness),
                        "safety_violations": int(task.safety_violations),
                        "subtask_completion_rate": float(task.subtask_completion_rate),
                        "stuck_count": int(task.stuck_count),
                        "subgoal_accuracy": self._calculate_subgoal_accuracy(task)
                        **self._calculate_prediction_metrics(task)
                    }
                    prompt_metrics[prompt_type].append(metrics_dict)
                except Exception as e:
                    rich.print(f"[red]Error processing task metrics:[/red] {e}")
                    continue

            # Generate plots with improved visualization
            self._generate_metric_plots(prompt_metrics, output_dir)

            # Generate summary report
            self._generate_summary_report(prompt_metrics, output_dir)

            # Generate confusion matrix
            self._plot_confusion_matrix(prompt_metrics, output_dir)

        except Exception as e:
            rich.print(f"[red]Error in generate_report:[/red] {e}")

    def _calculate_subgoal_accuracy(self, task):
        """Calculate accuracy of model's subgoal completion predictions"""
        if not task.ground_truth_subgoal_successes:
            return 0.0
        correct = sum(1 for sg, pred in task.model_predicted_successes.items()
                     if sg in task.ground_truth_subgoal_successes
                     and pred == task.ground_truth_subgoal_successes[sg])
        total = len(task.ground_truth_subgoal_successes)
        return correct / total if total > 0 else 0.0

    def _calculate_prediction_metrics(self, task):
        """Calculate confusion matrix metrics for model predictions vs ground truth"""
        metrics = {
            "true_positives": 0,  # Model predicted success correctly
            "false_positives": 0, # Model predicted success incorrectly
            "true_negatives": 0,  # Model predicted failure correctly
            "false_negatives": 0, # Model predicted failure incorrectly
            "prediction_accuracy": 0.0
        }

        # Overall task completion accuracy
        if task.ground_truth_success is not None:
            if task.completed and task.ground_truth_success:
                metrics["true_positives"] += 1
            elif task.completed and not task.ground_truth_success:
                metrics["false_positives"] += 1
            elif not task.completed and not task.ground_truth_success:
                metrics["true_negatives"] += 1
            elif not task.completed and task.ground_truth_success:
                metrics["false_negatives"] += 1

        # Calculate per-subgoal accuracy
        total = len(task.ground_truth_subgoal_successes)
        if total > 0:
            correct = sum(1 for sg, pred in task.model_predicted_successes.items()
                        if sg in task.ground_truth_subgoal_successes
                        and pred == task.ground_truth_subgoal_successes[sg])
            metrics["prediction_accuracy"] = correct / total

        return metrics

    def _generate_metric_plots(self, prompt_metrics, output_dir):
        """Generate visualizations for key metrics"""
        metrics_to_plot = [
            ("success", "Task Success Rate"),
            ("path_smoothness", "Path Smoothness"),
            ("safety_violations", "Safety Violations"),
            ("subtask_completion_rate", "Subtask Completion Rate"),
            ("subgoal_accuracy", "Subgoal Prediction Accuracy"),
            ("prediction_accuracy", "Model Prediction Accuracy"),
        ]

        for metric, title in metrics_to_plot:
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
                    plt.title(title)
                    plt.ylabel(title)
                    plt.xlabel("Task Type")
                    plt.xticks(rotation=45)
                    plt.tight_layout()
                    plt.savefig(os.path.join(output_dir, f"{metric}_comparison.png"))
                plt.close()
            except Exception as e:
                rich.print(f"[red]Error generating plot for {metric}:[/red] {e}")
                continue

    # Add confusion matrix plot
    def _plot_confusion_matrix(self, prompt_metrics, output_dir):
        """Generate confusion matrix visualization"""
        try:
            plt.figure(figsize=(8, 6))

            tp = sum(m["true_positives"] for metrics in prompt_metrics.values() for m in metrics)
            fp = sum(m["false_positives"] for metrics in prompt_metrics.values() for m in metrics)
            tn = sum(m["true_negatives"] for metrics in prompt_metrics.values() for m in metrics)
            fn = sum(m["false_negatives"] for metrics in prompt_metrics.values() for m in metrics)

            cm = [[tp, fp], [fn, tn]]
            sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                    xticklabels=['Predicted Success', 'Predicted Failure'],
                    yticklabels=['Actual Success', 'Actual Failure'])

            plt.title('Model Prediction Confusion Matrix')
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, "confusion_matrix.png"))
            plt.close()
        except Exception as e:
            rich.print(f"[red]Error generating confusion matrix plot:[/red] {e}")


# evaluation_metrics.py

    def _generate_summary_report(self, prompt_metrics: Dict, output_dir: str):
        """Generate summary report with aggregate metrics"""
        try:
            report = {
                "overall_metrics": {
                    "total_tasks": len(self.tasks),
                    "success_rate": sum(1 for t in self.tasks if t.completed) / len(self.tasks),
                    "ground_truth_success_rate": sum(1 for t in self.tasks if t.ground_truth_success) / len(self.tasks),
                    "prediction_accuracy": sum(1 for t in self.tasks
                                            if t.completed == t.ground_truth_success) / len(self.tasks)
                },
                "metrics_by_task_type": {}
            }

            # Calculate per task type metrics
            for prompt_type, metrics in prompt_metrics.items():
                report["metrics_by_task_type"][prompt_type] = {
                    "total_tasks": len(metrics),
                    "success_rate": sum(m["success"] for m in metrics) / len(metrics),
                    "ground_truth_success_rate": sum(m["ground_truth_success"] for m in metrics) / len(metrics),
                    "avg_duration": float(np.mean([m["duration"] for m in metrics])),
                    "avg_actions": float(np.mean([m["action_count"] for m in metrics])),
                    "avg_smoothness": float(np.mean([m["path_smoothness"] for m in metrics])),
                    "avg_safety_violations": float(np.mean([m["safety_violations"] for m in metrics])),
                    "avg_subtask_completion": float(np.mean([m["subtask_completion_rate"] for m in metrics])),
                    "prediction_metrics": {
                        "true_positives": sum(m["true_positives"] for m in metrics),
                        "false_positives": sum(m["false_positives"] for m in metrics),
                        "true_negatives": sum(m["true_negatives"] for m in metrics),
                        "false_negatives": sum(m["false_negatives"] for m in metrics),
                        "prediction_accuracy": float(np.mean([m["prediction_accuracy"] for m in metrics]))
                    }
                }

            # Save report
            with open(os.path.join(output_dir, "evaluation_report.json"), "w") as f:
                json.dump(report, f, indent=2)

        except Exception as e:
            rich.print(f"[red]Error generating summary report:[/red] {e}")
