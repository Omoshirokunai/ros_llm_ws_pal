import os

import rich
from dotenv import load_dotenv
from LLM_robot_control_models import LLMController
from rich.console import Console
from rich.prompt import Prompt
from sensor_data import LOCAL_PATHS, fetch_images

console = Console()

def main():
    load_dotenv()

    # Initialize controller
    controller = LLMController()

    # Get image paths from environment
    current_image_path = os.getenv("LOCAL_IMAGE_PATH")
    previous_image_path = os.path.join(os.path.dirname(current_image_path), "previous.jpg")
    controller.debug = True  # Enable debug output
    while True:
        # Get user input
        prompt = Prompt.ask("\n[cyan]Enter your command[/cyan] (or '\[q]uit' to exit)")

        if prompt.lower() in ['quit', 'q']:
            break

        # Generate subgoals
        console.print("\n[yellow]Generating subgoals...[/yellow]")
        subgoals = controller.generate_subgoals(prompt)

        if not subgoals:
            console.print("[red]Failed to generate subgoals[/red]")
            continue

        console.print("[green]Subgoals:[/green]")
        for i, subgoal in enumerate(subgoals, 1):
            console.print(f"{i}. {subgoal}")

        # Process each subgoal
        for subgoal in subgoals:
            try:
                if not fetch_images():
                    console.print("[red]Failed to fetch required images[/red]")
                    continue
                # Load images
                # with open(current_image_path, 'rb') as curr_file, \
                #      open(previous_image_path, 'rb') as prev_file:
                #     current_image = curr_file.read()
                #     previous_image = prev_file.read()
                with open(LOCAL_PATHS['current'], 'rb') as curr_file, \
                    open(LOCAL_PATHS['previous'], 'rb') as prev_file:
                        current_image = curr_file.read()
                        previous_image = prev_file.read()
                # Get robot control action
                console.print(f"\n[yellow]Processing subgoal:[/yellow] {subgoal}")
                action = controller.control_robot(subgoal, current_image, previous_image)
                console.print(f"[blue]Robot action:[/blue] {action}")

                # Get feedback
                feedback = controller.get_feedback(current_image, previous_image)
                console.print(f"[purple]Feedback:[/purple] {feedback}")

                if feedback == "no progress":
                    console.print("[yellow]No progress detected, trying different action...[/yellow]")
                    continue
                elif feedback == "subtask complete":
                    console.print("[green]Subtask completed, moving to next subtask[/green]")
                    break
                elif feedback == "main goal complete":
                    console.print("[green]Main goal achieved![/green]")
                    return

            except Exception as e:
                console.print(f"[red]Error processing subgoal:[/red] {str(e)}")

if __name__ == "__main__":
    main()