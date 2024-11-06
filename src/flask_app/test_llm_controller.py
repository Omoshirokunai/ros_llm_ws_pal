import os

import rich
from dotenv import load_dotenv
from LLM_robot_control_models import LLMController
from rich.console import Console
from rich.prompt import Prompt

console = Console()

def main():
    load_dotenv()

    # Initialize controller
    controller = LLMController()

    # Get image paths from environment
    current_image_path = os.getenv("LOCAL_IMAGE_PATH")
    previous_image_path = os.path.join(os.path.dirname(current_image_path), "previous.jpg")

    while True:
        # Get user input
        prompt = Prompt.ask("\n[cyan]Enter your command[/cyan] (or 'quit' to exit)")

        if prompt.lower() == 'quit':
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
                # Load images
                with open(current_image_path, 'rb') as curr_file, \
                     open(previous_image_path, 'rb') as prev_file:
                    current_image = curr_file.read()
                    previous_image = prev_file.read()

                # Get robot control action
                console.print(f"\n[yellow]Processing subgoal:[/yellow] {subgoal}")
                action = controller.control_robot(subgoal, current_image, previous_image)
                console.print(f"[blue]Robot action:[/blue] {action}")

                # Get feedback
                feedback = controller.get_feedback(current_image, previous_image)
                console.print(f"[purple]Feedback:[/purple] {feedback}")

                if feedback == "main goal complete":
                    break

            except Exception as e:
                console.print(f"[red]Error processing subgoal:[/red] {str(e)}")

if __name__ == "__main__":
    main()