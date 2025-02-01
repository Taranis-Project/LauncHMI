import subprocess
import os

class ExeLauncher:
    def __init__(self, exe_path):
        """
        Initialize the launcher with the path to the .exe file.
        """
        self.exe_path = exe_path

    def is_valid_path(self):
        """
        Check if the .exe path exists.
        """
        return os.path.exists(self.exe_path)

    def launch(self, *args):
        """
        Launch the .exe file with optional arguments.
        Args:
            *args: Optional arguments to pass to the .exe file.
        Returns:
            Output of the subprocess.
        Raises:
            FileNotFoundError: If the .exe path is invalid.
            subprocess.CalledProcessError: If the .exe fails during execution.
        """
        if not self.is_valid_path():
            raise FileNotFoundError(f"File not found: {self.exe_path}")

        # Combine the .exe path and the optional arguments
        command = [self.exe_path, *args]

        try:
            # Run the .exe with arguments
            result = subprocess.run(command, check=True, text=True, capture_output=True)
            print("Standard Output:", result.stdout)
            print("Error Output:", result.stderr)  # Add this to print error messages
            print("Execution successful!")
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"Error during execution: {e}")
            raise
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            raise