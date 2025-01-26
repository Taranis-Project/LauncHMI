import subprocess
import os

class BatchFileRunner:
    def __init__(self, batch_file_name="startupsequence.bat"):
        # Get the directory of the current Python file
        self.script_dir = os.path.dirname(__file__)
        self.batch_file_name = batch_file_name
        self.batch_file_path = os.path.join(self.script_dir, self.batch_file_name)

    def run_batch_file(self):
        """Checks if the batch file exists and runs it."""
        if os.path.exists(self.batch_file_path):
            subprocess.run(["cmd.exe", "/c", self.batch_file_path])
        else:
            print(f"Batch file not found at: {self.batch_file_path}")

# Example usage
runner = BatchFileRunner("startupsequence.bat")
runner.run_batch_file()
