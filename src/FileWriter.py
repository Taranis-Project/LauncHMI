import os

class FileWriter:

    def __init__(self, relative_file_path):
        """
        Initializes the FileWriter with the given file path.

        Args:
            file_path (str): The path to the file.
        """
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.join(current_dir, relative_file_path)

    def write_to_file(self, content):
        """
        Writes the given content to the file.

        Args:
            content (str): The content to be written to the file.
        """
        try:
            with open(self.file_path, 'a') as file:
                file.write(content+ '\n')
            #print(f"File '{self.file_path}' created successfully.")
        except IOError as e:
            print(f"An I/O error occurred: {e}")

#Testing :
"""
file_writer = FileWriter("raw/test.txt")
while True:
    file_writer.write_to_file("This is the content of the new file.")  
"""