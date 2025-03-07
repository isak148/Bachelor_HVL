import os

def check_and_write_to_file(file_path, data):
    if os.path.exists(file_path):
        with open(file_path, 'a') as file:
            file.write(data + '\n')
        print(f"Data written to {file_path}")
    else:
        print(f"File {file_path} does not exist.")

# Example usage
file_path = '/c:/Users/isakh/Documents/skole/Bachelor/Bachelor_HVL/Filbehandling/data.txt'
data = "Sample data from another program"
check_and_write_to_file(file_path, data)