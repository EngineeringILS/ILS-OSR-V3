import os

# Get the absolute path of the current script file
absolute_script_path = os.path.abspath(__file__)

# If you need the directory containing the script:
script_directory = os.path.dirname(absolute_script_path) + "/images"

print(f"Absolute script path: {absolute_script_path}")
print(f"Script directory: {script_directory}")