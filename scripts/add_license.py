# SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

import os
import sys

def add_license_to_file(file_path, copyright_text=None, license_type="BSD-3-Clause"):
    """
    Adds a license header to the specified file with appropriate comment style.
    """
    try:
        with open(file_path, 'r') as file:
            content = file.read()

        # Check if the license header already exists
        if "SPDX-FileCopyrightText" in content:
            print(f"License header already exists in {file_path}. Skipping.")
            return

        # Generate the license header with appropriate comments
        license_header = generate_license_header(file_path, copyright_text, license_type)

        # Add the license header at the top of the file
        new_content = license_header + "\n" + content

        with open(file_path, 'w') as file:
            file.write(new_content)

        print(f"License header added to {file_path}.")

    except Exception as e:
        print(f"Error processing {file_path}: {e}")

# Write a function that adds comment to lines in input based on the type of file

def add_comments_to_lines(lines, file_path):
    """
    Adds appropriate comment syntax to lines based on file extension.
    
    Args:
        lines (list): List of strings to be commented
        file_path (str): Path to the file to determine comment style
        
    Returns:
        list: List of commented strings
    """
    # Get file extension
    _, ext = os.path.splitext(file_path.lower())
    
    # Define comment styles for different file types
    comment_styles = {
        # C/C++ style comments
        '.cpp': '//',
        '.c': '//',
        '.h': '//',
        '.hpp': '//',
        '.cc': '//',
        '.cxx': '//',
        '.hxx': '//',
        '.java': '//',
        '.js': '//',
        '.ts': '//',
        '.cs': '//',
        '.go': '//',
        '.swift': '//',
        '.kt': '//',
        '.scala': '//',
        '.thrift': '//',
        
        # Shell/Python/Ruby style comments
        '.py': '#',
        '.sh': '#',
        '.bash': '#',
        '.zsh': '#',
        '.fish': '#',
        '.rb': '#',
        '.pl': '#',
        '.pm': '#',
        '.yaml': '#',
        '.yml': '#',
        '.toml': '#',
        '.ini': '#',
        '.cfg': '#',
        '.conf': '#',
        '.cmake': '#',
        
        # MATLAB style comments
        '.m': '%',
        
        # SQL style comments
        '.sql': '--',
        '.lua': '--',
    }
    
    # Special cases for files without extensions or specific names
    filename = os.path.basename(file_path).lower()
    if filename in ['Dockerfile', 'CMakeLists.txt']:
        comment_char = '#'
    else:
        comment_char = comment_styles.get(ext, '#')  # Default to # if unknown
    
    # Add comments to each line
    commented_lines = []
    for line in lines:
        if line.strip():  # Only add comment to non-empty lines
            commented_lines.append(f"{comment_char} {line}")
        else:
            commented_lines.append(line)  # Keep empty lines as-is
    
    return commented_lines

def generate_license_header(file_path, copyright_text=None, license_type="BSD-3-Clause"):
    """
    Generates a properly commented license header for a file.
    
    Args:
        file_path (str): Path to the file
        copyright_text (str): Copyright text (default: uses "2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia")
        license_type (str): License identifier
        
    Returns:
        str: Commented license header
    """
    if copyright_text is None:
        copyright_text = "2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia"
    
    license_lines = [
        f"SPDX-FileCopyrightText: {copyright_text}",
        f"SPDX-License-Identifier: {license_type}",
        ""  # Empty line after license
    ]
    
    commented_lines = add_comments_to_lines(license_lines, file_path)
    return '\n'.join(commented_lines)


if __name__ == "__main__":
    # Check if we're receiving input from stdin (pipe) or command line args
    if not sys.stdin.isatty():
        # Reading from stdin (pipe)
        print("Reading file paths from stdin...")
        for line in sys.stdin:
            file_path = line.strip()
            if file_path:  # Skip empty lines
                print(f"Processing: {file_path}")
                add_license_to_file(file_path)
    
    elif len(sys.argv) > 1:
        # Reading from command line arguments
        print("Processing files from command line arguments...")
        for file_path in sys.argv[1:]:
            print(f"Processing: {file_path}")
            add_license_to_file(file_path)
    
    else:
        # No input provided - show usage
        print("Usage:")
        print("  # From pipe:")
        print("  find . -name '*.cpp' | python add_license.py")
        print("  ./check_license.pl | grep 'NOT OK' | python add_license.py")
        print("")
        print("  # From command line:")
        print("  python add_license.py file1.cpp file2.h file3.py")
        print("")
        print("  # Example with license check script:")
        print("  ./tests/misc/check_license.pl 2>&1 | grep '\\[NOT OK' | sed 's/.*] //' | python add_license.py")