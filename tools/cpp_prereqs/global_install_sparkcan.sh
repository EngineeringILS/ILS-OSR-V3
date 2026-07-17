!bin/bash

#!/bin/bash

# Define the library name to check
LIB_NAME="sparkcan"

echo "Checking if $LIB_NAME is installed..."

# Check if the library exists in the system linker cache
if ldconfig -p | grep -iq "$LIB_NAME"; then
    echo "$LIB_NAME is already installed globally. Skipping installation."
else
    echo "$LIB_NAME not found. Proceeding with installation..."

    # Create a temporary directory in the home folder
    TEMP_DIR="$HOME/sparkcan_tmp_build"
    mkdir -p "$TEMP_DIR"
    cd "$TEMP_DIR" || exit

    # Clone the repository
    echo "Cloning sparkcan repository..."
    git clone https://github.com/grayson-arendt/sparkcan.git .

    # Build and install globally
    echo "Building sparkcan..."
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)

    echo "Installing $LIB_NAME globally..."
    sudo make install
    sudo ldconfig

    # Cleanup
    echo "Cleaning up temporary files..."
    cd "$HOME" || exit
    rm -rf "$TEMP_DIR"

    echo "Installation complete!"
fi