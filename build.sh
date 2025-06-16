#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- 1. Clean up the previous build directory ---
echo "Removing existing build directory..."
rm -rf builddir/

# --- 2. Configure the project with Meson ---
echo "Configuring Meson build..."
meson setup --reconfigure builddir \
    --cross-file cross_configs/telink_target_cross_file.ini \
    -Dproduct_family=RABBlueController \
    -Dproduct_pid=108F \
    -Dble_stack_variant=standard

# --- 3. Compile the project with Ninja ---
echo "Building project in 'builddir'..."
ninja -C builddir

echo "Build finished successfully!"