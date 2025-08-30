#!/bin/bash

# Build script for SLAM Server
# This script builds only the slam_server executable

echo "Building SLAM Server..."

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build only the slam_server target
echo "Building slam_server..."
make slam_server -j$(nproc)

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "📍 Executable location: Examples/Monocular/slam_server"
    echo ""
    echo "Usage:"
    echo "  ./Examples/Monocular/slam_server vocabulary.txt settings.yaml [options]"
    echo ""
    echo "Options:"
    echo "  --port 8080              Set server port"
    echo "  --output-dir ./output    Set output directory"
    echo "  --max-queue 10           Set maximum queue size"
    echo "  --viewer                 Enable SLAM viewer"
    echo "  --help                   Show help"
    echo ""
    echo "Test with:"
    echo "  python3 Examples/Monocular/test_client.py --server http://localhost:8080"
else
    echo "❌ Build failed!"
    exit 1
fi