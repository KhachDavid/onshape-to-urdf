#!/bin/bash
# Quick script to run conversion for your robot

echo "=========================================="
echo "  Converting Onshape Robot to URDF"
echo "=========================================="
echo ""

# Check if environment variables are set
if [ -z "$ONSHAPE_ACCESS_KEY" ] || [ -z "$ONSHAPE_SECRET_KEY" ]; then
    echo "Warning: Onshape API credentials not set"
    echo ""
    echo "Loading from .env file..."
    
    if [ -f .env ]; then
        export $(cat .env | grep -v '^#' | xargs)
        echo "Loaded credentials from .env"
    else
        echo "Error: .env file not found"
        echo ""
        echo "Please create .env file with:"
        echo "  ONSHAPE_ACCESS_KEY=your_key"
        echo "  ONSHAPE_SECRET_KEY=your_secret"
        echo ""
        echo "Get your keys from: https://dev-portal.onshape.com/"
        exit 1
    fi
fi

echo ""
echo "Starting conversion..."
echo ""

# Run the conversion (also emit standalone URDF with absolute mesh paths)
python3 convert.py --config config.json --verbose --standalone

echo ""
echo "=========================================="
echo "Done! Check the output directory for results."
echo "=========================================="

