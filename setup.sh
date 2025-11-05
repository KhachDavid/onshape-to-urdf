#!/bin/bash
# Setup script for Onshape to URDF Pipeline

set -e  # Exit on error

echo "=========================================="
echo "  Onshape to URDF Pipeline Setup"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check Python installation
echo "Checking Python installation..."
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Error: Python 3 is not installed${NC}"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo -e "${GREEN}✓${NC} Python $PYTHON_VERSION found"
echo ""

# Check pip installation
echo "Checking pip installation..."
if ! command -v pip3 &> /dev/null; then
    echo -e "${RED}Error: pip is not installed${NC}"
    echo "Install it with: sudo apt install python3-pip"
    exit 1
fi

echo -e "${GREEN}✓${NC} pip found"
echo ""

# Create virtual environment (optional but recommended)
read -p "Create a virtual environment? (recommended) [Y/n] " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    source venv/bin/activate
    echo -e "${GREEN}✓${NC} Virtual environment created and activated"
    echo ""
fi

# Install dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Dependencies installed successfully"
else
    echo -e "${RED}Error installing dependencies${NC}"
    exit 1
fi
echo ""

# Verify onshape-to-robot installation
echo "Verifying onshape-to-robot installation..."
if command -v onshape-to-robot &> /dev/null; then
    echo -e "${GREEN}✓${NC} onshape-to-robot command available"
else
    echo -e "${YELLOW}Warning: onshape-to-robot command not in PATH${NC}"
    echo "You may need to use: python -m onshape_to_robot"
fi
echo ""

# Setup environment file
if [ ! -f .env ]; then
    echo "Creating .env file from template..."
    cp .env.template .env
    echo -e "${YELLOW}⚠${NC}  Please edit .env file and add your Onshape API credentials"
    echo "   Get your keys from: https://dev-portal.onshape.com/"
else
    echo -e "${GREEN}✓${NC} .env file already exists"
fi
echo ""

# Create output directory
mkdir -p output
echo -e "${GREEN}✓${NC} Output directory created"
echo ""

# Make convert.py executable
chmod +x convert.py
echo -e "${GREEN}✓${NC} Made convert.py executable"
echo ""

# Setup complete
echo "=========================================="
echo -e "${GREEN}  Setup Complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Edit .env file with your Onshape API credentials"
echo "  2. Edit config.json with your document ID"
echo "  3. Run: python convert.py --config config.json"
echo ""
echo "For virtual environment users:"
echo "  - Activate: source venv/bin/activate"
echo "  - Deactivate: deactivate"
echo ""
echo "Documentation:"
echo "  - README.md - Main documentation"
echo "  - docs/design_checklist.md - Design guidelines"
echo "  - docs/troubleshooting.md - Common issues"
echo ""
echo "Happy robot building!"

