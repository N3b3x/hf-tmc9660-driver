#!/bin/bash
set -e

echo "Building HF-TMC9660 Documentation..."

# Check if required tools are installed
if ! command -v doxygen &> /dev/null; then
    echo "Error: doxygen is not installed. Please install it first:"
    echo "  sudo apt-get install doxygen graphviz"
    exit 1
fi

if ! command -v jekyll &> /dev/null; then
    echo "Error: jekyll is not installed. Please install it first:"
    echo "  sudo gem install jekyll jekyll-theme-slate"
    exit 1
fi

# Clean previous build
echo "Cleaning previous build..."
rm -rf _site docs/html

# Generate API documentation with Doxygen
echo "Generating API documentation with Doxygen..."
doxygen Doxyfile

# Build Jekyll site
echo "Building Jekyll site..."
cd docs
ruby -S jekyll build --destination ../_site
cd ..

echo "✅ Documentation build complete!"
echo "📁 Output directory: _site/"
echo "🌐 To serve locally, run: cd _site && python3 -m http.server 8000"
echo "📖 Then visit: http://localhost:8000"