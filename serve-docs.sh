#!/bin/bash
set -e

echo "🚀 Starting local documentation development server..."

# Check dependencies
if ! command -v doxygen &> /dev/null; then
    echo "❌ Please install doxygen first: sudo apt-get install doxygen graphviz"
    exit 1
fi

if ! command -v jekyll &> /dev/null; then
    echo "❌ Please install jekyll first: sudo gem install jekyll jekyll-theme-slate"
    exit 1
fi

# Build API docs
echo "📚 Building API documentation..."
doxygen Doxyfile

# Start Jekyll with live reload
echo "🌐 Starting Jekyll server with live reload..."
echo "📖 Documentation will be available at: http://localhost:4000"
echo "🔄 Files will auto-reload when you edit markdown files"
echo "⚠️  Note: To see API changes, re-run doxygen and refresh browser"
echo ""

cd docs
bundle exec jekyll serve --livereload --host 0.0.0.0 --destination ../_site || jekyll serve --livereload --host 0.0.0.0 --destination ../_site