# Gemfile for HardFOC Interface Wrapper Documentation
# MCU Project Jekyll Dependencies

source "https://rubygems.org"

# Jekyll core
gem "jekyll", "~> 4.3.0"

# Theme
gem "minima", "~> 2.5"

# Plugins for enhanced functionality
gem "jekyll-feed", "~> 0.12"
gem "jekyll-sitemap", "~> 1.4"
gem "jekyll-seo-tag", "~> 2.8"

# Development and testing
group :development do
  gem "github-pages", group: :jekyll_plugins
  gem "jekyll-admin", group: :jekyll_plugins
  gem "jekyll-compose", group: :jekyll_plugins
end

# Performance and optimization
group :jekyll_plugins do
  gem "jekyll-minifier"
  gem "jekyll-assets"
end

# Ruby version
ruby ">= 2.7.0"

# Platform specific gems
platforms :ruby do
  gem "wdm", "~> 0.1.1", :install_if => Gem.win_platform?
end

# Windows specific
platforms :mingw, :x64_mingw, :mswin, :jruby do
  gem "tzinfo", ">= 1", "< 3"
  gem "tzinfo-data"
end

# Performance gems
gem "ffi", "~> 1.0"
gem "sassc", "~> 2.4"
