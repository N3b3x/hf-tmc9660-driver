---
layout: default
title: Hosting Documentation with GitHub Pages
---

# Hosting Documentation with GitHub Pages

Automatically publishing the generated Doxygen output keeps the online documentation up to date. The repository includes a workflow that builds the HTML files and deploys them to GitHub Pages.

The Markdown guides under `docs/` form the written tutorial while Doxygen
generates the C++ API reference in `docs/html`. Deploying the entire `docs`
directory lets `index.md` act as the landing page with a complete table of
contents. The API reference is then available at the `html/` subfolder.

## Workflow Snippet

```yaml
permissions:
  contents: read
  pages: write
  id-token: write

steps:
  - uses: actions/checkout@v4
  - name: Setup Pages
    id: pages
    uses: actions/configure-pages@v4
  - name: Install dependencies
    run: sudo apt-get update && sudo apt-get install -y doxygen graphviz ruby-full && sudo gem install jekyll jekyll-theme-slate
  - name: Build documentation
    run: |
      doxygen Doxyfile
      ruby -S jekyll build --source docs --destination _site --baseurl "${{ steps.pages.outputs.base_path }}"
  - name: Upload Pages artifact
    uses: actions/upload-pages-artifact@v3
    with:
      path: _site
  - name: Deploy to GitHub Pages
    uses: actions/deploy-pages@v4
```

If the step fails with a `403` error, the token lacks permission to push to the repository. Enable **Read and write permissions** under **Settings → Actions → General**, or provide a Personal Access Token with the `repo` scope and reference it as `${{ secrets.PAT }}`.

After pushing to `main`, visit **Settings → Pages** to find the deployed site.
You should see the table of contents from `docs/index.md` and a link to the API
reference in the `html/` directory.

---

[\u2B05\uFE0F Prev](CommonOperations.html) | [\u2B06\uFE0F Back to Index](index.html)
