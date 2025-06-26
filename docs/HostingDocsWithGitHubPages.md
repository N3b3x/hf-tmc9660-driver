# Hosting Documentation with GitHub Pages

Automatically publishing the generated Doxygen output keeps the online documentation up to date. The repository includes a workflow that builds the HTML files and deploys them to GitHub Pages.

## Workflow Snippet

```yaml
permissions:
  contents: read
  pages: write
  id-token: write

steps:
  - uses: actions/checkout@v4
  - name: Build documentation
    run: doxygen Doxyfile
  - name: Deploy to GitHub Pages
    uses: peaceiris/actions-gh-pages@v3
    with:
      github_token: ${{ secrets.GITHUB_TOKEN }}
      publish_dir: docs/html
```

If the step fails with a `403` error, the token lacks permission to push to the repository. Enable **Read and write permissions** under **Settings → Actions → General**, or provide a Personal Access Token with the `repo` scope and reference it as `${{ secrets.PAT }}`.

---

[\u2B05\uFE0F Prev](CommonOperations.md) | [\u2B06\uFE0F Back to Index](index.md)
