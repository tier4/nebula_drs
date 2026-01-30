# Publishing documentation

The documentation site is built and deployed by the [documentation](../../.github/workflows/documentation.yml) workflow on push to `main` or `develop`, or when a pull request is merged.

## Enabling GitHub Pages

For the workflow to succeed, **GitHub Pages must be enabled** in the repository:

1. Open **Settings** → **Pages** for the repository.
2. Under **Build and deployment**, set **Source** to **GitHub Actions**.

If Pages is not enabled, the `deploy-pages` step fails with HTTP 404 and the message:
"Ensure GitHub Pages has been enabled: https://github.com/OWNER/REPO/settings/pages"

Only users with admin access to the repository can enable GitHub Pages.
