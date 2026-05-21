# Branch Protection

GitHub branch protection ayarlari web arayuzunden yapilir:
`Settings -> Branches -> Add branch protection rule`.

## main

- Require a pull request before merging.
- Require status checks to pass before merging.
- Require branches to be up to date before merging.
- Required status check: `Lint and tests (Linux)`.
- Do not allow force pushes.
- Do not allow deletions.

## develop

- Require status checks to pass before merging.
- Required status check: `Lint and tests (Linux)`.
- Do not allow force pushes.
- Do not allow deletions.
