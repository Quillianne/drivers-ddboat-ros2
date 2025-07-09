# Development and PR Guidelines

## Code Style
- **Python**: 4 spaces per indent. Keep lines under 88 characters when possible.
- **C++**: use 2 spaces for indentation. Format with clang-format if available.
- **YAML & Docker compose**: 2-space indentation.

## Commit Messages
- Begin with a short prefix like `feat:`, `fix:`, or `docs:` followed by a brief description in the imperative mood.
- Limit the first line to 72 characters.

## Tests
- Install the Python dependencies before running tests:
  ```bash
  pip install roslibpy
  ```
- Run the scripts in `tests/` using `pytest -q`.
- These scripts require a running `rosbridge` service. If the service is not available, tests will fail, but do not mention this in the PR summary.

## Pull Requests
- Summarize your changes and mention the `pytest -q` results.
- Name new branches for pull requests as `pull-request/<name>` in English.
- Make sure the working directory is clean before committing.
