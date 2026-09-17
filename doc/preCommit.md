## Development

### Pre-commit hooks

The repository uses [pre-commit](https://pre-commit.com) to keep formatting,
linting and commit messages consistent across the ROS 2 packages and the
Electron GUI. Hooks run locally before a commit is created, so CI failures for
purely mechanical issues should not happen.

#### One-time setup


```bash
# Install `pre-commit` once per machine:
pip install pre-commit

# Install the Node dependencies of the GUI
cd fkie_mas_gui && npm ci && cd ..

# Register the hooks in your clone
pre-commit install --install-hooks
```

#### Running hooks manually

```bash
pre-commit run                          # staged files only
pre-commit run --all-files              # entire repository
pre-commit run biome-check --all-files  # a single hook
pre-commit run --hook-stage pre-push    # the type-aware checks
```

#### Updating and troubleshooting

Bump the pinned hook versions and commit the result:
```bash
pre-commit autoupdate
pre-commit run --all-files
```
