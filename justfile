# Synchronize every independently releasable workspace package.
default: check

setup:
    uv sync --all-packages --all-extras --all-groups

install: setup

# Install every optional dependency exercised by automated validation.
install-ci:
    uv sync --all-packages --all-extras --all-groups

# Remove Python build and cache products without touching analysis evidence.
clean:
    rm -rf .venv
    rm -rf .pytest_cache
    rm -rf .ruff_cache
    find packages -type d -name __pycache__ -prune -exec rm -rf {} +
    find packages -type d -name build -prune -exec rm -rf {} +
    find packages -type d -name dist -prune -exec rm -rf {} +
    find packages -type d -name '*.egg-info' -prune -exec rm -rf {} +

# Product regression gates.
test-contracts:
    uv run --package suspension-contracts pytest packages/suspension_contracts/tests

test-kinematics:
    uv run --package suspension-kinematics pytest packages/suspension_kinematics/tests

build-axle-native:
    uv run python packages/suspension_multibody/scripts/build_axle_native.py

test-multibody: build-axle-native
    uv run --package suspension-multibody pytest packages/suspension_multibody/tests

test: test-contracts test-kinematics test-multibody

# Static gates over the workspace-defined scope.
lint:
    uv run --all-packages ruff check .

type-check:
    uv run --all-packages ty check .

check: lint type-check build import-smoke cli-smoke

# Build each independently releasable wheel.
build:
    uv build --package suspension-contracts
    uv build --package suspension-kinematics
    uv run python packages/suspension_multibody/scripts/build_axle_native.py
    uv build --package suspension-multibody

import-smoke:
    uv run --package suspension-contracts python -c "import suspension_contracts"
    uv run --package suspension-kinematics python -c "import suspension_kinematics"
    uv run --package suspension-multibody python -c "import suspension_multibody"

cli-smoke:
    uv run --package suspension-kinematics suspension-kinematics --help
    uv run --package suspension-multibody suspension-multibody --help

# Regenerate kinematics end-to-end reference files after fixture changes.
regen-refs:
    uv run --package suspension-kinematics suspension-kinematics sweep --geometry packages/suspension_kinematics/tests/data/geometry.yaml --sweep packages/suspension_kinematics/tests/data/sweep.yaml --out packages/suspension_kinematics/tests/data/e2e/output.csv
    uv run --package suspension-kinematics suspension-kinematics sweep --geometry packages/suspension_kinematics/tests/data/geometry.yaml --sweep packages/suspension_kinematics/tests/data/sweep.yaml --out packages/suspension_kinematics/tests/data/e2e/output.parquet

generate-animation-test:
    uv run --package suspension-kinematics pytest packages/suspension_kinematics/tests/manual/test_run_with_viz.py -m manual -s

generate-animation:
    mkdir -p packages/suspension_kinematics/artifacts/visualization
    uv run --package suspension-kinematics suspension-kinematics sweep --geometry packages/suspension_kinematics/tests/data/geometry.yaml --sweep packages/suspension_kinematics/tests/data/sweep.yaml --out packages/suspension_kinematics/artifacts/visualization/results.csv --animation-out packages/suspension_kinematics/artifacts/visualization/animation.gif

generate-jacobians:
    uv run --package suspension-kinematics python packages/suspension_kinematics/tools/generate_jacobians.py

format:
    uv run --all-packages ruff format .

spellcheck:
    uv run --all-packages codespell packages/suspension_contracts/src packages/suspension_contracts/tests packages/suspension_kinematics/src packages/suspension_kinematics/tests packages/suspension_multibody/src packages/suspension_multibody/tests

spellcheck-fix:
    uv run --all-packages codespell --write-changes packages/suspension_contracts/src packages/suspension_contracts/tests packages/suspension_kinematics/src packages/suspension_kinematics/tests packages/suspension_multibody/src packages/suspension_multibody/tests

spellcheck-interactive:
    uv run --all-packages codespell --write-changes --interactive 3 packages/suspension_contracts/src packages/suspension_contracts/tests packages/suspension_kinematics/src packages/suspension_kinematics/tests packages/suspension_multibody/src packages/suspension_multibody/tests
