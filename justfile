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

# The real kernel build: CMake + Ninja, sources under packages/suspension_kernel.
build-kernel:
    uv run python packages/suspension_kernel/scripts/build_suspension_kernel.py

test-kernel: build-kernel
    uv run --package suspension-kernel pytest packages/suspension_kernel/tests

test-multibody: build-axle-native
    uv run --package suspension-multibody pytest packages/suspension_multibody/tests

test: test-contracts test-kinematics test-kernel test-multibody

# --- fast iteration (see AGENTS.md) -----------------------------------------

# Static gates, the architecture gate scripts, and the fast test set: the
# daily loop, about one minute end to end.
check-fast: lint type-check gate-architecture test-fast test-other

# The architecture gate scripts.  They are the structural evidence for the
# composable-architecture work, and each finishes in a couple of seconds: what
# the tests/architecture directory checks, minus the ten-minute pairwise
# process sweep that also lives there.
gate-architecture:
    uv run --package suspension-multibody python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
    uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
    uv run python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation

# Every test directory except the three slow ones, about 25 s against roughly
# 33 minutes for the whole package.  `--ignore` rather than a directory list, so
# a newly added test directory is picked up by default instead of being
# silently skipped.  The excluded three are Adams numerical equivalence,
# the architecture sweep, and the full-vehicle K/C grid.
test-fast:
    uv run --package suspension-multibody pytest packages/suspension_multibody/tests --ignore=packages/suspension_multibody/tests/adams --ignore=packages/suspension_multibody/tests/architecture --ignore=packages/suspension_multibody/tests/cases -q -p no:cacheprovider

# The other two packages.  A separate invocation on purpose: passing them to the
# same pytest call moves the rootdir and breaks the multibody tests' own
# `from tests.benchmark_fixture import ...`.
test-other:
    uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q -p no:cacheprovider
    uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q -p no:cacheprovider

# The frozen numerical gates.  Run these whenever a change touches the solve
# path: they are the only evidence that the physics and the performance budget
# did not move.  kc_parity is deliberately absent: without --actual-dir it
# compares the frozen snapshot against itself, so it proves nothing.
gate-numeric:
    uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
    uv run python packages/suspension_multibody/scripts/case_parity_check.py
    uv run python packages/suspension_multibody/scripts/kc_perf_gate.py

# The slow directories, kept separate so they are run deliberately rather than
# by default: Adams numerical equivalence, and the full-vehicle K/C grid.
test-slow:
    uv run --package suspension-multibody pytest packages/suspension_multibody/tests/adams packages/suspension_multibody/tests/cases -q -p no:cacheprovider

# The whole multibody suite, slow directories included.  About 33 minutes.
test-all:
    uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q -p no:cacheprovider

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
    uv run python packages/suspension_kernel/scripts/build_suspension_kernel.py
    uv build --package suspension-kernel
    uv run python packages/suspension_multibody/scripts/build_axle_native.py
    uv build --package suspension-multibody

import-smoke:
    uv run --package suspension-contracts python -c "import suspension_contracts"
    uv run --package suspension-kernel python -c "import suspension_kernel"
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
