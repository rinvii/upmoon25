# upmoon25-auto Makefile

JETSON_HOST ?= upmoon25@100.97.14.103
JETSON_DIR ?= ~/ros2/upmoon25-auto
RSYNC_SSH_OPTS ?= -o StrictHostKeyChecking=no
RSYNC_EXCLUDES = \
	--exclude '.git/' \
	--exclude '.lunar/' \
	--exclude '.venv-dashboard/' \
	--exclude '__pycache__/' \
	--exclude '*.pyc' \
	--exclude '/build/' \
	--exclude '/install/' \
	--exclude '/log/' \
	--exclude '.ruff_cache/' \
	--exclude 'lunar/.venv/' \
	--exclude 'lunar/mission-control/node_modules/'
DEPLOY_PATHS = \
	BUILD.bash \
	Dockerfile \
	INSTALL.bash \
	MAX_RUN.sh \
	Makefile \
	README.md \
	RUN_LAPTOP_RC.bash \
	RUN_ROBOT.bash \
	docker-compose.yml \
	docs \
	firmware \
	gz_worlds \
	lunar \
	src

ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
MISSION_CONTROL_DIR := $(ROOT)/lunar/mission-control
input ?= field_photos/in
output ?= field_photos/out

# Prefer global/Corepack `pnpm`; otherwise bootstrap pinned pnpm via npx (no global install needed).
MISSION_PNPM := $(if $(shell command -v pnpm 2>/dev/null),pnpm,npx --yes pnpm@9)

.PHONY: build up down restart shell sim dashboard streamlit-dashboard mission-bridge mission-control mission-control-demo mission-control-demo-copy mission-control-install mission-control-build mission-control-offline-prep autonomy-stack lint test test-firmware test-offline ci tune-flags run kill logs lunar-logs-pack lunar-logs-path check monitor keyboard config shell-env ros-shell install deploy deploy-dry-run help

help:
	@echo "upmoon25-auto Docker Management"
	@echo ""
	@echo "Usage:"
	@echo "  make build    - Build/Rebuild the Docker image"
	@echo "  make up       - Start the container in background"
	@echo "  make down     - Stop and remove the container"
	@echo "  make restart  - Restart the container"
	@echo "  make shell    - Enter the container's shell"
	@echo "  make sim world=gz_worlds/basic.world - Start simulation"
	@echo "  make dashboard - Launch React mission-control in the container (default port 8501)"
	@echo "  make streamlit-dashboard - Launch legacy Streamlit command center on this host"
	@echo "  make tune-flags input=... output=... - Batch flag detection on images (no ROS; defaults field_photos/in out)"
	@echo "  make mission-bridge - Launch the safe-command mission-control ROS bridge"
	@echo "  make mission-control - Launch the new web mission-control app"
	@echo "  make mission-control-demo - mission-control with fake data only + 0.0.0.0:5173 (Tailscale-friendly)"
	@echo "  make mission-control-demo-copy - rsync mission-control to ~/mission-control-demo (no node_modules)"
	@echo "  make mission-control-install - pnpm install (mission-control, frozen lockfile)"
	@echo "  make mission-control-build - Production build (mission-control)"
	@echo "  make mission-control-offline-prep - install + build for airgapped Jetson deploy"
	@echo "  make autonomy-stack grid=[coarse|standard|fine] - Launch shadow perception/autonomy (+ nav controller by default inside container)"
	@echo "  make lint       - ty check + ruff + mission-control eslint"
	@echo "  make test       - lint + all offline Python unit tests"
	@echo "  make test-firmware - Native encoder quadrature unit tests (g++; no ROS)"
	@echo "  make test-offline - Run hardware-free backend/bridge unit tests only"
	@echo "  make ci         - lint + mission-control production build + offline tests (no Jetson)"
	@echo "  make run profile=[robot|rc|autonomy|nav|nav-dig|test-encoder|dig|dig-backup] rotary=<ticks> - lunar run in Docker (dig/dig-backup need rotary= unless using LUNAR_RUN_FLAGS='--dig-timing-ms N')"
	@echo "  make lunar-logs-pack - Zip .lunar/runs/latest (combined.log, RUN_META.txt, optional rosbag from lunar run)"
	@echo "  make lunar-logs-path - Print path of .lunar/runs/latest"
	@echo "  (Optional: LUNAR_RUN_FLAGS='--no-session-bag' or '--session-bag-depth' on make run)"
	@echo "  make kill     - Stop all simulation and ROS processes"
	@echo "  make logs     - View logs"
	@echo "  make check    - Unified health audit"
	@echo "  make monitor  - Live error stream"
	@echo "  make keyboard - Control robot via terminal keyboard"
	@echo "  make config   - Show current configuration"
	@echo "  make shell-env - Print export commands for ROS shell setup"
	@echo "  make ros-shell - Open an interactive shell with ROS env loaded"
	@echo "  make install  - Install the lunar CLI (inside container)"
	@echo "  make deploy   - Prep mission-control (pnpm install + build), then rsync to Jetson"
	@echo "  make deploy OFFLINE_PREP=0 - Rsync only (skip pnpm install/build on this machine)"
	@echo "  make deploy-dry-run - Same prep, then preview rsync"

build:
	docker compose build

up:
	docker compose up -d

install:
	docker exec -it upmoon25_ros pip3 install -e ./lunar

down:
	docker compose down

restart:
	docker compose restart

shell:
	docker exec -it upmoon25_ros bash

sim:
	docker exec -it upmoon25_ros lunar sim --world $(world) --headless

dashboard:
	docker exec -it upmoon25_ros lunar dashboard

streamlit-dashboard:
	cd "$(ROOT)/lunar" && uv run --offline lunar streamlit-dashboard

mission-bridge:
	docker exec -it upmoon25_ros lunar mission-bridge --foreground

mission-control:
	cd "$(MISSION_CONTROL_DIR)" && $(MISSION_PNPM) dev -- --host 0.0.0.0 --port 8501

mission-control-demo:
	cd "$(MISSION_CONTROL_DIR)" && $(MISSION_PNPM) run dev:demo

mission-control-demo-copy:
	bash "$(MISSION_CONTROL_DIR)/scripts/copy-demo-dashboard.sh"

mission-control-build:
	cd "$(MISSION_CONTROL_DIR)" && $(MISSION_PNPM) build

autonomy-stack:
	docker exec -it upmoon25_ros lunar autonomy-stack --grid-preset $(or $(grid),standard)

lint:
	@if [ -x "$(ROOT)/lunar/.venv/bin/python" ]; then \
		cd "$(ROOT)" && uvx ty check src/backend lunar/src --python "$(ROOT)/lunar/.venv/bin/python"; \
	else \
		cd "$(ROOT)" && uvx ty check src/backend lunar/src; \
	fi
	cd "$(ROOT)" && uvx ruff check src/backend lunar/src
	cd "$(MISSION_CONTROL_DIR)" && $(MISSION_PNPM) lint

test: lint
	cd $(ROOT)/lunar && uv run python ../src/backend/test/run_offline_unit_tests.py

test-firmware:
	$(MAKE) -C firmware/arduino/tests test

test-offline:
	cd $(ROOT)/lunar && uv run python ../src/backend/test/run_offline_unit_tests.py

ci: lint
	cd "$(MISSION_CONTROL_DIR)" && $(MISSION_PNPM) build
	cd "$(ROOT)/lunar" && uv run python ../src/backend/test/run_offline_unit_tests.py

tune-flags:
	@mkdir -p "$(ROOT)/$(output)"
	cd "$(ROOT)" && PYTHONPATH="$(ROOT)/src/backend" uv run --project lunar python "$(ROOT)/src/backend/scripts/tune_flags_on_images.py" --input "$(ROOT)/$(input)" --output "$(ROOT)/$(output)"

# profile: robot | rc | autonomy | nav | dig | dig-backup
rotary ?=

# Optional extra args to `lunar run` (e.g. --no-session-bag or --session-bag-depth).
LUNAR_RUN_FLAGS ?=

run:
ifneq ($(filter $(strip $(profile)),dig dig-backup),$(strip $(profile)))
	docker exec -it upmoon25_ros lunar run $(profile) $(LUNAR_RUN_FLAGS)
else
ifeq ($(strip $(rotary)),)
	$(error make run profile=dig or dig-backup requires rotary=<positive_ticks> unless using LUNAR_RUN_FLAGS='--dig-timing-ms <positive_ms>')
endif
	docker exec -it upmoon25_ros lunar run $(profile) --calibrated-rotary $(rotary) $(LUNAR_RUN_FLAGS)
endif

kill:
	docker exec -it upmoon25_ros lunar kill

logs:
	docker exec -it upmoon25_ros lunar logs

# Pack the most recent `lunar run` session (same tree as `lunar run` prints: Session folder).
# Requires a background profile started with `lunar run` (e.g. make run profile=nav); writes under .lunar/
lunar-logs-pack:
	@LATEST="$(ROOT)/.lunar/runs/latest"; \
	if [ ! -e "$$LATEST" ]; then echo "No $$LATEST — start a background lunar run first (e.g. make run profile=nav)." >&2; exit 1; fi; \
	OUT="$(ROOT)/.lunar/lunar_session_$$(date +%Y%m%d_%H%M%S).zip"; \
	( cd "$$LATEST" && zip -rq "$$OUT" . ) && echo "Wrote $$OUT"

lunar-logs-path:
	@LATEST="$(ROOT)/.lunar/runs/latest"; \
	if [ ! -e "$$LATEST" ]; then echo "No $$LATEST yet." >&2; exit 1; fi; \
	realpath "$$LATEST" 2>/dev/null || readlink "$$LATEST" || echo "$$LATEST"

check:
	docker exec -it upmoon25_ros lunar check

monitor:
	docker exec -it upmoon25_ros lunar check --live

keyboard:
	docker exec -it upmoon25_ros lunar keyboard

config:
	docker exec -it upmoon25_ros lunar config

shell-env:
	@cd lunar && uv run --no-sync lunar shell-env

ros-shell:
	@bash -lc 'cd lunar && eval "$$(uv run --no-sync lunar shell-env)" && (ros2 daemon stop >/dev/null 2>&1 || true) && (ros2 daemon start >/dev/null 2>&1 || true) && export PS1="(lunar-ros) $$PS1" && exec bash --noprofile --norc -i'

mission-control-install:
	@command -v node >/dev/null 2>&1 || { echo "node required for mission-control (install Node 20+)" >&2; exit 1; }
	cd "$(MISSION_CONTROL_DIR)" && $(MISSION_PNPM) install --frozen-lockfile

mission-control-offline-prep: mission-control-install mission-control-build
	@echo "mission-control: dist/ is rsynced to the robot; node_modules stays on this machine (OS/arch specific)."

deploy:
ifneq ($(OFFLINE_PREP),0)
	$(MAKE) mission-control-offline-prep
endif
	rsync -azv --itemize-changes -e "ssh $(RSYNC_SSH_OPTS)" $(RSYNC_EXCLUDES) $(DEPLOY_PATHS) $(JETSON_HOST):$(JETSON_DIR)/

deploy-dry-run:
ifneq ($(OFFLINE_PREP),0)
	$(MAKE) mission-control-offline-prep
endif
	rsync -azvn --itemize-changes -e "ssh $(RSYNC_SSH_OPTS)" $(RSYNC_EXCLUDES) $(DEPLOY_PATHS) $(JETSON_HOST):$(JETSON_DIR)/
