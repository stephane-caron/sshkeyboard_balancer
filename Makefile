# Makefile for upkie targets
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# NB: this Makefile is for Raspberry Pi targets only. Use Bazel or Bazelisk to
# build targets on your computer.

# Hostname or IP address of the Raspberry Pi
# Uses the value from the ROBOT environment variable, if defined.
# Valid usage: ``make upload ROBOT=foobar``
REMOTE = ${ROBOT}

# Project name, needs to match the one in WORKSPACE
PROJECT_NAME = sshkeyboard_balancer

BAZEL = $(CURDIR)/tools/bazelisk
CURDATE = $(shell date --iso=seconds)
CURDIR_NAME = $(shell basename $(CURDIR))
RASPUNZEL = $(CURDIR)/tools/raspunzel

# Help snippet adapted from:
# http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help
help:
	@echo "Host targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*? ## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'
	@echo "\nRaspberry Pi targets:\n"
	@grep -P '^[a-zA-Z0-9_-]+:.*?### .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?### "}; {printf "    \033[36m%-24s\033[0m %s\n", $$1, $$2}'

# HOST TARGETS
# ============

.PHONY: check-robot
check-robot:
	@ if [ -z "${ROBOT}" ]; then \
		echo "ERROR: Environment variable ROBOT is not set.\n"; \
		echo "This variable should contain the robot's hostname or IP address for SSH. You"; \
		echo "can define it inline for a one-time use:\n"; \
		echo "    make some_target ROBOT=your_robot_hostname\n"; \
		echo "Or add the following line to your shell configuration:\n"; \
		echo "    export ROBOT=your_robot_hostname\n"; \
		exit 1; \
	fi

.DEFAULT_GOAL := help
.PHONY: clean_broken_links
clean_broken_links:
	find -L $(CURDIR) -type l ! -exec test -e {} \; -delete

.PHONY: build
build: clean_broken_links  ## build Raspberry Pi targets
	$(BAZEL) build --config=pi64 //sshkeyboard_balancer
	$(BAZEL) build --config=pi64 @upkie//spines:mock
	$(BAZEL) build --config=pi64 @upkie//spines:pi3hat

.PHONY: clean
clean:  ## clean all local build and intermediate files
	$(BAZEL) clean --expunge

.PHONY: upload
upload: check-robot build  ## upload built targets to the Raspberry Pi
	ssh $(REMOTE) sudo date -s "$(CURDATE)"
	ssh $(REMOTE) mkdir -p $(PROJECT_NAME)
	ssh $(REMOTE) sudo find $(PROJECT_NAME) -type d -name __pycache__ -user root -exec chmod go+wx {} "\;"
	rsync -Lrtu --delete-after --delete-excluded --exclude bazel-out/ --exclude bazel-testlogs/ --exclude bazel-$(CURDIR_NAME) --exclude bazel-$(PROJECT_NAME)/ --progress $(CURDIR)/ $(REMOTE):$(PROJECT_NAME)/

start_simulation:  ## start a simulation spine
	$(BAZEL) run @upkie//spines:bullet -- --show

# REMOTE TARGETS
# ==============

run_mock_spine:  ### run the mock spine on the Raspberry Pi
	$(RASPUNZEL) run -s @upkie//spines:mock

run_pi3hat_spine:  ### run the pi3hat spine on the Raspberry Pi
	$(RASPUNZEL) run -s @upkie//spines:pi3hat

run_sshkeyboard_balancer:  ### sandbox agent
	$(RASPUNZEL) run -v -s //sshkeyboard_balancer -- --config pi3hat --configure-cpu
