# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@rules_python//python:pip.bzl", "pip_parse")

def parse_deps():
    """
    Parse PyPI packages to a @pip_sshkeyboard_balancer external repository.

    This function intended to be loaded and called from your WORKSPACE.
    """
    pip_parse(
        name = "pip_sshkeyboard_balancer",
        requirements_lock = Label("//tools/workspace/pip_sshkeyboard_balancer:requirements_lock.txt"),
    )
