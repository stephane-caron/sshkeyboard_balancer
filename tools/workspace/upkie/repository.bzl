# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def upkie_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "upkie",
        remote = "https://github.com/tasts-robots/upkie.git",
        commit = "2372ac79332bb2df6314832e531f6ddadccdbbd9",
        shallow_since = "1685015203 +0200",
    )
