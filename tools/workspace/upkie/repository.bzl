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
        commit = "0798aaffe4c528c803fd0f8875edf327a0589deb",
        shallow_since = "1690373415 +0200",
    )
