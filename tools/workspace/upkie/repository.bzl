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
        commit = "e924dc3c52bd5fe16222254b283541483c4a0a4f",
        shallow_since = "1690381693 +0200",
    )
