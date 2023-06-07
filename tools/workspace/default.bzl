# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/workspace/upkie:repository.bzl", "upkie_repository")

def add_default_repositories():
    """
    Declares workspace repositories for all dependencies (other than those
    built into Bazel, of course).

    This function intended to be loaded and called from a WORKSPACE file. If
    your project depends on @upkie, you will need to call this function from
    its WORKSPACE.
    """
    upkie_repository()
