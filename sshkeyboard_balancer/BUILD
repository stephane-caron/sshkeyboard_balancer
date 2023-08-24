# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("//tools/lint:lint.bzl", "add_lint_tests")
load("@pip_upkie_sandbox//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "servo_controller",
    srcs = [
        "servo_controller.py",
        "wheel_balancer.py",
    ],
    deps = [
        "@upkie//observers/base_pitch",
        "@upkie//utils:clamp",
        "@upkie//utils:exceptions",
        "@upkie//utils:filters",
        requirement("gin-config"),
    ],
)

py_binary(
    name = "sshkeyboard_balancer",
    srcs = ["main.py"],
    main = "main.py",
    data = [
        "//sshkeyboard_balancer/config",
    ] + select({
        "//:pi32_config": ["@upkie//spines:pi3hat"],
        "//:pi64_config": ["@upkie//spines:pi3hat"],
        "//conditions:default": [],
    }),
    deps = [
        ":servo_controller",
        "@mpacklog//:python",
        "@upkie//utils:raspi",
        "@upkie//utils:spdlog",
        "@vulp//spine:python",
        requirement("loop_rate_limiters"),
        requirement("pyyaml"),
        requirement("sshkeyboard"),
    ],
)

add_lint_tests()
