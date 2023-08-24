# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@pip_sshkeyboard_balancer//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "servo_controller",
    srcs = [
        "servo_controller.py",
        "wheel_balancer.py",
    ],
    deps = [
        "@upkie//:python",
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
        "//:pi64_config": ["@upkie//spines:pi3hat"],
        "//conditions:default": [],
    }),
    deps = [
        ":servo_controller",
        "@upkie//:python",
        "@vulp//:python",
        requirement("loop_rate_limiters"),
        requirement("mpacklog"),
        requirement("pyyaml"),
        requirement("sshkeyboard"),
    ],
)
