# Upkie sshkeyboard balancer

Experimental Upkie agent (not maintained) using ``sshkeyboard`` for remote inputs.

Run the agent directly in Python directly:

```console
python sshkeyboard_balancer/main.py -c bullet
```

Or with Bazel:

```console
./tools/bazelisk run //sshkeyboard_balancer -- -c bullet
```
