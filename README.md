# Upkie sshkeyboard balancer

Experimental Upkie agent (not maintained) using ``sshkeyboard`` for remote inputs.

Run the agent in Python directly:

```console
python sshkeyboard_balancer/main.py -c bullet
```

Or with Bazel:

```console
./tools/bazelisk run //sshkeyboard_balancer -- -c bullet
```

The Makefile in this repository can upload and run spines and agents on the Raspberry Pi. Check out the [motion control software instructions](https://github.com/tasts-robots/upkie/wiki/5%29-Motion-control-software) and run ``make help`` for details.
