# Upkie sshkeyboard balancer

Experimental Upkie agent (not maintained) using ``sshkeyboard`` for remote inputs.

First, start a simulation (or pi3hat) spine:

```console
./start_simulation.sh
```

You can run the agent via Bazel:

```console
./tools/bazelisk run //sshkeyboard_balancer -- -c bullet
```

Or, assuming you have pip-installed `upkie==1.4.0`, run the main script directly:

```console
python sshkeyboard_balancer/main.py -c bullet
```

The Makefile in this repository can upload and run spines and agents on the Raspberry Pi. Check out the [motion control software instructions](https://github.com/upkie/upkie/wiki/5%29-Motion-control-software) and run ``make help`` for details.
