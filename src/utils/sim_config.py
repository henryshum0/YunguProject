#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read the Yungu simulation configuration (config/simulation.yaml).

Centralizes the Gazebo model, map/world, and the GZ topics bridged into ROS 2.
start_sim.sh and the GZ<->ROS bridge read it through this helper so they do not
have to implement YAML parsing themselves.

Usage:
    sim_config.py [--config PATH] get <dotted.key>
        Print a scalar value, e.g. `model`, `world`, `gz_version`, `xrce_port`,
        `bridge.enabled`. Booleans are printed as `true`/`false`, `None` as an
        empty string.

    sim_config.py [--config PATH] bridge-config
        Print the configured bridge topics as a YAML list in the ros_gz_bridge
        `parameter_bridge` config format, ready to be passed via
        `--ros-args -p config_file:=<file>`.

The default config path is <project_root>/config/simulation.yaml, resolved
relative to this script's location.
"""

import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.realpath(__file__))  # .../src/utils
DEFAULT_CONFIG = os.path.join(HERE, '..', '..', 'config', 'simulation.yaml')


def _load(path):
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        sys.exit(f"ERROR: '{path}' must contain a YAML mapping at the top level")
    return cfg


def _resolve_key(cfg, key):
    node = cfg
    for part in key.split('.'):
        if not isinstance(node, dict) or part not in node:
            sys.exit(f"ERROR: key '{key}' not found in the simulation config")
        node = node[part]
    return node


def _print_scalar(value, key):
    if isinstance(value, bool):
        print('true' if value else 'false')
    elif value is None:
        print('')
    elif isinstance(value, (dict, list)):
        sys.exit(f"ERROR: key '{key}' is not a scalar "
                 "(use 'bridge-config' for the topic list)")
    else:
        print(value)


def main():
    parser = argparse.ArgumentParser(
        description='Read the Yungu simulation configuration.')
    parser.add_argument('--config', default=DEFAULT_CONFIG,
                        help='path to the simulation config YAML '
                             '(default: %(default)s)')
    sub = parser.add_subparsers(dest='cmd', required=True)

    get_parser = sub.add_parser('get', help='print a scalar config value')
    get_parser.add_argument('key',
                            help='dotted key, e.g. model, bridge.tf_enabled')

    sub.add_parser('bridge-config',
                   help='print the ros_gz_bridge topic config YAML')

    args = parser.parse_args()
    cfg = _load(args.config)

    if args.cmd == 'get':
        _print_scalar(_resolve_key(cfg, args.key), args.key)
    elif args.cmd == 'bridge-config':
        topics = _resolve_key(cfg, 'bridge.topics')
        if not isinstance(topics, list):
            sys.exit("ERROR: 'bridge.topics' must be a list of topic bridge entries")
        print(yaml.safe_dump(topics, default_flow_style=False,
                             sort_keys=False).rstrip())


if __name__ == '__main__':
    main()
