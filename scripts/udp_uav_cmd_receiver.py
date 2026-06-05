#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launchable wrapper for test/udp_uav_cmd_receiver.py.

The implementation stays in test/udp_uav_cmd_receiver.py to avoid duplicating
the UDP command bridge logic. This wrapper lets roslaunch start it from the
package scripts path.
"""

import os
import runpy


if __name__ == "__main__":
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    receiver_path = os.path.join(package_dir, "test", "udp_uav_cmd_receiver.py")
    runpy.run_path(receiver_path, run_name="__main__")
