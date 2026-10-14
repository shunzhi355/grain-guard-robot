"""Entry point for the mock-robot console script.

Re-exports ``main()`` from :mod:`mock_robot.mock_robot_node` so that
the ``setup.py`` entry point ``mock-robot=mock_robot.main:main`` resolves
correctly.
"""

from mock_robot.mock_robot_node import main

__all__ = ["main"]
