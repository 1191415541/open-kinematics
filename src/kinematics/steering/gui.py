"""Compatibility wrapper for the steering GUI entry point."""

from kinematics.gui.steering import SteeringWorkbenchApp, main

__all__ = ["SteeringWorkbenchApp", "main"]


if __name__ == "__main__":
    main()

