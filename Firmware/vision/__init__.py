"""OpenAutoTurret vision daemon (Phase 4).

Camera capture, detection, target association, and latest-value IPC. This
package NEVER opens CAN or drives the motor — it only publishes timestamped
target measurements (architecture §5.1).
"""
