# Legacy individual systemd templates

For the current station use [STATION_OPERATIONS.md](../docs/STATION_OPERATIONS.md)
and `Firmware/scripts/run_application.sh`. That launcher owns controller,
native perception and web as one stack, and defaults to automatic roam/track.

The `.service` files here and `tools/install_station.py` describe the older
separate-service deployment. They use the legacy `vision.visiond` pipeline and
are not the supported installation method for the commissioned IMX500 station.
Do not enable them alongside the launcher: the camera and CAN link need a
single owner.

The [old procedure](../docs/archive/systemd_operations_v1.md) is retained as
historical evidence. Its system-Python and always-home-on-restart claims are
superseded by the project venv and validated retained calibration. The current
workflow does not install or enable boot-time systemd services.
