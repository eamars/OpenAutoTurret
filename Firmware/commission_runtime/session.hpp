#pragma once
namespace ota::commission {
int yaw_control_session(const char* manifest_path);
int sensorless_homing_session(const char* manifest_path);
int validate_homing_manifest(const char* manifest_path);
}
