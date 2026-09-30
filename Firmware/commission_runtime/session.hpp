#pragma once
namespace ota::commission {
int capture_session(const char* manifest_path);
int current_preparation_session(const char* manifest_path);
int current_characterization_session(const char* manifest_path);
int sensorless_homing_session(const char* manifest_path);
int validate_homing_manifest(const char* manifest_path);
}
