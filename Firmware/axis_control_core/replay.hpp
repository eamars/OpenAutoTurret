#pragma once
namespace ota::axis {
// Strict local replay entry: no device IO, owner handover or production qualification.
int replay_file(const char* path);
}
