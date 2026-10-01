# Sensor Integration Maintenance

### Purpose
Correct firmware integration errors while preserving the shared driver interfaces.

### Implementation Plan
1. Align peripheral state names and component dependencies.
2. Correct sensor diagnostics, failed-read recovery, and terminal output.
3. Update driver comments and component documentation to match implemented behavior.
4. Build the ESP32-S3 firmware and run the standalone common tests.

### Validation
- [x] ESP32-S3 firmware build with ESP-IDF v5.4.3; no compiler warnings
- [x] Standalone common sensor test
- [x] Host regression tests for register conversion, read recovery, sample preservation, resource cleanup, and partial serial writes
- [x] Host regression tests included in GitHub Actions
- [ ] Physical sensor readings, GPIO output, and NeoPixel timing
