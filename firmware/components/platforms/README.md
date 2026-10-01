### Platforms Component
- Provides board-specific pin mappings through shared protocol configuration types.
- `I2C()` populates a port configuration and returns false for unsupported ports.
- The base power-control methods return false when not implemented by a board.

### Supported Platforms
| Board | SDA | SCL | Frequency | I2C Power Control |
|-------|-----|-----|-----------|-------------------|
| FeatherS3TFT | 42 | 41 | 100 kHz | GPIO 21 |
| FeatherS3RevTFT | 3 | 4 | 100 kHz | Not implemented |

> Note: Both definitions expose port 0 only. FeatherS3TFT also exposes its GPIO 13 LED configuration.
