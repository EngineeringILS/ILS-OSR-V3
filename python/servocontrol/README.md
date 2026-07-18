# Phase 1 Roboclaw Motor Control Harness:

### Purpose
- Provide Basic WASD Motor Control for individualized servo turning.

### Key Requirements
- Python with `venv` for proper development environment setup.
- Adafruit Servo Driver connected and identified over I2C with configuration defined in `config.py`.

### Module Structure
```
lunabotics-cdh-dev/python/servocontrol
├── config.py            # Platform Specific Configuration 
├── main.py              # Motor Control Harness main executable
├── requirements.txt     
├── servo_methods.py  # Basic Wrapped Methods to provide simple and quick setup.
└── README.md
```

### Planned and Implemented Functionality
- Individualized turning regimes with proper OOP best practices
