# Phase 1 Roboclaw Motor Control Harness:

### Purpose
- Provide Basic WASD Motor Control for Simple Tank Drive to meet TAMU Lunabotics 2026 Phase 1 Goals.

### Key Requirements
- Python with `venv` for proper development environment setup.
- Roboclaw MCU connected and identified over Serial with configuration defined in `config.py`.

### Module Structure
```
lunabotics-cdh-dev/python/motorcontrol
├── config.py            # Platform Specific Configuration 
├── main.py              # Motor Control Harness main executable
├── requirements.txt     
├── roboclaw_methods.py  # Basic Wrapped Methods to provide simple and quick setup.
└── README.md
```

### Planned and Implemented Functionality
- Tank Drive using 2 Track Drive Motors and the Roboclaw MCU.
