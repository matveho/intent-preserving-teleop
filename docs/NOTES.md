Proposing a structure something along the lines of the following (very rough). Will have to see exactly how the phantom repositories and MuJoCo integrate. Kinda don't love it, will have to discuss.

```
intent-preserving-teleop/
├── src/
│   ├── cisst/
│   ├── cisst-ros/
│   ├── sawSensablePhantom/
│   ├── crtk_msgs/
│   │   ...
│   └── intent_preserving_teleop/
│       ├── config/
│       ├── assets/
│       │   ├── robots/
│       │   ├── environments/
│       │   └── scenes/
│       └── intent_preserving_teleop/
│           ├── __init__.py
│           ├── main.py
│           └── ...
├── .venv/
├── build/
├── install/
├── log/
└── ...
```