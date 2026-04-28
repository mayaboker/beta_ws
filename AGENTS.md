# Project Agent Instructions

## Python Logging

- Do not add new `print()` statements in Python modules.
- Use `loguru` for runtime messages instead.
- Prefer structured log levels:
  - `logger.debug()` for detailed development information.
  - `logger.info()` for normal progress messages.
  - `logger.warning()` for recoverable unexpected behavior.
  - `logger.error()` for failures that prevent the requested operation.

Example:

```python
from loguru import logger

logger.info("Subscribed to Gazebo camera topic: {}", topic)
```

When editing existing Python code, replace nearby `print()` statements with `loguru` logging if the change is in scope.
