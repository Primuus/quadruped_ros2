# Logger Utility

## Usage

### Include logger in your node
```cpp
#include "logger/logger.hpp"

// Create logger instance
Logger logger("node_name");

// Log messages
logger.info("Information message");
logger.warn("Warning message");
logger.error("Error message");
```

### Log levels
- **INFO**: General information
- **WARN**: Warning messages
- **ERROR**: Error messages
