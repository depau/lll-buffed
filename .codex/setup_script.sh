#!/bin/bash
set -euo pipefail

# Install PlatformIO
curl -L -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
export PATH="$HOME/.platformio/penv/bin:$PATH"

# Install PlatformIO packages
pio pkg install
pio check
pio test || true
