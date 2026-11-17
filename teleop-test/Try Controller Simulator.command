#!/bin/bash
set -euo pipefail
exec bash "$(dirname -- "$0")/Open ARX Controller.command" --simulation
