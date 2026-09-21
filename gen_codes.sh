#!/usr/bin/env bash
# Generate N unique fresh random launch codes, 3-6 chars from A-Z0-9.
# Avoid codes that YAML/members cannot round-trip: pure digits (1E5/0X1F
# arrive as numbers) and YAML booleans (YES/OFF/TRUE...). Usage: ./gen_codes.sh 20
set -euo pipefail

COUNT="${1:-3}"
python3 - "$COUNT" <<'PY'
import random
import string
import sys

count = int(sys.argv[1])
alphabet = string.ascii_uppercase + string.digits
bools = {
    'Y', 'YES', 'N', 'NO', 'TRUE', 'FALSE', 'ON', 'OFF',
    'T', 'F', 'YE', 'OU', 'OF',
}
codes = set()
while len(codes) < count:
    code = ''.join(random.choice(alphabet) for _ in range(random.randint(3, 6)))
    if code.isdigit():
        continue
    if not any(c.isalpha() for c in code):
        continue
    if code in bools:
        continue
    codes.add(code)
print(' '.join(sorted(codes)))
PY
