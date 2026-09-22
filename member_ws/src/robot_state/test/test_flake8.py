"""Style gate, run out-of-process on purpose.

flake8 runs its checks in a ``multiprocessing.Pool`` sized to ``cpu_count()``,
and on Linux that pool is forked. Forking out of the pytest process is not
safe here: by the time this test runs, earlier tests have left OpenCV's
worker threads running, and a child forked from a multi-threaded parent
inherits their held mutexes. In CI that child segfaults and the parent then
blocks forever on a result that never arrives, which wedged the job until it
hit its limit. A fresh interpreter is single-threaded, so the pool it forks
is safe.
"""

from pathlib import Path
import subprocess
import sys

_RUNNER = """
import sys
from ament_flake8.main import main_with_errors

code, errors = main_with_errors(argv=['--linelength', '99', sys.argv[1]])
sys.stdout.write('\\n'.join(errors))
sys.exit(code)
"""


def test_flake8():
    package = Path(__file__).resolve().parents[1]
    finished = subprocess.run(
        [sys.executable, '-c', _RUNNER, str(package)],
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert finished.returncode == 0, finished.stdout + finished.stderr
