#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# == 0 ]]; then
  bashrc_path="$HOME/.bashrc"
elif [[ $# == 2 && "$1" == --bashrc ]]; then
  bashrc_path="$2"
else
  echo "Usage: bash setup-shell.sh [--bashrc FILE]" >&2
  exit 2
fi

python3 - "$SCRIPT_DIR" "$bashrc_path" <<'PY'
from pathlib import Path
import shlex
import shutil
import sys

script_dir = Path(sys.argv[1]).resolve()
bashrc = Path(sys.argv[2]).expanduser()
install_setup = (script_dir / "../../workspace/install/setup.bash").resolve()
aliases = script_dir / "aliases/limo.bash"
begin = "# >>> LIMO shell setup >>>"
end = "# <<< LIMO shell setup <<<"
block = "\n".join([
    begin,
    "if [[ -f /opt/ros/foxy/setup.bash ]]; then",
    "  source /opt/ros/foxy/setup.bash",
    "fi",
    "if [[ -f {} ]]; then".format(shlex.quote(str(install_setup))),
    "  source {}".format(shlex.quote(str(install_setup))),
    "fi",
    "if [[ -f {} ]]; then".format(shlex.quote(str(aliases))),
    "  source {}".format(shlex.quote(str(aliases))),
    "fi",
    end,
    "",
])
original = bashrc.read_text() if bashrc.exists() else ""
lines = original.splitlines(keepends=True)
starts = [i for i, line in enumerate(lines) if line.rstrip("\r\n") == begin]
ends = [i for i, line in enumerate(lines) if line.rstrip("\r\n") == end]
if not starts and not ends:
    updated = original + ("\n" if original and not original.endswith("\n") else "") + block
elif len(starts) == len(ends) == 1 and starts[0] < ends[0]:
    updated = "".join(lines[:starts[0]]) + block + "".join(lines[ends[0] + 1:])
else:
    sys.exit("Invalid LIMO setup markers in {}. No changes made.".format(bashrc))

if updated != original:
    backup = bashrc.with_name(bashrc.name + ".limo-backup")
    if bashrc.exists() and not backup.exists() and not backup.is_symlink():
        shutil.copy2(str(bashrc), str(backup))
    bashrc.write_text(updated)
print("LIMO shell setup configured in {}".format(bashrc))
print("Open a new terminal or run: source {}".format(shlex.quote(str(bashrc))))
PY
