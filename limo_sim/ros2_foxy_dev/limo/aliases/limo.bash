# Source this file from Bash to load the physical robot shortcuts.
_limo_aliases_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

printf -v _limo_alias_command 'bash %q' "$_limo_aliases_dir/../build-robot.sh"
alias cb_limo="$_limo_alias_command"

printf -v _limo_alias_command 'cd -- %q' "$_limo_aliases_dir/../../../workspace"
alias wsp="$_limo_alias_command"

printf -v _limo_alias_command 'bash %q' "$_limo_aliases_dir/../launch-robot.sh"
alias start="$_limo_alias_command"

unset _limo_aliases_dir _limo_alias_command
