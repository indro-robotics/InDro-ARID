# io.sh: colours, output helpers, traps, user-exit cleanup, prompt_* family.

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[1;34m'; BOLD='\033[1m'; NC='\033[0m'

CURRENT_STEP="(not started)"
step() { CURRENT_STEP="$*"; echo -e "\n${BLUE}${BOLD}==> $*${NC}"; }
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
skip() { echo -e "  [SKIP]  $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }

# Case-insensitive y/yes n/no; strip non-letters first (a stray CR breaks the match).
is_yes() { local a="${1//[^A-Za-z]/}"; case "${a,,}" in y|yes) return 0 ;; *) return 1 ;; esac; }
is_no()  { local a="${1//[^A-Za-z]/}"; case "${a,,}" in n|no)  return 0 ;; *) return 1 ;; esac; }

# Validated yes/no prompt. $1 = prompt text, $2 = default on Enter (y or n).
# Re-prompts on anything else, so a stray key is never silently read as "no".
ask_yn() {
    local a
    while true; do
        read -r -p "$1" a || a=""
        a="${a//[^A-Za-z]/}"
        case "${a,,}" in
            y|yes) return 0 ;;
            n|no)  return 1 ;;
            "")    case "${2:-}" in y) return 0 ;; n) return 1 ;; *) echo "  please answer y or n" ;; esac ;;
            *)     echo "  please answer y or n" ;;
        esac
    done
}

# True when this shell runs inside a NoMachine session (NX_* session vars are inherited by
# setup; SSH/console shells have none). Process ancestry is unreliable - the terminal is
# re-parented under the desktop. Setup uses this to leave NoMachine untouched: a reinstall
# would drop the very session running setup.
is_inside_nomachine() {
    [[ -n "${NX_SESSION_ID:-}${NXSESSIONID:-}${NX_RUNNER:-}${NX_CONNECTION:-}${NX_CLIENT:-}" ]]
}

# Clear resume hooks on a user-initiated exit. Not called from the failure() ERR trap:
# a hard failure keeps its checkpoints so './setup.sh --continue' can pick up.
cleanup_user_exit() {
    local home="${HOME_DIR:-$HOME}"
    rm -f "${home}/.arid_resume_setup" \
          "${home}/.arid_setup_continue" \
          "${home}/.arid_run_smoke" \
          "${home}/.arid_setup_log" \
          "${home}/.arid_pending_build_isaac" 2>/dev/null || true
    # camera_focus leftovers: a quit at the focus prompt exits without unwinding the
    # function, so its detached Foxglove bridge + camera pipelines are reaped here.
    if [ -f /tmp/arid_focus_fox.pgid ]; then
        kill -KILL -- -"$(cat /tmp/arid_focus_fox.pgid)" 2>/dev/null || true
        rm -f /tmp/arid_focus_fox.pgid
        ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true
        ros2 service call /gst_camera_manager/cam_down  std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true
    fi
}

# user_exit [CODE] - clear hooks, then exit with CODE (default 1).
user_exit() { cleanup_user_exit; exit "${1:-1}"; }

failure() {
    err "======================================================"
    err "SETUP FAILED in step: ${CURRENT_STEP}"
    err "Line $1: '${BASH_COMMAND}'"
    err "Log: ${LOG_FILE:-<not yet set>}"
    err "Resume where it stopped: ./setup.sh --continue"
    err "======================================================"
    exit 1
}

# Ctrl+C at any time. Default = quit setup; n = continue where we left off.
quit_handler() {
    echo
    (( ${PRE:-0} )) && user_exit 130
    local ans
    read -r -p "  Interrupted. Quit setup? (Enter/q = quit, n = continue): " ans </dev/tty || ans=""
    case "${ans,,}" in n|no) echo "  Continuing where we left off..."; return ;;
                       *) echo "  Setup cancelled."; user_exit 130 ;;
    esac
}

# prompt_failure_action ERR_MSG [REASON]
# Returns 0 = continue setup (skip this section). On the exit choice: clears hooks and exits 1.
prompt_failure_action() {
    err "$1"; [[ -n "${2:-}" ]] && echo "  ${2}"
    (( ${PRE:-0} )) && { warn "non-interactive - exiting on section failure (resumable via --continue)"; exit 1; }
    local ans
    read -r -p "  Continue setup (skip this section) or exit? (c = continue, Enter = exit): " ans || ans=""
    case "${ans,,}" in c|continue) return 0 ;; *) user_exit 1 ;; esac
}

# prompt_section_or_skip PROMPT - returns 0 = run this section, 1 = skip. Not an exit path.
prompt_section_or_skip() {
    (( ${PRE:-0} )) && return 0
    local ans
    read -r -p "  $1 (y/q to skip, Enter = run): " ans || ans=""
    case "${ans,,}" in q|skip) return 1 ;; *) return 0 ;; esac
}

# prompt_core_failure LABEL [DETAIL] -> echoes retry|skip|exit. Prompts on /dev/tty so it
# works under --full; no tty -> exit. Callers handle "exit" and should call user_exit.
prompt_core_failure() {
    err "$1 failed" >&2; [[ -n "${2:-}" ]] && echo "  $2" >&2
    local ans
    read -r -p "  (r)etry / (s)kip / (e)xit setup? [r/s/e]: " ans </dev/tty 2>/dev/null || ans=""
    case "${ans,,}" in r*) echo retry ;; s*) echo skip ;; *) echo exit ;; esac
}
