# io.sh: colours, output helpers, traps, user-exit cleanup, prompt_* family.

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[1;34m'; BOLD='\033[1m'; NC='\033[0m'

CURRENT_STEP="(not started)"
step() { CURRENT_STEP="$*"; echo -e "\n${BLUE}${BOLD}==> $*${NC}"; }
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
skip() { echo -e "  [SKIP]  $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }

# Non-letters are stripped before matching: a CR from a pasted or piped answer breaks it.
is_yes() { local a="${1//[^A-Za-z]/}"; case "${a,,}" in y|yes) return 0 ;; *) return 1 ;; esac; }
is_no()  { local a="${1//[^A-Za-z]/}"; case "${a,,}" in n|no)  return 0 ;; *) return 1 ;; esac; }

# $1 = prompt text, $2 = the answer Enter means (y or n). Unrecognised input re-prompts
# rather than resolving to no.
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

# NX_* session vars are inherited by every shell in the session and absent over SSH or on
# the console. Process ancestry does not answer this: the terminal is re-parented under the
# desktop session.
is_inside_nomachine() {
    [[ -n "${NX_SESSION_ID:-}${NXSESSIONID:-}${NX_RUNNER:-}${NX_CONNECTION:-}${NX_CLIENT:-}" ]]
}

# $1 = wired NIC. The default route is the test, not carrier: a plugged-in LiDAR raises
# carrier on the LiDAR NIC without setup running over it.
is_provisioning_link() {
    ip route show default 2>/dev/null | grep -qE "^default .* dev ${1}( |$)"
}

# User-initiated exits only. The ERR trap must not call this: a failed run keeps its hooks
# so --resume can pick the run back up.
cleanup_user_exit() {
    local home="${HOME_DIR:-$HOME}"
    rm -f "${home}/.arid_resume_setup" \
          "${home}/.arid_setup_continue" \
          "${home}/.arid_run_smoke" \
          "${home}/.arid_setup_log" \
          "${home}/.arid_pending_build_isaac" 2>/dev/null || true
    # Quitting at the camera_focus prompt exits without unwinding that function, leaving its
    # backgrounded Foxglove bridge and the camera pipeline running. Reaped here instead.
    if [ -f /tmp/arid_focus_fox.pgid ]; then
        kill -KILL -- -"$(cat /tmp/arid_focus_fox.pgid)" 2>/dev/null || true
        rm -f /tmp/arid_focus_fox.pgid
        ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true
    fi
}

# $1 = exit code, default 1.
user_exit() { cleanup_user_exit; exit "${1:-1}"; }

failure() {
    err "======================================================"
    err "SETUP FAILED in step: ${CURRENT_STEP}"
    err "Line $1: '${BASH_COMMAND}'"
    err "Log: ${LOG_FILE:-<not yet set>}"
    err "======================================================"
    exit 1
}

quit_handler() {
    echo
    (( ${PRE:-0} )) && user_exit 130
    local ans
    read -r -p "  Interrupted. Quit setup? (Enter/q = quit, n = continue): " ans </dev/tty || ans=""
    case "${ans,,}" in n|no) echo "  Continuing where we left off..."; return ;;
                       *) echo "  Setup cancelled."; user_exit 130 ;;
    esac
}

# $1 = error message, $2 = optional reason. Returns 0 to continue with this section skipped;
# the exit choice does not return.
prompt_failure_action() {
    err "$1"; [[ -n "${2:-}" ]] && echo "  ${2}"
    (( ${PRE:-0} )) && { warn "non-interactive - exiting on section failure (resumable via --resume)"; exit 1; }
    local ans
    read -r -p "  Continue setup (skip this section) or exit? (c = continue, Enter = exit): " ans || ans=""
    case "${ans,,}" in c|continue) return 0 ;; *) user_exit 1 ;; esac
}

# $1 = prompt text. Returns 0 to run the section, 1 to skip it.
prompt_section_or_skip() {
    (( ${PRE:-0} )) && return 0
    local ans
    read -r -p "  $1 (y/q to skip, Enter = run): " ans || ans=""
    case "${ans,,}" in q|skip) return 1 ;; *) return 0 ;; esac
}

# $1 = label, $2 = optional detail. Echoes retry, skip or exit. Reads /dev/tty rather than
# stdin so the prompt still reaches the operator under --full; with no tty the answer is exit.
prompt_core_failure() {
    err "$1 failed" >&2; [[ -n "${2:-}" ]] && echo "  $2" >&2
    local ans
    read -r -p "  (r)etry / (s)kip / (e)xit setup? [r/s/e]: " ans </dev/tty 2>/dev/null || ans=""
    case "${ans,,}" in r*) echo retry ;; s*) echo skip ;; *) echo exit ;; esac
}
