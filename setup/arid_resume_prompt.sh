#!/bin/bash
# ARID resume-after-reboot prompt. Sourced (NOT executed) from the ARID block in ~/.bashrc.
# DO NOT inline into ~/.bashrc: backticks/$() in the bashrc heredoc once command-substituted
# at write time and corrupted ~/.bashrc; keep the dangerous code in a real script.

# Gate: only run when the resume flag is armed AND this is a real interactive TTY.
[ -f "$HOME/.arid_resume_setup" ] && [ -t 0 ] && [[ $- == *i* ]] || return 0

echo ""
# Re-prompt on garbage so a stray key cannot silently dismiss the resume. Enter or
# Ctrl+C/Ctrl+D dismiss without resuming; retry later with `./setup.sh --resume`.
while :; do
    read -r -p "ARID setup is paused after a reboot. Continue setup now? (y/n): " _aridans || { _aridans=""; break; }
    _aridans="${_aridans//[^A-Za-z]/}"; _aridans="${_aridans,,}"
    case "${_aridans}" in y|yes|n|no|"") break ;; *) echo "  please answer y or n" ;; esac
done
# Clear the flag the moment the operator engages so the prompt cannot reappear in other
# terminals (including while --resume is still running).
rm -f "$HOME/.arid_resume_setup"
case "${_aridans}" in
    y|yes) /bin/bash "${WORKSPACES:-/home/jetson/workspaces}/setup.sh" --resume ;;
esac
unset _aridans
