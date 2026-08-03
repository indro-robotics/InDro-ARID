#!/bin/bash
# ARID resume-after-reboot prompt. Sourced (NOT executed) from the ARID block in ~/.bashrc.
# Never inline this into ~/.bashrc: backticks and $() inside the bashrc heredoc are
# command-substituted at write time, which corrupts the file it is being written into.

[ -f "$HOME/.arid_resume_setup" ] && [ -t 0 ] && [[ $- == *i* ]] || return 0

echo ""
# Re-prompt on unrecognised input: a stray keypress must not dismiss the resume.
while :; do
    read -r -p "ARID setup is paused after a reboot. Continue setup now? (y/n): " _aridans || { _aridans=""; break; }
    _aridans="${_aridans//[^A-Za-z]/}"; _aridans="${_aridans,,}"
    case "${_aridans}" in y|yes|n|no|"") break ;; *) echo "  please answer y or n" ;; esac
done
# Cleared before the resume runs, so the prompt cannot reappear in another terminal
# while --resume is still working.
rm -f "$HOME/.arid_resume_setup"
case "${_aridans}" in
    y|yes) /bin/bash "${WORKSPACES:-/home/jetson/workspaces}/setup.sh" --resume ;;
esac
unset _aridans
