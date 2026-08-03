#!/bin/bash
# ARID resume-after-reboot prompt.
#
# Sourced, not executed, from the ARID block in ~/.bashrc on every interactive shell.
# Never inline this into ~/.bashrc: backticks and $() inside a bashrc heredoc are
# command-substituted at write time.

[ -f "$HOME/.arid_resume_setup" ] && [ -t 0 ] && [[ $- == *i* ]] || return 0

echo ""
# Unrecognised input re-prompts: any accepted answer clears the resume flag below, so a
# stray key must not count as one.
while :; do
    read -r -p "ARID setup is paused after a reboot. Continue setup now? (y/n): " _aridans || { _aridans=""; break; }
    _aridans="${_aridans//[^A-Za-z]/}"; _aridans="${_aridans,,}"
    case "${_aridans}" in y|yes|n|no|"") break ;; *) echo "  please answer y or n" ;; esac
done
# Cleared before the branch, not after it: any answer must stop the prompt reappearing in
# other terminals, including while --resume is still running.
rm -f "$HOME/.arid_resume_setup"
case "${_aridans}" in
    y|yes) /bin/bash "${WORKSPACES:-/home/jetson/workspaces}/setup.sh" --resume ;;
esac
unset _aridans
