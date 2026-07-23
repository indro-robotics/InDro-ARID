#!/bin/bash
# ARID resume-after-reboot prompt. Sourced (NOT executed) from the ARID block in ~/.bashrc.
# DO NOT inline into ~/.bashrc: backticks/$() in the bashrc heredoc once command-substituted
# at write time and corrupted ~/.bashrc; keep the dangerous code in a real script.

# Gate: only run when the resume flag is armed AND this is a real interactive TTY.
[ -f "$HOME/.arid_resume_setup" ] && [ -t 0 ] && [[ $- == *i* ]] || return 0

/bin/bash "${WORKSPACES:-/home/jetson/workspaces}/setup.sh" --resume || true
# Flag cleared unconditionally; retry manually with ./setup.sh --resume.
rm -f "$HOME/.arid_resume_setup"
