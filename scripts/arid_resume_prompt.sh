#!/bin/bash
# ARID resume-after-reboot prompt.
#
# Sourced (NOT executed) from the ARID block in ~/.bashrc on every interactive shell.
# Lives in its own .sh file so the bashrc heredoc in setup.sh never contains command
# substitution, $(...), backticks, or runtime $VAR - the historical corruption mode (seen
# on SKID and structurally identical here) was a bashrc heredoc with unescaped backticks
# that bash command-substituted at heredoc-write time, capturing setup.sh's stdout and
# writing it INTO ~/.bashrc.
#
# DO NOT inline this into ~/.bashrc. Keep the dangerous code in a normal shell script
# where backticks and $() mean what they look like.

# Gate: only run when the resume flag is armed AND this is a real interactive TTY.
[ -f "$HOME/.arid_resume_setup" ] && [ -t 0 ] && [[ $- == *i* ]] || return 0

# ARID's previous hook auto-ran the resume without prompting; preserve that operator UX.
/bin/bash "${WORKSPACES:-/home/jetson/Desktop/InDro-ARID}/setup.sh" --resume || true
# Clear the flag unconditionally after the resume attempt - if a retry is needed
# (smoke-test failures, transient build issues), run `./setup.sh --resume` manually instead
# of relying on this hook.
rm -f "$HOME/.arid_resume_setup"
