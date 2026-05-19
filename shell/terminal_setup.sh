#!/bin/bash

# Add this in bashrc
# Check for an optional argument to set PS1 
# if [ -n "$CUSTOM_PS1" ]; then 
#     export PS1="$CUSTOM_PS1" 
# fi

#simple
#PS1='\[\033]0;test1\007\]\u@\h:\w\$ '
#original
#PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
export GREEN='\[\033[01;32m\]'
export WHITE='\[\033[00m\]'
export BLUE='\[\033[01;34m\]'
export PROMPT="$GREEN"'\u@\h'"$WHITE"':'"$BLUE"'\w'"$WHITE"'\$ '
PS1='\[\033]0;test1\007\]'"$PROMPT"
cd ~/free/ws
clear

gnome-terminal --tab -- bash -c " 
    export CUSTOM_PS1='\[\033]0;test2\007\]$PROMPT'; 
    cd ~/free;
    clear;
    exec bash;
"

gnome-terminal --tab -- bash -c " 
    export CUSTOM_PS1='\[\033]0;test3\007\]$PROMPT'; 
    cd ~/Documents;
    clear;
    exec bash;
"
