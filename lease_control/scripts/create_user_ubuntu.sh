#!/usr/bin/env zsh

username=$1
password=$2
comment=$3

useradd -s /bin/bash -d /home/$username/ -c $comment -m $username
echo "$password\n$password" | passwd $username

