#!/usr/bin/env zsh

username=$1
password=$2

useradd -s /bin/bash -d /home/$username/ -m $username
echo "$password\n$password" | passwd $username

