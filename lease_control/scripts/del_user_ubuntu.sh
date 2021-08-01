#!/usr/bin/env zsh

username=$1

passwd --lock $username
killall -9 -u $username
deluser --remove-home $username
