#!/bin/bash
# push_to_github.sh — quick sync from vehicle to GitHub

cd /home/wfolsom/simplr-auv/SIMPLR-AUV || exit 1

msg="$1"
if [ -z "$msg" ]; then
  msg="Vehicle update: $(date '+%Y-%m-%d %H:%M:%S')"
fi

git add .
git commit -m "$msg"
git push origin main
