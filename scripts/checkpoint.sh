#!/usr/bin/env bash

set -e

if [ -z "$1" ]; then
    echo "Usage:"
    echo './scripts/checkpoint.sh "commit message"'
    exit 1
fi

MESSAGE="$1"

echo "================================="
echo "ROS 2 Project Checkpoint"
echo "================================="

source /opt/ros/jazzy/setup.bash

echo
echo "[1/4] Building workspace..."

colcon build --symlink-install

echo
echo "[2/4] Checking repository..."

git status --short

if [ -z "$(git status --porcelain)" ]; then
    echo "No changes to commit."
    exit 0
fi

echo
echo "[3/4] Creating commit..."

git add .
git commit -m "$MESSAGE"

echo
echo "[4/4] Uploading to GitHub..."

git push origin main

echo
echo "================================="
echo "Checkpoint successfully uploaded"
echo "================================="

