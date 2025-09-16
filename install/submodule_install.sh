#!/bin/bash

REPO_URL="https://github.com/TimLakemann/ami.git"
TARGET_DIR="ami"
REV="f9b44c56297effa62badf9cab835b06a6528e074" # Change to "master" if required
LINK_TARGET="../include/ami"


if [ ! -d "$TARGET_DIR" ]; then
    echo "Cloning repository..."
    git clone "$REPO_URL" "$TARGET_DIR"
fi

cd "$TARGET_DIR"

git checkout "$REV"

echo "Initializing and updating submodules..."
git submodule update --init --recursive

if [ -L "$LINK_TARGET" ]; then
    echo "Symbolic link already exists."
else
    ln -s "$(pwd)" "$LINK_TARGET"
    echo "Symbolic link created: $(pwd) -> $LINK_TARGET"
fi
