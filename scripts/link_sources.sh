#!/usr/bin/env bash
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do
  SCRIPT_DIR=$(cd -P "$(dirname "$SOURCE")" >/dev/null 2>&1 && pwd)
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE
done
SCRIPT_DIR=$(cd -P "$(dirname "$SOURCE")" >/dev/null 2>&1 && pwd)

BLENDER_VERSION="4.0"
ADDON_DIR="$HOME/.config/blender/$BLENDER_VERSION/scripts/addons/urmoco"
URMOCO_DIR=$(readlink -f "$SCRIPT_DIR/../urmoco")

rm -rf "$ADDON_DIR"
ln -s "$URMOCO_DIR" "$ADDON_DIR"
