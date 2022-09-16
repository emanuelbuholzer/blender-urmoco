#!/usr/bin/env bash

rm -r dist
mkdir -p dist

echo "Bundling urmoco blender addon"
TMP=$(mktemp -d)
PREV_PWD=$(pwd)
cp -R urmoco $TMP/
pushd $TMP
zip -r urmoco.zip urmoco
cp $TMP/urmoco.zip $PREV_PWD/dist/urmoco.zip
popd > /dev/null

echo -e "\nAdding assets"
cp -v assets/* dist

echo -e "\nBundle created at" $PREV_PWD/dist
