#!/usr/bin/env bash
sudo mkdir -p /etc/ursim/programs
sudo cp ursim.container /etc/containers/systemd/ursim.container

sudo systemctl daemon-reload
sudo systemctl restart ursim.service
