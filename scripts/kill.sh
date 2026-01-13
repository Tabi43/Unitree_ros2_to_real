#!/bin/bash
sudo ps -A | grep point | awk '{print $1}' | xargs -r sudo kill -9

