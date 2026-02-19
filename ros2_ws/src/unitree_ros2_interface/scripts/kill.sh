#!/bin/bash
ps -A | grep point | awk '{print $1}' | xargs kill -9