#!/bin/bash

docker build --rm $@ -t limo_bot:sim -f "$(dirname "$0")/../../docker/sim.Dockerfile" "$(dirname "$0")/../.."