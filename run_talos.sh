#!/bin/bash
STARTDIR=`pwd`

#Switch to this directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"


cd bin
./run_talos.sh $@
cd ..

cd "$STARTDIR"

exit 0
