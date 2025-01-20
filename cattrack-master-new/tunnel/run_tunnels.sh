#!/bin/sh

WORKING_DIR=`dirname $0`

cd ${WORKING_DIR}
nohup ./ssh-tunnel.sh ssh >> ssh.log &
#nohup ./ssh-tunnel.sh web >> web.log &
