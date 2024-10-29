#!/bin/sh

WORKING_DIR=`dirname $0`

if [ -z "$1" ]; then
    echo " "
    echo "  Usage: $0 <tunnel-name>"
    echo " "
    echo "Create and maintain a reverse SSH tunnel using settings from"
    echo "a corresponding configuration file <tunnel-name>.tunnel.conf"
    exit 1
fi
TUNNEL_NAME=$1
CONF_FILE=$WORKING_DIR/$TUNNEL_NAME.tunnel.conf

if [ ! -f $CONF_FILE ]; then
    echo "Configuration file, $CONF_FILE, not found"
    exit 2
fi

# Load user configuration
. $CONF_FILE

while true; do
    echo `date +"%Y-%m-%d %H:%M:%S"` \
        Establishing SSH tunnel to $SSH_TUNNEL_REMOTE_HOST 1>&2
    ssh -N -4 \
        -v \
        -o ConnectTimeout=30 \
        -o TCPKeepAlive=yes \
        -o ExitOnForwardFailure=yes \
        -o ServerAliveInterval=$SSH_TUNNEL_SERVER_ALIVE_INTERVAL \
        -o ServerAliveCountMax=$SSH_TUNNEL_SERVER_ALIVE_COUNT_MAX \
        -R 0.0.0.0:$SSH_TUNNEL_REMOTE_PORT:$SSH_TUNNEL_LOCAL_HOST:$SSH_TUNNEL_LOCAL_PORT \
        $SSH_TUNNEL_REMOTE_USER@$SSH_TUNNEL_REMOTE_HOST 1>&2
    echo `date +"%Y-%m-%d %H:%M:%S"` SSH tunnel terminated 1>&2
    sleep $SSH_TUNNEL_WAIT_DURATION
done
