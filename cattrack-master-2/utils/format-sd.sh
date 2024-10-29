#!/bin/sh

if [ $UID -ne 0 ]; then
    echo "This script must be run as root."
    exit 1
fi

if [ -z $3 ]; then
    echo "Usage: $0 <disk> <data-part> <log-part>"
    exit 1
fi

echo "WARNING: ALL DATA ON THE SPECIFIED DISK WILL BE LOST!"
echo "Press ENTER to continue..."
read
echo "Writing MBR..."
dd if=mbr.bin of=$1

echo "Press ENTER to continue..."
read

echo "Formatting data partition..."
dd if=part-data.bin of=$2

echo
echo "Formatting log partition..."
dd if=part-log.bin of=$3
