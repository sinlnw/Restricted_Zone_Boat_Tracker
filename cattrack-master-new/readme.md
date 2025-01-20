Notes on Adafruit Feather-M0
============================

* If uploading always fails, try pressing RESET once while IDE is scanning for
  ports.

* In case the node is not responding (due to sleep, etc), press RESET twice to
  force bootloader.

* RFM95's CS must be OUTPUT and set to HIGH to disable the radio module.
  Otherwise the MISO pin will compete the logic level with SD card.

* radio.send() must be supplied with a pointer to length that is PRESET with
  the maximum packet size to be received.  If the given length is zero, the
  returned packet will have a zero length.

* DO NOT use detachInterrupt() inside an ISR.

* Make variables modified in an ISR volatile.

* Accessing SD card at the BYTE-LEVEL must be done in 64 bits.  Literals must
  be suffixed with ULL, e.g., 1ULL.  Any expression potentially resulting in
  anything wider than 32 bits must be casted to uint64_t.

* Although partition type for FAT16 is set to 0x0E, Arduino's SD library still
  thinks it's FAT12 if the number of clusters is less than 4085.

* GPS module reports RMC and GGA sentences separately.  An RMC sentence
  contains timestamp, lat/lon, and datestamp (no quality), while a GGA
  sentence contains timestamp, lat/lon, fix quality, and satellite count (no
  date).  Hence, BOTH sentences must be received and parsed to get all
  required pieces of information.


Notes on Raspberry Pi
=====================
* Raspberry Pi pinouts

    https://www.raspberrypi-spy.co.uk/2012/06/simple-guide-to-the-rpi-gpio-header-and-pins/#prettyPhoto

* Connecting USB-Serial TTL

    https://www.anintegratedworld.com/how-to-connect-raspberry-pi-to-ubuntu-via-usb-cable/

* Setting up WiFi
    https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md

* Setting up real-time clock
    https://www.raspberrypi.org/forums/viewtopic.php?t=161133

* Generate SSH key and copy id

    ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
    ssh-copy-id <host>

* Turn of HDMI.  Put the following in /etc/rc.local

   /usr/bin/tvservice -o

* To make the server bind to ANY address, specify a reverse ssh channel with
  0.0.0.0 at the beginning of the tunnel spec, e.g.,

    ssh -R 0.0.0.0:80:localhost:22222

  In addition, the server must be configured with GatewayPorts=yes in /etc/ssh/sshd_config.

* To clear inactive port bindings, run ssh with -o ExitOnForwardFailure at the
  client.

