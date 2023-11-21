# shakeometer

## To chage the raspberry to connect to a network instead of hosting a hotspot:
`nmcli d wifi connect <my_wifi> password <password>``

### To change back to hosting hotspot:

###
`nmcli dev wifi show-password" shows the Wi-Fi name and password.

## To find the port the divice is connected to:
`sudo dmesg | grep tty`

## When the IMU is connected set the serial to low latency on the divice:
`setserial /dev/ttyUSB0 low_latency`

