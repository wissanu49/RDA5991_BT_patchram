## RDA5991e BT init program

Based on the code from Orange PI 2G-IOT SDK

```
gcc bt_init.c -o bt_init
./bt_init
hciattach -s 921600 /dev/ttyS1 any 921600 flow
```

#### Issues:

- hciattach doesn't work with speed suggested by platform config (3000000)
- scan doesn't work
- discoverable mode doesn't work

#### TODO:

- add command line parameters processing for the UART baud rate
- add compatibility with other BT variants if necessary
