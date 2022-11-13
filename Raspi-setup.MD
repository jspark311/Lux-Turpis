# Raspi setup

Build is using DietPi on a Raspberry Pi 2.

Hostname was set.


## Command log for setup

```
vi /etc/hostname

# Add user/SSH keys, sudo, etc...
adduser ian
usermod -a -G sudo ian
su - ian
mkdir .ssh
vi .ssh/authorized_keys
chmod 755 .ssh
chmod 600 .ssh/authorized_keys

apt-get install minicom gpsd gpsd-clients pps-tools ntp net-tools openvpn python3-venv cmake
minicom -b 9600 -D /dev/ttyAMA0
i2cdetect -y 1

echo 18 > /sys/class/gpio/export; echo "out" > /sys/class/gpio/gpio18/direction   # Red LED
echo 23 > /sys/class/gpio/export; echo "out" > /sys/class/gpio/gpio23/direction   # Green LED
echo 24 > /sys/class/gpio/export; echo "out" > /sys/class/gpio/gpio24/direction   # GPS Enable (Yellow LED)
echo 1 > /sys/class/gpio/gpio24/value; echo 0 > /sys/class/gpio/gpio23/value; echo 0 > /sys/class/gpio/gpio18/value
cat /sys/class/gpio/gpio18/value; cat /sys/class/gpio/gpio23/value; cat /sys/class/gpio/gpio24/value;

# The PPS pin is owned by the PPS kernel module. This was used for testing.
#echo 22 > export; echo "in" > gpio22/direction    # GPS PPS
#cat gpio22/value

sed -i 's/USBAUTO="true"/USBAUTO="false"/g' /etc/default/gpsd
sed -i 's:DEVICES="":DEVICES="/dev/ttyAMA0 /dev/pps0":g' /etc/default/gpsd
sed -i 's:GPSD_OPTIONS="":GPSD_OPTIONS="-n":g' /etc/default/gpsd
systemctl enable gpsd

# Remove DHCP hooks to invoke NTP...
if [ -f  "/etc/dhcp/dhclient-exit-hooks.d/ntp" ]; then mv /etc/dhcp/dhclient-exit-hooks.d/ntp /etc/dhcp/dhclient-exit-hooks.d/xxxntp-orig; fi

touch /etc/rc.local
chmod a+x /etc/rc.local
sed -i '$ s/exit 0/gpspipe -r -n 1 \&/g' /etc/rc.local ; echo exit 0 >> /etc/rc.local

echo dtoverlay=pps-gpio,gpiopin=22 >> /boot/config.txt
echo KERNEL==\"ttyAMA0\", SUBSYSTEM==\"tty\", DRIVER==\"\", OWNER==\"root\", GROUP==\"tty\", MODE==\"0777\", SYMLINK+=\"gps0\" >> /etc/udev/rules.d/09-pps.rules
echo pps-gpio >> /etc/modules
reboot

ppstest /dev/pps0
vi /etc/ntp.conf

# Testing the hardware RTC... These lines allow manual bring-up.
#modprobe i2c:mcp7941x
#echo mcp7941x 0x6f > /sys/class/i2c-dev/i2c-1/device/new_device
#hwclock -w

# That seemed to work. Add the module, and dtoverlay...
echo dtoverlay=i2c-rtc,mcp7941x >> /boot/config.txt
echo rtc-mcp7941x >> /etc/modules

# Remove the fake-hwclock package, and the exit(0) line from the hwclock-set script.
apt remove fake-hwclock
update-rc.d -f fake-hwclock remove
vi /lib/udev/hwclock-set

# Set the current datetime...
date
hwclock -w
hwclock -r

# /dev/rtc0 ought to be present after this reboot.
reboot
hwclock -r
ntpq -crv -pn

# TODO: We need to refuse to serve time if there hasn't been a GPS fix since last reboot,
and the hardware clock backup is dead.
```

How nice... DietPi came with the support enabled and tooling installed for the
hardware tRNG. So no extra work needed there.

I think the last step is to grab a copy of ESP-IDF, and install for the ESP32...

```
git clone https://github.com/espressif/esp-idf.git
cd esp-idf/
./install.sh esp32
```

#### References

https://www.slsmk.com/how-to-setup-a-gps-pps-ntp-time-server-on-raspberry-pi/
