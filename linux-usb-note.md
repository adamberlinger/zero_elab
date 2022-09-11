*English description below*

# Možné problémy s VCOM USB pod systémy GNU/Linux

*Následující text byl sepsán v roce 2016 je možné že některé části již nejsou aktuální*

Operační systém linux má výhodu, že nepotřebuje ovladač pro zařízení typu virtual COM port (CDC třída USB).
Některé distribuce však mohou přistupovat k těmto zařízením poněkud nestandardně.
Zde je stručný návod pro distribuci Debian, ale na jiných distribucích bude postup podobný.
Navíc některé distribuce jako např. Ubuntu jsou od Debianu odvozené.

Zaprvé je vhodné povolit přístup k těmto zařízením bez administrátorských práv.
To je možné udělat přídáním uživatele do skupiny `dialout`. Stačí na to příkaz
`usermod -a -G dialout MY_USER_NAME` (pochopitelně s administrátorskými právy).

Další problém je, že systém může vyskoušet, zda připojené zařízení není náhodou modem.
To je realizováno posílání tzv. AT příkazů do zařízení. To přináší dva problémy:
 - zařízení dostává data, která neočekává (AT příkazy)
 - uživatel musí počkat, až systém přestane se "testem" a teprve potom se může připojit k zařízení

Řešením je vypnout toto "testování" pro daný typ zařízení podle USB VID/PID
(Vendor a Přoduct ID). V distribuci Debian toto lze dosáhnout vytvořením souboru
`/etc/udev/rules.d/77-st-devices.rules` (opět s administratorskými právy) s následujícím obsahem:

```
# ST Microelectronics VCOM port and ST-Link devices (version 2 & 2.1)
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374a", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
```

Tímto se "testování" provede pouze na výslovný příkaz uživatele.
Soubor obsahuje VID/PID pro STMicroelectronics virtual COM port a ST-Link verze 2/2.1

(Pozn.: Částečně je problém i na straně USB zařízení,
které mají nastaveno v USB deskriptorech, že AT příkazy mohou přijímat.
Bohužel vypnutí tohoto příznaku zatím nemá na chování vliv a autoři příslušné
aplikace ModemManager tuto variantu sice zvažují, ale její implementace není
triviální, neboť vyžaduje přístup k informacím z Linuxového jádra.

V budoucnu se snad tato zařízení objeví na tzv. "gray-listu" projektu ModemManager
a budou vypnuta ve výchozím nastavení. Pak nebude třeba vytvářet výše zmíněný soubor.)

Stránky projektu ModemManager: https://www.freedesktop.org/wiki/Software/ModemManager/

# Possible problems with VCOM USB in GNU/Linux systems

*Please note that this information is comming from around 2016 and might be outdated.*

GNU/Linux based operating systems don't require dedicated USB CDC (or VCOM) drivers as they are already included in the system. However some distributions might access the devices in not expected way. Here is short tutorial for Debian distribution, but it might apply to other distributions such as Ubuntu, which is based on Debian.

First it is necessary to allow regular user to access the device (without using `sudo` or similar). This is done by adding the user to the `dialout` group. To do that you can run the following run (under the root): 
```
usermod -a -G dialout MY_USER_NAME
```

Another issue I encountered, is that some systems scan and probe all USB CDC devices for modems. This is done by sending some AT commands when the device is connected. This can lead to following issues:
 - The device receives unexpected and unknown data
 - The device is blocked for some time, so user must wait before this probing is finished

It is possible to disable this probing base on USB VID/PID (Vendor ID and Product ID) used by the particular device. In Debian this can be done by creating file `/etc/udev/rules.d/77-st-devices.rules` (we need root access) with following content:

```
# ST Microelectronics VCOM port and ST-Link devices (version 2 & 2.1)
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374a", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", ENV{ID_MM_DEVICE_MANUAL_SCAN_ONLY}="1"
```

This will limit the probing only to explicit command from the user. Above lines contain VID/PID for STMicroelectronics virtual COM port and ST-Link version 2/2.1.

(Note: Partially this might be also issue of the USB device, since in USB descriptors some devices have stated that they will accept AT commands. However when I tested changin this flag, it didn't fix the issue. The ModemManage developers are considering the make such change, but it is not straight-forward since it might require access to Linux core.

In the future maybe such devices will appear on the "gray-list"  of ModemManager. In such case the scanning would be disabled by default)

V budoucnu se snad tato zařízení objeví na tzv. "gray-listu" projektu ModemManager
a budou vypnuta ve výchozím nastavení. Pak nebude třeba vytvářet výše zmíněný soubor.)

Stránky projektu ModemManager: https://www.freedesktop.org/wiki/Software/ModemManager/
