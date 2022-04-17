from datetime import datetime, timezone, timedelta
import os
import time

# Target datetime to set
tt = datetime(year=2022, month=4, day=17, hour=1, minute=40, second=35, tzinfo=timezone.utc)

# Construct RTC time and date register contents from target time
rtc_tr = 1048576*(tt.hour // 10) + \
         65536*(tt.hour % 10) + \
         4096*(tt.minute // 10) + \
         256*(tt.minute % 10) + \
         16*(tt.second // 10) + \
         (tt.second % 10);
rtc_dr = 1048576*((tt.year-2000) // 10) + \
         65536*((tt.year-2000) % 10) + \
         8192*(tt.isoweekday()) + \
         4096*(tt.month // 10) + \
         256*(tt.month % 10) + \
         16*(tt.day // 10) + \
         (tt.day % 10);

# Create J-Link Commander script file with the necessary JTAG commands and register writes
str = """
r
w4 0x40021038, 0x10000000 // set PWREN
w4 0x40007000, 0x00001100 // set DBP
w4 0x40021050, 0x0c050504 // set RTCEN, RTCSEL_LSE, LSEBYP, LSEON
w4 0x40002824, 0x000000ca // RTC->WPR
w4 0x40002824, 0x00000053 // RTC->WPR
w4 0x4000280c, 0x00000080 // RTC->ISR set INIT
sleep 1
w4 0x40002810, 0x007f00ff // RTC->PRER
w4 0x40002800, 0x{:08x} // RTC->TR
w4 0x40002804, 0x{:08x} // RTC->DR
w4 0x4000280c, 0x00000000 // RTC->ISR clear INIT
q
""".format(rtc_tr, rtc_dr)

print(str)
with open('rtc.jlink', 'w+') as f:
    f.writelines(str)

d = datetime.now(timezone.utc)
print('Current time {}\n Target time {}\n'.format(d, tt))

# Run a bit early to account for processing time
tt = tt - timedelta(milliseconds=600)

if d < tt:
    while datetime.now(timezone.utc) < tt:
        time.sleep(0.1)

    os.system(r'C:\"Program Files (x86)\SEGGER\JLink\jlink.exe" -device STM32L031F4 -if SWD -speed 1000 -CommanderScript rtc.jlink')
else:
    print('Target time has already passed; doing nothing.')
