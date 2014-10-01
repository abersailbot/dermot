























.build/uno/TinyGPS/TinyGPS.o : lib/TinyGPS/TinyGPS.cpp
	@echo [93mTinyGPS/TinyGPS.cpp[0m
	@mkdir -p .build/uno/TinyGPS
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c lib/TinyGPS/TinyGPS.cpp
include .build/uno/TinyGPS/TinyGPS.d



.build/uno/TinyGPS/libTinyGPS.a : .build/uno/TinyGPS/TinyGPS.o
	@echo [92mLinking libTinyGPS.a[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-ar rcs $@ $^







.build/uno/Wire/utility/twi.o : /usr/share/arduino/libraries/Wire/utility/twi.c
	@echo [93mWire/utility/twi.c[0m
	@mkdir -p .build/uno/Wire/utility
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/libraries/Wire/utility/twi.c
include .build/uno/Wire/utility/twi.d






.build/uno/Wire/Wire.o : /usr/share/arduino/libraries/Wire/Wire.cpp
	@echo [93mWire/Wire.cpp[0m
	@mkdir -p .build/uno/Wire
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/libraries/Wire/Wire.cpp
include .build/uno/Wire/Wire.d



.build/uno/Wire/libWire.a : .build/uno/Wire/utility/twi.o .build/uno/Wire/Wire.o
	@echo [92mLinking libWire.a[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-ar rcs $@ $^












.build/uno/SoftwareSerial/SoftwareSerial.o : /usr/share/arduino/libraries/SoftwareSerial/SoftwareSerial.cpp
	@echo [93mSoftwareSerial/SoftwareSerial.cpp[0m
	@mkdir -p .build/uno/SoftwareSerial
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/libraries/SoftwareSerial/SoftwareSerial.cpp
include .build/uno/SoftwareSerial/SoftwareSerial.d



.build/uno/SoftwareSerial/libSoftwareSerial.a : .build/uno/SoftwareSerial/SoftwareSerial.o
	@echo [92mLinking libSoftwareSerial.a[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-ar rcs $@ $^












.build/uno/Servo/Servo.o : /usr/share/arduino/libraries/Servo/Servo.cpp
	@echo [93mServo/Servo.cpp[0m
	@mkdir -p .build/uno/Servo
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/libraries/Servo/Servo.cpp
include .build/uno/Servo/Servo.d



.build/uno/Servo/libServo.a : .build/uno/Servo/Servo.o
	@echo [92mLinking libServo.a[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-ar rcs $@ $^












.build/uno/Time/Time.o : lib/Time/Time.cpp
	@echo [93mTime/Time.cpp[0m
	@mkdir -p .build/uno/Time
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c lib/Time/Time.cpp
include .build/uno/Time/Time.d



.build/uno/Time/libTime.a : .build/uno/Time/Time.o
	@echo [92mLinking libTime.a[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-ar rcs $@ $^







.build/uno/arduino/wiring_analog.o : /usr/share/arduino/hardware/arduino/cores/arduino/wiring_analog.c
	@echo [93marduino/wiring_analog.c[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/wiring_analog.c
include .build/uno/arduino/wiring_analog.d

.build/uno/arduino/WInterrupts.o : /usr/share/arduino/hardware/arduino/cores/arduino/WInterrupts.c
	@echo [93marduino/WInterrupts.c[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/WInterrupts.c
include .build/uno/arduino/WInterrupts.d

.build/uno/arduino/wiring_pulse.o : /usr/share/arduino/hardware/arduino/cores/arduino/wiring_pulse.c
	@echo [93marduino/wiring_pulse.c[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/wiring_pulse.c
include .build/uno/arduino/wiring_pulse.d

.build/uno/arduino/wiring_digital.o : /usr/share/arduino/hardware/arduino/cores/arduino/wiring_digital.c
	@echo [93marduino/wiring_digital.c[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/wiring_digital.c
include .build/uno/arduino/wiring_digital.d

.build/uno/arduino/wiring_shift.o : /usr/share/arduino/hardware/arduino/cores/arduino/wiring_shift.c
	@echo [93marduino/wiring_shift.c[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/wiring_shift.c
include .build/uno/arduino/wiring_shift.d

.build/uno/arduino/wiring.o : /usr/share/arduino/hardware/arduino/cores/arduino/wiring.c
	@echo [93marduino/wiring.c[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/wiring.c
include .build/uno/arduino/wiring.d

.build/uno/arduino/avr-libc/malloc.o : /usr/share/arduino/hardware/arduino/cores/arduino/avr-libc/malloc.c
	@echo [93marduino/avr-libc/malloc.c[0m
	@mkdir -p .build/uno/arduino/avr-libc
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/avr-libc/malloc.c
include .build/uno/arduino/avr-libc/malloc.d

.build/uno/arduino/avr-libc/realloc.o : /usr/share/arduino/hardware/arduino/cores/arduino/avr-libc/realloc.c
	@echo [93marduino/avr-libc/realloc.c[0m
	@mkdir -p .build/uno/arduino/avr-libc
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc   -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/avr-libc/realloc.c
include .build/uno/arduino/avr-libc/realloc.d






.build/uno/arduino/USBCore.o : /usr/share/arduino/hardware/arduino/cores/arduino/USBCore.cpp
	@echo [93marduino/USBCore.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/USBCore.cpp
include .build/uno/arduino/USBCore.d

.build/uno/arduino/Print.o : /usr/share/arduino/hardware/arduino/cores/arduino/Print.cpp
	@echo [93marduino/Print.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/Print.cpp
include .build/uno/arduino/Print.d

.build/uno/arduino/HID.o : /usr/share/arduino/hardware/arduino/cores/arduino/HID.cpp
	@echo [93marduino/HID.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/HID.cpp
include .build/uno/arduino/HID.d

.build/uno/arduino/WMath.o : /usr/share/arduino/hardware/arduino/cores/arduino/WMath.cpp
	@echo [93marduino/WMath.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/WMath.cpp
include .build/uno/arduino/WMath.d

.build/uno/arduino/Tone.o : /usr/share/arduino/hardware/arduino/cores/arduino/Tone.cpp
	@echo [93marduino/Tone.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/Tone.cpp
include .build/uno/arduino/Tone.d

.build/uno/arduino/Stream.o : /usr/share/arduino/hardware/arduino/cores/arduino/Stream.cpp
	@echo [93marduino/Stream.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/Stream.cpp
include .build/uno/arduino/Stream.d

.build/uno/arduino/IPAddress.o : /usr/share/arduino/hardware/arduino/cores/arduino/IPAddress.cpp
	@echo [93marduino/IPAddress.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/IPAddress.cpp
include .build/uno/arduino/IPAddress.d

.build/uno/arduino/main.o : /usr/share/arduino/hardware/arduino/cores/arduino/main.cpp
	@echo [93marduino/main.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/main.cpp
include .build/uno/arduino/main.d

.build/uno/arduino/HardwareSerial.o : /usr/share/arduino/hardware/arduino/cores/arduino/HardwareSerial.cpp
	@echo [93marduino/HardwareSerial.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/HardwareSerial.cpp
include .build/uno/arduino/HardwareSerial.d

.build/uno/arduino/new.o : /usr/share/arduino/hardware/arduino/cores/arduino/new.cpp
	@echo [93marduino/new.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/new.cpp
include .build/uno/arduino/new.d

.build/uno/arduino/CDC.o : /usr/share/arduino/hardware/arduino/cores/arduino/CDC.cpp
	@echo [93marduino/CDC.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/CDC.cpp
include .build/uno/arduino/CDC.d

.build/uno/arduino/WString.o : /usr/share/arduino/hardware/arduino/cores/arduino/WString.cpp
	@echo [93marduino/WString.cpp[0m
	@mkdir -p .build/uno/arduino
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions  -o $@ -c /usr/share/arduino/hardware/arduino/cores/arduino/WString.cpp
include .build/uno/arduino/WString.d



.build/uno/arduino/libarduino.a : .build/uno/arduino/wiring_analog.o .build/uno/arduino/WInterrupts.o .build/uno/arduino/wiring_pulse.o .build/uno/arduino/wiring_digital.o .build/uno/arduino/wiring_shift.o .build/uno/arduino/wiring.o .build/uno/arduino/avr-libc/malloc.o .build/uno/arduino/avr-libc/realloc.o .build/uno/arduino/USBCore.o .build/uno/arduino/Print.o .build/uno/arduino/HID.o .build/uno/arduino/WMath.o .build/uno/arduino/Tone.o .build/uno/arduino/Stream.o .build/uno/arduino/IPAddress.o .build/uno/arduino/main.o .build/uno/arduino/HardwareSerial.o .build/uno/arduino/new.o .build/uno/arduino/CDC.o .build/uno/arduino/WString.o
	@echo [92mLinking libarduino.a[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-ar rcs $@ $^















.build/uno/src/demot.o : .build/uno/src/demot.cpp
	@echo [93msrc/demot.cpp[0m
	@mkdir -p .build/uno/src
	@/usr/share/arduino/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=106 -I/usr/share/arduino/hardware/arduino/cores/arduino -ffunction-sections -fdata-sections -g -Os -w -I/usr/share/arduino/hardware/arduino/variants/standard -Ilib/TinyGPS -I/usr/share/arduino/libraries/Wire -I/usr/share/arduino/libraries/Wire/utility -I/usr/share/arduino/libraries/SoftwareSerial -I/usr/share/arduino/libraries/Servo -Ilib/Time -I/usr/share/arduino/hardware/arduino/cores/arduino -I/usr/share/arduino/hardware/arduino/cores/arduino/avr-libc -fno-exceptions -iquote src  -o $@ -c .build/uno/src/demot.cpp
include .build/uno/src/demot.d







.build/uno/firmware.elf : .build/uno/src/demot.o .build/uno/TinyGPS/libTinyGPS.a .build/uno/Wire/libWire.a .build/uno/SoftwareSerial/libSoftwareSerial.a .build/uno/Servo/libServo.a .build/uno/Time/libTime.a .build/uno/arduino/libarduino.a
	@echo [92mLinking firmware.elf[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-gcc -mmcu=atmega328p -Wl,-Os -Wl,--gc-sections -o $@ $^ -lm


.build/uno/firmware.hex : .build/uno/firmware.elf
	@echo [92mConverting to firmware.hex[0m
	@/usr/share/arduino/hardware/tools/avr/bin/avr-objcopy -O ihex -R .eeprom $^ $@

include .build/uno/src/dependencies.d .build/uno/arduino/dependencies.d .build/uno/TinyGPS/dependencies.d .build/uno/Wire/dependencies.d .build/uno/SoftwareSerial/dependencies.d .build/uno/Servo/dependencies.d .build/uno/Time/dependencies.d

all : .build/uno/firmware.hex
	@true

