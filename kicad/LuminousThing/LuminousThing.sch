EESchema Schematic File Version 4
LIBS:LuminousThing-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "Luminous Things Multi-Lamp Controller"
Date "2021-01-22"
Rev ""
Comp "Augury"
Comment1 "David L Norris"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:ESP-12E U1
U 1 1 600B4BC5
P 6100 3950
F 0 "U1" H 6100 4928 50  0000 C CNN
F 1 "ESP-12E" H 6100 4837 50  0000 C CNN
F 2 "RF_Module:ESP-12E" H 6100 3950 50  0001 C CNN
F 3 "http://wiki.ai-thinker.com/_media/esp8266/esp8266_series_modules_user_manual_v1.1.pdf" H 5750 4050 50  0001 C CNN
	1    6100 3950
	1    0    0    -1  
$EndComp
$Comp
L 4xxx:4016 U2
U 1 1 600B4C90
P 8100 3400
F 0 "U2" V 8146 3273 50  0000 R CNN
F 1 "4016" V 8055 3273 50  0000 R CNN
F 2 "" H 8100 3400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4016b.pdf" H 8100 3400 50  0001 C CNN
	1    8100 3400
	0    -1   -1   0   
$EndComp
$Comp
L 4xxx:4016 U2
U 3 1 600B4DA3
P 8100 5100
F 0 "U2" V 8146 4973 50  0000 R CNN
F 1 "4016" V 8055 4973 50  0000 R CNN
F 2 "" H 8100 5100 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4016b.pdf" H 8100 5100 50  0001 C CNN
	3    8100 5100
	0    -1   -1   0   
$EndComp
$Comp
L 4xxx:4016 U2
U 5 1 600B4EC7
P 1350 6750
F 0 "U2" H 1580 6796 50  0000 L CNN
F 1 "4016" H 1580 6705 50  0000 L CNN
F 2 "" H 1350 6750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4016b.pdf" H 1350 6750 50  0001 C CNN
	5    1350 6750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRLB8721PBF Q4
U 1 1 600B508E
P 8900 2300
F 0 "Q4" H 9105 2346 50  0000 L CNN
F 1 "IRLB8721PBF" H 9105 2255 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9150 2225 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/irlb8721pbf.pdf?fileId=5546d462533600a40153566056732591" H 8900 2300 50  0001 L CNN
	1    8900 2300
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRLB8721PBF Q3
U 1 1 600B50F7
P 8900 3100
F 0 "Q3" H 9105 3146 50  0000 L CNN
F 1 "IRLB8721PBF" H 9105 3055 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9150 3025 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/irlb8721pbf.pdf?fileId=5546d462533600a40153566056732591" H 8900 3100 50  0001 L CNN
	1    8900 3100
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRLB8721PBF Q2
U 1 1 600B5160
P 8900 3950
F 0 "Q2" H 9105 3996 50  0000 L CNN
F 1 "IRLB8721PBF" H 9105 3905 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9150 3875 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/irlb8721pbf.pdf?fileId=5546d462533600a40153566056732591" H 8900 3950 50  0001 L CNN
	1    8900 3950
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRLB8721PBF Q1
U 1 1 600B51DC
P 8900 4800
F 0 "Q1" H 9105 4846 50  0000 L CNN
F 1 "IRLB8721PBF" H 9105 4755 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 9150 4725 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/irlb8721pbf.pdf?fileId=5546d462533600a40153566056732591" H 8900 4800 50  0001 L CNN
	1    8900 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 2300 8500 2300
Wire Wire Line
	8100 3100 8500 3100
Wire Wire Line
	8100 3950 8500 3950
Wire Wire Line
	8100 4800 8500 4800
Entry Wire Line
	7200 4150 7300 4250
Entry Wire Line
	7200 4050 7300 4150
Entry Wire Line
	7200 3950 7300 4050
Entry Wire Line
	7300 5000 7400 5100
Entry Wire Line
	7300 4150 7400 4250
Entry Wire Line
	7300 3300 7400 3400
Entry Wire Line
	7300 2500 7400 2600
Wire Wire Line
	7200 3950 6700 3950
Wire Wire Line
	7200 4050 6700 4050
Wire Wire Line
	7200 4150 6700 4150
Entry Wire Line
	7200 4350 7300 4450
Wire Wire Line
	6700 4350 7200 4350
Text Label 6800 3950 0    50   ~ 0
Sig_Spot2
Text Label 6800 4050 0    50   ~ 0
Sig_Spot1
Text Label 6800 4150 0    50   ~ 0
Sig_Spot3
Text Label 6800 4350 0    50   ~ 0
Sig_Spot4
$Comp
L 4xxx:4016 U2
U 4 1 600B4E35
P 8100 2600
F 0 "U2" V 8146 2473 50  0000 R CNN
F 1 "4016" V 8055 2473 50  0000 R CNN
F 2 "" H 8100 2600 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4016b.pdf" H 8100 2600 50  0001 C CNN
	4    8100 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7400 2600 7800 2600
Wire Wire Line
	7400 3400 7800 3400
Wire Wire Line
	7400 4250 7800 4250
Wire Wire Line
	7400 5100 7800 5100
Text Label 7400 2600 0    50   ~ 0
Sig_Spot4
Text Label 7400 3400 0    50   ~ 0
Sig_Spot1
Text Label 7400 4250 0    50   ~ 0
Sig_Spot2
Text Label 7400 5100 0    50   ~ 0
Sig_Spot3
$Comp
L power:GND #PWR0101
U 1 1 600B5586
P 9100 2600
F 0 "#PWR0101" H 9100 2350 50  0001 C CNN
F 1 "GND" V 9105 2472 50  0000 R CNN
F 2 "" H 9100 2600 50  0001 C CNN
F 3 "" H 9100 2600 50  0001 C CNN
	1    9100 2600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 600B5603
P 9100 3400
F 0 "#PWR0102" H 9100 3150 50  0001 C CNN
F 1 "GND" V 9105 3272 50  0000 R CNN
F 2 "" H 9100 3400 50  0001 C CNN
F 3 "" H 9100 3400 50  0001 C CNN
	1    9100 3400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 600B5680
P 9100 4250
F 0 "#PWR0103" H 9100 4000 50  0001 C CNN
F 1 "GND" V 9105 4122 50  0000 R CNN
F 2 "" H 9100 4250 50  0001 C CNN
F 3 "" H 9100 4250 50  0001 C CNN
	1    9100 4250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 600B56FD
P 9100 5100
F 0 "#PWR0104" H 9100 4850 50  0001 C CNN
F 1 "GND" V 9105 4972 50  0000 R CNN
F 2 "" H 9100 5100 50  0001 C CNN
F 3 "" H 9100 5100 50  0001 C CNN
	1    9100 5100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9000 2500 9000 2600
Wire Wire Line
	9000 2600 9100 2600
Wire Wire Line
	9000 3300 9000 3400
Wire Wire Line
	9000 3400 9100 3400
Wire Wire Line
	9000 4150 9000 4250
Wire Wire Line
	9000 4250 9100 4250
Wire Wire Line
	9000 5000 9000 5100
Wire Wire Line
	9000 5100 9100 5100
$Comp
L Device:R_US R4A
U 1 1 600B72D2
P 8650 2600
F 0 "R4A" V 8445 2600 50  0000 C CNN
F 1 "R_US" V 8536 2600 50  0000 C CNN
F 2 "" V 8690 2590 50  0001 C CNN
F 3 "~" H 8650 2600 50  0001 C CNN
	1    8650 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R1A
U 1 1 600B739C
P 8650 3400
F 0 "R1A" V 8445 3400 50  0000 C CNN
F 1 "R_US" V 8536 3400 50  0000 C CNN
F 2 "" V 8690 3390 50  0001 C CNN
F 3 "~" H 8650 3400 50  0001 C CNN
	1    8650 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R2A
U 1 1 600B740D
P 8650 4250
F 0 "R2A" V 8445 4250 50  0000 C CNN
F 1 "R_US" V 8536 4250 50  0000 C CNN
F 2 "" V 8690 4240 50  0001 C CNN
F 3 "~" H 8650 4250 50  0001 C CNN
	1    8650 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R3A
U 1 1 600B74CC
P 8650 5100
F 0 "R3A" V 8445 5100 50  0000 C CNN
F 1 "R_US" V 8536 5100 50  0000 C CNN
F 2 "" V 8690 5090 50  0001 C CNN
F 3 "~" H 8650 5100 50  0001 C CNN
	1    8650 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	8500 2600 8500 2300
Connection ~ 8500 2300
Wire Wire Line
	8500 2300 8700 2300
Wire Wire Line
	8800 2600 9000 2600
Connection ~ 9000 2600
$Comp
L Device:R_US R4B
U 1 1 600B7A42
P 8650 2900
F 0 "R4B" V 8855 2900 50  0000 C CNN
F 1 "R_US" V 8764 2900 50  0000 C CNN
F 2 "" V 8690 2890 50  0001 C CNN
F 3 "~" H 8650 2900 50  0001 C CNN
	1    8650 2900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R1B
U 1 1 600B7AF0
P 8650 3700
F 0 "R1B" V 8445 3700 50  0000 C CNN
F 1 "R_US" V 8536 3700 50  0000 C CNN
F 2 "" V 8690 3690 50  0001 C CNN
F 3 "~" H 8650 3700 50  0001 C CNN
	1    8650 3700
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R2B
U 1 1 600B7B6F
P 8650 4550
F 0 "R2B" V 8445 4550 50  0000 C CNN
F 1 "R_US" V 8536 4550 50  0000 C CNN
F 2 "" V 8690 4540 50  0001 C CNN
F 3 "~" H 8650 4550 50  0001 C CNN
	1    8650 4550
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R3B
U 1 1 600B7BF8
P 8650 5400
F 0 "R3B" V 8445 5400 50  0000 C CNN
F 1 "R_US" V 8536 5400 50  0000 C CNN
F 2 "" V 8690 5390 50  0001 C CNN
F 3 "~" H 8650 5400 50  0001 C CNN
	1    8650 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	8800 2900 8800 2600
Connection ~ 8800 2600
Wire Wire Line
	8800 3400 9000 3400
Connection ~ 9000 3400
Wire Wire Line
	8800 3400 8800 3700
Connection ~ 8800 3400
Wire Wire Line
	8800 4250 9000 4250
Connection ~ 9000 4250
Wire Wire Line
	8800 4250 8800 4550
Connection ~ 8800 4250
Wire Wire Line
	8800 5100 9000 5100
Connection ~ 9000 5100
Wire Wire Line
	8800 5100 8800 5400
Connection ~ 8800 5100
$Comp
L 4xxx:4016 U2
U 2 1 600B4D1D
P 8100 4250
F 0 "U2" V 8146 4123 50  0000 R CNN
F 1 "4016" V 8055 4123 50  0000 R CNN
F 2 "" H 8100 4250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4016b.pdf" H 8100 4250 50  0001 C CNN
	2    8100 4250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8500 2900 8100 2900
Wire Wire Line
	8100 3700 8500 3700
Wire Wire Line
	8100 4550 8500 4550
Wire Wire Line
	8100 5400 8500 5400
Wire Wire Line
	8500 5100 8500 4800
Connection ~ 8500 4800
Wire Wire Line
	8500 4800 8700 4800
Wire Wire Line
	8500 4250 8500 3950
Connection ~ 8500 3950
Wire Wire Line
	8500 3950 8700 3950
Wire Wire Line
	8500 3400 8500 3100
Connection ~ 8500 3100
Wire Wire Line
	8500 3100 8700 3100
$Comp
L power:+5V #PWR0105
U 1 1 600BC21A
P 8000 2900
F 0 "#PWR0105" H 8000 2750 50  0001 C CNN
F 1 "+5V" V 8015 3028 50  0000 L CNN
F 2 "" H 8000 2900 50  0001 C CNN
F 3 "" H 8000 2900 50  0001 C CNN
	1    8000 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 600BC2AF
P 8000 3700
F 0 "#PWR0106" H 8000 3550 50  0001 C CNN
F 1 "+5V" V 8015 3828 50  0000 L CNN
F 2 "" H 8000 3700 50  0001 C CNN
F 3 "" H 8000 3700 50  0001 C CNN
	1    8000 3700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0107
U 1 1 600BC344
P 8000 4550
F 0 "#PWR0107" H 8000 4400 50  0001 C CNN
F 1 "+5V" V 8015 4678 50  0000 L CNN
F 2 "" H 8000 4550 50  0001 C CNN
F 3 "" H 8000 4550 50  0001 C CNN
	1    8000 4550
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0108
U 1 1 600BCB54
P 8000 5400
F 0 "#PWR0108" H 8000 5250 50  0001 C CNN
F 1 "+5V" V 8015 5528 50  0000 L CNN
F 2 "" H 8000 5400 50  0001 C CNN
F 3 "" H 8000 5400 50  0001 C CNN
	1    8000 5400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8000 2900 8100 2900
Connection ~ 8100 2900
Wire Wire Line
	8100 3700 8000 3700
Connection ~ 8100 3700
Wire Wire Line
	8100 4550 8000 4550
Connection ~ 8100 4550
Wire Wire Line
	8100 5400 8000 5400
Connection ~ 8100 5400
$Comp
L power:+5V #PWR0109
U 1 1 600B5E6B
P 1350 6250
F 0 "#PWR0109" H 1350 6100 50  0001 C CNN
F 1 "+5V" H 1365 6423 50  0000 C CNN
F 2 "" H 1350 6250 50  0001 C CNN
F 3 "" H 1350 6250 50  0001 C CNN
	1    1350 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 600B5EF4
P 1350 7250
F 0 "#PWR0110" H 1350 7000 50  0001 C CNN
F 1 "GND" H 1355 7077 50  0000 C CNN
F 2 "" H 1350 7250 50  0001 C CNN
F 3 "" H 1350 7250 50  0001 C CNN
	1    1350 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 600B5FE5
P 6100 4650
F 0 "#PWR0111" H 6100 4400 50  0001 C CNN
F 1 "GND" H 6105 4477 50  0000 C CNN
F 2 "" H 6100 4650 50  0001 C CNN
F 3 "" H 6100 4650 50  0001 C CNN
	1    6100 4650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0112
U 1 1 600B6098
P 6100 2850
F 0 "#PWR0112" H 6100 2700 50  0001 C CNN
F 1 "+3.3V" H 6115 3023 50  0000 C CNN
F 2 "" H 6100 2850 50  0001 C CNN
F 3 "" H 6100 2850 50  0001 C CNN
	1    6100 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3150 6100 2850
Entry Wire Line
	7000 3850 7100 3750
Entry Wire Line
	6900 3550 7000 3450
Entry Wire Line
	6900 3350 7000 3250
Text Label 6800 3850 0    50   ~ 0
SPI1
Entry Wire Line
	6900 1450 7000 1550
Entry Wire Line
	6900 1550 7000 1650
$Comp
L power:+12V #PWR0115
U 1 1 600BF3DD
P 7750 1150
F 0 "#PWR0115" H 7750 1000 50  0001 C CNN
F 1 "+12V" H 7765 1323 50  0000 C CNN
F 2 "" H 7750 1150 50  0001 C CNN
F 3 "" H 7750 1150 50  0001 C CNN
	1    7750 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1250 7750 1150
Wire Wire Line
	7750 1850 7750 1950
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 600B6CB3
P 6200 1550
F 0 "J2" H 6120 1225 50  0000 C CNN
F 1 "Conn_01x02" H 6120 1316 50  0000 C CNN
F 2 "" H 6200 1550 50  0001 C CNN
F 3 "~" H 6200 1550 50  0001 C CNN
	1    6200 1550
	-1   0    0    1   
$EndComp
Wire Wire Line
	6700 3850 7000 3850
Wire Wire Line
	6700 3350 6900 3350
Wire Wire Line
	6700 3550 6900 3550
Wire Wire Line
	6400 1450 6900 1450
Wire Wire Line
	6400 1550 6900 1550
Text Label 6700 3350 0    50   ~ 0
ON_BOOT
Text Label 6700 3550 0    50   ~ 0
GOOD
Text Label 6400 1550 0    50   ~ 0
ON_BOOT
Text Label 6400 1450 0    50   ~ 0
GOOD
$Comp
L Regulator_Linear:AP1117-33 U?
U 1 1 600CBACE
P 2200 6850
F 0 "U?" H 2200 7092 50  0000 C CNN
F 1 "AP1117-33" H 2200 7001 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2200 7050 50  0001 C CNN
F 3 "http://www.diodes.com/datasheets/AP1117.pdf" H 2300 6600 50  0001 C CNN
	1    2200 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 7250 2200 7250
Wire Wire Line
	2200 7250 2200 7150
Connection ~ 1350 7250
Wire Wire Line
	1350 6250 1900 6250
Wire Wire Line
	1900 6250 1900 6850
Connection ~ 1350 6250
Wire Wire Line
	2500 6850 2550 6850
Wire Wire Line
	2550 6850 2550 6250
$Comp
L power:+3.3V #PWR0119
U 1 1 600DA7D0
P 2550 6250
F 0 "#PWR0119" H 2550 6100 50  0001 C CNN
F 1 "+3.3V" H 2565 6423 50  0000 C CNN
F 2 "" H 2550 6250 50  0001 C CNN
F 3 "" H 2550 6250 50  0001 C CNN
	1    2550 6250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 JS4
U 1 1 600E5393
P 9900 2000
F 0 "JS4" H 9980 1992 50  0000 L CNN
F 1 "Conn_01x02" H 9980 1901 50  0000 L CNN
F 2 "" H 9900 2000 50  0001 C CNN
F 3 "~" H 9900 2000 50  0001 C CNN
	1    9900 2000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 JS1
U 1 1 600E5491
P 9900 2800
F 0 "JS1" H 9980 2792 50  0000 L CNN
F 1 "Conn_01x02" H 9980 2701 50  0000 L CNN
F 2 "" H 9900 2800 50  0001 C CNN
F 3 "~" H 9900 2800 50  0001 C CNN
	1    9900 2800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 JS2
U 1 1 600E6E73
P 9900 3650
F 0 "JS2" H 9980 3642 50  0000 L CNN
F 1 "Conn_01x02" H 9980 3551 50  0000 L CNN
F 2 "" H 9900 3650 50  0001 C CNN
F 3 "~" H 9900 3650 50  0001 C CNN
	1    9900 3650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 JS3
U 1 1 600E6EC4
P 9900 4500
F 0 "JS3" H 9980 4492 50  0000 L CNN
F 1 "Conn_01x02" H 9980 4401 50  0000 L CNN
F 2 "" H 9900 4500 50  0001 C CNN
F 3 "~" H 9900 4500 50  0001 C CNN
	1    9900 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2100 9700 2100
Wire Wire Line
	9700 2900 9000 2900
Wire Wire Line
	9700 3750 9000 3750
Wire Wire Line
	9700 4600 9000 4600
$Comp
L power:+12V #PWR?
U 1 1 600EDC5B
P 9700 1850
F 0 "#PWR?" H 9700 1700 50  0001 C CNN
F 1 "+12V" H 9715 2023 50  0000 C CNN
F 2 "" H 9700 1850 50  0001 C CNN
F 3 "" H 9700 1850 50  0001 C CNN
	1    9700 1850
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 600EDC9F
P 9700 2650
F 0 "#PWR?" H 9700 2500 50  0001 C CNN
F 1 "+12V" H 9715 2823 50  0000 C CNN
F 2 "" H 9700 2650 50  0001 C CNN
F 3 "" H 9700 2650 50  0001 C CNN
	1    9700 2650
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 600EDCDC
P 9700 3550
F 0 "#PWR?" H 9700 3400 50  0001 C CNN
F 1 "+12V" H 9715 3723 50  0000 C CNN
F 2 "" H 9700 3550 50  0001 C CNN
F 3 "" H 9700 3550 50  0001 C CNN
	1    9700 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 600EDD19
P 9700 4350
F 0 "#PWR?" H 9700 4200 50  0001 C CNN
F 1 "+12V" H 9715 4523 50  0000 C CNN
F 2 "" H 9700 4350 50  0001 C CNN
F 3 "" H 9700 4350 50  0001 C CNN
	1    9700 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 1850 9700 2000
Wire Wire Line
	9700 2650 9700 2800
Wire Wire Line
	9700 3550 9700 3650
Wire Wire Line
	9700 4350 9700 4500
Entry Wire Line
	6900 3650 7000 3550
Entry Wire Line
	6900 3450 7000 3350
Wire Wire Line
	6700 3450 6900 3450
Wire Wire Line
	6900 3650 6700 3650
Text Label 6700 3450 0    50   ~ 0
TXD
Text Label 6700 3650 0    50   ~ 0
RXD
$Comp
L Interface_USB:FT232RL U?
U 1 1 601037C5
P 2850 3400
F 0 "U?" H 2850 4578 50  0000 C CNN
F 1 "FT232RL" H 2850 4487 50  0000 C CNN
F 2 "Package_SO:SSOP-28_5.3x10.2mm_P0.65mm" H 2850 3400 50  0001 C CNN
F 3 "http://www.ftdichip.com/Products/ICs/FT232RL.htm" H 2850 3400 50  0001 C CNN
	1    2850 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 2700 2050 2400
Wire Wire Line
	2050 2400 2750 2400
$Comp
L Device:C C1
U 1 1 6010A0E6
P 2050 2250
F 0 "C1" H 2165 2296 50  0000 L CNN
F 1 "100nF" H 2165 2205 50  0000 L CNN
F 2 "" H 2088 2100 50  0001 C CNN
F 3 "~" H 2050 2250 50  0001 C CNN
	1    2050 2250
	1    0    0    -1  
$EndComp
Connection ~ 2050 2400
$Comp
L power:GND #PWR?
U 1 1 6010A2E7
P 2050 2100
F 0 "#PWR?" H 2050 1850 50  0001 C CNN
F 1 "GND" H 2055 1927 50  0000 C CNN
F 2 "" H 2050 2100 50  0001 C CNN
F 3 "" H 2050 2100 50  0001 C CNN
	1    2050 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	3650 3100 4100 3100
Text Label 3700 3100 0    50   ~ 0
ON_BOOT
$Comp
L Device:R_US R3
U 1 1 6010CC8A
P 6900 1300
F 0 "R3" H 6968 1346 50  0000 L CNN
F 1 "10K" H 6968 1255 50  0000 L CNN
F 2 "" V 6940 1290 50  0001 C CNN
F 3 "~" H 6900 1300 50  0001 C CNN
	1    6900 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6010CE75
P 6900 1150
F 0 "#PWR?" H 6900 1000 50  0001 C CNN
F 1 "+3.3V" H 6915 1323 50  0000 C CNN
F 2 "" H 6900 1150 50  0001 C CNN
F 3 "" H 6900 1150 50  0001 C CNN
	1    6900 1150
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Mini J?
U 1 1 60111D8F
P 1050 3000
F 0 "J?" H 1105 3467 50  0000 C CNN
F 1 "USB_B_Mini" H 1105 3376 50  0000 C CNN
F 2 "" H 1200 2950 50  0001 C CNN
F 3 "~" H 1200 2950 50  0001 C CNN
	1    1050 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 3000 2050 3000
Wire Wire Line
	2050 3100 1350 3100
$Comp
L power:+5V #PWR?
U 1 1 6011E408
P 1350 2100
F 0 "#PWR?" H 1350 1950 50  0001 C CNN
F 1 "+5V" H 1365 2273 50  0000 C CNN
F 2 "" H 1350 2100 50  0001 C CNN
F 3 "" H 1350 2100 50  0001 C CNN
	1    1350 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2100 1350 2800
$Comp
L power:GND #PWR?
U 1 1 60120E73
P 1050 3500
F 0 "#PWR?" H 1050 3250 50  0001 C CNN
F 1 "GND" H 1055 3327 50  0000 C CNN
F 2 "" H 1050 3500 50  0001 C CNN
F 3 "" H 1050 3500 50  0001 C CNN
	1    1050 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3400 950  3450
Wire Wire Line
	950  3450 1050 3450
Wire Wire Line
	1050 3450 1050 3400
Connection ~ 1050 3450
Wire Wire Line
	1050 3450 1050 3500
NoConn ~ 1350 3200
NoConn ~ 2050 3400
NoConn ~ 2050 4100
NoConn ~ 3650 4100
NoConn ~ 3650 4000
NoConn ~ 3650 3900
NoConn ~ 3650 3800
NoConn ~ 3650 3700
NoConn ~ 3650 3400
NoConn ~ 3650 3300
NoConn ~ 3650 3200
NoConn ~ 3650 3000
$Comp
L power:+5V #PWR?
U 1 1 60153F97
P 2950 2100
F 0 "#PWR?" H 2950 1950 50  0001 C CNN
F 1 "+5V" H 2965 2273 50  0000 C CNN
F 2 "" H 2950 2100 50  0001 C CNN
F 3 "" H 2950 2100 50  0001 C CNN
	1    2950 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 2400 2950 2100
Wire Notes Line
	700  900  4300 900 
Wire Notes Line
	4300 900  4300 5000
Wire Notes Line
	4300 5000 700  5000
Wire Notes Line
	700  5000 700  900 
Wire Wire Line
	5500 2900 3650 2900
Wire Wire Line
	5500 2900 5500 3350
Entry Wire Line
	6900 2300 7000 2200
Entry Wire Line
	6900 2150 7000 2050
Wire Wire Line
	6900 2150 3900 2150
Wire Wire Line
	3900 2150 3900 2700
Wire Wire Line
	3900 2700 3650 2700
Wire Wire Line
	6900 2300 4050 2300
Wire Wire Line
	4050 2300 4050 2800
Wire Wire Line
	4050 2800 3650 2800
Text Label 6750 2150 0    50   ~ 0
TXD
Text Label 6750 2300 0    50   ~ 0
RXD
Wire Notes Line
	10500 900  10500 5600
Wire Notes Line
	10500 5600 5200 5600
Wire Notes Line
	5200 5600 5200 900 
Wire Notes Line
	5100 900  10400 900 
Text Notes 7200 850  0    100  ~ 0
Logic + Power Control
Text Notes 1150 850  0    100  ~ 0
In-Circuit USB Serial Programmer\n
Wire Notes Line
	650  5500 4300 5500
Wire Notes Line
	4300 5500 4300 7700
Wire Notes Line
	4300 7700 650  7700
Wire Notes Line
	650  7700 650  5500
Text Notes 1750 5450 0    100  ~ 0
Power Regulation
$Comp
L power:GND #PWR?
U 1 1 60180768
P 2950 4500
F 0 "#PWR?" H 2950 4250 50  0001 C CNN
F 1 "GND" H 2955 4327 50  0000 C CNN
F 2 "" H 2950 4500 50  0001 C CNN
F 3 "" H 2950 4500 50  0001 C CNN
	1    2950 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 4400 3050 4450
Wire Wire Line
	3050 4450 2950 4450
Wire Wire Line
	2650 4450 2650 4400
Wire Wire Line
	2850 4400 2850 4450
Connection ~ 2850 4450
Wire Wire Line
	2850 4450 2650 4450
Wire Wire Line
	2950 4400 2950 4450
Connection ~ 2950 4450
Wire Wire Line
	2950 4450 2850 4450
Wire Wire Line
	2950 4450 2950 4500
NoConn ~ 5500 4450
NoConn ~ 5500 4350
NoConn ~ 5500 4250
NoConn ~ 5500 4150
NoConn ~ 5500 4050
NoConn ~ 5500 3950
NoConn ~ 5500 3750
NoConn ~ 5500 3550
NoConn ~ 6700 4250
NoConn ~ 6700 3750
$Comp
L Connector:DIN-3 J1
U 1 1 601C46E8
P 7750 1550
F 0 "J1" H 7750 1276 50  0000 C CNN
F 1 "DIN-3" H 7750 1185 50  0000 C CNN
F 2 "" H 7750 1550 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 7750 1550 50  0001 C CNN
	1    7750 1550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6020A02E
P 7750 1950
F 0 "#PWR?" H 7750 1700 50  0001 C CNN
F 1 "GND" H 7755 1777 50  0000 C CNN
F 2 "" H 7750 1950 50  0001 C CNN
F 3 "" H 7750 1950 50  0001 C CNN
	1    7750 1950
	1    0    0    -1  
$EndComp
Entry Wire Line
	7100 1650 7200 1550
Wire Wire Line
	7200 1550 7450 1550
Wire Bus Line
	7100 1550 7100 3750
Wire Bus Line
	7300 2500 7300 5000
Wire Bus Line
	7000 1550 7000 3550
Text Label 7250 1550 0    50   ~ 0
SPI1
$EndSCHEMATC
