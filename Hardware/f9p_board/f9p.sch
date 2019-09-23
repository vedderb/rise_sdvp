EESchema Schematic File Version 5
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
$Comp
L f9p:Ublox_ZED_F9P U1
U 1 1 5D82C8DE
P 3975 3350
F 0 "U1" H 3950 4975 50  0000 C CNN
F 1 "Ublox_ZED_F9P" H 3950 4884 50  0000 C CNN
F 2 "f9p:UBLOX_ZED_F9P" H 3975 3350 50  0001 C CNN
F 3 "" H 3975 3350 50  0001 C CNN
	1    3975 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5D82E054
P 1125 2525
F 0 "#PWR0101" H 1125 2275 50  0001 C CNN
F 1 "GND" H 1130 2352 50  0000 C CNN
F 2 "" H 1125 2525 50  0001 C CNN
F 3 "" H 1125 2525 50  0001 C CNN
	1    1125 2525
	1    0    0    -1  
$EndComp
Text GLabel 1125 2525 1    50   Input ~ 0
GND
Text GLabel 2775 1950 0    50   Input ~ 0
GND
Text GLabel 2775 2150 0    50   Input ~ 0
GND
Text GLabel 2775 3250 0    50   Input ~ 0
GND
Text GLabel 5125 2350 2    50   Input ~ 0
GND
Text GLabel 5125 3250 2    50   Input ~ 0
GND
Text GLabel 5125 4050 2    50   Input ~ 0
GND
NoConn ~ 5125 1950
NoConn ~ 5125 2050
NoConn ~ 5125 2150
NoConn ~ 5125 2250
NoConn ~ 2775 2650
NoConn ~ 2775 2750
NoConn ~ 2775 2850
NoConn ~ 2775 2950
NoConn ~ 2775 3150
NoConn ~ 2775 3450
NoConn ~ 2775 3550
NoConn ~ 2775 3650
NoConn ~ 2775 3750
NoConn ~ 2775 4050
NoConn ~ 2775 4150
NoConn ~ 2775 4250
NoConn ~ 2775 4350
NoConn ~ 2775 4450
NoConn ~ 5125 4450
NoConn ~ 5125 4650
NoConn ~ 5950 5000
Text GLabel 3875 5050 3    50   Input ~ 0
GND
$Comp
L Device:R_Small R4
U 1 1 5D84F531
P 6300 3050
F 0 "R4" V 6104 3050 50  0000 C CNN
F 1 "27 Ohm" V 6195 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6300 3050 50  0001 C CNN
F 3 "~" H 6300 3050 50  0001 C CNN
	1    6300 3050
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5D84FC6B
P 6700 3150
F 0 "R5" V 6504 3150 50  0000 C CNN
F 1 "27 Ohm" V 6595 3150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6700 3150 50  0001 C CNN
F 3 "~" H 6700 3150 50  0001 C CNN
	1    6700 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 3050 5125 3050
Wire Wire Line
	6600 3150 5125 3150
Text GLabel 7275 3150 2    50   Input ~ 0
F9P_USB_DP
Text GLabel 7275 3050 2    50   Input ~ 0
F9P_USB_DM
Wire Wire Line
	6800 3150 7275 3150
Wire Wire Line
	6400 3050 7275 3050
$Comp
L Device:R_Small R3
U 1 1 5D863978
P 5750 2825
F 0 "R3" H 5808 2779 50  0000 L CNN
F 1 "100K" H 5808 2870 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5750 2825 50  0001 C CNN
F 3 "~" H 5750 2825 50  0001 C CNN
	1    5750 2825
	-1   0    0    1   
$EndComp
Wire Wire Line
	5125 2950 5750 2950
Wire Wire Line
	5750 2950 5750 2925
Wire Wire Line
	5750 2950 6150 2950
Wire Wire Line
	6150 2950 6150 2550
Connection ~ 5750 2950
Text GLabel 5125 2850 2    50   Input ~ 0
GND
Text GLabel 5750 2725 1    50   Input ~ 0
GND
$Comp
L Connector:Conn_01x01_Female J5
U 1 1 5DA49920
P 7010 4015
F 0 "J5" H 7038 4041 50  0000 L CNN
F 1 "Active Antenna" H 7038 3950 50  0000 L CNN
F 2 "Connector_Coaxial:U.FL_Molex_MCRF_73412-0110_Vertical" H 7010 4015 50  0001 C CNN
F 3 "~" H 7010 4015 50  0001 C CNN
	1    7010 4015
	1    0    0    -1  
$EndComp
Text GLabel 6150 2550 2    50   Input ~ 0
F9P_3v3
$EndSCHEMATC
