# Do It Yourself, Electric Vehicle, Vehicle Control Unit
A DIY VCU project that uses ESP32 to control the engine and other systems. The Engine and Inverter chosen for this project is from Nissan Leaf 2015 (Gen 2). Charger is from Mistubishi Outlander 2017. This repository is not ment for public contribution and it is not made for the ease of re-use for other EV projects. Feel free to look around and use any useful code or info.
 
# Requirements
* ESP32
* autowp-mcp2515 library

# Capabilities
* Communication with the Nissan Leaf 2015 Inverter.
* Communication and charging with the Mitsubishi Outlander 2017 Charger.
* Capable of using the Nissan Leaf Throttle. (Or any analog signal, or multiple in conjunction)
* Motor Regen.
* Vacuum control.
* Pre-charge control.

# Useful links
* Some Nissan Leaf CAN code [Open-Vehicle-Monitoring-System](https://github.com/openvehicles/Open-Vehicle-Monitoring-System-3/blob/master/vehicle/OVMS.V3/components/vehicle_nissanleaf/src/vehicle_nissanleaf.cpp)
* Nissan Leaf VCU PCB and code [Nissan-Leaf-Inverter-Controller](https://github.com/damienmaguire/Nissan-Leaf-Inverter-Controller/tree/master)
* Nissan Leaf BMS and chademo code [stm32-car](https://github.com/jsphuebner/stm32-car/tree/master/src)
* Nissan Leaf Gen1/2/3 inverter via CAN, Gen2 PDM (Charger and DCDC), Battery (all variants) [ZombieVerter VCU](https://github.com/damienmaguire/Stm32-vcu)
* Mitsubishi Outlander [info](https://openinverter.org/wiki/Mitsubishi_Outlander_DCDC_OBC)
