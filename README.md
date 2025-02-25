# Nissan_Leaf_VCU_Arduino
A Nissan Leaf VCU project that uses Arduino Due to control the engine and other systems. Engine, Inverter and Power Delivery Module chosen for this project is from 2015 (Gen 2). This repository is not ment for public contribution but rather to help others to implement an affordable VCU solution as I develop this for my own use. Why do they cost so much anyways?
 
# Requirements
* Arduino Due (Uses ~58416 bytes of ROM, 7 digital pins, SPI, 2 Analog to digital pins)
* autowp-mcp2515 library

# Capabilities
* Communication with the Inverter and the PDM was successful.
* Spun the motor with and without the Nissan Leaf throttle pedal.
* The 12V charging is giving me problems though :/

# Useful links
* Some Nissan Leaf CAN code [Open-Vehicle-Monitoring-System](https://github.com/openvehicles/Open-Vehicle-Monitoring-System-3/blob/master/vehicle/OVMS.V3/components/vehicle_nissanleaf/src/vehicle_nissanleaf.cpp)
* Nissan Leaf VCU PCB and code [Nissan-Leaf-Inverter-Controller](https://github.com/damienmaguire/Nissan-Leaf-Inverter-Controller/tree/master)
* Nissan Leaf BMS and chademo code [stm32-car](https://github.com/jsphuebner/stm32-car/tree/master/src)
* Nissan Leaf Gen1/2/3 inverter via CAN, Gen2 PDM (Charger and DCDC), Battery (all variants) [ZombieVerter VCU](https://github.com/damienmaguire/Stm32-vcu)
