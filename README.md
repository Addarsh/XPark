# XPark
A Low cost Wireless Sensor Network for Automated Parking

# Description

Parking is a known problem in major urban areas in the world. It is both time-consuming and stressful to find a good parking location close to the desired destination. Often times, drivers have no assistance in finding a good parking spot and therefore waste time and fuel driving around to find an open parking spot. With the population of urban cities growing dramatically and the number of parking vacancies not keeping up, finding parking will only become a bigger problem ultimately leading to traffic congestion and inefficiency.

The goal of this project is to utilize low cost sensors to demostrate an automated web-based solution to parking.

Each parking lot is monitored by one battery powered sensor that is used to detect the occupancy of the spot at all times.
Once a car enters/exits the spot, the sensor detects this change and relays the data to a neighboring Local gateway node. The Local gateway node then relays this to a remote Gateway which then ultimately sends this to a web-server. For more information, check the documentation here: https://addarsh.github.io/projects/.

I used TI's CC26640R2F Launchpad for development and modified their example project. Most of the changes can be found within the ble5_multi_role_cc2640r2lp_app/Application/multi_role.c and ble5_multi_role_cc2640r2lp_app/PROFILES/simple_gatt_profile.c.

 
