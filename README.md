# Traffic light finite state machine
This project is my finished work for lab 10 of Embedded Systems - Shape The World: Microcontroller Input/Output course. 
Coded in C 

## Hardware
This project interfaces the TM4C123 microcontroller, switches for car sensors and pedestrian sensor, LEDs for output.

## Description 
This project implements a simple intersection control system for a 4-corner intersection. 
The system utilizes a microcontroller, specifically the LaunchPad, along with various sensors and LEDs to regulate traffic and ensure pedestrian safety.

## System Overview
The intersection consists of two one-way streets, namely the South road (cars travel South) and the West road (cars travel West). 
The LaunchPad receives three inputs: two car sensors and one pedestrian sensor.
- The South car sensor is true (3.3V) when one or more cars are near the intersection on the South road.
- The West car sensor is true (3.3V) when one or more cars are near the intersection on the West road.
- The Walk sensor is true (3.3V) when a pedestrian is present and wishes to cross in any direction.

## LED Interface
The system interfaces with six LEDs to represent the traffic lights and pedestrian signals:
- Two Red-Yellow-Green LEDs for the traffic lights.
- PF3 green LED for the "walk" light.
- PF1 red LED for the "don't walk" light.

## Logic
Traffic should not be allowed to crash. I.e., there should not be a green or yellow on one road at the same time there is a green or  yellow LED on the other road. In other words, while traffic is flowing in one direction, there should be a red light in the other direction.
