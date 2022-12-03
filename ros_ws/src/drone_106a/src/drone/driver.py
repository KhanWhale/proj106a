#!/usr/bin/env python3

# Send req to driver-manager-startup with startupCheck.srv
# Handles e-stop via keyboard, drone errors, and regular commands
# Set timer v. last command and send "battery?" command if timer goes off
# Receives commands on topic droneCommand
# Send error responses on topic droneResp => messagetype String


if __name__ == "__main__":
    print("driver")
