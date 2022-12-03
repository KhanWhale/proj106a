#!/usr/bin/env python3

# Send req to controller-manager-startup with startupCheck.srv
# Listen on topic handState for changes in input
# multithreaded
# Send final command on t:opic droneCommand


if __name__ == "__main__":
    print("controller")
