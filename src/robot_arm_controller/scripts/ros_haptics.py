#!/usr/bin/env python3

import asyncio
from bleak import BleakClient
from Haptics import Haptics
import time
import rospy
from std_msgs.msg import Float32


deviceAddress = "48:e7:29:08:3c:8e"  # Replace with your BLE device's address
characteristicUUID = "0000ff01-0000-1000-8000-00805f9b34fb"  # Replace with the characteristic UUID for this action
haptics = Haptics(whichHand="Right")
target_pressure = 100 
compensate_hysteresis = True
apply_haptics = False
prev_apply_haptics = False

client_trigger = None


current_task = None

def haptic_callback(msg):
    global prev_apply_haptics, apply_haptics, current_task

    gripper_torque = msg.data
    print(f"Received gripper torque: {gripper_torque}")
    prev_apply_haptics = apply_haptics
    if gripper_torque >= 5:
        apply_haptics = True
    else:
        apply_haptics = False
    if apply_haptics != prev_apply_haptics:
        if current_task is None or current_task.done():
            current_task = asyncio.run(trigger_haptics(apply_haptics, haptics, target_pressure, compensate_hysteresis))

async def connect_haptics():
    airPresSourceCtrlStarted = False  # Initially, the pump has not been yet started.
    sourcePres = 255

    # This initiates the pumping of the air into the reservoir.
    async with BleakClient(deviceAddress) as client:
        print(f"Connected to {deviceAddress}")
        reservoirData = haptics.air_pressure_source_control(airPresSourceCtrlStarted, sourcePres)
        await client.write_gatt_char(characteristicUUID, reservoirData)

async def initialize_haptics():
    global client_trigger
    client_trigger = BleakClient(deviceAddress)
    await client_trigger.connect()
    print(f"Connected to {deviceAddress}")

async def trigger_haptics(apply_haptics, haptics, target_pressure, compensate_hysteresis):
    global client_trigger
    print(f"started at {time.strftime('%X')}")
    clutch_state = haptics.set_clutch_state_single("Palm", apply_haptics)
    haptics_data = haptics.apply_haptics(clutch_state, target_pressure, compensate_hysteresis)
    # Now we send the haptics data to trigger haptics onto the index finger to the device.
    print(f"Applying haptics to Index Finger: {haptics_data}")

    await client_trigger.write_gatt_char(characteristicUUID, bytearray(haptics_data))
    print(f"finished at {time.strftime('%X')}")
    # To disable the haptics, modify the apply_haptics in the clutch state to false and call the
    # apply haptics function again with the modified clutch state.

# Run the BLE connection and control logic
if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(connect_haptics())
    loop.run_until_complete(initialize_haptics())
    rospy.init_node('haptic_controller', anonymous=True)
    rospy.Subscriber("haptic/palm", Float32, haptic_callback)
    rospy.spin()
    

