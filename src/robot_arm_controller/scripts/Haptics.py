#!/usr/bin/env python3

class Haptics:
    def __init__(self, whichHand):
        self.whichHand = whichHand
        self.buffer = bytearray(1024)
        self.oneFrame = bytearray(128)
        self.fingerPositionData = [0] * 5
        self.hapticStartPosition = [0.0] * 5
        self.pressureData = [0] * 7
        self.flag_MicrotubeDataReady = False
        self.flag_pressureDataReady = False

    @staticmethod
    def get_ghost_finger_name(buf):
        ghost_fingers = {
            0: "GhostThumb",
            1: "GhostIndex",
            2: "GhostMiddle",
            3: "GhostRing",
            4: "GhostPinky",
            5: "GhostPalm"
        }
        return ghost_fingers.get(buf, None)

    @staticmethod
    def set_clutch_state(buf_name, buf_state):
        num_array = [0xFF, 0xFF]

        finger_map = {
            "GhostThumb": 0,
            "GhostIndex": 1,
            "GhostMiddle": 2,
            "GhostRing": 3,
            "GhostPinky": 4,
            "GhostPalm": 5
        }

        state_map = {
            "Enter": 0,
            "Stay": 1,
            "Exit": 2
        }

        if buf_name in finger_map:
            num_array[0] = finger_map[buf_name]

        if buf_state in state_map:
            num_array[1] = state_map[buf_state]

        return num_array

    @staticmethod
    def is_hand_valid(whichHand):
        return whichHand in ["Left", "Right"]

    def set_clutch_state_single(self, finger, apply_haptics):
        num_array = [0xFF, 0xFF]
        finger_map = {
            "Thumb": 0,
            "Index": 1,
            "Middle": 2,
            "Ring": 3,
            "Pinky": 4,
            "Palm": 5
        }

        if finger in finger_map:
            num_array[0] = finger_map[finger]

        num_array[1] = 0 if apply_haptics else 2

        if num_array[0] != 0xFF and num_array[1] != 0xFF:
            return num_array
        else:
            print("Invalid parameter")
            return None

    def set_clutch_state_multiple(self, fingers, apply_haptics):
        num_array = []
        state = 0 if apply_haptics else 2

        if state == 0xFF:
            print("Invalid parameter")
            return None

        finger_map = {
            "Thumb": 0,
            "Index": 1,
            "Middle": 2,
            "Ring": 3,
            "Pinky": 4,
            "Palm": 5
        }

        for finger in fingers:
            if finger in finger_map:
                num_array.append([finger_map[finger], state])
            else:
                print("Invalid parameter")
                return None

        return num_array

    # def apply_haptics(self, clutch_state, target_pres, compensate_hysteresis):
    #     if not self.is_hand_valid(self.whichHand):
    #         print(f"Invalid hand name: {self.whichHand}")
    #         return None
    #
    #     frequency = 0xFF if 0 < target_pres < 10 else 0
    #     pres_source = self.pressureData[5]
    #
    #     valve_timing = HaptGloveValvesCalibrationData.calculate_valve_timing(target_pres, clutch_state[0], pres_source,
    #                                                                          self.whichHand)
    #     if clutch_state[0] != 0xFF and clutch_state[1] != 0xFF:
    #         n1, n2 = valve_timing
    #         Encode.instance().add_u8(frequency)
    #         Encode.instance().add_u8(clutch_state[0])
    #         Encode.instance().add_u8(clutch_state[1])
    #         Encode.instance().add_u8(target_pres)
    #         Encode.instance().add_u8(n1)
    #         Encode.instance().add_u8(n2)
    #         Encode.instance().add_b1(compensate_hysteresis)
    #         data = Encode.instance().add_fun(0x03)
    #         Encode.instance().clear_list()
    #         print("Thumb data", data)
    #         return data
    #     else:
    #         return None
    @staticmethod
    def air_pressure_source_control(airPresSourceCtrlStarted, sourcePres):
        try:
            encode_instance = Encode()
            if not airPresSourceCtrlStarted:
                encode_instance.add_u8(1)  # Send start signal
                encode_instance.add_u8(sourcePres)
                data = encode_instance.add_fun(2)
                if len(data) > 0:
                    # Send the data via BLE
                    # await client.write_gatt_char(characteristic_uuid, data)
                    return data
                else:
                    print("Error: Data is empty after encoding")
                encode_instance.clear_list()

            else:
                # Stop air pressure control
                print("Stopping air pressure control...")
                encode_instance.add_u8(0)  # Send stop signal
                encode_instance.add_u8(0)
                data = encode_instance.add_fun(2)  # Add function ID
                if len(data) > 0:
                    # Send the data via BLE
                    # await client.write_gatt_char(characteristic_uuid, data)
                    return data
                else:
                    print("Error: Data is empty after encoding")
                # Clear the buffer after sending data
                encode_instance.clear_list()

        except Exception as e:
            print(f"Error: {e}")

    def apply_haptics(self, clutchState, targetPres, compensateHysteresis):
        if not Haptics.is_hand_valid(self.whichHand):
            print(f"Invalid hand name: {self.whichHand}")
            return None

        n1 = 0
        if 0 < targetPres < 10:
            n1 = 255  # Equivalent to byte.MaxValue in C#

        presSource = self.pressureData[5]
        valveTiming = HaptGloveValvesCalibrationData.calculate_valve_timing(targetPres, clutchState[0],
                                                                                    presSource, self.whichHand)

        if clutchState[0] == 255 or clutchState[1] == 255:  # Equivalent to checking for byte.MaxValue in C#
            return None

        n2 = valveTiming[0]
        n3 = valveTiming[1]

        # Using the Encode class (similar to the C# Encode.Instance)
        encode = Encode()
        encode.add_u8(n1)
        encode.add_u8(clutchState[0])
        encode.add_u8(clutchState[1])
        encode.add_u8(targetPres)
        encode.add_u8(n2)
        encode.add_u8(n3)
        encode.add_b1(compensateHysteresis)

        numArray = encode.add_fun(3)
        encode.clear_list()

        return numArray

    def decode_glove_data(self, glove_data):
        self.buffer.extend(glove_data)
        while len(self.buffer) >= 5:
            if self.buffer[1] in Haptics.fun_list:
                length = self.buffer[0]
                if len(self.buffer) < length:
                    break

                checksum = 0
                for i in range(length - 1):
                    checksum ^= self.buffer[i]

                if checksum != self.buffer[length - 1]:
                    self.buffer = self.buffer[length:]
                else:
                    self.oneFrame = self.buffer[:length]
                    self.buffer = self.buffer[length:]
                    self.frame_data_analysis(self.oneFrame)
            else:
                self.buffer = self.buffer[1:]

    def frame_data_analysis(self, frame):
        if frame[1] == 1:
            self.decode_pressure(frame)
        elif frame[1] in [4, 5]:
            self.decode_microtube(frame)

    def decode_pressure(self, frame):
        for i in range(7):
            self.pressureData[i] = int.from_bytes(frame[3 + i * 5:8 + i * 5], byteorder="little", signed=True)
        self.flag_pressureDataReady = True

    def decode_microtube(self, frame):
        for i in range(5):
            self.fingerPositionData[i] = int.from_bytes(frame[3 + i * 5:8 + i * 5], byteorder="little", signed=True)
        self.flag_MicrotubeDataReady = True


class Encode:
    _instance = None

    @staticmethod
    def instance():
        if Encode._instance is None:
            Encode._instance = Encode()
        return Encode._instance

    def __init__(self):
        self.list = []

    def clear_list(self):
        self.list.clear()

    def add_fun(self, n):
        length = len(self.list) + 2
        self.list.insert(0, n)
        self.list.insert(0, length)
        return bytearray(self.list)

    def add_u8(self, n):
        self.list.extend([0x01, n])

    def add_b1(self, n):
        self.list.extend([0x11, 0x01 if n else 0x00])


class HaptGloveValvesCalibrationData:
    valveCaliOn_Left = [
        [
            [13, 20, 27, 34, 44, 130],
            [12, 18, 25, 32, 42, 130],
            [12, 17, 23, 30, 39, 130],
            [12, 17, 23, 30, 39, 130],
            [13, 18, 24, 31, 40, 130],
            [13, 18, 24, 31, 40, 130]
        ],
        [
            [14, 21, 27, 35, 46, 200],
            [12, 19, 25, 33, 44, 200],
            [12, 17, 24, 31, 41, 200],
            [12, 17, 23, 30, 41, 200],
            [13, 18, 25, 32, 41, 200],
            [13, 18, 25, 32, 41, 200]
        ],
        [
            [14, 21, 28, 36, 48, 255],
            [13, 19, 26, 34, 46, 255],
            [12, 18, 24, 32, 42, 255],
            [12, 18, 24, 31, 42, 255],
            [13, 19, 25, 33, 43, 255],
            [13, 19, 25, 33, 43, 255]
        ],
        [
            [14, 21, 29, 37, 52, 255],
            [13, 20, 27, 36, 49, 255],
            [12, 18, 25, 33, 46, 255],
            [12, 18, 24, 33, 46, 255],
            [13, 19, 26, 35, 47, 255],
            [13, 19, 26, 35, 47, 255]
        ],
        [
            [14, 22, 30, 39, 56, 255],
            [13, 20, 28, 37, 54, 255],
            [12, 19, 25, 34, 54, 255],
            [12, 19, 25, 34, 54, 255],
            [13, 20, 27, 36, 54, 255],
            [13, 20, 27, 36, 54, 255]
        ]
    ]

    valveCaliOff_Left = [
        [100, 130, 160, 190, 210, 230],
        [100, 130, 160, 190, 210, 230],
        [100, 130, 160, 190, 210, 230],
        [100, 130, 160, 190, 210, 230],
        [100, 130, 160, 190, 210, 230],
        [100, 130, 160, 190, 210, 230]
    ]

    valveCaliOn_Right = [
        [
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [7, 13, 22, 36, 50, 255]
        ],
        [
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [7, 13, 22, 36, 50, 255]
        ],
        [
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [7, 13, 22, 36, 50, 255]
        ],
        [
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [8, 15, 25, 40, 80, 255]
        ],
        [
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [3, 4, 5, 6, 8, 20],
            [9, 16, 27, 44, 90, 255]
        ]
    ]

    valveCaliOff_Right = [
        [50, 60, 70, 80, 90, 100],
        [50, 60, 70, 80, 90, 100],
        [50, 60, 70, 80, 90, 100],
        [50, 60, 70, 80, 90, 100],
        [50, 60, 70, 80, 90, 100],
        [60, 80, 110, 130, 160, 200]
    ]

    @staticmethod
    def calculate_valve_timing(tarPres, fingerID, presSource, whichHand):
        if whichHand == "Left":
            valveCaliOn = HaptGloveValvesCalibrationData.valveCaliOn_Left
            valveCaliOff = HaptGloveValvesCalibrationData.valveCaliOff_Left
        elif whichHand == "Right":
            valveCaliOn = HaptGloveValvesCalibrationData.valveCaliOn_Right
            valveCaliOff = HaptGloveValvesCalibrationData.valveCaliOff_Right
        else:
            return None

        index1 = (170000 - presSource) // 2000
        index1 = max(0, min(index1, 4))

        valveSelectedOn = valveCaliOn[index1][fingerID][:]
        valveSelectedOff = valveCaliOff[fingerID][:]

        return HaptGloveValvesCalibrationData.GetValveTiming(tarPres, valveSelectedOn, valveSelectedOff)

    @staticmethod
    def GetValveTiming(tarPres, valveSelectedOn, valveSelectedOff):
        valveTiming = [0, 0]
        if tarPres == 0:
            valveTiming[0] = 0
            valveTiming[1] = 0
        elif tarPres < 10:
            valveTiming[0] = 150
            valveTiming[1] = tarPres
        elif tarPres < 20:
            valveTiming[0] = int(((tarPres - 10) / 10.0) * (valveSelectedOn[1] - valveSelectedOn[0]) + valveSelectedOn[0])
            valveTiming[1] = int(((tarPres - 10) / 10.0) * (valveSelectedOff[1] - valveSelectedOff[0]) + valveSelectedOff[0])
        elif tarPres < 30:
            valveTiming[0] = int(((tarPres - 20) / 10.0) * (valveSelectedOn[2] - valveSelectedOn[1]) + valveSelectedOn[1])
            valveTiming[1] = int(((tarPres - 20) / 10.0) * (valveSelectedOff[2] - valveSelectedOff[1]) + valveSelectedOff[1])
        elif tarPres < 40:
            valveTiming[0] = int(((tarPres - 30) / 10.0) * (valveSelectedOn[3] - valveSelectedOn[2]) + valveSelectedOn[2])
            valveTiming[1] = int(((tarPres - 30) / 10.0) * (valveSelectedOff[3] - valveSelectedOff[2]) + valveSelectedOff[2])
        elif tarPres < 50:
            valveTiming[0] = int(((tarPres - 40) / 10.0) * (valveSelectedOn[4] - valveSelectedOn[3]) + valveSelectedOn[3])
            valveTiming[1] = int(((tarPres - 40) / 10.0) * (valveSelectedOff[4] - valveSelectedOff[3]) + valveSelectedOff[3])
        elif tarPres < 60:
            valveTiming[0] = int(((tarPres - 50) / 10.0) * (valveSelectedOn[5] - valveSelectedOn[4]) + valveSelectedOn[4])
            valveTiming[1] = int(((tarPres - 50) / 10.0) * (valveSelectedOff[5] - valveSelectedOff[4]) + valveSelectedOff[4])
        else:
            valveTiming[0] = valveSelectedOn[5]
            valveTiming[1] = valveSelectedOff[5]
        return valveTiming


Haptics.fun_list = {
    1: 'FI_BMP280',
    4: 'FI_MICROTUBE',
    5: 'FI_CLUTCHGOTACTIVATED'
}
