 #! /usr/bin/env python
# coding=utf-8
class PLCDriverCommands:
    def __init__(self):
        self.TEST_PLC_COMUNICATION=[0x04,0x03,0x00,0x05,0x00,0x01]#RETURN 01 OK D5 register
        self.READ_LINE_ENCODE=[0x04,0x03,0x00,0x0A,0x00,0x01]#D10
        self.READ_LIMIT_SWITCH_STATUS=[0x04,0x03,0x00,0x0B,0x00,0x01]#D11
        self.READ_ECHOS_STATUS=[0x04,0x03,0x00,0x15,0x00,0x01]#D21
        self.WRITE_FRONT_LIGHT_OPEN_FOREVER=[0x04,0x06,0x00,0x14,0x00,0x00]#D20
        self.WRITE_FRONT_LIGHT_FAST_BLINK=[0x04,0x06,0x00,0x14,0x00,0x01]
        self.WRITE_FRONT_LIGHT_SLOW_BLINK=[0x04,0x06,0x00,0x14,0x00,0x02]
        self.WRITE_FRONT_LIGHT_CLOSE=[0x04,0x06,0x00,0x14,0x00,0x03]
        self.WRITE_FLEX_POLE_MOTOR_CLOSE=[0x04,0x06,0x00,0x1E,0x00,0x00]#D21
        self.WRITE_FLEX_POLE_MOTOR_UP=[0x04,0x06,0x00,0x1E,0x00,0x01]
        self.WRITE_FLEX_POLE_MOTOR_DOWN=[0x04,0x06,0x00,0x1E,0x00,0x02]
        self.WRITE_MOBILE_PLATFORM_BRAKE_CLOSE=[0x04,0x06,0x00,0x28,0x00,0x00]
        self.WRITE_MOBILE_PLATFORM_BRAKE_OPEN=[0x04,0x06,0x00,0x28,0x00,0x01]
        self.WRITE_ELECTRIC_SWITCH_PAINTING_CLOSE=[0x04,0x06,0x00,0x32,0x00,0x00]
        self.WRITE_ELECTRIC_SWITCH_PAINTING_OPEN=[0x04,0x06,0x00,0x32,0x00,0x01]

        self.SET_ROTATION_ENCODE_BAUDRATE=[0x02,0x06,0x00,0x05,0x00,0x04]#115200
        self.SET_ROTATION_ENCODE_ADDRESS=[0x02,0x06,0x00,0x04,0x00,0x02]#address 02
        self.READ_ROTATION_ENCODE_DATA=[0x02,0x03,0x00,0x00,0x00,0x02]       


