import time
from pymavlink import mavutil

import modules.pyRockBlock3.rockBlock as rockBlock
from modules.pyRockBlock3.rockBlock import rockBlockProtocol

master = 'udpout:0.0.0.0:14551'

lora_port = '/dev/lora'
lora_interval = 10  # Seconds
lora_heartbeat_count = 0

satellite_port = "/dev/rockblock"
satellite_interval = 600  # Seconds
satellite_heartbeat_count = 0

vehicle = mavutil.mavlink_connection(master)
lora = mavutil.mavlink_connection(lora_port, baud=115200)

#lora = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
#vehicle = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)


hl_received = False

class MoExample(rockBlockProtocol):
    def send(self, message):
        rb = rockBlock.rockBlock("/dev/rockblock", self)
        rb.sendMessage( "Hello World RockBLOCK!")
        rb.close()

    def rockBlockTxStarted(self):
        print("rockBlockTxStarted")

    def rockBlockTxFailed(self):
        print("rockBlockTxFailed")

    def rockBlockTxSuccess(self, momsn):
        print("rockBlockTxSuccess " + str(momsn))


def wait_conn(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time()),  # Unix time
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)


wait_conn(vehicle)

while True:
    try:
        vehicle_message = vehicle.recv_msg()
        lora_message = lora.recv_msg()

        if not hl_received:
            #This doesn't actually work
            vehicle.mav.message_interval_send(235, 100000, 0)

        if vehicle_message:
            vehicle_message_dict = vehicle_message.to_dict()

            if vehicle_message_dict['mavpackettype'] == 'HIGH_LATENCY2':
                hl_received = True
                if lora_heartbeat_count >= lora_interval:
                    try:
                        lora.mav.srcSystem = vehicle.sysid
                        lora.mav.srcComponent = 1
                        lora.mav.send(vehicle_message)
                        lora_heartbeat_count = 0
                    except Exception as e:
                        print(e)
                if satellite_heartbeat_count >= satellite_interval:
                    try:
                        #basestation.mav.srcSystem = vehicle.sysid
                        #basestation.mav.srcComponent = 1
                        #basestation.mav.send(vehicle_message)
                        print("Send satellite message here")
                        satellite_heartbeat_count = 0
                    except Exception as e:
                        print(e)

            if vehicle_message_dict['mavpackettype'] == 'HEARTBEAT':
                lora_heartbeat_count += 1
                satellite_heartbeat_count = 0
        if lora_message:
            vehicle.mav.srcSystem = 201
            vehicle.mav.srcComponent = 1
            vehicle.mav.send(lora_message)
            lora_message_dict = lora_message.to_dict()
            print("LoRa: ", lora_message_dict)

        if not lora_message and not vehicle_message:
            # No Messages at this time
            # Wait a little
            time.sleep(0.1)

    except KeyboardInterrupt:
        raise
    except:
        pass
