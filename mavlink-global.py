import time
from pymavlink import mavutil

import modules.pyRockBlock3.rockBlock as rockBlock
from modules.pyRockBlock3.rockBlock import rockBlockProtocol

master = 'udp:0.0.0.0:14551'

#lora_port = '/dev/lora'
lora_port = 'udpout:0.0.0.0:14771'
lora_interval = 10  # Seconds
lora_heartbeat_count = 0

satellite_port = "/dev/rockblock"
satellite_interval = 600  # Seconds
satellite_heartbeat_count = 0

vehicle = mavutil.mavlink_connection(master)
lora = mavutil.mavlink_connection(lora_port)

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
        basestation_message = lora.recv_msg()

        if vehicle_message:
            vehicle_message_dict = vehicle_message.to_dict()

            if vehicle_message_dict['mavpackettype'] == 'HIGH_LATENCY2':
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
        if basestation_message:
            vehicle.mav.srcSystem = 201
            vehicle.mav.srcComponent = 1
            vehicle.mav.send(basestation_message)
            basestation_message_dict = basestation_message.to_dict()
            print("BaseStation: ", basestation_message_dict)

        if not basestation_message and not vehicle_message:
            # No Messages at this time
            # Wait a little
            time.sleep(0.1)

    except KeyboardInterrupt:
        raise
    except:
        pass
