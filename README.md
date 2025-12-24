# MAVCore

## Set Up Environment
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## How to Use
1. Create a `MAVDevice` object to connect to the drone

```python
from mavcore import MAVDevice

udp_device = MAVDevice("udp:127.0.0.1:14550")
serial_device = MAVDevice("/dev/ttyACM0")
```

2. Add listeners to monitor telemetry

```python
from mavcore.messages import Heartbeat, LocalPositionNED

# Monitors heartbeat and local position
heartbeat = device.add_listener(Heartbeat())
local_pos = device.add_listener(LocalPositionNED())

while True:
    print(heartbeat)
    print(local_pos.vx)
```

3. Run protocols

```python
from mavcore.protocols import HeartbeatProtocol, SetModeProtocol
from mavcore.messages import FlightMode

# Sends periodic heartbeats from device and sets mode
hb_protocol = device.run_protocol(HeartbeatProtocol())  # will auto send at 1 hz
set_guided = device.run_protocol(SetModeProtocol(FlightMode.GUIDED))
```


## How to Develop

### Create a MAVMessage:
1. Create a class for your message in the `messages` folder
2. Inherit from the `MAVMessage` class
3. In the `super().__init__` make sure to pass in the name of the message that is the same it appears in the mavlink documentation all caps seperated by _
4. If you want this message to be able to be recieved from the flight controller, override the `decode` function
5. If you want this message to be able to be sent to the flight controller, override the `encode` function
6. (Optional) Override the `__repr__` function to assist in debugging

### Create a MAVProtocol:
Not necessary if you are only receiving a particular message, but neccessary if you want to send something
1. Create a class for your protocol in the `protocols` folder. For convention append "Protocol" to the end of the name.
2. Override the `run` function using `sender.send_msg` and `receiver.wait_for_msg` with `MAVMessage` to build your protocol

## How to Test
After you have installed the [SITL](https://github.com/uci-uav-forge/GNC-26-Knowledge-Base/blob/main/docs/env_setup.md)

1. In one terminal launch the SITL. The coords below launch at the ARC main field west. Give this 2 minutes to load, it should begin listening on UDP Port 14550.
```bash
 cd simcore
 ./sitl_setup.sh 33.64293210548397 -117.82628818855002
```

2. In order to connect your script, instantiate your `MAVDevice` as follows
```python
device = MAVDevice("udp:127.0.0.1:14550")
```

3. If you want to perform commands on the drone thats not in your script go back to the terminal you started up the sitl in. It is interactive and you can issue [mavproxy commands](https://ardupilot.org/mavproxy/docs/getting_started/cheatsheet.html) to it.
Just press `enter` a couple times on the terminal to see the mavproxy vehicle mode prefix.

NOTE: When running on the SITL, mavproxy automatically sends message interval requests for several messages. This means you may see messages published during your script in the SITL that don't appear when testing in person on a real flight controller. Remember to double check your message requests.
