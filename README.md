# AI-Hat-2-RPI5



FOR SITL:
~/ardupilot/ArduCopter$ sim_vehicle.py -v ArduCopter --console --map --out=udp:192.168.1.181:14550   ---ip for the raspberry pi  (no physical pixhawk required)
Run the code after running the previous command on any other PC.. The udpout should be the IP of the RPI.
In the code: use 0.0.0.0:14550 to get connected to the SITL of another PC.


FOR Real-life:
Run mavproxy.py and the device will get detected
use output - 127.0.0.1:14550
In the code: use 127.0.0.1:14550 to get the physical connection to the vehicle and run the strike



👨‍💻 Maintainer
```bash
Udit Ray
Email: udit.ray@indowings.com
Org: Indo Wings Private Limited
Website: https://www.indowings.com

