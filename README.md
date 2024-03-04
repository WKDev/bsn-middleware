# api_middleware
- robot software for smart nursing system.

## consists of....
- AGV
    - api_middleware : collects cmds and fetches stat from AGV by TCP Serial
    - teleop.py : testing tool
    - teleop_gamepad : publishes /cmd_vel from gamepad

- Cobot
    - not yet 


- Smart-rack
    - env_manager.py : controls LED, collects env_sensor
    - gpio_driver.py : controls GPIO, 
    - rs485_driver.py : fetches data from rs485-modbus, it is used for reading envsensor
    - teleop rack : testing tool for smart-rack

