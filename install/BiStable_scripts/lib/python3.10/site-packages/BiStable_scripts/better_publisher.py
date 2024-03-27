from approxeng.input.selectbinder import ControllerResource
from time import sleep

while True:
    try:
        with ControllerResource() as joystick:
            print('Found a joystick and connected')
            prev_button_states = {button: False for button in ['square', 'cross', 'circle', 'triangle']}
            prev_trigger_states = {trigger: False for trigger in ['l1', 'l2', 'r1', 'r2']}

            while joystick.connected:
                # Read left joystick axes
                left_x = joystick['lx']
                left_y = joystick['ly']
                print(left_x, left_y)
                
                # Read button and trigger states
                button_states = {button: joystick[button] for button in prev_button_states}
                trigger_states = {trigger: joystick[trigger] for trigger in prev_trigger_states}
                
                for button in button_states:
                    if button_states[button] and not prev_button_states[button]:
                        for trigger in trigger_states:
                            if trigger_states[trigger]:
                                if button == 'square':
                                    print("kp -0.1")
                                elif button == 'cross':
                                    print("ki -0.1")
                                elif button == 'circle':
                                    print("kd -0.1")
                                elif button == 'triangle':
                                    print("eq -0.1")
                            elif trigger_states[trigger]:
                                if button == 'square':
                                    print("kp -0.01")
                                elif button == 'cross':
                                    print("ki -0.01")
                                elif button == 'circle':
                                    print("kd -0.01")
                                elif button == 'triangle':
                                    print("eq -0.01")
                            elif trigger_states[trigger]:
                                if button == 'square':
                                    print("kp +0.01")
                                elif button == 'cross':
                                    print("ki +0.01")
                                elif button == 'circle':
                                    print("kd +0.01")
                                elif button == 'triangle':
                                    print("eq +0.01")
                            elif trigger_states[trigger]:
                                if button == 'square':
                                    print("kp +0.1")
                                elif button == 'cross':
                                    print("ki +0.1")
                                elif button == 'circle':
                                    print("kd +0.1")
                                elif button == 'triangle':
                                    print("eq +0.1")
                                    
                # Update previous button states
                prev_button_states = button_states
                
                # Sleep for a short duration to avoid busy-waiting
                sleep(0.05)  # Adjust the sleep duration as needed
        # Joystick disconnected...
        print('Connection to joystick lost')
    except IOError:
        # No joystick found, wait for a bit before trying again
        print('Unable to find any joysticks')
        sleep(1.0)
