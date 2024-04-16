from approxeng.input.selectbinder import ControllerResource
from time import sleep

tracking_state = 0
while True:
    try:
        with ControllerResource() as joystick:
            print('Found a joystick and connected')
            prev_square_state = None
            prev_cross_state = None
            prev_circle_state = None
            prev_triangle_state = None
            prev_L1_state = None
            prev_R1_state = None            

            while joystick.connected:
                
                eq_change = 0
                kp_change = 0
                ki_change = 0
                kd_change = 0

                # Read left joystick axes
                left_x = joystick['lx']
                left_y = joystick['ly']
                #print(left_x, left_y)
                
                # Read button states
                square_state = joystick['square']
                cross_state = joystick['cross']
                circle_state = joystick['circle']
                triangle_state = joystick['triangle']

                L1_state = joystick['l1']
                L2_state = joystick['l2']
                R1_state = joystick['r1']
                R2_state = joystick['r2']
                #R3_state = joystick['r3']

                if L1_state != prev_L1_state and R1_state:
                    tracking_state = 1 - tracking_state

                if R1_state != prev_R1_state and L1_state:
                    tracking_state = 1 - tracking_state


                #print(tracking_state)
                # Check if button is pressed down
                if square_state and not prev_square_state:
                    if L1_state:
                        #print("kp -0.1")
                        kp_change = -0.1
                    
                if cross_state and not prev_cross_state:
                    if L1_state:
                        #print("ki -0.1")
                        ki_change = -0.1

                if circle_state and not prev_circle_state:
                    if L1_state:
                        #print("kd -0.1")
                        kd_change = -0.1

                if triangle_state and not prev_triangle_state:
                    if L1_state:
                        #print("eq -0.1")
                        eq_change = -0.1

                if square_state and not prev_square_state:
                    if L2_state:
                        #print("kp -0.01")
                        kp_change = -0.01
                    
                if cross_state and not prev_cross_state:
                    if L2_state:
                        #print("ki -0.01")
                        ki_change = -0.01

                if circle_state and not prev_circle_state:
                    if L2_state:
                        #print("kd -0.01")
                        kd_change = -0.01

                if triangle_state and not prev_triangle_state:
                    if L2_state:
                        #print("eq -0.01")
                        eq_change = -0.01

                if square_state and not prev_square_state:
                    if R1_state:
                        #print("kp +0.01")
                        kp_change = 0.1
                    
                if cross_state and not prev_cross_state:
                    if R1_state:
                        #print("ki +0.01")
                        ki_change = 0.1

                if circle_state and not prev_circle_state:
                    if R1_state:
                        #print("kd +0.01")
                        kd_change = 0.1

                if triangle_state and not prev_triangle_state:
                    if R1_state:
                        #print("eq +0.01")
                        eq_change = 0.1

                if square_state and not prev_square_state:
                    if R2_state:
                        #print("kp +0.1")
                        kp_change = 0.01
                    
                if cross_state and not prev_cross_state:
                    if R2_state:
                        #print("ki +0.1")
                        ki_change = 0.01

                if circle_state and not prev_circle_state:
                    if R2_state:
                        #print("kd +0.1")
                        kd_change = 0.01

                if triangle_state and not prev_triangle_state:
                    if R2_state:
                        #print("eq +0.1")
                        eq_change = 0.01

                # Update previous button states
                prev_square_state = square_state
                prev_cross_state = cross_state
                prev_circle_state = circle_state
                prev_triangle_state = triangle_state
                prev_L1_state = L1_state
                prev_R1_state = R1_state

                data_to_be_published = [left_x, left_y, eq_change, kp_change, ki_change, kd_change, tracking_state]
                print(data_to_be_published)
                
                # Sleep for a short duration to avoid busy-waiting
                sleep(0.05)  # Adjust the sleep duration as needed
        # Joystick disconnected...
        print('Connection to joystick lost')
    except IOError:
        # No joystick found, wait for a bit before trying again
        print('Unable to find any joysticks')
        sleep(1.0)
