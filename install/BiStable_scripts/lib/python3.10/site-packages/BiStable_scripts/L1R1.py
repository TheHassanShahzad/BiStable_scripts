import approxeng.input

# Initialize the controller
controller = approxeng.input.Controller()

# Initialize the boolean value
toggle_value = False

# Loop to continuously check for button inputs
while True:
    # Read the controller input
    controller.check_connected()
    
    # Check if L1 button is pressed
    if controller.has_button('l1') and controller.get_button('l1'):
        toggle_value = True
    # Check if R1 button is pressed
    elif controller.has_button('r1') and controller.get_button('r1'):
        toggle_value = False

    # Print the current value of the toggle
    print("Toggle value:", toggle_value)

    # Sleep to avoid hogging CPU resources
    controller.wait_for_changes()
