import smbus2  # Install with: pip install smbus2
import time

# Define I2C parameters
I2C_BUS = 1           # Raspberry Pi uses I2C bus 1

# Create I2C bus object
bus = smbus2.SMBus(I2C_BUS)

def send_data(addr, data):
    """Send a string to the specified I2C board address."""
    message = data.encode('utf-8')  # Convert string to bytes
    try:
        # The second parameter is the command byte; here we use the first byte of our message,
        # and the remainder as the actual payload.
        bus.write_i2c_block_data(addr, list(message)[0], list(message)[1:])
        print(f"Sent to board at 0x{addr:02X}: {data}")
    except Exception as e:
        print(f"Error sending data: {e}")
        reset_i2c_bus()

def read_data(addr):
    """Read response from the board at the specified I2C address."""
    try:
        length = 32  # Max length to read
        data = bus.read_i2c_block_data(addr, 0, length)
        message = bytes(data).decode('utf-8').strip("\x00")
        print(f"Received from board at 0x{addr:02X}: {message}")
        return message
    except Exception as e:
        print(f"Error reading data: {e}")
        reset_i2c_bus()
        return None

def reset_i2c_bus():
    """Close and reopen the I2C bus to fix timeout errors."""
    global bus
    try:
        bus.close()
        time.sleep(1)
        bus = smbus2.SMBus(I2C_BUS)
        print("I2C bus reset successfully.")
    except Exception as e:
        print(f"Error resetting I2C bus: {e}")

while True:
    '''
    Commands:
    
    D1 - Set motor direction to 1
    D0 - Set motor direction to 0
    S200 - Move motor 200 steps
    '''
    user_input = input("Enter board address and command (e.g., 0x10 D1 / S200): ")
    if user_input.strip() == "":
        continue  # Skip empty input

    # Split into two parts: address and command
    parts = user_input.split(" ", 1)
    if len(parts) != 2:
        print("Please enter an address followed by a command.")
        continue

    # Parse the address (hexadecimal if it starts with "0x", else decimal)
    try:
        if parts[0].lower().startswith("0x"):
            addr = int(parts[0], 16)
        else:
            addr = int(parts[0])
        command = parts[1]
    except Exception as e:
        print("Error parsing input:", e)
        continue

    send_data(addr, command)
    # Optionally, you can add a delay and then read a response:
    # time.sleep(1)
    # read_data(addr)
