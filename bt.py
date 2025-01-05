import socket

def main():
    # Bluetooth configuration
    bluetooth_address = "0.0.0.0"  # Listen on all interfaces
    bluetooth_port = 5005         # Port to match with your gamepad setup

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bluetooth_address, bluetooth_port))

    print(f"Listening for joystick data on {bluetooth_address}:{bluetooth_port}...")

    try:
        while True:
            # Receive data from the Bluetooth connection
            data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
            decoded_data = data.decode('utf-8').strip()
            
            # Log the received data
            print(f"Received from {addr}: {decoded_data}")

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
