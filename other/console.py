import serial.tools.list_ports
import serial
import sys
import os
import time

# Platform-specific imports for detecting key presses
if os.name == 'nt':  # Windows
    import msvcrt
else:  # Unix (Linux, macOS)
    import tty
    import termios

class REPL:
    def __init__(self):
        self.commands = {
            'help': self.help_command,
            'list': self.list_ports,
            'exit': self.exit_command,
            'connect': self.connect_command,
            'disconnect': self.disconnect_command,
            'listen': self.listen_command,
            'savehex': self.savehex_command,
            'clear': self.clear_command
        }
        self.serial_port = None
        self.baudrate = 921600  # Default baud rate
        self.running = True
        self.hex_file = None  # File object for saving hex data
        self.is_connected = False  # Connection status flag
        self.connected_port = None  # The port we are connected to

    def help_command(self, args):
        """Displays help information."""
        print("Available commands:")
        for command in self.commands:
            print(f"- {command}")

    def list_ports(self, args):
        """Lists all available COM ports and their descriptions."""
        ports = serial.tools.list_ports.comports()
        if ports:
            print("Available COM ports:")
            for port in ports:
                print(f"{port.device}: {port.description}")
        else:
            print("No COM ports found.")

    def exit_command(self, args):
        """Exits the REPL."""
        print("Exiting REPL...")
        self.running = False
        sys.exit(0)

    def connect_command(self, args):
        """Connects to a specified serial port."""
        if self.is_connected:
            print(f"You are already connected to {self.connected_port}.")
            return

        if len(args) < 1:
            print("Please specify a port to connect to.")
            return

        port = args[0]
        try:
            self.serial_port = serial.Serial(port, self.baudrate, timeout=1)
            self.is_connected = True  # Set the connection status to True
            self.connected_port = port  # Store the connected port name
            print(f"Connected to {port} at {self.baudrate} baud.")
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")

    def disconnect_command(self, args):
        """Disconnects from the serial port."""
        if not self.is_connected:
            print("You are not connected to any serial port.")
            return

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False  # Set the connection status to False
            print(f"Disconnected from {self.connected_port}")
            self.connected_port = None  # Reset connected port
        else:
            print("No serial port is open to disconnect.")

    def listen_command(self, args):
        """Listens to the serial port and prints out received data."""
        if not self.is_connected or not self.serial_port.is_open:
            print("No serial port is connected.")
            return

        print("Listening for data on the serial port. Press 'Esc' to stop.")
        
        while True:
            # Check if data is available to read from the serial port
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)
                decoded_data = self.decode_data(data)
                print(decoded_data, end='', flush=True)

            # Check for Esc key press to stop listening
            if self.check_escape_key():
                print("\nStopping listening.")
                break

    def savehex_command(self, args):
        """Starts saving the received data to a hex file."""
        if not self.is_connected or not self.serial_port.is_open:
            print("No serial port is connected.")
            return

        # Open the hex file to write the data
        filename = "received_data.hex"
        self.hex_file = open(filename, 'ab')  # Open in append binary mode
        print(f"Saving data to {filename}. Press 'Esc' to stop saving.")

        while True:
            # Check if data is available to read from the serial port
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)

                # Write the raw data (in binary format) to the file
                self.hex_file.write(data)  # Save raw data to hex file

                # Print a dot for each byte of data received
                print(".", end="", flush=True)

            # Check for Esc key press to stop saving
            if self.check_escape_key():
                break

        # Close the file after saving is stopped
        if self.hex_file:
            self.hex_file.close()
            print(f"Hex data saved to {filename}")


    def decode_data(self, data):
        """Decodes data into a string, replacing invalid bytes with placeholders."""
        decoded = []
        for byte in data:
            try:
                # Attempt to decode each byte as UTF-8
                decoded.append(chr(byte))  # Convert the byte to its character representation
            except ValueError:
                # If decoding fails, append a placeholder for the invalid byte
                decoded.append(f'\\x{byte:02x}')
        return ''.join(decoded)

    def check_escape_key(self):
        """Check if the Esc key was pressed."""
        if os.name == 'nt':  # Windows
            return msvcrt.kbhit() and msvcrt.getch() == b'\x1b'  # ESC key
        else:  # Unix (Linux, macOS)
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                return ch == '\x1b'  # ESC key
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def clear_command(self, args):
        """Clears the terminal screen."""
        os.system('cls' if os.name == 'nt' else 'clear')

    def run(self):
        """Main REPL loop."""
        print("Welcome to the Python REPL. Type 'help' for a list of commands.")
        
        while self.running:
            # Show the connected port in the prompt, or "disconnected" if not connected
            connection_status = f"connected to {self.connected_port}" if self.is_connected else "disconnected"
            command = input(f"({connection_status}) > ").strip()

            if command:
                parts = command.split()
                cmd = parts[0]
                args = parts[1:]

                if cmd in self.commands:
                    self.commands[cmd](args)
                else:
                    print(f"Unknown command: {cmd}. Type 'help' for a list of commands.")

def main():
    repl = REPL()
    repl.run()

if __name__ == "__main__":
    main()
