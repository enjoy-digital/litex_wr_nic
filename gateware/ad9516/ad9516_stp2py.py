#!/usr/bin/env python3
import sys

def parse_stp_to_python(stp_file_path):
    """
    Parses a .stp file into a Python configuration format for AD9516.

    :param stp_file_path: Path to the .stp file.
    :return: Python configuration structure as a list of (address, value) tuples.
    """
    config = []

    try:
        with open(stp_file_path, 'r') as file:
            lines = file.readlines()

        for line in lines:
            # Only process lines that look like address-value pairs
            parts = line.strip().strip('"').split('","')
            if len(parts) == 3:  # Ensure the line has exactly 3 parts
                addr_hex, _, value_hex = parts
                try:
                    addr = int(addr_hex, 16)  # Convert address from hex to int
                    value = int(value_hex, 16)  # Convert value from hex to int
                    config.append((addr, value))
                except ValueError:
                    continue  # Skip lines that cannot be parsed
    except FileNotFoundError:
        print(f"Error: File '{stp_file_path}' not found.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error reading file: {e}", file=sys.stderr)
        sys.exit(1)

    return config


def generate_python_config(config):
    """
    Generates the Python configuration as a string, packing 4 tuples per line.

    :param config: List of (address, value) tuples.
    :return: Python configuration as a string.
    """
    output = ["AD9516_CONFIG = ["]
    for i in range(0, len(config), 4):  # Process 4 tuples per line
        line = ", ".join(f"(0x{addr:04X}, 0x{value:02X})" for addr, value in config[i:i+4])
        output.append(f"    {line},")
    output.append("]")
    return "\n".join(output)


def main():
    # Check if a .stp file argument is provided
    if len(sys.argv) != 2:
        print("Usage: ad9516_stp2py <stp_file_path>", file=sys.stderr)
        sys.exit(1)

    stp_file_path = sys.argv[1]

    # Parse the .stp file
    config = parse_stp_to_python(stp_file_path)

    # Generate and print the Python configuration
    python_config = generate_python_config(config)
    print(python_config)


if __name__ == "__main__":
    main()
