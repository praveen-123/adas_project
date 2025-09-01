# ecu_logger.py
import csv
from datetime import datetime

# --- CONFIGURATION ---
input_log = 'camera_log.csv'  # Input file from the camera simulator
output_log = 'can_messages.log' # Output file for our simulated CAN messages
ecu_id = '0x7E0' # Simulated ECU ID. 0x7E0 is a common ID for powertrain ECUs.

# --- MENTOR'S EXPLANATION ---
# In a real vehicle, an ECU doesn't write a CSV file. It takes data from its sensors,
# packages it into a standard format (a CAN frame), and broadcasts it on the CAN bus.
# This script simulates that process. It reads the human-readable CSV and converts it
# into a log that resembles what an automotive engineer would see when debugging a CAN bus.

print(f"Simulating ECU logging process...")
print(f"Reading from: {input_log}")
print(f"Writing to: {output_log}")
print(f"Simulated ECU ID: {ecu_id}")

try:
    # Open the camera log for reading and the CAN log for writing
    with open(input_log, 'r') as csv_input, open(output_log, 'w') as log_output:
        # Create a CSV reader to parse the input file
        reader = csv.reader(csv_input)
        # Skip the header row of the input CSV
        next(reader)

        # Write a header for our new CAN log file
        log_output.write("Timestamp [UTC]\tECU_ID\tMessage_Type\tPayload\n")
        log_output.write("---\n")

        # Process each row from the camera log
        for row in reader:
            # Unpack the row into variables
            frame_id, timestamp, status, file_path = row

            # --- SIMULATE CAN MESSAGE CREATION ---
            # A real CAN message has an ID, a type, and a data payload (usually 8 bytes).
            # We will create a simplified textual representation of this.

            # Simulate different message types based on the camera status
            if status == "OK":
                # If status is OK, create a normal data message.
                # The payload might contain the frame number and a checksum in a real system.
                message_type = "DATA"
                # Simulate a simple payload: "FRAME_<ID>_OK"
                payload = f"FRAME_{frame_id}_OK"
            else:
                # If status is not OK, create an error/diagnostic message.
                message_type = "DTC"  # Diagnostic Trouble Code
                # Simulate a payload with a standard DTC format (e.g., P-Camera-001)
                payload = f"U0100"  # A real DTC code for 'Lost Communication With Camera'

            # --- SIMULATE LOGGING THE CAN MESSAGE ---
            # In a real system, a tool like Vector CANalyzer would log this data.
            # The log format is often: Timestamp, ECU_ID, Message_Type, Data
            log_line = f"{timestamp}\t{ecu_id}\t{message_type}\t{payload}\n"
            log_output.write(log_line)

            # Print the simulated message to the console for debugging
            print(f"Logged: {log_line.strip()}")

    print("\n✅ ECU logging simulation complete!")
    print(f"✅ CAN messages saved to: {output_log}")

except FileNotFoundError:
    print(f"❌ Error: Could not find the input file '{input_log}'.")
    print("Please run the camera_simulator.py script first to generate it.")
except Exception as e:
    print(f"❌ An unexpected error occurred: {e}")