#!/usr/bin/env python3
"""
ArduPilot BIN Log Parser - RCOU Channel Extractor
Extracts RCOU channels 1-4 from ArduPilot BIN logs and saves to CSV
"""

import argparse
import csv
from pymavlink import mavutil


def parse_bin_to_csv(filename):
    """
    Parse ArduPilot BIN log and extract RCOU channels 1-4 to CSV
    
    Args:
        filename: Path to the .bin log file
    """
    # Open the BIN log file
    mlog = mavutil.mavlink_connection(filename)
    
    # Prepare output filename
    output_filename = filename + "_rcou.csv"
    
    # Open CSV file for writing
    with open(output_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write header
        writer.writerow(['TimeUS', 'C1', 'C2', 'C3', 'C4'])
        
        # Parse through the log
        while True:
            msg = mlog.recv_match(type='RCOU')
            if msg is None:
                break
            
            # Extract timestamp and channels 1-4
            time_us = msg.TimeUS
            c1 = msg.C1
            c2 = msg.C2
            c3 = msg.C3
            c4 = msg.C4
            
            # Write to CSV
            writer.writerow([time_us, c1, c2, c3, c4])
    
    print(f"Successfully converted {filename}")
    print(f"Output saved to: {output_filename}")


def main():
    parser = argparse.ArgumentParser(
        description='Extract RCOU channels 1-4 from ArduPilot BIN log to CSV'
    )
    parser.add_argument(
        '--filename',
        required=True,
        help='Path to the ArduPilot BIN log file'
    )
    
    args = parser.parse_args()
    
    try:
        parse_bin_to_csv(args.filename)
    except FileNotFoundError:
        print(f"Error: File '{args.filename}' not found")
    except Exception as e:
        print(f"Error processing file: {e}")


if __name__ == '__main__':
    main()
