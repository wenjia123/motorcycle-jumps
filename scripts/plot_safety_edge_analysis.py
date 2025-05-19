import os
import pandas as pd
import sqlite3 as sql
import matplotlib.pyplot as plt

# Set the base directory
base_dir = r"C:.\Downloads\RSS2024-main"

# Set the directory where the files are located
directory = os.path.join(base_dir, "scripts", "casestudy_data")

# List the files you want to exclude
excluded_files = ['edge_slant', 'road_position_index', 'road_positions']

# Create a connection to the SQLite database
sharp_conn = sql.connect('sharp_database.db')
safety_conn = sql.connect('safety_database.db')
misc_conn = sql.connect('misc_database.db')

safety_cur = safety_conn.cursor()
sharp_cur = sharp_conn.cursor()

# Create a list to store the data for all trials
safety_data = []
sharp_data = []

# Loop through all files in the directory
for filename in os.listdir(directory):
    if filename.endswith(".txt") and filename not in excluded_files:
        file_path = os.path.join(directory, filename)
        # Specify the column names explicitly as a list of strings
        column_names = ['time', 'goalRoll', 'Torque', 'speed', 'roll', 'rollrate', 'steer', 'steerrate', 'intE', 'goalLane', 'yaw', 'y']
        df = pd.read_csv(file_path, names=column_names)
        table_name = filename[:-4]  # Remove the '.txt' extension

        # Determine which database to use based on the filename
        if 'slant_90' in table_name:
            df.to_sql(table_name, sharp_conn, if_exists='replace', index=False)
            print(f"Created table '{table_name}' in the sharp database.")
            sharp_cur.execute(f"SELECT MAX(Torque) AS max_torque, MAX(roll) AS max_roll, MAX(steer) AS max_steer FROM '{table_name}'")
            max_values = sharp_cur.fetchone()
            sharp_data.append(max_values)
        elif 'slant_60' in table_name:
            df.to_sql(table_name, safety_conn, if_exists='replace', index=False)
            print(f"Created table '{table_name}' in the safety database.")
            safety_cur.execute(f"SELECT MAX(Torque) AS max_torque, MAX(roll) AS max_roll, MAX(steer) AS max_steer FROM '{table_name}'")
            max_values = safety_cur.fetchone()
            safety_data.append(max_values)
        else:
            # Create a table for the current dataframe in the case_study_data.db database
            df.to_sql(table_name, misc_conn, if_exists='replace', index=False)
            print(f"Created table '{table_name}' in the case_study_data.db database.")


# Close the database connections
sharp_conn.close()
safety_conn.close()
misc_conn.close()
