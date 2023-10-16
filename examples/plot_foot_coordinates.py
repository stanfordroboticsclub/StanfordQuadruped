import pandas as pd
import matplotlib.pyplot as plt
import argparse
import numpy as np

# Argument parsing setup
parser = argparse.ArgumentParser(
    description="Plot data from a CSV log file using Matplotlib."
)
parser.add_argument("--logfile", help="Path to the CSV log file to be plotted.")
args = parser.parse_args()

# Load the CSV file into a DataFrame without setting an index column
df = pd.read_csv(args.logfile, index_col=False)

# Define the columns you'd like to plot
columns = [
    "fr_x",
    "fr_y",
    "fr_z",
    # "fl_x",
    # "fl_y",
    # "fl_z",
    # "br_x",
    # "br_y",
    # "br_z",
    # "bl_x",
    # "bl_y",
    # "bl_z",
]
plt.figure()
# Plot each column
for col in columns:
    plt.plot(df[col], label=col, marker=".")

# Setting legend, title, and labels for clarity
plt.legend()
plt.title("Values from CSV Log File")
plt.xlabel("Entry #")
plt.ylabel("Value")

plt.figure()
plt.plot(np.array(df["fr_x"]), np.array(df["fr_z"]), marker=".")

# Display the plot
plt.show()
