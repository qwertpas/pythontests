import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

dir = os.path.dirname(os.path.abspath(__file__))+'/'

c_filename = "dq0.c"
name = c_filename.split('.')[0] #just the filename without extension

# Compile the C file (replace gcc with the appropriate compiler if needed)
binary = dir+"binaries/"+name
compile_command = ["/usr/bin/g++", dir+c_filename, "-o", binary]
subprocess.run(compile_command, check=True)

# Execute the compiled C file and capture the output in output.txt
output_csv = dir+name+"_out.csv"
with open(output_csv, "w") as output_file:
    subprocess.run(binary, stdout=output_file, check=True)

# Read c output 
df = pd.read_csv(output_csv, delimiter=',')
df.columns = df.columns.str.strip()

pd.set_option('display.max_rows', None)
print(df)

rms_err_d = np.sqrt((df["errId"] ** 2).mean())
rms_err_q = np.sqrt((df["errIq"] ** 2).mean())
print(rms_err_d, rms_err_q)

df.plot(marker='o', markersize=1, linewidth=1)

plt.plot(df.index, df['errIq'], label='errIq', markersize=1)
plt.plot(df.index, df['errId'], label='errId', markersize=1)
plt.legend()
plt.show()