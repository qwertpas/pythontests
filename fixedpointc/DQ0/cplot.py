import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

dir = os.path.dirname(os.path.abspath(__file__))+'/'

c_filename = "abc.c"
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

rms_error = np.sqrt((df["V_w-V_w_f"] ** 2).mean())
print(rms_error)

df.plot()
plt.show()