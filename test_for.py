import numpy as np

i = 0
for l in np.arange(0.07, 0.46, 0.03):
    
    print(i, l)
    i += 1

l = 0.099

print(int((l-0.07)/0.03))