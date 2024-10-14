# import numpy as np
# from pyswarm import pso
# from math import pi
# import multiprocessing as mp
# import time

# from getObjectiveFcn import getObjectivesFcn

# def optimize_pso(bounds):
#     c_proc = mp.current_process()
#     print("Running on Process", c_proc.name)
#     lb, ub = bounds
#     xopt, fopt = pso(getObjectivesFcn, lb, ub, swarmsize=1, maxiter=1, debug=True)
#     return xopt, fopt

# def main():
#     num_processes = 8
#     start = time.time()
#     lb = [-1.0, 0.0, -0.5, -pi/2, 0.0, -pi, -pi/2]
#     ub = [0.0, 0.8, 1.0, 0, pi/2, pi, pi/2]
#     bounds = [(lb, ub)] * num_processes

#     with mp.Pool(processes=num_processes) as pool:  
#         results = pool.map(optimize_pso, bounds)
#         print(results)
#         best_result = min(results, key=lambda x: x[1])
#         print("best result: ", best_result)
#         end = time.time()
#         print("Time taken: ", end-start)

# if __name__ == "__main__":
#     main()

from scipy.optimize import differential_evolution
from getObjectiveFcn import getObjectivesFcn
from math import pi
import time
import numpy as np
from concurrent.futures import ProcessPoolExecutor

progress = []
progress_val = []

def cb(x, convergence):
    progress.append(x)
    progress_val.append(getObjectivesFcn(x))
    print("***********************************************************")
    print("Current position: ", x)

lb = [-1.0, 0.0, -0.5, -pi/2, 0.0, -pi, -pi/2]
ub = [0.0, 0.8, 1.0, 0, pi/2, pi, pi/2]
start = time.time()
with ProcessPoolExecutor(max_workers=8) as executer:  # Adjust the number of processes based on machine's capability
    result = differential_evolution(getObjectivesFcn, bounds=list(zip(lb, ub)), workers=executer.map, disp=True, callback=cb, updating='deferred')

end = time.time()

progress = np.array(progress)
progress_val = np.array(progress_val)

print("Best result: ", result)
print("Time taken: ", end-start)
print("Progress: ", progress)
print("Progress Value: ", progress_val)

import matplotlib.pyplot as plt
plt.plot(progress_val)
plt.xlabel('Iteration')
plt.ylabel('Objective Function Value')
plt.title('Convergence of Differential Evolution')
plt.show()

np.savetxt('optim_progress_val.txt', progress_val)
np.savetxt('optim_progress.txt', progress)
