import numpy.random as rand
import numpy as np
import matplotlib.pyplot as plt

# You have to write a python program that produces a set of resampled robot poses x using the Sampling-
# Importance-Resampling algorithm and the above stated pose distribution p(x) and proposal distribution
# q(x). 
# Show the distribution of samples after the resampling step for k = 20, 100, 1000 samples /
# particles. Plot a histogram of the samples together with the wanted pose distribution p(x) (Hint: Take
# care the the histogram should be scaled as a probability density function to be comparable with p(x)).
# How well does the histogram of samples fit with p(x) for the different choices of k? Can you imagine
# any problems occurring when using a uniform proposal distribution with our particular choice of p(x)?

def norm(x, my, sig):
   return (1/(np.sqrt(2*np.pi))*sig) * (np.e**((-1/2)*((x-my)**2/sig**2)))

def p(x):
   return 0.3 * norm(x, 2.0, 1) + 0.4 * norm(x, 5.0, 2) + 0.3 * norm(x, 9.0, 1) 

def q(x):
    if 0 <= x < 16:
        return 1/15
    else:
        return 0

data = rand.uniform(low=0.0, high=15.0, size=20) 
data1 = rand.uniform(low=0.0, high=15.0, size=100) 
data2 = rand.uniform(low=0.0, high=15.0, size=1000) 


def SIR(data, p, q):
    w = []
    w_norm = []
    for elem in data:
        w.append(p(elem)/q(elem))

    for elem in w:
        w_norm.append(elem/(sum(w)))

    return rand.choice(a=data, replace=True, p=w_norm, size=len(data))

res = SIR(data2, p, q)
plt.figure() # initializes the plot.
x = np.linspace(0,15,1000)
plt.plot(x, p(x))
plt.hist(res, density=True, bins=20)
plt.show()
