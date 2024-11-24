import numpy as np
import socket
import time as t
import os
from scipy.linalg import sqrtm, pinv

#import dataread_local as drl
#### Development of UKF-UMV based nonlinear indoor thermal state estimation with unknown inputs
# Code developed by Bed Prakash Das
# Version . 4
# Implemented with 8 hours data

__author__ = "Bed Prakash Das"
__license__ = "GNU General Public License v3.0"
__version__ = "1.0"
__maintainer__ = "B P Das"
__status__ = "Pre-Production"



# Function for socket communication

def getsckt_data(port,host):
        data = []
        dummy = []
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
                s.bind((host,port))
        except socket.error as e:
                s.connect((host,port))
                temp = s.recv(1024).decode()
                data.append(temp)
        print(data[0].split(","))
        return(np.array(data[0].split(",")))
T_O = 30.0
T_W_1 = 24.0
T_W_2 = 25.0
T_W_3 = 26.0
T_W_4 = 25.0

file_toSave = 'T_Estimated.csv'

Mode = 'w'

f = open(file_toSave,Mode)

f.write('slno, Wall_1,Humid1, Wall_2,Humid2, Wall3,Humid3, Wall_4,Humid4, Outside, Comp_Time\n')
# Code initialization for UKF-UMV-UI
# Constants and initialization
StateDim = 16
MeasurementDim = 3
dT = 0.25
N = 240

Dm1 = 10
Dm2 = 10
di = np.array([Dm1, Dm2])

"""
# these values may not work in all application; and should be tuned judiciously

Rw1 = 0.000351140288228321
Rw2 = 0.000428279727702223
Rw3 = 0.000415072995189592
Rw4 = 0.000374471937078509
Rw12 = 0.003452776048919
Rw23 = 0.008986615088136
Rw34 = 0.003649527938506
Rw41 = 0.203470366704623
Cw1 = 5137272.603364927
Cw2 = 5210316.84760057
Cw3 = 5188792.80074657
Cw4 = 5084124.008002829

"""
Rw1, Rw2, Rw3, Rw4 = 3.511402882283215e-04, 4.282797277022237e-04, 4.150729951895922e-04, 3.744719370785092e-04
Rw12, Rw23, Rw34, Rw41 = 0.003452776048919, 0.008986615088136, 0.003649527938506, 0.203470366704623
Cw1, Cw2, Cw3, Cw4 = 5.137272603364927e+06, 5.210316847600570e+06, 5.188792800746570e+06, 5.084124008002829e+06
Cpa, Cpw, hwe = 1.005, 1.84, 2501


N = 1920

#Pk_ = np.diag([R^16, R^16]); this is to be optimized using meta-heuristic optimization strategy depending upon the problem statement 

P0 = np.diag([1, 1, 1, 1, 2.981867542213313e4, 1.253867615088063e4, 0.075126803647606e4,
              0.786389066097622e4, 0.410039634238496e4, 0.729781657409922e4,
              0.160731461578683e4, 1e4, 0.093401755868530e4, 0.515301478312748e4,
              0.307001448908194e4, 1.593946276831190e4])

Q = np.diag([0.085, 0.085, 0.085, 0.085, 0.0005, 0.0005, 0.0005, 0.0005, 
             0.00008, 0.00008, 0.00008, 0.00008, 1e5, 1e5, 1e5, 1e5])

R = np.diag([0.361820168372802 * 4, 0.2457 * 4, 0.411676732304285 * 2])
Pk = np.eye(StateDim)
F = np.eye(StateDim)

H = np.eye(3,16)
h = np.ones((3,N))

#Q =  np.diag([R^16, R^16]);  depending upon the problem statement and shound be selected judiciously 
#R =  np.diag([R^3, R^3]); this is to be optimized using meta-heuristic optimization strategy depending upon the problem statement 


# Initial state
x = np.array([t_wall_1[0], t_wall_2[0], t_wall_3[0], t_wall_4[0], Rw1, Rw2, Rw3, Rw4,
              Rw12, Rw23, Rw34, Rw41, Cw1, Cw2, Cw3, Cw4])
H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

u_kn = t_out


x = np.ones((16,N))
y = np.zeros((3,N))

x_hat_= np.zeros((16,N))
x_hat = np.zeros((16,N))
# Assuming `t_wall_1`, `t_wall_2`, `t_wall_3`, `t_wall_4`, `t_out` are loaded as numpy arrays
# You need to replace these placeholders with your data loading method
# Here, we have taken the data from the sensor node

#x[:,0] = [T_W_1, T_W_2, T_W_3, T_W_4, Rw1, Rw2, Rw3, Rw4, Rw12, Rw23, Rw34, Rw41, Cw1, Cw2, Cw3, Cw4]
x_hat_[:,1] = x[:,0]
x_hat[:,0] = x[:,0]


os.system('sudo irsend SEND_ONCE Carrier_AC 27')

for k in range(1,N):
        #st = t.time()
        print(k)

        #HVAC
        T_W_1, H1 = getsckt_data(5545,'10.0.0.106')
        #Beside the projector
        T_W_2, H2 = getsckt_data(5530,'10.0.0.107')
        #Lab Entrance door
        T_W_3, H3 = getsckt_data(5550,'10.0.0.108')
        #Inside Lab door
        T_W_4, H4 = getsckt_data(5500,'10.0.0.109')
        T_O = 30.0
        print("sleep invoked")
        t.sleep(1)
        print("sleep timeout")
        #print(x[:,0])
        st = t.time()

# Unscented Kalman Filter parameters
alpha = 1
beta = 2
kappa = 0
lambda_ = alpha**2 * (StateDim + kappa) - StateDim

# Initialize arrays for storing results
x_estimates = np.zeros((StateDim, N))
P_estimates = np.zeros((StateDim, StateDim, N))
x_estimates[:, 0] = x
P_hat = P0

# Sigma points
sqrt_P = sqrtm((StateDim + lambda_) * P0)
X = np.hstack((x.reshape(-1, 1), x.reshape(-1, 1) + sqrt_P, x.reshape(-1, 1) - sqrt_P))

# Measurement data
z = np.vstack([t_wall_1, t_wall_3, t_wall_4])

# UKF main loop
for k in range(N):
    # Process model
    def process_model(a, u):
        return np.array([
            a[0] + dT * ((((1/a[4]) + (1/a[8]) + (1/a[11])) * (-a[0]/a[12])) + 
                         (a[1]/(a[8]*a[12])) + (a[3]/(a[11]*a[12])) + ((1/a[4]) * (u/a[12]))),
            # Add remaining equations here...
            a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15]
        ])
    
    # Propagate sigma points through process model
    for i in range(2 * StateDim + 1):
        X[:, i] = process_model(X[:, i], u_kn[k])
    x_hat = np.mean(X, axis=1)

    # Predicted covariance
    P_hat = Q + sum(np.outer(X[:, i] - x_hat, X[:, i] - x_hat) for i in range(2 * StateDim + 1)) / (2 * StateDim + lambda_)

    # Measurement update
    Z = H @ X  # Transform sigma points into measurement space
    z_hat = np.mean(Z, axis=1)
    P_zz = R + sum(np.outer(Z[:, i] - z_hat, Z[:, i] - z_hat) for i in range(2 * StateDim + 1)) / (2 * StateDim + lambda_)
    P_xz = sum(np.outer(X[:, i] - x_hat, Z[:, i] - z_hat) for i in range(2 * StateDim + 1)) / (2 * StateDim + lambda_)
    
    # Kalman gain
    K = P_xz @ np.linalg.inv(P_zz)
    x = x_hat + K @ (z[:, k] - z_hat)
    P = P_hat - K @ P_zz @ K.T

    # Store results
    x_estimates[:, k] = x
    P_estimates[:, :, k] = P


f.close()