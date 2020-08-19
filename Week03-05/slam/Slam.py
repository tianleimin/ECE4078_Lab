import numpy as np
import SlamMap
import matplotlib.patches as patches

class Slam:
    # Implementation of an EKF for SLAM
    # The state is ordered as [x; y; theta; l1x; l1y; ...; lnx; lny]

    # Utility
    # -------

    def __init__(self, robot):
        # State components
        self.robot = robot
        self.markers = np.zeros((2,0))
        self.taglist = []

        # Covariance matrix
        self.P = np.zeros((3,3))
        self.init_lm_cov = 1e3

    def number_landmarks(self):
        return int(self.markers.shape[1])

    def get_state_vector(self):
        state = np.concatenate((self.robot.state, np.reshape(self.markers, (-1,1), order='F')), axis=0)
        return state
    
    def set_state_vector(self, state):
        self.robot.state = state[0:3,:]
        self.markers = np.reshape(state[3:,:], (2,-1), order='F')
    
    def save_map(self, fname="slam_map.txt"):
        if self.number_landmarks() > 0:
            slam_map = SlamMap.SlamMap(self.markers, self.P[3:,3:], self.taglist)
            slam_map.save(fname)
        
    
    # EKF functions
    # -------------

    def predict(self, raw_drive_meas):
        # TODO: implement the prediction step of EKF
        # ------------------------------------------
        # ----------- Add your code here -----------
        # ------------------------------------------


    def update(self, measurements):
        if not measurements:
            return

        # Construct measurement index list
        tags = [lm.tag for lm in measurements]
        idx_list = [self.taglist.index(tag) for tag in tags]

        # Stack measurements and set covariance
        z = np.concatenate([lm.position.reshape(-1,1) for lm in measurements], axis=0)
        R = np.zeros((2*len(measurements),2*len(measurements)))
        for i in range(len(measurements)):
            R[2*i:2*i+2,2*i:2*i+2] = measurements[i].covariance


        # TODO: compute own measurements
        z_hat = self.robot.measure(self.markers, idx_list)
        z_hat = z_hat.reshape((-1,1),order="F")
        H = self.robot.derivative_measure(self.markers, idx_list)

        x = self.get_state_vector()
        # ------------------------------------------
        # ----------- Add your code here -----------
        # ------------------------------------------


    def state_transition(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        F = np.eye(n)
        F[0:3,0:3] = self.robot.derivative_drive(raw_drive_meas)
        return F
    
    def predict_covariance(self, raw_drive_meas):
        n = self.number_landmarks()*2 + 3
        Q = np.zeros((n,n))
        Q[0:3,0:3] = self.robot.covariance_drive(raw_drive_meas)
        return Q

    def add_landmarks(self, measurements):
        if not measurements:
            return

        th = self.robot.state[2]
        robot_xy = self.robot.state[0:2,:]
        R_theta = np.block([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]])

        # Add new landmarks to the state
        for lm in measurements:
            if lm.tag in self.taglist:
                # ignore known tags
                continue
            
            lm_bff = lm.position
            lm_inertial = robot_xy + R_theta @ lm_bff

            self.taglist.append(int(lm.tag))
            self.markers = np.concatenate((self.markers, lm_inertial), axis=1)

            # Create a simple, large covariance to be fixed by the update step
            self.P = np.concatenate((self.P, np.zeros((2, self.P.shape[1]))), axis=0)
            self.P = np.concatenate((self.P, np.zeros((self.P.shape[0], 2))), axis=1)
            self.P[-2,-2] = self.init_lm_cov**2
            self.P[-1,-1] = self.init_lm_cov**2

    # Plotting functions
    # ------------------

    def draw_slam_state(self, ax) -> None:
        # Draw landmarks
        if self.number_landmarks() > 0:
            ax.plot(self.markers[0,:], self.markers[1,:], 'ko')

        # Draw robot
        arrow_scale = 0.4
        ax.arrow(self.robot.state[0,0], self.robot.state[1,0],
                 arrow_scale * np.cos(self.robot.state[2,0]), arrow_scale * np.sin(self.robot.state[2,0]),
                 head_width=0.3*arrow_scale)

        # Draw covariance
        robot_cov_ellipse = self.make_ellipse(self.robot.state[0:2,0], self.P[0:2,0:2])
        ax.plot(robot_cov_ellipse[0,:], robot_cov_ellipse[1,:], 'r-')

        for i in range(self.number_landmarks()):
            lmi = self.markers[:,i]
            Plmi = self.P[3+2*i:3+2*(i+1),3+2*i:3+2*(i+1)]
            lmi_cov_ellipse = self.make_ellipse(lmi, Plmi)
            ax.plot(lmi_cov_ellipse[0,:], lmi_cov_ellipse[1,:], 'b-')
        
        ax.axis('equal')
        ax.set_xlim(-5+self.robot.state[0],5+self.robot.state[0])
        ax.set_ylim(-5+self.robot.state[1],5+self.robot.state[1])
    
    @staticmethod
    def make_ellipse(x, P):
        p = 0.5
        s = -2 * np.log(1 - p)
        e_vals, e_vecs = np.linalg.eig(P * s)

        t = np.linspace(0, 2 * np.pi)
        ellipse = (e_vecs @ np.sqrt(np.diag(e_vals))) @ np.block([[np.cos(t)],[np.sin(t)]])
        ellipse = ellipse + x.reshape(-1,1)

        return ellipse
