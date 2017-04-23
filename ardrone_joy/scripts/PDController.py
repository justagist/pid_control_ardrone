import numpy as np

class PDController:

	def __init__(self,kp,kd):

		self.kp = kp
		self.kd = kd


	# s = [x,y,z,phi1,phi2,phi3]
	# s_goal = [x_g,y_g,z_g,phi1_g,phi2_g,phi3_g]
	# return: twist = [vx,vy,vz,w1,w2,w3]
	def computeCmd(self,s_current,s_goal):

		twist = [0,0,0,0,0,0]

		e_x = s_goal[0] - s_current[0]
		e_y = s_goal[1] - s_current[1]
		e_z = s_goal[2] - s_current[2]

		e_phi1 = s_goal[3] - s_current[3]
		e_phi2 = s_goal[4] - s_current[4]
		e_phi3 = s_goal[5] - s_current[5]

		e = [e_x,e_y,e_z,e_phi1,e_phi2,e_phi3]

		linear_error = np.linalg.norm(np.array(e[:3]))
		angular_error = np.linalg.norm(np.array(e[3:]))
		error = [linear_error, angular_error]

		for i in range(0,len(twist)):
			twist[i] = self.kp[i]*e[i] 

		return twist, error


