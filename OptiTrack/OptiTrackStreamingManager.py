# -----------------------------------------------------------------------
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/8/19
# Summary:  OptiTrackからデータを受け取るマネージャー
# -----------------------------------------------------------------------

from threading import local
from . import NatNetClient
import numpy as np

serverAddress = ''
localAddress = ''

class OptiTrackStreamingManager:
	# ---------- Variables ---------- #
	position = {}	# dict { 'ParticipantN': [x, y, z] }. 	N is the number of participants' rigid body. Unit = [m]
	rotation = {}	# dict { 'ParticipantN': [x, y, z, w]}. N is the number of participants' rigid body

	def __init__(self, rigid_body_num: int = 2, mocapServer: str = '', mocapLocal: str = ''):
		global serverAddress
		global localAddress
		serverAddress = mocapServer
		localAddress = mocapLocal

		for i in range(rigid_body_num):
			self.position['rigid_body'+str(i+1)] = np.zeros(3)
			self.rotation['rigid_body'+str(i+1)] = np.zeros(4)


	# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
	def receive_new_frame(self, data_dict):
		order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
					"labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
		dump_args = False
		if dump_args == True:
			out_string = "    "
			for key in data_dict:
				out_string += key + "="
				if key in data_dict :
					out_string += data_dict[key] + " "
				out_string+="/"
			print(out_string)

	# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
	def receive_rigid_body_frame( self, rigid_body_id, position, rotation ):
		"""
		Receives the position and rotation of the active RigidBody.
		Position: [x, y, z], Unit = [m]
		Rotation: [x, y, z, w]

		Parameters
		----------
		new_id: int
			RigidBody id
		position: array
			Position
		rotation: array
			Rotation
		"""
		if 'rigid_body'+str(rigid_body_id) in self.position:
			self.position['rigid_body'+str(rigid_body_id)] = np.array(position)
			self.rotation['rigid_body'+str(rigid_body_id)] = np.array(rotation)

	def stream_run(self):
		streamingClient = NatNetClient.NatNetClient(serverIP=serverAddress, localIP=localAddress)

		# Configure the streaming client to call our rigid body handler on the emulator to send data out.
		streamingClient.new_frame_listener = self.receive_new_frame
		streamingClient.rigid_body_listener = self.receive_rigid_body_frame
		streamingClient.run()
