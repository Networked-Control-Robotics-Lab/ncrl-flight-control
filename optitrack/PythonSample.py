from NatNetClient import NatNetClient

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    print( "Received frame", frameNumber )

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    print( "Received frame for rigid body", id )

# This will create a new NatNet client
streamingClient = NatNetClient()

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()
