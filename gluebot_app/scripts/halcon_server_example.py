#!/usr/bin/env python

import socket
import json

# settings
TCP_IP = 'localhost'
TCP_PORT = 9091
BUFFER_SIZE = 1024

# connect to rosbridge server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

# send json string to advertise service
service_msgs = '{"op": "advertise_service", "type": "gluebot_app/GetPose2D", "service": "/halcon"}'
s.send(service_msgs)

# res += '"values": {"pose": {"x": 0.5645, "y": -0.3488, "theta": -327.99}},'
# function to create the response string with the same id as the request
def create_response(request_msg):
  """ create response string

  The id in the response must match the id in the request.
  We use the json library to convert the rosbridge string into
  a python dict object.

  """

  request_data = json.loads(request_msg)
  request_id = '"' + request_data['id'] + '"'

  res = '{ "op": "service_response",'
  res += '"id": ' + request_id + ', '
  res += '"service": "/halcon",'
  res += '"values": {"pose": {"x": 0.6, "y": 0.1, "theta": 90.0}},'
  res += '"result": true'
  res += '}'
  return res


# keep listening for service requests
while(True):
  req = s.recv(BUFFER_SIZE)
  if (len(req) > 0):

    print("Client received request:")
    print(req)

    resp = create_response(req)

    print("Client sending response:")
    print(resp)

    s.send(resp)


s.close()


