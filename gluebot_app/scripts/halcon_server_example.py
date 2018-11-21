#!/usr/bin/env python

import socket


TCP_IP = 'localhost'
TCP_PORT = 9090
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

message = '{ "op": "subscribe", "topic": "/chatter" }'

service_msgs = '{"op": "advertise_service", "type": "std_srvs/SetBool", "service": "/halcon"}'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

#test_data = '{"args": {"data": true}, "id": "service_request:/halcon:1", "service": "/halcon", "op": "call_service"}'



def create_response(id):
  res = '{ "op": "service_response",'
  res += '"id": ' + id
  res += '"service": "/halcon",'
  res += '"values": {"success": true, "message": "I got the call"},'
  res += '"result": true,'
  res += '}'
  return res

response= '{ "op": "service_response", "id": "service_request:/halcon:1", "service": "/halcon", "values": {"success": true, "message": "I got the call"}, "result": true}'

#



#s.send(message)
s.send(service_msgs)

while(True):
  data = s.recv(BUFFER_SIZE)
  print "received data:", data
  if (len(data) > 0):
    s.send(response)
    #s.send(create_response('"service_request:/halcon:1"'))


s.close()


