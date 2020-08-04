import paho.mqtt.client as mqtt
import time

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("Summer/Team2/Statistics/Jamahl")
    #client.subscribe("Summer/Test2/Camera")
    #client.subscribe("Summer/Test2/Rover/StopSignal")
    


    
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set("zkrbbakj", "tDWwUUrBl3lH")
client.connect("m16.cloudmqtt.com", 15663, 60)


# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
topic_name_1 = "Summer/Test2/Camera"
topic_name_2 = "Summer/Test2/Rover/StopSignal"

rover_status = "MOVING"
color = "WHITE"
position = "60"

sequenceNumber = 0
var = 1

while var == 1:
    client.loop_start()
    time.sleep(0.5)
    client.publish(topic_name_2,"{\"Topic\":\"%s\", \"Sequence Number\":\"%d\", \"Rover Status\":\"%s\"}" % (topic_name_2, sequenceNumber, rover_status ), 0, False)
    time.sleep(0.5)
    client.publish(topic_name_1,"{\"Topic\":\"%s\", \"Sequence Number\":\"%d\", \"Color\":\"%s\", \"Position\":\"%s\"}" % (topic_name_1, sequenceNumber, color, position ), 0, False)
    sequenceNumber+=1    
    #time.sleep(0.5)
    client.loop_stop()

#client.loop_forever()

