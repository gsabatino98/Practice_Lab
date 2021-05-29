# file.msg creati

I file.msg creati si trovano della directory ***"publisher_subscriber_msg/msg"*** e sono:

* **diagnosticRobot.msg :** il quale è composto dalle variabili timestamp (int64 indicante la data in timestamp in cuiè stata eseguita la diagnostica del giunto), position (float64[] vettore indicante la posizione nel piano 3D del giunto), velocity (float64), acceleration (float64) e status_sensors (bool idicante lo stato dei sensori del giunto).
* **jointInfo.msg :** il quale è compopsto dalle variabili joint_names (string[]), diagnostic (publisher_subscriber_msgs/diagnosticRobot[]) e joint_temps (sensor_msgs/Temperature[], un messaggio standard appartenente alla libreria di ros indicante le temperatura e la varianza dei sensori del giunto).
