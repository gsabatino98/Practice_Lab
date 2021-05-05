# **ex1_publisher_subscriber**

In questo esercizio sono stati creati due pkg in in cui ci implementa la comunicazione tra due nodi di un robot, tramite l'invio di messaggi riguardanti la posizione dei suoi giunti. I pkg sono:

* publisher_subscriber
* publisher_subscriber_msgs

***

## **publisher_subscriber**

In questo pkg per l'invio dei dati è stata utilizzata la libreria **trajectory_msgs/jointTrajectory.h** che permette di creare un messaggio contenente:

* **header:** di tipo std_msgs/Header
* **joint_names:** di tipo String[]
* **points:** di tipo trajectory_msgs/jointTrajectoryPoints []

## file.cpp utilizzati: publisher_subscriber

I file.cpp utilizzati si trovano della directory "/src" e sono un publisher ed un subscriber che comunincano tra loro attraverso il topic ***"robo_test_sr"***. Ad ognuno di questi file.cpp è stato assegnato un nodo (creati nel file publisher_subscriber/CMakeLists.txt), e sono:

* ***publisher.cpp :*** il quale ad ogni loop crea un serie di messaggi riguardanti i 6 giunti del robot i cui dati vengono inseriti tramite l'utilizzo di due for annidati. Questo eseguibile è stato poi asseganto, nel file CMakeList.txt, al nodo ***publisher_subscriber_sender***.
* ***subscriber.cpp :*** il quale tramite una funzione di callback resta in che il publisher invii i messaggi al topic per poi poterli leggere. Questo eseguibile è stato poi asseganto, nel file CMakeList.txt, al nodo ***publisher_subscriber_receiver***.
  
***

## **publisher_subscriber_msgs**

in questo pkg è stato utilizzato un messaggio custom per l'invio dei dati. I file.msg creati si trovano della directory ***"publisher_subscriber_msg/msg"*** e sono:

* **diagnosticRobot.msg :** il quale è composto dalle variabili timestamp (int64 indicante la data in timestamp in cuiè stata eseguita la diagnostica del giunto), position (float64[] vettore indicante la posizione nel piano 3D del giunto), velocity (float64), acceleration (float64) e status_sensors (bool idicante lo stato dei sensori del giunto)
* **kointInfo.msg :** il quale è compopsto dalle variabili joint_names (string[]), diagnostic (publisher_subscriber_msgs/diagnosticRobot[]) e joint_temps (sensor_msgs/Temperature[], un messaggio standard appartenente alla libreria di ros indicante le temperatura e la varianza dei sensori del giunto)
  
## file.cpp utilizzati: publisher_subscriber_msgs

I file.cpp utilizzati si trovano della directory "/src" e sono un publisher ed un subscriber che comunincano tra loro attraverso il topic ***"robo_custom_msgs"***. Ad ognuno di questi file.cpp è stato assegnato un nodo (creati nel file publisher_subscriber_msg/CMakeLists.txt), e sono:

* ***test_pub_custom.cpp :*** il quale ad ogni loop crea un serie di messaggi riguardanti i 4 giunti del robot i cui dati vengono inseriti tramite l'utilizzo di due for annidati. Questo eseguibile è stato poi asseganto, nel file CMakeList.txt, al nodo ***publisher_subscriber_msg_customSender***
* ***test_rec_custom.cpp :*** il quale tramite una funzione di callback resta in che il publisher invii i messaggi al topic per poi poterli leggere. Questo eseguibile è stato poi asseganto, nel file CMakeList.txt, al nodo ***publisher_subscriber_msg_customReceiver***
