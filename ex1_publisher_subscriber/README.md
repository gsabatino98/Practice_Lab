# **ex1_publisher_subscriber**

In questo esercizio sono stati creati due pkg in in cui ci implementa la comunicazione tra due nodi di un robot, tramite l'invio di messaggi riguardanti la posizione dei suoi giunti. I pkg sono:

* publisher_subscriber
* publisher_subscriber_msgs
* publisher_subscriber_custom

***

## **publisher_subscriber**

In questo pkg per l'invio dei dati è stata utilizzata la libreria **trajectory_msgs/jointTrajectory.h** che permette di creare un messaggio contenente:

* **header:** di tipo std_msgs/Header.
* **joint_names:** di tipo String[].
* **points:** di tipo trajectory_msgs/jointTrajectoryPoints [].
  
***

## **publisher_subscriber_msgs**

Questo pkg è stato usato per l'inserimento di messaggi di tipo custom.

***

## **publisher_subscriber_custom**

Questo pkg utilizza le librerie custom create nel pkg **publisher_subscriber_msgs**, per far comunicare i due nodi.
