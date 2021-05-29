# file.cpp utilizzati: publisher_subscriber_msgs

I file.cpp utilizzati si trovano della directory "/src" e sono un publisher ed un subscriber che comunincano tra loro attraverso il topic ***"robo_custom_msgs"***. Ad ognuno di questi file.cpp è stato assegnato un nodo (creati nel file publisher_subscriber_msg/CMakeLists.txt), e sono:

* ***test_pub_custom.cpp :*** il quale ad ogni loop crea un serie di messaggi riguardanti i 4 giunti del robot i cui dati vengono inseriti tramite l'utilizzo di due for annidati. Questo eseguibile è stato poi asseganto, nel file CMakeList.txt, al nodo ***publisher_subscriber_msg_customSender***
* ***test_rec_custom.cpp :*** il quale tramite una funzione di callback resta in che il publisher invii i messaggi al topic per poi poterli leggere. Questo eseguibile è stato poi asseganto, nel file CMakeList.txt, al nodo ***publisher_subscriber_msg_customReceiver***
