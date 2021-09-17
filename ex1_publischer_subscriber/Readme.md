# __ex1_publisher_subscriber__

## __Info__

In questo esercizio si va a creare una comunicazione tra due nodi sul topic `joint_position` in cui il publisher invia un messaggio custom contenente informazioni su 6 giunti di un robot, come ad esempio le informazioni relative alla loro posizione. Una volata inviata sul canale di comunicazione, ci sarà un subscriber che prenderà queste informazioni e le stamperà a video.

## __Start_up__

Per avviare la comunicazione tra i nodi creati avremo bisogno di utilizzare 2 comandi (tenendo conto che il nodo master sia già stato istanziato):

Avvio del publisher:

```bash
rosrun pub_sub pub_sub_talker
```

Avvio del subscriber:

```bash
rosrun pub_sub pub_sub_listener
```
