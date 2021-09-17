# __ex3_kinematics__

## __Info__

In questo esercizio si va a calcolare la cinematica diretta ed inversa del manipolatore `fanuc m20ia`.

### __Forward Kinematics__

Per quanto riguarda il calcolo della cinematica diretta, i nodi client e server comunicheranno tramite un `service` custom `fk_service.srv`. Il client invierà una richiesta contenente un `robot_state` con le informazioni relative al manipolatore, mentre il server una volta ricevuta la richiesta calcolerà la cinematica diretta per tutta la catena cinemtaica, inviando come risposta le pose per ogni link al client.

### __Inverse Kinematics__

Per quanto riguarda il calcolo della cinematica inversa, i nodi client e server comunicheranno tramite un `action` custom `ik_action.action`. Il client invierà un `Goal` contenente un `robot_state` con le informazioni relative al manipolatore ed la posa dell' end-effector su cui calcolare la cinematica inversa. Il server calcolerà la cinematica inversa ed a ogni soluzione trovata invierà un `Feedback` al client contenente un `robot_state` che in questo caso rappresenta il robot nella configurazione trovata. Una volta finita la ricerca il server invierà un `Result` contenente tutte le soluzioni trovata.

## __Start_up__

### __Forward Kinematics__

Per avviare la comunicazione tra ServiceClient e ServiceServer, bisognerà lanciare i seguenti comandi:

```bash
rosrun forward_kinematics forward_kinematics_server_node
```

```bash
roslaunch forward_kinematics demo.launch 
```

### __Inverse_Kinematics__

Per avviare la comunicazione tra ActionClient e ActionServer, bisognerà lanciare i seguenti comandi:

```bash
roslaunch inverse_kinematics demo.launch 
```

```bash
rosrun inverse_kinemtatics forward_kinematics_server_node
```
