# __ex2_tf__

## __Info__

In questo esercizio si crea un nodo che calcola e stampa la TF tra la `flangia` ed ogni `link` dal robot fanuc. I valori che verranno stampati saranno posizioni ed orientamenti relativi all' end-effector rispetto ad ogni link.

## __Start_up__

Prima di avviare il nodo che avvierà la comunicazione con il topic `/tf`, bisognerà istanziarlo. Per fare ciò bisogna lanciare il seguente comando:

```bash
roslaunch urdf_tutorial display.launch model:='$(find fanuc_description)/robot/m20ia.urdf'
```

Una volta fatto ciò basta avviare il nodo listener tramite il seguente comando:

```bash
rosrun m20ia_publisher m20ia_publisher_node
```
