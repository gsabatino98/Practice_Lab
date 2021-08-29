# __smartsix_controller__

Il package `smartsix_controller` contiene al suo interno il nodo che prende la traiettoria che il manipolatore deve attuare e la invia al controller in ascolto sul canale.

Il tipo di pianificazione attuato è una pianificazione che sfrutta la funzione `'computeCartesianPath'` appartenente al interfaccia `MoveGroupInterface` messa a disposizione da MoveIt! .

La traiettoria viene ottenuta tramite uno script Matlab che genera un file `circular.traj`, ottenuta tramite l'interpolazione di 25 punti nel piano cartesiano.

## __Come avviare la demo__

### __Introduzione__

In questa demo permette di simulare, come già detto, una traiettoria circolare ottenuta con un `COMAU Smartsix`.

La traiettoria verrà visualizzata con l'utilizzo di `RViz`, mentre tramite l'utilizzio di `rqt_multiplot` è possibile monitorare i valori di posizione, velocità ed accelerazione ottenuti dai giunti che stanno attuando la traiettoria.

### __Comando__

Per avviare la demo basta avviare il launch file :

```bash
roslaunch smartsix_controller demo_start_up.launch
```

All'interno di questo file vengono richiamati altri 3 comandi che servono per:

* avviare il file `demo_gazebo.launch`, presente all'interno del pkg _'smartsix_moveit_config'_ , che avvia __RViz__ e __Gazebo__.
* avviare il nodo __smartsix_controller__ il file `smartsix_controller.launch`, presente all'interno del pkg_'smatsix_controller'_, che carica il file circular.traj e ne calcola a traiettoria che verrà poi pubblicata sul topic che si occuperà di simularla.
* avviare `rqt_multiplot`, sul quale dovrà essere caricato dall'utente il plot per visualizzare, durante la simulazione, i valori di posizione, velocità e accelerazione. Il plot è presente all'interno di questo package ed è `rqt_plot_planned_trajectory.xml`, __che dovrà essere inserito ed avviato prima di fare partire la simulazione su RViz__, altrimenti i nodi subscirber utilizzati da rqt non verranno instanziati e di conseguenza non riceveranno i risultati della traiettoria pubblicati dal nodo _smartsix_controller_.
