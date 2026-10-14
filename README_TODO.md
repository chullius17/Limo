vedere se la sostituzione della motion mode è veramente utile 

Sì, se la modalità comunicata dal firmware corrisponde sempre alla configurazione meccanica del robot.
Dipende da cosa vuoi rimuovere:
- Tutto l’override: puoi tornare a switch (motion_mode_). Il driver seguirà sempre il telaio e il parametro motion_mode non servirà più.
- Solo il ripiego sul feedback: devi rendere obbligatoria una scelta valida tra 0 e 2, rifiutando -1 e gli altri valori all’avvio.
- Solo il blocco iniziale di lettura e stampa: la callback continuerà a leggere il parametro e scegliere la conversione come prima.
Con la situazione descritta per il tuo robot — meccanica Ackermann e firmware che riporta Mecanum — l’override serve ancora. Rimuoverlo farebbe usare la conversione Mecanum.

