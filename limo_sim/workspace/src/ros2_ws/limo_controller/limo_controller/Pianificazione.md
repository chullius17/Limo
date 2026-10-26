allora abbiamo 9 azioni base 
Proviamo un piccolo insieme fisso di azioni: accelerare, mantenere o frenare, combinato con aumentare, mantenere o diminuire lo sterzo.

abbiamo velocità massima e accelerazione massima

abbiamo i limiti cinematici

abbiamo i limmiti critici che riguardano tutta la footprint

possiamo trasformare quelle azioni in funzioni di costo sull'input, quindi creeremo ad esempio 9 riferimenti di input per cui calcoleremo delle funzioni di costo con vari weight, magari settabili come parametro. combiniamo i weight alla costmap che useremo come costo per i vari stati del robot e a tutti i limiti che abbiamo prima. per ogni azione di base calcoliamo 3 riferimenti di input casuali, che probabilmente saranno costanti, tipo sterzate costanti, accelerazioni costanti oppure velocità fisse. e mi raccomando: con la footprint del robot dobbiamo finire dentro il tubo. cosa ne pensi di questo procedimento?
