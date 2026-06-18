# progetto-identificazione_stima

Sensori scelti per le misure:
- Accelerometro asse X ($y_1$): Posizionato sul corpo dell'elicottero, misura la proiezione della gravità sul proprio asse. Non misura l'angolo, ma una sua funzione trigonometrica:
$y_1 = -g \sin(\alpha) + v_1$.
Modello commerciale: STMicroelectronics ISM330DHCX oppure l'onnipresente MPU-6050 / MPU-9250.Corrispondenza fisica: L'equazione y_meas(1) = -p.g * sin(alpha) sfrutta l'accelerometro 
come inclinometro. Quando l'elicottero è fermo o si muove lentamente, l'accelerometro misura la scomposizione della gravità lungo il suo asse. Conoscendo l'accelerazione misurata e l'accelerazione di gravità $g$, si risale matematicamente ad $\alpha$.  

- Magnetometro asse asse Y ($y_2$): Un sensore di campo magnetico (bussola) solidale al corpo. Assumendo il Nord magnetico allineato con l'inerziale X, la componente Y misuratadipende sia dal pitch che dallo yaw:
$y_2 = M \cos(\alpha) \sin(\beta) + v_2$ (con $M$ modulo del campo, normalizzabile a 1).
Modello commerciale: AK8963 (spesso integrato dentro l'MPU9250) o LIS3MDL.Corrispondenza fisica: L'equazione y_meas(2) = cos(alpha) * sin(beta) descrive come la bussolamagnetica legge il campo magnetico terrestre. Ruotando sul piano orizzontale ($\beta$), la componente del Nord magnetico letta dal sensore varia seguendo il seno e il coseno degli angoli di assetto.
 
STATO: $x = [\alpha, \dot{\alpha}, \beta, \dot{\beta}]^T$
- $\alpha$: Pitch
- $\beta$: Yaw

Generazione del Dataset (gen_dataset.m)
Questo script simula la dinamica non lineare dell'elicottero 2DoF per generare il "Ground Truth" (la traiettoria reale) e le misurazioni rumorose dei sensori. Rappresenta l'impianto fisico virtuale su cui verranno testati i filtri.
- Dinamica e Ingressi: Integra le equazioni del moto non lineari (con inerzia di yaw $J_\beta(\alpha)$ dipendente dal pitch) partendo da una condizione iniziale nota. Utilizza segnali di ingresso sinusoidali sovrapposti per garantire la persistenza dell'eccitazione e sollecitare le dinamiche transitorie.
- Integrazione Numerica: Attualmente utilizza l'integrazione di Eulero in avanti. Nota di sviluppo: è prevista una potenziale transizione al metodo di Runge-Kutta (RK4) per allineare la fedeltà numerica dello script al solutore standard ode4 di Simulink ed evitare derive numeriche a lungo termine.
- Modello Sensori: Genera le misure indirette di accelerometro e magnetometro aggiungendo rumore gaussiano bianco (definito dalle varianze reali dei datasheet commerciali). I dati completi e le matrici di covarianza ottime ($Q_{true}$ e $R_{true}$) vengono salvati nel file dataset_elicottero.mat. 

Stimatori Online: EKF vs UKF (ekf_ukf.m)
Questo script implementa e mette a confronto due tecniche di filtraggio stocastico per la stima online dello stato $x = [\alpha, \dot{\alpha}, \beta, \dot{\beta}]^T$, leggendo i dati generati dal simulatore. Entrambi i filtri rispettano il principio di causalità temporale utilizzando l'ingresso $u_{k-1}$ per il passo di predizione.
- Extended Kalman Filter (EKF): Sfrutta un'approssimazione al primo ordine. Per gestire la forte non-linearità parametrica senza incorrere in errori algebrici, la matrice Jacobiana di transizione dello stato ($F$) viene calcolata numericamente per perturbazione locale, mentre la Jacobiana di misura ($H$) è derivata analiticamente.
- Unscented Kalman Filter (UKF): Evita la linearizzazione propagando un set di Sigma Points attraverso le equazioni non lineari originali, catturando in modo più accurato media e covarianza in presenza dell'inerzia variabile.
- Analisi dei Risultati: Lo script produce grafici avanzati per la validazione degli stimatori, includendo il confronto delle traiettorie, l'analisi di consistenza statistica (con i bound di incertezza a $\pm 3\sigma$), la stazionarietà delle innovazioni (residui) e il calcolo quantitativo del Root Mean Square Error (RMSE). 

Traiettoria Regolarizzata Offline (smoother_rts.m)
Questo script completa il modulo di stima implementando un algoritmo di regolarizzazione offline: lo smoother di Rauch-Tung-Striebel (RTS). L'obiettivo è ottenere la miglior stima possibile a posteriori dell'intera traiettoria.
- Ricorsione Backward: Partendo dall'ultimo istante di tempo della stima EKF ($t=N$), l'algoritmo procede a ritroso, ricalcolando gli stati lisciati. Utilizza il guadagno di smoothing per iniettare l'informazione "futura" nelle stime passate, impiegando l'operatore di divisione matriciale (/) per garantire la massima stabilità numerica delle matrici di covarianza.
- Analisi e Validazione: Genera plot di confronto diretti tra il filtro EKF (forward) e lo smoother RTS (backward). Dimostra graficamente il teorema della riduzione dell'incertezza (covariance shrinkage), mostrando come i limiti a $\pm 3\sigma$ dell'RTS siano strettamente inferiori o uguali a quelli dell'EKF, per poi quantificare il miglioramento globale tramite il confronto degli RMSE.

I risultati risultano molto molto simili
1. Regime dei Piccoli Angoli (Quasi-Linearità)I segnali eccitanti che hai impostato nel generatore del dataset (0.5 * sin(2*t) e 0.3 * cos(1.5*t)) sono relativamente dolci per le masse in gioco. Questo fa sì che l'elicottero oscilli in un range di pochi decimi di radiante.In questo regime (piccoli angoli), sappiamo che $\sin(\alpha) \approx \alpha$ e $\cos(\alpha) \approx 1$. Pertanto, le fortissime nonlinearità del sistema, come l'inerzia variabile $J_{\beta}(\alpha)$ dipendente dal $\sin^2(\alpha)$, non si manifestano in modo aggressivo. Il sistema si sta comportando in modo quasi-lineare, e in un sistema lineare (o debolmente non lineare) l'approssimazione al primo ordine dell'EKF è praticamente perfetta.
2. Passo di Campionamento Altissimo ($dt = 0.01$ s)Stai campionando e linearizzando il sistema a 100 Hz. Questo significa che l'EKF calcola una nuova Jacobiana (la tangente alla curva dinamica) ogni 10 millisecondi. In un lasso di tempo così infinitesimo, l'errore di linearizzazione non ha letteralmente il tempo di crescere. Prima che la curva reale si allontani dalla tangente dell'EKF, arriva la misura successiva a correggere il tiro.
3. Il Jacobiano Numerico è "Infallibile"La scelta di calcolare la matrice $F$ tramite perturbazione numerica (delta = 1e-5) è stata un'arma a doppio taglio per questo confronto. Molti EKF falliscono perché chi li programma calcola male le derivate analitiche a mano (errori di segno, termini dimenticati). Il tuo EKF calcola una Jacobiana locale rasente alla perfezione della macchina, rubando all'UKF parte del suo vantaggio strutturale.
4. L'Esperimento del "Gemello Identico"Entrambi i filtri conoscono la fisica esatta del sistema. I parametri $m, l, J_{\alpha}$, ecc., usati nel filtro sono identici a quelli del simulatore. Non c'è incertezza parametrica non modellata che possa mandare in crisi l'EKF.

Provando a "rompere" il sistema, ho messo per esempio (in gen_dataset.m): x_hat_0 = [1,5; 0; -1.5; 0] e dt = 0.1. I risultati venivano diversi tra loro e, ovviamente, peggioravano.


SIMULINK

ATTENZIONE: 
- Solutore del modello settato su ode1 (Eulero) in modo che al momento corrisponda al metodo usato nei codici. In caso si cambiasse metodo nei codici, cambiare il solutore con ode4 (Runge-Kutta).
- Come dimensione del fixed step è stato messo 0.01 (= dt nel file gen_dataset) in caso di modifica nel codice, va modificato anache nel solutore.
- Fare sempre caso alle condizioni iniziali di tutti gli integratori e dei vari blocchi, al momento corrispondono a quelle dei codici, in caso di modifica di uno deve essere modificato anche l'altro.

Modello di Simulazione Real-Time (sim_elicottero_filtri.slx)
Questo schema Simulink implementa l'architettura completa del sistema, unendo l'impianto fisico (Ground Truth) e gli stimatori di stato (Filtri) in un ambiente di esecuzione online sincrono (campionato a frequenza $dt$).
- Solutore Numerico: Configurato rigorosamente a passo fisso (Fixed-step) utilizzando il metodo di integrazione di Eulero in avanti (ode1). Potenzialmente cambiabile in ode4 come scritto sopra
- Ground Truth e Sensori: La dinamica dell'elicottero e i modelli matematici dell'IMU sono incapsulati in blocchi MATLAB Function. Il rumore di processo viene iniettato sulle accelerazioni (prima dell'integratore cinematico), mentre il rumore di misura modella le varianze commerciali direttamente sulle uscite ideali dei sensori.
- Filtri Embedded (EKF e UKF): Gli stimatori operano in tempo reale. Per gestire la natura ricorsiva degli algoritmi di Kalman, i blocchi di stima utilizzano cicli di feedback dotati di ritardi unitari (Unit Delay, $1/z$), che fungono da memoria per propagare lo stato stimato $\hat{x}_{k-1}$ e la matrice di covarianza $P_{k-1}$ all'istante successivo.
- Causalità e Data Logging: L'ingresso $u$ viene ritardato di un passo ($u_{k-1}$) per rispettare la causalità fisica della stima. I risultati dell'esecuzione (inclusi stati, covarianze, innovazioni e Jacobiane analitiche) vengono esportati in formato array nel Workspace di MATLAB tramite appositi blocchi To Workspace per l'analisi offline.


Estrazione Dati e Analisi Grafica (plot_risultati_simulink.m)
Script di post-processing dedicato all'estrazione, formattazione e validazione dei dati generati dalla simulazione Simulink (salvati di default nella struttura globale out).
- Allineamento Dati: Si occupa di estrarre e trasporre le matrici tridimensionali e i vettori loggati dai blocchi To Workspace per renderli compatibili con l'ambiente di plotting.
- Validazione Statistica: Genera automaticamente i grafici di confronto temporale tra Ground Truth e le stime (EKF vs UKF). Implementa un rigoroso test di consistenza sovrapponendo l'errore di stima ai bound di incertezza statistica $\pm 3\sigma$ ricavati dalle diagonali delle matrici di covarianza $P_{k|k}$.
- Analisi di Performance: Include l'analisi di stazionarietà delle innovazioni (residui) dei sensori e sintetizza le performance globali dei filtri attraverso un grafico a barre del Root Mean Square Error (RMSE).

Smoothing Offline RTS su Dati Simulink (sim_RTS.m)
Implementazione del regolarizzatore di Rauch-Tung-Striebel (RTS) per ottenere la stima a posteriori della traiettoria ottimale, rielaborando i dati salvati in uscita dalla simulazione Simulink.
- Integrazione Simulink-MATLAB: Sfrutta i log avanzati esportati dal blocco EKF in Simulink (specificamente lo stato predetto x_pred_EKF, la covarianza a priori P_pred_EKF e la Jacobiana di transizione F_EKF).
- Ricorsione Backward: L'algoritmo parte dall'istante finale $t=N$ e "riavvolge" il tempo fino a $t=1$, calcolando il guadagno di smoothing $C_k$ tramite divisione matriciale (/) per garantire il condizionamento numerico.
- Dimostrazione Teorica: Lo script è progettato per dimostrare analiticamente il teorema della riduzione dell'incertezza (covariance shrinkage). Produce grafici che evidenziano come i limiti di confidenza $\pm 3\sigma$ dell'RTS siano sistematicamente e strettamente inferiori a quelli del filtro forward (EKF), validando matematicamente la superiorità della stima offline.

PARAMETRI CON CUI GIOCARE
Guida al Tuning: I Parametri di Progetto
Il comportamento del sistema e le performance degli stimatori possono essere alterati modificando specifici parametri. Tutti i parametri principali sono centralizzati nello script di inizializzazione (gen_dataset.m) per essere caricati nel Workspace prima di lanciare la simulazione.

A. Parametri Fisici e "Ground Truth" (La Realtà)
Modificare questi parametri cambia il modo in cui l'impianto fisico si comporta e la qualità dei sensori scelti. Si trovano nello Script di Inizializzazione.
- Ingressi di Controllo (u / Segnali Sinusoidali):
    Cosa fanno: Rappresentano le forze applicate dai rotori. Cambiarne l'ampiezza o la frequenza altera quanto l'elicottero oscilla.
    Effetto: Segnali molto aggressivi spingono il sistema in regimi fortemente non lineari (grandi angoli), mettendo in crisi l'EKF e facendo risaltare la superiorità dell'UKF.Dove si modificano: In Simulink, direttamente nei blocchi Sine Wave all'ingresso dell'impianto (Ampiezza, Frequenza, Fase).
- Passo di Campionamento (dt):
    Cosa fa: Definisce la frequenza di esecuzione del solutore e dei filtri (es. 0.01 = 100 Hz).
    Effetto: Aumentare il dt (es. 0.05 o 0.1) degrada l'accuratezza dell'integrazione numerica e delle derivate dell'EKF, causando ritardi di fase maggiori e perdita di tracking nei transienti.
- Rumori Reali (std_w_alpha_dot, std_w_beta_dot, std_v_acc, std_v_mag):
    Cosa fanno: Rappresentano la fisica del disturbo (es. raffiche di vento sulle velocità) e la reale imprecisione costruttiva dei sensori IMU.
    Dove si applicano: Vengono usati per calcolare la Noise Power nei blocchi Band-Limited White Noise in Simulink (Formula: $\text{Varianza} \times dt$).

B. Parametri dei Filtri (Il Tuning degli Stimatori)
Questi sono i parametri "software" che gli algoritmi di Kalman usano per fidarsi più o meno del modello matematico rispetto ai sensori. Si modificano nello Script di Inizializzazione.
- Matrice di Covarianza del Rumore di Processo (Q):
    Cosa fa: Quantifica quanta "sfiducia" ha il filtro nelle proprie equazioni interne (es. approssimazioni di Eulero, attriti non modellati).
    Effetto: * Aumentare Q (Tuning Robusto): Rende il filtro più reattivo e veloce a seguire i cambiamenti bruschi, allargando i limiti di confidenza a $\pm 3\sigma$ (utile per riassorbire problemi di inconsistenza statistica e ritardi di fase). Tuttavia, rende la stima più "tremolante" e rumorosa.
        Diminuire Q: Rende la traiettoria stimata molto liscia, ma lenta e soggetta a ritardi se la dinamica reale è molto nervosa.
- Matrice di Covarianza del Rumore di Misura (R):
    Cosa fa: Indica al filtro quanto deve fidarsi dei sensori IMU.
    Effetto: Se R viene impostata artificialmente più alta rispetto alla reale varianza dei sensori (es. si dichiara $0.05$ per l'accelerometro anche se quello reale ha $0.01$), il filtro si fiderà meno delle misure e agirà da "passa-basso", addolcendo la curva ma rischiando di ignorare variazioni reali veloci.
- Stima e Incertezza Iniziale (x_hat_0, P_0):
    Cosa fanno: Definiscono la condizione di partenza a $t=0$ per i blocchi Unit Delay di EKF e UKF.
    Effetto: Mettere un errore iniziale deliberato (es. far partire l'elicottero fermo, ma dire al filtro che l'angolo di pitch è 85 gradi) serve per fare gli "stress test". Un P_0 elevato aiuterà il filtro a convergere rapidamente al valore vero ignorando la falsa partenza iniziale.


# Confronto tra l'implementazione fornita a lezione e l'implementazione sviluppata per il progetto
# Codici EKF E UKF

## Differenze nell'implementazione dell'Extended Kalman Filter (EKF)

Negli esempi forniti a lezione i Jacobiani del modello dinamico e del modello di misura vengono ricavati analiticamente e implementati esplicitamente all'interno del codice. Questa scelta è particolarmente efficace per modelli di dimensioni ridotte e con espressioni matematiche relativamente semplici.

Nel progetto si è invece scelto di calcolare numericamente il Jacobiano della dinamica mediante differenze finite. Per ogni stato viene introdotta una piccola perturbazione e viene stimata la derivata locale della funzione di transizione. Questa soluzione rende il codice più generale e facilmente adattabile a modelli complessi, evitando il calcolo manuale di derivate spesso lunghe e soggette a errori.

Il Jacobiano del modello di misura è stato invece mantenuto in forma analitica, poiché la sua espressione risulta sufficientemente compatta e facilmente derivabile.

Dal punto di vista concettuale il funzionamento dell'EKF rimane invariato: il filtro continua a utilizzare una linearizzazione locale del modello non lineare per propagare la covarianza e aggiornare la stima.

---

## Differenze nell'implementazione dell'Unscented Kalman Filter (UKF)

La differenza più significativa tra il codice fornito a lezione e quello sviluppato per il progetto riguarda la gestione del rumore di processo.

Nel codice didattico viene utilizzata una formulazione di tipo **Augmented UKF**. In questo approccio il vettore di stato viene esteso includendo esplicitamente le variabili di rumore di processo. I sigma points vengono quindi generati nello spazio aumentato stato-rumore e successivamente propagati attraverso il modello non lineare.

Questa formulazione consente di rappresentare in maniera molto accurata gli effetti del rumore, ma comporta un aumento della dimensione del problema e quindi del numero di sigma points da propagare.

Nel progetto è stata invece adottata una formulazione **Additive Noise UKF**, assumendo che il rumore di processo entri additivamente nella dinamica. In questo caso i sigma points vengono generati esclusivamente sullo stato e la matrice di covarianza del rumore di processo viene aggiunta successivamente alla covarianza predetta.

Questa scelta riduce il costo computazionale, semplifica l'implementazione e risulta adeguata per il modello considerato, mantenendo prestazioni equivalenti nei casi in cui il rumore possa essere trattato come additivo.

---

## Parametri della trasformazione Unscented

Un'ulteriore differenza riguarda i parametri utilizzati nella Unscented Transform.

Negli esempi forniti a lezione viene utilizzato:

[
\alpha = 1,\quad \beta = 2,\quad \kappa = 0
]

Questa scelta produce sigma points relativamente dispersi attorno alla media e consente di esplorare in maniera più ampia le non linearità del modello.

Nel progetto si è scelto invece:

[
\alpha = 10^{-3},\quad \beta = 2,\quad \kappa = 0
]

che rappresenta una configurazione molto diffusa in letteratura. I sigma points risultano maggiormente concentrati attorno alla stima corrente e consentono una migliore approssimazione locale quando il sistema opera in prossimità del punto di linearizzazione.

## Codici RTS

Non ci sono grosse differenze, le uniche preseni riguardano i grafici e l'uso che si fa di RTS per la validazione.