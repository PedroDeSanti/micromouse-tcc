---
title: Computação
---

## Arquitetura de Software

O firmware foi desenvolvido em **C++ Bare Metal**, mantendo a robusta arquitetura em camadas já estabelecida na versão anterior (Micras v1). O padrão **Hardware Proxy** foi fundamental para o sucesso do projeto: ao desacoplar rigidamente a lógica de navegação das especificidades do hardware, **não houve necessidade de portar código**.

Graças a esse isolamento, os novos algoritmos de alto nível (como o *ActionQueuer* e o controle de velocidade) puderam ser desenvolvidos e validados diretamente na plataforma antiga (v1), sem exigir qualquer alteração nas camadas de baixo nível.

### Máquina de Estados Finita (FSM)

Para superar as limitações de lógicas centralizadas, o controle foi refatorado utilizando o **State Pattern**. O sistema opera em estados discretos e isolados:

- **Initialization (Init):** Autoteste de hardware e configuração de periféricos.
- **Idle:** Estado de baixo consumo aguardando comandos.
- **Calibration:** Rotina isolada para normalização dos sensores IR e giroscópio.
- **Run:** Estado ativo onde residem os algoritmos de navegação (Exploração ou Speed Run).
- **Error:** *Trap state* de segurança para falhas críticas (ex: detecção de colisão).

### Interface e Telemetria

A camada de Interface abstrai as entradas físicas em eventos semânticos (ex: `Event::EXPLORE`), permitindo que a estratégia seja definida via chaves DIP ou injetada remotamente.

O sistema de comunicação (`micras_comm`) implementa um **Pool de Variáveis Seriais**, permitindo monitoramento e ajuste de constantes (PID, velocidade) em tempo real via Bluetooth, sem necessidade de recompilação.

---

## Navegação e Inteligência

O módulo `micras_nav` orquestra a percepção e o planejamento. A grande inovação desta versão é a transição de uma odometria global propensa a *drift* para uma **Odometria Local**, onde a posição é recalculada a cada ação discreta.

### Localização e Percepção

* **Odometria Local:** Em vez de rastrear coordenadas absolutas (X, Y) indefinidamente, o robô zera sua referência relativa ao iniciar cada movimento (ex: avançar 1 célula). Isso isola o erro acumulado a segmentos individuais.
* **Fusão de Sensores:** Integração dos encoders magnéticos com o giroscópio (IMU) para manter a orientação precisa mesmo em caso de derrapagem das rodas.
* **Correção de Posts:** O sistema detecta as bordas das paredes (posts) através da derivada do sinal dos sensores IR, usando esses marcos físicos para "zerar" o erro longitudinal ao entrar em uma nova célula.

### Mapeamento e Planejamento

O robô utiliza representação em grafo (Grid 16x16) e algoritmos distintos para cada fase:

1.  **Exploração (Flood Fill / BFS):** Utiliza *Breadth-First Search* para criar um mapa de custos (gradiente) e identificar a célula não visitada mais próxima, garantindo exploração sistemática.
2.  **Otimização de Dead-Ends:** Detecta becos sem saída e insere paredes virtuais no mapa, impedindo que o planejador perca tempo revisitando essas áreas.
3.  **Speed Run (DFS + Pruning):** Para a corrida final, utiliza *Depth-First Search* com poda heurística. Diferente do BFS, este algoritmo minimiza o **tempo estimado** (considerando aceleração e curvas), e não apenas a distância geométrica.

---

## Controle e Movimento

A execução física das trajetórias é gerenciada pelo `ActionQueuer` e pelo `SpeedController`, que convertem o caminho abstrato em tensão nos motores.

### Pipeline de Ações

O `ActionQueuer` processa a rota em etapas de otimização:

1.  **Diagonalização:** Identifica padrões em "L" e os converte em movimentos diagonais (45°), reduzindo a distância percorrida.
2.  **Fusão:** Mescla movimentos lineares consecutivos para permitir aceleração até a velocidade máxima.
3.  **Trimming (Curvas Contínuas):** Calcula a distância exata antes e depois de uma esquina para iniciar a curva sem parar o robô.

### Dinâmica de Curvas

Para realizar curvas em alta velocidade, o sistema utiliza uma aproximação numérica baseada em integrais de Fresnel. Isso permite calcular a **velocidade angular máxima** permitida para qualquer raio de curva, garantindo que a aceleração centrípeta se mantenha dentro dos limites de atrito dos pneus.

### Controlador Feed-Forward

O controle de velocidade abandonou a abordagem puramente reativa (PID) por uma arquitetura híbrida **Feed-Forward**:

$$u(t) = \underbrace{K_v \cdot v_{ref}(t) + K_a \cdot a_{ref}(t)}_{\text{Preditivo (Modelo Físico)}} + \underbrace{\text{PID}(e)}_{\text{Corretivo}}$$

* **Preditivo:** Calcula a tensão necessária baseada no modelo físico do motor (inércia e força contra-eletromotriz).
* **Corretivo:** O PID atua apenas sobre pequenos erros e distúrbios, resultando em uma resposta muito mais rápida e estável.
