---
title: Computação
---

## Arquitetura de Software

O firmware foi refatorado de uma arquitetura monolítica e procedural para um design modular e orientado a eventos. A transição utiliza o **State Pattern** para separar a lógica da navegação da implementação de hardware, permitindo maior flexibilidade e testabilidade.

### Máquina de Estados (FSM)

O sistema opera em cinco estados principais:

- **Initialization (Init):** Verificação de hardware (IMU, sensores) e inicialização de periféricos
- **Idle:** Estado padrão aguardando entrada do usuário
- **Calibration:** Sintonia de sensores de parede e offset do giroscópio
- **Run:** Estado ativo com navegação e exploração do labirinto
- **Error:** Estado de segurança para falhas críticas de hardware

### Abstração de Interface Humana (HMI)

A camada Interface atua como ponte entre o mundo físico (botões, chaves) e o lógico (eventos, objetivos). O sistema mapeia entradas para eventos semânticos, permitindo controle por:

- Botões físicos durante a competição
- Chaves DIP para configuração (estratégia, performance, perfil de risco)
- Controle remoto via Bluetooth através do módulo `micras_comm`

---

## Sistema de Navegação e Controle

O módulo `micras_nav` é o núcleo lógico responsável por percepção, planejamento de trajetória e execução de comandos motores. A refatoração migrou de uma abordagem reativa baseada em posição para uma arquitetura preditiva centrada em velocidade.

### Componentes Principais

1. **Odometria:** Estimação de posição/velocidade via sensor fusion (encoders rotativos + IMU)
2. **Mapa (Costmap):** Representação em grid 16×16 da estrutura do labirinto
3. **Planejamento:** Algoritmos para exploração e cálculo da rota ótima
4. **ActionQueuer:** Conversão de pontos de grid em ações motoras executáveis
5. **SpeedController:** Controle de velocidade com feed-forward + PID

### Percepção e Localização

**Odometria com Fusão de Sensores:**

- **Encoders:** Medição de rotação das rodas (resolução alta)
- **Giroscópio (IMU):** Velocidade angular precisa, especialmente importante durante derrapagem
- **Filtragem:** Butterworth filter suaviza ruído nas leituras de velocidade

**Estratégia Odométrica Local:**

- Em vez de rastrear posição global desde o início (acumula erro), o robô reseta sua referência ao iniciar cada ação
- **Detecção de Posts:** Pelos sensores IR identifica bordas de paredes em interseções, "travando" a estimativa à grade conhecida
- Limita o acúmulo de erro mesmo em corridas longas

### Mapeamento e Planejamento

**Costmap (Flood Fill - BFS):**

- Grid 16×16 armazena estado das paredes e camadas de custo
- Algoritmo BFS calcula distância mínima de cada célula para o alvo
- Atualização eficiente quando novas informações de parede são detectadas

**Exploração (BFS Frontier + Gradient Descent):**

- `get_next_bfs_goal`: Encontra a célula não explorada mais próxima
- Gradient descent: Segue o caminho de menor custo no Flood Fill
- Paredes virtuais: Detecta dead-ends automaticamente e as marca para evitar revisitas

**Rota Ótima (DFS com Backtracking):**

- Calcula o caminho mais rápido considerando a dinâmica do robô (não apenas número de células)
- Heurística com distância Manhattan para poda de ramos
- Resultado: rota que minimiza tempo de travessia, não apenas comprimento geométrico

### Planejamento de Movimento: Action Queue

O `ActionQueuer` traduz decisões de navegação em sequências de ações atômicas (movimentos, rotações).

**Tipos de Ações:**

- START, MOVE_FORWARD, TURN, SPIN, DIAGONAL, STOP

**Pipeline de Otimização (6 estágios):**

1. **Geração primitiva:** Conversão de célula-célula em ações básicas
2. **Injeção de diagonais:** Reconhece padrões em L e os converte em movimentos 45° (economia ~7% de tempo)
3. **Consolidação:** Ações consecutivas do mesmo tipo são mescladas (ex: 3 MOVE → 1 MOVE de 3× distância)
4. **Trimming de curvas:** Ajusta raios de curvatura para manter segurança em relação às paredes
5. **Perfis dinâmicos:** Assign aceleração/desaceleração específicas de cada fase
6. **Integração de curvas:** Transições suaves mantêm velocidade constante através de rotações

**Modo Exploração vs. Solve:**

- **Exploração:** Ações dinâmicas (1-2 à frente), dinâmica conservadora, seguimento de parede habilitado
- **Solve:** Rota pré-compilada, dinâmica agressiva (aceleração máxima, velocidades altas), seguimento desabilitado

### Controle de Movimento

**Perfis Cinemáticos:**

- **Movimentos lineares:** Perfil de velocidade trapezoidal (aceleração → velocidade constante → desaceleração)
- **Rotações:** Perfil triangular de aceleração angular (suave, sem jerk)
- Precisão de parada calculada com Torricelli: $v_f^2 = v_i^2 + 2a\Delta x$

**Controle de Velocidade:**

O sistema foi migrado de **controle de posição** para **controle de velocidade** para maior previsibilidade:

- Posição é implicitamente mantida pela integração dos comandos de velocidade precisos
- Dinâmica consistente e repetível
- Permite entrada/saída de curvas em velocidades exatas

**SpeedController (Cascata Feed-Forward + PID):**

$$u(t) = \underbrace{K_v \cdot v_{ref}(t) + K_a \cdot a_{ref}(t)}_{\text{Feed-Forward}} + \underbrace{K_p e(t) + K_i \int e(t) \, dt + K_d \frac{de}{dt}}_{\text{PID}}$$

- **Feed-Forward:** Calcula o comando motor necessário para atingir velocidade/aceleração desejadas *antes* de qualquer erro
- **PID:** Correção reativa baseada no erro de velocidade ($e = v_{ref} - v_{actual}$)
- Ganhos PID mais baixos (sistema mais suave) pois FF já faz a maior parte do trabalho

### Wall Following (Seguimento de Parede)

Proporciona correção lateral e angular em tempo real baseada em sensores IR:

- Mantém centralização do robô na passagem
- Post detection: Identifica bordas de paredes e reseta a odometria global para a grade
- Reduz erro acumulado durante movimentos longos

---

## Modos de Execução

O robô opera em três modos durante uma corrida:

### 1. Exploração

- **Objetivo:** Mapear o labirinto progredindo para o centro
- **Decisões:** Dinâmicas - Action Queue preenchida com 1-2 ações à frente
- **Dinâmica:** Conservadora (aceleração menor, velocidades moderadas)
- **Seguimento de parede:** Habilitado
- **Duração:** ~2-3 minutos

### 2. Retorno ao Início

- **Objetivo:** Voltar ao ponto de partida
- **Mesma lógica:** Dinamicamente replanejado, mas com costmap otimizado para o início
- **Duração:** ~1-2 minutos

### 3. Solve (Corrida de Velocidade)

- **Objetivo:** Executar a rota pré-computada mais rápida
- **Decisões:** Nenhuma - rota totalmente pré-compilada
- **Dinâmica:** Agressiva (aceleração máxima, velocidades altas, raios de curvatura reduzidos)
- **Seguimento de parede:** Desabilitado
- **Todas as otimizações:** Ativas (diagonais, consolidação, suavização de curvas)
- **Duração:** ~30-60 segundos
- **Repetibilidade:** Alta - mesma rota executada consistentemente

---

## Resumo de Algoritmos

| **Componente** | **Algoritmo** | **Benefício** |
|---|---|---|
| Odometria | Differential Drive + Butterworth | Sensor fusion suave, reduz ruído |
| Costmap | Flood Fill (BFS) | O(n), ótimo para grids uniformes |
| Exploração | BFS Frontier + Gradient Descent | Descobre labirinto minimizando backtracking |
| Rota Ótima | DFS + Heurística | Otimização de tempo real (não apenas células) |
| Sequenciação | Pipeline 6-estágios | Consolidação, diagonais, suavização de curvas |
| Perfis | Trapezoidal + Triangular | Parada precisa, aceleração sem jerk |
| Controle Motor | Feed-Forward + PID | Rastreamento preciso de velocidade |

---

## Loop de Controle

O sistema executa em **frequência fixa (1000 Hz, 1 ms por ciclo)**:

1. Atualizar odometria
2. Obter velocidades desejadas da ação atual
3. Verificar se a ação foi concluída
4. SpeedController computa comandos motores
5. Enviar PWM aos drivers dos motores
