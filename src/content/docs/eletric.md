---
title: Elétrica
---

## Visão Geral

A eletrônica do Micras v2 foi desenvolvida com foco em miniaturização extrema e integridade de sinal, adotando o conceito de PCB-as-Chassis, onde a placa principal atua como elemento estrutural do robô.

Projetado no Altium Designer, o sistema é composto por três módulos interconectados, fabricados com processos híbridos (montagem industrial para componentes 0402 e soldagem manual para ICs críticos). Abaixo, você pode explorar interativamente os esquemáticos e layouts 3D de cada subsistema através do Altium 365 Viewer.

---

## Placa Principal

O núcleo do sistema. Esta PCB de 4 camadas utiliza um stackup otimizado (Signal-GND-Power-Signal) para garantir imunidade a ruídos eletromagnéticos. Ela abriga o microcontrolador STM32H7 (550 MHz), o sistema de potência que eleva a tensão da bateria 3S para 20V (para maior torque nos motores) e os drivers de atuação. Observe a ausência de componentes na face inferior (Bottom Layer), garantindo o selamento perfeito para o sistema de sucção a vácuo.

<iframe src="https://personal-viewer.365.altium.com/client/index.html?feature=embed&source=305CCBD5-F931-4366-BC04-BF67891CA224&activeView=PCB" width="1280" height="720" style="overflow:hidden;border:none;width:100%;height:720px;" scrolling="no" allowfullscreen="true" onload="window.top.scrollTo(0,0);"></iframe>

---

## Placa de Encoders

Módulo satélite responsável pela leitura precisa da velocidade e posição. Esta placa utiliza sensores magnéticos AS5047U e se encaixa perpendicularmente em slots fresados na placa principal, permitindo soldagem direta sem o uso de fios ou conectores propensos a falhas.

<iframe src="https://personal-viewer.365.altium.com/client/index.html?feature=embed&source=303E9A79-9798-4947-9D6B-57623CEBA378&activeView=PCB" width="1280" height="720" style="overflow:hidden;border:none;width:100%;height:720px;" scrolling="no" allowfullscreen="true" onload="window.top.scrollTo(0,0);"></iframe>

---

## Placa Bluetooth BLE

Módulo de comunicação sem fio baseado no chip HM-19. Este design utiliza a porta USB-C do robô de forma dual: além de permitir gravação via cabo, ela aceita este módulo plug-and-play que converte os pinos de dados em uma interface UART, habilitando a telemetria e o ajuste de parâmetros em tempo real via Bluetooth

<iframe src="https://personal-viewer.365.altium.com/client/index.html?feature=embed&source=3A771C40-5F45-40D0-BBB9-96463FF6FA8F&activeView=PCB" width="1280" height="720" style="overflow:hidden;border:none;width:100%;height:720px;" scrolling="no" allowfullscreen="true" onload="window.top.scrollTo(0,0);"></iframe>