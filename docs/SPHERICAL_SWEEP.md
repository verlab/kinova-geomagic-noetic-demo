# Varredura de poses do Gen3 de 7 DoF

Teste preliminar de movimentação, executado uma única vez. O script
`kinova_spherical_sweep.py` usa a mesma API de waypoints angulares e a mesma
política de duração de `kinova_wrist_wave.py`. O controle é por juntas, inclusive
na configuração vertical. Não há aquisição de sensores nesta sequência.

## Sequência

J1, J2, J3, J4 e J7 ficam em 0° em todos os alvos. A numeração é de 1 a 7;
J5 e J6 correspondem aos índices 4 e 5 do vetor enviado à API.

| Etapa | J6 | J5, na ordem de execução |
| --- | --- | --- |
| Inicial | 0° | 0° (todas as juntas em zero) |
| Anel 1 | 120° | 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315° |
| Anel 2 | 90° | 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315° |
| Anel 3 | 60° | 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315° |
| Anel 4 | 30° | 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315° |
| Anel 5 | 0° | 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315° |
| Final | 0° | 360° ≡ 0°, encerrando a execução |

São **42 alvos com parada**. Cada transição entre anéis envia J5 e J6 juntos em
um único waypoint: de `(J5=315°, J6=θ)` para `(J5=0°, J6=θ−30°)`. Não há parada
intermediária para mover uma junta de cada vez. A passagem de J5 por zero é
expressa no comando como o ângulo equivalente mais próximo do feedback (por
exemplo, 315° → 360°). O feedback acumula deslocamentos circulares de J5 e
interrompe a sequência se detectar movimento reverso maior que 1° ou avanço
maior que o esperado mais 1°. Essa tolerância é uma escolha de monitoramento do
script, não uma margem recomendada pela Kinova. O comportamento de passagem
por zero ainda precisa ser confirmado no firmware do braço.

O desenho é uma série de anéis discretos com transições entre inclinações;
não é uma interpolação cartesiana que mantenha o TCP exatamente sobre uma
esfera. Na inclinação zero, J5 continua a varredura solicitada mesmo quando a
posição do TCP não muda. Ao final J5 terá completado cinco voltas positivas;
0° é uma equivalência angular e não significa desfazer essas voltas em cabos
ou acessórios externos.

## Limites, velocidade e parada

Referência: [guia do usuário Gen3 r12.0](EN-UG-014-Gen3-user-guide-r12.0.pdf),
**página impressa 109, tabela 38** (página 117 do PDF): J5 é contínua e J6 tem
limites de posição de **−120,3° a +120,3°**. A página 157 descreve a atuação dos
limites de software antes dos limites físicos. Não foi encontrada uma margem
angular adicional recomendada pelo fabricante para este ensaio.

O alvo inicial de **120°** fica 0,3° abaixo do limite de software. Isso permite
mantê-lo como alvo nominal, mas não certifica ausência de colisões: a trena,
suporte, cabos e ambiente precisam ter espaço durante toda a varredura. Cada
waypoint é submetido a `ValidateWaypointList` antes da execução; uma rejeição
persistente interrompe o programa, sem reduzir o ângulo silenciosamente. Para
escolher explicitamente um primeiro anel mais afastado do limite, existe
`first_tilt_deg` em `(90, 120]`; por exemplo, 115° é uma escolha do operador,
**não uma recomendação do fabricante**. Os anéis seguintes permanecem em
90°, 60°, 30° e 0°.

O guia, página 106, explica o uso do modo angular para sair de configurações
singulares. Por isso a varredura utiliza alvos articulares, sem cinemática
inversa cartesiana ou MoveIt.

- A política de movimento é a do exemplo existente: tentar duração 0 s e
  aumentar em passos de 1 s até a primeira validação aceita, abaixo de 30 s.
  Não são alterados limites de velocidade ou aceleração do controlador.
  O exemplo original não define uma velocidade fixa em °/s; trajetórias de
  amplitudes diferentes podem ter durações e velocidades efetivas diferentes.
- Cada ação contém **um waypoint**, com `use_optimal_blending=false`. O programa
  aguarda `ACTION_END` da ação correspondente, e então exige feedback de todas
  as sete juntas dentro de **0,1°** do alvo e com velocidade de módulo
  **≤0,5°/s** por **1 s contínuo** (`hold_sec=1.0`, como no exemplo).
- A contagem de espera recomeça se as juntas saírem dessas tolerâncias. Esses
  limiares verificam repouso articular aproximado; não medem vibração da trena.
- Feedback ausente por mais de 0,5 s, dados inválidos, ação abortada/pausada,
  timeout de ação (45 s) ou de estabilização (10 s mais a espera) interrompem
  a sequência e enviam `Stop`. `Ctrl+C` também envia `Stop`.
- Se já estiver no alvo e em repouso, o comando redundante é dispensado, mas a
  espera inteira é cumprida. Não há repetição automática do ensaio.

## Selecionar RViz ou braço real

O parâmetro **`execution_mode`** seleciona a execução:

| Valor | Comportamento |
| --- | --- |
| `rviz` (padrão) | Anima o modelo no RViz; não instancia clientes nem assinaturas Kortex. |
| `hardware` | Executa no braço real e abre o RViz para acompanhar o feedback. |

A prévia utiliza a mesma lista de 42 poses, com transições simultâneas de J5/J6,
passagem positiva por 360° e espera de 1 s em cada alvo. Inicia no zero virtual.
A interpolação articular é cúbica, com velocidade zero nas extremidades, e dura
**2 s por movimento** (`visual_move_sec`). Esse tempo é apenas de visualização:
as durações reais continuam sendo selecionadas pela validação do controlador.
A prévia não calcula dinâmica ou colisões. Ao terminar, mantém a pose final
visível até `Ctrl+C`, sem repetir a sequência.

O modelo, as posições das juntas e os tópicos TF da prévia ficam no namespace
`/spherical_sweep_preview`, separados dos tópicos do driver real. Não execute
outro publicador de juntas dentro desse namespace durante a prévia.

No diretório `kinova-geomagic-noetic-demo`, com a imagem
`kinova-geomagic-noetic:demo` disponível, monte o pacote local para usar as
alterações sem reconstruir a imagem. **Apenas RViz:**

```bash
docker compose run --rm \
  -v "$(pwd)/catkin_ws/src/kinova_geomagic_demo:/catkin_ws/src/kinova_geomagic_demo:ro" \
  demo roslaunch kinova_geomagic_demo demo_kinova_spherical_sweep.launch \
  execution_mode:=rviz
```

**Braço real com RViz** (este comando movimenta o hardware):

```bash
docker compose run --rm \
  -v "$(pwd)/catkin_ws/src/kinova_geomagic_demo:/catkin_ws/src/kinova_geomagic_demo:ro" \
  demo roslaunch kinova_geomagic_demo demo_kinova_spherical_sweep.launch \
  execution_mode:=hardware robot_ip:=192.168.1.10
```

O módulo de visão Kinova está habilitado por padrão (`vision:=true`), conforme
a configuração deste braço. Para um braço sem esse módulo, use `vision:=false`.
Ambos os modos utilizam a configuração gráfica do Docker descrita no README.
`start_rviz:=false` desativa somente a janela gráfica; não muda o modo de execução.
No hardware, o launch abre uma configuração própria com `RobotModel` habilitado,
referencial fixo `base_link` e `robot_description` resolvido no namespace
`robot_name`. Isso também funciona com `start_driver:=false` quando o driver
correspondente já está ativo; não depende da configuração salva do RViz.

No modo `hardware`, `start_driver:=false` permite utilizar um driver já ativo;
`robot_name` deve corresponder ao namespace desse driver. Esse parâmetro não é
um modo de simulação. No modo `rviz`, o driver nunca é iniciado, independentemente
do valor de `start_driver`. Valores inválidos de `execution_mode` são rejeitados.

Se reconstruir a imagem, o script e os arquivos de launch/RViz já serão incluídos,
e o volume pode ser omitido. A reconstrução requer os submódulos e arquivos de
fabricante descritos no README.

## Verificação sem hardware

```bash
python3 -m unittest discover -s tests -p 'test_spherical_sweep.py' -v
```

Os testes usam mensagens e serviços simulados em Python para verificar a ordem,
paradas, transições simultâneas, retorno angular a zero, rejeições, feedback,
timeouts e interrupção. Não substituem validação no Docker/ROS nem ensaio físico.
