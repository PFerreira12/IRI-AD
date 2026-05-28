# Comparacao das Politicas de Atendimento

Teste realizado sem obstaculos dinamicos ativos (`DYNAMIC_ENVIRONMENT = False`).

Foram comparadas tres politicas:

- **FIFO**: atende os pedidos pela ordem de chegada.
- **NEAREST**: atende primeiro a mesa mais proxima do robo.
- **HYBRID**: combina tempo de espera e distancia.

## Ranking final

O ranking foi feito usando o **fair objective**:

```text
fair_objective = effective_wait_mean + 0.25 * effective_wait_max
```

Quanto menor o valor, melhor.

| Rank | Politica | Fair objective | Effective wait medio | Effective wait maximo | Wait medio | Pending medio |
|---:|---|---:|---:|---:|---:|---:|
| 1 | NEAREST | 163.183s | 86.416s | 307.070s | 34.739s | 189.768s |
| 2 | HYBRID | 178.566s | 118.934s | 238.530s | 117.970s | 121.730s |
| 3 | FIFO | 189.288s | 123.040s | 264.990s | 127.060s | 111.382s |

## Leitura rapida

### Melhor resultado global: NEAREST

A politica **NEAREST** teve o melhor resultado no ranking final.

O principal motivo e que reduziu muito o tempo medio de espera dos pedidos servidos:

```text
NEAREST wait medio = 34.739s
HYBRID  wait medio = 117.970s
FIFO    wait medio = 127.060s
```

Isto mostra que atender a mesa mais proxima torna o robo muito mais eficiente em deslocacoes.

### Trade-off do NEAREST

Apesar de ganhar no resultado global, o **NEAREST** teve mais pedidos pendentes:

```text
NEAREST pending count = 15
HYBRID  pending count = 10
FIFO    pending count = 10
```

Isto confirma o problema esperado: o NEAREST pode favorecer mesas proximas e deixar algumas mesas mais afastadas a espera durante mais tempo.

### HYBRID como compromisso

O **HYBRID** nao teve o melhor resultado global, mas reduziu o pior caso em relacao ao NEAREST:

```text
NEAREST effective wait maximo = 307.070s
HYBRID  effective wait maximo = 238.530s
```

Isto sugere que a abordagem hibrida pode ser interessante quando queremos evitar esperas extremas, mesmo que o tempo medio piore.

## Conclusao

Neste teste, a melhor politica foi **NEAREST**, porque minimizou bastante o tempo medio de espera dos pedidos servidos.

No entanto, a politica **HYBRID** mostrou-se mais equilibrada em termos de pior caso, sendo uma alternativa relevante se o objetivo for evitar que certas mesas fiquem demasiado tempo sem atendimento.

Assim, para eficiencia geral, a melhor escolha e **NEAREST**. Para maior equilibrio entre eficiencia e justica, vale a pena continuar a afinar a politica **HYBRID**.

