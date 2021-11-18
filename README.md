# Otimização e Meta-Heurísticas 2019

Trabalhos apresentados na disciplina de Otimização e Meta-Heurísticas para o problema de roteamento de veículos. No caso, foram aplicados os métodos *Particle Swarm Optimization* e *Clustering Search* para solução do *Capacited Vehicle Routing Problem* (CVRP). O [primeiro trabalho](trabalhos/) consistiu na aplicação dos dois modelos. Enquanto isso, o [segundo](trabalhos/trabalho-2-ocmh.pdf) é caracterizado pela combinação dos dois métodos para melhoria dos resultados observados.

## *Clustering Search* (\*CS)

*Clustering Search* (\*CS) é uma meta-heurística que compreende a aplicação de clusterização em meio aos resultados dos otimizadores. A proposta dividi-se em quatro partes: meta-heurística de busca, clusterização iterativa, módulo de análise e a busca local. Assim, a técnica refere-se ao agrupamento de respostas similares resultantes de uma meta-heurística inicial para estabelercer processos de otimização independentes via busca local.

## *Particle Swarm Optimization* (PSO)

*Particle Swarm Optimization* (PSO) é uma meta-heurística baseada no comportamento de coletivo de animais. Este tipo de comportamento biológico é modelado através da *Swarm Intelligence*, subárea da Computação Evolutiva. A *Swarm Intelligence* caracteriza-se pela modelagem descentralizada de indivíduos de modo autoorganizado, seja ele natural ou artificial. Aplicado em geral para conjuntos de solução contínuos, a implementação foi adaptada aos conjuntos discretos, denominado *Discrete Particle Swarm Optimization* (DPSO) na literatura.