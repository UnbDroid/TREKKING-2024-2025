# TREKKING2K24

## Descrição
Projeto de controle e automação desenvolvido pela equipe UnbDroid para a competição **TREKKING 2024**.

## Estrutura do Projeto
- **include/**: Arquivos de cabeçalho (.h). Declaração das classes, suas principais informações (variáveis e funções) e também armazenamento de informações relevantes, tais como a pinagem dos componentes.
- **lib/**: Bibliotecas externas utilizadas, de forma principal o Servo.h, e o MPU6050_light.
- **src/**: Código-fonte principal. Onde está a parte principal do código, com a descrição das funções das classes, criação dos objetos e desenvolvimento da lógica. Separados em aquivos referentes a cada classe, com "Robo.cpp" contendo a lógica principal de movimentação, e "main.cpp" o código final, com a rota a ser percorrida.
- **test/**: Scripts de teste e validação. Inutilizada.
- **V6_128_ncnn_model/**: Modelos pré-treinados em NCNN (modelo de otimização ideal para uso na Raspberry Pi 4).

## Requisitos
- Visual Studio Code com  a extensão PlatformIO
- Python 3.8+
- Bibliotecas Servo e MPU6050_light

## Como Usar
1. Clone o repositório.
2. Instale as dependências com PlatformIO.
3. Execute o código principal do Arduino na pasta `src`. Execute o código da Visão rodando o arquivo RetornoDeDistancia.py.
