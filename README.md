# TREKKING2K24

## Descrição
Projeto de robô autônomo off-road desenvolvido pela equipe Droid para a competição **TREKKING 2024**, na Robocore Experience 2024, na CPXP SP.

## Estrutura do Projeto
- **include/**: Arquivos de cabeçalho (.h). Declaração das classes, suas principais informações (variáveis e funções) e também armazenamento de informações relevantes, tais como a pinagem dos componentes. As classes são: **MotorDC**, que diz respeito a cada motor individualmente, **Volante**, que diz respeito ao servo motor que controla a direção do eixo de direção das rodas dianteiras, **Tempo**, que contém variáveis que gravam o tempo total percorrido, e o tempo entre cada iteração do código, **Robo**, que trata do robô como um todo, juntando as outras classes em uma maior, contendo as funções principais do robô em si. **Pinos.h** contém a pinagem adequada de cada componente do robô, e caso deseje usar em outros pinos, as variáveis devem ser alteradas de acordo.
- **lib/**: Bibliotecas externas utilizadas, de forma principal o *Servo.h*, e o *MPU6050_light*.
- **src/**: Código-fonte principal. Onde está a parte principal do código, com a descrição das funções das classes, criação dos objetos e desenvolvimento da lógica. Separados em aquivos referentes a cada classe, com *main.cpp* o código final, com a declaração dos objetos e a rota a ser percorrida.
- **test/**: Scripts de teste e validação. Inutilizada.
- **V6_128_ncnn_model/**: Modelo pré-treinado em *NCNN* (modelo de otimização ideal para uso na Raspberry Pi 4).
- **V6_128.onnx**: Modelo pré-treinado em *ONNX* (outro modelo de otimização, usar apenas se a instalação do NCNN não funcionar, ou o desempenho não estiver bom).
- **V6_128.pt**: Modelo pré-treinado base, na extensão padrão do *PyTorch*.
- **RetornoDeDistancia.py**: Arquivo principal da visão. É esse que você irá rodar para começar o código da visão. Para rodar, abra a pasta do repositório, depois ative sua venv, e digite ```python3 RetornoDeDistancia.py```.
- **convert_onnx.py**: Arquivo criado para converter modelos para ONNX (pode ser modificado para NCNN também, basta trocar onde está escrito "onnx" e colocar "ncnn").

## Requisitos
- Para enviar código pra o Arduino: Visual Studio Code com a extensão PlatformIO, Bibliotecas Servo e MPU6050_light
- Para visão: Python3.8+, RaspiOS Lite arm64-2024-03-15, torch=2.3.0, torchvision=0.xx.0

## Como Usar
1. Clone o repositório.
2. Caso não tenha feito ainda, instale o Visual Studio Code, e posteriormente o PlatformIO.
3. Acesse o código principal do Arduino nas pastas `src`, `include` e `lib`.
4. Envie o modelo desejado e o arquivo RetornoDeDistancia.py para o Raspberry Pi por meio do comando scp.
5. Na Raspberry Pi, entre na pasta onde está o modelo.
6. **Caso não tenha feito ainda (após primeira instalação não precisa repetir)**, execute o comando ```python3 -m venv venv_visao```, para criar a venv.
7. Após isso, execute o comando ```source "/caminho/ate/venv_visao/bin/activate"``` para ativar a venv (substitua o caminho pelo real).
8. **Caso não tenha feito ainda (após primeira instalação não precisa repetir)**, instale as dependências necessárias para rodar o código da visão:
```
pip install torch=2.3.0 torchvision=0.xx.0
pip install ultralytics
```
9. Execute o código da visão com o comando ```python3 RetornoDeDistancia.py```.
