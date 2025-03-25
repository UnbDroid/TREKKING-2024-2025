# Define o componente com suas dependências
idf_component_register(
    SRCS "RobotProperties.cpp"            # Fontes do componente
    INCLUDE_DIRS "include"        # Diretórios de inclusão para o componente
    REQUIRES MotorDC      # Dependências públicas do componente       # Dependências privadas do componente
)

