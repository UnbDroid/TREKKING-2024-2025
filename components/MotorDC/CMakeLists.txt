# Define o componente com suas dependências
idf_component_register(
    SRCS "MotorDC.cpp"            # Fontes do componente
    INCLUDE_DIRS "include"        # Diretórios de inclusão para o componente
    REQUIRES driver esp_timer      # Dependências públicas do componente
    PRIV_REQUIRES PinConfig       # Dependências privadas do componente
)

