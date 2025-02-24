idf_component_register(SRCS "src/ps4.c" "src/ps4_l2cap.c" "src/ps4_parser.c" "src/ps4_spp.c"
                       INCLUDE_DIRS "src" "src/osi" "src/stack"
                      PRIV_REQUIRES bt
                     )

