idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES robot server
)
# By default, the main component requires all components, but I'm writing
# it explicitly to make it clear
