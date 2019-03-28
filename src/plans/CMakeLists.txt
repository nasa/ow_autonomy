# A simple Makefile for compiling and running the PLEXIL plans (.plp files) in
# this directory.


file( GLOB PLANS "${CMAKE_CURRENT_SOURCE_DIR}/*.plp" )

add_plexil_plans(europa_sol_0 ${PLANS})
