# A simple Makefile for compiling and running the PLEXIL plans (.plp files) in
# this directory.

set(PLANS
  OceanWorldMission
  Downlink
  Image
  DownlinkImage
  ImageSampleSite
  SampleAnalysis
  Stub)

  for_each()
add_plexil_plans(europa_sol_0 ${PLANS})
