# A simple Makefile for compiling and running the PLEXIL plans (.plp files) in
# this directory.

PLANS = OceanWorldMission Downlink Image DownlinkImage ImageSampleSite \
        SampleAnalysis Stub

PLP = $(PLANS:%=%.plp)

PLE = $(PLANS:%=%.ple)

# An intermediate representation generated from .ple, deleted automatically by
# default.
EPX = $(PLANS:%=%.epx)

PLX = $(PLANS:%=%.plx)

#LOG = $(PLP:%=%.log)


%.ple: %.plp OceanWorldDefs.h
    ${PLEXIL_HOME}/scripts/plexilpp $<

%.plx: %.ple
    ${PLEXIL_HOME}/scripts/plexilc $<

default: $(PLE) $(PLX)

run:
    plexilexec -p OceanWorldMission.plx -c ow-config.xml

clean:
    @ rm -f $(PLX) $(PLE) $(EPX)
