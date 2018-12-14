# A simple Makefile for compiling, running, and testing the PLEXIL plans in this
# directory (.plp files).

PLP = Downlink Image OceanWorldMission SampleAnalysis Stub

PLE = $(PLP:%=%.ple)

PLX = $(PLP:%=%.plx)

#LOG = $(PLP:%=%.log)

# TODO: move plexilpp to plexil-4
%.ple: %.plp OceanWorldMission.h
	./plexilpp $<

%.plx: %.ple
	${PLEXIL_HOME}/scripts/plexilc $<

# %.log: %.plx
#	@ ./logrun $<

default: $(PLE) $(PLX)

# Test all scenarios.  Compile scenarios and run them first if needed.
# test: default $(LOG)
# 	@ for i in $(UNIT); do \
#         echo Verifying $$i ... ; \
# 		./check $$i ; \
# 	  done

# retest: dust test

# For developers: Bless the current output, making it the new benchmark.
# bless: default $(LOG)
# 	@ for i in $(UNIT); do \
# 		./make-valid $$i ; \
# 	  done

# dust:
# 	@ rm -f $(LOG)

clean:
	@ rm -f $(PLX) $(PLE)
