SHELL = /bin/sh
DTC := dtc
DTBINDS := /opt/source/dtb-4.19-ti/include/
DTCFLAGS := -Wno-unit_address_vs_reg -Wno-pci_bridge -Wno-simple_bus_reg -Wno-avoid_unnecessary_addr_size -Wno-alias_paths -Wno-unique_unit_address

SRCS = $(wildcard *.dts)
BINARIES = $(patsubst %.dts,%.dtbo,$(SRCS))

build: $(BINARIES)

$(OUT):
	mkdir -p $(OUT)

%.dtbo: %.dts
	$(CPP) -Wp,-MD,.$@.d.pre.tmp -nostdinc -I$(DTINCLUDES) -undef -D__DTS__ -x assembler-with-cpp -o .$@.dts.tmp $<
	$(DTC) -O dtb -o $@ -b 0 -@ $(DTCFLAGS) -d .$@.d.dtc.tmp .$@.dts.tmp
	cat .$@.d.pre.tmp .$@.d.dtc.tmp > .$@.d

install:
	cp $(BINARIES) /lib/firmware

clean:
	rm -f .*.tmp .*.d *.dtbo
