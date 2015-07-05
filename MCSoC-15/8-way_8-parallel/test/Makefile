
# ----- SRCS and TARGET setting ----- #
# T    : src file name you want to test
# PRE  : test bench prefix (Write test bench file like tb_hoge.v)
# POST : filename extension
T=core
PRE=tb_
POST=.v

SRCS=$(PRE)$(T)$(POST)
TARGET=$(PRE)$(T)

# ----- Log files setting ----- #
# LOG  : simulation result (text, Verilog)
# VCD  : simulation result (wave, Verilog)
LOG=log.txt
VCD=uut.vcd

# ----- iverilog setting ----- #
IV=iverilog -I ../src/ -o $(TARGET) $(SRCS)
# IV=iverilog -o $(TARGET) $(SRCS)

# ----- VCS setting ----- #
VCS=vcs -full64 -v2005 +incdir+../src/ -o $(TARGET) $(SRCS)

# ----- gtkwave setting ----- #
GTK=gtkwave $(VCD) &


iverilog:
	$(IV)

vcs:
	$(VCS)

run:
	./$(TARGET)

dump:
	./$(TARGET) > $(LOG)

# view:
# 	$(GTK)

clean:
	rm -rf $(TARGET) $(VCD) $(LOG) csrc $(TARGET).daidir ucli.key

