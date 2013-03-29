test_sfly: ../../bin/test_sfly
	../../bin/test_sfly 1LoopDown/imu_1loopDown.txt 1LoopDown/vicon_1loopDown.txt > test_sfly_filter.txt && ./plot_test_sfly_filter.m

test_sfly_gdb: ../../bin/test_sfly
	gdb --args ../../bin/test_sfly 1LoopDown/imu_1loopDown.txt 1LoopDown/vicon_1loopDown.txt

test_sfly_valgrind: ../../bin/test_fly
	valgrind -v --track-origins=yes ../../bin/test_sfly 1LoopDown/imu_1loopDown.txt 1LoopDown/vicon_1loopDown.txt

../../bin/test_sfly: 
	$(MAKE) -C ../../build test_sfly

.PHONY: ../../bin/test_sfly
