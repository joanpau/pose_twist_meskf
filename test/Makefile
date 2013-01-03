simple_test:
	../bin/simple_test  > simple_test_filter.txt && ./plot_simple_test_filter.m

simple_test_gdb:
	gdb --args ../bin/simple_test

simple_test_valgrind:
	valgrind -v --track-origins=yes ../bin/simple_test

sfly_test:
	../bin/sfly_test 1LoopDown/imu_1loopDown.txt 1LoopDown/vicon_1loopDown.txt > sfly_test_filter.txt && ./plot_sfly_test_filter.m

sfly_test_gdb:
	gdb --args ../bin/sfly_test 1LoopDown/imu_1loopDown.txt 1LoopDown/vicon_1loopDown.txt

sfly_test_valgrind:
	valgrind -v --track-origins=yes ../bin/sfly_test 1LoopDown/imu_1loopDown.txt 1LoopDown/vicon_1loopDown.txt

