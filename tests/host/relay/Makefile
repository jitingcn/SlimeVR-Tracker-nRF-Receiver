CC ?= cc
SRC := ../../../src/connection
CFLAGS := -std=c11 -Wall -Wextra -Werror -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -I$(SRC)
TMP := $(shell mktemp -d 2>/dev/null || echo /tmp/relay-host)

.PHONY: check clean
check:
	$(CC) $(CFLAGS) -DCONFIG_RELAY_COORD_CAPACITY=10 test_capacity.c $(SRC)/relay_coord.c -o $(TMP)/capacity
	$(TMP)/capacity
	CC="$(CC)" python3 run_capacity.py
	$(CC) $(CFLAGS) test_demand_coverage.c $(SRC)/relay_coord.c -o $(TMP)/coverage
	$(TMP)/coverage
	$(CC) $(CFLAGS) test_b2_deadline.c -o $(TMP)/b2_deadline
	$(TMP)/b2_deadline
	$(CC) $(CFLAGS) test_policy.c -o $(TMP)/policy
	$(TMP)/policy
	$(CC) $(CFLAGS) test_demand.c $(SRC)/relay_coord.c -o $(TMP)/demand
	$(TMP)/demand
	CC="$(CC)" python3 run_runtime_settings.py
	$(CC) $(CFLAGS) test_runtime_wire.c -o $(TMP)/runtime_wire
	$(TMP)/runtime_wire
	$(CC) $(CFLAGS) test_runtime_geometry.c $(SRC)/relay_coord.c -o $(TMP)/runtime_geometry
	$(TMP)/runtime_geometry
	CC="$(CC)" python3 run_plan_snapshot.py
	CC="$(CC)" python3 run_cedeea0.py
	CC="$(CC)" python3 run_snapshot_equivalence.py
	CC="$(CC)" python3 run_layout_range.py
	$(CC) $(CFLAGS) -DCONFIG_RELAY_LAYOUT_FOLLOW_RECEIVER=1 test_geometry_link.c geometry_provider.c -o $(TMP)/geometry_link
	$(TMP)/geometry_link
	$(CC) $(CFLAGS) -DRELAY_HOST_DIRECT_SLOTS=16 -DRELAY_HOST_REPAIR_SLOTS=3 test_narrow_pool.c $(SRC)/relay_coord.c -o $(TMP)/narrow_pool
	$(TMP)/narrow_pool
	$(CC) $(CFLAGS) test_layouts.c $(SRC)/relay_coord.c -o $(TMP)/layout0
	$(TMP)/layout0
	$(CC) $(CFLAGS) -DRELAY_HOST_DIRECT_SLOTS=16 -DRELAY_HOST_REPAIR_SLOTS=3 test_layouts.c $(SRC)/relay_coord.c -o $(TMP)/layout1
	$(TMP)/layout1
	python3 run_freeze_wait.py
	python3 run_rx_metadata.py
	python3 run_fatal.py
	python3 run_rtc_alloc.py
	$(CC) $(CFLAGS) test_rescue.c $(SRC)/relay_coord.c -o $(TMP)/test_rescue
	$(TMP)/test_rescue
	$(CC) $(CFLAGS) test_split.c $(SRC)/relay_coord.c -o $(TMP)/test_split
	$(TMP)/test_split
	$(CC) $(CFLAGS) -DRELAY_HOST_DIRECT_SLOTS=16 -DRELAY_HOST_REPAIR_SLOTS=3 test_split.c $(SRC)/relay_coord.c -o $(TMP)/test_split_full16
	$(TMP)/test_split_full16
	$(CC) $(CFLAGS) test_coord.c $(SRC)/relay_coord.c -o $(TMP)/test_coord
	$(TMP)/test_coord
	$(CC) $(CFLAGS) test_wire_v3.c -o $(TMP)/test_wire_v3
	$(TMP)/test_wire_v3
	$(CC) $(CFLAGS) test_fast_coord.c $(SRC)/relay_coord.c -o $(TMP)/test_fast_coord
	$(TMP)/test_fast_coord
	$(CC) $(CFLAGS) test_learn.c $(SRC)/relay_coord.c -o $(TMP)/test_learn
	$(TMP)/test_learn
	CC="$(CC)" python3 run_broadcast.py
	python3 check_protocol_sync.py
	CC="$(CC)" python3 run_hid.py
	CC="$(CC)" python3 run_deferred.py
	CC="$(CC)" python3 run_learning_snapshot.py
	CC="$(CC)" python3 run_early_receipt.py
	CC="$(CC)" python3 run_multiblock.py

clean:
	@:
