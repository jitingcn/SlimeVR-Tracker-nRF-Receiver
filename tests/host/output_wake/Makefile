.PHONY: check clean

check:
	CC="$(CC)" python3 run.py
	CC="$(CC)" python3 run_cdc_startup.py

# Generated source and binaries are scoped to the runner's temporary directory.
clean:
