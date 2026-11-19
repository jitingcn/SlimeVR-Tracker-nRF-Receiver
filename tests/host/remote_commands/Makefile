.PHONY: check

check:
	CC="$(CC)" python3 run.py
	CC="$(CC)" python3 run_channel_migration.py
	CC="$(CC)" python3 run_channel_completion.py
	CC="$(CC)" python3 run_policy_publication.py
