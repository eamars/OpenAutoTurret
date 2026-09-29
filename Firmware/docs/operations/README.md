# Operating cards

One card per operation, one hop from [`../README.md`](../README.md). Cards are procedure: commands,
what each command proves, and what it does not prove. Facts that justify a procedure (measured
limits, owner rulings, failure history) stay in [`../STATION_OPERATIONS.md`](../STATION_OPERATIONS.md)
and are linked, not copied — a number written in two places is a number that will disagree with
itself.

Every card follows the same skeleton, so that a reader under time pressure knows where to look:

1. **What this is for** — one sentence, and when *not* to use it.
2. **Where the work happens** — which machine does the compiling/moving/starting. This is the line
   people skip and then act on an assumption.
3. **The command** — exact, copy-pasteable, with the interpreter spelled out.
4. **What it proves** — the claim you may make after it succeeds.
5. **What it does not prove** — the claim you may not. Nearly every incident in this project's
   history came from a step being read as proving more than it does.
6. **When it fails** — the known failure modes, each with the reason, because a message without a
   reason makes the next person re-run the experiment.

Add a card by writing it here and adding one row to the table in `../README.md`; the tree checker
fails if a card exists that the map does not name, or is named and missing.
