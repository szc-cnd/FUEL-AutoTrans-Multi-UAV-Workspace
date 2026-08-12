# Planner Crash and Mode-B First-Door Design

## Scope

This change has exactly two independent parts:

1. Stop `exploration_node` from crashing during rapid replanning.
2. Make entry-detection mode B select the first structurally valid door.

Mode A must remain behaviorally and structurally unchanged. The two modes must not share new ranking logic.

## Crash fix

The latest run ended with `exploration_node` exit code `-11`. Immediately before the crash, a published trajectory started a detached visualization thread, then tracking safety forced a replan. The detached thread reads the current trajectory and exploration visualization containers while replanning can replace the same data.

Remove the detached visualization thread. Run `visualize()` synchronously after publishing and committing the new trajectory, while the ROS single-threaded callback still owns the planning data. Do not introduce locks, another worker, or changes to planning behavior.

## Mode-B first-door selection

Keep all existing mode-B structural acceptance checks, including door width, post support, free opening, alignment, and through-path checks. After those checks accept candidates, mode B selects the candidate with the smallest forward `progress`.

Only candidates on effectively the same door plane may use the existing structural score as a tie-breaker. The same-plane tolerance is `0.10 m` in forward progress. A farther candidate must not replace a nearer candidate merely because it has greater progress, open depth, or accumulated score.

Implement this inside the mode-B detection path. Do not change `shouldReplaceDoorCandidate()` or any mode-A candidate path.

The existing three-cycle temporal confirmation remains unchanged.

## Verification

- Add a focused test proving that mode B keeps a nearer accepted door over a farther higher-score candidate.
- Add a focused test proving that candidates within `0.10 m` may still use score as the tie-breaker.
- Verify the source no longer creates or detaches a visualization thread.
- Build the affected ROS packages and run their available tests.
- Confirm mode-A selection code and launch mode choice are unchanged.
