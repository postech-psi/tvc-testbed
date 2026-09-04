"""
verify -- does the simulator do what we think it does?
================================================================================
This is VERIFICATION, not validation: it checks internal consistency, never
agreement with the real vehicle. Nothing in this project has been compared
against flight data -- see docs/6-CREDIBILITY.md.

    scenarios.py  Five closed-loop runs, each asserting a threshold chosen to
                  catch one specific structural mistake.
    golden.py     Freezes the analytic simulator's numbers so a refactor that
                  claims to move none can be checked instead of trusted.

Both are also imported by tests/, so a scenario is defined once and run from
two places.
"""
