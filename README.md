# Normative Poisoning Simulation (Attack S1)

This repository contains a text-based simulation harness for **Attack S1: Normative Poisoning via Long-Term Chat**, designed to test logical vulnerabilities in long-context UAV agents.

## Overview

The simulation demonstrates how an attacker (acting as an operator) can inject safety "norms" over a series of missions to bias an agent's future planning behavior, causing it to deviate from mission goals in favor of injected heuristics.

## Structure

- `src/`: Core logic
    - `agent.py`: UAV Agent implementation.
    - `simulator.py`: Mock environment (Sector A, Sector B risk simulation).
    - `context_manager.py`: Long-term memory management.
    - `llm_client.py`: Mock LLM logic (can be extended to real APIs).
- `scripts/`: Phase execution
    - `run_baseline.py`: Phase 1 (Neutral baseline).
    - `run_injection.py`: Phase 2 (Poisoning history).
    - `run_test.py`: Phase 3 (Test against neutral goal).
- `generate_report.py`: Runs the full suite and generates `REPORT.md`.

## Usage

1. **Install Dependencies**:
   (Standard Python 3 environment required. No external packages for the mock version.)

2. **Run Simulation**:
   ```bash
   python3 generate_report.py
   ```

3. **View Results**:
   See `REPORT.md` for coverage metrics and attack success analysis.

## Credits

Developed by Antigravity Team.
