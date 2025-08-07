
# SmartCharging-S3E

A MuJoCo-based robotic simulation for semantic reasoning tasks involving battery sorting and dishwasher loading. Developed as part of the **236502 – Artificial Intelligence Project** at the Technion's CLAIR Lab.

## 🔧 Project Overview

This project includes two simulation domains:

### 1. **Battery Sorting & Charging Station**
- Robot sorts batteries based on:
  - **Charge level**
  - **Compatibility with charger**
  - **Physical damage**
- Semantic predicates:
  - `IsCharged(battery)`
  - `IsCharging(battery)`
  - `IsDamaged(battery)`
  - `ShouldBeDiscarded(battery)`
  - `IsChargerFreeFor(battery)`

### 2. **Dishwasher Loading**
- Robot places kitchenware in the correct slots.
- Semantic predicates:
  - `IsCorrectSlot(object, slot)`
  - `IsStable(object)`
  - `IsFragile(object)`
  - `HasSpace()`

Each domain supports structured state extraction and a task `success_score()` metric to evaluate semantic correctness.

---

## 📁 Repository Structure

- `sim_env.py` – Wrapper for MuJoCo environment.
- `battery_class.py` – Logic for battery tracking, charging, damage detection.
- `semantic_questions.py` – Semantic predicate evaluation & state extraction.
- `*.xml` – Object and scene configuration for MuJoCo.
- `test_*.py` – Test files for individual predicates and evaluation logic.
- `README.md` – This file.

---

## 🚀 Getting Started

### Prerequisites

- Python 3.10 or 3.11
- MuJoCo physics engine
- Install requirements:

```bash
pip install -r requirements.txt
```

### Clone the repository

```bash
git clone https://github.com/adamassi/SmartCharging-S3E
cd SmartCharging-S3E
```

### Run the simulation

**Windows**:

```bash
py -3.10 -m venv venv
.venv\Scripts\Activate
pip install -r requirements.txt
```

**Mac/Linux**:

```bash
python3.10 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Run simulation script:
```bash
mjpython -m <script_name>.py
```

---

## ✅ Testing

Run semantic and domain tests:
- `test_is_charged.py`
- `test_is_damaged.py`
- `test_should_be_discarded.py`
- `test_is_charger_free_for.py`
- `test_get_state.py`
- `test_success_score.py`

Each test checks semantic question logic and state evaluation.

---

## 📊 Evaluation

Each domain defines:
- `get_state()` – Returns a list of boolean answers and their predicates.
- `success_score(state, goal)` – Scores task completion from 0–100 based on semantic correctness.

---

## 👥 Contributors

- **Firas Hilu**
- **Adam Assi**

---

## 🏫 Course Info

- Course: 236502 – Artificial Intelligence
- Lab: CLAIR Lab, Technion – Israel Institute of Technology
- Instructor: Matan Argaman
