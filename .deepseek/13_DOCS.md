# §13 · Documentation

**Governs:** `docs/`, `reproduction/REPRODUCTION_REPORT.md`, the root `README.md`
**Milestone:** M10
**Done when:** A9 holds, the three derivations compile, and no `TODO`/`TBD` remains.

The documentation is a deliverable of this repository, not a wrapper around it. A reviewer decides
whether to read the code based on `docs/README_math.md` and the README's Limitations section.

---

## §13.1 `docs/README_math.md`

The readable summary; the derivations carry the proofs. Keep every symbol identical to the code
and to the `.tex` files — `gamma`, `omega`, `N`, `N_CBF`, `Omega`, `W`.

| Section | Content |
| --- | --- |
| Notation | state space, safe set, class-K conventions |
| 1 | DT-CBF definition, invariance, the linear specialisation, why `γ ≤ 1` is not optional |
| 2 | the MPC-CBF OCP as implemented, why the condition is imposed along the prediction, MPC-DC contrast |
| 3 | relaxed decay and the `ωγ ≤ 1` proof |
| 4 | **feasibility — the honest section** |
| 5 | tube MPC-CBF: RPI set, tightening, the guarantee |
| 6 | **what this repository does not prove** |

§4 and §6 are load-bearing. §6 must reproduce [01_OVERVIEW.md §1.6](01_OVERVIEW.md) — recursive
feasibility, the local RPI certificate, obstacle-prediction error, estimation error, multi-agent
scope. A reviewer who finds a limitation you did not list stops trusting the ones you did.

Write §4 so it states plainly that DCBF constraints do not confer recursive feasibility, names the
two standard sufficient conditions, says which one is implemented (neither), and presents the
empirical study as measurement. Any wording that implies a guarantee is a bug — fix the wording,
not the reader's expectations.

## §13.2 `docs/derivations/*.tex`

Three files, all compiled by CI with `-halt-on-error`. A `.tex` that has not compiled since March
is not a derivation.

| File | Must contain |
| --- | --- |
| `discrete_time_cbf.tex` | the DT-CBF definition; **the invariance proof written out** (two lines, and they are the entire safety argument); the linear specialisation and the `(1−γ)^k` envelope; necessity of `γ ≤ 1`; the squared-distance barrier and its gradient; the relaxed-decay safety proposition; the relation to the continuous-time condition |
| `mpc_cbf_unified_formulation.tex` | the full OCP; **the specialisation table** (`ω≡1` → ACC 2021, `ω` free → CDC 2021, no decay row → MPC-DC, `N=1` → CBF-QP); why the decay row is imposed along the prediction; choice of `N_CBF` with measured solve-time scaling; an honest feasibility section; the numerical formulation and why `DISCRETE` is mandatory |
| `tube_mpc_robustness.tex` | assumptions, and where the nonlinear models violate them; the RPI lemma and Raković's construction with the closed form for the diagonal test case; constraint tightening; **the barrier-tightening derivation of [08_TUBE.md §8.4](08_TUBE.md) in full, with the sign argument spelled out**; the robust safety theorem with every hypothesis named; conservatism; limitations |

The specialisation table in the second file is the contribution of that note and the reason the
codebase has one solver class rather than four. Write it as a table, not prose.

The sign argument in the third is the one place where an error would produce a tightening that
looks reasonable and is not. Show that the dropped `‖Pe‖²` term is non-negative in `h(z+e)` and
therefore that discarding it is conservative.

**Worked numbers in the derivations must be cross-checked against the notebooks' simulations**
before either is called done. When they disagree, one of them is wrong and you do not yet know
which.

## §13.3 `docs/PRIOR_ART.md`

An annotated bibliography, 3–6 sentences per entry: the idea, what it assumes, what it guarantees,
how it relates to this repository. Sections: continuous-time foundations, discrete-time CBFs,
predictive safety filters, robust and tube MPC, robust/stochastic CBFs, multi-layered safety on
hardware, distributed MPC-CBF.

**Every citation must be checked against the actual publication** — authors, venue, year, title
(rule 4, §0). An invented DOI or a misattributed year in a document whose whole purpose is to
demonstrate literature command is worse than omitting the entry.

The closing section, "the gap this thesis fills", is the paragraph a reviewer will actually read.
Three parts:

1. **What is solved.** Single-agent MPC-CBF, reproduced here; the tube extension handles bounded
   disturbance for one agent.
2. **What is not.** Every formulation cited assumes a fixed agent set and fixed constraint
   topology. At a split, merge or morph event the constraint set itself changes, the state jumps
   to a set-valued reset, and neither the CBF invariance argument nor the tube's RPI certificate
   survives the discontinuity. Robustness results assume disturbances inside a fixed `W`, not a
   change in problem dimension.
3. **The claim.** Transition-viable distributed MPC-CBF, with per-agent solvers of exactly the
   kind implemented here. Name the concrete deliverable.

Write it so the delta is locatable in one read. No hedging, no "future work will explore".

## §13.4 Keeping the documents consistent

Four places state the mathematics: `docs/README_math.md`, the three `.tex` files, the code
comments, and this specification. They drift.

The rule from [00_RULES.md](00_RULES.md) applies here too: **when you find a disagreement, fix
both in the same commit.** In particular, if you change the tightening formula, the DCBF row, or
the `ωγ ≤ 1` bound, grep for all four.

## §13.5 `reproduction/REPRODUCTION_REPORT.md`

The credential the repository exists to earn. Fill every `TBD` from notebook output.

**Never hand-type a number; paste the emitted table** (rule 4, §0). The notebooks print their
summary tables as markdown for exactly this reason.

| Section | Notes |
| --- | --- |
| Environment | date, commit, OS, compiler, **acados tag**, CasADi/NumPy/SciPy versions, CPU |
| Per-paper figure tables | one row per figure: reproduced?, our value, paper value, tolerance, notebook cell |
| Quantitative claims | one row per claim from [12_ANALYSIS.md §12.1–§12.2](12_ANALYSIS.md) |
| **Deviations** | one row per deviation, with a reason |
| Baselines for downstream citation | the numbers `transition-viable-swarm` and the planned CDC 2027 paper will cite |

**The Deviations rows are the most valuable part of the document.** A reproduction that claims
everything matched is not credible; a reproduction that says "Fig. 4's control profile differs
because the paper does not state the terminal weight; we used `Qf = 10Q`, which changes peak
acceleration by 12 %" is.

A `TBD` remaining in this file blocks the release gate ([15_ROADMAP.md §18](15_ROADMAP.md)).

## §13.6 The root `README.md`

The five-minute document. Order matters:

1. One-line tagline and status badges — **badges only once the workflow is actually green**.
2. What it is, in three sentences, including the three layers.
3. The three GIFs ([12_ANALYSIS.md §12.6](12_ANALYSIS.md)).
4. Quick start that works verbatim on a clean machine: acados install, codegen, build, one demo.
   Test it from a clean clone before every release; a README quick-start that does not work is
   the most common defect in research repositories.
5. Results table — **every row names its hardware and git SHA**, and every number is reproducible
   from `analysis/` or the test suite.
6. Reproductions: link `REPRODUCTION_REPORT.md`, summarise what matched in one table.
7. **Limitations** — complete and specific, mirroring §1.6. Not a disclaimer paragraph; a list.
8. Citation (`CITATION.cff`) and the pointer to the thesis work this feeds.

The Limitations section is not a weakness to be minimised. In a portfolio repository it is the
section that demonstrates the author knows what the method does, and its absence is what a
reviewer notices.

## §13.7 Where the author's original specification lives

[INFO.md](INFO.md) in this directory is the author's portfolio specification for this repository —
**read-only**. It fixes the target file structure and the demo deliverables.
[01_OVERVIEW.md §1.5](01_OVERVIEW.md) records every file the skeleton adds beyond it.

It was moved here from the repository root, together with the root `IMPLEMENTATION_GUIDE.md`,
so that there is exactly one specification directory rather than two documents at the root that
can drift apart from it.
