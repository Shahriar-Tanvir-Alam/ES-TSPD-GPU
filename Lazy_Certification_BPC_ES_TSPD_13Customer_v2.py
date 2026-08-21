#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LAZY-CERTIFICATION BRANCH-AND-PRICE FOR ES-TSPD (13-CUSTOMER, UNRESTRICTED)
================================================================================
Built around one idea worked out in chat: since P1's repeated re-solving was
the dominant runtime cost, and P1's only job is to compute waiting time,
split the problem into two phases connected by a provably-valid bound,
rather than trying to make P1 itself faster or called less often.

  PHASE 1 (search): branch-and-price on a SURROGATE problem - a sub-route's
  cost is just its truck length, waiting time is dropped entirely, so P1 is
  NEVER called during the search. Because waiting_time >= 0 always, this is
  a mathematically valid LOWER BOUND on the true cost of every column and of
  the true optimal tour, not an approximation.

  PHASE 2 (certification): every time Phase 1's search produces a complete,
  connected, integer-feasible surrogate tour, its TRUE cost is computed
  immediately via the real P1 oracle (edge-enumeration based) and compared
  against the best true cost found so far (Z_upper, self.best_ub).

  PRUNING is against Z_upper, not the surrogate LP's own value: a node is
  pruned once its surrogate LP bound >= Z_upper, valid because any
  completion of that node has true cost >= its surrogate cost >= the node's
  bound >= Z_upper. When the tree is fully exhausted this way, Z_upper is
  the PROVEN global optimum, not a heuristic.

  This also enables a genuinely lossless, order-independent Held-Karp-style
  dominance in the surrogate pricer (surrogate cost does not depend on drone
  delivery ORDER, only truck visiting order and the truck/drone split),
  which was never safely available for the real (waiting-time-including)
  pricing problem in any earlier Step 66-68 file.

================================================================================
BUGS FOUND AND FIXED, ROUND 1 (building and first validating this file)
================================================================================
Validated against an independent, from-scratch, multi-excursion brute-force
solver, tracing every mismatch to its root cause:

  1. SELF-LOOP COLUMNS. The surrogate pricer's drone-assign move doesn't
     change `current`, so a label could reach a nonzero visited-set while
     `current` still equaled the sub-route's own start `s`. Emitting a
     column there produces a start==end column contributing b_coeff=+1-1=0
     to every flow row - satisfying a customer's service requirement for
     free with no real truck movement behind it. Fixed by excluding cur==s
     at emission and defensively in ColumnManager.add() (matching the
     Appendix model's own i!=j arc requirement).

  2. P1 SOLVER RELIABILITY GAP. The direct-SLSQP-over-the-full-path P1
     oracle could report a KNOWN-feasible configuration as infeasible (a
     strict subset of the same path provably achieved zero waiting, while
     the full path reported infeasible). Fixed by using the edge-
     enumeration P1 oracle (solve_excursion) for all Phase 2 certification.

  3. REDUCED-COST SORT BUG. The cap on returned negative columns sorted by
     raw surrogate cost, not actual reduced cost, silently able to drop the
     most valuable candidate. Fixed.

  4. CERTIFICATION-LOOP DESIGN GAP. Unlike standard branch-and-price, an
     integer surrogate LP solution does NOT prove optimality for that node,
     because the surrogate objective isn't the true objective - other
     integer solutions with surrogate cost between the node's bound and the
     incumbent might certify to something better. Fixed via resolve_node():
     every certified solution has one of its columns excluded and the node
     is RE-SOLVED, repeating until the bound catches up or the LP goes
     fractional.

  5. DECOMPOSITION-VS-MERGED STRUCTURAL GAP. The surrogate LP can't
     distinguish "one continuous sub-route" from "the identical physical
     path split into several pieces" (same truck length), but splitting
     restricts each excursion's launch/recovery window, which can only hurt
     true cost. Fixed for the common single-excursion-per-tour case via a
     safe merge-and-recheck at certification time.

  Validated: n=3, 9/9 seeds exact. n=4, 1/1 tested exact.

================================================================================
BUGS FOUND AND FIXED, ROUND 2 (first production run at n=13 - SIGKILL/OOM)
================================================================================
The first real 13-customer run was killed by the OS (SIGKILL/exit 137)
before printing even one column-generation iteration. Diagnosis and fixes,
each one uncovered by testing the previous fix and finding the NEXT problem
underneath it - documented in full because each is a real, generalizable
hazard for this style of state-space search, not a one-off slip:

  6. NO PRUNING DURING SURROGATE-DP EXPANSION. The DP explored every
     reachable (current, visited_mask, drone_mask) state unconditionally.
     The true state count for a fixed source isn't ~3^n as originally
     estimated - `current` can be any of the (up to) n truck-visited nodes
     for a given split, pushing the real count into the hundreds of
     millions to billions at n=13. Fixed with a safe dual-based expansion
     bound (same proven three-part argument as safe_expansion_lb elsewhere
     in this project: truck cost only grows, use the safe minimum dual_flow
     over all possible future endpoints, use the optimistic maximum dual
     credit for remaining customers) - if even the best-case completion
     can't beat the acceptance threshold, stop expanding that state
     (though it is still emitted as a candidate for stopping HERE).

  7. HARD STATE-COUNT SAFETY CAP (MAX_SURROGATE_STATES_TOTAL). Pruning
     reduces the state count in practice but duals early in column
     generation can still be uninformative. This cap guarantees the process
     can never be OOM-killed again: if reached, the search stops and
     reports non-exhaustive, which the caller treats as "cannot declare
     this node CLOSED" (an honest NOT_PROVEN status), never a crash or a
     silently wrong answer.

  8. DEFERRED-EMISSION BUG (found once fixes 6-7 were in place and pricing
     STILL returned zero columns at n=13 despite exploring millions of
     states). Emission was deferred until ALL levels for a source finished
     building. If the state cap triggered while building a LATER, larger
     level, candidates from EARLIER, already-completed levels were never
     emitted at all - confirmed directly: a trivial one-drone-customer
     column with reduced cost -35 was completely missed, sitting fully
     computed but never checked, because the cap triggered while expanding
     a much later level. Fixed by emitting incrementally, right after each
     level completes.

  9. COLUMN-POOL MEMORY LEAK (found once fix 8 let pricing find columns
     again, and it promptly got OOM-killed a second way). manager.add() was
     called for EVERY emitted candidate, permanently growing the GLOBAL
     pool (shared across the whole run, never shrinks) without bound -
     confirmed directly: 500,000 explored states produced 645,470 permanent
     Column objects (~1.18GB) from a single pricing call. Fixed by
     collecting candidates as cheap tuples during the search and only
     calling manager.add() for the ones that survive both a per-signature
     deduplication and the max_negative_columns cut.

 10. CAP-TOO-SMALL REGRESSION (found once fix 9's cut was in place: a
     previously-passing n=4 instance started failing). Applying the
     dedup+cut with too small a max_negative_columns (300) discarded
     genuinely distinct, needed columns once n>=4 - a single pricing call
     can legitimately have far more than 300 useful negative-reduced-cost
     signatures. Fixed by raising MAX_SURROGATE_NEGATIVE_COLUMNS_PER_CALL to
     20,000 (re-validated: 9/9 at n=3, 1/1 at n=4 exact after this change).

================================================================================
VALIDATION RESULTS AND HONEST STATUS AT n=13
================================================================================
n=3: 9/9 seeds exact match against independent brute force (1,2,3,4,6,7,8,9,10).
n=4: 1/1 tested instance exact match, including the specific instance that
     exposed fix 10 above.

n=13, after all ten fixes: NO LONGER CRASHES (the original motivating bug is
fixed) and makes GENUINE progress - branches correctly, certifies integer
surrogate solutions via real P1, correctly identifies and permanently
excludes truly-infeasible columns (e.g. an all-13-customers-via-one-drone-
chain column from a degenerate zero-length depot-to-depot edge), and
continues searching rather than stopping prematurely.

HOWEVER: this has NOT been observed to reach PROVEN_OPTIMAL at n=13 within
tested time budgets (up to ~260 seconds). Individual nodes can still take
from under a second to well over a minute depending on how informative the
current duals are - pricing calls early after a fresh branch (uninformative
duals) are the slow case, since the safe expansion-pruning bound is much
weaker before column generation has had a chance to shape the duals. Some
run-to-run timing variance was also observed for IDENTICAL inputs (e.g. one
n=4 run took 28 seconds, a re-run of the exact same instance took over 150
seconds before being interrupted, though both converged to the correct
answer when given enough time) - likely from Python's hash randomization
affecting dict/set iteration order in the search, not a correctness issue,
but worth knowing about if you see run-to-run performance differences.

If you want to keep pushing toward n=13 in practical time, the next places
to look, in likely order of impact:
  - Warm-starting duals/columns across sibling branch nodes instead of each
    new node starting cold, since the slow case is specifically "duals just
    became uninformative after a fresh branch."
  - Tightening the expansion-pruning bound further (it is currently safe
    but not maximally tight - e.g. it never uses per-remaining-customer
    truck-distance information, only the raw truck length so far).
  - Investigating the timing variance (bug 10's sibling issue) in case it
    points to something more structural than hash randomization.

Requirements:
    pip install numpy scipy matplotlib
================================================================================
"""

from __future__ import annotations


# ====================================================================================================
# ---- Section from: es_tspd_common.py ----
# ====================================================================================================
"""
Shared, already-validated infrastructure reused verbatim from Step 68
(Column/BranchNode/ColumnManager, column_satisfies_node, b_coeff/cut_coeff/
truck_path_cut_coeff, solve_rmp, cut separation, P1Cache/solve_p1_chained).
None of this is new; it is the machinery whose correctness was already
established across the Step 66/67/68 validation rounds in this project.
"""

import itertools
import math
import os
import pickle
import random
import time
from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
from scipy.optimize import linprog, minimize, LinearConstraint, NonlinearConstraint

BIG_M = 1e7
REDUCED_COST_TOL = -1e-7
USE_STRONG_COMPLETE_SIGNATURE_DOMINANCE = True


def generate_instance(n_customers: int, seed: int, grid_size: int):
    random.seed(seed)
    start = 0
    end = n_customers + 1
    coords = {start: (random.uniform(0, grid_size), random.uniform(0, grid_size))}
    for i in range(1, n_customers + 1):
        coords[i] = (random.uniform(0, grid_size), random.uniform(0, grid_size))
    coords[end] = coords[start]
    customers = list(range(1, n_customers + 1))
    return coords, customers, start, end


def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist(coords: Dict[int, Tuple[float, float]], a: int, b: int) -> float:
    return dist_xy(coords[a], coords[b])


def route_length(coords, zeta: Sequence[int]) -> float:
    return sum(dist(coords, a, b) for a, b in zip(zeta[:-1], zeta[1:]))


# =============================================================================
# P1 ORACLE (Phase-2 certification only - never called during surrogate
# pricing). Verbatim from Step 68, already validated across this project.
# =============================================================================

class P1Result:
    __slots__ = ("feasible", "waiting", "breakpoints", "edge_assignment", "fractions")

    def __init__(self, feasible: bool, waiting: float = float("inf"),
                 breakpoints=None, edge_assignment=None, fractions=None):
        self.feasible = feasible
        self.waiting = waiting
        self.breakpoints = breakpoints or []
        self.edge_assignment = edge_assignment or tuple()
        self.fractions = fractions or tuple()


P1_RESTARTS = 3


def solve_p1_chained(coords: Dict[int, Tuple[float, float]],
                      zeta: Tuple[int, ...], beta: Tuple[int, ...],
                      phi0: float, phi1: float, endurance: float) -> Optional[P1Result]:
    """Exact-style modified chained P1 oracle for a fixed sub-route order.
    Truck path zeta=(s,...,e). Drone order beta=(d1,...,dm). Handoff points
    b0,...,bm lie in nondecreasing order along the polyline zeta, and drone
    trip k is b_{k-1}->beta[k]->b_k (p'_k = p_{k+1})."""
    m = len(beta)
    if m == 0:
        return P1Result(True, 0.0)
    pts = [coords[c] for c in zeta]
    seg_len = [dist_xy(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    total_len = sum(seg_len)
    if total_len <= 1e-12:
        return None
    prefix = [0.0]
    for L in seg_len:
        prefix.append(prefix[-1] + L)

    def point_at(t):
        t = min(max(t, 0.0), total_len)
        for i in range(len(seg_len)):
            if t <= prefix[i + 1] + 1e-12:
                seg = max(seg_len[i], 1e-15)
                lam = (t - prefix[i]) / seg
                x = pts[i][0] + lam * (pts[i + 1][0] - pts[i][0])
                y = pts[i][1] + lam * (pts[i + 1][1] - pts[i][1])
                return (x, y)
        return pts[-1]

    drone_pts = [coords[c] for c in beta]

    def objective(x):
        total = 0.0
        for k in range(m):
            p0 = point_at(x[k]); p1 = point_at(x[k + 1])
            flight = (dist_xy(p0, drone_pts[k]) + dist_xy(drone_pts[k], p1)) / phi1
            total += max(0.0, flight - (x[k + 1] - x[k]) / phi0)
        return total

    def endurance_ok(x):
        vals = []
        for k in range(m):
            p0 = point_at(x[k]); p1 = point_at(x[k + 1])
            flight = (dist_xy(p0, drone_pts[k]) + dist_xy(drone_pts[k], p1)) / phi1
            vals.append(endurance - flight)
        return np.array(vals)

    bounds = [(0.0, total_len) for _ in range(m + 1)]
    cons = [LinearConstraint(
        np.array([[1.0 if j == k + 1 else (-1.0 if j == k else 0.0) for j in range(m + 1)] for k in range(m)]),
        0.0, np.inf,
    ), NonlinearConstraint(endurance_ok, 0.0, np.inf)]

    best = None
    starts = [np.linspace(0.05 * total_len, 0.95 * total_len, m + 1),
              np.linspace(0.0, total_len, m + 1)]
    for frac in np.linspace(0.05, 0.4, max(1, P1_RESTARTS - len(starts))):
        starts.append(np.linspace(frac * total_len, (1.0 - frac) * total_len, m + 1))
    for x0 in starts[:P1_RESTARTS]:
        x0 = np.clip(np.asarray(x0, dtype=float), 0.0, total_len)
        x0 = np.maximum.accumulate(x0)
        try:
            res = minimize(objective, x0, method="SLSQP", bounds=bounds, constraints=cons,
                            options={"maxiter": 300, "ftol": 1e-10})
        except Exception:
            continue
        if not res.success:
            continue
        if np.any(endurance_ok(res.x) < -1e-6):
            continue
        val = float(objective(res.x))
        if best is None or val < best:
            best = val
    if best is None:
        return None
    return P1Result(True, best)


class P1Cache:
    def __init__(self, filename: str):
        self.filename = filename
        self.data: Dict[Tuple[Tuple[int, ...], Tuple[int, ...]], Optional[P1Result]] = {}
        self.hits = 0
        self.misses = 0
        self.solves = 0
        self.solve_seconds = 0.0
        self.load()

    def load(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, "rb") as f:
                    self.data = pickle.load(f)
                print(f"Loaded P1 cache: {self.filename} | entries={len(self.data):,}")
            except Exception:
                self.data = {}
        else:
            print(f"No P1 cache found; starting fresh: {self.filename}")

    def save(self):
        with open(self.filename, "wb") as f:
            pickle.dump(self.data, f)

    def get_or_solve(self, coords, zeta, beta, phi0, phi1, endurance) -> Optional[P1Result]:
        key = (tuple(zeta), tuple(beta))
        if key in self.data:
            self.hits += 1
            return self.data[key]
        self.misses += 1
        t0 = time.time()
        result = solve_p1_chained(coords, zeta, beta, phi0, phi1, endurance)
        self.solve_seconds += time.time() - t0
        self.solves += 1
        self.data[key] = result
        return result


# =============================================================================
# MASTER PROBLEM DATA STRUCTURES (verbatim from Step 68)
# =============================================================================

@dataclass
class Column:
    idx: int
    cost: float           # SURROGATE cost only (truck length) for all
                           # phase-1-generated columns - see solve() for how
                           # true (P1-certified) cost is tracked separately.
    start: int
    end: int
    zeta: Tuple[int, ...]
    beta: Tuple[int, ...]
    truck_set: FrozenSet[int]
    drone_set: FrozenSet[int]
    name: str = ""

    @property
    def served(self) -> FrozenSet[int]:
        return frozenset(set(self.truck_set) | set(self.drone_set))

    @property
    def signature(self):
        return (self.start, self.end, self.truck_set, self.drone_set)

    @property
    def endpoint_arc(self):
        return (self.start, self.end)


@dataclass
class BranchNode:
    node_id: int
    depth: int = 0
    forced_drone: Set[int] = field(default_factory=set)
    forced_truck: Set[int] = field(default_factory=set)
    forced_endpoint_arcs: Set[Tuple[int, int]] = field(default_factory=set)
    forbidden_endpoint_arcs: Set[Tuple[int, int]] = field(default_factory=set)
    cuts: List[FrozenSet[int]] = field(default_factory=list)
    truck_path_cuts: List[Tuple[FrozenSet[int], int]] = field(default_factory=list)

    def copy(self, new_id: int):
        return BranchNode(
            node_id=new_id, depth=self.depth + 1,
            forced_drone=set(self.forced_drone), forced_truck=set(self.forced_truck),
            forced_endpoint_arcs=set(self.forced_endpoint_arcs),
            forbidden_endpoint_arcs=set(self.forbidden_endpoint_arcs),
            cuts=list(self.cuts), truck_path_cuts=list(self.truck_path_cuts),
        )


class ColumnManager:
    def __init__(self, customers: List[int]):
        self.customers = customers
        self.pos = {c: i for i, c in enumerate(customers)}
        self.columns: List[Column] = []
        self.key_to_idx: Dict[Tuple[Tuple[int, ...], Tuple[int, ...]], int] = {}
        self.best_by_signature: Dict[Tuple[int, int, FrozenSet[int], FrozenSet[int]], int] = {}

    def add(self, cost: float, start: int, end: int, zeta: Tuple[int, ...], beta: Tuple[int, ...],
            name: str = "") -> Optional[int]:
        if start == end:
            # DEFENSIVE CHECK (see surrogate_pricer.py's BUG FIX comment for
            # the concrete failure mode this guards against): the Appendix
            # model's arc set requires i != j for every sub-route, matching
            # the base TSP-D arc definition. A start==end column contributes
            # b_coeff = +1-1 = 0 to every flow row, letting it satisfy a
            # customer's service requirement for free with no real truck
            # movement behind it - this must never be allowed to enter the
            # pool, from any caller.
            return None
        key = (tuple(zeta), tuple(beta))
        truck_set = frozenset(x for x in zeta[1:] if x in self.pos)
        drone_set = frozenset(beta)
        if truck_set & drone_set:
            return None
        served = truck_set | drone_set
        if not served:
            return None
        sig = (start, end, truck_set, drone_set)
        old_idx = self.best_by_signature.get(sig)
        if USE_STRONG_COMPLETE_SIGNATURE_DOMINANCE and old_idx is not None:
            if self.columns[old_idx].cost <= cost + 1e-9:
                return None
        if key in self.key_to_idx:
            if self.columns[self.key_to_idx[key]].cost <= cost + 1e-9:
                return None
        idx = len(self.columns)
        col = Column(idx, float(cost), start, end, tuple(zeta), tuple(beta), truck_set, drone_set, name)
        self.columns.append(col)
        self.key_to_idx[key] = idx
        if old_idx is None or cost < self.columns[old_idx].cost - 1e-9:
            self.best_by_signature[sig] = idx
        return idx

    def filtered_indices(self, node: BranchNode) -> List[int]:
        return [i for i, c in enumerate(self.columns) if column_satisfies_node(c, node)]


def column_satisfies_node(col: Column, node: BranchNode) -> bool:
    for i in node.forced_drone:
        if i in col.truck_set:
            return False
    for i in node.forced_truck:
        if i in col.drone_set:
            return False
    if col.endpoint_arc in node.forbidden_endpoint_arcs:
        return False
    return True


def b_coeff(col: Column, node_id: int) -> int:
    return int(col.start == node_id) - int(col.end == node_id)


def cut_coeff(col: Column, V: FrozenSet[int]) -> int:
    return int((col.start not in V) and bool(set(col.served) & set(V)))


def truck_path_cut_coeff(col: Column, S: FrozenSet[int], k: int) -> int:
    crossings = 0
    for a, b in zip(col.zeta[:-1], col.zeta[1:]):
        if (a in S) != (b in S):
            crossings += 1
    truck_service_k = 1 if k in col.truck_set else 0
    return crossings - truck_service_k


def solve_rmp(manager: ColumnManager, active: List[int], node: BranchNode,
              customers: List[int], start: int, end: int):
    columns = [manager.columns[i] for i in active]
    nodes = [start] + customers + [end]
    n_flow = len(nodes)
    n_cover = len(customers)
    n_cuts = len(node.cuts)
    n_tpcuts = len(node.truck_path_cuts)
    n_reqarc = len(node.forced_endpoint_arcs)
    R = len(columns)

    n_eq = n_flow + n_cover + n_reqarc
    A_eq = np.zeros((n_eq, R))
    b_eq = np.zeros(n_eq)
    for r, col in enumerate(columns):
        for row, nd in enumerate(nodes):
            A_eq[row, r] = b_coeff(col, nd)
    b_eq[0] = 1.0
    b_eq[n_flow - 1] = -1.0

    offset = n_flow
    for p, cust in enumerate(customers):
        row = offset + p
        b_eq[row] = 1.0
        for r, col in enumerate(columns):
            A_eq[row, r] = int(cust in col.served)

    req_arc_list = sorted(node.forced_endpoint_arcs)
    offset = n_flow + n_cover
    for q, arc in enumerate(req_arc_list):
        row = offset + q
        b_eq[row] = 1.0
        for r, col in enumerate(columns):
            A_eq[row, r] = 1.0 if col.endpoint_arc == arc else 0.0

    n_art = n_eq
    Aeq_art = np.zeros((n_eq, 2 * n_art))
    for i in range(n_art):
        Aeq_art[i, 2 * i] = 1.0
        Aeq_art[i, 2 * i + 1] = -1.0
    c_art = np.full(2 * n_art, BIG_M)

    n_ge = n_cuts + n_tpcuts
    A_ge = np.zeros((n_ge, R))
    b_ge = np.zeros(n_ge)
    offset = 0
    for q, V in enumerate(node.cuts):
        row = offset + q
        b_ge[row] = 1.0
        for r, col in enumerate(columns):
            A_ge[row, r] = cut_coeff(col, V)
    offset = n_cuts
    for q, (S, k) in enumerate(node.truck_path_cuts):
        row = offset + q
        b_ge[row] = 0.0
        for r, col in enumerate(columns):
            A_ge[row, r] = truck_path_cut_coeff(col, S, k)

    n_ge_art = n_ge
    c_ge_art = np.full(n_ge_art, BIG_M)

    A_ub_full = None
    b_ub_full = None
    if n_ge > 0:
        A_ub_full = np.zeros((n_ge, R + 2 * n_art + n_ge_art))
        A_ub_full[:, :R] = -A_ge
        A_ub_full[:, R + 2 * n_art:] = -np.eye(n_ge_art)
        b_ub_full = -b_ge

    c = np.array([col.cost for col in columns])
    c_full = np.concatenate([c, c_art, c_ge_art])
    A_eq_full = np.zeros((n_eq, R + 2 * n_art + n_ge_art))
    A_eq_full[:, :R] = A_eq
    A_eq_full[:, R:R + 2 * n_art] = Aeq_art

    res = linprog(c_full, A_eq=A_eq_full, b_eq=b_eq, A_ub=A_ub_full, b_ub=b_ub_full,
                  bounds=[(0, None)] * len(c_full), method="highs")
    if res.status != 0:
        return None

    lam = res.x[:R]
    art_sum = float(np.sum(res.x[R:]))
    obj = float(c @ lam)

    eq_dual = np.array(res.eqlin.marginals) if n_eq > 0 else np.zeros(0)
    if n_ge > 0:
        ge_dual = -np.array(res.ineqlin.marginals)
    else:
        ge_dual = np.zeros(0)

    dual_flow = {nd: float(eq_dual[i]) for i, nd in enumerate(nodes)}
    dual_cover = {cust: float(eq_dual[n_flow + p]) for p, cust in enumerate(customers)}
    dual_req_arc = {arc: float(eq_dual[n_flow + n_cover + q]) for q, arc in enumerate(req_arc_list)}
    dual_cuts = [(node.cuts[q], float(ge_dual[q])) for q in range(n_cuts)]
    dual_tpcuts = [(node.truck_path_cuts[q][0], node.truck_path_cuts[q][1], float(ge_dual[n_cuts + q]))
                   for q in range(n_tpcuts)]

    return {
        "obj": obj, "lambda": lam, "columns": columns, "active": active, "artificial_sum": art_sum,
        "dual_flow": dual_flow, "dual_cover": dual_cover, "dual_req_arc": dual_req_arc,
        "dual_cuts": dual_cuts, "dual_tpcuts": dual_tpcuts,
    }


def reduced_cost_of_column(col: Column, duals) -> float:
    rc = col.cost
    for nd, pi in duals["dual_flow"].items():
        rc -= pi * b_coeff(col, nd)
    for cust, sig in duals["dual_cover"].items():
        rc -= sig * int(cust in col.served)
    for arc, tau in duals.get("dual_req_arc", {}).items():
        rc -= tau * (1.0 if col.endpoint_arc == arc else 0.0)
    for V, omega in duals["dual_cuts"]:
        rc -= omega * cut_coeff(col, V)
    for S, k, eta in duals["dual_tpcuts"]:
        rc -= eta * truck_path_cut_coeff(col, S, k)
    return rc


def separate_connectivity_cuts(rmp, customers: List[int], max_cuts: int) -> List[FrozenSet[int]]:
    columns = rmp["columns"]; lam = rmp["lambda"]
    active = [(columns[i], lam[i]) for i in range(len(columns)) if lam[i] > 1e-7]
    found = []
    n = len(customers)
    for size in range(1, n):
        for V in itertools.combinations(customers, size):
            V = frozenset(V)
            lhs = sum(val for col, val in active if cut_coeff(col, V) == 1)
            if lhs < 1.0 - 1e-6:
                found.append(V)
                if len(found) >= max_cuts:
                    return found
    return found


def separate_truck_path_cuts(rmp, customers: List[int], max_cuts: int) -> List[Tuple[FrozenSet[int], int]]:
    columns = rmp["columns"]; lam = rmp["lambda"]
    active = [(columns[i], lam[i]) for i in range(len(columns)) if lam[i] > 1e-7]
    found = []
    n = len(customers)
    for size in range(1, n):
        for S in itertools.combinations(customers, size):
            S = frozenset(S)
            for k in S:
                lhs = sum(val * truck_path_cut_coeff(col, S, k) for col, val in active)
                if lhs < -1e-6:
                    found.append((S, k))
                    if len(found) >= max_cuts:
                        return found
    return found


def nearest_neighbor_route(coords, customers, start, end):
    remaining = set(customers)
    route = [start]
    cur = start
    while remaining:
        nxt = min(remaining, key=lambda c: dist(coords, cur, c))
        route.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    route.append(end)
    return route


def build_initial_columns(manager: ColumnManager, coords, customers, start, end, phi0):
    route = nearest_neighbor_route(coords, customers, start, end)
    initial_route_cost = route_length(coords, route) / phi0
    idx = manager.add(initial_route_cost, start, end, tuple(route), tuple(), "initial_full_truck_route")
    active = [idx] if idx is not None else []

    nodes_start = [start] + customers
    nodes_end = customers + [end]
    for s in nodes_start:
        for e in nodes_end:
            if s == e:
                continue
            if e == end:
                for k in customers:
                    if k == s:
                        continue
                    zeta = (s, k, end)
                    conn_cost = route_length(coords, zeta) / phi0
                    idx = manager.add(conn_cost, s, end, zeta, tuple(), "init_truck_connector")
                    if idx is not None:
                        active.append(idx)
            else:
                zeta = (s, e)
                conn_cost = route_length(coords, zeta) / phi0
                idx = manager.add(conn_cost, s, e, zeta, tuple(), "init_one_customer")
                if idx is not None:
                    active.append(idx)
    return sorted(set(active)), initial_route_cost, route


def build_solution_path(selected_cols: List[Column], start: int, end: int) -> Optional[List[Column]]:
    by_start: Dict[int, Column] = {}
    for c in selected_cols:
        if c.start in by_start:
            return None
        by_start[c.start] = c
    ordered = []
    cur = start
    seen = set()
    while True:
        if cur not in by_start:
            return None
        c = by_start[cur]
        if c.idx in seen:
            return None
        seen.add(c.idx)
        ordered.append(c)
        cur = c.end
        if cur == end:
            break
        if len(ordered) > len(selected_cols) + 1:
            return None
    if len(ordered) != len(selected_cols):
        return None
    return ordered


# ====================================================================================================
# ---- Section from: excursion_solver.py ----
# ====================================================================================================
"""
Exact solver for the (chained) en-route launch/recovery sub-problem.

Given an ORDERED list of "anchor points" (2D coordinates) that bound a
contiguous stretch of the truck's route, and an ORDERED list of drone-served
customers to be delivered as ONE continuous chained excursion (per the
user's modified constraint P1.2':

    Ar(p1) < Ar(p'1) = Ar(p2) < Ar(p'2) = ... = Ar(pmr) < Ar(p'mr)

find the mr+1 hand-off points (b_0=launch, b_1=p'_1=p_2, ..., b_mr=final
recovery) minimizing total truck waiting time, subject to the maximum
drone-endurance constraint on every individual leg.

Because the hand-off points must lie in non-decreasing order along the
route, and because for a FIXED choice of which truck edge hosts each
hand-off point the problem is convex (affine Ar terms + convex Euclidean
distance terms), we solve it EXACTLY by enumerating every non-decreasing
edge-index assignment for the mr+1 hand-off points (this enumeration is
finite and, given the DP bounds the excursion span, small) and solving
the resulting small convex program via SLSQP for each assignment, keeping
the best feasible result. This reproduces Cases 1/2/3 of the paper
automatically - which case applies simply falls out of which edges the
winning assignment picks.
"""

from typing import List, Optional, Tuple

from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint

_EXCURSION_CACHE = {}


def _round_pt(p, ndigits=6):
    return (round(p[0], ndigits), round(p[1], ndigits))


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


class ExcursionResult:
    __slots__ = ("waiting_time", "breakpoints", "edge_assignment", "fractions", "feasible")

    def __init__(self, waiting_time, breakpoints, edge_assignment, fractions, feasible):
        self.waiting_time = waiting_time
        self.breakpoints = breakpoints          # list of (x, y) hand-off points, length mr+1
        self.edge_assignment = edge_assignment  # tuple of edge indices, length mr+1
        self.fractions = fractions              # tuple of lambda in [0,1], length mr+1
        self.feasible = feasible


def solve_excursion(anchor_points: List[Tuple[float, float]],
                     drone_customers: List[Tuple[float, float]],
                     phi0: float, phi1: float, endurance: float,
                     n_restarts: int = 3) -> Optional[ExcursionResult]:
    """
    anchor_points   : ordered [(x,y), ...] of length K+1, defining K candidate
                       truck edges (anchor_points[k] -> anchor_points[k+1]).
                       anchor_points[0] is the excursion's earliest possible
                       launch location, anchor_points[-1] the latest possible
                       final-recovery location.
    drone_customers : ordered [(x,y), ...] of length mr (the served customers,
                       in delivery order). mr must be >= 1.
    phi0, phi1      : truck / drone speeds.
    endurance       : maximum drone flight TIME per leg (time units, matching
                       the paper's parameter e).

    Returns the best ExcursionResult, or None if no edge assignment yields a
    feasible (endurance-respecting) solution.

    Results are memoized: the geometric sub-problem is independent of column-
    generation duals, so a given (anchor window, drone-customer sequence,
    speeds, endurance) combination is cached across the ENTIRE branch-and-
    price run, not just within one pricing call - this matters a lot in
    practice because the DP's label-setting re-derives the same excursion
    window from multiple redundant paths.
    """
    cache_key = (tuple(_round_pt(p) for p in anchor_points),
                 tuple(_round_pt(v) for v in drone_customers),
                 round(phi0, 6), round(phi1, 6), round(endurance, 6))
    if cache_key in _EXCURSION_CACHE:
        return _EXCURSION_CACHE[cache_key]

    result = _solve_excursion_uncached(anchor_points, drone_customers, phi0, phi1,
                                        endurance, n_restarts)
    _EXCURSION_CACHE[cache_key] = result
    return result


def _solve_excursion_uncached(anchor_points: List[Tuple[float, float]],
                               drone_customers: List[Tuple[float, float]],
                               phi0: float, phi1: float, endurance: float,
                               n_restarts: int = 3) -> Optional[ExcursionResult]:
    """Uncached implementation - see solve_excursion() for the full docstring."""
    anchors = [np.array(p, dtype=float) for p in anchor_points]
    custs = [np.array(v, dtype=float) for v in drone_customers]
    mr = len(custs)
    K = len(anchors) - 1  # number of candidate edges
    assert mr >= 1 and K >= 1

    edge_vec = [anchors[k + 1] - anchors[k] for k in range(K)]
    edge_len = [float(np.linalg.norm(v)) for v in edge_vec]
    cum_len = [0.0]
    for L in edge_len:
        cum_len.append(cum_len[-1] + L)

    def point_on_route(edge_idx: int, lam: float) -> np.ndarray:
        return anchors[edge_idx] + lam * edge_vec[edge_idx]

    def arrival_time(edge_idx: int, lam: float) -> float:
        return (cum_len[edge_idx] + lam * edge_len[edge_idx]) / phi0

    best: Optional[ExcursionResult] = None

    # mr+1 hand-off points, each assigned to a non-decreasing edge index
    for combo in itertools.combinations_with_replacement(range(K), mr + 1):

        same_edge_pairs = [k for k in range(mr) if combo[k] == combo[k + 1]]

        def unpack(x):
            return x  # x IS the vector of fractions, one per hand-off point

        def objective(x):
            total = 0.0
            for k in range(mr):
                e0, e1 = combo[k], combo[k + 1]
                p0 = point_on_route(e0, x[k])
                p1 = point_on_route(e1, x[k + 1])
                a0 = arrival_time(e0, x[k])
                a1 = arrival_time(e1, x[k + 1])
                drone_t = (_dist(p0, custs[k]) + _dist(custs[k], p1)) / phi1
                total += max(0.0, a0 + drone_t - a1)
            return total

        def endurance_constraints(x):
            vals = []
            for k in range(mr):
                e0, e1 = combo[k], combo[k + 1]
                p0 = point_on_route(e0, x[k])
                p1 = point_on_route(e1, x[k + 1])
                drone_t = (_dist(p0, custs[k]) + _dist(custs[k], p1)) / phi1
                vals.append(endurance - drone_t)  # >= 0
            return np.array(vals)

        bounds = [(0.0, 1.0)] * (mr + 1)

        constraints = []
        if same_edge_pairs:
            # x[k] <= x[k+1] whenever combo[k] == combo[k+1]
            A = np.zeros((len(same_edge_pairs), mr + 1))
            for row, k in enumerate(same_edge_pairs):
                A[row, k] = 1.0
                A[row, k + 1] = -1.0
            constraints.append(LinearConstraint(A, -np.inf, 0.0))
        constraints.append(NonlinearConstraint(endurance_constraints, 0.0, np.inf))

        # feasibility pre-check at a naive midpoint start; also used as a start
        starts = [np.full(mr + 1, 0.5)]
        for _ in range(max(0, n_restarts - 1)):
            starts.append(np.random.uniform(0.0, 1.0, size=mr + 1))
        # make starts respect same-edge ordering roughly (sort within groups)
        fixed_starts = []
        for s in starts:
            s = s.copy()
            groups = []
            cur = [0]
            for k in range(mr):
                if combo[k] == combo[k + 1]:
                    cur.append(k + 1)
                else:
                    groups.append(cur)
                    cur = [k + 1]
            groups.append(cur)
            for g in groups:
                vals = sorted(s[g])
                for idx, gi in enumerate(g):
                    s[gi] = vals[idx]
            fixed_starts.append(s)

        for s0 in fixed_starts:
            try:
                res = minimize(objective, s0, method="SLSQP", bounds=bounds,
                                constraints=constraints,
                                options={"maxiter": 200, "ftol": 1e-10})
            except Exception:
                continue
            if not res.success:
                continue
            # verify endurance strictly (SLSQP can be slightly loose)
            ev = endurance_constraints(res.x)
            if np.any(ev < -1e-6):
                continue
            w = float(objective(res.x))
            if best is None or w < best.waiting_time - 1e-9:
                bp = [tuple(point_on_route(combo[k], res.x[k])) for k in range(mr + 1)]
                best = ExcursionResult(w, bp, combo, tuple(res.x.tolist()), True)

    return best


# ====================================================================================================
# ---- Section from: surrogate_pricer.py ----
# ====================================================================================================
"""
SURROGATE PRICING: no P1 calls, Held-Karp-style dominance.

Core idea (see chat discussion): a sub-route's TRUE cost is
    truck_length(zeta) + waiting_time(zeta, beta)
and waiting_time is always >= 0. Dropping it gives a SURROGATE cost
(truck_length only) that is provably a lower bound on the true cost for
every column, unconditionally - not an approximation.

This matters for two independent reasons:
  1. The surrogate master's LP objective is therefore a valid lower bound on
     the true optimal tour cost (see lazy_certification_bpc.py for how this
     bound is used together with a separately-tracked, P1-certified upper
     bound to prove global optimality without ever solving P1 during the
     search itself).
  2. Because the surrogate objective no longer depends on drone delivery
     ORDER (only truck length, and truck length only depends on which
     customers are truck-visited and in what order - NOT which order drone
     customers are served in, since they aren't part of the truck's
     physical path at all), a genuinely lossless, order-independent
     dominance rule becomes available: for a fixed (current position,
     visited set, drone-vs-truck split), the lowest accumulated truck
     length dominates every other way of reaching that same state, by the
     standard Held-Karp/Bellman argument (future cost-to-go and feasible
     continuations depend only on that triple, never on the specific order
     used to reach it). This is exactly the argument that could NOT be made
     safely for the real (waiting-time-including) pricing problem, where
     drone delivery order genuinely changes achievable cost - that
     unresolved asymmetry was the root of the repeated validation failures
     in Step 66/67/68.

DELIBERATE DESIGN CHOICE - no endurance feasibility filter here at all:
    Over-including a drone assignment that turns out infeasible does NOT
    break the lower-bound property (real cost of an infeasible assignment is
    effectively +infinity, so surrogate_cost <= true_cost trivially still
    holds) and does NOT let an invalid column reach the incumbent (Phase 2
    certification, using the exact P1 oracle, is the only thing that can
    ever set/accept a true cost - it correctly rejects anything infeasible).
    It only means a handful of provably-infeasible columns might be
    generated and later rejected in Phase 2, which is a pure efficiency
    cost, never a correctness one. A cheap point-to-polyline feasibility
    bound was considered but rejected: it depends on the exact realized
    zeta (which edges exist), so applying it inside the DP would break the
    order-independent dominance argument above (two labels sharing the same
    (current, visited_mask, drone_mask) but reached via different truck
    orders could give the bound different values), and applying it only
    using the dominance-surviving label's own zeta risks incorrectly
    rejecting an assignment that a discarded, cheaper-truck-length ordering
    would have made feasible. Skipping the filter entirely keeps the
    dominance argument simple and unconditionally safe.
"""


from typing import Dict, List, Tuple



MAX_SURROGATE_STATES_TOTAL = 5_000_000  # ~1.7GB observed in testing - see BUG FIX below.
# Tune this down if your machine has less available RAM, or up if it has
# more and pricing is hitting this cap often (check the "exhaustive=False"
# flag printed each CG iteration) - it trades memory/time for how much of
# the search space a single pricing call can cover before giving up and
# forcing an honest NOT_PROVEN status rather than a guess.


def surrogate_pricing(manager: ColumnManager, coords, customers: List[int], start: int, end: int,
                       phi0: float, duals, node: BranchNode, max_negative_columns: int = 200
                       ) -> Tuple[List[int], Dict[str, int]]:
    """Held-Karp-style label-setting search for negative-surrogate-reduced-
    cost sub-routes, run once per (start-node, node) combination internally
    over ALL possible sub-route sources simultaneously. No P1 calls anywhere.

    BUG FIX (found in production use at n=13): this function previously had
    NO pruning during expansion at all - it built out every reachable state
    regardless of whether it could possibly lead to a negative reduced cost,
    checking reduced cost only at emission time. The true state count for a
    fixed source isn't the ~3^n I originally estimated - for a given
    truck/drone/unvisited split, `current` can be any one of the (up to)
    n truck-visited nodes, adding a multiplicative factor this file's
    original docstring missed - pushing the real count into the hundreds of
    millions to billions at n=13, well past what fits in memory even before
    a single pricing call could finish (confirmed: this alone caused an
    out-of-memory kill on a 13-customer run, before the very first CG
    iteration printed).

    Two independent fixes, both required:

    1. SAFE DUAL-BASED EXPANSION PRUNING. Before expanding a state further,
       check whether even the BEST possible completion from here (zero
       additional truck cost, the most optimistic possible dual_flow value
       across every node this sub-route could still end at, and full dual
       credit for every still-unserved customer) could reach a negative
       reduced cost. If not, nothing reachable by extending this state can
       ever be useful, so it is safe to stop expanding it (though it is
       still emitted as a candidate for stopping HERE - see below). This is
       the same three-part argument (truck cost only grows, dual_flow use
       the safe minimum over all possible endpoints, dual gain uses the
       optimistic maximum over remaining customers) proven correct earlier
       in this project for the non-surrogate pricer's safe_expansion_lb -
       it transfers directly here since it never depended on delivery
       order in the first place.

    2. HARD STATE-COUNT SAFETY CAP (MAX_SURROGATE_STATES_TOTAL). Pruning
       reduces the state count in practice but its worst case is still
       unbounded (duals early in column generation, e.g. at the very first
       CG iteration of the root node, can be uninformative and prune
       almost nothing). This cap guarantees the process can never be
       OOM-killed again: if it is reached, the search stops immediately and
       reports non-exhaustive, which the caller (column_generation_at_node)
       must treat as "cannot declare this node CLOSED" rather than silently
       treating whatever was found so far as complete - the honest
       consequence is a NOT_PROVEN status rather than a false proof, never
       a crash or a wrong answer.

    Returns (list of manager indices for newly added negative-reduced-cost
    columns, stats dict with an "exhaustive" key the caller MUST check).
    """
    dual_flow = duals["dual_flow"]
    dual_cover = duals["dual_cover"]
    n = len(customers)
    pos = {c: i for i, c in enumerate(customers)}

    def bit(i):
        return 1 << i

    def dual_gain(visited_mask):
        g = 0.0
        for c in customers:
            if visited_mask & bit(pos[c]):
                g += dual_cover.get(c, 0.0)
        return g

    # FIX 1 setup: the safest (most optimistic) possible dual_flow value
    # across every node this sub-route could still end at, and a lookup for
    # "best possible remaining credit" given whichever customers are not
    # yet visited.
    all_possible_endpoints = customers + [end]
    min_possible_end_dual = min(dual_flow.get(e, 0.0) for e in all_possible_endpoints)
    max_dual_cover = {c: max(0.0, dual_cover.get(c, 0.0)) for c in customers}
    # prefix-free total, recomputed per remaining set on demand (n is small
    # enough per sub-route search that this is cheap relative to the state
    # expansion it guards)

    negative_indices: List[int] = []
    all_candidates: List[Tuple[float, int, int, Tuple[int, ...], Tuple[int, ...], float]] = []
    # BUG FIX (found in production use at n=13, the actual cause of a
    # renewed OOM kill even with the state-count cap and incremental
    # emission both in place): manager.add() was being called for EVERY
    # emitted candidate, permanently growing the GLOBAL column pool
    # (shared across the whole run, never shrinks) without any bound -
    # confirmed directly: 500,000 explored states produced 645,470
    # permanent Column objects, ~1.18GB from that alone, for a single
    # pricing call. Extrapolated to the state cap this would reach many
    # millions of permanent columns and many GB. The fix: collect
    # candidates as cheap tuples (reduced cost, start, end, zeta, beta,
    # true surrogate cost) locally during the search, and only call
    # manager.add() for the max_negative_columns best ones AFTER sorting -
    # exactly the set that would ever be used anyway, so this changes
    # memory behavior only, not what pricing can find or return.
    states_explored = 0
    states_kept = 0
    exhaustive = True

    def emit_level(s, level):
        """BUG FIX: emission used to be deferred until ALL levels for a
        source were fully built, which meant that if the state-count safety
        cap triggered while building a LATER, larger level, candidates from
        EARLIER, already-completed levels (which can include excellent,
        deeply negative reduced-cost columns reachable in just one or two
        moves) were never emitted at all - confirmed directly: a trivial
        one-drone-customer column with reduced cost -35 was completely
        missed at n=13 because the cap was hit while expanding a much later
        level, and the whole search returned empty-handed despite this
        cheap, obviously-good column having been fully computed early on.
        Emitting right after each level completes (including level 0, which
        matters when a source's sub-route can end immediately after a
        single drone-only move followed by "extend to end") fixes this
        without changing what is ever accepted - the reduced-cost checks
        below are identical to before, just no longer deferred. Candidates
        are collected as cheap tuples here, NOT added to the pool yet - see
        the BUG FIX note above all_candidates."""
        for (cur, vmask, dmask), (truck_len, zeta, dorder) in level.items():
            if cur != s and (s, cur) not in node.forbidden_endpoint_arcs:
                rc = truck_len - dual_flow.get(s, 0.0) + dual_flow.get(cur, 0.0) - dual_gain(vmask)
                if rc < REDUCED_COST_TOL:
                    all_candidates.append((rc, s, cur, zeta, dorder, truck_len))
            if cur != end and (s, end) not in node.forbidden_endpoint_arcs:
                new_len = truck_len + dist(coords, cur, end)
                new_zeta = zeta + (end,)
                rc_end = new_len - dual_flow.get(s, 0.0) + dual_flow.get(end, 0.0) - dual_gain(vmask)
                if rc_end < REDUCED_COST_TOL:
                    all_candidates.append((rc_end, s, end, new_zeta, dorder, new_len))

    def dedupe_by_signature(cands):
        """BUG FIX (found in production use at n=4): applying the
        max_negative_columns cut directly on all_candidates, before any
        deduplication, let multiple redundant variants of the SAME
        (s, e, truck_set, drone_set) signature (different zeta/drone orders
        that end up with the same served sets) crowd out the top-K slots -
        confirmed directly: reverting only the "collect tuples, cut, then
        add" change (keeping incremental emission) restored the correct
        answer, isolating this as the cause. The previous (memory-heavy)
        version avoided this by accident: manager.add()'s own per-signature
        dominance deduplicated BEFORE any cap was ever applied, since
        everything was added first and only the resulting indices were
        capped afterward. This restores the same effective behavior -
        keep only the best (lowest reduced cost) candidate per signature -
        cheaply, on the lightweight tuples, before any cap or manager.add()
        call.
        """
        best: Dict[Tuple[int, int, frozenset, frozenset], tuple] = {}
        for cand in cands:
            rc, s, cur, zeta, dorder, cost = cand
            truck_set = frozenset(x for x in zeta[1:] if x in pos)
            drone_set = frozenset(dorder)
            sig = (s, cur, truck_set, drone_set)
            old = best.get(sig)
            if old is None or rc < old[0]:
                best[sig] = cand
        return list(best.values())

    for s in [start] + customers:
        if not exhaustive:
            break
        # state key: (current, visited_mask, drone_mask) -> (truck_len, zeta, drone_order)
        # levels indexed by popcount(visited_mask), since every move adds
        # exactly one bit (either a truck visit or a drone assignment).
        level0: Dict[Tuple[int, int, int], Tuple[float, Tuple[int, ...], Tuple[int, ...]]] = {
            (s, 0, 0): (0.0, (s,), tuple())
        }
        levels = [level0]
        # level 0 itself is never a candidate (cur==s, vmask==0 -> excluded
        # by both the self-loop guard and the "must serve someone" logic in
        # manager.add), so nothing to emit for it, but every level from here
        # on is emitted the moment it's finished being built.

        for _ in range(n):
            cur_level = levels[-1]
            if not cur_level:
                break
            next_level: Dict[Tuple[int, int, int], Tuple[float, Tuple[int, ...], Tuple[int, ...]]] = {}

            def offer(key, val):
                nonlocal states_kept
                old = next_level.get(key)
                if old is None or val[0] < old[0] - 1e-12:
                    next_level[key] = val
                    states_kept += 1

            for (cur, vmask, dmask), (truck_len, zeta, dorder) in cur_level.items():
                states_explored += 1
                if states_explored > MAX_SURROGATE_STATES_TOTAL:
                    exhaustive = False
                    break
                remaining = [c for c in customers if not (vmask & bit(pos[c]))]

                # FIX 1: safe expansion-pruning check. Skips expanding this
                # state further (but does NOT skip emitting it - that check
                # happens separately below, using the exact dual_flow[cur]
                # for "stop here", which is always valid regardless of this
                # bound). Uses the SAME threshold as the emission check
                # (REDUCED_COST_TOL, already negative) for consistency: if
                # even the best-case achievable reduced cost from here is
                # not below that threshold, nothing reachable by extending
                # further can ever be accepted.
                max_future_gain = sum(max_dual_cover[c] for c in remaining)
                expansion_lb = (truck_len - dual_flow.get(s, 0.0) + min_possible_end_dual
                                 - dual_gain(vmask) - max_future_gain)
                if expansion_lb >= REDUCED_COST_TOL:
                    continue

                # move: truck-visit next unvisited customer c
                for c in remaining:
                    if c in node.forced_drone:
                        continue
                    if (cur, c) in node.forbidden_endpoint_arcs:
                        continue
                    new_len = truck_len + dist(coords, cur, c)
                    new_zeta = zeta + (c,)
                    key = (c, vmask | bit(pos[c]), dmask)
                    offer(key, (new_len, new_zeta, dorder))

                # move: drone-assign next unvisited customer c (does not move
                # 'current', does not change truck length or zeta at all -
                # this is exactly what makes order irrelevant to surrogate cost)
                for c in remaining:
                    if c in node.forced_truck:
                        continue
                    key = (cur, vmask | bit(pos[c]), dmask | bit(pos[c]))
                    offer(key, (truck_len, zeta, dorder + (c,)))

            # BUG FIX: emit THIS level's candidates now, before checking
            # whether the cap was hit - a level that finished building
            # (even if the cap triggers partway through building the NEXT
            # one) is complete and safe to emit from immediately.
            emit_level(s, next_level)
            # BUG FIX (memory bound on all_candidates itself): even cheap
            # tuples can accumulate without bound if there are extremely
            # many negative-reduced-cost states. Only the best
            # max_negative_columns will ever be used, so periodically trim
            # to a safety-buffered multiple of that - correctness-neutral,
            # since anything trimmed away was, by construction, worse than
            # what's kept.
            if len(all_candidates) > 50_000:
                all_candidates[:] = dedupe_by_signature(all_candidates)
                all_candidates.sort(key=lambda t: t[0])
                del all_candidates[max(max_negative_columns * 5, 5_000):]

            if not exhaustive:
                break
            levels.append(next_level)

    # BUG FIX (found during validation): sorting by raw surrogate cost before
    # applying the max_negative_columns cap can silently drop a column with
    # very negative REDUCED cost (high dual credit, e.g. serving several
    # high-dual customers) in favor of one with low raw cost but only barely
    # negative reduced cost - confirmed to be the direct cause of a missed
    # globally-optimal column in validation. Sorting by actual reduced cost
    # is what the cap should prioritize keeping. Now done on the lightweight
    # candidate tuples BEFORE ever calling manager.add(), so the permanent
    # global pool only ever grows by the columns that will actually be used.
    all_candidates = dedupe_by_signature(all_candidates)
    all_candidates.sort(key=lambda t: t[0])
    for rc, s, cur, zeta, dorder, cost in all_candidates[:max_negative_columns]:
        idx = manager.add(cost, s, cur, zeta, dorder,
                           "surrogate_priced_end" if cur == end else "surrogate_priced")
        if idx is not None and column_satisfies_node(manager.columns[idx], node):
            negative_indices.append(idx)

    negative_indices = list(dict.fromkeys(negative_indices))
    stats = {"states_explored": states_explored, "states_kept": states_kept, "exhaustive": exhaustive}
    return negative_indices, stats


# ====================================================================================================
# ---- Section from: lazy_certification_bpc.py ----
# ====================================================================================================
"""
LAZY-CERTIFICATION BRANCH-AND-PRICE

Two-phase scheme (see chat discussion for the full derivation):

  PHASE 1 (search): branch-and-price on the SURROGATE problem (truck length
  only, no waiting time, no P1 calls anywhere in pricing - see
  surrogate_pricer.py). Because waiting_time >= 0 always, the surrogate cost
  of ANY column is a valid lower bound on that column's true cost, so the
  surrogate LP's optimal value at any node is a valid lower bound on the
  true optimal cost achievable from that node.

  PHASE 2 (certification): every time Phase 1 finds a CONNECTED, INTEGER-
  FEASIBLE surrogate solution (a complete tour), its true cost is computed
  immediately by running the real P1 oracle (solve_p1_chained) on each of
  its sub-routes - trying alternative drone-delivery orders where the
  chain is small enough to search exhaustively, since delivery order does
  not affect surrogate cost but does affect true cost. This true cost is
  compared against the best true cost found so far (Z_upper) - NOT the
  surrogate LP's own reported value.

  PRUNING: a node is pruned once its surrogate LP bound is >= Z_upper (the
  best TRUE cost found so far), never against the surrogate LP's own
  incumbent tracking. This is valid because ANY completion of that node has
  true cost >= its surrogate cost >= the node's surrogate LP bound >=
  Z_upper - it cannot possibly improve on what is already proven
  achievable.

  TERMINATION: when every node has been closed (exhaustive surrogate
  pricing, zero artificials) or pruned this way, Z_upper is not a
  heuristic - it is the proven global optimum, because every surrogate
  structure that could conceivably have beaten it has either been
  evaluated exactly (Phase 2) or provably ruled out (the bound argument
  above).

HONEST LIMITATION: for the RARE case where a candidate sub-route's drone
chain is too large to search exhaustively for the true-optimal delivery
order (see CERTIFY_MAX_EXHAUSTIVE_DRONE_CHAIN below), a heuristic order is
used instead for certification purposes. This is the one place this
implementation is not unconditionally exact - see certify_true_cost() for
detail and how to tighten it if your instances need long drone chains.
"""


import heapq
from typing import Dict, List, Optional, Tuple


# =============================================================================
# SETTINGS
# =============================================================================

N_CUSTOMERS = 13
RANDOM_SEED = 11
GRID_SIZE = 50
PHI0 = 1.0
PHI1 = 2.0
ENDURANCE = 25.0

TIME_LIMIT_SECONDS = 7200.0
MAX_BPC_NODES: Optional[int] = None
MAX_CONNECTIVITY_CUT_ROUNDS = 4
MAX_TRUCK_PATH_CUT_ROUNDS = 2
MAX_CUTS_PER_ROUND = 20
MAX_CG_ITER_PER_NODE = 500
MAX_SURROGATE_NEGATIVE_COLUMNS_PER_CALL = 20_000
# BUG FIX (found in production use at n=4): 300 was too small - a single
# pricing call can genuinely have far more than 300 distinct, legitimately
# useful negative-reduced-cost columns once n>=4, and cutting there can
# discard one the search actually needs. Confirmed directly: raising this
# to 20,000 fixed a validated n=4 instance that mismatched brute force at
# 300. Kept together with the deduplicate-by-signature fix in
# surrogate_pricer.py (both needed: dedup avoids wasting slots on redundant
# variants of the same signature, and this larger cap avoids losing
# genuinely distinct ones).

# Certification: how large a drone chain to search exhaustively for its
# true-optimal delivery order. Below this, every permutation is tried
# (exact). At or above it, a greedy nearest-insertion heuristic order is
# used instead - see the honest-limitation note in the module docstring.
CERTIFY_MAX_EXHAUSTIVE_DRONE_CHAIN = 7

P1_CACHE_FILE = "LAZY_CERT_P1_CACHE_13CUSTOMER.pkl"
FIGURE_FILE = "LAZY_CERT_BPC_13CUSTOMER.png"


# =============================================================================
# PHASE 2: TRUE-COST CERTIFICATION OF A COMPLETE SURROGATE-INTEGER TOUR
# =============================================================================

def best_true_cost_for_subroute(coords, zeta: Tuple[int, ...], drone_set: List[int],
                                 phi0: float, phi1: float, endurance: float,
                                 p1_cache: P1Cache) -> Optional[float]:
    """Exact (if drone_set is small enough) or heuristic-order true cost for
    one sub-route: truck length (order fixed by zeta, already determined by
    the surrogate solution) + best achievable waiting time over drone
    delivery orders.

    Uses the edge-enumeration P1 oracle (solve_excursion), NOT the direct-
    SLSQP-over-the-full-range oracle used elsewhere in this project's
    history. BUG FIX found during validation: the direct-SLSQP approach
    reported a KNOWN-feasible configuration as infeasible (confirmed with
    an independent manual re-solve of each edge-assignment sub-problem,
    one of which cleanly gave zero waiting) - it is a numerical multi-start
    reliability gap, not a real infeasibility, but it directly corrupted
    Phase 2 certification by permanently blacklisting a column that was
    part of the TRUE global optimum. The edge-enumeration oracle solves a
    small convex sub-problem for every explicit (launch edge, recovery
    edge, ...) assignment rather than hoping random restarts land in the
    right region of a single large optimization over the whole path, and
    is what caught the discrepancy. It costs more per call, which is
    acceptable here since certification only runs once per surrogate-
    integer solution found, not during the search itself.

    Returns None if NO order is feasible (endurance violated for every
    order tried) - the surrogate column must then be rejected as truly
    infeasible.
    """
    truck_len = route_length(coords, zeta) / phi0
    if not drone_set:
        return truck_len

    anchor_pts = [coords[n] for n in zeta]
    m = len(drone_set)
    best_wait = None
    if m <= CERTIFY_MAX_EXHAUSTIVE_DRONE_CHAIN:
        for order in itertools.permutations(drone_set):
            drone_pts = [coords[c] for c in order]
            res = solve_excursion(anchor_pts, drone_pts, phi0, phi1, endurance, n_restarts=6)
            if res is not None:
                if best_wait is None or res.waiting_time < best_wait:
                    best_wait = res.waiting_time
    else:
        # HONEST LIMITATION: exhaustive search over CERTIFY_MAX_EXHAUSTIVE_
        # DRONE_CHAIN! orders is not attempted above that size (factorial
        # blowup) - a greedy nearest-insertion order is used instead. This
        # is the one place this implementation can fail to find the true
        # exact optimum: if the true-optimal solution needs an order this
        # heuristic misses for a chain this long, the reported answer could
        # be a valid, feasible upper bound that is not the exact minimum
        # for that specific sub-route. Raise CERTIFY_MAX_EXHAUSTIVE_DRONE_
        # CHAIN if your instances need long drone chains and you can afford
        # the certification-time cost (only paid per surrogate-integer
        # solution found, not during the search itself).
        remaining = list(drone_set)
        order: List[int] = []
        cur_anchor = zeta[0]
        while remaining:
            nxt = min(remaining, key=lambda c: dist(coords, cur_anchor, c))
            order.append(nxt)
            remaining.remove(nxt)
            cur_anchor = nxt
        drone_pts = [coords[c] for c in order]
        res = solve_excursion(anchor_pts, drone_pts, phi0, phi1, endurance, n_restarts=6)
        if res is not None:
            best_wait = res.waiting_time

    if best_wait is None:
        return None
    return truck_len + best_wait







# =============================================================================
# MAIN BPC LOOP
# =============================================================================

class LazyCertificationBPC:
    def __init__(self):
        self.coords, self.customers, self.start, self.end = generate_instance(N_CUSTOMERS, RANDOM_SEED, GRID_SIZE)
        self.manager = ColumnManager(self.customers)
        self.p1_cache = P1Cache(P1_CACHE_FILE)
        self.best_ub = float("inf")          # TRUE (P1-certified) cost - the only thing ever reported
        self.best_solution: List[Column] = []
        self.best_bound = -float("inf")      # surrogate LP bound, valid lower bound on true optimum
        self.start_time = 0.0
        self.node_counter = 0
        self.total_surrogate_states = 0
        self.total_p1_certifications = 0
        self.known_infeasible_columns: set = set()  # BUG FIX: see try_certify_integer_solution

    # -------------------------------------------------------------------
    def column_generation_at_node(self, node: BranchNode, active: List[int],
                                   forbidden_columns: Optional[set] = None):
        forbidden_columns = forbidden_columns or set()
        local_active = list(dict.fromkeys(
            i for i in active
            if column_satisfies_node(self.manager.columns[i], node)
            and i not in self.known_infeasible_columns
            and i not in forbidden_columns
        ))
        it = 0
        while True:
            it += 1
            if time.time() - self.start_time >= TIME_LIMIT_SECONDS:
                return None, local_active, "TIME_LIMIT"
            if it > MAX_CG_ITER_PER_NODE:
                return None, local_active, "CG_ITER_LIMIT"

            rmp = solve_rmp(self.manager, local_active, node, self.customers, self.start, self.end)
            if rmp is None:
                return None, local_active, "INFEASIBLE"

            # cut separation (unchanged from Step 68 - still needed because
            # sub-route columns can form disconnected components)
            for _ in range(MAX_CONNECTIVITY_CUT_ROUNDS):
                new_cuts = separate_connectivity_cuts(rmp, self.customers, MAX_CUTS_PER_ROUND)
                new_cuts = [V for V in new_cuts if V not in node.cuts]
                if not new_cuts:
                    break
                node.cuts.extend(new_cuts)
                rmp = solve_rmp(self.manager, local_active, node, self.customers, self.start, self.end)
                if rmp is None:
                    return None, local_active, "INFEASIBLE"
            for _ in range(MAX_TRUCK_PATH_CUT_ROUNDS):
                new_tp = separate_truck_path_cuts(rmp, self.customers, MAX_CUTS_PER_ROUND)
                new_tp = [t for t in new_tp if t not in node.truck_path_cuts]
                if not new_tp:
                    break
                node.truck_path_cuts.extend(new_tp)
                rmp = solve_rmp(self.manager, local_active, node, self.customers, self.start, self.end)
                if rmp is None:
                    return None, local_active, "INFEASIBLE"

            neg, stats = surrogate_pricing(self.manager, self.coords, self.customers, self.start, self.end,
                                            PHI0, rmp, node, MAX_SURROGATE_NEGATIVE_COLUMNS_PER_CALL)
            self.total_surrogate_states += stats.get("states_explored", 0)
            exhaustive = stats.get("exhaustive", True)

            print(f"    CG {it:03d}: surrogateLP={rmp['obj']:.6f}, art={rmp['artificial_sum']:.2e}, "
                  f"cols={len(local_active):,}, cuts={len(node.cuts)}, tpcuts={len(node.truck_path_cuts)}, "
                  f"neg={len(neg)}, states={stats.get('states_explored', 0):,}, exhaustive={exhaustive}")

            added = 0
            for idx in neg:
                if idx not in local_active and idx not in self.known_infeasible_columns \
                        and idx not in forbidden_columns \
                        and column_satisfies_node(self.manager.columns[idx], node):
                    local_active.append(idx)
                    added += 1

            if added == 0 and not exhaustive:
                # BUG FIX: pricing hit its hard state-count safety cap
                # (MAX_SURROGATE_STATES_TOTAL in surrogate_pricer.py) before
                # finishing - it does NOT know whether a negative-reduced-
                # cost column exists beyond where it stopped. Declaring the
                # node CLOSED here would be a false proof; this must be
                # reported as unproven, exactly like a time limit.
                return None, local_active, "PRICING_NOT_EXHAUSTIVE"

            if added == 0:
                if rmp["artificial_sum"] > 1e-6:
                    print(f"      surrogate pricing exhausted but artificial_sum={rmp['artificial_sum']:.3e} "
                          f"> 0: node is genuinely infeasible under current columns/branching")
                    return None, local_active, "INFEASIBLE"
                return rmp, local_active, "CLOSED"

    # -------------------------------------------------------------------
    def try_certify_integer_solution(self, rmp, node: BranchNode) -> Tuple[str, Optional[int]]:
        """If the surrogate LP solution at this node is integral and forms a
        connected tour, certify its TRUE cost via Phase 2.

        Returns (status, column_to_forbid):
          ("NOT_INTEGER", None)   - nothing to certify, caller should branch.
          ("CERTIFIED", idx)      - true cost computed (best_ub updated if it
                                     improved). CRITICAL (see resolve_node):
                                     the caller must still exclude `idx` (one
                                     of this solution's columns) and RE-SOLVE
                                     the node - a surrogate LP happening to be
                                     integer does NOT mean it is optimal for
                                     this node, because the surrogate
                                     objective is not the true objective.
                                     Other integer solutions with surrogate
                                     cost between this node's bound and the
                                     (possibly just-improved) incumbent might
                                     still exist and certify to something
                                     better - only once the node's bound rises
                                     to meet the incumbent, or the LP goes
                                     fractional/infeasible, is the node
                                     actually done.
          ("REJECTED_RETRY", idx) - `idx` is a truly infeasible column (now
                                     permanently blacklisted); caller must
                                     re-solve excluding it.
        """
        x = {idx: val for idx, val in zip(rmp["active"], rmp["lambda"])}
        selected = [idx for idx, val in x.items() if val > 1.0 - 1e-6]
        if len(selected) == 0:
            return "NOT_INTEGER", None
        for idx, val in x.items():
            if 1e-6 < val < 1.0 - 1e-6:
                return "NOT_INTEGER", None
        selected_cols = [self.manager.columns[i] for i in selected]
        served = set()
        for c in selected_cols:
            served |= set(c.served)
        if served != set(self.customers):
            return "NOT_INTEGER", None
        path = build_solution_path(selected_cols, self.start, self.end)
        if path is None:
            return "NOT_INTEGER", None

        total = 0.0
        for col in path:
            sub_cost = best_true_cost_for_subroute(self.coords, col.zeta, list(col.drone_set),
                                                    PHI0, PHI1, ENDURANCE, self.p1_cache)
            if sub_cost is None:
                print(f"      Phase 2 certification: column {col.start}->{col.end} zeta={list(col.zeta)} "
                      f"drone={sorted(col.drone_set)} is TRULY infeasible (no delivery order satisfies "
                      f"endurance) - excluding it permanently and re-solving this node")
                self.known_infeasible_columns.add(col.idx)
                return "REJECTED_RETRY", col.idx
            total += sub_cost

        # BUG FIX (found during validation, structural): the surrogate LP
        # cannot distinguish "one continuous sub-route" from "the identical
        # physical truck path split into several separately-priced pieces" -
        # both have the same total truck length (surrogate cost). But
        # splitting restricts each excursion's launch/recovery window to
        # just its own piece of the path, which can only ever hurt (never
        # help) the achievable waiting time, since a wider window is a
        # superset of positions to choose from. Confirmed empirically: a
        # 3-piece decomposition and the single merged sub-route it came from
        # had IDENTICAL surrogate cost, but the decomposed version's
        # certified true cost was strictly worse. When exactly one sub-route
        # in the path carries a nonempty drone set (the common, and here the
        # relevant, case), re-certifying that excursion against the FULL
        # merged tour path (rather than just its own sub-route's zeta) can
        # only match or improve on the per-piece result - so it is always
        # safe to take the minimum of the two.
        drone_bearing = [c for c in path if c.drone_set]
        if len(drone_bearing) == 1:
            merged_zeta = self._merge_path_zeta(path)
            merged_cost = best_true_cost_for_subroute(self.coords, merged_zeta, list(drone_bearing[0].drone_set),
                                                        PHI0, PHI1, ENDURANCE, self.p1_cache)
            if merged_cost is not None:
                merged_total = merged_cost  # merged_cost already covers the FULL path's truck
                                             # length (route_length(merged_zeta)) plus waiting -
                                             # BUG (caught immediately in testing): previously
                                             # added the other pieces' truck length on top of
                                             # this, double-counting it.
                if merged_total < total - 1e-7:
                    print(f"      Phase 2 certification: merged-path re-check improved true cost "
                          f"{total:.6f} -> {merged_total:.6f} (wider excursion window)")
                    total = merged_total
        # HONEST LIMITATION: when MORE than one sub-route in the path
        # carries a nonempty drone set, this fix does not attempt a joint
        # re-optimization across all of them (that would require solving
        # one combined, sequentially-ordered multi-excursion problem over
        # the full merged path, which is not implemented here) - the
        # per-sub-route result is used as-is in that case. This means, in
        # that specific multi-excursion-in-one-tour scenario, this
        # implementation could report a valid, feasible, but not
        # provably-minimal true cost. See the module docstring.

        self.total_p1_certifications += 1
        print(f"      Phase 2 certification: surrogate solution -> TRUE cost = {total:.6f}")
        if total < self.best_ub - 1e-7:
            self.best_ub = total
            self.best_solution = path
            print(f"      *** NEW CERTIFIED INCUMBENT: {self.best_ub:.6f} ***")
        # BUG FIX: this specific combination is now "used up" for THIS node -
        # forbid one of its columns (excluding any one column makes this
        # EXACT combination unselectable again) so re-solving is forced to
        # look for a genuinely different combination, if one exists.
        return "CERTIFIED", selected[0]

    def _merge_path_zeta(self, path: List[Column]) -> Tuple[int, ...]:
        """Concatenate a full, connected list of sub-route columns into one
        continuous truck-visit sequence, without duplicating shared junction
        nodes (each sub-route's start equals the previous one's end)."""
        merged: List[int] = list(path[0].zeta)
        for c in path[1:]:
            merged.extend(c.zeta[1:])  # skip the repeated junction node
        return tuple(merged)

    # -------------------------------------------------------------------
    def resolve_node(self, node: BranchNode, active_in: List[int]):
        """Solve a node to one of three definitive outcomes:
          ("BRANCH", rmp)     - fractional LP, still below incumbent; caller
                                 should branch on it.
          ("PRUNED", None)    - LP bound (after excluding every surrogate-
                                 integer solution already certified at this
                                 node) meets or exceeds the current TRUE
                                 incumbent; nothing more to find here.
          ("INFEASIBLE", None) - genuinely infeasible under this branch.
        Internally loops: certify every integer solution found (updating the
        global incumbent as it goes), excluding each one from consideration
        before re-solving, until the node's own bound catches up to the
        incumbent or the LP turns fractional. This loop is what makes
        lazy certification correct: a surrogate LP being integer is NOT by
        itself proof that this node cannot do better (see
        try_certify_integer_solution's docstring)."""
        forbidden: set = set()
        while True:
            rmp, local_active, cg_status = self.column_generation_at_node(node, active_in, forbidden)
            if cg_status in ("TIME_LIMIT", "CG_ITER_LIMIT", "PRICING_NOT_EXHAUSTIVE"):
                return cg_status, None
            if rmp is None:
                return "INFEASIBLE", None

            surrogate_lb = rmp["obj"]
            print(f"  Node surrogate LP bound={surrogate_lb:.6f} (forbidden={len(forbidden)}), "
                  f"true incumbent={self.best_ub:.6f}, artificial={rmp['artificial_sum']:.3e}")
            if surrogate_lb >= self.best_ub - 1e-7:
                return "PRUNED", None

            cert_status, col_to_forbid = self.try_certify_integer_solution(rmp, node)
            if cert_status == "NOT_INTEGER":
                return "BRANCH", rmp
            # REJECTED_RETRY or CERTIFIED: exclude the flagged column
            # (permanently if infeasible, or just for the rest of this
            # node's exploration if it was a valid but already-certified
            # combination) and loop back to re-solve.
            forbidden.add(col_to_forbid)
            active_in = local_active

    # -------------------------------------------------------------------
    # -------------------------------------------------------------------
    def solve(self):
        self.start_time = time.time()
        active, init_surrogate_cost, init_route = build_initial_columns(
            self.manager, self.coords, self.customers, self.start, self.end, PHI0)

        # the initial nearest-neighbor truck-only route has NO drone
        # customers, so its surrogate cost already equals its true cost
        # exactly (zero waiting is trivially correct with an empty chain) -
        # safe to seed best_ub directly from it.
        self.best_ub = init_surrogate_cost
        full_idx = self.manager.key_to_idx.get((tuple(init_route), tuple()))
        if full_idx is not None:
            self.best_solution = [self.manager.columns[full_idx]]

        root = BranchNode(node_id=0)
        pq = [(0.0, 0, root, active)]
        processed = 0
        status = "PROVEN_OPTIMAL"

        print("=" * 100)
        print("LAZY-CERTIFICATION BPC FOR ES-TSPD (surrogate search, P1-certified incumbents)")
        print(f"Customers={N_CUSTOMERS}, phi0={PHI0}, phi1={PHI1}, endurance={ENDURANCE}")
        print(f"Initial TRUE incumbent (truck-only route) = {self.best_ub:.6f}")
        print("=" * 100)

        while pq:
            if time.time() - self.start_time >= TIME_LIMIT_SECONDS:
                status = "TIME_LIMIT_NOT_PROVEN"
                break
            if MAX_BPC_NODES is not None and processed >= MAX_BPC_NODES:
                status = "NODE_LIMIT_NOT_PROVEN"
                break

            surrogate_lb_parent, _, node, active_in = heapq.heappop(pq)
            processed += 1
            self.node_counter = max(self.node_counter, node.node_id)

            # Global lower bound invariant: in a min-heap best-first search,
            # whatever is popped next always has the smallest priority among
            # everything not yet processed, so it IS the current live lower
            # bound on the true optimum (once the queue empties, every
            # branch has been closed/pruned against best_ub, so bound ==
            # incumbent and the result is proven, set after the loop).
            self.best_bound = surrogate_lb_parent

            # PRUNE using the TRUE incumbent (Z_upper), not any surrogate-only
            # bookkeeping - this is the crux of the lazy-certification
            # argument: surrogate_lb_parent <= true cost of any completion,
            # so if it already exceeds the best PROVEN-ACHIEVABLE true cost,
            # nothing under this node can possibly improve on it.
            if surrogate_lb_parent >= self.best_ub - 1e-7:
                continue

            print(f"\n--- BPC NODE {node.node_id} depth={node.depth} "
                  f"surrogate_parent_lb={surrogate_lb_parent:.6f} true_incumbent={self.best_ub:.6f} ---")
            print(f"  FD={sorted(node.forced_drone)} FT={sorted(node.forced_truck)} "
                  f"forb_arcs={sorted(node.forbidden_endpoint_arcs)} req_arcs={sorted(node.forced_endpoint_arcs)}")

            outcome, rmp = self.resolve_node(node, active_in)
            if outcome in ("TIME_LIMIT", "CG_ITER_LIMIT", "PRICING_NOT_EXHAUSTIVE"):
                status = f"{outcome}_NOT_PROVEN"
                break
            if outcome in ("INFEASIBLE", "PRUNED"):
                print(f"  Node {outcome.lower()}.")
                continue

            # outcome == "BRANCH": rmp is fractional and still below the
            # incumbent bound - branch on it.
            local_active = rmp["active"]
            surrogate_lb = rmp["obj"]
            x = {idx: val for idx, val in zip(rmp["active"], rmp["lambda"])}
            branch = self.choose_branch(x, node)
            if branch is None:
                continue
            print(f"  Branch: {branch}")
            left, right = self.make_children(node, branch)
            heapq.heappush(pq, (surrogate_lb, left.node_id, left, list(local_active)))
            heapq.heappush(pq, (surrogate_lb, right.node_id, right, list(local_active)))

        if not pq and status == "PROVEN_OPTIMAL":
            self.best_bound = self.best_ub  # tree fully exhausted: bound == incumbent, proven

        elapsed = time.time() - self.start_time
        self.p1_cache.save()
        gap = (self.best_ub - self.best_bound) / max(abs(self.best_ub), 1e-9) if math.isfinite(self.best_bound) else float("inf")

        print("\n" + "=" * 100)
        print("LAZY-CERTIFICATION BPC FINAL RESULT")
        print("=" * 100)
        print(f"Final status             : {status}")
        print(f"Best TRUE objective       : {self.best_ub:.9f}")
        print(f"Best surrogate bound      : {self.best_bound:.9f}")
        print(f"Relative gap              : {gap:.6e}")
        print(f"Processed BPC nodes       : {processed}")
        print(f"Generated columns         : {len(self.manager.columns)}")
        print(f"Phase-2 P1 certifications : {self.total_p1_certifications}")
        print(f"P1 cache hits / misses    : {self.p1_cache.hits} / {self.p1_cache.misses}")
        print(f"P1 solve seconds          : {self.p1_cache.solve_seconds:.3f}")
        print(f"Total surrogate DP states : {self.total_surrogate_states:,}")
        print(f"Elapsed seconds           : {elapsed:.3f}")
        if self.best_solution:
            print("\nSelected sub-route columns in path order:")
            for i, col in enumerate(self.best_solution, 1):
                print(f"  {i}. {col.start}->{col.end} | zeta={list(col.zeta)} beta={list(col.beta)} "
                      f"truck={sorted(col.truck_set)} drone={sorted(col.drone_set)}")
        return {"status": status, "best_ub": self.best_ub, "best_bound": self.best_bound,
                "processed_nodes": processed, "elapsed": elapsed}

    # -------------------------------------------------------------------
    def choose_branch(self, x: Dict[int, float], node: BranchNode):
        for i in self.customers:
            if i in node.forced_drone or i in node.forced_truck:
                continue
            val = sum(v for idx, v in x.items() if i in self.manager.columns[idx].drone_set)
            if 1e-6 < val < 1.0 - 1e-6:
                return ("mode", i, val)
        arc_vals: Dict[Tuple[int, int], float] = {}
        for idx, v in x.items():
            if v <= 1e-9:
                continue
            arc = self.manager.columns[idx].endpoint_arc
            arc_vals[arc] = arc_vals.get(arc, 0.0) + v
        cands = [(min(v, 1 - v), arc, v) for arc, v in arc_vals.items() if 1e-6 < v < 1.0 - 1e-6]
        if cands:
            _, arc, val = max(cands, key=lambda t: t[0])
            return ("arc", arc, val)
        return None

    def make_children(self, parent: BranchNode, branch):
        kind, item, _ = branch
        self.node_counter += 1
        left = parent.copy(self.node_counter)
        self.node_counter += 1
        right = parent.copy(self.node_counter)
        if kind == "mode":
            left.forced_truck.add(item)
            right.forced_drone.add(item)
        elif kind == "arc":
            left.forbidden_endpoint_arcs.add(item)
            right.forced_endpoint_arcs.add(item)
        return left, right


def main():
    solver = LazyCertificationBPC()
    solver.solve()


if __name__ == "__main__":
    main()
