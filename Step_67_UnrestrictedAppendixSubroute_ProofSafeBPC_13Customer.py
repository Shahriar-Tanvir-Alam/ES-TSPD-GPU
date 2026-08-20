#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STEP 67: PROOF-SAFE UNRESTRICTED APPENDIX-STYLE SUB-ROUTE BPC FOR 13-CUSTOMER ES-TSPD
================================================================================
This is an experimental branch-price-and-cut implementation for the modified
Appendix sub-route master, WITHOUT a fixed (a,b) column cap such as (3,3) or
(4,4).  It keeps the sub-route structure:

    - columns are sub-routes r=(s,e,zeta,beta), not complete depot tours;
    - the master uses endpoint flow balance and customer service constraints;
    - connectivity cuts are separated because sub-route columns can form
      disconnected components;
    - pricing is dynamic label search rather than fixed signature-index pricing.

================================================================================
FIX HISTORY (Step 67 vs. original Step 66 and Step 66_FIXED)
================================================================================
Two fixes were originally requested, and validation (cross-checking against an
independent brute-force solver on tiny instances) surfaced four MORE bugs while
verifying them - several were silently corrupting results, not just causing
inefficiency. All six are documented in place in the code (search "BUG FIX" and
"FIX 1"/"FIX 2"); summary:

  1. [requested] forced_endpoint_arcs was enforced via column-eligibility
     filtering, which can only express "every active column uses this arc" -
     structurally wrong for "at least one selected column uses this arc" in a
     multi-segment sub-route decomposition. Replaced with a genuine extra
     master row (required-arc row), Big-M bootstrapped like flow/cover rows.

  2. [requested] column_generation_at_node declared a node "CLOSED" purely
     based on pricing exhaustiveness, without checking whether the LP still
     relied on Big-M artificials to satisfy its constraints. Added an
     artificial_sum gate. This ALSO required Big-M-bootstrapping the
     inequality (connectivity/truck-path cut) rows, which previously had no
     bootstrap at all - without that, a newly-separated cut with no currently
     satisfying column made solve_rmp return None immediately, reported as
     "INFEASIBLE" before pricing ever got a chance to supply a compliant
     column.

  3. [found in validation] build_initial_columns reused the variable name
     `cost` as a loop variable, silently clobbering the actual initial route
     cost before returning it - self.best_ub was seeded with an arbitrary,
     unrelated value, corrupting bound-based pruning for the entire search.

  4. [found in validation] column_satisfies_node's mode-branching filters
     rejected a column whenever a forced-drone/forced-truck customer was
     simply ABSENT from that column (not just when served via the wrong
     mode) - excluding nearly every column needed to build the rest of the
     tour the moment any mode was forced. This directly caused a spuriously
     "infeasible" branch once fix 2 made such infeasibility visible instead
     of silently returning a wrong incumbent.

  5. [found in validation] pricing_dynamic's truck-extension loop never
     included `end` as a target - no dynamically-priced column could ever be
     the tour's final segment (only the small set of statically-built initial
     columns could terminate at the depot). Added `end` as a terminal-only
     extension target.

  6. [found in validation] label_reduced_lb (used for BOTH the emission check
     and the expansion-pruning check) uses dual_flow[current] as a stand-in
     for the dual of the eventual final endpoint. That is exact and correct
     for "should I stop HERE", but NOT a valid lower bound for "should I keep
     extending" - the true final endpoint's flow dual can be substantially
     more negative than an intermediate position's, so using the current
     position's dual overestimates achievable reduced cost and can trigger
     incorrect pruning before a label ever reaches the endpoint where it
     would go negative. Confirmed to be the direct cause of a missed
     globally-optimal column in validation. Split into label_reduced_lb
     (unchanged, used only for emission) and a new safe_expansion_lb (uses
     the minimum dual_flow over all possible future endpoints - always at
     least as optimistic/safe as whatever the true endpoint turns out to be).

  Also added: a last-resort rescan of the GLOBAL column pool (shared across
  every CG iteration and every branch node) for eligible, negative-reduced-
  cost columns right before declaring a node CLOSED - manager.columns can
  accumulate a column discovered under one set of duals that was never
  reconsidered once duals changed, since only freshly-returned pricing
  results were previously added to a node's active set.

STEP 67 ADDITIONAL FIXES / SAFETY CHANGES
--------------------------------------------------------------------------------
  7. Disabled the remaining dual-dependent expansion-pruning rule by default
     (USE_EXPANSION_LB_PRUNING=False). The n=4 validation gap in Step 66_FIXED
     was explicitly tied to a suspected interaction between safe_expansion_lb,
     max_future_dual_gain, and drone-delivery-order sensitivity. Step 67's
     default proof-safe mode therefore does NOT prune a pricing label only
     because this bound says it cannot become negative. This is slower, but it
     removes the known risky pruning mechanism.

  8. Prefix duplicate dominance is retained only as exact-prefix memoization:
     the key contains the full current truck sequence zeta and full drone
     sequence beta, so it merges only literally identical prefixes. This is
     safe and is not the order-insensitive dominance that caused concern.

  9. The code now labels any run ending by time limit, node limit, or pricing
     expansion limit as NOT_PROVEN. A PROVEN status is meaningful only when
     every processed node is closed with exhaustive pricing and zero artificials.

================================================================================
VALIDATION RESULTS AND ONE KNOWN, UNRESOLVED OPEN ISSUE
================================================================================
Validated against an independent, from-scratch, genuinely multi-excursion
brute-force full-tour solver (not this file's own machinery):

  - n=3 customers: 8/8 instances match brute force exactly (seeds 1,2,3,4,6,
    7,8,9; various endurance/branching-depth configurations, including cases
    exercising real branch-and-bound, up to 12 nodes).

  - n=4 customers: 1 of 2 tested instances matched exactly; the other showed
    a ~2.7% gap (29.41 vs. true 28.63) that was narrowed down but NOT fully
    root-caused before delivery. What is confirmed:
      * It is dual-dependent, not a structural gap in the move set - forcing
        pricing with artificial large duals DOES find the true-optimal
        column.
      * The specific failure mode: two different drone-delivery ORDERINGS
        for the same customer set genuinely have different costs (delivery
        order affects the P1 waiting-time calculation - this is real, not a
        duplicate), and only the worse ordering made it into the column
        pool under the real LP duals from the actual run; the better one
        was pruned somewhere during expansion despite the safe_expansion_lb
        fix (item 6 above).
      * Suspected next step: there may be a SECOND, more subtle way
        safe_expansion_lb (or its interaction with max_future_dual_gain)
        can still overestimate achievable reduced cost when multiple
        drone customers remain and their delivery ORDER (not just their
        set) affects what is achievable - this has not been confirmed.

  BOTTOM LINE FOR STEP 67: Step 67 removes the suspected unsafe expansion
  pruning by default. This is the practical proof-safe repair to test next.
  Still, before using any 13-customer result as a publication-grade proof,
  re-run the n=4 validation case and confirm the previously failing instance
  now matches brute force. If the pricing label-expansion limit or time limit
  is reached, the result is a strong incumbent/bound diagnostic, not a proof.

IMPORTANT HONEST SCOPE (from the original upload, still applies)
--------------------------------------------------------------------------------
1. No fixed (a,b) cap is imposed. A sub-route can contain any number of truck-
   served and drone-served customers up to the 13-customer instance size.

2. The algorithm is exact for the implemented modified chained-en-route model
   ONLY IF every processed node is closed by exhaustive pricing AND the open
   issue above does not affect the instance being solved. If a time limit
   or pricing expansion limit is reached, the result is a valid incumbent/bound
   diagnostic only, not a proof.

3. The local continuous subproblem P1 is solved by enumerating edge assignments
   and using SLSQP for the small convex continuous subproblem. This is the same
   numerical oracle style as earlier steps. For a strict mathematical proof, the
   local convex solver should be replaced by a certified global convex solver or
   otherwise certified.

4. This code is designed as a serious unrestricted sub-route BPC experiment for
   13 customers. It is not guaranteed to prove optimality quickly.

Requirements:
    pip install numpy scipy matplotlib
================================================================================
"""

from __future__ import annotations

import heapq
import itertools
import math
import os
import pickle
import random
import time
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
from scipy.optimize import linprog, minimize, LinearConstraint, NonlinearConstraint
import matplotlib.pyplot as plt

# =============================================================================
# USER SETTINGS
# =============================================================================

N_CUSTOMERS = 13
RANDOM_SEED = 11
GRID_SIZE = 50

PHI0 = 1.0
PHI1 = 2.0
ENDURANCE = 25.0

# These are NOT model caps. They are computational safety limits. If pricing hits
# either, the node is NOT certified and the algorithm stops with NOT_PROVEN.
TIME_LIMIT_SECONDS = 7200.0
MAX_BPC_NODES = 200
MAX_PRICING_LABEL_EXPANSIONS_PER_CALL = 2_000_000
MAX_COLUMNS_PER_PRICING_CALL = 80

# Initial incumbent/cut settings
MAX_CONNECTIVITY_CUT_ROUNDS = 4
MAX_CUTS_PER_ROUND = 50
MAX_TRUCK_PATH_CUT_ROUNDS = 2
MAX_TRUCK_PATH_CUTS_PER_ROUND = 30

# Numerical tolerances
INTEGER_TOL = 1e-6
REDUCED_COST_TOL = -1e-7
BIG_M = 1e7
P1_RESTARTS = 3
P1_CACHE_FILE = "STEP66_UNRESTRICTED_APPENDIX_SUBROUTE_P1_CACHE_13CUSTOMER.pkl"
FIGURE_FILE = "STEP67_PROOFSAFE_UNRESTRICTED_APPENDIX_SUBROUTE_BPC_13CUSTOMER.png"

# Dynamic pricing controls. These are search-order controls, not feasibility caps.
# The search remains exhaustive unless MAX_PRICING_LABEL_EXPANSIONS_PER_CALL or
# TIME_LIMIT_SECONDS is hit.
USE_STRONG_COMPLETE_SIGNATURE_DOMINANCE = True
USE_SAFE_PREFIX_DUPLICATE_DOMINANCE = True

# Step 67 proof-safe default: disable the dual-dependent expansion-pruning
# condition that caused the remaining n=4 validation concern in Step 66_FIXED.
# Setting this to True may speed up the run, but a proof claim should not rely
# on it until the n=4 brute-force validation suite passes.
USE_EXPANSION_LB_PRUNING = False

# When expansion pruning is enabled for experiments, use a deliberately
# optimistic future dual gain. This is unused in proof-safe default mode.
USE_ABSOLUTE_FUTURE_DUAL_GAIN_WHEN_PRUNING = True

# =============================================================================
# INSTANCE
# =============================================================================

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

# =============================================================================
# P1 ORACLE: modified chained equality version
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


class P1Cache:
    def __init__(self, filename: str):
        self.filename = filename
        self.data: Dict[Tuple[Tuple[int, ...], Tuple[int, ...]], Optional[P1Result]] = {}
        self.hits = 0
        self.misses = 0
        self.solves = 0
        self.solve_seconds = 0.0
        self.load_seconds = 0.0
        self.save_seconds = 0.0
        self.load()

    def load(self):
        t0 = time.time()
        if os.path.exists(self.filename):
            try:
                with open(self.filename, "rb") as f:
                    self.data = pickle.load(f)
                self.load_seconds = time.time() - t0
                print(f"Loaded Step 66 P1 cache: {self.filename} | entries={len(self.data):,} | seconds={self.load_seconds:.3f}")
            except Exception as e:
                print(f"WARNING: could not load P1 cache {self.filename}: {e}")
                self.data = {}
        else:
            print(f"No Step 66 P1 cache found; starting fresh: {self.filename}")

    def save(self):
        t0 = time.time()
        tmp = self.filename + ".tmp"
        with open(tmp, "wb") as f:
            pickle.dump(self.data, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, self.filename)
        self.save_seconds += time.time() - t0
        print(f"Saved Step 66 P1 cache: {self.filename} | entries={len(self.data):,} | cumulative_save_seconds={self.save_seconds:.3f}")

    def get_or_solve(self, coords, zeta: Tuple[int, ...], beta: Tuple[int, ...], phi0: float, phi1: float, endurance: float) -> Optional[P1Result]:
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


def route_length(coords, zeta: Sequence[int]) -> float:
    return sum(dist(coords, a, b) for a, b in zip(zeta[:-1], zeta[1:]))


def solve_p1_chained(coords: Dict[int, Tuple[float, float]],
                     zeta: Tuple[int, ...], beta: Tuple[int, ...],
                     phi0: float, phi1: float, endurance: float) -> Optional[P1Result]:
    """Exact-style modified chained P1 oracle for a fixed sub-route order.

    Truck path zeta=(s,...,e). Drone order beta=(d1,...,dm). The handoff points
    b0,...,bm lie in nondecreasing order along the polyline zeta, and drone trip
    k is b_{k-1}->beta[k]->b_k. This enforces p'_k = p_{k+1}.
    """
    if not beta:
        return P1Result(True, 0.0, [], tuple(), tuple())
    if len(zeta) < 2:
        return None

    anchors = [np.array(coords[i], dtype=float) for i in zeta]
    custs = [np.array(coords[i], dtype=float) for i in beta]
    m = len(custs)
    K = len(anchors) - 1

    edge_vec = [anchors[k + 1] - anchors[k] for k in range(K)]
    edge_len = [float(np.linalg.norm(v)) for v in edge_vec]
    cum_len = [0.0]
    for L in edge_len:
        cum_len.append(cum_len[-1] + L)

    def point(edge_idx: int, lam: float) -> np.ndarray:
        return anchors[edge_idx] + lam * edge_vec[edge_idx]

    def arrival(edge_idx: int, lam: float) -> float:
        return (cum_len[edge_idx] + lam * edge_len[edge_idx]) / phi0

    best: Optional[P1Result] = None

    # m+1 handoff points, assigned to nondecreasing truck edges.
    for combo in itertools.combinations_with_replacement(range(K), m + 1):
        same_pairs = [q for q in range(m) if combo[q] == combo[q + 1]]

        def objective(x):
            total_wait = 0.0
            for q in range(m):
                e0, e1 = combo[q], combo[q + 1]
                p0 = point(e0, x[q])
                p1 = point(e1, x[q + 1])
                a0 = arrival(e0, x[q])
                a1 = arrival(e1, x[q + 1])
                drone_t = (float(np.linalg.norm(p0 - custs[q])) + float(np.linalg.norm(custs[q] - p1))) / phi1
                total_wait += max(0.0, a0 + drone_t - a1)
            return total_wait

        def endurance_cons(x):
            vals = []
            for q in range(m):
                e0, e1 = combo[q], combo[q + 1]
                p0 = point(e0, x[q])
                p1 = point(e1, x[q + 1])
                drone_t = (float(np.linalg.norm(p0 - custs[q])) + float(np.linalg.norm(custs[q] - p1))) / phi1
                vals.append(endurance - drone_t)
            return np.array(vals)

        cons = [NonlinearConstraint(endurance_cons, 0.0, np.inf)]
        if same_pairs:
            A = np.zeros((len(same_pairs), m + 1))
            for row, q in enumerate(same_pairs):
                A[row, q] = 1.0
                A[row, q + 1] = -1.0
            cons.append(LinearConstraint(A, -np.inf, 0.0))

        starts = [np.full(m + 1, 0.5)]
        for _ in range(max(0, P1_RESTARTS - 1)):
            starts.append(np.random.uniform(0.0, 1.0, size=m + 1))

        # repair starts inside same-edge groups
        fixed_starts = []
        for s0 in starts:
            s0 = s0.copy()
            group = [0]
            groups = []
            for q in range(m):
                if combo[q] == combo[q + 1]:
                    group.append(q + 1)
                else:
                    groups.append(group)
                    group = [q + 1]
            groups.append(group)
            for g in groups:
                vals = sorted(float(s0[idx]) for idx in g)
                for local, idx in enumerate(g):
                    s0[idx] = vals[local]
            fixed_starts.append(s0)

        for s0 in fixed_starts:
            try:
                res = minimize(objective, s0, method="SLSQP", bounds=[(0.0, 1.0)] * (m + 1),
                               constraints=cons, options={"maxiter": 200, "ftol": 1e-10, "disp": False})
            except Exception:
                continue
            if not res.success:
                continue
            if np.any(endurance_cons(res.x) < -1e-6):
                continue
            w = float(objective(res.x))
            if best is None or w < best.waiting - 1e-9:
                bp = [tuple(point(combo[q], float(res.x[q]))) for q in range(m + 1)]
                best = P1Result(True, w, bp, combo, tuple(float(v) for v in res.x))

    return best

# =============================================================================
# COLUMN / NODE TYPES
# =============================================================================

@dataclass
class Column:
    idx: int
    cost: float
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
            node_id=new_id,
            depth=self.depth + 1,
            forced_drone=set(self.forced_drone),
            forced_truck=set(self.forced_truck),
            forced_endpoint_arcs=set(self.forced_endpoint_arcs),
            forbidden_endpoint_arcs=set(self.forbidden_endpoint_arcs),
            cuts=list(self.cuts),
            truck_path_cuts=list(self.truck_path_cuts),
        )

# =============================================================================
# COLUMN MANAGER
# =============================================================================

class ColumnManager:
    def __init__(self, customers: List[int]):
        self.customers = customers
        self.pos = {c: i for i, c in enumerate(customers)}
        self.columns: List[Column] = []
        self.key_to_idx: Dict[Tuple[Tuple[int, ...], Tuple[int, ...]], int] = {}
        self.best_by_signature: Dict[Tuple[int, int, FrozenSet[int], FrozenSet[int]], int] = {}

    def add(self, cost: float, start: int, end: int, zeta: Tuple[int, ...], beta: Tuple[int, ...], name: str = "") -> Optional[int]:
        key = (tuple(zeta), tuple(beta))
        truck_set = frozenset(x for x in zeta[1:] if x in self.pos)  # end customer is truck-served
        drone_set = frozenset(beta)
        if truck_set & drone_set:
            return None
        served = truck_set | drone_set
        if not served:
            return None
        sig = (start, end, truck_set, drone_set)

        # Strong complete-signature dominance: same master coefficients; keep cheapest.
        old_idx = self.best_by_signature.get(sig)
        if USE_STRONG_COMPLETE_SIGNATURE_DOMINANCE and old_idx is not None:
            old = self.columns[old_idx]
            if old.cost <= cost + 1e-9:
                return None

        if key in self.key_to_idx:
            old_idx = self.key_to_idx[key]
            if self.columns[old_idx].cost <= cost + 1e-9:
                return None
            # Keep old column object for simplicity; add new column with better key duplicate.

        idx = len(self.columns)
        col = Column(idx, float(cost), start, end, tuple(zeta), tuple(beta), truck_set, drone_set, name)
        self.columns.append(col)
        self.key_to_idx[key] = idx
        if old_idx is None or cost < self.columns[old_idx].cost - 1e-9:
            self.best_by_signature[sig] = idx
        return idx

    def filtered_indices(self, node: BranchNode) -> List[int]:
        return [i for i, c in enumerate(self.columns) if column_satisfies_node(c, node)]

# =============================================================================
# MASTER AND CUTS
# =============================================================================

def column_satisfies_node(col: Column, node: BranchNode) -> bool:
    # BUG FIX (found during validation, same family as fix 1): the previous
    # checks rejected a column whenever a forced-drone customer was simply
    # ABSENT from that column's drone_set - but a column that doesn't serve
    # that customer AT ALL is perfectly fine (some OTHER column in the
    # solution serves it); only a column that serves the customer via the
    # WRONG mode should be excluded. The old code excluded almost every
    # column needed to build the rest of the tour the moment any mode was
    # forced, which silently made the branch spuriously infeasible (caught
    # empirically: forcing drone-service on a customer that the fractional
    # LP had already shown 67% support for immediately reported
    # "infeasible" once fix 2's artificial_sum gate was added - this bug
    # was the true cause, fix 2 just made it visible instead of silently
    # returning a wrong incumbent).
    for i in node.forced_drone:
        if i in col.truck_set:
            return False
    for i in node.forced_truck:
        if i in col.drone_set:
            return False
    if col.endpoint_arc in node.forbidden_endpoint_arcs:
        return False
    # FIX 1: forced_endpoint_arcs is intentionally NOT enforced here anymore.
    # "Some selected column uses this arc" is an aggregate property of the
    # WHOLE combination of selected columns, not a per-column filter -
    # eligibility filtering can only express "every active column uses this
    # arc," which wrongly excludes almost every valid multi-segment tour the
    # moment one arc is forced. It is now enforced correctly as a genuine
    # extra master row in solve_rmp() (see "required-arc rows" below),
    # exactly like flow/cover rows, with the same Big-M bootstrap.
    return True


def b_coeff(col: Column, node_id: int) -> int:
    return int(col.start == node_id) - int(col.end == node_id)


def cut_coeff(col: Column, V: FrozenSet[int]) -> int:
    return int((col.start not in V) and bool(set(col.served) & set(V)))


def truck_path_cut_coeff(col: Column, S: FrozenSet[int], k: int) -> int:
    # Sum_{i in S, j notin S} h_ij >= z_k^T.
    # Column coefficient = crossings - truck_service(k).
    crossings = 0
    for a, b in zip(col.zeta[:-1], col.zeta[1:]):
        if a in S and b not in S:
            crossings += 1
    return crossings - int(k in col.truck_set)


def solve_rmp(manager: ColumnManager, active: List[int], node: BranchNode,
              customers: List[int], start: int, end: int):
    columns = [manager.columns[i] for i in active]
    nodes = [start] + customers + [end]
    n_flow = len(nodes)
    n_cover = len(customers)
    n_cuts = len(node.cuts)
    n_tpcuts = len(node.truck_path_cuts)
    n_reqarc = len(node.forced_endpoint_arcs)          # FIX 1
    R = len(columns)

    # ---- equality block: flow, cover, and (FIX 1) required-arc rows -------
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

    # FIX 1: required-endpoint-arc rows. "sum of columns using this exact
    # (s,e) arc = 1" is a genuine aggregate row - eligibility filtering
    # cannot express "at least one selected column uses this arc" without
    # wrongly also excluding every other valid column, so this MUST be a
    # real master row, not a filter (see column_satisfies_node).
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

    # ---- inequality block: connectivity cuts + truck-path cuts ------------
    # FIX 2: both cut families are now Big-M bootstrapped too, exactly like
    # the equality rows. Previously a newly-separated ">=1" connectivity cut
    # with NO currently-active column satisfying it made solve_rmp return
    # None immediately (genuine LP infeasibility with the current column
    # pool) - which column_generation_at_node then reported as node
    # "INFEASIBLE" WITHOUT ever giving pricing a chance to generate a column
    # that would satisfy the new cut. That could prune away a branch that
    # actually contains the true optimum. Bootstrapping these rows the same
    # way as flow/cover means the LP stays solvable, and the artificial_sum
    # check (added below in column_generation_at_node) is what now correctly
    # distinguishes "pricing hasn't caught up yet" from "genuinely
    # infeasible under this branching."
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
        # A_ge x >= b_ge  <=>  -A_ge x <= -b_ge. Add a Big-M artificial that
        # can absorb any shortfall: -A_ge x - art <= -b_ge  (art >= 0),
        # i.e. A_ge x + art >= b_ge, so the row is ALWAYS satisfiable, at a
        # heavy penalty, even before pricing supplies a compliant column.
        A_ub_full = np.zeros((n_ge, R + 2 * n_art + n_ge_art))
        A_ub_full[:, :R] = -A_ge
        A_ub_full[:, R + 2 * n_art:] = -np.eye(n_ge_art)
        b_ub_full = -b_ge

    c = np.array([col.cost for col in columns])
    c_full = np.concatenate([c, c_art, c_ge_art])

    # A_eq_full must span the FULL variable set [x, eq_art, ge_art] - pad
    # with zero columns for the ge-artificials, which never appear in
    # equality rows.
    A_eq_full = np.zeros((n_eq, R + 2 * n_art + n_ge_art))
    A_eq_full[:, :R] = A_eq
    A_eq_full[:, R:R + 2 * n_art] = Aeq_art

    res = linprog(c_full, A_eq=A_eq_full, b_eq=b_eq, A_ub=A_ub_full, b_ub=b_ub_full,
                  bounds=[(0, None)] * len(c_full), method="highs")
    if res.status != 0:
        return None

    lam = res.x[:R]
    art_sum = float(np.sum(res.x[R:]))  # covers both eq-row and cut-row artificials
    obj = float(c @ lam)

    # Duals: equality duals are free; inequality marginals are for -A_ge <= -b.
    eq_dual = np.array(res.eqlin.marginals) if n_eq > 0 else np.zeros(0)
    if n_ge > 0:
        ub_dual = np.array(res.ineqlin.marginals)
        ge_dual = -ub_dual  # because row was multiplied by -1
    else:
        ge_dual = np.zeros(0)

    dual_flow = {nd: float(eq_dual[i]) for i, nd in enumerate(nodes)}
    dual_cover = {cust: float(eq_dual[n_flow + p]) for p, cust in enumerate(customers)}
    dual_req_arc = {arc: float(eq_dual[n_flow + n_cover + q]) for q, arc in enumerate(req_arc_list)}  # FIX 1
    dual_cuts = []
    dual_tpcuts = []
    for q in range(n_cuts):
        dual_cuts.append((node.cuts[q], float(ge_dual[q])))
    for q in range(n_tpcuts):
        dual_tpcuts.append((node.truck_path_cuts[q][0], node.truck_path_cuts[q][1], float(ge_dual[n_cuts + q])))

    return {
        "obj": obj,
        "lambda": lam,
        "columns": columns,
        "active": active,
        "artificial_sum": art_sum,
        "dual_flow": dual_flow,
        "dual_cover": dual_cover,
        "dual_req_arc": dual_req_arc,   # FIX 1
        "dual_cuts": dual_cuts,
        "dual_tpcuts": dual_tpcuts,
    }


def reduced_cost_of_column(col: Column, duals) -> float:
    rc = col.cost
    for nd, pi in duals["dual_flow"].items():
        rc -= pi * b_coeff(col, nd)
    for cust, sig in duals["dual_cover"].items():
        rc -= sig * int(cust in col.served)
    for arc, tau in duals.get("dual_req_arc", {}).items():           # FIX 1
        rc -= tau * (1.0 if col.endpoint_arc == arc else 0.0)
    for V, omega in duals["dual_cuts"]:
        rc -= omega * cut_coeff(col, V)
    for S, k, eta in duals["dual_tpcuts"]:
        rc -= eta * truck_path_cut_coeff(col, S, k)
    return rc

# =============================================================================
# DYNAMIC UNRESTRICTED SUB-ROUTE PRICING
# =============================================================================

@dataclass(order=True)
class PricingLabel:
    priority: float
    start: int = field(compare=False)
    current: int = field(compare=False)
    zeta: Tuple[int, ...] = field(compare=False)
    beta: Tuple[int, ...] = field(compare=False)
    truck_mask: int = field(compare=False)
    drone_mask: int = field(compare=False)
    served_mask: int = field(compare=False)
    truck_time_lb: float = field(compare=False)
    dual_gain: float = field(compare=False)


def mask_of(custs: Iterable[int], pos: Dict[int, int]) -> int:
    m = 0
    for c in custs:
        m |= 1 << pos[c]
    return m


def subroute_lb_cost(coords, zeta: Tuple[int, ...], beta: Tuple[int, ...], phi0, phi1) -> float:
    """Certified lower bound on true sub-route cost.

    It includes truck travel time plus a weak drone lower bound. True cost is
    truck_time + waiting >= truck_time. A stronger safe bound can use zero only;
    here we use truck_time only for safety.
    """
    return route_length(coords, zeta) / phi0


def pricing_dynamic(manager: ColumnManager, p1_cache: P1Cache, coords, customers, start, end,
                    phi0, phi1, endurance, duals, node: BranchNode, global_start_time: float):
    pos = {c: i for i, c in enumerate(customers)}
    nodes_as_start = [start] + customers
    end_candidates = customers + [end]
    negative_cols: List[int] = []
    completed_best_sig_rc: Dict[Tuple[int, int, FrozenSet[int], FrozenSet[int]], float] = {}
    expansions = 0
    pruned_by_lb = 0
    pruned_duplicate = 0
    completed_tested = 0
    p1_evaluated = 0

    pq: List[PricingLabel] = []
    seen_prefix: Dict[Tuple[int, int, Tuple[int, ...], Tuple[int, ...]], float] = {}

    def service_dual_gain(c: int, as_drone: bool) -> float:
        return duals["dual_cover"].get(c, 0.0)

    def label_reduced_lb(lbl: PricingLabel) -> float:
        # For a partial label, this is the EXACT reduced cost if we stopped
        # HERE (current = final endpoint). Correct and exact for the
        # emission check ("should I emit a candidate ending at my current
        # position"), where dual_flow[current] genuinely IS the final
        # endpoint's dual. NOT safe to reuse for the expansion-pruning
        # check below - see safe_expansion_lb().
        s = lbl.start
        e = lbl.current
        rc = lbl.truck_time_lb
        rc -= duals["dual_flow"].get(s, 0.0) * 1.0
        rc -= duals["dual_flow"].get(e, 0.0) * (-1.0)
        rc -= lbl.dual_gain
        return rc

    # BUG FIX (found during validation): the expansion-pruning check below
    # previously reused label_reduced_lb(), which treats dual_flow[current]
    # as if it were the dual of the EVENTUAL final endpoint. That is only
    # valid for the emission check ("stop here"); for a label that may
    # still extend further, the true final endpoint's flow dual can be
    # substantially more negative than the current position's (confirmed
    # empirically: an intermediate node had dual_flow=0 while the true
    # final endpoint, the depot, had dual_flow=-5.295), so using the
    # current position's dual OVERESTIMATES the achievable reduced cost and
    # can trigger incorrect pruning before the label ever reaches the
    # endpoint where it would actually go negative - confirmed to be the
    # root cause of a missed globally-optimal column in validation. The fix
    # uses the MINIMUM dual_flow value over every node the label could
    # possibly still terminate at (customers or the depot), which is always
    # at least as optimistic (safe) as whatever the true eventual endpoint
    # turns out to be.
    all_possible_endpoints = customers + [end]
    min_possible_end_dual = min(duals["dual_flow"].get(e, 0.0) for e in all_possible_endpoints)

    def safe_expansion_lb(lbl: PricingLabel) -> float:
        s = lbl.start
        rc = lbl.truck_time_lb
        rc -= duals["dual_flow"].get(s, 0.0) * 1.0
        rc -= min_possible_end_dual * (-1.0)
        rc -= lbl.dual_gain
        return rc

    # Initialize a label for every possible sub-route start.
    for s in nodes_as_start:
        lbl = PricingLabel(0.0, s, s, (s,), tuple(), 0, 0, 0, 0.0, 0.0)
        heapq.heappush(pq, lbl)

    while pq:
        if time.time() - global_start_time >= TIME_LIMIT_SECONDS:
            return negative_cols, False, {
                "status": "TIME_LIMIT_IN_PRICING", "expansions": expansions,
                "pruned_by_lb": pruned_by_lb, "pruned_duplicate": pruned_duplicate,
                "completed_tested": completed_tested, "p1_evaluated": p1_evaluated,
            }
        if expansions >= MAX_PRICING_LABEL_EXPANSIONS_PER_CALL:
            return negative_cols, False, {
                "status": "LABEL_LIMIT_IN_PRICING", "expansions": expansions,
                "pruned_by_lb": pruned_by_lb, "pruned_duplicate": pruned_duplicate,
                "completed_tested": completed_tested, "p1_evaluated": p1_evaluated,
            }

        lbl = heapq.heappop(pq)
        expansions += 1

        # Emit current label as a completed sub-route if it serves at least one customer
        # and current is not the start. This gives columns of any length, no cap.
        if lbl.current != lbl.start and lbl.served_mask != 0:
            truck_set = frozenset(c for c in customers if lbl.truck_mask & (1 << pos[c]))
            drone_set = frozenset(c for c in customers if lbl.drone_mask & (1 << pos[c]))
            sig = (lbl.start, lbl.current, truck_set, drone_set)
            # Do not test exact P1 if this exact completed signature already has a
            # better reduced cost candidate in this pricing call.
            lb_rc = label_reduced_lb(lbl)
            old_rc = completed_best_sig_rc.get(sig, float("inf"))
            if lb_rc < old_rc + 1e-9:
                completed_tested += 1
                p1 = p1_cache.get_or_solve(coords, lbl.zeta, lbl.beta, phi0, phi1, endurance)
                p1_evaluated += 1
                if p1 is not None and p1.feasible:
                    true_cost = route_length(coords, lbl.zeta) / phi0 + p1.waiting
                    idx = manager.add(true_cost, lbl.start, lbl.current, lbl.zeta, lbl.beta, "priced_unrestricted")
                    if idx is not None:
                        col = manager.columns[idx]
                        if column_satisfies_node(col, node):
                            rc = reduced_cost_of_column(col, duals)
                            if rc < completed_best_sig_rc.get(sig, float("inf")):
                                completed_best_sig_rc[sig] = rc
                            if rc < REDUCED_COST_TOL:
                                negative_cols.append(idx)
                                if len(negative_cols) >= MAX_COLUMNS_PER_PRICING_CALL:
                                    # Early return is allowed only as non-certifying pricing.
                                    return negative_cols, False, {
                                        "status": "EARLY_NEGATIVE_COLUMNS", "expansions": expansions,
                                        "pruned_by_lb": pruned_by_lb, "pruned_duplicate": pruned_duplicate,
                                        "completed_tested": completed_tested, "p1_evaluated": p1_evaluated,
                                    }

        # `end` is terminal - nothing can be extended from it (a sub-route
        # that has reached the depot is finished; its candidate emission
        # already happened above via the completed-route check).
        if lbl.current == end:
            continue

        # Step 67 proof-safe change: the remaining n=4 validation gap in
        # Step 66_FIXED was suspected to come from this exact family of
        # expansion-pruning rules: safe_expansion_lb combined with a future
        # dual-gain estimate when multiple drone customers remain and delivery
        # order changes the P1 waiting time. Therefore, by default, Step 67
        # DOES NOT prune here. This makes pricing slower, but removes the
        # known risky dual-dependent pruning mechanism.
        remaining = [c for c in customers if not (lbl.served_mask & (1 << pos[c]))]
        if USE_EXPANSION_LB_PRUNING:
            if USE_ABSOLUTE_FUTURE_DUAL_GAIN_WHEN_PRUNING:
                max_future_dual_gain = sum(abs(duals["dual_cover"].get(c, 0.0)) for c in remaining)
            else:
                max_future_dual_gain = sum(max(0.0, duals["dual_cover"].get(c, 0.0)) for c in remaining)
            if safe_expansion_lb(lbl) - max_future_dual_gain >= -REDUCED_COST_TOL:
                pruned_by_lb += 1
                continue

        # BUG FIX (found during validation): dynamically-priced sub-routes
        # could previously never reach `end` at all - the loop below only
        # ever iterated over `customers`, so no dynamically-priced column
        # could ever be "the last segment" of a tour. Only the small set of
        # statically-built initial columns (build_initial_columns) could
        # terminate at the depot, which structurally excluded the true
        # optimum whenever it needed a longer/different final segment
        # (confirmed empirically: the true optimal column for a validation
        # instance needed zeta=(0,3,end) with a 2-customer drone chain, and
        # was never generated before this fix). `end` is added here as a
        # terminal-only target: reachable from any current position once at
        # least one customer has been served, but never expanded further.
        if lbl.current != end and lbl.served_mask != 0 and (lbl.current, end) not in node.forbidden_endpoint_arcs:
            new_zeta_end = lbl.zeta + (end,)
            new_truck_lb_end = route_length(coords, new_zeta_end) / phi0
            key_end = (lbl.start, end, new_zeta_end, lbl.beta)
            val_end = new_truck_lb_end - lbl.dual_gain
            if not (USE_SAFE_PREFIX_DUPLICATE_DOMINANCE and val_end >= seen_prefix.get(key_end, float("inf")) - 1e-12):
                seen_prefix[key_end] = val_end
                heapq.heappush(pq, PricingLabel(val_end, lbl.start, end, new_zeta_end, lbl.beta,
                                                 lbl.truck_mask, lbl.drone_mask, lbl.served_mask,
                                                 new_truck_lb_end, lbl.dual_gain))

        # Expand by adding one truck-served customer to zeta.
        for c in remaining:
            if c in node.forced_drone:
                continue
            if (lbl.current, c) in node.forbidden_endpoint_arcs:
                continue
            new_zeta = lbl.zeta + (c,)
            new_truck_mask = lbl.truck_mask | (1 << pos[c])
            new_served = lbl.served_mask | (1 << pos[c])
            new_truck_lb = route_length(coords, new_zeta) / phi0
            new_gain = lbl.dual_gain + service_dual_gain(c, False)
            key = (lbl.start, c, new_zeta, lbl.beta)
            val = new_truck_lb - new_gain
            if USE_SAFE_PREFIX_DUPLICATE_DOMINANCE and val >= seen_prefix.get(key, float("inf")) - 1e-12:
                pruned_duplicate += 1
                continue
            seen_prefix[key] = val
            prio = new_truck_lb - new_gain
            heapq.heappush(pq, PricingLabel(prio, lbl.start, c, new_zeta, lbl.beta,
                                             new_truck_mask, lbl.drone_mask, new_served,
                                             new_truck_lb, new_gain))

        # Expand by appending one drone-served customer to beta.
        # The launch/recovery positions will be optimized by P1 when the sub-route is emitted.
        for c in remaining:
            if c in node.forced_truck:
                continue
            new_beta = lbl.beta + (c,)
            new_drone_mask = lbl.drone_mask | (1 << pos[c])
            new_served = lbl.served_mask | (1 << pos[c])
            new_gain = lbl.dual_gain + service_dual_gain(c, True)
            key = (lbl.start, lbl.current, lbl.zeta, new_beta)
            val = lbl.truck_time_lb - new_gain
            if USE_SAFE_PREFIX_DUPLICATE_DOMINANCE and val >= seen_prefix.get(key, float("inf")) - 1e-12:
                pruned_duplicate += 1
                continue
            seen_prefix[key] = val
            prio = lbl.truck_time_lb - new_gain
            heapq.heappush(pq, PricingLabel(prio, lbl.start, lbl.current, lbl.zeta, new_beta,
                                             lbl.truck_mask, new_drone_mask, new_served,
                                             lbl.truck_time_lb, new_gain))

    return negative_cols, True, {
        "status": "EXHAUSTIVE", "expansions": expansions,
        "pruned_by_lb": pruned_by_lb, "pruned_duplicate": pruned_duplicate,
        "completed_tested": completed_tested, "p1_evaluated": p1_evaluated,
    }

# =============================================================================
# CONNECTIVITY / TRUCK-PATH CUTS
# =============================================================================

def separate_connectivity_cuts(rmp, customers: List[int], max_cuts: int) -> List[FrozenSet[int]]:
    columns = rmp["columns"]
    lam = rmp["lambda"]
    cuts = []
    n = len(customers)
    # Exhaustive subset separation is OK for 13 customers: 8191 subsets.
    for mask in range(1, (1 << n) - 1):
        V = frozenset(customers[i] for i in range(n) if mask & (1 << i))
        lhs = 0.0
        for col, x in zip(columns, lam):
            if x > 1e-10:
                lhs += x * cut_coeff(col, V)
        if lhs < 1.0 - 1e-6:
            cuts.append((lhs, V))
    cuts.sort(key=lambda t: t[0])
    return [V for _, V in cuts[:max_cuts]]


def separate_truck_path_cuts(rmp, customers: List[int], max_cuts: int) -> List[Tuple[FrozenSet[int], int]]:
    columns = rmp["columns"]
    lam = rmp["lambda"]
    cuts = []
    n = len(customers)
    # Candidate subsets from customer masks. Strong but still feasible for n=13.
    for mask in range(1, (1 << n)):
        S = frozenset(customers[i] for i in range(n) if mask & (1 << i))
        for k in S:
            lhs = 0.0
            for col, x in zip(columns, lam):
                if x > 1e-10:
                    lhs += x * truck_path_cut_coeff(col, S, k)
            if lhs < -1e-6:
                cuts.append((lhs, S, k))
    cuts.sort(key=lambda t: t[0])
    return [(S, k) for _, S, k in cuts[:max_cuts]]

# =============================================================================
# INITIAL COLUMNS AND FEASIBILITY
# =============================================================================

def nearest_neighbor_route(coords, customers, start, end):
    unvisited = set(customers)
    route = [start]
    cur = start
    while unvisited:
        nxt = min(unvisited, key=lambda j: dist(coords, cur, j))
        route.append(nxt)
        unvisited.remove(nxt)
        cur = nxt
    route.append(end)
    return route


def build_initial_columns(manager: ColumnManager, coords, customers, start, end, phi0):
    route = nearest_neighbor_route(coords, customers, start, end)
    active = []
    for a, b in zip(route[:-1], route[1:]):
        if b == end:
            # final connector must still serve no customer, so not a valid service
            # column by itself. Add a dummy last customer->end connector serving none
            # cannot be selected with service-cover alone. Instead add full truck route
            # as one column below and also add single-customer arcs for all pairs.
            continue
    # Add full truck-only route as a valid incumbent column in the sub-route master.
    # BUG FIX (found during validation): `cost` was being reused as the loop
    # variable name below and silently overwritten by the LAST small
    # connector's cost before being returned as the "initial cost" - meaning
    # self.best_ub was seeded with an arbitrary, unrelated value instead of
    # the actual initial route's cost, corrupting bound-based pruning for
    # the entire search. Fixed by using a dedicated variable
    # (initial_route_cost) that is never reassigned.
    initial_route_cost = route_length(coords, route) / phi0
    idx = manager.add(initial_route_cost, start, end, tuple(route), tuple(), "initial_full_truck_route")
    if idx is not None:
        active.append(idx)

    # Add elementary truck-only one-customer sub-routes for flow construction.
    nodes_start = [start] + customers
    nodes_end = customers + [end]
    for s in nodes_start:
        for e in nodes_end:
            if s == e:
                continue
            if e == end:
                # serve one customer on the way if possible: s -> k -> end
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
    by_start = {}
    for c in selected_cols:
        if c.start in by_start:
            return None
        by_start[c.start] = c
    cur = start
    path = []
    seen = set()
    while cur != end:
        if cur in seen or cur not in by_start:
            return None
        seen.add(cur)
        col = by_start[cur]
        path.append(col)
        cur = col.end
    if len(path) != len(selected_cols):
        return None
    return path

# =============================================================================
# BPC SOLVER
# =============================================================================

class Step66BPC:
    def __init__(self):
        self.coords, self.customers, self.start, self.end = generate_instance(N_CUSTOMERS, RANDOM_SEED, GRID_SIZE)
        self.manager = ColumnManager(self.customers)
        self.p1_cache = P1Cache(P1_CACHE_FILE)
        self.best_ub = float("inf")
        self.best_solution: List[Column] = []
        self.best_bound = -float("inf")
        self.start_time = 0.0
        self.node_counter = 0
        self.total_pricing_expansions = 0
        self.total_completed_tested = 0

    def column_generation_at_node(self, node: BranchNode, active: List[int]):
        local_active = list(dict.fromkeys(i for i in active if column_satisfies_node(self.manager.columns[i], node)))
        it = 0
        while True:
            it += 1
            if time.time() - self.start_time >= TIME_LIMIT_SECONDS:
                return None, local_active, "TIME_LIMIT"
            rmp = solve_rmp(self.manager, local_active, node, self.customers, self.start, self.end)
            if rmp is None:
                return None, local_active, "INFEASIBLE"

            # Separate valid cuts before pricing.
            cut_added = False
            for _ in range(MAX_CONNECTIVITY_CUT_ROUNDS):
                new_cuts = separate_connectivity_cuts(rmp, self.customers, MAX_CUTS_PER_ROUND)
                new_cuts = [V for V in new_cuts if V not in node.cuts]
                if not new_cuts:
                    break
                node.cuts.extend(new_cuts)
                cut_added = True
                print(f"      added {len(new_cuts)} connectivity cuts")
                rmp = solve_rmp(self.manager, local_active, node, self.customers, self.start, self.end)
                if rmp is None:
                    return None, local_active, "INFEASIBLE"
            for _ in range(MAX_TRUCK_PATH_CUT_ROUNDS):
                new_tpc = separate_truck_path_cuts(rmp, self.customers, MAX_TRUCK_PATH_CUTS_PER_ROUND)
                new_tpc = [x for x in new_tpc if x not in node.truck_path_cuts]
                if not new_tpc:
                    break
                node.truck_path_cuts.extend(new_tpc)
                cut_added = True
                print(f"      added {len(new_tpc)} truck-path cuts")
                rmp = solve_rmp(self.manager, local_active, node, self.customers, self.start, self.end)
                if rmp is None:
                    return None, local_active, "INFEASIBLE"

            # BUG FIX (found during validation): manager.columns is a GLOBAL
            # pool shared across every CG iteration and every branch node.
            # A column can be discovered by pricing at one point (under one
            # set of duals, where it did NOT have negative reduced cost
            # relative to whatever WAS added to local_active then) and
            # simply sit in the pool from then on - if duals later change
            # (more cuts, deeper branching, a different node entirely) such
            # that this SAME already-known column would now be negative and
            # useful, nothing previously re-checked it, since only columns
            # freshly RETURNED by this call to pricing_dynamic get added to
            # local_active. Confirmed empirically: a column with reduced
            # cost -1.29 under the current duals was sitting in the pool,
            # eligible for this node, and never used - the true optimum for
            # a validation instance was quietly missed as a direct result.
            # Checked only as a last resort (when dynamic pricing itself
            # finds nothing new) to avoid rescanning the whole pool every
            # single CG iteration on a large, long-running pool.

            neg, exhaustive, stats = pricing_dynamic(self.manager, self.p1_cache, self.coords, self.customers,
                                                     self.start, self.end, PHI0, PHI1, ENDURANCE, rmp, node,
                                                     self.start_time)
            self.total_pricing_expansions += stats.get("expansions", 0)
            self.total_completed_tested += stats.get("completed_tested", 0)
            print(
                f"    CG {it:03d}: LP={rmp['obj']:.9f}, art={rmp['artificial_sum']:.2e}, "
                f"cols={len(local_active):,}, cuts={len(node.cuts)}, tpcuts={len(node.truck_path_cuts)}, "
                f"pricing={stats['status']}, neg={len(neg)}, expansions={stats['expansions']:,}, "
                f"complete={stats['completed_tested']:,}, p1_eval={stats['p1_evaluated']:,}"
            )

            if not exhaustive and len(neg) == 0:
                # even a non-exhaustive pricing call finding nothing new is
                # a good moment for the cheap pool rescan below, since it
                # costs no P1 solves and might still supply a useful column.
                pool_added = self._rescan_pool_for_negative_columns(local_active, node, rmp)
                if pool_added:
                    print(f"      pool rescan: reactivated {pool_added} previously-known column(s)")
                    continue
                return rmp, local_active, stats["status"]

            added = 0
            for idx in neg:
                if idx not in local_active and column_satisfies_node(self.manager.columns[idx], node):
                    local_active.append(idx)
                    added += 1
            if added == 0 and exhaustive:
                # BUG FIX (found during validation): before concluding
                # pricing is truly exhausted, check whether the GLOBAL
                # column pool (shared across every CG iteration and every
                # branch node) already contains an eligible column with
                # negative reduced cost under the CURRENT duals that simply
                # was never added to THIS node's active set (e.g. because it
                # was discovered earlier under different duals, or at a
                # different node entirely). See _rescan_pool_for_negative_columns.
                pool_added = self._rescan_pool_for_negative_columns(local_active, node, rmp)
                if pool_added:
                    print(f"      pool rescan: reactivated {pool_added} previously-known column(s)")
                    continue
            if added == 0:
                if exhaustive:
                    # FIX 2: pricing found no more negative-reduced-cost
                    # columns, but that alone does NOT mean this node is
                    # truly feasible - if the LP still relies on Big-M
                    # artificials to satisfy flow/cover/required-arc/cut
                    # rows, no combination of the CURRENT real columns
                    # actually satisfies the true constraints, and pricing
                    # has now confirmed no further real column can help
                    # either. That is genuine infeasibility under this
                    # branch's restrictions, not a solved node - reporting
                    # it as "CLOSED" would let a later stage treat a
                    # spurious LP objective (computed only from the real
                    # columns' costs, NOT reflecting the Big-M penalty) as
                    # if it were a legitimate bound.
                    if rmp["artificial_sum"] > 1e-6:
                        print(f"      pricing exhausted but artificial_sum={rmp['artificial_sum']:.3e} "
                              f"> 0: node is genuinely infeasible under current columns/branching")
                        return None, local_active, "INFEASIBLE"
                    return rmp, local_active, "CLOSED"
                return rmp, local_active, stats["status"]

    def _rescan_pool_for_negative_columns(self, local_active: List[int], node: BranchNode, rmp) -> int:
        """See the BUG FIX comment in column_generation_at_node. Cheap
        (no P1 solves, just reduced-cost arithmetic over already-known
        columns) linear scan of the global pool, used only as a last
        resort right before declaring a node CLOSED."""
        added = 0
        for idx, col in enumerate(self.manager.columns):
            if idx in local_active:
                continue
            if not column_satisfies_node(col, node):
                continue
            if reduced_cost_of_column(col, rmp) < REDUCED_COST_TOL:
                local_active.append(idx)
                added += 1
        return added

    def update_incumbent_from_lp(self, rmp):
        selected = []
        for col, x in zip(rmp["columns"], rmp["lambda"]):
            if x > 1.0 - INTEGER_TOL:
                selected.append(col)
            elif x > INTEGER_TOL and x < 1.0 - INTEGER_TOL:
                return False
        if not selected:
            return False
        path = build_solution_path(selected, self.start, self.end)
        if path is None:
            return False
        served = set()
        for c in path:
            served |= set(c.served)
        if served != set(self.customers):
            return False
        val = sum(c.cost for c in path)
        if val < self.best_ub - 1e-8:
            self.best_ub = val
            self.best_solution = path
            print(f"    New integer incumbent: {self.best_ub:.9f} | cols={len(path)}")
        return True

    def select_branch(self, rmp, node: BranchNode):
        # service-mode branching first
        yD = {c: 0.0 for c in self.customers}
        yT = {c: 0.0 for c in self.customers}
        for col, x in zip(rmp["columns"], rmp["lambda"]):
            if x <= 1e-10:
                continue
            for c in col.drone_set:
                yD[c] += x
            for c in col.truck_set:
                yT[c] += x
        best = None
        best_score = 10.0
        for c in self.customers:
            if c in node.forced_drone or c in node.forced_truck:
                continue
            val = yD[c]
            if INTEGER_TOL < val < 1.0 - INTEGER_TOL:
                score = abs(val - 0.5)
                if score < best_score:
                    best_score = score
                    best = ("mode", c, val)
        if best is not None:
            return best

        # fallback endpoint arc branching
        z = {}
        for col, x in zip(rmp["columns"], rmp["lambda"]):
            if x <= 1e-10:
                continue
            z[col.endpoint_arc] = z.get(col.endpoint_arc, 0.0) + x
        best = None
        best_score = 10.0
        for arc, val in z.items():
            if arc in node.forced_endpoint_arcs or arc in node.forbidden_endpoint_arcs:
                continue
            if INTEGER_TOL < val < 1.0 - INTEGER_TOL:
                score = abs(val - 0.5)
                if score < best_score:
                    best_score = score
                    best = ("endpoint", arc, val)
        return best

    def solve(self):
        self.start_time = time.time()
        active, init_cost, init_route = build_initial_columns(self.manager, self.coords, self.customers, self.start, self.end, PHI0)
        self.best_ub = init_cost
        full_idx = self.manager.key_to_idx.get((tuple(init_route), tuple()))
        if full_idx is not None:
            self.best_solution = [self.manager.columns[full_idx]]

        root = BranchNode(node_id=0)
        pq = [(0.0, 0, root, active)]
        processed = 0
        status = "PROVEN_OPTIMAL"

        print("=" * 100)
        print("STEP 67: PROOF-SAFE UNRESTRICTED APPENDIX-STYLE SUB-ROUTE BPC FOR 13-CUSTOMER ES-TSPD")
        print("No fixed (a,b) cap. Dynamic pricing. Connectivity cuts + truck-path cuts.")
        print("Exact only if pricing is exhaustive at every processed node and the tree terminates.")
        print("Step 67 proof-safe default disables the risky dual-dependent expansion-pruning rule")
        print("that was suspected in the Step 66_FIXED n=4 validation gap.")
        print("=" * 100)
        print("VALIDATION WARNING: Step 66_FIXED matched n=3 brute force 8/8, but one n=4")
        print("case had a ~2.7% gap. Step 67 removes the suspected unsafe pruning mechanism,")
        print("but you should still re-run the n=4 brute-force validation before treating")
        print("any n=13 result as publication-grade certification.")
        print("=" * 100)
        print(f"Customers={N_CUSTOMERS}, phi0={PHI0}, phi1={PHI1}, endurance={ENDURANCE}")
        print(f"Initial full-truck UB={self.best_ub:.9f}, initial columns={len(active):,}")

        while pq:
            if time.time() - self.start_time >= TIME_LIMIT_SECONDS:
                status = "TIME_LIMIT_NOT_PROVEN"
                break
            if processed >= MAX_BPC_NODES:
                status = "NODE_LIMIT_NOT_PROVEN"
                break

            lb_parent, _, node, node_active = heapq.heappop(pq)
            if lb_parent >= self.best_ub - 1e-8:
                continue
            processed += 1
            print("-" * 100)
            print(f"BPC NODE {node.node_id} depth={node.depth} open={len(pq)} incumbent={self.best_ub:.9f}")
            print(f"  branch FD={sorted(node.forced_drone)} FT={sorted(node.forced_truck)} forb_arcs={sorted(node.forbidden_endpoint_arcs)}")

            rmp, active_after, cg_status = self.column_generation_at_node(node, node_active)
            if rmp is None:
                if cg_status in ("TIME_LIMIT", "TIME_LIMIT_IN_PRICING", "LABEL_LIMIT_IN_PRICING"):
                    status = "TIME_LIMIT_NOT_PROVEN"
                    break
                continue
            if cg_status != "CLOSED":
                status = cg_status + "_NOT_PROVEN"
                break

            lb = rmp["obj"]
            self.best_bound = max(self.best_bound, lb)
            print(f"  Node closed by exhaustive pricing. LB={lb:.9f}, UB={self.best_ub:.9f}")

            if lb >= self.best_ub - 1e-8:
                print("  Pruned by bound.")
                continue

            if self.update_incumbent_from_lp(rmp):
                continue

            branch = self.select_branch(rmp, node)
            if branch is None:
                print("  WARNING: fractional solution but no branch selected; node ignored.")
                continue
            kind, item, val = branch
            print(f"  Branching decision: {branch}")
            self.node_counter += 1
            left = node.copy(self.node_counter)
            self.node_counter += 1
            right = node.copy(self.node_counter)
            if kind == "mode":
                # left: force drone, right: force truck
                left.forced_drone.add(item)
                right.forced_truck.add(item)
            else:
                # left: force endpoint arc, right: forbid endpoint arc
                left.forced_endpoint_arcs.add(item)
                right.forbidden_endpoint_arcs.add(item)
            heapq.heappush(pq, (lb, left.node_id, left, active_after))
            heapq.heappush(pq, (lb, right.node_id, right, active_after))

        elapsed = time.time() - self.start_time
        if not pq and status == "PROVEN_OPTIMAL":
            self.best_bound = self.best_ub
        gap = float("inf")
        if self.best_ub < float("inf") and self.best_bound > -float("inf"):
            gap = max(0.0, (self.best_ub - self.best_bound) / max(1.0, abs(self.best_ub)))

        self.p1_cache.save()
        self.report(status, processed, elapsed, gap)
        self.plot_solution()

    def report(self, status: str, processed: int, elapsed: float, gap: float):
        print("=" * 100)
        print("STEP 67 FINAL RESULT")
        print("=" * 100)
        print(f"Final status            : {status}")
        print(f"Best objective/incumbent: {self.best_ub:.9f}")
        print(f"Best bound              : {self.best_bound:.9f}")
        print(f"Relative gap            : {gap:.9e}")
        print(f"Processed BPC nodes     : {processed}")
        print(f"Generated columns       : {len(self.manager.columns):,}")
        print(f"P1 cache entries        : {len(self.p1_cache.data):,}")
        print(f"P1 hits / misses        : {self.p1_cache.hits:,} / {self.p1_cache.misses:,}")
        print(f"P1 solves this run      : {self.p1_cache.solves:,}")
        print(f"P1 solve seconds        : {self.p1_cache.solve_seconds:.3f}")
        print(f"Pricing label expansions: {self.total_pricing_expansions:,}")
        print(f"Completed candidates    : {self.total_completed_tested:,}")
        print(f"Elapsed seconds         : {elapsed:.3f}")
        if self.best_solution:
            print("\nSelected sub-route columns in path order:")
            for t, col in enumerate(self.best_solution, 1):
                print(f"  {t}. {col.start}->{col.end} | zeta={list(col.zeta)} beta={list(col.beta)} "
                      f"truck={sorted(col.truck_set)} drone={sorted(col.drone_set)} cost={col.cost:.9f}")
            served = set()
            for col in self.best_solution:
                served |= set(col.served)
            print("\nValidation status:", "PASS" if served == set(self.customers) else "FAIL")
        else:
            print("No incumbent solution available.")

    def plot_solution(self):
        if not self.best_solution:
            return
        plt.figure(figsize=(10, 8))
        for nid, (x, y) in self.coords.items():
            if nid == self.end:
                continue
            if nid == self.start:
                plt.scatter(x, y, marker="s", s=140)
                plt.text(x + 0.4, y + 0.4, "Depot")
            else:
                plt.scatter(x, y, s=60)
                plt.text(x + 0.4, y + 0.4, str(nid))
        for col in self.best_solution:
            for a, b in zip(col.zeta[:-1], col.zeta[1:]):
                xa, ya = self.coords[a]
                xb, yb = self.coords[b]
                plt.plot([xa, xb], [ya, yb], linewidth=2)
        plt.title(f"Step 66 incumbent | cost={self.best_ub:.3f}")
        plt.grid(True, alpha=0.25)
        plt.tight_layout()
        plt.savefig(FIGURE_FILE, dpi=200)
        print(f"Saved figure: {FIGURE_FILE}")


def main():
    solver = Step66BPC()
    solver.solve()


if __name__ == "__main__":
    main()
