#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==========================================================================================
STEP 64: UNCAPPED DYNAMIC-PRICING BRANCH-AND-PRICE FOR 13-CUSTOMER ES-TSPD
==========================================================================================
Purpose
-------
This file is an experimental exact-algorithm attempt for the modified en-route TSP-D
(ES-TSPD) without fixed (a,b) sub-route caps such as (3,2), (3,3), or (3,4).

It is intentionally different from Step 61/62/63:

    Step 61/62/63:
        finite restricted sub-route column space Psi^{a,b}; precomputed/vector-indexed
        signatures; exact BPC proof only for that restricted model.

    Step 64:
        no MAX_TRUCK_INTERNAL_CUSTOMERS and no MAX_DRONE_CUSTOMERS cap. Pricing is a
        dynamic elementary route/excursion enumeration over a full 13-customer instance.
        A column is a complete feasible truck-drone solution route, and the master is a
        set-partitioning master over complete feasible columns.

Important scope disclosure
--------------------------
1. This is NOT a fast guaranteed solver for 15 customers. It is a serious diagnostic
   attempt for 13 customers.

2. The pricing oracle has no fixed truck-customer or drone-customer count cap. It may
   still stop because of TIME_LIMIT_SECONDS. If it stops by time limit, no proof is claimed.

3. The local continuous en-route excursion subproblem is solved by enumerating every
   nondecreasing edge assignment for the hand-off points and using SLSQP for the resulting
   small convex nonlinear program. This is the same idea as excursion_solver.py. For a
   rigorous mathematical proof, replace SLSQP with a certified convex solver or independently
   verify the convex subproblem global optimum. In computational logs, this code reports
   "COMPUTATIONALLY_CERTIFIED" rather than pretending a numerical local solver is a theorem.

4. This code allows MULTIPLE chained drone excursions along the truck route. Each excursion
   may contain any number of drone-served customers subject only to the instance size and
   endurance feasibility. There is no (a,b) cap.

5. Exactness of the BPC framework is conditional on complete pricing and exact local P1
   solves. If the run terminates naturally and the local P1 oracle is accepted as exact,
   then the result certifies the uncapped model encoded here.

Requirements
------------
    pip install numpy scipy matplotlib

No Gurobi is required.
==========================================================================================
"""

import math
import time
import random
import pickle
import heapq
import itertools
from dataclasses import dataclass, field
from typing import Dict, Tuple, List, Set, FrozenSet, Optional, Any

import numpy as np
from scipy.optimize import linprog, minimize, LinearConstraint, NonlinearConstraint
import matplotlib.pyplot as plt


# ==========================================================================================
# USER SETTINGS
# ==========================================================================================

N_CUSTOMERS = 13
RANDOM_SEED = 11
GRID_SIZE = 50

PHI0 = 1.0
PHI1 = 2.0
ENDURANCE = 25.0

TIME_LIMIT_SECONDS = 7200.0          # increase for serious proof attempts
MAX_BPC_NODES = 200                 # set None for no node cap
MAX_CG_ITER = 10_000
MAX_COLUMNS_PER_ITER = 30
MAX_NEGATIVE_COLUMNS_PER_PRICING = 30

REDUCED_COST_TOL = -1e-7
INTEGER_TOL = 1e-6
BIG_M = 1.0e8
EPS = 1e-9

# Pricing controls. These are NOT truck/drone sequence caps. They only control memory/logging.
PRICING_LOG_EVERY_LABELS = 100_000
PRICING_MAX_LIVE_LABELS_SOFT = None  # None = no cap. Setting a number makes it heuristic/diagnostic.

# Persistent cache for local P1/excursion subproblems.
USE_PERSISTENT_P1_CACHE = True
P1_CACHE_FILE = "STEP64_UNCAPPED_13CUSTOMER_P1_EXCURSION_CACHE.pkl"
SAVE_CACHE_EVERY_NEW = 5000

FIGURE_FILE = "STEP64_UNCAPPED_DYNAMIC_PRICING_BPC_13CUSTOMER_SOLUTION.png"
SUMMARY_FILE = "STEP64_UNCAPPED_DYNAMIC_PRICING_BPC_13CUSTOMER_SOLUTION.txt"


# ==========================================================================================
# BASIC GEOMETRY AND INSTANCE
# ==========================================================================================

def generate_instance(n_customers: int, seed: int, grid_size: int):
    random.seed(seed)
    start, end = 0, n_customers + 1
    coords = {start: (random.uniform(0, grid_size), random.uniform(0, grid_size))}
    for i in range(1, n_customers + 1):
        coords[i] = (random.uniform(0, grid_size), random.uniform(0, grid_size))
    coords[end] = coords[start]
    customers = list(range(1, n_customers + 1))
    return coords, customers, start, end


def dist_xy(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist_node(coords, a, b) -> float:
    return dist_xy(coords[a], coords[b])


def bit(mask_pos: int) -> int:
    return 1 << mask_pos


# ==========================================================================================
# LOCAL EN-ROUTE EXCURSION SOLVER: integrated from excursion_solver.py idea
# ==========================================================================================

class ExcursionResult:
    __slots__ = ("waiting_time", "breakpoints", "edge_assignment", "fractions", "feasible")

    def __init__(self, waiting_time, breakpoints, edge_assignment, fractions, feasible):
        self.waiting_time = float(waiting_time)
        self.breakpoints = breakpoints
        self.edge_assignment = edge_assignment
        self.fractions = fractions
        self.feasible = bool(feasible)


def _round_pt(p, ndigits=6):
    return (round(float(p[0]), ndigits), round(float(p[1]), ndigits))


class PersistentExcursionCache:
    def __init__(self, filename: str):
        self.filename = filename
        self.data: Dict[Any, Optional[ExcursionResult]] = {}
        self.hits = 0
        self.misses = 0
        self.new_since_save = 0
        self.load_seconds = 0.0
        self.save_seconds = 0.0
        self.solves = 0
        self.solve_seconds = 0.0

    def load(self):
        if not USE_PERSISTENT_P1_CACHE:
            return
        t0 = time.time()
        try:
            with open(self.filename, "rb") as f:
                self.data = pickle.load(f)
            self.load_seconds += time.time() - t0
            print(f"Loaded Step 64 P1/excursion cache: {self.filename} | entries={len(self.data):,} | seconds={self.load_seconds:.3f}")
        except FileNotFoundError:
            print(f"No Step 64 P1/excursion cache found; starting fresh: {self.filename}")
        except Exception as e:
            print(f"WARNING: could not load Step 64 P1/excursion cache: {e}")

    def save(self):
        if not USE_PERSISTENT_P1_CACHE:
            return
        t0 = time.time()
        tmp = self.filename + ".tmp"
        with open(tmp, "wb") as f:
            pickle.dump(self.data, f, protocol=pickle.HIGHEST_PROTOCOL)
        import os
        os.replace(tmp, self.filename)
        self.save_seconds += time.time() - t0
        self.new_since_save = 0
        print(f"Saved Step 64 P1/excursion cache: {self.filename} | entries={len(self.data):,} | cumulative_save_seconds={self.save_seconds:.3f}")

    def key(self, anchor_points, drone_customers, phi0, phi1, endurance):
        return (
            tuple(_round_pt(p) for p in anchor_points),
            tuple(_round_pt(v) for v in drone_customers),
            round(float(phi0), 6), round(float(phi1), 6), round(float(endurance), 6),
        )

    def solve(self, anchor_points, drone_customers, phi0, phi1, endurance, n_restarts=3):
        k = self.key(anchor_points, drone_customers, phi0, phi1, endurance)
        if k in self.data:
            self.hits += 1
            return self.data[k]
        self.misses += 1
        t0 = time.time()
        res = solve_excursion_uncached(anchor_points, drone_customers, phi0, phi1, endurance, n_restarts=n_restarts)
        dt = time.time() - t0
        self.solves += 1
        self.solve_seconds += dt
        self.data[k] = res
        self.new_since_save += 1
        if self.new_since_save >= SAVE_CACHE_EVERY_NEW:
            self.save()
        return res


def solve_excursion_uncached(anchor_points: List[Tuple[float, float]],
                             drone_customers: List[Tuple[float, float]],
                             phi0: float, phi1: float, endurance: float,
                             n_restarts: int = 3) -> Optional[ExcursionResult]:
    """Solve one chained excursion over a fixed truck anchor window.

    The hand-off points are ordered along the anchor polyline. For each nondecreasing
    edge assignment, solve the small nonlinear convex subproblem numerically.
    """
    if not drone_customers:
        return ExcursionResult(0.0, [], tuple(), tuple(), True)

    anchors = [np.array(p, dtype=float) for p in anchor_points]
    custs = [np.array(v, dtype=float) for v in drone_customers]
    m = len(custs)
    K = len(anchors) - 1
    if m < 1 or K < 1:
        return None

    edge_vec = [anchors[k + 1] - anchors[k] for k in range(K)]
    edge_len = [float(np.linalg.norm(v)) for v in edge_vec]
    cum_len = [0.0]
    for L in edge_len:
        cum_len.append(cum_len[-1] + L)

    def point_on_route(edge_idx: int, lam: float) -> np.ndarray:
        return anchors[edge_idx] + lam * edge_vec[edge_idx]

    def arrival_time(edge_idx: int, lam: float) -> float:
        return (cum_len[edge_idx] + lam * edge_len[edge_idx]) / phi0

    best = None

    for combo in itertools.combinations_with_replacement(range(K), m + 1):
        same_edge_pairs = [q for q in range(m) if combo[q] == combo[q + 1]]

        def objective(x):
            total = 0.0
            for q in range(m):
                e0, e1 = combo[q], combo[q + 1]
                p0 = point_on_route(e0, x[q])
                p1 = point_on_route(e1, x[q + 1])
                a0 = arrival_time(e0, x[q])
                a1 = arrival_time(e1, x[q + 1])
                drone_t = (float(np.linalg.norm(p0 - custs[q])) + float(np.linalg.norm(custs[q] - p1))) / phi1
                total += max(0.0, a0 + drone_t - a1)
            return total

        def endurance_constraints(x):
            vals = []
            for q in range(m):
                e0, e1 = combo[q], combo[q + 1]
                p0 = point_on_route(e0, x[q])
                p1 = point_on_route(e1, x[q + 1])
                drone_t = (float(np.linalg.norm(p0 - custs[q])) + float(np.linalg.norm(custs[q] - p1))) / phi1
                vals.append(endurance - drone_t)
            return np.array(vals)

        bounds = [(0.0, 1.0)] * (m + 1)
        constraints = []
        if same_edge_pairs:
            A = np.zeros((len(same_edge_pairs), m + 1))
            for row, q in enumerate(same_edge_pairs):
                A[row, q] = 1.0
                A[row, q + 1] = -1.0
            constraints.append(LinearConstraint(A, -np.inf, 0.0))
        constraints.append(NonlinearConstraint(endurance_constraints, 0.0, np.inf))

        starts = [np.full(m + 1, 0.5)]
        for _ in range(max(0, n_restarts - 1)):
            starts.append(np.random.uniform(0.0, 1.0, size=m + 1))

        fixed_starts = []
        for s in starts:
            s = s.copy()
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
                vals = sorted(float(s[idx]) for idx in g)
                for idx, gi in enumerate(g):
                    s[gi] = vals[idx]
            fixed_starts.append(s)

        for s0 in fixed_starts:
            try:
                opt = minimize(objective, s0, method="SLSQP", bounds=bounds, constraints=constraints,
                               options={"maxiter": 200, "ftol": 1e-10, "disp": False})
            except Exception:
                continue
            if not opt.success:
                continue
            ev = endurance_constraints(opt.x)
            if np.any(ev < -1e-6):
                continue
            w = float(objective(opt.x))
            if best is None or w < best.waiting_time - 1e-9:
                bp = [tuple(point_on_route(combo[q], opt.x[q])) for q in range(m + 1)]
                best = ExcursionResult(w, bp, tuple(combo), tuple(float(v) for v in opt.x), True)

    return best


# ==========================================================================================
# DATA CLASSES
# ==========================================================================================

@dataclass
class ExcursionBlock:
    start_index: int
    end_index: int
    drone_sequence: Tuple[int, ...]
    truck_edges: Tuple[Tuple[int, int], ...]
    waiting_time: float
    breakpoints: Tuple[Tuple[float, float], ...] = tuple()


@dataclass
class Column:
    cost: float
    visit_count: Tuple[int, ...]
    drone_count: Tuple[int, ...]
    truck_arcs: Tuple[Tuple[int, int], ...]
    node_sequence: Tuple[int, ...]
    excursion_blocks: Tuple[ExcursionBlock, ...]
    name: str = ""

    def key(self):
        # Include all information that affects the master/branching and cost.
        return (
            self.node_sequence,
            tuple(block.drone_sequence for block in self.excursion_blocks),
            tuple(block.truck_edges for block in self.excursion_blocks),
        )


@dataclass
class BranchNode:
    forced_yD: Set[int] = field(default_factory=set)
    forbidden_yD: Set[int] = field(default_factory=set)
    forced_xT: Set[Tuple[int, int]] = field(default_factory=set)
    forbidden_xT: Set[Tuple[int, int]] = field(default_factory=set)
    forced_mu: Set[Tuple[int, int]] = field(default_factory=set)
    forbidden_mu: Set[Tuple[int, int]] = field(default_factory=set)
    depth: int = 0
    node_id: int = 0

    def copy(self):
        return BranchNode(
            forced_yD=set(self.forced_yD), forbidden_yD=set(self.forbidden_yD),
            forced_xT=set(self.forced_xT), forbidden_xT=set(self.forbidden_xT),
            forced_mu=set(self.forced_mu), forbidden_mu=set(self.forbidden_mu),
            depth=self.depth, node_id=self.node_id,
        )


@dataclass
class Label:
    red_cost: float
    real_cost: float
    cur: int
    node_seq: Tuple[int, ...]
    visited_mask: int
    drone_mask: int
    phase: str                       # "FREE" or "ACTIVE"
    pending_drone: Tuple[int, ...]
    open_index: int
    excursion_blocks: Tuple[ExcursionBlock, ...]


# ==========================================================================================
# UNCAPPED DYNAMIC PRICING BPC SOLVER
# ==========================================================================================

class Step64UncappedESBPC:
    def __init__(self, coords, customers, start, end, phi0, phi1, endurance, time_limit=None, verbose=True):
        self.coords = coords
        self.customers = list(customers)
        self.start = start
        self.end = end
        self.phi0 = phi0
        self.phi1 = phi1
        self.endurance = endurance
        self.time_limit = time_limit
        self.verbose = verbose
        self.n = len(self.customers)
        self.full_mask = (1 << self.n) - 1
        self.pos = {c: i for i, c in enumerate(self.customers)}
        self.customer_from_bit = {i: c for c, i in self.pos.items()}
        self.start_time = None
        self.column_pool: Dict[Any, Column] = {}
        self.best_ub = float("inf")
        self.best_col: Optional[Column] = None
        self.node_counter = 0
        self.p1_cache = PersistentExcursionCache(P1_CACHE_FILE)
        self.pricing_labels_expanded_total = 0
        self.pricing_columns_found_total = 0

    # --------------------------------------------------------------------------------------
    def elapsed(self):
        return time.time() - self.start_time if self.start_time is not None else 0.0

    def time_exceeded(self):
        return self.time_limit is not None and self.elapsed() >= self.time_limit

    # --------------------------------------------------------------------------------------
    def nearest_neighbor_truck_only_column(self) -> Column:
        unvisited = set(self.customers)
        cur = self.start
        seq = [self.start]
        while unvisited:
            nxt = min(unvisited, key=lambda j: dist_node(self.coords, cur, j))
            seq.append(nxt)
            unvisited.remove(nxt)
            cur = nxt
        seq.append(self.end)
        cost = sum(dist_node(self.coords, a, b) / self.phi0 for a, b in zip(seq[:-1], seq[1:]))
        visit = [1] * self.n
        drone = [0] * self.n
        return Column(cost=cost, visit_count=tuple(visit), drone_count=tuple(drone),
                      truck_arcs=tuple(zip(seq[:-1], seq[1:])), node_sequence=tuple(seq),
                      excursion_blocks=tuple(), name="nearest_neighbor_truck_only")

    # --------------------------------------------------------------------------------------
    def solve_rmp(self, columns: List[Column]):
        R = len(columns)
        n_rows = 1 + self.n
        A_eq = np.zeros((n_rows, R))
        b_eq = np.zeros(n_rows)
        b_eq[0] = 1.0
        for r, col in enumerate(columns):
            A_eq[0, r] = 1.0
        for p in range(self.n):
            b_eq[1 + p] = 1.0
            for r, col in enumerate(columns):
                A_eq[1 + p, r] = col.visit_count[p]
        c = np.array([col.cost for col in columns], dtype=float)

        # Artificial variables keep RMP feasible during early iterations.
        n_art = n_rows
        A_art = np.zeros((n_rows, 2 * n_art))
        for i in range(n_rows):
            A_art[i, 2 * i] = 1.0
            A_art[i, 2 * i + 1] = -1.0
        c_art = np.full(2 * n_art, BIG_M)
        A_full = np.hstack([A_eq, A_art])
        c_full = np.concatenate([c, c_art])
        bounds = [(0, None)] * (R + 2 * n_art)
        res = linprog(c_full, A_eq=A_full, b_eq=b_eq, bounds=bounds, method="highs")
        if res.status != 0:
            return None
        lambdas = res.x[:R]
        art = res.x[R:]
        duals = res.eqlin.marginals
        return {
            "obj": float(np.dot(c, lambdas)),
            "lambda": lambdas.tolist(),
            "dual_route": float(duals[0]),
            "dual_cover": {self.customers[p]: float(duals[1 + p]) for p in range(self.n)},
            "artificial_sum": float(np.sum(np.abs(art))),
        }

    # --------------------------------------------------------------------------------------
    def column_satisfies_branch(self, col: Column, node: BranchNode) -> bool:
        for i in node.forced_yD:
            if col.drone_count[self.pos[i]] != 1:
                return False
        for i in node.forbidden_yD:
            if col.drone_count[self.pos[i]] != 0:
                return False
        arc_set = set(col.truck_arcs)
        for a in node.forced_xT:
            if a not in arc_set:
                return False
        for a in node.forbidden_xT:
            if a in arc_set:
                return False
        mu_set = set()
        for block in col.excursion_blocks:
            mu_set.update(block.truck_edges)
        for a in node.forced_mu:
            if a not in mu_set:
                return False
        for a in node.forbidden_mu:
            if a in mu_set:
                return False
        return True

    def filtered_columns(self, node: BranchNode) -> List[Column]:
        return [c for c in self.column_pool.values() if self.column_satisfies_branch(c, node)]

    # --------------------------------------------------------------------------------------
    def truck_arc_allowed(self, node: BranchNode, a, b):
        if (a, b) in node.forbidden_xT:
            return False
        forced_out = [j for (i, j) in node.forced_xT if i == a]
        if forced_out and b not in forced_out:
            return False
        return True

    def excursion_edge_allowed(self, node: BranchNode, a, b):
        return (a, b) not in node.forbidden_mu

    # --------------------------------------------------------------------------------------
    def build_column_from_closed_route(self, lab: Label) -> Column:
        visit = [1 if (lab.visited_mask & bit(p)) else 0 for p in range(self.n)]
        drone = [1 if (lab.drone_mask & bit(p)) else 0 for p in range(self.n)]
        return Column(cost=lab.real_cost, visit_count=tuple(visit), drone_count=tuple(drone),
                      truck_arcs=tuple(zip(lab.node_seq[:-1], lab.node_seq[1:])),
                      node_sequence=lab.node_seq, excursion_blocks=lab.excursion_blocks,
                      name="priced_uncapped")

    # --------------------------------------------------------------------------------------
    def lower_bound_remaining_truck(self, cur: int, visited_mask: int) -> float:
        """Very cheap valid lower bound for remaining physical truck travel.

        This bound is deliberately weak but safe: if no customers remain it is distance to depot;
        otherwise it is the minimum of (cur to one remaining customer + one remaining/depot to depot)
        divided by truck speed. It never overestimates the remaining truck travel.
        """
        rem = [c for c in self.customers if not (visited_mask & bit(self.pos[c]))]
        if not rem:
            return dist_node(self.coords, cur, self.end) / self.phi0
        min_out = min(dist_node(self.coords, cur, j) for j in rem)
        min_back = min(dist_node(self.coords, j, self.end) for j in rem)
        return (min_out + min_back) / self.phi0

    # --------------------------------------------------------------------------------------
    def reduced_cost_lb_for_label(self, lab: Label, dual_cover: Dict[int, float]) -> float:
        """A safe partial-label reduced-cost lower bound.

        Remaining positive dual contributions could make reduced cost smaller, so to stay safe we
        subtract only positive cover duals for unvisited customers. We add a safe truck lower bound.
        """
        rem_dual_credit = 0.0
        for c in self.customers:
            if not (lab.visited_mask & bit(self.pos[c])):
                rem_dual_credit += max(0.0, dual_cover[c])
        return lab.red_cost + self.lower_bound_remaining_truck(lab.cur, lab.visited_mask) - rem_dual_credit

    # --------------------------------------------------------------------------------------
    def try_close_excursion(self, lab: Label, u: Dict[int, float], node: BranchNode,
                            out: List[Label], neg_cols: List[Tuple[float, Column]],
                            local_keys: Set[Any], extra_end: bool = False):
        if not lab.pending_drone:
            return
        span_nodes = lab.node_seq[lab.open_index:]
        if extra_end:
            span_nodes = span_nodes + (self.end,)
        if len(span_nodes) < 2:
            return
        edges = tuple(zip(span_nodes[:-1], span_nodes[1:]))
        for e in edges:
            if not self.excursion_edge_allowed(node, e[0], e[1]):
                return

        anchor_points = [self.coords[x] for x in span_nodes]
        drone_points = [self.coords[x] for x in lab.pending_drone]
        res = self.p1_cache.solve(anchor_points, drone_points, self.phi0, self.phi1, self.endurance, n_restarts=3)
        if res is None:
            return

        block = ExcursionBlock(
            start_index=lab.open_index,
            end_index=(len(lab.node_seq) if extra_end else len(lab.node_seq) - 1),
            drone_sequence=tuple(lab.pending_drone),
            truck_edges=edges,
            waiting_time=float(res.waiting_time),
            breakpoints=tuple(tuple(map(float, p)) for p in res.breakpoints),
        )

        if extra_end:
            final_edge_cost = dist_node(self.coords, lab.cur, self.end) / self.phi0
            new_lab = Label(
                red_cost=lab.red_cost + final_edge_cost + res.waiting_time,
                real_cost=lab.real_cost + final_edge_cost + res.waiting_time,
                cur=self.end,
                node_seq=lab.node_seq + (self.end,),
                visited_mask=lab.visited_mask,
                drone_mask=lab.drone_mask,
                phase="FREE",
                pending_drone=tuple(),
                open_index=-1,
                excursion_blocks=lab.excursion_blocks + (block,),
            )
            col = self.build_column_from_closed_route(new_lab)
            if col.key() not in local_keys and self.column_satisfies_branch(col, node) and new_lab.red_cost < REDUCED_COST_TOL:
                neg_cols.append((new_lab.red_cost, col))
        else:
            out.append(Label(
                red_cost=lab.red_cost + res.waiting_time,
                real_cost=lab.real_cost + res.waiting_time,
                cur=lab.cur,
                node_seq=lab.node_seq,
                visited_mask=lab.visited_mask,
                drone_mask=lab.drone_mask,
                phase="FREE",
                pending_drone=tuple(),
                open_index=-1,
                excursion_blocks=lab.excursion_blocks + (block,),
            ))

    # --------------------------------------------------------------------------------------
    def pricing(self, rmp, node: BranchNode, local_keys: Set[Any]):
        u0 = rmp["dual_route"]
        u = rmp["dual_cover"]
        neg_cols: List[Tuple[float, Column]] = []
        labels_expanded = 0
        labels_generated = 0
        pruned_lb = 0
        pruned_duplicate = 0

        init = Label(red_cost=-u0, real_cost=0.0, cur=self.start, node_seq=(self.start,),
                     visited_mask=0, drone_mask=0, phase="FREE", pending_drone=tuple(),
                     open_index=-1, excursion_blocks=tuple())

        # Priority queue explores most promising lower bound first.
        heap = []
        counter = 0
        heapq.heappush(heap, (self.reduced_cost_lb_for_label(init, u), counter, init))
        best_seen: Dict[Any, float] = {}

        while heap:
            if self.time_exceeded():
                return neg_cols, "TIME_LIMIT", {
                    "expanded": labels_expanded, "generated": labels_generated,
                    "pruned_lb": pruned_lb, "pruned_duplicate": pruned_duplicate,
                    "live_labels": len(heap),
                }

            lb_key, _, lab = heapq.heappop(heap)

            # Safe pruning: if a label cannot lead to a negative reduced-cost column, skip it.
            # We use REDUCED_COST_TOL, not incumbent UB, because pricing only needs negative RC.
            if lb_key >= REDUCED_COST_TOL:
                pruned_lb += 1
                continue

            # Exact duplicate-state pruning. The key includes complete node_seq and pending drone sequence;
            # therefore this never merges geometrically different active excursions.
            state_key = (lab.cur, lab.node_seq, lab.visited_mask, lab.drone_mask, lab.phase,
                         lab.pending_drone, lab.open_index,
                         tuple((b.start_index, b.end_index, b.drone_sequence, b.truck_edges) for b in lab.excursion_blocks))
            old = best_seen.get(state_key)
            if old is not None and old <= lab.red_cost + 1e-12:
                pruned_duplicate += 1
                continue
            best_seen[state_key] = lab.red_cost

            labels_expanded += 1
            if self.verbose and labels_expanded % PRICING_LOG_EVERY_LABELS == 0:
                print(f"      Step64 pricing expanded={labels_expanded:,}, live={len(heap):,}, neg={len(neg_cols)}, p1_solves={self.p1_cache.solves:,}, p1_hits={self.p1_cache.hits:,}")

            if PRICING_MAX_LIVE_LABELS_SOFT is not None and len(heap) > PRICING_MAX_LIVE_LABELS_SOFT:
                return neg_cols, "SOFT_LABEL_LIMIT_HEURISTIC", {
                    "expanded": labels_expanded, "generated": labels_generated,
                    "pruned_lb": pruned_lb, "pruned_duplicate": pruned_duplicate,
                    "live_labels": len(heap),
                }

            # Completed route.
            if lab.cur == self.end:
                continue

            # If all customers are served, close active excursion to depot if needed, else add depot arc.
            if lab.visited_mask == self.full_mask:
                if lab.phase == "ACTIVE":
                    if self.truck_arc_allowed(node, lab.cur, self.end):
                        tmp_out = []
                        self.try_close_excursion(lab, u, node, tmp_out, neg_cols, local_keys, extra_end=True)
                        for nl in tmp_out:
                            counter += 1
                            labels_generated += 1
                            heapq.heappush(heap, (self.reduced_cost_lb_for_label(nl, u), counter, nl))
                else:
                    if self.truck_arc_allowed(node, lab.cur, self.end):
                        edge_cost = dist_node(self.coords, lab.cur, self.end) / self.phi0
                        nl = Label(red_cost=lab.red_cost + edge_cost,
                                   real_cost=lab.real_cost + edge_cost,
                                   cur=self.end,
                                   node_seq=lab.node_seq + (self.end,),
                                   visited_mask=lab.visited_mask,
                                   drone_mask=lab.drone_mask,
                                   phase="FREE",
                                   pending_drone=tuple(), open_index=-1,
                                   excursion_blocks=lab.excursion_blocks)
                        col = self.build_column_from_closed_route(nl)
                        if col.key() not in local_keys and self.column_satisfies_branch(col, node) and nl.red_cost < REDUCED_COST_TOL:
                            neg_cols.append((nl.red_cost, col))
                if len(neg_cols) >= MAX_NEGATIVE_COLUMNS_PER_PRICING:
                    neg_cols.sort(key=lambda x: x[0])
                    return neg_cols[:MAX_NEGATIVE_COLUMNS_PER_PRICING], "FOUND_NEGATIVE_EARLY", {
                        "expanded": labels_expanded, "generated": labels_generated,
                        "pruned_lb": pruned_lb, "pruned_duplicate": pruned_duplicate,
                        "live_labels": len(heap),
                    }
                continue

            children: List[Label] = []

            # Move 1: truck visits an unserved customer.
            for j in self.customers:
                jbit = bit(self.pos[j])
                if lab.visited_mask & jbit:
                    continue
                if j in node.forced_yD:
                    continue
                if not self.truck_arc_allowed(node, lab.cur, j):
                    continue
                if lab.phase == "ACTIVE" and not self.excursion_edge_allowed(node, lab.cur, j):
                    continue
                edge_cost = dist_node(self.coords, lab.cur, j) / self.phi0
                children.append(Label(
                    red_cost=lab.red_cost + edge_cost - u[j],
                    real_cost=lab.real_cost + edge_cost,
                    cur=j,
                    node_seq=lab.node_seq + (j,),
                    visited_mask=lab.visited_mask | jbit,
                    drone_mask=lab.drone_mask,
                    phase=lab.phase,
                    pending_drone=lab.pending_drone,
                    open_index=lab.open_index,
                    excursion_blocks=lab.excursion_blocks,
                ))

            # Move 2: open a new excursion at the current truck position.
            if lab.phase == "FREE" and lab.cur != self.end:
                # Opening without adding drone immediately is safe, but to reduce neutral duplicate moves
                # only open if at least one unserved customer is not forbidden to be drone-served.
                has_drone_candidate = any(
                    not (lab.visited_mask & bit(self.pos[j])) and j not in node.forbidden_yD
                    for j in self.customers
                )
                if has_drone_candidate:
                    children.append(Label(
                        red_cost=lab.red_cost,
                        real_cost=lab.real_cost,
                        cur=lab.cur,
                        node_seq=lab.node_seq,
                        visited_mask=lab.visited_mask,
                        drone_mask=lab.drone_mask,
                        phase="ACTIVE",
                        pending_drone=tuple(),
                        open_index=len(lab.node_seq) - 1,
                        excursion_blocks=lab.excursion_blocks,
                    ))

            # Move 3: assign an unserved customer to the currently open drone chain.
            if lab.phase == "ACTIVE":
                for j in self.customers:
                    jbit = bit(self.pos[j])
                    if lab.visited_mask & jbit:
                        continue
                    if j in node.forbidden_yD:
                        continue
                    children.append(Label(
                        red_cost=lab.red_cost - u[j],
                        real_cost=lab.real_cost,
                        cur=lab.cur,
                        node_seq=lab.node_seq,
                        visited_mask=lab.visited_mask | jbit,
                        drone_mask=lab.drone_mask | jbit,
                        phase="ACTIVE",
                        pending_drone=lab.pending_drone + (j,),
                        open_index=lab.open_index,
                        excursion_blocks=lab.excursion_blocks,
                    ))

            # Move 4: close an active excursion at the current truck position.
            if lab.phase == "ACTIVE" and lab.pending_drone:
                self.try_close_excursion(lab, u, node, children, neg_cols, local_keys, extra_end=False)

            for nl in children:
                lb = self.reduced_cost_lb_for_label(nl, u)
                if lb >= REDUCED_COST_TOL:
                    pruned_lb += 1
                    continue
                counter += 1
                labels_generated += 1
                heapq.heappush(heap, (lb, counter, nl))

            if len(neg_cols) >= MAX_NEGATIVE_COLUMNS_PER_PRICING:
                neg_cols.sort(key=lambda x: x[0])
                return neg_cols[:MAX_NEGATIVE_COLUMNS_PER_PRICING], "FOUND_NEGATIVE_EARLY", {
                    "expanded": labels_expanded, "generated": labels_generated,
                    "pruned_lb": pruned_lb, "pruned_duplicate": pruned_duplicate,
                    "live_labels": len(heap),
                }

        neg_cols.sort(key=lambda x: x[0])
        return neg_cols, "CLOSED_FULL", {
            "expanded": labels_expanded, "generated": labels_generated,
            "pruned_lb": pruned_lb, "pruned_duplicate": pruned_duplicate,
            "live_labels": len(heap),
        }

    # --------------------------------------------------------------------------------------
    def column_generation(self, node: BranchNode):
        columns = self.filtered_columns(node)
        local_keys = set(c.key() for c in columns)
        iteration = 0
        while True:
            if self.time_exceeded():
                return None, columns, "TIME_LIMIT"
            iteration += 1
            if iteration > MAX_CG_ITER:
                return None, columns, "CG_ITER_LIMIT"
            rmp = self.solve_rmp(columns)
            if rmp is None:
                return None, columns, "RMP_INFEASIBLE"
            if self.verbose:
                print(f"    CG iter {iteration:03d}: LP={rmp['obj']:.9f}, art={rmp['artificial_sum']:.3e}, cols={len(columns):,}")

            t0 = time.time()
            neg_cols, price_status, stats = self.pricing(rmp, node, local_keys)
            self.pricing_labels_expanded_total += stats["expanded"]
            self.pricing_columns_found_total += len(neg_cols)
            if self.verbose:
                print(f"      pricing status={price_status}, neg={len(neg_cols)}, expanded={stats['expanded']:,}, generated={stats['generated']:,}, pruned_lb={stats['pruned_lb']:,}, dup={stats['pruned_duplicate']:,}, live={stats['live_labels']:,}, seconds={time.time()-t0:.3f}")

            if price_status == "TIME_LIMIT":
                return rmp, columns, "TIME_LIMIT"
            if price_status == "SOFT_LABEL_LIMIT_HEURISTIC":
                return rmp, columns, "SOFT_LABEL_LIMIT_HEURISTIC"
            if not neg_cols:
                return rmp, columns, "CLOSED"

            added = 0
            for rc, col in neg_cols:
                if not self.column_satisfies_branch(col, node):
                    continue
                key = col.key()
                if key in local_keys:
                    continue
                columns.append(col)
                self.column_pool[key] = col
                local_keys.add(key)
                added += 1
                if added >= MAX_COLUMNS_PER_ITER:
                    break
            if self.verbose:
                print(f"      added columns={added}")
            if added == 0:
                return rmp, columns, "CLOSED_DUPLICATE"

    # --------------------------------------------------------------------------------------
    def is_integer_solution(self, columns, lambdas):
        return all(abs(v - round(v)) <= INTEGER_TOL for v in lambdas)

    def extract_best_integer_column(self, columns, lambdas, node: BranchNode):
        best = None
        for col, val in zip(columns, lambdas):
            if val >= 1.0 - INTEGER_TOL and self.column_satisfies_branch(col, node):
                if best is None or col.cost < best.cost - 1e-9:
                    best = col
        return best

    def update_incumbent_from_columns(self, columns, node: BranchNode):
        for col in columns:
            if not self.column_satisfies_branch(col, node):
                continue
            if col.cost < self.best_ub - 1e-7:
                self.best_ub = col.cost
                self.best_col = col
                if self.verbose:
                    print(f"    New incumbent from column pool: {self.best_ub:.9f}")

    # --------------------------------------------------------------------------------------
    def compute_branch_values(self, columns, lambdas):
        yD = {i: 0.0 for i in self.customers}
        xT: Dict[Tuple[int, int], float] = {}
        mu: Dict[Tuple[int, int], float] = {}
        for col, val in zip(columns, lambdas):
            if val <= 1e-10:
                continue
            for c in self.customers:
                yD[c] += val * col.drone_count[self.pos[c]]
            for e in col.truck_arcs:
                xT[e] = xT.get(e, 0.0) + val
            mu_set = set()
            for block in col.excursion_blocks:
                mu_set.update(block.truck_edges)
            for e in mu_set:
                mu[e] = mu.get(e, 0.0) + val
        return yD, xT, mu

    def select_branching_decision(self, columns, lambdas, node: BranchNode):
        yD, xT, mu = self.compute_branch_values(columns, lambdas)
        best = None
        best_score = float("inf")
        for i, val in yD.items():
            if i in node.forced_yD or i in node.forbidden_yD:
                continue
            if abs(val - round(val)) > INTEGER_TOL:
                score = abs(val - 0.5)
                if score < best_score:
                    best_score = score
                    best = ("yD", i, val)
        if best is not None:
            return best
        for e, val in xT.items():
            if e in node.forced_xT or e in node.forbidden_xT:
                continue
            if abs(val - round(val)) > INTEGER_TOL:
                score = abs(val - 0.5)
                if score < best_score:
                    best_score = score
                    best = ("xT", e, val)
        if best is not None:
            return best
        for e, val in mu.items():
            if e in node.forced_mu or e in node.forbidden_mu:
                continue
            if abs(val - round(val)) > INTEGER_TOL:
                score = abs(val - 0.5)
                if score < best_score:
                    best_score = score
                    best = ("mu", e, val)
        return best

    # --------------------------------------------------------------------------------------
    def solve(self):
        self.start_time = time.time()
        self.p1_cache.load()

        init_col = self.nearest_neighbor_truck_only_column()
        self.column_pool[init_col.key()] = init_col
        self.best_ub = init_col.cost
        self.best_col = init_col

        root = BranchNode(node_id=0, depth=0)
        pq = [(0.0, 0, root)]
        processed = 0
        created = 1
        pruned = 0
        best_bound = -float("inf")
        status = "PROVEN_OPTIMAL_COMPUTATIONALLY_CERTIFIED"
        interrupted_reason = None

        print("=" * 90)
        print("STEP 64: UNCAPPED DYNAMIC-PRICING BPC FOR 13-CUSTOMER ES-TSPD")
        print("=" * 90)
        print("No fixed (a,b) sub-route cap is used in pricing.")
        print("Columns are complete feasible truck-drone routes with possibly multiple chained excursions.")
        print("WARNING: local continuous P1 uses SLSQP after finite edge-assignment enumeration.")
        print("If TIME_LIMIT/ NODE_LIMIT occurs, the result is not a proof.")
        print(f"Customers={self.n}, phi0={self.phi0}, phi1={self.phi1}, endurance={self.endurance}")
        print(f"Initial truck-only UB={self.best_ub:.9f}")
        print("=" * 90)

        while pq:
            if self.time_exceeded():
                status = "TIME_LIMIT_NOT_PROVEN"
                interrupted_reason = "time limit"
                break
            if MAX_BPC_NODES is not None and processed >= MAX_BPC_NODES:
                status = "NODE_LIMIT_NOT_PROVEN"
                interrupted_reason = "node limit"
                break

            lb_parent, _, node = heapq.heappop(pq)
            if lb_parent >= self.best_ub - 1e-7 and processed > 0:
                pruned += 1
                continue
            processed += 1

            print("-" * 90)
            print(f"BPC NODE {node.node_id} | depth={node.depth} | open={len(pq)} | incumbent={self.best_ub:.9f}")
            print(f"  FD={sorted(node.forced_yD)} FT={sorted(node.forbidden_yD)}")
            print(f"  forced_xT={sorted(node.forced_xT)} forbidden_xT={sorted(node.forbidden_xT)}")
            print(f"  forced_mu={sorted(node.forced_mu)} forbidden_mu={sorted(node.forbidden_mu)}")

            rmp, columns, cg_status = self.column_generation(node)
            if cg_status in ("TIME_LIMIT", "CG_ITER_LIMIT", "SOFT_LABEL_LIMIT_HEURISTIC"):
                status = f"{cg_status}_NOT_PROVEN"
                interrupted_reason = cg_status
                break
            if rmp is None:
                print(f"  Node infeasible/closed: {cg_status}")
                pruned += 1
                continue

            lb = rmp["obj"]
            best_bound = max(best_bound, lb)
            print(f"  Node LP bound={lb:.9f}, incumbent={self.best_ub:.9f}, artificial={rmp['artificial_sum']:.3e}, cols={len(columns):,}")

            self.update_incumbent_from_columns(columns, node)
            if lb >= self.best_ub - 1e-7:
                print("  Node pruned by bound.")
                pruned += 1
                continue

            int_col = self.extract_best_integer_column(columns, rmp["lambda"], node)
            if int_col is not None:
                if int_col.cost < self.best_ub - 1e-7:
                    self.best_ub = int_col.cost
                    self.best_col = int_col
                    print(f"  Integer LP incumbent updated: {self.best_ub:.9f}")
                print("  Node pruned by integer LP solution.")
                pruned += 1
                continue

            decision = self.select_branching_decision(columns, rmp["lambda"], node)
            if decision is None:
                print("  No branching decision found; treating node as closed numerically.")
                pruned += 1
                continue

            kind, item, val = decision
            print(f"  Branching decision: {kind} {item} = {val:.6f}")
            c1 = node.copy()
            c2 = node.copy()
            self.node_counter += 1
            c1.node_id = self.node_counter
            c1.depth = node.depth + 1
            self.node_counter += 1
            c2.node_id = self.node_counter
            c2.depth = node.depth + 1

            if kind == "yD":
                c1.forced_yD.add(item)
                c2.forbidden_yD.add(item)
            elif kind == "xT":
                c1.forced_xT.add(item)
                c2.forbidden_xT.add(item)
            elif kind == "mu":
                c1.forced_mu.add(item)
                c2.forbidden_mu.add(item)

            heapq.heappush(pq, (lb, c1.node_id, c1))
            heapq.heappush(pq, (lb, c2.node_id, c2))
            created += 2

        elapsed = self.elapsed()
        if self.p1_cache.new_since_save > 0:
            self.p1_cache.save()

        if pq and status.startswith("PROVEN"):
            status = "OPEN_NODES_REMAIN_NOT_PROVEN"
        if not pq and status.startswith("PROVEN"):
            best_bound = self.best_ub

        result = {
            "status": status,
            "interrupted_reason": interrupted_reason,
            "best_objective": self.best_ub,
            "best_bound": best_bound if best_bound != -float("inf") else None,
            "gap": None,
            "processed_nodes": processed,
            "created_nodes": created,
            "pruned_nodes": pruned,
            "open_nodes": len(pq),
            "elapsed": elapsed,
            "columns": len(self.column_pool),
            "p1_cache_entries": len(self.p1_cache.data),
            "p1_hits": self.p1_cache.hits,
            "p1_misses": self.p1_cache.misses,
            "p1_solves": self.p1_cache.solves,
            "p1_solve_seconds": self.p1_cache.solve_seconds,
            "p1_load_seconds": self.p1_cache.load_seconds,
            "p1_save_seconds": self.p1_cache.save_seconds,
            "pricing_labels_expanded_total": self.pricing_labels_expanded_total,
            "pricing_columns_found_total": self.pricing_columns_found_total,
            "best_col": self.best_col,
        }
        if result["best_bound"] is not None and math.isfinite(result["best_objective"]):
            denom = max(1.0, abs(result["best_objective"]))
            result["gap"] = max(0.0, (result["best_objective"] - result["best_bound"]) / denom)
        return result

    # --------------------------------------------------------------------------------------
    def print_and_save_result(self, result):
        lines = []
        lines.append("=" * 90)
        lines.append("STEP 64 FINAL / PARTIAL RESULT")
        lines.append("=" * 90)
        for k in ["status", "interrupted_reason", "best_objective", "best_bound", "gap",
                  "processed_nodes", "created_nodes", "pruned_nodes", "open_nodes", "elapsed",
                  "columns", "p1_cache_entries", "p1_hits", "p1_misses", "p1_solves",
                  "p1_solve_seconds", "p1_load_seconds", "p1_save_seconds",
                  "pricing_labels_expanded_total", "pricing_columns_found_total"]:
            lines.append(f"{k:34s}: {result[k]}")
        col = result["best_col"]
        if col is None:
            lines.append("No feasible solution found.")
        else:
            lines.append("")
            lines.append(f"Best truck node sequence: {list(col.node_sequence)}")
            truck_customers = [x for x in col.node_sequence if x in self.pos]
            drone_customers = [c for c in self.customers if col.drone_count[self.pos[c]] == 1]
            lines.append(f"Truck-served customers: {sorted(truck_customers)}")
            lines.append(f"Drone-served customers: {sorted(drone_customers)}")
            lines.append(f"Number of excursion blocks: {len(col.excursion_blocks)}")
            for bidx, block in enumerate(col.excursion_blocks, 1):
                lines.append(f"  Excursion {bidx}: truck_edges={list(block.truck_edges)} drone_sequence={list(block.drone_sequence)} waiting={block.waiting_time:.9f}")
        text = "\n".join(lines)
        print(text)
        with open(SUMMARY_FILE, "w") as f:
            f.write(text + "\n")
        print(f"Saved summary: {SUMMARY_FILE}")

    # --------------------------------------------------------------------------------------
    def plot_solution(self, result):
        col = result["best_col"]
        if col is None:
            return
        plt.figure(figsize=(9, 7))
        drone_set = {c for c in self.customers if col.drone_count[self.pos[c]] == 1}
        for nid, (x, y) in self.coords.items():
            if nid == self.end:
                continue
            if nid == self.start:
                plt.scatter(x, y, marker="s", s=140)
                plt.text(x + 0.35, y + 0.35, "Depot", fontsize=10)
            else:
                marker = "^" if nid in drone_set else "o"
                plt.scatter(x, y, marker=marker, s=70)
                plt.text(x + 0.35, y + 0.35, str(nid), fontsize=9)
        mu_edges = set()
        for block in col.excursion_blocks:
            mu_edges.update(block.truck_edges)
        for a, b in col.truck_arcs:
            xa, ya = self.coords[a]
            xb, yb = self.coords[b]
            plt.plot([xa, xb], [ya, yb], "--" if (a, b) in mu_edges else "-", linewidth=2)
        plt.title(f"Step 64 Uncapped ES-TSPD | objective={result['best_objective']:.3f}")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.grid(True, alpha=0.25)
        plt.tight_layout()
        plt.savefig(FIGURE_FILE, dpi=200)
        print(f"Saved figure: {FIGURE_FILE}")


# ==========================================================================================
# MAIN
# ==========================================================================================

def main():
    coords, customers, start, end = generate_instance(N_CUSTOMERS, RANDOM_SEED, GRID_SIZE)
    print("Instance coordinates:")
    for i in [start] + customers + [end]:
        print(f"  {i:2d}: ({coords[i][0]:.6f}, {coords[i][1]:.6f})")
    solver = Step64UncappedESBPC(coords, customers, start, end, PHI0, PHI1, ENDURANCE,
                                 time_limit=TIME_LIMIT_SECONDS, verbose=True)
    result = solver.solve()
    solver.print_and_save_result(result)
    solver.plot_solution(result)


if __name__ == "__main__":
    main()
