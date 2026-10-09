# coresense_vampire

Wrapper around the Vampire ATP in terms of a ROS 2 node

`coresense_vampire` makes the [Vampire](https://github.com/vprover/vampire) automated theorem prover
available to ROS 2 systems. It is the Higher-Order Reasoning (HOR) cognitive module of the
[CoreSense](https://coresense.eu) project, described in CoreSense Deliverable D1.13
(*Automated theorem proving final version*, CS-151).

The node

- keeps **reasoning sessions**: knowledge written in the [TPTP](https://tptp.org) language is added
  to a session in named **formula sets**, which can later be removed again as a whole;
- answers **queries** through a cancellable ROS 2 action, reporting the prover's output (proofs, answer
  bindings) together with a detailed **status code**;
- lets parts of the knowledge live outside the prover: predicates declared **external** are answered
  on demand by other ROS 2 nodes, through a simple service interface.

## Installation

Build the package in a colcon workspace together with the CoreSense interface definitions
(`coresense_msgs`, part of [coresense_common](https://github.com/CoreSenseEU/coresense_common)):

```bash
cd ~/ros2_ws/src
git clone https://github.com/CoreSenseEU/coresense_common
git clone https://github.com/CoreSenseEU/coresense_vampire
cd ~/ros2_ws
colcon build --packages-up-to coresense_vampire
source install/setup.bash
```

The package ships a statically linked Vampire executable for x86-64 Linux
(`tools/vampire_z3_rel_static_martin-xdb-coresense_10529`), built from the branch
[`martin-xdb-coresense`](https://github.com/vprover/vampire/commits/martin-xdb-coresense/) of Vampire,
which adds support for external sources. It is installed with the package and found at run time;
no separate installation of Vampire is needed. Higher-order reasoning is not enabled in this build.

A manifest for building the package with [Pixi](https://pixi.sh) is included, see the
[CoreSense guide on releasing packages with Pixi](https://coresenseeu.github.io/tutorials/docs/pixi_release.html).

## Running

```bash
ros2 run coresense_vampire ros_vampire.py   # the reasoning node
ros2 run coresense_vampire external.py      # an example external source, serving /external
ros2 run coresense_vampire session_test.py  # an integration test / demo client
```

## Interface

### Services

| Service | Request | Response | Purpose |
|---|---|---|---|
| `/vampire/start_session` | | `session_id` | Create a new, empty session. |
| `/vampire/add_to_session` | `session_id`, `tptp`, `formula_set_id` | `success` | Add TPTP text (any sequence of annotated formulas, also `include` directives) to a formula set; the set is created if needed. |
| `/vampire/remove_from_session` | `session_id`, `formula_set_id` | `success` | Remove a formula set with all its formulas. |
| `/vampire/list_session` | `session_id` | `success`, `formulas` | List the formulas of a session. |
| `/vampire/get_solution` | `session_id` | `success`, `solution` | Output of the last successful query on the session. |
| `/vampire/end_session` | `session_id` | `success` | Discard a session (refused while a query on it is running). |

All fields are strings, except `success` (bool) and `formulas` (string array).
The service types are `coresense_msgs/srv/{StartSession,AddToSession,RemoveFromSession,ListSession,GetSolution,EndSession}`.

### Query action

The action `/vampire/query` (type `coresense_msgs/action/QueryReasoner`) runs Vampire on the current
content of a session:

- **goal**: `session_id`; `query`, TPTP text appended after all formulas of the session (typically one
  formula with the role `conjecture` or `question`); `configuration`, command-line options passed to
  Vampire unchanged;
- **feedback**: `status`;
- **result**: `result`, the complete output of Vampire (standard output and standard error);
  `code` and `code_msg`, the status of the reasoning attempt (see below).

Goals for unknown sessions are rejected. A running query can be cancelled; Vampire is then terminated
(and killed if it does not exit within two seconds). Several queries, on the same or on different
sessions, can run in parallel, each in its own Vampire process.

Useful options for `configuration` include `-t 10` (time limit in seconds), `-s <n>` or
`--decode <strategy>` (a particular proof search strategy), and `-qa plain` (report answer bindings for
an existentially quantified conjecture). Formulas with the TPTP role `question` switch on answer
reporting automatically.

### Status codes

| `code` | Meaning |
|---|---|
| 1 | Proof found: the conjecture follows from the session (answers are in `result` if requested). |
| 2 | Saturation: the input is satisfiable, the conjecture does not follow. |
| 3 / 4 / 5 / 6 | Time / instruction / memory / activation limit reached. |
| 7 | Inappropriate strategy for the input (e.g. finite model finding with arithmetic). |
| 8 | Vampire's own "unknown" termination reason. |
| 9 | An incomplete strategy failed to resolve the problem. |
| 10 | The query was cancelled. |
| 11 | Vampire was interrupted by a termination signal (SIGINT, SIGTERM, SIGHUP, SIGXCPU). |
| 12 | Vampire was stopped by another signal (SIGABRT, SIGSEGV, ...); likely a bug. |
| 13 | Unhandled exception in Vampire, e.g. due to malformed input (see the error message in `result`). |
| 0 | None of the above could be determined. |

Only codes 1 and 2 are definite answers. Codes 3 to 9 are normal outcomes for an undecidable logic:
the question could not be decided with the given resources and options.

## External predicates

A predicate is declared external by a TPTP formula with the role `external`, followed by the name of
the ROS 2 service that answers questions about it:

```
tff(t1, type, j : $tType).
tff(t2, type, p : j > $o).
tff(a, external, ?[A:j]: p(A), "/external").
```

The formula consists of an optional block of universally quantified variables, an optional block of
existentially quantified variables, and one positive atom. An argument whose variable is existentially
quantified may be left open in a question (the source is expected to supply values for it); an argument
whose variable is universally quantified must be known, i.e., ground, when the question is asked.

During proof search, when Vampire selects a negative literal of an external predicate whose arguments
fit a declaration, it sends the corresponding atom as a question to the node (over a socket pair passed
to Vampire with the `-esfd` option). The node calls the service named in the declaration and returns the
answers to Vampire, which adds them as ground facts. Each literal is asked about at most once.

The service has the type `coresense_msgs/srv/VampireExternalPredicate`:

- **request** `parameters` (string array): the arguments of the question; open (variable) arguments are
  empty strings;
- **response** `answers` (string array): a flat list whose length is a multiple of the arity of the
  predicate; each consecutive group of arity many strings is one answer tuple.

If the service is unavailable, does not respond within two seconds, or the question cannot be parsed, the
node answers "no answers". Constants returned by a source need not be declared: Vampire assigns them the
type of the argument position in which they first occur. External predicates must have at least one
argument.

## Example

The integration test `session_test.py` (with `external.py` running) adds the three formulas above in two
formula sets, `types` (the type declarations) and `axioms` (the external declaration), and queries

```
tff(c, conjecture, ?[A:j]: p(A)).
```

with the configuration `--input_syntax tptp -updr off -s 1010 -t 1`. Vampire asks the question `p(X0)`,
the example source answers `c0`, and the query succeeds with code 1. After removing the formula set
`axioms`, the same query ends with code 9.

## Users

- [coresense_understanding](https://github.com/CoreSenseEU/coresense_understanding), the CoreSense
  Understanding System, uses the node to compute understanding strategies over the TPTP model in
  [understanding-logic](https://github.com/CoreSenseEU/understanding-logic).

## Acknowledgement

<img src="https://github.com/user-attachments/assets/b11da974-9201-4f79-902e-c9c20e8aa7a4" alt="Funded by the European Union" width="240"/>

This work has received funding from the European Union's Horizon Europe research and innovation programme under grant agreement No 101070254 ([CORESENSE](https://coresense.eu)). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Commission. Neither the European Union nor the granting authority can be held responsible for them.
