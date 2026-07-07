#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import uuid
import subprocess
import time
import select
import queue, threading

from coresense_msgs.srv import StartSession, AddToSession, RemoveFromSession, ListSession, GetSolution, VampireExternalPredicate, EndSession
from coresense_msgs.action import QueryReasoner

import os, socket
from ament_index_python.packages import get_package_prefix

from enum import Enum

class Code(Enum):
    UNKNOWN              = (0, "Unknown") # If this is returned, I need to investigate the case and assign a better code to it eventually

    SUCCESS_PROOF        = (1, "Successfully shown Theorem/Unsatisfiability, proof/answer should be available in result (depending on invocation)")
    SUCCESS_SATURATION   = (2, "Successfully (finitely) saturated the input, showing Non-theoremhood/Satisfiability")

    TIME_LIMIT           = (3, "Time limit reached")
    INSTR_LIMIT          = (4, "Instruction limit reached")
    MEMORY_LIMIT         = (5, "Memory limit reached")
    ACTIVATION_LIMIT     = (6, "Activation limit reached")
    INAPPROPRIATE        = (7, "Inappropriate strategy used (e.g. finite model finding currently does not support arithmetic domains)")
    VAMPIRES_UNKNOWN     = (8, "Vampire's internal UNKNOWN value for termination reason; should be debugged in Vampire")
    INCOMPLETE_STRATEGY  = (9, "Vampire used an incomplete strategy and failed to resolve the problem")

    CANCELLED            = (10, "The ROS2 action being cancelled before finishing")

    # VAMP_RESULT_STATUS_INTERRUPTED
    INTERRUPTED          = (11, "A polite (non-crashing) interrupt sent to Vampire (SIGINT / SIGTERM / SIGHUP / SIGXCPU), who shut up and exited immediately")
    # VAMP_RESULT_STATUS_OTHER_SIGNAL
    SIGNALLED            = (12, "A crashy or impolite interrupt sent to Vampire (SIGABRT / SIGFPE / SIGILL / SIGSEGV / SIGQUIT / SIGBUS), likely means a bug.")
    # VAMP_RESULT_STATUS_UNHANDLED_EXCEPTION
    UNHANDLED_EXCEPTION  = (13, "Unhandled exception inside Vampire (could be an assertion violations in a debug build), or user error (check the error message)")

    def __init__(self, value, label):
        self._value_ = value
        self.label = label

def code_to_result(result,code):
    result.code = code.value
    result.code_msg = code.label

def resolve_success(out):
    for line in out.split("\n"):
        if line.startswith("% Termination reason:"):
            if "Refutation" in line:
                return Code.SUCCESS_PROOF
            elif "Satisfiable" in line:
                return Code.SUCCESS_SATURATION
    return Code.UNKNOWN

def resolve_failure(out):
    for line in out.split("\n"):
        if line.startswith("% Termination reason:"):
            if "Time limit" in line:
                return Code.TIME_LIMIT
            elif "Instruction limit" in line:
                return Code.INSTR_LIMIT
            elif "Memory limit" in line:
                return Code.MEMORY_LIMIT
            elif "Activation limit" in line:
                return Code.ACTIVATION_LIMIT
            elif "Inappropriate" in line:
                return Code.INAPPROPRIATE
            elif "Unknown" in line:
                return Code.VAMPIRES_UNKNOWN
            elif "Refutation not found" in line:
                return Code.INCOMPLETE_STRATEGY

    return Code.UNKNOWN


class CantParseQuestionException(Exception):
    pass

def _skip_quoted(s, i, quote_char):
    """Advance i past the closing quote_char, respecting backslash escapes.
    i should point to the opening quote. Returns index of the closing quote."""
    i += 1  # skip opening quote
    while i < len(s):
        if s[i] == '\\':
            i += 2  # skip escaped char
        elif s[i] == quote_char:
            return i  # caller will i += 1 in the main loop
        else:
            i += 1
    raise CantParseQuestionException(
        f"Unterminated {quote_char}-quoted string in: {s}")

def process_tptp_question(tptp_question):
    """
        A tptp question can look like this: p(a,X0,b), or could even contain nested subterms: q(f(a,f(b,c)),Y)
        Split it into a predicate_name and a list of arguments, where variable arguments are represented as emtpy strings

        For now, we don't allow propositions (zero arity predicates);
        (Note that confirming a proposition is true cannot be achieved as the empty-list answer means "no, nothing";
            we would need a proper list-of-list convention with the distinction between [] - for "no", and [[]] - "yes (and no args)")
    """
    i = 0
    state = 0
    depth = 0
    last_mark = None
    predname = None

    args = []
    def add_arg(arg):
        if not arg:
            raise CantParseQuestionException(f"Question {tptp_question} yields an empty argument!")
        args.append(arg)

    while i < len(tptp_question):
      if state == 0: # reading pred, waiting for "("
        if tptp_question[i] in ("'", '"'):
          i = _skip_quoted(tptp_question, i, tptp_question[i])
        elif tptp_question[i] == "(":
          predname = tptp_question[:i]
          if not len(predname):
              raise CantParseQuestionException(f"Question {tptp_question} had an empty predicate name!")
          state = 1
          last_mark = i+1
      elif state == 1:
        if tptp_question[i] in ("'", '"'):
          i = _skip_quoted(tptp_question, i, tptp_question[i])
        elif tptp_question[i] == "(":
          depth += 1
        elif tptp_question[i] == ")":
          if depth > 0:
            depth -= 1
          else:
            add_arg(tptp_question[last_mark:i])
            state = 2
        elif tptp_question[i] == "," and depth == 0:
          add_arg(tptp_question[last_mark:i])
          last_mark = i+1
      else: # state == 2
        raise CantParseQuestionException(f"Reading tptp question {tptp_question} past the closing ')'")
      i += 1

    if state != 2:
      raise CantParseQuestionException(f"Tptp question {tptp_question} not properly closed with ')'")

    # from args, replace those that start with a capital letter (i.e., the variables) with ""
    args = ["" if a and a[0].isupper() else a for a in args]
    return predname, args

def create_tptp_answer(predname, answer_args):
    return f"{predname}({",".join(answer_args)})"

def stream_reader(pipe, output_queue):
    try:
        for line in iter(pipe.readline, ''):
            output_queue.put(line)
    finally:
        pipe.close()

def drain_queue(q):
    items = []
    while True:
        try:
            items.append(q.get_nowait())
        except queue.Empty:
            break
    return items

class VampireRunner(Node):
    def __init__(self):
        super().__init__('session_manager')

        self.sessions = {}      # session_id -> {formula_set_id -> list of strings}
        self.solutions = {}     # session_id -> string
        self.active_sessions = set()       # session_ids with in-flight queries
        self.active_sessions_lock = threading.Lock()

        # Services
        self.start_srv = self.create_service(StartSession, '/vampire/start_session', self.start_session_cb)
        self.add_srv = self.create_service(AddToSession, '/vampire/add_to_session', self.add_to_session_cb)
        self.remove_srv = self.create_service(RemoveFromSession, '/vampire/remove_from_session', self.remove_from_session_cb)
        self.list_srv = self.create_service(ListSession, '/vampire/list_session', self.list_session_cb)
        self.get_sol_srv = self.create_service(GetSolution, '/vampire/get_solution', self.get_solution_cb)
        self.end_srv = self.create_service(EndSession, '/vampire/end_session', self.end_session_cb)

        # Action
        self.solve_action = ActionServer(
            self,
            QueryReasoner,
            '/vampire/query',
            execute_callback=self.execute_solve_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().info("VampireRunner node ready.")

    # ---- Services ----
    def start_session_cb(self, request, response):
        session_id = str(uuid.uuid4())
        self.sessions[session_id] = {}
        self.get_logger().info(f"Created session {session_id}")
        response.session_id = session_id
        return response

    def add_to_session_cb(self, request, response):
        sid = request.session_id
        if sid not in self.sessions:
            self.get_logger().info(f"Couldn't add to session {sid}. Session not found!")
            response.success = False
            return response
        fsid = request.formula_set_id
        self.get_logger().info(f"Adding to session {sid}, formula_set '{fsid}'.")
        self.get_logger().debug(f"Formulas:\n{request.tptp}")
        if fsid not in self.sessions[sid]:
            self.sessions[sid][fsid] = []
        self.sessions[sid][fsid].append(request.tptp)
        response.success = True
        return response

    def remove_from_session_cb(self, request, response):
        sid = request.session_id
        fsid = request.formula_set_id
        if sid not in self.sessions or fsid not in self.sessions[sid]:
            self.get_logger().info(f"Couldn't remove formula_set '{fsid}' from session {sid}. Not found!")
            response.success = False
            return response
        del self.sessions[sid][fsid]
        self.get_logger().info(f"Removed formula_set '{fsid}' from session {sid}.")
        response.success = True
        return response

    def list_session_cb(self, request, response):
        sid = request.session_id
        if sid not in self.sessions:
            self.get_logger().info(f"Couldn't list a session {sid}. Session not found!")
            response.success = False
            response.formulas = []
            return response
        self.get_logger().info(f"Listing a session {sid}.")
        response.success = True
        response.formulas = [f for fs in self.sessions[sid].values() for f in fs]
        return response

    def get_solution_cb(self, request, response):
        sid = request.session_id
        if sid not in self.solutions:
            self.get_logger().info(f"Get solution failed for session {sid}. Solution not found!")
            response.success = False
            response.solution = ""
        else:
            response.success = True
            response.solution = self.solutions[sid]
            self.get_logger().info(f"Get solution called for session {sid}.")
        return response

    def end_session_cb(self, request, response):
        sid = request.session_id
        with self.active_sessions_lock:
            if sid in self.active_sessions:
                self.get_logger().info(f"Cannot end session {sid}: query in progress")
                response.success = False
                return response
            if sid not in self.sessions:
                self.get_logger().info(f"Cannot end session {sid}: not found")
                response.success = False
                return response
            del self.sessions[sid]
        self.solutions.pop(sid, None)
        self.get_logger().info(f"Ended session {sid}")
        response.success = True
        return response

    # ---- Actions ----
    def goal_cb(self, goal_request):
        self.get_logger().info(f"Received goal for session {goal_request.session_id}")
        if goal_request.session_id not in self.sessions:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info(f"Request to cancel solve action for session {goal_handle.request.session_id}")
        return CancelResponse.ACCEPT

    def call_service(self, srv_type, srv_name, request,
                    availability_timeout=1.0,
                    response_timeout=2.0):

        # Create client
        client = self.create_client(srv_type, srv_name)

        # Check availability with timeout
        available = client.wait_for_service(timeout_sec=availability_timeout)
        if not available:
            raise RuntimeError(f"Service {srv_name} not available after {availability_timeout}s")

        # Call asynchronously
        future = client.call_async(request)

        # Wait for response (executor threads will handle callbacks)
        start = time.time()
        while not future.done():
            if time.time() - start > response_timeout:
                raise TimeoutError(f"Service {srv_name} did not respond within {response_timeout}s")
            time.sleep(0.01)   # yield without busy spinning

        return future.result()



    def execute_solve_cb(self, goal_handle):
        sid = goal_handle.request.session_id
        with self.active_sessions_lock:
            self.active_sessions.add(sid)
        try:
            return self._execute_solve_inner(goal_handle, sid)
        finally:
            with self.active_sessions_lock:
                self.active_sessions.discard(sid)

    def _execute_solve_inner(self, goal_handle, sid):
        goal_handle.publish_feedback(QueryReasoner.Feedback(status=f"Launching solver for {sid}..."))

        prefix = get_package_prefix('coresense_vampire')
        exe = os.path.join(prefix, 'lib', 'coresense_vampire', 'vampire_z3_rel_static_martin-xdb-coresense_10529')

        parent_sock, child_sock = socket.socketpair(socket.AF_UNIX)
        sock_file = parent_sock.makefile("rwb", buffering=0)

        with parent_sock, sock_file:
            # self.get_logger().info(f"child_sock.fileno() was {child_sock.fileno()}")

            solver_proc = subprocess.Popen(
                # "valgrind --leak-check=full --track-origins=yes".split()+
                [exe,"-esfd",str(child_sock.fileno())]+goal_handle.request.configuration.split(),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                pass_fds=[child_sock.fileno()],   # keep this fd open across exec
                text=True
            )

            child_sock.close()

            all_formulas = [f for fs in self.sessions[sid].values() for f in fs]
            data = "\n".join(all_formulas+[goal_handle.request.query])

            try:
                solver_proc.stdin.write(data)
                solver_proc.stdin.close()
            except Exception:
                pass

            stdout_q = queue.Queue()
            stderr_q = queue.Queue()

            stdout_thread = threading.Thread(
                target=stream_reader,
                args=(solver_proc.stdout, stdout_q),
                daemon=True
            )
            stdout_thread.start()

            stderr_thread = threading.Thread(
                target=stream_reader,
                args=(solver_proc.stderr, stderr_q),
                daemon=True
            )
            stderr_thread.start()

            # Monitor process in a loop
            while solver_proc.poll() is None:  # still running
                self.get_logger().info("Polling...")
                time.sleep(0.1)

                # Non-blocking readiness check
                readable, _, _ = select.select([parent_sock], [], [], 0)
                if readable:
                    line = sock_file.readline()
                    if line: # otherwise child closed FD ?
                        msg = line.decode().rstrip()
                        self.get_logger().info(f"Received: {msg}")

                        answers = [] # vampire is waiting; will answer "nothing" if the actual source fails to deliver
                        try:
                            service_name,tptp_question = msg.split()
                            predname, args = process_tptp_question(tptp_question)

                            try:
                                answers = self.call_service(VampireExternalPredicate, service_name, VampireExternalPredicate.Request(parameters=args)).answers
                            except Exception as e:
                                self.get_logger().warning(f"Exception during external service call: {e}")

                        except CantParseQuestionException:
                            self.get_logger().warning(f"Couldn't parse tptp question: '{tptp_question}'. Will pretend the source has no answers.")

                        try:
                            if len(answers) % len(args) != 0:
                                self.get_logger().warning(f"Number of provided answer slots does not devide question predicate's arity!")

                            num_lines = len(answers) // len(args)
                            self.get_logger().info(f"Sending back {num_lines} answer lines:")
                            while len(answers) >= len(args):
                                answer_args = answers[0:len(args)]
                                answers = answers[len(args):]

                                atom = create_tptp_answer(predname,answer_args)
                                self.get_logger().info(f"   {atom}")
                                sock_file.write(f"{atom}\n".encode())

                            sock_file.write(f"\n".encode())
                            sock_file.flush()
                        except BrokenPipeError as e:
                            # if we can't talk to it, it probably died
                            pass

                # Check for cancellation from client
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Cancel requested, stopping solver...")

                    solver_proc.terminate()

                    try:
                        solver_proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        solver_proc.kill()

                    goal_handle.canceled()
                    result = QueryReasoner.Result()

                    result.result = ""
                    code_to_result(result,Code.CANCELLED)

                    return result

            """
            solver_proc.stdin = None # so that communicate won't touch stdin
            out, err = solver_proc.communicate()
            """

            stdout_thread.join(timeout=1.0)
            stderr_thread.join(timeout=1.0)

            out = ''.join(drain_queue(stdout_q))
            err = ''.join(drain_queue(stderr_q))

            result = QueryReasoner.Result()
            result.result = f"Out:\n{out}\nErr:\n{err}"

            '''
            There is the following enum in Vampire (with the first two options worth of breaking down further
                VAMP_RESULT_STATUS_SUCCESS,
                VAMP_RESULT_STATUS_UNKNOWN,
                VAMP_RESULT_STATUS_OTHER_SIGNAL,
                VAMP_RESULT_STATUS_INTERRUPTED,
                VAMP_RESULT_STATUS_UNHANDLED_EXCEPTION
            '''

            if solver_proc.returncode == 0: # VAMP_RESULT_STATUS_SUCCESS
                code_to_result(result,resolve_success(out))
            elif solver_proc.returncode == 1: # VAMP_RESULT_STATUS_UNKNOWN
                code_to_result(result,resolve_failure(out))
            elif solver_proc.returncode == 2: # VAMP_RESULT_STATUS_OTHER_SIGNAL
                code_to_result(result,Code.INTERRUPTED)
            elif solver_proc.returncode == 3: # VAMP_RESULT_STATUS_INTERRUPTED
                code_to_result(result,Code.SIGNALLED)
            elif solver_proc.returncode == 4: # VAMP_RESULT_STATUS_UNHANDLED_EXCEPTION
                code_to_result(result,Code.UNHANDLED_EXCEPTION)
            else:
                code_to_result(result,Code.UNKNOWN)

            if solver_proc.returncode != 0:
                self.get_logger().info(f"Solver failed to resolve the query:\nOut:\n{out}\nErr:\n{err}")
            else:
                self.get_logger().info(f"Solver succeeded for {sid}")
                self.solutions[sid] = out.strip()

            goal_handle.succeed()

            return result

def main(args=None):
    rclpy.init(args=args)
    node = VampireRunner()
    rclpy.spin(node,executor = rclpy.executors.MultiThreadedExecutor())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
