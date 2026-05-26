#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from coresense_msgs.srv import StartSession, AddToSession, RemoveFromSession, ListSession, GetSolution, EndSession

from rclpy.action import ActionClient
from coresense_msgs.action import QueryReasoner


class SessionTester(Node):
    def __init__(self):
        super().__init__('session_tester')

    def call_service(self, srv_type, srv_name, request):
        client = self.create_client(srv_type, srv_name)
        self.get_logger().info(f"Waiting for {srv_name}...")
        client.wait_for_service()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_reasoner_query(self, session_id, query, configuration=""):
        """
        Call the QueryReasoner action and wait for the result.
        """

        action_client = ActionClient(self, QueryReasoner, '/query_reasoner')

        self.get_logger().info("Waiting for QueryReasoner action server...")
        action_client.wait_for_server()

        goal_msg = QueryReasoner.Goal()
        goal_msg.session_id = session_id
        goal_msg.query = query
        goal_msg.configuration = configuration

        # Send the goal
        self.get_logger().info(f"Sending query: {query} under configuration: {configuration}")
        goal_future = action_client.send_goal_async(goal_msg)

        if False:
            def cancel_later():
                if goal_future.done():
                    goal_handle = goal_future.result()
                    self.get_logger().info("Sending cancel request!")
                    goal_handle.cancel_goal_async()
                else:
                    self.get_logger().warning("Goal not yet accepted, cannot cancel.")

            self.create_timer(0.5, cancel_later)

        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Reasoner rejected the goal.")
            return None

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f"Reasoner result: {result.result}")
        self.get_logger().info(f"Reasoner code: {result.code}")

        return result

    def run_get_solution(self, session_id):
        solution_req = GetSolution.Request()
        solution_req.session_id = session_id
        solution_res = self.call_service(GetSolution, '/get_solution', solution_req)

        if solution_res.success:
            self.get_logger().info(f"Get solution for session {session_id} successful and returned:\n{solution_res.solution}")
        else:
            self.get_logger().info(f"Get solution for session {session_id} failed")

    def run(self):
        # Start session
        start_req = StartSession.Request()
        start_res = self.call_service(StartSession, '/start_session', start_req)

        session_id = start_res.session_id
        self.get_logger().info(f"Session ID: {session_id}")

        # Add formulas
        for formula in [
            # "include('SET010+0.ax').",
            'tff(t1,type,j : $tType).',
            'tff(t2,type,p : j > $o).',
          # 'tff(t3,type,c0 : j).',
            'tff(a,external,?[A:j]:p(A),"/external").'
        ]:
            add_req = AddToSession.Request()
            add_req.session_id = session_id
            add_req.tptp = formula
            add_req.formula_set_id = "base"

            add_res = self.call_service(AddToSession, '/add_to_session', add_req)
            self.get_logger().info(f"Added formula '{formula}': {add_res.success}")

        # List session
        list_req = ListSession.Request()
        list_req.session_id = session_id
        list_res = self.call_service(ListSession, '/list_session', list_req)
        self.get_logger().info(f"Session contents: {list_res.formulas}")

        self.run_get_solution(session_id)

        query = "tff(c,conjecture,?[A:j]:p(A))."
        reasoner_result = self.run_reasoner_query(session_id, query, configuration="--input_syntax tptp -updr off -t 1")
        # if reasoner_result:
        #    self.get_logger().info(f"The whole reasoner_result: {reasoner_result}")

        self.run_get_solution(session_id)

    def run_two(self):
        # --------------------------------------------------------
        # Create TWO sessions
        # --------------------------------------------------------
        start1 = self.call_service(StartSession, '/start_session', StartSession.Request())
        start2 = self.call_service(StartSession, '/start_session', StartSession.Request())

        sid1 = start1.session_id
        sid2 = start2.session_id

        self.get_logger().info(f"Session 1 ID: {sid1}")
        self.get_logger().info(f"Session 2 ID: {sid2}")

        # --------------------------------------------------------
        # Fill each session with formulas
        # --------------------------------------------------------
        formulas = ["include('SET010+0.ax')."]

        for formula in formulas:
            req = AddToSession.Request()
            req.session_id = sid1
            req.formula = formula
            self.call_service(AddToSession, '/add_to_session', req)

        for formula in formulas:
            req = AddToSession.Request()
            req.session_id = sid2
            req.formula = formula
            self.call_service(AddToSession, '/add_to_session', req)

        # --------------------------------------------------------
        # Check list contents
        # --------------------------------------------------------
        def show_contents(sid):
            list_req = ListSession.Request()
            list_req.session_id = sid
            res = self.call_service(ListSession, '/list_session', list_req)
            self.get_logger().info(f"[{sid}] contents: {res.formulas}")

        show_contents(sid1)
        show_contents(sid2)

        # --------------------------------------------------------
        # RUN TWO SOLVERS IN PARALLEL
        # --------------------------------------------------------
        self.get_logger().info("Running QueryReasoner on BOTH sessions in parallel...")

        # Create two action clients (can reuse or create fresh)
        client1 = ActionClient(self, QueryReasoner, '/query_reasoner')
        client2 = ActionClient(self, QueryReasoner, '/query_reasoner')

        client1.wait_for_server()
        client2.wait_for_server()

        # Build two goals
        goal1 = QueryReasoner.Goal()
        goal1.session_id = sid1
        goal1.query = "cnf(b,negated_conjecture,a!=b)."
        goal1.configuration = "--input_syntax tptp -t 10"

        goal2 = QueryReasoner.Goal()
        goal2.session_id = sid2
        goal2.query = "cnf(b,negated_conjecture,a!=b)."
        goal2.configuration = "--input_syntax tptp -t 10"

        # Send BOTH goals (parallel)
        goal_future1 = client1.send_goal_async(goal1)
        goal_future2 = client2.send_goal_async(goal2)

        # Wait for acceptance of both goals
        rclpy.spin_until_future_complete(self, goal_future1)
        rclpy.spin_until_future_complete(self, goal_future2)

        goal_handle1 = goal_future1.result()
        goal_handle2 = goal_future2.result()

        if not goal_handle1.accepted:
            self.get_logger().error(f"Session {sid1} goal rejected.")
        if not goal_handle2.accepted:
            self.get_logger().error(f"Session {sid2} goal rejected.")

        # Get result futures (still parallel)
        result_future1 = goal_handle1.get_result_async()
        result_future2 = goal_handle2.get_result_async()

        # Now block until BOTH results are available
        rclpy.spin_until_future_complete(self, result_future1)
        rclpy.spin_until_future_complete(self, result_future2)

        result1 = result_future1.result().result
        result2 = result_future2.result().result

        # Print results
        self.get_logger().info(f"[{sid1}] QueryReasoner result: {result1.result}")
        self.get_logger().info(f"[{sid2}] QueryReasoner result: {result2.result}")

        # Also show codes if desired
        self.get_logger().info(f"[{sid1}] Return code: {result1.code}")
        self.get_logger().info(f"[{sid2}] Return code: {result2.code}")


    def run_formula_sets(self):
        # Start session
        start_res = self.call_service(StartSession, '/start_session', StartSession.Request())
        session_id = start_res.session_id
        self.get_logger().info(f"Session ID: {session_id}")

        # Add type declarations under "types" formula set
        for formula in [
            'tff(t1,type,j : $tType).',
            'tff(t2,type,p : j > $o).',
        ]:
            req = AddToSession.Request()
            req.session_id = session_id
            req.tptp = formula
            req.formula_set_id = "types"
            res = self.call_service(AddToSession, '/add_to_session', req)
            self.get_logger().info(f"Added to 'types': '{formula}': {res.success}")

        # Add external axiom under "axioms" formula set
        axiom = 'tff(a,external,?[A:j]:p(A),"/external").'
        req = AddToSession.Request()
        req.session_id = session_id
        req.tptp = axiom
        req.formula_set_id = "axioms"
        res = self.call_service(AddToSession, '/add_to_session', req)
        self.get_logger().info(f"Added to 'axioms': '{axiom}': {res.success}")

        # List session — should contain all 3 formulas
        list_req = ListSession.Request()
        list_req.session_id = session_id
        list_res = self.call_service(ListSession, '/list_session', list_req)
        self.get_logger().info(f"Session contents (all sets): {list_res.formulas}")

        # Query — should succeed with the external axiom present
        query = "tff(c,conjecture,?[A:j]:p(A))."
        config = "--input_syntax tptp -updr off -s 1010 -t 1"
        result = self.run_reasoner_query(session_id, query, configuration=config)
        assert result.code == 1, f"Expected SUCCESS_PROOF (1), got {result.code}"
        self.get_logger().info("First query succeeded as expected (with axioms).")
        self.get_logger().info(f"The code_msg was: {result.code_msg}")

        # Remove the "axioms" formula set
        rm_req = RemoveFromSession.Request()
        rm_req.session_id = session_id
        rm_req.formula_set_id = "axioms"
        rm_res = self.call_service(RemoveFromSession, '/remove_from_session', rm_req)
        self.get_logger().info(f"Removed 'axioms' formula set: {rm_res.success}")

        # List session — should only have the type declarations
        list_res = self.call_service(ListSession, '/list_session', list_req)
        self.get_logger().info(f"Session contents (after removal): {list_res.formulas}")

        # Query again — should timeout/fail since the external axiom is gone
        result = self.run_reasoner_query(session_id, query, configuration=config)
        assert result.code == 9, f"Expected INCOMPLETE_STRATEGY (9), got {result.code}"
        self.get_logger().info("Second query returned INCOMPLETE_STRATEGY as expected (axioms removed).")
        self.get_logger().info(f"The code_msg was: {result.code_msg}")

        # --- EndSession tests ---

        # Fail: try to end a non-existent session
        end_req = EndSession.Request()
        end_req.session_id = "nonexistent"
        end_res = self.call_service(EndSession, '/end_session', end_req)
        assert not end_res.success, "Expected failure when ending non-existent session"
        self.get_logger().info("EndSession correctly rejected non-existent session.")

        # Success: end the real session
        end_req.session_id = session_id
        end_res = self.call_service(EndSession, '/end_session', end_req)
        assert end_res.success, "Expected success when ending idle session"
        self.get_logger().info(f"EndSession succeeded for {session_id}.")


def main():
    rclpy.init()
    node = SessionTester()
    node.run_formula_sets()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()