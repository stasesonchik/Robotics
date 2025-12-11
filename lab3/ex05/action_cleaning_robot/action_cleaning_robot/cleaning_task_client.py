import rclpy
import sys
from rclpy.action import ActionClient
from rclpy.node import Node
from action_cleaning_robot_interface.action import CleaningTask


class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'cleaning_task')
        self._tasks = []  
        self._current_task = None

    def send_goal(self, tasks):
        self._tasks = tasks
        if not self._tasks:
            self.get_logger().error('No tasks provided!')
            return

        self._action_client.wait_for_server()
        self._send_next_goal()

    def _send_next_goal(self):
        if not self._tasks:
            self.get_logger().info('All tasks completed!')
            rclpy.shutdown()
            return

        task = self._tasks.pop(0)
        task_type, area_size, target_x, target_y = task
        self._current_task = task_type

        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = float(area_size)
        goal_msg.target_x = float(target_x)
        goal_msg.target_y = float(target_y)

        self.get_logger().info(f'Sending goal: {task_type}')
        send_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'Goal "{self._current_task}" rejected!')
            self._send_next_goal() 
            return

        self.get_logger().info(f'Goal "{self._current_task}" accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result for "{self._current_task}": {result}')
        self._send_next_goal()  

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback [{self._current_task}]: {feedback.progress_percent}% ({feedback.current_cleaned_points} points)')




def parse_args(args):
    parsed_argv = []
    try:
        if len(args) % 4 != 0:
            raise ValueError("Неверное количество аргументов. Формат: task_type area_size target_x target_y ...")

        for i in range(0, len(args), 4):
            task_type = str(args[i])
            area_size = float(args[i + 1])
            target_x = float(args[i + 2])
            target_y = float(args[i + 3])

            if task_type not in ["clean_circle", "return_home"]:
                raise ValueError(f"Неверный тип задачи: {task_type}. Допустимые: clean_circle, return_home")

            parsed_argv.append((task_type, area_size, target_x, target_y))

    except (IndexError, ValueError) as e:
        print(f"Ошибка при разборе аргументов: {e}")
        print("Формат использования:")
        print("   ros2 run action_cleaning_robot cleaning_client <task_type> <area_size> <target_x> <target_y> [ещё задачи...]")
        print("   Пример:")
        print("   ros2 run action_cleaning_robot cleaning_client clean_circle 2.0 5.0 5.0 return_home 0.0 5.5 5.5")
        parsed_argv = []

    return parsed_argv

def main(args=None):
    rclpy.init(args=args)
    client = CleaningActionClient()
    raw_args = sys.argv[1:]
    set = parse_args(raw_args)
    print(set)
    client.send_goal(set)

    rclpy.spin(client)


if __name__ == '__main__':
    main()
