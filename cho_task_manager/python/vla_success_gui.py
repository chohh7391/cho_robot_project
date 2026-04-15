import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import tkinter as tk
import threading

class VLASuccessGUI(Node):
    def __init__(self):
        super().__init__('vla_success_gui_node')
        self.client = self.create_client(Trigger, '/vla/trigger_success')
        self.get_logger().info("VLA Success GUI Node started.")

    def trigger_success(self):
        """버튼을 눌렀을 때 실행되는 함수"""
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⚠️ '/vla/trigger_success' 서비스가 켜져있지 않습니다!")
            return

        self.get_logger().info("Sending Success Trigger...")
        req = Trigger.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self._response_callback)

    def _response_callback(self, future):
        """서비스 응답을 받았을 때 실행되는 콜백"""
        try:
            response = future.result()
            self.get_logger().info(f"✅ Trigger Response: {response.success} - {response.message}")
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")

def run_gui(ros_node):
    """Tkinter GUI 창 생성 및 실행"""
    root = tk.Tk()
    root.title("VLA Controller")
    root.geometry("350x150")  # 창 크기
    root.attributes('-topmost', True) # 창을 항상 맨 위에 띄우기 (옵션)

    # 버튼 클릭 이벤트와 ROS 2 함수 연결
    def on_click():
        ros_node.trigger_success()

    # 크고 예쁜 버튼 생성
    btn = tk.Button(
        root, 
        text="🚀 Task Success (Trigger)", 
        font=("Arial", 16, "bold"), 
        bg="#4CAF50", # 초록색 계열
        fg="white",
        activebackground="#45a049",
        command=on_click
    )
    btn.pack(expand=True, fill='both', padx=20, pady=20)

    # 창의 X 버튼을 눌러 종료할 때 안전하게 ROS 2 끄기
    def on_closing():
        root.destroy()
        rclpy.shutdown()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui_node = VLASuccessGUI()

    # ROS 2 Spin을 백그라운드 스레드에서 실행 (GUI 멈춤 방지)
    spin_thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    spin_thread.start()

    # 메인 스레드에서는 GUI 루프 실행
    run_gui(gui_node)

    # 창이 닫히면 스레드 종료 대기
    spin_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()